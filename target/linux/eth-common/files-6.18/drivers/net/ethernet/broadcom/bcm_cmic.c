// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: VENDOR */
/*
 * The CMIC of a Broadcom XGS switch core, the host's window on the switch: its S-Channel carries one register or
 * memory entry per message, its MIIM master reaches the switch's PHYs and SerDes. A chip's init calls the accessors
 * below in order; the first operation the hardware refuses stops the rest.
 */

#include <linux/bits.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>

#include "bcm_cmic.h"

#define CMIC_SCHAN_CTRL		0x31000
#define CMIC_SCHAN_START	BIT(0)
#define CMIC_SCHAN_DONE		BIT(1)
#define CMIC_SCHAN_ERR		(BIT(20) | BIT(21))	/* SER check failed, not acknowledged */
#define CMIC_SCHAN_MSG		0x3100c
#define CMIC_SCHAN_DATA_WORDS	20
#define CMIC_MIIM_PARAM		0x31080
#define CMIC_MIIM_READ_DATA	0x31084
#define CMIC_MIIM_ADDRESS	0x31088
#define CMIC_MIIM_CTRL		0x3108c
#define CMIC_MIIM_WRITE		BIT(0)
#define CMIC_MIIM_READ		BIT(1)
#define CMIC_MIIM_STAT		0x31090
#define CMIC_MIIM_DONE		BIT(0)
#define CMIC_WAIT_US		10000
#define CMIC_POLL_US		100000

enum { SCHAN_WRITE_MEM = 9, SCHAN_READ_REG = 11, SCHAN_READ_REG_ACK = 12, SCHAN_WRITE_REG = 13 };

#define MII_BMCR_RESET		BIT(15)

/* One S-Channel message: `len` bytes of `data` to (`blk`, `addr`); a read's first reply word in *reply. */
static int cmic_schan(void __iomem *cmic, u32 op, u32 blk, u32 addr, const u32 *data, u32 words, u32 len,
		      u32 *reply)
{
	u32 ctrl;
	int ret;

	if (words > CMIC_SCHAN_DATA_WORDS)
		return -E2BIG;
	writel(op << 26 | blk << 20 | len << 7, cmic + CMIC_SCHAN_MSG);
	writel(addr, cmic + CMIC_SCHAN_MSG + 4);
	for (u32 i = 0; i < words; i++)
		writel(data[i], cmic + CMIC_SCHAN_MSG + 8 + 4 * i);
	writel(CMIC_SCHAN_START, cmic + CMIC_SCHAN_CTRL);
	ret = readl_poll_timeout_atomic(cmic + CMIC_SCHAN_CTRL, ctrl, ctrl & CMIC_SCHAN_DONE, 1, CMIC_WAIT_US);
	if (!ret && (ctrl & CMIC_SCHAN_ERR))
		ret = -EIO;
	if (!ret && reply) {
		if (readl(cmic + CMIC_SCHAN_MSG) >> 26 != SCHAN_READ_REG_ACK)
			ret = -EPROTO;
		*reply = readl(cmic + CMIC_SCHAN_MSG + 4);
	}
	writel(0, cmic + CMIC_SCHAN_CTRL);
	return ret;
}

static int cmic_miim(void __iomem *cmic, u32 phy_reg, u32 cmd, u32 val, u32 *reply)
{
	u32 stat;
	int ret;

	writel((phy_reg >> 16) << 16 | (val & 0xffff), cmic + CMIC_MIIM_PARAM);
	writel(phy_reg & 0xffff, cmic + CMIC_MIIM_ADDRESS);
	writel(cmd, cmic + CMIC_MIIM_CTRL);
	ret = readl_poll_timeout_atomic(cmic + CMIC_MIIM_STAT, stat, stat & CMIC_MIIM_DONE, 1, CMIC_WAIT_US);
	if (!ret && reply)
		*reply = readl(cmic + CMIC_MIIM_READ_DATA) & 0xffff;
	writel(0, cmic + CMIC_MIIM_CTRL);
	return ret;
}

/* A clause 22 control write that resets the PHY: done when the reset bit has cleared itself. */
static int cmic_miim_write(void __iomem *cmic, u32 phy_reg, u32 val)
{
	u32 bmcr;
	int ret = cmic_miim(cmic, phy_reg, CMIC_MIIM_WRITE, val, NULL);

	if (ret || (phy_reg & 0xffff) != 0 || !(val & MII_BMCR_RESET))
		return ret;
	return read_poll_timeout(cmic_miim, ret, ret || !(bmcr & MII_BMCR_RESET), 100, CMIC_POLL_US, false,
				 cmic, phy_reg, CMIC_MIIM_READ, 0, &bmcr);
}

static int cmic_poll(void __iomem *cmic, u32 blk, u32 addr, u32 mask)
{
	u32 val = 0;
	int ret, err;

	ret = read_poll_timeout(cmic_schan, err, err || (val & mask) == mask, 10, CMIC_POLL_US, false,
				cmic, SCHAN_READ_REG, blk, addr, NULL, 0, 4, &val);
	return ret ?: err;
}

/* Counts @ret as the operation just done; the first failure is logged and kept. */
static void cmic_account(struct bcm_cmic *c, int ret, const char *what, u32 blk, u32 addr)
{
	if (ret) {
		c->err = ret;
		dev_err(c->dev, "init op %u (%s blk %u addr 0x%08x) failed: %d\n", c->ops, what, blk, addr, ret);
	}
	c->ops++;
}

void bcm_cmic_write(struct bcm_cmic *c, u32 off, u32 val)
{
	if (c->err)
		return;
	if (off & 3 || (u64)off + 4 > c->size) {
		cmic_account(c, -ERANGE, "write", 0, off);
		return;
	}
	writel(val, c->base + off);
	cmic_account(c, 0, "write", 0, off);
}
EXPORT_SYMBOL_GPL(bcm_cmic_write);

static void cmic_schan_write(struct bcm_cmic *c, u32 op, u8 blk, u32 addr, const u32 *data, unsigned int words)
{
	if (!c->err)
		cmic_account(c, cmic_schan(c->base, op, blk, addr, data, words, 4 * words, NULL),
			     op == SCHAN_WRITE_MEM ? "mem" : "reg", blk, addr);
}

void bcm_cmic_reg(struct bcm_cmic *c, u8 blk, u32 addr, u32 val)
{
	cmic_schan_write(c, SCHAN_WRITE_REG, blk, addr, &val, 1);
}
EXPORT_SYMBOL_GPL(bcm_cmic_reg);

void bcm_cmic_reg64(struct bcm_cmic *c, u8 blk, u32 addr, u64 val)
{
	u32 data[2] = { lower_32_bits(val), upper_32_bits(val) };

	cmic_schan_write(c, SCHAN_WRITE_REG, blk, addr, data, 2);
}
EXPORT_SYMBOL_GPL(bcm_cmic_reg64);

void bcm_cmic_mem(struct bcm_cmic *c, u8 blk, u32 addr, const u32 *entry, unsigned int words)
{
	cmic_schan_write(c, SCHAN_WRITE_MEM, blk, addr, entry, words);
}
EXPORT_SYMBOL_GPL(bcm_cmic_mem);

void bcm_cmic_mem32(struct bcm_cmic *c, u8 blk, u32 addr, u32 val)
{
	cmic_schan_write(c, SCHAN_WRITE_MEM, blk, addr, &val, 1);
}
EXPORT_SYMBOL_GPL(bcm_cmic_mem32);

void bcm_cmic_poll(struct bcm_cmic *c, u8 blk, u32 addr, u32 mask)
{
	if (!c->err)
		cmic_account(c, cmic_poll(c->base, blk, addr, mask), "poll", blk, addr);
}
EXPORT_SYMBOL_GPL(bcm_cmic_poll);

void bcm_cmic_miim(struct bcm_cmic *c, u16 phy, u8 reg, u16 val)
{
	if (!c->err)
		cmic_account(c, cmic_miim_write(c->base, (u32)phy << 16 | reg, val), "miim", phy, reg);
}
EXPORT_SYMBOL_GPL(bcm_cmic_miim);

void bcm_cmic_miim_seq(struct bcm_cmic *c, u16 phy, const struct bcm_cmic_mii_wr *seq, unsigned int n)
{
	for (unsigned int i = 0; i < n; i++)
		bcm_cmic_miim(c, phy, seq[i].reg, seq[i].val);
}
EXPORT_SYMBOL_GPL(bcm_cmic_miim_seq);

int bcm_cmic_done(struct bcm_cmic *c)
{
	if (!c->err)
		dev_info(c->dev, "switch init: %u operations\n", c->ops);
	return c->err;
}
EXPORT_SYMBOL_GPL(bcm_cmic_done);

MODULE_DESCRIPTION("Broadcom XGS switch CMIC: S-Channel and MIIM accessors");
MODULE_LICENSE("GPL");
