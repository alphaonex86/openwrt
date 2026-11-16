// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: VENDOR */
/*
 * The CMIC of a Broadcom XGS switch core, the host's window on the switch: its S-Channel carries one register or
 * memory entry per message, its MIIM master reaches the switch's PHYs and SerDes. bcm_cmic_run() replays a chip's
 * init table through them, step by step, and stops at the first step the hardware refuses.
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

static int cmic_step(void __iomem *cmic, const struct bcm_cmic_table *t, const struct bcm_cmic_step *s)
{
	int ret = 0;

	switch (s->op) {
	case BCM_CMIC_WR:
		writel(s->val, cmic + s->addr);
		return 0;
	case BCM_CMIC_REG:
		return cmic_schan(cmic, SCHAN_WRITE_REG, s->blk, s->addr, s->words == 1 ? &s->val : t->words + s->val,
				  s->words, 4 * s->words, NULL);
	case BCM_CMIC_MEM:
		for (u32 i = 0; i < s->count && !ret; i++)
			ret = cmic_schan(cmic, SCHAN_WRITE_MEM, s->blk, s->addr + i, t->words + s->val + i * s->words,
					 s->words, 4 * s->words, NULL);
		return ret;
	case BCM_CMIC_POLL:
		return cmic_poll(cmic, s->blk, s->addr, s->val);
	case BCM_CMIC_MIIM:
		return cmic_miim_write(cmic, s->addr, s->val);
	}
	return -EINVAL;
}

/* -> 0, or why the table is not one bcm_cmic_run() can replay. */
static int cmic_table_check(const struct bcm_cmic_table *t, resource_size_t size, unsigned int *bad)
{
	for (*bad = 0; *bad < t->n_steps; (*bad)++) {
		const struct bcm_cmic_step *s = &t->steps[*bad];
		bool inline_val = s->op != BCM_CMIC_MEM && !(s->op == BCM_CMIC_REG && s->words > 1);

		if (s->op > BCM_CMIC_MIIM || !s->words || s->words > CMIC_SCHAN_DATA_WORDS)
			return -EINVAL;
		if (s->op == BCM_CMIC_WR && (s->addr & 3 || (u64)s->addr + 4 > size))
			return -ERANGE;
		if (!inline_val && (u64)s->val + (u64)s->count * s->words > t->n_words)
			return -ERANGE;
	}
	return 0;
}

int bcm_cmic_run(struct device *dev, void __iomem *cmic, resource_size_t size, const struct bcm_cmic_table *t)
{
	unsigned int i;
	int ret = cmic_table_check(t, size, &i);

	if (ret) {
		dev_err(dev, "init table step %u is malformed: %d\n", i, ret);
		return ret;
	}
	for (i = 0; i < t->n_steps; i++) {
		const struct bcm_cmic_step *s = &t->steps[i];

		ret = cmic_step(cmic, t, s);
		if (ret) {
			dev_err(dev, "init step %u/%u (op %u blk %u addr 0x%08x val 0x%08x) failed: %d\n",
				i, t->n_steps, s->op, s->blk, s->addr, s->val, ret);
			return ret;
		}
	}
	dev_info(dev, "switch init: %u steps replayed\n", t->n_steps);
	return 0;
}
EXPORT_SYMBOL_GPL(bcm_cmic_run);

MODULE_DESCRIPTION("Broadcom XGS switch CMIC: S-Channel, MIIM and init table replay");
MODULE_LICENSE("GPL");
