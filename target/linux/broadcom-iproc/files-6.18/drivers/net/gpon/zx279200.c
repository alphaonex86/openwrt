// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CHIP */
/*
 * ZX279200: the 8-port GPON OLT PON MAC on PCIe (V-SOL V1600G). Its bring-up, stage by stage in the order the
 * unit's own firmware runs it, each stage proven by the register it leaves set. Registers are in the chip's byte
 * order (big-endian on the bus). Offsets, values and order are read from the unit and its firmware's behaviour.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "gpon_regseq.h"

#define ZX_CHIP_ID		0x00000
#define ZX_CHIP_SEL		0x00004
#define ZX_RESET		0x00008		/* bit 31 holds the core in reset */
#define ZX_STATUS		0x0000c		/* 30: pll_312 locked, 29:28: reset done */
#define ZX_RESET_KEY		0x00024
#define ZX_LINK_CFG(n)		(0x0b024 + 0x100 * (n))
#define ZX_DDR(c)		(0x2c000 + 0x2000 * (c))
#define ZX_DDR_CTRLS		2
#define ZX_LINKS		8
#define ZX_POLL_TRIES		500		/* x GPON_REGSEQ_POLL_US = 100 ms; the firmware waits 1 + 2 + 4 ms */

static void __iomem *zx_regs;		/* one chip per board: gpon_regseq's io carries no context */

static u32 zx_rd(u32 off)
{
	return ioread32be(zx_regs + off);
}

static void zx_wr(u32 off, u32 val)
{
	iowrite32be(val, zx_regs + off);
}

static void zx_delay_ms(unsigned int ms)
{
	msleep(ms);
}

static void zx_delay_us(unsigned int us)
{
	usleep_range(us, us + us / 4);
}

static const struct gpon_regseq_io zx_io = {
	.rd = zx_rd, .wr = zx_wr, .delay_ms = zx_delay_ms, .delay_us = zx_delay_us,
};

/* Core reset: each link's bit 0 cleared, the reset bit pulsed with its key until both done bits are up. */
static const struct gpon_regseq_op zx_reset_seq[] = {
	GPON_POLL(ZX_STATUS, 30, ZX_POLL_TRIES),
	GPON_FLD(ZX_LINK_CFG(0), 0, 0, 0), GPON_FLD(ZX_LINK_CFG(1), 0, 0, 0),
	GPON_FLD(ZX_LINK_CFG(2), 0, 0, 0), GPON_FLD(ZX_LINK_CFG(3), 0, 0, 0),
	GPON_FLD(ZX_LINK_CFG(4), 0, 0, 0), GPON_FLD(ZX_LINK_CFG(5), 0, 0, 0),
	GPON_FLD(ZX_LINK_CFG(6), 0, 0, 0), GPON_FLD(ZX_LINK_CFG(7), 0, 0, 0),
	GPON_WR(ZX_RESET, 0x80000000),
	GPON_WR(ZX_RESET_KEY, 0x22a722a7),
	GPON_POLL(ZX_STATUS, 28, ZX_POLL_TRIES),
	GPON_POLL(ZX_STATUS, 29, ZX_POLL_TRIES),
	GPON_WR(ZX_RESET, 0),
	GPON_WR(ZX_CHIP_SEL, 0),
};

/* One DDR controller, relative to ZX_DDR(c): timings, start, init done (31:30). */
static const struct gpon_regseq_op zx_ddr_start[] = {
	GPON_WR(0x140, 0x80000000), GPON_WR(0x410, 0x00000803), GPON_WR(0x430, 0x00080404),
	GPON_WR(0x440, 0x20000000), GPON_WR(0x450, 0x00000020), GPON_WR(0x530, 0x80808080),
	GPON_WR(0x540, 0x80808080), GPON_WR(0x500, 0xa5000002), GPON_WR(0x510, 0x0000a500),
	GPON_WR(0x010, 0x28b0b000), GPON_WR(0x020, 0x08080a28), GPON_WR(0x030, 0x000fa123),
	GPON_WR(0x040, 0x02030305), GPON_WR(0x050, 0x0305360f), GPON_WR(0x060, 0x05050c05),
	GPON_WR(0x070, 0x10040005), GPON_WR(0x080, 0x01000039), GPON_WR(0x090, 0x00030602),
	GPON_WR(0x0a0, 0x00000010), GPON_WR(0x0b0, 0x1b600046), GPON_WR(0x0c0, 0x01000404),
	GPON_WR(0x0d0, 0x02080239), GPON_WR(0x0e0, 0x00060b08), GPON_WR(0x140, 0x80000001),
	GPON_FLD(0x000, 0, 0, 1),
	GPON_DLY(1),
	GPON_POLL(0x1b0, 30, ZX_POLL_TRIES),
	GPON_POLL(0x1b0, 31, ZX_POLL_TRIES),
};

/* Read-data training, done in 0xc00 bit 0; its results land in 0xcc0..0xdf0. */
static const struct gpon_regseq_op zx_ddr_train[] = {
	GPON_WR(0x0f0, 1), GPON_WR(0x230, 0x00001000), GPON_WR(0x210, 0x0f000000),
	GPON_WR(0x200, 0x0000f000), GPON_WR(0x200, 0x0000f001), GPON_WR(0x200, 0x0000f000),
	GPON_POLL(0xc00, 0, ZX_POLL_TRIES),
	GPON_WR(0x230, 0), GPON_WR(0x210, 0), GPON_WR(0x240, 0), GPON_WR(0x200, 0), GPON_WR(0x0f0, 0),
};

/* The trained delays take effect: 0x450 bit 9 set, 0x410 bit 9 pulsed. */
static const struct gpon_regseq_op zx_ddr_apply[] = {
	GPON_FLD(0x450, 9, 9, 1), GPON_FLD(0x410, 9, 9, 1), GPON_FLD(0x410, 9, 9, 0),
};

/* Calibration, done in 0x800 bit 0, failed if bit 9. */
static const struct gpon_regseq_op zx_ddr_cal[] = {
	GPON_WR(0x0f0, 2), GPON_WR(0x210, 0), GPON_WR(0x220, 0x00001000),
	GPON_WR(0x200, 1), GPON_WR(0x200, 0),
	GPON_POLL(0x800, 0, ZX_POLL_TRIES),
	GPON_WR(0x210, 0), GPON_WR(0x220, 0), GPON_WR(0x200, 0), GPON_WR(0x0f0, 0),
};

static u8 zx_lane(const u32 *pair, unsigned int lane)
{
	return pair[lane / 4] >> (8 * (lane % 4));
}

/* The 8 lanes' delays from the training results (0xcc0..0xdf0) to 0x460..0x4d0, as the unit's firmware sets them:
 * a lane whose window (0xd00) plus half its span (0xdc0) passes 0x1fe starts one step earlier; lane 7 repeats lane 6
 * in the first and second word. The sums are 32-bit adds, as there. */
static void zx_ddr_center(u32 base, u32 res[12])
{
	static const u16 at[12] = { 0xcc0, 0xcd0, 0xd00, 0xd10, 0xd40, 0xd50, 0xd80, 0xd90,
				    0xdc0, 0xdd0, 0xde0, 0xdf0 };
	u32 out[8] = { 0 }, start[8];
	unsigned int i;

	for (i = 0; i < 12; i++)
		res[i] = zx_rd(base + at[i]);
	for (i = 0; i < 8; i++) {
		u32 span = (res[8 + i / 2] >> (16 * (i % 2))) & 0xffff;

		start[i] = zx_lane(&res[0], i);
		if (zx_lane(&res[2], i) + (span >> 1) > 0x1fe)
			start[i]--;
	}
	for (i = 0; i < 4; i++) {
		out[0] += start[i] << (8 * i);
		out[1] += start[i + 4] << (8 * i);
		out[2] += (u32)zx_lane(&res[2], i) << (8 * i);
		out[3] += (u32)zx_lane(&res[2], i + 4) << (8 * i);
		out[4] += (u32)zx_lane(&res[4], i) << (8 * i);
		out[5] += (u32)zx_lane(&res[4], i + 4) << (8 * i);
		out[6] += (u32)zx_lane(&res[6], i) << (8 * i);
		out[7] += (u32)zx_lane(&res[6], i + 4) << (8 * i);
	}
	out[1] = (out[1] & 0x00ffffff) + (start[6] << 24);
	out[3] = (out[3] & 0x00ffffff) + ((u32)zx_lane(&res[2], 6) << 24);
	for (i = 0; i < 8; i++)
		zx_wr(base + 0x460 + 0x10 * i, out[i]);
}

/* gpon_regseq_run on a table whose addresses are relative to @base. */
static int zx_run_at(u32 base, const struct gpon_regseq_op *seq, unsigned int n)
{
	struct gpon_regseq_op op;
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		op = seq[i];
		if (op.opc != GPON_REGSEQ_DLY)
			op.addr += base;
		ret = gpon_regseq_run(&zx_io, &op, 1);
		if (ret)
			return ret;
	}
	return 0;
}

static int zx_ddr_init(struct device *dev, unsigned int c)
{
	u32 base = ZX_DDR(c), res[12];
	int ret;

	ret = zx_run_at(base, zx_ddr_start, ARRAY_SIZE(zx_ddr_start));
	if (ret) {
		dev_err(dev, "ddr%u: init not done: 0x1b0=0x%08x (%d)\n", c, zx_rd(base + 0x1b0), ret);
		return ret;
	}
	ret = zx_run_at(base, zx_ddr_train, ARRAY_SIZE(zx_ddr_train));
	if (ret) {
		dev_err(dev, "ddr%u: training not done: 0xc00=0x%08x (%d)\n", c, zx_rd(base + 0xc00), ret);
		return ret;
	}
	zx_ddr_center(base, res);
	ret = zx_run_at(base, zx_ddr_apply, ARRAY_SIZE(zx_ddr_apply));
	if (!ret)
		ret = zx_run_at(base, zx_ddr_cal, ARRAY_SIZE(zx_ddr_cal));
	if (!ret && (zx_rd(base + 0x800) & BIT(9)))
		ret = -EIO;
	if (ret) {
		dev_err(dev, "ddr%u: calibration failed: 0x800=0x%08x (%d)\n", c, zx_rd(base + 0x800), ret);
		return ret;
	}
	dev_info(dev, "ddr%u: init 0x%08x train 0x%08x cal 0x%08x, lanes %*ph\n", c, zx_rd(base + 0x1b0),
		 zx_rd(base + 0xc00), zx_rd(base + 0x800), (int)sizeof(res), res);
	return 0;
}

static int zx_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	unsigned int c;
	int ret;

	if (zx_regs)
		return dev_err_probe(dev, -EBUSY, "a second ZX279200: one chip per board\n");
	ret = pcim_enable_device(pdev);
	if (ret)
		return dev_err_probe(dev, ret, "enable\n");
	zx_regs = pcim_iomap(pdev, 0, 0);
	if (!zx_regs)
		return dev_err_probe(dev, -ENOMEM, "BAR0 not mapped\n");
	dev_info(dev, "chip 0x%08x status 0x%08x\n", zx_rd(ZX_CHIP_ID), zx_rd(ZX_STATUS));

	ret = gpon_regseq_run(&zx_io, zx_reset_seq, ARRAY_SIZE(zx_reset_seq));
	if (ret) {
		dev_err(dev, "reset: status 0x%08x (%d)\n", zx_rd(ZX_STATUS), ret);
		goto fail;
	}
	for (c = 0; c < ZX_DDR_CTRLS; c++) {
		ret = zx_ddr_init(dev, c);
		if (ret)
			goto fail;
	}
	dev_info(dev, "stage ddr: done\n");
	return 0;
fail:
	zx_regs = NULL;
	return ret;
}

static void zx_remove(struct pci_dev *pdev)
{
	zx_regs = NULL;
}

static const struct pci_device_id zx_ids[] = {
	{ PCI_DEVICE(0x1556, 0x5555) },
	{ }
};
MODULE_DEVICE_TABLE(pci, zx_ids);

static struct pci_driver zx_driver = {
	.name = "zx279200",
	.id_table = zx_ids,
	.probe = zx_probe,
	.remove = zx_remove,
};
module_pci_driver(zx_driver);

MODULE_DESCRIPTION("ZX279200 GPON OLT PON MAC");
MODULE_LICENSE("GPL");
