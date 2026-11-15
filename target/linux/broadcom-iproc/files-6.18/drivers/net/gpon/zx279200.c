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

#include "gpon_olt_mac.h"
#include "gpon_regseq.h"
#include "gpon_olt_zte.h"

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

/* PON link n's MAC window (gpptest 2.3.1R ZX2763_*); the port below drives link 0. */
#define ZX_W(n)			(0x14000 + ((n) >> 1) * 0x4000 + ((n) & 1) * 0x1800)
#define ZX_MAP_T_RANGING	3		/* "SnMem": the ranging map */
#define ZX_MAP_T_CFG		4
#define ZX_MAP_SN_ENTRY		1150		/* bit 3: serial-number acquisition open (stock in its VM) */
#define ZX_MAP_SN_OPEN		BIT(3)
#define ZX_DS_CTRL		0x1000		/* in a link's window: bit 1 its downstream sender on */
#define ZX_DS_ON		BIT(1)
#define ZX_ONUS			8
#define ZX_PORT_LINKS		1		/* link 0 only until the others are brought up */

struct zx_chip {
	struct gpon_olt_mac mac;
};

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

static int zx_ddr_init(struct device *dev, unsigned int c)
{
	u32 base = ZX_DDR(c), res[12];
	int ret;

	ret = gpon_regseq_run_at(&zx_io, base, zx_ddr_start, ARRAY_SIZE(zx_ddr_start));
	if (ret) {
		dev_err(dev, "ddr%u: init not done: 0x1b0=0x%08x (%d)\n", c, zx_rd(base + 0x1b0), ret);
		return ret;
	}
	ret = gpon_regseq_run_at(&zx_io, base, zx_ddr_train, ARRAY_SIZE(zx_ddr_train));
	if (ret) {
		dev_err(dev, "ddr%u: training not done: 0xc00=0x%08x (%d)\n", c, zx_rd(base + 0xc00), ret);
		return ret;
	}
	zx_ddr_center(base, res);
	ret = gpon_regseq_run_at(&zx_io, base, zx_ddr_apply, ARRAY_SIZE(zx_ddr_apply));
	if (!ret)
		ret = gpon_regseq_run_at(&zx_io, base, zx_ddr_cal, ARRAY_SIZE(zx_ddr_cal));
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

/* ---- the PON links: the ZTE PON MAC block (zte_ponmac) under the common OLT PON MAC shell (gpon_olt_mac) ------ */

static u32 zx_pm_rd(void *ctx, u32 off)
{
	return zx_rd(off);
}

static void zx_pm_wr(void *ctx, u32 off, u32 val)
{
	zx_wr(off, val);
}

static const struct zte_ponmac_layout zx_layout = {
	.ploamu_msg = 0x898, .ploamu_rtd = 0x8a8, .ploamu_rtd_mask = 0xffff,	/* the round trip: INFERRED */
	.ploamu_held = 0x8ac, .ploamd_ctrl = 0x1280, .ploamd_msg = 0x1284,
	.map_cmd = 0x1814, .map_read_bit = 27, .map_table_shift = 22, .ind_ready_first = true,
};

static const u32 zx_window[ZX_LINKS] = {
	ZX_W(0), ZX_W(1), ZX_W(2), ZX_W(3), ZX_W(4), ZX_W(5), ZX_W(6), ZX_W(7),
};

static const struct zte_ponmac zx_pm = {
	.lay = &zx_layout, .rd = zx_pm_rd, .wr = zx_pm_wr, .window = zx_window,
};

static int zx_ploam_tx(void *chip, unsigned int link, const u8 m[GPON_OLT_PLOAM_DS_LEN])
{
	return zte_ponmac_ploam_tx(&zx_pm, link, m);
}

static bool zx_ploam_rx(void *chip, unsigned int link, u8 m[GPON_OLT_PLOAM_US_LEN], u32 *rtd)
{
	return zte_ponmac_ploam_rx(&zx_pm, link, m, rtd);
}

static int zx_sn_window(void *chip, unsigned int link, bool open)
{
	u32 cfg;
	int ret = zte_ponmac_map_read(&zx_pm, link, ZX_MAP_T_CFG, ZX_MAP_SN_ENTRY, &cfg, 1);

	if (ret)
		return ret;
	cfg = open ? cfg | ZX_MAP_SN_OPEN : cfg & ~ZX_MAP_SN_OPEN;
	return zte_ponmac_map_write(&zx_pm, link, ZX_MAP_T_CFG, ZX_MAP_SN_ENTRY, &cfg, 1);
}

/* A ranging map entry for the ONU's default Alloc-ID (its ONU-ID) with PLOAMu. INFERRED: start/stop left 0. */
static int zx_ranging_grant(void *chip, unsigned int link, u8 onu_id)
{
	u32 w[2] = { 0, (u32)onu_id << 12 | BIT(10) };

	return zte_ponmac_map_write(&zx_pm, link, ZX_MAP_T_RANGING, 0, w, 2);
}

/* No access to the chip after this: at reboot the host bridge resets the link next (an access then aborts). */
static void zx_stop(void *chip)
{
	unsigned int n;

	for (n = 0; n < ZX_PORT_LINKS; n++)
		zx_wr(ZX_W(n) + ZX_DS_CTRL, zx_rd(ZX_W(n) + ZX_DS_CTRL) & ~ZX_DS_ON);
}

static const struct gpon_olt_mac_ops zx_ops = {
	.ploam_tx = zx_ploam_tx, .sn_window = zx_sn_window, .ranging_grant = zx_ranging_grant,
	.ploam_rx = zx_ploam_rx, .stop = zx_stop,	/* no OMCI path yet: the CPU frame pipe is a later stage */
};

static int zx_pon_start(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpon_olt_profile tmpl;
	struct zx_chip *z;
	unsigned int n;
	int ret;

	z = devm_kzalloc(dev, sizeof(*z), GFP_KERNEL);
	if (!z)
		return -ENOMEM;
	/* Each link's downstream sender, off after the reset: until it is on, its PLOAMd takes 14 messages, then holds
	 * the start bit for ever (unit, 2026-11-15; stock sets the bit last before its first PLOAM). */
	for (n = 0; n < ZX_PORT_LINKS; n++)
		zx_wr(ZX_W(n) + ZX_DS_CTRL, zx_rd(ZX_W(n) + ZX_DS_CTRL) | ZX_DS_ON);
	gpon_olt_profile_default(&tmpl);
	tmpl.alloc_id = 256;
	tmpl.gem_port = 1024;
	ret = gpon_olt_mac_start(&z->mac, dev, "zx279200", &zx_ops, z, ZX_PORT_LINKS, ZX_ONUS, &tmpl);
	if (ret)
		return ret;
	pci_set_drvdata(pdev, z);
	dev_info(dev, "OLT PON port on link 0, %u ONUs\n", ZX_ONUS);
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
	ret = zx_pon_start(pdev);
	if (ret)
		goto fail;
	return 0;
fail:
	zx_regs = NULL;
	return ret;
}

static void zx_remove(struct pci_dev *pdev)
{
	struct zx_chip *z = pci_get_drvdata(pdev);

	if (z)
		gpon_olt_mac_stop(&z->mac);
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
	.shutdown = zx_remove,
};
module_pci_driver(zx_driver);

MODULE_DESCRIPTION("ZX279200 GPON OLT PON MAC");
MODULE_LICENSE("GPL");
