// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CHIP */
/*
 * ZX279200: the 8-port GPON OLT PON MAC on PCIe (V-SOL V1600G). Its bring-up, stage by stage in the order the
 * unit's own firmware runs it, each stage proven by the register it leaves set. Registers are in the chip's byte
 * order (big-endian on the bus). Offsets, values and order are read from the unit and its firmware's behaviour.
 */

#include <linux/unaligned.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "gpon_olt_mac.h"
#include "gpon_olt_pon.h"
#include "gpon_regseq.h"
#include "gpon_olt_zte.h"
#include "eth_rx_geom.h"
#include "zx279200_mac_tm.h"

#define ZX_CHIP_ID		0x00000
#define ZX_CHIP_SEL		0x00004
#define ZX_RESET		0x00008		/* bit 31 holds the core in reset */
#define ZX_STATUS		0x0000c		/* 30: pll_312 locked, 29:28: reset done */
#define ZX_RESET_KEY		0x00024
#define ZX_DDR(c)		(0x2c000 + 0x2000 * (c))
#define ZX_DDR_CTRLS		2
#define ZX_LINKS		8

#define ZX_MAP_T_RANGING	3		/* "SnMem": the ranging map */
#define ZX_MAP_T_CFG		4
#define ZX_MAP_SN_ENTRY		1150		/* bit 3: serial-number acquisition open (stock in its VM) */
#define ZX_MAP_SN_OPEN		BIT(3)
#define ZX_XAUI_STA		0x0005c		/* 19:16 XAUI lanes 3..0 in sync: 0 with the lanes held in reset (unit, 2026-11-16) */
#define ZX_XAUI_LANES		0x000f0000
/* The CPU frame pipe (the unit's firmware): two rings of {status, address} descriptors, bit 31 the chip's. */
#define ZX_BR_INT_EN		0x00010
#define ZX_TX_BASE		0x10400
#define ZX_TX_CFG		0x10404
#define ZX_RX_BASE		0x10408
#define ZX_RX_CFG		0x1040c
#define ZX_DMA_CTRL		0x10410
#define ZX_INT_MASK		0x10418
#define ZX_RING			255
#define ZX_RING_CFG		0x1ff		/* the firmware's: 8 set, 7:0 the ring length */
#define ZX_DMA_CTRL_ON		0x08000030	/* the firmware's */
#define ZX_DESC_OWN		BIT(31)
#define ZX_RXD_LEN_MASK		0xffff
#define ZX_RX_BUF		2048
#define ZX_TX_BUF		64
#define ZX_HDR			8		/* 0x30, 0, chip, link, 0, 0, GEM port */
#define ZX_HDR_OMCI		0x30
#define ZX_ONUS			8
#define ZX_PORT_LINKS		1		/* link 0 only until the others are brought up */

/* An upstream frame: the chip's 8-byte header, then the payload; the length counts both and no FCS (the firmware
 * copies header + 48 bytes of an OMCI frame of that length). */
static const struct eth_rx_layout zx_rx_layout = {
	.prefix = 0, .head = ZX_HDR, .fcs = ETH_RX_FCS_NOT_IN_LEN, .buf = ZX_RX_BUF,
};

struct zx_desc {
	__be32 status, addr;
};

struct zx_chip {
	struct pci_dev *pdev;
	struct gpon_olt_mac mac;
	struct zx_desc *tx, *rx;
	dma_addr_t tx_dma, rx_dma, txb_dma, rxb_dma;
	u8 *txb, *rxb;
	unsigned int tx_cur, rx_cur;
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

/* The allocator timer restart of stock's link DBA start, relative to the link's window (DBA_CPU_CTRL, DBA_TMRCTL). */
static const struct gpon_regseq_op zx_dba_restart[] = {
	GPON_WR(0x1b14, 0),
	GPON_WR(0x1914, 0),
	GPON_WR(0x1914, 1),
};

/* Stock's CbaModeSet: CBA mode on, the map switch bit written back as read, bit 16 written 0. */
static void zx_cba_on(u32 w)
{
	zx_wr(w + ZX_DBA_CBAC, ZX_CBA_ON | (zx_rd(w + ZX_DBA_CBAC) & ZX_CBA_SWITCH));
}

/* Stock's "LINK %d DBA start" (2.1.9R in its VM, 2026-11-18): the rest of it is in zx_mac_tm_init. */
static int zx_dba_start(void *chip, unsigned int link)
{
	u32 w = ZX_W(link);
	int ret;

	if (!(zx_rd(w + ZX_DBA_CBAC) & ZX_CBA_READY))
		return -EIO;
	zx_cba_on(w);
	ret = gpon_regseq_run_at(&zx_io, w, zx_dba_restart, ARRAY_SIZE(zx_dba_restart));
	if (ret)
		return ret;
	zx_cba_on(w);
	return 0;
}

/* A ranging map entry for the ONU's default Alloc-ID (its ONU-ID) with PLOAMu. INFERRED: start/stop left 0. */
static int zx_ranging_grant(void *chip, unsigned int link, u8 onu_id)
{
	u32 w[2] = { 0, (u32)onu_id << 12 | BIT(10) };

	return zte_ponmac_map_write(&zx_pm, link, ZX_MAP_T_RANGING, 0, w, 2);
}

static int zx_omci_tx(void *chip, unsigned int link, u16 gem_port, const u8 *pdu, unsigned int len)
{
	struct zx_chip *z = chip;
	struct zx_desc *d = &z->tx[z->tx_cur];
	u8 *buf = z->txb + ZX_TX_BUF * z->tx_cur;

	if (len > ZX_TX_BUF - ZX_HDR)
		return -EMSGSIZE;
	if (be32_to_cpu(READ_ONCE(d->status)) & ZX_DESC_OWN)
		return -EBUSY;
	memset(buf, 0, ZX_HDR);
	buf[0] = ZX_HDR_OMCI;
	buf[3] = link;
	put_unaligned_be16(gem_port, buf + 6);
	memcpy(buf + ZX_HDR, pdu, len);
	dma_wmb();
	WRITE_ONCE(d->status, cpu_to_be32(ZX_DESC_OWN | (ZX_HDR + len)));
	z->tx_cur = (z->tx_cur + 1) % ZX_RING;
	return 0;
}

static int zx_omci_rx(void *chip, unsigned int *link, u16 *gem_port, u8 pdu[GPON_OLT_OMCI_LEN])
{
	struct zx_chip *z = chip;
	struct zx_desc *d = &z->rx[z->rx_cur];
	u32 st = be32_to_cpu(READ_ONCE(d->status));
	const u8 *f = z->rxb + ZX_RX_BUF * z->rx_cur;
	struct eth_rx_frame fr = { 0 };
	int rc = -EPROTO;

	if (!(st & ZX_DESC_OWN))
		return 0;
	dma_rmb();
	if (eth_rx_frame_geom(&zx_rx_layout, eth_rx_dlen(st, ZX_RXD_LEN_MASK), true, &fr) == ETH_RX_OK &&
	    fr.len >= GPON_OLT_OMCI_LEN && f[0] == ZX_HDR_OMCI) {
		*link = f[3] & 0x7f;
		*gem_port = get_unaligned_be16(f + 6);
		memcpy(pdu, f + fr.off, GPON_OLT_OMCI_LEN);
		rc = 1;
	}
	WRITE_ONCE(d->status, 0);
	z->rx_cur = (z->rx_cur + 1) % ZX_RING;
	return rc;
}

/* No access to the chip after this: at reboot the host bridge resets the link next (an access then aborts). */
static void zx_stop(void *chip)
{
	unsigned int n;

	zx_wr(ZX_INT_MASK, 0);
	zx_wr(ZX_TX_CFG, 0);	/* INFERRED: bit 8 clear stops a ring */
	zx_wr(ZX_RX_CFG, 0);
	for (n = 0; n < ZX_PORT_LINKS; n++)
		zx_wr(ZX_W(n) + ZX_DS_CTRL, zx_rd(ZX_W(n) + ZX_DS_CTRL) & ~ZX_DS_ON);
}

/* The PON netdev's carrier: the link's downstream sender on and the XAUI to the switch in sync. */
static bool zx_link_up(void *chip, unsigned int link)
{
	return (zx_rd(ZX_XAUI_STA) & ZX_XAUI_LANES) == ZX_XAUI_LANES && (zx_rd(ZX_W(link) + ZX_DS_CTRL) & ZX_DS_ON);
}

/* ethtool -d pon0: identity and status, the XAUI to the switch, link 0's sender, the port words stock leaves set. */
static const u32 zx_pon_regs[] = {
	ZX_CHIP_ID, ZX_STATUS, ZX_XAUI_STA, 0x00084, ZX_W(0) + ZX_DS_CTRL,
	0x302a4, 0x303a4, 0x304a4, 0x305a4, 0x305c0, 0x307a4,
};

static const struct gpon_olt_pon_chip zx_pon = {
	.link_up = zx_link_up, .rd = zx_pm_rd, .regs = zx_pon_regs, .n_regs = ARRAY_SIZE(zx_pon_regs),
};

/* The burst the receiver detects once stock's MAC/TM init is in: stock's own Upstream_Overhead (32 guard bits,
 * type 3 preamble 0xaa, delimiter 0xab5983, pre-equalised, no delay) and Extended_Burst_Length (119 / 15 bytes). */
static const struct gpon_olt_overhead zx_burst = {
	.guard_bits = 0x20, .preamble3_pattern = 0xaa, .delimiter = { 0xab, 0x59, 0x83 }, .options = 0x20,
	.preamble3_unranged = 0x77, .preamble3_ranged = 0x0f,
};

static const struct gpon_olt_mac_ops zx_ops = {
	.ploam_tx = zx_ploam_tx, .sn_window = zx_sn_window, .ranging_grant = zx_ranging_grant,
	.omci_tx = zx_omci_tx, .ploam_rx = zx_ploam_rx, .omci_rx = zx_omci_rx, .stop = zx_stop, .burst = &zx_burst,
	.dba_start = zx_dba_start,
};

/* The frame pipe as the firmware sets it up: both rings, the DMA control word, both interrupts unmasked. */
static int zx_pipe_init(struct zx_chip *z)
{
	struct device *dev = &z->pdev->dev;
	unsigned int i;

	z->tx = dmam_alloc_coherent(dev, ZX_RING * sizeof(*z->tx), &z->tx_dma, GFP_KERNEL);
	z->rx = dmam_alloc_coherent(dev, ZX_RING * sizeof(*z->rx), &z->rx_dma, GFP_KERNEL);
	z->txb = dmam_alloc_coherent(dev, ZX_RING * ZX_TX_BUF, &z->txb_dma, GFP_KERNEL);
	z->rxb = dmam_alloc_coherent(dev, ZX_RING * ZX_RX_BUF, &z->rxb_dma, GFP_KERNEL);
	if (!z->tx || !z->rx || !z->txb || !z->rxb)
		return -ENOMEM;
	for (i = 0; i < ZX_RING; i++) {
		z->tx[i] = (struct zx_desc){ 0, cpu_to_be32(z->txb_dma + ZX_TX_BUF * i) };
		z->rx[i] = (struct zx_desc){ 0, cpu_to_be32(z->rxb_dma + ZX_RX_BUF * i) };
	}
	zx_wr(ZX_TX_BASE, z->tx_dma);
	zx_wr(ZX_RX_BASE, z->rx_dma);
	zx_wr(ZX_DMA_CTRL, ZX_DMA_CTRL_ON);
	zx_wr(ZX_INT_MASK, 3);
	zx_wr(ZX_BR_INT_EN, 0x40);
	zx_wr(ZX_TX_CFG, ZX_RING_CFG);
	zx_wr(ZX_RX_CFG, ZX_RING_CFG);
	return 0;
}

static int zx_pon_start(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpon_olt_profile tmpl;
	struct zx_chip *z;
	unsigned int n;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");
	pci_set_master(pdev);
	z = devm_kzalloc(dev, sizeof(*z), GFP_KERNEL);
	if (!z)
		return -ENOMEM;
	z->pdev = pdev;
	ret = zx_pipe_init(z);
	if (ret)
		return dev_err_probe(dev, ret, "frame pipe\n");
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
	ret = gpon_olt_pon_register(dev, &z->mac, &zx_pon);
	if (ret) {
		gpon_olt_mac_stop(&z->mac);
		return ret;
	}
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
	ret = gpon_olt_pon_lasers_ready(dev);
	if (ret)
		return dev_err_probe(dev, ret, "laser lines\n");
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
	ret = zx_mac_tm_init(&zx_io);
	if (ret) {
		dev_err(dev, "mac/tm init (%d)\n", ret);
		goto fail;
	}
	dev_info(dev, "stage mac/tm: done\n");
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
