// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/*
 * ZTE ZX279128 GPON MAC: the shell of the common gpon_* ONU core. PLOAM goes through the MAC's registers; the MAC
 * answers serial-number and ranging grants itself, from the state, ONU-ID and serial number it is given. OMCI and
 * the WAN's frames go through the packet processor's PON rings, which zx279128_eth.c owns; the WAN is the common
 * gpon0 (gpon_soft_wan.c) on the GEM port and T-CONT the common data plan arms. Facts: ONU-F680.md.
 * The optics: the laser supply (`laser-enable-gpios`, GPIO 72) follows the MAC's transmitter; the module's SFF-8472
 * diagnostics (`optics`, an I2C client) are sampled by gpon_optics_i2c.c, for /proc/gpon and ANI-G.
 * Not here yet: the analog bring-up (clock, SERDES, trims) and the packet processor's own forwarding. Without a
 * serial number the transmitter stays off; the number comes by the `sn` parameter, on the command line or written
 * later (/sys/module/zx279128_eth/parameters/sn).
 */
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/unaligned.h>

#include "gpon_common.h"
#include "gpon_data_plan.h"
#include "gpon_hwaddr.h"
#include "gpon_ploam.h"
#include "gpon_omci_core.h"
#include "gpon_optics_i2c.h"
#include "gpon_omci_me.h"
#include "gpon_sn.h"
#include "gpon_soft_wan.h"
#include "zx279128_gpon.h"

/* Chip level (offsets from 0x92000000). */
#define TOP_RAM_INIT		0x04001c	/* bits 3-0 written 1 start a RAM init and read 0 once done */
#define TOP_CAP			0x040020
#define TOP_DYING_GASP		0x040024	/* bit 16: watch the supply */
#define TOP_INT			0x040040
#define TOP_INT_MASK		0x040044	/* 1 = masked */
#define TOP_INT_GPON		BIT(0)
#define TOP_CTRL		0x040068	/* bit 15 dying gasp valid, bit 2 transmit path, bit 1 rogue detect */
/* MAC, downstream. */
#define MAC_INT			0x0c0000	/* write 1 to clear; bit 7 is a level: a received PLOAM waits */
#define MAC_INT_MASK		0x0c0004	/* 1 = masked */
#define MAC_INT_PLOAM		BIT(7)
#define MAC_INT_ALL		0x1ef
#define MAC_INT_OPEN		0xfffffe10	/* stock's mask: every source but bit 4 */
#define MAC_BIP_CTRL		0x0c0008	/* bit 0 set around a read of MAC_BIP clears the counter */
#define MAC_RX_CFG		0x0c4000	/* bits 1-0 receive, 6-4 and 10-8 the frame sync thresholds */
#define MAC_RX_STATUS		0x0c4014
#define MAC_RX_LOS		BIT(5)
#define MAC_RX_LOF		BIT(4)
#define MAC_PLOAM_CFG		0x0c4024
#define MAC_BIP			0x0c4070
#define MAC_RX_PLOAM		0x0c4084	/* three big-endian words: ONU-ID, message ID, 10 octets */
#define MAC_RX_POP		0x0c4094	/* a write drops the message read */
#define MAC_AES_DS_KEY		0x0c80d0
#define MAC_AES_DS_SWITCH	0x0c80e4
#define MAC_GEM(port)		(0x0cc000 + 4 * (port))	/* bit 0 accepted, bit 1 encrypted */
#define MAC_GEM_PORTS		4096
#define MAC_OMCC		0x0d0000	/* bits 31-20 the OMCC's GEM port, bit 0 OMCI to the CPU */
#define MAC_OMCI_MAX		0x0d0008
/* MAC, upstream. */
#define MAC_ONU			0x0d4000	/* bits 2-0 the O-state, bits 23-16 the ONU-ID */
#define MAC_RAND_DELAY		0x0d4004	/* bits 7-0, units of 32 bytes */
#define MAC_RAND_DELAY_MAX	233
#define MAC_RAND_EN		0x0d4008
#define MAC_EQD			0x0d4014	/* bits 15-0 bytes, 20-16 frames, 23-21 bits */
#define MAC_START_CHECK		0x0d4018
#define MAC_PRE_EQD		0x0d4024	/* the pre-assigned delay, bytes */
#define MAC_TCONT0		0x0d407c	/* bit 12 enabled, bits 11-0 the Alloc-ID */
#define MAC_TCONT_PAIR(n)	(0x0d4080 + 4 * (((n) - 1) / 2))	/* odd n low half, even n high half */
#define MAC_TCONTS		32
#define MAC_TCONT_EN		BIT(12)
#define MAC_LASER		0x0d8000	/* bit 1: forced off */
#define MAC_SN			0x0d8040
#define MAC_US_GO		0x0d8048	/* write 1 to send; reads non-zero while busy */
#define MAC_US_PLOAM		0x0d804c
#define MAC_US_REPEAT		0x0d8058	/* bits 17-16 the repeat count, bits 5-0 = 3 as stock */
#define MAC_PREAMBLE		0x0d80d4	/* bits: Type 1 << 18 | Type 2 << 10 | Type 3 (10 bits) */
#define MAC_PREAMBLE_PTN	0x0d80d8
#define MAC_DELIM_SIZE		0x0d80dc
#define MAC_DELIM		0x0d80e0
#define MAC_BOH_LATCH		0x0d80e4
#define MAC_POWER_LEVEL		0x0d8064	/* bits 1-0 */
#define US_AES_SWITCH		0x100020
#define US_AES_KEY		0x100038
#define US_PLI_MAX		0x100414
#define TM_TCONTS		0x160000	/* the T-CONT count, bit 3 a request that clears itself */

#define ZX_TICK_MS		10
#define ZX_US_FIFO		8
#define ZX_PLOAM_BURST		16
#define ZX_PON_MAC_INDEX	4	/* after the LAN ports' 0-3 */
#define PINMUX_PON_BIT		BIT(17)	/* in the pinmux word (0x9420000c): cleared at start, as stock */

struct zx_reg {
	u32 off, mask, val;
};

/* The MAC as stock's GPON module leaves it before it starts: O1, no ONU-ID, the default burst overhead. */
static const struct zx_reg zx_mac_init[] = {
	{ MAC_ONU, 0x00ff0007, 0xff << 16 | GPON_O1_INITIAL }, { MAC_EQD, 0x00ffffff, 0 },
	{ MAC_PREAMBLE_PTN, 0xff, 0xaa }, { MAC_PREAMBLE, 0x03ffffff, 3 },
	{ MAC_DELIM, 0x00ffffff, 0xab5983 }, { MAC_DELIM_SIZE, 3, 3 },
	{ MAC_RX_CFG, 0x770, 0x520 }, { MAC_PLOAM_CFG, 0xf, 8 },
	{ TOP_CAP, 0x7ff, 0x100 }, { US_PLI_MAX, ~0u, 400 }, { TM_TCONTS, ~0u, 16 | 8 },
	{ TOP_CTRL, BIT(15) | BIT(1), BIT(15) | BIT(1) }, { MAC_BIP_CTRL, 1, 0 },
	{ MAC_START_CHECK, BIT(4), BIT(4) }, { MAC_RX_CFG, BIT(16), 0 }, { MAC_OMCI_MAX, 0x3fff, 2000 },
};

/* Stock's transmit enable, in its order; off is every value 0 and the laser forced off. */
static const struct zx_reg zx_tx_on[] = {
	{ 0x0d80e8, 0xff, 1 }, { 0x0d80fc, 0x7f, 1 }, { 0x0d80e8, 0xff00, 6 << 8 }, { TOP_CTRL, BIT(2), BIT(2) },
	{ MAC_LASER, BIT(1), 0 }, { 0x0d8004, 1, 1 }, { 0x0d800c, 1, 1 }, { 0x0d80f4, 1, 1 },
};

struct zx_pon {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct zx_eth *eth;
	spinlock_t lock;		/* the core is single-threaded: every call holds this */
	struct timer_list tick;
	struct gpon_ploam p;
	struct gpon_ploam_cfg cfg;
	struct omci_onu *onu;
	u8 sn[GPON_SN_BYTES];
	bool omcc;
	u16 omcc_gem;
	struct gpon_soft_wan wan;
	struct gpon_data_armed armed;	/* the WAN's GEM port and Alloc-ID in the MAC */
	struct gpio_desc *laser;	/* the optics' laser supply, NULL when the board has no line */
	void __iomem *pinmux;		/* the node's third register range */
	struct gpon_optics_i2c optics;
	u8 us_fifo[ZX_US_FIFO][GPON_PLOAM_US_LEN];
	unsigned int us_head, us_count;
	u32 o5_lost;
	u64 ploam_rx, ploam_tx, ploam_tx_dropped, omci_rx, omci_tx, omci_tx_fail, rx_foreign;
};

static DEFINE_MUTEX(zx_pon_mutex);	/* zx_pon and zx_sn */
static struct zx_pon *zx_pon;		/* the SoC's one MAC, for /proc and the sn parameter */
static u8 zx_sn[GPON_SN_BYTES];

static u32 rd(struct zx_pon *pon, u32 off)
{
	return readl(pon->base + off);
}

static void wr(struct zx_pon *pon, u32 off, u32 val)
{
	writel(val, pon->base + off);
}

static void fld(struct zx_pon *pon, u32 off, u32 mask, u32 val)
{
	wr(pon, off, (rd(pon, off) & ~mask) | (val & mask));
}

static void zx_regs(struct zx_pon *pon, const struct zx_reg *r, unsigned int n, bool on)
{
	for (; n; n--, r++)
		fld(pon, r->off, r->mask, on ? r->val : 0);
}

static void zx_tx_enable(struct zx_pon *pon, bool on)
{
	zx_regs(pon, zx_tx_on, ARRAY_SIZE(zx_tx_on), on);
	if (!on)
		fld(pon, MAC_LASER, BIT(1), BIT(1));
	gpiod_set_value(pon->laser, on);
}

/* --- upstream PLOAM: one message at a time in the MAC, the rest wait here */
static void zx_us_drain(struct zx_pon *pon)
{
	while (pon->us_count && !rd(pon, MAC_US_GO)) {
		const u8 *m = pon->us_fifo[pon->us_head];
		int w;

		wr(pon, MAC_US_REPEAT, 1 << 16 | 3);
		for (w = 0; w < 3; w++)
			wr(pon, MAC_US_PLOAM + 4 * w, get_unaligned_be32(m + 4 * w));
		wr(pon, MAC_US_GO, 1);
		pon->us_head = (pon->us_head + 1) % ZX_US_FIFO;
		pon->us_count--;
		pon->ploam_tx++;
	}
}

static void zx_ploam_tx(void *sh, u8 queue, const u8 m[GPON_PLOAM_US_LEN])
{
	struct zx_pon *pon = sh;

	/* its Serial_Number_ONU and its idle PLOAM are the MAC's own */
	if (queue != PLM_US_QUEUE_URG)
		return;
	if (pon->us_count == ZX_US_FIFO) {
		pon->ploam_tx_dropped++;
		dev_warn_ratelimited(pon->dev, "upstream PLOAM 0x%02x dropped: %d already wait\n", m[1], ZX_US_FIFO);
		return;
	}
	memcpy(pon->us_fifo[(pon->us_head + pon->us_count++) % ZX_US_FIFO], m, GPON_PLOAM_US_LEN);
	zx_us_drain(pon);
}

/* --- the core's ops */
static void zx_boh_write(void *sh, u32 cfg_word, const u8 *oh, u8 size)
{
	struct zx_pon *pon = sh;
	const struct gpon_ploam *p = &pon->p;

	wr(pon, MAC_PREAMBLE_PTN, p->boh_ptn);
	wr(pon, MAC_PREAMBLE, min_t(u32, p->boh_t1, 31) * 8 << 18 | min_t(u32, p->boh_t2, 31) * 8 << 10 |
	   min_t(u32, p->boh_t3, 127) * 8);
	/* level 3 is reserved: stock keeps the level it had */
	if (p->boh_power != 3)
		fld(pon, MAC_POWER_LEVEL, 3, p->boh_power);
	wr(pon, MAC_DELIM, p->boh_delim[0] << 16 | p->boh_delim[1] << 8 | p->boh_delim[2]);
	wr(pon, MAC_DELIM_SIZE, 3);
	wr(pon, MAC_BOH_LATCH, 1);
	/* a new serial-number response delay with each overhead, as stock */
	fld(pon, MAC_RAND_DELAY, 0xff, get_random_u32_below(MAC_RAND_DELAY_MAX));
	wr(pon, MAC_RAND_EN, 1);
}

static u32 zx_min_delay(void *sh)
{
	return 0;
}

static void zx_set_eqd(void *sh, u32 multiframe, u32 intraframe)
{
	struct zx_pon *pon = sh;

	if (pon->p.state < GPON_O4_RANGING) {
		wr(pon, MAC_PRE_EQD, (multiframe * (GPON_PLOAM_EQD_FRAME_LEN / 8) + intraframe / 8) & 0x1fffff);
		wr(pon, MAC_EQD, 0);
		return;
	}
	if (multiframe > 31)
		dev_warn_ratelimited(pon->dev, "equalization delay of %u frames: the MAC holds 31\n", multiframe);
	wr(pon, MAC_EQD, (intraframe % 8) << 21 | min(multiframe, 31u) << 16 | intraframe / 8);
}

static void zx_none(void *sh)
{
}

static void zx_set_hw_state(void *sh, enum gpon_ostate st)
{
	fld(sh, MAC_ONU, 7, st);
}

static void zx_data_teardown(struct zx_pon *pon)
{
	if (pon->armed.installed)
		fld(pon, MAC_GEM(pon->armed.gem), 1, 0);
	memset(&pon->armed, 0, sizeof(pon->armed));
}

/* What one activation installed: gone with the ONU-ID. */
static void zx_session_clear(struct zx_pon *pon)
{
	int i;

	zx_data_teardown(pon);
	wr(pon, MAC_TCONT0, 0);
	for (i = 1; i <= MAC_TCONTS; i += 2)
		wr(pon, MAC_TCONT_PAIR(i), 0);
	if (pon->omcc)
		wr(pon, MAC_GEM(pon->omcc_gem), 0);
	fld(pon, MAC_OMCC, 0xfff << 20, 0);
	pon->omcc = false;
	wr(pon, MAC_EQD, 0);
	pon->us_count = 0;
}

static void zx_set_hw_onu_id(void *sh, u8 onu_id)
{
	struct zx_pon *pon = sh;

	fld(pon, MAC_ONU, 0xff << 16, onu_id << 16);
	if (onu_id == GPON_PLOAM_ONU_ID_BROADCAST)
		zx_session_clear(pon);
}

static void zx_on_below_o5(void *sh)
{
	struct zx_pon *pon = sh;

	pon->o5_lost++;
}

static void zx_rng(void *sh, u8 *out, unsigned int len)
{
	get_random_bytes(out, len);
}

static void zx_aes_stage_key(void *sh, const u8 key[16])
{
	struct zx_pon *pon = sh;
	int i;

	for (i = 0; i < 4; i++) {
		wr(pon, MAC_AES_DS_KEY + 4 * i, get_unaligned_be32(key + 4 * i));
		wr(pon, US_AES_KEY + 4 * i, get_unaligned_be32(key + 4 * i));
	}
	fld(pon, US_AES_SWITCH, BIT(30), BIT(30));
}

static void zx_aes_arm_switch(void *sh, u32 superframe)
{
	struct zx_pon *pon = sh;

	fld(pon, US_AES_SWITCH, 0x3fffffff, superframe);
	fld(pon, US_AES_SWITCH + 4, BIT(31), BIT(31));
	wr(pon, MAC_AES_DS_SWITCH, (superframe & 0x3fffffff) | BIT(30));
}

static int zx_install_omcc(void *sh, u16 gem)
{
	struct zx_pon *pon = sh;

	if (gem >= MAC_GEM_PORTS)
		return -EINVAL;
	fld(pon, MAC_OMCC, 0xfff << 20, gem << 20);
	wr(pon, MAC_GEM(gem), 1);
	pon->omcc = true;
	pon->omcc_gem = gem;
	return 0;
}

static int zx_install_tcont(void *sh, u8 tcont, u16 alloc)
{
	struct zx_pon *pon = sh;

	if (tcont > MAC_TCONTS || alloc > 0xfff)
		return -EINVAL;
	if (!tcont)
		wr(pon, MAC_TCONT0, MAC_TCONT_EN | alloc);
	else if (tcont & 1)
		fld(pon, MAC_TCONT_PAIR(tcont), 0x1fff, MAC_TCONT_EN | alloc);
	else
		fld(pon, MAC_TCONT_PAIR(tcont), 0x1fff << 16, (MAC_TCONT_EN | alloc) << 16);
	return 0;
}

static void zx_install_data_gem(void *sh, u16 gem)
{
	if (gem < MAC_GEM_PORTS)
		fld(sh, MAC_GEM(gem), 1, 1);
}

static void zx_port_encrypt(void *sh, u16 port, bool encrypted)
{
	if (port < MAC_GEM_PORTS)
		fld(sh, MAC_GEM(port), BIT(1), encrypted ? BIT(1) : 0);
}

static int zx_ds_bip_errors(void *sh, u32 *count)
{
	struct zx_pon *pon = sh;

	wr(pon, MAC_BIP_CTRL, 1);
	*count = rd(pon, MAC_BIP);
	wr(pon, MAC_BIP_CTRL, 0);
	return 0;
}

/* The MAC sends its own Dying_Gasp: the core's template has no register here, the supply watch does. */
static void zx_arm_dying_gasp(void *sh, const u8 m[GPON_PLOAM_US_LEN])
{
	fld(sh, TOP_DYING_GASP, BIT(16), BIT(16));
}

static bool zx_omci_send(struct zx_pon *pon, const u8 *msg)
{
	int ret = zx_eth_pon_tx(pon->eth, msg, OMCI_LEN, pon->omcc_gem, pon->cfg.omcc_tcont, true);

	if (ret) {
		pon->omci_tx_fail++;
		dev_warn_ratelimited(pon->dev, "upstream OMCI not sent: %d\n", ret);
		return false;
	}
	pon->omci_tx++;
	return true;
}

static void zx_report_oper_up(void *sh)
{
	struct zx_pon *pon = sh;
	u8 msg[OMCI_LEN];

	if (pon->omcc && omci_onu_emit_veip_up_avc(pon->onu, msg) == OMCI_LEN)
		zx_omci_send(pon, msg);
}

static void zx_trace(void *sh, enum gpon_ploam_ev ev, u32 a, u32 b)
{
	struct zx_pon *pon = sh;

	if (ev == GPON_PLOAM_EV_STATE)
		dev_info(pon->dev, "PON state O%u => O%u\n", a, b);
}

static void zx_unsupported(void *sh, const char *kind, int cls, u32 val, const char *want, const u8 *data,
			   unsigned int len)
{
	struct zx_pon *pon = sh;

	dev_info_ratelimited(pon->dev, "unsupported %s %d 0x%x (want %s)\n", kind, cls, val, want ? want : "-");
}

static const struct gpon_ploam_ops zx_ops = {
	.ploam_tx = zx_ploam_tx, .boh_write = zx_boh_write, .get_min_delay = zx_min_delay, .set_eqd = zx_set_eqd,
	.us_ploam_flush = zx_none, .set_hw_state = zx_set_hw_state, .set_hw_onu_id = zx_set_hw_onu_id,
	.on_below_o5 = zx_on_below_o5, .o5_rearm_burst = zx_none, .analog_relock = zx_none,
	.o3_feed_reset = zx_none, .cdr_reseat = zx_none, .rng = zx_rng, .aes_stage_key = zx_aes_stage_key,
	.aes_arm_switch = zx_aes_arm_switch, .install_omcc = zx_install_omcc, .install_tcont = zx_install_tcont,
	.install_data_gem = zx_install_data_gem, .omci_report_oper_up = zx_report_oper_up,
	.ds_bip_errors = zx_ds_bip_errors, .port_encrypt = zx_port_encrypt, .arm_dying_gasp = zx_arm_dying_gasp,
	.trace = zx_trace, .unsupported = zx_unsupported,
};

/* --- the WAN: the GEM port and the Alloc-ID the OLT provisioned, armed and withdrawn by the common plan */
static void zx_data_sync(struct zx_pon *pon)
{
	bool omcc_up = pon->omcc && pon->p.state == GPON_O5_OPERATION;
	struct gpon_data_want want = { .omcc_alloc = pon->p.onu_id, .omcc_up = omcc_up };
	struct omci_data_binding b;

	gpon_soft_wan_poll(&pon->wan, pon->onu, omcc_up, pon->omcc_gem);
	omci_data_binding_snapshot(pon->onu, pon->omcc_gem, GPON_MCAST_GEM_PORT, &b);
	want.gem = b.gem_present ? b.gem_port : 0;
	want.alloc = b.alloc_id;
	want.alloc_known = b.gem_present && b.alloc_known &&
			   (want.alloc == want.omcc_alloc || gpon_ploam_data_alloc_known(&pon->p, want.alloc));
	switch (gpon_data_plan_decide(&pon->armed, &want)) {
	case GPON_DATA_TEARDOWN:
	case GPON_DATA_REPLACE:		/* the next tick installs */
		zx_data_teardown(pon);
		break;
	case GPON_DATA_INSTALL:
		pon->armed.alloc = gpon_gem_us_alloc_id(want.alloc);
		pon->armed.gem = gpon_gem_us_port_id(want.gem);
		pon->armed.rides_omcc = gpon_gem_us_tcont_decide(want.alloc, want.omcc_alloc, false) ==
					GPON_GEM_US_BIND_IS_OMCC;
		pon->armed.alloc_bound = true;
		pon->armed.installed = true;
		fld(pon, MAC_GEM(pon->armed.gem), 1, 1);
		dev_info(pon->dev, "WAN on GEM port %u, Alloc-ID %u%s\n", pon->armed.gem, pon->armed.alloc,
			 pon->armed.rides_omcc ? ", the OMCC's" : "");
		break;
	default:
		break;
	}
}

static int zx_wan_tx(void *sh, const struct omci_vlan_service *svc, const u8 *frame, unsigned int len)
{
	struct zx_pon *pon = sh;
	unsigned int tcont;
	bool armed;

	spin_lock_bh(&pon->lock);
	armed = pon->armed.installed && svc->gem_port == pon->armed.gem;
	tcont = pon->armed.rides_omcc ? pon->cfg.omcc_tcont : pon->cfg.data_tcont;
	spin_unlock_bh(&pon->lock);
	return armed ? zx_eth_pon_tx(pon->eth, frame, len, svc->gem_port, tcont, false) : -ENOLINK;
}

/* --- downstream */
void zx_pon_rx(struct zx_pon *pon, bool omci, u16 gem, const u8 *msg, unsigned int len)
{
	u8 resp[OMCI_LEN];

	spin_lock_bh(&pon->lock);
	if (!omci && pon->armed.installed && gem == pon->armed.gem) {
		gpon_soft_wan_rx(&pon->wan, gem, msg, len);
	} else if (!omci || len < OMCI_LEN || !pon->omcc || gem != pon->omcc_gem ||
		   pon->p.state != GPON_O5_OPERATION) {
		pon->rx_foreign++;
	} else {
		pon->omci_rx++;
		if (omci_onu_input(pon->onu, msg, OMCI_LEN, resp) == OMCI_LEN)
			zx_omci_send(pon, resp);
		if (omci_onu_owed_avc(pon->onu, resp) == OMCI_LEN && zx_omci_send(pon, resp))
			omci_onu_owed_avc_sent(pon->onu);
	}
	spin_unlock_bh(&pon->lock);
}

static irqreturn_t zx_pon_irq(int irq, void *data)
{
	struct zx_pon *pon = data;
	u32 now = jiffies_to_msecs(jiffies);
	unsigned int n;

	if (!(rd(pon, TOP_INT) & ~rd(pon, TOP_INT_MASK) & TOP_INT_GPON))
		return IRQ_NONE;
	spin_lock_bh(&pon->lock);
	wr(pon, MAC_INT, rd(pon, MAC_INT) & MAC_INT_ALL & ~MAC_INT_PLOAM);
	for (n = 0; n < ZX_PLOAM_BURST && (rd(pon, MAC_INT) & MAC_INT_PLOAM); n++) {
		u8 m[GPON_PLOAM_DS_LEN] = { 0 };
		int w;

		for (w = 0; w < 3; w++)
			put_unaligned_be32(rd(pon, MAC_RX_PLOAM + 4 * w), m + 4 * w);
		wr(pon, MAC_RX_POP, 0);
		pon->ploam_rx++;
		gpon_ploam_ds(&pon->p, m, sizeof(m), now);
	}
	zx_us_drain(pon);
	spin_unlock_bh(&pon->lock);
	return IRQ_HANDLED;
}

/* --- the optics: the common sampler reads the module; the tick answers an ANI-G test from its last sample */
static void zx_optic_selftest(struct zx_pon *pon)
{
	u8 msg[OMCI_LEN];

	if (pon->omcc && pon->optics.reading.have && omci_onu_selftest_pending(pon->onu) &&
	    omci_onu_selftest_result(pon->onu, &pon->optics.reading, msg) == OMCI_LEN)
		zx_omci_send(pon, msg);
}

static void zx_pon_tick(struct timer_list *t)
{
	struct zx_pon *pon = timer_container_of(pon, t, tick);
	u32 now = jiffies_to_msecs(jiffies), rx;

	spin_lock(&pon->lock);
	rx = rd(pon, MAC_RX_STATUS);
	gpon_ploam_tick(&pon->p);
	gpon_ploam_sn_changed(&pon->p, now);
	gpon_ploam_poll_provision(&pon->p, now);
	gpon_ploam_poll_los(&pon->p, rx & MAC_RX_LOS, rx & MAC_RX_LOF, now);
	gpon_ploam_poll_keepalive(&pon->p, now);
	gpon_ploam_poll_rei(&pon->p, now);
	zx_data_sync(pon);
	zx_optic_selftest(pon);
	zx_us_drain(pon);
	spin_unlock(&pon->lock);
	mod_timer(&pon->tick, jiffies + msecs_to_jiffies(ZX_TICK_MS));
}

/* --- /proc/gpon_health: the lines every family's driver gives; /proc/gpon: this shell's counters */
static int zx_health_show(struct seq_file *m, void *v)
{
	struct zx_pon *pon = zx_pon;

	spin_lock_bh(&pon->lock);
	seq_printf(m, "onu_state: %u\nlos: %d\noptics_backend: %s\no5_lost: %u\n", pon->p.state,
		   !!(rd(pon, MAC_RX_STATUS) & MAC_RX_LOS), gpon_optics_i2c_name(&pon->optics), pon->o5_lost);
	spin_unlock_bh(&pon->lock);
	return 0;
}

static int zx_gpon_show(struct seq_file *m, void *v)
{
	struct zx_pon *pon = zx_pon;
	char far[96], optic[GPON_DDM_A2H_FMT_LEN];
	u32 rx;

	spin_lock_bh(&pon->lock);
	rx = rd(pon, MAC_RX_STATUS);
	omci_farend_fmt(pon->onu, far, sizeof(far));
	gpon_optics_i2c_fmt(&pon->optics, optic, sizeof(optic));
	seq_printf(m, "onu_state:   O%u\nonu_id:      %u\nfiber:       los=%d lof=%d\n", pon->p.state, pon->p.onu_id,
		   !!(rx & MAC_RX_LOS), !!(rx & MAC_RX_LOF));
	seq_printf(m, "omcc:        %s gem=%u\n", pon->omcc ? "up" : "down", pon->omcc_gem);
	seq_printf(m, "ploam:       rx=%llu tx=%llu tx_dropped=%llu\n", pon->ploam_rx, pon->ploam_tx,
		   pon->ploam_tx_dropped);
	seq_printf(m, "omci:        rx=%llu tx=%llu tx_fail=%llu\n", pon->omci_rx, pon->omci_tx, pon->omci_tx_fail);
	seq_printf(m, "wan:         %s gem=%u alloc=%u service=%s vid=%u rx_foreign=%llu\n",
		   pon->armed.installed ? "armed" : "off", pon->armed.gem, pon->armed.alloc,
		   pon->wan.up ? "up" : "down", pon->wan.svc.vid, pon->rx_foreign);
	seq_puts(m, far);
	seq_puts(m, optic);
	spin_unlock_bh(&pon->lock);
	return 0;
}

/* The MAC answers serial-number grants itself: its copy of the number, and no transmitter without one. */
static void zx_mac_identity(struct zx_pon *pon)
{
	wr(pon, MAC_SN, get_unaligned_be32(pon->sn));
	wr(pon, MAC_SN + 4, get_unaligned_be32(pon->sn + 4));
	zx_tx_enable(pon, gpon_sn_is_set(pon->sn));
}

static int zx_mac_start(struct zx_pon *pon)
{
	u32 v;
	int i;

	if (readl_poll_timeout(pon->base + TOP_RAM_INIT, v, !(v & 0xf), 10, 10000)) {
		dev_err(pon->dev, "the MAC's RAM init never completed (0x%08x)\n", v);
		return -ETIMEDOUT;
	}
	writel(readl(pon->pinmux) & ~PINMUX_PON_BIT, pon->pinmux);
	zx_regs(pon, zx_mac_init, ARRAY_SIZE(zx_mac_init), true);
	for (i = 0; i < MAC_GEM_PORTS; i++)
		wr(pon, MAC_GEM(i), 0);
	wr(pon, MAC_INT, MAC_INT_ALL);
	wr(pon, MAC_INT_MASK, MAC_INT_OPEN);
	zx_mac_identity(pon);
	fld(pon, MAC_RX_CFG, 3, 3);
	fld(pon, MAC_OMCC, 1, 1);
	return 0;
}

/* A new serial number is a new unit to the OLT: the activation and the MIB start over. */
static int zx_sn_set(const char *val, const struct kernel_param *kp)
{
	char text[GPON_SN_TEXT_LEN + 1] = "";
	u8 parsed[GPON_SN_BYTES];

	if (strcspn(val, "\n") != GPON_SN_TEXT_LEN)
		return -EINVAL;
	memcpy(text, val, GPON_SN_TEXT_LEN);
	if (gpon_sn_parse(text, parsed) || !gpon_sn_is_set(parsed))
		return -EINVAL;
	mutex_lock(&zx_pon_mutex);
	memcpy(zx_sn, parsed, sizeof(zx_sn));
	if (zx_pon) {
		spin_lock_bh(&zx_pon->lock);
		memcpy(zx_pon->sn, zx_sn, sizeof(zx_sn));
		gpon_ploam_set_sn(&zx_pon->p, zx_sn);
		omci_onu_init(zx_pon->onu, zx_sn, get_random_u8());
		omci_onu_set_wan_mac(zx_pon->onu, zx_pon->wan.dev->dev_addr);
		zx_mac_identity(zx_pon);
		spin_unlock_bh(&zx_pon->lock);
	}
	mutex_unlock(&zx_pon_mutex);
	return 0;
}

static int zx_sn_get(char *buf, const struct kernel_param *kp)
{
	char text[GPON_SN_TEXT_LEN + 1] = "";

	mutex_lock(&zx_pon_mutex);
	if (gpon_sn_is_set(zx_sn))
		gpon_sn_format(zx_sn, text);
	mutex_unlock(&zx_pon_mutex);
	return sysfs_emit(buf, "%s\n", text);
}

static const struct kernel_param_ops zx_sn_ops = { .set = zx_sn_set, .get = zx_sn_get };
module_param_cb(sn, &zx_sn_ops, NULL, 0644);
MODULE_PARM_DESC(sn, "G.984.3 serial number, AAAAhhhhhhhh; without one the unit never answers the OLT");

struct zx_pon *zx_pon_probe(struct device *dev, void __iomem *base, int irq, struct zx_eth *eth)
{
	u8 dt_mac[ETH_ALEN], mac[ETH_ALEN];
	const char *missing;
	bool has_mac;
	struct zx_pon *pon;
	int ret;

	pon = devm_kzalloc(dev, sizeof(*pon), GFP_KERNEL);
	if (!pon)
		return ERR_PTR(-ENOMEM);
	pon->onu = devm_kzalloc(dev, sizeof(*pon->onu), GFP_KERNEL);
	if (!pon->onu)
		return ERR_PTR(-ENOMEM);
	pon->dev = dev;
	pon->base = base;
	pon->irq = irq;
	pon->eth = eth;
	/* off until the MAC's transmitter is enabled: zx_mac_identity() */
	pon->laser = devm_gpiod_get_optional(dev, "laser-enable", GPIOD_OUT_LOW);
	if (IS_ERR(pon->laser))
		return ERR_PTR(dev_err_probe(dev, PTR_ERR(pon->laser), "laser-enable GPIO\n"));
	pon->pinmux = devm_platform_ioremap_resource(to_platform_device(dev), 2);
	if (IS_ERR(pon->pinmux))
		return ERR_CAST(pon->pinmux);
	spin_lock_init(&pon->lock);
	mutex_lock(&zx_pon_mutex);
	memcpy(pon->sn, zx_sn, sizeof(pon->sn));
	if (!gpon_sn_is_set(pon->sn))
		dev_warn(dev, "no serial number yet: the PON transmitter stays off and the OLT is not answered\n");
	pon->cfg.los_rerange_ticks = 30;
	pon->cfg.o4_ranging_timeout_ticks = GPON_DWELL_TO1_TICKS;
	pon->cfg.omcc_tcont = 0;
	pon->cfg.data_tcont = 1;
	pon->cfg.trace = true;
	omci_onu_init(pon->onu, pon->sn, get_random_u8());
	missing = gpon_ploam_init(&pon->p, &zx_ops, &pon->cfg, pon, pon->sn);
	if (missing)
		dev_err(dev, "the PLOAM core refused the shell: %s\n", missing);
	pon->wan.lock = &pon->lock;
	pon->wan.sh = pon;
	pon->wan.tx = zx_wan_tx;
	has_mac = !of_get_mac_address(dev->of_node, dt_mac);
	if (has_mac)
		gpon_hwaddr_derive(mac, dt_mac, ZX_PON_MAC_INDEX);
	ret = missing ? -EINVAL : gpon_soft_wan_init(&pon->wan, pon->onu, has_mac ? mac : NULL);
	if (!ret)
		ret = zx_mac_start(pon);
	if (!ret) {
		ret = request_threaded_irq(irq, NULL, zx_pon_irq, IRQF_ONESHOT, "zx279128-pon", pon);
		if (ret)
			dev_err(dev, "PON IRQ %d: %d\n", irq, ret);
	}
	if (!ret)
		zx_pon = pon;
	mutex_unlock(&zx_pon_mutex);
	if (ret) {
		gpon_soft_wan_exit(&pon->wan);
		return ERR_PTR(ret);
	}
	if (!proc_create_single("gpon_health", 0444, NULL, zx_health_show) ||
	    !proc_create_single("gpon", 0444, NULL, zx_gpon_show))
		dev_warn(dev, "no /proc/gpon, /proc/gpon_health\n");
	timer_setup(&pon->tick, zx_pon_tick, 0);
	mod_timer(&pon->tick, jiffies + msecs_to_jiffies(ZX_TICK_MS));
	fld(pon, TOP_INT_MASK, TOP_INT_GPON, 0);
	gpon_optics_i2c_start(&pon->optics, dev, &pon->lock, pon->onu);
	return pon;
}

void zx_pon_remove(struct zx_pon *pon)
{
	remove_proc_entry("gpon", NULL);
	remove_proc_entry("gpon_health", NULL);
	mutex_lock(&zx_pon_mutex);
	zx_pon = NULL;
	mutex_unlock(&zx_pon_mutex);
	fld(pon, TOP_INT_MASK, TOP_INT_GPON, TOP_INT_GPON);
	wr(pon, MAC_INT_MASK, ~0u);
	free_irq(pon->irq, pon);
	timer_delete_sync(&pon->tick);
	gpon_optics_i2c_stop(&pon->optics);
	zx_tx_enable(pon, false);
	fld(pon, MAC_RX_CFG, 3, 0);
	fld(pon, MAC_OMCC, 1, 0);
	gpon_soft_wan_exit(&pon->wan);
}
