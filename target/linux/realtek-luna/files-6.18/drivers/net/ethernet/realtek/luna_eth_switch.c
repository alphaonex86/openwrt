// SPDX-License-Identifier: GPL-2.0-only
/* TIER: FAMILY -- the RTL9607C and RTL9603CVD rows of the Luna Ethernet shell (luna_eth.c): the per-chip switch
 * table, the open-L2 switch bring-up, GPHY access and patch, the port gate, the CPU-tag framing and the flow
 * engine (luna_flow.c).  The two dies differ in their SWITCH registers only; the table carries them.
 *
 * Switch bring-up facts, established on the RTL9607C:
 *  - The load-bearing RX gate is the per-port spanning-tree state (MSTI_CTRL); SRC_PORT_PERMIT has ONE
 *    statement, luna_eth_regs.h's @src_permit.
 *  - MSTI_CTRL resets to 0x000000FF on both chips; ports found non-forwarding at probe are the boot loader's
 *    doing, so the write below is a correction.
 *  - The integrated copper PHYs need no analog calibration, except a rev B RTL9607C.
 *  - The ordered sequence matters: replaying only the final register values is not sufficient. */

#include "luna_eth.h"
#include <linux/firmware.h>
#include <linux/of_device.h>
#include "luna_ponmac.h"	/* LUNA_MODEL_NAME_INFO and the RTL9607C revision decode */
#include "luna_flow.h"
#ifdef CONFIG_GPON_FLOW_OFFLOAD
#include <net/pkt_cls.h>	/* enum tc_setup_type, for the ndo */
#endif

static bool sw_cpu_tag;
module_param(sw_cpu_tag, bool, 0644);
MODULE_PARM_DESC(sw_cpu_tag, "enable the switch CPU-port tag engine (default off: plain-L2 forwarding)");

/* CONFIG_LUNA_DEBUG: the bring-up aids below are compiled in only on request;
 * off, each knob is a constant 0 and its branches are dead code. */
#if IS_ENABLED(CONFIG_LUNA_DEBUG)
static unsigned int diag_ms = 3000;
module_param(diag_ms, uint, 0644);
MODULE_PARM_DESC(diag_ms, "period of the per-port real-link/rxpkts diagnostic (0 = off)");

static int diag_count = 12;
module_param(diag_count, int, 0644);
MODULE_PARM_DESC(diag_count, "number of periodic link/rxpkts diagnostic dumps");

static bool phy_survey = true;
module_param(phy_survey, bool, 0644);
MODULE_PARM_DESC(phy_survey, "at open, READ each copper port's BMCR/BMSR under BOTH OCP maps and dump them (read-only; turns a 3-boot experiment into a 1-boot one)");
#else
static const unsigned int diag_ms;
static const int diag_count;
static const bool phy_survey;
#endif

static bool copper_phy = true;
module_param(copper_phy, bool, 0644);
MODULE_PARM_DESC(copper_phy, "power up + auto-neg the internal copper PHYs (ports 0-4)");

static bool rtl8221b_phy = true;
module_param(rtl8221b_phy, bool, 0644);
MODULE_PARM_DESC(rtl8221b_phy, "de-assert the RTL8221B 2.5G PHY reset (SerDes-6 uplink)");

/* Default 0 = do NOT assert the GPHY reset, which is what the vendor does. A
 * param and not a deletion, so the old behaviour is one bootarg away. */
static bool gphy_reset;
module_param(gphy_reset, bool, 0444);
MODULE_PARM_DESC(gphy_reset,
		 "assert SOFTWARE_RST.CMD_GPHY_RST_PS during copper PHY bring-up "
		 "(default 0: the vendor never asserts it, and we run none of the "
		 "re-initialisation that reset would require)");

/* Which receive FIFOs stay in reset. Stock holds NONE; the one this board had
 * asserted came from our own GPON pad recipe writing CFG_PCSXF, which is 0x48
 * on this chip -- fixed at the source. -1 leaves the bootloader's value. */
static int rst_rxfifo;
module_param(rst_rxfifo, int, 0644);
MODULE_PARM_DESC(rst_rxfifo,
	"CFG_PCSXF RST_RXFIFO mask to program (-1 = leave the bootloader's value)");

/* CFG_PHY_CTRL BASE_PHYAD. MEASURED 2026-08-24 on the G24W: ...
 * dev/MEASURED-luna_eth.c.md sec 57. */
static int base_phyad = 0;
module_param(base_phyad, int, 0644);
MODULE_PARM_DESC(base_phyad,
	"CFG_PHY_CTRL BASE_PHYAD to program (-1 = leave the bootloader's value)");

/* The PHY survey is a probe-time dump, so testing one BASE_PHYAD used to cost
 * one build and one boot. This lets a single image answer the whole sweep. */
static struct luna_eth *survey_ep;

/* Let an OCP transaction START before sampling BUSY. The vendor uses a flat
 * mdelay(10) and no poll at all; 0 = sample immediately, the old behaviour. */
static int gphy_settle_us = 200;
module_param(gphy_settle_us, int, 0644);
MODULE_PARM_DESC(gphy_settle_us,
		 "microseconds to let an internal-PHY OCP transaction START before "
		 "sampling BUSY (0 = sample immediately, which returns success "
		 "before the transaction begins and reads a stale zero)");

/* Default 1 since 2026-08-23: emulating ... -- dev/MEASURED-luna_eth.c.md sec 58. */
static int gphy_map = 1;
/* The writable parameter itself is declared beside luna_chip_gphy_lock: its setter
 * has to take that mutex, and a knob that steers an address must not be able to
 * move between the read and the write of one transaction. */

static int phy_settle_ms;
module_param(phy_settle_ms, int, 0644);
MODULE_PARM_DESC(phy_settle_ms, "ms to wait after the PHY patch-done bit (0 = current behaviour; the RTL9603CVD's own U-Boot waits 800)");

static bool cpu_no_loopback = true;
module_param(cpu_no_loopback, bool, 0644);
MODULE_PARM_DESC(cpu_no_loopback, "drop the CPU port from its own egress flood (stops self-loopback RX)");

/* ★ A FRAME THE CPU FLOODS COMES BACK IN ON THE CPU PORT, AND ...
 * dev/MEASURED-luna_eth.c.md sec 2. */
static bool lan_flood_direct = true;
module_param(lan_flood_direct, bool, 0644);
MODULE_PARM_DESC(lan_flood_direct, "send CPU-originated multicast/broadcast to an explicit egress mask (the flood set minus the CPU port) instead of letting the switch flood it back at us");

/* CPU-tag engine config. CTEN_RX (bit31) makes the MAC strip the 8-byte switch
 * tag in hardware on RX and expose the parsed ingress port in the descriptor;
 * the rest selects tag sizes, the 0x04 protocol and the 0x8899 match. Clearing
 * CTEN_RX leaves the raw in-band tag in the delivered frame. */
#define CPUTAGCR_INIT	0x9022FF04
/* R_CPUTAG1CR[14:8] selects WHICH downstream stream-id the ...
 * dev/MEASURED-luna_eth.c.md sec 4. */
#define CPUTAG1CR_SID(s)	(((s) & 0x7fu) << 8)	/* R_CPUTAG1CR[14:8] */
#define CPUTAG1CR_SID_MASK	CPUTAG1CR_SID(0x7fu)
#define ABLTY_CPU_FORCE	0xBFFF		/* CPU-port forced-ability mode (keep)	*/


/* switch core (SWCORE), phys 0x1b000000 -- dev/MEASURED-luna_eth.c.md sec 5. */
#define   STP_STATE_MASK	0x3
#define   STP_FORWARDING	0x3

/* THE PER-CHIP TABLE. Everything in it MOVED between the two parts, and every
 * field was read from that chip's OWN vendor firmware (tier 3) rather than inferred from
 * the sibling. A zero means "this chip has no such register" and the code must
 * SKIP the write -- offset 0 is the PHY indirect-access data register. */
struct luna_eth_chip {
	const char *name;
	const struct luna_flow_layout *flow;

	/* The switch-core map for THIS chip, and the ONLY home of its port
	 * numbers: a pointer into luna_eth_regs.h so the sibling driver and this
	 * one read the SAME numbers and a correction lands once. */
	const struct luna_sw_map *sw_map;

	/* --- switch port map ------------------------------------------------ */
	/* ★ Force the PON port's MAC link like the CPU port's -- 1 only on a
	 * chip where STOCK was MEASURED doing it. See the write site. */
	u8	force_pon_ablty;
	u8	last_port;	/* highest port to iterate, INCLUSIVE		*/
	u8	gphy_ports;	/* bitmap: ports whose PHY is a GPHY, not FE	*/

	/* --- switch registers that MOVED ------------------------------------ */
	/* force_ablty / p_ablty / ablty_force live in luna_sw_map (->sw_map) */
	u32	msti_ctrl;	/* + 4*port: per-port spanning-tree state	*/
	u32	cpu_tag_insert;
	u32	cpu_tag_aware;
	u32	swcore_rst;	/* swcore soft reset (bit10), excludes cfg	*/
	/* gphy_misc lives in luna_sw_map too: the RTL9602C driver needs the same fact */
	u32	fephy_poll;	/* 0 on a chip with no FE-PHY auto-poller	*/
	u32	cfg_phy_ini;	/* per-port PHY enable; U-Boot loads it from efuse*/
	u32	cfg_phy_ctrl;	/* MSK_MDI[8:5] | BASE_PHYAD[4:0]; 0 = not known */
	u32	cfg_pcsxf;	/* RST_RXFIFO[13:10] | MIIRX_IPG[9:5] | PCSXF[4:1] */
	u32	igr_bwctrl_glb;	/* bit0: OMCI/OAM/IGMP bypass ingress rate and flow
				 * control; 0 = stock leaves it clear, not written */

	/* --- port-isolation packing, which differs in SHAPE not just offset -- */
	u8	piso_per_word;	/* how many ports share one 32-bit word		*/
	u8	piso_bits;	/* width of one port's mask			*/
	u32	piso_all;	/* the all-open value for ONE port		*/

	/* --- SerDes uplink: present only on the bigger part ------------------ */
	u32	serdes_linemode;	/* 0 = no SerDes on this chip		*/
	u32	force_ablty_x;		/* SerDes/PBO ability trio, 0 if absent	*/
	u32	ablty_force_x;
	u32	sds_fib_status;		/* + 0x20*idx, 0 if absent		*/

	/* --- SoC glue OUTSIDE the switch's own register space ---------------- */
	u32	sys_status;	/* 0 = this chip's bring-up does not use it	*/

	/* ★ A MODEL FACT LIVING IN THE CHIP TABLE, said out loud ...
	 * dev/MEASURED-luna_eth.c.md sec 6. */
	u8	wan_mac_offset;
	u32	cold_phy_status;	/* native cold PHY recipe; 0 keeps the warm path */
	/* The one silicon revision (MODEL_NAME_INFO, read at bring-up) whose
	 * stock kernel patches this chip's GPHYs; 0 = no analog patch. */
	u8	gphy_patch_rev;
};

/* The RTL9607C entry is this file's previous constants, value for value: the
 * engineering board that boots today must see a byte-identical register
 * sequence, so any behaviour change on it is a defect of the refactor. */
static const struct luna_eth_chip luna_chip_rtl9607c = {
	.sw_map		= &rtl9607c_sw_map,
	.name		= "RTL9607C",
	.flow		= NULL,	/* Flow layout not established on this die. */
	/* 0 DELIBERATELY, a scope statement and not a finding: nobody has diffed
	 * SWCORE 0x1cc/0x238 stock-vs-ours on the RTL9607C board, and this chip
	 * reaches its PON/PBO abilities through the force_ablty_x trio below. */
	.force_pon_ablty = 0,
	.last_port	= 11,	/* 0..4,8 copper; 5 PON; 6,7 SerDes; 9 CPU; 11 PBO */
	.gphy_ports	= 0x1f,	/* all five copper ports are GPHYs here	*/
	.msti_ctrl	= 0x1704C,
	.cpu_tag_insert	= 0x230F4,
	.cpu_tag_aware	= 0x230F8,
	.swcore_rst	= 0x00108,
	.fephy_poll	= 0,	/* every PHY here is a GPHY: no FE auto-poller	*/
	.cfg_phy_ini	= 0x0004C,
	/* ★ 0 WRITTEN DOWN, not left to the compiler (2026-09-14). ...
	 * dev/MEASURED-luna_eth.c.md sec 7. */
	.cfg_phy_ctrl	= 0,	/* NOT ESTABLISHED on this die -- never the sibling's */
	.cfg_pcsxf	= 0,	/* NOT ESTABLISHED on this die -- the write is skipped */
	.piso_per_word	= 1,
	.piso_bits	= 29,
	.piso_all	= 0x1FFFFFFF,
	.serdes_linemode = 0x00084,
	.force_ablty_x	= 0x002F4,
	.ablty_force_x	= 0x002FC,
	.sds_fib_status	= 0x0028C,
	.sys_status	= 0,
	/* 0 DELIBERATELY: nobody has captured a WAN identity on the engineering
	 * board, and it carries no ISP service.  Copying the G24W's 5 here would
	 * be one model's answer standing in for another's. */
	.wan_mac_offset	= 0,
	.cold_phy_status = 0xb8000044,
	.gphy_patch_rev	= LUNA_REV_B,	/* rev A/C dies run other patches: not ported */
};

/* The RTL9603CVD entry -- every field read from THIS chip's ...
 * dev/MEASURED-luna_eth.c.md sec 8. */
static const struct luna_eth_chip luna_chip_rtl9603cvd = {
	.sw_map		= &rtl9603cvd_sw_map,
	.name		= "RTL9603CVD",
	.flow = &luna_flow_rtl9603cvd,
	/* MEASURED 2026-08-27, stock vs ours, SWCORE 0x180..0x1fc: ...
	 * dev/MEASURED-luna_eth.c.md sec 9. */
	.force_pon_ablty = 1,
	.last_port	= 5,	/* 0..2 FE; 3 GE; 4 PON; 5 CPU (6 = PBO loopback)*/
	.gphy_ports	= 0x08,	/* ONLY port 3 is a GPHY; 0..2 are FE PHYs	*/
	.msti_ctrl	= 0x1713C,
	.cpu_tag_insert	= 0x2303C,
	.cpu_tag_aware	= 0x23040,
	.swcore_rst	= 0x000E0,
	.fephy_poll	= 0x0000C,
	.cfg_phy_ini	= 0x00050,
	.cfg_phy_ctrl	= 0x0004C,
	.cfg_pcsxf	= 0x00048,
	.igr_bwctrl_glb	= 0x230E0,	/* stock G24W reads 1 (swcore capture 2026-11-02) */
	.piso_per_word	= 2,
	.piso_bits	= 12,
	.piso_all	= 0xFFF,
	.serdes_linemode = 0,	/* no SerDes on this part			*/
	.force_ablty_x	= 0,
	.ablty_force_x	= 0,
	.sds_fib_status	= 0,
	.sys_status	= 0xB8000044,	/* SoC handshake, outside SWCORE		*/
	/* MEASURED on THIS unit's own stock, two independent sources ...
	 * dev/MEASURED-luna_eth.c.md sec 10. */
	.wan_mac_offset	= 5,
	.cold_phy_status = 0,	/* keep the existing warm PHY path on this die */
};

/* Accessors: the table lives in `ep->cs.c`, so a per-port register is one call and
 * a chip that lacks a register is answered with 0 and SKIPPED by the caller. */
#define SW_FORCE_ABLTY(ep, p)	((ep)->cs.c->sw_map->force_ablty + (p) * 4)
#define SW_P_ABLTY(ep, p)	((ep)->cs.c->sw_map->p_ablty + (p) * 4)
#define SW_ABLTY_FORCE(ep, p)	((ep)->cs.c->sw_map->ablty_force + (p) * 4)
#define SW_MSTI_CTRL(ep, p)	((ep)->cs.c->msti_ctrl + (p) * 4)

/* VLAN: filtering must not gate CPU<->LAN egress (the boot loader may leave it on
 * with the CPU port outside the member set). */
#define   VLAN_FILTERING	BIT(0)

/* Per-port lookup-miss (unknown-DA) action, 2 bits/port; 0 = FORWARD. Needed for
 * the post-ARP unicast / IPv6-ND path (the first broadcast already floods). */
#define   DA_ACT_PORTS		0x3FFFFF	/* ports 0..10, 2 bits each		*/

#define ABLTY_1G_FULL_LINK	0x16	/* speed=1000, duplex=full, link=up	*/
#define ABLTY_FORCE_ALL		0xFFF	/* force all basic abilities		*/

/* The port map lives in the chip table: the RTL9607C has 11 ...
 * dev/MEASURED-luna_eth.c.md sec 11. */
#define RTL_CPU_TAG_LEN		8
#define RTL_CPU_TAG_ETYPE	0x8899

/* SOC_SW_ENABLE and its bits are the FAMILY's -- ... -- dev/MEASURED-luna_eth.c.md sec 12. */
#define   CMD_GPHY_RST_PS	BIT(6)	/* SOFTWARE_RST(0x0E0) bit 6	*/
#define   FEPHY_STOP_POLL	BIT(16)	/* in fephy_poll: 1 = auto-poller OFF	*/

/* Live link / speed (genuine, independent of the MAC force). Copper genuine link
 * = MDIO BMSR bit2; SerDes genuine link = SDS_FIB_STATUS. */
#define SW_SDS_FIB_STATUS(ep, s) ((ep)->cs.c->sds_fib_status + (s) * 0x20)
#define   SDS_LINK_OK		BIT(4)
#define   SDS_SDET		BIT(17)

/* Per-port RX MIB counters (direct reads; block base 0x32600, stride 0x80). */
#define SW_MIB_RX_UCAST(p)	(0x32620 + (p) * 0x80)
#define SW_MIB_RX_MCAST(p)	(0x32628 + (p) * 0x80)
#define SW_MIB_RX_BCAST(p)	(0x3262C + (p) * 0x80)

/* RTL8221B 2.5G PHY reset line: DTS rtl8221b_dev0_reset = <&gpio1 28 1> (active
 * low). gpio1 = bank 1 (pins 32..63); pin 28 -> bit 28 of bank-1 DIR/DAT, plus
 * the GPIO function-enable for pins 32..63. */
#define SW_IO_GPIO_EN_HI	0x03c		/* pinmux function-enable, pins 32..63	*/
#define SOC_GPIO_B1_DIR	((void __iomem *)0xb8003324ul)	/* bank1 direction (1=out)*/
#define SOC_GPIO_B1_DAT	((void __iomem *)0xb8003328ul)	/* bank1 data		*/
#define RTL8221B_RST_BIT	BIT(28)
#define RTL8221B_PHYAD		6

/* MII BMCR/BMSR bits. The MII register bits are ... -- dev/MEASURED-luna_eth.c.md sec 13. */


static const struct luna_eth_gmac luna_chip_gmac = {
	.name	= "RTL9607C/RTL9603CVD",
	.cmd	= 0x0A,		/* RxChkSum | RxJumboSupport */
	.tcr	= 0x00000C01,	/* IFG=3, TX enabled via IO_CMD */
};

struct luna_eth_chip_state {
	const struct luna_eth_chip *c;	/* this die's row */
	struct luna_flow_engine *flow;	/* NULL = no flow offload on this die */
	struct timer_list diag;		/* CONFIG_LUNA_DEBUG link/rxpkts dump */
	struct work_struct diag_work;
	int diag_left;
	u32 lan_flood_mask;		/* the flood set minus the CPU port, 0 = not built */
	u32 rx_reason_seen[8];		/* first-seen ledgers: one line per distinct value, ever */
	u16 rx_src_port_seen;
	u32 rx_pon_sid_seen[4];
	unsigned int omci_sid;		/* the OMCC stream id, 0 until the OLT assigns one */
};

/* The indirect PHY window has ONE owner; the shell's port gate takes it too. */
static DEFINE_MUTEX(luna_chip_gphy_lock);

#include "luna_eth.c"

static inline u32 sw_rd(struct luna_eth *ep, u32 r) { return ioread32(ep->sw + r); }
static inline void sw_wr(struct luna_eth *ep, u32 r, u32 v) { iowrite32(v, ep->sw + r); }
static inline void sw_or(struct luna_eth *ep, u32 r, u32 v) { sw_wr(ep, r, sw_rd(ep, r) | v); }

#include "luna_flow.c"

/* internal GPHY MDIO (indirect window) -- dev/MEASURED-luna_eth.c.md sec 17. */

/* gphy_map picks the OCP map, so it is part of the ADDRESS ...
 * dev/MEASURED-luna_eth.c.md sec 18. */
static int gphy_map_set(const char *val, const struct kernel_param *kp)
{
	int rc;

	mutex_lock(&luna_chip_gphy_lock);
	rc = param_set_int(val, kp);
	mutex_unlock(&luna_chip_gphy_lock);
	return rc;
}
static const struct kernel_param_ops gphy_map_ops = {
	.set = gphy_map_set,
	.get = param_get_int,
};
module_param_cb(gphy_map, &gphy_map_ops, &gphy_map, 0644);
MODULE_PARM_DESC(gphy_map, "internal-PHY OCP map: 0 = per the chip table, 1 = force GPHY page 0xA40 on every port, 2 = force the flat FE map (a bring-up experiment: the vendor firmware and its own OCP map disagree for the FE ports)");

static int gphy_wait(struct luna_eth *ep)
{
	int i;

	/* let the transaction START before asking whether it has finished. */
	if (gphy_settle_us)
		udelay(gphy_settle_us);

	for (i = 0; i < 10000; i++) {
		if (!(sw_rd(ep, SW_GPHY_IND_RD) & GPHY_IND_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

/* The OCP address of standard MII register `reg` on the PHY behind switch port
 * `p`. See the note above GPHY_MII_PAGE for why this is not one constant. */
static u32 gphy_ocp(struct luna_eth *ep, unsigned int p, unsigned int reg)
{
	if (gphy_map == 1 || (gphy_map == 0 && (ep->cs.c->gphy_ports & BIT(p))))
		return luna_gphy_mii_ocp(reg);		/* GPHY page 0xA40 */
	return (reg & 0x1f) << 1;			/* FE PHY: flat map */
}

/* Read one OCP address on one PHY. The SURVEY needs this because it asks the
 * SAME register through BOTH maps, which `gphy_read()` cannot express: that one
 * consults the chip table (correctly) and so can only ever return one answer. */
static int gphy_read_ocp__locked(struct luna_eth *ep, unsigned int phyad,
				 u32 ocp, u16 *out)
{
	/* ★ PREFLIGHT.  Issuing a command while the previous transaction is
	 *   still BUSY makes the window answer for the wrong one. */
	if (gphy_wait(ep))
		return -ETIMEDOUT;
	sw_wr(ep, SW_GPHY_IND_CMD, (phyad << 16) | ocp | GPHY_IND_EN);
	if (gphy_wait(ep))
		return -ETIMEDOUT;
	*out = sw_rd(ep, SW_GPHY_IND_RD) & 0xffff;
	return 0;
}

/* > 0 and *out set, or negative with *out LEFT EXACTLY AS IT ...
 * dev/MEASURED-luna_eth.c.md sec 20. */
static int gphy_read__locked(struct luna_eth *ep, unsigned int phyad,
			     unsigned int reg, u16 *out)
{
	u32 adr = (phyad << 16) | gphy_ocp(ep, phyad, reg);

	if (gphy_wait(ep))
		goto timeout;
	sw_wr(ep, SW_GPHY_IND_CMD, adr | GPHY_IND_EN);
	if (gphy_wait(ep))
		goto timeout;
	*out = sw_rd(ep, SW_GPHY_IND_RD) & 0xffff;
	return 0;
timeout:
	dev_warn_ratelimited(ep->dev,
		"gphy: read timeout phy %u reg %u -- the indirect bus never cleared BUSY; NO value is returned\n",
		phyad, reg);
	return -ETIMEDOUT;
}

static int gphy_write_ocp__locked(struct luna_eth *ep, unsigned int phyad,
				  u32 ocp, u16 val)
{
	/* ★ PREFLIGHT BEFORE THE DATA REGISTER.  SW_GPHY_IND_WD was written while a
	 *   previous transaction could still be BUSY, so the value could be
	 *   consumed by that one instead of this one. */
	if (gphy_wait(ep))
		goto timeout;
	sw_wr(ep, SW_GPHY_IND_WD, val);
	sw_wr(ep, SW_GPHY_IND_CMD, GPHY_IND_PHY(phyad) | ocp |
				   GPHY_IND_WREN | GPHY_IND_EN);
	if (gphy_wait(ep))
		goto timeout;
	return 0;
timeout:
	dev_warn_ratelimited(ep->dev,
		"gphy: write timeout phy %u ocp %#06x val %04x -- the write did NOT land\n",
		phyad, ocp, val);
	return -ETIMEDOUT;
}

static int gphy_write__locked(struct luna_eth *ep, unsigned int phyad,
			      unsigned int reg, u16 val)
{
	return gphy_write_ocp__locked(ep, phyad, gphy_ocp(ep, phyad, reg), val);
}

static int luna_phy_read(struct net_device *ndev, unsigned int port,
			 unsigned int reg, u16 *val)
{
	return gphy_read__locked(netdev_priv(ndev), port, reg, val);
}

static int luna_phy_write(struct net_device *ndev, unsigned int port,
			  unsigned int reg, u16 val)
{
	return gphy_write__locked(netdev_priv(ndev), port, reg, val);
}

static const struct luna_phy_ops luna_chip_phy_ops = {
	.lock	= &luna_chip_gphy_lock,
	.read	= luna_phy_read,
	.write	= luna_phy_write,
};

/* The OCP address THIS BOARD'S OWN ADMINISTRATIVE PATH uses, ...
 * dev/MEASURED-luna_eth.c.md sec 21. */
static u32 gphy_ocp_miim(struct luna_eth *ep, unsigned int p, unsigned int reg)
{
	if (ep->cs.c->gphy_ports & BIT(p))
		return luna_gphy_mii_ocp(reg);
	return (reg & 0x1f) << 1;
}

/* Power up + (re)start auto-negotiation on the integrated ...
 * dev/MEASURED-luna_eth.c.md sec 22. */
static void eth_phy_survey__locked(struct luna_eth *ep)
{
	unsigned int p;

	lockdep_assert_held(&luna_chip_gphy_lock);

	if (ep->cs.c->cfg_phy_ini)
		dev_info(ep->dev,
			 "phy survey: CFG_PHY_INI(%#05x) = %08x  (U-Boot loads its per-port field from the efuse; we do NOT write it -- the polarity is unresolved)\n",
			 ep->cs.c->cfg_phy_ini, sw_rd(ep, ep->cs.c->cfg_phy_ini));

	for (p = 0; p < ep->cs.c->sw_map->n_copper; p++) {
		u16 g_bmcr = 0, g_bmsr = 0, f_bmcr = 0, f_bmsr = 0;
		int rc;

		/* four reads of ONE port through BOTH maps, inside the caller's
		 * single acquisition: a window taken per access would let
		 * another user answer in the middle of them */
		rc = gphy_read_ocp__locked(ep, p, luna_gphy_mii_ocp(MII_BMCR), &g_bmcr);
		if (!rc)
			rc = gphy_read_ocp__locked(ep, p, luna_gphy_mii_ocp(MII_BMSR), &g_bmsr);
		if (!rc)
			rc = gphy_read_ocp__locked(ep, p, 0 << 1, &f_bmcr);
		if (!rc)
			rc = gphy_read_ocp__locked(ep, p, 1 << 1, &f_bmsr);
		if (rc) {
			/* ★ NOT PRINTED AS VALUES.  The old code printed 0xffff
			 * and then reasoned about whether 0xffff meant anything. */
			dev_info(ep->dev,
				 "phy survey: port %u -- the bus did not answer (%d); no register values\n",
				 p, rc);
			continue;
		}

		dev_info(ep->dev,
			 "phy survey: port %u (%s by table)  gphy-page[bmcr=%04x bmsr=%04x]%s  flat[bmcr=%04x bmsr=%04x]%s\n",
			 p, (ep->cs.c->gphy_ports & BIT(p)) ? "GPHY" : "FE",
			 g_bmcr, g_bmsr,
			 (g_bmcr == 0xffff && g_bmsr == 0xffff) ? " <- ALL-ONES, i.e. no answer" : "",
			 f_bmcr, f_bmsr,
			 (f_bmcr == 0xffff && f_bmsr == 0xffff) ? " <- ALL-ONES, i.e. no answer" : "");
	}
}

static void eth_phy_survey(struct luna_eth *ep)
{
	mutex_lock(&luna_chip_gphy_lock);
	eth_phy_survey__locked(ep);
	mutex_unlock(&luna_chip_gphy_lock);
}

/* Release the receive FIFOs. MEASURED 2026-08-24, stock vs ...
 * dev/MEASURED-luna_eth.c.md sec 23. */
static void eth_rxfifo_release(struct luna_eth *ep)
{
	u32 was, now;

	if (!ep->cs.c->cfg_pcsxf || rst_rxfifo < 0)
		return;

	was = sw_rd(ep, ep->cs.c->cfg_pcsxf);
	if (((was >> 10) & 0xf) == (u32)(rst_rxfifo & 0xf)) {
		dev_info(ep->dev,
			 "CFG_PCSXF(%#05x) = %08x already has RST_RXFIFO %#x -- left alone\n",
			 ep->cs.c->cfg_pcsxf, was, rst_rxfifo & 0xf);
		return;
	}
	sw_wr(ep, ep->cs.c->cfg_pcsxf,
	      (was & ~(0xfu << 10)) | ((u32)(rst_rxfifo & 0xf) << 10));
	now = sw_rd(ep, ep->cs.c->cfg_pcsxf);
	dev_info(ep->dev,
		 "CFG_PCSXF(%#05x): RST_RXFIFO %#x -> %#x (%08x -> %08x)\n",
		 ep->cs.c->cfg_pcsxf, (was >> 10) & 0xf, rst_rxfifo & 0xf, was, now);
}

/* Program CFG_PHY_CTRL, and SAY what was there before: the ...
 * dev/MEASURED-luna_eth.c.md sec 24. */
static int eth_phy_ctrl_apply__locked(struct luna_eth *ep)
{
	u32 was, back;

	lockdep_assert_held(&luna_chip_gphy_lock);
	if (!ep->cs.c->cfg_phy_ctrl || base_phyad < 0)
		return 0;

	was = sw_rd(ep, ep->cs.c->cfg_phy_ctrl);
	if ((was & 0x1f) == (u32)(base_phyad & 0x1f)) {
		dev_info(ep->dev,
			 "CFG_PHY_CTRL(%#05x) = %08x already carries BASE_PHYAD %u -- left alone\n",
			 ep->cs.c->cfg_phy_ctrl, was, base_phyad & 0x1f);
		return 0;
	}
	/* ADMISSION: the window must be idle before the address moves. */
	if (gphy_wait(ep)) {
		dev_err(ep->dev,
			"CFG_PHY_CTRL(%#05x): the indirect window never cleared BUSY, so BASE_PHYAD %u -> %u is NOT applied -- retargeting it now would re-point a transaction already in flight\n",
			ep->cs.c->cfg_phy_ctrl, was & 0x1f, base_phyad & 0x1f);
		return -ETIMEDOUT;
	}
	sw_wr(ep, ep->cs.c->cfg_phy_ctrl, (was & ~0x1fu) | (base_phyad & 0x1f));
	back = sw_rd(ep, ep->cs.c->cfg_phy_ctrl);
	if ((back & 0x1f) != (u32)(base_phyad & 0x1f)) {
		dev_err(ep->dev,
			"CFG_PHY_CTRL(%#05x): BASE_PHYAD %u did NOT take -- it reads %08x, so every PHY address after this would be wrong\n",
			ep->cs.c->cfg_phy_ctrl, base_phyad & 0x1f, back);
		return -EIO;
	}
	dev_info(ep->dev,
		 "CFG_PHY_CTRL(%#05x): BASE_PHYAD %u -> %u (%08x -> %08x)\n",
		 ep->cs.c->cfg_phy_ctrl, was & 0x1f, base_phyad & 0x1f, was, back);
	return 0;
}

static int eth_phy_ctrl_apply(struct luna_eth *ep)
{
	int rc;

	mutex_lock(&luna_chip_gphy_lock);
	rc = eth_phy_ctrl_apply__locked(ep);
	mutex_unlock(&luna_chip_gphy_lock);
	return rc;
}

#if IS_ENABLED(CONFIG_LUNA_DEBUG)
/* Writing this re-applies BASE_PHYAD and dumps the survey again, so a whole
 * sweep costs one boot instead of one build each. */
static int resurvey_set(const char *val, const struct kernel_param *kp)
{
	struct luna_eth *ep;
	int rc;

	/* ⚠ A MODULE-PARAMETER WRITER CAN ARRIVE AT ANY MOMENT, ...
	 * dev/MEASURED-luna_eth.c.md sec 25. */
	mutex_lock(&luna_chip_gphy_lock);
	ep = survey_ep;
	if (!ep) {
		mutex_unlock(&luna_chip_gphy_lock);
		return -ENODEV;
	}
	rc = eth_phy_ctrl_apply__locked(ep);
	if (!rc)
		eth_phy_survey__locked(ep);
	mutex_unlock(&luna_chip_gphy_lock);
	return rc;
}
#endif
/* Clear the diagnostic's subject.  Called from ndo_stop and, as the lifetime
 * guarantee, from a devres action that runs before devres frees @ep. */
static void luna_eth_survey_withdraw(void *cookie)
{
	struct luna_eth *ep = cookie;

	mutex_lock(&luna_chip_gphy_lock);
	if (survey_ep == ep)
		survey_ep = NULL;
	mutex_unlock(&luna_chip_gphy_lock);
}

#if IS_ENABLED(CONFIG_LUNA_DEBUG)
static const struct kernel_param_ops resurvey_ops = { .set = resurvey_set };
module_param_cb(resurvey, &resurvey_ops, NULL, 0200);
MODULE_PARM_DESC(resurvey, "write anything: re-apply base_phyad and re-dump the PHY survey");
#endif

/* The three public accessors: each takes the indirect window for the WHOLE
 * command/data sequence, and nothing inside takes it again. */
static int gphy_read(struct luna_eth *ep, unsigned int phyad, unsigned int reg,
		     u16 *out)
{
	int rc;

	mutex_lock(&luna_chip_gphy_lock);
	rc = gphy_read__locked(ep, phyad, reg, out);
	mutex_unlock(&luna_chip_gphy_lock);
	return rc;
}

/* Copper ports this package's strap disables (capability word 0x23024). */
static u32 luna_gphy_strap_off(struct luna_eth *ep)
{
	return (sw_rd(ep, 0x23024) & 0x3f) >> 1;
}

/* The RTL9607C monitor's cold recipe (83c02524 in image 66ef3f7b), also
 * measured from a NAND cold boot on 2026-10-05. TFTP had hidden this missing
 * initialization. The done bit belongs LAST, after the patch and MIIM writes. */
static int eth_copper_cold_init__locked(struct luna_eth *ep)
{
	void __iomem *status;
	u32 disabled, ini;
	unsigned int p, i;
	int rc;

	lockdep_assert_held(&luna_chip_gphy_lock);
	if (!ep->cs.c->cold_phy_status)
		return 0;
	status = (void __iomem *)(uintptr_t)ep->cs.c->cold_phy_status;
	if (readl(status) & BIT(0))
		return 0;
	for (i = 0; i < 1000; i++) {
		if (readl(status) & BIT(1))
			break;
		udelay(100);
	}
	if (i == 1000) {
		dev_err(ep->dev, "cold PHY: SoC ready timed out; no PHY writes\n");
		return -ETIMEDOUT;
	}
	rc = gphy_wait(ep);
	if (rc) {
		dev_err(ep->dev, "cold PHY: MIIM window busy (%d); no PHY writes\n", rc);
		return rc;
	}
	disabled = luna_gphy_strap_off(ep);
	ini = sw_rd(ep, ep->cs.c->cfg_phy_ini);
	sw_wr(ep, ep->cs.c->cfg_phy_ini, (ini & ~(0x1fu << 10)) | (disabled << 10));
	if ((sw_rd(ep, ep->cs.c->cfg_phy_ini) & (0x1fu << 10)) != (disabled << 10)) {
		dev_err(ep->dev, "cold PHY: strap-derived port mask did not take\n");
		return -EIO;
	}
	sw_wr(ep, ep->cs.c->sw_map->gphy_misc, 1);
	if (!(sw_rd(ep, ep->cs.c->sw_map->gphy_misc) & BIT(0))) {
		dev_err(ep->dev, "cold PHY: patch flag did not take\n");
		return -EIO;
	}
	msleep(800);
	p = ep->cs.c->sw_map->cpu_port;
	sw_wr(ep, SW_FORCE_ABLTY(ep, p), 0x96);
	sw_wr(ep, SW_ABLTY_FORCE(ep, p), 0xffff);
	/* Bit 14 is reserved and reads zero on this die. */
	if ((sw_rd(ep, SW_FORCE_ABLTY(ep, p)) & ABLTY_CPU_FORCE) != 0x96 ||
	    (sw_rd(ep, SW_ABLTY_FORCE(ep, p)) & ABLTY_CPU_FORCE) != ABLTY_CPU_FORCE) {
		dev_err(ep->dev, "cold PHY: CPU port force did not take\n");
		return -EIO;
	}
	for (p = 0; p < ep->cs.c->sw_map->n_copper; p++) {
		u16 bmcr = 0x1140;

		if (disabled & BIT(p))
			continue;
		if (luna_uni_port_locked(p))
			bmcr |= BMCR_PDOWN;
		rc = gphy_write_ocp__locked(ep, p, gphy_ocp_miim(ep, p, 0), bmcr);
		if (rc) {
			dev_err(ep->dev, "cold PHY: port %u write failed (%d); done not published\n", p, rc);
			return rc;
		}
	}
	sw_wr(ep, ep->cs.c->sw_map->gphy_misc, 1);
	if (!(sw_rd(ep, ep->cs.c->sw_map->gphy_misc) & BIT(0))) {
		dev_err(ep->dev, "cold PHY: final patch flag did not take\n");
		return -EIO;
	}
	writel(readl(status) | BIT(0), status);
	if (!(readl(status) & BIT(0))) {
		dev_err(ep->dev, "cold PHY: completion bit did not take\n");
		return -EIO;
	}
	/* The cold PHYs expose the programmed BMCR only after completion. */
	for (p = 0; p < ep->cs.c->sw_map->n_copper; p++) {
		u16 bmcr, expected = 0x1140 | (luna_uni_port_locked(p) ? BMCR_PDOWN : 0);

		if (disabled & BIT(p))
			continue;
		rc = gphy_read_ocp__locked(ep, p, gphy_ocp_miim(ep, p, 0), &bmcr);
		if (rc) {
			dev_err(ep->dev, "cold PHY: port %u readback failed after completion (%d)\n", p, rc);
			return rc;
		}
		if (bmcr != expected) {
			dev_err(ep->dev, "cold PHY: port %u BMCR %04x, expected %04x after completion\n",
				p, bmcr, expected);
			return -EIO;
		}
	}
	return 1;
}

#if IS_ENABLED(CONFIG_LUNA_GPHY_RTL9607C_REVB)
/* RTL9607C rev B GPHY analog bring-up, in the stock order and on all five PHYs:
 * PHY parameters, EN70 calibration, MDI amplitude, then the optional 220908
 * common-mode patch. The two PHY-MCU programs are firmware files: EN70 is
 * extracted from the UNKNOWN's stock kernel, 220908 is in no stock image of a
 * rev B unit. Each C record is one PHY-SRAM write: address, value. */
#define GPHY_SRAM_ADDR		0xa436
#define GPHY_SRAM_DATA		0xa438
#define GPHY_PATCH_REQ		0xb820	/* bit 4: the PHY MCU takes a patch	*/
#define GPHY_PATCH_RDY		0xb800	/* bit 6: ... and is ready for it	*/
#define GPHY_PATCH_LOCK		0xb82e	/* bit 0				*/
#define GPHY_PATCH_KEY		0x8028	/* SRAM; 0x3700 on this die		*/
#define GPHY_OCP_A432		0xa432	/* bit 6 cleared after 220908; meaning unread */
#define GPHY_PATCH_POLLS	10000	/* x 1 ms, the stock bound		*/
#define GPHY_PORTS		5	/* the stock patches all five, present or not */
#define GPHY_FW_MAGIC		"LUNAGPHY"	/* then a BE count, then BE (OCP, value) */
#define GPHY_FW_HDR		10
#define RTL9607C_EN70_FW	"rtl9607c-revb-en70.fw"
#define RTL9607C_CM220908_FW	"rtl9607c-revb-cm220908.fw"
#define RTL9607C_EFUSE_IND_CMD	0x0001c	/* CMD_EN bit 16 | entry [15:0]	*/
#define RTL9607C_EFUSE_IND_RD	0x00020	/* BUSY bit 16 | data [15:0]	*/
#define RTL9607C_GDAC_IB(speed, p) (0x1f068 + (speed) * 0x14 + (p) * 4)
#define RTL9607C_OTP_EN70_LONG	18	/* minus the port: port 4 reads 14	*/
#define RTL9607C_OTP_EN70_SHORT	23	/* minus the port: port 4 reads 19	*/

static bool gphy_cm220908;
module_param(gphy_cm220908, bool, 0444);
MODULE_PARM_DESC(gphy_cm220908, "rev B GPHY: also load " RTL9607C_CM220908_FW
		 " (default off: no stock image of a rev B unit carries it)");

static const u16 rtl9607c_revb_phy[][2] = {
	{0x809a, 0x7db4}, {0x809c, 0xb507}, {0x809e, 0x6106}, {0x80a0, 0x0980},
	{0x80a2, 0x6019}, {0x80a4, 0x9a0c}, {0x80ac, 0x1a84}, {0x80ae, 0x3607},
	{0x80b0, 0x2106}, {0x80b2, 0x0a6f}, {0x80b4, 0x3d19}, {0x80b6, 0x800c},
	{0x8088, 0xa08b}, {0x808a, 0x4709}, {0x808c, 0xab08}, {0x808e, 0x20b4},
	{0x8090, 0xc41f}, {0x8092, 0x801e}, {0x8065, 0x1b33}, {0x8069, 0x0822},
	{0x805d, 0x0911}, {0x8061, 0x0820}, {0x804d, 0x2444}, {0x8051, 0x0823},
	{0x8160, 0x3727}, {0x8162, 0xc487}, {0x8164, 0x2f84}, {0x8166, 0xeca5},
	{0x8174, 0x04bd}, {0x8176, 0xe9e5}, {0x8178, 0x5ed9}, {0x817a, 0x348b},
	{0x8211, 0x13d6}, {0x8213, 0xd5de}, {0x8215, 0x3fb3}, {0x8217, 0xcf97},
};

static const u32 rtl9607c_revb_gdac[3] = { 0x2ff00, 0x2aa00, 0x2aa00 };	/* 10/100/1000M */

static int gphy_sram_write__locked(struct luna_eth *ep, unsigned int p,
				   const u16 rec[][2], unsigned int n)
{
	unsigned int i;
	int rc = 0;

	for (i = 0; !rc && i < n; i++) {
		rc = gphy_write_ocp__locked(ep, p, GPHY_SRAM_ADDR, rec[i][0]);
		if (!rc)
			rc = gphy_write_ocp__locked(ep, p, GPHY_SRAM_DATA, rec[i][1]);
	}
	return rc;
}

static u16 gphy_fw_count(const struct firmware *fw)
{
	return (fw->data[8] << 8) | fw->data[9];
}

static u16 gphy_fw_word(const struct firmware *fw, unsigned int i, unsigned int half)
{
	const u8 *b = fw->data + GPHY_FW_HDR + 4 * i + 2 * half;

	return (b[0] << 8) | b[1];
}

/* Refused whole unless every record writes the SRAM window. */
static int gphy_fw_load(struct luna_eth *ep, const char *name, const struct firmware **fw)
{
	unsigned int i, n;
	int rc = request_firmware_direct(fw, name, ep->dev);

	if (rc) {
		dev_err(ep->dev, "gphy rev B: %s not loaded (%d): no analog patch\n", name, rc);
		*fw = NULL;
		return rc;
	}
	n = (*fw)->size >= GPHY_FW_HDR ? gphy_fw_count(*fw) : 0;
	rc = n && (*fw)->size == GPHY_FW_HDR + 4 * n &&
	     !memcmp((*fw)->data, GPHY_FW_MAGIC, 8) ? 0 : -EINVAL;
	for (i = 0; !rc && i < n; i++)
		if (gphy_fw_word(*fw, i, 0) != GPHY_SRAM_ADDR &&
		    gphy_fw_word(*fw, i, 0) != GPHY_SRAM_DATA)
			rc = -EINVAL;
	if (rc) {
		dev_err(ep->dev, "gphy rev B: %s malformed (%zu bytes): no analog patch\n",
			name, (*fw)->size);
		release_firmware(*fw);
		*fw = NULL;
	}
	return rc;
}

static int gphy_fw_write__locked(struct luna_eth *ep, unsigned int p, const struct firmware *fw)
{
	unsigned int i, n = gphy_fw_count(fw);
	int rc = 0;

	for (i = 0; !rc && i < n; i++)
		rc = gphy_write_ocp__locked(ep, p, gphy_fw_word(fw, i, 0), gphy_fw_word(fw, i, 1));
	return rc;
}

static int gphy_modify__locked(struct luna_eth *ep, unsigned int p, u32 ocp,
			       u16 clear, u16 set)
{
	u16 v;
	int rc = gphy_read_ocp__locked(ep, p, ocp, &v);

	return rc ? rc : gphy_write_ocp__locked(ep, p, ocp, (v & ~clear) | set);
}

static int gphy_patch_wait__locked(struct luna_eth *ep, unsigned int p, bool ready)
{
	unsigned int i;
	u16 v;
	int rc;

	for (i = 0; i < GPHY_PATCH_POLLS; i++) {
		rc = gphy_read_ocp__locked(ep, p, GPHY_PATCH_RDY, &v);
		if (rc)
			return rc;
		if (!!(v & BIT(6)) == ready)
			return 0;
		usleep_range(1000, 1100);
	}
	return -ETIMEDOUT;
}

static int gphy_patch_open__locked(struct luna_eth *ep, unsigned int p)
{
	static const u16 key[][2] = { {GPHY_PATCH_KEY, 0x3700} };
	int rc = gphy_modify__locked(ep, p, GPHY_PATCH_REQ, 0, BIT(4));

	if (!rc)
		rc = gphy_patch_wait__locked(ep, p, true);
	return rc ? rc : gphy_sram_write__locked(ep, p, key, 1);
}

/* Leave no PHY in patch mode: the lock bit first, then the request. */
static void gphy_patch_abort__locked(struct luna_eth *ep)
{
	unsigned int p;
	u16 lock, req;
	int rc;

	for (p = 0; p < GPHY_PORTS; p++) {
		rc = gphy_read_ocp__locked(ep, p, GPHY_PATCH_LOCK, &lock);
		if (!rc && (lock & BIT(0)))
			rc = gphy_write_ocp__locked(ep, p, GPHY_PATCH_LOCK, lock & ~BIT(0));
		if (!rc)
			rc = gphy_read_ocp__locked(ep, p, GPHY_PATCH_REQ, &req);
		if (!rc && (req & BIT(4)))
			rc = gphy_write_ocp__locked(ep, p, GPHY_PATCH_REQ, req & ~BIT(4));
		if (rc)
			dev_err(ep->dev, "gphy rev B: port %u may be left in patch mode (%d)\n", p, rc);
	}
}

static int rtl9607c_otp_read(struct luna_eth *ep, u16 entry, u16 *out)
{
	unsigned int i;
	u32 v;

	sw_wr(ep, RTL9607C_EFUSE_IND_CMD, BIT(16) | entry);
	for (i = 0; i < 1000; i++) {
		v = sw_rd(ep, RTL9607C_EFUSE_IND_RD);
		if (!(v & BIT(16))) {
			*out = v & 0xffff;
			return 0;
		}
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int rtl9607c_revb_en70__locked(struct luna_eth *ep, unsigned int p,
				      const struct firmware *fw, u16 len_long, u16 len_short)
{
	const u16 lengths[][2] = {
		{0x8777, 0x4600},	/* cable-length threshold */
		{0x8778, len_long}, {0x877a, len_short}, {0x877c, len_short}, {0x877e, len_short},
	};
	int rc = gphy_patch_open__locked(ep, p);

	if (!rc)
		rc = gphy_modify__locked(ep, p, GPHY_PATCH_LOCK, 0, BIT(0));
	if (!rc)
		rc = gphy_fw_write__locked(ep, p, fw);
	if (!rc)
		rc = gphy_sram_write__locked(ep, p, lengths, ARRAY_SIZE(lengths));
	if (!rc)
		rc = gphy_modify__locked(ep, p, GPHY_PATCH_LOCK, BIT(0), 0);
	return rc ? rc : gphy_write_ocp__locked(ep, p, GPHY_PATCH_REQ, 0);
}

static int rtl9607c_revb_cm220908__locked(struct luna_eth *ep, unsigned int p,
					  const struct firmware *fw)
{
	static const u16 start[][2] = { {GPHY_PATCH_LOCK, 0x0001}, {GPHY_PATCH_REQ, 0x0090} };
	static const u16 end[][2] = { {GPHY_PATCH_REQ, 0x0000} };
	int rc = gphy_patch_open__locked(ep, p);

	if (!rc)
		rc = gphy_sram_write__locked(ep, p, start, ARRAY_SIZE(start));
	if (!rc)
		rc = gphy_fw_write__locked(ep, p, fw);
	if (!rc)
		rc = gphy_sram_write__locked(ep, p, end, ARRAY_SIZE(end));
	if (!rc)
		rc = gphy_patch_wait__locked(ep, p, false);
	return rc ? rc : gphy_modify__locked(ep, p, GPHY_OCP_A432, BIT(6), 0);
}

/* Stage-major like the stock; EN70 runs from the highest port down. The OTP
 * lengths are read first; any failure takes every PHY out of patch mode. */
static int rtl9607c_revb_patch__locked(struct luna_eth *ep, const struct firmware *en70,
				       const struct firmware *cm)
{
	u16 len_long[GPHY_PORTS], len_short[GPHY_PORTS];
	const char *stage = "EN70 OTP read";
	unsigned int p, s;
	int rc;

	for (p = 0; p < GPHY_PORTS; p++) {
		rc = rtl9607c_otp_read(ep, RTL9607C_OTP_EN70_LONG - p, &len_long[p]);
		if (!rc)
			rc = rtl9607c_otp_read(ep, RTL9607C_OTP_EN70_SHORT - p, &len_short[p]);
		if (rc)
			goto fail;
	}
	stage = "PHY parameters";
	for (p = 0; p < GPHY_PORTS; p++) {
		rc = gphy_sram_write__locked(ep, p, rtl9607c_revb_phy, ARRAY_SIZE(rtl9607c_revb_phy));
		if (rc)
			goto fail;
	}
	stage = "EN70 calibration";
	for (p = GPHY_PORTS; p-- > 0;) {
		rc = rtl9607c_revb_en70__locked(ep, p, en70, len_long[p], len_short[p]);
		if (rc)
			goto fail;
	}
	for (p = 0; p < GPHY_PORTS; p++)
		for (s = 0; s < ARRAY_SIZE(rtl9607c_revb_gdac); s++)
			sw_wr(ep, RTL9607C_GDAC_IB(s, p), rtl9607c_revb_gdac[s]);
	stage = "common-mode 220908";
	for (p = 0; cm && p < GPHY_PORTS; p++) {
		rc = rtl9607c_revb_cm220908__locked(ep, p, cm);
		if (rc)
			goto fail;
	}
	return 0;
fail:
	dev_err(ep->dev, "gphy rev B: port %u %s failed (%d)\n", p, stage, rc);
	gphy_patch_abort__locked(ep);
	return rc;
}

/* Patch with the present PHYs powered down, as the stock does, then give each
 * back its BMCR. A missing firmware or a failed patch leaves the PHYs unpatched
 * but the LAN up; a BMCR write that does not land fails the bring-up. */
static int rtl9607c_revb_analog_patch__locked(struct luna_eth *ep)
{
	const struct firmware *en70 = NULL, *cm = NULL;
	u32 model, present, down = 0;
	u16 bmcr[GPHY_PORTS] = { 0 };
	unsigned int p;
	int rev, rc = 0;

	model = sw_rd(ep, LUNA_MODEL_NAME_INFO - SWCORE_PHYS);
	rev = luna_rtl9607c_rev(model);
	if (rev != ep->cs.c->gphy_patch_rev) {
		dev_info(ep->dev, "gphy: MODEL_NAME_INFO %08x (rev %d): no GPHY analog patch\n",
			 model, rev);
		return 0;
	}
	if (gphy_fw_load(ep, RTL9607C_EN70_FW, &en70) ||
	    (gphy_cm220908 && gphy_fw_load(ep, RTL9607C_CM220908_FW, &cm)))
		goto out;
	present = GENMASK(ep->cs.c->sw_map->n_copper - 1, 0) & ~luna_gphy_strap_off(ep) &
		  GENMASK(GPHY_PORTS - 1, 0);
	for (p = 0; !rc && p < GPHY_PORTS; p++) {
		if (!(present & BIT(p)))
			continue;
		rc = gphy_read_ocp__locked(ep, p, gphy_ocp_miim(ep, p, MII_BMCR), &bmcr[p]);
		if (!rc)
			rc = gphy_write_ocp__locked(ep, p, gphy_ocp_miim(ep, p, MII_BMCR),
						    bmcr[p] | BMCR_PDOWN);
		if (!rc)
			down |= BIT(p);
	}
	if (rc)
		dev_err(ep->dev, "gphy rev B: power-down failed (%d): no analog patch\n", rc);
	else if (!rtl9607c_revb_patch__locked(ep, en70, cm))
		dev_info(ep->dev, "gphy rev B: analog patch applied%s (MODEL_NAME_INFO %08x)\n",
			 cm ? " with 220908" : "", model);
	for (p = 0; p < GPHY_PORTS; p++) {
		u16 v = bmcr[p];
		int err;

		if (!(down & BIT(p)))
			continue;
		if ((v & BMCR_ANENABLE) && !(v & BMCR_PDOWN))
			v |= BMCR_ANRESTART;
		err = gphy_write_ocp__locked(ep, p, gphy_ocp_miim(ep, p, MII_BMCR), v);
		if (err) {
			dev_err(ep->dev, "gphy rev B: port %u BMCR restore did not land (%d): its power state is UNKNOWN\n",
				p, err);
			rc = err;
		}
	}
out:
	release_firmware(cm);
	release_firmware(en70);
	return rc;
}
#endif

static int eth_gphy_analog_patch__locked(struct luna_eth *ep)
{
	if (!ep->cs.c->gphy_patch_rev)
		return 0;
#if IS_ENABLED(CONFIG_LUNA_GPHY_RTL9607C_REVB)
	return rtl9607c_revb_analog_patch__locked(ep);
#else
	dev_info(ep->dev, "gphy: rev B analog patch not built (CONFIG_LUNA_GPHY_RTL9607C_REVB)\n");
	return 0;
#endif
}

/* No gphy_write() wrapper: its only writer today is the ...
 * dev/MEASURED-luna_eth.c.md sec 26. */
static int eth_copper_phy_up(struct luna_eth *ep)
{
	unsigned int p;
	int rc = 0;

	/* ★ ONE ACQUISITION FOR THE WHOLE SEQUENCE, not just the BMCR ...
	 * dev/MEASURED-luna_eth.c.md sec 59. */
	mutex_lock(&luna_chip_gphy_lock);
	rc = eth_copper_cold_init__locked(ep);
	if (rc) {
		if (rc > 0)
			rc = eth_gphy_analog_patch__locked(ep);
		goto out;
	}

	/* Save CFG_PHY_INI across the GPHY reset and put it back. The ...
	 * dev/MEASURED-luna_eth.c.md sec 27. */
	{
		u32 phy_ini = ep->cs.c->cfg_phy_ini
			      ? sw_rd(ep, ep->cs.c->cfg_phy_ini) : 0;

		/* The vendor NEVER asserts this, and we re-init nothing after ...
		 * dev/MEASURED-luna_eth.c.md sec 28. */
		if (gphy_reset) {
			sw_or(ep, ep->cs.c->swcore_rst, CMD_GPHY_RST_PS);
			msleep(50);
		}

		if (ep->cs.c->cfg_phy_ini) {
			u32 after_rst = sw_rd(ep, ep->cs.c->cfg_phy_ini);

			sw_wr(ep, ep->cs.c->cfg_phy_ini, phy_ini);
			dev_info(ep->dev,
				 "phy power: CFG_PHY_INI(%#05x) %08x -> %08x across the GPHY reset, restored to %08x (read back %08x)\n",
				 ep->cs.c->cfg_phy_ini, phy_ini, after_rst,
				 phy_ini, sw_rd(ep, ep->cs.c->cfg_phy_ini));
		}
	}

	/* The FE auto-poller is OFF at reset on the chips that have one, and that
	 * is a "configured OK but dead" trap: with it stopped the switch never
	 * learns FE link state, so a socket trains a good link and the switch
	 * forwards nothing through it. The chip's own boot loader clears it. */
	if (ep->cs.c->fephy_poll)
		sw_wr(ep, ep->cs.c->fephy_poll,
		      sw_rd(ep, ep->cs.c->fephy_poll) & ~FEPHY_STOP_POLL);

	/* The UNI decision is made inside the same acquisition: read outside and
	 * an administrative lock landing in between is undone by this write. */
	for (p = 0; p < ep->cs.c->sw_map->n_copper; p++) {
		u16 bmcr;

		/* ★★ A FAILED READ WRITES NOTHING.  With the old sentinel this
		 *    loop read 0xffff, modified it and wrote it back -- setting
		 *    loopback, isolate and reset on a port whose bus was merely
		 *    slow to answer. */
		rc = gphy_read__locked(ep, p, 0, &bmcr);
		if (rc) {
			dev_err(ep->dev,
				"gphy: port %u BMCR could not be read (%d) -- nothing is written back and this bring-up FAILS\n",
				p, rc);
			break;
		}
		if (luna_uni_port_locked(p)) {
			/* the OLT has this UNI locked: it stays powered down,
			 * and auto-negotiation is NOT restarted under it */
			bmcr |= BMCR_PDOWN;
			bmcr &= ~BMCR_ANRESTART;
		} else {
			bmcr &= ~BMCR_PDOWN;		/* leave power-down	*/
			bmcr |= BMCR_ANENABLE | BMCR_ANRESTART;	/* + restart	*/
		}
		rc = gphy_write__locked(ep, p, 0, bmcr);
		if (rc) {
			dev_err(ep->dev,
				"gphy: port %u BMCR write did not land (%d); its power state is UNKNOWN, not assumed\n",
				p, rc);
			break;
		}
	}
	/* ★★ AND A LOCKED PORT IS RE-APPLIED THROUGH THE ...
	 * dev/MEASURED-luna_eth.c.md sec 29. */
	for (p = 0; !rc && p < ep->cs.c->sw_map->n_copper; p++) {
		u32 ocp = gphy_ocp_miim(ep, p, 0);
		u16 bmcr;

		if (!luna_uni_port_locked(p))
			continue;
		/* ⚠ A VALUE THAT COULD NOT BE READ IS ITS OWN ERROR, and it is
		 *   reported separately: printing "reads %04x" for a read that
		 *   never happened puts a number nobody measured in the log. */
		rc = gphy_read_ocp__locked(ep, p, ocp, &bmcr);
		if (!rc && !(bmcr & BMCR_PDOWN))
			rc = gphy_write_ocp__locked(ep, p, ocp,
						    (u16)(bmcr | BMCR_PDOWN));
		if (!rc)
			rc = gphy_read_ocp__locked(ep, p, ocp, &bmcr);
		if (rc) {
			dev_err(ep->dev,
				"gphy: port %u is administratively LOCKED and its administrative-map BMCR could not be read or written (%d) -- its power state is UNKNOWN, not assumed\n",
				p, rc);
			break;
		}
		/* ★ A DESIRED MASK IS NOT PROOF THE PHY STAYED DOWN. */
		if (!(bmcr & BMCR_PDOWN)) {
			dev_err(ep->dev,
				"gphy: port %u is administratively LOCKED and its administrative-map BMCR reads %04x -- the power-down did not hold across the bring-up; refusing to bring the interface up over it\n",
				p, bmcr);
			rc = -EIO;
			break;
		}
	}
	if (rc)
		goto out;
	sw_or(ep, ep->cs.c->sw_map->gphy_misc, BIT(0));		/* patch-done sticky	*/

	/* The RTL9603CVD's own U-Boot sets the same patch-done bit and then waits
	 * 800 ms before touching the PHYs again; we set it and carried straight
	 * on. Default 0 is today's behaviour deliberately -- this lands with the
	 * survey, and changing two things at once makes one boot say nothing. */
	if (phy_settle_ms > 0)
		msleep(phy_settle_ms);
	rc = eth_gphy_analog_patch__locked(ep);
out:
	/* ONE acquisition, ONE release: the patch-done sticky and the settle are
	 * part of the same sequence, and handing the window back between them
	 * would let a diagnostic read a PHY mid-bring-up. */
	mutex_unlock(&luna_chip_gphy_lock);
	return rc;
}

static int eth_port_gate__locked(struct luna_eth *ep, unsigned int port, bool down,
				 const char *who, u16 *bmcr_back, u32 *mode_back)
{
	u32 force, mode, back;
	u16 bmcr, want;
	int rc;

	force = sw_rd(ep, SW_FORCE_ABLTY(ep, port));
	mode = sw_rd(ep, SW_ABLTY_FORCE(ep, port));
	if (down) {
		sw_wr(ep, SW_FORCE_ABLTY(ep, port), force & ~ABLTY_LINK);
		sw_wr(ep, SW_ABLTY_FORCE(ep, port), mode | ABLTY_LINK);
	} else {
		sw_wr(ep, SW_ABLTY_FORCE(ep, port), mode & ~ABLTY_LINK);
	}
	back = sw_rd(ep, SW_ABLTY_FORCE(ep, port));
	force = sw_rd(ep, SW_FORCE_ABLTY(ep, port));
	/* BOTH words, because the claim is two gates: the MODE word says the
	 * LINK ability is forced, and the VALUE word says what it is forced TO.
	 * Checking only the first would report a lock that forces the link UP. */
	if (!(back & ABLTY_LINK) != !down ||
	    (down && (force & ABLTY_LINK))) {
		dev_err(ep->dev,
			"%s port %u: the MAC gate did not take -- FORCE_P_ABLTY %08x, ABLTY_FORCE_MODE %08x, so the port is NOT gated\n",
			who, port, force, back);
		return -EIO;
	}

	rc = gphy_read_ocp__locked(ep, port, gphy_ocp_miim(ep, port, 0), &bmcr);
	if (rc) {
		dev_err(ep->dev,
			"%s port %u: its BMCR could not be read (%d); nothing is written back and the state is owed\n",
			who, port, rc);
		return rc;
	}
	/* ★ ONLY THE POWER BIT MOVES. This board's own phyPowerDown ...
	 * dev/MEASURED-luna_eth.c.md sec 31. */
	want = (u16)(down ? (bmcr | BMCR_PDOWN) : (bmcr & ~BMCR_PDOWN));
	rc = gphy_write_ocp__locked(ep, port, gphy_ocp_miim(ep, port, 0), want);
	if (!rc)
		rc = gphy_read_ocp__locked(ep, port,
					   gphy_ocp_miim(ep, port, 0), &bmcr);
	if (!rc && (!(bmcr & BMCR_PDOWN) != !down))
		rc = -EIO;
	if (rc) {
		dev_err(ep->dev,
			"%s port %u: the PHY did not reach %s (%d, BMCR %04x); the gate is owed\n",
			who, port, down ? "power-down" : "power-up", rc, bmcr);
		return rc;
	}
	*mode_back = back;
	*bmcr_back = bmcr;
	return 0;
}

/* Release the external RTL8221B 2.5G PHY from reset (active-low). This alone does
 * not bring up its SerDes link (HiSGMII mode + analog patch is a separate, larger
 * sequence) but lets the PHY run; combined with the boot-loader-warmed SerDes it
 * gives the host's 2.5G port a chance to stay up. */
static void eth_rtl8221b_reset_release(struct luna_eth *ep)
{
	/* select GPIO function for the pin (bank-1 / pins 32..63 word). */
	sw_or(ep, SW_IO_GPIO_EN_HI, RTL8221B_RST_BIT);
	/* drive it as an output and pulse reset: assert (low) then release (high). */
	writel(readl(SOC_GPIO_B1_DIR) | RTL8221B_RST_BIT, SOC_GPIO_B1_DIR);
	writel(readl(SOC_GPIO_B1_DAT) & ~RTL8221B_RST_BIT, SOC_GPIO_B1_DAT);
	msleep(10);
	writel(readl(SOC_GPIO_B1_DAT) | RTL8221B_RST_BIT, SOC_GPIO_B1_DAT);
	msleep(10);
}

/* ---- per-port real-link + RX-MIB diagnostic ------------------------------- */
static u32 eth_mib_rx_pkts(struct luna_eth *ep, unsigned int p)
{
	return sw_rd(ep, SW_MIB_RX_UCAST(p)) + sw_rd(ep, SW_MIB_RX_MCAST(p)) +
	       sw_rd(ep, SW_MIB_RX_BCAST(p));
}

/* Genuine link (independent of the MAC force): copper = MDIO BMSR bit2,
 * SerDes (ports 6,7) = SDS_FIB_STATUS bit4. Returns -1 for ports with no
 * directly-readable PHY/SerDes (5 PON, 8 RGMII, 9 CPU, 10, 11). */
static int eth_port_real_link(struct luna_eth *ep, unsigned int p)
{
	if (p >= ep->cs.c->sw_map->n_copper && p != 6 && p != 7)
		return -1;
	if (p < ep->cs.c->sw_map->n_copper) {
		u16 bmsr;

		/* -1 is this function's COULD NOT ASK, which it already uses for
		 * a port it cannot reach.  A bus timeout is that, never "down". */
		if (gphy_read(ep, p, 1, &bmsr))
			return -1;
		return !!(bmsr & BMSR_LSTATUS);		/* BMSR */
	}
	if (!ep->cs.c->sds_fib_status)		/* no SerDes on this chip */
		return -1;
	return !!(sw_rd(ep, SW_SDS_FIB_STATUS(ep, p - 6)) & SDS_LINK_OK);
}

static void eth_diag_dump(struct luna_eth *ep)
{
	unsigned int p;

	for (p = 0; p <= ep->cs.c->last_port; p++) {
		int link = eth_port_real_link(ep, p);
		u32 ablty = sw_rd(ep, SW_P_ABLTY(ep, p));
		const char *ls = link < 0 ? "n/a  " : (link ? "UP   " : "down ");

		dev_info(ep->dev, "  port %2u: link=%s spdcode=%u stp=%u rxpkts=%u (ablty=%04x)\n",
			 p, ls, (ablty & 3) | (((ablty >> 12) & 3) << 2),
			 sw_rd(ep, SW_MSTI_CTRL(ep, p)) & STP_STATE_MASK,
			 eth_mib_rx_pkts(ep, p), ablty);
	}
	if (ep->cs.c->sds_fib_status)
		dev_info(ep->dev, "  serdes6 fib=%08x serdes7 fib=%08x\n",
			 sw_rd(ep, SW_SDS_FIB_STATUS(ep, 0)),
			 sw_rd(ep, SW_SDS_FIB_STATUS(ep, 1)));
}

/* ★★ THE DIAGNOSTIC'S PHY READS RUN IN A WORK ITEM, NOT THE ...
 * dev/MEASURED-luna_eth.c.md sec 32. */
static void eth_diag_work(struct work_struct *w)
{
	struct luna_eth *ep = container_of(w, struct luna_eth, cs.diag_work);

	dev_info(ep->dev, "link/rxpkts diag (%d left):\n", ep->cs.diag_left);
	eth_diag_dump(ep);
}

static void eth_diag_timer(struct timer_list *t)
{
	struct luna_eth *ep = timer_container_of(ep, t, cs.diag);

	schedule_work(&ep->cs.diag_work);
	/* the RE-ARM stays here, so the work never arms the timer and a
	 * timer_delete_sync() followed by a cancel_work_sync() is enough */
	if (--ep->cs.diag_left > 0 && diag_ms)
		mod_timer(&ep->cs.diag, jiffies + msecs_to_jiffies(diag_ms));
}

/* ---- switch open-L2 bring-up (ordered; see file header) ------------------- */
#define LUT_BC_TRAP	1	/* LUT_BC_BEHAVE action: trap to the CPU */

static void sw_port_act(struct luna_eth *ep, u32 base, unsigned int port, u32 act)
{
	u32 reg = sw_packed_off(base, port, SW_DA_ACT_BITS);

	sw_wr(ep, reg, sw_packed_ins(sw_rd(ep, reg), port, SW_DA_ACT_BITS, act));
}

/* The PON is a routed WAN, never a bridge leg: a frame from it that misses the lookup (unknown
 * unicast -- the switch never learns gpon0's own MAC --, unknown multicast, broadcast) goes to
 * the CPU alone instead of flooding the LAN. Per ingress port, so the flow engine's LAN<->PON
 * NAT is untouched, which port isolation would block (G24W 2026-10-31). */
static void eth_pon_miss_to_cpu(struct luna_eth *ep)
{
	const struct luna_sw_map *m = ep->cs.c->sw_map;

	sw_port_act(ep, m->lut_unkn_uc_da, m->pon_port, LUT_MISS_TRAP);
	luna_pon_mc_admit(ep->sw, m, false);	/* no LUT entry per joined group on these dies yet */
	if (m->bc_behave)
		sw_port_act(ep, m->bc_behave, m->pon_port, LUT_BC_TRAP);
}

static int eth_switch_init(struct luna_eth *ep)
{
	unsigned int p;

	/* 1. enable the switch IP block (SoC control, not in SWCORE). */
	writel(readl(SOC_SW_ENABLE) | SW_EN_BIT | SW_PBO_BIT, SOC_SW_ENABLE);

	/* 1b. SoC handshake, where the chip declares one: the switch block asks
	 *     to be told the SoC is ready before it will accept its patches. The
	 *     chip's own boot loader spins on the same bit; a chip whose table
	 *     leaves it 0 simply has no such handshake. */
	if (ep->cs.c->sys_status) {
		void __iomem *ss = (void __iomem *)(uintptr_t)ep->cs.c->sys_status;
		int i;

		for (i = 0; i < 1000 && !(readl(ss) & BIT(1)); i++)
			udelay(100);
		if (!(readl(ss) & BIT(1)))
			dev_warn(ep->dev,
				 "switch: SoC never reported ready-for-patch (SYS_STATUS %08x after 100 ms) -- continuing, but the PHY patches may not stick\n",
				 readl(ss));
		writel(readl(ss) | BIT(0), ss);		/* soc_init_rdy */
	}

	/* 2. re-assert the SerDes egress line mode (boot-ROM leaves ...
	 * dev/MEASURED-luna_eth.c.md sec 60. */
	if (ep->cs.c->serdes_linemode)
		sw_wr(ep, ep->cs.c->serdes_linemode, 0x44);

	/* 3. bring the physical PHYs up so a host on a jack actually ...
	 * dev/MEASURED-luna_eth.c.md sec 34. */
	{
		int rc = eth_phy_ctrl_apply(ep);

		if (rc)
			return rc;
	}
	eth_rxfifo_release(ep);
	if (copper_phy) {
		int rc = eth_copper_phy_up(ep);

		if (rc)
			return rc;	/* the PHYs ARE the LAN: do not claim them up */
	}
	/* AFTER the power-up, so the survey reads the PHYs in the state the rest
	 * of the bring-up will actually see -- not the pre-power-up one, which
	 * would answer a question nobody asked. */
	if (phy_survey)
		eth_phy_survey(ep);
	/* The external 2.5G PHY hangs off the SerDes uplink, so it exists only on
	 * a chip that HAS one. Asking for it elsewhere would drive a GPIO chosen
	 * for a different board. */
	if (rtl8221b_phy && ep->cs.c->sds_fib_status)
		eth_rtl8221b_reset_release(ep);

	/* 4. open the L2 forwarding plane. ★ THE PACKED ARITHMETIC IS ...
	 * dev/MEASURED-luna_eth.c.md sec 35. */
	for (p = 0; p <= ep->cs.c->sw_map->cpu_port; p++) {
		u32 reg = sw_packed_off(ep->cs.c->sw_map->lut_unkn_sa, p,
					SW_DA_ACT_BITS);

		/* unknown-source-MAC action 0 = learn + forward */
		sw_wr(ep, reg, sw_packed_ins(sw_rd(ep, reg), p,
					     SW_DA_ACT_BITS, 0));
	}
	/* The PON port is not a flood destination (2026-08-27): ...
	 * dev/MEASURED-luna_eth.c.md sec 36. */
	{
		u32 flood = ep->cs.c->sw_map->port_mask, keep = ~flood;

		if (ep->cs.c->force_pon_ablty)
			flood &= ~BIT(ep->cs.c->sw_map->pon_port);
		/* the port field written whole: the reset value already holds the PON bit, an OR kept it */
		sw_wr(ep, ep->cs.c->sw_map->bc_flood, (sw_rd(ep, ep->cs.c->sw_map->bc_flood) & keep) | flood);
		sw_wr(ep, ep->cs.c->sw_map->unkn_mc_flood, (sw_rd(ep, ep->cs.c->sw_map->unkn_mc_flood) & keep) | flood);
		sw_wr(ep, ep->cs.c->sw_map->unkn_uc_flood, (sw_rd(ep, ep->cs.c->sw_map->unkn_uc_flood) & keep) | flood);
		ep->cs.lan_flood_mask = flood & ~BIT(ep->cs.c->sw_map->cpu_port);
	}
	/* 0 is the RESET value and is written only because it is: forcing every
	 * bit moved no witness on either Luna die (luna_eth_regs.h @src_permit). */
	sw_wr(ep, ep->cs.c->sw_map->src_permit, 0x00000000);

	/* 4a2. per-port unknown-DA lookup-miss action = FORWARD(0). We only set the
	 *      unknown-SOURCE action above; the unknown-DESTINATION action must also
	 *      forward, else post-ARP unicast / IPv6-ND to a not-yet-learned MAC is
	 *      dropped instead of flooded. Clear the 2-bit field for ports 0..10. */
	sw_wr(ep, ep->cs.c->sw_map->lut_unkn_uc_da,  sw_rd(ep, ep->cs.c->sw_map->lut_unkn_uc_da)  & ~DA_ACT_PORTS);
	sw_wr(ep, ep->cs.c->sw_map->unkn_l2_mc,  sw_rd(ep, ep->cs.c->sw_map->unkn_l2_mc)  & ~DA_ACT_PORTS);
	sw_wr(ep, ep->cs.c->sw_map->unkn_ip4_mc, sw_rd(ep, ep->cs.c->sw_map->unkn_ip4_mc) & ~DA_ACT_PORTS);
	sw_wr(ep, ep->cs.c->sw_map->unkn_ip6_mc, sw_rd(ep, ep->cs.c->sw_map->unkn_ip6_mc) & ~DA_ACT_PORTS);
	eth_pon_miss_to_cpu(ep);
	if (ep->cs.c->sw_map->l2_cfg)
		sw_wr(ep, ep->cs.c->sw_map->l2_cfg,
		      (sw_rd(ep, ep->cs.c->sw_map->l2_cfg) & ~GENMASK(2, 0)) | CTL_RX_PRIO);
	if (ep->cs.c->sw_map->ctl_acl) {
		int rc = luna_ctl_acl_install(ep->sw, ep->cs.c->sw_map->ctl_acl, CTL_RX_PRIO);

		if (rc)
			dev_err(ep->dev, "control ACL not installed (%d): ARP shares the data ring under a flood\n", rc);
	}

	/* 4a3. VLAN must not gate CPU<->LAN egress. The boot loader can leave VLAN
	 *      filtering ON with the CPU port outside the member set, which silently
	 *      drops the CPU's reply to the host (RX fixed, but ping return blocked).
	 *      Report the live state, then disable filtering for flat-L2 forwarding. */
	{
		u32 vc = sw_rd(ep, SW_VLAN_CTRL);

		sw_wr(ep, SW_VLAN_CTRL, vc & ~VLAN_FILTERING);
		dev_info(ep->dev, "vlan_ctrl %08x (filtering %s) -> %08x\n",
			 vc, (vc & VLAN_FILTERING) ? "ON" : "off",
			 sw_rd(ep, SW_VLAN_CTRL));
	}

	/* 4b. set every port's spanning-tree state to FORWARDING. The ...
	 * dev/MEASURED-luna_eth.c.md sec 61. */
	for (p = 0; p <= ep->cs.c->last_port; p++) {
		u32 v = sw_rd(ep, SW_MSTI_CTRL(ep, p));

		sw_wr(ep, SW_MSTI_CTRL(ep, p),
		      (v & ~STP_STATE_MASK) | STP_FORWARDING);
	}

	/* 4c. (optional) drop the CPU port from its own egress flood ...
	 * dev/MEASURED-luna_eth.c.md sec 62. */
	if (cpu_no_loopback) {
		unsigned int cp = ep->cs.c->sw_map->cpu_port;
		unsigned int per = ep->cs.c->piso_per_word;
		u32 reg = ep->cs.c->sw_map->piso_base + (cp / per) * 4;
		unsigned int shift = (cp % per) * ep->cs.c->piso_bits;
		u32 fld = ep->cs.c->piso_all & ~BIT(cp);

		sw_wr(ep, reg,
		      (sw_rd(ep, reg) & ~(ep->cs.c->piso_all << shift)) |
		      (fld << shift));
	}

	/* 5. force every port's MAC link up (no PHY autoneg). The CPU ...
	 * dev/MEASURED-luna_eth.c.md sec 63. */
	for (p = 0; p <= ep->cs.c->last_port; p++) {
		if (p == ep->cs.c->sw_map->cpu_port) {
			sw_or(ep, SW_FORCE_ABLTY(ep, p), BIT(4));
			sw_wr(ep, SW_ABLTY_FORCE(ep, p), ABLTY_CPU_FORCE);
		} else {
			/* Force NOTHING on a UTP port. This used to write ...
			 * dev/MEASURED-luna_eth.c.md sec 37. */
			continue;
		}
	}
	if (ep->cs.c->force_ablty_x) {
		sw_wr(ep, ep->cs.c->force_ablty_x, ABLTY_1G_FULL_LINK);
		sw_wr(ep, ep->cs.c->ablty_force_x, ABLTY_FORCE_ALL);
	}

	/* 5b. The PON port is not a UTP port and the rule above does ...
	 * dev/MEASURED-luna_eth.c.md sec 38. */
	if (ep->cs.c->force_pon_ablty) {
		unsigned int pp = ep->cs.c->sw_map->pon_port;
		u32 was = sw_rd(ep, SW_FORCE_ABLTY(ep, pp));

		sw_wr(ep, SW_FORCE_ABLTY(ep, pp), ABLTY_1G_FULL_LINK);
		sw_wr(ep, SW_ABLTY_FORCE(ep, pp), ABLTY_CPU_FORCE);
		dev_info(ep->dev,
			 "switch: PON port %u forced up like the CPU port (stock's own pair): FORCE_P_ABLTY %08x -> %08x, ABLTY_FORCE_MODE -> %08x, P_ABLTY reads %08x\n",
			 pp, was, sw_rd(ep, SW_FORCE_ABLTY(ep, pp)),
			 sw_rd(ep, SW_ABLTY_FORCE(ep, pp)),
			 sw_rd(ep, SW_P_ABLTY(ep, pp)));
	}

	/* 6. CPU-tag engine. The switch frames RX to the CPU with the 0x8899 tag
	 *    by default, so the CPU port is tag-aware; keep it on unless asked. */
	if (sw_cpu_tag) {
		sw_or(ep, ep->cs.c->cpu_tag_insert, BIT(0));
		sw_or(ep, ep->cs.c->cpu_tag_aware, BIT(0));
	}

	/* 7. Control frames (OMCI, OAM, IGMP) bypass ingress rate and flow control,
	 *    as stock leaves it: a policed port must not starve them. */
	if (ep->cs.c->igr_bwctrl_glb)
		sw_or(ep, ep->cs.c->igr_bwctrl_glb, BIT(0));

	dev_info(ep->dev,
		 "switch: %s open-L2 up (cpu-port %u of 0..%u, src-permit=%08x, cpu-ablty=%04x)\n",
		 ep->cs.c->name, ep->cs.c->sw_map->cpu_port, ep->cs.c->last_port,
		 sw_rd(ep, ep->cs.c->sw_map->src_permit),
		 sw_rd(ep, SW_P_ABLTY(ep, ep->cs.c->sw_map->cpu_port)));
	/* Baseline real-link snapshot; the periodic diag (armed at open) then shows
	 * which port's genuine link comes up + rxpkts climb under host traffic. */
	eth_diag_dump(ep);
	return 0;
}

static int luna_chip_port_gate__locked(struct luna_eth *ep, unsigned int port, bool down, bool mac_suffices,
				       const char *who, u16 *bmcr_back, u32 *mode_back)
{
	/* No PHY power guard on these dies: the MAC and the PHY gate together, @mac_suffices has no case. */
	return eth_port_gate__locked(ep, port, down, who, bmcr_back, mode_back);
}

static int luna_chip_sw_bringup(struct luna_eth *ep)
{
	int ret = eth_switch_init(ep);

	if (ret)
		return ret;
	if (diag_ms && diag_count > 0) {
		ep->cs.diag_left = diag_count;
		mod_timer(&ep->cs.diag, jiffies + msecs_to_jiffies(diag_ms));
	}
	/* The resurvey knob gets its subject once the bring-up that gives the reading a meaning is done, under
	 * the window's own lock, which keeps a concurrent writer out of a half-built state. */
	mutex_lock(&luna_chip_gphy_lock);
	survey_ep = ep;
	mutex_unlock(&luna_chip_gphy_lock);
	return 0;
}

/* eth_switch_init() in the bring-up already programmed every flood and VLAN this die needs. */
static void luna_chip_sw_min_init(struct luna_eth *ep)
{
}

static void luna_chip_cpu_tag_arm(struct luna_eth *ep)
{
	/* the MAC strips the in-band switch tag on RX; the IP-block reset clears it, so every program re-asserts */
	ep_wr(ep, R_CPUTAGCR, CPUTAGCR_INIT);
	ep_wr(ep, R_CPUTAG1CR, CPUTAG1CR_SID(ep->cs.omci_sid));
}

static void luna_chip_omci_sid(struct luna_eth *ep, unsigned int sid)
{
	u32 v = ep_rd(ep, R_CPUTAG1CR);

	ep->cs.omci_sid = sid;
	ep_wr(ep, R_CPUTAG1CR, (v & ~CPUTAG1CR_SID_MASK) | CPUTAG1CR_SID(sid));
	netdev_info(ep->ndev, "OMCI trap armed: OMCC sid %u, DS cpu-tag reason %u, PON port %u, CPUTAG1CR 0x%08x -> 0x%08x\n",
		    sid, ep->swm->omci_cpu_reason, ep->swm->pon_port, v, ep_rd(ep, R_CPUTAG1CR));
}

static u32 luna_chip_pon_txd_word2(struct luna_eth *ep, u32 ovr)
{
	return ovr ? ovr : luna_gmac_cputag_txd_word2(ep->swm->pon_port);
}

static u32 luna_chip_omci_txd_word3(struct luna_eth *ep, u32 ovr)
{
	return ovr ? ovr : luna_gmac_cputag_txd_word3(ep->cs.omci_sid);
}

static u32 luna_chip_wan_txd_word3(struct luna_eth *ep, unsigned int flow)
{
	return luna_gmac_cputag_txd_word3(flow);
}

/* One line per DISTINCT cpu-tag reason, ingress port and PON stream, ever: what tells a WAN that counts nothing
 * apart from an OLT that sends nothing. */
static void luna_chip_rx_seen(struct luna_eth *ep, const struct rx_desc *d, unsigned int len)
{
	const struct luna_rx_layout *rxl = ep->swm->rx_layout;
	unsigned int rsn = (d->opts2 >> 21) & 0xff;
	unsigned int sp = luna_gmac_rx_src_port(rxl, d->opts3);

	if (!(ep->cs.rx_reason_seen[rsn >> 5] & BIT(rsn & 31))) {
		ep->cs.rx_reason_seen[rsn >> 5] |= BIT(rsn & 31);
		netdev_info(ep->ndev, "rx cpu-tag reason %u seen (opts2=%08x opts3=%08x frame=%u); this chip traps DS OMCI as %u\n",
			    rsn, d->opts2, d->opts3, len, ep->swm->omci_cpu_reason);
	}
	if (sp < 16 && !(ep->cs.rx_src_port_seen & BIT(sp))) {
		ep->cs.rx_src_port_seen |= BIT(sp);
		netdev_info(ep->ndev, "rx ingress port %u seen (opts3=%08x frame=%u); this chip's fibre port is %u, CPU port %u\n",
			    sp, d->opts3, len, ep->swm->pon_port, ep->swm->cpu_port);
	}
	/* the stream id is extspa on a frame from anywhere but the fibre */
	if (sp == ep->swm->pon_port) {
		unsigned int sid = luna_gmac_rx_pon_sid(d->opts3);

		if (!(ep->cs.rx_pon_sid_seen[sid >> 5] & BIT(sid & 31))) {
			ep->cs.rx_pon_sid_seen[sid >> 5] |= BIT(sid & 31);
			netdev_info(ep->ndev, "rx PON stream %u seen (opts3=%08x frame=%u); OMCC is %u, WAN data is %u, multicast is %u\n",
				    sid, d->opts3, len, ep->cs.omci_sid, GPON_DATA_FLOW, GPON_MCAST_FLOW);
		}
	}
}

/* The CPU port frames a packet as [DA][SA][switch tag][ethertype][payload]: excise the 8-byte 0x8899 tag by
 * sliding DA+SA over it, then drop our own egress flooded back to the CPU port (a LAN loop's witness). */
static bool luna_chip_rx_consume(struct luna_eth *ep, struct sk_buff *skb, struct net_device *rdev, u32 opts3)
{
	if (luna_gmac_rx_cpu_tag_present(skb->data, skb->len, RTL_CPU_TAG_LEN)) {
		memmove(skb->data + RTL_CPU_TAG_LEN, skb->data, 2 * ETH_ALEN);
		skb_pull(skb, RTL_CPU_TAG_LEN);
	}
	if (skb->len < 2 * ETH_ALEN || !ether_addr_equal(skb->data + ETH_ALEN, rdev->dev_addr))
		return false;
	if (rdev == ep->ndev)
		luna_loop_rx(ep->loop, skb->data, skb->len, luna_gmac_rx_src_port(ep->swm->rx_layout, opts3));
	dev_kfree_skb_any(skb);
	return true;
}

/* Only the fibre port: on these dies fibre-ingress frames carry the PON port (G24W descriptors, 2026-09-11), the
 * RTL9602C drain signature is not a fact of this silicon, and a destination-MAC match would make a LAN spoof WAN
 * ingress (gpon_eth_path_diff_test). */
static bool luna_chip_rx_is_wan(struct luna_eth *ep, u32 opts3, const u8 *dst)
{
	return luna_gmac_rx_is_wan(ep->swm->rx_layout, opts3, ep->swm->pon_port);
}

/* The opts1 checksum bits are proven on the RTL9602C only (the RX descriptor layout differs per die): no RX
 * checksum offload here until a live read proves them. */
static const struct eth_rx_csum_layout *const luna_chip_rx_csum = NULL;

/* A CPU flood goes out to an explicit egress mask, so the switch does not flood it back at the CPU. */
static u32 luna_chip_lan_txd_word2(struct luna_eth *ep, const struct sk_buff *skb)
{
	if (lan_flood_direct && ep->cs.lan_flood_mask && is_multicast_ether_addr(skb->data))
		return luna_gmac_cputag_txd_pmask(ep->cs.lan_flood_mask);
	return 0;
}

static unsigned int luna_chip_sw_blocks(void *ctx, struct eth_state_block *b)
{
	struct luna_eth *ep = ctx;
	unsigned int n = 0;

	eth_state_add(b, &n, "nic", false, luna_state_reg_rd, ep->base, luna_state_nic, 8, NULL);
	eth_state_add(b, &n, "nic status", true, luna_state_reg_rd, ep->base, luna_state_nic_status, 8, NULL);
	return n;
}

/* No switch soft reset on these rows (the recovery refuses), so the flow engine's tables only move under rtnl
 * and luna_flow_mutex: nothing more to hold here. */
static void luna_chip_offload_lock(struct luna_eth *ep)
{
}

static void luna_chip_offload_unlock(struct luna_eth *ep)
{
}

static void luna_chip_offload_assert_held(struct luna_eth *ep)
{
}

static void luna_chip_offload_probe(struct luna_eth *ep, struct device *dev)
{
	int ret = luna_flow_probe(ep);

	if (ret)
		dev_warn(dev, "hardware flow offload unavailable: %d\n", ret);
	if (ep->cs.flow) {
		ep->ndev->hw_features |= NETIF_F_HW_TC;
		ep->ndev->features |= NETIF_F_HW_TC;
	}
}

static void luna_chip_wan_mac(struct luna_eth *ep, u8 *out, const u8 *base)
{
	gpon_hwaddr_derive(out, base, ep->cs.c->wan_mac_offset);
}

static void luna_chip_wan_pin(struct luna_eth *ep, const u8 *mac)
{
	if (!mac)
		luna_flow_stop(ep);
	luna_flow_wan_pin(ep, mac);
}

static void luna_chip_stop(struct luna_eth *ep)
{
	luna_flow_stop(ep);
}

#ifdef CONFIG_GPON_FLOW_OFFLOAD
static int luna_chip_setup_tc(struct net_device *dev, enum tc_setup_type type, void *type_data)
{
	return luna_flow_setup_tc(dev, type, type_data);
}
#endif

static int luna_chip_probe(struct luna_eth *ep, struct platform_device *pdev)
{
	ep->cs.c = of_device_get_match_data(&pdev->dev);
	if (!ep->cs.c)
		return -ENODEV;
	ep->swm = ep->cs.c->sw_map;
	timer_setup(&ep->cs.diag, eth_diag_timer, 0);
	INIT_WORK(&ep->cs.diag_work, eth_diag_work);
	/* devres releases in REVERSE order: this runs after the netdev is unregistered and before @ep is freed */
	return devm_add_action_or_reset(&pdev->dev, luna_eth_survey_withdraw, ep);
}

static const struct of_device_id luna_eth_of_match[] = {
	{ .compatible = "realtek,rtl9607c-nic",   .data = &luna_chip_rtl9607c },
	{ .compatible = "realtek,rtl9603cvd-nic", .data = &luna_chip_rtl9603cvd },
	{ }
};
MODULE_DEVICE_TABLE(of, luna_eth_of_match);

static struct platform_driver luna_eth_driver = {
	.probe	= luna_eth_probe,
	.driver	= {
		.name		= "luna-eth",	/* the family's name: this TU serves two dies */
		.of_match_table	= luna_eth_of_match,
	},
};
module_platform_driver(luna_eth_driver);

MODULE_DESCRIPTION("Realtek Luna (RTL9607C / RTL9603CVD) Ethernet driver");
MODULE_LICENSE("GPL");
