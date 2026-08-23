// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek Luna (MIPS interAptiv) GMAC0 + on-chip switch — eth0.
 *
 * Clean-room driver for the SoC's CPU-port Gigabit MAC (the "GMAC0" engine at
 * phys 0x18012000) and a minimal open-L2 bring-up of the on-chip switch core
 * (phys 0x1b000000). The CPU reaches the LAN only through the switch:
 *
 *	CPU <-> GMAC0 (eth0) <-> switch CPU-port <-> physical LAN port <-> wire
 *
 * ★★ WHY THIS FILE IS NAMED rtl960x_* AND NOT rtl9607c_*.
 * It serves TWO chips, and the project's naming rule is explicit: family-shared
 * code may not hide behind one chip's name, because the next human or LLM has to
 * land on the work BY NAME. It was renamed the day the second chip was proven to
 * use it, not later. The chip-specific half is a TABLE (`struct luna_eth_chip`),
 * never an `#ifdef` and never a second copy of the file — a duplicated driver is
 * how a repair lands on one board and not the others.
 *
 *	RTL9607C   engineering board, 11 switch ports, 3 CPU GMACs, SerDes uplink
 *	RTL9603CVD LANLY G24W,         6 switch ports, 1 CPU GMAC,  no SerDes
 *
 * ★ THE MAC ENGINE IS IDENTICAL ON BOTH; THE SWITCH REGISTERS ARE NOT.
 * The vendor compiles ONE NIC source for both parts, from one header, and takes
 * the register base from its device tree — so base, every MAC offset, both
 * descriptor layouts, the interrupt bits and the DMA model carry over unchanged
 * [tier 3, the chips' own 4.4.140 SDK]. The SWITCH is a different story: of 1361
 * name-matched switch registers, 853 keep their offset and **508 move**. The
 * ability block is the trap that pays for this table — the RTL9607C's `P_ABLTY`
 * at 0x200 is `SDS_CFG` on the RTL9603CVD, so a driver that "just worked
 * because the family is the same" would be reading a SerDes configuration word
 * and calling it a link state.
 *
 * Switch bring-up facts (behavioural, established on the RTL9607C):
 *  - The CPU port is this GMAC; the LAN copper ports and the fibre/PON port are
 *    per chip (see the table).
 *  - ★ THE LOAD-BEARING RX GATE IS THE PER-PORT SPANNING-TREE STATE
 *    (`MSTI_CTRL`), not a permit mask. An earlier version of this comment said
 *    the bootloader leaves a "source permit" mask cleared and that opening it
 *    was the fix — that is RETRACTED. `SRC_PORT_PERMIT` is a per-source-port
 *    egress-FILTER ENABLE whose forwarding-permissive value is 0 (its own reset
 *    value); writing all-ones there silently dropped every LAN->CPU frame.
 *  - ⚠ AND THE SILICON IS NOT THE ONE THAT BLOCKS: `MSTI_CTRL` resets to
 *    0x000000FF on BOTH chips, i.e. every port FORWARDING. If ports are found
 *    non-forwarding at probe, that is the BOOT LOADER's doing, not a reset
 *    state — so the write below is a correction, not an initialisation, and it
 *    must stay even if a future bootloader stops needing it.
 *  - The integrated copper PHYs need no analog calibration on either chip; the
 *    deterministic path is MAC-force-link every port plus open-L2 flood.
 *  - The ordered sequence matters: replaying only the final register values is
 *    not sufficient on this hardware.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/timer.h>

/* ---- bring-up knobs (live-tunable; the datapath framing is HW-uncertain on
 * first contact, so expose the few values most likely to need a tweak) ------ */
static int rx_prefix = 2;
module_param(rx_prefix, int, 0644);
MODULE_PARM_DESC(rx_prefix, "bytes the CPU-port prepends ahead of each RX frame (stripped)");

static unsigned int backstop_ms = 10;
module_param(backstop_ms, uint, 0644);
MODULE_PARM_DESC(backstop_ms, "RX/TX drain backstop poll period (catches a missed IRQ)");

static bool sw_cpu_tag;
module_param(sw_cpu_tag, bool, 0644);
MODULE_PARM_DESC(sw_cpu_tag, "enable the switch CPU-port tag engine (default off: plain-L2 forwarding)");

static int rx_dump = 6;
module_param(rx_dump, int, 0644);
MODULE_PARM_DESC(rx_dump, "hex-dump the first N received frames (bring-up framing check)");

static int tx_dump = 6;
module_param(tx_dump, int, 0644);
MODULE_PARM_DESC(tx_dump, "hex-dump the first N transmitted frames + descriptors");

static bool copper_phy = true;
module_param(copper_phy, bool, 0644);
MODULE_PARM_DESC(copper_phy, "power up + auto-neg the internal copper PHYs (ports 0-4)");

static bool rtl8221b_phy = true;
module_param(rtl8221b_phy, bool, 0644);
MODULE_PARM_DESC(rtl8221b_phy, "de-assert the RTL8221B 2.5G PHY reset (SerDes-6 uplink)");

static unsigned int diag_ms = 3000;
module_param(diag_ms, uint, 0644);
MODULE_PARM_DESC(diag_ms, "period of the per-port real-link/rxpkts diagnostic (0 = off)");

static int diag_count = 12;
module_param(diag_count, int, 0644);
MODULE_PARM_DESC(diag_count, "number of periodic link/rxpkts diagnostic dumps");

static int gphy_map;
module_param(gphy_map, int, 0644);
MODULE_PARM_DESC(gphy_map, "internal-PHY OCP map: 0 = per the chip table, 1 = force GPHY page 0xA40 on every port, 2 = force the flat FE map (a bring-up experiment: the vendor SDK and its own OCP map disagree for the FE ports)");

static int phy_settle_ms;
module_param(phy_settle_ms, int, 0644);
MODULE_PARM_DESC(phy_settle_ms, "ms to wait after the PHY patch-done bit (0 = current behaviour; the RTL9603CVD's own U-Boot waits 800)");

static bool phy_survey = true;
module_param(phy_survey, bool, 0644);
MODULE_PARM_DESC(phy_survey, "at open, READ each copper port's BMCR/BMSR under BOTH OCP maps and dump them (read-only; turns a 3-boot experiment into a 1-boot one)");

static bool cpu_no_loopback = true;
module_param(cpu_no_loopback, bool, 0644);
MODULE_PARM_DESC(cpu_no_loopback, "drop the CPU port from its own egress flood (stops self-loopback RX)");

/* ---- GMAC0 register block (offsets from the DT reg base 0x18012000) -------- */
#define R_IDR0		0x00	/* station MAC [0:3], MSB first			*/
#define R_IDR4		0x04	/* station MAC [4:5] in [31:16]			*/
#define R_MAR0		0x08	/* multicast hash [31:0]				*/
#define R_MAR4		0x0C	/* multicast hash [63:32]			*/
#define R_CMD		0x3B	/* 8-bit command: bit0 RST			*/
#define   CMD_RXCHK	0x02	/* RX checksum offload				*/
#define   CMD_RXJUMBO	0x08	/* accept jumbo					*/
#define R_IMR		0x3C	/* 16-bit RX/TX interrupt mask			*/
#define R_ISR		0x3E	/* 16-bit RX/TX interrupt status (W1C)		*/
#define R_TCR		0x40	/* TX control					*/
#define R_RCR		0x44	/* RX control (bit0 = accept-all-physical)	*/
#define R_CPUTAGCR	0x48	/* CPU-tag insert config			*/
#define R_CONFIG	0x4C
#define R_CPUTAG1CR	0x50
#define R_MSR		0x58	/* media/flow status; top byte = force flow ctl	*/
#define R_IMR0		0xD0	/* 32-bit per-ring TX-completion mask		*/
#define R_ISR1		0xD8	/* 32-bit per-ring TX-completion status (W1C)	*/
#define R_TxFDP0	0x1300	/* TX ring0 fetch-descriptor pointer		*/
#define R_TxCDO0	0x1304	/* TX ring0 current-descriptor offset (u16)	*/
#define R_RRING_ROUTE	0x1370	/* RX class -> ring routing			*/
#define R_RxFDP0	0x13F0	/* RX ring0 fetch-descriptor pointer		*/
#define R_RxCDO0	0x13F4	/* RX ring0: RxCDO[31:16] | RxRingSize[15:8]	*/
				/* 32-BIT register. The (u16) this comment used to carry
				 * named the FIELD, and an iowrite16 was written to match
				 * it -- see the store below.			*/
#define R_RxDesNum	0x1430	/* RX ring0 size + flow-control thresholds	*/
#define R_IO_CMD	0x1434	/* DMA enable + per-ring TX kick (bit0 = ring0)	*/
#define R_IO_CMD1	0x1438

/* engine enable values (inherited-config + DMA-enable edge) */
#define IO_CMD_ENABLE	0xc059f130
#define IO_CMD1_ENABLE	0x32000001
#define IMR_RX_BITS	0xf835	/* RX-OK + RX-error + ring descriptor-unavailable */
#define IMR0_TX_BITS	0x3f	/* per-ring TX completion				*/

/* CPU-tag engine config. CTEN_RX (bit31) makes the MAC strip the 8-byte switch
 * tag in hardware on RX and expose the parsed ingress port in the descriptor;
 * the rest selects tag sizes, the 0x04 protocol and the 0x8899 match. Clearing
 * CTEN_RX leaves the raw in-band tag in the delivered frame. */
#define CPUTAGCR_INIT	0x9022FF04
#define CPUTAG1CR_INIT	0x00004000	/* CPU-tag SID base (64 << 8)		*/
#define ABLTY_CPU_FORCE	0xBFFF		/* CPU-port forced-ability mode (keep)	*/

/* descriptor flags (word0 / opts1, both rings) */
#define D_OWN		BIT(31)	/* 1 = owned by the DMA engine			*/
#define D_EOR		BIT(30)	/* end of ring (wrap)				*/
#define D_FS		BIT(29)	/* first segment				*/
#define D_LS		BIT(28)	/* last segment					*/
#define D_TXCRC		BIT(23)	/* TX: append FCS				*/
#define RXD_CRCERR	BIT(27)
#define RXD_DMAERR	BIT(24)
#define RXD_LEN_MASK	0x1fff
#define TXD_LEN_MASK	0x1ffff

#define RX_RING_SIZE	64
#define TX_RING_SIZE	64
#define RX_BUF_SIZE	2048
#define TH_ON_VAL	0x10	/* RX flow-control assert / de-assert thresholds	*/
#define TH_OFF_VAL	0x30

/* ---- switch core (SWCORE), phys 0x1b000000 ---------------------------------
 * ★ THE BASE IS THE SAME ON BOTH CHIPS AND THAT IS ESTABLISHED, NOT ASSUMED.
 * The RTL9603CVD's own chipdef spells its reset table in ABSOLUTE addresses
 * (0x1b000000 .. 0x1bf15438) where the RTL9607C's spells OFFSETS, and the
 * difference of the two top entries is exactly 0x1b000000. The block layout is
 * otherwise identical. [tier 3, each chip's own SDK]
 *
 * ⚠ THE WINDOW BELOW IS THE UNION AND IT IS DELIBERATELY GENEROUS: the
 * RTL9607C's highest named register sits at 0x42E7C, so the 0x42000 this file
 * used to map was 0xE7C SHORT — a register at the top of the block would have
 * been written into a hole with nothing to read. */
#define SWCORE_PHYS		0x1b000000UL
#define SWCORE_SIZE		0x43000

/* -- offsets that are the SAME on every Luna part covered here -------------- */
#define SW_GPHY_WD		0x00000	/* internal-PHY indirect: write data	*/
#define SW_GPHY_CMD		0x00004	/*   ... command (phy<<16 | ocp)		*/
#define SW_GPHY_RD		0x00008	/*   ... read data + BUSY		*/
#define SW_LUT_UNKN_SA		0x1C004	/* unknown-SA action, 2 bits/port	*/
#define SW_LUT_BC_FLOOD		0x1C028	/* broadcast flood, 1 bit/port		*/
#define SW_LUT_UNKN_MC_FLOOD	0x1C02C
#define SW_LUT_UNKN_UC_FLOOD	0x1C030
#define   STP_STATE_MASK	0x3
#define   STP_FORWARDING	0x3

/* ---------------------------------------------------------------------------
 * ★★ THE PER-CHIP TABLE.  Everything in it MOVED between the two parts, and
 * every field was read from that chip's OWN SDK (tier 3) rather than inferred
 * from the sibling.  A zero means "this chip has no such register" and the code
 * must SKIP the write — never write to offset 0, which on both chips is the
 * PHY indirect-access data register.
 * --------------------------------------------------------------------------- */
struct luna_eth_chip {
	const char *name;

	/* --- switch port map ------------------------------------------------ */
	u8	cpu_port;	/* the port this GMAC is				*/
	u8	pon_port;	/* the fibre port (no copper PHY behind it)	*/
	u8	last_port;	/* highest port to iterate, INCLUSIVE		*/
	u8	n_copper;	/* copper PHY ports, always 0..n_copper-1	*/
	u8	gphy_ports;	/* bitmap: ports whose PHY is a GPHY, not FE	*/
	u32	port_mask;	/* flood/member mask covering 0..last_port	*/

	/* --- switch registers that MOVED ------------------------------------ */
	u32	force_ablty;	/* + 4*port: forced ability values		*/
	u32	p_ablty;	/* + 4*port: LIVE ability, read-only		*/
	u32	ablty_force;	/* + 4*port: which ability fields are forced	*/
	u32	msti_ctrl;	/* + 4*port: per-port spanning-tree state	*/
	u32	src_permit;	/* source-port egress FILTER ENABLE (want 0)	*/
	u32	cpu_tag_insert;
	u32	cpu_tag_aware;
	u32	swcore_rst;	/* swcore soft reset (bit10), excludes cfg	*/
	u32	gphy_misc;	/* WRAP_GPHY_MISC: bit0 = PHY patch done	*/
	u32	fephy_poll;	/* 0 on a chip with no FE-PHY auto-poller	*/
	u32	cfg_phy_ini;	/* per-port PHY enable; U-Boot loads it from efuse*/

	/* --- port-isolation packing, which differs in SHAPE not just offset -- */
	u32	piso_base;
	u8	piso_per_word;	/* how many ports share one 32-bit word		*/
	u8	piso_bits;	/* width of one port's mask			*/
	u32	piso_all;	/* the all-open value for ONE port		*/

	/* --- SerDes uplink: present only on the bigger part ------------------ */
	u32	serdes_linemode;	/* 0 = no SerDes on this chip		*/
	u32	force_ablty_x;		/* SerDes/PBO ability trio, 0 if absent	*/
	u32	p_ablty_x;
	u32	ablty_force_x;
	u32	sds_fib_status;		/* + 0x20*idx, 0 if absent		*/

	/* --- SoC glue OUTSIDE the switch's own register space ---------------- */
	u32	sys_status;	/* 0 = this chip's bring-up does not use it	*/
};

/* ---------------------------------------------------------------------------
 * ★ THE RTL9607C ENTRY IS THIS FILE'S PREVIOUS CONSTANTS, VALUE FOR VALUE.
 * That is deliberate and it is the acceptance test for the refactor: the
 * engineering board that boots today must see a byte-identical register
 * sequence, so any behaviour change on it would be a defect of THIS commit and
 * not of the new chip.
 * --------------------------------------------------------------------------- */
static const struct luna_eth_chip luna_chip_rtl9607c = {
	.name		= "RTL9607C",
	.cpu_port	= 9,
	.pon_port	= 5,
	.last_port	= 11,	/* 0..4,8 copper; 5 PON; 6,7 SerDes; 9 CPU; 11 PBO */
	.n_copper	= 5,	/* ports 0..4 */
	.gphy_ports	= 0x1f,	/* all five copper ports are GPHYs here	*/
	.port_mask	= GENMASK(9, 0),
	.force_ablty	= 0x001CC,
	.p_ablty	= 0x00200,
	.ablty_force	= 0x00238,
	.msti_ctrl	= 0x1704C,
	.src_permit	= 0x1C114,
	.cpu_tag_insert	= 0x230F4,
	.cpu_tag_aware	= 0x230F8,
	.swcore_rst	= 0x00108,
	.gphy_misc	= 0x00114,
	.fephy_poll	= 0,	/* every PHY here is a GPHY: no FE auto-poller	*/
	.cfg_phy_ini	= 0x0004C,
	.piso_base	= 0x27000,
	.piso_per_word	= 1,
	.piso_bits	= 29,
	.piso_all	= 0x1FFFFFFF,
	.serdes_linemode = 0x00084,
	.force_ablty_x	= 0x002F4,
	.p_ablty_x	= 0x002F8,
	.ablty_force_x	= 0x002FC,
	.sds_fib_status	= 0x0028C,
	.sys_status	= 0,
};

/* ---------------------------------------------------------------------------
 * ★★ THE RTL9603CVD ENTRY -- EVERY FIELD READ FROM *THIS CHIP'S OWN* SDK
 * (tier 3), never carried over from the sibling.  The four that matter most,
 * and why an inherited value would have been silently wrong:
 *
 *   msti_ctrl   0x1704C -> 0x1713C.  This is the LOAD-BEARING RX GATE. At the
 *               9607C offset it would write into the middle of a different
 *               block on this chip and every LAN->CPU frame would vanish with
 *               the port counters climbing -- the exact "configured OK but
 *               dead" signature.
 *   p_ablty     0x00200 -> 0x001B8.  ⚠ THE 9607C's 0x200 IS `SDS_CFG` HERE.
 *               A driver that "just worked because the family is the same"
 *               would read a SerDes configuration word and print it as a link
 *               state -- a phantom that reads healthy.
 *   cpu_tag_*   0x230F4/F8 -> 0x2303C/40.  The whole MAC-control block sits
 *               0xB8 lower on this chip, because the 9607C inserts three
 *               CPU_HASH_* registers this part does not have.
 *   piso        same BASE, different SHAPE: 12-bit masks packed TWO PORTS PER
 *               WORD (6 ports + 6 ext), against the 9607C's 29-bit one-per-word
 *               (11 ports + 18 ext). Both tile exactly, which is the
 *               cross-check that makes the packing believable rather than
 *               guessed.
 *
 * ★ AND `fephy_poll` HAS NO 9607C COUNTERPART AT ALL. Its reset value stops
 * the FE-PHY auto-poller, so the switch never learns FE link state until
 * somebody clears bit 16 -- the chip's own U-Boot does exactly that, and a
 * port that "links but forwards nothing" is what forgetting it looks like.
 * --------------------------------------------------------------------------- */
static const struct luna_eth_chip luna_chip_rtl9603cvd = {
	.name		= "RTL9603CVD",
	.cpu_port	= 5,
	.pon_port	= 4,
	.last_port	= 5,	/* 0..2 FE; 3 GE; 4 PON; 5 CPU (6 = PBO loopback)*/
	.n_copper	= 4,	/* ports 0..3					*/
	.gphy_ports	= 0x08,	/* ONLY port 3 is a GPHY; 0..2 are FE PHYs	*/
	.port_mask	= GENMASK(5, 0),
	.force_ablty	= 0x00198,
	.p_ablty	= 0x001B8,
	.ablty_force	= 0x001DC,
	.msti_ctrl	= 0x1713C,
	.src_permit	= 0x1C0B0,
	.cpu_tag_insert	= 0x2303C,
	.cpu_tag_aware	= 0x23040,
	.swcore_rst	= 0x000E0,
	.gphy_misc	= 0x000EC,
	.fephy_poll	= 0x0000C,
	.cfg_phy_ini	= 0x00050,
	.piso_base	= 0x27000,
	.piso_per_word	= 2,
	.piso_bits	= 12,
	.piso_all	= 0xFFF,
	.serdes_linemode = 0,	/* no SerDes on this part			*/
	.force_ablty_x	= 0,
	.p_ablty_x	= 0,
	.ablty_force_x	= 0,
	.sds_fib_status	= 0,
	.sys_status	= 0xB8000044,	/* SoC handshake, outside SWCORE		*/
};

/* Accessors: the table lives in `ep->c`, so a per-port register is one call and
 * a chip that lacks a register is answered with 0 and SKIPPED by the caller. */
#define SW_FORCE_ABLTY(ep, p)	((ep)->c->force_ablty + (p) * 4)
#define SW_P_ABLTY(ep, p)	((ep)->c->p_ablty + (p) * 4)
#define SW_ABLTY_FORCE(ep, p)	((ep)->c->ablty_force + (p) * 4)
#define SW_MSTI_CTRL(ep, p)	((ep)->c->msti_ctrl + (p) * 4)

/* VLAN: filtering must not gate CPU<->LAN egress (the boot loader may leave it on
 * with the CPU port outside the member set). */
#define SW_VLAN_CTRL		0x13008
#define   VLAN_FILTERING	BIT(0)
#define SW_VLAN_ACCEPT		0x13000	/* per-port accept-frame-type (0 = accept all) */
#define SW_VLAN_PB_VID		0x1300C	/* per-port default VID (PVID), stride 4	*/

/* Per-port lookup-miss (unknown-DA) action, 2 bits/port; 0 = FORWARD. Needed for
 * the post-ARP unicast / IPv6-ND path (the first broadcast already floods). */
#define SW_UNKN_UC_DA		0x1C00C
#define SW_UNKN_L2_MC		0x1C018
#define SW_UNKN_IP4_MC		0x1C01C
#define SW_UNKN_IP6_MC		0x1C020
#define   DA_ACT_PORTS		0x3FFFFF	/* ports 0..10, 2 bits each		*/

#define ABLTY_1G_FULL_LINK	0x16	/* speed=1000, duplex=full, link=up	*/
#define ABLTY_FORCE_ALL		0xFFF	/* force all basic abilities		*/

/* ★ THE PORT MAP MOVED INTO THE CHIP TABLE. It used to be these three
 * constants, and they described the RTL9607C only: 11 ports with the CPU at 9.
 * The RTL9603CVD has SIX, with the CPU at 5 and the PON at 4 -- so a loop
 * bounded by 11 there walks four ports that do not exist, reading and writing
 * registers past the end of every per-port array in the block. That is not a
 * harmless over-read: the per-port arrays are adjacent, so port 6..11 of a
 * 6-port chip lands in whatever register file follows.
 * ⚠ ONE MORE TRAP RECORDED SO NOBODY RE-CHASES IT: the vendor NIC header
 * declares a "LAN_PORT5 = 8" for the RTL9603CVD. It is REFUTED -- that chip's
 * own chipdef marks port 8 as RT_PORT_NONE and caps the port space at 0..6,
 * and the board's own boot log only ever uses 0..4. It reads like a copy-paste
 * from the sibling branch below it. DO NOT USE PORT 8 ON THE 9603CVD.
 */

/* switch CPU-port control tag: ethertype 0x8899 followed by 6 control bytes,
 * inserted after the source MAC on frames to/from the CPU. */
#define RTL_CPU_TAG_LEN		8
#define RTL_CPU_TAG_ETYPE	0x8899

/* SoC peripheral control (fixed uncached KSEG1, no ioremap) */
#define SOC_IP_SEL	((void __iomem *)0xb8000600ul)	/* per-engine clock/reset*/
#define   IPSEL_GMAC0	BIT(1)
#define SOC_SW_ENABLE	((void __iomem *)0xb800063cul)	/* switch-core enable	*/
#define   SW_EN_BIT	BIT(5)
#define   SW_PBO_BIT	BIT(25)				/* required on rev > A	*/

/* Internal-PHY indirect window. The register TRIO and its bit layout are the
 * same on both chips (declared at the top with the other shared offsets); the
 * PHYs behind it are addressed by their switch port number, and BUSY lives in
 * the RD register, NOT in CMD.
 *
 * ★ THE OCP ADDRESS OF BMCR IS NOT ONE CONSTANT. A GPHY answers on page 0xA40
 * (BMCR = 0xA400, register N adds (N&7)<<1); an FE PHY has a FLAT map where the
 * OCP address is simply reg<<1, so BMCR = 0x0000. The RTL9607C has only GPHYs,
 * which is why this used to be a single #define.
 * ⚠ AND THE SOURCES DISAGREE FOR THE FE PORTS -- the board must settle it. The
 * RTL9603CVD's own SDK power-up loop uses 0xA400 for ALL of ports 0..5,
 * character-for-character the 9607C's, while that same SDK's OCP map says its
 * FE PHYs are flat. Either the loop was carried over unadapted (so only port 3
 * is really being powered) or the FE PHY aliases. `gphy_map` is a module
 * parameter for exactly that one-boot experiment; the DEFAULT follows the
 * chip table, and 1 reproduces the vendor's own behaviour. */
#define GPHY_MII_PAGE	0xA400		/* GPHY: page 0xA40, register 0			*/
#define   GPHY_CMD_EN	BIT(21)		/* start					*/
#define   GPHY_WREN	BIT(22)		/* write strobe					*/
#define   GPHY_RD_BUSY	BIT(16)		/* in the RD register				*/
#define   GPHY_MACRO_RST	BIT(6)	/* in swcore_rst: GPHY-macro reset	*/
#define   FEPHY_STOP_POLL	BIT(16)	/* in fephy_poll: 1 = auto-poller OFF	*/

/* Live link / speed (genuine, independent of the MAC force). Copper genuine link
 * = MDIO BMSR bit2; SerDes genuine link = SDS_FIB_STATUS. */
#define SW_SDS_FIB_STATUS(ep, s) ((ep)->c->sds_fib_status + (s) * 0x20)
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

/* MII BMCR/BMSR bits. */
#define MII_PDOWN	0x0800
#define MII_ANENABLE	0x1000
#define MII_ANRESTART	0x0200
#define MII_LSTATUS	0x0004

/* The DMA descriptor address bus window (0 on this SoC). */
#define DMA_BUS_WINDOW	0u

struct rx_desc { u32 opts1, addr, opts2, opts3; };
struct tx_desc { u32 opts1, addr, opts2, opts3, opts4; };

struct luna_eth {
	const struct luna_eth_chip *c;	/* THE per-chip table -- never an #ifdef */
	struct net_device	*ndev;
	struct device		*dev;
	void __iomem		*base;	/* GMAC0			*/
	void __iomem		*sw;	/* switch core			*/
	int			irq;

	struct napi_struct	napi;
	struct timer_list	backstop;
	struct timer_list	diag;
	int			diag_left;
	spinlock_t		tx_lock;

	/* Plain streaming DMA: the kernel manages the L2 so dma_map/unmap flush +
	 * invalidate it -- no bounce buffers needed. */
	struct rx_desc		*rx_ring;
	dma_addr_t		rx_ring_dma;
	struct sk_buff		*rx_skb[RX_RING_SIZE];
	dma_addr_t		rx_buf_dma[RX_RING_SIZE];
	unsigned int		rx_head;

	struct tx_desc		*tx_ring;
	dma_addr_t		tx_ring_dma;
	struct sk_buff		*tx_skb[TX_RING_SIZE];
	dma_addr_t		tx_buf_dma[TX_RING_SIZE];
	unsigned int		tx_buf_len[TX_RING_SIZE];
	void			*tx_buf[TX_RING_SIZE];	/* per-slot linear copy buffer */
	unsigned int		tx_head, tx_dirty;	/* free-running counters	*/

	int			rx_dumped;
	int			tx_dumped;
};

static inline u32 ep_rd(struct luna_eth *ep, u32 r) { return ioread32(ep->base + r); }
static inline void ep_wr(struct luna_eth *ep, u32 r, u32 v) { iowrite32(v, ep->base + r); }
static inline u32 sw_rd(struct luna_eth *ep, u32 r) { return ioread32(ep->sw + r); }
static inline void sw_wr(struct luna_eth *ep, u32 r, u32 v) { iowrite32(v, ep->sw + r); }
static inline void sw_or(struct luna_eth *ep, u32 r, u32 v) { sw_wr(ep, r, sw_rd(ep, r) | v); }

static inline unsigned int tx_slot(unsigned int counter) { return counter % TX_RING_SIZE; }

/* ---- internal GPHY MDIO (indirect window) --------------------------------- */
static int gphy_wait(struct luna_eth *ep)
{
	int i;

	for (i = 0; i < 10000; i++) {
		if (!(sw_rd(ep, SW_GPHY_RD) & GPHY_RD_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

/* The OCP address of standard MII register `reg` on the PHY behind switch port
 * `p`. See the note above GPHY_MII_PAGE for why this is not one constant. */
static u32 gphy_ocp(struct luna_eth *ep, unsigned int p, unsigned int reg)
{
	if (gphy_map == 1 || (gphy_map == 0 && (ep->c->gphy_ports & BIT(p))))
		return GPHY_MII_PAGE | ((reg & 7) << 1);	/* GPHY page 0xA40 */
	return (reg & 0x1f) << 1;			/* FE PHY: flat map */
}

/* Read one OCP address on one PHY. The SURVEY needs this because it asks the
 * SAME register through BOTH maps, which `gphy_read()` cannot express: that one
 * consults the chip table (correctly) and so can only ever return one answer. */
static u16 gphy_read_ocp(struct luna_eth *ep, unsigned int phyad, u32 ocp)
{
	sw_wr(ep, SW_GPHY_CMD, (phyad << 16) | ocp | GPHY_CMD_EN);
	if (gphy_wait(ep))
		return 0xffff;
	return sw_rd(ep, SW_GPHY_RD) & 0xffff;
}

static u16 gphy_read(struct luna_eth *ep, unsigned int phyad, unsigned int reg)
{
	u32 adr = (phyad << 16) | gphy_ocp(ep, phyad, reg);

	sw_wr(ep, SW_GPHY_CMD, adr | GPHY_CMD_EN);
	if (gphy_wait(ep)) {
		/* ★ NEVER 0xffff SILENTLY. An all-ones MII read is a PERFECTLY
		 * PLAUSIBLE register value -- BMSR 0xffff reads as "link up, every
		 * ability" -- so returning it for a DEAD BUS manufactures a healthy
		 * answer out of a broken instrument. Say so, rate-limited. */
		dev_warn_ratelimited(ep->dev,
			"gphy: read timeout phy %u reg %u -- the indirect bus never cleared BUSY; the 0xffff returned is NOT a register value\n",
			phyad, reg);
		return 0xffff;
	}
	return sw_rd(ep, SW_GPHY_RD) & 0xffff;
}

static void gphy_write(struct luna_eth *ep, unsigned int phyad,
		       unsigned int reg, u16 val)
{
	u32 adr = (phyad << 16) | gphy_ocp(ep, phyad, reg);

	sw_wr(ep, SW_GPHY_WD, val);
	sw_wr(ep, SW_GPHY_CMD, adr | GPHY_WREN | GPHY_CMD_EN);
	if (gphy_wait(ep))
		dev_warn_ratelimited(ep->dev,
			"gphy: write timeout phy %u reg %u val %04x -- the write may never have landed\n",
			phyad, reg, val);
}

/* Power up + (re)start auto-negotiation on the integrated copper PHYs so a copper
 * LAN jack actually trains. Each PHY is addressed by its switch port number. */
/* ★★ ONE BOOT INSTEAD OF THREE.  The open M2 question is whether this chip's FE
 * PHYs answer on the GPHY page (0xA40, so BMCR = 0xA400) or on a FLAT map
 * (ocp = reg<<1, so BMCR = 0x0000) -- the RTL9603CVD's own SDK power-up loop
 * uses the GPHY page for ALL ports while that same SDK's OCP map says its FE
 * PHYs are flat, and the two cannot both be right.
 *
 * Sweeping `gphy_map` would cost a BOOT PER VALUE. Reading the same two
 * registers through BOTH maps in one pass costs four MDIO reads per port and
 * answers it outright: the map that returns a PLAUSIBLE BMCR/BMSR pair is the
 * map, and if both return 0xffff the answer is that neither is -- which is
 * itself a finding, and points at the patch/settle or the per-port enable
 * rather than at the address.
 *
 * ★ IT IS READ-ONLY AND IT SAYS SO. Nothing here writes a PHY, so it cannot
 * change the outcome it is measuring -- which is the whole reason it can be
 * left on by default on both chips.
 *
 * ★ AND 0xffff IS PRINTED AS WHAT IT IS. An all-ones MDIO read is a perfectly
 * plausible register value (BMSR 0xffff reads as "link up, every ability"), so
 * the dump labels it rather than letting a dead bus look like a healthy PHY.
 */
static void eth_phy_survey(struct luna_eth *ep)
{
	unsigned int p;

	if (ep->c->cfg_phy_ini)
		dev_info(ep->dev,
			 "phy survey: CFG_PHY_INI(%#05x) = %08x  (U-Boot loads its per-port field from the efuse; we do NOT write it -- the polarity is unresolved)\n",
			 ep->c->cfg_phy_ini, sw_rd(ep, ep->c->cfg_phy_ini));

	for (p = 0; p < ep->c->n_copper; p++) {
		u16 g_bmcr = gphy_read_ocp(ep, p, GPHY_MII_PAGE | (0 << 1));
		u16 g_bmsr = gphy_read_ocp(ep, p, GPHY_MII_PAGE | (1 << 1));
		u16 f_bmcr = gphy_read_ocp(ep, p, 0 << 1);
		u16 f_bmsr = gphy_read_ocp(ep, p, 1 << 1);

		dev_info(ep->dev,
			 "phy survey: port %u (%s by table)  gphy-page[bmcr=%04x bmsr=%04x]%s  flat[bmcr=%04x bmsr=%04x]%s\n",
			 p, (ep->c->gphy_ports & BIT(p)) ? "GPHY" : "FE",
			 g_bmcr, g_bmsr,
			 (g_bmcr == 0xffff && g_bmsr == 0xffff) ? " <- ALL-ONES, i.e. no answer" : "",
			 f_bmcr, f_bmsr,
			 (f_bmcr == 0xffff && f_bmsr == 0xffff) ? " <- ALL-ONES, i.e. no answer" : "");
	}
}

static void eth_copper_phy_up(struct luna_eth *ep)
{
	unsigned int p;

	/* ★★★ SAVE CFG_PHY_INI ACROSS THE GPHY-MACRO RESET, AND PUT IT BACK.
	 * MEASURED 2026-08-23 on the G24W, on our own running image:
	 *
	 *     CFG_PHY_INI (SWCORE 0x00050) = 0x00000200
	 *
	 * which is the register's RESET VALUE, i.e. PWRUP[8:5] = 0 -- all four
	 * PHY macros UNPOWERED. That is why every PHY register reads 0000, why
	 * no UTP port links, and why nothing enters the switch.
	 *
	 * It was not always 0. In the SAME boot, minutes earlier, U-Boot moved
	 * 4.1 MB through port 3 by TFTP: the switch's own MIB counted 2823
	 * unicast frames against a 2824-packet transfer, with port 5 carrying
	 * the matching ACKs. The PHY linked, the switch forwarded both ways --
	 * and the counters FREEZE the instant this driver takes over.
	 *
	 * ⚠⚠ THE MECHANISM I EXPECTED IS REFUTED, BY THIS VERY LOG LINE. I
	 * thought the pulse below cleared PWRUP and that nothing restored it.
	 * The first boot with this code printed
	 *
	 *     CFG_PHY_INI(0x050) 00000200 -> 00000200 across the GPHY reset,
	 *                        restored to 00000200 (read back 00000200)
	 *
	 * -- the value was ALREADY 0x200 going in. The reset does not clear it,
	 * and U-Boot leaves it at 0x200 as well. So U-Boot moved 4.1 MB through
	 * these PHYs with PWRUP reading zero: on this board PWRUP[8:5] is NOT
	 * the gate that powers a GPHY, whatever the field name suggests.
	 * ⇒ PWRUP = 0 is NOT the fault, and nobody should re-chase it.
	 *
	 * ★ THE SAVE/RESTORE STAYS ANYWAY, and it is not superstition: it is a
	 * reset-domain re-lock, the shape this project's anti-repeat list names
	 * first, and it costs one read and one write. It copies forward whatever
	 * the bootloader and the efuse agreed on rather than inventing a value,
	 * so it stays correct on a board where the reset DOES clear it. What
	 * earns its keep here is the LOG: it refuted a plausible hypothesis in a
	 * single boot instead of a bisect.
	 *
	 * ★ The vendor never pulses this reset at all in its own switch_init
	 * (tier 2: index 0x000E0 is written only by `chip_reset` and
	 * `ponmac_mode_set`), so not pulsing is the other candidate repair --
	 * kept separate, because one change at a time is what makes a bisect
	 * mean something.
	 */
	{
		u32 phy_ini = ep->c->cfg_phy_ini
			      ? sw_rd(ep, ep->c->cfg_phy_ini) : 0;

		sw_or(ep, ep->c->swcore_rst, GPHY_MACRO_RST);
		msleep(50);

		if (ep->c->cfg_phy_ini) {
			u32 after_rst = sw_rd(ep, ep->c->cfg_phy_ini);

			sw_wr(ep, ep->c->cfg_phy_ini, phy_ini);
			dev_info(ep->dev,
				 "phy power: CFG_PHY_INI(%#05x) %08x -> %08x across the GPHY reset, restored to %08x (read back %08x)\n",
				 ep->c->cfg_phy_ini, phy_ini, after_rst,
				 phy_ini, sw_rd(ep, ep->c->cfg_phy_ini));
		}
	}

	/* ★ THE FE AUTO-POLLER IS OFF AT RESET ON THE CHIPS THAT HAVE ONE, and
	 * that is a "configured OK but dead" trap with the port's own name on
	 * it: with it stopped the switch never learns FE link state, so a socket
	 * trains a perfectly good link and the switch forwards nothing through
	 * it. The chip's own boot loader clears this bit; so do we, and only on
	 * a chip whose table declares the register. */
	if (ep->c->fephy_poll)
		sw_wr(ep, ep->c->fephy_poll,
		      sw_rd(ep, ep->c->fephy_poll) & ~FEPHY_STOP_POLL);

	for (p = 0; p < ep->c->n_copper; p++) {
		u16 bmcr = gphy_read(ep, p, 0);

		bmcr &= ~MII_PDOWN;			/* leave power-down	*/
		bmcr |= MII_ANENABLE | MII_ANRESTART;	/* auto-neg + restart	*/
		gphy_write(ep, p, 0, bmcr);
	}
	sw_or(ep, ep->c->gphy_misc, BIT(0));		/* patch-done sticky	*/

	/* ★ THE SETTLE THIS DRIVER NEVER SPENT. The RTL9603CVD's own U-Boot sets
	 * the same patch-done bit and then waits 800 ms before touching the PHYs
	 * again. We set the bit and carried straight on, which is the shape
	 * CLAUDE.md names outright: a bring-up is an FSM that must RUN and
	 * COMPLETE, not a set of resting values to write.
	 * DEFAULT 0 = today's behaviour, DELIBERATELY: this lands together with
	 * the survey below, and changing two things at once would make one boot
	 * unable to say which of them mattered. Set phy_settle_ms=800 on the boot
	 * that tests it. */
	if (phy_settle_ms > 0)
		msleep(phy_settle_ms);
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
	if (p >= ep->c->n_copper && p != 6 && p != 7)
		return -1;
	if (p < ep->c->n_copper)
		return !!(gphy_read(ep, p, 1) & MII_LSTATUS);	/* BMSR */
	if (!ep->c->sds_fib_status)		/* no SerDes on this chip */
		return -1;
	return !!(sw_rd(ep, SW_SDS_FIB_STATUS(ep, p - 6)) & SDS_LINK_OK);
}

static void eth_diag_dump(struct luna_eth *ep)
{
	unsigned int p;

	for (p = 0; p <= ep->c->last_port; p++) {
		int link = eth_port_real_link(ep, p);
		u32 ablty = sw_rd(ep, SW_P_ABLTY(ep, p));
		const char *ls = link < 0 ? "n/a  " : (link ? "UP   " : "down ");

		dev_info(ep->dev, "  port %2u: link=%s spdcode=%u stp=%u rxpkts=%u (ablty=%04x)\n",
			 p, ls, (ablty & 3) | (((ablty >> 12) & 3) << 2),
			 sw_rd(ep, SW_MSTI_CTRL(ep, p)) & STP_STATE_MASK,
			 eth_mib_rx_pkts(ep, p), ablty);
	}
	if (ep->c->sds_fib_status)
		dev_info(ep->dev, "  serdes6 fib=%08x serdes7 fib=%08x\n",
			 sw_rd(ep, SW_SDS_FIB_STATUS(ep, 0)),
			 sw_rd(ep, SW_SDS_FIB_STATUS(ep, 1)));
}

static void eth_diag_timer(struct timer_list *t)
{
	struct luna_eth *ep = timer_container_of(ep, t, diag);

	dev_info(ep->dev, "link/rxpkts diag (%d left):\n", ep->diag_left);
	eth_diag_dump(ep);
	if (--ep->diag_left > 0 && diag_ms)
		mod_timer(&ep->diag, jiffies + msecs_to_jiffies(diag_ms));
}

/* ---- station address ------------------------------------------------------ */
/* The address the silicon/bootloader leaves in IDR0/IDR4 when nothing has
 * programmed a real one. It is a VALID unicast address, which is exactly why it
 * has to be named: `is_valid_ether_addr()` accepts it and the random fallback
 * would never fire. MEASURED on the LANLY G24W (2026-08-20). */
static const u8 luna_default_mac[ETH_ALEN] = {
	0x00, 0xe0, 0x4c, 0x86, 0x70, 0x01
};

static bool luna_mac_is_default(const u8 *mac)
{
	return ether_addr_equal(mac, luna_default_mac);
}

static void eth_get_hwaddr(struct luna_eth *ep, u8 *mac)
{
	u32 lo = ep_rd(ep, R_IDR0), hi = ep_rd(ep, R_IDR4);

	mac[0] = lo >> 24; mac[1] = lo >> 16; mac[2] = lo >> 8; mac[3] = lo;
	mac[4] = hi >> 24; mac[5] = hi >> 16;
}

static void eth_set_hwaddr(struct luna_eth *ep, const u8 *mac)
{
	ep_wr(ep, R_IDR0, ((u32)mac[0] << 24) | ((u32)mac[1] << 16) |
			  ((u32)mac[2] << 8) | mac[3]);
	ep_wr(ep, R_IDR4, ((u32)mac[4] << 24) | ((u32)mac[5] << 16));
}

/* ---- rings ---------------------------------------------------------------- */
/* Allocate a fresh RX skb, stream-map it, and arm the descriptor on it. */
static int eth_refill(struct luna_eth *ep, unsigned int idx)
{
	struct sk_buff *skb = netdev_alloc_skb(ep->ndev, RX_BUF_SIZE);
	dma_addr_t da;
	u32 opts1;

	if (!skb)
		return -ENOMEM;
	da = dma_map_single(ep->dev, skb->data, RX_BUF_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}
	ep->rx_skb[idx] = skb;
	ep->rx_buf_dma[idx] = da;
	ep->rx_ring[idx].addr = da | DMA_BUS_WINDOW;
	ep->rx_ring[idx].opts2 = 0;
	ep->rx_ring[idx].opts3 = 0;
	opts1 = D_OWN | RX_BUF_SIZE;
	if (idx == RX_RING_SIZE - 1)
		opts1 |= D_EOR;
	ep->rx_ring[idx].opts1 = opts1;
	return 0;
}

static void eth_free_rings(struct luna_eth *ep)
{
	unsigned int i;

	for (i = 0; i < RX_RING_SIZE; i++) {
		if (ep->rx_skb[i]) {
			dma_unmap_single(ep->dev, ep->rx_buf_dma[i], RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
			dev_kfree_skb_any(ep->rx_skb[i]);
			ep->rx_skb[i] = NULL;
		}
	}
	for (i = 0; i < TX_RING_SIZE; i++) {
		kfree(ep->tx_buf[i]);
		ep->tx_buf[i] = NULL;
	}
	/* TX skbs are freed inline at xmit (copied into tx_buf), nothing to free. */
	if (ep->rx_ring)
		dma_free_coherent(ep->dev, RX_RING_SIZE * sizeof(struct rx_desc),
				  ep->rx_ring, ep->rx_ring_dma);
	if (ep->tx_ring)
		dma_free_coherent(ep->dev, TX_RING_SIZE * sizeof(struct tx_desc),
				  ep->tx_ring, ep->tx_ring_dma);
	ep->rx_ring = NULL;
	ep->tx_ring = NULL;
}

static int eth_alloc_rings(struct luna_eth *ep)
{
	unsigned int i;

	ep->rx_ring = dma_alloc_coherent(ep->dev,
			RX_RING_SIZE * sizeof(struct rx_desc),
			&ep->rx_ring_dma, GFP_KERNEL);
	ep->tx_ring = dma_alloc_coherent(ep->dev,
			TX_RING_SIZE * sizeof(struct tx_desc),
			&ep->tx_ring_dma, GFP_KERNEL);
	if (!ep->rx_ring || !ep->tx_ring)
		return -ENOMEM;

	ep->rx_head = ep->tx_head = ep->tx_dirty = 0;
	for (i = 0; i < TX_RING_SIZE; i++) {
		ep->tx_buf[i] = kmalloc(RX_BUF_SIZE, GFP_KERNEL);
		if (!ep->tx_buf[i])
			return -ENOMEM;
		ep->tx_ring[i].opts1 = (i == TX_RING_SIZE - 1) ? D_EOR : 0;
		ep->tx_skb[i] = NULL;
	}
	for (i = 0; i < RX_RING_SIZE; i++)
		if (eth_refill(ep, i))
			return -ENOMEM;
	return 0;
}

/* ---- switch open-L2 bring-up (ordered; see file header) ------------------- */
static void eth_switch_init(struct luna_eth *ep)
{
	unsigned int p;

	/* 1. enable the switch IP block (SoC control, not in SWCORE). */
	writel(readl(SOC_SW_ENABLE) | SW_EN_BIT | SW_PBO_BIT, SOC_SW_ENABLE);

	/* 1b. SoC handshake, where the chip declares one: the switch block asks
	 *     to be told the SoC is ready before it will accept its patches. The
	 *     chip's own boot loader spins on the same bit; a chip whose table
	 *     leaves it 0 simply has no such handshake. */
	if (ep->c->sys_status) {
		void __iomem *ss = (void __iomem *)(uintptr_t)ep->c->sys_status;
		int i;

		for (i = 0; i < 1000 && !(readl(ss) & BIT(1)); i++)
			udelay(100);
		if (!(readl(ss) & BIT(1)))
			dev_warn(ep->dev,
				 "switch: SoC never reported ready-for-patch (SYS_STATUS %08x after 100 ms) -- continuing, but the PHY patches may not stick\n",
				 readl(ss));
		writel(readl(ss) | BIT(0), ss);		/* soc_init_rdy */
	}

	/* 2. re-assert the SerDes egress line mode (boot-ROM leaves it warm; a
	 *    fresh value stops the uplink decaying before the rootfs mounts).
	 *    Skipped entirely on a chip with no SerDes -- offset 0 is the PHY
	 *    indirect write-data register, so writing "nothing" there is not a
	 *    no-op, it is a corruption. */
	if (ep->c->serdes_linemode)
		sw_wr(ep, ep->c->serdes_linemode, 0x44);

	/* 3. bring the physical PHYs up so a host on a jack actually trains a real
	 *    link (MAC-force alone only sets the MAC-side bit). Copper jacks: power
	 *    up + auto-neg the integrated PHYs. SerDes-6 (external RTL8221B 2.5G):
	 *    release its reset so it runs (full HiSGMII SerDes bring-up is a larger
	 *    sequence, added separately). */
	if (copper_phy)
		eth_copper_phy_up(ep);
	/* AFTER the power-up, so the survey reads the PHYs in the state the rest
	 * of the bring-up will actually see -- not the pre-power-up one, which
	 * would answer a question nobody asked. */
	if (phy_survey)
		eth_phy_survey(ep);
	/* The external 2.5G PHY hangs off the SerDes uplink, so it exists only on
	 * a chip that HAS one. Asking for it elsewhere would drive a GPIO chosen
	 * for a different board. */
	if (rtl8221b_phy && ep->c->sds_fib_status)
		eth_rtl8221b_reset_release(ep);

	/* 4. open the L2 forwarding plane. */
	for (p = 0; p <= ep->c->cpu_port; p++) {
		u32 reg = SW_LUT_UNKN_SA + (p / 16) * 4;

		/* unknown-source-MAC action 0 = learn + forward */
		sw_wr(ep, reg, sw_rd(ep, reg) & ~(3u << ((p % 16) * 2)));
	}
	sw_or(ep, SW_LUT_BC_FLOOD, ep->c->port_mask);
	sw_or(ep, SW_LUT_UNKN_MC_FLOOD, ep->c->port_mask);
	sw_or(ep, SW_LUT_UNKN_UC_FLOOD, ep->c->port_mask);
	/* SW_SRC_PORT_PERMIT (0x1C114) is a per-source-port EGRESS-FILTER ENABLE
	 * (EN, 1 bit/port), NOT a permit bitmap: EN=1 turns on source-port egress
	 * filtering and DROPS the forwarded frame after lookup/flood selection. The
	 * working firmware leaves it 0 (no filtering = forward). Writing all-ones here
	 * silently dropped every LAN->CPU frame (port RX climbed, CPU RX stayed flat).
	 * Correct forwarding-permissive value is 0. */
	sw_wr(ep, ep->c->src_permit, 0x00000000);

	/* 4a2. per-port unknown-DA lookup-miss action = FORWARD(0). We only set the
	 *      unknown-SOURCE action above; the unknown-DESTINATION action must also
	 *      forward, else post-ARP unicast / IPv6-ND to a not-yet-learned MAC is
	 *      dropped instead of flooded. Clear the 2-bit field for ports 0..10. */
	sw_wr(ep, SW_UNKN_UC_DA,  sw_rd(ep, SW_UNKN_UC_DA)  & ~DA_ACT_PORTS);
	sw_wr(ep, SW_UNKN_L2_MC,  sw_rd(ep, SW_UNKN_L2_MC)  & ~DA_ACT_PORTS);
	sw_wr(ep, SW_UNKN_IP4_MC, sw_rd(ep, SW_UNKN_IP4_MC) & ~DA_ACT_PORTS);
	sw_wr(ep, SW_UNKN_IP6_MC, sw_rd(ep, SW_UNKN_IP6_MC) & ~DA_ACT_PORTS);

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

	/* 4b. set every port's spanning-tree state to FORWARDING. The boot loader
	 *     leaves physical ports in the non-forwarding reset state, so an ingress
	 *     frame is RX-counted but never L2-forwarded to the CPU port — this is
	 *     what blocks port 3 -> CPU 9 (CPU-injected TX bypasses ingress STP, so
	 *     it looked like only the CPU->all direction worked). */
	for (p = 0; p <= ep->c->last_port; p++) {
		u32 v = sw_rd(ep, SW_MSTI_CTRL(ep, p));

		sw_wr(ep, SW_MSTI_CTRL(ep, p),
		      (v & ~STP_STATE_MASK) | STP_FORWARDING);
	}

	/* 4c. (optional) drop the CPU port from its own egress flood so it stops
	 *     receiving the broadcasts it injected (the observed self-loopback). */
	/* ★ THE PACKING IS PER CHIP, NOT JUST THE OFFSET: one 12-bit mask per
	 * port packed TWO PER WORD here, one 29-bit mask per word there. A
	 * read-modify-write is therefore mandatory -- a plain store would wipe
	 * the neighbouring port's mask on the chip that shares a word. */
	if (cpu_no_loopback) {
		unsigned int cp = ep->c->cpu_port;
		unsigned int per = ep->c->piso_per_word;
		u32 reg = ep->c->piso_base + (cp / per) * 4;
		unsigned int shift = (cp % per) * ep->c->piso_bits;
		u32 fld = ep->c->piso_all & ~BIT(cp);

		sw_wr(ep, reg,
		      (sw_rd(ep, reg) & ~(ep->c->piso_all << shift)) |
		      (fld << shift));
	}

	/* 5. force every port's MAC link up (no PHY autoneg). The CPU port (9) is
	 *    the internal MAC<->switch link the boot loader already set up with a
	 *    specific forced-ability mode; preserve that mode and only OR in the
	 *    link bit, so we don't disturb the working internal link (clobbering it
	 *    can stop the switch egressing CPU-injected frames). */
	for (p = 0; p <= ep->c->last_port; p++) {
		if (p == ep->c->cpu_port) {
			sw_or(ep, SW_FORCE_ABLTY(ep, p), BIT(4));
			sw_wr(ep, SW_ABLTY_FORCE(ep, p), ABLTY_CPU_FORCE);
		} else {
			/* ★★★ FORCE NOTHING ON A UTP PORT. We used to write
			 * FORCE_P_ABLTY = 1G/full/link and ABLTY_FORCE_MODE =
			 * 0xFFF here, i.e. override a live auto-negotiated link
			 * with a value of our own choosing.
			 *
			 * THE VENDOR DOES THE OPPOSITE, on this exact chip:
			 * `dal_rtl9603cvd_port_adminEnable_set(port, ENABLED)`
			 * on a UTP port does one thing -- it CLEARS
			 * ABLTY_FORCE_MODE.FORCE_LINK_ABLTY, i.e. STOPS forcing
			 * and lets the PHY drive the MAC. The force is applied
			 * to the CPU PORT ONLY (`dal_rtl9603cvd_port_init`).
			 *
			 * OUR OWN WORKING SIBLING ALREADY LEARNED THIS and the
			 * repair was never carried across. rtl9602c_eth.c, whose
			 * RX works, says it in its own words: "Force NO port ...
			 * Force-up ... overrides that auto-linked state and
			 * kills CPU->LAN egress (MIB: all LAN-port TX=0). Leave
			 * every port at its auto-negotiated reset state, as the
			 * bootloader does."
			 *
			 * AND THE BOOTLOADER IS THE ORACLE HERE. Measured
			 * 2026-08-23: U-Boot moved 4.1 MB through port 3 by TFTP
			 * (the switch MIB counted 2823 unicast against a 2824
			 * packet transfer, port 5 carrying the matching ACKs) --
			 * so the auto-negotiated state this code was overriding
			 * is, by construction, a state that WORKS on this board.
			 * The MIB then freezes the instant this driver runs.
			 *
			 * P_ABLTY's reset value on this chip is 0x60, the
			 * "auto-linked" value. Leaving it alone is not doing
			 * nothing: it is keeping what the hardware negotiated.
			 */
			continue;
		}
	}
	if (ep->c->force_ablty_x) {
		sw_wr(ep, ep->c->force_ablty_x, ABLTY_1G_FULL_LINK);
		sw_wr(ep, ep->c->ablty_force_x, ABLTY_FORCE_ALL);
	}

	/* 6. CPU-tag engine. The switch frames RX to the CPU with the 0x8899 tag
	 *    by default, so the CPU port is tag-aware; keep it on unless asked. */
	if (sw_cpu_tag) {
		sw_or(ep, ep->c->cpu_tag_insert, BIT(0));
		sw_or(ep, ep->c->cpu_tag_aware, BIT(0));
	}

	dev_info(ep->dev,
		 "switch: %s open-L2 up (cpu-port %u of 0..%u, src-permit=%08x, cpu-ablty=%04x)\n",
		 ep->c->name, ep->c->cpu_port, ep->c->last_port,
		 sw_rd(ep, ep->c->src_permit),
		 sw_rd(ep, SW_P_ABLTY(ep, ep->c->cpu_port)));
	/* Baseline real-link snapshot; the periodic diag (armed at open) then shows
	 * which port's genuine link comes up + rxpkts climb under host traffic. */
	eth_diag_dump(ep);
}

/* ---- MAC engine ----------------------------------------------------------- */
static void eth_hw_stop(struct luna_eth *ep)
{
	ep_wr(ep, R_IO_CMD, 0);
	ep_wr(ep, R_IO_CMD1, 0);
	iowrite16(0, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, 0);
	iowrite16(0xffff, ep->base + R_ISR);
	ep_wr(ep, R_ISR1, 0xffffffff);
	udelay(10);
}

/* GMAC0 IP-block power-cycle: the multi-ring fetch engine only latches its ring
 * state across this reset edge, so it must run before the descriptor program. */
static void eth_ipsel_cycle(void)
{
	writel(readl(SOC_IP_SEL) & ~IPSEL_GMAC0, SOC_IP_SEL);
	msleep(12);
	writel(readl(SOC_IP_SEL) | IPSEL_GMAC0, SOC_IP_SEL);
	msleep(2);
}

static void eth_hw_program(struct luna_eth *ep)
{
	u32 desnum;

	iowrite8(CMD_RXCHK | CMD_RXJUMBO, ep->base + R_CMD);
	ep_wr(ep, R_TCR, 0x00000C01);		/* IFG=3, TX enabled via IO_CMD	*/
	ep_wr(ep, R_RCR, 0x0000000E);		/* accept broadcast + matching	*/
	ep_wr(ep, R_CONFIG, 0x21000000);
	/* Enable the CPU-tag engine so the MAC strips the in-band 8-byte switch
	 * tag on RX (CTEN_RX); TX stays plain (per-descriptor opts2.cputag=0 =>
	 * the switch forwards by L2 lookup). The IP-block reset above clears this,
	 * so it must be re-asserted here. */
	ep_wr(ep, R_CPUTAGCR, CPUTAGCR_INIT);
	ep_wr(ep, R_CPUTAG1CR, CPUTAG1CR_INIT);

	/* ring pointers (writable only while the engine is stopped). */
	ep_wr(ep, R_TxFDP0, ep->tx_ring_dma | DMA_BUS_WINDOW);
	iowrite16(0, ep->base + R_TxCDO0);
	ep_wr(ep, R_RxFDP0, ep->rx_ring_dma | DMA_BUS_WINDOW);
	desnum = ((RX_RING_SIZE - 1) & 0xff) << 24 | (TH_ON_VAL & 0xff) << 16 |
		 (TH_OFF_VAL & 0xff) << 8 | (((RX_RING_SIZE - 1) >> 8) & 0xf) << 4;
	ep_wr(ep, R_RxDesNum, desnum);
	/* ★★ 32-BIT, like the vendor and like rtl9602c_eth.c -- NOT iowrite16.
	 * 0x13F4 is one 32-bit word holding RxCDO[31:16] (hardware-owned; the
	 * vendor only ever READS it, and RMWs this word with mask 0xffff00f0
	 * to preserve it) and RxRingSize[15:8].
	 * MEASURED on this board 2026-08-23: a 16-bit store lands in the UPPER
	 * half -- `iowrite16(0xf835, base + 0x3C)` read back as
	 * `0x1801203c f8350000`. So the old iowrite16 of 0x3F00 wrote
	 * 0x3F000000: RxRingSize = 0 (never programmed) and RxCDO stomped with
	 * 0x3F00. A ring of size ZERO fits every symptom -- the GMAC accepted
	 * frames (0x18012010 = 00080008, RXOKCNT = 8) with MISSPKT = 0, while
	 * eth0 RX packets stayed 0 and ISR never latched.
	 * The identical value is stored 32-bit by rtl9602c_eth.c:3371, whose RX
	 * WORKS -- a repair that lives in one copy of this driver and not the
	 * other. Guarded by ONU-test-case/reg_store_width_guard.py.	*/
	ep_wr(ep, R_RxCDO0, ((RX_RING_SIZE - 1) & 0xff) << 8 |
			    (((RX_RING_SIZE - 1) >> 8) & 0xf) << 4);
	/* route every RX class to ring 0. */
	{
		unsigned int k;

		for (k = 0; k < 7; k++)
			ep_wr(ep, R_RRING_ROUTE + k * 4, 0);
	}

	/* force the internal MAC<->switch link flow-control (top byte of MSR). */
	ep_wr(ep, R_MSR, (ep_rd(ep, R_MSR) & 0x00ffffff) | 0xf0000000);
	eth_set_hwaddr(ep, ep->ndev->dev_addr);
	ep_wr(ep, R_MAR0, 0xffffffff);
	ep_wr(ep, R_MAR4, 0xffffffff);

	/* enable edge: IO_CMD1 first, then IO_CMD (latches the fetch engine). */
	ep_wr(ep, R_IO_CMD1, IO_CMD1_ENABLE);
	ep_wr(ep, R_IO_CMD, IO_CMD_ENABLE);

	iowrite16(0xffff, ep->base + R_ISR);
	ep_wr(ep, R_ISR1, 0xffffffff);
	iowrite16(IMR_RX_BITS, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, IMR0_TX_BITS);
}

/* ---- RX / TX datapath ----------------------------------------------------- */
static int eth_rx(struct luna_eth *ep, int budget)
{
	struct net_device *ndev = ep->ndev;
	int done = 0;

	while (done < budget) {
		unsigned int i = ep->rx_head;
		u32 opts1 = ep->rx_ring[i].opts1;
		struct sk_buff *skb;
		u32 len;

		if (opts1 & D_OWN)		/* still HW-owned */
			break;

		len = opts1 & RXD_LEN_MASK;
		skb = ep->rx_skb[i];
		dma_unmap_single(ep->dev, ep->rx_buf_dma[i], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);

		if (ep->rx_dumped < rx_dump && len) {
			ep->rx_dumped++;
			print_hex_dump(KERN_INFO, "rx0: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, min_t(u32, len, 32), false);
		}

		if ((opts1 & (RXD_CRCERR | RXD_DMAERR)) ||
		    len <= (u32)rx_prefix + ETH_HLEN || len > RX_BUF_SIZE) {
			ndev->stats.rx_errors++;
			dev_kfree_skb_any(skb);
		} else {
			/* The CPU port frames a packet as:
			 *   [front prefix][DA][SA][switch tag][ethertype][payload]
			 * Strip the fixed front prefix, then excise the 8-byte 0x8899
			 * switch tag (if present) by sliding DA+SA over it. */
			skb_put(skb, len);
			if (rx_prefix)
				skb_pull(skb, rx_prefix);
			if (skb->len > 2 * ETH_ALEN + RTL_CPU_TAG_LEN &&
			    skb->data[2 * ETH_ALEN] == 0x88 &&
			    skb->data[2 * ETH_ALEN + 1] == 0x99) {
				if (ep->rx_dumped <= rx_dump)
					dev_info(ep->dev, "rx tag: %*ph\n",
						 RTL_CPU_TAG_LEN,
						 skb->data + 2 * ETH_ALEN);
				memmove(skb->data + RTL_CPU_TAG_LEN, skb->data,
					2 * ETH_ALEN);
				skb_pull(skb, RTL_CPU_TAG_LEN);
			}
			/* Drop our own egress that the switch flooded back to the CPU
			 * port (source MAC == ours) so the bridge does not log
			 * "received packet ... with own address as source". */
			if (skb->len >= 2 * ETH_ALEN &&
			    ether_addr_equal(skb->data + ETH_ALEN, ndev->dev_addr)) {
				dev_kfree_skb_any(skb);
			} else {
				/* NAPI poll context: use the receive path, not netif_rx. */
				skb->protocol = eth_type_trans(skb, ndev);
				ndev->stats.rx_packets++;
				ndev->stats.rx_bytes += len;
				napi_gro_receive(&ep->napi, skb);
			}
		}

		if (eth_refill(ep, i)) {	/* re-arm with a fresh skb */
			ndev->stats.rx_dropped++;
			break;
		}
		ep->rx_head = (i + 1) % RX_RING_SIZE;
		done++;
	}
	return done;
}

static void eth_tx_reclaim(struct luna_eth *ep)
{
	/* The skb was already freed at xmit (its bytes were copied into tx_buf),
	 * so reclaim just unmaps the copy buffer and releases the ring slot once
	 * the DMA engine has handed the descriptor back (OWN cleared). */
	while (ep->tx_dirty != ep->tx_head) {
		unsigned int i = tx_slot(ep->tx_dirty);

		if (ep->tx_ring[i].opts1 & D_OWN)	/* not transmitted yet */
			break;
		dma_unmap_single(ep->dev, ep->tx_buf_dma[i], ep->tx_buf_len[i],
				 DMA_TO_DEVICE);
		ep->tx_dirty++;
	}
	if (netif_queue_stopped(ep->ndev) &&
	    (ep->tx_head - ep->tx_dirty) < TX_RING_SIZE - 1)
		netif_wake_queue(ep->ndev);
}

static int eth_napi_poll(struct napi_struct *napi, int budget)
{
	struct luna_eth *ep = container_of(napi, struct luna_eth, napi);
	unsigned long flags;
	int work;

	work = eth_rx(ep, budget);
	spin_lock_irqsave(&ep->tx_lock, flags);
	eth_tx_reclaim(ep);
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (work < budget) {
		napi_complete_done(napi, work);
		/* W1C any status latched while masked, then re-unmask. */
		iowrite16(ioread16(ep->base + R_ISR), ep->base + R_ISR);
		ep_wr(ep, R_ISR1, ep_rd(ep, R_ISR1));
		iowrite16(IMR_RX_BITS, ep->base + R_IMR);
		ep_wr(ep, R_IMR0, IMR0_TX_BITS);
	}
	return work;
}

static irqreturn_t eth_irq(int irq, void *data)
{
	struct luna_eth *ep = netdev_priv((struct net_device *)data);
	u16 isr = ioread16(ep->base + R_ISR);

	if (!isr && !ep_rd(ep, R_ISR1))
		return IRQ_NONE;
	/* mask and let NAPI drain + re-unmask. */
	iowrite16(0, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, 0);
	napi_schedule(&ep->napi);
	return IRQ_HANDLED;
}

/* Backstop drain: catches a missed/unrouted IRQ so the datapath always makes
 * progress during bring-up (ping does not need IRQ latency). */
static void eth_backstop(struct timer_list *t)
{
	struct luna_eth *ep = timer_container_of(ep, t, backstop);

	napi_schedule(&ep->napi);
	mod_timer(&ep->backstop, jiffies + msecs_to_jiffies(backstop_ms));
}

static netdev_tx_t eth_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct luna_eth *ep = netdev_priv(ndev);
	unsigned long flags;
	unsigned int i, len = skb->len;
	dma_addr_t da;
	void *buf;
	u32 opts1;

	if (len > RX_BUF_SIZE) {		/* must fit a copy slot */
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	if ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}
	i = tx_slot(ep->tx_head);
	buf = ep->tx_buf[i];

	/* Copy the WHOLE frame into the linear copy slot. Use skb_copy_bits, not a
	 * flat memcpy(skb->data,...): non-linear/fragmented skbs (e.g. the ICMP echo
	 * reply assembled by ip_append_data in the RX softirq) keep only skb_headlen
	 * bytes at skb->data and the rest in frags — a flat memcpy would copy the
	 * header plus out-of-bounds garbage, which the host then drops. The buffer is
	 * then stream-mapped; the kernel-managed L2 makes that DMA coherent. */
	if (skb_copy_bits(skb, 0, buf, len)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (len < ETH_ZLEN) {		/* zero-pad runt frames (e.g. a 42-byte ARP reply)
					 * in the copy buffer and extend the DMA length;
					 * skb_padto only guarantees tailroom, it does NOT
					 * grow skb->len, so we pad here after the copy. */
		memset((u8 *)buf + len, 0, ETH_ZLEN - len);
		len = ETH_ZLEN;
	}

	if (ep->tx_dumped < tx_dump) {
		ep->tx_dumped++;
		print_hex_dump(KERN_INFO, "tx0: ", DUMP_PREFIX_OFFSET, 16, 1,
			       buf, min_t(unsigned int, len, 48), false);
	}

	da = dma_map_single(ep->dev, buf, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	ep->tx_buf_dma[i] = da;
	ep->tx_buf_len[i] = len;
	ep->tx_ring[i].addr = da | DMA_BUS_WINDOW;
	ep->tx_ring[i].opts2 = 0;	/* plain frame: switch forwards by L2 DA */
	ep->tx_ring[i].opts3 = 0;
	ep->tx_ring[i].opts4 = 0;
	opts1 = D_OWN | D_FS | D_LS | D_TXCRC | (len & TXD_LEN_MASK);
	if (i == TX_RING_SIZE - 1)
		opts1 |= D_EOR;
	wmb();				/* descriptor body before ownership */
	ep->tx_ring[i].opts1 = opts1;
	wmb();

	ep->tx_head++;
	ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* kick ring 0 */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	dev_consume_skb_any(skb);	/* bytes copied; release immediately */
	return NETDEV_TX_OK;
}

static void eth_set_rx_mode(struct net_device *ndev)
{
	struct luna_eth *ep = netdev_priv(ndev);
	u32 rcr = ep_rd(ep, R_RCR);

	/* a bridge enslaving eth0 sets promisc; accept-all-physical (RCR bit0)
	 * is then required to receive LAN-client frames whose DA != our MAC. */
	if (ndev->flags & (IFF_PROMISC | IFF_ALLMULTI))
		rcr |= BIT(0);
	else
		rcr &= ~BIT(0);
	ep_wr(ep, R_RCR, rcr);
}

static int eth_set_mac_address(struct net_device *ndev, void *addr)
{
	struct luna_eth *ep = netdev_priv(ndev);
	int ret = eth_mac_addr(ndev, addr);

	if (!ret)
		eth_set_hwaddr(ep, ndev->dev_addr);
	return ret;
}

/* ---- open / stop ---------------------------------------------------------- */
static int eth_open(struct net_device *ndev)
{
	struct luna_eth *ep = netdev_priv(ndev);
	int ret;

	ret = eth_alloc_rings(ep);
	if (ret) {
		eth_free_rings(ep);
		return ret;
	}

	eth_hw_stop(ep);
	eth_ipsel_cycle();
	eth_switch_init(ep);		/* program forwarding before the DMA starts */
	eth_hw_program(ep);

	if (ep->irq > 0) {
		ret = request_irq(ep->irq, eth_irq, 0, ndev->name, ndev);
		if (ret) {
			netdev_warn(ndev, "IRQ %d request failed (%d); poll-only\n",
				    ep->irq, ret);
			ep->irq = -1;
		}
	}
	napi_enable(&ep->napi);
	timer_setup(&ep->backstop, eth_backstop, 0);
	mod_timer(&ep->backstop, jiffies + msecs_to_jiffies(backstop_ms));

	/* periodic real-link/rxpkts diagnostic (bring-up: locate the host's jack). */
	ep->diag_left = diag_count;
	timer_setup(&ep->diag, eth_diag_timer, 0);
	if (diag_ms && diag_count > 0)
		mod_timer(&ep->diag, jiffies + msecs_to_jiffies(diag_ms));

	netif_start_queue(ndev);
	netif_carrier_on(ndev);
	netdev_info(ndev, "up: irq=%d rx_prefix=%d backstop=%ums copper_phy=%d rtl8221b=%d\n",
		    ep->irq, rx_prefix, backstop_ms, copper_phy, rtl8221b_phy);
	return 0;
}

static int eth_stop(struct net_device *ndev)
{
	struct luna_eth *ep = netdev_priv(ndev);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	timer_delete_sync(&ep->diag);
	timer_delete_sync(&ep->backstop);
	napi_disable(&ep->napi);
	if (ep->irq > 0)
		free_irq(ep->irq, ndev);
	eth_hw_stop(ep);
	eth_free_rings(ep);
	return 0;
}

static const struct net_device_ops luna_eth_netdev_ops = {
	.ndo_open		= eth_open,
	.ndo_stop		= eth_stop,
	.ndo_start_xmit		= eth_xmit,
	.ndo_set_rx_mode	= eth_set_rx_mode,
	.ndo_set_mac_address	= eth_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
};

/* ---- probe ---------------------------------------------------------------- */
static int luna_eth_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct luna_eth *ep;
	u8 mac[ETH_ALEN];
	int ret;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ndev = devm_alloc_etherdev(dev, sizeof(*ep));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, dev);
	platform_set_drvdata(pdev, ndev);

	ep = netdev_priv(ndev);
	/* ★ THE CHIP TABLE COMES FROM THE MATCH, AND ITS ABSENCE IS FATAL. There
	 * is deliberately NO default: falling back to "the chip we happened to
	 * write first" is precisely how a sibling's register map reaches a new
	 * board, which is the defect this whole table exists to prevent. */
	ep->c = of_device_get_match_data(dev);
	if (!ep->c) {
		dev_err(dev, "no chip table for this compatible -- refusing to probe rather than guess a register map\n");
		return -ENODEV;
	}
	ep->ndev = ndev;
	ep->dev = dev;
	spin_lock_init(&ep->tx_lock);

	ep->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ep->base))
		return PTR_ERR(ep->base);
	ep->sw = devm_ioremap(dev, SWCORE_PHYS, SWCORE_SIZE);
	if (!ep->sw)
		return -ENOMEM;

	/* MAC: DT/nvmem, else the value the boot loader programmed into the MAC
	 * engine, else a random locally-administered address.
	 *
	 * ★★★ AND THE ENGINE'S VALUE IS NOT AUTOMATICALLY A BOARD MAC. MEASURED
	 * on the LANLY G24W, 2026-08-20: with nothing in DT and no `ethaddr` in
	 * this product's U-Boot environment, IDR0/IDR4 hold the SILICON BRING-UP
	 * DEFAULT 00:e0:4c:86:70:01 -- a Realtek OUI address that
	 * `is_valid_ether_addr()` accepts, so the random fallback never fired and
	 * the board came up with it.
	 *
	 * ⚠ THAT IS A CROSS-BOARD COLLISION, NOT AN AESTHETIC PROBLEM. It is the
	 * same value on EVERY board that boots this image, and this lab runs three
	 * ONUs on ONE L2 segment. Two of them holding one MAC does not produce an
	 * error anywhere -- it produces a switch that learns the address on
	 * whichever port spoke last, and measurements on somebody else's bench
	 * that fail for no visible reason.
	 *
	 * ⇒ the default is REFUSED and a random LAA is used instead, LOUDLY. A
	 * random address is unique by construction; a shared one is wrong by
	 * construction. This board's REAL MAC (5c:19:23:b3:ce:90 on the label)
	 * lives in the vendor's `config` MTD partition MIB -- the vendor sets it
	 * from userspace, `ifconfig eth0 hw ether ...`, not from the bootloader --
	 * so recovering it is flash-reading work, and OWED. Until then a unique
	 * wrong address beats a shared one.
	 */
	if (of_get_ethdev_address(dev->of_node, ndev)) {
		eth_get_hwaddr(ep, mac);
		if (is_valid_ether_addr(mac) && !luna_mac_is_default(mac)) {
			eth_hw_addr_set(ndev, mac);
		} else {
			eth_hw_addr_random(ndev);
			dev_warn(dev,
				 "MAC engine holds %pM: %s. Using a random locally-administered address %pM instead -- this board's real MAC lives in the vendor config partition and is not read yet\n",
				 mac,
				 is_valid_ether_addr(mac)
				 ? "the SILICON BRING-UP DEFAULT, identical on every board of this family, which would collide on a shared segment"
				 : "not a valid unicast address",
				 ndev->dev_addr);
		}
	}

	ndev->netdev_ops = &luna_eth_netdev_ops;
	netif_carrier_off(ndev);
	netif_napi_add(ndev, &ep->napi, eth_napi_poll);

	ep->irq = platform_get_irq_optional(pdev, 0);
	if (ep->irq < 0)
		ep->irq = -1;

	ret = devm_register_netdev(dev, ndev);
	if (ret)
		return ret;

	dev_info(dev, "%s NIC at %pR, MAC %pM, irq %d\n", ep->c->name,
		 platform_get_resource(pdev, IORESOURCE_MEM, 0),
		 ndev->dev_addr, ep->irq);
	return 0;
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
		.name		= "rtl960x-eth",
		.of_match_table	= luna_eth_of_match,
	},
};
module_platform_driver(luna_eth_driver);

/*
 * GPON OMCI glue stubs — the shared GPON FSM (gpon-rtl9602c.c) references these
 * symbols declared in rtl9602c_gpon_nic.h. On the 9602C the real implementations
 * live in rtl9602c_eth.c; on the 9607C the OMCI datapath is M4 and these are
 * minimal no-ops so the kernel links. They are enough for M3 (reach O5 + DS).
 */
#include "rtl9602c_gpon_nic.h"

void rtl9602c_eth_set_omci_sid(unsigned int sid) { }
EXPORT_SYMBOL(rtl9602c_eth_set_omci_sid);

void rtl9602c_eth_set_omci_identity(const u8 *sn8) { }
EXPORT_SYMBOL(rtl9602c_eth_set_omci_identity);

u32 rtl9602c_eth_omci_rx_count(void) { return 0; }
EXPORT_SYMBOL(rtl9602c_eth_omci_rx_count);

u32 rtl9602c_eth_wan_rx_count(void) { return 0; }
EXPORT_SYMBOL(rtl9602c_eth_wan_rx_count);

u32 rtl9602c_eth_omci_tx_dirty(void) { return 0; }
EXPORT_SYMBOL(rtl9602c_eth_omci_tx_dirty);

void rtl9602c_eth_omci_selftest(void) { }
EXPORT_SYMBOL(rtl9602c_eth_omci_selftest);

void rtl9602c_eth_omci_report_oper_up(void) { }
EXPORT_SYMBOL(rtl9602c_eth_omci_report_oper_up);

MODULE_DESCRIPTION("Realtek Luna (RTL9607C / RTL9603CVD) GMAC0 + switch Ethernet driver");
MODULE_LICENSE("GPL");
