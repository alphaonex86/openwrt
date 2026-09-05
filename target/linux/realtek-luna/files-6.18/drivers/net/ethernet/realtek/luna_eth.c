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
 * ★★ WHY THIS FILE IS NAMED luna_* AND NOT rtl9607c_*.
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
#include "luna_eth_regs.h"	/* the family MAC/switch register map + per-chip table */
#include "luna_gmac_logic.h"	/* family GMAC ring packings + this shell's hoisted RX verdicts (flowcore) */

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
/* ★ DEFAULT 0 = do NOT assert the GPHY reset, which is what the vendor does.
 * A param and not a deletion, so the OLD behaviour is one bootarg away and the
 * A/B costs no rebuild -- and so a board that turns out to NEED it can have it.
 */
static bool gphy_reset;
module_param(gphy_reset, bool, 0444);
MODULE_PARM_DESC(gphy_reset,
		 "assert SOFTWARE_RST.CMD_GPHY_RST_PS during copper PHY bring-up "
		 "(default 0: the vendor never asserts it, and we run none of the "
		 "re-initialisation that reset would require)");

module_param(diag_ms, uint, 0644);
MODULE_PARM_DESC(diag_ms, "period of the per-port real-link/rxpkts diagnostic (0 = off)");

static int diag_count = 12;
module_param(diag_count, int, 0644);
MODULE_PARM_DESC(diag_count, "number of periodic link/rxpkts diagnostic dumps");

/* ★★★ DEFAULT 1 SINCE 2026-08-23, and the EMULATED vendor trace is why.
 * Running `dal_rtl9603cvd_switch_init` under Unicorn showed the vendor write the
 * PHY POWER register at OCP `0xA400` on **all of phy0..phy5**, and use the flat
 * low addresses `0x0020`-`0x003E` ONLY for the FE *patch* register block on
 * phy0/1/2. Those are two different register SETS, not two maps for one
 * register -- which settles a contradiction this file's own comment called
 * unresolved.
 * With `gphy_map = 0` and this chip's `gphy_ports = 0x08`, our BMCR and
 * AN-restart writes on THREE of the four copper ports went to flat `0x0000`
 * where the vendor uses `0xA400`.
 * ⚠ A SINGLE-VARIABLE EXPERIMENT, not a proven fix: the previous three
 * candidates (the RxCDO store width, the force-every-UTP-port policy, the GPHY
 * reset) were each plausible and each REFUTED by measurement. `gphy_map=0` is
 * one bootarg away, so this is reversible without a rebuild.
 */
/* ★ How long to let an OCP transaction start before sampling BUSY. The vendor
 * uses a flat mdelay(10) and no poll at all; 200 us is two orders of magnitude
 * less and still far longer than a bus turnaround, and the poll below keeps the
 * timeout safety the vendor's version does not have. 0 = the old behaviour.
 */
/* ★★ MEASURED 2026-08-24 -- STOCK vs OURS, the SAME register, same board.
 * SWCORE 0x0000004C is CFG_PHY_CTRL, named and laid out by THIS chip's own
 * chipdef (rtl9603cvd): MSK_MDI[8:5] | BASE_PHYAD[4:0].
 *
 *      stock  0x00000000  ->  BASE_PHYAD = 0
 *      ours   0x00000019  ->  BASE_PHYAD = 25
 *
 * ⚠ CORRECTED 2026-08-26 -- IT WAS NOT THE BOOTLOADER, IT WAS OUR OWN GPON
 * DRIVER, and blaming U-Boot sent this investigation to the wrong tree.
 * gpon-rtl960x.c used the RTL9602C literal SOC_IO_GPIO_EN = 0x48 for the
 * optical-SD GPIO pad recipe. On the RTL9603CVD 0x48 is CFG_PCSXF and 0x4c is
 * CFG_PHY_CTRL, so its second word landed HERE:
 *
 *      SOC_IO_GPIO_EN_W1 = 0x819,  0x819 & 0x1ff = 0x019
 *      -> MSK_MDI[8:5] = 0, BASE_PHYAD[4:0] = 25
 *
 * which is bit-for-bit the 0x00000019 measured above. The boot log shows both
 * halves in order: "gpon: GPIO pads set (gpio_en1=0x00000019)" at t=0.75 s,
 * then this driver's "BASE_PHYAD 25 -> 0" at t=7.02 s -- a 6.3 s window in
 * which the switch ran with the wrong MDIO base. The GPON driver now skips
 * that recipe on this chip, so there is nothing left to repair and the write
 * below is a no-op that merely asserts the value.
 * Beside that, every PHY read here returns 0000 with BUSY never once
 * asserting, while the identical window on stock holds a real datum
 * (GPHY_IND_RD = 0x1940, a BMCR with PDOWN set). If the indirect GPHY access
 * is addressed through the MDIO master BASE_PHYAD points at, our phyid 0..3
 * land on MDIO 25..28, where nothing answers -- which is what a read of 0000
 * with no BUSY looks like.
 *
 * -1 leaves the register exactly as the bootloader left it. That is how this
 * gets FALSIFIED rather than believed. */
/* Which receive FIFOs stay in reset. Stock holds NONE.
 *
 * ⚠ CORRECTED 2026-08-26, SAME CAUSE AS CFG_PHY_CTRL ABOVE: the one asserted
 * FIFO was not the bootloader's either. The GPON driver's first pad word,
 * SOC_IO_GPIO_EN_W0 = 0x40202006, landed on CFG_PCSXF (0x48 on this chip) and
 * its writable bits are 0x2006 -> RST_RXFIFO[13:10] = 8, i.e. exactly one FIFO
 * held in reset. Fixed at the source; -1 still = leave whatever was there. */
static int rst_rxfifo;
module_param(rst_rxfifo, int, 0644);
MODULE_PARM_DESC(rst_rxfifo,
	"CFG_PCSXF RST_RXFIFO mask to program (-1 = leave the bootloader's value)");

/* ★★★ THE BOARD'S OWN MAC, HANDED IN AT BOOT (2026-08-24).
 *
 * The comment on the probe path below already records where this board's real
 * address lives: the vendor's `config` MTD partition, in a MIB the vendor
 * applies from USERSPACE -- not from the bootloader, whose environment carries
 * only the shared default this driver refuses. Reading that partition in the
 * kernel is a JFFS2 mount away and the value inside is zlib-compressed, so an
 * `nvmem-cell` (which reads raw bytes at a fixed offset) cannot express it.
 *
 * What CAN be done today, and is the FIRST rung of this project's own declared
 * precedence -- "U-Boot ethaddr bootarg -> DT/nvmem -> random LAA" -- is to
 * hand the address in on the kernel command line:
 *
 *     luna_eth.mac=5c:19:23:b3:ce:90
 *
 * ⚠ AND THE VALUE IS NOT WRITTEN HERE OR IN THE DTS. It is per-UNIT, and a DTS
 * literal would hand the second G24W the first one's identity -- silently, on a
 * segment where three ONUs already share one L2 domain. The boot tool reads it
 * from THAT BOARD's own declaration and passes it; a board that declares none
 * passes nothing and the random-LAA behaviour below is unchanged.
 */
static char *mac_param;
module_param_named(mac, mac_param, charp, 0444);
MODULE_PARM_DESC(mac,
	"the board's own MAC, handed in at boot (xx:xx:xx:xx:xx:xx). Empty = "
	"fall back to DT, then the engine, then a random locally-administered one");

static int base_phyad = 0;
module_param(base_phyad, int, 0644);
MODULE_PARM_DESC(base_phyad,
	"CFG_PHY_CTRL BASE_PHYAD to program (-1 = leave the bootloader's value)");

/* The PHY survey is a probe-time dump, so testing one BASE_PHYAD used to cost
 * one build and one boot. This lets a single image answer the whole sweep. */
static struct luna_eth *survey_ep;

static int gphy_settle_us = 200;
module_param(gphy_settle_us, int, 0644);
MODULE_PARM_DESC(gphy_settle_us,
		 "microseconds to let an internal-PHY OCP transaction START before "
		 "sampling BUSY (0 = sample immediately, which returns success "
		 "before the transaction begins and reads a stale zero)");

static int gphy_map = 1;
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
#define R_MAR0		0x08	/* multicast hash [31:0]				*/
#define R_MAR4		0x0C	/* multicast hash [63:32]			*/
#define R_CMD		0x3B	/* 8-bit command: bit0 RST			*/
#define   CMD_RXCHK	0x02	/* RX checksum offload				*/
#define   CMD_RXJUMBO	0x08	/* accept jumbo					*/
#define R_MSR		0x58	/* media/flow status; top byte = force flow ctl	*/
#define R_TxFDP0	0x1300	/* TX ring0 fetch-descriptor pointer		*/
#define R_TxCDO0	0x1304	/* TX ring0 current-descriptor offset (u16)	*/
#define R_RRING_ROUTE	0x1370	/* RX class -> ring routing			*/
#define R_RxFDP0	0x13F0	/* RX ring0 fetch-descriptor pointer		*/
#define R_RxCDO0	0x13F4	/* RX ring0: RxCDO[31:16] | RxRingSize[15:8]	*/
				/* 32-BIT register. The (u16) this comment used to carry
				 * named the FIELD, and an iowrite16 was written to match
				 * it -- see the store below.			*/

/* engine enable values (inherited-config + DMA-enable edge) */
/* MSR(0x58) top byte.  Live working stock runs 0xf0 (FORCE_TRXFCE|RXFCE|TXFCE:
 * flow control FORCED on the internal GMAC<->switch links).
 *
 * ★★ HW-PROVEN ON THE SIBLING CHIP (RTL9602C, 2026-06-12): with our MINIMAL
 * init, 0xf0 STALLS the LAN datapath (ping 0/60) while 0x10 keeps it healthy
 * (40/40).  Stock tolerates 0xf0 because its FULL init configures flow control
 * properly; ours does not, so 0xf0's forced pause-frame handling wedges the
 * MAC.  It is an init-COMPLETENESS problem, not a value we can simply match.
 *
 * ⚠ THIS DRIVER HARDCODED 0xf0 UNTIL 2026-08-28 -- the exact value the other
 * Luna driver had already measured as harmful.  That is the failure mode the
 * whole deduplication exists to stop: a repair that lands in one board's copy
 * of a shared driver and never reaches the others.  The two drivers had no
 * symbol in common, so nothing could have flagged it.
 *
 * ★★ THE A/B WAS RUN ON THIS BOARD, 2026-08-28, AND IT REFUTED THE TRANSFER.
 * Two cold TFTP boots of the same image, one at 0x10 and one with
 * `luna_eth.msr_top=0xf0` in bootargs: the LAN answered 5/5 BOTH TIMES.  So
 * the RTL9602C finding does NOT carry to the RTL9603CVD -- forced flow control
 * is not what decides this board's datapath, and nobody should re-chase it.
 *
 * The default stays 0x10 because it is the only value anyone has MEASURED as
 * safe on this family and it makes the two sibling drivers agree; it is NOT
 * claimed as a fix, and on this chip both values were measured equal.
 *
 * ⚠ NOTE WHAT THIS ALSO SAYS: the LAN answered at all.  The board's standing
 * "RX path exhausted" investigation describes a dead LAN->CPU ingress, and on
 * this date that path carries pings on both settings -- so that document is
 * either stale or the fault is intermittent.  Re-verify it before building on
 * it; do not read this comment as "RX is fixed".
 */
static unsigned int msr_top = 0x10;
module_param(msr_top, uint, 0644);
MODULE_PARM_DESC(msr_top, "MSR(0x58) top byte (0x10 = healthy with our init; 0xf0 = stock's value, MEASURED to stall the LAN on the RTL9602C)");

#define IO_CMD_ENABLE	0xc059f130
#define IO_CMD1_ENABLE	0x32000001

/* CPU-tag engine config. CTEN_RX (bit31) makes the MAC strip the 8-byte switch
 * tag in hardware on RX and expose the parsed ingress port in the descriptor;
 * the rest selects tag sizes, the 0x04 protocol and the 0x8899 match. Clearing
 * CTEN_RX leaves the raw in-band tag in the delivered frame. */
#define CPUTAGCR_INIT	0x9022FF04
#define CPUTAG1CR_INIT	0x00004000	/* CPU-tag SID base (64 << 8)		*/
#define ABLTY_CPU_FORCE	0xBFFF		/* CPU-port forced-ability mode (keep)	*/


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

/* -- offsets that are the SAME on every Luna part covered here -------------- */
#define SW_GPHY_WD		0x00000	/* internal-PHY indirect: write data	*/
#define SW_GPHY_CMD		0x00004	/*   ... command (phy<<16 | ocp)		*/
#define SW_GPHY_RD		0x00008	/*   ... read data + BUSY		*/
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

	/* ★ The switch-core map for THIS chip -- and the ONLY home of this
	 * chip's port numbers.  They were briefly duplicated here AND in
	 * luna_sw_map (2026-08-28, by the same change that tabulated them for
	 * the sibling driver); the two agreed, which is luck and not structure.
	 * Two copies of one silicon fact is how they come to disagree in silence.  Kept as a pointer into
	 * luna_eth_regs.h rather than copied in, so the sibling driver and
	 * this one read the SAME numbers and a correction lands once. */
	const struct luna_sw_map *sw_map;

	/* --- switch port map ------------------------------------------------ */
	/* ★ Force the PON port's MAC link like the CPU port's -- 1 only on a
	 * chip where STOCK was MEASURED doing it. See the write site. */
	u8	force_pon_ablty;
	u8	last_port;	/* highest port to iterate, INCLUSIVE		*/
	u8	n_copper;	/* copper PHY ports, always 0..n_copper-1	*/
	u8	gphy_ports;	/* bitmap: ports whose PHY is a GPHY, not FE	*/

	/* --- switch registers that MOVED ------------------------------------ */
	/* force_ablty MOVED to luna_sw_map (via ->sw_map) */	/* + 4*port: forced ability values		*/
	/* p_ablty MOVED to luna_sw_map (via ->sw_map) */	/* + 4*port: LIVE ability, read-only		*/
	/* ablty_force MOVED to luna_sw_map (via ->sw_map) */	/* + 4*port: which ability fields are forced	*/
	u32	msti_ctrl;	/* + 4*port: per-port spanning-tree state	*/
	u32	cpu_tag_insert;
	u32	cpu_tag_aware;
	u32	swcore_rst;	/* swcore soft reset (bit10), excludes cfg	*/
	/* gphy_misc MOVED to luna_sw_map (reached via ->sw_map): the RTL9602C
	 * driver needs the same fact and a second home is how the two come to
	 * disagree.  Same reasoning, and the same fix, as the port numbers. */
	u32	fephy_poll;	/* 0 on a chip with no FE-PHY auto-poller	*/
	u32	cfg_phy_ini;	/* per-port PHY enable; U-Boot loads it from efuse*/
	u32	cfg_phy_ctrl;	/* MSK_MDI[8:5] | BASE_PHYAD[4:0]; 0 = not known */
	u32	cfg_pcsxf;	/* RST_RXFIFO[13:10] | MIIRX_IPG[9:5] | PCSXF[4:1] */

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
};

/* ---------------------------------------------------------------------------
 * ★ THE RTL9607C ENTRY IS THIS FILE'S PREVIOUS CONSTANTS, VALUE FOR VALUE.
 * That is deliberate and it is the acceptance test for the refactor: the
 * engineering board that boots today must see a byte-identical register
 * sequence, so any behaviour change on it would be a defect of THIS commit and
 * not of the new chip.
 * --------------------------------------------------------------------------- */
static const struct luna_eth_chip luna_chip_rtl9607c = {
	.sw_map		= &rtl9607c_sw_map,
	.name		= "RTL9607C",
	/* ⚠ 0 DELIBERATELY, and it is a scope statement rather than a
	 * finding: nobody has diffed SWCORE 0x1cc/0x238 stock-vs-ours on the
	 * RTL9607C engineering board, and this chip reaches its PON/PBO
	 * abilities through the separate force_ablty_x trio below. The
	 * acceptance test for this driver is that the 9607C sees a
	 * byte-identical register sequence, so a value measured on another
	 * die may not be applied here on the strength of the family name. */
	.force_pon_ablty = 0,
	.last_port	= 11,	/* 0..4,8 copper; 5 PON; 6,7 SerDes; 9 CPU; 11 PBO */
	.n_copper	= 5,	/* ports 0..4 */
	.gphy_ports	= 0x1f,	/* all five copper ports are GPHYs here	*/
	.msti_ctrl	= 0x1704C,
	.cpu_tag_insert	= 0x230F4,
	.cpu_tag_aware	= 0x230F8,
	.swcore_rst	= 0x00108,
	.fephy_poll	= 0,	/* every PHY here is a GPHY: no FE auto-poller	*/
	.cfg_phy_ini	= 0x0004C,
	.piso_per_word	= 1,
	.piso_bits	= 29,
	.piso_all	= 0x1FFFFFFF,
	.serdes_linemode = 0x00084,
	.force_ablty_x	= 0x002F4,
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
	.sw_map		= &rtl9603cvd_sw_map,
	.name		= "RTL9603CVD",
	/* ★★ MEASURED 2026-08-27, stock vs ours, SWCORE 0x180..0x1fc
	 * (swcore_diff.py --board=RTL9603CVD/LANLY/G24W --diff):
	 *     FORCE_P_ABLTY[4]     stock 0x00000016   ours 0x00000000
	 *     ABLTY_FORCE_MODE[4]  stock 0x0000bfff   ours 0x00000000
	 * i.e. stock forces the PON port up with EXACTLY the pair it applies
	 * to the CPU port, and we leave it at reset. Both are portless
	 * internal MAC links: there is no PHY to auto-negotiate with, so the
	 * `leave a UTP port alone` rule below does not reach them. */
	.force_pon_ablty = 1,
	.last_port	= 5,	/* 0..2 FE; 3 GE; 4 PON; 5 CPU (6 = PBO loopback)*/
	.n_copper	= 4,	/* ports 0..3					*/
	.gphy_ports	= 0x08,	/* ONLY port 3 is a GPHY; 0..2 are FE PHYs	*/
	.msti_ctrl	= 0x1713C,
	.cpu_tag_insert	= 0x2303C,
	.cpu_tag_aware	= 0x23040,
	.swcore_rst	= 0x000E0,
	.fephy_poll	= 0x0000C,
	.cfg_phy_ini	= 0x00050,
	.cfg_phy_ctrl	= 0x0004C,
	.cfg_pcsxf	= 0x00048,
	.piso_per_word	= 2,
	.piso_bits	= 12,
	.piso_all	= 0xFFF,
	.serdes_linemode = 0,	/* no SerDes on this part			*/
	.force_ablty_x	= 0,
	.ablty_force_x	= 0,
	.sds_fib_status	= 0,
	.sys_status	= 0xB8000044,	/* SoC handshake, outside SWCORE		*/
};

/* Accessors: the table lives in `ep->c`, so a per-port register is one call and
 * a chip that lacks a register is answered with 0 and SKIPPED by the caller. */
#define SW_FORCE_ABLTY(ep, p)	((ep)->c->sw_map->force_ablty + (p) * 4)
#define SW_P_ABLTY(ep, p)	((ep)->c->sw_map->p_ablty + (p) * 4)
#define SW_ABLTY_FORCE(ep, p)	((ep)->c->sw_map->ablty_force + (p) * 4)
#define SW_MSTI_CTRL(ep, p)	((ep)->c->msti_ctrl + (p) * 4)

/* VLAN: filtering must not gate CPU<->LAN egress (the boot loader may leave it on
 * with the CPU port outside the member set). */
#define   VLAN_FILTERING	BIT(0)

/* Per-port lookup-miss (unknown-DA) action, 2 bits/port; 0 = FORWARD. Needed for
 * the post-ARP unicast / IPv6-ND path (the first broadcast already floods). */
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

/* SOC_SW_ENABLE and its bits are the FAMILY's -- luna_eth_regs.h, which
 * also records that this driver and gpon-rtl960x.c name bit 5 differently. */

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
#define   CMD_GPHY_RST_PS	BIT(6)	/* SOFTWARE_RST(0x0E0) bit 6	*/
/* ★ RENAMED 2026-08-23. It was `GPHY_MACRO_RST`, a name this chip does not
 * use: `MACRO` occurs ZERO times in the RTL9603CVD field list. The register
 * at 0x0E0 is SOFTWARE_RST and bit 6 is CMD_GPHY_RST_PS (lsp 6, len 1),
 * sibling to SW_RST / PONMAC_RST / CMD_CHIP_RST_PS. The offset and the bit
 * were right; the name described a register that does not exist here, and a
 * misleading name is renamed the day it is proven wrong.	*/
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
/* DMA_BUS_WINDOW is the FAMILY's -- luna_eth_regs.h, with the measurement
 * that explains why it is zero and must stay a named knob. */

/* struct rx_desc / struct tx_desc are the FAMILY's -- luna_eth_regs.h.
 * They were declared identically in both drivers; one type now. */

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
/* ★★★ SETTLE BEFORE THE FIRST SAMPLE, OR THE POLL CANNOT FAIL.
 * `gphy_wait` waits for BUSY to CLEAR and samples immediately after the CMD
 * store. If BUSY has not yet ASSERTED -- the transaction has not started -- the
 * very first read sees BUSY=0, the wait returns SUCCESS instantly, and the
 * caller then reads RD_DAT holding whatever was there before: ZERO.
 *
 * MEASURED 2026-08-24 on the G24W, and it is uniform: EVERY copper port, on
 * BOTH OCP maps, reads `bmcr=0000 bmsr=0000` -- and NOT ONE "read timeout"
 * warning is emitted, because we never time out. We return zero and call it a
 * register value. That is a check that cannot fail, in the reassuring
 * direction, and it sits UPSTREAM of every candidate tested so far: the RxCDO
 * store width, the force-every-UTP-port policy, the GPHY reset and the OCP map
 * were all about WHAT we write to a PHY we were never reaching.
 *
 * ★ THE VENDOR DOES NOT POLL BUSY AT ALL. `_dal_rtl9603cvd_switch_phyPower_set`
 * does CMD -> mdelay(10) -> read RD -> modify -> WD -> CMD -> mdelay(10). A
 * flat settle, no handshake -- i.e. the vendor's own code treats BUSY as
 * unusable for this.
 *
 * ⚠ SINGLE VARIABLE, and reversible: `gphy_settle_us=0` restores the old
 * behaviour without a rebuild.
 */
static int gphy_wait(struct luna_eth *ep)
{
	int i;

	/* let the transaction START before asking whether it has finished. */
	if (gphy_settle_us)
		udelay(gphy_settle_us);

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

/* Program CFG_PHY_CTRL, and SAY what was there before: the previous value is
 * the measurement -- a silent write would destroy the evidence it rests on. */
/* ★★★ RELEASE THE RECEIVE FIFOs. MEASURED 2026-08-24, stock vs ours, the same
 * board within minutes, SWCORE 0x00048 = CFG_PCSXF (this chip's own chipdef:
 * RST_RXFIFO[13:10] | CFG_MIIRX_IPG[9:5] | CFG_PCSXF[4:1] | COL_10M[0]):
 *
 *      stock  0x00000000  ->  RST_RXFIFO = 0     no FIFO held in reset
 *      ours   0x00002006  ->  RST_RXFIFO = 0b1000   ONE held, and it is the
 *                                                   cabled port's
 *
 * We never wrote that register: the value is the bootloader's, and stock clears
 * it. Exactly the shape of the BASE_PHYAD defect found the same morning -- a
 * register nobody on our side had ever read.
 *
 * It explains what nothing else did: the PHY answers, the port reports LINK=1
 * with autoneg complete, and the switch's own per-port RX counter does not move
 * by a single frame for traffic this host demonstrably delivers to that port on
 * stock. A receive FIFO held in reset ingresses nothing and reports nothing.
 *
 * `rst_rxfifo` is a parameter, and -1 leaves the bootloader's value, which is
 * how this claim gets FALSIFIED instead of believed. The witness is NOT the
 * link -- that was already up. It is the switch MIB at SWCORE 0x32620 + 3*0x80
 * moving past the 2824 frames U-Boot's own TFTP left frozen there.
 */
static void eth_rxfifo_release(struct luna_eth *ep)
{
	u32 was, now;

	if (!ep->c->cfg_pcsxf || rst_rxfifo < 0)
		return;

	was = sw_rd(ep, ep->c->cfg_pcsxf);
	if (((was >> 10) & 0xf) == (u32)(rst_rxfifo & 0xf)) {
		dev_info(ep->dev,
			 "CFG_PCSXF(%#05x) = %08x already has RST_RXFIFO %#x -- left alone\n",
			 ep->c->cfg_pcsxf, was, rst_rxfifo & 0xf);
		return;
	}
	sw_wr(ep, ep->c->cfg_pcsxf,
	      (was & ~(0xfu << 10)) | ((u32)(rst_rxfifo & 0xf) << 10));
	now = sw_rd(ep, ep->c->cfg_pcsxf);
	dev_info(ep->dev,
		 "CFG_PCSXF(%#05x): RST_RXFIFO %#x -> %#x (%08x -> %08x)\n",
		 ep->c->cfg_pcsxf, (was >> 10) & 0xf, rst_rxfifo & 0xf, was, now);
}

static void eth_phy_ctrl_apply(struct luna_eth *ep)
{
	u32 was;

	if (!ep->c->cfg_phy_ctrl || base_phyad < 0)
		return;

	was = sw_rd(ep, ep->c->cfg_phy_ctrl);
	if ((was & 0x1f) == (u32)(base_phyad & 0x1f)) {
		dev_info(ep->dev,
			 "CFG_PHY_CTRL(%#05x) = %08x already carries BASE_PHYAD %u -- left alone\n",
			 ep->c->cfg_phy_ctrl, was, base_phyad & 0x1f);
		return;
	}
	sw_wr(ep, ep->c->cfg_phy_ctrl, (was & ~0x1fu) | (base_phyad & 0x1f));
	dev_info(ep->dev,
		 "CFG_PHY_CTRL(%#05x): BASE_PHYAD %u -> %u (%08x -> %08x)\n",
		 ep->c->cfg_phy_ctrl, was & 0x1f, base_phyad & 0x1f,
		 was, sw_rd(ep, ep->c->cfg_phy_ctrl));
}

/* Writing this re-applies BASE_PHYAD and dumps the survey again, so a whole
 * sweep costs one boot instead of one build each. */
static int resurvey_set(const char *val, const struct kernel_param *kp)
{
	if (!survey_ep)
		return -ENODEV;
	eth_phy_ctrl_apply(survey_ep);
	eth_phy_survey(survey_ep);
	return 0;
}
static const struct kernel_param_ops resurvey_ops = { .set = resurvey_set };
module_param_cb(resurvey, &resurvey_ops, NULL, 0200);
MODULE_PARM_DESC(resurvey, "write anything: re-apply base_phyad and re-dump the PHY survey");

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

		/* ★★★ THE VENDOR NEVER ASSERTS THIS, AND WE NEVER RE-INIT AFTER IT.
		 * Measured 2026-08-23 by EMULATING `dal_rtl9603cvd_switch_init`
		 * (0x802dcd04) under Unicorn with MMIO hooks -- it ran to
		 * completion, returning 0, and logged 457 SWCORE writes, 442
		 * reads, 209 internal-PHY writes and 40 same-value rewrites.
		 * `SOFTWARE_RST.CMD_GPHY_RST_PS` is NOT among them.
		 * Three independent confirmations: absent from that trace; the
		 * 9603CVD DAL never writes the field at all (only the interactive
		 * `switch reset ... gphy` diag command does); and where the 9607C
		 * DOES write it, in `_dal_rtl9607c_gen2_switch_phyCal`, it pairs it
		 * with mdelay(20) AND a full re-programming of every PHY.
		 * We paired it with msleep(50) and nothing: no patch, no cal, no
		 * vendor power sequence -- and it is an `sw_or` with no clear.
		 * That is the shape this project's anti-repeat list names first: a
		 * cal/bring-up is an FSM that must RUN and COMPLETE, not a reset to
		 * assert. It explains every symptom at once -- all copper PHYs
		 * dead, ZERO ingress on EVERY port (the switch MIB frozen on all
		 * six), and counters that freeze exactly when our driver takes
		 * over, while U-Boot's configuration worked because U-Boot never
		 * asserts it.
		 * ⚠ THIS IS THE FALSIFIER, NOT A PROVEN FIX. Removing one line and
		 * re-measuring RX is the whole experiment: RX still dead ⇒
		 * refuted, and the next candidates are already ranked (the FE-PHY
		 * OCP map, the FEPHY_POLL polarity, the three absent patch loads).
		 */
		if (gphy_reset) {
			sw_or(ep, ep->c->swcore_rst, CMD_GPHY_RST_PS);
			msleep(50);
		}

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
	sw_or(ep, ep->c->sw_map->gphy_misc, BIT(0));		/* patch-done sticky	*/

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
/* The bring-up default and its predicate are FAMILY facts and live in
 * luna_eth_regs.h.  They were defined here first; the copy was removed on
 * 2026-08-28 after the RTL9602C driver was found shipping that very address,
 * because its own byte-identical IDR reader had never received this refusal. */

/* Thin wrappers over the family helpers, kept at their own names so the call
 * sites and this diff stay small. The BODIES live in luna_eth_regs.h. */
static void eth_get_hwaddr(struct luna_eth *ep, u8 *mac)
{
	luna_idr_get(ep->base, mac);
}

static void eth_set_hwaddr(struct luna_eth *ep, const u8 *mac)
{
	luna_idr_set(ep->base, mac);
}

/* ---- rings ---------------------------------------------------------------- */
/* Allocate a fresh RX skb, stream-map it, and arm the descriptor on it. */
/* The body is the FAMILY's (luna_eth_regs.h): both drivers had it character
 * for character apart from the struct that reached `->rx_ring`.  The wrapper
 * keeps the old name and signature so every call site is untouched. */
static int eth_refill(struct luna_eth *ep, unsigned int idx)
{
	return luna_rx_refill(ep->ndev, ep->dev, ep->rx_ring, ep->rx_skb,
				 ep->rx_buf_dma, idx, RX_RING_SIZE, RX_BUF_SIZE);
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
	survey_ep = ep;
	/* BEFORE the power-up: the power-up itself talks to the PHYs through the
	 * very bus this register addresses. */
	eth_phy_ctrl_apply(ep);
	eth_rxfifo_release(ep);
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
	for (p = 0; p <= ep->c->sw_map->cpu_port; p++) {
		u32 reg = ep->c->sw_map->lut_unkn_sa + (p / 16) * 4;

		/* unknown-source-MAC action 0 = learn + forward */
		sw_wr(ep, reg, sw_rd(ep, reg) & ~(3u << ((p % 16) * 2)));
	}
	/*
	 * ★ THE PON PORT IS NOT A FLOOD DESTINATION (2026-08-27).
	 * Flooding LAN broadcast, unknown multicast and unknown unicast out of
	 * the fibre port sends every ARP and every DHCP DISCOVER on the LAN
	 * upstream to the OLT. Our RTL9602C sibling excludes it for exactly this
	 * reason. It is invisible from the ONU -- only the OLT or a fibre capture
	 * would ever see it -- which is why it is fixed now rather than after
	 * ranging works and it becomes a real leak.
	 * `pon_port` is only meaningful on a chip that declares one; the mask is
	 * left untouched where force_pon_ablty is 0, so the RTL9607C engineering
	 * board sees the identical write it saw before.
	 */
	{
		u32 flood = ep->c->sw_map->port_mask;

		if (ep->c->force_pon_ablty)
			flood &= ~BIT(ep->c->sw_map->pon_port);
		sw_or(ep, ep->c->sw_map->bc_flood, flood);
		sw_or(ep, ep->c->sw_map->unkn_mc_flood, flood);
		sw_or(ep, ep->c->sw_map->unkn_uc_flood, flood);
	}
	/* SW_SRC_PORT_PERMIT (0x1C114) is a per-source-port EGRESS-FILTER ENABLE
	 * (EN, 1 bit/port), NOT a permit bitmap: EN=1 turns on source-port egress
	 * filtering and DROPS the forwarded frame after lookup/flood selection. The
	 * working firmware leaves it 0 (no filtering = forward). Writing all-ones here
	 * silently dropped every LAN->CPU frame (port RX climbed, CPU RX stayed flat).
	 * Correct forwarding-permissive value is 0. */
	sw_wr(ep, ep->c->sw_map->src_permit, 0x00000000);

	/* 4a2. per-port unknown-DA lookup-miss action = FORWARD(0). We only set the
	 *      unknown-SOURCE action above; the unknown-DESTINATION action must also
	 *      forward, else post-ARP unicast / IPv6-ND to a not-yet-learned MAC is
	 *      dropped instead of flooded. Clear the 2-bit field for ports 0..10. */
	sw_wr(ep, ep->c->sw_map->lut_unkn_uc_da,  sw_rd(ep, ep->c->sw_map->lut_unkn_uc_da)  & ~DA_ACT_PORTS);
	sw_wr(ep, ep->c->sw_map->unkn_l2_mc,  sw_rd(ep, ep->c->sw_map->unkn_l2_mc)  & ~DA_ACT_PORTS);
	sw_wr(ep, ep->c->sw_map->unkn_ip4_mc, sw_rd(ep, ep->c->sw_map->unkn_ip4_mc) & ~DA_ACT_PORTS);
	sw_wr(ep, ep->c->sw_map->unkn_ip6_mc, sw_rd(ep, ep->c->sw_map->unkn_ip6_mc) & ~DA_ACT_PORTS);

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
		unsigned int cp = ep->c->sw_map->cpu_port;
		unsigned int per = ep->c->piso_per_word;
		u32 reg = ep->c->sw_map->piso_base + (cp / per) * 4;
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
		if (p == ep->c->sw_map->cpu_port) {
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

	/*
	 * ★★★ 5b. THE PON PORT IS NOT A UTP PORT, AND THE RULE ABOVE DOES NOT
	 * REACH IT.  Step 5 deliberately forces nothing on ports 0..last except
	 * the CPU port, because overriding a live auto-negotiated copper link
	 * kills CPU->LAN egress.  The PON port has NO PHY behind it -- there is
	 * nothing to auto-negotiate with, and its P_ABLTY therefore never leaves
	 * the reset value on its own.
	 *
	 * STOCK IS THE ORACLE AND IT WAS READ, not reasoned about.  On the G24W,
	 * SWCORE 0x180..0x1fc, stock vs ours, same board, same bench:
	 *
	 *     FORCE_P_ABLTY[4]     stock 0x00000016   ours 0x00000000
	 *     ABLTY_FORCE_MODE[4]  stock 0x0000bfff   ours 0x00000000
	 *
	 * 0x16 is ABLTY_1G_FULL_LINK and 0xbfff is ABLTY_CPU_FORCE -- the SAME
	 * pair stock applies to the CPU port, which is the other portless
	 * internal link.  So this is not a new recipe, it is the recipe already
	 * in this function applied to the port it was always missing.
	 *
	 * ⚠ WHAT THIS DOES AND DOES NOT CLAIM.  It makes the switch side of the
	 * PON path forward; it says NOTHING about ranging, and this board's PON
	 * MAC is at O1.  A silent switch port would have hidden a working PON
	 * MAC behind it, which is why it is repaired now rather than after.
	 */
	if (ep->c->force_pon_ablty) {
		unsigned int pp = ep->c->sw_map->pon_port;
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
		sw_or(ep, ep->c->cpu_tag_insert, BIT(0));
		sw_or(ep, ep->c->cpu_tag_aware, BIT(0));
	}

	dev_info(ep->dev,
		 "switch: %s open-L2 up (cpu-port %u of 0..%u, src-permit=%08x, cpu-ablty=%04x)\n",
		 ep->c->name, ep->c->sw_map->cpu_port, ep->c->last_port,
		 sw_rd(ep, ep->c->sw_map->src_permit),
		 sw_rd(ep, SW_P_ABLTY(ep, ep->c->sw_map->cpu_port)));
	/* Baseline real-link snapshot; the periodic diag (armed at open) then shows
	 * which port's genuine link comes up + rxpkts climb under host traffic. */
	eth_diag_dump(ep);
}

/* ---- MAC engine ----------------------------------------------------------- */
/* The body is the family's (luna_eth_regs.h): both drivers had it, identical
 * but for the struct that reached `->base`. */
static void eth_hw_stop(struct luna_eth *ep)
{
	luna_eth_hw_stop(ep->base);
}

/* GMAC0 IP-block power-cycle: the multi-ring fetch engine only latches its ring
 * state across this reset edge, so it must run before the descriptor program. */
/* The body and the hang warning are the FAMILY's (luna_eth_regs.h). */
static void eth_ipsel_cycle(void)
{
	luna_ipsel_cycle_gmac0();
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
	/* Was this file's own spelling of the family packing -- the THIRD copy
	 * (rtl9602c_eth.c spells it twice via the same flowcore call). */
	desnum = luna_gmac_rxdesnum_pack(RX_RING_SIZE, TH_ON_VAL, TH_OFF_VAL);
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
	ep_wr(ep, R_RxCDO0, luna_gmac_rxcdo_pack(RX_RING_SIZE));
	/* route every RX class to ring 0. */
	{
		unsigned int k;

		for (k = 0; k < 7; k++)
			ep_wr(ep, R_RRING_ROUTE + k * 4, 0);
	}

	/* MSR(0x58) top byte -- see the msr_top param note.  0xf0 is stock's
	 * value and it WEDGES our datapath. */
	ep_wr(ep, R_MSR,
	      (ep_rd(ep, R_MSR) & 0x00ffffff) | ((msr_top & 0xffu) << 24));
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
			/*
			 * ★★★ THE DESCRIPTOR, BESIDE THE BYTES (2026-08-27).
			 *
			 * This driver has NEVER read opts2/opts3 on RX. On the
			 * RTL9602C sibling those two words ARE the WAN demux:
			 * opts3[19:16] is the ingress switch port and
			 * opts3[31:20]==0x23e marks the PON-IP NIC drain, which
			 * is how `gpon0` tells a downstream frame from a LAN one.
			 * Whether the same layout holds on THIS die is unknown --
			 * the two chips' TX word3 layouts already differ (SID at
			 * [22:16] vs [6:0]), and assuming they matched cost this
			 * project the US-OMCI wall once already.
			 *
			 * It is printed HERE, gated by the existing rx_dump, so
			 * the answer arrives on a boot that was going to happen
			 * anyway instead of costing one of its own. The bytes and
			 * the descriptor on the same frame are what make it a
			 * measurement: the DA tells you what the frame IS, and
			 * opts3 tells you what the silicon SAID it was.
			 *
			 * ⚠ The in-band 0x8899 tag is NOT present on this board
			 * today (every dump in results/ is [prefix][DA][SA][type]),
			 * so the excision branch below has never fired and the tag
			 * is not an available demux source without enabling the
			 * switch's trap-tag insert first.
			 */
			dev_info(ep->dev,
				 "rx0 desc: opts1=%08x opts2=%08x opts3=%08x len=%u (src_port_if_9602c_layout=%u reason=%u)\n",
				 opts1, ep->rx_ring[i].opts2, ep->rx_ring[i].opts3,
				 len, (ep->rx_ring[i].opts3 >> 16) & 0xf,
				 (ep->rx_ring[i].opts2 >> 21) & 0xff);
			print_hex_dump(KERN_INFO, "rx0: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, min_t(u32, len, 32), false);
		}

		/* Bad-frame verdict hoisted to luna_gmac_rx_frame_bad()
		 * (flowcore): the error mask and the header floor are passed,
		 * not re-spelled; the deliberately-lax floor (vs the 9602C
		 * shell's min-Ethernet-frame bound) is documented there. */
		if (luna_gmac_rx_frame_bad(opts1, RXD_CRCERR | RXD_RCDF, len,
					   (u32)rx_prefix + ETH_HLEN,
					   RX_BUF_SIZE)) {
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
			/* Tag classifier hoisted to flowcore; the excision
			 * memmove/skb_pull below stay -- buffer surgery, not
			 * a decision. */
			if (luna_gmac_rx_cpu_tag_present(skb->data, skb->len,
							 RTL_CPU_TAG_LEN)) {
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
	    !luna_gmac_tx_ring_full(ep->tx_head, ep->tx_dirty,
				     TX_RING_SIZE, 0))
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
	/* Ring-space test hoisted to luna_gmac_tx_ring_full() (flowcore): the
	 * ring size is passed, and the reserve is ZERO because this shell has a
	 * single producer on ring 0.  The reserve argument exists because the
	 * family's other Ethernet shell shares this ring with an OMCI injector
	 * and must stop that many slots early -- one of its five spellings was
	 * measured carrying the wrong bound (rtl9602c_eth.c:1690-1702), which is
	 * why the expression now lives in one place.  The wake-queue twin in
	 * eth_tx_reclaim() is still spelled by hand and INVERTED; converting it
	 * is the point of the helper and is left to the pass that may touch that
	 * function. */
	if (luna_gmac_tx_ring_full(ep->tx_head, ep->tx_dirty, TX_RING_SIZE, 0)) {
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
	/* Body word composed by luna_gmac_txd_word0() (flowcore); the flag set,
	 * the length mask and D_EOR are this shell's and are PASSED, never
	 * re-spelled there.  D_OWN is deliberately NOT part of the composed word:
	 * ownership is publish-order, so it is OR'd on at the store below, after
	 * the barrier that orders the body ahead of it. */
	opts1 = luna_gmac_txd_word0(D_FS | D_LS | D_TXCRC, len, TXD_LEN_MASK,
				    luna_gmac_slot_is_eor(i, TX_RING_SIZE),
				    D_EOR);
	wmb();				/* descriptor body before ownership */
	ep->tx_ring[i].opts1 = D_OWN | opts1;
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

	luna_eth_set_promisc(ep->base,
				 !!(ndev->flags & (IFF_PROMISC | IFF_ALLMULTI)));
}

/* Parse "xx:xx:xx:xx:xx:xx" into `out`. -> true on success.
 *
 * ★ It REFUSES anything that is not exactly six colon-separated octets, and it
 * refuses a multicast or all-zero address: a malformed parameter must leave the
 * fallback chain intact rather than half-programme an interface.
 */
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
	ep->sw = devm_ioremap(dev, SWCORE_PHYS, ep->c->sw_map->swcore_size);
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
	if (mac_param && luna_mac_from_param(mac_param, mac)) {
		/* ★ FIRST RUNG, and it is the only one that can carry a PER-UNIT
		 * value on this product today. Announced, because a MAC that
		 * arrived from outside the device must be auditable in the log. */
		eth_hw_addr_set(ndev, mac);
		dev_info(dev, "MAC %pM taken from the `mac=` boot parameter\n", mac);
	} else if (of_get_ethdev_address(dev->of_node, ndev)) {
		eth_get_hwaddr(ep, mac);
		if (is_valid_ether_addr(mac) && !luna_mac_is_bringup_default(mac)) {
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
 * GPON OMCI glue stubs — the shared GPON FSM (gpon-rtl960x.c) references these
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
