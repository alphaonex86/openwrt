/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register facts for the Ethernet MAC and switch core on the Realtek Luna
 * family (RTL9602C, RTL9603CVD, and the siblings that follow).
 *
 * ★ WHY THIS FILE EXISTS, AND WHAT IT MEASURED.  The family had TWO Ethernet
 * drivers -- rtl9602c_eth.c for the X111W and rtl960x_eth.c for the G24W --
 * each carrying its own copy of the register map.  Compared symbol by symbol on
 * 2026-08-28, with the comments stripped so that a re-worded comment could not
 * masquerade as a different value:
 *
 *     34 symbols  IDENTICAL on both chips   -> family facts, and they live here
 *      4 symbols  genuinely per-chip        -> struct rtl960x_sw_map, below
 *
 * That ratio is the argument for the whole port strategy: a new Luna board owes
 * a TABLE, not a driver.  Two copies of 34 agreeing constants is not redundancy,
 * it is two chances to edit one of them.
 *
 * ★★ THE PER-CHIP FOUR ARE NOT A MISTAKE, AND THEY ARE THE DANGEROUS KIND.
 * The switch LUT block MOVED between the two silicon revisions: the RTL9603CVD
 * inserted registers, so everything from LUT_UNKN_UC_DA_CTRL onward shifted.
 * Confirmed on 2026-08-28 from each chip's OWN chipdef in the vendor SDK, which
 * agrees with what both drivers already had -- two independent tiers:
 *
 *     register              rtl9602c   rtl9603cvd
 *     LUT_UNKN_SA_CTRL       0x1C004     0x1C004    (unmoved)
 *     LUT_UNKN_UC_DA_CTRL    0x1C008     0x1C00C    (+4)
 *     UNKN_L2_MC             0x1C010     0x1C018    (+8)
 *     UNKN_IP4_MC            0x1C014     0x1C01C    (+8)
 *     UNKN_IP6_MC            0x1C018     0x1C020    (+8)
 *     LUT_BC_FLOOD           0x1C020     0x1C028    (+8)
 *     LUT_UNKN_MC_FLOOD      0x1C024     0x1C02C    (+8)
 *     LUT_UNKN_UC_FLOOD      0x1C028     0x1C030    (+8)
 *
 * ⚠ READ THE FIRST AND LAST ROWS TOGETHER: the 9602C's LUT_UNKN_UC_FLOOD and
 * the 9603CVD's LUT_BC_FLOOD are BOTH 0x1C028.  Using one chip's constant on
 * the other's silicon does not fault and does not read back wrong -- it
 * configures unicast flooding when broadcast flooding was meant.  That is the
 * same shape as the CFG_PHY_CTRL defect that cost this project weeks, and it is
 * exactly why these four are a table and not a #define.
 */
#ifndef _RTL960X_ETH_REGS_H
#define _RTL960X_ETH_REGS_H

#include <linux/types.h>
#include <linux/bits.h>

/* ───────────────────────── family-invariant facts ───────────────────────── */
/* MAC block, offsets from the MAC base. */
#define R_IDR0			0x00	/* station MAC [0:3], MSB first */
#define R_IDR4			0x04	/* station MAC [4:5] in [31:16] */
#define R_TCR			0x40	/* TX control */
#define R_RCR			0x44	/* RX control (bit0 = accept-all-physical) */
#define R_CPUTAGCR		0x48	/* CPU-tag insert config */
#define R_CONFIG		0x4C
#define R_CPUTAG1CR		0x50
#define R_IMR			0x3c	/* 16-bit RX/TX IRQ mask (stock operating = 0xf835) */
#define R_ISR			0x3e	/* 16-bit RX/TX IRQ status, write-1-to-clear */
#define R_IMR0			0xd0	/* 32-bit per-ring TX-completion mask (stock = 0x3f) */
#define R_ISR1			0xd8	/* 32-bit per-ring TX-completion status, W1C */
#define R_RxDesNum		0x1430	/* RX ring0 size + flow-control thresholds */
#define R_IO_CMD		0x1434	/* DMA enable + per-ring TX kick (bit0 = ring0) */
#define R_IO_CMD1		0x1438

/* Interrupt bit groups, as stock programs them. */
#define IMR_RX_BITS		0xf835	/* RX-OK + RX-error + ring descriptor-unavailable */
#define IMR0_TX_BITS		0x3f	/* the 6 per-ring TX-completion IRQs */

/* Descriptor ownership and framing bits, shared by TX and RX rings. */
#define D_OWN			BIT(31)	/* 1 = owned by the DMA engine */
#define D_EOR			BIT(30)	/* end of ring (wrap) */
#define D_FS			BIT(29)	/* first segment */
#define D_LS			BIT(28)	/* last segment */
#define D_TXCRC			BIT(23)	/* TX: append FCS */
#define RXD_CRCERR		BIT(27)	/* RX: CRC error */
#define RXD_LEN_MASK		0x1fff	/* RX length, low bits of opts1 */
#define TXD_LEN_MASK		0x1ffff	/* TX length */

/* Ring geometry.  Not silicon: our own sizing, but identical on both drivers,
 * so it is a family choice rather than a per-board one. */
#define RX_RING_SIZE		64
#define TX_RING_SIZE		64
#define RX_BUF_SIZE		2048

/* RX flow-control assert / de-assert thresholds. */
#define TH_ON_VAL		0x10
#define TH_OFF_VAL		0x30

/* Switch core: the VLAN block did NOT move between these two revisions. */
#define SW_VLAN_ACCEPT		0x13000	/* per-port accept-frame-type (0 = accept all) */
#define SW_VLAN_CTRL		0x13008
#define SW_VLAN_PB_VID		0x1300C	/* per-port default VID (PVID), stride 4 */

/* SoC glue that is at the same physical address on every Luna part seen here. */
#define SWCORE_PHYS		0x1B000000UL
#define SOC_IP_SEL		((void __iomem *)0xb8000600ul)	/* per-engine clock/reset */

/* ─────────────────────────── the per-chip table ─────────────────────────── */
/**
 * struct rtl960x_sw_map - the switch-core facts that differ between Luna chips
 * @swcore_size:      ioremap length; must cover the highest block the driver
 *                    touches (MIB, PISO).  Too small and those reads land
 *                    outside the mapping instead of failing loudly.
 * @lut_unkn_sa:      unknown-SA action, 2 bits per port
 * @lut_unkn_uc_da:   per-port unknown-UC DLF action, 16 bits per port
 * @unkn_l2_mc:       unknown L2 multicast action
 * @unkn_ip4_mc:      unknown IPv4 multicast action
 * @unkn_ip6_mc:      unknown IPv6 multicast action
 * @bc_flood:         broadcast flood, one bit per port
 * @unkn_mc_flood:    unknown-multicast flood, one bit per port
 * @unkn_uc_flood:    unknown-unicast flood, one bit per port
 *
 * A new chip adds ONE instance here and nothing else.  Every field is an
 * absolute offset within the switch core, never a delta from a sibling: a
 * "+8 from the 9602C" table is a table that silently follows the wrong chip the
 * day a third revision moves only half the block.
 */
struct rtl960x_sw_map {
	u32 swcore_size;
	u32 lut_unkn_sa;
	u32 lut_unkn_uc_da;
	u32 unkn_l2_mc;
	u32 unkn_ip4_mc;
	u32 unkn_ip6_mc;
	u32 bc_flood;
	u32 unkn_mc_flood;
	u32 unkn_uc_flood;
};

static const struct rtl960x_sw_map rtl9602c_sw_map = {
	.swcore_size	= 0x40000,	/* must cover MIB @0x32000 + PISO @0x27000 */
	.lut_unkn_sa	= 0x1C004,	/* unmoved: the SAME offset on both chips */
	.lut_unkn_uc_da	= 0x1C008,
	.unkn_l2_mc	= 0x1C010,
	.unkn_ip4_mc	= 0x1C014,
	.unkn_ip6_mc	= 0x1C018,
	.bc_flood	= 0x1C020,
	.unkn_mc_flood	= 0x1C024,
	.unkn_uc_flood	= 0x1C028,
};

static const struct rtl960x_sw_map rtl9603cvd_sw_map = {
	.swcore_size	= 0x43000,
	.lut_unkn_sa	= 0x1C004,
	.lut_unkn_uc_da	= 0x1C00C,
	.unkn_l2_mc	= 0x1C018,
	.unkn_ip4_mc	= 0x1C01C,
	.unkn_ip6_mc	= 0x1C020,
	.bc_flood	= 0x1C028,
	.unkn_mc_flood	= 0x1C02C,
	.unkn_uc_flood	= 0x1C030,
};

/* The RTL9607C's eight LUT offsets were CROSS-READ from its own chipdef on
 * 2026-08-28 and are identical to the RTL9603CVD's, every one of them -- so
 * rtl960x_eth.c serving both chips from one constant set was correct, and this
 * table records that rather than leaving it as an assumption.
 *
 * ⚠ swcore_size is NOT from the chipdef: it is an ioremap LENGTH, a decision
 * about how much of the block this driver touches, not a silicon fact.  It
 * carries the value that driver has been using.
 */
static const struct rtl960x_sw_map rtl9607c_sw_map = {
	.swcore_size	= 0x43000,
	.lut_unkn_sa	= 0x1C004,
	.lut_unkn_uc_da	= 0x1C00C,
	.unkn_l2_mc	= 0x1C018,
	.unkn_ip4_mc	= 0x1C01C,
	.unkn_ip6_mc	= 0x1C020,
	.bc_flood	= 0x1C028,
	.unkn_mc_flood	= 0x1C02C,
	.unkn_uc_flood	= 0x1C030,
};

#endif /* _RTL960X_ETH_REGS_H */
