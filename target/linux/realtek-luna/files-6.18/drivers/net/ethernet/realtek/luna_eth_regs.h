/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register facts for the Ethernet MAC and switch core on the Realtek Luna
 * family (RTL9602C, RTL9603CVD, and the siblings that follow).
 *
 * ★ WHY THIS FILE EXISTS, AND WHAT IT MEASURED.  The family had TWO Ethernet
 * drivers -- rtl9602c_eth.c for the X111W and luna_eth.c for the G24W --
 * each carrying its own copy of the register map.  Compared symbol by symbol on
 * 2026-08-28, with the comments stripped so that a re-worded comment could not
 * masquerade as a different value:
 *
 *     34 symbols  IDENTICAL on both chips   -> family facts, and they live here
 *      4 symbols  genuinely per-chip        -> struct luna_sw_map, below
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
#ifndef _LUNA_ETH_REGS_H
#define _LUNA_ETH_REGS_H


#include <linux/etherdevice.h>	/* ETH_ALEN, ether_addr_equal */
#include <linux/delay.h>	/* udelay */
#include <linux/dma-mapping.h>	/* dma_map_single, DMA_FROM_DEVICE */
#include <linux/netdevice.h>	/* netdev_alloc_skb */
#include <linux/skbuff.h>	/* struct sk_buff, dev_kfree_skb_any */
#include <linux/io.h>		/* ioread32 / iowrite32 */
#include <linux/kernel.h>	/* sscanf */
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
#define RXD_RCDF		BIT(24)	/* RX: DMA error.  RCDF is the SILICON's own
					 * name (the vendor NIC driver's rx_info
					 * opts1 bitfield, `rcdf:1;//24`, same in
					 * the 4.4 and 5.10 SDK drops).  This one
					 * bit carried TWO shell names, RXD_DMAERR
					 * (luna_eth.c, invented -- in no vendor
					 * source) and RXD_RCDF (rtl9602c_eth.c),
					 * until 2026-09-03. */
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

/* Switch-core registers the two chipdefs place at the SAME address, so they are
 * family constants rather than table fields.  Cross-read 2026-08-28 from each
 * chip's own reg_list.c; the silicon's own names are kept, because a name that
 * follows the silicon is one a reader can look up. */
#define SW_CHIP_INFO		0x10004	/* CHIP_INFO: low 16 bits = the GPHY variant */
#define SW_METER_TB_CTRL	0x25000	/* METER_TB_CTRL: meter tick/token config */
#define SW_VLAN_EGRESS_TAG	0x2A000	/* VLAN_EGRESS_TAG */
#define SW_STAT_PORT_TX_MIB	0x32000	/* STAT_PORT_TX_MIB, +0x80 per port */

/* Switch core: the VLAN block did NOT move between these two revisions. */
/* ★ NAMES BOTH CHIPDEFS AGREE ON, for the SWCORE registers
 * rtl9602c_datapath_tables_init writes (2026-09-04).  It spelled these as
 * bare hex; `bare_offset_chip_audit.py` resolves each address in the
 * rtl9602c AND rtl9603cvd chipdefs and only a SAME verdict is written here.
 * The SW_ prefix is this header's own convention for the SWCORE window.
 * ⚠ SW_VLAN_PORT_ACCEPT_FRAME_TYPE below is an ABBREVIATION the tree established earlier
 * (the chipdefs call 0x13000 VLAN_PORT_ACCEPT_FRAME_TYPE); it is left as it
 * is because renaming a name the tree already uses is a separate question
 * from naming a number that had none. */
#define SW_VLAN_INGRESS			0x13004
#define SW_LUT_CFG			0x17000
#define SW_LUT_AGEOUT_CTRL		0x17004
#define SW_LUT_UNMATCHED_SA_CTRL	0x1C000
#define SW_LUT_UNKN_SA_CTRL		0x1C004
#define SW_PISO_PORT			0x27000
/* ⚠ AN ARRAY, NOT FOUR REGISTERS.  Only element 0 carries a chipdef name;
 * the two maps name DIFFERENT later elements (PISO_EXT at 0x27008 on the
 * RTL9602C, at 0x2700c on the RTL9603CVD) because the array's extent
 * differs, not because anything moved.  Treating one element as a
 * standalone register made the G24W write 0x2700c twice and never write
 * 0x27008 -- so the BASE is named and the elements are arithmetic. */
#define SW_PISO_PORT_STRIDE	4u
#define SW_VLAN_PORT_ACCEPT_FRAME_TYPE		0x13000	/* per-port accept-frame-type (0 = accept all) */
#define SW_VLAN_CTRL		0x13008
#define SW_VLAN_PB_VID		0x1300C	/* per-port default VID (PVID), stride 4 */

/* SoC glue that is at the same physical address on every Luna part seen here. */
#define SWCORE_PHYS		0x1B000000UL
#define SOC_IP_SEL		((void __iomem *)0xb8000600ul)	/* per-engine clock/reset */

/* ─────────────────────────── the per-chip table ─────────────────────────── */
/**
 * struct luna_sw_map - the switch-core facts that differ between Luna chips
 * @src_permit:       L2_SRC_PORT_PERMIT -- egress FILTER ENABLE, permissive
 *                    value is 0 (writing all-ones reflects every frame back
 *                    out its own ingress port; that was a real broadcast loop).
 *                    ⚠ ON THE RTL9602C THIS ADDRESS WAS WRONG ONCE, and the
 *                    note travels with the value: it was 0x1C114, which is
 *                    QOS_PB_PRI on that chip, so the CPU port's ingress permit
 *                    was never set and the fabric dropped every CPU-injected
 *                    frame after DMA -- TX counter climbing, nothing egressing.
 * @piso_base:        per-port egress-forward (isolation) matrix
 * @cpu_port:         the switch port this GMAC is
 * @pon_port:         the fibre port (no copper PHY behind it)
 * @port_mask:        flood/member mask covering every port on this chip
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
struct luna_sw_map {
	/* ★ PORT NUMBERS ARE PER-CHIP AND WERE HALF-TABULATED.  luna_eth.c
	 * already carried pon_port/cpu_port per chip while rtl9602c_eth.c
	 * hardcoded RTL9602C_PON_PORT=2 and SW_CPU_PORT=3 -- the same shape as
	 * the MSR value one sibling made tunable and the other did not.  The
	 * numbers differ genuinely: PON is port 2, 4 and 5 on the three chips. */
	u32 src_permit;
	u32 piso_base;
	u8  cpu_port;
	u8  pon_port;
	u32 port_mask;
	u32 swcore_size;
	u32 lut_unkn_sa;
	u32 lut_unkn_uc_da;
	u32 unkn_l2_mc;
	u32 unkn_ip4_mc;
	u32 unkn_ip6_mc;
	/* ★★ CFG_UNHIOL, and it is IN THE TABLE for a measured reason: on the
	 * RTL9602C this address is CFG_UNHIOL (bit0 = IPG_COMPENSATION), and on
	 * the RTL9603CVD the SAME address is CPU_TAG_AWARE -- two different
	 * blocks, one number.  A family #define here would configure CPU tagging
	 * where IPG compensation was meant, without faulting and without reading
	 * back wrong: the CFG_PHY_CTRL defect, re-created.  0 = this chip's
	 * bring-up does not touch it. */
	/* ★ WRAP_GPHY_MISC, bit0 = "PHY patch done" -- the sticky the stock
	 * switch init asserts at completion.  THREE addresses for one job
	 * (9602C 0x110, 9607C 0x114, 9603CVD 0xEC), so it is per-chip data.
	 * It lives HERE and not in luna_eth_chip so that both Luna ethernet
	 * drivers read the SAME number: the family driver reaches it through
	 * its `sw_map` pointer, exactly as it already does for the port
	 * numbers, and a correction lands once. */
	/* ★ PER-PORT ABILITY TRIO -- per chip, and DEMONSTRABLY so: the RTL9603CVD
	 * puts force_ablty at 0x198 and p_ablty at 0x1B8, while the RTL9602C's own
	 * driver puts its force-ability array at 0x180 and its force-MODE array at
	 * 0x1B4.  The numbers overlap between chips without meaning the same
	 * thing, which is the third register block today found to do that.
	 * 0 = this chip's driver does not use that member. */
	u32 force_ablty;	/* + 4*port: forced ability values	*/
	u32 p_ablty;		/* + 4*port: LIVE ability, read-only	*/
	u32 ablty_force;	/* + 4*port: which fields are forced	*/
	u32 gphy_misc;
	u32 cfg_unhiol;
	u32 bc_flood;
	u32 unkn_mc_flood;
	u32 unkn_uc_flood;
};

static const struct luna_sw_map rtl9602c_sw_map = {
	.force_ablty	= 0x00180,
	.p_ablty	= 0,		/* this driver never reads it */
	.ablty_force	= 0x001B4,
	.gphy_misc	= 0x00110,
	.cfg_unhiol	= 0x23040,	/* CFG_UNHIOL on THIS chip -- see the field */
	.src_permit	= 0x1C088,
	.piso_base	= 0x27000,
	.cpu_port	= 3,
	.pon_port	= 2,
	.port_mask	= 0xf,
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

static const struct luna_sw_map rtl9603cvd_sw_map = {
	.force_ablty	= 0x00198,
	.p_ablty	= 0x001B8,
	.ablty_force	= 0x001DC,
	.gphy_misc	= 0x000EC,
	.cfg_unhiol	= 0,		/* this chip's bring-up does not touch it */
	.src_permit	= 0x1C0B0,
	.piso_base	= 0x27000,
	.cpu_port	= 5,
	.pon_port	= 4,
	.port_mask	= GENMASK(5, 0),
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
 * luna_eth.c serving both chips from one constant set was correct, and this
 * table records that rather than leaving it as an assumption.
 *
 * ⚠ swcore_size is NOT from the chipdef: it is an ioremap LENGTH, a decision
 * about how much of the block this driver touches, not a silicon fact.  It
 * carries the value that driver has been using.
 */
static const struct luna_sw_map rtl9607c_sw_map = {
	.force_ablty	= 0x001CC,
	.p_ablty	= 0x00200,
	.ablty_force	= 0x00238,
	.gphy_misc	= 0x00114,
	.cfg_unhiol	= 0,		/* this chip's bring-up does not touch it */
	/* ⚠ 0x1C114, NOT the RTL9603CVD's 0x1C0B0.  This was written as 0x1C0B0 by
	 * copying the sibling's value; the driver's own table and this chip's
	 * chipdef both say 0x1C114, and two guards caught it before it shipped.
	 * The number matters more than it looks: 0x1C114 is what the RTL9602C's
	 * own comment records as WRONG for THAT chip -- it is QOS_PB_PRI there, and
	 * using it left the CPU port's ingress permit unset so the fabric dropped
	 * every CPU-injected frame.  One address, correct on one die and
	 * catastrophic on another: that is what this table is for. */
	.src_permit	= 0x1C114,
	.piso_base	= 0x27000,
	.cpu_port	= 9,
	.pon_port	= 5,
	.port_mask	= GENMASK(9, 0),
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


/* ---- station address: the family's BRING-UP DEFAULT ----------------------- */
/* The address the silicon/bootloader leaves in IDR0/IDR4 when nothing has
 * programmed a real one.  It is a VALID unicast address, which is exactly why
 * it has to be NAMED: `is_valid_ether_addr()` accepts it, so a plain validity
 * check ships it and the random fallback never fires.
 *
 * ★ A FAMILY FACT, MEASURED ON TWO BOARDS OF DIFFERENT SILICON: the LANLY G24W
 * (RTL9603CVD, 2026-08-20) and the HSGQ X111W (RTL9602C, 2026-08-28 -- read
 * from the lab host's own ARP table while the board was answering pings, so
 * the two readings are independent of each other and of any one driver).
 * Two different chips holding ONE address is the whole problem: this lab runs
 * three ONUs on ONE L2 segment, and two of them sharing a MAC raises no error
 * anywhere -- it produces a switch that learns the address on whichever port
 * spoke last, and measurements on somebody else's bench that fail for no
 * visible reason.
 *
 * ⚠ IT LIVES IN THE FAMILY HEADER BECAUSE THE TWO-COPY VERSION ALREADY COST A
 * LIVE DEFECT.  The refusal was written into luna_eth.c alone, while the
 * RTL9602C's own driver carried a byte-identical IDR reader and a plain
 * `is_valid_ether_addr()` test -- so the 9602C shipped the shared default for
 * as long as the two copies existed, and nothing anywhere said so.  A family
 * fact kept in two chip shells is a repair with a delay fuse on it. */
#define LUNA_MAC_BRINGUP_DEFAULT	{ 0x00, 0xe0, 0x4c, 0x86, 0x70, 0x01 }

static inline bool luna_mac_is_bringup_default(const u8 *mac)
{
	static const u8 dflt[ETH_ALEN] = LUNA_MAC_BRINGUP_DEFAULT;

	return ether_addr_equal(mac, dflt);
}


/* ---- the station address in the MAC engine (IDR0/IDR4) -------------------- */
/* ★ SHARED BY TAKING THE **MAPPED BASE**, NOT THE DRIVER STRUCT.  Both Luna
 * ethernet drivers had a byte-identical pair of these, differing only in the
 * struct they dereferenced to reach `->base`.  That is the whole obstacle to
 * sharing driver code in this tree, and passing the io handle removes it: the
 * register layout is a FAMILY fact (R_IDR0/R_IDR4 above are already shared),
 * only the container was per-chip.
 *
 * ⚠ AND THE DUPLICATE PAIR IS WHY THE BRING-UP-DEFAULT REFUSAL BELOW REACHED
 * ONLY ONE OF THEM for eight days.  Two copies of one fact do not merely cost
 * lines; they cost the NEXT repair, which lands in whichever copy the author
 * happened to be reading. */
static inline void luna_idr_get(void __iomem *base, u8 *mac)
{
	u32 lo = ioread32(base + R_IDR0), hi = ioread32(base + R_IDR4);

	mac[0] = lo >> 24; mac[1] = lo >> 16; mac[2] = lo >> 8; mac[3] = lo;
	mac[4] = hi >> 24; mac[5] = hi >> 16;
}

static inline void luna_idr_set(void __iomem *base, const u8 *mac)
{
	iowrite32(((u32)mac[0] << 24) | ((u32)mac[1] << 16) |
		  ((u32)mac[2] << 8) | mac[3], base + R_IDR0);
	iowrite32(((u32)mac[4] << 24) | ((u32)mac[5] << 16), base + R_IDR4);
}

/* ---- a MAC handed in on the kernel command line --------------------------- */
/* THE FIRST RUNG of this project's declared precedence (bootarg -> DT/nvmem ->
 * random LAA).  On these products it is the ONLY rung that can carry a PER-UNIT
 * value today: no DTS in this target declares `mac-address`, and the real
 * address lives zlib-compressed inside the vendor MIB in the config partition,
 * which the kernel cannot read (no JFFS2, and an nvmem cell cannot express a
 * compressed value).  The suite hands it in from the BOARD's own declaration --
 * `rig/bootmode.py:_mac_bootarg`, keyed on that board's ETH_MAC_BOOTARG.
 *
 * ⚠ NO ADDRESS IS EVER SPELLED IN THE KERNEL.  A literal here, or in a DTS,
 * would hand the second unit of the same product the first one's identity. */
static inline bool luna_mac_from_param(const char *s, u8 *out)
{
	u8 v[ETH_ALEN];
	int n;

	if (!s || !*s)
		return false;
	n = sscanf(s, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
	if (n != ETH_ALEN || !is_valid_ether_addr(v))
		return false;
	memcpy(out, v, ETH_ALEN);
	return true;
}


/* ---- GMAC stop, and the promiscuous bit -----------------------------------
 * Two more bodies that were written twice, differing only in the struct they
 * dereferenced to reach `->base`.  Both are pure register work, so the (hwio)
 * conversion is the whole of it: take the mapped base.
 *
 * ★ THE COMMENT THAT EXPLAINS THE PROMISC BIT LIVED IN ONLY ONE OF THE TWO
 * COPIES, which is the quieter half of duplication: the code survives the
 * copy, the REASON does not, and the next reader of the poorer copy has to
 * re-derive it or guess. */
static inline void luna_eth_hw_stop(void __iomem *base)
{
	iowrite32(0, base + R_IO_CMD);
	iowrite32(0, base + R_IO_CMD1);
	iowrite16(0, base + R_IMR);
	iowrite32(0, base + R_IMR0);
	iowrite16(0xffff, base + R_ISR);
	iowrite32(0xffffffff, base + R_ISR1);
	udelay(10);
}

/* A bridge enslaving the CPU netdev sets promisc; accept-all-physical (RCR
 * bit0) is then REQUIRED to receive LAN-client frames whose DA is not our MAC.
 * Without it a bridged port silently forwards nothing it did not address. */
static inline void luna_eth_set_promisc(void __iomem *base, bool on)
{
	u32 rcr = ioread32(base + R_RCR);

	if (on)
		rcr |= BIT(0);
	else
		rcr &= ~BIT(0);
	iowrite32(rcr, base + R_RCR);
}


/* The DMA descriptor address bus window.  ZERO on this SoC, and it is a named
 * knob rather than a bare 0 for a measured reason:
 *
 * ⚠ OR-ING A NON-ZERO WINDOW INTO A DESCRIPTOR ADDRESS IS HARMFUL HERE.  It was
 * observed to be a NO-OP for TX egress and to DEGRADE RX -- `dma_alloc_coherent`
 * already yields correct bus addresses on this SoC (an artifact of its 1:1 map),
 * so or-ing the window CORRUPTS them.
 *
 * ★ THAT PARAGRAPH EXISTED IN ONE OF THE TWO COPIES ONLY.  Both drivers defined
 * this constant, one as 0x00000000u with the warning above and one as 0u with a
 * single line that says none of it.  The value survived being copied; the
 * measurement behind it did not. */
#define DMA_BUS_WINDOW		0x00000000u

/* ---- the DMA descriptors ---------------------------------------------------
 * ★ ONE TYPE, not two identical declarations.  Both Luna ethernet drivers
 * declared `struct rx_desc` and `struct tx_desc` with the same names and the
 * same layout, in their own files.  A duplicated TYPE is worse than a
 * duplicated function: the compiler checks each copy against itself, so the day
 * one of them gains a field the two silently describe different memory and the
 * engine reads a ring nobody wrote.
 *
 * The layout is the SILICON's, which is why it belongs to the family and not to
 * either chip: opts1 carries OWN/EOR and the buffer length, addr is the bus
 * address (with DMA_BUS_WINDOW already or'd in), opts2/opts3 carry the per-frame
 * classification words and opts4 the TX-only extension. */
struct rx_desc { u32 opts1, addr, opts2, opts3; };
struct tx_desc { u32 opts1, addr, opts2, opts3, opts4; };

/* ---- RX refill --------------------------------------------------------------
 * Arm ONE RX slot with a fresh skb.  Shared because both drivers had it
 * character for character apart from the struct they reached `->rx_ring`
 * through; the pieces are passed explicitly rather than behind a new container,
 * so neither driver has to change who owns its rings.
 *
 * ⚠ EOR IS SET ON THE LAST SLOT AND NOWHERE ELSE.  Getting that wrong does not
 * fault: the engine simply walks off the end of the ring into whatever follows
 * it, which is the quietest possible corruption.  `nr` is the ring's own entry
 * count, passed in, never a #define read from whichever driver compiled last. */
static inline int luna_rx_refill(struct net_device *ndev, struct device *dev,
				    struct rx_desc *ring, struct sk_buff **skbs,
				    dma_addr_t *dmas, unsigned int idx,
				    unsigned int nr, unsigned int buf_size)
{
	struct sk_buff *skb = netdev_alloc_skb(ndev, buf_size);
	dma_addr_t da;
	u32 opts1;

	if (!skb)
		return -ENOMEM;
	da = dma_map_single(dev, skb->data, buf_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, da)) {
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}
	skbs[idx] = skb;
	dmas[idx] = da;
	ring[idx].addr = da | DMA_BUS_WINDOW;
	ring[idx].opts2 = 0;
	ring[idx].opts3 = 0;
	opts1 = D_OWN | buf_size;
	if (idx == nr - 1)
		opts1 |= D_EOR;
	ring[idx].opts1 = opts1;
	return 0;
}


/* ---- the GMAC0 clock/reset gate -------------------------------------------
 * ★ ONE NAME FOR ONE BIT.  It was `IPSEL_GMAC0` in one driver and
 * `IPSEL_EN_GMAC0` in the other, both BIT(1) of the same SOC_IP_SEL word this
 * header already declared.  Two spellings of one register field is the shape
 * this project has paid for repeatedly: a correction lands on one name and the
 * other keeps the old behaviour, and nothing says so. */
#define IPSEL_GMAC0		BIT(1)

/* Power-cycle GMAC0's clock domain.
 *
 * ⚠ RESTORE ONLY THE GMAC BIT.  A trial that also OR'd the stock NIC bring-up
 * mask 0x1805 here -- taken from a chip-id-0x6266-conditional stock path --
 * coincided with a HARD SoC HANG on the RTL9602C: the sibling IP_SEL bits gate
 * other clock domains and are not ours to touch.
 *
 * ★ THAT WARNING EXISTED IN ONLY ONE OF THE TWO COPIES, which is the expensive
 * half of duplication -- the code survives being copied, the reason it is
 * shaped that way does not, and the next author of the poorer copy has nothing
 * to stop them re-running the experiment that hung the board. */
static inline void luna_ipsel_cycle_gmac0(void)
{
	writel(readl(SOC_IP_SEL) & ~IPSEL_GMAC0, SOC_IP_SEL);
	msleep(12);
	writel(readl(SOC_IP_SEL) | IPSEL_GMAC0, SOC_IP_SEL);
	msleep(2);
}


/* ---- SoC peripheral clock/enable word -------------------------------------
 * ⚠ THIS IS **NOT** `SOC_IP_SEL`.  They are 0x3c apart in the same SoC window,
 * and one of the three places that used this address called its pointer
 * `ipsel` -- which is the other register's name.  Two SoC control words a
 * stone's throw apart, one of them wearing the other's name, is precisely the
 * confusion this project renames on sight.
 *
 * ★ IT HAD THREE SPELLINGS: a #define here in one driver, a bare literal in the
 * second, and a locally-named pointer in the third.  One home now.
 *
 * ⚠⚠ AND THE TWO DRIVERS DISAGREE ABOUT WHAT BIT 5 IS, which is recorded
 * rather than resolved: luna_eth.c calls it "switch-core enable" and
 * gpon-rtl960x.c calls the SAME bit at the SAME address "PONPBO IP enable".
 * Both set it.  Either one name is wrong, or the bit gates a block both need
 * and neither name says so.  OWED: settle it from the chip's own SDK
 * (tier 3) or a live read, and rename on the spot -- do NOT pick one by
 * majority, which is how two wrong names cancel and survive. */
#define SOC_SW_ENABLE	((void __iomem *)0xb800063cul)
#define   SW_EN_BIT	BIT(5)		/* see the disagreement above	*/
#define   SW_PBO_BIT	BIT(25)		/* required on rev > A		*/

#endif /* _LUNA_ETH_REGS_H */
