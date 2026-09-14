// SPDX-License-Identifier: GPL-2.0-only
/*
 * cortina-l3fe.c - RTL9607F / Cortina CA8277C "Elnath" L3FE main-hash
 * flow-engine bring-up (the one-time init chain that must complete before
 * any flow can be added for a lookup to HIT), plus the mask-table and
 * HS_SWO HW-CRC primitives used by the offload backend and its selftest.
 *
 * Companion of cortina-ni-flowoffload.c (the nf_flow_table flow_block
 * backend).  Called once from the cortina-ni probe via
 * cortina_ni_flowoffload_probe().
 *
 * Facts + citations: dev/x400axf/HW_FLOW_OFFLOAD_L3FE_INIT.md and
 * HW_FLOW_OFFLOAD_DESIGN.md (clean-room RE of the stock ca-ne.ko register
 * sequences).  Every literal below was LIVE-VERIFIED against stock's armed
 * engine (devmem of 0xf43037xx-0x3cxx, 2026-07-18), which corrected two RE
 * assumptions: HS_HASH_INI = 0x0003007D means hb_size=1 (8-way buckets, NOT
 * 32) and def_reg=1 (miss actions come from the internal HS_DEFAULT_ACTION
 * registers, not a DDR table); and only BA_MH0/BA_MA0 are armed, so the DDR
 * carve is key(256K) + FIB(2M) only.
 *
 * NOTE phase 1 arms the engine only.  The ingress classify plumbing that makes
 * traffic actually consult the hash is phase-2/3 and deliberately NOT touched
 * here -- the working RX/TX datapath depends on the live classifier state.
 */

#include <linux/kernel.h>
#include "cortina_l3fe_logic.h"	/* hoisted logic */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>

#include "cortina-l3fe.h"
#include "cortina_ni_rx_logic.h"	/* the shared FDB key pack + action word */
#include "cortina-l3fe-regs.h"	/* the L3FE registers more than one file needs */

/* ------------------------------------------------------------------ *
 *  L3FE HS register map, NE-relative (NI window phys 0xf4300000).      *
 * ------------------------------------------------------------------ */
#define L3FE_HS_BA_MH1			0x3838	/* key table base, phys[39:32] */
#define L3FE_HS_BA_MA1			0x3840	/* action FIB base, phys[39:32] */
#define L3FE_HS_DEFAULT_ACTION(i)	(0x3860 + (i) * 4) /* internal default/miss actions (def_reg=1) */
#define L3FE_HS_CACHE_MISC		0x38c4	/* cache replacement policy */
#define L3FE_HS_MASK_DATA(n)		(0x3920 - (n) * 4) /* MASK0..3 = 0x3920,191c,1918,1914 */
#define L3FE_HS_CHK_FAIL_CTRL		0x3940	/* double-check-fail -> punt */
#define L3FE_HS_RSV0			0x3944	/* HW patch: bit31 crc_offload + bit0 */
#define L3FE_HS_RSV1			0x3948	/* HW patch: bit0 */
#define L3FE_AQM_TIMER			0x3aa8	/* AQM flow-stat timer cfg */
#define L3FE_AXIM2_CONFIG		0x3c80	/* AXI outstanding-transaction depth */

/* Internal hash-miss action FIB (HASH_INI.def_reg=1 mode): 6 entries x 3
 * regs = 96 bits each, holding the packed default (miss) action.  The
 * per-profile HS_PROFILEn_INI default_sel picks the entry; a routed miss
 * uses it to punt to CPU_0 (never drop). */
#define L3FE_HS_DEF_REG0_ETY0		0x39dc	/* first of the 6x3-reg internal FIB */
#define L3FE_HS_DEF_REG_COUNT		12	/* entries 0..3 captured live from stock */

/* L3-CLS classifier FIB (indirect): 7 words, DATA0 at 0x33cc down to
 * DATA6 at 0x33b4; ACCESS = GO|WR|idx.  The per-profile routing DEFAULT
 * actions live at idx (max_entry-16)|(profile<<2): 1024/1025 = profile 0
 * (WAN ingress), 1028 = profile 1 (LAN ingress). */
#define L3FE_CLS_FIB_ACCESS		0x33b0
#define L3FE_CLS_FIB_DATA0		0x33cc	/* word0; word i at DATA0 - i*4 */
#define L3FE_CLS_FIB_WORDS		7

/* Main-hash per-profile TUPLE0 maskptr (maskptr[5:0], pri[10:8], type[12]);
 * profile stride 0x2c.  Re-pointed at the 5-tuple mask under hw_l3_fwd so a
 * routed flow's install/lookup CRC uses the 5-tuple-only mask. */
#define L3FE_HS_PROFILE_TUPLE0(p)	(0x3704 + (p) * 0x2c)
#define L3FE_MAIN_HASH_PROFILE_WAN	0
#define L3FE_MAIN_HASH_PROFILE_LAN	1
/* ★ The hash profile the LIVE admission actually runs (P4, 2026-07-19): the
 * transit LAN unicast matches the LAN cls-trap catch-all rows (FIB 256/257/
 * 260/264), and the profile stamped there is 3 - on stock, profile 3 is the
 * flow profile the catch-all class feeds (the RTK asicDriver maps its
 * catch-all flow key-type to HASH_PROFILE_3; tier-1: stock's live routed FIB
 * rows carry t2_ctrl=3).  Ours re-purposes profile 3 with the 5-tuple mask 8
 * (gated), so install + lookup agree end to end. */
#define L3FE_MAIN_HASH_PROFILE_ROUTED	3
/* the engine has 7 hash profiles (0..6), stride 0x2c from HS_PROFILE0_INI */
#define L3FE_MAIN_HASH_PROFILE_MAX	6u
#define L3FE_5TUPLE_MASK_ID		8

/*
 * L3FE PE config (direct MMIO): the GEM-map mode for PON US hit-forwarding.
 * With gemid_map=1, a hit-action carrying the gemMapMode-1 encoding (mc=1,
 * mcgid=gem_id, group-20 {pop_l3_vld=1, t2_ctrl1=tcont}) egresses as
 * hdr_a.ldpid = ldpid_base + {ldpid_offset_msb, t2_ctrl1} - i.e. the PON US
 * logical port 0x20 + tcont, exactly the ldpid the proven CPU US data path
 * injects with (vendor aal_l3pe_pe_gemid_map_set(1) +
 * aal_l3pe_pe_ldpid_base_set(PON_US_0=0x20), rtk_rg_asic_l3qm_init).
 * Register fields per the rtl8277c map: ldpid_base[9:4], gemid_map[10].
 */
#define L3FE_PE_CFG			0x351c
#define L3FE_PE_CFG_LDPID_BASE		GENMASK(9, 4)
#define L3FE_PE_CFG_GEMID_MAP		BIT(10)
#define L3FE_PE_CFG_PAD_CTRL		BIT(12)		/* vendor-init: set by stock */
#define L3FE_PE_CFG_RSVD14		BIT(14)		/* vendor-init: set by stock */
#define L3FE_PE_CFG_MTU_CHK_EN		BIT(31)		/* ★ reset=1; stock CLEARS it.  Left set,
							 * every PE-transiting T2-miss CPU punt is MTU-
							 * length-checked against the (mostly-zero)
							 * L3FE_PE_MTU_SIZE table -> large frames (ssh KEX,
							 * bulk shell) diverted to CPU_REASON_MTU and lost;
							 * small frames (SYN/banner/ICMP) pass = the
							 * terminating-TCP "mangle".  Clear to match stock. */
#define L3FE_LDPID_PON_US_0		0x20	/* AAL_LPORT_PON_US_0 */

/*
 * L3FE PE PPPoE encap globals (direct MMIO, one-time; tier-2 disasm-confirmed
 * on the shipped ca-ne.ko: aal_l3pe_pppoe_cfg_set writes base+0x3500,
 * aal_l3pe_pppoe_ipv4/ipv6_prot_set write base+0x3504).  Consumed by the PE
 * only when a hit-action carries the PPPoE ADD encap (pppoe_set=1 +
 * pppoe_vld=1): CFG supplies the header's code/version/type, PROT_CFG the
 * PPP protocol number per inner IP version.  The 0x8864 ethertype is fixed/
 * implicit and the PPPoE payload length is HW-computed per packet.
 * Facts: scratchpad RE_pppoe_aal77c.md section 3d / PPPoE_DRIVER_SPEC.md 2a.
 */
#define L3FE_PE_PPPOE_CFG		0x3500	/* [15:8]=code 0 (session), [7:4]=ver 1, [3:0]=type 1 */
#define L3FE_PE_PPPOE_CFG_VAL		0x00000011u
#define L3FE_PE_PPPOE_PROT_CFG		0x3504	/* [15:0]=v4 PPP-proto 0x0021, [31:16]=v6 0x0057 */
#define L3FE_PE_PPPOE_PROT_CFG_VAL	0x00570021u

/*
 * Egress L3-IF table (fe_l3e_if_tbl): 32 x 32-bit entries, GLB indirect
 * access (same GO|WRITE protocol as the mask table) - tier-2 binary-confirmed
 * descriptor {access=0xf4303000, 1 data reg, 32 entries}.  Entry, LSB-first:
 * pppoe_set:1(b0) pppoe_vld:1(b1) pppoe_session_id:16(b2-17) mac_sa_vld:1(b18)
 * mac_sa_an_sel:4(b19-22) pppoe_len_control:1(b23).  A hit-action selects an
 * entry via GROUP_20 l3_if_vld1 + egr_l3_if_idx1.
 *
 * Two independent tiers agree on those field positions: the shipping-die
 * register description (L3FE_GLB_EGRESS_L3_IF_TBL_DATA at 0xf4303004:
 * PPPOE_SET/PPPOE_VLD/PPPOE_SESSION_ID/MAC_SA_VLD/MAC_SA_AN_SEL/PAD_CTRL) and
 * the reference HAL's 24-bit table-entry description for the same table (which
 * names b23 pppoe_len_control instead of PAD_CTRL - same bit, two names).  The
 * entry has NO other fields on this die: there is exactly ONE egress L3-IF word
 * per WAN interface, so the SMAC substitution and the PPPoE header ADD must live
 * in the SAME entry - see l3fe_l3if_entry().
 */
#define L3FE_L3IF_ACCESS		0x3000
#define L3FE_L3IF_DATA			0x3004
#define L3FE_L3IF_ENTRIES		32
#define L3FE_L3IF_PPPOE_SET		BIT(0)
#define L3FE_L3IF_PPPOE_VLD		BIT(1)
#define L3FE_L3IF_PPPOE_SESSION		GENMASK(17, 2)
#define L3FE_L3IF_MAC_SA_VLD		BIT(18)
#define L3FE_L3IF_MAC_SA_AN_SEL		GENMASK(22, 19)
#define L3FE_L3IF_PAD_CTRL		BIT(23)	/* = pppoe_len_control; stock sets it */

/* HW ager cadence for the gated experiment: non-zero so the on-chip ager
 * runs and the lookup's HIT age-re-arm is observable (the SECONDARY hit
 * witness - with granularity 0 the ager block is off and the age slot reads
 * stale).  0x08000000 is the stock-magnitude slow value (~20 min class per
 * decay step), so an idle 2-bit IDLE(1) entry outlives any test window and
 * nf-managed lifetimes never race it. */
#define L3FE_AGING_GRAN_SLOW		0x08000000u

#define L3FE_GO				BIT(31)
#define L3FE_WRITE			BIT(30)
#define L3FE_MASK_UPPER128		BIT(6)
/* L2FE FDB engine (direct regs, NE window; same protocol as the fdb path in
 * cortina-ni-rx.c).  Used here for the terminating DS-WAN delivery entry:
 * the Venus-family design keeps L2 MY-MAC detection off and routes MyMAC
 * frames to the L3FE via a STATIC FDB entry — stock's FDB holds its WAN MAC
 * -> L3_WAN (0x18).  Without it a PON DS unicast to the WAN MAC is a DLF in
 * the L2FE and gets flooded out instead of delivered. */
#define L3FE_FDB_CMD_RETURN		0x1c2c
#define L3FE_FDB_ACCESS			0x1ca0
#define L3FE_FDB_OP_APPEND		0x45
#define L3FE_FDB_DATA3			0x1ca4
#define L3FE_FDB_DATA2			0x1ca8
#define L3FE_FDB_DATA1			0x1cac
#define L3FE_FDB_DATA0			0x1cb0
/* The five FDB FIELD bits were removed 2026-09-03: a second, unused copy of a
 * hardware format whose only live spelling is flowcore/cortina_ni_rx_logic.c.
 * The ADDRESS defines above stay -- this file owns its register base and does
 * its own writes; only the value computation is common. */

/* ------------------------------------------------------------------ *
 *  Transit-frame INGRESS ADMISSION registers (Divergence C).           *
 *                                                                      *
 *  How a routed transit frame physically ENTERS the L3FE on stock      *
 *  (RE of ca-ne.ko ca_l3_intf_create/aal_l3fe_stg0_set_normal +        *
 *  cortina-api route.c/port.c, cross-checked tier-1 live):             *
 *   - LAN side: a static L2FE FDB entry {router MAC} forwards the      *
 *     frame to LDPID 0x19 (L3_LAN pseudo-port); the ARB LDPID->PDPID   *
 *     map resolves 0x19 -> physical port 0x0d = the L3FE LAN ingress.  *
 *   - WAN side: the PON PDC stamps DS data-GEM frames with LDPID 0x18  *
 *     (L3_WAN); the map resolves 0x18 -> physical port 0x0a = the      *
 *     L3FE WAN ingress (stock live: PDPID_MAP[0x18]=0xA [0x19]=0xD,    *
 *     dev/x400axf/stock_golden_qm.txt).                                *
 *   - Inside the L3FE, STG0_LDPID_MAP (0x3404 = 0x03985907) selects    *
 *     LPB profile by HDR_A.ldpid: 0x07->prof0, 0x19->prof1,            *
 *     0x18->prof2; the LPB profile rewrites HDR_I.lspid to L3_WAN      *
 *     (0x18) / L3_LAN (0x19) and picks the T1 classifier profile       *
 *     (WAN=0 / LAN=1) - all already stock-programmed by cortina-ni.    *
 * ------------------------------------------------------------------ */

/* L2FE ARB LDPID->PDPID map (indirect, generic GO protocol): index =
 * {my_mac bit7, dbuf bit6, ldpid[5:0]}, DATA = pdpid[3:0]. */
#define L3FE_L2FE_PDPID_MAP_ACCESS	0x166c
#define L3FE_L2FE_PDPID_MAP_DATA	0x1670
#define L3FE_PDPID_IDX_DBUF		BIT(6)
#define L3FE_PDPID_IDX_MYMAC		BIT(7)
#define L3FE_LDPID_L3_WAN		0x18	/* AAL_LPORT_L3_WAN */
#define L3FE_PDPID_L3_WAN		0x0a	/* AAL_PPORT_L3_WAN (stock live 0xA) */

/*
 * L3FE PP FIELD-CAM - the my-MAC / MAC-DA recognition CAM (15 entries).
 * A frame whose DA matches entry i carries mac_da_an_sel = i+1 in HDR_I
 * (the "routing MAC" recognition the T1 classifier and the STG0 lspid
 * rewrite key on).  Protocol (stock cam_hw_entry_set, disasm-verified):
 * data words to DATA0..DATA4 (0x3214 down to 0x3204), then ACCESS =
 * GO | WRITE | (cam_table_sel << 16) | entry_idx, poll GO clear.
 * MAC-DA entry layout: word0 = mac[2..5] (mac[2] in bits31:24), word1 =
 * vld(bit16) | mac[0]<<8 | mac[1], words 2..4 = 0.
 * NOTE our earlier code wrote only 0x3210/0x3214 - that is the DATA
 * STAGING window; without the 0x3200 ACCESS commit the CAM itself is
 * never written (stock live 0x3214 held an 0x86dd ETHERTYPE residue,
 * proving these are shared staging latches, not entry-0 registers).
 */
#define L3FE_PP_FIELD_CAM_ACCESS	0x3200
#define L3FE_PP_FIELD_CAM_DATA(n)	(0x3214 - (n) * 4) /* DATA0..DATA4 */
#define L3FE_CAM_SEL_MAC_DA		3	/* MAC-DA table select (dport=0) */
#define L3FE_CAM_MAC_DA_ENTRIES		15
#define L3FE_CAM_MAC_DA_VLD		BIT(16)	/* in data word1 */

/* The two router-MAC CAM entries this port provisions: entry 0 = the LAN
 * gateway MAC, entry 1 = the WAN MAC (= base + 1).  The PP stamps
 * HDR_I.mac_da_an_sel = entry + 1 on a DA hit (0 = not a router MAC). */
#define L3FE_AN_IDX_LAN			0
#define L3FE_AN_IDX_WAN			1
#define L3FE_AN_SEL(idx)		((idx) + 1)

/*
 * STG0 per-profile LPB HIGH word (direct MMIO, LOW/MID/HIGH stride 0xC from
 * 0x3408; HIGH0 = 0x3410).  mac_da_an_mask = HIGH[17:10]: bit (mac_idx+1)
 * enables the MAC-DA CAM compare-and-stamp for that ingress profile.
 * mac_da_match_en = HIGH[18] stays 0 (promiscuous: stamp, never filter) -
 * tier-1 stock prof3 HIGH=0x1a1bfd90 decodes an_mask=0xff, match_en=0.
 * Profile use (tier-1 STG0_LDPID_MAP 0x3404=0x03985907): ldpid 0x07->prof0,
 * 0x19 (L3_LAN)->prof1, 0x18 (L3_WAN)->prof2; prof3 = OAM.  The an-mask bits
 * go into prof0/1/2 (both router MACs on every routed ingress class - the
 * mask only widens the promiscuous compare, exactly stock prof3's 0xff).
 */
#define L3FE_STG0_LPB_HIGH(p)		(0x3410 + (p) * 0xC)
#define L3FE_LPB_AN_MASK(sel)		BIT(10 + (sel))
#define L3FE_LPB_AN_PROFILES		3	/* prof 0..2 get the an-mask */

/*
 * L3-CLS classifier KEY table (indirect, same GO protocol as the FIB): 11
 * words, word i at ACCESS + (11 - i)*4 (word0 @ 0x33ac .. word10 @ 0x3384).
 * Partitioned: classifier profile 0 (WAN ingress) = KEY[0..63], profile 1
 * (LAN ingress) = KEY[64..127]; rows 0/1/2 + 64/65/66 hold the stock spcl
 * CPU-trap rows (cortina-ni-rx.c cls_key_golden).  FIB idx = (key_row << 2)
 * | sub_slot.
 */
#define L3FE_CLS_KEY_ACCESS		0x3380
#define L3FE_CLS_KEY_WORDS		11

/* The dedicated pri-6 ROUTED rules live in the first free row of each
 * partition; sub-slot 0. */
#define L3FE_CLS_KEY_ROW_WAN		3
#define L3FE_CLS_KEY_ROW_LAN		67
/* The pri-8 WAN non-IP control trap (PPPoE LCP/IPCP/PAP/CHAP etc) - next
 * free row of the WAN partition. */
#define L3FE_CLS_KEY_ROW_WAN_CTL	4
#define L3FE_CLS_FIB_IDX(key_row)	((key_row) << 2)

/*
 * Stock-armed register values, live-captured 2026-07-18 (tier-1) and
 * mirrored verbatim.  HASH_INI decodes as hb_size=1 (8-way bucket),
 * ht_size=7 (64K entries), ha_width=3 (256-bit FIB), def_reg=1,
 * crc_ntfy_en=1.
 */
#define L3FE_HASH_INI_VAL		0x0003007Du
#define L3FE_DEFAULT_ACTION_VAL		0x0E4D0000u	/* words 0..3; miss -> punt */
#define L3FE_CACHE_INI_VAL		0x00050304u
#define L3FE_CACHE_MISC_VAL		0xA0000000u
#define L3FE_CHK_FAIL_CTRL_VAL		0x0000D0D0u
#define L3FE_AXIM2_CONFIG_VAL		0x000002FFu	/* reset value is 0x200 */
#define L3FE_AQM_TIMER_VAL		0x5000A2D0u	/* reset value is 0x9000A2D0 */
#define L3FE_RSV0_PATCH			(BIT(31) | BIT(0))
#define L3FE_RSV1_PATCH			BIT(0)

/* Poll a self-clearing bit, bounded; 0 on clear, -ETIMEDOUT on cap. */

int cortina_l3fe_engine_init(void __iomem *ne, const struct cn_l3e_tables *t)
{
	int ret, i;

	/*
	 * 1. Engine SRAM/table self-init - MUST precede the base/size arm.
	 *    Kick req_sts (bit0), poll its self-clear.
	 */
	writel(1, ne + L3FE_HS_MEM_INI);
	ret = l3fe_access_wait(ne, L3FE_HS_MEM_INI, BIT(0));
	if (ret)
		return ret;

	/* 2. SW-zero the DDR tables (belt and braces, matches vendor). */
	memset(t->key_virt, 0, CN_L3E_KEY_TBL_BYTES);
	memset(t->fib_virt, 0, CN_L3E_FIB_TBL_BYTES);
	wmb();	/* coherent carve: make the zeroing visible before the arm */

	/* 3. DDR base registers (physical, 128-byte aligned, [31:7] in
	 * place; hi regs hold phys[39:32]).  Overflow/default/cache bases
	 * stay 0 as on stock. */
	writel(lower_32_bits(t->key_pa), ne + L3FE_HS_BA_MH0);
	writel(upper_32_bits(t->key_pa) & 0xff, ne + L3FE_HS_BA_MH1);
	writel(lower_32_bits(t->fib_pa), ne + L3FE_HS_BA_MA0);
	writel(upper_32_bits(t->fib_pa) & 0xff, ne + L3FE_HS_BA_MA1);

	/* 4. Geometry + cache config (stock-verbatim). */
	writel(L3FE_HASH_INI_VAL, ne + L3FE_HS_HASH_INI);
	writel(L3FE_CACHE_INI_VAL, ne + L3FE_HS_CACHE_INI);
	writel(L3FE_CACHE_MISC_VAL, ne + L3FE_HS_CACHE_MISC);

	/* 5. Anti-wedge HW patch (do NOT skip: the engine can stall under
	 * DDR read load without it) + AXI outstanding depth. */
	writel(readl(ne + L3FE_HS_RSV0) | L3FE_RSV0_PATCH, ne + L3FE_HS_RSV0);
	writel(readl(ne + L3FE_HS_RSV1) | L3FE_RSV1_PATCH, ne + L3FE_HS_RSV1);
	writel(L3FE_AXIM2_CONFIG_VAL, ne + L3FE_AXIM2_CONFIG);

	/* 6. Miss/fail never drops: double-check-fail punt + the internal
	 * default (miss) actions, def_reg=1 mode - stock programs words
	 * 0..3, the rest stay 0. */
	writel(L3FE_CHK_FAIL_CTRL_VAL, ne + L3FE_HS_CHK_FAIL_CTRL);
	for (i = 0; i < 4; i++)
		writel(L3FE_DEFAULT_ACTION_VAL, ne + L3FE_HS_DEFAULT_ACTION(i));

	/* 7. HW auto-age-countdown OFF (stock): hardware must never age a
	 * flow out from under the Linux flowtable.  Liveness = HW hit-rearm
	 * + the SW sweep; lifetime = nf gc + FLOW_CLS_DESTROY. */
	writel(0, ne + L3FE_HS_AGING_GRANULARITY);

	/* 8. AQM flow-stat timer, stock-verbatim. */
	writel(L3FE_AQM_TIMER_VAL, ne + L3FE_AQM_TIMER);

	return 0;
}

/*
 * Profile/tuple + mask-table classify config, tier-1 captured live from the
 * stock-armed engine (RTK_GW 5.10.226, 2026-07-18; the exact register dump is
 * in dev/x400axf/stock_l3fe_dump_full.txt).  Programming this makes the
 * main-hash engine parse/key a routed packet exactly like stock: profiles 0-5
 * (INI/tuple/type-action) select mask-table entries 0-7 (the in-use masks;
 * 8-63 are uninitialised SRAM on stock too).  PF_KEY/TPL stay 0 -
 * hash_key_select is .bss (=0) on stock, so there is no per-profile CRC
 * rotate.  STG0/LDPID (0x3400/0x3404) already match stock via cortina-ni.c.
 *
 * ★ P2 FINDING (2026-07-18) - this is NECESSARY but NOT SUFFICIENT for a HW
 * hit.  On our datapath LAN->WAN packets are SOFTWARE-forwarded, so the main
 * hash is never consulted: 1.9M matching packets with SWO-correct entries in
 * ALL 8 profile/mask buckets gave ZERO age re-arm.  Steering routed packets
 * into the HW lookup is a datapath piece beyond this table config; until then
 * the install stays gated OFF.  Programming the config is runtime-verified NOT
 * to regress the datapath (WAN 0% loss).
 */
/*
 * Masks 0-7 = the stock classify masks (tier-1 captured).  ★ Mask 8 = a
 * dedicated 5-TUPLE-ONLY NAPT mask (P3, 2026-07-19): stock mask 0 also folds
 * mac_sa/mac_da/lspid/dscp/ecn/VLAN/PPPoE -- non-zero on a real routed frame,
 * zero in the driver's synthetic key -- so an install CRC from a sparse key
 * can never equal a parsed packet's lookup CRC under mask 0.  Live on-board
 * probes confirm which HDR_I offsets feed the hash: {233-264 DA, 361-392 SA,
 * 492-499 proto} MOVE the CRC, {116 dscp, 600/700/726 mac/lspid} do NOT.
 * Re-pointed at by cortina_l3fe_hw_l3_forward_enable() (gated), so programming
 * this spare index changes no datapath behaviour.
 */
#define L3FE_MASK_5TUPLE	8	/* 5-tuple-only NAPT mask index */

/*
 * ★ Mask-entry field geometry, L4 region (needed by the exact-port invariant
 * below).  A mask entry is 224 meaningful bits, one FIELD (not one bit) per
 * hashable key field, LSB-first over the entry:
 *
 *   [0]     l4_chksum_zero      [9:1]   tcp/rdp control flags
 *   [26:10] l4_dp  - 17 bits    [43:27] l4_sp  - 17 bits
 *   [44]    l3_chksum_err       ... [56:55] ip_ttl (2-bit enum)
 *   [66:58] ip_da keep-length   [75:67] ip_sa keep-length
 *   [77]    ip_protocol         [86] ip_ver   [87] ip_vld
 *
 * ★★ Each L4-port field is 17 bits for a 16-bit port, and the extra top bit
 * (field bit 16 = entry bit 26 for dp, 43 for sp) is a MODE SELECT, not a
 * mask bit:
 *   field == 0                  -> EXACT match: the 16-bit port value itself
 *                                  enters the hash tuple (nothing masked).
 *   field == (1<<16) | ~rng_vec -> RANGE match: the tuple takes the parser's
 *                                  port-RANGE-match vector instead of the port
 *                                  value, and the low 16 bits mask which range
 *                                  slots matter.
 * Evidence: tier-3 the vendor classifier's two branches build exactly those
 * two shapes ("one l4 port" -> mask field 0; "l4 port range" -> a port-range
 * CAM entry plus mask field (1<<16)|~(1<<slot)); tier-1 the eight stock masks
 * captured live agree - the two that keep the ports (0 and 7) have BOTH port
 * fields all-zero, and every mask that drops the ports has them all-ones.
 */
#define L3FE_MASK_L4_DP_LSB	10	/* l4_dp field  = entry bits [26:10] */
#define L3FE_MASK_L4_SP_LSB	27	/* l4_sp field  = entry bits [43:27] */
#define L3FE_MASK_L4_PORT_BITS	17	/* 16 value bits + bit16 = range mode */

/*
 * Mask 8 words, named so the exact-port invariant below is a BUILD-time check.
 * Word i covers entry bits [32i+31 : 32i].
 */
#define L3FE_MASK5_W0		0x000003ffu
#define L3FE_MASK5_W1		0x827ff000u
#define L3FE_MASK5_W2		0xff3fd100u
#define L3FE_MASK5_W3		0xffffffffu

/*
 * ★ INVARIANT D (build-time): the 5-tuple mask MUST select EXACT L4-port
 * match, i.e. both 17-bit port fields all-zero.  Setting only the range-mode
 * bit - the 2026-07-19..24 value 0x040003ff/0x827ff800 - is a legal encoding
 * that silently swaps the port VALUE out of the hash tuple for the parser's
 * range-match vector: the boot key-packing test then reports "dport/sport did
 * NOT move the CRC", two NAPT flows differing only in their ports collide on
 * one entry (and the post-hit double-check, which re-derives the hash under
 * this same mask, cannot tell them apart), and matching a real frame starts to
 * depend on the port-range CAM SRAM, which this driver never programs.  Guard
 * it here so an edit fails the build, not a benchmark.
 */
static_assert((L3FE_MASK5_W0 & GENMASK(31, L3FE_MASK_L4_DP_LSB)) == 0,
	      "5-tuple mask word0: l4_dp/l4_sp fields must be 0 = exact-port match");
static_assert((L3FE_MASK5_W1 &
	       GENMASK(L3FE_MASK_L4_SP_LSB + L3FE_MASK_L4_PORT_BITS - 1 - 32, 0)) == 0,
	      "5-tuple mask word1: l4_sp field top bits must be 0 = exact-port match");
static const u32 l3fe_mask_lo[9][4] = {
	{ 0x000003ff, 0x0221f000, 0x15001402, 0xc0f03fe1 },
	{ 0xffffffff, 0x027fffff, 0x1f403000, 0xc0f01fe1 },
	{ 0xffffffff, 0x027fffff, 0xff3ff002, 0xffffffff },
	{ 0xffffffff, 0x027fffff, 0x1f003402, 0xffffffe1 },
	{ 0xffffffff, 0x027fffff, 0x1f003402, 0xffffffe1 },
	{ 0xffffffff, 0x027fffff, 0xfffff000, 0xc0ffffff },
	{ 0xffffffff, 0x027fffff, 0xff7ff000, 0xffffffff },
	{ 0x000003ff, 0x0221f000, 0x15001402, 0xc0f03fe1 },
	/*
	 * ★ mask 8: 5-TUPLE-ONLY NAPT mask, from the aal-77c aal_hash_mask_t
	 * field table (this chip's own tree, NOT aal-gen2), derived TWO
	 * independent ways that agree bit-for-bit on all meaningful bits [219:0].
	 * Polarity: 1 = EXCLUDE, 0 = KEEP; ip_da/ip_sa are a 9-bit KEEP-LENGTH
	 * (0x020 = keep the top 32 = the IPv4 address); ip_ttl is a 2-bit enum,
	 * 0/1 exclude, 2/3 keep.  KEEP = {l4_dp, l4_sp (all 17 bits = EXACT port,
	 * INVARIANT D above), ip_protocol, ip_ver, ip_vld, ip_da/32, ip_sa/32};
	 * everything else excluded, so a real routed frame hashes identically to
	 * the driver's sparse 5-tuple install.
	 *
	 * ⚠ TWO EARLIER VALUES WERE WRONG, and the reason each was wrong is the
	 * reason the bits above are spelled out:
	 *   - 0x040003ff/0x827ff800 set field bit 16 of both port fields, read as
	 *     "exclude the port's exact/range flag".  It is the RANGE-MODE SELECT
	 *     (INVARIANT D), so the tuple carried the parser's range-match vector
	 *     INSTEAD of the port value: ports stopped moving the CRC, flows
	 *     differing only in their ports aliased onto one entry, and matching
	 *     came to depend on port-range CAM SRAM this driver never programs.
	 *   - the hand-built "mask0|~mask1" kept ip_ttl (enum 3, and it armed the
	 *     TTL>10 range-check), used a malformed keep-length 511, and kept
	 *     ipv6_doh/rh/hbh + ip_fragment_flag + ip_options -- all parser-set
	 *     and zero in the sparse build.  Board-measured 2026-07-23: install
	 *     CRC c9b5981e/5fc9 vs the HW crc_ntfy tap 6667e6d3/5a64.
	 *
	 * Only reachable under hw_l3_fwd; gate-off leaves the stock masks alone.
	 * The DOUBLE-CHECK is separate and already correct (action chk_msk_ptr1 =
	 * mask 8 + cache_ctrl1 = 1; the flow FIB never fetches GROUP_21, so no
	 * xor32 is needed).
	 */
	{ L3FE_MASK5_W0, L3FE_MASK5_W1, L3FE_MASK5_W2, L3FE_MASK5_W3 },
};
static const u32 l3fe_mask_hi[9][4] = {
	{ 0xffff807f, 0xffffffff, 0xfeffffff, 0xffffffff },
	{ 0xffff807f, 0xffffffff, 0xfeffffff, 0xffffffff },
	{ 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff },
	{ 0xffff807f, 0xffffffff, 0xfeffffff, 0xffffffff },
	{ 0x0000007f, 0x00000000, 0xfeffff80, 0xffffffff },
	{ 0xffff807f, 0xffffffff, 0xffffffff, 0xffffffff },
	{ 0xffffffff, 0xffffffff, 0xfefffeff, 0xffffffff },
	{ 0x007f807f, 0xff800000, 0xfeffffff, 0xffffffff },
	/* mask 8: 5-tuple only - exclude all L2/lspid/dscp/vlan/pppoe */
	{ 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff },
};
/* The HASH-PROFILE block -- RE'd 2026-08-29 from the stock ca-ne.ko (tier 2):
 * aal_hash_profile_init / _tuple_add / _defAct_update all compute
 * 0x3700 + profile*0x2c and `cmp w0, #6` bounds SEVEN profiles (0..6).
 * Per profile: +0x00 default-action word (defAct_update puts a into [8:4] AND
 * [18:14], b into [13:9] AND [23:19]; all six live values satisfy that
 * duplication, so disasm and dump confirm each other), +0x04/+0x08 the tuple
 * list (second entry carries id | BIT(8)), +0x24/+0x28 init words A/B.
 * ⚠ NOT DECODED, and named so nobody reads them as understood: default-action
 *   bits [1:0] (set on live stock, written by neither init nor defAct_update),
 *   the BIT(8) tuple flag, and what the 25-bit init values mean.
 * ⚠ CORRECTED THE SAME DAY IT WAS WRITTEN: the first version put the base at
 *   0x3724 and called 0x3700 an "engine enable" -- off by one word, derived
 *   from the offsets alone.  A structure guessed from data can be
 *   self-consistent and wrong; the stock CODE is what settled it.
 * ★ NOT request_firmware() DATA: 25 words.  That rule is for blobs another
 *   PROCESSOR executes; a 200-byte register seed would trade a readable table
 *   for a loader, an image file, and a failure mode where the L3FE cannot
 *   classify because the rootfs lost a file.
 */
static const u32 l3fe_profile_regs[][2] = {
	/* -- profile 0 (base 0x3700): defact [1:0]=1, no tuples -- */
	{ 0x3700, 0x00000001 }, { 0x3724, 0x06140000 }, { 0x3728, 0x06000000 },
	/* -- profile 1 (0x372c): defact 1, tuple 1 -- */
	{ 0x372c, 0x00000001 }, { 0x3730, 0x00000001 },
	{ 0x3750, 0x06140000 }, { 0x3754, 0x06000000 },
	/* -- profile 2 (0x3758): defact acts a=1,b=0 + [1:0]=2, tuples 2, 3|F -- */
	{ 0x3758, 0x00004012 }, { 0x375c, 0x00000002 }, { 0x3760, 0x00000103 },
	{ 0x377c, 0x06140000 }, { 0x3780, 0x06000000 },
	/* -- profile 3 (0x3784): acts a=1,b=1 + [1:0]=1, tuple 4 -- */
	{ 0x3784, 0x00084211 }, { 0x3788, 0x00000004 },
	{ 0x37a8, 0x06140000 }, { 0x37ac, 0x06000000 },
	/* -- profile 4 (0x37b0): acts a=1,b=1 + [1:0]=1, tuple 5 -- */
	{ 0x37b0, 0x00084211 }, { 0x37b4, 0x00000005 },
	{ 0x37d4, 0x06140000 }, { 0x37d8, 0x06000000 },
	/* -- profile 5 (0x37dc): defact [1:0]=2, tuples 6, 7|F -- */
	{ 0x37dc, 0x00000002 }, { 0x37e0, 0x00000006 }, { 0x37e4, 0x00000107 },
	{ 0x3800, 0x06140000 }, { 0x3804, 0x06000000 },
};

int cortina_l3fe_classify_setup(void __iomem *ne)
{
	int i, ret;

	/* masks 0-7 = stock; mask 8 = the spare 5-tuple NAPT mask (unused
	 * unless a profile's maskptr is re-pointed at it under hw_l3_fwd) */
	for (i = 0; i < 9; i++) {
		ret = cortina_l3fe_mask_write(ne, i, l3fe_mask_lo[i],
					      l3fe_mask_hi[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < (int)ARRAY_SIZE(l3fe_profile_regs); i++)
		writel(l3fe_profile_regs[i][1],
		       ne + l3fe_profile_regs[i][0]);
	return 0;
}

int cortina_l3fe_mask_write(void __iomem *ne, u32 idx,
			    const u32 lo[4], const u32 hi[4])
{
	int i, ret;

	/* lower 128 bits: data words then the GO|W commit */
	for (i = 0; i < 4; i++)
		writel(lo[i], ne + L3FE_HS_MASK_DATA(i));
	writel(L3FE_GO | L3FE_WRITE | (idx & 0x3f), ne + L3FE_HS_MASK_ACCESS);
	ret = l3fe_access_wait(ne, L3FE_HS_MASK_ACCESS, L3FE_GO);
	if (ret)
		return ret;

	/* upper 128 bits: second beat with bit6 */
	for (i = 0; i < 4; i++)
		writel(hi[i], ne + L3FE_HS_MASK_DATA(i));
	writel(L3FE_GO | L3FE_WRITE | L3FE_MASK_UPPER128 | (idx & 0x3f),
	       ne + L3FE_HS_MASK_ACCESS);
	return l3fe_access_wait(ne, L3FE_HS_MASK_ACCESS, L3FE_GO);
}

int cortina_l3fe_swo_crc(void __iomem *ne, const u32 *words, int nwords,
			 u32 mask_id, u32 *crc32_out, u16 *crc16_out)
{
	int i, ret;

	if (nwords < 1 || nwords > 32)
		return -EINVAL;

	/* key words at SWO index 0.. (DAT auto-increments IDX) */
	writel(0, ne + L3FE_HS_SWO_IDX);
	for (i = 0; i < nwords; i++)
		writel(words[i], ne + L3FE_HS_SWO_DAT);

	/* mask pointer at SWO index 32 */
	writel(32, ne + L3FE_HS_SWO_IDX);
	writel(mask_id, ne + L3FE_HS_SWO_DAT);

	/* run: bit0 = go/busy (dedicated, not the bit31 protocol) */
	ret = l3fe_access_wait(ne, L3FE_HS_SWO_CTRL, BIT(0));
	if (ret)
		return ret;
	writel(1, ne + L3FE_HS_SWO_CTRL);
	ret = l3fe_access_wait(ne, L3FE_HS_SWO_CTRL, BIT(0));
	if (ret)
		return ret;

	/* results: SWO index 33 = CRC32, 34 = CRC16 (read-only slots) */
	writel(33, ne + L3FE_HS_SWO_IDX);
	*crc32_out = readl(ne + L3FE_HS_SWO_DAT);
	writel(34, ne + L3FE_HS_SWO_IDX);
	*crc16_out = readl(ne + L3FE_HS_SWO_DAT) & 0xffff;
	return 0;
}

/* ------------------------------------------------------------------ *
 *  Divergence B: enable HW L3-forwarding (miss -> CPU).               *
 * ------------------------------------------------------------------ */

/*
 * Internal hash-miss action FIB, tier-1 captured from the stock-armed engine
 * (2026-07-18, running HW-NAT: /proc/fc/ctrl/hwnat=1).  With
 * HASH_INI.def_reg=1 the main-hash lookup fetches the MISS action from these
 * registers, NOT a DDR table; our engine_init left them 0, so a miss (which
 * happens for EVERY packet until a flow is installed) had a null action.
 * Entries 1..3 mirror the stock capture (other miss types an empty-bucket
 * zero-flow lookup never selects).
 *
 * ★ Entry 0 = TRAP-TO-CPU_0, our deliberate deviation from the stock capture.
 * The tier-1 stock entry 0 decodes to permit=0 (DROP) with keep_ts set —
 * stock traps its terminating DS-WAN traffic (DHCP/ICMP-to-router/ARP) to the
 * CPU via the L3FE spcl-packet handler + per-interface CLS rules keyed on the
 * router's own IP, which this port does not replicate (it uses the CLS
 * per-profile DEFAULTS only).  Without those, a terminating DS frame that
 * enters the L3FE (PDC ldpid L3_WAN, fe_bypass=0) falls to the hash miss and
 * stock's permit=0 entry 0 DROPS it — verified on the board: frames enter the
 * L3FE (l3fe_rx climbs) but the CPU-RX spy never fires and the DHCP OFFER
 * never reaches gpon0.  For our nf_flow_table model the miss MUST punt to the
 * CPU (the first packet of any flow, and all terminating traffic, is software-
 * handled), so entry 0 is a plain {permit, dpid_vld, dpid_pri, mcgid=CPU_0
 * (0x10), mc=0} trap = the SAME dpid action the CLS per-profile defaults carry
 * (l3fe_cls_default row 1024, decode-verified).  With install gated OFF this
 * makes every DS frame miss -> trap to CPU_0 -> gpon0 = the zero-flow no-
 * regression path.  This table is written only inside the hw_l3_fwd-gated
 * cortina_l3fe_hw_l3_forward_enable(), so gate-off behaviour is unchanged.
 */
static const u32 l3fe_def_reg_stock[L3FE_HS_DEF_REG_COUNT] = {
	/* entry 0: TRAP the hash miss to CPU_0 -- the SAME dpid action the CLS
	 * per-profile defaults carry.  ON-BOARD PROOF: a routed LAN my-MAC miss
	 * reaches the CPU (SSH stays up with admission on).  Re-pointing it at the
	 * deep-queue ldpid 0x32 BROKE LAN CPU-RX, so CPU_0 is the right delivery.
	 * ★ word1 bits 22/23 = keep_orig_pkt_vld|keep_orig_pkt: every vendor to-CPU
	 * miss action carries them so the punt is delivered VERBATIM; without them
	 * the PE rebuilds the frame and it dies at/after CPU RX -- which is why
	 * enabling the hash-consult BROKE the routed LAN->WAN path at zero flows,
	 * the same class as the board-proven PPPoE-LCP mangling. */
	0x00000000, 0x00300000, 0x00000000,	/* entry 0: stock hash-miss (NO dpid; the CLS row supplies the CPU port - tier-1 devmem 2026-07-23) */
	/* ★ entry 1 = TRAP -> CPU_0 too (deviation from the stock capture
	 * 0x00009811): profile 3's INI (0x3784 = 0x00084211) points all four
	 * default_sel nibbles at DEFAULT_ACTION entry 1, and the gated routed
	 * admission now stamps t2_ctrl=3 on the LAN catch-all - so a profile-3
	 * T2 MISS (= every ONU-terminating/unoffloaded LAN frame, incl. SSH)
	 * resolves through entry 1.  Stock's 0x9811 is its bridge-flow miss
	 * action; ours MUST punt to CPU_0 or gate-on would black-hole LAN
	 * management on the first miss. */
	0x00009811, 0x00000000, 0x00000000,	/* entry 1: stock bridge/flood miss action 0x9811 (tier-1 devmem 2026-07-23) */
	0x00040001, 0x00300000, 0x00404000,	/* entry 2 */
	0x00009831, 0x00300000, 0x00000000,	/* entry 3 */
};

/*
 * CLS per-profile routing DEFAULT actions - the hash-CONSULT enable.  A routed
 * frame that matches no specific CLS rule falls through to the profile default
 * at idx (max_entry-16)|(profile<<2)|((rslt_type&1)<<1) - ★ that indexing
 * scheme is REFUTED, see the correction note directly above the table.  Per the
 * ca-ne.ko RE
 * (convert_intf_to_cls route.c / cls_type_1_default_set classifier.c) the CLS
 * result's t2_ctrl field is what points HDR_I at the T2 main hash: 0 = WAN
 * hash profile, 1 = LAN hash profile, 0xF = BYPASS.  All rows keep the stock
 * CPU-fallback {permit, dpid_pri, mcgid=CPU_0} so a hash MISS still traps to
 * the CPU.  word0..word6 (word0 = DATA0 at 0x33cc), struct-decode verified.
 *
 * All three rows = EXACT stock bytes.  (An earlier build forced t2_ctrl_vld=1
 * on row 1028 - REFUTED as a lever: the default rows are dead behind the
 * all-wildcard trap row KEY[66]/KEY[2], and T2 admission for routed traffic
 * is the dedicated pri-6 mac_da_an_sel rule (cortina_l3fe_intf_add), exactly
 * stock's ca_l3_intf_add scheme.)
 */
/*
 * ★★ INDICES CORRECTED 2026-07-25 (tier-2): the rows are right, the addresses
 * were not.  There is no "default FIB region" above the real entries -- the CLS
 * FIB has 512 entries and every index is (key_row << 2) | sub_slot.  The old
 * 1024/1025/1028 exceeded the 9-bit HW address field and ALIASED onto
 * FIB[0]/[1]/[4], and FIB[4] is the WAN partition's routed-unicast action.  It
 * was harmless ONLY because these words are byte-identical to the golden rows
 * cortina-ni-rx.c programs there; one edit to either table would have made it a
 * silent WAN-only breakage indistinguishable from a hash bug.
 */
static const struct { u16 idx; u32 w[L3FE_CLS_FIB_WORDS]; } l3fe_cls_default[] = {
	{ 0, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },	/* WAN KEY[0] slot0 (was 1024) */
	{ 1, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },	/* WAN KEY[0] slot1 (was 1025) */
	{ 4, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000200 } },	/* WAN KEY[1] slot0 (was 1028) */
};

/*
 * One CLS-FIB indirect write: words then ACCESS=GO|WR|idx, poll GO clear.
 *
 * ★★ RANGE CHECK (2026-07-25, tier-2).  The CLS FIB has 512 entries (stock's
 * own table descriptor: max_entry=0x200, entry_size=0x1c, data_reg_num=7,
 * ACCESS=0x33b0 -- the last three match this driver's offsets, so the 512 is
 * authoritative).  ⚠ The ACCESS address field decodes only 9 bits, so an index
 * >= 512 does NOT fail: it ALIASES onto idx & 511 and quietly overwrites a live
 * row.  Refuse loudly instead.
 */
#define L3FE_CLS_FIB_ENTRIES	512

static int l3fe_cls_fib_write(void __iomem *ne, u16 idx, const u32 w[L3FE_CLS_FIB_WORDS])
{
	int i;

	if (idx >= L3FE_CLS_FIB_ENTRIES) {
		pr_err("cortina-l3fe: CLS FIB idx %u out of range (max %u) - the HW address field is 9 bits and would ALIAS onto FIB[%u], silently overwriting a live row; refusing\n",
		       idx, L3FE_CLS_FIB_ENTRIES - 1,
		       idx & (L3FE_CLS_FIB_ENTRIES - 1));
		return -ERANGE;
	}
	for (i = 0; i < L3FE_CLS_FIB_WORDS; i++)
		writel(w[i], ne + L3FE_CLS_FIB_DATA0 - i * 4);
	writel(L3FE_GO | L3FE_WRITE | idx, ne + L3FE_CLS_FIB_ACCESS);
	return l3fe_access_wait(ne, L3FE_CLS_FIB_ACCESS, L3FE_GO);
}

/* Commit one PP FIELD-CAM MAC-DA entry (proper ACCESS commit - see the
 * block comment at the register defines). */
static int l3fe_mac_da_cam_set(void __iomem *ne, u32 idx, const u8 *mac)
{
	if (idx >= L3FE_CAM_MAC_DA_ENTRIES)
		return -EINVAL;

	writel(((u32)mac[2] << 24) | ((u32)mac[3] << 16) |
	       ((u32)mac[4] << 8) | mac[5], ne + L3FE_PP_FIELD_CAM_DATA(0));
	writel(L3FE_CAM_MAC_DA_VLD | ((u32)mac[0] << 8) | mac[1],
	       ne + L3FE_PP_FIELD_CAM_DATA(1));
	writel(0, ne + L3FE_PP_FIELD_CAM_DATA(2));
	writel(0, ne + L3FE_PP_FIELD_CAM_DATA(3));
	writel(0, ne + L3FE_PP_FIELD_CAM_DATA(4));
	writel(L3FE_GO | L3FE_WRITE | (L3FE_CAM_SEL_MAC_DA << 16) | idx,
	       ne + L3FE_PP_FIELD_CAM_ACCESS);
	return l3fe_access_wait(ne, L3FE_PP_FIELD_CAM_ACCESS, L3FE_GO);
}

/* One CLS-KEY indirect write: 11 words then ACCESS=GO|WR|idx, poll GO clear. */
static int __maybe_unused l3fe_cls_key_write(void __iomem *ne, u16 idx,
					     const u32 w[L3FE_CLS_KEY_WORDS])
{
	int i;

	for (i = 0; i < L3FE_CLS_KEY_WORDS; i++)
		writel(w[i], ne + L3FE_CLS_KEY_ACCESS +
		       (L3FE_CLS_KEY_WORDS - i) * 4);
	writel(L3FE_GO | L3FE_WRITE | (idx & 0x7ff), ne + L3FE_CLS_KEY_ACCESS);
	return l3fe_access_wait(ne, L3FE_CLS_KEY_ACCESS, L3FE_GO);
}

/*
 * ★ The dedicated pri-6 ROUTED CLS rules -- the piece that RUNS T2 (the main
 * hash) on a routed my-MAC transit frame.  Clean-room re-expression of stock's
 * ca_l3_intf_add / convert_intf_to_cls scheme.
 *
 * KEY (cl_if_id_key_t; don't-care = msk-bit 1 with the value bits all-1, the
 * golden rows' own convention; sub-slot 0 only): mac_da_an_sel == AN_SEL(idx)
 * EXACT, so only frames whose DST-MAC hit the router-MAC CAM match -- mutually
 * exclusive with the mac_da_an_sel==0 L2UC catch-all; lspid == L3_LAN 0x19 /
 * L3_WAN 0x18 EXACT; ip/L4/VLAN/PPPoE don't-care (a transit frame's ip.da is
 * the far end, and terminating traffic is resolved by T2-MISS -> the HS_DEF
 * CPU_0 punt, never dropped); trailer cls_pri = 6, above the pri-0/1 catch-alls
 * and below the pri-7..11 traps a transit unicast does not key.
 *
 * ACTION (FIB word6): t2_ctrl_vld + t2_ctrl = the main-hash profile (LAN=1 ->
 * 0x1A00, WAN=0 -> 0x0A00, the tier-1 stock routing-default bytes) + word5
 * stage2_ctrl_vld.  ★ NO permit / dpid / mcgid / keep_orig_pkt: forwarding is
 * the T2 HIT action's, and a MISS falls to the HS_DEF CPU_0 trap.
 */
/* Refuted (tier-1 2026-07-23): stock leaves CLS rows 3/67 empty; kept
 * __maybe_unused until the golden-row t2_ctrl approach is board-proven. */
static const struct { u16 idx; u32 w[L3FE_CLS_KEY_WORDS]; }
	l3fe_cls_routed_key[] __maybe_unused = {
	/* WAN ingress: an_sel=2 (WAN MAC, CAM idx 1), lspid=0x18, pri 6 */
	{ L3FE_CLS_KEY_ROW_WAN, { 0xFFFFFFFF, 0xFFFFFFFF, 0x00030FE4, 0, 0,
				  0, 0, 0, 0, 0, 0x08180000 } },
	/* LAN ingress: an_sel=1 (LAN gateway MAC, CAM idx 0), lspid=0x19, pri 6 */
	{ L3FE_CLS_KEY_ROW_LAN, { 0xFFFFFFFF, 0xFFFFFFFF, 0x00032FE2, 0, 0,
				  0, 0, 0, 0, 0, 0x08180000 } },
	/*
	 * ★ WAN NON-IP CONTROL TRAP (the PPPoE LCP un-mangle fix).  Key:
	 * {ip_vld == 0 EXACT}, everything else don't-care; pri 8 (= stock
	 * CL_RUL_PRIO_L3_TUNNEL_PPPOE - above the pri-6 routed rule, below the
	 * pri-9 IP-multicast traps); slot 0 only.  This row lives in the WAN
	 * partition (KEY[0..63]) so it is searched only by WAN-ingress frames.
	 *
	 * WHY: with hw_l3_fwd on, a DS PPPoE frame carrying a CONTROL proto (LCP,
	 * IPCP, IPV6CP, PAP, CHAP) rode the pri-6 routed row into T2, missed, and
	 * the HS_DEF CPU punt re-emerged from the PE with the PPP proto mangled /
	 * the frame decapsulated, so pppd never saw it (BOARD-PROVEN 2026-07-20:
	 * DS LCP left hades on-wire and reached pppd 0/0; a keep_orig CPU trap
	 * delivered them and LCP+IPCP completed to a lease).
	 *
	 * Every control proto parses ip_vld=0 while session DATA parses ip_vld=1,
	 * so ip_vld==0 IS the control-vs-data split and this row steals nothing
	 * from the data path.  It replaces stock's per-proto PPP CAM scheme with
	 * one row.  Non-IP non-PPPoE WAN frames (ARP, PPPoE Discovery) land here
	 * too -- already CPU-bound, now delivered with original bytes.
	 * ★ lspid==0x18 is NOT in the key: board-proven, these DS control frames
	 * do NOT carry it at the CLS and adding it dropped every match; the WAN
	 * partition already scopes the row.  Word build: word0 0xFFFFFCFF =
	 * msk_ip_vld/ip_vld (bits 8/9), rest don't-care; trailer pri=8, slot0.
	 */
	{ L3FE_CLS_KEY_ROW_WAN_CTL, { 0xFFFFFCFF, 0xFFFFFFFF, 0x0007FFFF, 0, 0,
				      0, 0, 0, 0, 0, 0x08200000 } },
};
static const struct { u16 idx; u32 w[L3FE_CLS_FIB_WORDS]; }
	l3fe_cls_routed_fib[] __maybe_unused = {
	/* ★ CARRY the to-CPU disposition ON THE CLS ROW (w4 = permit|dpid_pri|
	 * dpid_vld 0x1C000000; w5 = mcgid CPU_0 + stage3_ctrl_vld 0x01000004),
	 * NOT on the HS_DEF miss action.  This silicon = the vendor G3 model
	 * (aal-gen2/aal_hash.c *_HASH_ACT_DEFAULT_TO_CPU: "use CLS to decide flow
	 * miss CPU port" - the HS_DEF miss default has NO dpid).  Ours had it
	 * inverted (no disposition on the row, relying on a HS_DEF dpid) so a
	 * routed T2 MISS carried no CPU disposition and was DROPPED - the enable
	 * regression.  Keep stage2_ctrl_vld (0x04000000, the hit-path NAT) +
	 * t2_ctrl (w6); a T2 HIT overrides the disposition with the flow action. */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_WAN),
	  { 0, 0, 0, 0, 0x1C000000, 0x05000004, 0x00000A00 } },
	/* LAN: run T2 profile 1, same to-CPU disposition on the row */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_LAN),
	  { 0, 0, 0, 0, 0x1C000000, 0x05000004, 0x00001A00 } },
	/*
	 * WAN non-IP control trap action: the stock unicast->CPU_0 disposition
	 * bytes (golden FIB[4]: w4 0x1C000000 = dpid_vld|dpid_pri|permit @bits
	 * 154-156, w5 mcgid=0x10 CPU_0 @158-167 + stage3_ctrl_vld @184, w6
	 * 0x200) PLUS keep_orig_pkt_vld|keep_orig_pkt (w5 bits 22/23 = abs bits
	 * 182/183, cls_fib_mod_0/1_t msk_ctrl run - offsets anchored tier-1 by
	 * dpid @154-156, mcgid @162 and stage2_ctrl_vld @186 in the same run):
	 * deliver the ORIGINAL frame bytes, no PE re-encap - so the punted LCP
	 * keeps its real PPP proto.  No t2_ctrl: a dispositioned frame gets no
	 * T2 lookup (control frames need none).
	 */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_WAN_CTL),
	  { 0, 0, 0, 0, 0x1C000000, 0x01C00004, 0x00000200 } },
};

int cortina_l3fe_intf_add(void __iomem *ne, const u8 *lan_mac)
{
	u8 wan_mac[6];
	int ret;

	if (!lan_mac)
		return -EINVAL;
	l3fe_wan_mac_derive(lan_mac, wan_mac);

	/* 1. Router MACs into the PP FIELD-CAM MAC-DA table: the PP then
	 * stamps HDR_I.mac_da_an_sel = idx+1 on every frame to that MAC. */
	ret = l3fe_mac_da_cam_set(ne, L3FE_AN_IDX_LAN, lan_mac);
	if (ret)
		return ret;
	ret = l3fe_mac_da_cam_set(ne, L3FE_AN_IDX_WAN, wan_mac);
	if (ret)
		return ret;

	/*
	 * Tier-1 stock-diff (2026-07-23, live devmem on stock NAND): stock's
	 * routed-ingress LPB profiles 0/1/2 carry mac_da_an_mask = 0, and CLS
	 * rows 3/67 (the pri-6 slots) are EMPTY.  Stock drives L3 HW-forward
	 * purely through the ALWAYS-ON golden CLS rows 0/1/2 + 64/65/66, whose
	 * FIB action already carries t2_ctrl + the CPU_0 miss disposition
	 * (cortina-ni-rx.c cls_fib_golden == stock FIB 0/4/8/256/260/264,
	 * byte-matched).  The earlier STG0 an-mask + pri-6 routed-rule scheme is
	 * NOT how stock works and corrupted the routed path on enable - removed.
	 * intf_add now only provisions the PP FIELD-CAM router-MAC entries
	 * (which stock also has: MAC-DA e0 = LAN gw, e1 = WAN = base+1).
	 */
	return 0;
}

/* One ARB LDPID->PDPID map entry (L2FE indirect, generic GO protocol). */
static int l3fe_pdpid_map_set(void __iomem *ne, u32 idx, u32 pdpid)
{
	writel(pdpid & 0xf, ne + L3FE_L2FE_PDPID_MAP_DATA);
	writel(L3FE_GO | L3FE_WRITE | idx, ne + L3FE_L2FE_PDPID_MAP_ACCESS);
	return l3fe_access_wait(ne, L3FE_L2FE_PDPID_MAP_ACCESS, L3FE_GO);
}

/* APPEND one static L2FE FDB entry {mac} -> {ldpid, valid, static, DA/SA
 * permit}.  Key packing = the vendor __aal_mac_2_fdb_data split (proven by
 * the read-back HIT on the live board); vid/scind/dot1p = 0.  The FDB engine
 * INIT already ran in the RX bring-up (cortina_ni_rx_fdb_add_cpu), which
 * always precedes this probe-time call. */
static int l3fe_fdb_static_add(void __iomem *ne, const u8 *mac, u32 ldpid)
{
	/* ★ THE KEY PACK AND THE ACTION WORD ARE THE COMMON LAYER'S, NOT OURS.
	 * This function spelled both inline, character-for-character identical to
	 * what cortina-ni-rx.c spelled -- two copies of one hardware format, and
	 * the L2FE and L3FE field vocabularies are the SAME BITS under two names:
	 * LPID GENMASK(5,0) == 0x3f, VALID BIT(9), STATIC BIT(19), DA_PERMIT
	 * BIT(20), SA_PERMIT BIT(21). Verified value-by-value before this call
	 * replaced them, because a shared spelling of two DIFFERENT formats is a
	 * far worse defect than the duplication it removes.
	 * The WRITES stay here: this file owns a different register base, and only
	 * the value computation is common. */
	u32 d3, d2, d1;
	u32 d0 = cortina_ni_l2fe_fdb_action(ldpid);

	cortina_ni_l2fe_fdb_key(mac, &d3, &d2, &d1);

	writel(0, ne + L3FE_FDB_CMD_RETURN);
	writel(d3, ne + L3FE_FDB_DATA3);
	writel(d2, ne + L3FE_FDB_DATA2);
	writel(d1, ne + L3FE_FDB_DATA1);
	writel(d0, ne + L3FE_FDB_DATA0);
	writel(L3FE_GO | L3FE_FDB_OP_APPEND, ne + L3FE_FDB_ACCESS);
	return l3fe_access_wait(ne, L3FE_FDB_ACCESS, L3FE_GO);
}

int cortina_l3fe_hw_l3_forward_enable(void __iomem *ne, const u8 *router_mac)
{
	int i, ret;

	/* 1. Internal hash-miss action -> punt to CPU (never drop).  This is
	 * the SAFETY KEYSTONE: with zero flows installed every lookup misses,
	 * so this action is what carries a routed frame to the CPU/software
	 * path unchanged.  Program it BEFORE enabling the hash consult. */
	for (i = 0; i < L3FE_HS_DEF_REG_COUNT; i++)
		writel(l3fe_def_reg_stock[i], ne + L3FE_HS_DEF_REG0_ETY0 + i * 4);

	/* 2. CLS per-profile routing DEFAULT rows, stock bytes (fallthrough CPU
	 * disposition; effectively dead behind the all-wildcard trap rows).  The
	 * REAL T2 admission is the pri-6 mac_da_an_sel rules below.
	 * ⚠ DO NOT BUILD ON "a dispositioned frame gets no T2 lookup" WITHOUT
	 * RE-MEASURING (2026-07-24).  The observation stands, the explanation is
	 * doubted: stock's own catch-alls carry dpid_vld|dpid_pri|permit AND
	 * t2_ctrl_vld at once, so a disposition evidently does not suppress T2
	 * (dpid_pri looks like the arbitration bit, t2_ctrl=0xF the real bypass).
	 * And the inertness was measured with HS_CACHE_CNT, since established as
	 * a PHANTOM -- it reads flat on a real main-hash hit on stock too. */
	for (i = 0; i < (int)ARRAY_SIZE(l3fe_cls_default); i++) {
		ret = l3fe_cls_fib_write(ne, l3fe_cls_default[i].idx,
					 l3fe_cls_default[i].w);
		if (ret)
			return ret;
	}

	/* 2b. Re-point the routed profiles' TUPLE0 maskptr at the 5-tuple-only
	 * mask (index 8) so both the SWO install-CRC and the HW lookup-CRC hash
	 * ONLY {l4_dp,l4_sp,ip_da/32,ip_sa/32,ip_protocol} - stock mask 0 also
	 * folds mac/lspid/dscp/vlan into the CRC (non-zero on a real frame, zero
	 * in the driver's sparse key) so a mask-0 install could never HIT.  Keep
	 * pri/type 0.  classify_setup already wrote mask 8's bits; this only runs
	 * under hw_l3_fwd, so gate-off leaves the profiles on the stock masks.
	 * ★ P4: profile 3 is the one the LIVE admission actually consults (the
	 * catch-all rows stamp t2_ctrl=3) - re-point it too; 0/1 kept for the
	 * dedicated per-interface rules once those key correctly. */
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_WAN));
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_LAN));
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_ROUTED));

	/* 2c. PON US egress plumbing for a T2 HIT that forwards WAN-ward
	 * (gemMapMode-1): PE gemid_map=1 + ldpid_base=0x20 so a hit-action's
	 * {mc=1, mcgid=gem, t2_ctrl1=tcont} egresses at hdr_a.ldpid 0x20+tcont
	 * = the SAME PON US ldpid the proven CPU data-TX path uses.  RMW: the
	 * reset default already carries ldpid_base=0x20; only gemid_map is
	 * added.  Zero-flow behaviour unchanged (the mode only interprets HIT
	 * actions carrying the gemMapMode encoding). */
	{
		u32 v = readl(ne + L3FE_PE_CFG);

		v &= ~(L3FE_PE_CFG_LDPID_BASE | L3FE_PE_CFG_MTU_CHK_EN);
		v |= FIELD_PREP(L3FE_PE_CFG_LDPID_BASE, L3FE_LDPID_PON_US_0) |
		     L3FE_PE_CFG_GEMID_MAP |
		     L3FE_PE_CFG_PAD_CTRL | L3FE_PE_CFG_RSVD14;
		writel(v, ne + L3FE_PE_CFG);	/* == stock 0x00105602 (tier-1 devmem) */
	}

	/* 2c-bis. PPPoE PE encap globals (one-time): header code/version/type
	 * + the v4/v6 PPP protocol numbers the PE stamps when a hit-action
	 * requests the PPPoE ADD encap.  Inert until an action carries
	 * pppoe_set/pppoe_vld - which only happens when a PPPoE WAN session is
	 * armed (cortina_ni_wan_pppoe_session_set); zero-flow and IPoE
	 * behaviour unchanged. */
	writel(L3FE_PE_PPPOE_CFG_VAL, ne + L3FE_PE_PPPOE_CFG);
	writel(L3FE_PE_PPPOE_PROT_CFG_VAL, ne + L3FE_PE_PPPOE_PROT_CFG);

	/* 2d. Slow HW ager ON (SECONDARY hit witness): with granularity 0 the
	 * ager block never runs and the age slot reads stale, so the age
	 * re-arm could never witness a hit (that broke the last cycle's
	 * witness).  The slow cadence keeps idle entries alive for far longer
	 * than any test window.  PRIMARY witness stays the forwarding change
	 * (CPU-forward counters flat while the far end still receives). */
	writel(L3FE_AGING_GRAN_SLOW, ne + L3FE_HS_AGING_GRANULARITY);

	/* 3. INGRESS ADMISSION, WAN leg: ARB LDPID->PDPID map [0x18] = 0x0a
	 * so a frame the PON PDC stamps LDPID 0x18 (L3_WAN) is physically
	 * handed to the L3FE WAN ingress (stock live [0x18]=0xA; our RX init
	 * leaves 0x18 at reset because nothing on the pre-offload datapath
	 * ever resolves to it).  All 4 {my_mac,dbuf} index combos, like the
	 * always-on map writes in cortina-ni-rx.c. */
	for (i = 0; i < 4; i++) {
		u32 idx = L3FE_LDPID_L3_WAN |
			  ((i & 1) ? L3FE_PDPID_IDX_DBUF : 0) |
			  ((i & 2) ? L3FE_PDPID_IDX_MYMAC : 0);

		ret = l3fe_pdpid_map_set(ne, idx, L3FE_PDPID_L3_WAN);
		if (ret)
			return ret;
	}

	/* 4. INGRESS ADMISSION, my-MAC recognition + the T2 admission rules:
	 * router-MAC CAM entries (0 = LAN gateway MAC, 1 = WAN MAC = base+1)
	 * + the STG0 LPB mac_da_an_mask bits + the dedicated pri-6 routed
	 * CLS rules whose action runs T2 - cortina_l3fe_intf_add(), stock's
	 * ca_l3_intf_add scheme.  Re-applied on every link-up by the
	 * cortina-ni-rx.c cls_init re-run (the my-MAC/STG0 re-init there
	 * rewrites the LPB HIGH words with the stock constants). */
	if (router_mac) {
		u8 wan_mac[6];

		ret = cortina_l3fe_intf_add(ne, router_mac);
		if (ret)
			return ret;
		l3fe_wan_mac_derive(router_mac, wan_mac);

		/* 5. ★ THE terminating DS-WAN delivery: static FDB entry
		 * {WAN MAC -> L3_WAN 0x18}.  This family keeps L2 MY-MAC
		 * detection OFF and uses a STATIC FDB to reach the L3FE, so
		 * without this entry a PON DS unicast to the WAN MAC (the DHCP
		 * OFFER, every ping reply) is a DLF and gets FLOODED OUT
		 * (proven live 2026-07-19: 0/200 hades pings without it,
		 * 200/200 + a lease + WAN 0% loss with it).  The probe-time
		 * install covers the window before the gate flips; link-up
		 * re-arms re-install it.
		 * ⚠ LPB spcl_pkt_en (0x3410/0x3428 bit20) stays at stock's 1:
		 * the L3 special-packet table behind it does not exist on this
		 * die (writing its 0x3440/0x3444 access regs SErrors) -- the
		 * bit is inert, live-A/B-verified. */
		ret = l3fe_fdb_static_add(ne, wan_mac, L3FE_LDPID_L3_WAN);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Re-point ONE main-hash profile's TUPLE0 maskptr at the routed 5-tuple mask
 * (index 8), same write cortina_l3fe_hw_l3_forward_enable() step 2b does for
 * the profiles the US leg needs.
 *
 * WHY a per-profile entry point exists: an installed entry is found only if the
 * LOOKUP hashes the packet under the SAME mask the install used.  The install
 * CRC always uses mask 8, but the lookup uses the mask of whichever profile the
 * ingress classifier stamped into HDR_I.t2_ctrl - so a direction whose CLS row
 * stamps a profile still pointing at a stock mask can never HIT.  Step 2b
 * covers profiles 0/1/3 (the ones the US/LAN admission was proven to stamp);
 * the DS (WAN-ingress) leg calls this for the remaining profiles so its
 * stamped profile cannot be the one that was left out.
 *
 * Harmless for any profile that carries no flows: the main hash holds only our
 * own entries, a mask-8 lookup that finds none falls to the same CPU punt as
 * before, and a "false" match requires an identical 5-tuple - i.e. it IS that
 * flow.  Caller must be inside the hw_l3_fwd gate (this is a datapath-config
 * write); pri/type are kept 0 exactly as step 2b writes them.
 */
int cortina_l3fe_hash_profile_mask_repoint(void __iomem *ne, u32 profile)
{
	if (profile > L3FE_MAIN_HASH_PROFILE_MAX)
		return -EINVAL;
	writel(L3FE_5TUPLE_MASK_ID, ne + L3FE_HS_PROFILE_TUPLE0(profile));
	return 0;
}

/*
 * ★ THE ONE egress L3-IF word, built the way stock builds it.
 *
 * On this die the whole entry is 24 bits and holds nothing but the egress SMAC
 * selector and the PPPoE header control -- there is no second table an interface
 * could keep its SMAC in, so ONE entry per egress interface carries both.
 * Corroborated tier-1 by the live stock WAN entry 0x00940001 = {pppoe_set=1,
 * pppoe_vld=0 (inert), session=0, mac_sa_vld=1, an_sel=2, pad_ctrl=1}: stock's
 * idle WAN entry is already pre-shaped for the overlay, so bringing a session up
 * only sets pppoe_vld and fills the id.  @session == 0 therefore reproduces that
 * exact word, byte-identical to what the IPoE path has always written.
 *
 * ★ WHY THE SMAC MATTERS (the defect this fixes): a PPPoE session is bound to
 * {session_id, peer MAC} from PADI/PADS, and the peer MAC is the ONU's WAN MAC
 * because pppd runs on the WAN netdev.  An entry that only ADDs the header
 * leaves the original source MAC -- on a routed transit frame, the LAN client's
 * -- so the access concentrator sees a session frame from an unknown MAC.
 *
 * bit23 (PAD_CTRL / pppoe_len_control) is set because stock's live WAN entry
 * sets it and our IPoE entries carried it through the board-proven 941/956 Mbps
 * runs.  Its consumer is NOT proven on this die; an on-wire PPPoE length that
 * disagrees with inner-IP-total-length + 2 is the signal to A/B it.
 */
static u32 l3fe_l3if_entry(u8 an_sel, u16 session)
{
	u32 entry = L3FE_L3IF_MAC_SA_VLD | L3FE_L3IF_PAD_CTRL |
		    L3FE_L3IF_PPPOE_SET |
		    FIELD_PREP(L3FE_L3IF_MAC_SA_AN_SEL, an_sel);

	if (session)
		entry |= L3FE_L3IF_PPPOE_VLD |
			 FIELD_PREP(L3FE_L3IF_PPPOE_SESSION, session);
	return entry;
}

static int l3fe_l3if_write(void __iomem *ne, u32 idx, u32 entry)
{
	if (idx >= L3FE_L3IF_ENTRIES)
		return -EINVAL;
	writel(entry, ne + L3FE_L3IF_DATA);
	writel(L3FE_GO | L3FE_WRITE | (idx & (L3FE_L3IF_ENTRIES - 1)),
	       ne + L3FE_L3IF_ACCESS);
	return l3fe_access_wait(ne, L3FE_L3IF_ACCESS, L3FE_GO);
}

/*
 * Program egress L3-IF entry @idx for a PPPoE WAN: substitute the egress SMAC
 * named by @an_sel (the WAN MAC) AND ADD the 8-byte 0x8864 session header for
 * @session.  A US hit-action that sets GROUP_20 {pppoe_set1, pppoe_vld1,
 * l3_if_vld1, egr_l3_if_idx1=@idx} then gets both rewrites on egress (the PE
 * globals 0x3500/0x3504 supply code/ver/type + the PPP protocol number).
 *
 * @session == 0 CLEARS the session: the entry reverts to the plain SMAC-only
 * shape (identical to the IPoE entry), so a frame that still referenced this
 * index for one packet during a teardown race leaves as a correct IPoE frame
 * rather than one carrying a stale header or the wrong source MAC.
 *
 * Called only under the hw_l3_fwd gate; an unreferenced entry is inert.
 */
int cortina_l3fe_pppoe_l3if_set(void __iomem *ne, u32 idx, u16 session,
				u8 an_sel)
{
	return l3fe_l3if_write(ne, idx, l3fe_l3if_entry(an_sel, session));
}

/* ---- L3FE HW flow-offload next-hop L2 rewrite (defect A2) -------------- *
 * A routed/NAT flow's hit-action rewrites the next-hop DMAC (mac_da_idx into
 * the HS_LIGHT indexed MAC table) + the egress SMAC (egr_l3_if_idx into the
 * L3-IF table, which selects our WAN MAC from the my-MAC CAM).  Without these
 * an offloaded flow egresses with the ONU's own DMAC (hairpin) - which is why
 * hw_l3_fwd is gated OFF.  Both are plain indirect-access register SRAM; recipe
 * RE'd from cortina-api/flow.c + aal-gen2/aal_hashlite.c + aal-77c/aal_l3_if.c
 * (the same GO/poll protocol as the mask/CLS/CAM tables). */

#define L3FE_MACDA_IDX_ENTRIES		1024

/*
 * ★ CRASH FIX (async SError on the first AUTO flow install) + aal-77c reality.
 *
 * The "HS_LIGHT indexed-interface MAC table" this used to write (ACCESS 0x3dc4,
 * DATA 0x3dc8/0x3dcc) was RE'd from aal-gen2 -- the WRONG chip tree.  On this
 * aal-77c die the L3FE window ends at ~0x3c8c, so those are UNMAPPED holes and a
 * write async-SErrors the CPU (the t=58.5 s panic on the first nf_flow_table
 * install); rtl8277c has NO HS_LIGHT register at all.
 *
 * The next-hop DMAC here comes from a different table: the egress L3-IF entry
 * carries only the SMAC, and the FIB mac_da_idx indexes the L2 FDB/LUT.  Porting
 * that is a follow-up; until then REFUSE, so cn_flow_replace keeps the flow on
 * the SW fast path -- no crash, no regression, just no HW offload of it yet.
 */
int cortina_l3fe_macda_idx_set(void __iomem *ne, u32 idx, const u8 *mac)
{
	(void)ne; (void)idx; (void)mac;
	return -EOPNOTSUPP;
}

/* IPoE egress L3-IF entry: substitute the egress SMAC from the my-MAC CAM entry
 * named by mac_sa_an_sel (WAN MAC = CAM idx 1 -> an_sel 2), PPPoE inert.  Same
 * builder as the PPPoE entry with session 0 - stock's idle WAN entry shape,
 * 0x00940001 for an_sel 2 (tier-1 live read), byte-identical to what this
 * function has written on every board-proven IPoE run. */
int cortina_l3fe_ipoe_l3if_set(void __iomem *ne, u32 idx, u8 an_sel)
{
	return l3fe_l3if_write(ne, idx, l3fe_l3if_entry(an_sel, 0));
}
