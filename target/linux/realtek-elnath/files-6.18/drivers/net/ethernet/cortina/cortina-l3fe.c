// SPDX-License-Identifier: GPL-2.0-only
/* cortina-l3fe.c - RTL9607F / Cortina CA8277C "Elnath" L3FE ...
 * dev/MEASURED-cortina-l3fe.c.md sec 1. */

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
/* ★ The hash profile the LIVE admission actually runs (P4, ...
 * dev/MEASURED-cortina-l3fe.c.md sec 2. */
#define L3FE_MAIN_HASH_PROFILE_ROUTED	3
/* the engine has 7 hash profiles (0..6), stride 0x2c from HS_PROFILE0_INI */
#define L3FE_MAIN_HASH_PROFILE_MAX	6u
#define L3FE_5TUPLE_MASK_ID		8

/* L3FE PE config (direct MMIO): the GEM-map mode for PON US ...
 * dev/MEASURED-cortina-l3fe.c.md sec 3. */
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

/* L3FE PE PPPoE encap globals (direct MMIO, one-time; tier-2 ...
 * dev/MEASURED-cortina-l3fe.c.md sec 4. */
#define L3FE_PE_PPPOE_CFG		0x3500	/* [15:8]=code 0 (session), [7:4]=ver 1, [3:0]=type 1 */
#define L3FE_PE_PPPOE_CFG_VAL		0x00000011u
#define L3FE_PE_PPPOE_PROT_CFG		0x3504	/* [15:0]=v4 PPP-proto 0x0021, [31:16]=v6 0x0057 */
#define L3FE_PE_PPPOE_PROT_CFG_VAL	0x00570021u

/* Egress L3-IF table (fe_l3e_if_tbl): 32 x 32-bit entries, ...
 * dev/MEASURED-cortina-l3fe.c.md sec 5. */
#define L3FE_L3IF_ACCESS		0x3000
#define L3FE_L3IF_DATA			0x3004
#define L3FE_L3IF_ENTRIES		32
#define L3FE_L3IF_PPPOE_SET		BIT(0)
#define L3FE_L3IF_PPPOE_VLD		BIT(1)
#define L3FE_L3IF_PPPOE_SESSION		GENMASK(17, 2)
#define L3FE_L3IF_MAC_SA_VLD		BIT(18)
#define L3FE_L3IF_MAC_SA_AN_SEL		GENMASK(22, 19)
#define L3FE_L3IF_PAD_CTRL		BIT(23)	/* = pppoe_len_control; stock sets it */

/* HW ager cadence for the gated experiment: non-zero so the ...
 * dev/MEASURED-cortina-l3fe.c.md sec 6. */
#define L3FE_AGING_GRAN_SLOW		0x08000000u

#define L3FE_GO				BIT(31)
#define L3FE_WRITE			BIT(30)
#define L3FE_MASK_UPPER128		BIT(6)
/* L2FE FDB engine (direct regs, NE window; same protocol as ...
 * dev/MEASURED-cortina-l3fe.c.md sec 7. */
#define L3FE_FDB_CMD_RETURN		0x1c2c
#define L3FE_FDB_ACCESS			0x1ca0
#define L3FE_FDB_OP_APPEND		0x45
#define L3FE_FDB_DATA3			0x1ca4
#define L3FE_FDB_DATA2			0x1ca8
#define L3FE_FDB_DATA1			0x1cac
#define L3FE_FDB_DATA0			0x1cb0
/* The five FDB FIELD bits were removed 2026-09-03: a second, ...
 * dev/MEASURED-cortina-l3fe.c.md sec 8. */
#define L3FE_L2FE_PDPID_MAP_ACCESS	0x166c
#define L3FE_L2FE_PDPID_MAP_DATA	0x1670
#define L3FE_PDPID_IDX_DBUF		BIT(6)
#define L3FE_PDPID_IDX_MYMAC		BIT(7)
#define L3FE_LDPID_L3_WAN		0x18	/* AAL_LPORT_L3_WAN */
#define L3FE_PDPID_L3_WAN		0x0a	/* AAL_PPORT_L3_WAN (stock live 0xA) */

/* L3FE PP FIELD-CAM - the my-MAC / MAC-DA recognition CAM (15 ...
 * dev/MEASURED-cortina-l3fe.c.md sec 9. */
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

/* STG0 per-profile LPB HIGH word (direct MMIO, LOW/MID/HIGH ...
 * dev/MEASURED-cortina-l3fe.c.md sec 10. */
#define L3FE_STG0_LPB_HIGH(p)		(0x3410 + (p) * 0xC)
#define L3FE_LPB_AN_MASK(sel)		BIT(10 + (sel))
#define L3FE_LPB_AN_PROFILES		3	/* prof 0..2 get the an-mask */

/* L3-CLS classifier KEY table (indirect, same GO protocol as ...
 * dev/MEASURED-cortina-l3fe.c.md sec 11. */
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

/* Stock-armed register values, live-captured 2026-07-18 ...
 * dev/MEASURED-cortina-l3fe.c.md sec 12. */
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

/* Profile/tuple + mask-table classify config, tier-1 captured ...
 * dev/MEASURED-cortina-l3fe.c.md sec 13. */
#define L3FE_MASK_5TUPLE	8	/* 5-tuple-only NAPT mask index */

/* ★ Mask-entry field geometry, L4 region (needed by the ...
 * dev/MEASURED-cortina-l3fe.c.md sec 14. */
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

/* ★ INVARIANT D (build-time): the 5-tuple mask MUST select ...
 * dev/MEASURED-cortina-l3fe.c.md sec 15. */
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
	/* ★ mask 8: 5-TUPLE-ONLY NAPT mask, from the aal-77c ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 16. */
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
/* The HASH-PROFILE block -- RE'd 2026-08-29 from the stock ...
 * dev/MEASURED-cortina-l3fe.c.md sec 17. */
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

/* Divergence B: enable HW L3-forwarding (miss -> CPU). * ...
 * dev/MEASURED-cortina-l3fe.c.md sec 18. */
static const u32 l3fe_def_reg_stock[L3FE_HS_DEF_REG_COUNT] = {
	/* entry 0: TRAP the hash miss to CPU_0 -- the SAME dpid ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 19. */
	0x00000000, 0x00300000, 0x00000000,	/* entry 0: stock hash-miss (NO dpid; the CLS row supplies the CPU port - tier-1 devmem 2026-07-23) */
	/* ★ entry 1 = TRAP -> CPU_0 too (deviation from the stock ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 20. */
	0x00009811, 0x00000000, 0x00000000,	/* entry 1: stock bridge/flood miss action 0x9811 (tier-1 devmem 2026-07-23) */
	0x00040001, 0x00300000, 0x00404000,	/* entry 2 */
	0x00009831, 0x00300000, 0x00000000,	/* entry 3 */
};

/* CLS per-profile routing DEFAULT actions - the hash-CONSULT ...
 * dev/MEASURED-cortina-l3fe.c.md sec 21. */
static const struct { u16 idx; u32 w[L3FE_CLS_FIB_WORDS]; } l3fe_cls_default[] = {
	{ 0, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },	/* WAN KEY[0] slot0 (was 1024) */
	{ 1, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },	/* WAN KEY[0] slot1 (was 1025) */
	{ 4, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000200 } },	/* WAN KEY[1] slot0 (was 1028) */
};

/* One CLS-FIB indirect write: words then ACCESS=GO|WR|idx, ...
 * dev/MEASURED-cortina-l3fe.c.md sec 22. */
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

/* ★ The dedicated pri-6 ROUTED CLS rules -- the piece that ...
 * dev/MEASURED-cortina-l3fe.c.md sec 23. */
static const struct { u16 idx; u32 w[L3FE_CLS_KEY_WORDS]; }
	l3fe_cls_routed_key[] __maybe_unused = {
	/* WAN ingress: an_sel=2 (WAN MAC, CAM idx 1), lspid=0x18, pri 6 */
	{ L3FE_CLS_KEY_ROW_WAN, { 0xFFFFFFFF, 0xFFFFFFFF, 0x00030FE4, 0, 0,
				  0, 0, 0, 0, 0, 0x08180000 } },
	/* LAN ingress: an_sel=1 (LAN gateway MAC, CAM idx 0), lspid=0x19, pri 6 */
	{ L3FE_CLS_KEY_ROW_LAN, { 0xFFFFFFFF, 0xFFFFFFFF, 0x00032FE2, 0, 0,
				  0, 0, 0, 0, 0, 0x08180000 } },
	/* ★ WAN NON-IP CONTROL TRAP (the PPPoE LCP un-mangle fix). Key
	 * dev/MEASURED-cortina-l3fe.c.md sec 24. */
	{ L3FE_CLS_KEY_ROW_WAN_CTL, { 0xFFFFFCFF, 0xFFFFFFFF, 0x0007FFFF, 0, 0,
				      0, 0, 0, 0, 0, 0x08200000 } },
};
static const struct { u16 idx; u32 w[L3FE_CLS_FIB_WORDS]; }
	l3fe_cls_routed_fib[] __maybe_unused = {
	/* ★ CARRY the to-CPU disposition ON THE CLS ROW (w4 = ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 25. */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_WAN),
	  { 0, 0, 0, 0, 0x1C000000, 0x05000004, 0x00000A00 } },
	/* LAN: run T2 profile 1, same to-CPU disposition on the row */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_LAN),
	  { 0, 0, 0, 0, 0x1C000000, 0x05000004, 0x00001A00 } },
	/* WAN non-IP control trap action: the stock unicast->CPU_0 ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 26. */
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

	/* Tier-1 stock-diff (2026-07-23, live devmem on stock NAND): ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 27. */
	return 0;
}

/* One ARB LDPID->PDPID map entry (L2FE indirect, generic GO protocol). */
static int l3fe_pdpid_map_set(void __iomem *ne, u32 idx, u32 pdpid)
{
	writel(pdpid & 0xf, ne + L3FE_L2FE_PDPID_MAP_DATA);
	writel(L3FE_GO | L3FE_WRITE | idx, ne + L3FE_L2FE_PDPID_MAP_ACCESS);
	return l3fe_access_wait(ne, L3FE_L2FE_PDPID_MAP_ACCESS, L3FE_GO);
}

/* APPEND one static L2FE FDB entry {mac} -> {ldpid, valid, ...
 * dev/MEASURED-cortina-l3fe.c.md sec 42. */
static int l3fe_fdb_static_add(void __iomem *ne, const u8 *mac, u32 ldpid)
{
	/* ★ THE KEY PACK AND THE ACTION WORD ARE THE COMMON LAYER'S, ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 28. */
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

	/* 2. CLS per-profile routing DEFAULT rows, stock bytes ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 29. */
	for (i = 0; i < (int)ARRAY_SIZE(l3fe_cls_default); i++) {
		ret = l3fe_cls_fib_write(ne, l3fe_cls_default[i].idx,
					 l3fe_cls_default[i].w);
		if (ret)
			return ret;
	}

	/* 2b. Re-point the routed profiles' TUPLE0 maskptr at the ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 30. */
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_WAN));
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_LAN));
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_ROUTED));

	/* 2c. PON US egress plumbing for a T2 HIT that forwards ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 31. */
	{
		u32 v = readl(ne + L3FE_PE_CFG);

		v &= ~(L3FE_PE_CFG_LDPID_BASE | L3FE_PE_CFG_MTU_CHK_EN);
		v |= FIELD_PREP(L3FE_PE_CFG_LDPID_BASE, L3FE_LDPID_PON_US_0) |
		     L3FE_PE_CFG_GEMID_MAP |
		     L3FE_PE_CFG_PAD_CTRL | L3FE_PE_CFG_RSVD14;
		writel(v, ne + L3FE_PE_CFG);	/* == stock 0x00105602 (tier-1 devmem) */
	}

	/* 2c-bis. PPPoE PE encap globals (one-time): header ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 32. */
	writel(L3FE_PE_PPPOE_CFG_VAL, ne + L3FE_PE_PPPOE_CFG);
	writel(L3FE_PE_PPPOE_PROT_CFG_VAL, ne + L3FE_PE_PPPOE_PROT_CFG);

	/* 2d. Slow HW ager ON (SECONDARY hit witness): with ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 33. */
	writel(L3FE_AGING_GRAN_SLOW, ne + L3FE_HS_AGING_GRANULARITY);

	/* 3. INGRESS ADMISSION, WAN leg: ARB LDPID->PDPID map [0x18] ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 34. */
	for (i = 0; i < 4; i++) {
		u32 idx = L3FE_LDPID_L3_WAN |
			  ((i & 1) ? L3FE_PDPID_IDX_DBUF : 0) |
			  ((i & 2) ? L3FE_PDPID_IDX_MYMAC : 0);

		ret = l3fe_pdpid_map_set(ne, idx, L3FE_PDPID_L3_WAN);
		if (ret)
			return ret;
	}

	/* 4. INGRESS ADMISSION, my-MAC recognition + the T2 admission ...
	 * dev/MEASURED-cortina-l3fe.c.md sec 35. */
	if (router_mac) {
		u8 wan_mac[6];

		ret = cortina_l3fe_intf_add(ne, router_mac);
		if (ret)
			return ret;
		l3fe_wan_mac_derive(router_mac, wan_mac);

		/* 5. ★ THE terminating DS-WAN delivery: static FDB entry {WAN ...
		 * dev/MEASURED-cortina-l3fe.c.md sec 36. */
		ret = l3fe_fdb_static_add(ne, wan_mac, L3FE_LDPID_L3_WAN);
		if (ret)
			return ret;
	}

	return 0;
}

/* Re-point ONE main-hash profile's TUPLE0 maskptr at the ...
 * dev/MEASURED-cortina-l3fe.c.md sec 37. */
int cortina_l3fe_hash_profile_mask_repoint(void __iomem *ne, u32 profile)
{
	if (profile > L3FE_MAIN_HASH_PROFILE_MAX)
		return -EINVAL;
	writel(L3FE_5TUPLE_MASK_ID, ne + L3FE_HS_PROFILE_TUPLE0(profile));
	return 0;
}

/* ★ THE ONE egress L3-IF word, built the way stock builds it. ...
 * dev/MEASURED-cortina-l3fe.c.md sec 38. */
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

/* Program egress L3-IF entry @idx for a PPPoE WAN: substitute ...
 * dev/MEASURED-cortina-l3fe.c.md sec 39. */
int cortina_l3fe_pppoe_l3if_set(void __iomem *ne, u32 idx, u16 session,
				u8 an_sel)
{
	return l3fe_l3if_write(ne, idx, l3fe_l3if_entry(an_sel, session));
}

/* L3FE HW flow-offload next-hop L2 rewrite (defect A2) ...
 * dev/MEASURED-cortina-l3fe.c.md sec 40. */

#define L3FE_MACDA_IDX_ENTRIES		1024

/* ★ CRASH FIX (async SError on the first AUTO flow install) + ...
 * dev/MEASURED-cortina-l3fe.c.md sec 41. */
int cortina_l3fe_macda_idx_set(void __iomem *ne, u32 idx, const u8 *mac)
{
	(void)ne; (void)idx; (void)mac;
	return -EOPNOTSUPP;
}

/* IPoE egress L3-IF entry: substitute the egress SMAC from ...
 * dev/MEASURED-cortina-l3fe.c.md sec 43. */
int cortina_l3fe_ipoe_l3if_set(void __iomem *ne, u32 idx, u8 an_sel)
{
	return l3fe_l3if_write(ne, idx, l3fe_l3if_entry(an_sel, 0));
}
