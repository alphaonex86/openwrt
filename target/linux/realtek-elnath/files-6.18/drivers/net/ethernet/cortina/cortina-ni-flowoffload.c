// SPDX-License-Identifier: GPL-2.0-only
/* cortina-ni-flowoffload.c - nf_flow_table HW offload glue ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 1. */

#include <linux/kernel.h>
#include "cortina_ni_flowoffload_logic.h"	/* hoisted logic */
#include "cortina_vlan_install.h"	/* the DMA-AFT VLAN word layout, once */
#include <linux/module.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bitrev.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/inetdevice.h>
#include <linux/io.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <net/flow_offload.h>
#include <net/pkt_cls.h>
#include <net/netfilter/nf_flow_table.h>

#include "gpon_flow.h"	/* the core's TC->5-tuple decode */
#include "gpon_flow_block.h"
#include "gpon_flow_offload.h"	/* the core TC-offload lifecycle */
#include "cortina-access.h"	/* the ONE indirect transaction */
#include "cortina-ni.h"
#include "cortina-l3fe.h"
#include "cortina-l3fe-regs.h"	/* the L3FE registers more than one file needs */

/* L3FE main-hash ("HS") engine registers, offsets from the NE ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 132. */

#define CN_L3E_HS_PROFILE_INI(p)	(0x3700 + (p) * 0x2c)	/* tpl_num[3:0], default_sel_0e[8:4], 0a[13:9], 1e[18:14], 1a[23:19] */
#define CN_L3E_HS_PROFILE_TUPLE(p, t)	(0x3704 + (p) * 0x2c + (t) * 4) /* maskptr[5:0], pri[10:8], type[12] */
#define CN_L3E_HS_PROFILE_T0_ACTION(p)	(0x3724 + (p) * 0x2c)	/* a_mask[24:0], fetch_sz[27:25] */
#define CN_L3E_HS_OVERFLOW_INI		0x3848	/* oa_width[2:0] */
#define CN_L3E_HS_BA_OA0		0x3850	/* overflow FIB base */
#define CN_L3E_HS_DEFAULT_INI		0x3854	/* da_width[2:0] */
#define CN_L3E_HS_BA_DA0		0x385c	/* default FIB base */
#define CN_L3E_HS_DEFAULT_ACTION(i)	(0x3860 + (i) * 4) /* fib_addr[24:0] | da_width<<25 */
#define CN_L3E_HS_BA_CA0		0x38a8	/* cache FIB base */
#define CN_L3E_HS_CACHE_AGE10		0x38b8	/* 16-bit cache aging units, ages 0/1 */
#define CN_L3E_HS_CACHE_AGE32		0x38bc	/* ages 2/3 */
/* action-cache utilisation count (ut_cnt[11:0]); climbs as the on-chip action
 * cache fills on HW hits.  07f offset = the ca8277b HS_CACHE_CNT (0x3900) minus
 * the live-verified 0x40 cache-block shift on this die = 0x38c0. */
#define CN_L3E_HS_CACHE_CNT		0x38c0
#define CN_L3E_HS_OVERFLOW_ACCESS	0x3904	/* 64-entry overflow key CAM (unused in phase 1) */
#define CN_L3E_HS_MASK_DATA(n)		(0x3920 - (n) * 4) /* MASK0..3 = 0x3920,191c,1918,1914 */
/* ★ MAIN-HASH age SRAM on THIS die = 2 DATA words, 16 ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 2. */
#define CN_L3E_HS_PF_KEY(p)		(0x394c + (p) * 0x14) /* sel[5:0]=0 CRC16, crc32_sel[7:6] */
/* # of per-profile hash key-selection blocks (the vendor key-selection writer
 * covers 6; TUPLE0/INI exist for 7 profiles).  All must read ZERO or each
 * profile would rotate/XOR the tuple differently - see invariant (B) in
 * cn_l3e_verify_profile_invariants(). */
#define CN_L3E_PF_KEY_PROFILES		6
#define CN_L3E_HS_PF_TPL_SP(p)		(0x3950 + (p) * 0x14)
#define CN_L3E_HS_PF_TPL_DP(p)		(0x3954 + (p) * 0x14)
#define CN_L3E_HS_PF_TPL_SIP(p)		(0x3958 + (p) * 0x14)
#define CN_L3E_HS_PF_TPL_DIP(p)		(0x395c + (p) * 0x14)

#define CN_L3E_GO			BIT(31)	/* indirect-access request/busy */
#define CN_L3E_WRITE			BIT(30)	/* indirect-access direction */
/* geometry (live-stock HASH_INI = 0x0003007D, devmem-captured ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 3. */
#define CN_L3E_ENTRIES			65536	/* ht_size = 7 */
#define CN_L3E_AGE_ROWS			(CN_L3E_ENTRIES / CN_L3E_AGE_SLOTS)
#define CN_L3E_FIB_BYTES		32	/* ha_width = 3 (256-bit, normal mode) */

/* CN_L3E_HASH_WAYS, CN_L3E_AGE_SLOTS and the 2-bit age codes ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 4. */
#define CN_L3E_PROFILE_WAN		0
#define CN_L3E_PROFILE_LAN		1
/* ★ P4: the profile the LIVE routed admission actually stamps ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 133. */
#define CN_L3E_PROFILE_ROUTED		3
/* 7 hash profiles (0..6), stride 0x2c - the DS gate re-points them all at the
 * 5-tuple mask so the DS-stamped profile cannot be one that was left out. */
#define CN_L3E_PROFILE_MAX		6

/* mask-table index per profile (= PROFILE_TUPLE.maskptr the ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 5. */
#define CN_L3E_WAN_MASK_ID		8	/* routed IPv4 5-tuple mask */
#define CN_L3E_LAN_MASK_ID		8	/* routed flow, either direction */
#define CN_L3E_BRIDGE_MASK_ID		1	/* L2 (MAC) key, non-routed */

/*
 * Action FIB, "normal" mode = action groups 18 + 20 (a_mask 0x140000),
 * 224 bits packed, fetched as one 256-bit FIB entry.
 */
struct cn_l3e_act {
	/* group 18 - forward/permit (19 bits) */
	u64 mrr_vld		: 1;
	u64 mrr_en		: 1;
	u64 no_drop_vld		: 1;
	u64 no_drop		: 1;
	u64 dpid_vld		: 1;
	u64 dpid_pri		: 1;
	u64 permit		: 1;
	u64 deepq		: 1;
	/*
	 * ★★ TRUE GROUP_18 TAIL (tier-2, four agreeing sources in the shipped
	 * binaries: the action dumper's ubfx reads, the serializer's bfi stores,
	 * aal_hash_actionGrpBitmask_length_get's "group 18 = 17 bits", and
	 * fc_mgr.ko's own writer):
	 *     bits  8..15  mcgid/ldpid       (8 bits, NOT 10)
	 *     bit   16     mc   - 1: the field is an MCGID, 0: it is an LDPID
	 *     bit   17     mdata_byte_vld    (the first bit of group 20)
	 * The 10-bit form below is REAL but belongs to OTHER tables - the hash KEY
	 * (bits 694..703), the L3-CLS FIB and HDR_I - which is the origin of the
	 * whole +2-bit family of errors this file has already been bitten by.
	 *
	 * ★ Kept as one 10-bit field ON PURPOSE: cn_l3e_set_us_egress writes
	 * CN_L3E_WAN_EGR_MCGID(gem) = (0x20 + gem) << 3, whose bit 16 is what supplies
	 * mc=1 for the PON egress, so narrowing it to 8 bits WITHOUT also making the
	 * US builder set mc explicitly silently regresses the 956 Mbps upstream.  The
	 * DS side is unaffected either way: its value is <= 0xff, so mc and
	 * mdata_byte_vld come out 0 - exactly what an Ethernet-port egress wants.  A
	 * host test pins the US macro's decode under the true layout.
	 */
	u64 mcgid		: 10;
	/* ★ aal-77c FIB layout fix (2026-07-24, tier-1): this die ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 6. */
	u64 mdata_byte		: 8;
	u64 l3_if_vld		: 1;
	u64 smac_trans		: 1;
	u64 igr_l3_if_idx	: 6;
	u64 egr_l3_if_idx	: 6;
	u64 l3_if_counter_en	: 1;
	u64 ip_ttl_dec		: 1;
	u64 ip_ttl_zero_drop	: 1;
	u64 ip_addr_vld		: 1;
	u64 ip_type		: 1;	/* which address is rewritten: 0 = SA, 1 = DA */
	u64 ip_addr		: 32;	/* the new IPv4 address */
	u64 ip_addr_napt6	: 1;
	u64 mac_da_idx_vld	: 1;
	u64 mac_da_idx		: 13;	/* next-hop MAC via the MAC-DA table */
	u64 chk_msk_ptr		: 6;
	u64 cache_ctrl		: 2;
	u64 pop_l3_vld		: 1;
	u64 pop_l3_chk_ecn_en	: 1;
	u64 pop_l3_en		: 1;
	u64 t2_ctrl_vld		: 1;
	u64 t2_ctrl		: 4;
	u64 ldpid_offset_msb	: 1;
	u64 ip_dscp_update_en	: 1;
	u64 ip_dscp		: 6;
	u64 cos_update_en	: 1;
	u64 cos			: 3;
	u64 inner_pcp_update_en	: 1;
	u64 inner_pcp		: 3;
	u64 top_pcp_update_en	: 1;
	u64 top_pcp		: 3;
	u64 inner_dei		: 1;
	u64 inner_vid		: 12;
	u64 inner_tpid_enc	: 3;
	u64 top_dei		: 1;
	u64 top_vid		: 12;
	u64 top_tpid_enc	: 3;
	u64 vlan_cnt		: 2;
	u64 vlan_vld		: 1;
	u64 pol_vld		: 1;
	u64 pol_en		: 1;
	u64 pol_id		: 8;
	u64 pol2_id_en		: 1;
	u64 pol2_id		: 6;
	u64 pol3_id_en		: 1;
	u64 pol3_id		: 6;
	u64 pppoe_vld		: 1;
	u64 pppoe_set		: 1;
	u64 l4_port		: 16;	/* the new L4 port */
	u64 ip_mtu_enc_vld	: 1;
	u64 ip_mtu_enc		: 4;
	u64 modify_vlan_only_vld : 1;
	u64 modify_vlan_only	: 1;
	u64 sixrd_fmr_idx_vld	: 1;
	u64 sixrd_fmr_idx	: 2;
	u64 vxlan_sport_msb15	: 6;
	u64 vxlan_sport_update	: 1;
	/* pad to the 32-byte FIB entry (34 = 32 + the 2 bits freed by dropping
	 * the aal-gen2 mc/mdata_byte_vld above; the 256-bit entry is unchanged). */
	u64 pad			: 34;
} __packed;

static_assert(sizeof(struct cn_l3e_act) == CN_L3E_FIB_BYTES);

/* ------------------------------------------------------------------ */
/* backend context (filled by cn_l3e_init() from the cortina-ni probe  */
/* in the build/wiring phase; NULL = offload rejected everywhere)      */
/* ------------------------------------------------------------------ */

struct cn_flow_priv;

struct cn_l3e {
	struct device	*dev;
	/* the owning NI instance - needed for cortina_ni_nihv_sample(), which is
	 * the ONE reader of the read-and-clear NI_HV counters this file also
	 * reports (see the note in cortina-ni.h) */
	struct cortina_ni *ni;
	void __iomem	*ne_base;	/* NE register window */
	void __iomem	*dma_base;	/* DMA/LDMA window - the DMA-AFT tables
					 * that carry the WAN VLAN edit live
					 * here (CA_NI_WIN_DMA, 0x4_f7001000) */
	spinlock_t	reg_lock;	/* serializes indirect GO cycles */
	/* DMA-AFT allocation state, guarded by aft_lock.  Tiny by construction:
	 * 64 fib entries and 64 map entries, and this board has ONE WAN VLAN. */
	spinlock_t	aft_lock;
	u64		aft_fib_used;	/* bitmap over CA_DMA_AFT_FIB_COUNT */
	u64		aft_map_used;	/* bitmap over CA_DMA_AFT_MAP_COUNT */
	u16		aft_fib_vid[CA_DMA_AFT_FIB_COUNT];   /* content key: vid */
	u8		aft_fib_cnt[CA_DMA_AFT_FIB_COUNT];   /* content key: tags */
	u8		aft_fib_ref[CA_DMA_AFT_FIB_COUNT];   /* share by content */
	u8		aft_fib_map[CA_DMA_AFT_FIB_COUNT][2];/* the map pair the fib
						      * owns; freed with the FIB,
						      * never with whichever flow
						      * happens to release last */
	/* ledger - every arm says which one fired, because that instrumentation
	 * is why the DS-leg misread was diagnosable at all */
	u32		aft_push;	/* US legs programmed with a tag   */
	u32		aft_strip;	/* DS legs programmed to pop       */
	u32		aft_reuse;	/* fib shared with an existing one */
	u32		aft_no_tpid;	/* REFUSED: no TPID slot matched   */
	u32		aft_tpid_armed;	/* L3FE PP TPID slots we programmed */
	u32		aft_full;	/* REFUSED: fib or map table full  */
	u32		aft_timeout;	/* REFUSED: a GO poll never cleared*/
	/* DDR carve (one dma_alloc_coherent block, key then FIB - the NE
	 * fabric is NON-coherent, a cached carve = stale matches) */
	void		*carve;
	dma_addr_t	carve_pa;
	u32		*key_tbl;	/* 64K x u32 CRC32 */
	dma_addr_t	key_tbl_pa;
	void		*fib_tbl;	/* 64K x 32 B actions */
	dma_addr_t	fib_tbl_pa;
	/* lean SW shadow (allocated by cn_l3e_init; ~0.9 MB total - never
	 * the vendor's >7 MB per-entry kmalloc model) */
	u32		*shadow_crc32;	/* per entry, 0 = free */
	u16		*shadow_crc16;	/* for cache-invalidate on delete */
	struct cn_flow_priv **entry_by_idx;	/* sweep reverse map */
	u8		*bucket_occ;	/* entries per AGE row: sweep skip mask */
	/* SWO HW-CRC selftest verdict (phase-1 gate instrument) */
	int		selftest_ret;
	u32		selftest_pass;
	u32		selftest_fail;
	/* HDR_I 5-tuple key-packing verdict (divergence-A gate): each 5-tuple
	 * field must move the SWO CRC under the 5-tuple mask */
	u32		hdri_live_pass;
	u32		hdri_live_fail;
	/* router (LAN) MAC for the my-MAC FIELD-CAM commit; WAN = base+1 */
	u8		router_mac[ETH_ALEN];
	bool		router_mac_valid;
	/* LIVE PON data-path identity, reported by the GPON driver at data-GEM
	 * install (cortina_ni_gpon_data_path_set): the US hit-action egresses
	 * WAN-ward via mcgid=data_gem (mc=1) + t2_ctrl=data_tcont.  0 gem = no
	 * data path armed (US forward action is left CPU-only). */
	u16		data_gem;
	u8		data_tcont;
	/* LIVE PPPoE WAN session id (0 = IPoE WAN / no session - the ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 134. */
	u16		data_pppoe_session;
};

static struct cn_l3e *cn_l3e;

/* ------------------------------------------------------------------ */
/* CRC over the masked key (SW path; verified against the HS_SWO HW    */
/* CRC engine by a bring-up selftest before first use)                 */
/* ------------------------------------------------------------------ */

static int cn_l3e_key_hash(struct cn_l3e *l3e, const struct cn_l3e_key *key,
			   int profile, u32 mask_id, u32 *crc32_out,
			   u16 *crc16_out)
{
	u32 hdri[CN_L3E_HDRI_WORDS];
	unsigned long flags;
	int ret;

	cn_l3e_build_hdri(key, profile, hdri);

	/* SWO engine == lookup CRC, by construction (the engine hashes the
	 * HDR_I).  Serialized under reg_lock; flow-add is process/workqueue
	 * context, never the packet path. */
	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = cortina_l3fe_swo_crc(l3e->ne_base, hdri, CN_L3E_HDRI_WORDS,
				   mask_id, crc32_out, crc16_out);
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	return ret;
}

/* indirect-access primitives Age access: ACCESS = bucket | GO ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 7. */
static int cn_l3e_age_set(struct cn_l3e *l3e, u32 idx, u32 age)
{
	u32 bucket = (idx >> 5) & (CN_L3E_AGE_ROWS - 1);
	struct pi_packed_slot slot = cn_age2_slot(idx);
	u32 data_reg = slot.reg ? L3FE_HS_AGE_DATA_HI
				: L3FE_HS_AGE_DATA_LO;
	const char *phase = "latch";
	unsigned long flags;
	u32 w;
	int ret;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = l3fe_access_go(l3e->ne_base, L3FE_HS_AGE_ACCESS, bucket | CN_L3E_GO,
			CN_L3E_GO);
	if (ret)
		goto out;

	w = readl(l3e->ne_base + data_reg);
	w = pi_packed_insert(w, &slot, age);
	writel(w, l3e->ne_base + data_reg);

	phase = "commit";
	ret = l3fe_access_go(l3e->ne_base, L3FE_HS_AGE_ACCESS,
			bucket | CN_L3E_WRITE | CN_L3E_GO, CN_L3E_GO);
out:
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	/* which GO timed out matters: a "commit" timeout means the age write
	 * WAS issued and may land late - the entry can go live after an error
	 * return, so the caller must fully undo (blackhole-safety). */
	if (ret)
		pr_err("cortina-l3fe: age_set idx=%u age=%u: %s GO timeout (%d)\n",
		       idx, age, phase, ret);
	return ret;
}

/* single-entry age read: the P2 bring-up oracle ("did my one flow HIT?" -
 * age > IDLE(1), i.e. re-armed to START(2), = matched since install; 1 =
 * live but idle; 0 = not live).  2-bit main-hash slot (see cn_l3e_age_set).
 * NEVER used on the stats path - that is the batch sweep's job. */
static int __maybe_unused cn_l3e_age_get(struct cn_l3e *l3e, u32 idx, u32 *age)
{
	u32 bucket = (idx >> 5) & (CN_L3E_AGE_ROWS - 1);
	struct pi_packed_slot slot = cn_age2_slot(idx);
	u32 data_reg = slot.reg ? L3FE_HS_AGE_DATA_HI
				: L3FE_HS_AGE_DATA_LO;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = l3fe_access_go(l3e->ne_base, L3FE_HS_AGE_ACCESS, bucket | CN_L3E_GO,
			CN_L3E_GO);
	if (!ret)
		*age = pi_packed_extract(readl(l3e->ne_base + data_reg), &slot);
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	return ret;
}

/* Batch had-traffic read+clear of one 32-slot bucket in a ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 8. */
static int cn_l3e_bucket_sweep(struct cn_l3e *l3e, u32 bucket, u32 *traffic)
{
	unsigned long flags;
	u32 w[2], n[2], trf = 0;
	int r, ret;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = l3fe_access_go(l3e->ne_base, L3FE_HS_AGE_ACCESS, bucket | CN_L3E_GO,
			CN_L3E_GO);
	if (ret)
		goto out;

	w[0] = readl(l3e->ne_base + L3FE_HS_AGE_DATA_LO);  /* slots 0-15  */
	w[1] = readl(l3e->ne_base + L3FE_HS_AGE_DATA_HI);  /* slots 16-31 */
	/* the read-and-step-down transform is the logic core's (one spelling
	 * of the 2-bit slot packing, shared with age_set/age_get) */
	for (r = 0; r < 2; r++) {
		u16 t;

		n[r] = cn_age2_sweep_word(w[r], &t);
		trf |= (u32)t << (r * 16);
	}
	writel(n[0], l3e->ne_base + L3FE_HS_AGE_DATA_LO);
	writel(n[1], l3e->ne_base + L3FE_HS_AGE_DATA_HI);

	ret = l3fe_access_go(l3e->ne_base, L3FE_HS_AGE_ACCESS,
			bucket | CN_L3E_WRITE | CN_L3E_GO, CN_L3E_GO);
out:
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	*traffic = trf;
	return ret;
}

/* Action-cache invalidate - MANDATORY after every ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 135. */
static int cn_l3e_cache_invalidate(struct cn_l3e *l3e, u32 idx, u16 crc16)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	/* slot = way within the HASH bucket (idx & (bucket_size-1); 8-way) */
	writel((idx & (CN_L3E_HASH_WAYS - 1)) | ((u32)crc16 << 5) | (1u << 30),
	       l3e->ne_base + L3FE_HS_CACHE_CTRL);

	/* the engine must be idle BEFORE the request is raised: this wait is on
	 * REQ, not on the CTRL word just written -- hence a bare wait and not
	 * the write-then-wait pair. */
	ret = l3fe_access_wait(l3e->ne_base, L3FE_HS_CACHE_CTRL_REQ, BIT(0));
	if (!ret)
		ret = l3fe_access_go(l3e->ne_base, L3FE_HS_CACHE_CTRL_REQ,
				readl(l3e->ne_base + L3FE_HS_CACHE_CTRL_REQ) | 1,
				BIT(0));
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	if (ret)
		pr_err("cortina-l3fe: cache_invalidate idx=%u crc16=%04x FAILED (%d) - a stale cached action may keep matching\n",
		       idx, crc16, ret);
	return ret;
}

/* flow add / delete on the engine Install one entry with a ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 9. */
static int cn_l3e_flow_add_rawcrc(struct cn_l3e *l3e, u32 crc32, u16 crc16,
				  const struct cn_l3e_act *act, u32 *idx_out)
{
	u32 idx;
	int ret;

	/* the way-pick (dup scan + free scan + the entry-0 guard) is the logic
	 * core's cn_hs_way_pick(); only the ledger lines stay here */
	ret = cn_hs_way_pick(l3e->shadow_crc32, crc16, crc32, &idx);
	if (ret == -EEXIST) {
		/* normal dup-key (not an error): flow already installed */
		pr_debug("cortina-l3fe: flow_add: EEXIST idx=%u crc32=%08x crc16=%04x\n",
			 idx, crc32, crc16);
		return -EEXIST;
	}
	if (ret == -ENOSPC) {
		/* bucket full: the flow simply stays on the sw path (not an error) */
		pr_debug("cortina-l3fe: flow_add: bucket FULL base=%u crc16=%04x\n",
			 idx, crc16);
		return -ENOSPC;
	}

	/* 1. action FIB, 2. key word, 3. age = go-live (order matters) */
	memcpy(l3e->fib_tbl + (size_t)idx * CN_L3E_FIB_BYTES, act,
	       CN_L3E_FIB_BYTES);
	l3e->key_tbl[idx] = crc32;
	/* both tables live in a non-cacheable/coherent carve; make the
	 * stores visible to the engine before arming the age */
	wmb();

	ret = cn_l3e_age_set(l3e, idx, CN_L3E_AGE_START);
	if (ret) {
		/* ★ Blackhole-safety: a "commit" GO timeout means the age ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 10. */
		l3e->key_tbl[idx] = 0;
		memset(l3e->fib_tbl + (size_t)idx * CN_L3E_FIB_BYTES, 0,
		       CN_L3E_FIB_BYTES);
		wmb();
		cn_l3e_age_set(l3e, idx, CN_L3E_AGE_FREE);
		cn_l3e_cache_invalidate(l3e, idx, crc16);
		return ret;
	}

	/* ★ HIT-WITNESS INTEGRITY, on every install path. The aging ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 11. */
	ret = cn_l3e_age_set(l3e, idx, CN_L3E_AGE_IDLE);
	if (ret)
		pr_warn_ratelimited("cortina-l3fe: flow_add: idx=%u age step-down to IDLE FAILED (%d) - the entry is LIVE but its first sweep will over-count one hw_hit\n",
				    idx, ret);

	l3e->shadow_crc32[idx] = crc32;
	l3e->shadow_crc16[idx] = crc16;
	*idx_out = idx;
	return 0;
}

static int cn_l3e_flow_add(struct cn_l3e *l3e, const struct cn_l3e_key *key,
			   const struct cn_l3e_act *act, int profile,
			   u32 mask_id, u32 *idx_out, u16 *crc16_out)
{
	u32 crc32;
	u16 crc16;
	int ret;

	ret = cn_l3e_key_hash(l3e, key, profile, mask_id, &crc32, &crc16);
	if (ret) {
		pr_err("cortina-l3fe: flow_add: SWO key-hash timeout (%d)\n",
		       ret);
		return ret;	/* SWO timeout: refuse, flow stays on the sw path */
	}

	ret = cn_l3e_flow_add_rawcrc(l3e, crc32, crc16, act, idx_out);
	if (ret)
		return ret;
	*crc16_out = crc16;
	return 0;
}

static int cn_l3e_flow_del(struct cn_l3e *l3e, u32 idx, u16 crc16)
{
	int age_ret, inv_ret;

	/* ★ Blackhole-safety: run EVERY teardown step even when one ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 12. */
	l3e->key_tbl[idx] = 0;
	memset(l3e->fib_tbl + (size_t)idx * CN_L3E_FIB_BYTES, 0,
	       CN_L3E_FIB_BYTES);
	wmb();
	l3e->shadow_crc32[idx] = 0;
	l3e->shadow_crc16[idx] = 0;

	age_ret = cn_l3e_age_set(l3e, idx, CN_L3E_AGE_FREE);
	inv_ret = cn_l3e_cache_invalidate(l3e, idx, crc16);

	return age_ret ? age_ret : inv_ret;
}

/* mainline flow_block glue (mtk_ppe_offload-shaped) one ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 13. */
struct cn_aft_ref {
	u8	fib;
	bool	valid;		/* holds a reference that must be released */
};

/* ★ THIS IS NO LONGER AN ENTRY, IT IS THE CORE'S `priv`. The ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 14. */
struct cn_flow_priv {
	u32			hash_idx;
	u16			crc16;
	unsigned long		last_hit;	/* fed by the stats sweep */
	u32			hits;		/* cumulative age re-arms seen for
						 * THIS flow - the per-flow leg of
						 * the us_hits/ds_hits witness */
	bool			ds;		/* the DS (WAN->LAN) reply leg */
	bool			pppoe;		/* the action carries the PPPoE push
						 * - feeds the pppoe_* ledger */
	unsigned long		installed_at;	/* for the PPPoE flap witness: an
						 * entry destroyed within
						 * CN_PPPOE_FLAP_MS of install is
						 * the GAP-2 HW->SW flap */
	u8			probe;		/* hw_ds_probe mode this entry was
						 * installed under (0 = real action) */
	struct cn_aft_ref	aft;		/* the hardware WAN VLAN edit this
						 * flow uses, if any */
};

static bool cn_flow_table_ready;

/* The core's TC-offload engine handle: it owns the cookie map, the entry
 * lifetime and the action decode; this driver supplies cn_flow_ops. */
static struct gpon_flow_offload *cn_fo;
/* Flow installs stay OFF until the P3 key-packing is SWO-validated: an
 * entry installed with the placeholder CRC could never match (harmless)
 * but would waste table slots and report a false "offloaded" state. */
static bool cn_l3e_install_ok;

/* ★ Divergence B gate. When set, cn_l3e_init also programs ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 15. */
static bool hw_l3_fwd = true;
module_param(hw_l3_fwd, bool, 0444);
MODULE_PARM_DESC(hw_l3_fwd,
	"initialize HW L3 forwarding at probe (default ON; boot-time only)");

/* ★ STATUS. Both legs are HW-forwarded today. The 2026-07-20 ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 16. */
static bool hw_pppoe = true;
static int hw_pppoe_set(const char *val, const struct kernel_param *kp);

static const struct kernel_param_ops hw_pppoe_ops = {
	.flags	= KERNEL_PARAM_OPS_FL_NOARG,	/* `hw_pppoe` with no value = 1,
						 * same as a plain bool param */
	.set	= hw_pppoe_set,
	.get	= param_get_bool,
};
module_param_cb(hw_pppoe, &hw_pppoe_ops, &hw_pppoe, 0644);
MODULE_PARM_DESC(hw_pppoe,
	"Enable PPPoE hardware flows (default ON). Requires hw_l3_fwd at boot; downstream also requires hw_ds_offload and cortina_gpon.hw_l3_ds. Disabling retires flows and clears the session before publishing the new value; errors leave the gate enabled for retry. Enabling affects new flows.");

/* ★ DS (WAN->LAN) offload leg. Default ON since 2026-07-25, ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 17. */
static bool hw_ds_offload = true;
module_param(hw_ds_offload, bool, 0644);
MODULE_PARM_DESC(hw_ds_offload,
	"install downstream HW flows (default ON; runtime changes affect new flows only); requires the data GEM routed into L3FE via cortina_gpon.hw_l3_ds=1");

/* DS LAN-egress port override, -1 = resolve it from the L2FE ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 18. */
static int hw_ds_lan_ldpid = -1;
module_param(hw_ds_lan_ldpid, int, 0644);
MODULE_PARM_DESC(hw_ds_lan_ldpid,
	"force the DS LAN-egress LDPID (0..6 = physical NI port; -1 = resolve from the L2FE FDB entry, default)");

/* # of installed nf_flow_table flows (the /proc auto_flows counter); defined
 * here so the PPPoE session-set path below can gate its BUG-B flush on it. */
static atomic_t cn_flow_installed = ATOMIC_INIT(0);

/* Cumulative HW-HIT witness (`hw_hits`). THE stock-validated ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 19. */
static atomic_t cn_l3e_hw_hits = ATOMIC_INIT(0);

/* ★ PER-DIRECTION HW-HIT witnesses, split out of ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 20. */
static atomic_t cn_l3e_us_hits = ATOMIC_INIT(0);
static atomic_t cn_l3e_ds_hits = ATOMIC_INIT(0);
static atomic_t cn_l3e_hits_unattr = ATOMIC_INIT(0);

/* ★★ PPPoE PER-STAGE LEDGER (`pppoe_stage:` + ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 21. */
#define CN_PPPOE_FLAP_MS		2000
static atomic_t cn_pppoe_arms = ATOMIC_INIT(0);
static atomic_t cn_pppoe_arm_fail = ATOMIC_INIT(0);
static atomic_t cn_pppoe_installed = ATOMIC_INIT(0);
static atomic_t cn_pppoe_us_hits = ATOMIC_INIT(0);
static atomic_t cn_pppoe_ds_hits = ATOMIC_INIT(0);
static atomic_t cn_pppoe_ds_refused = ATOMIC_INIT(0);
static atomic_t cn_pppoe_us_refused = ATOMIC_INIT(0);
static atomic_t cn_pppoe_early_gone = ATOMIC_INIT(0);

/* Un-account a removed flow from the PPPoE ledger. ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 22. */
static void cn_pppoe_entry_gone(const struct cn_flow_priv *e, bool count_flap)
{
	if (!e->pppoe)
		return;
	atomic_dec(&cn_pppoe_installed);
	if (count_flap &&
	    time_before(jiffies,
			e->installed_at + msecs_to_jiffies(CN_PPPOE_FLAP_MS)))
		atomic_inc(&cn_pppoe_early_gone);
}

/* "is this device on the LAN side of the router?" - a bridge ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 23. */
static bool cn_dev_is_lan_side(const struct net_device *dev)
{
	return dev && (netif_is_any_bridge_port(dev) ||
		       netif_is_bridge_master(dev));
}

/* ★★ GAP-2 INSTRUMENT - the DS PPPoE punt integrity check. ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 24. */
bool cortina_ni_pppoe_punt_check;
module_param_named(pppoe_punt_check, cortina_ni_pppoe_punt_check, bool, 0644);
MODULE_PARM_DESC(pppoe_punt_check,
	"inspect every CPU-punted 0x8864 PPPoE session frame for self-consistency (PPPoE length vs inner IP total length, inner TCP data-offset sanity, session id) and report in debugfs .../cortina-l3fe/state under `pppoe_punt:` - the GAP-2 (DS-mangle) witness. Default OFF; run it with hw_pppoe=0 FIRST to establish the clean baseline");

static atomic_t cn_pppoe_punt_seen = ATOMIC_INIT(0);
static atomic_t cn_pppoe_punt_ctrl = ATOMIC_INIT(0);
static atomic_t cn_pppoe_punt_len_bad = ATOMIC_INIT(0);
static atomic_t cn_pppoe_punt_tcp_bad = ATOMIC_INIT(0);
static atomic_t cn_pppoe_punt_sid_bad = ATOMIC_INIT(0);
static atomic_t cn_pppoe_punt_short = ATOMIC_INIT(0);
/* ★ THE DENOMINATOR (added 2026-07-25). `seen` counts every ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 25. */
static atomic_t cn_pppoe_punt_data = ATOMIC_INIT(0);
/* ★ Malformation SHAPE discriminators, so ONE live run says ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 26. */
static atomic_t cn_pppoe_punt_shift8 = ATOMIC_INIT(0);
static atomic_t cn_pppoe_punt_dblenc = ATOMIC_INIT(0);
/* the armed shadow disagreeing with the wire - a DIFFERENT question from
 * sid_bad, and deliberately no longer folded into it (see the inspect shell) */
static atomic_t cn_pppoe_punt_sid_vs_armed = ATOMIC_INIT(0);
/* first session id seen on the wire. ★ It is the ONLY sid ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 136. */
static u16 cn_pppoe_punt_sid_seen;

/* Imperative shell: run the predicate on a punted frame and account it. */
void cortina_ni_pppoe_punt_inspect(const u8 *f, unsigned int len)
{
	struct cn_pppoe_punt_info pi;
	u16 exp_sid, armed;
	u32 v;

	/* ★ ONE reference in every mode: the first session id seen ON THE WIRE.
	 * See cn_pppoe_punt_sid_seen - comparing against the armed shadow when one
	 * existed made the hw_pppoe=1 run answer a different question than its own
	 * baseline.  The wire-vs-armed question is kept, separately, below. */
	exp_sid = cn_pppoe_punt_sid_seen;
	v = cn_pppoe_punt_classify(f, len, exp_sid, &pi);
	if (!(v & CN_PPPOE_PUNT_SESSION))
		return;

	atomic_inc(&cn_pppoe_punt_seen);
	if (!cn_pppoe_punt_sid_seen)
		cn_pppoe_punt_sid_seen = pi.sid;
	if (v & CN_PPPOE_PUNT_DATA)
		atomic_inc(&cn_pppoe_punt_data);
	armed = cn_l3e ? READ_ONCE(cn_l3e->data_pppoe_session) : 0;
	if (armed && pi.sid != armed)
		atomic_inc(&cn_pppoe_punt_sid_vs_armed);
	if (v & CN_PPPOE_PUNT_SID_BAD) {
		atomic_inc(&cn_pppoe_punt_sid_bad);
		pr_warn_ratelimited("cortina-l3fe: pppoe_punt sid %#x != the first sid seen on the wire %#x (armed=%#x)\n",
				    pi.sid, exp_sid, armed);
	}
	if (v & CN_PPPOE_PUNT_CTRL)
		atomic_inc(&cn_pppoe_punt_ctrl);
	if (v & CN_PPPOE_PUNT_SHORT)
		atomic_inc(&cn_pppoe_punt_short);
	if (v & CN_PPPOE_PUNT_LEN_BAD) {
		atomic_inc(&cn_pppoe_punt_len_bad);
		if (v & CN_PPPOE_PUNT_SHIFT8)
			atomic_inc(&cn_pppoe_punt_shift8);
		if (v & CN_PPPOE_PUNT_DBLENC)
			atomic_inc(&cn_pppoe_punt_dblenc);
		/* The head bytes ARE the evidence: shape (shift8/dblenc/neither)
		 * plus the first 32 bytes settle what edited the frame, which no
		 * counter can.  %*ph is bounded by the min() below. */
		pr_warn_ratelimited("cortina-l3fe: pppoe_punt MANGLED sid=%#x pppoe_len=%u inner{ver=%u ihl=%u total_len=%u} expected pppoe_len=%u frame_len=%u shape=%s head=%*ph\n",
				    pi.sid, pi.pppoe_len, pi.ip_ver, pi.ihl,
				    pi.ip_len, pi.ip_len + 2, len,
				    (v & CN_PPPOE_PUNT_DBLENC) ? "DOUBLE-ENCAP" :
				    (v & CN_PPPOE_PUNT_SHIFT8) ? "8-BYTE-INSERT" :
								 "NEITHER (not an encap edit - suspect the punt buffer)",
				    (int)min(len, 32u), f);
	}
	if (v & CN_PPPOE_PUNT_TCP_BAD) {
		atomic_inc(&cn_pppoe_punt_tcp_bad);
		pr_warn_ratelimited("cortina-l3fe: pppoe_punt TCP header implausible sid=%#x doff=%u ihl=%u ip_len=%u flags=%#02x - the ~8-byte-shift signature\n",
				    pi.sid, pi.tcp_doff, pi.ihl, pi.ip_len,
				    pi.tcp_flags);
	}
}

/* ★★ DS STAGE DISCRIMINATOR - the one instrument that ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 27. */
static int hw_ds_probe;
module_param(hw_ds_probe, int, 0644);
MODULE_PARM_DESC(hw_ds_probe,
	"DS stage discriminator: 0 = real DS egress action (default), 1 = match-only (mrr_vld=0; proves ingress+hash with NO datapath change), 2 = CPU_0-punt hit-action");

/* ★ DS-leg deepq (the ARB dbuf bit) - see the long note in ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 28. */
static bool hw_ds_deepq;
module_param(hw_ds_deepq, bool, 0644);
MODULE_PARM_DESC(hw_ds_deepq,
	"DS action deepq/dbuf bit: 0 = ARB identity row -> the physical LAN port (default, the fix), 1 = the vendor's unconditional deepq -> PPORT_QM on our current ARB map (needs PDPID_MAP[0x40..0x46] reprogrammed first)");

/* CPU_0 LDPID = the L3FE punt destination, i.e. the mcgid the always-on CLS
 * rows and the HS_DEF miss action already carry (tier-1 golden CLS FIB
 * word5 mcgid field).  Used only by hw_ds_probe=2. */
#define CN_L3E_CPU0_MCGID		0x10

/* ★ __maybe_unused below: read ONLY by the CONFIG_GPON_FLOW_DIAG-gated
 * cortina_ni_l3fe_debug_show(), so =n leaves them unused and WERROR fails.
 * NI_HV per-interface RX packet counters - read-only witnesses
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 29. */
static u64 __maybe_unused cn_l3e_ni_rx_prev[2];

/*
 * ★★ L3FE GLOBAL DEBUG / MONITOR BLOCK - the engine's OWN per-stage
 * instrumentation, and the answer to "which of ingress / hash-match / egress
 * failed".  Tier-2 (stock ca-ne.ko: aal_l3fe_glb_dbg_get,
 * aal_l3fe_glb_cls_stg_monitor_get, aal_l3fe_glb_dbg_latch_trigger_set,
 * aal_l3fe_glb_dbg_latch_monitor_get), each corroborated by the vendor sibling
 * header field-for-field.
 *
 * ★ Two indirect read ports, each {index register, data register}, no GO/busy
 * handshake - write the index, read the data:
 *   DBG   0x30b8/0x30bc  index = (vector << 5) | word,  vector 0..31
 *   CLS   0x30b0/0x30b4  index = BIT(8) | (vector << 5) | word  (enable = BIT(8))
 * plus a one-shot LATCH that freezes one frame's descriptor (0x30c0/0x30c4/
 * 0x30c8, below).
 *
 * ★ WHY THE LATCH MATTERS: the vendor's own help text warns that the unlatched
 * taps are a read-mux over LIVE pipeline shadow registers, so different words
 * of one "dump" can come from DIFFERENT packets - they are gauges, never
 * coherent snapshots.
 *
 * ★★ CORRECTION OF OUR OWN HEADER (2026-07-25): cortina-ni-regs.h calls 0x30b4
 * "GLB_LF_CFG" and 0x30bc "GLB_ILPB_00" and the RX bring-up WRITES both, but
 * per the tier-2 accessors above they are the CLS-monitor RETURN and the DBG
 * DATA read-data ports - so those writes are INERT and the claim that the
 * "LF_CFG thresholds" unblocked the L3FE ingress FIFO is a false attribution.
 * Left alone (they are on the shipping-proven boot path) but not to be trusted
 * as the reason anything works.  Nothing below writes 0x30b4/0x30bc.
 */
#define CN_L3E_GLB_DBG_IDX		0x30b8	/* [11:5] vector, [4:0] word */
#define CN_L3E_GLB_DBG_DAT		0x30bc
#define CN_L3E_GLB_DBG_VEC_PKTCNT	15	/* 2 words = 4 lanes of 10 bits */
#define CN_L3E_GLB_LATCH_TRIG		0x30c0
#define  CN_L3E_LATCH_MODE		BIT(1)
#define  CN_L3E_LATCH_ARM		BIT(0)	/* toggle, one capture per flip */
#define CN_L3E_GLB_LATCH_CTRL		0x30c4	/* [7:5] vector, [4:0] word */
#define CN_L3E_GLB_LATCH_DAT		0x30c8
#define CN_L3E_LATCH_VEC_HDRI_PRE_PE	2	/* HDR_I before the packet editor:
						 * every lookup resolved, so it
						 * carries the engine's own key
						 * CRC32/CRC16, hash_idx,
						 * hash_profile, the CLS hit
						 * class and hash_dbl_chk_fail */
#define CN_L3E_LATCH_VEC_HDRI_INGRESS	0	/* HDR_I between PP and STG0 */
#define CN_L3E_LATCH_WORDS		31	/* 124 B, the descriptor size */

/* The four 10-bit per-stage packet counters (DBG vector 15). ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 30. */
enum {
	CN_L3E_STG_IN,		/* frames entering the L3FE */
	CN_L3E_STG_OUT,		/* frames leaving the L3FE */
	CN_L3E_STG_T1_T2,	/* frames at the CLS / main-hash stage */
	CN_L3E_STG_STG3_PE,	/* frames at STG3 / the packet editor */
	CN_L3E_STG_N
};
static const char * const __maybe_unused cn_l3e_stage_name[CN_L3E_STG_N] = {
	"l3fe_in", "l3fe_out", "t1_t2", "stg3_pe"
};
static u16 __maybe_unused cn_l3e_stage_prev[CN_L3E_STG_N];
static bool __maybe_unused cn_l3e_stage_seen;

static void __maybe_unused cn_l3e_stage_read(struct cn_l3e *l3e,
					     u16 c[CN_L3E_STG_N])
{
	u32 w[2];
	int i;

	for (i = 0; i < 2; i++) {
		writel((CN_L3E_GLB_DBG_VEC_PKTCNT << 5) | i,
		       l3e->ne_base + CN_L3E_GLB_DBG_IDX);
		w[i] = readl(l3e->ne_base + CN_L3E_GLB_DBG_DAT);
	}
	c[CN_L3E_STG_IN]      = w[0] & 0xffff;
	c[CN_L3E_STG_OUT]     = w[0] >> 16;
	c[CN_L3E_STG_T1_T2]   = w[1] & 0xffff;
	c[CN_L3E_STG_STG3_PE] = w[1] >> 16;
}

/* Arm one latch capture: set latch mode, then TOGGLE the arm bit.  The next
 * frame the parser sees is frozen into the read-out port. */
static void cn_l3e_latch_arm(struct cn_l3e *l3e)
{
	u32 v = readl(l3e->ne_base + CN_L3E_GLB_LATCH_TRIG);

	v |= CN_L3E_LATCH_MODE;
	writel(v, l3e->ne_base + CN_L3E_GLB_LATCH_TRIG);
	v ^= CN_L3E_LATCH_ARM;
	writel(v, l3e->ne_base + CN_L3E_GLB_LATCH_TRIG);
}

static void __maybe_unused cn_l3e_latch_read(struct cn_l3e *l3e, int vector,
					     u32 *w, int n)
{
	int i;

	for (i = 0; i < n; i++) {
		writel(((u32)vector << 5) | (u32)i,
		       l3e->ne_base + CN_L3E_GLB_LATCH_CTRL);
		w[i] = readl(l3e->ne_base + CN_L3E_GLB_LATCH_DAT);
	}
}

/* latch read-out buffer + which vector is staged, filled on the /proc read
 * after `echo latch [vector] > /proc/cortina_l3fe` armed a capture */
static u32 __maybe_unused cn_l3e_latch_buf[CN_L3E_LATCH_WORDS];
static int cn_l3e_latch_vec = -1;	/* -1 = not armed */

/* ★★ PER-ENTRY TRAFFIC BITMAP - a NON-DESTRUCTIVE hit ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 31. */
#define CN_L3E_HS_TRAFFIC_WORD(idx)	((0x4000u + 4u * ((idx) >> 5)) & 0xfffcu)
#define CN_L3E_HS_TRAFFIC_MAX_IDX	16383u	/* the 0xfffc mask ceiling */

/* # of installed DS (WAN->LAN) legs, a subset of ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 32. */
static atomic_t cn_ds_installed = ATOMIC_INIT(0);
static atomic_t cn_ds_last_ldpid = ATOMIC_INIT(-1);

/* Serializes every flow install/destroy/sweep and the PPPoE session arm/clear
 * (defined here rather than beside the flow table below because the PON
 * data-path teardown above the flow table needs it - GAP-3). */
static DEFINE_MUTEX(cn_flow_offload_mutex);

/* Flush every installed nf_flow_table flow (defined after the flow table
 * below); used on a PPPoE sid change - see BUG-B. */
static int cn_l3e_flush_auto_flows(struct cn_l3e *l3e);

/* Cross-module gate probe for the WAN-side ingress admission: ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 33. */
bool cortina_ni_hw_l3_fwd_active(void)
{
	return hw_l3_fwd && cn_l3e && cn_l3e_install_ok;
}
EXPORT_SYMBOL_GPL(cortina_ni_hw_l3_fwd_active);

/* ★ The DS PDC route the GPON driver actually programmed for ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 34. */
static int cn_ds_pdc_into_l3fe = -1;

void cortina_ni_gpon_ds_route_set(bool into_l3fe)
{
	cn_ds_pdc_into_l3fe = into_l3fe ? 1 : 0;
}
EXPORT_SYMBOL_GPL(cortina_ni_gpon_ds_route_set);

/* Refresh the backend's router-MAC shadow when the netdev MAC ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 35. */
void cortina_ni_flowoffload_router_mac_set(const u8 *mac)
{
	struct cn_l3e *l3e = cn_l3e;

	if (!l3e || !mac)
		return;
	ether_addr_copy(l3e->router_mac, mac);
	l3e->router_mac_valid = true;
}

/* The GPON driver reports the LIVE data-path identity (data ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 36. */
void cortina_ni_gpon_data_path_set(u16 gem_id, u8 tcont_idx)
{
	struct cn_l3e *l3e;

	/* ⚠ THE GLOBAL IS ACQUIRED INSIDE THE LOCK, not tested ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 37. */
	mutex_lock(&cn_flow_offload_mutex);
	l3e = cn_l3e;
	if (!l3e) {
		mutex_unlock(&cn_flow_offload_mutex);
		return;
	}
	WRITE_ONCE(l3e->data_gem, gem_id);
	WRITE_ONCE(l3e->data_tcont, tcont_idx);
	pr_info("cortina-l3fe: live PON data-path gem=%u tcont=%u\n",
		gem_id, tcont_idx);
	/* ★ GAP-3: the WAN data path going away takes any PPPoE ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 38. */
	if (!gem_id && READ_ONCE(l3e->data_pppoe_session))
		cortina_ni_wan_pppoe_session_set(0);	/* lock already held */
	mutex_unlock(&cn_flow_offload_mutex);
}
EXPORT_SYMBOL_GPL(cortina_ni_gpon_data_path_set);

/* GROUP_18/20 offset within the PON US ldpid map (aal_l3pe ldpid_base=0x20);
 * a T-CONT <= 7 rides the deep queue (vendor flow.c:1116). */
#define CN_L3E_PON_DEEPQ_TCONT_MAX	7
/* ★ The routed WAN-egress destination (GROUP_18 mcgid) is the ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 39. */
#define CN_L3E_PON_LDPID_BASE		0x20
#define CN_L3E_WAN_EGR_MCGID(tcont)	(((CN_L3E_PON_LDPID_BASE + (tcont)) & 0x3f) << 3)

/* The dedicated egress L3-IF entry carrying the PPPoE WAN ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 40. */
#define CN_L3E_PPPOE_L3IF_IDX		1

/* A2 next-hop L2 rewrite (IPoE US LAN->WAN). L3-IF entry 2 ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 137. */
#define CN_L3E_IPOE_L3IF_IDX		2
#define CN_L3E_IPOE_AN_SEL		2
/* Both WAN egress entries (IPoE idx 2, PPPoE idx 1) substitute the SAME source
 * MAC - the ONU's WAN MAC - so they share one an_sel; the only difference is
 * whether the PPPoE ADD is armed. */
#define CN_L3E_WAN_AN_SEL		CN_L3E_IPOE_AN_SEL
#define CN_L3E_MACDA_GW_IDX		0

/* DS (WAN->LAN) egress SMAC: the mirror of the US entry ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 41. */
#define CN_L3E_LAN_L3IF_IDX		3
#define CN_L3E_LAN_AN_SEL		1

/* ★★ TRUE GROUP_18 LAYOUT (tier-2, recovered from the stock ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 42. */
#define CN_L3E_LAN_EGR_MCGID(ldpid)	((ldpid) & 0xff)
#define CN_L3E_LAN_PORT_LDPID_MAX	6u	/* NI ports 0..6 (ARB identity map) */

/* LIVE PPPoE WAN session push (0 = torn down / IPoE). Same ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 43. */
int cortina_ni_wan_pppoe_session_set(u16 session)
{
	struct cn_l3e *l3e = cn_l3e;
	unsigned long flags;
	int ret = 0;

	if (!l3e)
		return -ENODEV;

	/* ★ BUG-B: a REAL session-id change invalidates every ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 44. */
	if (session != READ_ONCE(l3e->data_pppoe_session) &&
	    atomic_read(&cn_flow_installed)) {
		int stale = cn_l3e_flush_auto_flows(l3e);

		/* The session DID change upstream, so the new L3-IF is still
		 * programmed below.  What this reports is narrower and worth
		 * saying: at least one hardware entry still rewrites with the
		 * OLD session id.  It keeps its software owner, so the next
		 * flush or destroy retries it. */
		if (stale)
			pr_warn_ratelimited("cortina-l3fe: a flow installed for PPPoE session %#x would not retire (%d); it still rewrites with the old session until it is retried\n",
					    READ_ONCE(l3e->data_pppoe_session),
					    stale);
	}

	if (hw_l3_fwd) {
		spin_lock_irqsave(&l3e->reg_lock, flags);
		ret = cortina_l3fe_pppoe_l3if_set(l3e->ne_base,
						  CN_L3E_PPPOE_L3IF_IDX,
						  session, CN_L3E_WAN_AN_SEL);
		spin_unlock_irqrestore(&l3e->reg_lock, flags);
	}
	/* ★ BUG-A: commit the shadow ONLY after the HW L3-IF entry is ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 45. */
	if (!ret)
		WRITE_ONCE(l3e->data_pppoe_session, session);
	if (ret)
		atomic_inc(&cn_pppoe_arm_fail);
	else if (session)
		atomic_inc(&cn_pppoe_arms);
	pr_info("cortina-l3fe: PPPoE WAN session %#x %s (L3-IF[%u] = WAN SMAC an_sel %u + %s, ret=%d)\n",
		session, session ? "armed" : "cleared",
		CN_L3E_PPPOE_L3IF_IDX, CN_L3E_WAN_AN_SEL,
		session ? "PPPoE ADD" : "PPPoE inert", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(cortina_ni_wan_pppoe_session_set);

static int hw_pppoe_set(const char *val, const struct kernel_param *kp)
{
	struct kernel_param parsed = *kp;
	bool next;
	int ret;

	parsed.arg = &next;
	ret = param_set_bool(val, &parsed);
	if (ret)
		return ret;
	mutex_lock(&cn_flow_offload_mutex);
	if (hw_pppoe && !next && cn_l3e) {
		ret = cn_l3e_flush_auto_flows(cn_l3e);
		if (!ret && READ_ONCE(cn_l3e->data_pppoe_session))
			ret = cortina_ni_wan_pppoe_session_set(0);
	}
	if (!ret)
		WRITE_ONCE(hw_pppoe, next);
	mutex_unlock(&cn_flow_offload_mutex);
	return ret;
}

/* Stamp the US (LAN->WAN) routed PON egress into a ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 47. */
static int cn_l3e_set_us_egress(struct cn_l3e *l3e, struct cn_l3e_act *act,
				u16 pppoe)
{
	u16 gem = READ_ONCE(l3e->data_gem);
	u8 tcont = READ_ONCE(l3e->data_tcont);

	if (!gem)
		return -ENODEV;

	/* ★ WAN-egress forward action, matched field-for-field to the ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 48. */
	act->mrr_vld = 1;
	act->mcgid = CN_L3E_WAN_EGR_MCGID(tcont);
	act->dpid_vld = 1;
	act->permit = 1;
	act->dpid_pri = 1;
	act->deepq = (tcont <= CN_L3E_PON_DEEPQ_TCONT_MAX);
	act->t2_ctrl = tcont & 0xf;	/* GROUP_20 t2_ctrl1 = T-CONT selector */
	act->pop_l3_vld = 1;		/* stock sets this on the routed WAN egress */

	/* Point the entry's "double check" at THIS flow's mask (8). ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 49. */
	act->chk_msk_ptr = CN_L3E_WAN_MASK_ID;
	act->cache_ctrl = 1;		/* TYPE0 */

	/* PPPoE WAN egress: ADD the 8-byte 0x8864 session header on ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 50. */
	if (pppoe) {
		if (pppoe != READ_ONCE(l3e->data_pppoe_session)) {
			/* ★ BUG-A: PROPAGATE the L3-IF write result. If the HW L3-IF ...
			 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 51. */
			int r = cortina_ni_wan_pppoe_session_set(pppoe);

			if (r)
				return r;
		}
		/* ★ KNOWN DEVIATION FROM STOCK, left in place deliberately. ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 52. */
		act->pppoe_set = 1;
		act->pppoe_vld = 1;
		act->l3_if_vld = 1;
		act->egr_l3_if_idx = CN_L3E_PPPOE_L3IF_IDX;
	} else {
		/* A2 IPoE US egress: the egress SMAC (our WAN MAC ...cc) ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 53. */
		act->l3_if_vld = 1;
		act->egr_l3_if_idx = CN_L3E_IPOE_L3IF_IDX;
	}
	return 0;
}

/* cn_l3e_set_us_wan_vlan() - put the WAN's 802.1Q tag ON the ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 54. */
static int cn_l3fe_tpid_ensure(struct cn_l3e *l3e, u16 tpid);	/* below */

static int cn_l3e_set_us_wan_vlan(struct cn_l3e *l3e, struct cn_l3e_act *act,
				  u16 vid)
{
	int slot;

	if (!vid)
		return 0;		/* untagged WAN: leave the block at zero */
	slot = cn_l3fe_tpid_ensure(l3e, CA_DMA_AFT_TPID_8021Q);
	if (slot < 0)
		return slot;
	act->vlan_vld = 1;		/* SET mode (not a valid bit) */
	act->vlan_cnt = 1;		/* one tag on the wire after the edit */
	act->top_vid = vid;
	act->top_tpid_enc = slot + 1;	/* 1-BASED; 0 would mean NO TAG */
	act->top_pcp = 0;
	act->top_dei = 0;
	return 0;
}

/* cn_l3e_set_ds_wan_vlan() - make the DS leg POP the WAN tag, ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 55. */
static void cn_l3e_set_ds_wan_vlan(struct cn_l3e_act *act, bool tagged)
{
	if (!tagged)
		return;			/* untagged WAN: nothing arrives to strip */
	act->vlan_vld = 1;		/* SET mode (not a valid bit) */
	act->vlan_cnt = 0;		/* zero tags on the wire after the edit */
	act->top_vid = 0;
	act->top_tpid_enc = 0;		/* 0 = no tag; no slot to resolve */
	act->top_pcp = 0;
	act->top_dei = 0;
}

/*
 * Stamp the DS (WAN->LAN) routed LAN egress into a hit-action - the MIRROR of
 * cn_l3e_set_us_egress above.  Every field that is not direction-specific is
 * kept IDENTICAL to the board-proven US shape (that shape is the only routed
 * forward+NAT action known to actually egress on this silicon); the four that
 * differ, and why:
 *
 *   mcgid   US: the PON egress port-group (ldpid_base 0x20 + T-CONT) << 3.
 *           DS: the LAN egress port-group = @lan_ldpid << 3, where @lan_ldpid is
 *           the physical NI port the client's MAC was learned on.
 *   deepq   US: set for T-CONT <= 7 - the PON US deep-buffer/QM path.
 *           DS: 0.  A LAN NI port egresses directly; the deep-queue rows of the
 *           ARB map (index bit6 dbuf=1) resolve to the QM, not to the port, so a
 *           deep-queued LAN egress would leave the wire path.  See the deepq
 *           note at its assignment for the ARB coupling and the open fork.
 *   t2_ctrl US: the T-CONT selector - under PE gemid_map=1 the PE derives
 *           hdr_a.ldpid = PE_CFG.ldpid_base + t2_ctrl for a PON egress.
 *           DS: 0 = no offset; the LAN destination is carried by mcgid alone.
 *   egr_l3_if_idx  US: entry 2 = substitute the WAN MAC as egress SMAC.
 *           DS: entry 3 = substitute the LAN/router MAC (an_sel 1).
 *
 * pop_l3_vld, chk_msk_ptr and cache_ctrl are carried over unchanged: pop_l3_vld
 * is what stock sets on a routed egress (with pop_l3_en left 0 = "valid, and the
 * answer is do not pop"), and chk_msk_ptr/cache_ctrl are the A1 fix - a 0 there
 * makes the engine re-validate under mask 0 on a match, fail, and divert the
 * frame off egress, so the entry forwards nothing.
 *
 * The NAT rewrite itself (ip_type=1 = rewrite the DESTINATION address,
 * ip_addr/l4_port = the client's original IP/port) is filled by the caller from
 * the reply rule's mangle actions, as is mac_da_idx.
 */
/*
 * One-time HW arm for the DS leg.  Idempotent, and callable BOTH at probe (when
 * the bootarg already set hw_ds_offload) and lazily from the first DS install
 * (when the operator flips the param at runtime) - a runtime flip must not be
 * able to arm DS entries against an unprogrammed L3-IF[3], which would
 * blackhole them.  A board boot is the scarce resource here, so the param stays
 * writable and this makes that safe.
 *
 *  (a) the LAN-egress SMAC L3-IF entry (idx 3, an_sel 1 = the router/LAN MAC via
 *      the my-MAC CAM) - the mirror of the US entry 2.  A DS hit-action selects
 *      it, so without it an offloaded reply leaves with the wrong source MAC.
 *  (b) point EVERY hash profile's TUPLE0 at the 5-tuple mask.  Step 2b of
 *      cortina_l3fe_hw_l3_forward_enable() already does profiles 0/1/3.  An
 *      entry's install CRC always uses mask 8, but the LOOKUP uses the mask of
 *      whichever profile the ingress CLS stamped into HDR_I.t2_ctrl - so a DS
 *      frame stamped with a profile still pointing at a stock mask could never
 *      match.  ★ CONCRETE, not hypothetical: the pri-9 CLS rows stamp hash
 *      profile 2, step 2b does NOT cover it, and whether those rows key on
 *      IP-multicast or on any MAC-DA CAM hit - in which case a DS unicast to
 *      our WAN MAC matches them first - could not be settled offline.  Covering
 *      all 7 removes the question.  Safe: the main hash holds only our own
 *      entries, a mask-8 lookup that finds none falls to the same CPU punt as
 *      before, and a match still requires an identical 5-tuple.
 *
 * Caller holds cn_flow_offload_mutex (or is the single-threaded probe).
 * Returns 0 or -errno; on error the caller disables the DS gate.
 */
static bool cn_ds_armed;

static int cn_l3e_arm_ds(struct cn_l3e *l3e)
{
	unsigned long flags;
	int p, ret;

	if (cn_ds_armed)
		return 0;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = cortina_l3fe_ipoe_l3if_set(l3e->ne_base, CN_L3E_LAN_L3IF_IDX,
					 CN_L3E_LAN_AN_SEL);
	for (p = 0; !ret && p <= CN_L3E_PROFILE_MAX; p++)
		ret = cortina_l3fe_hash_profile_mask_repoint(l3e->ne_base, p);
	spin_unlock_irqrestore(&l3e->reg_lock, flags);

	if (ret)
		return ret;
	cn_ds_armed = true;
	pr_info("cortina-l3fe: DS (WAN->LAN) offload leg ARMED (LAN SMAC L3-IF[%d] an_sel=%d, hash profiles 0..%d -> 5-tuple mask %d)\n",
		CN_L3E_LAN_L3IF_IDX, CN_L3E_LAN_AN_SEL, CN_L3E_PROFILE_MAX,
		CN_L3E_WAN_MASK_ID);
	/* ★ Arming this leg is necessary but NOT sufficient: the DS ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 56. */
	if (cn_ds_pdc_into_l3fe == 0)
		pr_warn("cortina-l3fe: DS leg armed but the PON DS route is CPU_0 + FE_BYPASS - DS frames bypass the L3FE and NO DS entry can be hit; boot cortina_gpon.hw_l3_ds=1 as well (see /proc/cortina_l3fe ds_pdc)\n");
	return 0;
}

static void cn_l3e_set_ds_egress(struct cn_l3e_act *act, u32 lan_ldpid)
{
	act->mrr_vld = 1;		/* forward/action-valid - without it the
					 * engine matches but never commits egress */
	/* Destination = the LAN NI port number verbatim, mc = 0 (which a value
	 * <= 0xff in this field gives us for free - see the GROUP_18 layout note
	 * at CN_L3E_LAN_EGR_MCGID).  The caller has already range-checked it to
	 * 0..6, the vendor's own bounds on this field. */
	act->mcgid = CN_L3E_LAN_EGR_MCGID(lan_ldpid);
	act->dpid_vld = 1;
	act->permit = 1;
	act->dpid_pri = 1;		/* let the L3FE's port decision win over
					 * the downstream lookup */
	/*
	 * ★★ deepq - DEFAULT 0 FOR A LAN-PORT EGRESS.  This was 1, which contradicted
	 * this driver's own GROUP_18 layout note and, more to the point, contradicted
	 * our own ARB map: cortina_ni_arb_lan_map_init() programs, for every LAN ldpid
	 * 0..6,
	 *     [my_mac<<7 |         ldpid] -> pdpid = ldpid  (identity = the RJ45)
	 *     [my_mac<<7 | BIT(6) | ldpid] -> pdpid = CA_NI_PPORT_QM (0x08)
	 * and the action's deepq IS the dbuf bit that selects between those two rows
	 * (tier-2: aal_port_arb_ldpid_pdpid_map_set composes the index as
	 * {arg1<<7 | dbuf<<6 | ldpid[5:0]}).  So deepq=1 on an action whose mcgid is a
	 * physical LAN port resolved to the QUEUE MANAGER instead of the port: the
	 * entry HITS, the frame is "forwarded", and it leaves the wire path - a
	 * textbook stage-C failure that no hit counter can see.
	 *
	 * ★ There is a genuine fork here, and it is NOT settled offline: the vendor
	 * sets deepq=1 unconditionally on EVERY flow type including Ethernet egress
	 * (tier-2) and stock's LAN egress demonstrably works, which can only mean
	 * stock's own PDPID_MAP[dbuf=1 | 0..6] does NOT point at the QM - and we have
	 * never read those rows from stock.  The ONE stock read that decides it is
	 * PDPID_MAP[0x00..0x06] and [0x40..0x46] via caregt on a stock boot; both
	 * candidate fixes and their cost are in
	 * dev/MEASURED-x400axf-pppoe-and-ds-leg-hw-offload-2026-08-03.md.  hw_ds_deepq
	 * exists so they can be A/B'd in a single boot.
	 */
	act->deepq = hw_ds_deepq ? 1 : 0;
	/* ★ gemMapMode = 0. These four bits (pop_l3_vld, ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 57. */
	act->pop_l3_vld = 0;
	act->pop_l3_chk_ecn_en = 0;
	act->pop_l3_en = 0;
	act->t2_ctrl_vld = 0;
	act->t2_ctrl = 0;
	act->ldpid_offset_msb = 0;
	/* On a match the engine re-derives the hash under the mask ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 58. */
	act->chk_msk_ptr = CN_L3E_LAN_MASK_ID;
	act->cache_ctrl = 1;		/* TYPE0 */
	/* Egress SMAC = the LAN/router MAC, via the dedicated L3-IF ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 59. */
	act->l3_if_vld = 1;
	act->egr_l3_if_idx = CN_L3E_LAN_L3IF_IDX;
	act->smac_trans = 0;		/* the L3-IF supplies the SMAC; the vendor
					 * explicitly clears this on the routed path */
}

/* Apply the hw_ds_probe override to a fully-built DS action - ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 60. */
static void cn_l3e_ds_probe_apply(struct cn_l3e_act *act)
{
	switch (hw_ds_probe) {
	case 1:
		/* MATCH-ONLY: keep every field, drop the forward commit.  The
		 * lookup still matches and re-arms the age; nothing egresses. */
		act->mrr_vld = 0;
		break;
	case 2:
		/* PUNT: the miss disposition expressed as a HIT action.  No
		 * address/port rewrite, no L2 substitution, no TTL edit - the
		 * CPU receives the frame byte-identical to the punt path it
		 * already takes today. */
		act->mcgid = CN_L3E_CPU0_MCGID;
		act->deepq = 0;
		act->ip_addr_vld = 0;
		act->ip_addr = 0;
		act->ip_type = 0;
		act->l4_port = 0;
		act->ip_ttl_dec = 0;
		act->ip_ttl_zero_drop = 0;
		act->l3_if_vld = 0;
		act->egr_l3_if_idx = 0;
		act->mac_da_idx_vld = 0;
		act->mac_da_idx = 0;
		break;
	default:
		break;
	}
}


/* Liveness sweep: every CN_L3E_SWEEP_MS walk ONLY the ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 61. */
#define CN_L3E_SWEEP_MS		5000

static void cn_l3e_sweep_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(cn_l3e_sweep, cn_l3e_sweep_work);

static void cn_l3e_sweep_work(struct work_struct *work)
{
	struct cn_l3e *l3e = cn_l3e;
	unsigned long traffic;
	u32 bucket, trf;
	int slot;

	if (!l3e)
		return;

	mutex_lock(&cn_flow_offload_mutex);
	for (bucket = 0; bucket < CN_L3E_AGE_ROWS; bucket++) {
		if (!l3e->bucket_occ[bucket])
			continue;
		if (cn_l3e_bucket_sweep(l3e, bucket, &trf))
			continue;	/* bounded timeout: retry next sweep */

		/* each re-armed slot this sweep = one flow the HW T2 forwarded
		 * since the last sweep -> the cumulative hw_hit witness. */
		if (trf)
			atomic_add(hweight32(trf), &cn_l3e_hw_hits);

		traffic = trf;
		for_each_set_bit(slot, &traffic, CN_L3E_AGE_SLOTS) {
			struct cn_flow_priv *e =
				l3e->entry_by_idx[bucket * CN_L3E_AGE_SLOTS +
						  slot];

			/* attribute the re-arm to its LEG (us_hits/ds_hits) -
			 * an unowned slot is a manual /proc flow, counted
			 * apart rather than blamed on either direction */
			if (!e) {
				atomic_inc(&cn_l3e_hits_unattr);
				continue;
			}
			e->last_hit = jiffies;
			e->hits++;
			atomic_inc(e->ds ? &cn_l3e_ds_hits : &cn_l3e_us_hits);
			if (e->pppoe)
				atomic_inc(e->ds ? &cn_pppoe_ds_hits :
						   &cn_pppoe_us_hits);
		}
		if (!(bucket & 0x3f))
			cond_resched();
	}
	mutex_unlock(&cn_flow_offload_mutex);

	schedule_delayed_work(&cn_l3e_sweep, msecs_to_jiffies(CN_L3E_SWEEP_MS));
}

/* Names every cn_flow_install refusal branch so a rejected/silently-erroring
 * REPLACE localises itself.  At pr_debug (dynamic-debug): first-class dump/spy
 * per project policy, but silent at the default loglevel so the shipped tree
 * is not info-spammy under a many-flow load. */
#define cn_rep_dbg(fmt, ...) \
	pr_debug("cn_flow_install: " fmt, ##__VA_ARGS__)

/* ★ THE REFUSAL LEDGER (`refused:`). Until this existed, a ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 62. */
static atomic_t cn_flow_refused = ATOMIC_INIT(0);
static atomic_t cn_flow_refused_unsupp = ATOMIC_INIT(0);
static atomic_t cn_flow_refused_full = ATOMIC_INIT(0);
static atomic_t cn_flow_refused_dup = ATOMIC_INIT(0);
static atomic_t cn_flow_refused_err = ATOMIC_INIT(0);
static atomic_t cn_flow_refused_last = ATOMIC_INIT(0);

static void cn_flow_refused_account(int err)
{
	if (!err)
		return;
	atomic_inc(&cn_flow_refused);
	atomic_set(&cn_flow_refused_last, err);
	switch (err) {
	case -EOPNOTSUPP:
		atomic_inc(&cn_flow_refused_unsupp);
		break;
	case -ENOSPC:
		atomic_inc(&cn_flow_refused_full);
		break;
	case -EEXIST:
		atomic_inc(&cn_flow_refused_dup);
		break;
	default:
		atomic_inc(&cn_flow_refused_err);
		break;
	}
}

/* ★★ A VLAN-CARRYING WAN, AND WHY THE TAG RIDES THE PER-FLOW ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 63. */
#define CN_VLAN_WAN_DIRECT	1	/* the WAN netdev IS an 802.1Q upper   */
#define CN_VLAN_WAN_UNDER	2	/* an 802.1Q layer UNDER an encap      */
#define CN_VLAN_WAN_ACTION	3	/* the rule carried VLAN_PUSH/POP      */

static atomic_t cn_vlan_wan_refused_us = ATOMIC_INIT(0);
static atomic_t cn_vlan_wan_refused_ds = ATOMIC_INIT(0);
static atomic_t cn_vlan_wan_last_vid = ATOMIC_INIT(-1);
/* ★ WHY the cause is counted separately and not merely ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 138. */
static atomic_t cn_vlan_wan_direct = ATOMIC_INIT(0);
static atomic_t cn_vlan_wan_under = ATOMIC_INIT(0);
static atomic_t cn_vlan_wan_action = ATOMIC_INIT(0);

static void cn_vlan_wan_account(bool ds_leg, u16 vid, int how)
{
	atomic_inc(ds_leg ? &cn_vlan_wan_refused_ds : &cn_vlan_wan_refused_us);
	atomic_set(&cn_vlan_wan_last_vid, (int)vid);
	switch (how) {
	case CN_VLAN_WAN_DIRECT:
		atomic_inc(&cn_vlan_wan_direct);
		break;
	case CN_VLAN_WAN_UNDER:
		atomic_inc(&cn_vlan_wan_under);
		break;
	default:
		atomic_inc(&cn_vlan_wan_action);
		break;
	}
}

/* cn_wan_chain_vlan() - does the egress chain under @dev ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 64. */
struct cn_wan_encap {
	int	vid;
	__be16	vproto;
	int	sid;
	u8	ac_mac[ETH_ALEN];
	bool	ac_mac_vld;
	bool	walk_ok;
};

/* cn_wan_chain_encap() - the walk above, keeping everything ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 65. */
static void cn_wan_chain_encap(struct net_device *dev, struct cn_wan_encap *e)
{
	static const u8 zero_daddr[ETH_ALEN] = {};
	struct net_device_path_stack stack;
	int i;

	memset(e, 0, sizeof(*e));
	e->vid = -1;
	e->sid = -1;
	if (!dev)
		return;
	rcu_read_lock();
	if (dev_fill_forward_path(dev, zero_daddr, &stack) >= 0) {
		e->walk_ok = true;
		for (i = 0; i < stack.num_paths; i++) {
			const struct net_device_path *p = &stack.path[i];

			if (p->type == DEV_PATH_VLAN && e->vid < 0) {
				e->vid = p->encap.id;
				e->vproto = p->encap.proto;
			} else if (p->type == DEV_PATH_PPPOE && e->sid < 0) {
				e->sid = p->encap.id;
				ether_addr_copy(e->ac_mac, p->encap.h_dest);
				e->ac_mac_vld =
					is_valid_ether_addr(p->encap.h_dest);
			}
		}
	}
	rcu_read_unlock();
	return;
}

static int cn_wan_chain_vlan(struct net_device *dev)
{
	struct cn_wan_encap e;

	cn_wan_chain_encap(dev, &e);
	return e.vid;
}

/* DMA-AFT: the hardware WAN VLAN edit. Stock reaches ~953 ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 66. */
static bool hw_vlan_wan = true;
module_param(hw_vlan_wan, bool, 0644);
MODULE_PARM_DESC(hw_vlan_wan,
		 "hardware-forward an IPoE flow on a VLAN-tagged WAN, carrying the tag on the per-flow hit action (default on; off = the software-fastpath behaviour). PPPoE over a tagged WAN is refused either way.");

/* cn_aft_go() - run one indirect access and WAIT for it, ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 67. */
#define CN_AFT_GO_TRIES	1000u

static int cn_aft_go(struct cn_l3e *l3e, u32 access_off, u32 val)
{

	writel(val, l3e->dma_base + access_off);
	/* stock polls 200 times for the L2FIB and 100 for the map with no
	 * delay between reads; 1000 with a 1 us gap is far more headroom
	 * than either, and still bounded. */
	if (ca_go_spin(l3e->dma_base + access_off, CN_AFT_GO_TRIES,
		       ca_pause_udelay1) >= 0)
		return 0;
	l3e->aft_timeout++;
	dev_err(l3e->dev,
		"DMA-AFT: GO never cleared on access reg +0x%03x (wrote 0x%08x) - the VLAN edit is NOT programmed; this flow falls back to software\n",
		access_off, val);
	return -ETIMEDOUT;
}

/* cn_aft_tpid_slot() - which of the 4 TPID slots holds @tpid? ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 68. */
static int cn_aft_tpid_slot(struct cn_l3e *l3e, u16 tpid)
{
	u32 w[2];
	int i;

	w[0] = readl(l3e->dma_base + CA_DMA_AFT_TPID01);
	w[1] = readl(l3e->dma_base + CA_DMA_AFT_TPID23);
	i = cn_tpid_find(w, tpid);	/* slots packed 2/word - one fact in flowcore */
	if (i >= 0)
		return i;
	l3e->aft_no_tpid++;
	dev_warn(l3e->dev,
		 "DMA-AFT: TPID 0x%04x matches none of the 4 slots (0x%04x 0x%04x 0x%04x 0x%04x) - the hardware would silently drop the VLAN edit, so this flow stays on the software fastpath\n",
		 tpid, cn_tpid_slot_at(w, 0), cn_tpid_slot_at(w, 1),
		 cn_tpid_slot_at(w, 2), cn_tpid_slot_at(w, 3));
	return -ENOENT;
}

/* cn_l3fe_tpid_ensure() - make sure the WAN tag's TPID is ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 69. */
static int cn_l3fe_tpid_ensure(struct cn_l3e *l3e, u16 tpid)
{
	u32 w[2], ctrl, mask;
	int i, free_slot = -1;

	w[0] = readl(l3e->ne_base + CA_NI_L3FE_PP_TPID01);
	w[1] = readl(l3e->ne_base + CA_NI_L3FE_PP_TPID23);
	ctrl = readl(l3e->ne_base + CA_NI_L3FE_PP_TPID_CTRL);
	mask = FIELD_GET(CA_NI_L3FE_PP_TPID_TOP_MASK, ctrl);

	for (i = 0; i < CA_DMA_AFT_TPID_SLOTS; i++) {
		u16 slot = cn_tpid_slot_at(w, i);	/* slots packed 2/word */
		bool en = mask & BIT(i);

		if (slot == tpid && en)
			return i;			/* already satisfied */
		if (slot == tpid && !en) {	/* right value, gate shut */
			ctrl |= FIELD_PREP(CA_NI_L3FE_PP_TPID_TOP_MASK, BIT(i));
			writel(ctrl, l3e->ne_base + CA_NI_L3FE_PP_TPID_CTRL);
			l3e->aft_tpid_armed++;
			dev_info(l3e->dev,
				 "L3FE TPID gate: slot %d already held 0x%04x but was DISABLED - enabling it; action generation would have aborted for every tagged flow\n",
				 i, tpid);
			return i;
		}
		if (!en && free_slot < 0)
			free_slot = i;			/* claimable */
	}

	if (free_slot < 0) {
		l3e->aft_no_tpid++;
		dev_warn(l3e->dev,
			 "L3FE TPID gate: 0x%04x is not registered and all 4 slots are ENABLED with other values (%04x %04x %04x %04x, top_mask=0x%x) - refusing to repurpose one; tagged flows stay on the software fastpath\n",
			 tpid, cn_tpid_slot_at(w, 0), cn_tpid_slot_at(w, 1),
			 cn_tpid_slot_at(w, 2), cn_tpid_slot_at(w, 3), mask);
		return -ENOSPC;
	}

	/* claim the disabled slot: value first, then the enable bit, so the
	 * parser can never see the bit set against a stale value.  The half-word
	 * packing is the core's cn_tpid_slot_store() - the write half of the
	 * same ONE locate cn_tpid_slot_at() reads through. */
	w[free_slot >> 1] = cn_tpid_slot_store(w[free_slot >> 1], free_slot,
					       tpid);
	writel(w[free_slot >> 1], l3e->ne_base +
	       (free_slot < 2 ? CA_NI_L3FE_PP_TPID01 : CA_NI_L3FE_PP_TPID23));
	wmb();
	ctrl |= FIELD_PREP(CA_NI_L3FE_PP_TPID_TOP_MASK, BIT(free_slot));
	writel(ctrl, l3e->ne_base + CA_NI_L3FE_PP_TPID_CTRL);
	l3e->aft_tpid_armed++;
	dev_info(l3e->dev,
		 "L3FE TPID gate: registered 0x%04x in slot %d and enabled it (top_mask 0x%x -> 0x%x); without this, action generation aborts for every tagged flow\n",
		 tpid, free_slot, mask, (u32)(mask | BIT(free_slot)));
	return free_slot;
}

/* cn_aft_fib_program() - write one L2FIB entry: the actual ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 70. */
static int cn_aft_fib_program(struct cn_l3e *l3e, u8 idx, u16 vid,
			      u8 tag_cnt, int tpid_slot)
{
	u32 d0, d1, d2;

	/* ★ THE PACKING MOVED TO flowcore/cortina_vlan_install.c ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 139. */
	cortina_vlan_aft_words(vid, tag_cnt, tpid_slot, &d0, &d1, &d2);

	writel(d0, l3e->dma_base + CA_DMA_AFT_L2FIB_DATA0);
	writel(d1, l3e->dma_base + CA_DMA_AFT_L2FIB_DATA1);
	writel(d2, l3e->dma_base + CA_DMA_AFT_L2FIB_DATA2);
	return cn_aft_go(l3e, CA_DMA_AFT_L2FIB_ACCESS,
			 CA_DMA_AFT_ACCESS_GO | CA_DMA_AFT_ACCESS_WRITE |
			 FIELD_PREP(CA_DMA_AFT_ACCESS_IDX, idx));
}

/* cn_aft_fib_read() - read one L2FIB entry back OUT of the ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 71. */
static int __maybe_unused cn_aft_fib_read(struct cn_l3e *l3e, u8 idx,
					  u32 *d0, u32 *d1, u32 *d2)
{
	unsigned long flags;
	int err;

	spin_lock_irqsave(&l3e->aft_lock, flags);
	err = cn_aft_go(l3e, CA_DMA_AFT_L2FIB_ACCESS,
			CA_DMA_AFT_ACCESS_GO |	/* bit30 clear = READ */
			FIELD_PREP(CA_DMA_AFT_ACCESS_IDX, idx));
	if (!err) {
		*d0 = readl(l3e->dma_base + CA_DMA_AFT_L2FIB_DATA0);
		*d1 = readl(l3e->dma_base + CA_DMA_AFT_L2FIB_DATA1);
		*d2 = readl(l3e->dma_base + CA_DMA_AFT_L2FIB_DATA2);
	}
	spin_unlock_irqrestore(&l3e->aft_lock, flags);
	return err;
}

/* write one MAP entry: "frames from this lspid use fib @fib". */
static int cn_aft_map_program(struct cn_l3e *l3e, u8 idx, u8 lspid_map, u8 fib)
{
	u32 w = cortina_vlan_aft_map_word(lspid_map, fib);

	writel(w, l3e->dma_base + CA_DMA_AFT_MAP_DATA);
	return cn_aft_go(l3e, CA_DMA_AFT_MAP_ACCESS,
			 CA_DMA_AFT_ACCESS_GO | CA_DMA_AFT_ACCESS_WRITE |
			 FIELD_PREP(CA_DMA_AFT_ACCESS_IDX, idx));
}

/* clear a MAP entry (vld = 0, en = 0) so a freed fib stops being reachable */
static void cn_aft_map_clear(struct cn_l3e *l3e, u8 idx)
{
	writel(0, l3e->dma_base + CA_DMA_AFT_MAP_DATA);
	cn_aft_go(l3e, CA_DMA_AFT_MAP_ACCESS,
		  CA_DMA_AFT_ACCESS_GO | CA_DMA_AFT_ACCESS_WRITE |
		  FIELD_PREP(CA_DMA_AFT_ACCESS_IDX, idx));
}

/* cn_aft_install() - give this leg a hardware VLAN edit. ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 72. */
static int cn_aft_install(struct cn_l3e *l3e, struct cn_aft_ref *ref,
			  u16 vid, bool ds_leg)
{
	u8 tag_cnt = ds_leg ? 0 : 1;
	int tpid_slot = 0;
	int fib = -1, i, err;
	int map[2] = { -1, -1 };
	unsigned long flags;

	if (!l3e->dma_base)
		return -ENODEV;

	/* the TPID probe reads global state and must happen before we take
	 * anything; it is also the arm most likely to refuse. */
	if (tag_cnt) {
		/* TWO different TPID tables, both of which must accept the tag
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 140. */
		if (cn_l3fe_tpid_ensure(l3e, CA_DMA_AFT_TPID_8021Q) < 0)
			return -ENOSPC;
		tpid_slot = cn_aft_tpid_slot(l3e, CA_DMA_AFT_TPID_8021Q);
		if (tpid_slot < 0)
			return tpid_slot;
	}

	spin_lock_irqsave(&l3e->aft_lock, flags);

	/* reuse an identical edit if one is already programmed */
	for (i = CA_DMA_AFT_FIB_DYN_FIRST; i < CA_DMA_AFT_FIB_COUNT; i++) {
		if ((l3e->aft_fib_used & BIT_ULL(i)) &&
		    l3e->aft_fib_ref[i] &&
		    l3e->aft_fib_cnt[i] == tag_cnt &&
		    l3e->aft_fib_vid[i] == (tag_cnt ? vid : 0)) {
			l3e->aft_fib_ref[i]++;
			ref->fib = i;
			ref->valid = true;
			l3e->aft_reuse++;
			spin_unlock_irqrestore(&l3e->aft_lock, flags);
			return 0;
		}
	}

	for (i = CA_DMA_AFT_FIB_DYN_FIRST; i < CA_DMA_AFT_FIB_COUNT; i++) {
		if (!(l3e->aft_fib_used & BIT_ULL(i))) {
			fib = i;
			l3e->aft_fib_used |= BIT_ULL(i);
			break;
		}
	}
	if (fib < 0)
		goto full;

	for (i = CA_DMA_AFT_MAP_DYN_FIRST; i < CA_DMA_AFT_MAP_COUNT &&
	     (map[0] < 0 || map[1] < 0); i++) {
		if (l3e->aft_map_used & BIT_ULL(i))
			continue;
		l3e->aft_map_used |= BIT_ULL(i);
		if (map[0] < 0)
			map[0] = i;
		else
			map[1] = i;
	}
	if (map[1] < 0)
		goto full;

	l3e->aft_fib_vid[fib] = tag_cnt ? vid : 0;
	l3e->aft_fib_cnt[fib] = tag_cnt;
	l3e->aft_fib_ref[fib] = 1;
	l3e->aft_fib_map[fib][0] = map[0];
	l3e->aft_fib_map[fib][1] = map[1];
	spin_unlock_irqrestore(&l3e->aft_lock, flags);

	err = cn_aft_fib_program(l3e, fib, vid, tag_cnt, tpid_slot);
	if (err)
		goto unwind;
	/* one map entry per CPU lspid, as stock does (lspid_map 0 and 1 =
	 * AAL_LPORT_CPU_0 / _1) */
	for (i = 0; i < 2; i++) {
		err = cn_aft_map_program(l3e, map[i], i, fib);
		if (err)
			goto unwind;
	}

	ref->fib = fib;
	ref->valid = true;
	if (tag_cnt)
		l3e->aft_push++;
	else
		l3e->aft_strip++;
	dev_dbg(l3e->dev,
		"DMA-AFT: %s leg -> fib %d (%s vid %u, tpid slot %d), map %d/%d\n",
		ds_leg ? "DS" : "US", fib, tag_cnt ? "push" : "strip",
		tag_cnt ? vid : 0, tpid_slot, map[0], map[1]);
	return 0;

full:
	l3e->aft_full++;
	spin_unlock_irqrestore(&l3e->aft_lock, flags);
	dev_warn(l3e->dev,
		 "DMA-AFT: table full (fib %d, map %d/%d) - this flow stays on the software fastpath\n",
		 fib, map[0], map[1]);
	err = -ENOSPC;
	goto release;

unwind:
release:
	spin_lock_irqsave(&l3e->aft_lock, flags);
	if (fib >= 0) {
		l3e->aft_fib_used &= ~BIT_ULL(fib);
		l3e->aft_fib_ref[fib] = 0;
	}
	for (i = 0; i < 2; i++)
		if (map[i] >= 0)
			l3e->aft_map_used &= ~BIT_ULL(map[i]);
	spin_unlock_irqrestore(&l3e->aft_lock, flags);
	for (i = 0; i < 2; i++)
		if (map[i] >= 0)
			cn_aft_map_clear(l3e, map[i]);
	return err;
}

/* release this flow's share of the edit; clears the hardware only when the
 * last user goes away, so a second flow on the same VLAN keeps working. */
static void cn_aft_release(struct cn_l3e *l3e, struct cn_aft_ref *ref)
{
	unsigned long flags;
	u8 map[2] = { 0, 0 };
	bool last = false;
	int i;

	if (!ref->valid)
		return;
	ref->valid = false;

	spin_lock_irqsave(&l3e->aft_lock, flags);
	if (ref->fib < CA_DMA_AFT_FIB_COUNT && l3e->aft_fib_ref[ref->fib]) {
		if (!--l3e->aft_fib_ref[ref->fib]) {
			/* last user of this edit: the FIB owns its map pair, so
			 * it is freed here whichever flow released last */
			map[0] = l3e->aft_fib_map[ref->fib][0];
			map[1] = l3e->aft_fib_map[ref->fib][1];
			l3e->aft_fib_used &= ~BIT_ULL(ref->fib);
			l3e->aft_map_used &= ~BIT_ULL(map[0]);
			l3e->aft_map_used &= ~BIT_ULL(map[1]);
			last = true;
		}
	}
	spin_unlock_irqrestore(&l3e->aft_lock, flags);

	if (last)
		for (i = 0; i < 2; i++)
			cn_aft_map_clear(l3e, map[i]);
}

/* cn_aft_wan_vid() - the VLAN this leg's WAN side carries, 0 ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 73. */
static u16 cn_aft_wan_vid(struct net_device *wan_dev)
{
	int vid;

	if (!wan_dev)
		return 0;
	if (is_vlan_dev(wan_dev))
		return (u16)vlan_dev_vlan_id(wan_dev);
	vid = cn_wan_chain_vlan(wan_dev);
	return vid > 0 ? (u16)vid : 0;
}

/* ★★ HARDWARE-FORWARD A *PPPoE* FLOW WHOSE WAN RIDES AN ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 74. */
static bool hw_vlan_pppoe = true;
module_param(hw_vlan_pppoe, bool, 0644);
MODULE_PARM_DESC(hw_vlan_pppoe,
	"hardware-forward a PPPoE flow whose WAN rides an 802.1Q tag (pppoe-wan over gpon0.46): the tag goes on the per-flow hit action, the 8-byte session header on the egress L3-IF, the next-hop MAC comes from the PPPoE peer. Needs hw_vlan_wan=1 and hw_pppoe=1. Writable at runtime, so ONE boot yields both the control (0) and the treatment (1); 0 is byte-identical to the pre-2026-08-05 software-fastpath behaviour.");

/* The vlan+PPPoE ledger.  Every arm that can decline is counted, because a leg
 * that quietly fell back to software is indistinguishable from one that was
 * never offered - the same reason the vlan_wan cause breakdown exists. */
static atomic_t cn_vlan_pppoe_ok = ATOMIC_INIT(0);
static atomic_t cn_vlan_pppoe_no_sid = ATOMIC_INIT(0);
static atomic_t cn_vlan_pppoe_no_mac = ATOMIC_INIT(0);
static atomic_t cn_vlan_pppoe_badtpid = ATOMIC_INIT(0);
static atomic_t cn_vlan_pppoe_mismatch = ATOMIC_INIT(0);
/* ★★ THE AC-MAC SUBSTITUTION IS COUNTED PER LEG, AND THE DS ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 75. */
static atomic_t cn_vlan_pppoe_acmac_us = ATOMIC_INIT(0);
static atomic_t cn_vlan_pppoe_acmac_ds_blocked = ATOMIC_INIT(0);
static atomic_t cn_vlan_pppoe_readback = ATOMIC_INIT(0);

/* cn_wan_vlan_programmable() - may THIS leg's WAN tag go on ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 76. */
static bool cn_wan_vlan_programmable(struct net_device *wan_dev, u16 vid,
				     struct cn_wan_encap *enc)
{
	if (!hw_vlan_wan || !vid)
		return false;
	/* (1) IPoE on a DIRECT 802.1Q upper - the 2026-08-04 board-proven path
	 * (983.1/983.2 Mbps).  Deliberately NOT routed through the walk: it must
	 * stay bit-for-bit the behaviour that was certified. */
	if (is_vlan_dev(wan_dev)) {
		memset(enc, 0, sizeof(*enc));
		enc->vid = vid;
		enc->sid = -1;
		return true;
	}
	/* (2) a tag UNDER an encapsulation.  Only PPPoE is modelled.  The walk
	 * itself and the ledger stay here; the verdict + its DECLINE ORDER are
	 * the hoisted pure predicate (cn_wan_vlan_walk_verdict), so they cannot
	 * silently drift from cn_flow_refuse_vlan_wan()'s arms. */
	if (!hw_vlan_pppoe)
		return false;
	cn_wan_chain_encap(wan_dev, enc);
	switch (cn_wan_vlan_walk_verdict(vid, enc->walk_ok, enc->vid,
					 enc->vproto == htons(ETH_P_8021Q),
					 enc->sid, enc->ac_mac_vld)) {
	case CN_WAN_VLAN_WALK_MISMATCH:
		return false;
	case CN_WAN_VLAN_BAD_TPID:
		/* only 0x8100 is registered in the packet-editor's TPID slots;
		 * a QinQ outer would need a different slot and a different
		 * nesting, neither of which has been established here */
		atomic_inc(&cn_vlan_pppoe_badtpid);
		return false;
	case CN_WAN_VLAN_NO_SID:
		atomic_inc(&cn_vlan_pppoe_no_sid);
		return false;	/* a tag under something we have not RE'd */
	case CN_WAN_VLAN_NO_MAC:
		atomic_inc(&cn_vlan_pppoe_no_mac);
		return false;
	case CN_WAN_VLAN_OK_PPPOE:
		break;
	}
	atomic_inc(&cn_vlan_pppoe_ok);
	return true;
}

/* cn_flow_refuse_vlan_wan() - must this leg be refused for a ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 77. */
static bool cn_flow_refuse_vlan_wan(struct net_device *wan_dev, bool ds_leg)
{
	int vid;

	if (!wan_dev)
		return false;

	/* (1) the WAN netdev IS the 802.1Q upper - IPoE on gpon0.46 */
	if (is_vlan_dev(wan_dev)) {
		vid = vlan_dev_vlan_id(wan_dev);
		cn_vlan_wan_account(ds_leg, (u16)vid, CN_VLAN_WAN_DIRECT);
		cn_rep_dbg("refuse: %s leg - WAN netdev %s is a VLAN upper (vid %u); this hit-action pushes no tag, so the flow stays on the SW fastpath\n",
			   ds_leg ? "DS" : "US", wan_dev->name, vid);
		return true;
	}

	/* (2) an 802.1Q layer hidden UNDER an encapsulation - PPPoE on gpon0.46 */
	vid = cn_wan_chain_vlan(wan_dev);
	if (vid >= 0) {
		cn_vlan_wan_account(ds_leg, (u16)vid, CN_VLAN_WAN_UNDER);
		cn_rep_dbg("refuse: %s leg - WAN netdev %s rides an 802.1Q layer (vid %u) under its encapsulation; neither the tag nor a reliable session id survives to this rule, so the flow stays on the SW fastpath\n",
			   ds_leg ? "DS" : "US", wan_dev->name, vid);
		return true;
	}
	return false;
}

/* Is @addr_h (host order) owned by a PPP device?  The DS leg's destination is
 * the WAN address, so this names the WAN's L3 layer when no chain walk can. */
static bool cn_addr_on_ppp(struct net_device *dev, u32 addr_h)
{
	struct net_device *l3;
	bool ppp;

	if (!dev)
		return false;
	rcu_read_lock();
	l3 = __ip_dev_find(dev_net(dev), htonl(addr_h), false);
	ppp = l3 && l3->type == ARPHRD_PPP;
	rcu_read_unlock();
	return ppp;
}

/* The tag came as RULE data (ops->wan_vlan): the real WAN device is in the
 * flowtable, so no chain walk exists to resolve the PPPoE layer.  The US leg
 * reads it from its own PPPOE_PUSH, the DS leg from the armed session shadow --
 * the same witness an untagged PPPoE WAN's DS leg answers from.  A DS leg whose
 * WAN address sits on PPP while no US leg has armed the shadow yet is REFUSED
 * (nf re-offers it): taken as IPoE it would arm a DMA-AFT strip on the CPU
 * lspids that carry the session's own LCP (sec 97).  The two knobs select
 * exactly as they do on the device route. -> false = refused. */
static bool cn_flow_rule_tag(bool ds_leg, u16 vid, u16 rule_sid,
			     struct net_device *idev, u32 ds_da,
			     bool *vlan_pppoe, u16 *sid)
{
	u16 s = ds_leg ? READ_ONCE(cn_l3e->data_pppoe_session) : rule_sid;

	if (ds_leg && !s && cn_addr_on_ppp(idev, ds_da)) {
		atomic_inc(&cn_vlan_pppoe_no_sid);
		return false;
	}
	if (!hw_vlan_wan || (s && !hw_vlan_pppoe)) {
		cn_vlan_wan_account(ds_leg, vid, CN_VLAN_WAN_ACTION);
		return false;
	}
	if (s) {
		*vlan_pppoe = true;
		*sid = s;
		atomic_inc(&cn_vlan_pppoe_ok);
	}
	return true;
}

/* The ENGINE half of a TC flow install. The LIFECYCLE half -- ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 78. */
static int cn_flow_install(void *sh, const struct gpon_flow_key *k,
			   const struct gpon_flow_act *a,
			   const struct gpon_flow_ctx *ctx, void *priv,
			   u32 *idx_out)
{
	struct cn_flow_priv *entry = priv;
	struct cn_l3e_key key = {};
	/* ⚠ THIS FLAG WAS WRITTEN IN TWO PLACES AND READ IN NONE, so calling it
	 *   an admission gate was a claim about a variable nobody consulted. It
	 *   is one now: the teardown clears it before retiring the table, and an
	 *   install arriving after that is refused rather than racing the free. */
	if (!READ_ONCE(cn_flow_table_ready))
		return -ENODEV;
	struct cn_l3e_act act = {};
	struct net_device *odev = ctx->odev;
	bool ds_leg = ctx->ds_leg, vlan_wan = false;
	u16 vlan_wan_vid = 0;			/* 0 = untagged WAN = untouched */
	int tag = CN_WAN_TAG_NONE;		/* where the WAN tag comes from */
	int profile, err;
	u16 pppoe_sid = a->pppoe_sid;
	struct cn_wan_encap wenc = { .vid = -1, .sid = -1 };
	u16 vlan_wan_sid = 0;		/* PPPoE sid resolved from the WAN chain */
	bool vlan_pppoe = false;	/* this leg is PPPoE *and* tagged */
	u8 gw_dmac[6];			/* next-hop MAC; may be REPLACED below */
	bool got_dmac_lo = a->dmac_valid, got_dmac_hi = a->dmac_valid;

	if (!cn_l3e || !cn_l3e_install_ok)
		return -EOPNOTSUPP;

	/* ★ THE 5-TUPLE AND THE ACTION COME DECIDED ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 79. */
	key.ip_protocol = k->ip_protocol;
	key.ip_sa_0     = k->ip_sa;
	key.ip_da_0     = k->ip_da;
	key.l4_sport    = k->l4_sport;
	key.l4_dport    = k->l4_dport;
	key.ip_ver      = k->ip_ver;
	key.ip_vld      = 1;

	act.ip_addr_vld = a->nat_valid;
	act.ip_type     = a->nat_is_da;	/* 0 = rewrite SA, 1 = DA */
	act.ip_addr     = a->nat_addr;
	act.l4_port     = a->nat_port;
	ether_addr_copy(gw_dmac, a->gw_dmac);

	/* ★ The DS leg's WAN side is its INGRESS device, so this is ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 80. */
	if (ds_leg) {
		tag = cn_wan_tag_resolve(true, ctx->idev ?
					 cn_aft_wan_vid(ctx->idev) : 0,
					 a->vlan_vid, a->vlan_pop, &vlan_wan_vid);
		if (tag == CN_WAN_TAG_DEV &&
		    !cn_wan_vlan_programmable(ctx->idev, vlan_wan_vid, &wenc))
			vlan_wan = cn_flow_refuse_vlan_wan(ctx->idev, true);
		else if (tag == CN_WAN_TAG_RULE)
			vlan_wan = !cn_flow_rule_tag(true, vlan_wan_vid, 0,
						     ctx->idev, k->ip_da,
						     &vlan_pppoe, &vlan_wan_sid);
	}
	if (vlan_wan)
		return -EOPNOTSUPP;
	if (ds_leg) {
		if (!hw_ds_offload) {
			cn_rep_dbg("refuse: DS/reply leg, hw_ds_offload=0 (keeps the CPU punt path)\n");
			return -EOPNOTSUPP;
		}
		/* lazy one-time arm, so flipping the param at runtime cannot
		 * install a DS entry before its HW pieces exist */
		if (cn_l3e_arm_ds(cn_l3e)) {
			hw_ds_offload = false;
			pr_warn("cortina-l3fe: DS leg arm FAILED - hw_ds_offload forced OFF (US offload unaffected)\n");
			return -EOPNOTSUPP;
		}
	}
	profile = CN_L3E_PROFILE_ROUTED;

	/* ★ US leg ONLY: the WAN is this leg's EGRESS, i.e. the ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 81. */
	if (!ds_leg) {
		/* the US leg's WAN side is the REDIRECT device */
		tag = cn_wan_tag_resolve(false, cn_aft_wan_vid(odev),
					 a->vlan_vid, false, &vlan_wan_vid);
		if (tag < 0) {
			cn_vlan_wan_account(false, a->vlan_vid,
					    CN_VLAN_WAN_ACTION);
			cn_rep_dbg("refuse: US leg pushes vid %u, the WAN device carries %u\n",
				   a->vlan_vid, vlan_wan_vid);
			return -EOPNOTSUPP;
		}
		/* ★ SCOPED TO A *DIRECT* VLAN UPPER UNTIL 2026-08-05, and the ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 82. */
		if (tag == CN_WAN_TAG_DEV &&
		    !cn_wan_vlan_programmable(odev, vlan_wan_vid, &wenc))
			vlan_wan = cn_flow_refuse_vlan_wan(odev, false);
		else if (tag == CN_WAN_TAG_RULE)
			vlan_wan = !cn_flow_rule_tag(false, vlan_wan_vid,
						     pppoe_sid, NULL, 0,
						     &vlan_pppoe, &vlan_wan_sid);
	}
	if (!ds_leg && vlan_wan)
		return -EOPNOTSUPP;

	/* ★★ THE PLUMBING. The VLAN number was already reaching us - ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 83. */
	if (vlan_wan_vid && wenc.sid >= 0) {
		vlan_pppoe = true;
		vlan_wan_sid = (u16)wenc.sid;
		/* TWO ROUTES, AND A DIFFERENCE IS THE FINDING.  Where the rule
		 * DID carry a push, it must agree with the chain; a disagreement
		 * is a topology we have not modelled, so refuse rather than
		 * install on a guess. */
		if (pppoe_sid && pppoe_sid != vlan_wan_sid) {
			atomic_inc(&cn_vlan_pppoe_mismatch);
			cn_rep_dbg("refuse: %s leg - the rule says sid=%#x, the WAN chain says sid=%#x (vid %u)\n",
				   ds_leg ? "DS" : "US", pppoe_sid,
				   vlan_wan_sid, vlan_wan_vid);
			return -EOPNOTSUPP;
		}
		/* The US leg's encap is driven by pppoe_sid a few lines below ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 84. */
		if (!ds_leg && !pppoe_sid)
			pppoe_sid = vlan_wan_sid;
	}

	/* ★ PPPoE-WAN leg gate. ONE pure predicate owns the whole ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 85. */
	if (cn_pppoe_shadow_stale(ds_leg, cn_dev_is_lan_side(odev), pppoe_sid,
				  READ_ONCE(cn_l3e->data_pppoe_session))) {
		pr_info("cortina-l3fe: WAN egress %s offers no PPPoE session while %#x is armed - the session is gone, disarming (a stale shadow refuses every upstream flow)\n",
			netdev_name(odev),
			READ_ONCE(cn_l3e->data_pppoe_session));
		cortina_ni_wan_pppoe_session_set(0);
	}
	/* ★ On a tagged PPPoE WAN the SHADOW is not armed - nothing ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 86. */
	switch (cn_pppoe_leg_check(hw_pppoe, ds_leg, pppoe_sid,
				   vlan_pppoe ? vlan_wan_sid :
					READ_ONCE(cn_l3e->data_pppoe_session))) {
	case CN_PPPOE_LEG_OK:
		break;
	case CN_PPPOE_LEG_MODE_OFF:
		atomic_inc(ds_leg ? &cn_pppoe_ds_refused : &cn_pppoe_us_refused);
		cn_rep_dbg("refuse: PPPoE-WAN flow on the %s leg (hw_pppoe=0; stays on the SW fastpath; sid=%#x)\n",
			   ds_leg ? "DS" : "US", pppoe_sid);
		return -EOPNOTSUPP;
	case CN_PPPOE_LEG_NO_PUSH:
		atomic_inc(&cn_pppoe_us_refused);
		cn_rep_dbg("refuse: session %#x armed but this US rule has no PPPOE_PUSH - cannot express it\n",
			   READ_ONCE(cn_l3e->data_pppoe_session));
		return -EOPNOTSUPP;
	case CN_PPPOE_LEG_UNEXPECTED_PUSH:
		cn_rep_dbg("refuse: unexpected PPPoE push on the DS leg (sid=%#x)\n",
			   pppoe_sid);
		return -EOPNOTSUPP;
	}

	/* US hit-action - GROUP_18 WAN-forward via the live PON data ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 87. */
	if (!ds_leg) {
		err = cn_l3e_set_us_egress(cn_l3e, &act, pppoe_sid);
		if (err) {
			cn_rep_dbg("refuse: no PON data path armed (set_us_egress %d)\n",
				   err);
			return -EOPNOTSUPP;
		}
		/* ★ The WAN's 802.1Q tag goes ON THIS ACTION - see ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 88. */
		err = cn_l3e_set_us_wan_vlan(cn_l3e, &act, vlan_wan_vid);
		if (err) {
			cn_rep_dbg("refuse: WAN VLAN %u not programmable into the action (%d)\n",
				   vlan_wan_vid, err);
			return -EOPNOTSUPP;
		}
	}
	act.ip_ttl_dec = 1;
	if (ds_leg) {
		/* The vendor sets ip_ttl_dec AND the TTL-zero discard ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 89. */
		act.ip_ttl_zero_drop = 1;
	}

	/* ★ A2 next-hop L2 rewrite (aal-77c, stock-mechanism): ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 90. */
	if (!got_dmac_lo || !got_dmac_hi) {
		cn_rep_dbg("refuse: no ETH-mangle next-hop DMAC (keeps SW path)\n");
		return -EOPNOTSUPP;
	}
	/* ★ SUSPECTED (kernel source read, ONE tier - so it is GATED, ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 91. */
	if (vlan_pppoe && wenc.ac_mac_vld && is_zero_ether_addr(gw_dmac)) {
		if (ds_leg) {
			/* counted, refused, and LOUD: reaching here at all means the ...
			 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 141. */
			atomic_inc(&cn_vlan_pppoe_acmac_ds_blocked);
			pr_warn_ratelimited("cortina-l3fe: DS leg offered a ZERO next-hop MAC on a tagged PPPoE WAN; the PPPoE peer is an UPSTREAM address and is NOT substituted downstream - this flow stays on the SW fastpath\n");
		} else {
			ether_addr_copy(gw_dmac, wenc.ac_mac);
			atomic_inc(&cn_vlan_pppoe_acmac_us);
			cn_rep_dbg("US leg: the ETH mangle carried a zero next hop; using the PPPoE peer %pM\n",
				   gw_dmac);
		}
	}
	/* ★ UNCONDITIONAL, and it is fail-closed rather than ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 142. */
	if (!is_valid_ether_addr(gw_dmac)) {
		cn_rep_dbg("refuse: %s leg next-hop DMAC %pM is not unicast (keeps SW path)\n",
			   ds_leg ? "DS" : "US", gw_dmac);
		return -EOPNOTSUPP;
	}
	if (!ds_leg) {
		int lut = cortina_ni_l2fe_fdb_add_idx(cn_l3e->ne_base, gw_dmac,
						      CA_NI_RX_L3WAN_LDPID);

		if (lut < 0) {
			cn_rep_dbg("refuse: L2-FDB next-hop add failed (keeps SW path)\n");
			return -EOPNOTSUPP;
		}
		act.mac_da_idx = lut;		/* egr_lutidx = the FDB entry index */
		act.mac_da_idx_vld = 1;
		cn_rep_dbg("A2 next-hop DMAC %pM -> L2-FDB[%d] (mac_da_idx=egr_lutidx), egress SMAC via L3-IF[%u]\n",
			   gw_dmac, lut, CN_L3E_IPOE_L3IF_IDX);
	} else {
		/* ★ DS next hop + LAN egress port, both from the ONE L2FE FDB ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 92. */
		u32 lan_ldpid = 0;
		int lut = cortina_ni_l2fe_fdb_lookup_idx(cn_l3e->ne_base,
							gw_dmac, &lan_ldpid);

		if (lut < 0) {
			cn_rep_dbg("refuse: DS next-hop %pM not in the L2-FDB (keeps SW path)\n",
				   gw_dmac);
			return -EOPNOTSUPP;
		}
		if (hw_ds_lan_ldpid >= 0)
			lan_ldpid = hw_ds_lan_ldpid;
		atomic_set(&cn_ds_last_ldpid, (int)lan_ldpid);
		if (lan_ldpid > CN_L3E_LAN_PORT_LDPID_MAX) {
			cn_rep_dbg("refuse: DS next-hop %pM FDB ldpid=0x%02x not a LAN NI port 0..%u (keeps SW path; force with hw_ds_lan_ldpid=)\n",
				   gw_dmac, lan_ldpid,
				   CN_L3E_LAN_PORT_LDPID_MAX);
			return -EOPNOTSUPP;
		}
		cn_l3e_set_ds_egress(&act, lan_ldpid);
		/* ★ The DS leg must POP the WAN tag, and "leave the block at ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 93. */
		cn_l3e_set_ds_wan_vlan(&act, tag != CN_WAN_TAG_NONE);
		act.mac_da_idx = lut;
		act.mac_da_idx_vld = 1;
		cn_rep_dbg("DS next-hop DMAC %pM -> L2-FDB[%d] ldpid=%u mcgid=0x%03x, egress SMAC via L3-IF[%u]\n",
			   gw_dmac, lut, lan_ldpid,
			   CN_L3E_LAN_EGR_MCGID(lan_ldpid),
			   CN_L3E_LAN_L3IF_IDX);
		/* stage discriminator LAST, so it overrides every field above */
		cn_l3e_ds_probe_apply(&act);
		if (hw_ds_probe)
			pr_info("cortina-l3fe: DS entry installed in PROBE mode %d (%s) - throughput is expected to stay at the CPU-punt baseline; watch ds_hits in /proc/cortina_l3fe\n",
				hw_ds_probe,
				hw_ds_probe == 1 ? "match-only, mrr_vld=0" :
						   "CPU_0 punt hit-action");
	}

	/* `entry` is the core's per-flow private area, already allocated and
	 * zeroed and freed with the entry; the cookie and the table are the
	 * core's business, not ours. */
	entry->last_hit = jiffies;
	entry->installed_at = jiffies;
	entry->ds = ds_leg;
	/* ★ "this entry belongs to a PPPoE-WAN flow", for the pppoe_* ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 94. */
	entry->pppoe = pppoe_sid || vlan_pppoe ||
		       (ds_leg && READ_ONCE(cn_l3e->data_pppoe_session));
	entry->probe = ds_leg ? hw_ds_probe : 0;

	err = cn_l3e_flow_add(cn_l3e, &key, &act, profile, CN_L3E_WAN_MASK_ID,
			      &entry->hash_idx, &entry->crc16);
	if (err) {
		/* ★ NEVER un-ratelimited here. nf_flow_table RE-OFFERS a ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 95. */
		if (err == -ENOSPC || err == -EEXIST)
			cn_rep_dbg("%s install refused (%d) - flow stays on the SW fastpath\n",
				   ds_leg ? "DS" : "US", err);
		else
			pr_err_ratelimited("cn_flow_install: %s install FAILED (%d) %pI4h:%u->%pI4h:%u proto=%u pppoe=%#x\n",
					   ds_leg ? "DS" : "US",
					   err, &(u32){ key.ip_sa_0 },
					   (u16)key.l4_sport,
					   &(u32){ key.ip_da_0 },
					   (u16)key.l4_dport,
					   (u8)key.ip_protocol, pppoe_sid);
		goto free;
	}

	/* ★★ READ THE ENTRY BACK OUT OF THE TABLE, BY LITERAL BIT ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 96. */
	if (tag != CN_WAN_TAG_NONE) {
		const void *raw = cn_l3e->fib_tbl +
				  (size_t)entry->hash_idx * CN_L3E_FIB_BYTES;
		u64 rb_vid = cn_fib_field(raw, 145, 12);
		u64 rb_tpid = cn_fib_field(raw, 157, 3);
		u64 rb_cnt = cn_fib_field(raw, 160, 2);
		u64 rb_vld = cn_fib_field(raw, 162, 1);

		if (rb_vld != 1 ||
		    rb_vid != (u64)(ds_leg ? 0 : vlan_wan_vid) ||
		    rb_cnt != (u64)(ds_leg ? 0 : 1) ||
		    (!ds_leg && rb_tpid == 0)) {
			atomic_inc(&cn_vlan_pppoe_readback);
			pr_err_ratelimited("cortina-l3fe: %s idx=%u VLAN READBACK MISMATCH: asked vid=%u, entry holds vid=%llu cnt=%llu vld=%llu tpid_enc=%llu - entry REMOVED, flow stays on the SW fastpath\n",
					   ds_leg ? "DS" : "US", entry->hash_idx,
					   vlan_wan_vid, rb_vid, rb_cnt, rb_vld,
					   rb_tpid);
			cn_l3e_flow_del(cn_l3e, entry->hash_idx, entry->crc16);
			err = -EOPNOTSUPP;
			goto free;
		}
	}

	/* The hardware WAN VLAN edit, last: the flow is in HW and ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 143. */
	if (tag != CN_WAN_TAG_NONE && !vlan_pppoe) {
		/* ★★ A PPPoE tagged flow is kept OUT of the DMA-AFT. THE ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 97. */
		err = cn_aft_install(cn_l3e, &entry->aft, vlan_wan_vid, ds_leg);
		if (err) {
			/* every arm already logged loudly and bumped its own
			 * counter; keep the refusal ledger consistent so
			 * /proc still reports this flow as VLAN-refused */
			cn_vlan_wan_account(ds_leg, vlan_wan_vid,
					    CN_VLAN_WAN_DIRECT);
			cn_l3e_flow_del(cn_l3e, entry->hash_idx, entry->crc16);
			err = -EOPNOTSUPP;
			goto free;
		}
	}

	/* register with the liveness sweep (under cn_flow_offload_mutex) */
	cn_l3e->entry_by_idx[entry->hash_idx] = entry;
	cn_l3e->bucket_occ[entry->hash_idx / CN_L3E_AGE_SLOTS]++;
	atomic_inc(&cn_flow_installed);
	if (ds_leg)
		atomic_inc(&cn_ds_installed);
	if (entry->pppoe)
		atomic_inc(&cn_pppoe_installed);
	/* per-flow install witness; pr_debug so a 1000-flow soak stays quiet.
	 * "nat=" is the rewritten end named by ip_type: SA on the US leg, DA on
	 * the DS leg. */
	pr_debug("cn_flow_install: %s INSTALLED idx=%u crc16=%04x %pI4h:%u->%pI4h:%u proto=%u pppoe=%#x nat[%s]=%pI4h:%u mcgid=0x%03x\n",
		 ds_leg ? "DS" : "US", entry->hash_idx, entry->crc16,
		 &(u32){ key.ip_sa_0 }, (u16)key.l4_sport,
		 &(u32){ key.ip_da_0 }, (u16)key.l4_dport,
		 (u8)key.ip_protocol, pppoe_sid,
		 act.ip_type ? "DA" : "SA",
		 &(u32){ act.ip_addr }, (u16)act.l4_port, (u32)act.mcgid);
	*idx_out = entry->hash_idx;
	return 0;
free:
	/* Reached only from the install steps above.  The core owns the entry
	 * memory and the cookie table, so there is nothing to free here -- the
	 * hardware undo that used to live at the rhashtable-failure label moved
	 * into cn_flow_remove(), which the core calls on ITS unwind. */
	return err;
}

/* Tear this flow out of the hardware. The core has already ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 98. */
static int cn_flow_remove(void *sh, u32 idx, void *priv)
{
	struct cn_flow_priv *entry = priv;

	cn_l3e->entry_by_idx[idx] = NULL;
	cn_l3e->bucket_occ[idx / CN_L3E_AGE_SLOTS]--;
	cn_l3e_flow_del(cn_l3e, idx, entry->crc16);
	cn_aft_release(cn_l3e, &entry->aft);
	if (entry->ds)
		atomic_dec(&cn_ds_installed);
	cn_pppoe_entry_gone(entry, true);
	atomic_dec(&cn_flow_installed);
	pr_debug("cn_flow_remove: removed idx=%u (flows=%d ds=%d)\n",
		 idx, atomic_read(&cn_flow_installed),
		 atomic_read(&cn_ds_installed));
	return 0;
}

static int cn_l3e_flush_auto_flows(struct cn_l3e *l3e)
{
	/* ★ BUG-B: tear down EVERY installed offloaded flow. Called ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 99. */
	return gpon_flow_offload_flush(cn_fo);
}

static int cn_flow_stats_op(void *sh, u32 idx, void *priv,
			    unsigned long *lastused)
{
	struct cn_flow_priv *entry = priv;

	/* No per-flow byte/pkt counters in the engine (AQM MIB meters only
	 * 2048 flows); report LIVENESS, fed by the batch bucket sweep -
	 * zero MMIO here, so 10k+ concurrent STATS queries stay free. */
	*lastused = entry->last_hit;
	return 0;
}

static bool cn_flow_is_lan_side(void *sh, struct net_device *dev)
{
	return cn_dev_is_lan_side(dev);
}

/* A rule carrying a VLAN push or pop is refused by the CORE -- no hit-action
 * here can express a tag.  This keeps the refusal ATTRIBUTED in /proc rather
 * than counted as one of N anonymous unsupported reasons, which is the
 * distinction whose absence made that defect unreadable twice. */
static void cn_flow_note_vlan_action(void *sh, bool ds_leg, u16 vid)
{
	cn_vlan_wan_account(ds_leg, vid, CN_VLAN_WAN_ACTION);
}

static const struct gpon_flow_ops cn_flow_ops = {
	.is_lan_side		= cn_flow_is_lan_side,
	.install		= cn_flow_install,
	.remove			= cn_flow_remove,
	.stats			= cn_flow_stats_op,
	.note_vlan_action	= cn_flow_note_vlan_action,
	.wan_vlan		= true,
	.priv_size		= sizeof(struct cn_flow_priv),
};


static int cn_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				void *cb_priv)
{
	struct flow_cls_offload *f = type_data;
	int err;

	if (type != TC_SETUP_CLSFLOWER)
		return -EOPNOTSUPP;

	mutex_lock(&cn_flow_offload_mutex);
	switch (f->command) {
	case FLOW_CLS_REPLACE:
		err = gpon_flow_offload_replace(cn_fo, f, cb_priv);
		/* every REPLACE outcome passes here: a refusal is counted, never
		 * silent (see the refusal-ledger comment above cn_flow_install) */
		cn_flow_refused_account(err);
		break;
	case FLOW_CLS_DESTROY:
		err = gpon_flow_offload_destroy(cn_fo, f);
		break;
	case FLOW_CLS_STATS:
		err = gpon_flow_offload_stats(cn_fo, f);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}
	mutex_unlock(&cn_flow_offload_mutex);
	return err;
}

static LIST_HEAD(cn_block_cb_list);

static int cn_setup_tc_block(struct net_device *dev,
			     struct flow_block_offload *f)
{
	return gpon_flow_block_setup(dev, f, &cn_block_cb_list,
				     cn_setup_tc_block_cb, NULL);
}

/* ndo_setup_tc hook for the cortina-ni netdevs (eth0 / gpon0) */
int cortina_ni_setup_tc(struct net_device *dev, enum tc_setup_type type,
			void *type_data)
{
	switch (type) {
	case TC_SETUP_BLOCK:
	case TC_SETUP_FT:
		return cn_setup_tc_block(dev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL_GPL(cortina_ni_setup_tc);

/* engine bring-up - called from the cortina-ni probe (wiring ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 144. */

static void cn_l3e_free_shadow(struct cn_l3e *l3e)
{
	kvfree(l3e->shadow_crc32);
	kvfree(l3e->shadow_crc16);
	kvfree(l3e->entry_by_idx);
	kvfree(l3e->bucket_occ);
	l3e->shadow_crc32 = NULL;
	l3e->shadow_crc16 = NULL;
	l3e->entry_by_idx = NULL;
	l3e->bucket_occ = NULL;
}

/* ★★ The four invariants the whole "one profile fits both ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 100. */
static int cn_l3e_verify_profile_invariants(struct cn_l3e *l3e)
{
	/* a representative routed 5-tuple - values are arbitrary but non-zero
	 * and distinct so the CRC cannot be degenerate */
	struct cn_l3e_key probe = {
		.ip_protocol	= IPPROTO_TCP,
		.ip_sa_0	= 0xc0a80102,	/* 192.168.1.2 */
		.ip_da_0	= 0x08080808,	/* 8.8.8.8 */
		.l4_sport	= 0x1234,
		.l4_dport	= 0x0050,
		.ip_ver		= 0,
		.ip_vld		= 1,
	};
	struct cn_l3e_key port_probe;
	u32 crc32_p0 = 0, crc32_pr = 0, def0, def1, bad_off = 0, bad_val = 0;
	u32 crc32_dp = 0, crc32_sp = 0;
	u16 crc16_p0 = 0, crc16_pr = 0, crc16_dp = 0, crc16_sp = 0;
	int p, w, ret, dret, fail = 0;

	/* (A) profile stamp must be invisible to the hash */
	ret = cn_l3e_key_hash(l3e, &probe, 0, CN_L3E_WAN_MASK_ID,
			      &crc32_p0, &crc16_p0);
	if (!ret)
		ret = cn_l3e_key_hash(l3e, &probe, CN_L3E_PROFILE_ROUTED,
				      CN_L3E_WAN_MASK_ID, &crc32_pr, &crc16_pr);
	if (ret) {
		dev_warn(l3e->dev,
			 "l3fe: profile-invariant (A) UNVERIFIED - SWO hash timeout (%d)\n",
			 ret);
		fail = 1;
	} else if (crc32_p0 != crc32_pr || crc16_p0 != crc16_pr) {
		dev_warn(l3e->dev,
			 "l3fe: profile-invariant (A) BROKEN - mask %d does NOT exclude the profile stamp (prof0 %08x/%04x != prof%d %08x/%04x); a cross-profile lookup can never match\n",
			 CN_L3E_WAN_MASK_ID, crc32_p0, crc16_p0,
			 CN_L3E_PROFILE_ROUTED, crc32_pr, crc16_pr);
		fail = 1;
	}

	/* (B) per-profile key-selection / tuple-rotate config must be all zero */
	for (p = 0; p < CN_L3E_PF_KEY_PROFILES; p++)
		for (w = 0; w < 5; w++) {
			u32 off = CN_L3E_HS_PF_KEY(p) + w * 4;
			u32 v = readl(l3e->ne_base + off);

			if (v) {
				bad_off = off;
				bad_val = v;
			}
		}
	if (bad_off) {
		dev_warn(l3e->dev,
			 "l3fe: profile-invariant (B) BROKEN - per-profile hash key/tuple config non-zero at 0x%04x = 0x%08x; profiles would hash differently\n",
			 bad_off, bad_val);
		fail = 1;
	}

	/* (C) the routed profile's miss default must match profile 0's (= punt) */
	def0 = readl(l3e->ne_base + CN_L3E_HS_DEFAULT_ACTION(0));
	def1 = readl(l3e->ne_base + CN_L3E_HS_DEFAULT_ACTION(1));
	if (def0 != def1) {
		dev_warn(l3e->dev,
			 "l3fe: profile-invariant (C) BROKEN - HS_DEFAULT_ACTION[1]=0x%08x != [0]=0x%08x; a T2 miss under the routed profile may DROP instead of punt\n",
			 def1, def0);
		fail = 1;
	}

	/* (D) the exact L4 ports must participate in the hash tuple.  Only
	 * meaningful if (A) got a baseline CRC out of the engine at all. */
	dret = ret;
	if (!dret) {
		port_probe = probe;
		port_probe.l4_dport = probe.l4_dport ^ 0x0ff0;
		dret = cn_l3e_key_hash(l3e, &port_probe, CN_L3E_PROFILE_ROUTED,
				       CN_L3E_WAN_MASK_ID, &crc32_dp, &crc16_dp);
	}
	if (!dret) {
		port_probe = probe;
		port_probe.l4_sport = probe.l4_sport ^ 0x0ff0;
		dret = cn_l3e_key_hash(l3e, &port_probe, CN_L3E_PROFILE_ROUTED,
				       CN_L3E_WAN_MASK_ID, &crc32_sp, &crc16_sp);
	}
	if (dret) {
		if (!ret)		/* (A) already reported an engine timeout */
			dev_warn(l3e->dev,
				 "l3fe: profile-invariant (D) UNVERIFIED - SWO hash timeout (%d)\n",
				 dret);
		fail = 1;
	} else if (crc32_dp == crc32_pr || crc32_sp == crc32_pr) {
		dev_warn(l3e->dev,
			 "l3fe: profile-invariant (D) BROKEN - mask %d does not use the EXACT L4 ports (base %08x, dport-perturbed %08x, sport-perturbed %08x); the mask's 17-bit port fields are in RANGE mode, so port-only-different flows would alias onto one entry - clear bit16 of both fields in l3fe_mask_lo[%d]\n",
			 CN_L3E_WAN_MASK_ID, crc32_pr, crc32_dp, crc32_sp,
			 CN_L3E_WAN_MASK_ID);
		fail = 1;
	}

	if (fail)
		return -EINVAL;
	dev_info(l3e->dev,
		 "l3fe: profile invariants OK (A: mask %d excludes the profile stamp, crc %08x/%04x either way; B: per-profile hash cfg all zero; C: HS_DEFAULT_ACTION[0]==[1]=0x%08x; D: exact L4 ports in the tuple, dport %08x/%04x sport %08x/%04x != base)\n",
		 CN_L3E_WAN_MASK_ID, crc32_p0, crc16_p0, def0,
		 crc32_dp, crc16_dp, crc32_sp, crc16_sp);
	return 0;
}

static int cn_l3e_init(struct cn_l3e *l3e)
{
	struct cn_l3e_tables t = {
		.key_virt	= l3e->key_tbl,
		.key_pa		= l3e->key_tbl_pa,
		.fib_virt	= l3e->fib_tbl,
		.fib_pa		= l3e->fib_tbl_pa,
	};
	int ret;

	cn_l3e_install_ok = false;

	/* lean SW shadow + sweep reverse map (~0.9 MB total) */
	l3e->shadow_crc32 = kvcalloc(CN_L3E_ENTRIES, sizeof(u32), GFP_KERNEL);
	l3e->shadow_crc16 = kvcalloc(CN_L3E_ENTRIES, sizeof(u16), GFP_KERNEL);
	l3e->entry_by_idx = kvcalloc(CN_L3E_ENTRIES,
				     sizeof(struct cn_flow_priv *),
				     GFP_KERNEL);
	l3e->bucket_occ = kvcalloc(CN_L3E_AGE_ROWS, sizeof(u8), GFP_KERNEL);
	if (!l3e->shadow_crc32 || !l3e->shadow_crc16 || !l3e->entry_by_idx ||
	    !l3e->bucket_occ) {
		ret = -ENOMEM;
		goto free;
	}

	/* the ordered engine arm (MEM_INI self-zero -> carve zero -> base
	 * regs -> geometry -> anti-wedge patch -> punt defaults ->
	 * granularity 0), all stock-mirrored - cortina-l3fe.c */
	ret = cortina_l3fe_engine_init(l3e->ne_base, &t);
	if (ret)
		goto free;

	/* stock profile/tuple/mask classify config so the engine ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 145. */
	ret = cortina_l3fe_classify_setup(l3e->ne_base);
	if (ret) {
		dev_warn(l3e->dev,
			 "l3fe: classify_setup timed out (%d) - hash lookup not configured\n",
			 ret);
		goto free;
	}

	/* ★ Divergence B+C (gated OFF by default): steer routed ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 146. */
	if (hw_l3_fwd) {
		ret = cortina_l3fe_hw_l3_forward_enable(l3e->ne_base,
							l3e->router_mac_valid ?
							l3e->router_mac : NULL);
		dev_info(l3e->dev,
			 "l3fe: HW L3-forwarding %s (miss->CPU, mac-cam %s, 5-tuple mask %d)\n",
			 ret ? "enable FAILED" : "ENABLED",
			 l3e->router_mac_valid ? "committed" : "SKIPPED (no netdev MAC)",
			 CN_L3E_WAN_MASK_ID);
		/* A2: program the IPoE egress L3-IF entry (idx 2, an_sel 2 = our
		 * WAN MAC via the my-MAC CAM) once, so an offloaded IPoE flow gets
		 * its source MAC rewritten to the WAN MAC on egress.  Gate
		 * install_ok on it too - without it an offloaded flow blackholes. */
		if (!ret) {
			ret = cortina_l3fe_ipoe_l3if_set(l3e->ne_base,
							 CN_L3E_IPOE_L3IF_IDX,
							 CN_L3E_IPOE_AN_SEL);
			if (ret)
				dev_warn(l3e->dev,
					 "l3fe: IPoE egress L3-IF[%d] program failed (%d)\n",
					 CN_L3E_IPOE_L3IF_IDX, ret);
		}
		/* ★ DS (WAN->LAN) leg: arm its two HW pieces here when the ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 101. */
		if (!ret && cn_l3e_verify_profile_invariants(l3e) &&
		    hw_ds_offload) {
			hw_ds_offload = false;
			dev_warn(l3e->dev,
				 "l3fe: hw_ds_offload forced OFF - a profile invariant does not hold, so a DS entry could never match (US offload left as-is)\n");
		}
		if (!ret && hw_ds_offload && cn_l3e_arm_ds(l3e)) {
			hw_ds_offload = false;
			dev_warn(l3e->dev,
				 "l3fe: DS (WAN->LAN) leg setup FAILED - hw_ds_offload forced OFF, US offload unaffected\n");
		}
		/* P3: with the engine armed, the routed profiles pointed at the
		 * 5-tuple mask, and the CLS admission stamping t2_ctrl (on the
		 * link-up cls_init re-run), flows may now be installed for a HW
		 * hit.  Only ungate under the gate + a successful enable. */
		if (!ret)
			cn_l3e_install_ok = true;
	}

	cn_l3e = l3e;
	return 0;
free:
	cn_l3e_free_shadow(l3e);
	return ret;
}

/* Pull every installed flow out of the hardware and release ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 102. */
void cortina_ni_flowoffload_quiesce(void)
{
	/* ⚠ THREE PHASES, AND THE LOCK IS HELD FOR TWO OF THEM. ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 103. */
	mutex_lock(&cn_flow_offload_mutex);
	cn_flow_table_ready = false;		/* nothing new gets past here */
	mutex_unlock(&cn_flow_offload_mutex);

	cancel_delayed_work_sync(&cn_l3e_sweep);/* NOT under the lock it takes */

	mutex_lock(&cn_flow_offload_mutex);
	gpon_flow_offload_free(cn_fo);		/* entries dropped with MMIO  */
	cn_fo = NULL;
	/* ⚠ AND THE PUBLISHED GLOBAL IS INVALIDATED. cn_l3e outlived ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 104. */
	cn_l3e = NULL;
	mutex_unlock(&cn_flow_offload_mutex);
}

void cortina_ni_flowoffload_exit(void)
{
	/* Idempotent by construction: gpon_flow_offload_free() returns on NULL,
	 * so a module unload after a device teardown finds nothing to do -- and
	 * an unload with no teardown still retires everything. */
	cortina_ni_flowoffload_quiesce();
}

static int cn_flowoffload_init(void)
{
	int ret;

	cn_fo = gpon_flow_offload_new(&cn_flow_ops, NULL);
	ret = cn_fo ? 0 : -ENOMEM;
	cn_flow_table_ready = !ret;
	if (!ret)
		schedule_delayed_work(&cn_l3e_sweep,
				      msecs_to_jiffies(CN_L3E_SWEEP_MS));
	return ret;
}

/* HS_SWO HW-CRC selftest - the phase-1 gate proof that the ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 105. */
#define CN_L3E_SWO_BIT0		240	/* inside the selected key window */
#define CN_L3E_SWO_NBITS	8
/* the selftest needs an all-ones mask; use a spare mask-table index so it
 * never clobbers the real classify masks 0-7 (cortina_l3fe_classify_setup) */
#define CN_L3E_SELFTEST_MASK	63

static int cn_l3e_swo_key(struct cn_l3e *l3e, const u32 *w, u32 *c32, u16 *c16)
{
	return cortina_l3fe_swo_crc(l3e->ne_base, w, CN_L3E_KEY_BYTES / 4,
				    CN_L3E_SELFTEST_MASK, c32, c16);
}

static void cn_l3e_swo_selftest(struct cn_l3e *l3e)
{
	static const u32 ones[4] = { ~0u, ~0u, ~0u, ~0u };
	u32 w[CN_L3E_KEY_BYTES / 4];
	u8 *kb = (u8 *)w;
	u32 z32, r32, ab32, d32[CN_L3E_SWO_NBITS];
	u16 z16, r16, ab16, d16[CN_L3E_SWO_NBITS];
	bool ok = true;
	int i, bit, ret;

	l3e->selftest_ret = cortina_l3fe_mask_write(l3e->ne_base,
						    CN_L3E_SELFTEST_MASK,
						    ones, ones);
	if (l3e->selftest_ret) {
		pr_warn("cortina-l3fe: selftest mask write failed (%d)\n",
			l3e->selftest_ret);
		return;
	}

#define SWO_RUN(c32p, c16p) do {					\
	ret = cn_l3e_swo_key(l3e, w, (c32p), (c16p));			\
	if (ret) {							\
		l3e->selftest_ret = ret;				\
		pr_warn("cortina-l3fe: SWO engine timeout (%d)\n", ret); \
		return;							\
	}								\
} while (0)

	/* 1. determinism on the all-zero key */
	memset(w, 0, sizeof(w));
	SWO_RUN(&z32, &z16);
	memset(w, 0, sizeof(w));
	SWO_RUN(&r32, &r16);
	if (r32 != z32 || r16 != z16) {
		pr_warn("cortina-l3fe: SWO not deterministic: %08x/%04x vs %08x/%04x\n",
			z32, z16, r32, r16);
		ok = false;
	}

	/* single-bit deltas over consecutive window bits */
	for (i = 0; i < CN_L3E_SWO_NBITS; i++) {
		bit = CN_L3E_SWO_BIT0 + i;
		memset(w, 0, sizeof(w));
		kb[bit >> 3] = 1u << (bit & 7);
		SWO_RUN(&r32, &r16);
		d32[i] = r32 ^ z32;
		d16[i] = r16 ^ z16;
		/* 2. window live */
		if (!d32[i] || !d16[i]) {
			pr_warn("cortina-l3fe: SWO key bit %d has no effect\n",
				bit);
			ok = false;
		}
	}

	/* 3. linearity: crc(bit0 + bit1) == z ^ d0 ^ d1 */
	memset(w, 0, sizeof(w));
	kb[CN_L3E_SWO_BIT0 >> 3] = 3u << (CN_L3E_SWO_BIT0 & 7);
	SWO_RUN(&ab32, &ab16);
	if (ab32 != (z32 ^ d32[0] ^ d32[1]) ||
	    ab16 != (z16 ^ d16[0] ^ d16[1])) {
		pr_warn("cortina-l3fe: SWO linearity fail: %08x/%04x want %08x/%04x\n",
			ab32, ab16, z32 ^ d32[0] ^ d32[1],
			z16 ^ d16[0] ^ d16[1]);
		ok = false;
	}

	/* 4. the CRC polynomial algebra across adjacent bits */
	for (i = 0; i + 1 < CN_L3E_SWO_NBITS; i++) {
		if (d32[i + 1] != cn_l3e_poly32_step(d32[i])) {
			pr_warn("cortina-l3fe: SWO crc32 poly fail at bit %d: %08x -> %08x\n",
				CN_L3E_SWO_BIT0 + i, d32[i], d32[i + 1]);
			ok = false;
		}
		if (d16[i + 1] != cn_l3e_poly16_step(d16[i])) {
			pr_warn("cortina-l3fe: SWO crc16 poly fail at bit %d: %04x -> %04x\n",
				CN_L3E_SWO_BIT0 + i, d16[i], d16[i + 1]);
			ok = false;
		}
	}
#undef SWO_RUN

	if (ok)
		l3e->selftest_pass = 1;
	else
		l3e->selftest_fail = 1;
}

/* HDR_I 5-tuple key-packing liveness (divergence-A gate proof)
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 106. */
static void cn_l3e_hdri_live_test(struct cn_l3e *l3e)
{
	struct cn_l3e_key base = {
		.ip_vld = 1, .ip_ver = 0, .ip_protocol = 6,   /* TCP */
		.ip_sa_0 = 0x0a000001, .ip_da_0 = 0x2de14b02,
		.l4_sport = 12345, .l4_dport = 80,
	};
	struct cn_l3e_key k;
	u32 b32, r32;
	u16 b16, r16;
	bool ok = true;
	int ret, i;

	ret = cn_l3e_key_hash(l3e, &base, CN_L3E_PROFILE_WAN,
			      CN_L3E_WAN_MASK_ID, &b32, &b16);
	if (ret) {
		pr_warn("cortina-l3fe: HDR_I liveness: SWO timeout (%d)\n", ret);
		l3e->hdri_live_fail = 1;
		return;
	}

#define HDRI_PERTURB(desc, field, newval) do {				\
	k = base;							\
	k.field = (newval);						\
	ret = cn_l3e_key_hash(l3e, &k, CN_L3E_PROFILE_WAN,		\
			      CN_L3E_WAN_MASK_ID, &r32, &r16);		\
	if (ret) { l3e->hdri_live_fail = 1; return; }			\
	if (r32 == b32 && r16 == b16) {					\
		pr_warn("cortina-l3fe: HDR_I liveness: %s did NOT move the CRC (masked-out under mask %d: wrong CN_HDRI_* offset, or the mask does not keep the field)\n", \
			desc, CN_L3E_WAN_MASK_ID);			\
		ok = false;						\
	}								\
} while (0)

	HDRI_PERTURB("dport", l4_dport, 443);
	HDRI_PERTURB("sport", l4_sport, 22);
	HDRI_PERTURB("daddr", ip_da_0, 0x2de14b09);
	HDRI_PERTURB("saddr", ip_sa_0, 0x0a000063);
	HDRI_PERTURB("proto", ip_protocol, 17);
	HDRI_PERTURB("daddr-low-byte", ip_da_0, 0x2de14bff);
	HDRI_PERTURB("saddr-low-byte", ip_sa_0, 0x0a0000ff);
#undef HDRI_PERTURB

	/* determinism: same tuple twice -> identical CRC */
	for (i = 0; i < 2; i++) {
		ret = cn_l3e_key_hash(l3e, &base, CN_L3E_PROFILE_WAN,
				      CN_L3E_WAN_MASK_ID, &r32, &r16);
		if (ret || r32 != b32 || r16 != b16) {
			pr_warn("cortina-l3fe: HDR_I liveness: non-deterministic\n");
			ok = false;
		}
	}

	if (ok)
		l3e->hdri_live_pass = 1;
	else
		l3e->hdri_live_fail = 1;
}

/* debugfs .../cortina-l3fe - manual flow install/read/delete ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 107. */
#define CN_L3E_PROC_MAX_MANUAL	8
/* auto (nf_flow_table) entries printed in full per read.  Kept small so the
 * whole /proc output stays inside one seq_file page: seq_read re-runs show()
 * from scratch when the buffer overflows, and this read CONSUMES age re-arms
 * (read+clear), so a second pass would double-count them. */
#define CN_L3E_PROC_MAX_AUTO	8
struct cn_l3e_manual {
	u32	idx;
	u16	crc16;
	bool	valid;
	/* echo of the installed key for the readout */
	u32	sa, da;
	u16	sp, dp;
	u8	proto, profile;
};
static struct cn_l3e_manual cn_l3e_manual[CN_L3E_PROC_MAX_MANUAL];

/* The offload engine's countable quantities, for `ethtool -S` ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 108. */
void cortina_ni_flowoffload_stats(u64 out[CA_L3FE_STAT_COUNT])
{
	struct cn_l3e *l3e = cn_l3e;

	out[CA_L3FE_FLOWS_RESIDENT]	= atomic_read(&cn_flow_installed);
	out[CA_L3FE_DS_FLOWS_RESIDENT]	= atomic_read(&cn_ds_installed);
	out[CA_L3FE_HW_HITS]		= atomic_read(&cn_l3e_hw_hits);
	out[CA_L3FE_US_HITS]		= atomic_read(&cn_l3e_us_hits);
	out[CA_L3FE_DS_HITS]		= atomic_read(&cn_l3e_ds_hits);
	out[CA_L3FE_HITS_UNATTRIBUTED]	= atomic_read(&cn_l3e_hits_unattr);
	out[CA_L3FE_PPPOE_US_HITS]	= atomic_read(&cn_pppoe_us_hits);
	out[CA_L3FE_PPPOE_DS_HITS]	= atomic_read(&cn_pppoe_ds_hits);
	out[CA_L3FE_FLOWS_REFUSED]	= atomic_read(&cn_flow_refused);
	out[CA_L3FE_REFUSED_UNSUPPORTED] = atomic_read(&cn_flow_refused_unsupp);
	out[CA_L3FE_REFUSED_TABLE_FULL]	= atomic_read(&cn_flow_refused_full);
	out[CA_L3FE_REFUSED_DUPLICATE]	= atomic_read(&cn_flow_refused_dup);
	out[CA_L3FE_REFUSED_ERROR]	= atomic_read(&cn_flow_refused_err);
	out[CA_L3FE_VLAN_WAN_REFUSED_US] = atomic_read(&cn_vlan_wan_refused_us);
	out[CA_L3FE_VLAN_WAN_REFUSED_DS] = atomic_read(&cn_vlan_wan_refused_ds);
	out[CA_L3FE_VLAN_PPPOE_PROGRAMMED] = atomic_read(&cn_vlan_pppoe_ok);
	out[CA_L3FE_VLAN_PPPOE_READBACK_FAIL] =
					atomic_read(&cn_vlan_pppoe_readback);
	/* the DMA-AFT ledger lives in the engine instance; the refusal counters
	 * above do not, and are counted even when the engine never armed - so
	 * only these two are gated on it */
	out[CA_L3FE_VLAN_PUSH_LEGS]	= l3e ? l3e->aft_push : 0;
	out[CA_L3FE_VLAN_STRIP_LEGS]	= l3e ? l3e->aft_strip : 0;
}

/* The offload engine's own narrative. debugfs ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 109. */
/* ★ CONFIG_GPON_FLOW_DIAG gates the READ only.  The bring-up WRITE below is a
 * control, not a measurement -- this family REPORTS, it never decides. */
#if IS_ENABLED(CONFIG_GPON_FLOW_DIAG)
static void cn_l3e_debug_commit(struct seq_file *m, const u64 rx[2],
				const u16 stage[CN_L3E_STG_N])
{
	int i;

	/* Preserve baselines and the one-shot latch across seq_file overflow retries. */
	if (seq_has_overflowed(m))
		return;
	cn_l3e_ni_rx_prev[0] = rx[0];
	cn_l3e_ni_rx_prev[1] = rx[1];
	for (i = 0; i < CN_L3E_STG_N; i++)
		cn_l3e_stage_prev[i] = stage[i];
	cn_l3e_stage_seen = true;
	cn_l3e_latch_vec = -1;
}

int cortina_ni_l3fe_debug_show(struct seq_file *m, void *v)
{
	struct cn_l3e *l3e = cn_l3e;
	unsigned long flags;
	u64 rx[2];
	u16 stage[CN_L3E_STG_N];
	u32 cache_cnt;
	int i;

	if (!l3e) {
		seq_puts(m, "l3fe: engine not armed (cn_l3e == NULL)\n");
		return 0;
	}

	mutex_lock(&cn_flow_offload_mutex);
	/* ★ THE CORE'S OWN LINE, identical on every family: which of the
	 * lifecycle's sixteen refusal causes fired.  `offered=n/a` is honest --
	 * this shell counts REFUSALS by errno and has never counted OFFERS, and
	 * a zero there would read as "the kernel offered nothing". */
	{
		struct gpon_flow_diag d = {
			.valid = GPON_FDIAG_HAS_LIVE |
				 GPON_FDIAG_HAS_CAPACITY |
				 GPON_FDIAG_HAS_HITS,
			.live = atomic_read(&cn_flow_installed),
			.capacity = CN_L3E_ENTRIES,
			.hw_hits = atomic_read(&cn_l3e_hw_hits),
		};
		char line[256];

		gpon_flow_offload_diag(cn_fo, &d, line, sizeof(line));
		seq_printf(m, "%s\n", line);
	}
	cache_cnt = readl(l3e->ne_base + CN_L3E_HS_CACHE_CNT);
	seq_printf(m,
		   "install_ok=%d auto_flows=%d hw_hits=%d HS_CACHE_CNT(0x38c0)=%u(PHANTOM,do-not-use) live_pon{gem=%u tcont=%u} pppoe_sess=%#x gran(0x3924)=0x%08x\n",
		   cn_l3e_install_ok, atomic_read(&cn_flow_installed),
		   atomic_read(&cn_l3e_hw_hits), cache_cnt,
		   READ_ONCE(l3e->data_gem), READ_ONCE(l3e->data_tcont),
		   READ_ONCE(l3e->data_pppoe_session),
		   readl(l3e->ne_base + L3FE_HS_AGING_GRANULARITY));
	/* ★ the REFUSAL ledger: without it, "auto_flows did not go up" cannot be
	 * told apart from "the kernel never offered a flow".  Cumulative since
	 * boot; read twice and difference for a rate.  unsupp/full/dup are NORMAL
	 * refusals (the flow rides the SW fastpath); err is a real failure. */
	seq_printf(m,
		   "refused: total=%d unsupp=%d full=%d dup=%d err=%d last_errno=%d [refused != never-offered; unsupp/full/dup are normal, err is not]\n",
		   atomic_read(&cn_flow_refused),
		   atomic_read(&cn_flow_refused_unsupp),
		   atomic_read(&cn_flow_refused_full),
		   atomic_read(&cn_flow_refused_dup),
		   atomic_read(&cn_flow_refused_err),
		   atomic_read(&cn_flow_refused_last));
	/* ★ the VLAN-WAN breakdown of `unsupp`, so THIS branch is one ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 110. */
	seq_printf(m,
		   "vlan_wan: refused_us=%d refused_ds=%d last_vid=%d cause{direct=%d under_encap=%d action=%d} [subset of unsupp; non-zero = the WAN carries an 802.1Q layer, HW pushes no tag, flow kept on the SW fastpath]\n",
		   atomic_read(&cn_vlan_wan_refused_us),
		   atomic_read(&cn_vlan_wan_refused_ds),
		   atomic_read(&cn_vlan_wan_last_vid),
		   atomic_read(&cn_vlan_wan_direct),
		   atomic_read(&cn_vlan_wan_under),
		   atomic_read(&cn_vlan_wan_action));
	/* ★ The tagged-PPPoE ledger. `ok` counts legs whose vid, ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 111. */
	seq_printf(m,
		   "vlan_pppoe: hw_vlan_pppoe=%d ok=%d declined{no_sid=%d no_mac=%d bad_tpid=%d mismatch=%d} ac_mac{us=%d ds_blocked=%d} readback_fail=%d [ok>0 = tag AND session programmed on one action; readback_fail MUST be 0; ac_mac ds_blocked MUST be 0 or the DS leg was offered a zero next hop]\n",
		   hw_vlan_pppoe ? 1 : 0,
		   atomic_read(&cn_vlan_pppoe_ok),
		   atomic_read(&cn_vlan_pppoe_no_sid),
		   atomic_read(&cn_vlan_pppoe_no_mac),
		   atomic_read(&cn_vlan_pppoe_badtpid),
		   atomic_read(&cn_vlan_pppoe_mismatch),
		   atomic_read(&cn_vlan_pppoe_acmac_us),
		   atomic_read(&cn_vlan_pppoe_acmac_ds_blocked),
		   atomic_read(&cn_vlan_pppoe_readback));
	/* ★ THE ANCHOR that defeats the shared-wrong-offset trap: the ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 147. */
	seq_printf(m,
		   "fib_anchor: drv_pa=%pad entry_bytes=%u [must equal the engine's L3FE_HS_BA_MA0/MA1, which l3fe_fib_oracle.py reads independently]\n",
		   &l3e->fib_tbl_pa, (unsigned int)CN_L3E_FIB_BYTES);
	/* The DMA-AFT ledger: which arm fired for a tagged WAN. ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 112. */
	seq_printf(m,
		   "dma_aft: push=%u strip=%u reuse=%u tpid_armed=%u refused{no_tpid=%u full=%u timeout=%u} hw_vlan_wan=%d [push/strip>0 = the WAN VLAN is edited in HW; all-zero on an untagged WAN is EXPECTED, not a fault]\n",
		   l3e->aft_push, l3e->aft_strip, l3e->aft_reuse,
		   l3e->aft_tpid_armed,
		   l3e->aft_no_tpid, l3e->aft_full, l3e->aft_timeout,
		   hw_vlan_wan);
	if (l3e->dma_base) {
		u32 tw[2], aw[2], tc;
		u64 used;
		int k;

		/* separate statements, not an initializer list: init-list
		 * expressions are indeterminately sequenced and these are
		 * MMIO reads whose order must stay exactly as it was */
		tw[0] = readl(l3e->ne_base + CA_NI_L3FE_PP_TPID01);
		tw[1] = readl(l3e->ne_base + CA_NI_L3FE_PP_TPID23);
		tc = readl(l3e->ne_base + CA_NI_L3FE_PP_TPID_CTRL);
		aw[0] = readl(l3e->dma_base + CA_DMA_AFT_TPID01);
		aw[1] = readl(l3e->dma_base + CA_DMA_AFT_TPID23);

		/* ★ the fail-closed gate: stock ABORTS action generation when the
		 * WAN tag's TPID is absent here or its enable bit is clear, so a
		 * tagged flow falls back to software with everything else correct.
		 * 0x8100 must appear AND its slot bit must be set in top_mask. */
		seq_printf(m,
			   "l3fe_pp_tpid: slots{%04x %04x %04x %04x} ctrl=0x%02x top_mask=0x%x inner_mask=0x%x | dma_aft_tpid: slots{%04x %04x %04x %04x} [0x8100 must be present AND enabled or action-gen ABORTS and the flow goes to SW]\n",
			   cn_tpid_slot_at(tw, 0), cn_tpid_slot_at(tw, 1),
			   cn_tpid_slot_at(tw, 2), cn_tpid_slot_at(tw, 3),
			   tc & 0xff,
			   (u32)FIELD_GET(CA_NI_L3FE_PP_TPID_TOP_MASK, tc),
			   (u32)FIELD_GET(CA_NI_L3FE_PP_TPID_INNER_MASK, tc),
			   cn_tpid_slot_at(aw, 0), cn_tpid_slot_at(aw, 1),
			   cn_tpid_slot_at(aw, 2), cn_tpid_slot_at(aw, 3));

		spin_lock_irqsave(&l3e->aft_lock, flags);
		used = l3e->aft_fib_used;
		spin_unlock_irqrestore(&l3e->aft_lock, flags);
		for (k = CA_DMA_AFT_FIB_DYN_FIRST; k < CA_DMA_AFT_FIB_COUNT; k++) {
			u32 d0 = 0, d1 = 0, d2 = 0;

			if (!(used & BIT_ULL(k)))
				continue;
			if (cn_aft_fib_read(l3e, k, &d0, &d1, &d2)) {
				seq_printf(m, "  fib[%02d]: READ-BACK FAILED\n", k);
				continue;
			}
			/* shadow = what we asked for, hw = what the table holds.
			 * They must agree; a divergence is the finding. */
			seq_printf(m,
				   "  fib[%02d]: shadow{vid=%u cnt=%u ref=%u} hw{set_mode=%u cnt=%u vid=%u tpid_slot_p1=%u} raw{%08x %08x %08x} -> %s\n",
				   k, l3e->aft_fib_vid[k], l3e->aft_fib_cnt[k],
				   l3e->aft_fib_ref[k],
				   /* ⚠ CAST AT THE CALL SITE, NOT %lu IN THE FORMAT. FIELD_GET() ...
				    * dev/MEASURED-cortina-ni-flowoffload.c.md sec 113. */
				   (u32)FIELD_GET(CORTINA_AFT_D2_VLAN_SET_MODE, d2),
				   (u32)FIELD_GET(CORTINA_AFT_D2_TAG_CNT_MASK, d2),
				   (u32)FIELD_GET(CORTINA_AFT_D1_TOP_VID_MASK, d1),
				   (u32)FIELD_GET(CORTINA_AFT_D2_TPID_SLOT_MASK, d2),
				   d0, d1, d2,
				   FIELD_GET(CORTINA_AFT_D2_TAG_CNT_MASK, d2) ?
				   "PUSH (the US leg: this carries the WAN VLAN)" :
				   "STRIP (the DS leg: vid is legitimately 0 here)");
		}
	}
	/* DS (WAN->LAN) leg: hw_ds = the gate, ds_flows = reply legs ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 114. */
	seq_printf(m,
		   "hw_ds=%d ds_armed=%d ds_flows=%d ds_ldpid=%d ds_lan_l3if=%d ds_force_ldpid=%d ds_probe=%d\n",
		   hw_ds_offload, cn_ds_armed, atomic_read(&cn_ds_installed),
		   atomic_read(&cn_ds_last_ldpid), CN_L3E_LAN_L3IF_IDX,
		   hw_ds_lan_ldpid, hw_ds_probe);
	/* ★ PER-STAGE LEDGER - the whole point of this block is that ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 115. */
	seq_printf(m,
		   "ds_stage: ds_installed=%d us_hits=%d ds_hits=%d hits_unattr=%d%s\n",
		   atomic_read(&cn_ds_installed), atomic_read(&cn_l3e_us_hits),
		   atomic_read(&cn_l3e_ds_hits),
		   atomic_read(&cn_l3e_hits_unattr),
		   /* Only claim a broken instrument when the US control really is
		    * silent - printing that hint unconditionally read as "the
		    * witness is broken" even on a healthy us_hits>0 sample. */
		   atomic_read(&cn_l3e_us_hits) ? "" :
		   " (no upstream control hit recorded; verify offered traffic)");
	/* ★★ STAGE-A PRECONDITION, reported by the GPON driver: which PDC route
	 * the DS data GEM was programmed with.  FE-bypass = the frame never
	 * reaches ANY forwarding engine, so ds_hits CANNOT be non-zero and no
	 * conclusion about the hash, the key or the action is available. */
	seq_printf(m,
		   "ds_pdc: route=%s  [%s]\n",
		   cn_ds_pdc_into_l3fe < 0 ? "unreported (no WAN data path armed yet)" :
		   cn_ds_pdc_into_l3fe ? "LDPID L3_WAN -> into the L3FE" :
					 "CPU_0 + FE_BYPASS -> skips BOTH forwarding engines",
		   cn_ds_pdc_into_l3fe == 0 ?
		   "★ DS OFFLOAD CANNOT WORK: set cortina_gpon.hw_l3_ds=1 (needs cortina_ni.hw_l3_fwd=1 too)" :
		   "stage-A precondition satisfied");
	{
		u64 nihv[CA_NI_NIHV_CNT_COUNT];
		u64 l3fe_rx, l3qm_rx;

		/* the ONE reader; never readl() 0xa9bc/0xa9fc from here */
		cortina_ni_nihv_sample(l3e->ni, nihv);
		l3fe_rx = nihv[CA_NI_NIHV_L3FE_RX];
		l3qm_rx = nihv[CA_NI_NIHV_L3QM_RX];

		seq_printf(m,
			   "ni_hv: l3fe_rx(0xa9bc)=%llu delta=%llu l3qm_rx(0xa9fc)=%llu delta=%llu  [cumulative total since boot; delta = since the previous read of THIS file]\n",
			   l3fe_rx, l3fe_rx - cn_l3e_ni_rx_prev[0],
			   l3qm_rx, l3qm_rx - cn_l3e_ni_rx_prev[1]);
		seq_puts(m, "ni_hv: aggregate ingress; LAN traffic also increments l3fe_rx under DS FE_BYPASS. Attribute downstream traffic separately.\n");
		rx[0] = l3fe_rx;
		rx[1] = l3qm_rx;
	}
	/* ★★ STAGE A, measured INSIDE the engine: the L3FE's own four ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 116. */
	{
		int k;

		cn_l3e_stage_read(l3e, stage);
		seq_puts(m, "l3fe_stage:");
		for (k = 0; k < CN_L3E_STG_N; k++)
			seq_printf(m, " %s=%u(%s)", cn_l3e_stage_name[k],
				   stage[k] & 0x3ff,
				   !cn_l3e_stage_seen ? "first-read" :
				   stage[k] != cn_l3e_stage_prev[k] ? "ADVANCING" :
								  "frozen");
		seq_puts(m, "  [10-bit aggregate counters, modulo 1024; an unchanged value does not prove absence of traffic]\n");
	}
	/* ★★ STAGE B vs C, from the engine's own frozen descriptor. ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 117. */
	if (cn_l3e_latch_vec >= 0) {
		u32 bucket, nonzero = 0;
		int k, hits = 0;

		cn_l3e_latch_read(l3e, cn_l3e_latch_vec, cn_l3e_latch_buf,
				  CN_L3E_LATCH_WORDS);
		seq_printf(m, "latch[vec=%d] words:", cn_l3e_latch_vec);
		for (k = 0; k < CN_L3E_LATCH_WORDS; k++) {
			seq_printf(m, " %08x", cn_l3e_latch_buf[k]);
			nonzero |= cn_l3e_latch_buf[k];
		}
		seq_puts(m, "\n");
		/* scan the descriptor for every installed entry's CRC32 */
		for (bucket = 0; bucket < CN_L3E_AGE_ROWS; bucket++) {
			int slot;

			if (!l3e->bucket_occ[bucket])
				continue;
			for (slot = 0; slot < CN_L3E_AGE_SLOTS; slot++) {
				u32 idx = bucket * CN_L3E_AGE_SLOTS + slot;
				struct cn_flow_priv *e = l3e->entry_by_idx[idx];
				u32 crc32 = l3e->shadow_crc32[idx];

				if (!e || !crc32)
					continue;
				for (k = 0; k < CN_L3E_LATCH_WORDS; k++) {
					if (cn_l3e_latch_buf[k] != crc32)
						continue;
					seq_printf(m,
						   "latch: crc32=%08x of auto[%s] idx=%u FOUND at word %d => the engine hashed the latched frame to THIS entry's key, so stages A+B are OK and a still-unforwarded flow is STAGE C (the egress action)\n",
						   crc32, e->ds ? "DS" : "US",
						   idx, k);
					hits++;
					break;
				}
			}
		}
		if (!nonzero)
			seq_puts(m,
				 "latch: the descriptor read back all-zero - nothing was captured (no frame passed since the arm, or this die does not implement the latch)\n");
		else if (!hits)
			seq_puts(m,
				 "latch: no installed entry's CRC32 appears in the descriptor => either the latched frame belonged to a different flow (re-arm with ONLY the flow under test running), or the engine built a DIFFERENT key from it (STAGE B)\n");
	}
	/* ★ Per-flow, per-direction HIT poll of the AUTO ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 118. */
	{
		u32 bucket, trf, printed = 0, us_now = 0, ds_now = 0;
		/* traffic-bit vs age-re-arm cross-tab, [dir][rearm][bit] */
		u32 xtab[2][2][2] = {};

		for (bucket = 0; bucket < CN_L3E_AGE_ROWS; bucket++) {
			unsigned long traffic;
			u32 tword;
			int slot;

			if (!l3e->bucket_occ[bucket])
				continue;
			/* one non-destructive load per occupied bucket - covers
			 * all 32 of its entries (bucket == idx >> 5) */
			tword = readl(l3e->ne_base +
				      CN_L3E_HS_TRAFFIC_WORD(bucket * CN_L3E_AGE_SLOTS));
			if (cn_l3e_bucket_sweep(l3e, bucket, &trf))
				continue;	/* bounded GO timeout: next read */
			/* this read CONSUMES the re-arms, so it must feed the ...
			 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 119. */
			if (trf)
				atomic_add(hweight32(trf), &cn_l3e_hw_hits);
			traffic = trf;
			for (slot = 0; slot < CN_L3E_AGE_SLOTS; slot++) {
				u32 idx = bucket * CN_L3E_AGE_SLOTS + slot;
				struct cn_flow_priv *e = l3e->entry_by_idx[idx];
				bool hit = traffic & BIT(slot);
				u32 tbit;

				if (!e) {
					if (hit)
						atomic_inc(&cn_l3e_hits_unattr);
					continue;
				}
				if (hit) {
					e->last_hit = jiffies;
					e->hits++;
					atomic_inc(e->ds ? &cn_l3e_ds_hits :
							   &cn_l3e_us_hits);
					/* ★ THE SAME ATTRIBUTION THE 5 s SWEEP DOES. Without it this ...
					 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 120. */
					if (e->pppoe)
						atomic_inc(e->ds ?
							&cn_pppoe_ds_hits :
							&cn_pppoe_us_hits);
					if (e->ds)
						ds_now++;
					else
						us_now++;
				}
				tbit = !!(tword & BIT(slot));
				xtab[e->ds][hit][tbit]++;
				if (printed++ < CN_L3E_PROC_MAX_AUTO)
					seq_printf(m,
						   "auto[%s%s] idx=%u crc16=%04x key_tbl=%08x fib0=%08x hits=%u tbit=%u%s %s\n",
						   e->ds ? "DS" : "US",
						   e->probe == 1 ? ",probe1" :
						   e->probe == 2 ? ",probe2" : "",
						   idx, e->crc16,
						   l3e->key_tbl[idx],
						   *(u32 *)(l3e->fib_tbl +
							    (size_t)idx * CN_L3E_FIB_BYTES),
						   e->hits, tbit,
						   idx > CN_L3E_HS_TRAFFIC_MAX_IDX ?
							"(idx>16383: tbit UNPROVEN)" : "",
						   hit ? "*** HW HIT this read ***" :
							 "(no re-arm this read)");
			}
			if (!(bucket & 0x3f))
				cond_resched();
		}
		seq_printf(m,
			   "this_read: auto_entries=%u us_rearm=%u ds_rearm=%u (fresh HW re-arms CONSUMED by this read)\n",
			   printed, us_now, ds_now);
		/* ★ Calibrate the traffic bit on the KNOWN-WORKING leg before ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 121. */
		seq_printf(m,
			   "tbit_cal: US{rearm1:bit1=%u bit0=%u rearm0:bit1=%u bit0=%u} DS{rearm1:bit1=%u bit0=%u rearm0:bit1=%u bit0=%u}\n",
			   xtab[0][1][1], xtab[0][1][0], xtab[0][0][1], xtab[0][0][0],
			   xtab[1][1][1], xtab[1][1][0], xtab[1][0][1], xtab[1][0][0]);
		if (!xtab[0][1][1] && !xtab[0][1][0])
			seq_puts(m,
				 "tbit_cal: INCONCLUSIVE - the US leg gave no age re-arm in this read, so the bit's polarity is uncalibrated; re-read while an upstream transfer is running before trusting any DS tbit\n");
		else if (xtab[0][1][1] && !xtab[0][1][0])
			seq_printf(m,
				   "tbit_cal: polarity CALIBRATED on the US leg -> tbit=1 means TRAFFIC SEEN. DS entries reading tbit=1: %u, tbit=0: %u\n",
				   xtab[1][1][1] + xtab[1][0][1],
				   xtab[1][1][0] + xtab[1][0][0]);
		else if (xtab[0][1][0] && !xtab[0][1][1])
			seq_printf(m,
				   "tbit_cal: polarity CALIBRATED on the US leg -> tbit=0 means TRAFFIC SEEN (INVERTED vs the naive reading). DS entries reading tbit=0: %u, tbit=1: %u\n",
				   xtab[1][1][0] + xtab[1][0][0],
				   xtab[1][1][1] + xtab[1][0][1]);
		else
			seq_puts(m,
				 "tbit_cal: CONTRADICTORY - US re-arms appear with BOTH bit values, so the bit is not a per-entry traffic flag on this die (or it is clear-on-read and this poll consumed it). Do not use tbit; fall back to the age re-arm alone\n");
	}
	/* ★ THE VERDICT. Reduces the ledger above to the one sentence ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 122. */
	{
		int us = atomic_read(&cn_l3e_us_hits);
		int ds = atomic_read(&cn_l3e_ds_hits);
		int dsn = atomic_read(&cn_ds_installed);
		const char *verdict;

		if (!hw_ds_offload)
			verdict = "DS leg is OFF (hw_ds_offload=0) - downstream rides the CPU punt by design";
		else if (cn_ds_pdc_into_l3fe == 0)
			verdict = "STAGE A OFF BY CONFIGURATION: the DS data GEM's PDC route is CPU_0 + FE_BYPASS, so DS frames skip both forwarding engines and NO DS entry can ever be hit - ds_hits=0 here says nothing about the hash or the action. Re-boot with cortina_gpon.hw_l3_ds=1 as well, then re-read this file";
		else if (!cn_ds_armed)
			verdict = "STAGE 0: the DS leg never ARMED (L3-IF[3] / profile re-point failed) - see the boot log";
		else if (!dsn)
			verdict = "STAGE 0: no DS entry in silicon - every reply rule was REFUSED; enable the cn_rep_dbg refusal lines to see which branch";
		else if (!us)
			verdict = "INCONCLUSIVE: no upstream control hit recorded; verify offered traffic before judging DS";
		else if (!ds)
			verdict = "STAGE A/B FAIL: DS entries are live and the witness works (us_hits>0), but a DS entry NEVER matched => the DS frame does not reach the T2 lookup, or the engine's HDR_I key differs from ours. Next: boot with cortina_ni.hw_ds_probe=1 (then 2) to confirm, and do NOT chase the egress action yet";
		else if (hw_ds_probe)
			verdict = "STAGE A+B OK (probe mode: matched with no egress commit) => ingress admission and the hash key are BOTH correct, so the real DS failure is STAGE C, the egress action. Re-boot with hw_ds_probe=0 and fix the action";
		else
			verdict = "STAGE A+B OK with the REAL action (ds_hits>0): if downstream throughput is still the CPU-punt baseline, the failure is STAGE C - the frame hits, is forwarded, and dies on egress (wrong mcgid/deepq/L3-IF/next-hop)";
		seq_printf(m, "ds_verdict: %s\n", verdict);
	}
	/* ★★ PPPoE PER-STAGE LEDGER + VERDICT - the mirror of ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 123. */
	seq_printf(m,
		   "pppoe_stage: hw_pppoe=%d sess=%#x arms=%d arm_fail=%d pppoe_installed=%d pppoe_us_hits=%d pppoe_ds_hits=%d(a REAL witness since 2026-07-25: the DS leg now offloads PPPoE, so 0 here WITH ds_refused=0 and downstream at the punt rate is a failure) us_refused=%d ds_refused=%d(must be 0 at hw_pppoe=1 - a non-zero value means downstream fell back to the CPU punt, which is the 934->243 Mbps collapse) early_gone=%d(<%ums after install = the GAP-2 HW->SW flap)\n",
		   hw_pppoe, READ_ONCE(l3e->data_pppoe_session),
		   atomic_read(&cn_pppoe_arms), atomic_read(&cn_pppoe_arm_fail),
		   atomic_read(&cn_pppoe_installed),
		   atomic_read(&cn_pppoe_us_hits),
		   atomic_read(&cn_pppoe_ds_hits),
		   atomic_read(&cn_pppoe_us_refused),
		   atomic_read(&cn_pppoe_ds_refused),
		   atomic_read(&cn_pppoe_early_gone),
		   (unsigned int)CN_PPPOE_FLAP_MS);
	seq_printf(m,
		   "pppoe_punt: check=%d seen=%d ctrl=%d data=%d len_bad=%d tcp_bad=%d shift8=%d dblenc=%d sid_bad=%d sid_vs_armed=%d short=%d wire_sid=%#x  [len_bad/tcp_bad are a rate over `data`, NOT over `seen` - a baseline with a tiny data= cannot disprove a sub-percent rate, which is exactly how the 2026-07-24 92-frame 'oracle' misled; shift8/dblenc SHAPE the malformation, and NEITHER of them set means it is not an encap edit at all]\n",
		   cortina_ni_pppoe_punt_check,
		   atomic_read(&cn_pppoe_punt_seen),
		   atomic_read(&cn_pppoe_punt_ctrl),
		   atomic_read(&cn_pppoe_punt_data),
		   atomic_read(&cn_pppoe_punt_len_bad),
		   atomic_read(&cn_pppoe_punt_tcp_bad),
		   atomic_read(&cn_pppoe_punt_shift8),
		   atomic_read(&cn_pppoe_punt_dblenc),
		   atomic_read(&cn_pppoe_punt_sid_bad),
		   atomic_read(&cn_pppoe_punt_sid_vs_armed),
		   atomic_read(&cn_pppoe_punt_short),
		   cn_pppoe_punt_sid_seen);
	{
		int us = atomic_read(&cn_l3e_us_hits);
		int inst = atomic_read(&cn_pppoe_installed);
		int hits = atomic_read(&cn_pppoe_us_hits);
		int bad = atomic_read(&cn_pppoe_punt_len_bad) +
			  atomic_read(&cn_pppoe_punt_tcp_bad);
		const char *verdict;

		if (!hw_pppoe)
			verdict = "PPPoE HW encap is OFF (hw_pppoe=0) - PPPoE rides the SW fastpath by design. This is the BASELINE run: arm cortina_ni.pppoe_punt_check=1 here first, so a later hw_pppoe=1 run has an oracle for the punt counters";
		else if (!hw_l3_fwd)
			verdict = "STAGE ARM blocked: hw_l3_fwd is OFF, so the egress L3-IF entry is never written and every PPPoE flow is refused with -ENODEV. hw_l3_fwd is boot-time only - reboot with cortina_ni.hw_l3_fwd=1";
		else if (atomic_read(&cn_pppoe_arm_fail))
			verdict = "STAGE ARM FAIL: the egress L3-IF write failed/timed out, so the offload was refused rather than pointed at an unprogrammed entry (BUG-A). Look for the L3-IF ret= line in dmesg";
		else if (!atomic_read(&cn_pppoe_arms))
			verdict = "STAGE ARM: no session was ever armed - no flow rule carried a FLOW_ACTION_PPPOE_PUSH. Either the WAN is not PPPoE, or fw4's flowtable does not include the WAN lower device (check `nft list ruleset` for `flags offload` and that firewall.@defaults[0].flow_offloading_hw=1), or every flow was refused before the encap (see us_refused)";
		else if (!inst)
			verdict = "STAGE INSTALL: a session is armed but NO pushed entry is in silicon right now - every PPPoE rule was refused (see us_refused + the cn_rep_dbg refusal lines) or all of them have since been removed (see early_gone)";
		else if (!us)
			verdict = "INCONCLUSIVE: the age-re-arm witness itself is silent on the PROVEN IPoE leg too (us_hits=0), so no PPPoE conclusion may be drawn - fix the witness first";
		else if (!hits)
			verdict = "STAGE HIT: pushed entries are live and the witness works (us_hits>0), but no pushed entry EVER matched => the US frame does not reach the T2 lookup or hashes to a different key. This is exactly what 2026-07-20 observed; do NOT chase the encap yet";
		else if (!atomic_read(&cn_pppoe_ds_hits))
			/* ★ THE 2026-07-25 root cause, reported before anything ...
			 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 124. */
			verdict = "DOWNSTREAM NOT OFFLOADED (pppoe_ds_hits=0 while the US leg hits): every reply frame is on the CPU punt path. That by itself is the 934->243 Mbps collapse, AND it flaps the upstream entry - nf_flow_state_check() inspects every punted frame, so one FIN/RST (or one corrupted flag byte) tears the offload down for good. Check, in order: ds_refused (non-zero = the PPPoE leg gate refused a reply rule), hw_ds_offload=1, cortina_gpon.hw_l3_ds=1, the DS-leg profile invariants, and whether the LAN next-hop is in the L2-FDB. Do NOT read the punt counters as a cause while this holds";
		else if (atomic_read(&cn_pppoe_early_gone))
			verdict = "STAGE HOLD FAIL: pushed entries HIT but are torn down within the flap window, i.e. the flow keeps falling back to software (a FIN/RST or a mangled flag byte on a CPU-punted frame -> NF_FLOW_CLOSING -> GC). With the DS leg offloaded, punted DS frames should be rare, so check pppoe_punt data= and shape= before concluding, and remember that a benchmark's own connection teardowns land here too";
		else if (bad)
			verdict = "STAGE HOLD, DS MANGLE PRESENT: pushed entries HIT and hold, but some punted DS session frames are NOT self-consistent (see pppoe_punt: read len_bad/tcp_bad as a rate over data=, and read shape= - 8-BYTE-INSERT or DOUBLE-ENCAP means the packet editor edited the punt, NEITHER means it did not and the punt BUFFER is the suspect)";
		else
			verdict = "STAGE HIT+HOLD OK: pushed entries HIT, hold, and no DS mangling was detected. The remaining claim - that the wire frames carry 0x8864 + the live session id + the ONU WAN source MAC, with PPPoE length == inner IP total length + 2 - can ONLY be settled by a far-end capture; nothing in this file proves it";
		seq_printf(m, "pppoe_verdict: %s\n", verdict);
	}
	seq_puts(m,
		 "witness: hw_hits (age-SRAM re-arm) = the HW-offload proof (climbs while HW-forwarding); HS_CACHE_CNT & auto_flows are NOT hit witnesses\n");
	seq_puts(m, "usage: echo 'install <sa> <da> <sp> <dp> <proto> <profile> [mcgid] [new_sa] [new_sp]' > /proc/cortina_l3fe\n");
	seq_puts(m, "       echo 'pppoe <session_id>' (0 = clear/IPoE) > /proc/cortina_l3fe\n");
	seq_puts(m,
		 "stage-probe: boot cortina_ni.hw_ds_probe=1 (match-only, no datapath change) -> ds_hits>0 means ingress+hash are OK and the bug is the egress action; =2 (CPU_0 punt hit-action) only if 1 shows nothing\n");
	seq_puts(m,
		 "       echo 'latch [vector]' > /proc/cortina_l3fe  (default 2 = HDR_I before the packet editor, 0 = at ingress) then `cat` - captures ONE frame's descriptor and reports whether an installed entry's CRC32 is in it (stage B vs C)\n");
	seq_puts(m, "       echo 'rawinst <crc32-hex> <crc16-hex> [mcgid]' (TEMP DIAG: install the rx_crc_tap HW-read crc verbatim) > /proc/cortina_l3fe\n");
	for (i = 0; i < CN_L3E_PROC_MAX_MANUAL; i++) {
		struct cn_l3e_manual *e = &cn_l3e_manual[i];
		u32 age = 0, key = 0, fib0 = 0;

		if (!e->valid)
			continue;
		cn_l3e_age_get(l3e, e->idx, &age);
		key = l3e->key_tbl[e->idx];
		fib0 = *(u32 *)(l3e->fib_tbl + (size_t)e->idx * CN_L3E_FIB_BYTES);
		seq_printf(m,
			   "[%d] idx=%u crc16=%04x prof=%u  %pI4h:%u -> %pI4h:%u proto=%u  key_tbl=%08x fib0=%08x age=%u %s\n",
			   i, e->idx, e->crc16, e->profile,
			   &e->sa, e->sp, &e->da, e->dp, e->proto,
			   key, fib0, age,
			   age > CN_L3E_AGE_IDLE ? "*** HW HIT (age re-armed to START 2) ***" :
			   age == CN_L3E_AGE_IDLE ? "(live @IDLE 1, no hit yet)" : "(free/INVALID 0)");
		/* feed the cumulative hw_hits witness for the manual path (the
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 148. */
		if (age > CN_L3E_AGE_IDLE) {
			atomic_inc(&cn_l3e_hw_hits);
			cn_l3e_age_set(l3e, e->idx, CN_L3E_AGE_IDLE);
		}
	}
	cn_l3e_debug_commit(m, rx, stage);
	mutex_unlock(&cn_flow_offload_mutex);
	return 0;
}
#else	/* the facility is ABSENT, and the node says so rather than printing
	 * counters nobody measured */
int cortina_ni_l3fe_debug_show(struct seq_file *m, void *v)
{
	(void)v;
	seq_puts(m,
		 "l3fe: diagnostics not compiled in (CONFIG_GPON_FLOW_DIAG=n); the bring-up write still works\n");
	return 0;
}
#endif

/* The engine's WRITE side: manual flow install/delete, the ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 125. */
ssize_t cortina_ni_l3fe_debug_write(struct file *file, const char __user *ubuf,
				    size_t len, loff_t *ppos)
{
	struct cn_l3e *l3e;
	char buf[160], cmd[16] = {};
	char sas[40], das[40], nsas[40] = {};
	unsigned int sp, dp, proto, profile, mcgid = 0, nsp = 0;
	int n, i, err;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;

	if (sscanf(buf, "%15s", cmd) != 1)
		return -EINVAL;

	mutex_lock(&cn_flow_offload_mutex);
	/* ⚠ THE GLOBAL IS ACQUIRED INSIDE THE LOCK, and the admission ...
	 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 126. */
	l3e = cn_l3e;
	if (!l3e) {
		err = -ENODEV;
		goto out;
	}
	if (strcmp(cmd, "read") && !READ_ONCE(cn_flow_table_ready)) {
		err = -ENODEV;
		goto out;
	}

	if (!strcmp(cmd, "del")) {
		err = 0;
		for (i = 0; i < CN_L3E_PROC_MAX_MANUAL; i++) {
			int drc;

			if (!cn_l3e_manual[i].valid)
				continue;
			drc = cn_l3e_flow_del(l3e, cn_l3e_manual[i].idx,
					      cn_l3e_manual[i].crc16);
			/* ⚠ A REFUSED DELETE KEEPS ITS OWNER. Clearing `valid` ...
			 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 127. */
			if (drc) {
				pr_err("cortina-l3fe: manual entry %d (idx %u) was NOT deleted (%d) -- it stays OWNED here\n",
				       i, cn_l3e_manual[i].idx, drc);
				if (!err)
					err = drc;
				continue;
			}
			cn_l3e_manual[i].valid = false;
		}
		goto out;
	}
	if (!strcmp(cmd, "read")) {
		err = 0;	/* the readout is `cat` (show) */
		goto out;
	}
	if (!strcmp(cmd, "latch")) {
		/* Arm ONE L3FE descriptor capture; the next `cat` prints the ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 128. */
		int vec = CN_L3E_LATCH_VEC_HDRI_PRE_PE;

		if (sscanf(buf, "%*s %i", &vec) == 1 && (vec < 0 || vec > 7)) {
			err = -EINVAL;
			goto out;
		}
		cn_l3e_latch_arm(l3e);
		cn_l3e_latch_vec = vec;
		pr_info("cortina-l3fe: latch ARMED (vector %d) - read /sys/kernel/debug/cortina-l3fe/state\n",
			vec);
		err = 0;
		goto out;
	}
	if (!strcmp(cmd, "pppoe")) {
		/* first-bring-up path for the live session id (dec or 0x hex);
		 * 0 = clear back to IPoE */
		int sess;

		if (sscanf(buf, "%*s %i", &sess) != 1 ||
		    sess < 0 || sess > 0xffff) {
			err = -EINVAL;
			goto out;
		}
		err = cortina_ni_wan_pppoe_session_set(sess);
		goto out;
	}

	if (!strcmp(cmd, "rawinst")) {
		/* ★ TEMPORARY DIAGNOSTIC (P3 crc_ntfy divergence hunt - remove
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 129. */
		unsigned int c32, c16;

		mcgid = 0;
		n = sscanf(buf, "%*s %x %x %x", &c32, &c16, &mcgid);
		if (n < 2 || c16 > 0xffff) {
			err = -EINVAL;
			goto out;
		}
		for (i = 0; i < CN_L3E_PROC_MAX_MANUAL; i++)
			if (!cn_l3e_manual[i].valid)
				break;
		if (i == CN_L3E_PROC_MAX_MANUAL) {
			err = -ENOSPC;
			goto out;
		}
		{
			struct cn_l3e_act act = {};
			struct cn_l3e_manual *e = &cn_l3e_manual[i];

			/* manual bring-up: the operator-armed session
			 * (`echo 'pppoe <sid>'`) IS this path's sid source */
			if (mcgid == 0 &&
			    cn_l3e_set_us_egress(l3e, &act,
						 READ_ONCE(l3e->data_pppoe_session)) == 0) {
				act.ip_ttl_dec = 1;
			} else {
				act.permit = 1;
				act.dpid_vld = 1;
				act.dpid_pri = 1;
				act.deepq = 1;
				act.ip_ttl_dec = 1;
				act.mcgid = mcgid & 0x3ff;
				/* pass the double check + fill the action
				 * cache so HS_CACHE_CNT witnesses the hit */
				act.chk_msk_ptr = CN_L3E_WAN_MASK_ID;
				act.cache_ctrl = 1;
			}

			err = cn_l3e_flow_add_rawcrc(l3e, c32, c16, &act,
						     &e->idx);
			if (!err) {
				/* the age step-down to IDLE(1) is done by
				 * cn_l3e_flow_add_rawcrc for EVERY install path
				 * now (manual and automatic alike), so only a HW
				 * T2 HIT re-arms this slot up to START(2) */
				e->crc16 = c16;
				e->sa = 0;
				e->da = 0;
				e->sp = 0;
				e->dp = 0;
				e->proto = 0;
				e->profile = 0;
				e->valid = true;
				pr_info("cortina-l3fe: RAWINST idx=%u crc32=%08x crc16=%04x age=IDLE(1) (TEMP DIAG: age->2 / HS_CACHE_CNT>0 = HW hit on the exact HW-read crc)\n",
					e->idx, (u32)c32, (u16)c16);
			}
		}
		goto out;
	}

	if (strcmp(cmd, "install")) {
		err = -EINVAL;
		goto out;
	}

	n = sscanf(buf, "%*s %39s %39s %u %u %u %u %u %39s %u",
		   sas, das, &sp, &dp, &proto, &profile, &mcgid, nsas, &nsp);
	if (n < 6) {
		err = -EINVAL;
		goto out;
	}

	for (i = 0; i < CN_L3E_PROC_MAX_MANUAL; i++)
		if (!cn_l3e_manual[i].valid)
			break;
	if (i == CN_L3E_PROC_MAX_MANUAL) {
		err = -ENOSPC;
		goto out;
	}

	{
		struct cn_l3e_key key = {};
		struct cn_l3e_act act = {};
		u32 mask_id = (profile == CN_L3E_PROFILE_LAN) ?
			      CN_L3E_LAN_MASK_ID : CN_L3E_WAN_MASK_ID;
		struct cn_l3e_manual *e = &cn_l3e_manual[i];

		key.ip_sa_0 = cn_l3e_proc_parse_ip(sas);
		key.ip_da_0 = cn_l3e_proc_parse_ip(das);
		key.l4_sport = sp;
		key.l4_dport = dp;
		key.ip_protocol = proto;
		key.ip_ver = 0;
		key.ip_vld = 1;

		/* Forward action. With mcgid==0 (default) and a live PON data ...
		 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 130. */
		if (mcgid == 0 &&
		    cn_l3e_set_us_egress(l3e, &act,
					 READ_ONCE(l3e->data_pppoe_session)) == 0) {
			act.ip_ttl_dec = 1;
		} else {
			act.permit = 1;
			act.dpid_vld = 1;
			act.dpid_pri = 1;
			act.deepq = 1;
			act.ip_ttl_dec = 1;
			act.mcgid = mcgid & 0x3ff;
		}
		if (n >= 8 && nsas[0]) {
			act.ip_addr_vld = 1;
			act.ip_type = 0;	/* rewrite SA (SNAT) */
			act.ip_addr = cn_l3e_proc_parse_ip(nsas);
		}
		if (n >= 9 && nsp) {
			act.l4_port = nsp;
		}

		err = cn_l3e_flow_add(l3e, &key, &act, profile, mask_id,
				      &e->idx, &e->crc16);
		if (!err) {
			/* ★ HIT WITNESS: cn_l3e_flow_add arms the entry at START(2) ...
			 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 131. */
			e->sa = key.ip_sa_0;
			e->da = key.ip_da_0;
			e->sp = sp;
			e->dp = dp;
			e->proto = proto;
			e->profile = profile;
			e->valid = true;
			pr_info("cortina-l3fe: manual install idx=%u crc16=%04x prof=%u mask=%u age=IDLE(1) (coherent key_tbl write; re-arm >IDLE = HW hit)\n",
				e->idx, e->crc16, profile, mask_id);
		}
	}
out:
	mutex_unlock(&cn_flow_offload_mutex);
	return err ? err : len;
}


/* probe entry (called once from the cortina-ni platform ...
 * dev/MEASURED-cortina-ni-flowoffload.c.md sec 149. */

int cortina_ni_flowoffload_probe(struct cortina_ni *ni)
{
	void __iomem *ne = ni->win[CA_NI_WIN_NI];
	struct cn_l3e *l3e;
	int ret;

	if (!ne)
		return -ENODEV;

	l3e = devm_kzalloc(ni->dev, sizeof(*l3e), GFP_KERNEL);
	if (!l3e)
		return -ENOMEM;
	l3e->dev = ni->dev;
	l3e->ni = ni;
	l3e->ne_base = ne;
	/* the DMA window is already mapped for the TX ring; the DMA-AFT VLAN
	 * edit tables live in the same 4K page, so there is nothing to map. */
	l3e->dma_base = ni->win[CA_NI_WIN_DMA];
	if (!l3e->dma_base)
		dev_warn(ni->dev,
			 "DMA window absent - the hardware WAN VLAN edit is unavailable; a tagged WAN will stay on the software fastpath\n");
	spin_lock_init(&l3e->reg_lock);
	spin_lock_init(&l3e->aft_lock);

	/* router MAC for the my-MAC FIELD-CAM commit (same source + fallback
	 * as the RX steer init); WAN MAC is derived as base+1 in the enable */
	if (ni->tx && ni->tx->netdev) {
		ether_addr_copy(l3e->router_mac, ni->tx->netdev->dev_addr);
		l3e->router_mac_valid = true;
	}

	/* one coherent carve: key table then FIB (stock places the FIB at
	 * key base + 0x40000 the same way) */
	l3e->carve = dma_alloc_coherent(ni->dev, CN_L3E_CARVE_BYTES,
					&l3e->carve_pa, GFP_KERNEL);
	if (!l3e->carve)
		return -ENOMEM;
	l3e->key_tbl = l3e->carve;
	l3e->key_tbl_pa = l3e->carve_pa;
	l3e->fib_tbl = l3e->carve + CN_L3E_KEY_TBL_BYTES;
	l3e->fib_tbl_pa = l3e->carve_pa + CN_L3E_KEY_TBL_BYTES;

	/* spy-first: the engine must be un-armed at this point (boot ROM /
	 * U-Boot never touch it; live-verified all-zero pre-init) */
	dev_info(ni->dev,
		 "l3fe: pre-arm MH0=%08x MA0=%08x INI=%08x (expect all 0)\n",
		 readl(ne + L3FE_HS_BA_MH0), readl(ne + L3FE_HS_BA_MA0),
		 readl(ne + L3FE_HS_HASH_INI));

	ret = cn_l3e_init(l3e);
	if (ret)
		goto err_free_carve;

	cn_l3e_swo_selftest(l3e);
	cn_l3e_hdri_live_test(l3e);

	ret = cn_flowoffload_init();
	if (ret) {
		cn_l3e = NULL;
		cn_l3e_free_shadow(l3e);
		goto err_free_carve;
	}

	/* the state dump + the manual-install control are published from
	 * cortina_ni_debugfs_init(), which runs at the end of probe */

	/* phase-1 gate evidence: read back everything the arm wrote */
	dev_info(ni->dev,
		 "l3fe: armed carve pa=%pad MH0=%08x MH1=%08x MA0=%08x MA1=%08x INI=%08x MEMINI=%08x RSV0=%08x RSV1=%08x AXIM2=%08x CHKFAIL=%08x GRAN=%08x AQM=%08x\n",
		 &l3e->carve_pa,
		 readl(ne + 0x383c), readl(ne + 0x3838),
		 readl(ne + 0x3844), readl(ne + 0x3840),
		 readl(ne + 0x3834), readl(ne + 0x393c),
		 readl(ne + 0x3944), readl(ne + 0x3948),
		 readl(ne + 0x3c80), readl(ne + 0x3940),
		 readl(ne + 0x3924), readl(ne + 0x3aa8));
	dev_info(ni->dev,
		 "l3fe: SWO CRC selftest %s (pass=%u fail=%u ret=%d)\n",
		 (!l3e->selftest_ret && l3e->selftest_fail == 0 &&
		  l3e->selftest_pass) ? "PASS" : "FAIL",
		 l3e->selftest_pass, l3e->selftest_fail, l3e->selftest_ret);
	dev_info(ni->dev,
		 "l3fe: HDR_I 5-tuple key-packing %s (pass=%u fail=%u)\n",
		 (l3e->hdri_live_pass && !l3e->hdri_live_fail) ? "LIVE" : "FAIL",
		 l3e->hdri_live_pass, l3e->hdri_live_fail);
	return 0;

err_free_carve:
	dma_free_coherent(ni->dev, CN_L3E_CARVE_BYTES, l3e->carve,
			  l3e->carve_pa);
	return ret;
}
