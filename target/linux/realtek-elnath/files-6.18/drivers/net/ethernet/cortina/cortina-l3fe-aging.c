// SPDX-License-Identifier: GPL-2.0-only
/*
 * cortina-l3fe-aging.c — Cortina CA8277C (RTL9607F "Elnath") L3FE main-hash
 * flow-engine AGING / GC / HIT-FEED sub-block for the nf_flow_table HW offload.
 *
 * This is the sub-block that keeps conntrack/flowtable timeouts refreshed from
 * hardware traffic at scale, and evicts dead flows out of the 64K main hash:
 *
 *   - autonomous HW hit-rearm: a packet match re-arms an entry's 2-bit age to
 *     START(2).
 *   - SW keepalive sweep: a batch "had-traffic" read+clear of a whole 32-entry
 *     bucket in ONE indirect access (cl3a_traffic_status_get) — bit set == HW saw
 *     a packet since the last sweep; the same access clears every live slot back
 *     to IDLE(1).  This feeds FLOW_CLS_STATS -> f->stats.lastused.
 *   - AQM byte/packet MIB (up to 2048 metered flows) for real counters.
 *   - go-live / kill via the age field; explicit on-chip cache invalidate on del.
 *
 * Clean-room re-expression of the shipped stock ca-ne.ko (aal-07f) register
 * sequences + the Cortina aal HAL semantics.  No vendor source text reproduced.
 * Facts, bit layouts and Ghidra citations:
 *   dev/x400axf/HW_FLOW_OFFLOAD_AGING_GC.md
 *
 * ★ Elnath-specific: the main-hash age is 2-BIT (0..3), START=2, STATIC=3 —
 *   NOT the 3-bit/START=6 model in the aal-77c HAL header.  Proven by the
 *   ca-ne.ko decomp (age masked &3, /0x2000 in aging_timer_set, 0x7ffffffe000 in
 *   hash_init).  A wrong-width age RMW corrupts the neighbouring slot.
 *
 * NOTE: built under CONFIG_CORTINA_NI_FLOWOFFLOAD as the documented aging/
 *       AQM reference.  The production liveness sweep is the copy folded into
 *       cortina-ni-flowoffload.c (integrated with its SW shadow); the AQM MIB
 *       reader here becomes the phase-4 per-flow byte/packet counter option.
 *       Register offsets verified decomp==rtl8277c header.
 *
 * ★★ WHAT THIS FILE IS, EXACTLY — read this before copying anything out of it.
 * Every definition below is `static inline` or `static __maybe_unused` and
 * NOTHING outside this file names a cl3a_* symbol, so the object it compiles to
 * is EMPTY: it emits no reachable code and no bus cycle.  It is a TRANSCRIPTION
 * of the vendor sequences (HW_FLOW_OFFLOAD_AGING_GC.md §3a-3g), kept because the
 * AQM byte/packet MIB reader below exists nowhere else in the tree.  Two
 * consequences a reader must carry away:
 *
 *   1. IT TAKES NO LOCK.  Every indirect latch/RMW/commit in the SHIPPING copy
 *      runs inside spin_lock_irqsave(&l3e->reg_lock) — cn_l3e_age_set(),
 *      cn_l3e_bucket_sweep(), cn_l3e_cache_invalidate() in
 *      cortina-ni-flowoffload.c.  Two CPUs in one of the sequences here would
 *      interleave a latch between another's read and its commit and write back
 *      a torn age row.  Wiring any of this up means supplying that lock.
 *   2. IT IS NOT THE AUTHORITY ON WHAT THE DRIVER DOES.  Where the shipping
 *      copy has since diverged, the divergence is written down at the site
 *      (the sweep's STATIC handling, the cache slot mask, the delete order),
 *      because a reference that silently disagrees with the driver is worse
 *      than no reference.
 *
 * ★ CORRECTED 2026-09-04: an earlier version of this comment claimed we keep the
 * HW auto-countdown (AGING_GRANULARITY) DISABLED "to match stock, so hardware
 * never ages a live offloaded flow out from under the Linux flowtable".  The
 * shipping driver does NOT do that.  It writes 0 at engine init
 * (cortina-l3fe.c, cortina_l3fe_engine_init) and then programs
 * L3FE_AGING_GRAN_SLOW in cortina_l3fe_hw_l3_forward_enable(), because with
 * granularity 0 the ager block never runs, the age slot reads stale, and the
 * age re-arm can therefore never witness a hit — which is exactly what the SW
 * sweep below depends on.  cl3a_aging_init() still writes 0 because it mirrors
 * ENGINE INIT only; see the note there.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include "cortina-l3fe-regs.h"	/* the L3FE registers more than one file needs */
/* CN_L3E_HASH_WAYS and CN_L3E_AGE_SLOTS -- the CORE owns this engine's two
 * geometries, see the geometry block below.  Declarations and #defines only, so
 * including it links nothing new; the -I is already there (cortina/Makefile:
 * -I$(srctree)/drivers/net/flowcore). */
#include "cortina_ni_flowoffload_logic.h"

/* -------------------------------------------------------------------------
 * NE register base + L3FE hash-engine (HS) aging/cache register file.
 * NE APB0 base = 0xf4300000 (proven: our RX driver uses the rtl8277c L3FE map,
 * ILPB_LDPID @0xf43030d8; the aal-07f decomp offsets match rtl8277c exactly).
 * Offsets below are relative to the ioremap of NE_REG_BASE.
 * ------------------------------------------------------------------------- */
#define CL3A_NE_REG_BASE		0xf4300000UL

/* --- indirect age memory (main-hash + overflow) --- */
#define  CL3A_AGE_TBL_HASH		(0u << 11)	/* address bit11 = age-table select */
#define  CL3A_AGE_TBL_OVERFLOW		(1u << 11)

/* --- on-chip action-cache control (invalidate/allocate) --- */
#define  CL3A_CACHE_CMD_INVALIDATE	(1u << 30)	/* cmd = 01 */
#define  CL3A_CACHE_STS_ERR_NCH		BIT(3)		/* "entry not cached" (non-fatal) */
#define  CL3A_CACHE_STS_EVICT		BIT(6)
#define  CL3A_CACHE_STS_CACHED		BIT(4)
#define  CL3A_CACHE_STS_NO_WAY		BIT(2)		/* alloc error */
#define  CL3A_CACHE_STS_INVALID		BIT(1)		/* alloc error */

/* --- AQM byte/packet MIB: shares the cache indirect access reg, table=2 --- */
#define CL3A_CACHE_HASH_ACCESS		0x38e4	/* AQM/cache indirect access reg */
#define  CL3A_AQM_TBL_SEL		(2u << 11)
#define CL3A_AQM_DATA_REGS		7	/* data word i at ACCESS + ((7-i)<<2) */

#define CL3A_GO				BIT(31)
#define CL3A_WRITE			BIT(30)

/* --- geometry ---
 * ★★ TWO HARDWARE FACTS, TWO NAMES — until 2026-09-04 one `CL3A_WAYS 32` was
 * used for BOTH, and they are not the same number:
 *
 *   AGE SRAM row  = 32 slots x 2 bits, FIXED.  64K entries / 32 = 2048 rows,
 *                   row = idx >> 5.  Independent of the hash configuration.
 *   HASH bucket   = 8 ways on this die.  HASH_INI reads 0x0003007D live
 *                   (hb_size = 1), devmem-captured 2026-07-18, and our own
 *                   driver writes that same word back (cortina-l3fe.c,
 *                   L3FE_HASH_INI_VAL).  32-way was the STATIC-RE guess and is
 *                   refuted — tier 1 wins.
 *
 * Only cl3a_cache_invalidate() wants the WAY count, and it had the wrong one:
 * the vendor's own sequence masks with `idx & 0x1f & (hb_ways - 1)`
 * (HW_FLOW_OFFLOAD_AGING_GC.md §3e) and this copy had dropped the hb_ways term,
 * leaving idx & 31 where the shipping driver writes idx & 7.
 *
 * ⚠ THE FIX IS NOT "SET CL3A_WAYS TO 8" — that would silently turn the 2048
 * age rows below into 8192.  Both numbers now come from the CORE, which has
 * kept them apart all along (cortina_ni_flowoffload_logic.h: CN_L3E_AGE_SLOTS,
 * CN_L3E_HASH_WAYS), so the two copies cannot drift apart again.
 * CL3A_ENTRIES stays local: the core owns no entry count (CN_L3E_ENTRIES is
 * private to cortina-ni-flowoffload.c), so there is nothing to route it to.
 */
#define CL3A_ENTRIES			65536
#define CL3A_BUCKETS			(CL3A_ENTRIES / CN_L3E_AGE_SLOTS)	/* 2048 */
#define CL3A_AQM_MAX			2048	/* L3FE_AQM_FLOW_STAT_MAX */

/* --- 2-bit age codes (Elnath 07f) ---
 * These four have the same values as the core's CN_L3E_AGE_* (in the header
 * included above) and are DELIBERATELY NOT merged into them.  The values agree;
 * the USE of code 3 does not — the sweep below demotes it, the core's
 * cn_age2_sweep_word() exempts it (see the ★ at cl3a_traffic_status_get).  One
 * shared name would assert an agreement that does not exist and would hide an
 * open question.  Merge them the day the two sweeps are made to agree.
 */
#define CL3A_AGE_FREE			0	/* aged-out / invalid, stops matching */
#define CL3A_AGE_IDLE			1	/* keepalive floor written by the sweep */
#define CL3A_AGE_START			2	/* on add; HW re-arms here on a hit */
/* 3 = STATIC.  What this line used to say — "never ages out" — is wrong about
 * this file's own code: the SW sweep below steps 3 down to IDLE like any other
 * live slot, which is what the vendor sequence does.  Whether the HW ager
 * exempts it is what the vendor NAME implies and nothing in this tree measures;
 * no code path writes 3 today, so the question has never been exercised. */
#define CL3A_AGE_STATIC			3

struct cl3a_ctx {
	void __iomem *ne;		/* ioremap of NE_REG_BASE */
	bool cache_enabled;		/* on-chip action cache in use */
};

/* ------------------------------------------------------------------ */
/* Indirect-access helpers — every op is a bounded poll on the GO bit. */
/*                                                                     */
/* ★ THE WRITE-THEN-POLL PAIR IS NOT SPELLED HERE ANY MORE.  Seven     */
/* sites below wrote an ACCESS word and then polled that SAME word's   */
/* GO bit by hand; they now call l3fe_access_go() (cortina-l3fe-regs.h)*/
/* which is gpon_ind_go() in the CORE (flowcore/regtable.h): the same  */
/* write, the same bound (L3FE_ACCESS_TRIES = the vendor's             */
/* TABLE_TRY_TIMEOUT), the same cpu_relax() pause and the same rc      */
/* convention, plus a reg_has() refusal so an absent register never    */
/* gets a transaction.  The shipping copy already routes the identical */
/* sequences that way (cortina-ni-flowoffload.c).                      */
/*                                                                     */
/* Byte-identical on the bus: cl3a_wr()'s dma_wmb() is not a bus       */
/* cycle, and on this arm64 target writel() already issues one itself  */
/* (asm-generic/io.h calls __io_bw(), which arm64 defines as           */
/* dma_wmb()), so the core's plain l3fe_hwio_wr() emits the same       */
/* barrier and the same store.  cl3a_wr() stays for the four writes    */
/* that are NOT a request word (the two age DATA words, the cache CTRL */
/* parameter word, and the aging-granularity write).                   */
/* ------------------------------------------------------------------ */
static inline u32 cl3a_rd(struct cl3a_ctx *c, u32 off)
{
	return readl(c->ne + off);
}

static inline void cl3a_wr(struct cl3a_ctx *c, u32 off, u32 val)
{
	/* dmb before every register write, mirroring the stock DataMemoryBarrier */
	dma_wmb();
	writel(val, c->ne + off);
}

/* Poll a self-clearing GO/busy bit; returns 0 on done, -ETIMEDOUT on cap. */

/* Map a slot to (age data-reg offset, in-word bit shift).  2-bit ages,
 * 16 slots per 32-bit word: word0 = slots 0..15, word1 = slots 16..31. */
static inline u32 cl3a_age_dataoff(u32 slot)
{
	return (slot & 0x1f) < 16 ? L3FE_HS_AGE_DATA_LO : L3FE_HS_AGE_DATA_HI;
}
static inline u32 cl3a_age_shift(u32 slot)
{
	return 2u * (slot & 0xf);
}

/* ------------------------------------------------------------------ */
/* 3a. Read one entry's 2-bit age.                                     */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_age_get(struct cl3a_ctx *c, u32 idx, u8 *age)
{
	u32 addr = ((idx >> 5) & 0x7ff) | CL3A_AGE_TBL_HASH;
	u32 word;
	int ret;

	/* latch the row: write the request, poll its GO bit */
	ret = l3fe_access_go(c->ne, L3FE_HS_AGE_ACCESS, addr | CL3A_GO, CL3A_GO);
	if (ret)
		return ret;

	word = cl3a_rd(c, cl3a_age_dataoff(idx));
	*age = (word >> cl3a_age_shift(idx)) & 0x3;
	return 0;
}

/* ------------------------------------------------------------------ */
/* 3b. Write one entry's 2-bit age.  age>0 = GO-LIVE, age==0 = KILL.   */
/*     RMW: ages share a 32-bit word, so read-modify the row first.    */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_age_set(struct cl3a_ctx *c, u32 idx, u8 age)
{
	u32 addr = ((idx >> 5) & 0x7ff) | CL3A_AGE_TBL_HASH;
	u32 dataoff = cl3a_age_dataoff(idx);
	u32 shift = cl3a_age_shift(idx);
	u32 word;
	int ret;

	age &= 0x3;

	/* 1. read the bucket's age row */
	ret = l3fe_access_go(c->ne, L3FE_HS_AGE_ACCESS, addr | CL3A_GO, CL3A_GO);
	if (ret)
		return ret;

	/* 2. patch just this slot's 2 bits */
	word = cl3a_rd(c, dataoff);
	word = (word & ~(0x3u << shift)) | ((u32)age << shift);
	cl3a_wr(c, dataoff, word);

	/* 3. commit = GO-LIVE (bit31 go + bit30 write) */
	return l3fe_access_go(c->ne, L3FE_HS_AGE_ACCESS,
			      addr | CL3A_GO | CL3A_WRITE, CL3A_GO);
}

/* ------------------------------------------------------------------ */
/* 3c. Batch had-traffic read+clear of ONE 32-entry bucket.            */
/*     bucket_base_idx must be 32-aligned (slot 0 of the group).       */
/*     *trf = 32-bit bitmap, bit k set iff slot k had traffic          */
/*     (age > IDLE) since the last sweep; the same op resets every      */
/*     live slot back to IDLE(1) so only a fresh HW hit lifts it again. */
/*     This is the scale path: ONE indirect read+clear per 32 flows,    */
/*     not one age_get per flow.                                        */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_traffic_status_get(struct cl3a_ctx *c, u32 bucket_base_idx,
				   u32 *trf)
{
	u32 addr = ((bucket_base_idx >> 5) & 0x7ff) | CL3A_AGE_TBL_HASH;
	static const u32 dregs[2] = { L3FE_HS_AGE_DATA_LO, L3FE_HS_AGE_DATA_HI };
	int ret, w, k;

	if (bucket_base_idx & 0x1f)		/* only the first index of a group */
		return -EINVAL;

	*trf = 0;

	/* read the whole 32-slot age row */
	ret = l3fe_access_go(c->ne, L3FE_HS_AGE_ACCESS, addr | CL3A_GO, CL3A_GO);
	if (ret)
		return ret;

	for (w = 0; w < 2; w++) {		/* 2 words x 16 slots x 2 bits */
		u32 word = cl3a_rd(c, dregs[w]);

		for (k = 0; k < 16; k++) {
			u32 age = (word >> (2 * k)) & 0x3;
			u32 slot = (w << 4) + k;

			/* ★ STATIC(3) IS TREATED AS TRAFFIC AND STEPPED DOWN
			 * TO IDLE HERE, AND THE CORE DOES THE OPPOSITE.
			 * This is the vendor sequence transcribed:
			 * HW_FLOW_OFFLOAD_AGING_GC.md §3c is `if age > 1:
			 * trfStatus |= ...` then `age == 0 ? 0 : 1`, with no
			 * exemption.  The core's cn_age2_sweep_word()
			 * (flowcore/cortina_ni_flowoffload_logic.c) tests
			 * `age > IDLE && age != STATIC` and leaves 3 alone.
			 * NOTHING IN THIS TREE SETTLES WHICH IS A HARDWARE
			 * REQUIREMENT — no code path writes age 3 today
			 * (install writes START, kill writes FREE, the sweep
			 * and the stats read write IDLE), so the difference
			 * has never been exercised.  It is carried visibly
			 * rather than flattened: this file's job is to say
			 * what the vendor does, the core's is to say what the
			 * driver does, and they do not agree here.
			 */
			if (age > CL3A_AGE_IDLE)	/* HW re-armed -> had traffic */
				*trf |= (1u << slot);
			/* keepalive: nonzero -> IDLE(1), zero stays 0 */
			word &= ~(0x3u << (2 * k));
			word |= (age ? CL3A_AGE_IDLE : CL3A_AGE_FREE) << (2 * k);
		}
		cl3a_wr(c, dregs[w], word);
	}

	/* commit the cleared row (GO-LIVE write) */
	return l3fe_access_go(c->ne, L3FE_HS_AGE_ACCESS,
			      addr | CL3A_GO | CL3A_WRITE, CL3A_GO);
}

/* ------------------------------------------------------------------ */
/* 3e. On-chip action-cache invalidate (part of flow delete).         */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_cache_invalidate(struct cl3a_ctx *c, u32 idx, u16 crc16)
{
	u32 ctrl, sts;
	int ret;

	/* ★ THE SLOT IS THE WAY WITHIN THE 8-WAY HASH BUCKET, and this copy had
	 * it as idx & 31 because one constant served two geometries (see the
	 * geometry block above).  The vendor masks with `idx & 0x1f &
	 * (hb_ways - 1)` — 0x1f is the CTRL slot[4:0] FIELD WIDTH, the second
	 * term is the ACTUAL bucket width, and only the first was here.  Both
	 * are kept, spelled as the vendor spells them; on this die hb_size = 1
	 * so this is idx & 7, which is what the shipping driver writes
	 * (cortina-ni-flowoffload.c, cn_l3e_cache_invalidate). */
	ctrl = (idx & 0x1f & (CN_L3E_HASH_WAYS - 1))	/* way within the bucket */
	     | ((u32)crc16 << 5)			/* crc16[20:5] */
	     | CL3A_CACHE_CMD_INVALIDATE;		/* cmd = 01 */

	/* decomp order: write CTRL params, wait REQ idle, pulse REQ GO, wait done */
	cl3a_wr(c, L3FE_HS_CACHE_CTRL, ctrl);
	/* ⚠ NOT a write-then-poll pair: this waits on CTRL_REQ, a DIFFERENT
	 * register from the CTRL word just written, so it stays a bare wait.
	 * The engine must be idle BEFORE the request is raised.  (The shipping
	 * copy makes the same exclusion, for the same reason.) */
	ret = l3fe_access_wait(c->ne, L3FE_HS_CACHE_CTRL_REQ, BIT(0));	/* wait not-busy */
	if (ret)
		return ret;
	/* the GO pulse IS a write-then-poll pair on one register: read-modify
	 * the REQ word, write it, poll bit0 until the engine clears it. */
	ret = l3fe_access_go(c->ne, L3FE_HS_CACHE_CTRL_REQ,
			     cl3a_rd(c, L3FE_HS_CACHE_CTRL_REQ) | 1, BIT(0));
	if (ret)
		return ret;

	sts = cl3a_rd(c, L3FE_HS_CACHE_CTRL_STS);
	if (sts & CL3A_CACHE_STS_ERR_NCH)
		pr_debug("l3fe cache invalidate idx %u: entry was not cached (non-fatal)\n",
			 idx);
	return 0;
}

/* ------------------------------------------------------------------ */
/* 3d. Flow delete — the vendor order (HW_FLOW_OFFLOAD_AGING_GC.md §3d): */
/*     stop matching, clear the action row, clear the key row, flush   */
/*     the on-chip cache.  Caller supplies the recovered crc16 (from   */
/*     the key row) and the FIB/key row clear callbacks.               */
/*                                                                     */
/* ★ BLACKHOLE-SAFETY: EVERY STEP RUNS EVEN WHEN ONE FAILS.  This used */
/* to return at the first error, which is the shape the shipping copy  */
/* abandoned after it bit: an age-GO timeout skipped the FIB clear, the*/
/* key clear and the cache invalidate, leaving a complete entry — key, */
/* action and a live age — orphaned and matching forever.  Worse, an   */
/* age_set "commit" timeout means the write WAS issued and may land    */
/* late, so the error return is exactly when the rest is most needed.  */
/* The first error is reported; nothing is skipped because of it.  The */
/* cache invalidate's rc was being discarded outright and is now kept: */
/* a stale cached action keeps matching {crc16, slot} after the DDR FIB*/
/* is gone, which is the failure this whole step exists to prevent.    */
/*                                                                     */
/* ⚠ NOT MERGED with cn_l3e_flow_del() (cortina-ni-flowoffload.c), and */
/* the difference is REAL, not cosmetic: there the key table is a DMA  */
/* buffer in DRAM, so `key_tbl[idx] = 0` cannot fail and is done FIRST,*/
/* which is a stronger stop-matching than the age write.  This copy has*/
/* no key table — it only has a callback that can fail — so it keeps   */
/* the vendor's age-first order.  Nothing in the tree says which order */
/* the SILICON requires; what is established is only that both must    */
/* precede the cache flush.                                            */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_flow_del(struct cl3a_ctx *c, u32 idx, u16 crc16,
			 int (*clear_fib)(struct cl3a_ctx *, u32),
			 int (*clear_key)(struct cl3a_ctx *, u32))
{
	int ret, first;

	first = cl3a_age_set(c, idx, CL3A_AGE_FREE);	/* 1. STOP MATCHING FIRST */
	if (clear_fib) {
		ret = clear_fib(c, idx);		/* 2. clear action/FIB row */
		if (ret && !first)
			first = ret;
	}
	if (clear_key) {
		ret = clear_key(c, idx);		/* 3. clear key/hash row */
		if (ret && !first)
			first = ret;
	}
	if (c->cache_enabled) {				/* 4. flush on-chip cache */
		ret = cl3a_cache_invalidate(c, idx, crc16);
		if (ret && !first)
			first = ret;
	}
	return first;
}

/* ------------------------------------------------------------------ */
/* 3g. AQM byte/packet MIB read (up to 2048 metered flows).           */
/*     MIB mode (AQM_SET.cnt_md=1): word0=byteCnt, word1=pktCnt[17:0]. */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_aqm_mib_get(struct cl3a_ctx *c, u32 idx, u64 *bytes, u32 *pkts)
{
	u32 words[CL3A_AQM_DATA_REGS];
	int ret, i;

	if (idx >= CL3A_AQM_MAX)
		return -EINVAL;

	ret = l3fe_access_go(c->ne, CL3A_CACHE_HASH_ACCESS,
			     (idx & 0x7ff) | CL3A_GO | CL3A_AQM_TBL_SEL,
			     CL3A_GO);				/* read, table=2 */
	if (ret)
		return ret;

	for (i = 0; i < CL3A_AQM_DATA_REGS; i++)
		words[i] = cl3a_rd(c, CL3A_CACHE_HASH_ACCESS + ((CL3A_AQM_DATA_REGS - i) << 2));

	if (bytes)
		*bytes = words[0];			/* byteCnt[31:0] */
	if (pkts)
		*pkts = words[1] & 0x3ffff;		/* pktCnt[17:0] */
	return 0;
}

/* ------------------------------------------------------------------ */
/* Aging-init — this is ENGINE INIT ONLY, and it is not the whole      */
/* story.  Writing 0 here matches what cortina_l3fe_engine_init() does */
/* at reset, but the shipping driver does NOT leave it at 0: it later  */
/* programs L3FE_AGING_GRAN_SLOW in cortina_l3fe_hw_l3_forward_enable()*/
/* because with granularity 0 the ager block never runs, the age slot  */
/* reads stale, and the HW hit re-arm can never be witnessed — which   */
/* would make the sweep above blind.  The slow cadence keeps idle       */
/* entries alive far longer than any test window, so hardware still    */
/* does not age a live flow out from under the Linux flowtable; that   */
/* is done by the cadence, NOT by leaving the ager off.                */
/* ------------------------------------------------------------------ */
static void __maybe_unused cl3a_aging_init(struct cl3a_ctx *c)
{
	cl3a_wr(c, L3FE_HS_AGING_GRANULARITY, 0);	/* auto age-countdown OFF */
}

/*
 * GC sweep (to be driven by a ~4s delayed_work in the wiring phase):
 *
 *   for each OCCUPIED bucket b (walk the SW shadow's per-bucket valid mask,
 *   NOT all 2048 blindly):
 *       cl3a_traffic_status_get(c, b << 5, &trf);
 *       for each valid slot k in b:
 *           if (trf & (1<<k)) entry[b*32+k]->last_hit = jiffies;   // fed to
 *                                                                  // FLOW_CLS_STATS
 *   Do NOT delete here on idle — report lastused and let the Linux flowtable
 *   teardown timer call cl3a_flow_del().  One indirect read+clear per 32 flows
 *   is what makes >10k flows affordable (2048 ops max vs 10k+ per-flow reads).
 */
