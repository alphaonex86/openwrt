// SPDX-License-Identifier: GPL-2.0-only
/* cortina-l3fe-aging.c — Cortina CA8277C (RTL9607F "Elnath") ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 1. */

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

/* NE register base + L3FE hash-engine (HS) aging/cache ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 2. */
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

/* geometry --- ★★ TWO HARDWARE FACTS, TWO NAMES — until ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 3. */
#define CL3A_ENTRIES			65536
#define CL3A_BUCKETS			(CL3A_ENTRIES / CN_L3E_AGE_SLOTS)	/* 2048 */
#define CL3A_AQM_MAX			2048	/* L3FE_AQM_FLOW_STAT_MAX */

/* 2-bit age codes (Elnath 07f) --- These four have the same ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 4. */
#define CL3A_AGE_FREE			0	/* aged-out / invalid, stops matching */
#define CL3A_AGE_IDLE			1	/* keepalive floor written by the sweep */
#define CL3A_AGE_START			2	/* on add; HW re-arms here on a hit */
/* 3 = STATIC. What this line used to say — "never ages out" — ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 12. */
#define CL3A_AGE_STATIC			3

struct cl3a_ctx {
	void __iomem *ne;		/* ioremap of NE_REG_BASE */
	bool cache_enabled;		/* on-chip action cache in use */
};

/* Indirect-access helpers — every op is a bounded poll on the ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 5. */
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

/* 3c. Batch had-traffic read+clear of ONE 32-entry bucket. ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 6. */
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

			/* ★ STATIC(3) IS TREATED AS TRAFFIC AND STEPPED DOWN TO IDLE ...
			 * dev/MEASURED-cortina-l3fe-aging.c.md sec 7. */
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

	/* ★ THE SLOT IS THE WAY WITHIN THE 8-WAY HASH BUCKET, and ...
	 * dev/MEASURED-cortina-l3fe-aging.c.md sec 8. */
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

/* 3d. Flow delete — the vendor order ... -- dev/MEASURED-cortina-l3fe-aging.c.md sec 9. */
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

/* Aging-init — this is ENGINE INIT ONLY, and it is not the ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 10. */
static void __maybe_unused cl3a_aging_init(struct cl3a_ctx *c)
{
	cl3a_wr(c, L3FE_HS_AGING_GRANULARITY, 0);	/* auto age-countdown OFF */
}

/* GC sweep (to be driven by a ~4s delayed_work in the wiring ...
 * dev/MEASURED-cortina-l3fe-aging.c.md sec 11. */
