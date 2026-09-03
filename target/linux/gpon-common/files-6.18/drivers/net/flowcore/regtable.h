/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * regtable.h -- the OTHER half of a one-hour port: offsets as DATA.
 *
 * ★★★ hwio.h answered "how do I reach a register". This answers "which
 * register", and the two together are what make the operator's target real
 * (2026-08-28: *"todo lo que puede permitir reducir el port time, tener mas
 * abstraction, codigo mas limpio mejor, y de paso le hace mas facil de hacer
 * fuzzing y test"*).
 *
 *     logic(hwio, regs)   <- one implementation, every chip
 *     regs                <- one C initialiser per chip. THE PORT.
 *     hwio                <- how the shell reaches that block on that board.
 *
 * ★ WHY A STRUCT AND NOT #defines, when this tree just spent effort splitting
 * 180 #defines into a per-chip header. Because a header is chosen at COMPILE
 * time: it gives one chip per object, so shared logic still has to be compiled
 * twice and can still diverge between the two builds. A struct is chosen at
 * RUN time by pointer, so ONE compiled function serves every chip -- and a
 * second chip becomes an initialiser a human can read and diff against the
 * first, which is what "a list of registers per board" actually means.
 *   Both forms are legitimate and this tree uses both: the compile-time header
 *   for a value in a hot path, the table for anything a shared function reads.
 *
 * ★ AND IT IS WHAT MAKES THE LOGIC FUZZABLE. A function taking (hwio, regs)
 * has no silicon in it: on x86 the test supplies an hwio backed by an array
 * and any table it likes, then asserts on what the function WROTE and WHERE.
 * That is thousands of cases a second against roughly one 200-second boot --
 * and it is why "more abstraction" and "easier testing" are not two goals.
 *
 * ★★ A ZERO IS NOT AN ADDRESS -- AND IT IS NOT THE ABSENCE MARKER EITHER.
 * Every table declares which fields it does not have; a chip that lacks a
 * block sets the field to REG_ABSENT and the logic must ASK rather than
 * write to offset 0 of something. reg_has() is that question, spelled once.
 * The sentinel is 0xffffffff and deliberately NOT 0, because "the register
 * at 0" is a real address on these parts (GPON_INT_DLT is 0x0000) and a
 * silent write there is exactly the class of bug that reads back fine.
 * (Until 2026-09-02 this paragraph said an absent field "leaves it 0" --
 * following that would have marked absence as an offset reg_has() calls
 * PRESENT, and the logic would have written to offset 0. The luna_sw_map
 * tables' 0-means-untouched convention is THEIRS, not this header's.)
 */
#ifndef _REGTABLE_H
#define _REGTABLE_H

#include <linux/errno.h>
#include <linux/types.h>

#include "hwio.h"	/* the injected accessor the logic below writes through */

/** Sentinel for "this chip does not have this register". */
#define REG_ABSENT	0xffffffffu

/** Is @off a register this chip actually has? */
static inline bool reg_has(u32 off)
{
	return off != REG_ABSENT;
}

/**
 * struct gpon_gtc_regs - the GTC/OMCI offsets a shared GPON function needs.
 *
 * Offsets are WITHIN the GTC block, never absolute: the base belongs to the
 * shell's hwio context. Names follow the silicon's own names, per this tree's
 * rule that a misleading name is a defect.
 *
 * ★ ADDING A CHIP IS ADDING ONE INITIALISER BELOW. That is the deliverable.
 */
struct gpon_gtc_regs {
	u32	ds_omci_pti;		/* [6:4] PTI_MASK, [2:0] END_PTI      */
	u32	gem_us_port_map;	/* array base; 32-bit words, stride 4 */
	u32	gem_us_port_stride;	/* the stride itself, in BYTES        */
	u32	gem_ds_mc_cfg;		/* broadcast / non-multicast / FCS    */
	u32	ds_traffic_cfg;		/* per-flow DS traffic config array   */
	u32	ds_traffic_stride;
	/* The two indirect GTC CAMs (gpon_gtc_cam_write() below).  Each is an
	 * op/index register plus a write-data register; the INDEX MASK is that
	 * CAM's own depth -- a per-CAM FACT (the GEM Port-ID CAM has 128 flows,
	 * OP_IDX[6:0]; the alloc CAM 32 T-CONTs, OP_IDX[4:0]) -- so it rides
	 * HERE with the registers, never as a magic number at a call site. */
	u32	ds_port_ind;		/* GEM Port-ID CAM op: OP_MODE[9:8] OP_IDX[6:0] */
	u32	ds_port_wr;		/* GEM Port-ID CAM write data: [11:0] gemPortId */
	u32	ds_port_idx_mask;	/* OP_IDX width as a mask (0x7f = 128 flows)    */
	u32	ds_alloc_ind;		/* T-CONT alloc CAM op: OP_IDX[4:0]             */
	u32	ds_alloc_wr;		/* alloc CAM write data: [11:0] allocateId      */
	u32	ds_alloc_idx_mask;	/* OP_IDX width as a mask (0x1f = 32 T-CONTs)   */
};

/**
 * struct gpon_chip - everything a shared GPON function needs to know about
 * one part. A port adds one of these.
 * @name: for messages only; never compared against.
 * @gtc:  the GTC/OMCI offsets.
 */
struct gpon_chip {
	const char		*name;
	struct gpon_gtc_regs	gtc;
};

/**
 * gpon_gtc_us_gem_stamp() - stamp a GEM Port-ID into one US port-map slot.
 * @io:      how to reach the GTC block (the shell's hwio over that block).
 * @r:       this chip's GTC offsets.
 * @flow:    the internal upstream flow/SID index (a slot of the port-map
 *           array; the caller owns the range check -- Luna pins its two flows
 *           with static_assert(GPON_GEM_US_RANGE_OK(...)) at compile time).
 * @port_id: the GEM Port-ID, ALREADY masked to its 12 on-wire bits --
 *           gpon_gem_us_port_id() (gpon_gem_us.h) is the one spelling of that
 *           mask, and this function writes exactly what it is handed.
 *
 * The FIRST logic(hwio, regs) function of this table -- the write that the
 * GEM_US_PORT_MAP stride regression (4 -> 0x20) once sent into a statistics
 * counter, leaving the real slot unmapped and the T-CONT silent ("Laser out").
 * The offset arithmetic now lives HERE, once, fed by per-chip DATA, and is
 * proven by a write-stream differential on x86
 * (dev/rtl9607c-test/gpon_regtable_diff_test) instead of by a boot.
 *
 * Return: true when the slot was written; false when this chip's table
 * declares no US port map (the reg_has() ask this header requires -- the
 * caller logs, because a chip without the array must never reach here).
 */
static inline bool gpon_gtc_us_gem_stamp(const struct hwio *io,
					 const struct gpon_gtc_regs *r,
					 u32 flow, u16 port_id)
{
	if (!reg_has(r->gem_us_port_map))
		return false;
	hwio_wr(io, r->gem_us_port_map + flow * r->gem_us_port_stride, port_id);
	return true;
}

/*
 * The indirect-CAM WRITE op word, shared by BOTH GTC CAMs and, per the vendor
 * chipdef, identical on the RTL9602C and the RTL9603CVD (PORTID_OP_* /
 * ALLOCID_OP_* fields of GPON_GTC_DS_PORT_IND 0x701100 / DS_ALLOC_IND
 * 0x7010C0: OP_MODE lsp 8 len 2, OP_REQ lsp 15, OP_COMPL lsp 14; both WR
 * registers carry WDATA lsp 0 len 12).  Only the OP_IDX width differs between
 * the two CAMs, which is why THAT is table data and these are not.
 */
#define GPON_GTC_CAM_OP_WRITE	(1u << 8)	/* OP_MODE[9:8] = WRITE(1) */
#define GPON_GTC_CAM_OP_REQ	(1u << 15)	/* trigger                 */
#define GPON_GTC_CAM_OP_COMPL	(1u << 14)	/* transaction complete    */
#define GPON_GTC_CAM_VAL_MASK	0xfffu		/* WDATA[11:0]             */
/* The poll budget of the pre-conversion call sites, verbatim: up to 1000
 * reads at 1 us apart.  All four converted sites used exactly this bound, so
 * it lives with the transaction; a chip that needs another budget makes it a
 * parameter THEN, not speculatively. */
#define GPON_GTC_CAM_TRIES	1000u

/**
 * gpon_gtc_cam_write() - one indirect GTC CAM write transaction.
 * @io:       how to reach the GTC block.
 * @ind:      the CAM's op/index register (offset within the block).
 * @wr:       the CAM's write-data register.
 * @idx_mask: the CAM's OP_IDX width as a mask -- a fact of THAT CAM's depth.
 * @idx:      the entry to write (flow / T-CONT).
 * @val:      the 12-bit value (GEM Port-ID / Alloc-ID); masked to WDATA[11:0]
 *            here, the same "& 0xfff" every pre-conversion site spelled.
 * @delay_us: the shell's sleep -- time is an EXPLICIT INPUT in this tier
 *            (the gpon_regseq_io precedent), so the same function runs on
 *            x86 under the write-stream differential with a counting fake.
 *
 * The sequence the four call sites in gpon-luna.c each spelled by hand until
 * 2026-09-03 -- and had already let diverge once: the multicast copy shipped
 * with NO completion check, so a timed-out CAM write fell through to "datapath
 * installed" while the GTC dropped the broadcast DS (the DHCP OFFER).  One
 * implementation is what stops that recurring.  Write the index, write the
 * value, re-write the index with REQ, poll for COMPL -- bounded, because a
 * timeout is not success.
 *
 * Return: the poll iteration at which COMPL was seen (>= 0; the T-CONT bind
 * logs it), -ETIMEDOUT when the budget expires, -ENODEV when this chip's
 * table declares no such CAM, -EINVAL with no delay op.  Proven against the
 * pre-conversion macro form on address, value, write count and read count by
 * dev/rtl9607c-test/gpon_regtable_diff_test.
 */
static inline int gpon_gtc_cam_write(const struct hwio *io,
				     u32 ind, u32 wr, u32 idx_mask,
				     u32 idx, u16 val,
				     void (*delay_us)(unsigned int us))
{
	u32 op = GPON_GTC_CAM_OP_WRITE | (idx & idx_mask);
	unsigned int i;

	if (!delay_us)
		return -EINVAL;
	if (!reg_has(ind) || !reg_has(wr))
		return -ENODEV;
	hwio_wr(io, ind, op);
	hwio_wr(io, wr, val & GPON_GTC_CAM_VAL_MASK);
	hwio_wr(io, ind, op | GPON_GTC_CAM_OP_REQ);
	for (i = 0; i < GPON_GTC_CAM_TRIES; i++) {
		if (hwio_rd(io, ind) & GPON_GTC_CAM_OP_COMPL)
			return (int)i;
		delay_us(1);
	}
	return -ETIMEDOUT;
}

/** Write @gem_port into entry @flow of the DS GEM Port-ID CAM. */
static inline int gpon_gtc_ds_port_write(const struct hwio *io,
					 const struct gpon_gtc_regs *r,
					 u32 flow, u16 gem_port,
					 void (*delay_us)(unsigned int us))
{
	return gpon_gtc_cam_write(io, r->ds_port_ind, r->ds_port_wr,
				  r->ds_port_idx_mask, flow, gem_port,
				  delay_us);
}

/** Write @alloc into entry @tcont of the T-CONT alloc CAM. */
static inline int gpon_gtc_ds_alloc_write(const struct hwio *io,
					  const struct gpon_gtc_regs *r,
					  u32 tcont, u16 alloc,
					  void (*delay_us)(unsigned int us))
{
	return gpon_gtc_cam_write(io, r->ds_alloc_ind, r->ds_alloc_wr,
				  r->ds_alloc_idx_mask, tcont, alloc,
				  delay_us);
}

#endif /* _REGTABLE_H */
