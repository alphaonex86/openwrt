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

#endif /* _REGTABLE_H */
