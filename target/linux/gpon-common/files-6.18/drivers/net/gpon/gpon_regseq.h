/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE (drivers/net/gpon) -- HW-DECOUPLED.  Builds on MIPS-BE, ARM64-LE
 * and x86.  No MMIO, no clock, no allocator: see "THE THREE TIERS" in
 * gpon_common.h.
 *
 * gpon_regseq -- a bring-up expressed as DATA, and the tiny interpreter that
 * runs it.
 *
 * ★ WHY THIS IS CORE AND NOT FAMILY.  The register ADDRESSES and VALUES of a
 * bring-up are silicon facts and stay with the silicon.  What runs them is not:
 * "write this word, then set these bits, then wait, then poll that bit until it
 * clears or give up" is the same algorithm on every PON MAC anyone has ever
 * shipped, and it was written once per family for no reason but that the
 * interpreter and the tables lived in one file.
 *
 * ★★ TIME IS AN EXPLICIT INPUT, and that is what made this movable at all.  The
 * family version called `mdelay()` and `udelay()` directly, which the core
 * forbids -- not as bureaucracy, but because a driver that sleeps cannot be run
 * on a host.  With the two delays as ops, the same interpreter that drives the
 * silicon can be driven by a FIXTURE clock in an offline test at thousands of
 * sequences per second, which is this project's stated preference over one
 * ~200 s board boot.
 *
 * ⚠ THE POLL BUDGET IS IN ITERATIONS, NOT MILLISECONDS, and it is the caller's.
 * A core that decided the timeout would be deciding a hardware fact it cannot
 * know; a family that decides it is stating one it measured.
 */
#ifndef GPON_REGSEQ_H
#define GPON_REGSEQ_H

#include <linux/types.h>

/* What a step does.  Kept small on purpose: an opcode nobody needs is an
 * opcode that will be used for something it does not mean. */
enum gpon_regseq_opc {
	GPON_REGSEQ_WR = 0,	/* write `val` to `addr`			*/
	GPON_REGSEQ_FLD,	/* read-modify-write bits [msb:lsb] = `val`	*/
	GPON_REGSEQ_DLY,	/* wait `val` milliseconds			*/
	GPON_REGSEQ_POLL,	/* poll bit `lsb` of `addr` up to `val` times	*/
};

struct gpon_regseq_op {
	u8  opc;
	u8  msb;
	u8  lsb;
	u32 addr;
	u32 val;
};

/*
 * The shell's whole contract.  Four function pointers and nothing else: no
 * device, no lock, no allocator.  `rd`/`wr` take an ABSOLUTE address because
 * the sequences are expressed that way in the silicon's own documentation --
 * translating them would invent a second numbering nobody could check against
 * the vendor's tables.
 */
/**
 * gpon_field_mask() - the mask for bits [@msb:@lsb]. -> u32
 *
 * ★★★ ONE OWNER since 2026-09-04. This arithmetic was written THREE times:
 * here (gpon_regseq_fld), in flowcore/hwio.h (hwio_rmw) and in
 * realtek-luna/luna_ponmac.h (luna_rfwr) -- and hwio.h's own comment already
 * said why that is dangerous: "both families have their own today and a field
 * written one bit wide too far is the class of bug that reads back correct and
 * behaves wrong".
 *
 * ★ IT LIVES IN gpon/ AND NOT IN flowcore/ FOR A MEASURED REASON. The include
 * direction is one-way: flowcore/Makefile carries
 * `-I$(srctree)/drivers/net/gpon` and gpon/Makefile carries nothing toward
 * flowcore -- verified with the compiler, not read off the Makefiles: a TU
 * including "gpon_regseq.h" with flowcore+gpon on -I compiles, and one
 * including "hwio.h" with only gpon on -I is a fatal error. So gpon is the
 * lower layer, and a helper both need can only live here.
 *
 * ⚠ THE FULL-WORD CASE IS SEPARATE ON PURPOSE, and all three copies had it:
 * `1u << 32` is undefined behaviour, so msb=31,lsb=0 cannot go through the
 * general formula. msb=31,lsb=0 is the ONLY pair that reaches width 32.
 *
 * ★ PROVEN EQUIVALENT before unifying: the three spellings were compiled side
 * by side and compared over every (msb,lsb) with msb>=lsb -- 528 pairs, 0
 * disagreements. This replaces them without changing a single computed mask.
 */
static inline u32 gpon_field_mask(u8 msb, u8 lsb)
{
	return (msb == 31 && lsb == 0)
	     ? 0xffffffffu
	     : (((1u << (msb - lsb + 1)) - 1u) << lsb);
}

struct gpon_regseq_io {
	u32  (*rd)(u32 addr);
	void (*wr)(u32 addr, u32 val);
	void (*delay_ms)(unsigned int ms);
	void (*delay_us)(unsigned int us);
};

/* How long one POLL step waits between reads.  A step's budget is a COUNT of
 * these, so a sequence author states "up to N tries" and the shell states how
 * long a try is. */
#define GPON_REGSEQ_POLL_US	200u

int gpon_regseq_run(const struct gpon_regseq_io *io,
		    const struct gpon_regseq_op *seq, unsigned int n);

void gpon_regseq_fld(const struct gpon_regseq_io *io, u32 addr,
		     u8 msb, u8 lsb, u32 val);

/* Step constructors, so a table reads as the bring-up and not as a struct. */
#define GPON_WR(a, v)		{ GPON_REGSEQ_WR,   0,   0,   (a), (v) }
#define GPON_FLD(a, m, l, v)	{ GPON_REGSEQ_FLD,  (m), (l), (a), (v) }
#define GPON_DLY(ms)		{ GPON_REGSEQ_DLY,  0,   0,   0,   (ms) }
#define GPON_POLL(a, bit, n)	{ GPON_REGSEQ_POLL, (bit), (bit), (a), (n) }

#endif /* GPON_REGSEQ_H */
