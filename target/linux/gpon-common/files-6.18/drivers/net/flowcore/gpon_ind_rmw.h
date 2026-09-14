/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_ind_rmw.h -- write ONE entry of an indirect ACCESS/DATA table, from the
 * core, through struct hwio: as a read-modify-write (gpon_ind_rmw) or as a
 * whole word (gpon_ind_set).
 *
 * TIER: flowcore -- hardware-decoupled (see gpon_common.h for the tier rule).
 *
 * THE HANDSHAKE, for every entry of a table behind a self-clearing GO bit:
 * write the index into ACCESS with GO (a READ transaction), wait for the
 * hardware to drop GO, take the entry out of DATA, put the new word back, write
 * ACCESS again with GO plus the WRITE-direction bit, wait once more.  The poll
 * half is gpon_ind_go() (regtable.h); this is the sequence around it, and it
 * carries the ONE decision the sequence has: ⚠ A READ HALF THAT NEVER RELEASED
 * GO MUST STOP THE WRITE HALF, because DATA then still holds whatever the
 * PREVIOUS transaction left there and writing it back commits a stale entry.
 *
 * ★ TWO SPELLINGS OF THE MIDDLE, AND THEY ARE NOT INTERCHANGEABLE.  A caller
 *   that owns only SOME bits reads DATA and puts back (DATA & ~clr) | set --
 *   gpon_ind_rmw(); a caller that owns the WHOLE word writes it with no read --
 *   gpon_ind_set().  Routing a whole-word site through the rmw form ADDS a read
 *   transaction to a live GPON MAC's bus stream; routing an rmw site through
 *   the set form ZEROES the bits it never owned.  Both halves are shared
 *   (gpon_ind_fetch / gpon_ind_commit); dev/rtl9607c-test/gpon_ind_diff_test
 *   holds each against the pre-conversion shell form on address, value, write
 *   count, read count and pause count.
 *
 * ★ THIS IS NOT gpon_gtc_cam_xact().  That engine is the Luna GTC CAM: an
 *   OP_MODE field, a REQ bit the caller raises, a COMPL bit the hardware SETS.
 *   Here the trigger is a GO bit the hardware CLEARS and the direction is a
 *   second bit in the same word -- the Cortina GPON-MAC shape.  Two handshakes,
 *   two engines, one poll; routing one through the other changes the bus stream.
 *
 * ★ WHY (clr, set) AND NOT hwio_rmw()'s ONE FIELD: the callers clear bits they
 *   do not set and set bits they do not clear (invalidate clears en|en|index and
 *   sets nothing; bind clears index and sets en|en|index).  A field form over
 *   one contiguous span lands the same word for today's masks by accident, and
 *   stops being correct the first time a caller clears a bit outside the span.
 *
 * ⚠ IT BELONGS IN regtable.h, beside gpon_ind_go(): it is here only because the
 *   briefs that produced it (2026-09-05) allowed one new core file and no edit
 *   to an existing one.  Fold it in; nothing depends on the split.
 */
#ifndef _GPON_IND_RMW_H
#define _GPON_IND_RMW_H

#include <linux/errno.h>
#include <linux/types.h>
#include "regtable.h"	/* gpon_ind_go(), reg_has(), struct hwio */

/**
 * struct gpon_ind_tbl - one indirect ACCESS/DATA table, as offsets WITHIN its
 * block (the block base is the hwio's ctx, per hwio.h).
 * @access: the ACCESS register: the entry index in its low bits, @go and @wr
 *          above them.
 * @data:   the DATA register the entry is exchanged through.
 * @go:     the trigger -- SET by the caller, CLEARED by the hardware when the
 *          transaction is done; the poll watches it.
 * @wr:     the direction -- set with @go for a WRITE, clear for a READ.
 * @tries:  the poll bound of each half.  A timeout is not success, so there is
 *          always one. The same bound applies before submission while a
 *          previous request still owns ACCESS and DATA.
 *
 * ★ ADDING A TABLE IS ADDING ONE INITIALISER IN THE SHELL.  The offsets come
 * from that chip's own register facts, never from a literal here.
 */
struct gpon_ind_tbl {
	u32		access;
	u32		data;
	u32		go;
	u32		wr;
	unsigned int	tries;
};

/**
 * gpon_ind_fetch() - the READ half: load entry @idx into DATA.
 * @idx:   ALREADY masked to the ACCESS index width by the caller
 *         (gpon_gem_us_alloc_id() / gpon_gem_us_index() are those spellings);
 *         this writes exactly what it is handed.
 * @pause: the shell's per-iteration pause -- the core has no clock, so the
 *         wait's cost is the shell's to define.
 * @stuck: out, optional -- on -ETIMEDOUT, the ACCESS word (with @go) the
 *         hardware never released. Zero when an older request prevented any
 *         new submission. Not written on success or argument refusal.
 *
 * Refuses BOTH registers before any bus traffic, not just ACCESS: the only
 * reason to fetch is to touch DATA next, so a chip whose table lacks DATA must
 * not have emitted the read transaction first.
 *
 * Return: 0; -ETIMEDOUT; -ENODEV when this chip's table has no such register;
 * -EINVAL with no pause op.  The last two are refused BEFORE any bus traffic.
 */
static inline int gpon_ind_fetch(const struct hwio *io,
				 const struct gpon_ind_tbl *t, u32 idx,
				 void (*pause)(void), u32 *stuck)
{
	u32 cmd = t->go | idx;
	int rc;
	if (!pause)
		return -EINVAL;
	if (!reg_has(t->access) || !reg_has(t->data))
		return -ENODEV;

	/* Retire a previous request before changing ACCESS or staging DATA.
	 * The caller serializes the complete transaction against other users. */
	rc = gpon_ind_poll(io, t->access, t->go, t->tries, pause);
	if (rc < 0) {
		if (stuck)
			*stuck = 0; /* This operation submitted no command. */
		return rc;
	}
	rc = gpon_ind_go(io, t->access, cmd, t->go, t->tries, pause);
	if (rc < 0) {
		if (stuck)
			*stuck = cmd;
		return rc;
	}
	return 0;
}

/**
 * gpon_ind_commit() - the WRITE half: DATA = @word, then the WRITE transaction.
 * @stuck: as gpon_ind_fetch(), plus @wr -- (*stuck & t->wr) says the WRITE half.
 *
 * ⚠ Only meaningful after a gpon_ind_fetch() of the same @idx that returned 0;
 * the two callers below enforce that order.  A commit after a timed-out fetch
 * would push whatever the previous transaction left in DATA, which is the one
 * decision this header exists to carry.
 *
 * Return: as gpon_ind_fetch().
 */
static inline int gpon_ind_commit(const struct hwio *io,
				  const struct gpon_ind_tbl *t, u32 idx,
				  u32 word, void (*pause)(void), u32 *stuck)
{
	u32 cmd = t->go | t->wr | idx;
	int rc;
	if (!pause)
		return -EINVAL;
	if (!reg_has(t->access) || !reg_has(t->data))
		return -ENODEV;

	/* Retire a previous request before changing ACCESS or staging DATA.
	 * The caller serializes the complete transaction against other users. */
	rc = gpon_ind_poll(io, t->access, t->go, t->tries, pause);
	if (rc < 0) {
		if (stuck)
			*stuck = 0; /* This operation submitted no command. */
		return rc;
	}
	hwio_wr(io, t->data, word);
	rc = gpon_ind_go(io, t->access, cmd, t->go, t->tries, pause);
	if (rc < 0) {
		if (stuck)
			*stuck = cmd;
		return rc;
	}
	return 0;
}

/**
 * gpon_ind_rmw() - entry @idx: DATA = (DATA & ~@clr) | @set, both halves polled.
 *
 * The stream: wait for idle, ACCESS(go|idx), poll, READ DATA, wait for idle,
 * WRITE DATA, ACCESS(go|wr|idx), poll. The WRITE half is never issued after a
 * timed-out READ half. An idle preflight adds one ACCESS read per phase;
 * DATA reads, write order and post-submission polling remain unchanged.
 *
 * Return: as gpon_ind_fetch(), with @stuck saying which half timed out.
 */
static inline int gpon_ind_rmw(const struct hwio *io,
			       const struct gpon_ind_tbl *t, u32 idx,
			       u32 clr, u32 set, void (*pause)(void),
			       u32 *stuck)
{
	int rc = gpon_ind_fetch(io, t, idx, pause, stuck);
	if (rc < 0)
		return rc;
	return gpon_ind_commit(io, t, idx,
			       (hwio_rd(io, t->data) & ~clr) | set, pause,
			       stuck);
}

/**
 * gpon_ind_set() - entry @idx: DATA = @word, both halves polled, NO DATA read.
 * @word:  the whole entry word; the caller owns every bit of it.
 *
 * The stream: gpon_ind_rmw() minus the one read -- exactly the pre-conversion
 * shape of cortina-gpon.c's cg_ds_gem_set() and cg_data_teardown()'s
 * stamp-clearing loop.
 * ⚠ THE READ TRANSACTION IS KEPT ALTHOUGH ITS RESULT IS DISCARDED: that is what
 * the shell has always emitted, and this header preserves bus streams, it does
 * not tidy them.
 *
 * Return: as gpon_ind_rmw().
 */
static inline int gpon_ind_set(const struct hwio *io,
			       const struct gpon_ind_tbl *t, u32 idx,
			       u32 word, void (*pause)(void), u32 *stuck)
{
	int rc = gpon_ind_fetch(io, t, idx, pause, stuck);
	if (rc < 0)
		return rc;
	return gpon_ind_commit(io, t, idx, word, pause, stuck);
}

#endif /* _GPON_IND_RMW_H */
