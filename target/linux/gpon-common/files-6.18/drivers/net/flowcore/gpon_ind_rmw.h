/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_ind_rmw.h -- read-modify-write ONE entry of an indirect ACCESS/DATA
 * table, from the core, through struct hwio.
 *
 * TIER: flowcore -- hardware-decoupled (see gpon_common.h for the tier rule).
 *
 * THE HANDSHAKE.  An indirect table behind a self-clearing GO bit is reached the
 * same way for every entry: write the entry's index into ACCESS with GO set (a
 * READ transaction), wait for the hardware to drop GO, take the entry out of
 * DATA, put the new word back into DATA, write ACCESS again with GO plus the
 * WRITE-direction bit, and wait once more.  The poll half of that is
 * gpon_ind_go() (regtable.h); this is the two-transaction sequence around it,
 * carrying the one decision the sequence has: a READ half that never released
 * GO must stop the WRITE half, because DATA then holds whatever the previous
 * transaction left there and writing it back would commit a stale entry.
 *
 * ★ THIS IS NOT gpon_gtc_cam_xact().  That engine is the Luna GTC CAM: an
 * OP_MODE field, a REQ bit the caller raises and a COMPL bit the hardware SETS.
 * Here the trigger is a GO bit the hardware CLEARS and the direction is a second
 * bit in the same word -- the Cortina GPON-MAC shape (TCONT_ACCESS/_DATA,
 * US_PORT_ACCESS/_DATA, DS_GEM_ACCESS/_DATA).  Two handshakes, two engines,
 * one poll.  Routing one through the other would change the bus stream.
 *
 * ★ WHY (clr, set) AND NOT hwio_rmw()'s ONE FIELD.  The callers clear bits they
 * do not set and set bits they do not clear (invalidate: clear en|en|index,
 * set nothing; bind: clear index, set en|en|index).  A field form over one
 * contiguous span lands the same word for today's masks by accident and stops
 * being correct the first time a caller clears a bit outside the span.
 *
 * ★ WHY A SEPARATE HEADER AND NOT regtable.h, where gpon_ind_go() lives and
 * where this belongs: the brief that produced this conversion (2026-09-05)
 * allowed ONE new core file and no edit to an existing one, because other
 * files were under concurrent edit.  Fold it in; nothing depends on the split.
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
 * @go:     the trigger bit -- SET by the caller, CLEARED by the hardware when
 *          the transaction is done; the poll watches it.
 * @wr:     the direction bit -- set together with @go for a WRITE transaction,
 *          clear for a READ.
 * @tries:  the poll bound of each half.  A timeout is not success, so there is
 *          always one.
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
 * gpon_ind_rmw() - entry @idx: DATA = (DATA & ~@clr) | @set, both halves polled.
 * @io:    how to reach the block.
 * @t:     the table's registers and bits.
 * @idx:   the entry, ALREADY masked to the ACCESS register's index width by the
 *         caller (gpon_gem_us_alloc_id() / gpon_gem_us_index() are those
 *         spellings); this function writes exactly what it is handed.
 * @clr:   the bits to clear in the entry word.
 * @set:   the bits to set, after clearing.
 * @pause: the shell's per-iteration pause -- gpon_ind_poll()'s contract: the
 *         core has no clock, so the wait's cost is the shell's to define.
 * @stuck: out, optional -- on -ETIMEDOUT, the ACCESS word (with @go) that the
 *         hardware never released, so the shell can log WHICH half failed and
 *         with what: (*stuck & t->wr) says the WRITE half.  Not written on
 *         success or on a refusal.
 *
 * Return: 0 on success; -ETIMEDOUT when either half never released @go (the
 * WRITE half is never issued after a timed-out READ half); -ENODEV when this
 * chip's table has no such register, refused BEFORE any bus traffic; -EINVAL
 * with no pause op.
 */
static inline int gpon_ind_rmw(const struct hwio *io,
			       const struct gpon_ind_tbl *t, u32 idx,
			       u32 clr, u32 set, void (*pause)(void),
			       u32 *stuck)
{
	u32 cmd = t->go | idx;
	int rc;

	if (!pause)
		return -EINVAL;
	if (!reg_has(t->access) || !reg_has(t->data))
		return -ENODEV;
	rc = gpon_ind_go(io, t->access, cmd, t->go, t->tries, pause);
	if (rc < 0)
		goto timed_out;
	hwio_wr(io, t->data, (hwio_rd(io, t->data) & ~clr) | set);
	cmd |= t->wr;
	rc = gpon_ind_go(io, t->access, cmd, t->go, t->tries, pause);
	if (rc < 0)
		goto timed_out;
	return 0;

timed_out:
	if (stuck)
		*stuck = cmd;
	return rc;
}

#endif /* _GPON_IND_RMW_H */
