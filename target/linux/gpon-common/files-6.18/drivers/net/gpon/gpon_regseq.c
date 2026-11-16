// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE */
/*
 * gpon_regseq -- the interpreter.  See gpon_regseq.h for why it is core.
 */
#include <linux/errno.h>
#include <linux/kernel.h>

#include "gpon_regseq.h"

/* Read-modify-write bits [msb:lsb]. Exported because a caller ...
 * dev/MEASURED-gpon_regseq.c.md sec 1. */
void gpon_regseq_fld(const struct gpon_regseq_io *io, u32 addr,
		     u8 msb, u8 lsb, u32 val)
{
	io->wr(addr, gpon_field_set(io->rd(addr), msb, lsb, val));
}

/* A REP at seq[i]: its slots fit in the table and each is a WR, or a POLL whose budget does not step, then its STEP.
 * -> the ops it spans, 0 if malformed. */
static unsigned int rep_span(const struct gpon_regseq_op *seq, unsigned int i, unsigned int n)
{
	unsigned int k, slots = seq[i].val;

	if (!slots || slots > (n - i - 1) / 2)
		return 0;
	for (k = 0; k < slots; k++) {
		const struct gpon_regseq_op *op = &seq[i + 1 + 2 * k], *step = op + 1;

		if (step->opc != GPON_REGSEQ_STEP)
			return 0;
		if (op->opc != GPON_REGSEQ_WR && (op->opc != GPON_REGSEQ_POLL || step->val))
			return 0;
	}
	return 1 + 2 * slots;
}

static int poll_bit(const struct gpon_regseq_io *io, u32 addr, u8 bit, u32 tries)
{
	u32 k;

	for (k = 0; k < tries; k++) {
		if (io->rd(addr) & (1u << bit))
			return 0;
		io->delay_us(GPON_REGSEQ_POLL_US);
	}
	return -ETIMEDOUT;
}

/* Run one sequence, every address but a delay's moved by @base. -> 0, or -ETIMEDOUT naming nothing: the ...
 * dev/MEASURED-gpon_regseq.c.md sec 2. */
static int regseq_run(const struct gpon_regseq_io *io, u32 base, const struct gpon_regseq_op *seq, unsigned int n)
{
	unsigned int i, k, it, span;

	if (!io || !io->rd || !io->wr || !io->delay_ms || !io->delay_us)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		const struct gpon_regseq_op *p = &seq[i];

		switch (p->opc) {
		case GPON_REGSEQ_WR:
			io->wr(base + p->addr, p->val);
			break;
		case GPON_REGSEQ_FLD:
			gpon_regseq_fld(io, base + p->addr, p->msb, p->lsb, p->val);
			break;
		case GPON_REGSEQ_DLY:
			io->delay_ms(p->val);
			break;
		case GPON_REGSEQ_POLL:
			if (poll_bit(io, base + p->addr, p->lsb, p->val))
				return -ETIMEDOUT;
			break;
		case GPON_REGSEQ_REP:
			/* checked whole before its first write: a half-run repeat is half-configured silicon */
			span = rep_span(seq, i, n);
			if (!span)
				return -EINVAL;
			for (it = 0; it < p->addr; it++)
				for (k = i + 1; k < i + span; k += 2) {
					u32 addr = base + seq[k].addr + it * seq[k + 1].addr;

					if (seq[k].opc == GPON_REGSEQ_WR)
						io->wr(addr, seq[k].val + it * seq[k + 1].val);
					else if (poll_bit(io, addr, seq[k].lsb, seq[k].val))
						return -ETIMEDOUT;
				}
			i += span - 1;
			break;
		default:
			/* An unknown opcode, or a STEP outside its REP, is a TABLE
			 * defect, and running the rest of a bring-up whose meaning
			 * we did not understand is how half-configured silicon
			 * reaches a measurement. */
			return -EINVAL;
		}
	}
	return 0;
}

int gpon_regseq_run(const struct gpon_regseq_io *io,
		    const struct gpon_regseq_op *seq, unsigned int n)
{
	return regseq_run(io, 0, seq, n);
}

int gpon_regseq_run_at(const struct gpon_regseq_io *io, u32 base,
		       const struct gpon_regseq_op *seq, unsigned int n)
{
	return regseq_run(io, base, seq, n);
}
