// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gpon_regseq -- the interpreter.  See gpon_regseq.h for why it is core.
 */
#include <linux/errno.h>
#include <linux/kernel.h>

#include "gpon_regseq.h"

/*
 * Read-modify-write bits [msb:lsb].  Exported because a caller that needs ONE
 * field outside a sequence would otherwise write the mask arithmetic again --
 * and a mask computed twice is a mask that will differ once.
 *
 * ⚠ THE FULL-WORD CASE IS SEPARATE ON PURPOSE: `1u << 32` is undefined
 * behaviour, so msb=31,lsb=0 cannot go through the general formula.
 */
void gpon_regseq_fld(const struct gpon_regseq_io *io, u32 addr,
		     u8 msb, u8 lsb, u32 val)
{
	u32 mask = (msb == 31 && lsb == 0)
		 ? 0xffffffffu
		 : (((1u << (msb - lsb + 1)) - 1) << lsb);

	io->wr(addr, (io->rd(addr) & ~mask) | ((val << lsb) & mask));
}

/*
 * Run one sequence. -> 0, or -ETIMEDOUT naming nothing: the CALLER knows which
 * sequence it handed over and can say what timed out; this function cannot,
 * and inventing a message here would put a hardware name in the core.
 */
int gpon_regseq_run(const struct gpon_regseq_io *io,
		    const struct gpon_regseq_op *seq, unsigned int n)
{
	unsigned int i, k;

	if (!io || !io->rd || !io->wr || !io->delay_ms || !io->delay_us)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		const struct gpon_regseq_op *p = &seq[i];

		switch (p->opc) {
		case GPON_REGSEQ_WR:
			io->wr(p->addr, p->val);
			break;
		case GPON_REGSEQ_FLD:
			gpon_regseq_fld(io, p->addr, p->msb, p->lsb, p->val);
			break;
		case GPON_REGSEQ_DLY:
			io->delay_ms(p->val);
			break;
		case GPON_REGSEQ_POLL:
			for (k = 0; k < p->val; k++) {
				if (io->rd(p->addr) & (1u << p->lsb))
					break;
				io->delay_us(GPON_REGSEQ_POLL_US);
			}
			if (k == p->val)
				return -ETIMEDOUT;
			break;
		default:
			/* An unknown opcode is a TABLE defect, and running the
			 * rest of a bring-up whose meaning we did not
			 * understand is how half-configured silicon reaches a
			 * measurement. */
			return -EINVAL;
		}
	}
	return 0;
}
