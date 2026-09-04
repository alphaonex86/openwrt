/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ONE owner for the Cortina NI's indirect transaction.
 *
 * ★ WHY THIS FILE EXISTS (2026-09-04).  Every indirect block in this driver
 * drives the SAME handshake: write a command word into that block's ACCESS
 * register, then re-read the same register until the hardware clears its GO
 * bit.  It was spelled by hand in eleven places under nine different names --
 * PLE, L2FE_FDB, L2FE_REDIR, DMA_LSO_BD, DMA_AFT, QM_FBM_CPU_CMD, QM_AXI_ATTR,
 * MIB, MDIO -- and the tree already proves they are one idea rather than nine:
 * cortina-ni-regs.h defines every one of those _GO names as an alias of
 * CA_NI_IND_ACCESS_GO.  The busy bit is therefore NOT a parameter here.  It is
 * a fact.
 *
 * ★ WHAT IS A PARAMETER IS THE PACE, because that is what the sites actually
 * disagreed on and nothing in the tree says which disagreement is a
 * requirement:
 *
 *     CA_NI_ACCESS_*        1 us / 10 ms   -- eight sites, the NI default
 *     CA_NI_FDB_INIT_*     10 us / 20 ms   -- the L2FE FDB engine INIT, which
 *                                             clears the whole table and was
 *                                             given a longer budget for it
 *
 * The MIB block keeps its own declared pace and passes it in.
 *
 * ★ AND IT IS DELIBERATELY *NOT* THE CORE'S gpon_ind_poll() YET.  The core
 * engine (flowcore/regtable.h) is the right destination and cortina-l3fe-regs.h
 * already reaches it, because that path polls with cpu_relax() under a
 * spinlock.  These sites poll through readl_poll_timeout(), which SLEEPS.
 * Swapping a sleeping poll for a spinning one changes how the CPU behaves while
 * the hardware works, on a bench that cannot cold-boot -- so the copies are
 * removed first, with the timing byte-for-byte unchanged, and the move to the
 * core stays a separate, measurable step.
 */
#ifndef _CORTINA_NI_ACCESS_H
#define _CORTINA_NI_ACCESS_H

#include <linux/io.h>
#include <linux/iopoll.h>

#include "cortina-ni-regs.h"

/* The L2FE FDB engine INIT rebuilds the whole hash table, so it was given ten
 * times the poll interval and twice the budget of an ordinary transaction.
 * Both of its sites wrote the two numbers out by hand; they are named here so
 * the reason travels with them. */
#define CA_NI_FDB_INIT_POLL_US		10
#define CA_NI_FDB_INIT_POLL_TIMEOUT_US	20000

/**
 * ca_ni_access_wait_paced - wait for an ALREADY-ISSUED transaction to finish
 * @acc: the block's ACCESS register
 * @out: receives the last word read (for the caller's diagnostic)
 * @us: poll interval
 * @timeout_us: total budget
 *
 * Issues nothing.  The PLE default-forward path opens with a bare idle-wait,
 * and routing that through a _go() would emit an ACCESS write the hardware
 * never saw.
 */
static inline int ca_ni_access_wait_paced(void __iomem *acc, u32 *out,
					  unsigned int us,
					  unsigned int timeout_us)
{
	u32 v;
	int ret = readl_poll_timeout(acc, v, !(v & CA_NI_IND_ACCESS_GO),
				     us, timeout_us);

	if (out)
		*out = v;
	return ret;
}

static inline int ca_ni_access_wait(void __iomem *acc, u32 *out)
{
	return ca_ni_access_wait_paced(acc, out, CA_NI_TX_POLL_US,
				       CA_NI_TX_POLL_TIMEOUT_US);
}

/**
 * ca_ni_access_go_paced - issue @val and wait for the block to clear GO
 *
 * @val carries GO plus whatever that block's command word needs; the caller
 * composes it, because the payload is the one thing that really does differ
 * per block (an index here, an opcode there, an index plus a write bit).
 */
static inline int ca_ni_access_go_paced(void __iomem *acc, u32 val, u32 *out,
					unsigned int us, unsigned int timeout_us)
{
	writel(val, acc);
	return ca_ni_access_wait_paced(acc, out, us, timeout_us);
}

static inline int ca_ni_access_go(void __iomem *acc, u32 val, u32 *out)
{
	return ca_ni_access_go_paced(acc, val, out, CA_NI_TX_POLL_US,
				     CA_NI_TX_POLL_TIMEOUT_US);
}

#endif /* _CORTINA_NI_ACCESS_H */
