/* SPDX-License-Identifier: GPL-2.0-only */
/* ONE owner for this driver's indirect transaction -- NI ...
 * dev/MEASURED-cortina-access.h.md sec 1. */
#ifndef _CORTINA_ACCESS_H
#define _CORTINA_ACCESS_H

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/processor.h>

#include "regtable.h"	/* the CORE's indirect-transaction poll */
#include "cortina-ni-regs.h"

/* The L2FE FDB engine INIT rebuilds the whole hash table, so it was given ten
 * times the poll interval and twice the budget of an ordinary transaction.
 * Both of its sites wrote the two numbers out by hand; they are named here so
 * the reason travels with them. */
#define CA_NI_FDB_INIT_POLL_US		10
#define CA_NI_FDB_INIT_POLL_TIMEOUT_US	20000

/* ca_ni_access_wait_paced - wait for an ALREADY-ISSUED ...
 * dev/MEASURED-cortina-access.h.md sec 2. */
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

/* ca_ni_access_go_paced - issue @val and wait for the block ...
 * dev/MEASURED-cortina-access.h.md sec 3. */
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

/* The OTHER pacing family: a bounded SPIN, counted in reads ...
 * dev/MEASURED-cortina-access.h.md sec 4. */
static inline void ca_pause_none(void) { }
static inline void ca_pause_relax(void) { cpu_relax(); }
static inline void ca_pause_udelay1(void) { udelay(1); }

/* ★ THE LOOP ITSELF LIVES IN THE CORE (2026-09-04). ...
 * dev/MEASURED-cortina-access.h.md sec 5. */
static inline u32 ca_hwio_rd(void *ctx, u32 off)
{
	return readl((void __iomem *)ctx + off);
}

static inline void ca_hwio_wr(void *ctx, u32 off, u32 val)
{
	writel(val, (void __iomem *)ctx + off);
}

static inline int ca_go_spin(void __iomem *reg, unsigned int tries,
			     void (*pause)(void))
{
	struct hwio io = { .rd = ca_hwio_rd, .wr = ca_hwio_wr,
			   .ctx = (void *)reg };

	/* the register IS the ctx here, so the offset within it is 0 -- and
	 * reg_make(0) says so explicitly, because a bare 0 is now UNSET. */
	return gpon_ind_poll(&io, reg_make(0), CA_NI_IND_ACCESS_GO, tries,
			     pause);
}

#endif /* _CORTINA_ACCESS_H */
