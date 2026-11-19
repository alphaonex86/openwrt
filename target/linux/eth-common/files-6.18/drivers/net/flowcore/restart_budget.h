/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* restart_budget.h -- ONE budget for every self-restart of our code (a ring, a port, the fibre path, a radio).
 * A restart after which the subsystem worked again cured it and spends nothing; RESTART_TRIES in a row that
 * did not are the last until it asks for none during RESTART_CURED_MS. What a spent budget means is the
 * caller's: a radio stays off and says so, a watch follows eth_watch_spent.h.
 * PURE: time is an input, in ms.
 */
#ifndef _RESTART_BUDGET_H
#define _RESTART_BUDGET_H

#include <linux/types.h>

#define RESTART_TRIES		3
#define RESTART_CURED_MS	600000

struct restart_budget {
	u64 asked_ms;		/* the last restart asked; 0 = never */
	u32 in_row;		/* restarts in a row that did not cure */
	u32 total;		/* restarts done since boot */
};

/* The subsystem worked since its last restart: that restart cured it. */
static inline void restart_budget_cured(struct restart_budget *b)
{
	b->in_row = 0;
}

/* -> true: restart now (counted); false: the budget is spent. `first_spent` (may be NULL) is set on the first
 * refusal only, so the caller says it once. */
static inline bool restart_budget_spend(struct restart_budget *b, u64 now_ms, bool *first_spent)
{
	if (b->asked_ms && now_ms - b->asked_ms >= RESTART_CURED_MS)
		b->in_row = 0;
	b->asked_ms = now_ms ? now_ms : 1;
	if (first_spent)
		*first_spent = b->in_row == RESTART_TRIES;
	if (b->in_row >= RESTART_TRIES) {
		b->in_row = RESTART_TRIES + 1;
		return false;
	}
	b->in_row++;
	b->total++;
	return true;
}
#endif
