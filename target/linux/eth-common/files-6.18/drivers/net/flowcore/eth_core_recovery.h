/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_core_recovery.h -- eth_core_recover() as a driver runs it: asked from any context (a watch timer, a knob),
 * run once at a time from a work item. The driver's begin() takes its locks (false: nothing to recover now), its
 * blocks() names the switch state it owns; this compares that state before the reset, after it and after the
 * re-init, and names in the log what the reset cleared, every step that failed and every word not restored.
 */
#ifndef ETH_CORE_RECOVERY_H
#define ETH_CORE_RECOVERY_H

#include <linux/bitops.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include "eth_core_recover.h"
#include "eth_regs_dump.h"

#define ETH_STATE_BLOCKS	16

struct eth_core_recovery {
	struct work_struct work;
	struct net_device *ndev;
	void *ctx;
	const struct eth_core_recover_ops *ops;
	bool (*begin)(void *ctx);
	void (*end)(void *ctx);
	unsigned int (*blocks)(void *ctx, struct eth_state_block *b);	/* rebuilt for each snapshot */
	struct eth_state_block b[ETH_STATE_BLOCKS];
	unsigned int n, words;
	u32 *before, *now;
	unsigned long busy;
	u32 runs, failed, unrestored;
};

static inline void eth_core_recovery_snap(void *sctx, unsigned int which)
{
	struct eth_core_recovery *r = sctx;

	if (!r->before || eth_state_words(r->b, r->n = r->blocks(r->ctx, r->b)) != r->words)
		return;
	eth_state_snap(r->b, r->n, which == ETH_SNAP_BEFORE ? r->before : r->now);
	if (which == ETH_SNAP_RESET)
		eth_state_log_delta(r->ndev, "core reset", r->b, r->n, r->before, r->now);
	else if (which == ETH_SNAP_REINIT)
		r->unrestored = eth_state_log_delta(r->ndev, "re-init", r->b, r->n, r->before, r->now);
}

static inline void eth_core_recovery_work(struct work_struct *work)
{
	struct eth_core_recovery *r = container_of(work, struct eth_core_recovery, work);
	unsigned int i;
	u32 failed;

	r->words = r->blocks ? eth_state_words(r->b, r->n = r->blocks(r->ctx, r->b)) : 0;
	r->before = r->words ? vmalloc(array_size(r->words, sizeof(u32))) : NULL;
	r->now = r->words ? vmalloc(array_size(r->words, sizeof(u32))) : NULL;
	if (!r->before || !r->now) {
		netdev_err(r->ndev, "switch core reset: the state is not compared (%u words, no memory)\n", r->words);
		vfree(r->before);
		vfree(r->now);
		r->before = r->now = NULL;
	}
	if (r->begin(r->ctx)) {
		r->unrestored = 0;
		failed = eth_core_recover(r->ops, r->ctx, r->before ? eth_core_recovery_snap : NULL, r);
		r->runs++;
		r->failed |= failed;
		for (i = 0; i < r->ops->n_steps; i++)
			if (failed & BIT(i))
				netdev_err(r->ndev, "switch core reset #%u: re-init step '%s' FAILED: the switch runs degraded\n",
					   r->runs, r->ops->steps[i].name);
		netdev_err(r->ndev, "switch core reset #%u done: %u step(s) failed, %s\n", r->runs, hweight32(failed),
			   !r->before ? "state not compared" : r->unrestored ?
			   "words NOT restored (re-init lines above)" : "the state we own is back");
		r->end(r->ctx);
	}
	vfree(r->before);
	vfree(r->now);
	r->before = r->now = NULL;
	clear_bit(0, &r->busy);
}

static inline void eth_core_recovery_init(struct eth_core_recovery *r, struct net_device *ndev, void *ctx,
					  const struct eth_core_recover_ops *ops, bool (*begin)(void *ctx),
					  void (*end)(void *ctx),
					  unsigned int (*blocks)(void *ctx, struct eth_state_block *b))
{
	INIT_WORK(&r->work, eth_core_recovery_work);
	r->ndev = ndev;
	r->ctx = ctx;
	r->ops = ops;
	r->begin = begin;
	r->end = end;
	r->blocks = blocks;
}

/* -> false when one is already asked or running */
static inline bool eth_core_recovery_ask(struct eth_core_recovery *r)
{
	if (test_and_set_bit(0, &r->busy))
		return false;
	schedule_work(&r->work);
	return true;
}

static inline bool eth_core_recovery_busy(const struct eth_core_recovery *r)
{
	return test_bit(0, &r->busy);
}

#endif
