/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_core_recover.h -- a switch core reset as a recovery: traffic quiesced, the state snapshotted, the reset (it
 * returns only once the block may be touched again), the snapshot again, the SAME init steps open() runs, a last
 * snapshot, traffic resumed. Every step runs even after one failed, so what can come back does, and each failure
 * is named. The order lives here only; the driver gives the operations and holds its locks around the call.
 */
#ifndef ETH_CORE_RECOVER_H
#define ETH_CORE_RECOVER_H

#include <linux/types.h>

struct eth_init_step {
	const char *name;
	int (*run)(void *ctx);
};

struct eth_core_recover_ops {
	void (*quiesce)(void *ctx);
	void (*reset)(void *ctx);
	void (*resume)(void *ctx);
	const struct eth_init_step *steps;
	unsigned int n_steps;
};

enum { ETH_SNAP_BEFORE, ETH_SNAP_RESET, ETH_SNAP_REINIT };

/* -> a mask of the steps that failed (bit i = steps[i]), 0 when all came back. */
static inline u32 eth_init_steps_run(const struct eth_init_step *s, unsigned int n, void *ctx)
{
	unsigned int i;
	u32 failed = 0;

	for (i = 0; i < n && i < 32; i++)
		if (s[i].run(ctx))
			failed |= 1u << i;
	return failed;
}

/* `snap(sctx, ETH_SNAP_*)` compares the state around the reset; NULL = not compared. */
static inline u32 eth_core_recover(const struct eth_core_recover_ops *o, void *ctx,
				   void (*snap)(void *sctx, unsigned int which), void *sctx)
{
	u32 failed;

	o->quiesce(ctx);
	if (snap)
		snap(sctx, ETH_SNAP_BEFORE);
	o->reset(ctx);
	if (snap)
		snap(sctx, ETH_SNAP_RESET);
	failed = eth_init_steps_run(o->steps, o->n_steps, ctx);
	if (snap)
		snap(sctx, ETH_SNAP_REINIT);
	o->resume(ctx);
	return failed;
}

#endif
