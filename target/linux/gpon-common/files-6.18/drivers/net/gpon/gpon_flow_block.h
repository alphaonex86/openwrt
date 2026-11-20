/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE */
#ifndef GPON_FLOW_BLOCK_H
#define GPON_FLOW_BLOCK_H

#include <linux/mutex.h>
#include <linux/slab.h>
#include <net/pkt_cls.h>

#include "gpon_flow_offload.h"

static inline void gpon_flow_bound_release(void *cb_priv)
{
	kfree(cb_priv);
}

/* The netdev a block cb was bound on, from its cb_priv. */
static inline struct net_device *gpon_flow_bound_dev(void *cb_priv)
{
	return ((struct gpon_flow_bound *)cb_priv)->dev;
}

/* Bind per block: fw4 probes a temporary table while the live table exists. `fo` is retired of the block's
 * flows on its last unbind, under the family's `lock` (the one its block cb holds). */
static inline int gpon_flow_block_setup(struct net_device *dev,
				       struct flow_block_offload *f,
				       struct list_head *blocks,
				       flow_setup_cb_t *cb, u32 *binds,
				       struct gpon_flow_offload *fo, struct mutex *lock)
{
	struct flow_block_cb *block_cb;
	struct gpon_flow_bound *bound;
	int err = -EOPNOTSUPP;

	if (f->binder_type != FLOW_BLOCK_BINDER_TYPE_CLSACT_INGRESS)
		goto out;
	f->driver_block_list = blocks;

	switch (f->command) {
	case FLOW_BLOCK_BIND:
		block_cb = flow_block_cb_lookup(f->block, cb, dev);
		if (block_cb) {
			flow_block_cb_incref(block_cb);
			break;
		}
		bound = kzalloc(sizeof(*bound), GFP_KERNEL);
		if (!bound) {
			err = -ENOMEM;
			goto out;
		}
		bound->dev = dev;
		bound->block = f->block;
		block_cb = flow_block_cb_alloc(cb, dev, bound, gpon_flow_bound_release);
		if (IS_ERR(block_cb)) {
			kfree(bound);
			err = PTR_ERR(block_cb);
			goto out;
		}
		flow_block_cb_incref(block_cb);
		flow_block_cb_add(block_cb, f);
		list_add_tail(&block_cb->driver_list, blocks);
		if (binds)
			(*binds)++;
		break;
	case FLOW_BLOCK_UNBIND:
		block_cb = flow_block_cb_lookup(f->block, cb, dev);
		if (!block_cb) {
			err = -ENOENT;
			goto out;
		}
		if (!flow_block_cb_decref(block_cb)) {
			int left;

			flow_block_cb_remove(block_cb, f);
			list_del(&block_cb->driver_list);
			mutex_lock(lock);
			left = gpon_flow_offload_unbind(fo, f->block);
			mutex_unlock(lock);
			if (left)
				netdev_err(dev, "flow-block: unbind kept flows the engine would not retire (%d)\n",
					   left);
		}
		break;
	default:
		goto out;
	}
	err = 0;
out:
#if IS_ENABLED(CONFIG_GPON_FLOW_DIAG)
	netdev_info(dev, "flow-block: command=%u binder=%u rc=%d\n",
		    f->command, f->binder_type, err);
#endif
	return err;
}

#endif
