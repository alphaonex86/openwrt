// SPDX-License-Identifier: GPL-2.0-or-later
/* gpon_flow_offload -- the TC hardware-offload lifecycle, ...
 * dev/MEASURED-gpon_flow_offload.c.md sec 1. */
#include <linux/errno.h>
#include <linux/ip.h>
#include <linux/jiffies.h>
#include <linux/netdevice.h>
#include <linux/rhashtable.h>
#include <linux/slab.h>
#include <net/flow_offload.h>
#include <net/pkt_cls.h>

#include "gpon_flow_offload.h"

struct gpon_flow_entry {
	struct rhash_head	node;
	unsigned long		cookie;
	u32			idx;		/* the engine's, opaque */
	bool			ds;
	/* ops->priv_size bytes of family state follow */
};

static void *entry_priv(struct gpon_flow_entry *e)
{
	return (void *)(e + 1);
}

struct gpon_flow_offload {
	const struct gpon_flow_ops	*ops;
	void				*sh;
	struct rhashtable		table;
	bool				table_ready;
};

static const struct rhashtable_params gpon_flow_ht_params = {
	.head_offset	= offsetof(struct gpon_flow_entry, node),
	.key_offset	= offsetof(struct gpon_flow_entry, cookie),
	.key_len	= sizeof(unsigned long),
	.automatic_shrinking = true,
};

struct gpon_flow_offload *gpon_flow_offload_new(const struct gpon_flow_ops *ops,
						void *sh)
{
	struct gpon_flow_offload *fo;

	if (!ops || !ops->is_lan_side || !ops->install || !ops->remove)
		return NULL;

	fo = kzalloc(sizeof(*fo), GFP_KERNEL);
	if (!fo)
		return NULL;
	fo->ops = ops;
	fo->sh = sh;
	if (rhashtable_init(&fo->table, &gpon_flow_ht_params)) {
		kfree(fo);
		return NULL;
	}
	fo->table_ready = true;
	return fo;
}

static void gpon_flow_entry_drop(void *ptr, void *arg)
{
	struct gpon_flow_offload *fo = arg;
	struct gpon_flow_entry *e = ptr;

	fo->ops->remove(fo->sh, e->idx, entry_priv(e));
	kfree(e);
}

void gpon_flow_offload_free(struct gpon_flow_offload *fo)
{
	if (!fo)
		return;
	if (fo->table_ready) {
		fo->table_ready = false;
		rhashtable_free_and_destroy(&fo->table, gpon_flow_entry_drop, fo);
	}
	kfree(fo);
}

/* ── the ACTION decode ... -- dev/MEASURED-gpon_flow_offload.c.md sec 2. */
int gpon_flow_act_from_tc(struct gpon_flow_offload *fo, struct flow_rule *rule,
			  bool ds_leg, struct gpon_flow_act *act,
			  struct net_device **odev_out)
{
	struct flow_action_entry *fa;
	bool got_dmac_lo = false, got_dmac_hi = false;
	int i;

	*odev_out = NULL;

	flow_action_for_each(i, fa, &rule->action) {
		switch (fa->id) {
		case FLOW_ACTION_REDIRECT:
			*odev_out = fa->dev;
			break;
		case FLOW_ACTION_MANGLE:
			switch (fa->mangle.htype) {
			case FLOW_ACT_MANGLE_HDR_TYPE_IP4:
				/* nf_flow_table's two legs of a masqueraded flow are exact ...
				 * dev/MEASURED-gpon_flow_offload.c.md sec 3. */
				if (fa->mangle.offset !=
				    (ds_leg ? offsetof(struct iphdr, daddr)
					    : offsetof(struct iphdr, saddr)))
					return -EOPNOTSUPP;
				act->nat_valid = 1;
				act->nat_is_da = ds_leg;
				act->nat_addr = ntohl(fa->mangle.val);
				break;
			case FLOW_ACT_MANGLE_HDR_TYPE_TCP:
			case FLOW_ACT_MANGLE_HDR_TYPE_UDP:
				/* The port rewrite is ONE big-endian 32-bit word at offset 0: ...
				 * dev/MEASURED-gpon_flow_offload.c.md sec 4. */
				if (fa->mangle.offset != 0 ||
				    (fa->mangle.mask == ~htonl(0xffff)) != ds_leg)
					return -EOPNOTSUPP;
				act->nat_port = ds_leg ?
					(ntohl(fa->mangle.val) & 0xffff) :
					(ntohl(fa->mangle.val) >> 16);
				act->port_valid = 1;
				break;
			case FLOW_ACT_MANGLE_HDR_TYPE_ETH:
				/* This leg's next-hop DMAC, emitted as TWO ETH mangles: ...
				 * dev/MEASURED-gpon_flow_offload.c.md sec 5. */
				if (fa->mangle.offset == 0) {
					u32 w = fa->mangle.val;

					memcpy(act->gw_dmac, &w, 4);
					got_dmac_lo = true;
				} else if (fa->mangle.offset == 4 &&
					   (fa->mangle.mask & 0xffff) == 0) {
					u16 w = (u16)fa->mangle.val;

					memcpy(act->gw_dmac + 4, &w, 2);
					got_dmac_hi = true;
				}
				break;
			default:
				return -EOPNOTSUPP;
			}
			break;
		case FLOW_ACTION_CSUM:
			break;		/* implicit in the HW rewrite */
		case FLOW_ACTION_PPPOE_PUSH:
			/* The LIVE negotiated session id (host-order u16, from the ...
			 * dev/MEASURED-gpon_flow_offload.c.md sec 10. */
			act->pppoe_sid = fa->pppoe.sid;
			break;
		case FLOW_ACTION_VLAN_PUSH:
		case FLOW_ACTION_VLAN_POP:
			/* Reached only when the WAN sub-interface's LOWER device is ...
			 * dev/MEASURED-gpon_flow_offload.c.md sec 6. */
			if (fo && fo->ops->note_vlan_action)
				fo->ops->note_vlan_action(fo->sh, ds_leg,
					fa->id == FLOW_ACTION_VLAN_PUSH ?
					fa->vlan.vid : 0);
			return -EOPNOTSUPP;
		default:
			return -EOPNOTSUPP;
		}
	}

	act->dmac_valid = got_dmac_lo && got_dmac_hi;

	/*
	 * Keep to the proven shape: a full inline NAT rewrite plus a redirect.
	 * Anything less cannot be expressed as one entry.
	 */
	if (!*odev_out || !act->nat_valid || !act->port_valid)
		return -EOPNOTSUPP;
	return 0;
}

/*
 * ── the three TC verbs ───────────────────────────────────────────────────
 */
int gpon_flow_offload_replace(struct gpon_flow_offload *fo,
			      struct flow_cls_offload *f,
			      struct net_device *blockdev)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct gpon_flow_entry *entry;
	struct gpon_flow_ctx ctx = {};
	struct gpon_flow_act act = {};
	struct gpon_flow_key key = {};
	int err;

	if (!fo || !fo->table_ready)
		return -EOPNOTSUPP;

	/* The other direction may already hold this cookie. Refusing ...
	 * dev/MEASURED-gpon_flow_offload.c.md sec 7. */
	if (rhashtable_lookup_fast(&fo->table, &f->cookie, gpon_flow_ht_params))
		return -EEXIST;

	err = gpon_flow_key_from_tc(rule, &key);
	if (err)
		return err;

	/* ★ The block-cb device is NOT the flow's ingress -- every ...
	 * dev/MEASURED-gpon_flow_offload.c.md sec 8. */
	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_META)) {
		struct flow_match_meta m;

		flow_rule_match_meta(rule, &m);
		ctx.idev = dev_get_by_index(dev_net(blockdev),
					    m.key->ingress_ifindex);
		if (ctx.idev)
			ctx.ds_leg = !fo->ops->is_lan_side(fo->sh, ctx.idev);
	}

	err = gpon_flow_act_from_tc(fo, rule, ctx.ds_leg, &act, &ctx.odev);
	if (err)
		goto out_put;

	entry = kzalloc(sizeof(*entry) + fo->ops->priv_size, GFP_KERNEL);
	if (!entry) {
		err = -ENOMEM;
		goto out_put;
	}
	entry->cookie = f->cookie;
	entry->ds = ctx.ds_leg;

	/* Every decision that needs silicon, and the install, in ONE ...
	 * dev/MEASURED-gpon_flow_offload.c.md sec 11. */
	err = fo->ops->install(fo->sh, &key, &act, &ctx, entry_priv(entry),
			       &entry->idx);
	if (err)
		goto out_free;

	err = rhashtable_insert_fast(&fo->table, &entry->node,
				     gpon_flow_ht_params);
	if (err) {
		/* the ONE unwind path: the flow is in hardware and would
		 * otherwise be unreachable by cookie, i.e. leaked for good */
		fo->ops->remove(fo->sh, entry->idx, entry_priv(entry));
		goto out_free;
	}

	if (ctx.idev)
		dev_put(ctx.idev);
	return 0;

out_free:
	kfree(entry);
out_put:
	if (ctx.idev)
		dev_put(ctx.idev);
	return err;
}

void gpon_flow_offload_flush(struct gpon_flow_offload *fo)
{
	struct rhashtable_iter it;
	struct gpon_flow_entry *e;

	if (!fo || !fo->table_ready)
		return;

	/* ⚠ REMOVING WHILE WALKING. rhashtable's iterator is ...
	 * dev/MEASURED-gpon_flow_offload.c.md sec 9. */
	rhashtable_walk_enter(&fo->table, &it);
	do {
		rhashtable_walk_start(&it);
		e = rhashtable_walk_next(&it);
		while (e && !IS_ERR(e)) {
			rhashtable_walk_stop(&it);
			fo->ops->remove(fo->sh, e->idx, entry_priv(e));
			rhashtable_remove_fast(&fo->table, &e->node,
					       gpon_flow_ht_params);
			kfree(e);
			rhashtable_walk_start(&it);
			e = rhashtable_walk_next(&it);
		}
		rhashtable_walk_stop(&it);
	} while (e == ERR_PTR(-EAGAIN));
	rhashtable_walk_exit(&it);
}

int gpon_flow_offload_destroy(struct gpon_flow_offload *fo,
			      struct flow_cls_offload *f)
{
	struct gpon_flow_entry *entry;

	if (!fo || !fo->table_ready)
		return -EOPNOTSUPP;

	entry = rhashtable_lookup_fast(&fo->table, &f->cookie,
				       gpon_flow_ht_params);
	if (!entry)
		return -ENOENT;

	fo->ops->remove(fo->sh, entry->idx, entry_priv(entry));
	rhashtable_remove_fast(&fo->table, &entry->node, gpon_flow_ht_params);
	kfree(entry);
	return 0;
}

int gpon_flow_offload_stats(struct gpon_flow_offload *fo,
			    struct flow_cls_offload *f)
{
	struct gpon_flow_entry *entry;
	unsigned long lastused = 0;
	int err;

	if (!fo || !fo->table_ready)
		return -EOPNOTSUPP;

	entry = rhashtable_lookup_fast(&fo->table, &f->cookie,
				       gpon_flow_ht_params);
	if (!entry)
		return -ENOENT;

	if (!fo->ops->stats)
		return -EOPNOTSUPP;

	err = fo->ops->stats(fo->sh, entry->idx, entry_priv(entry), &lastused);
	if (err)
		return err;

	f->stats.lastused = lastused;
	return 0;
}
