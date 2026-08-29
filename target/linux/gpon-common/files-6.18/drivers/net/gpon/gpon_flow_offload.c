// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gpon_flow_offload -- the TC hardware-offload lifecycle, shared.
 *
 * See gpon_flow_offload.h for WHERE the seam is and why it is not where it
 * first looked.  This file holds the half that is a fact about nf_flow_table
 * and about keeping a cookie map; every line that would have to read or write
 * a register is on the other side of `ops`.
 */
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

/*
 * ── the ACTION decode ────────────────────────────────────────────────────
 *
 * Everything here is nf_flow_table's own encoding, documented against what
 * emits it, because the encodings are not obvious and getting one backwards
 * installs a WORKING-LOOKING entry that rewrites the wrong end of the flow.
 */
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
				/*
				 * nf_flow_table's two legs of a masqueraded
				 * flow are exact mirrors: the ORIGINAL rule
				 * mangles saddr (offset 12) to our WAN address,
				 * the REPLY rule mangles daddr (offset 16) back
				 * to the client's.  The discriminator is the
				 * INGRESS SIDE and NOT the SNAT/DNAT flag, so
				 * an inbound port-forward works too -- its
				 * WAN-ingress leg is a daddr rewrite and its
				 * LAN-ingress leg a saddr rewrite, the same two
				 * shapes reached via ipv4_dnat().
				 *
				 * A doubly-NAT'd flow emits BOTH mangles on one
				 * leg; the second trips this and the flow is
				 * refused to software -- correct, because an
				 * entry carries exactly one rewrite.
				 */
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
				/*
				 * The port rewrite is ONE big-endian 32-bit
				 * word at offset 0: source in the upper half,
				 * dest in the lower.  The MASK is the leg
				 * discriminator (flow_offload_port_snat):
				 * ORIGINAL -> mask ~htonl(0xffff0000), value
				 * p<<16 = the new SOURCE port; REPLY -> mask
				 * ~htonl(0xffff), value p = restore the DEST
				 * port to the client's original one.
				 */
				if (fa->mangle.offset != 0 ||
				    (fa->mangle.mask == ~htonl(0xffff)) != ds_leg)
					return -EOPNOTSUPP;
				act->nat_port = ds_leg ?
					(ntohl(fa->mangle.val) & 0xffff) :
					(ntohl(fa->mangle.val) >> 16);
				act->port_valid = 1;
				break;
			case FLOW_ACT_MANGLE_HDR_TYPE_ETH:
				/*
				 * This leg's next-hop DMAC, emitted as TWO ETH
				 * mangles: offset 0 = dmac[0..3]; offset 4 with
				 * the low 16 bits changed (the mask KEEPS the
				 * high 16) = dmac[4..5].  The offset-4-high and
				 * offset-8 words are the SMAC, which the engine
				 * substitutes itself -- ignored here.
				 *
				 * Direction-agnostic on purpose: the same parse
				 * yields the WAN gateway MAC on the US leg and
				 * the LAN client's MAC on the DS leg, because
				 * flow_offload_eth_dst() resolves the neighbour
				 * of the OTHER tuple's source address.
				 */
				if (fa->mangle.offset == 0) {
					act->gw_dmac[0] = fa->mangle.val & 0xff;
					act->gw_dmac[1] = (fa->mangle.val >> 8) & 0xff;
					act->gw_dmac[2] = (fa->mangle.val >> 16) & 0xff;
					act->gw_dmac[3] = (fa->mangle.val >> 24) & 0xff;
					got_dmac_lo = true;
				} else if (fa->mangle.offset == 4 &&
					   (fa->mangle.mask & 0xffff) == 0) {
					act->gw_dmac[4] = fa->mangle.val & 0xff;
					act->gw_dmac[5] = (fa->mangle.val >> 8) & 0xff;
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
			/*
			 * The LIVE negotiated session id (host-order u16, from
			 * the reply tuple's encap -- nf_flow_rule_route_common;
			 * mtk_ppe precedent).  NEVER a configured constant.
			 */
			act->pppoe_sid = fa->pppoe.sid;
			break;
		case FLOW_ACTION_VLAN_PUSH:
		case FLOW_ACTION_VLAN_POP:
			/*
			 * Reached only when the WAN sub-interface's LOWER device
			 * is itself a flowtable device, so the encap survived
			 * nft_dev_forward_path() and a push/pop was emitted for
			 * it.  No hit-action here can express a tag, so it is
			 * refused -- but ATTRIBUTED, because the absence of that
			 * attribution made this defect unreadable twice.
			 */
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

	/*
	 * The other direction may already hold this cookie.  Refusing here is
	 * not an error path: nf_flow_table offers both legs and each carries
	 * its own cookie, so a COLLISION means we are being re-offered one we
	 * already installed.
	 */
	if (rhashtable_lookup_fast(&fo->table, &f->cookie, gpon_flow_ht_params))
		return -EEXIST;

	err = gpon_flow_key_from_tc(rule, &key);
	if (err)
		return err;

	/*
	 * ★ The block-cb device is NOT the flow's ingress -- every registered
	 * cb sees every rule.  The rule carries the real ingress ifindex in its
	 * META key.  LAN ingress = the US (LAN->WAN) transit direction;
	 * anything else is the DS (WAN->LAN) reply leg, which nf_flow_table
	 * offers as a second REPLACE with its own cookie once
	 * NF_FLOW_HW_BIDIRECTIONAL is set (nft_flow_offload and xt_FLOWOFFLOAD
	 * both set it unconditionally).
	 *
	 * ⚠ A rule with NO META key leaves ds_leg false, i.e. it is treated as
	 * the US leg.  That is the historical behaviour and it is kept
	 * deliberately: changing it here would change which mangle offsets are
	 * accepted, on every board at once, for a shape nobody has measured.
	 */
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

	/*
	 * Every decision that needs silicon, in one call.  It may amend `act`
	 * (a family that resolves the PPPoE session from its own WAN chain does
	 * it here) and it runs before anything is installed, so a refusal costs
	 * no unwind.
	 */
	if (fo->ops->prepare) {
		err = fo->ops->prepare(fo->sh, &key, &act, &ctx,
				       entry_priv(entry));
		if (err)
			goto out_free;
	}

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
