// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL9602C L34 -> the COMMON TC hardware-offload lifecycle.
 *
 * ★★ WHY THIS FILE EXISTS (operator, 2026-08-28): *"hay una parte de hw
 * acceleration que no activas y que es presente en todos y tiene el codigo en
 * X400AXF"*.  He was right about this board, and the measurement is worse than
 * the suspicion: rtl9602c_l34.c is a COMPLETE NAPT engine -- it hashes into the
 * NAPTR-in and NAPT-out tables, finds a free way and programs both -- and
 * `rtl9602c_l34_flow_add`, `_flow_del`, `_flow_hit`, `_wan_setup` and
 * `_lan_setup` were called by NOBODY.  The engine was initialised, published a
 * /proc node, and never received one flow.  Every routed packet was moved by
 * the CPU on a board that has silicon to do it.  The five ops below are what
 * connects them, and op_install is where the last of it was closed
 * (2026-09-11): it fills the interface tables from the live kernel edge, and
 * only then hands a flow to the engine.
 *
 * ⚠ AND HIS PREMISE IS HALF RIGHT, WHICH MATTERS FOR WHERE THIS LIVES.  The
 * vendor's own abstraction layer carries an `l34` for `rtl9602c` and for
 * nothing else in this family -- not rtl9603cvd, not rtl9607c, not rtl9607f.
 * The G24W does not even compile this file (it builds luna_eth.c, and the
 * L34 is textually included by rtl9602c_eth.c alone).  And the X400AXF's
 * accelerator is the CORTINA L3FE, different silicon with different tables --
 * so "use the X400AXF's accelerator on the others" is not available at the
 * ENGINE level.
 *
 * ★ WHAT *IS* SHARED IS THE LIFECYCLE, and it already is: the cookie->flow map,
 * the TC action decode, the entry and its unwind, destroy, stats and flush live
 * in drivers/net/gpon/gpon_flow_offload.c and are compiled by whichever family
 * selects CONFIG_GPON_FLOW_OFFLOAD.  A family without an engine compiles none
 * of it.  This file is that family's five-op table -- which is the whole
 * measure of whether the seam was drawn in the right place.
 */

#include <net/flow_offload.h>	/* struct flow_cls_offload, the block cb type */
#include <net/pkt_cls.h>	/* FLOW_CLS_*, FLOW_BLOCK_*, flow_block_cb_*   */

/*
 * ★ THE INSTALL REFUSED UNCONDITIONALLY UNTIL 2026-09-11, AND THE REASON IT
 * GAVE WAS TRUE: `struct l34_flow` carries `egress_netif`, an index into the
 * L34_TBL_NETIF / L34_TBL_EXTIP tables, and nothing filled those tables, so an
 * installed flow pointed at slot 0 of empty state -- an entry that reads back
 * perfectly and blackholes ("plumb the VALUE, not the flag", paid for once
 * already).
 *
 * What lifts it is not a deleted check.  gpon_edge_read() (core) derives the
 * WAN address and MAC, the resolved next hop and the LAN subnet from the LIVE
 * kernel, rtl9602c_l34_provision() writes them into NETIF / L3ROUTE / NEXTHOP /
 * EXTIP / ARP, and only a 0 from THAT admits a flow.  The refusal survives
 * wherever the values do not: no address yet, no resolved gateway, a WAN that
 * egresses through a device this engine cannot express.
 */

/*
 * ⚠ FORWARD DECLARATION, and it is load-bearing.  This file is textually
 * included BEFORE rtl9602c_eth_netdev_ops is defined -- it has to be, because
 * that ops table names rtl9602c_l34_setup_tc -- while this file needs the ops
 * table's ADDRESS to tell a LAN netdev from the PON-side WAN one.  The two
 * refer to each other, so one of them is declared first.
 */
static const struct net_device_ops rtl9602c_eth_netdev_ops;

/*
 * The driver behind a netdev.  ⚠ THE TWO NETDEVS DO NOT STORE IT THE SAME WAY:
 * the LAN device embeds `struct rtl9602c_eth` in its private area, the PON-side
 * WAN device stores a POINTER to it.  Reading the wrong shape yields a pointer
 * that is not a driver and dereferences cleanly into whatever is there, so the
 * discriminator is the same STRUCTURAL one is_lan_side uses -- which ops the
 * device carries -- and never a name or a guess.
 */
static struct rtl9602c_eth *rtl9602c_eth_of(struct net_device *dev)
{
	if (!dev)
		return NULL;
	if (dev->netdev_ops == &rtl9602c_eth_wan_ops)
		return *(struct rtl9602c_eth **)netdev_priv(dev);
	if (dev->netdev_ops == &rtl9602c_eth_netdev_ops)
		return netdev_priv(dev);
	return NULL;
}

struct rtl9602c_l34_priv {
	struct l34_flow	f;		/* handed back to flow_del */
	unsigned long	last_hit;	/* fed by the stats op */
};

static bool rtl9602c_l34_is_lan_side(void *sh, struct net_device *dev)
{
	/*
	 * ★ STRUCTURAL, never a name.  This driver registers two distinct
	 * net_device_ops, one for the LAN netdev and one for the PON-side WAN
	 * netdev, so the question is answered by WHICH OPS the device carries.
	 * Matching on "gpon0" would be a netdev NAME deciding a verdict, which
	 * this tree has a guard against for good reason.
	 */
	return dev && dev->netdev_ops != &rtl9602c_eth_wan_ops;
}

/*
 * ⚠ ONE HARDWARE ENTRY PER CONNECTION, AND IT IS THE UPSTREAM ONE.
 * nf_flow_table offers both legs, each with its own cookie.  This engine's
 * NAPT pair already covers both directions from the upstream tuple alone --
 * NAPT_OUT is keyed on the LAN->WAN 5-tuple and NAPTR_IN on the translated WAN
 * address/port -- so installing the reply leg as well would take a second slot
 * describing the same connection.  Refusing it is not a gap: the core treats
 * -EOPNOTSUPP as "this cookie stays on the software fastpath", and the reply
 * packets are accelerated by the entry the upstream leg already programmed.
 */
static int l34_refuse(struct rtl9602c_l34 *l, const char *why)
{
	l->refusals++;
	l->refuse_why = why;			/* a static literal, always */
	pr_debug_ratelimited("rtl9602c-l34: not offloading -- %s\n", why);
	return -EOPNOTSUPP;
}

static int rtl9602c_l34_op_install(void *sh, const struct gpon_flow_key *k,
				   const struct gpon_flow_act *a,
				   const struct gpon_flow_ctx *ctx, void *priv,
				   u32 *idx_out)
{
	struct rtl9602c_eth *ep = sh;
	struct rtl9602c_l34_priv *p = priv;
	struct l34_flow *f = &p->f;
	struct gpon_edge edge;
	const char *why = "";
	int ret;

	*idx_out = 0;
	if (!ep || !ep->l34.ready)
		return -ENODEV;
	/*
	 * ⚠ THE REPLY LEG IS NOT A REFUSAL AND MAY NOT BE COUNTED AS ONE. It
	 * arrives once per connection, so folding it into `refusals` would make
	 * the most common outcome the permanent value of `last_refusal` -- and
	 * the one line written to answer "why is nothing offloaded" would then
	 * always read "the reply leg needs no entry of its own", drowning every
	 * refusal worth reading. It gets its own counter.
	 */
	if (ctx->ds_leg || a->nat_is_da) {
		ep->l34.ds_legs++;
		return -EOPNOTSUPP;
	}

	/*
	 * ★ THE VALUES FIRST, THE FLOW SECOND.  The interface tables are filled
	 * from what Linux has already resolved for THIS destination, and the
	 * flow is only installed once that returned 0.  Reading the edge here
	 * rather than at probe is what makes it current: a flow exists because
	 * a packet was forwarded along this path a moment ago.
	 */
	ret = gpon_edge_read(ep->ndev, ep->wan_ndev, k->ip_da, &edge, &why);
	if (ret)
		return l34_refuse(&ep->l34, why);

	/*
	 * The rule's own next hop and NAT address must be the ones the tables
	 * describe.  A flow leaving through a different gateway, or NAT'd to an
	 * address other than the WAN interface's, would be rewritten by the
	 * engine to the interface values and sent somewhere Linux did not
	 * choose -- healthy in every readback, wrong on the wire.
	 */
	if (a->dmac_valid && !ether_addr_equal(a->gw_dmac, edge.gw_mac))
		return l34_refuse(&ep->l34,
				  "the rule's next hop is not the WAN gateway "
				  "the interface tables describe");
	if (a->nat_addr != edge.wan_ip)
		return l34_refuse(&ep->l34,
				  "the rule NATs to an address the WAN "
				  "interface does not hold");

	if (!ep->l34.provisioned || !gpon_edge_same(&edge, &ep->l34.edge)) {
		/*
		 * ⚠ THE INSTALLED FLOWS BELONG TO THE OLD EDGE.  A NAPTR's
		 * inbound bucket is hashed from the WAN address it was
		 * installed with, so every existing entry becomes unreachable
		 * the moment that address moves -- the same reason the Cortina
		 * engine flushes on a PPPoE session change.  The core owns the
		 * cookie table, so the core does the flush.
		 */
		if (ep->l34.provisioned)
			gpon_flow_offload_flush(ep->fo);
		ret = rtl9602c_l34_provision(&ep->l34, &edge);
		if (ret) {
			pr_warn_ratelimited("rtl9602c-l34: the interface tables would not program (%d); staying on software forwarding\n",
				ret);
			return l34_refuse(&ep->l34,
					  "the interface tables would not "
					  "program (see dmesg)");
		}
	}

	/*
	 * Everything the engine needs about the FLOW is already decided by the
	 * core, in vocabulary no accelerator owns.  The mapping is one-to-one,
	 * which is the evidence that the core's key/act are the right shape and
	 * not a Cortina struct wearing a generic name.
	 */
	f->l4proto    = k->ip_protocol;
	f->orig_sip   = k->ip_sa;
	f->orig_dip   = k->ip_da;
	f->orig_sport = k->l4_sport;
	f->orig_dport = k->l4_dport;
	f->nat_sip    = a->nat_addr;
	f->nat_sport  = a->nat_port;
	f->egress_netif = L34_NETIF_WAN;
	f->nexthop      = L34_NETIF_WAN;

	ret = rtl9602c_l34_flow_add(&ep->l34, f);
	if (ret) {
		l34_refuse(&ep->l34, ret == -ENOSPC
			   ? "every way of the flow's hash bucket is taken"
			   : "the engine refused the NAPT entry");
		return ret;
	}
	*idx_out = f->hw_index;
	return 0;
}

static int rtl9602c_l34_op_remove(void *sh, u32 idx, void *priv)
{
	struct rtl9602c_eth *ep = sh;
	struct rtl9602c_l34_priv *p = priv;

	if (!ep || !ep->l34.ready)
		return -ENODEV;
	p->f.hw_index = (u16)idx;
	return rtl9602c_l34_flow_del(&ep->l34, &p->f);
}

static int rtl9602c_l34_op_stats(void *sh, u32 idx, void *priv,
				 unsigned long *lastused)
{
	struct rtl9602c_eth *ep = sh;
	struct rtl9602c_l34_priv *p = priv;
	bool active = false;

	if (!ep || !ep->l34.ready)
		return -ENODEV;
	/*
	 * The engine reports LIVENESS, not counters -- the same shape the
	 * Cortina side reports, which is why the core's stats op asks for a
	 * `lastused` and not for packets and bytes.  Reading the hit bit CLEARS
	 * it, so the timestamp has to be kept here.
	 */
	if (!rtl9602c_l34_flow_hit(&ep->l34, (u16)idx, &active) && active)
		p->last_hit = jiffies;
	*lastused = p->last_hit;
	return 0;
}

/*
 * devm release: pull every installed flow out of the hardware, then free the
 * handle.  gpon_flow_offload_free() walks the cookie table and calls this
 * driver's remove op per entry, so nothing is left programmed in silicon that
 * no software knows about.
 */
static void rtl9602c_l34_fo_release(void *data)
{
	struct rtl9602c_eth *ep = data;

	gpon_flow_offload_free(ep->fo);
	ep->fo = NULL;
}

/*
 * ★★ THE CORE OFFERS THIS ATTRIBUTION AND THIS FAMILY THREW IT AWAY. A VLAN
 * push/pop on a rule is refused by the core for every engine here -- no hit
 * action can express a tag -- but WITHOUT this op the refusal arrives as one
 * more anonymous "the lifecycle declined it", which is exactly how the same
 * defect became unreadable twice. MEASURED on this board 2026-09-11: with the
 * WAN on a VLAN sub-interface, 56 of 56 offered flows were refused and nothing
 * anywhere said the word VLAN.
 */
static void rtl9602c_l34_op_note_vlan(void *sh, bool ds_leg, u16 vid)
{
	struct rtl9602c_eth *ep = sh;

	if (!ep)
		return;
	ep->l34.vlan_refused++;
	ep->l34.vlan_refused_vid = vid;
}

static const struct gpon_flow_ops rtl9602c_l34_flow_ops = {
	.is_lan_side	= rtl9602c_l34_is_lan_side,
	.note_vlan_action = rtl9602c_l34_op_note_vlan,
	.install	= rtl9602c_l34_op_install,
	.remove		= rtl9602c_l34_op_remove,
	.stats		= rtl9602c_l34_op_stats,
	.priv_size	= sizeof(struct rtl9602c_l34_priv),
};

/* ── the TC block, same shape as the Cortina family's ─────────────────────
 *
 * The DISPATCH is the core's; what a driver still owns is the block plumbing
 * (which binder type it accepts, the per-block refcount) because that is a
 * property of which netdevs it registered, not of any accelerator.
 */
static DEFINE_MUTEX(rtl9602c_l34_tc_mutex);
static LIST_HEAD(rtl9602c_l34_block_cb_list);

static int rtl9602c_l34_block_cb(enum tc_setup_type type, void *type_data,
				 void *cb_priv)
{
	struct flow_cls_offload *f = type_data;
	struct net_device *dev = cb_priv;
	struct rtl9602c_eth *ep;
	int err;

	if (type != TC_SETUP_CLSFLOWER)
		return -EOPNOTSUPP;
	ep = rtl9602c_eth_of(dev);
	if (!ep || !ep->fo)
		return -EOPNOTSUPP;

	mutex_lock(&rtl9602c_l34_tc_mutex);
	switch (f->command) {
	case FLOW_CLS_REPLACE: {
		/*
		 * ⚠ WHO REFUSED IS DECIDED BY WHAT MOVED DURING *THIS* CALL.
		 * Testing `refuse_why` instead would be wrong after the first
		 * engine refusal ever: that pointer PERSISTS, so every later
		 * core refusal would go uncounted and the two faults would
		 * merge back into one number.
		 */
		u32 before = ep->l34.refusals + ep->l34.ds_legs;

		ep->l34.offered++;
		err = gpon_flow_offload_replace(ep->fo, f, dev);
		/* -EEXIST is the other leg's cookie, not a refusal */
		if (err < 0 && err != -EEXIST &&
		    ep->l34.refusals + ep->l34.ds_legs == before)
			ep->l34.core_refused++;
		break;
	}
	case FLOW_CLS_DESTROY:
		err = gpon_flow_offload_destroy(ep->fo, f);
		break;
	case FLOW_CLS_STATS:
		err = gpon_flow_offload_stats(ep->fo, f);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}
	mutex_unlock(&rtl9602c_l34_tc_mutex);
	return err;
}

static int rtl9602c_l34_setup_block(struct net_device *dev,
				    struct flow_block_offload *f)
{
	flow_setup_cb_t *cb = rtl9602c_l34_block_cb;
	struct rtl9602c_eth *ep = rtl9602c_eth_of(dev);
	struct flow_block_cb *block_cb;

	/*
	 * ★ DO NOT CLAIM A BLOCK WE CANNOT HONOUR.  `hw_nat` defaults to 0 and
	 * nothing in this target sets it, so on the shipped image ep->fo is
	 * NULL and rtl9602c_l34_block_cb() refuses every single flow one level
	 * below.  Accepting the BIND anyway made the netfilter flowtable install
	 * a callback that can only ever answer -EOPNOTSUPP, and it put the
	 * refusal a level away from the reason for it.  Refuse here instead:
	 * the caller then takes its own no-offload path, which is what actually
	 * happens today.
	 */
	if (!ep || !ep->fo)
		return -EOPNOTSUPP;

	/*
	 * ★★ THIS CHECK IS NOT THE THROUGHPUT BLOCKER, AND IT WAS RECORDED AS
	 * ONE (measured against linux-6.18.31, 2026-09-08).  Three places in
	 * this project asserted that "the netfilter flowtable binds
	 * FLOW_BLOCK_BINDER_TYPE_FT", making this line look like the reason the
	 * L34 engine never sees a flow.  Both halves are false in this kernel:
	 *
	 *   - `enum flow_block_binder_type` (include/net/flow_offload.h) has no
	 *     `FT` value at all -- only UNSPEC / CLSACT_INGRESS / CLSACT_EGRESS
	 *     / RED_EARLY_DROP / RED_MARK.  `FT` is a `tc_setup_type`
	 *     (TC_SETUP_FT), a different field, accepted in setup_tc() below.
	 *   - nf_flow_table_block_offload_init() sets binder_type =
	 *     CLSACT_INGRESS and then calls ndo_setup_tc(dev, TC_SETUP_FT, bo),
	 *     so this comparison PASSES for the flowtable.  Per flow it calls
	 *     the block callback with TC_SETUP_CLSFLOWER, also accepted.
	 *
	 * The real refusals are the ep->fo guard above (hw_nat=0) and, behind
	 * it, the unconditional -EOPNOTSUPP in rtl9602c_l34_op_install() while
	 * the NETIF/NEXTHOP tables are unprovisioned.  Deleting this line buys
	 * nothing; it only widens what we accept and then refuse.
	 */
	if (f->binder_type != FLOW_BLOCK_BINDER_TYPE_CLSACT_INGRESS)
		return -EOPNOTSUPP;
	f->driver_block_list = &rtl9602c_l34_block_cb_list;

	switch (f->command) {
	case FLOW_BLOCK_BIND:
		block_cb = flow_block_cb_lookup(f->block, cb, dev);
		if (block_cb) {
			flow_block_cb_incref(block_cb);
			return 0;
		}
		block_cb = flow_block_cb_alloc(cb, dev, dev, NULL);
		if (IS_ERR(block_cb))
			return PTR_ERR(block_cb);
		ep->l34.binds++;
		flow_block_cb_incref(block_cb);
		flow_block_cb_add(block_cb, f);
		list_add_tail(&block_cb->driver_list,
			      &rtl9602c_l34_block_cb_list);
		return 0;
	case FLOW_BLOCK_UNBIND:
		block_cb = flow_block_cb_lookup(f->block, cb, dev);
		if (!block_cb)
			return -ENOENT;
		if (!flow_block_cb_decref(block_cb)) {
			flow_block_cb_remove(block_cb, f);
			list_del(&block_cb->driver_list);
		}
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int rtl9602c_l34_setup_tc(struct net_device *dev,
				 enum tc_setup_type type, void *type_data)
{
	switch (type) {
	case TC_SETUP_BLOCK:
	case TC_SETUP_FT:
		return rtl9602c_l34_setup_block(dev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}
