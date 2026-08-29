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
 * `_lan_setup` are called by NOBODY.  The engine is initialised, publishes a
 * /proc node, and never receives one flow.  Every routed packet is moved by the
 * CPU on a board that has silicon to do it.
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
 * ⚠⚠ THE INSTALL REFUSES TODAY, DELIBERATELY, AND THIS IS NOT A STUB.
 *
 * `struct l34_flow` carries `egress_netif` and `nexthop` -- indices into the
 * L34_TBL_NETIF and L34_TBL_NEXTHOP tables that `rtl9602c_l34_wan_setup()` and
 * `_lan_setup()` fill.  NOTHING calls those either, so those tables are EMPTY.
 * Installing a flow that points at index 0 of an empty table produces an entry
 * that looks perfectly healthy in every readback and BLACKHOLES the traffic --
 * the exact failure this project has already paid for once ("plumb the VALUE,
 * not the flag").
 *
 * So the refusal is the honest state, and it names what is missing.  Lifting it
 * requires provisioning the two tables from real values (WAN address and MAC,
 * gateway MAC, LAN network) and then MEASURING a flow on the board -- which
 * cannot be done from a build.  Until that happens this file proves exactly one
 * thing, and it is the thing that was asked: the core's contract is reachable
 * from a second family.
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

static int rtl9602c_l34_op_install(void *sh, const struct gpon_flow_key *k,
				   const struct gpon_flow_act *a,
				   const struct gpon_flow_ctx *ctx, void *priv,
				   u32 *idx_out)
{
	struct rtl9602c_eth *ep = sh;
	struct rtl9602c_l34_priv *p = priv;
	struct l34_flow *f = &p->f;

	if (!ep || !ep->l34.ready)
		return -ENODEV;

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
	if (a->nat_valid) {
		if (a->nat_is_da)
			f->nat_dip = a->nat_addr;
		else
			f->nat_sip = a->nat_addr;
	}
	if (a->port_valid) {
		if (a->nat_is_da)
			f->nat_dport = a->nat_port;
		else
			f->nat_sport = a->nat_port;
	}

	/*
	 * ⚠ AND HERE IS WHERE IT STOPS.  See the banner: the NETIF and NEXTHOP
	 * tables are empty because nothing calls wan_setup/lan_setup, so
	 * `egress_netif` and `nexthop` have no meaningful value to carry and an
	 * installed flow would blackhole while reading back as healthy.
	 *
	 * -EOPNOTSUPP is a NORMAL outcome for the core: the flow stays on the
	 * Linux software fastpath, exactly where it is today.  Nothing is lost
	 * by refusing and something real is lost by guessing.
	 */
	pr_debug_ratelimited("rtl9602c-l34: refusing %pI4h:%u->%pI4h:%u proto=%u -- "
			     "the NETIF/NEXTHOP tables are not provisioned "
			     "(wan_setup/lan_setup are still called by nobody), "
			     "so an installed flow would blackhole\n",
			     &f->orig_sip, f->orig_sport,
			     &f->orig_dip, f->orig_dport, f->l4proto);
	*idx_out = 0;
	return -EOPNOTSUPP;
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

static const struct gpon_flow_ops rtl9602c_l34_flow_ops = {
	.is_lan_side	= rtl9602c_l34_is_lan_side,
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
	case FLOW_CLS_REPLACE:
		err = gpon_flow_offload_replace(ep->fo, f, dev);
		break;
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
	struct flow_block_cb *block_cb;

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
