// SPDX-License-Identifier: GPL-2.0-only
/*
 * gpon_edge -- read the router edge out of the live kernel.  See gpon_edge.h
 * for why this is core code and why it is pulled rather than pushed.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include <net/arp.h>
#include <net/neighbour.h>
#include <net/route.h>

#include "gpon_edge.h"

/*
 * The L3 interface above a driver's netdev.  On a router built the ordinary
 * way the LAN port is enslaved to a bridge and the address lives THERE, so
 * reading the port's own in_device finds nothing at all.  The master is asked
 * structurally; no name is matched, because "br-lan" is a distribution's
 * convention and not a fact about the device.
 */
static struct net_device *gpon_edge_l3_dev(struct net_device *dev)
{
	struct net_device *master = netdev_master_upper_dev_get_rcu(dev);

	return master ? master : dev;
}

static bool gpon_edge_ipv4(struct net_device *dev, u32 *ip, u8 *prefix)
{
	struct in_device *in = __in_dev_get_rcu(dev);
	const struct in_ifaddr *ifa;

	if (!in)
		return false;
	in_dev_for_each_ifa_rcu(ifa, in) {
		if (ifa->ifa_scope != RT_SCOPE_UNIVERSE)
			continue;
		*ip = ntohl(ifa->ifa_local);
		if (prefix)
			*prefix = ifa->ifa_prefixlen;
		return true;
	}
	return false;
}

static u16 gpon_edge_vlan(struct net_device *dev)
{
	return is_vlan_dev(dev) ? vlan_dev_vlan_id(dev) : 0;
}

/* Is @l3 the driver's own @port, or a VLAN riding directly on it? */
static bool gpon_edge_rides(struct net_device *l3, struct net_device *port)
{
	if (l3 == port)
		return true;
	return is_vlan_dev(l3) && vlan_dev_real_dev(l3) == port;
}

static bool gpon_edge_gw_mac(struct net_device *dev, __be32 gw, u8 *mac)
{
	struct neighbour *n = __ipv4_neigh_lookup_noref(dev, (__force u32)gw);

	if (!n || !(READ_ONCE(n->nud_state) & NUD_VALID))
		return false;
	neigh_ha_snapshot((char *)mac, n, dev);
	return is_valid_ether_addr(mac);
}

int gpon_edge_read(struct net_device *lan, struct net_device *wan,
		   u32 peer, struct gpon_edge *e, const char **why)
{
	struct net_device *lan_l3, *wan_l3;
	struct flowi4 fl4 = { .daddr = htonl(peer) };
	struct rtable *rt;
	int ret = -EAGAIN;

	*why = "";
	if (!lan || !wan) {
		*why = "the driver has no LAN/WAN netdev pair";
		return -ENODEV;
	}

	memset(e, 0, sizeof(*e));
	rcu_read_lock();

	/*
	 * ★★ THE ROUTE COMES FIRST, AND IT NAMES THE WAN L3 DEVICE.
	 *
	 * ⚠ THE OBVIOUS ORDER IS WRONG AND THE BENCH PROVED IT. Reading the WAN
	 * address off the driver's own netdev (or its MASTER upper) finds
	 * NOTHING the moment the WAN rides a VLAN: `gpon0.46` is an UPPER of
	 * `gpon0` but not its master, so the address sits on a device that
	 * lookup never reaches and the engine refused a perfectly healthy WAN
	 * with "carries no IPv4 address yet". Asking the FIB which device this
	 * flow leaves through answers for the tagged and untagged cases with
	 * one mechanism, and for a PPP or tunnel WAN it answers with a device
	 * the check below then rejects -- which is the right answer too.
	 */
	rt = ip_route_output_key(dev_net(wan), &fl4);
	if (IS_ERR(rt)) {
		*why = "no route to the flow's destination";
		ret = PTR_ERR(rt);
		goto out;
	}
	if (rt->rt_gw_family != AF_INET) {
		*why = "the route to the flow's destination has no IPv4 gateway";
		goto out_put;
	}
	wan_l3 = rt->dst.dev;
	/*
	 * ⚠ AND IT MUST LEAVE THROUGH THE PORT THIS ENGINE OWNS. A PPPoE or
	 * tunnel WAN egresses through a device the accelerator cannot express,
	 * and the tables would then describe a path the packet never takes --
	 * an entry that reads back healthy and blackholes.
	 */
	if (!gpon_edge_rides(wan_l3, wan)) {
		*why = "the WAN route leaves through a device this engine does not drive";
		ret = -EOPNOTSUPP;
		goto out_put;
	}
	if (!gpon_edge_ipv4(wan_l3, &e->wan_ip, NULL)) {
		*why = "the WAN interface carries no IPv4 address yet";
		goto out_put;
	}
	e->wan_vlan = gpon_edge_vlan(wan_l3);
	ether_addr_copy(e->wan_mac, wan_l3->dev_addr);
	e->gw_ip = ntohl(rt->rt_gw4);
	if (!gpon_edge_gw_mac(wan_l3, rt->rt_gw4, e->gw_mac)) {
		*why = "the gateway's MAC is not resolved yet";
		goto out_put;
	}

	/* The LAN side IS reached through the master upper: an ordinary router
	 * enslaves the LAN port to a bridge and the address lives there. */
	lan_l3 = gpon_edge_l3_dev(lan);
	if (!gpon_edge_ipv4(lan_l3, &e->lan_ip, &e->lan_prefix)) {
		*why = "the LAN interface carries no IPv4 address yet";
		goto out_put;
	}
	e->lan_net = e->lan_ip & (e->lan_prefix == 32
				  ? ~0u : ~0u << (32 - e->lan_prefix));
	e->lan_vlan = gpon_edge_vlan(lan_l3);
	ether_addr_copy(e->lan_mac, lan_l3->dev_addr);
	ret = 0;
out_put:
	ip_rt_put(rt);
out:
	rcu_read_unlock();
	return ret;
}

bool gpon_edge_same(const struct gpon_edge *a, const struct gpon_edge *b)
{
	return a->wan_ip == b->wan_ip && a->gw_ip == b->gw_ip &&
	       a->lan_ip == b->lan_ip && a->lan_net == b->lan_net &&
	       a->wan_vlan == b->wan_vlan && a->lan_vlan == b->lan_vlan &&
	       a->lan_prefix == b->lan_prefix &&
	       ether_addr_equal(a->wan_mac, b->wan_mac) &&
	       ether_addr_equal(a->gw_mac, b->gw_mac) &&
	       ether_addr_equal(a->lan_mac, b->lan_mac);
}
