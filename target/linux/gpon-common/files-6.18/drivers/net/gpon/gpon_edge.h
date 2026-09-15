/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TIER: CORE, not in the STRICT subset -- it reads Linux's own routing state on
 * purpose.  No MMIO, no device pointer, no table word, no chip fact below.
 *
 * gpon_edge -- the router EDGE, read from the LIVE kernel.  Every flow
 * accelerator here needs the same five things before it can rewrite a packet:
 * which source address and MAC the WAN egresses with, which next hop it
 * egresses THROUGH, and which subnet is behind the LAN.  None is a property of
 * any silicon, so deriving it once costs the next board a TABLE, not a copy.
 *
 * ★ PULLED, NOT PUSHED.  A notifier chain was the obvious shape and is the
 * wrong one: the values are wanted at exactly one moment, when a flow is being
 * installed, and are all resolved by construction then (the flow exists because
 * Linux forwarded a packet along them).  Pulling keeps the mechanism free of a
 * workqueue, a timer and a lock, and it cannot go stale.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#ifndef GPON_EDGE_H
#define GPON_EDGE_H

#include <linux/types.h>
#include <linux/if_ether.h>

struct net_device;

/*
 * Addresses are HOST order.  Every engine packs its own table words with
 * explicit shift/mask, so a network-order value here would only be a second
 * place to get the byte order wrong.
 */
struct gpon_edge {
	u32	wan_ip;			/* the address a US frame is NAT'd to	*/
	u32	gw_ip;			/* the next hop that address egresses to */
	u32	lan_ip;			/* the ONU's own address on the LAN	*/
	u32	lan_net;		/* the LAN subnet, masked		*/
	u8	wan_mac[ETH_ALEN];	/* the WAN egress source MAC		*/
	u8	gw_mac[ETH_ALEN];	/* the next hop's resolved MAC		*/
	u8	lan_mac[ETH_ALEN];	/* the LAN L3 interface's MAC		*/
	u16	wan_vlan;		/* 0 = untagged				*/
	u16	lan_vlan;
	u8	lan_prefix;		/* LAN prefix length, 1..32		*/
};

/*
 * Fill *e from the live kernel state.  @lan and @wan are the driver's OWN
 * netdevs; the L3 interface above each (a bridge, a VLAN) is found from them,
 * never named.  @peer is a remote address the caller already holds -- a flow's
 * destination -- so the default route is found by ASKING FOR IT rather than by
 * naming a probe address nobody chose.
 *
 * Returns 0, or a negative errno with *why naming what is not established.
 * ⚠ A failure is a NORMAL outcome (gateway not resolved yet, WAN over a device
 * this engine cannot express); *why exists so the caller can say WHICH, instead
 * of reporting one anonymous refusal.
 */
int gpon_edge_read(struct net_device *lan, struct net_device *wan,
		   u32 peer, struct gpon_edge *e, const char **why);

/* Byte-equality, so a caller can tell "the edge moved" from "nothing changed"
 * without re-deriving what any individual field means. */
bool gpon_edge_same(const struct gpon_edge *a, const struct gpon_edge *b);

#endif /* GPON_EDGE_H */
