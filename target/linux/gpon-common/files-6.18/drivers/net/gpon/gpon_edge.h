/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TIER: CORE.  Like gpon_flow_offload it is not in the STRICT host-buildable
 * subset -- it reads Linux's own routing state on purpose.  The line is the
 * REGISTER, not Linux: there is no MMIO, no device pointer, no table word and
 * no chip fact anywhere below.
 *
 * gpon_edge -- the router EDGE, read from the LIVE kernel.
 *
 * ★ WHY IT IS CORE AND NOT PER-CHIP.  Every flow accelerator in this tree has
 * to be told the same five things before it can rewrite a packet: which source
 * address and MAC the WAN side egresses with, which next hop it egresses
 * THROUGH, and which subnet is behind the LAN.  None of that is a property of
 * any silicon -- it is what Linux already decided -- so deriving it once here
 * costs the next board a TABLE and not a copy of this file.
 *
 * ★ AND IT IS PULLED, NOT PUSHED.  A notifier chain was the obvious shape and
 * it is the wrong one: the values are wanted at exactly one moment, when a
 * flow is being installed, and at that moment they are all resolved by
 * construction (the flow exists because Linux forwarded a packet along them).
 * Pulling keeps the whole mechanism free of a workqueue, a timer and a lock,
 * and it cannot go stale -- the caller compares what it reads against what it
 * last programmed.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#ifndef GPON_EDGE_H
#define GPON_EDGE_H

#include <linux/types.h>
#include <linux/if_ether.h>

struct net_device;

/*
 * Addresses are HOST order.  Every engine here packs its own table words with
 * explicit shift/mask, so a network-order value in this struct would only be a
 * second place to get the byte order wrong.
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
 * ⚠ A failure is a NORMAL outcome (the gateway is not resolved yet, the WAN
 * runs over a device this engine cannot express); *why exists so the caller
 * can say WHICH, instead of reporting one anonymous refusal.
 */
int gpon_edge_read(struct net_device *lan, struct net_device *wan,
		   u32 peer, struct gpon_edge *e, const char **why);

/* Byte-equality, so a caller can tell "the edge moved" from "nothing changed"
 * without re-deriving what any individual field means. */
bool gpon_edge_same(const struct gpon_edge *a, const struct gpon_edge *b);

#endif /* GPON_EDGE_H */
