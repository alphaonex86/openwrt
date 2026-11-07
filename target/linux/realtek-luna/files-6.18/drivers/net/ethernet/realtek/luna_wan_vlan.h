/* SPDX-License-Identifier: GPL-2.0 */
/* Every WAN frame's VID, both directions: luna_gpon.c renders the census as /proc/gpon_vlan. */
#ifndef _LUNA_WAN_VLAN_H
#define _LUNA_WAN_VLAN_H

#include <linux/if_vlan.h>
#include "eth_rx_geom.h"
#include "gpon_omci_vlan.h"
#include "luna_gpon_nic.h"

extern struct gpon_vlan_census luna_vlan_census;
/* The OLT's downstream multicast tag control (ME 309/65531 #16), refreshed per OMCI message. */
extern u8 luna_mcast_ds_control;

static inline void luna_vlan_note(unsigned int dir, const struct sk_buff *skb)
{
	gpon_vlan_census_note(&luna_vlan_census, dir, skb_vlan_tag_present(skb) ? skb_vlan_tag_get_id(skb) :
			      gpon_vlan_frame_vid(skb->data, skb_headlen(skb)));
}

/* Cut a multicast outer tag the OLT says to strip (ME 309/65531 #16) that no VLAN netdev of @wan claims. */
static inline void luna_mcast_untag(struct sk_buff *skb, struct net_device *wan)
{
	if (!eth_rx_vid_cut(wan, gpon_mcast_ds_strip_vid(skb->data, skb_headlen(skb),
							 READ_ONCE(luna_mcast_ds_control))))
		return;
	memmove(skb->data + VLAN_HLEN, skb->data, 2 * ETH_ALEN);
	skb_pull(skb, VLAN_HLEN);
}

/* The US flow of a WAN frame: its VID's service flow, else the WAN's control or data flow. */
static inline u8 luna_wan_tx_flow(const u8 *f, unsigned int len, bool ctrl_flow)
{
	u8 svc = luna_gpon_svc_flow(gpon_vlan_frame_vid(f, len));

	if (svc)
		return svc;
	return ctrl_flow && luna_wan_control_frame(f, len) && luna_gpon_ctrl_flow_ready() ?
		GPON_CTRL_FLOW : GPON_DATA_FLOW;
}

#endif
