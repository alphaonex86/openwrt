/* SPDX-License-Identifier: GPL-2.0 */
/* Every WAN frame's VID, both directions: luna_gpon.c renders the census as /proc/gpon_vlan. */
#ifndef _LUNA_WAN_VLAN_H
#define _LUNA_WAN_VLAN_H

#include <linux/if_vlan.h>
#include "gpon_omci_vlan.h"

extern struct gpon_vlan_census luna_vlan_census;

static inline void luna_vlan_note(unsigned int dir, const struct sk_buff *skb)
{
	gpon_vlan_census_note(&luna_vlan_census, dir, skb_vlan_tag_present(skb) ? skb_vlan_tag_get_id(skb) :
			      gpon_vlan_frame_vid(skb->data, skb_headlen(skb)));
}

#endif
