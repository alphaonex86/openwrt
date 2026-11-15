/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_pon_link.h -- a PON WAN netdev's link by the standard call. Every reader of ethtool depends on it: netifd
 * reports a device's type only once ETHTOOL_GLINKSETTINGS answers, so without it a VLAN upper of the PON netdev
 * has no type and fw4 never walks down to the PON netdev: PPPoE over that VLAN stays out of the flowtable. */
#ifndef _ETH_PON_LINK_H
#define _ETH_PON_LINK_H

#include <linux/ethtool.h>
#include <linux/netdevice.h>

#define ETH_PON_GPON_DS_MBPS	2488	/* G.984.2 downstream line rate, 2488.32 Mbit/s */

/* The ethtool core zeroes `ks` before the call. */
static inline int eth_pon_get_link_ksettings(struct net_device *ndev, struct ethtool_link_ksettings *ks)
{
	bool up = netif_carrier_ok(ndev);

	ethtool_link_ksettings_add_link_mode(ks, supported, FIBRE);
	ks->base.port = PORT_FIBRE;
	ks->base.autoneg = AUTONEG_DISABLE;
	ks->base.speed = up ? ETH_PON_GPON_DS_MBPS : SPEED_UNKNOWN;
	ks->base.duplex = up ? DUPLEX_FULL : DUPLEX_UNKNOWN;
	return 0;
}

#endif
