/* SPDX-License-Identifier: GPL-2.0-only */
/* Luna family: the switch and MAC statistics block, published ...
 * dev/MEASURED-luna_eth_mib.h.md sec 1. */
#ifndef _LUNA_ETH_MIB_H
#define _LUNA_ETH_MIB_H

#include <linux/netdevice.h>
#include <linux/types.h>

struct luna_sw_map;

void luna_mib_attach(struct net_device *ndev, void __iomem *sw,
		     void __iomem *mac, const struct luna_sw_map *map);
void luna_mib_gmac_tick(void);

#endif /* _LUNA_ETH_MIB_H */
