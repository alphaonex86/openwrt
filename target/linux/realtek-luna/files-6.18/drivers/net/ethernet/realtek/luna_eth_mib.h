/* SPDX-License-Identifier: GPL-2.0-only */
/* Shared switch ethtool: statistics and per-port link modes; attach before
 * registering the netdev. */
#ifndef _LUNA_ETH_MIB_H
#define _LUNA_ETH_MIB_H

#include <linux/netdevice.h>
#include <linux/types.h>

struct luna_sw_map;
struct mutex;

/* A driver's internal-PHY access, keyed by switch port; read and write are
 * called with *lock held. */
struct luna_phy_ops {
	struct mutex *lock;
	int (*read)(struct net_device *ndev, unsigned int port, unsigned int reg,
		    u16 *val);
	int (*write)(struct net_device *ndev, unsigned int port, unsigned int reg,
		     u16 val);
};

int luna_mib_attach(struct net_device *ndev, void __iomem *sw,
		    const struct luna_sw_map *map, const struct luna_phy_ops *phy);
void luna_mib_phy_reapply(struct net_device *ndev);

#endif /* _LUNA_ETH_MIB_H */
