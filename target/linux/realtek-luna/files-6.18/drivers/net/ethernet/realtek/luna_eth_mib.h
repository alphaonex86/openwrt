/* SPDX-License-Identifier: GPL-2.0-only */
/* Shared switch ethtool: statistics and per-port link modes; attach before
 * registering the netdev. */
#ifndef _LUNA_ETH_MIB_H
#define _LUNA_ETH_MIB_H

#include <linux/netdevice.h>
#include <linux/printk.h>
#include <linux/types.h>

#include "eth_rx_geom.h"

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

/* Received frames whose length is off their own IP header, by side: ethtool -S rx_len_*. */
extern struct eth_rx_len luna_rx_len;

static inline void luna_rx_len_note(const u8 *frame, u32 len, bool wan)
{
	int excess = eth_rx_len_note(&luna_rx_len, frame, len, wan);

	if (excess)
		pr_warn_once("luna: a received %s frame is %d byte(s) off its IP length (%u B)\n",
			     wan ? "WAN" : "LAN", excess, len);
}

#endif /* _LUNA_ETH_MIB_H */
