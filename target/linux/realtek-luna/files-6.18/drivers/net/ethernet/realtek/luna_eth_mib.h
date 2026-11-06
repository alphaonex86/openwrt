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
struct luna_rx_len {
	u64 long_lan, short_lan, long_wan, short_wan;
};
extern struct luna_rx_len luna_rx_len;

/* Count, never trim: trimming would hide the bug this exists to show. */
static inline void luna_rx_len_note(const u8 *frame, u32 len, bool wan)
{
	int excess;

	if (!eth_frame_l3_excess(frame, len, &excess) || !excess)
		return;
	if (excess > 0)
		wan ? luna_rx_len.long_wan++ : luna_rx_len.long_lan++;
	else
		wan ? luna_rx_len.short_wan++ : luna_rx_len.short_lan++;
	pr_warn_once("luna: a received %s frame is %d byte(s) off its IP length (%u B)\n",
		     wan ? "WAN" : "LAN", excess, len);
}

#endif /* _LUNA_ETH_MIB_H */
