/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: FAMILY */
/* LAN loop detection shared by both Luna Ethernet shells. */
#ifndef _LUNA_LOOP_H
#define _LUNA_LOOP_H

#include <linux/netdevice.h>
#include <linux/types.h>

struct luna_sw_map;
struct luna_loop;

/* What the shell does for the loop worker (process context).
 * @gate: take copper @port down (MAC link and PHY power) or let it up again;
 *        the shell keeps an administratively locked port down either way.
 * @xmit: send @skb out of LAN @port only (a die without the switch engine). */
struct luna_loop_ops {
	int (*gate)(struct net_device *ndev, unsigned int port, bool down);
	int (*xmit)(struct net_device *ndev, struct sk_buff *skb, unsigned int port);
};

struct luna_loop *luna_loop_attach(struct device *dev, struct net_device *ndev,
				   void __iomem *sw, const struct luna_sw_map *map,
				   const struct luna_loop_ops *ops);
void luna_loop_start(struct luna_loop *l);
void luna_loop_rx(struct luna_loop *l, const u8 *f, unsigned int len,
		  unsigned int rx_port);
bool luna_loop_port_down(struct luna_loop *l, unsigned int port);

#endif /* _LUNA_LOOP_H */
