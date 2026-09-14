/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Luna family: the switch and MAC statistics block, published through
 * `ethtool -S`.
 *
 * ★ WHY IT IS ONE FILE FOR THE WHOLE FAMILY.  The three Luna dies declare the
 * SAME 23 RX and 27 TX fields at the SAME byte offsets inside the same 0x80
 * per-port window; only the RX base and the port count move.  So the logic --
 * which quantities exist, what each is called, how the two 64-bit octet pairs
 * are joined -- is written once here, and a new die costs the three numbers in
 * its struct luna_sw_map row and nothing else.
 *
 * ★ WHAT A SHELL OWES.  One call, at probe, after the switch core is mapped:
 *
 *	luna_mib_attach(ndev, ep->sw, ep->base, ep->swm);
 *
 * and one call from a path that runs while traffic flows:
 *
 *	luna_mib_gmac_tick();
 *
 * The tick is a no-op on a die whose MAC MIB array is not established, so a
 * shell never branches on the chip.
 */
#ifndef _LUNA_ETH_MIB_H
#define _LUNA_ETH_MIB_H

#include <linux/netdevice.h>
#include <linux/types.h>

struct luna_sw_map;

void luna_mib_attach(struct net_device *ndev, void __iomem *sw,
		     void __iomem *mac, const struct luna_sw_map *map);
void luna_mib_gmac_tick(void);

#endif /* _LUNA_ETH_MIB_H */
