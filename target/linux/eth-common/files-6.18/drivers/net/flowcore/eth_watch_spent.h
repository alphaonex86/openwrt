/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_watch_spent.h -- what a spent recovery budget does, ONE policy for every watch (fibre, CPU port, copper, PON
 * upstream, NIC ring), set by the image (operator, 2026-11-19). A diagnosis image (CONFIG_ETH_WATCH_SPENT_STAY_UP:
 * the lab, a field unit under investigation) stays up and stops recovering until the budget's quiet time renews
 * it: the fault state is evidence. A production image restarts, the crash record naming the watch, so a customer's
 * unit comes back working by itself. eth_watch.spent_stay_up overrides the image at run time. An OLT that grants
 * nothing never gets here: it spends no budget.
 */
#ifndef ETH_WATCH_SPENT_H
#define ETH_WATCH_SPENT_H

#include <linux/netdevice.h>
#include <linux/panic.h>

extern bool eth_watch_spent_stay_up;

#define eth_watch_spent(ndev, fmt, ...)									\
do {														\
	if (eth_watch_spent_stay_up)										\
		netdev_err(ndev, fmt ": not recovered, the unit stays up (spent budget policy: stay up)\n",	\
			   ##__VA_ARGS__);									\
	else													\
		panic("%s: " fmt ": restarting (spent budget policy: restart)\n", netdev_name(ndev),		\
		      ##__VA_ARGS__);										\
} while (0)

#endif
