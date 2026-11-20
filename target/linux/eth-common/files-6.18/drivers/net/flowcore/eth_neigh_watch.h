/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_neigh_watch.h -- an engine that copied a neighbour's MAC into its own table (the WAN next hop, a LAN host)
 * forgets it when the kernel does: a neighbour the owner holds whose MAC moved, or that is no longer valid,
 * schedules the owner's flush out of the notifier's atomic context. The owner answers "which MAC do my rows hold
 * for this IPv4 address", NULL for none; that answer runs in atomic context.
 */
#ifndef ETH_NEIGH_WATCH_H
#define ETH_NEIGH_WATCH_H

#include <linux/etherdevice.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <net/arp.h>
#include <net/neighbour.h>
#include <net/netevent.h>

struct eth_neigh_watch {
	struct notifier_block	nb;
	struct work_struct	work;
	const u8 *(*held)(struct eth_neigh_watch *w, u32 ip);	/* host order */
	void (*flush)(struct eth_neigh_watch *w);
};

static inline void eth_neigh_watch_work(struct work_struct *work)
{
	struct eth_neigh_watch *w = container_of(work, struct eth_neigh_watch, work);

	w->flush(w);
}

static inline int eth_neigh_watch_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct eth_neigh_watch *w = container_of(nb, struct eth_neigh_watch, nb);
	struct neighbour *n = ptr;
	const u8 *mac;
	u8 now[ETH_ALEN];

	if (event != NETEVENT_NEIGH_UPDATE || n->tbl != &arp_tbl)
		return NOTIFY_DONE;
	mac = w->held(w, ntohl(*(__be32 *)n->primary_key));
	if (!mac)
		return NOTIFY_DONE;
	neigh_ha_snapshot(now, n, n->dev);
	if (!(READ_ONCE(n->nud_state) & NUD_VALID) || !ether_addr_equal(now, mac))
		schedule_work(&w->work);
	return NOTIFY_DONE;
}

/* -> 0, or the notifier's error: then a moved neighbour keeps its stale rows. */
static inline int eth_neigh_watch_start(struct eth_neigh_watch *w)
{
	INIT_WORK(&w->work, eth_neigh_watch_work);
	w->nb.notifier_call = eth_neigh_watch_event;
	return register_netevent_notifier(&w->nb);
}

static inline void eth_neigh_watch_stop(struct eth_neigh_watch *w)
{
	if (!w->nb.notifier_call)
		return;		/* never started */
	unregister_netevent_notifier(&w->nb);
	cancel_work_sync(&w->work);
}

#endif
