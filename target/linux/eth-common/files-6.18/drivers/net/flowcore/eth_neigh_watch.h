/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_neigh_watch.h -- an engine that copied a neighbour's MAC into its own tables (the WAN next hop, a LAN host)
 * follows the kernel: every update of an IPv4 neighbour the owner names is queued, out of the notifier's atomic
 * context, and the owner re-reads that neighbour and applies it. A full queue asks the owner to re-read them all.
 */
#ifndef ETH_NEIGH_WATCH_H
#define ETH_NEIGH_WATCH_H

#include <linux/etherdevice.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <net/arp.h>
#include <net/neighbour.h>
#include <net/netevent.h>

#define ETH_NEIGH_PENDING	16

struct eth_neigh_seen {
	u32	ip;		/* host order */
	int	ifindex;
};

struct eth_neigh_watch {
	struct notifier_block	nb;
	struct work_struct	work;
	spinlock_t		lock;
	struct eth_neigh_seen	pending[ETH_NEIGH_PENDING];
	unsigned int		n;
	bool			overflow;
	bool (*names)(struct eth_neigh_watch *w, u32 ip);	/* atomic context */
	void (*sync)(struct eth_neigh_watch *w, u32 ip, int ifindex);
	void (*sync_all)(struct eth_neigh_watch *w);
};

/* What the kernel holds for @ip on @dev now: true and *mac when valid (a dead entry is not). */
static inline bool eth_neigh_now(struct net_device *dev, u32 ip, u8 *mac, u8 *state)
{
	__be32 key = htonl(ip);
	struct neighbour *n = neigh_lookup(&arp_tbl, &key, dev);
	bool valid;

	*state = NUD_NONE;
	if (!n)
		return false;
	*state = READ_ONCE(n->nud_state);
	valid = (*state & NUD_VALID) && !READ_ONCE(n->dead);
	if (valid)
		neigh_ha_snapshot(mac, n, dev);
	neigh_release(n);
	return valid;
}

static inline void eth_neigh_watch_work(struct work_struct *work)
{
	struct eth_neigh_watch *w = container_of(work, struct eth_neigh_watch, work);
	struct eth_neigh_seen seen[ETH_NEIGH_PENDING];
	unsigned int i, n;
	bool overflow;

	spin_lock_bh(&w->lock);
	n = w->n;
	memcpy(seen, w->pending, n * sizeof(seen[0]));
	overflow = w->overflow;
	w->n = 0;
	w->overflow = false;
	spin_unlock_bh(&w->lock);
	for (i = 0; i < n; i++)
		w->sync(w, seen[i].ip, seen[i].ifindex);
	if (overflow)
		w->sync_all(w);
}

static inline int eth_neigh_watch_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct eth_neigh_watch *w = container_of(nb, struct eth_neigh_watch, nb);
	struct neighbour *n = ptr;
	struct eth_neigh_seen s;
	unsigned int i;

	if (event != NETEVENT_NEIGH_UPDATE || n->tbl != &arp_tbl)
		return NOTIFY_DONE;
	s.ip = ntohl(*(__be32 *)n->primary_key);
	s.ifindex = n->dev->ifindex;
	if (!w->names(w, s.ip))
		return NOTIFY_DONE;
	spin_lock_bh(&w->lock);
	for (i = 0; i < w->n && (w->pending[i].ip != s.ip || w->pending[i].ifindex != s.ifindex); i++)
		;
	if (i == w->n) {
		if (w->n < ETH_NEIGH_PENDING)
			w->pending[w->n++] = s;
		else
			w->overflow = true;
	}
	spin_unlock_bh(&w->lock);
	schedule_work(&w->work);
	return NOTIFY_DONE;
}

/* -> 0, or the notifier's error: then a moved neighbour keeps its stale rows. */
static inline int eth_neigh_watch_start(struct eth_neigh_watch *w)
{
	spin_lock_init(&w->lock);
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
