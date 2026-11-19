/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_queue_watch.h -- is a ring STALLED (a frame waits at the consumer index and nobody takes it) or only
 * SATURATED (full, but taken from)? And when does a recovery that keeps being needed become a reboot.
 * The decision is PURE (time is an input, in ms); the escalation below it is the one kernel action.
 */
#ifndef _ETH_QUEUE_WATCH_H
#define _ETH_QUEUE_WATCH_H

#include <linux/types.h>
#include "restart_budget.h"

/* A poll runs every few ms under load: a waiting frame untouched for 2 s is never saturation, and it is
 * caught well before the soft-lockup detector (20 s) or a hardware watchdog (~35 s) would act. */
#define ETH_RX_STALL_MS		2000
/* The kernel's own TX watchdog period when a driver names none (dev_watchdog: 5 * HZ). */
#define ETH_TX_TIMEOUT_MS	5000

enum eth_watch { ETH_WATCH_OK, ETH_WATCH_RECOVER, ETH_WATCH_REBOOT };

struct eth_queue_watch {
	u64 since_ms;		/* a frame has waited since then with nothing consumed; 0 = none waits */
	u32 consumed;		/* the ring's consumed count at the last sample */
	struct restart_budget rx_budget;	/* a frame taken after a recovery cured it */
	struct restart_budget tx_budget;	/* cured only by RESTART_CURED_MS without a timeout */
	u32 rx_stalls;		/* ethtool -S rx_ring_stalls */
	u32 tx_timeouts;	/* ethtool -S tx_ring_timeouts */
	u32 recoveries;		/* ethtool -S ring_recoveries */
};

/* Every recovery, RX or TX, spends its direction's restart budget; a spent one is eth_watch_spent.h's.
 * RX traffic never cures a TX hang. */
static inline enum eth_watch eth_watch_recovering(struct eth_queue_watch *w, struct restart_budget *b, u64 now_ms)
{
	w->recoveries++;
	return restart_budget_spend(b, now_ms, NULL) ? ETH_WATCH_RECOVER : ETH_WATCH_REBOOT;
}

/* One sample: `pending` = a frame waits at the consumer index, `consumed` = frames taken so far (free running). */
static inline enum eth_watch eth_rx_watch_sample(struct eth_queue_watch *w, bool pending, u32 consumed, u64 now_ms)
{
	bool took = consumed != w->consumed;

	w->consumed = consumed;
	if (took)
		restart_budget_cured(&w->rx_budget);
	if (took || !pending) {
		w->since_ms = 0;
		return ETH_WATCH_OK;
	}
	if (!w->since_ms) {
		w->since_ms = now_ms ? now_ms : 1;
		return ETH_WATCH_OK;
	}
	if (now_ms - w->since_ms < ETH_RX_STALL_MS)
		return ETH_WATCH_OK;
	w->since_ms = 0;
	w->rx_stalls++;
	return eth_watch_recovering(w, &w->rx_budget, now_ms);
}

static inline enum eth_watch eth_tx_watch_timeout(struct eth_queue_watch *w, u64 now_ms)
{
	w->tx_timeouts++;
	return eth_watch_recovering(w, &w->tx_budget, now_ms);
}

/* An egress port that sends NOTHING of what it is handed is not congested (a congested port still sends):
 * it is a black hole, and the link above it stays up. 8 frames over 10 s is one stall; each is recovered through
 * the restart budget; a spent one (ETH_WATCH_REBOOT here) is eth_watch_spent.h's. */
#define ETH_EGRESS_STALL_MS	10000
#define ETH_EGRESS_STALL_FRAMES	8
struct eth_egress_watch {
	u64 since_ms;		/* frames were handed since then and none left; 0 = none waits */
	u32 offered;		/* the handed count when the port last sent, or was last judged */
	u32 sent;		/* the port's own sent count at the last sample */
	struct restart_budget budget;	/* the recoveries of its stalls; a send cures it */
	u32 stalls;		/* ethtool -S egress_stalls */
};
/* One sample of one port: `offered` = frames handed to it, `sent` = what it counted out, both free running.
 * -> true at a stall, the budget not yet asked. */
static inline bool eth_egress_watch_stalled(struct eth_egress_watch *e, u32 offered, u32 sent, u64 now_ms)
{
	if (sent != e->sent) {
		e->sent = sent;
		restart_budget_cured(&e->budget);
	} else if (offered != e->offered) {
		if (!e->since_ms)
			e->since_ms = now_ms ? now_ms : 1;
		if (now_ms - e->since_ms < ETH_EGRESS_STALL_MS || offered - e->offered < ETH_EGRESS_STALL_FRAMES)
			return false;
		e->stalls++;
		e->offered = offered;
		e->since_ms = 0;
		return true;
	}
	e->offered = offered;
	e->since_ms = 0;
	return false;
}
static inline enum eth_watch eth_egress_watch_spend(struct eth_egress_watch *e, u64 now_ms)
{
	return restart_budget_spend(&e->budget, now_ms, NULL) ? ETH_WATCH_RECOVER : ETH_WATCH_REBOOT;
}
static inline enum eth_watch eth_egress_watch_sample(struct eth_egress_watch *e, u32 offered, u32 sent, u64 now_ms)
{
	return eth_egress_watch_stalled(e, offered, sent, now_ms) ? eth_egress_watch_spend(e, now_ms) : ETH_WATCH_OK;
}
/* The same for a port nobody hands frames to one by one: `queued` = what its queue holds now. */
static inline enum eth_watch eth_egress_held_sample(struct eth_egress_watch *e, u32 queued, u32 sent, u64 now_ms)
{
	return eth_egress_watch_sample(e, e->offered + (queued >= ETH_EGRESS_STALL_FRAMES ? queued : 0), sent, now_ms);
}
/* One switch port: `ready` = it has a link (the CPU port: its ring owes nothing); a port its partner paused since
 * the last sample holds its queue by right and is not judged either. `drain` = the least the port counts out when
 * the queue it holds leaves: a port that keeps its queue and counts out less in a whole second is held too, its
 * trickle is not sending (units in service: 1014 pages kept 7 min at 25 octets/s, 430 pages 79 min at a few kB/s). */
#define ETH_EGRESS_SPAN_MS	1000
struct eth_port_egress {
	struct eth_egress_watch watch;
	u64 span_ms;		/* start of the current span */
	u32 span_sent;		/* the port's sent count then */
	u32 paused;
	bool trickle;		/* the last whole span counted out less than the queue kept */
};

static inline enum eth_watch eth_port_egress_sample(struct eth_port_egress *p, bool ready, u32 paused, u32 queued,
						    u32 drain, u32 sent, u64 now_ms)
{
	bool judged = ready && paused == p->paused;
	bool held;

	if (now_ms - p->span_ms >= ETH_EGRESS_SPAN_MS) {
		p->trickle = sent - p->span_sent < drain;
		p->span_sent = sent;
		p->span_ms = now_ms;
	}
	held = judged && queued >= ETH_EGRESS_STALL_FRAMES && p->trickle;
	p->paused = paused;
	return eth_egress_held_sample(&p->watch, judged ? queued : 0, held ? p->watch.sent : sent, now_ms);
}
#ifdef __KERNEL__
#include <linux/ethtool.h>
#include <linux/jiffies.h>
#include <linux/netdevice.h>
#include <linux/panic.h>
#include <linux/spinlock.h>
#include "eth_watch_spent.h"

/* A driver's watch: sampled from its timer, a TX timeout from the kernel's; the lock keeps them apart on SMP.
 * Its `w` is the rx_watch of the netdev contract. */
struct eth_ring_watch {
	spinlock_t		lock;
	struct eth_queue_watch	w;
};

#define ETH_RING_WATCH_INIT(name)	{ .lock = __SPIN_LOCK_UNLOCKED(name.lock) }
#define ETH_RING_WATCH_STATS		3	/* ethtool -S rows, below */

static inline void eth_ring_watch_strings(u8 **data)
{
	ethtool_puts(data, "rx_ring_stalls");
	ethtool_puts(data, "tx_ring_timeouts");
	ethtool_puts(data, "ring_recoveries");
}

static inline u64 *eth_ring_watch_stats(const struct eth_ring_watch *rw, u64 *data)
{
	*data++ = READ_ONCE(rw->w.rx_stalls);
	*data++ = READ_ONCE(rw->w.tx_timeouts);
	*data++ = READ_ONCE(rw->w.recoveries);
	return data;
}

static inline void eth_watch_act(struct net_device *ndev, const struct eth_queue_watch *w, enum eth_watch v,
				 const char *what)
{
	if (v == ETH_WATCH_REBOOT)
		eth_watch_spent(ndev, "%s: %u ring recoveries in a row did not cure it (rx stalls %u, tx timeouts %u)",
				what, RESTART_TRIES, w->rx_stalls, w->tx_timeouts);
	if (v == ETH_WATCH_RECOVER)
		netdev_err(ndev, "%s: ring recovery #%u (rx stalls %u, tx timeouts %u)\n", what, w->recoveries,
			   w->rx_stalls, w->tx_timeouts);
}

/* A stall is logged and is the driver's to recover; once RESTART_TRIES recoveries in a row did not make the port
 * send, eth_watch_spent.h decides. */
static inline void eth_egress_watch_act(struct net_device *ndev, const struct eth_egress_watch *e, enum eth_watch v,
					const char *port)
{
	if (v == ETH_WATCH_REBOOT)
		eth_watch_spent(ndev, "the %s port sent none of the frames handed to it after %u recoveries (stall #%u)",
				port, RESTART_TRIES, e->stalls);
	if (v == ETH_WATCH_RECOVER)
		netdev_err(ndev, "the %s port sent none of the frames handed to it for %u s (stall #%u, recovery %u of %u)\n",
			   port, ETH_EGRESS_STALL_MS / 1000, e->stalls, e->budget.in_row, RESTART_TRIES);
}
/* -> true when the caller must recover the port (reset its switch core). */
static inline bool eth_port_egress_judge(struct net_device *ndev, struct eth_port_egress *p, const char *name,
					 bool ready, u32 paused, u32 queued, u32 drain, u32 sent)
{
	enum eth_watch v = eth_port_egress_sample(p, ready, paused, queued, drain, sent,
						  jiffies64_to_msecs(get_jiffies_64()));

	eth_egress_watch_act(ndev, &p->watch, v, name);
	return v == ETH_WATCH_RECOVER;
}
/* -> true when the caller should recover (schedule its NAPI). */
static inline bool eth_ring_watch_rx(struct eth_ring_watch *rw, struct net_device *ndev, bool pending, u32 consumed)
{
	struct eth_queue_watch now;
	unsigned long flags;
	enum eth_watch v;

	spin_lock_irqsave(&rw->lock, flags);
	v = eth_rx_watch_sample(&rw->w, pending, consumed, jiffies64_to_msecs(get_jiffies_64()));
	now = rw->w;
	spin_unlock_irqrestore(&rw->lock, flags);
	eth_watch_act(ndev, &now, v, "rx ring stalled");
	return v == ETH_WATCH_RECOVER;
}

static inline void eth_ring_watch_tx(struct eth_ring_watch *rw, struct net_device *ndev)
{
	struct eth_queue_watch now;
	unsigned long flags;
	enum eth_watch v;

	spin_lock_irqsave(&rw->lock, flags);
	v = eth_tx_watch_timeout(&rw->w, jiffies64_to_msecs(get_jiffies_64()));
	now = rw->w;
	spin_unlock_irqrestore(&rw->lock, flags);
	eth_watch_act(ndev, &now, v, "tx timeout");
}
#endif

#endif
