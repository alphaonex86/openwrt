/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_tx_ring.h -- a transmit ring the driver fills and the hardware drains, which tells only how many it sent
 * since it was last asked: where the next descriptor goes, how many are in flight, and whether a frame fits what
 * one descriptor can say. PURE.
 */
#ifndef _ETH_TX_RING_H
#define _ETH_TX_RING_H

#include <linux/types.h>

struct eth_tx_ring {
	unsigned int next, inflight;
};

/* The largest values one descriptor can say. */
struct eth_tx_limits {
	u32 buf, len, offset;
};

/* Can len bytes at byte `offset` of buffer `buf` (buf_size bytes) be described? */
static inline bool eth_tx_fits(const struct eth_tx_limits *max, u32 buf, u32 offset, u32 len, u32 buf_size)
{
	return buf <= max->buf && offset <= max->offset && len && len <= max->len && offset < buf_size &&
	       len <= buf_size - offset;
}

/* Half the ring is in flight: time to ask the hardware how many it sent, and tell eth_tx_ring_sent(). */
static inline bool eth_tx_ring_ask(const struct eth_tx_ring *t, unsigned int slots)
{
	return t->inflight >= slots / 2;
}

static inline void eth_tx_ring_sent(struct eth_tx_ring *t, unsigned int sent)
{
	t->inflight -= sent < t->inflight ? sent : t->inflight;
}

/* One slot always stays empty, so a full ring is never taken for an empty one. */
static inline bool eth_tx_ring_room(const struct eth_tx_ring *t, unsigned int slots)
{
	return t->inflight + 1 < slots;
}

/* Take the next slot. -> its index. */
static inline unsigned int eth_tx_ring_take(struct eth_tx_ring *t, unsigned int slots)
{
	unsigned int slot = t->next;

	t->next = (slot + 1) % slots;
	t->inflight++;
	return slot;
}

#endif
