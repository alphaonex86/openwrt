/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_arp_rows.h -- the LAN hosts an L3 engine's ARP table names, each row naming the L2 row of the host's MAC.
 * That L2 row carries the engine's "used by ARP" mark exactly while a row of ours names it (a mark found set is
 * left set): the die drops a frame whose ARP row names an unmarked L2 row. ONE place writes and clears both, for
 * every LAN-side ARP row. A row is owned by flows (`users`) and/or by the kernel neighbour it follows (`held`).
 * The chip reads and writes its tables; this decides. PURE: host-tested.
 */
#ifndef ETH_ARP_ROWS_H
#define ETH_ARP_ROWS_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>

struct eth_arp_row {
	u32	ip;		/* host order */
	u16	l2idx, users;
	u8	mac[6];
	bool	held;		/* the kernel neighbour owns it */
	bool	ready;		/* the ARP row is written */
	bool	live;		/* the ARP row may be on the die (a timed-out write may land) */
	bool	marked;		/* l2idx carries the mark for this row */
	bool	was_marked;	/* the mark was set before any row of ours named l2idx: left set */
};

struct eth_arp_rows_hw {
	/* the L2 row of @mac -> its index; -ENOENT none yet, -EOPNOTSUPP a WAN-side row, or the read's error */
	int (*l2_find)(void *ctx, const u8 *mac);
	/* sets or clears the mark of L2 row @l2idx while it holds @mac, *was = the mark before (set before any write);
	 * -ESTALE it holds another MAC now (nothing written); another error: the write may have landed */
	int (*l2_mark)(void *ctx, u16 l2idx, const u8 *mac, bool on, bool *was);
	int (*arp_write)(void *ctx, unsigned int slot, u32 ip, u16 l2idx);
	int (*arp_clear)(void *ctx, unsigned int slot);
	int (*arp_busy)(void *ctx, unsigned int slot);	/* -> 1 the die's row is valid, 0 free, or an error */
};

struct eth_arp_rows {
	const struct eth_arp_rows_hw *hw;
	void			*ctx;
	struct eth_arp_row	*row;
	unsigned int		n;
};

static inline bool eth_arp_row_owned(const struct eth_arp_row *a)
{
	return a->users || a->held;
}

static inline int eth_arp_rows_find(const struct eth_arp_rows *r, u32 ip)
{
	unsigned int i;

	for (i = 0; i < r->n; i++)
		if (eth_arp_row_owned(&r->row[i]) && r->row[i].ip == ip)
			return i;
	return -ENOENT;
}

/* Another row of ours that marked @l2idx, or NULL. */
static inline const struct eth_arp_row *eth_arp_l2_sharer(const struct eth_arp_rows *r, unsigned int slot, u16 l2idx)
{
	unsigned int i;

	for (i = 0; i < r->n; i++)
		if (i != slot && r->row[i].marked && r->row[i].l2idx == l2idx)
			return &r->row[i];
	return NULL;
}

/* The ARP row first, then its mark unless another row still names the L2 row. A failed clear keeps its state. */
static inline int eth_arp_row_unbind(struct eth_arp_rows *r, unsigned int slot)
{
	struct eth_arp_row *a = &r->row[slot];
	bool was;
	int ret;

	a->ready = false;
	if (a->live) {
		ret = r->hw->arp_clear(r->ctx, slot);
		if (ret)
			return ret;
		a->live = false;
	}
	if (a->marked) {
		if (!a->was_marked && !eth_arp_l2_sharer(r, slot, a->l2idx)) {
			ret = r->hw->l2_mark(r->ctx, a->l2idx, a->mac, false, &was);
			if (ret && ret != -ESTALE)
				return ret;
		}
		a->marked = false;
	}
	return 0;
}

/* Points row @slot at @ip on @mac's L2 row, the mark before the ARP row. An unchanged row is not rewritten; a host
 * whose L2 row is gone loses its row (a row naming a reused L2 index would black-hole it). */
static inline int eth_arp_row_bind(struct eth_arp_rows *r, unsigned int slot, u32 ip, const u8 *mac)
{
	struct eth_arp_row *a = &r->row[slot];
	const struct eth_arp_row *s;
	int l2, ret;
	bool was;

	l2 = r->hw->l2_find(r->ctx, mac);
	if (a->ready && l2 >= 0 && a->ip == ip && a->l2idx == l2 && !memcmp(a->mac, mac, 6))
		return 0;
	ret = eth_arp_row_unbind(r, slot);
	if (ret)
		return ret;
	if (l2 < 0)
		return l2;
	a->ip = ip;
	a->l2idx = l2;
	memcpy(a->mac, mac, 6);
	s = eth_arp_l2_sharer(r, slot, l2);
	if (s) {
		a->was_marked = s->was_marked;
	} else {
		was = true;	/* unknown: never cleared by us */
		ret = r->hw->l2_mark(r->ctx, l2, mac, true, &was);
		a->was_marked = was;
		if (ret == -ESTALE)
			return ret;
		a->marked = true;	/* a timed-out write may have landed: the owner's release clears it */
		if (ret)
			return ret;
	}
	a->marked = true;
	a->live = true;
	ret = r->hw->arp_write(r->ctx, slot, ip, l2);
	if (!ret)
		a->ready = true;
	return ret;
}

static inline int eth_arp_row_free(struct eth_arp_rows *r)
{
	unsigned int i;
	int busy;

	for (i = 0; i < r->n; i++) {
		if (eth_arp_row_owned(&r->row[i]))
			continue;
		busy = r->hw->arp_busy(r->ctx, i);
		if (busy < 0)
			return busy;
		if (!busy)
			return i;
	}
	return -ENOSPC;
}

/* A new owner of a free row; a bind that wrote nothing gives it back. */
static inline int eth_arp_row_take(struct eth_arp_rows *r, u32 ip, const u8 *mac, bool held, unsigned int *slot)
{
	struct eth_arp_row *a;
	int i = eth_arp_row_free(r), ret;

	if (i < 0)
		return i;
	a = &r->row[i];
	memset(a, 0, sizeof(*a));
	a->ip = ip;
	a->users = !held;
	a->held = held;
	*slot = i;
	ret = eth_arp_row_bind(r, i, ip, mac);
	if (ret && !a->live && !a->marked)
		memset(a, 0, sizeof(*a));
	return ret;
}

/* A flow takes @ip's row (shared with every flow and the neighbour holding it), moved to @mac if it changed.
 * -> 0, *owned true and *slot set whenever the flow now owns a row (also on an error: its removal clears it);
 * -EBUSY a row of @ip whose last write failed or whose neighbour is invalid, -ENOSPC every row taken, or bind's
 * error. */
static inline int eth_arp_row_get(struct eth_arp_rows *r, u32 ip, const u8 *mac, unsigned int *slot, bool *owned)
{
	int i = eth_arp_rows_find(r, ip), ret;
	struct eth_arp_row *a;

	*owned = false;
	if (i < 0) {
		*slot = r->n;
		ret = eth_arp_row_take(r, ip, mac, false, slot);
		*owned = *slot < r->n && r->row[*slot].users;
		return ret;
	}
	a = &r->row[i];
	if (!a->ready)
		return -EBUSY;
	if (a->users == 0xffff)
		return -ENOSPC;
	if (memcmp(a->mac, mac, 6)) {
		ret = eth_arp_row_bind(r, i, ip, mac);
		if (ret)
			return ret;
	}
	a->users++;
	*slot = i;
	*owned = true;
	return 0;
}

/* The row is cleared with its last owner. */
static inline int eth_arp_row_drop(struct eth_arp_rows *r, unsigned int slot)
{
	struct eth_arp_row *a = &r->row[slot];
	int ret;

	if (eth_arp_row_owned(a))
		return 0;
	ret = eth_arp_row_unbind(r, slot);
	if (!ret)
		memset(a, 0, sizeof(*a));
	return ret;
}

/* A flow lets its row go; the last owner's clear may fail, the row then stays owned (and not shared) for a retry. */
static inline int eth_arp_row_put(struct eth_arp_rows *r, unsigned int slot)
{
	struct eth_arp_row *a = &r->row[slot];
	int ret;

	if (a->users > 1 || (a->users && a->held)) {
		a->users--;
		return 0;
	}
	ret = eth_arp_row_unbind(r, slot);
	if (!ret)
		memset(a, 0, sizeof(*a));
	return ret;
}

/* A failed clear stays held, so the next release retries it. */
static inline int eth_arp_row_unhold_one(struct eth_arp_rows *r, unsigned int slot)
{
	int ret;

	r->row[slot].held = false;
	ret = eth_arp_row_drop(r, slot);
	if (ret)
		r->row[slot].held = true;
	return ret;
}

/* The kernel neighbour of @ip: @mac its address, NULL once it is no longer valid (deleted, failed). @hold: it owns
 * a row (a block host), taken when none. Any row of @ip follows it: ARP row and mark cleared while the neighbour is
 * invalid (its flows keep the slot, unbound), bound again when it is valid. */
static inline int eth_arp_row_follow(struct eth_arp_rows *r, u32 ip, const u8 *mac, bool hold)
{
	int i = eth_arp_rows_find(r, ip), ret;
	struct eth_arp_row *a;
	unsigned int slot;
	bool held;

	if (i < 0)
		return mac && hold ? eth_arp_row_take(r, ip, mac, true, &slot) : 0;
	a = &r->row[i];
	if (mac) {
		a->held |= hold;
		return eth_arp_row_bind(r, i, ip, mac);
	}
	held = a->held;
	a->held = false;
	ret = a->users ? eth_arp_row_unbind(r, i) : eth_arp_row_drop(r, i);
	if (ret)
		a->held = held;
	return ret;
}

/* Every held row released (flows keep theirs). -> 0 or the first clear's error. */
static inline int eth_arp_rows_unhold(struct eth_arp_rows *r)
{
	unsigned int i;
	int ret;

	for (i = 0; i < r->n; i++) {
		ret = r->row[i].held ? eth_arp_row_unhold_one(r, i) : 0;
		if (ret)
			return ret;
	}
	return 0;
}

/* An L2 row the engine takes in its own form (e.g. a gateway's: static, marked) and gives back as it found it: the
 * row learnt before word for word, else none. */
#define ETH_L2_WORDS 4
struct eth_l2_taken {
	u32	before[ETH_L2_WORDS];
	u8	mac[6];
	bool	taken, had;
};
struct eth_l2_own_hw {
	int (*read)(void *ctx, const u8 *mac, u32 *w);	/* -> 1 and *w: @mac's row, 0 none, or the read's error */
	int (*take)(void *ctx, const u8 *mac, u8 port);	/* writes the engine's form -> the row index */
	int (*put)(void *ctx, const u8 *mac, const u32 *w);	/* @w NULL: delete */
	bool (*ours)(const u32 *w);				/* the engine's form */
};

static inline int eth_l2_take(const struct eth_l2_own_hw *hw, void *ctx, struct eth_l2_taken *t, const u8 *mac,
			      u8 port)
{
	u32 w[ETH_L2_WORDS] = {};
	int ret = hw->read(ctx, mac, w);

	if (ret < 0)
		return ret;
	memcpy(t->before, w, sizeof(w));
	memcpy(t->mac, mac, 6);
	t->had = ret;
	t->taken = true;	/* before the write: a timed-out insert may land */
	return hw->take(ctx, mac, port);
}

/* -> 0, or -EIO the engine's form is still there (a learnt row may be back at once: only that form must go). A
 * failure keeps it taken, for a retry. */
static inline int eth_l2_give(const struct eth_l2_own_hw *hw, void *ctx, struct eth_l2_taken *t)
{
	u32 w[ETH_L2_WORDS] = {};
	int ret;

	if (!t->taken)
		return 0;
	ret = hw->put(ctx, t->mac, t->had ? t->before : NULL);
	if (ret)
		return ret;
	ret = hw->read(ctx, t->mac, w);
	if (ret < 0)
		return ret;
	if ((ret && hw->ours(w)) != (t->had && hw->ours(t->before)))
		return -EIO;
	t->taken = false;
	return 0;
}
#endif
