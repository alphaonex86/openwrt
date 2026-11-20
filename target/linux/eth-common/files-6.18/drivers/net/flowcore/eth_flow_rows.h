/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_flow_rows.h -- a small engine table of per-flow rows: which rows are owned, the lookup the chip enables only
 * while a row is live, and a sticky shadow of the chip's hit bits, each owner consuming its own bit, so a
 * clear-on-read die loses no other row's hit. The chip writes a row, switches its lookup and reads its hit words;
 * the pool decides. A row is released only once its clear landed. PURE (time is an input): host-tested.
 */
#ifndef ETH_FLOW_ROWS_H
#define ETH_FLOW_ROWS_H

#include <linux/errno.h>
#include <linux/types.h>

#define ETH_FLOW_ROWS_WORDS(n)	(((n) + 31) / 32)

struct eth_flow_rows_hw {
	int (*write)(void *ctx, unsigned int row, const u32 *words);	/* NULL words = clear the row */
	void (*lookup)(void *ctx, bool on);
	u32 (*hit_word)(void *ctx, unsigned int word);
};

struct eth_flow_rows {
	const struct eth_flow_rows_hw *hw;
	void		*ctx;
	unsigned int	n;		/* rows the table has */
	unsigned int	live;		/* rows owned */
	unsigned long	swept;		/* `now` of the last hit-word read, 0 = never */
	u32		*owned;		/* n bits */
	u32		*hit;		/* n bits, sticky until the owner asks */
};

static inline bool eth_flow_rows_owned(const struct eth_flow_rows *r, unsigned int row)
{
	return row < r->n && (r->owned[row / 32] & (1u << (row % 32)));
}

/* Writes @words into the first free row, *row = it. -> 0; -ENOSPC when every row is taken; the write's error, the
 * row then still owned (a timed-out write may land): the caller clears it with eth_flow_rows_del. */
static inline int eth_flow_rows_add(struct eth_flow_rows *r, const u32 *words, unsigned int *row)
{
	unsigned int i;
	int ret;

	for (i = 0; i < r->n && eth_flow_rows_owned(r, i); i++)
		;
	if (i == r->n)
		return -ENOSPC;
	r->owned[i / 32] |= 1u << (i % 32);
	r->hit[i / 32] &= ~(1u << (i % 32));
	r->live++;
	*row = i;
	ret = r->hw->write(r->ctx, i, words);
	if (!ret)
		r->hw->lookup(r->ctx, true);
	return ret;
}

/* Clears @row; the last live row switches the lookup off. -> 0 or the clear's error (the row stays owned). */
static inline int eth_flow_rows_del(struct eth_flow_rows *r, unsigned int row)
{
	int ret;

	if (!eth_flow_rows_owned(r, row))
		return 0;
	ret = r->hw->write(r->ctx, row, NULL);
	if (ret)
		return ret;
	r->owned[row / 32] &= ~(1u << (row % 32));
	r->hit[row / 32] &= ~(1u << (row % 32));
	if (!--r->live)
		r->hw->lookup(r->ctx, false);
	return 0;
}

/* Clears every owned row, *cleared counting them. -> 0 or the first clear's error. */
static inline int eth_flow_rows_flush(struct eth_flow_rows *r, unsigned int *cleared)
{
	unsigned int i;
	int ret;

	for (i = 0; i < r->n; i++) {
		if (!eth_flow_rows_owned(r, i))
			continue;
		ret = eth_flow_rows_del(r, i);
		if (ret)
			return ret;
		(*cleared)++;
	}
	return 0;
}

/* Was @row hit since it was last asked? The hit words are read at most once per `now`. */
static inline bool eth_flow_rows_hit(struct eth_flow_rows *r, unsigned int row, unsigned long now)
{
	unsigned int i;
	bool hit;

	if (!eth_flow_rows_owned(r, row))
		return false;
	if (!r->swept || r->swept != now) {
		r->swept = now;
		for (i = 0; i < ETH_FLOW_ROWS_WORDS(r->n); i++)
			r->hit[i] |= r->hw->hit_word(r->ctx, i);
	}
	hit = r->hit[row / 32] & (1u << (row % 32));
	r->hit[row / 32] &= ~(1u << (row % 32));
	return hit;
}

#endif
