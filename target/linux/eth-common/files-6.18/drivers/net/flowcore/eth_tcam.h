/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* A TCAM rule as fields (bits, care, value), encoded into the X/Y word pair a switch table stores. PURE. */
#ifndef ETH_TCAM_H
#define ETH_TCAM_H

#include <linux/string.h>
#include <linux/types.h>
#include "eth_row_bits.h"

struct eth_tcam_field {
	u16 lsp;
	u8 width;
	u32 care, value;
};

/* @care_is_one: data = care & value, mask = care & ~value, a free bit 0 in both halves; else the complement,
 * data = ~care | value, mask = ~care | ~value, a free bit 1 in both. */
static inline void eth_tcam_xy(const struct eth_tcam_field *f, unsigned int n, bool care_is_one,
			       u32 *data, u32 *mask, unsigned int words)
{
	unsigned int i;

	memset(data, 0, words * sizeof(*data));
	memset(mask, 0, words * sizeof(*mask));
	for (i = 0; i < n; i++) {
		eth_row_set(data, f[i].lsp, f[i].width, f[i].care & f[i].value);
		eth_row_set(mask, f[i].lsp, f[i].width, f[i].care & ~f[i].value);
	}
	for (i = 0; !care_is_one && i < words; i++) {
		u32 d = data[i];

		data[i] = ~mask[i];
		mask[i] = ~d;
	}
}

/* A trap of one subnet's frames to another subnet, sent to one MAC. */
struct eth_tcam_trap {
	u32 sip, dip;
	u8 splen, dplen, mac[6];
};

#define ETH_TCAM_TRAP_FIELDS 4

/* The trap's DMAC, SIP and DIP as consecutive fields from bit @lsp, each value's low half first. */
static inline void eth_tcam_trap_fields(struct eth_tcam_field *f, const struct eth_tcam_trap *t, u16 lsp)
{
	const u8 *m = t->mac;

	f[0] = (struct eth_tcam_field){ lsp, 32, ~0u, (u32)m[2] << 24 | (u32)m[3] << 16 | (u32)m[4] << 8 | m[5] };
	f[1] = (struct eth_tcam_field){ lsp + 32, 16, 0xffff, (u32)m[0] << 8 | m[1] };
	f[2] = (struct eth_tcam_field){ lsp + 48, 32, t->splen ? ~0u << (32 - t->splen) : 0, t->sip };
	f[3] = (struct eth_tcam_field){ lsp + 80, 32, t->dplen ? ~0u << (32 - t->dplen) : 0, t->dip };
}

/* A rule that follows a trap: ETH_TCAM_SET when wanted and new or changed, ETH_TCAM_CLEAR when no longer wanted;
 * eth_tcam_followed() records the write's outcome (a failed set leaves the rule off: the next call retries). */
enum { ETH_TCAM_KEEP, ETH_TCAM_SET, ETH_TCAM_CLEAR };

struct eth_tcam_follow {
	bool on;
	struct eth_tcam_trap t;
};

static inline int eth_tcam_follow(const struct eth_tcam_follow *s, const struct eth_tcam_trap *t, bool want)
{
	if (want)
		return s->on && !memcmp(&s->t, t, sizeof(*t)) ? ETH_TCAM_KEEP : ETH_TCAM_SET;
	return s->on ? ETH_TCAM_CLEAR : ETH_TCAM_KEEP;
}

static inline void eth_tcam_followed(struct eth_tcam_follow *s, const struct eth_tcam_trap *t, int step, int ret)
{
	if (step == ETH_TCAM_SET) {
		s->on = !ret;
		s->t = *t;
	} else if (step == ETH_TCAM_CLEAR && !ret) {
		s->on = false;
	}
}
#endif
