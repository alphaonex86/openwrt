/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_row_bits.h -- a field of an engine table row: `width` bits at bit `lsp` of a row of u32 words, bit 0 the
 * LSB of word 0, a field free to straddle words. Moved from rtl9602c_l34_logic.c (2026-11-20), unchanged: every
 * table of a row-addressed engine packs this way. PURE.
 */
#ifndef ETH_ROW_BITS_H
#define ETH_ROW_BITS_H

#include <linux/minmax.h>
#include <linux/types.h>

static inline void eth_row_set(u32 *w, unsigned int lsp, unsigned int width, u32 val)
{
	unsigned int word = lsp / 32, bit = lsp % 32, take;

	val &= (width >= 32) ? ~0u : ((1u << width) - 1);
	while (width) {
		take = min(width, 32 - bit);
		w[word] &= ~((((take >= 32) ? ~0u : ((1u << take) - 1))) << bit);
		w[word] |= (val & ((take >= 32) ? ~0u : ((1u << take) - 1))) << bit;
		/* ⚠ `val >>= take` was UNDEFINED on the 32-bit-wide fields ...
		 * dev/MEASURED-rtl9602c_l34_logic.c.md sec 1. */
		val = (take >= 32) ? 0 : (val >> take);
		width -= take;
		word++;
		bit = 0;
	}
}

/* Inverse of eth_row_set. The pair shares one home -- the SID2QID lesson: a wrong get mirroring a wrong set is
 * invisible when the two live apart. */
static inline u32 eth_row_get(const u32 *w, unsigned int lsp, unsigned int width)
{
	unsigned int word = lsp / 32, bit = lsp % 32, take, got = 0;
	u32 val = 0;

	while (width) {
		take = min(width, 32 - bit);
		val |= ((w[word] >> bit) & ((take >= 32) ? ~0u : ((1u << take) - 1)))
		       << got;
		got += take;
		width -= take;
		word++;
		bit = 0;
	}
	return val;
}

#endif
