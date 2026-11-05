// SPDX-License-Identifier: GPL-2.0-only
/*
 * Flow-key hashing primitives. See flowcore.h for why they are common and why
 * they carry no kernel dependency.
 */
#include <linux/types.h>

#include "flowcore.h"

/* ===== packed-array slot addressing (moved from ... -- dev/MEASURED-flowcore_hash.c.md sec 1. */
struct pi_packed_slot pi_packed_locate(u32 base, unsigned int idx,
				       unsigned int bits)
{
	unsigned int epw = 32u / bits;		/* entries per 32-bit word (top bits wasted) */
	struct pi_packed_slot slot;

	slot.reg   = base + (idx / epw) * 4;
	slot.shift = (idx % epw) * bits;
	slot.mask  = (bits >= 32) ? 0xffffffffu : ((1u << bits) - 1);
	return slot;
}

/* Insert `val` into its slot inside the sampled 32-bit word; the shell writes
 * the result back (read-modify-write stays at the register). */
u32 pi_packed_insert(u32 word, const struct pi_packed_slot *slot, u32 val)
{
	return (word & ~(slot->mask << slot->shift)) |
	       ((val & slot->mask) << slot->shift);
}

/* Extract the slot's field from the sampled 32-bit word. */
u32 pi_packed_extract(u32 word, const struct pi_packed_slot *slot)
{
	return (word >> slot->shift) & slot->mask;
}
