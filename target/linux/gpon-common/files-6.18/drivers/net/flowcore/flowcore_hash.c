// SPDX-License-Identifier: GPL-2.0-only
/*
 * Flow-key hashing primitives. See flowcore.h for why they are common and why
 * they carry no kernel dependency.
 */
#include <linux/types.h>

#include "flowcore.h"

u8 flowcore_bitrev8(u8 v)
{
	v = (u8)((v >> 4) | (v << 4));
	v = (u8)(((v & 0xccu) >> 2) | ((v & 0x33u) << 2));
	v = (u8)(((v & 0xaau) >> 1) | ((v & 0x55u) << 1));
	return v;
}

u32 flowcore_bitrev32(u32 v)
{
	v = (v >> 16) | (v << 16);
	v = ((v & 0xff00ff00u) >> 8) | ((v & 0x00ff00ffu) << 8);
	v = ((v & 0xf0f0f0f0u) >> 4) | ((v & 0x0f0f0f0fu) << 4);
	v = ((v & 0xccccccccu) >> 2) | ((v & 0x33333333u) << 2);
	v = ((v & 0xaaaaaaaau) >> 1) | ((v & 0x55555555u) << 1);
	return v;
}

void flowcore_key_bitrev(u32 *w, int n_words)
{
	int i;
	u32 t;

	if (!w || n_words <= 0)
		return;
	for (i = 0; i < n_words / 2; i++) {
		t = w[i];
		w[i] = flowcore_bitrev32(w[n_words - 1 - i]);
		w[n_words - 1 - i] = flowcore_bitrev32(t);
	}
	/* ★ THE ODD MIDDLE WORD IS NOT A CORNER CASE, IT IS HALF THE KEYS: an
	 * odd word count leaves one word untouched by the swap loop, and it
	 * still has to be reversed. Dropping this line changes the hash for
	 * every odd-length key and for no even one -- the kind of difference
	 * that shows up as a few flows mysteriously not matching. */
	if (n_words & 1)
		w[i] = flowcore_bitrev32(w[i]);
}

u32 flowcore_crc32_reflected(const u8 *p, u32 len)
{
	u32 crc = ~0u;
	u32 i;
	int b;

	if (!p)
		return 0;
	/* Bitwise rather than table-driven: this is the reference/fuzz path --
	 * the runtime hash is computed by the silicon's own CRC unit -- and a
	 * 1 KiB table in a common object every board links is not worth it. */
	for (i = 0; i < len; i++) {
		crc ^= p[i];
		for (b = 0; b < 8; b++)
			crc = (crc >> 1) ^ (0xedb88320u & (u32)(-(int)(crc & 1u)));
	}
	return flowcore_bitrev32(crc);
}

u16 flowcore_crc16_ccitt_reflected(const u8 *p, u32 len)
{
	u16 crc = 0xffff;
	u32 i;
	int b;

	if (!p)
		return 0;
	for (i = 0; i < len; i++) {
		crc ^= p[i];
		for (b = 0; b < 8; b++)
			crc = (u16)((crc >> 1) ^ ((crc & 1u) ? 0x8408u : 0u));
	}
	return (u16)((flowcore_bitrev8((u8)(crc & 0xff)) << 8) |
		     flowcore_bitrev8((u8)(crc >> 8)));
}

/* ===== packed-array slot addressing (moved from gpon_rtl9602c_logic.c
 * 2026-09-02, round 3 -- see flowcore.h for why it lives in THIS object) ===
 * PACKING CORRECTED 2026-06-13 (from the stock array-field-write routine):
 * the reg-array helper packs entries `entries_per_word = 32/bits` PER 32-bit
 * WORD, word-aligned, leaving the top (32 - entries_per_word*bits) bits of each
 * word UNUSED. A field NEVER straddles a word boundary. So:
 *   epw   = 32 / bits
 *   word  = idx / epw          (byte addr = base + word*4)
 *   shift = (idx % epw) * bits
 * The OLD code used CONTIGUOUS bit-packing (bit = idx*bits) which is only
 * correct when bits divides 32 evenly (1b, 2b, 4b). For the 7-bit SID2QID
 * array it addressed the wrong word: SID 64 -> 0x2130 (== HW SID 56's slot)
 * instead of the true 0x2138. That single off-by-one-word bug pointed the OMCI
 * SID-64 classify entry at a data flow's queue, so the US-NIC never classified
 * upstream OMCI to its T-CONT16/q0 -> the OLT never received the MIB-upload.
 * (Matches stock: SID 64 -> base 0x20f8 + 16*4 = 0x2138.) This is the same
 * class as l34_field_set: a self-consistent wrong offset survives every
 * readback, so the addressing lives HERE, x86-testable, and set/get share it.
 * bits must be 1..32 (epw = 32/bits; bits = 0 is a caller bug). */
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
