/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * flowcore.h -- the hardware-decoupled half of L3/L4 FLOW OFFLOAD.
 *
 * ★★★ WHY A SECOND COMMON LAYER EXISTS (operator, 2026-08-27: *"si se puede
 * mejor, al menos para ir alistando el trabajo"*). drivers/net/gpon holds the
 * GPON PROTOCOL core; flow offload is not GPON, so putting it there would have
 * been a naming lie of the kind this tree renames on sight. It is its own tier,
 * a sibling, reached by the same FILES_DIR both targets already carry -- so it
 * costs one obj- line and no target Makefile change.
 *
 * ★★ WHAT IT IS FOR, and it is NOT the usual de-duplication. MEASURED the day
 * it was created: the two boards' flow engines are not two copies, they are one
 * mature implementation and one embryo --
 *
 *     cortina-ni-flowoffload.c  6595 lines   (X400AXF: the verified board)
 *     rtl9602c_l34.c             611 lines   (Luna)
 *
 * -- while implementing the SAME concepts (flow add/delete, key hashing, NAT,
 * VLAN derivation, tuples, buckets). So the value here is not merging two
 * things: it is stopping the small one from growing its own version of what the
 * big one already got right, which is how the OMCI responder came to exist
 * twice and drift.
 *
 * ★ NOTHING VENDOR-SPECIFIC MAY LAND HERE. No register, no offset, no engine
 * name: the shells keep those. What belongs here is what a flow IS and how its
 * key is derived -- arithmetic that two different silicons must agree on
 * exactly, or their tables disagree about the same packet.
 *
 * ★ AND IT IS HOST-BUILDABLE ON PURPOSE. Only <linux/types.h>, no kernel
 * helpers (not even crc32_le or bitrev32, which the shell used) and no cast
 * over wire bytes -- so the same code fuzzes on x86 at thousands of cases a
 * second and runs identically on big-endian MIPS and little-endian ARM64.
 * A hash that differs by endianness is a table that silently forwards the
 * wrong packet.
 */
#ifndef _FLOWCORE_H
#define _FLOWCORE_H

#include <linux/types.h>

/** Reverse the bits of a byte. */
u8 flowcore_bitrev8(u8 v);
/** Reverse the bits of a 32-bit word. */
u32 flowcore_bitrev32(u32 v);

/**
 * flowcore_key_bitrev() - reverse a key's WORD ORDER, reversing each word.
 * @w:       the key words, modified in place.
 * @n_words: how many.
 *
 * Steps 3-4 of the lookup engine's key recipe. An odd word count leaves the
 * middle word in place, bit-reversed -- which the loop must not skip.
 */
void flowcore_key_bitrev(u32 *w, int n_words);

/**
 * flowcore_crc32_reflected() - reflected CRC-32 (Ethernet polynomial).
 * Seed ~0, reflected output, NO final xor -- the engine's convention, which is
 * NOT the same as the kernel's crc32_le() result and must not be swapped for it.
 */
u32 flowcore_crc32_reflected(const u8 *p, u32 len);

/**
 * flowcore_crc16_ccitt_reflected() - reflected CRC-16/CCITT (poly 0x8408).
 * Seed 0xffff, reflected output, no final xor, bytes swapped on the way out --
 * again the engine's convention, verified against the hardware CRC unit by a
 * bring-up selftest on the X400AXF before first use.
 */
u16 flowcore_crc16_ccitt_reflected(const u8 *p, u32 len);

/* ------------------------------------------------------------------ */
/* Packed-array slot math: where a `bits`-wide entry lives inside an   */
/* array of 32-bit words, plus the insert/extract that share the ONE   */
/* locate result.                                                      */
/*                                                                     */
/* MOVED HERE 2026-09-02 (round 3) from gpon_rtl9602c_logic.{c,h}: it  */
/* is generic math with no register, offset or engine name in it, but  */
/* it lived in an object gated on CONFIG_RTL9602C_GPON -- so the       */
/* Cortina flow engine (CONFIG_CORTINA_NI_FLOWOFFLOAD, a board that    */
/* never sets the 9602C symbol) could reach it only by writing a       */
/* second copy.  flowcore_hash.o is obj-y on every board that carries  */
/* this tree, which is what makes the call legal everywhere            */
/* (crossconfig_call_guard's defect class).  The pi_ name keeps its    */
/* origin (the 9602C's pi register space) because the luna shell calls */
/* it by that name and renaming would touch a shell this move must not.*/
/* ------------------------------------------------------------------ */

/* One packed-array slot: where a `bits`-wide entry lives inside the pi register
 * space. Set and get share this ONE locate result -- before the hoist they
 * could only agree by parallel maintenance, which is how the contiguous-pack
 * SID2QID mis-addressing stayed invisible (the wrong get mirrored the wrong
 * set). */
struct pi_packed_slot {
	u32 reg;		/* byte address of the 32-bit word (driver-relative) */
	unsigned int shift;	/* bit position of the field inside that word */
	u32 mask;		/* field mask, unshifted */
};

struct pi_packed_slot pi_packed_locate(u32 base, unsigned int idx,
				       unsigned int bits);
u32 pi_packed_insert(u32 word, const struct pi_packed_slot *slot, u32 val);
u32 pi_packed_extract(u32 word, const struct pi_packed_slot *slot);

#endif /* _FLOWCORE_H */
