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

#endif /* _FLOWCORE_H */
