// SPDX-License-Identifier: GPL-2.0-only
/* See cortina_ni_flowoffload_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>
#include <linux/bitrev.h>
#include <linux/crc32.h>
#include <linux/kernel.h>

#include "cortina_ni_flowoffload_logic.h"

/* Steps 3-4 of the HW recipe (host-fuzz reference for the SW CRC path; the
 * runtime hash uses the SWO engine - see cn_l3e_key_hash). */
void cn_l3e_bitrev_key(u32 *w, int n_words)
{
	int i;
	u32 t;

	for (i = 0; i < n_words / 2; i++) {
		t = w[i];
		w[i] = bitrev32(w[n_words - 1 - i]);
		w[n_words - 1 - i] = bitrev32(t);
	}
	if (n_words & 1)
		w[i] = bitrev32(w[i]);
}

u32 cn_l3e_crc32(const u8 *p, size_t len)
{
	/* reflected CRC-32 (Ethernet poly), seed ~0, reflected output, no
	 * final xor - the engine's convention */
	return bitrev32(crc32_le(~0u, p, len));
}

u16 cn_l3e_crc16(const u8 *p, size_t len)
{
	/* reflected CRC-16/CCITT (poly 0x8408), seed 0xffff, reflected
	 * output, no final xor */
	u16 crc = 0xffff;
	int i;

	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
	}
	return bitrev8(crc & 0xff) << 8 | bitrev8(crc >> 8);
}

/*
 * ★ HW hash recipe - FULLY RECOVERED 2026-07-18 (tier-2 Ghidra decomp of
 * stock hash_value_calculate() @0x11e0 + tier-1 single-bit SWO sweeps on the
 * live engine).  The lookup HW computes, for a 92-byte key and a 256-bit
 * mask-table entry:
 *   1. mask-apply, per FIELD (not per bit): field &= ~mask_field
 *      (mask bit 1 = EXCLUDE the field; 0 = keep it).
 *   2. HW-derived nonlinear FLAG bits (address-equal / zero / prefix-length
 *      checks) are folded into the reduced tuple - so even an all-zero key
 *      does NOT hash to CRC(zeros): the transform is NOT linear in the key
 *      bytes (proven: SWO(0)=0x7fc13ab0 != crc32(92*0x00)=0x3d4ad918).
 *   3. bit-reverse the whole key: 23 u32 words, bitrev32 EACH word AND
 *      reverse the WORD ORDER (cn_l3e_bitrev_key does exactly this).
 *   4. CRC32: reflected poly 0xEDB88320, seed ~0, final = bitrev32 (NO xor).
 *      CRC16: reflected poly 0x8408, seed 0xffff, final = bitrev16 (NO xor).
 *      (cn_l3e_crc32 / cn_l3e_crc16 implement 3+4 exactly - confirmed against
 *      the crctable_32/crctable_ccitt16 tables in the stock .ko.)
 *   5. optional per-profile CRC16 rotate/xor - gated on table_id==1 and
 *      indexed by hash_key_select[ctrl_set_id]; hash_key_select is .bss and
 *      stays 0 in stock (the <0x40 rule keeps CRC32 standard) => rotate is
 *      identity in practice.
 *
 * The nonlinear FLAG derivation (step 2) + the per-field mask-apply (step 1)
 * are ~1100 lines of stock logic; rather than transliterate them (fragile,
 * and the mask VALUES are external to ca-ne.ko anyway), the runtime hash
 * DRIVES THE ON-CHIP SWO CRC ENGINE - the SAME engine the lookup path uses -
 * so the {crc32,crc16} are IDENTICAL to what a parsed packet hashes to, by
 * construction (this is exactly what stock's own runtime add path does; the
 * SW CRC above is stock's host-fuzz-only fallback).  cn_l3e_crc32/16 +
 * cn_l3e_bitrev_key are kept as the host-fuzz reference for step 3/4.
 *
 * ★ KEY LAYOUT (divergence-A fix, 2026-07-18): the SWO engine hashes the
 * 128-byte HW HDR_I descriptor, NOT our 92-byte cn_l3e_key.  cn_l3e_key_hash
 * therefore converts the SW key to HDR_I (cn_l3e_build_hdri) and feeds all 32
 * HDR_I words to the SWO.  The profile id that stock aal_hash_value_cal_part_0
 * stamps into the SW key's ctrl_set_id lands in HDR_I as t2_ctrl (07f has no
 * separate table_id field; its SW-key table_id is always mask-zeroed pre-CRC).
 * (Feeding the raw 92-byte key - the previous bug - put every 5-tuple field in
 * a masked-out HDR_I position, so the CRC was constant; see cn_l3e_build_hdri.)
 */
u32 cn_l3e_proc_parse_ip(const char *s)
{
	u8 b[4];
	unsigned int v;

	if (sscanf(s, "%hhu.%hhu.%hhu.%hhu", &b[0], &b[1], &b[2], &b[3]) == 4)
		return ((u32)b[0] << 24) | ((u32)b[1] << 16) |
		       ((u32)b[2] << 8) | b[3];
	if (kstrtouint(s, 0, &v) == 0)
		return v;
	return 0;
}
