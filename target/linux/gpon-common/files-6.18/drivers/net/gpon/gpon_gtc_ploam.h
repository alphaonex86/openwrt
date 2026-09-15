/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_gtc_ploam.h -- the GTC downstream-PLOAM message buffer, read through
 * struct hwio: the word-unpack luna_gpon.c's gpon_ploam_read() carried beside
 * its own MMIO until 2026-09-05.
 *
 * WHAT THE SILICON DOES.  The Luna GTC block latches each received downstream
 * PLOAM in an 8-word buffer (GPON_GTC_DS_PLOAM_MSG, 0x10a0 within the block on
 * the RTL9602C), TWO message octets per 32-bit word: octet 2i in [15:8], octet
 * 2i+1 in [7:0].  The 13 octets of a DS PLOAM (G.984.3: ONU-ID, type, 10 data,
 * CRC-8) therefore span seven words, the seventh carrying only the CRC in
 * [15:8].  gpon_send_cpu_ploam() PACKS the upstream message into US_PLOAM_DATA
 * with the same rule; this is its inverse.
 *
 * ★ WHY A (hwio, offset) FUNCTION -- regtable.h's law: which word and which
 *   byte lane is not board-specific, only the OFFSET and the accessor are, so
 *   the unpack is compiled once and fuzzed on x86 through a recording hwio
 *   (gpon_gtc_ploam_diff_test).  OWED: a `ds_ploam_msg` slot in struct
 *   gpon_gtc_regs plus a table wrapper, in regtable.h / luna_gpon_regs.h.
 *
 * ★ DELIBERATELY NOT HERE: the buffer-empty ask (DS_PLOAM_IND BUF_EMPTY) and
 *   the DEQ strobe are the shell's QUEUE DISCIPLINE -- a reader that advanced
 *   the queue itself would consume a message it was only asked to look at.  Nor
 *   any claim about Cortina: its MAC parses DS PLOAM in silicon and its DS FIFO
 *   is four registers (PLOAMD_FIFO0..3), a layout this does not model.
 */
#ifndef _GPON_GTC_PLOAM_H
#define _GPON_GTC_PLOAM_H

#include <linux/types.h>

#include "hwio.h"	/* the injected accessor */
#include "regtable.h"	/* reg_has(): the ask this tier owes before using an offset */
#include "gpon_ploam.h"	/* GPON_PLOAM_DS_LEN -- the one spelling of the 13-octet DS message */

/**
 * gpon_gtc_ds_ploam_read() - unpack the latched downstream PLOAM.
 * @io:      how to reach the GTC block (the shell's hwio over that block).
 * @msg_off: GPON_GTC_DS_PLOAM_MSG within the block, from the per-SoC header
 *           (reg_make()) or a per-chip table.  A DECLARED absence and a field
 *           NOBODY REGISTERED are both refused before any bus traffic.
 * @m:       out -- the GPON_PLOAM_DS_LEN (13) message octets.  NOT written
 *           on refusal.
 *
 * Seven reads at @msg_off + 0, 4, ... 24, in that order, and no write.  Byte
 * arithmetic is the pre-conversion driver's, verbatim:
 *   m[2i] = (w >> 8) & 0xff;  m[2i+1] = w & 0xff;  i = 0..5
 *   m[12] = (word 6 >> 8) & 0xff
 * Proven against that form on address, order, read count and every octet by
 * dev/rtl9607c-test/gpon_gtc_ploam_diff_test.
 *
 * Return: true when @m was filled; false when this chip declares no message
 * buffer (the caller logs -- a chip without it must never reach here).
 */
static inline bool gpon_gtc_ds_ploam_read(const struct hwio *io,
					  struct reg msg_off,
					  u8 m[GPON_PLOAM_DS_LEN])
{
	const unsigned int full = GPON_PLOAM_DS_LEN / 2u;	/* 6 two-octet words */
	unsigned int i;
	u32 base;

	if (!reg_has(msg_off))
		return false;
	base = reg_at(msg_off);
	for (i = 0; i < full; i++) {
		u32 w = hwio_rd(io, base + i * 4u);

		m[2 * i]     = (w >> 8) & 0xff;
		m[2 * i + 1] = w & 0xff;
	}
	m[GPON_PLOAM_DS_LEN - 1] = (hwio_rd(io, base + full * 4u) >> 8) & 0xff;
	return true;
}

#endif /* _GPON_GTC_PLOAM_H */
