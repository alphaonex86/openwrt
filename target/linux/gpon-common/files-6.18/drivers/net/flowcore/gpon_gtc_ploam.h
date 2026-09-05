/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_gtc_ploam.h -- the GTC downstream-PLOAM message buffer, read through
 * struct hwio: the word-unpack that gpon-luna.c's gpon_ploam_read() carried
 * beside its own MMIO until 2026-09-05.
 *
 * WHAT THE SILICON DOES.  The Luna GTC block latches each received downstream
 * PLOAM in an 8-word buffer (GPON_GTC_DS_PLOAM_MSG, 0x10a0 within the block
 * on the RTL9602C), TWO message octets per 32-bit word: octet 2i in [15:8],
 * octet 2i+1 in [7:0].  The 13 octets of a DS PLOAM (G.984.3: ONU-ID, type,
 * 10 data, CRC-8) therefore span seven words, the seventh carrying only the
 * CRC in [15:8].  gpon_send_cpu_ploam() in the shell PACKS the upstream
 * message into US_PLOAM_DATA with the same two-octets-per-word rule; this is
 * that rule's inverse, and the upstream half is its natural future sibling.
 *
 * ★ WHY A (hwio, offset) FUNCTION.  regtable.h's law: the reasoning -- which
 *   word, which byte lane -- is not board-specific, only the OFFSET and the
 *   accessor are, so the shell hands both in and the unpack is compiled once,
 *   fuzzable on x86 through a recording hwio (gpon_gtc_ploam_diff_test).  The
 *   offset comes as a PARAMETER, the transaction-layer shape of
 *   gpon_gtc_cam_xact() / gpon_gtc_cntr_read(): a `ds_ploam_msg` slot in
 *   struct gpon_gtc_regs plus a table wrapper is the owed next step, and it
 *   lands in regtable.h / luna_gpon_regs.h, not here.
 *
 * ★ WHAT IS DELIBERATELY NOT HERE.  The buffer-empty ask (DS_PLOAM_IND
 *   BUF_EMPTY) and the DEQ strobe that surround this read in the shell's poll
 *   loop are the shell's QUEUE DISCIPLINE; a reader that advanced the queue
 *   itself would consume a message it was only asked to look at.  Nor is any
 *   claim about the Cortina family: its MAC parses DS PLOAM in silicon
 *   (cortina-gpon.c, "no SW DS-PLOAM parsing needed") and its DS FIFO is four
 *   registers (PLOAMD_FIFO0..3 in the stock register-name table), a layout
 *   this function does not model.
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
 *           or table.  REG_ABSENT is refused before any bus traffic.
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
static inline bool gpon_gtc_ds_ploam_read(const struct hwio *io, u32 msg_off,
					  u8 m[GPON_PLOAM_DS_LEN])
{
	const unsigned int full = GPON_PLOAM_DS_LEN / 2u;	/* 6 two-octet words */
	unsigned int i;

	if (!reg_has(msg_off))
		return false;
	for (i = 0; i < full; i++) {
		u32 w = hwio_rd(io, msg_off + i * 4u);

		m[2 * i]     = (w >> 8) & 0xff;
		m[2 * i + 1] = w & 0xff;
	}
	m[GPON_PLOAM_DS_LEN - 1] = (hwio_rd(io, msg_off + full * 4u) >> 8) & 0xff;
	return true;
}

#endif /* _GPON_GTC_PLOAM_H */
