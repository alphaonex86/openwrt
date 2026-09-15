/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE, and in the STRICT host-buildable subset -- G.984.4 byte math and
 * nothing else.  No register, no bus, no device pointer, no lock, no allocator,
 * no clock.  See gpon_common.h for the tier rule; it is written out once there.
 *
 * gpon_omci_mic -- WHICH CRC-32 CONVENTION STAMPED THIS DOWNSTREAM FRAME'S MIC.
 *
 * ★ A DECISION, NOT A TIDY-UP.  AAL5-BE is spelled EXACTLY ONCE
 *   (omci_mic_compute) because it had already diverged here: Luna stamped the
 *   reflected zlib crc32_le while the core verified crc32_be, and a correctly
 *   behaving OLT rejected every frame that ONU sent.  The OTHER half -- the
 *   reflected spelling and the VERDICT naming which one a frame carries -- was
 *   still hand-written in cortina-gpon.c's DS MIC self-check, a second spelling
 *   in the file whose job is registers.  It lives here now and is exercised on
 *   x86 by dev/rtl9607c-test/gpon_omci_rx_diff_test.c with no board.
 *
 * ★ A DIAGNOSTIC VERDICT, NEVER AN ACCEPTANCE GATE.  Whether a frame may be
 *   ACTED ON is omci_mic_ok() in gpon_omci_core.c and this header offers no
 *   second answer.  The two agree by construction -- conv() returns AAL5_BE
 *   exactly when omci_mic_ok() is true -- and the differential asserts it over
 *   a randomised corpus so the pair cannot drift into two rules.
 *
 * ⚠ THE CALLER OWNS THE POLICY: sampling, log level, device, counters.  This
 *   file only RETURNS A VERDICT -- it cannot print, so it cannot flood.
 *
 * ⚠ NAMING: the enumerator spellings ARE the strings the shells already print
 *   ("AAL5-BE" / "ZLIB-LE" / "NEITHER").  They are kept byte-for-byte because a
 *   log line is a measurement instrument here -- the DS MIC self-check is what
 *   settled the convention on this OLT in the first place -- and a renamed
 *   verdict would silently invalidate every capture taken before the rename.
 */
#ifndef GPON_OMCI_MIC_H
#define GPON_OMCI_MIC_H

#include <linux/crc32.h>
#include <linux/types.h>

#include "gpon_omci_core.h"	/* OMCI_LEN + omci_mic_compute(): the ONE
				 * AAL5-BE spelling.  This header must never
				 * respell it -- reaching for ~crc32_be here
				 * would recreate the exact defect above. */

/*
 * The conventions a downstream OMCI trailer can be carrying.
 *
 * SHORT is a value and not an error return: "cannot carry a MIC at all" is a
 * framing or GEM-reassembly fault upstream of OMCI, "carries one we do not
 * recognise" is corruption on a well-framed PDU, and gpon_omci_core.c already
 * keeps rx_runt and rx_bad_mic apart for that reason.  Collapsing them would
 * make a broken reassembler look like a noisy fibre.
 */
enum gpon_mic_conv {
	GPON_MIC_CONV_SHORT = 0,	/* < OMCI_LEN: no MIC to judge          */
	GPON_MIC_CONV_AAL5_BE,		/* ~crc32_be(~0, pdu, 44)   -- G.984.4  */
	GPON_MIC_CONV_ZLIB_LE,		/* crc32_le(~0, pdu, 44) ^ ~0 reflected */
	GPON_MIC_CONV_NEITHER,		/* corrupt, or a convention we lack     */
};

/* The MIC as the OLT stamped it: bytes 44..47, big-endian.  Explicit byte
 * math, never a cast over the wire buffer -- one source, two byte orders.
 *
 * ⚠ PRECONDITION: @pdu holds at least OMCI_LEN octets.  This and
 *   gpon_omci_mic_zlib_le() are the RAW halves, exposed only so a self-check
 *   can PRINT the three numbers it compared.  A new call site uses
 *   gpon_omci_mic_conv(), which takes the length and answers SHORT rather than
 *   reading bytes nobody sent. */
static inline u32 gpon_omci_mic_stamped(const u8 *pdu)
{
	return ((u32)pdu[44] << 24) | ((u32)pdu[45] << 16) |
	       ((u32)pdu[46] << 8) | pdu[47];
}

/* The reflected (zlib) CRC-32 over bytes 0..43 -- the OTHER convention, spelled
 * ONCE, here.  It is NOT what we emit; it exists so a self-check can SAY that
 * a link is dead because the far end speaks the other dialect, instead of the
 * link simply going quiet.  Same OMCI_LEN precondition as above. */
static inline u32 gpon_omci_mic_zlib_le(const u8 *pdu)
{
	return crc32_le(~0u, pdu, 44) ^ ~0u;
}

/*
 * Which convention stamped @pdu.  Pure: no state, no counters, no printing.
 *
 * ⚠ AAL5-BE IS TESTED FIRST AND THEREFORE WINS A TIE.  That is the order the
 *   shell's hand-rolled ternary used, and the order matters: G.984.4 specifies
 *   AAL5, so a frame that satisfies both must be reported as the conformant
 *   one, never as the dialect.
 */
static inline enum gpon_mic_conv gpon_omci_mic_conv(const u8 *pdu,
						    unsigned int len)
{
	u32 want;

	if (!pdu || len < OMCI_LEN)
		return GPON_MIC_CONV_SHORT;

	want = gpon_omci_mic_stamped(pdu);
	if (omci_mic_compute(pdu) == want)
		return GPON_MIC_CONV_AAL5_BE;
	if (gpon_omci_mic_zlib_le(pdu) == want)
		return GPON_MIC_CONV_ZLIB_LE;
	return GPON_MIC_CONV_NEITHER;
}

/* Never NULL, exactly as gpon_omci_mt_name() promises -- so no caller can
 * forget the check and print a nul pointer into a console. */
static inline const char *gpon_mic_conv_name(enum gpon_mic_conv c)
{
	switch (c) {
	case GPON_MIC_CONV_AAL5_BE:
		return "AAL5-BE";
	case GPON_MIC_CONV_ZLIB_LE:
		return "ZLIB-LE";
	case GPON_MIC_CONV_SHORT:
		return "short";
	case GPON_MIC_CONV_NEITHER:
		break;
	}
	return "NEITHER";
}

#endif /* GPON_OMCI_MIC_H */
