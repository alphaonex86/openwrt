/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE, and in the STRICT host-buildable subset -- it is G.988 byte math
 * and nothing else.  No register, no bus, no device pointer, no Linux API
 * beyond a string formatter.
 *
 * gpon_omci_trace -- say what a downstream OMCI PDU IS, in words.
 *
 * ★ WHY THIS IS A PROMOTION AND NOT A TIDY-UP.  It existed in ONE family
 * (cortina-gpon.c) and was ABSENT from the other: `grep -c mt_name` reads 2 on
 * Cortina and 0 on Luna.  So this is not a duplicate being merged -- it is a
 * DIAGNOSTIC one board had and the other did not, and every offset it reads is
 * a G.988 constant, not a fact about either silicon.  Moving it costs the Luna
 * boards nothing and hands them a decode they never had.
 *
 * ⚠ The CALLER still owns the policy: rate limiting, which PDUs to print, and
 * which log level.  Those are the family's, because how chatty a board may be
 * on its console is a property of that board's boot, not of G.988.  This file
 * only ever FORMATS INTO A BUFFER -- it cannot print, so it cannot flood.
 */
#ifndef GPON_OMCI_TRACE_H
#define GPON_OMCI_TRACE_H

#include <linux/types.h>

/* Baseline OMCI PDU (G.988, all big-endian byte math):
 *   [0:1] TCI   [2] msg-type {AR=bit6, AK=bit5, MT=bits4:0}
 *   [3]   device-id (0x0A = baseline)
 *   [4:5] ME class   [6:7] ME instance
 *   [8:39] contents  [40:47] trailer (incl. the 4-byte MIC)
 */
#define GPON_OMCI_MIN_HDR	8

/* G.988 Table 11.2.2-1.  "?" for a message type this table does not name --
 * never NULL, so no caller can forget the check. */
const char *gpon_omci_mt_name(u8 msg_type);

/*
 * One line describing a PDU: length, TCI, message type + its name, the AR/AK
 * flags, the device id and the ME class/instance.  Returns the number of
 * characters written (scnprintf semantics: never more than `sz - 1`).
 *
 * Returns 0 and writes an empty string when `len` is below GPON_OMCI_MIN_HDR
 * -- a runt is not describable, and inventing fields for it is how a phantom
 * gets into a log.
 */
int gpon_omci_describe(const u8 *pdu, unsigned int len, char *out, size_t sz);

/*
 * The extra detail a GET exchange carries: the mask asked for, the mask
 * answered, and the two failure masks the responder reports.  Writes an empty
 * string when this is not a Get, or when either side is too short to hold the
 * fields -- so a caller can always append the result unconditionally.
 *
 * `resp`/`resp_len` are the response the responder produced; pass resp = NULL
 * to describe a Get that got no answer at all.
 */
int gpon_omci_describe_get(const u8 *pdu, unsigned int len,
			   const u8 *resp, int resp_len, char *out, size_t sz);

/* Is this PDU a Get?  Exposed because callers branch on it before deciding
 * whether the response is worth capturing. */
bool gpon_omci_is_get(const u8 *pdu, unsigned int len);

#endif /* GPON_OMCI_TRACE_H */
