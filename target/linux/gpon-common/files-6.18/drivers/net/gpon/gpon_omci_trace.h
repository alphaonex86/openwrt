/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE, strict host-buildable subset -- G.988 byte math and nothing else.
 *
 * gpon_omci_trace -- say what a downstream OMCI PDU IS, in words.
 *
 * ★ A PROMOTION, NOT A TIDY-UP: it existed in ONE family and was ABSENT from
 * the other (`grep -c mt_name` read 2 on Cortina, 0 on Luna), so moving it
 * costs the Luna boards nothing and hands them a decode they never had.
 *
 * ⚠ The CALLER owns the policy -- rate limit, which PDUs, which log level.
 *   This file only ever FORMATS INTO A BUFFER, so it cannot print or flood.
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
 * flags, the device id and the ME class/instance.  Returns the characters
 * written (scnprintf semantics).
 *
 * Returns 0 and writes an empty string below GPON_OMCI_MIN_HDR -- a runt is
 * not describable, and inventing fields for it is how a phantom gets into a log.
 */
int gpon_omci_describe(const u8 *pdu, unsigned int len, char *out, size_t sz);

/* Which request is this?  Exposed because a caller branches on the type before
 * deciding which detail it may read out of the request -- a Create's ATTRIBUTE
 * BODY starts at octet 8 where a Get and a Set put a mask.  False for a runt. */
bool gpon_omci_is_get(const u8 *pdu, unsigned int len);
bool gpon_omci_is_set(const u8 *pdu, unsigned int len);
bool gpon_omci_is_create(const u8 *pdu, unsigned int len);

/*
 * Does a baseline RESPONSE to this request carry a RESULT CODE at octet 8?
 *
 * ★★ IT IS NOT UNIVERSAL, AND ASSUMING IT WAS PRINTED A FABRICATED NUMBER.
 *    The upload and alarm-list families answer "how many follow" or "here is
 *    the next row" in exactly those octets, so octet 8 of a MIB-Upload-Next
 *    reply is the HIGH BYTE OF A CLASS ID.  Measured on the X111W 2026-09-13:
 *    26 replies rendered `rc=1`, which is ME 256's 0x01 -- 26 upload failures
 *    that never happened.
 */
bool gpon_omci_has_result_code(const u8 *pdu, unsigned int len);

/*
 * Is this one of the BULK message types -- the ones an OLT sends in long runs
 * and which may therefore be rate-limited in a log, unlike a Create or an Alarm?
 *
 * ★ THE ANSWER IS G.988, NOT SILICON.  A shell decoding `pdu[2] & 0x1f` itself
 * holds a second copy of the message-type numbering that rots the day G.988
 * handling changes on one side only; luna_eth.c held that copy until 2026-09-10.
 *
 * False for a runt, so a caller that cannot see the type logs it rather than
 * silently rate-limiting a PDU nobody could classify.
 */
bool gpon_omci_is_bulk(const u8 *pdu, unsigned int len);

/*
 * Is this PDU a Create for @me_class?  The CLASS is the caller's -- this file
 * holds no ME numbering -- so a shell asks
 * `gpon_omci_is_create_of(pdu, len, OMCI_ME_GEM_CTP)` instead of spelling
 * `(pdu[2] & 0x1f) == 4 && ((pdu[4] << 8) | pdu[5]) == 268` for itself.
 * False for a runt.
 */
bool gpon_omci_is_create_of(const u8 *pdu, unsigned int len, u16 me_class);

/*
 * The body of a baseline PDU and its length, CLAMPED to the baseline frame.
 *
 * ⚠ THE CLAMP IS THE POINT.  The core's body length is a `u8` and
 * `(u8)(len - 8)` WRAPS: a 264-byte frame became 0 and was read as a runt, a
 * 512-byte one became a plausible 248.  Everything past OMCI_LEN is padding, so
 * the length is BOUNDED, never truncated modulo 256.
 */
const u8 *gpon_omci_body(const u8 *pdu, unsigned int len, u8 *blen);

#endif /* GPON_OMCI_TRACE_H */
