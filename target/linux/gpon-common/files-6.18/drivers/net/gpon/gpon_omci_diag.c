// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: CORE, strict host-buildable subset.  See gpon_omci_diag.h for what this
 * is and why an unreported mask must render as `n/a` and never as 0x0000.
 */
#include <linux/kernel.h>
#include <linux/types.h>

#include "gpon_omci_core.h"	/* OMCI_LEN: one baseline PDU */
#include "gpon_omci_diag.h"
#include "gpon_omci_trace.h"

/* The request octets a Get must hold before [8:9] can be read as its mask. */
#define GET_MASK_END	10
/* How many value octets a Set line renders.  Bounded on purpose -- see the
 * note in set_detail(). */
#define SET_VALUE_OCTETS	8
/* A Create has no mask: its attribute body starts where a Get puts one. */
#define CREATE_BODY_START	8

/*
 * The detail a Get exchange carries.  Appended at `*pos`; the buffer is the
 * caller's and scnprintf never writes past it.
 *
 * [8:9] of the request is the mask the OLT asked for; of the response, [9:10] is
 * the mask actually answered, [36:37] the attributes we do not model, [38:39]
 * those that did not fit the 25-octet value area, and [8] the result code.
 */
static void get_detail(char *out, size_t sz, int *pos, const u8 *pdu,
		       unsigned int len, const u8 *resp, int resp_len)
{
	if ((size_t)*pos >= sz)
		return;
	if (resp_len == GPON_OMCI_NO_RESP_REPORT) {
		*pos += scnprintf(out + *pos, sz - *pos, " resp=n/a");
		return;
	}
	if (len < GET_MASK_END) {
		*pos += scnprintf(out + *pos, sz - *pos, " mask=n/a");
		return;
	}
	/*
	 * The response is decoded only when it is a WHOLE baseline PDU.  A short
	 * one is not a Get response with missing fields, it is something else
	 * entirely, and reading masks out of it would print four confident
	 * numbers taken from whatever the buffer happened to hold.
	 */
	if (!resp || resp_len < OMCI_LEN) {
		*pos += scnprintf(out + *pos, sz - *pos, " noresp");
		return;
	}
	*pos += scnprintf(out + *pos, sz - *pos,
			  " mask=0x%04x rmask=0x%04x unsup=0x%04x failed=0x%04x rc=%u",
			  ((u16)pdu[8] << 8) | pdu[9],
			  ((u16)resp[9] << 8) | resp[10],
			  ((u16)resp[36] << 8) | resp[37],
			  ((u16)resp[38] << 8) | resp[39],
			  resp[8]);
}

/*
 * The detail a SET exchange carries, and it is DELIBERATELY only two fields.
 *
 * ★★ IT EXISTS BECAUSE THE ONE QUESTION THIS LINE COULD NOT ANSWER WAS THE ONE
 *    THAT MATTERED.  Two boards were measured losing provisioning immediately
 *    after an OLT Set on ME 11, and every saved line read
 *    `mt=8(Set) AR dev=0x0a me=11/257` and stopped there -- no mask, no result.
 *    The model keeps ONE last_req/last_resp pair that any later PDU overwrites,
 *    so the bytes were gone by the time anyone looked.  WHICH ATTRIBUTES the
 *    OLT asked for, and WHAT WE ANSWERED, are both one field each.
 *
 * ★ AND NOTHING ELSE IS PRINTED.  G.988 puts optional masks in a Set response
 *   only in some result cases, and this core does not write them; rendering
 *   them anyway would print confident numbers taken from whatever the buffer
 *   held -- the defect the Get path above is shaped to avoid.
 */
/*
 * The first SET_VALUE_OCTETS octets at @from, as hex, when the PDU is long
 * enough to hold them.  ONE renderer for the Set's value area and the Create's
 * attribute body, because they are the same question asked at two offsets.
 *
 * ⚠ SPELLED OUT, NOT `%*phN`.  That is a KERNEL vsnprintf extension and on the
 *   x86 host the same format renders a POINTER followed by the literal "hN", so
 *   an offline case judging this line would be judging a shim's emulation
 *   instead of the shipped code.  gpon_unsup.h states the same finding with the
 *   footprint numbers behind it.
 */
static void append_octets(char *out, size_t sz, int *pos, const char *label,
			  const u8 *pdu, unsigned int len, unsigned int from)
{
	static const char hexd[] = "0123456789abcdef";
	char hex[2u * SET_VALUE_OCTETS + 1u];
	unsigned int i;

	if (len < from + SET_VALUE_OCTETS)
		return;			/* absent is not zero: print nothing */
	for (i = 0; i < SET_VALUE_OCTETS; i++) {
		hex[2u * i] = hexd[(pdu[from + i] >> 4) & 0xfu];
		hex[2u * i + 1u] = hexd[pdu[from + i] & 0xfu];
	}
	hex[2u * SET_VALUE_OCTETS] = '\0';
	*pos += scnprintf(out + *pos, sz - *pos, "%s%s", label, hex);
}

/*
 * THE RESULT CODE, AND EVERY ANSWERED PDU HAS ONE.
 *
 * ★ IT EXISTS BECAUSE A CREATE WAS THE NEXT WALL AND THE LINE COULD NOT SEE IT.
 *   With ME 11 repaired the X111W's OLT went on to Create ME 47/52/171/268 --
 *   and RETRANSMITTED the same tci on ME 268, the data GEM, while our own
 *   counters said we had answered every request (queued 979, ustx 979,
 *   tx_errors 0).  "We answered" and "we answered YES" are different claims,
 *   and only [8] of the response separates them.  Byte [8] is the result code
 *   of a baseline response whatever the message type, so nothing here needs to
 *   know which type it is looking at.
 */
static void result_detail(char *out, size_t sz, int *pos,
			  const u8 *pdu, unsigned int len,
			  const u8 *resp, int resp_len)
{
	if (!gpon_omci_has_result_code(pdu, len))
		return;			/* the field does not exist: print none */
	if (resp_len == GPON_OMCI_NO_RESP_REPORT) {
		*pos += scnprintf(out + *pos, sz - *pos, " resp=n/a");
		return;
	}
	/* AR=0 is a legal request with no response at all; a SHORT one is not a
	 * response with fields missing.  Neither may be read. */
	if (!resp || resp_len < OMCI_LEN) {
		*pos += scnprintf(out + *pos, sz - *pos, " noresp");
		return;
	}
	*pos += scnprintf(out + *pos, sz - *pos, " rc=%u", resp[8]);
}

/*
 * The detail a CREATE carries: its ATTRIBUTE BODY, and our result code.
 *
 * ★ THE BODY IS WHERE THE IDENTITY LIVES.  A Create names the ME in its header
 *   and everything else in the body -- for ME 268 (GEM Port Network CTP) the
 *   FIRST TWO OCTETS are attribute #1, the GEM Port-ID.  Rendering only the
 *   result code says we accepted it and not WHAT we accepted, and that is the
 *   gap that left the X111W's data path unexplained: the OLT created a GEM,
 *   we answered rc=0, and nothing in the driver could say which port it was.
 *   Same shape as the ME 11 Set whose mask was visible and whose VALUES were
 *   not -- one message further along.
 */
static void create_detail(char *out, size_t sz, int *pos, const u8 *pdu,
			  unsigned int len, const u8 *resp, int resp_len)
{
	if ((size_t)*pos >= sz)
		return;
	append_octets(out, sz, pos, " body=", pdu, len, CREATE_BODY_START);
	result_detail(out, sz, pos, pdu, len, resp, resp_len);
}

static void set_detail(char *out, size_t sz, int *pos, const u8 *pdu,
		       unsigned int len, const u8 *resp, int resp_len)
{
	if ((size_t)*pos >= sz)
		return;
	if (len < GET_MASK_END) {		/* the same [8:9] the Get reads */
		*pos += scnprintf(out + *pos, sz - *pos, " mask=n/a");
		return;
	}
	*pos += scnprintf(out + *pos, sz - *pos, " mask=0x%04x",
			  ((u16)pdu[8] << 8) | pdu[9]);
	/* ★ THE FIRST VALUE OCTETS, AND ONLY THE FIRST.  The mask alone says
	 *   WHICH attributes the OLT asked for; deciding whether we can honour
	 *   one needs WHAT it asked for -- an auto-detection config of 0 is the
	 *   default and costs nothing, a different value is a physical change.
	 *   The count is fixed and small because this is a log line, not a
	 *   dump: the value area is 30 octets and rendering all of them would
	 *   make every Set a flood.  Sizes are per attribute and live in the
	 *   ME table, which this tier does not reach, so the octets are printed
	 *   RAW and the reader applies the mask. */
	append_octets(out, sz, pos, " val=", pdu, len, GET_MASK_END);
	result_detail(out, sz, pos, pdu, len, resp, resp_len);
}

int gpon_omci_diag_line(const u8 *pdu, unsigned int len,
			const u8 *resp, int resp_len, char *out, size_t sz)
{
	int pos;

	if (!out || !sz)
		return 0;
	pos = gpon_omci_describe(pdu, len, out, sz);
	if (!pos)
		return pos;
	if (gpon_omci_is_get(pdu, len))
		get_detail(out, sz, &pos, pdu, len, resp, resp_len);
	else if (gpon_omci_is_set(pdu, len))
		set_detail(out, sz, &pos, pdu, len, resp, resp_len);
	else if (gpon_omci_is_create(pdu, len))
		create_detail(out, sz, &pos, pdu, len, resp, resp_len);
	else
		result_detail(out, sz, &pos, pdu, len, resp, resp_len);
	return pos;
}
