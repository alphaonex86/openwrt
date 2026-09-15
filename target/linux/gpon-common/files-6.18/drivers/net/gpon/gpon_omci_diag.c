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

/* The detail a Get exchange carries. Appended at `*pos`; the ...
 * dev/MEASURED-gpon_omci_diag.c.md sec 1. */
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
	/* The response is decoded only when it is a WHOLE baseline ...
	 * dev/MEASURED-gpon_omci_diag.c.md sec 2. */
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

/* The detail a SET exchange carries, and it is DELIBERATELY ...
 * dev/MEASURED-gpon_omci_diag.c.md sec 3. */
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

/* THE RESULT CODE, AND EVERY ANSWERED PDU HAS ONE. ★ IT ...
 * dev/MEASURED-gpon_omci_diag.c.md sec 4. */
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

/* The detail a CREATE carries: its ATTRIBUTE BODY, and our ...
 * dev/MEASURED-gpon_omci_diag.c.md sec 5. */
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
	/* ★ THE FIRST VALUE OCTETS, AND ONLY THE FIRST. The mask ...
	 * dev/MEASURED-gpon_omci_diag.c.md sec 6. */
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
