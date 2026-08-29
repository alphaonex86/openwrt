// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gpon_omci_trace -- see gpon_omci_trace.h.  G.988 byte math, nothing else.
 */
#include <linux/kernel.h>
#include <linux/types.h>

#include "gpon_omci_trace.h"

/* G.988 Table 11.2.2-1.  32 entries because the message type is 5 bits. */
static const char *const gpon_omci_mt[32] = {
	[4] = "Create", [5] = "Delete", [8] = "Set", [9] = "Get",
	[11] = "Get-all-alarms", [12] = "Get-all-alarms-next",
	[13] = "MIB-upload", [14] = "MIB-upload-next",
	[15] = "MIB-reset", [16] = "Alarm", [17] = "AVC", [18] = "Test",
	[19] = "Start-SW-dl", [20] = "DL-section", [21] = "End-SW-dl",
	[22] = "Activate-SW", [23] = "Commit-SW", [24] = "Sync-time",
	[25] = "Reboot", [26] = "Get-next", [27] = "Test-result",
	[28] = "Get-current-data", [29] = "Set-table",
};

const char *gpon_omci_mt_name(u8 msg_type)
{
	const char *n = gpon_omci_mt[msg_type & 0x1f];

	return n ? n : "?";
}

bool gpon_omci_is_get(const u8 *pdu, unsigned int len)
{
	return pdu && len >= 3 && (pdu[2] & 0x1f) == 9;
}

int gpon_omci_describe(const u8 *pdu, unsigned int len, char *out, size_t sz)
{
	u8 mt;

	if (!out || !sz)
		return 0;
	out[0] = '\0';
	if (!pdu || len < GPON_OMCI_MIN_HDR)
		return 0;

	mt = pdu[2];
	return scnprintf(out, sz,
			 "len=%u tci=0x%02x%02x mt=%u(%s)%s%s dev=0x%02x me=%u/%u",
			 len, pdu[0], pdu[1], mt & 0x1f, gpon_omci_mt_name(mt),
			 (mt & 0x40) ? " AR" : "", (mt & 0x20) ? " AK" : "",
			 pdu[3],
			 ((u16)pdu[4] << 8) | pdu[5],
			 ((u16)pdu[6] << 8) | pdu[7]);
}

int gpon_omci_describe_get(const u8 *pdu, unsigned int len,
			   const u8 *resp, int resp_len, char *out, size_t sz)
{
	if (!out || !sz)
		return 0;
	out[0] = '\0';

	if (!gpon_omci_is_get(pdu, len))
		return 0;

	/*
	 * ⚠ The response is described only when it is a WHOLE baseline PDU.  A
	 * short one is not a Get response with missing fields, it is something
	 * else entirely, and reading masks out of it would print four confident
	 * numbers taken from whatever the buffer happened to hold.
	 */
	if (!resp || resp_len < 40)
		return scnprintf(out, sz, " noresp");

	if (len < 10)
		return 0;

	/* [8:9] the requested attribute mask; in the response, [9:10] the mask
	 * actually answered, [36:37] the unsupported attributes, [38:39] the
	 * ones that failed, and [8] the result/reason code. */
	return scnprintf(out, sz,
			 " mask=0x%04x rmask=0x%04x unsup=0x%04x failed=0x%04x rc=%u",
			 ((u16)pdu[8] << 8) | pdu[9],
			 ((u16)resp[9] << 8) | resp[10],
			 ((u16)resp[36] << 8) | resp[37],
			 ((u16)resp[38] << 8) | resp[39],
			 resp[8]);
}
