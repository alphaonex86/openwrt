// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gpon_omci_trace -- see gpon_omci_trace.h.  G.988 byte math, nothing else.
 */
#include <linux/kernel.h>
#include <linux/types.h>

#include "gpon_omci_core.h"	/* OMCI_MT_*: the ONE numbering of Table 11.2.2-1 */
#include "gpon_omci_trace.h"

/*
 * G.988 Table 11.2.2-1.  32 entries because the message type is 5 bits.
 *
 * ★★★ INDEXED BY THE CORE'S OWN OMCI_MT_* NAMES, NOT BY BARE NUMBERS, AND THE
 *   TWO SPELLINGS DISAGREED (found 2026-09-05).  This table used to write the
 *   numbering a SECOND time, in decimal, and it said
 *
 *	[5] = "Delete"
 *
 *   while gpon_omci_core.h:80 says OMCI_MT_DELETE 0x06 -- and so does the
 *   independent oracle (rtl9607c-oracle/omci_msg.h:90) and G.988 itself, where
 *   5 is a deprecated type no OLT sends.  So every Delete this ONU actually
 *   received printed as unknown, and the one code that would have printed
 *   "Delete" is one that never arrives.  A tracer that mislabels the message
 *   the OLT sent is worse than one that says nothing: it is read as evidence.
 *
 *   The differential could not see it -- its "before" half calls this same
 *   table, so both sides were wrong together.  What finds this shape is not a
 *   test, it is refusing to let one fact have two spellings.
 *
 *   ⇒ with the index written as OMCI_MT_*, a disagreement of this kind is no
 *   longer expressible: there is one numbering and this file reads it.
 */
static const char *const gpon_omci_mt[32] = {
	[OMCI_MT_CREATE]		= "Create",
	[OMCI_MT_DELETE]		= "Delete",
	[OMCI_MT_SET]			= "Set",
	[OMCI_MT_GET]			= "Get",
	[OMCI_MT_GET_ALL_ALARMS]	= "Get-all-alarms",
	[OMCI_MT_GET_ALL_ALRM_NX]	= "Get-all-alarms-next",
	[OMCI_MT_MIB_UPLOAD]		= "MIB-upload",
	[OMCI_MT_MIB_UPLOAD_NX]		= "MIB-upload-next",
	[OMCI_MT_MIB_RESET]		= "MIB-reset",
	[OMCI_MT_ALARM]			= "Alarm",
	[OMCI_MT_AVC]			= "AVC",
	[OMCI_MT_TEST]			= "Test",
	[OMCI_MT_START_SW_DL]		= "Start-SW-dl",
	[OMCI_MT_DOWNLOAD_SEC]		= "DL-section",
	[OMCI_MT_END_SW_DL]		= "End-SW-dl",
	[OMCI_MT_ACTIVATE_SW]		= "Activate-SW",
	[OMCI_MT_COMMIT_SW]		= "Commit-SW",
	[OMCI_MT_SYNC_TIME]		= "Sync-time",
	[OMCI_MT_REBOOT]		= "Reboot",
	[OMCI_MT_GET_NEXT]		= "Get-next",
	[27]				= "Test-result",
	[28]				= "Get-current-data",
	[29]				= "Set-table",
	/* ⚠ 27/28/29 stay NUMERIC: the core declares no OMCI_MT_* for them, and
	 * naming one here would be a second spelling again -- the exact defect
	 * this change removes.  They are owed a core #define, not a local one. */
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
