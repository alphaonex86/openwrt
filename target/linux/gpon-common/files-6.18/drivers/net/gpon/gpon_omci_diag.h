/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE, strict host-buildable subset -- G.988 byte math and a formatter.
 *
 * gpon_omci_diag -- the ONE spelling of the per-PDU downstream OMCI trace line,
 * gated by CONFIG_GPON_OMCI_DIAG.  Third member of the
 * CONFIG_GPON_<SUBSYSTEM>_DIAG family, after CONFIG_GPON_PLOAM_DIAG.
 *
 * ★ WHY IT EXISTS (2026-09-11).  The 2026-09-10 extraction moved the DECODE
 * into the core and left the LINE behind, so one idea had two spellings and the
 * suite read only one: every Luna board's OMCI trace was invisible with the
 * lines in dmesg, and the resulting BLOCK named a knob that cannot exist on
 * that shell.  A reader keyed on one spelling of two goes blind on half the
 * fleet without saying so.
 *
 * ⚠ THE THREE ANSWERS A GET LINE CAN CARRY, AND THEY MAY NOT BE SPELLED ALIKE.
 *   ` mask=.. rmask=.. unsup=.. failed=.. rc=..`  the responder answered
 *   ` noresp`      the responder deliberately built no response -- a DEVICE fact
 *   ` resp=n/a`    this shell does not report it at all -- COULD NOT ASK, an
 *                  INSTRUMENT fact, and never a zero mask.
 * A missing mask rendered as 0x0000 reads as "the OLT requested nothing", which
 * is a device finding nobody measured.
 *
 * ⚠ The CALLER owns level, rate limit and sampling.  This file only FORMATS
 *   INTO A BUFFER, so it cannot print and cannot flood.
 */
#ifndef GPON_OMCI_DIAG_H
#define GPON_OMCI_DIAG_H

#include <linux/types.h>

/* Pass as `resp_len` when the shell has no responder output to report here.
 * Distinct from 0, which means "a response was expected and none was built". */
#define GPON_OMCI_NO_RESP_REPORT	(-1)

/*
 * The whole trace line body: what gpon_omci_describe() says about the PDU, plus
 * the Get exchange detail.  Returns the characters written (scnprintf
 * semantics), 0 for a PDU too short to describe.
 */
int gpon_omci_diag_line(const u8 *pdu, unsigned int len,
			const u8 *resp, int resp_len, char *out, size_t sz);

#endif /* GPON_OMCI_DIAG_H */
