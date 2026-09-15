/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE, strict host-buildable subset.
 *
 * CONFIG_GPON_GEM_DIAG -- third member of the CONFIG_GPON_<SUBSYSTEM>_DIAG
 * family, beside GPON_PLOAM_DIAG and GPON_OMCI_DIAG.  It reports the WAN DATA
 * PATH: what the OLT provisioned, what PLOAM authorised, what is armed in
 * silicon, and -- the part nothing could say before -- WHICH precondition is
 * missing when nothing is armed.
 *
 * ★★★ A DECLARED CAPABILITY, NOT A PROBE SOMEBODY ADDS AGAIN.  "O5 and no data
 *     GEM" has been asked on every board this project has brought up and was
 *     answered each time by hand-adding prints and reverting them.  The name is
 *     the SUBSYSTEM, never the chip: one flag serves Luna and Cortina alike.
 *
 * ⚠ IT REPORTS.  It changes no decision and writes no register: the plan and
 *   the blocker both come from the PURE functions in gpon_data_plan.h, so the
 *   line cannot disagree with the behaviour it describes.
 */
#ifndef GPON_GEM_DIAG_H
#define GPON_GEM_DIAG_H

#include <linux/types.h>
#include "gpon_data_plan.h"

/*
 * Render ONE line describing the data path right now.  Returns the characters
 * written (0 when it could write nothing), never writing past @sz.
 *
 * The two INPUTS then the two VERDICTS, in that order, because a reader's first
 * question is always "what did the OLT actually say":
 *
 *   want alloc=0x0100 known=1 gem=0x00c1 omcc=up/0x0100 | armed alloc=0x0000
 *   bound=0 gem=0x0000 omcc-ride=0 installed=0 | plan=WAIT blocked=alloc-unknown
 */
int gpon_gem_diag_line(const struct gpon_data_armed *armed,
		       const struct gpon_data_want *want,
		       char *out, size_t sz);

#endif /* GPON_GEM_DIAG_H */
