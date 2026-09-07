/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE, and in the STRICT host-buildable subset -- a struct, the sample
 * points, and a string formatter.  No register, no bus, no device pointer, no
 * lock, no clock: the caller's time arrives as an argument.
 *
 * gpon_ploam_diag -- WHAT is worth reading at an activation transition, and
 * the SHAPE of the line that reports it.
 *
 * ★ WHY IT EXISTS (2026-09-06).  The O4 wall on both Luna boards was split by
 * hand-adding the BWmap acceptance counters to the Deactivate log line,
 * building, reading one boot, and reverting -- and the same class of probe
 * had already been hand-added on the Cortina board before that.  The third
 * time is a defect in the environment, not a missing log line: the question
 * "between Assign_ONU-ID and what followed, did a grant reach us, did our GTC
 * accept it, and did the two ONU-ID registers agree" is the same on every
 * silicon.  So it is asked ONCE, here, and each family answers it from its
 * own registers through `struct gpon_ploam_diag`.
 *
 * ★ THE CONTRACT THAT KEEPS A PHANTOM OUT OF THE LOG.  A family fills only
 * what its silicon ESTABLISHES and sets the matching GPON_PDIAG_HAS_* bit; a
 * clear bit renders as `n/a`, never as 0.  On a counter, "0 grants" is a
 * device finding and "could not ask" is not, and a reader cannot tell them
 * apart once both are spelled the same.
 *
 * ★ MEASURED ON THE RTL9602C, 2026-09-06: its DS misc counters are
 * CLEAR-ON-READ (ploam_acpt read 30, then 6, then 0 across three reads
 * ~60 ms apart while the OLT kept talking; /proc/gpon's two lines that read
 * the same words print the second as 0 for the same reason).  A value is
 * therefore "since the previous read", and two readers steal from each other.
 * The family owning the read says so beside its accessor; it is recorded
 * here because the consumer of this line is a human, and a human reading
 * `bwm_acpt=0` needs to know which zero it is.
 *
 * ⚠ The CALLER owns the policy -- level, rate, and which points it samples.
 *   This file only ever FORMATS INTO A BUFFER; it cannot print, so it cannot
 *   flood, and with CONFIG_GPON_PLOAM_DIAG=n it is not built at all.
 */
#ifndef GPON_PLOAM_DIAG_H
#define GPON_PLOAM_DIAG_H

#include <linux/types.h>
#include "gpon_ploam.h"		/* enum gpon_ploam_ev: the WHEN the core already owns */

/* The WHEN, as data.  These are the activation transitions at which the
 * counters below decide something; every other event is GPON_PDIAG_NONE. */
enum gpon_ploam_diag_point {
	GPON_PDIAG_ASSIGN,		/* Assign_ONU-ID accepted -> O4 (one per copy the OLT sends) */
	GPON_PDIAG_RANGING_TIME,	/* Ranging_Time accepted -> O5: the ranging grant WAS answered */
	GPON_PDIAG_DEACT,		/* Deactivate_ONU-ID: the window closed without ranging     */
	GPON_PDIAG_NONE,		/* not a sample point                                        */
};

#define GPON_PDIAG_HAS_PLOAM_ACPT	0x01u
#define GPON_PDIAG_HAS_BWM_ACPT		0x02u
#define GPON_PDIAG_HAS_BWM_FAIL		0x04u
#define GPON_PDIAG_HAS_BWM_INV		0x08u
#define GPON_PDIAG_HAS_US_ONU_ID	0x10u
#define GPON_PDIAG_HAS_DS_ONU_ID	0x20u
#define GPON_PDIAG_HAS_US_PLOAM_TX	0x40u
#define GPON_PDIAG_HAS_ALL		0x7fu

struct gpon_ploam_diag {
	u32 valid;		/* GPON_PDIAG_HAS_*: a clear bit is COULD NOT ASK       */
	u32 ploam_acpt;		/* DS PLOAMs that passed the ONU-ID filter               */
	u32 bwm_acpt;		/* BWmap grants accepted for our Alloc-ID / ONU-ID       */
	u32 bwm_fail;		/* BWmap grants rejected (CRC / format)                  */
	u32 bwm_inv;		/* BWmap grants invalid                                  */
	u32 us_ploam_tx;	/* upstream PLOAMs the MAC actually transmitted (CPU-queued + auto-fired): the "did we ANSWER the grant" half */
	u8  us_onu_id;		/* the ONU-ID the upstream burst is stamped with (readback) */
	u8  ds_onu_id;		/* the ONU-ID the downstream filter holds (readback)     */
};

/* Which sample point a core PLOAM event is, or GPON_PDIAG_NONE.  A shell
 * that dispatches through the core FSM asks this from its `trace` op; one
 * that still runs its own FSM names the point at its own handler. */
enum gpon_ploam_diag_point gpon_ploam_diag_point_of(enum gpon_ploam_ev ev);

/* "assign" / "ranging_time" / "deact"; "?" for anything else, never NULL. */
const char *gpon_ploam_diag_point_name(enum gpon_ploam_diag_point p);

/*
 * One line, fields in a fixed order so a reader can grep it:
 *   ploam-diag <point> t=<ms>ms O<state> ploam_acpt=.. bwm_acpt=.. bwm_fail=..
 *              bwm_inv=.. us_ploam_tx=.. us_onu_id=.. ds_onu_id=..
 * with `n/a` wherever the matching GPON_PDIAG_HAS_* bit is clear.  `t_ms` is
 * the caller's clock (this layer has none); `ostate` the O-state 1..5 at the
 * moment of the read.  scnprintf semantics: never more than `sz - 1`.
 */
int gpon_ploam_diag_format(char *out, size_t sz, enum gpon_ploam_diag_point p,
			   u32 t_ms, u8 ostate, const struct gpon_ploam_diag *d);

#endif /* GPON_PLOAM_DIAG_H */
