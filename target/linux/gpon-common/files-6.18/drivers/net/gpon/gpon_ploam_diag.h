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
 * hand-adding the BWmap acceptance counters to the Deactivate line, reading one
 * boot and reverting -- and the same probe had already been hand-added on the
 * Cortina board.  The third time is a defect in the ENVIRONMENT: "between
 * Assign_ONU-ID and what followed, did a grant reach us, did our GTC accept it,
 * and did the two ONU-ID registers agree" is the same question on every
 * silicon, so it is asked ONCE and each family answers it from its own
 * registers through `struct gpon_ploam_diag`.
 *
 * ★ THE CONTRACT THAT KEEPS A PHANTOM OUT OF THE LOG.  A family fills only what
 * its silicon ESTABLISHES and sets the matching GPON_PDIAG_HAS_* bit; a clear
 * bit renders `n/a`, never 0.  "0 grants" is a device finding and "could not
 * ask" is not, and spelled alike a reader cannot tell them apart.
 *
 * ★ MEASURED ON THE RTL9602C, 2026-09-06: its DS misc counters are
 * CLEAR-ON-READ -- ploam_acpt read 30, then 6, then 0 across three reads
 * ~60 ms apart while the OLT kept talking, and /proc/gpon's two lines over the
 * same words print the second as 0 for the same reason.  A value is "since the
 * previous read" and two readers steal from each other; recorded here because
 * a human reading `bwm_acpt=0` needs to know which zero it is.
 *
 * ⚠ The CALLER owns level, rate and sampling.  This file only FORMATS INTO A
 *   BUFFER, so it cannot flood, and with CONFIG_GPON_PLOAM_DIAG=n it is not
 *   built at all.
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
	/* ★★★ THE EARLY LADDER: a dwell at O1, O2 or O3 (2026-09-08). Every
	 * other point here is a TRANSITION, so a board that never leaves O1
	 * produced no line at all -- the G24W's stall, undiagnosable for that
	 * reason alone. The O-state rides in the line's `ostate`, so ONE point
	 * covers O1, O2 and O3 without three near-identical cases. */
	GPON_PDIAG_EARLY,
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
#define GPON_PDIAG_HAS_US_SN_TX		0x80u
#define GPON_PDIAG_HAS_POLL_GAP		0x100u
#define GPON_PDIAG_HAS_RX_BURST		0x200u
#define GPON_PDIAG_HAS_SN_REQ		0x400u
#define GPON_PDIAG_HAS_RNG_REQ		0x800u
#define GPON_PDIAG_HAS_ALL		0xfffu

/*
 * ★ THE READER'S LATENESS IS PART OF THE READING (2026-09-07).  At the SECOND
 * Assign_ONU-ID the lines showed six DS PLOAMs accepted since the first, and
 * zero at the third and at the Deactivate: every message of the OLT's 240 ms
 * sequence had already reached the silicon before the software handled the
 * second one, so the software was draining a backlog and the ONU-ID written at
 * the first Assign went into a window the OLT had partly spent.  Counters
 * reported without saying how late their reader was cannot show that.  Hence
 * `poll_gap_ms` (the shell's poll cadence as it actually ran) and
 * `rx_burst_idx` (DS PLOAMs this same poll dequeued before the one reported:
 * 0 = fresh, N = a backlog of N).
 */
struct gpon_ploam_diag {
	u32 valid;		/* GPON_PDIAG_HAS_*: a clear bit is COULD NOT ASK       */
	u32 ploam_acpt;		/* DS PLOAMs that passed the ONU-ID filter               */
	u32 bwm_acpt;		/* BWmap grants accepted for our Alloc-ID / ONU-ID       */
	u32 bwm_fail;		/* BWmap grants rejected (CRC / format)                  */
	u32 bwm_inv;		/* BWmap grants invalid                                  */
	u32 us_ploam_tx;	/* upstream PLOAMs the MAC actually transmitted, every kind (CPU normal+urgent, auto No_message+SN): the "did we ANSWER the grant" half */
	u32 us_sn_tx;		/* of those, the auto-fired Serial_Number_ONU -- the one a ranging grant is answered with */
	/* ★ THE TWO THAT DECIDE THE O4 FORK (2026-09-07).  The Luna GTC counts,
	 * in hardware, the Serial_Number REQUESTS and the RANGING REQUESTS it
	 * received -- a ranging grant addressed to this ONU is a distinct event to
	 * the silicon, counted whether or not anything was transmitted back.  So
	 * `rng_req` > 0 in O4 means the OLT DID grant us and the fault is in our
	 * answer; 0 means no ranging request reached the GTC at all.  `sn_req` is
	 * the in-line positive control: the SN request(s) answered in O3. */
	u32 sn_req;		/* Serial_Number requests the GTC received since the previous read */
	u32 rng_req;		/* ranging requests the GTC received since the previous read */
	u32 poll_gap_ms;	/* ms between the start of the poll handling this message and the start of the previous poll */
	u8  rx_burst_idx;	/* DS PLOAMs this same poll dequeued BEFORE this one (0 = the first) */
	u8  us_onu_id;		/* the ONU-ID the upstream burst is stamped with (readback) */
	u8  ds_onu_id;		/* the ONU-ID the downstream filter holds (readback)     */
};

/*
 * ★ THE ACCEPTED-GRANT CAPTURE, accumulated between two sample points
 * (2026-09-07).  The Luna GTC keeps a capture buffer of the BWmap allocations
 * it ACCEPTED, post-filter -- measured on the RTL9602C with a second ONU Online
 * on the same PON: 20 s of capture held exactly ONE entry, the SN grant we
 * answered, and none of the other ONU's.  The shell harvests it every poll
 * while activating, decodes its own capture format and hands over counts plus
 * the raw words of the last PLOAMu allocation resolving to the OMCC T-CONT; the
 * core only counts and formats.
 *
 * `valid` clear = COULD NOT ASK, every field n/a.  Set, a zero is a zero: "no
 * grant accepted between these two points" is the finding this exists for.
 */
#define GPON_BWCAP_HAS			0x01u

struct gpon_bwcap_diag {
	u32 valid;		/* GPON_BWCAP_HAS, or 0 = COULD NOT ASK                   */
	u32 harvests;		/* how many polls emptied the capture into this          */
	u32 nonempty;		/* of those, how many found at least one VALID entry     */
	u32 entries;		/* VALID allocations accepted, all T-CONTs               */
	u32 ploamu;		/* of those, carrying a PLOAMu slot                      */
	u32 omcc_ploamu;	/* of those, resolving to the OMCC T-CONT (the ranging grant's home) */
	u32 tconts;		/* bitmap of the T-CONT indexes the accepted grants resolved to */
	u32 overfl;		/* harvests that found the capture's overflow flag set   */
	u32 last_raw0;		/* the last OMCC-PLOAMu entry, raw, family layout         */
	u32 last_raw1;
};

/*
 * One line, fields in a fixed order:
 *   bwcap-diag <point> t=<ms>ms O<state> harvests=.. nonempty=.. entries=..
 *              ploamu=.. omcc_ploamu=.. tconts=0x.. overfl=.. last=<raw0>/<raw1>
 * `last=-` when no OMCC-PLOAMu entry was accepted (a real absence);
 * every field n/a when GPON_BWCAP_HAS is clear.  scnprintf semantics.
 */
int gpon_bwcap_diag_format(char *out, size_t sz, enum gpon_ploam_diag_point p,
			   u32 t_ms, u8 ostate, const struct gpon_bwcap_diag *b);

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
 *              us_sn_tx=.. poll_gap_ms=.. rx_burst=.. sn_req=.. rng_req=..
 * with `n/a` wherever the matching GPON_PDIAG_HAS_* bit is clear.  `t_ms` is
 * the caller's clock (this layer has none); `ostate` the O-state 1..5 at the
 * moment of the read.  scnprintf semantics: never more than `sz - 1`.
 */
int gpon_ploam_diag_format(char *out, size_t sz, enum gpon_ploam_diag_point p,
			   u32 t_ms, u8 ostate, const struct gpon_ploam_diag *d);

#endif /* GPON_PLOAM_DIAG_H */
