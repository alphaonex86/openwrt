/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MAY THIS ONU RANGE, AND WHAT MUST CHANGE IF NOT.
 *
 * The decision is protocol, not silicon: an ONU may transmit upstream only
 * when it knows who it is AND its laser can burst. Both families ask exactly
 * that question, so it is asked ONCE, here, with no MMIO and no device -- which
 * is also what makes the LIVE re-provision path testable, and it is the path
 * that had the defects.
 *
 * ⚠ WHY IT IS A PLAN AND NOT A BOOL. A refusal on a board that is ALREADY
 *   ranging has to CLEAR the GO bit, not merely decline to set it. A shell that
 *   returns early from a fresh-probe path looks correct and leaves a live ONU
 *   transmitting under an identity it has just been told is wrong.
 */
#ifndef _GPON_RANGE_GATE_H
#define _GPON_RANGE_GATE_H

#include <linux/types.h>

enum gpon_range_reason {
	GPON_RANGE_GO,			/* arm it                            */
	GPON_RANGE_UNCHANGED,		/* already ranging with this identity */
	GPON_RANGE_NO_IDENTITY,		/* no defined serial number          */
	GPON_RANGE_NOT_WANTED,		/* activation switched off           */
	GPON_RANGE_LASER_OFF,		/* laser deliberately unprogrammed   */
	GPON_RANGE_LASER_FAILED,	/* the laser could not be programmed */
};

struct gpon_range_request {
	bool identity_defined;	/* the serial in force is a real one         */
	bool identity_changed;	/* it differs from the one last armed with   */
	bool ranging_now;	/* the GO bit is asserted RIGHT NOW          */
	bool activation_wanted;	/* the operator has not switched ranging off */
	bool laser_wanted;	/* the operator has not switched the laser off */
};

/*
 * THE ORDER IS THE ANSWER, which is why this is three flags and not a verdict.
 *
 * ⚠ A LIVE RE-PROVISION MUST QUIESCE BEFORE IT PROGRAMS ANYTHING. The serial
 *   number and the MAC configuration are documented as write-while-en=0, and a
 *   serial CHANGED through /proc arrives with en ALREADY SET -- so a plan that
 *   merely said "arm" left the identity being rewritten under a running FSM,
 *   and the driver's own "config while en=0" comment was false on exactly the
 *   path a subscriber travels. The laser is programmed between the two, because
 *   its bus traffic must not happen while the part is still bursting either.
 */
struct gpon_range_action {
	bool quiesce;		/* FIRST: clear the GO bit                   */
	bool program;		/* THEN: program the laser, config, and arm  */
	bool activated;		/* the readiness flag IF those all succeed   */
	int  result;		/* what the caller that asked is told        */
	enum gpon_range_reason reason;
};

void gpon_range_plan(const struct gpon_range_request *r,
		     struct gpon_range_action *a);
const char *gpon_range_reason_str(enum gpon_range_reason why);

#endif /* _GPON_RANGE_GATE_H */
