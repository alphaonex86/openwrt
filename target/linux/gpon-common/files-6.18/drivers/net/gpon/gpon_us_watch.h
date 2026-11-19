/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIER: CORE (prefix gpon_) — decides, never touches hardware; time is an input.
 *
 * gpon_us_watch.h — the PON upstream as an egress PAST the switch: the frames the PON MAC took from the switch
 * against those it transmitted, judged by the common egress watch (8 taken in 10 s, none sent, is one stall) only
 * while the ONU is in service: O5, the OMCC installed. The switch keeps handing frames to a PON MAC whose upstream
 * feed is parked, so the switch-side watch never sees this one.
 * Lab X111W 2026-11-19: the upstream GMII TX enable gated off 16 s, then back, left the WAN dead 17+ min with O5 and
 * carrier kept (pages staged, the underflow latched, grants still accepted); re-arming the feed brought it back in
 * 6 s. Stock counts the underflow and recovers nothing.
 * An OLT that grants this ONU nothing gives the same shape and is not ours to recover: a stall is a wedge only while
 * grants arrive (re-armed under the restart budget, never a reboot); without grants it is named and counted, nothing
 * spent; a chip that cannot count grants says so and is re-armed at most once per stall, the budget never asked.
 */
#ifndef GPON_US_WATCH_H
#define GPON_US_WATCH_H

#include "eth_queue_watch.h"

#define GPON_US_GRANTS_UNKNOWN	(~0u)	/* the chip has no grant counter: COULD NOT ASK */

enum gpon_us { GPON_US_OK, GPON_US_WEDGE, GPON_US_SPENT, GPON_US_GRANTLESS, GPON_US_UNASKED };

struct gpon_us_watch {
	struct eth_egress_watch e;
	bool granted;		/* a grant arrived since frames started waiting */
	u32 grantless;		/* stalls through which the OLT granted nothing */
	u32 unasked;		/* stalls judged without grant evidence */
};

/* `grants` = the grants accepted for this ONU since the last sample, or GPON_US_GRANTS_UNKNOWN. */
static inline enum gpon_us gpon_us_watch_sample(struct gpon_us_watch *w, bool in_service, u32 grants, u32 taken,
						u32 sent, u64 now_ms)
{
	struct eth_egress_watch *e = &w->e;
	bool known = grants != GPON_US_GRANTS_UNKNOWN;

	if (!in_service) {
		e->offered = taken;	/* out of service nothing is owed: back in service, the count starts again */
		e->sent = sent;
		e->since_ms = 0;
		return GPON_US_OK;
	}
	if (!e->since_ms)
		w->granted = false;
	w->granted |= known && grants;
	if (!eth_egress_watch_stalled(e, taken, sent, now_ms))
		return GPON_US_OK;
	if (!known) {
		w->unasked++;
		return GPON_US_UNASKED;
	}
	if (!w->granted) {
		w->grantless++;
		return GPON_US_GRANTLESS;
	}
	return eth_egress_watch_spend(e, now_ms) == ETH_WATCH_RECOVER ? GPON_US_WEDGE : GPON_US_SPENT;
}

#endif
