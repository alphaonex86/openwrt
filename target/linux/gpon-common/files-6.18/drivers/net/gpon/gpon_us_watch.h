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
 * 6 s. Stock counts the underflow and recovers nothing. An OLT that stops granting the data T-CONT gives the same
 * shape: the restart budget bounds it (RESTART_TRIES re-arms in a row, then a reboot).
 */
#ifndef GPON_US_WATCH_H
#define GPON_US_WATCH_H

#include "eth_queue_watch.h"

static inline enum eth_watch gpon_us_watch_sample(struct eth_egress_watch *e, bool in_service, u32 taken, u32 sent,
						  u64 now_ms)
{
	if (in_service)
		return eth_egress_watch_sample(e, taken, sent, now_ms);
	e->offered = taken;	/* out of service nothing is owed: back in service, the count starts again */
	e->sent = sent;
	e->since_ms = 0;
	return ETH_WATCH_OK;
}

#endif
