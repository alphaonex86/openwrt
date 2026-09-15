/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIER: CORE (prefix gpon_) — decides, never touches hardware, and compiles for
 * MIPS-BE, ARM64-LE and x86.  Canonical tier rule and guard: see "THE THREE
 * TIERS" in gpon_common.h (this directory).
 *
 * gpon_gem_us.h — the UPSTREAM half of GEM provisioning: the two mappings a
 * G.984 ONU must get right before it may transmit.
 *   1. Alloc-ID -> T-CONT: may an OLT-assigned Alloc-ID be bound to the data
 *      T-CONT, or is it already the OMCC's?  Both targets implemented this
 *      predicate independently and BOTH produced a real outage from it.
 *   2. GEM Port-ID -> the upstream index slots that stamp it, as a DECLARED
 *      range the shell supplies — never a computed one.
 * A decision layer, not a lifecycle one: it holds no state, so the same inputs
 * are the same answer on any target, at any time, in any context.
 *
 * ★ NO GEM HEADER BUILD, FRAGMENTATION OR HEC HERE, and nothing to hoist:
 *   measured 2026-08-05, the GEM Transmission Convergence layer is SILICON on
 *   both shipping targets.  The INDEPENDENT model of it lives in
 *   dev/rtl9607c-oracle/ (gem_tc.c, gem_us_encap.c) and MUST stay there -- a
 *   reference merged into the thing it checks can no longer fail.
 */
#ifndef GPON_GEM_US_H
#define GPON_GEM_US_H

#include <linux/types.h>

/* ★ THREE DIFFERENT 12-BIT-ISH THINGS ARE ALL CALLED "gem" IN THIS TREE, and
 * confusing two of them has already cost a board-proven regression:
 *   GEM Port-ID  the number ON THE WIRE (G.984.3).  The only one of the three
 *                that is a protocol value, and the only one spelled "port_id".
 *   US index     the per-chip slot in the upstream port-map array that STAMPS a
 *                Port-ID onto a burst.  Called "index"/"range", never "gem".
 *   mcgid        (Cortina only) an egress LDPID port-group, not a GEM number at
 *                all.  Set to a GEM Port-ID once, it steered the hit frame to
 *                the WRONG egress and the far end received nothing
 *                (cortina-ni-flowoffload.c:2156).  It appears nowhere here.
 */

/* G.984.3 field widths: Alloc-ID and GEM Port-ID are each 12 bits, which is why
 * they may live in common code, and both targets already mask with exactly
 * these.  The T-CONT index width is NOT here — 5 bits is a CAM-index width the
 * two silicons happen to share, not something G.984.3 defines. */
#define GPON_GEM_US_PORT_MASK	0x0fffu
#define GPON_GEM_US_ALLOC_MASK	0x0fffu

/* The value written to an upstream port-map slot that must stamp nothing.
 * ★ Only elnath ever writes it; luna has NO upstream unstamp at all — its
 *   re-arm is a flag, so a stale Port-ID survives until the next install
 *   overwrites it.  That asymmetry is REAL and is preserved: making luna clear
 *   its slot would change what the silicon holds after a Deactivate. */
#define GPON_GEM_US_PORT_NONE	0u

/* The upstream index slots that carry ONE GEM Port-ID, DECLARED by the shell.
 *
 * ★ DECLARED, NEVER COMPUTED — the reason this struct exists.  On elnath the
 *   slot IS the VoQ (CG_DATA_GEM_IDX = CG_DATA_TCONT_IDX * 8, 8 queues per
 *   T-CONT); on luna it is a fixed GTC flow/SID per role with no relation to a
 *   T-CONT.  `index = tcont * 8` is TRUE on Cortina and FALSE on Luna, so no
 *   function here derives a base from a T-CONT.
 * @index_max is the last slot the chip's array ACTUALLY has (luna 127, elnath
 * 255): a count, a maximum, a stride and an index space are four different
 * quantities, and a write past the end of the port-map array lands in an
 * unrelated register and is accepted without complaint. */
struct gpon_gem_us_range {
	u16	base;		/* first upstream slot that stamps the Port-ID */
	u16	count;		/* how many consecutive slots (>= 1)           */
	u16	index_max;	/* last slot the chip's port-map array has     */
};

/* Compile-time range check, usable in static_assert() with literal per-chip
 * constants: a bad declared range fails the BUILD and emits no code. */
#define GPON_GEM_US_RANGE_OK(base, count, index_max)			\
	((count) >= 1u && (u32)(base) + (u32)(count) - 1u <= (u32)(index_max))

/* Runtime form of the same predicate.  Reports only; it never repairs. */
bool gpon_gem_us_range_ok(const struct gpon_gem_us_range *r);

/* The n-th upstream slot of @r.  Meaningful only when gpon_gem_us_range_ok(). */
static inline u16 gpon_gem_us_index(const struct gpon_gem_us_range *r,
				    unsigned int n)
{
	return (u16)(r->base + n);
}

/* Mask a GEM Port-ID / an Alloc-ID to its 12 on-wire bits, in one place. */
static inline u16 gpon_gem_us_port_id(u32 port_id)
{
	return (u16)(port_id & GPON_GEM_US_PORT_MASK);
}
static inline u16 gpon_gem_us_alloc_id(u32 alloc)
{
	return (u16)(alloc & GPON_GEM_US_ALLOC_MASK);
}

/* The Alloc-ID -> T-CONT verdict.  Three outcomes, because "do not bind" is
 * more than one fact and collapsing them is how a shell ends up unable to say
 * WHY the data path never came up.  BIND_IS_OMCC means binding would MOVE the
 * OMCC off its own T-CONT. */
enum gpon_gem_us_bind {
	GPON_GEM_US_BIND_TCONT = 0,
	GPON_GEM_US_BIND_DONE,
	GPON_GEM_US_BIND_IS_OMCC,
};

/* ★★★ SINGLE-ALLOC OLTs: THE DATA RIDES THE OMCC T-CONT.
 *
 * BIND_IS_OMCC says what must NOT happen, not what to do instead, and a shell
 * that only WARNED then fell through was the defect: it stamped the data GEM
 * into a dedicated T-CONT with no CAM entry, so no grant ever drained it --
 * every upstream frame queued forever, carrier-on lied to udhcpc, and the latch
 * blocked any retry.  A silent permanent dead WAN on the WHOLE single-alloc OLT
 * class.  The recovery is 9602C-PROVEN on this lab OLT class (30/30 soak,
 * 354/354 DNS): stamp the data GEM into the OMCC T-CONT's own upstream slots,
 * using the ones the US OMCI does not need, and steer data TX there.
 *
 * ★ OMCI KEEPS THE TOP SLOTS: the chip serves a T-CONT's queues by STRICT
 *   PRIORITY, so a saturated data flow can never starve a PLOAM/OMCI response.
 *   Reversing the split would make an OMCI timeout a function of user traffic.
 *   The GEOMETRY is the shell's; the SPLIT is protocol. */
#define GPON_GEM_US_OMCI_RESERVED_SLOTS	2	/* the top two: OMCI + PLOAM */

/* Derive the sub-range of @omcc the data GEM may use.  -> false when the OMCC
 * run is too short to spare a slot, and the caller must then install NO data
 * path rather than steal an OMCI slot. */
bool gpon_gem_us_ride_range(const struct gpon_gem_us_range *omcc,
			    struct gpon_gem_us_range *out);

/* Decide whether @alloc may be bound to the data T-CONT.  @omcc_alloc and
 * @already_bound are the CALLER's own values, passed through unchanged because
 * the two targets genuinely pass different ones and a code-motion refactor may
 * not change either.  Pure: safe from any context including softirq. */
enum gpon_gem_us_bind gpon_gem_us_tcont_decide(u16 alloc, u16 omcc_alloc,
					       bool already_bound);

/* One-line name for a verdict, for logs and for test failure messages. */
const char *gpon_gem_us_bind_name(enum gpon_gem_us_bind v);

/* ★★★ ONE PROTOCOL RULE THAT WAS WRITTEN THREE TIMES — the core PLOAM FSM, the
 * Luna native FSM (the copy that SHIPS) and the Cortina shell, none able to
 * call another (2026-09-03).
 *
 * ⚠ THE COST IS MEASURED: as a ONE-SHOT guard ("already installed, do nothing")
 *   an OLT that moves the OMCC to another GEM after O5 leaves us bound to the
 *   old port.  DS de-encap follows the MAC's own re-latch, so DS keeps working
 *   while our upstream OMCI replies ride a port the OLT no longer accepts: its
 *   audit times out, it DEACTIVATES us, and only a full re-range recovers.  The
 *   Luna copy carried that for weeks after the other two were repaired. */
enum gpon_omcc_action {
	GPON_OMCC_IGNORE,	/* enable=0: a Configure_Port-ID transient. Write
				 * NOTHING and keep the link; the following
				 * enable=1 re-latch carries the final id */
	GPON_OMCC_INSTALL,	/* not bound yet -> bind @want_gem */
	GPON_OMCC_REBIND,	/* bound to a DIFFERENT gem -> move the TRANSPORT
				 * only.  The responder session is NOT re-armed:
				 * re-arming resets the MIB under a live OLT */
	GPON_OMCC_UNCHANGED,	/* the same gem re-sent (PLOAMs come 3x): write
				 * nothing, so the proven LOS/fiber-pull keep
				 * path stays byte-identical */
};

/**
 * gpon_omcc_decide - what a Configure_Port-ID means for the OMCC transport
 * @port_en:    d[0] bit0, the OLT's enable
 * @want_gem:   the GEM the OLT is assigning, (d[1] << 4) | (d[2] >> 4)
 * @installed:  the caller's shadow -- is the OMCC bound at all?
 * @cur_gem:    the caller's shadow -- which gem (meaningless if !@installed)
 *
 * The caller ACTS and owns its shadow, and must update it ONLY after a
 * successful write: on failure the OLD gem has to survive, so the next event
 * retries to convergence instead of latching a half-done rebind.
 * Pure: no state, no side effect, safe from any context including softirq.
 */
enum gpon_omcc_action gpon_omcc_decide(bool port_en, u16 want_gem,
				       bool installed, u16 cur_gem);

/* One-line name for a verdict, for logs and for test failure messages. */
const char *gpon_omcc_action_name(enum gpon_omcc_action a);

/* ★★★ THE SECOND OMCC RULE THAT WAS WRITTEN TWICE: gpon_omcc_decide() settled
 * WHICH GEM the OMCC rides; this is the other half, from Assign_ONU-ID — WHICH
 * ALLOC-ID the OMCC's management T-CONT binds to.  Still spelled twice with
 * different variable names on 2026-09-10, and the copy that BOOTS was the Luna
 * one -- the same asymmetry that hid the Configure_Port-ID one-shot defect. */
struct gpon_omcc_tcont_plan {
	u16  alloc;		/* Alloc-ID to bind the OMCC's own T-CONT to  */
	bool bind_alt;		/* ALSO bind @alloc to the ALTERNATE T-CONT   */
};

/**
 * gpon_omcc_tcont_decide - which Alloc-ID the OMCC's management T-CONT takes
 * @alloc_override: a board/module override, 0 = "use the live ONU-ID"
 * @onu_id:         the ONU-ID the OLT just assigned
 * @alt_bind:       the NON-STOCK double-bind knob (default off everywhere)
 * @out:            filled with the plan; untouched when @out is NULL
 *
 * The OMCC upstream Alloc-ID IS the live ONU-ID (G.984.3 default, and what
 * stock does).  The override exists only to A/B an OLT that grants T-CONT 1
 * rather than 0, and it is a CONSTANT the caller supplies -- never a value
 * captured off the wire, which is what both copies' older comments claimed.
 *
 * @bind_alt stays false when the plan lands on the plain ONU-ID: binding the
 * same alloc to a SECOND T-CONT makes the GTC alloc-CAM resolve a BWMAP grant
 * to the EMPTY one, whose DBRu then reports zero occupancy while the real
 * T-CONT holds the pages -- the OLT grants once and stops.
 *
 * Pure: no state, no side effect, safe from any context including softirq.
 */
void gpon_omcc_tcont_decide(u16 alloc_override, u8 onu_id, bool alt_bind,
			  struct gpon_omcc_tcont_plan *out);

#endif /* GPON_GEM_US_H */
