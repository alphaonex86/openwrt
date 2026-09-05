/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: the WAN data path's RECONCILE verdict — armed vs provisioned.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 *
 * gpon_data_plan.h — "is what is ARMED still what the OLT WANTS?", answered
 * once instead of three times.
 *
 * ★★★ THE SAME QUESTION WAS ANSWERED IN THREE PLACES, EACH COVERING A
 *     DIFFERENT SUBSET, AND NONE OF THEM COULD CALL ANOTHER (measured
 *     2026-09-04, by reading all three):
 *
 *   1. gpon_ploam.c:1201-1203 — the core's own re-arm:
 *          if (o->data_installed && port_id != o->data_gem_port)
 *                  o->data_installed = false;
 *      It notices the GEM MOVING.  It has NO equivalent for the ALLOC moving,
 *      and none for the ride-premise being lost.
 *   2. gpon-luna.c:8728-8732 — Luna's only stale-clear, and it is not a
 *      reconcile at all: it fires on an explicit OLT Deallocate
 *      (d[2] == 0xff && installed && alloc == gpon_data_alloc).  An OLT that
 *      REASSIGNS without deallocating is invisible to it.
 *   3. cortina-gpon.c cg_data_try_install() — the widest of the three, and
 *      the only one that carries the two clauses below.
 *
 *   This is exactly the shape gpon_omcc_decide() was created for on
 *   2026-09-03 (gpon_gem_us.h: "ONE PROTOCOL RULE THAT WAS WRITTEN THREE
 *   TIMES").  The rule here is the same kind of fact: G.984.3 lets the OLT
 *   reassign an Alloc-ID or a GEM Port-ID at any time, so an ONU that latches
 *   "installed" and never re-asks will burst a stale GEM into a grant slot
 *   that now belongs to somebody else.  Which registers hold the binding is
 *   silicon; whether the binding is still the right one is protocol.
 *
 * ★ THE TWO CLAUSES THAT EXIST IN NEITHER OTHER COPY, spelled out because
 *   they are the whole reason this file is not just a rename:
 *
 *   (a) THE ALLOC MOVED WITH THE GEM UNCHANGED.  The core (1) compares only
 *       the GEM.  An OLT that re-provisions ME 262 onto a different Alloc-ID
 *       while keeping ME 268 leaves the ONU's T-CONT CAM pointing at an
 *       Alloc-ID the OLT no longer grants: upstream simply stops, with every
 *       shadow reading healthy.
 *
 *   (b) THE RIDE PREMISE WAS LOST.  On a single-alloc OLT the data path is
 *       installed on the OMCC's OWN T-CONT because the data Alloc-ID WAS the
 *       OMCC's (gpon_gem_us.h, GPON_GEM_US_BIND_IS_OMCC).  If the OLT then
 *       moves the OMCC to a different Alloc-ID mid-O5, that premise is gone:
 *       the data stamps stay in the OMCC T-CONT's queues and the user traffic
 *       silently follows whatever Alloc-ID the OMCC moved to — which is not
 *       the one provisioned for it.  Neither {alloc, gem} comparison can see
 *       this, because neither of those two values changed.
 *
 * ★ WHAT THIS FILE DELIBERATELY DOES *NOT* DO.  It decides; it never does, and
 *   it does not even ASK for a teardown through an op table:
 *     - gpon_gem_us.c:196-200 already records that the upstream TEARDOWN ORDER
 *       is a hardware requirement (drain the T-CONT's VoQs FIRST), so a core
 *       may request a teardown but performing one is the shell's, in the
 *       shell's order.  Nothing here sequences anything.
 *     - struct gpon_shell_ops::data_teardown is DECLARED in gpon_common.h, but
 *       the tree contains ZERO instances of struct gpon_shell_ops (checked
 *       2026-09-04: `grep -rn 'struct gpon_shell_ops [a-z_]* *=' --include=*.c`
 *       over target/linux returns rc=1), and luna_ponmac.c:122 says in so many
 *       words "DO NOT ADD a struct gpon_shell_ops instance to this file".  So
 *       the core cannot today CALL a teardown.  Everything below is a PURE
 *       VERDICT the shell asks for, never an op it registers — which is also
 *       what keeps it fuzzable at thousands of cases per second on x86.
 *
 * ★ HEADER-ONLY, ON PURPOSE.  static inline in a .h, so it needs no Makefile
 *   line and no object.  The precedent in this tree is flowcore/regtable.h
 *   (16 static inlines).  ⚠ THE CONSEQUENCE MUST BE SAID OUT LOUD: the strict-
 *   subset host-build gate, gpon_layer_hostbuild_test.sh, walks a list of .c
 *   files, so it does NOT reach this header.  Its host-build coverage is
 *   dev/rtl9607c-test/gpon_data_plan_diff_test.c, which #includes this file
 *   directly and compiles it on x86 — that binary IS the purity proof for this
 *   file, and deleting it would silently remove one.
 *
 * ★ THE HONEST CAVEAT, so nobody has to re-derive it.  The RECONCILE half
 *   below is the load-bearing one: it is the third partial answer to one
 *   question and it adds two clauses the other two copies do not have.  The
 *   UNDO half has exactly ONE consumer today — Luna has no data teardown at
 *   all (gpon_common.h:446 records ->data_teardown as NULL there) — so its
 *   value is preventing a future re-derivation, not deduplicating an existing
 *   copy.  That is weaker, and it is not claimed to be more.
 */
#ifndef GPON_DATA_PLAN_H
#define GPON_DATA_PLAN_H

#include <linux/types.h>

/* for the two 12-bit wire masks: the armed identity is stored MASKED and the
 * provisioned one is not, so the comparison must mask exactly where the shell
 * masked before.  gpon_gem_us.h is the CONCEPTUAL home of this decision, but
 * it is a strict-subset file with its own .c and its own guard, so this lives
 * beside it rather than inside it. */
#include "gpon_gem_us.h"

/*
 * What the SHELL has actually armed in silicon.
 *
 * ⚠ @alloc and @gem are the MASKED, on-wire values, because that is what the
 * shell recorded after a SUCCESSFUL register write.  Zero means "nothing
 * armed" for both — a G.984.3 Alloc-ID of 0 is not a data allocation and GEM
 * Port-ID 0 is not a data port, so the shells' existing `if (hw_data_gem)`
 * spelling loses nothing.
 */
struct gpon_data_armed {
	u16	alloc;		/* Alloc-ID armed in the T-CONT CAM   (0 = none) */
	u16	gem;		/* GEM Port-ID armed in the DS/US CAM (0 = none) */
	bool	rides_omcc;	/* armed on the OMCC's T-CONT, not a dedicated one */
	bool	installed;	/* the WHOLE path is armed */
};

/*
 * What the OLT has provisioned, plus the live OMCC binding.
 *
 * ⚠ @alloc and @gem are UNMASKED, exactly as the shell snooped them out of
 * OMCI ME 262 / ME 268.  The distinction is not cosmetic: the "has the OLT
 * provisioned both halves yet" gate below tests them UNMASKED (that is what
 * both shells do today and Luna has no masked equivalent), while the staleness
 * comparison masks them.  Masking here instead would change behaviour for a
 * provisioned value whose low 12 bits happen to be zero.
 */
struct gpon_data_want {
	u16	alloc;		/* ME 262 Alloc-ID, as read */
	u16	gem;		/* ME 268 GEM Port-ID, as read */
	u16	omcc_alloc;	/* the Alloc-ID actually bound to the OMCC T-CONT */
	bool	omcc_up;	/* the OMCC transport is up */
};

/*
 * The verdict.  Five outcomes, because "do nothing" and "do nothing YET" are
 * different facts and a shell that collapses them cannot say why the data path
 * never came up.
 */
enum gpon_data_plan {
	/* preconditions unmet (no OMCC, or the OLT has provisioned nothing and
	 * nothing is armed) -> write NOTHING.  Not an error: this is the state
	 * an ONU sits in between O5 and the OLT's ME 262/268. */
	GPON_DATA_WAIT = 0,
	/* armed == provisioned -> the PROVEN KEEP-PATH.  Write nothing, take no
	 * branch: this is what makes a LOS/fiber-pull re-range byte-identical
	 * and is why the 30/30 soak holds.  Re-installing here would be
	 * needless register churn on a healthy link. */
	GPON_DATA_KEEP,
	/* nothing armed, both halves provisioned -> install. */
	GPON_DATA_INSTALL,
	/* a DIFFERENT identity is armed -> tear the stale one down FIRST, then
	 * install.  Never install over it: a stale CAM entry that outlives its
	 * grant assignment bursts into somebody else's slot. */
	GPON_DATA_REPLACE,
	/* armed, and the OLT has DEPROVISIONED (or moved) -> undo only.  There
	 * is nothing to install afterwards. */
	GPON_DATA_TEARDOWN,
};

/**
 * gpon_data_armed_is_stale - is the armed identity no longer the wanted one?
 * @armed: what the shell put in silicon
 * @want:  what the OLT provisioned, plus the live OMCC binding
 *
 * Split out from the verdict so a caller (and the differential) can ask the
 * question on its own, and so the three clauses are readable one per line.
 * Nothing armed is never stale — there is nothing to be stale.
 *
 * Pure: no state, no side effect, safe from any context including softirq.
 */
static inline bool gpon_data_armed_is_stale(const struct gpon_data_armed *armed,
					    const struct gpon_data_want *want)
{
	if (!armed || !want || !armed->alloc)
		return false;

	return armed->alloc != gpon_gem_us_alloc_id(want->alloc) ||
	       armed->gem   != gpon_gem_us_port_id(want->gem)    ||
	       /* clause (b): the ride premise.  Only meaningful when we chose
		* the ride route, and it compares the ARMED alloc against the
		* OMCC's CURRENT one — so an OMCC that moved makes the data
		* path stale even though neither ME 262 nor ME 268 changed. */
	       (armed->rides_omcc && armed->alloc != want->omcc_alloc);
}

/**
 * gpon_data_plan_decide - what should happen to the WAN data path right now
 * @armed: what the shell put in silicon
 * @want:  what the OLT provisioned, plus the live OMCC binding
 *
 * The caller ACTS and owns its shadows, and must update them ONLY after the
 * register writes succeeded: on failure the OLD identity has to survive, so
 * the next event retries to convergence instead of latching a half-done
 * install.  That is why nothing here writes anything, not even a shadow.
 *
 * ORDER MATTERS AND IS NOT ARBITRARY.  Staleness is computed BEFORE the
 * "provisioned yet?" gate, because a deprovision (ME 262/268 gone to zero)
 * still owes a teardown of whatever is armed; testing the gate first would
 * leave a stale CAM entry armed forever.
 *
 * Pure: no state, no side effect, safe from any context including softirq.
 */
static inline enum gpon_data_plan
gpon_data_plan_decide(const struct gpon_data_armed *armed,
		      const struct gpon_data_want *want)
{
	bool stale;

	if (!armed || !want || !want->omcc_up)
		return GPON_DATA_WAIT;

	stale = gpon_data_armed_is_stale(armed, want);

	/* "has the OLT provisioned BOTH halves yet" — UNMASKED on purpose, see
	 * struct gpon_data_want. */
	if (!want->alloc || !want->gem)
		return stale ? GPON_DATA_TEARDOWN : GPON_DATA_WAIT;

	if (stale)
		return GPON_DATA_REPLACE;
	if (armed->installed)
		return GPON_DATA_KEEP;
	return GPON_DATA_INSTALL;
}

/* One-line name for a verdict, for logs and for test failure messages. */
static inline const char *gpon_data_plan_name(enum gpon_data_plan p)
{
	switch (p) {
	case GPON_DATA_WAIT:		return "WAIT";
	case GPON_DATA_KEEP:		return "KEEP";
	case GPON_DATA_INSTALL:		return "INSTALL";
	case GPON_DATA_REPLACE:		return "REPLACE";
	case GPON_DATA_TEARDOWN:	return "TEARDOWN";
	}
	return "?";
}

/*
 * Which parts of a teardown apply to the identity that is actually armed.
 *
 * ★ BOTH NON-TRIVIAL FIELDS ARE ONE INVARIANT: THE OMCC'S T-CONT IS SACRED.
 *   The core already states it declaratively as GPON_GEM_US_BIND_IS_OMCC; the
 *   Cortina shell restates it twice imperatively, once per teardown step.  It
 *   is a protocol fact, not a Cortina one: on a single-alloc OLT the OMCC's
 *   T-CONT carries OMCI and PLOAM, so disabling its queues or invalidating its
 *   CAM entry to tear a data path down takes the management channel with it —
 *   and an ONU that cannot answer an OMCI audit gets DEACTIVATED.
 *
 * ★ THERE IS DELIBERATELY NO FIELD FOR THE US PORT-STAMP CLEAR.  It is
 *   unconditional in the shell, in ride mode too — clearing the stamps is
 *   precisely what stops the data GEM riding — and an always-true field is a
 *   check that cannot fail, which is a named recurring defect in this project.
 */
struct gpon_data_undo {
	bool	drain_tcont;	/* disable + flush the DATA T-CONT's VoQs first */
	bool	unbind_gem;	/* invalidate the unicast DS GEM CAM entry */
	bool	unbind_alloc;	/* invalidate the data T-CONT CAM entry */
};

/**
 * gpon_data_undo_plan - which teardown steps apply to what is armed
 * @armed:      what the shell put in silicon
 * @omcc_alloc: the Alloc-ID currently bound to the OMCC's own T-CONT
 * @out:        filled in; never read
 *
 * It says WHICH steps apply.  It says nothing about their ORDER, because the
 * order is a hardware fact that differs per family (drain before CAM) and
 * gpon_gem_us.c:196-200 already forbids this tier from owning it.
 *
 * Pure: no state, no side effect, safe from any context including softirq.
 */
static inline void gpon_data_undo_plan(const struct gpon_data_armed *armed,
				       u16 omcc_alloc,
				       struct gpon_data_undo *out)
{
	/* never disable the OMCC T-CONT's queues to tear a rider down */
	out->drain_tcont  = !armed->rides_omcc;
	out->unbind_gem   = armed->gem != 0;
	/* never invalidate the OMCC's own CAM entry */
	out->unbind_alloc = armed->alloc && armed->alloc != omcc_alloc;
}

#endif /* GPON_DATA_PLAN_H */
