/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: Upstream GEM and T-CONT mapping — interface.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 *
 * gpon_gem_us.h — the UPSTREAM half of GEM provisioning: the two mappings a
 * G.984 ONU must get right before it may transmit, common to every OpenWrt
 * GPON target in this tree.
 *
 * WHAT THIS IS
 *   Two mappings, and nothing else:
 *     1. Alloc-ID -> T-CONT.  Whether an OLT-assigned Alloc-ID may be bound to
 *        the ONU's data T-CONT, or must be left alone because it is already
 *        the OMCC's.  Both targets implemented this predicate independently and
 *        both produced a real outage from it (see gpon_gem_us_tcont_decide()).
 *     2. GEM Port-ID -> the upstream index slots that stamp it.  Which entries
 *        of the per-chip upstream port-map array carry a given GEM Port-ID, as
 *        a DECLARED range the shell supplies — never a computed one.
 *
 *   It is a decision layer, not a lifecycle layer.  It holds no state: the
 *   provisioning shadow (has the OLT sent ME 262 and ME 268 yet, what is armed
 *   in silicon, what survives a LOS) belongs to the GEM provisioning lifecycle
 *   and to each target's shell.  Everything here is a pure function of its
 *   arguments, so the same call with the same inputs is the same answer on any
 *   target, at any time, in any context.
 *
 * ★ WHAT IS DELIBERATELY *NOT* HERE — GEM header build, fragmentation, HEC
 *   The refactor brief asked this file to also carry "header build,
 *   fragmentation".  It does not, because on both shipping targets there is no
 *   such software to carry: the GEM Transmission Convergence layer is silicon.
 *   Measured 2026-08-05, and stated here so nobody spends a day looking for it:
 *     - realtek-elnath: the MAC encapsulates.  cortina-gpon.c:2390 — the 48-byte
 *       OMCI PDU "goes out the NI DMA-LSO ring; the HW GEM-encapsulates it onto
 *       the OMCC upstream on the next matching BWmap grant".  Upstream
 *       fragmentation is a PUC scheduler register field, not code:
 *       cortina-gpon.c:1362-1369 programs BTCCFG schmode=FRAGMENT(0) and
 *       lrgfrmfragen=1.  A search of that file for PLI / GEM header / fragment /
 *       reassembly framing returns only hardware counters
 *       (CG_REG_GEM_FRAG_DROP 0x1ac, CG_REG_DS_ASMBL_DROP 0x1cc) — i.e. reports
 *       from the MAC's own framer and reassembler.
 *     - realtek-luna: the GTC encapsulates.  The driver writes the port-map
 *       stamp (gpon-luna.c:5909, :5644) and the reassembly timer / PTI mask
 *       (:5449, :5478) and the engine does the rest.  It builds no GEM header.
 *   ⇒ There is nothing to hoist.  The INDEPENDENT model of what that silicon
 *   does lives in dev/rtl9607c-oracle/ (gem_tc.c, gem_us_encap.c) and MUST stay
 *   there: it is the differential reference this layer is tested against, and a
 *   reference merged into the thing it checks can no longer fail.
 *   A future target whose MAC does less — the ARM OLT on the roadmap, another
 *   brand — would need a real software framer.  It belongs beside this file as
 *   its own gpon_gem_tc.c when a target actually needs it, written against the
 *   oracle, and NOT invented speculatively here.
 *
 * WHY IT IS COMMON — which targets and which architectures compile it
 *   Operator, 2026-08-05: "en openwrt debería estar estructurado algo así:
 *   rtl960x* para la familia para tener código común" … "la idea es poner en
 *   común el código que corresponde para no tener mucho duplicado", and on the
 *   two per-target monoliths each carrying their own copy: "mal, poner en
 *   común".
 *   Compiled by:
 *     - realtek-elnath (RTL9607F, Cortina)  aarch64, LITTLE-endian
 *     - realtek-luna   (RTL960xC, Luna)     MIPS32,  BIG-endian
 *     - dev/rtl9607c-test on x86-64 through fuzz_shims/, where it is driven
 *       offline against the oracle with no board in the loop
 *   The prefix is gpon_ and not luna_ on purpose: the layer must also serve
 *   the future ARM OLT and other brands, so a Realtek-named prefix would be too
 *   narrow for what it covers.
 *
 * THE CORE/SHELL RULE IT OBEYS
 *   FUNCTIONAL CORE.  It decides; it never does.  No MMIO, no readl/writel, no
 *   ioremap, no device pointer, no allocation, no lock, no sleeping, no clock
 *   read.  The shell owns every register transaction, every completion poll and
 *   the order they run in — the upstream teardown order in particular is a
 *   hardware fact that differs per family and must never migrate here.
 *   ★ THIS FILE MUST NEVER GAIN AN MMIO ACCESS.  The moment it does, it stops
 *   being drivable from the x86 suite, the offline adversarial gate loses this
 *   area, and the only remaining way to check an upstream mapping is a ~200 s
 *   board boot.  A guard scans every gpon_*.c/h for exactly that.
 *
 *   Endianness: this file parses no wire bytes — the Alloc-ID and the GEM
 *   Port-ID arrive already extracted by the PLOAM/OMCI layers.  So the
 *   explicit-byte-math rule binds it vacuously, and it is NOT evidence that the
 *   rule is being honoured.  What the executed big-endian run does prove is
 *   that the index arithmetic and the field masks below give identical answers
 *   on MIPS-BE and on x86 — which is the only endianness claim this file makes.
 * ⚠ PROVENANCE POINTERS BELOW NAME gpon-luna.c, WHICH WAS CALLED
 *   gpon-rtl9602c.c / gpon-rtl960x.c UNTIL THE 2026-08-29 RENAME.  The
 *   LINE NUMBERS ARE AS THEY WERE WHEN THE MOVE WAS RECORDED -- later
 *   edits moved them, and they are kept because a provenance note is a
 *   dated fact about where code CAME FROM, not a pointer to today.
 *   Renaming without saying so would turn a dated record into a claim
 *   about the current file, which is the 'wrong in a new way' that
 *   citation_guard warns a bare sed produces.
 */
#ifndef GPON_GEM_US_H
#define GPON_GEM_US_H

#include <linux/types.h>

/*
 * ★ THREE DIFFERENT 12-BIT-ISH THINGS ARE ALL CALLED "gem" IN THIS TREE, AND
 * CONFUSING TWO OF THEM HAS ALREADY COST A BOARD-PROVEN REGRESSION.  They are
 * named apart here, once, so a later reader cannot unify them by accident:
 *
 *   GEM Port-ID   the number ON THE WIRE (G.984.3, 12 bits).  The OLT assigns
 *                 it; the OMCC's arrives in Configure_Port-ID, the data one in
 *                 OMCI ME 268.  This is the only one of the three that is a
 *                 protocol value, and the only one this file names "port_id".
 *   US index      the per-chip slot number in the upstream port-map array that
 *                 STAMPS a Port-ID onto a burst.  A silicon index, unrelated to
 *                 the wire value.  Called "index"/"range" here, never "gem".
 *   mcgid         (Cortina only) an egress LDPID port-group.  Not a GEM number
 *                 at all.  cortina-ni-flowoffload.c:2156-2163 records what
 *                 happened when it was set to a GEM Port-ID: "the old
 *                 mcgid = gem_id(223) was an aal-gen2/gemMapMode misread and
 *                 steered the hit frame to the WRONG egress -> far-end received
 *                 nothing."  It stays entirely inside the NE and appears
 *                 nowhere in this file.
 */

/* G.984.3 field widths.  Both are SPEC facts (Alloc-ID and GEM Port-ID are each
 * 12 bits, 0..4095), which is why they may live in common code — and both
 * targets already mask with exactly these:  Elnath cortina-gpon.c:2198/:2205
 * (alloc) and :2216/:2071 (port-id); Luna gpon-luna.c:6193 (alloc) and
 * :5645/:5486 (port-id).
 * The T-CONT index width is NOT here: 5 bits is a CAM-index width that both
 * silicons happen to share, not something G.984.3 defines, so it stays a
 * per-chip fact in each shell (Luna GPON_GTC_DS_ALLOC_IND, Elnath
 * CG_TCONT_INDEX). */
#define GPON_GEM_US_PORT_MASK	0x0fffu
#define GPON_GEM_US_ALLOC_MASK	0x0fffu

/* The value written to an upstream port-map slot that must stamp nothing.
 * ★ Only one target ever writes it: Elnath clears all eight of the data
 * T-CONT's slots on teardown (cortina-gpon.c:2122-2130).  Luna has no upstream
 * unstamp at all — its re-arm is the gpon_data_installed flag
 * (gpon-luna.c:968), so a stale Port-ID stays in the map until the next
 * install overwrites it.  That asymmetry is REAL and is preserved: making Luna
 * clear its slot would change what the silicon holds after a Deactivate, which
 * is a behaviour change and not part of a code-motion refactor. */
#define GPON_GEM_US_PORT_NONE	0u

/*
 * The upstream index slots that carry ONE GEM Port-ID, DECLARED by the shell.
 *
 * ★ DECLARED, NEVER COMPUTED — this is the whole reason the struct exists.
 * The two families number their upstream slots on incompatible principles:
 *
 *   Elnath (Cortina)  the slot number IS the VoQ, so the data T-CONT's slots
 *                     are tcont * queues_per_tcont .. +7.  cortina-gpon.c:310-316
 *                     ("the vendor keeps the internal GEM index == VoQ for US
 *                     GEMs"), :321 CG_DATA_GEM_IDX = CG_DATA_TCONT_IDX * 8,
 *                     :567 CG_PUC_QUEUE_PER_TCONT = 8.
 *                     OMCC  = { .base = 0, .count = 8 }   (:304, :2067)
 *                     data  = { .base = 8, .count = 8 }   (:2210-2220)
 *   Luna (RTL960xC)   the slot is a fixed GTC flow/SID per role, with no
 *                     arithmetic relationship to any T-CONT.
 *                     OMCC  = { .base = 64, .count = 1 }  (:5287, :5486)
 *                     data  = { .base = 1,  .count = 1 }  (:5644,
 *                                                 rtl9602c_gpon_nic.h:59)
 *
 * So `index = tcont * 8` is TRUE on Cortina and FALSE on Luna, and any common
 * code that computes it is silently wrong on one of the two targets.  There is
 * deliberately no function here that derives a base from a T-CONT.
 *
 * @index_max is the last slot the chip's array actually has (Luna 127, Elnath
 * 255).  It is carried so the range can be checked against the ARRAY's extent
 * rather than against a maximum somebody remembered: a count, a maximum, a
 * stride and an index space are four different quantities, and an upstream
 * write past the end of the port-map array lands in an unrelated register and
 * is accepted without complaint.
 */
struct gpon_gem_us_range {
	u16	base;		/* first upstream slot that stamps the Port-ID */
	u16	count;		/* how many consecutive slots (>= 1)           */
	u16	index_max;	/* last slot the chip's port-map array has     */
};

/*
 * Compile-time range check.  Usable in static_assert() with literal per-chip
 * constants, so a bad declared range fails the BUILD and costs nothing at run
 * time — which is what keeps this a pure code-motion addition: it can change
 * no behaviour because it emits no code.  Luna already uses static_assert this
 * way for the port-map stride (gpon-luna.c:5698).
 */
#define GPON_GEM_US_RANGE_OK(base, count, index_max)			\
	((count) >= 1u && (u32)(base) + (u32)(count) - 1u <= (u32)(index_max))

/* Runtime form of the same predicate, for a shell that builds a range at probe
 * and for the offline suite.  Reports only; it never repairs a bad range. */
bool gpon_gem_us_range_ok(const struct gpon_gem_us_range *r);

/* The n-th upstream slot of @r.  Callers loop n = 0 .. r->count-1; the result
 * is only meaningful when gpon_gem_us_range_ok(r). */
static inline u16 gpon_gem_us_index(const struct gpon_gem_us_range *r,
				    unsigned int n)
{
	return (u16)(r->base + n);
}

/* Mask a GEM Port-ID to its 12 on-wire bits.  Both shells already do exactly
 * this before the register write; naming it keeps the wire field in one place. */
static inline u16 gpon_gem_us_port_id(u32 port_id)
{
	return (u16)(port_id & GPON_GEM_US_PORT_MASK);
}

/* Mask an Alloc-ID to its 12 on-wire bits. */
static inline u16 gpon_gem_us_alloc_id(u32 alloc)
{
	return (u16)(alloc & GPON_GEM_US_ALLOC_MASK);
}

/*
 * The verdict of the Alloc-ID -> T-CONT decision.  Four outcomes, because
 * "do not bind" is three different facts and collapsing them is how a shell
 * ends up unable to say WHY the data path never came up.
 */
enum gpon_gem_us_bind {
	/* Bind this Alloc-ID to the data T-CONT. */
	GPON_GEM_US_BIND_TCONT = 0,
	/* Already bound: nothing to do, and re-binding would be a needless
	 * register transaction on a healthy link. */
	GPON_GEM_US_BIND_DONE,
	/* This Alloc-ID is the OMCC's.  Binding it to the data T-CONT would
	 * MOVE the OMCC off its own T-CONT.  Never do it — see the .c. */
	GPON_GEM_US_BIND_IS_OMCC,
};

/*
 * ★★★ SINGLE-ALLOC OLTs: THE DATA RIDES THE OMCC T-CONT.
 *
 * BIND_IS_OMCC says what must NOT happen; it does not say what the data path
 * should do instead, and a shell that only WARNED then fell through was the
 * defect: it stamped the data GEM into the dedicated T-CONT's VoQs, raised
 * carrier and set data_installed -- while that T-CONT had no CAM entry, so no
 * grant ever drained it.  Every upstream frame (the DHCP DISCOVER first)
 * queued forever, carrier-on lied to udhcpc, and the latch blocked any retry.
 * A silent, permanent dead WAN on the WHOLE single-alloc OLT class, against a
 * bar that says "work with ANY stock OLT, unattended, for months".
 *
 * The recovery is 9602C-PROVEN on this same lab OLT class (30/30 soak,
 * 354/354 DNS): stamp the data GEM into the OMCC T-CONT's OWN upstream slots,
 * using the ones the US OMCI does not need, and steer data TX to that T-CONT.
 * The data then rides the management Alloc-ID's grants.
 *
 * ★ WHY OMCI STAYS SAFE, and it is the reason the split is at the TOP: the
 *   chip serves a T-CONT's queues by STRICT PRIORITY, so leaving OMCI on the
 *   HIGHEST slots means a saturated data flow can never starve a PLOAM/OMCI
 *   response.  Reversing the split would make an OMCI timeout a function of
 *   user traffic -- which is how an ONU gets deactivated under load.
 *
 * ★ THE GEOMETRY IS THE SHELL'S, THE SPLIT IS OURS.  How many upstream slots a
 *   T-CONT has is silicon; which of them the OMCI keeps is a protocol
 *   decision, and it belongs here so both families make it identically.
 */
#define GPON_GEM_US_OMCI_RESERVED_SLOTS	2	/* the top two: OMCI + PLOAM */

/*
 * Derive the sub-range of @omcc the data GEM may use.  -> false when the OMCC
 * run is too short to spare any slot, in which case the caller must NOT install
 * a data path at all rather than steal an OMCI slot.
 */
bool gpon_gem_us_ride_range(const struct gpon_gem_us_range *omcc,
			    struct gpon_gem_us_range *out);

/*
 * Decide whether @alloc may be bound to the data T-CONT.
 *
 * @alloc        the Alloc-ID the OLT provisioned for user data.
 * @omcc_alloc   the Alloc-ID the OMCC's own T-CONT is bound to, as the CALLER
 *               knows it.  It is an input and not something this layer derives,
 *               because the two targets pass genuinely different values today
 *               (see the .c) and a code-motion refactor may not change either.
 * @already_bound  the caller's own "this is already done" flag, likewise passed
 *               through unchanged.
 *
 * Pure: no state, no side effect, safe from any context including softirq.
 */
enum gpon_gem_us_bind gpon_gem_us_tcont_decide(u16 alloc, u16 omcc_alloc,
					       bool already_bound);

/* One-line name for a verdict, for logs and for test failure messages. */
const char *gpon_gem_us_bind_name(enum gpon_gem_us_bind v);

/*
 * ★★★ ONE PROTOCOL RULE THAT WAS WRITTEN THREE TIMES.
 *
 * G.984.3 Configure_Port-ID assigns the OMCC GEM port. What to do with it is
 * PROTOCOL, not silicon -- and on 2026-09-03 it existed in three places, none
 * of which could call another: inline in the core PLOAM FSM's
 * PLM_DS_CONFIG_PORT case, again in the Luna native FSM (the copy that
 * actually SHIPS on that family), and a third time in the Cortina shell.
 *
 * ⚠ THE COST IS MEASURED, NOT IMAGINED. As a ONE-SHOT guard -- "we already
 * installed the OMCC, do nothing" -- an OLT that moves the OMCC to another GEM
 * after O5 leaves the ONU bound to the old port. Downstream de-encap follows
 * the MAC's own re-latch, so DS keeps working while our upstream OMCI replies
 * ride a port the OLT no longer accepts: its audit times out, it DEACTIVATES
 * us, and a full re-range is the only way back. The Luna copy carried that
 * defect for weeks AFTER the other two were repaired, because nothing made
 * them one decision.
 */
enum gpon_omcc_action {
	GPON_OMCC_IGNORE,	/* enable=0: a Configure_Port-ID transient. Write
				 * NOTHING and keep the link -- the following
				 * enable=1 re-latch carries the final id */
	GPON_OMCC_INSTALL,	/* not bound yet -> bind @want_gem */
	GPON_OMCC_REBIND,	/* bound to a DIFFERENT gem -> move the TRANSPORT
				 * only. The responder session is NOT re-armed:
				 * an OMCC port move is a transport event, and
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
 *
 * Pure: no state, no side effect, safe from any context including softirq.
 */
enum gpon_omcc_action gpon_omcc_decide(bool port_en, u16 want_gem,
				       bool installed, u16 cur_gem);

/* One-line name for a verdict, for logs and for test failure messages. */
const char *gpon_omcc_action_name(enum gpon_omcc_action a);

#endif /* GPON_GEM_US_H */
