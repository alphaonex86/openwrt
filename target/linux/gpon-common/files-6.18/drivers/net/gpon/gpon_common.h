/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gpon_common.h — THE CONTRACT between the HW-decoupled GPON protocol core and
 * the per-target imperative shell: the canonical ONU state encoding, the
 * G.984/G.988 wire enumerations, the per-chip fact block, and the op table the
 * core reaches hardware through.  Built by realtek-luna (MIPS32 BE),
 * realtek-elnath (ARM64 LE) and by dev/rtl9607c-test on x86.
 *
 * ★★ THE THREE TIERS — WHICH FILE MAY HOLD WHAT.  THIS IS THE CANONICAL COPY;
 * every other file carries a one-line "TIER:" header pointing here.
 *
 *   core    gpon_                     logic no single family owns: the PLOAM
 *                                     FSM, G.988 OMCI + ME model, GEM/T-CONT
 *                                     mapping, wire codecs.  NEVER touches HW
 *   family  luna_ (MIPS)              one silicon family's registers, DMA,
 *           cortina- (ARM64 NE)       IRQ, PON-MAC, SerDes
 *   chip    rtl9602c_ rtl9607c_       exactly one part's registers and quirks
 *           rtl9607f_
 *
 * The core line is ONE question — does it name a register or reach a bus?  If
 * not it belongs here however much Linux it uses (widened by the operator
 * 2026-08-28, because the old "protocol only" rule kept 347 register-free lines
 * of cn_flow_replace forked per family).
 *
 * ⚠ THE STRICT SUBSET stays free of Linux APIs too, because it is what fuzzes
 * on x86 at thousands of cases/s.  ITS MEMBERSHIP IS NOT RESTATED HERE — it is
 * STRICT_SUBSET in dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step
 * 17), and the copy that used to sit here had drifted to SIX files against that
 * array's ELEVEN (measured 2026-09-11): a list duplicated into a comment is a
 * list that drifts.  That gate COMPILES them against stubs declaring no
 * accessor, clock, lock or allocator — so impurity cannot build — and FAILS on
 * zero sources found, so "pure" and "absent" cannot look alike.  Core code that
 * needs Linux is a NEW file beside them, never an edit to one of those.
 * ⚠ A TOKEN GREP IS THE WRONG INSTRUMENT and this banner is the proof: it NAMES
 *   every forbidden token, so a raw scan scores 5 on this file and 0 on its code
 *   (2026-08-05, same pattern after gcc -fpreprocessed -dD -E).
 * ⚠ `gpon_core_purity_test` is named by gpon-common/build_wiring_guard.py:24 and
 *   build_wiring_x86_check.sh:18 and DOES NOT EXIST (2026-08-05).  It is owed
 *   work, not coverage; the compile guard above is the coverage.
 *
 * CORE decides and never performs I/O; SHELL does, and calls the core through
 * the table below.  Three rules make the core replayable and fuzzable:
 *   1. a side effect leaves as a RETURN VALUE wherever it can — an op exists
 *      only for what a return value cannot express;
 *   2. TIME IS AN INPUT: every entry point takes an explicit ms/tick argument;
 *   3. per-chip facts arrive as `const struct gpon_chip_cfg *`, never #ifdef.
 *
 * ENDIANNESS: multi-octet wire fields are assembled by explicit byte math
 * ((d[1]<<8)|d[2]).  Never a packed struct, a cast over wire bytes, or htons.
 *
 * ⚠ The `luna_gpon.c:NNN` pointers below were re-found BY TEXT at the 2026-09-10
 *   rename (citation_repair.py), so they name today's content, not their era.
 */
#ifndef GPON_COMMON_H
#define GPON_COMMON_H

#include <linux/types.h>

/* `fallthrough` for the HOST builds only; in kernel context
 * compiler_attributes.h already defines it and this #ifndef is false.
 * ★ A COMMENT WILL NOT DO: both targets build -Werror -Wimplicit-fallthrough=5
 *   (measured 2026-08-05 from scripts/mod/.empty.o.cmd), and at level 5 only the
 *   ATTRIBUTE suppresses the diagnostic — no comment spelling ever does. */
#ifndef fallthrough
# if defined(__GNUC__) && __GNUC__ >= 7
#  define fallthrough	__attribute__((__fallthrough__))
# else
#  define fallthrough	do {} while (0)
# endif
#endif

/* Broadcast/unassigned ONU-ID: G.984.3 says the ONU carries 0xff until
 * Assign_ONU-ID lands, and every re-range path restores it. */
#define GPON_ONU_ID_BROADCAST	0xffu

/* The G.984 broadcast GEM port-id.  Both targets already spell this number
 * (luna GPON_MCAST_GEM 0xfff, elnath CG_MCAST_GEM_ID 4095), so it is one spec
 * value written twice and not a per-chip difference. */
#define GPON_MCAST_GEM_PORT	4095u

/* Octets of an upstream PLOAM handed to ->ploam_tx().  G.984.3 Table 9-3:
 * ONU-ID + Message-ID + 10 message octets; the 13th octet (CRC) is appended by
 * SILICON and is therefore NOT part of this buffer. */
#define GPON_PLOAM_US_LEN	12u

/* Canonical ONU activation state, G.984.3 clause 10 names, 1-BASED.
 * ★ THE TWO SILICONS DISAGREE AND NEITHER DRIVER IS WRONG: luna's HW field is
 *   1-based and matches directly, elnath's is 0-based (CG_STATE_RANGING 3,
 *   CG_STATE_OPERATION 4), so each shell maps at the boundary.
 * ⚠ Do NOT adopt the host oracle's enum (dev/rtl9607c-oracle/ploam_fsm.h:22):
 *   its names are SHIFTED — it calls state 1 "STANDBY" when O1 is Initial.
 * ⚠ Mapping to this enum must not change either target's LOG STRINGS. */
enum gpon_ostate {
	GPON_O1_INITIAL		= 1,	/* no downstream sync                */
	GPON_O2_STANDBY		= 2,	/* DS sync, awaiting parameters      */
	GPON_O3_SERIAL		= 3,	/* serial-number acquisition         */
	GPON_O4_RANGING		= 4,	/* ranging (EqD assignment)          */
	GPON_O5_OPERATION	= 5,	/* operational                       */
	GPON_O6_POPUP		= 6,	/* intermittent LOS/LOF, popup       */
	GPON_O7_EMERGENCY	= 7,	/* emergency stop (SN disabled)      */
};

/* ME 268 (GEM port network CTP) "direction", as it arrives in a Set-by-Create.
 * ★ G.988 clause 9.2.3 is the authority and 00-COMMON-LAYER-PLAN.md §3 has US
 *   and DS SWAPPED — a wrong name in the CONTRACT is the class of defect this
 *   project has already paid for twice (`mcgid`; mt_name[5]="Delete"). */
enum gpon_gem_dir {
	GPON_GEM_US	= 1,	/* upstream only   (UNI -> ANI) */
	GPON_GEM_DS	= 2,	/* downstream only (ANI -> UNI) */
	GPON_GEM_BIDIR	= 3,	/* bidirectional — THE data GEM */
};

/* Which upstream PLOAM queue a decision wants its message on: an ABSTRACT
 * urgency class, never a register value.  The RTL9602C mapping (US_PLOAM_IND
 * [10:8]) is a chip fact and stays in the shell.
 * ⚠ The plan's ploam_tx() drops this argument; dropping it would put every US
 *   PLOAM on one queue and CHANGE LUNA'S BEHAVIOUR, so it is carried. */
enum gpon_ploam_q {
	GPON_PLOAM_Q_SN,	/* Serial_Number_ONU: the auto-SN slot        */
	GPON_PLOAM_Q_URGENT,	/* Acknowledge, Password, Encryption_Key      */
	GPON_PLOAM_Q_NOMSG,	/* the idle No_message keepalive slot         */
};

/* Per-chip facts.  A member belongs here only if BOTH hold: the two silicons
 * are MEASURED to differ on it, AND the core must know it (the shell cannot
 * just pass it as a call argument).  Test two is what stops this becoming a
 * dumping ground — data_gem_port, gem_flow_base and the T-CONT indices are all
 * arguments or shell tokens, not core facts.
 * ⚠ NOTHING READS THIS STRUCT YET and setting it today changes nothing: the
 *   common responder hardcodes the conformant values because plan D-3 keeps
 *   realtek-luna on its own responder.  Do not add a member without the code
 *   that reads it — a knob that silently does nothing is this project's most
 *   expensive recurring defect, and the `mic` member proved it. */
struct gpon_chip_cfg {
	/* F2 — one-past-the-end of the GET response value area.  36 is
	 * conformant (G.988 always reserves 36-37 and 38-39); realtek-luna
	 * writes to 40 (rtl9602c_eth.c:2265), overwriting both masks. */
	u16			get_val_end;
	/* F1 — the Get-Next message type.  0x1a is conformant; realtek-luna
	 * uses 0x10 (rtl9602c_eth.c:1698), which is the ALARM opcode, so its
	 * Get-Next falls through to NOT_SUPPORTED.  Unfalsifiable on the wire:
	 * only a build-time constant extractor catches it. */
	u8			mt_get_next;
};

/* ANYTHING THE CORE DID NOT UNDERSTAND — two classes, and collapsing them
 * would make every foreign OLT look like a broken device (operator, 2026-08-20):
 *   UNKNOWN  we do not model this value.  NOT a fault — it is the new-OLT
 *            support work list, and it is what another vendor's OLT produces.
 *   RANGE    outside the DECLARED domain (a grant naming a T-CONT we never
 *            configured).  That IS a defect or corruption, and it is a finding.
 * The worked example is the X111W alloc-CAM wall (2026-08-20): the driver was
 * granted T-CONTs it had never configured, said nothing, and the symptom
 * surfaced weeks later at the OLT as LOAi, three layers from the cause.
 * The emitting macro lives in gpon_unsup.h, NOT here: this header is CORE and
 * may never gain a pr_*. */
enum gpon_unsup_class {
	GPON_UNSUP_UNKNOWN = 0,
	GPON_UNSUP_RANGE   = 1,
};

/* The op signature, named ONCE: `unsupported` is declared in BOTH op tables
 * (here and struct gpon_ploam_ops), and a member spelled out twice is a member
 * that drifts in one of the two places. */
typedef void (*gpon_unsup_fn)(void *sh, const char *kind, int cls, u32 val,
			      const char *want, const u8 *data,
			      unsigned int len);

/* The NULL-safe caller every core uses: the guard lives in ONE place so no call
 * site can forget it, and "the shell does not implement this" stays legal. */
static inline void gpon_unsup_call(gpon_unsup_fn fn, void *sh, const char *kind,
				   enum gpon_unsup_class cls, u32 val,
				   const char *want, const u8 *data,
				   unsigned int len)
{
	if (fn)
		fn(sh, kind, (int)cls, val, want, data, len);
}

/* CALL CONTEXT — two classes, and mixing them is the largest correctness risk
 * in the carve: *_decide() entry points may run in SOFTIRQ and must not sleep
 * or lock; *_apply() run in PROCESS context only and are the only callers of an
 * op that may block, poll or time out.  ⇒ a mutex in common state would break
 * the softirq caller and a spinlock would be a behaviour change; neither is
 * done.
 *
 * ORDERING is a hardware fact the core may REQUEST but never PERFORM: elnath
 * must drain the VoQ before the CAM moves (cortina-gpon.c:2113), luna differs.
 *
 * ERRORS: an int-returning op returns 0 or a negative errno, and -ETIMEDOUT is
 * REACHABLE — every elnath table write polls a bounded completion
 * (cortina-gpon.c:1977).  A core that assumes an install succeeded leaves
 * silicon carrying a binding its shadow does not record.
 *
 * A NULL op means "this target has no such hardware step" and the core SKIPS
 * it rather than failing: luna has no downstream-GEM CAM teardown at all.
 *
 * @sh is the shell's opaque handle, void * so no target's device struct can
 * reach this header. */
struct gpon_shell_ops {
	/* --- PLOAM activation.  One consumer today (realtek-luna): elnath has
	 * NO software PLOAM FSM, its MAC runs O1->O5 in silicon.  These ops are
	 * justified by offline testability and the roadmap, NOT by de-duplication.
	 */

	/* Queue one upstream PLOAM; silicon appends the CRC.
	 * <- luna_gpon.c:5704 gpon_send_cpu_ploam() */
	void (*ploam_tx)(void *sh, enum gpon_ploam_q q,
			 const u8 msg[GPON_PLOAM_US_LEN]);

	/* Publish the activation state into the GTC status field.
	 * <- luna_gpon.c:6638 gpon_fsm_set_state() */
	void (*set_hw_state)(void *sh, enum gpon_ostate st);

	/* Publish the OLT-assigned ONU-ID (GPON_ONU_ID_BROADCAST on every
	 * teardown).  ONE op and not two register writes because luna must
	 * write BOTH the downstream and the upstream copy.
	 * <- luna_gpon.c:6805-6236 and the four teardown paths */
	void (*set_hw_onu_id)(void *sh, u8 onu_id);

	/* Program the upstream equalization delay, in bits, as Ranging_Time
	 * gave it.  <- luna_gpon.c:7010 gpon_set_eqd()
	 * ⚠ COARSER THAN THE PLAN on purpose: the EqD arithmetic is three CHIP
	 *   facts to one protocol fact (MIN_DELAY1, the 128-bit unit, the
	 *   [26:24]+[17:0] packing), so it stays shell-side and `get_min_delay`
	 *   is NOT declared — an op with no caller cannot be verified. */
	void (*set_eqd)(void *sh, u32 eqd_bits);

	/* Apply the burst-overhead parameters the core computed, pre-ranged or
	 * ranged.  <- luna_gpon.c:6940 gpon_apply_boh() */
	void (*apply_boh)(void *sh, bool ranged);

	/* Re-lock the transmit analog at O3 entry: the cold-start TX-CMU/PLL
	 * relock without which a cold boot never ranges while a warm reboot
	 * does.  <- luna_gpon.c:6623 gpon_txpll_relock()
	 * ⚠ void, not the plan's int: BOTH implementations return void, and an
	 *   int nobody sets is a value the core would branch on. */
	void (*analog_relock)(void *sh);

	/* Load a 128-bit AES key into the STAGED bank; the hardware promotes
	 * staged->active at the switch time below.
	 * <- luna_gpon.c:5843 gpon_aes_stage_key() */
	void (*aes_stage_key)(void *sh, const u8 key[16]);

	/* Arm the superframe at which the staged key is promoted, from
	 * Key_Switching_Time.  Without it an un-armed switch corrupts AES.
	 * <- luna_gpon.c:7551 */
	void (*aes_set_switch_time)(void *sh, u32 superframe);

	/* Fill @out with @len random octets.  INJECTED rather than called so
	 * the core stays deterministic under replay and fuzzing — which is why
	 * the key path is testable offline at all. */
	void (*rng)(void *sh, u8 *out, unsigned int len);

	/* --- GEM / T-CONT.  THE GRANULARITY IS "INSTALL", NEVER "WRITE THE CAM
	 * WORD": elnath's data install is ONE transaction with seven early
	 * returns over the T-CONT CAM, eight US port stamps, two DS CAM entries,
	 * two PDC map writes, a PUC enable and a VoQ flush
	 * (cortina-gpon.c:2171-2287) — decomposing it WOULD reorder it.  The
	 * core names WIRE IDENTITIES (alloc-id, GEM port-id); the shell owns
	 * every silicon index.  @gem is never an internal GEM index and never an
	 * `mcgid`, a third unrelated 12-bit field that already cost one
	 * board-proven regression.
	 */

	/* Bind the OMCC: the management alloc-id and its GEM port-id.
	 * <- luna_gpon.c:5662 gpon_install_omcc()
	 *    cortina-gpon.c:2027 cg_omcc_tcont_bind() */
	int (*omcc_install)(void *sh, u16 alloc, u16 gem);

	/* Bind the user-data path, as snooped from ME 262 / ME 268.
	 * <- luna_gpon.c:6602 gpon_install_data_gem()
	 *    cortina-gpon.c:2171 cg_data_try_install() */
	int (*data_install)(void *sh, u16 alloc, u16 gem);

	/* Tear the user-data path down in whatever order that silicon requires.
	 * NULL on realtek-luna.  <- cortina-gpon.c:2113 cg_data_teardown() */
	void (*data_teardown)(void *sh);

	/* Transmit one OMCI PDU upstream.  @len is carried even though baseline
	 * OMCI is always OMCI_LEN: an implicit length is what makes a buffer
	 * unfuzzable.  <- cortina-gpon.c:2393 cg_omci_tx() */
	int (*omci_tx)(void *sh, const u8 *pdu, unsigned int len);

	/* --- diagnostics: NEVER load-bearing.  A NULL ->trace must change no
	 * decision and no emitted byte.
	 * ⚠ It is NOT how a target's existing log lines are preserved: those
	 *   carry register reads inside their argument lists, so they stay in
	 *   the shell and must be replayed there in the same order. */
	void (*trace)(void *sh, unsigned int ev, u32 a, u32 b);

	/* Report ONE datum the core received and could not place.  Same
	 * diagnostic contract as ->trace, but it carries a BUFFER, which ->trace
	 * cannot: two u32s can say an unhandled type went by and cannot say WHAT
	 * went by, and a report WITH the datum is the specification for
	 * implementing it — which is how a foreign OLT gets supported.
	 * @kind  a stable slug; it is the rate-limit key in the shell AND the
	 *        aggregation key in the reader, so a new kind is never buried.
	 * @cls   enum gpon_unsup_class, widened to int so this table needs no
	 *        ordering dependency on that enum.
	 * @want  the DECLARED domain, as a SPACE-FREE token: the reader parses
	 *        `want=` up to the first space and silently loses the rest.
	 * @data  the minimal dump, @len octets; the shell clips it.  NULL/0 is
	 *        legal — a report with no dump is still a report. */
	gpon_unsup_fn unsupported;
};

/* Each core owns a CONTEXT STRUCT carrying { ops, sh, chip } plus its own
 * state, and takes a pointer to it on every entry point.  NEVER file-scope
 * globals: the abandoned gpon_proto.c kept its FSM state in ~11 externs, and
 * that is exactly what made it impossible to instantiate twice and run against
 * the host oracle for a differential. */
struct gpon_core_base {
	const struct gpon_shell_ops	*ops;
	void				*sh;	/* opaque shell handle       */
	const struct gpon_chip_cfg	*chip;
};

#endif /* GPON_COMMON_H */
