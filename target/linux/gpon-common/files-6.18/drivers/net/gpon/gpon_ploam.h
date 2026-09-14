/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE (prefix gpon_) — decides, never touches hardware, and compiles for
 * MIPS-BE, ARM64-LE and x86.  Canonical tier rule and guard: see "THE THREE
 * TIERS" in gpon_common.h (this directory).
 *
 * gpon_ploam.h — the G.984.3 PLOAM activation state machine (O1..O7): downstream
 * message dispatch, the O1->O5 transitions, the upstream message builders, the
 * burst-overhead and equalization-delay computations, and the periodic-poll
 * decisions (SN re-offer, O5 keepalive, the DS-LOS re-range debounce, the O5
 * provisioning watchdog).  Pure byte math over the 13-byte wire message and
 * pure arithmetic over counters; every SIDE EFFECT leaves through
 * struct gpon_ploam_ops, and TIME IS AN EXPLICIT INPUT (now_ms).
 *
 * ★ HONEST SCOPE, MEASURED 2026-08-05: this file has exactly ONE kernel
 *   consumer.  Elnath has NO software PLOAM FSM — its MAC runs O1->O5 and
 *   auto-ACKs in silicon — so this is NOT justified by de-duplication.  It is
 *   justified by offline adversarial testing (the FSM becomes fuzzable on x86
 *   against an adversarial OLT with no board in the loop, which is this
 *   project's PRIMARY correctness gate) and by the roadmap.  Claiming a saving
 *   that does not exist would be the reassuring-direction reporting this
 *   project keeps paying for.
 *
 * PROVENANCE: code motion out of the Luna monolith, which is at stock parity.
 * Behaviour is intended to be BIT-IDENTICAL; a latent defect found while moving
 * is moved UNCHANGED and recorded as a follow-up at that site.
 */
#ifndef GPON_PLOAM_H
#define GPON_PLOAM_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/*
 * ★ TEMPORARY COMPATIBILITY SHIM — IT BELONGS IN gpon_common.h, NOT HERE, and
 * it exists because that header includes <linux/types.h> unconditionally, which
 * makes the common layer kernel-only.
 *
 * ★ AND IT FAILS IN THE REASSURING DIRECTION, which is why it survived: on a
 *   glibc host <linux/types.h> RESOLVES — to the UAPI copy — so the include
 *   succeeds and the file looks portable.  That header defines __u8/__u32 but
 *   NOT the kernel-internal u8/u16/u32, so the build dies far below on the
 *   first struct member and reads like a typo rather than a missing host
 *   branch.  Delete this block the moment gpon_common.h grows its own
 *   #ifndef __KERNEL__ branch; a duplicate typedef of the identical type is
 *   legal C11, so the two can coexist meanwhile.
 */
#ifndef GPON_HOST_TYPES_DEFINED
#define GPON_HOST_TYPES_DEFINED
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
#endif
#endif /* !__KERNEL__ */

#include "gpon_common.h"	/* enum gpon_ostate — the canonical G.984.3 states */
#include "gpon_data_plan.h"	/* allocation domain shared with reconciliation */

/* G.984.3 §9.2.3 PLOAM message types and US queue selectors — wire FACTS, not
 * vendor expression. */
#define PLM_DS_UPSTREAM_OVERHEAD	0x01

/* The broadcast/unassigned ONU-ID: G.984.3 reserves 0xFF for a message the OLT
 * addresses to every ONU, and it is what an ONU carries until it is assigned
 * one.  A PROTOCOL constant, so neither family should spell the literal. */
#define GPON_PLOAM_ONU_ID_BROADCAST	0xffu

#define PLM_DS_ASSIGN_ONU_ID		0x03
#define PLM_DS_RANGING_TIME		0x04
#define PLM_DS_DEACTIVATE_ONU		0x05
#define PLM_DS_DISABLE_SN		0x06
#define PLM_DS_EXT_BURST_LENGTH		0x14
#define PLM_DS_ENCRYPT_PORT		0x08	/* Encrypted_Port-ID (ACK) */
#define PLM_DS_REQUEST_KEY		0x0d	/* Request_key (-> Encryption_Key) */
#define PLM_DS_KEY_SWITCH		0x13	/* Key_Switching_Time (-> arm HW key switch) */
#define PLM_DS_ASSIGN_ALLOC_ID		0x0a	/* Assign_Alloc-ID (ACK) */
#define PLM_DS_REQUEST_PASSWORD		0x09	/* Request_Password (-> US Password 0x02) */
#define PLM_DS_CONFIG_PORT		0x0e	/* Configure_Port-ID (ACK) */
#define PLM_DS_CFG_VPVC			0x07	/* Configure_VP/VC (ATM, unsupported; ACK) */
#define PLM_DS_BER_INTERVAL		0x12	/* BER interval (ACK; arms BER reporting) */
#define PLM_US_ENCRYPT_KEY		0x05	/* US Encryption_Key response */
#define PLM_US_PASSWORD			0x02	/* US Password response (to Request_Password) */
#define PLM_US_SERIAL_NUMBER		0x01
#define PLM_US_ACKNOWLEDGE		0x09	/* US Acknowledge message type */

/* Was a bare 0x04 literal in the Luna driver, commented "US_NOMESSAGE". */
#define PLM_US_NO_MESSAGE		0x04

#define PLM_US_QUEUE_SN			0x6	/* US_PLOAM_IND[10:8] auto-SN queue */
#define PLM_US_QUEUE_URG		0x1	/* US_PLOAM_IND[10:8] urgent queue (ACKs) */
#define PLM_US_QUEUE_NOMSG		0x7	/* US_PLOAM_IND[10:8] HW auto-No_message slot */

/* A DS PLOAM is 13 bytes ([0]=ONU-ID [1]=type [2..11]=data [12]=CRC-8); a US
 * PLOAM is 12 and the hardware appends the CRC.  gpon_common.h owns these — it
 * is the contract — so they are GUARDED rather than duplicated: the two headers
 * agree whichever is included first, and a -Werror build is not broken by a
 * redefinition warning.  Same value either way. */
#ifndef GPON_PLOAM_DS_LEN
#define GPON_PLOAM_DS_LEN		13u
#endif
#ifndef GPON_PLOAM_US_LEN
#define GPON_PLOAM_US_LEN		12u
#endif

/* Burst-overhead geometry: 12 = TOTAL_OVERHEAD_BITS(96)/8 stored bytes, which
 * the hardware extends to BOH_LENGTH via the REPEAT pointer, and 252 is the
 * BOH_LENGTH field cap.  FOLLOW-UP: on a chip whose BOH_DATA array is not 12
 * deep these become per-chip config members; today every RTL960x agrees and no
 * second consumer exists to disagree. */
#define GPON_PLOAM_BOH_LEN		12
#define GPON_PLOAM_BOH_MAX_LEN		252

/* One upstream GTC frame in bits (G.984.3: 19440 octets at 1.24416 Gbit/s).  A
 * specification constant, not a register. */
#define GPON_PLOAM_EQD_FRAME_LEN	(19440 * 8)

/* Activation admission is shared by native and op-table FSMs. A failed
 * management T-CONT installation leaves O3 with the assigned ONU-ID already
 * stored, so EqD must not promote that incomplete assignment to O5. O5 EqD
 * adjustments remain legal. Serial matching stays with the caller. */
static inline bool gpon_ploam_assign_allowed(u8 state, u8 addressed_onu)
{
	return state == GPON_O3_SERIAL &&
	       addressed_onu == GPON_PLOAM_ONU_ID_BROADCAST;
}

static inline bool gpon_ploam_ranging_allowed(u8 state, u8 addressed_onu,
					     u8 own_onu, u8 path)
{
	return (state == GPON_O4_RANGING || state == GPON_O5_OPERATION) &&
	       addressed_onu == own_onu && !(path & 1);
}

/* Diagnostic events.  The core NEVER formats a string: it reports what happened
 * and the shell renders it with whatever counters it wants to read, which is
 * how the seven-counter DEACT line and its siblings keep their MMIO reads out
 * of this file.  ★ `trace` is NEVER load-bearing: a NULL op changes no decision,
 * and no core branch may depend on it. */
enum gpon_ploam_ev {
	GPON_PLOAM_EV_STATE,		/* a = previous O-state, b = new O-state   */
	GPON_PLOAM_EV_RERANGE_DONE,	/* a = re-range count, b = outage ms       */
	GPON_PLOAM_EV_BOH,		/* a = ranged?, b = BOH_CFG word           */
	GPON_PLOAM_EV_DS,		/* a = DS ONU-ID, b = DS type              */
	GPON_PLOAM_EV_O3_FEED_RESET,	/* -                                       */
	GPON_PLOAM_EV_ONU_ID,		/* a = assigned ONU-ID, b = T-CONT16 alloc */
	GPON_PLOAM_EV_CAM_READBACK,	/* a = T-CONT, b = wanted alloc            */
	GPON_PLOAM_EV_RANGING_TIME,	/* a = EqD                                 */
	GPON_PLOAM_EV_DISABLE_SN,	/* a = disable code d[0]                   */
	GPON_PLOAM_EV_DEACT,		/* a = our ONU-ID at the time              */
	GPON_PLOAM_EV_DEACT_KEEP_LOCK,	/* a = ticks held at O5 (reseat skipped)   */
	GPON_PLOAM_EV_EXT_BURST,	/* a = t3pre, b = t3ranged                 */
	GPON_PLOAM_EV_ACK,		/* a = acknowledged DS type                */
	GPON_PLOAM_EV_ASSIGN_ALLOC,	/* a = alloc, b = op code d[2]             */
	GPON_PLOAM_EV_DATA_TCONT,	/* a = alloc, b = data T-CONT              */
	GPON_PLOAM_EV_REQ_KEY,		/* -                                       */
	GPON_PLOAM_EV_KEY_SENT,		/* a = key index                           */
	GPON_PLOAM_EV_REQ_PW,		/* -                                       */
	GPON_PLOAM_EV_KEY_SWITCH_ARM,	/* a = superframe count armed              */
	GPON_PLOAM_EV_KEY_SWITCH,	/* a = key staged?, b = armed superframe   */
	GPON_PLOAM_EV_UNHANDLED,	/* a = DS type, b = DS ONU-ID              */
	GPON_PLOAM_EV_SN_REPROVISIONED,	/* -                                       */
	GPON_PLOAM_EV_O5_WATCHDOG,	/* a = ticks held at O5 with no WAN RX     */
	GPON_PLOAM_EV_O4_TIMEOUT,	/* a = ticks held at O4, no Ranging_Time   */
	GPON_PLOAM_EV_LOS_RERANGE,	/* a = consecutive LOS ticks               */
	/* ★★★ THE EARLY LADDER, AND IT ONLY REPORTS (operator, 2026-09-08).
	 * Every other activation event fires on a TRANSITION, so a board that
	 * never leaves O1 emits NOTHING -- which is exactly the G24W's stall and
	 * exactly why it could only be guessed at.  This one fires while the FSM
	 * SITS in O1/O2/O3, on a cadence, and changes no state.
	 * a = the O-state, b = ticks held there. */
	GPON_PLOAM_EV_EARLY_DWELL,
};

/* The imperative shell: every entry is a SIDE EFFECT the core decided on and
 * cannot express as a return value.
 *
 * ★ ORDER IS PART OF THE CONTRACT.  These are invoked at exactly the points the
 *   Luna driver invoked the register writes they replace: the burst-overhead
 *   write happens BEFORE the EqD write on the O3 edge, and the US-PLOAM flush
 *   happens AFTER the ranged BOH switch on the O4 edge.  A shell that reorders
 *   them changes behaviour.
 * ★ DELIBERATELY NOT struct gpon_shell_ops from the plan: that sketch drops the
 *   US_PLOAM_IND queue selector (the urgent queue pre-empts the auto-SN burst,
 *   which is what gets an Acknowledge to the OLT before it raises LOAi) and its
 *   set_eqd implies the core packs the EqD REGISTER word, whose field layout is
 *   a register fact.  Both would be wire or behaviour changes. */
struct gpon_ploam_ops {
	/* Enqueue one 12-byte US PLOAM on `queue` (PLM_US_QUEUE_*).  Replaces
	 * gpon_send_cpu_ploam(). */
	void (*ploam_tx)(void *sh, u8 queue, const u8 m[GPON_PLOAM_US_LEN]);
	/* Write BOH_CFG and the first `size` BOH_DATA bytes.  <- :6991 */
	void (*boh_write)(void *sh, u32 cfg_word, const u8 *oh, u8 size);
	/* The ONE hardware read taken mid-decision: US_MIN_DELAY[15:7].
	 * Injected so the EqD computation is reproducible offline.  <- :7012 */
	u32  (*get_min_delay)(void *sh);
	/* Write the equalization delay from its two computed halves. <- :7017 */
	void (*set_eqd)(void *sh, u32 multiframe, u32 intraframe);
	/* Flush the single shared CPU US-PLOAM TX buffer so no pre-ranged-format
	 * burst is latched when the first ranged grant fires.  <- :7353 */
	void (*us_ploam_flush)(void *sh);
	/* Reflect the canonical O-state into hardware and the PON LED; the shell
	 * MAPS it to its own encoding (Luna 1-based, Elnath 0-based). <- :7166 */
	void (*set_hw_state)(void *sh, enum gpon_ostate st);
	/* Write the ONU-ID into BOTH hardware fields (US_ONU_ID[15:8] and
	 * DS_ONU_STATUS[15:8]).  <- the gpon_field() pairs at :6235 and four
	 * teardown paths */
	void (*set_hw_onu_id)(void *sh, u8 onu_id);
	/* The FSM dropped below O5: the shell re-arms whatever it gates on O5
	 * (Luna: the VLAN filter).  <- :7154 */
	void (*on_below_o5)(void *sh);
	/* The FSM entered O5 and the burst-gate re-arm is enabled: re-apply the
	 * US packed-burst register cluster.  The core emits the No_message that
	 * follows it.  <- :7142 */
	void (*o5_rearm_burst)(void *sh);
	/* O3-entry TX-CMU/PLL re-lock, the cold-start fix.  <- :7264
	 * gpon_txpll_relock() */
	void (*analog_relock)(void *sh);
	/* The post-relock full GPON-datapath reset-B edge that un-parks the
	 * GEM-US feed.  <- :7270 */
	void (*o3_feed_reset)(void *sh);
	/* Re-seat the US-TX serializer on a re-range: the interface reset-B edge
	 * plus the deferred CDR-lock pulse.  <- :6359, :6665, :6714 */
	void (*cdr_reseat)(void *sh);
	/* Random bytes, injected so a differential/fuzz run is reproducible. */
	void (*rng)(void *sh, u8 *out, unsigned int len);
	/* Load the key into the hardware STAGED bank.  <- :5884 */
	void (*aes_stage_key)(void *sh, const u8 key[16]);
	/* Arm the key-switch superframe comparator.  <- :7551 */
	void (*aes_arm_switch)(void *sh, u32 superframe);
	/* --- GEM / T-CONT: the PLOAM layer decides WHICH identities to bind and
	 * the CAM/table work is reached through these.  Every one may fail (a
	 * table op can time out) and the core MUST honour it — a non-zero return
	 * leaves the installed flag CLEAR so the next event retries. */
	int  (*install_omcc)(void *sh, u16 gem);
	int  (*install_tcont)(void *sh, u8 tcont, u16 alloc);
	/* Optional ownership consumer. If present it records the PLOAM assignment
	 * independently of CAM completion, including deallocation after partial
	 * failure. NULL retains the immediate install_tcont path. */
	void (*data_alloc_changed)(void *sh, u16 alloc, bool assigned);
	/* @gem is the wire GEM Port-ID from the OLT's ME 268 Create.  It is NOT
	 * a per-board constant: this lab's OLT was measured handing out 223 to
	 * one board and 193 to another. */
	void (*install_data_gem)(void *sh, u16 gem);
	/* Report the WAN-egress (VEIP) operational state up to the OLT. */
	void (*omci_report_oper_up)(void *sh);
	/* --- diagnostics: NEVER load-bearing, NULL is always legal. */
	void (*trace)(void *sh, enum gpon_ploam_ev ev, u32 a, u32 b);
	/* ONE datum the FSM received and could not place.  Same contract as
	 * `trace`, but it carries a BUFFER, which `trace` cannot: two u32s can
	 * say an unhandled type went by and cannot say WHAT went by, and a
	 * 13-octet PLOAM does not fit in them.  Spelled through the typedef in
	 * gpon_common.h so this member and the one in struct gpon_shell_ops
	 * cannot drift before the two tables are unified. */
	gpon_unsup_fn unsupported;
};

/* ★★★ THE MANDATORY SET — C's answer to a C++ PURE VIRTUAL.  A designated
 * initialiser that omits a member is legal C and leaves it NULL, so a family
 * that forgets a callback gets a silent no-op at best and, at worst, a NULL
 * dereference the first time the FSM reaches that transition — minutes into a
 * boot, on a board with no console budget, reading like a silicon fault.
 *
 * A callback is MANDATORY when gpon_ploam.c calls it WITHOUT a NULL check on a
 * path that runs.  Everything else is OPTIONAL for a stated reason: ->trace is
 * guarded by ev(), and every ->unsupported call goes through gpon_unsup_call(),
 * whose whole purpose is that the NULL check lives in ONE place.
 *
 * ⚠ THE EVIDENCE IS THE CALL SITE, NOT THIS LIST, which can drift from the code
 *   it describes: dev/ONU-test-case/ploam_ops_mandatory_guard.py re-derives the
 *   split from gpon_ploam.c's own call sites and FAILS when it disagrees.  The
 *   :NNN pointers below are line numbers AS MEASURED ON 2026-09-09 — they say
 *   where the evidence WAS; the guard is what stays true. */
#define GPON_PLOAM_OPS_MANDATORY(X)					\
	X(ploam_tx)		/* gpon_ploam.c:195  */			\
	X(boh_write)		/* :395  */				\
	X(get_min_delay)	/* :413  */				\
	X(set_eqd)		/* :418  */				\
	X(us_ploam_flush)	/* :656  */				\
	X(set_hw_state)		/* :517  */				\
	X(set_hw_onu_id)	/* :612 :706 :977 :1088 :1108 :1168 */	\
	X(on_below_o5)		/* :511  */				\
	X(o5_rearm_burst)	/* :502  */				\
	X(analog_relock)	/* :593  */				\
	X(o3_feed_reset)	/* :599  */				\
	X(cdr_reseat)		/* :719 :1110 :1170 */			\
	X(rng)			/* :284  */				\
	X(aes_stage_key)	/* :297  */				\
	X(aes_arm_switch)	/* :862  */				\
	X(install_omcc)		/* :770  */				\
	X(install_tcont)	/* :621 :632 :803 */			\
	X(install_data_gem)	/* :1004 */				\
	X(omci_report_oper_up)	/* :1010 */

/* warn_unused_result, spelled locally so this header keeps building on all
 * three toolchains without <linux/compiler_attributes.h>, which the host stubs
 * do not provide.  It is the half of the mechanism the COMPILER enforces: the
 * target kernels build -Werror, so a family cannot quietly ignore the refusal. */
#ifndef GPON_MUST_CHECK
#  if defined(__GNUC__)
#    define GPON_MUST_CHECK __attribute__((warn_unused_result))
#  else
#    define GPON_MUST_CHECK
#  endif
#endif

/**
 * gpon_ploam_ops_missing() - is this table complete enough to drive the FSM?
 * @ops: the shell's table, or NULL.
 *
 * PURE.  Returns the name of the FIRST mandatory op this table leaves NULL, as
 * a static string the caller may print, or NULL when the table is complete.  A
 * NULL @ops answers "ops".  The NAME and not an errno, because the core may not
 * printk and `-EINVAL` would say something is wrong without saying which of
 * nineteen things.
 */
const char *gpon_ploam_ops_missing(const struct gpon_ploam_ops *ops);

/* Per-board / per-chip configuration, READ-ONLY on the core side.  The Luna
 * driver read ~12 module_param globals directly from FSM code; a common core
 * cannot own a module_param, so they became fields here.  To keep them
 * RUNTIME-WRITABLE exactly as before, a shell points module_param_named() AT
 * these members rather than mirroring them — the core reads through the pointer
 * on every use, so a live write takes effect on the next tick. */
struct gpon_ploam_cfg {
	/* Park the FSM at O1: refuse every advance so the GPON never ranges and
	 * the shared switch datapath stops churning. */
	bool hold;
	/* Per-PLOAM tracing.  Only ever gates a `trace` op call. */
	bool trace;
	/* NON-STOCK double-bind of the OMCC alloc to the alternate T-CONT. */
	bool omcc_alt_bind;
	/* Re-seat the US-TX serializer on a re-range. */
	bool cdr_reseat_on_reactivate;
	/* Install the WAN data GEM once the OLT has solicited it. */
	bool data_gem_en;
	/* Re-apply the O5 packed-burst register cluster on every O5 entry, not
	 * only at init. */
	bool o5_rearm_burst_gate;
	/* Take the full WSDS datapath reset-B edge after the O3 relock. */
	bool o3_feed_reset;
	/* OMCC Alloc-ID override; 0 (default) = bind the LIVE ONU-ID, which is
	 * the G.984.3 default and what stock does. */
	u16 omcc_alloc_override;
	/* Consecutive (optic_los && !sds_sdet) ticks before an autonomous
	 * downstream-LOS re-range.  0 disables. */
	unsigned int los_rerange_ticks;
	/* Re-range if held at O5 this many ticks with zero WAN RX.  0 disables,
	 * and that is the shipped default: proven ineffective. */
	unsigned int o5_provision_watchdog_ticks;
	/* Emit a No_message US PLOAM every N ticks at O5.  0 disables, and that
	 * is the shipped default: stock emits no unsolicited US PLOAM at O5. */
	unsigned int o5_ploam_keepalive_ticks;
	/* ★★ G.984.3's RANGING TIMER: leave O4 after this many ticks with no
	 * Ranging_Time.  0 disables.  NOTHING ELSE can take the FSM out of O4 —
	 * poll_sn_reoffer() only fires while onu_id is 0xff and the watchdog
	 * above only covers O5 — and an ONU stuck in O4 cannot be ranged by ANY
	 * OLT, so the far end's only remaining move is Deactivate.
	 * ⚠ MEASURED BOTH HALVES: the offline pairing holds the FSM in O4 for
	 *   20000 poll rounds and it never leaves (sim_ploam), and the X111W
	 *   bench shows exactly that shape -- O4, then DEACTIVATE, never O5. */
	unsigned int o4_ranging_timeout_ticks;
	/* How often to report a dwell at O1/O2/O3.  0 = silent, which is what a
	 * family that sets nothing gets. */
	unsigned int early_dwell_report_ticks;
	/* T-CONT indices this silicon uses. */
	u8 omcc_tcont;		/* Luna 16 */
	u8 omcc_tcont_alt;	/* Luna 1, used only when omcc_alt_bind */
	u8 data_tcont;		/* Luna 8  */
};

/* The FSM context.  ALL state lives here — there is not one file-scope variable
 * in gpon_ploam.c, so two instances can run side by side in one address space,
 * which is what makes a host-side differential against the oracle possible and
 * is precisely what the dead gpon_proto.c could not do.
 * The shell may read every field directly (/proc does); only the core writes. */
struct gpon_ploam {
	const struct gpon_ploam_ops *ops;
	const struct gpon_ploam_cfg *cfg;
	void *sh;			/* opaque shell handle passed to every op */
	/* --- identity ----------------------------------------------------- */
	u8 sn[8];			/* G.984.3 ONU-SN: 4-byte ID + 4-byte serial */
	u8 onu_id;			/* 0xff = unassigned                         */
	bool sn_changed;		/* SN was (re)provisioned -> must re-range   */
	/* --- activation state --------------------------------------------- */
	enum gpon_ostate state;
	u32 ticks;			/* FSM poll ticks (~10 ms each)              */
	u32 o5_entry_tick;		/* tick of the last O5 entry (0 = not at O5) */
	u32 o4_entry_tick;		/* tick of the last O4 entry (0 = not at O4) */
	u32 early_entry_tick;		/* tick this O1/O2/O3 dwell began (0 = not early) */
	u32 early_report_tick;		/* tick of the last dwell report (0 = none yet)   */
	int avc_sent;			/* oper-state AVCs emitted this O5           */
	u32 los_run;			/* consecutive real-LOS ticks                */
	/* --- burst overhead learned from the OLT -------------------------- */
	u8 boh_guard;			/* Upstream_Overhead d[0]   = guard bits        */
	u8 boh_ptn;			/* Upstream_Overhead d[3]   = Type-3 pattern    */
	u8 boh_delim[3];		/* Upstream_Overhead d[4..6]                    */
	u8 boh_t3pre;			/* Ext_Burst_Length d[0] = pre-ranged Type-3 len */
	u8 boh_t3ranged;		/* Ext_Burst_Length d[1] = ranged Type-3 len     */
	/* --- provisioning flags the PLOAM layer owns: set and cleared by PLOAM
	 * events and by the poll teardowns.  The CAM/table work itself is behind
	 * the install ops. */
	bool omcc_installed;		/* OMCC GEM datapath installed              */
	/* ★★ THE GEM THAT WAS INSTALLED, so a REASSIGNMENT is not ignored.  The
	 * install used to be one-shot (`!omcc_installed`), so an OLT moving the
	 * OMCC to a different GEM after O5 was silently dropped: the ONU kept
	 * binding the old port and management died with nothing to read.
	 * ⚠ 0 when nothing is installed; GEM 0 is not a legal OMCC port. */
	u16 omcc_gem;
	/* ★ MOVED UNCHANGED, AND IT IS DEAD: measured 2026-08-05, nothing ever
	 * assigns `true`.  It is cleared on all four teardown paths and read
	 * only to pick a log string and to print `tcont_done=`, so both readers
	 * report a constant false.  Recorded, not fixed: setting it would change
	 * which branch the Assign_ONU-ID log takes. */
	bool tcont_installed;		/* OMCC T-CONT/alloc-id bound (one-shot)     */
	/* Cleared by the teardowns here and READ by the install gate here, but
	 * SET by the GEM layer from inside its install, deliberately BEFORE the
	 * US-NIC modeset because the modeset's collision fix reads it.  Setting
	 * it from the core after the op returned would change behaviour; use
	 * gpon_ploam_set_data_installed() at exactly that point instead. */
	bool data_installed;		/* WAN data GEM datapath installed           */
	/* ★ THESE THREE ARE SESSION STATE, AND EVERY TEARDOWN CLEARS THEM.  They
	 * used to survive: data_tcont_installed had NO teardown clear at all,
	 * only the OLT's Deallocate, which additionally demands the Alloc-ID of
	 * the session that just ended — so after any re-config the ONU refused
	 * the OLT's NEW Alloc-ID and waited for a Deallocate that would never
	 * come again.  "Works after a cold boot, dies after churn."  The contract
	 * is asserted per teardown path by
	 * dev/rtl9607c-test/gpon_data_bind_test.c (step 19). */
	bool data_gem_solicited;	/* OLT sent the OMCI ME268 Create -> may install */
	u16  data_gem_port;		/* ★ the OLT's WIRE GEM Port-ID from that ME 268 */
	/* PLOAM authorization is independent of the single selected HW data path.
	 * A fixed 512-byte bitmap represents all 12-bit identities without a
	 * capacity policy. LOS/popup keeps it; Deactivate and new serial clear it. */
	u32 data_alloc_assigned[128];
	bool data_tcont_installed;	/* the OLT's DATA Alloc-ID bound to the data T-CONT */
	u16  data_alloc;		/* the OLT's data Alloc-ID                   */
	/* --- AES key exchange --------------------------------------------- */
	u8   aes_key[16];
	u8   key_index;
	u32  aes_switch_time;		/* last armed Key_Switching_Time (de-dup)    */
	bool key_staged;		/* a valid key is loaded in the staged bank  */
	/* --- outage / re-range accounting (time in ms, supplied by the shell) */
	u32 rerange_start_ms;		/* 0 = no outage in flight                   */
	u32 rerange_last_log_ms;	/* 0 = never logged (flap damping)           */
	u32 rerange_cnt;
	u32 last_outage_ms;
	/* --- counters the shell/proc read -------------------------------- */
	u32 sn_tx;			/* Serial_Number_ONU messages emitted        */
	u32 ds_rx;			/* DS PLOAMs dispatched (DS-lock liveness)   */
	u32 tx_total;			/* US PLOAMs emitted (all queues)            */
	u8  last_ds_type;		/* last DS PLOAM type seen                   */
};

static inline void gpon_ploam_data_alloc_note(struct gpon_ploam *o,
					      u16 alloc, bool assigned)
{
	if (!gpon_data_alloc_valid(alloc))
		return;
	if (assigned)
		o->data_alloc_assigned[alloc / 32] |= 1u << (alloc % 32);
	else
		o->data_alloc_assigned[alloc / 32] &= ~(1u << (alloc % 32));
}

static inline bool gpon_ploam_data_alloc_known(const struct gpon_ploam *o,
					      u16 alloc)
{
	return alloc <= 4095 &&
	       (o->data_alloc_assigned[alloc / 32] & (1u << (alloc % 32)));
}

static inline void gpon_ploam_data_alloc_reset(struct gpon_ploam *o)
{
	unsigned int i;

	for (i = 0; i < 128; i++)
		o->data_alloc_assigned[i] = 0;
}


/* Entry points.  ★ EVERY ONE TAKES `now_ms`, a free-running millisecond clock
 * the shell supplies; the core never reads a clock, and `now_ms` is only ever
 * used through wrap-safe signed differences, so any 32-bit monotonic ms source
 * works.
 *
 * ★ THE POLL ISLANDS ARE SEPARATE CALLS ON PURPOSE.  In the Luna driver these
 *   decisions are INTERLEAVED with shell-only work inside one poll function,
 *   and the interleaving is load-bearing: the feed re-kick and the
 *   upstream-interrupt service are gated on `state == O5 && omcc_installed`,
 *   which the watchdog and the LOS re-range can clear in the same tick.
 *   Merging the islands into one poll() would change which of them run on the
 *   tick a re-range fires. */

/* Initialise the context.  `sn` may be NULL (left zero until provisioned).  Sets
 * the G.984.3 burst defaults and parks the FSM at O1; touches no hardware and
 * calls no op.
 * ★★ IT VALIDATES THE TABLE FIRST AND REFUSES: returns NULL on success, or the
 *    name of the first MANDATORY op the table leaves NULL.  On refusal NOTHING
 *    is installed — @o is left zeroed with ops == NULL — so a half-wired shell
 *    cannot drive hardware and cannot oops on the first transition that reaches
 *    the hole.  init is the ONLY way to get a usable context, so the check
 *    cannot be opted out of, and GPON_MUST_CHECK makes ignoring the answer a
 *    -Werror build failure. */
GPON_MUST_CHECK const char *
gpon_ploam_init(struct gpon_ploam *o, const struct gpon_ploam_ops *ops,
		const struct gpon_ploam_cfg *cfg, void *sh, const u8 sn[8]);

/* Install a (re)provisioned serial number and ask the FSM to re-range; the
 * re-range happens on the next gpon_ploam_sn_changed().  ★ THE CODEC IS NOT
 * HERE: "XPON12345678" -> 8 wire bytes is gpon_sn_parse() in gpon_sn.c, one
 * implementation for every family and for x86. */
void gpon_ploam_set_sn(struct gpon_ploam *o, const u8 sn[8]);

/* The OMCI layer saw the OLT's ME 268 (GEM-CTP) Create for the WAN data GEM.
 * Until this is set the data GEM is never installed — installing ahead of the
 * OLT's own Create is what made a second admit churn-lock.
 * ★ @port_id is G.988 ME 268 attribute 1, the WIRE GEM Port-ID, and it is the
 *   OLT's to choose (measured on this lab OLT: 223 to one board, 193 to
 *   another).  The caller — the only layer that knows which ME 268 instance is
 *   the WAN one — must NOT offer the multicast GEM here.  A @port_id differing
 *   from the one installed re-arms the install; repeating one is idempotent. */
void gpon_ploam_set_data_gem_solicited(struct gpon_ploam *o, bool solicited,
				       u16 port_id);

/* The GEM layer has installed (or torn down) the WAN data GEM datapath.  Called
 * from inside the install, BEFORE the US-NIC modeset, which reads the flag. */
void gpon_ploam_set_data_installed(struct gpon_ploam *o, bool installed);

/* Advance the FSM tick counter by one and return the new value; the shell uses
 * the return for its own cadence gates, so there is a single tick source. */
u32 gpon_ploam_tick(struct gpon_ploam *o);

/* Dispatch one downstream PLOAM.  `len` must be at least GPON_PLOAM_DS_LEN; a
 * shorter frame is DROPPED with no state change — the driver could never be
 * handed one, and the length exists so the offline fuzzer cannot walk off the
 * buffer.  Returns the number of US PLOAMs emitted while handling it. */
int gpon_ploam_ds(struct gpon_ploam *o, const u8 *m, unsigned int len, u32 now_ms);

/* Poll islands, in the order the shell must call them.  Each returns the number
 * of US PLOAMs it emitted. */
int gpon_ploam_sn_changed(struct gpon_ploam *o, u32 now_ms);
int gpon_ploam_poll_provision(struct gpon_ploam *o, u32 now_ms);
/* `wan_rx_zero` = the WAN netdev has received nothing since the last re-range;
 * sampled by the shell because it lives in another driver. */
int gpon_ploam_poll_watchdog(struct gpon_ploam *o, bool wan_rx_zero, u32 now_ms);
/* `optic_los` = the GTC optical loss-of-signal bit, `sds_dark` = the SoC SerDes
 * signal-detect is gone.  Both sampled by the shell; only the AND is a real
 * fibre pull. */
int gpon_ploam_poll_los(struct gpon_ploam *o, bool optic_los, bool sds_dark, u32 now_ms);
int gpon_ploam_poll_sn_reoffer(struct gpon_ploam *o, u32 now_ms);
int gpon_ploam_poll_keepalive(struct gpon_ploam *o, u32 now_ms);

#endif /* GPON_PLOAM_H */
