// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: G.988 OMCI baseline message layer.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 */
/*
 * gpon_omci_core.c — the ITU-T G.988 OMCI baseline MESSAGE layer, common to
 * every OpenWrt GPON target in this tree.  See gpon_omci_core.h for the full
 * statement of what this file is; the short form, repeated here because this
 * is the file people edit:
 *
 *   WHAT   parse a DS baseline PDU, dispatch it by message type, build the US
 *          response (trailer + MIC).  NO managed-entity storage: the ME model,
 *          the board identity data, the MIB-Upload rows and the dynamic
 *          OLT-created instance store are the ME-model layer's, and this file
 *          reaches them only through the calls listed under CONTRACT below.
 *
 *   WHY    Operator, 2026-08-05: "en openwrt debería estar estructurado algo
 *          así: rtl960x* para la familia para tener código común" … "la idea es
 *          poner en común el código que corresponde para no tener mucho
 *          duplicado"; on the two per-target monoliths each carrying a private
 *          copy of G.988: "mal, poner en común".  G.988 is a specification, not
 *          a chip fact, so one copy.  Compiled by realtek-elnath (RTL9607F /
 *          Cortina, aarch64 LITTLE-endian), by realtek-luna (RTL960xC / Luna,
 *          MIPS32 BIG-endian) once follow-ups F1/F2/F3 land, and by
 *          dev/rtl9607c-test on x86-64 through fuzz_shims/.
 *
 *   RULE   FUNCTIONAL CORE — it decides, it never does.  No device pointer, no
 *          lock (the caller serialises), no allocation, no sleep, no clock.
 *          All state lives in the caller's struct omci_onu.
 *          ⇒ THIS FILE MUST NEVER GAIN AN MMIO ACCESS.  No readl/writel, no
 *          ioremap, no msleep/udelay, no jiffies, no spin_lock/mutex, no
 *          kmalloc, no dev_ or netdev_ logging.  The purity check greps for
 *          that exact set; one of them turns the offline gate red.  That
 *          property is what makes this layer host-compilable, fuzzable and
 *          portable across both architectures at once.
 *
 *   BYTES  All wire access is explicit byte math, never a struct or pointer
 *          cast over wire bytes.  One source, two endiannesses, same octets.
 *
 * PROVENANCE.  This is CODE MOTION, not a redesign: every function body below
 * came unchanged from the responder that is live on Elnath at stock parity
 * (realtek-elnath .../cortina/omci_responder.c), whose wire layout was itself
 * proven end-to-end — Online/normal + WAN — against the HSGQ-G008 OLT.  The
 * layout rule the whole thing follows: message contents start at octet 8, and
 * only a response that carries a RESULT code spends that octet on the result.
 * Where Luna's independent responder disagrees, the divergence is named at the
 * line it concerns with its follow-up id; NOTHING was converged here, because
 * converging changes bytes on a wire and this refactor may not.
 *
 * CONTRACT — what this layer requires from the ME-model layer (gpon_omci_me.h):
 *   types      struct omci_onu, struct omci_me_inst, struct omci_mib_row
 *   store      omci_store_find, omci_store_has_class, omci_store_nth,
 *              omci_store_put, omci_store_merge, omci_store_del
 *   model      omci_me_fill, omci_inst_exists, omci_class_modelled
 * All are pure.  Nothing here calls anything else outside libc-equivalents.
 */
#include <linux/crc32.h>
#include <linux/string.h>

#include "gpon_omci_core.h"
#include "gpon_omci_me.h"	/* struct omci_onu + the ME model / dynamic store */

/*
 * MIC (bytes 44..47) = the I.363.5 / AAL5 CRC-32 over bytes 0..43 (G.984.4
 * baseline trailer): NON-reflected polynomial 0x04C11DB7 MSB-first, init
 * all-ones, final complement — the kernel's crc32_be — stored big-endian.
 * LIVE-PROVEN on this OLT: the DS OMCI frames' MIC matches ~crc32_be(~0,
 * msg, 44) and NOT the reflected zlib crc32_le (the DS MIC self-check in
 * cortina-gpon.c logs which variant each received frame carries).  Computed
 * in SOFTWARE: the GPON MAC's own OMCI CRC engine stays enabled
 * (onu_cfg.omci_crc_dis = 0, the stock value) — if the HW also inserts, it
 * writes the same correct bytes.  A zero/wrong MIC = the OLT silently drops
 * every response and loops its GET audit (proven failure class).
 *
 * DIVERGENCE, follow-up F3 — NOT resolved here, deliberately.  Luna computes
 * the OTHER variant: rtl9602c_eth.c's rtl9602c_omci_set_mic() does
 * crc32_le(~0, msg, 44) ^ ~0, the reflected zlib CRC-32.  G.984.4 says AAL5
 * and the Elnath form is the one measured on the wire, yet Luna reaches O5 and
 * provisions against that same OLT — which nothing in either tree explains
 * (does its MAC also insert?  does the OLT not validate US?).  The host oracle
 * computes no MIC at all and therefore cannot arbitrate.  Picking on the
 * strength of a comment would change Luna's emitted bytes on a hunch, so when
 * Luna joins this engine the variant becomes a per-chip config selector and
 * each target keeps the bytes it emits today.  The measurement that settles
 * it: capture X111W's US OMCI and compare bytes 44..47 against both variants,
 * exactly as the Elnath DS self-check already does for the downstream.
 */
/*
 * ★★★ A DS FRAME WHOSE MIC DOES NOT VERIFY IS DISCARDED (G.988: an
 * invalid-MIC message is discarded).  Until this existed EVERY frame reached
 * the responder, including 8..47-byte runts that cannot carry a MIC, and the
 * consequences were not theoretical:
 *
 *   - a corrupted Set was APPLIED and ACKed with the OLT's own TID, so the OLT
 *     never retransmitted and MDS stayed in LOCKSTEP with its lsync -- no ME2
 *     audit could ever detect the divergence;
 *   - a garbage alloc-id so latched reaches the HW T-CONT CAM, worst case
 *     bursting into ANOTHER ONU's grant slot, which is the never-wedge-the-PON
 *     bar this project holds itself to;
 *   - a corrupted mt byte faked a whole MIB-Reset teardown.
 *
 * The recovery is the OLT's own: its AR-timeout retransmit (typically x3) is
 * the fast path, and a lost non-AR config still surfaces at the next ME2 MDS
 * audit because our un-bumped MDS then mismatches lsync.  TWO OLT-driven
 * self-heal layers, both proven live on this HG08.
 *
 * ★ NO NEW CONVENTION RISK: omci_set_mic below already commits us to AAL5-BE
 *   on TX and this OLT accepts those MICs, so RX enforces the SAME convention.
 */
bool omci_mic_ok(const u8 *msg, unsigned int len)
{
	u32 c;

	/* A frame shorter than the baseline 48 cannot CARRY a MIC, so it cannot
	 * be verified -- and an unverifiable frame is exactly what must not be
	 * acted on.  This is the runt case that used to reach the MIB-Reset arm. */
	if (!msg || len < OMCI_LEN)
		return false;
	c = ~crc32_be(~0u, msg, 44);
	return msg[44] == (u8)(c >> 24) && msg[45] == (u8)(c >> 16) &&
	       msg[46] == (u8)(c >> 8)  && msg[47] == (u8)c;
}

/* ★ EXPORTED so a host test STAMPS WITH THE SHIPPED STAMPER instead of a copy.
 * The MIC gate above means an unstamped frame is now correctly discarded, and
 * several host tests were building PDUs with no MIC at all -- they model an OLT,
 * and a real OLT always stamps.  Handing them this function rather than letting
 * each grow its own keeps ONE convention: if AAL5-BE ever changed, the tests
 * would follow instead of silently testing the old one. */
void omci_set_mic(u8 *msg)
{
	u32 c = ~crc32_be(~0u, msg, 44);

	msg[44] = (u8)(c >> 24);
	msg[45] = (u8)(c >> 16);
	msg[46] = (u8)(c >> 8);
	msg[47] = (u8)c;
}

/* Stamp the baseline trailer (40..43 = 00 00 00 28) + MIC.  Call LAST. */
static void omci_finalize(u8 *msg)
{
	msg[40] = 0x00;
	msg[41] = 0x00;
	msg[42] = 0x00;
	msg[43] = 0x28;
	omci_set_mic(msg);
}

/*
 * GET-response filler: result(8) + attr-mask(9,10) + values(11..35) + the two
 * masks G.988 RESERVES at 36..39 even on a success reply — the optional-
 * attribute ("unsupported") mask and the attribute-execution ("failed") mask.
 * So the value area is 25 octets, not 29: ONU-G attrs 1|2|3 are 4+14+8 = 26
 * bytes and a conformant OLT decoder would read a serial number short by its
 * last byte plus a bogus non-zero unsupported mask.
 *
 * Three masks decide the answer:
 *   requested (@mask), known (what the ME models), returned (what fit)
 *   unsupported = requested & ~known      -> named at 36..37
 *   failed      = requested & known & ~returned -> named at 38..39
 *   result      = 0x09 when either is set, else 0x00
 * "result 0 with a short attribute mask" is the audit-loop generator: the OLT
 * has no way to learn which attributes to stop asking for, so it re-GETs
 * forever.  Naming them is what ends the loop.
 *
 * Falls back to the dynamic store for OLT-created MEs (a GET of a provisioned
 * ME must not answer UNKNOWN_ME, which aborts the OLT's config load).
 *
 * DIVERGENCE, follow-up F2 — Luna's rtl9602c_omci_get_fill() passes resp + 40
 * as the end of the value area (29 octets) and so overwrites BOTH reserved
 * masks.  Not changed here; changing it changes Luna's wire bytes.
 */
static u8 omci_get_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
			u8 *resp)
{
	u16 rmask = 0, known = 0, unsup, failed;
	u8 rc = omci_me_fill(o, class_id, inst, mask, resp + 11, resp + 36,
			     &rmask, &known);

	if (rc == OMCI_RC_UNKNOWN_ME) {
		struct omci_me_inst *e = omci_store_find(o, class_id, inst);

		if (!e) {
			/* Nothing here at all.  If the OLT created OTHER
			 * instances of this class the class IS known and only
			 * the instance is not (0x05); otherwise the class
			 * itself is unknown (0x04). */
			omci_put_be16(resp + 9, 0);
			return omci_store_has_class(o, class_id) ?
					OMCI_RC_UNKNOWN_INST :
					OMCI_RC_UNKNOWN_ME;
		}
		/* Opaque set-by-create/set body: no descriptor table exists for
		 * an OLT-created class, so the bytes are replayed as-is and the
		 * requested mask is echoed (best-effort, bounded by the 25-octet
		 * area).  Naming them unsupported instead would make the OLT
		 * abandon a ME it just provisioned. */
		memcpy(resp + 11, e->body, e->blen > 25 ? 25 : e->blen);
		rmask = mask;
		known = mask;
	}

	omci_put_be16(resp + 9, rmask);
	unsup = (u16)(mask & ~known);
	failed = (u16)(mask & known & ~rmask);
	omci_put_be16(resp + 36, unsup);
	omci_put_be16(resp + 38, failed);
	return (unsup | failed) ? OMCI_RC_ATTR_FAILED : OMCI_RC_OK;
}

/*
 * Create / Set / Delete: APPLY or NAK, and move MIB-Data-Sync ONLY when the
 * MIB actually changed.  An ACK the ONU did not honour is worse than a NAK:
 * the OLT stops retrying AND its lsync still matches our MDS, so the ME 2
 * audit can never discover the divergence.
 *   Create: duplicate instance -> 0x07, full store -> 0x09 (frozen MDS lets
 *           the OLT's own audit self-heal), else store + MDS+1.
 *   Delete: absent instance -> 0x05.
 *   Set:    unknown class -> 0x04, known class + absent instance -> 0x05.
 * Attribute-level Set validation is deliberately NOT done: this OLT Sets
 * ME 131 (OLT-G) attributes the ONU does not model and expects OK, and G.988
 * has no way for the ONU to announce a per-attribute write capability.
 */
static u8 omci_config_apply(struct omci_onu *o, u8 mt, u16 class_id, u16 inst,
			    const u8 *msg, unsigned int len)
{
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);
	u16 mask;

	switch (mt) {
	case OMCI_MT_CREATE:
		/* The dynamic store is the OLT-created space only: an
		 * auto-instantiated ME is not "existing" for Create purposes
		 * (this OLT Creates ME 262/268-shaped instances that the static
		 * model also describes). */
		if (e)
			return OMCI_RC_INST_EXISTS;
		if (!omci_store_put(o, class_id, inst, msg + 8,
				    (len > 8) ? (int)(len - 8) : 0))
			return OMCI_RC_ATTR_FAILED;
		break;
	case OMCI_MT_DELETE:
		if (!e)
			return OMCI_RC_UNKNOWN_INST;
		omci_store_del(o, class_id, inst);
		break;
	default:					/* OMCI_MT_SET */
		if (len < 10)		/* no attribute mask on the wire */
			return OMCI_RC_PARAM_ERROR;
		if (!omci_inst_exists(o, class_id, inst))
			return (omci_class_modelled(class_id) ||
				omci_store_has_class(o, class_id)) ?
					OMCI_RC_UNKNOWN_INST :
					OMCI_RC_UNKNOWN_ME;
		mask = ((u16)msg[8] << 8) | msg[9];
		if (e)
			omci_store_merge(e, msg + 10, (int)(len - 10));
		/* An OLT Set of ME2 attr-1 is an explicit resync write: take
		 * its byte first, then this Set's own +1 still applies. */
		if (class_id == OMCI_ME_ONU_DATA && len >= 11 &&
		    (mask & 0x8000))
			o->mds = msg[10];
		break;
	}

	/* MIB-Data-Sync: +1 per applied config message (not per attribute),
	 * wrap 255 -> 1 (0 = just-reset). */
	if (++o->mds == 0)
		o->mds = 1;
	return OMCI_RC_OK;
}

/*
 * The adaptive MIB-Data-Sync walk.  See OMCI_MDS_WALK_STEP in gpon_omci_me.h for
 * why 0 is outside the search space rather than folded onto 1.
 */
void omci_mds_walk(struct omci_onu *o)
{
	if (!o->mds_adapt || !o->mds_adapt_reads)
		return;
	if (++o->audit_reads < o->mds_adapt_reads)
		return;
	o->audit_reads = 0;
	/* 1..255: an mds of 0 on entry (only reachable straight after an OLT
	 * MIB-Reset) steps to OMCI_MDS_WALK_STEP, staying inside the range. */
	o->mds = (u8)(1 + ((unsigned int)o->mds + OMCI_MDS_WALK_STEP - 1) % 255);
	o->mds_tries++;
}

int omci_onu_input(struct omci_onu *o, const u8 *msg, unsigned int len, u8 *resp)
{
	u16 class_id, inst;
	u8 mt, devid;

	/*
	 * ★ A BASELINE OMCI PDU IS 48 BYTES, FULL STOP (G.988 A.3).  This gate
	 * used to be `len < 8` — enough to READ the header — so a truncated
	 * frame was answered with a full 48-byte response built from bytes the
	 * OLT never sent.  MEASURED 2026-08-30: every length 8..47 drew a
	 * reply, and the oracle mirrored the same wrong rule, so the
	 * differential reported ZERO divergence on 1276 malformed frames.  Two
	 * independent implementations agreeing on a defect is exactly the case
	 * a differential cannot see, and it is why this needed a spec reading
	 * rather than a comparison.
	 *
	 * ★ COUNTED SEPARATELY FROM rx_bad_mic, because "too short to be a
	 * message" and "a message whose MIC failed" are different facts about
	 * the link: the first is a framing or GEM-reassembly fault upstream of
	 * OMCI, the second is corruption on an otherwise well-framed PDU.
	 * Collapsing them would make a broken GEM reassembler look like a noisy
	 * fibre.
	 */
	if (len < OMCI_LEN) {
		o->rx_runt++;
		return 0;
	}
	devid = msg[3];
	mt = msg[2] & 0x1f;
	class_id = ((u16)msg[4] << 8) | msg[5];
	inst = ((u16)msg[6] << 8) | msg[7];

	if (devid != 0x0a) {
		/* Only the BASELINE message set is modelled.  An extended-format
		 * request (devid 0x0b) cannot be answered in baseline format —
		 * the response device identifier must match — so it is counted
		 * and dropped rather than answered wrongly.  ONU2-G attribute 2
		 * (OMCC version) therefore advertises 0x80 = G.984.4 BASELINE:
		 * a conformant OLT never sends an extended frame to us, and the
		 * counter says loudly if one ever does. */
		if (devid == 0x0b)
			o->rx_extended++;
		return 0;
	}

	/* ★ THE MIC GATE, BEFORE THE REPLAY CACHE.  A corrupted frame must not be
	 * served from the cache either: the cache is keyed on bytes 0..39, so a
	 * frame whose corruption lies there would miss it anyway, and one whose
	 * corruption lies in 40..47 would be REPLAYED as though it were the good
	 * request.  Counted so a link going bad is visible rather than silent. */
	if (!omci_mic_ok(msg, len)) {
		o->rx_bad_mic++;
		return 0;
	}

	/* G.988 11.2.2.1 retained last response: the OMCC is stop-and-wait, so
	 * a byte-identical repeat of the request we last answered is a
	 * RETRANSMISSION (our US response was lost — cg_omci_tx drops on NI
	 * ring-busy, and a US burst can die on the wire).  Replay the stored
	 * response instead of re-executing: re-execution bumps MDS a second
	 * time for ONE OLT transaction, and ONU mds = OLT lsync + 1 costs a
	 * full MIB-Reset/re-provision churn window at the next ME 2 audit.
	 * Bytes 40..47 (trailer + MIC) are derived, so 0..39 is the identity. */
	if (o->have_last && len >= 40 && !memcmp(msg, o->last_req, 40)) {
		memcpy(resp, o->last_resp, OMCI_LEN);
		o->dup_replay++;
		return OMCI_LEN;
	}

	memset(resp, 0, OMCI_LEN);
	resp[0] = msg[0];			/* TID echo */
	resp[1] = msg[1];
	resp[2] = (msg[2] & 0x1f) | 0x20;	/* clear AR, set AK */
	resp[3] = 0x0a;
	resp[4] = msg[4];			/* class echo */
	resp[5] = msg[5];
	resp[6] = msg[6];			/* instance echo */
	resp[7] = msg[7];

	switch (mt) {
	case OMCI_MT_MIB_RESET:
		/* On-wire MIB-Reset: zero MIB-Data-Sync (the OLT recounts its
		 * lsync from 0; keeping a seed here = permanent mismatch ->
		 * Deactivate loop, proven) + drop the provisioned store. */
		o->mds = 0;
		memset(o->store, 0, sizeof(o->store));
		o->store_n = 0;
		/* a provisioning event is the walk's GOAL, reached: rearm it */
		o->audit_reads = 0;
		o->mds_tries = 0;
		/* ★★★ AND THE VEIP OPER-UP AVC MUST BE RE-EMITTED.  A MIB-Reset
		 * wipes the OLT's view, so it will wait for the port-up AVC
		 * again -- but the latch said "already sent" and the ~31 s work
		 * never re-ran, leaving the WAN GATED FOREVER with no recovery
		 * short of a deact/re-range churn the production bar forbids.
		 * The boot path only ever worked because the OLT's MIB-Reset
		 * happens to land BEFORE the 31 s timer fires; a mid-session
		 * one had nothing behind it.  Clearing the latch here is the
		 * responder's half; the shell re-arms the timer (its own half). */
		o->avc_veip_up_sent = false;
		resp[8] = OMCI_RC_OK;
		break;
	case OMCI_MT_MIB_UPLOAD:
		/* Row count at contents[8..9], NO result byte (a result byte
		 * here made the OLT read count=0 and never walk, proven). */
		omci_put_be16(resp + 8, o->nrows + o->store_n);
		break;
	case OMCI_MT_GET:
		if (len < 10)	/* mask missing: a shorter GET would read
				 * stale bytes into the reply (info leak) */
			return 0;
		resp[8] = omci_get_fill(o, class_id, inst,
					((u16)msg[8] << 8) | msg[9], resp);
		/* ★ THE WALK IS CALLED FROM HERE AND NOWHERE ELSE: a GET is the
		 * unit the OLT's audit is made of, so it is the only event that
		 * means "read us again without provisioning". */
		omci_mds_walk(o);
		break;
	case OMCI_MT_SET:
	case OMCI_MT_CREATE:
	case OMCI_MT_DELETE:
		resp[8] = omci_config_apply(o, mt, class_id, inst, msg, len);
		if (resp[8] == OMCI_RC_OK) {
			/* the OLT provisioned: the walk reached its goal */
			o->audit_reads = 0;
			o->mds_tries = 0;
		}
		break;
	case OMCI_MT_GET_ALL_ALARMS:
		/* Alarm-entry count at contents[8..9], NO result byte — same
		 * shape as MIB-Upload.  (At 9..10 the count's high byte lands
		 * where the OLT reads a result code: latent while the count is
		 * always 0, wrong the moment an alarm is reported.) */
		/* ★★ WHAT IS ACTUALLY ASSERTED, not a constant. This answered a
		 * hardcoded 0 until 2026-09-02, so the ONU told every OLT that
		 * nothing was wrong however loudly the silicon disagreed -- and
		 * the comment above was the only record that the byte layout was
		 * correct BY ACCIDENT while the count could not be non-zero. */
		omci_put_be16(resp + 8, omci_alarm_count(o));
		break;
	case OMCI_MT_MIB_UPLOAD_NX: {
		/* Request seq at msg[8..9]; reply = class[8..9] + inst[10..11]
		 * + attr-mask[12..13] + values[14..39], NO result byte. */
		u16 seq;
		u16 wmask = 0, wknown = 0;

		if (len < 10)
			return 0;
		seq = ((u16)msg[8] << 8) | msg[9];
		if (seq < o->nrows) {
			const struct omci_mib_row *r = &o->rows[seq];

			omci_put_be16(resp + 8, r->class_id);
			omci_put_be16(resp + 10, r->inst);
			omci_me_fill(o, r->class_id, r->inst, r->mask,
				     resp + 14, resp + 40, &wmask, &wknown);
			omci_put_be16(resp + 12, wmask);
		} else if (seq < o->nrows + o->store_n) {
			/* provisioned MEs after the static rows: present-only
			 * (mask 0); values are served via GET. */
			const struct omci_me_inst *e =
				omci_store_nth(o, seq - o->nrows);

			if (e) {
				omci_put_be16(resp + 8, e->class_id);
				omci_put_be16(resp + 10, e->inst);
			}
		}
		/* out-of-range seq -> all-zero row, still well-formed */
		break;
	}
	case OMCI_MT_TEST:
	case OMCI_MT_SYNC_TIME:
	case OMCI_MT_REBOOT:		/* ACK, do NOT actually reboot */
	case OMCI_MT_START_SW_DL:
	case OMCI_MT_DOWNLOAD_SEC:
	case OMCI_MT_END_SW_DL:
	case OMCI_MT_ACTIVATE_SW:
	case OMCI_MT_COMMIT_SW:
		/* not performed (no SW image to flash), but must ACK OK so
		 * the OLT's provisioning FSM completes */
		resp[8] = OMCI_RC_OK;
		break;
	case OMCI_MT_GET_ALL_ALRM_NX:
	case OMCI_MT_GET_NEXT:
		/* Get Next walks a TABLE attribute; the ME model defines none
		 * (only an OLT-created ME could have one, and its body is
		 * opaque to us), so the honest answer is the "action this ME
		 * does not implement" one below: result 0x00 with empty
		 * contents = nothing to return / end of table.  Get-All-Alarms-
		 * Next likewise: no alarm table to walk.  resp is already zero. */
		break;
	default:
		/* A message type with no ONU-side action.  Answer result 0x00
		 * with EMPTY contents, never 0x02 and never silence: stock
		 * behaves this way, an unanswered OLT request is a documented
		 * deactivation trigger, and 0x02 has been seen to abort a
		 * foreign OLT's config load.  Counted so /proc shows if an OLT
		 * ever sends one (spy-capability rule). */
		o->unhandled++;
		break;
	}

	omci_finalize(resp);

	/*
	 * Refresh the retransmission cache.  It may only ever hold the response
	 * to the request we answered MOST RECENTLY: when this request produced
	 * no response (AR clear) or is too short to be identified by its first
	 * 40 bytes, the previous entry must be DROPPED — the MIB may just have
	 * changed underneath it, and replaying it would answer a later
	 * transaction with a pre-change reply (an AR=0 Set followed by a repeat
	 * of the ME 2 audit GET would report the OLD MIB-Data-Sync).
	 */
	if ((msg[2] & 0x40) && len >= 40) {
		memcpy(o->last_req, msg, 40);
		memcpy(o->last_resp, resp, OMCI_LEN);
		o->have_last = true;
	} else {
		o->have_last = false;
	}

	/* AR clear = the OLT asked for no acknowledgement (G.988): the message
	 * is APPLIED above, but nothing goes upstream.  Counted, because a
	 * silent path still has to be observable — /proc says whether this OLT
	 * ever uses it (spy-capability rule). */
	if (!(msg[2] & 0x40)) {
		o->no_ack++;
		return 0;
	}
	return OMCI_LEN;
}

/*
 * Autonomous AVC (MT 0x11, TID 0): report that (class, inst)'s attributes in
 * @mask changed to @val.  The OLT never GETs the data-plane MEs after
 * creating them — its per-class AVC handlers gate DOWNSTREAM user-data
 * forwarding on the ONU's operational report. */
void omci_onu_set_alarms(struct omci_onu *o, u16 class_id, u16 inst,
			 u16 bitmap)
{
	if (!o)
		return;
	/* ★ ONE ME'S WORTH TODAY, and the shape says so rather than pretending
	 * otherwise: both shipping families report their PON conditions against
	 * a single ME. A second asserting ME needs a small array here, not a
	 * different design -- the message, the edge and the count are already
	 * per-instance. */
	o->alarm_class = class_id;
	o->alarm_inst = inst;
	o->alarm_active = bitmap;
}

u16 omci_alarm_count(const struct omci_onu *o)
{
	/* G.988: the number of ME INSTANCES currently reporting an alarm, not
	 * the number of alarm BITS. One instance with three conditions is one
	 * entry the OLT would walk. */
	return (o && o->alarm_class && o->alarm_active) ? 1 : 0;
}

int omci_onu_emit_alarm(struct omci_onu *o, u8 *out)
{
	if (!o || !out)
		return 0;
	if (!o->alarm_class)
		return 0;			/* nobody has reported anything */
	if (o->alarm_active == o->alarm_told)
		return 0;			/* ★ no EDGE -> no message */

	memset(out, 0, OMCI_LEN);
	/* TCI stays 0: G.988 marks an ONU-autonomous notification with a zero
	 * transaction id, the same convention omci_emit_avc uses one function
	 * below. AR/AK stay clear -- the OLT does not acknowledge an alarm. */
	out[2] = OMCI_MT_ALARM;
	out[3] = 0x0a;				/* DevID: baseline */
	omci_put_be16(out + 4, o->alarm_class);
	omci_put_be16(out + 6, o->alarm_inst);

	/* Contents (octets 8..39). G.988 clause 11.2.2: the alarm BITMAP occupies
	 * the first 28 octets, alarm number N living in bit (7 - N % 8) of octet
	 * N / 8 -- alarm 0 is the MOST significant bit of the first octet. The
	 * remaining octets are reserved, and the LAST octet of the contents area
	 * carries the alarm SEQUENCE NUMBER, which is how an OLT detects that it
	 * missed one. */
	out[8] = (u8)(o->alarm_active >> 8);
	out[9] = (u8)(o->alarm_active & 0xff);

	o->alarm_seq++;				/* wraps at 255 by construction */
	if (!o->alarm_seq)
		o->alarm_seq = 1;		/* 0 is reserved for "no sequence" */
	out[39] = o->alarm_seq;

	omci_finalize(out);
	o->alarm_told = o->alarm_active;	/* the edge is consumed HERE */
	o->alarm_emitted++;
	return OMCI_LEN;
}

void omci_emit_avc(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
		   const u8 *val, unsigned int vlen, u8 *out)
{
	memset(out, 0, OMCI_LEN);
	out[2] = OMCI_MT_AVC;
	out[3] = 0x0a;
	omci_put_be16(out + 4, class_id);
	omci_put_be16(out + 6, inst);
	omci_put_be16(out + 8, mask);
	if (val && vlen) {
		if (vlen > 30)
			vlen = 30;
		memcpy(out + 10, val, vlen);
	}
	omci_finalize(out);
	o->avc_count++;
}

int omci_onu_emit_veip_up_avc(struct omci_onu *o, u8 *out)
{
	/* VEIP inst 0x0601 attr #2 (operational state) mask 0x4000,
	 * value 0 = enabled (G.988). */
	static const u8 up = 0x00;

	omci_emit_avc(o, OMCI_ME_VEIP, 0x0601, OMCI_ATTR_BIT(2), &up, 1, out);
	o->avc_veip_up_sent = true;
	return OMCI_LEN;
}
