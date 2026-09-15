// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware: no
 * register access, no clock, no lock, no allocator, no device pointer.  One
 * source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Canonical tier rule and file map: "THE THREE TIERS" in gpon_common.h.
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17)
 * COMPILES this tier against stubs declaring no accessor, clock, lock or
 * allocator, so impurity cannot build.
 *
 * gpon_omci_core.c — the ITU-T G.988 OMCI baseline MESSAGE layer: parse a DS
 * baseline PDU, dispatch by message type, build the US response (trailer +
 * MIC).  NO managed-entity storage -- the ME model, the board identity, the
 * MIB-Upload rows and the dynamic OLT-created instance store are the ME-model
 * layer's, reached only through CONTRACT below.  Full statement:
 * gpon_omci_core.h.
 *
 * WHY (operator, 2026-08-05, on two per-target monoliths each carrying a
 * private copy of G.988: *"mal, poner en común"*): G.988 is a specification,
 * not a chip fact, so one copy.  Compiled by realtek-elnath (aarch64 LE), by
 * realtek-luna (MIPS32 BE) once follow-ups F1/F2/F3 land, and by
 * dev/rtl9607c-test on x86-64 through fuzz_shims/.
 *
 * RULE: it decides, it never does.  ⇒ NEVER GAIN AN MMIO ACCESS -- no
 * readl/writel, ioremap, msleep/udelay, jiffies, spin_lock/mutex, kmalloc, or
 * dev_/netdev_ logging.  The purity check greps for that exact set.  All wire
 * access is explicit byte math: one source, two endiannesses, same octets.
 *
 * PROVENANCE: CODE MOTION, not a redesign.  Every function body came unchanged
 * from the responder live on Elnath at stock parity, proven end-to-end
 * (Online/normal + WAN) against the HSGQ-G008 OLT.  The layout rule: message
 * contents start at octet 8, and only a response carrying a RESULT code spends
 * that octet on it.  Where Luna's independent responder disagrees the
 * divergence is named at the line it concerns with its follow-up id; NOTHING
 * was converged here, because converging changes bytes on a wire.
 *
 * CONTRACT — what this layer needs from gpon_omci_me.h, all pure:
 *   types  struct omci_onu, struct omci_me_inst, struct omci_mib_row
 *   store  omci_store_find, omci_store_has_class, omci_store_nth,
 *          omci_store_put, omci_store_merge, omci_store_del
 *   model  omci_me_fill, omci_inst_exists, omci_class_modelled
 */
#include <linux/crc32.h>
#include <linux/string.h>

#include "gpon_omci_core.h"
#include "gpon_omci_me.h"	/* struct omci_onu + the ME model / dynamic store */

/*
 * MIC (bytes 44..47) = the I.363.5 / AAL5 CRC-32 over bytes 0..43 (G.984.4
 * baseline trailer): NON-reflected polynomial 0x04C11DB7 MSB-first, init
 * all-ones, final complement — the kernel's crc32_be — stored big-endian.
 * LIVE-PROVEN on this OLT: the DS frames' MIC matches ~crc32_be(~0, msg, 44)
 * and NOT the reflected zlib crc32_le.  Computed in SOFTWARE with the MAC's own
 * OMCI CRC engine left enabled (onu_cfg.omci_crc_dis = 0, the stock value): if
 * the HW also inserts, it writes the same bytes.  A zero/wrong MIC = the OLT
 * silently drops every response and loops its GET audit (proven failure class).
 *
 * DIVERGENCE, follow-up F3 — NOT resolved here, deliberately.  Luna computes
 * the reflected zlib variant (rtl9602c_eth.c: crc32_le(~0, msg, 44) ^ ~0) and
 * still reaches O5 and provisions against the same OLT, which nothing in either
 * tree explains.  The host oracle computes no MIC and cannot arbitrate, so when
 * Luna joins this engine the variant becomes a per-chip selector and each
 * target keeps the bytes it emits today.  The measurement that settles it:
 * capture the X111W's US OMCI and compare bytes 44..47 against both variants.
 */
/*
 * ★★★ A DS FRAME WHOSE MIC DOES NOT VERIFY IS DISCARDED (G.988).  Until this
 * existed EVERY frame reached the responder, runts included, and the
 * consequences were not theoretical: a corrupted Set was APPLIED and ACKed with
 * the OLT's own TID, so MDS stayed in LOCKSTEP with its lsync and no ME2 audit
 * could detect the divergence; a garbage alloc-id so latched reaches the HW
 * T-CONT CAM, worst case bursting into ANOTHER ONU's grant slot; a corrupted mt
 * byte faked a whole MIB-Reset teardown.
 *
 * Recovery is the OLT's own -- its AR-timeout retransmit (typically x3), and a
 * lost non-AR config still surfaces at the next ME2 MDS audit.  Both self-heal
 * layers proven live on this HG08.
 *
 * ★ NO NEW CONVENTION RISK: omci_set_mic already commits us to AAL5-BE on TX
 *   and this OLT accepts those MICs, so RX enforces the SAME convention.
 */
/* ★ THE ONE PLACE THE AAL5-BE CONVENTION IS SPELLED.  It used to be spelled
 * three times -- verify and stamp here, plus the Cortina shell's DS self-check
 * -- and this exact CRC had already diverged once, when rtl9602c_eth.c stamped
 * the reflected zlib crc32_le while this file verified crc32_be and a correctly
 * behaving OLT rejected every frame that ONU sent.
 *
 * I.363.5 / AAL5: non-reflected CRC-32, init all-ones, final complement, over
 * bytes 0..43, stored big-endian at 44..47.  LIVE-PROVEN against this OLT.
 */
u32 omci_mic_compute(const u8 *msg)
{
	return ~crc32_be(~0u, msg, 44);
}

bool omci_mic_ok(const u8 *msg, unsigned int len)
{
	u32 c;

	/* A frame shorter than the baseline 48 cannot CARRY a MIC, so it cannot
	 * be verified -- and an unverifiable frame is exactly what must not be
	 * acted on.  This is the runt case that used to reach the MIB-Reset arm. */
	if (!msg || len < OMCI_LEN)
		return false;
	c = omci_mic_compute(msg);
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
	u32 c = omci_mic_compute(msg);

	msg[44] = (u8)(c >> 24);
	msg[45] = (u8)(c >> 16);
	msg[46] = (u8)(c >> 8);
	msg[47] = (u8)c;
}

/* Stamp the baseline trailer (40..43 = 00 00 00 28) + MIC.  Call LAST.
 *
 * ★ NOT static any more (2026-09-10).  The 0x0028 trailer length is a G.988
 * constant, so by this tree's tiering rule exactly one copy of it may exist --
 * and a second one had grown in a CHIP file: rtl9602c_omci_finalize() in
 * realtek-luna/.../rtl9602c_eth.c respelled these same four stores next to a
 * call to our omci_set_mic().  That is the identical shape that produced the
 * two-MIC-polynomial defect on this very function (see the note beside
 * omci_set_mic above): a shell copy of a spec constant, correct on the day it
 * was written, with nothing able to notice when the spec side moves.
 * Publishing it is the REBASE -- the home already existed, so no new core file
 * was added. */
void omci_finalize(u8 *msg)
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
	u8 rc;

	if (omci_me_mutable(class_id) && !omci_inst_exists(o, class_id, inst))
		return OMCI_RC_UNKNOWN_INST;
	rc = omci_me_fill(o, class_id, inst, mask, resp + 11, resp + 36,
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
 * Mapped classes validate masks atomically before changing state. Other
 * classes keep their existing compatibility behavior, including ME 131.
 */
static u8 omci_config_apply(struct omci_onu *o, u8 mt, u16 class_id, u16 inst,
			    const u8 *msg, unsigned int len, u8 *resp)
{
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);
	u16 mask;

	/* T-CONTs, PPTP Ethernet UNIs and UNI-Gs are all auto-instantiated: the
	 * ONU presents them and the OLT Sets and Gets them.  Own-stock action
	 * mask 0x300 permits Set/Get, not opaque Create/Delete shadow
	 * instances -- and for the two UNI classes a Create reaching the
	 * dynamic store would put an OPAQUE DUPLICATE of an inventory instance
	 * there: uploaded twice, answered from the inventory, and holding a
	 * store slot a provisioned ME then cannot have. */
	/* ME 50 joins them for the same reason and a different mechanism: G.988
	 * 9.3.4 makes the bridge TABLE ME the ONU's, created and deleted WITH
	 * its bridge port below, so an OLT Create would shadow one the ONU owns. */
	/* ME 79 is the third of that kind: G.988 9.3.3 makes the per-protocol
	 * filter pre-assign table the ONU's, created and deleted WITH the same
	 * bridge port, and stock's own plugin agrees -- its EntityId carries no
	 * set-by-create bit at all, so nothing about it is the OLT's to
	 * instantiate. */
	/* ★★ ME 49 is the fourth, and here the vendor states it outright rather
	 * than by omission: the ACTION MASK its own plugin registers with the
	 * framework is 0x04000300 -- Set, Get and Get-Next -- where ME 47 and ME
	 * 268, the two the OLT really does create, carry 0x350 =
	 * Create|Delete|Set|Get.  Read statically from each mibTable_init and
	 * IDENTICAL on both Luna dies, so refusing a Create of 49 is not our
	 * policy, it is stock's declared one. */
	if ((class_id == OMCI_ME_TCONT || class_id == OMCI_ME_PPTP_ETH_UNI ||
	     class_id == OMCI_ME_UNI_G ||
	     class_id == OMCI_ME_MAC_BRIDGE_TABLE ||
	     class_id == OMCI_ME_MAC_BRIDGE_FILTER ||
	     class_id == OMCI_ME_PREASSIGN_FILTER) &&
	    (mt == OMCI_MT_CREATE || mt == OMCI_MT_DELETE))
		return OMCI_RC_NOT_SUPPORTED;

	switch (mt) {
	case OMCI_MT_CREATE:
		if (e)
			return OMCI_RC_INST_EXISTS;
		if (!omci_store_create(o, class_id, inst, msg + 8,
				    (len > 8) ? (int)(len - 8) : 0))
			return OMCI_RC_ATTR_FAILED;
		/* The bridge port's own table ME, ATOMICALLY: a store with no
		 * room for the companion undoes the port too, rather than
		 * leaving a bridge port whose ME 50 the OLT can never Get. */
		if (class_id == OMCI_ME_MAC_BRIDGE_PORT &&
		    !omci_store_put(o, OMCI_ME_MAC_BRIDGE_TABLE, inst,
				    NULL, 0)) {
			omci_store_del(o, class_id, inst);
			return OMCI_RC_ATTR_FAILED;
		}
		/* ...and its filter pre-assign table, the same way and for the
		 * same reason.  It goes through omci_store_create() rather than
		 * omci_store_put() because it HAS a dense layout: a zero-length
		 * body would make every later Set fail the dense-length gate,
		 * which is a refusal the OLT would read as a broken ONU. */
		if (class_id == OMCI_ME_MAC_BRIDGE_PORT &&
		    !omci_store_create(o, OMCI_ME_PREASSIGN_FILTER, inst,
				       NULL, 0)) {
			omci_store_del(o, OMCI_ME_MAC_BRIDGE_TABLE, inst);
			omci_store_del(o, class_id, inst);
			return OMCI_RC_ATTR_FAILED;
		}
		/* ...and its MAC filter table, the third companion.  Through
		 * omci_store_put() like ME 50 rather than omci_store_create()
		 * like ME 79, because ME 49 has NO dense attribute at all --
		 * its one attribute is the row table, which lives in the VLAN /
		 * classification model beside ME 171's rows.  An instance with
		 * an empty body is exactly what the Set path wants: the dense
		 * gate is skipped for a class whose dense length is zero. */
		if (class_id == OMCI_ME_MAC_BRIDGE_PORT &&
		    !omci_store_put(o, OMCI_ME_MAC_BRIDGE_FILTER, inst,
				    NULL, 0)) {
			omci_store_del(o, OMCI_ME_PREASSIGN_FILTER, inst);
			omci_store_del(o, OMCI_ME_MAC_BRIDGE_TABLE, inst);
			omci_store_del(o, class_id, inst);
			return OMCI_RC_ATTR_FAILED;
		}
		break;
	case OMCI_MT_DELETE:
		if (!e)
			return OMCI_RC_UNKNOWN_INST;
		omci_store_del(o, class_id, inst);
		if (class_id == OMCI_ME_MAC_BRIDGE_PORT) {
			omci_store_del(o, OMCI_ME_MAC_BRIDGE_TABLE, inst);
			omci_store_del(o, OMCI_ME_PREASSIGN_FILTER, inst);
			omci_store_del(o, OMCI_ME_MAC_BRIDGE_FILTER, inst);
			/* the filter rows live outside the store, so the
			 * instance going away must take them with it --
			 * otherwise the next bridge port with this id inherits
			 * a stranger's MAC filter */
			gpon_mac_filter_del(&o->vlan, inst);
		}
		/* An ME 171 instance owns rows the dense store never held, so
		 * deleting the instance must drop them too -- otherwise the
		 * next instance with the same id inherits a stranger's VLAN. */
		if (class_id == OMCI_ME_EXT_VLAN)
			gpon_ext_vlan_del(&o->vlan, inst);
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
		if (omci_me_mutable(class_id)) {
			u16 unsupported, failed;
			u8 rc = omci_me_set(o, class_id, inst, mask, msg + 10,
					   30, &unsupported, &failed);

			/* Set response masks immediately follow the result byte;
			 * Get reserves its masks at 36/38 instead. */
			omci_put_be16(resp + 9, unsupported);
			omci_put_be16(resp + 11, failed);
			if (rc != OMCI_RC_OK)
				return rc;
		} else if (e && mask) {
			omci_store_merge(e, msg + 10, (int)(len - 10));
		}
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

int omci_onu_input_ex(struct omci_onu *o, const u8 *msg, unsigned int len,
		      u8 *resp, struct omci_accepted *accepted)
{
	u16 class_id, inst;
	u8 mt, devid;

	if (accepted)
		memset(accepted, 0, sizeof(*accepted));

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
		omci_me_reset_values(o);
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
		if (accepted)
			accepted->kind = OMCI_ACCEPT_RESET;
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
		resp[8] = omci_config_apply(o, mt, class_id, inst, msg, len, resp);
		if (resp[8] == OMCI_RC_OK) {
			if (accepted)
				accepted->kind = OMCI_ACCEPT_CONFIG;
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
			/* Mapped dynamic values use the same encoder as Get.
			 * Other classes retain their present-only rows. */
			const struct omci_me_inst *e =
				omci_store_nth(o, seq - o->nrows);

			if (e) {
				omci_put_be16(resp + 8, e->class_id);
				omci_put_be16(resp + 10, e->inst);
				/* ★ EVERY DENSE CLASS, not just ME 268.  A row
				 * that announces an instance and serves an
				 * EMPTY attribute mask tells the OLT the ME
				 * exists and nothing about it -- which is all a
				 * class with an opaque body can honestly say,
				 * and is now the answer only for classes that
				 * really are opaque. */
				if (omci_me_dense_len(e->class_id)) {
					omci_me_fill(o, e->class_id, e->inst, 0xffff,
						     resp + 14, resp + 40, &wmask, &wknown);
					omci_put_be16(resp + 12, wmask);
				}
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
		/* Get Next walks a TABLE attribute.  ⚠ THE MODEL NOW DEFINES
		 * TWO -- ME 171 #6, the subscriber VLAN rows, and ME 49 #1, the
		 * per-bridge-port MAC filter -- and BOTH ARE HELD, with the
		 * rows readable through gpon_ext_vlan_raw() /
		 * gpon_mac_filter_raw().  What is still missing is the
		 * ENCODING: how many rows a reply carries, how the sequence
		 * number indexes them, and what the last one answers.  So the
		 * answer stays "end of table" -- result 0x00 with empty
		 * contents -- and an OLT that AUDITS one of those tables
		 * re-writes it instead of reading it back, which is idempotent
		 * on both.  OWED, and it is a READ-BACK, never a write:
		 * RE libomci_mib.so's Get/Get-Next handler, which is on disk.
		 * Get-All-Alarms-Next likewise: no alarm table to walk.
		 * resp is already zero. */
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

	/* Acceptance describes a committed request, independently of AR and
	 * response delivery. The replay path above never reaches this point. */
	if (accepted && accepted->kind != OMCI_ACCEPT_NONE) {
		accepted->mt = mt;
		accepted->class_id = class_id;
		accepted->inst = inst;
		if (mt == OMCI_MT_SET)
			accepted->applied_mask = ((u16)msg[8] << 8) | msg[9];
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

int omci_onu_input(struct omci_onu *o, const u8 *msg, unsigned int len, u8 *resp)
{
	return omci_onu_input_ex(o, msg, len, resp, NULL);
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
