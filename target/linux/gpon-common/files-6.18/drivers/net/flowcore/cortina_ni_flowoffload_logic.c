// SPDX-License-Identifier: GPL-2.0-only
/* See cortina_ni_flowoffload_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>
#include <linux/bitrev.h>
#include <linux/crc32.h>
#include <linux/in.h>		/* IPPROTO_TCP */
#include <linux/kernel.h>
#include <linux/string.h>

#include "cortina_ni_flowoffload_logic.h"

/* Steps 3-4 of the HW recipe (host-fuzz reference for the SW CRC path; the
 * runtime hash uses the SWO engine - see cn_l3e_key_hash). */
void cn_l3e_bitrev_key(u32 *w, int n_words)
{
	int i;
	u32 t;

	for (i = 0; i < n_words / 2; i++) {
		t = w[i];
		w[i] = bitrev32(w[n_words - 1 - i]);
		w[n_words - 1 - i] = bitrev32(t);
	}
	if (n_words & 1)
		w[i] = bitrev32(w[i]);
}

u32 cn_l3e_crc32(const u8 *p, size_t len)
{
	/* reflected CRC-32 (Ethernet poly), seed ~0, reflected output, no
	 * final xor - the engine's convention */
	return bitrev32(crc32_le(~0u, p, len));
}

u16 cn_l3e_crc16(const u8 *p, size_t len)
{
	/* reflected CRC-16/CCITT (poly 0x8408), seed 0xffff, reflected
	 * output, no final xor */
	u16 crc = 0xffff;
	int i;

	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
	}
	return bitrev8(crc & 0xff) << 8 | bitrev8(crc >> 8);
}

/*
 * ★ HW hash recipe - FULLY RECOVERED 2026-07-18 (tier-2 Ghidra decomp of
 * stock hash_value_calculate() @0x11e0 + tier-1 single-bit SWO sweeps on the
 * live engine).  The lookup HW computes, for a 92-byte key and a 256-bit
 * mask-table entry:
 *   1. mask-apply, per FIELD (not per bit): field &= ~mask_field
 *      (mask bit 1 = EXCLUDE the field; 0 = keep it).
 *   2. HW-derived nonlinear FLAG bits (address-equal / zero / prefix-length
 *      checks) are folded into the reduced tuple - so even an all-zero key
 *      does NOT hash to CRC(zeros): the transform is NOT linear in the key
 *      bytes (proven: SWO(0)=0x7fc13ab0 != crc32(92*0x00)=0x3d4ad918).
 *   3. bit-reverse the whole key: 23 u32 words, bitrev32 EACH word AND
 *      reverse the WORD ORDER (cn_l3e_bitrev_key does exactly this).
 *   4. CRC32: reflected poly 0xEDB88320, seed ~0, final = bitrev32 (NO xor).
 *      CRC16: reflected poly 0x8408, seed 0xffff, final = bitrev16 (NO xor).
 *      (cn_l3e_crc32 / cn_l3e_crc16 implement 3+4 exactly - confirmed against
 *      the crctable_32/crctable_ccitt16 tables in the stock .ko.)
 *   5. optional per-profile CRC16 rotate/xor - gated on table_id==1 and
 *      indexed by hash_key_select[ctrl_set_id]; hash_key_select is .bss and
 *      stays 0 in stock (the <0x40 rule keeps CRC32 standard) => rotate is
 *      identity in practice.
 *
 * The nonlinear FLAG derivation (step 2) + the per-field mask-apply (step 1)
 * are ~1100 lines of stock logic; rather than transliterate them (fragile,
 * and the mask VALUES are external to ca-ne.ko anyway), the runtime hash
 * DRIVES THE ON-CHIP SWO CRC ENGINE - the SAME engine the lookup path uses -
 * so the {crc32,crc16} are IDENTICAL to what a parsed packet hashes to, by
 * construction (this is exactly what stock's own runtime add path does; the
 * SW CRC above is stock's host-fuzz-only fallback).  cn_l3e_crc32/16 +
 * cn_l3e_bitrev_key are kept as the host-fuzz reference for step 3/4.
 *
 * ★ KEY LAYOUT (divergence-A fix, 2026-07-18): the SWO engine hashes the
 * 128-byte HW HDR_I descriptor, NOT our 92-byte cn_l3e_key.  cn_l3e_key_hash
 * therefore converts the SW key to HDR_I (cn_l3e_build_hdri) and feeds all 32
 * HDR_I words to the SWO.  The profile id that stock aal_hash_value_cal_part_0
 * stamps into the SW key's ctrl_set_id lands in HDR_I as t2_ctrl (07f has no
 * separate table_id field; its SW-key table_id is always mask-zeroed pre-CRC).
 * (Feeding the raw 92-byte key - the previous bug - put every 5-tuple field in
 * a masked-out HDR_I position, so the CRC was constant; see cn_l3e_build_hdri.)
 */
u32 cn_l3e_proc_parse_ip(const char *s)
{
	u8 b[4];
	unsigned int v;

	if (sscanf(s, "%hhu.%hhu.%hhu.%hhu", &b[0], &b[1], &b[2], &b[3]) == 4)
		return ((u32)b[0] << 24) | ((u32)b[1] << 16) |
		       ((u32)b[2] << 8) | b[3];
	if (kstrtouint(s, 0, &v) == 0)
		return v;
	return 0;
}

/* set `width` bits at LSB-first bit offset `off` in a little-endian buffer */
void cn_l3e_hdri_set(u8 *h, unsigned int off, unsigned int width, u64 val)
{
	unsigned int i;

	for (i = 0; i < width; i++, off++)
		if ((val >> i) & 1)
			h[off >> 3] |= 1u << (off & 7);
		/* buffer is pre-zeroed, so only 1-bits need writing */
}

/*
 * ★ SW-tuple -> HW HDR_I packing (divergence-A fix, 2026-07-18).
 *
 * The SWO/lookup engine hashes the 128-byte L3FE_HDR_I descriptor, NOT our
 * 92-byte cn_l3e_key (the aal_hash_key_t SW shadow).  Build the HDR_I with
 * every flow field at its HW bit position so the SWO CRC equals what the
 * classify/parse stage produces for a matching packet.  Only the fields the
 * 5-tuple mask (mask 0) leaves live matter to the CRC; the rest stay zero.
 *
 * Faithful to the stock HDR_I build (aal_hash_crc_sw_hw_calc_check): the
 * 5-tuple + ip_l4_type + ip_vld/ip_ver + the profile id stamp (t2_ctrl).  DA/SA
 * are 128-bit fields (4 consecutive 32-bit words) so an IPv6 key packs the
 * upper 96 bits too; an IPv4 key leaves them zero (masked out under mask 0).
 */
void cn_l3e_build_hdri(const struct cn_l3e_key *k, int profile,
		       u32 words[CN_L3E_HDRI_WORDS])
{
	u8 h[CN_L3E_HDRI_BYTES] = { 0 };

	/* L4 */
	cn_l3e_hdri_set(h, CN_HDRI_L4_DP, 16, k->l4_dport);
	cn_l3e_hdri_set(h, CN_HDRI_L4_SP, 16, k->l4_sport);
	/* IP DA/SA - 4x32b each, LSW first (IPv4 = word 0 only) */
	cn_l3e_hdri_set(h, CN_HDRI_IP_DA0 + 0,  32, k->ip_da_0);
	cn_l3e_hdri_set(h, CN_HDRI_IP_DA0 + 32, 32, k->ip_da_1);
	cn_l3e_hdri_set(h, CN_HDRI_IP_DA0 + 64, 32, k->ip_da_2);
	cn_l3e_hdri_set(h, CN_HDRI_IP_DA0 + 96, 32, k->ip_da_3);
	cn_l3e_hdri_set(h, CN_HDRI_IP_SA0 + 0,  32, k->ip_sa_0);
	cn_l3e_hdri_set(h, CN_HDRI_IP_SA0 + 32, 32, k->ip_sa_1);
	cn_l3e_hdri_set(h, CN_HDRI_IP_SA0 + 64, 32, k->ip_sa_2);
	cn_l3e_hdri_set(h, CN_HDRI_IP_SA0 + 96, 32, k->ip_sa_3);
	/* IP proto / L4 type / validity */
	cn_l3e_hdri_set(h, CN_HDRI_IP_L4_TYPE, 3, k->ip_l4_type);
	cn_l3e_hdri_set(h, CN_HDRI_IP_PROTO,   8, k->ip_protocol);
	cn_l3e_hdri_set(h, CN_HDRI_IP_VER,     1, k->ip_ver);
	cn_l3e_hdri_set(h, CN_HDRI_IP_VLD,     1, k->ip_vld);
	/* profile id stamp -> HDR_I t2_ctrl (mirrors stock; masked under mask 0) */
	cn_l3e_hdri_set(h, CN_HDRI_T2_CTRL,    4, profile & 0xf);

	memcpy(words, h, CN_L3E_HDRI_BYTES);
}

/**
 * cn_fib_field() - read one field out of a raw 32-byte FIB entry BY BIT NUMBER.
 * @fib:   the entry as it sits in the table the engine reads.
 * @bit:   the field's first bit, LSB-first within the entry.
 * @width: its width in bits (<= 64).
 *
 * ★ DELIBERATELY NOT struct cn_l3e_act.  A readback through the same struct
 * that wrote the bytes is self-agreement, not verification: our WRITE and our
 * READBACK once shared one wrong bit-packed offset and "matched stock" through
 * three digs (the SID2QID saga), and only a read at an independently-derived
 * offset exposed it.  The literal numbers this is called with are the ones the
 * live STOCK oracle solved for on this silicon (top_vid@145, top_tpid_enc@157,
 * vlan_cnt@160, vlan_vld@162), which is a different tier from our header.
 */
u64 cn_fib_field(const void *fib, u32 bit, u32 width)
{
	const u8 *b = fib;
	u64 v = 0;
	u32 i;

	for (i = 0; i < width; i++)
		v |= (u64)((b[(bit + i) >> 3] >> ((bit + i) & 7)) & 1) << i;
	return v;
}

/*
 * ★★ THE PPPoE-WAN LEG GATE - a PURE predicate (functional core: no MMIO, no
 * state, no allocation), so the policy deciding which legs of a PPPoE flow may
 * enter hardware is host-testable and cannot drift silently.
 *
 * @pppoe_mode  the hw_pppoe module param.
 * @ds_leg      this rule is the DS (WAN->LAN) reply direction.
 * @rule_sid    the FLOW_ACTION_PPPOE_PUSH sid carried by THIS rule (0 = none).
 *              nf_flow_table emits the push only on the leg whose OTHER tuple
 *              holds the encap, i.e. on the US leg of a PPPoE WAN - so a DS rule
 *              legitimately carries 0 even for a PPPoE flow.
 * @armed_sid   the live session shadow (data_pppoe_session) = "this WAN IS
 *              PPPoE", which is the only way the DS leg can know.
 *
 * ★★ 2026-07-25 ROOT-CAUSE FIX - GAP-2 re-diagnosed, and the DS leg re-opened.
 * The DS leg used to be refused UNCONDITIONALLY once a session was armed, on the
 * theory that a DS action "would have to POP the 8-byte session header, which
 * this action shape cannot express".  The board had already refuted that, inside
 * the hw_pppoe=0 benchmark itself: with hw_pppoe=0 nothing arms the shadow, so
 * the gate never fired on the DS leg, the reply rule WAS installed, and
 * downstream ran 934.2 Mbps end-to-end over that same PPPoE WAN while the CPU
 * saw essentially no punted session frames (the punt ledger: seen=92, of which
 * ctrl=84, over the whole run - i.e. ~8 data frames, so the reply traffic was
 * NOT going through the CPU).
 *
 * ★ And the stock oracle says the same, at tier-2 PROVEN.  RE of the shipped
 * flow-cache manager (the module that decides and installs stock's HW flows and
 * egress interface entries) shows it builds this same 32-bit egress L3-IF word
 * from a 4-valued per-interface PPPoE action, with exactly three encodings:
 *     KEEP   -> pppoe_set 0, pppoe_vld 0            (leave the layer alone)
 *     ADD    -> pppoe_set 1, pppoe_vld 1, session   (MODIFY packs identically)
 *     REMOVE -> pppoe_set 1, pppoe_vld 0            (a DISTINCT encoding, so
 *                                                    set=1/vld=0 is NOT "inert")
 * (the same three values that the sibling reference SDK names KEEP/ADD/MODIFY/
 * REMOVE).  So the DS pop is a property of the EGRESS INTERFACE word, and it is
 * the only place it can live: stock's per-flow hash action has just two inline
 * PPPoE bits and NO session-id field at all, and its flow-action builder never
 * writes those two bits in ANY mode - for either direction.  There is no
 * dedicated pop bit anywhere in that action (`pop_l3_*` is the IP-in-IP /
 * DS-Lite / 6RD outer-header decap, named by the engine's own drop-reason
 * strings - nothing to do with PPPoE).  The packet editor rebuilds the egress
 * encapsulation from the selected L3-IF word, so an entry that does not say
 * "carry PPPoE" emits no session header: an effective strip, with no pop bit.
 * Our LAN egress entry is the explicit REMOVE form (l3fe_l3if_entry() always
 * sets PPPOE_SET; stock's LAN interface uses KEEP) - both mean "no PPPoE out",
 * and ours is the one that ran at 934.2 Mbps.
 *
 * Nothing in the DS action or key is PPPoE-specific: the 5-tuple mask 8 excludes
 * all four PPPoE fields, so the entry matches on the INNER 5-tuple whatever the
 * encap (host-proven, Step 12g).
 *
 * Flipping hw_pppoe to 1 armed the shadow and thereby switched that working DS
 * leg OFF (`ds_refused` counted the refusals) - which is what collapsed
 * downstream from 934.2 to 242.9 Mbps: the reply traffic moved from the hash
 * engine onto the CPU punt path (the same ledger then counted 280708 punted
 * session frames, a ~3000x jump).  It cost the upstream too: with every DS frame
 * back on the CPU, every DS TCP flag byte passes nf_flow_state_check(), so one
 * FIN/RST - or one corrupted flag byte - tears the offload down inside the flap
 * window (`early_gone`) and the flow oscillates HW->SW, which is the 40.5 Mbps
 * upstream rather than a line-rate one.
 *
 * So a PPPoE WAN no longer refuses the DS leg.  What IS still refused, and why:
 *   - anything PPPoE while hw_pppoe=0 - the mode gate.  The SW fastpath forwards
 *     PPPoE correctly, and this keeps the default-OFF behaviour byte-identical;
 *   - a US rule with no push on a PPPoE WAN: it cannot express the encap, and
 *     installing it would put an un-encapsulated frame on the WAN;
 *   - a push on the DS leg: nf never emits one there, so it means an encap model
 *     we have not RE'd.
 */
enum cn_pppoe_leg_verdict cn_pppoe_leg_check(bool pppoe_mode, bool ds_leg,
					     u16 rule_sid, u16 armed_sid)
{
	if (ds_leg && rule_sid)
		return CN_PPPOE_LEG_UNEXPECTED_PUSH;
	if (!rule_sid && !armed_sid)
		return CN_PPPOE_LEG_OK;		/* IPoE WAN - nothing to decide */
	if (!pppoe_mode)
		return CN_PPPOE_LEG_MODE_OFF;
	if (!ds_leg && !rule_sid)
		return CN_PPPOE_LEG_NO_PUSH;
	return CN_PPPOE_LEG_OK;
}

/*
 * ★★ THE DISARM HALF OF GAP-3 - a PURE predicate (functional core: no MMIO, no
 * state), the exact mirror of the arm.
 *
 * The armed shadow (l3e->data_pppoe_session) is set LAZILY, by the first
 * offloaded US flow carrying a FLOW_ACTION_PPPOE_PUSH - nothing else arms it.
 * It used to be cleared by only three events: the WAN data path going away
 * (cortina_ni_gpon_data_path_set(gem_id = 0)), the hw_pppoe 1->0 edge, and the
 * manual `pppoe 0` control write.  NONE of them happens when the WAN is simply
 * reconfigured from PPPoE back to IPoE with the PON link left up - which is an
 * ordinary service change, not an exotic case.  The shadow then outlives its
 * session, cn_pppoe_leg_check() reads {no rule sid, a shadow} as NO_PUSH, and
 * EVERY upstream flow is refused for the rest of the boot.  Nothing breaks:
 * the WAN keeps working, on the CPU, which is why it survived so long.
 *
 * ★ MEASURED on this board, 2026-08-09, driving dhcp -> pppoe -> dhcp:
 * upstream 954.9 Mbps at 3.0 % CPU before the transition, 581.7 Mbps with one
 * core at 99.5 % after, 4 runs of 4.  The driver's own ledger on that boot read
 * `us_refused = 672` with `ds_refused = 0` and `unsupp = 672` - so the PPPoE US
 * gate accounted for the WHOLE unsupported-refusal count, to the unit.  The
 * downstream leg was never refused (cn_pppoe_leg_check returns OK for it), and
 * downstream indeed stayed accelerated: the collapse is upstream-only, which is
 * the second, independent prediction this mechanism makes.
 *
 * The disarm is therefore driven by the same evidence as the arm, and that
 * evidence is authoritative: nf_flow_table builds the US rule from the ACTUAL
 * forward path (pppoe_fill_forward_path), so a rule whose egress IS the WAN and
 * which carries no session id is the kernel stating that this WAN no longer has
 * one.  A live PPPoE WAN cannot produce that rule shape - if it could, the same
 * absent sid could never have armed the shadow in the first place.
 *
 * @ds_leg        the reply leg never carries a push, so it can say NOTHING
 *                about the WAN's encapsulation - only the US leg is evidence.
 * @egress_is_lan the redirect device is LAN-side, so this leg's egress is not
 *                the WAN and its lack of a session means nothing about it.
 * @rule_sid      the session THIS rule carries (its own PPPOE_PUSH, or the one
 *                resolved off a tagged WAN chain).  Non-zero = still PPPoE.
 * @armed_sid     the shadow.  Zero = there is nothing to disarm.
 *
 * ⚠ One deliberate consequence: an operator who armed the shadow by hand for a
 * manual install (`pppoe <sid>`) has it disarmed by the next auto US flow.  That
 * is the better of the two behaviours - the alternative is the manual arm
 * silently costing the whole box its upstream offload, which is this very bug.
 */
bool cn_pppoe_shadow_stale(bool ds_leg, bool egress_is_lan,
			   u16 rule_sid, u16 armed_sid)
{
	return !ds_leg && !egress_is_lan && armed_sid && !rule_sid;
}

/*
 * PURE predicate (functional core - no MMIO, no state, no allocation, host
 * fuzzable): classify ONE received frame.  Returns 0 when the frame is not a
 * PPPoE session frame at all, otherwise CN_PPPOE_PUNT_SESSION plus whatever is
 * wrong with it.  @exp_sid 0 = do not judge the session id.
 *
 * All wire reads are explicit byte math (one image runs big- and little-endian)
 * and every access is bounded by @len BEFORE it is made.
 */
u32 cn_pppoe_punt_classify(const u8 *f, unsigned int len, u16 exp_sid,
			   struct cn_pppoe_punt_info *pi)
{
	unsigned int hdr = 14, off;
	const u8 *pppoe, *ip, *tcp;
	u32 v = CN_PPPOE_PUNT_SESSION;

	memset(pi, 0, sizeof(*pi));
	/* Ethernet, optionally one VLAN tag, then the session ethertype. */
	if (len < hdr + 8)
		return 0;
	if (f[12] == 0x81 && f[13] == 0x00) {
		hdr += 4;
		if (len < hdr + 8)
			return 0;
	}
	if (f[hdr - 2] != 0x88 || f[hdr - 1] != 0x64)
		return 0;

	pppoe = f + hdr;
	pi->sid = ((u16)pppoe[2] << 8) | pppoe[3];
	pi->pppoe_len = ((u16)pppoe[4] << 8) | pppoe[5];
	pi->ppp_proto = ((u16)pppoe[6] << 8) | pppoe[7];

	if (exp_sid && pi->sid != exp_sid)
		v |= CN_PPPOE_PUNT_SID_BAD;

	/* 0x0021 = PPP-IPv4 (data).  Everything else is control (LCP, IPCP,
	 * PAP/CHAP, IPv6CP): counted, not judged - those frames carry no inner
	 * IPv4 header for the length identity below. */
	if (pi->ppp_proto != 0x0021)
		return v | CN_PPPOE_PUNT_CTRL;
	v |= CN_PPPOE_PUNT_DATA;	/* the frame the identities below judge */

	ip = pppoe + 8;
	off = hdr + 8;
	if (len < off + 20)
		return v | CN_PPPOE_PUNT_SHORT;
	pi->ip_ver = ip[0] >> 4;
	pi->ihl = (ip[0] & 0xf) * 4;
	pi->ip_len = ((u16)ip[2] << 8) | ip[3];

	/* ★ THE identity a correctly-encapsulated session frame must satisfy:
	 * the PPPoE length field covers the 2-byte PPP protocol plus the whole
	 * inner IP datagram.  An 8-byte shift breaks it. */
	if (pi->ip_ver != 4 || pi->ihl < 20 ||
	    pi->pppoe_len != (u16)(pi->ip_len + 2)) {
		v |= CN_PPPOE_PUNT_LEN_BAD;
		/*
		 * ★ SHAPE the malformation instead of leaving it to be guessed.
		 * Both tests are bounded by @len before any read, and both are
		 * pure - they only describe the bytes in hand.
		 *
		 * DBLENC: a whole second session header sits where the inner IP
		 * should be - {ver 1, type 1} in byte 0, code 0 (session) in
		 * byte 1, and PPP-IPv4 in its own protocol field.  That is a
		 * frame encapsulated twice, i.e. an ADD applied to a frame that
		 * already carried its header.
		 *
		 * SHIFT8: the frame becomes self-consistent when the inner IP is
		 * taken 8 bytes further in - an 8-byte insert between the session
		 * header and the IP header.  Accept either length convention:
		 * ip_len+2 if the PPPoE length field was NOT recomputed over the
		 * inserted bytes, ip_len+10 if it was.
		 */
		if (len >= off + 8 && ip[0] == 0x11 && ip[1] == 0x00 &&
		    (((u16)ip[6] << 8) | ip[7]) == 0x0021)
			v |= CN_PPPOE_PUNT_DBLENC;
		if (len >= off + 8 + 20 && (ip[8] >> 4) == 4) {
			u16 l2 = ((u16)ip[10] << 8) | ip[11];

			if (pi->pppoe_len == (u16)(l2 + 2) ||
			    pi->pppoe_len == (u16)(l2 + 10))
				v |= CN_PPPOE_PUNT_SHIFT8;
		}
		return v;
	}

	/* An 8-byte shift ALSO lands garbage in the TCP data-offset nibble, so
	 * check that independently - two witnesses for one malformation. */
	if (ip[9] == IPPROTO_TCP) {
		off += pi->ihl;
		if (len < off + 20)
			return v | CN_PPPOE_PUNT_SHORT;
		tcp = ip + pi->ihl;
		pi->tcp_doff = (tcp[12] >> 4) * 4;
		pi->tcp_flags = tcp[13];
		if (pi->tcp_doff < 20 ||
		    (unsigned int)pi->ihl + pi->tcp_doff > pi->ip_len)
			return v | CN_PPPOE_PUNT_TCP_BAD;
	}
	return v;
}
