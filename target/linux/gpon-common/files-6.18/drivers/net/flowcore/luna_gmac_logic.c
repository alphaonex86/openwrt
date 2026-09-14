// SPDX-License-Identifier: GPL-2.0-only
/* See luna_gmac_logic.h.  The two pack functions are MOVED verbatim from
 * rtl9602c_l34_logic.c (names only, rtl9602c_ -> luna_gmac_); the RX verdicts
 * are SPLIT out of luna_eth.c's eth_rx() with the extracted expressions kept
 * byte-identical and every deviation declared at the function.
 */
#include <linux/types.h>
#include "luna_gmac_logic.h"

/* R_RxDesNum field packing: RX ring0 size plus the flow-control ON/OFF
 * thresholds, in the GMAC's field layout.  ⚠ The neighbouring R_RxCDO store was
 * the MEASURED 2026-08-23 16-bit-store defect, "a ring of size ZERO". */
u32 luna_gmac_rxdesnum_pack(unsigned int ring_size, unsigned int th_on,
			    unsigned int th_off)
{
	return ((ring_size - 1) & 0xff) << 24 | (th_on & 0xff) << 16 |
	       (th_off & 0xff) << 8 | (((ring_size - 1) >> 8) & 0xf) << 4;
}

/* R_RxCDO field packing: RxRingSize[15:8] low byte + [7:4] the high nibble.
 * RxCDO[31:16] is left 0 — hardware-owned, and the vendor RMWs to preserve it.
 * That RMW is the SHELL's, because it needs a read. */
u32 luna_gmac_rxcdo_pack(unsigned int ring_size)
{
	return ((ring_size - 1) & 0xff) << 8 |
	       (((ring_size - 1) >> 8) & 0xf) << 4;
}

/* Bad-frame verdict, SPLIT from luna_eth.c eth_rx(): is a returned RX
 * descriptor discarded?  @err_mask is the shell's RXD_CRCERR | RXD_RCDF and
 * @hdr_floor its (u32)rx_prefix + ETH_HLEN, both passed rather than re-spelled;
 * `<=` is the shell's own spelling, kept verbatim.
 * ⚠ RXD_RCDF is BIT(24)'s surviving name — RCDF is the silicon's own name in
 *   the vendor NIC driver, while DMAERR appears in no vendor source.
 * ★ DELIBERATELY NOT MERGED with rtl9602c_rx_frame_bad, and the two sitting in
 *   one tier is the point: that shell bounds at the minimum Ethernet frame
 *   (min accepted 62) while this one accepts anything longer than prefix + a
 *   bare Ethernet header (min accepted 17 at rx_prefix=2), because rx_prefix is
 *   a LIVE-TUNABLE bring-up param on a die whose RX framing was HW-uncertain on
 *   first contact.  Forcing either bound onto the other is a behaviour change
 *   nobody measured; the disagreement is now visible in one file instead of
 *   hidden in two. */
bool luna_gmac_rx_frame_bad(u32 opts1, u32 err_mask, u32 len,
			    u32 hdr_floor, u32 buf_size)
{
	return (opts1 & err_mask) || len <= hdr_floor || len > buf_size;
}

/* In-band switch CPU-tag classifier, SPLIT from luna_eth.c eth_rx(): after the
 * front prefix is stripped, is a raw 0x8899 Realtek control tag still sitting
 * between SA and ethertype?  With CTEN_RX the MAC strips it in hardware, so this
 * catches the raw-tag shape — and the branch has NEVER fired on the G24W, every
 * captured frame being already [DA][SA][type], which makes this predicate the
 * witness that would say if that ever changes.  @tag_len is the shell's
 * RTL_CPU_TAG_LEN, passed not re-spelled.  The excision memmove and skb_pull
 * stay in the shell: buffer surgery, not a decision.  No 9602C counterpart
 * exists to merge with — that shell's trap-tag path is descriptor-based. */
bool luna_gmac_rx_cpu_tag_present(const u8 *data, u32 len,
				  unsigned int tag_len)
{
	return len > 12 + tag_len &&	/* 12 = 2 * ETH_ALEN (DA + SA) */
	       data[12] == 0x88 && data[13] == 0x99;
}

/* ─── CPU-side OMCI (OMCC) datapath, family tier ────────────────────────────
 * These three are the whole DECISION of the OMCI datapath — which trapped frame
 * is an OMCI frame, and how a response is steered back onto the OMCC — as pure
 * bit arithmetic over the family's GMAC descriptor.  The shell keeps the ring,
 * the DMA and the responder call; nothing here reads a register, which is what
 * lets the encoders be exercised on x86 with no board. */

/* Is this returned RX descriptor a DOWNSTREAM OMCI frame trapped to the CPU?
 *
 * ★ ONE CONDITION, DELIBERATELY: the reason code, exactly as the vendor's own
 *   NIC RX hook decides it.  The reason lives at opts2[28:21] on every chip of
 *   this family, but its OMCI VALUE does not — 246 on the RTL9602C, 229 on the
 *   RTL9607C and RTL9603CVD — which is why it arrives as @omci_reason from the
 *   per-chip table and is not spelled here.
 * ★★ NOT MERGED WITH rtl9602c_rx_is_ds_omci(), and that is a safety decision:
 *    that verdict ORs in a CONTENT heuristic ("byte 3 is 0x0a or 0x0b and byte 2
 *    has bit7 clear") which its own shell needs because on that die a trapped
 *    frame and a switch-routed one carry different descriptor shapes.  On a die
 *    whose OMCI reason is a DIFFERENT number, that heuristic promotes ordinary
 *    LAN frames: any frame whose fourth byte happens to be 0x0a would be handed
 *    to the G.988 responder and dropped from the bridge.
 * @len is the DESCRIPTOR length, prefix included; the bound pair is the OOB
 * guard, because a caller must never index into a buffer this says nothing
 * about. */
bool luna_gmac_rx_is_ds_omci(const struct luna_rx_layout *rxl,
			     bool trap_on, u32 opts2, u32 opts3, u32 len,
			     unsigned int omci_reason,
			     unsigned int cpu_prefix, u32 buf_size)
{
	if (!trap_on || len < cpu_prefix + 8 || len > buf_size)
		return false;
	return luna_gmac_rx_reason(rxl, opts2, opts3) == omci_reason;
}

/* US-OMCI TX steering, word2 (opts2): CPUTAG + the egress port mask.  The
 * vendor's own OMCI transmit zeroes a tx_info and sets exactly CPUTAG_PSEL,
 * DISLRN, KEEP, CPUTAG, TX_PMASK = (1 << ponPort) and TX_DST_STREAM_ID —
 * nothing else.
 *
 * ⚠ THE FIELD PLACEMENT IS THE RTL9607C GENERATION'S, NOT THE RTL9602C'S, and
 *   reading across the two is the defect this family keeps paying for.  Here:
 *     opts2  cputag:31 | tx_portmask:16~26
 *     opts3  cputag_pri:24~26 | keep:23 | dislrn:21 | cputag_psel:20 |
 *            l34_keep:17 | extspa:13~15 | tx_dst_stream_id:0~6
 *   On the RTL9602C keep/dislrn/psel live in OPTS1 and the stream id at
 *   opts3[22:16] — that chip's fact, never reached for from this shell.
 * ⚠ AND THERE IS NO EFID BIT ON THIS GENERATION.  The RTL9602C's proven stock
 *   word2 is 0x80080000 = cputag|efid, and the vendor puts the EFID pair behind
 *   an #if excluding the 9607C/9603CVD.  Copying the sibling's 0x00080000 would
 *   set opts2 bit19, which is inside cvlan_vidl on this layout. */
u32 luna_gmac_cputag_txd_pmask(u32 port_mask)
{
	return 0x80000000u |			/* opts2[31] cputag	*/
	       (port_mask & 0x7ffu) << 16;	/* opts2[26:16] pmask	*/
}

u32 luna_gmac_cputag_txd_word2(unsigned int pon_port)
{
	return luna_gmac_cputag_txd_pmask(1u << pon_port);
}

/* Cpu-tag direct-TX steering, word3 (opts3): the behaviour bits + the DS/US
 * stream id.  Same source and same generation caveat as word2 above.
 *
 * cputag_psel (bit20) is the load-bearing one: it makes the GMAC DIRECT-TX the
 * cpu-tagged frame at the selected port instead of handing it to the L2 lookup,
 * which is how an OMCI PDU — a frame with no meaningful Ethernet header at all
 * — reaches the PON port.  keep (bit23) stops the switch rewriting it, and
 * dislrn (bit21) stops the PDU's first six bytes being learned into the L2
 * table as if they were a source MAC.
 *
 * @stream_id is the ONLY thing that differs between the OMCC and the WAN data
 * GEM, which is the whole reason this is one function: the US-NIC classifies a
 * transmitted frame by this field and never by its content.
 * ⚠ dislrn IS KEPT FOR WAN DATA TOO, and that is not an oversight carried over
 *   from the OMCI caller: a US user-data frame DOES have a real source MAC, and
 *   learning it would install the CPU port as the owner of an address the OLT
 *   sees arriving from the fibre. */
u32 luna_gmac_cputag_txd_word3(unsigned int stream_id)
{
	return (1u << 23) |		/* opts3[23] keep		*/
	       (1u << 21) |		/* opts3[21] dislrn		*/
	       (1u << 20) |		/* opts3[20] cputag_psel	*/
	       (stream_id & 0x7fu);	/* opts3[6:0] tx_dst_stream_id	*/
}

/* The switch port a received frame ingressed on: rx opts3[19:16].
 * ★ TIER 3 FOR BOTH DIES THIS FILE SERVES, and that had been recorded as tier 4.
 *   The vendor NIC driver is named re8686_rtl9607c.{c,h} but its own build gate
 *   compiles it for CONFIG_RTL9603CVD_SERIES as well, `struct rx_info` sits
 *   outside every chip conditional, and it spells `src_port_num:4;//16~19`.  So
 *   this is the RTL9603CVD's own NIC source; the file is merely named after its
 *   elder.
 * ⚠ THE FIELD POSITION IS ESTABLISHED; WHAT ARRIVES IN IT IS NOT.  On the
 *   RTL9602C, downstream WAN data does NOT reach the GMAC carrying the PON
 *   switch port — it drains through the PON-IP NIC with src_port 0 and a
 *   separate descriptor signature, which is why that shell's demux has three
 *   disjuncts.  Whether this generation drains the same way is a question only
 *   the board answers. */
/* ★★★ THE RX DESCRIPTOR LAYOUT IS STATED HERE AND NOWHERE ELSE (2026-09-12).
 * Five sites spelled these shifts inline -- rtl9602c_eth.c, luna_eth.c twice,
 * and rtl9602c_l34_logic.c twice -- so the bit positions were five facts that
 * could disagree, and a repair would have been five edits with one forgotten.
 *
 * ⚠ AND THEY MAY BE WRONG. Stock's own rxinfo_debug @0x801ae880 decodes
 *   reason = (word3 >> 13) & 255 and source = word3 >> 28, against our
 *   reason = (opts2 >> 21) & 0xff and source = (opts3 >> 16) & 0xf. The two
 *   readings differ on a real frame: stock's ARP/neighbour-miss reason 21 puts
 *   0x2a000 in word3 low for LAN port 0, which our shift calls source 2 = PON.
 *   That is a candidate mechanism for the X111W's selective unicast loss -- a
 *   LAN frame to our own MAC steered to gpon0 while ordinary ARP (reason 0)
 *   stays on eth0 -- and it is NOT MEASURED on a failing packet yet.
 * ⇒ consolidating FIRST is the point: whichever way the descriptor/CPU-tag
 *   comparison lands, the change is one line here instead of five.
 */
const struct luna_rx_layout luna_rx_layout_rtl9602c = {
	.src_lsb = 28, .src_bits = 4,
	.rsn_in_opts3 = true, .rsn_lsb = 13, .rsn_bits = 8,
};

const struct luna_rx_layout luna_rx_layout_rtl9603cvd = {
	.src_lsb = 16, .src_bits = 4,
	.rsn_in_opts3 = false, .rsn_lsb = 21, .rsn_bits = 8,
};

/* ⚠ ~0u IS A REFUSAL, NOT A FALLBACK, AND THE CALLER MUST TREAT IT AS ONE.
 * It matches no port and no reason by construction, but a shell without a
 * payload path then classifies nothing, and any caller doing BIT(value) on it
 * shifts by UINT_MAX -- undefined behaviour. Every die this tree builds for
 * now declares a layout (the 9607C's is a DECLARED inheritance, see
 * luna_eth_regs.h), so this arm is unreachable today and exists to make a
 * future unlisted die fail loudly rather than decode with a sibling's map. */
unsigned int luna_gmac_rx_reason(const struct luna_rx_layout *l, u32 opts2,
				 u32 opts3)
{
	if (!l)
		return ~0u;
	return ((l->rsn_in_opts3 ? opts3 : opts2) >> l->rsn_lsb) &
		((1u << l->rsn_bits) - 1u);
}

unsigned int luna_gmac_rx_src_port(const struct luna_rx_layout *l, u32 opts3)
{
	if (!l)
		return ~0u;
	return (opts3 >> l->src_lsb) & ((1u << l->src_bits) - 1u);
}

/* Does this received frame belong to the WAN netdev rather than the LAN one?
 * ONE condition, deliberately: it ingressed on the fibre port.  That is the
 * shape the vendor's own RX takes on this generation — it indexes a
 * port-to-netdev table by exactly this field and by nothing else — and it is
 * the first of the RTL9602C verdict's three disjuncts.
 * ★ THE OTHER TWO ARE REFUSED UNTIL THIS DIE EARNS THEM.  rtl9602c_rx_wan_demux
 *   also accepts opts3[31:20] == 0x23e and a destination-MAC match.  The first
 *   is a MEASURED RTL9602C constant that decodes against THIS generation's
 *   opts3 layout as internal_priority 1, pon_sid 15, l3routing 1 — a live value
 *   combination, not a structural marker, and 15 is not a stream this driver
 *   installs.  The second would hand any frame addressed to the WAN MAC to the
 *   WAN netdev whatever port it arrived on, which is how a LAN-side spoof
 *   becomes WAN ingress.  A number measured on other silicon is not evidence
 *   about this one. */
bool luna_gmac_rx_is_wan(const struct luna_rx_layout *rxl, u32 opts3, unsigned int pon_port)
{
	return luna_gmac_rx_src_port(rxl, opts3) == pon_port;
}

/* The PON stream id (GEM flow) a fibre-ingress frame arrived on: opts3[28:22].
 * ★ ADDED 2026-09-11 BECAUSE THE FIRST BOOT PROVED THE PORT WAS NOT ENOUGH: the
 *   ingress-port ledger caught one frame on the fibre and it decoded as
 *   src_port 4, prio 7, sid 127 — and 127 is the OMCC this driver installs, so
 *   the frame was OMCI, not user data.
 * ⚠ IT IS A WITNESS, NOT A SECOND VERDICT, and the demux does not consult it:
 *   narrowing luna_gmac_rx_is_wan() to exclude the OMCC would rest on ONE
 *   sample, and the field's meaning on a PON-IP-DRAINED descriptor is not
 *   established.  The day two tiers agree on what it holds for drained data, a
 *   verdict may use it.
 * ⚠ AND THE SAME SEVEN BITS ARE extspa ON A NON-PON FRAME (the vendor header
 *   declares both readings over one union).  Ask luna_gmac_rx_is_wan() first:
 *   on this board's own capture the two LAN-ingress frames read sid 0 and the
 *   fibre one read 127, exactly what that overlay predicts. */
unsigned int luna_gmac_rx_pon_sid(u32 opts3)
{
	return (opts3 >> 22) & 0x7f;
}
