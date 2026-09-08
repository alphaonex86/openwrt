// SPDX-License-Identifier: GPL-2.0-only
/* See luna_gmac_logic.h.  The two pack functions are MOVED verbatim from
 * rtl9602c_l34_logic.c (only the names changed, rtl9602c_ -> luna_gmac_,
 * because luna_eth.c is the proven second caller); the two RX verdicts are
 * SPLIT out of luna_eth.c's eth_rx() with the extracted expressions kept
 * byte-identical and every deviation declared at the function.
 */
#include <linux/types.h>

#include "luna_gmac_logic.h"

/* R_RxDesNum field packing: RX ring0 size + the flow-control ON/OFF
 * thresholds, in the GMAC's field layout.  Family history of this ONE
 * expression: spelled in rtl9602c_hw_program(), again in the legacy path of
 * rtl9602c_eth_open() (both now call here), and a THIRD time inline in
 * luna_eth.c's eth_hw_program() -- where the neighbouring R_RxCDO store was
 * the measured 2026-08-23 16-bit-store defect ("a ring of size ZERO").  One
 * spelling now. */
u32 luna_gmac_rxdesnum_pack(unsigned int ring_size, unsigned int th_on,
			    unsigned int th_off)
{
	return ((ring_size - 1) & 0xff) << 24 | (th_on & 0xff) << 16 |
	       (th_off & 0xff) << 8 | (((ring_size - 1) >> 8) & 0xf) << 4;
}

/* R_RxCDO field packing (same three-spelling history as
 * luna_gmac_rxdesnum_pack): RxRingSize[15:8] low byte + [7:4] the high
 * nibble; RxCDO[31:16] left 0 (hardware-owned, the vendor RMWs to preserve
 * it -- that RMW is the SHELL's, it needs a read). */
u32 luna_gmac_rxcdo_pack(unsigned int ring_size)
{
	return ((ring_size - 1) & 0xff) << 8 |
	       (((ring_size - 1) >> 8) & 0xf) << 4;
}

/*
 * Bad-frame verdict, SPLIT from luna_eth.c eth_rx(): is a returned RX
 * descriptor discarded?  @err_mask is the shell's RXD_CRCERR | RXD_RCDF --
 * passed, not re-spelled, exactly the mask rtl9602c_rx_frame_bad takes.
 * (DONE 2026-09-03: BIT(24) had carried two shell names, RXD_DMAERR and
 * RXD_RCDF; the surviving name is RXD_RCDF, in luna_eth_regs.h beside
 * RXD_CRCERR, because RCDF is the silicon's own name in the vendor NIC
 * driver while DMAERR appears in no vendor source.)
 * @hdr_floor is the shell's (u32)rx_prefix + ETH_HLEN, and
 * `<=` is the shell's own spelling, kept verbatim.
 *
 * ★ DELIBERATELY NOT MERGED with rtl9602c_rx_frame_bad, and the difference is
 * the point of them sitting in one tier: the 9602C shell bounds at the minimum
 * Ethernet frame (`len < 60 + cpu_prefix`, min accepted 62 with its fixed
 * 2-byte prefix) while this shell accepts anything longer than prefix + a bare
 * Ethernet header (min accepted 17 at the default rx_prefix=2), because
 * rx_prefix is a LIVE-TUNABLE bring-up param on a die whose RX framing was
 * HW-uncertain on first contact.  Forcing either bound onto the other shell is
 * a behaviour change nobody measured; the disagreement is now visible in one
 * file instead of hidden in two.
 */
bool luna_gmac_rx_frame_bad(u32 opts1, u32 err_mask, u32 len,
			    u32 hdr_floor, u32 buf_size)
{
	return (opts1 & err_mask) || len <= hdr_floor || len > buf_size;
}

/*
 * In-band switch CPU-tag classifier, SPLIT from luna_eth.c eth_rx(): after
 * the front prefix is stripped, is a raw 0x8899 Realtek control tag still
 * sitting between SA and ethertype?  (With CTEN_RX the MAC strips it in
 * hardware; this catches the raw-tag shape.  The luna_eth.c dump note records
 * the branch has NEVER fired on the G24W -- every captured frame was already
 * [DA][SA][type] -- so this predicate is also the witness that would say if
 * that ever changes.)  @tag_len is the shell's RTL_CPU_TAG_LEN (passed, not
 * re-spelled); the 12 is 2 * ETH_ALEN re-expressed, the declared deviation
 * this tier already uses (etherdevice.h is not includable here).  The excision
 * memmove and skb_pull stay in the shell: they are buffer surgery, not a
 * decision.  No 9602C counterpart exists to merge with: that shell never
 * sees a raw in-band tag (its trap-tag path is descriptor-based).
 */
bool luna_gmac_rx_cpu_tag_present(const u8 *data, u32 len,
				  unsigned int tag_len)
{
	return len > 12 + tag_len &&	/* 12 = 2 * ETH_ALEN (DA + SA) */
	       data[12] == 0x88 && data[13] == 0x99;
}

/*
 * ─── CPU-side OMCI (OMCC) datapath, family tier ────────────────────────────
 *
 * WHY THESE THREE ARE HERE AND NOT IN luna_eth.c: they are the whole DECISION
 * of the OMCI datapath -- which trapped frame is an OMCI frame, and how a
 * response is steered back onto the OMCC -- expressed as pure bit arithmetic
 * over the family's GMAC descriptor.  The shell keeps the ring, the DMA and
 * the responder call; nothing here reads a register.  That split is what lets
 * the encoders be exercised on x86 with no board.
 */

/*
 * Is this returned RX descriptor a DOWNSTREAM OMCI frame trapped to the CPU?
 *
 * ★ ONE CONDITION, DELIBERATELY: the reason code, exactly as the vendor's own
 * NIC RX hook decides it (`GMAC_RXINFO_REASON(pRxInfo) == omciRsn`, tier 3,
 * rtl86900/sdk/src/module/gpon/gponapi.c rtk_gponapp_omci_rx_wrapper).  The
 * reason lives at opts2[28:21] on every chip of this family -- the field is
 * `reason:8;//21~28` in both descriptor headers -- but its OMCI VALUE does not
 * (246 on the RTL9602C, 229 on the RTL9607C and RTL9603CVD), which is why it
 * arrives as @omci_reason from the per-chip table and is not spelled here.
 *
 * ★★ NOT MERGED WITH rtl9602c_rx_is_ds_omci(), and refusing to merge them is a
 * safety decision, not tidiness.  That verdict ORs in a CONTENT heuristic --
 * "byte 3 is 0x0a or 0x0b and byte 2 has bit7 clear" -- which its own shell
 * needs because on that die a trapped frame and a switch-routed one carry
 * different descriptor shapes.  On a die whose OMCI reason is a DIFFERENT
 * number, that same heuristic promotes ordinary LAN frames: any frame whose
 * fourth byte happens to be 0x0a would be handed to the G.988 responder and
 * dropped from the bridge.  A predicate that can only be right when its
 * neighbours are is not a shared predicate.
 *
 * @len is the DESCRIPTOR length, prefix included, and the bound pair is the
 * OOB guard the sibling verdict already carries: a caller must never index
 * into a buffer this says nothing about.
 */
bool luna_gmac_rx_is_ds_omci(bool trap_on, u32 opts2, u32 len,
			     unsigned int omci_reason,
			     unsigned int cpu_prefix, u32 buf_size)
{
	if (!trap_on || len < cpu_prefix + 8 || len > buf_size)
		return false;
	return ((opts2 >> 21) & 0xff) == omci_reason;
}

/*
 * US-OMCI TX steering, word2 (opts2): CPUTAG + the egress port mask.
 *
 * Tier 3, and the source is the vendor's OWN OMCI transmit
 * (rtl86900/sdk/src/module/gpon/gpon_omci.c, gpon_omci_tx): it memsets a
 * tx_info to zero and sets exactly CPUTAG_PSEL, DISLRN, KEEP, CPUTAG,
 * TX_PMASK = (1 << ponPort) and TX_DST_STREAM_ID = omcc_flow -- nothing else.
 *
 * ⚠ THE FIELD PLACEMENT IS THE RTL9607C GENERATION'S, WHICH IS NOT THE
 * RTL9602C'S, and reading across the two is the defect this family keeps
 * paying for.  Here (rtl86900/nicDriver/re8686_rtl9607c.h struct tx_info):
 *   opts2  cputag:31 | tx_portmask:16~26
 *   opts3  cputag_pri:24~26 | keep:23 | dislrn:21 | cputag_psel:20 |
 *          l34_keep:17 | extspa:13~15 | tx_dst_stream_id:0~6
 * On the RTL9602C keep/dislrn/psel live in OPTS1 and the stream id at
 * opts3[22:16] -- see TXD3_OMCI_9602C in rtl9602c_l34_logic.h, which is that
 * chip's and must never be reached for from this shell.
 *
 * ⚠ AND THERE IS NO EFID BIT ON THIS GENERATION.  The RTL9602C's proven stock
 * word2 is 0x80080000 = cputag|efid; the vendor's own transmit puts the EFID
 * pair behind `#if !(CONFIG_SDK_RTL9607C || CONFIG_SDK_RTL9603CVD)`, so the
 * word here is cputag|portmask and nothing more.  Copying the sibling's
 * 0x00080000 would set opts2 bit19, which is inside cvlan_vidl on this layout.
 */
u32 luna_gmac_omci_txd_word2(unsigned int pon_port)
{
	return 0x80000000u |				/* opts2[31] cputag	*/
	       ((1u << pon_port) & 0x7ffu) << 16;	/* opts2[26:16] pmask	*/
}

/*
 * US-OMCI TX steering, word3 (opts3): the cpu-tag behaviour bits + the OMCC
 * stream id.  Same source and same generation caveat as word2 above.
 *
 * cputag_psel (bit20) is the load-bearing one: it makes the GMAC DIRECT-TX the
 * cpu-tagged frame at the selected port instead of handing it to the L2
 * lookup, which is how an OMCI PDU -- a frame with no meaningful Ethernet
 * header at all -- reaches the PON port.  keep (bit23) stops the switch
 * rewriting it and dislrn (bit21) stops the OMCI PDU's first six bytes being
 * learned into the L2 table as if they were a source MAC.
 */
u32 luna_gmac_omci_txd_word3(unsigned int omcc_flow)
{
	return (1u << 23) |		/* opts3[23] keep		*/
	       (1u << 21) |		/* opts3[21] dislrn		*/
	       (1u << 20) |		/* opts3[20] cputag_psel	*/
	       (omcc_flow & 0x7fu);	/* opts3[6:0] tx_dst_stream_id	*/
}
