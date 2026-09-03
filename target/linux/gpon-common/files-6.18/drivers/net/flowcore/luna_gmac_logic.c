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
