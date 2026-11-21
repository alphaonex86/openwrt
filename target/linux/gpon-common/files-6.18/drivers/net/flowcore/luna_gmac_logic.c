// SPDX-License-Identifier: GPL-2.0-only
/* TIER: FAMILY */
/* See luna_gmac_logic.h. The two pack functions are MOVED ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 10. */
#include <linux/types.h>
#include "luna_gmac_logic.h"

/* R_RxDesNum: the CPU's TAIL (the last slot it armed) plus the flow-control ON/OFF thresholds; the GMAC pauses its
 * switch port while it counts th_on or fewer slots armed from its position to the tail. Stock rewrites the tail
 * after every arm and so does the boot loader (live stock: tail 0x3da at position 0x3db).
 * ⚠ The neighbouring R_RxCDO store was the MEASURED 2026-08-23 16-bit-store defect, "a ring of size ZERO". */
u32 luna_gmac_rxdesnum_pack(unsigned int cpu_tail, unsigned int th_on,
			    unsigned int th_off)
{
	return (cpu_tail & 0xff) << 24 | (th_on & 0xff) << 16 |
	       (th_off & 0xff) << 8 | ((cpu_tail >> 8) & 0xf) << 4;
}

/* The last slot armed when `armed` slots from `head` hold a buffer, in a ring of `n`. */
unsigned int luna_gmac_rx_tail(unsigned int head, unsigned int armed, unsigned int n)
{
	return (head + armed - 1) % n;
}

/* What a 16-bit GMAC counter that wraps and is never cleared counted since `*last`, which takes `now`. Exact while
 * it counts under 65536 between two reads. */
u16 luna_gmac_cnt16_delta(u16 now, u16 *last)
{
	u16 d = now - *last;

	*last = now;
	return d;
}

/* R_RxCDO field packing: RxRingSize [15:8] low byte + RxRingSize_h [3:0] the high nibble (live stock 0xff03 =
 * 1024 slots; a nibble at [7:4] does not hold: a 1024 ring read back 0xff00, 256 slots declared).
 * RxCDO[31:16] is left 0 — hardware-owned, and the vendor RMWs to preserve it.
 * That RMW is the SHELL's, because it needs a read. */
u32 luna_gmac_rxcdo_pack(unsigned int ring_size)
{
	return ((ring_size - 1) & 0xff) << 8 | (((ring_size - 1) >> 8) & 0xf);
}

/* The ring size a R_RxCDO word declares, whoever wrote it: 1..4096. */
unsigned int luna_gmac_rxcdo_entries(u32 rxcdo)
{
	return ((rxcdo >> 8) & 0xff) + ((rxcdo & 0xf) << 8) + 1;
}

/* May a TX frame be DMA'd from skb->data in place? Only a LINEAR frame of at
 * least the minimum length: a fragmented skb needs the linear copy (sec 47)
 * and a runt needs the zero-pad the copy slot provides. */
bool luna_gmac_tx_zero_copy(bool nonlinear, unsigned int len,
			    unsigned int min_len)
{
	return !nonlinear && len >= min_len;
}

/* Bad-frame verdict, SPLIT from luna_eth.c eth_rx(): is a ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 1. */
bool luna_gmac_rx_frame_bad(u32 opts1, u32 err_mask, u32 len,
			    u32 hdr_floor, u32 buf_size)
{
	return (opts1 & err_mask) || len <= hdr_floor || len > buf_size;
}

/* In-band switch CPU-tag classifier, SPLIT from luna_eth.c ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 2. */
bool luna_gmac_rx_cpu_tag_present(const u8 *data, u32 len,
				  unsigned int tag_len)
{
	return len > 12 + tag_len &&	/* 12 = 2 * ETH_ALEN (DA + SA) */
	       data[12] == 0x88 && data[13] == 0x99;
}

/* ─── CPU-side OMCI (OMCC) datapath, family tier ... -- dev/MEASURED-luna_gmac_logic.c.md sec 3. */
bool luna_gmac_rx_is_ds_omci(const struct luna_rx_layout *rxl,
			     bool trap_on, u32 opts2, u32 opts3, u32 len,
			     unsigned int omci_reason,
			     unsigned int cpu_prefix, u32 buf_size)
{
	if (!trap_on || len < cpu_prefix + 8 || len > buf_size)
		return false;
	return luna_gmac_rx_reason(rxl, opts2, opts3) == omci_reason;
}

/* US-OMCI TX steering, word2 (opts2): CPUTAG + the egress ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 4. */
u32 luna_gmac_cputag_txd_pmask(u32 port_mask)
{
	return 0x80000000u |			/* opts2[31] cputag	*/
	       (port_mask & 0x7ffu) << 16;	/* opts2[26:16] pmask	*/
}

u32 luna_gmac_cputag_txd_word2(unsigned int pon_port)
{
	return luna_gmac_cputag_txd_pmask(1u << pon_port);
}

/* Cpu-tag direct-TX steering, word3 (opts3): the behaviour ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 5. */
u32 luna_gmac_cputag_txd_word3(unsigned int stream_id)
{
	return (1u << 23) |		/* opts3[23] keep		*/
	       (1u << 21) |		/* opts3[21] dislrn		*/
	       (1u << 20) |		/* opts3[20] cputag_psel	*/
	       (stream_id & 0x7fu);	/* opts3[6:0] tx_dst_stream_id	*/
}

/* The switch port a received frame ingressed on: rx ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 6. */
const struct luna_rx_layout luna_rx_layout_rtl9602c = {
	.src_lsb = 28, .src_bits = 4,
	.rsn_in_opts3 = true, .rsn_lsb = 13, .rsn_bits = 8,
};

const struct luna_rx_layout luna_rx_layout_rtl9603cvd = {
	.src_lsb = 16, .src_bits = 4,
	.rsn_in_opts3 = false, .rsn_lsb = 21, .rsn_bits = 8,
};

/* ⚠ ~0u IS A REFUSAL, NOT A FALLBACK, AND THE CALLER MUST ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 7. */
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

/* Does this received frame belong to the WAN netdev rather ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 8. */
bool luna_gmac_rx_is_wan(const struct luna_rx_layout *rxl, u32 opts3, unsigned int pon_port)
{
	return luna_gmac_rx_src_port(rxl, opts3) == pon_port;
}

/* The PON stream id (GEM flow) a fibre-ingress frame arrived ...
 * dev/MEASURED-luna_gmac_logic.c.md sec 9. */
unsigned int luna_gmac_rx_pon_sid(u32 opts3)
{
	return (opts3 >> 22) & 0x7f;
}
