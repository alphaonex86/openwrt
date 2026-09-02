/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * rtl9602c_l34_logic.h -- logic hoisted out of the rtl9602c_eth.c TU
 * (rtl9602c_l34.c is #included into it, so one pair serves the whole shell).
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _RTL9602C_L34_LOGIC_H
#define _RTL9602C_L34_LOGIC_H

#include <linux/types.h>

void l34_field_set(u32 *w, unsigned int lsp, unsigned int width, u32 val);
u16 l34_hash_out(bool is_tcp, u32 sip, u16 sport, u32 dip, u16 dport);
u16 l34_hash_in(bool is_tcp, u32 dip, u16 dport);

/* ---- hoisted from rtl9602c_eth.c (same shell TU) ---------------------- */

/* RX cpu-tag reason code = OMCI (opts2[28:21]); consumed by the DS-OMCI
 * classifier below.  Moved here from rtl9602c_eth.c with that classifier. */
#define RTL9602C_OMCI_REASON	246

/*
 * 9602C GMAC TX steering-descriptor facts, moved here from rtl9602c_eth.c so
 * the cross-chip trap that file documents exists ONCE, where a sibling chip's
 * table can visibly disagree with it:
 *
 *   9602C opts3/word3 layout (authoritative: the 9602C TX-descriptor field
 *   map): extspa[31:29] | tx_portmask[28:23] | tx_dst_stream_id[22:16] |
 *   rsvd | l34_keep[1] | ptp[0]
 *
 * The 9607C puts dst_stream_id at [6:0]; writing SID 64 there on 9602C landed
 * it in RESERVED bits (ignored), so the US-NIC never saw SID 64 (rxsid stayed
 * 0 -- the whole US-OMCI blocker).  extspa is left 0 for OMCI.
 *
 * GROUND TRUTH from a live working stock ref ONU (devmem of its OMCI TX ring
 * 0, 2026-06-11): every OMCI descriptor is word2=0x80080000, word3=0x02400000.
 * Decoding word3 in the 9602C layout: tx_portmask[28:23] = (1<<2) and
 * tx_dst_stream_id[22:16] = 64.  So the PON port for the GMAC tx_portmask is
 * 2, not 4 (the earlier guess).
 */
#define GMAC_PON_PORT		2	/* GMAC tx_portmask PON bit (stock = 1<<2 -> word3 0x02400000) */
#define TXD2_OMCI_CPUTAG	0x80000000u	/* opts2 bit31 cputag */
#define TXD2_OMCI_EFID		0x00080000u	/* opts2 bit19 efid (stock word2 = 0x80080000) */
#define TXD3_9602C_PMASK(p)	(((p) & 0x3Fu) << 23)	/* opts3[28:23] tx_portmask */
#define TXD3_9602C_DST_SID(s)	(((s) & 0x7Fu) << 16)	/* opts3[22:16] tx_dst_stream_id (the steering SID) */
#define TXD3_OMCI_9602C(sid)	(TXD3_9602C_PMASK(1u << GMAC_PON_PORT) | \
				 TXD3_9602C_DST_SID(sid))	/* = 0x08400000 for SID 64 */

void rtl9602c_wan_mac_add(u8 *out, const u8 *base, unsigned int add);
bool rtl9602c_rx_is_ds_omci(bool trap_on, u32 opts2, u32 opts3,
			    const u8 *data, u32 len, unsigned int pon_port,
			    unsigned int cpu_prefix, u32 buf_size);
bool rtl9602c_rx_wan_demux(u32 opts3, const u8 *dst, const u8 *wan_mac,
			   unsigned int pon_port);
unsigned int rtl9602c_omci_hwring(unsigned int omci_tx_ring);
bool rtl9602c_omci_doorbell(unsigned int omci_tx_ring,
			    unsigned int doorbell_ovr,
			    unsigned int *hwring, u32 *mask);
u32 rtl9602c_rxdesnum_pack(unsigned int ring_size, unsigned int th_on,
			   unsigned int th_off);
u32 rtl9602c_rxcdo_pack(unsigned int ring_size);
u32 rtl9602c_omci_txd_word2(u32 ovr);
u32 rtl9602c_omci_txd_word3(u32 ovr, unsigned int sid);

#endif /* _RTL9602C_L34_LOGIC_H */
