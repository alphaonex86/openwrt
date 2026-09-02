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
u32 l34_field_get(const u32 *w, unsigned int lsp, unsigned int width);
u16 l34_hash_out(bool is_tcp, u32 sip, u16 sport, u32 dip, u16 dport);
u16 l34_hash_in(bool is_tcp, u32 dip, u16 dport);

/* ---- L34/L2 entry LAYOUTS + their encoders --------------------------------
 * Moved from rtl9602c_l34.h / split out of rtl9602c_l34.c (2026-09-02): the
 * layout defines travel WITH their encoders (the TXD3_9602C_* precedent) so
 * every layout fact exists ONCE; the shell reaches them through this header.
 * {LSP, W} = LSB bit position within the packed entry and field width in
 * bits.  The data bank is significance-ordered (w[i] holds entry bits
 * [32*i+31 : 32*i]); a field's word index is LSP/32.  Hardware facts; the
 * naming/expression is original.
 *
 * Every encoder below writes into a CALLER-ZEROED word array (the shell's
 * on-stack `= { 0 }`): l34_field_set is read-modify-write, deliberately, so
 * the shell's rollback path (re-writing a zeroed entry) stays possible.
 * Registers, table types, word counts and the engine/L2 command plumbing
 * stay in the shell's rtl9602c_l34.h -- they are transport, not layout.
 */

/* NAPT_OUT (type 10, 1 word): outbound hash slot. The slot's table index is
 * itself the outbound hash; the word only points at the rewrite entry. */
#define L34_NAPT_HASHIN_IDX_LSP	0	/* -> NAPTR_IN entry index */
#define L34_NAPT_HASHIN_IDX_W	12
#define L34_NAPT_VALID_LSP	12
#define L34_NAPT_VALID_W	1
#define L34_NAPT_PRIVALID_LSP	13
#define L34_NAPT_PRIVALID_W	1
#define L34_NAPT_PRIORITY_LSP	14
#define L34_NAPT_PRIORITY_W	3

/* NAPTR_IN (type 9, 3 words): the inbound / rewrite entry. */
#define L34_NAPTR_INTIP_LSP	0	/* internal (LAN) host IP */
#define L34_NAPTR_INTIP_W	32
#define L34_NAPTR_INTPORT_LSP	32	/* internal host L4 port */
#define L34_NAPTR_INTPORT_W	16
#define L34_NAPTR_REMHASH_LSP	48	/* remote-peer confirm hash (type 2/3) */
#define L34_NAPTR_REMHASH_W	16
#define L34_NAPTR_EXTIPIDX_LSP	64	/* -> EXTIP slot (WAN src IP + next-hop) */
#define L34_NAPTR_EXTIPIDX_W	3
#define L34_NAPTR_EXTPORT_LSP	67	/* post-NAT (WAN) L4 port */
#define L34_NAPTR_EXTPORT_W	16
#define L34_NAPTR_TCP_LSP	83	/* 1 = TCP, 0 = UDP */
#define L34_NAPTR_TCP_W		1
#define L34_NAPTR_VALID_LSP	84	/* 2-bit valid / NAT-type (modes below) */
#define L34_NAPTR_VALID_W	2
#define L34_NAPTR_PRIVALID_LSP	86
#define L34_NAPTR_PRIVALID_W	1
#define L34_NAPTR_PRIORITY_LSP	87
#define L34_NAPTR_PRIORITY_W	3
/* NAPTR VALID/type: 0 invalid; 1 full-cone (remHash ignored); 2 port-restricted
 * (remHash = hash(remIP,remPort)); 3 restricted-cone (remHash = hash(remIP)). */
#define L34_NAPTR_TYPE_INVALID	0
#define L34_NAPTR_TYPE_FULLCONE	1
#define L34_NAPTR_TYPE_IPPORT	2
#define L34_NAPTR_TYPE_IP	3

/* EXTIP (type 4, 3 words, 8 slots): a NAPTR's EXTIP_IDX points here for the WAN
 * source address + the next-hop the rewritten frame egresses through. */
#define L34_EXTIP_INTIP_LSP	0
#define L34_EXTIP_INTIP_W	32
#define L34_EXTIP_EXTIP_LSP	32	/* WAN external IP */
#define L34_EXTIP_EXTIP_W	32
#define L34_EXTIP_VALID_LSP	64
#define L34_EXTIP_VALID_W	1
#define L34_EXTIP_TYPE_LSP	65	/* 0 = NAPT */
#define L34_EXTIP_TYPE_W	2
#define L34_EXTIP_NHIDX_LSP	67	/* -> NEXTHOP slot */
#define L34_EXTIP_NHIDX_W	4

/* NETIF (type 3, 4 words, 16 slots): per-interface MAC/VLAN/MTU/IP. */
#define L34_NETIF_VALID_LSP	0
#define L34_NETIF_VALID_W	1
#define L34_NETIF_VLANID_LSP	1
#define L34_NETIF_VLANID_W	12
#define L34_NETIF_GMAC_LSP	13	/* 48-bit gateway/source MAC, spans w0..w1 */
#define L34_NETIF_GMAC_W	48
#define L34_NETIF_MACMASK_LSP	61
#define L34_NETIF_MACMASK_W	3
#define L34_NETIF_ENRTR_LSP	64	/* enable routing */
#define L34_NETIF_ENRTR_W	1
#define L34_NETIF_MTU_LSP	65
#define L34_NETIF_MTU_W		14
#define L34_NETIF_L34_LSP	82	/* classify this interface into the L34 NAT domain */
#define L34_NETIF_L34_W		1
#define L34_NETIF_IP_LSP	83	/* interface IP, spans w2..w3 */
#define L34_NETIF_IP_W		32
#define L34_NETIF_DEF_MTU	1500
#define L34_NETIF_DEF_MACMASK	0x7

/* NEXTHOP (type 2, 1 word, 16 slots). nhIdx is an L2-unicast-table index. */
#define L34_NH_TYPE_LSP		0	/* 0 = ETHER, 1 = PPPOE */
#define L34_NH_TYPE_W		1
#define L34_NH_IFIDX_LSP	1	/* -> NETIF slot */
#define L34_NH_IFIDX_W		4
#define L34_NH_NHIDX_LSP	8	/* -> L2 unicast entry holding the dst MAC */
#define L34_NH_NHIDX_W		11

/* ARP_CAM (type 13, 2 words, 128 slots). The MAC lives in the L2 table; this
 * holds the gateway IP and the L2 index where its MAC is resolved. */
#define L34_ARP_IP_LSP		0
#define L34_ARP_IP_W		32
#define L34_ARP_VALID_LSP	32
#define L34_ARP_VALID_W		1
#define L34_ARP_NHIDX_LSP	33	/* -> L2 unicast entry */
#define L34_ARP_NHIDX_W		11

/* L3 ROUTE (type 0, 2 words, 16 slots). The per-netif local route (process=ARP)
 * classifies an ingress frame into the L34 NAT domain and sets US/DS direction;
 * it is what an offloaded NAPT flow needs in order to match. Slot = netif idx. */
#define L34_RT_IP_LSP		0
#define L34_RT_IP_W		32
#define L34_RT_MASK_LSP		32	/* prefix code 0..31 (0 => /1 anchor) */
#define L34_RT_MASK_W		5
#define L34_RT_VALID_LSP	37
#define L34_RT_VALID_W		1
#define L34_RT_PROCESS_LSP	38	/* 0=CPU 1=DROP 2=ARP(local) 3=NH(global) */
#define L34_RT_PROCESS_W	2
#define L34_RT_INT_LSP		40	/* 1 = LAN, 0 = WAN */
#define L34_RT_INT_W		1
#define L34_RT_DENTIF_LSP	41	/* netif index (local-route view of [44:41]) */
#define L34_RT_DENTIF_W		4
#define L34_RT_RT2WANINF_LSP	45	/* route to WAN interface */
#define L34_RT_RT2WANINF_W	1
#define L34_RT_PROCESS_CPU	0	/* terminate locally (to the CPU) */
#define L34_RT_PROCESS_ARP	2

/* L2_UC entry (3 words). The 48-bit MAC is packed octet[0] at the field MSB. */
#define L2UC_MAC_LSP		0
#define L2UC_STATIC_LSP		62	/* no-source-learn => static (not aged out) */
#define L2UC_STATIC_W		1
#define L2UC_SPA_LSP		65	/* [66:65] egress port */
#define L2UC_SPA_W		2
#define L2UC_AGE_LSP		67	/* static forces this non-zero */
#define L2UC_AGE_W		3
#define L2UC_ARPUSED_LSP	73
#define L2UC_ARPUSED_W		1
#define L2UC_VALID_LSP		77
#define L2UC_VALID_W		1

/* L2 insert-status DECODE bits (the engine's reply word).  They moved with
 * l34_l2uc_sts_index(); the register offset and the BUSY bit stay in
 * rtl9602c_l34.h with the poll that owns them.  Spelled without BIT() --
 * this header includes only <linux/types.h>. */
#define L2_STS_ADDR_MASK	0x3ff		/* [9:0] hardware-assigned entry address */
#define L2_STS_CAM		(1u << 10)	/* hash(0)/CAM(1): index = (cam << 10) | addr */
#define L2_STS_HIT		(1u << 12)

void l34_mac48_set(u32 *w, unsigned int lsp, const u8 *mac);
void l34_naptr_encode(u32 *w, u32 int_ip, u16 int_port, u8 extip_idx,
		      u16 ext_port, bool is_tcp);
void l34_napt_encode(u32 *w, u16 naptr_idx);
void l34_netif_encode(u32 *w, const u8 *mac, u32 ip, u16 vlan);
void l34_rt_wan_encode(u32 *w, u8 netif_idx);
void l34_rt_lan_encode(u32 *w, u32 lan_net, u8 prefix, u8 netif_idx);
void l34_rt_cpu_encode(u32 *w, u32 own_ip, u8 netif_idx);
void l34_nexthop_encode(u32 *w, u8 ifidx, unsigned int l2idx);
void l34_extip_encode(u32 *w, u32 wan_ip, u8 nhidx);
void l34_arp_encode(u32 *w, u32 gw_ip, unsigned int l2idx);
void l34_l2uc_encode(u32 *w, const u8 *mac, u8 port);
int l34_l2uc_sts_index(u32 sts);

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
bool rtl9602c_rx_frame_bad(u32 opts1, u32 err_mask, u32 len,
			   unsigned int cpu_prefix, u32 buf_size);
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
