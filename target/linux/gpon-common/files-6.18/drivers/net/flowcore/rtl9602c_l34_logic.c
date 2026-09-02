// SPDX-License-Identifier: GPL-2.0-only
/* See rtl9602c_l34_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>
#include <linux/minmax.h>

#include "rtl9602c_l34_logic.h"

void l34_field_set(u32 *w, unsigned int lsp, unsigned int width, u32 val)
{
	unsigned int word = lsp / 32, bit = lsp % 32, take;

	val &= (width >= 32) ? ~0u : ((1u << width) - 1);
	while (width) {
		take = min(width, 32 - bit);
		w[word] &= ~((((take >= 32) ? ~0u : ((1u << take) - 1))) << bit);
		w[word] |= (val & ((take >= 32) ? ~0u : ((1u << take) - 1))) << bit;
		val >>= take;
		width -= take;
		word++;
		bit = 0;
	}
}

/*
 * NAPT bucket hashes (clean-room: the arithmetic is re-expressed from observed
 * behaviour). Each folds its key to a 10-bit bucket (0..1023); the 4096-entry
 * NAPT/NAPTR tables are 4-way, so an entry index is (bucket << 2) + way, with
 * way 0..3. The is_tcp argument is a 1-bit flag (1=TCP, 0=UDP) — NOT the IP
 * protocol number; only its bit 0 feeds the hash.
 */
u16 l34_hash_out(bool is_tcp, u32 sip, u16 sport, u32 dip, u16 dport)
{
	/* low 16 bits of both addresses + both ports, summed then folded 18->10 */
	u32 lo   = (sip & 0xffff) + (dip & 0xffff) + sport + dport;
	u32 fold = (lo & 0x3ff) + ((lo >> 10) & 0xff);

	/* the address upper halves and the TCP/UDP selector, mixed in by XOR */
	u32 mix  = ((sip >> 16) & 0x3ff) ^ ((dip >> 16) & 0x3ff);

	mix ^= ((sip >> 26) & 0x3f) + ((u32)(is_tcp & 1) << 9);
	mix ^= ((dip >> 26) & 0x3f) << 4;

	return (fold ^ mix) & 0x3ff;
}

/* Inbound NAPTR bucket: keyed only on the post-NAT (WAN-side) dest IP + port. */
u16 l34_hash_in(bool is_tcp, u32 dip, u16 dport)
{
	u32 lo   = dport + (dip & 0xffff);
	u32 fold = (lo & 0x3ff) + ((lo >> 10) & 0x7f);
	u32 mix  = ((dip >> 16) & 0x3ff)
		 ^ (((dip >> 26) & 0x3f) + ((u32)(is_tcp & 1) << 9));

	return (fold ^ mix) & 0x3ff;
}

/* Inverse of l34_field_set: extract a (possibly word-straddling) field.
 * Moved verbatim from rtl9602c_l34.c (2026-09-02) so the set/get pair shares
 * one home -- the SID2QID lesson: a wrong get mirroring a wrong set is
 * invisible when the two live apart. */
u32 l34_field_get(const u32 *w, unsigned int lsp, unsigned int width)
{
	unsigned int word = lsp / 32, bit = lsp % 32, take, got = 0;
	u32 val = 0;

	while (width) {
		take = min(width, 32 - bit);
		val |= ((w[word] >> bit) & ((take >= 32) ? ~0u : ((1u << take) - 1)))
		       << got;
		got += take;
		width -= take;
		word++;
		bit = 0;
	}
	return val;
}

/* ===== entry encoders, SPLIT from rtl9602c_l34.c (2026-09-02) ==========
 * Each is the pure entry-word build of a mixed shell function; the shell
 * keeps the lock, the lazy engine-on, the free-way scans, the table writes
 * and the unwind ordering.  The l34_field_set sequences are byte-identical
 * to the shell's; the declared deviations are (a) struct-member reads
 * became arguments (f->orig_sip and friends -- this file reads no shell
 * state), and (b) the triple-spelled 48-bit-MAC two-word split now exists
 * once, as l34_mac48_set().  Proven equal old-vs-new on x86
 * (hoist2-luna-eth harness).
 */

/* 48-bit MAC into a packed entry: octet[0] sits at the field's
 * most-significant byte.  Was spelled VERBATIM three times (the L2 unicast
 * key, the WAN netif, the LAN netif). */
void l34_mac48_set(u32 *w, unsigned int lsp, const u8 *mac)
{
	l34_field_set(w, lsp, 32,
		      ((u32)mac[2] << 24) | ((u32)mac[3] << 16) |
		      ((u32)mac[4] << 8)  |  (u32)mac[5]);
	l34_field_set(w, lsp + 32, 16,
		      ((u32)mac[0] << 8) | (u32)mac[1]);
}

/* NAPTR_IN rewrite entry: internal host addr/port + post-NAT port, WAN side
 * via the EXTIP slot; written full-cone (remHash unused) so the return path
 * matches on the WAN addr/port alone. */
void l34_naptr_encode(u32 *w, u32 int_ip, u16 int_port, u8 extip_idx,
		      u16 ext_port, bool is_tcp)
{
	l34_field_set(w, L34_NAPTR_INTIP_LSP,    L34_NAPTR_INTIP_W,    int_ip);
	l34_field_set(w, L34_NAPTR_INTPORT_LSP,  L34_NAPTR_INTPORT_W,  int_port);
	l34_field_set(w, L34_NAPTR_EXTIPIDX_LSP, L34_NAPTR_EXTIPIDX_W, extip_idx);
	l34_field_set(w, L34_NAPTR_EXTPORT_LSP,  L34_NAPTR_EXTPORT_W,  ext_port);
	l34_field_set(w, L34_NAPTR_TCP_LSP,      L34_NAPTR_TCP_W,      is_tcp);
	l34_field_set(w, L34_NAPTR_VALID_LSP,    L34_NAPTR_VALID_W,    L34_NAPTR_TYPE_FULLCONE);
}

/* NAPT_OUT hash slot: the entry index IS the outbound hash; the word only
 * points at the rewrite entry. */
void l34_napt_encode(u32 *w, u16 naptr_idx)
{
	l34_field_set(w, L34_NAPT_HASHIN_IDX_LSP, L34_NAPT_HASHIN_IDX_W, naptr_idx);
	l34_field_set(w, L34_NAPT_VALID_LSP,      L34_NAPT_VALID_W,      1);
}

/* NETIF entry: egress source MAC + VLAN/MTU/IP, routing enabled, classified
 * into the L34 NAT domain.  Was spelled VERBATIM twice (wan_setup and
 * lan_setup, 11 identical l34_field_set lines differing only in arguments). */
void l34_netif_encode(u32 *w, const u8 *mac, u32 ip, u16 vlan)
{
	l34_field_set(w, L34_NETIF_VALID_LSP,   L34_NETIF_VALID_W,   1);
	l34_field_set(w, L34_NETIF_VLANID_LSP,  L34_NETIF_VLANID_W,  vlan);
	l34_mac48_set(w, L34_NETIF_GMAC_LSP, mac);
	l34_field_set(w, L34_NETIF_MACMASK_LSP, L34_NETIF_MACMASK_W, L34_NETIF_DEF_MACMASK);
	l34_field_set(w, L34_NETIF_ENRTR_LSP,   L34_NETIF_ENRTR_W,   1);
	l34_field_set(w, L34_NETIF_MTU_LSP,     L34_NETIF_MTU_W,     L34_NETIF_DEF_MTU);
	l34_field_set(w, L34_NETIF_L34_LSP,     L34_NETIF_L34_W,     1);
	l34_field_set(w, L34_NETIF_IP_LSP,      L34_NETIF_IP_W,      ip);
}

/* WAN local route: classify L34-domain ingress on this WAN netif and set the
 * US/DS direction the NAPT path keys on (the vendor leaves valid=0 here,
 * which is why offload silently fails -- we set valid=1).  IP/MASK/INT left
 * 0 (WAN). */
void l34_rt_wan_encode(u32 *w, u8 netif_idx)
{
	l34_field_set(w, L34_RT_PROCESS_LSP,   L34_RT_PROCESS_W,   L34_RT_PROCESS_ARP);
	l34_field_set(w, L34_RT_DENTIF_LSP,    L34_RT_DENTIF_W,    netif_idx);
	l34_field_set(w, L34_RT_RT2WANINF_LSP, L34_RT_RT2WANINF_W, 1);
	l34_field_set(w, L34_RT_VALID_LSP,     L34_RT_VALID_W,     1);
}

/* LAN subnet route (process=ARP, internal=1).  ⚠ mask = prefix - 1: the
 * 5-bit MASK field is a prefix CODE (0 => /1 anchor, 31 => /32) -- the
 * off-by-one fact a host test pins here. */
void l34_rt_lan_encode(u32 *w, u32 lan_net, u8 prefix, u8 netif_idx)
{
	l34_field_set(w, L34_RT_IP_LSP,      L34_RT_IP_W,      lan_net);
	l34_field_set(w, L34_RT_MASK_LSP,    L34_RT_MASK_W,    prefix - 1);
	l34_field_set(w, L34_RT_PROCESS_LSP, L34_RT_PROCESS_W, L34_RT_PROCESS_ARP);
	l34_field_set(w, L34_RT_INT_LSP,     L34_RT_INT_W,     1);	/* LAN */
	l34_field_set(w, L34_RT_DENTIF_LSP,  L34_RT_DENTIF_W,  netif_idx);
	l34_field_set(w, L34_RT_VALID_LSP,   L34_RT_VALID_W,   1);
}

/* The more-specific /32 CPU self-route (mask code 31): traffic addressed to
 * the interface's OWN IP terminates locally instead of being ARP-routed by
 * the subnet route -- which black-holes management once the LAN netif is in
 * the L34 domain. */
void l34_rt_cpu_encode(u32 *w, u32 own_ip, u8 netif_idx)
{
	l34_field_set(w, L34_RT_IP_LSP,      L34_RT_IP_W,      own_ip);
	l34_field_set(w, L34_RT_MASK_LSP,    L34_RT_MASK_W,    31);	/* /32 */
	l34_field_set(w, L34_RT_PROCESS_LSP, L34_RT_PROCESS_W, L34_RT_PROCESS_CPU);
	l34_field_set(w, L34_RT_INT_LSP,     L34_RT_INT_W,     1);
	l34_field_set(w, L34_RT_DENTIF_LSP,  L34_RT_DENTIF_W,  netif_idx);
	l34_field_set(w, L34_RT_VALID_LSP,   L34_RT_VALID_W,   1);
}

/* Ethernet next-hop via NETIF[ifidx], dst MAC = L2[l2idx]. */
void l34_nexthop_encode(u32 *w, u8 ifidx, unsigned int l2idx)
{
	l34_field_set(w, L34_NH_TYPE_LSP,  L34_NH_TYPE_W,  0);	/* ETHER */
	l34_field_set(w, L34_NH_IFIDX_LSP, L34_NH_IFIDX_W, ifidx);
	l34_field_set(w, L34_NH_NHIDX_LSP, L34_NH_NHIDX_W, l2idx);
}

/* EXTIP slot: the WAN source IP a NAPT rewrite applies, via NEXTHOP[nhidx]. */
void l34_extip_encode(u32 *w, u32 wan_ip, u8 nhidx)
{
	l34_field_set(w, L34_EXTIP_EXTIP_LSP, L34_EXTIP_EXTIP_W, wan_ip);
	l34_field_set(w, L34_EXTIP_VALID_LSP, L34_EXTIP_VALID_W, 1);
	l34_field_set(w, L34_EXTIP_TYPE_LSP,  L34_EXTIP_TYPE_W,  0);	/* NAPT */
	l34_field_set(w, L34_EXTIP_NHIDX_LSP, L34_EXTIP_NHIDX_W, nhidx);
}

/* ARP entry: gateway IP -> the L2 entry holding its MAC. */
void l34_arp_encode(u32 *w, u32 gw_ip, unsigned int l2idx)
{
	l34_field_set(w, L34_ARP_IP_LSP,    L34_ARP_IP_W,    gw_ip);
	l34_field_set(w, L34_ARP_VALID_LSP, L34_ARP_VALID_W, 1);
	l34_field_set(w, L34_ARP_NHIDX_LSP, L34_ARP_NHIDX_W, l2idx);
}

/* The 3-word static L2 unicast (FDB) key: MAC + static/spa/age/arpused/valid.
 * The Cortina family's cortina_ni_l2fe_fdb_key is the same job for its
 * silicon -- the pair now sits in the same tier. */
void l34_l2uc_encode(u32 *w, const u8 *mac, u8 port)
{
	/* 48-bit MAC: octet[0] sits at the field's most-significant byte. */
	l34_mac48_set(w, L2UC_MAC_LSP, mac);
	l34_field_set(w, L2UC_STATIC_LSP,  L2UC_STATIC_W,  1);
	l34_field_set(w, L2UC_SPA_LSP,     L2UC_SPA_W,     port);
	l34_field_set(w, L2UC_AGE_LSP,     L2UC_AGE_W,     1);
	l34_field_set(w, L2UC_ARPUSED_LSP, L2UC_ARPUSED_W, 1);
	l34_field_set(w, L2UC_VALID_LSP,   L2UC_VALID_W,   1);
}

/* Decode the L2 insert engine's reply: the hardware-assigned entry index
 * ((cam << 10) | addr), or -1 when the engine reports no hit (table full).
 * Declared deviation: the errno (-ENOSPC) stays shell-side -- this file
 * includes no errno.h. */
int l34_l2uc_sts_index(u32 sts)
{
	if (!(sts & L2_STS_HIT))
		return -1;
	return ((sts & L2_STS_CAM) ? (1 << 10) : 0) | (sts & L2_STS_ADDR_MASK);
}

/* ===== hoisted from rtl9602c_eth.c (same shell TU) =====================
 * The verbatim rule holds per function below; where a function is a SPLIT
 * (the pure verdict extracted from a mixed function) the extracted
 * expressions are byte-identical to the shell's and the deviations from a
 * verbatim move are declared at the function.
 */

/*
 * WAN identity MAC (the MAC the ISP/OLT recognises the ONU by) = LAN base
 * MAC + `add`, as a 48-bit ripple-carry add over the 6-byte buffer (stock
 * nas0_0 = base+3, verified on two boards).  Moved from rtl9602c_wan_mac()
 * -- the shell keeps a wrapper under that name -- with two declared
 * deviations: the wan_mac_offset module param became the `add` argument
 * (this file reads no shell file-scope state), and ether_addr_copy() became
 * a plain 6-byte copy (etherdevice.h is not includable here).  The add loop
 * is verbatim.
 */
void rtl9602c_wan_mac_add(u8 *out, const u8 *base, unsigned int add)
{
	int i;

	for (i = 0; i < 6; i++)		/* was ether_addr_copy(out, base) */
		out[i] = base[i];
	for (i = 6 - 1; i >= 0 && add; i--) {
		unsigned int s = out[i] + (add & 0xff);

		out[i] = s & 0xff;
		add = (add >> 8) + (s >> 8);
	}
}

/*
 * DS-OMCI classifier + length guard, SPLIT from rtl9602c_eth_rx(): whether a
 * received descriptor + first bytes is a DS OMCI frame the OMCC responder
 * must consume.  `data` is the raw frame as delivered to the CPU (pre-pull:
 * the cpu_prefix bytes still in front); buf_size is the DMA buffer size the
 * caller mapped.
 *
 * The length guard comes FIRST because the OMCI path reads
 * `len - cpu_prefix` bytes from the RX buffer, so `len` must be
 * range-checked on BOTH ends before any data byte is touched:
 *   - lower (>= cpu_prefix + 8): a runt reason==246 descriptor (len < 2)
 *     wrapped the unsigned `len - cpu_prefix` to ~4 GiB -> massive OOB read.
 *   - upper (<= buf_size): the descriptor len field is 13 bits (up to 8191)
 *     but the DMA buffer is only 2 KB, so an oversized len reads past it.
 * Both remotely triggerable (DoS/info-leak).  Found -- and the lower-bound-
 * only partial fix re-caught -- by fuzz/fuzz_rx.c (ASan), which can now
 * drive THIS exact shipping predicate on x86 instead of a re-derivation.
 *
 * The match is either the cpu-tag trap (rx-reason 246 from the PON port) or
 * the switch-routed shape: DS OMCI actually arrives SWITCH-routed (no
 * reason==246) -- the de-encapsulated baseline OMCI rides the GMAC CPU-port
 * behind the prefix as raw G.988 -> [TID(2)][MT(1)][DevID(1)=0x0a baseline/
 * 0x0b extended][class(2)][inst(2)]...  Match by DevID + MT destination-bit
 * clear.  A LAN frame to the CPU has dst-MAC[3] here (board MAC ..:32:..,
 * bcast 0xff) never 0x0a, so this does not steal LAN traffic.  Verified
 * live: OLT sent MT 0x49 (GET) DevID 0x0a class 0x0101.
 */
bool rtl9602c_rx_is_ds_omci(bool trap_on, u32 opts2, u32 opts3,
			    const u8 *data, u32 len, unsigned int pon_port,
			    unsigned int cpu_prefix, u32 buf_size)
{
	if (!trap_on || len < cpu_prefix + 8 || len > buf_size)
		return false;
	if (((opts2 >> 21) & 0xff) == RTL9602C_OMCI_REASON &&
	    ((opts3 >> 16) & 0xf) == pon_port)
		return true;
	return (data[cpu_prefix + 3] == 0x0a || data[cpu_prefix + 3] == 0x0b) &&
	       !(data[cpu_prefix + 2] & 0x80);
}

/*
 * WAN demux verdict, SPLIT from rtl9602c_eth_rx(): does this good frame
 * belong to the WAN netdev (gpon0) rather than eth0?  True when any of:
 *   - opts3 src_port_num [19:16] == the PON switch port (de-encapsulated
 *     data-GEM traffic that ingressed on the PON port);
 *   - the PON-IP-NIC drain signature: WAN downstream DATA drains via the
 *     PON-IP NIC and lands with src_port=0 (NOT the PON switch-port) AND a
 *     fixed descriptor signature opts3[31:20] == 0x23e (confirmed live
 *     2026-06-15) -- LAN frames arrive switch-forwarded with the real
 *     ingress port and a different opts3;
 *   - DA == the WAN identity MAC (unicast addressed to gpon0).
 * `dst` points at the Ethernet header (prefix already skipped by the
 * caller); the byte compare is ether_addr_equal() re-expressed (etherdevice.h
 * is not includable here).  The shell keeps the netdev pointers, the
 * host_port learning side effect and the hand-up.
 */
bool rtl9602c_rx_wan_demux(u32 opts3, const u8 *dst, const u8 *wan_mac,
			   unsigned int pon_port)
{
	unsigned int sp = (opts3 >> 16) & 0xf;
	bool wan_drain = (opts3 >> 20) == 0x23e;
	int i;

	if (sp == pon_port || wan_drain)
		return true;
	for (i = 0; i < 6; i++)		/* was ether_addr_equal(dst, wan_mac) */
		if (dst[i] != wan_mac[i])
			return false;
	return true;
}

/* The HW ring h behind the omci_tx_ring module param: instances >5 clamp to
 * ring 4.  Was spelled seven times across the shell (xmit, rekick, align,
 * hw_program, open x2, diag). */
unsigned int rtl9602c_omci_hwring(unsigned int omci_tx_ring)
{
	return omci_tx_ring > 5 ? 4 : omci_tx_ring;
}

/*
 * Which doorbell kicks HW ring h, SPLIT from rtl9602c_eth_omci_xmit() /
 * _tx_rekick() / _diag_show() (a recorded past bug was exactly a ring armed
 * at h=4 but kicked at bit0 -- deriving both from one place makes that
 * disagreement impossible).  Stock kick behaviour: h<4 -> R_IO_CMD |= 1<<h;
 * h==4 -> R_IO_CMD1 |= 0x100 (TX_POLL5, bit8).  doorbell_ovr is the
 * omci_doorbell_bit debug override: 0xff = stock derivation, else the forced
 * R_IO_CMD bit number.
 *
 * Returns true when the kick is R_IO_CMD1, false when it is R_IO_CMD; *mask
 * is the value to OR into that register, *hwring the clamped ring h.
 */
bool rtl9602c_omci_doorbell(unsigned int omci_tx_ring,
			    unsigned int doorbell_ovr,
			    unsigned int *hwring, u32 *mask)
{
	unsigned int h = rtl9602c_omci_hwring(omci_tx_ring);
	unsigned int dbit = (doorbell_ovr == 0xff)
		? (h & 0x1f)	/* R_IO_CMD bit number == HW ring (h<4) */
		: doorbell_ovr;

	*hwring = h;
	if (h == 4 && doorbell_ovr == 0xff) {
		*mask = 0x100;	/* R_IO_CMD1 TX_POLL5 (bit8 = 0x100) */
		return true;
	}
	*mask = 1u << dbit;
	return false;
}

/* rtl9602c_rxdesnum_pack / rtl9602c_rxcdo_pack MOVED to luna_gmac_logic.c as
 * luna_gmac_rxdesnum_pack / luna_gmac_rxcdo_pack (2026-09-02), the day
 * luna_eth.c was proven to spell the identical expressions: the GMAC ring
 * packing is the FAMILY's (both shells read R_RxDesNum, RX_RING_SIZE and the
 * TH_* values from luna_eth_regs.h), and family-shared code may not hide
 * behind one chip's name.  This TU is gated on CONFIG_RTL9602C_ETH alone, so
 * the second caller could not link against it -- the new object is gated
 * under both Ethernet symbols. */

/*
 * The 9602C OMCI/WAN directed-TX steering words (opts2/word2, opts3/word3),
 * SPLIT from the three CPU->US-NIC xmit paths (omci_xmit, omci_xmit_ring0,
 * wan_xmit), each of which spelled the override-else-stock ternary itself.
 * ovr == 0 means no override (the omci_word2_ovr/omci_word3_ovr module
 * params' resting value).  Stock values on the live ref ONU: word2 =
 * 0x80080000 (cputag|efid), word3 = 0x02400000 for the OMCC SID 64 (see the
 * layout note at the defines).  word0 stays in the shell: it is composed
 * from the LUNA-family descriptor flags (D_FS/D_LS/D_TXCRC/...), which live
 * in the family header this file cannot include, and OWN/EOR placement is
 * publish-order -- shell by definition.
 */
u32 rtl9602c_omci_txd_word2(u32 ovr)
{
	return ovr ? ovr : (TXD2_OMCI_CPUTAG | TXD2_OMCI_EFID);
}

u32 rtl9602c_omci_txd_word3(u32 ovr, unsigned int sid)
{
	return ovr ? ovr : TXD3_OMCI_9602C(sid);
}

/*
 * Bad-frame verdict, SPLIT from rtl9602c_eth_rx(): is a returned RX
 * descriptor discarded?  Completes the RX verdict trio (rtl9602c_rx_is_ds_omci
 * and rtl9602c_rx_wan_demux above) so fuzz_rx.c can drive the WHOLE shipping
 * RX classification chain on x86, not two-thirds of it.
 *
 * @err_mask is the family's RXD_CRCERR | RXD_RCDF -- those bits are LUNA
 * FAMILY descriptor facts (luna_eth_regs.h / the shell) and passing the mask
 * keeps this header from re-spelling them (a second spelling of BIT(27) is
 * the drift this tier exists to remove).  The 60 is ETH_ZLEN re-expressed
 * (if_ether.h is not includable here) -- same declared deviation as
 * ether_addr_copy in rtl9602c_wan_mac_add().
 */
bool rtl9602c_rx_frame_bad(u32 opts1, u32 err_mask, u32 len,
			   unsigned int cpu_prefix, u32 buf_size)
{
	return (opts1 & err_mask) ||
	       len < 60 + cpu_prefix ||	/* 60 = ETH_ZLEN, the min Ethernet frame */
	       len > buf_size;
}
