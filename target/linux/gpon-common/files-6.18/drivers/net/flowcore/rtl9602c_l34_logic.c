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

/* R_RxDesNum field packing: RX ring0 size + the flow-control ON/OFF
 * thresholds, in the GMAC's field layout.  Was spelled identically in
 * rtl9602c_hw_program() and the legacy path of rtl9602c_eth_open(). */
u32 rtl9602c_rxdesnum_pack(unsigned int ring_size, unsigned int th_on,
			   unsigned int th_off)
{
	return ((ring_size - 1) & 0xff) << 24 | (th_on & 0xff) << 16 |
	       (th_off & 0xff) << 8 | (((ring_size - 1) >> 8) & 0xf) << 4;
}

/* R_RxCDO field packing (same duplication as rtl9602c_rxdesnum_pack). */
u32 rtl9602c_rxcdo_pack(unsigned int ring_size)
{
	return ((ring_size - 1) & 0xff) << 8 |
	       (((ring_size - 1) >> 8) & 0xf) << 4;
}

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
