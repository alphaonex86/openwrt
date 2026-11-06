/* SPDX-License-Identifier: GPL-2.0-only */
/* eth_rx_geom.h -- where a received frame sits in its DMA buffer, decided ONCE for every chip.
 *
 * Field X111W 2026-11-06: each driver computed the frame from the descriptor length by hand, and one
 * kept 2 FCS bytes (an earlier try cut 2). The descriptor length is a distinct type that only
 * eth_rx_frame_geom() opens; guards/rx_frame_len_guard.py refuses any other reader and any skb
 * length call outside eth_rx_frame_put(). eth_frame_l3_excess() is the runtime witness. PURE.
 */
#ifndef _ETH_RX_GEOM_H
#define _ETH_RX_GEOM_H

#include <linux/types.h>

#define ETH_RX_FCS_LEN	4
#define ETH_RX_HLEN	14
/* ETH_ZLEN plus two VLAN tags: a frame this short may legally carry padding after its datagram. */
#define ETH_RX_PADDED_MAX	68

enum eth_rx_fcs {
	ETH_RX_FCS_UNMEASURED = 0,	/* a zero row: nothing binds on a guessed fact */
	ETH_RX_FCS_IN_LEN,
	ETH_RX_FCS_NOT_IN_LEN,
};

/* What one die's RX descriptor length counts, MEASURED. */
struct eth_rx_layout {
	u8 prefix;		/* bytes the CPU port writes ahead of the frame, outside the length */
	enum eth_rx_fcs fcs;
	u16 buf;		/* bytes of the DMA buffer */
};

/* Opened only by eth_rx_frame_geom(): a driver cannot hand it to skb_put() by mistake. */
struct eth_rx_dlen {
	u32 unopened;
};

struct eth_rx_frame {
	u32 off;		/* where the frame starts in the buffer */
	u32 len;		/* the frame: no prefix, no FCS */
};

enum eth_rx_verdict {
	ETH_RX_OK = 0,
	ETH_RX_UNMEASURED,
	ETH_RX_PARTIAL,		/* one segment of a frame longer than the buffer */
	ETH_RX_RUNT,
	ETH_RX_OVERSIZE,
};

static inline struct eth_rx_dlen eth_rx_dlen(u32 word, u32 mask)
{
	struct eth_rx_dlen d = { .unopened = word & mask };

	return d;
}

static inline enum eth_rx_verdict eth_rx_frame_geom(const struct eth_rx_layout *l, struct eth_rx_dlen d,
						    bool whole, struct eth_rx_frame *fr)
{
	u32 fcs = l->fcs == ETH_RX_FCS_IN_LEN ? ETH_RX_FCS_LEN : 0;

	if (l->fcs == ETH_RX_FCS_UNMEASURED)
		return ETH_RX_UNMEASURED;
	if (!whole)
		return ETH_RX_PARTIAL;
	if (d.unopened < fcs + ETH_RX_HLEN)
		return ETH_RX_RUNT;
	if (l->prefix + d.unopened > l->buf)
		return ETH_RX_OVERSIZE;
	fr->off = l->prefix;
	fr->len = d.unopened - fcs;
	return ETH_RX_OK;
}

/* How far a frame's length is off its own IPv4/IPv6/PPPoE header, through up to two VLAN tags:
 * > 0 bytes after the datagram, < 0 truncated. -> false when it cannot be judged (not IP, a frame
 * short enough to carry padding, an IPv6 jumbogram). */
static inline bool eth_frame_l3_excess(const u8 *f, u32 len, int *excess)
{
	u32 at = 12, end;
	unsigned int type;

	if (len <= ETH_RX_PADDED_MAX)
		return false;
	type = f[at] << 8 | f[at + 1];
	while ((type == 0x8100 || type == 0x88a8) && at < 20) {
		at += 4;
		type = f[at] << 8 | f[at + 1];
	}
	at += 2;
	if (len < at + 40)
		return false;
	if (type == 0x8864)		/* PPPoE session: its length counts the PPP payload */
		end = at + 6 + (f[at + 4] << 8 | f[at + 5]);
	else if (type == 0x0800 && f[at] >> 4 == 4)
		end = at + (f[at + 2] << 8 | f[at + 3]);
	else if (type == 0x86dd && f[at] >> 4 == 6 && (f[at + 4] | f[at + 5]))
		end = at + 40 + (f[at + 4] << 8 | f[at + 5]);
	else
		return false;
	*excess = (int)len - (int)end;
	return true;
}

#ifdef __KERNEL__
#include <linux/skbuff.h>
#define ETH_RX_GEOM_SKB
#endif

#ifdef ETH_RX_GEOM_SKB		/* the kernel's, or a host test's own skb_put/skb_pull */
/* The one place a received frame's length reaches the skb. */
static inline void eth_rx_frame_put(struct sk_buff *skb, const struct eth_rx_frame *fr)
{
	skb_put(skb, fr->off + fr->len);
	skb_pull(skb, fr->off);
}
#endif

#endif /* _ETH_RX_GEOM_H */
