/* SPDX-License-Identifier: GPL-2.0-only */
/* eth_rx_geom.h -- where a received frame sits in its DMA buffer, decided ONCE for every chip.
 *
 * Field X111W 2026-11-06: each driver computed the frame from the descriptor length by hand, and one
 * kept 2 FCS bytes (an earlier try cut 2). The length is a distinct type that only this header opens;
 * guards/rx_frame_len_guard.py refuses any other reader and any skb length call outside the
 * eth_rx_frame_put/copy/seg_put helpers. eth_frame_l3_excess() is the runtime witness. PURE.
 */
#ifndef _ETH_RX_GEOM_H
#define _ETH_RX_GEOM_H

#include <linux/types.h>

#define ETH_RX_FCS_LEN	4
#define ETH_RX_ALEN	6
#define ETH_RX_HLEN	14
/* ETH_ZLEN plus two VLAN tags: a frame this short may legally carry padding after its datagram. */
#define ETH_RX_PADDED_MAX	68

enum eth_rx_fcs {
	ETH_RX_FCS_UNMEASURED = 0,	/* a zero row: nothing binds on a guessed fact */
	ETH_RX_FCS_IN_LEN,
	ETH_RX_FCS_NOT_IN_LEN,
};

/* What one RX length (a descriptor's, or a header the chip writes into the buffer) counts, MEASURED. */
struct eth_rx_layout {
	u8 prefix;		/* buffer bytes ahead of the first byte the length counts */
	u8 head;		/* bytes the length counts ahead of the frame: a header the chip writes in-band */
	enum eth_rx_fcs fcs;
	u16 buf;		/* bytes of the DMA buffer a frame may use */
};

/* Opened only here: a driver cannot hand it to skb_put() by mistake. */
struct eth_rx_dlen {
	u32 unopened;
};

struct eth_rx_frame {
	u32 off;		/* where the frame (or a chain segment) starts in the buffer */
	u32 len;		/* its bytes: no prefix, no header, no FCS */
};

enum eth_rx_verdict {
	ETH_RX_OK = 0,
	ETH_RX_UNMEASURED,
	ETH_RX_PARTIAL,		/* one segment of a frame longer than the buffer */
	ETH_RX_RUNT,
	ETH_RX_OVERSIZE,
};

/* The length field `mask` of a descriptor or header word, wherever the field sits in it. */
static inline struct eth_rx_dlen eth_rx_dlen(u64 word, u64 mask)
{
	struct eth_rx_dlen d = { .unopened = (word & mask) >> __builtin_ctzll(mask) };

	return d;
}

static inline u32 eth_rx_fcs_bytes(const struct eth_rx_layout *l)
{
	return l->fcs == ETH_RX_FCS_IN_LEN ? ETH_RX_FCS_LEN : 0;
}

static inline enum eth_rx_verdict eth_rx_frame_geom(const struct eth_rx_layout *l, struct eth_rx_dlen d,
						    bool whole, struct eth_rx_frame *fr)
{
	u32 fcs = eth_rx_fcs_bytes(l);

	if (l->fcs == ETH_RX_FCS_UNMEASURED)
		return ETH_RX_UNMEASURED;
	if (!whole)
		return ETH_RX_PARTIAL;
	if (d.unopened < l->head + fcs + ETH_RX_HLEN)
		return ETH_RX_RUNT;
	if (d.unopened > l->buf || l->prefix + d.unopened > l->buf)
		return ETH_RX_OVERSIZE;
	fr->off = l->prefix + l->head;
	d.unopened -= l->head;
	fr->len = d.unopened - fcs;
	return ETH_RX_OK;
}

/* Where in the buffer the frame a length describes ends (FCS excluded): two lengths of one frame that
 * disagree are a buffer reused under its descriptor. */
static inline u32 eth_rx_dlen_end(const struct eth_rx_layout *l, struct eth_rx_dlen d)
{
	return l->prefix + d.unopened - eth_rx_fcs_bytes(l);
}

/* A frame spread over several buffers: its first one read under a layout, the rest from `cont`. */
struct eth_rx_chain_geom {
	u32 cont;		/* where a continuation buffer's bytes start */
	u32 max_len;		/* the longest frame a chain may promise */
	u16 max_segs;
};

struct eth_rx_chain {
	u32 total;		/* frame bytes the first segment's length promised */
	u32 wire;		/* the bytes that length counts after its header: the frame and its FCS */
	u32 got;		/* of `wire`, taken so far */
	u16 segs;
	bool open;
};

enum eth_rx_chain_act {
	ETH_RX_CHAIN_OPEN,	/* first segment accepted, more expected */
	ETH_RX_CHAIN_APPEND,	/* continuation accepted, more expected */
	ETH_RX_CHAIN_DONE,	/* last segment accepted: the frame is complete */
	/* malformations: the caller drops the partial frame for each */
	ETH_RX_CHAIN_ORPHAN,	/* continuation with no chain open */
	ETH_RX_CHAIN_BADTOTAL,	/* first length unmeasured or out of range, or no room in the buffer */
	ETH_RX_CHAIN_TOOLONG,	/* segment cap hit, or a segment overran the frame */
	ETH_RX_CHAIN_SHORT,	/* last segment with fewer bytes than promised */
};

/* One buffer of a chain; `l` reads the first one (d = its length) and gives every buffer's end (l->buf).
 * -> *seg: the frame bytes this buffer holds. */
static inline enum eth_rx_chain_act eth_rx_chain_step(const struct eth_rx_layout *l, struct eth_rx_chain *c,
						      bool sop, bool eop, struct eth_rx_dlen d,
						      const struct eth_rx_chain_geom *g, struct eth_rx_frame *seg)
{
	u32 off, avail, take;

	seg->off = seg->len = 0;
	if (sop) {
		u32 fcs = eth_rx_fcs_bytes(l);

		off = l->prefix + l->head;
		if (l->fcs == ETH_RX_FCS_UNMEASURED || d.unopened < l->head + fcs + ETH_RX_HLEN ||
		    d.unopened - l->head - fcs > g->max_len || off >= l->buf)
			return ETH_RX_CHAIN_BADTOTAL;
		c->wire = d.unopened - l->head;
		c->total = c->wire - fcs;
		c->got = 0;
		c->segs = 0;
		c->open = true;
	} else {
		if (!c->open)
			return ETH_RX_CHAIN_ORPHAN;
		off = g->cont;
		if (off >= l->buf)
			return ETH_RX_CHAIN_BADTOTAL;
	}
	if (++c->segs > g->max_segs)
		return ETH_RX_CHAIN_TOOLONG;
	avail = l->buf - off;
	take = c->wire - c->got < avail ? c->wire - c->got : avail;
	seg->off = off;
	seg->len = c->got >= c->total ? 0 : c->total - c->got < take ? c->total - c->got : take;
	c->got += take;
	if (eop) {
		if (c->got != c->wire)
			return ETH_RX_CHAIN_SHORT;
		c->open = false;
		return ETH_RX_CHAIN_DONE;
	}
	/* not the last segment, so it must have filled its buffer: else the length and the chip disagree */
	if (take != avail)
		return ETH_RX_CHAIN_TOOLONG;
	return sop ? ETH_RX_CHAIN_OPEN : ETH_RX_CHAIN_APPEND;
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

/* Received frames whose length is off their own IP header, by side: ethtool -S rx_len_*. */
struct eth_rx_len {
	u64 long_lan, short_lan, long_wan, short_wan;
};

/* Count, never trim: trimming would hide the bug this exists to show. -> the excess, 0 when exact or unjudged. */
static inline int eth_rx_len_note(struct eth_rx_len *c, const u8 *frame, u32 len, bool wan)
{
	int excess;

	if (!eth_frame_l3_excess(frame, len, &excess) || !excess)
		return 0;
	if (excess > 0)
		wan ? c->long_wan++ : c->long_lan++;
	else
		wan ? c->short_wan++ : c->short_lan++;
	return excess;
}

#ifdef __KERNEL__
#include <linux/skbuff.h>
#define ETH_RX_GEOM_SKB
#define ETH_RX_GEOM_COPY
#endif

#ifdef ETH_RX_GEOM_SKB		/* the kernel's, or a host test's own skb calls */
/* The one place a received frame's length reaches the skb, for a driver that hands its DMA buffer up. */
static inline void eth_rx_frame_put(struct sk_buff *skb, const struct eth_rx_frame *fr)
{
	skb_put(skb, fr->off + fr->len);
	skb_pull(skb, fr->off);
}
#endif

#ifdef ETH_RX_GEOM_COPY		/* needs napi_alloc_skb() and skb_put_data() */
/* For a driver that recycles its DMA buffer: the frame copied into a fresh skb, less the `cut` bytes that
 * follow its two MAC addresses (a tag the stack must not see; never more than the frame holds). */
static inline struct sk_buff *eth_rx_frame_copy(struct napi_struct *napi, const u8 *buf,
						const struct eth_rx_frame *fr, u32 cut)
{
	const u8 *f = buf + fr->off;
	struct sk_buff *skb;

	if (fr->len < 2 * ETH_RX_ALEN + cut)
		cut = 0;
	skb = napi_alloc_skb(napi, fr->len - cut);
	if (!skb)
		return NULL;
	if (cut) {
		skb_put_data(skb, f, 2 * ETH_RX_ALEN);
		skb_put_data(skb, f + 2 * ETH_RX_ALEN + cut, fr->len - 2 * ETH_RX_ALEN - cut);
	} else {
		skb_put_data(skb, f, fr->len);
	}
	return skb;
}

/* One chain segment appended to the skb its chain's total sized. */
static inline void eth_rx_seg_put(struct sk_buff *skb, const u8 *buf, const struct eth_rx_frame *seg)
{
	skb_put_data(skb, buf + seg->off, seg->len);
}
#endif

#endif /* _ETH_RX_GEOM_H */
