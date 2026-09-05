/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cortina_ni_rx_geom.h -- RX BUFFER GEOMETRY, lifted out of the NAPI receive
 * path: which pool owns a descriptor's buffer, where inside that buffer the
 * frame starts and how long it is, and whether what arrived is the PDC's PON
 * control punt.
 *
 * TIER: core.  The rule is written out once, in drivers/net/gpon/gpon_common.h;
 * do not restate it here.
 *
 * ★ WHY IT IS HERE AND NOT IN drivers/net/gpon/.  Nothing below is GPON
 * protocol.  It is Cortina EPP descriptor geometry, a CPU-DRAM pool layout and
 * the PDC's vendor link-type marker -- none of which appears anywhere in the
 * realtek-luna net drivers.  Putting it under drivers/net/gpon/ would have been
 * the naming lie flowcore.h:6-9 exists to refuse.  It is a sibling of
 * cortina_ni_rx_logic.h, reached by the same -I the shell already carries.
 *
 * ★ HEADER-ONLY, static inline, ON PURPOSE.  Both the shell and the x86
 * differential reach it with an #include and no Makefile line, so it cannot
 * become a scraped copy of a body (the mechanism gen_rx_chain_impl.sh still has
 * to use for ca_ni_chain_step, and the one this shape replaces).
 *
 * ★ NOT ONE CONSTANT OF THE SHELL'S IS RE-SPELLED HERE.  Every pool size,
 * offset, tailroom and header length arrives as an ARGUMENT, exactly as
 * luna_gmac_logic.h:28-29 takes the family descriptor bit masks -- so this file
 * can never drift from cortina-ni-regs.h / cortina-ni.h, because it does not
 * know what is in them.  What it owns is the ARITHMETIC and the VERDICT.
 *
 * ★ WHAT IT IS WORTH.  cortina-ni.h:225-236 records the defect that lived in
 * this arithmetic: the payload bound used the BUFFER SIZE rather than the
 * buffer's usable window, 384 bytes too generous, so a frame could be copied
 * "out of memory the hardware never wrote ... silently, with no counter
 * moving".  The chain path's copy of that window is pinned on x86 by
 * rx_chain_test; the single-buffer path had no host test at all.  Now both
 * halves of the window -- who owns the buffer, and where the frame may end --
 * are enumerable off the board.
 *
 * Endianness: no cast over wire bytes anywhere below.  The one place wire bytes
 * are read (ca_ni_rx_pon_ctrl) uses explicit byte math, so the same code decides
 * identically on MIPS BE, ARM64 LE and x86.
 */
#ifndef _CORTINA_NI_RX_GEOM_H
#define _CORTINA_NI_RX_GEOM_H

#include <linux/types.h>

/* ------------------------------------------------------------------ */
/* 1. WHICH POOL OWNS THIS BUFFER, AND IS IT OURS TO RECYCLE           */
/* ------------------------------------------------------------------ */

/*
 * The mapped region holds three pools back to back: the two CPU pools, then the
 * deep-queue pool.  A CPU-pool PA is handed back to its free list once the frame
 * has been copied out; the deep-queue pool is HARDWARE-managed, so pushing one
 * of its buffers into a CPU pool's free list would put the same buffer in two
 * allocators at once.  That is why the answer is a POOL and not a bool.
 */
enum ca_ni_rx_pool {
	CA_NI_RX_POOL_BAD = 0,	/* PA outside the mapped window - not ours at all */
	CA_NI_RX_POOL_CPU0,	/* first CPU pool  - ours, hand the PA back */
	CA_NI_RX_POOL_CPU1,	/* second CPU pool - ours, hand the PA back */
	CA_NI_RX_POOL_DQ,	/* deep-queue pool - HW-managed, NEVER push it back */
};

/*
 * The board's pool geometry.  Every field is a SHELL constant, passed in: this
 * file must not know any of these values.
 *
 * @map_size:    bytes of the one mapping that covers all three pools
 * @pool0_bytes: where the first CPU pool ends (= where the second begins)
 * @dq_off:      where the deep-queue pool begins
 * @pool0_bufsz: buffer stride of the first CPU pool
 * @pool1_bufsz: buffer stride of the second CPU pool
 * @dq_bufsz:    buffer stride of the deep-queue pool.  A SEPARATE field even
 *               when the shell happens to pass the same value as @pool0_bufsz:
 *               "the deep queue uses the same buffers" is the shell's fact to
 *               state, not this file's to assume.
 * @tailroom:    bytes the QM reserves at the BACK of every buffer, which the
 *               frame DMA never writes
 */
struct ca_ni_rx_pool_geom {
	u32 map_size;
	u32 pool0_bytes;
	u32 dq_off;
	u32 pool0_bufsz;
	u32 pool1_bufsz;
	u32 dq_bufsz;
	u32 tailroom;
};

/*
 * @off_in_region: byte offset of the buffer inside the mapping (the shell turns
 *                 this into a VA; this file never holds a pointer to it)
 * @buf_max:       one past the last byte the frame DMA may have written -- the
 *                 END OF THE USABLE WINDOW, not the buffer size
 * @rpa:           the PA to hand back to a free list, or 0 when the buffer is
 *                 not ours to recycle
 */
struct ca_ni_rx_buf {
	u32 off_in_region;
	u32 buf_max;
	u32 rpa;
};

/*
 * The usable payload window of one buffer.  Clamped at zero: a tailroom that
 * reached or passed the buffer size would otherwise wrap u32 and produce a
 * ~4 GB ceiling, i.e. exactly the "bound too generous" failure this arithmetic
 * already suffered once, in its most extreme form.  With the shipped constants
 * the clamp never fires; it exists so a mis-declared pool fails CLOSED.
 */
static inline u32 ca_ni_rx_usable_end(u32 bufsz, u32 tailroom)
{
	return bufsz > tailroom ? bufsz - tailroom : 0;
}

/**
 * ca_ni_rx_buf_locate() - place a descriptor's buffer PA in the pool map.
 * @pa:   the descriptor's buffer PA, already stripped of its flag bits
 * @base: PA of the start of the mapping
 * @g:    the board's pool geometry
 * @out:  filled on every return; zeroed when the PA is outside the mapping
 *
 * Return: which pool owns @pa, CA_NI_RX_POOL_BAD when it owns none.
 *
 * ★ THE RANGE TEST IS ONE SUBTRACTION, NOT THE PAIR THE SHELL SPELLED.  It was
 * `pa < base || pa >= base + map_size`; it is now `pa - base >= map_size` on
 * u32, and the difference is worth stating because it is NOT the one it looks
 * like.
 *
 *   - Whenever the mapping does not straddle the top of the 32-bit PA space --
 *     every board in this tree -- the two are EXACTLY equivalent, case for
 *     case: for pa below @base the wrapped difference is at least
 *     2^32 - @base >= @map_size, so it is refused just the same.
 *   - When the mapping DOES straddle the top, `base + map_size` wraps to a
 *     small number, `pa >= base + map_size` becomes true for every PA at or
 *     above @base, and the old pair refuses the ENTIRE mapping: the receive
 *     path stops dead.  The subtraction cannot wrap, so it accepts exactly the
 *     addresses that are in the region -- both the part below the top and the
 *     part that wrapped past it -- and @off_in_region comes out right for both.
 *
 * So the single test is never more permissive about memory OUTSIDE the mapping;
 * it is simply unable to fail closed on the whole of one.
 */
static inline enum ca_ni_rx_pool
ca_ni_rx_buf_locate(u32 pa, u32 base, const struct ca_ni_rx_pool_geom *g,
		    struct ca_ni_rx_buf *out)
{
	u32 off = pa - base;

	out->off_in_region = 0;
	out->buf_max = 0;
	out->rpa = 0;

	if (off >= g->map_size)
		return CA_NI_RX_POOL_BAD;

	out->off_in_region = off;

	if (off < g->pool0_bytes) {
		out->buf_max = ca_ni_rx_usable_end(g->pool0_bufsz, g->tailroom);
		out->rpa = pa;
		return CA_NI_RX_POOL_CPU0;
	}
	if (off < g->dq_off) {
		out->buf_max = ca_ni_rx_usable_end(g->pool1_bufsz, g->tailroom);
		out->rpa = pa;
		return CA_NI_RX_POOL_CPU1;
	}
	out->buf_max = ca_ni_rx_usable_end(g->dq_bufsz, g->tailroom);
	out->rpa = 0;			/* hardware-managed: never recycle it */
	return CA_NI_RX_POOL_DQ;
}

/* ------------------------------------------------------------------ */
/* 2. WHERE THE FRAME STARTS, HOW LONG IT IS, AND IS IT WELL FORMED    */
/* ------------------------------------------------------------------ */

enum ca_ni_rx_geom_verdict {
	CA_NI_RX_GEOM_OK = 0,
	CA_NI_RX_GEOM_RUNT,	/* shorter than an Ethernet header: a bad frame */
	CA_NI_RX_GEOM_OVERSIZE,	/* a good frame that does not fit the usable window */
};

/*
 * The buffer's fixed layout, all of it the shell's:
 * @hdra_off:     offset of the descriptor-side header block
 * @frame_off:    offset the frame starts at when that header block is present
 * @hdr_cpu_len:  extra bytes inserted ahead of the frame when the CPU-header
 *                flag is set
 * @desc_hdr_len: frame offset in the HEADERLESS format
 * @eth_hlen:     the runt floor
 */
struct ca_ni_rx_layout {
	u32 hdra_off;
	u32 frame_off;
	u32 hdr_cpu_len;
	u32 desc_hdr_len;
	u32 eth_hlen;
};

/*
 * @off:   where the frame starts inside the buffer
 * @len:   how many bytes of frame there are.  SIGNED on purpose: the two
 *         subtractions below can drive it negative on a malformed descriptor,
 *         and that is precisely the RUNT the caller must count rather than
 *         silently turn into a ~4 GB copy.
 * @stale: the two INDEPENDENT length witnesses disagree (see below)
 */
struct ca_ni_rx_geom {
	u32 off;
	int len;
	bool stale;
};

/**
 * ca_ni_rx_frame_geom() - decide a received frame's offset, length and validity.
 * @l:             the buffer layout (shell constants)
 * @swid:          the descriptor's software id; 0 selects the header-A format
 * @dlen:          the DESCRIPTOR's own length field
 * @hdra_cpu_flg:  header-A says a CPU header block is present
 * @hdra_pkt_size: header-A's packet size, RAW (before any header subtraction)
 * @buf_max:       end of the buffer's usable window, from ca_ni_rx_buf_locate()
 * @out:           filled on every return, verdict or not
 *
 * ★ THE STALENESS WITNESS IS FREE, AND IT IS THE POINT OF TAKING @dlen HERE.
 * The descriptor's length and header-A's packet size describe the SAME frame
 * from two INDEPENDENT writers -- the descriptor ring and the frame DMA -- and
 * on a good frame differ by exactly the header block (@frame_off - @hdra_off).
 * A mismatch means the buffer no longer holds the frame this descriptor was
 * written for.  It is REPORTED, never acted on: dropping would turn a duplicate
 * into a hole, the datagram fails either way, and a silent behaviour change
 * would muddy the A/B this counter exists to serve.  It is meaningful only in
 * the header-A format, so the headerless path always reports false.
 *
 * ★ RUNT AND OVERSIZE ARE SPLIT because the two causes mean opposite things and
 * an aggregate cannot tell them apart: a runt is a bad frame, an oversize one is
 * a good frame that needs the multi-buffer path.
 *
 * Return: OK, RUNT or OVERSIZE.  @out->off / @out->len are the derived geometry
 * either way; the caller may only USE them on OK.
 */
static inline enum ca_ni_rx_geom_verdict
ca_ni_rx_frame_geom(const struct ca_ni_rx_layout *l, u32 swid, u32 dlen,
		    bool hdra_cpu_flg, int hdra_pkt_size, u32 buf_max,
		    struct ca_ni_rx_geom *out)
{
	u32 off;
	int len;

	out->stale = false;

	if (!swid) {
		len = hdra_pkt_size;
		out->stale = ((u32)(len + (int)(l->frame_off - l->hdra_off)) !=
			      dlen);
		off = l->frame_off;
		if (hdra_cpu_flg) {
			off += l->hdr_cpu_len;
			len -= (int)l->hdr_cpu_len;
		}
	} else {
		/* headerless format: frame at a fixed offset, length from the
		 * descriptor.  There is no second witness here, so no staleness
		 * verdict is available -- and none is invented. */
		off = l->desc_hdr_len;
		len = (int)dlen - (int)l->desc_hdr_len;
	}

	out->off = off;
	out->len = len;

	if (len < (int)l->eth_hlen)
		return CA_NI_RX_GEOM_RUNT;
	/* len >= eth_hlen here, so the widening to u32 cannot go wrong */
	if (off + (u32)len > buf_max)
		return CA_NI_RX_GEOM_OVERSIZE;
	return CA_NI_RX_GEOM_OK;
}

/* ------------------------------------------------------------------ */
/* 3. THE PDC's CONTROL PUNT: is this a PON control frame, and where   */
/*    is the PDU inside it                                             */
/* ------------------------------------------------------------------ */

/* @off/@len are relative to the START OF THE FRAME the caller passed in. */
struct ca_ni_pon_pdu {
	u32 off;
	u32 len;
};

/**
 * ca_ni_rx_pon_ctrl() - is this the PDC's control punt, and what is its PDU?
 * @frame:     the first byte of the received Ethernet frame
 * @len:       how many bytes of it there are
 * @lnk_type:  the vendor link-type marker the PDC stamps on a control frame
 * @hdr_len:   how many bytes the PDC prepends ahead of the PDU
 * @out:       {0,0} unless a PDU is present
 *
 * The marker sits where an ethertype would, and is read with explicit byte math
 * so the answer is the same on either byte order.  A real Ethernet frame cannot
 * false-match: the caller's marker is not a valid ethertype on this LAN.
 *
 * ★ THE LENGTH GATE IS THE REASON THIS IS THREE LINES AND STILL WORTH LIFTING.
 * It is the single place a bad bound would hand a NEGATIVE length to the OMCI
 * core: @out->len is published only when @len is STRICTLY greater than
 * @hdr_len, so a control frame that carries no PDU yields {0,0} and a caller
 * that checks @out->len can never compute a wrapped size.
 *
 * The `len < 14` guard is not a policy: 14 is simply the smallest length at
 * which @frame[13] is inside the buffer at all.
 *
 * Return: true when the frame IS a control punt -- whether or not it carried a
 * PDU, because the two are different facts and the caller counts the first.
 */
static inline bool
ca_ni_rx_pon_ctrl(const u8 *frame, int len, u16 lnk_type, u32 hdr_len,
		  struct ca_ni_pon_pdu *out)
{
	out->off = 0;
	out->len = 0;

	if (!frame || len < 14)
		return false;
	if ((u16)(((u16)frame[12] << 8) | frame[13]) != lnk_type)
		return false;

	if ((u32)len > hdr_len) {
		out->off = hdr_len;
		out->len = (u32)len - hdr_len;
	}
	return true;
}

#endif /* _CORTINA_NI_RX_GEOM_H */
