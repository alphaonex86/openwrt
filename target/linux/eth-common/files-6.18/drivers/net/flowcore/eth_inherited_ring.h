/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_inherited_ring.h -- a DMA ring an earlier owner (a boot loader, a previous kernel) left to an engine still
 * running: which slots the engine owns and where their buffers lie, which it filled that nobody took. The caller
 * proves the ring is memory and maps it; the walk stops at the ring's wrap slot. PURE.
 */
#ifndef _ETH_INHERITED_RING_H
#define _ETH_INHERITED_RING_H

#include <linux/types.h>

struct eth_inherited_ring {
	unsigned int slots, armed, filled;
	u32 buf_lo, buf_hi;	/* the buffers of the armed slots, buf_hi exclusive; 0..0 when none */
};

/* Descriptors of `stride` bytes, native-endian words: the control word at `ctrl_off` carries `own` (the engine's)
 * and `wrap` (the last slot), the buffer's bus address at `buf_off`. */
static inline void eth_inherited_ring_walk(struct eth_inherited_ring *r, const u8 *ring, unsigned int n,
					   unsigned int stride, unsigned int ctrl_off, unsigned int buf_off,
					   u32 own, u32 wrap, u32 buf_mask, u32 buf_bytes)
{
	*r = (struct eth_inherited_ring){ .buf_lo = 0xffffffffu };
	while (r->slots < n) {
		const u8 *d = ring + r->slots++ * stride;
		u32 ctrl = *(const volatile u32 *)(d + ctrl_off);
		u32 buf = *(const volatile u32 *)(d + buf_off) & buf_mask;

		if (ctrl & own) {
			r->armed++;
			r->buf_lo = buf < r->buf_lo ? buf : r->buf_lo;
			r->buf_hi = buf + buf_bytes > r->buf_hi ? buf + buf_bytes : r->buf_hi;
		} else {
			r->filled++;
		}
		if (ctrl & wrap)
			break;
	}
	if (!r->armed)
		r->buf_lo = 0;
}

#endif /* _ETH_INHERITED_RING_H */
