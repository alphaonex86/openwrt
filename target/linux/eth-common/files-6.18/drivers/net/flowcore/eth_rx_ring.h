/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_rx_ring.h -- a receive ring whose hardware write position no register tells and no reset clears (a boot
 * loader's traffic leaves it mid-ring): the reader zeroes each descriptor it takes, so the written descriptors
 * ARE the pending frames and the position is found in the ring itself. PURE.
 */
#ifndef _ETH_RX_RING_H
#define _ETH_RX_RING_H

#include <linux/types.h>

/* A descriptor of `words` words the hardware has not written since the reader zeroed it: it names no buffer. */
static inline bool eth_rx_desc_unwritten(const u32 *desc, unsigned int words)
{
	unsigned int i;

	for (i = 0; i < words; i++)
		if (desc[i])
			return false;
	return true;
}

/* Where the pending frames start: the written descriptor that follows an unwritten one.
 * -> -1 when none or all of the `slots` descriptors are written. */
static inline int eth_rx_ring_run_start(const u32 *ring, unsigned int slots, unsigned int words)
{
	unsigned int i;

	for (i = 0; i < slots; i++)
		if (!eth_rx_desc_unwritten(ring + i * words, words) &&
		    eth_rx_desc_unwritten(ring + (i ? i - 1 : slots - 1) * words, words))
			return i;
	return -1;
}

#endif
