/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_rx_csum.h -- the RX checksum verdict a chip writes into its descriptor, decoded once for every chip.
 *
 * A chip states where its parsed-protocol field and its failure flags sit. The verdict is good only for a
 * protocol the table names whose checked sums passed; an empty table never answers good. PURE.
 */
#ifndef _ETH_RX_CSUM_H
#define _ETH_RX_CSUM_H

#include <linux/types.h>

struct eth_rx_csum_layout {
	u8 proto_shift;		/* the parsed protocol: (word >> proto_shift) & proto_mask */
	u8 proto_mask;
	u32 v4_protos;		/* one bit per protocol value whose IPv4 header and L4 sums the chip checked */
	u32 v6_protos;		/* one bit per protocol value whose L4 sum the chip checked */
	u32 v4_failed;		/* flags that void a verdict on IPv4 */
	u32 v6_failed;		/* flags that void a verdict on IPv6 */
	u32 never;		/* flags that void any verdict */
};

static inline bool eth_rx_csum_ok(const struct eth_rx_csum_layout *l, u32 word)
{
	unsigned int proto = (word >> l->proto_shift) & l->proto_mask;

	if (proto > 31 || (word & l->never))
		return false;
	if (l->v4_protos & (1u << proto))
		return !(word & l->v4_failed);
	if (l->v6_protos & (1u << proto))
		return !(word & l->v6_failed);
	return false;
}

#endif /* _ETH_RX_CSUM_H */
