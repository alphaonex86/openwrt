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
