/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CHIP */
/* zx279128_gpon_desc.h -- the transmit descriptor of the ZX279128 PON ring, PURE (host-tested): word 1 bits 26-24
 * the queue (7 for OMCI), bits 31-27 the T-CONT, word 2 bits 13-2 the GEM port, bit 31 OMCI, and word 4 zero.
 */
#ifndef _ZX279128_GPON_DESC_H
#define _ZX279128_GPON_DESC_H

#include "zx279128_eth_logic.h"

#define ZX_TXD_PON_W2		0x20000000u
#define ZX_TXD_PON_OMCI		0x80000000u	/* word 2 */
#define ZX_TXD_PON_W3		0x10000000u
#define ZX_TXD_OMCI_QUEUE	7u
#define ZX_TXD_GEM_MAX		0xfffu
#define ZX_TXD_TCONT_MAX	31u

/* A frame of len bytes at byte `offset` of buffer `buf` (buf_size bytes) for GEM port `gem` (0-4095) of T-CONT
 * `tcont` (0-31); an OMCI message takes the OMCI queue. -> false when it cannot be described, and d is left zero. */
static inline bool zx279128_pon_tx_desc(u32 d[ZX_TX_DESC_WORDS], u32 buf, u32 offset, u32 len, u32 buf_size, u16 gem,
					unsigned int tcont, bool omci)
{
	if (!zx279128_tx_fits(d, buf, offset, len, buf_size) || gem > ZX_TXD_GEM_MAX || tcont > ZX_TXD_TCONT_MAX)
		return false;
	d[1] = (tcont << 3 | (omci ? ZX_TXD_OMCI_QUEUE : 0)) << 24 | 1;
	d[2] = ZX_TXD_PON_W2 | (omci ? ZX_TXD_PON_OMCI : 0) | buf << 14 | gem << 2;
	d[3] = ZX_TXD_PON_W3 | len << 13 | offset << 7;
	return true;
}

#endif
