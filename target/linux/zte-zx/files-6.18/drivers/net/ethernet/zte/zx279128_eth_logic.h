/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CHIP */
/* zx279128_eth_logic.h -- the ZX279128 packet processor's Ethernet descriptors, PURE (host-tested).
 *
 * Receive: 16-byte descriptors, word 1 bits 29-25 = source port + 1, bit 24 OMCI, bit 31 jumbo pool; word 2
 * bits 14-0 the buffer, bits 28-15 the length. Transmit: 32-byte descriptors, word 2 bits 28-14 the buffer,
 * word 3 bits 26-13 the length, bits 12-7 the byte where the frame starts in its buffer, bits 3-0 the egress
 * port + 1; word 4 bits 13-0 the length again. Facts: the stock kernel's pp_net_poll / pp_net_tx and the
 * unit's U-Boot network driver (ONU-F680.md).
 */
#ifndef _ZX279128_ETH_LOGIC_H
#define _ZX279128_ETH_LOGIC_H

#include <linux/types.h>
#include "eth_rx_geom.h"
#include "eth_rx_ring.h"

#define ZX_ETH_PORTS		4
#define ZX_RX_DESC_WORDS	4
#define ZX_TX_DESC_WORDS	8
#define ZX_RXD_LEN_MASK		0x1fff8000u	/* word 2 */
#define ZX_RXD_BUF_MASK		0x00007fffu	/* word 2 */
#define ZX_RXD_PORT_MASK	0x3e000000u	/* word 1 */
#define ZX_RXD_OMCI		(1u << 24)	/* word 1 */
#define ZX_RXD_JUMBO		(1u << 31)	/* word 1 */
#define ZX_BUF_SIZE		0x800
#define ZX_BUF_OFFSET		16	/* where a frame starts in its buffer, both ways, once 0x92140060 = 0x11 */

/* MEASURED on F680 unit 2, 2026-11-12: its U-Boot's receive descriptors gave 60 for the host's 42-byte ARP
 * and ICMP replies (padded to 60 on the wire): the length excludes the FCS. */
extern const struct eth_rx_layout zx279128_rx_layout;

enum zx_rx_verdict {
	ZX_RX_OK = 0,
	ZX_RX_NOT_LAN,		/* OMCI, or no Ethernet source port */
	ZX_RX_BAD_BUFFER,	/* a buffer index outside its pool */
	ZX_RX_BAD_FRAME,	/* the length, judged by eth_rx_geom.h (rx->geom says how) */
	ZX_RX_UNWRITTEN,	/* all zero: the hardware wrote nothing here, it names NO buffer */
};

struct zx_rx {
	unsigned int port;	/* 0-3 */
	bool jumbo;		/* the buffer is the jumbo pool's */
	u32 buf;		/* its index in that pool */
	struct eth_rx_frame fr;
	enum eth_rx_verdict geom;
};

/* One receive descriptor; the buffer fields are filled whenever the index is valid, so the caller can free it. */
enum zx_rx_verdict zx279128_rx_parse(const u32 w[ZX_RX_DESC_WORDS], const struct eth_rx_layout *l,
				     u32 normal_bufs, u32 jumbo_bufs, struct zx_rx *rx);
/* A frame of len bytes at byte `offset` of buffer `buf` (buf_size bytes) to port 0-3. -> false when it cannot be
 * described, and d is left zero. */
bool zx279128_tx_desc(u32 d[ZX_TX_DESC_WORDS], u32 buf, u32 offset, u32 len, u32 buf_size, unsigned int port);

#endif
