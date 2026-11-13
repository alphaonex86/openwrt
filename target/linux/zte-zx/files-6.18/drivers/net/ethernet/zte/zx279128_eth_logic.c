// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/* zx279128_eth_logic.c -- descriptor decode and build for the ZX279128 Ethernet path. PURE. */
#include "zx279128_eth_logic.h"

#define ZX_TXD_W1		0x80000001u
#define ZX_TXD_W2		0x20000002u
#define ZX_TXD_W3		0x90000000u
#define ZX_TXD_BUF_MAX		0x7fffu		/* word 2 bits 28-14 */
#define ZX_TXD_LEN_MAX		0x3fffu		/* word 3 bits 26-13, word 4 bits 13-0 */
#define ZX_TXD_OFFSET_MAX	0x3fu		/* word 3 bits 12-7 */

const struct eth_rx_layout zx279128_rx_layout = {
	.prefix = ZX_BUF_OFFSET, .head = 0, .fcs = ETH_RX_FCS_NOT_IN_LEN, .buf = ZX_BUF_SIZE,
};

enum zx_rx_verdict zx279128_rx_parse(const u32 w[ZX_RX_DESC_WORDS], const struct eth_rx_layout *l,
				     u32 normal_bufs, u32 jumbo_bufs, struct zx_rx *rx)
{
	unsigned int src = (w[1] & ZX_RXD_PORT_MASK) >> 25;

	rx->jumbo = w[1] & ZX_RXD_JUMBO;
	rx->buf = w[2] & ZX_RXD_BUF_MASK;
	rx->port = 0;
	rx->geom = ETH_RX_UNMEASURED;
	if (eth_rx_desc_unwritten(w, ZX_RX_DESC_WORDS))
		return ZX_RX_UNWRITTEN;
	if (rx->buf >= (rx->jumbo ? jumbo_bufs : normal_bufs))
		return ZX_RX_BAD_BUFFER;
	if ((w[1] & ZX_RXD_OMCI) || src < 1 || src > ZX_ETH_PORTS)
		return ZX_RX_NOT_LAN;
	rx->port = src - 1;
	rx->geom = eth_rx_frame_geom(l, eth_rx_dlen(w[2], ZX_RXD_LEN_MASK), true, &rx->fr);
	return rx->geom == ETH_RX_OK ? ZX_RX_OK : ZX_RX_BAD_FRAME;
}

bool zx279128_tx_desc(u32 d[ZX_TX_DESC_WORDS], u32 buf, u32 offset, u32 len, u32 buf_size, unsigned int port)
{
	unsigned int i;

	for (i = 0; i < ZX_TX_DESC_WORDS; i++)
		d[i] = 0;
	if (port >= ZX_ETH_PORTS || buf > ZX_TXD_BUF_MAX || offset > ZX_TXD_OFFSET_MAX || !len ||
	    len > ZX_TXD_LEN_MAX || offset >= buf_size || len > buf_size - offset)
		return false;
	d[1] = ZX_TXD_W1;
	d[2] = ZX_TXD_W2 | buf << 14;
	d[3] = ZX_TXD_W3 | len << 13 | offset << 7 | (port + 1);
	d[4] = len;
	return true;
}
