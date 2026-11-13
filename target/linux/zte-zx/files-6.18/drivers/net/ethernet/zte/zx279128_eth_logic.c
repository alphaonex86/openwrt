// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/* zx279128_eth_logic.c -- descriptor decode and build for the ZX279128 Ethernet path. PURE. */
#include "zx279128_eth_logic.h"

#define ZX_TXD_W1		0x80000001u
#define ZX_TXD_W2		0x20000002u
#define ZX_TXD_W3		0x90000000u

/* word 2 bits 28-14 the buffer, word 3 bits 26-13 the length and bits 12-7 the offset */
static const struct eth_tx_limits zx_tx_max = { .buf = 0x7fff, .len = 0x3fff, .offset = 0x3f };

const struct eth_rx_layout zx279128_rx_layout = {
	.prefix = ZX_BUF_OFFSET, .head = 0, .fcs = ETH_RX_FCS_NOT_IN_LEN, .buf = ZX_BUF_SIZE,
};

enum zx_rx_verdict zx279128_rx_parse(const u32 w[ZX_RX_DESC_WORDS], const struct eth_rx_layout *l,
				     u32 normal_bufs, u32 jumbo_bufs, bool pon_ring, struct zx_rx *rx)
{
	unsigned int src = (w[1] & ZX_RXD_PORT_MASK) >> 25;
	bool omci = w[1] & ZX_RXD_OMCI;

	rx->jumbo = w[1] & ZX_RXD_JUMBO;
	rx->buf = w[2] & ZX_RXD_BUF_MASK;
	rx->port = 0;
	rx->gem = 0;
	rx->geom = ETH_RX_UNMEASURED;
	if (eth_rx_desc_unwritten(w, ZX_RX_DESC_WORDS))
		return ZX_RX_UNWRITTEN;
	if (rx->buf >= (rx->jumbo ? jumbo_bufs : normal_bufs))
		return ZX_RX_BAD_BUFFER;
	if (!omci && !pon_ring && (src < 1 || src > ZX_ETH_PORTS))
		return ZX_RX_NOT_LAN;
	rx->geom = eth_rx_frame_geom(l, eth_rx_dlen(w[2], ZX_RXD_LEN_MASK), true, &rx->fr);
	if (rx->geom != ETH_RX_OK)
		return ZX_RX_BAD_FRAME;
	if (omci || pon_ring) {
		rx->gem = (w[1] & ZX_RXD_GEM_MASK) >> 10;
		return omci ? ZX_RX_OMCI : ZX_RX_GEM;
	}
	rx->port = src - 1;
	return ZX_RX_OK;
}

bool zx279128_tx_fits(u32 d[ZX_TX_DESC_WORDS], u32 buf, u32 offset, u32 len, u32 buf_size)
{
	unsigned int i;

	for (i = 0; i < ZX_TX_DESC_WORDS; i++)
		d[i] = 0;
	return eth_tx_fits(&zx_tx_max, buf, offset, len, buf_size);
}

bool zx279128_tx_desc(u32 d[ZX_TX_DESC_WORDS], u32 buf, u32 offset, u32 len, u32 buf_size, unsigned int port)
{
	if (!zx279128_tx_fits(d, buf, offset, len, buf_size) || port >= ZX_ETH_PORTS)
		return false;
	d[1] = ZX_TXD_W1;
	d[2] = ZX_TXD_W2 | buf << 14;
	d[3] = ZX_TXD_W3 | len << 13 | offset << 7 | (port + 1);
	d[4] = len;
	return true;
}
