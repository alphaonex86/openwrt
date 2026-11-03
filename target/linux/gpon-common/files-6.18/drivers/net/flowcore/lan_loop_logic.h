/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * lan_loop_logic.h -- LAN loop detection, the pure half: the probe a switch
 * shell sends out of each LAN port, and the verdict on a frame that comes back.
 * No MMIO, no lock, no clock: the shell owns the timer, the TX and the port block.
 *
 * Probe: broadcast, source = our own MAC, IEEE 802 local experimental ethertype,
 * then a per-boot random id and the port it left by. Our own probe arriving on a
 * LAN port means a path leads from that port back to us: a loop.
 */
#ifndef _LAN_LOOP_LOGIC_H
#define _LAN_LOOP_LOGIC_H

#include <linux/types.h>
#include <linux/string.h>

#define LAN_LOOP_ETHERTYPE	0x88B5	/* IEEE Std 802 local experimental 1 */
#define LAN_LOOP_FRAME_LEN	60
#define LAN_LOOP_ID_LEN		4
#define LAN_LOOP_OFF_ID		14
#define LAN_LOOP_OFF_PORT	(LAN_LOOP_OFF_ID + LAN_LOOP_ID_LEN)

static inline void lan_loop_probe_build(u8 *f, const u8 *own_mac, const u8 *id,
					u8 tx_port)
{
	memset(f, 0, LAN_LOOP_FRAME_LEN);
	memset(f, 0xff, 6);
	memcpy(f + 6, own_mac, 6);
	f[12] = LAN_LOOP_ETHERTYPE >> 8;
	f[13] = LAN_LOOP_ETHERTYPE & 0xff;
	memcpy(f + LAN_LOOP_OFF_ID, id, LAN_LOOP_ID_LEN);
	f[LAN_LOOP_OFF_PORT] = tx_port;
}

/* -> the port our probe @f left by, or -1 when @f is not one of our probes. */
static inline int lan_loop_probe_tx_port(const u8 *f, unsigned int len,
					 const u8 *own_mac, const u8 *id)
{
	if (len <= LAN_LOOP_OFF_PORT || memcmp(f + 6, own_mac, 6) ||
	    f[12] != (LAN_LOOP_ETHERTYPE >> 8) ||
	    f[13] != (LAN_LOOP_ETHERTYPE & 0xff) ||
	    memcmp(f + LAN_LOOP_OFF_ID, id, LAN_LOOP_ID_LEN))
		return -1;
	return f[LAN_LOOP_OFF_PORT];
}

/* -> the LAN port to block for a probe sent on @tx_port and received on
 * @rx_port, or -1. Both must be LAN ports (@lan_mask). The receiving port is
 * blocked, unless either end already is: the probe that crossed a two-port
 * loop the other way must not block its second port too. */
static inline int lan_loop_port_to_block(int tx_port, unsigned int rx_port,
					 u32 lan_mask, u32 blocked)
{
	if (tx_port < 0 || tx_port >= 32 || rx_port >= 32 ||
	    !(lan_mask & (1u << tx_port)) || !(lan_mask & (1u << rx_port)))
		return -1;
	if (blocked & ((1u << tx_port) | (1u << rx_port)))
		return -1;
	return (int)rx_port;
}

#endif /* _LAN_LOOP_LOGIC_H */
