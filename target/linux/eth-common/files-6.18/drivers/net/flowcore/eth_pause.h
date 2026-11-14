/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_pause.h -- a LAN port's 802.3x rule, for every ONU (operator, 2026-11-14): it never SENDS pause (it drops at
 * the tail instead) unless ethtool -A asks, and it honours the pause it receives. Pure, host-tested; a driver gives
 * its MAC's own flow-control bits. */
#ifndef _ETH_PAUSE_H
#define _ETH_PAUSE_H

#include <linux/types.h>

#define ETH_PAUSE_TX_DEFAULT	false
#define ETH_PAUSE_RX_DEFAULT	true

/* A MAC whose flow-control directions are fields of a force-mode word: a direction set off is forced (its value
 * cleared by eth_pause_force_value), one set on follows the negotiation. */
static inline u32 eth_pause_force_mode(u32 mode, u32 tx_bit, u32 rx_bit, bool tx, bool rx)
{
	return (mode & ~(tx_bit | rx_bit)) | (tx ? 0 : tx_bit) | (rx ? 0 : rx_bit);
}

static inline u32 eth_pause_force_value(u32 value, u32 tx_bit, u32 rx_bit)
{
	return value & ~(tx_bit | rx_bit);
}

/* ethtool's advertisement for (tx, rx): Pause = rx, Asym_Pause = tx != rx (IEEE 802.3 Annex 28B). */
static inline void eth_pause_adv(bool tx, bool rx, bool *sym, bool *asym)
{
	*sym = rx;
	*asym = tx != rx;
}

/* The audit: a port with link whose MAC runs TX pause although it is set off. */
static inline bool eth_pause_tx_leaks(bool link, bool tx_running, bool tx)
{
	return !tx && link && tx_running;
}

#ifdef __KERNEL__
#include <linux/ethtool.h>
#include <linux/linkmode.h>

static inline void eth_pause_advertise(unsigned long *advertising, bool tx, bool rx)
{
	bool sym, asym;

	eth_pause_adv(tx, rx, &sym, &asym);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_Pause_BIT, advertising, sym);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, advertising, asym);
}
#endif

#endif
