// SPDX-License-Identifier: GPL-2.0-only
/* Luna family: switch and MAC statistics over `ethtool -S`. ★ ...
 * dev/MEASURED-luna_eth_mib.c.md sec 1. */

#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include "luna_eth_regs.h"
#include "luna_eth_mib.h"

/* One countable quantity: its published name and where it sits inside the
 * per-port window.  `wide` = a 64-bit counter stored as low word at `off` and
 * high word at `off + 4`. */
struct luna_mib_field {
	const char	*name;
	u16		off;
	bool		wide;
};

/* ── STAT_PORT_RX_MIB, one 0x80 window per port ... -- dev/MEASURED-luna_eth_mib.c.md sec 2. */
static const struct luna_mib_field luna_mib_rx[] = {
	{ "rx_octets",		0x00, true },	/* ifInOctets, 64-bit pair	*/
	{ "rx_crc_align_err",	0x08 },		/* etherStatsCRCAlignErrors	*/
	{ "rx_symbol_err",	0x0c },		/* dot3StatsSymbolErrors	*/
	{ "rx_pause",		0x10 },		/* dot3InPauseFrames		*/
	{ "rx_unknown_opcode",	0x14 },		/* dot3ControlInUnknownOpcodes	*/
	{ "rx_fragments",	0x18 },		/* etherStatsFragments		*/
	{ "rx_jabbers",		0x1c },		/* etherStatsJabbers		*/
	{ "rx_unicast",		0x20 },		/* ifInUcastPkts		*/
	{ "rx_drop_events",	0x24 },		/* etherStatsDropEvents		*/
	{ "rx_multicast",	0x28 },		/* ifInMulticastPkts		*/
	{ "rx_broadcast",	0x2c },		/* ifInBroadcastPkts		*/
	{ "rx_1519_to_max_frames", 0x30 },
	{ "rx_undersize",	0x38 },		/* etherStatsUndersizePkts	*/
	{ "rx_oversize",	0x3c },		/* etherStatsOversizePkts	*/
	{ "rx_64_frames",	0x40 },
	{ "rx_65_to_127_frames", 0x44 },
	{ "rx_128_to_255_frames", 0x48 },
	{ "rx_256_to_511_frames", 0x4c },
	{ "rx_512_to_1023_frames", 0x50 },
	{ "rx_1024_to_1518_frames", 0x54 },
};

/* ── STAT_PORT_TX_MIB, base 0x32000 on every Luna die ...
 * dev/MEASURED-luna_eth_mib.c.md sec 3. */
static const struct luna_mib_field luna_mib_tx[] = {
	{ "tx_multicast_ethstat", 0x00 },	/* etherStatsMulticastPkts	*/
	{ "tx_broadcast_ethstat", 0x04 },	/* etherStatsBroadcastPkts	*/
	{ "tx_undersize",	0x08 },
	{ "tx_oversize",	0x0c },
	{ "tx_64_frames",	0x10 },
	{ "tx_65_to_127_frames", 0x14 },
	{ "tx_128_to_255_frames", 0x18 },
	{ "tx_256_to_511_frames", 0x1c },
	{ "tx_512_to_1023_frames", 0x20 },
	{ "tx_1024_to_1518_frames", 0x24 },
	{ "tx_octets",		0x28, true },	/* ifOutOctets, 64-bit pair	*/
	{ "tx_single_collision", 0x30 },	/* dot3StatsSingleCollisionFrames */
	{ "tx_multi_collision",	0x34 },		/* dot3StatsMultipleCollisionFrames */
	{ "tx_deferred",	0x38 },		/* dot3StatsDeferredTransmissions */
	{ "tx_late_collision",	0x3c },		/* dot3StatsLateCollisions	*/
	{ "tx_collisions",	0x40 },		/* etherStatsCollisions		*/
	{ "tx_excess_collision", 0x44 },	/* dot3StatsExcessiveCollisions	*/
	{ "tx_pause",		0x48 },		/* dot3OutPauseFrames		*/
	{ "tx_discards",	0x4c },		/* ifOutDiscards		*/
	{ "tx_1519_to_max_frames", 0x50 },
	{ "tx_bridge_discards",	0x58 },		/* dot1dTpPortInDiscards	*/
	{ "tx_unicast",		0x5c },		/* ifOutUcastPkts		*/
	{ "tx_multicast",	0x60 },		/* ifOutMulticastPkts		*/
	{ "tx_broadcast",	0x64 },		/* ifOutBroadcastPkts		*/
};

/* ── the MAC-level MIB: fourteen SIXTEEN-bit counters, packed ...
 * dev/MEASURED-luna_eth_mib.c.md sec 4. */
static const char * const luna_mac_mib[] = {
	"mac_txok",	"mac_rxok",
	"mac_txerr",	"mac_rxerr",
	"mac_misspkt",	"mac_fae",
	"mac_tx1col",	"mac_txmcol",
	"mac_rxokphy",	"mac_rxokbrd",
	"mac_rxokmul",	"mac_txabt",
	"mac_txundrn",	"mac_rdumisspkt",
};
#define LUNA_MAC_MIB_N	ARRAY_SIZE(luna_mac_mib)
static_assert(LUNA_MAC_MIB_N % 2 == 0,
	      "the MAC MIB is read two counters per 32-bit word");

/* THE ONE BOUND INSTANCE. Both shells are already ... -- dev/MEASURED-luna_eth_mib.c.md sec 6. */
struct luna_mib_state {
	void __iomem		*sw;
	void __iomem		*mac;
	const struct luna_sw_map *map;
	spinlock_t		lock;		/* guards the accumulator	*/
	unsigned long		tick_jiffies;
	bool			tick_seen;
	u16			last[LUNA_MAC_MIB_N];
	u64			total[LUNA_MAC_MIB_N];
};
static struct luna_mib_state luna_mib = { .lock = __SPIN_LOCK_UNLOCKED(luna_mib.lock) };

static bool luna_mib_has_rx(const struct luna_mib_state *s)
{
	return s->map && s->map->rx_mib && s->map->n_mib_ports;
}

static bool luna_mib_has_mac(const struct luna_mib_state *s)
{
	return s->mac && s->map && s->map->gmac_mib16;
}

static unsigned int luna_mib_ports(const struct luna_mib_state *s)
{
	return (s->map && s->sw) ? s->map->n_mib_ports : 0;
}

static unsigned int luna_mib_per_port(const struct luna_mib_state *s)
{
	return ARRAY_SIZE(luna_mib_tx) +
	       (luna_mib_has_rx(s) ? ARRAY_SIZE(luna_mib_rx) : 0);
}

static int luna_mib_count(const struct luna_mib_state *s)
{
	return luna_mib_ports(s) * luna_mib_per_port(s) +
	       (luna_mib_has_mac(s) ? (int)LUNA_MAC_MIB_N : 0);
}

/* One 32-bit or 64-bit counter out of a per-port window. The ...
 * dev/MEASURED-luna_eth_mib.c.md sec 7. */
static u64 luna_mib_read(void __iomem *sw, u32 base, unsigned int port,
			 const struct luna_mib_field *f)
{
	u32 at = base + port * SW_STAT_PORT_MIB_STRIDE + f->off;
	u32 lo, hi, lo2;

	if (!f->wide)
		return ioread32(sw + at);

	lo = ioread32(sw + at);
	hi = ioread32(sw + at + 4);
	lo2 = ioread32(sw + at);
	if (lo2 < lo)
		hi = ioread32(sw + at + 4);
	return ((u64)hi << 32) | lo2;
}

/* Fold the 16-bit MAC counters into 64-bit totals. Exact ...
 * dev/MEASURED-luna_eth_mib.c.md sec 5. */
void luna_mib_gmac_tick(void)
{
	struct luna_mib_state *s = &luna_mib;
	unsigned long flags, now = jiffies;
	unsigned int w;

	if (!luna_mib_has_mac(s) || s->tick_jiffies == now)
		return;

	spin_lock_irqsave(&s->lock, flags);
	if (s->tick_jiffies != now) {
		s->tick_jiffies = now;
		for (w = 0; w < LUNA_MAC_MIB_N / 2; w++) {
			u32 word = ioread32(s->mac + s->map->gmac_mib16 + w * 4);
			u16 cur[2] = { (u16)(word >> 16), (u16)word };
			unsigned int h;

			for (h = 0; h < 2; h++) {
				unsigned int i = w * 2 + h;

				if (s->tick_seen)
					s->total[i] += (u16)(cur[h] - s->last[i]);
				s->last[i] = cur[h];
			}
		}
		s->tick_seen = true;
	}
	spin_unlock_irqrestore(&s->lock, flags);
}
EXPORT_SYMBOL_GPL(luna_mib_gmac_tick);

static int luna_mib_get_sset_count(struct net_device *ndev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;
	return luna_mib_count(&luna_mib);
}

static void luna_mib_get_strings(struct net_device *ndev, u32 sset, u8 *data)
{
	const struct luna_mib_state *s = &luna_mib;
	unsigned int port, i, n = luna_mib_ports(s);

	if (sset != ETH_SS_STATS)
		return;

	for (port = 0; port < n; port++) {
		if (luna_mib_has_rx(s))
			for (i = 0; i < ARRAY_SIZE(luna_mib_rx); i++)
				ethtool_sprintf(&data, "p%u_%s", port,
						luna_mib_rx[i].name);
		for (i = 0; i < ARRAY_SIZE(luna_mib_tx); i++)
			ethtool_sprintf(&data, "p%u_%s", port,
					luna_mib_tx[i].name);
	}
	if (luna_mib_has_mac(s))
		for (i = 0; i < LUNA_MAC_MIB_N; i++)
			ethtool_puts(&data, luna_mac_mib[i]);
}

static void luna_mib_get_stats(struct net_device *ndev,
			       struct ethtool_stats *stats, u64 *data)
{
	struct luna_mib_state *s = &luna_mib;
	unsigned int port, i, n = luna_mib_ports(s);
	unsigned long flags;

	for (port = 0; port < n; port++) {
		if (luna_mib_has_rx(s))
			for (i = 0; i < ARRAY_SIZE(luna_mib_rx); i++)
				*data++ = luna_mib_read(s->sw, s->map->rx_mib,
							port, &luna_mib_rx[i]);
		for (i = 0; i < ARRAY_SIZE(luna_mib_tx); i++)
			*data++ = luna_mib_read(s->sw, SW_STAT_PORT_TX_MIB,
						port, &luna_mib_tx[i]);
	}
	if (!luna_mib_has_mac(s))
		return;
	/* Fold in whatever has happened since the last tick, so a read taken
	 * between two ticks is not a stale total. */
	luna_mib_gmac_tick();
	spin_lock_irqsave(&s->lock, flags);
	for (i = 0; i < LUNA_MAC_MIB_N; i++)
		*data++ = s->total[i];
	spin_unlock_irqrestore(&s->lock, flags);
}

static void luna_mib_get_drvinfo(struct net_device *ndev,
				 struct ethtool_drvinfo *di)
{
	strscpy(di->driver, ndev->dev.parent ? dev_driver_string(ndev->dev.parent)
					     : "luna-eth", sizeof(di->driver));
	strscpy(di->bus_info, ndev->dev.parent ? dev_name(ndev->dev.parent)
					       : "soc", sizeof(di->bus_info));
}

static const struct ethtool_ops luna_mib_ethtool_ops = {
	.get_drvinfo		= luna_mib_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_sset_count		= luna_mib_get_sset_count,
	.get_strings		= luna_mib_get_strings,
	.get_ethtool_stats	= luna_mib_get_stats,
};

/* ★ THE NAME-LENGTH CHECK IS HERE AND NOT IN A COMMENT, ...
 * dev/MEASURED-luna_eth_mib.c.md sec 8. */
static void luna_mib_check_names(struct device *dev, unsigned int ports)
{
	char buf[ETH_GSTRING_LEN * 2];
	unsigned int i;

	if (!ports)
		return;
	for (i = 0; i < ARRAY_SIZE(luna_mib_rx) + ARRAY_SIZE(luna_mib_tx); i++) {
		const char *n = i < ARRAY_SIZE(luna_mib_rx) ?
			luna_mib_rx[i].name :
			luna_mib_tx[i - ARRAY_SIZE(luna_mib_rx)].name;

		if (scnprintf(buf, sizeof(buf), "p%u_%s", ports - 1, n) >=
		    ETH_GSTRING_LEN - 1)
			dev_err(dev, "ethtool stat name truncated: %s\n", buf);
	}
}

void luna_mib_attach(struct net_device *ndev, void __iomem *sw,
		     void __iomem *mac, const struct luna_sw_map *map)
{
	struct luna_mib_state *s = &luna_mib;

	if (WARN_ON(!ndev || !map))
		return;
	if (WARN_ON(s->map && (s->map != map || s->sw != sw)))
		return;		/* a second, different switch core: refuse */

	s->sw = sw;
	s->mac = mac;
	s->map = map;
	ndev->ethtool_ops = &luna_mib_ethtool_ops;
	luna_mib_check_names(ndev->dev.parent, luna_mib_ports(s));
	netdev_info(ndev, "ethtool -S: %d counter(s)%s%s\n", luna_mib_count(s),
		    luna_mib_has_rx(s) ? "" : " (no rx: this die's RX MIB base is not established)",
		    luna_mib_has_mac(s) ? "" : " (no mac_*: this die's MAC MIB array is not established)");
}
EXPORT_SYMBOL_GPL(luna_mib_attach);
