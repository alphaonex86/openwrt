// SPDX-License-Identifier: GPL-2.0-only
/* TIER: FAMILY */
/* Luna switch ethtool: per-port statistics and link modes; no polling in the
 * datapath. */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/linkmode.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/string.h>

#include "luna_eth_regs.h"
#include "luna_eth_mib.h"
#include "luna_gpon_nic.h"
#include "eth_pause.h"
#include "eth_chip_patch.h"

/* One countable quantity: its published name and where it sits inside the
 * per-port window.  `wide` = a 64-bit counter stored as low word at `off` and
 * high word at `off + 4`. */
struct luna_mib_field {
	const char	*name;
	u16		off;
	bool		wide;
};

#define LUNA_MIB_RX_PAUSE	0x10	/* a paused port is not a stalled one, below */
/* ── STAT_PORT_RX_MIB, one 0x80 window per port ... -- dev/MEASURED-luna_eth_mib.c.md sec 2. */
static const struct luna_mib_field luna_mib_rx[] = {
	{ "rx_octets",		0x00, true },	/* ifInOctets, 64-bit pair	*/
	{ "rx_crc_align_err",	0x08 },		/* etherStatsCRCAlignErrors	*/
	{ "rx_symbol_err",	0x0c },		/* dot3StatsSymbolErrors	*/
	{ "rx_pause",		LUNA_MIB_RX_PAUSE },		/* dot3InPauseFrames		*/
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
#define LUNA_MIB_TX_OCTETS	0x28	/* its low word is the fibre port's progress witness, below */
#define LUNA_MIB_TX_PAUSE	0x48	/* the switch pausing the NIC: its ring holds by right, below */
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
	{ "tx_octets",		LUNA_MIB_TX_OCTETS, true },	/* ifOutOctets, 64-bit pair */
	{ "tx_single_collision", 0x30 },	/* dot3StatsSingleCollisionFrames */
	{ "tx_multi_collision",	0x34 },		/* dot3StatsMultipleCollisionFrames */
	{ "tx_deferred",	0x38 },		/* dot3StatsDeferredTransmissions */
	{ "tx_late_collision",	0x3c },		/* dot3StatsLateCollisions	*/
	{ "tx_collisions",	0x40 },		/* etherStatsCollisions		*/
	{ "tx_excess_collision", 0x44 },	/* dot3StatsExcessiveCollisions	*/
	{ "tx_pause",		LUNA_MIB_TX_PAUSE },	/* dot3OutPauseFrames		*/
	{ "tx_discards",	0x4c },		/* ifOutDiscards		*/
	{ "tx_1519_to_max_frames", 0x50 },
	{ "tx_bridge_discards",	0x58 },		/* dot1dTpPortInDiscards	*/
	{ "tx_unicast",		0x5c },		/* ifOutUcastPkts		*/
	{ "tx_multicast",	0x60 },		/* ifOutMulticastPkts		*/
	{ "tx_broadcast",	0x64 },		/* ifOutBroadcastPkts		*/
};

#define LUNA_LINK_PORTS	8

/* One port's mode, re-applied after every PHY bring-up: the vendor port
 * init's until ethtool sets another. */
struct luna_port_mode {
	u8	autoneg;
	u8	duplex;
	bool	tx_pause, rx_pause;	/* ethtool -a/-A: eth_pause.h */
	u32	speed;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(advertising);
};

/* MAC counters are only 16-bit; no totals without a lossless sampling clock. */
struct luna_mib_state {
	struct net_device	*ndev;
	const struct ethtool_ops *previous_ops;
	void __iomem		*sw;
	const struct luna_sw_map *map;
	const struct luna_phy_ops *phy;
	unsigned int		port;	/* the copper port ethtool shows and sets */
	struct luna_port_mode	mode[LUNA_LINK_PORTS];
	const struct luna_ring_ops *ring;
};
static struct luna_mib_state luna_mib;

struct eth_rx_len luna_rx_len;
EXPORT_SYMBOL_GPL(luna_rx_len);
static const char *const luna_ring_names[LUNA_RING_STATS] = {
	ETH_RX_WINDOW_STAT_NAMES
};
static const char *const luna_rx_len_names[] = {
	"rx_len_long_lan", "rx_len_short_lan", "rx_len_long_wan", "rx_len_short_wan",
};

struct eth_ring_watch luna_ring_watch = ETH_RING_WATCH_INIT(luna_ring_watch);
EXPORT_SYMBOL_GPL(luna_ring_watch);

/* The register dump of this chip (its map's ranges): ethtool -d, and the kernel log at the first stalled
 * port of a boot, before anything recovers it. Once: a second stall prints its state line only. */
static u32 luna_sw_rd(void *ctx, u32 at)
{
	return ioread32(luna_mib.sw + at);
}

static int luna_regs_len(struct net_device *ndev)
{
	return eth_regs_words(luna_mib.map->dump, ARRAY_SIZE(luna_mib.map->dump)) * 2 * sizeof(u32);
}

static void luna_regs_get(struct net_device *ndev, struct ethtool_regs *regs, void *data)
{
	regs->version = 1;
	eth_regs_pairs(luna_mib.map->dump, ARRAY_SIZE(luna_mib.map->dump), luna_sw_rd, NULL, data);
}

static void luna_mib_stall_dump(struct net_device *ndev)
{
	static bool printed;

	if (printed)
		return;
	printed = true;
	eth_regs_log(ndev, "stall dump", luna_mib.map->dump, ARRAY_SIZE(luna_mib.map->dump), luna_sw_rd, NULL);
	if (IS_ENABLED(CONFIG_LUNA_GPON))
		luna_gpon_stall_dump(ndev);
}

/* What a stall looked like, before it is recovered: the link above the port, the chip's own crash word, and per port
 * the egress pages it holds (now in the low half, the most ever in the high half) with the pause frames it received. */
static void luna_mib_stall_state(struct net_device *ndev)
{
	struct luna_mib_state *s = &luna_mib;
	char ports[LUNA_LINK_PORTS * 20 + 1] = "", crash[12] = "not asked";
	unsigned int port, n = 0;

	for (port = 0; s->map->pe_used && port < s->map->n_mib_ports && port < LUNA_LINK_PORTS; port++)
		n += scnprintf(ports + n, sizeof(ports) - n, " %08x/%u", ioread32(s->sw + s->map->pe_used + port * 4),
			       ioread32(s->sw + s->map->rx_mib + port * SW_STAT_PORT_MIB_STRIDE + LUNA_MIB_RX_PAUSE));
	if (s->map->crash_intr)
		scnprintf(crash, sizeof(crash), "%08x", ioread32(s->sw + s->map->crash_intr));
	netdev_err(ndev, "stall state: carrier %d, up %d time(s), crash word %s, egress pages/pause frames in per port%s\n",
		   netif_carrier_ok(ndev), atomic_read(&ndev->carrier_up_count), crash, ports);
	luna_mib_stall_dump(ndev);
}

static u32 luna_core_resets;

/* A stalled CPU or copper egress is recovered by the switch core reset: field X111W 11-15, three CPU-port stalls and
 * a copper hang that a link bounce, an eth0 down/up and the stock scheduler mode left held. It is no fibre recovery:
 * lab X111W 11-19, the fibre stall it followed left the LAN dead. The driver runs it from process context, followed
 * by its own init (eth_core_recover.h). Nothing on this CPU touches the core until the vendor's 10 ms settle has
 * passed: a read of a block in reset never completes. Only the RTL9602C (one CPU) has sw_rst. */
#define LUNA_CORE_RST_SETTLE_MS	10

void luna_mib_core_reset(struct net_device *ndev)
{
	struct luna_mib_state *s = &luna_mib;
	unsigned long flags;

	if (!s->sw || !s->map->sw_rst)
		return;
	local_irq_save(flags);
	iowrite32(ioread32(s->sw + s->map->sw_rst) | SW_RST_CORE, s->sw + s->map->sw_rst);
	mdelay(LUNA_CORE_RST_SETTLE_MS);
	local_irq_restore(flags);
	luna_core_resets++;
}
EXPORT_SYMBOL_GPL(luna_mib_core_reset);

/* The fibre port as an egress: what the driver handed it against the octets the switch counted out of it.
 * -> true when the caller recovers its own side (the NIC); the switch core is left alone. */
static struct eth_egress_watch luna_pon_egress;

bool luna_mib_pon_egress_sample(struct net_device *wan, u32 offered)
{
	struct luna_mib_state *s = &luna_mib;
	enum eth_watch v;
	u32 sent;

	if (IS_ENABLED(CONFIG_LUNA_GPON))
		luna_gpon_us_sample(wan);	/* the next stage: what the PON-IP took, sent upstream */
	if (!s->sw)
		return false;
	sent = ioread32(s->sw + SW_STAT_PORT_TX_MIB + s->map->pon_port * SW_STAT_PORT_MIB_STRIDE + LUNA_MIB_TX_OCTETS);
	v = eth_egress_watch_sample(&luna_pon_egress, offered, sent, jiffies64_to_msecs(get_jiffies_64()));
	if (v != ETH_WATCH_OK)
		luna_mib_stall_state(wan);
	eth_egress_watch_act(wan, &luna_pon_egress, v, "fibre");
	return v == ETH_WATCH_RECOVER;
}
EXPORT_SYMBOL_GPL(luna_mib_pon_egress_sample);

/* The NIC's own transmit ring into the CPU port, in frames; the switch pausing the NIC holds it by right. */
static struct eth_port_egress luna_nic_tx;

bool luna_mib_nic_tx_sample(struct net_device *ndev, bool ready, u32 queued, u32 sent)
{
	struct luna_mib_state *s = &luna_mib;
	u32 paused = s->sw ? ioread32(s->sw + SW_STAT_PORT_TX_MIB + s->map->cpu_port * SW_STAT_PORT_MIB_STRIDE +
				      LUNA_MIB_TX_PAUSE) : 0;

	return eth_ring_egress_judge(ndev, &luna_nic_tx, ready, paused, queued, sent);
}
EXPORT_SYMBOL_GPL(luna_mib_nic_tx_sample);

/* The CPU port and each copper port as an egress: the pages its queues hold against the octets it counted out.
 * A port whose link is down or whose partner pauses it is not judged, nor the CPU port while its ring waits on us.
 * The fibre port is judged above: its queue also waits on the grants of the far end. */
static struct eth_port_egress luna_port_egress[LUNA_LINK_PORTS];

bool luna_mib_port_egress_sample(struct net_device *ndev, bool cpu_ready)
{
	struct luna_mib_state *s = &luna_mib;
	unsigned int port;
	bool reset = false;

	if (!s->sw || !s->map->pe_used || !s->map->sw_rst || !s->map->rx_mib)
		return false;
	for (port = 0; port < s->map->n_mib_ports && port < LUNA_LINK_PORTS; port++) {
		u32 at = port * SW_STAT_PORT_MIB_STRIDE;
		bool cpu = port == s->map->cpu_port;
		bool ready = cpu ? cpu_ready : port < s->map->n_copper &&
			(ioread32(s->sw + s->map->p_ablty + port * 4) & ABLTY_LINK);
		u32 held = ioread32(s->sw + s->map->pe_used + port * 4) & PE_USED_NOW;

		/* a page carries a minimal frame at least */
		if (!eth_port_egress_judge(ndev, &luna_port_egress[port], cpu ? "CPU" : "copper", ready,
					   ioread32(s->sw + s->map->rx_mib + at + LUNA_MIB_RX_PAUSE), held,
					   held * ETH_ZLEN,
					   ioread32(s->sw + SW_STAT_PORT_TX_MIB + at + LUNA_MIB_TX_OCTETS)))
			continue;
		luna_mib_stall_state(ndev);
		reset = true;
	}
	return reset;
}
EXPORT_SYMBOL_GPL(luna_mib_port_egress_sample);

static bool luna_mib_has_rx(const struct luna_mib_state *s)
{
	return s->map && s->map->rx_mib && s->map->n_mib_ports;
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

/* The two role rows first: a reader tells the PON and CPU ports from the LAN ones without a port map;
 * then the received-frame length witnesses and the ring watch. */
#define LUNA_MIB_ROLES	(2 + ARRAY_SIZE(luna_rx_len_names) + ETH_RING_WATCH_STATS + 5)

static int luna_mib_count(const struct luna_mib_state *s)
{
	unsigned int n = luna_mib_ports(s);

	return n ? LUNA_MIB_ROLES + n * luna_mib_per_port(s) + (s->ring ? LUNA_RING_STATS : 0) : 0;
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

	if (sset != ETH_SS_STATS || !n)
		return;

	ethtool_puts(&data, "pon_port");
	ethtool_puts(&data, "cpu_port");
	for (i = 0; i < ARRAY_SIZE(luna_rx_len_names); i++)
		ethtool_puts(&data, luna_rx_len_names[i]);
	eth_ring_watch_strings(&data);
	ethtool_puts(&data, "pon_egress_stalls");
	ethtool_puts(&data, "nic_tx_stalls");
	ethtool_puts(&data, "pon_us_stalls");
	ethtool_puts(&data, "pon_us_grantless");
	ethtool_puts(&data, "switch_core_resets");
	for (port = 0; port < n; port++) {
		if (luna_mib_has_rx(s))
			for (i = 0; i < ARRAY_SIZE(luna_mib_rx); i++)
				ethtool_sprintf(&data, "p%u_%s", port,
						luna_mib_rx[i].name);
		for (i = 0; i < ARRAY_SIZE(luna_mib_tx); i++)
			ethtool_sprintf(&data, "p%u_%s", port,
					luna_mib_tx[i].name);
	}
	if (s->ring)
		for (i = 0; i < LUNA_RING_STATS; i++)
			ethtool_puts(&data, luna_ring_names[i]);
}

static void luna_mib_get_stats(struct net_device *ndev,
			       struct ethtool_stats *stats, u64 *data)
{
	struct luna_mib_state *s = &luna_mib;
	unsigned int port, i, n = luna_mib_ports(s);

	if (!n)
		return;
	BUILD_BUG_ON(ARRAY_SIZE(luna_rx_len_names) != sizeof(luna_rx_len) / sizeof(u64));
	*data++ = s->map->pon_port;
	*data++ = s->map->cpu_port;
	*data++ = luna_rx_len.long_lan;
	*data++ = luna_rx_len.short_lan;
	*data++ = luna_rx_len.long_wan;
	*data++ = luna_rx_len.short_wan;
	data = eth_ring_watch_stats(&luna_ring_watch, data);
	*data++ = READ_ONCE(luna_pon_egress.stalls);
	*data++ = READ_ONCE(luna_nic_tx.watch.stalls);
	*data++ = IS_ENABLED(CONFIG_LUNA_GPON) ? luna_gpon_us_stalls() : 0;
	*data++ = IS_ENABLED(CONFIG_LUNA_GPON) ? luna_gpon_us_grantless() : 0;
	*data++ = READ_ONCE(luna_core_resets);
	for (port = 0; port < n; port++) {
		if (luna_mib_has_rx(s))
			for (i = 0; i < ARRAY_SIZE(luna_mib_rx); i++)
				*data++ = luna_mib_read(s->sw, s->map->rx_mib,
							port, &luna_mib_rx[i]);
		for (i = 0; i < ARRAY_SIZE(luna_mib_tx); i++)
			*data++ = luna_mib_read(s->sw, SW_STAT_PORT_TX_MIB,
						port, &luna_mib_tx[i]);
	}
	if (s->ring)
		s->ring->stats(ndev, data);
}

static void luna_mib_get_drvinfo(struct net_device *ndev,
				 struct ethtool_drvinfo *di)
{
	strscpy(di->driver, ndev->dev.parent ? dev_driver_string(ndev->dev.parent)
					     : "luna-eth", sizeof(di->driver));
	strscpy(di->bus_info, ndev->dev.parent ? dev_name(ndev->dev.parent)
					       : "soc", sizeof(di->bus_info));
}

/* ── Link modes of the copper ports ───────────────────────────────────────
 * Two places decide a copper port's speed and duplex:
 *   MAC  FORCE_P_ABLTY holds forced values, ABLTY_FORCE_MODE says which fields
 *        are forced; a field not forced follows the PHY through the switch's
 *        own PHY poll. P_ABLTY is what the MAC actually runs.
 *   PHY  BMCR + ADVERTISE + CTRL1000: auto-negotiation, or forced 10/100.
 * A copper MAC forces only LINK (the UNI administrative lock), as the vendor
 * port init leaves it: every mode is chosen in the PHY and the MAC follows.
 * The PHY starts as the vendor port init sets it: auto-negotiation, every
 * speed it has, symmetric and asymmetric pause.
 * A forced MAC speed or duplex under an autonegotiating PHY is a mismatch
 * (late collisions, size-dependent loss), so it is reported, never silent.
 *
 * ethtool reaches one port at a time through phyad:
 *   ethtool -s eth0 phyad 1                  select port 1, apply nothing
 *   ethtool eth0                             the selected port: PHY + MAC view
 *   ethtool -s eth0 phyad 0 speed 100 duplex full autoneg off
 *   ethtool -s eth0 autoneg on advertise 0x2f
 *   ethtool -r eth0                          restart auto-negotiation
 * A mode set here survives the driver's PHY bring-up (luna_mib_phy_reapply). */

static int luna_link_rd(struct luna_mib_state *s, unsigned int port,
			unsigned int reg, u16 *val)
{
	return s->phy->read(s->ndev, port, reg, val);
}

static int luna_link_wr(struct luna_mib_state *s, unsigned int port,
			unsigned int reg, u16 val)
{
	return s->phy->write(s->ndev, port, reg, val);
}

static void luna_link_supported(unsigned long *sup, u16 bmsr, u16 estat)
{
	linkmode_zero(sup);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, sup);
	linkmode_set_bit(ETHTOOL_LINK_MODE_TP_BIT, sup);
	linkmode_set_bit(ETHTOOL_LINK_MODE_MII_BIT, sup);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, sup);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, sup);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, sup, bmsr & BMSR_10HALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, sup, bmsr & BMSR_10FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, sup, bmsr & BMSR_100HALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, sup, bmsr & BMSR_100FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT, sup, estat & ESTATUS_1000_THALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, sup, estat & ESTATUS_1000_TFULL);
}

/* The MAC half of ethtool -A: the PHY advertises, the MAC's own FC fields decide what the port sends. */
static void luna_pause_apply(struct luna_mib_state *s, unsigned int port, const struct luna_port_mode *m)
{
	void __iomem *value = s->sw + s->map->force_ablty + port * 4;
	void __iomem *mode = s->sw + s->map->ablty_force + port * 4;

	iowrite32(eth_pause_force_value(ioread32(value), ABLTY_TX_FC, ABLTY_RX_FC), value);
	iowrite32(eth_pause_force_mode(ioread32(mode), ABLTY_TX_FC, ABLTY_RX_FC, m->tx_pause, m->rx_pause), mode);
}

static void luna_link_check_mac(struct luna_mib_state *s, unsigned int port)
{
	u32 forced = ioread32(s->sw + s->map->ablty_force + port * 4);
	u32 live = ioread32(s->sw + s->map->p_ablty + port * 4);

	if (eth_pause_tx_leaks(live & ABLTY_LINK, live & ABLTY_TX_FC, s->mode[port].tx_pause))
		netdev_err(s->ndev, "port %u: the MAC sends pause although it is set off (P_ABLTY %08x, ABLTY_FORCE_MODE %08x)\n",
			   port, live, forced);

	if (forced & (ABLTY_SPEED | ABLTY_DUPLEX))
		netdev_err(s->ndev,
			   "port %u: the MAC forces speed/duplex (ABLTY_FORCE_MODE %08x, FORCE_P_ABLTY %08x) under its PHY -- a duplex mismatch\n",
			   port, forced,
			   ioread32(s->sw + s->map->force_ablty + port * 4));
}

static int luna_link_get(struct net_device *ndev,
			 struct ethtool_link_ksettings *ks)
{
	struct luna_mib_state *s = &luna_mib;
	unsigned int port = s->port;
	u16 bmcr, bmsr, adv, lpa, estat = 0, ctrl1000 = 0, stat1000 = 0;
	u32 mac;
	int rc;

	if (!s->phy)
		return -EOPNOTSUPP;
	mutex_lock(s->phy->lock);
	rc = luna_link_rd(s, port, MII_BMCR, &bmcr);
	if (!rc)
		rc = luna_link_rd(s, port, MII_BMSR, &bmsr);
	if (!rc)
		rc = luna_link_rd(s, port, MII_ADVERTISE, &adv);
	if (!rc)
		rc = luna_link_rd(s, port, MII_LPA, &lpa);
	if (!rc && (bmsr & BMSR_ESTATEN))
		rc = luna_link_rd(s, port, MII_ESTATUS, &estat);
	if (!rc && (bmsr & BMSR_ESTATEN))
		rc = luna_link_rd(s, port, MII_CTRL1000, &ctrl1000);
	if (!rc && (bmsr & BMSR_ESTATEN))
		rc = luna_link_rd(s, port, MII_STAT1000, &stat1000);
	mutex_unlock(s->phy->lock);
	if (rc)
		return rc;

	luna_link_supported(ks->link_modes.supported, bmsr, estat);
	mii_adv_mod_linkmode_adv_t(ks->link_modes.advertising, adv);
	mii_ctrl1000_mod_linkmode_adv_t(ks->link_modes.advertising, ctrl1000);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, ks->link_modes.advertising,
			 bmcr & BMCR_ANENABLE);
	if (bmsr & BMSR_ANEGCOMPLETE) {
		mii_lpa_mod_linkmode_lpa_t(ks->link_modes.lp_advertising, lpa);
		mii_stat1000_mod_linkmode_lpa_t(ks->link_modes.lp_advertising, stat1000);
	}

	/* speed and duplex are the MAC's, so a mismatch with the PHY shows */
	mac = ioread32(s->sw + s->map->p_ablty + port * 4);
	ks->base.speed = SPEED_UNKNOWN;
	ks->base.duplex = DUPLEX_UNKNOWN;
	if (mac & ABLTY_LINK) {
		static const u32 speeds[] = { SPEED_10, SPEED_100, SPEED_1000, SPEED_UNKNOWN };

		ks->base.speed = speeds[FIELD_GET(ABLTY_SPEED, mac)];
		ks->base.duplex = (mac & ABLTY_DUPLEX) ? DUPLEX_FULL : DUPLEX_HALF;
	}
	ks->base.autoneg = (bmcr & BMCR_ANENABLE) ? AUTONEG_ENABLE : AUTONEG_DISABLE;
	ks->base.port = PORT_TP;
	ks->base.phy_address = port;
	luna_link_check_mac(s, port);
	return 0;
}

static int luna_link_apply(struct luna_mib_state *s, unsigned int port,
			   const struct luna_port_mode *m)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(sup);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(adv);
	u16 bmcr, bmsr, estat = 0, reg;
	int rc;

	mutex_lock(s->phy->lock);
	rc = luna_link_rd(s, port, MII_BMCR, &bmcr);
	if (!rc)
		rc = luna_link_rd(s, port, MII_BMSR, &bmsr);
	if (!rc && (bmsr & BMSR_ESTATEN))
		rc = luna_link_rd(s, port, MII_ESTATUS, &estat);
	if (rc)
		goto out;
	luna_link_supported(sup, bmsr, estat);
	bmcr &= BMCR_PDOWN;	/* the administrative lock stays as it is */

	if (m->autoneg == AUTONEG_ENABLE) {
		linkmode_and(adv, m->advertising, sup);
		if (!(linkmode_adv_to_mii_adv_t(adv) & ADVERTISE_ALL) &&
		    !linkmode_adv_to_mii_ctrl1000_t(adv)) {
			rc = -EINVAL;
			goto out;
		}
		rc = luna_link_rd(s, port, MII_ADVERTISE, &reg);
		if (!rc)
			rc = luna_link_wr(s, port, MII_ADVERTISE,
					  (reg & ~(ADVERTISE_ALL | ADVERTISE_PAUSE_CAP |
						   ADVERTISE_PAUSE_ASYM)) |
					  linkmode_adv_to_mii_adv_t(adv));
		if (!rc && (bmsr & BMSR_ESTATEN))
			rc = luna_link_rd(s, port, MII_CTRL1000, &reg);
		if (!rc && (bmsr & BMSR_ESTATEN))
			rc = luna_link_wr(s, port, MII_CTRL1000,
					  (reg & ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF)) |
					  linkmode_adv_to_mii_ctrl1000_t(adv));
		bmcr |= BMCR_ANENABLE;
		if (!(bmcr & BMCR_PDOWN))
			bmcr |= BMCR_ANRESTART;
	} else {
		/* 1000BASE-T cannot run without auto-negotiation */
		if ((m->speed != SPEED_10 && m->speed != SPEED_100) ||
		    (m->duplex != DUPLEX_HALF && m->duplex != DUPLEX_FULL)) {
			rc = -EINVAL;
			goto out;
		}
		bmcr |= mii_bmcr_encode_fixed(m->speed, m->duplex);
	}
	if (!rc)
		rc = luna_link_wr(s, port, MII_BMCR, bmcr);
out:
	mutex_unlock(s->phy->lock);
	if (rc)
		netdev_err(s->ndev, "port %u: link mode not applied (%d)\n", port, rc);
	else
		netdev_info(s->ndev, "port %u: %s\n", port,
			    m->autoneg == AUTONEG_ENABLE ? "auto-negotiation" :
			    m->duplex == DUPLEX_FULL ? "forced full duplex" :
			    "forced half duplex");
	return rc;
}

static bool luna_link_same(const struct ethtool_link_ksettings *a,
			   const struct ethtool_link_ksettings *b)
{
	return a->base.autoneg == b->base.autoneg &&
	       a->base.speed == b->base.speed &&
	       a->base.duplex == b->base.duplex &&
	       linkmode_equal(a->link_modes.advertising, b->link_modes.advertising);
}

static int luna_link_set(struct net_device *ndev,
			 const struct ethtool_link_ksettings *ks)
{
	struct luna_mib_state *s = &luna_mib;
	unsigned int port = ks->base.phy_address;
	struct ethtool_link_ksettings cur = {};
	struct luna_port_mode m = {};
	int rc;

	if (!s->phy)
		return -EOPNOTSUPP;
	if (port >= s->map->n_copper)
		return -EINVAL;
	rc = luna_link_get(ndev, &cur);
	if (rc)
		return rc;
	if (port != s->port && luna_link_same(&cur, ks)) {
		s->port = port;		/* phyad alone selects */
		return 0;
	}
	s->port = port;
	m.autoneg = ks->base.autoneg;
	m.speed = ks->base.speed;
	m.duplex = ks->base.duplex;
	m.tx_pause = s->mode[port].tx_pause;
	m.rx_pause = s->mode[port].rx_pause;
	linkmode_copy(m.advertising, ks->link_modes.advertising);
	rc = luna_link_apply(s, port, &m);
	if (!rc)
		s->mode[port] = m;
	return rc;
}

static int luna_link_nway_reset(struct net_device *ndev)
{
	struct luna_mib_state *s = &luna_mib;
	u16 bmcr;
	int rc;

	if (!s->phy)
		return -EOPNOTSUPP;
	mutex_lock(s->phy->lock);
	rc = luna_link_rd(s, s->port, MII_BMCR, &bmcr);
	if (!rc && !(bmcr & BMCR_ANENABLE))
		rc = -EINVAL;
	if (!rc)
		rc = luna_link_wr(s, s->port, MII_BMCR, bmcr | BMCR_ANRESTART);
	mutex_unlock(s->phy->lock);
	return rc;
}

static void luna_pause_get(struct net_device *ndev, struct ethtool_pauseparam *p)
{
	const struct luna_port_mode *m = &luna_mib.mode[luna_mib.port];

	p->autoneg = m->autoneg;
	p->tx_pause = m->tx_pause;
	p->rx_pause = m->rx_pause;
}

static int luna_pause_set(struct net_device *ndev, struct ethtool_pauseparam *p)
{
	struct luna_mib_state *s = &luna_mib;
	struct luna_port_mode m;
	int rc;

	if (!s->phy)
		return -EOPNOTSUPP;
	m = s->mode[s->port];
	if (p->autoneg != m.autoneg)
		return -EINVAL;		/* pause follows the port's own auto-negotiation */
	m.tx_pause = p->tx_pause;
	m.rx_pause = p->rx_pause;
	eth_pause_advertise(m.advertising, m.tx_pause, m.rx_pause);
	rc = luna_link_apply(s, s->port, &m);
	if (rc)
		return rc;
	luna_pause_apply(s, s->port, &m);
	s->mode[s->port] = m;
	return 0;
}

static void luna_link_default(struct luna_port_mode *m)
{
	static const int bits[] = {
		ETHTOOL_LINK_MODE_10baseT_Half_BIT, ETHTOOL_LINK_MODE_10baseT_Full_BIT,
		ETHTOOL_LINK_MODE_100baseT_Half_BIT, ETHTOOL_LINK_MODE_100baseT_Full_BIT,
		ETHTOOL_LINK_MODE_1000baseT_Half_BIT, ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
		ETHTOOL_LINK_MODE_Pause_BIT, ETHTOOL_LINK_MODE_Asym_Pause_BIT,
		ETHTOOL_LINK_MODE_Autoneg_BIT,
	};

	memset(m, 0, sizeof(*m));
	m->autoneg = AUTONEG_ENABLE;
	m->tx_pause = ETH_PAUSE_TX_DEFAULT;
	m->rx_pause = ETH_PAUSE_RX_DEFAULT;
	linkmode_set_bit_array(bits, ARRAY_SIZE(bits), m->advertising);
	eth_pause_advertise(m->advertising, m->tx_pause, m->rx_pause);
}

/* Called by a driver after its own PHY bring-up: every port gets its mode
 * back, and the MAC is audited. */
void luna_mib_phy_reapply(struct net_device *ndev)
{
	struct luna_mib_state *s = &luna_mib;
	unsigned int port;

	if (!s->phy || s->ndev != ndev)
		return;
	for (port = 0; port < s->map->n_copper; port++) {
		luna_link_apply(s, port, &s->mode[port]);
		luna_pause_apply(s, port, &s->mode[port]);
		luna_link_check_mac(s, port);
	}
}
EXPORT_SYMBOL_GPL(luna_mib_phy_reapply);

/* One 16-bit entry of the chip's efuse. -> 0, -EOPNOTSUPP on a chip whose efuse registers are not established. */
int luna_efuse_read(void __iomem *sw, const struct luna_sw_map *map, unsigned int entry, u16 *out)
{
	static DEFINE_MUTEX(lock);
	int rc = -ETIMEDOUT;
	unsigned int i;
	u32 v;

	if (!map->efuse_cmd)
		return -EOPNOTSUPP;
	if (entry >= LUNA_EFUSE_ENTRIES)
		return -EINVAL;
	mutex_lock(&lock);
	iowrite32(LUNA_EFUSE_BUSY | entry, sw + map->efuse_cmd);
	for (i = 0; rc && i < 1000; i++) {
		v = ioread32(sw + map->efuse_cmd + 4);
		if (v & LUNA_EFUSE_BUSY) {
			udelay(1);
			continue;
		}
		*out = v & 0xffff;
		rc = 0;
	}
	mutex_unlock(&lock);
	return rc;
}
EXPORT_SYMBOL_GPL(luna_efuse_read);

static int luna_efuse_len(struct net_device *ndev)
{
	return luna_mib.map->efuse_cmd ? LUNA_EFUSE_ENTRIES * 2 : 0;
}

static int luna_efuse_entry(void *ctx, unsigned int entry, u16 *out)
{
	return luna_efuse_read(luna_mib.sw, luna_mib.map, entry, out);
}

/* The chip efuse as ethtool's EEPROM: ethtool -e. */
static int luna_efuse_get(struct net_device *ndev, struct ethtool_eeprom *ee, u8 *data)
{
	static const struct eth_chip_patch_io io = { .efuse = luna_efuse_entry };

	return eth_efuse_bytes(&io, NULL, ee->offset, ee->len, data);
}

static const struct ethtool_ops luna_mib_ethtool_ops = {
	.get_drvinfo		= luna_mib_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_sset_count		= luna_mib_get_sset_count,
	.get_strings		= luna_mib_get_strings,
	.get_ethtool_stats	= luna_mib_get_stats,
	.get_link_ksettings	= luna_link_get,
	.set_link_ksettings	= luna_link_set,
	.nway_reset		= luna_link_nway_reset,
	.get_pauseparam		= luna_pause_get,
	.set_pauseparam		= luna_pause_set,
	.get_eeprom_len		= luna_efuse_len,
	.get_eeprom		= luna_efuse_get,
	.get_regs_len		= luna_regs_len,
	.get_regs		= luna_regs_get,
};

static void luna_ring_get(struct net_device *ndev, struct ethtool_ringparam *r,
			  struct kernel_ethtool_ringparam *kr, struct netlink_ext_ack *ack)
{
	luna_mib.ring->get(ndev, r);
}

static int luna_ring_set(struct net_device *ndev, struct ethtool_ringparam *r,
			 struct kernel_ethtool_ringparam *kr, struct netlink_ext_ack *ack)
{
	return luna_mib.ring->set(ndev, r);
}

/* The same table plus the ring sizes, for a driver that attached its ring (luna_mib_ring_attach). */
static const struct ethtool_ops luna_mib_ring_ethtool_ops = {
	.get_drvinfo		= luna_mib_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_sset_count		= luna_mib_get_sset_count,
	.get_strings		= luna_mib_get_strings,
	.get_ethtool_stats	= luna_mib_get_stats,
	.get_link_ksettings	= luna_link_get,
	.set_link_ksettings	= luna_link_set,
	.nway_reset		= luna_link_nway_reset,
	.get_pauseparam		= luna_pause_get,
	.set_pauseparam		= luna_pause_set,
	.get_eeprom_len		= luna_efuse_len,
	.get_eeprom		= luna_efuse_get,
	.get_regs_len		= luna_regs_len,
	.get_regs		= luna_regs_get,
	.get_ringparam		= luna_ring_get,
	.set_ringparam		= luna_ring_set,
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

static void luna_mib_detach(void *data)
{
	struct net_device *ndev = data;

	ndev->ethtool_ops = luna_mib.previous_ops;
	memset(&luna_mib, 0, sizeof(luna_mib));
}

int luna_mib_attach(struct net_device *ndev, void __iomem *sw,
		    const struct luna_sw_map *map, const struct luna_phy_ops *phy)
{
	struct luna_mib_state *s = &luna_mib;
	const struct ethtool_ops *previous_ops;
	unsigned int i;
	int ret;

	if (!ndev || !ndev->dev.parent || !sw || !map || !map->n_mib_ports ||
	    map->n_copper > LUNA_LINK_PORTS ||
	    (phy && (!phy->lock || !phy->read || !phy->write || !map->p_ablty || !map->force_ablty ||
		     !map->ablty_force)))
		return -EINVAL;
	if (s->ndev || ndev->ethtool_ops == &luna_mib_ethtool_ops)
		return -EBUSY;
	previous_ops = ndev->ethtool_ops;
	netdev_set_default_ethtool_ops(ndev, &luna_mib_ethtool_ops);
	if (ndev->ethtool_ops != &luna_mib_ethtool_ops)
		return -EBUSY;

	s->ndev = ndev;
	s->previous_ops = previous_ops;
	s->sw = sw;
	s->map = map;
	s->phy = phy;
	for (i = 0; i < map->n_copper; i++)
		luna_link_default(&s->mode[i]);
	/* Attach before netdev registration: devres detaches after ndo_stop. */
	ret = devm_add_action_or_reset(ndev->dev.parent, luna_mib_detach, ndev);
	if (ret)
		return ret;
	luna_mib_check_names(ndev->dev.parent, luna_mib_ports(s));
	netdev_info(ndev, "ethtool -S: %d switch counters%s\n", luna_mib_count(s),
		    luna_mib_has_rx(s) ? "" : " (RX base unavailable)");
	return 0;
}
EXPORT_SYMBOL_GPL(luna_mib_attach);

int luna_mib_ring_attach(struct net_device *ndev, const struct luna_ring_ops *ops)
{
	struct luna_mib_state *s = &luna_mib;

	if (!ops || !ops->get || !ops->set || !ops->stats || s->ndev != ndev ||
	    ndev->ethtool_ops != &luna_mib_ethtool_ops || ndev->reg_state != NETREG_UNINITIALIZED)
		return -EINVAL;
	s->ring = ops;
	ndev->ethtool_ops = &luna_mib_ring_ethtool_ops;
	return 0;
}
EXPORT_SYMBOL_GPL(luna_mib_ring_attach);
MODULE_LICENSE("GPL");
