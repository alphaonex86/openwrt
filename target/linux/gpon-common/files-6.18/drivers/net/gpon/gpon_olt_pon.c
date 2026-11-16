// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE, not in the STRICT subset */
/* gpon_olt_pon.c -- see gpon_olt_pon.h. */
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/eth_netdev_contract.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/workqueue.h>

#include "gpon_olt_mac.h"
#include "gpon_olt_pon.h"

#define PON_POLL_MS	1000
#define PON_DS_MBPS	2488		/* G.984.2 downstream line rate, 2488.32 Mbit/s */

struct pon_priv {
	struct net_device *ndev;
	struct gpon_olt_mac *mac;
	unsigned int link;
	const struct gpon_olt_pon_chip *pc;
	struct delayed_work poll;
};

static const struct {
	char name[ETH_GSTRING_LEN];
	size_t off;
} pon_stats[] = {
#define S(f) { #f, offsetof(struct gpon_olt_mac_link, f) }
	S(ds_ploam), S(ploam_busy), S(ploam_drop), S(us_ploam), S(ds_omci), S(omci_full), S(us_omci), S(grant_fail),
#undef S
};

static void pon_poll(struct work_struct *w)
{
	struct pon_priv *p = container_of(to_delayed_work(w), struct pon_priv, poll);
	struct net_device *ndev = p->ndev;
	bool up;

	if (!READ_ONCE(p->mac->running))
		return;		/* the chip is stopped: no read of it any more */
	up = p->pc->link_up(p->mac->chip, p->link);
	if (up != netif_carrier_ok(ndev)) {
		if (up)
			netif_carrier_on(ndev);
		else
			netif_carrier_off(ndev);
	}
	schedule_delayed_work(&p->poll, msecs_to_jiffies(PON_POLL_MS));
}

static int pon_open(struct net_device *ndev)
{
	struct pon_priv *p = netdev_priv(ndev);

	schedule_delayed_work(&p->poll, 0);
	return 0;
}

static int pon_stop(struct net_device *ndev)
{
	struct pon_priv *p = netdev_priv(ndev);

	cancel_delayed_work_sync(&p->poll);
	netif_carrier_off(ndev);
	return 0;
}

static netdev_tx_t pon_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	dev_core_stats_tx_dropped_inc(ndev);	/* no CPU data path yet: counted, never silent */
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops pon_netdev_ops = {
	.ndo_open = pon_open,
	.ndo_stop = pon_stop,
	.ndo_start_xmit = pon_xmit,
};

static int pon_get_link_ksettings(struct net_device *ndev, struct ethtool_link_ksettings *ks)
{
	ethtool_link_ksettings_zero_link_mode(ks, supported);
	ethtool_link_ksettings_zero_link_mode(ks, advertising);
	ks->base.speed = netif_carrier_ok(ndev) ? PON_DS_MBPS : SPEED_UNKNOWN;
	ks->base.duplex = netif_carrier_ok(ndev) ? DUPLEX_FULL : DUPLEX_UNKNOWN;
	ks->base.port = PORT_FIBRE;
	ks->base.autoneg = AUTONEG_DISABLE;
	return 0;
}

static int pon_get_sset_count(struct net_device *ndev, int sset)
{
	return sset == ETH_SS_STATS ? ARRAY_SIZE(pon_stats) : -EOPNOTSUPP;
}

static void pon_get_strings(struct net_device *ndev, u32 sset, u8 *data)
{
	unsigned int i;

	if (sset != ETH_SS_STATS)
		return;
	for (i = 0; i < ARRAY_SIZE(pon_stats); i++)
		ethtool_puts(&data, pon_stats[i].name);
}

static void pon_get_ethtool_stats(struct net_device *ndev, struct ethtool_stats *st, u64 *data)
{
	struct pon_priv *p = netdev_priv(ndev);
	const u8 *l = (const u8 *)&p->mac->links[p->link];
	unsigned int i;

	spin_lock_bh(&p->mac->lock);
	for (i = 0; i < ARRAY_SIZE(pon_stats); i++)
		data[i] = *(const u64 *)(l + pon_stats[i].off);
	spin_unlock_bh(&p->mac->lock);
}

static int pon_get_regs_len(struct net_device *ndev)
{
	struct pon_priv *p = netdev_priv(ndev);

	return p->pc->n_regs * sizeof(u32);
}

static void pon_get_regs(struct net_device *ndev, struct ethtool_regs *regs, void *buf)
{
	struct pon_priv *p = netdev_priv(ndev);
	u32 *out = buf;
	unsigned int i;

	regs->version = 0;
	for (i = 0; i < p->pc->n_regs; i++)
		out[i] = p->pc->rd(p->mac->chip, p->pc->regs[i]);
}

static const struct ethtool_ops pon_ethtool_ops = {
	.get_regs_len = pon_get_regs_len,
	.get_regs = pon_get_regs,
	.get_link = ethtool_op_get_link,
	.get_link_ksettings = pon_get_link_ksettings,
	.get_sset_count = pon_get_sset_count,
	.get_strings = pon_get_strings,
	.get_ethtool_stats = pon_get_ethtool_stats,
};

int gpon_olt_pon_register(struct device *dev, struct gpon_olt_mac *mac, const struct gpon_olt_pon_chip *pc)
{
	struct net_device *ndev;
	struct pon_priv *p;
	unsigned int i;
	int ret;

	if (!pc || !pc->link_up || !pc->rd || !pc->regs || !pc->n_regs)
		return dev_err_probe(dev, -EINVAL, "%s: a PON netdev needs the chip's link state and registers\n",
				     mac->name);
	for (i = 0; i < mac->n_links; i++) {
		ndev = devm_alloc_etherdev(dev, sizeof(*p));	/* devm_register_netdev takes only these */
		if (!ndev)
			return -ENOMEM;
		strscpy(ndev->name, "pon%d", IFNAMSIZ);
		p = netdev_priv(ndev);
		p->ndev = ndev;
		p->mac = mac;
		p->link = i;
		p->pc = pc;
		INIT_DELAYED_WORK(&p->poll, pon_poll);
		SET_NETDEV_DEV(ndev, dev);
		ndev->netdev_ops = &pon_netdev_ops;
		ndev->ethtool_ops = &pon_ethtool_ops;
		eth_hw_addr_random(ndev);
		netif_carrier_off(ndev);
		ret = eth_devm_register_netdev(dev, ndev, NULL,
					       ETH_NO_ndo_tx_timeout | ETH_NO_set_link_ksettings | ETH_NO_get_pauseparam |
					       ETH_NO_set_pauseparam | ETH_NO_get_ringparam | ETH_NO_rx_watch);
		if (ret)
			return dev_err_probe(dev, ret, "%s: link %u: PON netdev not registered\n", mac->name, i);
	}
	return 0;
}
