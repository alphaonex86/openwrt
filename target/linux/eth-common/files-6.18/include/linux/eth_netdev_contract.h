/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_netdev_contract.h -- a netdev of ours registers only COMPLETE: each standard operation below is
 * implemented, or the driver declares it absent by name (ETH_NO_<op>). A missing operation and a stale
 * declaration both refuse the registration, so the list of absences can only be the truth.
 */
#ifndef _LINUX_ETH_NETDEV_CONTRACT_H
#define _LINUX_ETH_NETDEV_CONTRACT_H

#include <linux/ethtool.h>
#include <linux/netdevice.h>

struct eth_queue_watch;

#define ETH_CONTRACT_NDO(X)	X(ndo_tx_timeout)
#define ETH_CONTRACT_ETHTOOL(X)	\
	X(get_link_ksettings) X(set_link_ksettings) X(get_pauseparam) X(set_pauseparam) \
	X(get_ethtool_stats) X(get_ringparam) X(get_regs)

enum {
#define X(op) ETH_BIT_##op,
	ETH_CONTRACT_NDO(X) ETH_CONTRACT_ETHTOOL(X)
#undef X
	ETH_BIT_rx_watch,
};

enum {
#define X(op) ETH_NO_##op = 1u << ETH_BIT_##op,
	ETH_CONTRACT_NDO(X) ETH_CONTRACT_ETHTOOL(X)
#undef X
	ETH_NO_rx_watch = 1u << ETH_BIT_rx_watch,	/* no RX ring of its own to watch for a stall */
};

static inline int eth_contract_op(struct net_device *ndev, const char *op, bool present, bool declared_absent)
{
	if (present != declared_absent)
		return 0;
	netdev_err(ndev, "driver contract: %s is %s\n", op,
		   present ? "implemented and still declared absent" : "missing and not declared absent");
	return 1;
}

/* Every mismatch is logged before the refusal: one boot names them all. */
static inline int eth_netdev_contract(struct net_device *ndev, const struct eth_queue_watch *rx_watch, u32 absent)
{
	const struct net_device_ops *ndo = ndev->netdev_ops;
	const struct ethtool_ops *eto = ndev->ethtool_ops;
	int bad = 0;

#define X(op) bad += eth_contract_op(ndev, #op, ndo && ndo->op, absent & ETH_NO_##op);
	ETH_CONTRACT_NDO(X)
#undef X
#define X(op) bad += eth_contract_op(ndev, #op, eto && eto->op, absent & ETH_NO_##op);
	ETH_CONTRACT_ETHTOOL(X)
#undef X
	bad += eth_contract_op(ndev, "rx_watch", rx_watch, absent & ETH_NO_rx_watch);
	return bad ? -ENOSYS : 0;
}

static inline int eth_register_netdev(struct net_device *ndev, const struct eth_queue_watch *rx_watch, u32 absent)
{
	return eth_netdev_contract(ndev, rx_watch, absent) ?: register_netdev(ndev);
}

static inline int eth_devm_register_netdev(struct device *dev, struct net_device *ndev,
					   const struct eth_queue_watch *rx_watch, u32 absent)
{
	return eth_netdev_contract(ndev, rx_watch, absent) ?: devm_register_netdev(dev, ndev);
}

#endif
