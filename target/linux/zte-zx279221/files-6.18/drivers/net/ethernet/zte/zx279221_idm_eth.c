// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/*
 * ZTE ZX279221 management Ethernet: the CPU port of the SoC's switch (its "IDM"), behind which the board's MGMT
 * RJ45 sits. U-Boot configures the switch and its PHY at every boot, before any boot command; this driver leaves
 * them as they are and drives only the CPU port. Facts: the unit's U-Boot network driver and its stock kernel's,
 * read from their binaries.
 *
 * One table of 8-byte descriptors, the receive ring then the transmit ring, each slot with its own 2 KiB buffer
 * (word 0 its address, word 1 the length << 4). The port counts what it filled and what it sent; the driver
 * acknowledges counts. Receive copies the frame out and hands the slot back; transmit copies the frame in, as stock.
 */
#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/eth_netdev_contract.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include "eth_rx_geom.h"
#include "eth_tx_ring.h"

#define DRV			"zx279221-idm"

#define IDM_CFG			0x00
#define IDM_CFG_ON		GENMASK(19, 16)		/* set by every boot stage, meaning not read */
#define IDM_CFG_BUF		GENMASK(27, 20)		/* the receive buffer, in 64-byte units */
#define IDM_RUN			0x04
#define IDM_INT_MASK		0x0c			/* 1 = enabled */
#define IDM_INT_STATUS		0x10
#define IDM_INT_RX		BIT(0)
#define IDM_TX_BASE		0x40			/* low word, then high */
#define IDM_RX_BASE		0x48
#define IDM_RING_SIZE		0x50
#define IDM_COAL_0		0x54			/* stock's values, meaning not read */
#define IDM_COAL_1		0x5c
#define IDM_COAL_2		0x60
#define IDM_TX_KICK		0x64			/* bits 31-16: descriptors queued; 15-0: sent ones acknowledged */
#define IDM_TX_SENT		0x68
#define IDM_RX_ACK		0x6c			/* bits 15-0: frames taken; 31-16: slots handed back */
#define IDM_RX_READY		0x70
#define IDM_COUNT		GENMASK(15, 0)

#define SLOTS			512
#define DESC_SIZE		8
#define BUF_SIZE		2048
#define IDM_RXD_LEN_MASK	GENMASK(17, 4)
#define IDM_TXD_LEN_SHIFT	4

/* The length counts the frame as the switch hands it, no FCS: the stock kernel passes it up whole. */
static const struct eth_rx_layout rx_layout = { .fcs = ETH_RX_FCS_NOT_IN_LEN, .buf = BUF_SIZE };

struct idm {
	struct device *dev;
	struct net_device *ndev;
	void __iomem *base;
	struct napi_struct napi;
	spinlock_t tx_lock;
	__le32 *ring;		/* receive descriptors, then transmit */
	dma_addr_t ring_dma;
	u8 *bufs;		/* receive buffers, then transmit */
	dma_addr_t bufs_dma;
	unsigned int rx_next;
	struct eth_tx_ring tx;
	struct eth_rx_len rx_len;
	u64 rx_bad_len, rx_nomem;
};

static u32 rd(struct idm *p, u32 reg)
{
	return readl(p->base + reg);
}

static void wr(struct idm *p, u32 reg, u32 val)
{
	writel(val, p->base + reg);
}

static __le32 *desc(struct idm *p, bool tx, unsigned int slot)
{
	return p->ring + ((tx ? SLOTS : 0) + slot) * DESC_SIZE / 4;
}

static u8 *buf(struct idm *p, bool tx, unsigned int slot)
{
	return p->bufs + ((tx ? SLOTS : 0) + slot) * BUF_SIZE;
}

static void idm_stop_hw(struct idm *p)
{
	wr(p, IDM_RUN, 0);
	wr(p, IDM_INT_MASK, 0);
}

/*
 * The stock kernel's order: stopped, coalescing, buffer size, rings, sizes, every receive slot handed over. New rings
 * do not rewind the port: it resumes each ring at the slot after the frames sent or received but not yet
 * acknowledged (netifd reopens the port at boot, ten sent still counted; on the unit, restarting at slot 0 put
 * every frame a ring lap late), so the driver starts there and acknowledges them.
 */
static void idm_init_hw(struct idm *p)
{
	dma_addr_t tx_ring = p->ring_dma + SLOTS * DESC_SIZE;
	unsigned int i, tx_left, rx_left;

	idm_stop_hw(p);
	tx_left = rd(p, IDM_TX_SENT) & IDM_COUNT;
	rx_left = rd(p, IDM_RX_READY) & IDM_COUNT;
	wr(p, IDM_COAL_1, 0x40);
	wr(p, IDM_COAL_2, 0x4e20);
	wr(p, IDM_COAL_0, 0x20);
	wr(p, IDM_CFG, (rd(p, IDM_CFG) & ~(IDM_CFG_ON | IDM_CFG_BUF)) | IDM_CFG_ON |
		       FIELD_PREP(IDM_CFG_BUF, BUF_SIZE / 64));
	for (i = 0; i < 2 * SLOTS; i++) {
		p->ring[i * 2] = cpu_to_le32(p->bufs_dma + i * BUF_SIZE);
		p->ring[i * 2 + 1] = 0;
	}
	wmb();	/* the table before the port reads it */
	wr(p, IDM_RX_BASE, lower_32_bits(p->ring_dma));
	wr(p, IDM_RX_BASE + 4, upper_32_bits(p->ring_dma));
	wr(p, IDM_TX_BASE, lower_32_bits(tx_ring));
	wr(p, IDM_TX_BASE + 4, upper_32_bits(tx_ring));
	wr(p, IDM_RING_SIZE, SLOTS << 16 | SLOTS);
	wr(p, IDM_RX_ACK, SLOTS << 16);
	p->rx_next = rx_left % SLOTS;
	p->tx.next = tx_left % SLOTS;
	p->tx.inflight = 0;
	if (rx_left)
		wr(p, IDM_RX_ACK, rx_left << 16 | rx_left);
	if (tx_left)
		wr(p, IDM_TX_KICK, tx_left);
	netdev_info(p->ndev, "%u sent and %u received left unacknowledged: the rings start at those slots\n",
		    tx_left, rx_left);
}

static void idm_rx_one(struct idm *p, unsigned int slot)
{
	struct eth_rx_frame fr;
	struct sk_buff *skb;
	u32 w1 = le32_to_cpu(READ_ONCE(desc(p, false, slot)[1]));

	if (eth_rx_frame_geom(&rx_layout, eth_rx_dlen(w1, IDM_RXD_LEN_MASK), true, &fr) != ETH_RX_OK) {
		p->rx_bad_len++;
		p->ndev->stats.rx_length_errors++;
		return;
	}
	skb = eth_rx_frame_copy(&p->napi, buf(p, false, slot), &fr, 0);
	if (!skb) {
		p->rx_nomem++;
		p->ndev->stats.rx_dropped++;
		return;
	}
	eth_rx_len_check(&p->rx_len, DRV, skb->data, skb->len, false);
	skb->protocol = eth_type_trans(skb, p->ndev);
	p->ndev->stats.rx_packets++;
	p->ndev->stats.rx_bytes += skb->len;
	napi_gro_receive(&p->napi, skb);
}

static int idm_poll(struct napi_struct *napi, int budget)
{
	struct idm *p = container_of(napi, struct idm, napi);
	unsigned int n = min_t(unsigned int, rd(p, IDM_RX_READY) & IDM_COUNT, budget), i;

	dma_rmb();	/* the descriptors the count covers */
	for (i = 0; i < n; i++) {
		idm_rx_one(p, p->rx_next);
		p->rx_next = (p->rx_next + 1) % SLOTS;
	}
	if (n)
		wr(p, IDM_RX_ACK, n << 16 | n);
	if (n < budget && napi_complete_done(napi, n))
		wr(p, IDM_INT_MASK, rd(p, IDM_INT_MASK) | IDM_INT_RX);
	return n;
}

static irqreturn_t idm_irq(int irq, void *data)
{
	struct idm *p = data;
	u32 mask = rd(p, IDM_INT_MASK);

	if (!(rd(p, IDM_INT_STATUS) & mask & IDM_INT_RX))
		return IRQ_NONE;
	wr(p, IDM_INT_MASK, mask & ~IDM_INT_RX);
	napi_schedule(&p->napi);
	return IRQ_HANDLED;
}

static netdev_tx_t idm_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct idm *p = netdev_priv(ndev);
	unsigned int slot, sent;

	if (skb_put_padto(skb, ETH_ZLEN)) {
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	if (skb->len > BUF_SIZE) {
		ndev->stats.tx_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	spin_lock(&p->tx_lock);
	if (eth_tx_ring_ask(&p->tx, SLOTS)) {
		sent = rd(p, IDM_TX_SENT) & IDM_COUNT;
		eth_tx_ring_sent(&p->tx, sent);
		if (sent)
			wr(p, IDM_TX_KICK, sent);
	}
	if (!eth_tx_ring_room(&p->tx, SLOTS)) {
		spin_unlock(&p->tx_lock);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	slot = eth_tx_ring_take(&p->tx, SLOTS);
	skb_copy_bits(skb, 0, buf(p, true, slot), skb->len);
	WRITE_ONCE(desc(p, true, slot)[1], cpu_to_le32(skb->len << IDM_TXD_LEN_SHIFT));
	wmb();	/* buffer and descriptor before the kick */
	wr(p, IDM_TX_KICK, 1 << 16);
	spin_unlock(&p->tx_lock);
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_consume_skb_any(skb);
	return NETDEV_TX_OK;
}

static int idm_open(struct net_device *ndev)
{
	struct idm *p = netdev_priv(ndev);

	idm_init_hw(p);
	napi_enable(&p->napi);
	wr(p, IDM_RUN, 1);
	wr(p, IDM_INT_MASK, IDM_INT_RX);
	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int idm_stop(struct net_device *ndev)
{
	struct idm *p = netdev_priv(ndev);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	idm_stop_hw(p);
	napi_disable(&p->napi);
	return 0;
}

static const struct {
	char name[ETH_GSTRING_LEN];
	size_t off;
} idm_stats[] = {
	{ "rx_bad_len", offsetof(struct idm, rx_bad_len) },
	{ "rx_nomem", offsetof(struct idm, rx_nomem) },
	{ "rx_len_long_lan", offsetof(struct idm, rx_len.long_lan) },
	{ "rx_len_short_lan", offsetof(struct idm, rx_len.short_lan) },
};

static int idm_get_sset_count(struct net_device *ndev, int sset)
{
	return sset == ETH_SS_STATS ? ARRAY_SIZE(idm_stats) : -EOPNOTSUPP;
}

static void idm_get_strings(struct net_device *ndev, u32 sset, u8 *data)
{
	int i;

	if (sset != ETH_SS_STATS)
		return;
	for (i = 0; i < ARRAY_SIZE(idm_stats); i++)
		ethtool_puts(&data, idm_stats[i].name);
}

static void idm_get_stats(struct net_device *ndev, struct ethtool_stats *stats, u64 *data)
{
	struct idm *p = netdev_priv(ndev);
	int i;

	for (i = 0; i < ARRAY_SIZE(idm_stats); i++)
		data[i] = *(u64 *)((u8 *)p + idm_stats[i].off);
}

static const struct ethtool_ops idm_ethtool_ops = {
	.get_link = ethtool_op_get_link,
	.get_sset_count = idm_get_sset_count,
	.get_strings = idm_get_strings,
	.get_ethtool_stats = idm_get_stats,
};

static const struct net_device_ops idm_netdev_ops = {
	.ndo_open = idm_open,
	.ndo_stop = idm_stop,
	.ndo_start_xmit = idm_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

static void idm_release_mem(void *dev)
{
	of_reserved_mem_device_release(dev);
}

static int idm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct idm *p;
	int irq, ret;

	ndev = devm_alloc_etherdev(dev, sizeof(*p));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, dev);
	p = netdev_priv(ndev);
	p->dev = dev;
	p->ndev = ndev;
	spin_lock_init(&p->tx_lock);
	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);
	idm_stop_hw(p);		/* U-Boot leaves it receiving into its own rings */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");
	/* rings and buffers in the window the stock kernel keeps for this port */
	ret = of_reserved_mem_device_init(dev);
	if (ret)
		return dev_err_probe(dev, ret, "no memory-region\n");
	ret = devm_add_action_or_reset(dev, idm_release_mem, dev);
	if (ret)
		return ret;
	p->ring = dmam_alloc_coherent(dev, 2 * SLOTS * DESC_SIZE, &p->ring_dma, GFP_KERNEL);
	p->bufs = dmam_alloc_coherent(dev, 2 * SLOTS * BUF_SIZE, &p->bufs_dma, GFP_KERNEL);
	if (!p->ring || !p->bufs)
		return dev_err_probe(dev, -ENOMEM, "no DMA memory for the rings\n");
	ndev->netdev_ops = &idm_netdev_ops;
	ndev->ethtool_ops = &idm_ethtool_ops;
	ndev->max_mtu = BUF_SIZE - VLAN_ETH_HLEN;
	if (of_get_ethdev_address(dev->of_node, ndev))
		eth_hw_addr_random(ndev);
	netif_napi_add(ndev, &p->napi, idm_poll);
	ret = devm_request_irq(dev, irq, idm_irq, 0, DRV, p);
	if (ret)
		return dev_err_probe(dev, ret, "IRQ %d\n", irq);
	ret = eth_devm_register_netdev(dev, ndev, NULL, ETH_NO_ndo_tx_timeout | ETH_NO_get_link_ksettings |
							ETH_NO_set_link_ksettings | ETH_NO_get_pauseparam | ETH_NO_set_pauseparam |
							ETH_NO_get_ringparam | ETH_NO_get_regs | ETH_NO_rx_watch);
	if (ret)
		return dev_err_probe(dev, ret, "register_netdev\n");
	netdev_info(ndev, "%pM, %u + %u slots, rings %pad, buffers %pad\n", ndev->dev_addr, SLOTS, SLOTS,
		    &p->ring_dma, &p->bufs_dma);
	return 0;
}

static const struct of_device_id idm_of_match[] = {
	{ .compatible = "zte,zx279221-idm" },
	{ }
};
MODULE_DEVICE_TABLE(of, idm_of_match);

static struct platform_driver idm_driver = {
	.probe = idm_probe,
	.driver = {
		.name = DRV,
		.of_match_table = idm_of_match,
	},
};
module_platform_driver(idm_driver);

MODULE_DESCRIPTION("ZTE ZX279221 management Ethernet (switch CPU port)");
MODULE_LICENSE("GPL");
