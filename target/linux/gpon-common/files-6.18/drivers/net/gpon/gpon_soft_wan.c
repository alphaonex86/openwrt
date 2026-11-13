// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CORE, not in the STRICT subset */
/* gpon_soft_wan.c -- see gpon_soft_wan.h. */
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/netdevice.h>
#include <linux/unaligned.h>

#include "gpon_common.h"
#include "gpon_soft_wan.h"

static int wan_meter_set(void *sh, u16 gem, enum omci_meter_dir dir, const struct gpon_gem_td *td)
{
	struct gpon_soft_wan *w = sh;

	if (dir != OMCI_METER_US)
		return td ? -EOPNOTSUPP : 0;
	if (gem != w->svc.gem_port)
		return td ? -ENOENT : 0;
	w->us_pir = td ? td->pir : 0;
	w->us_burst = max_t(u32, td && td->pbs ? td->pbs : w->us_pir / 50, ETH_FRAME_LEN + VLAN_HLEN);
	return 0;
}

static const struct omci_meter_ops wan_meter_ops = { .rate_set = wan_meter_set };

/* The upstream PIR as a token bucket refilled by elapsed time: @len taken, or the frame is dropped. */
static bool wan_meter_admit(struct gpon_soft_wan *w, unsigned int len)
{
	unsigned long now = jiffies;
	u64 fill;

	if (!w->us_pir)
		return true;
	fill = div_u64((u64)w->us_pir * jiffies_to_usecs(now - w->us_stamp), USEC_PER_SEC);
	w->us_stamp = now;
	w->us_tokens = min_t(u64, w->us_tokens + fill, w->us_burst);
	if (w->us_tokens < len)
		return false;
	w->us_tokens -= len;
	return true;
}

void gpon_soft_wan_poll(struct gpon_soft_wan *w, const struct omci_onu *onu, bool omcc_up, u16 omcc_gem)
{
	struct omci_vlan_service s;
	bool up = omcc_up && omci_vlan_services(onu, omcc_gem, GPON_MCAST_GEM_PORT, &s, 1) == 1 && s.alloc_known;

	if (up && (!w->up || s.vid != w->svc.vid || s.gem_port != w->svc.gem_port || s.alloc_id != w->svc.alloc_id)) {
		w->svc = s;
		netdev_info(w->dev, "WAN service VLAN %u on GEM %u alloc %u\n", s.vid, s.gem_port, s.alloc_id);
	}
	omci_meters_apply(&w->meter, &s, up ? 1 : 0);
	if (up == w->up)
		return;
	w->up = up;
	if (up)
		netif_carrier_on(w->dev);
	else
		netif_carrier_off(w->dev);
}
EXPORT_SYMBOL_GPL(gpon_soft_wan_poll);

void gpon_soft_wan_rx(struct gpon_soft_wan *w, u16 gem, const u8 *f, unsigned int len)
{
	bool tagged = len >= VLAN_ETH_HLEN && get_unaligned_be16(f + 12) == ETH_P_8021Q;
	unsigned int cut = w->svc.vid && tagged && (get_unaligned_be16(f + 14) & VLAN_VID_MASK) == w->svc.vid ?
			   VLAN_HLEN : 0;
	struct sk_buff *skb = NULL;

	if (w->up && gem == w->svc.gem_port && len >= ETH_HLEN && (!w->svc.vid || cut))
		skb = netdev_alloc_skb(w->dev, len - cut + NET_IP_ALIGN);
	if (!skb) {
		w->dev->stats.rx_dropped++;
		return;
	}
	skb_reserve(skb, NET_IP_ALIGN);
	skb_put_data(skb, f, 12);
	skb_put_data(skb, f + 12 + cut, len - 12 - cut);
	skb->protocol = eth_type_trans(skb, w->dev);
	w->dev->stats.rx_packets++;
	w->dev->stats.rx_bytes += skb->len;
	netif_rx(skb);
}
EXPORT_SYMBOL_GPL(gpon_soft_wan_rx);

static netdev_tx_t wan_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct gpon_soft_wan *w = *(struct gpon_soft_wan **)netdev_priv(dev);
	struct omci_vlan_service svc;
	bool admitted;

	spin_lock_bh(w->lock);
	svc = w->svc;
	admitted = w->up && skb->len >= ETH_HLEN && wan_meter_admit(w, skb->len);
	spin_unlock_bh(w->lock);
	if (admitted && svc.vid)
		skb = vlan_insert_tag(skb, htons(ETH_P_8021Q), svc.vid);	/* frees it when it fails */
	if (admitted && skb && !skb_linearize(skb) && !w->tx(w->sh, &svc, skb->data, skb->len)) {
		dev->stats.tx_packets++;
		dev->stats.tx_bytes += skb->len;
	} else {
		dev->stats.tx_dropped++;
	}
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops wan_ops = {
	.ndo_start_xmit = wan_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

int gpon_soft_wan_init(struct gpon_soft_wan *w, struct omci_onu *onu, const u8 *mac)
{
	int rc;

	if (!w->lock || !w->tx)
		return -EINVAL;
	w->dev = alloc_etherdev(sizeof(w));
	if (!w->dev)
		return -ENOMEM;
	*(struct gpon_soft_wan **)netdev_priv(w->dev) = w;
	strscpy(w->dev->name, "gpon0", IFNAMSIZ);
	w->dev->netdev_ops = &wan_ops;
	if (mac)
		eth_hw_addr_set(w->dev, mac);
	else
		eth_hw_addr_random(w->dev);
	netif_carrier_off(w->dev);
	rc = omci_meter_init(&w->meter, &wan_meter_ops, w);
	if (!rc)
		rc = register_netdev(w->dev);
	if (rc) {
		free_netdev(w->dev);
		w->dev = NULL;
		return rc;
	}
	omci_onu_set_wan_mac(onu, w->dev->dev_addr);
	return 0;
}
EXPORT_SYMBOL_GPL(gpon_soft_wan_init);

void gpon_soft_wan_exit(struct gpon_soft_wan *w)
{
	if (!w->dev)
		return;
	unregister_netdev(w->dev);
	free_netdev(w->dev);
	w->dev = NULL;
}
EXPORT_SYMBOL_GPL(gpon_soft_wan_exit);
