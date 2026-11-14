// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE */
/*
 * ponvirt_olt.c -- the PON MAC of a virtual OLT (an OLT board's own VM): our OLT port core (gpon_olt_port.c) over
 * an Ethernet wire (owned: no bridge or IP stack on it) whose far end is the host's splitter (dev/virt/vodn
 * --olt-vm), at frame level (ponvirt_wire.h), the twin of ponvirt.c on the ONU side. As on the ONU VMs, the VM's
 * first NIC (eth0) is the management port and the second (eth1) the wire. Every serial heard is authorized (zero
 * touch). The data path is pon0: an ONU's upstream GEM frame is received on it; a frame sent on it goes down the GEM
 * port of the ONU that sent its destination address, or of every ONU in service for a group or unknown one, so a
 * Linux bridge of pon0 and the uplink is the OLT's layer-2 service. No optics, no burst timing, no BWmap.
 */
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <linux/rtnetlink.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>

#include "gpon_olt_port.h"
#include "gpon_sn.h"
#include "ponvirt_wire.h"

static char *wire = "eth1";
module_param(wire, charp, 0444);
MODULE_PARM_DESC(wire, "the NIC whose far end is the virtual ODN's splitter");
static uint onus = 8;
module_param(onus, uint, 0444);
MODULE_PARM_DESC(onus, "ONUs this PON port serves (an OMCI engine each)");
static ushort vlan = 100;
module_param(vlan, ushort, 0444);
MODULE_PARM_DESC(vlan, "the service VLAN every ONU's UNI is provisioned with (0: transparent)");

#define PVO_TICK_MS	10
#define PVO_MACS	256

struct pvo {
	struct net_device *wire, *pon;
	spinlock_t lock;		/* the core is single-threaded: every call holds this */
	struct timer_list tick;
	struct sk_buff_head out;	/* wire frames built under the lock, sent after it */
	struct gpon_olt_port port;
	struct gpon_olt_omci *eng;
	struct { u8 mac[ETH_ALEN]; u8 slot; bool used; } mac[PVO_MACS];
	u64 us_frames, ds_frames, wire_drop;
};

static struct pvo *pvo;

static u32 pvo_now(void)
{
	return jiffies_to_msecs(jiffies);
}

static void pvo_put16(u8 *d, u16 v)
{
	d[0] = v >> 8;
	d[1] = v & 0xff;
}

static u16 pvo_get16(const u8 *d)
{
	return ((u16)d[0] << 8) | d[1];
}

/* A wire frame, broadcast as on a fibre: the kind, then @len octets of @a and @blen of @b. Under the lock. */
static void pvo_emit(u8 kind, const u8 *a, unsigned int len, const u8 *b, unsigned int blen)
{
	struct sk_buff *skb = alloc_skb(LL_RESERVED_SPACE(pvo->wire) + ETH_HLEN + 1 + len + blen, GFP_ATOMIC);
	u8 *d;

	if (!skb) {
		pvo->wire_drop++;
		return;
	}
	skb_reserve(skb, LL_RESERVED_SPACE(pvo->wire) + ETH_HLEN);
	d = skb_put(skb, 1 + len + blen);
	d[0] = kind;
	memcpy(d + 1, a, len);
	if (blen)
		memcpy(d + 1 + len, b, blen);
	skb->dev = pvo->wire;
	skb->protocol = htons(PONVIRT_ETHERTYPE);
	dev_hard_header(skb, pvo->wire, PONVIRT_ETHERTYPE, pvo->wire->broadcast, pvo->wire->dev_addr, skb->len);
	__skb_queue_tail(&pvo->out, skb);
	pvo->ds_frames++;
}

static void pvo_flush(void)
{
	struct sk_buff_head q;
	struct sk_buff *skb;

	__skb_queue_head_init(&q);
	spin_lock_bh(&pvo->lock);
	skb_queue_splice_init(&pvo->out, &q);
	spin_unlock_bh(&pvo->lock);
	while ((skb = __skb_dequeue(&q)))
		dev_queue_xmit(skb);
}

/* ---- the OLT port's ops ---------------------------------------------------------------- */

static void pvo_ds_ploam(void *sh, const u8 m[GPON_OLT_PLOAM_DS_LEN])
{
	pvo_emit(PV_DS_PLOAM, m, GPON_OLT_PLOAM_DS_LEN, NULL, 0);
}

static void pvo_sn_grant(void *sh)
{
	pvo_emit(PV_SN_GRANT, NULL, 0, NULL, 0);
}

static void pvo_ranging_grant(void *sh, u8 onu_id)
{
	pvo_emit(PV_RANGING_GRANT, &onu_id, 1, NULL, 0);
}

static int pvo_ds_omci(void *sh, u16 gem_port, const u8 *pdu, unsigned int len)
{
	u8 gem[2];

	pvo_put16(gem, gem_port);
	pvo_emit(PV_DS_OMCI, gem, 2, pdu, len);
	return 0;
}

static void pvo_event(void *sh, u8 onu_id, const char *what, const char *detail)
{
	pr_info("ponvirt_olt: onu %u %s%s%s\n", onu_id, what, detail ? " " : "", detail ? detail : "");
}

static void pvo_unsupported(void *sh, const char *kind, int cls, u32 val, const char *want, const u8 *data,
			    unsigned int len)
{
	pr_info_ratelimited("ponvirt_olt: unsupported %s %d 0x%x (want %s)\n", kind, cls, val, want ? want : "-");
}

static const struct gpon_olt_port_ops pvo_ops = {
	.ds_ploam = pvo_ds_ploam, .sn_grant = pvo_sn_grant, .ranging_grant = pvo_ranging_grant,
	.ds_omci = pvo_ds_omci, .event = pvo_event, .unsupported = pvo_unsupported,
};

/* ---- the data path: which ONU sent an address ------------------------------------------- */

static unsigned int pvo_hash(const u8 *mac)
{
	return (mac[4] * 31u + mac[5]) % PVO_MACS;
}

static void pvo_learn(const u8 *mac, unsigned int slot)
{
	unsigned int i, h = pvo_hash(mac);

	for (i = 0; i < PVO_MACS; i++, h = (h + 1) % PVO_MACS)
		if (!pvo->mac[h].used || ether_addr_equal(pvo->mac[h].mac, mac)) {
			ether_addr_copy(pvo->mac[h].mac, mac);
			pvo->mac[h].slot = (u8)slot;
			pvo->mac[h].used = true;
			return;
		}
}

static int pvo_owner(const u8 *mac)
{
	unsigned int i, h = pvo_hash(mac);

	for (i = 0; i < PVO_MACS && pvo->mac[h].used; i++, h = (h + 1) % PVO_MACS)
		if (ether_addr_equal(pvo->mac[h].mac, mac))
			return pvo->mac[h].slot;
	return -1;
}

static void pvo_up_gem(const u8 *d, unsigned int n)
{
	int slot = n >= PV_US_GEM_HDR + ETH_HLEN ? gpon_olt_port_slot_of_gem(&pvo->port, pvo_get16(d + 3)) : -1;
	struct sk_buff *skb;

	if (slot < 0) {
		pvo->pon->stats.rx_dropped++;
		return;
	}
	pvo_learn(d + PV_US_GEM_HDR + ETH_ALEN, (unsigned int)slot);
	skb = netdev_alloc_skb_ip_align(pvo->pon, n - PV_US_GEM_HDR);
	if (!skb) {
		pvo->pon->stats.rx_dropped++;
		return;
	}
	skb_put_data(skb, d + PV_US_GEM_HDR, n - PV_US_GEM_HDR);
	skb->protocol = eth_type_trans(skb, pvo->pon);
	pvo->pon->stats.rx_packets++;
	pvo->pon->stats.rx_bytes += skb->len + ETH_HLEN;
	netif_rx(skb);
}

static netdev_tx_t pvo_pon_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned int slot;
	int owner;
	u8 gem[2];

	if (skb_linearize(skb) || skb->len < ETH_HLEN) {
		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	spin_lock_bh(&pvo->lock);
	owner = is_multicast_ether_addr(skb->data) ? -1 : pvo_owner(skb->data);
	for (slot = 0; slot < pvo->port.n_eng; slot++) {
		if ((owner >= 0 && slot != (unsigned int)owner) || !gpon_olt_port_online(&pvo->port, slot))
			continue;
		pvo_put16(gem, gpon_olt_port_gem(&pvo->port, slot));
		pvo_emit(PV_DS_GEM, gem, 2, skb->data, skb->len);
	}
	spin_unlock_bh(&pvo->lock);
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);
	pvo_flush();
	return NETDEV_TX_OK;
}

static int pvo_pon_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}

static int pvo_pon_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static const struct net_device_ops pvo_pon_ops = {
	.ndo_open = pvo_pon_open,
	.ndo_stop = pvo_pon_stop,
	.ndo_start_xmit = pvo_pon_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

/* ---- upstream: one wire frame is one PON event ------------------------------------------- */

static void pvo_us(const u8 *d, unsigned int n, u32 now)
{
	pvo->us_frames++;
	switch (d[0]) {
	case PV_US_PLOAM:
		if (n >= 1 + GPON_OLT_PLOAM_US_LEN)
			gpon_olt_port_us_ploam(&pvo->port, d + 1, GPON_OLT_PLOAM_US_LEN, now);
		break;
	case PV_RANGING_BURST:
		/* the round trip vodn gives each ONU-ID: a fibre length per ONU, constant */
		if (n >= 2)
			gpon_olt_port_ranging_burst(&pvo->port, d[1], 100 + 40 * d[1], now);
		break;
	case PV_US_OMCI:
		if (n >= PV_GEM_HDR + GPON_OLT_OMCI_LEN)
			gpon_olt_port_us_omci(&pvo->port, pvo_get16(d + 1), d + PV_GEM_HDR, GPON_OLT_OMCI_LEN, now);
		break;
	case PV_US_GEM:
		pvo_up_gem(d, n);
		break;
	}
}

/* The wire is the fibre: this PON MAC owns it (rx_handler), so no bridge or IP stack takes its frames. */
static rx_handler_result_t pvo_rx(struct sk_buff **pskb)
{
	struct sk_buff *skb = skb_share_check(*pskb, GFP_ATOMIC);

	if (!skb)
		return RX_HANDLER_CONSUMED;
	spin_lock_bh(&pvo->lock);
	if (skb->protocol == htons(PONVIRT_ETHERTYPE) && !skb_linearize(skb) && skb->len >= 1)
		pvo_us(skb->data, skb->len, pvo_now());
	else
		pvo->wire_drop++;
	spin_unlock_bh(&pvo->lock);
	pvo_flush();
	consume_skb(skb);
	return RX_HANDLER_CONSUMED;
}

static void pvo_tick(struct timer_list *t)
{
	spin_lock_bh(&pvo->lock);
	gpon_olt_port_tick(&pvo->port, pvo_now());
	spin_unlock_bh(&pvo->lock);
	pvo_flush();
	mod_timer(&pvo->tick, jiffies + msecs_to_jiffies(PVO_TICK_MS));
}

/* /proc/ponvirt_olt: one line per ONU slot heard, read the same way by the host witness and a person. */
static int pvo_show(struct seq_file *m, void *v)
{
	char sn[GPON_SN_TEXT_SIZE];
	unsigned int i;

	spin_lock_bh(&pvo->lock);
	seq_printf(m, "wire %s us %llu ds %llu drop %llu pon0 rx %lu tx %lu\n", wire, pvo->us_frames,
		   pvo->ds_frames, pvo->wire_drop, pvo->pon->stats.rx_packets, pvo->pon->stats.tx_packets);
	for (i = 0; i < pvo->port.n_eng; i++) {
		const struct gpon_olt_link_rec *r = &pvo->port.pon.onu[i];
		const struct gpon_olt_omci *e = &pvo->eng[i];

		if (!r->used)
			continue;
		gpon_sn_format(r->sn, sn);
		seq_printf(m, "onu %u sn %s link %s phase %s online %d gem %u alloc %u omci tx %u rx %u fail %s\n",
			   gpon_olt_onu_id(i), sn, gpon_olt_link_name(r->state), gpon_olt_phase_name(e->phase),
			   e->online, e->prof.gem_port, e->prof.alloc_id, e->tx, e->rx,
			   e->fail.why ? gpon_olt_why_name(e->fail.why) : "-");
	}
	spin_unlock_bh(&pvo->lock);
	return 0;
}

static int pvo_open_wire(void)
{
	int rc;

	pvo->wire = dev_get_by_name(&init_net, wire);
	if (!pvo->wire) {
		pr_err("ponvirt_olt: no NIC %s: the virtual ODN is unreachable\n", wire);
		return -ENODEV;
	}
	rtnl_lock();
	rc = dev_set_mtu(pvo->wire, PONVIRT_MTU);
	if (!rc)
		rc = dev_open(pvo->wire, NULL);
	rtnl_unlock();
	if (rc) {
		pr_err("ponvirt_olt: %s: MTU %u / open refused (%d)\n", wire, PONVIRT_MTU, rc);
		dev_put(pvo->wire);
	}
	return rc;
}

static int __init ponvirt_olt_init(void)
{
	struct gpon_olt_profile tmpl;
	const char *refused;
	int rc;

	if (!onus || onus > GPON_OLT_PON_ONUS) {
		pr_err("ponvirt_olt: onus=%u, 1..%u\n", onus, GPON_OLT_PON_ONUS);
		return -EINVAL;
	}
	pvo = kzalloc(sizeof(*pvo), GFP_KERNEL);
	if (!pvo)
		return -ENOMEM;
	pvo->eng = vzalloc(array_size(onus, sizeof(*pvo->eng)));
	rc = -ENOMEM;
	if (!pvo->eng)
		goto free_pvo;
	spin_lock_init(&pvo->lock);
	skb_queue_head_init(&pvo->out);
	gpon_olt_profile_default(&tmpl);
	tmpl.alloc_id = 256;	/* vodn's numbering: ONU-ID k gets alloc-id 256+k and GEM port 1024+k */
	tmpl.gem_port = 1024;
	tmpl.vlan = vlan;
	refused = gpon_olt_port_init(&pvo->port, &pvo_ops, pvo, pvo->eng, onus, &tmpl, true);
	rc = -EINVAL;
	if (refused) {
		pr_err("ponvirt_olt: the OLT port refused its %s\n", refused);
		goto free_eng;
	}
	rc = pvo_open_wire();
	if (rc)
		goto free_eng;
	rc = -ENOMEM;
	pvo->pon = alloc_etherdev(0);
	if (!pvo->pon)
		goto put_wire;
	strscpy(pvo->pon->name, "pon0", IFNAMSIZ);
	pvo->pon->netdev_ops = &pvo_pon_ops;
	pvo->pon->max_mtu = PONVIRT_MTU - PV_GEM_HDR - ETH_HLEN;
	eth_hw_addr_random(pvo->pon);
	rc = register_netdev(pvo->pon);
	if (rc) {
		pr_err("ponvirt_olt: pon0 not registered (%d)\n", rc);
		goto free_pon;
	}
	if (!proc_create_single("ponvirt_olt", 0444, NULL, pvo_show)) {
		pr_err("ponvirt_olt: /proc/ponvirt_olt not created\n");
		rc = -ENOMEM;
		goto unregister;
	}
	rtnl_lock();
	rc = netdev_rx_handler_register(pvo->wire, pvo_rx, pvo);
	rtnl_unlock();
	if (rc) {
		pr_err("ponvirt_olt: %s already owned by another driver (%d)\n", wire, rc);
		goto remove_proc;
	}
	timer_setup(&pvo->tick, pvo_tick, 0);
	mod_timer(&pvo->tick, jiffies + msecs_to_jiffies(PVO_TICK_MS));
	pr_info("ponvirt_olt: OLT PON port on %s, %u ONUs, service VLAN %u, data path pon0\n", wire, onus, vlan);
	return 0;

remove_proc:
	remove_proc_entry("ponvirt_olt", NULL);
unregister:
	unregister_netdev(pvo->pon);
free_pon:
	free_netdev(pvo->pon);
put_wire:
	dev_put(pvo->wire);
free_eng:
	vfree(pvo->eng);
free_pvo:
	kfree(pvo);
	pvo = NULL;
	return rc;
}
late_initcall(ponvirt_olt_init);

MODULE_DESCRIPTION("Virtual OLT PON MAC: the GPON OLT core over an Ethernet wire");
MODULE_LICENSE("GPL");
