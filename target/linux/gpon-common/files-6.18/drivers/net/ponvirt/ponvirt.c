// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ponvirt.c -- the PON MAC of a virtual ONU (QEMU): the boards' own gpon_* ONU core over an
 * Ethernet wire to the host's virtual ODN (dev/virt/vodn.c), at frame level (ponvirt_wire.h).
 * The WAN is gpon0 as on the boards; the service VLAN the OLT provisions is pushed upstream and
 * popped downstream here, as the ONU's VLAN row does on silicon. No optics, no burst timing.
 */
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/rtnetlink.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include "gpon_ploam.h"
#include "gpon_omci_core.h"
#include "gpon_omci_me.h"
#include "gpon_sn.h"
#include "ponvirt_wire.h"

static char *wire = "eth1";
module_param(wire, charp, 0444);
MODULE_PARM_DESC(wire, "the NIC whose far end is the virtual ODN");
static char *sn = "XPON00000001";
module_param(sn, charp, 0444);
MODULE_PARM_DESC(sn, "this virtual unit's G.984.3 serial number, AAAAhhhhhhhh");

#define PV_TICK_MS	20

struct ponvirt {
	struct net_device *wire, *wan;
	struct packet_type pt;
	spinlock_t lock;		/* the core is single-threaded: every call holds this */
	struct timer_list tick;
	struct sk_buff_head out;	/* wire frames built under the lock, sent after it */
	struct gpon_ploam p;
	struct gpon_ploam_cfg cfg;
	struct omci_onu *onu;
	u8 sn[GPON_SN_BYTES];
	u8 offer[GPON_PLOAM_US_LEN];
	bool has_offer, omcc;
	u16 omcc_gem;
	struct omci_vlan_service svc;
	bool svc_up;
	u8 uni_port[8], uni_port_n;
	u64 omci_rx, omci_tx, gem_rx, gem_tx, gem_drop;
};

static struct ponvirt *pv;

static u32 pv_now(void)
{
	return jiffies_to_msecs(jiffies);
}

/* A wire frame: Ethernet header, the kind, then @len octets of @a and @blen of @b. */
static void pv_emit(u8 kind, const u8 *a, unsigned int len, const u8 *b, unsigned int blen)
{
	struct sk_buff *skb = alloc_skb(LL_RESERVED_SPACE(pv->wire) + ETH_HLEN + 1 + len + blen,
					GFP_ATOMIC);
	u8 *d;

	if (!skb) {
		pv->gem_drop++;
		return;
	}
	skb_reserve(skb, LL_RESERVED_SPACE(pv->wire) + ETH_HLEN);
	d = skb_put(skb, 1 + len + blen);
	d[0] = kind;
	memcpy(d + 1, a, len);
	if (blen)
		memcpy(d + 1 + len, b, blen);
	skb->dev = pv->wire;
	skb->protocol = htons(PONVIRT_ETHERTYPE);
	dev_hard_header(skb, pv->wire, PONVIRT_ETHERTYPE, pv->wire->broadcast,
			pv->wire->dev_addr, skb->len);
	__skb_queue_tail(&pv->out, skb);
}

static void pv_flush(void)
{
	struct sk_buff_head q;
	struct sk_buff *skb;

	__skb_queue_head_init(&q);
	spin_lock_bh(&pv->lock);
	skb_queue_splice_init(&pv->out, &q);
	spin_unlock_bh(&pv->lock);
	while ((skb = __skb_dequeue(&q)))
		dev_queue_xmit(skb);
}

static void pv_put16(u8 *d, u16 v)
{
	d[0] = v >> 8;
	d[1] = v & 0xff;
}

static u16 pv_get16(const u8 *d)
{
	return ((u16)d[0] << 8) | d[1];
}

/* ---- the PLOAM core's ops: a frame for each message, nothing for the silicon ---------- */

static void pv_ploam_tx(void *sh, u8 queue, const u8 m[GPON_PLOAM_US_LEN])
{
	if (queue == PLM_US_QUEUE_SN) {
		memcpy(pv->offer, m, GPON_PLOAM_US_LEN);
		pv->has_offer = true;
		return;
	}
	pv_emit(PV_US_PLOAM, m, GPON_PLOAM_US_LEN, NULL, 0);
}

static int pv_install_omcc(void *sh, u16 gem)
{
	pv->omcc = true;
	pv->omcc_gem = gem;
	return 0;
}

static void pv_report_oper_up(void *sh)
{
	u8 msg[OMCI_LEN], gem[2];

	if (pv->omcc && omci_onu_emit_veip_up_avc(pv->onu, msg) == OMCI_LEN) {
		pv_put16(gem, pv->omcc_gem);
		pv_emit(PV_US_OMCI, gem, 2, msg, OMCI_LEN);
		pv->omci_tx++;
	}
}

static void pv_rng(void *sh, u8 *out, unsigned int len)
{
	get_random_bytes(out, len);
}

static void pv_unsupported(void *sh, const char *kind, int cls, u32 val, const char *want,
			   const u8 *data, unsigned int len)
{
	pr_info_ratelimited("ponvirt: unsupported %s %d 0x%x (want %s)\n", kind, cls, val,
			    want ? want : "-");
}

static void pv_boh(void *sh, u32 cfg, const u8 *oh, u8 size) { }
static u32 pv_min_delay(void *sh) { return 0; }
static void pv_eqd(void *sh, u32 multiframe, u32 intraframe) { }
static void pv_void(void *sh) { }
static void pv_state(void *sh, enum gpon_ostate st) { }
static void pv_onu_id(void *sh, u8 onu_id) { }
static void pv_key(void *sh, const u8 key[16]) { }
static void pv_switch(void *sh, u32 superframe) { }
static int pv_tcont(void *sh, u8 tcont, u16 alloc) { return 0; }
static void pv_gem(void *sh, u16 gem) { }

static const struct gpon_ploam_ops pv_ops = {
	.ploam_tx = pv_ploam_tx, .boh_write = pv_boh, .get_min_delay = pv_min_delay,
	.set_eqd = pv_eqd, .us_ploam_flush = pv_void, .set_hw_state = pv_state,
	.set_hw_onu_id = pv_onu_id, .on_below_o5 = pv_void, .o5_rearm_burst = pv_void,
	.analog_relock = pv_void, .o3_feed_reset = pv_void, .cdr_reseat = pv_void,
	.rng = pv_rng, .aes_stage_key = pv_key, .aes_arm_switch = pv_switch,
	.install_omcc = pv_install_omcc, .install_tcont = pv_tcont, .install_data_gem = pv_gem,
	.omci_report_oper_up = pv_report_oper_up, .unsupported = pv_unsupported,
};

/* ---- the WAN data path ---------------------------------------------------------------- */

/* The WAN service the OLT provisioned, re-read every tick: carrier follows it. */
static void pv_service(void)
{
	struct omci_vlan_service s;
	bool up = pv->p.state == GPON_O5_OPERATION && pv->omcc &&
		  omci_vlan_services(pv->onu, pv->omcc_gem, GPON_MCAST_GEM_PORT, &s, 1) == 1 &&
		  s.alloc_known;

	if (up && (!pv->svc_up || s.vid != pv->svc.vid || s.gem_port != pv->svc.gem_port ||
		   s.alloc_id != pv->svc.alloc_id)) {
		pv->svc = s;
		pr_info("ponvirt: WAN service VLAN %u on GEM %u alloc %u\n", s.vid, s.gem_port,
			s.alloc_id);
	}
	if (up != pv->svc_up) {
		pv->svc_up = up;
		if (up)
			netif_carrier_on(pv->wan);
		else
			netif_carrier_off(pv->wan);
	}
}

static void pv_ds_gem(u16 gem, const u8 *f, unsigned int len)
{
	bool tagged = len >= VLAN_ETH_HLEN && pv_get16(f + 12) == ETH_P_8021Q;
	unsigned int cut = pv->svc.vid && tagged && (pv_get16(f + 14) & VLAN_VID_MASK) == pv->svc.vid
			   ? VLAN_HLEN : 0;
	struct sk_buff *skb;

	if (!pv->svc_up || gem != pv->svc.gem_port || len < ETH_HLEN || (pv->svc.vid && !cut)) {
		pv->gem_drop++;
		return;
	}
	skb = netdev_alloc_skb(pv->wan, len - cut + NET_IP_ALIGN);
	if (!skb) {
		pv->gem_drop++;
		return;
	}
	skb_reserve(skb, NET_IP_ALIGN);
	skb_put_data(skb, f, 12);
	skb_put_data(skb, f + 12 + cut, len - 12 - cut);
	skb->protocol = eth_type_trans(skb, pv->wan);
	pv->gem_rx++;
	pv->wan->stats.rx_packets++;
	pv->wan->stats.rx_bytes += skb->len;
	netif_rx(skb);
}

static netdev_tx_t pv_wan_xmit(struct sk_buff *skb, struct net_device *dev)
{
	u8 hdr[4 + ETH_ALEN * 2 + VLAN_HLEN];

	if (skb_linearize(skb) || skb->len < ETH_HLEN)
		goto drop;
	spin_lock_bh(&pv->lock);
	if (!pv->svc_up) {
		spin_unlock_bh(&pv->lock);
		goto drop;
	}
	pv_put16(hdr, pv->svc.alloc_id);
	pv_put16(hdr + 2, pv->svc.gem_port);
	if (pv->svc.vid) {
		memcpy(hdr + 4, skb->data, ETH_ALEN * 2);
		pv_put16(hdr + 16, ETH_P_8021Q);
		pv_put16(hdr + 18, pv->svc.vid);
		pv_emit(PV_US_GEM, hdr, sizeof(hdr), skb->data + ETH_ALEN * 2, skb->len - ETH_ALEN * 2);
	} else {
		pv_emit(PV_US_GEM, hdr, 4, skb->data, skb->len);
	}
	pv->gem_tx++;
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;
	spin_unlock_bh(&pv->lock);
	consume_skb(skb);
	pv_flush();
	return NETDEV_TX_OK;
drop:
	dev->stats.tx_dropped++;
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops pv_wan_ops = {
	.ndo_start_xmit = pv_wan_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

/* ---- downstream: one wire frame is one PON event -------------------------------------- */

static void pv_ds(const u8 *d, unsigned int n, u32 now)
{
	u8 resp[OMCI_LEN], gem[2];

	switch (d[0]) {
	case PV_DS_PLOAM:
		if (n >= 1 + GPON_PLOAM_DS_LEN)
			gpon_ploam_ds(&pv->p, d + 1, GPON_PLOAM_DS_LEN, now);
		break;
	case PV_SN_GRANT:
		if (pv->has_offer && pv->p.state == GPON_O3_SERIAL)
			pv_emit(PV_US_PLOAM, pv->offer, GPON_PLOAM_US_LEN, NULL, 0);
		break;
	case PV_RANGING_GRANT:
		if (n >= 2 && d[1] == pv->p.onu_id && pv->p.state == GPON_O4_RANGING)
			pv_emit(PV_RANGING_BURST, d + 1, 1, NULL, 0);
		break;
	case PV_DS_OMCI:
		if (n < PV_GEM_HDR + OMCI_LEN || !pv->omcc || pv_get16(d + 1) != pv->omcc_gem ||
		    pv->p.state != GPON_O5_OPERATION)
			break;
		pv->omci_rx++;
		if (omci_onu_input(pv->onu, d + PV_GEM_HDR, OMCI_LEN, resp) == OMCI_LEN) {
			pv_put16(gem, pv->omcc_gem);
			pv_emit(PV_US_OMCI, gem, 2, resp, OMCI_LEN);
			pv->omci_tx++;
		}
		break;
	case PV_DS_GEM:
		if (n > PV_GEM_HDR)
			pv_ds_gem(pv_get16(d + 1), d + PV_GEM_HDR, n - PV_GEM_HDR);
		break;
	}
}

static int pv_rx(struct sk_buff *skb, struct net_device *dev, struct packet_type *pt,
		 struct net_device *orig)
{
	skb = skb_share_check(skb, GFP_ATOMIC);
	if (!skb)
		return NET_RX_DROP;
	if (dev == pv->wire && !skb_linearize(skb) && skb->len >= 1) {
		spin_lock_bh(&pv->lock);
		pv_ds(skb->data, skb->len, pv_now());
		spin_unlock_bh(&pv->lock);
		pv_flush();
	}
	consume_skb(skb);
	return NET_RX_SUCCESS;
}

static void pv_tick(struct timer_list *t)
{
	u32 now = pv_now();

	spin_lock_bh(&pv->lock);
	gpon_ploam_tick(&pv->p);
	gpon_ploam_poll_provision(&pv->p, now);
	gpon_ploam_poll_keepalive(&pv->p, now);
	pv_service();
	spin_unlock_bh(&pv->lock);
	pv_flush();
	mod_timer(&pv->tick, jiffies + msecs_to_jiffies(PV_TICK_MS));
}

/* /proc/ponvirt: one line the host launcher and a person read the same way. */
static int pv_show(struct seq_file *m, void *v)
{
	spin_lock_bh(&pv->lock);
	seq_printf(m, "state O%d onu_id %u omcc %s gem %u service %s vid %u gem %u alloc %u "
		   "omci rx %llu tx %llu gem rx %llu tx %llu drop %llu\n",
		   pv->p.state, pv->p.onu_id, pv->omcc ? "up" : "down", pv->omcc_gem,
		   pv->svc_up ? "up" : "down", pv->svc.vid, pv->svc.gem_port, pv->svc.alloc_id,
		   pv->omci_rx, pv->omci_tx, pv->gem_rx, pv->gem_tx, pv->gem_drop);
	spin_unlock_bh(&pv->lock);
	return 0;
}

static const void *pv_of_prop(void *np, const char *name, int *len)
{
	*len = -1;
	return of_get_property(np, name, len);
}

/* The board's own /omci-uni panel: a virtual X400AXF declares what the X400AXF declares. */
static void pv_declare_panel(void)
{
	struct device_node *np = of_find_node_by_path("/omci-uni");
	struct omci_panel_decl d;

	if (!np) {
		pr_warn("ponvirt: no /omci-uni in this device tree: the single-UNI default\n");
		return;
	}
	omci_onu_declare_panel(pv->onu, pv_of_prop, np, pv->uni_port, ARRAY_SIZE(pv->uni_port),
			       &pv->uni_port_n, &d);
	of_node_put(np);
	if (d.unis_why)
		pr_err("ponvirt: /omci-uni REFUSED: %s\n", d.unis_why);
	if (d.part)
		pr_err("ponvirt: /omci-uni %s REFUSED: %s\n", d.part, d.part_why);
}

static int pv_open_wire(void)
{
	int rc;

	pv->wire = dev_get_by_name(&init_net, wire);
	if (!pv->wire) {
		pr_err("ponvirt: no NIC %s: the virtual ODN is unreachable\n", wire);
		return -ENODEV;
	}
	rtnl_lock();
	rc = dev_set_mtu(pv->wire, PONVIRT_MTU);
	if (!rc)
		rc = dev_open(pv->wire, NULL);
	rtnl_unlock();
	if (rc)
		pr_err("ponvirt: %s: MTU %u / open refused (%d)\n", wire, PONVIRT_MTU, rc);
	return rc;
}

static int __init ponvirt_init(void)
{
	const char *missing;
	int rc;

	pv = kzalloc(sizeof(*pv), GFP_KERNEL);
	if (!pv)
		return -ENOMEM;
	pv->onu = kzalloc(sizeof(*pv->onu), GFP_KERNEL);
	if (!pv->onu)
		return -ENOMEM;
	spin_lock_init(&pv->lock);
	skb_queue_head_init(&pv->out);
	if (gpon_sn_parse(sn, pv->sn)) {
		pr_err("ponvirt: sn '%s' is not AAAAhhhhhhhh\n", sn);
		return -EINVAL;
	}
	rc = pv_open_wire();
	if (rc)
		return rc;
	pv->wan = alloc_etherdev(0);
	if (!pv->wan)
		return -ENOMEM;
	strscpy(pv->wan->name, "gpon0", IFNAMSIZ);
	pv->wan->netdev_ops = &pv_wan_ops;
	eth_hw_addr_random(pv->wan);
	netif_carrier_off(pv->wan);
	rc = register_netdev(pv->wan);
	if (rc) {
		pr_err("ponvirt: gpon0 not registered (%d)\n", rc);
		return rc;
	}
	omci_onu_init(pv->onu, pv->sn, get_random_u8());
	omci_onu_set_wan_mac(pv->onu, pv->wan->dev_addr);
	pv_declare_panel();
	missing = gpon_ploam_init(&pv->p, &pv_ops, &pv->cfg, pv, pv->sn);
	if (missing) {
		pr_err("ponvirt: the PLOAM core refused the shell: %s\n", missing);
		return -EINVAL;
	}
	proc_create_single("ponvirt", 0444, NULL, pv_show);
	pv->pt.type = htons(PONVIRT_ETHERTYPE);
	pv->pt.dev = pv->wire;
	pv->pt.func = pv_rx;
	dev_add_pack(&pv->pt);
	timer_setup(&pv->tick, pv_tick, 0);
	mod_timer(&pv->tick, jiffies + msecs_to_jiffies(PV_TICK_MS));
	pr_info("ponvirt: ONU %s on %s, WAN gpon0\n", sn, wire);
	return 0;
}
late_initcall(ponvirt_init);
MODULE_DESCRIPTION("Virtual PON MAC: the GPON ONU core over an Ethernet wire");
MODULE_LICENSE("GPL");
