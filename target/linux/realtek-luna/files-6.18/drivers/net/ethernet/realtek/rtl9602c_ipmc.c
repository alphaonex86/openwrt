// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/* RTL9602C routed IPv4 multicast in the switch: WAN -> the wired LAN ports; and the WAN's
 * multicast filter: the fibre's multicast nobody joined is dropped in the switch.
 *
 * ipmr's routes (the standard fib notifier) say which groups the router forwards
 * from the WAN to the LAN bridge; the bridge MDB (IGMP snooping) says which of them
 * a port outside the switch -- WiFi -- joined, and those stay on the CPU, the only
 * path to that port. The switch routes the rest: an L3 group entry in the L2 table
 * whose LAN ports rewrite through IPMC_TRAN[L34_NETIF_LAN] (the LAN interface's
 * MAC, untagged). Measured on the X111W 2026-11-01: 1900/1900 pps delivered, CPU
 * 65 % -> 12 %. Every other group or address the host joined on the WAN or a VLAN of it
 * (igmpproxy's upstream joins, ff02::1, a solicited-node address; ndo_set_rx_mode says when),
 * and every routed group the switch does not route, has an entry toward the CPU; the rest of
 * the fibre's multicast is dropped (stock drops it too). Included by rtl9602c_eth.c after
 * rtl9602c_l34_tc.c.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#if IS_ENABLED(CONFIG_IP_MROUTE) && IS_ENABLED(CONFIG_BRIDGE_IGMP_SNOOPING) && \
	IS_ENABLED(CONFIG_NET_SWITCHDEV)
#include <linux/if_bridge.h>
#include <linux/mroute.h>
#include <linux/mroute_base.h>
#include <net/arp.h>
#include <net/fib_notifier.h>
#include <net/route.h>
#include <net/switchdev.h>

#define IPMC_TRAN(i)		(0x2A0C0 + 4 * (i))
#define LUT_CFG_IPMC_DIP_HASH	BIT(23)	/* IPv4 groups keyed by DIP + FID/VID, not by MAC */
#define SW_TBL_VLAN_RD(vid)	(SW_TBL_VLAN_WR(vid) & ~L2_CMD_WR)

static void ipmc_sg_note(struct rtl9602c_ipmc *m, const struct mfc_cache *c, bool del)
{
	const struct mr_mfc *r = &c->_c;
	struct ipmc_sg s = { .src = ntohl(c->mfc_origin), .grp = ntohl(c->mfc_mcastgrp),
			     .parent = r->mfc_parent };
	unsigned int i, v;

	for (v = r->mfc_un.res.minvif; v < r->mfc_un.res.maxvif && v < IPMC_VIFS; v++)
		if (r->mfc_un.res.ttls[v] < 255)
			s.oifs |= BIT(v);
	for (i = 0; i < m->n_sg; i++)
		if (m->sg[i].src == s.src && m->sg[i].grp == s.grp)
			break;
	if (del) {
		if (i < m->n_sg)
			m->sg[i] = m->sg[--m->n_sg];
		if (!m->n_sg)
			m->lost = false;
		return;
	}
	if (i == m->n_sg) {
		if (m->n_sg == IPMC_GROUPS) {
			m->lost = true;	/* an untracked sibling could forbid a group */
			return;
		}
		m->n_sg++;
	}
	m->sg[i] = s;
}

/* Atomic context: copy, then let the work program the switch. */
static int rtl9602c_ipmc_fib_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct rtl9602c_ipmc *m = container_of(nb, struct rtl9602c_ipmc, fib_nb);
	struct vif_entry_notifier_info *vif = ptr;
	struct mfc_entry_notifier_info *mfc = ptr;

	if (vif->info.family != RTNL_FAMILY_IPMR)
		return NOTIFY_DONE;
	spin_lock_bh(&m->lock);
	switch (event) {
	case FIB_EVENT_VIF_ADD:
	case FIB_EVENT_VIF_DEL:
		m->n_vif += event == FIB_EVENT_VIF_ADD ? 1 : -1;
		if (vif->tb_id == RT_TABLE_DEFAULT && vif->vif_index < IPMC_VIFS)
			m->vif_ifindex[vif->vif_index] =
				event == FIB_EVENT_VIF_ADD ? vif->dev->ifindex : 0;
		break;
	case FIB_EVENT_ENTRY_ADD:
	case FIB_EVENT_ENTRY_REPLACE:
	case FIB_EVENT_ENTRY_DEL:
		if (mfc->tb_id == RT_TABLE_DEFAULT)
			ipmc_sg_note(m, (const struct mfc_cache *)mfc->mfc,
				     event == FIB_EVENT_ENTRY_DEL);
		break;
	default:
		spin_unlock_bh(&m->lock);
		return NOTIFY_DONE;
	}
	spin_unlock_bh(&m->lock);
	mod_delayed_work(system_wq, &m->sync, 0);
	return NOTIFY_DONE;
}

/* A port joined or left a group: who the CPU must still serve may have changed. */
static int rtl9602c_ipmc_mdb_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct rtl9602c_ipmc *m = container_of(nb, struct rtl9602c_ipmc, mdb_nb);
	struct switchdev_notifier_port_obj_info *info = ptr;

	if ((event == SWITCHDEV_PORT_OBJ_ADD || event == SWITCHDEV_PORT_OBJ_DEL) &&
	    info->obj->id == SWITCHDEV_OBJ_ID_PORT_MDB)
		mod_delayed_work(system_wq, &m->sync, 0);
	return NOTIFY_DONE;
}

static bool ipmc_src_via_wan(struct rtl9602c_eth *ep, u32 src)
{
	struct flowi4 fl4 = { .daddr = htonl(src) };
	struct net_device *dev;
	struct rtable *rt;
	bool wan;

	rt = ip_route_output_key(dev_net(ep->ndev), &fl4);
	if (IS_ERR(rt))
		return false;
	dev = rt->dst.dev;
	wan = dev == ep->wan_ndev || (is_vlan_dev(dev) && vlan_dev_real_dev(dev) == ep->wan_ndev);
	ip_rt_put(rt);
	return wan;
}

/* Nothing else ARPs for an on-link source (an IPTV server in the WAN subnet): ask, so
 * the retry finds the next hop the interface tables need. */
static void ipmc_kick_next_hop(struct rtl9602c_eth *ep, u32 src)
{
	struct flowi4 fl4 = { .daddr = htonl(src) };
	struct neighbour *n;
	struct rtable *rt;
	__be32 nh;

	rt = ip_route_output_key(dev_net(ep->ndev), &fl4);
	if (IS_ERR(rt))
		return;
	nh = rt_nexthop(rt, fl4.daddr);
	n = __neigh_lookup(&arp_tbl, &nh, rt->dst.dev, 1);
	if (!IS_ERR_OR_NULL(n)) {
		neigh_event_send(n, NULL);
		neigh_release(n);
	}
	ip_rt_put(rt);
}

/* -> the (S,G) count copied to m->snap, with every VIF classified and the groups a
 * non-switch bridge port joined in m->keep_cpu; 0 and m->why when none may route. */
static unsigned int ipmc_snapshot(struct rtl9602c_eth *ep, struct rtl9602c_ipmc *m)
{
	int ifindex[IPMC_VIFS];
	struct net_device *br, *dev;
	struct br_ip_list *e, *tmp;
	LIST_HEAD(joined);
	unsigned int v, n_sg = 0;

	m->n_keep = 0;
	rtnl_lock();
	m->cpu.n = 0;
	if (ep->wan_ndev)
		eth_mc_wan_joins(ep->wan_ndev, &m->cpu);
	br = netdev_master_upper_dev_get(ep->ndev);
	spin_lock_bh(&m->lock);
	m->sg_lost = m->lost;
	n_sg = m->n_snap = m->lost ? 0 : m->n_sg;
	memcpy(m->snap, m->sg, n_sg * sizeof(*m->sg));
	memcpy(ifindex, m->vif_ifindex, sizeof(ifindex));
	spin_unlock_bh(&m->lock);

	for (v = 0; v < IPMC_VIFS; v++) {
		dev = ifindex[v] ? __dev_get_by_index(dev_net(ep->ndev), ifindex[v]) : NULL;
		m->vif_vid[v] = 0;
		if (!dev) {
			m->vif_kind[v] = IPMC_VIF_NONE;
		} else if (dev == br) {
			m->vif_kind[v] = IPMC_VIF_LAN;
		} else if (dev == ep->wan_ndev ||
			   (is_vlan_dev(dev) && vlan_dev_real_dev(dev) == ep->wan_ndev)) {
			m->vif_kind[v] = IPMC_VIF_WAN;
			m->vif_vid[v] = is_vlan_dev(dev) ? vlan_dev_vlan_id(dev) : 0;
		} else {
			m->vif_kind[v] = IPMC_VIF_OTHER;
		}
	}
	if (m->sg_lost) {
		m->why = "more (S,G) routes than tracked";
		goto out;
	}
	if (!n_sg)
		m->why = "no multicast route";
	if (!br || !netif_is_bridge_master(br) || !br_multicast_enabled(br)) {
		m->why = "the LAN port is in no bridge running IGMP snooping";
		n_sg = 0;
		goto out;
	}
	for (v = 0; v < n_sg; v++)
		m->snap[v].via_wan = ipmc_src_via_wan(ep, m->snap[v].src);
	br_multicast_list_adjacent(ep->ndev, &joined);
	list_for_each_entry_safe(e, tmp, &joined, list) {
		if (e->addr.proto == htons(ETH_P_IP)) {
			if (m->n_keep < IPMC_GROUPS)
				m->keep_cpu[m->n_keep++] = ntohl(e->addr.dst.ip4);
			else
				n_sg = 0;	/* a WiFi group not tracked: route nothing */
		}
		list_del(&e->list);
		kfree(e);
	}
	if (!n_sg && m->n_keep >= IPMC_GROUPS)
		m->why = "more groups joined beyond the switch than tracked";
out:
	rtnl_unlock();
	return n_sg;
}

/* The LAN interface row the TRAN entry rewrites through, programmed as the first
 * NAT flow would program it when no flow did yet. */
static int ipmc_provision(struct rtl9602c_eth *ep, u32 src)
{
	struct gpon_edge edge;
	const char *why = NULL;
	int ret = 0;

	mutex_lock(&rtl9602c_l34_tc_mutex);
	if (!ep->l34.provisioned) {
		ret = gpon_edge_read(ep->ndev, ep->wan_ndev, src, &edge, &why);
		if (!ret) {
			ret = rtl9602c_l34_provision(&ep->l34, &edge);
			why = "the interface tables would not program";
		}
	}
	mutex_unlock(&rtl9602c_l34_tc_mutex);
	if (ret)
		ep->l34.ipmc.why = why;
	return ret;
}

static int sw_vlan_read(struct rtl9602c_l34 *l, u16 vid, u32 *entry)
{
	int ret = l2_busy_wait(l);

	if (ret)
		return ret;
	l34_wr(l, L2_CMD, SW_TBL_VLAN_RD(vid));
	ret = l2_busy_wait(l);
	if (!ret)
		*entry = l34_rd(l, L2_RDATA);
	return ret;
}

/* The same LUT key: an IPv4 group by its address, an L2 entry by its MAC, both in the same VLAN key. */
static bool mc_same_key(const u32 *a, const u32 *b)
{
	u32 l3 = l34_field_get(a, L3MC_L3LOOKUP_LSP, 1);

	if (l3 != l34_field_get(b, L3MC_L3LOOKUP_LSP, 1) ||
	    l34_field_get(a, L3MC_KEY_LSP, L3MC_KEY_W) != l34_field_get(b, L3MC_KEY_LSP, L3MC_KEY_W))
		return false;
	if (l3)
		return l34_field_get(a, L3MC_GIP_LSP, L3MC_GIP_W) == l34_field_get(b, L3MC_GIP_LSP, L3MC_GIP_W);
	return a[0] == b[0] && l34_field_get(a, 32, 16) == l34_field_get(b, 32, 16);
}

static bool mc_to_cpu(const u32 *w)
{
	return !l34_field_get(w, L3MC_WANSA_LSP, 1);
}

/* Clear VALID only and rewrite by hash, so the write finds this entry's own way;
 * the read by address then proves it gone. */
static int ipmc_lut_del(struct rtl9602c_l34 *l, const struct eth_mc_entry *h)
{
	u32 w[3];
	int ret;

	memcpy(w, h->w, sizeof(w));
	l34_field_set(w, L3MC_VALID_LSP, 1, 0);
	ret = l2uc_access(l, 0, w, false, true);
	if (ret < 0)
		return ret;
	ret = l2uc_access(l, h->idx, w, false, false);
	if (ret == -ENOENT)
		return 0;
	if (ret < 0)
		return ret;
	return l34_field_get(w, L3MC_VALID_LSP, 1) && mc_same_key(w, h->w) ? -EIO : 0;
}

static int ipmc_lut_add(struct rtl9602c_l34 *l, struct eth_mc_entry *h)
{
	u32 rd[3];
	int ret;

	ret = l2uc_access(l, 0, h->w, false, true);
	if (ret < 0)
		return ret == -ENOENT ? -ENOSPC : ret;
	h->idx = ret;
	ret = l2uc_access(l, h->idx, rd, false, false);
	if (ret < 0)
		return ret;
	if (memcmp(rd, h->w, sizeof(rd))) {
		pr_warn("rtl9602c-ipmc: %pI4h at %u reads %08x %08x %08x, written %08x %08x %08x\n",
			&h->grp, h->idx, rd[0], rd[1], rd[2], h->w[0], h->w[1], h->w[2]);
		return -EIO;
	}
	return 0;
}

/* eth_mc_reconcile's table access: the log and counters of one entry. */
static int ipmc_del_cb(void *ctx, const struct eth_mc_entry *e)
{
	struct rtl9602c_eth *ep = ctx;
	struct rtl9602c_ipmc *m = &ep->l34.ipmc;
	int ret = ipmc_lut_del(&ep->l34, e);

	if (ret) {
		m->failures++;
		pr_warn_ratelimited("rtl9602c-ipmc: %pI4h at %u would not clear (%d); still in the switch\n",
				    &e->grp, e->idx, ret);
		return ret;
	}
	m->removals++;
	if (!mc_to_cpu(e->w))
		pr_info("rtl9602c-ipmc: %pI4h back to the CPU (LUT %u)\n", &e->grp, e->idx);
	return 0;
}

static int ipmc_add_cb(void *ctx, struct eth_mc_entry *e)
{
	struct rtl9602c_eth *ep = ctx;
	struct rtl9602c_ipmc *m = &ep->l34.ipmc;
	int ret = ipmc_lut_add(&ep->l34, e);

	if (ret == -EIO && ipmc_lut_del(&ep->l34, e))
		ret = 0;	/* written and not clearable: track it so a later sync retries */
	if (ret) {
		m->failures++;
		pr_warn_ratelimited("rtl9602c-ipmc: %pI4h %08x %08x not installed (%d): the CPU gets the fibre's multicast\n",
				    &e->grp, e->w[0], e->w[1], ret);
		return ret;
	}
	m->installs++;
	if (!mc_to_cpu(e->w))
		pr_info("rtl9602c-ipmc: %pI4h routed by the switch (LUT %u)\n", &e->grp, e->idx);
	return 0;
}

/* Encode next[*n]: the routed group `s` toward the LAN ports, else the join `k` toward the CPU.
 * -> false when its VLAN cannot be read. */
static bool ipmc_encode(struct rtl9602c_eth *ep, struct rtl9602c_ipmc *m, unsigned int n,
			const struct ipmc_sg *s, const struct eth_mc_key *k)
{
	u16 vid = s ? (s->parent < IPMC_VIFS ? m->vif_vid[s->parent] : 0) : k->vid, key;
	struct eth_mc_entry *e = &m->next[n];
	u8 lan = BIT(ep->swm->n_copper) - 1;
	u32 vlan;
	bool ivl;

	vid = vid ?: SW_WAN_PVID;
	if (sw_vlan_read(&ep->l34, vid, &vlan)) {
		m->failures++;
		pr_warn_ratelimited("rtl9602c-ipmc: VLAN %u unreadable: its group has no entry\n", vid);
		return false;
	}
	key = l34_vlan_lut_key(vlan, vid, &ivl);
	memset(e, 0, sizeof(*e));
	e->grp = s ? s->grp : k->grp;
	if (s)
		l34_l3mc_encode(e->w, s->grp, key, ivl, lan, L34_NETIF_LAN);
	else if (k->grp)
		l34_l3mc_cpu_encode(e->w, k->grp, key, ivl, ep->swm->cpu_port, lan);
	else
		l34_l2mc_cpu_encode(e->w, k->mac, key, ivl, ep->swm->cpu_port, lan);
	return true;
}

/* Under l34.lock: the LUT holds exactly the n_want routed groups of m->want, and an entry toward
 * the CPU for every other group the host joined on the WAN or ipmr routes from it.
 * -> false when one of them is not in the LUT: the fibre's multicast must then reach the CPU. */
static bool ipmc_apply(struct rtl9602c_eth *ep, struct rtl9602c_ipmc *m, unsigned int n_want)
{
	struct rtl9602c_l34 *l = &ep->l34;
	u8 lan = BIT(ep->swm->n_copper) - 1;
	unsigned int i, n = 0;
	bool all = true;

	if (n_want)
		l34_wr(l, IPMC_TRAN(L34_NETIF_LAN), l34_ipmc_tran_encode(lan, L34_NETIF_LAN));
	l34_wr(l, SW_LUT_CFG, l34_rd(l, SW_LUT_CFG) | LUT_CFG_IPMC_DIP_HASH);
	for (i = 0; i < n_want; i++)
		if (ipmc_encode(ep, m, n, &m->snap[m->want[i]], NULL))
			n++;
	/* a group ipmr routes from the WAN, not by the switch: the CPU needs its frames */
	for (i = 0; i < m->n_snap; i++)
		if (m->snap[i].parent < IPMC_VIFS && m->vif_kind[m->snap[i].parent] == IPMC_VIF_WAN)
			eth_mc_key_add(&m->cpu, m->snap[i].grp, NULL, m->vif_vid[m->snap[i].parent]);
	for (i = 0; i < m->cpu.n && n < MC_ENTRIES; i++)
		if (ipmc_encode(ep, m, n, NULL, &m->cpu.k[i]))
			eth_mc_next_keep(m->next, &n, mc_same_key);
		else
			all = false;
	all &= i == m->cpu.n && !m->cpu.lost;
	all &= !eth_mc_reconcile(m->hw, &m->n_hw, MC_ENTRIES, m->next, n, ipmc_del_cb, ipmc_add_cb, ep);
	for (i = m->n_cpu_hw = 0; i < m->n_hw; i++)
		m->n_cpu_hw += mc_to_cpu(m->hw[i].w);
	return all;
}

/* ndo_set_rx_mode of the WAN, atomic: the host joined or left a group there. */
static void rtl9602c_ipmc_kick(struct rtl9602c_eth *ep)
{
	struct rtl9602c_ipmc *m = &ep->l34.ipmc;

	if (READ_ONCE(m->watching))
		mod_delayed_work(system_wq, &m->sync, 0);
}

static void rtl9602c_ipmc_sync(struct work_struct *work)
{
	struct rtl9602c_ipmc *m = container_of(to_delayed_work(work), struct rtl9602c_ipmc, sync);
	struct rtl9602c_eth *ep = container_of(m, struct rtl9602c_eth, l34.ipmc);
	unsigned int n_sg, n_want = 0;
	bool all;
	int ret;

	if (!READ_ONCE(m->watching))
		return;
	n_sg = ipmc_snapshot(ep, m);
	if (n_sg) {
		n_want = l34_ipmc_wanted(m->snap, n_sg, m->vif_kind, m->keep_cpu, m->n_keep,
					 m->want, IPMC_GROUPS);
		m->why = n_want ? NULL : "no route goes from the WAN to the LAN bridge alone";
	}
	if (n_want) {
		ret = ipmc_provision(ep, m->snap[m->want[0]].src);
		if (ret == -EAGAIN) {	/* next hop or WAN address not there yet: the stream must not wait for another event */
			ipmc_kick_next_hop(ep, m->snap[m->want[0]].src);
			mod_delayed_work(system_wq, &m->sync, HZ / 10);	/* ARP answers in ms; each 1 s of wait is 1 s of a stream on the CPU */
		}
		if (ret)
			n_want = 0;
	}
	mutex_lock(&ep->l34.lock);
	all = ipmc_apply(ep, m, n_want);
	m->n_want = n_want;
	mutex_unlock(&ep->l34.lock);
	if (ep->wan_ndev) {
		WRITE_ONCE(m->promiscuity, READ_ONCE(ep->wan_ndev->promiscuity));
		WRITE_ONCE(m->capturers, eth_mc_capturers(m->promiscuity, READ_ONCE(ep->wan_ndev->uc_promisc)));
	}
	WRITE_ONCE(m->overflow, !all || m->sg_lost);
	rtl9602c_pon_mc_policy(ep);
}

static void rtl9602c_ipmc_stop(void *data)
{
	struct rtl9602c_eth *ep = data;
	struct rtl9602c_ipmc *m = &ep->l34.ipmc;

	WRITE_ONCE(m->watching, false);
	unregister_fib_notifier(&init_net, &m->fib_nb);
	unregister_switchdev_blocking_notifier(&m->mdb_nb);
	cancel_delayed_work_sync(&m->sync);
	rtl9602c_pon_mc_policy(ep);
	m->n_snap = m->cpu.n = 0;
	mutex_lock(&ep->l34.lock);
	ipmc_apply(ep, m, 0);
	mutex_unlock(&ep->l34.lock);
	if (m->n_hw)
		pr_warn("rtl9602c-ipmc: %u group(s) left in the switch at stop\n", m->n_hw);
}

static int rtl9602c_ipmc_start(struct rtl9602c_eth *ep)
{
	struct rtl9602c_ipmc *m = &ep->l34.ipmc;
	int ret;

	spin_lock_init(&m->lock);
	INIT_DELAYED_WORK(&m->sync, rtl9602c_ipmc_sync);
	m->cpu.k = m->cpu_key;
	m->cpu.max = MC_CPU_KEYS;
	m->why = "no multicast route yet";
	m->mdb_nb.notifier_call = rtl9602c_ipmc_mdb_event;
	ret = register_switchdev_blocking_notifier(&m->mdb_nb);
	if (ret)
		return ret;
	m->fib_nb.notifier_call = rtl9602c_ipmc_fib_event;
	ret = register_fib_notifier(&init_net, &m->fib_nb, NULL, NULL);
	if (ret) {
		unregister_switchdev_blocking_notifier(&m->mdb_nb);
		return ret;
	}
	WRITE_ONCE(m->watching, true);
	mod_delayed_work(system_wq, &m->sync, 0);	/* the joins made before this moment */
	return 0;
}
#else
static void rtl9602c_ipmc_kick(struct rtl9602c_eth *ep)
{
}

static void rtl9602c_ipmc_stop(void *data)
{
}

static int rtl9602c_ipmc_start(struct rtl9602c_eth *ep)
{
	ep->l34.ipmc.why = "built without IP_MROUTE, BRIDGE_IGMP_SNOOPING or NET_SWITCHDEV";
	return -EOPNOTSUPP;
}
#endif
