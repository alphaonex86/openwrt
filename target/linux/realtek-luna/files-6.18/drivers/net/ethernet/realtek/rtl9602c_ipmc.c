// SPDX-License-Identifier: GPL-2.0-only
/* RTL9602C routed IPv4 multicast in the switch: WAN -> the wired LAN ports.
 *
 * ipmr's routes (the standard fib notifier) say which groups the router forwards
 * from the WAN to the LAN bridge; the bridge MDB (IGMP snooping) says which of them
 * a port outside the switch -- WiFi -- joined, and those stay on the CPU, the only
 * path to that port. The switch routes the rest: an L3 group entry in the L2 table
 * whose LAN ports rewrite through IPMC_TRAN[L34_NETIF_LAN] (the LAN interface's
 * MAC, untagged). Measured on the X111W 2026-11-01: 1900/1900 pps delivered, CPU
 * 65 % -> 12 %. Included by rtl9602c_eth.c after rtl9602c_l34_tc.c.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#if IS_ENABLED(CONFIG_IP_MROUTE) && IS_ENABLED(CONFIG_BRIDGE_IGMP_SNOOPING) && \
	IS_ENABLED(CONFIG_NET_SWITCHDEV)
#include <linux/if_bridge.h>
#include <linux/mroute.h>
#include <linux/mroute_base.h>
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
	schedule_work(&m->sync);
	return NOTIFY_DONE;
}

/* A port joined or left a group: who the CPU must still serve may have changed. */
static int rtl9602c_ipmc_mdb_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct rtl9602c_ipmc *m = container_of(nb, struct rtl9602c_ipmc, mdb_nb);
	struct switchdev_notifier_port_obj_info *info = ptr;

	if ((event == SWITCHDEV_PORT_OBJ_ADD || event == SWITCHDEV_PORT_OBJ_DEL) &&
	    info->obj->id == SWITCHDEV_OBJ_ID_PORT_MDB)
		schedule_work(&m->sync);
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
	br = netdev_master_upper_dev_get(ep->ndev);
	if (!br || !netif_is_bridge_master(br) || !br_multicast_enabled(br)) {
		m->why = "the LAN port is in no bridge running IGMP snooping";
		goto out;
	}
	spin_lock_bh(&m->lock);
	if (m->lost) {
		spin_unlock_bh(&m->lock);
		m->why = "more (S,G) routes than tracked";
		goto out;
	}
	n_sg = m->n_sg;
	memcpy(m->snap, m->sg, n_sg * sizeof(*m->sg));
	if (!n_sg)
		m->why = "no multicast route";
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

/* Clear VALID only and rewrite by hash, so the write finds this entry's own way;
 * the read by address then proves it gone. */
static int ipmc_lut_del(struct rtl9602c_l34 *l, const struct ipmc_hw *h)
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
	return l34_field_get(w, L3MC_VALID_LSP, 1) &&
	       l34_field_get(w, L3MC_GIP_LSP, L3MC_GIP_W) ==
	       l34_field_get(h->w, L3MC_GIP_LSP, L3MC_GIP_W) ? -EIO : 0;
}

static int ipmc_lut_add(struct rtl9602c_l34 *l, struct ipmc_hw *h)
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

/* Under l34.lock: the LUT holds exactly the n_want groups of m->want. */
static void ipmc_apply(struct rtl9602c_eth *ep, struct rtl9602c_ipmc *m, unsigned int n_want)
{
	struct rtl9602c_l34 *l = &ep->l34;
	u8 lan = BIT(ep->swm->n_copper) - 1;
	unsigned int i, j;
	struct ipmc_hw want;
	u32 vlan;
	u16 vid, key;
	bool ivl;
	int ret;

	if (n_want) {
		l34_wr(l, IPMC_TRAN(L34_NETIF_LAN), l34_ipmc_tran_encode(lan, L34_NETIF_LAN));
		l34_wr(l, SW_LUT_CFG, l34_rd(l, SW_LUT_CFG) | LUT_CFG_IPMC_DIP_HASH);
	}
	for (j = 0; j < n_want; j++) {
		const struct ipmc_sg *s = &m->snap[m->want[j]];

		vid = m->vif_vid[s->parent] ?: SW_WAN_PVID;
		ret = sw_vlan_read(l, vid, &vlan);
		if (ret) {
			m->failures++;
			pr_warn_ratelimited("rtl9602c-ipmc: VLAN %u unreadable (%d): %pI4h stays on the CPU\n",
					    vid, ret, &s->grp);
			m->want[j--] = m->want[--n_want];
			continue;
		}
		key = l34_vlan_lut_key(vlan, vid, &ivl);
		l34_l3mc_encode(m->hw_next[j], s->grp, key, ivl, lan, L34_NETIF_LAN);
	}
	/* retire what is no longer wanted, or wanted with other words */
	for (i = 0; i < m->n_hw; ) {
		for (j = 0; j < n_want && memcmp(m->hw_next[j], m->hw[i].w, sizeof(m->hw[i].w)); j++)
			;
		if (j < n_want) {
			i++;
			continue;
		}
		ret = ipmc_lut_del(l, &m->hw[i]);
		if (ret) {
			m->failures++;
			pr_warn_ratelimited("rtl9602c-ipmc: %pI4h at %u would not clear (%d); still routed\n",
					    &m->hw[i].grp, m->hw[i].idx, ret);
			i++;
			continue;
		}
		m->removals++;
		pr_info("rtl9602c-ipmc: %pI4h back to the CPU (LUT %u)\n", &m->hw[i].grp, m->hw[i].idx);
		m->hw[i] = m->hw[--m->n_hw];
	}
	for (j = 0; j < n_want && m->n_hw < IPMC_GROUPS; j++) {
		for (i = 0; i < m->n_hw && memcmp(m->hw_next[j], m->hw[i].w, sizeof(m->hw[i].w)); i++)
			;
		if (i < m->n_hw)
			continue;
		want.grp = m->snap[m->want[j]].grp;
		memcpy(want.w, m->hw_next[j], sizeof(want.w));
		ret = ipmc_lut_add(l, &want);
		if (ret == -EIO && ipmc_lut_del(l, &want))
			ret = 0;	/* written and not clearable: track it so a later sync retries */
		if (ret) {
			m->failures++;
			pr_warn_ratelimited("rtl9602c-ipmc: %pI4h not installed (%d); the CPU routes it\n",
					    &want.grp, ret);
			continue;
		}
		m->hw[m->n_hw++] = want;
		m->installs++;
		pr_info("rtl9602c-ipmc: %pI4h routed by the switch (LUT %u)\n", &want.grp, want.idx);
	}
}

static void rtl9602c_ipmc_sync(struct work_struct *work)
{
	struct rtl9602c_ipmc *m = container_of(work, struct rtl9602c_ipmc, sync);
	struct rtl9602c_eth *ep = container_of(m, struct rtl9602c_eth, l34.ipmc);
	unsigned int n_sg, n_want = 0;

	n_sg = ipmc_snapshot(ep, m);
	if (n_sg) {
		n_want = l34_ipmc_wanted(m->snap, n_sg, m->vif_kind, m->keep_cpu, m->n_keep,
					 m->want, IPMC_GROUPS);
		m->why = n_want ? NULL : "no route goes from the WAN to the LAN bridge alone";
	}
	if (n_want && ipmc_provision(ep, m->snap[m->want[0]].src))
		n_want = 0;
	mutex_lock(&ep->l34.lock);
	ipmc_apply(ep, m, n_want);
	m->n_want = n_want;
	mutex_unlock(&ep->l34.lock);
}

static void rtl9602c_ipmc_stop(void *data)
{
	struct rtl9602c_eth *ep = data;
	struct rtl9602c_ipmc *m = &ep->l34.ipmc;

	unregister_fib_notifier(&init_net, &m->fib_nb);
	unregister_switchdev_blocking_notifier(&m->mdb_nb);
	cancel_work_sync(&m->sync);
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
	INIT_WORK(&m->sync, rtl9602c_ipmc_sync);
	m->why = "no multicast route yet";
	m->mdb_nb.notifier_call = rtl9602c_ipmc_mdb_event;
	ret = register_switchdev_blocking_notifier(&m->mdb_nb);
	if (ret)
		return ret;
	m->fib_nb.notifier_call = rtl9602c_ipmc_fib_event;
	ret = register_fib_notifier(&init_net, &m->fib_nb, NULL, NULL);
	if (ret)
		unregister_switchdev_blocking_notifier(&m->mdb_nb);
	return ret;
}
#else
static void rtl9602c_ipmc_stop(void *data)
{
}

static int rtl9602c_ipmc_start(struct rtl9602c_eth *ep)
{
	ep->l34.ipmc.why = "built without IP_MROUTE, BRIDGE_IGMP_SNOOPING or NET_SWITCHDEV";
	return -EOPNOTSUPP;
}
#endif
