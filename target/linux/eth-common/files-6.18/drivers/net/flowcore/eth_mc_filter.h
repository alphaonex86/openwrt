/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_mc_filter.h -- the WAN port's multicast filter, on any switch: the groups and addresses the host
 * joined on its WAN netdev and every VLAN of it, a table of switch entries kept equal to a wanted list,
 * and the action for the fibre's multicast that no entry holds. Stock drops that multicast in the switch.
 * A chip keeps only its entry words and its table access.
 */
#ifndef _ETH_MC_FILTER_H
#define _ETH_MC_FILTER_H

#include <linux/types.h>

#define ETH_MC_WORDS	4

struct eth_mc_key {
	u32	grp;		/* an IPv4 group, else 0 and `mac` (IPv6, L2) */
	u8	mac[6];
	u16	vid;		/* the VLAN it arrives in: 0 = untagged */
};

struct eth_mc_keys {
	struct eth_mc_key	*k;
	unsigned int		n, max;
	bool			lost;	/* a key did not fit */
};

struct eth_mc_entry {
	u32	w[ETH_MC_WORDS];	/* the chip's entry words, unused ones zero */
	u32	grp;			/* for the log */
	u16	idx;			/* where the switch put it */
};

static inline void eth_mc_key_add(struct eth_mc_keys *s, u32 grp, const u8 *mac, u16 vid)
{
	struct eth_mc_key k = { .grp = grp, .vid = vid };
	unsigned int i;

	if (mac)
		__builtin_memcpy(k.mac, mac, sizeof(k.mac));
	for (i = 0; i < s->n; i++)
		if (!__builtin_memcmp(&s->k[i], &k, sizeof(k)))
			return;
	if (s->n == s->max)
		s->lost = true;
	else
		s->k[s->n++] = k;
}

/* Who wants every frame of the WAN: its promiscuity holders (a capture, a bridge port), less the one a
 * secondary unicast address takes on a device with no unicast filter (a VLAN or macvlan upper with its own
 * MAC): that one asks for an address, never for every multicast group. */
static inline unsigned int eth_mc_capturers(unsigned int promiscuity, bool uc_promisc)
{
	return promiscuity - (uc_promisc && promiscuity);
}

/* The miss action: dropped only while every join has its entry and nothing captures the WAN. */
static inline bool eth_mc_miss_drop(bool watching, bool complete, unsigned int capturers)
{
	return watching && complete && !capturers;
}

/* next[*n] was just encoded: keep it unless an earlier entry has its key. */
static inline void eth_mc_next_keep(struct eth_mc_entry *next, unsigned int *n,
				    bool (*same_key)(const u32 *a, const u32 *b))
{
	unsigned int j;

	for (j = 0; j < *n && !same_key(next[j].w, next[*n].w); j++)
		;
	*n += j == *n;
}

/* The table holds exactly next[0..n_next): retire the rest, then install what is missing. A retire that
 * fails stays tracked; `add` refuses with nonzero. -> the entries of next[] not in the table. */
static inline unsigned int eth_mc_reconcile(struct eth_mc_entry *hw, unsigned int *n_hw, unsigned int max,
					    const struct eth_mc_entry *next, unsigned int n_next,
					    int (*del)(void *ctx, const struct eth_mc_entry *e),
					    int (*add)(void *ctx, struct eth_mc_entry *e), void *ctx)
{
	unsigned int i, j, missing = 0;
	struct eth_mc_entry e;

	for (i = 0; i < *n_hw; ) {
		for (j = 0; j < n_next && __builtin_memcmp(next[j].w, hw[i].w, sizeof(hw[i].w)); j++)
			;
		if (j < n_next || del(ctx, &hw[i]))
			i++;
		else
			hw[i] = hw[--*n_hw];
	}
	for (j = 0; j < n_next; j++) {
		for (i = 0; i < *n_hw && __builtin_memcmp(next[j].w, hw[i].w, sizeof(hw[i].w)); i++)
			;
		if (i < *n_hw)
			continue;
		e = next[j];
		if (*n_hw == max || add(ctx, &e))
			missing++;
		else
			hw[(*n_hw)++] = e;
	}
	return missing;
}

#ifdef __KERNEL__
#include <linux/if_vlan.h>
#include <linux/igmp.h>
#include <linux/inetdevice.h>
#include <linux/netdevice.h>

/* Under RTNL: what the host joined on `wan` and every VLAN of it. An IPv4 group is kept by its address
 * (a switch looks it up by group), so its 01:00:5e address is not kept twice. */
static inline void eth_mc_wan_joins(struct net_device *wan, struct eth_mc_keys *s)
{
	struct netdev_hw_addr *ha;
	struct net_device *dev;
	struct ip_mc_list *im;
	struct in_device *in;
	u16 vid;

	s->n = 0;
	s->lost = false;
	for_each_netdev(dev_net(wan), dev) {
		if (dev != wan && !(is_vlan_dev(dev) && vlan_dev_real_dev(dev) == wan))
			continue;
		vid = is_vlan_dev(dev) ? vlan_dev_vlan_id(dev) : 0;
		in = __in_dev_get_rtnl(dev);
		for (im = in ? rtnl_dereference(in->mc_list) : NULL; im; im = rtnl_dereference(im->next_rcu))
			eth_mc_key_add(s, ntohl(im->multiaddr), NULL, vid);
		netif_addr_lock_bh(dev);
		netdev_for_each_mc_addr(ha, dev)
			if (ha->addr[0] != 0x01 || ha->addr[1] != 0x00 || ha->addr[2] != 0x5e)
				eth_mc_key_add(s, 0, ha->addr, vid);
		netif_addr_unlock_bh(dev);
	}
}
#endif

#endif
