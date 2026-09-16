// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL9602C "Luna" hardware L3/L4 forwarding + NAPT engine.
 *
 * Indirect table-access plumbing + NAPT flow programming for the SoC switch
 * core's L3/L4 accelerator. Established connections handed down by the kernel
 * flow-offload path are written into the hardware NAPT tables so they are
 * NAT'd/forwarded by the switch instead of the CPU.
 *
 * Clean-room: the register/table behaviour is hardware fact; all expression
 * (names, packing helpers, the access sequence) is original. Endian-safe: the
 * packed entry words are built with explicit shift/mask on a local u32 array,
 * never struct overlays, so the same code is correct on big-endian MIPS now and
 * little-endian ARM later.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include "rtl9602c_l34_logic.h"	/* hoisted logic */
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "luna_l34_acc.h"
#include "rtl9602c_l34.h"
#include "gpon_flow_offload.h"

#define L34_EXE_POLL_US		2000	/* engine clears EXE well under this */

static inline u32 l34_rd(struct rtl9602c_l34 *l, u32 off)
{
	return readl(l->sw + off);
}

static inline void l34_wr(struct rtl9602c_l34 *l, u32 off, u32 val)
{
	writel(val, l->sw + off);
}

/* l34_field_get() moved to rtl9602c_l34_logic.c (flowcore, ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 1. */
/* ★ THE BODY MOVED TO FAMILY TIER, 2026-09-16 (luna_l34_acc.h).  This is the ONE
 * thing this family's two L3/L4 engines share -- the NAPT model here and the
 * FLOWBASED model on the RTL9603CVD -- and it lived as `static` inside the file
 * belonging to exactly one of them, so a second die could only get it by copying.
 * Nothing the engine sees changes: this chip's four addresses and its command-word
 * layout are the same values, now expressed as a per-die TABLE. */
static int l34_tbl_op(struct rtl9602c_l34 *l, enum l34_tbl type, u16 idx,
		      u32 *w, unsigned int n, bool write)
{
	return luna_l34_tbl_op(l->sw, &luna_l34_acc_rtl9602c, (u8)type, idx,
			       w, n, write);
}

static int l34_tbl_write(struct rtl9602c_l34 *l, enum l34_tbl type, u16 idx,
			 u32 *w, unsigned int n)
{
	return l34_tbl_op(l, type, idx, w, n, true);
}

static int l34_tbl_read(struct rtl9602c_l34 *l, enum l34_tbl type, u16 idx,
			u32 *w, unsigned int n)
{
	return l34_tbl_op(l, type, idx, w, n, false);
}

/* The writer flowcore's l34_prog_run() drives. flowcore holds ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 2. */
static int l34_prog_wr(void *ctx, u8 tbl, u16 idx, const u32 *w, unsigned int n)
{
	struct rtl9602c_l34 *l = ctx;
	u32 buf[L34_WORDS_MAX];
	int ret;

	memcpy(buf, w, n * sizeof(*w));
	ret = l34_tbl_write(l, (enum l34_tbl)tbl, idx, buf, n);
	if (ret)
		pr_err("rtl9602c-l34: table %u entry %u write failed (%d)\n",
		       tbl, idx, ret);
	return ret;
}

/* Enable the L34 NAT engine. DEFERRED from init: writing the ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 3. */
static void l34_engine_on(struct rtl9602c_l34 *l)
{
	u32 v;

	l34_wr(l, L34_CLR, 0xffff);		/* reset the NAT table types we use */
	usleep_range(50, 100);

	v = l34_rd(l, L34_SWTCR0);
	v &= ~(0x3u << L34_SWTCR0_NATMODE_SH);
	v |=  (0x3u << L34_SWTCR0_NATMODE_SH);		/* L3 + L4 NAT enable */
	v &= ~(0x3u << L34_SWTCR0_LIMDBC_SH);
	v |=  (0x2u << L34_SWTCR0_LIMDBC_SH);		/* what stock also holds */
	l34_wr(l, L34_SWTCR0, v);

	l34_wr(l, L34_GLB_CFG, l34_rd(l, L34_GLB_CFG) | BIT(0));	/* master enable */

	l->engine_on = true;
}

int rtl9602c_l34_init(struct rtl9602c_l34 *l, void __iomem *sw)
{
	if (!sw)
		return -EINVAL;
	l->sw = sw;
	mutex_init(&l->lock);
	l->ready = true;	/* table-access ready; the engine is enabled lazily */
	return 0;
}

/* Scan the 4 ways of a hash bucket for the first slot whose VALID field is 0;
 * returns the entry index, a negative table error, or -ENOSPC if full. */
static int l34_free_way(struct rtl9602c_l34 *l, enum l34_tbl type, u16 bucket,
			unsigned int words, unsigned int valid_lsp,
			unsigned int valid_w)
{
	u32 probe[L34_WORDS_NAPTR_IN];	/* sized to the widest entry */
	unsigned int way;
	int ret;

	for (way = 0; way < L34_NAPT_WAYS; way++) {
		ret = l34_tbl_read(l, type, (bucket << 2) + way, probe, words);
		if (ret)
			return ret;
		if (!l34_field_get(probe, valid_lsp, valid_w))
			return (bucket << 2) + way;
	}
	return -ENOSPC;
}

/* NAPT flow programming. A 4-way hashed pair: the outbound ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 4. */
int rtl9602c_l34_flow_add(struct rtl9602c_l34 *l, struct l34_flow *f)
{
	u32 naptr[L34_WORDS_NAPTR_IN] = { 0 };
	u32 napt[L34_WORDS_NAPT_OUT] = { 0 };
	int naptr_idx, napt_idx, ret;
	u16 out_bucket, in_bucket;
	bool is_tcp;

	if (!l->ready)
		return -ENODEV;
	if (f->l4proto != IPPROTO_TCP && f->l4proto != IPPROTO_UDP)
		return -EOPNOTSUPP;
	is_tcp = (f->l4proto == IPPROTO_TCP);

	out_bucket = l34_hash_out(is_tcp, f->orig_sip, f->orig_sport,
				  f->orig_dip, f->orig_dport);
	in_bucket  = l34_hash_in(is_tcp, f->nat_sip, f->nat_sport);

	mutex_lock(&l->lock);
	if (!l->engine_on)
		l34_engine_on(l);

	naptr_idx = l34_free_way(l, L34_TBL_NAPTR_IN, in_bucket, L34_WORDS_NAPTR_IN,
				 L34_NAPTR_VALID_LSP, L34_NAPTR_VALID_W);
	if (naptr_idx < 0) {
		ret = naptr_idx;
		goto out;
	}
	napt_idx = l34_free_way(l, L34_TBL_NAPT_OUT, out_bucket, L34_WORDS_NAPT_OUT,
				L34_NAPT_VALID_LSP, L34_NAPT_VALID_W);
	if (napt_idx < 0) {
		ret = napt_idx;
		goto out;
	}

	l34_naptr_encode(naptr, f->orig_sip, f->orig_sport, f->egress_netif,
			 f->nat_sport, is_tcp);
	ret = l34_tbl_write(l, L34_TBL_NAPTR_IN, naptr_idx, naptr, L34_WORDS_NAPTR_IN);
	if (ret)
		goto out;

	l34_napt_encode(napt, naptr_idx);
	ret = l34_tbl_write(l, L34_TBL_NAPT_OUT, napt_idx, napt, L34_WORDS_NAPT_OUT);
	if (ret) {
		memset(naptr, 0, sizeof(naptr));	/* roll back the rewrite entry */
		l34_tbl_write(l, L34_TBL_NAPTR_IN, naptr_idx, naptr, L34_WORDS_NAPTR_IN);
		goto out;
	}

	f->hw_index = napt_idx;
	l->installs++;
	ret = 0;
out:
	mutex_unlock(&l->lock);
	return ret;
}

int rtl9602c_l34_flow_del(struct rtl9602c_l34 *l, struct l34_flow *f)
{
	u32 zero[L34_WORDS_NAPTR_IN] = { 0 };
	u32 napt[L34_WORDS_NAPT_OUT];
	int ret;

	if (!l->ready)
		return -ENODEV;
	mutex_lock(&l->lock);
	/* follow the outbound slot's pointer to also clear the rewrite entry */
	ret = l34_tbl_read(l, L34_TBL_NAPT_OUT, f->hw_index, napt, L34_WORDS_NAPT_OUT);
	if (!ret && l34_field_get(napt, L34_NAPT_VALID_LSP, L34_NAPT_VALID_W)) {
		u16 naptr_idx = l34_field_get(napt, L34_NAPT_HASHIN_IDX_LSP,
					      L34_NAPT_HASHIN_IDX_W);

		l34_tbl_write(l, L34_TBL_NAPT_OUT, f->hw_index, zero,
			      L34_WORDS_NAPT_OUT);
		l34_tbl_write(l, L34_TBL_NAPTR_IN, naptr_idx, zero,
			      L34_WORDS_NAPTR_IN);
		l->removals++;
	}
	mutex_unlock(&l->lock);
	return ret;
}

int rtl9602c_l34_flow_hit(struct rtl9602c_l34 *l, u16 hw_index, bool *active)
{
	u32 word;

	if (!l->ready)
		return -ENODEV;
	word = l34_rd(l, L34_NAPT_HIT + 4 * (hw_index / 32));
	*active = !!(word & BIT(hw_index % 32));
	return 0;
}

/* WAN forwarding bring-up: the per-interface state a NAPT ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 13. */
static int l2_busy_wait(struct rtl9602c_l34 *l)
{
	unsigned int t;

	for (t = 0; t < L34_EXE_POLL_US; t++) {
		if (!(l34_rd(l, L2_STS) & L2_STS_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

/* Insert a static unicast MAC by the MAC-hash method; the engine picks a free
 * way and reports the entry index (used as NEXTHOP/ARP nhIdx). */
static int l2uc_add_static(struct rtl9602c_l34 *l, const u8 *mac, u8 port)
{
	u32 w[L34_WORDS_L2UC] = { 0 };
	int ret;

	l34_l2uc_encode(w, mac, port);	/* key + flags (flowcore) */

	ret = l2_busy_wait(l);
	if (ret)
		return ret;
	l34_wr(l, L2_WDATA + 0, w[0]);
	l34_wr(l, L2_WDATA + 4, w[1]);
	l34_wr(l, L2_WDATA + 8, w[2]);
	l34_wr(l, L2_CMD, L2_CMD_WR | (L2_METHOD_MAC << L2_CMD_METHOD_SH));

	ret = l2_busy_wait(l);
	if (ret)
		return ret;
	/* the assigned-index decode is l34_l2uc_sts_index() (flowcore); the
	 * errno stays here -- flowcore includes no errno.h */
	ret = l34_l2uc_sts_index(l34_rd(l, L2_STS));
	return ret < 0 ? -ENOSPC : ret;
}

int rtl9602c_l34_wan_setup(struct rtl9602c_l34 *l, u8 idx, u32 wan_ip,
			   const u8 *wan_mac, u32 gw_ip, const u8 *gw_mac,
			   u8 wan_port, u16 vlan)
{
	struct l34_prog_step step[7] = { { 0 } };
	int l2idx, ret;

	if (!l->ready)
		return -ENODEV;
	if (idx >= L34_EXTIP_SLOTS)	/* NAPTR EXTIP_IDX is only 3 bits wide */
		return -EINVAL;

	mutex_lock(&l->lock);
	if (!l->engine_on)
		l34_engine_on(l);

	/* gateway dst MAC -> L2 unicast table; capture the assigned index. It
	 * is an ALLOCATION, so it precedes the program rather than joining it. */
	l2idx = l2uc_add_static(l, gw_mac, wan_port);
	if (l2idx < 0) {
		ret = l2idx;
		goto out;
	}

	/* ★ STEP 0 IS THE CLAIM -- see struct l34_prog_step (flowcore): a NETIF
	 * entry takes traffic the instant it is written, so it is what a later
	 * failure must revoke. NETIF[idx]: egress source MAC + VLAN/MTU/IP. */
	step[0].tbl = L34_TBL_NETIF;
	step[0].idx = idx;
	step[0].words = L34_WORDS_NETIF;
	l34_netif_encode(step[0].w, wan_mac, wan_ip, vlan);

	/* LOCAL ROUTE[idx]: the why (vendor leaves valid=0, offload silently
	 * fails) is at l34_rt_wan_encode() in flowcore */
	step[1].tbl = L34_TBL_L3ROUTE;
	step[1].idx = idx;
	step[1].words = L34_WORDS_L3ROUTE;
	l34_rt_wan_encode(step[1].w, idx);

	/* NEXTHOP[idx]: ethernet next-hop via NETIF[idx], dst MAC = L2[l2idx] */
	step[2].tbl = L34_TBL_NEXTHOP;
	step[2].idx = idx;
	step[2].words = L34_WORDS_NEXTHOP;
	l34_nexthop_encode(step[2].w, idx, l2idx);

	/* EXTIP[idx]: the WAN source IP a NAPT rewrite applies, via NEXTHOP[idx] */
	step[3].tbl = L34_TBL_EXTIP;
	step[3].idx = idx;
	step[3].words = L34_WORDS_EXTIP;
	l34_extip_encode(step[3].w, wan_ip, idx);

	/* ARP entry: gateway IP -> the same L2 entry (placed in the WAN half). */
	step[4].tbl = L34_TBL_ARP;
	step[4].idx = L34_ARP_WAN_BASE + idx;
	step[4].words = L34_WORDS_ARP;
	l34_arp_encode(step[4].w, gw_ip, l2idx);

	/* The /32 CPU self-route for the WAN address.  The LAN side has always
	 * written one and this side never did, so l34_iface_blackholes() -- our
	 * own predicate, "the engine claims traffic for this interface and
	 * nothing terminates its OWN address locally" -- was true for the WAN
	 * netif on every boot, and /proc/flowdump printed `netif[0] blackholes
	 * 1` while nobody acted on it.  A NATed reply is addressed to this very
	 * address.  dev/MEASURED-rtl9602c_l34.c.md sec 14. */
	step[5].tbl = L34_TBL_L3ROUTE;
	step[5].idx = idx + L34_RT_CPU_SLOT_OFF;
	step[5].words = L34_WORDS_L3ROUTE;
	l34_rt_cpu_encode(step[5].w, wan_ip, idx);

	/* The UPPER half of the default route.  MASK is a prefix CODE (0 => /1),
	 * so /0 cannot be expressed and the local route above covers 0.0.0.0/1
	 * ONLY -- every destination at or above 128.0.0.0 had no LPM match, and
	 * an offloaded flow to one is dropped while software forwards it fine.
	 * Two /1 entries express what /0 cannot.  RE: stock writes one routing
	 * entry per route (rtk_l34_routingTable_set); this is the same table. */
	step[6].tbl = L34_TBL_L3ROUTE;
	step[6].idx = L34_RT_POOL_BASE + idx;
	step[6].words = L34_WORDS_L3ROUTE;
	l34_rt_wan_net_encode(step[6].w, 0x80000000u, 1, idx);

	ret = l34_prog_run(step, ARRAY_SIZE(step), l34_prog_wr, l);
out:
	mutex_unlock(&l->lock);
	return ret;
}

/* Program a LAN-side interface: a NETIF (the ONU's LAN ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 5. */
int rtl9602c_l34_lan_setup(struct rtl9602c_l34 *l, u8 idx, u32 lan_ip,
			   const u8 *lan_mac, u32 lan_net, u8 prefix, u16 vlan)
{
	struct l34_prog_step step[3] = { { 0 } };
	int ret;

	if (!l->ready)
		return -ENODEV;
	if (idx >= L34_EXTIP_SLOTS || prefix < 1 || prefix > 32)
		return -EINVAL;

	/* ★ STEP 0 IS THE CLAIM, AND ON THIS SIDE IT IS THE ONU'S OWN ...
	 * dev/MEASURED-rtl9602c_l34.c.md sec 6. */
	step[0].tbl = L34_TBL_NETIF;
	step[0].idx = idx;
	step[0].words = L34_WORDS_NETIF;
	l34_netif_encode(step[0].w, lan_mac, lan_ip, vlan);

	/* LAN subnet route (mask = prefix code: the off-by-one fact is pinned
	 * at l34_rt_lan_encode() in flowcore) */
	step[1].tbl = L34_TBL_L3ROUTE;
	step[1].idx = idx;
	step[1].words = L34_WORDS_L3ROUTE;
	l34_rt_lan_encode(step[1].w, lan_net, prefix, idx);

	/* The /32 CPU self-route. ⚠ On its own it does NOT keep ...
	 * dev/MEASURED-rtl9602c_l34.c.md sec 14. */
	step[2].tbl = L34_TBL_L3ROUTE;
	step[2].idx = idx + L34_RT_CPU_SLOT_OFF;
	step[2].words = L34_WORDS_L3ROUTE;
	l34_rt_cpu_encode(step[2].w, lan_ip, idx);

	mutex_lock(&l->lock);
	if (!l->engine_on)
		l34_engine_on(l);
	ret = l34_prog_run(step, ARRAY_SIZE(step), l34_prog_wr, l);
	mutex_unlock(&l->lock);
	return ret;
}

/* Program BOTH interface slots from one reading of the live ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 7. */
int rtl9602c_l34_provision(struct rtl9602c_l34 *l, const struct gpon_edge *e)
{
	int ret;

	l->provisioned = false;
	ret = rtl9602c_l34_wan_setup(l, L34_NETIF_WAN, e->wan_ip, e->wan_mac,
				     e->gw_ip, e->gw_mac, GMAC_PON_PORT,
				     e->wan_vlan);
	if (ret)
		return ret;
	ret = rtl9602c_l34_lan_setup(l, L34_NETIF_LAN, e->lan_ip, e->lan_mac,
				     e->lan_net, e->lan_prefix, e->lan_vlan);
	if (ret)
		return ret;

	l->edge = *e;
	l->provisioned = true;
	pr_info("rtl9602c-l34: provisioned WAN %pI4h via %pI4h (%pM) on netif %u, LAN %pI4h/%u on netif %u\n",
		&e->wan_ip, &e->gw_ip, e->gw_mac, L34_NETIF_WAN,
		&e->lan_net, e->lan_prefix, L34_NETIF_LAN);
	return 0;
}

/* /proc/flowdump: read the outbound NAPT hit bitmap; write ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 8. */
static ssize_t l34_proc_write(struct file *fp, const char __user *ub,
			      size_t n, loff_t *off)
{
	struct rtl9602c_l34 *l = pde_data(file_inode(fp));
	u8 wmac[6], gmac[6];
	char buf[192];
	int ret = 0;

	if (n >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ub, n))
		return -EFAULT;
	buf[n] = '\0';

	if (buf[0] == 'w') {
		unsigned int port, vlan;
		u32 wip, gip;

		if (sscanf(buf, "w %2hhx%2hhx%2hhx%2hhx%2hhx%2hhx %x %x %2hhx%2hhx%2hhx%2hhx%2hhx%2hhx %u %u",
			   &wmac[0], &wmac[1], &wmac[2], &wmac[3], &wmac[4], &wmac[5],
			   &wip, &gip,
			   &gmac[0], &gmac[1], &gmac[2], &gmac[3], &gmac[4], &gmac[5],
			   &port, &vlan) != 16)
			return -EINVAL;
		ret = rtl9602c_l34_wan_setup(l, L34_NETIF_WAN, wip, wmac, gip, gmac,
					     port, vlan);
		pr_info("rtl9602c_l34: wan_setup -> %d\n", ret);
	} else if (buf[0] == 'l') {
		unsigned int prefix, vlan;
		u32 lip, lnet;

		if (sscanf(buf, "l %2hhx%2hhx%2hhx%2hhx%2hhx%2hhx %x %x %u %u",
			   &wmac[0], &wmac[1], &wmac[2], &wmac[3], &wmac[4], &wmac[5],
			   &lip, &lnet, &prefix, &vlan) != 10)
			return -EINVAL;
		ret = rtl9602c_l34_lan_setup(l, L34_NETIF_LAN, lip, wmac, lnet,
					     prefix, vlan);
		pr_info("rtl9602c_l34: lan_setup -> %d\n", ret);
	} else if (buf[0] == 'f') {
		struct l34_flow f = { 0 };
		unsigned int proto, sp, dp, nsp;

		if (sscanf(buf, "f %u %x %u %x %u %x %u",
			   &proto, &f.orig_sip, &sp, &f.orig_dip, &dp,
			   &f.nat_sip, &nsp) != 7)
			return -EINVAL;
		f.l4proto = proto;
		f.orig_sport = sp;
		f.orig_dport = dp;
		f.nat_sport = nsp;
		ret = rtl9602c_l34_flow_add(l, &f);
		pr_info("rtl9602c_l34: flow_add -> %d hw_index=%u\n", ret, f.hw_index);
	} else if (buf[0] == 'x') {
		/* Raw indirect table write: the companion of the read-back ...
		 * dev/MEASURED-rtl9602c_l34.c.md sec 9. */
		unsigned int tbl, tidx;
		u32 w[L34_WORDS_MAX] = { 0 };
		int got = sscanf(buf, "x %u %u %x %x %x %x",
				 &tbl, &tidx, &w[0], &w[1], &w[2], &w[3]);

		if (got < 3)
			return -EINVAL;
		mutex_lock(&l->lock);
		ret = l34_tbl_write(l, (enum l34_tbl)tbl, tidx, w, got - 2);
		mutex_unlock(&l->lock);
		pr_info("rtl9602c_l34: raw write tbl %u idx %u words %u -> %d\n",
			tbl, tidx, got - 2, ret);
	} else {
		return -EINVAL;
	}
	return ret ? ret : n;
}

/* ⚠ `hits_seen` IS RATE-DEPENDENT AND THE NODE SAYS SO. The ...
 * dev/MEASURED-rtl9602c_l34.c.md sec 10. */
/* ★ CONFIG_GPON_FLOW_DIAG gates the READ only.  The bring-up WRITE below is a
 * control, not a measurement: dropping it with a diagnostic flag would change
 * behaviour, and this family is DIAG -- it reports, it never decides. */
#if IS_ENABLED(CONFIG_GPON_FLOW_DIAG)

static const char *const l34_rt_process_name[4] = { "CPU", "DROP", "ARP", "NH" };

static void l34_proc_show_rt(struct seq_file *sf, struct rtl9602c_l34 *l,
			     u16 idx)
{
	u32 w[L34_WORDS_L3ROUTE] = { 0 };
	u32 ip, mask, proc;

	if (l34_tbl_read(l, L34_TBL_L3ROUTE, idx, w, L34_WORDS_L3ROUTE)) {
		seq_printf(sf, "route[%2u] READ FAILED\n", idx);
		return;
	}
	if (!l34_field_get(w, L34_RT_VALID_LSP, L34_RT_VALID_W)) {
		seq_printf(sf, "route[%2u] invalid raw %08x %08x\n",
			   idx, w[0], w[1]);
		return;
	}
	ip = l34_field_get(w, L34_RT_IP_LSP, L34_RT_IP_W);
	mask = l34_field_get(w, L34_RT_MASK_LSP, L34_RT_MASK_W);
	proc = l34_field_get(w, L34_RT_PROCESS_LSP, L34_RT_PROCESS_W);
	seq_printf(sf,
		   "route[%2u] %pI4h maskcode %u (/%u) process %s int %u netif %u rt2wan %u raw %08x %08x\n",
		   idx, &ip, mask, mask + 1, l34_rt_process_name[proc],
		   l34_field_get(w, L34_RT_INT_LSP, L34_RT_INT_W),
		   l34_field_get(w, L34_RT_DENTIF_LSP, L34_RT_DENTIF_W),
		   l34_field_get(w, L34_RT_RT2WANINF_LSP, L34_RT_RT2WANINF_W),
		   w[0], w[1]);
}

static void l34_proc_show_iface(struct seq_file *sf, struct rtl9602c_l34 *l)
{
	u32 netif[L34_WORDS_NETIF] = { 0 };
	u32 cpu_rt[L34_WORDS_L3ROUTE] = { 0 };
	u8 mac[6];
	unsigned int i;
	u32 ip, v, mac_lo, mac_hi, valid;

	if (!l->engine_on) {
		seq_puts(sf, "engine off -- interface tables not read back\n");
		return;
	}
	/* ★ THE ENGINE CONTROL WORDS, READ BACK. l34_engine_on() ...
	 * dev/MEASURED-rtl9602c_l34.c.md sec 11. */
	v = l34_rd(l, L34_SWTCR0);
	seq_printf(sf,
		   "swtcr0 %08x natmode %u limdbc %u v4flrt %u v6flrt %u\nglb_cfg %08x\n",
		   v, (v >> L34_SWTCR0_NATMODE_SH) & 0x3,
		   (v >> L34_SWTCR0_LIMDBC_SH) & 0x3,
		   (v & L34_SWTCR0_V4FLRT_EN) ? 1 : 0,
		   (v & L34_SWTCR0_V6FLRT_EN) ? 1 : 0,
		   l34_rd(l, L34_GLB_CFG));
	/* NETIF has 16 entries even though EXTIP can reference only eight.
	 * Keep invalid entries visible: stale fields and a failed read are
	 * different observations. All twelve fields come from this die's own
	 * stock table descriptor (k0_kernel, file offset 0xcb2d68). */
	for (i = 0; i < L34_NETIF_SLOTS; i++) {
		if (l34_tbl_read(l, L34_TBL_NETIF, i, netif, L34_WORDS_NETIF)) {
			seq_printf(sf, "netif[%u] READ FAILED\n", i);
			continue;
		}
		valid = l34_field_get(netif, L34_NETIF_VALID_LSP,
				      L34_NETIF_VALID_W);
		ip = l34_field_get(netif, L34_NETIF_IP_LSP, L34_NETIF_IP_W);
		mac_lo = l34_field_get(netif, L34_NETIF_GMAC_LSP, 32);
		mac_hi = l34_field_get(netif, L34_NETIF_GMAC_LSP + 32, 16);
		mac[0] = mac_hi >> 8;
		mac[1] = mac_hi;
		mac[2] = mac_lo >> 24;
		mac[3] = mac_lo >> 16;
		mac[4] = mac_lo >> 8;
		mac[5] = mac_lo;
		seq_printf(sf,
			   "netif[%u] %pI4h vlan %u claims %u raw %08x %08x %08x %08x valid %u mac %pM mask %u routing %u mtu %u l34 %u dslite %u dslite_idx %u ctag %u ipv6 %u\n",
			   i, &ip,
			   l34_field_get(netif, L34_NETIF_VLANID_LSP,
					 L34_NETIF_VLANID_W),
			   l34_netif_claims(netif) ? 1 : 0,
			   netif[0], netif[1], netif[2], netif[3], valid, mac,
			   l34_field_get(netif, L34_NETIF_MACMASK_LSP, L34_NETIF_MACMASK_W),
			   l34_field_get(netif, L34_NETIF_ENRTR_LSP, L34_NETIF_ENRTR_W),
			   l34_field_get(netif, L34_NETIF_MTU_LSP, L34_NETIF_MTU_W),
			   l34_field_get(netif, L34_NETIF_L34_LSP, L34_NETIF_L34_W),
			   l34_field_get(netif, L34_NETIF_DSLITE_LSP, L34_NETIF_DSLITE_W),
			   l34_field_get(netif, L34_NETIF_DSLITE_IDX_LSP, L34_NETIF_DSLITE_IDX_W),
			   l34_field_get(netif, L34_NETIF_CTAG_LSP, L34_NETIF_CTAG_W),
			   l34_field_get(netif, L34_NETIF_IPV6_LSP, L34_NETIF_IPV6_W));
		if (!valid || i >= L34_EXTIP_SLOTS)
			continue;
		l34_proc_show_rt(sf, l, i);
		l34_proc_show_rt(sf, l, i + L34_RT_CPU_SLOT_OFF);
		/* the safety verdict, on what came BACK from the silicon */
		if (l34_tbl_read(l, L34_TBL_L3ROUTE, i + L34_RT_CPU_SLOT_OFF,
				 cpu_rt, L34_WORDS_L3ROUTE))
			continue;
		seq_printf(sf, "netif[%u] blackholes %u\n", i,
			   l34_iface_blackholes(netif, cpu_rt, ip, i) ? 1 : 0);
	}
	/* ★★ THE GENERAL LPM POOL, WHICH THE LOOP ABOVE COULD NOT REACH.  It
	 * prints route[i] and route[i+8] for VALID netifs only, so an entry in
	 * L3ROUTE[2..7] -- where a real route goes, netifs 2..7 never being
	 * provisioned -- was written and then INVISIBLE.  Adding a table entry a
	 * dump cannot show is how a repair becomes unverifiable: the upper half
	 * of the default route landed there and could not be confirmed. */
	for (i = 0; i < L34_RT_POOL_N; i++)
		l34_proc_show_rt(sf, l, L34_RT_POOL_BASE + i);

}

/* The NAPT entries this driver most recently INSTALLED, read BACK from the
 * silicon and decoded.  Until 2026-09-16 this node showed the hit BITMAP and
 * the interface tables and never an entry, so the one question a non-matching
 * flow raises -- does the installed key match the traffic? -- could not be
 * asked from the board.  Nothing could disagree with us, so no measurement
 * could catch a wrong field.  MEASURED that day: installs=15, live=1, hits=11
 * across a 105 Mbps flow, with the CPU still at 83%.
 *
 * It reads what the HARDWARE holds, never what the driver believes it wrote:
 * an entry the silicon rejected or rewrote is exactly the case worth seeing.
 */
static void l34_proc_show_recent(struct seq_file *sf, struct rtl9602c_l34 *l)
{
	u32 w[L34_WORDS_NAPTR_IN] = { 0 };
	unsigned int i, n;

	n = l->offer_n < L34_RECENT ? l->offer_n : L34_RECENT;
	seq_printf(sf, "recent flow keys OFFERED (%u of %u):\n", n, l->offer_n);
	for (i = 0; i < n; i++) {
		const struct l34_offer *o = &l->recent_offer[i];

		seq_printf(sf, "  offer %pI4h:%u -> %pI4h:%u proto %u\n",
			   &o->sip, o->sport, &o->dip, o->dport, o->proto);
	}

	n = l->recent_n < L34_RECENT ? l->recent_n : L34_RECENT;
	seq_printf(sf, "recent NAPT entries (%u of %u installed):\n",
		   n, l->recent_n);
	for (i = 0; i < n; i++) {
		u16 idx = l->recent_idx[i];
		u32 out[L34_WORDS_NAPT_OUT] = { 0 };
		u32 intip, extport, intport, tcp, valid, in_idx;

		/* ⚠⚠ TWO INDEX SPACES, AND CONFLATING THEM READS ZEROS FOREVER.
		 * `hw_index` addresses NAPT_OUT (the hash slot), and that word
		 * CARRIES the NAPTR_IN index -- L34_NAPT_HASHIN_IDX says so in
		 * as many words. The first cut of this renderer read NAPTR_IN
		 * AT the NAPT_OUT index and printed seven all-zero entries,
		 * which reads exactly like "the silicon holds nothing" and is
		 * a bug in the reader. The hit bitmap is indexed by hw_index
		 * too, which is the cross-check: hits were non-zero while this
		 * showed empty entries, and that contradiction is what caught
		 * it. */
		if (l34_tbl_read(l, L34_TBL_NAPT_OUT, idx, out,
				 L34_WORDS_NAPT_OUT)) {
			seq_printf(sf, "  napt_out[%4u] READ FAILED\n", idx);
			continue;
		}
		in_idx = l34_field_get(out, L34_NAPT_HASHIN_IDX_LSP, 12);
		seq_printf(sf, "  napt_out[%4u] valid %u -> naptr_in[%u] raw %08x\n",
			   idx, l34_field_get(out, L34_NAPT_VALID_LSP, 1),
			   in_idx, out[0]);
		if (l34_tbl_read(l, L34_TBL_NAPTR_IN, in_idx, w,
				 L34_WORDS_NAPTR_IN)) {
			seq_printf(sf, "  naptr[%4u] READ FAILED\n", in_idx);
			continue;
		}
		idx = (u16)in_idx;
		intip   = l34_field_get(w, L34_NAPTR_INTIP_LSP, 32);
		intport = l34_field_get(w, L34_NAPTR_INTPORT_LSP, 16);
		extport = l34_field_get(w, L34_NAPTR_EXTPORT_LSP, 16);
		tcp     = l34_field_get(w, L34_NAPTR_TCP_LSP, 1);
		valid   = l34_field_get(w, L34_NAPTR_VALID_LSP, 2);
		seq_printf(sf,
			   "  naptr[%4u] %pI4h:%u -> extport %u %s valid %u extipidx %u raw %08x %08x %08x\n",
			   idx, &intip, intport, extport, tcp ? "tcp" : "udp",
			   valid,
			   l34_field_get(w, L34_NAPTR_EXTIPIDX_LSP, 3),
			   w[0], w[1], w[2]);
	}
}

static int l34_proc_show(struct seq_file *sf, void *v)
{
	struct rtl9602c_l34 *l = sf->private;
	unsigned int i, now = 0;

	/* ★★★ THE GATE'S OWN STATE COMES FIRST, AND BEFORE THE LOCK.  This node
	 * used to be created INSIDE `if (hw_nat)`, so the one question it exists
	 * to answer -- why does the engine install nothing? -- could not be asked
	 * while the engine was off: the file simply did not exist, which reads as
	 * "this image has no accelerator instrumentation" rather than "the knob is
	 * 0".  It is registered unconditionally now, so the ANSWER has to live
	 * here.
	 *
	 * ⚠ BEFORE `mutex_lock`, NOT AFTER: with the gate closed
	 * `rtl9602c_l34_init()` never ran, so `l->lock` was never `mutex_init`ed
	 * and taking it is undefined behaviour.  `l->ready` is safe to read
	 * because the private area of the netdev is zeroed by
	 * devm_alloc_etherdev() long before this can be opened.
	 */
	if (!l || !l->ready) {
		seq_puts(sf,
			 "engine NOT INITIALISED\n"
			 "The L34 flow engine is gated on the rtl9602c_eth.hw_nat module\n"
			 "parameter and it is 0 in this boot, so the engine was never brought\n"
			 "up and nothing below it was ever established. This is a CONFIG\n"
			 "state, not a device finding: no flow was refused, because none was\n"
			 "ever offered to an engine that does not exist.\n");
		return 0;
	}

	/* A table read writes the shared command register. Serialize the whole
	 * snapshot against flow installs, raw writes and other readers. */
	mutex_lock(&l->lock);
	seq_printf(sf, "provisioned %u\n", l->provisioned ? 1 : 0);
	if (l->provisioned)
		seq_printf(sf, "wan %pI4h via %pI4h %pM netif %u\nlan %pI4h/%u %pM netif %u\n",
			   &l->edge.wan_ip, &l->edge.gw_ip, l->edge.gw_mac,
			   L34_NETIF_WAN, &l->edge.lan_net, l->edge.lan_prefix,
			   l->edge.lan_mac, L34_NETIF_LAN);
	/* ★ THE THREE COUNTS THAT SEPARATE FOUR DIFFERENT SILENCES, ...
	 * dev/MEASURED-rtl9602c_l34.c.md sec 12. */
	/* ★ THE CORE'S OWN LINE, identical on every family: which of the
	 * lifecycle's sixteen refusal causes fired, and how often.  The four
	 * fields below it are what only THIS shell can read; `cap` is the chip's
	 * own table size and `hits` its hit witness. */
	{
		struct gpon_flow_diag d = {
			.valid = GPON_FDIAG_HAS_OFFERED |
				 GPON_FDIAG_HAS_LIVE |
				 GPON_FDIAG_HAS_CAPACITY |
				 GPON_FDIAG_HAS_HITS,
			.offered = l->offered,
			.live = l->installs - l->removals,
			.capacity = L34_NAPT_ENTRIES,
			.hw_hits = l->hits_seen,
		};
		char line[256];

		gpon_flow_offload_diag(l->fo, &d, line, sizeof(line));
		seq_printf(sf, "%s\n", line);
	}
	seq_printf(sf, "binds %u\noffered %u\ninstalls %u\nremovals %u\nrefusals %u\nds_legs %u\n",
		   l->binds, l->offered, l->installs,
		   l->removals, l->refusals, l->ds_legs);
	/* ★ THE FEW LINES THAT DECIDE, ON DEMAND -- not a log flood. ...
	 * dev/MEASURED-rtl9602c_l34.c.md sec 15. */
	if (l->vlan_refused)
		seq_printf(sf, "vlan_refused %u last_vid %u\n",
			   l->vlan_refused, l->vlan_refused_vid);
	if (l->refuse_why)
		seq_printf(sf, "last_refusal %s\n", l->refuse_why);

	l34_proc_show_iface(sf, l);
	l34_proc_show_recent(sf, l);

	/* ONE pass: the read is what clears the bitmap, so it cannot be walked
	 * twice, and buffering 128 words on the kernel stack to print a summary
	 * line first is not worth the frame. */
	seq_puts(sf, "outbound NAPT hit bitmap (set bits = slots matched since the last read of this file):\n");
	for (i = 0; i < L34_NAPT_ENTRIES / 32; i++) {
		u32 w = l34_rd(l, L34_NAPT_HIT + 4 * i);

		now += hweight32(w);
		if (w)
			seq_printf(sf, "  [%4u] 0x%08x\n", i * 32, w);
	}
	l->hits_seen += now;
	seq_printf(sf, "hits_now %u\nhits_seen %u\n", now, l->hits_seen);
	mutex_unlock(&l->lock);
	return 0;
}

#else	/* the facility is ABSENT, and the node says so rather than printing
	 * counters nobody measured */
static int l34_proc_show(struct seq_file *sf, void *v)
{
	(void)v;
	seq_puts(sf,
		 "flowdump: diagnostics not compiled in (CONFIG_GPON_FLOW_DIAG=n); the bring-up write still works\n");
	return 0;
}
#endif

static int l34_proc_open(struct inode *ino, struct file *fp)
{
	return single_open(fp, l34_proc_show, pde_data(ino));
}

static const struct proc_ops l34_proc_ops = {
	.proc_open	= l34_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= l34_proc_write,
};

void rtl9602c_l34_proc_init(struct rtl9602c_l34 *l)
{
	proc_create_data("flowdump", 0600, NULL, &l34_proc_ops, l);
}
