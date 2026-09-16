/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RTL9602C "Luna" hardware L3/L4 forwarding + NAPT engine.
 *
 * The SoC switch core carries an L3/L4 (routing + NAPT) accelerator reached
 * through a single indirect table-access block. This header describes that
 * block and the per-flow NAPT table model the flow-offload path programs so
 * established WAN<->LAN connections are NAT'd/forwarded in hardware instead of
 * on the CPU. Clean-room: register offsets, field positions and table geometry
 * are hardware facts, re-expressed here as new idiomatic definitions.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#ifndef _RTL9602C_L34_H
#define _RTL9602C_L34_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/io.h>

#include "gpon_edge.h"	/* core: the router edge, read from the live kernel */

/* Indirect table-access block (byte offsets within the ...
 * dev/MEASURED-rtl9602c_l34.h.md sec 7. */
#define L34_CMD			0x800100	/* command/trigger register */
#define  L34_CMD_RD_EXE		BIT(25)		/* start read  (self-clears when done) */
#define  L34_CMD_WR_EXE		BIT(24)		/* start write (self-clears when done) */
#define  L34_CMD_TYPE_SHIFT	16		/* [19:16] table type */
#define  L34_CMD_TYPE_MASK	0xf
#define  L34_CMD_IDX_MASK	0xffff		/* [15:0] entry index */

#define L34_CLR			0x800104	/* per-table-type reset, self-clearing */
#define L34_RDATA		0x800108	/* read-data bank base  (word0..) */
#define L34_WDATA		0x80011c	/* write-data bank base (word0..) */
#define L34_SWTCR0		0x800010	/* engine control (NAT mode / flow route) */
/* ⚠⚠ THESE BIT NAMES WERE WRONG UNTIL 2026-09-12, AND THE ...
 * dev/MEASURED-rtl9602c_l34.h.md sec 1. */
#define  L34_SWTCR0_V6FLRT_EN	BIT(29)		/* IPv6 flow routing	*/
#define  L34_SWTCR0_V4FLRT_EN	BIT(30)		/* IPv4 flow routing	*/
#define  L34_SWTCR0_CF_SIP_ARP_TRF_EN BIT(31)	/* NOT a routing enable */
#define  L34_SWTCR0_NATMODE_SH	10		/* [11:10] b0=L3 NAT en, b1=L4 NAT en */
/* [9:8] the chipdef calls this LIMDBC.  The old comment read "0=VLAN-base,
 * 2=MAC-base"; that reading is UNVERIFIED and is kept only as a lead -- what IS
 * measured is that stock holds 2 here and so do we. */
#define  L34_SWTCR0_LIMDBC_SH	8
#define L34_GLB_CFG		0x01106c	/* master L34 routing enable */
#define L34_NAPT_HIT		0x800400	/* outbound NAPT hit/age bitmap (idx/32 words) */

/* Table types (L34_CMD_TYPE). The NAPT path is a hashed pair: ...
 * dev/MEASURED-rtl9602c_l34.h.md sec 2. */
enum l34_tbl {
	L34_TBL_L3ROUTE		= 0,	/* 16 entries, 2 words: LPM route -> nexthop */
	L34_TBL_NEXTHOP		= 2,	/* 16 entries, 1 word:  egress intf + L2 (ARP) index */
	L34_TBL_NETIF		= 3,	/* 16 entries, 4 words: per-interface MAC/VLAN/MTU/IP */
	L34_TBL_EXTIP		= 4,	/* 8 entries,  3 words: external (WAN) IP -> nexthop */
	L34_TBL_NAPTR_IN	= 9,	/* 4096 entries, 3 words: inbound rewrite (NAT tuple) */
	L34_TBL_NAPT_OUT	= 10,	/* 4096 entries, 1 word:  outbound hash slot */
	L34_TBL_ARP		= 13,	/* 128 entries, 2 words: IP -> L2 (next-hop MAC) index */
};

/* Word counts per table type (data-bank words moved per op). */
#define L34_WORDS_L3ROUTE	2
#define L34_WORDS_NEXTHOP	1
#define L34_WORDS_NETIF		4
#define L34_WORDS_EXTIP		3
#define L34_WORDS_NAPTR_IN	3
#define L34_WORDS_NAPT_OUT	1
#define L34_WORDS_ARP		2

#define L34_NAPT_ENTRIES	4096
#define L34_NAPT_WAYS		4		/* 4-way bucket: index = (hash << 2) + way */

/* Entry field LAYOUTS (L34_NAPT_* / L34_NAPTR_* / L34_EXTIP_* ...
 * dev/MEASURED-rtl9602c_l34.h.md sec 3. */
#define L34_EXTIP_SLOTS		8	/* EXTIP/EXTIP_IDX is 3-bit: netif idx must be < 8 */
#define L34_NETIF_DEF_VLAN	1

/* The two interface slots this driver provisions. They are a ...
 * dev/MEASURED-rtl9602c_l34.h.md sec 4. */
#define L34_NETIF_WAN		0
#define L34_NETIF_LAN		1

/* Two more index CHOICES the same argument covers, and they ...
 * dev/MEASURED-rtl9602c_l34.h.md sec 5. */
#define L34_RT_CPU_SLOT_OFF	8	/* L3ROUTE[netif + 8] = that netif's CPU self-route */

/* The general LPM pool: L3ROUTE[2..7].  The table's convention is
 * L3ROUTE[netif] = that netif's local route and L3ROUTE[netif + 8] = its CPU
 * self-route, and this driver provisions exactly TWO netifs (0 = WAN, 1 = LAN),
 * so slots 2..7 are unreachable by that convention and free for real routes.
 * ⚠ THE CONSTRAINT IS LOAD-BEARING: provisioning a third netif would collide,
 * so l34_rt_pool_slot() refuses an index the pool cannot hold. */
#define L34_RT_POOL_BASE	2
#define L34_RT_POOL_N		6
#define L34_ARP_WAN_BASE	64	/* the WAN half of the 128-entry ARP table */

/* L2 unicast table (the gateway/peer destination MAC). ...
 * dev/MEASURED-rtl9602c_l34.h.md sec 6. */
#define L2_CMD			0x12000
#define  L2_CMD_TYPE_SH		0	/* [2:0] table type (0 = L2_UC) */
#define  L2_CMD_WR		BIT(3)	/* 0 = read, 1 = write */
#define  L2_CMD_METHOD_SH	4	/* [6:4] 0 = MAC-hash, 1 = direct address */
#define L2_STS			0x12004
/* the ADDR/CAM/HIT decode bits moved to rtl9602c_l34_logic.h with
 * l34_l2uc_sts_index(); BUSY stays here with the poll that owns it */
#define  L2_STS_BUSY		BIT(13)
#define L2_WDATA		0x12008	/* word0..2 @ +4 */
#define L2_RDATA		0x1201c	/* word0..2 @ +4 */
#define L34_WORDS_L2UC		3
#define L2_METHOD_MAC		0

/* Per-flow programming request, filled by the flow-offload glue (endian-safe:
 * addresses/ports are kept in host order here and packed with explicit math). */
struct gpon_flow_offload;

struct l34_flow {
	u8	l4proto;		/* IPPROTO_TCP / IPPROTO_UDP */
	u32	orig_sip, orig_dip;	/* ingress (original-direction) 5-tuple */
	u16	orig_sport, orig_dport;
	u32	nat_sip, nat_dip;	/* post-NAT addresses (0 = unchanged) */
	u16	nat_sport, nat_dport;	/* post-NAT ports (0 = unchanged) */
	u8	egress_netif;		/* L34_TBL_NETIF index of the output interface */
	u8	nexthop;		/* L34_TBL_NEXTHOP index (gateway L2) */
	u16	hw_index;		/* assigned NAPT slot, valid after add (for del/stats) */
};

struct rtl9602c_l34 {
	void __iomem	*sw;		/* switch-core MMIO base (offsets above are relative) */
	struct mutex	lock;		/* serialises table ops */
	bool		ready;		/* table-access plumbing usable */
	bool		engine_on;	/* NAT engine enabled (deferred, lazy) */
	bool		provisioned;	/* the interface tables hold real values */
	struct gpon_edge edge;		/* ...and THESE are the values they hold */
	u32		installs;	/* flows programmed into the engine	*/
	u32		removals;
	u32		hits_seen;	/* set hit bits observed -- see the node */
	/* ★★ ONE READER FOR A CLEAR-ON-READ BITMAP.  L34_NAPT_HIT clears on
	 * read, and it had TWO readers: the flowtable's per-flow liveness op
	 * and /proc/flowdump.  Each consumed the other's evidence -- the
	 * diagnostic could age a live flow OUT by clearing the bit its own GC
	 * was about to look for, and the witness read zeros because the GC had
	 * just harvested them.  The bitmap is now swept ONCE per jiffy into a
	 * sticky shadow; liveness consumes only its OWN slot, and the
	 * diagnostic consumes nothing, because a DIAG may not destroy evidence
	 * a control path needs. */
	u32		hit_shadow[L34_NAPT_ENTRIES / 32];
	unsigned long	hit_swept;	/* jiffies of the last sweep */
	u32		refusals;	/* flows left on the software path	*/
	u32		ds_legs;	/* reply legs declined -- an EXPECTED non-event */
	/* The last few NAPT indexes this driver INSTALLED, so the dump can read
	 * the entries BACK.  /proc/flowdump rendered the hit BITMAP and the
	 * interface tables and never the entries, so *does the installed key
	 * match the flow?* could not be asked from the board at all -- the shape
	 * this project calls a register the driver writes and cannot read back,
	 * where nothing can disagree with us.  MEASURED 2026-09-16: installs=15,
	 * live=1, hits=11 across a 105 Mbps flow, i.e. an entry exists and the
	 * traffic is not matching it, and no instrument could say which field is
	 * wrong.  A ring, not a list: it costs 16 bytes and answers the question
	 * for the flows that matter, which are the most recent. */
#define L34_RECENT		8
	u16		recent_idx[L34_RECENT];
	u8		recent_n;

	/* The last few flow keys the core OFFERED, recorded BEFORE any refusal so
	 * a declined one is visible too.  MEASURED 2026-09-16: offered=188,
	 * installed=7, live=1, why{dup-cookie=87 engine=94} -- and the single LIVE
	 * entry was UDP while the throughput legs were TCP, so what was being
	 * accelerated was the latency probe.  Whether a TCP 5-tuple was ever
	 * offered at all could not be asked: the counters name CAUSES, never the
	 * flow they were about. */
	struct l34_offer {
		u32	sip, dip;
		u16	sport, dport;
		u8	proto;
	}		recent_offer[L34_RECENT];
	u8		offer_n;
	u32		binds;		/* flowtable blocks this driver accepted	*/
	u32		offered;	/* per-flow requests that REACHED this driver */
	struct gpon_flow_offload *fo;	/* the COMMON lifecycle, for its diag line */
	u32		vlan_refused;	/* ...of those, ones carrying a VLAN tag */
	u16		vlan_refused_vid;
	const char	*refuse_why;	/* ...and the reason for the last one	*/
};

/* Public API (mainline flow-offload glue calls these). */
int  rtl9602c_l34_init(struct rtl9602c_l34 *l, void __iomem *sw);
int  rtl9602c_l34_wan_setup(struct rtl9602c_l34 *l, u8 idx, u32 wan_ip,
			    const u8 *wan_mac, u32 gw_ip, const u8 *gw_mac,
			    u8 wan_port, u16 vlan);
int  rtl9602c_l34_lan_setup(struct rtl9602c_l34 *l, u8 idx, u32 lan_ip,
			    const u8 *lan_mac, u32 lan_net, u8 prefix, u16 vlan);
int  rtl9602c_l34_provision(struct rtl9602c_l34 *l, const struct gpon_edge *e);
int  rtl9602c_l34_flow_add(struct rtl9602c_l34 *l, struct l34_flow *f);
int  rtl9602c_l34_flow_del(struct rtl9602c_l34 *l, struct l34_flow *f);
int  rtl9602c_l34_flow_hit(struct rtl9602c_l34 *l, u16 hw_index, bool *active);
void rtl9602c_l34_proc_init(struct rtl9602c_l34 *l);	/* bring-up test harness */

#endif /* _RTL9602C_L34_H */
