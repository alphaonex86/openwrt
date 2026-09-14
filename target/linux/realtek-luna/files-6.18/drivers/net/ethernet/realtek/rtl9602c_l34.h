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

/*
 * Indirect table-access block (byte offsets within the switch-core MMIO).
 * A table op writes the data bank, issues a command with the table type and
 * entry index, then polls the matching EXE bit until the engine clears it.
 */
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
/*
 * ⚠⚠ THESE BIT NAMES WERE WRONG UNTIL 2026-09-12, AND THE WRONG ONE WAS ON THE
 * REGISTER THAT DECIDES.  They read V6RT_EN = BIT(31) and V4RT_EN = BIT(30);
 * this die's own chipdef (SWTCR0_RTL9602C_FIELDS) puts CF_SIP_ARP_TRF_EN at 31,
 * V4FLRT_EN at 30 and V6FLRT_EN at **29**, and the vendor's own DAL confirms the
 * meaning by name -- L34_GLOBAL_V4FLOW_RT_STATE writes V4FLRT_ENf.
 * So l34_proc_show_iface() printed `v6rt 0` while IPv6 flow routing was ENABLED
 * and bit 31 -- an ARP-trap enable, not a routing enable -- was being read as if
 * it were one.  A name that is off by two bits is not cosmetic: it is what a
 * reader checks the silicon against.
 * MEASURED the same day on this board, stock vs ours, over the L34 page:
 *   stock 0x84801e10   ours 0x64000020 (boot) -> 0x64000e20 (engine armed)
 * i.e. stock runs with BOTH flow-route enables CLEAR and we run with BOTH SET,
 * and nothing in this tree writes either -- they are the power-on default the
 * vendor clears and we never did.
 */
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

/*
 * Table types (L34_CMD_TYPE). The NAPT path is a hashed pair: an outbound slot
 * table (key hash -> inbound index) and an inbound rewrite table (the actual
 * 5-tuple + post-NAT addresses/ports). EXTIP/NEXTHOP/NETIF/ARP/L3ROUTE support
 * the routing the NAT entries reference.
 */
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

/*
 * Entry field LAYOUTS (L34_NAPT_* / L34_NAPTR_* / L34_EXTIP_* / L34_NETIF_*
 * / L34_NH_* / L34_ARP_* / L34_RT_* / L2UC_* and the L2_STS decode bits)
 * moved to rtl9602c_l34_logic.h WITH their encoders (flowcore, 2026-09-02)
 * -- the TXD3_9602C_* precedent: a layout fact exists ONCE, where the
 * function that packs it lives, and the host suite drives the shipping
 * encoders on x86.  This file keeps what is TRANSPORT: registers, table
 * types, word counts, geometry, and the two shell-facing values below.
 */
#define L34_EXTIP_SLOTS		8	/* EXTIP/EXTIP_IDX is 3-bit: netif idx must be < 8 */
#define L34_NETIF_DEF_VLAN	1

/*
 * The two interface slots this driver provisions.  They are a CHOICE, not a
 * hardware fact, and they are named because three tables index each other by
 * them: a NAPTR's EXTIP_IDX is the WAN slot, NEXTHOP[WAN] and EXTIP[WAN] carry
 * that interface's next hop, and lan_setup writes L3ROUTE[LAN] and
 * L3ROUTE[LAN + 8].  Spelling them as bare 0 and 1 at four call sites is how
 * two of them come to disagree.
 */
#define L34_NETIF_WAN		0
#define L34_NETIF_LAN		1

/*
 * Two more index CHOICES the same argument covers, and they were bare
 * literals at their call sites: the CPU self-route slot offset (lan_setup
 * writes L3ROUTE[LAN] and L3ROUTE[LAN + this]) and the ARP half the WAN
 * gateway is placed in.  ⚠ THE SELF-ROUTE OFFSET IS NOT A HARDWARE FACT --
 * L3ROUTE holds 16 entries and any free one is legal; whether a HIGHER index
 * can win a lookup against a lower one is a MEASUREMENT, read back through
 * /proc/flowdump ("route" section), not something to assume from the prefix.
 */
#define L34_RT_CPU_SLOT_OFF	8	/* L3ROUTE[netif + 8] = that netif's CPU self-route */
#define L34_ARP_WAN_BASE	64	/* the WAN half of the 128-entry ARP table */

/*
 * L2 unicast table (the gateway/peer destination MAC). Reached through a
 * SEPARATE indirect block from the L34 NAT block: a MAC-method insert lets the
 * engine hash the MAC, place it in a free way, and report the assigned index
 * that NEXTHOP/ARP nhIdx then reference.
 */
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
	u32		refusals;	/* flows left on the software path	*/
	u32		ds_legs;	/* reply legs declined -- an EXPECTED non-event */
	u32		binds;		/* flowtable blocks this driver accepted	*/
	u32		offered;	/* per-flow requests that REACHED this driver */
	u32		core_refused;	/* ...that the common lifecycle declined first */
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
