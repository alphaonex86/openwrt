// SPDX-License-Identifier: GPL-2.0
/* Cortina-Access NI Ethernet driver for the Realtek RTL9607F ...
 * dev/MEASURED-cortina-ni-ethtool.c.md sec 1. */

#include <linux/bitfield.h>
#include <linux/build_bug.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/ratelimit.h>
#include <linux/spinlock.h>
#include <linux/stddef.h>
#include <linux/string.h>

#include "cortina-ni.h"
#include "cortina-ni-regs.h"
#include "cortina-access.h"

static inline void __iomem *ni_base(struct cortina_ni *ni)
{
	return ni->win[CA_NI_WIN_NI];
}

#if IS_ENABLED(CONFIG_GPON_FLOW_DIAG)
static DEFINE_MUTEX(cortina_l2_drop_lock);

static u64 cortina_ni_l2_drop_read(struct cortina_ni *ni, unsigned int reason)
{
	void __iomem *base = ni_base(ni);
	u64 value = ~0ULL;
	int ret;

	if (!base || reason >= CA_NI_L2FE_DROP_REASON_COUNT)
		return value;
	mutex_lock(&cortina_l2_drop_lock);
	ret = ca_ni_access_go(base + CA_NI_L2FE_PE_DROP_STTS_ACCESS,
			      CA_NI_IND_ACCESS_GO | reason, NULL);
	if (ret)
		dev_warn_ratelimited(ni->dev, "L2 drop counter %u: %d\n",
				     reason, ret);
	else
		value = readl(base + CA_NI_L2FE_PE_DROP_STTS_DATA);
	mutex_unlock(&cortina_l2_drop_lock);
	return value;
}
#endif

/* the NI_HV read-and-clear counters: ONE reader, driver-side ...
 * dev/MEASURED-cortina-ni-ethtool.c.md sec 2. */
static const u32 cortina_ni_nihv_off[CA_NI_NIHV_CNT_COUNT] = {
	[CA_NI_NIHV_L3FE_RX]	= CA_NI_NI_L3FE_RX_PKT_CNT,
	[CA_NI_NIHV_L3QM_RX]	= CA_NI_NI_L3QM_RX_PKT_CNT,
	[CA_NI_NIHV_L3QM_TX]	= CA_NI_NI_L3QM_TX_PKT_CNT,
	[CA_NI_NIHV_MCE_RX]	= CA_NI_NI_MCE_RX_PKT_CNT,
	[CA_NI_NIHV_DMA_RX]	= CA_NI_NI_DMA_RX_PKT_CNT,
};

void cortina_ni_nihv_sample(struct cortina_ni *ni,
			    u64 out[CA_NI_NIHV_CNT_COUNT])
{
	void __iomem *base = ni_base(ni);
	unsigned long flags;
	unsigned int i;

	if (!base) {
		memset(out, 0, sizeof(*out) * CA_NI_NIHV_CNT_COUNT);
		return;
	}

	/* The readl and the fold are ONE critical section on purpose. ...
	 * dev/MEASURED-cortina-ni-ethtool.c.md sec 3. */
	spin_lock_irqsave(&ni->nihv_lock, flags);
	for (i = 0; i < CA_NI_NIHV_CNT_COUNT; i++) {
		ni->nihv_total[i] += readl(base + cortina_ni_nihv_off[i]);
		out[i] = ni->nihv_total[i];
	}
	spin_unlock_irqrestore(&ni->nihv_lock, flags);
}

/* ------------------------------------------------------------------ */
/* the statistics table                                                 */
/* ------------------------------------------------------------------ */

enum ca_ni_stat_src {
	CA_ST_RX_U64,		/* u64 at @arg bytes into struct cortina_ni_rx */
	CA_ST_TX_U64,		/* u64 at @arg bytes into struct cortina_ni_tx */
	CA_ST_NI_REG,		/* plain cumulative register at NI + @arg      */
	CA_ST_NIHV,		/* index @arg into the read-and-clear totals   */
	CA_ST_PORT_MIB,		/* per-port RX MIB, @arg = counter id, i = port*/
	CA_ST_DRV_FLAG,		/* a driver state bit, @arg selects which      */
	CA_ST_L3FE,		/* index @arg into the offload snapshot        */
	CA_ST_EPP_WRPTR,	/* EPP write pointer, masked to a ring offset  */
	CA_ST_CB_OCC,		/* central-buffer occupancy aggregate, @arg    */
	CA_ST_CB_PORT_FREE,	/* CB per-port free-count word, @arg = port    */
	CA_ST_PHY_LINK,		/* per-GPHY-port PHY link, i = port            */
	CA_ST_L2_DROP,
};

/* CA_ST_CB_OCC selectors */
#define CA_ST_CB_TOTAL		0
#define CA_ST_CB_MAX		1
#define CA_ST_CB_NONZERO	2

/* CA_ST_DRV_FLAG selectors */
#define CA_ST_FLAG_RX_UP	0

/* One ROW may stand for a FAMILY of counters: @n repeats, ...
 * dev/MEASURED-cortina-ni-ethtool.c.md sec 4. */
struct ca_ni_stat_grp {
	const char		*fmt;
	u16			n;
	enum ca_ni_stat_src	src;
	u32			arg;
	u32			step;
};

#define S_RX(f)		((u32)offsetof(struct cortina_ni_rx, f))
#define S_TX(f)		((u32)offsetof(struct cortina_ni_tx, f))

static const struct ca_ni_stat_grp cortina_ni_stat_grps[] = {
	/* ---- receive, the driver's own software counters ---------------- */
	{ "rx_datapath_up",		1, CA_ST_DRV_FLAG, CA_ST_FLAG_RX_UP },
	{ "rx_frames",			1, CA_ST_RX_U64, S_RX(frames) },
	{ "rx_bytes",			1, CA_ST_RX_U64, S_RX(bytes) },
	{ "rx_napi_polls",		1, CA_ST_RX_U64, S_RX(polls) },
	{ "rx_headerless_frames",	1, CA_ST_RX_U64, S_RX(swid_frames) },
	{ "rx_pon_omci_frames",		1, CA_ST_RX_U64, S_RX(pon_frames) },
	{ "rx_pon_wan_frames",		1, CA_ST_RX_U64, S_RX(wan_frames) },
	{ "rx_pon_wan_l3_miss_frames",	1, CA_ST_RX_U64, S_RX(wan_l3_frames) },
	{ "rx_deepq_frames",		1, CA_ST_RX_U64, S_RX(dq_frames) },
	{ "rx_drop_no_sop",		1, CA_ST_RX_U64, S_RX(drop_nosop) },
	{ "rx_drop_bad_paddr",		1, CA_ST_RX_U64, S_RX(drop_badpa) },
	{ "rx_drop_bad_len",		1, CA_ST_RX_U64, S_RX(drop_len) },
	{ "rx_drop_runt",		1, CA_ST_RX_U64, S_RX(drop_runt) },
	{ "rx_drop_oversize",		1, CA_ST_RX_U64, S_RX(drop_oversize) },
	{ "rx_drop_no_buffer",		1, CA_ST_RX_U64, S_RX(drop_nobuf) },
	{ "rx_slot_dead",		1, CA_ST_RX_U64, S_RX(slot_dead) },
	{ "rx_stale_buffer",		1, CA_ST_RX_U64, S_RX(stale_buf) },
	{ "rx_pool_push_fail",		1, CA_ST_RX_U64, S_RX(push_fail) },
	{ "rx_gphy_recoveries",		1, CA_ST_RX_U64, S_RX(recoveries) },
	{ "rx_link_rearms",		1, CA_ST_RX_U64, S_RX(rearms) },
	/* one flow must stay on ONE VoQ: two of these climbing during a
	 * unidirectional run means the hardware spread it and the fixed drain
	 * order can reorder the flow */
	{ "rx_voq%u_frames",	CA_NI_RX_VOQ_COUNT, CA_ST_RX_U64,
	  S_RX(voq_frames), sizeof(u64) },
	/* per-CPU-port EPP interrupt index (silicon), NOT the Linux IRQ number
	 * the board happened to allocate */
	{ "rx_epp_irq%u_events", CA_NI_RX_NUM_IRQS, CA_ST_RX_U64,
	  S_RX(irq_hits), sizeof(u64) },
	{ "rx_chain_frames",		1, CA_ST_RX_U64, S_RX(chain_frames) },
	{ "rx_chain_segments",		1, CA_ST_RX_U64, S_RX(chain_segs) },
	/* a HIGH-WATER MARK, not a count - named so nobody differences it */
	{ "rx_chain_max_segments",	1, CA_ST_RX_U64, S_RX(chain_max_segs) },
	{ "rx_chain_abort",		1, CA_ST_RX_U64, S_RX(chain_abort) },
	{ "rx_chain_reopen",		1, CA_ST_RX_U64, S_RX(chain_reopen) },
	{ "rx_chain_orphan",		1, CA_ST_RX_U64, S_RX(chain_orphan) },
	{ "rx_chain_bad_total",		1, CA_ST_RX_U64, S_RX(chain_badtotal) },
	{ "rx_chain_too_long",		1, CA_ST_RX_U64, S_RX(chain_toolong) },
	{ "rx_chain_short",		1, CA_ST_RX_U64, S_RX(chain_short) },
	{ "rx_chain_headerless",	1, CA_ST_RX_U64, S_RX(chain_swid) },
	{ "rx_chain_dlen_mismatch",	1, CA_ST_RX_U64, S_RX(chain_dlen_diff) },

	/* ---- transmit, the driver's own software counters ---------------- */
	{ "tx_drop_no_dma_map",		1, CA_ST_TX_U64, S_TX(drop_nomap) },
	{ "tx_drop_linearize",		1, CA_ST_TX_U64, S_TX(drop_linearize) },
	{ "tx_drop_oversize",		1, CA_ST_TX_U64, S_TX(drop_oversize) },
	{ "tx_ring_busy",		1, CA_ST_TX_U64, S_TX(tx_busy) },
	{ "tx_lan_fdb_hit",		1, CA_ST_TX_U64, S_TX(lan_hit) },
	{ "tx_lan_flood",		1, CA_ST_TX_U64, S_TX(lan_flood) },
	{ "tx_lan_flood_copies",	1, CA_ST_TX_U64, S_TX(lan_dup) },
	{ "tx_lan_fdb_learn",		1, CA_ST_TX_U64, S_TX(lan_learn) },
	{ "tx_lan_fdb_flush",		1, CA_ST_TX_U64, S_TX(lan_flush) },
	{ "tx_pon_omci_frames",		1, CA_ST_TX_U64, S_TX(pon_enq) },
	{ "tx_pon_omci_fail",		1, CA_ST_TX_U64, S_TX(pon_fail) },
	{ "tx_pon_wan_frames",		1, CA_ST_TX_U64, S_TX(pon_data_enq) },
	{ "tx_vp%u_enqueued",	CA_NI_TX_NUM_VPS, CA_ST_TX_U64,
	  S_TX(txq[0].enq), sizeof(struct cortina_ni_txq) },
	{ "tx_vp%u_reclaimed",	CA_NI_TX_NUM_VPS, CA_ST_TX_U64,
	  S_TX(txq[0].reclaimed), sizeof(struct cortina_ni_txq) },

	/* per-port MAC RX MIB -- dev/MEASURED-cortina-ni-ethtool.c.md sec 5. */
	{ "port%u_mac_rx_unicast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_MIB,
	  CA_NI_MIB_RX_UC_PKT },
	{ "port%u_mac_rx_multicast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_MIB,
	  CA_NI_MIB_RX_MC_PKT },
	{ "port%u_mac_rx_broadcast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_MIB,
	  CA_NI_MIB_RX_BC_PKT },

	/* ---- engine, read-and-clear (accumulated; see the header) -------- */
	{ "l3fe_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_L3FE_RX },
	{ "l3qm_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_L3QM_RX },
	{ "l3qm_tx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_L3QM_TX },
	{ "mce_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_MCE_RX },
	{ "dma_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_DMA_RX },

	/* engine, cumulative registers -- dev/MEASURED-cortina-ni-ethtool.c.md sec 6. */
	{ "l2fe_ni_ingress_drops",	1, CA_ST_NI_REG,
	  CA_NI_L2FE_NI_INTF_DROP_CNT },
	{ "l2fe_dos_flood_drops",	1, CA_ST_NI_REG,
	  CA_NI_L2FE_DOS_FLOOD_CNT },
#if IS_ENABLED(CONFIG_GPON_FLOW_DIAG)
	{ "l2fe_drop_reason_0", 1, CA_ST_L2_DROP, 0 },
	{ "l2fe_drop_ipv4_checksum", 1, CA_ST_L2_DROP, 1 },
	{ "l2fe_drop_dpid_blackhole", 1, CA_ST_L2_DROP, 2 },
	{ "l2fe_drop_ingress_stp", 1, CA_ST_L2_DROP, 3 },
	{ "l2fe_drop_reason_4", 1, CA_ST_L2_DROP, 4 },
	{ "l2fe_drop_vlan_type", 1, CA_ST_L2_DROP, 5 },
	{ "l2fe_drop_ingress_rule", 1, CA_ST_L2_DROP, 6 },
	{ "l2fe_drop_vid4095", 1, CA_ST_L2_DROP, 7 },
	{ "l2fe_drop_unknown_vlan", 1, CA_ST_L2_DROP, 8 },
	{ "l2fe_drop_unicast_mc_vlan", 1, CA_ST_L2_DROP, 9 },
	{ "l2fe_drop_destination_deny", 1, CA_ST_L2_DROP, 10 },
	{ "l2fe_drop_invalid_source", 1, CA_ST_L2_DROP, 11 },
	{ "l2fe_drop_learning_error", 1, CA_ST_L2_DROP, 12 },
	{ "l2fe_drop_source_deny", 1, CA_ST_L2_DROP, 13 },
	{ "l2fe_drop_source_learning", 1, CA_ST_L2_DROP, 14 },
	{ "l2fe_drop_unknown_type", 1, CA_ST_L2_DROP, 15 },
	{ "l2fe_drop_ingress_vlan", 1, CA_ST_L2_DROP, 16 },
	{ "l2fe_drop_port_membership", 1, CA_ST_L2_DROP, 17 },
	{ "l2fe_drop_egress_vlan", 1, CA_ST_L2_DROP, 18 },
	{ "l2fe_drop_reason_19", 1, CA_ST_L2_DROP, 19 },
	{ "l2fe_drop_rule", 1, CA_ST_L2_DROP, 20 },
	{ "l2fe_drop_loopback", 1, CA_ST_L2_DROP, 21 },
	{ "l2fe_drop_egress_stp", 1, CA_ST_L2_DROP, 22 },
	{ "l2fe_drop_reason_23", 1, CA_ST_L2_DROP, 23 },
	{ "l2fe_drop_reason_24", 1, CA_ST_L2_DROP, 24 },
	{ "l2fe_drop_reason_25", 1, CA_ST_L2_DROP, 25 },
	{ "l2fe_drop_blackhole", 1, CA_ST_L2_DROP, 26 },
	{ "l2fe_drop_loopback_filter", 1, CA_ST_L2_DROP, 27 },
	{ "l2fe_drop_ttl_zero", 1, CA_ST_L2_DROP, 28 },
	{ "l2fe_drop_dos", 1, CA_ST_L2_DROP, 29 },
	{ "l2fe_drop_reason_30", 1, CA_ST_L2_DROP, 30 },
	{ "l2fe_drop_reason_31", 1, CA_ST_L2_DROP, 31 },
#endif
	{ "l2tm_bm_rx_packets",		1, CA_ST_NI_REG, CA_NI_L2TM_BM_RX_PCNT },
	{ "l2tm_bm_tx_packets",		1, CA_ST_NI_REG, CA_NI_L2TM_BM_TX_PCNT },
	{ "l2tm_bm_drop_shared_buffer",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_SB_DPCNT },
	{ "l2tm_bm_drop_header",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_HDR_DPCNT },
	{ "l2tm_bm_drop_threshold",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_TE_DPCNT },
	{ "l2tm_bm_drop_error",		1, CA_ST_NI_REG, CA_NI_L2TM_BM_ERR_DPCNT },
	{ "l2tm_bm_drop_enqueue",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_RX_DPCNT },
	{ "l2tm_bm_drop_no_buffer",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_NOBUF_DPCNT },
	{ "qm_rx_packets",		1, CA_ST_NI_REG, CA_NI_QM_RX_CNTR },
	{ "qm_tx_packets",		1, CA_ST_NI_REG, CA_NI_QM_TX_CNTR },
	{ "qm_drop_no_buffer",		1, CA_ST_NI_REG, CA_NI_QM_RMU_NO_BUF_DROP },
	{ "qm_drop_fe",			1, CA_ST_NI_REG, CA_NI_QM_RMU_FE_DROP },
	{ "qm_drop_rx_eop",		1, CA_ST_NI_REG, CA_NI_QM_RX_EOP_DROP_CNTR },
	{ "qm_drop_rx_len_err",		1, CA_ST_NI_REG, CA_NI_QM_RX_LEN_ERR_CNTR },
	{ "qm_drop_rx_l2te",		1, CA_ST_NI_REG, CA_NI_QM_RX_L2TE_DROP_CNTR },

	/* engine GAUGES -- dev/MEASURED-cortina-ni-ethtool.c.md sec 7. */
	{ "qm_free_pages",	1, CA_ST_NI_REG, CA_NI_L2TM_QM_EQ_GLB_FREECNT },
	{ "qm_interrupt_source", 1, CA_ST_NI_REG, CA_NI_QM_INT_SRC },

	/* The CPU port's two empty-buffer pools, as the RAW PA_REQ ...
	 * dev/MEASURED-cortina-ni-ethtool.c.md sec 8. */
	{ "rx_cpu_pool%u_pa_req",	2, CA_ST_NI_REG,
	  CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID), 4 },

	/* The two packed NI_HV error words, published AS WORDS. Each ...
	 * dev/MEASURED-cortina-ni-ethtool.c.md sec 9. */
	{ "rx_missing_sop_eop_word",	1, CA_ST_NI_REG,
	  CA_NI_NI_L3QM_RX_MISS_SOP_EOP },
	{ "rx_short_err_word",		1, CA_ST_NI_REG,
	  CA_NI_NI_L3QM_RX_SHORT_ERR },

	/* ---- indirect / derived / MDIO: what -d cannot carry -------------- */
	{ "rx_epp_wrptr_voq%u",	CA_NI_RX_VOQ_COUNT, CA_ST_EPP_WRPTR },
	{ "cb_voq_used_pages_total",	1, CA_ST_CB_OCC, CA_ST_CB_TOTAL },
	{ "cb_voq_used_pages_max",	1, CA_ST_CB_OCC, CA_ST_CB_MAX },
	{ "cb_voq_nonzero_count",	1, CA_ST_CB_OCC, CA_ST_CB_NONZERO },
	{ "cb_lan_port_free_word",	1, CA_ST_CB_PORT_FREE,
	  CA_NI_RX_CB_PORT_LAN },
	{ "cb_cpu_port_free_word",	1, CA_ST_CB_PORT_FREE,
	  CA_NI_RX_CB_PORT_CPU },
	/* ★ PER-PORT PHY LINK - which PRINTED socket the cable is ...
	 * dev/MEASURED-cortina-ni-ethtool.c.md sec 10. */
	{ "port%u_phy_link",	CA_NI_LAN_PORT_COUNT, CA_ST_PHY_LINK },

#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	/* L3FE flow offload -- dev/MEASURED-cortina-ni-ethtool.c.md sec 12. */
	{ "l3fe_flows_resident",	1, CA_ST_L3FE, CA_L3FE_FLOWS_RESIDENT },
	{ "l3fe_ds_flows_resident",	1, CA_ST_L3FE, CA_L3FE_DS_FLOWS_RESIDENT },
	{ "l3fe_hw_hits",		1, CA_ST_L3FE, CA_L3FE_HW_HITS },
	{ "l3fe_us_hits",		1, CA_ST_L3FE, CA_L3FE_US_HITS },
	{ "l3fe_ds_hits",		1, CA_ST_L3FE, CA_L3FE_DS_HITS },
	{ "l3fe_hits_unattributed",	1, CA_ST_L3FE, CA_L3FE_HITS_UNATTRIBUTED },
	{ "l3fe_pppoe_us_hits",		1, CA_ST_L3FE, CA_L3FE_PPPOE_US_HITS },
	{ "l3fe_pppoe_ds_hits",		1, CA_ST_L3FE, CA_L3FE_PPPOE_DS_HITS },
	{ "l3fe_flows_refused",		1, CA_ST_L3FE, CA_L3FE_FLOWS_REFUSED },
	{ "l3fe_refused_unsupported",	1, CA_ST_L3FE, CA_L3FE_REFUSED_UNSUPPORTED },
	{ "l3fe_refused_table_full",	1, CA_ST_L3FE, CA_L3FE_REFUSED_TABLE_FULL },
	{ "l3fe_refused_duplicate",	1, CA_ST_L3FE, CA_L3FE_REFUSED_DUPLICATE },
	{ "l3fe_refused_error",		1, CA_ST_L3FE, CA_L3FE_REFUSED_ERROR },
	{ "l3fe_vlan_wan_refused_us",	1, CA_ST_L3FE, CA_L3FE_VLAN_WAN_REFUSED_US },
	{ "l3fe_vlan_wan_refused_ds",	1, CA_ST_L3FE, CA_L3FE_VLAN_WAN_REFUSED_DS },
	{ "l3fe_vlan_pppoe_programmed",	1, CA_ST_L3FE, CA_L3FE_VLAN_PPPOE_PROGRAMMED },
	{ "l3fe_vlan_pppoe_readback_fail", 1, CA_ST_L3FE,
	  CA_L3FE_VLAN_PPPOE_READBACK_FAIL },
	{ "l3fe_vlan_push_legs",	1, CA_ST_L3FE, CA_L3FE_VLAN_PUSH_LEGS },
	{ "l3fe_vlan_strip_legs",	1, CA_ST_L3FE, CA_L3FE_VLAN_STRIP_LEGS },
#endif
};

/* One sample of everything a single `ethtool -S` needs from a shared source,
 * so a read-and-clear register is sampled once per invocation and not once per
 * row that mentions it. */
struct ca_ni_stat_ctx {
	u64	nihv[CA_NI_NIHV_CNT_COUNT];
	/* The central-buffer scan is 128 indirect ACCESS/DATA transactions and
	 * all three aggregates come out of ONE walk, so it is sampled once per
	 * `ethtool -S` like the read-and-clear block above - not once per row
	 * that mentions it. */
	u64	cb_occ[3];
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	u64	l3fe[CA_L3FE_STAT_COUNT];
#endif
};

/* CA_NI_RX_EQ_ID / _ID2 are the CPU port's two empty-buffer ...
 * dev/MEASURED-cortina-ni-ethtool.c.md sec 13. */
static_assert(CA_NI_RX_EQ_ID2 == CA_NI_RX_EQ_ID + 1,
	      "rx_cpu_pool%u_pa_req reads the two pools as one stride-4 family");

static u64 ca_ni_stat_value(struct cortina_ni *ni,
			    const struct ca_ni_stat_grp *g, unsigned int i,
			    const struct ca_ni_stat_ctx *ctx)
{
	switch (g->src) {
	case CA_ST_RX_U64:
		if (!ni->rx)
			return 0;	/* rx_datapath_up says why */
		return *(const u64 *)((const u8 *)ni->rx + g->arg +
				      (size_t)i * g->step);
	case CA_ST_TX_U64:
		if (!ni->tx)
			return 0;
		return *(const u64 *)((const u8 *)ni->tx + g->arg +
				      (size_t)i * g->step);
	case CA_ST_NI_REG:
		if (!ni_base(ni))
			return 0;
		return readl(ni_base(ni) + g->arg + (size_t)i * g->step);
	case CA_ST_NIHV:
		return ctx->nihv[g->arg + i];
#if IS_ENABLED(CONFIG_GPON_FLOW_DIAG)
	case CA_ST_L2_DROP:
		return cortina_ni_l2_drop_read(ni, g->arg + i);
#endif
	case CA_ST_PORT_MIB:
		if (!ni_base(ni))
			return 0;
		/* ~0u on a stuck indirect access, so a broken instrument is
		 * visible instead of reading as a silent zero */
		return cortina_ni_rx_mib_read(ni, i, g->arg);
	case CA_ST_DRV_FLAG:
		switch (g->arg) {
		case CA_ST_FLAG_RX_UP:
			return ni->rx ? 1 : 0;
		}
		return 0;
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	case CA_ST_L3FE:
		return ctx->l3fe[g->arg + i];
#endif
	case CA_ST_EPP_WRPTR:
		return cortina_ni_rx_epp_wrptr(ni, i);
	case CA_ST_CB_OCC:
		return ctx->cb_occ[g->arg];
	case CA_ST_CB_PORT_FREE:
		return cortina_ni_rx_cb_port_free_word(ni, g->arg);
	case CA_ST_PHY_LINK: {
		int up = cortina_ni_rx_phy_link(ni, i);

		/* ~0ULL, not 0: "the MDIO read did not complete" and "this
		 * socket has no cable" are different answers, and reporting
		 * the second when the first happened is a sentence about the
		 * device manufactured by a fault of the instrument. */
		return up < 0 ? ~0ULL : (u64)up;
	}
	default:
		return 0;
	}
}

/* ------------------------------------------------------------------ */
/* ethtool ops                                                          */
/* ------------------------------------------------------------------ */

static struct cortina_ni *cortina_ni_of_netdev(struct net_device *dev)
{
	return *(struct cortina_ni **)netdev_priv(dev);
}

static int cortina_ni_get_sset_count(struct net_device *dev, int sset)
{
	unsigned int i, n = 0;

	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	for (i = 0; i < ARRAY_SIZE(cortina_ni_stat_grps); i++)
		n += cortina_ni_stat_grps[i].n;
	return n;
}

static void cortina_ni_get_strings(struct net_device *dev, u32 sset, u8 *data)
{
	unsigned int g, i;

	if (sset != ETH_SS_STATS)
		return;

	for (g = 0; g < ARRAY_SIZE(cortina_ni_stat_grps); g++) {
		const struct ca_ni_stat_grp *grp = &cortina_ni_stat_grps[g];

		for (i = 0; i < grp->n; i++) {
			/* a %u-less format simply ignores the argument, so the
			 * single-member and the family rows share one path */
			snprintf((char *)data, ETH_GSTRING_LEN, grp->fmt, i);
			data += ETH_GSTRING_LEN;
		}
	}
}

static void cortina_ni_get_ethtool_stats(struct net_device *dev,
					 struct ethtool_stats *stats, u64 *data)
{
	struct cortina_ni *ni = cortina_ni_of_netdev(dev);
	struct ca_ni_stat_ctx ctx;
	unsigned int g, i, n = 0;

	/* ONE sample of every shared source, before the walk */
	cortina_ni_nihv_sample(ni, ctx.nihv);
	cortina_ni_rx_cb_occupancy(ni, &ctx.cb_occ[CA_ST_CB_TOTAL],
				   &ctx.cb_occ[CA_ST_CB_MAX],
				   &ctx.cb_occ[CA_ST_CB_NONZERO]);
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	cortina_ni_flowoffload_stats(ctx.l3fe);
#endif

	for (g = 0; g < ARRAY_SIZE(cortina_ni_stat_grps); g++) {
		const struct ca_ni_stat_grp *grp = &cortina_ni_stat_grps[g];

		for (i = 0; i < grp->n; i++)
			data[n++] = ca_ni_stat_value(ni, grp, i, &ctx);
	}
}

/* `ethtool -d`: the curated NI-window register snapshot, as a ...
 * dev/MEASURED-cortina-ni-ethtool.c.md sec 11. */
#define CA_NI_REGDUMP_VERSION	1

static int cortina_ni_get_regs_len(struct net_device *dev)
{
	return (int)(cortina_ni_regdump_len() * sizeof(u32));
}

static void cortina_ni_get_regs(struct net_device *dev,
				struct ethtool_regs *regs, void *p)
{
	regs->version = CA_NI_REGDUMP_VERSION;
	cortina_ni_regdump_fill(cortina_ni_of_netdev(dev), p);
}

static void cortina_ni_get_drvinfo(struct net_device *dev,
				   struct ethtool_drvinfo *info)
{
	strscpy(info->driver, CA_NI_DRV_NAME, sizeof(info->driver));
	strscpy(info->bus_info, dev_name(cortina_ni_of_netdev(dev)->dev),
		sizeof(info->bus_info));
}

const struct ethtool_ops cortina_ni_ethtool_ops = {
	.get_drvinfo		= cortina_ni_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.get_sset_count		= cortina_ni_get_sset_count,
	.get_strings		= cortina_ni_get_strings,
	.get_ethtool_stats	= cortina_ni_get_ethtool_stats,
	.get_regs_len		= cortina_ni_get_regs_len,
	.get_regs		= cortina_ni_get_regs,
};
