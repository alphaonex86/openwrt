/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cortina-Access NI Ethernet driver for the Realtek RTL9607F "Elnath" -
 * shared declarations between the core (probe/MDIO) and the TX datapath.
 */

#ifndef _CORTINA_NI_H
#define _CORTINA_NI_H

#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "cortina-ni-regs.h"

/* platform driver name; also what `ethtool -i` reports */
#define CA_NI_DRV_NAME		"cortina-ni"

/* peek "window" selector for the peri block (not a DT window index) */
#define CA_NI_PEEK_PERI		0xff
#define CA_NI_PEEK_MAX		64	/* max 32-bit words per peek */
#define CA_NI_GSRAM_MAX		1024	/* max SRAM words per /proc/gsram dump */

struct mii_bus;
struct dentry;
/* forward-declared HERE, before the first prototype that takes one: the full
 * definition is further down, and a struct first named inside a parameter list
 * is a DIFFERENT type scoped to that declaration */
struct cortina_ni;
struct seq_file;

/* ★★ THE NI_HV COUNTER BLOCK IS READ-AND-CLEAR: IT HAS ...
 * dev/MEASURED-cortina-ni.h.md sec 1. */
enum cortina_ni_nihv_cnt {
	CA_NI_NIHV_L3FE_RX,		/* 0xa9bc NI_HV iface +0x00 RX_PKT_CNT */
	CA_NI_NIHV_L3QM_RX,		/* 0xa9fc NI_HV iface +0x40 RX_PKT_CNT */
	CA_NI_NIHV_L3QM_TX,		/* 0xaa10 NI_HV L3QM TX_PKT_CNT       */
	CA_NI_NIHV_MCE_RX,		/* 0xaa3c NI_HV iface +0x80 RX_PKT_CNT */
	CA_NI_NIHV_DMA_RX,		/* 0xaa7c NI_HV iface +0xc0 RX_PKT_CNT */
	CA_NI_NIHV_CNT_COUNT,
};

/* Sample every NI_HV read-and-clear counter ONCE, fold it ...
 * dev/MEASURED-cortina-ni.h.md sec 29. */
void cortina_ni_nihv_sample(struct cortina_ni *ni,
			    u64 out[CA_NI_NIHV_CNT_COUNT]);

/* The curated NI-window register snapshot, published through ...
 * dev/MEASURED-cortina-ni.h.md sec 2. */
unsigned int cortina_ni_regdump_len(void);
void cortina_ni_regdump_fill(struct cortina_ni *ni, u32 *buf);
/* name + NI-window offset of dump word @i, so the opaque `ethtool -d` blob can
 * be decoded (surfaced as debugfs .../regdump_map). */
void cortina_ni_regdump_entry(unsigned int i, const char **name, u32 *off);

/* Read one per-port NI RX MIB counter through the indirect ...
 * dev/MEASURED-cortina-ni.h.md sec 3. */
u32 cortina_ni_rx_mib_read(struct cortina_ni *ni, u32 port, u32 cnt_id);

/* The values `ethtool -d` structurally cannot carry, so they ...
 * dev/MEASURED-cortina-ni.h.md sec 30. */
u32 cortina_ni_rx_epp_wrptr(struct cortina_ni *ni, unsigned int voq);
void cortina_ni_rx_cb_occupancy(struct cortina_ni *ni, u64 *total, u64 *max,
				u64 *nonzero);
u32 cortina_ni_rx_cb_port_free_word(struct cortina_ni *ni, unsigned int port);
/* -1 when the MDIO read failed: "could not ask" is not "no link". */
int cortina_ni_rx_phy_link(struct cortina_ni *ni, unsigned int port);

/* Is this switch port administratively LOCKED by the OLT?  Consulted by every
 * writer in this driver that would otherwise restore a port. */
bool cortina_ni_uni_port_locked(unsigned int port);

/* One RMW of a port's global config with the administrative power-down bit
 * decided inside the critical section, so a stale decision cannot undo a lock
 * that landed between the read and the write. */
void cortina_ni_uni_port_glb_rmw(struct cortina_ni *ni, unsigned int port,
				 u32 clr, u32 set, u32 down_bit);

/* Drive one UNI's administrative state. -> 0 applied, ... -- dev/MEASURED-cortina-ni.h.md sec 4. */
int cortina_ni_uni_admin_set(unsigned int port, bool locked);

/* Withdraw the published NI before devres frees it. */
/* Publish the NI for the paths that reach it without a handle.  Called ONCE,
 * from the real completed initialisation -- never from a restoring function,
 * which would re-open a publication teardown had closed. */
void cortina_ni_rx_publish(struct cortina_ni *ni);

void cortina_ni_rx_unpublish(void);

/* The hand-debugging narratives. They live beside the state ...
 * dev/MEASURED-cortina-ni.h.md sec 5. */
struct seq_file;
int cortina_ni_rx_debug_show(struct seq_file *m, void *v);
int cortina_ni_tx_debug_show(struct seq_file *m, void *v);
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
int cortina_ni_l3fe_debug_show(struct seq_file *m, void *v);
ssize_t cortina_ni_l3fe_debug_write(struct file *file, const char __user *ubuf,
				    size_t len, loff_t *ppos);
#endif

/* One DMA-LSO virtual port (VP); M2b uses TXQ 0 of each CPU VP only. */
struct cortina_ni_txq {
	u8		vp;		/* DMA-LSO VP index (CPU n -> VP n+2) */
	__le32		*desc;		/* coherent descriptor ring, 2 words/desc */
	dma_addr_t	desc_dma;
	u16		wptr;		/* next descriptor to fill (SW) */
	u16		finished;	/* oldest un-reclaimed descriptor */
	spinlock_t	lock;		/* xmit vs. reclaim-timer (both BH) */
	struct {
		struct sk_buff	*skb;
		dma_addr_t	addr;
		unsigned int	len;
		/* skb == NULL descriptor kinds: 0 = unused/hole, 1 = PON ...
		 * dev/MEASURED-cortina-ni.h.md sec 31. */
		u8		pon;
		/* 1 = an extra copy of a FLOODED eth0 frame: it points at the
		 * mapping owned by the LAST descriptor of the same burst, so
		 * there is nothing here to unmap or free.  Fits in the struct's
		 * existing tail padding - no extra memory. */
		u8		dup;
	} slot[CA_NI_TX_RING_SIZE];
	/* spy counters (project rule: dump/probe capability is first-class) */
	u64		enq;
	u64		reclaimed;
};

struct cortina_ni_tx {
	struct net_device	*netdev;
	struct phy_device	*phydev;
	struct cortina_ni_txq	txq[CA_NI_TX_NUM_VPS];
	struct timer_list	reclaim_timer;
	struct work_struct	announce_work;	/* gratuitous ARP on link-up */
	bool			announced;
	u64			drop_nomap;
	u64			drop_linearize;
	u64			drop_oversize;
	u64			tx_busy;
	u32			last_word1;	/* last descriptor word1 (spy) */

	/* CPU->LAN egress port binding (cortina-ni-tx.c): DA -> RJ45, learned
	 * from the ingress port of received frames.  One bucket = one atomically
	 * published u64 {mac[47:0], port[50:48], valid[51]}, so the RX learn path
	 * and the TX lookup need no lock.  512 bytes total. */
	u64			lan_fdb[64];
	u32			lan_link;	/* RJ45s with a PHY link, bit = port */
	u64			lan_hit;	/* frames sent to a learned port */
	u64			lan_flood;	/* frames flooded (BC/MC/unknown) */
	u64			lan_dup;	/* extra descriptors a flood cost */
	u64			lan_learn;	/* DA bindings installed/changed */
	u64			lan_flush;	/* table flushes (link set changed) */

	/* US PON control-frame (OMCI) TX: coherent scratch of
	 * CA_NI_PON_TX_SLOTS slots, each {16B DMA-LSO header block @0,
	 * frame @+32}, sent as a 2-descriptor HEADER_A chain on txq[0].
	 * pon_busy (slot bitmap) is guarded by txq[0].lock. */
	void			*pon_buf;
	dma_addr_t		pon_buf_dma;
	u32			pon_busy;	/* CA_NI_PON_TX_SLOTS-bit in-use bitmap */
	/* spy counters (project rule: dump/probe capability is first-class) */
	u64			pon_enq;
	u64			pon_fail;
	u64			pon_data_enq;	/* US WAN data frames enqueued */
	/* register_netdev() succeeded. The teardown action is registered BEFORE
	 * publication so it covers a failed publish, so it cannot assume it. */
	bool			netdev_registered;
};

struct cortina_ni;

/* one RX pool buffer: the skb whose ->data was pushed to the HW pool */
struct cortina_ni_rx_buf {
	struct sk_buff	*skb;
	dma_addr_t	addr;		/* mapped PA of skb->data (128B aligned) */
	u8		eqid;		/* which CPU pool (EQ13/EQ14) this slot feeds */
};

struct cortina_ni_rx_irqctx {
	struct cortina_ni	*ni;
	u8			idx;	/* DT interrupt index 0..7 */
};

/* ★★★ A pool buffer's USABLE PAYLOAD WINDOW is not its size. ...
 * dev/MEASURED-cortina-ni.h.md sec 6. */
#define CA_NI_RX_BUF_TAILROOM		(CA_NI_QM_PKT_BUF_TAIL_UNITS * 16)
#define CA_NI_RX_BUF_USABLE_END(bufsz)	((bufsz) - CA_NI_RX_BUF_TAILROOM)

/* Chain bounds. A malformed chain must cost a counter, never ...
 * dev/MEASURED-cortina-ni.h.md sec 7. */
#define CA_NI_RX_CHAIN_MAX_LEN		2048u
/* Segment cap. 8 is generous for the pools we configure (a ...
 * dev/MEASURED-cortina-ni.h.md sec 32. */
#define CA_NI_RX_CHAIN_MAX_SEGS		8u

/* Multi-buffer receive: the arithmetic of one in-flight ...
 * dev/MEASURED-cortina-ni.h.md sec 8. */
struct ca_ni_chain_state {
	u32	total;		/* payload bytes the SOP HEADER_A promised */
	u32	got;		/* payload bytes accounted so far */
	u16	segs;		/* descriptors consumed by this chain */
	bool	open;		/* a chain is in flight on this voq */
};

/* One in-flight chain, PER CPU-port VOQ.  Per-voq and not global: a frame's
 * descriptors are appended to one voq's FIFO in order, while the NAPI budget
 * can cut a chain in half - so the state must survive into the next poll
 * without the frames of the next voq we drain appending into it. */
struct cortina_ni_rx_chain {
	struct sk_buff			*skb;	/* NULL = nothing held */
	/* HEADER_A word 1 of the SOP buffer. The delivery decision ...
	 * dev/MEASURED-cortina-ni.h.md sec 33. */
	u32				hdra_lo;
	struct ca_ni_chain_state	st;
};

struct cortina_ni_rx {
	struct cortina_ni	*ni;
	struct net_device	*netdev;
	struct napi_struct	napi;
	__le64			*ring;		/* coherent EPP descriptor ring (8 voqs) */
	dma_addr_t		ring_dma;
	u32			rptr[CA_NI_RX_VOQ_COUNT];	/* SW read ptr per voq, byte offset */
	/* CPU-pool DRAM region (EQ5+EQ6, cpu_eq=0, HW self-populating): the RMU0
	 * admits a CPU-dest frame into a buffer here; NAPI reads it via the phys
	 * offset (bufPA - cpu_dram_dma) and the HW recycles the bid on the EPP
	 * read-pointer advance.  Mapped WC, so no per-frame map/sync. */
	void			*cpu_dram;
	dma_addr_t		cpu_dram_dma;
	/* legacy CPU-push bookkeeping, from before the CPU pools ...
	 * dev/MEASURED-cortina-ni.h.md sec 9. */
	struct cortina_ni_rx_buf buf[CA_NI_RX_POOL_SIZE];
	unsigned int		nbufs;		/* buffers live in the HW pool */
	bool			qm_up;		/* QM_PHY_PORT_STS.qm_init_done seen */
	struct cortina_ni_rx_irqctx irqctx[CA_NI_RX_NUM_IRQS];
	int			irq[CA_NI_RX_NUM_IRQS];	/* <0 = not mapped */
	/* GPHY fault poll + reinit (stock aal_internal_phy_recovery, 1 Hz) */
	struct delayed_work	recovery_work;
	u16			gphy_cal[CA_NI_GPHY_COUNT][CA_NI_RX_GPHY_CAL_REGS]; /* per-bank probe snapshot */
	bool			intf_done;	/* per-port GPHY->MAC interface established (once) */
	/* ★ RATE-BOUNDED, NEVER COUNT-CAPPED (2026-08-20). The ...
	 * dev/MEASURED-cortina-ni.h.md sec 10. */
	u64			bringup_ticks;	/* recovery ticks with the bring-up owed */
	u64			bringup_calls;	/* times the bring-up actually ran */
	/* spy counters (project rule: dump/probe capability is first-class) */
	u64			rearms;		/* link-up RX re-arms */
	u64			recoveries;	/* GPHY reinits fired */
	u32			last_fault;	/* last GPHY fault-latch read */
	u64			irq_hits[CA_NI_RX_NUM_IRQS];
	u64			polls;
	/* per-voq delivered frames: a single flow must land on ONE voq; two
	 * or more climbing during a unidirectional bench = the HW spreads the
	 * flow across voqs and the fixed 0..7 drain order can reorder it (the
	 * packet-order live check on the board) */
	u64			voq_frames[CA_NI_RX_VOQ_COUNT];
	u64			frames;
	u64			bytes;
	u64			swid_frames;	/* headerless (sw_id != 0) frames */
	u64			pon_frames;	/* DS PON control frames (0xfff1) handed to the GPON hook */
	u64			wan_frames;	/* DS PON data frames (lspid=PON) delivered to the WAN netdev */
	u64			wan_l3_frames;	/* HW-L3 miss-punt DS frames (lspid=L3_WAN) delivered to the WAN netdev */
	u64			drop_nosop;	/* descriptor without SOP */
	u64			drop_badpa;	/* PA not in our map */
	u64			drop_len;	/* bad frame length */
	/* drop_len's two causes, split so the buffer-window fix is measurable:
	 * a runt is a real bad frame, an oversize one is a frame that does not
	 * fit the buffer's usable window and needs the chain path (rx_chain). */
	u64			drop_runt;	/* len < ETH_HLEN */
	u64			drop_oversize;	/* off + len past the usable window */
	u64			drop_nobuf;	/* refill alloc failed */
	u64			slot_dead;	/* buffer lost (remap failed) */
	/* ---- multi-buffer receive (rx_chain).  One in-flight chain per voq,
	 * plus the malformation ledger: the hardware is not trusted to
	 * terminate a chain, so every way one can go wrong is counted and
	 * logged rather than assumed impossible. */
	struct cortina_ni_rx_chain chain[CA_NI_RX_VOQ_COUNT];
	u32			chain_rest_off;	/* payload offset in a non-first buffer */
	u64			chain_frames;	/* chains assembled and delivered */
	u64			chain_segs;	/* segments those frames consumed */
	u64			chain_max_segs;	/* deepest chain seen */
	u64			chain_abort;	/* partial frames freed (total) */
	u64			chain_reopen;	/* SOP arrived with a chain still open */
	u64			chain_orphan;	/* non-SOP segment, no chain open */
	u64			chain_badtotal;	/* SOP pkt_size out of range */
	u64			chain_toolong;	/* segment cap hit / segments overran total */
	u64			chain_short;	/* EOP with fewer bytes than promised */
	u64			chain_swid;	/* headerless format: geometry unknown */
	/* Per-segment descriptor pkt_size as REPORTED by the ...
	 * dev/MEASURED-cortina-ni.h.md sec 34. */
	u32			chain_dlen_seen[CA_NI_RX_CHAIN_MAX_SEGS];
	u32			chain_dlen_calc[CA_NI_RX_CHAIN_MAX_SEGS];
	u64			chain_dlen_diff;	/* how often the two disagreed */
	/* ★ RX-buffer ownership witnesses (see cpu_pool_push in ...
	 * dev/MEASURED-cortina-ni.h.md sec 11. */
	u64			stale_buf;
	u64			push_fail;
	/* frames delivered out of the hardware-managed DEEP-QUEUE pool (EQ12).
	 * Non-zero is the witness that that pool is populated and the deep-queue
	 * admission path is alive; the GPON downstream punt rides it. */
	u64			dq_frames;
	u64			last_desc;	/* last non-empty descriptor */
	u64			last_hdra;	/* last HEADER_A (host order) */
	/* ★ TEMPORARY DIAGNOSTIC (P3 crc_ntfy tap, rx_crc_tap gate - REVERT
	 * once the T2 hash divergence is pinned): the HW lookup CRC read from
	 * the last matching punted frame's HEADER_CPU meta (+0x48/+0x4C). */
	u64			tap_hits;	/* matching punted frames seen */
	u32			tap_crc32;	/* HEADER_CPU +0x48 (BE) last match */
	u16			tap_crc16;	/* HEADER_CPU +0x4c (BE16) last match */
	u8			tap_cpuflg;	/* HEADER_A cpu_flg of last match */
};

/* /proc/cortina_ni_peek query state (single-user debug tool) */
struct cortina_ni_peek {
	u8	win;		/* CA_NI_WIN_* index, or CA_NI_PEEK_PERI */
	u32	off;		/* byte offset within that window */
	u32	count;		/* number of 32-bit words, 1..CA_NI_PEEK_MAX */
};

/* /proc/cortina_ni_gsram query state (ours-vs-stock internal-GPHY SRAM diff) */
struct cortina_ni_gsram {
	u8	bank;		/* internal-PHY bank 0..CA_NI_GPHY_COUNT-1 */
	u16	start;		/* first SRAM word address */
	u16	count;		/* words to dump, 1..CA_NI_GSRAM_MAX */
};

struct cortina_ni {
	struct device		*dev;
	void __iomem		*win[CA_NI_WIN_COUNT];
	size_t			winsz[CA_NI_WIN_COUNT];	/* mapped size, 0 = absent */
	void __iomem		*peri;	/* hardcoded 4K block @0xf4329000 */
	struct cortina_ni_peek	peek;
	struct cortina_ni_gsram	gsram;
	struct mii_bus		*mii;
	/* per-internal-PHY page-select shadow (reg 0x1f is not a HW reg) */
	u16			gphy_page[CA_NI_GPHY_COUNT];
	/* internal-GPHY SRAM firmware applied, per bank (one-shot per boot) */
	bool			gphy_patched[CA_NI_GPHY_COUNT];
	struct cortina_ni_tx	*tx;
	struct cortina_ni_rx	*rx;
	/* NI_HV read-and-clear counter totals - see cortina_ni_nihv_sample().
	 * The lock is what makes "one reader" true when two files are cat'ed
	 * at the same moment: the sample and the fold are one critical section,
	 * so a count can be taken once and only once. */
	spinlock_t		nihv_lock;
	u64			nihv_total[CA_NI_NIHV_CNT_COUNT];
	/* debugfs root (the bounded arbitrary-offset peek + the ethtool -d
	 * decode map); NULL when debugfs is not built in */
	struct dentry		*dbgfs;
};

/* DS PON control-frame hand-off: the NI CPU-RX path ... -- dev/MEASURED-cortina-ni.h.md sec 12. */
typedef void (*cortina_ni_pon_rx_fn)(const u8 *pdu, unsigned int len);
void cortina_ni_pon_rx_hook_set(cortina_ni_pon_rx_fn fn);

/* US PON control-frame TX (the cortina-gpon responder calls ...
 * dev/MEASURED-cortina-ni.h.md sec 13. */
int cortina_ni_pon_tx(const u8 *pdu, unsigned int len);

/* DS PON DATA (WAN) delivery: frames whose RX HEADER_A.lspid ...
 * dev/MEASURED-cortina-ni.h.md sec 35. */
void cortina_ni_pon_wan_ndev_set(struct net_device *ndev);

/* Print BOTH directions' CPU-forward counters (US ... -- dev/MEASURED-cortina-ni.h.md sec 14. */
struct seq_file;
void cortina_ni_cpu_fwd_show(struct seq_file *m, struct cortina_ni *ni);

/* US PON DATA (WAN) TX (the GPON WAN netdev's ndo_start_xmit ...
 * dev/MEASURED-cortina-ni.h.md sec 15. */
netdev_tx_t cortina_ni_pon_data_tx(struct sk_buff *skb,
				   struct net_device *ndev);

int cortina_ni_tx_probe(struct cortina_ni *ni);
/*
 * Close the PON TX entry and take the netdev down WHILE the rx and l3e
 * contexts still exist. See cortina_ni_teardown() for why the order matters.
 */
void cortina_ni_tx_withdraw(struct cortina_ni *ni);
/*
 * Register the netdev and open the PON TX entry. Called ONLY once rx and the
 * l3e context exist, because both are reachable the instant this returns.
 */
int cortina_ni_tx_publish(struct cortina_ni *ni);
int cortina_ni_rx_probe(struct cortina_ni *ni);

/* CPU->LAN egress port binding (cortina-ni-tx.c), driven from ...
 * dev/MEASURED-cortina-ni.h.md sec 16. */
void cortina_ni_lan_tx_learn(struct cortina_ni *ni, const u8 *sa, u32 lspid);
void cortina_ni_lan_tx_link_set(struct cortina_ni *ni, u32 link);

/* Front-panel per-RJ45 link lamps (cortina-ni-leds.c), the ...
 * dev/MEASURED-cortina-ni.h.md sec 17. */
void cortina_ni_leds_probe(struct cortina_ni *ni);
void cortina_ni_leds_link_set(u32 link);

/* Program a static L2FE FDB entry {mac -> ldpid} and return ...
 * dev/MEASURED-cortina-ni.h.md sec 18. */
int cortina_ni_l2fe_fdb_add_idx(void __iomem *base, const u8 *mac, u32 ldpid);

/* LOOK UP {mac} in the L2FE FDB - no table write - and report ...
 * dev/MEASURED-cortina-ni.h.md sec 19. */
int cortina_ni_l2fe_fdb_lookup_idx(void __iomem *base, const u8 *mac,
				   u32 *ldpid_out);

/* L3FE main-hash flow engine (nf_flow_table HW offload backend
 * dev/MEASURED-cortina-ni.h.md sec 20. */
void cortina_ni_pon_data_set_tcont(u8 tcont);

#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
int cortina_ni_flowoffload_probe(struct cortina_ni *ni);
int cortina_ni_setup_tc(struct net_device *dev, enum tc_setup_type type,
			void *type_data);
void cortina_ni_flowoffload_exit(void);
/* Stop the periodic sweep and wait for it, while its context is still valid. */
void cortina_ni_flowoffload_quiesce(void);
/* true only when the hw_l3_fwd experiment is armed AND the L3FE engine init
 * succeeded; the GPON driver keys the DS data-GEM PDC route on it (LDPID
 * L3_WAN into the L3FE vs the proven CPU_0 + FE-bypass delivery). */
bool cortina_ni_hw_l3_fwd_active(void);
/* per-L3-interface T2 admission (CAM + LPB an-mask + pri-6 routed CLS rules,
 * cortina-l3fe.c); re-applied from the link-up cls_init re-run under the
 * hw_l3_fwd gate because the my-MAC/STG0 re-init rewrites the LPB words. */
int cortina_l3fe_intf_add(void __iomem *ne, const u8 *lan_mac);
/* LIVE PON data-path identity push (GPON -> offload backend): ...
 * dev/MEASURED-cortina-ni.h.md sec 21. */
void cortina_ni_gpon_data_path_set(u16 gem_id, u8 tcont_idx);
/* Steer the upstream DATA queue to a hw T-CONT at runtime. ...
 * dev/MEASURED-cortina-ni.h.md sec 22. */
void cortina_ni_gpon_ds_route_set(bool into_l3fe);
/* LIVE PPPoE WAN session push (offload backend): report the ...
 * dev/MEASURED-cortina-ni.h.md sec 23. */
int cortina_ni_wan_pppoe_session_set(u16 session);
/* ★ GAP-2 instrument: inspect a CPU-punted PPPoE session ...
 * dev/MEASURED-cortina-ni.h.md sec 24. */
extern bool cortina_ni_pppoe_punt_check;
#define cortina_ni_pppoe_punt_armed()	READ_ONCE(cortina_ni_pppoe_punt_check)
void cortina_ni_pppoe_punt_inspect(const u8 *f, unsigned int len);
/* refresh the backend's probe-time router-MAC shadow when the netdev MAC
 * changes (the HW consumers - FDB/comparator/FIELD-CAM - are re-programmed
 * by cortina_ni_rx_mac_rearm, which is the only caller) */
void cortina_ni_flowoffload_router_mac_set(const u8 *mac);
/* The offload engine's countable quantities, for `ethtool ...
 * dev/MEASURED-cortina-ni.h.md sec 25. */
enum cortina_ni_l3fe_stat {
	CA_L3FE_FLOWS_RESIDENT,		/* GAUGE: entries currently in silicon */
	CA_L3FE_DS_FLOWS_RESIDENT,	/* GAUGE: the DS subset of the above   */
	CA_L3FE_HW_HITS,
	CA_L3FE_US_HITS,
	CA_L3FE_DS_HITS,
	CA_L3FE_HITS_UNATTRIBUTED,
	CA_L3FE_PPPOE_US_HITS,
	CA_L3FE_PPPOE_DS_HITS,
	CA_L3FE_FLOWS_REFUSED,
	CA_L3FE_REFUSED_UNSUPPORTED,
	CA_L3FE_REFUSED_TABLE_FULL,
	CA_L3FE_REFUSED_DUPLICATE,
	CA_L3FE_REFUSED_ERROR,
	CA_L3FE_VLAN_WAN_REFUSED_US,
	CA_L3FE_VLAN_WAN_REFUSED_DS,
	CA_L3FE_VLAN_PPPOE_PROGRAMMED,
	CA_L3FE_VLAN_PPPOE_READBACK_FAIL,
	CA_L3FE_VLAN_PUSH_LEGS,
	CA_L3FE_VLAN_STRIP_LEGS,
	CA_L3FE_STAT_COUNT,
};
void cortina_ni_flowoffload_stats(u64 out[CA_L3FE_STAT_COUNT]);
#else
static inline int cortina_ni_flowoffload_probe(struct cortina_ni *ni)
{
	return 0;
}
static inline int cortina_ni_setup_tc(struct net_device *dev,
				      enum tc_setup_type type, void *type_data)
{
	return -EOPNOTSUPP;
}
static inline bool cortina_ni_hw_l3_fwd_active(void)
{
	return false;
}
static inline int cortina_l3fe_intf_add(void __iomem *ne, const u8 *lan_mac)
{
	return 0;
}
static inline void cortina_ni_gpon_data_path_set(u16 gem_id, u8 tcont_idx)
{
}
static inline void cortina_ni_gpon_ds_route_set(bool into_l3fe)
{
}
static inline int cortina_ni_wan_pppoe_session_set(u16 session)
{
	return -EOPNOTSUPP;
}
#define cortina_ni_pppoe_punt_armed()	false
static inline void cortina_ni_pppoe_punt_inspect(const u8 *f, unsigned int len)
{
}
/* ★ THE ONE STUB THAT WAS MISSING FROM THIS #else (added ...
 * dev/MEASURED-cortina-ni.h.md sec 26. */
static inline void cortina_ni_flowoffload_exit(void) { }
static inline void cortina_ni_flowoffload_quiesce(void) { }

static inline void cortina_ni_flowoffload_router_mac_set(const u8 *mac)
{
}
#endif
void cortina_ni_rx_open(struct cortina_ni *ni);
void cortina_ni_rx_stop(struct cortina_ni *ni);
void cortina_ni_rx_link_up(struct cortina_ni *ni);	/* phylib link-up hook */
/* re-key the MAC-keyed admission/offload tables (L2FE FDB, my-MAC comparator,
 * PP FIELD-CAM, offload router-MAC shadow) from the current dev_addr; called
 * from .ndo_set_mac_address (netifd applies the factory MAC after the last
 * link-up re-arm).  hw_l3_fwd-gated no-op otherwise. */
void cortina_ni_rx_mac_rearm(struct cortina_ni *ni);
/* internal-GPHY SRAM firmware patch + uC resume; called at link-up (the uC is
 * only held/writable then, not at probe) */
void cortina_ni_gphy_patch_and_resume(struct cortina_ni *ni);

/* The STANDARD counter + register-snapshot interface ... -- dev/MEASURED-cortina-ni.h.md sec 27. */
extern const struct ethtool_ops cortina_ni_ethtool_ops;

/* Bounded arbitrary-offset register peek/poke. Shared by the ...
 * dev/MEASURED-cortina-ni.h.md sec 28. */
struct cortina_ni_peek_hole {
	u8		win;		/* CA_NI_WIN_* */
	u32		first, last;	/* inclusive byte offsets */
	bool		read_faults;	/* true: reads fault too, not just writes */
	const char	*why;
};
/* @why receives the refusal reason; returns false when the access is refused */
bool cortina_ni_peek_access_ok(u8 win, u32 off, bool write, const char **why);
/* parse one "[win] <hex_off> [count]" / "poke [win] <hex_off> <hex_val>"
 * command into @ni->peek, performing the poke when asked.  Returns 0 or a
 * negative errno; @buf is modified in place. */
int cortina_ni_peek_command(struct cortina_ni *ni, char *buf);
/* render the armed peek window */
void cortina_ni_peek_render(struct seq_file *m, struct cortina_ni *ni);
void cortina_ni_debugfs_init(struct cortina_ni *ni);

#endif /* _CORTINA_NI_H */
