// SPDX-License-Identifier: GPL-2.0
/* Cortina-Access NI Ethernet driver for the Realtek RTL9607F ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 1. */

#include <linux/bitfield.h>
#include "cortina_ni_rx_logic.h"	/* hoisted logic */
#include "cortina_ni_rx_geom.h"	/* RX pool + frame geometry, host-tested */
#include "cortina_l3fe_logic.h"	/* l3fe_wan_mac_derive(): the ONE WAN-MAC = LAN-MAC + 1 */
#include <linux/crc32.h>	/* ★ TEMP DIAG rx_frag_tap - revert with it */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>	/* ★ TEMP DIAG rx_stack_tap - revert with it */
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/unaligned.h>
#include <net/net_namespace.h>

#include "cortina-access.h"	/* the ONE indirect transaction */
#include "cortina-ni.h"

/* DIAGNOSTIC (temporary): when set, link_up skips ALL port-MAC/GPHY reconfig
 * (wrap/static_cfg/autosync/GLB/RXMAC) and does ONLY the CPU delivery chain -
 * leaving the GPHY<->MAC datapath exactly as U-Boot (working) left it.  Tests
 * whether our reconfig is the clobber.  Set via bootargs cortina_ni_rx.rx_skip_portcfg=1 */
static bool rx_skip_portcfg;
module_param(rx_skip_portcfg, bool, 0644);

#if IS_ENABLED(CONFIG_CORTINA_DEBUG)
static bool rx_debug;
module_param(rx_debug, bool, 0644);
MODULE_PARM_DESC(rx_debug, "dump the first received descriptors/frames");
#else
static const bool rx_debug;	/* CONFIG_CORTINA_DEBUG off: dead code */
#endif

/* ★ TEMPORARY DIAGNOSTIC (rx_frag_tap, 2026-07-27 - REVERT ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 2. */
static int rx_frag_tap;
module_param(rx_frag_tap, int, 0644);
MODULE_PARM_DESC(rx_frag_tap,
	"TEMP DIAG: log buffer identity + re-read checksum for the first N IPv4 fragments (0 = off)");

static int rx_frag_tap_us = 20;
module_param(rx_frag_tap_us, int, 0644);
MODULE_PARM_DESC(rx_frag_tap_us,
	"TEMP DIAG: delay before the rx_frag_tap re-read, microseconds (0 = control run)");

static void cortina_ni_rx_frag_tap(struct net_device *ndev, const u8 *buf,
				   u32 off, int len, u64 desc, u32 pa, u32 dlen,
				   u32 hdra_lo, const u8 *copied);

/* ★ TEMPORARY DIAGNOSTIC (rx_ds_tap, 2026-07-28 - REVERT with ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 3. */
static int rx_ds_tap;
module_param(rx_ds_tap, int, 0644);
MODULE_PARM_DESC(rx_ds_tap,
	"TEMP DIAG: log descriptor+pool identity for the first N GPON downstream frames (HEADER_A lspid=PON or ldpid=L3_WAN), sampled before the PON/WAN delivery branches (0 = off)");

static void cortina_ni_rx_ds_tap(struct net_device *ndev, const u8 *buf,
				 u32 off, int len, u64 desc, u32 pa, u32 dlen,
				 u32 hdra_hi, u32 hdra_lo);

/* ★ TEMPORARY DIAGNOSTIC (P3 crc_ntfy tap, 2026-07-23 - ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 4. */
static bool rx_crc_tap;
module_param(rx_crc_tap, bool, 0644);
MODULE_PARM_DESC(rx_crc_tap,
	"TEMP DIAG: log the HW lookup CRC (HEADER_CPU crc_ntfy) of punted UDP:19555 frames");

/* ★ TEMPORARY DIAGNOSTIC (P3 packet-STACK tap, 2026-07-23 - ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 5. */
static bool rx_stack_tap;
module_param(rx_stack_tap, bool, 0644);
MODULE_PARM_DESC(rx_stack_tap,
	"TEMP DIAG: dump SW frame-stack vs HW HDR_I parse for punted UDP:19555 frames");

/* ★ Decoupled datapath bring-up (default ON, 2026-07-22). The ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 6. */
static bool rx_decoupled_bringup = true;
module_param(rx_decoupled_bringup, bool, 0644);
MODULE_PARM_DESC(rx_decoupled_bringup,
		 "drive the datapath bring-up from the 1Hz poll so LAN ports come up without an eth0 cable (default on)");

/* ★ build88: A/B which CPU-EPP ring NAPI reads descriptors from - 0 = the LOW ring at
 * PADDR(0x7200)=0x0bc48000 (default/current); 1 = the HIGH ring at PADDR_HI(0x7220)=
 * 0x0bc4a000.  The engine advances wptr but the LOW ring reads poison, so the engine
 * may write descriptors into the HIGH ring; set rx_ring_hi=1 to test reading it. */
static bool rx_ring_hi;
module_param(rx_ring_hi, bool, 0644);
MODULE_PARM_DESC(rx_ring_hi, "NAPI reads the HIGH (PADDR_HI 0x0bc4a000) CPU-EPP ring instead of the LOW one");

/* ★ FBM pool ENABLE/FILL/PRELOAD gate - DEFAULT OFF because ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 7. */
static bool fbm_enable;
module_param(fbm_enable, bool, 0444);
MODULE_PARM_DESC(fbm_enable, "Boot-only: enable+fill+preload the FBM pool (DANGER: crashes until reserved-mem+feed fixed)");

/* (build27's qm_reset dropped: 0x6988 bit30=1 proved the QM ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 8. */
static u8 redir_cpu_ldpid;	/* 0 = stock (no redir; DFT_FWD->L3_LAN->L3FE->CLS trap delivers the CPU copy) */
module_param(redir_cpu_ldpid, byte, 0444);
MODULE_PARM_DESC(redir_cpu_ldpid, "Boot-only: REDIR_LDPID[0x19] destination (0=stock; 0x10=CPU unicast redirect)");

/* ★★ ARB_CTRL.dbuf_dpid = the deep_q TRIGGER (bits[7:4]; a ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 9. */
static u8 arb_dbuf_dpid = 0x08;	/* build68 STOCK: ARB_CTRL 0x89c71c82 (bits[7:4]=8); 0xf gave 0x89c71cf2 */
module_param(arb_dbuf_dpid, byte, 0444);
MODULE_PARM_DESC(arb_dbuf_dpid, "Boot-only: ARB_CTRL deep_q trigger pdpid (0xf=normal path, 8=stock deep_q)");

/* ★ build101 EXPERIMENT (prior-session never-tested fix): ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 161. */
static u8 pdpid_l3lan = 0x0d;	/* ★ STOCK L3_LAN->L3FE (entry fix); paired with the unconditional pool seed (drain fix) so frames enter L3FE AND RMU0 has buffers to admit them. */
module_param(pdpid_l3lan, byte, 0444);
MODULE_PARM_DESC(pdpid_l3lan, "Boot-only: PDPID_MAP[0x19] route (0=CPU-port0, 9=CPU/L2FE, 0xd=stock L3_LAN)");

/* ★★ The CPU-EPP descriptor-ring AXI attr. The coherent ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 10. */
static bool ring_noncoh = true;
module_param(ring_noncoh, bool, 0444);
MODULE_PARM_DESC(ring_noncoh, "Boot-only: CPU-EPP ring AXI attributes (1=non-coherent DDR_POOL, 0=coherent CPU_EPP)");

/* ★★★ RX-buffer OWNERSHIP. CFG2.cpu_eq (bit3) selects which ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 11. */
static bool cpu_pool_push = true;
module_param(cpu_pool_push, bool, 0444);
MODULE_PARM_DESC(cpu_pool_push,
	"CPU RX pool ownership: 1 = software-owned (cpu_eq=1 + CPU_PUSH_PADDR recycle, vendor model; fixes IPv4 fragment reassembly but currently breaks the WAN downstream punt), 0 = hardware-managed self-populating pool (default; collides a back-to-back frame pair onto one buffer)");

/* ★★ MULTI-BUFFER RECEIVE (the SOP..EOP descriptor chain). ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 12. */
static bool rx_chain;
module_param(rx_chain, bool, 0444);
MODULE_PARM_DESC(rx_chain,
	"assemble multi-buffer frames from the SOP..EOP descriptor chain (1) or treat a non-self-contained descriptor as today - deliver a SOP short, drop a continuation as nosop (0, default)");

/* Whether a CONTINUATION buffer repeats HEADER_A at +0x40, or ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 13. */
static bool rx_chain_rest_hdra;
module_param(rx_chain_rest_hdra, bool, 0444);
MODULE_PARM_DESC(rx_chain_rest_hdra,
	"a continuation buffer of a chain repeats HEADER_A at +0x40, so its payload starts at +0x48 (1), or its payload starts at +0x40 (0, default, and what the shipped firmware does)");

/* ★★ The seed count is deliberately NOT a constant of its ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 14. */
#define CA_NI_RX_PUSH_SEED_MIN		64
/* A full seed must fit inside the sub-region the PA->VA math maps, whatever a
 * future total_buf or buffer size is set to. */
static_assert(CA_NI_RX_EQ_TOTAL_BUF * CA_NI_RX_CPU_POOL0_BUFSZ <=
	      CA_NI_RX_CPU_POOL0_BYTES,
	      "CPU pool0: total_buf x buffer_size overflows its sub-region");
static_assert(CA_NI_RX_CPU_POOL0_BYTES +
	      CA_NI_RX_EQ2_TOTAL_BUF * CA_NI_RX_CPU_POOL1_BUFSZ <=
	      CA_NI_RX_CPU_DRAM_SIZE,
	      "CPU pool1: total_buf x buffer_size overflows the reserved region");
/* Per-poll recycle batch.  The NAPI budget is clamped to this so a poll can
 * never consume more buffers than we can hand back - an overflow would leak
 * buffers out of the pool until RX starves. */
#define CA_NI_RX_RECYCLE_MAX		64
/* ★★★ DEEP-QUEUE pool (EQ12), required by the software-owned ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 15. */
#define CA_NI_RX_DQ_POOL_OFF		CA_NI_RX_CPU_DRAM_SIZE
#define CA_NI_RX_DQ_POOL_PHYS \
	(CA_NI_RX_CPU_POOL_PHYS + CA_NI_RX_DQ_POOL_OFF)
/* One mapping covers both CPU pools AND the deep-queue pool, so the PA->VA
 * math and the bounds check in cortina_ni_rx_frame() need no special case. */
#define CA_NI_RX_MAP_SIZE \
	(CA_NI_RX_CPU_DRAM_SIZE + CA_NI_RX_DQ_DRAM_SIZE)
/* ★ NOT stock's CFG2 0x0000ff02. Buffer-size index 2 is 512B ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 16. */
#define CA_NI_RX_DQ_CFG2		0x0000ff04u
/* The EQ profile the deep queue gets.  prof_sel is a 4-bit DIRECT index, so any
 * of 0..15 is reachable; use 12 - stock's own deep-queue profile index, and the
 * only choice that collides with nothing this driver writes (profiles 0..7 are
 * filled wholesale by a loop in eq_init and 13 is the direct punt's). */
#define CA_NI_RX_DQ_PROFILE_SEL		CA_NI_RX_EQ12_PROFILE	/* = 12 */
/* the deep-queue pool is hardware-managed, so BOTH halves of the profile point
 * at it - there is no software-owned overflow reserve for this consumer */
#define CA_NI_RX_DQ_PROFILE_VAL \
	(FIELD_PREP(CA_NI_QM_EQ_PROF_EQP0, CA_NI_RX_EQ12_ID) | \
	 FIELD_PREP(CA_NI_QM_EQ_PROF_EQP1, CA_NI_RX_EQ12_ID))
static_assert(CA_NI_RX_EQ12_TOTAL_BUF * 2048u <= CA_NI_RX_DQ_DRAM_SIZE,
	      "deep-queue pool: total_buf x buffer_size overflows its region");

/* Ready-poll bound for the NAPI recycle path. Deliberately ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 162. */
#define CA_NI_RX_PUSH_TIMEOUT_NAPI_US	20

/* head_room_rest == head_room_first and tail_room_rest == ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 163. */
static_assert(CA_NI_QM_PKT_BUF_HEAD_UNITS * 16 == CA_NI_RX_BUF_HEADROOM,
	      "pkt-buf head_room does not put HEADER_A at CA_NI_RX_HDRA_OFF");
static_assert(CA_NI_RX_CHAIN_MAX_SEGS *
	      (CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL0_BUFSZ) -
	       CA_NI_RX_FRAME_OFF) >= CA_NI_RX_CHAIN_MAX_LEN,
	      "chain: MAX_SEGS cannot carry MAX_LEN out of a CPU-pool buffer");
/* The shipped firmware hardcodes this same window as the literal 1600 in both of
 * its receive polls; our derivation from the pkt-buf config must agree with it on
 * the 2048-byte pools, and unlike the literal it stays correct if a pool is ever
 * given a different buffer size - which is the point of the chain path. */
static_assert(CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL0_BUFSZ) -
	      CA_NI_RX_BUF_HEADROOM == 1600,
	      "CPU-pool usable window is not the firmware's 1600 bytes");

static inline void __iomem *ni_base(struct cortina_ni *ni)
{
	return ni->win[CA_NI_WIN_NI];
}

static inline void ni_rmw(struct cortina_ni *ni, u32 off, u32 clr, u32 set)
{
	writel((readl(ni_base(ni) + off) & ~clr) | set, ni_base(ni) + off);
}

/* ⚠ cortina_ni_rx_pool_eqid() WAS HERE AND IS DELETED ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 17. */
static int cortina_ni_rx_push_buf(struct cortina_ni *ni, u32 eqid, u32 pa,
				  unsigned int timeout_us)
{
	void __iomem *rdy = ni_base(ni) +
			    CA_NI_QM_CPU_PUSH_READY(CA_NI_RX_CPU_PORT);
	unsigned int i;

	for (i = 0; i < timeout_us; i++) {
		if (readl(rdy) & CA_NI_QM_PUSH_READY) {
			writel((pa & CA_NI_QM_PUSH_ADDR) |
			       FIELD_PREP(CA_NI_QM_PUSH_EQID, eqid),
			       ni_base(ni) +
			       CA_NI_QM_CPU_PUSH_PADDR(CA_NI_RX_CPU_PORT));
			return 0;
		}
		udelay(1);
	}
	return -EBUSY;
}

/* CPU-pool buffers (HW self-populating, copy-break) Under ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 18. */
static void cortina_ni_rx_irq_set(struct cortina_ni *ni, bool enable)
{
	writel(enable ? CA_NI_QM_EPP64_INT_EN0_STOCK
		      : CA_NI_QM_EPP64_INT_EN0_MASKED,
	       ni_base(ni) + CA_NI_QM_EPP64_INT_EN0);
}

static irqreturn_t cortina_ni_rx_isr(int irq, void *dev_id)
{
	struct cortina_ni_rx_irqctx *ctx = dev_id;
	struct cortina_ni *ni = ctx->ni;

	ni->rx->irq_hits[ctx->idx]++;

	/* mask (this is also the only ack - level by FIFO occupancy) */
	cortina_ni_rx_irq_set(ni, false);
	napi_schedule(&ni->rx->napi);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* NAPI poll                                                           */
/* ------------------------------------------------------------------ */

static u32 cortina_ni_rx_wptr_voq(struct cortina_ni *ni, unsigned int voq)
{
	u32 w = readl(ni_base(ni) +
		      CA_NI_QM_EPP64_WRPTR(CA_NI_RX_CPU_PORT, voq));

	/* byte offset, wraps at the per-voq ring size (stock count formula) */
	return (w & CA_NI_QM_EPP64_PTR) & (CA_NI_RX_RING_BYTES - 1);
}

static u32 cortina_ni_rx_wptr(struct cortina_ni *ni)
{
	return cortina_ni_rx_wptr_voq(ni, CA_NI_RX_VOQ);
}

/* DS PON control-frame consumer (the cortina-gpon driver ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 164. */
static cortina_ni_pon_rx_fn __rcu cortina_ni_pon_rx_cb;

/* ⚠ A BARE STORE IS NOT A WITHDRAWAL. This published NULL and ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 19. */
void cortina_ni_pon_rx_hook_set(cortina_ni_pon_rx_fn fn)
{
	rcu_assign_pointer(cortina_ni_pon_rx_cb, fn);
	if (!fn)
		synchronize_rcu();
}
EXPORT_SYMBOL_GPL(cortina_ni_pon_rx_hook_set);

/* DS PON DATA (WAN) consumer: de-encapsulated data-GEM frames ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 20. */
static struct net_device *cortina_ni_pon_wan_ndev;

void cortina_ni_pon_wan_ndev_set(struct net_device *ndev)
{
	WRITE_ONCE(cortina_ni_pon_wan_ndev, ndev);
}
EXPORT_SYMBOL_GPL(cortina_ni_pon_wan_ndev_set);

/* ★ TEMP DIAG rx_stack_tap implementation (REVERT with ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 21. */
#define CA_NI_L3FE_DBG_ADDR		0x30b8
#define CA_NI_L3FE_DBG_DATA		0x30bc
#define CA_NI_L3FE_DBG_TAP_T2IN		2
#define CA_NI_HDRI_WORDS		32

/* a_cut HDR_I bit offsets (LSB-first over the 128-byte ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 22. */
#define HDRI_L4_DP		74	/* 16b dest L4 port */
#define HDRI_L4_SP		90	/* 16b src L4 port */
#define HDRI_L3_TOTLEN		122	/* 14b IP total length */
#define HDRI_L3_CK_ERR		136	/*  1b l3_chksum_err */
#define HDRI_L3_CKSUM		137	/* 16b IP header checksum */
#define HDRI_IP_DA0		233	/* 32b v4 DA (LSW of the 128b field) */
#define HDRI_IP_SA0		361	/* 32b v4 SA (LSW of the 128b field) */
#define HDRI_IP_L4_TYPE		489	/*  3b 0=UDP 1=TCP .. */
#define HDRI_IP_PROTO		492	/*  8b IP protocol */
#define HDRI_IP_IHL		500	/*  4b IHL (words) */
#define HDRI_IP_VER		504	/*  1b 0=IPv4 1=IPv6 */
#define HDRI_IP_VLD		505	/*  1b parsed an IP header */
#define HDRI_PPP_PROTO_ENC	506	/*  4b ppp_protocol_enc */
#define HDRI_PPPOE_SESS		510	/* 16b pppoe_session_id */
#define HDRI_PPPOE_TYPE		526	/*  2b pppoe_type */
#define HDRI_INNER_1P		528	/*  3b inner 802.1p */
#define HDRI_TOP_1P		531	/*  3b top 802.1p */
#define HDRI_INNER_DEI		534	/*  1b */
#define HDRI_INNER_VID		535	/* 12b */
#define HDRI_INNER_TPID_ENC	547	/*  3b */
#define HDRI_TOP_DEI		550	/*  1b */
#define HDRI_TOP_VID		551	/* 12b */
#define HDRI_TOP_TPID_ENC	563	/*  3b */
#define HDRI_VLAN_CNT		566	/*  2b tags still ON the frame at T2 */
#define HDRI_ETYPE_ENC		585	/*  4b ethertype_enc */
#define HDRI_ETYPE		589	/* 16b ethertype (post-tag) */
#define HDRI_O_LSPID		816	/*  6b original lspid */
#define HDRI_LSPID		822	/*  6b lspid (post-LPB rewrite) */
#define HDRI_L4_OFFSET		897	/*  8b PE: L4 offset in the frame */
#define HDRI_L3_OFFSET		913	/*  8b PE: L3 offset in the frame */
#define HDRI_PKT_LEN		929	/* 14b PE: orig_packet_len */

/* SW-decoded frame layering (explicit byte math, endianness-agnostic) */
struct rx_stack_sw {
	u8	tags;			/* VLAN tags found (2 recorded) */
	u16	tpid[2], tci[2];	/* outermost first */
	u16	ethertype;		/* after the last tag */
	bool	pppoe;
	u16	pppoe_sess, ppp_proto;
	bool	ip, inner;		/* outer IPv4 seen / IP-in-IP seen */
	u8	ipver, ihl;		/* outer version, outer IHL bytes */
	u8	proto;			/* INNERMOST IPv4 protocol */
	u32	sa, da;			/* INNERMOST IPv4, host-order value */
	bool	l4;
	u16	sp, dp;			/* innermost L4 ports */
};

static bool rx_stack_sw_parse(const u8 *p, int len, struct rx_stack_sw *s)
{
	int pos = 12, ip_pos, i;
	u16 et = 0;

	memset(s, 0, sizeof(*s));
	if (len < 14)
		return false;
	for (i = 0; i < 3; i++) {	/* walk up to 3 stacked VLAN tags */
		if (pos + 4 > len)
			return false;
		et = ((u16)p[pos] << 8) | p[pos + 1];
		if (et != 0x8100 && et != 0x88a8 && et != 0x9100)
			break;
		if (s->tags < 2) {
			s->tpid[s->tags] = et;
			s->tci[s->tags] = ((u16)p[pos + 2] << 8) | p[pos + 3];
		}
		s->tags++;
		pos += 4;
	}
	s->ethertype = et;
	pos += 2;			/* now at the payload */

	if (et == 0x8863) {		/* PPPoE discovery: no IP inside */
		s->pppoe = true;
		return true;
	}
	if (et == 0x8864) {		/* PPPoE session */
		if (pos + 8 > len)
			return false;
		s->pppoe = true;
		s->pppoe_sess = ((u16)p[pos + 2] << 8) | p[pos + 3];
		s->ppp_proto = ((u16)p[pos + 6] << 8) | p[pos + 7];
		if (s->ppp_proto != 0x0021)	/* descend only into PPP-IPv4 */
			return true;
		pos += 8;
		et = 0x0800;
	}
	if (et != 0x0800)		/* v6/ARP/...: stack recorded, no v4 */
		return true;

	ip_pos = pos;
	if (ip_pos + 20 > len)
		return false;
	s->ip = true;
	s->ipver = p[ip_pos] >> 4;
	s->ihl = (p[ip_pos] & 0xf) * 4;
	s->proto = p[ip_pos + 9];
	s->sa = get_unaligned_be32(p + ip_pos + 12);
	s->da = get_unaligned_be32(p + ip_pos + 16);
	if (s->proto == 4) {		/* IPv4-in-IPv4 */
		ip_pos += s->ihl;
		if (ip_pos + 20 > len)
			return false;
		s->inner = true;
		s->proto = p[ip_pos + 9];
		s->sa = get_unaligned_be32(p + ip_pos + 12);
		s->da = get_unaligned_be32(p + ip_pos + 16);
	}
	/* L4 of the innermost IPv4 (proto 41 = 6in4: flagged, not descended) */
	pos = ip_pos + (p[ip_pos] & 0xf) * 4;
	if ((s->proto == 17 || s->proto == 6) && pos + 4 <= len) {
		s->l4 = true;
		s->sp = ((u16)p[pos] << 8) | p[pos + 1];
		s->dp = ((u16)p[pos + 2] << 8) | p[pos + 3];
	}
	return true;
}

static noinline void cortina_ni_rx_stack_tap(struct cortina_ni *ni,
					     const u8 *p, int len)
{
	static DEFINE_RATELIMIT_STATE(rs, 2 * HZ, 2);
	static unsigned int hits;
	struct net_device *ndev = ni->rx->netdev;
	u32 w[CA_NI_HDRI_WORDS];
	struct rx_stack_sw s;
	u32 hw_dp, hw_sa, hw_da, hw_vcnt, hw_vld;
	char fl[128];
	unsigned int i;
	int n = 0;

	if (!rx_stack_sw_parse(p, len, &s))
		return;
	/* the offload probe: innermost IPv4/UDP dport 19555, any layering */
	if (!s.ip || s.proto != 17 || !s.l4 || s.dp != 19555)
		return;
	if (!__ratelimit(&rs))
		return;
	hits++;

	/* (a) actual frame stack + anomaly flags vs plain {Eth->IPv4->UDP} */
	fl[0] = '\0';
	if (s.tags)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85VLAN %04x/vid=0x%03x NOT untagged%s",
			       s.tpid[0], s.tci[0] & 0xfff,
			       s.tags > 1 ? " +QinQ" : "");
	if (s.tags > 1)
		n += scnprintf(fl + n, sizeof(fl) - n, " inner %04x/vid=0x%03x",
			       s.tpid[1], s.tci[1] & 0xfff);
	if (s.pppoe)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85PPPoE sess=0x%04x", s.pppoe_sess);
	if (s.inner)
		n += scnprintf(fl + n, sizeof(fl) - n, " \xe2\x98\x85IP-over-IP");
	if (!n)
		scnprintf(fl, sizeof(fl), " plain Eth/IPv4/UDP (as expected)");
	netdev_info(ndev,
		    "stack_tap#%u SW : eth{da=%pM sa=%pM} tags=%u et=%04x ip{v%u ihl=%u proto=%u sa=%08x da=%08x} l4{sp=%u dp=%u} |%s\n",
		    hits, p, p + 6, s.tags, s.ethertype, s.ipver, s.ihl,
		    s.proto, s.sa, s.da, s.sp, s.dp, fl);

	/* (b) the HW's parse: HDR_I at the T2 (hash) input, via the debug mux */
	for (i = 0; i < CA_NI_HDRI_WORDS; i++) {
		writel((CA_NI_L3FE_DBG_TAP_T2IN << 5) | i,
		       ni_base(ni) + CA_NI_L3FE_DBG_ADDR);
		w[i] = readl(ni_base(ni) + CA_NI_L3FE_DBG_DATA);
	}
	hw_dp   = rx_hdri_get(w, HDRI_L4_DP, 16);
	hw_sa   = rx_hdri_get(w, HDRI_IP_SA0, 32);
	hw_da   = rx_hdri_get(w, HDRI_IP_DA0, 32);
	hw_vcnt = rx_hdri_get(w, HDRI_VLAN_CNT, 2);
	hw_vld  = rx_hdri_get(w, HDRI_IP_VLD, 1);
	netdev_info(ndev,
		    "stack_tap#%u HW : HDR_I@T2 lspid=%02lx(o=%02lx) et=%04lx(enc%lx) vlan_cnt=%u top{tpid%lx vid=0x%03lx p%lu d%lu} inner{tpid%lx vid=0x%03lx p%lu d%lu} pppoe{t%lu sess=0x%04lx enc%lx} ip{vld%u v%lu ihl=%lu proto=%lu l4t=%lu} sa=%08x da=%08x sp=%lu dp=%u l3{ck=%04lx err%lu len=%lu} pe_off{l3=%lu l4=%lu plen=%lu}\n",
		    hits,
		    (unsigned long)rx_hdri_get(w, HDRI_LSPID, 6),
		    (unsigned long)rx_hdri_get(w, HDRI_O_LSPID, 6),
		    (unsigned long)rx_hdri_get(w, HDRI_ETYPE, 16),
		    (unsigned long)rx_hdri_get(w, HDRI_ETYPE_ENC, 4),
		    hw_vcnt,
		    (unsigned long)rx_hdri_get(w, HDRI_TOP_TPID_ENC, 3),
		    (unsigned long)rx_hdri_get(w, HDRI_TOP_VID, 12),
		    (unsigned long)rx_hdri_get(w, HDRI_TOP_1P, 3),
		    (unsigned long)rx_hdri_get(w, HDRI_TOP_DEI, 1),
		    (unsigned long)rx_hdri_get(w, HDRI_INNER_TPID_ENC, 3),
		    (unsigned long)rx_hdri_get(w, HDRI_INNER_VID, 12),
		    (unsigned long)rx_hdri_get(w, HDRI_INNER_1P, 3),
		    (unsigned long)rx_hdri_get(w, HDRI_INNER_DEI, 1),
		    (unsigned long)rx_hdri_get(w, HDRI_PPPOE_TYPE, 2),
		    (unsigned long)rx_hdri_get(w, HDRI_PPPOE_SESS, 16),
		    (unsigned long)rx_hdri_get(w, HDRI_PPP_PROTO_ENC, 4),
		    hw_vld,
		    (unsigned long)rx_hdri_get(w, HDRI_IP_VER, 1),
		    (unsigned long)rx_hdri_get(w, HDRI_IP_IHL, 4),
		    (unsigned long)rx_hdri_get(w, HDRI_IP_PROTO, 8),
		    (unsigned long)rx_hdri_get(w, HDRI_IP_L4_TYPE, 3),
		    hw_sa, hw_da,
		    (unsigned long)rx_hdri_get(w, HDRI_L4_SP, 16),
		    hw_dp,
		    (unsigned long)rx_hdri_get(w, HDRI_L3_CKSUM, 16),
		    (unsigned long)rx_hdri_get(w, HDRI_L3_CK_ERR, 1),
		    (unsigned long)rx_hdri_get(w, HDRI_L3_TOTLEN, 14),
		    (unsigned long)rx_hdri_get(w, HDRI_L3_OFFSET, 8),
		    (unsigned long)rx_hdri_get(w, HDRI_L4_OFFSET, 8),
		    (unsigned long)rx_hdri_get(w, HDRI_PKT_LEN, 14));

	/* verdict: same frame?  (mux = LAST frame through STG1->T2) + diffs */
	n = 0;
	fl[0] = '\0';
	if (hw_vcnt != s.tags)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85vlan_cnt HW=%u vs frame=%u",
			       hw_vcnt, s.tags);
	if (hw_vcnt)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85tag still present at T2");
	if (!hw_vld)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85HW parsed NO IP header");
	if (!((hw_sa == s.sa || hw_sa == swab32(s.sa)) &&
	      (hw_da == s.da || hw_da == swab32(s.da))))
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85HW sa/da != frame sa/da");
	if (!n)
		scnprintf(fl, sizeof(fl), " HW parse == SW stack");
	netdev_info(ndev, "stack_tap#%u -->: %s;%s\n", hits,
		    (hw_dp == 19555 || hw_dp == swab16(19555)) ?
		    "same-frame" :
		    "\xe2\x98\x85STALE snapshot (another frame raced the mux)",
		    fl);

	/* raw words for offline decode of anything not printed above */
	if (hits <= 4)
		print_hex_dump(KERN_INFO, "stack_tap HDR_I: ",
			       DUMP_PREFIX_OFFSET, 16, 4, w, sizeof(w), false);
}

/* multi-buffer receive: the SOP..EOP descriptor chain Where a ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 23. */
static inline struct net_device *cortina_ni_rx_wan_dest(u32 hdra_lo, bool *is_l3)
{
	enum ca_ni_rx_wan_class cls =
		cortina_ni_rx_wan_class(FIELD_GET(CA_NI_HDRA_W1_LSPID,
						  hdra_lo));

	if (cls == CA_NI_RX_WAN_NONE)
		return NULL;
	/* the L3 class counts only while the HW-L3 path is armed - a LIVE
	 * read of the offload module's state, applied HERE and not inside
	 * the classifier so a LAN frame never pays for the call */
	if (cls == CA_NI_RX_WAN_L3 && !cortina_ni_hw_l3_fwd_active())
		return NULL;
	*is_l3 = (cls == CA_NI_RX_WAN_L3);
	return READ_ONCE(cortina_ni_pon_wan_ndev);
}

/* what one descriptor of a chain contributes */
struct ca_ni_chain_seg {
	u32	off;		/* payload offset inside this buffer */
	u32	len;		/* payload bytes to take from it */
	u32	dlen_expect;	/* what this descriptor's own pkt_size should read */
};

enum ca_ni_chain_act {
	CA_NI_CHAIN_OPEN,	/* SOP accepted, more segments expected */
	CA_NI_CHAIN_APPEND,	/* continuation accepted, more expected */
	CA_NI_CHAIN_DONE,	/* EOP accepted, the frame is complete */
	/* ---- malformations; the caller drops the partial frame for each ---- */
	CA_NI_CHAIN_ORPHAN,	/* continuation with no chain open on this voq */
	CA_NI_CHAIN_BADTOTAL,	/* SOP pkt_size out of range, or no room in the window */
	CA_NI_CHAIN_TOOLONG,	/* segment cap hit, or the segments overran total */
	CA_NI_CHAIN_SHORT,	/* EOP with fewer bytes than pkt_size promised */
};

/* The chain state machine, as pure arithmetic: no skb, no ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 24. */
static enum ca_ni_chain_act
ca_ni_chain_step(struct ca_ni_chain_state *st, bool sop, bool eop,
		 u32 hdra_hi, u32 hdra_lo, u32 buf_end, u32 rest_off,
		 struct ca_ni_chain_seg *out)
{
	u32 off, want, avail;

	out->off = out->len = out->dlen_expect = 0;

	if (sop) {
		int total = FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo);

		off = CA_NI_RX_FRAME_OFF;
		if (hdra_hi & CA_NI_HDRA_W0_CPU_FLG) {
			off += CA_NI_RX_HDR_CPU_LEN;
			total -= CA_NI_RX_HDR_CPU_LEN;
		}
		if (total < (int)ETH_HLEN ||
		    total > (int)CA_NI_RX_CHAIN_MAX_LEN || off >= buf_end)
			return CA_NI_CHAIN_BADTOTAL;
		st->total = total;
		st->got = 0;
		st->segs = 0;
		st->open = true;
	} else {
		if (!st->open)
			return CA_NI_CHAIN_ORPHAN;
		off = rest_off;
		if (off >= buf_end)
			return CA_NI_CHAIN_BADTOTAL;
	}

	if (++st->segs > CA_NI_RX_CHAIN_MAX_SEGS)
		return CA_NI_CHAIN_TOOLONG;

	avail = buf_end - off;
	want = st->total - st->got;
	out->off = off;
	out->len = min(want, avail);
	out->dlen_expect = (off - CA_NI_RX_HDRA_OFF) + out->len;
	st->got += out->len;

	if (eop) {
		if (st->got != st->total)
			return CA_NI_CHAIN_SHORT;
		/* the frame is complete, so this voq holds no chain any more. ...
		 * dev/MEASURED-cortina-ni-rx.c.md sec 25. */
		st->open = false;
		return CA_NI_CHAIN_DONE;
	}
	/* Not the last segment, so it must have FILLED its window: if ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 26. */
	if (out->len != avail)
		return CA_NI_CHAIN_TOOLONG;
	return sop ? CA_NI_CHAIN_OPEN : CA_NI_CHAIN_APPEND;
}

/* free a held partial frame WITHOUT touching the accounting (the SOP path has
 * already re-initialised it for the new chain) */
static void cortina_ni_rx_chain_free(struct cortina_ni_rx *rx,
				     struct cortina_ni_rx_chain *ch)
{
	if (!ch->skb)
		return;
	dev_kfree_skb_any(ch->skb);
	ch->skb = NULL;
	rx->chain_abort++;
}

/* abandon a chain entirely: free the partial frame and clear the accounting */
static void cortina_ni_rx_chain_reset(struct cortina_ni_rx *rx,
				      struct cortina_ni_rx_chain *ch)
{
	cortina_ni_rx_chain_free(rx, ch);
	memset(&ch->st, 0, sizeof(ch->st));
}

/* Consume one descriptor of a multi-buffer frame. Reached ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 27. */
static noinline u32 cortina_ni_rx_chain_seg(struct cortina_ni *ni,
					    unsigned int voq, u64 desc,
					    const u8 *buf, u32 buf_end, u32 rpa)
{
	struct cortina_ni_rx *rx = ni->rx;
	struct cortina_ni_rx_chain *ch = &rx->chain[voq];
	struct net_device *ndev = rx->netdev;
	u32 lo = lower_32_bits(desc);
	bool sop = !!(lo & CA_NI_RX_DESC_SOP);
	bool eop = !!(lo & CA_NI_RX_DESC_EOP);
	u32 dlen = FIELD_GET(CA_NI_RX_DESC_LEN, desc);
	u32 hdra_hi = 0, hdra_lo = 0;
	struct ca_ni_chain_seg seg;
	enum ca_ni_chain_act act;
	struct net_device *wan;
	bool is_l3 = false;

	/* The headerless (sw_id != 0) format's chain geometry is unknown - its
	 * frame starts at +0x10, inside the head_room the chain arithmetic is
	 * built on, and no such descriptor has ever been observed on this board.
	 * Refuse it rather than guess; a non-zero counter here is a finding. */
	if (unlikely(FIELD_GET(CA_NI_RX_DESC_SWID, desc))) {
		rx->chain_swid++;
		ndev->stats.rx_errors++;
		net_warn_ratelimited("%s: RX chain: headerless (sw_id) segment, geometry unknown - dropped\n",
				     netdev_name(ndev));
		return rpa;
	}

	if (sop) {
		hdra_hi = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF);
		hdra_lo = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF + 4);
		rx->last_hdra = ((u64)hdra_hi << 32) | hdra_lo;
		/* a chain still open here never got its EOP: drop the partial
		 * frame and let this SOP start over */
		if (unlikely(ch->st.open)) {
			cortina_ni_rx_chain_free(rx, ch);
			rx->chain_reopen++;
			ndev->stats.rx_errors++;
		}
		ch->hdra_lo = hdra_lo;
	}

	act = ca_ni_chain_step(&ch->st, sop, eop, hdra_hi, hdra_lo, buf_end,
			       rx->chain_rest_off, &seg);

	switch (act) {
	case CA_NI_CHAIN_OPEN:
	case CA_NI_CHAIN_APPEND:
	case CA_NI_CHAIN_DONE:
		break;
	case CA_NI_CHAIN_ORPHAN:
		rx->chain_orphan++;
		ndev->stats.rx_errors++;
		net_warn_ratelimited("%s: RX chain: continuation with no chain open on voq %u (desc %016llx)\n",
				     netdev_name(ndev), voq, desc);
		return rpa;
	case CA_NI_CHAIN_BADTOTAL:
		rx->chain_badtotal++;
		goto drop;
	case CA_NI_CHAIN_TOOLONG:
		rx->chain_toolong++;
		goto drop;
	case CA_NI_CHAIN_SHORT:
		rx->chain_short++;
		goto drop;
	}

	/* ★ The per-segment descriptor pkt_size is RECORDED, not ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 28. */
	if (ch->st.segs <= CA_NI_RX_CHAIN_MAX_SEGS) {
		rx->chain_dlen_seen[ch->st.segs - 1] = dlen;
		rx->chain_dlen_calc[ch->st.segs - 1] = seg.dlen_expect;
	}
	if (dlen != seg.dlen_expect)
		rx->chain_dlen_diff++;

	if (sop) {
		/* ONE allocation per frame, sized from the SOP's pkt_size and
		 * never grown - the arithmetic above cannot exceed it */
		ch->skb = napi_alloc_skb(&rx->napi, ch->st.total);
		if (unlikely(!ch->skb)) {
			rx->drop_nobuf++;
			ndev->stats.rx_dropped++;
			memset(&ch->st, 0, sizeof(ch->st));
			return rpa;
		}
	} else if (unlikely(!ch->skb)) {
		/* accounting says a chain is open but the allocation failed
		 * earlier: swallow the rest of it quietly */
		if (eop)
			memset(&ch->st, 0, sizeof(ch->st));
		return rpa;
	}

	skb_put_data(ch->skb, buf + seg.off, seg.len);

	if (act != CA_NI_CHAIN_DONE)
		return rpa;

	/* complete frame: same delivery decision as the single-buffer path.
	 * A PON control frame is never chained (an OMCI PDU is tens of bytes),
	 * so this path deliberately does not re-test the 0xfff1 link type. */
	{
		struct sk_buff *skb = ch->skb;
		u32 segs = ch->st.segs;
		/* the frame length must be taken BEFORE the skb is handed over:
		 * eth_type_trans() pulls the Ethernet header off it and
		 * napi_gro_receive() consumes it outright */
		u32 flen = ch->st.total;

		ch->skb = NULL;
		memset(&ch->st, 0, sizeof(ch->st));
		rx->chain_frames++;
		rx->chain_segs += segs;
		if (segs > rx->chain_max_segs)
			rx->chain_max_segs = segs;
		wan = cortina_ni_rx_wan_dest(ch->hdra_lo, &is_l3);
		if (wan) {
			if (is_l3)
				rx->wan_l3_frames++;
			else
				rx->wan_frames++;
			wan->stats.rx_packets++;
			wan->stats.rx_bytes += flen;
			skb->protocol = eth_type_trans(skb, wan);
			napi_gro_receive(&rx->napi, skb);
			return rpa;
		}
		rx->frames++;
		rx->bytes += flen;
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += flen;
		skb->protocol = eth_type_trans(skb, ndev);
		napi_gro_receive(&rx->napi, skb);
	}
	return rpa;

drop:
	ndev->stats.rx_errors++;
	net_warn_ratelimited("%s: RX chain: malformed (act %d, segs %u got %u of %u) - partial frame dropped\n",
			     netdev_name(ndev), act, ch->st.segs, ch->st.got,
			     ch->st.total);
	cortina_ni_rx_chain_reset(rx, ch);
	return rpa;
}

/* consume one CPU-EPP descriptor. The frame sits in a ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 29. */
static u32 cortina_ni_rx_frame(struct cortina_ni *ni, unsigned int voq, u64 desc)
{
	/* ★ THE BOARD'S GEOMETRY, DECLARED HERE AND DECIDED WITH IN ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 30. */
	static const struct ca_ni_rx_pool_geom pool_geom = {
		.map_size	= CA_NI_RX_MAP_SIZE,
		.pool0_bytes	= CA_NI_RX_CPU_POOL0_BYTES,
		.dq_off		= CA_NI_RX_DQ_POOL_OFF,
		.pool0_bufsz	= CA_NI_RX_CPU_POOL0_BUFSZ,
		.pool1_bufsz	= CA_NI_RX_CPU_POOL1_BUFSZ,
		/* the deep-queue pool uses the same 2048B buffers -- OUR fact to
		 * state, which is why the core takes it as its own field */
		.dq_bufsz	= CA_NI_RX_CPU_POOL0_BUFSZ,
		.tailroom	= CA_NI_RX_BUF_TAILROOM,
	};
	static const struct ca_ni_rx_layout layout = {
		.hdra_off	= CA_NI_RX_HDRA_OFF,
		.frame_off	= CA_NI_RX_FRAME_OFF,
		.hdr_cpu_len	= CA_NI_RX_HDR_CPU_LEN,
		.desc_hdr_len	= CA_NI_RX_DESC_HDR_LEN,
		.eth_hlen	= ETH_HLEN,
	};
	struct cortina_ni_rx *rx = ni->rx;
	struct net_device *ndev = rx->netdev;
	u32 pa = lower_32_bits(desc) & CA_NI_RX_DESC_PA;
	u32 dlen = FIELD_GET(CA_NI_RX_DESC_LEN, desc);
	u32 swid = FIELD_GET(CA_NI_RX_DESC_SWID, desc);
	u32 base = lower_32_bits(rx->cpu_dram_dma);
	u32 hdra_hi = 0, hdra_lo = 0, buf_max;
	u32 rpa;	/* the PA to hand back, or 0 if this buffer is not ours */
	enum ca_ni_rx_geom_verdict verdict;
	struct ca_ni_rx_geom fgeom;
	struct ca_ni_rx_buf bufloc;
	enum ca_ni_rx_pool pool;
	int pkt_size = 0;	/* header-A's RAW packet size, before any header */
	struct sk_buff *skb;
	const u8 *buf;
	unsigned int off;
	int len;

	rx->last_desc = desc;

	/* One buffer's span, and who owns it. bufPA (128B-aligned, ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 31. */
	pool = ca_ni_rx_buf_locate(pa, base, &pool_geom, &bufloc);
	if (unlikely(pool == CA_NI_RX_POOL_BAD)) {
		rx->drop_badpa++;
		ndev->stats.rx_errors++;
		net_err_ratelimited("%s: RX desc %016llx: PA outside CPU pool\n",
				    netdev_name(ndev), desc);
		return 0;	/* unknown PA: cannot safely recycle it */
	}
	buf = (const u8 *)rx->cpu_dram + bufloc.off_in_region;
	buf_max = bufloc.buf_max;
	rpa = bufloc.rpa;
	if (unlikely(pool == CA_NI_RX_POOL_DQ))
		rx->dq_frames++;

	/* ★ multi-buffer receive (rx_chain, default off). A ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 165. */
	if (unlikely(rx_chain &&
		     (lower_32_bits(desc) & (CA_NI_RX_DESC_SOP |
					     CA_NI_RX_DESC_EOP)) !=
		     (CA_NI_RX_DESC_SOP | CA_NI_RX_DESC_EOP)))
		return cortina_ni_rx_chain_seg(ni, voq, desc, buf, buf_max, rpa);

	if (unlikely(!(lower_32_bits(desc) & CA_NI_RX_DESC_SOP))) {
		/* SOP-less descriptor = jumbo continuation or desync; drop */
		rx->drop_nosop++;
		ndev->stats.rx_errors++;
		return rpa;
	}

	if (likely(!swid)) {
		/* normal frame: HEADER_A at +0x40 = two BIG-ENDIAN 32-bit ...
		 * dev/MEASURED-cortina-ni-rx.c.md sec 32. */
		hdra_hi = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF);
		hdra_lo = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF + 4);
		rx->last_hdra = ((u64)hdra_hi << 32) | hdra_lo;
		pkt_size = FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo);
	} else {
		/* headerless format (stock: nonzero sw_id, WiFi-FF style):
		 * frame at +0x10, length from the descriptor */
		rx->swid_frames++;
	}

	/* ★ WHERE THE FRAME IS AND WHETHER IT IS WELL FORMED - the core's
	 * (ca_ni_rx_frame_geom): the header-A vs headerless offset, the CPU
	 * header block, the runt/oversize split against the usable window, and
	 * the staleness witness. */
	verdict = ca_ni_rx_frame_geom(&layout, swid, dlen,
				      !!(hdra_hi & CA_NI_HDRA_W0_CPU_FLG),
				      pkt_size, buf_max, &fgeom);
	off = fgeom.off;
	len = fgeom.len;

	/* ★ Staleness witness, free: the descriptor's own pktlen and ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 33. */
	if (unlikely(fgeom.stale)) {
		rx->stale_buf++;
		net_warn_ratelimited("%s: RX stale buffer: desc pktlen=%u vs HEADER_A.pkt_size=%d at pa=%08x (buffer reused before the copy)\n",
				     netdev_name(ndev), dlen, pkt_size, pa);
	}

	if (unlikely(rx_debug && rx->frames < 4)) {
		netdev_info(ndev,
			    "RX desc=%016llx hdra=%08x:%08x (lspid=%u pkt_size=%u cpu=%u swid=%u dlen=%u) pa=%08x len=%d off=%u\n",
			    desc, hdra_hi, hdra_lo,
			    (u32)FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo),
			    (u32)FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo),
			    !!(hdra_hi & CA_NI_HDRA_W0_CPU_FLG), swid, dlen,
			    pa, len, off);
		print_hex_dump(KERN_INFO, "RX buf+0x40: ", DUMP_PREFIX_OFFSET,
			       16, 1, buf + CA_NI_RX_HDRA_OFF, 96, false);
	}

	if (unlikely(verdict != CA_NI_RX_GEOM_OK)) {
		rx->drop_len++;
		/* ★ split, because the two causes mean opposite things and the
		 * dev/MEASURED-cortina-ni-rx.c.md sec 166. */
		if (verdict == CA_NI_RX_GEOM_RUNT)
			rx->drop_runt++;
		else
			rx->drop_oversize++;
		ndev->stats.rx_length_errors++;
		ndev->stats.rx_errors++;
		return rpa;
	}

	/* ★ TEMPORARY DIAGNOSTIC (rx_ds_tap) - revert with rx_frag_tap.  Sampled
	 * HERE, ahead of the PON-control and WAN-netdev branches below, both of
	 * which return early: a downstream frame must be visible to the tap
	 * whichever branch it goes on to take. */
	if (unlikely(rx_ds_tap > 0 && !swid))
		cortina_ni_rx_ds_tap(ndev, buf, off, len, desc, pa, dlen,
				     hdra_hi, hdra_lo);

	/* DS-WAN delivery spy (rx_debug): a DHCP frame (UDP src/dst ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 34. */
	if (unlikely(rx_debug && !swid && off + 38 <= buf_max &&
		     buf[off + 12] == 0x08 && buf[off + 13] == 0x00 &&
		     buf[off + 23] == 0x11)) {
		u16 dport = ((u16)buf[off + 36] << 8) | buf[off + 37];
		u16 sport = ((u16)buf[off + 34] << 8) | buf[off + 35];

		if (dport == 67 || dport == 68 || sport == 67 || sport == 68)
			net_info_ratelimited(
			  "%s: DHCP DS frame lspid=%lu swid=%u sport=%u dport=%u DA=%02x:%02x:%02x:%02x:%02x:%02x len=%d\n",
			  netdev_name(ndev),
			  FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo), swid,
			  sport, dport, buf[off], buf[off+1], buf[off+2],
			  buf[off+3], buf[off+4], buf[off+5], len);
	}

	/* ★ TEMPORARY DIAGNOSTIC crc_ntfy tap (rx_crc_tap gate - ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 35. */
	if (unlikely(rx_crc_tap && !swid && off + 38 <= buf_max &&
		     buf[off + 12] == 0x08 && buf[off + 13] == 0x00 &&
		     buf[off + 23] == 0x11 &&
		     ((((u16)buf[off + 36] << 8) | buf[off + 37]) == 19555))) {
		u32 c32 = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF + 8);
		u16 c16 = get_unaligned_be16(buf + CA_NI_RX_HDRA_OFF + 12);
		bool cpuf = !!(hdra_hi & CA_NI_HDRA_W0_CPU_FLG);

		rx->tap_hits++;
		rx->tap_crc32 = c32;
		rx->tap_crc16 = c16;
		rx->tap_cpuflg = cpuf;
		net_info_ratelimited(
			"%s: crc_tap %pI4:%u -> %pI4:19555 UDP lspid=%lu cpu_flg=%u HW crc32=%08x crc16=%04x (vs the install crc = the divergence)\n",
			netdev_name(ndev), buf + off + 26,
			(u16)(((u16)buf[off + 34] << 8) | buf[off + 35]),
			buf + off + 30,
			FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo),
			cpuf, c32, c16);
	}

	/* ★ TEMP DIAG packet-stack tap (rx_stack_tap gate - REVERT ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 36. */
	if (unlikely(rx_stack_tap && !swid))
		cortina_ni_rx_stack_tap(ni, buf + off, len);

	/* ★ PPPoE punt-integrity witness (GAP-2). A DS PPPoE session ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 37. */
	if (unlikely(cortina_ni_pppoe_punt_armed()))
		cortina_ni_pppoe_punt_inspect(buf + off, len);

	/* DS PON control frame (GPON OMCI): the PDC steers the OMCC ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 38. */
	{
		/* ★ THE PREDICATE AND THE PDU BOUND ARE THE CORE'S ...
		 * dev/MEASURED-cortina-ni-rx.c.md sec 39. */
		struct ca_ni_pon_pdu pdu;

		if (unlikely(ca_ni_rx_pon_ctrl(buf + off, len, 0xfff1,
					       CA_NI_PON_HDR_LEN, &pdu))) {
			cortina_ni_pon_rx_fn fn;

			/* the read side of the withdrawal above: the callback
			 * runs INSIDE the section synchronize_rcu() waits on */
			rcu_read_lock();
			fn = rcu_dereference(cortina_ni_pon_rx_cb);
			rx->pon_frames++;
			if (fn && pdu.len)
				fn(buf + off + pdu.off, pdu.len);
			rcu_read_unlock();
			return rpa;
		}
	}

	/* DS PON DATA (WAN): a de-encapsulated data-GEM frame carries ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 40. */
	if (unlikely(!swid)) {
		/* ONE copy of this decision, shared with the chain path's
		 * completion handler - see cortina_ni_rx_wan_dest() */
		bool is_l3wan = false;
		struct net_device *wan = cortina_ni_rx_wan_dest(hdra_lo,
								&is_l3wan);

		if (wan) {
			if (is_l3wan)
				rx->wan_l3_frames++;
			else
				rx->wan_frames++;
			skb = napi_alloc_skb(&rx->napi, len);
			if (unlikely(!skb)) {
				rx->drop_nobuf++;
				wan->stats.rx_dropped++;
				return rpa;
			}
			skb_put_data(skb, buf + off, len);
			skb->protocol = eth_type_trans(skb, wan);
			napi_gro_receive(&rx->napi, skb);
			wan->stats.rx_packets++;
			wan->stats.rx_bytes += len;
			return rpa;
		}
		/* no WAN netdev registered: fall through to eth0 */
	}

	/* ★ CPU->LAN egress binding: remember which RJ45 this source ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 41. */
	if (likely(!swid && len >= ETH_HLEN))
		cortina_ni_lan_tx_learn(ni, buf + off + ETH_ALEN,
					FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo));

	skb = napi_alloc_skb(&rx->napi, len);
	if (unlikely(!skb)) {
		rx->drop_nobuf++;
		ndev->stats.rx_dropped++;
		return rpa;
	}
	skb_put_data(skb, buf + off, len);
	/* ★ TEMPORARY DIAGNOSTIC (rx_frag_tap) - revert with the params above.
	 * Must run before eth_type_trans(), which pulls the Ethernet header off
	 * the skb; skb->data still points at the copied frame here. */
	if (unlikely(rx_frag_tap > 0))
		cortina_ni_rx_frag_tap(ndev, buf, off, len, desc, pa, dlen,
				       hdra_lo, skb->data);
	skb->protocol = eth_type_trans(skb, ndev);
	napi_gro_receive(&rx->napi, skb);

	rx->frames++;
	rx->bytes += len;
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += len;
	return rpa;
}

/* ★ TEMPORARY DIAGNOSTIC (rx_frag_tap) - revert with the module params above.
 * noinline and called under an unlikely() guard so the disabled cost is one
 * global load and a predicted-not-taken branch on the 1 Gbps hot path.  Wire
 * bytes are read with explicit byte math, never a struct cast. */
static noinline void cortina_ni_rx_frag_tap(struct net_device *ndev,
					    const u8 *buf, u32 off, int len,
					    u64 desc, u32 pa, u32 dlen,
					    u32 hdra_lo, const u8 *copied)
{
	static atomic_t seen = ATOMIC_INIT(0);
	const u8 *f = buf + off;
	u32 ip_id, frag, tot_len, crc_skb, crc_reread, hdra_lo2;
	const u8 *ip;
	int n;

	if (len < ETH_HLEN + 20)
		return;
	/* plain Ethernet/IPv4 only: the ladder is a ping, no tag expected */
	if (f[12] != 0x08 || f[13] != 0x00)
		return;
	ip = f + ETH_HLEN;
	if ((ip[0] >> 4) != 4)
		return;
	frag = ((u32)ip[6] << 8) | ip[7];
	/* a fragment carries a non-zero offset or the more-fragments bit */
	if (!(frag & 0x3fff) && !(frag & 0x2000))
		return;

	n = atomic_inc_return(&seen);
	if (n > rx_frag_tap)
		return;

	ip_id = ((u32)ip[4] << 8) | ip[5];
	tot_len = ((u32)ip[2] << 8) | ip[3];

	/* the bytes as we handed them to the stack, then the same range read
	 * back out of the pool buffer after a delay */
	crc_skb = crc32(0, copied, len);
	if (rx_frag_tap_us > 0)
		udelay(rx_frag_tap_us);
	crc_reread = crc32(0, buf + off, len);
	hdra_lo2 = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF + 4);

	netdev_info(ndev,
		    "frag_tap#%d pa=%08x eqid=%u sop=%u eop=%u csum_err=%u dlen=%u hdra_pkt_size=%u len=%d off=%u ip{id=%04x frag=%04x tot_len=%u} crc=%08x/%08x %s%s\n",
		    n, pa, (u32)(desc & CA_NI_RX_DESC_EQID),
		    !!(desc & CA_NI_RX_DESC_SOP), !!(desc & CA_NI_RX_DESC_EOP),
		    !!(desc & CA_NI_RX_DESC_CSUM_ERR), dlen,
		    (u32)FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo),
		    len, off, ip_id, frag, tot_len,
		    crc_skb, crc_reread,
		    crc_skb == crc_reread ? "STABLE" : "CHANGED-AFTER-COPY",
		    hdra_lo2 != hdra_lo ? " HDRA-ALSO-CHANGED" : "");
}

/* ★ TEMPORARY DIAGNOSTIC (rx_ds_tap) - revert with ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 42. */
static noinline void cortina_ni_rx_ds_tap(struct net_device *ndev,
					  const u8 *buf, u32 off, int len,
					  u64 desc, u32 pa, u32 dlen,
					  u32 hdra_hi, u32 hdra_lo)
{
	static atomic_t seen = ATOMIC_INIT(0);
	u32 lspid = FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo);
	u32 ldpid = FIELD_GET(CA_NI_HDRA_W1_LDPID, hdra_lo);
	u32 etype;
	int n;

	/* downstream = the PON source lspid, or the L3_WAN ldpid the HW-L3 DS
	 * route stamps.  Everything else (the LAN lspids) is not our question. */
	if (lspid != CA_NI_LSPID_PON && ldpid != CA_NI_LSPID_L3_WAN)
		return;

	n = atomic_inc_return(&seen);
	if (n > rx_ds_tap)
		return;

	/* 0xfff1 = the vendor PON link-type marker on a DS OMCI control frame;
	 * anything else here is a de-encapsulated data-GEM frame */
	etype = ((u32)buf[off + 12] << 8) | buf[off + 13];

	netdev_info(ndev,
		    "ds_tap#%d pa=%08x eqid=%u sop=%u eop=%u csum_err=%u dlen=%u hdra_pkt_size=%u len=%d off=%u lspid=%u ldpid=0x%02x cpu_flg=%u deep_q=%u etype=%04x %s\n",
		    n, pa, (u32)(desc & CA_NI_RX_DESC_EQID),
		    !!(desc & CA_NI_RX_DESC_SOP), !!(desc & CA_NI_RX_DESC_EOP),
		    !!(desc & CA_NI_RX_DESC_CSUM_ERR), dlen,
		    (u32)FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo),
		    len, off, lspid, ldpid,
		    !!(hdra_hi & CA_NI_HDRA_W0_CPU_FLG),
		    !!(hdra_hi & CA_NI_HDRA_W0_DEEP_Q), etype,
		    etype == 0xfff1 ? "OMCI-control" : "data/punt");
}

/* drain one voq's CPU-EPP ring; returns work done, updates rx->rptr[voq] */
static int cortina_ni_rx_poll_voq(struct cortina_ni *ni, unsigned int voq,
				  int budget)
{
	struct cortina_ni_rx *rx = ni->rx;
	__le64 *vring = rx->ring +
		(rx_ring_hi ? CA_NI_RX_RING_HI_OFFSET / sizeof(__le64) : 0) +
		voq * CA_NI_RX_RING_SLOTS_PER_VOQ;
	u32 recycle[CA_NI_RX_RECYCLE_MAX];
	unsigned int nrecycle = 0, i;
	u32 rptr = rx->rptr[voq];
	u32 wptr;
	int work = 0;

	/* never consume more buffers in one poll than we can hand ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 43. */
	if (budget > CA_NI_RX_RECYCLE_MAX)
		budget = CA_NI_RX_RECYCLE_MAX;

	wptr = cortina_ni_rx_wptr_voq(ni, voq);
	dma_rmb();	/* descriptor reads after the pointer read */

	while (work < budget) {
		u64 desc;
		u32 pa;

		if (rptr == wptr) {
			wptr = cortina_ni_rx_wptr_voq(ni, voq);
			dma_rmb();
			if (rptr == wptr)
				break;
		}

		desc = le64_to_cpu(READ_ONCE(vring[rptr / CA_NI_RX_DESC_SIZE]));
		WRITE_ONCE(vring[rptr / CA_NI_RX_DESC_SIZE], 0);
		rptr = (rptr + CA_NI_RX_DESC_SIZE) & (CA_NI_RX_RING_BYTES - 1);
		work++;

		pa = cortina_ni_rx_frame(ni, voq, desc);
		if (cpu_pool_push && pa)
			/* Return the buffer to the pool the HARDWARE says it came ...
			 * dev/MEASURED-cortina-ni-rx.c.md sec 44. */
			recycle[nrecycle++] = (pa & CA_NI_QM_PUSH_ADDR) |
					      (u32)(desc & CA_NI_RX_DESC_EQID);
	}

	dma_wmb();	/* stock: dmb oshst before the rdptr store */
	writel(rptr, ni_base(ni) +
	       CA_NI_QM_EPP64_RDPTR(CA_NI_RX_CPU_PORT, voq));
	rx->rptr[voq] = rptr;
	rx->voq_frames[voq] += work;	/* spy: flow→voq spread (order check) */

	/* Return the buffers we have finished with, one batch per ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 45. */
	for (i = 0; i < nrecycle; i++) {
		if (unlikely(cortina_ni_rx_push_buf(ni,
				recycle[i] & CA_NI_QM_PUSH_EQID,
				recycle[i],
				CA_NI_RX_PUSH_TIMEOUT_NAPI_US))) {
			rx->push_fail++;
			net_err_ratelimited("%s: RX pool push timeout for pa=%08x - buffer lost, pool will shrink\n",
					    netdev_name(rx->netdev),
					    recycle[i]);
		}
	}
	return work;
}

static int cortina_ni_rx_poll(struct napi_struct *napi, int budget)
{
	struct cortina_ni_rx *rx =
		container_of(napi, struct cortina_ni_rx, napi);
	struct cortina_ni *ni = rx->ni;
	unsigned int voq;
	int work = 0, more;

	rx->polls++;

	/* ★ Drain ALL 8 CPU-port VOQs: the deep_q frame may land on any voq (by
	 * cos/priority), and stock arms every voq's CPU-EPP FIFO.  Round-robin
	 * within the shared NAPI budget. */
	for (voq = 0; voq < CA_NI_RX_VOQ_COUNT && work < budget; voq++)
		work += cortina_ni_rx_poll_voq(ni, voq, budget - work);

	if (work < budget && napi_complete_done(napi, work)) {
		cortina_ni_rx_irq_set(ni, true);
		/* close the enable-vs-new-frame race across all voqs */
		more = 0;
		for (voq = 0; voq < CA_NI_RX_VOQ_COUNT; voq++)
			if (cortina_ni_rx_wptr_voq(ni, voq) != rx->rptr[voq])
				more = 1;
		if (more && napi_schedule(&rx->napi))
			cortina_ni_rx_irq_set(ni, false);
	}
	return work;
}

/* RX steer: the full forwarding-engine path (stock golden ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 46. */
static int cortina_ni_rx_ind_idle(struct cortina_ni *ni, u32 access_reg)
{
	u32 v;
	int ret = ca_ni_access_wait(ni_base(ni) + access_reg, &v);

	if (ret)
		dev_warn_ratelimited(ni->dev,
				     "indirect idle @0x%x GO stuck (0x%08x)\n",
				     access_reg, v);
	return ret;
}

/* Generic indirect-table store: write ACCESS = GO|WR|idx, poll GO clear
 * (stock DO_INDIRCT_OP write path).  Bounded + non-fatal. */
static int cortina_ni_rx_ind_store(struct cortina_ni *ni, u32 access_reg, unsigned int idx)
{
	u32 v;
	int ret;

	ret = ca_ni_access_go(ni_base(ni) + access_reg,
			      CA_NI_IND_ACCESS_GO | CA_NI_IND_ACCESS_WR | idx,
			      &v);
	if (ret)
		dev_warn(ni->dev, "indirect store @0x%x[%u] GO stuck (0x%08x)\n",
			 access_reg, idx, v);
	return ret;
}

/* Generic indirect-table read: write ACCESS = GO|idx (rbw=0), poll GO clear;
 * the entry is then latched in the DATA registers for the caller to read.
 * Bounded + non-fatal. */
static int cortina_ni_rx_ind_read(struct cortina_ni *ni, u32 access_reg, unsigned int idx)
{
	u32 v;
	int ret;

	ret = ca_ni_access_go(ni_base(ni) + access_reg,
			      CA_NI_IND_ACCESS_GO | idx, &v);
	if (ret)
		/* RATELIMITED: this runs from /proc show paths, which a soak or a
		 * benchmark harness polls in a loop - an un-ratelimited warn there
		 * floods the serial console the witnesses are read over. */
		dev_warn_ratelimited(ni->dev,
				     "indirect read @0x%x[%u] GO stuck (0x%08x)\n",
				     access_reg, idx, v);
	return ret;
}

/* Read ONE word of an indirect table entry: latch it, then ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 47. */
static u32 cortina_ni_rx_ind_entry(struct cortina_ni *ni, u32 access_reg,
				   unsigned int idx, u32 data_reg)
{
	if (cortina_ni_rx_ind_read(ni, access_reg, idx))
		return 0;
	return readl(ni_base(ni) + data_reg);
}

/* program one PLE default-forward table entry: redirect a lookup-miss
 * traffic type of <lspid> to CPU port 0 (indirect read-modify-write) */
static int cortina_ni_rx_ple_dft_fwd(struct cortina_ni *ni, u32 lspid,
				     u32 type)
{
	u32 addr = lspid << 2 | type;
	u32 val;
	int ret;

	/* (0x1560/0x156c are NOT a DFT_FWD control block - the old writes here
	 * corrupted the VLAN check-id map; see cortina-ni-regs.h.  The default-forward
	 * table itself is programmed via the ACCESS/DATA indirect protocol below.) */
	ret = cortina_ni_rx_ind_idle(ni, CA_NI_PLE_DFT_FWD_ACCESS);
	if (ret)
		return ret;

	/* latch the entry into DATA */
	ret = cortina_ni_rx_ind_read(ni, CA_NI_PLE_DFT_FWD_ACCESS, addr);
	if (ret)
		return ret;

	val = readl(ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);
	/* ★ Stock LAN->CPU base forwarding, VERBATIM: 0x1832 decode ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 48. */
	val = CA_NI_RX_DFT_FWD_CPU_VAL;
	writel(val, ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);

	/* write it back */
	ret = cortina_ni_rx_ind_store(ni, CA_NI_PLE_DFT_FWD_ACCESS, addr);
	if (ret)
		return ret;

	/* ★ READ IT BACK (2026-08-04). The indirect protocol ACKs by ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 167. */
	ret = cortina_ni_rx_ind_read(ni, CA_NI_PLE_DFT_FWD_ACCESS, addr);
	if (ret)
		return ret;
	val = readl(ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);
	if (val != CA_NI_RX_DFT_FWD_CPU_VAL) {
		dev_err(ni->dev,
			"PLE dft-fwd entry %#x (lspid %u type %u) read back %#010x, wrote %#010x -- the entry did not take\n",
			addr, lspid, type, val, CA_NI_RX_DFT_FWD_CPU_VAL);
		return -EIO;
	}
	return 0;
}

/* Async-SError fault-attribution helper: full barrier so the suspect MMIO
 * write has posted, then a short delay so a delayed AXI external-abort (async
 * SError) is taken HERE - before the next marker prints.  So the LAST marker
 * line in the log before a panic pins the exact faulting access. */
static void cortina_ni_rx_settle(void)
{
	mb();
	mdelay(2);
}


/* ★★ ROOT-CAUSE FIX for the constant blackhole (rx_fe ldpid ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 49. */
static void cortina_ni_rx_port_profiles_init(struct cortina_ni *ni)
{
	struct ca_ni_lport_profile p;
	u32 i, d0, d1, d2;

	dev_info(ni->dev, "port-prof: programming IPPB/MMSHP/ILPB/ELPB (64 lports)\n");

	for (i = 0; i < CA_NI_L2FE_LPORT_COUNT; i++) {
		cortina_ni_rx_lport_profile(i, &p);

		/* IPPB: physical -> logical source-port map, identity */
		writel(i, ni_base(ni) + CA_NI_L2FE_IPPB_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_IPPB_ACCESS, i);

		/* MMSHP: allowed-ldpid bitmap = all-but-self (isolation off) */
		writel(p.mmshp_d1, ni_base(ni) + CA_NI_L2FE_MMSHP_DATA1);
		writel(p.mmshp_d0, ni_base(ni) + CA_NI_L2FE_MMSHP_DATA0);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_MMSHP_ACCESS, i);

		/* MMSHP check-id map: lport -> VLAN membership check-id (stock
		 * __g_l2_vlan_port_map).  Unprogrammed (our old bug wrote garbage
		 * into entry 63 only), the membership check qualifies nothing and
		 * LAN frames never reach the CPU port. */
		writel(p.chkid, ni_base(ni) + CA_NI_L2FE_CHKID_MAP_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_CHKID_MAP_ACCESS, i);

		/* ILPB ingress profile: stp_mode=FWD+LEARN + the stock defaults */
		writel(p.ilpb_d4, ni_base(ni) + CA_NI_L2FE_ILPB_DATA4);
		writel(p.ilpb_d3, ni_base(ni) + CA_NI_L2FE_ILPB_DATA3);
		writel(p.ilpb_d2, ni_base(ni) + CA_NI_L2FE_ILPB_DATA2);
		writel(p.ilpb_d1, ni_base(ni) + CA_NI_L2FE_ILPB_DATA1);
		writel(p.ilpb_d0, ni_base(ni) + CA_NI_L2FE_ILPB_DATA0);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_ILPB_ACCESS, i);

		/* ELPB egress profile: egr STP forward + vlan-aware (+dest_wan
		 * on the PON-side dest ports and the L3_LAN dest) */
		writel(0, ni_base(ni) + CA_NI_L2FE_ELPB_DATA1);
		writel(p.elpb_d0, ni_base(ni) + CA_NI_L2FE_ELPB_DATA0);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_ELPB_ACCESS, i);
	}

	/* the L2FE direct config regs stock also moves off hardware default
	 * (tier-1 live-stock values; incl. PLE_CTL skip_port_lpbk_chk+pon_mode
	 * and PARSER_CTRL arp_op_filter_dis) */
	writel(CA_NI_L2FE_GLB_CTRL_STOCK, ni_base(ni) + CA_NI_L2FE_GLB_CTRL);
	writel(CA_NI_L2FE_TPID_S_STOCK, ni_base(ni) + CA_NI_L2FE_PP_TPID_CMP_S);
	writel(CA_NI_L2FE_TPID_O_STOCK, ni_base(ni) + CA_NI_L2FE_PP_TPID_CMP_O);
	writel(CA_NI_L2FE_PARSER_CTRL_STOCK,
	       ni_base(ni) + CA_NI_L2FE_PP_PARSER_CTRL);
	writel(CA_NI_L2FE_L2_LEARNING_STOCK,
	       ni_base(ni) + CA_NI_L2FE_PLC_L2_LEARNING);
	writel(CA_NI_L2FE_VLAN_MODE_STOCK, ni_base(ni) + CA_NI_L2FE_PLC_VLAN_MODE);
	writel(CA_NI_L2FE_PLE_CTL_STOCK, ni_base(ni) + CA_NI_L2FE_PLE_CTL);
	writel(CA_NI_L2FE_UNKWN_VLAN_DFT1_STOCK,
	       ni_base(ni) + CA_NI_L2FE_PLE_UNKWN_VLAN_DFT1);
	/* PLE regs stock programs that ours skipped (golden 2026-07-15) */
	writel(CA_NI_L2FE_PLE_TRUNK_STOCK, ni_base(ni) + CA_NI_L2FE_PLE_TRUNK0);
	writel(CA_NI_L2FE_PLE_TRUNK_STOCK, ni_base(ni) + CA_NI_L2FE_PLE_TRUNK1);
	writel(CA_NI_L2FE_PLE_HD_FF_STOCK, ni_base(ni) + CA_NI_L2FE_PLE_HD_FF_CTL);

	/* readback proof (genuine indirect reads) for the boot log */
	cortina_ni_rx_ind_read(ni, CA_NI_L2FE_ILPB_ACCESS, CA_NI_RX_PORT);
	d2 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA2);
	d1 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA1);
	d0 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA0);
	cortina_ni_rx_ind_read(ni, CA_NI_L2FE_MMSHP_ACCESS, CA_NI_RX_PORT);
	dev_info(ni->dev,
		 "port-prof: ilpb[0] d2=0x%08x d1=0x%08x d0=0x%08x (stp=%lu want 3) mmshp[0]=%08x_%08x ple_ctl=0x%08x\n",
		 d2, d1, d0, FIELD_GET(CA_NI_L2FE_ILPB_STP_MODE, d2),
		 readl(ni_base(ni) + CA_NI_L2FE_MMSHP_DATA1),
		 readl(ni_base(ni) + CA_NI_L2FE_MMSHP_DATA0),
		 readl(ni_base(ni) + CA_NI_L2FE_PLE_CTL));
}

/* ★ Program the L2FE PDPID_MAP so our redir dest (CPU_0, ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 50. */
static void cortina_ni_rx_redir_ldpid_set(struct cortina_ni *ni, u8 idx,
					  u8 dest_ldpid);

/* Stock golden PDPID_MAP (tier-1 stock_l2fe_forwarding.txt): ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 51. */
static const struct { u8 ldpid, pdpid; } cortina_ni_rx_pdpid_map[] = {
	{ 0x08, 0x0c },	/* 9QUEUE_NI0 (CA_NI_LDPID_9QUEUE_LO)   -> PPORT_OAM: CPU-injected PON control frames; stock maps every 9QUEUE row 0x08..0x0f to 0x0c */
	{ 0x09, 0x0c },	/* 9QUEUE_NI1 (9QUEUE_LO + 1)            -> PPORT_OAM */
	{ 0x0d, 0x0c },	/* 9QUEUE_NI5 (9QUEUE_LO + 5)            -> PPORT_OAM */
	{ 0x10, 0x09 },	/* CPU_0 (CA_NI_RX_CPU_LDPID)             -> PPORT_CPU (CA_NI_RX_CPU_PDPID): the redir dest resolves to the CPU */
	{ 0x19, 0x0d },	/* L3_LAN (CA_NI_RX_L3LAN_LDPID)          -> PPORT_L3_LAN: my-MAC/ARP/L3-hit -> ES8 -> L3FE -> CLS trap -> CPU.  ROW NEVER READ, see above */
	{ 0x1d, 0x09 },	/* CPU_Q (AAL_LPORT_CPU_Q)                -> PPORT_CPU */
	{ 0x1f, 0x0f },	/* BLACKHOLE (AAL_LPORT_BLACKHOLE)        -> PPORT_BLACKHOLE (CA_NI_PPORT_BLACKHOLE): drop */
	{ 0x32, 0x08 },	/* CA_NI_RX_MC_CPU_LDPID -> PPORT_QM (CA_NI_PPORT_QM).
			 * ⚠ 0x32 has NO declared enumerator: cortina-ni-regs.h:1946-1947
			 * declares the ENDPOINTS only (CA_NI_LDPID_CPU_MQ_LO 0x20 =
			 * AAL_LPORT_CPU_MQ_0 / LLID_GEM_INDEX_0, _HI 0x3f). 0x32 is
			 * 0x20 + 18, i.e. INSIDE that declared range -- a derivation,
			 * not a name. This comment first said `CPU_MQ_18 /
			 * LLID_GEM_INDEX_18`, which the tree does not establish. Stock
			 * maps the whole 0x20..0x3f range to QM, which is what the
			 * endpoints are for. */
	/* ★★★ The map is the plain STOCK literal ([0x19]=0x0d L3_LAN unicast,
	 * [0x32]=0x08 QM): stock's CPU copy comes from DFT_FWD 0x1832 -> mcgid 0x19
	 * -> the L3FE/CLS trap, NOT from a PDPID unicast dest in this table.  A
	 * build67 "unicast-to-CPU-port" theory that rewrote [0x19]/[0x32] to 0x00 was
	 * refuted on HW (rmu_rx still 0) and reverted. */
};

/* ★★ 2026-07-15 THE CPU-RX FIX: zero the ARB FLOW_DBUF table ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 52. */
static void cortina_ni_rx_flow_dbuf_init(struct cortina_ni *ni)
{
	u32 b[8], a[8];
	unsigned int i;

	for (i = 0; i < 8; i++) {
		b[i] = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS,
						     i, CA_NI_L2FE_ARB_FLOW_DBUF_DATA);
	}
	for (i = 0; i < CA_NI_L2FE_ARB_FLOW_DBUF_ENTRIES; i++) {
		writel(0, ni_base(ni) + CA_NI_L2FE_ARB_FLOW_DBUF_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS, i);
	}
	for (i = 0; i < 8; i++) {
		a[i] = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS,
						     i, CA_NI_L2FE_ARB_FLOW_DBUF_DATA);
	}
	dev_info(ni->dev,
		 "flow-dbuf(0x165c/0x1660) before[0..7]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		 b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
	dev_info(ni->dev,
		 "flow-dbuf AFTER[0..7]=%08x %08x %08x %08x %08x %08x %08x %08x (want all 0 = stock, no deep_q marks)\n",
		 a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);
}

static void cortina_ni_rx_arb_deepq_init(struct cortina_ni *ni)
{
	u32 d0, d1, i, e;

	/* ★★ THE CPU-RX handoff (rev. 2026-07-11): resolve every ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 53. */
	writel(0, ni_base(ni) + CA_NI_L2FE_ARB_PORT_DBUF_DATA);
	cortina_ni_rx_ind_store(ni, CA_NI_L2FE_ARB_PORT_DBUF_ACCESS, 0);

	/* ★★ FULL PDPID_MAP = STOCK GOLDEN literals (tier-1 ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 54. */
	for (e = 0; e < ARRAY_SIZE(cortina_ni_rx_pdpid_map); e++) {
		u8 pdpid = cortina_ni_rx_pdpid_map[e].pdpid;

		/* L3_LAN (ldpid 0x19) is routed by the pdpid_l3lan parameter, not by
		 * the table row above: 0x0d = stock L3_LAN -> L3FE/CLS trap, 0x00 =
		 * straight to CPU port 0 (the prior-session test that bypassed the
		 * trap), 0x09 = CPU as the L2FE names it. */
		if (cortina_ni_rx_pdpid_map[e].ldpid == CA_NI_RX_L3LAN_LDPID)
			pdpid = pdpid_l3lan;

		/* Every other ldpid gets its STOCK PDPID with no CPU override: the CPU
		 * frame reaches the QM by resolving to ldpid 0x32 ([0x32]=0x08), the
		 * stock path, not by bodging [0x19]->0x08 (the wrong ES-port route). */
		for (i = 0; i < 4; i++) {
			u32 idx = cortina_ni_rx_pdpid_map[e].ldpid |
				  ((i & 1) ? CA_NI_L2FE_PDPID_IDX_DBUF : 0) |
				  ((i & 2) ? CA_NI_L2FE_PDPID_IDX_MYMAC : 0);

			writel(FIELD_PREP(CA_NI_L2FE_PDPID_MAP_PDPID, pdpid),
			       ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA);
			cortina_ni_rx_ind_store(ni, CA_NI_L2FE_PDPID_MAP_ACCESS, idx);
		}
	}
	dev_info(ni->dev, "pdpid: map written ([0x19]=0x%02x (build101 override, stock=0x0d), [0x32]=0x08 QM); L3_LAN routed to pdpid 0x%02x\n",
		 pdpid_l3lan, pdpid_l3lan);

	/* ★★ REMOVED the LDPID->CPU redirect (was ... -- dev/MEASURED-cortina-ni-rx.c.md sec 55. */
	dev_info(ni->dev,
		 "arb-deepq: REDIR_LDPID[0x19] data=0x%08x (redirect REMOVED; want NOT rdir_en|0x%x)\n",
		 cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_REDIR_LDPID_ACCESS,
					 CA_NI_RX_L3LAN_LDPID,
					 CA_NI_L2FE_REDIR_LDPID_DATA),
		 CA_NI_RX_CPU_LDPID);

	d0 = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				     CA_NI_RX_QM_REDIR_LDPID,
				     CA_NI_L2FE_PDPID_MAP_DATA) &
	     CA_NI_L2FE_PDPID_MAP_PDPID;
	d1 = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				     CA_NI_L2FE_PDPID_IDX_DBUF |
				     CA_NI_RX_QM_REDIR_LDPID,
				     CA_NI_L2FE_PDPID_MAP_DATA) &
	     CA_NI_L2FE_PDPID_MAP_PDPID;
	dev_info(ni->dev,
		 "arb-deepq: base LAN->CPU: PDPID_MAP[0x32]{dbuf0}=0x%x {dbuf1}=0x%x arb_ctrl=0x%08x (want both 0x%x=CPU); DFT_FWD set to 0x%04x\n",
		 d0, d1, readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL),
		 CA_NI_RX_CPU_PDPID, CA_NI_RX_DFT_FWD_CPU_VAL);
}

/* ★ De-secure the shared CapSRAM (TrustZone), then deassert ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 56. */
static void __maybe_unused cortina_ni_rx_fabric_mce_ungate(struct cortina_ni *ni)
{
	void __iomem *glb = ni->win[CA_NI_WIN_GLB];
	u32 before, after;

	if (!glb) {
		dev_warn(ni->dev, "fabric-ungate: no GLB window\n");
		return;
	}
	before = readl(glb + CA_NI_GLB_BLOCK_RESET_EXT);
	dev_emerg(ni->dev, "FABRIC 1: reset(a4)=0x%08x, clearing bit5 (NI-MCE)\n",
		  before);
	writel(before & ~CA_NI_GLB_BLOCK_RESET_EXT_MCE, glb + CA_NI_GLB_BLOCK_RESET_EXT);
	cortina_ni_rx_settle();
	after = readl(glb + CA_NI_GLB_BLOCK_RESET_EXT);
	dev_emerg(ni->dev, "FABRIC 2: reset(a4) now 0x%08x (stock=0x00079f00)\n",
		  after);
}

static void __maybe_unused cortina_ni_rx_capsram_desecure(struct cortina_ni *ni)
{
	void __iomem *cap;
	u32 before, after;

	dev_emerg(ni->dev, "CAPSRAM 1: ioremap TZCONTROL @0x%llx\n",
		  (unsigned long long)CA_NI_CAPSRAM_PHYS);
	cap = ioremap(CA_NI_CAPSRAM_PHYS, 0x10);
	if (!cap) {
		dev_err(ni->dev, "capsram: ioremap failed\n");
		return;
	}

	dev_emerg(ni->dev, "CAPSRAM 2: read TZCONTROL\n");
	before = readl(cap + CA_NI_CAPSRAM_TZCONTROL);
	cortina_ni_rx_settle();

	dev_emerg(ni->dev, "CAPSRAM 3: TZCONTROL=0x%08x, writing 0 (all NS)\n",
		  before);
	writel(0, cap + CA_NI_CAPSRAM_TZCONTROL);
	cortina_ni_rx_settle();

	after = readl(cap + CA_NI_CAPSRAM_TZCONTROL);
	dev_emerg(ni->dev, "CAPSRAM 4: TZCONTROL now 0x%08x (write %s)\n",
		  after, after ? "IGNORED - EL3-protected?" : "TOOK");
	iounmap(cap);
}

/* ★ MC-flood-to-CPU (needs the CapSRAM de-secure above to not ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 57. */
static void __maybe_unused cortina_ni_rx_mc_flood_init(struct cortina_ni *ni)
{
	/* MC_FIB[16]: per-copy action -> ldpid 16 (CPU); no VLAN edit */
	dev_emerg(ni->dev, "MCFLOOD 1: MC_FIB[%u] DATA write\n", CA_NI_RX_CPU_LDPID);
	writel(0, ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA0);
	writel(FIELD_PREP(CA_NI_L2FE_MC_FIB_LDPID, CA_NI_RX_CPU_LDPID),
	       ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA1);
	writel(0, ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA2);
	cortina_ni_rx_settle();
	dev_emerg(ni->dev, "MCFLOOD 2: MC_FIB[%u] ACCESS store\n", CA_NI_RX_CPU_LDPID);
	cortina_ni_rx_ind_store(ni, CA_NI_L2FE_MC_FIB_ACCESS, CA_NI_RX_CPU_LDPID);
	cortina_ni_rx_settle();

	/* ni_mce_indx[G].mc_vec = bit 16 (CPU) - the write that SErrored pre-fix */
	dev_emerg(ni->dev, "MCFLOOD 3: mce_indx[%u] DATA write (was the SError)\n",
		  CA_NI_RX_FLOOD_MCGID);
	writel(BIT(CA_NI_RX_CPU_LDPID), ni_base(ni) + CA_NI_NI_MCE_INDX_DATA0);
	writel(0, ni_base(ni) + CA_NI_NI_MCE_INDX_DATA1);
	cortina_ni_rx_settle();
	dev_emerg(ni->dev, "MCFLOOD 4: mce_indx[%u] ACCESS store\n", CA_NI_RX_FLOOD_MCGID);
	cortina_ni_rx_ind_store(ni, CA_NI_NI_MCE_INDX_ACCESS, CA_NI_RX_FLOOD_MCGID);
	cortina_ni_rx_settle();

	/* pollable read-back for the boot log */
	dev_emerg(ni->dev, "MCFLOOD 5 (survived): mcgid=%u mc_vec.lo=0x%08x\n",
		  CA_NI_RX_FLOOD_MCGID,
		  cortina_ni_rx_ind_entry(ni, CA_NI_NI_MCE_INDX_ACCESS,
					  CA_NI_RX_FLOOD_MCGID,
					  CA_NI_NI_MCE_INDX_DATA0));
}

/* Initialize forwarding tables; register values do not prove CPU delivery. */
static void cortina_ni_rx_mc_group_init(struct cortina_ni *ni)
{
	/* Clear the default multicast vector as observed in stock. */
	writel(0, ni_base(ni) + CA_NI_NI_MCE_INDX_DATA1);
	writel(0, ni_base(ni) + CA_NI_NI_MCE_INDX_DATA0);
	cortina_ni_rx_settle();
	cortina_ni_rx_ind_store(ni, CA_NI_NI_MCE_INDX_ACCESS, CA_NI_RX_DFT_FWD_MCGID);
	cortina_ni_rx_settle();

	/* ★ 2026-07-15 relabeled: this table @0x1634 is NOT the ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 60. */
	{
		static const u8 nkpol_d[] = {
			/* row = unkwn_pol_idx << 2 | pkt_type  ->  flooding_pol_id */
			0x0F,	/* [0x10] idx 4, MC_UC : default profile 15 */
			0x04,	/* [0x11] idx 4, UUC   : profile 4 */
			0x0F,	/* [0x12] idx 4, UMC   : default profile 15 */
			0x09,	/* [0x13] idx 4, BC    : profile 9 */
			0x0F,	/* [0x14] idx 5, MC_UC : default profile 15 */
			0x05,	/* [0x15] idx 5, UUC   : profile 5 */
			0x0F,	/* [0x16] idx 5, UMC   : default profile 15 */
			0x0A,	/* [0x17] idx 5, BC    : profile 10 */
			0x0F,	/* [0x18] idx 6, MC_UC : default profile 15 */
			0x0B,	/* [0x19] idx 6, UUC   : profile 11 */
			0x0F,	/* [0x1a] idx 6, UMC   : default profile 15 */
			0x0C,	/* [0x1b] idx 6, BC    : profile 12 */
		};
		unsigned int k;

		for (k = 0; k < ARRAY_SIZE(nkpol_d); k++) {
			writel(0, ni_base(ni) + CA_NI_L2FE_DSCP_TE_DATA);
			writel(0, ni_base(ni) + CA_NI_L2FE_DSCP_TE_ACCESS);
			writel(nkpol_d[k], ni_base(ni) + CA_NI_L2FE_NKPOL_MAP_DATA);
			cortina_ni_rx_settle();
			cortina_ni_rx_ind_store(ni, CA_NI_L2FE_NKPOL_MAP_ACCESS,
						CA_NI_RX_CPU_LDPID + k);	/* = row 0x10 + k = {unkwn_pol_idx, pkt_type}, NOT an ldpid */
			cortina_ni_rx_settle();
		}
		dev_info(ni->dev,
			 "tbl@0x1634 (ex-\"MC_FIB\", stock-match): [0x19]=0x%08x (want 0x0b); [0x10..0x1b] set\n",
			 cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_NKPOL_MAP_ACCESS,
						 CA_NI_RX_L3LAN_LDPID,
						 CA_NI_L2FE_NKPOL_MAP_DATA));
	}

	/* Readback proves the vector contents, not the packet's destination. */
	cortina_ni_rx_ind_read(ni, CA_NI_NI_MCE_INDX_ACCESS, CA_NI_RX_DFT_FWD_MCGID);
	dev_info(ni->dev,
		 "mc-group[0x%x]: mc_vec hi(0xaaf8)=0x%08x lo(0xaafc)=0x%08x (expected 0/0; CPU delivery unverified)\n",
		 CA_NI_RX_DFT_FWD_MCGID,
		 readl(ni_base(ni) + CA_NI_NI_MCE_INDX_DATA1),
		 readl(ni_base(ni) + CA_NI_NI_MCE_INDX_DATA0));

	/* Optional boot-time redirect; validate delivery externally. */
	if (redir_cpu_ldpid) {
		cortina_ni_rx_redir_ldpid_set(ni, CA_NI_RX_DFT_FWD_MCGID,
					      redir_cpu_ldpid);
		dev_info(ni->dev,
			 "mc-group: REDIR_LDPID[0x%x]->0x%x forced (fallback)\n",
			 CA_NI_RX_DFT_FWD_MCGID, redir_cpu_ldpid);
	}
}

/* ★ THE own-MAC CPU trap: install a static L2 FDB entry {our ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 61. */
static int cortina_ni_l2fe_fdb_read_idx(void __iomem *base, u32 d3, u32 d2,
					u32 d1, u32 *act_out)
{
	u32 acc, cr;
	int idx;

	writel(0, base + CA_NI_L2FE_FDB_CMD_RETURN);
	writel(d3, base + CA_NI_L2FE_FDB_DATA3);
	writel(d2, base + CA_NI_L2FE_FDB_DATA2);
	writel(d1, base + CA_NI_L2FE_FDB_DATA1);
	if (ca_ni_access_go(base + CA_NI_L2FE_FDB_ACCESS,
			    CA_NI_L2FE_FDB_GO | CA_NI_L2FE_FDB_OP_READ, &acc))
		return -1;
	cr = readl(base + CA_NI_L2FE_FDB_CMD_RETURN);
	idx = cortina_ni_l2fe_fdb_cmd_status_idx(cr);
	if (idx < 0)
		return -1;			/* not present */
	if (act_out)
		*act_out = readl(base + CA_NI_L2FE_FDB_DATA0);
	return idx;
}

int cortina_ni_l2fe_fdb_add_idx(void __iomem *base, const u8 *mac, u32 ldpid)
{
	u32 d0, d1, d2, d3, acc;

	cortina_ni_l2fe_fdb_key(mac, &d3, &d2, &d1);
	d0 = cortina_ni_l2fe_fdb_action(ldpid);

	writel(0, base + CA_NI_L2FE_FDB_CMD_RETURN);
	writel(d3, base + CA_NI_L2FE_FDB_DATA3);
	writel(d2, base + CA_NI_L2FE_FDB_DATA2);
	writel(d1, base + CA_NI_L2FE_FDB_DATA1);
	writel(d0, base + CA_NI_L2FE_FDB_DATA0);
	if (ca_ni_access_go(base + CA_NI_L2FE_FDB_ACCESS,
			    CA_NI_L2FE_FDB_GO | CA_NI_L2FE_FDB_OP_APPEND,
			    &acc))
		return -1;

	/* READ back the key: CMD_RETURN.status[3:0]=0x5 HIT, ext_status[16:4]=idx */
	return cortina_ni_l2fe_fdb_read_idx(base, d3, d2, d1, NULL);
}

/* LOOK UP @mac in the L2FE FDB without touching the table, ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 62. */
int cortina_ni_l2fe_fdb_lookup_idx(void __iomem *base, const u8 *mac,
				   u32 *ldpid_out)
{
	u32 d1, d2, d3, act = 0;
	int idx;

	cortina_ni_l2fe_fdb_key(mac, &d3, &d2, &d1);
	idx = cortina_ni_l2fe_fdb_read_idx(base, d3, d2, d1, &act);
	if (idx < 0)
		return -1;
	if (!cortina_ni_l2fe_fdb_action_da(act, ldpid_out))
		return -1;		/* present but not forwardable as a DA */
	return idx;
}

static void cortina_ni_rx_fdb_append(struct cortina_ni *ni, const u8 *mac,
				     u32 ldpid)
{
	int idx = cortina_ni_l2fe_fdb_add_idx(ni_base(ni), mac, ldpid);

	dev_info(ni->dev, "fdb-add: %pM -> ldpid 0x%02x : entry_idx=%d %s\n",
		 mac, ldpid, idx, idx < 0 ? "(FAILED)" : "(HIT)");
}

/* No netdev, no key: this and cortina_ni_rx_mymac_trap() ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 168. */
static void cortina_ni_rx_fdb_add_cpu(struct cortina_ni *ni)
{
	const u8 *mac;
	u32 acc;
	int ret;

	if (!ni->tx || !ni->tx->netdev) {
		dev_warn(ni->dev,
			 "fdb-add: no netdev yet -- refusing to key the FDB on an address this board does not hold\n");
		return;
	}
	mac = ni->tx->netdev->dev_addr;

	/* (0) ★ one-time FDB engine INIT (opcode 0) - the hash table ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 169. */
	writel(0, ni_base(ni) + CA_NI_L2FE_FDB_CMD_RETURN);
	ret = ca_ni_access_go_paced(ni_base(ni) + CA_NI_L2FE_FDB_ACCESS,
				    CA_NI_L2FE_FDB_GO | CA_NI_L2FE_FDB_OP_INIT,
				    &acc, CA_NI_FDB_INIT_POLL_US,
				    CA_NI_FDB_INIT_POLL_TIMEOUT_US);
	if (ret)
		dev_warn(ni->dev, "fdb-add: engine INIT GO stuck\n");

	/* (1) router (LAN) MAC -> L3_LAN (0x19): stock's my-MAC route */
	cortina_ni_rx_fdb_append(ni, mac, CA_NI_RX_L3LAN_LDPID);

	/* (2) ★ WAN MAC (= base+1, the stock-confirmed per-board ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 63. */
	if (cortina_ni_hw_l3_fwd_active()) {
		u8 wan_mac[ETH_ALEN];

			/* REBASED, not re-spelled: flowcore's l3fe_wan_mac_derive()
			 * is the ONE copy of the 48-bit +1-with-carry.  (With
			 * CONFIG_CORTINA_NI_FLOWOFFLOAD unset the gate above folds to
			 * constant false and the call is compiled out.) */
		l3fe_wan_mac_derive(mac, wan_mac);
		cortina_ni_rx_fdb_append(ni, wan_mac, CA_NI_RX_L3WAN_LDPID);
	}
}

/* ★ THE own-MAC CPU trap (architectural, via the L3FE). ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 64. */
static void cortina_ni_rx_mymac_trap(struct cortina_ni *ni)
{
	bool l2_trap = !cortina_ni_hw_l3_fwd_active();
	u32 det, ctrl, hi_p0;
	const u8 *mac;

	/* see the note above cortina_ni_rx_fdb_add_cpu(): no netdev, no key. */
	if (!ni->tx || !ni->tx->netdev) {
		dev_warn(ni->dev,
			 "mymac: no netdev yet -- refusing to arm the comparator on an address this board does not hold\n");
		return;
	}
	mac = ni->tx->netdev->dev_addr;

	/* (A) NI-global my-MAC: CFG0=bytes0-3, CFG1[7:0]=byte4, ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 65. */
	dev_emerg(ni->dev, "MYMAC 1: comparator A (0xa024/a028/a5c0)\n");
	if (l2_trap) {
		writel(((u32)mac[0] << 24) | ((u32)mac[1] << 16) |
		       ((u32)mac[2] << 8) | mac[3],
		       ni_base(ni) + CA_NI_L3FE_NI_MAC_CFG0);
		ni_rmw(ni, CA_NI_L3FE_NI_MAC_CFG1, CA_NI_L3FE_NI_MAC_BYTE4,
		       mac[4]);
		ni_rmw(ni, CA_NI_L3FE_PT_PORT_STATIC_CFG,
		       CA_NI_L3FE_PT_MAC_BYTE5,
		       FIELD_PREP(CA_NI_L3FE_PT_MAC_BYTE5, mac[5]));
	} else {
		writel(0, ni_base(ni) + CA_NI_L3FE_NI_MAC_CFG0);
		ni_rmw(ni, CA_NI_L3FE_NI_MAC_CFG1, CA_NI_L3FE_NI_MAC_BYTE4, 0);
		ni_rmw(ni, CA_NI_L3FE_PT_PORT_STATIC_CFG,
		       CA_NI_L3FE_PT_MAC_BYTE5, 0);
	}

	/* enable my-MAC detection (0x3400 b21 chk_mymac_for_lan left 0 = stock).
	 * Unconditional - kept SET under hw_l3_fwd too (stock keeps it set;
	 * clearing it broke GPON/OMCI - see the block comment above). */
	dev_emerg(ni->dev, "MYMAC 2: my_mac_enable (0x3218 bit2)\n");
	ni_rmw(ni, CA_NI_L3FE_SPCL_PKT_DET_CFG, 0, CA_NI_L3FE_MY_MAC_EN);

	if (!l2_trap)
		dev_info(ni->dev,
			 "mymac-trap: comparator A cleared under hw_l3_fwd (0x3218 my_mac_enable kept stock/set) -> transit rides FDB into L3FE\n");

	/* ★ match stock STG0 LPB profiles + ldpid_map (tier-1; ours ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 170. */
	dev_emerg(ni->dev, "MYMAC 3: STG0 LPB match (0x3404-3434)\n");
	/* ★ WAN LPB profiles (prof0 @HIGH0, prof2 @HIGH2) = stock ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 66. */
	hi_p0 = CA_NI_L3FE_LPB_HIGH_P0;
	writel(CA_NI_L3FE_STG0_LDPID_MAP_VAL,
	       ni_base(ni) + CA_NI_L3FE_STG0_LDPID_MAP);
	writel(0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW0);
	writel(CA_NI_L3FE_LPB_MID_SEL0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID0);
	writel(hi_p0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH0);
	writel(0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW1);
	writel(CA_NI_L3FE_LPB_MID_SEL1, ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID1);
	writel(CA_NI_L3FE_LPB_HIGH_P1, ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH1);
	writel(0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW2);
	writel(CA_NI_L3FE_LPB_MID_SEL0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID2);
	writel(hi_p0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH2);
	writel(0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW3);
	writel(CA_NI_L3FE_LPB_MID_SEL1, ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID3);
	writel(CA_NI_L3FE_LPB_HIGH_P3, ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH3);

	/* ★ Program the L3FE STG0 my-MAC (0x3210/0x3214) from our per-board MAC + the
	 * valid bit (0x3210 bit16).  Tier-1 stock diff (2026-07-12): stock sets these to
	 * its board MAC WITH the valid bit; ours were 0 (no MAC, valid clear).  Encoding:
	 * LO = valid | mac[0]<<8 | mac[1];  HI = mac[2..5]. */
	writel(CA_NI_L3FE_MY_MAC_VALID | ((u32)mac[0] << 8) | mac[1],
	       ni_base(ni) + CA_NI_L3FE_MY_MAC_LO);
	writel(((u32)mac[2] << 24) | ((u32)mac[3] << 16) | ((u32)mac[4] << 8) | mac[5],
	       ni_base(ni) + CA_NI_L3FE_MY_MAC_HI);

	dev_info(ni->dev,
		 "l3fe-mymac: tcp_cos2=0x%08x remap_ctrl=0x%08x hash_smac/dmac=0x%08x/0x%08x my_mac lo=0x%08x hi=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_TCP_COS_MOD_2),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_CTRL),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_SMAC_PRO),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_DMAC_PRO),
		 readl(ni_base(ni) + CA_NI_L3FE_MY_MAC_LO),
		 readl(ni_base(ni) + CA_NI_L3FE_MY_MAC_HI));

	dev_emerg(ni->dev, "MYMAC 4 (survived - no SError)\n");
	det = readl(ni_base(ni) + CA_NI_L3FE_SPCL_PKT_DET_CFG);
	ctrl = readl(ni_base(ni) + CA_NI_L3FE_STG0_CTRL);
	dev_info(ni->dev,
		 "mymac-trap: %pM niA cfg0=0x%08x cfg1=0x%08x pt(a5c0)=0x%08x | det=0x%08x(myen=%u) stg0=0x%08x(chklan=%u)\n",
		 mac, readl(ni_base(ni) + CA_NI_L3FE_NI_MAC_CFG0),
		 readl(ni_base(ni) + CA_NI_L3FE_NI_MAC_CFG1),
		 readl(ni_base(ni) + CA_NI_L3FE_PT_PORT_STATIC_CFG),
		 det, !!(det & CA_NI_L3FE_MY_MAC_EN),
		 ctrl, !!(ctrl & CA_NI_L3FE_CHK_MYMAC_LAN));
	dev_info(ni->dev,
		 "mymac-trap: STG0 hi[0-3]=0x%08x 0x%08x 0x%08x 0x%08x lo0=0x%08x mid0=0x%08x ldpid_map=0x%08x (stock hi0=0x18100190, spcl_pkt_en=bit20)\n",
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH0),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH1),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH2),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH3),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW0),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID0),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LDPID_MAP));
}

/* ★ Re-key every MAC-keyed admission/offload table from the ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 67. */
void cortina_ni_rx_mac_rearm(struct cortina_ni *ni)
{
	if (!ni->rx || !ni->tx || !ni->tx->netdev ||
	    !cortina_ni_hw_l3_fwd_active())
		return;

	cortina_ni_flowoffload_router_mac_set(ni->tx->netdev->dev_addr);
	cortina_ni_rx_fdb_add_cpu(ni);
	cortina_ni_rx_mymac_trap(ni);
	cortina_l3fe_intf_add(ni_base(ni), ni->tx->netdev->dev_addr);
	dev_info(ni->dev,
		 "MAC-keyed admission re-armed for %pM (FDB + my-MAC + router-CAM)\n",
		 ni->tx->netdev->dev_addr);
}

/* Program one REDIR_LDPID_CONFIG entry: idx (a redir-LDPID) -> real dest LDPID.
 * Indirect table: write DATA, then ACCESS = GO|WR|idx, poll GO clear (stock aal
 * FIND_INDIRCT_ADDRESS / CHECK_INDIRCT_OPERATE_STATE).  Bounded + non-fatal. */
static void __maybe_unused
cortina_ni_rx_redir_ldpid_set(struct cortina_ni *ni, u8 idx,
			      u8 dest_ldpid)
{
	int ret;

	writel(FIELD_PREP(CA_NI_L2FE_REDIR_RDIR_LDPID, dest_ldpid) |
	       CA_NI_L2FE_REDIR_RDIR_EN,
	       ni_base(ni) + CA_NI_L2FE_REDIR_LDPID_DATA);
	ret = cortina_ni_rx_ind_store(ni, CA_NI_L2FE_REDIR_LDPID_ACCESS, idx);
	if (ret)
		dev_warn(ni->dev,
			 "redir_ldpid[0x%x]->0x%x: ACCESS GO stuck\n",
			 idx, dest_ldpid);
}

/* L2FE forwarding-control regs = stock's live values ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 68. */
static void cortina_ni_rx_l2fe_forwarding(struct cortina_ni *ni)
{
	unsigned int i;

	writel(CA_NI_L2FE_DPID_FWD_CTRL_VAL,
	       ni_base(ni) + CA_NI_L2FE_PLC_DPID_FWD_CTRL);
	writel(CA_NI_L2FE_LRN_FWD_CTRL_0_VAL,
	       ni_base(ni) + CA_NI_L2FE_PLC_LRN_FWD_CTRL_0);
	writel(CA_NI_L2FE_LRN_FWD_CTRL_1_VAL,
	       ni_base(ni) + CA_NI_L2FE_PLC_LRN_FWD_CTRL_1);
	writel(CA_NI_L2FE_PLE_DEFAULT_VAL,
	       ni_base(ni) + CA_NI_L2FE_PLE_DEFAULT_REG);
	/* arb_ctrl: dbuf_dpid[7:4]=8 (deep_q comparator) AND cpu_dpid[27:24]=9 (the CPU
	 * comparator - a resolved PDPID==9 is what should set the header cpu_flg).  Ours
	 * matches stock 0x89c71c82, so the comparators are correct. */
	writel(CA_NI_L2FE_ARB_CTRL_VAL, ni_base(ni) + CA_NI_L2FE_ARB_CTRL);
	/* ★ override the deep_q trigger dbuf_dpid[7:4] (0xf = disable, so the
	 * pdpid-0x08 CPU frame is deep_q=0 and takes the normal BM path to L3QM). */
	ni_rmw(ni, CA_NI_L2FE_ARB_CTRL, CA_NI_L2FE_ARB_DBUF_DPID,
	       FIELD_PREP(CA_NI_L2FE_ARB_DBUF_DPID, arb_dbuf_dpid));
	dev_info(ni->dev, "arb: dbuf_dpid(deep_q trigger)=0x%x -> arb_ctrl=0x%08x (0xf=deep_q OFF/normal path)\n",
		 arb_dbuf_dpid, readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL));
	writel(CA_NI_L2FE_ARB_REG_1608_VAL,
	       ni_base(ni) + CA_NI_L2FE_ARB_REG_1608);
	writel(CA_NI_L2FE_ARB_CTRL_EXT_VAL,
	       ni_base(ni) + CA_NI_L2FE_ARB_CTRL_EXT);
	/* ★★ ARB per-port allow/classify masks (0x1614-0x1620) = stock 0xFFFFFFFF each.
	 * We never wrote them (reset 0 = deny), the top suspect for cpu_flg staying 0
	 * despite a resolved PDPID==cpu_dpid(9): a 0 mask masks CPU classification off. */
	for (i = 0; i < CA_NI_L2FE_ARB_ALLOW_MASK_COUNT; i++)
		writel(CA_NI_L2FE_ARB_ALLOW_MASK_VAL,
		       ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(i));

	dev_info(ni->dev,
		 "l2fe-fwd: arb_ctrl=0x%08x reg1608=0x%08x allow[0..3]=0x%08x/0x%08x/0x%08x/0x%08x ple_dflt=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_REG_1608),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(0)),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(1)),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(2)),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(3)),
		 readl(ni_base(ni) + CA_NI_L2FE_PLE_DEFAULT_REG));
}

/* RX steer = the FULL FORWARDING-ENGINE path, matching ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 69. */
static unsigned int dq_tmport_map = 0x76543210u;	/* build68 STOCK identity: DQ N -> TM port N.  0x88888888 (build39) forced ALL DQs to ES port 8 = L3LAN dead-end -> frame never reached ES7/L3QM -> rmu_rx=0.  THE routing bug. */
module_param(dq_tmport_map, uint, 0444);
MODULE_PARM_DESC(dq_tmport_map, "Boot-only: DQ-to-TM-port map (0x76543210=stock identity)");

static void cortina_ni_rx_flow_ctrl_init(struct cortina_ni *ni)
{
	int i;

	/* flow-control enable (RMW OR bit19) */
	ni_rmw(ni, CA_NI_NI_FLOWCTRL_EN, 0, CA_NI_NI_FLOWCTRL_EN_BIT);

	/* BM dequeue -> TM-port uplink-flag map: every TM-port -> L3QM/ES8 (= QM/CPU) */
	writel(CA_NI_L2TM_BM_DQ_PORT_MAP_VAL,
	       ni_base(ni) + CA_NI_L2TM_BM_DQ_PORT_MAP);

	/* ★★ the PHYSICAL DQ->TM-port NUMBER map (0x212c), then read back the REAL
	 * 0x2124 + 0x212c so dmesg proves what the HW actually holds (this settles
	 * the "is 0x2124 a /proc shadow?" question). */
	writel(dq_tmport_map, ni_base(ni) + CA_NI_L2TM_BM_DQ_TO_TM_PORT_MAP);
	dev_info(ni->dev,
		 "bm-dq-map: REAL 0x2124=0x%08x (uplink-flag, want 0x88888888) 0x212c=0x%08x (phys DQ->TMport, set 0x%08x)\n",
		 readl(ni_base(ni) + CA_NI_L2TM_BM_DQ_PORT_MAP),
		 readl(ni_base(ni) + CA_NI_L2TM_BM_DQ_TO_TM_PORT_MAP),
		 dq_tmport_map);

	/* per-port flow-control thresholds 0x9798..0x97b0 (stock bfxil+bfi) */
	for (i = 0; i < CA_NI_NI_FLOWCTRL_THRESH_CNT; i++)
		ni_rmw(ni, CA_NI_NI_FLOWCTRL_THRESH + i * 4,
		       CA_NI_NI_FLOWCTRL_THRESH_CLR, CA_NI_NI_FLOWCTRL_THRESH_VAL);

	/* ★★ L2TM TM-egress -> CPU-queue map block (stock values; U-Boot leaves these,
	 * esp. 0x2100, unset).  Without them a non-deep CPU frame is BM-dequeued (tm
	 * tx++) but routed to the wrong TM output and never reaches RMU0's CPU queue
	 * (0x6900=0, no drop).  Match stock byte-for-byte. */
	writel(CA_NI_L2TM_TM_CFG_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_CFG);
	writel(CA_NI_L2TM_TM_MAP_A_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_MAP_A);
	writel(CA_NI_L2TM_TM_MAP_B_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_MAP_B);
	writel(CA_NI_L2TM_TM_TO_CPUQ_VAL, ni_base(ni) + CA_NI_L2TM_TM_TO_CPUQ_MAP);
	writel(CA_NI_L2TM_TM_MAP_C_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_MAP_C);
	writel(CA_NI_L2TM_TM_MAP_D_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_MAP_D);
	dev_info(ni->dev,
		 "l2tm tm-map: 0x2100=0x%08x 0x2114=0x%08x 0x2118=0x%08x 0x2120=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L2TM_TM_CFG),
		 readl(ni_base(ni) + CA_NI_L2TM_TM_MAP_B),
		 readl(ni_base(ni) + CA_NI_L2TM_TM_TO_CPUQ_MAP),
		 readl(ni_base(ni) + CA_NI_L2TM_TM_MAP_D));

	/* ★★ NI->L3QM ingress-handoff flow-control the frame must PRESENT to the QM
	 * (stock __ni_flow_ctrl_init + aal_ni_rxmux_fc_thrshld_set + the NI-HV RXFIFO
	 * threshold).  Missing, the NIRX->L3QM FIFO back-pressures and the QM ingress
	 * stays 0 even though the L2TM egresses.  Tier-1 stock live values. */
	ni_rmw(ni, CA_NI_NI_FC_2914, 0, CA_NI_NI_FC_2914_EN);	/* 0x2914 |= bit31 */
	for (i = 0; i < CA_NI_NI_RXMUX_FC_THR_CNT; i++)
		writel(i < CA_NI_NI_RXMUX_FC_THR_LO_CNT ?
			       CA_NI_NI_RXMUX_FC_THR_LO_VAL :
			       CA_NI_NI_RXMUX_FC_THR_HI_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_NI_RXMUX_FC_THR, i,
						CA_NI_NI_RXMUX_FC_THR_COUNT));
	writel(CA_NI_NI_RXFIFO_THR_B0_VAL, ni_base(ni) + CA_NI_NI_RXFIFO_THR_B0);
	writel(CA_NI_NI_RXFIFO_THR_B4_VAL, ni_base(ni) + CA_NI_NI_RXFIFO_THR_B4);
	writel(CA_NI_NI_RXFIFO_THR_B8_VAL, ni_base(ni) + CA_NI_NI_RXFIFO_THR_B8);
	dev_info(ni->dev,
		 "ni->qm handoff: 0x2914=0x%08x rxmux_fc[0]=0x%08x rxfifo_thr(0xa1b8)=0x%08x (l3qm_rxfifo_hi[6:0]=0x%02x)\n",
		 readl(ni_base(ni) + CA_NI_NI_FC_2914),
		 readl(ni_base(ni) + CA_NI_NI_RXMUX_FC_THR(0)),
		 readl(ni_base(ni) + CA_NI_NI_RXFIFO_THR_B8),
		 readl(ni_base(ni) + CA_NI_NI_RXFIFO_THR_B8) & 0x7f);
}

/* ★★ Enable the L2TM egress scheduler (stock aal_l2_tm_init). ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 70. */
static void cortina_ni_rx_l2tm_es_init(struct cortina_ni *ni)
{
	int i;

	/* tx_en + per-port enable (ports 0-13,15) - preserve the reset deglitch
	 * bits (26,29) via RMW-OR */
	ni_rmw(ni, CA_NI_L2TM_ES_CTRL, 0,
	       CA_NI_L2TM_ES_TX_EN | CA_NI_L2TM_ES_PORT_EN_ALL);

	/* per-scheduler VOQ enable (bits[7:0]); reset default is already 0xff but
	 * re-assert so instance 8 (L3QM) definitely drains all VOQs */
	for (i = 0; i < CA_NI_L2TM_ES_SCH_INSTANCES; i++)
		ni_rmw(ni, CA_NI_L2TM_ES_SCH_CFG(i), 0, CA_NI_L2TM_ES_VOQ_EN_ALL);

	dev_info(ni->dev,
		 "l2tm-es: es_ctrl=0x%08x sch0=0x%08x sch8(L3QM)=0x%08x (want es_ctrl bit31+bit8 set, sch voq_en=0xff)\n",
		 readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL),
		 readl(ni_base(ni) + CA_NI_L2TM_ES_SCH_CFG(0)),
		 readl(ni_base(ni) + CA_NI_L2TM_ES_SCH_CFG(CA_NI_L2TM_ES_PORT_L3QM)));
}

/* A register RUN: `count` consecutive u32 registers (stride ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 71. */
struct cortina_ni_reg_run {
	u16 base;
	u16 count;
	u32 val;
};

/* Expand `runs` in declaration order: for each, write `val` to `count`
 * consecutive registers from `base`.  Returns how many registers were
 * written, so a caller can log the expanded count, not the run count. */
static unsigned int cortina_ni_rx_write_runs(struct cortina_ni *ni,
					     const struct cortina_ni_reg_run *runs,
					     unsigned int nruns)
{
	unsigned int i, k, wrote = 0;

	for (i = 0; i < nruns; i++)
		for (k = 0; k < runs[i].count; k++, wrote++)
			writel(runs[i].val,
			       ni_base(ni) + runs[i].base + 4 * k);
	return wrote;
}

/* ★★ Deep-queue / central-buffer SCHEDULER init (L2TE_CB + ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 72. */
static const struct cortina_ni_reg_run cortina_ni_deepq_cb_cfg[] = {
	/* L2TM ES per-scheduler cfg words for instances 8, 10, 13 (instance 8 =
	 * CA_NI_L2TM_ES_PORT_L3QM, the port that drains the deep queue); low byte
	 * 0xff = the voq_en field l2tm_es_init re-asserts */
	{ CA_NI_L2TM_ES_SCH_CFG(8), 1, 0x000600ffu },
	{ CA_NI_L2TM_ES_SCH_CFG(10), 1, 0x000600ffu },
	{ CA_NI_L2TM_ES_SCH_CFG(13), 1, 0x000600ffu },
	{ 0x23b8, 3, 0x01010101u },	/* ..0x23c0 */
	{ 0x23c4, 1, 0x01000101u },
	{ 0x23c8, 1, 0x00000014u },
	{ 0x23cc, 1, 0x00000740u },
	{ 0x23d0, 1, 0x00003fffu },
	{ 0x23e4, 5, 0xffffffffu },	/* ..0x23f4 */
	{ 0x2404, 1, 0x0700000fu },
	{ 0x2410, 1, 0x8700f000u },
	{ 0x2414, 1, 0x000007ffu },
	{ 0x241c, 1, 0x20000f00u },
	{ 0x2500, 1, 0x00000500u },
	{ 0x2504, 15, 0x00000502u },	/* ..0x253c */
	{ 0x2540, 1, 0x4000000au },
	{ 0x2544, 1, 0x00000064u },
	{ 0x2548, 1, 0x00017c01u },
	{ 0x254c, 1, 0x900005f0u },
	{ 0x2550, 19, 0x00000502u },	/* ..0x2598 */
	{ 0x259c, 1, 0x40000006u },
	{ 0x25a0, 1, 0x00000040u },
	{ 0x25a4, 1, 0x7ffff9ffu },
	{ 0x25a8, 1, 0xfdffffe7u },
	{ 0x25ac, 11, 0x00000502u },	/* ..0x25d4 */
	{ 0x25f4, 1, 0x2ff3e723u },
	{ 0x25f8, 1, 0x000001f3u },
	{ 0x25fc, 4, 0x001fffffu },	/* ..0x2608 */
	{ 0x260c, 1, 0xdc0087c0u },
	{ 0x2700, 1, 0x14141414u },
	{ 0x2714, 1, 0x0000007cu },
	{ 0x2718, 1, 0x40000006u },
	{ 0x271c, 1, 0x3ffffe01u },
	{ 0x2720, 1, 0x01ffffe7u },
	{ 0x2748, 1, 0x001fffffu },
	{ 0x274c, 1, 0x2ff3e723u },
	{ 0x2750, 1, 0x000001f3u },
	/* 0x2d80..0x2d98: DIRECT DQSCH regs whose 0x7fff7fff/0x7fff3fff MATCH
	 * stock - NOT the indexed VOQ profile tables (see the "TWO different
	 * things at 0x2d../0x2e.." comment below) */
	{ 0x2d7c, 1, 0x02000010u },
	{ 0x2d80, 1, 0x7fff7fffu },
	{ 0x2d84, 1, 0x03000010u },
	{ 0x2d88, 1, 0x7fff7fffu },
	{ 0x2d8c, 1, 0x7fff3fffu },
	{ 0x2d90, 1, 0x7fff7fffu },
	{ 0x2d94, 1, 0x7fff3fffu },
	{ 0x2d98, 1, 0x7fff7fffu },
	{ 0x2db0, 1, 0x00000002u },
	/* stock's RESTING values of the CB port-freecnt indirect ACCESS/DATA
	 * pair; step (2) of deepq_sched_init later drives the same pair
	 * properly, once per port */
	{ CA_NI_L2TM_CB_PORT_FREECNT_ACCESS, 1, 0x4000000fu },
	{ CA_NI_L2TM_CB_PORT_FREECNT_DATA, 1, 0x8e308000u },
	{ 0x2dcc, 1, 0x0e200020u },
	{ 0x2dd0, 8, 0x00200010u },	/* ..0x2dec */
	{ 0x2df4, 4, 0x0c000100u },	/* ..0x2e00 */
	{ 0x2e08, 1, 0x02800110u },
	{ 0x2e0c, 7, 0x01800110u },	/* ..0x2e24 */
	{ 0x2e28, 1, 0x02800110u },
	{ 0x2e2c, 7, 0x01800060u },	/* ..0x2e44 */
	{ 0x2e58, 1, 0x00007fffu },
	{ 0x2e5c, 1, 0x00200020u },
	{ 0x2e60, 1, 0x05200020u },
	{ 0x2e64, 2, 0x3fff3fffu },	/* ..0x2e68 */
	/* 0x2ec8..0x2ee8: direct 0x7fff7fff words, stock-matching (see below) */
	{ 0x2ec8, 9, 0x7fff7fffu },	/* ..0x2ee8 */
};

/* ★★ UPSTREAM QUEUE DEPTH - the L2TM deep-queue per-VoQ ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 73. */
static unsigned int deepq_voq_thrsh = CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE;
static bool deepq_cb_stock;
/* set once the NI is probed, so a sysfs write can re-walk the tables */
static struct cortina_ni *cortina_ni_deepq_ni;

/* ★★★ THE UNI ADMINISTRATIVE LOCK IS A DESIRED STATE, NOT A ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 74. */
static u32 cortina_ni_uni_locked_ports;

/* ★★★ ONE OUTERMOST LOCK, AND A NON-TORN WORD IS NOT ENOUGH. ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 75. */
static DEFINE_MUTEX(cortina_ni_uni_lock);

bool cortina_ni_uni_port_locked(unsigned int port)
{
	return port < CA_NI_GPHY_COUNT &&
	       (READ_ONCE(cortina_ni_uni_locked_ports) & BIT(port));
}
EXPORT_SYMBOL_GPL(cortina_ni_uni_port_locked);

/* One read-modify-write of a port's global config with the ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 76. */
void cortina_ni_uni_port_glb_rmw(struct cortina_ni *ni, unsigned int port,
				 u32 clr, u32 set, u32 down_bit)
{
	mutex_lock(&cortina_ni_uni_lock);
	if (cortina_ni_uni_port_locked(port)) {
		set |= down_bit;
		clr &= ~down_bit;
	} else {
		clr |= down_bit;
		set &= ~down_bit;
	}
	ni_rmw(ni, CA_NI_PORT_GLB_CFG(port), clr, set);
	mutex_unlock(&cortina_ni_uni_lock);
}
EXPORT_SYMBOL_GPL(cortina_ni_uni_port_glb_rmw);

int cortina_ni_uni_admin_set(unsigned int port, bool locked)
{
	struct cortina_ni *ni;
	void __iomem *gphy, *bank;
	u32 want, val, glb, mac;
	int rc = 0;

	if (port >= CA_NI_GPHY_COUNT)
		return -EINVAL;

	mutex_lock(&cortina_ni_uni_lock);
	/* ★ THE DESIRED STATE IS RECORDED FIRST AND UNCONDITIONALLY.  Even with
	 *   no NI to drive yet, the next establish or link-up honours it -- so a
	 *   lock the OLT set during probe is late, never lost. */
	want = READ_ONCE(cortina_ni_uni_locked_ports);
	want = locked ? (want | BIT(port)) : (want & ~BIT(port));
	WRITE_ONCE(cortina_ni_uni_locked_ports, want);

	/* ⚠ EVERY DEREFERENCE OF ni HAPPENS UNDER THIS LOCK, the dev_info included.
	 *   unpublish() takes the same lock before nulling the publication, so a
	 *   captured pointer cannot outlive the object. */
	ni = READ_ONCE(cortina_ni_deepq_ni);
	if (!ni || !ni->mii) {
		mutex_unlock(&cortina_ni_uni_lock);
		return -ENODEV;		/* recorded, NOT applied: the caller retries */
	}

	/* ★★ BOTH DIRECTIONS, TOGETHER. Driving only the RX gate ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 77. */
	if (locked) {
		ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(port), CA_NI_PORT_RXMAC_RX_EN, 0);
		ni_rmw(ni, CA_NI_PORT_GLB_CFG(port), 0,
		       CA_NI_PORT_GLB_PWR_DWN_RX | CA_NI_PORT_GLB_PWR_DWN_TX);
	} else {
		ni_rmw(ni, CA_NI_PORT_GLB_CFG(port),
		       CA_NI_PORT_GLB_PWR_DWN_RX | CA_NI_PORT_GLB_PWR_DWN_TX, 0);
		ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(port), 0, CA_NI_PORT_RXMAC_RX_EN);
	}

	/* ★★ CONFIRMED, NOT ASSUMED.  An unconfirmed write reports success and the
	 *    model then believes a port is down that is still forwarding.  These bits
	 *    latch, so a read-back that disagrees means the write did not take - and
	 *    the safe answer is to leave the obligation PENDING. */
	glb = readl(ni_base(ni) + CA_NI_PORT_GLB_CFG(port));
	mac = readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(port));
	if (!!(glb & CA_NI_PORT_GLB_PWR_DWN_RX) != locked ||
	    !!(glb & CA_NI_PORT_GLB_PWR_DWN_TX) != locked ||
	    !!(mac & CA_NI_PORT_RXMAC_RX_EN) == locked) {
		dev_err(ni->dev,
			"UNI port %u: the MAC did not take the %s (glb=0x%08x rxmac=0x%08x)\n",
			port, locked ? "lock" : "unlock", glb, mac);
		rc = -EIO;
	}

	gphy = ni->win[CA_NI_WIN_GPHY];
	if (!gphy) {
		/* ⚠ NOT A SUCCESS.  This branch used to fall through and report
		 *   the port driven; without the GPHY window the PHY half never
		 *   happened at all. */
		dev_err(ni->dev,
			"UNI port %u: no GPHY window, the PHY half of the %s was not applied\n",
			port, locked ? "lock" : "unlock");
		rc = rc ? rc : -ENODEV;
	} else {
		bank = gphy + CA_NI_GPHY_BANK(port);
		mutex_lock(&ni->mii->mdio_lock);
		val = readl(bank + CA_NI_GPHY_BMCR);
		writel(locked ? (val | CA_NI_GPHY_BMCR_PDOWN)
			      : (val & ~CA_NI_GPHY_BMCR_PDOWN),
		       bank + CA_NI_GPHY_BMCR);
		val = readl(bank + CA_NI_GPHY_BMCR);
		mutex_unlock(&ni->mii->mdio_lock);
		if (!!(val & CA_NI_GPHY_BMCR_PDOWN) != locked) {
			dev_err(ni->dev,
				"UNI port %u: the PHY did not take the %s (bmcr=0x%08x)\n",
				port, locked ? "lock" : "unlock", val);
			rc = rc ? rc : -EIO;
		}
	}

	if (!rc)
		dev_info(ni->dev, "UNI port %u %s\n", port,
			 locked ? "LOCKED (RXMAC off, RX and PHY powered down)"
				: "unlocked");
	mutex_unlock(&cortina_ni_uni_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(cortina_ni_uni_admin_set);

/* Stop serving the published NI.  ⚠ IT WAS PUBLISHED AND NEVER WITHDRAWN: the
 * pointer outlived the devres free, so a sysfs write or an OMCI apply arriving
 * after teardown wrote through freed memory. */
void cortina_ni_rx_publish(struct cortina_ni *ni)
{
	mutex_lock(&cortina_ni_uni_lock);
	WRITE_ONCE(cortina_ni_deepq_ni, ni);
	mutex_unlock(&cortina_ni_uni_lock);
}

void cortina_ni_rx_unpublish(void)
{
	/* ⚠ UNDER THE SAME LOCK ITS READERS HOLD.  Nulling the publication only
	 *   stops NEW readers; one that already captured the pointer is still
	 *   inside, and devres frees the object the moment teardown returns. */
	mutex_lock(&cortina_ni_uni_lock);
	WRITE_ONCE(cortina_ni_deepq_ni, NULL);
	mutex_unlock(&cortina_ni_uni_lock);
}
static DEFINE_MUTEX(cortina_ni_deepq_lock);

/* Write the CURRENT parameter values into all 8 entries of BOTH per-VoQ threshold
 * profile tables (DQSCH @0x2e70/0x2e74 and CB @0x2da0/0x2da4/0x2da8).  Every
 * entry of every table, always - a partial walk would leave some VoQs deep and
 * make any measurement meaningless. */
static void cortina_ni_rx_deepq_thrsh_program(struct cortina_ni *ni)
{
	u32 dq = READ_ONCE(deepq_voq_thrsh);
	bool cbs = READ_ONCE(deepq_cb_stock);
	u32 cb1 = cbs ? CA_NI_L2TM_CB_VOQ_THRSH_D1
		      : CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE;
	u32 cb0 = cbs ? CA_NI_L2TM_CB_VOQ_THRSH_D0
		      : CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE;
	unsigned int i;

	mutex_lock(&cortina_ni_deepq_lock);
	for (i = 0; i < CA_NI_L2TM_DEEPQ_VOQ_ENTRIES; i++) {
		writel(dq, ni_base(ni) + CA_NI_L2TM_DQSCH_VOQ_THRSH_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS, i);

		writel(cb1, ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_DATA1);
		writel(cb0, ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_DATA0);
		cortina_ni_rx_ind_store(ni, CA_NI_L2TM_CB_VOQ_THRSH_ACCESS, i);
	}
	mutex_unlock(&cortina_ni_deepq_lock);

	dev_info(ni->dev,
		 "deepq-thrsh: %u entries x {dqsch(0x2e74)=0x%08x, cb(0x2da4/0x2da8)={0x%08x,0x%08x}} [dqsch stock=0x%08x permissive=0x%08x; cb stock={0x%08x,0x%08x}]\n",
		 CA_NI_L2TM_DEEPQ_VOQ_ENTRIES, dq, cb1, cb0,
		 CA_NI_L2TM_DQSCH_VOQ_THRSH_VAL,
		 CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE,
		 CA_NI_L2TM_CB_VOQ_THRSH_D1, CA_NI_L2TM_CB_VOQ_THRSH_D0);
}

/* Live readback of one entry of each table (indirect read: ACCESS = GO|idx, then
 * the entry is latched in DATA).  Bounded + non-fatal; the ACCESS word is printed
 * alongside so a GO that never cleared is visible instead of silently believed. */
static void cortina_ni_rx_deepq_thrsh_read(struct cortina_ni *ni, unsigned int idx,
					   u32 *dq, u32 *cb1, u32 *cb0)
{
	mutex_lock(&cortina_ni_deepq_lock);
	*dq = cortina_ni_rx_ind_entry(ni, CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS, idx,
				      CA_NI_L2TM_DQSCH_VOQ_THRSH_DATA);
	cortina_ni_rx_ind_read(ni, CA_NI_L2TM_CB_VOQ_THRSH_ACCESS, idx);
	*cb1 = readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_DATA1);
	*cb0 = readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_DATA0);
	mutex_unlock(&cortina_ni_deepq_lock);
}

static int cortina_ni_rx_deepq_thrsh_reprogram(void)
{
	struct cortina_ni *ni;

	/* ⚠ THE CAPTURE AND THE USE ARE ONE CRITICAL SECTION. This is ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 173. */
	mutex_lock(&cortina_ni_uni_lock);
	ni = READ_ONCE(cortina_ni_deepq_ni);
	/* set before probe (bootarg / insmod): the init walk picks it up */
	if (ni)
		cortina_ni_rx_deepq_thrsh_program(ni);
	mutex_unlock(&cortina_ni_uni_lock);
	return 0;
}

static int deepq_voq_thrsh_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_uint(val, kp);

	return ret ? ret : cortina_ni_rx_deepq_thrsh_reprogram();
}

static int deepq_cb_stock_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	return ret ? ret : cortina_ni_rx_deepq_thrsh_reprogram();
}

static const struct kernel_param_ops deepq_voq_thrsh_ops = {
	.set	= deepq_voq_thrsh_set,
	.get	= param_get_uint,
};
static const struct kernel_param_ops deepq_cb_stock_ops = {
	.flags	= KERNEL_PARAM_OPS_FL_NOARG,	/* bare name = 1, like a bool param */
	.set	= deepq_cb_stock_set,
	.get	= param_get_bool,
};

module_param_cb(deepq_voq_thrsh, &deepq_voq_thrsh_ops, &deepq_voq_thrsh, 0644);
MODULE_PARM_DESC(deepq_voq_thrsh,
	"L2TM DQSCH per-VoQ admission threshold, all 8 profile entries (0x2e70/0x2e74). 0x7fff7fff = permissive/DEFAULT (32767 per half, ~292x stock - the shipping behaviour, unchanged); 0x00700070 = the tier-1 stock value (112 per half); any other value is written verbatim. RUNTIME-settable (re-walks the table); read the active value back with 'grep deepq-thrsh /proc/net/cortina_ni_rx'");

module_param_cb(deepq_cb_stock, &deepq_cb_stock_ops, &deepq_cb_stock, 0644);
MODULE_PARM_DESC(deepq_cb_stock,
	"L2TM central-buffer per-VoQ threshold, all 8 profile entries (0x2da0/0x2da4/0x2da8). 0 = DEFAULT 0x7fff7fff in both words (the shipping behaviour); 1 = the tier-1 stock pair {hi=0x0fffffff, lo=0xffffffff}, which is DEEPER than ours, not shallower. RUNTIME-settable");

/* ★★ THE DEEP-QUEUE / DQSCH init (stock aal_l2_tm_cb_init) - ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 78. */
static void cortina_ni_rx_deepq_sched_init(struct cortina_ni *ni)
{
	unsigned int i;

	/* (1) DIRECT threshold/watermark/credit/per-queue config (stock resting values) */
	cortina_ni_rx_write_runs(ni, cortina_ni_deepq_cb_cfg,
				 ARRAY_SIZE(cortina_ni_deepq_cb_cfg));

	/* (2) CB per-port free-buffer count (indexed, all 48 ports) - without it the CB
	 * has 0 free deep-queue buffers and drops the frame at the L2TM->CB enqueue. */
	for (i = 0; i < CA_NI_L2TM_CB_PORT_COUNT; i++) {
		writel(CA_NI_L2TM_CB_FREECNT_VAL,
		       ni_base(ni) + CA_NI_L2TM_CB_PORT_FREECNT_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2TM_CB_PORT_FREECNT_ACCESS, i);
	}

	/* (3) the two INDEXED per-VOQ profile tables (8 entries each) ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 79. */
	cortina_ni_rx_deepq_thrsh_program(ni);

	/* (4) ARB_CTRL.dbuf_sel (bit1)=1 so a PDPID-8 frame takes the deep-buffer path */
	ni_rmw(ni, CA_NI_L2FE_ARB_CTRL, 0, CA_NI_L2FE_ARB_DBUF_SEL);

	/* ★★ (4b) THE DQSCH-OUTPUT -> TM-port binding (0x2f00/04/08) ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 174. */
	writel(CA_NI_L2TM_DQSCH_OUT_CFG0_VAL,	  ni_base(ni) + CA_NI_L2TM_DQSCH_OUT_CFG0);
	writel(CA_NI_L2TM_DQSCH_OUT_PORT_MAP_VAL, ni_base(ni) + CA_NI_L2TM_DQSCH_OUT_PORT_MAP);
	writel(CA_NI_L2TM_DQSCH_OUT_CFG2_VAL,	  ni_base(ni) + CA_NI_L2TM_DQSCH_OUT_CFG2);

	/* (5) THE TWO MASTER ENABLES - LAST, IN ORDER (cb_ctrl then abr_ctrl) */
	writel(CA_NI_L2TM_CB_CTRL_STOCK, ni_base(ni) + CA_NI_L2TM_CB_CTRL);
	writel(CA_NI_L2TM_CB_ABR_CTRL_STOCK, ni_base(ni) + CA_NI_L2TM_CB_ABR_CTRL);

	dev_info(ni->dev,
		 "deepq-sched: cb_ctrl(0x2d0c)=0x%08x abr(0x2eec)=0x%08x out_map(0x2f04)=0x%08x dqschA(0x2e70)=0x%08x cbB(0x2da0)=0x%08x arb=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L2TM_CB_CTRL),
		 readl(ni_base(ni) + CA_NI_L2TM_CB_ABR_CTRL),
		 readl(ni_base(ni) + CA_NI_L2TM_DQSCH_OUT_PORT_MAP),
		 readl(ni_base(ni) + CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS),
		 readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_ACCESS),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL));
}

/* ★★ Enable port MAC blocks 1-6 (stock enables all 7; ports ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 80. */
static void cortina_ni_rx_enable_internal_ports(struct cortina_ni *ni)
{
	int p;

	/* ★ A SIXTH RESTORING WRITER. This runs during probe and ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 175. */
	for (p = 1; p < CA_NI_PORT_COUNT; p++) {
		if (cortina_ni_uni_port_locked(p)) {
			writel(CA_NI_PORT_TXMAC_EN_VAL,
			       ni_base(ni) + CA_NI_PORT_TXMAC_CFG(p));
			writel(CA_NI_PORT_RX_CNTRL_STOCK_VAL,
			       ni_base(ni) + CA_NI_PORT_RX_CNTRL_CFG(p));
			continue;
		}
		writel(CA_NI_PORT_RXMAC_EN_VAL,
		       ni_base(ni) + CA_NI_PORT_RXMAC_CFG(p));
		writel(CA_NI_PORT_TXMAC_EN_VAL,
		       ni_base(ni) + CA_NI_PORT_TXMAC_CFG(p));
		writel(CA_NI_PORT_RX_CNTRL_STOCK_VAL,
		       ni_base(ni) + CA_NI_PORT_RX_CNTRL_CFG(p));
	}
	dev_info(ni->dev,
		 "ports 1-6 enabled (internal 5/6=CPU/QM/L3QM): p5 rxmac=0x%08x txmac=0x%08x; p6 rxmac=0x%08x txmac=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(5)),
		 readl(ni_base(ni) + CA_NI_PORT_TXMAC_CFG(5)),
		 readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(6)),
		 readl(ni_base(ni) + CA_NI_PORT_TXMAC_CFG(6)));
}

/* ★★★ The L3-CLS special-packet TRAP - VERBATIM replication ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 81. */
static bool cls_trap_enable = true;
module_param(cls_trap_enable, bool, 0444);
MODULE_PARM_DESC(cls_trap_enable, "Boot-only: install stock L3-CLS CPU_0 trap rows (ARP-to-CPU)");

/* stock KEY rows, struct word0..word10 (verbatim, reversed ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 176. */
static const struct { u16 idx; u32 w[CA_NI_L3FE_CLS_KEY_WORDS]; } cls_key_golden[] = {
	/* profile 0 (WAN) */
	{ 0, { 0xFFFFFEFF, 0xFEFFFFFF, 0xF77FFFFF, 0xFFFFFFFF, 0xFFFDFFFF,
	       0x0000003F, 0, 0, 0, 0, 0x18240000 } },
	{ 1, { 0xFFFFFECF, 0xCFFFFFFF, 0x0007FFFF, 0, 0,
	       0, 0, 0, 0, 0, 0x08040000 } },
	{ 2, { 0xFFFFFFFF, 0xFFFFFFFF, 0x0007FFFF, 0, 0,
	       0, 0, 0, 0, 0, 0x08000000 } },
	/* profile 1 (LAN) - identical rows at +64 */
	{ 64, { 0xFFFFFEFF, 0xFEFFFFFF, 0xF77FFFFF, 0xFFFFFFFF, 0xFFFDFFFF,
		0x0000003F, 0, 0, 0, 0, 0x18240000 } },
	{ 65, { 0xFFFFFECF, 0xCFFFFFFF, 0x0007FFFF, 0, 0,
		0, 0, 0, 0, 0, 0x08040000 } },
	{ 66, { 0xFFFFFFFF, 0xFFFFFFFF, 0x0007FFFF, 0, 0,
		0, 0, 0, 0, 0, 0x08000000 } },
};

/* stock FIB rows, struct word0..word6 (verbatim; word0=DATA0=0x33cc low .. word6=DATA6).
 * FIB idx = (key_row<<2)|slot: profile0 rows 0/1/2 -> 0,1,4,8; profile1 rows 64/65/66 ->
 * 256,257,260,264 (KEY64 slots0/1, KEY65 slot0, KEY66 slot0 = the LAN bcast catch-all). */
static const struct { u16 idx; u32 w[CA_NI_L3FE_CLS_FIB_WORDS]; } cls_fib_golden[] = {
	/* profile 0 (WAN) */
	{ 0,   { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },
	{ 1,   { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },
	{ 4,   { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000200 } },
	{ 8,   { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000600 } },
	/* profile 1 (LAN) */
	{ 256, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },
	{ 257, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },
	{ 260, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000200 } },
	{ 264, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000600 } },
};

static void cortina_ni_rx_l3fe_glb_init(struct cortina_ni *ni)
{
	writel(CA_NI_L3FE_GLB_CFG_VAL, ni_base(ni) + CA_NI_L3FE_GLB_CFG);
	writel(CA_NI_L3FE_GLB_LF_CFG_VAL, ni_base(ni) + CA_NI_L3FE_GLB_LF_CFG);
	writel(CA_NI_L3FE_GLB_TE_OPTION_VAL, ni_base(ni) + CA_NI_L3FE_GLB_TE_OPTION);
	writel(CA_NI_L3FE_CLS_MON_RETURN_VAL, ni_base(ni) + CA_NI_L3FE_CLS_MON_RETURN);
	writel(CA_NI_L3FE_GLB_DBG_DAT_VAL, ni_base(ni) + CA_NI_L3FE_GLB_DBG_DAT);
	writel(CA_NI_L3FE_GLB_RES_CTRL_VAL, ni_base(ni) + CA_NI_L3FE_GLB_RES_CTRL);

	/* CPU destination remapping and hash preprocessing. */
	writel(CA_NI_L3FE_GLB_LDPID_REMAP_CTRL_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_CTRL);
	writel(CA_NI_L3FE_GLB_LDPID_REMAP_SMAC_PRO_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_SMAC_PRO);
	writel(CA_NI_L3FE_GLB_LDPID_REMAP_DMAC_PRO_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_DMAC_PRO);
	writel(CA_NI_L3FE_GLB_LDPID_REMAP_SIP_PRO_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_SIP_PRO);
	writel(CA_NI_L3FE_GLB_LDPID_REMAP_DIP_PRO_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_DIP_PRO);
	writel(CA_NI_L3FE_GLB_LDPID_REMAP_SPORT_PRO_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_SPORT_PRO);
	writel(CA_NI_L3FE_GLB_LDPID_REMAP_DPORT_PRO_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_DPORT_PRO);

	dev_info(ni->dev,
		 "l3fe-glb-init: cfg=0x%08x lf_cfg=0x%08x cls_mon_return=0x%08x dbg_dat=0x%08x te_option=0x%08x res_ctrl=0x%08x remap_ctrl=0x%08x hash_smac/dmac=0x%08x/0x%08x hash_sip/dip=0x%08x/0x%08x hash_sport/dport=0x%08x/0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_CFG),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LF_CFG),
		 readl(ni_base(ni) + CA_NI_L3FE_CLS_MON_RETURN),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_DBG_DAT),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_TE_OPTION),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_RES_CTRL),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_CTRL),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_SMAC_PRO),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_DMAC_PRO),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_SIP_PRO),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_DIP_PRO),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_SPORT_PRO),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LDPID_REMAP_DPORT_PRO));
}

/* ★★ The L3FE AXI read-reorder channel init (vendor ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 84. */
static void cortina_ni_rx_l3fe_axi_reo_init(struct cortina_ni *ni)
{
	void __iomem *reo = ni->win[CA_NI_WIN_AXI_REO];

	if (!reo) {
		dev_warn(ni->dev, "RX: L3FE AXI-REO window (idx %d) not mapped\n",
			 CA_NI_WIN_AXI_REO);
		return;
	}
	writel(CA_NI_L3FE_AXI_REO_ORIG_ID_VAL, reo + CA_NI_L3FE_AXI_REO_ORIG_ID);
	writel(CA_NI_L3FE_AXI_REO_NEW_ID_VAL, reo + CA_NI_L3FE_AXI_REO_NEW_ID);
	writel(CA_NI_L3FE_AXI_REO_TOP_ADDR_VAL, reo + CA_NI_L3FE_AXI_REO_TOP_ADDR);
	writel(CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK_VAL,
	       reo + CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK);
	writel(CA_NI_L3FE_AXI_REO_NEW_ID0_VAL, reo + CA_NI_L3FE_AXI_REO_NEW_ID0);
	writel(CA_NI_L3FE_AXI_REO_RD18_VAL, reo + CA_NI_L3FE_AXI_REO_RD18);
	writel(CA_NI_L3FE_AXI_REO_RD24_VAL, reo + CA_NI_L3FE_AXI_REO_RD24);

	dev_info(ni->dev,
		 "l3fe-axi-reo (win10+0x480, abs 0xf432d480): orig=0x%08x new=0x%08x top=0x%08x mask=0x%08x new0=0x%08x +18=0x%08x +24=0x%08x (want 2/8|/1e8/1e8-mask/9|/FFFFFFFF x2)\n",
		 readl(reo + CA_NI_L3FE_AXI_REO_ORIG_ID),
		 readl(reo + CA_NI_L3FE_AXI_REO_NEW_ID),
		 readl(reo + CA_NI_L3FE_AXI_REO_TOP_ADDR),
		 readl(reo + CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK),
		 readl(reo + CA_NI_L3FE_AXI_REO_NEW_ID0),
		 readl(reo + CA_NI_L3FE_AXI_REO_RD18),
		 readl(reo + CA_NI_L3FE_AXI_REO_RD24));
}

static void cortina_ni_rx_cls_init(struct cortina_ni *ni)
{
	unsigned int e, i;

	if (!cls_trap_enable)
		return;

	/* ★★ Fix L3FE STG0_CTRL first - our reset default 0x001c7c7e has 2 bits (1,10)
	 * that stock's stg0_set_normal clears (lpb_idx_mode=0 etc).  A wrong
	 * lpb_idx_mode mis-indexes the STG0 LPB so the CLS lookup cannot match our LAN
	 * ingress, and byte-exact rows never fired.  Match stock's 0x001c787c. */
	writel(CA_NI_L3FE_STG0_CTRL_VAL, ni_base(ni) + CA_NI_L3FE_STG0_CTRL);

	/* commit ALL FIBs first, then ALL KEYs (stock aal_l3_cls_add order) */
	for (e = 0; e < ARRAY_SIZE(cls_fib_golden); e++) {
		for (i = 0; i < CA_NI_L3FE_CLS_FIB_WORDS; i++)
			writel(cls_fib_golden[e].w[i],
			       ni_base(ni) + CA_NI_L3FE_CLS_FIB_ACCESS +
			       (CA_NI_L3FE_CLS_FIB_WORDS - i) * 4);
		cortina_ni_rx_ind_store(ni, CA_NI_L3FE_CLS_FIB_ACCESS,
					cls_fib_golden[e].idx);
	}
	for (e = 0; e < ARRAY_SIZE(cls_key_golden); e++) {
		for (i = 0; i < CA_NI_L3FE_CLS_KEY_WORDS; i++)
			writel(cls_key_golden[e].w[i],
			       ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS +
			       (CA_NI_L3FE_CLS_KEY_WORDS - i) * 4);
		cortina_ni_rx_ind_store(ni, CA_NI_L3FE_CLS_KEY_ACCESS,
					cls_key_golden[e].idx);
	}

	/* ★ Tier-1 stock-diff (2026-07-23, live devmem on stock ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 85. */
	if (cortina_ni_hw_l3_fwd_active() && ni->tx && ni->tx->netdev) {
		int ret = cortina_l3fe_intf_add(ni_base(ni),
						ni->tx->netdev->dev_addr);

		dev_info(ni->dev, "cls: PP MAC-DA router-CAM re-applied %s\n",
			 ret ? "FAILED" : "ok");
	}

	/* read back the broadcast row (KEY[2]) for the boot log */
	cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_KEY_ACCESS, 2);
	dev_info(ni->dev,
		 "cls-trap: stg0_ctrl(0x3400)=0x%08x (want 0x001c787c); rows 0/1/2 + fib 0/1/4/8 written; KEY[2] w0(0x33ac)=0x%08x trailer(0x3384)=0x%08x det_cfg(0x3218)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_CTRL),
		 readl(ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS + 11 * 4),
		 readl(ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS + 1 * 4),
		 readl(ni_base(ni) + CA_NI_L3FE_SPCL_PKT_DET_CFG));
}

static int cortina_ni_rx_steer_init(struct cortina_ni *ni)
{
	int type, ret;

	/* ★ L2TM BM dequeue->TM-port map (stock __ni_flow_ctrl_init) - MUST run so
	 * a dequeued CPU-dest frame reaches the QM (else qm_rx_cntr stays 0). */
	cortina_ni_rx_flow_ctrl_init(ni);

	/* ★★ Deep-queue/central-buffer SCHEDULER (DQSCH) - drains the deep queue a
	 * deep_q=1 frame lands in, to the RMU.  Absent = frame enqueued but never
	 * drained (0x6900=0).  Must run alongside the L2TM ES enable. */
	cortina_ni_rx_deepq_sched_init(ni);

	/* ★★ Enable the L2TM egress scheduler (ES port 8 = L3QM) so the deep-queue
	 * frame is actually drained OUT of the L2TM to the QM -> RMU -> CPU.  At
	 * reset the scheduler is OFF (ES_CTRL=0x24000000), so without this the frame
	 * sits in the L2TM (tm rx climbs) and RMU0 never admits it (0x6900=0). */
	cortina_ni_rx_l2tm_es_init(ni);

	/* ★★ THE blackhole root-cause fix: L2FE per-port profiles (ILPB stp,
	 * MMSHP isolation, ELPB egr-stp, IPPB map).  Unprogrammed = force-DROP
	 * upstream of every forwarding table, resolving all frames to 0x1f. */
	cortina_ni_rx_port_profiles_init(ni);

	/* one-shot L2E hash (FDB) init: with stp=fwd+LEARN the engine now
	 * learns SAs and looks up DAs - an uninitialized hash SRAM could
	 * false-hit and mis-steer unicast.  Once only: re-running on a link
	 * bounce would flush learned entries. */
	{
		static bool fdb_hash_ready;

		if (!fdb_hash_ready) {
			u32 acc;

			if (ca_ni_access_go_paced(
				    ni_base(ni) + CA_NI_L2FE_FDB_ACCESS,
				    CA_NI_L2FE_FDB_GO | CA_NI_L2FE_FDB_OP_INIT,
				    &acc, CA_NI_FDB_INIT_POLL_US,
				    CA_NI_FDB_INIT_POLL_TIMEOUT_US))
				dev_warn(ni->dev, "fdb hash init timeout (0x%08x)\n",
					 acc);
			else
				fdb_hash_ready = true;
		}
	}

	/* L2FE forwarding-control regs (ARB/PLC/PLE_DEFAULT) = stock byte-for-byte;
	 * the DFT_FWD loop below sets the port-0 DLF -> CPU redir (0x1832).  No
	 * mc-flood (mce_indx/MC_FIB): stock uses plain REDIR, and the MCE path
	 * SErrored - it was never stock's mechanism. */
	cortina_ni_rx_l2fe_forwarding(ni);

	/* ★★ 2026-07-13 (ca-ne.ko RE + live-stock): the FDB GLOBAL ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 86. */
	writel(CA_NI_L2FE_FDB_CTRL_0_VAL, ni_base(ni) + CA_NI_L2FE_FDB_CTRL_0);
	writel(CA_NI_L2FE_FDB_CTRL_1_VAL, ni_base(ni) + CA_NI_L2FE_FDB_CTRL_1);

	/* ★★ 2026-07-13: initialise the FDB engine (OP_INIT builds the hash table) +
	 * add the own-MAC entry.  Without OP_INIT the FDB is dead, so the FDB
	 * forwarding-control never takes effect and DLF/broadcast frames fall through
	 * to DFT_FWD -> forwarded out (l3fe_rx=0). */
	cortina_ni_rx_fdb_add_cpu(ni);

	/* ★ ARB deep-queue: DeepQ_0 -> PDPID=QM -> ES port 7 -> RMU -> CPU (PDPID map
	 * + dbuf_dpid + REDIR[0x1f] belt).  Must run before the DFT_FWD redir. */
	cortina_ni_rx_arb_deepq_init(ni);
	cortina_ni_rx_flow_dbuf_init(ni);	/* zero the deep_q source (dbuf_sel=1) to stock - build100's 0x0f marks were the CPU-RX regression */

	/* ★ Build the REAL one-member MC group (member ldpid = DeepQ_0) that DFT_FWD
	 * replicates DLF frames to.  Without it, DFT_FWD -> reserved null group 0 ->
	 * blackhole(0x1f) drop.  Must run before the DFT_FWD loop points at the group. */
	cortina_ni_rx_mc_group_init(ni);

	/* ★ THE own-MAC CPU trap: an own-MAC frame is classified ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 177. */
	cortina_ni_rx_mymac_trap(ni);

	/* ★★ THE missing L3FE global init (0x30ac-0x30f8) - runs BEFORE the CLS rows,
	 * mirroring the vendor aal_l3fe_init order (l2lookup_init/glb-setters before the
	 * classifier rules).  This is what actually lets a frame INGRESS the L3FE stage
	 * (l3fe_rx) so the CLS trap below can even see it. */
	cortina_ni_rx_l3fe_glb_init(ni);

	/* ★★ build96: the L3FE AXI read-reorder channel - vendor runs aal_l3fe_axi_reo_init
	 * LAST in aal_l3fe_init (after l2lookup).  This is the L3FE's own DMA read-reorder,
	 * distinct from the main NI AXI-REO we already program - lets the L3FE fetch the
	 * frame from memory so it can ingest (l3fe_rx). */
	cortina_ni_rx_l3fe_axi_reo_init(ni);

	/* ★★★ build71: the L3-CLS special-packet trap - broadcast ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 178. */
	cortina_ni_rx_cls_init(ni);

	/* full FE path: byp OFF, pass OAM, drop unknown opcodes (stock policy).
	 * ★ Apply to EVERY GPHY LAN port, not just CA_NI_RX_PORT, so whichever port
	 * has the host cable runs the FE lookup (and hits the DLF trap below). */
	{
		unsigned int p;

		for (p = 0; p < CA_NI_GPHY_COUNT; p++)
			ni_rmw(ni, CA_NI_PORT_RX_CNTRL_CFG(p),
			       CA_NI_RX_CNTRL_BYP_EN | CA_NI_RX_CNTRL_BYP_DPID |
			       CA_NI_RX_CNTRL_BYP_COS | CA_NI_RX_CNTRL_UKOP_DROP_DIS,
			       CA_NI_RX_CNTRL_OAM_DROP_DIS);
	}

	/* L3FE demux golden routing map (FE output -> L3QM CPU-EPP) */
	writel(CA_NI_NIRX_L3FE_DEMUX0_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX0);
	writel(CA_NI_NIRX_L3FE_DEMUX1_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX1);
	writel(CA_NI_NIRX_L3FE_DEMUX2_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX2);
	writel(CA_NI_NIRX_L3FE_DEMUX3_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX3);
	writel(CA_NI_NIRX_L3FE_DEMUX4_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX4);
	writel(CA_NI_NIRX_L3FE_DEMUX5_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX5);

	/* ★★ The DEEP-QUEUE (deep_q=1) parallel per-ldpid demux table ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 87. */
	writel(CA_NI_NIRX_L3FE_DPQ_DEMUX_48_63_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ_DEMUX_48_63);
	writel(CA_NI_NIRX_L3FE_DPQ_DEMUX_32_47_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ_DEMUX_32_47);
	writel(CA_NI_NIRX_L3FE_DPQ_DEMUX_16_31_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ_DEMUX_16_31);
	writel(CA_NI_NIRX_L3FE_DPQ_DEMUX_0_15_VAL,  ni_base(ni) + CA_NI_NIRX_L3FE_DPQ_DEMUX_0_15);

	/* ★★ The REAL aal_ni NIRX-L3FE-demux table (0xa1d4-0xa1f0) - ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 88. */
	writel(CA_NI_NIRX_L3FE_DEMUX_NORM0_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM0);
	writel(CA_NI_NIRX_L3FE_DEMUX_NORM1_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM1);
	writel(CA_NI_NIRX_L3FE_DEMUX_NORM2_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM2);
	writel(CA_NI_NIRX_L3FE_DEMUX_NORM3_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM3);
	writel(CA_NI_NIRX_L3FE_DPQ0_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ0);
	writel(CA_NI_NIRX_L3FE_DPQ1_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ1);
	writel(CA_NI_NIRX_L3FE_DPQ2_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ2);
	writel(CA_NI_NIRX_L3FE_DPQ3_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ3);
	dev_info(ni->dev,
		 "l3fe-demux (REAL 0xa1d4-f0): norm3(0xa1e0)=0x%08x dpq0(0xa1e4)=0x%08x dpq3(0xa1f0)=0x%08x (want stock 0x64503C28/0x780C7864/0x00006050)\n",
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM3),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DPQ0),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DPQ3));

	/* ★★ Enable the internal (CPU/QM/L3QM-facing) port MAC ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 179. */
	cortina_ni_rx_enable_internal_ports(ni);

	/* ★ 0xa1c0 = 0x76543210 - the ONE NI_HV word our driver never wrote (all others
	 * 0xa180-0xa1bc match stock).  Prime L2TM-egress -> L3QM source-select suspect. */
	writel(CA_NI_NI_L3QMRX_PORT_ORDER_VAL,
	       ni_base(ni) + CA_NI_NI_L3QMRX_PORT_ORDER);

	/* second egress-enable layer: all NI egress ports (stock 0x610c) */
	writel(CA_NI_QM_L3TM_NI_PORT_ENA_ALL,
	       ni_base(ni) + CA_NI_QM_L3TM_NI_PORT_ENA);

	/* DLF trap: every FE lookup miss (BC/UUC/UL2MC/UL3MC) -> ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 89. */
	{
		unsigned int p;

		for (p = 0; p < CA_NI_GPHY_COUNT; p++)
			for (type = 0; type < CA_NI_PLE_TYPE_COUNT; type++) {
				ret = cortina_ni_rx_ple_dft_fwd(ni, p, type);
				if (ret) {
					dev_err(ni->dev,
						"PLE dft-fwd (lspid %u type %d) timed out\n",
						p, type);
					return ret;
				}
			}

		for (type = 0; type < CA_NI_PLE_TYPE_COUNT; type++) {
			ret = cortina_ni_rx_ple_dft_fwd(ni, CA_NI_LSPID_PON, type);
			if (ret)
				dev_err(ni->dev,
					"PLE dft-fwd (PON lspid %u type %d) failed (%d): downstream broadcast (an ARP request for our WAN address, a broadcast DHCP OFFER) will be DROPPED instead of trapped to the CPU\n",
					CA_NI_LSPID_PON, type, ret);
		}
		ret = 0;
	}

	/* ★ ORDER TEST: re-arm the L2TE->L3FE ready handshake ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 90. */
	writel(CA_NI_NI_NIRX_MISC_STOCK_VAL, ni_base(ni) + CA_NI_NI_NIRX_MISC_CFG);
	dev_info(ni->dev,
		 "l3fe-handoff re-arm (post-L3FE-config): nirx_misc(0xa1bc)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_NI_NIRX_MISC_CFG));

	return 0;
}

/* L3QM empty-buffer pool + delivery-chain init (stock ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 91. */
static void cortina_ni_rx_eq_commit(struct cortina_ni *ni)
{
	/* ★ Vendor-EXACT commit-latch (aal_l3qm_load_eq_config ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 92. */
	writel(CA_NI_QM_HDM_UNLOCK, ni_base(ni) + CA_NI_QM_HDM_WRITE_PROT);
	writel(CA_NI_QM_EQ_CFG_LOAD_ALL, ni_base(ni) + CA_NI_QM_EQ_CFG_LOAD);
	writel(0, ni_base(ni) + CA_NI_QM_EQ_CFG_LOAD);
	writel(0, ni_base(ni) + CA_NI_QM_HDM_WRITE_PROT);
}

/* Program ONE QM AXI-attribute table entry via the indirect ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 93. */
static void cortina_ni_rx_set_axi_attrib(struct cortina_ni *ni, u32 entry,
					 u32 data0)
{
	u32 acc;
	int i;

	writel(data0, ni_base(ni) + CA_NI_QM_AXI_ATTR_DATA0);
	writel(CA_NI_QM_AXI_ATTR_GO | CA_NI_QM_AXI_ATTR_RBW |
	       FIELD_PREP(CA_NI_QM_AXI_ATTR_ADDR, entry),
	       ni_base(ni) + CA_NI_QM_AXI_ATTR_ACCESS);
	for (i = 0; i < CA_NI_QM_AXI_ATTR_POLL_MAX; i++) {
		acc = readl(ni_base(ni) + CA_NI_QM_AXI_ATTR_ACCESS);
		if (!(acc & CA_NI_QM_AXI_ATTR_GO))
			return;
		udelay(1);
	}
	dev_warn(ni->dev, "AXI-attr entry %u commit timeout (acc=0x%08x)\n",
		 entry, acc);
}

/* ★★ DIAGNOSTIC: read ONE AXI-attr entry back via the ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 94. */
static int cortina_ni_rx_get_axi_attrib(struct cortina_ni *ni, u32 entry,
					u32 *out)
{
	int i;

	writel(CA_NI_QM_AXI_ATTR_GO | FIELD_PREP(CA_NI_QM_AXI_ATTR_ADDR, entry),
	       ni_base(ni) + CA_NI_QM_AXI_ATTR_ACCESS);
	for (i = 0; i < CA_NI_QM_AXI_ATTR_POLL_MAX; i++) {
		if (!(readl(ni_base(ni) + CA_NI_QM_AXI_ATTR_ACCESS) &
		      CA_NI_QM_AXI_ATTR_GO)) {
			*out = readl(ni_base(ni) + CA_NI_QM_AXI_ATTR_DATA0);
			return 0;
		}
		udelay(1);
	}
	return -ETIMEDOUT;
}

/* One readback line.  A timeout SAYS SO instead of printing whatever DATA0
 * happened to hold -- the point of the readback is to be believable. */
static void rx_show_axi_attrib(struct cortina_ni *ni, const char *what,
			       unsigned int n, u32 entry)
{
	u32 v;

	if (cortina_ni_rx_get_axi_attrib(ni, entry, &v))
		dev_info(ni->dev,
			 "axi-attr-readback: %s[%u] idx%u = NOT READ (GO never cleared)\n",
			 what, n, entry);
	else
		dev_info(ni->dev, "axi-attr-readback: %s[%u] idx%u = 0x%08x\n",
			 what, n, entry, v);
}

/* Initialise the QM AXI-attribute table BEFORE any frame can ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 95. */
static void cortina_ni_rx_axi_attrib_init(struct cortina_ni *ni)
{
	static const u32 ddr_eqs[] = {
		CA_NI_RX_EQ_ID, CA_NI_RX_EQ_ID2, CA_NI_RX_EQ8_ID,
	};
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ddr_eqs); i++)
		cortina_ni_rx_set_axi_attrib(ni,
			CA_NI_QM_AXI_ATTR_EQ_BASE + ddr_eqs[i],
			CA_NI_QM_AXI_ATTR_DDR_POOL);

	/* ★★ Stock writes the coherent CPU_EPP attr 0x12008060 here ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 96. */
	for (i = 0; i < CA_NI_QM_CPU_PORT_COUNT; i++)
		cortina_ni_rx_set_axi_attrib(ni,
			CA_NI_QM_AXI_ATTR_CPU_BASE + i,
			ring_noncoh ? CA_NI_QM_AXI_ATTR_DDR_POOL :
				      CA_NI_QM_AXI_ATTR_CPU_EPP);
	dev_info(ni->dev, "axi-attr: CPU-EPP entries = %s\n",
		 ring_noncoh ? "non-coherent DDR 0x04000010 (ring_noncoh=1)" :
			       "stock 0x12008060 (coherent)");

	/* ★★ DIAGNOSTIC: read back the 8 CPU-EPP entries + the 2 pool EQs to prove the
	 * writes latched, plus the EPP cmd/ctrl block (0x6a30-0x6a3c; ours
	 * 0x6a3c=0x04 cmd_mode bit2). */
	for (i = 0; i < CA_NI_QM_CPU_PORT_COUNT; i++)
		rx_show_axi_attrib(ni, "cpu_epp", i,
				   CA_NI_QM_AXI_ATTR_CPU_BASE + i);
	rx_show_axi_attrib(ni, "eq", CA_NI_RX_EQ_ID,
			   CA_NI_QM_AXI_ATTR_EQ_BASE + CA_NI_RX_EQ_ID);
	rx_show_axi_attrib(ni, "eq", CA_NI_RX_EQ_ID2,
			   CA_NI_QM_AXI_ATTR_EQ_BASE + CA_NI_RX_EQ_ID2);
	/* ⚠ 0x6a30 STAYS BARE ON PURPOSE. This header calls it ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 180. */
	dev_info(ni->dev, "epp-ctrl: 0x6a30=0x%08x 0x6a34=0x%08x 0x6a38=0x%08x 0x6a3c=0x%08x\n",
		 readl(ni_base(ni) + 0x6a30), readl(ni_base(ni) + CA_NI_QM_BURST_BUF_SEG_ID_MON),
		 readl(ni_base(ni) + CA_NI_QM_DEBUG_CFG), readl(ni_base(ni) + CA_NI_QM_EPP));
}

/* Program the CPU-RX DELIVERY CHAIN (stock ... -- dev/MEASURED-cortina-ni-rx.c.md sec 97. */
static void cortina_ni_rx_eq_cfg_pool(struct cortina_ni *ni, unsigned int eqid,
				      u32 cfg0, u32 bid_start, u32 total_buf,
				      u32 cfg2)
{
	writel(cfg0, ni_base(ni) + CA_NI_QM_CFG0_EQ(eqid));
	writel(FIELD_PREP(CA_NI_QM_CFG1_BID_START, bid_start) |
	       FIELD_PREP(CA_NI_QM_CFG1_TOTAL_BUF_NUM, total_buf),
	       ni_base(ni) + CA_NI_QM_CFG1_EQ(eqid));
	writel(cfg2, ni_base(ni) + CA_NI_QM_CFG2_EQ(eqid));
	/* AXI cache/snoop attrs, same for both CPU pools (tier-1 = 0x10) */
	writel(CA_NI_QM_CFG3_CPU_POOL_VAL, ni_base(ni) + CA_NI_QM_CFG3_EQ(eqid));
	writel(0, ni_base(ni) + CA_NI_QM_CFG4_EQ(eqid));
}

/* Hand the software-owned CPU pools their buffers ... -- dev/MEASURED-cortina-ni-rx.c.md sec 98. */
static int cortina_ni_rx_push_seed(struct cortina_ni *ni)
{
	/* nbufs MUST equal the total_buf each pool was configured with (the same
	 * constants cortina_ni_rx_eq_init passes to CFG1) - see the note at
	 * CA_NI_RX_PUSH_SEED_MIN. */
	static const struct {
		u32 eqid, base_off, bufsz, nbufs;
	} pools[] = {
		{ CA_NI_RX_EQ_ID,  0,
		  CA_NI_RX_CPU_POOL0_BUFSZ, CA_NI_RX_EQ_TOTAL_BUF },
		{ CA_NI_RX_EQ_ID2, CA_NI_RX_CPU_POOL0_BYTES,
		  CA_NI_RX_CPU_POOL1_BUFSZ, CA_NI_RX_EQ2_TOTAL_BUF },
	};
	unsigned int p, i, want = 0, done = 0;
	int ret = 0;

	for (p = 0; p < ARRAY_SIZE(pools); p++)
		want += pools[p].nbufs;

	for (p = 0; p < ARRAY_SIZE(pools) && !ret; p++) {
		for (i = 0; i < pools[p].nbufs; i++) {
			u32 pa = CA_NI_RX_CPU_POOL_PHYS + pools[p].base_off +
				 i * pools[p].bufsz;

			ret = cortina_ni_rx_push_buf(ni, pools[p].eqid, pa,
						     CA_NI_RX_PUSH_TIMEOUT_US);
			if (ret)
				break;
			done++;
		}
	}

	if (done < CA_NI_RX_PUSH_SEED_MIN) {
		dev_err(ni->dev,
			"RX: CPU pool seed FAILED - staged only %u of %u buffers (push stage never freed a slot).  RX would starve; boot with cortina_ni_rx.cpu_pool_push=0 to fall back to the hardware-managed pool.\n",
			done, want);
		return -EBUSY;
	}
	if (ret)
		dev_warn(ni->dev,
			 "RX: CPU pool seed SHORT - staged %u of %u buffers; the pools will report an inactive-bid shortfall\n",
			 done, want);
	else
		dev_info(ni->dev,
			 "RX: CPU pool seeded %u software-owned buffers (EQ%u=%u + EQ%u=%u, = CFG1.total_buf; expect inactive=0 ready=0)\n",
			 done, CA_NI_RX_EQ_ID, CA_NI_RX_EQ_TOTAL_BUF,
			 CA_NI_RX_EQ_ID2, CA_NI_RX_EQ2_TOTAL_BUF);
	return 0;
}

/* Log QM_PHY_PORT_STS with the handshake bits decoded (all should climb from
 * 0 toward the 0xa5ffffff default as the NI/TE/ES/AXI blocks come up). */
static void cortina_ni_rx_log_qm_sts(struct cortina_ni *ni, const char *stage)
{
	u32 s = readl(ni_base(ni) + CA_NI_QM_PHY_PORT_STS);

	dev_info(ni->dev,
		 "QM-hs[%s]: phy_sts=0x%08x nirx_port_rdy=0x%02lx te_es_ni_ok=0x%02lx nirx_qm_rdy=%u axi_wr=%u axi_rd=%u\n",
		 stage, s,
		 (unsigned long)FIELD_GET(CA_NI_QM_STS_NIRX_PORT_RDY, s),
		 (unsigned long)FIELD_GET(CA_NI_QM_STS_TE_ES_NI_OK, s),
		 !!(s & CA_NI_QM_STS_NIRX_QM_RDY),
		 !!(s & CA_NI_QM_STS_AXI_WR_RDY),
		 !!(s & CA_NI_QM_STS_AXI_RD_RDY));
}

/* Match STOCK-LINUX's LIVE working NI-RX->CPU routing ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 99. */
static void cortina_ni_rx_stock_routing(struct cortina_ni *ni)
{
	cortina_ni_rx_log_qm_sts(ni, "before");

	writel(CA_NI_QM_AXIM2_STOCK_VAL, ni_base(ni) + CA_NI_QM_AXIM2_CONFIG);
	writel(CA_NI_NI_DEMUX1_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_L3QMRX_DEMUX_CFG1);
	writel(CA_NI_NI_DEMUX0_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_L3QMRX_DEMUX_CFG0);
	/* ★★ THE l3fe_rx=0 FIX (live-stock 2026-07-13): the REAL per-ldpid
	 * L2FE-vs-L3FE ingress fork is NIRX_L3FE_DEMUX_CFG1/0 at 0xa1c4/0xa1c8, not the
	 * 0xa188/0xa18c above (rate-meter regs).  Stock 0x00CBDA98/0x7000DA98. */
	writel(CA_NI_NIRX_L3FE_DEMUX_CFG1_REAL_VAL,
	       ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_CFG1_REAL);
	writel(CA_NI_NIRX_L3FE_DEMUX_CFG0_REAL_VAL,
	       ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_CFG0_REAL);
	writel(CA_NI_NI_PORTORDER_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_PORTORDER_CFG);
	/* ★ INTERNAL_PORT_ID_CFG (0xa180) = 0x00A87F00: l3qmrx_demux_sel[7:0]=0 =
	 * present ALL ports' NI-RX to the L3QM = THE NI->QM LAN handoff.  We never
	 * wrote 0xa180 before (wrote 0x3e80 to 0xa1bc, which is NIRX_MISC). */
	writel(CA_NI_NI_INTERNAL_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG);
	/* NIRX_MISC_CFG (0xa1bc) = 0x3E80 (stock live) - the value our old code
	 * coincidentally wrote here thinking it was intern_pid; keep it. */
	writel(CA_NI_NI_NIRX_MISC_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_NIRX_MISC_CFG);
	writel(CA_NI_NI_AUTOSYNC_STOCK_VAL,
	       ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC);

	/* ★★ OR NI_HV_GLB_STATIC_CFG (0xa01c) bits[17:16] - the ONLY ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 181. */
	writel(readl(ni_base(ni) + CA_NI_NI_GLB_STATIC_CFG) | CA_NI_NI_GLB_STATIC_L3QM_EN,
	       ni_base(ni) + CA_NI_NI_GLB_STATIC_CFG);

	dev_info(ni->dev,
		 "stock-routing: intern_pid(a1bc)=0x%08x portorder(a1c0)=0x%08x autosync(a010)=0x%08x glb_static(a01c)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG),
		 readl(ni_base(ni) + CA_NI_NI_PORTORDER_CFG),
		 readl(ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC),
		 readl(ni_base(ni) + CA_NI_NI_GLB_STATIC_CFG));

	cortina_ni_rx_log_qm_sts(ni, "stock-routing");
}

/* ★ Bisect-from-working: replicate stock's FULL QM config ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 100. */
static const struct cortina_ni_reg_run cortina_ni_stock_qm_cfg[] = {
	/* ★ EQ8/EQ12 pool-enable INTENTIONALLY OMITTED (they HUNG the ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 101. */
	{ 0x656c, 62, 0x01010101 },	/* ..0x6660 */
	/* ★ THE DRAIN-DELIVERY block (0x6664-0x66cc) stock programs ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 102. */
	{ 0x6664, 6, 0x11111111 },	/* ..0x6678 */
	{ 0x667c, 1, 0x000c0114 },
	/* the eight per-FIFO delivery descriptors, profiles 0..7 */
	{ CA_NI_QM_CPU_EPP_FIFO_PROF(0), 4, 0xe0008001 },	/* ..0x66b0 */
	{ CA_NI_QM_CPU_EPP_FIFO_PROF(4), 1, 0xe00040f1 },
	{ CA_NI_QM_CPU_EPP_FIFO_PROF(5), 1, 0x20006801 },
	{ CA_NI_QM_CPU_EPP_FIFO_PROF(6), 1, 0x2000c801 },
	{ CA_NI_QM_CPU_EPP_FIFO_PROF(7), 1, 0xe0080001 },
	/* per-queue CPU-EPP-FIFO cfg: CA_NI_QM_CPU_EPP_FIFO_CFG(p, q), 64 words
	 * covering ports 0..7 x queues 0..7 (0x66cc..0x67c8) */
	{ CA_NI_QM_CPU_EPP_FIFO_CFG(0, 0), 16, 0x00000004 },	/* ..0x6708: ports 0-1, q0..7 */
	{ CA_NI_QM_CPU_EPP_FIFO_CFG(2, 0), 1, 0x00000006 },
	{ CA_NI_QM_CPU_EPP_FIFO_CFG(2, 1), 3, 0x00000005 },	/* ..0x6718 */
	{ CA_NI_QM_CPU_EPP_FIFO_CFG(2, 4), 1, 0x00000006 },
	{ CA_NI_QM_CPU_EPP_FIFO_CFG(2, 5), 3, 0x00000005 },	/* ..0x6728 */
	{ CA_NI_QM_CPU_EPP_FIFO_CFG(3, 0), 40, 0x00000007 },	/* ..0x67c8: ports 3-7, q0..7 */
	/* ★ 0x67cc CONFIRMED TOXIC + REQUIRED: per-queue CPU-EPP-FIFO ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 182. */
	{ 0x69b4, 1, 0x80080000 },
	{ 0x69bc, 1, 0x06061616 },
	/* ★ 0x69bc corrected 0x06006666 -> stock 0x06061616 (our own ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 183. */
	{ 0x69f8, 1, 0x000000ff },
	{ CA_NI_QM_EPP_CPU_EGR_EN, 1, 0x0000ff00 },	/* 0x6a00 */
	{ CA_NI_QM_EPP64_INT_EN0, 1, 0x0000ffff },	/* 0x6110 */
	{ CA_NI_QM_EPP64_INT_EN2, 1, 0x00000100 },	/* 0x6118 */
	/* 0x611c: the header names it CA_NI_QM_INT_SRC, eqm_readback prints it as
	 * "refill_en" - two in-tree claims, unresolved, so the number stays */
	{ 0x611c, 1, 0x10000000 },
	{ 0x6120, 1, 0xe6d54f85 },
	/* ★ EXCLUDED (hang triggers / board-specific DMA state): 0x67cc=0x4000000F
	 * (bit30 per-queue indirect COMMIT) + the 0x69c4-0x69e0 block (0x0863A000 etc.
	 * = STOCK's CPU-EPP ring/DMA addresses; ours live at 0x0bc48000 - replicating
	 * stock's would point the QM at wrong memory = hang). */
};

/* ★★ The EQM buffer-availability + RMU0-admit ledger.  The cpu_eq=1 fix should
 * make pa_req (EQ13 0x63bc / EQ14 0x63c0) climb >0 so RMU0 admits: 0x6940
 * (NO_BUF_DROP) STOPS climbing, 0x6900 (RMU0_RX) climbs, epp_wptr(0x7000)
 * advances.  (0x69xx read-back dropped: RE-confirmed READ-ONLY RMU0 status.) */
static void cortina_ni_rx_eqm_readback(struct cortina_ni *ni, const char *label)
{
	dev_info(ni->dev,
		 "eqm-readback(%s): pa_req eq13(0x63bc)=0x%08x eq14(0x63c0)=0x%08x | eq_prof5(0x613c)=0x%08x | fifo_prof4(0x66b4)=0x%08x (want 0xE00040F1) | int_en0(0x6110)=0x%08x en1(0x6114)=0x%08x refill_en(0x611c)=0x%08x REAL_int_src(0x6120)=0x%08x [b22=eqm_cfg_err b21=buf_size b20=cpuepp_fifo] | no_buf(0x6940)=%u rmu_rx(0x6900)=%u tx_cntr(0x690c)=%u epp_wptr(0x7000)=0x%06x\n",
		 label,
		 /* 0x63bc/0x63c0: NOT in the vendor NAME->ADDRESS table, so there
		  * is nothing to name them FROM -- see
		  * FINDING-the-vendor-table-does-not-cover-the-serdes-windows.md
		  * for the same shape in another window.  Bare is honest here. */
		 readl(ni_base(ni) + 0x63bc),
		 readl(ni_base(ni) + 0x63c0),
		 readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE_SEL)),
		 readl(ni_base(ni) + CA_NI_QM_CPU_EPP_FIFO_PROF(CA_NI_RX_PROFILE_ID)),
		 readl(ni_base(ni) + CA_NI_QM_EPP64_INT_EN0),
		 readl(ni_base(ni) + CA_NI_QM_EPP64_INT_EN1),
		 readl(ni_base(ni) + CA_NI_QM_INT_SRC),
		 readl(ni_base(ni) + CA_NI_QM_INT_SRCE),
		 readl(ni_base(ni) + CA_NI_QM_RMU_NO_BUF_DROP),
		 readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
		 readl(ni_base(ni) + CA_NI_QM_TX_CNTR),
		 cortina_ni_rx_wptr(ni));
}

static void __maybe_unused cortina_ni_rx_match_stock_qm(struct cortina_ni *ni)
{
	unsigned int wrote;

	wrote = cortina_ni_rx_write_runs(ni, cortina_ni_stock_qm_cfg,
					 ARRAY_SIZE(cortina_ni_stock_qm_cfg));
	dev_info(ni->dev, "match-stock-qm: wrote %u QM cfg regs (EQ8/12 pools, DWRR, per-queue, misc)\n",
		 wrote);
}

static int cortina_ni_rx_eq_init(struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx = ni->rx;
	u32 cfg0_p0, cfg0_p1, cfg2_p0, cfg2_p1;
	u32 sts;
	int i, ret;

	/* (0) NI-RX->QM routing = U-Boot's live working values (bisect-from-
	 * working): the real fix for "frames never reach the QM". */
	cortina_ni_rx_stock_routing(ni);

	/* (1) wait for the QM block's own init to finish before touching it (stock
	 * aal_l3qm_check_init_done); bounded + non-fatal.  We no longer GATE on
	 * qm_init_done (a phantom 0 even in the 0xa5ffffff default) - qm_up is kept as
	 * a logged diagnostic only. */
	ret = readl_poll_timeout(ni_base(ni) + CA_NI_QM_PHY_PORT_STS, sts,
				 sts & CA_NI_QM_INIT_DONE, 10,
				 CA_NI_QM_INIT_DONE_TIMEOUT_US);
	rx->qm_up = !ret;
	if (ret)
		dev_warn(ni->dev, "RX: QM init-done not seen (sts=0x%x) - is the TQM reset firing?\n",
			 sts);
	else
		dev_info(ni->dev, "RX: QM init-done OK (sts=0x%x)\n", sts);

	/* (2) master L3QM RX off while (re)programming (stock order) */
	ni_rmw(ni, CA_NI_QM_RMU0_CTRL, CA_NI_QM_RMU0_RX_EN, 0);

	/* spy: report every EQ already holding a bid range (overlap with ours
	 * would corrupt the pool accounting) */
	for (i = 0; i < CA_NI_QM_EQ_COUNT; i++) {
		u32 c1 = readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(i));

		if (FIELD_GET(CA_NI_QM_CFG1_TOTAL_BUF_NUM, c1))
			dev_info(ni->dev,
				 "RX: EQ%d pre-set: bid_start=%lu num=%lu\n", i,
				 FIELD_GET(CA_NI_QM_CFG1_BID_START, c1),
				 FIELD_GET(CA_NI_QM_CFG1_TOTAL_BUF_NUM, c1));
	}

	/* (3) CPU empty-buffer pools EQ5(pool0)/EQ6(pool1). ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 103. */
	if (cpu_pool_push) {
		/* ★ REVERTED 2026-07-28: keep the REAL base here too. Stock ...
		 * dev/MEASURED-cortina-ni-rx.c.md sec 104. */
		cfg0_p0 = (CA_NI_RX_CPU_POOL_PHYS &
			   CA_NI_QM_CFG0_PHY_ADDR_START) | CA_NI_QM_CFG0_EQ_EN;
		cfg0_p1 = ((CA_NI_RX_CPU_POOL_PHYS + CA_NI_RX_CPU_POOL0_BYTES) &
			   CA_NI_QM_CFG0_PHY_ADDR_START) | CA_NI_QM_CFG0_EQ_EN;
		cfg2_p0 = CA_NI_QM_EQ13_CFG2 | CA_NI_QM_CFG2_CPU_EQ;
		cfg2_p1 = CA_NI_QM_EQ14_CFG2 | CA_NI_QM_CFG2_CPU_EQ;
	} else {
		cfg0_p0 = (CA_NI_RX_CPU_POOL_PHYS &
			   CA_NI_QM_CFG0_PHY_ADDR_START) | CA_NI_QM_CFG0_EQ_EN;
		cfg0_p1 = ((CA_NI_RX_CPU_POOL_PHYS + CA_NI_RX_CPU_POOL0_BYTES) &
			   CA_NI_QM_CFG0_PHY_ADDR_START) | CA_NI_QM_CFG0_EQ_EN;
		cfg2_p0 = CA_NI_QM_EQ13_CFG2;
		cfg2_p1 = CA_NI_QM_EQ14_CFG2;
	}
	cortina_ni_rx_eq_cfg_pool(ni, CA_NI_RX_EQ_ID, cfg0_p0,
				  CA_NI_RX_EQ_BID_START, CA_NI_RX_EQ_TOTAL_BUF,
				  cfg2_p0);
	cortina_ni_rx_eq_cfg_pool(ni, CA_NI_RX_EQ_ID2, cfg0_p1,
				  CA_NI_RX_EQ2_BID_START, CA_NI_RX_EQ2_TOTAL_BUF,
				  cfg2_p1);
	dev_info(ni->dev,
		 "RX: CPU pools EQ%u/EQ%u %s (cfg0=0x%08x/0x%08x cfg2=0x%08x/0x%08x)\n",
		 CA_NI_RX_EQ_ID, CA_NI_RX_EQ_ID2,
		 cpu_pool_push ? "SOFTWARE-OWNED (cpu_eq=1, push recycle)" :
				 "hardware-managed (cpu_eq=0, self-populating)",
		 cfg0_p0, cfg0_p1, cfg2_p0, cfg2_p1);

	/* ★★★ (3a) The DEEP-QUEUE pool, EQ12, hardware-managed - only ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 105. */
	if (cpu_pool_push) {
		cortina_ni_rx_eq_cfg_pool(ni, CA_NI_RX_EQ12_ID,
					  (CA_NI_RX_DQ_POOL_PHYS &
					   CA_NI_QM_CFG0_PHY_ADDR_START) |
					  CA_NI_QM_CFG0_EQ_EN,
					  CA_NI_RX_EQ12_BID_START,
					  CA_NI_RX_EQ12_TOTAL_BUF,
					  CA_NI_RX_DQ_CFG2);
			/* BOTH halves of the profile point at EQ12, deliberately: this
			 * dev/MEASURED-cortina-ni-rx.c.md sec 106. */
		writel(CA_NI_RX_DQ_PROFILE_VAL,
		       ni_base(ni) +
		       CA_NI_QM_EQ_PROFILE(CA_NI_RX_DQ_PROFILE_SEL));
		dev_info(ni->dev,
			 "RX: deep-queue pool EQ%u hardware-managed @0x%08x (%u bufs, bid 0x%x, cfg2=0x%08x) -> EQ_PROFILE(%u)=0x%02x\n",
			 CA_NI_RX_EQ12_ID, (u32)CA_NI_RX_DQ_POOL_PHYS,
			 CA_NI_RX_EQ12_TOTAL_BUF,
			 (u32)CA_NI_RX_EQ12_BID_START,
			 (u32)CA_NI_RX_DQ_CFG2, CA_NI_RX_DQ_PROFILE_SEL,
			 (u32)CA_NI_RX_DQ_PROFILE_VAL);
	}

	/* (3b) Steer the CPU-bound frame to our pool via ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 107. */
	writel(CA_NI_RX_EQ_PROFILE_VAL,
	       ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE));
	/* ★ Program ALL 8 3-bit-reachable profiles (0..7) = ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 108. */
	for (i = 0; i < 8; i++)
		writel(CA_NI_RX_EQ_PROFILE_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_QM_EQ_PROFILE, i,
						CA_NI_QM_EQ_PROFILE_COUNT));
	for (i = 0; i < CA_NI_QM_EQ_PROFILE_GLOBAL_COUNT; i++)
		writel(CA_NI_RX_CPU_PROFILE_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_QM_EQ_PROFILE_GLOBAL, i,
						CA_NI_QM_EQ_PROFILE_GLOBAL_COUNT));
	/* build77: cpu_port 0 (CPU_0) -> profile_sel=2 -> EQ_PROFILE[2]={EQ5,EQ6} (the
	 * stock CPU pools, now configured+seeded).  Our 0..7 loop above set EQ_PROFILE[2]
	 * to {EQ5,EQ6}; point destport0 at profile 2 (stock value; was 0xf8=profile 8). */
	writel((CA_NI_NI_DESTPORT0_STOCK_VAL & ~CA_NI_QM_DEST_PORT_PROF_SEL) |
	       FIELD_PREP(CA_NI_QM_DEST_PORT_PROF_SEL, 2),
	       ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(0));
	/* The deep-queue dest-ports (8..15, incl. the CPU slot 15). ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 109. */
	for (i = CA_NI_RX_DEEPQ_DEST_PORT_LO; i <= CA_NI_RX_DEEPQ_DEST_PORT_HI; i++)
		writel(cpu_pool_push ?
		       FIELD_PREP(CA_NI_QM_DEST_PORT_PROF_SEL,
				  CA_NI_RX_DQ_PROFILE_SEL) :
		       CA_NI_RX_CPU_PROFILE_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_QM_DEST_PORT_EQ_CFG, i,
						CA_NI_QM_DEST_PORT_ENTRIES));
	/* ★ 2026-07-23: stock configures DEST_PORT_EQ_CFG for the ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 110. */
	for (i = CA_NI_RX_DEEPQ_DEST_PORT_HI + 1; i <= CA_NI_QM_DEST_PORT_LAST; i++)
		writel(cpu_pool_push ?
		       FIELD_PREP(CA_NI_QM_DEST_PORT_PROF_SEL,
				  CA_NI_RX_DQ_PROFILE_SEL) :
		       CA_NI_RX_CPU_PROFILE_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_QM_DEST_PORT_EQ_CFG, i,
						CA_NI_QM_DEST_PORT_ENTRIES));

	/* 0x6ab0 = 0x300 (stock-matching; this is NOT the real ES_CTRL2 - kept as a
	 * harmless match). */
	writel(CA_NI_QM_ES_CTRL2_STOCK_VAL, ni_base(ni) + CA_NI_QM_ES_CTRL2);

	/* ★ REMOVED the 0x6a30 (ni_qm_hol bit1) write: a full ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 111. */
	ni_rmw(ni, CA_NI_QM_DEST_PORT_PKT_BUF_CFG(CA_NI_RX_CPU_PORT),
	       CA_NI_QM_PKT_BUF_HEAD_FIRST | CA_NI_QM_PKT_BUF_TAIL_FIRST |
	       CA_NI_QM_PKT_BUF_HEAD_REST | CA_NI_QM_PKT_BUF_TAIL_REST,
	       FIELD_PREP(CA_NI_QM_PKT_BUF_HEAD_FIRST, CA_NI_QM_PKT_BUF_HEAD_UNITS) |
	       FIELD_PREP(CA_NI_QM_PKT_BUF_TAIL_FIRST, CA_NI_QM_PKT_BUF_TAIL_UNITS) |
	       FIELD_PREP(CA_NI_QM_PKT_BUF_HEAD_REST, CA_NI_QM_PKT_BUF_HEAD_UNITS) |
	       FIELD_PREP(CA_NI_QM_PKT_BUF_TAIL_REST, CA_NI_QM_PKT_BUF_TAIL_UNITS));

	/* ★ Initialise the QM AXI-attribute table (0x67cc indirect) for our EQ pools
	 * + CPU-EPP ports.  This is the piece the raw 0x67cc=0x4000000F write botched:
	 * unprogrammed entries stall the QM's buffer-DMA so admission (qm_rx) never
	 * advances.  Uses the bounded DATA0->ACCESS(GO)->poll protocol (no hang). */
	cortina_ni_rx_axi_attrib_init(ni);

	/* ★ config block RE-ENABLED with the suspect HW-triggers (0x67cc + 0x69xx)
	 * excluded from the table (see cortina_ni_stock_qm_cfg) - bisecting the hang.
	 * DWRR weights + per-queue profile-sel only. */
	cortina_ni_rx_match_stock_qm(ni);

	/* (4) COMMIT: latch both bid ranges into the empty-buffer manager
	 * (stock aal_l3qm_load_eq_config).  Until this fires the pushed PAs
	 * never leave the shallow push stage and EQM_PA_REQ stays 0. */
	cortina_ni_rx_eq_commit(ni);
	dev_info(ni->dev,
		 "RX: committed EQ%d(%u)+EQ%d(%u), bid 0x%x/0x%x\n",
		 CA_NI_RX_EQ_ID, CA_NI_RX_EQ_TOTAL_BUF,
		 CA_NI_RX_EQ_ID2, CA_NI_RX_EQ2_TOTAL_BUF,
		 CA_NI_RX_EQ_BID_START, CA_NI_RX_EQ2_BID_START);

	/* (5) NO l3qmrx_to_lan / ni_qm_hol handoff here: the WORKING ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 112. */
	for (i = 0; i < CA_NI_QM_VOQ_EN_COUNT; i++)
		ni_rmw(ni, CA_NI_QM_VOQ_EN(i), 0, CA_NI_QM_VOQ_EN_ALL);
	dev_info(ni->dev, "qm-voq: 0x6424=0x%08x 0x6428=0x%08x (want voq_en[7:0]=0xff)\n",
		 readl(ni_base(ni) + CA_NI_QM_VOQ_EN(0)),
		 readl(ni_base(ni) + CA_NI_QM_VOQ_EN(1)));

	return 0;
}

/* Enable the L3QM egress scheduler master (stock ... -- dev/MEASURED-cortina-ni-rx.c.md sec 113. */
static void cortina_ni_rx_es_enable(struct cortina_ni *ni)
{
	/* match stock 0x8462FFFF: set tx_en/ni_en/inccfg, CLEAR the stray
	 * bit25 (ours came up 0x8662...); cpu_en is armed by es_cpu at open */
	ni_rmw(ni, CA_NI_QM_ES_CTRL,
	       CA_NI_QM_ES_CPU_EN | CA_NI_QM_ES_NI_EN |
	       CA_NI_QM_ES_INCCFG_PKT | CA_NI_QM_ES_INCCFG_ERR |
	       CA_NI_QM_ES_RSVD25,
	       CA_NI_QM_ES_TX_EN |
	       FIELD_PREP(CA_NI_QM_ES_NI_EN, 0xff) |
	       FIELD_PREP(CA_NI_QM_ES_INCCFG_PKT, CA_NI_QM_ES_INCCFG_PKT_VAL) |
	       FIELD_PREP(CA_NI_QM_ES_INCCFG_ERR, CA_NI_QM_ES_INCCFG_ERR_VAL));
	/* (A second copy of this write used to go to 0x7108 as a supposed "real"
	 * ES_CTRL.  0x7108 is EPP64_RDPTR(cpu_port 0, voq 2) - a ring read pointer - so
	 * that wrote a control word into hardware ring state, harmless only because
	 * cortina_ni_rx_poll_voq stores the rdptr unconditionally.  Removed. */
}

/* arm/disarm the CPU-port drain (stock aal_l3qm_enable_tx_cpu ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 184. */
static void cortina_ni_rx_es_cpu(struct cortina_ni *ni, bool enable)
{
	u32 mask = FIELD_PREP(CA_NI_QM_ES_CPU_EN, CA_NI_QM_ES_CPU_EN_ALL);

	if (enable)
		ni_rmw(ni, CA_NI_QM_ES_CTRL, CA_NI_QM_ES_RSVD25, mask);
	else
		ni_rmw(ni, CA_NI_QM_ES_CTRL, mask, 0);
}

/* FBM pools the RMU allocates CPU-RX buffers from. RE: the ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 114. */
static const struct { u8 id; u32 exstack; u32 buf_base; } cortina_ni_fbm_pools[] = {
	{ 0, 0x0A000000u, 0x09404000u },	/* cpu_pool0 (non-deep) - our EQ14 2048B region */
	{ 7, 0x0A010000u, 0x0A100000u },	/* deep-queue pool (voqid=8, stock fbm_pool_id 7) */
};

/* ★★ FBM (Free Buffer Manager) init - the HW buffer-allocator ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 115. */
static void cortina_ni_rx_fbm_init(struct cortina_ni *ni)
{
	void __iomem *fbm_glb    = ni->win[CA_NI_WIN_FBM_GLB];
	void __iomem *axi    = ni->win[CA_NI_WIN_FBM_AXI];
	void __iomem *pool   = ni->win[CA_NI_WIN_FBM_POOL];
	void __iomem *ni_glb = ni->win[CA_NI_WIN_GLB];
	unsigned int k;

	if (!fbm_glb || !axi || !pool) {
		dev_warn(ni->dev,
			 "fbm: window(s) unmapped (fbm_glb=%d axi=%d pool=%d) - RMU cannot alloc\n",
			 !!fbm_glb, !!axi, !!pool);
		return;
	}

	/* (1) aal_fbm_reset: soft-reset the FBM via NI GLB-ctrl +0xa0 bit17, pulse 1->0. */
	if (ni_glb) {
		u32 v = readl(ni_glb + CA_NI_GLB_FBM_RESET);

		writel(v | CA_NI_GLB_FBM_RESET_BIT, ni_glb + CA_NI_GLB_FBM_RESET);
		cortina_ni_rx_settle();
		writel(v & ~CA_NI_GLB_FBM_RESET_BIT, ni_glb + CA_NI_GLB_FBM_RESET);
		cortina_ni_rx_settle();
	}

	/* (2) aal_fbm_init GLB config: +0x04 mode, +0x70 ECC, +0x00 low byte 0xFF =
	 * enable pools 0-7 (★ THE pool enable is this GLB bit, NOT POOL+0x30). */
	writel(0x00060100, fbm_glb + CA_NI_QM_FBM_GLB_MODE);
	writel(0xE0C04025, fbm_glb + CA_NI_QM_FBM_GLB_ECC);
	writel(0x010109FF, fbm_glb + CA_NI_QM_FBM_GLB_POOL_EN);

	writel(0x00000200, axi + 0x00);

	/* (3) aal_fbm_pool_init per pool (id*0x80): geometry + the EXSTACK pointer-spill
	 * region.  +0x04 = exstack_phys>>12 in [31:4]; +0x08 = spill depth; +0x0c =
	 * ((count/64)-1)<<6.  NO +0x30 (debug DMA), NO +0x40 here. */
	for (k = 0; k < ARRAY_SIZE(cortina_ni_fbm_pools); k++) {
		void __iomem *p = pool + CA_NI_QM_FBM_POOL(cortina_ni_fbm_pools[k].id);
		u32 exstack = (cortina_ni_fbm_pools[k].exstack >> 12) << 4;

		writel(0xC0400300, p + CA_NI_QM_FBM_POOL_CFG0);
		writel(exstack, p + CA_NI_QM_FBM_POOL_EXSTACK);
		writel(CA_NI_RX_FBM_EXSTACK_DEPTH, p + CA_NI_QM_FBM_POOL_DEPTH);
		writel(((CA_NI_RX_FBM_POOL0_COUNT / 64) - 1) << 6, p + CA_NI_QM_FBM_POOL_COUNT);

		dev_info(ni->dev,
			 "fbm cfg pool%u: cfg0=0x%08x exstack(0x04)=0x%08x depth=0x%08x cnt=0x%08x\n",
			 cortina_ni_fbm_pools[k].id,
			 readl(p + CA_NI_QM_FBM_POOL_CFG0), readl(p + CA_NI_QM_FBM_POOL_EXSTACK),
			 readl(p + CA_NI_QM_FBM_POOL_DEPTH), readl(p + CA_NI_QM_FBM_POOL_COUNT));
	}
	dev_info(ni->dev, "fbm cfg: glb0=0x%08x (%zu pools configured)\n",
		 readl(fbm_glb + CA_NI_QM_FBM_GLB_POOL_EN), ARRAY_SIZE(cortina_ni_fbm_pools));
}

/* ★★ Fill the FBM pool free-list via the FBM_CPU GATED ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 116. */
static void cortina_ni_rx_fbm_fill(struct cortina_ni *ni)
{
	void __iomem *pool = ni->win[CA_NI_WIN_FBM_POOL];
	void __iomem *cpu  = ni->win[CA_NI_WIN_FBM_CPU];
	unsigned int k, i, s;

	if (!pool || !cpu) {
		dev_warn(ni->dev, "fbm fill: pool/cpu window unmapped - cannot fill\n");
		return;
	}

	for (k = 0; k < ARRAY_SIZE(cortina_ni_fbm_pools); k++) {
		u8 id = cortina_ni_fbm_pools[k].id;
		void __iomem *db = cpu + CA_NI_QM_FBM_CPU_DOORBELL(id);
		void __iomem *p  = pool + CA_NI_QM_FBM_POOL(id);
		u32 base = cortina_ni_fbm_pools[k].buf_base;

		for (i = 0; i < CA_NI_RX_FBM_POOL0_COUNT; i++) {
			u32 buf = base + i * CA_NI_RX_FBM_POOL_BUFSZ;

			/* gate 1: outstanding < depth (else the push is rejected -1) */
			for (s = 0; s < CA_NI_FBM_GATE_TRIES; s++) {
				if (readl(p + CA_NI_QM_FBM_POOL_OUTSTND) <
				    CA_NI_RX_FBM_EXSTACK_DEPTH)
					break;
				cpu_relax();
			}
			/* gate 2: FBM_CPU cmd not BUSY (bit31 clear) before issuing.
			 * cpu_relax, not udelay: this runs inside the fill loop
			 * and the command clears in a handful of reads. */
			ca_go_spin(db + CA_NI_QM_FBM_CPU_CMD, CA_NI_FBM_GATE_TRIES,
				   ca_pause_relax);
			writel(0, db + CA_NI_QM_FBM_CPU_ADDR_HI);
			writel(buf, db + CA_NI_QM_FBM_CPU_ADDR_LO);
			writel(CA_NI_QM_FBM_CPU_CMD_GO | CA_NI_QM_FBM_CPU_CMD_PUSH,
			       db + CA_NI_QM_FBM_CPU_CMD);	/* GO | op=push (pool = offset) */
		}

		dev_info(ni->dev,
			 "fbm fill pool%u: pushed %u bufs @0x%08x+; outstanding(0x2c)=0x%08x\n",
			 id, CA_NI_RX_FBM_POOL0_COUNT, base,
			 readl(p + CA_NI_QM_FBM_POOL_OUTSTND));
	}
}

/* ★★ Program the RMU AXI read/write REORDER engine (stock ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 117. */
static const struct { u16 off; u32 val; } cortina_ni_axi_reo_cfg[] = {
	/* READ channel: AXI ID 0x0F -> 0x0C, one remap region at 0x10000000 */
	{ CA_NI_AXI_REO_RD_ORIG_ID,		0x0000000F },
	{ CA_NI_AXI_REO_RD_NEW_ID,		0x8000000C },
	{ CA_NI_AXI_REO_RD_TOP_ADDR0,		0x10000000 },
	{ CA_NI_AXI_REO_RD_TOP_ADDR_MASK0,	0x10000000 },
	{ CA_NI_AXI_REO_RD_NEW_ID0,		0x8000000D },
	{ CA_NI_AXI_REO_RD_TOP_ADDR_MASK1,	0xFFFFFFFF },
	{ CA_NI_AXI_REO_RD_TOP_ADDR_MASK2,	0xFFFFFFFF },
	/* WRITE channel: same shape, AXI ID 0x04 instead of 0x0F */
	{ CA_NI_AXI_REO_WR_ORIG_ID,		0x00000004 },
	{ CA_NI_AXI_REO_WR_NEW_ID,		0x8000000C },
	{ CA_NI_AXI_REO_WR_TOP_ADDR0,		0x10000000 },
	{ CA_NI_AXI_REO_WR_TOP_ADDR_MASK0,	0x10000000 },
	{ CA_NI_AXI_REO_WR_NEW_ID0,		0x8000000D },
	{ CA_NI_AXI_REO_WR_TOP_ADDR_MASK1,	0xFFFFFFFF },
	{ CA_NI_AXI_REO_WR_TOP_ADDR_MASK2,	0xFFFFFFFF },
	/* WRITE2 = the L3FE read channel, AXI ID 2 -> 8. Offsets AND values from the
	 * CA_NI_L3FE_AXI_REO_* pairs, so nothing here is a second spelling. */
	{ CA_NI_L3FE_AXI_REO_ORIG_ID,		CA_NI_L3FE_AXI_REO_ORIG_ID_VAL },
	{ CA_NI_L3FE_AXI_REO_NEW_ID,		CA_NI_L3FE_AXI_REO_NEW_ID_VAL },
	{ CA_NI_L3FE_AXI_REO_TOP_ADDR,		CA_NI_L3FE_AXI_REO_TOP_ADDR_VAL },
	{ CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK,	CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK_VAL },
	{ CA_NI_L3FE_AXI_REO_NEW_ID0,		CA_NI_L3FE_AXI_REO_NEW_ID0_VAL },
	{ CA_NI_L3FE_AXI_REO_RD18,		CA_NI_L3FE_AXI_REO_RD18_VAL },
	{ CA_NI_L3FE_AXI_REO_RD24,		CA_NI_L3FE_AXI_REO_RD24_VAL },
};

static void cortina_ni_rx_axi_reo_init(struct cortina_ni *ni)
{
	void __iomem *reo = ni->win[CA_NI_WIN_AXI_REO];
	unsigned int i;

	if (!reo) {
		dev_warn(ni->dev,
			 "RX: AXI-reorder window (idx %d) not mapped - RMU DMA stalls\n",
			 CA_NI_WIN_AXI_REO);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(cortina_ni_axi_reo_cfg); i++)
		writel(cortina_ni_axi_reo_cfg[i].val,
		       reo + cortina_ni_axi_reo_cfg[i].off);

	dev_info(ni->dev,
		 "RX: AXI-reorder init (%zu regs): rd[0x00/0x0c/0x10]=0x%08x/0x%08x/0x%08x wr[0x400]=0x%08x wr2[0x480]=0x%08x\n",
		 ARRAY_SIZE(cortina_ni_axi_reo_cfg),
		 readl(reo + CA_NI_AXI_REO_RD_ORIG_ID),
		 readl(reo + CA_NI_AXI_REO_RD_TOP_ADDR_MASK0),
		 readl(reo + CA_NI_AXI_REO_RD_NEW_ID0),
		 readl(reo + CA_NI_AXI_REO_WR_ORIG_ID),
		 readl(reo + CA_NI_L3FE_AXI_REO_ORIG_ID));
}

/* ------------------------------------------------------------------ */
/* L3QM CPU-EPP ring init (stock aal_l3qm_init_cpu_epp, port0/voq0)    */
/* ------------------------------------------------------------------ */

static void cortina_ni_rx_epp_init(struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx = ni->rx;
	u32 wptr;
	unsigned int i;

	/* ★★ Set the STOCK CPU-EPP interrupt-enables now ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 185. */
	writel(CA_NI_QM_EPP64_INT_EN0_STOCK, ni_base(ni) + CA_NI_QM_EPP64_INT_EN0);
	writel(0, ni_base(ni) + CA_NI_QM_EPP64_INT_EN1);
	writel(CA_NI_QM_EPP64_INT_EN2_STOCK, ni_base(ni) + CA_NI_QM_EPP64_INT_EN2);

	/* ★★ The 0x6a3c bit2 cmd_mode/GO set is MOVED to AFTER the ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 118. */
	ni_rmw(ni, CA_NI_QM_CPU_EPP_CFG(CA_NI_RX_CPU_PORT),
	       CA_NI_QM_EPP_MAP_MODE, 0);

	/* no descriptor coalescing timer */
	writel(0, ni_base(ni) + CA_NI_QM_CPU_EPP_CT_CFG);

	/* ★★ Do NOT re-write CPU_EPP_FIFO_PROF(4)=0x66b4 here. ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 119. */
	WARN_ON_ONCE(upper_32_bits(rx->ring_dma));
	for (i = 0; i < CA_NI_RX_VOQ_COUNT; i++) {
		u32 vphys = lower_32_bits(rx->ring_dma) + i * CA_NI_RX_RING_BYTES;

		ni_rmw(ni, CA_NI_QM_CPU_EPP_FIFO_CFG(CA_NI_RX_CPU_PORT, i),
		       CA_NI_QM_EPP_PROFILE_SEL,
		       FIELD_PREP(CA_NI_QM_EPP_PROFILE_SEL, CA_NI_RX_PROFILE_ID));

		writel(vphys, ni_base(ni) +
		       CA_NI_QM_EPP64_PADDR_START(CA_NI_RX_CPU_PORT, i));
		/* ★★★ build80: the SECOND per-voq ring buffer (PADDR_HI = PADDR + 0x2000).
		 * Stock sets it; ours was 0 -> the writeback engine wrote ZERO descriptors. */
		writel(vphys + CA_NI_RX_RING_HI_OFFSET, ni_base(ni) +
		       CA_NI_QM_EPP64_PADDR_HI(i));

		/* adopt the HW write pointer (0 after reset) as this voq's start */
		wptr = cortina_ni_rx_wptr_voq(ni, i);
		if (wptr)
			dev_warn(ni->dev, "RX voq%u wptr not idle at init (0x%x)\n",
				 i, wptr);
		rx->rptr[i] = wptr;
		writel(wptr, ni_base(ni) +
		       CA_NI_QM_EPP64_RDPTR(CA_NI_RX_CPU_PORT, i));
	}

	/* ★★ THE FIX: set 0x6a3c bit2 (cmd_mode/GO = the EPP ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 186. */
	ni_rmw(ni, CA_NI_QM_EPP, 0, CA_NI_QM_EPP_CMD_MODE_64);
	dev_info(ni->dev, "epp-init: cmd_mode/GO(0x6a3c)=0x%08x set LAST (after per-voq paddr)\n",
		 readl(ni_base(ni) + CA_NI_QM_EPP));

	/* ★★ THE CPU-egress ES enable (enable_tx_cpu equivalent). ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 120. */
	writel(CA_NI_QM_EPP_EGR_EN_ALL, ni_base(ni) + CA_NI_QM_EPP_TX_EGR_EN);
	writel(CA_NI_QM_EPP_EGR_EN_ALL, ni_base(ni) + CA_NI_QM_EPP_CPU_EGR_EN);
	dev_info(ni->dev, "epp-egr: tx(0x6a20)=0x%08x cpu(0x6a00)=0x%08x (want both 0x0000ff00)\n",
		 readl(ni_base(ni) + CA_NI_QM_EPP_TX_EGR_EN),
		 readl(ni_base(ni) + CA_NI_QM_EPP_CPU_EGR_EN));
}

/* GPHY fault poll + port reinit (stock ... -- dev/MEASURED-cortina-ni-rx.c.md sec 121. */

static inline void __iomem *cortina_ni_rx_gphy(struct cortina_ni *ni)
{
	/* the gphy window is optional in the DT: NULL = feature unavailable */
	if (!ni->win[CA_NI_WIN_GPHY])
		return NULL;
	return ni->win[CA_NI_WIN_GPHY] + CA_NI_GPHY_BANK(CA_NI_RX_PORT);
}

static u32 cortina_ni_rx_gphy_fault(struct cortina_ni *ni)
{
	void __iomem *gphy = cortina_ni_rx_gphy(ni);

	return gphy ? readl(gphy + CA_NI_GPHY_FAULT) & 0xffff : 0;
}

/* analog calibration registers stock reloads after the reinit */
static const u32 cortina_ni_rx_gphy_cal_off[CA_NI_RX_GPHY_CAL_REGS] = {
	CA_NI_GPHY_EXT(0xbcd, 22),	/* rc_cal_len_l */
	CA_NI_GPHY_EXT(0xbcd, 23),
	CA_NI_GPHY_EXT(0xbcf, 18),	/* r_cal (tapbin A-D) */
	CA_NI_GPHY_EXT(0xbcf, 19),
	CA_NI_GPHY_EXT(0xbcf, 20),
	CA_NI_GPHY_EXT(0xbcf, 21),
	CA_NI_GPHY_EXT(0xbca, 22),	/* amp_cal (ibadj) */
};

static void cortina_ni_rx_gphy_cal_save(struct cortina_ni *ni)
{
	void __iomem *gphy = ni->win[CA_NI_WIN_GPHY];
	unsigned int b;
	int i;

	if (!gphy)
		return;

	/* taken at probe: the U-Boot-initialized state that just TFTP'd the kernel over
	 * the cabled port, i.e. a proven-working calibration.  Snapshot EVERY bank so
	 * the per-port reinit can restore the cabled port's cal (any of 0..3). */
	for (b = 0; b < CA_NI_GPHY_COUNT; b++)
		for (i = 0; i < CA_NI_RX_GPHY_CAL_REGS; i++)
			ni->rx->gphy_cal[b][i] =
				readl(gphy + CA_NI_GPHY_BANK(b) +
				      cortina_ni_rx_gphy_cal_off[i]);
}

/* Force the GPHY-wrapper enables to the stock golden steady ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 122. */
static void cortina_ni_rx_wrap_establish(struct cortina_ni *ni)
{
	void __iomem *wrap = ni->win[CA_NI_WIN_GPHY_WRAP];

	if (!wrap)
		return;
	writel(CA_NI_GPHY_WRAP_EN0_VAL, wrap + CA_NI_GPHY_WRAP_EN0);
	writel(CA_NI_GPHY_WRAP_EN1_VAL, wrap + CA_NI_GPHY_WRAP_EN1);
}

/* ★ Per-port GPHY<->MAC interface establishment (Fable RE ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 123. */
static void cortina_ni_rx_gphy_intf_establish_locked(struct cortina_ni *ni,
						    unsigned int port)
{
	void __iomem *gphy = ni->win[CA_NI_WIN_GPHY];
	void __iomem *wrap = ni->win[CA_NI_WIN_GPHY_WRAP];
	void __iomem *bank;
	u32 val;
	int i;

	if (!gphy || !ni->mii || port >= CA_NI_GPHY_COUNT)
		return;
	bank = gphy + CA_NI_GPHY_BANK(port);

	mutex_lock(&ni->mii->mdio_lock);

	/* 1st INTF_RST pulse for THIS port */
	writel(CA_NI_HV_INTF_RST_GPHY(port), ni_base(ni) + CA_NI_HV_INTF_RST);
	usleep_range(1000, 1500);
	writel(0, ni_base(ni) + CA_NI_HV_INTF_RST);

	/* re-enable the uC patch/self-check on THIS bank */
	val = readl(bank + CA_NI_GPHY_PATCH_EN);
	writel(val | CA_NI_GPHY_PATCH_EN_BIT, bank + CA_NI_GPHY_PATCH_EN);

	/* wrapper EN1_IF(port) 0->1 EDGE (connect THIS GPHY to its MAC) */
	if (wrap) {
		val = readl(wrap + CA_NI_GPHY_WRAP_EN1);
		writel(val & ~CA_NI_GPHY_WRAP_EN1_IF(port),
		       wrap + CA_NI_GPHY_WRAP_EN1);
		writel(val | CA_NI_GPHY_WRAP_EN1_IF(port),
		       wrap + CA_NI_GPHY_WRAP_EN1);
	}

	/* 2nd INTF_RST pulse + 200 ms settle (stock) */
	writel(CA_NI_HV_INTF_RST_GPHY(port), ni_base(ni) + CA_NI_HV_INTF_RST);
	usleep_range(1000, 1500);
	writel(0, ni_base(ni) + CA_NI_HV_INTF_RST);
	msleep(200);

	/* restore THIS bank's probe-time analog cal */
	for (i = 0; i < CA_NI_RX_GPHY_CAL_REGS; i++)
		writel(ni->rx->gphy_cal[port][i],
		       bank + cortina_ni_rx_gphy_cal_off[i]);

	/* power up + release hold on THIS bank -- unless the OLT has this UNI
	 * locked, in which case the establish leaves it powered DOWN.  Without
	 * this the 1 Hz recovery work undoes an administrative lock within a
	 * second of it being applied, and nothing reports that it did. */
	val = readl(bank + CA_NI_GPHY_BMCR);
	if (cortina_ni_uni_port_locked(port))
		writel(val | CA_NI_GPHY_BMCR_PDOWN, bank + CA_NI_GPHY_BMCR);
	else
		writel(val & ~CA_NI_GPHY_BMCR_PDOWN, bank + CA_NI_GPHY_BMCR);
	val = readl(bank + CA_NI_GPHY_HOLD);
	writel(val & ~CA_NI_GPHY_HOLD_BIT, bank + CA_NI_GPHY_HOLD);

	/* land the wrapper on stock steady EN1=0x1001 */
	cortina_ni_rx_wrap_establish(ni);

	mutex_unlock(&ni->mii->mdio_lock);
	dev_info(ni->dev, "gphy port %u: MAC<->GPHY interface established\n",
		 port);
}

/* 1 Hz self-rearming poll, stock cadence ("recover check first").  Runs
 * between open and stop; each pass is one register read unless faulted. */
/* The entry point for callers that do NOT already hold the UNI lock. */
static void cortina_ni_rx_gphy_intf_establish(struct cortina_ni *ni,
					      unsigned int port)
{
	mutex_lock(&cortina_ni_uni_lock);
	cortina_ni_rx_gphy_intf_establish_locked(ni, port);
	mutex_unlock(&cortina_ni_uni_lock);
}

static void cortina_ni_rx_recovery_work(struct work_struct *work)
{
	struct cortina_ni_rx *rx = container_of(to_delayed_work(work),
						struct cortina_ni_rx,
						recovery_work);
	struct cortina_ni *ni = rx->ni;
	u32 fault = cortina_ni_rx_gphy_fault(ni);

	rx->last_fault = fault;
	if (unlikely(fault)) {
		dev_warn(ni->dev,
			 "GPHY port %d fault latch 0x%04x - reinit (#%llu)\n",
			 CA_NI_RX_PORT, fault, rx->recoveries + 1);
		/* the stock port-0 reinit IS gphy_intf_establish(port 0); see the
		 * merge note on that function.  It logs a line of its own. */
		cortina_ni_rx_gphy_intf_establish(ni, CA_NI_RX_PORT);
		rx->recoveries++;
		dev_info(ni->dev, "GPHY port %d reinit done (latch now 0x%04x)\n",
			 CA_NI_RX_PORT, cortina_ni_rx_gphy_fault(ni));
	}

	/* ★ Decoupled datapath bring-up: until every GPHY bank is ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 124. */
	if (rx_decoupled_bringup && !rx->intf_done) {
		unsigned int b;
		bool all_patched;

		/* keep driving the bring-up (patches the GPHY banks) until done */
		rx->bringup_ticks++;
			/* ONE line when the cadence steps down, never per tick: a ...
			 * dev/MEASURED-cortina-ni-rx.c.md sec 125. */
		if (rx->bringup_ticks == CA_NI_RX_BRINGUP_FAST_TICKS + 1 ||
		    rx->bringup_ticks == CA_NI_RX_BRINGUP_MID_TICKS + 1)
			dev_warn(ni->dev,
				 "LAN bring-up still owed after %llu tick(s) (%llu attempt(s)); backing the cadence off to 1/%us - NOT giving up\n",
				 rx->bringup_ticks, rx->bringup_calls,
				 cortina_ni_rx_bringup_period(rx->bringup_ticks));
		if (cortina_ni_rx_bringup_due(rx->bringup_ticks)) {
			rx->bringup_calls++;
			cortina_ni_rx_link_up(ni);
		}

		all_patched = true;
		for (b = 0; b < CA_NI_GPHY_COUNT; b++)
			if (!ni->gphy_patched[b]) {
				all_patched = false;
				break;
			}

			/* ★ once EVERY bank is patched, establish the MAC<->GPHY ...
			 * dev/MEASURED-cortina-ni-rx.c.md sec 126. */
		if (all_patched) {
			unsigned int p;

			rx->intf_done = true;
			dev_info(ni->dev,
				 "all GPHY banks patched -> establishing MAC<->GPHY interface on all ports\n");
			for (p = 0; p < CA_NI_GPHY_COUNT; p++)
				cortina_ni_rx_gphy_intf_establish(ni, p);
		}
	}

	/* ★ Hold the CPU-port (eth0) carrier UP every tick once the ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 187. */
	if (rx->rearms && rx->netdev && !netif_carrier_ok(rx->netdev)) {
		netif_carrier_on(rx->netdev);
		netdev_info(rx->netdev,
			    "CPU-port carrier re-asserted (switch datapath up)\n");
	}

	/* ★ Publish which RJ45s have a PHY link, for the CPU->LAN ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 127. */
	if (ni->mii) {
		u32 link = 0;
		unsigned int p;

		for (p = 0; p < CA_NI_LAN_PORT_COUNT; p++) {
			int a = CA_NI_GPHY_FIRST + p, bmsr;

			mdiobus_read(ni->mii, a, MII_BMSR);	/* clear latch */
			bmsr = mdiobus_read(ni->mii, a, MII_BMSR);
			if (bmsr >= 0 && (bmsr & BMSR_LSTATUS))
				link |= BIT(p);
		}
		cortina_ni_lan_tx_link_set(ni, link);

			/* ★ Same bitmap, one more consumer: the front-panel link lamps
			 * dev/MEASURED-cortina-ni-rx.c.md sec 128. */
		cortina_ni_leds_link_set(link);
	}

	schedule_delayed_work(&rx->recovery_work, HZ);
}

/* phylib link-up hook (called from ... -- dev/MEASURED-cortina-ni-rx.c.md sec 129. */
void cortina_ni_rx_link_up(struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx = ni->rx;

	if (!rx)
		return;		/* TX-only mode */

	/* ★ HELD ACROSS THE WHOLE RESTORING BODY, not around each write.  This function
	 *   exists to put every port back, so "is this one locked" and the writes that
	 *   follow must not be separable by an administrative change landing in between
	 *   -- a stale decision re-enables a socket the OLT was already told is down. */
	mutex_lock(&cortina_ni_uni_lock);

	/* ★ DIAGNOSTIC: dphy_rst reset-manager (GLB+0xa0) = internal digital-PHY reset.
	 * Ours boots 0x50302340 (many reset bits set), STOCK(working)=0x10000000, so
	 * ours holds internal-GPHY/datapath sub-blocks in reset. */
	if (ni->win[CA_NI_WIN_GLB]) {
		void __iomem *glb = ni->win[CA_NI_WIN_GLB];

		dev_info(ni->dev, "dphy_rst(glb+0xa0) was 0x%08x -> writing 0x10000000\n",
			 readl(glb + CA_NI_GLB_BLOCK_RESET));
		writel(0x10000000, glb + CA_NI_GLB_BLOCK_RESET);
	}

	/* ★ Load the internal-GPHY SRAM firmware + resume the uC HERE ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 130. */
	cortina_ni_gphy_patch_and_resume(ni);

	/* ★ Once every GPHY bank is patched, establish the per-port ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 131. */
	if (ni->rx && !ni->rx->intf_done) {
		unsigned int b, p;
		bool all_patched = true;

		for (b = 0; b < CA_NI_GPHY_COUNT; b++)
			if (!ni->gphy_patched[b]) {
				all_patched = false;
				break;
			}
		if (all_patched) {
			ni->rx->intf_done = true;
			dev_info(ni->dev,
				 "all GPHY banks patched -> establishing MAC<->GPHY interface on all ports (in link_up)\n");
			for (p = 0; p < CA_NI_GPHY_COUNT; p++)
				cortina_ni_rx_gphy_intf_establish_locked(ni, p);
		}
	}

	if (!rx_skip_portcfg) {
		unsigned int p;

		/* ★ re-establish the GPHY->port-MAC datapath (wrapper EN1 bit12) FIRST:
		 * this is the ingress determinism gate - without it the PHY links but no
		 * frame reaches the MAC.  Idempotent; also heals any phylib disturbance. */
		cortina_ni_rx_wrap_establish(ni);

			/* ★ re-assert the MAC<->GPHY internal GMII interface ...
			 * dev/MEASURED-cortina-ni-rx.c.md sec 132. */
		for (p = 0; p < CA_NI_GPHY_COUNT; p++)
			ni_rmw(ni, CA_NI_PORT_STATIC_CFG(p),
			       CA_NI_PORT_STATIC_INT_CFG | CA_NI_PORT_STATIC_PHY_MODE |
			       CA_NI_PORT_STATIC_LPBK_MODE, 0);

		/* autosync = 0xF (match STOCK live-Linux; U-Boot's 0 was the
		 * wrong reference for the CPU-EPP RX path) */
		writel(CA_NI_NI_AUTOSYNC_STOCK_VAL,
		       ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC);
	}

	/* re-apply the FULL golden FE-path config: steer/demux/DLF + ES master
	 * (+RSVD25 clear) + cpu_en=0xff + RMU RX + port MAC RX (stock 0x3001) */
	cortina_ni_rx_steer_init(ni);
	cortina_ni_rx_es_enable(ni);
	cortina_ni_rx_es_cpu(ni, true);
	ni_rmw(ni, CA_NI_QM_RMU0_CTRL, 0, CA_NI_QM_RMU0_RX_EN);
	/* ★ power-up RX + enable RXMAC on EVERY GPHY LAN port (not just
	 * CA_NI_RX_PORT) - same reason as the GMII loop above, so the cabled port
	 * (any of 0..3) ingresses to the CPU. */
	{
		unsigned int p;

		for (p = 0; p < CA_NI_GPHY_COUNT; p++) {
			/* a locked UNI is not re-armed by someone else's
			 * link-up: it stays powered down with its RXMAC off */
			if (cortina_ni_uni_port_locked(p)) {
				ni_rmw(ni, CA_NI_PORT_GLB_CFG(p), 0,
				       CA_NI_PORT_GLB_PWR_DWN_RX);
				ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(p),
				       CA_NI_PORT_RXMAC_RX_EN, 0);
				continue;
			}
			ni_rmw(ni, CA_NI_PORT_GLB_CFG(p),
			       CA_NI_PORT_GLB_PWR_DWN_RX, 0);
			if (!rx_skip_portcfg)
				ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(p),
				       CA_NI_PORT_RXMAC_STOCK_CLR,
				       CA_NI_PORT_RXMAC_RX_EN |
				       CA_NI_PORT_RXMAC_STOCK_SET);
			else	/* leave U-Boot's rxmac bits, just ensure RX_EN on */
				ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(p), 0,
				       CA_NI_PORT_RXMAC_RX_EN);
		}
	}

	rx->rearms++;
	dev_info(ni->dev,
		 "RX (re)armed on link-up (#%llu): wrap_en1=0x%08x rxmac=0x%08x es_ctrl=0x%08x gphy_fault=0x%04x\n",
		 rx->rearms,
		 ni->win[CA_NI_WIN_GPHY_WRAP] ?
			readl(ni->win[CA_NI_WIN_GPHY_WRAP] + CA_NI_GPHY_WRAP_EN1) : 0,
		 readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT)),
		 readl(ni_base(ni) + CA_NI_QM_ES_CTRL),
		 cortina_ni_rx_gphy_fault(ni));

	/* ★ eth0 is the CPU<->switch port, NOT a single physical ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 133. */
	if (rx->netdev && !netif_carrier_ok(rx->netdev))
		netif_carrier_on(rx->netdev);

	/* fault check now instead of waiting for the next 1 Hz tick */
	mutex_unlock(&cortina_ni_uni_lock);

	if (cortina_ni_rx_gphy(ni))
		mod_delayed_work(system_wq, &rx->recovery_work, 0);
}

/* ------------------------------------------------------------------ */
/* open/stop hooks (called from the netdev ops in cortina-ni-tx.c)     */
/* ------------------------------------------------------------------ */

void cortina_ni_rx_open(struct cortina_ni *ni)
{
	if (!ni->rx)
		return;		/* TX-only mode (RX probe failed/absent) */

	napi_enable(&ni->rx->napi);

	/* arm the CPU-port drain (stock enable_tx_cpu at ca_ni_open) so the ES
	 * scheduler starts writing descriptors for our CPU port */
	cortina_ni_rx_es_cpu(ni, true);

	/* port-0 MAC RX on (M2b left it off), stock pattern 0x3001: ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 134. */
	mutex_lock(&cortina_ni_uni_lock);
	if (cortina_ni_uni_port_locked(CA_NI_RX_PORT)) {
		ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT),
		       CA_NI_PORT_RXMAC_RX_EN, 0);
		ni_rmw(ni, CA_NI_PORT_GLB_CFG(CA_NI_RX_PORT), 0,
		       CA_NI_PORT_GLB_PWR_DWN_RX | CA_NI_PORT_GLB_PWR_DWN_TX);
	} else {
		if (!rx_skip_portcfg)
			ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT),
			       CA_NI_PORT_RXMAC_STOCK_CLR,
			       CA_NI_PORT_RXMAC_RX_EN | CA_NI_PORT_RXMAC_STOCK_SET);
		else	/* leave U-Boot's rxmac bits, just ensure RX_EN on */
			ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT), 0,
			       CA_NI_PORT_RXMAC_RX_EN);
		ni_rmw(ni, CA_NI_PORT_GLB_CFG(CA_NI_RX_PORT),
		       CA_NI_PORT_GLB_PWR_DWN_RX, 0);
	}
	mutex_unlock(&cortina_ni_uni_lock);

	cortina_ni_rx_irq_set(ni, true);

	/* start the 1 Hz GPHY fault poll (stock runs it for the device's
	 * whole lifetime; ours runs while the netdev is up) */
	if (cortina_ni_rx_gphy(ni))
		schedule_delayed_work(&ni->rx->recovery_work, HZ);
}

void cortina_ni_rx_stop(struct cortina_ni *ni)
{
	unsigned int i;

	if (!ni->rx)
		return;

	cancel_delayed_work_sync(&ni->rx->recovery_work);

	/* stop new ingress first, then quiesce the drain side.  Buffers already pushed
	 * to the HW pool CANNOT be popped back (no CPU pop primitive on this path) -
	 * they stay allocated for the device's lifetime and are reused on the next open. */
	ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT),
	       CA_NI_PORT_RXMAC_RX_EN, 0);
	cortina_ni_rx_es_cpu(ni, false);	/* disarm the CPU-port drain */
	cortina_ni_rx_irq_set(ni, false);
	napi_disable(&ni->rx->napi);

	/* NAPI is quiesced, so nothing can be mid-chain: release any partially assembled
	 * frame instead of carrying it across a close/open (the accounting would be
	 * reset by the next SOP, but the skb would not be - one per voq). */
	for (i = 0; i < CA_NI_RX_VOQ_COUNT; i++)
		cortina_ni_rx_chain_reset(ni->rx, &ni->rx->chain[i]);
}

/* spy/dump hook: /proc/net/cortina_ni_rx (project rule: ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 135. */
u32 cortina_ni_rx_mib_read(struct cortina_ni *ni, u32 port, u32 cnt_id)
{
	u32 val;

	if (ca_ni_access_go_paced(ni_base(ni) + CA_NI_HV_RXMIB_ACCESS,
				  CA_NI_MIB_ACCESS_GO |
				  FIELD_PREP(CA_NI_MIB_ACCESS_OPCODE,
					     CA_NI_MIB_OP_READ_ONLY) |
				  FIELD_PREP(CA_NI_MIB_ACCESS_PORT, port) |
				  FIELD_PREP(CA_NI_MIB_ACCESS_CNTID, cnt_id),
				  &val, CA_NI_MIB_POLL_US,
				  CA_NI_MIB_POLL_TIMEOUT_US))
		return ~0u;
	return readl(ni_base(ni) + CA_NI_HV_RXMIB_DATA0);
}

/* values `ethtool -d` structurally cannot carry, published to ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 136. */
u32 cortina_ni_rx_epp_wrptr(struct cortina_ni *ni, unsigned int voq)
{
	if (!ni_base(ni) || voq >= CA_NI_RX_VOQ_COUNT)
		return 0;
	return cortina_ni_rx_wptr_voq(ni, voq);
}

/* Central-buffer occupancy, AGGREGATED here rather than ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 137. */
void cortina_ni_rx_cb_occupancy(struct cortina_ni *ni, u64 *total, u64 *max,
				u64 *nonzero)
{
	unsigned int q;

	*total = 0;
	*max = 0;
	*nonzero = 0;
	if (!ni_base(ni))
		return;

	for (q = 0; q < CA_NI_RX_CB_VOQ_ENTRIES; q++) {
		u32 c, pages;

		c = cortina_ni_rx_ind_entry(ni, CA_NI_L2TM_CB_VOQ_BUFCNT_ACCESS,
						     q, CA_NI_L2TM_CB_VOQ_BUFCNT_DATA);
		if (!c)
			continue;
		pages = c >> CA_NI_RX_CB_VOQ_PAGES_SHIFT;
		*total += pages;
		if (pages > *max)
			*max = pages;
		(*nonzero)++;
	}
}

/* The per-port free-buffer count register WORD, undecoded and named as a word:
 * the packing is not proven on this silicon, so a row called "free pages"
 * would be naming it wrongly. */
u32 cortina_ni_rx_cb_port_free_word(struct cortina_ni *ni, unsigned int port)
{
	if (!ni_base(ni))
		return 0;
	/* ~0u, NOT 0, on a stuck indirect access. This word leaves ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 138. */
	if (cortina_ni_rx_ind_read(ni, CA_NI_L2TM_CB_PORT_FREECNT_ACCESS, port))
		return ~0u;
	return readl(ni_base(ni) + CA_NI_L2TM_CB_PORT_FREECNT_DATA);
}

/* Per-GPHY-port PHY link. ★ THIS IS THE ONE ANSWER TO "WHICH ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 139. */
int cortina_ni_rx_phy_link(struct cortina_ni *ni, unsigned int port)
{
	int addr, bmsr;

	if (!ni->mii || port >= CA_NI_GPHY_COUNT)
		return -1;
	addr = CA_NI_GPHY_FIRST + port;
	mdiobus_read(ni->mii, addr, MII_BMSR);		/* clear the latch */
	bmsr = mdiobus_read(ni->mii, addr, MII_BMSR);
	if (bmsr < 0)
		return -1;
	return !!(bmsr & BMSR_LSTATUS);
}

/* Full curated NI-window register snapshot for a ... -- dev/MEASURED-cortina-ni-rx.c.md sec 140. */
static const struct {
	const char	*name;
	u32		off;
} cortina_ni_rx_regs[] = {
	/* NI_HV globals */
	{ "hv_init_done",	CA_NI_HV_INIT_DONE },
	{ "hv_intf_rst",	CA_NI_HV_INTF_RST },
	{ "hv_mac_autosync",	CA_NI_HV_MAC_AUTOSYNC },
	{ "hv_pkt_len",		CA_NI_HV_PKT_LEN },
	{ "hv_pkt_len_rx",	CA_NI_HV_PKT_LEN_RX },
	{ "hv_cfg_a1b8",	CA_NI_HV_CFG_A1B8 },
	{ "ni_intern_portid",	CA_NI_NI_INTERNAL_PORT_ID_CFG },
	{ "hv_cfg_a420",	CA_NI_HV_CFG_A420 },
	{ "hv_cfg_aaf0",	CA_NI_HV_CFG_AAF0 },
	/* L3FE demux golden routing map (FE output -> CPU-EPP) */
	{ "l3fe_demux0_a190",	CA_NI_NIRX_L3FE_DEMUX0 },
	{ "l3fe_demux1_a194",	CA_NI_NIRX_L3FE_DEMUX1 },
	{ "l3fe_demux2_a198",	CA_NI_NIRX_L3FE_DEMUX2 },
	{ "l3fe_demux3_a19c",	CA_NI_NIRX_L3FE_DEMUX3 },
	{ "l3fe_demux4_a1a0",	CA_NI_NIRX_L3FE_DEMUX4 },
	{ "l3fe_demux5_a1a4",	CA_NI_NIRX_L3FE_DEMUX5 },
	/* deep_q=1 demux (build34): ldpid 0x32 routing = dpq_48_63(0xa1a8) bits[5:4], want 0=L3QM */
	{ "dpq_demux_a1a8",	CA_NI_NIRX_L3FE_DPQ_DEMUX_48_63 },
	{ "dpq_demux_a1ac",	CA_NI_NIRX_L3FE_DPQ_DEMUX_32_47 },
	{ "dpq_demux_a1b0",	CA_NI_NIRX_L3FE_DPQ_DEMUX_16_31 },
	{ "dpq_demux_a1b4",	CA_NI_NIRX_L3FE_DPQ_DEMUX_0_15 },
	/* L2FE forwarding-control (stock 0x140c=0x0c100c10, 0x160c=0x1) */
	{ "l2fe_lrn_fwd1_140c",	CA_NI_L2FE_PLC_LRN_FWD_CTRL_1 },
	{ "l2fe_arb_ext_160c",	CA_NI_L2FE_ARB_CTRL_EXT },
	/* ★ __ni_flow_ctrl_init map (stock: 0x2124=0x88888888 BM dq->TM-port8=QM,
	 * 0x3400=0x001c787c, 0x9798=0x81a80178). If 0x2124!=0x88888888 the BM
	 * dequeues CPU frames to the wrong port -> qm_rx=0. */
	{ "bm_dq_port_map_2124", CA_NI_L2TM_BM_DQ_PORT_MAP },
	{ "ni_flowctrl_en_3400", CA_NI_NI_FLOWCTRL_EN },
	{ "ni_flowthr0_9798",	CA_NI_NI_FLOWCTRL_THRESH },
	/* port-0 MAC block */
	{ "p0_glb",		CA_NI_PORT_GLB_CFG(CA_NI_RX_PORT) },
	{ "p0_rxmac",		CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT) },
	{ "p0_txmac",		CA_NI_PORT_TXMAC_CFG(CA_NI_RX_PORT) },
	{ "p0_rx_cntrl",	CA_NI_PORT_RX_CNTRL_CFG(CA_NI_RX_PORT) },
	/* L2TM (TX-side scheduler/QM, dumped for completeness) */
	{ "l2tm_qm_eq_cfg",	CA_NI_L2TM_QM_EQ_CFG },
	{ "l2tm_qm_glob_buf",	CA_NI_L2TM_QM_GLOB_BUF_CFG },
	{ "l2tm_qm_prvt_prof0",	CA_NI_L2TM_QM_PORT_PRVT_PROF0 },
	{ "l2tm_es_ctrl",	CA_NI_L2TM_ES_CTRL },
	{ "l2tm_es_sch0",	CA_NI_L2TM_ES_SCH_CFG(0) },
	/* L3QM / RMU (RX drain path) */
	{ "qm_rmu0_ctrl",	CA_NI_QM_RMU0_CTRL },
	{ "qm_es_ctrl_6108",	CA_NI_QM_ES_CTRL },
	{ "qm_l3tm_ni_ena",	CA_NI_QM_L3TM_NI_PORT_ENA },	/* 0x610c */
	{ "qm_int_en0",		CA_NI_QM_EPP64_INT_EN0 },
	{ "qm_int_en1",		CA_NI_QM_EPP64_INT_EN1 },
	{ "qm_eq_profile13",	CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE) },  /* {eqp0=13,eqp1=14} */
	{ "qm_eq14_cfg1",	CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID2) },
	{ "qm_eq14_pa_req",	CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID2) },
	{ "qm_es_ctrl2",	CA_NI_QM_ES_CTRL2 },
	{ "qm_epp_cmd",		CA_NI_QM_EPP },
	{ "qm_destp0_eq_cfg",	CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT) },
	{ "qm_destp0_pkt_buf",	CA_NI_QM_DEST_PORT_PKT_BUF_CFG(CA_NI_RX_CPU_PORT) },
	{ "qm_eq13_cfg0",	CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ_ID) },
	{ "qm_eq13_cfg1",	CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID) },
	{ "qm_eq13_cfg2",	CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID) },
	{ "qm_eq13_cfg3",	CA_NI_QM_CFG3_EQ(CA_NI_RX_EQ_ID) },
	{ "qm_eq13_pa_req",	CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID) },
	{ "qm_push_ready0",	CA_NI_QM_CPU_PUSH_READY(CA_NI_RX_CPU_PORT) },
	{ "qm_voq_en0",		CA_NI_QM_VOQ_EN(CA_NI_RX_CPU_PORT) },
	{ "qm_eq_cfg_load",	CA_NI_QM_EQ_CFG_LOAD },
	{ "qm_hdm_wr_prot",	CA_NI_QM_HDM_WRITE_PROT },
	{ "qm_rx_status0",	CA_NI_QM_RX_STATUS0 },
	{ "qm_rx_status1",	CA_NI_QM_RX_STATUS1 },
	{ "qm_tx_cntr",		CA_NI_QM_TX_CNTR },
	{ "qm_rx_cntr",		CA_NI_QM_RX_CNTR },
	/* CPU-EPP FIFO config + ring pointers (port0/voq0) */
	{ "qm_cpu_epp_cfg0",	CA_NI_QM_CPU_EPP_CFG(CA_NI_RX_CPU_PORT) },
	{ "qm_cpu_epp_ct",	CA_NI_QM_CPU_EPP_CT_CFG },
	{ "qm_epp_fifo_prof0_66a4",	CA_NI_QM_CPU_EPP_FIFO_PROF(0) },	/* stock 0xE0008001 */
	{ "qm_epp_fifo_prof4",	CA_NI_QM_CPU_EPP_FIFO_PROF(CA_NI_RX_PROFILE_ID) },
	{ "qm_epp_fifo_cfg0",	CA_NI_QM_CPU_EPP_FIFO_CFG(CA_NI_RX_CPU_PORT,
							 CA_NI_RX_VOQ) },
	{ "qm_epp_wrptr0",	CA_NI_QM_EPP64_WRPTR(CA_NI_RX_CPU_PORT,
						     CA_NI_RX_VOQ) },
	{ "qm_epp_rdptr0",	CA_NI_QM_EPP64_RDPTR(CA_NI_RX_CPU_PORT,
						     CA_NI_RX_VOQ) },
	{ "qm_epp_paddr0",	CA_NI_QM_EPP64_PADDR_START(CA_NI_RX_CPU_PORT,
							   CA_NI_RX_VOQ) },
};

static void cortina_ni_rx_dump_regs(struct seq_file *m, struct cortina_ni *ni)
{
	int i;

	seq_puts(m, "regs:\n");
	for (i = 0; i < ARRAY_SIZE(cortina_ni_rx_regs); i++)
		seq_printf(m, "  %-20s ni+0x%04x = 0x%08x\n",
			   cortina_ni_rx_regs[i].name,
			   cortina_ni_rx_regs[i].off,
			   readl(ni_base(ni) + cortina_ni_rx_regs[i].off));

	/* Per-port-0 MAC block (0xa5c4..0xa630): the GMAC-level mac_rx=0 gate is a
	 * per-port config we set only 4 of - dump the mapped ones so a good-vs-bad diff
	 * finds the interface/media/enable we skip.  NOTE: 0xa634..0xa64c is an unmapped
	 * hole on this SoC (readl faults), so stop at 0xa630 - never widen past 0x70. */
	seq_puts(m, "port0 block (0xa5c0..0xa630):\n");
	/* ★ 0xa5c0 = STATIC_CFG (MAC<->PHY interface: int_cfg[3:0]/phy_mode[4]/
	 * lpbk[13:12]) - the true port-block base, one word below GLB_CFG, and
	 * the bidirectional datapath gate our driver used to skip */
	seq_printf(m, "  p0-0x04   ni+0x%04x = 0x%08x  <- STATIC_CFG (int_cfg/phy_mode/lpbk)\n",
		   CA_NI_PORT_STATIC_CFG(CA_NI_RX_PORT),
		   readl(ni_base(ni) + CA_NI_PORT_STATIC_CFG(CA_NI_RX_PORT)));
	for (i = 0; i < 0x70; i += 4) {
		u32 off = CA_NI_PORT_GLB_CFG(CA_NI_RX_PORT) + i;

		seq_printf(m, "  p0+0x%02x   ni+0x%04x = 0x%08x\n",
			   i, off, readl(ni_base(ni) + off));
	}

	/* the port-0 GPHY block (bank 0): fault latch + BMCR/BMSR + link */
	if (cortina_ni_rx_gphy(ni)) {
		void __iomem *gphy = cortina_ni_rx_gphy(ni);

		seq_printf(m, "  %-20s gphy+0x%05x = 0x%08x\n", "gphy_fault",
			   CA_NI_GPHY_FAULT, readl(gphy + CA_NI_GPHY_FAULT));
		seq_printf(m, "  %-20s gphy+0x%05x = 0x%08x\n", "gphy_bmcr",
			   CA_NI_GPHY_BMCR, readl(gphy + CA_NI_GPHY_BMCR));
		seq_printf(m, "  %-20s gphy+0x%05x = 0x%08x\n", "gphy_bmsr",
			   CA_NI_GPHY_BMCR + CA_NI_GPHY_REG_STRIDE,
			   readl(gphy + CA_NI_GPHY_BMCR + CA_NI_GPHY_REG_STRIDE));
	}

	/* ★ GPHY wrapper: EN1 bit12 (patch_phy_done) is the ingress determinism
	 * gate; also the analog/datapath regs w+00..0c (golden 0x3110/0x10ff0/
	 * 0x10bc06/0x0800a400) for a good-vs-bad-boot diff */
	if (ni->win[CA_NI_WIN_GPHY_WRAP]) {
		void __iomem *w = ni->win[CA_NI_WIN_GPHY_WRAP];
		static const struct { const char *n; u32 off; } wr[] = {
			{ "wrap_r00", CA_NI_GPHY_WRAP_R00 },
			{ "wrap_r04", CA_NI_GPHY_WRAP_R04 },
			{ "wrap_r08", CA_NI_GPHY_WRAP_R08 },
			{ "wrap_r0c", CA_NI_GPHY_WRAP_R0C },
			{ "wrap_en0", CA_NI_GPHY_WRAP_EN0 },
			{ "wrap_en1", CA_NI_GPHY_WRAP_EN1 },
		};

		for (i = 0; i < ARRAY_SIZE(wr); i++)
			seq_printf(m, "  %-20s wrap+0x%02x = 0x%08x\n",
				   wr[i].n, wr[i].off, readl(w + wr[i].off));
	}
}

/* ★ QM+L2TM full-block offset sweep (ours-vs-stock diff hunt ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 141. */
static const struct cortina_ni_reg_range {
	u16 base;
	u16 count;	/* consecutive u32 regs at stride 4 */
} cortina_ni_qmdump_ranges[] = {
	{ 0x2000,  10 },	/* ..0x2024 */
	{ 0x2100,  46 },	/* ..0x21b4  L2TM TM/BM cfg + counters */
	{ 0x2200,  43 },	/* ..0x22a8  L2TM QM EQ/buffer cfg */
	{ 0x2300,  64 },	/* ..0x23fc  L2TM ES + the deepq direct cfg */
	{ 0x6000,   4 },	/* ..0x600c  QM AXIM cfg */
	{ 0x6100, 627 },	/* ..0x6ac8  the whole QM block */
	/* ★ CPU-EPP ring pointer block (wptr 0x7000 = THE drain-alive
	 * discriminator: stock advances, ours stuck 0) + per-voq ring config, so
	 * the qmblock dump shows the EPP delivery stage next to the QM
	 * drain-map. */
	{ 0x7000,  12 },	/* ..0x702c */
};

/* `ethtool -d`: the same curated snapshot, through a standard ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 142. */
unsigned int cortina_ni_regdump_len(void)
{
	unsigned int i, n = ARRAY_SIZE(cortina_ni_rx_regs);

	for (i = 0; i < ARRAY_SIZE(cortina_ni_qmdump_ranges); i++)
		n += cortina_ni_qmdump_ranges[i].count;
	return n;
}

void cortina_ni_regdump_fill(struct cortina_ni *ni, u32 *buf)
{
	unsigned int i, k, n = 0;

	if (!ni_base(ni)) {
		memset(buf, 0, cortina_ni_regdump_len() * sizeof(*buf));
		return;
	}
	for (i = 0; i < ARRAY_SIZE(cortina_ni_rx_regs); i++)
		buf[n++] = readl(ni_base(ni) + cortina_ni_rx_regs[i].off);
	for (i = 0; i < ARRAY_SIZE(cortina_ni_qmdump_ranges); i++)
		for (k = 0; k < cortina_ni_qmdump_ranges[i].count; k++)
			buf[n++] = readl(ni_base(ni) +
					 cortina_ni_qmdump_ranges[i].base + 4 * k);
}

/* Decode key for word @i of the dump above: its name and its NI-window offset.
 * Generated from the SAME tables as the dump, so the map cannot describe a
 * different snapshot than the one taken.  The sweep half has no vendor name. */
void cortina_ni_regdump_entry(unsigned int i, const char **name, u32 *off)
{
	unsigned int r;

	if (i < ARRAY_SIZE(cortina_ni_rx_regs)) {
		*name = cortina_ni_rx_regs[i].name;
		*off = cortina_ni_rx_regs[i].off;
		return;
	}
	i -= ARRAY_SIZE(cortina_ni_rx_regs);
	for (r = 0; r < ARRAY_SIZE(cortina_ni_qmdump_ranges); r++) {
		if (i < cortina_ni_qmdump_ranges[r].count) {
			*name = "qm_l2tm_sweep";
			*off = cortina_ni_qmdump_ranges[r].base + 4 * i;
			return;
		}
		i -= cortina_ni_qmdump_ranges[r].count;
	}
	*name = "<out of range>";
	*off = 0;
}

/* ★ BOTH DIRECTIONS' CPU-forward witness, in ONE read. ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 143. */
void cortina_ni_cpu_fwd_show(struct seq_file *m, struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx = ni->rx;
	struct cortina_ni_tx *tx = ni->tx;

	seq_printf(m,
		   "cpu_fwd: us_data_enq=%llu [UPSTREAM only: LAN->WAN frames the CPU enqueued to PON TX] ds_wan_l3=%llu [DOWNSTREAM: HW-L3 miss punted to CPU] ds_wan_pon=%llu [DOWNSTREAM: lspid=PON, incl. terminating traffic] -- BOTH directions must stay FLAT to claim HW offload; us_data_enq alone proves nothing about DS\n",
		   tx ? tx->pon_data_enq : 0ULL,
		   rx ? rx->wan_l3_frames : 0ULL,
		   rx ? rx->wan_frames : 0ULL);
}

/* The RX-side narrative dump. ★ IT IS DEBUGFS NOW, NOT /proc ...
 * dev/MEASURED-cortina-ni-rx.c.md sec 144. */
static void rx_dump_fwd_chain(struct seq_file *m, struct cortina_ni *ni,
			      u64 l3fe_rx, u64 l3qm_rx)
{
	u32 addr = CA_NI_RX_PORT << 2;	/* port-0, DLF type 0 */
	u32 dft = 0, pdpid = 0, p19 = 0;

	if (!cortina_ni_rx_ind_read(ni, CA_NI_PLE_DFT_FWD_ACCESS, addr))
		dft = readl(ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);

	pdpid = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
					CA_NI_RX_CPU_LDPID,
					CA_NI_L2FE_PDPID_MAP_DATA) &
		CA_NI_L2FE_PDPID_MAP_PDPID;

	/* PDPID_MAP[0x19] (L3_LAN classifier output): must read QM(0x08)
	 * after our remap so my-MAC/ARP frames reach the RMU, not ES8. */
	p19 = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				      CA_NI_RX_L3LAN_LDPID,
				      CA_NI_L2FE_PDPID_MAP_DATA) &
	      CA_NI_L2FE_PDPID_MAP_PDPID;

	/* PDPID_MAP[0x18] (L3_WAN): the HW-L3 DS ingress admission - a PON
	 * PDC frame stamped ldpid L3_WAN must resolve to pdpid 0x0a (the
	 * L3FE WAN physical ingress).  0 here = the DS data GEM's L3_WAN
	 * frames never enter the L3FE (stock live [0x18]=0xA). */
	{
		u32 p18;

		p18 = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
					      CA_NI_RX_L3WAN_LDPID,
					      CA_NI_L2FE_PDPID_MAP_DATA) &
		      CA_NI_L2FE_PDPID_MAP_PDPID;
		seq_printf(m, "fwd-chain: pdpid[0x18]=0x%x (L3_WAN; stock 0x0a = L3FE WAN ingress; 0 = DS never enters L3FE)\n",
			   p18);
	}

	/* ★ pdpid[0x19]=0x0d (L3_LAN) is STOCK-CORRECT (vendor ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 145. */
	seq_printf(m,
		   "fwd-chain: dft_fwd[p0]=0x%08x (redir_en=%u mcgid=0x%lx) pdpid[0x10]=0x%x pdpid[0x19]=0x%x(stock 0x0d L3_LAN; NOT 0x8=QM->wire) qm_rx=%u qm_tx=%u l3fe_rx(0xa9bc)=%llu l3qm_rx(0xa9fc)=%llu [totals since boot]\n",
		   dft, !!(dft & CA_NI_PLE_DFT_REDIR_EN),
		   FIELD_GET(CA_NI_PLE_DFT_MC_GROUP_ID, dft), pdpid, p19,
		   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
		   readl(ni_base(ni) + CA_NI_QM_TX_CNTR),
		   l3fe_rx, l3qm_rx);	/* the sampled TOTAL; see the note at the top */
}

/* the default-forward table and the RMU0 RX header. */
static void rx_dump_dft_fwd_and_rmu(struct seq_file *m, struct cortina_ni *ni)
{
	unsigned int i;
	u32 v;

	seq_puts(m, "build68 dft_fwd[0..15]:");
	for (i = 0; i < 16; i++) {
		u32 d = 0;

		if (!cortina_ni_rx_ind_read(ni, CA_NI_PLE_DFT_FWD_ACCESS, i << 2))
			d = readl(ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);
		seq_printf(m, " [%u]=0x%08x", i, d);
	}
	/* verify the VLAN check-id map is programmed (the CPU-RX-dead fix) */
	v = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_CHKID_MAP_ACCESS,
					     0x10, CA_NI_L2FE_CHKID_MAP_DATA);
	seq_printf(m, "  chkid[CPU_0]=%u(want 8) chkid[L3_LAN]=%u(want 15)\n",
		   v, cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_CHKID_MAP_ACCESS,
					      0x19, CA_NI_L2FE_CHKID_MAP_DATA));

	/* ★ 2026-07-15: real MC_FIB is @0x1644 and STOCK KEEPS IT EMPTY (no
	 * flood-to-CPU).  Dump it (want all 0) plus the 0x1634 table once misread as
	 * MC_FIB (want stock's 0F 04 0F 09 .. values). */
	seq_puts(m, "mc_fib@0x1644 [0x10..0x1b] D2:");
	for (i = 0x10; i <= 0x1b; i++)
		seq_printf(m, " [0x%x]=0x%08x", i,
			   cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_MC_FIB_ACCESS, i,
						   CA_NI_L2FE_MC_FIB_DATA2));
	seq_puts(m, "  (want all 0 = stock EMPTY)\ntbl@0x1634 [0x10..0x1b]:");
	for (i = 0x10; i <= 0x1b; i++)
		seq_printf(m, " [0x%x]=0x%08x", i,
			   cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_NKPOL_MAP_ACCESS, i,
						   CA_NI_L2FE_NKPOL_MAP_DATA));
	seq_printf(m, "  (want stock 0f 04 0f 09 0f 05 0f 0a 0f 0b 0f 0c; arb_ctrl0x1600=0x%08x want 0x89c71c82; dq_tmport0x212c=0x%08x want 0x76543210)\n",
		   readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_DQ_TO_TM_PORT_MAP));

	/* the full mc_fib[0x19] entry - all 5 Elnath data words (want all 0) */
	{
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_MC_FIB_ACCESS, 0x19);
		seq_printf(m,
			   "mc_fib[0x19] full D4..D0(0x1648..0x1658): %08x %08x %08x %08x %08x (want all 0 = stock)\n",
			   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA4),
			   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA3),
			   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA2),
			   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA1),
			   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA0));
	}

	/* ★ the admitted-frame header - THE last-ring witness.  Want 0x80000010 (dest
	 * 0x10=CPU0, deep_q CLEAR) like stock, NOT 0xc0000020 (dest 0x20=CPU_MQ +
	 * deep_q -> wrong CPU-EPP256 ring). */
	v = readl(ni_base(ni) + CA_NI_QM_RMU0_RX_HDR_INFO0);
	seq_printf(m,
		   "build69 rmu0_rx_hdr(0x6904)=0x%08x dest_ldpid=0x%lx deep_q=%u (want 0x80000010 dest 0x10 deep_q 0); rmu_rx(0x6900)=%u epp_wptr(0x7000)=0x%08x\n",
		   v, FIELD_GET(CA_NI_QM_RMU0_RX_DEST_LDPID, v),
		   !!(v & CA_NI_QM_RMU0_RX_DEEP_Q),
		   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
		   readl(ni_base(ni) + CA_NI_QM_EPP64_WRPTR(0, 0)));
}

/* the L3 classifier KEY/FIB readback -- the ARP-trap tables,
 * so ours can be diffed against the stock golden. */
static void rx_dump_cls_keys_and_fib(struct seq_file *m, struct cortina_ni *ni)
{
	unsigned int e, w;

	for (e = 0; e < CA_NI_RX_CLS_ENTRIES; e++) {
		cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_KEY_ACCESS, e);
		seq_printf(m, "build71 cls_key[%2u]:", e);
		for (w = 0; w < CA_NI_L3FE_CLS_KEY_WORDS; w++)
			seq_printf(m, " %08x",
				   readl(ni_base(ni) +
					 CA_NI_L3FE_CLS_KEY_DATA_BASE + 4 * w));
		cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_FIB_ACCESS, e);
		seq_puts(m, "  fib:");
		for (w = 0; w < CA_NI_L3FE_CLS_FIB_WORDS; w++)
			seq_printf(m, " %08x",
				   readl(ni_base(ni) +
					 CA_NI_L3FE_CLS_FIB_DATA_BASE + 4 * w));
		seq_puts(m, "\n");
	}
	seq_printf(m, "build73 stg0_ctrl(0x3400)=0x%08x (want 0x001c787c) spcl_pkt_det(0x3218)=0x%08x my_mac lo(0x3210)=0x%08x hi(0x3214)=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_L3FE_STG0_CTRL),
		   readl(ni_base(ni) + CA_NI_L3FE_SPCL_PKT_DET_CFG),
		   readl(ni_base(ni) + CA_NI_L3FE_MY_MAC_LO),
		   readl(ni_base(ni) + CA_NI_L3FE_MY_MAC_HI));

	/* ★★ WITHDRAWN 2026-07-25 - the "cls_hit[0..3]" probe was ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 146. */
	seq_printf(m,
		   "l3fe_glb: cls_mon_ctrl(0x30b0)=0x%08x cls_mon_return(0x30b4)=0x%08x (also written, inertly, with 0x%08x - see the resolved-conflict note in cortina-ni-regs.h; monitor enable is BIT(8), and the real stage counters live in /proc/cortina_l3fe)\n",
		   readl(ni_base(ni) + CA_NI_L3FE_CLS_MON_CTRL),
		   readl(ni_base(ni) + CA_NI_L3FE_CLS_MON_RETURN),
		   CA_NI_L3FE_CLS_MON_RETURN_VAL);

	/* ★ CPU-trap rows: the LAN classifier searches KEY[64..127]; ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 147. */
	{
		unsigned int k;

		seq_puts(m, "cls golden rows:");
		for (k = 0; k < ARRAY_SIZE(cls_key_golden); k++) {
			u16 row = cls_key_golden[k].idx;

			cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_KEY_ACCESS, row);
			seq_printf(m, " key[%u]{w0=0x%08x tr=0x%08x}", row,
				   readl(ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS + 11 * 4),
				   readl(ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS + 1 * 4));
		}
		for (k = 0; k < ARRAY_SIZE(cls_fib_golden); k++) {
			u16 row = cls_fib_golden[k].idx;

			cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_FIB_ACCESS, row);
			seq_printf(m, " fib[%u]{d4=0x%08x d6=0x%08x}", row,
				   readl(ni_base(ni) + CA_NI_L3FE_CLS_FIB_ACCESS + 3 * 4),
				   readl(ni_base(ni) + CA_NI_L3FE_CLS_FIB_ACCESS + 1 * 4));
		}
		seq_puts(m, " (want fib[264] d4=0x1c000000 d6=0x600)\n");
	}
}

/* the L2FE arbitration and deep-queue admission. */
static void rx_dump_l2fe_arbitration(struct seq_file *m, struct cortina_ni *ni)
{
	u32 arb = readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL);
	u32 portdbuf, pd0, pd1, bmhdr;

	portdbuf = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_ARB_PORT_DBUF_ACCESS,
					     0, CA_NI_L2FE_ARB_PORT_DBUF_DATA);
	pd0 = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				      CA_NI_RX_REDIR_LDPID,
				      CA_NI_L2FE_PDPID_MAP_DATA) &
	      CA_NI_L2FE_PDPID_MAP_PDPID;
	pd1 = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				      CA_NI_L2FE_PDPID_IDX_DBUF |
				      CA_NI_RX_REDIR_LDPID,
				      CA_NI_L2FE_PDPID_MAP_DATA) &
	      CA_NI_L2FE_PDPID_MAP_PDPID;
	bmhdr = cortina_ni_rx_ind_entry(ni, CA_NI_L2TM_BM_PKT_MEM_ACCESS,
					     0, CA_NI_L2TM_BM_PKT_MEM_DATA7);

	seq_printf(m,
		   "arb-deepq: arb_ctrl=0x%08x (dbuf_sel=%u dbuf_dpid=%lu use_hdr_a=%u) port_dbuf[0]=0x%08x pdpid{DeepQ0,dbuf0/1}=0x%x/0x%x bm_word0=0x%08x (deep_q=%u cpu=%u)\n",
		   arb, !!(arb & CA_NI_L2FE_ARB_DBUF_SEL),
		   FIELD_GET(CA_NI_L2FE_ARB_DBUF_DPID, arb),
		   !!(arb & CA_NI_L2FE_ARB_USE_HDR_A_DBUF),
		   portdbuf, pd0, pd1, bmhdr,
		   !!(bmhdr & BIT(30)), !!(bmhdr & BIT(31)));

	/* ★ FLOW_DBUF (the deep_q source when dbuf_sel=1) at traffic time -
	 * want all 0 = stock (0x0f = the build100 deep_q regression is back). */
	{
		u32 fd[4];
		unsigned int k;

		for (k = 0; k < 4; k++)
			fd[k] = cortina_ni_rx_ind_entry(ni,
					CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS, k,
					CA_NI_L2FE_ARB_FLOW_DBUF_DATA);
		seq_printf(m,
			   "flow-dbuf[0..3]@0x165c=0x%08x 0x%08x 0x%08x 0x%08x (want all 0 = stock; 0x0f = deep_q regression)\n",
			   fd[0], fd[1], fd[2], fd[3]);
	}
}

/* the HV init-done and ready-enable gates. */
static void rx_dump_hv_init_and_rdy(struct seq_file *m, struct cortina_ni *ni)
{
	u32 initd = readl(ni_base(ni) + CA_NI_HV_INIT_DONE);
	void __iomem *glb = ni->win[CA_NI_WIN_GLB];

	/* ★ NIRX_MISC_CFG offset is DISPUTED - our driver treats ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 188. */
	seq_printf(m,
		   "gate: ni_init_done(a004)=0x%08x (ni_done=%u) nirx_misc@0xa1bc=0x%08x nirx_misc@0xa1f8=0x%08x (real one holds rdy_en bits9-13 ~0x3e80; bit11=l3felan_rdy)\n",
		   initd, !!(initd & CA_NI_HV_INIT_DONE_NI),
		   readl(ni_base(ni) + CA_NI_NI_NIRX_MISC_CFG),
		   readl(ni_base(ni) + CA_NI_NI_TXFIFO_THR_L3FE_CFG2));
	if (glb)
		seq_printf(m,
			   /* ★ LABELS ANCHORED IN STOCK'S OWN REGISTER TABLE (tier 2), ...
			    * dev/MEASURED-cortina-ni-rx.c.md sec 148. */
			   "gate glb: bist_ctrl4(28)=0x%08x opt_module_status(98)=0x%08x pon_cntl(9c)=0x%08x block_reset(a0)=0x%08x block_reset_ext(a4)=0x%08x\n",
			   readl(glb + CA_NI_GLB_BIST_CONTROL4),
			   readl(glb + CA_NI_GLB_OPT_MODULE_STATUS),
			   readl(glb + CA_NI_GLB_PON_CNTL),
			   readl(glb + CA_NI_GLB_BLOCK_RESET),
			   readl(glb + CA_NI_GLB_BLOCK_RESET_EXT));
}

int cortina_ni_rx_debug_show(struct seq_file *m, void *v)
{
	struct cortina_ni *ni = m->private;
	struct cortina_ni_rx *rx = ni->rx;
	u32 pa_req;
	int i;
	/* ★★ THE CLEAR-ON-READ NI_HV COUNTERS ARE NOT READ HERE ANY ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 149. */
	u64 nihv[CA_NI_NIHV_CNT_COUNT];
	u64 l3fe_rx, l3qm_rx;

	cortina_ni_nihv_sample(ni, nihv);
	l3fe_rx = nihv[CA_NI_NIHV_L3FE_RX];
	l3qm_rx = nihv[CA_NI_NIHV_L3QM_RX];

	/* RX fell back to TX-only (pool never came up): rx is gone, but dump the
	 * QM/pool registers so the failure is debuggable live */
	if (!rx) {
		u32 pr8 = readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID));
		u32 pr9 = readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID2));
		u32 sts = readl(ni_base(ni) + CA_NI_QM_PHY_PORT_STS);

		seq_printf(m, "mode=tx-only (RX pool never came up)\n");
		seq_printf(m, "qm_phy_sts=0x%08x qm_init_done(phantom)=%u | l3qm_sts(0x6988)=0x%08x init_done(b30)=%u [REAL, want 1]\n",
			   sts, !!(sts & CA_NI_QM_INIT_DONE),
			   readl(ni_base(ni) + CA_NI_QM_L3QM_STS),
			   !!(readl(ni_base(ni) + CA_NI_QM_L3QM_STS) & CA_NI_QM_L3QM_INIT_DONE));
		seq_printf(m, "es_ctrl=0x%08x es_ctrl2=0x%08x rmu0=0x%08x\n",
			   readl(ni_base(ni) + CA_NI_QM_ES_CTRL),
			   readl(ni_base(ni) + CA_NI_QM_ES_CTRL2),
			   readl(ni_base(ni) + CA_NI_QM_RMU0_CTRL));
		seq_printf(m, "eq13 cfg0/1/2=0x%08x/0x%08x/0x%08x pa_req=0x%08x (req=%u inact=%lu)\n",
			   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ_ID)),
			   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID)),
			   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID)),
			   pr8, !!(pr8 & CA_NI_QM_PA_REQ_READY),
			   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT, pr8));
		seq_printf(m, "eq14 cfg1=0x%08x pa_req=0x%08x (req=%u inact=%lu)\n",
			   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID2)),
			   pr9, !!(pr9 & CA_NI_QM_PA_REQ_READY),
			   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT, pr9));
		seq_printf(m, "push_rdy0=0x%08x profile4=0x%08x destp8_eq_cfg=0x%08x\n",
			   readl(ni_base(ni) + CA_NI_QM_CPU_PUSH_READY(CA_NI_RX_CPU_PORT)),
			   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)),
			   readl(ni_base(ni) +
				 CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)));
		seq_printf(m, "epp wptr=0x%06x rdptr=0x%06x cmd_mode=%s demux_sel=0x%x\n",
			   (u32)(readl(ni_base(ni) + CA_NI_QM_EPP64_WRPTR(0, 0)) &
				 CA_NI_QM_EPP64_PTR),
			   (u32)(readl(ni_base(ni) + CA_NI_QM_EPP64_RDPTR(0, 0)) &
				 CA_NI_QM_EPP64_PTR),
			   (readl(ni_base(ni) + CA_NI_QM_EPP) & CA_NI_QM_EPP_CMD_MODE_64) ?
			   "64b" : "32b",
			   (unsigned int)(readl(ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG) &
					  CA_NI_NI_L3QMRX_DEMUX_SEL_ALL));
		return 0;
	}

	seq_printf(m, "mode=fe-path ring @%pad rptr[0]=0x%03x (8 voqs)\n",
		   &rx->ring_dma, rx->rptr[0]);
	seq_printf(m, "hw wptr=0x%06x rdptr=0x%06x paddr_start=0x%08x\n",
		   (u32)(readl(ni_base(ni) + CA_NI_QM_EPP64_WRPTR(0, 0)) &
			 CA_NI_QM_EPP64_PTR),
		   (u32)(readl(ni_base(ni) + CA_NI_QM_EPP64_RDPTR(0, 0)) &
			 CA_NI_QM_EPP64_PTR),
		   readl(ni_base(ni) + CA_NI_QM_EPP64_PADDR_START(0, 0)));
	seq_printf(m, "rx_cntrl=0x%08x rxmac=0x%08x int_en=0x%08x/0x%08x\n",
		   readl(ni_base(ni) + CA_NI_PORT_RX_CNTRL_CFG(CA_NI_RX_PORT)),
		   readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT)),
		   readl(ni_base(ni) + CA_NI_QM_EPP64_INT_EN0),
		   readl(ni_base(ni) + CA_NI_QM_EPP64_INT_EN1));
	/* QM egress-scheduler drain gate + MAC RX MIB: if es_ctrl has tx_en + our cpu_en
	 * set and mac_rx_{uc,mc,bc} climb while hw wptr stays 0, the fault is downstream
	 * of the MAC (ES/steer/ring); if the MIB stays 0, frames never reach the port MAC
	 * (link/steer/wire). */
	seq_printf(m, "es_ctrl=0x%08x\n", readl(ni_base(ni) + CA_NI_QM_ES_CTRL));
	/* DIAGNOSTIC: read the RX MIB for ALL 4 ports - if the host's frames land
	 * on a port != CA_NI_RX_PORT, our port<->GPHY mapping assumption is wrong */
	{
		int p;
		for (p = 0; p < 4; p++)
			seq_printf(m, "mac_rx_p%d: uc=%u mc=%u bc=%u\n", p,
				   cortina_ni_rx_mib_read(ni, p, CA_NI_MIB_RX_UC_PKT),
				   cortina_ni_rx_mib_read(ni, p, CA_NI_MIB_RX_MC_PKT),
				   cortina_ni_rx_mib_read(ni, p, CA_NI_MIB_RX_BC_PKT));
	}
	/* ★ DATAPATH BISECT (real counters, unlike the phantom MAC MIB): the FIRST stage
	 * that stays 0 after a ping while the prior increments = the death point.  L2FE
	 * ingest -> L2FE drop -> L2FE->TM forward -> QM RMU ingest -> QM drops -> wptr. */
	{
		/* l2fe_ni_pkt = {sop[31:16], eop[15:0]}: sop==eop on stock (clean
		 * frames); ours diverges = looping/fragmented frames.  l2fe_tm_fwd
		 * is the same {sop,eop} form.  (qm_rmu_rx 0x67d8 dropped - phantom,
		 * reads 0 even on working stock.) */
		u32 nipkt = readl(ni_base(ni) + CA_NI_L2FE_NI_INTF_PKT_CNT);
		u32 tmfwd = readl(ni_base(ni) + CA_NI_L2FE_PE_TM_PKT_CNT);

		seq_printf(m,
			   "bisect: l2fe_ni sop=%u eop=%u l2fe_ni_drop=%u l2fe_dos=%u l2fe_tm sop=%u eop=%u qm_eop_drop=%u qm_len_err=%u qm_l2te_drop=%u\n",
			   nipkt >> 16, nipkt & 0xffff,
			   readl(ni_base(ni) + CA_NI_L2FE_NI_INTF_DROP_CNT),
			   readl(ni_base(ni) + CA_NI_L2FE_DOS_FLOOD_CNT),
			   tmfwd >> 16, tmfwd & 0xffff,
			   readl(ni_base(ni) + CA_NI_QM_RX_EOP_DROP_CNTR),
			   readl(ni_base(ni) + CA_NI_QM_RX_LEN_ERR_CNTR),
			   readl(ni_base(ni) + CA_NI_QM_RX_L2TE_DROP_CNTR));
	}
	/* ★ TM->CPU final hop: does the frame get into the TM BM, drain OUT to the QM
	 * (tx), or get dropped (esp. NOBUF = no CPU buffer)?  Then the QM per-VoQ
	 * non-empty status: a VoQ bit set with wptr 0 = the frame reached the CPU VoQ and
	 * the drain to CPU-EPP is the break; no VoQ bit = dest routing is the break. */
	seq_printf(m,
		   "tm: rx=%u tx=%u drop{nobuf=%u rx=%u te=%u err=%u hdr=%u}\n",
		   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_PCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_TX_PCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_NOBUF_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_TE_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_ERR_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_HDR_DPCNT));
	seq_printf(m,
		   "voq_status: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(0)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(1)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(2)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(3)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(4)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(5)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(6)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(7)));
	seq_printf(m,
		   "dest-maps: tm_to_cpuq(2118)=0x%08x upper_ldpid(68f8)=0x%08x destp8_eq(6168)=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_L2TM_TM_TO_CPUQ_MAP),
		   readl(ni_base(ni) + CA_NI_QM_UPPER_LDPID_MAP),
		   readl(ni_base(ni) +
			 CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)));
	/* ★ ingress-baseline check: glb_static(a01c) port_to_cpu nibble MUST be the
	 * boot-ROM value (NOT 0) - a stale port_to_cpu=0 latch diverted port-0 off the
	 * L2FE and PERSISTED across warm reboots; only a cold boot clears it. */
	{
		u32 glb = readl(ni_base(ni) + CA_NI_NI_GLB_STATIC_CFG);

		seq_printf(m, "fwd: glb_static(a01c)=0x%08x port_to_cpu_nib=0x%x\n",
			   glb, (unsigned int)(glb & CA_NI_NI_PORT_TO_CPU));
	}
	/* ★ CPU-forwarding chain: the DFT_FWD[port-0] redir entry (indirect read) +
	 * PDPID_MAP[CPU_0] resolution.  Expect dft_fwd = 0x1820 (redir_en=1,
	 * mc_group_id=0x10=CPU_0) and pdpid=0x9 (CPU).  If the frame reaches TM but
	 * qm_rx_cntr stays 0, the death is between the redir resolution and the QM. */
	rx_dump_fwd_chain(m, ni, l3fe_rx, l3qm_rx);
	/* full DFT_FWD[0..15] + MC_FIB[0x10..0x1b] dump so the routing tables can be
	 * VERIFIED without devmem.  DFT_FWD read = addr(lspid<<2|type=0); MC_FIB read =
	 * indirect ACCESS[idx] then DATA0..2. */
	rx_dump_dft_fwd_and_rmu(m, ni);
	/* ★ build71: L3-CLS KEY[0..15] + FIB[0..15] readback (the ARP-trap tables) so the
	 * coordinator can diff our install vs the stock golden.  Read-only indirect. */
	rx_dump_cls_keys_and_fib(m, ni);
	/* ★ per-port profile readback (the blackhole root-cause tables): expect ilpb[0]
	 * d2=0x18022163 (stp=3) d1=0x800001cb d0=0xc1000000 d3=0x00100003,
	 * mmshp[0]=ffffffff_fffffffe, elpb[0]=0x3, ple_ctl=0x27b.  stp=0 or mmshp=0 =
	 * the force-drop state is back. */
	{
		u32 pd0, pd1, pd2, pd3, mh, ml, el;

		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_ILPB_ACCESS, CA_NI_RX_PORT);
		pd3 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA3);
		pd2 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA2);
		pd1 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA1);
		pd0 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA0);
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_MMSHP_ACCESS, CA_NI_RX_PORT);
		mh = readl(ni_base(ni) + CA_NI_L2FE_MMSHP_DATA1);
		ml = readl(ni_base(ni) + CA_NI_L2FE_MMSHP_DATA0);
		el = cortina_ni_rx_ind_entry(ni, CA_NI_L2FE_ELPB_ACCESS,
						     CA_NI_RX_PORT, CA_NI_L2FE_ELPB_DATA0);

		seq_printf(m,
			   "port-prof: ilpb[0]={d3=%08x d2=%08x d1=%08x d0=%08x} stp=%lu mmshp[0]=%08x_%08x elpb[0]=0x%02x ple_ctl=0x%08x\n",
			   pd3, pd2, pd1, pd0,
			   FIELD_GET(CA_NI_L2FE_ILPB_STP_MODE, pd2),
			   mh, ml, el,
			   readl(ni_base(ni) + CA_NI_L2FE_PLE_CTL));
	}
	/* ★ ARB deep-queue diagnostics (2026-07-15): PORT_DBUF[0] want 0 = stock (a
	 * dbuf_flg mark = the deep-queue regression is back); BM word0 bit30 = deep_q on
	 * the last frame in buffer 0 (want 0). */
	rx_dump_l2fe_arbitration(m, ni);
	/* ★ LAST-FRAME resolution: the BM latches the last RX FE (L2FE-resolved) header,
	 * the raw RX-NI and the dequeued TX-NI header.  This shows what a REAL ingress
	 * frame resolved to (deep_q b30 / cpu b31 and the resolved ldpid), separating
	 * "redir didn't fire" from "resolved to DeepQ0/PDPID8 but the enqueue is gated". */
	{
		u32 fe_lo = readl(ni_base(ni) + CA_NI_L2TM_BM_RX_FE_HDR_LO);
		u32 fe_hi = readl(ni_base(ni) + CA_NI_L2TM_BM_RX_FE_HDR_HI);

		/* HEADER_A: low word (0x2170) = {cos[2:0], ldpid[8:3], lspid[14:9],
		 * pkt_size[28:15], ...}; high word (0x2174) = pkt_info, deep_q=bit30,
		 * cpu_flg=bit31.  A resolved ldpid in 0x0..0x6 = DeepQ (-> QM). */
		seq_printf(m,
			   "bm-hdr: rx_fe=%08x_%08x (ldpid=0x%lx lspid=0x%lx deep_q=%u cpu=%u) rx_ni=%08x_%08x tx_ni=%08x_%08x bm_sts=0x%08x\n",
			   fe_hi, fe_lo,
			   FIELD_GET(GENMASK(8, 3), fe_lo),
			   FIELD_GET(GENMASK(14, 9), fe_lo),
			   !!(fe_hi & BIT(30)), !!(fe_hi & BIT(31)),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_NI_HDR_HI),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_NI_HDR_LO),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_TX_NI_HDR_HI),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_TX_NI_HDR_LO),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_STS));
	}
	/* ★ 2026-07-15: L2FE post-parse HEADER_A of the LAST parsed ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 189. */
	{
		u32 hi = readl(ni_base(ni) + CA_NI_L2FE_PP_HEADER_A_HI);
		u32 mid = readl(ni_base(ni) + CA_NI_L2FE_PP_HEADER_A_MID);
		u32 low = readl(ni_base(ni) + CA_NI_L2FE_PP_HEADER_A_LOW);

		seq_printf(m,
			   "header_a(pp 0x11c4-cc): hi=%08x mid=%08x low=%08x | ldpid=0x%02x(want 0x19) lspid=0x%02x cpu_flag=%u deep_q=%u(want 0) mcgid=0x%02x drop_code=%u fe_bypass=%u pkt_size=%u\n",
			   hi, mid, low,
			   (mid >> 3) & 0x3f, (mid >> 9) & 0x3f,
			   hi >> 31, (hi >> 30) & 1, hi & 0xff,
			   (hi >> 8) & 7, (mid >> 29) & 1, (mid >> 15) & 0x3fff);
	}
	/* ★ GATE DIAGNOSTIC (all SAFE reads, GLB window). Only the ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 150. */
	rx_dump_hv_init_and_rdy(m, ni);
	/* ★ PORT CHECK: which physical GPHY carries the host's link.  Stock's
	 * only-carrier port may not be our configured port 0 - link=1 on a port != 0
	 * (with mac/l2fe counters moving only there) means our port<->GPHY mapping is the
	 * bug (stock configures all 4 LAN ports). */
	if (ni->mii) {
		int p;

		for (p = 0; p < CA_NI_GPHY_COUNT; p++) {
			int a = CA_NI_GPHY_FIRST + p;
			int bmsr;

			mdiobus_read(ni->mii, a, MII_BMSR);	/* clear latch */
			bmsr = mdiobus_read(ni->mii, a, MII_BMSR);
			seq_printf(m, "link port%d (phy%d): bmsr=0x%04x link=%d\n",
				   p, a, bmsr,
				   bmsr >= 0 ? !!(bmsr & BMSR_LSTATUS) : -1);
		}
	}

	/* self-populating pool health: inactive MUST be 0 and stay 0 (a bid that
	 * goes inactive without a refill source = a leaked buffer) */
	pa_req = readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID));
	/* "self-pop" was hardcoded and became a lie once the pool model was made
	 * selectable; print the live mode.  inactive = buffers the pool is SHORT, so 0
	 * is the healthy reading in both models. */
	seq_printf(m, "pool: %s eq%d ready=%lu inactive=%lu (want 0)\n",
		   cpu_pool_push ? "sw-owned" : "self-pop",
		   CA_NI_RX_EQ_ID,
		   (unsigned long)FIELD_GET(CA_NI_QM_PA_REQ_READY, pa_req),
		   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT, pa_req));
	seq_printf(m, "eq%d cfg0/1/2=0x%08x/0x%08x/0x%08x rmu0=0x%08x rmu_fe_drop(0x6944)=0x%08x rx_cntr(0x6900)=0x%08x\n",
		   CA_NI_RX_EQ_ID,
		   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ_ID)),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID)),
		   readl(ni_base(ni) + CA_NI_QM_RMU0_CTRL),
		   readl(ni_base(ni) + CA_NI_QM_RMU_FE_DROP),
		   readl(ni_base(ni) + CA_NI_QM_RX_CNTR));
	/* ★ EQ-profile routing. prof_sel is a 4-bit DIRECT index (see ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 190. */
	seq_printf(m, "dq-chain: destp8(0x6188)=0x%08x destp15=0x%08x prof13(0x615c)=0x%08x(want 0xed) prof12=0x%08x eq12 cfg0/1/2=0x%08x/0x%08x/0x%08x\n",
		   readl(ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(8)),
		   readl(ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ12_PROFILE)),
		   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ12_ID)),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ12_ID)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ12_ID)));
	/* ★ ALL-16-EQ active-pool map: per EQ, eq_en (cfg0 bit0) + total_buf
	 * (cfg1[29:16]).  CPU pool = EQ13/14.  The DEEP-QUEUE pool is at whichever OTHER
	 * EQs are active - NOT EQ0/1/2 (those read reset on stock). */
	{
		int e;

		seq_printf(m, "eq-map (en:total):");
		for (e = 0; e < 16; e++) {
			u32 c0 = readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(e));
			u32 c1 = readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(e));

			seq_printf(m, " eq%d=%u:%lu", e, c0 & 1,
				   (unsigned long)FIELD_GET(CA_NI_QM_CFG1_TOTAL_BUF_NUM, c1));
		}
		seq_printf(m, "\n");
	}
	seq_printf(m, "eq-prof0-7: %08x %08x %08x %08x %08x %08x %08x %08x (prof13=%08x)\n",
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(0)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(1)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(2)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(3)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(4)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(5)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(6)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(7)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)));
	/* pool1 (EQ14) PA-request + the CPU-EPP ring pointers: if eq13 ready=1
	 * and the ring wptr advances past rdptr, the delivery chain is live */
	pa_req = readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID2));
	/* ★ An old annotation here recommended "cfg2 want 0xff0d: ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 151. */
	seq_printf(m, "eq%d ready=%lu inactive=%lu cfg1=0x%08x cfg2=0x%08x (sw-owned wants 0x0000ff0c: cpu_eq=1 bufsz idx4=2048)\n",
		   CA_NI_RX_EQ_ID2,
		   (unsigned long)FIELD_GET(CA_NI_QM_PA_REQ_READY, pa_req),
		   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT, pa_req),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID2)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID2)));
	seq_printf(m, "l2tm-es: es_ctrl=0x%08x (tx_en=%u p8_L3QM=%u) sch8=0x%08x (voq_en=0x%02lx) bm_dq_map=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL),
		   !!(readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL) & CA_NI_L2TM_ES_TX_EN),
		   !!(readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL) & BIT(CA_NI_L2TM_ES_PORT_L3QM)),
		   readl(ni_base(ni) + CA_NI_L2TM_ES_SCH_CFG(CA_NI_L2TM_ES_PORT_L3QM)),
		   (unsigned long)(readl(ni_base(ni) + CA_NI_L2TM_ES_SCH_CFG(CA_NI_L2TM_ES_PORT_L3QM)) & CA_NI_L2TM_ES_VOQ_EN_ALL),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_DQ_PORT_MAP));
	seq_printf(m, "ni_qm_hol: es_ctrl2_real(0x6a30)=0x%08x (bit1=%u want 1) rmu0_ctrl=0x%08x qm_es_ctrl=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_QM_ES_CTRL2_REAL),
		   !!(readl(ni_base(ni) + CA_NI_QM_ES_CTRL2_REAL) & CA_NI_QM_ES_CTRL2_NI_QM_HOL),
		   readl(ni_base(ni) + CA_NI_QM_RMU0_CTRL),
		   readl(ni_base(ni) + CA_NI_QM_ES_CTRL));
	seq_printf(m, "epp ring: wptr=0x%06x rdptr_sw=0x%x cmd_mode=%s es_ctrl2=0x%08x demux_sel=0x%x\n",
		   cortina_ni_rx_wptr(ni), rx->rptr[0],
		   (readl(ni_base(ni) + CA_NI_QM_EPP) & CA_NI_QM_EPP_CMD_MODE_64) ?
		   "64b" : "32b",
		   readl(ni_base(ni) + CA_NI_QM_ES_CTRL2),
		   (unsigned int)(readl(ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG) &
				  CA_NI_NI_L3QMRX_DEMUX_SEL_ALL));
	/* ★★ THE DECISIVE TEST: dump the CPU-virtual EPP ring DRAM (0x0bc48000).  After
	 * arping, non-zero descriptor slots => the HW writeback LANDS (the gap is
	 * wptr/detection); all-zero => the writeback FAILS (0x611c bit22 error is real). */
	if (rx->ring) {
		u64 s0 = le64_to_cpu(rx->ring[0]), s1 = le64_to_cpu(rx->ring[1]);
		u64 s2 = le64_to_cpu(rx->ring[2]), s3 = le64_to_cpu(rx->ring[3]);
		unsigned int v;

		seq_printf(m, "epp-ring voq0: slot0=%08x_%08x slot1=%08x_%08x slot2=%08x_%08x slot3=%08x_%08x\n",
			   upper_32_bits(s0), lower_32_bits(s0), upper_32_bits(s1), lower_32_bits(s1),
			   upper_32_bits(s2), lower_32_bits(s2), upper_32_bits(s3), lower_32_bits(s3));
		seq_puts(m, "epp-ring voq0..7 slot0:");
		for (v = 0; v < CA_NI_RX_VOQ_COUNT; v++) {
			u64 sv = le64_to_cpu(rx->ring[v * CA_NI_RX_RING_SLOTS_PER_VOQ]);

			seq_printf(m, " v%u=%08x_%08x", v, upper_32_bits(sv), lower_32_bits(sv));
		}
		seq_puts(m, "\n");

		/* ★ DECISIVE: dump BOTH per-voq rings' voq0 first 16 entries - LOW =
		 * PADDR(0x7200)=0x0bc48000 (where NAPI reads) vs HIGH =
		 * PADDR_HI(0x7220)=0x0bc4a000.  wptr advanced 38 while NAPI's LOW read
		 * poison, so the engine's real descriptors are in whichever is NOT deadbeef. */
		{
			unsigned int hi = CA_NI_RX_RING_HI_OFFSET / sizeof(__le64);
			unsigned int k;

			seq_puts(m, "build88 LOW(0x0bc48000) voq0:");
			for (k = 0; k < 16; k++) {
				u64 d = le64_to_cpu(rx->ring[k]);

				seq_printf(m, " %08x_%08x", upper_32_bits(d), lower_32_bits(d));
			}
			seq_puts(m, "\nbuild88 HIGH(0x0bc4a000) voq0:");
			for (k = 0; k < 16; k++) {
				u64 d = le64_to_cpu(rx->ring[hi + k]);

				seq_printf(m, " %08x_%08x", upper_32_bits(d), lower_32_bits(d));
			}
			seq_puts(m, "  (deadbeef=poison; real desc = a 0x094xxxxx PA + small len; rx_ring_hi param = which ring NAPI reads)\n");
		}
	}
	/* ★★ THE CLEAR-ON-READ NI_HV COUNTERS ARE NOT READ HERE ANY ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 152. */
	seq_printf(m, "datapath: bm_rx(0x213c)=%u bm_tx(0x2140)=%u | ni2qm_rx(0xa9fc)=%llu miss_sop_eop(0xa9f4)=0x%08x short_err(0xa9f8)=0x%08x ni2qm_tx(0xaa10)=%llu | rmu_rx(0x6900)=%u rmu_sched(0x690c)=%u | epp_wptr(0x7000)=0x%06x | drop no_buf(0x6940)=%u fe(0x6944)=%u\n",
		   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_PCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_TX_PCNT),
		   l3qm_rx,
		   readl(ni_base(ni) + CA_NI_NI_L3QM_RX_MISS_SOP_EOP),
		   readl(ni_base(ni) + CA_NI_NI_L3QM_RX_SHORT_ERR),
		   nihv[CA_NI_NIHV_L3QM_TX],
		   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
		   readl(ni_base(ni) + CA_NI_QM_TX_CNTR),
		   cortina_ni_rx_wptr(ni),
		   readl(ni_base(ni) + CA_NI_QM_RMU_NO_BUF_DROP),
		   readl(ni_base(ni) + CA_NI_QM_RMU_FE_DROP));
	/* ★ BM-drop ledger: the FIRST drop that climbs +N under an N-ARP flood tells
	 * WHERE a deep_q frame dies before L3QM.  te=threshold-engine, sb=shared-buffer
	 * full, nobuf=no free buffer, hdr=header, err=error, rx=enqueue drop.  All 0 +
	 * bm_tx climbing = egressed-but-lost downstream. */
	seq_printf(m, "bm-drops: te(0x214c)=%u sb(0x2144)=%u nobuf(0x216c)=%u hdr(0x2148)=%u err(0x2150)=%u rx(0x2164)=%u\n",
		   readl(ni_base(ni) + CA_NI_L2TM_BM_TE_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_SB_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_NOBUF_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_HDR_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_ERR_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_DPCNT));
	/* ★ WHICH NI_HV interface does the dequeue land on? (all 4 RX_PKT_CNT, stride
	 * 0x40, all from the ONE sample taken at the top).  bm_tx +9 but l3qm(0xa9fc)=0
	 * -> if l3fe(0xa9bc) climbs the frame goes to the L3FE interface we don't drain;
	 * if ALL 4 stay 0 the BM->NI_HV handoff is the gate. */
	seq_printf(m, "ni_hv-rx: l3fe(0xa9bc)=%llu l3qm(0xa9fc)=%llu mce(0xaa3c)=%llu dma(0xaa7c)=%llu [read-and-clear registers, accumulated: totals since boot]\n",
		   l3fe_rx,
		   l3qm_rx,
		   nihv[CA_NI_NIHV_MCE_RX],
		   nihv[CA_NI_NIHV_DMA_RX]);
	/* ★★ The QM/RMU0-side bisect: no_buf climbing while rmu_rx=0 = the frame reached
	 * RMU0 but EQ13 was empty = seed EQ13.  RE offsets are shown next to our Elnath
	 * offsets (0x6900/0x6940) to see which are live. */
	seq_printf(m, "qm-rmu: rmu_rx[RE0x67d8]=%u [ours0x6900]=%u | no_buf[RE0x6818]=%u [ours0x6940]=%u | eq13_usg(0x695c)=0x%08x cpu_push_rdy(0x6368)=0x%08x eq_unfill(0x63c0)=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_QM_RMU0_RX_PKT_CNTR_RE),
		   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
		   readl(ni_base(ni) + CA_NI_QM_RMU0_NO_BUF_DROP_RE),
		   readl(ni_base(ni) + CA_NI_QM_RMU_NO_BUF_DROP),
		   readl(ni_base(ni) + CA_NI_QM_EQ13_BUF_USG),
		   readl(ni_base(ni) + CA_NI_QM_CPU_PUSH_RDY0_RE),
		   readl(ni_base(ni) + CA_NI_QM_EQ_STACK_UNFILL));
	/* ★ NI_HV interconnect region (L2TM-egress -> L3QM-ingress demux/source-select),
	 * the proven death stage.  0xa180-0xa1c4 vs the stock golden; 0xa1c0=0x76543210
	 * was the one we never wrote. */
	{
		static const u32 nihv_want[] = {
			0x00a87f00, 0x0024009b, 0x00000000, 0xffff7f7f,	/* a180 a184 a188 a18c */
			0x040c2040, 0x00007185, 0x00000000, 0x00000002,	/* a190 a194 a198 a19c */
			0x22aa0000, 0x00000000, 0x00000000, 0x00000000,	/* a1a0 a1a4 a1a8 a1ac */
			0x22aa0000, 0xaaaa0000, 0x00086ffc, 0x00003e80,	/* a1b0 a1b4 a1b8 a1bc */
			0x76543210,					/* a1c0 */
		};
		unsigned int j;

		seq_puts(m, "ni_hv (0xa180-0xa1c0, L2TM->L3QM demux; ! = differs from stock):\n");
		for (j = 0; j < ARRAY_SIZE(nihv_want); j++) {
			u32 off = 0xa180 + j * 4;
			u32 v = readl(ni_base(ni) + off);

			seq_printf(m, "  0x%04x=0x%08x want 0x%08x %s\n",
				   off, v, nihv_want[j],
				   v == nihv_want[j] ? "" : "  !MISMATCH");
		}
	}
	/* ★ per-voq CPU-EPP wptrs + RMU0 admission/drop bisect: a moving voq wptr means
	 * the RMU pushed to it; 0x6900 at 0 with no_buf/fe_drop climbing means the frame
	 * reaches RMU0 with no buffer; both 0 = the frame never arrives. */
	{
		unsigned int q;

		seq_printf(m, "epp voq-wptr:");
		for (q = 0; q < CA_NI_RX_VOQ_COUNT; q++)
			seq_printf(m, " q%u=0x%03x", q,
				   cortina_ni_rx_wptr_voq(ni, q));
		seq_printf(m, " | rmu0_rx=%u no_buf_drop(0x6940)=%u fe_drop(0x6944)=%u\n",
			   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
			   readl(ni_base(ni) + CA_NI_QM_RMU0_NO_BUF_DROP),
			   readl(ni_base(ni) + CA_NI_QM_RMU0_FE_DROP));
	}
	/* ★ CB occupancy A/B bisect: scan VOQ buf-cnt 0..63, print non-zero (frame IN the
	 * central buffer = scanner/drain gap; all 0 while stock climbs = deep_q never
	 * enqueued into the CB). */
	{
		unsigned int q, nz = 0;

		seq_printf(m, "cb-occupancy voq_bufcnt(nonzero, idx 0..127):");
		for (q = 0; q < 128; q++) {
			u32 c;

			c = cortina_ni_rx_ind_entry(ni, CA_NI_L2TM_CB_VOQ_BUFCNT_ACCESS,
							     q, CA_NI_L2TM_CB_VOQ_BUFCNT_DATA);
			if (c) {
				seq_printf(m, " q%u=%u", q, c);
				nz++;
			}
		}
		if (!nz)
			seq_printf(m, " NONE(all 0)");
		seq_printf(m, "\n");
	}
	/* ★ deep-queue populate check: EQ12 pa_req (bit31=req active, like stock's
	 * 0x80000000) + the CB per-port free-buf-cnt we seeded (ports 0/8). */
	{
		u32 f0, f8;

		f0 = cortina_ni_rx_ind_entry(ni, CA_NI_L2TM_CB_PORT_FREECNT_ACCESS,
						     0, CA_NI_L2TM_CB_PORT_FREECNT_DATA);
		f8 = cortina_ni_rx_ind_entry(ni, CA_NI_L2TM_CB_PORT_FREECNT_ACCESS,
						     8, CA_NI_L2TM_CB_PORT_FREECNT_DATA);
		seq_printf(m, "dq-populate: eq12_pa_req(0x63d8)=0x%08x (req=%u want 1) cb_freebuf[p0]=0x%08x [p8]=0x%08x\n",
			   readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ12_ID)),
			   !!(readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ12_ID)) & CA_NI_QM_PA_REQ_READY),
			   f0, f8);
	}
	/* ★ QUEUE-DEPTH READBACK - which setting was actually in force.  Prints the
	 * parameters AND a live indirect read of the first and last profile entry of both
	 * per-VoQ threshold tables: a benchmark whose configuration cannot be read back
	 * afterwards is not evidence. */
	{
		u32 dq0, cb0_1, cb0_0, dq7, cb7_1, cb7_0;

		cortina_ni_rx_deepq_thrsh_read(ni, 0, &dq0, &cb0_1, &cb0_0);
		cortina_ni_rx_deepq_thrsh_read(ni,
					       CA_NI_L2TM_DEEPQ_VOQ_ENTRIES - 1,
					       &dq7, &cb7_1, &cb7_0);
		seq_printf(m,
			   "deepq-thrsh: param{deepq_voq_thrsh=0x%08x deepq_cb_stock=%d} hw_dqsch[0]=0x%08x hw_dqsch[%u]=0x%08x hw_cb[0]={0x%08x,0x%08x} hw_cb[%u]={0x%08x,0x%08x} acc{dqsch(0x2e70)=0x%08x cb(0x2da0)=0x%08x}\n",
			   READ_ONCE(deepq_voq_thrsh), READ_ONCE(deepq_cb_stock),
			   dq0, CA_NI_L2TM_DEEPQ_VOQ_ENTRIES - 1, dq7,
			   cb0_1, cb0_0, CA_NI_L2TM_DEEPQ_VOQ_ENTRIES - 1,
			   cb7_1, cb7_0,
			   readl(ni_base(ni) + CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS),
			   readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_ACCESS));
		seq_puts(m,
			 "deepq-thrsh: SCOPE = the deep-queue path only: CPU-RX always; HW-offloaded US only when the live data T-CONT <= 7 (see live_pon{tcont=} in /proc/cortina_l3fe); HW-offloaded DS only with hw_ds_deepq=1 (default off); CPU-forwarded US (pon_data_enq) never\n");
		seq_printf(m,
			   "deepq-thrsh: dqsch stock=0x%08x permissive=0x%08x (ours ~292x deeper); cb stock={0x%08x,0x%08x} permissive=0x%08x (stock is DEEPER here); direct regs 0x2d80/88/8c/90/94/98 + 0x2ec8..0x2ee8 are tier-1 stock golden and are NOT part of this knob\n",
			   CA_NI_L2TM_DQSCH_VOQ_THRSH_VAL,
			   CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE,
			   CA_NI_L2TM_CB_VOQ_THRSH_D1, CA_NI_L2TM_CB_VOQ_THRSH_D0,
			   CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE);
	}
	cortina_ni_cpu_fwd_show(m, ni);
	seq_printf(m, "destport0: eq_cfg=0x%08x pkt_buf=0x%08x profile%d=0x%08x voq_en=0x%08x\n",
		   readl(ni_base(ni) +
			 CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)),
		   readl(ni_base(ni) +
			 CA_NI_QM_DEST_PORT_PKT_BUF_CFG(CA_NI_RX_CPU_PORT)),
		   CA_NI_RX_EQ_PROFILE,
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_EN(CA_NI_RX_CPU_PORT)));
	/* ★ The CPU_0 pools = EQ5(pool0)/EQ6(pool1) (RE of init_empty_buffer_CPU).
	 * Confirm cfg2 has cpu_eq=1+bufsz, ibid (inactive_bid) dropping as buffers push,
	 * EQ_PROFILE[2]={eqp0=5,eqp1=6}=0x65, destport0 profile_sel=2. */
	seq_printf(m,
		   "build81 EQ5{cfg0=0x%08x cfg1=0x%08x cfg2=0x%08x pa_req(0x72f8)=0x%08x} EQ6{cfg0=0x%08x cfg1=0x%08x cfg2=0x%08x pa_req(0x72fc)=0x%08x} prof[2]=0x%08x(want 0x65) destp0=0x%08x(psel 2) [want pa_req req(bit31)=0]\n",
		   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(5)),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(5)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(5)),
		   readl(ni_base(ni) + CA_NI_QM_EQM_INACTIVE_BID(5)),
		   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(6)),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(6)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(6)),
		   readl(ni_base(ni) + CA_NI_QM_EQM_INACTIVE_BID(6)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(2)),
		   readl(ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(0)));
	/* GPHY wedge spy: fault != 0 on a zero-RX boot = the root cause the
	 * 1 Hz recovery is there to heal; recoveries counts reinits fired */
	seq_printf(m, "gphy: fault=0x%04x last=0x%04x recoveries=%llu rearms=%llu\n",
		   cortina_ni_rx_gphy_fault(ni), rx->last_fault,
		   rx->recoveries, rx->rearms);
	/* THE WITNESS FOR THE COUNT-CAP DEFECT (2026-08-20).  `intf_done` is the single
	 * bit that says whether any RJ45 can ingress at all, and the two bring-up
	 * counters say whether the recovery is still trying and at what cadence.  Without
	 * these a board whose LAN is dead reads exactly like a board with no cable. */
	seq_printf(m, "bringup: intf_done=%d ticks=%llu calls=%llu period=%us\n",
		   rx->intf_done ? 1 : 0, rx->bringup_ticks, rx->bringup_calls,
		   cortina_ni_rx_bringup_period(rx->bringup_ticks));
	seq_printf(m, "frames=%llu bytes=%llu polls=%llu swid=%llu pon=%llu wan=%llu wan_l3=%llu\n",
		   rx->frames, rx->bytes, rx->polls, rx->swid_frames,
		   rx->pon_frames, rx->wan_frames, rx->wan_l3_frames);
	/* packet-order spy: one flow must stay on ONE voq (>=2 climbing under a
	 * unidirectional bench = HW spreads the flow, drain order can reorder) */
	seq_puts(m, "voq_frames:");
	for (i = 0; i < CA_NI_RX_VOQ_COUNT; i++)
		seq_printf(m, " %d:%llu", i, rx->voq_frames[i]);
	seq_puts(m, "\n");
	seq_printf(m, "drops: nosop=%llu badpa=%llu len=%llu (runt=%llu oversize=%llu) nobuf=%llu dead=%llu\n",
		   rx->drop_nosop, rx->drop_badpa, rx->drop_len,
		   rx->drop_runt, rx->drop_oversize,
		   rx->drop_nobuf, rx->slot_dead);
	/* Multi-buffer receive. frames>0 is the witness that the path ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 153. */
	{
		unsigned int open = 0;

		for (i = 0; i < CA_NI_RX_VOQ_COUNT; i++)
			if (rx->chain[i].st.open)
				open++;
		seq_printf(m, "chain: mode=%s rest_off=+0x%02x frames=%llu segs=%llu max_segs=%llu open_now=%u\n",
			   rx_chain ? "on" : "OFF", rx->chain_rest_off,
			   rx->chain_frames, rx->chain_segs,
			   rx->chain_max_segs, open);
		seq_printf(m, "chain-bad: abort=%llu reopen=%llu orphan=%llu badtotal=%llu toolong=%llu short=%llu swid=%llu (all want 0)\n",
			   rx->chain_abort, rx->chain_reopen, rx->chain_orphan,
			   rx->chain_badtotal, rx->chain_toolong,
			   rx->chain_short, rx->chain_swid);
		if (rx->chain_frames) {
			seq_printf(m, "chain-dlen: diff=%llu seen/calc", rx->chain_dlen_diff);
			for (i = 0; i < CA_NI_RX_CHAIN_MAX_SEGS &&
				    rx->chain_dlen_calc[i]; i++)
				seq_printf(m, " %u:%u/%u", i,
					   rx->chain_dlen_seen[i],
					   rx->chain_dlen_calc[i]);
			seq_puts(m, " (observation, not a fault)\n");
		}
	}
	/* pool ownership: stale_buf MUST be 0 - non-zero means a buffer was
	 * reused before NAPI copied it out (the fragmented-datagram defect) */
	seq_printf(m, "pool-own: mode=%s stale_buf=%llu (want 0) push_fail=%llu (want 0)\n",
		   cpu_pool_push ? "sw-owned(cpu_eq=1)" : "hw-managed(cpu_eq=0)",
		   rx->stale_buf, rx->push_fail);
	/* deep-queue pool witness: dq_frames > 0 proves EQ12 is populated and the
	 * deep-queue admission path is delivering (the GPON DS punt rides it); inactive
	 * is the shortfall gauge, 0 = the QM self-populated it. */
	{
		u32 dq_req = readl(ni_base(ni) +
				   CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ12_ID));

		seq_printf(m, "pool-dq: eq%u frames=%llu cfg0=0x%08x cfg1=0x%08x cfg2=0x%08x prof%u=0x%08x ready=%lu inactive=%lu (want inactive 0; frames>0 once DS flows)\n",
			   CA_NI_RX_EQ12_ID, rx->dq_frames,
			   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ12_ID)),
			   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ12_ID)),
			   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ12_ID)),
			   CA_NI_RX_DQ_PROFILE_SEL,
			   readl(ni_base(ni) +
				 CA_NI_QM_EQ_PROFILE(CA_NI_RX_DQ_PROFILE_SEL)),
			   (unsigned long)FIELD_GET(CA_NI_QM_PA_REQ_READY, dq_req),
			   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT,
						    dq_req));
	}
	seq_printf(m, "last_desc=%016llx last_hdra=%016llx\n",
		   rx->last_desc, rx->last_hdra);
	/* ★ TEMP DIAG (rx_crc_tap): machine-readable HW lookup-CRC witness */
	if (rx_crc_tap)
		seq_printf(m,
			   "crc_tap: hits=%llu hw_crc32=%08x hw_crc16=%04x cpu_flg=%u (TEMP DIAG - diff vs install crc)\n",
			   rx->tap_hits, rx->tap_crc32, rx->tap_crc16,
			   rx->tap_cpuflg);
	seq_puts(m, "irq_hits:");
	for (i = 0; i < CA_NI_RX_NUM_IRQS; i++)
		seq_printf(m, " %d:%llu", rx->irq[i], rx->irq_hits[i]);
	seq_puts(m, "\n");

	cortina_ni_rx_dump_regs(m, ni);

	/* ★ QM+L2TM full-block sweep (grep/diff-friendly, one per line) - the
	 * ours-vs-stock hunt for the L2TM->QM admit gate (qm_rx_cntr=0). */
	{
		unsigned int r, k;

		seq_puts(m, "qmblock:\n");
		for (r = 0; r < ARRAY_SIZE(cortina_ni_qmdump_ranges); r++)
			for (k = 0; k < cortina_ni_qmdump_ranges[r].count; k++) {
				unsigned int off =
					cortina_ni_qmdump_ranges[r].base + 4 * k;

				seq_printf(m, "  0x%04x=0x%08x\n", off,
					   readl(ni_base(ni) + off));
			}
	}

	/* ★ axi_reo (RMU DMA-reorder) window dump - a SEPARATE MMIO window (idx 10), NOT
	 * covered by the NI-core qmblock sweep.  The 3 channel blocks, so the 21-reg
	 * golden (READ 0x000 / WRITE 0x400 / WRITE2 0x480) is diff-able vs stock. */
	{
		void __iomem *reo = ni->win[CA_NI_WIN_AXI_REO];
		unsigned int k;

		seq_puts(m, "axi_reo (window idx10, g_ne_axi_reo):\n");
		if (!reo) {
			seq_puts(m, "  <window not mapped>\n");
		} else {
			for (k = 0; k < ARRAY_SIZE(cortina_ni_axi_reo_cfg); k++)
				seq_printf(m, "  0x%04x=0x%08x (want 0x%08x)\n",
					   cortina_ni_axi_reo_cfg[k].off,
					   readl(reo + cortina_ni_axi_reo_cfg[k].off),
					   cortina_ni_axi_reo_cfg[k].val);
		}
	}

	/* ★ FBM window dump (GLB/AXI/POOL) - the RMU buffer-allocator, separate windows
	 * (idx 18/19/21).  glb0 low byte = pool-enable (want 0xFF), pool0+0x10 = refill
	 * (want 0 = OFF).  Not covered by the NI-core qmblock sweep. */
	{
		void __iomem *fbm_glb  = ni->win[CA_NI_WIN_FBM_GLB];
		void __iomem *axi  = ni->win[CA_NI_WIN_FBM_AXI];
		void __iomem *pool = ni->win[CA_NI_WIN_FBM_POOL];

		seq_puts(m, "fbm (windows idx18/19/21):\n");
		if (!fbm_glb || !axi || !pool) {
			seq_printf(m, "  <unmapped fbm_glb=%d axi=%d pool=%d>\n",
				   !!fbm_glb, !!axi, !!pool);
		} else {
			seq_printf(m, "  fbm_glb 0x00=0x%08x 0x04=0x%08x 0x0c=0x%08x 0x10=0x%08x 0x70=0x%08x\n",
				   readl(fbm_glb + CA_NI_QM_FBM_GLB_POOL_EN), readl(fbm_glb + CA_NI_QM_FBM_GLB_MODE),
				   readl(fbm_glb + 0x0c), readl(fbm_glb + 0x10),
				   readl(fbm_glb + CA_NI_QM_FBM_GLB_ECC));
			seq_printf(m, "  axi 0x00=0x%08x (want 0x200)\n",
				   readl(axi + 0x00));
			seq_printf(m, "  pool0 cfg0x00=0x%08x exstack0x04=0x%08x depth0x08=0x%08x cnt0x0c=0x%08x outstanding0x2c=0x%08x\n",
				   readl(pool + CA_NI_QM_FBM_POOL_CFG0),
				   readl(pool + CA_NI_QM_FBM_POOL_EXSTACK),
				   readl(pool + CA_NI_QM_FBM_POOL_DEPTH),
				   readl(pool + CA_NI_QM_FBM_POOL_COUNT),
				   readl(pool + CA_NI_QM_FBM_POOL_OUTSTND));
		}
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* probe                                                               */
/* ------------------------------------------------------------------ */

static int cortina_ni_rx_irqs_init(struct cortina_ni *ni)
{
	struct platform_device *pdev = to_platform_device(ni->dev);
	struct cortina_ni_rx *rx = ni->rx;
	int i, irq, ret, got = 0;

	/* DT lists the 8 per-cpu-port EPP interrupts first (SPI ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 154. */
	for (i = 0; i < CA_NI_RX_NUM_IRQS; i++) {
		rx->irq[i] = -1;
		rx->irqctx[i].ni = ni;
		rx->irqctx[i].idx = i;

		irq = platform_get_irq_optional(pdev, i);
		if (irq < 0)
			continue;

		ret = devm_request_irq(ni->dev, irq, cortina_ni_rx_isr, 0,
				       devm_kasprintf(ni->dev, GFP_KERNEL,
						      "%s-rx%d",
						      dev_name(ni->dev), i),
				       &rx->irqctx[i]);
		if (ret) {
			dev_warn(ni->dev, "cannot request RX irq %d (#%d)\n",
				 irq, i);
			continue;
		}
		rx->irq[i] = irq;
		got++;
	}

	if (rx->irq[0] < 0) {
		dev_err(ni->dev, "RX interrupt 0 (SPI 0x54) unavailable\n");
		/* ⚠ GIVE BACK WHAT THIS FUNCTION TOOK, HERE. Interrupts 1..7 ...
		 * dev/MEASURED-cortina-ni-rx.c.md sec 155. */
		for (i = 0; i < CA_NI_RX_NUM_IRQS; i++) {
			if (rx->irq[i] < 0)
				continue;
			devm_free_irq(ni->dev, rx->irq[i], &rx->irqctx[i]);
			rx->irq[i] = -1;
		}
		return -ENXIO;
	}
	dev_info(ni->dev, "RX: %d EPP interrupts requested (irq0=%d)\n",
		 got, rx->irq[0]);
	return 0;
}

int cortina_ni_rx_probe(struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx;
	int ret;

	if (!ni->tx || !ni->tx->netdev)
		return dev_err_probe(ni->dev, -ENODEV,
				     "RX needs the TX netdev first\n");

	rx = devm_kzalloc(ni->dev, sizeof(*rx), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;
	rx->ni = ni;
	rx->netdev = ni->tx->netdev;
	INIT_DELAYED_WORK(&rx->recovery_work, cortina_ni_rx_recovery_work);
	/* where a chain continuation's payload starts.  Resolved once, at probe,
	 * so the receive path reads a field instead of a module parameter. */
	rx->chain_rest_off = CA_NI_RX_HDRA_OFF +
			     (rx_chain_rest_hdra ? CA_NI_RX_HDR_CPU_LEN : 0);
	ni->rx = rx;
	if (rx_chain)
		dev_info(ni->dev,
			 "RX: multi-buffer receive ON (chain payload window %u B/buffer, continuation at +0x%02x, max %u segs, max %u B)\n",
			 CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL0_BUFSZ) -
			 CA_NI_RX_BUF_HEADROOM, rx->chain_rest_off,
			 CA_NI_RX_CHAIN_MAX_SEGS, CA_NI_RX_CHAIN_MAX_LEN);

	/* snapshot the GPHY calibration while it is in the U-Boot-proven
	 * state, and log the fault latch so a wedge already present at
	 * probe is visible in the boot log */
	cortina_ni_rx_gphy_cal_save(ni);
	dev_info(ni->dev, "RX: GPHY port %d fault latch 0x%04x at probe\n",
		 CA_NI_RX_PORT, cortina_ni_rx_gphy_fault(ni));

	/* CPU-EPP descriptor ring: STOCK puts it at a FIXED phys in ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 156. */
	rx->ring_dma = CA_NI_RX_RING_PHYS;
	/* ★★★ Map the CPU-EPP ring UNCACHED (MEMREMAP_WC = Normal-Non-Cacheable on
	 * ARM64).  The NE DMA is NON-coherent (proven on HW: NAPI read the ring's stale
	 * CACHED poison 0xdeadbeef instead of the HW-DMA'd descriptor -> rx_errs, "PA
	 * outside pool"); the earlier MEMREMAP_WB + `dma-coherent` assumption was WRONG. */
	rx->ring = devm_memremap(ni->dev, CA_NI_RX_RING_PHYS,
				 CA_NI_RX_RING_TOTAL_BYTES, MEMREMAP_WC);
	if (IS_ERR_OR_NULL(rx->ring)) {
		dev_err(ni->dev, "RX ring memremap(%pa) failed\n",
			&rx->ring_dma);
		ni->rx = NULL;
		return -ENOMEM;
	}
	/* ★★ SEED 0xDEADBEEF into the whole ring BEFORE arming EPP/RMU0 (stock
	 * aal_l3qm_insert_magic_number).  The HW writeback engine may refuse to write a
	 * slot that does not already hold the sentinel; our ring was un-seeded poison.
	 * Fill every u32 word (a superset of stock's tail guard-band). */
	{
		u32 *r = (u32 *)rx->ring;
		unsigned int n;

		for (n = 0; n < CA_NI_RX_RING_TOTAL_BYTES / sizeof(u32); n++)
			r[n] = 0xDEADBEEFu;
		dma_wmb();
		dev_info(ni->dev, "RX ring: seeded 0x%x u32 = 0xDEADBEEF (uncached WC) @0x%08x\n",
			 (unsigned int)(CA_NI_RX_RING_TOTAL_BYTES / sizeof(u32)),
			 (u32)CA_NI_RX_RING_PHYS);
	}

	/* ★★★ CPU-pool buffer region: like the ring, it MUST live at ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 157. */
	rx->cpu_dram_dma = CA_NI_RX_CPU_POOL_PHYS;
	rx->cpu_dram = devm_memremap(ni->dev, CA_NI_RX_CPU_POOL_PHYS,
				     CA_NI_RX_MAP_SIZE, MEMREMAP_WC);
	if (IS_ERR_OR_NULL(rx->cpu_dram)) {
		dev_err(ni->dev, "RX: CPU-pool memremap(%pa) failed\n",
			&rx->cpu_dram_dma);
		ni->rx = NULL;
		return -ENOMEM;
	}
	dev_info(ni->dev,
		 "RX: CPU-pool DRAM @%pad size %u (reserved-window; %u CPU pools + %u deep-queue @0x%08x)\n",
		 &rx->cpu_dram_dma, CA_NI_RX_MAP_SIZE, CA_NI_RX_CPU_DRAM_SIZE,
		 CA_NI_RX_DQ_DRAM_SIZE, (u32)CA_NI_RX_DQ_POOL_PHYS);

	/* U-Boot TFTP'd through port 0 and may have left the MAC RX on;
	 * force it off so nothing feeds the ring before ndo_open */
	ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT),
	       CA_NI_PORT_RXMAC_RX_EN, 0);

	ret = cortina_ni_rx_eq_init(ni);
	if (ret) {
		ni->rx = NULL;
		return ret;
	}
	cortina_ni_rx_epp_init(ni);
	dev_info(ni->dev, "RX: EQ pool + EPP ring init done\n");

	/* ★ FBM bring-up (the RMU buffer-allocator, must be up BEFORE ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 158. */
	if (fbm_enable) {
		cortina_ni_rx_fbm_init(ni);	/* reset + GLB/AXI/POOL config + exstack base */
		cortina_ni_rx_fbm_fill(ni);	/* FBM_CPU gated doorbell push */
	} else {
		dev_info(ni->dev, "fbm: DISABLED (safe baseline) - set cortina_ni_rx.fbm_enable=1 to bring up\n");
	}

	/* Enable the L3QM egress-scheduler master BEFORE the RX master, exactly as stock
	 * ca_ni_init_l3qm orders aal_l3qm_enable_tx(1) then aal_l3qm_enable_rx(1).  This
	 * is the drain gate that lets an enqueued descriptor reach the CPU-EPP ring; the
	 * per-CPU-port cpu_en stays off until open, so nothing is delivered yet. */
	cortina_ni_rx_es_enable(ni);

	/* master L3QM RX on - stock ca_ni_init_l3qm ends with aal_l3qm_enable_rx(1).  The
	 * engine must be live before pushing buffers so the pushed PAs leave the shallow
	 * push stage into the committed bid pool.  Ingress cannot flow yet: the port-0
	 * MAC RX stays off until open. */
	ni_rmw(ni, CA_NI_QM_RMU0_CTRL, 0, CA_NI_QM_RMU0_RX_EN);
	cortina_ni_rx_eqm_readback(ni, "after RMU0 enable, pre-populate");

	/* ★ Software-owned pools: stage their buffers now - the push stage only drains
	 * once the EQ config is committed and RMU0 runs, both of which have just
	 * happened, and the port-0 MAC RX is still off so nothing can consume a buffer
	 * before the pools are full.  A failure here is fatal to RX. */
	if (cpu_pool_push) {
		ret = cortina_ni_rx_push_seed(ni);
		if (ret) {
			ni->rx = NULL;
			return ret;
		}
	}

	/* ★★ RMU AXI reorder engine (stock runs this right after enable_rx) - the
	 * separate g_ne_axi_reo MMIO block; without it the RMU dequeue DMA never
	 * completes and no CPU frame is admitted. */
	cortina_ni_rx_axi_reo_init(ni);

	/* the narrative dump is published from ... -- dev/MEASURED-cortina-ni-rx.c.md sec 159. */
	dev_info(ni->dev,
		 "RX pool self-populated (%u+%u DRAM bufs @0x%08x): eq5_pa_req=0x%08x eq6_pa_req=0x%08x wptr=0x%06x (want pa_req 0)\n",
		 CA_NI_RX_EQ_TOTAL_BUF, CA_NI_RX_EQ2_TOTAL_BUF,
		 (u32)CA_NI_RX_CPU_POOL_PHYS,
		 readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID)),
		 readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID2)),
		 cortina_ni_rx_wptr(ni));
	cortina_ni_rx_eqm_readback(ni, "self-populating pools (pa_req should stay 0)");

	/* ★★ W1C-clear the LATCHED eqm_cfg_error at QM_INT_SRC 0x611c ONLY (write bit22).
	 * Do NOT write 0x6120 - that is the INT_SRCE ENABLE MASK (0xe6d54f85), not a
	 * status latch, and clobbering it to 0x00400000 disabled most int sources. */
	writel(BIT(22), ni_base(ni) + CA_NI_QM_INT_SRC);
	dev_info(ni->dev, "eqm_cfg_error W1C: int_src 0x611c=0x%08x en_mask 0x6120=0x%08x (want 0x611c bit22 CLEAR, 0x6120=0xe6d54f85)\n",
		 readl(ni_base(ni) + CA_NI_QM_INT_SRC), readl(ni_base(ni) + CA_NI_QM_INT_SRCE));

	/* (the FBM pool config+enable+fill+preload is done above, ...
	 * dev/MEASURED-cortina-ni-rx.c.md sec 160. */
	mutex_lock(&cortina_ni_uni_lock);
	ret = cortina_ni_rx_steer_init(ni);
	mutex_unlock(&cortina_ni_uni_lock);
	if (ret) {
		/* steer failed: RX can't deliver, but don't take TX down */
		dev_err(ni->dev, "RX steer init failed (%d) - staying TX-only\n",
			ret);
		ni_rmw(ni, CA_NI_QM_RMU0_CTRL, CA_NI_QM_RMU0_RX_EN, 0);
		ni->rx = NULL;
		return 0;
	}
	dev_info(ni->dev, "RX: steer done\n");

	netif_napi_add(rx->netdev, &rx->napi, cortina_ni_rx_poll);

	ret = cortina_ni_rx_irqs_init(ni);
	if (ret) {
		/* no RX IRQ: keep TX alive rather than fail the whole device */
		dev_err(ni->dev, "RX irq init failed (%d) - staying TX-only\n",
			ret);
		netif_napi_del(&rx->napi);
		ni_rmw(ni, CA_NI_QM_RMU0_CTRL, CA_NI_QM_RMU0_RX_EN, 0);
		ni->rx = NULL;
		return 0;
	}

	/* /proc already created before the seed (NULL-safe) */

	/* devmem-verify: final golden FE-path config after probe (rxmac rx_en
	 * is added at open; the stock bit12/13/es cpu_en=0xff show here too) */
	dev_info(ni->dev,
		 "M2c RX ready (FE path): port %d -> cpu0/voq0, %u DRAM buffers\n",
		 CA_NI_RX_PORT, CA_NI_RX_EQ_TOTAL_BUF + CA_NI_RX_EQ2_TOTAL_BUF);
	dev_info(ni->dev,
		 "RX golden: static_cfg(a5c0)=0x%08x rx_cntrl=0x%08x rxmac=0x%08x es_ctrl=0x%08x l3tm=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_PORT_STATIC_CFG(CA_NI_RX_PORT)),
		 readl(ni_base(ni) + CA_NI_PORT_RX_CNTRL_CFG(CA_NI_RX_PORT)),
		 readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT)),
		 readl(ni_base(ni) + CA_NI_QM_ES_CTRL),
		 readl(ni_base(ni) + CA_NI_QM_L3TM_NI_PORT_ENA));
	dev_info(ni->dev,
		 "RX golden demux: a190=0x%08x a194=0x%08x a19c=0x%08x a1a0=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX0),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX1),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX3),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX4));
	dev_info(ni->dev,
		 "RX delivery chain: destp9(61a4)=0x%08x prof13(615c)=0x%08x eq13_cfg1(6350)=0x%08x eq13_cfg2(6354)=0x%08x pkt_buf(6228)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)),
		 readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)),
		 readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID)),
		 readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID)),
		 readl(ni_base(ni) + CA_NI_QM_DEST_PORT_PKT_BUF_CFG(CA_NI_RX_CPU_PORT)));
	dev_info(ni->dev,
		 "RX misc match: intern_portid(a1bc)=0x%08x autosync(a010)=0x%08x l2tm_glob(2210)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG),
		 readl(ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC),
		 readl(ni_base(ni) + CA_NI_L2TM_QM_GLOB_BUF_CFG));
	return 0;
}
