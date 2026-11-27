// SPDX-License-Identifier: GPL-2.0-only
/* TIER: FAMILY -- the Luna GMAC0 Ethernet shell: rings, RX window, NAPI, TX, OMCI, gpon0, ethtool, recovery,
 * probe. Never built alone: a chip TU includes luna_eth.h, defines struct luna_eth_chip_state, includes this
 * file, then its bring-up and its struct luna_eth_chip rows (rtl9602c_eth.c; luna_eth_switch.c). */

#include <linux/of_net.h>	/* of_get_mac_address() -- the DT rung of the ladder */
#include <linux/interrupt.h>	/* request_irq, irqreturn_t, IRQF_SHARED */
#include "gpon_omci_core.h"	/* the responder + omci_finalize + the AVC emitters */

/* The DRAM controller page (a die with luna_eth_gmac.memctl). The boot loader trains it on every boot; we write only what stock's own
 * value proves: MEMCTL_MCR bit 19 (stock set, the boot loader clear) and the per-packet flush bit of
 * MEMCTL_FLUSH that stock's NIC xmit pokes. dram_ctl_write_guard.py refuses any other write. */
#define MEMCTL_PHYS	0x18001000
#define MEMCTL_MCR	0x00
#define MEMCTL_FLUSH	0x38

/* US-OMCI TX tuning knobs (see luna_eth_omci_xmit): the ...
 * dev/MEASURED-luna_eth.c.md sec 2. */
static unsigned int omci_sid_idx = 4;	/* PON_SID one-hot shift (word3[28:23]); 4 = SID-64 classify slot (RX_SID group [4]) */

module_param(omci_sid_idx, uint, 0644);
MODULE_PARM_DESC(omci_sid_idx, "US-OMCI PON_SID one-hot shift count (default 4 = SID-64 classify slot)");

/* ExtSpa / PON port (word3[22:16] = (port&0x7F)<<16). The ...
 * dev/MEASURED-luna_eth.c.md sec 3. */
static unsigned int omci_pon_port = 2;	/* ExtSpa / PON port (word3[22:16]) = our PON switch port */
module_param(omci_pon_port, uint, 0644);
MODULE_PARM_DESC(omci_pon_port, "US-OMCI PON port / ExtSpa (default 2 = PON switch port)");

/* DEV sweep: raw descriptor overrides. 0 = use the computed value. Lets us sweep
 * the runtime-unknown opts fields (gmac_id[19:18], extspa[15:13], portmask) live at
 * O5 (echo to the param) without a rebuild, watching RX_SID_GOOD_CNT_US. */
static unsigned int omci_word2_ovr;
module_param(omci_word2_ovr, uint, 0644);
static unsigned int omci_word3_ovr;
module_param(omci_word3_ovr, uint, 0644);
static unsigned int omci_minimal;	/* TEST: 1 = LAN-identical desc (no word0 keep/dislrn/psel, opts2/3=0) to isolate the GMAC TX halt-after-~3 */
module_param(omci_minimal, uint, 0644);

/* HW ring for the US-OMCI descriptor ring: it indexes ...
 * dev/MEASURED-luna_eth.c.md sec 4. */
static unsigned int omci_tx_ring = 0;	/* 0 = LAN ring (GMAC FETCHES it; ring 4 dedicated does NOT fetch — dirty stays 0); 4 = dedicated */	/* dedicated HW-descriptor TX ring (opts3 carries the SID). The ring0 SW 0x8899-tag path only works on the FPGA model — on real silicon the L2 switch strips the inline tag so US OMCI never reaches the US-NIC; the HW descriptor (word3=0x00b20040, DST_SID 64) is the only path that classifies to SID-64 group[4]. */
module_param(omci_tx_ring, uint, 0644);
MODULE_PARM_DESC(omci_tx_ring, "US-OMCI HW TX ring: 0=shared LAN ring0 (test default), 1..5=dedicated ring");

/* Doorbell override. 0xff = "auto": compute the kick from the SAME HW ring used
 * to arm TxFDP (h<4 -> R_IO_CMD bit h; h==4 -> R_IO_CMD1 |= 0x100). Any other
 * value forces an explicit R_IO_CMD bit number (debug only). */
static unsigned int omci_doorbell_bit = 0xff;
module_param(omci_doorbell_bit, uint, 0644);
MODULE_PARM_DESC(omci_doorbell_bit,
		 "US-OMCI doorbell override (0xff=auto from HW ring, default auto)");

ETH_RX_WINDOW_LAB_PARAM();

/* GMAC bring-up mode. Stock latches the multi-ring fetch ...
 * dev/MEASURED-luna_eth.c.md sec 5. */
static unsigned int gmac_reset = 1;	/* now safe to reset: rtl9602c_uboot_swcore_bringup() re-runs U-Boot's GMAC<->switch resync after the IP-block reset, breaking the old "reset kills egress" catch-22, and the stock GMAC init re-establishes the GMAC0-TX->US-NIC direct link */
module_param(gmac_reset, uint, 0644);	/* sampled at open(): ifdown/ifup re-applies */
MODULE_PARM_DESC(gmac_reset, "1=cold GMAC bring-up (IP-block reset + stock IO_CMD edge), 0=legacy inherited");

/* The stock device pokes the network-engine GO (0x18001038[31] + poll-clear) on
 * EVERY submit, not just OMCI injects; mirror it on the LAN xmit path too. */
static unsigned int txgo_xmit = 1;
module_param(txgo_xmit, uint, 0644);
MODULE_PARM_DESC(txgo_xmit, "1=stock per-packet TX GO handshake on the LAN xmit path too");

/* Park-buster: once the TX fetch engine DRAINS a ring it ...
 * dev/MEASURED-luna_eth.c.md sec 7. */
static unsigned int tx_softrearm = 1;
module_param(tx_softrearm, uint, 0644);
MODULE_PARM_DESC(tx_softrearm, "1=re-arm TxFDP at the pending slot on every empty-ring publish (un-parks the drained fetch engine)");

/* Un-park flavor for the soft re-arm (runtime A/B, no rebuild)
 * dev/MEASURED-luna_eth.c.md sec 121. */
static unsigned int unpark_mode;	/* HW A/B 2026-06-11: mode 1 showed no
					 * park-rate benefit (33/100s vs 60/150s)
					 * and ran in the boot that hard-hung the
					 * SoC; default to the proven plain mode */
module_param(unpark_mode, uint, 0644);
MODULE_PARM_DESC(unpark_mode, "soft re-arm flavor: 0=FDP re-point only, 1=+IO_CMD bit5 off/on edge");

/* MSR (0x58) top byte. Live stock runs 0xf0 ... -- dev/MEASURED-luna_eth.c.md sec 8. */
static unsigned int msr_top = 0x10;
module_param(msr_top, uint, 0644);
MODULE_PARM_DESC(msr_top, "MSR(0x58) top byte (0x10=LAN-healthy w/ our init; 0xf0=stock value, stalls our LAN)");

/* Recovery flavor: 1 = CMD-register (0x3B) RST soft-reset + ...
 * dev/MEASURED-luna_eth.c.md sec 122. */
static unsigned int recover_rst = 1;
module_param(recover_rst, uint, 0644);
MODULE_PARM_DESC(recover_rst, "1=CMD.RST soft-reset recovery instead of the IP-block power-cycle");

/* GMAC register offsets (from the NIC base). */
#define R_TxFDP1	0x1300	/* TX ring0 fetch-descriptor pointer (phys) */
#define R_TxCDO1	0x1304	/* TX ring0 current-descriptor offset (u16) */
/* Per-ring TX descriptor {FDP,CDO} pairs, 16-byte stride, NOT 8: stock's ring
 * init writes 0x1300/0x1310/0x1320/0x1330/0x1340 for rings 0..4.  The US-OMCI
 * path uses HW ring 4 (base 0x1340, kick R_IO_CMD1 |= 0x100); h is derived once
 * so the FDP arm and the doorbell can never name different rings. */
#define R_TxFDP(k)	(0x1300 + (k) * 16)	/* ring k fetch-descriptor pointer (stock stride 16) */
#define R_TxCDO(k)	(0x1304 + (k) * 16)	/* ring k current-descriptor offset (u16) */
/* RX multi-ring config block at 0x1380 + k*16 (RxFDP2 region), rings 1..5; ring 0
 * is the RxFDP/RxCDO pair below.  NOT a TX table -- a TX ring activates on TxFDP
 * + OWN(opts1) + kick.  Kept only to document the address. */
#define R_RxMRingCfg(k)	(0x1380 + (k) * 16)	/* RX multiring k config block (stock stride 16) */
/* ★ RX multiring k's {FDP, CDO} pair sits at +0 / +4 of its 16-byte block --
 * the layout ring 0 has at R_RxFDP/R_RxCDO and every TX ring has at
 * R_TxFDP/R_TxCDO, and stock's own hw_reg dump lists RxFDPk/RxCDOk per ring
 * (dev/STOCK_GMAC_REGS.md).  Read only, by the /proc dump (2026-09-05). */
#define R_RxMRingCDO(k)	(R_RxMRingCfg(k) + 4)	/* [RxCDO:RxRingSize] of ring k */
static_assert(R_RxMRingCDO(1) == 0x1394 && R_RxMRingCDO(5) == 0x13d4,
	      "the RX multiring block moved");
/* MSR was a bare literal at five sites here while luna_eth.c already named it
 * (2026-09-05); its right home is the family header, beside R_IMR. */
#define R_MSR		0x58	/* media/flow status; top byte = force flow ctl (msr_top) */
/* ★ THE GMAC0 MAC-LEVEL MIB COUNTERS ARE ONE ARRAY: fourteen ...
 * dev/MEASURED-luna_eth.c.md sec 10. */
#define R_MIB16(n)	(0x10 + 2 * (n))
#define R_TXOKCNT	R_MIB16(0)	/* word = [TXOKCNT:RXOKCNT] */
#define R_TXERR		R_MIB16(2)	/* word = [TXERR:RXERR]     */
#define R_MISSPKT	R_MIB16(4)	/* word = [MISSPKT:FAE]     */
#define R_RXOKPHY	R_MIB16(8)	/* word = [RXOKPHY:RXOKBRD] */
#define R_RXOKMUL	R_MIB16(10)	/* word = [RXOKMUL:TXABT]   */
#define R_TXUNDRN	R_MIB16(12)	/* word = [TXUNDRN:RDUMISSPKT]: frames dropped with no armed descriptor */
static_assert(R_TXOKCNT == 0x10 && R_TXERR == 0x14 && R_MISSPKT == 0x18 &&
	      R_RXOKPHY == 0x20 && R_RXOKMUL == 0x24, "the GMAC MIB array moved");
/* Live-stock 9602C operating values (read off a running stock ONU at O5). The
 * U-Boot value 0x400f3330 is its polled-TFTP config; the stock OS reprograms
 * IO_CMD/IO_CMD1 after the IP-block reset. Bits[3:0] of IO_CMD stay 0 here (the
 * self-clearing per-ring TX_POLL kicks). */
/* Stock RCR: AcceptAllPhys|PhysMatch|Multicast|Broadcast, set from the first program.
 * The switch already filtered; DS OMCI arrives with its TCI where the DA sits, so an
 * even TCI high byte without AcceptAllPhys is a foreign unicast and is dropped. */
#define RCR_STOCK 0x0000000Fu
#define IOCMD_STOCK	0xc059f130
#define IOCMD1_STOCK	0x32000001
#define IOCMD_UBOOT	0x400F3330
#define IOCMD1_UBOOT	0x323F0001
/* SoC per-IP enable (system block 0xb8000600, OUTSIDE the ...
 * dev/MEASURED-luna_eth.c.md sec 11. */
#define R_RRING_ROUTING1 0x1370	/* RX-ring routing by priority: PRI_n_ROUTE = ring# at nibble n (operational default 0x65432100). 0 => all priorities to ring 0. */
/* The control RX ring: the ingress ACL (luna_ctl_acl.h) stamps ARP and PPPoE discovery CTL_RX_PRIO, the
 * GMAC routes that priority to its ring 2 and NAPI drains it first, so a WAN flood that fills the data
 * ring cannot take the link's control with it (the GMAC is luna_eth.c's). */
#define CTL_RING_SIZE	64
#define CTL_RX_PRIO	7
#define CTL_RX_ROUTE	0x21111111	/* one nibble per priority: 7 -> ring 2, the rest ring 1 */
#define IOCMD1_RX_RINGS(n)	(GENMASK((n) - 1, 0) << 16)

/* Descriptor opts1 bits (shared TX/RX where noted). */
#define D_IPCS		BIT(27)	/* TX: insert IPv4 csum */

/* TX cpu-tag DESCRIPTOR path: UNUSED -- we send plain frames ...
 * dev/MEASURED-luna_eth.c.md sec 12. */

#define OTX_RING_SIZE	8	/* dedicated US-OMCI TX ring (low-rate control) */
#define OMCI_RESV	2	/* LAN xmit stops this many slots early so the sparse shared-ring OMCI inject always has room (never dropped) */
#define DUMMY_RING_SIZE	4	/* idle filler armed on the unused HW TX rings (gap rings) */
#define RX_CPU_PREFIX	2	/* switch CPU-port prepends a 2-byte offset word on RX */
/* MEASURED (field X111W capture 2026-11-06): the descriptor length counts the frame AND its FCS, not the prefix. */
static const struct eth_rx_layout luna_eth_rx_layout = {
	.prefix = RX_CPU_PREFIX, .fcs = ETH_RX_FCS_IN_LEN, .buf = RX_BUF_SIZE,
};
/* TX_CPUTAG picks the CPU->switch egress method: 1 = prepend ...
 * dev/MEASURED-luna_eth.c.md sec 13. */
#define TX_CPUTAG	0
#define POLL_INTERVAL	msecs_to_jiffies(2)	/* legacy pure-poll fallback (ep->irq<=0) */
#define REKICK_INTERVAL	msecs_to_jiffies(100)	/* slow TX-unpark backstop when IRQ-driven */

/* The UNI administrative lock, as a DESIRED state per copper ...
 * dev/MEASURED-luna_eth.c.md sec 16. */
static u32 luna_uni_locked_ports;

static bool luna_uni_port_locked(unsigned int port)
{
	return port < 32 && (READ_ONCE(luna_uni_locked_ports) & BIT(port));
}

struct luna_rxq {
	struct rx_desc	*ring;
	dma_addr_t	ring_dma;
	struct sk_buff	**skb;
	dma_addr_t	*buf_dma;
	unsigned int	head;
	unsigned int	n;		/* entries in use */
	unsigned int	tail;		/* the CPU tail the GMAC was last given */
	unsigned int	tail_reg;	/* where it reads it */
	struct eth_rx_window_ring rw;	/* its window (eth_rx_window.h); the tick decides under rxw_lock */
};

struct luna_eth {
	/* ★ THE PER-CHIP TABLE. The switch LUT block MOVED between ...
	 * dev/MEASURED-luna_eth.c.md sec 21. */
	const struct luna_sw_map *swm;
	void __iomem	*base;
	void __iomem	*sw;	/* switch core */
	void __iomem	*txgo;	/* the DRAM controller page, MEMCTL_PHYS */
	struct net_device *ndev;
	struct net_device *wan_ndev;	/* gpon0: WAN data-GEM netdev (DS demux'd from PON port 2, US steered to GPON_DATA_FLOW) */
	struct device	*dev;

	struct luna_rxq rxq[2];	/* [0] data (<= RX_RING_SIZE, DT), [1] control */
	unsigned int	n_rxq;
	bool		ctl_ring_on;	/* the GMAC was programmed with rxq[1] */
	struct sk_buff	*rx_skb[RX_RING_SIZE];
	dma_addr_t	rx_buf_dma[RX_RING_SIZE];
	struct sk_buff	*ctl_skb[CTL_RING_SIZE];
	dma_addr_t	ctl_buf_dma[CTL_RING_SIZE];

	struct tx_desc	*tx_ring;
	dma_addr_t	tx_ring_dma;
	struct sk_buff	*tx_skb[TX_RING_SIZE];
	dma_addr_t	tx_buf_dma[TX_RING_SIZE];
	unsigned int	tx_buf_len[TX_RING_SIZE];
	unsigned int	tx_head, tx_dirty;
	spinlock_t	tx_lock;	/* serialises tx_head/tx_ring: xmit (process)
					 * vs OMCI inject (poll-timer softirq) */
	/* HW ring rotation: the slot TxFDP currently points at (0 after open).
	 * The stall recovery re-arms TxFDP at the first PENDING slot and moves
	 * EOR to the slot before it, so the 64 descriptors stay one ring, just
	 * rotated: HW index j <-> SW slot (rot + j) % size. SW head/dirty slot
	 * numbering is unchanged; only the EOR placement and the TxCDO->slot
	 * translation depend on rot. */
	unsigned int	tx_rot, otx_rot;
	/* stall_since = jiffies of the first tick that saw OWN stuck at the HW
	 * cursor with no dirty progress, 0 = healthy (ethdump stall_ms). The
	 * recovery is the family ring watch's (luna_mib_nic_tx_sample). */
	unsigned long	stall_since;
	unsigned int	stall_lastdirty;
	u32		tx_dropped;	/* frames a recovery dropped: never "sent" to the watch */
	u32		dbg_rearm;	/* soft TxFDP re-arms (publish + watchdog) */
	struct work_struct recover_work;
	struct work_struct rxw_work;	/* the RX window's allocations and frees */
	unsigned int	rx_floor;	/* the data ring's smallest window, from the DRAM the board declares */
	u16		rdu_last;	/* RDUMISSPKT at the last poll */
	spinlock_t	rxw_lock;	/* rxq[0].w: the tick vs ethtool -G */
	struct eth_core_recovery sw_recovery;	/* the switch core reset and its re-init */
	struct luna_state_rd sw_rd[ETH_STATE_BLOCKS];
	struct eth_reg_range sw_rg[ETH_STATE_BLOCKS][3];
	bool		closing;	/* gate recover_work vs ndo_stop teardown */
	bool		hw_up;		/* rings + DMA live from the first open on: the OMCC
					 * and gpon0 ride them whatever eth0's state */
	bool		in_recovery;	/* GMAC block may be power-gated: /proc
					 * diag must not touch its MMIO (bus abort) */
	u32		dbg_tx_recover;	/* completed GMAC power-cycle recoveries */

	/* Dedicated US-OMCI TX ring (the stock OMCC TX ring, default ring 4).
	 * Small single-purpose ring: US OMCI is low-rate control traffic, so a
	 * handful of descriptors is ample, and keeping it separate from the LAN
	 * ring 0 means the OMCC steering descriptor never mixes with LAN frames. */
	struct tx_desc	*otx_ring;
	dma_addr_t	otx_ring_dma;
	struct sk_buff	*otx_skb[OTX_RING_SIZE];
	dma_addr_t	otx_buf_dma[OTX_RING_SIZE];
	unsigned int	otx_buf_len[OTX_RING_SIZE];
	unsigned int	otx_head, otx_dirty;
	/* Idle dummy TX ring armed on the UNUSED HW TX rings (the ...
	 * dev/MEASURED-luna_eth.c.md sec 22. */
	struct tx_desc	*dummy_ring;
	dma_addr_t	dummy_ring_dma;
	/* Shared-ring-0 OMCI test path (omci_tx_ring==0): the OMCI ...
	 * dev/MEASURED-luna_eth.c.md sec 23. */
	int		omci_r0_last_slot;
	/* US OMCI (OMCC) responder state. */
	u8		omci_sn[8];	/* G.984.3 ONU-SN (4 ASCII ID + 4 serial),
					 * for the ONU-G Vendor-ID/Serial GET reply */
	u8		omci_mds;	/* ONU-data (ME 2) MIB-Data-Sync counter */
	/* omci_audit_reads and omci_mds_tries USED TO BE HERE and ...
	 * dev/MEASURED-luna_eth.c.md sec 24. */
	u32		dbg_omci_tx;		/* US OMCI responses queued */
	u32		dbg_omci_tx_drop;	/* dropped: ring full / alloc / map */
	u32		dbg_omci_unhandled;	/* requests with no modelled reply */

	struct timer_list poll_timer;	/* IRQ-driven: 100ms TX-unpark backstop; no IRQ: 2ms pure-poll */
	struct napi_struct napi;
	int		irq;	/* GMAC0 INTC input (platform_get_irq); <=0 = pure-poll fallback */
	struct luna_eth_chip_state cs;	/* the including chip TU's own state */
	/* Host uplink port, learned from the RX descriptor src_port_num. All RX
	 * arrives on the board's single connected LAN port, so this resolves to the
	 * physical switch port the host is on — we then steer CPU->LAN TX there
	 * regardless of the (ambiguous) static port numbering. 0xff = not yet seen. */
	unsigned int	host_port;

	/* Bootloader GMAC0 control snapshot (inherited). */
	u32		ub_tcr, ub_rcr, ub_config, ub_cputagcr, ub_cputag1cr;

	/* RX datapath debug counters (see /proc/ethdump). */
	u32		dbg_poll;	/* poll-timer ticks */
	u32		dbg_filled;	/* RX descriptors HW handed back (D_OWN cleared) */
	u32		dbg_good;	/* frames pushed up the stack */
	u32		dbg_err;	/* RX descriptors with error/oversize */
	u32		dbg_rx_repoll;	/* frames found waiting after the IRQ re-arm, re-polled */
	u32		dbg_ctl_rx;	/* frames the control ring delivered */
	u32		dbg_rxlen;	/* raw length of the last captured RX frame */
	u8		dbg_rxbuf[48];	/* first bytes of the last RX frame (pre-pull) */

	/* OMCI (OMCC stream) trap, armed by the GPON driver once it installs the
	 * OMCC GEM datapath (rtl9602c_eth_set_omci_sid). */
	bool		omci_trap_on;
	u32		dbg_omci_rx;	/* DS OMCI frames trapped to the CPU */
	u32		dbg_omci_lookalike;	/* frames that look like OMCI, not trapped as OMCI: forwarded */
	u32		dbg_omci_rxlen;	/* length of the last OMCI frame */
	u8		dbg_omci_rxbuf[48];	/* the last DS OMCI baseline message */

	struct luna_loop	*loop;	/* LAN loop detection (luna_loop.c) */
};

static struct luna_eth *g_ep;	/* single-instance, for /proc diag */

static inline u32 ep_rd(struct luna_eth *ep, u32 r) { return ioread32(ep->base + r); }
static inline void ep_wr(struct luna_eth *ep, u32 r, u32 v) { iowrite32(v, ep->base + r); }

/* The MSR top byte is ours (msr_top): a recovery of the CPU/fibre path puts it back and names a drift. */
static void luna_eth_msr_apply(struct luna_eth *ep, bool recovering)
{
	u32 was = ep_rd(ep, R_MSR), want = (was & 0x00ffffff) | ((msr_top & 0xffu) << 24);

	if (recovering && was != want)
		netdev_err(ep->ndev, "NIC MSR read %08x, ours %08x: put back\n", was, want);
	ep_wr(ep, R_MSR, want);
}

static inline unsigned int tx_slot(struct luna_eth *ep, unsigned int counter)
{
	return (ep->tx_rot + counter) % TX_RING_SIZE;
}

static inline unsigned int otx_slot(struct luna_eth *ep, unsigned int counter)
{
	return (ep->otx_rot + counter) % OTX_RING_SIZE;
}

static inline unsigned int tx_eor_slot(struct luna_eth *ep)
{
	return TX_RING_SIZE - 1;	/* wrap descriptor: fixed physical slot */
}

static inline unsigned int otx_eor_slot(struct luna_eth *ep)
{
	return OTX_RING_SIZE - 1;
}

/* Align the SW producers to the live engine positions. Call ...
 * dev/MEASURED-luna_eth.c.md sec 27. */
static void luna_tx_align(struct luna_eth *ep)
{
	ep->tx_rot = luna_gmac_ring_rot(ioread16(ep->base + R_TxCDO1), ep->tx_dirty, TX_RING_SIZE);
	ep->otx_rot = luna_gmac_ring_rot(0, ep->otx_dirty, OTX_RING_SIZE);
	if (omci_tx_ring) {
		unsigned int h = luna_gmac_omci_hwring(omci_tx_ring);

		ep->otx_rot = luna_gmac_ring_rot(ioread16(ep->base + R_TxCDO(h)), ep->otx_dirty, OTX_RING_SIZE);
	}
	ep->dbg_rearm++;
}

/* RX ring per priority: every priority to the data ring, CTL_RX_PRIO to the control ring once it is armed. */
static void luna_rx_route(struct luna_eth *ep)
{
	unsigned int k;

	for (k = 0; k < 7; k++)
		ep_wr(ep, R_RRING_ROUTING1 + k * 4, ep->ctl_ring_on ? CTL_RX_ROUTE : 0);
}

/* Arm the GMAC OMCI trap so DS frames on stream-id `sid` (the ...
 * dev/MEASURED-luna_eth.c.md sec 28. */
void rtl9602c_eth_set_omci_sid(unsigned int sid)
{
	struct luna_eth *ep = g_ep;

	if (!ep)
		return;
	/* cpu-tag (CPUTAGCR/CPUTAG1CR) is armed once in open() at the ...
	 * dev/MEASURED-luna_eth.c.md sec 29. */
	luna_chip_omci_sid(ep, sid);
	luna_rx_route(ep);
	ep->omci_trap_on = true;
	netdev_dbg(ep->ndev, "OMCI trap armed: SID %u (cpu-tag CPUTAGCR/CPUTAG1CR armed at open(); RX-ring routing re-applied)\n",
		    sid);
}
EXPORT_SYMBOL(rtl9602c_eth_set_omci_sid);

/* Provision the ONU identity (G.984.3 ONU-SN: 4 ASCII vendor ...
 * dev/MEASURED-luna_eth.c.md sec 30. */
void rtl9602c_eth_set_omci_identity(const u8 *sn8)
{
	luna_omci_set_sn(sn8);
}
EXPORT_SYMBOL(rtl9602c_eth_set_omci_identity);

/* DS OMCI frames that actually reached the CPU NIC ring (private dbg counter). */
u32 rtl9602c_eth_omci_rx_count(void)
{
	struct luna_eth *ep = g_ep;

	return ep ? ep->dbg_omci_rx : GPON_OMCI_RX_UNAVAIL;
}
EXPORT_SYMBOL(rtl9602c_eth_omci_rx_count);

/* gpon0 (WAN) RX packet count: no WAN netdev is COULD NOT ASK, never 0 (the
 * O5 watchdog tears a ranged ONU down on 0) -- dev/MEASURED-luna_eth.c.md sec 123. */
u32 rtl9602c_eth_wan_rx_count(void)
{
	struct luna_eth *ep = g_ep;

	return (ep && ep->wan_ndev) ? (u32)ep->wan_ndev->stats.rx_packets
				    : GPON_OMCI_RX_UNAVAIL;
}
EXPORT_SYMBOL(rtl9602c_eth_wan_rx_count);

/* US-OMCI TX-ring "dirty" cursor = number of OMCC descriptors ...
 * dev/MEASURED-luna_eth.c.md sec 31. */
u32 rtl9602c_eth_omci_tx_dirty(void)
{
	struct luna_eth *ep = g_ep;

	if (!ep)
		return 0;
	return (omci_tx_ring == 0) ? ep->tx_dirty : ep->otx_dirty;
}
EXPORT_SYMBOL(rtl9602c_eth_omci_tx_dirty);

/* US-OMCI responses the ring REFUSED (full / closing / no buffer): beside the queued count, the diagnosis. */
u32 rtl9602c_eth_omci_tx_dropped(void)
{
	struct luna_eth *ep = g_ep;

	return ep ? ep->dbg_omci_tx_drop : 0;
}
EXPORT_SYMBOL(rtl9602c_eth_omci_tx_dropped);

/* THE FIRST RUNG of the declared precedence (bootarg -> ...
 * dev/MEASURED-luna_eth.c.md sec 44. */
static char *mac_param;
module_param_named(mac, mac_param, charp, 0444);
MODULE_PARM_DESC(mac, "station MAC handed in at boot, aa:bb:cc:dd:ee:ff");

/* Thin wrappers over the family helpers, kept at their own names so the call
 * sites and this diff stay small. The BODIES live in luna_eth_regs.h. */
static void luna_eth_get_hwaddr(struct luna_eth *ep, u8 *mac)
{
	luna_idr_get(ep->base, mac);
}

static void luna_eth_set_hwaddr(struct luna_eth *ep, const u8 *mac)
{
	luna_idr_set(ep->base, mac);
}


/* OMCI MIB-Data-Sync (ME 2 attr 1) seed. THE PROVISIONING GATE
 * dev/MEASURED-luna_eth.c.md sec 46. */
static unsigned int omci_mds_seed = OMCI_MDS_POISON_SEED;	/* 1..30: satisfies `rsync < 31` unconditionally, so the
					 * OLT provisions whatever it has stored. NOT 0 -- see above. */
module_param(omci_mds_seed, uint, 0644);
MODULE_PARM_DESC(omci_mds_seed, "OMCI ME2 MIB-Data-Sync boot seed (1..30 forces the OLT to re-provision: its gate takes rsync<31 as not-in-sync)");
/* mds_reset0: on an on-wire MIB-Reset (MT 0x4f), zero the ...
 * dev/MEASURED-luna_eth.c.md sec 47. */
static bool mds_reset0 = true;
module_param(mds_reset0, bool, 0644);
MODULE_PARM_DESC(mds_reset0, "on MIB-Reset zero ME2 MIB-Data-Sync (G.988/stock, default) vs re-seed");

/* THE MIB-DATA-SYNC SEED IS A GUESS, AND A GUESS CANNOT BE ...
 * dev/MEASURED-luna_eth.c.md sec 48. */
static bool omci_mds_adapt = true;
module_param(omci_mds_adapt, bool, 0644);
MODULE_PARM_DESC(omci_mds_adapt, "walk the reported ME2 MIB-Data-Sync when the OLT reads but never provisions (default on)");
static unsigned int omci_mds_adapt_reads = 12;
module_param(omci_mds_adapt_reads, uint, 0644);
MODULE_PARM_DESC(omci_mds_adapt_reads, "DS OMCI reads with no provisioning before the MIB-Data-Sync is advanced");

/* The MDS walk's step is the CORE's OMCI_MDS_WALK_STEP ...
 * dev/MEASURED-luna_eth.c.md sec 49. */
/* Program the station address into the hardware (IDR) on a ...
 * dev/MEASURED-luna_eth.c.md sec 50. */
static int luna_eth_set_mac_address(struct net_device *ndev, void *p)
{
	struct luna_eth *ep = netdev_priv(ndev);
	int ret = eth_mac_addr(ndev, p);

	if (ret)
		return ret;
	luna_eth_set_hwaddr(ep, ndev->dev_addr);
	/* Keep the WAN (gpon0) identity tracking the board MAC provisioned onto eth0
	 * (rtk_factory), applying the stock model offset: WAN = LAN + wan_mac_offset
	 * (see luna_wan_mac). This is the MAC the ISP/OLT identifies the ONU by. */
	if (ep->wan_ndev && ep->wan_ndev->addr_assign_type != NET_ADDR_SET) {
		u8 wmac[ETH_ALEN];

		luna_chip_wan_mac(ep, wmac, ndev->dev_addr);
		eth_hw_addr_set(ep->wan_ndev, wmac);
		luna_omci_set_wan_mac(wmac);
	}
	return 0;
}

/* Give RX descriptor @idx a fresh buffer and hand it to HW (own=1). */
/* The body is the FAMILY's (luna_eth_regs.h). */
static int luna_eth_refill(struct luna_eth *ep, struct luna_rxq *q, unsigned int idx)
{
	return luna_rx_refill(ep->ndev, ep->dev, q->ring, q->skb, q->buf_dma, idx, q->n, RX_BUF_SIZE);
}

/* ===== M2: G.988 OMCI responder + upstream OMCC TX ...
 * dev/MEASURED-luna_eth.c.md sec 51. */
#define TXD0_OMCI_KEEP_DISLRN_PSEL 0x02240000u
/* TXD2_OMCI_CPUTAG / TXD2_OMCI_EFID: moved to rtl9602c_l34_logic.h with
 * the hoisted steering encode (rtl9602c_omci_txd_word2). */
/* word0 descriptor-flag ORs applied by the ring-submit path (it also re-ORs
 * word0 & 0x077e0000, see above). */
#define TXD0_DESC_FLAGS		(0xb8800000u | 0x40000000u)	/* = 0xf8800000 */
/* The ★ 9602C opts3/word3 layout (the 9602C-vs-9607C ...
 * dev/MEASURED-luna_eth.c.md sec 124. */
#define LUNA_OMCC_SID	64			/* OMCC US SID (== GPON flow 64) */
#define RTL8_4_TAG_LEN		8			/* software rtl8_4 0x8899 cpu-tag (also used by the LAN xmit below) */

/* LAYER BOUNDARY (2026-08-05; MIC resolved 2026-09-01, ...
 * dev/MEASURED-luna_eth.c.md sec 52. */
static void luna_eth_omci_reclaim(struct luna_eth *ep)
{
	while (ep->otx_dirty != ep->otx_head) {
		unsigned int i = otx_slot(ep, ep->otx_dirty);

		if (ep->otx_ring[i].opts1 & D_OWN)	/* HW still owns it (OWN in opts1/word0) */
			break;
		dma_unmap_single(ep->dev, ep->otx_buf_dma[i], ep->otx_buf_len[i],
				 DMA_TO_DEVICE);
		dev_consume_skb_any(ep->otx_skb[i]);
		ep->otx_skb[i] = NULL;
		ep->otx_dirty++;
	}
}

/* Forward decl: the shared-ring-0 OMCI path reclaims the LAN ring (defined with
 * the rest of the LAN datapath further down). */
static void luna_eth_tx_reclaim(struct luna_eth *ep);
static void luna_eth_tx_rekick(struct luna_eth *ep);

/* Stock per-packet TX-fetch GO (observed in the stock ...
 * dev/MEASURED-luna_eth.c.md sec 53. */
static void luna_eth_tx_fetch(struct luna_eth *ep)
{
	int n;

	if (!ep->txgo)
		return;
	iowrite32(ioread32(ep->txgo + MEMCTL_FLUSH) | BIT(31), ep->txgo + MEMCTL_FLUSH);
	for (n = 0; n < 100; n++) {
		if (!(ioread32(ep->txgo + MEMCTL_FLUSH) & BIT(31)))
			break;
		cpu_relax();
	}
}

/* the steered-descriptor LAN-ring-0 publish, written ONCE
 * dev/MEASURED-luna_eth.c.md sec 54. */
static inline u32 txd_word0_steered(unsigned int len)
{
	return D_FS | D_LS | D_TXCRC | D_IPCS | (len & TXD_LEN_MASK) |
	       TXD0_OMCI_KEEP_DISLRN_PSEL;
}

/* Claim the next LAN-ring-0 slot for @skb and describe its ...
 * dev/MEASURED-luna_eth.c.md sec 125. */
static unsigned int txd_take_slot(struct luna_eth *ep, struct sk_buff *skb, dma_addr_t da, u32 len)
{
	unsigned int i = tx_slot(ep, ep->tx_head);

	ep->tx_skb[i] = skb;		/* the LAN ring reclaim frees it */
	ep->tx_buf_dma[i] = da;
	ep->tx_buf_len[i] = len;
	ep->tx_ring[i].addr = da | DMA_BUS_WINDOW;
	return i;
}

/* Hand slot @i to the hardware. OWN goes into opts1 LAST and ...
 * dev/MEASURED-luna_eth.c.md sec 126. */
static void txd_publish(struct luna_eth *ep, unsigned int i, u32 word0,
			bool may_skip_go)
{
	wmb();				/* descriptor body before ownership */
	ep->tx_ring[i].opts1 = word0 | D_OWN;	/* OWN in opts1: publish to HW */
	wmb();
	ep->tx_head++;
	/* THE SAME KNOB AS THE LAN PATH, AND IT WAS ONLY ON ONE OF ...
	 * dev/MEASURED-luna_eth.c.md sec 55. */
	if (txgo_xmit || !may_skip_go)
		luna_eth_tx_fetch(ep);	/* stock per-packet TX-fetch GO (0x18001038[31]) */
	ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* kick LAN ring 0 */
}

/* Shared-ring-0 US-OMCI transmit (omci_tx_ring==0): the OMCI ...
 * dev/MEASURED-luna_eth.c.md sec 56. */
static int luna_eth_omci_xmit_ring0(struct luna_eth *ep, const u8 *omci,
					unsigned int len)
{
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i;
	dma_addr_t da;
	u32 word0;

	/* LAN-ring HW-descriptor path: the bare OMCI PDU on the proven-to-fetch LAN
	 * ring 0 (kick R_IO_CMD bit0) with the CORRECTED HW cpu-tag descriptor -- NOT
	 * the SW 0x8899 tag.  Isolates RING from DESCRIPTOR: HW ring 4 does fetch, yet
	 * RX_SID_GOOD stays 0. */
	{	/* Stock sends the OMCI PDU RAW (48 bytes, no Ethernet pad) — the
		 * working stock ref ONU's TX descriptor is len=0x30=48. The old
		 * pad-to-60 was a TX-stall workaround, now obsolete (the SW-follows-
		 * HW ring alignment fixed the runt stall). Send exactly what stock
		 * sends so the US-NIC sees the same frame. */
		skb = netdev_alloc_skb(ep->ndev, len);
		if (!skb) {
			ep->dbg_omci_tx_drop++;
			return -ENOMEM;
		}
		skb_put(skb, len);
		memcpy(skb->data, omci, len);	/* bare OMCI PDU, raw length */
	}

	da = dma_map_single(ep->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -ENOMEM;
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	/* The GPON driver calls in from ITS OWN timer/softirq, so the ...
	 * dev/MEASURED-luna_eth.c.md sec 57. */
	if (ep->closing || !ep->tx_ring) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -ENODEV;
	}
	/* Reclaim the LAN ring first so a free slot is visible; if the LAN ring is
	 * full, DROP the OMCI frame (do NOT stop the queue — that would stall LAN
	 * TX for a control frame). The OLT retransmits the OMCI request. */
	luna_eth_tx_reclaim(ep);
	/* If the shared LAN ring is full, the GMAC simply hasn't ...
	 * dev/MEASURED-luna_eth.c.md sec 58. */
	{
		int spin = 64;

		while (luna_gmac_tx_ring_full(ep->tx_head, ep->tx_dirty,
					      TX_RING_SIZE, 0) && spin-- > 0) {
			ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* retire ring0 */
			cpu_relax();
			luna_eth_tx_reclaim(ep);
		}
	}
	if (luna_gmac_tx_ring_full(ep->tx_head, ep->tx_dirty,
				   TX_RING_SIZE, 0)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -EBUSY;
	}
	i = txd_take_slot(ep, skb, da, len);

	/* word0 (opts1) = FS|LS|len|0x02240000 ... -- dev/MEASURED-luna_eth.c.md sec 59. */
	word0 = txd_word0_steered(len);
	if (i == tx_eor_slot(ep))
		word0 |= D_EOR;

	/* HW cpu-tag steering words -- EXACT stock values (live ...
	 * dev/MEASURED-luna_eth.c.md sec 127. */
	ep->tx_ring[i].opts2 = luna_chip_pon_txd_word2(ep, omci_word2_ovr);
	ep->tx_ring[i].opts3 = luna_chip_omci_txd_word3(ep, omci_word3_ovr);
	ep->tx_ring[i].opts4 = 0;
	if (omci_minimal) {	/* TEST: descriptor IDENTICAL to the draining LAN path (no keep/dislrn/psel, no cpu-tag) */
		word0 = D_FS | D_LS | D_TXCRC | (len & TXD_LEN_MASK);
		if (i == tx_eor_slot(ep))
			word0 |= D_EOR;
		ep->tx_ring[i].opts2 = 0;
		ep->tx_ring[i].opts3 = 0;
	}
	/* Recorded BEFORE the publish, not after it as this used to be: it is a
	 * diagnostic, and naming the slot while it is still ours is the half that
	 * can never describe a descriptor the engine has already eaten. */
	ep->omci_r0_last_slot = (int)i;
	txd_publish(ep, i, word0, false);  /* US OMCI: the GO is NOT optional */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (IS_ENABLED(CONFIG_LUNA_DEBUG) && ep->dbg_omci_tx < 30) {
		/* PRIME PROBE: did the GMAC INSERT the cpu-tag on the ...
		 * dev/MEASURED-luna_eth.c.md sec 60. */
		u32 post_w0;
		udelay(120);			/* let the GMAC retire this 48B TX */
		post_w0 = ep->tx_ring[i].opts1;
		netdev_info(ep->ndev,
			"omci_tx[ring0]: w3=%08x slot=%u | post_w0=%08x (0x30000030=tag-inserted, 0x32240030=NOT)\n",
			ep->tx_ring[i].opts3, i, post_w0);
	}

	ep->dbg_omci_tx++;
	ep->ndev->stats.tx_packets++;
	ep->ndev->stats.tx_bytes += len;
	return 0;
}

/* US-OMCI upstream transmit on the DEDICATED OMCC ring. The ...
 * dev/MEASURED-luna_eth.c.md sec 61. */
static netdev_tx_t luna_eth_wan_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct luna_eth *ep = *(struct luna_eth **)netdev_priv(ndev);
	unsigned long flags;
	unsigned int i, len;
	dma_addr_t da;
	u32 word0;

	if (skb_put_padto(skb, ETH_ZLEN))	/* pad runts to the min Ethernet frame; frees skb on error */
		return NETDEV_TX_OK;
	luna_vlan_note(GPON_VLAN_TX, skb);
	len = skb->len;

	da = dma_map_single(ep->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	if (ep->closing || !ep->tx_ring || !luna_gpon_data_ready()) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	luna_eth_tx_reclaim(ep);	/* shared LAN ring 0; drop on full (DHCP retransmits) */
	/* THE OMCI RESERVE IS HONOURED HERE TOO (2026-09-04). This ...
	 * dev/MEASURED-luna_eth.c.md sec 62. */
	if (luna_gmac_tx_ring_full(ep->tx_head, ep->tx_dirty,
				   TX_RING_SIZE, OMCI_RESV)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	i = txd_take_slot(ep, skb, da, len);
	word0 = txd_word0_steered(len);
	if (i == tx_eor_slot(ep))
		word0 |= D_EOR;
	ep->tx_ring[i].opts2 = luna_chip_pon_txd_word2(ep, 0);	/* stock cputag|efid */
	ep->tx_ring[i].opts3 = luna_chip_wan_txd_word3(ep, luna_wan_tx_flow(skb->data, len, luna_chip_gmac.wan_ctrl_flow));
	ep->tx_ring[i].opts4 = 0;
	txd_publish(ep, i, word0, true);   /* WAN: the throughput A/B may skip it */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	return NETDEV_TX_OK;
}

static int luna_eth_wan_open(struct net_device *ndev)
{
	struct luna_eth *ep = *(struct luna_eth **)netdev_priv(ndev);

	/* Set the WAN identity the OLT/ISP uses to recognise this ...
	 * dev/MEASURED-luna_eth.c.md sec 128. */
	/* An assigned MAC (an ISP's original ONU, cloned: operator 2026-10-22) is kept across ifup; the
	 * board-derived one is only the default. */
	if (ep && ep->ndev && is_valid_ether_addr(ep->ndev->dev_addr) && ndev->addr_assign_type != NET_ADDR_SET) {
		u8 wmac[ETH_ALEN];

		luna_chip_wan_mac(ep, wmac, ep->ndev->dev_addr);
		eth_hw_addr_set(ndev, wmac);
	}
	luna_omci_set_wan_mac(ndev->dev_addr);
	if (ep)
		luna_chip_wan_pin(ep, ndev->dev_addr);
	/* RX/TX rings + NAPI are owned by eth0 (shared HW); open just enables the queue and
	 * holds carrier up so netifd runs the DHCP client. */
	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int luna_eth_wan_set_mac(struct net_device *ndev, void *addr)
{
	struct luna_eth *ep = *(struct luna_eth **)netdev_priv(ndev);
	int rc = eth_mac_addr(ndev, addr);

	if (rc)
		return rc;
	luna_omci_set_wan_mac(ndev->dev_addr);	/* the OLT reads the WAN MAC through OMCI too */
	if (ep && netif_running(ndev))
		luna_chip_wan_pin(ep, ndev->dev_addr);
	return 0;
}

static int luna_eth_wan_stop(struct net_device *ndev)
{
	struct luna_eth *ep = *(struct luna_eth **)netdev_priv(ndev);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	if (ep)
		luna_chip_wan_pin(ep, NULL);
	return 0;
}

/* The kernel's TX watchdog on eth0 and gpon0, which share ring 0; gpon0 drops on a full ring rather than stop its
 * queue, so in practice eth0 is the one it fires on. */
static void luna_eth_ring_timeout(struct luna_eth *ep, struct net_device *ndev)
{
	unsigned long flags;

	netdev_err(ndev, "tx timeout: ring head %u dirty %u, omci head %u dirty %u\n",
		   ep->tx_head, ep->tx_dirty, ep->otx_head, ep->otx_dirty);
	eth_ring_watch_tx(&luna_ring_watch, ndev);
	spin_lock_irqsave(&ep->tx_lock, flags);
	luna_eth_tx_reclaim(ep);
	luna_eth_omci_reclaim(ep);
	luna_eth_tx_rekick(ep);
	spin_unlock_irqrestore(&ep->tx_lock, flags);
}

static void luna_eth_wan_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	luna_eth_ring_timeout(*(struct luna_eth **)netdev_priv(ndev), ndev);
}

static const struct net_device_ops luna_eth_wan_ops = {
#ifdef CONFIG_GPON_FLOW_OFFLOAD
	.ndo_setup_tc		= luna_chip_setup_tc,
#endif
#ifdef LUNA_CHIP_WAN_RX_MODE
	.ndo_set_rx_mode	= LUNA_CHIP_WAN_RX_MODE,
#endif
	.ndo_open		= luna_eth_wan_open,
	.ndo_stop		= luna_eth_wan_stop,
	.ndo_start_xmit		= luna_eth_wan_xmit,
	.ndo_tx_timeout		= luna_eth_wan_tx_timeout,
	.ndo_set_mac_address	= luna_eth_wan_set_mac,
	.ndo_validate_addr	= eth_validate_addr,
};

static int luna_eth_omci_xmit(struct luna_eth *ep, const u8 *omci,
				  unsigned int len)
{
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i, hwring;
	bool kick_iocmd1;
	dma_addr_t da;
	u32 word0, word2, word3, dmask;

	if (len < 8 || len > 1500)
		return -EINVAL;
	/* omci_tx_ring==0: shared-ring-0 test path — enqueue on the proven LAN
	 * ring 0 with the OMCI steering descriptor (isolates steering from the
	 * "ring 4 won't fetch" problem). All ring instances 1..5 use the dedicated
	 * ring below. */
	if (omci_tx_ring == 0)
		return luna_eth_omci_xmit_ring0(ep, omci, len);
	/* omci_tx_ring is the HW ring h directly — the SAME h used to arm R_TxFDP(h)
	 * in open(). Derive the doorbell from h so they can never disagree (the prior
	 * bug: ring armed at h=4/0x1340 but kicked R_IO_CMD bit0 = h=0). Stock kick
	 * behavior: h<4 -> R_IO_CMD |= 1<<h; h==4 -> R_IO_CMD1 |= 0x100. */
	kick_iocmd1 = luna_gmac_omci_doorbell(omci_tx_ring, omci_doorbell_bit,
					     &hwring, &dmask);

	{	/* Pad a runt OMCI PDU up to the min Ethernet frame. The GMAC TX engine
		 * stalls when fed sub-60B frames (the LAN path never sends runts; the
		 * 48B selftest PDU did — the ring drained ~3 then froze, OWN stuck). Zero-
		 * pad to 60 so the GMAC fetch engine keeps draining. */
		unsigned int srclen = len;
		if (len < 60)
			len = 60;
		skb = netdev_alloc_skb(ep->ndev, len);
		if (!skb) {
			ep->dbg_omci_tx_drop++;
			return -ENOMEM;
		}
		skb_put(skb, len);
		memset(skb->data, 0, len);
		memcpy(skb->data, omci, srclen);	/* bare OMCI PDU, zero-padded to 60 */
	}

	da = dma_map_single(ep->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -ENOMEM;
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	if (ep->closing || !ep->otx_ring) {	/* see the ring0-path guard */
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -ENODEV;
	}
	luna_eth_omci_reclaim(ep);
	if (luna_gmac_tx_ring_full(ep->otx_head, ep->otx_dirty,
				   OTX_RING_SIZE, 0)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -EBUSY;		/* ring full; OLT retransmits the request */
	}
	i = otx_slot(ep, ep->otx_head);
	ep->otx_skb[i] = skb;
	ep->otx_buf_dma[i] = da;
	ep->otx_buf_len[i] = len;
	ep->otx_ring[i].addr = da | DMA_BUS_WINDOW;

	/* word0 (opts1) = FS|LS|len|0x02240000 ... -- dev/MEASURED-luna_eth.c.md sec 63. */
	word0 = D_FS | D_LS | D_TXCRC | D_IPCS | (len & TXD_LEN_MASK);
	word0 |= TXD0_OMCI_KEEP_DISLRN_PSEL;
	/* Same fix as the ring-0 path: DO NOT OR TXD0_DESC_FLAGS (0xf8800000) — it sets
	 * D_EOR (bit30) on EVERY descriptor, so the GMAC treats each as end-of-ring and
	 * never drains the ring (OWN stuck, dirty stalls). D_EOR belongs ONLY on the wrap
	 * slot (set just below), exactly like the working LAN TX path. */
	if (i == otx_eor_slot(ep))
		word0 |= D_EOR;

	/* word2 (opts2) = cputag(31) | efid(19), tx_portmask [18:16] ...
	 * dev/MEASURED-luna_eth.c.md sec 64. */
	word2 = luna_chip_pon_txd_word2(ep, omci_word2_ovr);	/* stock cputag|efid; hoisted encode */

	/* word3 (opts3): stock US-OMCI steering = one-hot PON_SID ...
	 * dev/MEASURED-luna_eth.c.md sec 65. */
	word3 = luna_chip_omci_txd_word3(ep, omci_word3_ovr);	/* 9602C: pmask[28:23]|SID[22:16] */

	if (omci_minimal) {	/* TEST: descriptor IDENTICAL to the draining LAN path */
		word0 = D_FS | D_LS | D_TXCRC | (len & TXD_LEN_MASK);
		if (i == otx_eor_slot(ep))
			word0 |= D_EOR;
		word2 = 0;
		word3 = 0;
	}
	ep->otx_ring[i].opts2 = word2;		/* body first */
	ep->otx_ring[i].opts3 = word3;
	ep->otx_ring[i].opts4 = 0;
	wmb();				/* descriptor body before ownership */
	ep->otx_ring[i].opts1 = word0 | D_OWN;	/* publish: OWN in opts1 (word0) */
	wmb();
	ep->otx_head++;
	/* Kick the per-ring poll doorbell for THIS HW ring (h). HW ring 4 is special-
	 * cased to R_IO_CMD1 |= 0x100; rings 0..3 use R_IO_CMD bit h -- register and
	 * mask both come from luna_gmac_omci_doorbell(), the SAME derivation as the
	 * R_TxFDP(h) arm in open(), so the engine fetches the ring we filled. */
	if (kick_iocmd1)
		/* HW ring 4 (TxFDP5) poll "go" = IO_CMD1 |= TX_POLL5 (bit8 = ...
		 * dev/MEASURED-luna_eth.c.md sec 129. */
		ep_wr(ep, R_IO_CMD1, ep_rd(ep, R_IO_CMD1) | dmask);
	else
		ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | dmask);
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (IS_ENABLED(CONFIG_LUNA_DEBUG) && ep->dbg_omci_tx < 8)
		netdev_info(ep->ndev,
			"omci_tx: w0=%08x w2=%08x w3=%08x hwring=%u doorbell=%s (sid_idx=%u pon=%u len=%u)\n",
			word0, word2, word3, hwring,
			kick_iocmd1 ? "R_IO_CMD1|0x100" : "R_IO_CMD bit",
			omci_sid_idx, omci_pon_port, len);

	ep->dbg_omci_tx++;
	ep->ndev->stats.tx_packets++;
	ep->ndev->stats.tx_bytes += len;
	return 0;
}

/* OLT-INDEPENDENT US-OMCI datapath self-test: inject a ...
 * dev/MEASURED-luna_eth.c.md sec 66. */
void rtl9602c_eth_omci_selftest(void)
{
	struct luna_eth *ep = g_ep;
	u8 frame[48];

	if (!ep)
		return;
	/* Arm the GMAC OMCI cpu-tag routing (CPUTAGCR=0x901eff04) — ...
	 * dev/MEASURED-luna_eth.c.md sec 130. */
	rtl9602c_eth_set_omci_sid(LUNA_OMCC_SID);
	memset(frame, 0, sizeof(frame));
	frame[0] = 0x00; frame[1] = 0x01;	/* TID */
	frame[2] = 0x29;			/* MT = Get-response (0x09 | AK 0x20) */
	frame[3] = 0x0a;			/* DevId (baseline) */
	frame[4] = 0x01; frame[5] = 0x00;	/* ME class 256 (ONT-G) */
	luna_eth_omci_xmit(ep, frame, sizeof(frame));
}
EXPORT_SYMBOL(rtl9602c_eth_omci_selftest);

/* OMCI (ITU-T G.988): THE RESPONDER IS THE COMMON CORE'S; ...
 * dev/MEASURED-luna_eth.c.md sec 67. */
static struct omci_onu luna_onu;

/* RX copies the original baseline prefix and length to the shared timer owner. */
static void luna_eth_omci_input(struct luna_eth *ep, const u8 *msg,
				unsigned int len)
{
	int rc = luna_omci_enqueue(ep, msg, len);

	if (rc)
		ep->dbg_omci_unhandled++;
}

/* LAYER BOUNDARY -- RESOLVED 2026-09-02: the autonomous AVC ...
 * dev/MEASURED-luna_eth.c.md sec 68. */
void rtl9602c_eth_omci_set_optical(s16 rx_level, s16 tx_level)
{
	luna_omci_set_optical((u16)rx_level, (u16)tx_level);
}

void rtl9602c_eth_omci_report_oper_up(void)
{
	luna_omci_report_oper_up();
}
EXPORT_SYMBOL(rtl9602c_eth_omci_report_oper_up);

/* `napi_ctx` says whether the caller is INSIDE napi->poll. It ...
 * dev/MEASURED-luna_eth.c.md sec 69. */

#ifdef CONFIG_LUNA_DEBUG
/* Bring-up framing check: the first N frames each way, descriptor beside the bytes. */
static int rx_dump = 6;
module_param(rx_dump, int, 0644);
MODULE_PARM_DESC(rx_dump, "hex-dump the first N received frames (bring-up framing check)");
static int tx_dump = 6;
module_param(tx_dump, int, 0644);
MODULE_PARM_DESC(tx_dump, "hex-dump the first N transmitted frames");
static int luna_rx_dumped, luna_tx_dumped;

static void luna_eth_rx_dump(struct luna_eth *ep, const struct rx_desc *d, const u8 *data, unsigned int len, int geom)
{
	if (luna_rx_dumped >= rx_dump)
		return;
	luna_rx_dumped++;
	dev_info(ep->dev, "rx0 desc: opts1=%08x opts2=%08x opts3=%08x frame=%u geom=%d (src_port=%u reason=%u)\n",
		 d->opts1, d->opts2, d->opts3, len, geom, luna_gmac_rx_src_port(ep->swm->rx_layout, d->opts3),
		 luna_gmac_rx_reason(ep->swm->rx_layout, d->opts2, d->opts3));
	print_hex_dump(KERN_INFO, "rx0: ", DUMP_PREFIX_OFFSET, 16, 1, data, 32, false);
}

static void luna_eth_tx_dump(const u8 *data, unsigned int len)
{
	if (luna_tx_dumped >= tx_dump)
		return;
	luna_tx_dumped++;
	print_hex_dump(KERN_INFO, "tx0: ", DUMP_PREFIX_OFFSET, 16, 1, data, min(len, 48u), false);
}
#else
static inline void luna_eth_rx_dump(struct luna_eth *ep, const struct rx_desc *d, const u8 *data, unsigned int len,
				    int geom) { }
static inline void luna_eth_tx_dump(const u8 *data, unsigned int len) { }
#endif

/* -> true for a DS OMCI the switch trapped. A frame that only looks like one is forwarded: counted, and under the
 * OMCI diagnostics named with its port, so where such frames come from is seen (the field's came before its CPU-port
 * stalls). */
static bool luna_omci_taken(struct luna_eth *ep, const struct rx_desc *d, const u8 *data,
				const struct eth_rx_frame *fr)
{
	enum luna_gmac_ds_omci omci = luna_gmac_rx_ds_omci(ep->swm->rx_layout, ep->omci_trap_on, d->opts2, d->opts3,
							  data, fr->off + fr->len, ep->swm->pon_port,
							  ep->swm->omci_cpu_reason, fr->off, RX_BUF_SIZE);

	if (omci != LUNA_GMAC_DS_OMCI_LOOKALIKE)
		return omci == LUNA_GMAC_DS_OMCI_TRAPPED;
	ep->dbg_omci_lookalike++;
#ifdef CONFIG_GPON_OMCI_DIAG
	if (net_ratelimit())
		netdev_info(ep->ndev, "a frame that looks like OMCI, not trapped as OMCI, forwarded: port %u reason %u %*ph\n",
			    luna_gmac_rx_src_port(ep->swm->rx_layout, d->opts3),
			    luna_gmac_rx_reason(ep->swm->rx_layout, d->opts2, d->opts3),
			    (int)min_t(u32, fr->len, 16), data + fr->off);
#endif
	return false;
}

/* The slot just taken gives its place to a buffer `armed` slots on: the window moves by one (the same slot when the
 * window is the whole ring). */
static void luna_rxq_rearm(struct luna_rxq *q, unsigned int i, struct sk_buff *skb, dma_addr_t dma)
{
	unsigned int j = eth_rx_window_slot(i, q->rw.armed, q->n);

	if (j != i)
		q->skb[i] = NULL;
	luna_rx_arm(q->ring, q->skb, q->buf_dma, j, q->n, RX_BUF_SIZE, skb, dma);
}

static int luna_eth_rx(struct luna_eth *ep, struct luna_rxq *q, int budget,
			   bool napi_ctx)
{
	struct net_device *ndev = ep->ndev;
	int rx_done = 0;

	while (rx_done < budget) {
		unsigned int i = q->head;
		u32 opts1 = q->ring[i].opts1;
		struct sk_buff *skb, *fresh;
		dma_addr_t fresh_dma;
		struct eth_rx_frame fr = { 0 };
		enum eth_rx_verdict geom;

		if (opts1 & D_OWN)		/* still HW-owned: nothing more */
			break;
		ep->dbg_filled++;		/* HW handed this descriptor back */

		/* Secure the REPLACEMENT before consuming the frame: every arm
		 * dev/MEASURED-luna_eth.c.md sec 70. */
		fresh = luna_rx_alloc(ndev, ep->dev, RX_BUF_SIZE, &fresh_dma);
		if (!fresh) {
			ndev->stats.rx_dropped++;
			luna_rxq_rearm(q, i, q->skb[i], q->buf_dma[i]);
			q->head = (i + 1) % q->n;
			rx_done++;
			continue;
		}

		/* A frame longer than one buffer spans several descriptors; only a
		 * whole one (first AND last segment) is a frame, as stock drops the rest. */
		geom = eth_rx_frame_geom(&luna_eth_rx_layout, eth_rx_dlen(opts1, RXD_LEN_MASK),
					 (opts1 & (D_FS | D_LS)) == (D_FS | D_LS), &fr);
		luna_chip_rx_seen(ep, &q->ring[i], fr.len);
		skb = q->skb[i];
		dma_unmap_single(ep->dev, q->buf_dma[i], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);
		luna_eth_rx_dump(ep, &q->ring[i], skb->data, fr.len, geom);

		if (geom != ETH_RX_OK) {
			ndev->stats.rx_length_errors++;
			ndev->stats.rx_errors++;
			dev_kfree_skb_any(skb);
		/* DS-OMCI classifier + BOTH-ends length guard: hoisted to ...
		 * dev/MEASURED-luna_eth.c.md sec 71. */
		} else if (luna_omci_taken(ep, &q->ring[i], skb->data, &fr)) {
			/* DS OMCI on the OMCC. Capture for /proc, then hand the raw G.988
			 * message (prefix stripped) to the responder. */
			ep->dbg_omci_rx++;
			ep->dbg_omci_rxlen = fr.len;
			memcpy(ep->dbg_omci_rxbuf, skb->data + fr.off,
			       min_t(unsigned int, fr.len, sizeof(ep->dbg_omci_rxbuf)));
			luna_eth_omci_input(ep, skb->data + fr.off, fr.len);
			dev_kfree_skb_any(skb);
		} else if (luna_gmac_rx_frame_bad(opts1, RXD_CRCERR | RXD_RCDF,
						 fr.off + fr.len, fr.off,
						 RX_BUF_SIZE)) {
			/* bad-frame verdict hoisted to flowcore: completes the
			 * RX classification trio fuzz_rx.c drives on x86 */
			ndev->stats.rx_errors++;
			ep->dbg_err++;
			dev_kfree_skb_any(skb);
		} else {
			struct net_device *rdev = ndev;	/* receive netdev: eth0, or gpon0 for PON-port WAN frames */

			ep->dbg_good++;
			/* Capture the raw frame (prefix included) for /proc diag; the frame itself
			 * is what luna_eth_rx_layout says, no prefix and no FCS. */
			ep->dbg_rxlen = fr.len;
			memcpy(ep->dbg_rxbuf, skb->data,
			       min_t(unsigned int, fr.off + fr.len, sizeof(ep->dbg_rxbuf)));
			eth_rx_frame_put(skb, &fr);
			/* opts3 src_port_num [19:16] = ingress port. WAN demux: ...
			 * dev/MEASURED-luna_eth.c.md sec 131. */
			{
				const u8 *dst = skb->data;
				unsigned int sp = luna_gmac_rx_src_port(ep->swm->rx_layout,
						       q->ring[i].opts3);

				/* Route drained WAN frames (and any unicast to the gpon0 MAC) ...
				 * dev/MEASURED-luna_eth.c.md sec 73. */
				if (ep->wan_ndev && luna_chip_rx_is_wan(ep, q->ring[i].opts3, dst))
					rdev = ep->wan_ndev;
				else
					ep->host_port = sp;
				/* DS-OFFER diag (find why the WAN DHCP OFFER misses gpon0):
				 * log any DHCP-to-client (UDP dst port 68) DS frame + where
				 * it routed. dst = eth hdr; [12:13]=ethertype 0x0800,
				 * [23]=IP proto 0x11(UDP), [36:37]=UDP dst port 0x0044(68). */
				if (IS_ENABLED(CONFIG_LUNA_DEBUG) &&
				    fr.len >= 38 &&
				    dst[12] == 0x08 && dst[13] == 0x00 &&
				    dst[23] == 0x11 &&
				    dst[36] == 0x00 && dst[37] == 0x44)
					pr_info("rtl9602c-eth: DHCP-DS sp=%u opts3=%08x dst=%pM -> %s\n",
						sp, q->ring[i].opts3, dst,
						rdev == ep->wan_ndev ? "gpon0" : "host");
			}
			if (luna_chip_rx_consume(ep, skb, rdev, q->ring[i].opts3))
				goto rx_rearm;
			if (rdev == ep->wan_ndev) {
				luna_vlan_note(GPON_VLAN_RX, skb);
				luna_mcast_untag(skb, rdev);
			}
			if (!netif_running(rdev)) {
				rdev->stats.rx_dropped++;
				dev_kfree_skb_any(skb);
				goto rx_rearm;
			}
			luna_rx_len_note(skb->data, skb->len, rdev == ep->wan_ndev);
			skb->protocol = eth_type_trans(skb, rdev);
			skb->ip_summed = luna_chip_rx_csum && (rdev->features & NETIF_F_RXCSUM) &&
					 eth_rx_csum_ok(luna_chip_rx_csum, opts1) ?
					 CHECKSUM_UNNECESSARY : CHECKSUM_NONE;
			rdev->stats.rx_packets++;
			rdev->stats.rx_bytes += fr.len;
			/* NAPI poll context: napi_gro_receive, not netif_rx -- which ...
			 * dev/MEASURED-luna_eth.c.md sec 75. */
			if (napi_ctx)
				napi_gro_receive(&ep->napi, skb);
			else
				netif_rx(skb);
		}
rx_rearm:
		/* hand the buffer secured above to HW, at the window end */
		luna_rxq_rearm(q, i, fresh, fresh_dma);
		q->head = (i + 1) % q->n;
		rx_done++;
	}
	return rx_done;
}

static void luna_eth_tx_reclaim(struct luna_eth *ep)
{
	while (ep->tx_dirty != ep->tx_head) {
		unsigned int i = tx_slot(ep, ep->tx_dirty);

		if (ep->tx_ring[i].opts1 & D_OWN)	/* not sent yet */
			break;
		dma_unmap_single(ep->dev, ep->tx_buf_dma[i], ep->tx_buf_len[i],
				 DMA_TO_DEVICE);
		dev_consume_skb_any(ep->tx_skb[i]);
		ep->tx_skb[i] = NULL;
		ep->tx_dirty++;
	}
	if (netif_queue_stopped(ep->ndev) &&
	    !luna_gmac_tx_ring_full(ep->tx_head, ep->tx_dirty,
				     TX_RING_SIZE, OMCI_RESV))
		netif_wake_queue(ep->ndev);
}

/* The GMAC TX DMA is a self-polling sequential fetch engine: ...
 * dev/MEASURED-luna_eth.c.md sec 76. */
static void luna_eth_tx_rekick(struct luna_eth *ep)
{
	bool parked = false;

	/* Park detection keys on the OLDEST PENDING descriptor (dirty ...
	 * dev/MEASURED-luna_eth.c.md sec 77. */
	if (ep->tx_head != ep->tx_dirty &&
	    (ep->tx_ring[tx_slot(ep, ep->tx_dirty)].opts1 & D_OWN)) {
		parked = true;
		ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));
	}
	if (omci_tx_ring && ep->otx_head != ep->otx_dirty &&
	    (ep->otx_ring[otx_slot(ep, ep->otx_dirty)].opts1 & D_OWN)) {
		unsigned int h;
		u32 dmask;

		parked = true;
		/* 0xff = the stock derivation: the re-kick deliberately keeps
		 * ignoring the omci_doorbell_bit debug override, exactly as
		 * before the hoist (only the xmit-time kick honours it). */
		if (luna_gmac_omci_doorbell(omci_tx_ring, 0xff, &h, &dmask))
			ep_wr(ep, R_IO_CMD1, ep_rd(ep, R_IO_CMD1) | dmask);
		else
			ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | dmask);
	}

	/* Park watchdog. A briefly-parked ring is normal (the ...
	 * dev/MEASURED-luna_eth.c.md sec 78. */
	if (!parked || ep->closing) {
		ep->stall_since = 0;
	} else if (!ep->stall_since ||
		   ep->tx_dirty + ep->otx_dirty != ep->stall_lastdirty) {
		ep->stall_since = jiffies;
		ep->stall_lastdirty = ep->tx_dirty + ep->otx_dirty;
	}
}

/* The GMAC pauses its switch port while it counts 16 or fewer slots armed from its position to the CPU tail
 * (TH_ON_VAL): the tail follows the CPU once per poll, as stock writes it after each arm. A tail left at the
 * ring end paused by ring position instead (X100DG live 0xff103000: tail 255 for ever). */
static void luna_rx_tail_publish(struct luna_eth *ep, struct luna_rxq *q, bool force)
{
	unsigned int tail = luna_gmac_rx_tail(q->head, q->rw.armed, q->n);

	if (tail == q->tail && !force)
		return;
	q->tail = tail;
	ep_wr(ep, q->tail_reg, q == ep->rxq ? luna_gmac_rxdesnum_pack(tail, TH_ON_VAL, TH_OFF_VAL) : tail);
}

static void luna_rx_tails_publish(struct luna_eth *ep)
{
	unsigned int k;

	for (k = 0; k < ep->n_rxq; k++)
		luna_rx_tail_publish(ep, &ep->rxq[k], false);
}

/* The window engine's ops on the data ring (ctx = ep): called per poll, per tick or from the worker, never per frame. */
static u32 luna_rxw_hw_pos(void *ctx)
{
	return ep_rd(ctx, R_RxCDO) >> 16;
}

static bool luna_rxw_arm(void *ctx, unsigned int slot, struct eth_rx_window_buf b)
{
	struct luna_rxq *q = &((struct luna_eth *)ctx)->rxq[0];

	if (WARN_ON_ONCE(q->skb[slot]))
		return false;
	luna_rx_arm(q->ring, q->skb, q->buf_dma, slot, q->n, RX_BUF_SIZE, b.buf, b.dma);
	return true;
}

static void luna_rxw_disarm(void *ctx, unsigned int slot, struct eth_rx_window_buf *b)
{
	struct luna_rxq *q = &((struct luna_eth *)ctx)->rxq[0];

	q->ring[slot].opts1 = slot == q->n - 1 ? D_EOR : 0;	/* not OWN: the GMAC stops before it */
	b->buf = q->skb[slot];
	b->dma = q->buf_dma[slot];
	q->skb[slot] = NULL;
}

static int luna_rxw_alloc(void *ctx, struct eth_rx_window_buf *b)
{
	struct luna_eth *ep = ctx;

	b->buf = luna_rx_alloc_gfp(ep->ndev, ep->dev, RX_BUF_SIZE, &b->dma, GFP_KERNEL | __GFP_NOWARN);
	return b->buf ? 0 : -ENOMEM;
}

static void luna_rxw_free(void *ctx, struct eth_rx_window_buf b)
{
	dma_unmap_single(((struct luna_eth *)ctx)->dev, b.dma, RX_BUF_SIZE, DMA_FROM_DEVICE);
	dev_kfree_skb_any(b.buf);
}

static const struct eth_rx_window_ops luna_rxw_ops = {
	.hw_pos = luna_rxw_hw_pos, .arm = luna_rxw_arm, .disarm = luna_rxw_disarm,
	.alloc = luna_rxw_alloc, .free = luna_rxw_free,
};

static void luna_rxw_kick_poll(struct luna_eth *ep)
{
	if (ep->irq > 0) {
		local_bh_disable();
		napi_schedule(&ep->napi);
		local_bh_enable();
	}
}

static void luna_rxw_work(struct work_struct *work)
{
	struct luna_eth *ep = container_of(work, struct luna_eth, rxw_work);

	if (eth_rx_window_ring_work(&ep->rxq[0].rw, &luna_rxw_ops, ep))
		luna_rxw_kick_poll(ep);
}

/* From the driver's timers (the engine decides once per ETH_RX_WINDOW_TICK_MS), then whoever acts is woken. */
static void luna_rxw_tick(struct luna_eth *ep)
{
	struct eth_rx_window_ring *r = &ep->rxq[0].rw;
	unsigned int kick;

	spin_lock(&ep->rxw_lock);
	kick = eth_rx_window_ring_tick(r, jiffies_to_msecs(jiffies), READ_ONCE(rx_window_lab));
	spin_unlock(&ep->rxw_lock);
	if (kick & ETH_RX_WINDOW_KICK_WORK)
		schedule_work(&ep->rxw_work);
	if ((kick & ETH_RX_WINDOW_KICK_POLL) && ep->irq > 0)
		napi_schedule(&ep->napi);
}

/* Once per poll: the frames the GMAC dropped for want of an armed descriptor (RDUMISSPKT) -> rx_missed_errors. */
static void luna_rx_missed_sample(struct luna_eth *ep)
{
	ep->ndev->stats.rx_missed_errors += luna_gmac_cnt16_delta(ep_rd(ep, R_TXUNDRN) & 0xffff, &ep->rdu_last);
}

static bool luna_eth_rx_waiting(const struct luna_eth *ep)
{
	return luna_rx_ready(ep->rxq[0].ring, ep->rxq[0].head) ||
	       (ep->n_rxq > 1 && luna_rx_ready(ep->rxq[1].ring, ep->rxq[1].head));
}

/* NAPI poll: drain RX up to `budget`, then (under tx_lock) ...
 * dev/MEASURED-luna_eth.c.md sec 79. */
static int luna_eth_napi_poll(struct napi_struct *napi, int budget)
{
	struct luna_eth *ep = container_of(napi, struct luna_eth, napi);
	unsigned long flags;
	int work;

	ep->dbg_poll++;
	luna_rx_missed_sample(ep);
	eth_rx_window_ring_sample(&ep->rxq[0].rw, &luna_rxw_ops, ep, ep->rxq[0].head, ep->rxq[0].n);
	/* control first, but at most half the budget: a trapped flood cannot starve the data */
	work = ep->n_rxq > 1 ? luna_eth_rx(ep, &ep->rxq[1], budget / 2, true) : 0;
	ep->dbg_ctl_rx += work;
	work += luna_eth_rx(ep, &ep->rxq[0], budget - work, true);	/* inside napi->poll */
	if (eth_rx_window_ring_resize(&ep->rxq[0].rw, &luna_rxw_ops, ep, ep->rxq[0].head, ep->rxq[0].n))
		schedule_work(&ep->rxw_work);
	luna_rx_tails_publish(ep);
	spin_lock_irqsave(&ep->tx_lock, flags);
	luna_eth_tx_reclaim(ep);
	luna_eth_omci_reclaim(ep);
	luna_eth_tx_rekick(ep);	/* un-park a stalled TX ring */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (work < budget && napi_complete_done(napi, work)) {
		/* Re-arm. W1C-ack any status latched while masked FIRST so we do
		 * not immediately re-fire on a stale bit, THEN set the mask bits. */
		iowrite16(ioread16(ep->base + R_ISR), ep->base + R_ISR);
		ep_wr(ep, R_ISR1, ep_rd(ep, R_ISR1));
		iowrite16(ioread16(ep->base + R_IMR) | IMR_RX_BITS, ep->base + R_IMR);
		ep_wr(ep, R_IMR0, ep_rd(ep, R_IMR0) | IMR0_TX_BITS);
		if (luna_eth_rx_waiting(ep)) {	/* its bit went to the ack */
			ep->dbg_rx_repoll++;
			napi_schedule(napi);
		}
	}
	return work;
}

/* Slow TX-unpark backstop (IRQ-driven mode). The GMAC TX DMA ...
 * dev/MEASURED-luna_eth.c.md sec 80. */
static void luna_eth_rekick_timer(struct timer_list *t)
{
	struct luna_eth *ep = timer_container_of(ep, t, poll_timer);
	unsigned long flags;
	u32 queued, sent;

	spin_lock_irqsave(&ep->tx_lock, flags);
	luna_eth_tx_reclaim(ep);
	luna_eth_omci_reclaim(ep);
	luna_eth_tx_rekick(ep);
	queued = ep->tx_head - ep->tx_dirty + ep->otx_head - ep->otx_dirty;
	sent = ep->tx_dirty + ep->otx_dirty - ep->tx_dropped;
	spin_unlock_irqrestore(&ep->tx_lock, flags);
	if (luna_mib_nic_tx_sample(ep->ndev, !ep->closing, queued, sent))
		schedule_work(&ep->recover_work);	/* one GMAC reset, the ring realigned */
	if (eth_ring_watch_rx(&luna_ring_watch, ep->ndev, luna_eth_rx_waiting(ep), ep->dbg_filled))
		napi_schedule(&ep->napi);
	if (ep->wan_ndev && luna_mib_pon_egress_sample(ep->wan_ndev, (u32)ep->wan_ndev->stats.tx_packets))
		luna_eth_msr_apply(ep, true);
	luna_rxw_tick(ep);
	if (!eth_core_recovery_busy(&ep->sw_recovery) &&
	    luna_mib_port_egress_sample(ep->ndev, !luna_eth_rx_waiting(ep)))
		eth_core_recovery_ask(&ep->sw_recovery);
	mod_timer(&ep->poll_timer, jiffies + REKICK_INTERVAL);
}

/* Legacy pure-poll fallback (ep->irq <= 0): the original 2ms ...
 * dev/MEASURED-luna_eth.c.md sec 132. */
static void luna_eth_poll(struct timer_list *t)
{
	struct luna_eth *ep = timer_container_of(ep, t, poll_timer);
	unsigned long flags;

	ep->dbg_poll++;
	luna_rx_missed_sample(ep);
	eth_rx_window_ring_sample(&ep->rxq[0].rw, &luna_rxw_ops, ep, ep->rxq[0].head, ep->rxq[0].n);
	if (ep->n_rxq > 1)
		luna_eth_rx(ep, &ep->rxq[1], ep->rxq[1].n, false);
	luna_eth_rx(ep, &ep->rxq[0], ep->rxq[0].n, false);	/* timer callback, NOT napi->poll */
	if (eth_rx_window_ring_resize(&ep->rxq[0].rw, &luna_rxw_ops, ep, ep->rxq[0].head, ep->rxq[0].n))
		schedule_work(&ep->rxw_work);
	luna_rx_tails_publish(ep);
	luna_rxw_tick(ep);
	spin_lock_irqsave(&ep->tx_lock, flags);
	luna_eth_tx_reclaim(ep);	/* under tx_lock (races the OMCI inject's locked reclaim on shared ring 0) */
	luna_eth_omci_reclaim(ep);
	luna_eth_tx_rekick(ep);	/* TDU-style keep-alive: un-park a stalled TX ring */
	spin_unlock_irqrestore(&ep->tx_lock, flags);
	mod_timer(&ep->poll_timer, jiffies + POLL_INTERVAL);
}

/* A GMAC restarted at slot 0 finds the window there: its buffers move with it and are armed again (what they
 * held is dropped, as a restart always did). */
static void luna_rxq_restart(struct luna_rxq *q)
{
	unsigned int i;

	eth_rx_window_rotate(q->skb, q->n, sizeof(*q->skb), q->head);
	eth_rx_window_rotate(q->buf_dma, q->n, sizeof(*q->buf_dma), q->head);
	for (i = 0; i < q->n; i++) {
		if (i < q->rw.armed) {
			q->ring[i].addr = q->buf_dma[i] | DMA_BUS_WINDOW;
			luna_rx_rearm(q->ring, i, q->n, RX_BUF_SIZE);
		} else {
			q->ring[i].opts1 = i == q->n - 1 ? D_EOR : 0;
		}
	}
	q->head = 0;
}

static int luna_eth_alloc_rings(struct luna_eth *ep)
{
	unsigned int i, k;

	for (i = 0; i < ep->n_rxq; i++) {
		struct luna_rxq *q = &ep->rxq[i];

		q->head = 0;
		q->ring = dma_alloc_coherent(ep->dev, q->n * sizeof(struct rx_desc), &q->ring_dma,
					     GFP_KERNEL);
		if (!q->ring)
			return -ENOMEM;
	}
	ep->tx_ring = dma_alloc_coherent(ep->dev,
			TX_RING_SIZE * sizeof(struct tx_desc),
			&ep->tx_ring_dma, GFP_KERNEL);
	/* Dedicated US-OMCI TX ring (the OMCC ring, default ring 4). */
	ep->otx_ring = dma_alloc_coherent(ep->dev,
			OTX_RING_SIZE * sizeof(struct tx_desc),
			&ep->otx_ring_dma, GFP_KERNEL);
	/* Idle filler for the unused HW TX rings (see struct comment). */
	if (luna_chip_gmac.tx_dummy_rings)
		ep->dummy_ring = dma_alloc_coherent(ep->dev,
			DUMMY_RING_SIZE * sizeof(struct tx_desc),
			&ep->dummy_ring_dma, GFP_KERNEL);
	if (!ep->tx_ring || !ep->otx_ring || (luna_chip_gmac.tx_dummy_rings && !ep->dummy_ring))
		return -ENOMEM;

	ep->tx_head = ep->tx_dirty = 0;
	ep->otx_head = ep->otx_dirty = 0;
	ep->tx_rot = ep->otx_rot = 0;	/* fresh rings: HW walk starts at slot 0 */
	ep->omci_r0_last_slot = -1;	/* no OMCI on shared LAN ring 0 yet */
	ep->host_port = 0xff;		/* uplink port unknown until first RX */
	for (i = 0; i < TX_RING_SIZE; i++) {
		ep->tx_ring[i].opts1 = (i == TX_RING_SIZE - 1) ? D_EOR : 0;
		ep->tx_skb[i] = NULL;
	}
	for (i = 0; i < OTX_RING_SIZE; i++) {
		/* EOR (wrap) in word0 on the last slot; OWN (opts1 bit31) clear so
		 * the slot starts CPU-owned (idle) until the first OMCI submit. */
		ep->otx_ring[i].opts1 = (i == OTX_RING_SIZE - 1) ? D_EOR : 0;
		ep->otx_ring[i].opts2 = 0;
		ep->otx_skb[i] = NULL;
	}
	for (i = 0; ep->dummy_ring && i < DUMMY_RING_SIZE; i++) {
		/* OWN clear (opts1 bit31 = 0) => always CPU-owned => the TX engine
		 * idles on this ring; EOR on the last slot. Never written again. */
		ep->dummy_ring[i].opts1 = (i == DUMMY_RING_SIZE - 1) ? D_EOR : 0;
		ep->dummy_ring[i].opts2 = 0;
	}
	for (k = 0; k < ep->n_rxq; k++) {
		struct luna_rxq *q = &ep->rxq[k];

		/* the data ring opens at its floor, the control ring is fixed */
		eth_rx_window_init(&q->rw.w, q->n, k ? q->n : ep->rx_floor, jiffies_to_msecs(jiffies));
		q->rw.armed = q->rw.w.target;
		for (i = 0; i < q->n; i++)
			if (i >= q->rw.armed)
				q->ring[i].opts1 = i == q->n - 1 ? D_EOR : 0;
			else if (luna_eth_refill(ep, q, i))
				return -ENOMEM;
	}
	return 0;
}

static void luna_eth_free_rings(struct luna_eth *ep)
{
	struct luna_rxq *q;
	unsigned int i;

	for (q = ep->rxq; q < ep->rxq + ep->n_rxq; q++) {
		for (i = 0; i < q->n; i++) {
			if (q->skb[i]) {
				dma_unmap_single(ep->dev, q->buf_dma[i], RX_BUF_SIZE, DMA_FROM_DEVICE);
				dev_kfree_skb_any(q->skb[i]);
				q->skb[i] = NULL;
			}
		}
		eth_rx_window_ring_release(&q->rw, &luna_rxw_ops, ep);
		if (q->ring)
			dma_free_coherent(ep->dev, q->n * sizeof(struct rx_desc), q->ring, q->ring_dma);
		q->ring = NULL;
	}
	for (i = 0; i < TX_RING_SIZE; i++) {
		if (ep->tx_skb[i]) {
			dma_unmap_single(ep->dev, ep->tx_buf_dma[i],
					 ep->tx_buf_len[i], DMA_TO_DEVICE);
			dev_kfree_skb_any(ep->tx_skb[i]);
			ep->tx_skb[i] = NULL;
		}
	}
	for (i = 0; i < OTX_RING_SIZE; i++) {
		if (ep->otx_skb[i]) {
			dma_unmap_single(ep->dev, ep->otx_buf_dma[i],
					 ep->otx_buf_len[i], DMA_TO_DEVICE);
			dev_kfree_skb_any(ep->otx_skb[i]);
			ep->otx_skb[i] = NULL;
		}
	}
	if (ep->tx_ring)
		dma_free_coherent(ep->dev, TX_RING_SIZE * sizeof(struct tx_desc),
				  ep->tx_ring, ep->tx_ring_dma);
	if (ep->otx_ring)
		dma_free_coherent(ep->dev, OTX_RING_SIZE * sizeof(struct tx_desc),
				  ep->otx_ring, ep->otx_ring_dma);
	if (ep->dummy_ring)
		dma_free_coherent(ep->dev, DUMMY_RING_SIZE * sizeof(struct tx_desc),
				  ep->dummy_ring, ep->dummy_ring_dma);
	ep->tx_ring = NULL;
	ep->otx_ring = NULL;
	ep->dummy_ring = NULL;
}

/* GMAC0 hard ISR (IRQF_SHARED, INTC input 26). Snapshot the ...
 * dev/MEASURED-luna_eth.c.md sec 81. */
static irqreturn_t luna_eth_isr(int irq, void *dev_id)
{
	struct luna_eth *ep = dev_id;
	u16 isr  = ioread16(ep->base + R_ISR) & IMR_RX_BITS;
	u32 isr1 = ep_rd(ep, R_ISR1) & IMR0_TX_BITS;

	if (!isr && !isr1)
		return IRQ_NONE;

	/* Mask first so the level line drops, then ack (W1C), then schedule. */
	iowrite16(ioread16(ep->base + R_IMR) & ~IMR_RX_BITS, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, ep_rd(ep, R_IMR0) & ~IMR0_TX_BITS);
	if (isr)
		iowrite16(isr, ep->base + R_ISR);
	if (isr1)
		ep_wr(ep, R_ISR1, isr1);
	napi_schedule(&ep->napi);
	return IRQ_HANDLED;
}

/* Halt the GMAC DMA + IRQ machinery (the stock stop sequence: ...
 * dev/MEASURED-luna_eth.c.md sec 82. */
static void luna_hw_stop(struct luna_eth *ep)
{
	luna_eth_hw_stop(ep->base);
}

/* RX ring 0 from its first slot: base, CPU tail and flow-control thresholds, size; RDUMISSPKT's baseline. */
static void luna_rx_ring0_program(struct luna_eth *ep)
{
	ep_wr(ep, R_RxFDP, ep->rxq[0].ring_dma | DMA_BUS_WINDOW);
	luna_rx_tail_publish(ep, &ep->rxq[0], true);
	ep_wr(ep, R_RxCDO, luna_gmac_rxcdo_pack(ep->rxq[0].n));
	ep->rdu_last = ep_rd(ep, R_TXUNDRN) & 0xffff;
}

static void luna_hw_program(struct luna_eth *ep)
{
	struct net_device *ndev = ep->ndev;
	unsigned int k, oring = luna_gmac_omci_hwring(omci_tx_ring);

	iowrite8(luna_chip_gmac.cmd, ep->base + R_CMD);
	ep_wr(ep, R_TCR, luna_chip_gmac.tcr);
	ep_wr(ep, R_RCR, RCR_STOCK);
	/* Stock O5 golden CONFIG = 0x21000000 (Rff 2k + rx-mring int split);
	 * after a true reset the split bit must be set by us, the per-ring
	 * IMR0/ISR1 model the driver already uses assumes it. */
	ep_wr(ep, R_CONFIG, 0x21000000);
	/* ⚠ R_RXOKMUL IS A MIB COUNTER WORD ([RXOKMUL:TXABT]), NOT A ...
	 * dev/MEASURED-luna_eth.c.md sec 133. */
	if (luna_chip_gmac.rxokmul_seed)
		ep_wr(ep, R_RXOKMUL, 0x010c0000);	/* stock O5 counter snapshot */
	/* cpu-tag ADD engine: off-then-on edge re-latches it (see open notes) */
	luna_chip_cpu_tag_arm(ep);

	/* Ring pointers, programmed while IO_CMD==0 (stock order; CDO is
	 * writable only in this stopped state). */
	ep_wr(ep, R_TxFDP1, ep->tx_ring_dma | DMA_BUS_WINDOW);
	iowrite16(0, ep->base + R_TxCDO1);
	if (omci_tx_ring != 0) {
		ep_wr(ep, R_TxFDP(oring), ep->otx_ring_dma | DMA_BUS_WINDOW);
		iowrite16(0, ep->base + R_TxCDO(oring));
	}
	for (k = 1; luna_chip_gmac.tx_dummy_rings && k <= 4; k++) {	/* idle dummy on the unused HW rings */
		if (k == oring)
			continue;
		ep_wr(ep, R_TxFDP(k), ep->dummy_ring_dma | DMA_BUS_WINDOW);
		iowrite16(0, ep->base + R_TxCDO(k));
	}
	luna_rx_ring0_program(ep);
	if (ep->n_rxq > 1) {
		ep_wr(ep, R_RxMRingCfg(1), ep->rxq[1].ring_dma | DMA_BUS_WINDOW);
		ep_wr(ep, R_RxMRingCDO(1), CTL_RING_SIZE - 1);
		luna_rx_tail_publish(ep, &ep->rxq[1], true);
		ep_wr(ep, R_RxMRingCfg(1) + 12, (TH_ON_VAL << 16) | TH_OFF_VAL);
	}
	ep->ctl_ring_on = ep->n_rxq > 1;
	luna_rx_route(ep);

	luna_eth_msr_apply(ep, false);
	luna_eth_set_hwaddr(ep, ndev->dev_addr);	/* IDR wiped by the reset */
	iowrite32(0xffffffff, ep->base + R_MAR0);		/* MAR0 */
	iowrite32(0xffffffff, ep->base + R_MAR4);		/* MAR4 */

	/* The enable edge: IO_CMD1 first, IO_CMD last (stock start order). */
	ep_wr(ep, R_IO_CMD1, IOCMD1_STOCK | (ep->n_rxq > 1 ? IOCMD1_RX_RINGS(2) : 0));
	ep_wr(ep, R_IO_CMD, IOCMD_STOCK);

	iowrite16(0xffff, ep->base + R_ISR);		/* ack anything latched */
	ep_wr(ep, R_ISR1, 0xffffffff);
	iowrite16(IMR_RX_BITS, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, IMR0_TX_BITS);
}

/* Stock GMAC reset path: GMAC0 IP-block power-cycle. The block
 * is UNREADABLE while gated (MMIO would bus-abort), so callers must fence off
 * the ISR/diag readers first. */
/* The body and the hang warning are the FAMILY's (luna_eth_regs.h). */
static void luna_ipsel_cycle(void)
{
	luna_ipsel_cycle_gmac0();
}

/* TX-DMA park recovery (process context, scheduled by the ...
 * dev/MEASURED-luna_eth.c.md sec 84. */
static void luna_eth_recover_work(struct work_struct *work)
{
	struct luna_eth *ep = container_of(work, struct luna_eth,
					       recover_work);
	struct net_device *ndev = ep->ndev;
	unsigned long flags;
	unsigned int i, k;
	bool use_rst = !!recover_rst;	/* snapshot: param is runtime-writable */

	if (ep->closing || !ep->hw_up)
		return;

	luna_gpon_nic_reset_begin();
	ep->in_recovery = true;
	netdev_warn(ndev,
		    "TX-DMA parked (head=%u dirty=%u otx=%u/%u cdo0=%u txok=%u IO_CMD=%08x ISR1=%08x): GMAC %s recovery #%u\n",
		    ep->tx_head, ep->tx_dirty, ep->otx_head, ep->otx_dirty,
		    ioread16(ep->base + R_TxCDO(0)), ep_rd(ep, R_TXOKCNT) >> 16,
		    ep_rd(ep, R_IO_CMD), ep_rd(ep, R_ISR1),
		    use_rst ? "reset" : "IP-block power-cycle", ep->dbg_tx_recover + 1);

	netif_stop_queue(ndev);
	napi_disable(&ep->napi);
	timer_delete_sync(&ep->poll_timer);
	if (!use_rst && ep->irq > 0)
		disable_irq(ep->irq);	/* gated block MMIO would bus-abort
					 * (ipsel path only; RST is GMAC-local) */

	spin_lock_irqsave(&ep->tx_lock, flags);
	luna_hw_stop(ep);
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (use_rst) {
		/* CMD.RST core soft-reset: microseconds, no IP-block gating.
		 * Self-clears on completion; fall through to the reprogram
		 * either way (a timeout leaves us no worse than before). */
		int n;

		iowrite8(0x0A | 0x01, ep->base + R_CMD);
		for (n = 0; n < 1000 && (ioread8(ep->base + R_CMD) & 1); n++)
			udelay(1);
	} else {
		luna_ipsel_cycle();
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	/* Drop everything in flight and re-arm both rings from scratch; the engine restarts at CDO 0, which
	 * luna_tx_align() maps onto the driver's counters. */
	ep->tx_dropped += ep->tx_head - ep->tx_dirty + ep->otx_head - ep->otx_dirty;
	while (ep->tx_dirty != ep->tx_head) {
		i = tx_slot(ep, ep->tx_dirty);
		if (ep->tx_skb[i]) {
			dma_unmap_single(ep->dev, ep->tx_buf_dma[i],
					 ep->tx_buf_len[i], DMA_TO_DEVICE);
			dev_kfree_skb_any(ep->tx_skb[i]);
			ep->tx_skb[i] = NULL;
		}
		ep->tx_dirty++;
	}
	while (ep->otx_dirty != ep->otx_head) {
		i = otx_slot(ep, ep->otx_dirty);
		if (ep->otx_skb[i]) {
			dma_unmap_single(ep->dev, ep->otx_buf_dma[i],
					 ep->otx_buf_len[i], DMA_TO_DEVICE);
			dev_kfree_skb_any(ep->otx_skb[i]);
			ep->otx_skb[i] = NULL;
		}
		ep->otx_dirty++;
	}
	for (i = 0; i < TX_RING_SIZE; i++)
		ep->tx_ring[i].opts1 = (i == TX_RING_SIZE - 1) ? D_EOR : 0;
	for (i = 0; i < OTX_RING_SIZE; i++)
		ep->otx_ring[i].opts1 = (i == OTX_RING_SIZE - 1) ? D_EOR : 0;
	for (k = 0; k < ep->n_rxq; k++)
		luna_rxq_restart(&ep->rxq[k]);
	wmb();				/* ring state before the engine restart */
	luna_hw_program(ep);
	luna_tx_align(ep);		/* the engine's slot 0 = the driver's next counter */
	ep->dbg_tx_recover++;
	ep->stall_since = 0;
	spin_unlock_irqrestore(&ep->tx_lock, flags);
	if (!use_rst && luna_chip_gmac.pbo_reinit)
		gpon_pbo_init();	/* an IP-block reset owes its PBO re-init (sec 96) */

	if (!use_rst && ep->irq > 0)
		enable_irq(ep->irq);
	ep->in_recovery = false;
	napi_enable(&ep->napi);
	mod_timer(&ep->poll_timer, jiffies + 1);
	netif_wake_queue(ndev);
	luna_gpon_nic_reset_end();
}

static int luna_eth_hw_start(struct luna_eth *ep)
{
	struct net_device *ndev = ep->ndev;
	int ret;

	ret = luna_eth_alloc_rings(ep);
	if (ret) {
		luna_eth_free_rings(ep);
		return ret;
	}
	luna_gpon_nic_reset_begin();
	ep->closing = false;
	ep->stall_since = 0;

	/* THE FULL SWITCH BRING-UP IS A ONE-TIME COST, NOT A PER-ifup ...
	 * dev/MEASURED-luna_eth.c.md sec 92. */
	if (gmac_reset || !luna_chip_gmac.uboot_inherit) {
		/* Stock-faithful cold start (the TX-park fix): halt the ...
		 * dev/MEASURED-luna_eth.c.md sec 93. */
		luna_hw_stop(ep);
		luna_ipsel_cycle();
		/* re-establish the GMAC<->switch sync the reset tore down (the
		 * catch-22 breaker) -- and it is the FABRIC, so a failure is not
		 * something to carry on past. */
		ret = luna_chip_sw_bringup(ep);
		if (ret)
			goto fail;
		/* Faithful full stock-equivalent datapath init, in stock ...
		 * dev/MEASURED-luna_eth.c.md sec 135. */
		luna_hw_program(ep);
		luna_tx_align(ep);	/* fresh engine: its slot 0 = the driver's next counter */
		if (ep->txgo)	/* stock's value of this one bit; the boot loader's training stays */
			iowrite32(ioread32(ep->txgo + MEMCTL_MCR) | BIT(19), ep->txgo + MEMCTL_MCR);
		/* Re-establish the FULL PON US/DS-NIC datapath against the ...
		 * dev/MEASURED-luna_eth.c.md sec 95. */
		if (luna_chip_gmac.pbo_reinit)
			gpon_pbo_init();
		/* Once per boot: ndo_stop never takes this down. sec 96. */
		goto hw_ready;
	}

	/* Program the ring pointers. The bootloader ORs 0x20000000 ...
	 * dev/MEASURED-luna_eth.c.md sec 97. */
	ep_wr(ep, R_TxFDP1, ep->tx_ring_dma | DMA_BUS_WINDOW);
	/* TxCDO is NOT writable on the live inherited engine — do not ...
	 * dev/MEASURED-luna_eth.c.md sec 98. */
	if (omci_tx_ring != 0) {
		unsigned int oring = luna_gmac_omci_hwring(omci_tx_ring);

		ep_wr(ep, R_TxFDP(oring), ep->otx_ring_dma | DMA_BUS_WINDOW);
		/* NOTE: a TX ring needs ONLY its TxFDP armed + the per-packet ...
		 * dev/MEASURED-luna_eth.c.md sec 99. */
	}
	/* Arm the UNUSED HW TX rings (the TxFDP gaps between the LAN ...
	 * dev/MEASURED-luna_eth.c.md sec 100. */
	{
		unsigned int k, oring = luna_gmac_omci_hwring(omci_tx_ring);

		for (k = 1; k <= 4; k++) {
			if (k == oring)
				continue;	/* OMCI ring already armed at its own base */
			ep_wr(ep, R_TxFDP(k), ep->dummy_ring_dma | DMA_BUS_WINDOW);
			iowrite16(0, ep->base + R_TxCDO(k));
		}
	}
	luna_rx_ring0_program(ep);
	/* (Reverted: a prior experiment pointed rings 1-5 at ring 0's ...
	 * dev/MEASURED-luna_eth.c.md sec 101. */
	ep_wr(ep, R_RCR, RCR_STOCK);
	ep_wr(ep, R_TCR, 0x00000C00);
	ep_wr(ep, R_CONFIG, 0x20000000);
	iowrite8(0x0A, ep->base + R_CMD);	/* CMD = RxChkSum|RxJumboSupport (keep working-RX baseline) */
	/* (RX-ring-size bytes 0x1430/0x1432/0x13f6 select a 16-entry ...
	 * dev/MEASURED-luna_eth.c.md sec 102. */
	luna_chip_cpu_tag_arm(ep);
	/* GMAC config regs that a LIVE stock ONU at O5 SETS but my ...
	 * dev/MEASURED-luna_eth.c.md sec 103. */
	ep_wr(ep, R_RXOKMUL, 0x010c0000);	/* stock O5 counter snapshot -- a MIB word, not config; see luna_hw_program() */
	ep_wr(ep, R_IMR0, IMR0_TX_BITS);	/* per-ring TX-completion IRQ mask, rings 0-5 (stock 0x3f) */
	iowrite16(IMR_RX_BITS, ep->base + R_IMR);	/* RX IRQ mask: RX_OK + RX-err + RDU (stock 0xf835) */
	luna_eth_msr_apply(ep, false);
	iowrite32(0xffffffff, ep->base + R_MAR0);	/* MAR0: accept-all-multicast */
	iowrite32(0xffffffff, ep->base + R_MAR4);	/* MAR4 */
	/* IO_CMD1 = the exact stock start value 0x323f0001, decoded ...
	 * dev/MEASURED-luna_eth.c.md sec 104. */
	ep_wr(ep, R_IO_CMD1, IOCMD1_UBOOT);	/* legacy: re-assert the inherited config */
	ep_wr(ep, R_IO_CMD, IOCMD_UBOOT);	/* full CMD_CONFIG, RX+TX DMA enable (last) */
	luna_tx_align(ep);	/* SW producer -> live engine position (CDO) */

hw_ready:
	luna_chip_sw_min_init(ep);	/* flood ingress to the CPU port */

	napi_enable(&ep->napi);
	/* Clear any IRQ status latched during bring-up before unmasking the line. */
	iowrite16(ioread16(ep->base + R_ISR), ep->base + R_ISR);
	ep_wr(ep, R_ISR1, ep_rd(ep, R_ISR1));
	if (ep->irq > 0) {
		ret = request_irq(ep->irq, luna_eth_isr, IRQF_SHARED,
				  ndev->name, ep);
		if (ret) {
			netdev_warn(ndev, "request_irq(%d) failed (%d); pure-poll fallback\n",
				    ep->irq, ret);
			ep->irq = -1;
		}
	}
	/* IRQ-driven: the ISR schedules NAPI; the timer is just a ...
	 * dev/MEASURED-luna_eth.c.md sec 136. */
	timer_setup(&ep->poll_timer,
		    (ep->irq > 0) ? luna_eth_rekick_timer : luna_eth_poll, 0);
	mod_timer(&ep->poll_timer,
		  jiffies + ((ep->irq > 0) ? REKICK_INTERVAL : POLL_INTERVAL));

	ep->hw_up = true;
	luna_gpon_nic_reset_end();
	return 0;

fail:
	/* A half-programmed fabric must not be reported as an ...
	 * dev/MEASURED-luna_eth.c.md sec 105. */
	{
		unsigned long flags;

		spin_lock_irqsave(&ep->tx_lock, flags);
		ep->closing = true;
		spin_unlock_irqrestore(&ep->tx_lock, flags);
	}
	luna_hw_stop(ep);
	luna_gpon_nic_reset_end();
	luna_eth_free_rings(ep);
	netdev_err(ndev,
		   "switch bring-up failed (%d): eth0 stays DOWN rather than up on a half-programmed fabric\n",
		   ret);
	return ret;
}

static int luna_eth_open(struct net_device *ndev)
{
	struct luna_eth *ep = netdev_priv(ndev);
	int ret;

	if (!ep->hw_up) {
		ret = luna_eth_hw_start(ep);
		if (ret)
			return ret;
		luna_mib_phy_reapply(ndev);
		luna_loop_start(ep->loop);
	}
	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;
}

/* The GMAC keeps running: the DS/US OMCI and gpon0 ride its rings, and an OLT
 * gives up on an ONU whose OMCI stops (the laptop X111W, 2026-09-30, O5 reached
 * between preinit's close and netifd's open). */
static int luna_eth_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	luna_chip_stop(netdev_priv(ndev));
	return 0;
}

/* CPU-directed TX uses a SOFTWARE DSA-style cpu-tag (mainline ...
 * dev/MEASURED-luna_eth.c.md sec 106. */
#define SW_TAG_LAN_MASK	0x7	/* forwarding mask: LAN ports 0,1,2 (CPU port = 3) */

/* @word2: the descriptor's egress steering, 0 = the switch's L2 lookup. */
static __always_inline netdev_tx_t luna_eth_xmit_word2(struct sk_buff *skb, struct net_device *ndev, u32 word2)
{
	struct luna_eth *ep = netdev_priv(ndev);
	unsigned int i;
	unsigned int len = skb->len;
	unsigned long flags;
	dma_addr_t da;
	u32 opts1;

	/* tx_lock taken first so the ring-full test precedes any skb ...
	 * dev/MEASURED-luna_eth.c.md sec 137. */
	spin_lock_irqsave(&ep->tx_lock, flags);
	if (luna_gmac_tx_ring_full(ep->tx_head, ep->tx_dirty,
				   TX_RING_SIZE, OMCI_RESV)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}
	if (len < ETH_ZLEN) {
		if (skb_padto(skb, ETH_ZLEN)) {
			spin_unlock_irqrestore(&ep->tx_lock, flags);
			return NETDEV_TX_OK;	/* skb freed by skb_padto */
		}
		len = ETH_ZLEN;
	}
#if TX_CPUTAG
	/* Prepend the software cpu-tag after DA+SA (the hardware portmask insertion
	 * is broken on this silicon — see RTL8_4 defines). */
	if (skb_cow_head(skb, RTL8_4_TAG_LEN)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	skb_push(skb, RTL8_4_TAG_LEN);
	memmove(skb->data, skb->data + RTL8_4_TAG_LEN, 2 * ETH_ALEN);
	{
		__be16 *t = (__be16 *)(skb->data + 2 * ETH_ALEN);
		t[0] = htons(0x8899);		/* Realtek EtherType */
		t[1] = htons(0x0400);		/* protocol 0x04 (rtl8_4), reason 0 */
		t[2] = htons(0x0020);	/* LEARN_DIS (rtl8_4 word2) */
		t[3] = htons(SW_TAG_LAN_MASK);	/* CPU->switch forwarding port mask */
	}
#endif
	len = skb->len;
	luna_eth_tx_dump(skb->data, len);
	da = dma_map_single(ep->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	i = tx_slot(ep, ep->tx_head);
	ep->tx_skb[i] = skb;
	ep->tx_buf_dma[i] = da;
	ep->tx_buf_len[i] = len;

	ep->tx_ring[i].addr = da | DMA_BUS_WINDOW;	/* TX desc.addr bus window |= 0x20000000 */
	/* Program the descriptor cpu-tag so the GMAC inserts a cpu-tag carrying a
	 * NON-ZERO egress portmask; the switch (TAG_AWARE) parses it and directs
	 * the frame to the LAN port(s). tx_portmask 0 was the bug: the switch then
	 * does an empty L2 DA lookup and drops the frame ("TX never egresses"). */
#if TX_CPUTAG
	/* The software in-band 0x8899 tag (built above) already carries the egress
	 * portmask; the GMAC must NOT also insert its own (broken-portmask) cpu-tag,
	 * so opts2=0. This software-tag path is the one that put frames on the wire
	 * at the host (observed: clean stripped IPv6 frames received). */
	ep->tx_ring[i].opts2 = 0;
	ep->tx_ring[i].opts3 = 0;
#else
	/* PLAIN frame (cputag/TAG_AWARE trio gave 0 egress even with the correct
	 * GMAC CPUtagCR — reverted). The bootloader's own TX path writes NO cputag
	 * either. */
	ep->tx_ring[i].opts2 = word2;
	ep->tx_ring[i].opts3 = 0;
#endif
	ep->tx_ring[i].opts4 = 0;
	opts1 = D_OWN | D_FS | D_LS | D_TXCRC | (len & TXD_LEN_MASK);
	if (i == tx_eor_slot(ep))
		opts1 |= D_EOR;
	wmb();				/* descriptor body before ownership */
	ep->tx_ring[i].opts1 = opts1;
	wmb();

	ep->tx_head++;
	if (txgo_xmit)
		luna_eth_tx_fetch(ep);	/* stock GO handshake on every submit */
	ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* kick ring 0 */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	return NETDEV_TX_OK;
}

static netdev_tx_t luna_eth_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	return luna_eth_xmit_word2(skb, ndev, luna_chip_lan_txd_word2(netdev_priv(ndev), skb));
}

static void luna_eth_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	luna_eth_ring_timeout(netdev_priv(ndev), ndev);
}

static const struct net_device_ops luna_eth_netdev_ops = {
#ifdef CONFIG_GPON_FLOW_OFFLOAD
	.ndo_setup_tc		= luna_chip_setup_tc,
#endif
	.ndo_open		= luna_eth_open,
	.ndo_stop		= luna_eth_stop,
	.ndo_start_xmit		= luna_eth_xmit,
	.ndo_tx_timeout		= luna_eth_tx_timeout,
	.ndo_set_mac_address	= luna_eth_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
};

/* /proc/ethdump `omci_inject`: 5 baseline OMCI frames onto the US OMCC TX path,
 * with no OLT. Run at O5; ustx (0x1b0329bc) climbing means the steering works. */
static ssize_t luna_ethdump_write(struct file *f, const char __user *ubuf,
				      size_t cnt, loff_t *off)
{
	struct luna_eth *ep = g_ep;
	unsigned int k;
	u8 msg[48];
	char verb[24];

	if (!ep)
		return -ENODEV;
	if (!cnt || cnt >= sizeof(verb))
		return -EINVAL;
	if (copy_from_user(verb, ubuf, cnt))
		return -EFAULT;
	verb[cnt] = '\0';
	if (strncmp(verb, "omci_inject", 11))
		return -EINVAL;
	/* Arm the GMAC OMCI state (CPUTAGCR=0x901eff04, the cpu-tag trap + routing) as
	 * the OLT-driven path would via gpon_install_omcc -> set_omci_sid. Without the
	 * OLT the GMAC sits at the non-OMCI 0x981aff04, so the self-test must arm it to
	 * test the cpu-tag insertion representatively. Idempotent. */
	rtl9602c_eth_set_omci_sid(LUNA_OMCC_SID);
	for (k = 0; k < 5; k++) {
		memset(msg, 0, sizeof(msg));
		msg[0] = 0x00; msg[1] = 0x42;	/* TID */
		msg[2] = 0x2f;			/* MIB-Reset response (AK) */
		msg[3] = 0x0a;			/* DevID baseline */
		msg[4] = 0x00; msg[5] = 0x02;	/* ME ONU-data */
		omci_finalize(msg);		/* core: trailer + MIC */
		luna_eth_omci_xmit(ep, msg, sizeof(msg));
	}
	return cnt;
}

static int luna_ethdump_show(struct seq_file *m, void *v)
{
	struct luna_eth *ep = g_ep;
	unsigned int i, own = 0, hwfilled = 0;

	if (!ep) { seq_puts(m, "no device\n"); return 0; }
	if (ep->in_recovery) {	/* GMAC may be power-gated: MMIO would bus-abort */
		seq_printf(m, "GMAC recovery in progress (recovers=%u)\n",
			   ep->dbg_tx_recover);
		return 0;
	}
	seq_printf(m, "sw_core_reset: runs %u, failed steps %#x, words not restored %u\n", ep->sw_recovery.runs,
		   ep->sw_recovery.failed, ep->sw_recovery.unrestored);

	for (i = 0; i < ep->rxq[0].n; i++) {
		if (ep->rxq[0].ring[i].opts1 & D_OWN)
			own++;
		else
			hwfilled++;
	}
	seq_printf(m, "poll=%u filled=%u good=%u err=%u rx_head=%u rx_repoll=%u ctl_ring=%u ctl_rx=%u\n",
		   ep->dbg_poll, ep->dbg_filled, ep->dbg_good, ep->dbg_err,
		   ep->rxq[0].head, ep->dbg_rx_repoll, ep->ctl_ring_on, ep->dbg_ctl_rx);
	seq_printf(m, "last RX frame (pre-pull, len=%u): %*ph\n",
		   ep->dbg_rxlen, (int)sizeof(ep->dbg_rxbuf), ep->dbg_rxbuf);
	seq_printf(m, "omci: trap=%u rx=%u lookalike_forwarded=%u lastlen=%u msg=%*ph\n",
		   ep->omci_trap_on, ep->dbg_omci_rx, ep->dbg_omci_lookalike, ep->dbg_omci_rxlen,
		   (int)sizeof(ep->dbg_omci_rxbuf), ep->dbg_omci_rxbuf);
	seq_printf(m, "omci_tx: resp=%u drop=%u unhandled=%u mds=%u sn=%*ph\n",
		   ep->dbg_omci_tx, ep->dbg_omci_tx_drop, ep->dbg_omci_unhandled,
		   ep->omci_mds, 8, ep->omci_sn);
	/* the shared core's own DS discard counters.  ★ Two numbers, not one: a
	 * runt is a framing / GEM-reassembly fault upstream of OMCI, a bad MIC
	 * is corruption on a well-framed PDU, and one figure for both would make
	 * a broken reassembler read as a noisy fibre. */
	{
		u32 bad_mic, runt;

		luna_omci_rx_errors(&bad_mic, &runt);
		seq_printf(m, "omci_rx_drop: bad_mic=%u runt=%u\n", bad_mic, runt);
	}
	{
		unsigned int oring;
		u32 dmask;
		/* Same flowcore derivation as the xmit kick, so the diag can
		 * never disagree with the doorbell actually used. */
		bool dk1 = luna_gmac_omci_doorbell(omci_tx_ring, omci_doorbell_bit,
						  &oring, &dmask);

		if (omci_tx_ring == 0) {
			/* Shared-ring-0 OMCI test path: the OMCI frame rides the LAN ...
			 * dev/MEASURED-luna_eth.c.md sec 107. */
			int own_omci = -1;
			int slot = ep->omci_r0_last_slot;

			if (ep->tx_ring && slot >= 0 && slot < TX_RING_SIZE)
				own_omci = !!(ep->tx_ring[slot].opts1 & D_OWN);

			seq_printf(m,
				"omci_txring: PATH=shared-LAN-ring0 sid_idx=%u pon=%u doorbell=R_IO_CMD bit0  LANring head=%u dirty=%u  omci_slot=%d own[omci_slot]=%d  TxFDP1=%08x lanRingDMA=%08x IO_CMD=%08x\n",
				omci_sid_idx, omci_pon_port,
				ep->tx_head, ep->tx_dirty, slot, own_omci,
				ep_rd(ep, R_TxFDP1), (u32)ep->tx_ring_dma,
				ep_rd(ep, R_IO_CMD));
		} else {
			/* Dedicated-ring path. OWN-bit (opts1 BIT31 — the HW-read ...
			 * dev/MEASURED-luna_eth.c.md sec 108. */
			int own_hm1 = -1, own_d = -1;
			/* IO_CMD1 bit(16+h) is the RX multiring bitmap (NOT a TX-fetch
			 * enable); reported only as an info bit, it does not gate TX. */
			unsigned int en_mask = 1u << (16 + oring);

			if (ep->otx_ring && ep->otx_head != ep->otx_dirty) {
				unsigned int hm1 = otx_slot(ep, ep->otx_head - 1);
				unsigned int di  = otx_slot(ep, ep->otx_dirty);

				own_hm1 = !!(ep->otx_ring[hm1].opts1 & D_OWN);
				own_d   = !!(ep->otx_ring[di].opts1  & D_OWN);
			}

			seq_printf(m,
				"omci_txring: PATH=dedicated hwring=%u sid_idx=%u pon=%u doorbell=%s head=%u dirty=%u own[h-1]=%d own[d]=%d TxFDP%u=%08x ringDMA=%08x rxmring(IO_CMD1 bit%u)=%u\n",
				oring, omci_sid_idx, omci_pon_port,
				(omci_doorbell_bit != 0xff) ? "R_IO_CMD(forced)" :
					(dk1 ? "R_IO_CMD1|0x100" : "R_IO_CMD bit h"),
				ep->otx_head, ep->otx_dirty, own_hm1, own_d, oring,
				ep_rd(ep, R_TxFDP(oring)), (u32)ep->otx_ring_dma,
				16 + oring, !!(ep_rd(ep, R_IO_CMD1) & en_mask));
		}
	}
	seq_printf(m, "rxring: HW-owned(D_OWN=1)=%u  CPU-owned(filled)=%u\n",
		   own, hwfilled);
	{
		/* TX ring OWN bitmap (slot 0 = LSB of the first hex word):
		 * separates "HW transmits but never clears OWN" from reclaim
		 * bugs at a glance. */
		u32 bm0 = 0, bm1 = 0;

		for (i = 0; i < 32; i++) {
			if (ep->tx_ring[i].opts1 & D_OWN)
				bm0 |= 1u << i;
			if (ep->tx_ring[i + 32].opts1 & D_OWN)
				bm1 |= 1u << i;
		}
		seq_printf(m, "txring own[31:0]=%08x own[63:32]=%08x head=%u dirty=%u rot=%u\n",
			   bm0, bm1, ep->tx_head, ep->tx_dirty, ep->tx_rot);
	}
	seq_printf(m, "GMAC IO_CMD=%08x IO_CMD1=%08x MSR(0x58)=%08x\n",
		   ep_rd(ep, R_IO_CMD), ep_rd(ep, R_IO_CMD1),
		   ep_rd(ep, R_MSR));
	/* TX-DMA park forensics: per-ring HW fetch cursors (descriptor index
	 * relative to TxFDP), the ring rotations, and the recovery counters.
	 * cdo0+rot vs (dirty%64) localises a park instantly. */
	seq_printf(m, "txdma cdo0=%u cdo1=%u cdo2=%u cdo3=%u cdo4=%u rot=%u/%u stall_ms=%u rearms=%u recovers=%u gmac_reset=%u\n",
		   ioread16(ep->base + R_TxCDO(0)), ioread16(ep->base + R_TxCDO(1)),
		   ioread16(ep->base + R_TxCDO(2)), ioread16(ep->base + R_TxCDO(3)),
		   ioread16(ep->base + R_TxCDO(4)), ep->tx_rot, ep->otx_rot,
		   ep->stall_since ? jiffies_to_msecs(jiffies - ep->stall_since) : 0,
		   ep->dbg_rearm, ep->dbg_tx_recover, gmac_reset);
	seq_printf(m, "GMAC RCR=%08x TCR=%08x CONFIG=%08x CPUTAGCR=%08x\n",
		   ep_rd(ep, R_RCR), ep_rd(ep, R_TCR), ep_rd(ep, R_CONFIG),
		   ep_rd(ep, R_CPUTAGCR));
	/* (GMAC1 0x18014000 / GMAC2 0x18016000 are DEAD MMIO on the 9602C — reading
	 * them bus-aborts the whole diag. The 9602C has only GMAC0; the d1 gmac_id=2
	 * is a 9607C-ism. US OMCI must egress GMAC0.) */
	seq_printf(m, "GMAC RxFDP=%08x RxCDO=%08x RxDesNum=%08x ringDMA=%08x\n",
		   ep_rd(ep, R_RxFDP), ep_rd(ep, R_RxCDO), ep_rd(ep, R_RxDesNum),
		   (u32)ep->rxq[0].ring_dma);
	/* Full GMAC config diff vs LIVE stock O5 (stock golden values ...
	 * dev/MEASURED-luna_eth.c.md sec 109. */
	seq_printf(m, "GMACcfg 10=%08x[f:04a80457] 20=%08x[034c0003] 24=%08x[010c0000] 38=%08x[0a] 3c=%08x[f8350240]\n",
		   ep_rd(ep, R_TXOKCNT), ep_rd(ep, R_RXOKPHY), ep_rd(ep, R_RXOKMUL),
		   ep_rd(ep, 0x38), ep_rd(ep, 0x3c));
	seq_printf(m, "GMACcfg 44=%08x[0f] 58=%08x[f0638000] 5c=%08x[04000000] d0=%08x[3f] d8=%08x[11110000]\n",
		   ep_rd(ep, R_RCR), ep_rd(ep, R_MSR), ep_rd(ep, 0x5c),
		   ep_rd(ep, R_IMR0), ep_rd(ep, R_ISR1));
	/* GMAC0 MAC-level MIB counters (16-bit, BE-packed two per ...
	 * dev/MEASURED-luna_eth.c.md sec 110. */
	seq_printf(m, "GMAC_MIB txok=%u rxok=%u txerr=%u rxerr=%u miss=%u\n",
		   ep_rd(ep, R_TXOKCNT) >> 16, ep_rd(ep, R_TXOKCNT) & 0xffff,
		   ep_rd(ep, R_TXERR) >> 16, ep_rd(ep, R_TXERR) & 0xffff,
		   ep_rd(ep, R_MISSPKT) >> 16);
	/* NIC interrupt status: per-ring RDU ... -- dev/MEASURED-luna_eth.c.md sec 111. */
	seq_printf(m, "NIC ISR(0x3c=[IMR:ISR])=%08x ISR1(0xd8)=%08x  perRingRxCDO r0=%04x r1=%04x r2=%04x r3=%04x r4=%04x r5=%04x\n",
		   ep_rd(ep, 0x3c), ep_rd(ep, R_ISR1),
		   ep_rd(ep, R_RxCDO) >> 16, ep_rd(ep, R_RxMRingCDO(1)) >> 16,
		   ep_rd(ep, R_RxMRingCDO(2)) >> 16, ep_rd(ep, R_RxMRingCDO(3)) >> 16,
		   ep_rd(ep, R_RxMRingCDO(4)) >> 16, ep_rd(ep, R_RxMRingCDO(5)) >> 16);
	if (ep->sw) {
		seq_printf(m, "SW permit(1c088)=%08x flood bc/mc/uc=%08x/%08x/%08x\n",
			   ioread32(ep->sw + ep->swm->src_permit),
			   ioread32(ep->sw + ep->swm->bc_flood),
			   ioread32(ep->sw + ep->swm->unkn_mc_flood),
			   ioread32(ep->sw + ep->swm->unkn_uc_flood));
		seq_printf(m, "SW vlan_ctrl(13008)=%08x cputag_ctrl(23030)=%08x\n",
			   ioread32(ep->sw + SW_VLAN_CTRL),
			   ioread32(ep->sw + SW_MAC_CPU_TAG_CTRL));
		/* ⚠ THESE FOUR ARE READ, NOT UNDERSTOOD (2026-09-04). They ...
		 * dev/MEASURED-luna_eth.c.md sec 112. */
		seq_printf(m, "SW sw(198)=%08x sw(1b8)=%08x sw(1d8)=%08x sw(1f8)=%08x\n",
			   ioread32(ep->sw + 0x198), ioread32(ep->sw + 0x1b8),
			   ioread32(ep->sw + 0x1d8), ioread32(ep->sw + 0x1f8));
		/* THE RX ADDRESSES BELOW DO NOT FOLLOW THE FORMULA, and they ...
		 * dev/MEASURED-luna_eth.c.md sec 113. */
		seq_printf(m, "MIB p0(LAN) tx=%08x | p1(LAN) tx=%08x\n",
			   ioread32(ep->sw + SW_STAT_PORT_TX_MIB), ioread32(ep->sw + SW_STAT_PORT_TX_MIB + 1 * SW_STAT_PORT_MIB_STRIDE));
		seq_printf(m, "MIB p2(PON) tx=%08x %08x %08x | rx=%08x %08x %08x\n",
			   ioread32(ep->sw + SW_STAT_PORT_TX_MIB + 2 * SW_STAT_PORT_MIB_STRIDE), ioread32(ep->sw + SW_STAT_PORT_TX_MIB + 2 * SW_STAT_PORT_MIB_STRIDE + 4),
			   ioread32(ep->sw + SW_STAT_PORT_TX_MIB + 2 * SW_STAT_PORT_MIB_STRIDE + 8), ioread32(ep->sw + 0x32500),
			   ioread32(ep->sw + 0x32504), ioread32(ep->sw + 0x32508));
		seq_printf(m, "MIB p3(CPU) tx=%08x %08x %08x | rx=%08x %08x %08x\n",
			   ioread32(ep->sw + SW_STAT_PORT_TX_MIB + 3 * SW_STAT_PORT_MIB_STRIDE), ioread32(ep->sw + SW_STAT_PORT_TX_MIB + 3 * SW_STAT_PORT_MIB_STRIDE + 4),
			   ioread32(ep->sw + SW_STAT_PORT_TX_MIB + 3 * SW_STAT_PORT_MIB_STRIDE + 8), ioread32(ep->sw + 0x32600),
			   ioread32(ep->sw + 0x32604), ioread32(ep->sw + 0x32608));
	}
	return 0;
}

static int luna_ethdump_open(struct inode *ino, struct file *fp)
{
	return single_open(fp, luna_ethdump_show, NULL);
}

static const struct proc_ops luna_ethdump_pops = {
	.proc_open	= luna_ethdump_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= luna_ethdump_write,
};

static int luna_eth_omci_send(void *cookie, const u8 *msg, unsigned int len)
{
	return luna_eth_omci_xmit(cookie, msg, len);
}

static void luna_eth_tx_fence(void *cookie)
{
	struct luna_eth *ep = cookie;
	unsigned long flags;

	spin_lock_irqsave(&ep->tx_lock, flags);
	spin_unlock_irqrestore(&ep->tx_lock, flags);
}

/* the UNI administrative state -- dev/MEASURED-luna_eth.c.md sec 114. */
static int luna_uni_admin_set(void *cookie, unsigned int port, bool locked)
{
	struct luna_eth *ep = cookie;
	u16 bmcr = 0;
	u32 mode = 0;
	int rc;

	/* A UNI is a COPPER port.  The PON and CPU ports share this index space,
	 * so a mis-declared panel would otherwise drive a PHY that is not there
	 * and then report the reserved port LOCKED. */
	if (!ep || port >= ep->swm->n_copper) {
		pr_err("rtl9602c-eth: UNI port %u is not one of this board's %u copper ports -- REFUSED, nothing is written\n",
		       port, ep ? ep->swm->n_copper : 0);
		return -EINVAL;
	}

	mutex_lock(&luna_chip_gphy_lock);
	WRITE_ONCE(luna_uni_locked_ports,
		   locked ? (luna_uni_locked_ports | BIT(port))
			  : (luna_uni_locked_ports & ~BIT(port)));
	rc = luna_chip_port_gate__locked(ep, port, locked || luna_loop_port_down(ep->loop, port),
					!locked, "UNI", &bmcr, &mode);
	mutex_unlock(&luna_chip_gphy_lock);
	if (rc)
		return rc;
	dev_info(ep->dev, "UNI port %u %s (BMCR %04x, ABLTY_FORCE_MODE %08x)\n",
		 port, locked ? "LOCKED" : "UNLOCKED", bmcr, mode);
	return 0;
}

static void luna_eth_omci_release(void *cookie)
{
	WRITE_ONCE(g_ep, NULL);
	luna_omci_detach(cookie);
}

static int luna_loop_gate(struct net_device *ndev, unsigned int port, bool down)
{
	struct luna_eth *ep = netdev_priv(ndev);
	u16 bmcr;
	u32 mode;
	int rc;

	mutex_lock(&luna_chip_gphy_lock);
	rc = luna_chip_port_gate__locked(ep, port, down || luna_uni_port_locked(port),
					!luna_uni_port_locked(port), "LAN loop", &bmcr, &mode);
	mutex_unlock(&luna_chip_gphy_lock);
	return rc;
}

/* A loop probe out of ONE port, for a die without the switch's loop engine: the descriptor's egress mask. */
static int luna_loop_xmit(struct net_device *ndev, struct sk_buff *skb, unsigned int port)
{
	if (luna_eth_xmit_word2(skb, ndev, luna_gmac_cputag_txd_pmask(BIT(port))) != NETDEV_TX_BUSY)
		return 0;
	dev_kfree_skb_any(skb);
	return -EBUSY;
}

static const struct luna_loop_ops luna_eth_loop_ops = {
	.gate	= luna_loop_gate,
	.xmit	= luna_loop_xmit,
};

/* The PON WAN netdev: link state and line rate by the standard call (eth_pon_link.h). */
static const struct ethtool_ops luna_wan_ethtool_ops = {
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= eth_pon_get_link_ksettings,
};

ETH_RX_WINDOW_DRAM_MIB();

/* ethtool -g: rx = the window now, rx max = the largest window (the ring less one); rx mini = the control ring's; tx = the fixed TX ring. */
static void luna_eth_ring_get(struct net_device *ndev, struct ethtool_ringparam *r)
{
	struct luna_eth *ep = netdev_priv(ndev);

	r->rx_max_pending = ep->rxq[0].rw.w.cap;
	r->rx_pending = READ_ONCE(ep->rxq[0].rw.armed);
	r->rx_mini_max_pending = r->rx_mini_pending = ep->n_rxq > 1 ? ep->rxq[1].rw.w.cap : 0;
	r->tx_max_pending = r->tx_pending = TX_RING_SIZE;
}

/* ethtool -G rx N pins the window at N rounded up to a group, floor..cap, until the next boot. */
static int luna_eth_ring_set(struct net_device *ndev, const struct ethtool_ringparam *r)
{
	struct luna_eth *ep = netdev_priv(ndev);
	struct luna_rxq *q = &ep->rxq[0];

	if (!ep->hw_up)
		return -EBUSY;
	if (r->tx_pending != TX_RING_SIZE || r->rx_mini_pending != (ep->n_rxq > 1 ? ep->rxq[1].rw.w.cap : 0) ||
	    r->rx_pending < q->rw.w.floor || r->rx_pending > q->rw.w.cap)
		return -EINVAL;
	spin_lock_bh(&ep->rxw_lock);
	eth_rx_window_ring_pin(&q->rw, r->rx_pending);
	spin_unlock_bh(&ep->rxw_lock);
	schedule_work(&ep->rxw_work);
	luna_rxw_kick_poll(ep);
	return 0;
}

static void luna_ring_stats(struct net_device *ndev, u64 *data)
{
	struct luna_eth *ep = netdev_priv(ndev);
	const struct luna_rxq *q = &ep->rxq[0];

	eth_rx_window_ring_stats(&q->rw, data);
}

static const struct luna_ring_ops luna_eth_ring_ops = {
	.get = luna_eth_ring_get, .set = luna_eth_ring_set, .stats = luna_ring_stats,
};

/* The GMAC's own registers, the same on every die: what a recovery snapshot reads of the NIC. */
static const struct eth_reg_range luna_state_nic[] = {
	{ 0x00, 4 }, { 0x14, 3 }, { 0x24, 6 }, { 0x40, 36 }, { 0xd4, 11 }, { 0, 0 } };
static const struct eth_reg_range luna_state_nic_status[] = {
	{ 0x10, 1 }, { 0x20, 1 }, { 0x3c, 1 }, { 0xd0, 1 }, { 0, 0 } };

static bool luna_sw_begin(void *ctx)
{
	struct luna_eth *ep = ctx;

	rtnl_lock();
	luna_chip_offload_lock(ep);
	if (ep->hw_up && ep->swm->sw_rst)
		return true;
	ep->sw_recovery.end(ctx);
	return false;
}

static void luna_sw_end(void *ctx)
{
	luna_chip_offload_unlock(ctx);
	rtnl_unlock();
}

static void luna_sw_quiesce(void *ctx)
{
	struct luna_eth *ep = ctx;

	netif_tx_disable(ep->ndev);
	if (ep->wan_ndev)
		netif_tx_disable(ep->wan_ndev);
	napi_disable(&ep->napi);
}

static void luna_sw_reset(void *ctx)
{
	struct luna_eth *ep = ctx;

	ASSERT_RTNL();	/* the order luna_sw_begin() took */
	luna_chip_offload_assert_held(ep);
	luna_mib_core_reset(ep->ndev);
}

static void luna_sw_resume(void *ctx)
{
	struct luna_eth *ep = ctx;

	napi_enable(&ep->napi);
	napi_schedule(&ep->napi);
	if (netif_running(ep->ndev))
		netif_wake_queue(ep->ndev);
	if (ep->wan_ndev && netif_running(ep->wan_ndev))
		netif_wake_queue(ep->wan_ndev);
}

static int luna_sw_reinit_msr(void *ctx)
{
	luna_eth_msr_apply(ctx, true);
	return 0;
}

static const struct eth_init_step luna_sw_reinit_steps[] = {
	{ "nic flow control", luna_sw_reinit_msr },
};

static const struct eth_core_recover_ops luna_sw_recover_ops = {
	.quiesce = luna_sw_quiesce,
	.reset = luna_sw_reset,
	.resume = luna_sw_resume,
	.steps = luna_sw_reinit_steps,
	.n_steps = ARRAY_SIZE(luna_sw_reinit_steps),
};

/* The recovery on demand: the work a stalled port asks for (a lab check, or a field unit by hand). */
static int luna_sw_core_reset_set(const char *val, const struct kernel_param *kp)
{
	bool go;
	int ret = kstrtobool(val, &go);

	if (ret || !go)
		return ret;
	if (!g_ep || !g_ep->hw_up || !g_ep->swm->sw_rst)
		return -ENODEV;
	return eth_core_recovery_ask(&g_ep->sw_recovery) ? 0 : -EBUSY;
}

static int luna_sw_core_reset_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%u\n", g_ep ? g_ep->sw_recovery.runs : 0);
}

static const struct kernel_param_ops luna_sw_core_reset_ops = {
	.set = luna_sw_core_reset_set,
	.get = luna_sw_core_reset_get,
};
module_param_cb(sw_core_reset, &luna_sw_core_reset_ops, NULL, 0644);
MODULE_PARM_DESC(sw_core_reset, "write 1: reset the switch core and re-init it, as a stalled port does; read: how many ran");

static int luna_eth_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct luna_eth *ep;
	unsigned int dram_mib;
	u8 mac[ETH_ALEN];
	int ret;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ndev = devm_alloc_etherdev(dev, sizeof(*ep));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, dev);
	platform_set_drvdata(pdev, ndev);

	ep = netdev_priv(ndev);
	ep->ndev = ndev;
	ep->dev = dev;
	spin_lock_init(&ep->tx_lock);
	INIT_WORK(&ep->recover_work, luna_eth_recover_work);
	INIT_WORK(&ep->rxw_work, luna_rxw_work);
	spin_lock_init(&ep->rxw_lock);
	eth_core_recovery_init(&ep->sw_recovery, ndev, ep, &luna_sw_recover_ops, luna_sw_begin, luna_sw_end,
			       luna_chip_sw_blocks);

	ep->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ep->base))
		return PTR_ERR(ep->base);

	/* A board may run a shorter RX ring than the family's stock 1024: the
	 * 32 MB X100DG's own stock runs 256, and 1024 buffers do not fit it. */
	ep->rxq[0].n = RX_RING_SIZE;
	if (!of_property_read_u32(dev->of_node, "realtek,rx-ring-entries", &ep->rxq[0].n) &&
	    (ep->rxq[0].n < 64 || ep->rxq[0].n > RX_RING_SIZE)) {
		dev_err(dev, "realtek,rx-ring-entries %u is outside 64..%u\n", ep->rxq[0].n,
			RX_RING_SIZE);
		return -EINVAL;
	}
	ep->rxq[0].skb = ep->rx_skb;
	ep->rxq[0].buf_dma = ep->rx_buf_dma;
	ep->rxq[0].tail_reg = R_RxDesNum;
	dram_mib = eth_rx_window_dram_mib();
	ep->rx_floor = eth_rx_window_floor(ep->rxq[0].n, dram_mib);
	dev_info(dev, "RX ring %u slots, window %u..%u (DRAM %u MiB)\n", ep->rxq[0].n, ep->rx_floor,
		 eth_rx_window_cap(ep->rxq[0].n), dram_mib);
	ep->rxq[1] = (struct luna_rxq){ .skb = ep->ctl_skb, .buf_dma = ep->ctl_buf_dma,
					    .n = CTL_RING_SIZE, .tail_reg = R_RxMRingCfg(1) + 8 };

	/* Switch core (best-effort; minimal L2 flood enabled at open). */
	ret = luna_chip_probe(ep, pdev);
	if (ret)
		return ret;
	ep->n_rxq = ep->swm->ctl_acl ? 2 : 1;
	ep->sw = devm_ioremap(dev, SWCORE_PHYS, ep->swm->swcore_size);
	if (luna_chip_gmac.memctl)
		ep->txgo = devm_ioremap(dev, MEMCTL_PHYS, 0x1000);

	/* Snapshot the bootloader's live GMAC0 control config to re-assert at open. */
	ep->ub_tcr = ep_rd(ep, R_TCR);
	ep->ub_rcr = ep_rd(ep, R_RCR);
	ep->ub_config = ep_rd(ep, R_CONFIG);
	ep->ub_cputagcr = ep_rd(ep, R_CPUTAGCR);
	ep->ub_cputag1cr = ep_rd(ep, R_CPUTAG1CR);

	/* ★ THE BRING-UP DEFAULT IS REFUSED, not merely validated. ...
	 * dev/MEASURED-luna_eth.c.md sec 116. */
	{
		u8 dt[GPON_HWADDR_BYTES], eng[GPON_HWADDR_BYTES];
		bool have_dt = of_get_mac_address(dev->of_node, dt) == 0;
		enum gpon_hwaddr_src src;
		bool eng_ok;

		luna_eth_get_hwaddr(ep, eng);
		eng_ok = !luna_mac_is_bringup_default(eng);
		src = gpon_hwaddr_resolve(mac_param, have_dt ? dt : NULL,
					  eng_ok ? eng : NULL, mac);
		eth_hw_addr_set(ndev, mac);

		if (src == GPON_HWADDR_RANDOM)
			dev_warn(dev,
				 "MAC engine holds %pM: %s. Using %s %pM instead -- this board's real MAC lives in the vendor config partition and is not read yet\n",
				 eng,
				 eng_ok
				 ? "not a valid unicast address"
				 : "the SILICON BRING-UP DEFAULT, identical on every board of this family, which would collide on a shared segment",
				 gpon_hwaddr_src_name(src), ndev->dev_addr);
		else
			/* an address from outside the device must be auditable */
			dev_info(dev, "MAC %pM from %s\n", ndev->dev_addr,
				 gpon_hwaddr_src_name(src));
	}

	luna_chip_offload_probe(ep, dev);

	ndev->netdev_ops = &luna_eth_netdev_ops;
	ndev->watchdog_timeo = msecs_to_jiffies(ETH_TX_TIMEOUT_MS);
	ret = luna_mib_attach(ndev, ep->sw, ep->swm, &luna_chip_phy_ops);
	if (ret)
		return dev_err_probe(dev, ret, "switch statistics attach failed\n");
	ret = luna_mib_ring_attach(ndev, &luna_eth_ring_ops);
	if (ret)
		return dev_err_probe(dev, ret, "ring parameters attach failed\n");
	ep->loop = luna_loop_attach(dev, ndev, ep->sw, ep->swm, &luna_eth_loop_ops);
	if (!ep->loop)
		return -ENOMEM;
	/* TC feature bit; nftables TC_SETUP_FT registration does not consult it. */
	ndev->hw_features |= NETIF_F_HW_TC | (luna_chip_rx_csum ? NETIF_F_RXCSUM : 0);
	ndev->features |= NETIF_F_HW_TC | (luna_chip_rx_csum ? NETIF_F_RXCSUM : 0);
	/* Permit a LIVE MAC change (no iface down/up): the per-board ...
	 * dev/MEASURED-luna_eth.c.md sec 120. */
	ndev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
	netif_carrier_off(ndev);

	/* INTC input 26 (GMAC0). <=0 -> no DT mapping: open() runs pure-poll. */
	ep->irq = platform_get_irq(pdev, 0);
	if (ep->irq < 0)
		ep->irq = -1;
	netif_napi_add(ndev, &ep->napi, luna_eth_napi_poll);

	ret = luna_omci_attach(&luna_onu, ep, (u8)omci_mds_seed,
			       luna_eth_omci_send, luna_eth_tx_fence,
			       luna_uni_admin_set);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, luna_eth_omci_release, ep);
	if (ret)
		return ret;
	WRITE_ONCE(g_ep, ep);
	ret = eth_devm_register_netdev(dev, ndev, &luna_ring_watch.w, 0);
	if (ret)
		return ret;

	dev_info(dev, "%s NIC at %pR, MAC %pM\n", luna_chip_gmac.name,
		 platform_get_resource(pdev, IORESOURCE_MEM, 0), ndev->dev_addr);

	/* gpon0: the WAN data-GEM netdev (clean-room nas0-equivalent). Shares ep's RX/TX
	 * rings + NAPI (owned by eth0); RX is demux'd by ingress port (PON port 2 -> gpon0)
	 * and US frames steer to GPON_DATA_FLOW. Its MAC is the WAN identity = board MAC +
	 * model offset (stock nas0_0 = base+3); see luna_wan_mac. */
	{
		struct net_device *wan = devm_alloc_etherdev(dev, sizeof(struct luna_eth *));
		u8 wmac[ETH_ALEN];

		if (wan) {
			*(struct luna_eth **)netdev_priv(wan) = ep;
			SET_NETDEV_DEV(wan, dev);
			strscpy(wan->name, "gpon0", IFNAMSIZ);
			wan->netdev_ops = &luna_eth_wan_ops;
			wan->ethtool_ops = &luna_wan_ethtool_ops;
			/* Keep the LAN and WAN feature declarations consistent. */
			wan->hw_features |= NETIF_F_HW_TC | (luna_chip_rx_csum ? NETIF_F_RXCSUM : 0);
			wan->features |= NETIF_F_HW_TC | (luna_chip_rx_csum ? NETIF_F_RXCSUM : 0);
			/* Initial WAN MAC = board MAC + offset; re-derived at open + on eth0 MAC
			 * changes once rtk_factory provisions the real board MAC onto eth0. */
			luna_chip_wan_mac(ep, wmac, ndev->dev_addr);
			eth_hw_addr_set(wan, wmac);
			luna_omci_set_wan_mac(wmac);
			netif_carrier_off(wan);
			if (eth_devm_register_netdev(dev, wan, NULL,
								     ETH_NO_set_link_ksettings | ETH_NO_get_pauseparam | ETH_NO_set_pauseparam |
								     ETH_NO_get_ethtool_stats | ETH_NO_get_ringparam | ETH_NO_get_regs | ETH_NO_rx_watch) == 0)
				ep->wan_ndev = wan;
			else
				dev_warn(dev, "gpon0 (WAN) register failed; WAN datapath disabled\n");
		}
	}

	ep->omci_mds = (u8)omci_mds_seed;	/* poison seed: fail the OLT's ME2 audit so a warm
						 * re-admit re-provisions (we hold no persistent MIB) */
	proc_create("ethdump", 0644, NULL, &luna_ethdump_pops);
	return 0;
}

