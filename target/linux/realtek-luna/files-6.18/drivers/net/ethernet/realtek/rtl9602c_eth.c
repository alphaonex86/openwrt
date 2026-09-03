// SPDX-License-Identifier: GPL-2.0-only
/*
 * TIER: CHIP — hardware shell for exactly ONE part: registers, DMA,
 * interrupts, board glue.  It DOES; the core DECIDES.  GPON protocol
 * logic belongs in the core tier (drivers/net/gpon), never here.
 * Role: RTL9602C Ethernet/NIC shell.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon-common/files-6.18/drivers/net/gpon/gpon_common.h.
 */
/*
 * Ethernet driver for the Realtek RTL9602C (RLX) GPON SoC.
 *
 * Independent implementation from the SoC's register interface and the
 * G.984/G.988 protocols. Drives the integrated GMAC/DMA NIC that attaches to
 * the SoC switch core. This increment adds the descriptor-ring DMA datapath
 * in POLLED mode (a periodic timer drains RX and reclaims TX) so the rings
 * can be validated before the NIC interrupt input is identified. The switch
 * and the GMAC control registers are left in the state the bootloader
 * configured: the bootloader used this NIC+switch for its TFTP transfer, so
 * re-asserting the established control words (IO_CMD last) brings the datapath
 * up without recomputing every enable bit. NAPI + hardware interrupt replace
 * the poll timer once the INTC input is known.
 *
 * Registers are accessed native (ioread32/iowrite32) — the SoC presents them
 * big-endian, matching the CPU.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/interrupt.h>	/* request_irq/free_irq, irqreturn_t, IRQF_SHARED */
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "luna_eth_regs.h"	/* the family MAC/switch register map + per-chip table */
#include "gpon_omci_core.h"	/* the responder + omci_set_mic + the AVC emitters */
#include "gpon_omci_trace.h"	/* G.988 decode-to-a-buffer for the log */
#include "gpon_omci_me.h"	/* the common OMCI ME store + context */
#include "rtl9602c_gpon_nic.h"
#include "luna_gmac_logic.h"	/* family GMAC ring packings (flowcore) */

/*
 * US-OMCI TX tuning knobs (see rtl9602c_eth_omci_xmit).
 *
 * The authoritative stock 9602C OMCI-TX path builds a 5-word "txInfo"
 * descriptor and submits it on a dedicated TX ring with a per-ring poll
 * doorbell. Two of the field values are derived at runtime by the stock
 * firmware from internal GPON state we do not mirror exactly:
 *   - the PON_SID one-hot SHIFT (the stock reads a GEM-SID-index global and
 *     emits a one-hot 1<<idx into word3[28:23]); the right small index for our
 *     single OMCC flow is unknown from the descriptor build alone, so expose it.
 *   - the PON port (ExtSpa) placed in word3[22:16]; our PON switch port is 2.
 *
 * HW-RING / DOORBELL (observed from the stock device's TX-submit behavior):
 * stock submits with a SW ring number and maps it to a HW ring via
 *   idx_sw2hw(ring) = 4 - ring   ("HW ring h").
 * The per-packet kick then targets that SAME HW ring h:
 *   if (h < 4)  R_IO_CMD  |= (1 << h);
 *   if (h == 4) R_IO_CMD1 |= 0x100;
 * and the descriptor ring lives at R_TxFDP(h) = 0x1300 + h*16. So all three
 * (descriptor ring, TxFDP arm, doorbell) MUST reference the SAME HW ring h.
 *
 * omci_tx_ring below is the HW ring number directly (it indexes R_TxFDP(h) and
 * derives the doorbell from the same h). Stock OMCI = SW ring 4 = HW ring 0; but
 * our LAN/normal-xmit path already owns HW ring 0 (R_TxFDP1=0x1300, kick R_IO_CMD
 * bit0), so to avoid a collision we place the US-OMCI on a FREE HW ring (default
 * HW ring 4: TxFDP 0x1340, kick R_IO_CMD1 |= 0x100). The encoding is identical;
 * only the ring instance differs. Kept tunable for HW bring-up.
 */
/*
 * PON_SID one-hot SHIFT for the OMCC US flow (word3[28:23] = ((1<<idx)&0x3F)<<23).
 *
 * Default 4 from clean-room study of the stock device's OMCI-TX behavior + a live-stock cross-check:
 *   - the stock OMCI-TX path builds the steering exactly as we do:
 *       v0=1; v0 <<= sid_idx; v0 &= 0x3F; v0 <<= 0x17(23)   -> word3[28:23] one-hot
 *       (sid_idx comes from internal GPON state, dereferenced from a per-flow
 *        mirror; the first descriptor builder reads the same value from that mirror).
 *     The value is PROVISIONED at OMCC GEM install (the setter just copies
 *     a caller-supplied word into the mirror), so it is NOT a baked constant — it
 *     is a small table index into the 6-way PON_SID classifier.
 *   - DECISIVE cross-check: a prior live-stock read showed US RX_SID_GOOD for SID 64
 *     increments group [4] (RX_SID_GOOD_CNT_US 0x204c). The 6-bit one-hot [28:23]
 *     selects one of the 6 classify slots whose good-counters are groups [0..4]
 *     (0x203c/0x2040/0x2044/0x2048/0x204c); the only one-hot bit that routes SID 64
 *     into group [4] is bit 4 => sid_idx = 4. The old default 0 emitted one-hot bit
 *     0 (group [0] @0x203c), so our OMCI never reached the SID-64 classify slot
 *     (us_rxsid[4] good=0 AND bad=0 — exactly the observed symptom).
 */
static unsigned int omci_sid_idx = 4;	/* PON_SID one-hot shift (word3[28:23]); 4 = SID-64 classify slot (RX_SID group [4]) */
module_param(omci_sid_idx, uint, 0644);
MODULE_PARM_DESC(omci_sid_idx, "US-OMCI PON_SID one-hot shift count (default 4 = SID-64 classify slot)");

/*
 * ExtSpa / PON port (word3[22:16] = (port&0x7F)<<16). The stock device reads this
 * from a per-port PON-MAC control field (the byte at the GPON-MAC control region offset 0x5DB:
 * load byte, &0x7F, <<16) — set at the US T-CONT/port binding (written as the high
 * byte of the 0x5D8 word during port install), so it too is provisioned, not baked.
 * For our single PON it is the PON switch port = 2.
 */
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
static unsigned int omci_minimal;	/* TEST: 1 = LAN-identical desc (no ORG, opts2/3=0) to isolate the GMAC TX halt-after-~3 */
module_param(omci_minimal, uint, 0644);

/* HW ring for the US-OMCI descriptor ring. This indexes R_TxFDP(h) AND derives
 * the doorbell, so they can never disagree. The dedicated path uses a FREE ring
 * (HW ring 4) since the LAN owns ring 0.
 *
 * SPECIAL CASE omci_tx_ring==0: instead of a dedicated ring, enqueue the OMCI
 * frame onto the SAME LAN ring 0 (ep->tx_ring) the normal TX path uses — which
 * is PROVEN to fetch (the host's SSH/LAN traffic flows through it). This ISOLATES
 * whether the corrected OMCI descriptor STEERING alone (word0 org bits, word3
 * one-hot PON_SID + ExtSpa) routes the frame to the PON US-NIC, independent of
 * the unresolved "HW ring 4 won't fetch" problem. The LAN TX path is shared
 * (same ring + same ep->tx_lock + same R_IO_CMD bit0 kick + same LAN reclaim),
 * so normal LAN traffic is undisturbed; if the ring is full the OMCI frame is
 * dropped (never blocks). omci_tx_ring!=0 keeps the dedicated-ring-4 path intact.
 *
 * NOTE: default is 0 (shared-ring-0 test path) FOR THIS TEST so the image boots
 * straight into the shared-ring-0 steering experiment. Set to 4 to restore the
 * dedicated ring. */
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

/*
 * GMAC bring-up mode. The TX-DMA hard-park root cause: the driver inherited the
 * GMAC from U-Boot's polled TFTP (IO_CMD 0x400f3330) and never reset it nor gave
 * it an IO_CMD 0->config enable edge. The stock NIC init behavior is categorical
 * that the multi-ring fetch engine only latches its internal ring state through a
 * BSP_IP_SEL IP-block power-cycle followed by programming the rings with IO_CMD=0
 * and writing IO_CMD1-then-IO_CMD last (otherwise "mring can't receive
 * packet at first time"); the stock 9602C reset path does the same:
 * IP_SEL bit1 off / ~10ms / on. On the inherited,
 * never-reset engine the first sparse park latches fatally (txok frozen, OWN
 * stuck at the HW cursor, doorbell-immune) - measured: dirty freezes at the 4th
 * descriptor ever, a plain LAN frame, before any OMCI inject.
 *   1 = stock-faithful cold start: stop_hw + IP-block power-cycle + full
 *       reprogram + IO_CMD edge with the LIVE-STOCK operating values.
 *   0 = legacy inherited-U-Boot bring-up (the wedging baseline, kept for A/B).
 */
/* HW verdict 2026-06-11: ANY GMAC reset (BSP_IP_SEL power-cycle OR CMD.RST)
 * permanently kills the GMAC->switch egress link on this board — descriptors
 * complete but no frame reaches switch port 3 afterwards (p3 tx MIB frozen,
 * txok 0, ~750 frames evaporated), exactly the "reset desyncs the U-Boot
 * GMAC<->switch IP-block sync" failure that the known-working approach avoids by
 * never resetting. Default OFF: inherit U-Boot's engine (egress proven) and rely on
 * the soft TxFDP re-arm for un-parking. */
static unsigned int gmac_reset = 1;	/* now safe to reset: rtl9602c_uboot_swcore_bringup() re-runs U-Boot's GMAC<->switch resync after the IP-block reset, breaking the old "reset kills egress" catch-22, and the stock GMAC init re-establishes the GMAC0-TX->US-NIC direct link */
/* Same-board diff (stock-WORKING vs mine-BROKEN) SoC-ctrl bits 0x18000100[8]/0x18000104[2]
 * (stock sets, mine doesn't; candidate IP-mux/US-NIC clock/power). Default 1 (test); 0=off. */
static unsigned int ipmux_soc = 1;
module_param(ipmux_soc, uint, 0644);
MODULE_PARM_DESC(ipmux_soc, "1=set the same-board-diff SoC-ctrl bits 0x18000100[8]/0x18000104[2] at reset");
/* Same-board diff (stock vs mine) NETWORK-ENGINE / IP-mux bits: 0x18001000 bit19 (stock set, mine clear)
 * and 0x18001098 (stock=0x0004e123, mine=0x0018a123: clear bits19,20; set bits14,18). These were ONLY
 * live-poked before (in the cycling/false-negative state) — NEVER baked at init before the US-NIC latches.
 * The IP-mux is exactly where the cpu-tag US-OMCI frame vanishes (RX_OK=0), so bake them quiescent. */
static unsigned int ipmux_neteng = 1;
module_param(ipmux_neteng, uint, 0644);
MODULE_PARM_DESC(ipmux_neteng, "1=bake same-board-diff network-engine IP-mux bits 0x18001000[19]/0x18001098 at reset");
module_param(gmac_reset, uint, 0644);	/* sampled at open(): ifdown/ifup re-applies */
MODULE_PARM_DESC(gmac_reset, "1=cold GMAC bring-up (IP-block reset + stock IO_CMD edge), 0=legacy inherited");

/* TX-DMA stall watchdog: if a ring has published OWN descriptors the HW cursor
 * sits on without fetching and dirty has not advanced for >250ms despite the
 * per-tick doorbell re-kicks, power-cycle the GMAC IP block and re-arm the rings
 * IN PLACE (rotated so TxFDP points at the first pending descriptor - nothing is
 * lost). This is the stock device's only known un-wedge (the GMAC reset path);
 * there is no lighter TX soft-reset on this IP. */
/* Reset-based recovery escalation. Default OFF (see gmac_reset note: resets
 * kill the switch egress on this board). The park watchdog + soft TxFDP
 * re-arm stay active regardless; this only gates the destructive level. */
static unsigned int tx_recover;
module_param(tx_recover, uint, 0644);
MODULE_PARM_DESC(tx_recover, "1=escalate a persistent TX-DMA park to a full GMAC reset (kills switch egress on this board!)");

/* The stock device pokes the network-engine GO (0x18001038[31] + poll-clear) on
 * EVERY submit, not just OMCI injects; mirror it on the LAN xmit path too. */
static unsigned int txgo_xmit = 1;
module_param(txgo_xmit, uint, 0644);
MODULE_PARM_DESC(txgo_xmit, "1=stock per-packet TX GO handshake on the LAN xmit path too");

/*
 * Park-buster (HW-measured behaviour, boot of 2026-06-11): once the TX fetch
 * engine DRAINS a ring it parks on the next (unpublished, OWN=0) descriptor
 * holding a STALE copy of it; publishing OWN later + any number of doorbells
 * never makes it re-fetch — the park is terminal. (Even on a freshly
 * IP-block-reset, stock-configured engine: it consumed exactly the in-flight
 * frames after each recovery, then parked again at the drain point. Bulk never
 * drains mid-stream, which is why only sparse TX wedges.) A full re-ARM forces
 * a fresh fetch: rotate the ring (EOR to the slot before the pending one),
 * re-point TxFDP at the pending slot, zero TxCDO, doorbell. Do this INLINE
 * whenever a frame is published into an EMPTY ring — the engine is then by
 * definition parked on (or done before) that very slot, idle, and re-pointing
 * is race-free. Costs a handful of uncached writes; no IP cycle, no outage.
 */
static unsigned int tx_softrearm = 1;
module_param(tx_softrearm, uint, 0644);
MODULE_PARM_DESC(tx_softrearm, "1=re-arm TxFDP at the pending slot on every empty-ring publish (un-parks the drained fetch engine)");

/* Un-park flavor for the soft re-arm (runtime A/B, no rebuild):
 *   0 = plain FDP/CDO re-point + doorbell (HW result: does NOT un-park a
 *       latched engine; kept as the control case)
 *   1 = additionally wrap the re-point in an IO_CMD TX-enable (bit5 TX_OWN)
 *       OFF->ON edge — a TX-engine-only restart latch, microseconds. */
static unsigned int unpark_mode;	/* HW A/B 2026-06-11: mode 1 showed no
					 * park-rate benefit (33/100s vs 60/150s)
					 * and ran in the boot that hard-hung the
					 * SoC; default to the proven plain mode */
module_param(unpark_mode, uint, 0644);
MODULE_PARM_DESC(unpark_mode, "soft re-arm flavor: 0=FDP re-point only, 1=+IO_CMD bit5 off/on edge");

/*
 * MSR (0x58) top byte. HW-BISECTED 2026-06-11: with 0xf0 (the live-stock O5
 * value, added Jun-10/11 to open the DS-NIC->GMAC RX link) the TX MAC enters
 * the terminal drain-park pathology — sparse TX dies after the ring first
 * empties and nothing software-visible recovers it. With the Jun-5 value 0x10
 * (bit28 only), sparse TX flows perfectly: a 58-publish ping burst produced
 * txok +58 with zero parks on the same never-reset engine. The 0xf0 bits are
 * a TX-killer on our config even though stock runs them; keep 0x10 until the
 * DS-RX dependency is re-bisected bit by bit (runtime-writable, re-applied on
 * open; poke 0x18012058 live for instant A/B).
 */
/* MSR(0x58) top byte. Live working stock runs 0xf0 (FORCE_TRXFCE|RXFCE|TXFCE,
 * flow-control force on the internal GMAC links) — but HW-PROVEN (2026-06-12):
 * with our MINIMAL init, msr_top=0xf0 STALLS LAN TX (ping 0/60), while 0x10
 * keeps LAN healthy (40/40). Stock tolerates 0xf0 because its FULL init sets up
 * flow control properly; our init does not, so 0xf0's forced pause-frame
 * handling wedges TX. This is the load-bearing proof that the US-OMCI gap is an
 * init-COMPLETENESS issue (our minimal init cannot support stock's operating
 * values), not a per-register value we can match. Keep 0x10 so LAN works; the
 * US-NIC RX needs the full stock init sequence, not this single bit. */
static unsigned int msr_top = 0x10;
module_param(msr_top, uint, 0644);
MODULE_PARM_DESC(msr_top, "MSR(0x58) top byte (0x10=LAN-healthy w/ our init; 0xf0=stock value, stalls our LAN)");

/* SW_MAC_CPU_TAG_CTRL (0x23030). 0x300 (TAG_AWARE|TRAP_INSERT) makes the
 * switch parse a cpu-tag on EVERY CPU-port ingress frame — plain LAN frames
 * (no tag) are then eaten (LAN dead even with TX working). Jun-5 ran 0 and
 * LAN worked; the HW-cputag OMCI steering experiments need 0x300. */
/* SW_MAC_CPU_TAG_CTRL (0x23030). REQUIRED for US-OMCI: with TAG_AWARE(bit9) off
 * the switch CPU-port HSB parser never reads the GMAC-inserted cpu-tag's
 * PON_SID/EXTSPA/PSEL, so a descriptor-steered OMCI frame is never direct-TX'd
 * to the US-NIC (it falls through to L2 flood). Stock runs 0x300 permanently
 * (TAG_AWARE | TRAP_TAGET_INSERT_EN), set during its CPU-port init. Default 0x300.
 * Caveat: a PLAIN LAN frame (no cpu-tag) at CPU-port ingress with TAG_AWARE on
 * could be mis-parsed — but our LAN egress path can prepend the SW 0x8899 tag
 * (TX_CPUTAG) if that surfaces; for now bring up US-OMCI (the gating feature). */
static unsigned int sw_tagaware = 0x300;
module_param(sw_tagaware, uint, 0644);
MODULE_PARM_DESC(sw_tagaware, "SW MAC_CPU_TAG_CTRL at open (0x300=cpu-tag parse for US-OMCI, 0=plain-LAN-only)");

/* Recovery flavor: 1 = CMD-register (0x3B) RST soft-reset + full reprogram
 * (microseconds, no IP-block gating) instead of the BSP_IP_SEL power-cycle
 * (~14ms outage, MIB wipe). The RST bit self-clears when the GMAC core
 * finishes resetting (rtl8139 heritage; the bit is defined in the register map
 * but stock never exercises it — hence param-gated, default off until HW-proven). */
static unsigned int recover_rst = 1;
module_param(recover_rst, uint, 0644);
MODULE_PARM_DESC(recover_rst, "1=CMD.RST soft-reset recovery instead of the IP-block power-cycle");

/* GMAC register offsets (from the NIC base). */
#define R_TxFDP1	0x1300	/* TX ring0 fetch-descriptor pointer (phys) */
#define R_TxCDO1	0x1304	/* TX ring0 current-descriptor offset (u16) */
/* Per-ring TX descriptor pointers. The GMAC has six TX descriptor rings; each
 * ring k (0-indexed: 0 = the LAN ring above) has an {FDP,CDO} register pair on a
 * 16-byte stride. Confirmed from the stock device's ring-init behavior (it
 * writes 0x1300/0x1310/0x1320/0x1330/0x1340 for rings 0..4 —
 * stride 16, NOT 8). The US-OMCI path submits on HW ring 4 (a free ring; the LAN
 * owns HW ring 0) so its descriptor base lives at R_TxFDP(4) = 0x1340 and the kick
 * targets the SAME ring (R_IO_CMD1 |= 0x100). HW ring h is consistent across the
 * FDP arm (0x1300 + h*16) and the doorbell — derive both from the one h. */
#define R_TxFDP(k)	(0x1300 + (k) * 16)	/* ring k fetch-descriptor pointer (stock stride 16) */
#define R_TxCDO(k)	(0x1304 + (k) * 16)	/* ring k current-descriptor offset (u16) */
/* RX multi-ring config block at NIC offset 0x1380 + k*16 (RxFDP2 region), filled
 * by the stock device's RX-init (the index 1..5 path = RX
 * rings 1..5; index 0 = the RxFDP/RxCPU-descriptor-count ring-0 path). This is NOT a TX
 * config table — TX ring activation is TxFDP + OWN(opts1) + kick, with no second
 * table. Macro kept only to document the address; our driver uses only RX ring 0. */
#define R_RxMRingCfg(k)	(0x1380 + (k) * 16)	/* RX multiring k config block (stock stride 16) */
#define R_RxFDP		0x13F0	/* RX ring0 fetch-descriptor pointer (phys) */
#define R_RxCDO		0x13F4	/* RX ring0 current-descriptor offset */
/* Live-stock 9602C operating values (read off a running stock ONU at O5). The
 * U-Boot value 0x400f3330 is its polled-TFTP config; the stock OS reprograms
 * IO_CMD/IO_CMD1 after the IP-block reset. Bits[3:0] of IO_CMD stay 0 here (the
 * self-clearing per-ring TX_POLL kicks). */
#define IOCMD_STOCK	0xc059f130
#define IOCMD1_STOCK	0x32000001
#define IOCMD_UBOOT	0x400F3330
#define IOCMD1_UBOOT	0x323F0001
/*
 * SoC per-IP enable (system block 0xb8000600, OUTSIDE the GMAC window; same
 * register the PCIe driver drives as SOC_IP_SEL). The stock GMAC reset path
 * does: clear bit1, ~10ms, set bit1 = GMAC0 IP-block power-cycle.
 * Stock treats this as REQUIRED before ring programming - without it the
 * multi-ring fetch engine's internal state never latches and it parks fatally
 * on sparse TX. KSEG1 is always mapped on MIPS, so address it directly like
 * pcie-rtl9602c.c does.
 */

/*
 * GMAC0 IRQ block. 0x3c/0x3e are 16-bit (RX/TX mask / status); 0xd0/0xd8 are
 * 32-bit (per-ring TX-completion mask / status). On this BE MIPS SoC a 32-bit
 * read at 0x3c returns the IMR in bits[31:16] (live stock golden 0xf8350240 ->
 * IMR=0xf835), so the 16-bit IMR/ISR are accessed via ioread16/iowrite16 at
 * 0x3c/0x3e. Values below are the live-stock operating masks.
 */
#define R_RRING_ROUTING1 0x1370	/* RX-ring routing by priority: PRI_n_ROUTE = ring# at nibble n (operational default 0x65432100). 0 => all priorities to ring 0. */

/* Descriptor opts1 bits (shared TX/RX where noted). */
#define D_IPCS		BIT(27)	/* TX: insert IPv4 csum */

/*
 * TX descriptor CPU-tag fields (the GMAC tx_info layout, selected by the GMAC
 * CPUtagCR = 0x901eff04). The GMAC reads these descriptor bits and INSERTS the
 * on-wire cpu-tag itself; we send a plain frame. Directed egress requires a
 * NON-ZERO tx_portmask — a zero CPU-netdev mask makes the switch fall back to
 * an empty L2 DA lookup and drop the frame (the "HW emits portmask 0" symptom).
 */
#define TXD2_CPUTAG	BIT(31)		/* opts2: descriptor carries cpu-tag fields */
#define TXD2_PMASK_SHIFT 16		/* opts2: tx_portmask occupies bits 26..16 */
#define TXD3_KEEP	BIT(23)		/* opts3: switch must not modify the frame */
#define TXD3_DISLRN	BIT(21)		/* opts3: do not learn the CPU SA */
#define TXD3_L34_KEEP	BIT(17)		/* opts3: do not L3/L4-filter the injected frame */
/* Egress port bitmask for CPU->LAN. RTL9602C port map: port 0 = FE LAN (100M),
 * port 1 = GE LAN (1000M), port 2 = PON/fiber, port 3 = CPU. Egress to the two
 * LAN jacks (0,1) only — NOT port 2 (PON, would go to the OLT) and NOT port 3
 * (CPU). The earlier 0x2f targeted PON + a nonexistent port 5. */
#define TXD_EGRESS_PMASK 0x3

/* SoC NIC-DMA bus window: the bootloader ORs 0x20000000 into ring/desc addrs
 * (an artifact of its 1:1 map). Observed to be a NO-OP for TX egress and to
 * DEGRADE RX (Linux dma_alloc_coherent already yields correct bus addrs;
 * OR-ing the window corrupts them). Set 0 to disable — kept as a named knob. */
/* DMA_BUS_WINDOW is the FAMILY's -- luna_eth_regs.h, with the measurement
 * that explains why it is zero and must stay a named knob. */

#define OTX_RING_SIZE	8	/* dedicated US-OMCI TX ring (low-rate control) */
#define OMCI_RESV	2	/* LAN xmit stops this many slots early so the sparse shared-ring OMCI inject always has room (never dropped) */
#define DUMMY_RING_SIZE	4	/* idle filler armed on the unused HW TX rings (gap rings) */
#define RX_CPU_PREFIX	2	/* switch CPU-port prepends a 2-byte offset word on RX */
/*
 * TX_CPUTAG selects the CPU->switch egress method:
 *   1 = prepend the software 0x8899 cpu-tag and rely on the switch TAG_AWARE
 *       parser to do directed egress per word3 portmask (rtl8_4 model).
 *   0 = send a PLAIN frame (no tag); switch floods/forwards by DA within VID1
 *       (TAG_AWARE off, VLAN filtering on). Does not depend on this SoC's
 *       cpu-tag format. Diagnostic: method 1's frames never reach the host on
 *       any port (directed egress reads portmask 0) — this SoC's cpu-tag layout
 *       differs from mainline rtl8_4.
 */
#define TX_CPUTAG	0
#define POLL_INTERVAL	msecs_to_jiffies(2)	/* legacy pure-poll fallback (ep->irq<=0) */
#define REKICK_INTERVAL	msecs_to_jiffies(100)	/* slow TX-unpark backstop when IRQ-driven */

/* struct rx_desc / struct tx_desc are the FAMILY's -- luna_eth_regs.h.
 * They were declared identically in both drivers; one type now. */

/*
 * SoC switch core (SWCORE), phys 0x1B000000. The switch has 4 ports (0-3); the
 * CPU port (where GMAC0 attaches) is port 3. Flood masks have one bit per port.
 */
/* per-port forced-ability value + force-mode (RTL9602C register map: base 0x180
 * / 0x1B4, stride 4). FORCE_P_ABLTY holds speed/duplex/link; ABLTY_FORCE_MODE
 * = 0xFFF forces all of them. */
/* The BASES are this chip's entry in luna_sw_map now -- a per-chip value
 * belongs in the table, not in a #define only one driver can see. */
/* ★ `ep` IS AN EXPLICIT ARGUMENT, not captured from the call site.  The first
 * cut of this macro read `ep->swm->...` implicitly: it compiled, because every
 * caller happened to have an `ep`, and it would have broken the first time
 * somebody used it where the pointer is called anything else -- with an error
 * naming a variable the macro's own text never mentions. */
#define SW_FORCE_P_ABLTY(ep, p)	((ep)->swm->force_ablty + ((p) << 2))
#define SW_ABLTY_FORCE_MODE(ep, p)	((ep)->swm->ablty_force + ((p) << 2))
#define SW_SYS_LRN_LIMITNO	0x17018	/* system MAC-learn limit [10:0]; 0 = no learning */
#define SW_DLF_ACT_TRAP2CPU	2
/* Forced ability value: 1000M (speed[1:0]=2) + full duplex (b2) + link-up (b4) */
#define SW_ABLTY_1G_FD_UP	(0x2 | BIT(2) | BIT(4))
/* MAC_CPU_TAG_CTRL: TAG_AWARE[9] makes the switch parse the CPU-tag on CPU-port
 * ingress and STRIP it before physical egress; TRAP_TAGET_INSERT_EN[8] inserts
 * a CPU-tag on frames trapped to the CPU. */
#define SW_MAC_CPU_TAG_CTRL	0x23030
/*
 * ★★ THE THREE "AUX" WORDS ARE FLOW-CONTROL THRESHOLDS, NOT CPU-TAG REGISTERS
 * (tier 3, 2026-08-29: the RTL9602C's OWN chipdef, rtk_rtl9602c_reg_list.c).
 * They were written here as magic copied off a working stock board, described
 * only as "Aux regs (live-stock)", and the family's per-chip table names two of
 * these very addresses cpu_tag_insert / cpu_tag_aware -- for the RTL9607C.
 *
 *   0x23030  MAC_CPU_TAG_CTRL     (this driver already had it right)
 *   0x230F0  FC_P_LO_TH           per-port flow-control LOW threshold
 *   0x230F4  FC_P_FCOFF_HI_TH     flow-control OFF, HIGH threshold
 *   0x230F8  FC_P_FCOFF_LO_TH     flow-control OFF, LOW threshold
 *   0x23040  CFG_UNHIOL           -- and THIS is why the 9607C names must never
 *                                    be adopted here: that is where the family
 *                                    table puts cpu_tag_aware for the 9603CVD.
 *
 * Same address, different silicon, different register.  Adopting the sibling's
 * name would have been the CFG_PHY_CTRL defect again -- our own driver
 * corrupting a register because a matching number was read as a matching
 * meaning.
 *
 * ⚠ THE VALUES ARE UNCHANGED AND STAY VERBATIM.  Naming a register is not
 * understanding its value: 0x00400034 / 0x00f000ea / 0x00400034 were measured on
 * a board that works, and nothing here has yet derived them from page counts.
 * What changes is that the next reader knows WHAT they configure.
 */
#define SW_FC_P_LO_TH		0x230F0
#define SW_FC_P_FCOFF_HI_TH	0x230F4
#define SW_FC_P_FCOFF_LO_TH	0x230F8
#define SW_TAG_AWARE		BIT(9)
#define SW_TRAP_TAG_INSERT_EN	BIT(8)
/* VLAN filtering: VLAN_CTRL @ 0x13008 bit0 = VLAN_FILTERING; VLAN_INGRESS @
 * 0x13004 = per-port ingress filter. The operational config enables both + a
 * default VLAN; for flat bring-up we DISABLE them so a parsed cpu-tag's directed
 * egress is not dropped by VLAN membership checks (we have no VLAN table set up). */
#define SW_VLAN_INGRESS		0x13004
#define SW_VLAN_FILTERING	BIT(0)
/* Operational value: VLAN_CTRL=0x19 (filtering + VID0/VID4095 type bits). */
#define SW_VLAN_CTRL_VAL	0x19	/* VLAN filtering ON at init — required during ranging/config-apply for
					 * reliable onlining (VLAN-off cold boots failed config-apply 4x; VLAN-on
					 * onlines + stays stable). BUT with filtering ON this switch does NOT pass
					 * LAN port<->CPU traffic (ping 192.168.1.1 fails), so LAN management access
					 * needs filtering OFF. HYBRID (gpon-rtl960x.c gpon_fsm_poll): keep 0x19 for
					 * config, then auto-clear bit0 (filtering off) once the ONU is stably at O5
					 * (config done) to open LAN; re-assert on any re-range. Proven viable: online
					 * with 0x19 then poke 0x13008=0 -> stays online 6h + LAN reachable. */
#define SW_DEFAULT_VID		1
/* Indirect VLAN 4k-table access (field positions):
 * TBL_ACCESS_CTRL[31]=start [20:9]=addr/VID [6:4]=method(1) [3]=cmd(1=write)
 * [2:0]=type(1=VLAN); STS bit13=BUSY; WR_DATA holds the entry word. */
#define SW_TBL_CTRL		0x12000
#define SW_TBL_STS		0x12004
#define SW_TBL_WRDATA		0x12008
#define SW_TBL_BUSY		BIT(13)
#define SW_TBL_VLAN_WR(vid)	(BIT(31) | (((vid) & 0xfff) << 9) | (1u << 4) | (1u << 3) | 1u)

/* OMCI (OMCC) trap. The GTC de-encapsulates DS OMCI GEM frames and delivers them
 * to the CPU port tagged with rx-reason OMCI from the PON port; CPUTAG1CR[14:8]
 * selects which DS stream-id the GMAC traps to the CPU. */
/* RTL9602C_OMCI_REASON (rx-reason 246 = OMCI) moved to rtl9602c_l34_logic.h
 * with the DS-OMCI classifier that consumes it (hoisted logic). */
#define CPUTAG1_OMCI_SID(s)	(((s) & 0x7f) << 8)	/* R_CPUTAG1CR[14:8] */
#define CPUTAG1_B1		0x2	/* bit1: live 9602C stock reads CPUTAG1CR=0x4002; not present in the 9607C register map */
/* Stock NIC init writes CPUTAG1CR = (SID<<8) | 0x70 (bits 4/5/6 = the
 * cpu-tag format/enable that make the GMAC actually PREPEND the on-wire tag the
 * switch HSB parser reads). Observed in the stock device's init (it ORs in
 * 0x4070). The earlier 0x02 (bit1 only) was a misread of a runtime snapshot. */
#define CPUTAG1_LOW		0x02	/* live-stock ref ONU devmem: CPUTAG1CR = 0x4002 (the earlier 0x4070 derivation was wrong) */

/* The switch L34 (NAPT) offload module is compiled in via this driver's TU to
 * avoid a separate Kbuild object; it carries its own header include guard. */
#include "rtl9602c_l34.c"
#ifdef CONFIG_GPON_FLOW_OFFLOAD
#include "gpon_flow_offload.h"	/* the core TC-offload lifecycle */
#endif

/* HW NAT (switch L34 offload) gate. Module-param gated (not Kconfig) so it can
 * be toggled without an OpenWrt kernel-config round-trip. Default off: engine
 * init is validated datapath-safe (WAN + internet stay 0% loss when enabled),
 * but per-flow offload is still being built, so the stock path is software
 * forwarding until rtl9602c_eth.hw_nat=1 is set. */
static int hw_nat = 0;	/* default off (gated); engine armed lazily on first offload, never at boot */
module_param(hw_nat, int, 0444);
MODULE_PARM_DESC(hw_nat, "enable RTL9602C switch L34 hardware NAT offload (0=off)");

struct rtl9602c_eth {
	/* ★ THE PER-CHIP TABLE.  The switch LUT block MOVED between Luna
	 * revisions, and one chip's constant on another's silicon does not
	 * fault -- it configures the wrong behaviour (see luna_eth_regs.h).
	 * Reaching it through a pointer is what lets a shared function body
	 * exist at all: a body that names a #define is a body that belongs to
	 * one chip. */
	const struct luna_sw_map *swm;
	void __iomem	*base;
	void __iomem	*sw;	/* switch core */
	void __iomem	*txgo;	/* network-engine TX-fetch page 0x18001000; +0x38 bit31 = per-packet GO */
	struct net_device *ndev;
	struct net_device *wan_ndev;	/* gpon0: WAN data-GEM netdev (DS demux'd from PON port 2, US steered to GPON_DATA_FLOW) */
	struct device	*dev;

	struct rx_desc	*rx_ring;
	dma_addr_t	rx_ring_dma;
	struct sk_buff	*rx_skb[RX_RING_SIZE];
	dma_addr_t	rx_buf_dma[RX_RING_SIZE];
	unsigned int	rx_head;

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
	/* TX-DMA park watchdog (see tx_recover). stall_since = jiffies of the
	 * first tick that saw OWN stuck at the HW cursor with no dirty progress;
	 * 0 = healthy. recover_work runs the IP-block power-cycle + re-arm. */
	unsigned long	stall_since;
	unsigned int	stall_lastdirty;
	unsigned int	stall_level;	/* 0: next escalation = soft re-arm;
					 * 1: soft re-arm failed -> IP cycle */
	u32		dbg_rearm;	/* soft TxFDP re-arms (publish + watchdog) */
	struct work_struct recover_work;
	bool		closing;	/* gate recover_work vs ndo_stop teardown */
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
	/* Idle dummy TX ring armed on the UNUSED HW TX rings (the TxFDP gaps between
	 * the LAN ring (HW0) and the OMCI ring (HW4)). When OMCI kicks the GMAC0
	 * multi-ring TX scheduler (IO_CMD1|0x100), the DMA engine walks ALL HW TX
	 * rings; any ring left with an UNPROGRAMMED TxFDP makes it DMA from a stale
	 * base and stalls the shared TX path -> the LAN CPU datapath wedges (measured:
	 * http 2/120s once OMCI TX active). Stock arms all 5 TxFDP rings for this.
	 * The dummy descriptors stay OWN=0 (CPU-owned) so nothing is ever sent. */
	struct tx_desc	*dummy_ring;
	dma_addr_t	dummy_ring_dma;
	/* Shared-ring-0 OMCI test path (omci_tx_ring==0): the OMCI frame is
	 * enqueued on the LAN ring (ep->tx_ring) so it rides the PROVEN-fetching
	 * HW ring 0, isolating whether the corrected OMCI descriptor STEERING
	 * alone routes to the PON US-NIC (independent of the "ring 4 won't fetch"
	 * problem). Track the last OMCI slot we put on ring 0 so the diag can show
	 * its OWN bit. -1 = none submitted yet on ring 0. */
	int		omci_r0_last_slot;
	/* US OMCI (OMCC) responder state. */
	u8		omci_sn[8];	/* G.984.3 ONU-SN (4 ASCII ID + 4 serial),
					 * for the ONU-G Vendor-ID/Serial GET reply */
	u8		omci_mds;	/* ONU-data (ME 2) MIB-Data-Sync counter */
	/* ⚠ omci_audit_reads and omci_mds_tries USED TO BE HERE and were dead.
	 * The adaptive MIB-Data-Sync walk they served moved into the common
	 * responder, which carries both under the SAME names with the same
	 * comments (gpon_omci_me.h) and actually increments and resets them
	 * (gpon_omci_core.c).  These two were left behind by that move -- never
	 * written, never read, never published: a counter that always reads 0,
	 * which is the phantom this tree has been burned by before.  Found by
	 * unread_member_guard.py, not by anything the compiler can see. */
	u32		dbg_omci_tx;		/* US OMCI responses queued */
	u32		dbg_omci_tx_drop;	/* dropped: ring full / alloc / map */
	u32		dbg_omci_unhandled;	/* requests with no modelled reply */

	struct timer_list poll_timer;	/* IRQ-driven: 100ms TX-unpark backstop; no IRQ: 2ms pure-poll */
	struct napi_struct napi;
	int		irq;	/* GMAC0 INTC input (platform_get_irq); <=0 = pure-poll fallback */
	struct rtl9602c_l34 l34;	/* switch L3/L4 (NAPT) hardware-offload engine */
#ifdef CONFIG_GPON_FLOW_OFFLOAD
	struct gpon_flow_offload *fo;	/* the COMMON TC lifecycle, drivers/net/gpon/ */
#endif
	/* Host uplink port, learned from the RX descriptor src_port_num. All RX
	 * arrives on the board's single connected LAN port, so this resolves to the
	 * physical switch port the host is on — we then steer CPU->LAN TX there
	 * regardless of the (ambiguous) static port numbering. 0xff = not yet seen. */
	unsigned int	host_port;

	/* Bootloader GMAC0 control snapshot (inherited). */
	u32		ub_tcr, ub_rcr, ub_config, ub_cputagcr, ub_cputag1cr;
	u32		ub_iocmd, ub_iocmd1;

	/* RX datapath debug counters (see /proc/rtl9602c_diag). */
	u32		dbg_poll;	/* poll-timer ticks */
	u32		dbg_filled;	/* RX descriptors HW handed back (D_OWN cleared) */
	u32		dbg_good;	/* frames pushed up the stack */
	u32		dbg_err;	/* RX descriptors with error/oversize */
	u32		dbg_rxlen;	/* raw length of the last captured RX frame */
	u8		dbg_rxbuf[48];	/* first bytes of the last RX frame (pre-pull) */

	/* OMCI (OMCC stream) trap, armed by the GPON driver once it installs the
	 * OMCC GEM datapath (rtl9602c_eth_set_omci_sid). */
	bool		omci_trap_on;
	u32		dbg_omci_rx;	/* DS OMCI frames trapped to the CPU */
	u32		dbg_omci_rxlen;	/* length of the last OMCI frame */
	u8		dbg_omci_rxbuf[48];	/* the last DS OMCI baseline message */
};

static struct rtl9602c_eth *g_ep;	/* single-instance, for /proc diag */

static inline u32 ep_rd(struct rtl9602c_eth *ep, u32 r) { return ioread32(ep->base + r); }
static inline void ep_wr(struct rtl9602c_eth *ep, u32 r, u32 v) { iowrite32(v, ep->base + r); }

/*
 * SW-follows-HW slot mapping. TxCDO is NOT writable while the engine runs, so
 * the engine's walk position after open is base + CDO_inherited — it can NOT
 * be forced back to slot 0, and every attempt to re-point TxFDP/rotate EOR
 * around a live engine produced off-by-CDO skips (HW-measured: own=0x2, the
 * oldest slot skipped, reclaim wedged). Instead the SOFTWARE producer aligns
 * to the ENGINE: tx_rot/otx_rot = the engine position read ONCE at open;
 * every slot derivation is (rot + counter) % size; TxFDP stays at the ring
 * base and D_EOR stays on the last PHYSICAL slot forever. The engine then
 * finds every published OWN descriptor exactly where it is already looking,
 * and (with a sane MSR, see msr_top) consumes sparse frames on the plain
 * doorbell with no parks — 58/58 and 30/30 consecutive sparse transmits
 * measured once alignment held.
 */
static inline unsigned int tx_slot(struct rtl9602c_eth *ep, unsigned int counter)
{
	return (ep->tx_rot + counter) % TX_RING_SIZE;
}

static inline unsigned int otx_slot(struct rtl9602c_eth *ep, unsigned int counter)
{
	return (ep->otx_rot + counter) % OTX_RING_SIZE;
}

static inline unsigned int tx_eor_slot(struct rtl9602c_eth *ep)
{
	return TX_RING_SIZE - 1;	/* wrap descriptor: fixed physical slot */
}

static inline unsigned int otx_eor_slot(struct rtl9602c_eth *ep)
{
	return OTX_RING_SIZE - 1;
}

/*
 * Align the SW producers to the live engine positions. Call ONCE per arm
 * (open, or after a recovery reset with the engine stopped): with TxFDP at
 * the ring base, the engine's next fetch is base + CDO, so the SW slot offset
 * is simply the CDO value. After this, never touch FDP/CDO/EOR again — the
 * producer walks to the engine, not the other way around.
 */
static void rtl9602c_tx_align(struct rtl9602c_eth *ep)
{
	ep->tx_rot = ioread16(ep->base + R_TxCDO1) % TX_RING_SIZE;
	ep->otx_rot = 0;
	if (omci_tx_ring) {
		unsigned int h = rtl9602c_omci_hwring(omci_tx_ring);

		ep->otx_rot = ioread16(ep->base + R_TxCDO(h)) % OTX_RING_SIZE;
	}
	ep->dbg_rearm++;
}

/*
 * Arm the GMAC OMCI trap so DS frames on stream-id `sid` (the OMCC) are delivered
 * to the CPU netdev instead of switched. Called by the GPON driver AFTER it has
 * installed the OMCC GEM datapath (NOT at NIC init — arming the trap before the
 * datapath exists was an earlier regression). CPUTAG1CR[14:8] = OMCI SID.
 */
void rtl9602c_eth_set_omci_sid(unsigned int sid)
{
	struct rtl9602c_eth *ep = g_ep;

	if (!ep)
		return;
	/* cpu-tag (CPUTAGCR/CPUTAG1CR) is armed once in open(); here only software
	 * state + RX-ring routing. The chip's OMCC SID is the fixed RTL9602C_OMCC_SID,
	 * latched into CPUTAG1CR at IO_CMD-enable time, so there is nothing to program
	 * for the cpu-tag insert engine at O5. */
	/* CONFIG_REG(0x4c): leave at the inherited 0x21000000. A live online stock ONU
	 * traps OMCI to the CPU with 0x4c = 0x21000000 (sideband bits 22/23 CLEAR), so the
	 * earlier "the controller requires config_rx_sideband on" guess was wrong — setting
	 * bits 22/23 diverged from the proven-working stock config. Do not touch 0x4c. */
	/* Route EVERY RX class to ring 0 (the only ring this driver allocates and
	 * drains). There are SEVEN routing tables (RRING_ROUTING1..7 @ 0x1370..0x1388,
	 * stride 4) selected by source/priority; OMCI's class may use one other than
	 * table 1, and an un-zeroed table sends it to an un-drained ring (the frame
	 * sticks in PON-IP -> US stall -> deactivate). Zero all 7 -> everything to ring 0
	 * (routing value 0 = ring 0, the table the LAN low-priority traffic already uses). */
	/* RRING routing: stock uses ROUTING1=0x65432100 because it sets up SIX RX rings
	 * and steers traffic by priority across them. My driver allocates only ring 0, so
	 * route EVERY priority to ring 0 (all nibbles 0) — otherwise a GMII-RX OMCI frame
	 * at a nonzero priority is steered to an unallocated ring and dropped. */
	{
		unsigned int r;
		for (r = 0; r < 7; r++)
			ep_wr(ep, R_RRING_ROUTING1 + r * 4, 0);
	}
	ep->omci_trap_on = true;
	netdev_dbg(ep->ndev, "OMCI trap armed: SID %u (cpu-tag CPUTAGCR/CPUTAG1CR armed at open(); RX-ring routing zeroed)\n",
		    sid);
}
EXPORT_SYMBOL(rtl9602c_eth_set_omci_sid);

/*
 * Provision the ONU identity (G.984.3 ONU-SN: 4 ASCII vendor + 4 serial bytes)
 * so the OMCI ONU-G (ME 256) GET reply reports a Vendor-ID/Serial matching the
 * PLOAM Serial_Number the OLT ranged. Called by the GPON driver once onu_sn is
 * parsed; the responder copies SN[0..3] as Vendor-ID and SN[0..7] as Serial.
 */
void rtl9602c_eth_set_omci_identity(const u8 *sn8)
{
	if (g_ep && sn8)
		memcpy(g_ep->omci_sn, sn8, sizeof(g_ep->omci_sn));
}
EXPORT_SYMBOL(rtl9602c_eth_set_omci_identity);

/* DS OMCI frames that actually reached the CPU NIC ring (private dbg counter). */
u32 rtl9602c_eth_omci_rx_count(void)
{
	struct rtl9602c_eth *ep = g_ep;

	return ep ? ep->dbg_omci_rx : 0;
}
EXPORT_SYMBOL(rtl9602c_eth_omci_rx_count);

/* gpon0 (WAN) RX packet count. 0 = the OLT has forwarded us NO downstream data
 * (no DHCP OFFER, no unicast, not even via the demux) = we are not provisioned /
 * "Laser out". Used by the GPON O5 provisioning watchdog to detect a stuck link
 * that reached O5 locally but the OLT never provisioned (so it can self-re-range
 * and re-roll the non-deterministic US-TX serializer phase). */
u32 rtl9602c_eth_wan_rx_count(void)
{
	struct rtl9602c_eth *ep = g_ep;

	return (ep && ep->wan_ndev) ? (u32)ep->wan_ndev->stats.rx_packets : 0;
}
EXPORT_SYMBOL(rtl9602c_eth_wan_rx_count);

/*
 * US-OMCI TX-ring "dirty" cursor = number of OMCC descriptors the HW has consumed
 * (OWN cleared) and our reclaim has retired. Non-zero => the dedicated HW ring is
 * actually fetching the descriptors we publish (the /proc 'dirty=' field, surfaced
 * here so the periodic O5 serial line can report it despite a flaky LAN). For the
 * shared-LAN-ring-0 path (omci_tx_ring==0) the OMCI rides the LAN ring, so report
 * the LAN ring's reclaim cursor instead.
 */
u32 rtl9602c_eth_omci_tx_dirty(void)
{
	struct rtl9602c_eth *ep = g_ep;

	if (!ep)
		return 0;
	return (omci_tx_ring == 0) ? ep->tx_dirty : ep->otx_dirty;
}
EXPORT_SYMBOL(rtl9602c_eth_omci_tx_dirty);

/*
 * Minimal switch bring-up: permit ingress from every port and flood
 * broadcast / unknown unicast+multicast to every port (incl. the CPU port).
 * All-ports masks avoid depending on the exact CPU port number. Idempotent
 * (OR-in). Without this, the bootloader-left switch state only forwarded its
 * own TFTP unicast flow and ingress never reached the CPU GMAC.
 */
static void rtl9602c_sw_min_init(struct rtl9602c_eth *ep)
{
	if (!ep->sw)
		return;

	/* --- Stock switch-init prerequisites (full-init-sequence equivalents) ---
	 * These are concrete register writes the stock switch init does that our
	 * minimal init omitted; identified from the chip's register/field map. They set
	 * up switch-side state the cpu-tag PSEL direct-TX path is gated on. */

	/* (a) PONPBO IP clock/reset enable — SoC 0x1800063C bit5 (KSEG1 0xB800063C);
	 * the switch IP-enable step. Stock does this FIRST in switch init: it
	 * is the prerequisite that makes the whole PON-IP US/DS NIC + PBO datapath
	 * reachable. Our minimal init never touched it (we only ever compared
	 * 0x18000600..0x624, never 0x63C). Read-modify-write, OR bit5. */
	/* The family's name, not a bare address: this is the same word and the
	 * same bit luna_eth.c and gpon-rtl960x.c both set. */
	writel(readl(SOC_SW_ENABLE) | SW_EN_BIT, SOC_SW_ENABLE);

	/* (b) CFG_UNHIOL.IPG_COMPENSATION — swcore 0x23040 bit0. ORACLE-CONFIRMED real
	 * value diff (ours read 0xa8, working stock 0xa9) that demonstrably changed the
	 * OMCI forwarding class when poked live. The stock switch init sets it. */
	if (ep->swm->cfg_unhiol)
		iowrite32(ioread32(ep->sw + ep->swm->cfg_unhiol) | BIT(0),
			  ep->sw + ep->swm->cfg_unhiol);

	/* (c) WRAP_GPHY_MISC.PATCH_PHY_DONE — swcore 0x110 bit0: the "switch ready /
	 * PHY patch done" latch the stock switch init asserts at completion. Without it
	 * the switch may not present itself as fully initialised to the direct-TX
	 * forwarding class. */
	if (ep->swm->gphy_misc)
		iowrite32(ioread32(ep->sw + ep->swm->gphy_misc) | BIT(0),
			  ep->sw + ep->swm->gphy_misc);

	/* Force CPU port (3) + both LAN ports (0=FE,1=GE) link-up (the switch will
	 * not egress to a port it thinks is link-down), permit all-port ingress at
	 * the CORRECT 9602C address (0x1C088), and open port isolation. */
	/* Force NO port. The bootloader's WORKING config (its TFTP egresses the GE
	 * host port) runs with FORCE_P_ABLTY=0 and ABLTY_FORCE_MODE=0 for EVERY port
	 * including the CPU port (P3), with P_ABLTY status=0x60 (auto-linked) on all.
	 * Force-up of the CPU port to a fixed 1000M overrides that auto-linked state
	 * and kills CPU->LAN egress (MIB: all LAN-port TX=0). Leave every port at its
	 * auto-negotiated reset state, as the bootloader does. */
	/* L2_SRC_PORT_PERMIT (0x1C088), INVERTED polarity: EN=1 PERMITS a frame to
	 * egress its OWN ingress port (reflection); EN=0 (reset default) SUPPRESSES it.
	 * Writing 0xffffffff (permit=1) was the ROOT CAUSE of the broadcast loop — the
	 * board reflected the host's broadcasts back out the GbE ("own address as
	 * source"), disrupting the LAN. Write 0 to suppress source-port egress.
	 * Cross-port forwarding (host port1 <-> CPU port3) is unaffected (different
	 * ports). PISO 0x27000 is an 11-bit positive egress matrix (reset 0x3FFFFF =
	 * forward to all); leave it at reset — do NOT write 0xffffffff (reserved bits). */
	iowrite32(0, ep->sw + ep->swm->src_permit);
	/* Flood masks: ports 0,1 (LAN) + 3 (CPU), but NOT port 2 (PON). Behavioral note:
	 * stock excludes the PON port from BC/MC/unknown-UC flood so a cpu-tagged US OMCI
	 * that the parser did not direct-TX has NO flood escape to switch-p2 — it is forced
	 * down the PSEL direct-TX path to the US-NIC (our measured symptom was OMCI
	 * L2-flooding to p2 instead). LAN/CPU flood (host BC/ARP/DHCP reach the CPU) is
	 * preserved; DS ingress from p2 and directed US GEM data are unaffected. */
	iowrite32((ioread32(ep->sw + ep->swm->bc_flood) | ep->swm->port_mask) & ~BIT(ep->swm->pon_port),
		  ep->sw + ep->swm->bc_flood);
	iowrite32((ioread32(ep->sw + ep->swm->unkn_mc_flood) | ep->swm->port_mask) & ~BIT(ep->swm->pon_port),
		  ep->sw + ep->swm->unkn_mc_flood);
	iowrite32((ioread32(ep->sw + ep->swm->unkn_uc_flood) | ep->swm->port_mask) & ~BIT(ep->swm->pon_port),
		  ep->sw + ep->swm->unkn_uc_flood);
	/* Unknown-unicast that misses the L2 lookup (e.g. the host's NDP/ARP
	 * reply to the not-yet-learned CPU MAC) must reach the CPU. Flooding via
	 * uc_flood alone proved ineffective for unicast DLF on this switch, so set
	 * the per-port destination-lookup-failure action to trap-to-CPU. Each port
	 * is a 16-bit field at 2-byte stride with ACT in bits[1:0]; one 32-bit
	 * write covers two ports. */
	iowrite32((SW_DLF_ACT_TRAP2CPU << 16) | SW_DLF_ACT_TRAP2CPU,
		  ep->sw + ep->swm->lut_unkn_uc_da);		/* ports 0,1 */
	iowrite32((SW_DLF_ACT_TRAP2CPU << 16) | SW_DLF_ACT_TRAP2CPU,
		  ep->sw + ep->swm->lut_unkn_uc_da + 4);		/* ports 2,3 */
	/* NOTE: 0x27000 (PISO) is a 5-bit isolation-vector INDEX per port, NOT a
	 * direct portmask — writing 0xffffffff selected index 0x1f and likely blocked
	 * CPU->LAN forwarding. The bootloader leaves it at default (TX works), so we
	 * do too. */
	/* NOTE: the SWITCH cpu-tag engine writes (MAC_CPU_TAG_CTRL=0x300 TAG_AWARE +
	 * the three FC_P_* flow-control thresholds) MOVED to rtl9602c_eth_open(), armed BEFORE R_IO_CMD
	 * enables TX — mainline rtl8365mb arms the cpu-tag engine before the
	 * TX/forwarding path; arming it here (post-TX-enable) left CPU-port-3 ingress
	 * unable to parse the cpu-tagged US OMCI. */
	/* VLAN forwarding domain. CPU->LAN egress is VLAN-DIRECTED, not flooded: the
	 * operational config runs with VLAN filtering ON (VLAN_CTRL=0x19) + per-port
	 * service VLANs, and every flat / no-VLAN test gave 0 egress. Create a default
	 * VLAN (VID 1) whose member + untag masks are ALL ports (CPU + LAN), point
	 * every port's PVID at it, accept all frame types, enable per-port ingress
	 * filter, then enable the VLAN function. A parsed cpu-tag's directed egress
	 * then lands in a VLAN the target LAN port belongs to instead of being
	 * filtered/dropped. */
	{
		int p, to;
		/* 4k-table entry: untag[7:4]=0xf | mbr[3:0]=0xf (all four ports) */
		iowrite32((0xf << 4) | 0xf, ep->sw + SW_TBL_WRDATA);
		iowrite32(SW_TBL_VLAN_WR(SW_DEFAULT_VID), ep->sw + SW_TBL_CTRL);
		for (to = 0; to < 1000 &&
			    (ioread32(ep->sw + SW_TBL_STS) & SW_TBL_BUSY); to++)
			udelay(1);
		iowrite32(0, ep->sw + SW_VLAN_ACCEPT);	/* accept all frame types */
		for (p = 0; p < 4; p++)
			iowrite32((ioread32(ep->sw + SW_VLAN_PB_VID + p * 4) & ~0xfffu) |
					  SW_DEFAULT_VID,
				  ep->sw + SW_VLAN_PB_VID + p * 4);
		iowrite32(0xf, ep->sw + SW_VLAN_INGRESS);	/* ingress filter, ports 0-3 */
		iowrite32(SW_VLAN_CTRL_VAL, ep->sw + SW_VLAN_CTRL); /* enable VLAN function */
	}
	/*
	 * Force the CPU port (3) link UP. CRITICAL: the bootloader leaves
	 * FORCE_P_ABLTY[3] with the LINK bit (bit4) CLEAR (value 0x186) while
	 * ABLTY_FORCE_MODE[3] forces ALL ability bits (0xfff) — so the switch treats
	 * the CPU port as link-DOWN and refuses to forward any frame to it. Result:
	 * the GMAC RX DMA never receives (CPU RX = 0) and CPU-originated TX never
	 * egresses. Writing FORCE_P_ABLTY[3] = 0x16 (speed 1000M | full-duplex | LINK)
	 * immediately starts RX. The LAN jack ports stay auto-negotiated (real PHYs);
	 * only the internal MAC<->MAC CPU port must be force-linked.
	 */
	iowrite32(SW_ABLTY_1G_FD_UP, ep->sw + SW_FORCE_P_ABLTY(ep, ep->swm->cpu_port));
	iowrite32(0xfff, ep->sw + SW_ABLTY_FORCE_MODE(ep, ep->swm->cpu_port));

	/* Accept short (runt) frames on the PON port (2) and CPU port (3). A DS OMCI
	 * frame is the 48-byte G.988 baseline + headers = ~60 bytes, BELOW the 64-byte
	 * Ethernet minimum, so the switch runt-filters it unless RX_SPC (P_MISC bit2) is
	 * set. Stock sets RX_SPC per-port. P_MISC = 0x020004 + port*0x20; bit2 =
	 * RX_SPC. Without it the de-encapsulated OMCI never reaches the CPU. */
	/* P_MISC = 0x20004 + port*0x400 (the chip's per-port register interval is 0x400 —
	 * the old 0x20-stride wrote unrelated registers and RX_SPC never landed). */
	iowrite32(ioread32(ep->sw + 0x20804) | BIT(2), ep->sw + 0x20804); /* port 2 (PON) */
	iowrite32(ioread32(ep->sw + 0x20C04) | BIT(2), ep->sw + 0x20C04); /* port 3 (CPU) */

	/*
	 * Force the PON port (2) link UP for the SAME reason as the CPU port: the
	 * PON-IP/PONNIC connects to switch port 2 over an internal MAC<->MAC GMII with
	 * no auto-negotiating PHY, so the bootloader leaves FORCE_P_ABLTY[2] link-DOWN
	 * and the switch refuses to forward de-encapsulated downstream frames (GEM data
	 * + OMCI,
	 * stream-id 64) from the PON port into the fabric — they never reach the CPU
	 * port and the GMAC RX ring stays empty (filled=0) despite the PON-IP DS
	 * datapath being fully up. Forcing 1000M/FD/LINK opens the PON->CPU path.
	 */
	iowrite32(SW_ABLTY_1G_FD_UP, ep->sw + SW_FORCE_P_ABLTY(ep, ep->swm->pon_port));
	/*
	 * Force P2 link UP (0xfff). This is REQUIRED for the DS path (PON->CPU forward).
	 * Tested the stock-style auto-link (0xfef, clearing FORCE_LINK_ABLTY) on HW:
	 * DS BROKE (OMCI responder resp -> 0, no DS OMCI reached the CPU) and US was NOT
	 * fixed (us_rxsid still 0). So the P2 force-link is load-bearing for DS and is
	 * NOT the US-egress blocker -- do not remove it.
	 */
	iowrite32(0xfff, ep->sw + SW_ABLTY_FORCE_MODE(ep, ep->swm->pon_port));
}

/* ★ THE FIRST RUNG of the declared precedence (bootarg -> DT/nvmem -> random
 * LAA), and on this product it is the only one that can carry a PER-UNIT value:
 * no DTS here declares `mac-address`, and the real address is zlib-compressed
 * in the vendor MIB.  The suite hands it in from the BOARD's own declaration
 * (`rig/bootmode.py:_mac_bootarg`, this board's ETH_MAC_BOOTARG).
 *
 * ⚠ IT WAS MISSING, AND THAT IS HALF OF WHY THIS BOARD SHIPPED THE FAMILY'S
 * BRING-UP DEFAULT.  The sibling driver has had this rung since 2026-08-24; the
 * RTL9602C had no way at all to be told its own address, so refusing the
 * default without adding this would only have traded a stable wrong address for
 * an unstable one. */
static char *mac_param;
module_param_named(mac, mac_param, charp, 0444);
MODULE_PARM_DESC(mac, "station MAC handed in at boot, aa:bb:cc:dd:ee:ff");

/* Thin wrappers over the family helpers, kept at their own names so the call
 * sites and this diff stay small. The BODIES live in luna_eth_regs.h. */
static void rtl9602c_eth_get_hwaddr(struct rtl9602c_eth *ep, u8 *mac)
{
	luna_idr_get(ep->base, mac);
}

static void rtl9602c_eth_set_hwaddr(struct rtl9602c_eth *ep, const u8 *mac)
{
	luna_idr_set(ep->base, mac);
}

/*
 * The stock firmware does NOT give the WAN (nas0_0) the same MAC as the LAN: it
 * derives WAN = base + a model-specific offset. Verified on the live stock 9602C
 * reference ONU (base 98:c7:a4:32:82:a2): br0/eth0/nas0 = ..a2 but nas0_0 (the
 * IPoE/DHCP WAN service) = ..a5, i.e. base + 3. A second board confirms it
 * (LAN ..8b:d4 -> WAN ..8b:d7 = +3). The ISP/OLT identifies the ONU by this WAN
 * MAC, so gpon0 must present base+offset, NOT a copy of eth0. Offset is applied
 * as a 48-bit add (carries up) and never touches the OUI, so the result stays a
 * valid global unicast.
 */
static unsigned int wan_mac_offset = 3;
module_param(wan_mac_offset, uint, 0644);
MODULE_PARM_DESC(wan_mac_offset, "WAN (gpon0) MAC = board/LAN MAC + this offset (default 3, this model)");

/* OMCI MIB-Data-Sync (ME 2 attr 1) seed. THE PROVISIONING GATE.
 *
 * ★★ THE GATE IS AN *OR*, AND THIS COMMENT USED TO STATE IT BACKWARDS. Read out
 * of the OLT's own decompiled `gpon_ont_cfg_process` (HSGQ-G008 gpondev) on
 * 2026-08-16, the branch that reaches "start issue auth profile" -- i.e. the
 * MIB-Reset + Create burst that gives us a data GEM -- is taken when ANY of:
 *
 *      rsync == 0  ||  rsync != lsync  ||  rsync < 31  ||  <ont field> == 2
 *
 * where rsync is the ME2 MIB-Data-Sync we report at the OLT's pre-config Get and
 * lsync is the value the OLT holds for this ONT. So the OLT provisions when the
 * MIB is NOT in sync -- which is the opposite of what this comment claimed
 * ("installs the downstream gem flow ONLY when rsync==lsync && rsync>30"). The
 * `>30` was real but its SENSE was inverted: being BELOW 31 is a REASON to
 * provision, never a requirement to skip.
 *
 * ⇒ SO THE SEED IS DERIVED, NOT GUESSED. Any value in 1..30 satisfies the third
 * clause NO MATTER what the OLT has stored, so it cannot be defeated by a stale
 * lsync.
 *
 * ★★ AND WHY NOT 0, WHICH IS WHAT STOCK REPORTS. Two independent RE passes over
 * the vendor's own omci_app (2026-08-16) agree that stock does NOT persist this
 * counter -- `gMibOntDataDefRow` sits in .bss with no initialiser and is
 * explicitly zeroed in mibTable_init -- so stock answers 0 at the OLT's
 * pre-config Get on every boot, which satisfies the gate's FIRST clause. That
 * also refutes this driver's older claim that "stock persists its mds so rsync
 * matches": it does not, and nothing here should rest on that sentence again.
 *
 * By the stock-is-the-oracle rule the seed should therefore be 0. It is 7
 * instead, and the reason is a CONFLICT this project's rules resolve in favour
 * of the measurement: our own note at the mds_reset0 param records mds=0 wedging
 * THIS OLT in a Get poll loop. That observation was never explained, and it is
 * contradicted from three directions (stock reports 0; the OLT's gate takes
 * rsync==0 as its first provisioning clause; no persistence exists to make it
 * otherwise) -- but a decompilation does not overrule an observation, it only
 * makes it suspect. 7 satisfies a DIFFERENT clause of the same gate, so it is
 * correct under both readings and touches nothing in dispute.
 *
 * ⇒ OWED, AND IT IS A MEASUREMENT, NOT MORE RE: boot stock on this PON port and
 * capture what it actually answers to the pre-config ME2 Get. If it reports 0
 * and is provisioned, the wedge note is wrong and this seed becomes 0 -- exactly
 * matching stock. Until that capture exists, the adaptive walk below is what
 * covers the gap, and it costs nothing when provisioning works.
 *
 * ★ WHY THE OLD VALUE FAILED, MEASURED 2026-08-16 -- the poison poisoned itself.
 * The seed was 200, chosen to "not match any plausible stored lsync". Once this
 * ONU had reported 200, the OLT stored 200 as ITS lsync for the record; from
 * then on rsync == lsync == 200, 200 >= 31, and every clause was false, so the
 * OLT never provisioned again. Observed on PON port 2: a healthy O5 with the
 * OMCC up, 59 DS OMCI messages answered, EVERY ONE a Get, and never a MIB-Reset
 * (MT 0x4f), never an Assign_Alloc-ID, never a Create (class 268) -- Match State
 * "Initial", no data GEM, no WAN. A value chosen to be unequal became equal the
 * moment the far end remembered it.
 *
 * ⇒ AND THAT IS WHY THE ADAPTIVE WALK BELOW STAYS. This seed is right for THIS
 * OLT because we read its decision code; the walk is what makes the ONU converge
 * on an OLT whose rule we have NOT read. A derived constant answers the OLT we
 * know; the loop answers the ones we do not. */
static unsigned int omci_mds_seed = OMCI_MDS_POISON_SEED;	/* 1..30: satisfies `rsync < 31` unconditionally, so the
					 * OLT provisions whatever it has stored. NOT 0 -- see above. */
module_param(omci_mds_seed, uint, 0644);
MODULE_PARM_DESC(omci_mds_seed, "OMCI ME2 MIB-Data-Sync boot seed (1..30 forces the OLT to re-provision: its gate takes rsync<31 as not-in-sync)");
/* mds_reset0: on an on-wire MIB-Reset (MT 0x4f), zero the MIB-Data-Sync counter per G.988 (and
 * stock omci_app OMCI_ResetMib @0x41057c) instead of re-seeding 125. CAPTURE-PROVEN (2026-06-18):
 * the OLT, after IT issues MIB-Reset, recounts its lsync from 0 and re-Creates the MEs; if we keep
 * 125 our rsync = 125+N is PERMANENTLY ahead of the OLT's 0+N -> every periodic ONU-Data(ME2) audit
 * mismatches -> the OLT loops MIB-Reset/Upload/re-Create and eventually Deactivate_ONU-ID(0x05) +
 * dealloc-churn (the ~4min WAN teardown). Zeroing makes rsync==lsync after ONE resync -> stable.
 * Default on (the WAN-stability fix); rtl9602c_eth.mds_reset0=0 = legacy re-seed.
 * ★ THE PREFIX WAS WRONG HERE UNTIL 2026-08-16 ("gpon.mds_reset0"), and a wrong
 * knob name fails in the reassuring direction: the boot accepts the unknown
 * argument, the parameter keeps its default, and the operator reads the result
 * as the behaviour of the value they think they set. There is no module named
 * `gpon` on this board -- these knobs are built from rtl9602c_eth.o, so the
 * prefix is `rtl9602c_eth.` (as line 491 already spells it for hw_nat), and the
 * GPON driver's own ~60 knobs live under `gpon_rtl9602c.`. Cross-checked against
 * the live board: /sys/module/rtl9602c_eth/parameters/ holds this file's knobs
 * and /sys/module/gpon_rtl9602c/parameters/ holds the other file's (measured
 * 2026-08-15, recorded in the board's config.xml MODPARAM_DRIVER_SOURCE). */
static bool mds_reset0 = true;
module_param(mds_reset0, bool, 0644);
MODULE_PARM_DESC(mds_reset0, "on MIB-Reset zero ME2 MIB-Data-Sync (G.988/stock, default) vs re-seed");

/* ★★ THE MIB-DATA-SYNC SEED IS A GUESS, AND A GUESS CANNOT BE RIGHT ON EVERY OLT
 * RECORD. omci_mds_seed exists to FAIL the OLT's ME2 audit on purpose, so the OLT
 * issues MIB-Reset and re-provisions us (see its comment). But which values fail
 * that audit is a property of the OLT's STORED lsync for THIS ONU on THIS PON
 * port -- something we cannot read and the OLT never tells us. The seed was
 * chosen against one port's observed range (~42-126) and the note above already
 * records the two ways a wrong choice ends: a value that MATCHES makes the OLT
 * skip MIB-Reset and every Create; mds=0 "wedges this OLT in a GET poll loop".
 *
 * MEASURED 2026-08-16, after this board was moved to the OLT's PON port 2: with
 * the compiled seed 200 the OLT sent 59 DS OMCI messages, EVERY ONE a Get (ME
 * 256/257/2/65530) on a 7-13 s loop, and NEVER a MIB-Reset (MT 0x4f), never an
 * Assign_Alloc-ID, never a Create (class 268). `mds` stayed at exactly 200 --
 * only a MIB-Reset or an applied Create/Set moves it -- the OLT held Match State
 * "Initial", and the ONU had no data GEM and no WAN while sitting healthy at O5.
 * The seed had become, on that record, the wedging value its own comment warns
 * about.
 *
 * ⇒ SO WE STOP GUESSING AND CLOSE THE LOOP ON THE OUTCOME. We cannot observe
 * lsync, but we can observe the ONE thing that matters: whether the OLT is
 * PROVISIONING us. If it keeps reading and never provisions, the value we report
 * is not doing its job, and we report a different one. That is the whole idea:
 * the ONU adapts to the OLT it is actually attached to, on any port, against any
 * stored value, with nothing to re-tune by hand.
 *
 * THE WALK: +37 each step over the range 1..255. 0 is not a reportable value at
 * all here (it is the value this rig observed wedging the OLT in a Get poll
 * loop), and 37 is coprime with 255, so the walk visits EVERY reportable value
 * before repeating -- an exhaustive search, not a sample. Rate-bounded by the
 * OLT's own audit cadence and NEVER count-capped: a recovery path that gives up
 * is a device that needs a human, which is what this project's robustness bar
 * forbids.
 *
 * ★ THE ARITHMETIC IS MODULO 255, AND THAT IS LOAD-BEARING -- MEASURED, NOT
 * ASSUMED. This walk was first written as `mds += 37; if (!mds) mds = 1;`, whose
 * comment claimed the same exhaustive search on the strength of 37 being coprime
 * with 256. It is not: folding 0 onto 1 splices the +37 orbit into a CYCLE, and
 * from mds == 1 that cycle is only 83 long (37 * 83 == 255 mod 256), so from the
 * seed the search covered 83 of 255 values -- 32%, silently. Caught by
 * dev/rtl9607c-test Step 4f case [d] on x86 with no board in the loop.
 *
 * IT COSTS NOTHING WHEN THINGS WORK. Every provisioning event -- a MIB-Reset, or
 * any applied Create/Set/Delete -- resets the counter, so on a healthy admission
 * the walk never takes a single step and the compiled seed is used exactly as
 * before. Set omci_mds_adapt=0 for the old fixed-seed behaviour. */
static bool omci_mds_adapt = true;
module_param(omci_mds_adapt, bool, 0644);
MODULE_PARM_DESC(omci_mds_adapt, "walk the reported ME2 MIB-Data-Sync when the OLT reads but never provisions (default on)");
static unsigned int omci_mds_adapt_reads = 12;
module_param(omci_mds_adapt_reads, uint, 0644);
MODULE_PARM_DESC(omci_mds_adapt_reads, "DS OMCI reads with no provisioning before the MIB-Data-Sync is advanced");

/* The walk's step. Coprime with 255, so stepping modulo 255 over the reportable
 * range 1..255 enumerates all of it before repeating. */
#define OMCI_MDS_WALK_STEP	37

/* Body hoisted to flowcore (rtl9602c_wan_mac_add): the 48-bit ripple-carry
 * add is pure byte arithmetic.  The wrapper keeps the name and signature so
 * every call site is untouched, and supplies the module param (the hoisted
 * logic reads no shell file-scope state). */
static void rtl9602c_wan_mac(u8 *out, const u8 *base)
{
	rtl9602c_wan_mac_add(out, base, wan_mac_offset);
}

/*
 * Program the station address into the hardware (IDR) on a MAC change, not just
 * ndev->dev_addr: the MyPhys RX filter matches IDR, so without this, unicast to
 * a freshly-set per-board MAC is dropped by hardware. gpon_provision applies the
 * per-board MAC this way at boot.
 */
static int rtl9602c_eth_set_mac_address(struct net_device *ndev, void *p)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);
	int ret = eth_mac_addr(ndev, p);

	if (ret)
		return ret;
	rtl9602c_eth_set_hwaddr(ep, ndev->dev_addr);
	/* Keep the WAN (gpon0) identity tracking the board MAC provisioned onto eth0
	 * (rtk_factory), applying the stock model offset: WAN = LAN + wan_mac_offset
	 * (see rtl9602c_wan_mac). This is the MAC the ISP/OLT identifies the ONU by. */
	if (ep->wan_ndev) {
		u8 wmac[ETH_ALEN];

		rtl9602c_wan_mac(wmac, ndev->dev_addr);
		eth_hw_addr_set(ep->wan_ndev, wmac);
	}
	return 0;
}

/* Give RX descriptor @idx a fresh buffer and hand it to HW (own=1). */
/* The body is the FAMILY's (luna_eth_regs.h): both drivers had it character
 * for character apart from the struct that reached `->rx_ring`.  The wrapper
 * keeps the old name and signature so every call site is untouched. */
static int rtl9602c_eth_refill(struct rtl9602c_eth *ep, unsigned int idx)
{
	return luna_rx_refill(ep->ndev, ep->dev, ep->rx_ring, ep->rx_skb,
				 ep->rx_buf_dma, idx, RX_RING_SIZE, RX_BUF_SIZE);
}

/* ===== M2: G.988 OMCI responder + upstream OMCC TX ======================= */

/*
 * US-OMCI "txInfo" descriptor encoding — the AUTHORITATIVE stock 9602C mechanism.
 *
 * Derived from observing the stock device's OMCI-TX behavior (clean-room: the
 * values below are FACTS; the code is our own). The stock OMCI-TX path
 * builds a zeroed 5-word txInfo on the stack and OR-patches three
 * words, then the NIC-TX layer copies the OMCI PDU into an skb and submits the
 * txInfo, where the ring engine OR-s the descriptor control flags
 * into word0, sets OWN in word2, publishes the ring slot and kicks the per-ring
 * poll doorbell. The descriptor is 5 u32 words at offsets 0/4/8/12/16 = opts1 /
 * addr / opts2 / opts3 / opts4. CONFIRMED from the stock submit path: OWN = bit31 of
 * the word at OFFSET 8 = opts2/word2 (read word at 8, OR in 0x80000000, store back);
 * word0/opts1 (offset 0) carries the org/desc-flag control and is the LAST
 * word copied to the live ring after a sync barrier (a sync, then the store of
 * word0). So OWN lives in opts2 and is set LAST in our build, after the body.
 * Word layout (32-bit words; word0 published last):
 *
 *   word0 (opts1): segment/org control. Stock OR-s 0x02240000 (bits 25/21/18) on
 *                  top of the FS/LS/len that the submit path fills; the submit
 *                  path additionally OR-s the descriptor flags 0xb8800000 |
 *                  0x40000000 and re-OR-s (word0 & 0x077e0000).
 *   word2/word3 = the stock GMAC tx_info (opts2->word2, opts3->word3, copied
 *   VERBATIM onto the ring; the GMAC tx_info bitfields). Stock
 *   OMCI-TX directs the OMCI PDU CPU->PON like this:
 *     word2 (opts2): cputag(bit31) | tx_portmask[26:16] = (1 << ponPort).
 *     word3 (opts3): keep(23) | dislrn(21) | cputag_psel(20) | l34_keep(17) |
 *                    tx_dst_stream_id[6:0] = OMCC SID (=64, a PLAIN 7-bit value,
 *                    NOT one-hot). cputag_psel = PORT-SELECT egress: the switch
 *                    egresses DIRECTLY to the masked port with NO L2 DA lookup,
 *                    bypassing VLAN/PISO/flooding. keep/l34_keep/dislrn stop the
 *                    switch modifying / L3-L4-filtering / SA-learning the frame.
 *                    extspa is NOT set for OMCI.
 *   ⛔ The OLD encoding was FABRICATED and never reached the US-NIC: word2
 *   0x80080000 set bit19, which lands in tx_portmask[26:16] as value 8 = the CPU
 *   port (3) -> the switch bounced the OMCI back to the CPU and flooded it to LAN
 *   (the LAN wedge); word3 used a PON_SID one-hot + ExtSpa with cputag_psel=0 and
 *   stream_id=0, so no port-select and wrong SID. rxsid=0, ustx=0, no egress.
 *   word1, word4: left zero (stock leaves them 0 from the initial memset).
 */
#define TXD0_OMCI_ORG		0x02240000u	/* word0 segment/org control bits */
/* TXD2_OMCI_CPUTAG / TXD2_OMCI_EFID: moved to rtl9602c_l34_logic.h with
 * the hoisted steering encode (rtl9602c_omci_txd_word2). */
#define TXD2_TX_PMASK(p)	(((p) & 0x7FFu) << 16)	/* opts2[26:16] tx_portmask */
/* word0 descriptor-flag ORs applied by the ring-submit path. */
#define TXD0_DESC_FLAGS		(0xb8800000u | 0x40000000u)	/* = 0xf8800000 */
#define TXD0_DESC_KEEP_MASK	0x077e0000u	/* word0 bits re-OR'd by submit */
/* word3 (opts3) stock OMCI directed-egress fields. */
#define TXD3_OMCI_KEEP		0x00800000u	/* opts3 bit23 keep (don't modify frame) */
#define TXD3_OMCI_DISLRN	0x00200000u	/* opts3 bit21 dislrn (don't learn CPU SA) */
#define TXD3_OMCI_PSEL		0x00100000u	/* opts3 bit20 cputag_psel (port-select egress) */
#define TXD3_OMCI_L34KEEP	0x00020000u	/* opts3 bit17 l34_keep (no L3/L4 filter) */
#define TXD3_DST_SID(s)		((s) & 0x7Fu)	/* opts3[6:0] tx_dst_stream_id (plain value) */
/* The ★ 9602C opts3/word3 layout (the 9602C-vs-9607C SID-placement trap),
 * its live-stock GROUND TRUTH (word2=0x80080000 word3=0x02400000, devmem
 * 2026-06-11), and GMAC_PON_PORT / TXD3_9602C_PMASK / TXD3_9602C_DST_SID /
 * TXD3_OMCI_9602C all moved to rtl9602c_l34_logic.h, beside the hoisted
 * rtl9602c_omci_txd_word2/word3() steering encode (flowcore). */
#define RTL9602C_OMCC_SID	64			/* OMCC US SID (== GPON flow 64) */
#define RTL8_4_TAG_LEN		8			/* software rtl8_4 0x8899 cpu-tag (also used by the LAN xmit below) */

/*
 * LAYER BOUNDARY (2026-08-05, RESOLVED 2026-09-01): the MIC and the baseline
 * trailer are G.988 message facts, so their common home is omci_set_mic() /
 * omci_finalize() in gpon-common .../drivers/net/gpon/gpon_omci_core.c.
 *
 * ★ THE MIC HAS NOW MOVED.  This block used to say "they did NOT move",
 * because this file carried its own crc32_le stamper against the core's
 * crc32_be.  It no longer does: rtl9602c_omci_finalize() calls the core's
 * omci_set_mic().  Only the baseline TRAILER (40..43) is still stamped here.
 * Divergence 1 of the nine in the responder boundary block further down —
 * search "IT DID NOT MOVE" — and that entry now reads RESOLVED.
 */
/*
 * GUARD: OMCI MIC (bytes 44..47) — MUST be correct or the OLT silently drops
 * every response. The OLT validates the CRC-32 MIC on every OMCI baseline
 * message; a zero or wrong MIC = silent drop -> the OLT never sees our
 * responses -> stuck in GET audit loop -> churn-lock.
 *
 * The HW MIC path (RTL9602C_OMCI_HW_MIC=1) leaves bytes 44..47 = 0, expecting
 * the GTC/GEM HW to append the CRC-32. But with DBG_IGNORE_TAG=1 (which strips
 * the CPU tag before GEM encapsulation), the HW MIC engine may not fire on the
 * stripped frame. If the HW doesn't append the MIC, the OLT receives zeros and
 * drops the frame.
 *
 * The SW MIC path stamps bytes 44..47 via the core's omci_set_mic(), i.e.
 * crc32_be (non-reflected AAL5, G.984.4) over bytes 0..43 -- NOT the crc32_le
 * this file used to compute for itself.  It is correct regardless of HW
 * behavior. If the HW
 * ALSO appends a MIC, the OLT sees a doubled frame — but the first MIC is at
 * 44..47 (where the OLT expects it), so the OLT validates the SW MIC and
 * ignores any trailing bytes.
 *
 * Set to 0 to use SW MIC (safer — works regardless of HW MIC behavior). */
#define RTL9602C_OMCI_HW_MIC	0


/* Stamp the baseline trailer (40..43 = 00 00 00 28) + MIC. Call LAST. */
static void rtl9602c_omci_finalize(u8 *msg)
{
	msg[40] = 0x00;
	msg[41] = 0x00;
	msg[42] = 0x00;
	msg[43] = 0x28;		/* baseline trailer constant 0x0028 */
#if RTL9602C_OMCI_HW_MIC
	msg[44] = msg[45] = msg[46] = msg[47] = 0x00;	/* GTC/GEM HW appends MIC */
#else
	/* ★★ THE CORE'S STAMPER, NOT A SECOND ONE.  This file used to carry
	 * rtl9602c_omci_set_mic(), which computed crc32_le (reflected, zlib)
	 * while gpon_omci_core.c computes crc32_be (AAL5, G.984.4) -- so this
	 * ONE BOARD emitted OMCI under TWO MIC polynomials, and at most one can
	 * be right.  omci_mic_parity.py reported it as DISAGREE and said exactly
	 * that; a wrong MIC is a SILENT DROP at the OLT.
	 *
	 * ⚠ AND THE PATH THAT CARRIED THE ODD ONE WAS THE AVC.  When this was
	 * written, rtl9602c_omci_finalize()'s other caller was the local AVC
	 * builder, and the VEIP oper-up AVC is what this port's WAN provisioning
	 * waits on -- so the one message the OLT most needs was the one stamped
	 * differently from every other message this board sends.  (The AVC has
	 * since been rebased onto the core, 2026-09-02; the remaining caller of
	 * this function is the /proc self-test injector below.)
	 *
	 * crc32_be is the right side of that pair and not merely the majority:
	 * G.984.4 asks for the non-reflected AAL5 CRC-32, gpon_omci_core.c
	 * records it as LIVE-PROVEN on the DS frames from this OLT, and the core
	 * already stamps every other OMCI message this board sends -- which this
	 * OLT demonstrably accepts, because the board provisions. */
	omci_set_mic(msg);
#endif
}

/* Reclaim completed descriptors on the dedicated US-OMCI ring. Called under
 * tx_lock. The ring engine clears OWN (word2 bit31) once it has consumed the
 * descriptor; until then the slot is HW-owned and must not be reused. */
static void rtl9602c_eth_omci_reclaim(struct rtl9602c_eth *ep)
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
static void rtl9602c_eth_tx_reclaim(struct rtl9602c_eth *ep);

/* Stock per-packet TX-fetch GO (observed in the stock device's submit path): set bit31
 * of the network-engine reg at txgo+0x38, then poll it clear (HW acks the fetch) up
 * to ~100 spins. This is what commands the self-polling GMAC TX DMA to FETCH the
 * freshly-published OWN descriptor; the per-ring R_IO_CMD doorbell alone only marks
 * the ring eligible. Required for the SPARSE OMCI inject (the engine parks otherwise
 * — txok frozen, dirty stuck ~3); continuous LAN traffic masks it via auto-fetch.
 * Timeout is non-fatal (stock just logs). Caller already holds tx_lock. */
static void rtl9602c_eth_tx_fetch(struct rtl9602c_eth *ep)
{
	int n;

	if (!ep->txgo)
		return;
	iowrite32(ioread32(ep->txgo + 0x38) | BIT(31), ep->txgo + 0x38);
	for (n = 0; n < 100; n++) {
		if (!(ioread32(ep->txgo + 0x38) & BIT(31)))
			break;
		cpu_relax();
	}
}

/*
 * Shared-ring-0 US-OMCI transmit (omci_tx_ring==0 test path).
 *
 * Enqueue the OMCI frame onto the SAME LAN ring 0 (ep->tx_ring) the normal TX
 * path uses, but with the OMCI descriptor STEERING (word0 org bits + word3
 * one-hot PON_SID / ExtSpa) instead of a plain LAN descriptor. HW ring 0 is the
 * proven-fetching ring (the host's SSH/LAN traffic flows through it), so this
 * isolates whether the corrected OMCI steering alone routes the frame to the PON
 * US-NIC — independent of the unresolved "HW ring 4 won't fetch" problem.
 *
 * Shares the LAN ring + ep->tx_lock + the bit0 doorbell + the LAN tx_reclaim, so
 * normal LAN traffic is undisturbed. The descriptor matches the LAN-ring layout
 * (OWN lives in opts1/word0 on this ring and is published LAST, exactly like
 * rtl9602c_eth_xmit) — NOT the dedicated-ring layout (OWN in opts2). If the ring
 * is full the OMCI frame is DROPPED (never blocks LAN traffic). Returns 0 on
 * success. Runs in poll-timer softirq context -> GFP_ATOMIC.
 */
static int rtl9602c_eth_omci_xmit_ring0(struct rtl9602c_eth *ep, const u8 *omci,
					unsigned int len)
{
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i;
	dma_addr_t da;
	u32 word0;

	/* LAN-ring HW-descriptor TEST: inject the bare OMCI PDU onto the PROVEN-to-fetch
	 * LAN ring 0 (ep->tx_ring, kick R_IO_CMD bit0) with the CORRECTED HW cpu-tag
	 * descriptor (word2 cputag+portmask, word3 one-hot PON_SID + ExtSpa) — NOT the SW
	 * 0x8899 tag. Isolates RING (HW ring 4 unwired) vs descriptor: HW ring 4 fetches
	 * but RX_SID_GOOD stays 0, so retry the SAME corrected descriptor on the ring that
	 * is known to reach the GMAC/switch. The old "HW desc = 0 egress portmask" note was
	 * recorded with the WRONG word3 (slot-0 one-hot + garbage ExtSpa). */
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
	/* The GPON driver calls in from ITS OWN timer/softirq, so the interface
	 * may be mid ndo_stop (netifd cycles eth0 during boot) or not yet open:
	 * tx_ring is then freed/NULL and a descriptor write is a NULL deref in
	 * interrupt context (observed panic: BadVA 0xe0 = &tx_ring[11].addr at
	 * the O5 selftest racing the boot-time network restart). Bail under the
	 * lock; ndo_stop takes the same lock as a barrier before freeing. */
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
	rtl9602c_eth_tx_reclaim(ep);
	/* If the shared LAN ring is full, the GMAC simply hasn't RETIRED the in-flight
	 * (HW-owned) LAN frames yet — a bare reclaim can't free a slot. Stock retires within
	 * microseconds via the TX-completion IRQ; we are polled (2ms), so the sparse OMCI
	 * inject kept hitting a full ring and getting DROPPED (dbg_omci_tx_drop climbed, US
	 * OMCI never egressed, rxsid=0). Instead of dropping, KICK the TX engine + re-reclaim
	 * in a BOUNDED loop to free a slot for the (sparse, important) OMCI frame. */
	{
		int spin = 64;

		while ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1 && spin-- > 0) {
			ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* retire ring0 */
			cpu_relax();
			rtl9602c_eth_tx_reclaim(ep);
		}
	}
	if ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -EBUSY;
	}
	i = tx_slot(ep, ep->tx_head);
	ep->tx_skb[i] = skb;		/* LAN reclaim frees it */
	ep->tx_buf_dma[i] = da;
	ep->tx_buf_len[i] = len;
	ep->tx_ring[i].addr = da | DMA_BUS_WINDOW;

	/*
	 * word0 (opts1) = FS|LS|len|0x02240000 (keep|dislrn|cputag_psel), OWN
	 * added at publish. The 0x02240000 org-control bits — especially
	 * cputag_psel (port-SELECT egress / direct-TX) — are what make the GMAC
	 * direct-transmit the cpu-tagged frame to the US-NIC, BYPASSING the L2
	 * switch fabric (verified: stock's switch port-2 TX MIB stays 0 while
	 * the US-NIC RX_OK climbs). The GMAC CONSUMES these bits during cpu-tag
	 * insertion, so a post-TX descriptor read shows them cleared (0x30000030)
	 * — they were set at submit. Without them the frame L2-floods and never
	 * reaches the US-NIC. D_EOR only on wrap slot.
	 *
	 * TxCRC (bit23) + IPCS (bit27): the stock submit path
	 * UNCONDITIONALLY OR's 0xb8800000 into opts1 for EVERY frame, OMCI included, so
	 * the stock OMCI submit word0 = 0xbaa40030 — the GMAC GENERATES + APPENDS the
	 * Ethernet FCS. The old "stock omits TxCRC" belief was a MISREAD of the
	 * post-consume readback (0x30000030 has crc/ipcs/own/org already cleared by the
	 * GMAC), mistaken for the submit value. Without D_TXCRC the 48B OMCI PDU goes out
	 * with NO valid FCS and the US-NIC/fabric MAC silently drops it pre-MAC
	 * (RX_OK=ERR=MISS=0, rxsid[4]=0 — the exact US-OMCI symptom). The LAN TX path
	 * sets D_TXCRC and works; only this OMCI path omitted it.
	 */
	word0 = D_FS | D_LS | D_TXCRC | D_IPCS | (len & TXD_LEN_MASK) | TXD0_OMCI_ORG;
	if (i == tx_eor_slot(ep))
		word0 |= D_EOR;

	/* HW cpu-tag steering words -- EXACT stock values (live ref-ONU TX
	 * ring): word2=0x80080000 (cputag|efid), word3=0x02400000 (pmask 1<<2 |
	 * SID 64).  Encode + override plumbing hoisted to
	 * rtl9602c_omci_txd_word2/word3() (flowcore), where the 9602C-vs-9607C
	 * SID-placement layout lives once. */
	ep->tx_ring[i].opts2 = rtl9602c_omci_txd_word2(omci_word2_ovr);
	ep->tx_ring[i].opts3 = rtl9602c_omci_txd_word3(omci_word3_ovr,
						       RTL9602C_OMCC_SID);
	ep->tx_ring[i].opts4 = 0;
	if (omci_minimal) {	/* TEST: descriptor IDENTICAL to the draining LAN path (no ORG, no cpu-tag) */
		word0 = D_FS | D_LS | D_TXCRC | (len & TXD_LEN_MASK);
		if (i == tx_eor_slot(ep))
			word0 |= D_EOR;
		ep->tx_ring[i].opts2 = 0;
		ep->tx_ring[i].opts3 = 0;
	}
	wmb();				/* descriptor body before ownership */
	ep->tx_ring[i].opts1 = word0 | D_OWN;	/* OWN in opts1: publish to HW */
	wmb();

	ep->omci_r0_last_slot = (int)i;
	ep->tx_head++;
	rtl9602c_eth_tx_fetch(ep);	/* stock per-packet TX-fetch GO (0x18001038[31]) — fetch the just-published OMCI descriptor */
	ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* kick LAN ring 0 */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (ep->dbg_omci_tx < 30) {	/* first few only, so serial isn't flooded */
		/* PRIME PROBE: did the GMAC INSERT the cpu-tag on the GMAC->switch wire?
		 * The GMAC CONSUMES the word0 org bits (0x02240000 keep/dislrn/psel) while
		 * inserting the cpu-tag, so the post-TX descriptor reads back with them
		 * CLEARED: submit 0xb2240030 -> post-TX 0x30000030 (OWN+org cleared) = cpu-tag
		 * INSERTED. If it reads 0x32240030 (OWN cleared but org 0x02240000 REMAINS),
		 * the GMAC did NOT process the cpu-tag -> the switch sees a plain frame ->
		 * L2 lookup -> TRAP2CPU (our measured p2-tx=0 / frame-trapped symptom). */
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

/*
 * US-OMCI upstream transmit — the AUTHORITATIVE stock 9602C mechanism.
 *
 * Clean-room re-implementation of the stock OMCI-TX path (build txInfo ->
 * NIC-TX layer -> submit). The frame body is the BARE OMCI PDU only
 * (no [DA][SA] L2 header, no in-band 0x8899 tag); steering rides entirely in the
 * 5-word txInfo descriptor, and the descriptor is submitted on the dedicated
 * OMCC TX ring with the per-ring poll doorbell. The encoding (word0/word2/word3
 * masks, ring, doorbell) was re-derived from the stock device's behavior and is described at
 * the TXD*_OMCI_* defines above. The descriptor ring, the TxFDP arm (in open())
 * and the doorbell ALL reference the same HW ring h, so the engine polls the ring
 * we actually filled. Differs from the prior driver: a dedicated HW ring (not the
 * LAN ring 0), word3 one-hot PON_SID + ExtSpa steering (not a word2 portmask),
 * word2 = 0x80080000 (bits 16..18 CLEARED).
 *
 * No ETH_ZLEN padding (GEM payload, no 60-byte minimum). Runs in poll-timer
 * softirq context -> GFP_ATOMIC. Returns 0 on success.
 */
/* ===== WAN data-GEM netdev (gpon0) — clean-room nas0-equivalent =====
 * Carries GPON WAN user data on the data GEM (the OLT's gem-port-id, gpon_data_gem_port,
 * on internal flow GPON_DATA_FLOW).
 * DS frames de-encapsulated from the data GEM arrive from switch PON port 2 and are demux'd
 * to this netdev in rtl9602c_eth_rx (src_port == ep->swm->pon_port). US frames use the SAME
 * HW cpu-tag direct-TX descriptor the OMCI path uses, but tx_dst_stream_id = GPON_DATA_FLOW
 * (the US-NIC then stamps the OLT's gem-id via GEM_US_PORT_MAP and routes to the OMCC's
 * T-CONT 16/qid 64 grants). Carrier is held up so netifd runs DHCP; pre-install US frames are
 * dropped by the US-NIC and DHCP retries until the OLT's OMCI config installs the data GEM. */
static netdev_tx_t rtl9602c_eth_wan_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rtl9602c_eth *ep = *(struct rtl9602c_eth **)netdev_priv(ndev);
	unsigned long flags;
	unsigned int i, len;
	dma_addr_t da;
	u32 word0;

	if (skb_put_padto(skb, ETH_ZLEN))	/* pad runts to the min Ethernet frame; frees skb on error */
		return NETDEV_TX_OK;
	len = skb->len;

	da = dma_map_single(ep->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	if (ep->closing || !ep->tx_ring) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	rtl9602c_eth_tx_reclaim(ep);	/* shared LAN ring 0; drop on full (DHCP retransmits) */
	if ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	i = tx_slot(ep, ep->tx_head);
	ep->tx_skb[i] = skb;		/* freed by the LAN ring reclaim */
	ep->tx_buf_dma[i] = da;
	ep->tx_buf_len[i] = len;
	ep->tx_ring[i].addr = da | DMA_BUS_WINDOW;
	word0 = D_FS | D_LS | D_TXCRC | D_IPCS | (len & TXD_LEN_MASK) | TXD0_OMCI_ORG;
	if (i == tx_eor_slot(ep))
		word0 |= D_EOR;
	ep->tx_ring[i].opts2 = rtl9602c_omci_txd_word2(0);	/* stock cputag|efid */
	ep->tx_ring[i].opts3 = rtl9602c_omci_txd_word3(0, GPON_DATA_FLOW);	/* steer to the data SID */
	ep->tx_ring[i].opts4 = 0;
	wmb();				/* descriptor body before ownership */
	ep->tx_ring[i].opts1 = word0 | D_OWN;
	wmb();
	ep->tx_head++;
	rtl9602c_eth_tx_fetch(ep);
	ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* kick LAN ring 0 */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	return NETDEV_TX_OK;
}

static int rtl9602c_eth_wan_open(struct net_device *ndev)
{
	struct rtl9602c_eth *ep = *(struct rtl9602c_eth **)netdev_priv(ndev);

	/* Set the WAN identity the OLT/ISP uses to recognise this ONU: the device's board
	 * MAC plus the stock model offset (WAN = LAN + wan_mac_offset; nas0_0 = base+3 on
	 * stock). The board MAC is provisioned onto eth0 (rtk_factory) at boot, AFTER this
	 * driver's probe, so gpon0 (created at probe) derives it here at open (by which point
	 * eth0 carries the real board MAC). */
	if (ep && ep->ndev && is_valid_ether_addr(ep->ndev->dev_addr)) {
		u8 wmac[ETH_ALEN];

		rtl9602c_wan_mac(wmac, ep->ndev->dev_addr);
		eth_hw_addr_set(ndev, wmac);
	}
	/* RX/TX rings + NAPI are owned by eth0 (shared HW); open just enables the queue and
	 * holds carrier up so netifd runs the DHCP client. */
	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int rtl9602c_eth_wan_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	return 0;
}

static const struct net_device_ops rtl9602c_eth_wan_ops = {
	.ndo_open		= rtl9602c_eth_wan_open,
	.ndo_stop		= rtl9602c_eth_wan_stop,
	.ndo_start_xmit		= rtl9602c_eth_wan_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

static int rtl9602c_eth_omci_xmit(struct rtl9602c_eth *ep, const u8 *omci,
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
		return rtl9602c_eth_omci_xmit_ring0(ep, omci, len);
	/* omci_tx_ring is the HW ring h directly — the SAME h used to arm R_TxFDP(h)
	 * in open(). Derive the doorbell from h so they can never disagree (the prior
	 * bug: ring armed at h=4/0x1340 but kicked R_IO_CMD bit0 = h=0). Stock kick
	 * behavior: h<4 -> R_IO_CMD |= 1<<h; h==4 -> R_IO_CMD1 |= 0x100. */
	kick_iocmd1 = rtl9602c_omci_doorbell(omci_tx_ring, omci_doorbell_bit,
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
	rtl9602c_eth_omci_reclaim(ep);
	if ((ep->otx_head - ep->otx_dirty) >= OTX_RING_SIZE - 1) {
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

	/*
	 * word0 (opts1) = FS|LS|len|0x02240000 (keep|dislrn|cputag_psel). The
	 * cputag_psel bit triggers GMAC direct-TX to the US-NIC (bypassing L2);
	 * the GMAC consumes these bits during cpu-tag insertion. D_TXCRC|D_IPCS: the
	 * stock submit path OR's 0xb8800000 into opts1 for every frame -> GMAC appends
	 * the FCS; omitting it dropped the OMCI pre-MAC (rxsid=0). See the ring-0 path.
	 */
	word0 = D_FS | D_LS | D_TXCRC | D_IPCS | (len & TXD_LEN_MASK);
	word0 |= TXD0_OMCI_ORG;
	/* Same fix as the ring-0 path: DO NOT OR TXD0_DESC_FLAGS (0xf8800000) — it sets
	 * D_EOR (bit30) on EVERY descriptor, so the GMAC treats each as end-of-ring and
	 * never drains the ring (OWN stuck, dirty stalls). D_EOR belongs ONLY on the wrap
	 * slot (set just below), exactly like the working LAN TX path. */
	if (i == otx_eor_slot(ep))
		word0 |= D_EOR;

	/* word2 (opts2): CpuTag(31)+bit19 set, bits 16..18 cleared. OWN does NOT
	 * live here: the GMAC fetch engine reads ownership from opts1 bit31 (word0)
	 * on EVERY TX ring, including HW ring 4. Confirmed from the stock device's
	 * descriptor handling (the TX descriptor's opts1 carries own/eor/fs/ls/crc; its
	 * reclaim/poll tests the ring tail's opts1 against DescOwn / 0x80000000).
	 * The previous OWN-in-opts2 left opts1 bit31
	 * clear, so the HW saw the descriptor as CPU-owned and never fetched ring 4
	 * (own[]=1, dirty=0). Mirror the proven LAN ring exactly: OWN in opts1,
	 * published LAST. */
	/* word2 (opts2): stock = cputag(31) | bit19 (directed-egress; steered by word3's
	 * one-hot PON_SID + ExtSpa, NOT a switch portmask), tx_portmask [18:16] CLEARED.
	 * From the stock OMCI-TX core: opts2 = (prev | 0x80080000) & 0xfff8ffff.
	 * The old TX_PMASK(port) bit18 = switch cpu-tag forwarding => 0 egress on this
	 * silicon (frame dropped before the US-NIC) — the real reason RX_SID_GOOD stayed 0. */
	word2 = rtl9602c_omci_txd_word2(omci_word2_ovr);	/* stock cputag|efid; hoisted encode */

	/* word3 (opts3): stock US-OMCI steering, derived from the stock device's OMCI-TX
	 * core (the OMCI-TX entry builds the txInfo, then the NIC-TX layer writes opts3):
	 * the steering is the one-hot PON_SID
	 * [28:23] = ((1<<omci_sid_idx)&0x3F)<<23 (idx 4 = the SID-64 classify slot ->
	 * RX_SID_GOOD group[4]) | ExtSpa/PON-port [22:16] = (omci_pon_port&0x7F)<<16 |
	 * tx_dst_stream_id [6:0] = OMCC SID 64. The earlier KEEP|DISLRN|PSEL|L34KEEP
	 * bits all fell INSIDE [28:16] and overwrote these fields — emitting one-hot
	 * bit0 (classify slot 0 / group[0]) + a garbage ExtSpa — so the US-NIC never
	 * stamped SID-64 and RX_SID_GOOD[4] stayed 0. omci_sid_idx/omci_pon_port were
	 * defined+documented but never wired into the descriptor until now. */
	word3 = rtl9602c_omci_txd_word3(omci_word3_ovr,
					RTL9602C_OMCC_SID);	/* 9602C: pmask[28:23]|SID[22:16] */

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
	 * mask both come from rtl9602c_omci_doorbell(), the SAME derivation as the
	 * R_TxFDP(h) arm in open(), so the engine fetches the ring we filled. */
	if (kick_iocmd1)
		/* HW ring 4 (TxFDP5) poll "go" = IO_CMD1 |= TX_POLL5 (bit8 = 0x100),
		 * EXACTLY as the stock per-ring TX kick. IO_CMD1[21:16] is
		 * the RX multiring bitmap, NOT a TX-fetch enable, so it is not touched
		 * here — the TX ring fetches once its TxFDP is armed and the descriptor
		 * publishes OWN in opts1. */
		ep_wr(ep, R_IO_CMD1, ep_rd(ep, R_IO_CMD1) | dmask);
	else
		ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | dmask);
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (ep->dbg_omci_tx < 8)	/* first few only, so serial isn't flooded */
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

/* OLT-INDEPENDENT US-OMCI datapath self-test: inject a synthetic 48-byte OMCI
 * frame through the normal US-OMCI TX path (same ring + descriptor steering as a
 * real Get response), so the US datapath can be validated at O5 WITHOUT the OLT
 * sending DS OMCI (the degraded OLT withholds it). After calls, watch
 * RX_SID_GOOD_CNT_US[4] (SID 64, pi 0x204c): climbing => the frame reaches the
 * US-NIC classifier (steering OK, stall is downstream); still 0 => the frame
 * never egresses (ring-fetch / CPU->PON switch-egress gap). The payload is
 * irrelevant to the steering test (the US-NIC classifies by SID, not content). */
void rtl9602c_eth_omci_selftest(void)
{
	struct rtl9602c_eth *ep = g_ep;
	u8 frame[48];

	if (!ep)
		return;
	/* Arm the GMAC OMCI cpu-tag routing (CPUTAGCR=0x901eff04) — the OLT path does
	 * this via gpon_install_omcc->set_omci_sid; without it the GMAC sits at the
	 * non-OMCI 0x981aff04 and the descriptor's cpu-tag is NOT processed, so a
	 * self-inject would never reach the US-NIC even with a correct descriptor.
	 * Idempotent. (The earlier ring0 selftest omitted this -> invalid rxsid=0.) */
	rtl9602c_eth_set_omci_sid(RTL9602C_OMCC_SID);
	memset(frame, 0, sizeof(frame));
	frame[0] = 0x00; frame[1] = 0x01;	/* TID */
	frame[2] = 0x29;			/* MT = Get-response (0x09 | AK 0x20) */
	frame[3] = 0x0a;			/* DevId (baseline) */
	frame[4] = 0x01; frame[5] = 0x00;	/* ME class 256 (ONT-G) */
	rtl9602c_eth_omci_xmit(ep, frame, sizeof(frame));
}
EXPORT_SYMBOL(rtl9602c_eth_omci_selftest);

/*
 * ============================================================================
 * OMCI (ITU-T G.988) BASELINE RESPONDER — LUNA'S OWN COPY.  IT DID NOT MOVE.
 * ============================================================================
 *
 * WHAT THIS IS, AND WHY IT IS STILL SITTING IN AN ETHERNET DRIVER
 *   Everything from here down to rtl9602c_eth_omci_report_oper_up() is
 *   PROTOCOL, not hardware: the G.988 message types, the managed-entity model,
 *   the dynamic OLT-provisioned ME store, the MIB-Upload row table and the
 *   downstream-request to upstream-response dispatcher.  None of it touches a
 *   register.  It lives in rtl9602c_eth.c only because the US-OMCI transmit
 *   path (the OMCC SID, the TX descriptor steering, the ring) is here and the
 *   responder was written next to its transmitter.  That is a layering
 *   violation, and it is KNOWN — this block is the record of it.
 *
 * WHERE THE COMMON LAYER IS
 *   target/linux/gpon-common/files-6.18/drivers/net/gpon/
 *     gpon_omci_core.{c,h}  the G.988 MESSAGE layer (parse, dispatch, response
 *                           build, trailer + MIC)
 *     gpon_omci_me.{c,h}    the ME MODEL layer (attribute descriptor table,
 *                           board identity, MIB-Upload rows, instance store)
 *   That layer is common because G.988 is a specification, not a chip fact:
 *   the same message rules hold on Luna (MIPS32, big-endian) and on Cortina
 *   (ARM64, little-endian), so exactly ONE copy of them belongs in the tree.
 *   realtek-elnath compiles it today.  realtek-luna does NOT, yet.
 *
 * WHY THIS CODE WAS NOT MOVED — MEASURED, NOT ASSUMED (2026-08-05)
 *   Re-pointing this responder at the common core is NOT code motion: it
 *   changes the octets this ONU puts on the fibre.  Both responders were
 *   compiled on x86 and driven with the SAME 82 downstream PDUs (MIB-Reset,
 *   MIB-Upload, a GET sweep over 18 classes x 3 masks, MIB-Upload-Next across
 *   the row space, Create/Set/Delete including their error paths, the odd
 *   message types, AR-clear, a non-baseline DevID, a runt, a retransmit, and
 *   the autonomous VEIP AVC).  Result: 80 of the 82 responses DIFFER.  The
 *   two that match are the two where both sides correctly emit nothing.
 *
 *   Nine independent causes; ONE IS NOW RESOLVED, eight remain, and each of
 *   the eight ALONE breaks bit-identity:
 *    1. MIC ALGORITHM — ✔ RESOLVED 2026-09-01, and it is no longer a
 *       DIFFERENCE because the second implementation is GONE.  This file used
 *       to compute crc32_le (reflected, zlib) in its own
 *       rtl9602c_omci_set_mic(), while the common core computes crc32_be
 *       (AAL5, G.984.4) -- so ONE BOARD emitted OMCI under TWO polynomials,
 *       which omci_mic_parity.py reports as DISAGREE and which no capture is
 *       needed to call wrong: at most one can be right.
 *
 *       ⚠ AND THE ODD ONE WAS ON THE AVC PATH.  rtl9602c_omci_finalize()'s
 *       other caller was the local AVC builder (itself rebased onto the
 *       core's omci_emit_avc on 2026-09-02), so the VEIP oper-up AVC that
 *       this port's WAN provisioning waits on was the one message stamped
 *       differently from everything else the board sends.
 *
 *       The local stamper is deleted and the call site uses the CORE's
 *       omci_set_mic().  crc32_be is the right side and not merely the
 *       majority: G.984.4 asks for the non-reflected AAL5 CRC-32, the core
 *       records it LIVE-PROVEN on this OLT's DS frames, and the core already
 *       stamps every other OMCI message this board sends -- which this OLT
 *       demonstrably accepts, because the board provisions.
 *
 *       The old note here said "do not fix either side without an on-wire
 *       capture".  That was right while the question was WHICH convention the
 *       OLT wants; it stopped applying once the question became "why does one
 *       board use two", which is answerable from the spec and from the code.
 *    2. GET VALUE-AREA END.  rtl9602c_omci_get_fill() below fills [11,40) =
 *       29 octets; the common core fills [11,36) = 25 and uses 36..39 for the
 *       G.988 unsupported- and failed-attribute masks.  Follow-up F2.
 *    3. GET RESULT CODE.  This file answers OK unless it overran the buffer;
 *       the common core answers 0x09 ATTR_FAILED whenever the OLT asked for an
 *       attribute the model does not know.  Measured: classes 2, 5, 7, 11,
 *       131 and 262 all flip 0x00 to 0x09.
 *    4. BOARD IDENTITY.  ME 257 attribute 1 reads "HSGQ-X111W" here and
 *       "HSGQ-X400AXF" in the common ME model — a DIFFERENT PRODUCT.  The ME
 *       model layer is per-board data by design, so Luna must supply its own
 *       before it can share the message layer (plan item M5, not done yet).
 *    5. CONFIG-APPLY RESULT CODES.  Create of an existing instance 0x00 to
 *       0x07, Delete of an absent one 0x00 to 0x05, Set of an unmodelled
 *       class 0x00 to 0x04.  This file never NAKs a config message.
 *    6. AR-CLEAR.  With the acknowledgement-request bit clear this file still
 *       transmits a reply; the common core applies the change and stays
 *       silent, which is what G.988 asks for.
 *    7. UNHANDLED MESSAGE TYPE.  This file answers 0x02 NOT_SUPPORTED; the
 *       common core answers 0x00 with empty contents.
 *    8. GET NEXT OPCODE.  OMCI_MT_GET_NEXT is 0x10 just below and 0x1a in the
 *       common header — 0x10 is really the ALARM opcode, so this file answers
 *       a real Get Next through its default arm.  Follow-up F1.  Adopting the
 *       common header is not even a compile-clean no-op: it raises
 *       "OMCI_MT_GET_NEXT redefined" (verified, not predicted).
 *    9. MIB-DATA-SYNC.  This file bumps MDS on every config message; the
 *       common core does not bump it on a rejected one.  After the same
 *       sequence: 46 here against 44 there.
 *
 *   Anyone proposing the move must land F1/F2/F3, give the common ME model
 *   Luna's board data, then re-run that differential — and a green x86
 *   differential still only GATES a boot, it never proves the OLT accepts the
 *   new bytes.  X111W is off the rig, so the change cannot be validated
 *   end-to-end today, and an OMCI responder the OLT silently drops is a
 *   churn-lock: three of this file's own guard comments below were written
 *   after exactly that.
 * ============================================================================
 */

/* G.988 baseline message-type action codes (low 5 bits of msg[2]). */
#define OMCI_MT_CREATE		0x04
#define OMCI_MT_DELETE		0x06
#define OMCI_MT_SET		0x08
#define OMCI_MT_GET		0x09
#define OMCI_MT_GET_ALL_ALARMS	0x0b
#define OMCI_MT_GET_ALL_ALRM_NX	0x0c
#define OMCI_MT_MIB_UPLOAD	0x0d
#define OMCI_MT_MIB_UPLOAD_NX	0x0e
#define OMCI_MT_MIB_RESET	0x0f
/* ★★★ 0x1a, NOT 0x10 -- CORRECTED 2026-08-27, and it is a WIRE defect.
 * G.988 assigns 0x10 (16) to the ONU-autonomous ALARM and 0x1a (26) to Get
 * Next. This file said 0x10, so the driver answered alarms as if they were Get
 * Next requests and NEVER handled a real one. It surfaced only when the two
 * copies of these constants were put side by side: gpon_omci_me.h has the
 * right value and had even recorded the disagreement as "follow-up F1", which
 * nobody could act on while a second copy existed. */
#define OMCI_MT_GET_NEXT	0x1a
/* Config-apply / management action MTs the OLT issues after MIB-upload+HGU classification.
 * We ACK them OK (no real action needed to pass config-load) so the OLT completes provisioning. */
#define OMCI_MT_TEST		0x12	/* 18 — ANI-G optical test */
#define OMCI_MT_START_SW_DL	0x13	/* 19 */
#define OMCI_MT_DOWNLOAD_SEC	0x14	/* 20 */
#define OMCI_MT_END_SW_DL	0x15	/* 21 */
#define OMCI_MT_ACTIVATE_SW	0x16	/* 22 */
#define OMCI_MT_COMMIT_SW	0x17	/* 23 */
#define OMCI_MT_SYNC_TIME	0x18	/* 24 — Synchronize Time (ONT-G) */
#define OMCI_MT_REBOOT		0x19	/* 25 */

/* G.988 result/reason codes (GET / response content byte 8). */
#define OMCI_RC_OK		0x00
#define OMCI_RC_NOT_SUPPORTED	0x02
#define OMCI_RC_UNKNOWN_ME	0x04
#define OMCI_RC_ATTR_FAILED	0x09

/* Managed-Entity classes we answer with real values. */
#define OMCI_ME_ONU_DATA	2
#define OMCI_ME_ONU_G		256
#define OMCI_ME_ONU2_G		257
#define OMCI_ME_CTC_LOID_AUTH	65530	/* 0xFFFA China-Telecom LOID auth */

/* GET attribute-mask bit for attribute number N (N=1 = first attr after the
 * ManagedEntityID): bit (16 - N). (Off-by-one here breaks every discovery GET.) */
#define OMCI_ATTR_BIT(n)	(1u << (16 - (n)))

/* omci_put_be16() is the CORE's, from gpon_omci_core.h.  The copy that used to
 * sit here was byte-identical to it and existed only because the core's was
 * `static inline` inside its .c, unreachable from any other unit.  (Since the
 * 2026-09-02 AVC rebase this shell no longer calls it at all -- the core
 * builds every OMCI frame this board sends.) */

/* G.988 ME class IDs of the auto-instantiated hardware MEs we present in the
 * MIB-Upload (in addition to the discovery MEs above). */
#define OMCI_ME_CARDHOLDER	5
#define OMCI_ME_CIRCUIT_PACK	6
#define OMCI_ME_SW_IMAGE	7
#define OMCI_ME_PPTP_ETH_UNI	11	/* THE HGU GATE: gpon_ont_sync_capability counts these */
#define OMCI_ME_OLT_G		131
#define OMCI_ME_TCONT		262
#define OMCI_ME_ANI_G		263
#define OMCI_ME_UNI_G		264
#define OMCI_ME_PRIORITY_QUEUE	277
#define OMCI_ME_TRAFFIC_SCHED	278
#define OMCI_ME_VEIP		329

/*
 * Static MIB-Upload row table. Each "row" is one MIB-Upload-Next entry the OLT
 * reads: a (class, instance, attribute-mask) triple whose selected attributes
 * fit in the 26 value-bytes of one Upload-Next reply (contents = class[8..9] +
 * inst[10..11] + mask[12..13] + values[14..39]). MEs whose selected attributes
 * exceed 26 value-bytes are SPLIT into multiple rows with DISJOINT masks. The
 * row values themselves come from rtl9602c_omci_me_fill() (single source of truth, so
 * GET and Upload byte-match). Built once at probe via omci_build_mib().
 */
/*
 * ★★★ ONE CONTEXT, FROM THE COMMON CORE (2026-08-27, operator: "deduplicar por
 * dios").  This shell used to declare its OWN `struct omci_me_inst`, its OWN
 * `struct omci_mib_row`, its own 128-entry store, its own row table and its own
 * find/nth/put/del/reset -- all of it already present, and already tested
 * offline, in drivers/net/gpon.  The two type definitions were byte-for-byte
 * identical to the core's AND CARRIED THE SAME NAMES, which is duplication in
 * the form that never announces itself: nobody has to copy anything for two
 * such copies to drift, they only have to be edited on different days.
 *
 * ⚠ THE CAPACITIES ARE NOT LOST IN THE MOVE.  The core defaults to 64 MEs and
 * 72 rows (the Elnath's numbers); this board has always carried 128/200, and
 * rebasing at the smaller size would have DROPPED provisioned MEs -- a
 * regression dressed as a cleanup.  They are set per board in this directory's
 * Makefile (-DOMCI_STORE_MAX=128 -DOMCI_MIB_ROWS_MAX=200), because a capacity
 * is a board value while the store's behaviour is not.
 */
static struct omci_onu luna_onu;

/*
 * LAYER BOUNDARY: THE DISPATCHER — this is "the FSM" a reader comes looking
 * for.  Common home = omci_onu_input() in gpon-common
 * .../drivers/net/gpon/gpon_omci_core.c.  It did NOT move.  Same skeleton
 * (runt guard, DevID gate, header echo, switch on the message type), but the
 * common one additionally: suppresses the reply when the acknowledgement-
 * request bit is clear, caches and replays a retransmitted request, counts a
 * DevID 0x0b extended frame, NAKs a duplicate Create / an absent Delete / a
 * Set of an unmodelled class, skips the MIB-Data-Sync bump on a rejected
 * message, answers an unknown message type with 0x00 rather than 0x02, and
 * zeroes MDS on MIB-Reset unconditionally where this one honours the
 * mds_reset0 and omci_mds_seed module parameters.  Divergences 5, 6, 7 and 9.
 *
 * The two shells also differ in KIND, which is why the common one returns a
 * length instead of transmitting: it decides, the caller does.  This function
 * calls rtl9602c_eth_omci_xmit() itself at the bottom, so adopting the common
 * core means moving the transmit decision out to the caller as well.
 */
/*
 * Downstream OMCI -> upstream OMCI response (M2 G.988 responder). @msg is the raw
 * baseline message (2-byte CPU prefix already stripped). Builds a 48-byte reply
 * (resp MT = (MT & 0x1f) | 0x20: clears AR, sets AK, keeps DB + action), result +
 * body per type, trailer + MIC, then TX on the OMCC. Minimal-but-correct: enough
 * for the OLT to finish discovery + config. Runs in poll-timer softirq context.
 */
static void rtl9602c_eth_omci_input(struct rtl9602c_eth *ep, const u8 *msg,
				    unsigned int len)
{
	u8 resp[OMCI_LEN];
	int n;

	if (len < 8)
		return;

	/* Board-side visibility: WHICH message arrived is a fact about this OLT
	 * and this link, and the core neither logs nor should.  Rate-limited on
	 * the bulk types so a MIB upload cannot flood.
	 *
	 * ★ THE DECODE IS THE CORE'S (gpon_omci_describe), and this board GAINED
	 * something by the move: the line used to print MT as a bare number,
	 * with no message-type NAME and no AR/AK flags.  The Cortina family had
	 * a richer decode of the SAME PDU -- two implementations of one idea,
	 * neither aware of the other, and no clone detector could see it because
	 * they shared not one substring.  There is one now, and it is G.988's.
	 *
	 * ⚠ THE POLICY STAYS HERE, and it is the BETTER of the two: Luna limits
	 * only the bulk types, so a Create or an Alarm is never dropped from the
	 * log, while the Cortina side rate-limits everything.  The core cannot
	 * print, so it cannot take this decision away. */
	if (((msg[2] & 0x1f) != OMCI_MT_GET &&
	     (msg[2] & 0x1f) != OMCI_MT_MIB_UPLOAD_NX &&
	     (msg[2] & 0x1f) != OMCI_MT_MIB_UPLOAD) || net_ratelimit()) {
		char det[96];

		gpon_omci_describe(msg, len, det, sizeof(det));
		netdev_info(ep->ndev, "OMCI DS: %s\n", det);
	}

	/*
	 * ★★★ THE G.988 RESPONDER IS THE COMMON ONE (2026-08-27). Operator:
	 * migrate to the family and "adaptar los demas" onto the X400AXF,
	 * because it is the board that is verified and works -- and the
	 * X400AXF has been running exactly this core responder all along
	 * (cortina-gpon.c calls omci_onu_init/omci_onu_input).
	 *
	 * ★ THE ~200 LINES THIS REPLACES WERE NOT MERELY A COPY, THEY WERE A
	 * WEAKER ONE. The previous author had already written down what the
	 * common one does that this shell did not: it suppresses the reply
	 * when the acknowledgement-request bit is clear, caches and REPLAYS a
	 * retransmitted request instead of re-executing it (so a lost upstream
	 * response cannot bump MIB-Data-Sync twice for one OLT transaction),
	 * counts DevID 0x0b extended frames, NAKs a duplicate Create, an
	 * absent Delete and a Set of an unmodelled class, skips the MDS bump
	 * on a rejected message, and answers an unknown message type with 0x00
	 * rather than 0x02. Those were logged as "divergences 5, 6, 7 and 9"
	 * and could not be acted on while two responders existed.
	 *
	 * ★ THE CORE DECIDES, THE SHELL DOES: it returns a LENGTH and never
	 * transmits, which is why the xmit call moved out here. That is the
	 * whole boundary -- no MMIO in the core, no protocol in the shell.
	 */
	n = omci_onu_input(&luna_onu, msg, len, resp);
	if (n > 0)
		rtl9602c_eth_omci_xmit(ep, resp, n);

	/* ★★★ THE ME 268 SNOOP -- RESTORED, AND MOVED INTO THE CORE (2026-09-02).
	 *
	 * This shell used to decode the OLT's GEM-CTP Create itself, one line
	 * inside its own responder:
	 *
	 *     if (class_id == 268 && mt == OMCI_MT_CREATE && len >= 10)
	 *             gpon_omci_note_gem_create(((u16)msg[8] << 8) | msg[9]);
	 *
	 * The responder was replaced by the common one (836b76be01) and that
	 * line went with it.  It was the ONLY setter of gpon_data_gem_solicited
	 * and of the core's data_gem_solicited -- and BOTH install paths are
	 * gated on that flag, so the flag's one setter ended up inside the
	 * function the flag guards.  The WAN data GEM stopped being installed
	 * from the OLT's Create at all.  Two more consumers were orphaned by the
	 * same commit; see
	 * dev/ONU-test-case/FINDING-luna-responder-swap-orphaned-three-consumers.md
	 *
	 * ★ IT IS A QUERY, NOT A PARSE, and that is the repair rather than a
	 *   revert: the responder ALREADY stores every Set-by-Create body, so
	 *   asking it costs no callback, no new lifecycle, and nothing can be
	 *   missed while this path is busy.  The DECISION (bidirectional only,
	 *   never the OMCC's gem, never the broadcast gem, never Port-ID 0)
	 *   lives once in the core, where both families reach it and where
	 *   gpon_dgem_test exercises it on x86 with no board.
	 *
	 * ⚠ AND IT KEEPS NO EDGE VARIABLE, deliberately -- my first spelling of
	 *   this did, and it carried the very defect it is here to repair: the
	 *   core clears its store on an on-wire MIB-Reset WITHOUT this shell
	 *   being called, so a shell-side "already reported" latch would still
	 *   hold the old port-id, the edge would never fire again when the OLT
	 *   re-provisioned the SAME one, and the WAN would be dead until a
	 *   reboot.  A second shadow of state somebody else owns is how that
	 *   whole family of bug starts.
	 * ★ note_gem_create() is idempotent (it re-arms only when the port
	 *   MOVES) and DS OMCI is management-rate, so the unconditional call is
	 *   both correct and cheap.
	 */
	{
		u16 gem = 0;

		if (omci_data_gem_port(&luna_onu, gpon_omcc_gem(),
				       GPON_MCAST_GEM, &gem))
			gpon_omci_note_gem_create(gem);
	}
}

/*
 * LAYER BOUNDARY -- RESOLVED 2026-09-02: the autonomous AVC is the CORE's
 * (omci_emit_avc() / omci_onu_emit_veip_up_avc(), gpon-common
 * .../drivers/net/gpon/gpon_omci_core.c).  This shell used to build the MT
 * 0x11 frame itself (rtl9602c_eth_omci_avc); the recorded blocker was
 * divergence 1 -- the two MIC polynomials -- which died 2026-09-01 when this
 * file adopted the core's omci_set_mic(), and the comment's remaining claim
 * (a differing value clamp) was STALE: both sides clamp at 30.  Byte
 * equality old-vs-core proven on x86 over a class/inst/mask/vlen sweep
 * (hoist2-luna-eth harness, 2026-09-02).  Declared side effect of the
 * rebase: the core bumps luna_onu.avc_count and sets avc_veip_up_sent.
 * The shell keeps the trap_on gate, the xmit and the log line -- the core
 * returns bytes, the shell transmits (the established boundary).
 *
 * WHY the AVC exists at all (kept from the old builder's comment): the OLT
 * NEVER polls (GETs) the data-plane MEs after creating them (verified live:
 * it only CREATE/SETs ME266/268/47/45/329/...); its per-class *_avc handlers
 * wait for the ONU to report the port operational and gate DOWNSTREAM
 * user-data forwarding on it.  A purely-reactive responder leaves the OLT
 * filling our downstream with idle GEM and forwarding no data.  TID=0 marks
 * an autonomous notification (AR=0, AK=0).
 */
/*
 * Report the HGU's WAN-egress port operational so the OLT un-gates downstream user data.
 * For an HGU the downstream WAN data egresses to the VEIP (ME329); the OLT's
 * virtual_ethernet_interface_point_avc gate keys on its operational state. Called from the
 * GPON FSM a few seconds after O5 (config-apply complete). VEIP inst 0x0601 + attr2
 * (operational state) mask 0x4000, value 0 = enabled (G.988).
 */
/*
 * ★★★ THE LIVE OPTICAL LEVELS -- RE-CONNECTED (2026-09-02).
 *
 * gpon_anig_optical_omci() had been DEFINED, DECLARED, and called from NOWHERE
 * in the whole tree since 836b76be01: its only caller lived inside this
 * shell's own OMCI responder and was deleted with it.  So the OLT's ME 263
 * ANI-G #10/#14 read back the conformant STATIC seed omci_onu_init() writes,
 * for months, while the board was busy measuring the real fiber every 3 s and
 * throwing the answer away.  It is the same orphaning as the ME 268 snoop
 * above and was found in the same audit.
 *
 * ★ IT PUBLISHES, IT DOES NOT GET CALLED BACK -- the mechanism Elnath already
 *   uses (cortina-gpon.c:2857), so the two families feed the core the same way
 *   and no new seam was invented for the second one.
 *
 * ★ CALLED FROM THE DDM WORKQUEUE (process context, ~3 s), never from the GET
 *   path: the i2c read busy-waits, and a stall in the OMCI softirq risks
 *   missed US-grant timing and an OLT-side deactivate.  The cache read itself
 *   is I2C-free, which is why this shim may take the value in any context.
 *
 * ★ NO LOCK, and that is a decision rather than an omission: these are two
 *   independent aligned scalars in the G.988 wire form, written here and read
 *   one at a time by the ANI-G GET arm.  A torn pair would report an RX from
 *   one 3-second sample beside a TX from the next -- which is what an optical
 *   reading IS.  Elnath holds a spinlock because its context has one already;
 *   inventing one here would buy nothing this datum needs.
 */
void rtl9602c_eth_omci_set_optical(s16 rx_level, s16 tx_level)
{
	omci_onu_set_optical(&luna_onu, (u16)rx_level, (u16)tx_level);
}

void rtl9602c_eth_omci_report_oper_up(void)
{
	struct rtl9602c_eth *ep = g_ep;
	u8 msg[OMCI_LEN];

	if (!ep || !ep->omci_trap_on)		/* OMCC SID must be armed to TX OMCI */
		return;
	omci_onu_emit_veip_up_avc(&luna_onu, msg);
	rtl9602c_eth_omci_xmit(ep, msg, sizeof(msg));
	netdev_info(ep->ndev, "OMCI AVC: class=%u inst=%#x mask=%04x v0=%02x\n",
		    OMCI_ME_VEIP, 0x0601, 0x4000, 0);
}
EXPORT_SYMBOL(rtl9602c_eth_omci_report_oper_up);

/*
 * `napi_ctx` says whether the caller is INSIDE napi->poll. It decides how the
 * frame is handed up, and it is not cosmetic: napi_gro_receive() may only be
 * called from NAPI poll context (it uses the NAPI instance's own GRO state),
 * while the legacy fallback below reaches here from a plain timer callback.
 * Passing the wrong one is a bug on the path nobody exercises -- which is
 * exactly the path kept as the safety net until INTC input 26 is proven.
 */
static int rtl9602c_eth_rx(struct rtl9602c_eth *ep, int budget, bool napi_ctx)
{
	struct net_device *ndev = ep->ndev;
	int rx_done = 0;

	while (rx_done < budget) {
		unsigned int i = ep->rx_head;
		u32 opts1 = ep->rx_ring[i].opts1;
		struct sk_buff *skb;
		u32 len;

		if (opts1 & D_OWN)		/* still HW-owned: nothing more */
			break;
		ep->dbg_filled++;		/* HW handed this descriptor back */

		len = (opts1 & RXD_LEN_MASK);
		skb = ep->rx_skb[i];
		dma_unmap_single(ep->dev, ep->rx_buf_dma[i], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);

		/* DS-OMCI classifier + BOTH-ends length guard: hoisted to
		 * rtl9602c_rx_is_ds_omci() (flowcore) -- the guard is the
		 * fuzz/ASan-found OOB fix and runs before any data byte is
		 * read; fuzz_rx.c can now drive the exact shipping predicate
		 * on x86.  The trap-vs-switch-routed match shapes and their
		 * live evidence are documented at the function. */
		if (rtl9602c_rx_is_ds_omci(ep->omci_trap_on,
					   ep->rx_ring[i].opts2,
					   ep->rx_ring[i].opts3,
					   skb->data, len, ep->swm->pon_port,
					   RX_CPU_PREFIX, RX_BUF_SIZE)) {
			/* DS OMCI on the OMCC. Capture for /proc, then hand the raw G.988
			 * message (prefix stripped) to the responder. */
			ep->dbg_omci_rx++;
			ep->dbg_omci_rxlen = len - RX_CPU_PREFIX;
			memcpy(ep->dbg_omci_rxbuf, skb->data + RX_CPU_PREFIX,
			       min_t(unsigned int, len - RX_CPU_PREFIX,
				     sizeof(ep->dbg_omci_rxbuf)));
			rtl9602c_eth_omci_input(ep, skb->data + RX_CPU_PREFIX,
						len - RX_CPU_PREFIX);
			dev_kfree_skb_any(skb);
		} else if (rtl9602c_rx_frame_bad(opts1, RXD_CRCERR | RXD_RCDF,
						 len, RX_CPU_PREFIX,
						 RX_BUF_SIZE)) {
			/* bad-frame verdict hoisted to flowcore: completes the
			 * RX classification trio fuzz_rx.c drives on x86 */
			ndev->stats.rx_errors++;
			ep->dbg_err++;
			dev_kfree_skb_any(skb);
		} else {
			struct net_device *rdev = ndev;	/* receive netdev: eth0, or gpon0 for PON-port WAN frames */

			ep->dbg_good++;
			/* Do NOT software-strip a 4-byte FCS here: on this GMAC the
			 * RX descriptor length already excludes most of the FCS, so
			 * subtracting ETH_FCS_LEN on top of the 2-byte CPU prefix
			 * left the IP packet 2 bytes short (truncated -> dropped,
			 * Icmp InEcho=0). The stack uses the L3 length, so any
			 * trailing FCS/pad bytes are ignored harmlessly. */
			skb_put(skb, len);
			/* Capture the raw frame (pre-pull) for /proc diag. */
			ep->dbg_rxlen = len;
			memcpy(ep->dbg_rxbuf, skb->data,
			       min_t(unsigned int, len, sizeof(ep->dbg_rxbuf)));
			/* opts3 src_port_num [19:16] = ingress port. WAN demux: frames that
			 * ingressed on the PON port (2) are de-encapsulated DATA-GEM traffic ->
			 * deliver to the gpon0 WAN netdev; LAN-port frames go to eth0. (OMCI was
			 * already trapped above, so PON frames here are pure user data.) Also learn
			 * the LAN uplink port so CPU->LAN TX egresses correctly. */
			{
				const u8 *dst = skb->data + RX_CPU_PREFIX;
				unsigned int sp = (ep->rx_ring[i].opts3 >> 16) & 0xf;

				/* Route drained WAN frames (and any unicast to the gpon0 MAC) to gpon0;
				 * keep everything else — INCLUDING LAN broadcast/ARP — on eth0 so the LAN
				 * bridge works. (An earlier is_multicast->gpon0 rule mis-sent LAN ARP to
				 * the WAN and broke 192.168.1.1 reachability.) */
				/* The verdict (PON src port, the PON-IP-NIC drain
				 * signature opts3[31:20]==0x23e -- confirmed live
				 * 2026-06-15 -- or DA == the WAN MAC) is hoisted to
				 * rtl9602c_rx_wan_demux() (flowcore). */
				if (ep->wan_ndev &&
				    rtl9602c_rx_wan_demux(ep->rx_ring[i].opts3, dst,
							  ep->wan_ndev->dev_addr,
							  ep->swm->pon_port))
					rdev = ep->wan_ndev;
				else
					ep->host_port = sp;
				/* DS-OFFER diag (find why the WAN DHCP OFFER misses gpon0):
				 * log any DHCP-to-client (UDP dst port 68) DS frame + where
				 * it routed. dst = eth hdr; [12:13]=ethertype 0x0800,
				 * [23]=IP proto 0x11(UDP), [36:37]=UDP dst port 0x0044(68). */
				if (len >= RX_CPU_PREFIX + 38 &&
				    dst[12] == 0x08 && dst[13] == 0x00 &&
				    dst[23] == 0x11 &&
				    dst[36] == 0x00 && dst[37] == 0x44)
					pr_info("rtl9602c-eth: DHCP-DS sp=%u opts3=%08x dst=%pM -> %s\n",
						sp, ep->rx_ring[i].opts3, dst,
						rdev == ep->wan_ndev ? "gpon0" : "host");
			}
			/*
			 * The switch CPU port prepends a 2-byte offset word
			 * ahead of the Ethernet header on every frame delivered
			 * to the CPU (the GMAC CPU-port RX framing). Strip it so
			 * eth_type_trans() parses the real dst-MAC / ethertype;
			 * without this every RX frame is malformed and the stack
			 * silently drops it (no ARP reply, no neigh resolution).
			 */
			skb_pull(skb, RX_CPU_PREFIX);
			skb->protocol = eth_type_trans(skb, rdev);
			rdev->stats.rx_packets++;
			rdev->stats.rx_bytes += len;
			/*
			 * NAPI poll context: use the receive path, not netif_rx
			 * -- the same call the 9607C sibling in this tree makes,
			 * for the same reason. netif_rx() re-queues every frame
			 * onto the per-CPU backlog and raises a SECOND softirq
			 * per packet, and forgoes GRO entirely; from poll
			 * context there is nothing to gain by it. The legacy
			 * timer fallback still uses netif_rx, because it is NOT
			 * in poll context and napi_gro_receive would be invalid
			 * there.
			 */
			if (napi_ctx)
				napi_gro_receive(&ep->napi, skb);
			else
				netif_rx(skb);
		}
		/* hand the slot back to HW with a fresh buffer */
		if (rtl9602c_eth_refill(ep, i)) {
			ndev->stats.rx_dropped++;
			/* leave CPU-owned; will retry next poll */
			break;
		}
		ep->rx_head = (i + 1) % RX_RING_SIZE;
		rx_done++;
	}
	return rx_done;
}

static void rtl9602c_eth_tx_reclaim(struct rtl9602c_eth *ep)
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
	    (ep->tx_head - ep->tx_dirty) < TX_RING_SIZE - 1 - OMCI_RESV)
		netif_wake_queue(ep->ndev);
}

/* The GMAC TX DMA is a self-polling sequential fetch engine: when it drains a ring to
 * the software producer head it PARKS on the current descriptor and only resumes on a
 * fresh go/poll doorbell. The continuous LAN stream re-arms it on every xmit so it never
 * parks; the sparse/bursty OMCI inject leaves it parked — and because the multi-ring TX
 * DMA is shared, a parked ring freezes ALL GMAC TX (LAN included; txok frozen). Stock
 * guards this with a TDU interrupt + a periodic re-kick timer; we are IRQ-less, so re-
 * kick once per poll tick: if a ring has outstanding work AND its HW cursor (R_TxCDO)
 * still points at an OWN (un-fetched) descriptor, re-assert its go-bit. OWN-guarded so
 * an idle ring is never needlessly poked. Facts re-expressed clean-room; caller holds
 * ep->tx_lock. */
static void rtl9602c_eth_tx_rekick(struct rtl9602c_eth *ep)
{
	bool parked = false;

	/*
	 * Park detection keys on the OLDEST PENDING descriptor (dirty front),
	 * NOT the HW cursor: a latched engine sits parked on the slot it
	 * drained to — which is already consumed (OWN=0) — so an OWN-guard at
	 * the TxCDO slot reads "no work" forever while slots dirty..head-1 sit
	 * published behind its back (HW-measured: head=67 dirty=4 cdo->slot3
	 * OWN=0, ring full, watchdog blind). "HW has not consumed the oldest
	 * published descriptor" is briefly true in normal flow too; the 250ms
	 * persistence filter below separates a park from in-flight latency.
	 */
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
		if (rtl9602c_omci_doorbell(omci_tx_ring, 0xff, &h, &dmask))
			ep_wr(ep, R_IO_CMD1, ep_rd(ep, R_IO_CMD1) | dmask);
		else
			ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | dmask);
	}

	/*
	 * Park watchdog. A briefly-parked ring is normal (the doorbell above is
	 * the equivalent of the stock device's ~10ms TDU kick); a park that survives re-kicks
	 * with dirty frozen >250ms is the fatal latch — only the IP-block
	 * power-cycle recovers it. Track progress via the combined dirty count
	 * so either ring advancing resets the clock.
	 */
	if ((!tx_recover && !tx_softrearm) || ep->closing) {
		ep->stall_since = 0;
	} else if (!parked) {
		ep->stall_since = 0;
		ep->stall_level = 0;
	} else if (!ep->stall_since ||
		   ep->tx_dirty + ep->otx_dirty != ep->stall_lastdirty) {
		if (ep->stall_since)
			ep->stall_level = 0;	/* progress: restart the ladder */
		ep->stall_since = jiffies;
		ep->stall_lastdirty = ep->tx_dirty + ep->otx_dirty;
	} else if (time_after(jiffies,
			      ep->stall_since + msecs_to_jiffies(60))) {
		if (tx_recover) {
			/* Level 1: full GMAC reset + reprogram (CMD.RST by
			 * default; BSP_IP_SEL power-cycle if recover_rst=0). */
			ep->stall_since = 0;
			ep->stall_level = 0;
			schedule_work(&ep->recover_work);
		}
	}
}

/*
 * NAPI poll: drain RX up to `budget`, then (under tx_lock) reclaim both TX rings
 * and un-park any stalled ring. When RX is fully drained (work < budget) complete
 * NAPI and re-arm the GMAC IRQ masks. The hard ISR schedules us. Replaces the 2ms
 * timer as the primary servicing path.
 */
static int rtl9602c_eth_napi_poll(struct napi_struct *napi, int budget)
{
	struct rtl9602c_eth *ep = container_of(napi, struct rtl9602c_eth, napi);
	unsigned long flags;
	int work;

	ep->dbg_poll++;
	work = rtl9602c_eth_rx(ep, budget, true);	/* inside napi->poll */
	spin_lock_irqsave(&ep->tx_lock, flags);
	rtl9602c_eth_tx_reclaim(ep);
	rtl9602c_eth_omci_reclaim(ep);
	rtl9602c_eth_tx_rekick(ep);	/* un-park a stalled TX ring */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (work < budget) {
		napi_complete_done(napi, work);
		/* Re-arm. W1C-ack any status latched while masked FIRST so we do
		 * not immediately re-fire on a stale bit, THEN set the mask bits. */
		iowrite16(ioread16(ep->base + R_ISR), ep->base + R_ISR);
		ep_wr(ep, R_ISR1, ep_rd(ep, R_ISR1));
		iowrite16(ioread16(ep->base + R_IMR) | IMR_RX_BITS, ep->base + R_IMR);
		ep_wr(ep, R_IMR0, ep_rd(ep, R_IMR0) | IMR0_TX_BITS);
	}
	return work;
}

/*
 * Slow TX-unpark backstop (IRQ-driven mode). The GMAC TX DMA parks a SPARSE ring
 * at its producer head and only a fresh doorbell resumes it; a TX-completion IRQ
 * may never fire for an already-parked ring, so a low-rate timer re-kicks it
 * (OWN-guarded -> no-op when idle). 100ms is ample for control-rate OMCI; LAN
 * xmit re-arms the ring inline on every frame.
 */
static void rtl9602c_eth_rekick_timer(struct timer_list *t)
{
	struct rtl9602c_eth *ep = timer_container_of(ep, t, poll_timer);
	unsigned long flags;

	spin_lock_irqsave(&ep->tx_lock, flags);
	rtl9602c_eth_tx_reclaim(ep);
	rtl9602c_eth_omci_reclaim(ep);
	rtl9602c_eth_tx_rekick(ep);
	spin_unlock_irqrestore(&ep->tx_lock, flags);
	/* While a park is pending, tighten the cadence so the 60ms escalation
	 * ladder is not quantised by the 100ms backstop interval. */
	mod_timer(&ep->poll_timer, jiffies +
		  (ep->stall_since ? msecs_to_jiffies(20) : REKICK_INTERVAL));
}

/*
 * Legacy pure-poll fallback (ep->irq <= 0): the original 2ms full-drain loop,
 * kept as the safety net until INTC input 26 is proven on silicon. RX_RING_SIZE
 * budget = drain the whole ring each tick (the prior behaviour).
 */
static void rtl9602c_eth_poll(struct timer_list *t)
{
	struct rtl9602c_eth *ep = timer_container_of(ep, t, poll_timer);
	unsigned long flags;

	ep->dbg_poll++;
	rtl9602c_eth_rx(ep, RX_RING_SIZE, false);	/* timer callback, NOT napi->poll */
	spin_lock_irqsave(&ep->tx_lock, flags);
	rtl9602c_eth_tx_reclaim(ep);	/* under tx_lock (races the OMCI inject's locked reclaim on shared ring 0) */
	rtl9602c_eth_omci_reclaim(ep);
	rtl9602c_eth_tx_rekick(ep);	/* TDU-style keep-alive: un-park a stalled TX ring */
	spin_unlock_irqrestore(&ep->tx_lock, flags);
	mod_timer(&ep->poll_timer, jiffies + POLL_INTERVAL);
}

static int rtl9602c_eth_alloc_rings(struct rtl9602c_eth *ep)
{
	unsigned int i;

	ep->rx_ring = dma_alloc_coherent(ep->dev,
			RX_RING_SIZE * sizeof(struct rx_desc),
			&ep->rx_ring_dma, GFP_KERNEL);
	ep->tx_ring = dma_alloc_coherent(ep->dev,
			TX_RING_SIZE * sizeof(struct tx_desc),
			&ep->tx_ring_dma, GFP_KERNEL);
	/* Dedicated US-OMCI TX ring (the OMCC ring, default ring 4). */
	ep->otx_ring = dma_alloc_coherent(ep->dev,
			OTX_RING_SIZE * sizeof(struct tx_desc),
			&ep->otx_ring_dma, GFP_KERNEL);
	/* Idle filler for the unused HW TX rings (see struct comment). */
	ep->dummy_ring = dma_alloc_coherent(ep->dev,
			DUMMY_RING_SIZE * sizeof(struct tx_desc),
			&ep->dummy_ring_dma, GFP_KERNEL);
	if (!ep->rx_ring || !ep->tx_ring || !ep->otx_ring || !ep->dummy_ring)
		return -ENOMEM;

	ep->rx_head = ep->tx_head = ep->tx_dirty = 0;
	ep->otx_head = ep->otx_dirty = 0;
	ep->tx_rot = ep->otx_rot = 0;	/* fresh rings: HW walk starts at slot 0 */
	ep->omci_r0_last_slot = -1;	/* no OMCI on shared LAN ring 0 yet */
	ep->host_port = 0xff;		/* uplink port unknown until first RX */
	for (i = 0; i < TX_RING_SIZE; i++) {
		ep->tx_ring[i].opts1 = (i == TX_RING_SIZE - 1) ? D_EOR : 0;
		ep->tx_skb[i] = NULL;
	}
	for (i = 0; i < OTX_RING_SIZE; i++) {
		/* EOR (wrap) in word0 on the last slot; OWN (word2 bit31) clear so
		 * the slot starts CPU-owned (idle) until the first OMCI submit. */
		ep->otx_ring[i].opts1 = (i == OTX_RING_SIZE - 1) ? D_EOR : 0;
		ep->otx_ring[i].opts2 = 0;
		ep->otx_skb[i] = NULL;
	}
	for (i = 0; i < DUMMY_RING_SIZE; i++) {
		/* OWN clear (opts1 bit31 = 0) => always CPU-owned => the TX engine
		 * idles on this ring; EOR on the last slot. Never written again. */
		ep->dummy_ring[i].opts1 = (i == DUMMY_RING_SIZE - 1) ? D_EOR : 0;
		ep->dummy_ring[i].opts2 = 0;
	}
	for (i = 0; i < RX_RING_SIZE; i++) {
		if (rtl9602c_eth_refill(ep, i))
			return -ENOMEM;
	}
	return 0;
}

static void rtl9602c_eth_free_rings(struct rtl9602c_eth *ep)
{
	unsigned int i;

	for (i = 0; i < RX_RING_SIZE; i++) {
		if (ep->rx_skb[i]) {
			dma_unmap_single(ep->dev, ep->rx_buf_dma[i],
					 RX_BUF_SIZE, DMA_FROM_DEVICE);
			dev_kfree_skb_any(ep->rx_skb[i]);
			ep->rx_skb[i] = NULL;
		}
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
	if (ep->rx_ring)
		dma_free_coherent(ep->dev, RX_RING_SIZE * sizeof(struct rx_desc),
				  ep->rx_ring, ep->rx_ring_dma);
	if (ep->tx_ring)
		dma_free_coherent(ep->dev, TX_RING_SIZE * sizeof(struct tx_desc),
				  ep->tx_ring, ep->tx_ring_dma);
	if (ep->otx_ring)
		dma_free_coherent(ep->dev, OTX_RING_SIZE * sizeof(struct tx_desc),
				  ep->otx_ring, ep->otx_ring_dma);
	if (ep->dummy_ring)
		dma_free_coherent(ep->dev, DUMMY_RING_SIZE * sizeof(struct tx_desc),
				  ep->dummy_ring, ep->dummy_ring_dma);
	ep->rx_ring = NULL;
	ep->tx_ring = NULL;
	ep->otx_ring = NULL;
	ep->dummy_ring = NULL;
}

/*
 * GMAC0 hard ISR (IRQF_SHARED, INTC input 26). Snapshot the masked RX status
 * (ISR & 0xf835) and per-ring TX-completion status (ISR1 & 0x3f); if neither is
 * ours return IRQ_NONE so a shared line keeps dispatching to co-handlers. Mask
 * our sources (so the level line de-asserts), W1C-ack the latched bits, and hand
 * the work to NAPI. Mirrors the stock device's ISR handling.
 */
static irqreturn_t rtl9602c_eth_isr(int irq, void *dev_id)
{
	struct rtl9602c_eth *ep = dev_id;
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

/*
 * Halt the GMAC DMA + IRQ machinery (the stock stop sequence: IO_CMD/IO_CMD1 = 0,
 * masks 0, W1C-ack everything, settle). Caller holds tx_lock or is open()
 * before anything runs.
 */
/* The body is the family's (luna_eth_regs.h): both drivers had it, identical
 * but for the struct that reached `->base`. */
static void rtl9602c_hw_stop(struct rtl9602c_eth *ep)
{
	luna_eth_hw_stop(ep->base);
}

/*
 * Full GMAC register program in the stock init ORDER, for an engine
 * whose IO_CMD is 0 (fresh from rtl9602c_hw_stop + the IP-block power-cycle):
 * CMD/TCR/RCR/CONFIG -> cpu-tag -> ring pointers (TxCDO written 16-BIT like
 * stock; a 32-bit write would also clobber 0x1306) -> MSR/IDR/MAR ->
 * IO_CMD1-then-IO_CMD (the 0->config enable edge that latches the multi-ring
 * fetch engine) -> ISR ack -> IMR unmask. Ring bases honour the recovery
 * rotation (TxFDP -> first pending slot). Values are the LIVE-STOCK operating
 * set, not U-Boot's polled config.
 */
static void rtl9602c_hw_program(struct rtl9602c_eth *ep)
{
	struct net_device *ndev = ep->ndev;
	unsigned int k, oring = rtl9602c_omci_hwring(omci_tx_ring);
	u32 desnum, rcr;

	iowrite8(0x0A, ep->base + 0x3B);	/* CMD: RxChkSum|RxJumboSupport */
	ep_wr(ep, R_TCR, 0x00000C00);		/* TX pad ON (bit0=0) */
	rcr = 0x0000000E;
	if (ndev->flags & (IFF_PROMISC | IFF_ALLMULTI))
		rcr |= BIT(0);			/* re-apply AcceptAllPhys */
	ep_wr(ep, R_RCR, rcr);
	/* Stock O5 golden CONFIG = 0x21000000 (Rff 2k + rx-mring int split);
	 * after a true reset the split bit must be set by us, the per-ring
	 * IMR0/ISR1 model the driver already uses assumes it. */
	ep_wr(ep, R_CONFIG, 0x21000000);
	ep_wr(ep, 0x24, 0x010c0000);		/* stock O5 */
	/* cpu-tag ADD engine: off-then-on edge re-latches it (see open notes) */
	ep_wr(ep, R_CPUTAGCR, 0x00000000u);
	wmb();
	ep_wr(ep, R_CPUTAGCR, 0x901eff04u);
	ep_wr(ep, R_CPUTAG1CR, CPUTAG1_OMCI_SID(RTL9602C_OMCC_SID) | CPUTAG1_LOW);	/* = 0x4002 (live-stock ref ONU devmem) */
	/* SWITCH side of the cpu-tag engine (param: 0 = plain-LAN-safe). */
	if (ep->sw) {
		/* OFF->ON edge re-latches the SWITCH CPU-port cpu-tag PARSER, mirroring the
		 * GMAC CPUTAGCR re-latch above. Behavioral note: TAG_AWARE
		 * re-arms the CPU-port HSB cpu-tag parser; a straight write can leave the parser
		 * half-armed so it ignores the descriptor's PSEL/PON_SID and the cpu-tagged US
		 * OMCI falls through to L2 flood instead of PSEL direct-TX to the PON/US-NIC.
		 * Clear-then-set forces the parser to re-sample TAG_AWARE/TRAP_INSERT. */
		iowrite32(0, ep->sw + SW_MAC_CPU_TAG_CTRL);
		wmb();					/* clear must land before the re-arm edge */
		iowrite32(sw_tagaware, ep->sw + SW_MAC_CPU_TAG_CTRL);
		iowrite32(0x00400034, ep->sw + SW_FC_P_LO_TH);
		iowrite32(0x00f000ea, ep->sw + SW_FC_P_FCOFF_HI_TH);
		iowrite32(0x00400034, ep->sw + SW_FC_P_FCOFF_LO_TH);
	}

	/* Ring pointers, programmed while IO_CMD==0 (stock order; CDO is
	 * writable only in this stopped state). */
	ep_wr(ep, R_TxFDP1, ep->tx_ring_dma | DMA_BUS_WINDOW);
	iowrite16(0, ep->base + R_TxCDO1);
	if (omci_tx_ring != 0) {
		ep_wr(ep, R_TxFDP(oring), ep->otx_ring_dma | DMA_BUS_WINDOW);
		iowrite16(0, ep->base + R_TxCDO(oring));
	}
	for (k = 1; k <= 4; k++) {	/* idle dummy on the unused HW rings */
		if (k == oring)
			continue;
		ep_wr(ep, R_TxFDP(k), ep->dummy_ring_dma | DMA_BUS_WINDOW);
		iowrite16(0, ep->base + R_TxCDO(k));
	}
	ep_wr(ep, R_RxFDP, ep->rx_ring_dma | DMA_BUS_WINDOW);
	desnum = luna_gmac_rxdesnum_pack(RX_RING_SIZE, TH_ON_VAL, TH_OFF_VAL);
	ep_wr(ep, R_RxDesNum, desnum);
	ep_wr(ep, R_RxCDO, luna_gmac_rxcdo_pack(RX_RING_SIZE));
	for (k = 0; k < 7; k++)		/* every RX class -> ring 0 */
		ep_wr(ep, R_RRING_ROUTING1 + k * 4, 0);

	/* MSR top byte: see the msr_top param note (0xf0 kills sparse TX). */
	ep_wr(ep, 0x58, (ep_rd(ep, 0x58) & 0x00ffffff) | ((msr_top & 0xffu) << 24));
	rtl9602c_eth_set_hwaddr(ep, ndev->dev_addr);	/* IDR wiped by the reset */
	iowrite32(0xffffffff, ep->base + 0x08);		/* MAR0 */
	iowrite32(0xffffffff, ep->base + 0x0C);		/* MAR4 */

	/* The enable edge: IO_CMD1 first, IO_CMD last (stock start order). */
	ep_wr(ep, R_IO_CMD1, IOCMD1_STOCK);
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
static void rtl9602c_ipsel_cycle(void)
{
	luna_ipsel_cycle_gmac0();
}

/*
 * TX-DMA park recovery (process context, scheduled by the rekick watchdog).
 * The stock device's only known un-wedge: stop -> IP-block power-cycle -> full
 * reprogram + IO_CMD edge. The TX rings are re-armed ROTATED so TxFDP points at
 * the first pending descriptor: nothing in flight is dropped, the restarted
 * engine resumes exactly where the old one parked. RX descriptors are
 * re-published from slot 0 (undrained frames are the recovery tax).
 */
static void rtl9602c_eth_recover_work(struct work_struct *work)
{
	struct rtl9602c_eth *ep = container_of(work, struct rtl9602c_eth,
					       recover_work);
	struct net_device *ndev = ep->ndev;
	unsigned long flags;
	unsigned int i;
	bool use_rst = !!recover_rst;	/* snapshot: param is runtime-writable */

	if (ep->closing || !netif_running(ndev))
		return;

	ep->in_recovery = true;
	netdev_warn(ndev,
		    "TX-DMA parked (head=%u dirty=%u otx=%u/%u cdo0=%u txok=%u IO_CMD=%08x ISR1=%08x): IP-block power-cycle recovery #%u\n",
		    ep->tx_head, ep->tx_dirty, ep->otx_head, ep->otx_dirty,
		    ioread16(ep->base + R_TxCDO(0)), ep_rd(ep, 0x10) >> 16,
		    ep_rd(ep, R_IO_CMD), ep_rd(ep, R_ISR1),
		    ep->dbg_tx_recover + 1);

	netif_stop_queue(ndev);
	napi_disable(&ep->napi);
	timer_delete_sync(&ep->poll_timer);
	if (!use_rst && ep->irq > 0)
		disable_irq(ep->irq);	/* gated block MMIO would bus-abort
					 * (ipsel path only; RST is GMAC-local) */

	spin_lock_irqsave(&ep->tx_lock, flags);
	rtl9602c_hw_stop(ep);
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (use_rst) {
		/* CMD.RST core soft-reset: microseconds, no IP-block gating.
		 * Self-clears on completion; fall through to the reprogram
		 * either way (a timeout leaves us no worse than before). */
		int n;

		iowrite8(0x0A | 0x01, ep->base + 0x3B);
		for (n = 0; n < 1000 && (ioread8(ep->base + 0x3B) & 1); n++)
			udelay(1);
	} else {
		rtl9602c_ipsel_cycle();
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	/* Last-resort path (off by default — resets kill the switch egress on
	 * this board): drop everything in flight, re-arm both rings from
	 * scratch (engine stopped => CDO genuinely resets), align at 0. */
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
	for (i = 0; i < RX_RING_SIZE; i++)
		ep->rx_ring[i].opts1 = D_OWN | RX_BUF_SIZE |
				       ((i == RX_RING_SIZE - 1) ? D_EOR : 0);
	ep->rx_head = 0;
	wmb();				/* ring state before the engine restart */
	rtl9602c_hw_program(ep);
	rtl9602c_tx_align(ep);		/* CDO=0 post-reset -> rot 0 */
	ep->dbg_tx_recover++;
	ep->stall_since = 0;
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (!use_rst && ep->irq > 0)
		enable_irq(ep->irq);
	ep->in_recovery = false;
	napi_enable(&ep->napi);
	mod_timer(&ep->poll_timer, jiffies + 1);
	netif_wake_queue(ndev);
}

/* gpon_pbo_init() — re-run the gpon driver's full PON US/DS-NIC bring-up after the
 * GMAC reset so the US-NIC latches against the fresh GMAC — is declared in the
 * shared rtl9602c_gpon_nic.h. */

/*
 * U-Boot LAN-RX-enable resync — the GMAC<->switch link
 * bring-up that the GMAC IP-block power-cycle (rtl9602c_ipsel_cycle) tears down.
 * U-Boot established the GMAC<->switch sync ONCE via the SoC 0xB8000044
 * RDY_FOR_PATCH handshake (poll bit1 -> GPHY analog patch -> switch CPU-port /
 * isolation / VLAN config -> set bit0 patch-done) and never re-ran it; that is
 * exactly why gmac_reset=1 used to PERMANENTLY kill CPU->switch egress (the
 * "reset desyncs the U-Boot GMAC<->switch IP sync" catch-22). Re-running this
 * AFTER the reset re-establishes egress — and lets the subsequent stock
 * GMAC init (hw_program) re-establish the GMAC0-TX->US-NIC internal direct-TX link
 * the US-OMCI rides (the non-register init-state the stock/ours diff proved is the gap).
 * The bootloader's switch-register accesses correspond to sw_wr(off) here.
 */
static void rtl9602c_uboot_swcore_bringup(struct rtl9602c_eth *ep)
{
	void __iomem *sysstat = (void __iomem *)0xb8000044ul;	/* SoC SYSREG */
	int to;

	if (!ep->sw)
		return;
	/* wait for RDY_FOR_PATCH (0xB8000044 bit1) after the IP-block reset */
	for (to = 0; to < 200000 && !(readl(sysstat) & 0x2); to++)
		udelay(1);
	/* GPHY analog patch (0x6485 variant only), switch regs 0x10004/0x0/0x4 */
	iowrite32(0xa0000000, ep->sw + SW_CHIP_INFO);
	if ((ioread32(ep->sw + SW_CHIP_INFO) & 0xffff) == 0x6485) {
		iowrite32(0x0000fffb, ep->sw + 0x0);
		iowrite32(0x0061b844, ep->sw + 0x4);
		iowrite32(0x0021b906, ep->sw + 0x4);
		iowrite32(0x0021b906, ep->sw + 0x4);
	}
	iowrite32(0x0, ep->sw + SW_CHIP_INFO);
	/* the SECOND site of the same register -- from the chip table too, or the
	 * conversion would be half done, which is worse than none: one write
	 * would follow a corrected offset and the other would not. */
	if (ep->swm->gphy_misc)
		iowrite32(1, ep->sw + ep->swm->gphy_misc);	/* PATCH_PHY_DONE */
	msleep(500);
	iowrite32(0x00003000, ep->sw + 0x0);	/* GPHY power down */
	iowrite32(0x0060a400, ep->sw + 0x4);
	msleep(500);
	iowrite32(0x00001140, ep->sw + 0x0);	/* GPHY power up + autoneg */
	iowrite32(0x0061a400, ep->sw + 0x4);
	msleep(500);
	iowrite32(0,          ep->sw + 0x230c4);	/* SVLAN uplink port */
	iowrite32(0x003fffff, ep->sw + 0x27000);	/* port isolation */
	iowrite32(0x003fffff, ep->sw + 0x27004);
	iowrite32(0x00000196, ep->sw + 0x18c);		/* CPU port ability */
	iowrite32(0x00000fff, ep->sw + 0x1c0);		/* CPU port force mode */
	iowrite32(0x00012bbd, ep->sw + SW_METER_TB_CTRL);	/* meter tick-token */
	iowrite32(0,          ep->sw + SW_VLAN_CTRL);	/* VLAN function disable */
	iowrite32(1, ep->sw + SW_VLAN_EGRESS_TAG);			/* VLAN keep-format p0-3 */
	iowrite32(1, ep->sw + 0x2a004);
	iowrite32(1, ep->sw + 0x2a008);
	iowrite32(1, ep->sw + 0x2a00c);
	iowrite32(0, ep->sw + 0x23030);			/* CPU_TAG_CTRL=0 (re-armed in hw_program) */
	writel(readl(sysstat) | 1, sysstat);		/* patch done: set 0xB8000044 bit0 */
}

static int rtl9602c_eth_open(struct net_device *ndev)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);
	u32 desnum;
	int ret;

	ret = rtl9602c_eth_alloc_rings(ep);
	if (ret) {
		rtl9602c_eth_free_rings(ep);
		return ret;
	}
	ep->closing = false;
	ep->stall_since = 0;

	/*
	 * ★★★ THE FULL SWITCH BRING-UP IS A ONE-TIME COST, NOT A PER-ifup ONE
	 * (measured 2026-08-27 on the X111W, and it cost 22 s and the GPON link).
	 *
	 * Everything below -- hw_stop, ipsel_cycle, uboot_swcore_bringup,
	 * datapath_tables_init, hw_program -- re-initialises the whole switch
	 * fabric.  This is `ndo_open`, so it ran on EVERY `ifup` of eth0, and a
	 * normal boot opens it twice: once from preinit, once when netifd builds
	 * br-lan.  The board's own boot log shows exactly what the second one
	 * costs:
	 *
	 *   [ 7.036] datapath_tables_init done (tbl_ok=1)   <- first open, fine
	 *   [12..29] Ranging_Time -> O5   x3   OLT assigned ONU-ID 1  x3
	 *            T-CONT 16 <- alloc bound  x3            <- GPON re-ranging
	 *   [29.212] datapath_tables_init done (tbl_ok=1)   <- second open
	 *
	 * Tearing the fabric down under a PON link that has already reached O5
	 * drops it, so the ONU re-ranges -- three full cycles -- and the OLT sees
	 * an ONU that keeps coming and going.  On a live PON that is not merely
	 * slow, it is churn the operator's OLT pays for.
	 *
	 * ⇒ the heavy bring-up runs ONCE.  A later open re-arms the GMAC (rings
	 *   are allocated above, hw_program below) and leaves the fabric and the
	 *   PON link alone -- which is what `ndo_open` is supposed to do.
	 *
	 * ⚠ NOT AN OPTIMISATION OF A CORRECT PATH: the second run was actively
	 *   harmful.  If a future board genuinely needs the fabric rebuilt on a
	 *   re-open, that is a `gmac_reset` mode of its own with a measurement
	 *   behind it, not the default.
	 */
	if (gmac_reset) {
		/*
		 * Stock-faithful cold start (the TX-park fix): halt the
		 * inherited engine, power-cycle the GMAC IP block (the stock
		 * GMAC reset path), then program EVERYTHING in the stock
		 * init order ending on the IO_CMD1->IO_CMD enable edge with
		 * the live-stock operating values. Without the power-cycle
		 * the multi-ring fetch engine never latches its ring state
		 * and parks fatally on the first sparse TX (stock note:
		 * "in old method, mring can't receive packet at first time").
		 */
		rtl9602c_hw_stop(ep);
		rtl9602c_ipsel_cycle();
		rtl9602c_uboot_swcore_bringup(ep);	/* re-establish GMAC<->switch sync the reset tore down (catch-22 breaker) */
		/* Faithful full stock-equivalent datapath init, in stock module order, on the now-
		 * QUIESCENT switch (TX not yet armed) — switch/l2/vlan/port/cpu/trap/
		 * classify/ponmac. Hypothesis: the switch honoring the cpu-tag PSEL
		 * directed-egress to the PON-MAC is an emergent property of the full
		 * ordered/quiescent bring-up that piecemeal flat writes never reproduced.
		 * Runs BEFORE hw_program arms the GMAC. Gated by full_datapath_init (revertible). */
		rtl9602c_datapath_tables_init();
		rtl9602c_hw_program(ep);
		rtl9602c_tx_align(ep);	/* fresh engine: CDO=0 -> rot 0 */
		/*
		 * SAME-BOARD DIFF FIX (stock-WORKING vs mine-BROKEN on Board-C): the only diffs
		 * in the IP-mux/SoC region are SoC-ctrl 0x18000100 bit8 and 0x18000104 bit2
		 * (stock sets them, my OS doesn't) — candidate clock/power enable for the
		 * IP-mux/US-NIC domain (network-engine 0x18001000/098 bits were tested: real
		 * config but no effect). Live-poking these crashed, so set them here at init
		 * (quiescent), RMW just the diff bits, before the PON-NIC bring-up. Gated.
		 */
		if (ipmux_soc) {
			void __iomem *s100 = (void __iomem *)0xb8000100ul;
			void __iomem *s104 = (void __iomem *)0xb8000104ul;
			writel(readl(s100) | BIT(8), s100);	/* 0x18000100 bit8 -> stock */
			writel(readl(s104) | BIT(2), s104);	/* 0x18000104 bit2 -> stock */
		}
		if (ipmux_neteng) {
			/* network-engine / IP-mux page 0x18001000 (KSEG1 0xb8001000). RMW the exact
			 * same-board-diff bits to stock-WORKING values (don't clobber dynamic bits):
			 *   0x18001000: set bit19 (stock 0x10281e6f vs mine 0x10201e6f)
			 *   0x18001098: clear bits19,20; set bits14,18 (stock 0x0004e123 vs mine 0x0018a123) */
			void __iomem *n000 = (void __iomem *)0xb8001000ul;
			void __iomem *n098 = (void __iomem *)0xb8001098ul;
			writel(readl(n000) | BIT(19), n000);
			writel((readl(n098) & ~(BIT(19) | BIT(20))) | BIT(14) | BIT(18), n098);
		}
		/*
		 * Re-establish the FULL PON US/DS-NIC datapath against the freshly-reset GMAC.
		 * The gpon driver's gpon_pbo_init() ran at BOOT (module init), BEFORE this
		 * ifup-time GMAC IP-block reset — so the US-NIC RX engine latched the PRE-reset
		 * GMAC0-TX clock/state and the GMAC0-TX->US-NIC internal direct-TX link is
		 * desynced (US-OMCI TXes but never reaches the US-NIC MAC: RX_OK=ERR=MISS=0).
		 * Re-run the whole PON-NIC bring-up here so it latches the new GMAC — this is
		 * the stock init ORDER (GMAC reset -> THEN PON-NIC/PBO setup). pbo_init is
		 * idempotent (DRAM pool alloc is one-shot guarded).
		 */
		gpon_pbo_init();
		/*
		 * ★★★ DO NOT GATE THIS BLOCK ON A "RAN ONCE" FLAG. It was tried on
		 * 2026-08-27 -- gating it on a per-device "already built" bool, to stop a
		 * later ifup rebuilding the fabric under a live PON link -- and it WEDGED
		 * THE CHIP. The comment above is the reason: the stock order is GMAC reset
		 * THEN PON-NIC/PBO setup, so gating the block leaves gpon_pbo_init() un-run
		 * against a GMAC that was reset anyway. The PON-IP page pool is then not
		 * armed while the datapath is live, and the FIRST FORWARDED FRAME stalls a
		 * bus access that never completes.
		 *
		 * ★ THE SIGNATURE, so it is recognised rather than re-diagnosed: a HARD
		 * HANG at `br-lan: port 1(eth0) entered forwarding state` around 30 s --
		 * no panic, no oops, no console, no network, and NOTHING from the
		 * softlockup or hung-task detectors even though BOTH are compiled in.
		 * That silence is the tell: the CPU is not executing, so no in-kernel
		 * detector can ever report it.
		 *
		 * ★ MEASURED BOTH WAYS, same image otherwise unchanged: with the flag,
		 * silent at ~30 s on every boot; without it, alive at 81 s, the LAN
		 * answers on the first try and GPON walks O1 -> O2 -> O3. The re-ranging
		 * cycles the flag was meant to avoid are a nicety; this was a wedge.
		 */
		goto hw_ready;
	}

	/* Program the ring pointers. The bootloader ORs 0x20000000 into the GMAC DMA
	 * ring base + every TX buffer address (TxFDP/RxFDP |= 0x20000000, desc.addr
	 * |= 0x20000000) — the SoC routes the NIC DMA master to DRAM through this bus
	 * window. RX happened to work with the plain (window-0) address, but TX egress
	 * never did; replicate the bus window on TX (the last unreplicated detail of
	 * the established CPU->LAN TX path). */
	ep_wr(ep, R_TxFDP1, ep->tx_ring_dma | DMA_BUS_WINDOW);
	/* TxCDO is NOT writable on the live inherited engine — do not try.
	 * rtl9602c_tx_align() below offsets the SW producer instead. */
	/* Point the dedicated US-OMCI ring's TX descriptor pointer at its ring
	 * (default HW ring 4 -> R_TxFDP(4) = 0x1340). The submit path derives its kick
	 * from this SAME HW ring (h<4 -> R_IO_CMD bit h; h==4 -> R_IO_CMD1 |= 0x100), so
	 * FDP-arm and doorbell agree. Without a programmed FDP the engine never fetches
	 * the ring. (oring == omci_tx_ring == the HW ring h used in omci_xmit.)
	 *
	 * SKIP entirely when omci_tx_ring==0 (shared-ring-0 OMCI test path): ring 0 is
	 * the LAN ring, already armed above (R_TxFDP1 = ep->tx_ring_dma). Re-pointing
	 * R_TxFDP(0) at ep->otx_ring_dma here would CLOBBER the LAN ring and break
	 * normal LAN traffic. The OMCI frame rides the LAN ring's own arm. */
	if (omci_tx_ring != 0) {
		unsigned int oring = rtl9602c_omci_hwring(omci_tx_ring);

		ep_wr(ep, R_TxFDP(oring), ep->otx_ring_dma | DMA_BUS_WINDOW);
		/* NOTE: a TX ring needs ONLY its TxFDP armed + per-packet kick + a
		 * descriptor with OWN(opts1 bit31) set — there is NO separate TX
		 * "ring-config table" / count / active-mask. Confirmed from the stock
		 * device's ring-init behavior: it writes
		 * TxFDP1..TxFDP5 (rings 0..4) and nothing else gates TX fetch; the
		 * 0x18013380+k*16 loop is the RX multi-ring config (RxFDP2
		 * block), NOT a TX table — writing the OMCI TX ring base there only
		 * scribbled RX-ring-5 config and never affected TX. Removed: the real
		 * "ring 4 never fetched" gate was OWN being published in opts2 instead of
		 * opts1 (the GMAC reads ownership from opts1 bit31 on every TX ring). */
	}
	/* Arm the UNUSED HW TX rings (the TxFDP gaps between the LAN ring (HW0) and
	 * the OMCI ring) at the idle dummy ring. ROOT CAUSE of the LAN wedge: the
	 * GMAC0 multi-ring TX DMA engine, once the OMCI ring kicks the multi-ring
	 * scheduler (IO_CMD1|0x100), walks ALL HW TX rings; a ring left with an
	 * UNPROGRAMMED TxFDP makes it DMA from a stale base and stall the shared TX
	 * path -> LAN CPU datapath wedges (measured http 2/120s under active OMCI TX).
	 * Stock arms all 5 TxFDP rings for exactly this. Dummy descriptors are OWN=0
	 * so the engine fetches nothing. Skip HW ring 0 (LAN) and the OMCI ring. */
	{
		unsigned int k, oring = rtl9602c_omci_hwring(omci_tx_ring);

		for (k = 1; k <= 4; k++) {
			if (k == oring)
				continue;	/* OMCI ring already armed at its own base */
			ep_wr(ep, R_TxFDP(k), ep->dummy_ring_dma | DMA_BUS_WINDOW);
			iowrite16(0, ep->base + R_TxCDO(k));
		}
	}
	ep_wr(ep, R_RxFDP, ep->rx_ring_dma | DMA_BUS_WINDOW);
	/* RX ring0 size + flow-control thresholds (GMAC field packing). */
	desnum = luna_gmac_rxdesnum_pack(RX_RING_SIZE, TH_ON_VAL, TH_OFF_VAL);
	ep_wr(ep, R_RxDesNum, desnum);
	ep_wr(ep, R_RxCDO, luna_gmac_rxcdo_pack(RX_RING_SIZE));
	/* (Reverted: a prior experiment pointed rings 1-5 at ring 0's buffer on the
	 * theory the OMCI was priority-routed to rings 1-5 — but 6 ring engines sharing
	 * ring 0's descriptors corrupts it. The OMCI arrives via the PON-NIC internal MII
	 * straight to GMAC0 GMII-RX -> ring 0; route everything to ring 0 instead, below.) */

	/*
	 * Program the GMAC control regs. TCR/RCR/CONFIG use known-good bring-up
	 * values (RCR=0x0F accepts bcast/mcast/myphys/allphys, CONFIG RFF-2k +
	 * rx-mring-int-split). IO_CMD is the full CMD_CONFIG that enables RX *and*
	 * TX DMA — the bootloader leaves RX off after its polled TFTP, so re-asserting
	 * the inherited IO_CMD (0x400f3330) never started the RX engine. IO_CMD is
	 * written last (it is the enable/kick).
	 *
	 * Apply the active GMAC init the bootloader uses during a net op (its TFTP
	 * does CPU->LAN TX over the GbE), which forwards CPU<->host both ways: the
	 * alternative recipe (0xd15ff130 / RCR 0x0F / TCR 0x0C01 / CONFIG 0x20c10000)
	 * enabled RX but not TX egress. The control words read at the idle bootloader
	 * prompt are NOT usable (the GMAC is torn down there: CPUtagCR reads reset
	 * 0x015c0000); the established active values are TCR 0x0c00, RCR 0x0e,
	 * CONFIG 0x20000000, IO_CMD 0x400f3330, CPUtagCR 0x981aff04. Write order
	 * matters; IO_CMD last (enable).
	 *
	 * NOTE on TCR: TCR=0x0C01 with auto-padding DISABLED (bit0=1, TX pad OFF) can
	 * corrupt the GMAC-inserted cpu-tag on short frames; the established active
	 * value keeps bit0=0 (TX pad ON), so use 0x0C00.
	 */
	ep_wr(ep, R_RCR, 0x0000000E);
	ep_wr(ep, R_TCR, 0x00000C00);
	ep_wr(ep, R_CONFIG, 0x20000000);
	iowrite8(0x0A, ep->base + 0x3B);	/* CMD = RxChkSum|RxJumboSupport (keep working-RX baseline) */
	/* (RX-ring-size bytes 0x1430/0x1432/0x13f6 select a 16-entry ring; our
	 * 64-entry ring is sized by R_RxDesNum/R_RxCDO above — don't clobber.) */
	/* Arm the OMCI cpu-tag (SID 64) here, ONCE, before R_IO_CMD enables the TX
	 * engine — re-arming CPUTAGCR/CPUTAG1CR at runtime (set_omci_sid, post-O5) left
	 * the cpu-tag-insert engine half-armed and the switch CPU-port-3 dropped the
	 * tagged US OMCI (matches stock: CPUTAGCR written once in init, never again).
	 * ★ The cpu-tag ADD engine must be RE-LATCHED with an off-then-on toggle:
	 * write CPUTAGCR=0 FIRST, THEN the operating value — the 0->value edge re-samples
	 * CT_TSIZE/CTPV/CT_APPLO so the GMAC actually PREPENDS the cpu-tag to egressing
	 * frames. Writing the value straight (no clear-first) left the engine half-armed:
	 * it emitted a BARE OMCI frame (port-mirror confirmed: no cpu-tag on the wire)
	 * -> switch can't directed-egress by portmask -> US OMCI never reaches PON P2
	 * -> us_rxsid good=0. CPUTAG1CR = CT1_SID(64<<8) | bit1 = 0x4002. NOTE: the 9607C
	 * register map defines only CT1_SID (0x4000), so an
	 * earlier port dropped the "|2" as "a stray bit stock never sets" — but that
	 * followed the 9607C definition, and the 9602C is a different chip: a LIVE 9602C stock
	 * register read (serial-pasted mmap reader on the stock ONU, 2026-06-11) shows
	 * CPUTAG1CR = 0x4002, i.e. bit1 IS set on this chip. Restoring it (the bare 0x4000
	 * was a 9607C->9602C port regression). SID is fixed (== GPON flow 64), latch before O5. */
	ep_wr(ep, R_CPUTAGCR, 0x00000000u);	/* OFF first — re-latches the cpu-tag ADD engine */
	wmb();					/* the clear must land before the re-arm edge */
	ep_wr(ep, R_CPUTAGCR, 0x901eff04u);	/* ON: re-latches the TX cpu-tag ADD (post-init stock value) */
	ep_wr(ep, R_CPUTAG1CR, CPUTAG1_OMCI_SID(RTL9602C_OMCC_SID) | CPUTAG1_LOW);	/* = 0x4002 (live-stock ref ONU devmem) */
	/* Arm the SWITCH cpu-tag engine (CPU-port tag-aware + insert mode + aux) here,
	 * BEFORE R_IO_CMD enables TX — mainline rtl8365mb arms the cpu-tag engine before
	 * the TX/forwarding path; doing it later (in sw_min_init, post-TX-enable) left
	 * port-3 ingress unable to parse the cpu-tagged US OMCI. Latching the full
	 * cpu-tag engine (GMAC CPUTAGCR/CPUTAG1CR above + switch MAC_CPU_TAG_CTRL/TAG_AWARE
	 * + aux here) before the TX engine comes up mirrors mainline ordering.
	 * MAC_CPU_TAG_CTRL = 0x300 (TAG_AWARE bit9 + TRAP_TARGET_INSERT_EN bit8): the
	 * switch PARSES the cpu-tag on CPU-port ingress so a CPU TX frame is steered by
	 * the tag (stream-id for the OMCC US OMCI) instead of L2-DA-flooded. Aux regs
	 * (live-stock, and NAMED from the chip's own chipdef 2026-08-29):
	 * FC_P_LO_TH=0x00400034, FC_P_FCOFF_HI_TH=0x00f000ea,
	 * FC_P_FCOFF_LO_TH=0x00400034. */
	if (ep->sw) {
		/* OFF->ON edge re-latches the SWITCH CPU-port cpu-tag PARSER, mirroring the
		 * GMAC CPUTAGCR re-latch above. Behavioral note: TAG_AWARE
		 * re-arms the CPU-port HSB cpu-tag parser; a straight write can leave the parser
		 * half-armed so it ignores the descriptor's PSEL/PON_SID and the cpu-tagged US
		 * OMCI falls through to L2 flood instead of PSEL direct-TX to the PON/US-NIC.
		 * Clear-then-set forces the parser to re-sample TAG_AWARE/TRAP_INSERT. */
		iowrite32(0, ep->sw + SW_MAC_CPU_TAG_CTRL);
		wmb();					/* clear must land before the re-arm edge */
		iowrite32(sw_tagaware, ep->sw + SW_MAC_CPU_TAG_CTRL);
		iowrite32(0x00400034, ep->sw + SW_FC_P_LO_TH);
		iowrite32(0x00f000ea, ep->sw + SW_FC_P_FCOFF_HI_TH);
		iowrite32(0x00400034, ep->sw + SW_FC_P_FCOFF_LO_TH);
	}
	/* GMAC config regs that a LIVE stock ONU at O5 SETS but my driver left at 0 /
	 * masked wrong — found by full block diff. The earlier masking of MSR(0x58) down
	 * to 0x10638000 was the bug: it CLEARED bits 31/30 that stock keeps set
	 * (0xf0638000 = internal DS-NIC<->GMAC RX link/force bits). With those cleared and
	 * 0xd0 (the per-ring RX-DMA enable mask, stock=0x3f = rings 0-5) left 0, the GMAC
	 * RX engine never DMA'd the DS-NIC-drained OMCI to any ring (filled=0 despite
	 * PKT_OK_CNT_DS climbing). Restore stock values. */
	ep_wr(ep, 0x24, 0x010c0000);		/* stock O5 */
	ep_wr(ep, R_IMR0, IMR0_TX_BITS);	/* per-ring TX-completion IRQ mask, rings 0-5 (stock 0x3f) */
	iowrite16(IMR_RX_BITS, ep->base + R_IMR);	/* RX IRQ mask: RX_OK + RX-err + RDU (stock 0xf835) */
	ep_wr(ep, 0x58, (ep_rd(ep, 0x58) & 0x00ffffff) |
			((msr_top & 0xffu) << 24));	/* MSR top byte: param (0xf0 kills sparse TX, see msr_top) */
	iowrite32(0xffffffff, ep->base + 0x08);	/* MAR0: accept-all-multicast */
	iowrite32(0xffffffff, ep->base + 0x0C);	/* MAR4 */
	/* IO_CMD1 = the exact stock start value (stock writes IO_CMD1 = 0x323f0001).
	 * Decoded against the chip's IO_CMD1 field definitions
	 * (iocmd1_reg construction): 0x323f0001 =
	 *   CMD1_CONFIG(0x30000000, "apollo desc-format") | RX_NOT_ONLY_RING1(1<<25)
	 *   | RX_MULTIRING_BITMAP(0x3f)<<16 | txq1_h(1<<0).
	 * IMPORTANT: bits[21:16] are the RX multiring bitmap (which RX rings are
	 * active), NOT a TX-fetch enable — TX rings need no IO_CMD1 enable bit. A TX
	 * ring fetches purely from its armed TxFDP + a descriptor that publishes OWN
	 * in opts1 bit31 + the per-packet poll (IO_CMD TX_POLL bits 0..3 for HW rings
	 * 0..3, IO_CMD1 TX_POLL5 bit8 for HW ring 4). We write the stock value verbatim
	 * because it is the known-LAN-safe config; it is unrelated to OMCI ring-4
	 * activation. */
	ep_wr(ep, R_IO_CMD1, IOCMD1_UBOOT);	/* legacy: re-assert the inherited config */
	ep_wr(ep, R_IO_CMD, IOCMD_UBOOT);	/* full CMD_CONFIG, RX+TX DMA enable (last) */
	rtl9602c_tx_align(ep);	/* SW producer -> live engine position (CDO) */

hw_ready:
	rtl9602c_sw_min_init(ep);	/* flood ingress to the CPU port */

	napi_enable(&ep->napi);
	/* Clear any IRQ status latched during bring-up before unmasking the line. */
	iowrite16(ioread16(ep->base + R_ISR), ep->base + R_ISR);
	ep_wr(ep, R_ISR1, ep_rd(ep, R_ISR1));
	if (ep->irq > 0) {
		ret = request_irq(ep->irq, rtl9602c_eth_isr, IRQF_SHARED,
				  ndev->name, ep);
		if (ret) {
			netdev_warn(ndev, "request_irq(%d) failed (%d); pure-poll fallback\n",
				    ep->irq, ret);
			ep->irq = -1;
		}
	}
	/*
	 * IRQ-driven: the ISR schedules NAPI; the timer is just a slow 100ms
	 * TX-unpark backstop. No IRQ (input 26 unproven / request_irq failed):
	 * fall back to the legacy 2ms full-drain pure-poll.
	 */
	timer_setup(&ep->poll_timer,
		    (ep->irq > 0) ? rtl9602c_eth_rekick_timer : rtl9602c_eth_poll, 0);
	mod_timer(&ep->poll_timer,
		  jiffies + ((ep->irq > 0) ? REKICK_INTERVAL : POLL_INTERVAL));

	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int rtl9602c_eth_stop(struct net_device *ndev)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);

	/* Fence the stall recovery FIRST: a mid-flight recover_work owns
	 * napi/timer state (it disables and re-enables them); cancelling before
	 * we touch either avoids a double napi_disable deadlock. After the
	 * cancel, the rekick's closing guard prevents any re-schedule. */
	ep->closing = true;
	cancel_work_sync(&ep->recover_work);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	/* Mask the GMAC IRQs before teardown so a late TX/RX completion cannot
	 * touch freed rings; free_irq must precede napi_disable so a racing ISR
	 * cannot napi_schedule a disabled context. */
	iowrite16(ioread16(ep->base + R_IMR) & ~IMR_RX_BITS, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, ep_rd(ep, R_IMR0) & ~IMR0_TX_BITS);
	if (ep->irq > 0)
		free_irq(ep->irq, ep);
	napi_disable(&ep->napi);
	timer_delete_sync(&ep->poll_timer);
	ep_wr(ep, R_IO_CMD, 0);		/* stop DMA */
	/* Barrier vs the GPON driver's OMCI injects (they run off a foreign
	 * timer): any inject already inside tx_lock finishes before we free the
	 * rings; later ones see closing/NULL under the lock and bail. */
	{
		unsigned long flags;

		spin_lock_irqsave(&ep->tx_lock, flags);
		spin_unlock_irqrestore(&ep->tx_lock, flags);
	}
	rtl9602c_eth_free_rings(ep);
	return 0;
}

/* CPU-directed TX (GMAC tx_info): opts2 cputag[31] | tx_portmask[26:16];
 * opts3 keep[23] | dislrn[21] | l34_keep[17]. Directs CPU-originated frames out
 * the physical LAN ports (keep/l34_keep stop the switch filtering them). */
#define TXD_CPUTAG	BIT(31)
#define TXD_PORTMASK(m)	(((m) & 0x7ff) << 16)
#define TXD_KEEP	BIT(23)
#define TXD_DISLRN	BIT(21)
#define TXD_L34_KEEP	BIT(17)
/* GMAC tx_portmask: user/LAN ports are 0..6; CPU ports are 7,9,10. 0x2f =
 * ports 0,1,2,3,5 = an "all user ports except CPU" broadcast-fallback mask.
 * Egressing to all user ports is harmless for ports without a linked PHY. */
#define TX_LAN_PORTS	0x2f
/* Software DSA-style cpu-tag (mainline net/dsa/tag_rtl8_4.c "rtl8_4" 0x8899
 * format, 8 bytes). The GMAC's HARDWARE cpu-tag insertion emits a ZERO egress
 * portmask on this 9602C silicon (observed: tag word[3]=0 regardless of
 * opts2.tx_portmask), so we BUILD the tag in software and send a plain frame
 * (opts2.cputag=0). The switch's TAG_AWARE parser then reads OUR portmask.
 * Layout: word0=0x8899 word1=proto(0x04)|reason(0) word2=LEARN_DIS
 * word3=forwarding port mask (RX field, GENMASK 10:0). */
/* RTL8_4_TAG_LEN moved earlier (near the OMCI defines) so the OMCI ring-0 path can use it. */
#define SW_TAG_LAN_MASK	0x7	/* forwarding mask: LAN ports 0,1,2 (CPU port = 3) */

static netdev_tx_t rtl9602c_eth_xmit(struct sk_buff *skb,
				     struct net_device *ndev)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);
	unsigned int i;
	unsigned int len = skb->len;
	unsigned long flags;
	dma_addr_t da;
	u32 opts1;

	/* tx_lock taken first so the ring-full test precedes any skb mutation
	 * (BUSY -> stack retries an UNtouched skb, no double cpu-tag) AND so
	 * tx_head/tx_ring/kick are atomic vs the softirq OMCI inject. The skb
	 * reallocs (padto/cow) and dma_map below all use GFP_ATOMIC / are IRQ-safe,
	 * so holding the irqsave lock across them is sound. */
	spin_lock_irqsave(&ep->tx_lock, flags);
	if ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1 - OMCI_RESV) {
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
	ep->tx_ring[i].opts2 = 0;
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
		rtl9602c_eth_tx_fetch(ep);	/* stock GO handshake on every submit */
	ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* kick ring 0 */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	return NETDEV_TX_OK;
}

/* Honour promiscuous/all-multi (the bridge enslaving eth0 requests promisc).
 * Without AcceptAllPhys the GMAC drops unicast frames whose DA != our station
 * MAC — i.e. exactly the LAN-client frames a router/bridge must receive and
 * forward. RCR bit0 = AcceptAllPhys. */
static void rtl9602c_eth_set_rx_mode(struct net_device *ndev)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);

	luna_eth_set_promisc(ep->base,
				 !!(ndev->flags & (IFF_PROMISC | IFF_ALLMULTI)));
}

#ifdef CONFIG_GPON_FLOW_OFFLOAD
/* This family's five-op table for the COMMON TC lifecycle.  Included here, not
 * at the top, because it needs `struct rtl9602c_eth` complete and the WAN ops
 * table defined -- and the LAN ops table below needs ITS ndo_setup_tc. */
#include "rtl9602c_l34_tc.c"
#endif

static const struct net_device_ops rtl9602c_eth_netdev_ops = {
#ifdef CONFIG_GPON_FLOW_OFFLOAD
	.ndo_setup_tc		= rtl9602c_l34_setup_tc,
#endif
	.ndo_open		= rtl9602c_eth_open,
	.ndo_stop		= rtl9602c_eth_stop,
	.ndo_start_xmit		= rtl9602c_eth_xmit,
	.ndo_set_rx_mode	= rtl9602c_eth_set_rx_mode,
	.ndo_set_mac_address	= rtl9602c_eth_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
};

/* Live datapath diagnostic. Read /proc/rtl9602c_diag to dump the GMAC DMA/link
 * state, RX-ring ownership, and switch forwarding regs — to localise where a
 * frame is lost on the CPU<->LAN path. */
/*
 * Self-test: writing /proc/rtl9602c_omci_test injects 5 dummy 48-byte baseline
 * OMCI frames straight through the US OMCC TX path (rtl9602c_eth_omci_xmit),
 * decoupled from the OLT's downstream OMCI. Run at O5 (grants flowing, idle16
 * climbing): if ustx (0x1b0329bc) then climbs, the descriptor steering reaches
 * the OMCC US engine; if it stays 0, the GMAC0-TX -> US-NIC steering is still the
 * gap. This breaks the "OLT won't send OMCI -> can't test US TX" deadlock.
 */
static ssize_t rtl9602c_omci_test_write(struct file *f, const char __user *ubuf,
					size_t cnt, loff_t *off)
{
	struct rtl9602c_eth *ep = g_ep;
	unsigned int k;
	u8 msg[48];

	if (!ep)
		return -ENODEV;
	/* Arm the GMAC OMCI state (CPUTAGCR=0x901eff04, the cpu-tag trap + routing) as
	 * the OLT-driven path would via gpon_install_omcc -> set_omci_sid. Without the
	 * OLT the GMAC sits at the non-OMCI 0x981aff04, so the self-test must arm it to
	 * test the cpu-tag insertion representatively. Idempotent. */
	rtl9602c_eth_set_omci_sid(RTL9602C_OMCC_SID);
	for (k = 0; k < 5; k++) {
		memset(msg, 0, sizeof(msg));
		msg[0] = 0x00; msg[1] = 0x42;	/* TID */
		msg[2] = 0x2f;			/* MIB-Reset response (AK) */
		msg[3] = 0x0a;			/* DevID baseline */
		msg[4] = 0x00; msg[5] = 0x02;	/* ME ONU-data */
		rtl9602c_omci_finalize(msg);	/* trailer + MIC */
		rtl9602c_eth_omci_xmit(ep, msg, sizeof(msg));
	}
	return cnt;
}

static const struct proc_ops rtl9602c_omci_test_pops = {
	.proc_write = rtl9602c_omci_test_write,
};

static int rtl9602c_diag_show(struct seq_file *m, void *v)
{
	struct rtl9602c_eth *ep = g_ep;
	unsigned int i, own = 0, hwfilled = 0;

	if (!ep) { seq_puts(m, "no device\n"); return 0; }
	if (ep->in_recovery) {	/* GMAC may be power-gated: MMIO would bus-abort */
		seq_printf(m, "GMAC recovery in progress (recovers=%u)\n",
			   ep->dbg_tx_recover);
		return 0;
	}

	for (i = 0; i < RX_RING_SIZE; i++) {
		if (ep->rx_ring[i].opts1 & D_OWN)
			own++;
		else
			hwfilled++;
	}
	seq_printf(m, "poll=%u filled=%u good=%u err=%u rx_head=%u\n",
		   ep->dbg_poll, ep->dbg_filled, ep->dbg_good, ep->dbg_err,
		   ep->rx_head);
	seq_printf(m, "last RX frame (pre-pull, len=%u): %*ph\n",
		   ep->dbg_rxlen, (int)sizeof(ep->dbg_rxbuf), ep->dbg_rxbuf);
	seq_printf(m, "omci: trap=%u rx=%u lastlen=%u msg=%*ph\n",
		   ep->omci_trap_on, ep->dbg_omci_rx, ep->dbg_omci_rxlen,
		   (int)sizeof(ep->dbg_omci_rxbuf), ep->dbg_omci_rxbuf);
	seq_printf(m, "omci_tx: resp=%u drop=%u unhandled=%u mds=%u sn=%*ph\n",
		   ep->dbg_omci_tx, ep->dbg_omci_tx_drop, ep->dbg_omci_unhandled,
		   ep->omci_mds, 8, ep->omci_sn);
	/* the shared core's own DS discard counters.  ★ Two numbers, not one: a
	 * runt is a framing / GEM-reassembly fault upstream of OMCI, a bad MIC
	 * is corruption on a well-framed PDU, and one figure for both would make
	 * a broken reassembler read as a noisy fibre. */
	seq_printf(m, "omci_rx_drop: bad_mic=%u runt=%u\n",
		   luna_onu.rx_bad_mic, luna_onu.rx_runt);
	{
		unsigned int oring;
		u32 dmask;
		/* Same flowcore derivation as the xmit kick, so the diag can
		 * never disagree with the doorbell actually used. */
		bool dk1 = rtl9602c_omci_doorbell(omci_tx_ring, omci_doorbell_bit,
						  &oring, &dmask);

		if (omci_tx_ring == 0) {
			/* Shared-ring-0 OMCI test path: the OMCI frame rides the LAN
			 * ring 0. Show the LAN ring head/dirty and the OWN bit (opts1
			 * on this ring) of the last OMCI slot we submitted. OWN cleared
			 * => HW IS fetching/consuming the OMCI descriptor on ring 0
			 * (steering is what's under test); OWN still set => the LAN
			 * engine has not consumed it. */
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
			/* Dedicated-ring path. OWN-bit (opts1 BIT31 — the HW-read ownership
			 * word, same as the LAN ring) of the last-submitted (head-1) and the
			 * next-to-reclaim (dirty) ring descriptors. After a HW test this
			 * disambiguates: OWN cleared => the HW IS fetching this HW ring (issue
			 * is reclaim/steering); OWN still set => the HW is NOT fetching
			 * (FDP-arm/doorbell/enable mismatch). */
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
		   ioread32(ep->base + 0x58));
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
		   (u32)ep->rx_ring_dma);
	/* Full GMAC config diff vs LIVE stock O5 (stock golden values in comments).
	 * Hunting the reg that brings up the internal DS-NIC->GMAC RX link (MSR 0x58
	 * stock=0xf0638000 vs mine=0x10638000). */
	seq_printf(m, "GMACcfg 10=%08x[f:04a80457] 20=%08x[034c0003] 24=%08x[010c0000] 38=%08x[0a] 3c=%08x[f8350240]\n",
		   ep_rd(ep, 0x10), ep_rd(ep, 0x20), ep_rd(ep, 0x24),
		   ep_rd(ep, 0x38), ep_rd(ep, 0x3c));
	seq_printf(m, "GMACcfg 44=%08x[0f] 58=%08x[f0638000] 5c=%08x[04000000] d0=%08x[3f] d8=%08x[11110000]\n",
		   ep_rd(ep, 0x44), ep_rd(ep, 0x58), ep_rd(ep, 0x5c),
		   ep_rd(ep, 0xd0), ep_rd(ep, 0xd8));
	/* GMAC0 MAC-level MIB counters (16-bit, BE-packed two per 32-bit word):
	 * 0x10=[TXOK:RXOK] 0x14=[TXERR:RXERR] 0x18=[MISS:..]. DECISIVE for the OMCI MII
	 * delivery: at O5 with no LAN traffic, if rxok climbs the OMCI frame reaches the
	 * GMAC0 MAC; if miss climbs it reached the MAC but couldn't DMA (descriptor gap);
	 * if BOTH stay flat the DS-NIC->GMAC0 internal MII never delivered the frame. */
	seq_printf(m, "GMAC_MIB txok=%u rxok=%u txerr=%u rxerr=%u miss=%u\n",
		   ep_rd(ep, 0x10) >> 16, ep_rd(ep, 0x10) & 0xffff,
		   ep_rd(ep, 0x14) >> 16, ep_rd(ep, 0x14) & 0xffff,
		   ep_rd(ep, 0x18) >> 16);
	/* NIC interrupt status: per-ring RDU (Receive-Descriptor-Unavailable) bits show
	 * a frame ARRIVED at the NIC on a ring with no posted descriptor. ISR(0x3c):
	 * RDU=bit5(ring0) RDU2=bit11(r1) RDU3=bit12(r2) RDU4=bit13(r3) RDU5=bit14(r4)
	 * RDU6=bit15(r5). If RDU2-6 set => OMCI is reaching the NIC on rings 1-5 that I
	 * don't set up (frame dropped). RxCDO per ring shows HW fetch progress. */
	seq_printf(m, "NIC ISR(0x3c)=%08x ISR1(0xd8)=%08x  perRingRxCDO r0=%04x r1=%04x r2=%04x r3=%04x r4=%04x r5=%04x\n",
		   ep_rd(ep, 0x3c), ep_rd(ep, 0xd8),
		   ep_rd(ep, 0x13f4) & 0xffff, ep_rd(ep, 0x1394) & 0xffff,
		   ep_rd(ep, 0x13a4) & 0xffff, ep_rd(ep, 0x13b4) & 0xffff,
		   ep_rd(ep, 0x13c4) & 0xffff, ep_rd(ep, 0x13d4) & 0xffff);
	if (ep->sw) {
		seq_printf(m, "SW permit(1c088)=%08x flood bc/mc/uc=%08x/%08x/%08x\n",
			   ioread32(ep->sw + ep->swm->src_permit),
			   ioread32(ep->sw + ep->swm->bc_flood),
			   ioread32(ep->sw + ep->swm->unkn_mc_flood),
			   ioread32(ep->sw + ep->swm->unkn_uc_flood));
		seq_printf(m, "SW vlan_ctrl(13008)=%08x cputag_ctrl(23030)=%08x\n",
			   ioread32(ep->sw + SW_VLAN_CTRL),
			   ioread32(ep->sw + SW_MAC_CPU_TAG_CTRL));
		seq_printf(m, "SW p0_sts(198)=%08x p1_sts(1b8)=%08x p2_sts(1d8)=%08x cpu_sts(1f8)=%08x\n",
			   ioread32(ep->sw + 0x198), ioread32(ep->sw + 0x1b8),
			   ioread32(ep->sw + 0x1d8), ioread32(ep->sw + 0x1f8));
		/* Per-port MIB packet counters (TX_MIB@0x32000+port*0x80, RX_MIB@0x32400+
		 * port*0x80; dump first 3 counters of each block). Localises the DS drain:
		 * p2(PON) rx>0 => PON-IP frames reach the switch; p3(CPU) tx>0 => switch
		 * forwards them to the CPU. */
		/* LAN ports p0(FE)/p1(GE): if the injected US OMCI floods here, the cpu-tag
		 * steering failed and the frame went to the L2 switch instead of the US-NIC. */
		seq_printf(m, "MIB p0(LAN) tx=%08x | p1(LAN) tx=%08x\n",
			   ioread32(ep->sw + SW_STAT_PORT_TX_MIB), ioread32(ep->sw + 0x32080));
		seq_printf(m, "MIB p2(PON) tx=%08x %08x %08x | rx=%08x %08x %08x\n",
			   ioread32(ep->sw + 0x32100), ioread32(ep->sw + 0x32104),
			   ioread32(ep->sw + 0x32108), ioread32(ep->sw + 0x32500),
			   ioread32(ep->sw + 0x32504), ioread32(ep->sw + 0x32508));
		seq_printf(m, "MIB p3(CPU) tx=%08x %08x %08x | rx=%08x %08x %08x\n",
			   ioread32(ep->sw + 0x32180), ioread32(ep->sw + 0x32184),
			   ioread32(ep->sw + 0x32188), ioread32(ep->sw + 0x32600),
			   ioread32(ep->sw + 0x32604), ioread32(ep->sw + 0x32608));
	}
	return 0;
}

static int rtl9602c_eth_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct rtl9602c_eth *ep;
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
	INIT_WORK(&ep->recover_work, rtl9602c_eth_recover_work);

	ep->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ep->base))
		return PTR_ERR(ep->base);

	/* Switch core (best-effort; minimal L2 flood enabled at open). */
	ep->swm = &rtl9602c_sw_map;
	ep->sw = devm_ioremap(dev, SWCORE_PHYS, ep->swm->swcore_size);
	/* network-engine TX-fetch GO register page (phys 0x18001000, a SEPARATE page
	 * below the GMAC). The stock device sets bit31 of +0x38 then polls it clear on
	 * EVERY submit to command the self-polling TX DMA to fetch the freshly-published
	 * descriptor; without it the sparse OMCI inject's descriptor is never fetched and
	 * the engine parks (txok frozen, dirty stuck). */
	ep->txgo = devm_ioremap(dev, 0x18001000, 0x1000);

	/* Snapshot the bootloader's live GMAC0 control config to re-assert at open. */
	ep->ub_tcr = ep_rd(ep, R_TCR);
	ep->ub_rcr = ep_rd(ep, R_RCR);
	ep->ub_config = ep_rd(ep, R_CONFIG);
	ep->ub_cputagcr = ep_rd(ep, R_CPUTAGCR);
	ep->ub_cputag1cr = ep_rd(ep, R_CPUTAG1CR);
	ep->ub_iocmd = ep_rd(ep, R_IO_CMD);
	ep->ub_iocmd1 = ep_rd(ep, R_IO_CMD1);

	/* ★ THE BRING-UP DEFAULT IS REFUSED, not merely validated.  This test used
	 * to be `is_valid_ether_addr()` alone, and the family's bring-up default
	 * PASSES it -- so this board shipped 00:e0:4c:86:70:01, the same address
	 * its RTL9603CVD sibling holds, on a segment carrying three ONUs.  MEASURED
	 * 2026-08-28 from the lab host's ARP table.  The predicate is the family's
	 * (luna_eth_regs.h): keeping a second copy here is what let the sibling's
	 * repair miss this driver in the first place. */
	if (mac_param && luna_mac_from_param(mac_param, mac)) {
		/* Announced, because a MAC that arrived from OUTSIDE the device
		 * must be auditable in the log: it is the one rung where a wrong
		 * declaration cannot be told from a right one by reading the
		 * board. */
		eth_hw_addr_set(ndev, mac);
		dev_info(dev, "MAC %pM taken from the `mac=` boot parameter\n", mac);
		goto mac_done;
	}
	rtl9602c_eth_get_hwaddr(ep, mac);
	if (is_valid_ether_addr(mac) && !luna_mac_is_bringup_default(mac)) {
		eth_hw_addr_set(ndev, mac);
	} else {
		eth_hw_addr_random(ndev);
		dev_warn(dev,
			 "MAC engine holds %pM: %s. Using a random locally-administered address %pM instead -- this board's real MAC lives in the vendor config partition and is not read yet\n",
			 mac,
			 is_valid_ether_addr(mac)
			 ? "the SILICON BRING-UP DEFAULT, identical on every board of this family, which would collide on a shared segment"
			 : "not a valid unicast address",
			 ndev->dev_addr);
	}
mac_done:

	/* Bring up the switch L3/L4 NAT engine (gated by the hw_nat param). This
	 * writes switch-core registers, so it is validated for datapath safety
	 * before any per-flow programming is added. */
	if (hw_nat) {
		if (rtl9602c_l34_init(&ep->l34, ep->sw)) {
			dev_warn(dev, "L34 hw-nat init failed; software forwarding\n");
		} else {
			dev_info(dev, "L34 hw-nat engine initialised\n");
			rtl9602c_l34_proc_init(&ep->l34);	/* bring-up harness */
#ifdef CONFIG_GPON_FLOW_OFFLOAD
			/* The COMMON lifecycle owns the cookie map and the
			 * entry; this driver supplies five ops.  ★ Its install
			 * REFUSES today -- the NETIF/NEXTHOP tables are not
			 * provisioned, so a flow would blackhole while reading
			 * back healthy.  Registering anyway is what proves the
			 * core's contract is reachable from this family. */
			ep->fo = gpon_flow_offload_new(&rtl9602c_l34_flow_ops, ep);
			if (!ep->fo) {
				dev_warn(dev, "L34: the common TC lifecycle could not be created; software forwarding\n");
			} else if (devm_add_action_or_reset(dev, rtl9602c_l34_fo_release,
							   ep)) {
				/* ★ THE HANDLE IS OWNED BY THE DEVICE, not by a
				 * .remove this driver does not have.  It is
				 * module_platform_driver() with NO .remove at
				 * all -- a pre-existing gap, and not one this
				 * change is entitled to close by inventing a
				 * teardown for the whole driver.  What it IS
				 * responsible for is the allocation it just
				 * made, so devm owns it: the flows are pulled
				 * out of the hardware and the handle freed
				 * whenever the device goes away, including on
                                 * a failed probe below this point. */
				dev_warn(dev, "L34: could not bind the TC lifecycle to the device; software forwarding\n");
				ep->fo = NULL;
			}
#endif
		}
	}

	ndev->netdev_ops = &rtl9602c_eth_netdev_ops;
	/* Permit a LIVE MAC change (no iface down/up): the per-board MAC is applied
	 * by the gpon_provision boot script after probe. eth0 is the shared GMAC that
	 * also carries the GPON upstream OMCI TX; a down/up to change the MAC tears
	 * down the GMAC rings + the once-at-probe GPON TX setup and permanently kills
	 * the US-OMCI path (OLT Deactivate/"Laser out" churn). A live set only
	 * reprograms the RX-filter IDR via ndo_set_mac_address, leaving TX intact. */
	ndev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
	netif_carrier_off(ndev);

	/* INTC input 26 (GMAC0). <=0 -> no DT mapping: open() runs pure-poll. */
	ep->irq = platform_get_irq(pdev, 0);
	if (ep->irq < 0)
		ep->irq = -1;
	netif_napi_add(ndev, &ep->napi, rtl9602c_eth_napi_poll);

	ret = devm_register_netdev(dev, ndev);
	if (ret)
		return ret;

	dev_info(dev, "RTL9602C NIC at %pR, MAC %pM (inherited IO_CMD %08x)\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0),
		 ndev->dev_addr, ep->ub_iocmd);

	/* gpon0: the WAN data-GEM netdev (clean-room nas0-equivalent). Shares ep's RX/TX
	 * rings + NAPI (owned by eth0); RX is demux'd by ingress port (PON port 2 -> gpon0)
	 * and US frames steer to GPON_DATA_FLOW. Its MAC is the WAN identity = board MAC +
	 * model offset (stock nas0_0 = base+3); see rtl9602c_wan_mac. */
	{
		struct net_device *wan = devm_alloc_etherdev(dev, sizeof(struct rtl9602c_eth *));
		u8 wmac[ETH_ALEN];

		if (wan) {
			*(struct rtl9602c_eth **)netdev_priv(wan) = ep;
			SET_NETDEV_DEV(wan, dev);
			strscpy(wan->name, "gpon0", IFNAMSIZ);
			wan->netdev_ops = &rtl9602c_eth_wan_ops;
			/* Initial WAN MAC = board MAC + offset; re-derived at open + on eth0 MAC
			 * changes once rtk_factory provisions the real board MAC onto eth0. */
			rtl9602c_wan_mac(wmac, ndev->dev_addr);
			eth_hw_addr_set(wan, wmac);
			netif_carrier_off(wan);
			if (devm_register_netdev(dev, wan) == 0)
				ep->wan_ndev = wan;
			else
				dev_warn(dev, "gpon0 (WAN) register failed; WAN datapath disabled\n");
		}
	}

	g_ep = ep;
	ep->omci_mds = (u8)omci_mds_seed;	/* poison seed: fail the OLT's ME2 audit so a warm
						 * re-admit re-provisions (we hold no persistent MIB) */
	{
		/* The identity comes from the PLOAM layer that owns it, not from
		 * a second copy here -- see gpon_onu_sn(). */
		u8 sn[8];

		gpon_onu_sn(sn);
		omci_onu_init(&luna_onu, sn, (u8)omci_mds_seed);
	}
	proc_create_single("rtl9602c_diag", 0444, NULL, rtl9602c_diag_show);
	proc_create("rtl9602c_omci_test", 0200, NULL, &rtl9602c_omci_test_pops);
	return 0;
}

static const struct of_device_id rtl9602c_eth_of_match[] = {
	{ .compatible = "realtek,rtl9602c-nic" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl9602c_eth_of_match);

static struct platform_driver rtl9602c_eth_driver = {
	.probe	= rtl9602c_eth_probe,
	.driver	= {
		.name		= "rtl9602c-eth",
		.of_match_table	= rtl9602c_eth_of_match,
	},
};
module_platform_driver(rtl9602c_eth_driver);

MODULE_DESCRIPTION("Realtek RTL9602C Luna Ethernet driver");
MODULE_LICENSE("GPL");
