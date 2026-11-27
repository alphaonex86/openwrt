// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP -- the RTL9602C row of the Luna Ethernet shell (luna_eth.c), RTL9601D too (same switch map): the
 * switch-core bring-up, GPHY access and patch, VLAN table, CPU-tag arming, port gate, core-recovery blocks and
 * the L34 NAPT engine. -- dev/MEASURED-luna_eth.c.md sec 1. */

#include "luna_eth.h"
#include "luna_gpon_regs.h"	/* the family's luna_sw_io(): a struct hwio over the SWCORE base */
#include "regtable.h"		/* flowcore: gpon_ind_poll() -- the ONE bounded busy-wait */
/* ★★★ ONE VID PER LAN PORT, so the CPU can tell the sockets apart.
 * Stock does exactly this (tier 2, its own kernel: it runs
 * 'brctl addif br0 eth0.%d' and a 'vconfig rem' loop over
 * /sys/class/net | grep eth0.), which is why a stock
 * X111W shows eth0.2/eth0.3 and ours shows one bare eth0: we give EVERY port
 * SW_DEFAULT_VID, so the per-socket information is destroyed in the switch
 * before the CPU sees it, and no netdev layer can get it back.
 *
 * ⚠ DEFAULT OFF, AND THAT IS NOT TIMIDITY. Turning it on moves LAN traffic
 * onto TAGGED frames at the CPU port, so bare eth0 stops receiving and
 * the board is unreachable over LAN until eth0.<vid> netdevs exist and
 * are
 * bridged. The image must carry BOTH halves or it locks itself out; the knob
 * is what lets the two land in separate, testable steps.
 */
static unsigned int port_vlans;
module_param(port_vlans, uint, 0644);
MODULE_PARM_DESC(port_vlans,
		 "one VID per LAN port so the CPU can tell sockets apart (default 0 = every port on SW_DEFAULT_VID, as before). 1 REQUIRES eth0.<vid> netdevs in the image or LAN goes dark");

/* The per-port VID base. Stock's own numbering is FOUND, never chosen -- its
 * X111W shows eth0.2/eth0.3 -- so this base only has to make the VIDs
 * DISTINCT and not collide with SW_DEFAULT_VID; which VID reaches which
 * PRINTED socket is a cable-move measurement (lan_map.py --ask), exactly as
 * this project's panel-map rule requires. */
#define SW_PORT_VID_BASE	2u

/* SW_MAC_CPU_TAG_CTRL (0x23030) = 0x300 (TAG_AWARE bit9 | ...
 * dev/MEASURED-luna_eth.c.md sec 9. */
static unsigned int sw_tagaware = 0x300;
module_param(sw_tagaware, uint, 0644);
MODULE_PARM_DESC(sw_tagaware, "SW MAC_CPU_TAG_CTRL at open (0x300=cpu-tag parse for US-OMCI, 0=plain-LAN-only)");

/* struct rx_desc / struct tx_desc are the FAMILY's -- ...
 * dev/MEASURED-luna_eth.c.md sec 14. */
#define SW_FORCE_P_ABLTY(ep, p)	((ep)->swm->force_ablty + ((p) << 2))
#define SW_ABLTY_FORCE_MODE(ep, p)	((ep)->swm->ablty_force + ((p) << 2))

/* The indirect PHY window has ONE owner on this shell too: ...
 * dev/MEASURED-luna_eth.c.md sec 15. */
static DEFINE_MUTEX(luna_chip_gphy_lock);
/* P_MISC, the per-port misc word (bit2 = RX_SPC: accept ...
 * dev/MEASURED-luna_eth.c.md sec 17. */
#define SW_SYS_LRN_LIMITNO	0x17018	/* system MAC-learn limit [10:0]; 0 = no learning */
#define SW_DLF_ACT_TRAP2CPU	2
/* Forced ability value: 1000M (speed[1:0]=2) + full duplex (b2) + link-up (b4) */
#define SW_ABLTY_1G_FD_UP	(0x2 | BIT(2) | BIT(4))
/* MAC_CPU_TAG_CTRL: TAG_AWARE[9] makes the switch parse the ...
 * dev/MEASURED-luna_eth.c.md sec 18. */
#define SW_WFQ_CTRL		0x2d800
#define   SW_WFQ_MODE_WRR	BIT(17)	/* queues served by packet count; clear = by byte weight */
#define SW_FC_PON_GLB_HI_TH	0x2311C	/* [25:16] on, [9:0] off, in pages of the shared pool */
#define SW_FC_PON_GLB_LO_TH	0x23120
#define SW_FC_P_LO_TH		0x230F0
#define SW_FC_P_FCOFF_HI_TH	0x230F4
#define SW_FC_P_FCOFF_LO_TH	0x230F8
#define SW_TAG_AWARE		BIT(9)
#define SW_TRAP_TAG_INSERT_EN	BIT(8)
/* VLAN filtering: VLAN_CTRL @ 0x13008 bit0 = VLAN_FILTERING; VLAN_INGRESS @
 * 0x13004 = per-port ingress filter, and that one is the HEADER's
 * (luna_eth_regs.h) since 2026-09-04.  SW_VLAN_FILTERING below has no twin in the
 * header and stays here.  dev/MEASURED-rtl9602c-eth-history-2026-09-14.md sec 5. */
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
/* The PON port's own VLAN (members PON + CPU), as stock does (PON 4025, LAN 4005): a WAN
 * broadcast then floods to the CPU only, never to a LAN port. 0 = the PON shares VID 1. */
static uint pon_vid = 4025;
module_param(pon_vid, uint, 0444);
MODULE_PARM_DESC(pon_vid, "VLAN of the PON port, apart from the LAN's (default 4025 as stock; 0 = shared VID 1)");
#define SW_WAN_PVID	(pon_vid ? pon_vid : SW_DEFAULT_VID)
/* Indirect VLAN 4k-table access (field positions):
 * TBL_ACCESS_CTRL[31]=start [20:9]=addr/VID [6:4]=method(1) [3]=cmd(1=write)
 * [2:0]=type(1=VLAN); STS bit13=BUSY; WR_DATA holds the entry word. */
#define SW_TBL_CTRL		0x12000
#define SW_TBL_STS		0x12004
#define SW_TBL_WRDATA		0x12008
#define SW_TBL_BUSY		BIT(13)
#define SW_TBL_TRIES		1000u	/* the bound the poll below refuses at */
#define SW_TBL_VLAN_WR(vid)	(BIT(31) | (((vid) & 0xfff) << 9) | (1u << 4) | (1u << 3) | 1u)
/* THE SAME THREE ADDRESSES ARE SPELLED THREE TIMES IN THIS ...
 * dev/MEASURED-luna_eth.c.md sec 19. */
#define CPUTAG1_OMCI_SID(s)	(((s) & 0x7f) << 8)	/* R_CPUTAG1CR[14:8] */
#define CPUTAG1_B1		0x2	/* bit1: live 9602C stock reads CPUTAG1CR=0x4002; not present in the 9607C register map */
/* SUPERSEDED: an earlier reading had stock OR-ing 0x4070 (bits 4/5/6 as the
 * cpu-tag format/enable).  The live 9602C devmem read below is 0x4002. */
#define CPUTAG1_LOW		0x02	/* live-stock ref ONU devmem: CPUTAG1CR = 0x4002 (the earlier 0x4070 derivation was wrong) */

/* The switch L34 (NAPT) offload module is compiled in via this driver's TU to
 * avoid a separate Kbuild object; it carries its own header include guard. */
#include "rtl9602c_l34.c"
#ifdef CONFIG_GPON_FLOW_OFFLOAD
#include <net/pkt_cls.h>	/* the flower classifier the TC offload below parses */
#include "gpon_flow_offload.h"	/* the core TC-offload lifecycle */
#endif

/* HW NAT (switch L34 offload) gate, module-param rather than Kconfig so the
 * offload-vs-AQM comparison is one bootarg apart rather than one build.  The
 * default is the OPERATOR'S: it decides which datapath ships, and that decision
 * is taken with measured throughput and loaded latency, never quietly here. */
static int hw_nat = 1;	/* ON by default (operator, 2026-09-15) */
/* ★★★ THE DEFAULT IS 1, AND IT IS THE OPERATOR'S CALL, NOT AN INFERENCE.
 * (2026-09-15: *"=1 por defecto y corrigir"*.)  The standing objective has
 * always been *"mantener el acelerador de hardware activo por defecto"*, and 0
 * contradicted it silently: at 0 neither the TC lifecycle handle nor the
 * /proc node exists, so the board forwarded in software at 11% of stock with
 * its CPU at 100% and nothing could say why.
 *
 * MEASURED before flipping it, not after: `hwnat_wedge_ab.py` booted BOTH arms
 * twice on the same image bytes and neither wedged -- 0/2 each, the host
 * pinging the board's own LAN address as the deciding witness.  ⚠ That was at
 * 10 pps, which is not traffic: it says the gate can be OPENED safely, never
 * that the engine survives load.  The load question belongs to the benchmark.
 *
 * ⚠ AND OPENING IT IS NOT SUFFICIENT, which is why the old comment's "armed
 * lazily on first offload" was wrong twice over: the thing it claimed to arm
 * was CONSTRUCTED INSIDE THIS GATE, so nothing could arm it.
 *
 * ✔ MEASURED 2026-09-16, and this comment used to end "with the gate open the
 * engine still reads provisioned 0 and offered=0 ... what installs a flow is
 * still owed".  That was true for one day.  With a forwarded connection through
 * the board the engine reads offered=28988 installed=372 hits=3012 -- it
 * installs flows and the hardware path counts hits.  What is still owed is
 * narrower and is NOT about this gate: the case's own forged flow is software
 * forwarded, and the leading explanation is our INSTRUMENT (the NAPT hit
 * bitmap is clear-on-read and has two readers).
 */
module_param(hw_nat, int, 0444);
MODULE_PARM_DESC(hw_nat, "enable RTL9602C switch L34 hardware NAT offload (0=off)");

static const struct luna_eth_gmac luna_chip_gmac = {
	.name		= "RTL9602C",
	.cmd		= 0x0A,		/* RxChkSum | RxJumboSupport */
	.tcr		= 0x00000C00,	/* TX pad ON (bit0 = 0) */
	.rxokmul_seed	= true,
	.tx_dummy_rings	= true,
	.uboot_inherit	= true,
	.memctl		= true,
	.pbo_reinit	= true,
	.wan_ctrl_flow	= true,
};

struct luna_eth_chip_state {
	struct rtl9602c_l34 l34;	/* switch L3/L4 (NAPT) hardware-offload engine */
#ifdef CONFIG_GPON_FLOW_OFFLOAD
	struct gpon_flow_offload *fo;	/* the COMMON TC lifecycle, drivers/net/gpon/ */
	struct notifier_block l34_addr_nb;	/* releases the WAN rows when its address leaves */
	struct eth_neigh_watch l34_neigh;	/* ...its rows follow the neighbours they copied */
	struct notifier_block l34_routed_nb;	/* ...and are released when the routed switch changes */
#endif
};

#ifdef CONFIG_GPON_FLOW_OFFLOAD
static void luna_chip_wan_rx_mode(struct net_device *ndev);
#define LUNA_CHIP_WAN_RX_MODE luna_chip_wan_rx_mode
#endif
#include "luna_eth.c"

/* SW-follows-HW slot mapping. TxCDO is NOT writable while the ...
 * dev/MEASURED-luna_eth.c.md sec 25. */
#define SW_PHY_PWRDN_GUARD	0x00214
#define   PHY_PWRDN_GUARD_OK	0x1f	/* low 5 bits, all set = port 0 may be powered down */

static int rtl9602c_gphy_wait(struct luna_eth *ep)
{
	int i;

	for (i = 0; i < 10000; i++) {
		if (!(ioread32(ep->sw + SW_GPHY_IND_RD) & GPHY_IND_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

/* -> 0 and *out set, or negative with *out LEFT EXACTLY AS IT WAS: an all-ones
 * MII read is a plausible register value, so a failure must not return one. */
static int rtl9602c_gphy_read__locked(struct luna_eth *ep, unsigned int phyad,
				      unsigned int reg, u16 *out)
{
	u32 adr = GPHY_IND_PHY(phyad) | luna_gphy_mii_ocp(reg);

	/* PREFLIGHT: a command issued while the previous transaction is still
	 * BUSY makes the window answer for the wrong one. */
	if (rtl9602c_gphy_wait(ep))
		return -ETIMEDOUT;
	iowrite32(adr | GPHY_IND_EN, ep->sw + SW_GPHY_IND_CMD);
	if (rtl9602c_gphy_wait(ep))
		return -ETIMEDOUT;
	*out = ioread32(ep->sw + SW_GPHY_IND_RD) & 0xffff;
	return 0;
}

static int rtl9602c_gphy_write__locked(struct luna_eth *ep, unsigned int phyad,
				       unsigned int reg, u16 val)
{
	u32 adr = GPHY_IND_PHY(phyad) | luna_gphy_mii_ocp(reg);

	/* PREFLIGHT BEFORE THE DATA REGISTER: the value would otherwise be
	 * consumed by a transaction still in flight. */
	if (rtl9602c_gphy_wait(ep))
		return -ETIMEDOUT;
	iowrite32(val, ep->sw + SW_GPHY_IND_WD);
	iowrite32(adr | GPHY_IND_WREN | GPHY_IND_EN, ep->sw + SW_GPHY_IND_CMD);
	return rtl9602c_gphy_wait(ep);
}

static int rtl9602c_phy_read(struct net_device *ndev, unsigned int port,
			     unsigned int reg, u16 *val)
{
	return rtl9602c_gphy_read__locked(netdev_priv(ndev), port, reg, val);
}

static int rtl9602c_phy_write(struct net_device *ndev, unsigned int port,
			      unsigned int reg, u16 val)
{
	return rtl9602c_gphy_write__locked(netdev_priv(ndev), port, reg, val);
}

static const struct luna_phy_ops luna_chip_phy_ops = {
	.lock	= &luna_chip_gphy_lock,
	.read	= rtl9602c_phy_read,
	.write	= rtl9602c_phy_write,
};

/* The boot loader's own command words, issued with the ...
 * dev/MEASURED-luna_eth.c.md sec 26. */
static int rtl9602c_gphy_raw_cmd__locked(struct luna_eth *ep, u32 cmd)
{
	if (rtl9602c_gphy_wait(ep))
		return -ETIMEDOUT;
	iowrite32(cmd, ep->sw + SW_GPHY_IND_CMD);
	return rtl9602c_gphy_wait(ep);
}

static int rtl9602c_gphy_raw_write__locked(struct luna_eth *ep, u16 val, u32 cmd)
{
	if (rtl9602c_gphy_wait(ep))
		return -ETIMEDOUT;
	iowrite32(val, ep->sw + SW_GPHY_IND_WD);
	return rtl9602c_gphy_raw_cmd__locked(ep, cmd);
}

/* One word of a PHY's own address space, beyond the MII registers. */
static int rtl9602c_gphy_ocp_write__locked(struct luna_eth *ep, unsigned int phyad, u32 ocp, u16 val)
{
	return rtl9602c_gphy_raw_write__locked(ep, val, GPHY_IND_PHY(phyad) | ocp | GPHY_IND_WREN | GPHY_IND_EN);
}

static int rtl9602c_gphy_ocp_read__locked(struct luna_eth *ep, unsigned int phyad, u32 ocp, u16 *out)
{
	int rc = rtl9602c_gphy_raw_cmd__locked(ep, GPHY_IND_PHY(phyad) | ocp | GPHY_IND_EN);

	if (!rc)
		*out = ioread32(ep->sw + SW_GPHY_IND_RD) & 0xffff;
	return rc;
}

/* Minimal switch bring-up: permit ingress from every port and ...
 * dev/MEASURED-luna_eth.c.md sec 32. */
static void rtl9602c_sw_tbl_pause(void)
{
	udelay(1);
}

/* One VLAN 4k-table entry: ext[17:11] | untag[7:4] | mbr[3:0], IVL and FID 0 as the L34 lookups
 * expect. The extension ports are the CPU's: without them the VLAN filter drops its traffic. */
static int rtl9602c_vlan_write(struct luna_eth *ep, u16 vid, u32 mbr, u32 untag, u32 ext)
{
	struct hwio io = luna_sw_io(ep->sw);

	iowrite32((ext << 11) | (untag << 4) | mbr, ep->sw + SW_TBL_WRDATA);
	iowrite32(SW_TBL_VLAN_WR(vid), ep->sw + SW_TBL_CTRL);
	return gpon_ind_poll(&io, reg_make(SW_TBL_STS), SW_TBL_BUSY, SW_TBL_TRIES,
			     rtl9602c_sw_tbl_pause);
}

/* The WAN is VLAN-transparent: every VID reaches the CPU from the PON, tagged, since an OLT never
 * names its multicast VLAN to the ONU. The VLANs written after this one take their own entries. */
static int rtl9602c_wan_vids_admit(struct luna_eth *ep)
{
	u32 mbr = BIT(ep->swm->pon_port) | BIT(ep->swm->cpu_port);
	u16 vid;

	for (vid = 1; vid < 4095; vid++)
		if (rtl9602c_vlan_write(ep, vid, mbr, 0, 0x1) < 0)
			return vid;
	return 0;
}

/* Multicast from the fibre reaches the CPU only through the LUT entry of a group the unit joined
 * (rtl9602c_ipmc.c); until those entries are kept, or when they do not fit, the CPU alone gets the rest. */
static void rtl9602c_pon_mc_policy(struct luna_eth *ep)
{
	const struct rtl9602c_ipmc *m = &ep->cs.l34.ipmc;

	luna_pon_mc_admit(ep->sw, ep->swm, eth_mc_miss_drop(READ_ONCE(m->watching), !READ_ONCE(m->overflow),
							     READ_ONCE(m->capturers)));
}

static void luna_chip_sw_min_init(struct luna_eth *ep)
{
	if (!ep->sw)
		return;

	/* Stock switch-init prerequisites (full-init-sequence ...
	 * dev/MEASURED-luna_eth.c.md sec 33. */
	writel(readl(SOC_SW_ENABLE) | SW_EN_BIT, SOC_SW_ENABLE);

	/* (b) CFG_UNHIOL.IPG_COMPENSATION — swcore 0x23040 bit0. ORACLE-CONFIRMED real
	 * value diff (ours read 0xa8, working stock 0xa9) that demonstrably changed the
	 * OMCI forwarding class when poked live. The stock switch init sets it. */
	if (ep->swm->cfg_unhiol)
		iowrite32(ioread32(ep->sw + ep->swm->cfg_unhiol) | BIT(0),
			  ep->sw + ep->swm->cfg_unhiol);

	/* Egress queues are served round-robin, as the stock firmware of this chip and of the
	 * RTL9601D runs them; the reset value is the byte-weighted mode stock never runs. */
	iowrite32(ioread32(ep->sw + SW_WFQ_CTRL) | SW_WFQ_MODE_WRR, ep->sw + SW_WFQ_CTRL);

	/* (c) WRAP_GPHY_MISC.PATCH_PHY_DONE — swcore 0x110 bit0: the "switch ready /
	 * PHY patch done" latch the stock switch init asserts at completion. Without it
	 * the switch may not present itself as fully initialised to the direct-TX
	 * forwarding class. */
	if (ep->swm->gphy_misc)
		iowrite32(ioread32(ep->sw + ep->swm->gphy_misc) | BIT(0),
			  ep->sw + ep->swm->gphy_misc);

	/* Force NO port. The bootloader's WORKING config runs ...
	 * dev/MEASURED-luna_eth.c.md sec 34. */
	iowrite32(0, ep->sw + ep->swm->src_permit);
	/* Flood masks: ports 0,1 (LAN) + 3 (CPU), but NOT port 2 ...
	 * dev/MEASURED-luna_eth.c.md sec 35. */
	iowrite32((ioread32(ep->sw + ep->swm->bc_flood) | ep->swm->port_mask) & ~BIT(ep->swm->pon_port),
		  ep->sw + ep->swm->bc_flood);
	iowrite32((ioread32(ep->sw + ep->swm->unkn_mc_flood) | ep->swm->port_mask) & ~BIT(ep->swm->pon_port),
		  ep->sw + ep->swm->unkn_mc_flood);
	iowrite32((ioread32(ep->sw + ep->swm->unkn_uc_flood) | ep->swm->port_mask) & ~BIT(ep->swm->pon_port),
		  ep->sw + ep->swm->unkn_uc_flood);
	rtl9602c_pon_mc_policy(ep);
	/* Unknown-unicast that misses the L2 lookup (the host's ...
	 * dev/MEASURED-luna_eth.c.md sec 36. */
	iowrite32(0, ep->sw + ep->swm->lut_unkn_uc_da);	/* all ports FORWARD */
	if (ep->swm->ctl_acl) {
		int rc = luna_ctl_acl_install(ep->sw, ep->swm->ctl_acl, CTL_RX_PRIO);

		if (rc)
			netdev_err(ep->ndev, "control ACL not installed (%d): ARP shares the data ring under a flood\n",
				   rc);
	}
	/* 0x27000 (PISO) is a 5-bit isolation-vector INDEX per port, ...
	 * dev/MEASURED-luna_eth.c.md sec 37. */
	{
		int p, rc;

		u32 lan_ext = 0x7, pon = BIT(ep->swm->pon_port) | BIT(ep->swm->cpu_port);

		rc = rtl9602c_wan_vids_admit(ep);
		if (rc)
			netdev_warn(ep->ndev,
				    "swcore VLAN table BUSY at VID %d: a WAN VLAN from that one on is dropped before the CPU\n",
				    rc);
		/* As stock's 4005: every port and the CPU's extension ports, all untagged */
		rc = rtl9602c_vlan_write(ep, SW_DEFAULT_VID, 0xf, 0xf, lan_ext);
		if (rc < 0)
			netdev_warn(ep->ndev,
				    "swcore VLAN 4k-table engine still BUSY after %u tries (rc=%d): default VLAN %u may not be installed, LAN egress may stay filtered\n",
				    SW_TBL_TRIES, rc, SW_DEFAULT_VID);
		/* As stock's 4025: the PON floods to the CPU only */
		if (pon_vid && rtl9602c_vlan_write(ep, pon_vid, pon, pon, 0x1) < 0)
			netdev_warn(ep->ndev,
				    "swcore VLAN table BUSY writing the PON VLAN %u: WAN broadcasts may reach the LAN ports\n",
				    pon_vid);
		iowrite32(0, ep->sw + SW_VLAN_PORT_ACCEPT_FRAME_TYPE);	/* accept all frame types */
		/* PVID is a 12-bit element array packed TWO PER WORD, so a ...
		 * dev/MEASURED-luna_eth.c.md sec 39. */
		for (p = 0; p < SW_VLAN_PB_VID_PORTS; p++) {
			void __iomem *w;
			u32 vid = p == ep->swm->pon_port ? SW_WAN_PVID : SW_DEFAULT_VID;

			if (!(ep->swm->port_mask & BIT(p)))
				continue;	/* not a port on this chip */

			/* One VID per LAN port: member = that port + the CPU
			 * port, UNTAGGED on the port and TAGGED toward the CPU,
			 * which is what lets eth0.<vid> demux the socket. The
			 * CPU port keeps SW_DEFAULT_VID itself. */
			/* ⚠ COPPER SOCKETS ONLY.  The CPU port carries the tagged
			 * trunk, and the PON port is the FIBRE -- giving either
			 * its own LAN VID would put the WAN datapath in a
			 * per-socket VLAN.  Both come from the chip's own map,
			 * never from a literal: pon_port is 2 here and 4 on the
			 * RTL9603CVD, so a hardcoded skip would be wrong on the
			 * next board. */
			if (port_vlans && p != ep->swm->cpu_port &&
			    p != ep->swm->pon_port) {
				u32 mbr = BIT(p) | BIT(ep->swm->cpu_port);

				vid = SW_PORT_VID_BASE + p;
				if (rtl9602c_vlan_write(ep, vid, mbr, BIT(p), lan_ext) < 0)
					netdev_warn(ep->ndev,
						    "swcore VLAN table BUSY writing per-port VLAN %u for port %u: that socket will not be separable\n",
						    vid, p);
			}

			w = ep->sw + sw_packed_off(SW_VLAN_PB_VID, p,
						   SW_VLAN_PB_VID_BITS);
			iowrite32(sw_packed_ins(ioread32(w), p,
						SW_VLAN_PB_VID_BITS,
						vid), w);
		}
		iowrite32(0xf, ep->sw + SW_VLAN_INGRESS);	/* ingress filter, ports 0-3 */
		iowrite32(SW_VLAN_CTRL_VAL, ep->sw + SW_VLAN_CTRL); /* enable VLAN function */
		luna_gpon_vlan_filter_ok(pon_vid != 0);
	}
	/* Force the CPU port link UP. CRITICAL: the bootloader leaves ...
	 * dev/MEASURED-luna_eth.c.md sec 40. */
	iowrite32(SW_ABLTY_1G_FD_UP, ep->sw + SW_FORCE_P_ABLTY(ep, ep->swm->cpu_port));
	iowrite32(0xfff, ep->sw + SW_ABLTY_FORCE_MODE(ep, ep->swm->cpu_port));

	/* Accept short (runt) frames on the PON port (2) and CPU port ...
	 * dev/MEASURED-luna_eth.c.md sec 41. */
	iowrite32(ioread32(ep->sw + SW_P_MISC_PORT_9602C(ep->swm->pon_port)) | BIT(2),
		  ep->sw + SW_P_MISC_PORT_9602C(ep->swm->pon_port));	/* port 2 (PON) */
	iowrite32(ioread32(ep->sw + SW_P_MISC_PORT_9602C(ep->swm->cpu_port)) | BIT(2),
		  ep->sw + SW_P_MISC_PORT_9602C(ep->swm->cpu_port));	/* port 3 (CPU) */

	/* Force the PON port link UP for the same reason: the ...
	 * dev/MEASURED-luna_eth.c.md sec 42. */
	iowrite32(SW_ABLTY_1G_FD_UP, ep->sw + SW_FORCE_P_ABLTY(ep, ep->swm->pon_port));
	/* Force P2 link UP (0xfff). This is REQUIRED for the DS path ...
	 * dev/MEASURED-luna_eth.c.md sec 43. */
	iowrite32(0xfff, ep->sw + SW_ABLTY_FORCE_MODE(ep, ep->swm->pon_port));
}

#ifdef CONFIG_GPON_FLOW_OFFLOAD
static void rtl9602c_ipmc_kick(struct luna_eth *ep);

/* Every multicast join or leave of the host on the WAN (or a VLAN of it) lands here: the switch follows it. */
static void luna_chip_wan_rx_mode(struct net_device *ndev)
{
	rtl9602c_ipmc_kick(*(struct luna_eth **)netdev_priv(ndev));
}
#endif

/* Full GMAC register program in the stock init ORDER, for an ...
 * dev/MEASURED-luna_eth.c.md sec 83. */
static void luna_chip_cpu_tag_arm(struct luna_eth *ep)
{
	ep_wr(ep, R_CPUTAGCR, 0x00000000u);	/* OFF first: re-latches the ADD engine */
	wmb();					/* the clear must land before the re-arm edge */
	ep_wr(ep, R_CPUTAGCR, 0x901eff04u);	/* ON: post-init stock value */
	ep_wr(ep, R_CPUTAG1CR,
	      CPUTAG1_OMCI_SID(LUNA_OMCC_SID) | CPUTAG1_LOW);	/* = 0x4002 (live stock) */

	if (!ep->sw)
		return;
	/* OFF->ON edge on the SWITCH CPU-port cpu-tag PARSER, mirroring the GMAC
	 * re-latch above. */
	iowrite32(0, ep->sw + SW_MAC_CPU_TAG_CTRL);
	wmb();					/* clear must land before the re-arm edge */
	iowrite32(sw_tagaware, ep->sw + SW_MAC_CPU_TAG_CTRL);
	/* aux flow-control thresholds, live-stock, NAMED from the chip's own
	 * register map (2026-08-29) */
	iowrite32(0x00400034, ep->sw + SW_FC_P_LO_TH);
	iowrite32(0x00f000ea, ep->sw + SW_FC_P_FCOFF_HI_TH);
	iowrite32(0x00400034, ep->sw + SW_FC_P_FCOFF_LO_TH);
	/* Stock puts the fibre-side global thresholds (560/548, 534/519) above what the shared pool can hold
	 * (it stops near 496), so that state never engages; the reset values (512/500, 486/471) are within reach. */
	iowrite32(0x02300224, ep->sw + SW_FC_PON_GLB_HI_TH);
	iowrite32(0x02160207, ep->sw + SW_FC_PON_GLB_LO_TH);
}

/* gpon_pbo_init() — re-run the gpon driver's full PON ...
 * dev/MEASURED-luna_eth.c.md sec 85. */
#define SW_RDY_FOR_PATCH_TRIES	200000	/* 200 ms at 1 us a try; the bound the wait below refuses at */

/* What the stock firmware writes to the two copper PHYs after every reset, before it declares them patched: the FE
 * PHY's link-down power saving, the GE PHY's 10M word, then the GE PHY's parameters (address, value) [tier 1: the
 * table of the stock kernel, the same in both RTL9602C boards' stock]. */
#define RTL9602C_GE_PHY		1
#define GPHY_SRAM_ADDR		0xa436	/* a PHY parameter: its address here ... */
#define GPHY_SRAM_DATA		0xa438	/* ... then its value here */
#define GE_PARAM(addr, val)	{ RTL9602C_GE_PHY, true, GPHY_SRAM_ADDR, addr }, { RTL9602C_GE_PHY, false, GPHY_SRAM_DATA, val }
static const struct eth_phy_word rtl9602c_gphy_words[] = {
	{ 0, false, 0xa430, 0x019c }, { RTL9602C_GE_PHY, false, 0xbce4, 0x0000 },
	GE_PARAM(0x809a, 0x4ad4), GE_PARAM(0x809e, 0x0506), GE_PARAM(0x809c, 0xd107), GE_PARAM(0x80a4, 0x930c), GE_PARAM(0x80a2, 0x8b19),	/* 1000M master */
	GE_PARAM(0x80b0, 0x8006), GE_PARAM(0x80ac, 0x429c), GE_PARAM(0x80b4, 0x5319), GE_PARAM(0x80b2, 0xff7f),			/* 1000M slave */
	GE_PARAM(0x808e, 0x15a4), GE_PARAM(0x8089, 0x6bf7), GE_PARAM(0x808a, 0xf708), GE_PARAM(0x8092, 0x401e), GE_PARAM(0x8090, 0xe61f),
	GE_PARAM(0x8088, 0xb06b),									/* 100M */
	GE_PARAM(0x8062, 0x2000), GE_PARAM(0x806a, 0x2300), GE_PARAM(0x8072, 0x2300), GE_PARAM(0x804d, 0x2444),			/* green tables */
};

static int rtl9602c_patch_efuse(void *ctx, unsigned int entry, u16 *out)
{
	struct luna_eth *ep = ctx;

	return luna_efuse_read(ep->sw, ep->swm, entry, out);
}

static int rtl9602c_patch_phy_rd(void *ctx, unsigned int phy, u32 reg, u16 *out)
{
	return rtl9602c_gphy_ocp_read__locked(ctx, phy, reg, out);
}

static int rtl9602c_patch_phy_wr(void *ctx, unsigned int phy, u32 reg, u16 val)
{
	return rtl9602c_gphy_ocp_write__locked(ctx, phy, reg, val);
}

static int rtl9602c_efuse_phy_wr(void *ctx, unsigned int phy, u32 reg, u16 val)
{
	struct luna_eth *ep = ctx;
	int rc = rtl9602c_gphy_ocp_write__locked(ep, phy, reg, val);

	netdev_info(ep->ndev, "efuse patch: phy %u word %#x <= %#06x (%d)\n", phy, reg, val, rc);
	return rc;
}

/* The efuse names a register by its unmapped address: only the switch core is this driver's to write. */
static int rtl9602c_efuse_reg_wr(void *ctx, u32 addr, u32 val)
{
	struct luna_eth *ep = ctx;
	u32 off = addr - (0xa0000000u | (u32)SWCORE_PHYS);

	if (addr < (0xa0000000u | (u32)SWCORE_PHYS) || off > ep->swm->swcore_size - 4 || (off & 3)) {
		netdev_warn(ep->ndev, "efuse patch: %#x <= %#x is outside the switch core: NOT written\n", addr, val);
		return -ERANGE;
	}
	iowrite32(val, ep->sw + off);
	netdev_info(ep->ndev, "efuse patch: switch %#x <= %#x\n", off, val);
	return 0;
}

static const struct eth_chip_patch_io rtl9602c_words_io = {
	.phy_rd = rtl9602c_patch_phy_rd, .phy_wr = rtl9602c_patch_phy_wr,
};
static const struct eth_chip_patch_io rtl9602c_efuse_io = {
	.efuse = rtl9602c_patch_efuse, .phy_wr = rtl9602c_efuse_phy_wr, .reg_wr = rtl9602c_efuse_reg_wr,
};
/* [tier 1: the stock reader and its patch loop] */
static const struct eth_efuse_geom rtl9602c_efuse_geom = {
	.entries = LUNA_EFUSE_ENTRIES, .count = 11, .first = 12, .phys = 3,
};

/* Never fatal: a unit whose PHY refuses a word runs as it did before this patch existed, and says so. */
static void rtl9602c_gphy_patch__locked(struct luna_eth *ep)
{
	struct eth_efuse_patch_stat st;
	struct eth_phy_words_stat ws;
	int rc, erc;

	rc = eth_phy_words_apply(&rtl9602c_words_io, ep, rtl9602c_gphy_words, ARRAY_SIZE(rtl9602c_gphy_words), &ws);
	if (rc)
		netdev_err(ep->ndev, "gphy patch: the PHY parameters did NOT all land (%d): the copper ports run unpatched\n",
			   rc);
	erc = eth_efuse_patch(&rtl9602c_efuse_io, ep, &rtl9602c_efuse_geom, &st);
	netdev_info(ep->ndev,
		    "gphy patch: %u word(s) (%d), %u did not hold their value (the first held %04x); efuse patch %s (%d): %u PHY word(s), %u register word(s), %u refused, %u failed\n",
		    ws.words, rc, ws.differed, ws.first, st.present ? "programmed" : "not programmed", erc,
		    st.phy_words, st.reg_words, st.refused, st.failed);
}

/* > 0, or negative and the caller must NOT bring the ...
 * dev/MEASURED-luna_eth.c.md sec 86. */
static int rtl9602c_uboot_swcore_bringup(struct luna_eth *ep)
{
	void __iomem *sysstat = (void __iomem *)0xb8000044ul;	/* SoC SYSREG */
	int to, rc = 0;

	if (!ep->sw)
		return -ENODEV;
	/* wait for RDY_FOR_PATCH (0xB8000044 bit1) after the IP-block ...
	 * dev/MEASURED-luna_eth.c.md sec 87. */
	for (to = 0; to < SW_RDY_FOR_PATCH_TRIES && !(readl(sysstat) & 0x2); to++)
		udelay(1);
	if (to == SW_RDY_FOR_PATCH_TRIES)
		netdev_warn(ep->ndev,
			    "SoC never raised RDY_FOR_PATCH after %d us: the GMAC<->switch resync below runs against a block that did not announce itself\n",
			    SW_RDY_FOR_PATCH_TRIES);
	/* GPHY analog patch (0x6485 variant only) through the GPHY ...
	 * dev/MEASURED-luna_eth.c.md sec 134. */
	iowrite32(0xa0000000, ep->sw + SW_CHIP_INFO);
	if ((ioread32(ep->sw + SW_CHIP_INFO) & 0xffff) == 0x6485) {
		static const u32 patch_cmd[] = {
			0x0061b844, 0x0021b906, 0x0021b906 /* indirect write, read, read */
		};
		unsigned int i;

		mutex_lock(&luna_chip_gphy_lock);
		rc = rtl9602c_gphy_raw_write__locked(ep, 0xfffb, patch_cmd[0]);
		for (i = 1; !rc && i < ARRAY_SIZE(patch_cmd); i++)
			rc = rtl9602c_gphy_raw_cmd__locked(ep, patch_cmd[i]);
		mutex_unlock(&luna_chip_gphy_lock);
	}
	/* ★ THE SELECTOR IS PUT BACK BEFORE ANY RETURN.  SW_CHIP_INFO is left
	 *   pointing at the variant window while the patch runs, and a bail-out
	 *   that skipped this would hand the rest of the driver a register block
	 *   that answers for something else. */
	iowrite32(0x0, ep->sw + SW_CHIP_INFO);
	if (rc) {
		netdev_err(ep->ndev,
			   "GPHY analog patch did not complete (%d): the 0x6485 variant fixup is NOT applied, so the patch-done bit is NOT asserted and this bring-up FAILS\n",
			   rc);
		return rc;
	}
	mutex_lock(&luna_chip_gphy_lock);
	rtl9602c_gphy_patch__locked(ep);
	mutex_unlock(&luna_chip_gphy_lock);
	/* the SECOND site of the same register -- from the chip table too, or the
	 * conversion would be half done, which is worse than none: one write
	 * would follow a corrected offset and the other would not. */
	if (ep->swm->gphy_misc)
		iowrite32(1, ep->sw + ep->swm->gphy_misc);	/* PATCH_PHY_DONE */
	msleep(500);
	/* ★ THE BOOT LOADER'S OWN BMCR PAIR, and NEITHER of them ...
	 * dev/MEASURED-luna_eth.c.md sec 88. */
	{
		static const u16 bringup_bmcr[2] = { 0x3000, 0x1140 };
		unsigned int phy;

		mutex_lock(&luna_chip_gphy_lock);
		for (phy = 0; phy < ARRAY_SIZE(bringup_bmcr); phy++) {
			bool want_down = luna_uni_port_locked(phy);
			u16 v = bringup_bmcr[phy], back = 0;

			if (want_down)
				v |= BMCR_PDOWN;
			rc = rtl9602c_gphy_write__locked(ep, phy, 0, v);
			if (!rc)
				rc = rtl9602c_gphy_read__locked(ep, phy, 0, &back);
			if (rc) {
				netdev_err(ep->ndev,
					   "gphy: phy %u BMCR %04x did not land (%d); its power state is UNKNOWN, not assumed\n",
					   phy, v, rc);
				break;
			}
			/* ★ A DESIRED MASK IS NOT PROOF THE PHY STAYED DOWN. This ...
			 * dev/MEASURED-luna_eth.c.md sec 89. */
			if (want_down && !(back & BMCR_PDOWN)) {
				netdev_err(ep->ndev,
					   "gphy: phy %u is administratively LOCKED and reads BMCR %04x -- the power-down did not hold across the reset; refusing to bring the interface up over it\n",
					   phy, back);
				rc = -EIO;
				break;
			}
			/* ⚠ AND THE UNLOCKED CASE IS A FAILURE TOO. This write is the ...
			 * dev/MEASURED-luna_eth.c.md sec 90. */
			if (!want_down && (back & BMCR_PDOWN)) {
				netdev_err(ep->ndev,
					   "gphy: phy %u is not locked and reads BMCR %04x -- it is powered DOWN, so that socket would link and carry nothing\n",
					   phy, back);
				rc = -EIO;
				break;
			}
			msleep(500);
		}
		mutex_unlock(&luna_chip_gphy_lock);
		if (rc)
			return rc;
	}
	/* ⚠ 0x230c4 STAYS A LITERAL: this chip's register map calls it SVLAN_UPLINK_PMSK,
	 * luna_gpon_regs.h calls the same address SMI_CTRL_3, and the two never run
	 * on one silicon -- dev/FINDING-one-address-two-blocks-smi-vs-svlan.md.
	 * Naming it either way would make the disagreement look settled. */
	iowrite32(0,          ep->sw + 0x230c4);	/* SVLAN uplink port */
	iowrite32(0x003fffff, ep->sw + SW_PISO_PORT);	/* port isolation, element 0 */
	iowrite32(0x003fffff, ep->sw + SW_PISO_PORT + 1 * SW_PISO_PORT_STRIDE);
	/* ★ THE SAME TWO REGISTERS THIS FILE ALREADY REACHES THROUGH ...
	 * dev/MEASURED-luna_eth.c.md sec 91. */
	iowrite32(0x00000196, ep->sw + SW_FORCE_P_ABLTY(ep, ep->swm->cpu_port));
	iowrite32(0x00000fff, ep->sw + SW_ABLTY_FORCE_MODE(ep, ep->swm->cpu_port));
	iowrite32(0x00012bbd, ep->sw + SW_METER_TB_CTRL);	/* meter tick-token */
	iowrite32(0,          ep->sw + SW_VLAN_CTRL);	/* VLAN function disable */
	iowrite32(1, ep->sw + SW_VLAN_EGRESS_TAG);			/* VLAN keep-format p0-3 */
	iowrite32(1, ep->sw + SW_VLAN_EGRESS_TAG + 1 * 4);
	iowrite32(1, ep->sw + SW_VLAN_EGRESS_TAG + 2 * 4);
	iowrite32(1, ep->sw + SW_VLAN_EGRESS_TAG + 3 * 4);
	iowrite32(0, ep->sw + SW_MAC_CPU_TAG_CTRL);			/* CPU_TAG_CTRL=0 (re-armed in hw_program) */
	writel(readl(sysstat) | 1, sysstat);		/* patch done: set 0xB8000044 bit0 */
	return 0;
}

#ifdef CONFIG_GPON_FLOW_OFFLOAD
/* This family's five-op table for the COMMON TC lifecycle.  Included here, not
 * at the top, because it needs `struct luna_eth` complete and the WAN ops
 * table defined -- and the LAN ops table below needs ITS ndo_setup_tc. */
#include "rtl9602c_l34_tc.c"
#include "rtl9602c_ipmc.c"
#endif

/* The switch core reset as a recovery (eth_core_recovery.h): the watch timer or sw_core_reset only asks.
 * LOCK ORDER: rtnl (VLAN uppers, open/stop) -> rtl9602c_l34_tc_mutex (flow installs) -> ep->cs.l34.lock (L34 and IPMC
 * table rows) -> luna_chip_gphy_lock (inside the PHY steps). Every table writer of this switch holds one of them,
 * so no row changes while the state is compared.
 * What the reset moves (lab X111W 2026-11-19, under traffic, flows offloaded): none of the words below that our
 * code writes, and no table row. It moves status only: CHIP_DEBUG_OUT 0xa0, 0x1c098, the low byte of 0x20808 and
 * 0x20c08, the buffer accounting 0x23100-0x23118 and the queue occupancy, all moving on their own on an idle unit;
 * 0x15000-0x15004, which the chip reloads itself (idle: before the re-init; after a CPU-port stall: later). In the
 * NIC, its counters and the interrupt masks that NAPI holds while it is stopped. Those are "status", compared and
 * never judged. */
static const struct eth_reg_range rtl9602c_state_regs[] = {
	{ 0x00000, 40 }, { 0x000a4, 23 }, { 0x00100, 128 }, { 0x11000, 128 }, { 0x13000, 16 }, { 0x15008, 22 },
	{ 0x16000, 8 }, { 0x17000, 4 }, { 0x1c000, 38 }, { 0x1c09c, 153 }, { 0x1e000, 24 }, { 0x20000, 256 },
	{ 0x20800, 2 }, { 0x2080c, 13 }, { 0x20c00, 2 }, { 0x20c0c, 13 }, { 0x23000, 64 }, { 0x2311c, 57 },
	{ 0x25000, 4 }, { 0x27000, 16 }, { 0x2a000, 10 }, { 0x2d800, 64 }, { 0, 0 } };
static const struct eth_reg_range rtl9602c_state_status[] = {
	{ 0x000a0, 1 }, { 0x15000, 2 }, { 0x1c098, 1 }, { 0x20808, 1 }, { 0x20c08, 1 }, { 0x23100, 7 }, { 0x2d018, 3 },
	{ 0x2d028, 4 }, { 0x2d058, 32 }, { 0, 0 } };
static const struct eth_reg_range rtl9602c_state_mib[] = { { SW_STAT_PORT_TX_MIB, 128 },
							   { SW_STAT_PORT_RX_MIB_RTL9602C, 128 }, { 0, 0 } };
static const struct { const char *name; u8 type, words; u16 rows; bool learned; } rtl9602c_state_l34[] = {
	{ "l34 route", L34_TBL_L3ROUTE, L34_WORDS_L3ROUTE, 16 }, { "l34 pppoe", L34_TBL_PPPOE, L34_WORDS_PPPOE, 8 },
	{ "l34 nexthop", L34_TBL_NEXTHOP, L34_WORDS_NEXTHOP, 16 }, { "l34 netif", L34_TBL_NETIF, L34_WORDS_NETIF, 16 },
	{ "l34 extip", L34_TBL_EXTIP, L34_WORDS_EXTIP, 8 }, { "l34 arp", L34_TBL_ARP, L34_WORDS_ARP, 128, true },
	{ "l34 napt in", L34_TBL_NAPTR_IN, L34_WORDS_NAPTR_IN, L34_NAPT_ENTRIES, true },
	{ "l34 napt out", L34_TBL_NAPT_OUT, L34_WORDS_NAPT_OUT, L34_NAPT_ENTRIES, true },
};

static unsigned int luna_chip_sw_blocks(void *ctx, struct eth_state_block *b)
{
	struct luna_eth *ep = ctx;
	const struct luna_ctl_acl *acl = ep->swm->ctl_acl;
	struct luna_state_rd *t = ep->sw_rd;
	struct eth_reg_range (*rg)[3] = ep->sw_rg;
	unsigned int i, n = 0;

	BUILD_BUG_ON(5 + 4 + ARRAY_SIZE(rtl9602c_state_l34) > ETH_STATE_BLOCKS);
	eth_state_add(b, &n, "registers", false, luna_state_reg_rd, ep->sw, rtl9602c_state_regs, 32, NULL);
	eth_state_add(b, &n, "status", true, luna_state_reg_rd, ep->sw, rtl9602c_state_status, 16, NULL);
	eth_state_add(b, &n, "mib", true, luna_state_reg_rd, ep->sw, rtl9602c_state_mib, 3, NULL);
	eth_state_add(b, &n, "nic", false, luna_state_reg_rd, ep->base, luna_state_nic, 8, NULL);
	eth_state_add(b, &n, "nic status", true, luna_state_reg_rd, ep->base, luna_state_nic_status, 8, NULL);
	*t = (struct luna_state_rd){ .base = ep->sw, .cmd = SW_TBL_VLAN_WR(0) & ~BIT(3), .addr_shift = 9 };
	luna_state_tbl(b, &n, "vlan", false, t, *rg++, luna_state_tbl_row, 2, 4096, 0);
	if (acl) {
		*++t = (struct luna_state_rd){ .base = ep->sw, .cmd = acl->rule_tbl, .addr_shift = acl->addr_shift };
		luna_state_tbl(b, &n, "acl rules", false, t, *rg++, luna_state_tbl_row, LUNA_ACL_WORDS, acl->rules,
			       acl->data_base);
		*++t = (struct luna_state_rd){ .base = ep->sw, .cmd = acl->act_tbl, .addr_shift = acl->addr_shift };
		luna_state_tbl(b, &n, "acl actions", false, t, *rg++, luna_state_tbl_row, acl->act_words, acl->rules, 0);
	}
	*++t = (struct luna_state_rd){ .base = ep->sw, .l2 = &luna_l2_rtl9602c };
	luna_state_tbl(b, &n, "l2", true, t, *rg++, luna_state_l2_row, 3, 1088, 0);
	for (i = 0; ep->cs.l34.ready && i < ARRAY_SIZE(rtl9602c_state_l34); i++) {
		*++t = (struct luna_state_rd){ .base = ep->cs.l34.sw, .type = rtl9602c_state_l34[i].type,
					       .l34 = &luna_l34_acc_rtl9602c };
		luna_state_tbl(b, &n, rtl9602c_state_l34[i].name, rtl9602c_state_l34[i].learned, t, *rg++,
			       luna_state_l34_row, rtl9602c_state_l34[i].words, rtl9602c_state_l34[i].rows, 0);
	}
	return n;
}

/* The offload engine's locks, taken inside rtnl around a switch-core recovery (luna_eth.c). */
static void luna_chip_offload_lock(struct luna_eth *ep)
{
#ifdef CONFIG_GPON_FLOW_OFFLOAD
	mutex_lock(&rtl9602c_l34_tc_mutex);
#endif
	if (ep->cs.l34.ready)
		mutex_lock(&ep->cs.l34.lock);
}

static void luna_chip_offload_unlock(struct luna_eth *ep)
{
	if (ep->cs.l34.ready)
		mutex_unlock(&ep->cs.l34.lock);
#ifdef CONFIG_GPON_FLOW_OFFLOAD
	mutex_unlock(&rtl9602c_l34_tc_mutex);
#endif
}

static void luna_chip_offload_assert_held(struct luna_eth *ep)
{
#ifdef CONFIG_GPON_FLOW_OFFLOAD
	lockdep_assert_held(&rtl9602c_l34_tc_mutex);
#endif
	if (ep->cs.l34.ready)
		lockdep_assert_held(&ep->cs.l34.lock);
}


/* Gate ONE copper port: down = its MAC link forced down and its PHY powered
 * down, up = both released. The UNI administrative lock and a LAN loop share
 * it; @who names the caller in the log. @mac_suffices (only a loop holds it
 * down): a port whose PHY this die will not power down is held at its MAC link
 * alone, since forwarding is what must stop. luna_chip_gphy_lock held. */
static int luna_chip_port_gate__locked(struct luna_eth *ep, unsigned int port, bool down,
				      bool mac_suffices, const char *who, u16 *bmcr_back,
				      u32 *mode_back)
{
	u32 force, mode, guard;
	bool phy_allowed = true;
	u16 bmcr, want;
	int rc;

	force = ioread32(ep->sw + SW_FORCE_P_ABLTY(ep, port));
	mode = ioread32(ep->sw + SW_ABLTY_FORCE_MODE(ep, port));
	if (down) {
		iowrite32(force & ~ABLTY_LINK, ep->sw + SW_FORCE_P_ABLTY(ep, port));
		iowrite32(mode | ABLTY_LINK, ep->sw + SW_ABLTY_FORCE_MODE(ep, port));
	} else {
		iowrite32(mode & ~ABLTY_LINK, ep->sw + SW_ABLTY_FORCE_MODE(ep, port));
	}
	force = ioread32(ep->sw + SW_FORCE_P_ABLTY(ep, port));
	mode = ioread32(ep->sw + SW_ABLTY_FORCE_MODE(ep, port));
	/* BOTH words: the mode says the LINK ability is forced, the value says
	 * what it is forced TO. */
	if (!(mode & ABLTY_LINK) != !down || (down && (force & ABLTY_LINK))) {
		dev_err(ep->dev,
			"%s port %u: the MAC gate did not take -- FORCE_P_ABLTY %08x, ABLTY_FORCE_MODE %08x, so the port is NOT gated\n",
			who, port, force, mode);
		return -EIO;
	}

	if (port == 0) {
		guard = ioread32(ep->sw + SW_PHY_PWRDN_GUARD);
		phy_allowed = (guard & PHY_PWRDN_GUARD_OK) == PHY_PWRDN_GUARD_OK;
	}
	if (!phy_allowed && down && mac_suffices) {
		dev_warn_once(ep->dev,
			      "%s port %u: this die guards port 0's PHY power on %#05x (%08x): the port is held down at its MAC link only\n",
			      who, port, SW_PHY_PWRDN_GUARD, guard);
		*bmcr_back = 0;
		*mode_back = mode;
		return 0;
	}
	if (!phy_allowed && down) {
		/* ⚠ THIS IS A FAILURE, NOT A PARTIAL SUCCESS. The board's own ...
		 * dev/MEASURED-luna_eth.c.md sec 138. */
		dev_err(ep->dev,
			"%s port %u: the MAC gate took but this die guards port 0's PHY power on %#05x, which reads %08x -- the port is NOT fully gated and the gate stays owed\n",
			who, port, SW_PHY_PWRDN_GUARD, guard);
		return -EBUSY;
	}

	rc = rtl9602c_gphy_read__locked(ep, port, 0, &bmcr);
	if (rc) {
		dev_err(ep->dev,
			"%s port %u: its BMCR could not be read (%d); nothing is written back and the state is owed\n",
			who, port, rc);
		return rc;
	}
	/* ONLY THE POWER BIT MOVES: this board's own power-down ...
	 * dev/MEASURED-luna_eth.c.md sec 115. A PHY this die guards was never
	 * powered down by us: its release has no pulse, only a power-up if needed. */
	want = (u16)(phy_allowed ? bmcr | BMCR_PDOWN : bmcr & ~BMCR_PDOWN);
	rc = phy_allowed || want != bmcr ? rtl9602c_gphy_write__locked(ep, port, 0, want) : 0;
	if (!rc && !down && phy_allowed) {
		u16 mid;

		rc = rtl9602c_gphy_read__locked(ep, port, 0, &mid);
		if (!rc && !(mid & BMCR_PDOWN)) {
			dev_err(ep->dev,
				"%s port %u: the power-down half of the unlock pulse did not land (BMCR %04x) -- the far end would never re-negotiate\n",
				who, port, mid);
			rc = -EIO;
		}
		if (!rc)
			rc = rtl9602c_gphy_write__locked(ep, port, 0,
							 (u16)(mid & ~BMCR_PDOWN));
	}
	if (!rc)
		rc = rtl9602c_gphy_read__locked(ep, port, 0, &bmcr);
	if (!rc && (!(bmcr & BMCR_PDOWN) != !down))
		rc = -EIO;
	if (rc) {
		dev_err(ep->dev,
			"%s port %u: the PHY did not reach %s (%d, BMCR %04x); the gate is owed\n",
			who, port, down ? "power-down" : "power-up", rc, bmcr);
		return rc;
	}
	*bmcr_back = bmcr;
	*mode_back = mode;
	return 0;
}
/* Stock does NOT give the WAN (nas0_0) the LAN MAC: it ...
 * dev/MEASURED-luna_eth.c.md sec 45. */
static unsigned int wan_mac_offset = 3;
module_param(wan_mac_offset, uint, 0644);
MODULE_PARM_DESC(wan_mac_offset, "WAN (gpon0) MAC = board/LAN MAC + this offset (default 3, this model)");

static void luna_chip_wan_mac(struct luna_eth *ep, u8 *out, const u8 *base)
{
	rtl9602c_wan_mac_add(out, base, wan_mac_offset);
}

static int luna_chip_probe(struct luna_eth *ep, struct platform_device *pdev)
{
	/* The board's own WAN MAC offset from its stock (X100DG: LAN+1). */
	of_property_read_u32(pdev->dev.of_node, "realtek,wan-mac-offset", &wan_mac_offset);
	ep->swm = &rtl9602c_sw_map;
	return 0;
}

/* The PON TX descriptor's cpu-tag words: stock's constant steering, the SID one-hot in word3. */
static u32 luna_chip_pon_txd_word2(struct luna_eth *ep, u32 ovr)
{
	return rtl9602c_omci_txd_word2(ovr);
}

static u32 luna_chip_omci_txd_word3(struct luna_eth *ep, u32 ovr)
{
	return rtl9602c_omci_txd_word3(ovr, LUNA_OMCC_SID);
}

static u32 luna_chip_wan_txd_word3(struct luna_eth *ep, unsigned int flow)
{
	return rtl9602c_omci_txd_word3(0, flow);
}

/* This die steers US OMCI by the descriptor's fixed SID; no register latches the OMCC SID. */
static void luna_chip_omci_sid(struct luna_eth *ep, unsigned int sid)
{
}

/* This die's switch does not reflect the CPU's own floods back to it: every frame is delivered. */
/* No per-frame ledger on this die: its RX diagnostics are the shell's counters. */
static void luna_chip_rx_seen(struct luna_eth *ep, const struct rx_desc *d, unsigned int len)
{
}

/* The L34 engine follows the kernel's neighbours and its own WAN netif: nothing to pin or stop here. */
static void luna_chip_wan_pin(struct luna_eth *ep, const u8 *mac)
{
}

static void luna_chip_stop(struct luna_eth *ep)
{
}

static bool luna_chip_rx_consume(struct luna_eth *ep, struct sk_buff *skb, struct net_device *rdev, u32 opts3)
{
	return false;
}

/* A plain LAN frame: the switch floods or forwards it by its own L2 lookup. */
static u32 luna_chip_lan_txd_word2(struct luna_eth *ep, const struct sk_buff *skb)
{
	return 0;
}

/* The fibre port, the measured drain signature (opts3[31:20] == 0x23e) or a unicast to the gpon0 MAC. */
static bool luna_chip_rx_is_wan(struct luna_eth *ep, u32 opts3, const u8 *dst)
{
	return rtl9602c_rx_wan_demux(ep->swm->rx_layout, opts3, dst, ep->wan_ndev->dev_addr, ep->swm->pon_port);
}

static const struct eth_rx_csum_layout luna_rx_csum = RTL9602C_RX_CSUM_LAYOUT;
static const struct eth_rx_csum_layout *const luna_chip_rx_csum = &luna_rx_csum;

/* Brings up the switch L3/L4 NAT engine and its common TC lifecycle (luna_eth.c's probe hook). */
static void luna_chip_offload_probe(struct luna_eth *ep, struct device *dev)
{
	/* Bring up the switch L3/L4 NAT engine (gated by the hw_nat ...
	 * dev/MEASURED-luna_eth.c.md sec 117. */
	/* ★★★ THE DIAGNOSTIC IS REGISTERED WHATEVER THE GATE DOES.  It used to sit
	 * inside the `if (hw_nat)` below, which put the one file that can say WHY
	 * the engine installed nothing behind the very thing it diagnoses: with
	 * the knob at its default 0 the node did not exist, and an absent file
	 * reads as "this image carries no accelerator instrumentation" rather than
	 * "the knob is off".  It is READ-ONLY and it changes no datapath
	 * behaviour; `l34_proc_show()` reports the gate's state and returns before
	 * touching a lock the closed gate never initialised.
	 */
	rtl9602c_l34_proc_init(&ep->cs.l34);

	if (hw_nat) {
		if (rtl9602c_l34_init(&ep->cs.l34, ep->sw)) {
			dev_warn(dev, "L34 hw-nat init failed; software forwarding\n");
		} else {
			dev_info(dev, "L34 hw-nat engine initialised\n");
#ifdef CONFIG_GPON_FLOW_OFFLOAD
			/* The COMMON lifecycle owns the cookie map and the entry; ...
			 * dev/MEASURED-luna_eth.c.md sec 118. */
			ep->cs.fo = gpon_flow_offload_new(&rtl9602c_l34_flow_ops, ep);
			ep->cs.l34_addr_nb.notifier_call = rtl9602c_l34_inetaddr_event;
			if (ep->cs.fo && register_inetaddr_notifier(&ep->cs.l34_addr_nb)) {
				dev_warn(dev, "L34: no address notifier; a WAN that changes address keeps its stale tables\n");
				ep->cs.l34_addr_nb.notifier_call = NULL;
			}
			ep->cs.l34_neigh.names = rtl9602c_l34_neigh_names;
			ep->cs.l34_neigh.sync = rtl9602c_l34_neigh_sync;
			ep->cs.l34_neigh.sync_all = rtl9602c_l34_neigh_sync_all;
			if (ep->cs.fo && eth_neigh_watch_start(&ep->cs.l34_neigh))
				dev_warn(dev, "L34: no neighbour notifier; a moved neighbour keeps its stale rows\n");
			ep->cs.l34_routed_nb.notifier_call = rtl9602c_l34_routed_event;
			if (ep->cs.fo && blocking_notifier_chain_register(&gpon_flow_routed_chain, &ep->cs.l34_routed_nb))
				dev_warn(dev, "L34: no routed-switch notifier; a switch change keeps the old tables\n");
			if (!ep->cs.fo) {
				dev_warn(dev, "L34: the common TC lifecycle could not be created; software forwarding\n");
			} else if (devm_add_action_or_reset(dev, rtl9602c_l34_fo_release,
							   ep)) {
				/* THE HANDLE IS OWNED BY THE DEVICE, not by a .remove this ...
				 * dev/MEASURED-luna_eth.c.md sec 119. */
				dev_warn(dev, "L34: could not bind the TC lifecycle to the device; software forwarding\n");
				ep->cs.fo = NULL;
			}
			/* the flowdump node renders the lifecycle's own diag line */
			ep->cs.l34.fo = ep->cs.fo;
			if (rtl9602c_ipmc_start(ep))
				dev_warn(dev, "L34: no multicast route notifier; the CPU routes every group\n");
			else if (devm_add_action_or_reset(dev, rtl9602c_ipmc_stop, ep))
				dev_warn(dev, "L34: multicast offload not bound to the device; the CPU routes every group\n");
#endif
		}
	}
}

/* The switch core from cold, then the datapath tables the engine reads (luna_eth.c's bring-up hook). */
static int luna_chip_sw_bringup(struct luna_eth *ep)
{
	int ret = rtl9602c_uboot_swcore_bringup(ep);

	if (ret)
		return ret;
	rtl9602c_datapath_tables_init();
	return 0;
}

static const struct of_device_id rtl9602c_eth_of_match[] = {
	{ .compatible = "realtek,rtl9602c-nic" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl9602c_eth_of_match);

static struct platform_driver rtl9602c_eth_driver = {
	.probe	= luna_eth_probe,
	.driver	= {
		.name		= "rtl9602c-eth",
		.of_match_table	= rtl9602c_eth_of_match,
	},
};
module_platform_driver(rtl9602c_eth_driver);

MODULE_DESCRIPTION("Realtek RTL9602C Luna Ethernet driver");
MODULE_LICENSE("GPL");
