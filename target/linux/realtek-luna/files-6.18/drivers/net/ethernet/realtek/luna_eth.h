/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: FAMILY -- the head of the Luna GMAC0 Ethernet shell (luna_eth.c): what a chip TU needs before it
 * defines its own state, and the row type (struct luna_eth_chip) through which the shell reaches the chip. */
#ifndef LUNA_ETH_H
#define LUNA_ETH_H

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/eth_netdev_contract.h>
#include "eth_pon_link.h"	/* the PON WAN netdev's ethtool link */
#include "eth_neigh_watch.h"	/* the engine's rows follow the kernel neighbours they copied */
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_address.h>	/* the DRAM the device tree declares: the RX window's floor */
#include <linux/io.h>
#include <linux/mii.h>	/* BMCR_* -- the standard MII bit names */
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "eth_rx_csum.h"
#include "eth_rx_window.h"	/* the RX ring's RAM follows its use */
#include "luna_eth_regs.h"	/* the family MAC/switch register map + per-chip table */
#include "gpon_omci_trace.h"	/* G.988 decode-to-a-buffer for the log */
#include "gpon_omci_diag.h"
#include "gpon_omci_me.h"	/* the common OMCI ME store + context */
#include "luna_gpon_nic.h"
#include "luna_wan_vlan.h"
#include "luna_eth_mib.h"
#include "eth_chip_patch.h"
#include "luna_loop.h"		/* LAN loop detection, the switch engine on this die */
#include "luna_gmac_logic.h"	/* family GMAC ring packings (flowcore) */
#include "gpon_hwaddr.h"	/* the ONE station-address ladder (drivers/net/gpon) */
#include "eth_core_recovery.h"
#include "luna_sw_state.h"
#include <linux/rtnetlink.h>

struct luna_eth;

/* The GMAC values that differ between the dies; each chip TU defines its luna_chip_gmac before luna_eth.c. */
struct luna_eth_gmac {
	const char *name;	/* the die, as the probe line names it */
	u8 cmd;			/* R_CMD at program */
	u32 tcr;		/* R_TCR at program */
	bool rxokmul_seed;	/* R_RXOKMUL takes stock's O5 snapshot */
	bool tx_dummy_rings;	/* the unused HW TX rings idle on a parked ring */
	bool uboot_inherit;	/* gmac_reset=0 may keep the bootloader's GMAC */
	bool memctl;		/* the MEMCTL TX-GO handshake exists */
	bool pbo_reinit;	/* a GMAC reset owes the PON buffer re-init */
	bool wan_ctrl_flow;	/* luna_wan_tx_flow()'s ctrl_flow */
};

/* The chip hooks: the TU that includes luna_eth.c defines every one (a missing one fails the build). */
static int luna_chip_sw_bringup(struct luna_eth *ep);
static void luna_chip_cpu_tag_arm(struct luna_eth *ep);
static void luna_chip_sw_min_init(struct luna_eth *ep);
static int luna_chip_port_gate__locked(struct luna_eth *ep, unsigned int port, bool down, bool mac_suffices,
				       const char *who, u16 *bmcr_back, u32 *mode_back);
static void luna_chip_offload_probe(struct luna_eth *ep, struct device *dev);
static void luna_chip_offload_lock(struct luna_eth *ep);
static void luna_chip_offload_unlock(struct luna_eth *ep);
static void luna_chip_offload_assert_held(struct luna_eth *ep);
static void luna_chip_wan_mac(struct luna_eth *ep, u8 *out, const u8 *base);
static int luna_chip_probe(struct luna_eth *ep, struct platform_device *pdev);
static u32 luna_chip_pon_txd_word2(struct luna_eth *ep, u32 ovr);
static u32 luna_chip_omci_txd_word3(struct luna_eth *ep, u32 ovr);
static u32 luna_chip_wan_txd_word3(struct luna_eth *ep, unsigned int flow);
static void luna_chip_omci_sid(struct luna_eth *ep, unsigned int sid);
static void luna_chip_rx_seen(struct luna_eth *ep, const struct rx_desc *d, unsigned int len);
static bool luna_chip_rx_consume(struct luna_eth *ep, struct sk_buff *skb, struct net_device *rdev, u32 opts3);
static u32 luna_chip_lan_txd_word2(struct luna_eth *ep, const struct sk_buff *skb);
/* -> true: a received frame belongs to gpon0. Per die: what marks a fibre ingress is a measured fact of each one. */
static bool luna_chip_rx_is_wan(struct luna_eth *ep, u32 opts3, const u8 *dst);
/* The RX opts1 checksum layout this die is proven to report; NULL = no RX checksum offload. */
static const struct eth_rx_csum_layout *const luna_chip_rx_csum;
static void luna_chip_wan_pin(struct luna_eth *ep, const u8 *mac);
static void luna_chip_stop(struct luna_eth *ep);
static unsigned int luna_chip_sw_blocks(void *ctx, struct eth_state_block *b);
static const struct luna_phy_ops luna_chip_phy_ops;
#ifdef CONFIG_GPON_FLOW_OFFLOAD
static int luna_chip_setup_tc(struct net_device *dev, enum tc_setup_type type, void *type_data);
#endif
/* Optional: a TU with a WAN multicast filter defines LUNA_CHIP_WAN_RX_MODE to its ndo_set_rx_mode. */

#endif /* LUNA_ETH_H */
