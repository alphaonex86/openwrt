/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE, not in the STRICT subset */
/*
 * gpon_olt_pon.h -- each PON link of an OLT PON MAC as a netdev pon<N>, registered through the netdev contract: its
 * carrier is the chip's word on the link (polled), ethtool -S reads the shell's counters. No data path on the CPU
 * yet (the PON's frames cross the board's switch): a frame sent on it is dropped and counted. A link's laser is the
 * device's tx-disable-gpios line of its index, when the board wires one: lit here, as stock lights a PON only once
 * its sender is on, and off again when the device goes.
 */
#ifndef GPON_OLT_PON_H
#define GPON_OLT_PON_H

#include <linux/types.h>

struct device;
struct gpon_olt_mac;

/* What the chip gives its PON netdevs; called in process context, never sleeping long. */
struct gpon_olt_pon_chip {
	/* true when the link's downstream sends and its path to the switch is up */
	bool (*link_up)(void *chip, unsigned int link);
	u32 (*rd)(void *chip, u32 off);
	const u32 *regs;		/* the words ethtool -d dumps, in this order */
	unsigned int n_regs;
};

/* 0, or -EPROBE_DEFER while the board's laser lines have no driver yet: asked before the chip's bring-up, which a
 * deferral at registration would repeat. */
int gpon_olt_pon_lasers_ready(struct device *dev);
/* After gpon_olt_mac_start() with every link's downstream sender on; devm-managed under @dev; @pc outlives it. */
int gpon_olt_pon_register(struct device *dev, struct gpon_olt_mac *mac, const struct gpon_olt_pon_chip *pc);

#endif /* GPON_OLT_PON_H */
