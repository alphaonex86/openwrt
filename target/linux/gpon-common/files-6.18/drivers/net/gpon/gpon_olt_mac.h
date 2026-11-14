/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE, not in the STRICT subset */
/*
 * gpon_olt_mac.h -- the shell every OLT PON MAC driver shares: one lock and one tick per chip, a gpon_olt_port per
 * PON link fed from the chip's upstream and drained to its downstream, the serial-number window held open for one
 * tick, the counters, /proc/<name>, and the stop order (tick, then the chip). The chip file keeps registers, tables
 * and its bring-up and gives the ops below; it holds no protocol, list or timer of its own.
 */
#ifndef GPON_OLT_MAC_H
#define GPON_OLT_MAC_H

#include <linux/spinlock.h>
#include <linux/timer.h>

#include "gpon_olt_port.h"

struct device;

/* Called under the shell's lock (softirq context, never sleeping); @link < n_links. */
struct gpon_olt_mac_ops {
	/* 0, or -EBUSY while the link's sender still holds the previous message; the chip adds the CRC */
	int (*ploam_tx)(void *chip, unsigned int link, const u8 msg[GPON_OLT_PLOAM_DS_LEN]);
	/* the chip grants serial-number requests while open; one that grants once per call ignores a close */
	int (*sn_window)(void *chip, unsigned int link, bool open);
	/* one ranging grant to the default Alloc-ID of @onu_id (its ONU-ID) */
	int (*ranging_grant)(void *chip, unsigned int link, u8 onu_id);
	/* 0, -EBUSY when the downstream queue is full */
	int (*omci_tx)(void *chip, unsigned int link, u16 gem_port, const u8 *pdu, unsigned int len);
	/* Polled each tick; NULL for a chip that pushes its upstream through gpon_olt_mac_us_*() instead.
	 * -> true with the oldest upstream PLOAM taken off the chip and the round trip measured with it */
	bool (*ploam_rx)(void *chip, unsigned int link, u8 msg[GPON_OLT_PLOAM_US_LEN], u32 *rtd);
	/* -> 1 with one upstream OMCI message, 0 when none is held, -EPROTO for a frame of another kind (taken) */
	int (*omci_rx)(void *chip, unsigned int *link, u16 *gem_port, u8 pdu[GPON_OLT_OMCI_LEN]);
	/* the chip silent: interrupts and queues off; nothing is called after it */
	void (*stop)(void *chip);
};

struct gpon_olt_mac;

struct gpon_olt_mac_link {
	struct gpon_olt_mac *mac;
	unsigned int link;
	struct gpon_olt_port port;
	bool sn_open;
	u64 ds_ploam, ploam_busy, us_ploam, ds_omci, omci_full, us_omci, grant_fail;
};

struct gpon_olt_mac {
	const struct gpon_olt_mac_ops *ops;
	void *chip;
	const char *name;		/* the driver's: log prefix and /proc/<name> */
	spinlock_t lock;		/* the core is single-threaded: every call into it holds this */
	struct timer_list tick;
	struct gpon_olt_mac_link *links;
	unsigned int n_links;
	u64 rx_other;
	bool running;
};

/* @mac zeroed. n_onus OMCI engines per link, allocated under @dev; @tmpl the service every ONU gets
 * (gpon_olt_port_init). Starts the tick and /proc/@name. */
int gpon_olt_mac_start(struct gpon_olt_mac *mac, struct device *dev, const char *name,
		       const struct gpon_olt_mac_ops *ops, void *chip, unsigned int n_links, unsigned int n_onus,
		       const struct gpon_olt_profile *tmpl);
/* The tick stopped and /proc removed, then ops->stop: no chip access after it (at reboot the bus resets next). */
void gpon_olt_mac_stop(struct gpon_olt_mac *mac);
/* A pushing chip's upstream, from softirq or process context only (a NAPI poll, a threaded handler): the lock is
 * taken with spin_lock_bh, so a hard interrupt handler defers to one of those. */
void gpon_olt_mac_us_ploam(struct gpon_olt_mac *mac, unsigned int link, const u8 msg[GPON_OLT_PLOAM_US_LEN],
			   u32 rtd);
void gpon_olt_mac_us_omci(struct gpon_olt_mac *mac, unsigned int link, u16 gem_port,
			  const u8 pdu[GPON_OLT_OMCI_LEN]);

#endif /* GPON_OLT_MAC_H */
