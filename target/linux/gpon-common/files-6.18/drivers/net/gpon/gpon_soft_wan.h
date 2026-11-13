/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE, not in the STRICT subset */
/* gpon_soft_wan.h -- gpon0 for a shell whose data GEM port reaches the CPU as whole frames: the service the OLT
 * provisioned (VLAN, GEM port, Alloc-ID, upstream PIR) re-read from the model, its VLAN pushed upstream and popped
 * downstream, the PIR kept as a token bucket. The shell sends and delivers frames and writes none of this.
 */
#ifndef GPON_SOFT_WAN_H
#define GPON_SOFT_WAN_H

#include <linux/spinlock.h>

#include "gpon_omci_me.h"

struct net_device;

struct gpon_soft_wan {
	spinlock_t *lock;	/* the shell's model lock */
	void *sh;
	/* One upstream frame, its service VLAN already pushed, for svc->alloc_id and svc->gem_port. Called without
	 * @lock. 0 = sent. */
	int (*tx)(void *sh, const struct omci_vlan_service *svc, const u8 *frame, unsigned int len);

	struct net_device *dev;
	struct omci_vlan_service svc;
	bool up;
	struct omci_meter meter;
	u32 us_pir, us_burst, us_tokens;	/* ME 280 upstream: bytes/s, bytes, bytes */
	unsigned long us_stamp;
};

/* @w zeroed but @lock, @sh and @tx; @mac NULL = a random address. Registers gpon0, carrier off. */
int gpon_soft_wan_init(struct gpon_soft_wan *w, struct omci_onu *onu, const u8 *mac);
void gpon_soft_wan_exit(struct gpon_soft_wan *w);
/* Under @lock, every tick: the service re-read; the carrier follows it. */
void gpon_soft_wan_poll(struct gpon_soft_wan *w, const struct omci_onu *onu, bool omcc_up, u16 omcc_gem);
/* Under @lock: one downstream frame of GEM port @gem. */
void gpon_soft_wan_rx(struct gpon_soft_wan *w, u16 gem, const u8 *frame, unsigned int len);

#endif
