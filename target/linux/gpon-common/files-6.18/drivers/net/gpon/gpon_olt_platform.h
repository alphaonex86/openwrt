/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gpon_olt_platform.h -- what every OLT hardware port must provide: the PON
 * MAC, the upstream grant map (DBA), GEM <-> uplink forwarding in hardware, a
 * queue per ONU, optics and counters, and the capacity it declares.  C has no
 * pure virtual: gpon_olt_platform_register() refuses a NULL op, or a capacity
 * that cannot carry every ONU at line rate in hardware, before installing
 * anything.  The PLOAM wire stays in gpon_olt_ploam_ops.
 *
 * TIER: CORE.  No MMIO, lock, allocator, sleep or clock.
 */
#ifndef GPON_OLT_PLATFORM_H
#define GPON_OLT_PLATFORM_H

#include <linux/types.h>

#include "gpon_common.h"
#include "gpon_olt_ploam.h"
#include "gpon_ploam.h"		/* GPON_MUST_CHECK */

#define GPON_OLT_ID_MAX		4095	/* 12-bit Port-ID and Alloc-ID (G.984.3) */

/* One allocation structure of the downstream BWmap (G.984.3 8.1.3.6). */
struct gpon_olt_grant {
	u16 alloc_id;
	u16 flags;
	u16 sstart;		/* bytes into the upstream frame */
	u16 sstop;
};

/* onu_id 0 is the port itself: its laser and temperature. */
struct gpon_olt_optics {
	s16 tx_dbm_x100;
	s16 rx_dbm_x100;	/* the ONU's last burst */
	s16 temp_c_x100;
};

struct gpon_olt_counters {
	u64 ds_bytes;
	u64 us_bytes;
	u64 ds_frames;
	u64 us_frames;
	u64 dropped;
	u64 bip_errors;
	u64 fec_corrected;
};

/* Declared per PON port by the port, checked by gpon_olt_capacity_flaw(). */
struct gpon_olt_capacity {
	u16 onus;
	u16 gem_ports;
	u16 alloc_ids;
	u32 ds_mbps;		/* line rate, 2488 for G.984 */
	u32 us_mbps;		/* 1244 */
	u32 fwd_mbps;		/* GEM <-> uplink forwarded by hardware, both ways */
	bool hw_datapath;	/* no frame of the data path through the CPU */
};

#define GPON_OLT_PLATFORM_OPS_MANDATORY(X)	\
	X(mac_start)				\
	X(mac_stop)				\
	X(bwmap_set)				\
	X(gem_bind)				\
	X(gem_unbind)				\
	X(onu_queue_set)			\
	X(optics_read)				\
	X(counters_read)

struct gpon_olt_platform_ops {
	int (*mac_start)(void *hw);
	void (*mac_stop)(void *hw);
	int (*bwmap_set)(void *hw, const struct gpon_olt_grant *g, unsigned int n);
	int (*gem_bind)(void *hw, u8 onu_id, u16 gem_port, u16 alloc_id, u16 uplink_vid);
	int (*gem_unbind)(void *hw, u16 gem_port);
	int (*onu_queue_set)(void *hw, u8 onu_id, u32 cir_kbps, u32 pir_kbps);
	int (*optics_read)(void *hw, u8 onu_id, struct gpon_olt_optics *o);
	int (*counters_read)(void *hw, u8 onu_id, struct gpon_olt_counters *c);
};

struct gpon_olt_platform {
	const struct gpon_olt_platform_ops *ops;
	struct gpon_olt_capacity cap;
	void *hw;
	u16 gems_bound;
};

const char *gpon_olt_platform_ops_missing(const struct gpon_olt_platform_ops *ops);
const char *gpon_olt_capacity_flaw(const struct gpon_olt_capacity *cap);
GPON_MUST_CHECK const char *gpon_olt_platform_register(struct gpon_olt_platform *p,
							const struct gpon_olt_platform_ops *ops,
							const struct gpon_olt_capacity *cap,
							void *hw);

int gpon_olt_platform_start(struct gpon_olt_platform *p);
void gpon_olt_platform_stop(struct gpon_olt_platform *p);
int gpon_olt_platform_bwmap(struct gpon_olt_platform *p,
			    const struct gpon_olt_grant *g, unsigned int n);
int gpon_olt_platform_gem_bind(struct gpon_olt_platform *p, u8 onu_id,
			       u16 gem_port, u16 alloc_id, u16 uplink_vid);
int gpon_olt_platform_gem_unbind(struct gpon_olt_platform *p, u16 gem_port);
int gpon_olt_platform_queue(struct gpon_olt_platform *p, u8 onu_id,
			    u32 cir_kbps, u32 pir_kbps);
int gpon_olt_platform_optics(struct gpon_olt_platform *p, u8 onu_id,
			     struct gpon_olt_optics *o);
int gpon_olt_platform_counters(struct gpon_olt_platform *p, u8 onu_id,
			       struct gpon_olt_counters *c);

#endif /* GPON_OLT_PLATFORM_H */
