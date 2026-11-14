/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE */
/*
 * gpon_olt_port.h -- one PON port of an OLT, the glue every OLT PON MAC shares: activation (gpon_olt_pon), one
 * OMCI engine per ONU started and stopped with its link, zero-touch authorization, and which ONU a GEM port
 * belongs to.  A PON MAC (virtual or silicon) feeds it the upstream and carries its downstream through the ops;
 * slot k serves ONU-ID k + 1, its T-CONT alloc_base + ONU-ID and its data GEM port gem_base + ONU-ID.
 *
 * TIER: CORE.  No MMIO, lock, allocator, sleep or clock: the shell owns the engines' memory and the time.
 */
#ifndef GPON_OLT_PORT_H
#define GPON_OLT_PORT_H

#include <linux/types.h>

#include "gpon_olt_omci.h"
#include "gpon_olt_ploam.h"

struct gpon_olt_port_ops {
	void (*ds_ploam)(void *sh, const u8 msg[GPON_OLT_PLOAM_DS_LEN]);
	void (*sn_grant)(void *sh);
	void (*ranging_grant)(void *sh, u8 onu_id);
	int (*ds_omci)(void *sh, u16 gem_port, const u8 *pdu, unsigned int len);
	/* optional: a link change or an OMCI phase change, for the shell's log */
	void (*event)(void *sh, u8 onu_id, const char *what, const char *detail);
	gpon_unsup_fn unsupported;
};

struct gpon_olt_port {
	const struct gpon_olt_port_ops *ops;
	void *sh;
	struct gpon_olt_pon pon;
	struct gpon_olt_omci *eng;	/* n_eng engines, slot k = ONU-ID k + 1 */
	unsigned int n_eng;
	struct gpon_olt_profile tmpl;	/* the service every ONU gets; its alloc_id and gem_port are the bases */
	bool zero_touch;		/* authorize every serial heard (the operator's production flow) */
	u8 phase_seen[GPON_OLT_PON_ONUS];
};

/* -> NULL, or the op or argument it refuses. */
const char *gpon_olt_port_init(struct gpon_olt_port *port, const struct gpon_olt_port_ops *ops, void *sh,
			       struct gpon_olt_omci *eng, unsigned int n_eng,
			       const struct gpon_olt_profile *tmpl, bool zero_touch);
/* -> the ONU-ID given to @sn, or a negative errno. */
int gpon_olt_port_authorize(struct gpon_olt_port *port, const u8 sn[8], const u8 *password);
void gpon_olt_port_us_ploam(struct gpon_olt_port *port, const u8 *m, unsigned int len, u32 now_ms);
void gpon_olt_port_ranging_burst(struct gpon_olt_port *port, u8 onu_id, u32 rtd, u32 now_ms);
void gpon_olt_port_us_omci(struct gpon_olt_port *port, u16 gem_port, const u8 *pdu, unsigned int len,
			   u32 now_ms);
void gpon_olt_port_tick(struct gpon_olt_port *port, u32 now_ms);
/* -> the slot whose data GEM port is @gem_port and whose ONU is in service, or -1. */
int gpon_olt_port_slot_of_gem(const struct gpon_olt_port *port, u16 gem_port);
bool gpon_olt_port_online(const struct gpon_olt_port *port, unsigned int slot);
/* -> the length of @slot's line in an OLT PON MAC's ONU table (one format for every MAC, newline included),
 * 0 for a slot never heard; the line is cut to @size. */
int gpon_olt_port_show(const struct gpon_olt_port *port, unsigned int slot, char *buf, size_t size);

static inline u16 gpon_olt_port_gem(const struct gpon_olt_port *port, unsigned int slot)
{
	return (u16)(port->tmpl.gem_port + gpon_olt_onu_id(slot));
}

#endif /* GPON_OLT_PORT_H */
