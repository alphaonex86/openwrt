// SPDX-License-Identifier: GPL-2.0-or-later
/* gpon_olt_platform.c -- the OLT hardware contract, checked (see the header). */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "gpon_olt_platform.h"

const char *gpon_olt_platform_ops_missing(const struct gpon_olt_platform_ops *ops)
{
	if (!ops)
		return "ops";
#define GPON_OLT_PLATFORM_CHECK_ONE(name)	\
	if (!ops->name)				\
		return #name;
	GPON_OLT_PLATFORM_OPS_MANDATORY(GPON_OLT_PLATFORM_CHECK_ONE)
#undef GPON_OLT_PLATFORM_CHECK_ONE
	return NULL;
}

/* Every ONU needs its OMCC GEM port, one data GEM port and one T-CONT. */
const char *gpon_olt_capacity_flaw(const struct gpon_olt_capacity *cap)
{
	if (!cap)
		return "capacity";
	if (!cap->onus || cap->onus > GPON_OLT_PON_ONUS)
		return "onus";
	if (cap->gem_ports < 2u * cap->onus)
		return "gem_ports";
	if (cap->alloc_ids < cap->onus)
		return "alloc_ids";
	if (!cap->ds_mbps || !cap->us_mbps)
		return "line_rate";
	if (!cap->hw_datapath)
		return "hw_datapath";
	if (cap->fwd_mbps < cap->ds_mbps + cap->us_mbps)
		return "fwd_mbps";
	return NULL;
}

const char *gpon_olt_platform_register(struct gpon_olt_platform *p,
				       const struct gpon_olt_platform_ops *ops,
				       const struct gpon_olt_capacity *cap,
				       void *hw)
{
	const char *flaw = gpon_olt_platform_ops_missing(ops);

	memset(p, 0, sizeof(*p));
	if (!flaw)
		flaw = gpon_olt_capacity_flaw(cap);
	if (flaw)
		return flaw;
	p->ops = ops;
	p->cap = *cap;
	p->hw = hw;
	return NULL;
}

static bool onu_ok(const struct gpon_olt_platform *p, u8 onu_id)
{
	return onu_id >= 1 && onu_id <= p->cap.onus;
}

int gpon_olt_platform_start(struct gpon_olt_platform *p)
{
	return p->ops->mac_start(p->hw);
}

void gpon_olt_platform_stop(struct gpon_olt_platform *p)
{
	p->ops->mac_stop(p->hw);
}

/* Grants in ascending order, never overlapping, inside the upstream frame. */
int gpon_olt_platform_bwmap(struct gpon_olt_platform *p,
			    const struct gpon_olt_grant *g, unsigned int n)
{
	u32 frame_bytes = p->cap.us_mbps * 125u / 8u;
	u32 next = 0;
	unsigned int i;

	if (n > p->cap.alloc_ids)
		return -ENOSPC;
	for (i = 0; i < n; i++) {
		if (g[i].alloc_id > GPON_OLT_ID_MAX || g[i].sstart < next ||
		    g[i].sstop < g[i].sstart || g[i].sstop >= frame_bytes)
			return -EINVAL;
		next = g[i].sstop + 1u;
	}
	return p->ops->bwmap_set(p->hw, g, n);
}

int gpon_olt_platform_gem_bind(struct gpon_olt_platform *p, u8 onu_id,
			       u16 gem_port, u16 alloc_id, u16 uplink_vid)
{
	int rc;

	if (!onu_ok(p, onu_id) || gem_port > GPON_OLT_ID_MAX ||
	    alloc_id > GPON_OLT_ID_MAX || uplink_vid > 4094)
		return -EINVAL;
	if (p->gems_bound >= p->cap.gem_ports)
		return -ENOSPC;
	rc = p->ops->gem_bind(p->hw, onu_id, gem_port, alloc_id, uplink_vid);
	if (!rc)
		p->gems_bound++;
	return rc;
}

int gpon_olt_platform_gem_unbind(struct gpon_olt_platform *p, u16 gem_port)
{
	int rc;

	if (gem_port > GPON_OLT_ID_MAX || !p->gems_bound)
		return -EINVAL;
	rc = p->ops->gem_unbind(p->hw, gem_port);
	if (!rc)
		p->gems_bound--;
	return rc;
}

int gpon_olt_platform_queue(struct gpon_olt_platform *p, u8 onu_id,
			    u32 cir_kbps, u32 pir_kbps)
{
	if (!onu_ok(p, onu_id) || cir_kbps > pir_kbps ||
	    pir_kbps > p->cap.us_mbps * 1000u)
		return -EINVAL;
	return p->ops->onu_queue_set(p->hw, onu_id, cir_kbps, pir_kbps);
}

int gpon_olt_platform_optics(struct gpon_olt_platform *p, u8 onu_id,
			     struct gpon_olt_optics *o)
{
	if (onu_id && !onu_ok(p, onu_id))
		return -EINVAL;
	return p->ops->optics_read(p->hw, onu_id, o);
}

int gpon_olt_platform_counters(struct gpon_olt_platform *p, u8 onu_id,
			       struct gpon_olt_counters *c)
{
	if (onu_id && !onu_ok(p, onu_id))
		return -EINVAL;
	return p->ops->counters_read(p->hw, onu_id, c);
}
