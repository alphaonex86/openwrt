// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: CORE, strict host-buildable subset.  See gpon_ploam_diag.h for what
 * this is and why a missing arm must render as `n/a` and never as 0.
 */
#include <linux/kernel.h>
#include <linux/types.h>

#include "gpon_ploam_diag.h"

enum gpon_ploam_diag_point gpon_ploam_diag_point_of(enum gpon_ploam_ev ev)
{
	switch (ev) {
	case GPON_PLOAM_EV_ONU_ID:
		return GPON_PDIAG_ASSIGN;
	case GPON_PLOAM_EV_RANGING_TIME:
		return GPON_PDIAG_RANGING_TIME;
	case GPON_PLOAM_EV_DEACT:
		return GPON_PDIAG_DEACT;
	default:
		return GPON_PDIAG_NONE;
	}
}

const char *gpon_ploam_diag_point_name(enum gpon_ploam_diag_point p)
{
	switch (p) {
	case GPON_PDIAG_ASSIGN:
		return "assign";
	case GPON_PDIAG_RANGING_TIME:
		return "ranging_time";
	case GPON_PDIAG_DEACT:
		return "deact";
	default:
		return "?";
	}
}

/* One `name=value` or `name=n/a` field, appended at `*pos`.  The buffer is
 * the caller's; scnprintf never writes past it, and a full buffer simply
 * stops appending -- a clipped line is still a true line. */
static void field_u32(char *out, size_t sz, int *pos, const char *name,
		      bool has, u32 v)
{
	if ((size_t)*pos >= sz)
		return;
	if (has)
		*pos += scnprintf(out + *pos, sz - *pos, " %s=%u", name, v);
	else
		*pos += scnprintf(out + *pos, sz - *pos, " %s=n/a", name);
}

int gpon_ploam_diag_format(char *out, size_t sz, enum gpon_ploam_diag_point p,
			   u32 t_ms, u8 ostate, const struct gpon_ploam_diag *d)
{
	int pos;

	if (!out || !sz)
		return 0;
	pos = scnprintf(out, sz, "ploam-diag %s t=%ums O%u",
			gpon_ploam_diag_point_name(p), t_ms, ostate);
	if (!d) {
		/* No readings at all is a valid report: every arm is n/a. */
		static const struct gpon_ploam_diag none = { .valid = 0 };

		d = &none;
	}
	field_u32(out, sz, &pos, "ploam_acpt",
		  !!(d->valid & GPON_PDIAG_HAS_PLOAM_ACPT), d->ploam_acpt);
	field_u32(out, sz, &pos, "bwm_acpt",
		  !!(d->valid & GPON_PDIAG_HAS_BWM_ACPT), d->bwm_acpt);
	field_u32(out, sz, &pos, "bwm_fail",
		  !!(d->valid & GPON_PDIAG_HAS_BWM_FAIL), d->bwm_fail);
	field_u32(out, sz, &pos, "bwm_inv",
		  !!(d->valid & GPON_PDIAG_HAS_BWM_INV), d->bwm_inv);
	field_u32(out, sz, &pos, "us_ploam_tx",
		  !!(d->valid & GPON_PDIAG_HAS_US_PLOAM_TX), d->us_ploam_tx);
	field_u32(out, sz, &pos, "us_onu_id",
		  !!(d->valid & GPON_PDIAG_HAS_US_ONU_ID), d->us_onu_id);
	field_u32(out, sz, &pos, "ds_onu_id",
		  !!(d->valid & GPON_PDIAG_HAS_DS_ONU_ID), d->ds_onu_id);
	return pos;
}
