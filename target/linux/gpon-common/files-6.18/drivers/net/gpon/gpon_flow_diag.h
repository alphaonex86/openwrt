/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE, strict host-buildable subset.  CONFIG_GPON_FLOW_DIAG, the fourth
 * member of the CONFIG_GPON_<SUBSYSTEM>_DIAG family -- why a flow was NOT
 * accelerated.  Evidence: dev/MEASURED-gpon_flow_diag.h.md sec 1. */
#ifndef GPON_FLOW_DIAG_H
#define GPON_FLOW_DIAG_H

#include <linux/types.h>

/* WHY a TC replace did not end up in the engine.  A CLOSED set: every refusal
 * point in gpon_flow.c and gpon_flow_offload.c names exactly one of these, and
 * the three errnos they collapse into cannot tell them apart --
 * dev/MEASURED-gpon_flow_diag.h.md sec 2. */
enum gpon_flow_refusal {
	GPON_FLOW_OK = 0,		/* installed                            */
	GPON_FLOW_REF_NO_ENGINE,	/* no lifecycle / table not ready       */
	GPON_FLOW_REF_DUP_COOKIE,	/* the other leg already holds it       */
	GPON_FLOW_REF_BAD_ARGS,		/* the caller handed us nothing to read */
	GPON_FLOW_REF_NOT_IPV4,		/* the 5-tuple is not IPv4              */
	GPON_FLOW_REF_NO_BASIC,		/* no BASIC dissector: no L4 proto      */
	GPON_FLOW_REF_NOT_TCP_UDP,	/* L4 proto the engine cannot key on    */
	GPON_FLOW_REF_NO_PORTS,		/* no PORTS dissector                   */
	GPON_FLOW_REF_NAT_OFFSET,	/* the IP mangle is not this leg's      */
	GPON_FLOW_REF_PORT_SHAPE,	/* the L4 port mangle is not this leg's */
	GPON_FLOW_REF_MANGLE_HTYPE,	/* a header class we do not rewrite     */
	GPON_FLOW_REF_VLAN_ACTION,	/* a tag push/pop: no engine here can   */
	GPON_FLOW_REF_ACTION_ID,	/* an action verb we do not implement   */
	GPON_FLOW_REF_INCOMPLETE,	/* redirect / NAT / port: one missing   */
	GPON_FLOW_REF_NOMEM,
	GPON_FLOW_REF_ENGINE,		/* the family's install() said no       */
	GPON_FLOW_REF_TABLE_INSERT,	/* the cookie map refused the entry     */
	GPON_FLOW_REF__COUNT
};

/* Never NULL, "?" for an out-of-enum value. */
const char *gpon_flow_refusal_name(enum gpon_flow_refusal r);

/* Record one outcome.  NULL-tolerant so a build with the flag off passes NULL
 * and the compiler drops the call. */
struct gpon_flow_tally {
	u32 n[GPON_FLOW_REF__COUNT];
	enum gpon_flow_refusal last;
};

void gpon_flow_tally_note(struct gpon_flow_tally *t, enum gpon_flow_refusal r);

/* What only the FAMILY can read: its own offer count and its engine's state.
 * A clear bit is COULD NOT ASK and renders n/a -- never 0, because "0 entries
 * live" is a device finding and "we never asked" is not --
 * dev/MEASURED-gpon_flow_diag.h.md sec 3. */
#define GPON_FDIAG_HAS_OFFERED		0x01u
#define GPON_FDIAG_HAS_LIVE		0x02u
#define GPON_FDIAG_HAS_CAPACITY		0x04u	/* a per-CHIP table fact */
#define GPON_FDIAG_HAS_HITS		0x08u
#define GPON_FDIAG_HAS_ALL		0x0fu

struct gpon_flow_diag {
	u32 valid;
	u32 offered;	/* TC replace requests that reached the family   */
	u32 live;	/* entries the engine holds right now            */
	u32 capacity;	/* the chip's table size, from its own table     */
	u32 hw_hits;	/* the hit witness, in the family's own units    */
};

/* ONE line, fields in a fixed order so a reader can grep it.  Only the causes
 * that FIRED are listed, so a healthy board prints a short line and this cannot
 * flood -- dev/MEASURED-gpon_flow_diag.h.md sec 4. */
int gpon_flow_diag_line(const struct gpon_flow_tally *t,
			const struct gpon_flow_diag *d, char *out, size_t sz);

/* The gate itself, so the lifecycle carries no #if of its own and the host
 * fixtures that extract its bodies see the same two spellings.  With the flag
 * off the struct loses the member and every note compiles away. */
#if IS_ENABLED(CONFIG_GPON_FLOW_DIAG)
#define GPON_FLOW_TALLY_FIELD	struct gpon_flow_tally tally;
#define GPON_FLOW_NOTE(fo, r)	gpon_flow_tally_note(&(fo)->tally, (r))
#else
#define GPON_FLOW_TALLY_FIELD
#define GPON_FLOW_NOTE(fo, r)	do { } while (0)
#endif

#endif /* GPON_FLOW_DIAG_H */
