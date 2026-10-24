/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gpon_olt_ploam.h -- the OLT role of G.984.3 activation for every ONU of one
 * PON: serial-number acquisition, ONU-ID assignment, ranging, password check
 * and the OMCC set-up (Configure_Port-ID, Assign_Alloc-ID), with timeouts.
 *
 * TIER: CORE.  No MMIO, lock, allocator, sleep or clock: time is an input and
 * the shell owns the MAC (grant windows, burst timing) and the wire.
 */
#ifndef GPON_OLT_PLOAM_H
#define GPON_OLT_PLOAM_H

#include <linux/types.h>

#include "gpon_common.h"

#define GPON_OLT_PON_ONUS	32
#define GPON_OLT_PLOAM_DS_LEN	13
#define GPON_OLT_PLOAM_US_LEN	12
#define GPON_OLT_PASSWORD_LEN	10

/* Upstream message types (G.984.3 clause 9.2.4). */
#define GPON_OLT_US_SN		0x01
#define GPON_OLT_US_PASSWORD	0x02
#define GPON_OLT_US_DYING_GASP	0x03
#define GPON_OLT_US_NO_MESSAGE	0x04
#define GPON_OLT_US_KEY		0x05
#define GPON_OLT_US_REI		0x08
#define GPON_OLT_US_ACK		0x09

/* Downstream message types (G.984.3 clause 9.2.3). */
#define GPON_OLT_DS_OVERHEAD	0x01
#define GPON_OLT_DS_ASSIGN_ID	0x03
#define GPON_OLT_DS_RANGING	0x04
#define GPON_OLT_DS_DEACTIVATE	0x05
#define GPON_OLT_DS_DISABLE_SN	0x06
#define GPON_OLT_DS_PASSWORD	0x09
#define GPON_OLT_DS_ALLOC	0x0a
#define GPON_OLT_DS_PORT	0x0e

enum gpon_olt_link {
	GPON_OLT_LINK_FREE,
	GPON_OLT_LINK_AUTOFIND,		/* serial heard, not authorized */
	GPON_OLT_LINK_RANGING,		/* ONU-ID assigned, ranging owed */
	GPON_OLT_LINK_PASSWORD,		/* ranged, password owed */
	GPON_OLT_LINK_OMCC,		/* acknowledgements of the OMCC set-up owed */
	GPON_OLT_LINK_UP,		/* operational, OMCC up */
	GPON_OLT_LINK_REJECTED,		/* deactivated by policy */
	GPON_OLT_LINK_LOST,		/* was assigned, then dropped */
	GPON_OLT_LINK_DISABLED,		/* serial disabled (blacklist) */
};

enum gpon_olt_down {
	GPON_OLT_DOWN_NONE,
	GPON_OLT_DOWN_RANGING_TIMEOUT,
	GPON_OLT_DOWN_PASSWORD_TIMEOUT,
	GPON_OLT_DOWN_PASSWORD_MISMATCH,
	GPON_OLT_DOWN_ACK_TIMEOUT,
	GPON_OLT_DOWN_LOS,
	GPON_OLT_DOWN_DYING_GASP,
	GPON_OLT_DOWN_DEACTIVATED,
	GPON_OLT_DOWN_DISABLED,
};

struct gpon_olt_ploam_ops {
	void (*ds_tx)(void *sh, const u8 msg[GPON_OLT_PLOAM_DS_LEN]);
	void (*sn_grant)(void *sh);
	void (*ranging_grant)(void *sh, u8 onu_id);
	void (*link_change)(void *sh, u8 onu_id, enum gpon_olt_link link,
			    enum gpon_olt_down why);
	gpon_unsup_fn unsupported;
};

struct gpon_olt_link_rec {
	u8 sn[8];
	u8 password[GPON_OLT_PASSWORD_LEN];
	bool used;
	bool has_password;
	bool authorized;
	u8 state;		/* enum gpon_olt_link */
	u8 down;		/* enum gpon_olt_down */
	u8 tries;
	u8 acks_owed;		/* GPON_OLT_OWE_* */
	u8 sent_port[GPON_OLT_PLOAM_DS_LEN];
	u8 sent_alloc[GPON_OLT_PLOAM_DS_LEN];
	u32 since_ms;
	u32 rtd;
	u32 eqd;
	u16 omcc_gem;
	u16 data_alloc;
	u32 ups;
	u32 losses;
};

#define GPON_OLT_OWE_PORT	0x01
#define GPON_OLT_OWE_ALLOC	0x02

/* G.984.3 9.2.3.1 Upstream_Overhead, as broadcast during acquisition. */
struct gpon_olt_overhead {
	u8 guard_bits;
	u8 preamble1_bits;
	u8 preamble2_bits;
	u8 preamble3_pattern;
	u8 delimiter[3];
	u8 options;		/* pre-assigned delay present, SN mask, power level */
	u16 preassigned_delay;
};

struct gpon_olt_pon {
	const struct gpon_olt_ploam_ops *ops;
	void *sh;
	struct gpon_olt_link_rec onu[GPON_OLT_PON_ONUS];
	struct gpon_olt_overhead overhead;
	u32 eqd_target;		/* bits: the common equalised delay */
	u32 acquire_ms;		/* serial-number window period */
	u32 timeout_ms;		/* per PLOAM exchange */
	u8 max_tries;
	bool auto_authorize;
	bool ack_lenient;	/* accept an Acknowledge that names no message */
	bool started;
	u32 next_acquire_ms;
	u32 us_rx;
	u32 us_bad;
	u32 us_unknown;
	u32 sn_rx;
	u32 ack_bad;
	u32 ack_unnamed;
	u32 no_message;
	u32 rei;
};

/* ONU-IDs are slot + 1, so an ONU keeps its ONU-ID across a re-activation. */
static inline u8 gpon_olt_onu_id(unsigned int slot)
{
	return (u8)(slot + 1);
}

void gpon_olt_pon_init(struct gpon_olt_pon *p,
		       const struct gpon_olt_ploam_ops *ops, void *sh);
int gpon_olt_pon_authorize(struct gpon_olt_pon *p, const u8 sn[8],
			   const u8 *password, u16 data_alloc);
void gpon_olt_pon_tick(struct gpon_olt_pon *p, u32 now_ms);
void gpon_olt_pon_us(struct gpon_olt_pon *p, const u8 *m, unsigned int len,
		     u32 now_ms);
void gpon_olt_pon_ranging_burst(struct gpon_olt_pon *p, u8 onu_id, u32 rtd,
				u32 now_ms);
void gpon_olt_pon_los(struct gpon_olt_pon *p, u8 onu_id, u32 now_ms);
void gpon_olt_pon_deactivate(struct gpon_olt_pon *p, u8 onu_id, u32 now_ms);
int gpon_olt_pon_disable_sn(struct gpon_olt_pon *p, const u8 sn[8],
			    bool disable, u32 now_ms);
u32 gpon_olt_pon_next_ms(const struct gpon_olt_pon *p, u32 now_ms);
struct gpon_olt_link_rec *gpon_olt_pon_find(struct gpon_olt_pon *p, u8 onu_id);

u8 gpon_olt_ploam_crc8(const u8 *d, unsigned int len);
const char *gpon_olt_link_name(unsigned int link);
const char *gpon_olt_down_name(unsigned int down);

#endif /* GPON_OLT_PLOAM_H */
