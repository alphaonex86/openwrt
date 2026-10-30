/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gpon_olt_omci.h -- the OLT role of G.988 OMCI: one engine per ONU that
 * resets, identifies, uploads and provisions the ONU's MIB for one bridged
 * service (plus an optional TR-069 server push), and checks every answer:
 * result codes, attributes read back after each Set, the MIB data sync
 * counter, and the service graph in a fresh upload of the ONU's own MIB.
 *
 * TIER: CORE.  It builds and parses with the ONU role's ME table
 * (omci_me_attr_size) and frame primitives (omci_finalize, omci_mic_ok);
 * no MMIO, lock, allocator, sleep or clock: time is an input.
 */
#ifndef GPON_OLT_OMCI_H
#define GPON_OLT_OMCI_H

#include <linux/types.h>

#include "gpon_common.h"

#define GPON_OLT_OMCI_LEN	48
#define GPON_OLT_MIB_ATTRS	8192
#define GPON_OLT_MIB_POOL	65536
#define GPON_OLT_UPLOAD_MAX	2048
#define GPON_OLT_STEPS		160
#define GPON_OLT_UNIS		8
#define GPON_OLT_ALARMS		32
#define GPON_OLT_TABLE_MAX	256
#define GPON_OLT_PART_LEN	25
#define GPON_OLT_URL_LEN	(15 * GPON_OLT_PART_LEN)

enum gpon_olt_phase {
	GPON_OLT_PH_IDLE,
	GPON_OLT_PH_RESET,
	GPON_OLT_PH_IDENTITY,
	GPON_OLT_PH_UPLOAD,
	GPON_OLT_PH_ALARMS,
	GPON_OLT_PH_PROVISION,
	GPON_OLT_PH_VERIFY,
	GPON_OLT_PH_ACS,
	GPON_OLT_PH_ONLINE,
	GPON_OLT_PH_FAILED,
};

enum gpon_olt_why {
	GPON_OLT_WHY_NONE,
	GPON_OLT_WHY_TIMEOUT,		/* every retry went unanswered */
	GPON_OLT_WHY_RESULT,		/* result code other than 0 */
	GPON_OLT_WHY_ECHO,		/* the answer names another type/class/instance */
	GPON_OLT_WHY_READBACK,		/* a Get after a Set returned other bytes */
	GPON_OLT_WHY_MASK,		/* a Get answered another attribute mask */
	GPON_OLT_WHY_MDS,		/* MIB data sync is not the count of changes */
	GPON_OLT_WHY_SERIAL,		/* ONU-G serial is not the PLOAM serial */
	GPON_OLT_WHY_LOID,		/* ONU-G logical ONU ID / password refused */
	GPON_OLT_WHY_UPLOAD_SIZE,	/* MIB upload announced 0 or too many rows */
	GPON_OLT_WHY_MIB_FULL,		/* the OLT's copy of the MIB is full */
	GPON_OLT_WHY_NO_TCONT,		/* no free T-CONT in the upload */
	GPON_OLT_WHY_NO_UNI,		/* no PPTP Ethernet UNI and no VEIP */
	GPON_OLT_WHY_GRAPH,		/* the re-uploaded MIB lacks the service */
	GPON_OLT_WHY_TABLE,		/* a table read back lacks the row set */
	GPON_OLT_WHY_PLAN,		/* the provisioning plan could not be built */
	GPON_OLT_WHY_NO_TR069,		/* no TR-069 management server ME */
};

enum gpon_olt_check {
	GPON_OLT_CHECK_NONE,		/* not asked */
	GPON_OLT_CHECK_PASS,
	GPON_OLT_CHECK_FAIL,
	GPON_OLT_CHECK_COULD_NOT_ASK,
};

struct gpon_olt_fail {
	u8 why;
	u8 mt;
	u8 attr;
	u16 class_id;
	u16 inst;
	u32 want;
	u32 got;
};

struct gpon_olt_profile {
	u8 sn[8];			/* zero: not checked */
	u16 alloc_id;
	u16 gem_port;
	u16 vlan;			/* 0: transparent, no VLAN filter */
	u8 pbit;
	u16 gem_payload;
	u32 us_pir;			/* bytes/s on an ME 280 named by ME 268 #5; 0: none */
	char loid[25];
	char loid_password[13];
	bool check_loid;
	char acs_url[GPON_OLT_URL_LEN + 1];	/* empty: no TR-069 push */
	char acs_user[GPON_OLT_PART_LEN + 1];
	char acs_password[GPON_OLT_PART_LEN + 1];
	u16 acs_tag;			/* 0xffff: untagged */
	u32 timeout_ms;
	u8 retries;
	bool mic_lenient;		/* count a bad MIC instead of dropping */
	u32 audit_ms;			/* 0: no periodic MIB data sync audit */
};

struct gpon_olt_omci_ops {
	int (*omci_tx)(void *sh, u8 onu_id, const u8 *pdu, unsigned int len);
	gpon_unsup_fn unsupported;
};

struct gpon_olt_attr {
	u16 class_id;
	u16 inst;
	u16 off;
	u8 attr;		/* 0x80 | n: the undecoded rest of a row from attr n */
	u8 len;
};

struct gpon_olt_alarm {
	u16 class_id;
	u16 inst;
	u8 bitmap[28];
};

struct gpon_olt_step {
	u8 mt;			/* 0: a local action, named by inst */
	u8 phase;
	u8 flags;
	u8 blen;
	u16 class_id;
	u16 inst;
	u8 body[32];		/* message contents, octets 8..39 */
	u8 size[16];		/* a Set's attribute sizes, attr n at n - 1 */
};

struct gpon_olt_omci {
	const struct gpon_olt_omci_ops *ops;
	void *sh;
	u8 onu_id;
	struct gpon_olt_profile prof;
	u8 phase;

	u8 req[GPON_OLT_OMCI_LEN];
	bool pending;
	u16 next_tci;
	u32 sent_ms;
	u8 tries;

	struct gpon_olt_step step[GPON_OLT_STEPS];
	u16 nsteps;
	u16 cur;
	u8 sub;
	u16 sub_i;
	u16 sub_n;
	u16 rb_left;
	u16 rb_chunk;
	u8 table[GPON_OLT_TABLE_MAX];
	u16 table_len;

	struct gpon_olt_attr mib[GPON_OLT_MIB_ATTRS];
	u16 nmib;
	u32 pool_used;
	u8 pool[GPON_OLT_MIB_POOL];

	u16 tcont;
	u16 pq_us;
	u16 uni[GPON_OLT_UNIS];
	u8 nuni;
	bool has_veip;
	u16 veip;
	bool has_tr069;
	u16 tr069;
	u16 ani_port;

	u16 confirm_class;	/* the last upload value a Get was asked to confirm */
	u16 confirm_inst;
	u8 confirm_attr;
	u8 mds;
	bool mib_synced;
	bool configured;
	bool online;
	u8 acs;			/* enum gpon_olt_check */
	u8 table_check;		/* enum gpon_olt_check */
	struct gpon_olt_fail fail;
	struct gpon_olt_fail acs_fail;
	struct gpon_olt_fail table_fail;

	struct gpon_olt_alarm alarm[GPON_OLT_ALARMS];
	u8 nalarm;
	u8 alarm_seq;
	bool alarm_resync;
	u32 audit_at;

	u32 tx;
	u32 rx;
	u32 retries;
	u32 stale;
	u32 bad_mic;
	u32 runt;
	u32 autonomous;
	u32 avc;
	u32 alarm_rx;
	u32 alarm_gap;
	u32 unknown;
	u32 upload_rows;
	u32 upload_opaque;
	u32 upload_suspect;
	u32 upload_disagree;
	u32 resyncs;
	u32 starts;
};

void gpon_olt_profile_default(struct gpon_olt_profile *p);
void gpon_olt_omci_init(struct gpon_olt_omci *o,
			const struct gpon_olt_omci_ops *ops, void *sh,
			u8 onu_id, const struct gpon_olt_profile *prof);
void gpon_olt_omci_start(struct gpon_olt_omci *o, u32 now_ms);
void gpon_olt_omci_stop(struct gpon_olt_omci *o);
void gpon_olt_omci_rx(struct gpon_olt_omci *o, const u8 *pdu,
		      unsigned int len, u32 now_ms);
void gpon_olt_omci_tick(struct gpon_olt_omci *o, u32 now_ms);
bool gpon_olt_omci_next_ms(const struct gpon_olt_omci *o, u32 *at);
const u8 *gpon_olt_mib_get(const struct gpon_olt_omci *o, u16 class_id,
			   u16 inst, u8 attr, u8 *len);

const char *gpon_olt_phase_name(unsigned int phase);
const char *gpon_olt_why_name(unsigned int why);
const char *gpon_olt_check_name(unsigned int check);

#endif /* GPON_OLT_OMCI_H */
