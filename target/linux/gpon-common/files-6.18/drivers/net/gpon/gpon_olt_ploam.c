// SPDX-License-Identifier: GPL-2.0-or-later
/* gpon_olt_ploam.c -- the OLT role of G.984.3 activation (see the header). */
#include <linux/kernel.h>
#include <linux/string.h>

#include "gpon_olt_ploam.h"

static bool reached(u32 now, u32 deadline)
{
	return (s32)(now - deadline) >= 0;
}

/* G.984.3 clause 9.2.1: CRC-8, generator x^8 + x^2 + x + 1, over octets 1..12. */
u8 gpon_olt_ploam_crc8(const u8 *d, unsigned int len)
{
	u8 crc = 0;
	unsigned int i, b;

	for (i = 0; i < len; i++) {
		crc ^= d[i];
		for (b = 0; b < 8; b++)
			crc = (crc & 0x80) ? (u8)((crc << 1) ^ 0x07) : (u8)(crc << 1);
	}
	return crc;
}

static void ring(struct gpon_olt_pon *p, enum gpon_unsup_class cls, u32 val,
		 const char *want, const u8 *m, unsigned int len)
{
	p->us_unknown++;
	gpon_unsup_call(p->ops->unsupported, p->sh, "olt-ploam", cls, val, want,
			m, len);
}

static void send(struct gpon_olt_pon *p, u8 *m)
{
	m[12] = gpon_olt_ploam_crc8(m, 12);
	p->ops->ds_tx(p->sh, m);
}

static void build(u8 *m, u8 onu_id, u8 type)
{
	memset(m, 0, GPON_OLT_PLOAM_DS_LEN);
	m[0] = onu_id;
	m[1] = type;
}

static void put_overhead(u8 *d, const struct gpon_olt_overhead *oh)
{
	d[0] = oh->guard_bits;
	d[1] = oh->preamble1_bits;
	d[2] = oh->preamble2_bits;
	d[3] = oh->preamble3_pattern;
	memcpy(&d[4], oh->delimiter, sizeof(oh->delimiter));
	d[7] = oh->options;
	d[8] = (u8)(oh->preassigned_delay >> 8);
	d[9] = (u8)oh->preassigned_delay;
}

static void put12(u8 *d, u16 v)
{
	d[0] = (u8)(v >> 4);
	d[1] = (u8)((v & 0xf) << 4);
}

static unsigned int slot_of(const struct gpon_olt_pon *p,
			    const struct gpon_olt_link_rec *r)
{
	return (unsigned int)(r - p->onu);
}

static u8 id_of(const struct gpon_olt_pon *p, const struct gpon_olt_link_rec *r)
{
	return gpon_olt_onu_id(slot_of(p, r));
}

struct gpon_olt_link_rec *gpon_olt_pon_find(struct gpon_olt_pon *p, u8 onu_id)
{
	if (!onu_id || onu_id > GPON_OLT_PON_ONUS || !p->onu[onu_id - 1].used)
		return NULL;
	return &p->onu[onu_id - 1];
}

static struct gpon_olt_link_rec *by_sn(struct gpon_olt_pon *p, const u8 *sn)
{
	unsigned int i;

	for (i = 0; i < GPON_OLT_PON_ONUS; i++)
		if (p->onu[i].used && !memcmp(p->onu[i].sn, sn, 8))
			return &p->onu[i];
	return NULL;
}

static struct gpon_olt_link_rec *new_rec(struct gpon_olt_pon *p, const u8 *sn)
{
	unsigned int i;

	for (i = 0; i < GPON_OLT_PON_ONUS; i++) {
		struct gpon_olt_link_rec *r = &p->onu[i];

		if (r->used)
			continue;
		memset(r, 0, sizeof(*r));
		r->used = true;
		memcpy(r->sn, sn, 8);
		return r;
	}
	return NULL;
}

static void set_link(struct gpon_olt_pon *p, struct gpon_olt_link_rec *r,
		     enum gpon_olt_link link, enum gpon_olt_down why, u32 now)
{
	bool notify = r->state != link;

	r->state = link;
	r->down = why;
	r->since_ms = now;
	r->tries = 0;
	if (link != GPON_OLT_LINK_OMCC)
		r->acks_owed = 0;
	if (link == GPON_OLT_LINK_UP)
		r->ups++;
	if (link == GPON_OLT_LINK_LOST)
		r->losses++;
	if (notify && p->ops->link_change)
		p->ops->link_change(p->sh, id_of(p, r), link, why);
}

static void deactivate(struct gpon_olt_pon *p, struct gpon_olt_link_rec *r,
		       enum gpon_olt_link link, enum gpon_olt_down why, u32 now)
{
	u8 m[GPON_OLT_PLOAM_DS_LEN];

	build(m, id_of(p, r), GPON_OLT_DS_DEACTIVATE);
	send(p, m);
	set_link(p, r, link, why, now);
}

static void assign(struct gpon_olt_pon *p, struct gpon_olt_link_rec *r, u32 now)
{
	u8 m[GPON_OLT_PLOAM_DS_LEN];

	build(m, GPON_ONU_ID_BROADCAST, GPON_OLT_DS_ASSIGN_ID);
	m[2] = id_of(p, r);
	memcpy(&m[3], r->sn, 8);
	send(p, m);
	set_link(p, r, GPON_OLT_LINK_RANGING, GPON_OLT_DOWN_NONE, now);
	if (p->ops->ranging_grant)
		p->ops->ranging_grant(p->sh, id_of(p, r));
}

static void send_owed(struct gpon_olt_pon *p, struct gpon_olt_link_rec *r)
{
	if (r->acks_owed & GPON_OLT_OWE_PORT)
		send(p, r->sent_port);
	if (r->acks_owed & GPON_OLT_OWE_ALLOC)
		send(p, r->sent_alloc);
}

static void start_omcc(struct gpon_olt_pon *p, struct gpon_olt_link_rec *r,
		       u32 now)
{
	u8 id = id_of(p, r);

	r->omcc_gem = id;
	build(r->sent_port, id, GPON_OLT_DS_PORT);
	r->sent_port[2] = 0x01;
	put12(&r->sent_port[3], r->omcc_gem);
	r->acks_owed = GPON_OLT_OWE_PORT;
	if (r->data_alloc) {
		build(r->sent_alloc, id, GPON_OLT_DS_ALLOC);
		put12(&r->sent_alloc[2], r->data_alloc);
		r->sent_alloc[4] = 0x01;
		r->acks_owed |= GPON_OLT_OWE_ALLOC;
	}
	set_link(p, r, GPON_OLT_LINK_OMCC, GPON_OLT_DOWN_NONE, now);
	send_owed(p, r);
}

static void request_password(struct gpon_olt_pon *p,
			     struct gpon_olt_link_rec *r)
{
	u8 m[GPON_OLT_PLOAM_DS_LEN];

	build(m, id_of(p, r), GPON_OLT_DS_PASSWORD);
	send(p, m);
}

void gpon_olt_pon_init(struct gpon_olt_pon *p,
		       const struct gpon_olt_ploam_ops *ops, void *sh)
{
	memset(p, 0, sizeof(*p));
	p->ops = ops;
	p->sh = sh;
	/* the burst our ONU cores range with on the virtual PON */
	p->overhead.guard_bits = 0x04;
	p->overhead.preamble3_pattern = 0xab;
	p->overhead.delimiter[0] = 0x78;
	p->overhead.delimiter[1] = 0x28;
	p->overhead.delimiter[2] = 0x28;
	p->overhead.options = 0x28;
	p->eqd_target = 0x8000;
	p->acquire_ms = 100;
	p->timeout_ms = 500;
	p->max_tries = 3;
}

int gpon_olt_pon_authorize(struct gpon_olt_pon *p, const u8 sn[8],
			   const u8 *password, u16 data_alloc)
{
	struct gpon_olt_link_rec *r = by_sn(p, sn);

	if (!r)
		r = new_rec(p, sn);
	if (!r)
		return -1;
	r->authorized = true;
	r->has_password = password != NULL;
	if (password)
		memcpy(r->password, password, GPON_OLT_PASSWORD_LEN);
	r->data_alloc = data_alloc;
	if (r->state == GPON_OLT_LINK_REJECTED)
		r->state = GPON_OLT_LINK_FREE;
	return id_of(p, r);
}

static void on_sn(struct gpon_olt_pon *p, const u8 *m, u32 now)
{
	struct gpon_olt_link_rec *r;

	p->sn_rx++;
	if (m[0] != GPON_ONU_ID_BROADCAST)
		return;
	r = by_sn(p, &m[2]);
	if (!r) {
		r = new_rec(p, &m[2]);
		if (!r) {
			ring(p, GPON_UNSUP_RANGE, GPON_OLT_PON_ONUS,
			     "a free ONU slot", m, GPON_OLT_PLOAM_US_LEN);
			return;
		}
		r->authorized = p->auto_authorize;
		set_link(p, r, GPON_OLT_LINK_AUTOFIND, GPON_OLT_DOWN_NONE, now);
	}
	switch (r->state) {
	case GPON_OLT_LINK_DISABLED:
	case GPON_OLT_LINK_REJECTED:
		return;
	case GPON_OLT_LINK_RANGING:
		break;		/* the Assign_ONU-ID was lost: assign again */
	case GPON_OLT_LINK_PASSWORD:
	case GPON_OLT_LINK_OMCC:
	case GPON_OLT_LINK_UP:
		/* an assigned ONU offering its serial again has restarted */
		set_link(p, r, GPON_OLT_LINK_LOST, GPON_OLT_DOWN_LOS, now);
		break;
	default:
		break;
	}
	if (!r->authorized) {
		if (r->state != GPON_OLT_LINK_AUTOFIND)
			set_link(p, r, GPON_OLT_LINK_AUTOFIND, GPON_OLT_DOWN_NONE, now);
		return;
	}
	assign(p, r, now);
}

static void on_password(struct gpon_olt_pon *p, struct gpon_olt_link_rec *r,
			const u8 *m, u32 now)
{
	if (r->state != GPON_OLT_LINK_PASSWORD)
		return;		/* the two repeats G.984.3 asks the ONU for */
	if (memcmp(&m[2], r->password, GPON_OLT_PASSWORD_LEN)) {
		deactivate(p, r, GPON_OLT_LINK_REJECTED,
			   GPON_OLT_DOWN_PASSWORD_MISMATCH, now);
		return;
	}
	start_omcc(p, r, now);
}

static bool ack_names(const u8 *m, const u8 *sent)
{
	return m[2] == sent[1] && !memcmp(&m[3], sent, 9);
}

static bool ack_unnamed(const u8 *m)
{
	unsigned int i;

	for (i = 2; i < GPON_OLT_PLOAM_US_LEN; i++)
		if (m[i])
			return false;
	return true;
}

static void on_ack(struct gpon_olt_pon *p, struct gpon_olt_link_rec *r,
		   const u8 *m, u32 now)
{
	if (r->state != GPON_OLT_LINK_OMCC)
		return;
	if ((r->acks_owed & GPON_OLT_OWE_PORT) && ack_names(m, r->sent_port)) {
		r->acks_owed &= (u8)~GPON_OLT_OWE_PORT;
	} else if ((r->acks_owed & GPON_OLT_OWE_ALLOC) &&
		   ack_names(m, r->sent_alloc)) {
		r->acks_owed &= (u8)~GPON_OLT_OWE_ALLOC;
	} else if (ack_unnamed(m)) {
		p->ack_unnamed++;
		if (!p->ack_lenient)
			return;
		r->acks_owed &= (u8)(r->acks_owed - 1);	/* the oldest owed */
	} else {
		p->ack_bad++;
		return;
	}
	if (!r->acks_owed)
		set_link(p, r, GPON_OLT_LINK_UP, GPON_OLT_DOWN_NONE, now);
}

void gpon_olt_pon_us(struct gpon_olt_pon *p, const u8 *m, unsigned int len,
		     u32 now_ms)
{
	struct gpon_olt_link_rec *r;

	p->us_rx++;
	if (!m || (len != GPON_OLT_PLOAM_US_LEN &&
		   len != GPON_OLT_PLOAM_US_LEN + 1) ||
	    (len == GPON_OLT_PLOAM_US_LEN + 1 &&
	     gpon_olt_ploam_crc8(m, GPON_OLT_PLOAM_US_LEN) != m[12])) {
		p->us_bad++;
		ring(p, GPON_UNSUP_RANGE, len, "a 12-octet PLOAM or 13 with its CRC",
		     m, m ? len : 0);
		return;
	}
	if (m[1] == GPON_OLT_US_SN) {
		on_sn(p, m, now_ms);
		return;
	}
	r = gpon_olt_pon_find(p, m[0]);
	switch (m[1]) {
	case GPON_OLT_US_NO_MESSAGE:
		p->no_message++;
		return;
	case GPON_OLT_US_REI:
		p->rei++;
		return;
	default:
		break;
	}
	if (!r || r->state == GPON_OLT_LINK_FREE ||
	    r->state == GPON_OLT_LINK_AUTOFIND) {
		ring(p, GPON_UNSUP_UNKNOWN, m[0], "an assigned ONU-ID", m, len);
		return;
	}
	switch (m[1]) {
	case GPON_OLT_US_PASSWORD:
		on_password(p, r, m, now_ms);
		break;
	case GPON_OLT_US_ACK:
		on_ack(p, r, m, now_ms);
		break;
	case GPON_OLT_US_DYING_GASP:
		if (r->state != GPON_OLT_LINK_LOST)
			set_link(p, r, GPON_OLT_LINK_LOST,
				 GPON_OLT_DOWN_DYING_GASP, now_ms);
		break;
	default:
		ring(p, GPON_UNSUP_UNKNOWN, m[1], "a PLOAM this OLT acts on", m,
		     len);
		break;
	}
}

void gpon_olt_pon_ranging_burst(struct gpon_olt_pon *p, u8 onu_id, u32 rtd,
				u32 now_ms)
{
	struct gpon_olt_link_rec *r = gpon_olt_pon_find(p, onu_id);
	u8 m[GPON_OLT_PLOAM_DS_LEN];

	if (!r || r->state != GPON_OLT_LINK_RANGING) {
		ring(p, GPON_UNSUP_UNKNOWN, onu_id, "a ranging ONU", NULL, 0);
		return;
	}
	r->rtd = rtd;
	r->eqd = p->eqd_target > rtd ? p->eqd_target - rtd : 0;
	build(m, onu_id, GPON_OLT_DS_RANGING);
	m[3] = (u8)(r->eqd >> 24);
	m[4] = (u8)(r->eqd >> 16);
	m[5] = (u8)(r->eqd >> 8);
	m[6] = (u8)r->eqd;
	send(p, m);
	if (r->has_password) {
		set_link(p, r, GPON_OLT_LINK_PASSWORD, GPON_OLT_DOWN_NONE, now_ms);
		request_password(p, r);
		return;
	}
	start_omcc(p, r, now_ms);
}

void gpon_olt_pon_los(struct gpon_olt_pon *p, u8 onu_id, u32 now_ms)
{
	struct gpon_olt_link_rec *r = gpon_olt_pon_find(p, onu_id);

	if (r && r->state >= GPON_OLT_LINK_RANGING &&
	    r->state <= GPON_OLT_LINK_UP)
		set_link(p, r, GPON_OLT_LINK_LOST, GPON_OLT_DOWN_LOS, now_ms);
}

void gpon_olt_pon_deactivate(struct gpon_olt_pon *p, u8 onu_id, u32 now_ms)
{
	struct gpon_olt_link_rec *r = gpon_olt_pon_find(p, onu_id);

	if (r)
		deactivate(p, r, GPON_OLT_LINK_LOST, GPON_OLT_DOWN_DEACTIVATED,
			   now_ms);
}

int gpon_olt_pon_disable_sn(struct gpon_olt_pon *p, const u8 sn[8],
			    bool disable, u32 now_ms)
{
	struct gpon_olt_link_rec *r = by_sn(p, sn);
	u8 m[GPON_OLT_PLOAM_DS_LEN];

	if (!r)
		r = new_rec(p, sn);
	if (!r)
		return -1;
	build(m, GPON_ONU_ID_BROADCAST, GPON_OLT_DS_DISABLE_SN);
	m[2] = disable ? 0xff : 0x00;
	memcpy(&m[3], sn, 8);
	send(p, m);
	set_link(p, r, disable ? GPON_OLT_LINK_DISABLED : GPON_OLT_LINK_FREE,
		 disable ? GPON_OLT_DOWN_DISABLED : GPON_OLT_DOWN_NONE, now_ms);
	return id_of(p, r);
}

static void expire(struct gpon_olt_pon *p, struct gpon_olt_link_rec *r, u32 now)
{
	static const u8 why[] = {
		[GPON_OLT_LINK_RANGING] = GPON_OLT_DOWN_RANGING_TIMEOUT,
		[GPON_OLT_LINK_PASSWORD] = GPON_OLT_DOWN_PASSWORD_TIMEOUT,
		[GPON_OLT_LINK_OMCC] = GPON_OLT_DOWN_ACK_TIMEOUT,
	};
	u8 tries = r->tries;

	if (tries + 1 >= p->max_tries) {
		deactivate(p, r, GPON_OLT_LINK_LOST, why[r->state], now);
		return;
	}
	if (r->state == GPON_OLT_LINK_RANGING && p->ops->ranging_grant)
		p->ops->ranging_grant(p->sh, id_of(p, r));
	else if (r->state == GPON_OLT_LINK_PASSWORD)
		request_password(p, r);
	else
		send_owed(p, r);
	r->tries = tries + 1;
	r->since_ms = now;
}

void gpon_olt_pon_tick(struct gpon_olt_pon *p, u32 now_ms)
{
	unsigned int i;

	if (!p->started) {
		p->started = true;
		p->next_acquire_ms = now_ms;
	}
	if (reached(now_ms, p->next_acquire_ms)) {
		u8 m[GPON_OLT_PLOAM_DS_LEN];

		build(m, GPON_ONU_ID_BROADCAST, GPON_OLT_DS_OVERHEAD);
		put_overhead(&m[2], &p->overhead);
		send(p, m);
		if (p->ops->sn_grant)
			p->ops->sn_grant(p->sh);
		p->next_acquire_ms = now_ms + p->acquire_ms;
	}
	for (i = 0; i < GPON_OLT_PON_ONUS; i++) {
		struct gpon_olt_link_rec *r = &p->onu[i];

		if (r->used && r->state >= GPON_OLT_LINK_RANGING &&
		    r->state <= GPON_OLT_LINK_OMCC &&
		    reached(now_ms, r->since_ms + p->timeout_ms))
			expire(p, r, now_ms);
	}
}

u32 gpon_olt_pon_next_ms(const struct gpon_olt_pon *p, u32 now_ms)
{
	u32 next = p->started ? p->next_acquire_ms : now_ms;
	unsigned int i;

	for (i = 0; i < GPON_OLT_PON_ONUS; i++) {
		const struct gpon_olt_link_rec *r = &p->onu[i];
		u32 at = r->since_ms + p->timeout_ms;

		if (r->used && r->state >= GPON_OLT_LINK_RANGING &&
		    r->state <= GPON_OLT_LINK_OMCC && (s32)(at - next) < 0)
			next = at;
	}
	return next;
}

const char *gpon_olt_link_name(unsigned int link)
{
	static const char *const names[] = {
		"free", "autofind", "ranging", "password", "omcc", "up",
		"rejected", "lost", "disabled",
	};

	return link < ARRAY_SIZE(names) ? names[link] : "?";
}

const char *gpon_olt_down_name(unsigned int down)
{
	static const char *const names[] = {
		"none", "ranging-timeout", "password-timeout",
		"password-mismatch", "ack-timeout", "los", "dying-gasp",
		"deactivated", "disabled",
	};

	return down < ARRAY_SIZE(names) ? names[down] : "?";
}
