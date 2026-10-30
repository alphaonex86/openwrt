// SPDX-License-Identifier: GPL-2.0-or-later
/* gpon_olt_omci.c -- the OLT role of G.988 OMCI (see the header). */
#include <linux/kernel.h>
#include <linux/string.h>

#include "gpon_omci_core.h"
#include "gpon_omci_me.h"
#include "gpon_omci_vlan.h"
#include "gpon_olt_omci.h"

enum { SUB_NONE, SUB_UPLOAD, SUB_ALARMS, SUB_READBACK, SUB_TABLE };
enum { LOCAL_PLAN = 1, LOCAL_GRAPH, LOCAL_ACS_DONE, LOCAL_ONLINE };

#define F_MDS		0x01	/* a success moves MIB data sync */
#define F_READBACK	0x02	/* a Set is read back by Get */
#define F_ACS		0x04	/* part of the TR-069 push */
#define F_TABLE		0x08	/* a Set of one table row, read back by Get next */
#define F_AUDIT		0x10	/* a mismatch re-provisions instead of failing */
#define F_BAD		0x80	/* the builder could not lay the step out */

/* Instances the OLT creates: one service per ONU. */
#define OLT_BRIDGE	1
#define OLT_GAL		1
#define OLT_CTP		1
#define OLT_IWTP	1
#define OLT_STRING	1
#define OLT_AUTH	1
#define OLT_ADDRESS	1
#define OLT_TD		1

/* G.988 MEs the OLT creates for the TR-069 push that the ONU model lacks. */
#define ME_NET_ADDRESS	137
#define ME_AUTH_METHOD	148
#define ME_LARGE_STRING	157

#define GET_VALUES	25	/* Get response value octets 11..35 */
#define NEXT_VALUES	29	/* Get next response value octets 11..39 */
#define UPLOAD_VALUES	26	/* MIB upload next value octets 14..39 */
#define AVC_VALUES	30	/* AVC value octets 10..39 */
#define TPID_C		0x8100

static u16 be16(const u8 *p)
{
	return ((u16)p[0] << 8) | p[1];
}

static u32 be_n(const u8 *p, unsigned int n)
{
	u32 v = 0;

	while (n--)
		v = (v << 8) | *p++;
	return v;
}

static unsigned int text_len(const char *s, unsigned int max)
{
	unsigned int n = 0;

	while (n < max && s[n])
		n++;
	return n;
}

static bool reached(u32 now, u32 at)
{
	return (s32)(now - at) >= 0;
}

static u8 extra_size(u16 class_id, u8 attr)
{
	switch (class_id) {
	case ME_LARGE_STRING:
		return attr == 1 ? 1 : attr <= 16 ? GPON_OLT_PART_LEN : 0;
	case ME_AUTH_METHOD:
		return attr == 1 ? 1 : attr <= 5 ? GPON_OLT_PART_LEN : 0;
	case ME_NET_ADDRESS:
		return attr <= 2 ? 2 : 0;
	}
	return 0;
}

static u8 attr_size(u16 class_id, u8 attr)
{
	u8 n = omci_me_attr_size(class_id, attr);

	return n ? n : extra_size(class_id, attr);
}

static void ring(struct gpon_olt_omci *o, enum gpon_unsup_class cls, u32 val,
		 const char *want, const u8 *data, unsigned int len)
{
	o->unknown++;
	gpon_unsup_call(o->ops->unsupported, o->sh, "olt-omci", cls, val, want,
			data, len);
}

/* ---- the OLT's copy of the ONU's MIB ---------------------------------- */

static int mib_index(const struct gpon_olt_omci *o, u16 class_id, u16 inst,
		     u8 attr)
{
	unsigned int i;

	for (i = 0; i < o->nmib; i++)
		if (o->mib[i].class_id == class_id && o->mib[i].inst == inst &&
		    o->mib[i].attr == attr)
			return (int)i;
	return -1;
}

static bool mib_put(struct gpon_olt_omci *o, u16 class_id, u16 inst, u8 attr,
		    const u8 *v, u8 len)
{
	int i = mib_index(o, class_id, inst, attr);
	struct gpon_olt_attr *e;

	if (i >= 0 && o->mib[i].len == len) {
		if (len)
			memcpy(o->pool + o->mib[i].off, v, len);
		return true;
	}
	if (o->pool_used + len > GPON_OLT_MIB_POOL ||
	    (i < 0 && o->nmib >= GPON_OLT_MIB_ATTRS))
		return false;
	e = i >= 0 ? &o->mib[i] : &o->mib[o->nmib++];
	e->class_id = class_id;
	e->inst = inst;
	e->attr = attr;
	e->len = len;
	e->off = (u16)o->pool_used;
	if (len)
		memcpy(o->pool + e->off, v, len);
	o->pool_used += len;
	return true;
}

const u8 *gpon_olt_mib_get(const struct gpon_olt_omci *o, u16 class_id,
			   u16 inst, u8 attr, u8 *len)
{
	int i = mib_index(o, class_id, inst, attr);

	if (i < 0)
		return NULL;
	*len = o->mib[i].len;
	return o->pool + o->mib[i].off;
}

static void mib_clear(struct gpon_olt_omci *o)
{
	o->nmib = 0;
	o->pool_used = 0;
}

/* Split @v by the attribute sizes of @mask; what cannot be sized is kept whole. */
static bool store_attrs(struct gpon_olt_omci *o, u16 class_id, u16 inst,
			u16 mask, const u8 *v, unsigned int vlen)
{
	unsigned int pos = 0;
	u8 attr;

	if (mib_index(o, class_id, inst, 0) < 0 &&
	    !mib_put(o, class_id, inst, 0, NULL, 0))
		return false;
	for (attr = 1; attr <= 16; attr++) {
		u8 sz;

		if (!(mask & OMCI_ATTR_BIT(attr)))
			continue;
		sz = attr_size(class_id, attr);
		if (!sz || pos + sz > vlen) {
			o->upload_opaque++;
			ring(o, GPON_UNSUP_UNKNOWN, ((u32)class_id << 8) | attr,
			     "an attribute size this model knows", v, vlen);
			return mib_put(o, class_id, inst, 0x80 | attr, v + pos,
				       (u8)(vlen - pos));
		}
		if (!mib_put(o, class_id, inst, attr, v + pos, sz))
			return false;
		pos += sz;
	}
	for (; pos < vlen; pos++)
		if (v[pos]) {
			o->upload_suspect++;
			break;
		}
	return true;
}

static bool mib_u32(const struct gpon_olt_omci *o, u16 class_id, u16 inst,
		    u8 attr, u32 *v)
{
	u8 len;
	const u8 *p = gpon_olt_mib_get(o, class_id, inst, attr, &len);

	if (!p || !len)
		return false;
	*v = be_n(p, len > 4 ? 4 : len);
	return true;
}

/* ---- failures ----------------------------------------------------------- */

static void next_step(struct gpon_olt_omci *o)
{
	o->cur++;
	o->sub = SUB_NONE;
}

static void fault(struct gpon_olt_omci *o, u8 why, u8 attr, u32 want, u32 got)
{
	const struct gpon_olt_step *s = &o->step[o->cur];
	struct gpon_olt_fail f = {
		.why = why, .mt = s->mt ? o->req[2] & 0x1f : 0, .attr = attr,
		.class_id = s->class_id, .inst = s->inst, .want = want, .got = got,
	};

	o->pending = false;
	o->sub = SUB_NONE;
	if (s->flags & F_ACS) {
		o->acs = GPON_OLT_CHECK_FAIL;
		o->acs_fail = f;
		while (o->cur < o->nsteps && (o->step[o->cur].flags & F_ACS))
			o->cur++;
		return;
	}
	o->fail = f;
	o->phase = GPON_OLT_PH_FAILED;
	o->online = false;
}

static void table_verdict(struct gpon_olt_omci *o, u8 check, u32 want, u32 got)
{
	const struct gpon_olt_step *s = &o->step[o->cur];

	if (o->table_check == GPON_OLT_CHECK_FAIL)
		return;
	if (o->table_check == GPON_OLT_CHECK_COULD_NOT_ASK &&
	    check == GPON_OLT_CHECK_PASS)
		return;
	o->table_check = check;
	if (check == GPON_OLT_CHECK_PASS)
		return;
	o->table_fail.why = GPON_OLT_WHY_TABLE;
	o->table_fail.mt = o->req[2] & 0x1f;
	o->table_fail.class_id = s->class_id;
	o->table_fail.inst = s->inst;
	o->table_fail.want = want;
	o->table_fail.got = got;
}

/* ---- the wire ------------------------------------------------------------ */

static void transmit(struct gpon_olt_omci *o, u32 now)
{
	o->pending = true;
	o->sent_ms = now;
	o->tx++;
	o->ops->omci_tx(o->sh, o->onu_id, o->req, GPON_OLT_OMCI_LEN);
}

static void request(struct gpon_olt_omci *o, u8 mt, u16 class_id, u16 inst,
		    const u8 *body, unsigned int blen, u32 now)
{
	u16 tci = o->next_tci;

	o->next_tci = tci >= 0x7fff ? 1 : tci + 1;
	memset(o->req, 0, sizeof(o->req));
	omci_put_be16(o->req, tci);
	o->req[2] = 0x40 | mt;
	o->req[3] = 0x0a;
	omci_put_be16(o->req + 4, class_id);
	omci_put_be16(o->req + 6, inst);
	if (blen)
		memcpy(o->req + 8, body, blen > 32 ? 32 : blen);
	omci_finalize(o->req);
	o->tries = 0;
	transmit(o, now);
}

static void request_seq(struct gpon_olt_omci *o, u8 mt, u16 class_id,
			u16 inst, u16 a, u16 b, unsigned int blen, u32 now)
{
	u8 body[4];

	omci_put_be16(body, a);
	omci_put_be16(body + 2, b);
	request(o, mt, class_id, inst, body, blen, now);
}

/* ---- the plan ------------------------------------------------------------ */

static void init_step(struct gpon_olt_step *s, u8 phase, u8 mt, u16 class_id,
		      u16 inst, u8 flags)
{
	memset(s, 0, sizeof(*s));
	s->phase = phase;
	s->mt = mt;
	s->class_id = class_id;
	s->inst = inst;
	s->flags = flags;
	if (mt == OMCI_MT_SET || mt == OMCI_MT_GET)
		s->blen = 2;
}

static struct gpon_olt_step *add(struct gpon_olt_omci *o, u8 phase, u8 mt,
				 u16 class_id, u16 inst, u8 flags)
{
	struct gpon_olt_step *s;

	if (o->nsteps >= GPON_OLT_STEPS) {
		/* the last slot becomes a step the runner refuses */
		s = &o->step[GPON_OLT_STEPS - 1];
		s->flags |= F_BAD;
		return s;
	}
	s = &o->step[o->nsteps++];
	init_step(s, phase, mt, class_id, inst, flags);
	return s;
}

static struct gpon_olt_step *local(struct gpon_olt_omci *o, u8 phase, u16 what,
				   u8 flags)
{
	return add(o, phase, 0, 0, what, flags);
}

static struct gpon_olt_step *get(struct gpon_olt_omci *o, u8 phase,
				 u16 class_id, u16 inst, u16 mask, u8 flags)
{
	struct gpon_olt_step *s = add(o, phase, OMCI_MT_GET, class_id, inst,
				      flags);

	omci_put_be16(s->body, mask);
	return s;
}

static struct gpon_olt_step *create(struct gpon_olt_omci *o, u16 class_id,
				    u16 inst, u8 flags)
{
	return add(o, GPON_OLT_PH_PROVISION, OMCI_MT_CREATE, class_id, inst,
		   F_MDS | flags);
}

static struct gpon_olt_step *set(struct gpon_olt_omci *o, u16 class_id,
				 u16 inst, u8 flags)
{
	return add(o, GPON_OLT_PH_PROVISION, OMCI_MT_SET, class_id, inst,
		   F_MDS | F_READBACK | flags);
}

/* Reserve attribute @attr of the step's class, in ascending order. */
static u8 *slot(struct gpon_olt_step *s, u8 attr, u8 *size)
{
	u8 sz = attr && attr <= 16 ? attr_size(s->class_id, attr) : 0;
	unsigned int i;
	u8 *p;

	for (i = attr ? attr - 1u : 16u; i < 16 && sz; i++)
		if (s->size[i])
			sz = 0;
	if (!sz || s->blen + sz > sizeof(s->body)) {
		s->flags |= F_BAD;
		return NULL;
	}
	p = s->body + s->blen;
	memset(p, 0, sz);
	s->size[attr - 1] = sz;
	s->blen += sz;
	if (s->mt == OMCI_MT_SET)
		omci_put_be16(s->body, be16(s->body) | OMCI_ATTR_BIT(attr));
	*size = sz;
	return p;
}

static void put_num(struct gpon_olt_step *s, u8 attr, u32 v)
{
	u8 sz, i;
	u8 *p = slot(s, attr, &sz);

	for (i = 0; p && i < sz && i < 4; i++)
		p[sz - 1 - i] = (u8)(v >> (8 * i));
}

static void put_str(struct gpon_olt_step *s, u8 attr, const void *v,
		    unsigned int n)
{
	u8 sz;
	u8 *p = slot(s, attr, &sz);

	if (p)
		memcpy(p, v, n < sz ? n : sz);
}

static void bridge_port(struct gpon_olt_step *s, u16 port, u8 tp_type,
			u16 tp)
{
	put_num(s, 1, OLT_BRIDGE);
	put_num(s, 2, port);
	put_num(s, 3, tp_type);
	put_num(s, 4, tp);
	put_num(s, 5, 0);
	put_num(s, 6, 1);
	put_num(s, 7, 0);
	put_num(s, 8, 1);
	put_num(s, 9, 1);
	put_num(s, 13, 0);
}

static void vlan_row(const struct gpon_olt_profile *p, bool tag, u8 *row)
{
	struct gpon_ext_vlan_row r = {
		.f_out_pri = GPON_VLAN_PRI_NONE, .f_out_vid = GPON_VLAN_VID_ANY,
		.f_in_pri = GPON_VLAN_PRI_NONE, .f_in_vid = GPON_VLAN_VID_ANY,
		.t_out_pri = GPON_VLAN_PRI_NONE, .t_in_pri = GPON_VLAN_PRI_NONE,
	};

	if (tag) {
		r.t_in_pri = p->pbit;
		r.t_in_vid = p->vlan;
		r.t_in_tpid = 4;	/* TPID 0x8100 */
	}
	gpon_ext_vlan_row_encode(&r, row);
}

static bool locked(const struct gpon_olt_omci *o, u16 class_id, u16 inst,
		   u8 attr)
{
	u32 v;

	return !mib_u32(o, class_id, inst, attr, &v) || v != 0;
}

static void uni_port(struct gpon_olt_omci *o, u16 port, u16 class_id,
		     u16 inst, u8 tp_type, u8 assoc, bool tag)
{
	u8 admin = class_id == OMCI_ME_PPTP_ETH_UNI ? 5 : 1;
	struct gpon_olt_step *s;
	u8 row[GPON_EXT_VLAN_ROW_LEN];

	if (locked(o, class_id, inst, admin))
		put_num(set(o, class_id, inst, 0), admin, 0);
	bridge_port(create(o, OMCI_ME_MAC_BRIDGE_PORT, port, 0), port, tp_type,
		    inst);
	s = create(o, OMCI_ME_EXT_VLAN, port, 0);
	put_num(s, 1, assoc);
	put_num(s, 7, inst);
	s = set(o, OMCI_ME_EXT_VLAN, port, 0);
	put_num(s, 3, TPID_C);
	put_num(s, 4, TPID_C);
	put_num(s, 5, 0);
	vlan_row(&o->prof, tag, row);
	put_str(set(o, OMCI_ME_EXT_VLAN, port, F_TABLE), 6, row, sizeof(row));
}

static void insert_uni(struct gpon_olt_omci *o, u16 inst)
{
	unsigned int i, j;

	for (i = 0; i < o->nuni; i++)
		if (o->uni[i] == inst)
			return;
	if (o->nuni >= GPON_OLT_UNIS)
		return;
	for (i = 0; i < o->nuni && o->uni[i] < inst; i++)
		;
	for (j = o->nuni; j > i; j--)
		o->uni[j] = o->uni[j - 1];
	o->uni[i] = inst;
	o->nuni++;
}

static void lowest(u16 inst, bool *have, u16 *best)
{
	if (!*have || inst < *best)
		*best = inst;
	*have = true;
}

static bool pick_resources(struct gpon_olt_omci *o)
{
	bool tcont = false, pq = false;
	unsigned int i;
	u32 v;

	o->nuni = 0;
	o->has_veip = false;
	o->has_tr069 = false;
	o->pq_us = 0;
	for (i = 0; i < o->nmib; i++) {
		const struct gpon_olt_attr *e = &o->mib[i];

		/* unassigned: 0xff, or no 12-bit Alloc-ID at all (0xffff, ZTE's 0xff00) */
		if (e->class_id == OMCI_ME_TCONT && e->attr == 1 &&
		    mib_u32(o, e->class_id, e->inst, 1, &v) &&
		    (v == 0xff || v > 0xfff))
			lowest(e->inst, &tcont, &o->tcont);
		else if (e->class_id == OMCI_ME_PPTP_ETH_UNI)
			insert_uni(o, e->inst);
		else if (e->class_id == OMCI_ME_VEIP)
			lowest(e->inst, &o->has_veip, &o->veip);
		else if (e->class_id == OMCI_ME_TR069_SERVER)
			lowest(e->inst, &o->has_tr069, &o->tr069);
	}
	if (!tcont) {
		fault(o, GPON_OLT_WHY_NO_TCONT, 1, 0xffff, 0);
		return false;
	}
	if (!o->nuni && !o->has_veip) {
		fault(o, GPON_OLT_WHY_NO_UNI, 0, 1, 0);
		return false;
	}
	for (i = 0; i < o->nmib; i++) {
		const struct gpon_olt_attr *e = &o->mib[i];

		if (e->class_id == OMCI_ME_PRIORITY_QUEUE && e->attr == 6 &&
		    e->inst >= 0x8000 &&
		    mib_u32(o, e->class_id, e->inst, 6, &v) &&
		    (v >> 16) == o->tcont)
			lowest(e->inst, &pq, &o->pq_us);
	}
	return true;
}

static void plan_acs(struct gpon_olt_omci *o)
{
	const struct gpon_olt_profile *p = &o->prof;
	unsigned int len = text_len(p->acs_url, GPON_OLT_URL_LEN);
	unsigned int parts = (len + GPON_OLT_PART_LEN - 1) / GPON_OLT_PART_LEN;
	unsigned int i;
	struct gpon_olt_step *s;

	if (!o->has_tr069) {
		struct gpon_olt_fail f = {
			.why = GPON_OLT_WHY_NO_TR069,
			.class_id = OMCI_ME_TR069_SERVER, .want = 1,
		};

		o->acs = GPON_OLT_CHECK_FAIL;
		o->acs_fail = f;
		return;
	}
	create(o, ME_LARGE_STRING, OLT_STRING, F_ACS);
	put_num(set(o, ME_LARGE_STRING, OLT_STRING, F_ACS), 1, parts);
	for (i = 0; i < parts; i++)
		put_str(set(o, ME_LARGE_STRING, OLT_STRING, F_ACS), (u8)(2 + i),
			p->acs_url + i * GPON_OLT_PART_LEN,
			len - i * GPON_OLT_PART_LEN);
	create(o, ME_AUTH_METHOD, OLT_AUTH, F_ACS);
	put_num(set(o, ME_AUTH_METHOD, OLT_AUTH, F_ACS), 1,
		p->acs_user[0] ? 1 : 0);
	put_str(set(o, ME_AUTH_METHOD, OLT_AUTH, F_ACS), 2, p->acs_user,
		text_len(p->acs_user, GPON_OLT_PART_LEN));
	put_str(set(o, ME_AUTH_METHOD, OLT_AUTH, F_ACS), 3, p->acs_password,
		text_len(p->acs_password, GPON_OLT_PART_LEN));
	s = create(o, ME_NET_ADDRESS, OLT_ADDRESS, F_ACS);
	put_num(s, 1, OLT_AUTH);
	put_num(s, 2, OLT_STRING);
	s = set(o, OMCI_ME_TR069_SERVER, o->tr069, F_ACS);
	put_num(s, 1, 0);
	put_num(s, 2, OLT_ADDRESS);
	if (p->acs_tag != 0xffff)
		put_num(s, 3, p->acs_tag);
	local(o, GPON_OLT_PH_ACS, LOCAL_ACS_DONE, F_ACS);
}

static void plan_service(struct gpon_olt_omci *o)
{
	const struct gpon_olt_profile *p = &o->prof;
	struct gpon_olt_step *s;
	u16 port = 0;
	unsigned int i;

	if (!pick_resources(o))
		return;
	put_num(set(o, OMCI_ME_TCONT, o->tcont, 0), 1, p->alloc_id);
	s = create(o, OMCI_ME_MAC_BRIDGE_SVC, OLT_BRIDGE, 0);
	put_num(s, 1, 0);
	put_num(s, 2, 1);
	put_num(s, 3, 0);
	put_num(s, 4, 0x8000);
	put_num(s, 5, 0x1400);
	put_num(s, 6, 0x0200);
	put_num(s, 7, 0x0f00);
	put_num(s, 8, 0);
	put_num(s, 9, 0);
	put_num(s, 10, 0);
	for (i = 0; i < o->nuni; i++)
		uni_port(o, ++port, OMCI_ME_PPTP_ETH_UNI, o->uni[i], 1, 2,
			 p->vlan != 0);
	if (o->has_veip)
		uni_port(o, ++port, OMCI_ME_VEIP, o->veip, 11, 10, false);
	o->ani_port = ++port;
	put_num(create(o, OMCI_ME_GAL_ETH_PROF, OLT_GAL, 0), 1, p->gem_payload);
	if (p->us_pir) {
		s = create(o, OMCI_ME_GEM_TRAFFIC_DESC, OLT_TD, 0);
		put_num(s, 1, 0);
		put_num(s, 2, p->us_pir);
	}
	s = create(o, OMCI_ME_GEM_CTP, OLT_CTP, 0);
	put_num(s, 1, p->gem_port);
	put_num(s, 2, o->tcont);
	put_num(s, 3, GPON_GEM_BIDIR);
	put_num(s, 4, o->pq_us);
	put_num(s, 5, p->us_pir ? OLT_TD : 0);
	put_num(s, 7, 0);
	s = create(o, OMCI_ME_GEM_IW_TP, OLT_IWTP, 0);
	put_num(s, 1, OLT_CTP);
	put_num(s, 2, 1);	/* MAC bridged LAN */
	put_num(s, 3, OLT_BRIDGE);
	put_num(s, 4, 0);
	put_num(s, 7, OLT_GAL);
	bridge_port(create(o, OMCI_ME_MAC_BRIDGE_PORT, o->ani_port, 0),
		    o->ani_port, OMCI_BRIDGE_TP_GEM_IW, OLT_IWTP);
	if (p->vlan) {
		u8 list[24] = { 0 };

		omci_put_be16(list, (u16)((p->pbit << 13) | p->vlan));
		s = create(o, OMCI_ME_VLAN_TAG_FILTER, o->ani_port, 0);
		put_str(s, 1, list, sizeof(list));
		put_num(s, 2, 0x10);
		put_num(s, 3, 1);
	}

	get(o, GPON_OLT_PH_VERIFY, OMCI_ME_ONU_DATA, 0, OMCI_ATTR_BIT(1), 0);
	add(o, GPON_OLT_PH_VERIFY, OMCI_MT_MIB_UPLOAD, OMCI_ME_ONU_DATA, 0, 0);
	local(o, GPON_OLT_PH_VERIFY, LOCAL_GRAPH, 0);
	if (p->acs_url[0])
		plan_acs(o);
	get(o, GPON_OLT_PH_VERIFY, OMCI_ME_ONU_DATA, 0, OMCI_ATTR_BIT(1), 0);
	local(o, GPON_OLT_PH_VERIFY, LOCAL_ONLINE, 0);
}

static void graph_fail(struct gpon_olt_omci *o, u16 class_id, u16 inst,
		       u8 attr, u32 want, u32 got)
{
	o->fail.why = GPON_OLT_WHY_GRAPH;
	o->fail.mt = OMCI_MT_MIB_UPLOAD;
	o->fail.attr = attr;
	o->fail.class_id = class_id;
	o->fail.inst = inst;
	o->fail.want = want;
	o->fail.got = got;
	o->phase = GPON_OLT_PH_FAILED;
}

/* An upload that disagrees is asked once by Get, then the graph is judged again. */
static bool expect(struct gpon_olt_omci *o, u16 class_id, u16 inst, u8 attr,
		   u32 want)
{
	struct gpon_olt_step *s = &o->step[o->cur];
	u32 got;

	if (!mib_u32(o, class_id, inst, attr, &got))
		got = 0xffffffff;
	else if (got == want)
		return true;
	if ((o->confirm_class == class_id && o->confirm_inst == inst &&
	     o->confirm_attr == attr) || o->nsteps + 2 > GPON_OLT_STEPS) {
		graph_fail(o, class_id, inst, attr, want, got);
		return false;
	}
	o->confirm_class = class_id;
	o->confirm_inst = inst;
	o->confirm_attr = attr;
	o->upload_disagree++;
	memmove(s + 3, s + 1, (o->nsteps - o->cur - 1) * sizeof(*s));
	o->nsteps += 2;
	init_step(s + 1, GPON_OLT_PH_VERIFY, OMCI_MT_GET, class_id, inst, 0);
	omci_put_be16(s[1].body, (u16)OMCI_ATTR_BIT(attr));
	init_step(s + 2, GPON_OLT_PH_VERIFY, 0, 0, LOCAL_GRAPH, 0);
	return false;
}

/* The service as the ONU's own upload states it, link by link. */
static void check_graph(struct gpon_olt_omci *o)
{
	const struct gpon_olt_profile *p = &o->prof;
	u16 port = 0;
	unsigned int i;

	if (!expect(o, OMCI_ME_TCONT, o->tcont, 1, p->alloc_id) ||
	    !expect(o, OMCI_ME_GEM_CTP, OLT_CTP, 1, p->gem_port) ||
	    !expect(o, OMCI_ME_GEM_CTP, OLT_CTP, 2, o->tcont) ||
	    !expect(o, OMCI_ME_GEM_IW_TP, OLT_IWTP, 1, OLT_CTP) ||
	    !expect(o, OMCI_ME_GEM_IW_TP, OLT_IWTP, 2, 1) ||
	    !expect(o, OMCI_ME_GEM_IW_TP, OLT_IWTP, 3, OLT_BRIDGE) ||
	    !expect(o, OMCI_ME_MAC_BRIDGE_PORT, o->ani_port, 1, OLT_BRIDGE) ||
	    !expect(o, OMCI_ME_MAC_BRIDGE_PORT, o->ani_port, 3,
		    OMCI_BRIDGE_TP_GEM_IW) ||
	    !expect(o, OMCI_ME_MAC_BRIDGE_PORT, o->ani_port, 4, OLT_IWTP))
		return;
	if (mib_index(o, OMCI_ME_MAC_BRIDGE_SVC, OLT_BRIDGE, 0) < 0) {
		graph_fail(o, OMCI_ME_MAC_BRIDGE_SVC, OLT_BRIDGE, 0, 1, 0);
		return;
	}
	for (i = 0; i < o->nuni + (o->has_veip ? 1u : 0u); i++) {
		u16 tp = i < o->nuni ? o->uni[i] : o->veip;

		port++;
		if (!expect(o, OMCI_ME_MAC_BRIDGE_PORT, port, 1, OLT_BRIDGE) ||
		    !expect(o, OMCI_ME_MAC_BRIDGE_PORT, port, 4, tp) ||
		    !expect(o, OMCI_ME_EXT_VLAN, port, 7, tp))
			return;
	}
	if (p->vlan &&
	    (!expect(o, OMCI_ME_VLAN_TAG_FILTER, o->ani_port, 1,
		     (u32)((p->pbit << 13) | p->vlan) << 16) ||
	     !expect(o, OMCI_ME_VLAN_TAG_FILTER, o->ani_port, 3, 1)))
		return;
	o->configured = true;
}

/* ---- answers ------------------------------------------------------------- */

static void mds_step(struct gpon_olt_omci *o)
{
	if (++o->mds == 0)
		o->mds = 1;
}

static unsigned int value_off(const struct gpon_olt_step *s, u8 attr)
{
	u16 mask = be16(s->body);
	unsigned int off = 2;
	u8 a;

	for (a = 1; a < attr; a++)
		if (mask & OMCI_ATTR_BIT(a))
			off += s->size[a - 1];
	return off;
}

static u16 chunk_of(const struct gpon_olt_step *s, u16 left)
{
	unsigned int sum = 0;
	u16 chunk = 0;
	u8 attr;

	for (attr = 1; attr <= 16; attr++) {
		u16 bit = (u16)OMCI_ATTR_BIT(attr);

		if (!(left & bit))
			continue;
		if (chunk && sum + s->size[attr - 1] > GET_VALUES)
			break;
		chunk |= bit;
		sum += s->size[attr - 1];
	}
	return chunk;
}

static void readback_next(struct gpon_olt_omci *o, u32 now)
{
	const struct gpon_olt_step *s = &o->step[o->cur];

	if (!o->rb_left) {
		next_step(o);
		return;
	}
	o->rb_chunk = chunk_of(s, o->rb_left);
	o->rb_left &= (u16)~o->rb_chunk;
	o->sub = SUB_READBACK;
	request_seq(o, OMCI_MT_GET, s->class_id, s->inst, o->rb_chunk, 0, 2, now);
}

static void table_next(struct gpon_olt_omci *o, u32 now)
{
	const struct gpon_olt_step *s = &o->step[o->cur];

	o->sub = SUB_TABLE;
	request_seq(o, OMCI_MT_GET_NEXT, s->class_id, s->inst, o->rb_chunk,
		    o->sub_i, 4, now);
}

static u8 only_attr(u16 mask)
{
	u8 a;

	for (a = 1; a <= 16; a++)
		if (mask & OMCI_ATTR_BIT(a))
			return a;
	return 0;
}

static void on_table_size(struct gpon_olt_omci *o, const u8 *pdu, u32 now)
{
	const struct gpon_olt_step *s = &o->step[o->cur];
	u8 row = s->size[only_attr(o->rb_chunk) - 1];
	u32 size = be_n(pdu + 11, 4);

	if (!size || size % row || size > GPON_OLT_TABLE_MAX) {
		table_verdict(o, GPON_OLT_CHECK_FAIL, row, size);
		readback_next(o, now);
		return;
	}
	o->sub_n = (u16)size;
	o->sub_i = 0;
	o->table_len = 0;
	table_next(o, now);
}

static void on_table_part(struct gpon_olt_omci *o, const u8 *pdu, u32 now)
{
	const struct gpon_olt_step *s = &o->step[o->cur];
	u8 attr = only_attr(o->rb_chunk);
	u8 row = s->size[attr - 1];
	const u8 *want = s->body + value_off(s, attr);
	unsigned int n, i;

	if (pdu[8] != OMCI_RC_OK) {
		table_verdict(o, GPON_OLT_CHECK_COULD_NOT_ASK, OMCI_RC_OK, pdu[8]);
		readback_next(o, now);
		return;
	}
	n = o->sub_n - o->table_len;
	if (n > NEXT_VALUES)
		n = NEXT_VALUES;
	memcpy(o->table + o->table_len, pdu + 11, n);
	o->table_len += n;
	o->sub_i++;
	if (o->table_len < o->sub_n) {
		table_next(o, now);
		return;
	}
	for (i = 0; i + row <= o->table_len; i += row)
		if (!memcmp(o->table + i, want, row))
			break;
	table_verdict(o, i + row <= o->table_len ? GPON_OLT_CHECK_PASS :
		      GPON_OLT_CHECK_FAIL, be_n(want, 4), be_n(o->table, 4));
	readback_next(o, now);
}

static void on_readback(struct gpon_olt_omci *o, const u8 *pdu, u32 now)
{
	const struct gpon_olt_step *s = &o->step[o->cur];
	unsigned int pos = 11;
	u8 attr;

	if (pdu[8] != OMCI_RC_OK) {
		fault(o, GPON_OLT_WHY_RESULT, only_attr(o->rb_chunk), OMCI_RC_OK,
		      pdu[8]);
		return;
	}
	if (be16(pdu + 9) != o->rb_chunk) {
		fault(o, GPON_OLT_WHY_MASK, 0, o->rb_chunk, be16(pdu + 9));
		return;
	}
	if (s->flags & F_TABLE) {
		on_table_size(o, pdu, now);
		return;
	}
	for (attr = 1; attr <= 16; attr++) {
		u8 sz = s->size[attr - 1];
		const u8 *want = s->body + value_off(s, attr);

		if (!(o->rb_chunk & OMCI_ATTR_BIT(attr)))
			continue;
		if (memcmp(pdu + pos, want, sz)) {
			fault(o, GPON_OLT_WHY_READBACK, attr,
			      be_n(want, sz > 4 ? 4 : sz),
			      be_n(pdu + pos, sz > 4 ? 4 : sz));
			return;
		}
		pos += sz;
	}
	readback_next(o, now);
}

static bool padded_equal(const u8 *v, u8 len, const char *text)
{
	u8 i;

	for (i = 0; i < len; i++)
		if (v[i] != (u8)(*text ? *text++ : 0))
			return false;
	return !*text;
}

static void check_identity(struct gpon_olt_omci *o)
{
	const struct gpon_olt_profile *p = &o->prof;
	static const u8 zero[8];
	const u8 *v;
	u8 len;

	v = gpon_olt_mib_get(o, OMCI_ME_ONU_G, 0, 3, &len);
	if (v && len == 8 && memcmp(p->sn, zero, 8) && memcmp(v, p->sn, 8)) {
		fault(o, GPON_OLT_WHY_SERIAL, 3, be_n(p->sn + 4, 4),
		      be_n(v + 4, 4));
		return;
	}
	if (!p->check_loid)
		return;
	v = gpon_olt_mib_get(o, OMCI_ME_ONU_G, 0, 10, &len);
	if (v && !padded_equal(v, len, p->loid)) {
		fault(o, GPON_OLT_WHY_LOID, 10, 0, be_n(v, len > 4 ? 4 : len));
		return;
	}
	v = gpon_olt_mib_get(o, OMCI_ME_ONU_G, 0, 11, &len);
	if (v && !padded_equal(v, len, p->loid_password))
		fault(o, GPON_OLT_WHY_LOID, 11, 0, be_n(v, len > 4 ? 4 : len));
}

static void on_get(struct gpon_olt_omci *o, const u8 *pdu, u32 now)
{
	const struct gpon_olt_step *s = &o->step[o->cur];
	u16 want = be16(s->body);

	if (pdu[8] != OMCI_RC_OK) {
		fault(o, GPON_OLT_WHY_RESULT, only_attr(want), OMCI_RC_OK, pdu[8]);
		return;
	}
	if (be16(pdu + 9) != want) {
		fault(o, GPON_OLT_WHY_MASK, 0, want, be16(pdu + 9));
		return;
	}
	if (!store_attrs(o, s->class_id, s->inst, want, pdu + 11, GET_VALUES)) {
		fault(o, GPON_OLT_WHY_MIB_FULL, 0, GPON_OLT_MIB_ATTRS, o->nmib);
		return;
	}
	if (s->class_id == OMCI_ME_ONU_DATA) {
		if (pdu[11] != o->mds) {
			if (s->flags & F_AUDIT) {
				o->resyncs++;
				gpon_olt_omci_start(o, now);
				return;
			}
			fault(o, GPON_OLT_WHY_MDS, 1, o->mds, pdu[11]);
			return;
		}
		o->mib_synced = true;
	} else if (s->class_id == OMCI_ME_ONU_G) {
		check_identity(o);
		if (o->phase == GPON_OLT_PH_FAILED)
			return;
	}
	next_step(o);
}

static void on_upload(struct gpon_olt_omci *o, const u8 *pdu, u32 now)
{
	u16 class_id = be16(pdu + 8);

	o->upload_rows++;
	if (!class_id)
		o->upload_suspect++;
	else if (!store_attrs(o, class_id, be16(pdu + 10), be16(pdu + 12),
			      pdu + 14, UPLOAD_VALUES)) {
		fault(o, GPON_OLT_WHY_MIB_FULL, 0, GPON_OLT_MIB_ATTRS, o->nmib);
		return;
	}
	if (++o->sub_i < o->sub_n) {
		request_seq(o, OMCI_MT_MIB_UPLOAD_NX, OMCI_ME_ONU_DATA, 0,
			    o->sub_i, 0, 2, now);
		return;
	}
	next_step(o);
}

static void alarm_store(struct gpon_olt_omci *o, u16 class_id, u16 inst,
			const u8 *bitmap)
{
	static const u8 clear[28];
	unsigned int i;

	for (i = 0; i < o->nalarm; i++)
		if (o->alarm[i].class_id == class_id && o->alarm[i].inst == inst)
			break;
	if (!memcmp(bitmap, clear, sizeof(clear))) {
		if (i < o->nalarm)
			o->alarm[i] = o->alarm[--o->nalarm];
		return;
	}
	if (i == o->nalarm) {
		if (o->nalarm >= GPON_OLT_ALARMS) {
			ring(o, GPON_UNSUP_RANGE, GPON_OLT_ALARMS,
			     "room in the alarm table", bitmap, 28);
			return;
		}
		o->nalarm++;
	}
	o->alarm[i].class_id = class_id;
	o->alarm[i].inst = inst;
	memcpy(o->alarm[i].bitmap, bitmap, 28);
}

static void on_alarm_row(struct gpon_olt_omci *o, const u8 *pdu, u32 now)
{
	alarm_store(o, be16(pdu + 8), be16(pdu + 10), pdu + 12);
	if (++o->sub_i < o->sub_n) {
		request_seq(o, OMCI_MT_GET_ALL_ALRM_NX, OMCI_ME_ONU_DATA, 0,
			    o->sub_i, 0, 2, now);
		return;
	}
	next_step(o);
}

static void on_count(struct gpon_olt_omci *o, const u8 *pdu, u32 now)
{
	const struct gpon_olt_step *s = &o->step[o->cur];
	u16 n = be16(pdu + 8);
	bool upload = s->mt == OMCI_MT_MIB_UPLOAD;

	if ((upload && !n) || n > GPON_OLT_UPLOAD_MAX) {
		fault(o, GPON_OLT_WHY_UPLOAD_SIZE, 0, GPON_OLT_UPLOAD_MAX, n);
		return;
	}
	if (upload)
		mib_clear(o);
	else
		o->nalarm = 0;
	if (!n) {
		next_step(o);
		return;
	}
	o->sub = upload ? SUB_UPLOAD : SUB_ALARMS;
	o->sub_i = 0;
	o->sub_n = n;
	request_seq(o, upload ? OMCI_MT_MIB_UPLOAD_NX : OMCI_MT_GET_ALL_ALRM_NX,
		    OMCI_ME_ONU_DATA, 0, 0, 0, 2, now);
}

static void on_answer(struct gpon_olt_omci *o, const u8 *pdu, u32 now)
{
	const struct gpon_olt_step *s = &o->step[o->cur];

	switch (o->sub) {
	case SUB_UPLOAD:
		on_upload(o, pdu, now);
		return;
	case SUB_ALARMS:
		on_alarm_row(o, pdu, now);
		return;
	case SUB_READBACK:
		on_readback(o, pdu, now);
		return;
	case SUB_TABLE:
		on_table_part(o, pdu, now);
		return;
	}
	switch (s->mt) {
	case OMCI_MT_MIB_UPLOAD:
	case OMCI_MT_GET_ALL_ALARMS:
		on_count(o, pdu, now);
		return;
	case OMCI_MT_GET:
		on_get(o, pdu, now);
		return;
	}
	if (pdu[8] != OMCI_RC_OK) {
		fault(o, GPON_OLT_WHY_RESULT, 0, OMCI_RC_OK, pdu[8]);
		return;
	}
	if (s->mt == OMCI_MT_MIB_RESET) {
		o->mds = 0;
		o->alarm_seq = 0;
	}
	if (s->flags & F_MDS)
		mds_step(o);
	if (s->flags & F_READBACK) {
		o->rb_left = be16(s->body);
		readback_next(o, now);
		return;
	}
	next_step(o);
}

static void on_autonomous(struct gpon_olt_omci *o, const u8 *pdu)
{
	u16 class_id = be16(pdu + 4), inst = be16(pdu + 6);
	u8 seq = pdu[39];

	o->autonomous++;
	switch (pdu[2] & 0x1f) {
	case OMCI_MT_ALARM:
		o->alarm_rx++;
		alarm_store(o, class_id, inst, pdu + 8);
		if (o->alarm_seq && seq != (o->alarm_seq == 255 ? 1 : o->alarm_seq + 1)) {
			o->alarm_gap++;
			o->alarm_resync = true;
		}
		o->alarm_seq = seq;
		break;
	case OMCI_MT_AVC:
		o->avc++;
		if (!store_attrs(o, class_id, inst, be16(pdu + 8), pdu + 10,
				 AVC_VALUES))
			ring(o, GPON_UNSUP_RANGE, GPON_OLT_MIB_ATTRS,
			     "room in the MIB copy", pdu, GPON_OLT_OMCI_LEN);
		break;
	default:
		ring(o, GPON_UNSUP_UNKNOWN, pdu[2] & 0x1f,
		     "an alarm or an attribute value change", pdu,
		     GPON_OLT_OMCI_LEN);
		break;
	}
}

/* ---- the runner ---------------------------------------------------------- */

static void run_local(struct gpon_olt_omci *o, const struct gpon_olt_step *s,
		      u32 now)
{
	switch (s->inst) {
	case LOCAL_PLAN:
		plan_service(o);
		break;
	case LOCAL_GRAPH:
		check_graph(o);
		break;
	case LOCAL_ACS_DONE:
		o->acs = GPON_OLT_CHECK_PASS;
		break;
	case LOCAL_ONLINE:
		o->online = true;
		o->phase = GPON_OLT_PH_ONLINE;
		o->audit_at = now + o->prof.audit_ms;
		break;
	}
	if (o->phase != GPON_OLT_PH_FAILED)
		o->cur++;
}

static void advance(struct gpon_olt_omci *o, u32 now)
{
	while (!o->pending && o->phase != GPON_OLT_PH_FAILED &&
	       o->cur < o->nsteps) {
		const struct gpon_olt_step *s = &o->step[o->cur];

		if (s->flags & F_BAD) {
			fault(o, GPON_OLT_WHY_PLAN, 0, GPON_OLT_STEPS, o->nsteps);
			continue;
		}
		if (s->phase != GPON_OLT_PH_ONLINE || !o->online)
			o->phase = s->phase;
		if (!s->mt) {
			run_local(o, s, now);
			continue;
		}
		request(o, s->mt, s->class_id, s->inst, s->body, s->blen, now);
	}
}

void gpon_olt_profile_default(struct gpon_olt_profile *p)
{
	memset(p, 0, sizeof(*p));
	p->alloc_id = 257;
	p->gem_port = 257;
	p->vlan = 100;
	p->gem_payload = 1600;
	p->acs_tag = 0xffff;
	p->timeout_ms = 1000;
	p->retries = 2;
}

void gpon_olt_omci_init(struct gpon_olt_omci *o,
			const struct gpon_olt_omci_ops *ops, void *sh,
			u8 onu_id, const struct gpon_olt_profile *prof)
{
	memset(o, 0, sizeof(*o));
	o->ops = ops;
	o->sh = sh;
	o->onu_id = onu_id;
	o->prof = *prof;
	o->next_tci = 1;
}

void gpon_olt_omci_start(struct gpon_olt_omci *o, u32 now_ms)
{
	static const struct gpon_olt_fail none;

	o->starts++;
	o->pending = false;
	o->sub = SUB_NONE;
	o->nsteps = 0;
	o->cur = 0;
	o->phase = GPON_OLT_PH_RESET;
	o->mib_synced = false;
	o->configured = false;
	o->online = false;
	o->acs = GPON_OLT_CHECK_NONE;
	o->table_check = GPON_OLT_CHECK_NONE;
	o->fail = none;
	o->confirm_class = 0;
	o->confirm_attr = 0;
	o->acs_fail = none;
	o->table_fail = none;
	o->alarm_resync = false;
	add(o, GPON_OLT_PH_RESET, OMCI_MT_MIB_RESET, OMCI_ME_ONU_DATA, 0, 0);
	get(o, GPON_OLT_PH_IDENTITY, OMCI_ME_ONU_G, 0,
	    OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(3), 0);
	if (o->prof.check_loid) {
		get(o, GPON_OLT_PH_IDENTITY, OMCI_ME_ONU_G, 0, OMCI_ATTR_BIT(10), 0);
		get(o, GPON_OLT_PH_IDENTITY, OMCI_ME_ONU_G, 0, OMCI_ATTR_BIT(11), 0);
	}
	add(o, GPON_OLT_PH_UPLOAD, OMCI_MT_MIB_UPLOAD, OMCI_ME_ONU_DATA, 0, 0);
	add(o, GPON_OLT_PH_ALARMS, OMCI_MT_GET_ALL_ALARMS, OMCI_ME_ONU_DATA, 0,
	    0)->blen = 1;
	local(o, GPON_OLT_PH_PROVISION, LOCAL_PLAN, 0);
	advance(o, now_ms);
}

void gpon_olt_omci_stop(struct gpon_olt_omci *o)
{
	o->pending = false;
	o->sub = SUB_NONE;
	o->nsteps = 0;
	o->cur = 0;
	o->online = false;
	o->phase = GPON_OLT_PH_IDLE;
}

void gpon_olt_omci_rx(struct gpon_olt_omci *o, const u8 *pdu,
		      unsigned int len, u32 now_ms)
{
	o->rx++;
	if (!pdu || len < GPON_OLT_OMCI_LEN) {
		o->runt++;
		ring(o, GPON_UNSUP_RANGE, len, "a 48-octet baseline frame", pdu,
		     pdu ? len : 0);
		return;
	}
	if (pdu[3] != 0x0a) {
		ring(o, GPON_UNSUP_UNKNOWN, pdu[3], "device identifier 0x0a", pdu,
		     GPON_OLT_OMCI_LEN);
		return;
	}
	if (!omci_mic_ok(pdu, len)) {
		o->bad_mic++;
		if (!o->prof.mic_lenient)
			return;
	}
	if (!(pdu[2] & 0x20)) {
		on_autonomous(o, pdu);
		return;
	}
	if (!o->pending || be16(pdu) != be16(o->req)) {
		o->stale++;
		return;
	}
	if ((pdu[2] & 0x1f) != (o->req[2] & 0x1f) ||
	    memcmp(pdu + 4, o->req + 4, 4)) {
		fault(o, GPON_OLT_WHY_ECHO, 0, be_n(o->req + 4, 4),
		      be_n(pdu + 4, 4));
		advance(o, now_ms);
		return;
	}
	o->pending = false;
	on_answer(o, pdu, now_ms);
	advance(o, now_ms);
}

void gpon_olt_omci_tick(struct gpon_olt_omci *o, u32 now_ms)
{
	if (o->pending && reached(now_ms, o->sent_ms + o->prof.timeout_ms)) {
		if (o->tries < o->prof.retries) {
			o->tries++;
			o->retries++;
			transmit(o, now_ms);
			return;
		}
		fault(o, GPON_OLT_WHY_TIMEOUT, 0, o->prof.retries + 1u, 0);
	}
	if (o->online && !o->pending && o->prof.audit_ms &&
	    reached(now_ms, o->audit_at)) {
		o->nsteps = 0;
		o->cur = 0;
		if (o->alarm_resync) {
			o->alarm_resync = false;
			add(o, GPON_OLT_PH_ONLINE, OMCI_MT_GET_ALL_ALARMS,
			    OMCI_ME_ONU_DATA, 0, 0)->blen = 1;
		}
		get(o, GPON_OLT_PH_ONLINE, OMCI_ME_ONU_DATA, 0, OMCI_ATTR_BIT(1),
		    F_AUDIT);
		local(o, GPON_OLT_PH_ONLINE, LOCAL_ONLINE, 0);
	}
	advance(o, now_ms);
}

bool gpon_olt_omci_next_ms(const struct gpon_olt_omci *o, u32 *at)
{
	if (o->pending) {
		*at = o->sent_ms + o->prof.timeout_ms;
		return true;
	}
	if (o->online && o->prof.audit_ms) {
		*at = o->audit_at;
		return true;
	}
	return false;
}

const char *gpon_olt_phase_name(unsigned int phase)
{
	static const char *const names[] = {
		"idle", "reset", "identity", "upload", "alarms", "provision",
		"verify", "acs", "online", "failed",
	};

	return phase < ARRAY_SIZE(names) ? names[phase] : "?";
}

const char *gpon_olt_why_name(unsigned int why)
{
	static const char *const names[] = {
		"none", "timeout", "result", "echo", "readback", "mask", "mds",
		"serial", "loid", "upload-size", "mib-full", "no-tcont",
		"no-uni", "graph", "table", "plan", "no-tr069",
	};

	return why < ARRAY_SIZE(names) ? names[why] : "?";
}

const char *gpon_olt_check_name(unsigned int check)
{
	static const char *const names[] = {
		"not-asked", "pass", "fail", "could-not-ask",
	};

	return check < ARRAY_SIZE(names) ? names[check] : "?";
}
