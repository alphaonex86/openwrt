// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE */
/* gpon_olt_port.c -- one OLT PON port: activation, OMCI engines, authorization (see the header). */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "gpon_olt_platform.h"	/* GPON_OLT_ID_MAX */
#include "gpon_olt_port.h"
#include "gpon_sn.h"

static void port_ds(void *sh, const u8 m[GPON_OLT_PLOAM_DS_LEN])
{
	struct gpon_olt_port *port = sh;

	port->ops->ds_ploam(port->sh, m);
}

static void port_sn_grant(void *sh)
{
	struct gpon_olt_port *port = sh;

	port->ops->sn_grant(port->sh);
}

static void port_ranging_grant(void *sh, u8 onu_id)
{
	struct gpon_olt_port *port = sh;

	port->ops->ranging_grant(port->sh, onu_id);
}

static void port_event(struct gpon_olt_port *port, u8 onu_id, const char *what, const char *detail)
{
	if (port->ops->event)
		port->ops->event(port->sh, onu_id, what, detail);
}

static void port_link(void *sh, u8 onu_id, enum gpon_olt_link link, enum gpon_olt_down why)
{
	struct gpon_olt_port *port = sh;
	struct gpon_olt_link_rec *r = gpon_olt_pon_find(&port->pon, onu_id);
	char sn[GPON_SN_TEXT_SIZE];
	unsigned int slot = onu_id - 1u;

	if (!r)
		return;
	if (slot >= port->n_eng) {
		port_event(port, onu_id, "no OMCI engine for this ONU-ID", gpon_olt_link_name(link));
		return;
	}
	port_event(port, onu_id, gpon_olt_link_name(link), why ? gpon_olt_down_name(why) : NULL);
	if (link == GPON_OLT_LINK_AUTOFIND && port->zero_touch) {
		gpon_sn_format(r->sn, sn);
		if (gpon_olt_pon_authorize(&port->pon, r->sn, NULL,
					   (u16)(port->tmpl.alloc_id + onu_id)) != onu_id)
			port_event(port, onu_id, "not authorized", sn);
		else
			port_event(port, onu_id, "authorized", sn);
	}
	if (link == GPON_OLT_LINK_UP)
		gpon_olt_omci_start(&port->eng[slot], port->pon.onu[slot].since_ms);
	else
		gpon_olt_omci_stop(&port->eng[slot]);
}

static int port_omci_tx(void *sh, u8 onu_id, const u8 *pdu, unsigned int len)
{
	struct gpon_olt_port *port = sh;
	struct gpon_olt_link_rec *r = gpon_olt_pon_find(&port->pon, onu_id);

	if (!r || len != GPON_OLT_OMCI_LEN)
		return -EINVAL;
	return port->ops->ds_omci(port->sh, r->omcc_gem, pdu, len);
}

static void port_unsup(void *sh, const char *kind, int cls, u32 val, const char *want, const u8 *data,
		       unsigned int len)
{
	struct gpon_olt_port *port = sh;

	port->ops->unsupported(port->sh, kind, cls, val, want, data, len);
}

static const struct gpon_olt_ploam_ops PORT_PLOAM = {
	.ds_tx = port_ds, .sn_grant = port_sn_grant, .ranging_grant = port_ranging_grant,
	.link_change = port_link, .unsupported = port_unsup,
};

static const struct gpon_olt_omci_ops PORT_OMCI = {
	.omci_tx = port_omci_tx, .unsupported = port_unsup,
};

const char *gpon_olt_port_init(struct gpon_olt_port *port, const struct gpon_olt_port_ops *ops, void *sh,
			       struct gpon_olt_omci *eng, unsigned int n_eng,
			       const struct gpon_olt_profile *tmpl, bool zero_touch)
{
	struct gpon_olt_profile prof;
	unsigned int i;

	if (!ops || !ops->ds_ploam || !ops->sn_grant || !ops->ranging_grant || !ops->ds_omci || !ops->unsupported)
		return "ops";
	if (!eng || !n_eng || n_eng > GPON_OLT_PON_ONUS)
		return "n_eng";
	if (!tmpl || tmpl->alloc_id + n_eng > GPON_OLT_ID_MAX || tmpl->gem_port + n_eng > GPON_OLT_ID_MAX)
		return "tmpl";
	memset(port, 0, sizeof(*port));
	port->ops = ops;
	port->sh = sh;
	port->eng = eng;
	port->n_eng = n_eng;
	port->tmpl = *tmpl;
	port->zero_touch = zero_touch;
	gpon_olt_pon_init(&port->pon, &PORT_PLOAM, port);
	for (i = 0; i < n_eng; i++) {
		prof = *tmpl;
		prof.alloc_id = (u16)(tmpl->alloc_id + gpon_olt_onu_id(i));
		prof.gem_port = gpon_olt_port_gem(port, i);
		gpon_olt_omci_init(&eng[i], &PORT_OMCI, port, gpon_olt_onu_id(i), &prof);
	}
	return NULL;
}

int gpon_olt_port_authorize(struct gpon_olt_port *port, const u8 sn[8], const u8 *password)
{
	unsigned int i;

	/* the slot gpon_olt_pon_authorize() will take: the serial's own, else the first free */
	for (i = 0; i < GPON_OLT_PON_ONUS && !(port->pon.onu[i].used && !memcmp(port->pon.onu[i].sn, sn, 8)); i++)
		;
	if (i == GPON_OLT_PON_ONUS)
		for (i = 0; i < GPON_OLT_PON_ONUS && port->pon.onu[i].used; i++)
			;
	if (i >= port->n_eng)
		return -ENOSPC;
	if (gpon_olt_pon_authorize(&port->pon, sn, password, (u16)(port->tmpl.alloc_id + gpon_olt_onu_id(i))) !=
	    gpon_olt_onu_id(i))
		return -EIO;
	memcpy(port->eng[i].prof.sn, sn, 8);	/* a serial named by the operator is checked again by OMCI */
	return gpon_olt_onu_id(i);
}

void gpon_olt_port_us_ploam(struct gpon_olt_port *port, const u8 *m, unsigned int len, u32 now_ms)
{
	gpon_olt_pon_us(&port->pon, m, len, now_ms);
}

void gpon_olt_port_ranging_burst(struct gpon_olt_port *port, u8 onu_id, u32 rtd, u32 now_ms)
{
	gpon_olt_pon_ranging_burst(&port->pon, onu_id, rtd, now_ms);
}

void gpon_olt_port_us_omci(struct gpon_olt_port *port, u16 gem_port, const u8 *pdu, unsigned int len,
			   u32 now_ms)
{
	unsigned int i;

	for (i = 0; i < port->n_eng; i++) {
		const struct gpon_olt_link_rec *r = &port->pon.onu[i];

		if (r->used && r->state == GPON_OLT_LINK_UP && r->omcc_gem == gem_port) {
			gpon_olt_omci_rx(&port->eng[i], pdu, len, now_ms);
			return;
		}
	}
}

void gpon_olt_port_tick(struct gpon_olt_port *port, u32 now_ms)
{
	unsigned int i;

	gpon_olt_pon_tick(&port->pon, now_ms);
	for (i = 0; i < port->n_eng; i++) {
		const struct gpon_olt_omci *e = &port->eng[i];

		gpon_olt_omci_tick(&port->eng[i], now_ms);
		if (e->phase == port->phase_seen[i])
			continue;
		port->phase_seen[i] = e->phase;
		port_event(port, gpon_olt_onu_id(i), gpon_olt_phase_name(e->phase),
			   e->fail.why ? gpon_olt_why_name(e->fail.why) : e->online ? "ONLINE" : NULL);
	}
}

bool gpon_olt_port_online(const struct gpon_olt_port *port, unsigned int slot)
{
	return slot < port->n_eng && port->eng[slot].online;
}

int gpon_olt_port_slot_of_gem(const struct gpon_olt_port *port, u16 gem_port)
{
	unsigned int i;

	for (i = 0; i < port->n_eng; i++)
		if (gpon_olt_port_gem(port, i) == gem_port)
			return gpon_olt_port_online(port, i) ? (int)i : -1;
	return -1;
}
