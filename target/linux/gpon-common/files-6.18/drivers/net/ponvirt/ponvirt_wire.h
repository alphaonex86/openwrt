/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE */
/*
 * ponvirt_wire.h -- the virtual ODN's wire, shared by the guest (ponvirt.c) and the host hub
 * (dev/virt/vodn.c): one Ethernet frame of PONVIRT_ETHERTYPE per PON event, byte 0 its kind,
 * multi-octet fields big-endian. Downstream is broadcast to every ONU, as on a fibre; each
 * ONU keeps what its ONU-ID or its GEM ports select. Frame level only: no optics, no BWmap.
 */
#ifndef PONVIRT_WIRE_H
#define PONVIRT_WIRE_H

#define PONVIRT_ETHERTYPE	0x88b5	/* IEEE local experimental */
#define PONVIRT_MTU		1600	/* a whole Ethernet frame plus the GEM header below */

enum ponvirt_kind {
	PV_DS_PLOAM = 1,	/* 13 octets */
	PV_US_PLOAM,		/* 12 octets */
	PV_SN_GRANT,		/* none: an ONU in O3 answers with its Serial_Number_ONU */
	PV_RANGING_GRANT,	/* ONU-ID */
	PV_RANGING_BURST,	/* ONU-ID: the granted ONU's ranging answer */
	PV_DS_OMCI,		/* GEM Port-ID (2) + 48 octets */
	PV_US_OMCI,		/* GEM Port-ID (2) + 48 octets */
	PV_DS_GEM,		/* GEM Port-ID (2) + an Ethernet frame */
	PV_US_GEM,		/* Alloc-ID (2) + GEM Port-ID (2) + an Ethernet frame */
	PV_US_LIGHT,		/* ONU-ID: a ranged ONU's upstream bursts, every PV_LIGHT_MS */
};

#define PV_LIGHT_MS	100	/* an ONU from O4 on is lit: its bursts reach the OLT this often */
#define PV_LOSI_MS	1000	/* an OLT declares LOSi for an ONU silent this long (ten missed bursts) */
#define PV_LOS_MS	1000	/* an ONU declares LOS on a downstream silent this long (the OLT broadcasts every 100 ms) */

#define PV_GEM_HDR	3	/* kind + GEM Port-ID */
#define PV_US_GEM_HDR	5	/* kind + Alloc-ID + GEM Port-ID */

/*
 * An ONU's downstream LOS: nothing heard for PV_LOS_MS. A tick later than half that means the guest itself
 * stalled (an emulator under load), not the fibre: the silence is then counted again from that tick.
 */
static inline bool pv_ds_los(uint32_t now, uint32_t *heard_ms, uint32_t *tick_ms)
{
	if (now - *tick_ms > PV_LOS_MS / 2)
		*heard_ms = now;
	*tick_ms = now;
	return now - *heard_ms > PV_LOS_MS;
}

#endif
