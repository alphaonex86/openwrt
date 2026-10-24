/* SPDX-License-Identifier: GPL-2.0-or-later */
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
};

#define PV_GEM_HDR	3	/* kind + GEM Port-ID */
#define PV_US_GEM_HDR	5	/* kind + Alloc-ID + GEM Port-ID */

#endif
