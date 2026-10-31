/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Is a received Ethernet frame link control traffic: ARP, PPPoE discovery, PPP's own protocols
 * (LCP, authentication, IPCP), DHCP, DHCPv6, ICMPv6 neighbour/router discovery and MLD, IGMP?
 * A driver records such a frame on its control RX queue, which carries no RPS map, so a flood
 * steered into another CPU's backlog never drops the lease, the PPP echo or the neighbour cache.
 * Pure byte math with no kernel API, so the x86 ladder tests the shipped code.
 */
#ifndef ONU_RX_CONTROL_H
#define ONU_RX_CONTROL_H

#define ONU_RX_QUEUE_DATA	0
#define ONU_RX_QUEUE_CONTROL	1
#define ONU_RX_QUEUES		2

static inline unsigned int onu_rx_be16(const unsigned char *p)
{
	return (unsigned int)p[0] << 8 | p[1];
}

static inline int onu_rx_ipv4_control(const unsigned char *ip, unsigned int len)
{
	unsigned int ihl;

	if (len < 20 || ip[0] >> 4 != 4)
		return 0;
	if (ip[9] == 2)					/* IGMP */
		return 1;
	ihl = (ip[0] & 0xf) * 4u;
	if (ip[9] != 17 || ihl < 20 || len < ihl + 4 || (onu_rx_be16(ip + 6) & 0x1fff))
		return 0;
	switch (onu_rx_be16(ip + ihl + 2)) {		/* UDP destination port */
	case 67: case 68:				/* DHCP */
		return 1;
	}
	return 0;
}

static inline int onu_rx_ipv6_control(const unsigned char *ip, unsigned int len)
{
	unsigned int next, pos = 40;

	if (len < 40 || ip[0] >> 4 != 6)
		return 0;
	next = ip[6];
	if (next == 0) {				/* hop-by-hop options: MLD's router alert */
		if (len < pos + 8)
			return 0;
		next = ip[pos];
		pos += (ip[pos + 1] + 1u) * 8;
	}
	if (len < pos + 4)
		return 0;
	if (next == 58)					/* ICMPv6: MLD 130-132, RS/RA/NS/NA/redirect 133-137 */
		return ip[pos] >= 130 && ip[pos] <= 137;
	if (next == 17) {
		unsigned int dport = onu_rx_be16(ip + pos + 2);

		return dport == 546 || dport == 547;	/* DHCPv6 */
	}
	return 0;
}

static inline int onu_rx_is_control(const unsigned char *frame, unsigned int len)
{
	unsigned int pos = 12, tags, type;

	for (tags = 0; tags < 3; tags++) {
		if (len < pos + 2)
			return 0;
		type = onu_rx_be16(frame + pos);
		if (type != 0x8100 && type != 0x88a8 && type != 0x9100)
			break;
		pos += 4;
	}
	if (len < pos + 2)
		return 0;
	type = onu_rx_be16(frame + pos);
	pos += 2;
	if (type == 0x8864) {				/* PPPoE session: the PPP protocol decides */
		if (len < pos + 8)
			return 0;
		type = onu_rx_be16(frame + pos + 6);
		pos += 8;
		if (type == 0x0021)
			return onu_rx_ipv4_control(frame + pos, len - pos);
		if (type == 0x0057)
			return onu_rx_ipv6_control(frame + pos, len - pos);
		return 1;				/* LCP, PAP, CHAP, IPCP, IPV6CP ... */
	}
	switch (type) {
	case 0x0806:					/* ARP */
	case 0x8863:					/* PPPoE discovery */
		return 1;
	case 0x0800:
		return onu_rx_ipv4_control(frame + pos, len - pos);
	case 0x86dd:
		return onu_rx_ipv6_control(frame + pos, len - pos);
	}
	return 0;
}

#endif
