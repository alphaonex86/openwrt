// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gpon_flow -- see gpon_flow.h for why this is core and why it is NOT in the
 * strict, host-buildable subset.
 */
#include <linux/errno.h>
#include <linux/in.h>
#include <net/flow_offload.h>

#include "gpon_flow.h"

int gpon_flow_key_from_tc(struct flow_rule *rule, struct gpon_flow_key *key)
{
	u16 addr_type = 0;

	if (!rule || !key)
		return -EINVAL;

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_CONTROL)) {
		struct flow_match_control m;

		flow_rule_match_control(rule, &m);
		addr_type = m.key->addr_type;
	}
	if (addr_type != FLOW_DISSECTOR_KEY_IPV4_ADDRS)
		return -EOPNOTSUPP;	/* IPv4 NAPT only, as before */

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_BASIC))
		return -EOPNOTSUPP;
	else {
		struct flow_match_basic m;

		flow_rule_match_basic(rule, &m);
		if (m.key->ip_proto != IPPROTO_TCP &&
		    m.key->ip_proto != IPPROTO_UDP)
			return -EOPNOTSUPP;
		key->ip_protocol = m.key->ip_proto;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_IPV4_ADDRS)) {
		struct flow_match_ipv4_addrs m;

		flow_rule_match_ipv4_addrs(rule, &m);
		key->ip_sa = be32_to_cpu(m.key->src);
		key->ip_da = be32_to_cpu(m.key->dst);
	}

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_PORTS))
		return -EOPNOTSUPP;
	else {
		struct flow_match_ports m;

		flow_rule_match_ports(rule, &m);
		key->l4_sport = be16_to_cpu(m.key->src);
		key->l4_dport = be16_to_cpu(m.key->dst);
	}

	key->ip_ver = 0;		/* IPv4 */
	return 0;
}
