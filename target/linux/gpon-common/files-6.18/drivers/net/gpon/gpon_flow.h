/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE (drivers/net/gpon).  ⚠ NOT part of the STRICT subset -- this file
 * uses Linux's flow-offload API on purpose, which the widened core admits and
 * the strict subset does not.  See "THE THREE TIERS" in gpon_common.h: the line
 * is the REGISTER, not Linux.  Do not add this to gpon_layer_hostbuild_test.
 *
 * gpon_flow -- what a hardware-offloaded flow IS, decided once.
 *
 * ★ WHY IT IS CORE.  "Read a TC flow rule, refuse what this family of silicon
 * cannot do, and hand back the 5-tuple" is not a fact about any accelerator: it
 * is the same decode in every driver that offloads NAPT, and it is the half
 * that a second family would otherwise write again from the same kernel
 * headers.  What stays in the driver is the half that IS a hardware fact -- how
 * that 5-tuple is packed into ITS engine's key registers.
 *
 * ★ IT IS NOT A DUPLICATE TODAY, and that was checked before writing it: the
 * Luna side does no TC offload at all (zero flow_rule_match_*, zero setup_tc),
 * so there is no second copy to converge.  It is written here rather than
 * beside the one caller so that the second family costs a call instead of a
 * decode -- the standing rule that every board must cost less than the one
 * before.
 */
#ifndef GPON_FLOW_H
#define GPON_FLOW_H

#include <linux/types.h>

struct flow_rule;

/*
 * The 5-tuple, in HOST byte order and with no hardware in sight.
 *
 * ⚠ HOST ORDER IS A DECISION, NOT AN ACCIDENT.  The wire hands these over
 * big-endian and every engine wants them in its own arrangement; converting
 * ONCE here means a driver that gets it wrong is wrong in one place, and a core
 * that is fuzzed on x86 and on MIPS-BE cannot hide an endianness bug in the
 * decode.
 */
struct gpon_flow_key {
	u32 ip_sa;		/* source address, host order		*/
	u32 ip_da;		/* destination address, host order	*/
	u16 l4_sport;
	u16 l4_dport;
	u8  ip_protocol;	/* IPPROTO_TCP or IPPROTO_UDP		*/
	u8  ip_ver;		/* 0 = IPv4.  IPv6 is not decoded yet	*/
};

/*
 * Decode one TC flow rule into a key. -> 0, or -EOPNOTSUPP.
 *
 * ★ THE REFUSALS ARE THE POINT, and they are deliberately the same three the
 * Cortina driver already made: not IPv4, not TCP/UDP, or no L4 ports.  A driver
 * that offloads something it did not understand installs a rule that forwards
 * traffic somewhere nobody chose, and on this hardware that is invisible until
 * a user notices their connection went to the wrong place.
 *
 * ⚠ IT DOES NOT LOOK AT ACTIONS, and that is the seam: mangles, VLAN pushes and
 * the next-hop MAC are what the ENGINE has to be told, so they stay with the
 * engine.  This answers only "which flow", never "what to do to it".
 */
int gpon_flow_key_from_tc(struct flow_rule *rule, struct gpon_flow_key *key);

#endif /* GPON_FLOW_H */
