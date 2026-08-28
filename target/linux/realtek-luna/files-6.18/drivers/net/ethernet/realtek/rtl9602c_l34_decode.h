/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * l34_decode.h -- the PURE half of the RTL9602C flow-offload glue.
 *
 * ★ WHY IT IS SEPARATE (project rule): "Keep protocol logic decoupled from HW
 *   I/O (functional core / imperative shell) so it compiles and fuzzes on x86",
 *   and "PREFER THE OFFLINE ADVERSARIAL PROOF ... the board only CONFIRMS, it
 *   does not DISCOVER."
 *
 * The kernel shim extracts plain scalars from a flow_cls_offload rule; THIS
 * decides what goes into `struct l34_flow`. Nothing here touches MMIO, a
 * device pointer, a lock, an allocator or a clock -- so the same code that runs
 * on the board is what the host test exercises.
 *
 * ⚠⚠ THE TRAP THIS EXISTS TO CLOSE. In `struct l34_flow`, a post-NAT field of
 * ZERO means "unchanged", not "rewrite to 0.0.0.0" and not "port 0". A decode
 * that copies the conntrack reply tuple blindly writes a rewrite where none was
 * asked for -- and the hardware then installs a perfectly healthy-looking entry
 * that BLACKHOLES the flow. This project has already paid for that exact shape
 * once ("plumb the VALUE, not the flag"), which is why the rule below is a
 * function with tests rather than four assignments at a call site.
 */
#ifndef L34_DECODE_H
#define L34_DECODE_H

/* ★★ ONE HEADER, TWO BUILDS -- and that is the point (2026-08-23). This file
 * lives in the DRIVER tree and the host test includes it FROM HERE, so what the
 * x86 test proves is the code that actually ships. A test that validates its own
 * private copy proves the copy; this project has been bitten by that shape and
 * the rule is the same one behind gen_driver_consts.sh -- assert against the
 * REAL constants, not a re-typed set. */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stdint.h>
#include <string.h>
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
#endif

/* Mirrors the kernel's struct l34_flow (rtl9602c_l34.h) field for field. */
struct l34_flow_d {
	u8  l4proto;
	u32 orig_sip, orig_dip;
	u16 orig_sport, orig_dport;
	u32 nat_sip, nat_dip;
	u16 nat_sport, nat_dport;
	u8  egress_netif;
	u8  nexthop;
	u16 hw_index;
};

/* What the shim hands over: the ORIGINAL direction and what NAT turned it into.
 * Both come from the offload rule; neither is invented here. */
struct l34_decode_in {
	u8  l4proto;
	u32 o_sip, o_dip;      u16 o_sport, o_dport;   /* original */
	u32 r_sip, r_dip;      u16 r_sport, r_dport;   /* post-NAT  */
	u8  egress_netif, nexthop;
	int      have_reply;        /* 0 = no NAT rewrite was described at all */
};

enum {
	L34_DEC_OK = 0,
	L34_DEC_BAD_PROTO,      /* the engine keys on TCP/UDP only            */
	L34_DEC_NO_TUPLE,       /* a zero address or port in the ORIGINAL side */
};

#define L34_IPPROTO_TCP 6
#define L34_IPPROTO_UDP 17

/*
 * Decode ONE rule. -> L34_DEC_*, filling `out` only on OK.
 *
 * ★ THE RULE, and every clause is a test below:
 *   - only TCP and UDP are keyed by this engine; anything else is REFUSED
 *     rather than installed, because an entry the hardware cannot match is an
 *     entry that silently drops what it was meant to accelerate;
 *   - the ORIGINAL 5-tuple must be complete: a zero address or port there is
 *     not a wildcard, it is a rule we failed to read;
 *   - a post-NAT field is written ONLY where it DIFFERS from the original.
 *     Equal means "unchanged", and unchanged is expressed as ZERO -- never by
 *     repeating the value, which the hardware would read as a rewrite.
 */
static inline int l34_decode(const struct l34_decode_in *in,
			     struct l34_flow_d *out)
{
	if (!in || !out)
		return L34_DEC_NO_TUPLE;
	if (in->l4proto != L34_IPPROTO_TCP && in->l4proto != L34_IPPROTO_UDP)
		return L34_DEC_BAD_PROTO;
	if (!in->o_sip || !in->o_dip || !in->o_sport || !in->o_dport)
		return L34_DEC_NO_TUPLE;

	memset(out, 0, sizeof(*out));
	out->l4proto    = in->l4proto;
	out->orig_sip   = in->o_sip;
	out->orig_dip   = in->o_dip;
	out->orig_sport = in->o_sport;
	out->orig_dport = in->o_dport;
	out->egress_netif = in->egress_netif;
	out->nexthop      = in->nexthop;

	if (!in->have_reply)
		return L34_DEC_OK;      /* routed, not NAT'd: all rewrites stay 0 */

	/* ONLY the fields that actually change. See the trap note above. */
	if (in->r_sip   && in->r_sip   != in->o_sip)   out->nat_sip   = in->r_sip;
	if (in->r_dip   && in->r_dip   != in->o_dip)   out->nat_dip   = in->r_dip;
	if (in->r_sport && in->r_sport != in->o_sport) out->nat_sport = in->r_sport;
	if (in->r_dport && in->r_dport != in->o_dport) out->nat_dport = in->r_dport;
	return L34_DEC_OK;
}

#endif /* L34_DECODE_H */
