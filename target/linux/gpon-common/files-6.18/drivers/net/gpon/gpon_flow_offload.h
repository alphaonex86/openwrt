/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE.  ⚠ NOT in the STRICT host-buildable subset -- it uses Linux's TC
 * offload API and an rhashtable on purpose.  See "THE THREE TIERS" in
 * gpon_common.h: the line is the REGISTER, not Linux.
 *
 * gpon_flow_offload -- the TC hardware-offload LIFECYCLE, once.
 *
 * ★★ WHY THIS EXISTS (operator, 2026-08-28): *"generaliza para todos los
 * hardware que no hacen offload y los que sí, así en la familia es solo un
 * #ifdef"*.  A family whose silicon has a flow accelerator supplies the ops
 * below; a family without one compiles none of this (CONFIG_GPON_FLOW_OFFLOAD
 * is not set, and on Luna that is not an object saved -- it is a fatal error
 * avoided, because <net/flow_offload.h> pulls in a TC stack that target does
 * not build).
 *
 * ─────────────────────────────────────────────────────────────────────────
 * ★★★ THE SEAM IS NOT WHERE I FIRST DREW IT, AND THE CODE SAID SO.
 *
 * The first version of this header offered "3 questions + 3 engine calls":
 * is_lan_side / wan_vid / wan_vlan_programmable, then install/remove/stats.
 * That contract is WRONG, and only READING the one existing implementation
 * showed it -- no classifier could have.  `cn_flow_replace()` is ~850 lines,
 * and its generic and silicon halves are not stacked, they are INTERLEAVED:
 * a PPPoE leg gate whose input is a driver-held session SHADOW; a lazy
 * one-time DS arm that touches hardware; an egress L3-IF program; a VLAN
 * readback BY LITERAL BIT NUMBER out of the FIB table.  None of that is a
 * fact about TC, and a core that reached for `wan_vid()` would have been
 * reaching for the two or three of those it happened to be able to name.
 *
 * So the seam is drawn where it actually falls:
 *
 *   CORE  -- what nf_flow_table means, and what a cookie map costs:
 *            the dedup, the action DECODE, the NAT-shape refusal, the entry
 *            allocation, the insert with its undo, destroy, stats.
 *   FAMILY-- ONE call, `prepare`, holding every decision that needs to read
 *            or write silicon, and `install`/`remove`/`stats` under it.
 *
 * `prepare` is deliberately allowed to be large.  It is not a wart: a 400-line
 * prepare in the family is exactly the code that MUST NOT be shared, and
 * naming it once is what lets the 200 lines around it be shared at all.
 * ─────────────────────────────────────────────────────────────────────────
 */
#ifndef GPON_FLOW_OFFLOAD_H
#define GPON_FLOW_OFFLOAD_H

#include <linux/types.h>

#include "gpon_flow.h"

struct net_device;
struct flow_cls_offload;

/*
 * WHAT TO DO to the flow, in vocabulary no accelerator owns.
 *
 * ★ ONE address and ONE port, not two.  That is nf_flow_table's shape, not a
 * limitation we chose: a masqueraded flow is offered as TWO rules that mirror
 * each other (flow_offload_ipv4_snat / _port_snat), the ORIGINAL rewriting the
 * source and the REPLY restoring the destination.  A doubly-NAT'd flow emits
 * both mangles on one leg and is refused -- correctly, because an entry
 * carries one rewrite.
 */
struct gpon_flow_act {
	u32 nat_addr;		/* host order					*/
	u16 nat_port;		/* host order					*/
	u16 pppoe_sid;		/* 0 = not a PPPoE leg; the LIVE negotiated id	*/
	u8  gw_dmac[6];		/* next hop, from the rule's ETH mangle		*/
	u8  nat_is_da;		/* 0 = rewrite the SA (US), 1 = the DA (DS)	*/
	u8  nat_valid;
	u8  port_valid;
	u8  dmac_valid;		/* BOTH ETH mangle halves were seen		*/
};

/*
 * The devices the rule names, and which leg this is.  The core holds a
 * REFERENCE on `idev` for the whole of `prepare` and `install`, because the
 * one existing implementation needs it there and dropping it earlier would
 * force the family to re-derive a device it was already handed.
 */
struct gpon_flow_ctx {
	struct net_device *idev;	/* the rule's META ingress	*/
	struct net_device *odev;	/* the REDIRECT target		*/
	bool ds_leg;			/* the WAN->LAN reply leg	*/
};

/*
 * ⚠ `install` OWNS THE INDEX IT RETURNS.  The core stores it against the
 * cookie and hands it back on remove; it never interprets it.  An engine whose
 * indices mean something (a hash bucket, a table row) keeps that meaning
 * private, which is what stops the core growing a second idea of the hardware.
 */
struct gpon_flow_ops {
	/* Which side of the box is this netdev on?  The core needs it before it
	 * can decode the actions at all: the leg discriminates every mangle. */
	bool (*is_lan_side)(void *sh, struct net_device *dev);

	/*
	 * EVERY decision that must read or write silicon.  Returns 0 to go
	 * ahead, or a negative errno -- -EOPNOTSUPP means "this flow stays on
	 * the software fastpath", which is a normal outcome and not an error.
	 * May amend `act` (a family that resolves the PPPoE session from its
	 * own WAN chain writes it here).  Runs BEFORE any entry exists, so a
	 * refusal costs no unwind.
	 */
	int (*prepare)(void *sh, const struct gpon_flow_key *key,
		       struct gpon_flow_act *act,
		       const struct gpon_flow_ctx *ctx, void *priv);

	int (*install)(void *sh, const struct gpon_flow_key *key,
		       const struct gpon_flow_act *act,
		       const struct gpon_flow_ctx *ctx, void *priv,
		       u32 *idx_out);
	int (*remove)(void *sh, u32 idx, void *priv);

	/* Fill `*lastused` (jiffies).  A family with no per-flow counters
	 * reports LIVENESS only, which is what TC actually asks for. */
	int (*stats)(void *sh, u32 idx, void *priv, unsigned long *lastused);

	/* Optional: a VLAN push/pop on the rule is always refused (no engine
	 * here can express a tag as a hit-action), but a family that keeps a
	 * refusal ledger wants to ATTRIBUTE it rather than count it as one of
	 * N anonymous unsupported reasons. */
	void (*note_vlan_action)(void *sh, bool ds_leg, u16 vid);

	/*
	 * Bytes of per-entry family state, appended to the core's entry and
	 * handed back as `priv` on every call.  ONE allocation, and the family
	 * keeps its own fields (a CRC, an age bucket, a table reference)
	 * without the core learning what they are.
	 */
	size_t priv_size;
};

struct gpon_flow_offload;

struct gpon_flow_offload *gpon_flow_offload_new(const struct gpon_flow_ops *ops,
						void *sh);
void gpon_flow_offload_free(struct gpon_flow_offload *fo);

/* The three TC verbs, dispatched by cookie. */
int gpon_flow_offload_replace(struct gpon_flow_offload *fo,
			      struct flow_cls_offload *f,
			      struct net_device *blockdev);
int gpon_flow_offload_destroy(struct gpon_flow_offload *fo,
			      struct flow_cls_offload *f);
int gpon_flow_offload_stats(struct gpon_flow_offload *fo,
			    struct flow_cls_offload *f);

/*
 * Decode a rule's ACTIONS.  Split out of replace() so a family that still owns
 * its own lifecycle can adopt the decode alone -- and so it can be tested
 * without a device.
 */
int gpon_flow_act_from_tc(struct gpon_flow_offload *fo, struct flow_rule *rule,
			  bool ds_leg, struct gpon_flow_act *act,
			  struct net_device **odev_out);

#endif /* GPON_FLOW_OFFLOAD_H */
