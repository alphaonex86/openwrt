/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: FAMILY */
/* Control ethertypes stamped CTL_RX_PRIO by the switch's ingress ACL, so a frame to gpon0's own
 * MAC (a lookup hit at data priority) still reaches the control RX ring under a flood. */
#ifndef _LUNA_CTL_ACL_H
#define _LUNA_CTL_ACL_H
#include <linux/bits.h>
#include <linux/kernel.h>
#include "luna_l2_access.h"
#include "eth_tcam.h"	/* core: a rule's fields as its X/Y pair */

#define LUNA_ACL_WORDS		5
#define LUNA_ACL_ACT_WORDS	3	/* the widest action entry (RTL9602C) */
#define LUNA_ACL_TOP_STORED	0x1ffffu	/* word 4: bits 128..143 and VALID (144) */
#define LUNA_ACL_ETHERTYPE	7		/* template field code */
#define LUNA_ACL_ACT_PRI	BIT(2)		/* rule action enable: priority */
#define LUNA_ACL_ACT_LOG_FWD	(BIT(3) | BIT(4))	/* rule action enable: policing / log, forward */

/* A rule is an X/Y pair: its mask half at entry r and its data half at data_base + r of rule_tbl,
 * through the table-access port. */
struct luna_ctl_acl {
	u32 tmpl, en, permit, act_ctrl, pri_sel;	/* template 0, per-port enable/permit, rule actions, priority groups */
	bool care_is_one;	/* RTL9602C: data = care & value, mask = care & ~value, VALID in data only */
	u16 data_base;
	u8 rule_tbl, act_tbl, act_words, rules, ports, pri_shift, pri_groups, acl_weight_shift, addr_shift;
	/* the trap rule (0 = none): its template (fields DMAC octets 4-5, 2-3, 0-1, SIP low, high, DIP low,
	 * high, in the chip's codes), its action (trap, counted in one log counter), that counter's register */
	u8 trap_rule, trap_tmpl;
	u32 trap_tmpl_words[2], trap_act[LUNA_ACL_ACT_WORDS], trap_hits;
};

/* A rule's X/Y pair: TYPE [2:0] = its template, its fields, VALID (144) in the data half (care_is_one) or every
 * stored top bit free. */
static inline void luna_acl_xy(const struct luna_ctl_acl *a, const struct eth_tcam_field *f, unsigned int n,
			       u32 data[LUNA_ACL_WORDS], u32 mask[LUNA_ACL_WORDS])
{
	eth_tcam_xy(f, n, a->care_is_one, data, mask, LUNA_ACL_WORDS);
	if (a->care_is_one)
		data[LUNA_ACL_WORDS - 1] |= BIT(16);
	else
		data[LUNA_ACL_WORDS - 1] = mask[LUNA_ACL_WORDS - 1] = LUNA_ACL_TOP_STORED;
}

/* Template 0, FIELD0 [31:16] == ethertype, any port. */
static inline void luna_ctl_acl_rule(const struct luna_ctl_acl *a, u16 ethertype,
				     u32 data[LUNA_ACL_WORDS], u32 mask[LUNA_ACL_WORDS])
{
	const struct eth_tcam_field f[] = { { 0, 3, 7, 0 }, { 16, 16, 0xffff, ethertype } };

	luna_acl_xy(a, f, ARRAY_SIZE(f), data, mask);
}

/* The trap template's rule: IPv4 (TAGS bit 3), @t's DMAC, SIP and DIP from FIELD0, any port. */
static inline void luna_acl_trap_rule(const struct luna_ctl_acl *a, const struct eth_tcam_trap *t,
				      u32 data[LUNA_ACL_WORDS], u32 mask[LUNA_ACL_WORDS])
{
	struct eth_tcam_field f[2 + ETH_TCAM_TRAP_FIELDS] = { { 0, 3, 7, a->trap_tmpl }, { 6, 1, 1, 1 } };

	eth_tcam_trap_fields(f + 2, t, 16);
	luna_acl_xy(a, f, ARRAY_SIZE(f), data, mask);
}

/* Rule @r: its action, then its mask and data halves (the data half holds VALID). */
static inline int luna_acl_write(void __iomem *sw, const struct luna_ctl_acl *a, unsigned int r, const u32 *act,
				 const u32 *mask, const u32 *data)
{
	return luna_tbl_write(sw, a->addr_shift, a->act_tbl, r, act, a->act_words) ?:
	       luna_tbl_write(sw, a->addr_shift, a->rule_tbl, r, mask, LUNA_ACL_WORDS) ?:
	       luna_tbl_write(sw, a->addr_shift, a->rule_tbl, a->data_base + r, data, LUNA_ACL_WORDS);
}

/* The trap rule: trap to the CPU, counted in the log counter, its actions enabled last. -> 0 or the write's error
 * (the rule then has no action). */
static inline int luna_acl_trap_set(void __iomem *sw, const struct luna_ctl_acl *a, const struct eth_tcam_trap *t)
{
	u32 data[LUNA_ACL_WORDS], mask[LUNA_ACL_WORDS];
	int ret;

	luna_acl_trap_rule(a, t, data, mask);
	writel(0, sw + a->act_ctrl + 4 * a->trap_rule);
	ret = luna_acl_write(sw, a, a->trap_rule, a->trap_act, mask, data);
	if (!ret)
		writel(LUNA_ACL_ACT_LOG_FWD, sw + a->act_ctrl + 4 * a->trap_rule);
	return ret;
}

/* Rule @r invalid and without action. */
static inline int luna_acl_off(void __iomem *sw, const struct luna_ctl_acl *a, unsigned int r)
{
	static const u32 zero[LUNA_ACL_WORDS];

	writel(0, sw + a->act_ctrl + 4 * r);
	return luna_acl_write(sw, a, r, zero, zero, zero);
}

/* Every rule invalidated, then one per control ethertype; the ACL is enabled last, and stays off on
 * an error. -> 0 or the failing table write's error. */
static inline int luna_ctl_acl_install(void __iomem *sw, const struct luna_ctl_acl *a, u8 prio)
{
	static const u16 ethertypes[] = { 0x0806 /* ARP */, 0x8863 /* PPPoE discovery */ };
	const u32 act[LUNA_ACL_ACT_WORDS] = { 0, (u32)prio << a->pri_shift };	/* PRIACT 0 = assign, PRIDX = prio */
	u32 data[LUNA_ACL_WORDS], mask[LUNA_ACL_WORDS];
	unsigned int r;
	int ret;

	writel(0, sw + a->en);
	for (r = 0; r < a->rules; r++) {
		ret = luna_acl_off(sw, a, r);
		if (ret)
			return ret;
	}
	writel(LUNA_ACL_ETHERTYPE, sw + a->tmpl);
	if (a->trap_rule) {
		writel(a->trap_tmpl_words[0], sw + a->tmpl + 8 * a->trap_tmpl);
		writel(a->trap_tmpl_words[1], sw + a->tmpl + 8 * a->trap_tmpl + 4);
	}
	for (r = 0; r < ARRAY_SIZE(ethertypes); r++) {
		luna_ctl_acl_rule(a, ethertypes[r], data, mask);
		ret = luna_acl_write(sw, a, r, act, mask, data);
		if (ret)
			return ret;
		writel(LUNA_ACL_ACT_PRI, sw + a->act_ctrl + 4 * r);
	}
	for (r = 0; r < a->pri_groups; r++)	/* every priority group: the ACL's priority wins */
		writel(readl(sw + a->pri_sel + 4 * r) | (7u << a->acl_weight_shift), sw + a->pri_sel + 4 * r);
	writel(a->ports, sw + a->permit);	/* an unmatched frame passes, BEFORE the ACL is on */
	writel(a->ports, sw + a->en);
	return 0;
}
#endif
