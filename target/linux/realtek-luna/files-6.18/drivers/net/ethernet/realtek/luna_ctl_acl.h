/* SPDX-License-Identifier: GPL-2.0-only */
/* Control ethertypes stamped CTL_RX_PRIO by the switch's ingress ACL, so a frame to gpon0's own
 * MAC (a lookup hit at data priority) still reaches the control RX ring under a flood. */
#ifndef _LUNA_CTL_ACL_H
#define _LUNA_CTL_ACL_H
#include <linux/bits.h>
#include <linux/kernel.h>
#include "luna_l2_access.h"

#define LUNA_ACL_WORDS		5
#define LUNA_ACL_ACT_WORDS	3	/* the widest action entry (RTL9602C) */
#define LUNA_ACL_TOP_STORED	0x1ffffu	/* word 4: bits 128..143 and VALID (144) */
#define LUNA_ACL_ETHERTYPE	7		/* template field code */
#define LUNA_ACL_ACT_PRI	BIT(2)		/* rule action enable: priority */

/* A rule is an X/Y pair: its mask half at entry r and its data half at data_base + r of rule_tbl,
 * through the table-access port. */
struct luna_ctl_acl {
	u32 tmpl, en, permit, act_ctrl, pri_sel;	/* template 0, per-port enable/permit, rule actions, priority groups */
	bool care_is_one;	/* RTL9602C: data = care & value, mask = care & ~value, VALID in data only */
	u16 data_base;
	u8 rule_tbl, act_tbl, act_words, rules, ports, pri_shift, pri_groups, acl_weight_shift, addr_shift;
};

/* Template 0, FIELD0 [31:16] == ethertype, any port, valid, as the X/Y pair the TCAM stores:
 * data = ~care | value, mask = ~care | ~value, or its complement on a care_is_one die. */
static inline void luna_ctl_acl_rule(const struct luna_ctl_acl *a, u16 ethertype,
				     u32 data[LUNA_ACL_WORDS], u32 mask[LUNA_ACL_WORDS])
{
	const u32 care = 0xffff0007, value = (u32)ethertype << 16;	/* FIELD0, TYPE [2:0] = template 0 */
	unsigned int i;

	data[0] = ~care | value;
	mask[0] = ~care | ~value;
	for (i = 1; i < LUNA_ACL_WORDS; i++)
		data[i] = mask[i] = ~0u;
	data[LUNA_ACL_WORDS - 1] = mask[LUNA_ACL_WORDS - 1] = LUNA_ACL_TOP_STORED;
	if (!a->care_is_one)
		return;
	for (i = 0; i < LUNA_ACL_WORDS; i++) {
		u32 d = data[i];

		data[i] = ~mask[i];
		mask[i] = ~d;
	}
	data[LUNA_ACL_WORDS - 1] = BIT(16);	/* VALID (144); the mask half has none */
	mask[LUNA_ACL_WORDS - 1] = 0;
}

/* Every rule invalidated, then one per control ethertype; the ACL is enabled last, and stays off on
 * an error. -> 0 or the failing table write's error. */
static inline int luna_ctl_acl_install(void __iomem *sw, const struct luna_ctl_acl *a, u8 prio)
{
	static const u16 ethertypes[] = { 0x0806 /* ARP */, 0x8863 /* PPPoE discovery */ };
	static const u32 zero[LUNA_ACL_WORDS];
	const u32 act[LUNA_ACL_ACT_WORDS] = { 0, (u32)prio << a->pri_shift };	/* PRIACT 0 = assign, PRIDX = prio */
	u32 data[LUNA_ACL_WORDS], mask[LUNA_ACL_WORDS];
	unsigned int r;
	int ret;

	writel(0, sw + a->en);
	for (r = 0; r < a->rules; r++) {
		writel(0, sw + a->act_ctrl + 4 * r);
		ret = luna_tbl_write(sw, a->addr_shift, a->rule_tbl, r, zero, LUNA_ACL_WORDS) ?:
		      luna_tbl_write(sw, a->addr_shift, a->rule_tbl, a->data_base + r, zero, LUNA_ACL_WORDS) ?:
		      luna_tbl_write(sw, a->addr_shift, a->act_tbl, r, zero, a->act_words);
		if (ret)
			return ret;
	}
	writel(LUNA_ACL_ETHERTYPE, sw + a->tmpl);
	for (r = 0; r < ARRAY_SIZE(ethertypes); r++) {
		luna_ctl_acl_rule(a, ethertypes[r], data, mask);
		ret = luna_tbl_write(sw, a->addr_shift, a->act_tbl, r, act, a->act_words) ?:
		      luna_tbl_write(sw, a->addr_shift, a->rule_tbl, r, mask, LUNA_ACL_WORDS) ?:
		      luna_tbl_write(sw, a->addr_shift, a->rule_tbl, a->data_base + r, data, LUNA_ACL_WORDS);
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
