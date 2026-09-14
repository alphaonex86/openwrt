/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cortina_vlan_install.h -- the CORTINA family's install for the OMCI WAN
 * service model's VLAN half: `struct gpon_vlan_ops`, filled.
 *
 * TIER: FAMILY (prefix cortina_).  The core (gpon_omci_vlan.h) DECIDES what
 * the OLT asked for; this turns one decided rule into the DMA-AFT egress VLAN
 * edit and programs it.  Per this tree's hoist rule, FIELD LAYOUTS live here
 * and register ADDRESSES stay in the shell -- the shell fills
 * struct cortina_vlan_regs from cortina-ni-regs.h, so this object names no
 * address and builds on x86.
 *
 * ★★ WHY THIS EXISTS AT ALL.  Until 2026-09-14 `struct gpon_vlan_ops` was
 *    declared and NO family filled it, so every ME 171 row an OLT wrote was
 *    decided and then counted as owed.  A model nobody installs is a model.
 *
 * ★ AND IT DEDUPLICATES RATHER THAN ADDING A COPY.  cortina-ni-flowoffload.c
 *   already packed these three words inside cn_aft_fib_program(); that packing
 *   moved HERE and the shell calls it, so the word layout exists once and is
 *   driven on x86 by dev/rtl9607c-test/gpon_vlan_install_diff_test.
 *
 * ⚠ WHAT THE DMA-AFT CANNOT DO, AND THE INSTALL REFUSES RATHER THAN PRETENDS:
 *   it is a header EDIT, not a filter -- it has no match half at all, so a
 *   DISCARD row and any row whose filter is narrower than "everything from
 *   this lspid" cannot be expressed and come back -EOPNOTSUPP.  Refusing is
 *   the point: gpon_vlan_rule_emit() then records GPON_VLAN_INSTALL_FAILED,
 *   which is a number somebody can read, where a silent success is not.
 */
#ifndef _CORTINA_VLAN_INSTALL_H
#define _CORTINA_VLAN_INSTALL_H

#include <linux/types.h>

#include "hwio.h"
#include "regtable.h"	/* struct reg: a field nobody REGISTERED is not offset 0 */
#include "gpon_omci_vlan.h"	/* the core's decided rule and the op table */

/*
 * The DMA-AFT L2FIB word layout.  MOVED from cortina-ni-regs.h, unchanged, so
 * there is ONE spelling: the shell's own banner there records how each field
 * was recovered (ca-ne.ko aal_ni_dma_lso_set_aft_l2fib* and fc_mgr.ko
 * rtk_9607f_asic_dmaAftFib_set, two independent modules) and why the earlier
 * map was wrong.
 *
 * ⚠ vlan_vld IS NOT A VALID BIT: 0 selects VLAN STACKING mode (where the count
 *   field is an opcode instead), 1 selects VLAN SET mode, in which the edit is
 *   declarative -- the frame LEAVES with exactly `tag_cnt` tags.  Reading it as
 *   "valid" made a correctly programmed STRIP look like an empty entry and cost
 *   a day.
 */
#define CORTINA_AFT_D2_VLAN_SET_MODE	(1u << 8)
#define CORTINA_AFT_D2_TAG_CNT_LSB	6		/* [7:6] */
#define CORTINA_AFT_D2_TAG_CNT_MASK	0xc0u
#define CORTINA_AFT_D2_TPID_SLOT_LSB	1		/* [3:1], 1-BASED */
#define CORTINA_AFT_D2_TPID_SLOT_MASK	0x0eu
#define CORTINA_AFT_D1_TOP_VID_LSB	19		/* [30:19] */
#define CORTINA_AFT_D1_TOP_VID_MASK	0x7ff80000u
#define CORTINA_AFT_D1_TPID_SRC_LO	(1u << 31)	/* selector bit0; bit1 is D2[0] */

/* The MAP word: "frames from this lspid use fib @fib". */
#define CORTINA_AFT_MAP_FIB_MASK	0x3fu		/* [5:0]  */
#define CORTINA_AFT_MAP_EN		(1u << 6)
#define CORTINA_AFT_MAP_LSPID_LSB	7		/* [10:7] */
#define CORTINA_AFT_MAP_LSPID_MASK	0x780u
#define CORTINA_AFT_MAP_VLD		(1u << 11)

/* The indirect ACCESS word: idx[5:0] plus the NI-wide GO and WRITE bits. */
#define CORTINA_AFT_ACCESS_IDX_MASK	0x3fu
#define CORTINA_AFT_ACCESS_WRITE	(1u << 30)
#define CORTINA_AFT_ACCESS_GO		(1u << 31)
#define CORTINA_AFT_GO_TRIES		1000u

/*
 * Where the DMA-AFT lives on one die.  A PORT ADDS ONE INITIALISER, and the
 * SHELL writes it: this file may not name an address.
 *
 * ⚠ AN ABSENT BLOCK IS REG_ABSENT, NEVER 0.  Offset 0 is a real register in
 *   this window, so a die without the engine must make the install REFUSE
 *   rather than write into something else -- the regtable.h rule, here too.
 * ⚠⚠ AND A FIELD NOBODY REGISTERED IS A THIRD ANSWER: `struct reg` carries
 *   the offset BIASED, so C's zero-fill decodes to UNSET and the install
 *   refuses with -ENXIO rather than editing whatever sits at offset 0.
 */
struct cortina_vlan_regs {
	const char *name;
	struct reg l2fib_access;
	struct reg l2fib_data2;
	struct reg l2fib_data1;
	struct reg l2fib_data0;
	struct reg map_access;
	struct reg map_data;
	u8	fib_entries;
	u8	map_entries;
};

/*
 * One decided rule, per install.  @tpid_slot is the FAIL-CLOSED gate this
 * engine is full of: the hardware compares the tag's TPID against four slots
 * and, on no match, SILENTLY disables the whole edit for that flow -- a
 * perfectly good edit that does nothing.  So a caller that could not place the
 * TPID passes -1 and the install REFUSES.
 */
struct cortina_vlan_ctx {
	struct hwio	io;		/* over the DMA window */
	const struct cortina_vlan_regs *regs;
	int		tpid_slot;	/* 0..3, or -1 = not placed */
	u8		fib_idx;
	u8		map_idx;
	u8		lspid_map;	/* lspid - CPU0, what the MAP field holds */
	void		(*pause)(void);
};

/* The three L2FIB data words for an edit that leaves @tag_cnt tags on the
 * frame, the top one carrying @vid from TPID slot @tpid_slot.  PURE. */
void cortina_vlan_aft_words(u16 vid, u8 tag_cnt, int tpid_slot,
			    u32 *d0, u32 *d1, u32 *d2);

/* The MAP word for "@lspid_map uses fib @fib", or 0 to clear the entry. PURE. */
u32 cortina_vlan_aft_map_word(u8 lspid_map, u8 fib);

/*
 * What this engine would have to do for @r, or a named refusal.  PURE, and it
 * is the whole capability judgement -- kept out of the core, which has no
 * business having an opinion about a classifier it cannot see.
 *
 * Return: 0 with *@tag_cnt and *@vid filled, or
 *   -EOPNOTSUPP  the shape needs something the DMA-AFT has no field for
 *   -EINVAL      no rule or no out params
 */
int cortina_vlan_rule_to_aft(const struct gpon_vlan_rule *r, u16 *vid,
			     u8 *tag_cnt);

extern const struct gpon_vlan_ops cortina_vlan_ops;

#endif /* _CORTINA_VLAN_INSTALL_H */
