/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * luna_vlan_install.h -- the LUNA family's install for the OMCI WAN service
 * model's VLAN half: `struct gpon_vlan_ops`, filled, plus the per-die table
 * that decides what each Luna part can actually be asked to do.
 *
 * TIER: FAMILY (prefix luna_).  The core (gpon_omci_vlan.h) DECIDES; this
 * programs.  Field layouts and geometry live here, addresses arrive in the
 * per-die table below -- so this object builds on x86 and its write SEQUENCE
 * is asserted with no board (dev/rtl9607c-test/gpon_vlan_install_diff_test).
 *
 * ★★★ WHAT THIS FAMILY CAN AND CANNOT DO, STATED BEFORE THE CODE, because the
 *     difference is the whole finding and a guess here does not fail loudly --
 *     it programs a rule the hardware ignores.
 *
 *   CAN, and every fact under it is ESTABLISHED:
 *     the per-port DEFAULT VID (VLAN_PB_VID).  Address, 12-bit element width,
 *     packing (word p/2, lsb (p%2)*12) and port range were read three ways out
 *     of OUR OWN stock kernel -- the register descriptor, _reg_addr_find and
 *     reg_array_field_write -- and the three chipdefs agree.
 *
 *   CANNOT, and the install REFUSES with a named reason:
 *     the per-flow CLASSIFIER ACTION, which is where stock puts an ME 171 row.
 *     Three things are unestablished at once and any one of them is fatal:
 *       - WHICH access engine reaches CF_ and ACL_.  The chipdef's "access table
 *         type" numbers are a per-ENGINE namespace and the two namespaces
 *         COLLIDE (type 2 is ACL_DATA *and* NEXT_HOP_TABLE, type 3 is
 *         ACL_ACTION_TABLE *and* NETIF), and no register named ACL_ACCESS
 *         exists on any of the three dies.
 *       - HOW ACL_DATA is told apart from ACL_MASK: both are type 2, so the
 *         table type alone cannot select between them.
 *       - the RTL9603CVD HAS NO CF BLOCK AT ALL.  Zero CF_* registers, zero
 *         CF_* tables, nothing in 0x16000..0x1600C.  Its classifier surface is
 *         ACL only, and it is a DIFFERENT shape (SACT/SVID/CACT/CVID) from the
 *         other two dies' CF_ACTION_US.  A family that assumed one classifier
 *         would be wrong on a board that boots today.
 *     and the EGRESS_MODE encoding: the field is at VLAN_EGRESS_TAG[1:0] on all
 *     three dies (chipdef, three-way agreement) and WHAT ITS FOUR VALUES MEAN
 *     is nowhere established, so this file never writes it.
 *
 * ★★ THE PVID MAPPING IS DECLARED, NOT MEASURED.  An ME 171 row that pushes
 *    one literal VID onto UNTAGGED frames from a UNI is expressed on this
 *    switch as that port's default VID.  Stock expresses the same instruction
 *    as a classifier entry; the two agree only for the catch-all row, which is
 *    exactly why every narrower filter is refused here rather than approximated.
 */
#ifndef _LUNA_VLAN_INSTALL_H
#define _LUNA_VLAN_INSTALL_H

#include <linux/types.h>

#include "hwio.h"
#include "regtable.h"	/* struct reg: a field nobody REGISTERED is not offset 0 */
#include "gpon_omci_vlan.h"

/* VLAN_PB_VID element width -- 12 bits, packed two per 32-bit word. */
#define LUNA_VLAN_PB_VID_BITS	12u

/*
 * One Luna die's VLAN surface.  A PORT ADDS ONE INITIALISER.
 *
 * ⚠ REG_ABSENT, NEVER 0, for a block this die does not have: offset 0 is a
 *   real register in this window (GPON_INT_DLT is 0x0000), so a silent write
 *   there is the class of bug that reads back fine.
 * ⚠⚠ AND A FIELD THE INITIALISER FORGETS IS A THIRD ANSWER, NOT AN ABSENCE:
 *   every offset is `struct reg`, spelled REG_AT(...), so C's zero-fill
 *   decodes to UNSET and the install refuses with -ENXIO instead of writing
 *   to offset 0.  reg_table_registered_guard.py requires every field of the
 *   initialisers below -- the port count and the four bit positions included,
 *   where a forgotten 0 is a shift, not an address.
 */
struct luna_vlan_regs {
	const char *name;
	struct reg pb_vid;	/* VLAN_PB_VID array base */
	struct reg vlan_ingress;/* VLAN_INGRESS, one bit per port */
	struct reg vlan_ctrl;	/* VLAN_CTRL, bit0 = VLAN_FILTERING */
	/* VLAN_EGRESS_TAG, one 32-bit word per port, EGRESS_MODE at [1:0].
	 * The ADDRESS is established on all three dies; the four MODE VALUES
	 * are not, so nothing here writes it.  Carried so the refusal has a
	 * subject and so a later RE lands in one place. */
	struct reg egress_tag;
	u8	n_ports;	/* port index range is 0..n_ports-1 */

	/*
	 * ★★★ THE INDIRECT TABLE-ACCESS ENGINE, AND ITS BIT LAYOUT IS PER DIE.
	 * Same four addresses on all three parts, DIFFERENT field positions:
	 *
	 *   die          ADDR lsb   ACCESS_METHOD lsb   CMD_TYPE lsb/width
	 *   RTL9602C         9              4                 3 / 1
	 *   RTL9603CVD      12              5                 3 / 2
	 *   RTL9607C        12              5                 3 / 2
	 *
	 * ⚠ luna_gpon.c's tbl_write() spells the RTL9602C layout as literals in
	 *   a file that BUILDS ON ALL THREE DIES.  It is correct today only
	 *   because its sole caller is reached from the 9602C shell; the day a
	 *   sibling calls it, it writes ADDR three bits low and ACCESS_METHOD
	 *   into CMD_TYPE's second bit.  luna_vlan_tbl_ctrl_word() is that
	 *   arithmetic as DATA, and the offline differential proves it is
	 *   byte-identical on the RTL9602C -- so adopting it there changes
	 *   nothing -- and different on the other two, which is the repair.
	 */
	struct reg tbl_ctrl;
	struct reg tbl_sts;
	struct reg tbl_wrdata;
	struct reg tbl_rddata;
	u8	tbl_addr_lsb;
	u8	tbl_method_lsb;
	u8	tbl_cmd_lsb;
	u8	tbl_cmd_bits;

	/*
	 * The per-flow classifier ACTION table -- where an ME 171 row belongs
	 * and where this family cannot yet put it.  REG_ABSENT on every die
	 * TODAY, deliberately, for the three unestablished reasons in the
	 * banner; `has_cf_block` additionally records that one die has no CF
	 * engine at all, which is a SILICON fact and not our blind spot.
	 */
	struct reg cf_action_us;
	bool	has_cf_block;
};

extern const struct luna_vlan_regs luna_vlan_rtl9602c;
extern const struct luna_vlan_regs luna_vlan_rtl9603cvd;
extern const struct luna_vlan_regs luna_vlan_rtl9607c;

struct luna_vlan_ctx {
	struct hwio	io;		/* over the switch core */
	const struct luna_vlan_regs *regs;
	u8		uni_port;	/* the switch port the association names */
	void		(*pause)(void);
};

/* Where element @port of a @bits-wide packed array lives. PURE. */
u32 luna_vlan_packed_off(u32 base, unsigned int port, unsigned int bits);
unsigned int luna_vlan_packed_lsb(unsigned int port, unsigned int bits);

/* The TABLE-ACCESS control word for this die. PURE, and the whole point is
 * that the SHIFTS come from @r and not from a literal. */
u32 luna_vlan_tbl_ctrl_word(const struct luna_vlan_regs *r, u32 type, u32 addr);

/*
 * -> 0 and *@vid filled when @r is a rule this family CAN express as a port
 * default VID; a named refusal otherwise.  PURE.
 *   -EOPNOTSUPP  the shape needs the classifier action table
 *   -ENOTSUPP is deliberately not used: one spelling, checked by the test
 */
int luna_vlan_rule_to_pvid(const struct gpon_vlan_rule *r, u16 *vid);

extern const struct gpon_vlan_ops luna_vlan_ops;

#endif /* _LUNA_VLAN_INSTALL_H */
