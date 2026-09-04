/* SPDX-License-Identifier: GPL-2.0 */
/*
 * luna_ponmac.h - board-agnostic bring-up of the Realtek RTL960x family
 * GPON PON-MAC / SerDes, matching the stock device's register behavior.
 *
 * Purpose: have a stock-faithful ponmac/SerDes bring-up for the WHOLE
 * RTL960x family in-tree, so any future 960x board can be brought online by
 * wiring its register accessor + chip id, without re-deriving the sequence
 * from scratch.
 *
 * Reconstructed per-chip from the stock register behavior, with every register
 * resolved to its ABSOLUTE physical address from that chip's register map and
 * field bit-ranges from its field map. The 9602C path is HW-tested (the
 * realtek-luna board); the other family members are behavior-faithful
 * transcriptions, untested for lack of hardware, ready for when a board is
 * connected.
 *
 * The bring-up uses ABSOLUTE physical register addresses; the board driver
 * supplies rd/wr that map phys->virt (KSEG1 0xa0000000|phys, or ioremap).
 * On the 9602C "Luna" SoC: swcore phys 0x1b000000, PON-IP 0x1bf00000,
 * GTC 0x1b700000 (all in the KSEG1-reachable 0x1bxxxxxx window).
 */
#ifndef _LUNA_PONMAC_H
#define _LUNA_PONMAC_H

#include <linux/types.h>

/*
 * The chips this library actually serves.  It is the LUNA MIPS silicon and
 * nothing else: the RTL9607F carries a Realtek part number from the same
 * series but is a Cortina Access NE core with a different register map, and
 * it is driven by target/linux/realtek-elnath.  It used to sit in this enum
 * as a placeholder returning -ENOTSUPP, which made the file look like it
 * covered the whole RTL960x number space while serving one half of it.
 */
enum luna_chip {
	LUNA_CHIP_9602C = 0,	/* + 9601C / 9601C_VB subtypes */
	LUNA_CHIP_9603CVD,
	LUNA_CHIP_9607C,
};

/* Chip revision id as read from HW: A=0x1, B=0x2, C=0x3, ... rev>A => B+ */
#define LUNA_REV_A		0x1
#define LUNA_REV_B		0x2
#define LUNA_REV_C		0x3

/* Subtype (9602C family): pick GponModeV3 (9601C) vs V2 on rev>A. */
#define LUNA_SUBTYPE_NONE		0x00
#define LUNA_SUBTYPE_9601C_VB	0x01
#define LUNA_SUBTYPE_9601C		0x03

/*
 * Board-agnostic register accessor. phys is the ABSOLUTE physical address from
 * the chip's register map. The board driver maps it (KSEG1 or ioremap).
 */
struct luna_ops {
	u32  (*rd)(u32 phys);
	void (*wr)(u32 phys, u32 val);
};

/* The field-mask owner lives in gpon/, which this family already has on its
 * include path (realtek-luna/Makefile: -I$(srctree)/drivers/net/gpon). */
#include "gpon_regseq.h"

/* read-modify-write bits [msb:lsb] at absolute phys address */
static inline void luna_rfwr(const struct luna_ops *o, u32 phys,
				u8 msb, u8 lsb, u32 val)
{
	u32 mask = gpon_field_mask(msb, lsb);   /* the ONE owner, in gpon/ */

	o->wr(phys, (o->rd(phys) & ~mask) | ((val << lsb) & mask));
}

/*
 * Bring-up entry points. rev = LUNA_REV_A or the HW chip-revision id; subtype
 * as above. Return 0 on success, -ETIMEDOUT if the SerDes analog-ready gate never
 * asserts (non-fatal; the board driver may proceed + diagnose).
 */
int luna_ponmac_init(enum luna_chip chip, int rev, int subtype,
			const struct luna_ops *o);
int luna_ponmac_mode_set(enum luna_chip chip, int rev, int subtype,
			    const struct luna_ops *o);
int luna_ponmac_serdes_cdr_reset(enum luna_chip chip,
				    const struct luna_ops *o);

struct seq_file;
void luna_c7_diag(const struct luna_ops *o, struct seq_file *s);

extern int luna_c2_postmode_perturb;
extern int luna_c2_sds_cfgrst;
extern int luna_c2_stock_analog;
extern int luna_c2_analog_postreset;
extern int luna_c2_cmu_settle_ms;
extern int luna_c2_clkgate_rstb;
extern int luna_c2_skip_rstb_dance;
extern int luna_c2_minimal_analog;
#endif /* _LUNA_PONMAC_H */
