/* SPDX-License-Identifier: GPL-2.0 */
/* luna_ponmac.h - board-agnostic bring-up of the Realtek ...
 * dev/MEASURED-luna_ponmac.h.md sec 1. */
#ifndef _LUNA_PONMAC_H
#define _LUNA_PONMAC_H

#include <linux/errno.h>
#include <linux/types.h>

/* The chips this library actually serves. It is the LUNA MIPS ...
 * dev/MEASURED-luna_ponmac.h.md sec 2. */
enum luna_chip {
	LUNA_CHIP_9602C = 0,	/* + 9601C / 9601C_VB subtypes */
	LUNA_CHIP_9603CVD,
	LUNA_CHIP_9607C,
};

/* Chip revision id as read from HW: A=0x1, B=0x2, C=0x3, ... rev>A => B+ */
#define LUNA_REV_A		0x1
#define LUNA_REV_B		0x2
#define LUNA_REV_C		0x3

/* MODEL_NAME_INFO (swcore 0x10000): RTL_ID [31:16], MODEL_CHAR_1ST [15:11]
 * ('C' = 3), a variant bit [4], RTL_VID [3:0]. The stock probe reads the
 * RTL9607C revision as RTL_VID + 2 (+ 1 with bit 4 set), capped at C; the
 * UNKNOWN board's 0x96071800 is rev B. -ENODEV when the word names another die. */
#define LUNA_MODEL_NAME_INFO	0x1B010000u

static inline int luna_rtl9607c_rev(u32 model_name_info)
{
	u32 rev;

	if ((model_name_info >> 16) != 0x9607 || ((model_name_info >> 11) & 0x1f) != 3)
		return -ENODEV;
	rev = (model_name_info & 0xf) + ((model_name_info & 0x10) ? 1 : 2);
	return rev > LUNA_REV_C ? LUNA_REV_C : (int)rev;
}

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
	o->wr(phys, gpon_field_set(o->rd(phys), msb, lsb, val));
}

/* Bring-up entry points. rev = LUNA_REV_A or the HW ... -- dev/MEASURED-luna_ponmac.h.md sec 3. */
int luna_ponmac_mode_set(enum luna_chip chip, int rev, int subtype,
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
