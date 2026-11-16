/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: VENDOR */
#ifndef BCM_CMIC_H
#define BCM_CMIC_H

#include <linux/device.h>
#include <linux/types.h>

/* One step of a switch init table (virt/broadcom_switch_fold.py writes a chip's). */
enum bcm_cmic_op {
	BCM_CMIC_WR,	/* CMIC register `addr` = `val` */
	BCM_CMIC_REG,	/* switch register (`blk`, `addr`) = `val`, or `words` words from the table's words at `val` */
	BCM_CMIC_MEM,	/* `count` entries of `words` words from the table's words at `val`, the first at (`blk`, `addr`) */
	BCM_CMIC_POLL,	/* until switch register (`blk`, `addr`) has every bit of `val` */
	BCM_CMIC_MIIM,	/* PHY `addr` >> 16 (bus 7:5, id 4:0, 0x200 internal) register `addr` & 0xffff = `val` */
};

struct bcm_cmic_step {
	u32 addr;
	u32 val;
	u16 count;
	u8 op;
	u8 blk;
	u8 words;
};

struct bcm_cmic_table {
	const struct bcm_cmic_step *steps;
	unsigned int n_steps;
	const u32 *words;
	unsigned int n_words;
};

#define CMIC_WR(a, v)			{ (a), (v), 1, BCM_CMIC_WR, 0, 1 }
#define CMIC_REG(b, a, v)		{ (a), (v), 1, BCM_CMIC_REG, (b), 1 }
#define CMIC_REGN(b, a, n, at)		{ (a), (at), 1, BCM_CMIC_REG, (b), (n) }
#define CMIC_MEM(b, a, n, cnt, at)	{ (a), (at), (cnt), BCM_CMIC_MEM, (b), (n) }
#define CMIC_POLL(b, a, mask)		{ (a), (mask), 1, BCM_CMIC_POLL, (b), 1 }
#define CMIC_MIIM(phy, reg, v)		{ (phy) << 16 | (reg), (v), 1, BCM_CMIC_MIIM, 0, 1 }

/* Replays @t through the CMIC mapped at @cmic (@size bytes); stops at the first step the hardware refuses. */
int bcm_cmic_run(struct device *dev, void __iomem *cmic, resource_size_t size, const struct bcm_cmic_table *t);

#endif /* BCM_CMIC_H */
