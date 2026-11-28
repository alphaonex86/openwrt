/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: VENDOR */
#ifndef BCM_CMIC_H
#define BCM_CMIC_H

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/types.h>

/* A switch init through the CMIC: the first failure is logged, then every later operation is skipped. */
struct bcm_cmic {
	struct device *dev;
	void __iomem *base;
	resource_size_t size;
	unsigned int ops;
	int err;
};

/* One write of a PHY register by the MIIM master. */
struct bcm_cmic_mii_wr {
	u8 reg;
	u16 val;
};

void bcm_cmic_write(struct bcm_cmic *c, u32 off, u32 val);
void bcm_cmic_reg(struct bcm_cmic *c, u8 blk, u32 addr, u32 val);
void bcm_cmic_reg64(struct bcm_cmic *c, u8 blk, u32 addr, u64 val);
void bcm_cmic_mem(struct bcm_cmic *c, u8 blk, u32 addr, const u32 *entry, unsigned int words);
void bcm_cmic_mem32(struct bcm_cmic *c, u8 blk, u32 addr, u32 val);
void bcm_cmic_poll(struct bcm_cmic *c, u8 blk, u32 addr, u32 mask);
/* @phy: bus 7:5, id 4:0, 0x200 for an internal SerDes */
void bcm_cmic_miim(struct bcm_cmic *c, u16 phy, u8 reg, u16 val);
void bcm_cmic_miim_seq(struct bcm_cmic *c, u16 phy, const struct bcm_cmic_mii_wr *seq, unsigned int n);
/* -> 0, or the first failure's error */
int bcm_cmic_done(struct bcm_cmic *c);

#define BCM_CMIC_MEM(c, blk, addr, ...) \
	bcm_cmic_mem(c, blk, addr, (const u32[]){ __VA_ARGS__ }, ARRAY_SIZE(((const u32[]){ __VA_ARGS__ })))
#define BCM_CMIC_MIIM_SEQ(c, phy, seq)	bcm_cmic_miim_seq(c, phy, seq, ARRAY_SIZE(seq))

#endif /* BCM_CMIC_H */
