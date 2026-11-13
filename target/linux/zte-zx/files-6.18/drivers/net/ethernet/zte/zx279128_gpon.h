/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CHIP */
/* zx279128_gpon.h -- the ZX279128 GPON MAC's shell, and what it needs from the packet processor's owner. */
#ifndef _ZX279128_GPON_H
#define _ZX279128_GPON_H

#include <linux/types.h>

struct device;
struct zx_eth;
struct zx_pon;

/* base maps 0x92000000; irq is the PON's line. -> the shell, or an ERR_PTR. */
struct zx_pon *zx_pon_probe(struct device *dev, void __iomem *base, int irq, struct zx_eth *eth);
void zx_pon_remove(struct zx_pon *pon);
/* What the packet processor's PON ring delivered on GEM port `gem`: an OMCI message, or a frame (softirq). */
void zx_pon_rx(struct zx_pon *pon, bool omci, u16 gem, const u8 *data, unsigned int len);
/* zx279128_eth.c: len bytes upstream on GEM port `gem` of T-CONT `tcont`; an OMCI message takes the OMCI queue.
 * 0 = queued. */
int zx_eth_pon_tx(struct zx_eth *eth, const u8 *data, unsigned int len, u16 gem, unsigned int tcont, bool omci);

#endif
