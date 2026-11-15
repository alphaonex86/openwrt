/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_regs_dump.h -- a chip's register dump as a table of ranges: offset and value pairs for ethtool -d, and
 * the nonzero words in the kernel log for a fault about to be recovered. The chip gives the ranges (only words
 * safe to read on a running unit) and a reader. */
#ifndef ETH_REGS_DUMP_H
#define ETH_REGS_DUMP_H

#include <linux/kernel.h>
#include <linux/netdevice.h>

struct eth_reg_range { u32 at; u16 words; };	/* 0 words ends a list */
typedef u32 (*eth_reg_read_t)(void *ctx, u32 at);

static inline unsigned int eth_regs_words(const struct eth_reg_range *r, unsigned int max)
{
	unsigned int i, words = 0;

	for (i = 0; i < max && r[i].words; i++)
		words += r[i].words;
	return words;
}

/* out: eth_regs_words() pairs of (offset, value) */
static inline void eth_regs_pairs(const struct eth_reg_range *r, unsigned int max, eth_reg_read_t rd, void *ctx,
				  u32 *out)
{
	unsigned int i, w;

	for (i = 0; i < max && r[i].words; i++)
		for (w = 0; w < r[i].words; w++) {
			*out++ = r[i].at + w * 4;
			*out++ = rd(ctx, r[i].at + w * 4);
		}
}

/* "<what> offset:value ..." six a line, nonzero words only: a reader diffs it against a healthy unit's. */
static inline void eth_regs_log(struct net_device *ndev, const char *what, const struct eth_reg_range *r,
				unsigned int max, eth_reg_read_t rd, void *ctx)
{
	char line[6 * 15 + 1];
	unsigned int i, w, n = 0;
	u32 val;

	for (i = 0; i < max && r[i].words; i++)
		for (w = 0; w < r[i].words; w++) {
			val = rd(ctx, r[i].at + w * 4);
			if (val)
				n += scnprintf(line + n, sizeof(line) - n, " %05x:%08x", r[i].at + w * 4, val);
			if (n + 15 < sizeof(line))
				continue;
			netdev_err(ndev, "%s%s\n", what, line);
			n = 0;
		}
	if (n)
		netdev_err(ndev, "%s%s\n", what, line);
}

#endif
