/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_init_table.h -- a chip's init as data, the way its stock firmware leaves it: register values, and rows of
 * an indirect table engine (wait for its ready flag, select the row, write the data words high to low: the
 * lowest commits the row). The chip gives the tables and its engine's offsets.
 */
#ifndef _ETH_INIT_TABLE_H
#define _ETH_INIT_TABLE_H

#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/types.h>

struct eth_reg_val {
	u32 off, val;
};

static inline void eth_reg_vals_write(void __iomem *base, const struct eth_reg_val *v, unsigned int n)
{
	for (; n; n--, v++)
		writel(v->val, base + v->off);
}

struct eth_tbl {
	struct device *dev;
	void __iomem *base;
	u32 select, ready, data;	/* offsets from an engine */
	u32 ready_bit;
};

static inline int eth_tbl_write(const struct eth_tbl *t, u32 eng, u32 sel, const u32 *w, unsigned int n)
{
	void __iomem *at = t->base + eng;
	u32 v;

	if (readl_poll_timeout(at + t->ready, v, v & t->ready_bit, 10, 100000)) {
		dev_err(t->dev, "table engine 0x%x row 0x%x never ready (0x%08x)\n", eng, sel, v);
		return -ETIMEDOUT;
	}
	writel(sel, at + t->select);
	while (n--)
		writel(w[n], at + t->data + 4 * n);
	return 0;
}

/* rows laid out flat, { select, words[n] } each; stops at the first row not written */
static inline int eth_tbl_write_rows(const struct eth_tbl *t, u32 eng, const u32 *rows, unsigned int count,
				     unsigned int n)
{
	int ret = 0;

	for (; count && !ret; count--, rows += 1 + n)
		ret = eth_tbl_write(t, eng, rows[0], rows + 1, n);
	return ret;
}

#endif /* _ETH_INIT_TABLE_H */
