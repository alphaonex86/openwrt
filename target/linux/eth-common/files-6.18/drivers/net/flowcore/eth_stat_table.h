/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_stat_table.h -- a driver's own u64 counters as `ethtool -S` rows: a table of their names and offsets in
 * the structure that holds them. */
#ifndef _ETH_STAT_TABLE_H
#define _ETH_STAT_TABLE_H

#include <linux/ethtool.h>
#include <linux/stddef.h>

struct eth_stat {
	char name[ETH_GSTRING_LEN];
	size_t off;
};

#define ETH_STAT(type, field)	{ #field, offsetof(type, field) }

static inline int eth_stat_count(int sset, unsigned int n)
{
	return sset == ETH_SS_STATS ? n : -EOPNOTSUPP;
}

static inline void eth_stat_strings(u32 sset, u8 *data, const struct eth_stat *s, unsigned int n)
{
	if (sset != ETH_SS_STATS)
		return;
	for (; n; n--, s++)
		ethtool_puts(&data, s->name);
}

static inline void eth_stat_values(u64 *data, const void *base, const struct eth_stat *s, unsigned int n)
{
	for (; n; n--, s++)
		*data++ = *(const u64 *)((const u8 *)base + s->off);
}

#endif /* _ETH_STAT_TABLE_H */
