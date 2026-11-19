/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_regs_dump.h -- a chip's register dump as a table of ranges: offset and value pairs for ethtool -d, and
 * the nonzero words in the kernel log for a fault about to be recovered. The chip gives the ranges (only words
 * safe to read on a running unit) and a reader. */
#ifndef ETH_REGS_DUMP_H
#define ETH_REGS_DUMP_H

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/errno.h>
#include <linux/string.h>

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

/* A block of state a reset may clear: registers or table rows, read word by word through rd. A table is a range
 * of row*words: rd decodes the row. `volatile` blocks (counters, learned rows) are reported, never judged. */
struct eth_state_block {
	const char *name;
	const struct eth_reg_range *r;
	unsigned int max;
	eth_reg_read_t rd;
	void *ctx;
	bool volatile_rows;
	const u32 *unread;	/* reads rd could not make (they count as 0), or NULL */
};

static inline void eth_state_add(struct eth_state_block *b, unsigned int *n, const char *name, bool learned,
				 eth_reg_read_t rd, void *ctx, const struct eth_reg_range *r, unsigned int max,
				 const u32 *unread)
{
	b[(*n)++] = (struct eth_state_block){ .name = name, .r = r, .max = max, .rd = rd, .ctx = ctx,
					      .volatile_rows = learned, .unread = unread };
}

/* A table as ranges: `rows` rows of `words` from row 0, and as many from row `first2` when it is not 0. */
static inline void eth_reg_rows(struct eth_reg_range r[3], u32 rows, u32 words, u32 first2)
{
	r[0] = (struct eth_reg_range){ 0, rows * words };
	r[1] = (struct eth_reg_range){ first2 * words * 4, first2 ? rows * words : 0 };
	r[2] = (struct eth_reg_range){ 0, 0 };
}

/* A table read row by row: a row is read once for all its words; a row that cannot be read reads 0 and counts. */
#define ETH_ROW_WORDS_MAX	8
struct eth_row_reader {
	int (*rd_row)(struct eth_row_reader *t, u32 row);	/* 0, -ENOENT (an empty row), or unread */
	u8 words;
	int row;		/* the row in w, -1 = none */
	u32 w[ETH_ROW_WORDS_MAX];
	u32 unread;
};

static inline void eth_row_reader_start(struct eth_row_reader *t)
{
	t->row = -1;
	t->unread = 0;
}

static inline u32 eth_row_rd(void *ctx, u32 at)
{
	struct eth_row_reader *t = ctx;
	u32 row = at / 4 / t->words;

	if (t->row != (int)row) {
		int ret = t->words <= ETH_ROW_WORDS_MAX ? t->rd_row(t, row) : -EINVAL;

		if (ret) {
			memset(t->w, 0, sizeof(t->w));
			if (ret != -ENOENT)
				t->unread++;
		}
		t->row = row;
	}
	return t->w[at / 4 % t->words];
}

static inline unsigned int eth_state_words(const struct eth_state_block *b, unsigned int n)
{
	unsigned int i, words = 0;

	for (i = 0; i < n; i++)
		words += eth_regs_words(b[i].r, b[i].max);
	return words;
}

/* out: eth_state_words() values, block after block */
static inline void eth_state_snap(const struct eth_state_block *b, unsigned int n, u32 *out)
{
	unsigned int i, k, w;

	for (i = 0; i < n; i++)
		for (k = 0; k < b[i].max && b[i].r[k].words; k++)
			for (w = 0; w < b[i].r[k].words; w++)
				*out++ = b[i].rd(b[i].ctx, b[i].r[k].at + w * 4);
}

/* How the words of `after` stand against `before`. */
struct eth_state_delta {
	u32 kept, cleared, changed, appeared;
};

static inline void eth_state_delta_count(const u32 *before, const u32 *after, unsigned int words,
					 struct eth_state_delta *d)
{
	unsigned int w;

	for (w = 0; w < words; w++)
		if (before[w] == after[w])
			d->kept++;
		else if (!after[w])
			d->cleared++;
		else if (!before[w])
			d->appeared++;
		else
			d->changed++;
}

#define ETH_STATE_LOG_RUNS	24	/* per block: a run is consecutive words that moved the same way */

/* One line per block (kept/cleared/changed/appeared), then its runs: "at:before>after" for one word,
 * "at-last cleared|appeared|changed(n)" for more. -> the words that moved in blocks that are judged. */
static inline u32 eth_state_log_delta(struct net_device *ndev, const char *what, const struct eth_state_block *b,
				      unsigned int n, const u32 *before, const u32 *after)
{
	static const char *const kind[] = { "kept", "cleared", "appeared", "changed" };
	unsigned int i, k, w, runs, at = 0;
	u32 judged = 0;

	for (i = 0; i < n; i++) {
		struct eth_state_delta d = { 0 };
		unsigned int words = eth_regs_words(b[i].r, b[i].max);

		eth_state_delta_count(before + at, after + at, words, &d);
		netdev_err(ndev, "%s %s: kept %u cleared %u changed %u appeared %u, unread %u%s\n", what, b[i].name,
			   d.kept, d.cleared, d.changed, d.appeared, b[i].unread ? *b[i].unread : 0,
			   b[i].volatile_rows ? " (volatile)" : "");
		if (!b[i].volatile_rows)
			judged += d.cleared + d.changed + d.appeared;
		runs = 0;
		for (k = 0; k < b[i].max && b[i].r[k].words; k++)
			for (w = 0; w < b[i].r[k].words; at++) {
				u32 off = b[i].r[k].at + w * 4, x = before[at], y = after[at];
				unsigned int how = x == y ? 0 : !y ? 1 : !x ? 2 : 3, len = 1;

				while (how && w + len < b[i].r[k].words &&
				       (before[at + len] == after[at + len] ? 0 : !after[at + len] ? 1 :
					!before[at + len] ? 2 : 3) == how)
					len++;
				if (how && runs++ < ETH_STATE_LOG_RUNS) {
					if (len == 1)
						netdev_err(ndev, "%s %s %05x:%08x>%08x\n", what, b[i].name, off, x, y);
					else
						netdev_err(ndev, "%s %s %05x-%05x %s(%u)\n", what, b[i].name, off,
							   off + (len - 1) * 4, kind[how], len);
				}
				w += len;
				at += len - 1;
			}
		if (runs > ETH_STATE_LOG_RUNS)
			netdev_err(ndev, "%s %s: %u more run(s) not printed\n", what, b[i].name,
				   runs - ETH_STATE_LOG_RUNS);
	}
	return judged;
}

#endif
