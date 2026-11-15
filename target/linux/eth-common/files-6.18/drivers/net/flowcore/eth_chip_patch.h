/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_chip_patch.h -- what a switch chip is given after reset, as its stock firmware does: a table of PHY
 * words, then the patch the chip carries in its efuse (PHY records, then register records). Pure logic: the
 * chip gives the table, the efuse geometry and the accessors. Host test: eth_chip_patch_test. */
#ifndef ETH_CHIP_PATCH_H
#define ETH_CHIP_PATCH_H

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>

struct eth_chip_patch_io {
	int (*efuse)(void *ctx, unsigned int entry, u16 *out);
	int (*phy_rd)(void *ctx, unsigned int phy, u32 reg, u16 *out);
	int (*phy_wr)(void *ctx, unsigned int phy, u32 reg, u16 val);
	int (*reg_wr)(void *ctx, u32 addr, u32 val);	/* -ERANGE: not a register of this chip */
};

/* select: the word only addresses the one after it (written, never compared) */
struct eth_phy_word { u8 phy; bool select; u16 reg, val; };

struct eth_phy_words_stat { u16 words, differed, first; };

/* Writes every word, as stock does, and counts those the PHY did not already hold. -> 0, or the error that
 * stopped it: nothing after it is written. */
static inline int eth_phy_words_apply(const struct eth_chip_patch_io *io, void *ctx, const struct eth_phy_word *w,
				      unsigned int n, struct eth_phy_words_stat *st)
{
	unsigned int i;
	u16 was = 0;
	int rc;

	memset(st, 0, sizeof(*st));
	for (i = 0; i < n; i++) {
		rc = w[i].select ? 0 : io->phy_rd(ctx, w[i].phy, w[i].reg, &was);
		if (!rc && !w[i].select) {
			st->words++;
			if (was != w[i].val && !st->differed++)
				st->first = was;
		}
		rc = rc ?: io->phy_wr(ctx, w[i].phy, w[i].reg, w[i].val);
		if (rc)
			return rc;
	}
	return 0;
}

struct eth_efuse_geom {
	u8 entries;	/* 16-bit entries the efuse answers for */
	u8 count;	/* the entry holding the number of PHY records */
	u8 first;	/* the first record */
	u8 phys;	/* PHYs a record's mask can name */
};

struct eth_efuse_patch_stat {
	bool present;
	u16 phy_words, reg_words, refused, failed;
};

/* -> 0 with w[] filled, 1 past the efuse's end, negative on a read error */
static inline int eth_efuse_words(const struct eth_chip_patch_io *io, void *ctx, const struct eth_efuse_geom *g,
				  unsigned int at, u16 *w, unsigned int n)
{
	unsigned int i;
	int rc;

	if (at + n > g->entries)
		return 1;
	for (i = 0; i < n; i++) {
		rc = io->efuse(ctx, at + i, &w[i]);
		if (rc)
			return rc;
	}
	return 0;
}

static inline bool eth_efuse_all(const u16 *w, unsigned int n, u16 v)
{
	while (n--)
		if (w[n] != v)
			return false;
	return true;
}

/* -> 0 (st says what was written), or the read error that stopped it. A record of zeroes ends the patch, a
 * record of ones is skipped; a word that cannot be written is counted, never fatal: the unit then runs as it
 * did without it. */
static inline int eth_efuse_patch(const struct eth_chip_patch_io *io, void *ctx, const struct eth_efuse_geom *g,
				  struct eth_efuse_patch_stat *st)
{
	unsigned int at = g->first, i, j, n;
	u16 w[4];
	int rc;

	memset(st, 0, sizeof(*st));
	rc = eth_efuse_words(io, ctx, g, 0, w, 3);
	if (rc)
		return rc < 0 ? rc : -EINVAL;
	if (!((w[0] ^ w[1] ^ w[2]) & 1))
		return 0;
	st->present = true;
	rc = eth_efuse_words(io, ctx, g, g->count, w, 1);
	if (rc)
		return rc < 0 ? rc : -EINVAL;
	for (n = w[0] & 0xff, i = 0; i < n; i++, at += 3) {
		rc = eth_efuse_words(io, ctx, g, at, w, 3);
		if (rc || eth_efuse_all(w, 3, 0))
			return rc < 0 ? rc : 0;
		if (eth_efuse_all(w, 3, 0xffff))
			continue;
		for (j = 0; j < g->phys; j++) {
			if (!(w[0] & (1u << j)))
				continue;
			if (io->phy_wr(ctx, j, w[1], w[2]))
				st->failed++;
			else
				st->phy_words++;
		}
	}
	for (;; at += 4) {
		rc = eth_efuse_words(io, ctx, g, at, w, 4);
		if (rc || eth_efuse_all(w, 4, 0))
			return rc < 0 ? rc : 0;
		if (eth_efuse_all(w, 4, 0xffff))
			continue;
		rc = io->reg_wr(ctx, (u32)w[0] << 16 | w[1], (u32)w[2] << 16 | w[3]);
		if (rc == -ERANGE)
			st->refused++;
		else if (rc)
			st->failed++;
		else
			st->reg_words++;
	}
}

/* The efuse as bytes (ethtool -e), each 16-bit entry high byte first. */
static inline int eth_efuse_bytes(const struct eth_chip_patch_io *io, void *ctx, u32 offset, u32 len, u8 *data)
{
	u16 word = 0;
	int rc = 0;
	u32 i;

	for (i = offset; !rc && i < offset + len; i++) {
		if (i == offset || !(i & 1))
			rc = io->efuse(ctx, i / 2, &word);
		data[i - offset] = i & 1 ? word & 0xff : word >> 8;
	}
	return rc;
}

#endif
