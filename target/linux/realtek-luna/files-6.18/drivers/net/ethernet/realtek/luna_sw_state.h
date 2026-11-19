/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: FAMILY */
/* luna_sw_state.h -- this family's switch tables row by row for eth_row_rd(): the table port's rows (VLAN, ACL),
 * the L2 table by index, the L3/L4 engine's tables.
 */
#ifndef _LUNA_SW_STATE_H
#define _LUNA_SW_STATE_H

#include "eth_regs_dump.h"
#include "luna_l2_access.h"
#include "luna_l34_acc.h"

struct luna_state_rd {
	struct eth_row_reader r;	/* first: eth_row_rd() is handed this struct */
	void __iomem *base;
	u32 cmd;			/* table port: the row read command of row 0 */
	u8 addr_shift, type;		/* type: the L3/L4 engine's table */
	const struct luna_l2_layout *l2;
	const struct luna_l34_acc *l34;
};

static inline u32 luna_state_reg_rd(void *ctx, u32 at)
{
	return readl((void __iomem *)ctx + at);
}

static inline int luna_state_tbl_row(struct eth_row_reader *r, u32 row)
{
	struct luna_state_rd *t = container_of(r, struct luna_state_rd, r);

	return luna_tbl_read(t->base, t->cmd | row << t->addr_shift, r->w, r->words);
}

static inline int luna_state_l2_row(struct eth_row_reader *r, u32 row)
{
	struct luna_state_rd *t = container_of(r, struct luna_state_rd, r);
	int ret = luna_l2_access(t->base, t->l2, row, r->w, false, false);

	return ret < 0 ? ret : 0;
}

static inline int luna_state_l34_row(struct eth_row_reader *r, u32 row)
{
	struct luna_state_rd *t = container_of(r, struct luna_state_rd, r);

	return luna_l34_tbl_op(t->base, t->l34, t->type, row, r->w, r->words, false);
}

/* One table as a block: `t` is filled for `rd_row`, its ranges in `rg`. */
static inline void luna_state_tbl(struct eth_state_block *b, unsigned int *n, const char *name, bool learned,
				  struct luna_state_rd *t, struct eth_reg_range rg[3],
				  int (*rd_row)(struct eth_row_reader *, u32), u8 words, u32 rows, u32 first2)
{
	t->r.rd_row = rd_row;
	t->r.words = words;
	eth_row_reader_start(&t->r);
	eth_reg_rows(rg, rows, words, first2);
	eth_state_add(b, n, name, learned, eth_row_rd, &t->r, rg, 3, &t->r.unread);
}

#endif
