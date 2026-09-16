/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * luna_l34_acc.h - the Luna L3/L4 engines' INDIRECT TABLE ACCESS, at family tier
 *
 * ★★★ WHY THIS IS FAMILY AND NOT CHIP.  This family carries TWO L3/L4 engines --
 * the NAPT model (RTL9602C, RTL9602BVB) and the FLOWBASED model (RTL9603CVD,
 * RTL9603D, RTL9607C) -- and this tree's own measurement of them says they
 * "share the indirect table-access mechanism and NOTHING else".  The mechanism
 * was written once, as `static` functions inside the RTL9602C's own driver, so
 * the ONE thing the two models have in common lived in the file belonging to
 * exactly one of them.  A second die could only get it by copying it.
 *
 * The primitive itself is four registers and a poll: put the words in the
 * write-data bank, write the command (EXE | table type | entry index), wait for
 * the engine to clear EXE, read the read-data bank.  Only the four ADDRESSES and
 * the command word's own field positions differ between dies -- so a new die
 * owes a TABLE here, never a driver, which is this port's whole strategy.
 *
 * ⚠⚠ AND WHAT IS NOT ESTABLISHED REFUSES RATHER THAN GUESSING.  For the
 * RTL9603CVD the four addresses ARE established (its own chipdef, corroborated by
 * that board's stock capture: 43 non-zero words at the FLOWBASED page while the
 * vendor engine forwards).  Its COMMAND WORD LAYOUT is not established on any
 * tier -- nobody has measured which bits are EXE, where the table type sits, or
 * how wide the index is.  Filling those in from the RTL9602C because the
 * addresses look alike is exactly the defect `reg_table_registered_guard` exists
 * to stop: a value the author supplied where a human had to register one.  So a
 * die with no measured layout carries a NULL `cmd_layout`, every call REFUSES
 * with -ENXIO, and nothing is written to silicon on a guess.
 */
#ifndef _LUNA_L34_ACC_H
#define _LUNA_L34_ACC_H

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/types.h>

/** The engine clears EXE well inside this; a longer wait is a dead engine. */
#define LUNA_L34_EXE_POLL_US	2000

/** Largest data-bank transfer any table on this family needs. */
#define LUNA_L34_WORDS_MAX	8

/**
 * struct luna_l34_cmd_layout - where the command word's fields sit, per die.
 * @rd_exe:	start-read bit (self-clearing)
 * @wr_exe:	start-write bit (self-clearing)
 * @type_shift:	shift of the table-type field
 * @type_mask:	mask of the table-type field, AFTER the shift
 * @idx_mask:	mask of the entry index, which sits at bit 0
 *
 * A die whose layout nobody has measured has NO instance of this -- see the
 * file header.  There is deliberately no "default" layout to fall back to.
 */
struct luna_l34_cmd_layout {
	u32	rd_exe;
	u32	wr_exe;
	u8	type_shift;
	u32	type_mask;
	u32	idx_mask;
};

/**
 * struct luna_l34_acc - one die's indirect table-access registers.
 * @cmd:	command/trigger register offset
 * @clr:	per-table-type reset (self-clearing)
 * @rdata:	read-data bank base (word 0)
 * @wdata:	write-data bank base (word 0)
 * @cmd_layout:	the command word's field positions, or NULL when NOT ESTABLISHED
 */
struct luna_l34_acc {
	u32					cmd;
	u32					clr;
	u32					rdata;
	u32					wdata;
	const struct luna_l34_cmd_layout	*cmd_layout;
};

/*
 * RTL9602C -- the NAPT model.  Every value here is what `rtl9602c_l34.h` has
 * carried and what the shipping driver has been using; moving it changes no
 * byte the engine sees.
 */
static const struct luna_l34_cmd_layout luna_l34_cmd_rtl9602c = {
	.rd_exe		= BIT(25),
	.wr_exe		= BIT(24),
	.type_shift	= 16,		/* [19:16] table type */
	.type_mask	= 0xf,
	.idx_mask	= 0xffff,	/* [15:0] entry index */
};

static const struct luna_l34_acc luna_l34_acc_rtl9602c = {
	.cmd		= 0x800100,
	.clr		= 0x800104,
	.rdata		= 0x800108,
	.wdata		= 0x80011c,
	.cmd_layout	= &luna_l34_cmd_rtl9602c,
};

/*
 * RTL9603CVD -- the FLOWBASED model.
 *
 * ADDRESSES: established.  Its own chipdef names NAT_TBL_ACCESS_CTRL 0x801100,
 * _CLR 0x801104, _RDDATA 0x801110, _WRDATA 0x801180, and that board's stock
 * capture finds the page live (43 non-zero words at 0x801000 with the vendor
 * engine forwarding) while the NAPT model's page answers 1024 words of zero on
 * this die -- two independent tiers agreeing on where the engine is.
 *
 * LAYOUT: NOT ESTABLISHED, deliberately NULL.  The addresses being one page
 * apart says nothing about the bits inside the command word, and this family
 * already has a measured case of a block MOVING between these two revisions.
 * Recovering it means reading `rtk_fc_flowEntry_path12_setting` out of that
 * board's own stock kernel, which needs symbol addresses (the image carries a
 * kallsyms table and no .symtab).  Until then every access REFUSES.
 */
static const struct luna_l34_acc luna_l34_acc_rtl9603cvd = {
	.cmd		= 0x801100,
	.clr		= 0x801104,
	.rdata		= 0x801110,
	.wdata		= 0x801180,
	.cmd_layout	= NULL,		/* NOT ESTABLISHED -- refuses, never guesses */
};

/**
 * luna_l34_tbl_op() - one indirect table access.  -> 0, or a negative errno.
 * @sw:		the switch-core MMIO base
 * @a:		this die's access table
 * @type:	table-type selector, as that die's engine numbers them
 * @idx:	entry index
 * @w:		the words -- written from on a write, filled in on a read
 * @n:		how many words this table moves
 * @write:	true to write, false to read
 *
 * ⚠ THREE ANSWERS, never two: -ENXIO means this die's command-word layout was
 * never measured, so nothing was attempted; -ETIMEDOUT means the engine did not
 * clear EXE, which is a fact about the silicon; 0 means it completed.  An
 * unmeasured layout must never read as a failed engine.
 */
static inline int luna_l34_tbl_op(void __iomem *sw,
				  const struct luna_l34_acc *a,
				  u8 type, u16 idx, u32 *w, unsigned int n,
				  bool write)
{
	const struct luna_l34_cmd_layout *L;
	u32 cmd, exe;
	unsigned int i, t;

	if (!sw || !a || !a->cmd_layout)
		return -ENXIO;
	if (n > LUNA_L34_WORDS_MAX)
		return -EINVAL;
	L = a->cmd_layout;
	exe = write ? L->wr_exe : L->rd_exe;

	if (write)
		for (i = 0; i < n; i++)
			writel(w[i], sw + a->wdata + 4 * i);

	cmd = exe | ((u32)(type & L->type_mask) << L->type_shift) |
	      ((u32)idx & L->idx_mask);
	writel(cmd, sw + a->cmd);

	for (t = 0; t < LUNA_L34_EXE_POLL_US; t++) {
		if (!(readl(sw + a->cmd) & exe))
			break;
		udelay(1);
	}
	if (readl(sw + a->cmd) & exe)
		return -ETIMEDOUT;

	if (!write)
		for (i = 0; i < n; i++)
			w[i] = readl(sw + a->rdata + 4 * i);
	return 0;
}

#endif /* _LUNA_L34_ACC_H */
