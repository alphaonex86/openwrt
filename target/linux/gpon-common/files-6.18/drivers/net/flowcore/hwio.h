/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * hwio.h -- the register accessor, injected, so LOGIC stops carrying MMIO.
 *
 * ★★★ WHY THIS EXISTS (operator, 2026-08-28, and it is the root cause of his
 * actual complaint). He asked why a port is not one hour of work: *"el port
 * deberia ser simple y solo hacer por equipo una lista de registros/datos y tal
 * vez algunos workaround no mas"*. It should be. It is not, and MEASURED three
 * ways in one sitting, this is why:
 *
 *     of the shells' functions, only ~31 in the four biggest files can move to
 *     common code untouched. The rest are not board-specific in their
 *     REASONING -- they have a readl() sitting in the middle of it.
 *
 * So the barrier is not that the logic differs per board. The barrier is that
 * every function that decides anything also DOES the I/O itself, and a function
 * that does I/O cannot be shared by two silicons that share no register.
 *
 * ★ THE FIX IS THE ONE THIS TREE ALREADY PROVED ON A SMALL SCALE.
 * luna_ponmac.c carries `struct luna_ops { rd, wr }` and serves FOUR
 * chips from one object with per-chip tables. That is the whole idea, and it
 * works today. What is missing is that it stopped there: the accessor is
 * Realtek-shaped (a bare physical address, no context pointer), so the Cortina
 * side could not adopt it and grew its own everything.
 *
 * ★ WHAT CHANGES FOR A NEW BOARD, which is the point:
 *     today   copy a driver, edit it until it works, then re-find every bug
 *             that was already fixed on the other board (this happened FOUR
 *             times in one day: a serial-number decoder that accepted garbage,
 *             an OMCI message type that was the ALARM code, a missing
 *             retransmission cache, fifteen documented A/B commands the kernel
 *             ignores).
 *     after   a table of offsets, a `struct hwio` that knows how to reach the
 *             block, and the workarounds that are genuinely this silicon's.
 *
 * ★ THE CONTRACT IS DELIBERATELY TINY. Two function pointers and a context.
 * Anything richer (locking, retries, tracing) belongs to the SHELL that
 * provides them -- the core must stay something that decides and never does,
 * so it keeps building on x86 for offline fuzzing.
 */
#ifndef _HWIO_H
#define _HWIO_H

#include <linux/types.h>

/**
 * struct hwio - how a caller reaches one register block.
 * @rd:  read a 32-bit register at @off within the block.
 * @wr:  write one.
 * @ctx: whatever the shell needs to do that -- an iomem base, a proxy handle,
 *       a device. The core NEVER looks inside it, which is what keeps the core
 *       free of struct device and free of any one vendor's shape.
 *
 * ★ @off IS AN OFFSET WITHIN A BLOCK, NEVER AN ABSOLUTE ADDRESS. That is the
 *   difference that lets one function serve two silicons: the offsets come
 *   from a per-chip table, and the block base lives in @ctx where the shell
 *   put it. An accessor taking absolute addresses -- which is what
 *   luna_ops does today -- silently ties its callers to one memory map.
 */
struct hwio {
	u32  (*rd)(void *ctx, u32 off);
	void (*wr)(void *ctx, u32 off, u32 val);
	void *ctx;
};

/** Read @off. Returns 0 when @io is not wired -- a caller that cares must ask. */
static inline u32 hwio_rd(const struct hwio *io, u32 off)
{
	return (io && io->rd) ? io->rd(io->ctx, off) : 0u;
}

/** Write @off. A no-op when @io is not wired. */
static inline void hwio_wr(const struct hwio *io, u32 off, u32 val)
{
	if (io && io->wr)
		io->wr(io->ctx, off, val);
}

/**
 * hwio_rmw() - read-modify-write the field [@msb:@lsb] at @off.
 *
 * ★ ONE IMPLEMENTATION OF THE MASK ARITHMETIC, because both families have
 *   their own today and a field written one bit wide too far is the class of
 *   bug that reads back correct and behaves wrong.
 */
static inline void hwio_rmw(const struct hwio *io, u32 off, u8 msb, u8 lsb,
			    u32 val)
{
	u32 mask = (msb == 31 && lsb == 0) ? 0xffffffffu
					   : (((1u << (msb - lsb + 1)) - 1u) << lsb);

	hwio_wr(io, off, (hwio_rd(io, off) & ~mask) | ((val << lsb) & mask));
}

#endif /* _HWIO_H */
