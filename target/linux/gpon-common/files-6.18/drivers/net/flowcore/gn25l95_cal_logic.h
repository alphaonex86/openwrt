/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GN25L95 per-unit calibration: the ORDERED OPERATIONS, as data.
 *
 * INVARIANT: this file computes what to do and never does it. No MMIO, no I2C,
 * no device, no sleep -- so the whole sequence is replayable on x86 and a
 * divergence from stock is a unit test, not a board boot.
 *
 * ONE ROUTINE, TWO VARIANT FIELDS. Both units' stock binaries agree on every
 * calibration range, index, order, the F8/F9/FC/FD read-modify-writes, the 0x6E
 * pulse, A0 = 0x6a and the password writes. They differ in exactly two places,
 * so those are DATA and not a second implementation.
 */
#ifndef _GN25L95_CAL_LOGIC_H
#define _GN25L95_CAL_LOGIC_H

#include <linux/types.h>

#define GN_CAL_LEN	1024u		/* the file is exactly this, or refused */

enum gn_op_kind {
	GN_SELECT,		/* choose a page/table: write `val` to `reg`   */
	GN_WRITE,		/* reg <- val                                  */
	GN_RMW,			/* reg <- (reg & ~mask) | val                  */
	GN_READ_EXPECT,		/* read reg; (v & mask) must equal val         */
	GN_READ_NOTE,		/* read reg; recorded, not judged (diagnostic) */
	GN_PULSE,		/* read reg once, then write (v|val) and (v&mask) */
};

struct gn_op {
	u8  kind;
	u8  slave;
	u16 reg;
	u8  val;
	u8  mask;
};

/* The two places the units' own binaries differ. */
struct gn_variant {
	bool bb_set;		/* G24W: BB |= 1   ·  X400AXF: BB &= ~1        */
	bool tail_c4_7f;	/* G24W only: page2 C4 = 0x7f, then 0x50:7f    */
};

extern const struct gn_variant gn_variant_g24w;
extern const struct gn_variant gn_variant_x400axf;

/*
 * Build the sequence for `cal` (GN_CAL_LEN bytes) into `out`.
 * -> number of ops, or -EINVAL when the inputs cannot produce a valid one.
 * The caller executes them in order and stops at the first failure; nothing
 * here assumes any of them succeed.
 */
int gn25l95_cal_ops(const u8 *cal, u32 cal_len, const struct gn_variant *v,
		    struct gn_op *out, u32 out_max);

#define GN_OPS_MAX	1200u

/*
 * Byte I/O, supplied by the family shell. The core builds and drives the
 * sequence; it never knows what bus is underneath -- which is what lets the
 * same calibration run on Cortina's i2c master and on Luna's, and lets the
 * whole thing be replayed against an archived bus trace on x86.
 * Both return 0 on success and a negative errno otherwise.
 */
struct gn_io {
	void *ctx;
	int (*rd)(void *ctx, u8 slave, u8 reg, u8 *val);
	int (*wr)(void *ctx, u8 slave, u8 reg, u8 val);
	/*
	 * Where GN_READ_NOTE values land, in order, when `notes` is non-NULL.
	 * The probe needs them: excluding the GN28L9x family is a test on the
	 * TRIPLE 0x80/0x85/0x86, which no per-byte expectation can express.
	 */
	u8  *notes;
	u32 notes_max;
	u32 notes_n;
};

/*
 * Where the sequence stopped. `op` is the index into the op list, so the caller
 * can name the operation rather than a write count. `err` is the bus error, or
 * 0 when the bus was fine and the part ANSWERED something it should not have --
 * the two are different faults and a shell that merges them cannot tell a dead
 * bus from an unprogrammed part.
 */
struct gn_fail {
	u32 op;
	u8  reg, got, want, mask;
	int err;
};

/*
 * Execute `ops` in order, stopping at the FIRST failure. -> 0, or the negative
 * errno of the failing bus operation, or -EIO when a readback was refused.
 * `f` is written ONLY on failure -- a caller may read it exactly when this
 * returns non-zero, and it still holds whatever it held on success.
 */
int gn25l95_cal_apply(const struct gn_op *ops, u32 n, struct gn_io *io,
		      struct gn_fail *f);

/*
 * POSITIVE IDENTIFICATION, exactly as both units' own stock detectors do it:
 * 11 bus events, three conditions, and the third alone is not enough.
 *   page 0   : 0x7b..0x7e must all read 0 -- the part is UNLOCKED
 *   page 0xff: 0x80/0x85/0x86 are the GN28L9x family id, NOTED not judged
 *   page 2   : 0xd1 high nibble 0xa0 IS a GN25L95 (measured 0xa3 on the wire)
 *
 * ⚠ D1 ALONE ACCEPTS PARTS BOTH ORIGINALS REJECT: a locked part with a
 *   nonzero password, and a GN28L9x whose D1 happens to match.
 *
 * ⚠ STOCK IGNORES THE FAILURE OF ITS TWO PAGE-SELECT WRITES HERE. We do not
 *   inherit that: a select nobody checked means the reads that follow came
 *   from whatever page was already current.
 */
#define GN_PROBE_OPS	11u
#define GN_PROBE_NOTES	3u

int gn25l95_probe_ops(struct gn_op *out, u32 out_max);

/*
 * -> true when the three noted bytes are a GN28L9x family id, i.e. NOT this
 * part. The union of both detectors' tables is refused: the X400AXF's knows
 * "G98" and the older G24W's does not, and refusing a part neither board
 * carries is the safe side of that difference. DECLARED, not silent.
 */
bool gn25l95_is_gn28l9x(const u8 id[GN_PROBE_NOTES]);

#endif /* _GN25L95_CAL_LOGIC_H */
