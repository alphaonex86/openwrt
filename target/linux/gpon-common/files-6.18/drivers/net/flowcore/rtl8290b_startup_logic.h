/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* RTL8290B laser start-up, driven by THIS unit's parameter block (150 bytes of
 * its own rtl8290b.data at byte 1400): the APC/ERC set-points, the MCU offset
 * walks, the DAC seeds, the loop mode and the FSU. Every field is range-checked
 * before the first write, so a refused block leaves the laser untouched. In
 * service (O5) the same block drives the periodic bias/modulation re-centring. */
#ifndef _RTL8290B_STARTUP_LOGIC_H
#define _RTL8290B_STARTUP_LOGIC_H

#include <linux/types.h>

#define RTL8290B_PARAM_FILE_OFF	1400u	/* in rtl8290b.data */
#define RTL8290B_PARAM_LEN	150u	/* exactly this, or refused */
#define RTL8290B_STARTUP_WALKS	12u	/* MCU commands, each followed by a 0x31d read */

/* Register I/O, supplied by the family shell; 0 or a negative errno. */
struct rtl8290b_io {
	void *ctx;
	int (*rd)(void *ctx, u16 reg, u8 *val);
	int (*wr)(void *ctx, u16 reg, u8 val);
	void (*delay_us)(void *ctx, unsigned int us);
};

/* Why the start-up ended. err: 0 done; -EINVAL block refused (param = its index,
 * or RTL8290B_PARAM_LEN for its length); -EOPNOTSUPP a mode this port lacks
 * (param); -EIO a bus access failed (reg); -ENODEV the MCU did not power on. */
struct rtl8290b_startup_res {
	int err;
	u16 reg;
	u8 param;
	u8 r29[RTL8290B_STARTUP_WALKS];	/* 0x31d after each MCU command */
	u32 accesses, failed;		/* bus accesses issued / refused */
	bool fsu_on;			/* FSU started: the shell watches it settle */
};

/* Validate @blk, then run the start-up through @io. @io NULL validates only. */
int rtl8290b_startup(const struct rtl8290b_io *io, const u8 *blk, u32 len,
		     struct rtl8290b_startup_res *res);

/* The in-service re-centring: every RTL8290B_IBIM_PERIOD_MS in O5, the APC's
 * bias/modulation readings are moved back toward the block's GPON targets
 * (Bt/Mt, BE32 at 142/146, uA) with the loop briefly open. */
#define RTL8290B_IBIM_PERIOD_MS	5000u
#define RTL8290B_IBIM_MAX_UA	100000u	/* the DAC's full scale; a larger target is refused */

struct rtl8290b_ibim {
	bool armed;			/* in O5, next evaluation at due_ms */
	u32 due_ms;
	u32 evals, kicks;		/* evaluations / seeds written */
	u32 bias_ua, mod_ua;		/* the last readings */
	u16 bias_code, mod_code;	/* the last seeds */
	int err;			/* last evaluation: 0, -EINVAL (block), -ERANGE (seed), -EIO (reg) */
	u16 reg;
	u32 accesses, failed;
};

/* @now_ms: the caller's clock (wraps). Out of O5: disarmed, no bus access. */
int rtl8290b_ibim_poll(const struct rtl8290b_io *io, const u8 *blk, u32 len,
		       struct rtl8290b_ibim *st, u32 now_ms, bool in_o5);

/* 0x27c[7:6] (the running loop mode) back to the block's numbering, the
 * inverse of the start-up's 0->0 1->3 2->1 3->2. */
static inline u8 rtl8290b_loop_mode_of(u8 reg27c)
{
	static const u8 mode_of[4] = { 0, 2, 3, 1 };

	return mode_of[reg27c >> 6];
}

#endif /* _RTL8290B_STARTUP_LOGIC_H */
