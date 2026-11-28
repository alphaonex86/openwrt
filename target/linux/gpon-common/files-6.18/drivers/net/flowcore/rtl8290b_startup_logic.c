// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CORE */
/* RTL8290B laser start-up from the unit's parameter block (see the header). */
#include <linux/types.h>
#include <linux/errno.h>

#include "rtl8290b_startup_logic.h"

struct su {
	const struct rtl8290b_io *io;	/* NULL: validation pass, no bus access */
	const u8 *blk;		/* exactly RTL8290B_PARAM_LEN bytes, checked once */
	struct rtl8290b_startup_res *res;	/* the start-up's only */
	u16 *reg;		/* the bus account of the caller's result */
	u32 *accesses, *failed;
};

/* The one way a field leaves the block, range-checked; every idx is a constant
 * below RTL8290B_PARAM_LEN. */
static int prm(struct su *s, u8 idx, u8 max, u8 *v)
{
	if (s->blk[idx] > max) {
		s->res->param = idx;
		return -EINVAL;
	}
	*v = s->blk[idx];
	return 0;
}

static int bus(struct su *s, u16 reg, int rc)
{
	(*s->accesses)++;
	if (rc) {
		(*s->failed)++;
		*s->reg = reg;
		return -EIO;
	}
	return 0;
}

static int rd(struct su *s, u16 reg, u8 *v)
{
	*v = 0;
	return s->io ? bus(s, reg, s->io->rd(s->io->ctx, reg, v)) : 0;
}

static int wr(struct su *s, u16 reg, u8 v)
{
	return s->io ? bus(s, reg, s->io->wr(s->io->ctx, reg, v)) : 0;
}

static void delay_us(struct su *s, unsigned int us)
{
	if (s->io)
		s->io->delay_us(s->io->ctx, us);
}

static u8 lsb_of(u8 mask)
{
	u8 sh = 0;

	while (!(mask & 1)) {
		mask >>= 1;
		sh++;
	}
	return sh;
}

/* reg[mask] = v, read-modify-write; v fits the field (prm checked it). */
static int fld(struct su *s, u16 reg, u8 mask, u8 v)
{
	u8 sh = lsb_of(mask), cur;
	int e;

	e = rd(s, reg, &cur);
	return e ? e : wr(s, reg, (cur & ~mask) | ((v << sh) & mask));
}

/* reg[mask] = block[idx], refused when it does not fit the field. */
static int fldp(struct su *s, u16 reg, u8 mask, u8 idx)
{
	u8 v;
	int e = prm(s, idx, mask >> lsb_of(mask), &v);

	return e ? e : fld(s, reg, mask, v);
}

static int bit(struct su *s, u16 reg, u8 b, u8 v)
{
	return fld(s, reg, (u8)(1u << b), v);
}

static const u8 walk1[] = { 0xa8, 0xb0, 0xd0, 0xd8, 0xe8, 0xe0 };
static const u8 walk2[] = { 0xb0, 0xd0, 0xb8, 0xb0, 0xd0, 0xc0 };

/* MCU commands into 0x24d, each: 10 ms, then the 0x31d status read. */
static int walk(struct su *s, const u8 *cmd, u32 n, u32 first)
{
	u32 i;
	int e;

	for (i = 0; i < n; i++) {
		e = wr(s, 0x24d, cmd[i]);
		if (e)
			return e;
		delay_us(s, 10000);
		e = rd(s, 0x31d, &s->res->r29[first + i]);
		if (e)
			return e;
	}
	return 0;
}

/* A 12-bit DAC seed: hi-8 into @hi, low nibble into 0x238[@lo_mask], latched by
 * the 0x23d b7 strobe. */
static int dac_seed(struct su *s, u8 idx, u16 hi, u8 lo_mask)
{
	u8 a, b;
	u16 code;
	int e;

	e = prm(s, idx, 0x0f, &a);
	e = e ? e : prm(s, idx + 1, 0xff, &b);
	if (e)
		return e;
	code = (u16)a << 8 | b;
	e = bit(s, 0x23d, 7, 0);
	e = e ? e : wr(s, hi, code >> 4);
	e = e ? e : fld(s, 0x238, lo_mask, code & 0x0f);
	return e ? e : bit(s, 0x23d, 7, 1);
}

/* The block's loop mode (0..3) as 0x27c[7:6] holds it. */
static const u8 loop_reg[4] = { 0, 3, 1, 2 };

#define TRY(x) do { int _e = (x); if (_e) return _e; } while (0)

static int run(struct su *s)
{
	u8 v, txsd_mode, fsu, fsu_skip, t;

	/* Modes this port does not carry are refused, never approximated. */
	TRY(prm(s, 0x65, 1, &v));		/* bias/mod from a temperature LUT */
	if (v) {
		s->res->param = 0x65;
		return -EOPNOTSUPP;
	}
	TRY(prm(s, 0x72, 3, &v));		/* ERC offset compensation */
	if (v) {
		s->res->param = 0x72;
		return -EOPNOTSUPP;
	}

	/* MCU power-on, read back. */
	TRY(wr(s, 0x380, 0x01));
	TRY(rd(s, 0x380, &v));
	if (s->io && v != 0x01) {
		s->res->reg = 0x380;
		return -ENODEV;
	}
	delay_us(s, 10000);

	TRY(bit(s, 0x24e, 7, 1));
	TRY(wr(s, 0x23e, 0xfd));
	TRY(wr(s, 0x23f, 0xfd));

	/* APC/ERC set-points and the TX signal-detect. */
	TRY(prm(s, 0x35, 1, &txsd_mode));
	TRY(bit(s, 0x23e, 0, txsd_mode));
	TRY(fldp(s, 0x24c, 0xe0, 0x36));
	TRY(fldp(s, 0x244, 0xc0, 0x27));
	TRY(prm(s, 0x1a, 0xff, &v));		/* average-power target */
	TRY(wr(s, 0x23a, v));
	TRY(fldp(s, 0x23b, 0xf0, 0x1b));	/* extinction ratio */
	TRY(fldp(s, 0x23b, 0x0c, 0x5d));
	TRY(fldp(s, 0x23b, 0x03, 0x5e));
	TRY(fldp(s, 0x240, 0x03, 0x1e));	/* GPON (rtk mode 0): the block's value; 0 is EPON's */
	TRY(fldp(s, 0x24c, 0x10, 0x37));
	TRY(fldp(s, 0x24c, 0x08, 0x38));
	TRY(fldp(s, 0x24c, 0x04, 0x39));
	TRY(fldp(s, 0x24c, 0x03, 0x3a));
	TRY(fldp(s, 0x27c, 0x03, 0x3b));

	/* MCU offset calibration. */
	TRY(walk(s, walk1, sizeof(walk1), 0));
	TRY(bit(s, 0x243, 7, 1));
	TRY(prm(s, 0x21, 1, &v));
	TRY(bit(s, 0x284, 6, v));
	TRY(bit(s, 0x284, 5, v));
	TRY(bit(s, 0x284, 4, v));
	TRY(rd(s, 0x27c, &t));
	TRY(wr(s, 0x27c, t & 0xf7));
	TRY(wr(s, 0x27c, t | 0x08));
	TRY(walk(s, walk2, sizeof(walk2), sizeof(walk1)));

	/* DAC seeds, loop mode. */
	TRY(dac_seed(s, 0x14, 0x236, 0x0f));
	TRY(dac_seed(s, 0x16, 0x237, 0xf0));
	TRY(prm(s, 0x18, 3, &v));
	TRY(fld(s, 0x27c, 0xc0, loop_reg[v]));
	TRY(prm(s, 0x19, 0xff, &v));
	TRY(bit(s, 0x24a, 4, !!(v & 4)));

	/* Loop timing and gains. */
	if (txsd_mode)
		TRY(fldp(s, 0x23d, 0x70, 0x12));
	else
		TRY(fld(s, 0x23d, 0x70, 2));
	TRY(fldp(s, 0x23d, 0x03, 0x13));
	TRY(fldp(s, 0x239, 0x38, 0x1f));
	TRY(fldp(s, 0x239, 0x07, 0x20));
	TRY(fldp(s, 0x23c, 0xf8, 0x1c));
	TRY(fldp(s, 0x23c, 0x07, 0x1d));
	TRY(fldp(s, 0x241, 0x40, 0x29));
	TRY(fldp(s, 0x241, 0x30, 0x2a));
	TRY(fldp(s, 0x241, 0x0c, 0x2b));
	TRY(fldp(s, 0x241, 0x03, 0x2c));
	TRY(fldp(s, 0x242, 0x80, 0x2d));
	TRY(fldp(s, 0x242, 0x60, 0x2e));
	TRY(fldp(s, 0x242, 0x18, 0x2f));
	TRY(fldp(s, 0x242, 0x06, 0x30));

	/* FSU. */
	TRY(prm(s, 0x6b, 0xff, &fsu_skip));
	TRY(prm(s, 0x28, 1, &fsu));
	if (!fsu_skip && fsu) {
		TRY(bit(s, 0x27c, 5, 0));
		TRY(bit(s, 0x27c, 4, 1));
		TRY(bit(s, 0x20e, 7, 1));
	}
	TRY(bit(s, 0x27c, 5, fsu));
	s->res->fsu_on = !fsu_skip && fsu;

	/* Current limits, each loaded under its 0x246 enable. */
	TRY(bit(s, 0x246, 2, 0));
	TRY(prm(s, 0x24, 0xff, &v));
	TRY(wr(s, 0x247, v));
	TRY(bit(s, 0x246, 2, 1));
	TRY(bit(s, 0x246, 1, 0));
	TRY(prm(s, 0x25, 0xff, &v));
	TRY(wr(s, 0x248, v));
	TRY(bit(s, 0x246, 1, 1));
	TRY(bit(s, 0x246, 0, 0));
	TRY(prm(s, 0x26, 0xff, &v));
	TRY(wr(s, 0x249, v));
	TRY(bit(s, 0x246, 0, 1));
	TRY(fldp(s, 0x246, 0x20, 0x5f));
	TRY(fldp(s, 0x246, 0x08, 0x61));
	TRY(prm(s, 0x23, 0xff, &v));
	TRY(wr(s, 0x283, v));
	TRY(fldp(s, 0x284, 0x80, 0x22));

	TRY(rd(s, 0x31f, &v));
	return fld(s, 0x232, 0xc0, v & 0x03);
}

static bool erased(const u8 *blk, u32 len)
{
	u32 i;

	for (i = 1; i < len; i++)
		if (blk[i] != blk[0])
			return false;
	return blk[0] == 0x00 || blk[0] == 0xff;
}

int rtl8290b_startup(const struct rtl8290b_io *io, const u8 *blk, u32 len,
		     struct rtl8290b_startup_res *res)
{
	struct su s = { .io = NULL, .blk = blk, .res = res, .reg = &res->reg,
			.accesses = &res->accesses, .failed = &res->failed };
	u32 i;

	res->err = 0;
	res->reg = 0;
	res->param = 0;
	res->accesses = res->failed = 0;
	res->fsu_on = false;
	for (i = 0; i < RTL8290B_STARTUP_WALKS; i++)
		res->r29[i] = 0;
	if (!blk || len != RTL8290B_PARAM_LEN || erased(blk, len)) {
		res->param = RTL8290B_PARAM_LEN;
		return res->err = -EINVAL;
	}
	res->err = run(&s);			/* the whole block, before any write */
	if (res->err || !io)
		return res->err;
	s.io = io;
	return res->err = run(&s);
}

static u32 be32_at(const u8 *p)
{
	return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3];
}

/* An APC monitor current in uA: the 12-bit code (hi-8 in @hi, low nibble in
 * 0x322[@lo_mask]) * 100000 / 8192, doubled. */
static int monitor_ua(struct su *s, u16 hi, u8 lo_mask, u32 *ua)
{
	u8 h, l;
	int e = rd(s, hi, &h);

	e = e ? e : rd(s, 0x322, &l);
	if (!e)
		*ua = 2 * ((((u32)h << 4 | (u32)(l & lo_mask) >> lsb_of(lo_mask)) * 100000u) >> 13);
	return e;
}

/* Half a target current (the seed's unit) as a 12-bit code; above 50000 refused.
 * Full scale is 0xfff for the modulation, wraps to 0 for the bias (as stock). */
static int seed_code(u32 half, bool full_scale_max, u16 *code)
{
	if (half > 50000u)
		return -ERANGE;
	*code = half == 50000u && full_scale_max ? 0xfff : ((half << 13) / 100000u) & 0xfff;
	return 0;
}

/* The re-centred bias/modulation (uA), or false inside the window. The
 * arithmetic wraps as u32 and clamps signed, as stock does. */
static bool ibim_target(u32 b, u32 m, u32 bt, u32 mt, s32 *nb, s32 *nm)
{
	if (b > bt * 8 / 10) {
		u32 m2 = m + 2 * (b - bt);

		if (m2 < mt * 8 / 10) {
			*nb = (s32)(b - ((mt - m) >> 1));
			*nm = (s32)mt;
		} else {
			*nb = (s32)bt;
			*nm = (s32)m2;
		}
		if (*nm > 80000)
			*nm = 80000;
	} else if (m < mt * 8 / 10) {
		*nb = (s32)(b - ((mt - m) >> 1));
		*nm = (s32)mt;
	} else {
		return false;
	}
	if (*nb < 4000)
		*nb = 4000;
	return true;
}

/* Read, and when out of the window seed both DACs under one 0x23d strobe with
 * the loop open; the loop mode is restored whatever failed before it. */
static int ibim_kick(struct su *s, struct rtl8290b_ibim *st, u32 bt, u32 mt)
{
	u8 loop;
	s32 nb, nm;
	u16 mc = 0, bc = 0;
	int e, r;

	TRY(rd(s, 0x27c, &loop));
	TRY(monitor_ua(s, 0x320, 0xf0, &st->mod_ua));
	TRY(monitor_ua(s, 0x321, 0x0f, &st->bias_ua));
	if (!ibim_target(st->bias_ua, st->mod_ua, bt, mt, &nb, &nm))
		return 0;
	e = fld(s, 0x27c, 0xc0, 0);
	if (!e) {
		e = seed_code((u32)(nm / 2), true, &mc);
		e = e ? e : seed_code((u32)nb >> 1, false, &bc);
		e = e ? e : bit(s, 0x23d, 7, 0);
		e = e ? e : fld(s, 0x237, 0xff, mc >> 4);
		e = e ? e : fld(s, 0x238, 0xf0, mc & 0x0f);
		e = e ? e : fld(s, 0x236, 0xff, bc >> 4);
		e = e ? e : fld(s, 0x238, 0x0f, bc & 0x0f);
		e = e ? e : bit(s, 0x23d, 7, 1);
	}
	r = fld(s, 0x27c, 0xc0, loop >> 6);
	if (e || r)
		return e ? e : r;
	st->mod_code = mc;
	st->bias_code = bc;
	st->kicks++;
	return 0;
}

int rtl8290b_ibim_poll(const struct rtl8290b_io *io, const u8 *blk, u32 len,
		       struct rtl8290b_ibim *st, u32 now_ms, bool in_o5)
{
	struct su s = { .io = io, .blk = blk, .reg = &st->reg,
			.accesses = &st->accesses, .failed = &st->failed };
	u32 bt, mt;

	if (!in_o5) {
		st->armed = false;
		return 0;
	}
	if (!io || !blk || len != RTL8290B_PARAM_LEN)
		return st->err = -EINVAL;
	if (!st->armed) {
		st->armed = true;
		st->due_ms = now_ms + RTL8290B_IBIM_PERIOD_MS;
		return 0;
	}
	if ((s32)(now_ms - st->due_ms) < 0)
		return 0;
	st->due_ms = now_ms + RTL8290B_IBIM_PERIOD_MS;
	st->evals++;
	/* Stock gates this on a BWmap grant < 125 bytes; its count always includes
	 * an empty first bucket, so it is true whenever in O5: not ported. */
	bt = be32_at(blk + 142);
	mt = be32_at(blk + 146);
	bt = bt ? bt : 16000;
	mt = mt ? mt : 27000;
	if (bt > RTL8290B_IBIM_MAX_UA || mt > RTL8290B_IBIM_MAX_UA)
		return st->err = -EINVAL;
	return st->err = ibim_kick(&s, st, bt, mt);
}
