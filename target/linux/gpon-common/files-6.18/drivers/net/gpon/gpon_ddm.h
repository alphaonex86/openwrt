/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_ddm.h -- optical diagnostic (DDM) unit conversion, shared core.
 *
 * The SFF-8472 A2h monitors report power as an unsigned 16-bit count of
 * 0.1 uW; every consumer (G.988 ANI-G optical levels, the ethtool/proc
 * reports) wants dBm, so the conversion is a pure log10 belonging to no chip.
 *
 * THE ZERO IS DELIBERATELY NOT HANDLED HERE.  A raw count of 0 means "no
 * reading" and the two families already report that absence differently (Luna
 * floors at -40.00 dBm, Cortina returns a NONE sentinel).  Unifying it would
 * change what each publishes -- a decision about the report, not the
 * arithmetic -- so this function is defined only for raw >= 1.
 */
#ifndef _GPON_DDM_H
#define _GPON_DDM_H

#include <linux/types.h>

/**
 * gpon_ddm_uw10_to_cdbm() - 0.1 uW count to centi-dBm (1 mW = 0 dBm)
 * @raw: SFF-8472 optical power count, in units of 0.1 uW.  MUST be >= 1.
 *
 * Return: power in centi-dBm.  Worst-case error 3.36 centi-dBm (0.034 dB) over
 * the whole 16-bit domain, mean 1.47 -- measured in gpon_ddm.c, pinned by
 * rtl9607c-test/optic_ddm_test.c.
 */
s32 gpon_ddm_uw10_to_cdbm(u32 raw);

/**
 * gpon_ddm_cdbm_to_anig() - centi-dBm to the G.988 ANI-G (ME 263) wire unit
 * @cdbm: power in centi-dBm.
 *
 * ANI-G #10/#14 report in 0.002 dB steps referred to 1 mW, two's complement --
 * five counts per centi-dBm.  The factor was a bare `* 5` in both families; it
 * is named here so the unit is stated once.  Clamped to the s16 the attribute
 * ships in; each family's own outer dBm bound is inert over that range and is
 * not unified.
 */
s16 gpon_ddm_cdbm_to_anig(s32 cdbm);

#endif /* _GPON_DDM_H */
