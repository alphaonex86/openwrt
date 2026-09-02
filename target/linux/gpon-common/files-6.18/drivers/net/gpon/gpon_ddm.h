/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_ddm.h -- optical diagnostic (DDM) unit conversion, shared core.
 *
 * The SFF-8472 A2h optical monitors report power as an unsigned 16-bit count
 * of 0.1 uW.  Every consumer here -- the G.988 ANI-G optical-level attributes,
 * the ethtool/proc diagnostic reports -- wants dBm instead, so the conversion
 * is a pure log10 and belongs to no chip and no family.
 *
 * THE ZERO IS DELIBERATELY NOT HANDLED HERE.  A raw count of 0 means "no
 * reading", and the two families already report that absence differently
 * (Luna floors it at -40.00 dBm, Cortina returns a dedicated NONE sentinel).
 * Unifying that would change what each one publishes, which is a decision
 * about the report and not about the arithmetic -- so the caller keeps its own
 * zero policy and this function is defined only for raw >= 1.
 */
#ifndef _GPON_DDM_H
#define _GPON_DDM_H

#include <linux/types.h>

/**
 * gpon_ddm_uw10_to_cdbm() - 0.1 uW count to centi-dBm (1 mW = 0 dBm)
 * @raw: SFF-8472 optical power count, in units of 0.1 uW.  MUST be >= 1.
 *
 * Return: power in centi-dBm.  Worst-case error 3.36 centi-dBm (0.034 dB)
 * over the whole 16-bit domain, mean 1.47 -- see gpon_ddm.c for how that was
 * measured, and rtl9607c-test/optic_ddm_test.c for the case that pins it.
 */
s32 gpon_ddm_uw10_to_cdbm(u32 raw);

#endif /* _GPON_DDM_H */
