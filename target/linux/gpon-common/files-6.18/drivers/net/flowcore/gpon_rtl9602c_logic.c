// SPDX-License-Identifier: GPL-2.0-only
/* See gpon_rtl9602c_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/limits.h>

#include "gpon_rtl9602c_logic.h"

/* Map an RTL8290B 12-bit register number to its paging I2C slave address. */
u8 bosa_slave_for(u16 reg)
{
	switch (reg >> 8) {
	case 0:  return 0x50;
	case 1:  return 0x51;
	case 2:  return 0x54;
	default: return 0x55;		/* page 3 and up */
	}
}

/*
 * Convert a raw SFF-8472 DDM power word (unit 0.1 microWatt/LSB) to the ANI-G
 * level encoding. power_mW = raw/10000, so
 *   level = 500 * 10*log10(raw/10000) = 1505*log2(raw)/65536 - 20000
 * (5000*log10(2) = 1505.15). log2 is computed in Q16 fixed point (fls for the
 * integer part + linear interpolation of the mantissa; ~0.25 dB error, ample for
 * the diagnostic ANI-G report) so no kernel FPU is needed. Returns INT_MIN when
 * the word reads "not available" (0x0000/0xffff) so the caller keeps the cache.
 */
s32 ddm_word_to_level(int raw)
{
	int e;
	u32 frac;
	s64 log2_q16, level;

	if (raw <= 0 || raw >= 0xffff)
		return INT_MIN;
	e = fls((u32)raw) - 1;				/* floor(log2(raw))      */
	frac = e ? (((u32)(raw - (1u << e)) << 16) >> e) : 0;	/* mantissa frac, Q16 */
	log2_q16 = ((s64)e << 16) | frac;		/* log2(raw) in Q16      */
	level = ((log2_q16 * 1505) >> 16) - 20000;
	if (level > 32767)
		level = 32767;
	else if (level < -32768)
		level = -32768;
	return (s32)level;
}

/* Linear power code (0.1uW) -> centi-dBm (0.01 dBm).
 *   cdBm = 1000*log10(code) - 4000 = (log2(code) * 1000*log10(2)) - 4000.
 * Same fixed-point log2 (fls integer part + Q16 linear-interp mantissa,
 * ~0.25 dB worst-case) as ddm_word_to_level -- exact enough for a diagnostic;
 * this is stock's pow2dbm1[] table computed instead of embedding 2 KB. Works
 * for TX power too (positive dBm). Floor -40.00 dBm for a dark/"n/a" read. */
s32 bosa_code_to_cdbm(u32 code)
{
	int e;
	u32 frac;
	s64 log2_q16, cdbm;

	if (code < 1)
		return -4000;
	e = fls(code) - 1;
	frac = e ? (((u64)(code - (1u << e)) << 16) >> e) : 0;
	log2_q16 = ((s64)e << 16) | frac;
	cdbm = ((log2_q16 * 301) >> 16) - 4000;		/* 301 = round(1000*log10(2)) */
	return cdbm < -4000 ? -4000 : (s32)cdbm;
}
