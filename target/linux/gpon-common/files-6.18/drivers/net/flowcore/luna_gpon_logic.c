// SPDX-License-Identifier: GPL-2.0-only
/* See gpon_rtl9602c_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>
#include "gpon_ddm.h"	/* the one 0.1 uW -> centi-dBm conversion */
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
	s32 level;

	if (raw <= 0 || raw >= 0xffff)
		return INT_MIN;
	level = gpon_ddm_cdbm_to_anig(gpon_ddm_uw10_to_cdbm((u32)raw));
	return level;
}

/* Linear power code (0.1uW) -> centi-dBm (0.01 dBm).
 *   cdBm = 1000*log10(code) - 4000 = (log2(code) * 1000*log10(2)) - 4000.
 * Same fixed-point log2 (fls integer part + Q16 linear-interp mantissa,
 * ~0.25 dB worst-case) as ddm_word_to_level -- exact enough for a diagnostic;
 * this is stock's pow2dbm1[] table computed instead of embedding 2 KB. Works
 * for TX power too (positive dBm). Floor -40.00 dBm for a dark/"n/a" read. */
s32 bosa_code_to_cdbm(u32 code)
{
	/* A dark or "n/a" read floors at -40.00 dBm here rather than returning a
	 * sentinel -- that is this family's reporting policy, so it stays here and
	 * only the arithmetic is shared. */
	if (code < 1)
		return -4000;
	return gpon_ddm_uw10_to_cdbm(code);
}

/* ===== BOSA/DDM optical measurement logic (hoisted 2026-09-02) ==========
 * The shell samples the RTL8290B over I2C (channel select, latch strobe,
 * settle sleeps) and hands the raw values here; these functions decide what
 * the samples MEAN. div_u64/div64_u64 are pure math helpers (math64.h), needed
 * because MIPS32 has no libgcc 64-bit divide -- they are not a kernel service
 * in the hoist-rule sense (no device, no state, no time). */
#include <linux/math64.h>

#define BOSA_ADC_VREF_UV	3300000		/* stock 3.3V ADC full-scale (0x325aa0) */

/* Faithful RX power code (0.1uW) from the raw ratiometric ADC samples
 * (europa_drv.ko rtl8290b_rxPower_get + _rtl8290b_rx_power_cal). The shell
 * passes rssi = SD-ADC at the single in-range gain (0xC2) and the two on-die
 * reference taps (0x314 low anchor, 0x305 span endpoint). All 64-bit divides
 * go through the kernel helpers (no __divdi3 on MIPS32); /8192 and /4096 are
 * shifts.
 *
 * ★ A FLOOR CONSTANT IS NOT A MEASUREMENT.
 *
 * The three sentinel exits used to return 11, the same value the tail clamps a
 * genuine faint reading to -- and bosa_code_to_cdbm(11) is EXACTLY -2985,
 * so /proc/gpon published "rx=-29.85dBm" whenever the I2C bus was dead.
 * That constant was read as an optical measurement in three separate
 * project documents. It is now BOSA_RX_CODE_NA, which the printers render
 * as "n/a"; only the tail clamp still returns a real (floored) 11.
 *
 * tap_hi <= tap_lo is the direct dead-bus witness: a NACKing bus makes
 * the shell's bosa_read24() return 0 and a floating bus returns 0xffffff, so
 * both reference taps read alike. The old code papered over it with span = 1,
 * manufacturing a denominator out of a failed read.
 */
u32 bosa_rx_code_calc(u32 rssi, u32 tap_lo, u32 tap_hi,
		      const struct bosa_optical_cal *cal)
{
	u32 span;
	u64 v_uv, irssi, code;
	u32 s1, q;

	if (tap_hi <= tap_lo)			/* dead/floating I2C: taps read alike */
		return BOSA_RX_CODE_NA;
	span = tap_hi - tap_lo;
	if (rssi <= tap_lo)			/* below the low reference tap        */
		return BOSA_RX_CODE_NA;
	v_uv = div64_u64((u64)(rssi - tap_lo) * BOSA_ADC_VREF_UV, span);
	if (v_uv <= cal->rx_vthr)		/* at/below the dark level -> no light */
		return BOSA_RX_CODE_NA;
	irssi = div64_u64((u64)(v_uv - cal->rx_vthr) * 1000 * (cal->rx_r1 + cal->rx_r2),
			  (u64)cal->rx_r1 * cal->rx_r2);
	s1 = (irssi < 65536) ? 10 : 100;
	q  = (u32)div64_u64(irssi, s1);
	code = (((u64)cal->rx_poly_b * q) >> 13) * s1 + ((1000u * (u32)cal->rx_poly_c) >> 12);
	code = div64_u64(code, 100);
	return code < 11 ? 11 : (u32)code;
}

/* Module temperature in deci-degC from 14 raw (0x302, 0x303) register pairs.
 * Kelvin code assembly (a[7:0]<<1 | b[7], 233..383 K = -40..+110 C), per-sample
 * clamp (catches torn/glitch reads, e.g. -292 C), sort, drop 2 low + 2 high,
 * mean the middle 10, minus the per-board Kelvin trim (stock
 * rtl8290b_temperature_get; stock stores SFF-8472 1/256 C). A negative sample
 * is a failed I2C read: INT_MIN, the "no reading" sentinel -- the check is
 * repeated here (the shell already aborts its sampling loop) so this function
 * is total over any input a host fuzzer hands it. */
s32 bosa_temp_dc_calc(const int *a, const int *b, s16 temp_off)
{
	u16 s[14];
	u32 sum = 0;
	int i, j;

	for (i = 0; i < 14; i++) {
		u16 code;

		if (a[i] < 0 || b[i] < 0)
			return INT_MIN;
		code = ((u16)(a[i] & 0xff) << 1) | ((b[i] >> 7) & 1);
		if (code < 233) code = 233;
		if (code > 383) code = 383;
		s[i] = code;
	}
	for (i = 0; i < 13; i++)
		for (j = 0; j < 13 - i; j++)
			if (s[j + 1] < s[j]) { u16 t = s[j]; s[j] = s[j + 1]; s[j + 1] = t; }
	for (i = 2; i < 12; i++)
		sum += s[i];
	sum /= 10;
	return ((int)sum - temp_off - 273) * 10;
}

/* Laser bias current in micro-amps from the raw (0x321, 0x322) register pair.
 * 12-bit monitor code (h[7:0]<<4 | l[3:0]), full-scale ~100 mA at code 8192
 * (stock A2 word is 2 uA/LSB). A negative input is a failed read -> 0. */
u32 bosa_bias_ua_calc(int h, int l)
{
	u32 code12;

	if (h < 0 || l < 0)
		return 0;
	code12 = ((u32)(h & 0xff) << 4) | (u32)(l & 0x0f);
	return (u32)div_u64((u64)code12 * 100000, 8192) * 2;
}

/* One TX-power sample's contribution to the 10-sample accumulator
 * (europa_drv update_ddmi_tx_power chain): MPD-minus-dark voltage -> power
 * code c (the /1374 slope and +50 offset), bias CLASSIFICATION of iavg into 5
 * classes, range compensation shift, and the shifted result. vmpd and dark are
 * millivolts from the shell's SD-ADC ch2 sequence; iavg is 0x23A[7:0], range
 * is 0x246[7:6]. */
u32 bosa_tx_sample_contrib(s32 vmpd, s32 dark, int iavg, int range)
{
	s32 c = (((vmpd - dark) * 1000 / 1374) >> 4) + 50;
	int cls, shift;

	if (c < 0)
		c = 0;
	cls   = iavg < 64 ? 0 : iavg < 96 ? 1 : iavg < 128 ? 2 : iavg < 160 ? 3 : 4;
	shift = cls - (range == 1 ? 1 : range == 2 ? 2 : 0);
	return shift >= 0 ? (u32)c << shift : (u32)c >> -shift;
}

/* Fold the accumulated TX sample contributions into the per-board 0.1uW word:
 * word = (avg*slope*10)>>8 + (offset*10>>5). The caller feeds the word to
 * bosa_code_to_cdbm for centi-dBm. n must be >= 1 (the shell returns its
 * "no reading" sentinel when no sample survived). */
u32 bosa_tx_word_calc(u64 sum, int n, s32 tx_slope, s32 tx_offset)
{
	return (u32)((((u64)div_u64(sum, n) * tx_slope * 10) >> 8) +
		     ((tx_offset * 10) >> 5));
}

/* ===== packed pi-register array addressing ==============================
 * pi_packed_locate/insert/extract MOVED to flowcore_hash.c on 2026-09-02
 * (round 3): generic packed-slot math the Cortina flow engine needed too,
 * and this object is gated on CONFIG_RTL9602C_GPON, which that board never
 * sets -- the definitions had to live in an obj-y object or be copied.
 * Declarations (and the SID2QID history that explains why set and get share
 * ONE locate result) are in flowcore.h, which gpon_rtl9602c_logic.h now
 * includes, so every existing caller compiles unchanged. */

/* ===== round 2 (2026-09-02): module identity + sample selection =========
 * Same rule as everything above: the shell samples (I2C strobes, latches,
 * settle sleeps) and stores the flags/prints; these decide. */
#include <linux/string.h>

/*
 * ★★★ THE MODULE'S OWN NAME DECIDES -- IT WAS READ AND NOT USED.
 *
 * This test used to be `ident > 0 && ident < 0x30 && extid >= 0`, i.e. "an
 * SFF-8472 identity exists" -> "therefore NOT an RTL8290B", while the vendor
 * string was read, formatted into the warning, and never consulted. An
 * RTL8290 that ships an SFF-8472 EEPROM -- and this one does -- was
 * classified as a foreign module and had EVERY register write refused with
 * -ENODEV, the RX enable among them.
 *
 * ⚠ MEASURED 2026-08-30 on the LANLY G24W, tier 1, over the board's own I2C:
 * slave 0x50 bytes 20..35 read `REALTEK` and bytes 40..55 read `RTL8290`, in
 * plain ASCII, while slave 0x54 (the control bank) answered 0xff to every
 * address -- which is what a bank nobody may write looks like. The board sat
 * at O1 with sdet=0 on a fibre stock had reached Online through hours earlier.
 *
 * ★ THE STRING IS EVIDENCE, THE IDENT BYTE IS NOT. A vendor's identity page
 * says WHAT the module is; the presence of that page says only that it HAS
 * one. Deciding on the second while holding the first is the
 * text-vs-structure family this tree keeps paying for, inverted: the
 * structure was there and the weaker signal was believed.
 *
 * ★ THREE OUTCOMES ARE KEPT, and the middle one is still ours: named as ours
 * -> the path stays enabled; a plausible SFF-8024 identifier (0x01..0x2x)
 * alongside a foreign name -> POSITIVE identification of a different module,
 * refuse the writes, which is what this guard is FOR; unreadable -> "could
 * not tell", never "it is not one". The ORDER of the first two tests is
 * load-bearing: a module both named ours AND carrying a plausible ident byte
 * (the G24W's exactly) must classify as ours.
 */
enum bosa_module_verdict bosa_module_classify(int ident, int extid,
					      const char *vend, const char *part)
{
	if (strstr(vend, "REALTEK") || strstr(part, "RTL8290"))
		return BOSA_MODULE_NAMED_OURS;
	if (ident > 0 && ident < 0x30 && extid >= 0)
		return BOSA_MODULE_FOREIGN;
	return BOSA_MODULE_COULD_NOT_TELL;
}

/* One SFF-8472 string field, sanitized for the log: printable ASCII kept, any
 * other byte -- including a failed (negative) I2C read -- rendered '.', then
 * NUL-terminated. Was spelled inline three times in the shell (two probe
 * fields + the 9607C DDM scan). */
void bosa_sff_text(char *dst, const int *raw, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		dst[i] = (raw[i] >= 0x20 && raw[i] < 0x7f) ? (char)raw[i] : '.';
	dst[n] = 0;
}

/* Which of up to n samples is THE reading: insertion-sort ascending in place,
 * return the upper median v[n/2] (n=1 -> the sample, n=2 -> the higher, n=3
 * -> the middle). Shared by the DDM 16-bit word read (median of the samples
 * that did not NACK) and the RX code chain (BOSA_RX_CODE_NA == 0 sorts to the
 * bottom, so a single glitched/dark sample is discarded and only 2-of-3 NA
 * makes the verdict NA -- equal to the old sum-minus-extremes median-of-3 for
 * every u32 triple, wraparound included). n must be >= 1; the shell keeps its
 * "every sample failed" sentinel. */
u32 bosa_median_u32(u32 *v, unsigned int n)
{
	unsigned int i, j;

	for (i = 1; i < n; i++) {
		u32 key = v[i];

		for (j = i; j > 0 && v[j - 1] > key; j--)
			v[j] = v[j - 1];
		v[j] = key;
	}
	return v[n / 2];
}

/* Whether an MPD ADC sample constitutes a measurement, and its ratiometric mV
 * value if so (europa_drv TX-power chain; the vmpd input of
 * bosa_tx_sample_contrib). hi == 0 or code at/below the zero tap is the same
 * taps-agree dead-bus shape bosa_rx_code_calc owns -> INT_MIN, "no reading";
 * else mV = (hi - zero) * 1200 / (code - zero), 64-bit intermediate. Moved
 * verbatim from the shell's bosa_vmpd_mv tail. */
s32 bosa_vmpd_mv_calc(u32 code, s32 hi, s32 zero)
{
	if (hi == 0 || (s32)code <= zero)
		return INT_MIN;
	return (s32)div_u64((u64)(u32)(hi - zero) * 1200, (u32)((s32)code - zero));
}
