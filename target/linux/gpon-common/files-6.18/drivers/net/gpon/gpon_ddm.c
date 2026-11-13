// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CORE */
/* gpon_ddm.c -- optical diagnostic (DDM) unit conversion. ONE ...
 * dev/MEASURED-gpon_ddm.c.md sec 1. */

#include <linux/kernel.h>

#include "gpon_ddm.h"

/* log10 of the mantissa, in milli-decades, sampled every 1/16 ...
 * dev/MEASURED-gpon_ddm.c.md sec 2. */
static const u16 MANT_LOG10[17] = {
	  0,  26,  51,  75,  97, 118, 138, 158, 176,
	194, 211, 227, 243, 258, 273, 287, 301,
};

s32 gpon_ddm_uw10_to_cdbm(u32 raw)
{
	u32 mantissa, index, weight;
	int octave = 0;
	s32 milli;

	if (!raw)
		return 0;	/* undefined input; the caller owns the zero */

	/* log10(raw) = octave*log10(2) + log10(mantissa), 1 <= mantissa < 2 */
	while (raw >> (octave + 1))
		octave++;

	/* mantissa - 1, as a fraction of 256 */
	mantissa = octave ? (((raw - (1u << octave)) << 8) >> octave) : 0;
	index  = mantissa >> 4;		/* which of the 16 intervals   */
	weight = mantissa & 0xf;	/* how far into it, out of 16  */

	/* 30103 = round(100000 * log10(2)); the +50 rounds the /100 */
	milli = (octave * 30103 + 50) / 100;

	/* linear interpolation between the two bracketing table entries */
	milli += MANT_LOG10[index] +
		 (s32)(MANT_LOG10[index + 1] - MANT_LOG10[index]) * (s32)weight / 16;

	return milli - 4000;		/* 0.1 uW = -40.00 dBm is the origin */
}

/* ANI-G counts per centi-dBm: the attribute's step is 0.002 dB, ours is 0.01. */
#define GPON_ANIG_PER_CDBM	5

s16 gpon_ddm_cdbm_to_anig(s32 cdbm)
{
	s32 level = cdbm * GPON_ANIG_PER_CDBM;

	if (level > 32767)
		return 32767;
	if (level < -32768)
		return -32768;
	return (s16)level;
}

static int gpon_ddm_fmt_cdbm(char *out, size_t sz, u16 uw10)
{
	return uw10 ? scnprintf(out, sz, "%d", gpon_ddm_uw10_to_cdbm(uw10)) : scnprintf(out, sz, "-inf");
}

int gpon_ddm_a2h_fmt(const struct gpon_ddm_a2h *d, char *out, size_t sz)
{
	unsigned int i;
	int pos;

	if (d->status != GPON_DDM_OK)
		return scnprintf(out, sz, "optic_ddm      = %s\n", gpon_ddm_status_str(d->status));
	pos = scnprintf(out, sz, "optic_ddm      = live (SFF-8472 A2h 0x%02x-0x%02x)\n", GPON_DDM_A2H_BASE,
			GPON_DDM_A2H_BASE + GPON_DDM_A2H_LEN - 1);
	pos += scnprintf(out + pos, sz - pos, "optic_rx_raw: 0x%04x optic_rx_cdbm: ", d->rx_pwr);
	pos += gpon_ddm_fmt_cdbm(out + pos, sz - pos, d->rx_pwr);
	pos += scnprintf(out + pos, sz - pos, " optic_tx_raw: 0x%04x\noptic_env:   temp_dc=%d bias_ua=%u tx_cdbm=",
			 d->tx_pwr, gpon_ddm_temp_dc(d->temp), gpon_ddm_bias_ua(d->bias));
	pos += gpon_ddm_fmt_cdbm(out + pos, sz - pos, d->tx_pwr);
	pos += scnprintf(out + pos, sz - pos, " vcc_mv=%u\noptic_ddm_raw: %02x..%02x =", gpon_ddm_vcc_mv(d->vcc),
			 GPON_DDM_A2H_BASE, GPON_DDM_A2H_BASE + GPON_DDM_A2H_LEN - 1);
	for (i = 0; i < GPON_DDM_A2H_LEN; i++)
		pos += scnprintf(out + pos, sz - pos, " %02x", d->raw[i]);
	return pos + scnprintf(out + pos, sz - pos, "\n");
}
