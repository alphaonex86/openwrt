// SPDX-License-Identifier: GPL-2.0-only
/* TIER: FAMILY */
/* See luna_gpon_logic.h. */
#include <linux/types.h>
#include "gpon_ddm.h"	/* the one 0.1 uW -> centi-dBm conversion */
#include <linux/bitops.h>
#include <linux/limits.h>

#include "luna_gpon_logic.h"

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

/* Linear power code (0.1 uW) -> centi-dBm. A dark or "n/a" read floors at
 * -40.00 dBm rather than returning a sentinel: this family's reporting policy. */
s32 bosa_code_to_cdbm(u32 code)
{
	if (code < 1)
		return -4000;
	return gpon_ddm_uw10_to_cdbm(code);
}

/* pi_packed_locate/insert/extract live in flowcore_hash.c, declared through
 * flowcore.h: the Cortina flow engine needs them too and this object is gated
 * on CONFIG_LUNA_GPON, which that board never sets. */
#include <linux/string.h>

/* The test ORDER is load-bearing. An RTL8290B may ship an ...
 * dev/MEASURED-luna_gpon_logic.c.md sec 2. */
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
 * other byte -- a failed (negative) I2C read included -- rendered '.'. */
void bosa_sff_text(char *dst, const int *raw, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		dst[i] = (raw[i] >= 0x20 && raw[i] < 0x7f) ? (char)raw[i] : '.';
	dst[n] = 0;
}

/* Which of up to n samples is THE reading: insertion-sort ascending in place,
 * return the upper median. BOSA_RX_CODE_NA == 0 sorts to the bottom, so one
 * glitched or dark sample is discarded and only 2-of-3 NA makes the verdict NA.
 * n must be >= 1. */
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
