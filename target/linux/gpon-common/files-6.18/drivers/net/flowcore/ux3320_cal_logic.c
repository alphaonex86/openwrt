// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CORE */
/* UX3320 per-unit calibration as ordered ops for the gn25l95_cal_apply()
 * executor. Its order and tables were read from the RTL9607C stock loader;
 * this unit's own 1024-byte file holds the values. */
#include <linux/errno.h>
#include <linux/kernel.h>
#include "gn25l95_cal_logic.h"

#define UX_SLAVE_A0	0x50
#define UX_SLAVE_A2	0x51
#define UX_REG_TABLE	0x7f
#define UX_TABLE_3	3
#define UX_REG_EEPROM	0xf0
#define UX_REG_CTRL	0x6e		/* bit 6: soft TX disable, pulsed */
#define UX_REG_T3_9E	0x9e
#define UX_FILE_T3	0x180		/* table 3 register r is file byte 0x180 + r */

/* Table 3 bias, modulation and LOS level: 0 at power-on without an EEPROM
 * (measured, RTL9607C unit 2, 2026-11-04), else this unit's file values. */
static const u8 UX_SIG_REG[3] = { 0x8c, 0x87, 0x9d };

int ux3320_probe_ops(struct gn_op *out, u32 out_max)
{
	u32 n = 0, i;

	if (!out)
		return -EINVAL;
	GN_EMIT(GN_SELECT, UX_SLAVE_A2, UX_REG_TABLE, UX_TABLE_3, 0);
	for (i = 0; i < ARRAY_SIZE(UX_SIG_REG); i++)
		GN_EMIT(GN_READ_NOTE, UX_SLAVE_A2, UX_SIG_REG[i], 0, 0);
	GN_EMIT(GN_READ_NOTE, UX_SLAVE_A2, UX_REG_EEPROM, 0, 0);
	return (int)n;
}

bool ux3320_identified(const u8 notes[UX_PROBE_NOTES], const u8 *cal, u32 cal_len)
{
	bool cold = true, ours = cal && cal_len == GN_CAL_LEN;
	u32 i;

	if (notes[3] != 0xff)		/* an EEPROM drives it: not ours to calibrate */
		return false;
	for (i = 0; i < ARRAY_SIZE(UX_SIG_REG); i++) {
		cold &= !notes[i];
		ours = ours && notes[i] == cal[UX_FILE_T3 + UX_SIG_REG[i]];
	}
	return cold || ours;
}

int ux3320_cal_ops(const u8 *cal, u32 cal_len, bool t3_9e_set,
		   struct gn_op *out, u32 out_max)
{
	/* Each table's upper window: register r takes cal[base + r]. */
	static const struct { u8 table; u16 base; u8 last; } TABLES[] = {
		{ 0, 0x100, 0xe8 }, { 4, 0x200, 0xe8 }, { 5, 0x280, 0xe8 },
		{ 6, 0x300, 0xe8 }, { UX_TABLE_3, UX_FILE_T3, 0xdc },
	};
	u32 n = 0, r, t;

	if (!cal || !out || cal_len != GN_CAL_LEN)
		return -EINVAL;
	for (r = 0; r <= 0xe8; r++)
		GN_EMIT(GN_WRITE, UX_SLAVE_A0, r, cal[r], 0);
	for (r = 0; r <= 0x7a; r++)
		GN_EMIT(GN_WRITE, UX_SLAVE_A2, r, cal[0x100 + r], 0);
	for (t = 0; t < ARRAY_SIZE(TABLES); t++) {
		GN_EMIT(GN_SELECT, UX_SLAVE_A2, UX_REG_TABLE, TABLES[t].table, 0);
		for (r = 0x80; r <= TABLES[t].last; r++)
			GN_EMIT(GN_WRITE, UX_SLAVE_A2, r, cal[TABLES[t].base + r], 0);
	}
	GN_EMIT(GN_PULSE, UX_SLAVE_A2, UX_REG_CTRL, 0x40, 0xbf);
	for (r = 0x7b; r <= 0x7e; r++)
		GN_EMIT(GN_WRITE, UX_SLAVE_A2, r, 0xff, 0);
	GN_EMIT(GN_SELECT, UX_SLAVE_A2, UX_REG_TABLE, UX_TABLE_3, 0);
	GN_EMIT(GN_RMW, UX_SLAVE_A2, UX_REG_T3_9E, t3_9e_set ? 0x20 : 0x00, 0x20);
	return (int)n;
}
