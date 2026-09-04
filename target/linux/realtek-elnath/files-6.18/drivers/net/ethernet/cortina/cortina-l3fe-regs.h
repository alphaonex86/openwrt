/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Cortina NE-window L3FE registers that MORE THAN ONE source file needs.
 *
 * Everything here was written down at least twice before 2026-09-04:
 * cortina-l3fe.c spelled the block as L3FE_*, cortina-ni-flowoffload.c as
 * CN_L3E_HS_*, and cortina-l3fe-aging.c as CL3A_*.  Sixteen addresses carried
 * two or three names, which is not merely untidy -- the COMMENTS had diverged,
 * and each copy knew something the others did not.  The cache-status word is
 * the clearest case: one copy documented bit0 busy and bit4 match, the other
 * documented bit3 as benign and bit6 as evicted, and no single reader of either
 * file could have known all four.  The comments below are the UNION, so the
 * knowledge stops depending on which file you happened to open.
 *
 * SCOPE, deliberately narrow: this header holds what is SHARED, not the whole
 * L3FE map.  A register only one file uses stays in that file, where its
 * neighbours and its usage are.  Widening this to the full block would mean
 * renaming ~40 more identifiers for no reader's benefit.
 *
 * ⇒ THE PREFIX ENCODES THE SCOPE, and the two spellings coexist on purpose: a
 * register named L3FE_* is SHARED and lives here, one still named CN_L3E_* or
 * CL3A_* is used by exactly one file and stays there.  Do not "tidy" a local
 * name into L3FE_* without moving it here -- two files defining the same
 * L3FE_ name at different offsets is precisely the silent divergence this
 * header exists to end.
 *
 * Offsets are NE-window relative (NE = CA_NI_WIN_NI = 0x4_f4300000), the same
 * convention as cortina-ni-regs.h.  Vendor names are from the NAME->ADDRESS
 * table shipped in the stock rootfs (tier 2) and are quoted where they add
 * something our name does not already say.
 */
#ifndef _CORTINA_L3FE_REGS_H
#define _CORTINA_L3FE_REGS_H

/* ---- hash engine: table geometry and the four table base addresses ------- */
/* hb_size[1:0], ht_size[4:2], ha_width[7:5], def_reg[16], crc_ntfy_en[17] */
#define L3FE_HS_HASH_INI		0x3834
#define L3FE_HS_BA_MH0			0x383c	/* hash-key table base, phys[31:7] in place */
#define L3FE_HS_BA_MA0			0x3844	/* main action FIB base, phys[31:7] in place */
#define L3FE_HS_MEM_INI			0x393c	/* bit0 req_sts: engine SRAM/table self-init */

/* ---- hash engine: the cache, and its indirect control/status pair -------- */
#define L3FE_HS_CACHE_INI		0x38a0
/* slot[4:0], crc16[20:5], loc[24], age[27:26], pri[29:28], cmd[31:30] */
#define L3FE_HS_CACHE_CTRL		0x38ac
#define L3FE_HS_CACHE_CTRL_REQ		0x38b0	/* bit0 req_sts = GO / busy */
/* bit0 bsy, bit1 err_hash, bit2 err_free, bit3 err_nch (BENIGN), bit4 match,
 * bit6 evicted.  The two halves of this list came from two different files. */
#define L3FE_HS_CACHE_CTRL_STS		0x38b4

/* ---- hash engine: the HW-CRC selftest engine (debug) --------------------- */
#define L3FE_HS_SWO_IDX			0x38d8	/* pointer; DAT auto-increments it */
#define L3FE_HS_SWO_DAT			0x38dc
#define L3FE_HS_SWO_CTRL		0x38e0	/* bit0 = GO / busy */

/* ---- hash engine: the mask table ---------------------------------------- */
/* idx | bit30 wr | bit31 GO | bit6 selects the upper-128 beat */
#define L3FE_HS_MASK_ACCESS		0x3910

/* ---- hash engine: aging ------------------------------------------------- */
/* timer[29:0] = age_time_s * core_clk / 0x2000; 0 = HW auto-countdown OFF */
#define L3FE_HS_AGING_GRANULARITY	0x3924
/* bucket/address[11:0] (bit11 = the overflow age bank) | bit30 wr | bit31 GO */
#define L3FE_HS_AGE_ACCESS		0x3928
#define L3FE_HS_AGE_DATA_LO		0x3938	/* slots  0..15, 2 bits of age each */
#define L3FE_HS_AGE_DATA_HI		0x3934	/* slots 16..31, 2 bits of age each */

#endif /* _CORTINA_L3FE_REGS_H */
