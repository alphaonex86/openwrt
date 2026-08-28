/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_rtl9602c_logic.h -- logic hoisted out of gpon-rtl9602c.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _GPON_RTL9602C_LOGIC_H
#define _GPON_RTL9602C_LOGIC_H

#include <linux/types.h>

u8 bosa_slave_for(u16 reg);
s32 ddm_word_to_level(int raw);
s32 bosa_code_to_cdbm(u32 code);

#endif /* _GPON_RTL9602C_LOGIC_H */
