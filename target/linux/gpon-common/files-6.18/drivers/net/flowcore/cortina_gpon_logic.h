/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cortina_gpon_logic.h -- logic hoisted out of cortina-gpon.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _CORTINA_GPON_LOGIC_H
#define _CORTINA_GPON_LOGIC_H

#include <linux/types.h>

u32 cg_sn_word(const u8 *p);
int cg_sn_parse(const char *s, u8 out[8]);

#endif /* _CORTINA_GPON_LOGIC_H */
