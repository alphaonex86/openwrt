/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cortina_ni_flowoffload_logic.h -- logic hoisted out of cortina-ni-flowoffload.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _CORTINA_NI_FLOWOFFLOAD_LOGIC_H
#define _CORTINA_NI_FLOWOFFLOAD_LOGIC_H

#include <linux/types.h>

void cn_l3e_bitrev_key(u32 *w, int n_words);
u32 cn_l3e_crc32(const u8 *p, size_t len);
u16 cn_l3e_crc16(const u8 *p, size_t len);
u32 cn_l3e_proc_parse_ip(const char *s);

#endif /* _CORTINA_NI_FLOWOFFLOAD_LOGIC_H */
