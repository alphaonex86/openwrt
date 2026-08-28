/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * rtl9602c_l34_logic.h -- logic hoisted out of rtl9602c_l34.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _RTL9602C_L34_LOGIC_H
#define _RTL9602C_L34_LOGIC_H

#include <linux/types.h>

void l34_field_set(u32 *w, unsigned int lsp, unsigned int width, u32 val);
u16 l34_hash_out(bool is_tcp, u32 sip, u16 sport, u32 dip, u16 dport);
u16 l34_hash_in(bool is_tcp, u32 dip, u16 dport);

#endif /* _RTL9602C_L34_LOGIC_H */
