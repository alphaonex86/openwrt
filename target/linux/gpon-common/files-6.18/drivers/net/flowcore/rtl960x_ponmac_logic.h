/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * rtl960x_ponmac_logic.h -- logic hoisted out of rtl960x_ponmac.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _RTL960X_PONMAC_LOGIC_H
#define _RTL960X_PONMAC_LOGIC_H

#include <linux/types.h>

bool c2_off_overconfig(u32 off);

#endif /* _RTL960X_PONMAC_LOGIC_H */
