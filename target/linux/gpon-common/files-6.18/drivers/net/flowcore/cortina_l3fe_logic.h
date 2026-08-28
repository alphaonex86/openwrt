/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cortina_l3fe_logic.h -- logic hoisted out of cortina-l3fe.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _CORTINA_L3FE_LOGIC_H
#define _CORTINA_L3FE_LOGIC_H

#include <linux/types.h>

void l3fe_wan_mac_derive(const u8 *lan_mac, u8 *wan_mac);

#endif /* _CORTINA_L3FE_LOGIC_H */
