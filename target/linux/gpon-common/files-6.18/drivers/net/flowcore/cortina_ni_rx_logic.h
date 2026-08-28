/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cortina_ni_rx_logic.h -- logic hoisted out of cortina-ni-rx.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _CORTINA_NI_RX_LOGIC_H
#define _CORTINA_NI_RX_LOGIC_H

#include <linux/types.h>

u32 rx_hdri_get(const u32 *w, unsigned int bit, unsigned int width);

#endif /* _CORTINA_NI_RX_LOGIC_H */
