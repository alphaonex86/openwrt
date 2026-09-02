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

/* Pack a MAC into the three L2FE FDB key words (aal __aal_mac_2_fdb_data;
 * vid/scind/dot1p = 0) - the append and the lookup-only path in
 * cortina-ni-rx.c share this so both hash to the same bucket. */
void cortina_ni_l2fe_fdb_key(const u8 *mac, u32 *d3, u32 *d2, u32 *d1);

/* THE BACKOFF LADDER for the decoupled LAN bring-up.  The recovery worker
 * runs at 1 Hz, so these are seconds: every second for the first half
 * minute, then one in 8, then one a minute - FOREVER.  There is
 * deliberately no attempt ceiling: a bank that becomes lockable at minute
 * 40 (a slow PHY, a cable inserted later, a cold-boot race) must still be
 * picked up, and the cost of asking once a minute is one idempotent
 * register walk.  The constants live HERE and not in the shell because the
 * shell's cadence-step-down warning reads FAST/MID_TICKS too. */
#define CA_NI_RX_BRINGUP_FAST_TICKS	30u	/* 1/s for the first 30 s */
#define CA_NI_RX_BRINGUP_MID_TICKS	300u	/* then 1/8 s out to 5 min */
#define CA_NI_RX_BRINGUP_MID_PERIOD	8u
#define CA_NI_RX_BRINGUP_SLOW_PERIOD	60u	/* then once a minute, forever */

unsigned int cortina_ni_rx_bringup_period(u64 ticks);
bool cortina_ni_rx_bringup_due(u64 ticks);

#endif /* _CORTINA_NI_RX_LOGIC_H */
