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
#include <linux/bits.h>

/*
 * Chip FACTS the hoisted logic computes over -- moved from cortina-gpon.c so
 * each is defined ONCE (the shell reads them back through this header, which
 * it already includes).  Field layouts and encodings only: register ADDRESSES
 * stay in the shell, per the hoist rule.
 */

/* onu.state encoding (vendor aal_gpon.h): 0=O1 Initial, 1=O2 Standby,
 * 2=O3 SerialNumber, 3=O4 Ranging, 4=O5 Operation, 5=O6 POPUP, 6=O7 EmrgStop.
 * The shell latches it from CG_REG_GPON_ONU; cg_link_down_transition()
 * classifies over it. */
#define CG_STATE_RANGING	3
#define CG_STATE_OPERATION	4
#define CG_STATE_POPUP		5
#define CG_STATE_ESTOP		6

/* PDC map DATA-word field layout (DATA0: cos[2:0], ldpid[8:3], lspid[14:9],
 * fe_bypass[15], no_drop[31]; DATA1: pol_id[12:4]) and the logical port ids
 * (vendor aal_port.h) cg_pdc_map_entry() routes between. */
#define CG_PDC_D1_POL_ID(x)	(((x) & 0x1ff) << 4)
#define CG_PDC_D0_COS(x)	((x) & 0x7)
#define CG_PDC_D0_LDPID(x)	(((x) & 0x3f) << 3)
#define CG_PDC_D0_LSPID(x)	(((x) & 0x3f) << 9)
#define CG_PDC_D0_FE_BYPASS	BIT(15)
#define CG_PDC_D0_NO_DROP	BIT(31)
#define CG_LPORT_CPU_0		0x10	/* CPU port 0 = the NI CPU-RX EPP port we drain */
#define CG_LPORT_L3_WAN		0x18
#define CG_LPORT_PON		0x07

/* 8Q mode -- the pvtbl entry layout packs exactly 8 9-bit voqN fields, so the
 * count is the ENTRY's own geometry, not a tunable.  cortina-gpon.c carries
 * an IDENTICAL define (gpon_gem_us_test extracts driver truth from there);
 * identical redefinition is silent C, drift warns at every shell build. */
#define CG_PUC_QUEUE_PER_TCONT	8

u32 cg_sn_word(const u8 *p);
int cg_sn_parse(const char *s, u8 out[8]);
void cg_vendor_unpack(u32 v, char out[5]);
void cg_pdc_map_entry(u32 idx, u32 omcc_gems, u32 *d0, u32 *d1);
void cg_puc_pvtbl_words(u32 tcont, bool ena, u32 *d0, u32 *d1, u32 *d2);
bool cg_link_down_transition(u8 last, u8 state);

#endif /* _CORTINA_GPON_LOGIC_H */
