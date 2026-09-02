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

/* ------------------------------------------------------------------ */
/* Round two (2026-09-02): the L2FE bring-up's DECISIONS move here --  */
/* the per-lport profile derivation, the FDB action/status codec and   */
/* the lspid WAN classifier.  The shell keeps the table PROTOCOL: the  */
/* DATA-register order, the GO kicks and the polls.                    */
/* ------------------------------------------------------------------ */

/*
 * ★ DELIBERATE IDENTICAL REDEFINITION.  The three constants below spell the
 * same values as cortina-ni-regs.h, token for token.  The logic functions
 * decide WITH them and may not read the shell's register header; the shell
 * still includes both headers, so if either side ever moves, every build of
 * the shell warns on the redefinition (a -Werror build refuses it) instead
 * of the two homes drifting silently.  The regs.h copies whose only code
 * uses moved here with round two are dead and listed for deletion in the
 * coordinator report.
 */
#define CA_NI_LSPID_PON			0x07
#define CA_NI_LSPID_L3_WAN		0x18
#define CA_NI_L2FE_LPORT_COUNT		64

/*
 * Which WAN delivery class a HEADER_A lspid selects.  Both WAN classes are
 * WAN-only lspids, so neither can steal a LAN frame; the counter attribution
 * (wan_frames vs wan_l3_frames) hangs off the same answer.
 *
 * CA_NI_RX_WAN_L3 names the lspid CLASS only: whether the HW-L3 path is
 * actually armed is a LIVE read of the offload module's state, so the SHELL
 * applies that gate.  Taking the gate as an argument here would make the
 * caller evaluate it for every frame, putting an exported-function call on
 * the hot path for LAN frames that classify to NONE anyway.
 */
enum ca_ni_rx_wan_class {
	CA_NI_RX_WAN_NONE = 0,	/* not a WAN lspid: fall through to eth0 */
	CA_NI_RX_WAN_PON,	/* de-encapsulated data-GEM frame, lspid PON */
	CA_NI_RX_WAN_L3,	/* L3-WAN lspid: L3FE punt, IF the HW-L3 path is armed */
};

enum ca_ni_rx_wan_class cortina_ni_rx_wan_class(u32 lspid);

/*
 * L2FE FDB action/status codec.  The engine protocol (DATA3..DATA0 order,
 * the GO|opcode kick, the GO poll) stays in the shell; what is here is what
 * the words MEAN:
 *   - the action word a static {mac -> ldpid} entry carries;
 *   - the CMD_RETURN decode (status nibble, 13-bit entry index);
 *   - the "forwardable as a DA" validity gate over a stored action.
 * cortina-l3fe.c still spells the action word (and the key pack) a second
 * time in l3fe_fdb_static_add() -- rewiring it is one call-site change in
 * that shell, left to the coordinator (out of this round's file scope).
 */
u32 cortina_ni_l2fe_fdb_action(u32 ldpid);
int cortina_ni_l2fe_fdb_cmd_status_idx(u32 cmd_return);
bool cortina_ni_l2fe_fdb_action_da(u32 action, u32 *ldpid);

/*
 * One lport's L2FE profile: every VALUE the per-port bring-up loop writes,
 * derived from the lport number alone.  @lport must be below
 * CA_NI_L2FE_LPORT_COUNT -- same contract as the table walk that consumes
 * this.  A host test can enumerate all 64 and byte-compare.
 */
struct ca_ni_lport_profile {
	u32 ilpb_d4;		/* WAN flag (GEM range) */
	u32 ilpb_d3;
	u32 ilpb_d2;		/* class: MC vs physical-port vs GEM */
	u32 ilpb_d1;		/* + STAMOVE on eth NI0-6 and CPU_0-7 */
	u32 ilpb_d0;
	u32 mmshp_d1;		/* all-but-self isolation bitmap, hi word */
	u32 mmshp_d0;		/* lo word */
	u32 elpb_d0;		/* egress WAN/LAN choice */
	u8 chkid;		/* VLAN membership check-id (stock table) */
};

void cortina_ni_rx_lport_profile(unsigned int lport,
				 struct ca_ni_lport_profile *p);

#endif /* _CORTINA_NI_RX_LOGIC_H */
