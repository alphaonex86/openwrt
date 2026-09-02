// SPDX-License-Identifier: GPL-2.0-only
/* See cortina_ni_rx_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>

#include "cortina_ni_rx_logic.h"

/* extract an LSB-first bitfield (width <= 32) from the 32 LE HDR_I words */
u32 rx_hdri_get(const u32 *w, unsigned int bit, unsigned int width)
{
	u64 v = ((u64)w[(bit >> 5) + 1] << 32) | w[bit >> 5];

	v >>= bit & 31;
	return v & (width < 32 ? (1u << width) - 1 : 0xffffffffu);
}

/* Pack a MAC into the FDB key words (aal __aal_mac_2_fdb_data; vid/scind/dot1p
 * = 0) - shared by the append and the lookup-only path in cortina-ni-rx.c so
 * both hash to the same bucket. */
void cortina_ni_l2fe_fdb_key(const u8 *mac, u32 *d3, u32 *d2, u32 *d1)
{
	*d3 = (mac[0] >> 5) & 0x7;
	*d2 = ((u32)(mac[0] & 0x1f) << 27) | ((u32)mac[1] << 19) |
	      ((u32)mac[2] << 11) | ((u32)mac[3] << 3) | ((mac[4] >> 5) & 0x7);
	*d1 = (u32)(((mac[4] & 0x1f) << 8) | mac[5]) << 19;
}

/*
 * Declared deviation from moved-verbatim, both functions below: the shell
 * passed its device state (`const struct cortina_ni_rx *rx`) and read
 * rx->bringup_ticks; that struct is the shell's and cannot cross the tier,
 * so the tick count itself is the argument.  It stays u64 like the field it
 * mirrors - `unsigned int` would change the ladder at the 2^32 wrap.  The
 * bodies are otherwise verbatim.
 */
unsigned int cortina_ni_rx_bringup_period(u64 ticks)
{
	if (ticks <= CA_NI_RX_BRINGUP_FAST_TICKS)
		return 1u;
	if (ticks <= CA_NI_RX_BRINGUP_MID_TICKS)
		return CA_NI_RX_BRINGUP_MID_PERIOD;
	return CA_NI_RX_BRINGUP_SLOW_PERIOD;
}

bool cortina_ni_rx_bringup_due(u64 ticks)
{
	unsigned int period = cortina_ni_rx_bringup_period(ticks);

	/* `bringup_ticks` has already been incremented for THIS tick, so tick
	 * 1 is due (1 % 1 == 0) and the ladder never leaves a silent gap at a
	 * phase boundary. */
	return (ticks % period) == 0u;
}

/* ------------------------------------------------------------------ */
/* Round two (2026-09-02).  Chip facts below are the RTL9607F's, from  */
/* the same tier-1/tier-2 evidence the shell's register header cites;  */
/* the cortina-ni-regs.h spellings whose only code uses moved here are */
/* retired (dead defines, listed for deletion).                        */
/* ------------------------------------------------------------------ */

/* HEADER_A lspids that select a WAN delivery (chip facts; the same values
 * are still live in cortina-ni-regs.h for the shell's diag taps, which is
 * why the two shared ones are re-spelled token-identically in the header
 * rather than privately here). */

enum ca_ni_rx_wan_class cortina_ni_rx_wan_class(u32 lspid)
{
	if (lspid == CA_NI_LSPID_PON)
		return CA_NI_RX_WAN_PON;
	if (lspid == CA_NI_LSPID_L3_WAN)
		return CA_NI_RX_WAN_L3;
	return CA_NI_RX_WAN_NONE;
}

/* The L2FE FDB action word (DATA0 of an APPEND) and CMD_RETURN layout.
 * ex-cortina-ni-regs.h spellings, single home now: action[5:0] = forward-to
 * ldpid; VALID bit 9; STATIC (no-age) bit 19; DA_PERMIT bit 20 (mandatory or
 * a DA hit won't forward); SA_PERMIT bit 21.  CMD_RETURN: status[3:0], 0x5 =
 * HIT; ext_status[16:4] = the 13-bit hash-table entry index (== the L3FE
 * forward action's mac_da_idx / aal-77c egr_lutidx). */
#define CA_NI_L2FE_FDB_LPID_MASK	0x3fu
#define CA_NI_L2FE_FDB_VALID		(1u << 9)
#define CA_NI_L2FE_FDB_STATIC		(1u << 19)
#define CA_NI_L2FE_FDB_DA_PERMIT	(1u << 20)
#define CA_NI_L2FE_FDB_SA_PERMIT	(1u << 21)
#define CA_NI_L2FE_FDB_STATUS_HIT	0x5

u32 cortina_ni_l2fe_fdb_action(u32 ldpid)
{
	return (ldpid & CA_NI_L2FE_FDB_LPID_MASK) |
	       CA_NI_L2FE_FDB_VALID | CA_NI_L2FE_FDB_STATIC |
	       CA_NI_L2FE_FDB_DA_PERMIT | CA_NI_L2FE_FDB_SA_PERMIT;
}

/* CMD_RETURN -> the entry index, or -1 when the status nibble is not HIT. */
int cortina_ni_l2fe_fdb_cmd_status_idx(u32 cmd_return)
{
	if ((cmd_return & 0xf) != CA_NI_L2FE_FDB_STATUS_HIT)
		return -1;
	return (int)((cmd_return >> 4) & 0x1fff);	/* ext_status[16:4] */
}

/* Is a stored action forwardable as a DA?  Only with VALID + DA_PERMIT;
 * @ldpid (optional) = the action's forward-to ldpid, written only on yes. */
bool cortina_ni_l2fe_fdb_action_da(u32 action, u32 *ldpid)
{
	if (!(action & CA_NI_L2FE_FDB_VALID) ||
	    !(action & CA_NI_L2FE_FDB_DA_PERMIT))
		return false;
	if (ldpid)
		*ldpid = action & CA_NI_L2FE_FDB_LPID_MASK;
	return true;
}

/* ------------------------------------------------------------------ */
/* The per-lport L2FE profile.  Stock init state (validated against    */
/* live-stock leftovers of the last-touched entries); with the tables  */
/* unprogrammed the pipeline forces every ingress frame to the         */
/* blackhole ldpid BEFORE the FDB/DFT_FWD decision, which is why every */
/* correctly-written forwarding table used to read back fine and do    */
/* nothing.                                                            */
/* ------------------------------------------------------------------ */

/* The lport space (ex-cortina-ni-regs.h; their only code uses live here
 * now): eth NI0-6 = 0-6, CPU_0-7 = 0x10-0x17, L3_LAN = 0x19, MC = 0x1b,
 * GEM/LLID + CPU-MQ = 0x20-0x3f. */
#define CA_NI_LPORT_ETH_NI6		0x06
#define CA_NI_LPORT_CPU_0		0x10
#define CA_NI_LPORT_CPU_7		0x17
#define CA_NI_LPORT_L3_LAN		0x19
#define CA_NI_LPORT_MC			0x1b
#define CA_NI_LPORT_GEM_FIRST		0x20

/* ILPB/ELPB profile words (stock init literals; ILPB d2 carries stp_mode =
 * FWD+LEARN, the d2 variant is the port CLASS; ELPB d0 is egress STP
 * forward + vlan-aware, the WAN variant adds dest_wan). */
#define CA_NI_L2FE_ILPB_D4_WAN		0x00000400u
#define CA_NI_L2FE_ILPB_D3_INIT		0x00100003u
#define CA_NI_L2FE_ILPB_D2_PORT		0x18022163u	/* LAN/CPU/L3/other <0x20 */
#define CA_NI_L2FE_ILPB_D2_MC		0x1803fd7fu	/* MC port 0x1b */
#define CA_NI_L2FE_ILPB_D2_GEM		0x180222a3u	/* GEM/LLID >=0x20 (S-TPID) */
#define CA_NI_L2FE_ILPB_D1_INIT		0x000001cbu
#define CA_NI_L2FE_ILPB_D1_STAMOVE	0x80000000u	/* regs.h spelled it BIT(31) */
#define CA_NI_L2FE_ILPB_D0_INIT		0xc1000000u
#define CA_NI_L2FE_ELPB_D0_LAN		0x00000003u
#define CA_NI_L2FE_ELPB_D0_WAN		0x0000000bu

/* Stock L2FE VLAN membership check-id map (aal_l2_vlan.c __g_l2_vlan_port_map):
 * lport -> membership check-id.  NI0-7 -> 0-7, CPU_0 -> 8, CPU_1 -> 9,
 * L3_WAN/L3_LAN (0x18/0x19) -> 15, GEM (0x20+) -> 7.  Programmed into the
 * MMSHP_CHK_ID_MAP so the membership check qualifies LAN<->CPU forwarding. */
static const u8 ca_ni_vlan_chkid_map[CA_NI_L2FE_LPORT_COUNT] = {
	 0, 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0,
	 8, 9, 0,10,11,12,13,14,15,15, 0, 0, 0, 0, 0, 0,
	 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
};

/*
 * Declared deviation from moved-verbatim: the shell derived these values
 * inline between its writel()s; here the derivation is gathered into one
 * pure function so a host test can enumerate all 64 lports and byte-compare
 * against what the shell writes.  The VALUES and the branch structure are
 * verbatim.  The shell's `~BIT(i - 32)` was unsigned long on arm64 and was
 * truncated to 32 bits by writel(); `~(1u << ...)` is that same truncated
 * value computed in u32.
 */
void cortina_ni_rx_lport_profile(unsigned int lport,
				 struct ca_ni_lport_profile *p)
{
	/* ILPB ingress profile: stp_mode=FWD+LEARN + the stock defaults */
	p->ilpb_d4 = lport >= CA_NI_LPORT_GEM_FIRST ? CA_NI_L2FE_ILPB_D4_WAN : 0;
	p->ilpb_d3 = CA_NI_L2FE_ILPB_D3_INIT;
	if (lport == CA_NI_LPORT_MC)
		p->ilpb_d2 = CA_NI_L2FE_ILPB_D2_MC;
	else if (lport < CA_NI_LPORT_GEM_FIRST)
		p->ilpb_d2 = CA_NI_L2FE_ILPB_D2_PORT;
	else
		p->ilpb_d2 = CA_NI_L2FE_ILPB_D2_GEM;
	p->ilpb_d1 = CA_NI_L2FE_ILPB_D1_INIT;
	if (lport <= CA_NI_LPORT_ETH_NI6 ||
	    (lport >= CA_NI_LPORT_CPU_0 && lport <= CA_NI_LPORT_CPU_7))
		p->ilpb_d1 |= CA_NI_L2FE_ILPB_D1_STAMOVE;
	p->ilpb_d0 = CA_NI_L2FE_ILPB_D0_INIT;

	/* MMSHP: allowed-ldpid bitmap = all-but-self (isolation off) */
	p->mmshp_d1 = lport >= 32 ? ~(1u << (lport - 32)) : ~0u;
	p->mmshp_d0 = lport < 32 ? ~(1u << lport) : ~0u;

	/* ELPB egress profile (+dest_wan on the PON-side dest ports and the
	 * L3_LAN dest) */
	p->elpb_d0 = (lport >= CA_NI_LPORT_GEM_FIRST ||
		      lport == CA_NI_LPORT_L3_LAN) ?
		     CA_NI_L2FE_ELPB_D0_WAN : CA_NI_L2FE_ELPB_D0_LAN;

	p->chkid = ca_ni_vlan_chkid_map[lport];
}
