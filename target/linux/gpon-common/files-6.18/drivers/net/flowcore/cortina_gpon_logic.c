// SPDX-License-Identifier: GPL-2.0-only
/* See cortina_gpon_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>
#include <linux/errno.h>

#include "gpon_sn.h"

#include "cortina_gpon_logic.h"

/* One 32-bit register value from 4 wire-order bytes (endianness-agnostic). */
u32 cg_sn_word(const u8 *p)
{
	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

/*
 * ★★★ THE SERIAL-NUMBER CODEC MOVED TO THE COMMON CORE (2026-08-27), operator:
 * *"se deberia migrar ya todo los funciones del X400AXF a la familia ... el
 * resto no funciona y X400AXF fue muy verificado y funciona"*.
 *
 * The G.984.3 ONU-SN format is a SPEC, not a property of this silicon, so it
 * is decoded once in drivers/net/gpon/gpon_sn.c and both targets call it. The
 * implementation promoted there is THIS one -- it validated length, the vendor
 * characters and every hex digit, and refused a malformed string -- because
 * the Luna copy it replaces silently turned a bad digit into 0xff.
 *
 * These two remain as one-line shims ONLY so the ten call sites below and the
 * driver's -EINVAL contract are untouched by the move. Nothing else changed:
 * that is what makes this step verifiable on the board rather than argued.
 */
int cg_sn_parse(const char *s, u8 out[8])
{
	return gpon_sn_parse(s, out) ? -EINVAL : 0;
}

/*
 * The exact inverse of cg_sn_word() above: unpack a 32-bit register value
 * into its 4 wire-order ASCII bytes + NUL (the vendor-id register).  An
 * endianness-agnostic codec carries both directions side by side so a
 * reviewer can check round-tripping.  Moved verbatim from cg_read_vendor();
 * the shell keeps the one register read.
 */
void cg_vendor_unpack(u32 v, char out[5])
{
	out[0] = (v >> 24) & 0xff;
	out[1] = (v >> 16) & 0xff;
	out[2] = (v >> 8) & 0xff;
	out[3] = v & 0xff;
	out[4] = '\0';
}

/*
 * One PDC map entry's DATA words, moved verbatim from cg_pdc_init():
 * idx < omcc_gems (the OMCC-reserved internal GEMs; the count is silicon
 * GEOMETRY and stays an INPUT, the precedent omci_dgem_classify() set) ->
 * CPU port 0, forwarding-engine bypass, no-drop, cos 6, pol_id 0x80+idx
 * (the 128..255 PON-DS policer bank); else (data GEMs) -> L3_WAN,
 * pol_id idx-8 (refined per-GEM at the OMCI Create in Stage D).
 * The shell keeps the indirect kick+poll write and the PDC_CTRL RMW.
 */
void cg_pdc_map_entry(u32 idx, u32 omcc_gems, u32 *d0, u32 *d1)
{
	if (idx < omcc_gems) {
		*d0 = CG_PDC_D0_COS(6) | CG_PDC_D0_LDPID(CG_LPORT_CPU_0) |
		      CG_PDC_D0_LSPID(CG_LPORT_PON) |
		      CG_PDC_D0_FE_BYPASS | CG_PDC_D0_NO_DROP;
		*d1 = CG_PDC_D1_POL_ID(idx + 0x80);
	} else {
		*d0 = CG_PDC_D0_LDPID(CG_LPORT_L3_WAN) |
		      CG_PDC_D0_LSPID(CG_LPORT_PON);
		*d1 = CG_PDC_D1_POL_ID(idx - 8);
	}
}

/*
 * The PUC pvtbl entry's DATA0/1/2 words for one T-CONT, moved verbatim from
 * cg_puc_pvtbl_program().  queue_id = q + tcont*8, @ena gates bit 8 of each
 * 9-bit voqN field; the voqN fields are bit-split across the DATA words
 * exactly as the vendor packs them; schmode = 0 (strict priority),
 * entryvld = 1.  The shell keeps the five DATA writels, the indirect
 * kick+poll and the per-VoQ back-pressure/valid programming.
 */
void cg_puc_pvtbl_words(u32 tcont, bool ena, u32 *d0, u32 *d1, u32 *d2)
{
	u32 voq[CG_PUC_QUEUE_PER_TCONT];
	u32 q;

	for (q = 0; q < CG_PUC_QUEUE_PER_TCONT; q++)
		voq[q] = (q + tcont * CG_PUC_QUEUE_PER_TCONT) | ((u32)ena << 8);

	*d0 = voq[0] | (voq[1] << 9) | (voq[2] << 18) |
	      ((voq[3] & 0x1f) << 27);
	*d1 = ((voq[3] >> 5) & 0xf) | (voq[4] << 4) | (voq[5] << 13) |
	      (voq[6] << 22) | ((voq[7] & 1) << 31);
	*d2 = ((voq[7] >> 1) & 0xff) | BIT(12);	/* schmode=0, entryvld=1 */
}

/*
 * O5 exit = link down (vendor condition; the same G.984.3 rule Luna must
 * apply, so this is also a dedupe seed): leaving Operation for anything but
 * POPUP, leaving POPUP for anything but Operation/Ranging (POPUP->Ranging is
 * the Type-B popup, kept alive), or entering EmergencyStop.  Moved verbatim
 * from cg_isr_work(); the verdict IS the shell's datapath-reset trigger, so
 * the hoist harness swept ALL 65536 (last, state) pairs against the old
 * expression -- edge-for-edge, not shape-for-shape.
 */
bool cg_link_down_transition(u8 last, u8 state)
{
	return (last == CG_STATE_OPERATION && state != CG_STATE_OPERATION &&
		state != CG_STATE_POPUP) ||
	       (last == CG_STATE_POPUP && state != CG_STATE_OPERATION &&
		state != CG_STATE_RANGING) ||
	       state == CG_STATE_ESTOP;
}
