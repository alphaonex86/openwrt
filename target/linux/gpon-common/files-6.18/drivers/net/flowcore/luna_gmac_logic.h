/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * luna_gmac_logic.h -- pure logic of the Luna FAMILY GMAC engine, shared by
 * the family's TWO Ethernet shells:
 *
 *	rtl9602c_eth.c  (CONFIG_RTL9602C_ETH, taroko / X111W)
 *	luna_eth.c      (CONFIG_LUNA_ETH, interaptiv / RTL9607C + RTL9603CVD)
 *
 * WHY A FILE OF ITS OWN AND NOT rtl9602c_l34_logic.{c,h}: the two shells are
 * gated on DISJOINT config symbols -- measured 2026-09-02 in this tree's own
 * configs: realtek-luna/config-6.18 sets RTL9602C_ETH=y with LUNA_ETH unset,
 * interaptiv/config-default the exact reverse -- so logic in a TU gated on one
 * shell's symbol links green on one board and dies `undefined reference` on
 * the other (the measured defect crossconfig_call_guard.py exists for).  This
 * object is therefore listed in the flowcore Makefile under BOTH symbols, the
 * same shape luna_ponmac_logic.o already uses for the two GPON configs.
 *
 * WHY luna_* AND NOT rtl9602c_*: the GMAC engine is the FAMILY's -- both
 * shells read every MAC offset, descriptor bit and ring constant from ONE
 * header (luna_eth_regs.h, the 34-identical/4-moved census of 2026-08-28) --
 * and family-shared code may not hide behind one chip's name.  The two pack
 * functions below moved here from rtl9602c_l34_logic.{c,h} and were renamed
 * THE DAY luna_eth.c was proven to spell the identical expressions (its own
 * R_RxDesNum comment already recorded "the identical value is stored ... by
 * rtl9602c_eth.c:3371"; that inline spelling was the family's THIRD copy).
 *
 * The tier rule holds: no MMIO, no kernel service, no shell file-scope state;
 * family descriptor bit MASKS are passed as arguments (the rtl9602c_rx_frame_bad
 * precedent) so this header never re-spells a fact luna_eth_regs.h owns.
 */
#ifndef _LUNA_GMAC_LOGIC_H
#define _LUNA_GMAC_LOGIC_H

#include <linux/types.h>

u32 luna_gmac_rxdesnum_pack(unsigned int ring_size, unsigned int th_on,
			    unsigned int th_off);
u32 luna_gmac_rxcdo_pack(unsigned int ring_size);
bool luna_gmac_rx_frame_bad(u32 opts1, u32 err_mask, u32 len,
			    u32 hdr_floor, u32 buf_size);
bool luna_gmac_rx_cpu_tag_present(const u8 *data, u32 len,
				  unsigned int tag_len);


/* ── family GMAC RING arithmetic (folded in from luna_gmac_ring.h,
 *    2026-09-04) ─────────────────────────────────────────────────────
 * ★ IT ARRIVED AS A SECOND FILE AND WAS FOLDED THE SAME DAY.  A parallel
 * pass created luna_gmac_ring.h in THIS directory, for THIS engine,
 * included by luna_eth.c on the line immediately below this header --
 * and its own comment prescribed the fix it had just made necessary:
 * "two homes for one engine's logic is the duplication this tier exists
 * to end".  The justification it gave for a separate file (a header
 * costs no Makefile line) is not a differentiator: these three inlines
 * cost none here either, and luna_gmac_logic.o is already built under
 * BOTH CONFIG_RTL9602C_ETH and CONFIG_LUNA_ETH -- the two shells that
 * consume them. */
/*
 * Is the TX ring full for a producer that holds @reserve slots back?
 *
 * @head and @dirty are the shell's FREE-RUNNING producer/reclaim counters, so
 * it is their UNSIGNED DIFFERENCE that is compared and never the two indices:
 * wrap is then correct by construction, which is why neither is reduced modulo
 * @size before it gets here.
 *
 * "- 1" is the slot always kept in hand: head == dirty is the EMPTY encoding,
 * so a ring of @size can hold at most @size - 1 outstanding descriptors.
 *
 * @reserve is the caller's ROLE, not a property of the ring -- a data producer
 * sharing the ring with an OMCI injector passes that injector's reserve, the
 * injector itself passes 0.  Both wake-queue sites call
 * !luna_gmac_tx_ring_full() so the full and not-full directions cannot
 * disagree the way seven separate hand-spellings could.
 *
 * ⚠ THAT SENTENCE WAS FALSE WHEN IT WAS FIRST WRITTEN (2026-09-04): it stated
 * the call shape as present fact while BOTH wake sites still spelled
 * `(head - dirty) < SIZE - 1` by hand, and a grep over target/linux found zero
 * callers.  All seven spellings -- one in luna_eth.c, six in rtl9602c_eth.c,
 * across the TX and the OMCI TX rings -- now call this function, so the
 * sentence is true.  It is kept as a record: a core header that asserts a call
 * shape nobody has yet written is the same drift gpon_common.h:100-104 already
 * paid for.
 *
 * ⚠ THE ONE INPUT THIS DOES NOT DEFEND AGAINST, said out loud rather than
 * silently clamped, and the boundary is EXACT because the differential walks
 * it: at @reserve == @size - 1 the threshold is 0 and the ring reads ALWAYS
 * full (a drop -- annoying, safe); at @reserve >= @size the unsigned threshold
 * UNDERFLOWS to ~0u and the ring reads NEVER full, which is an overrun.  The
 * two directions are one step apart.  The expression is kept byte-identical to
 * the seven shell spellings on purpose -- this is code motion, and inventing a
 * different bound is a behaviour change nobody measured; every caller today
 * passes a compile-time constant far below the ring size.  The differential PROVES that failure direction rather than
 * assuming it, so the day a variable reserve appears the fact is already on
 * record.
 */
static inline bool luna_gmac_tx_ring_full(unsigned int head, unsigned int dirty,
					  unsigned int size, unsigned int reserve)
{
	return (head - dirty) >= size - 1 - reserve;
}

/*
 * Is @slot the ring's wrap descriptor -- the one that carries D_EOR?
 *
 * @slot is a PHYSICAL slot index, already reduced modulo @size and already
 * rotated if that shell rotates.  That is the whole content of this function
 * and the reason it is not folded into the caller: on a ring whose software
 * start is rotated to the engine's CDO (rtl9602c_eth.c:658-678, paid for with
 * "off-by-CDO skips ... reclaim wedged"), the wrap bit belongs to the last
 * PHYSICAL slot and NOT to the last counter value.  luna_eth.c does not rotate,
 * so there the two coincide -- by accident, and nothing said so in one place
 * until now.
 */
static inline bool luna_gmac_slot_is_eor(unsigned int slot, unsigned int size)
{
	return slot == size - 1;
}

/*
 * The TX descriptor BODY word (opts1/word0) of a single-segment frame.
 *
 * @flags     the shell's own D_FS | D_LS | D_TXCRC [| D_IPCS | steering bits]
 * @len       DMA length in bytes, AFTER any runt padding the shell did
 * @len_mask  the shell's TXD_LEN_MASK
 * @eor       luna_gmac_slot_is_eor() for this slot
 * @eor_bit   the shell's D_EOR
 *
 * The masking is the part that earns its keep beside the EOR conditional:
 * TXD_LEN_MASK is 17 bits wide and D_TXCRC sits at bit 23, so an unmasked
 * oversize length spills upward into flag territory -- and the two are now
 * decided in one expression instead of a compose line plus a conditional
 * repeated at seven sites.
 *
 * D_OWN is NOT here; see the file header.
 */
static inline u32 luna_gmac_txd_word0(u32 flags, u32 len, u32 len_mask,
				      bool eor, u32 eor_bit)
{
	return flags | (len & len_mask) | (eor ? eor_bit : 0u);
}

#endif /* _LUNA_GMAC_LOGIC_H */
