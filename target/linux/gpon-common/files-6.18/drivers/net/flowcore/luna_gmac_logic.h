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

#endif /* _LUNA_GMAC_LOGIC_H */
