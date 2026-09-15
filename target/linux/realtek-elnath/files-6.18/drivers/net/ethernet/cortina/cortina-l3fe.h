/* SPDX-License-Identifier: GPL-2.0 */
/* cortina-l3fe.h - RTL9607F "Elnath" NE L3FE main-hash ...
 * dev/MEASURED-cortina-l3fe.h.md sec 9. */

#ifndef _CORTINA_L3FE_H
#define _CORTINA_L3FE_H

#include <linux/io.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>

/* DDR tables the engine DMA-reads (one contiguous ... -- dev/MEASURED-cortina-l3fe.h.md sec 1. */
#define CN_L3E_KEY_TBL_BYTES	0x40000		/* 64K x u32 CRC32 */
#define CN_L3E_FIB_TBL_BYTES	0x200000	/* 64K x 32 B actions */
#define CN_L3E_CARVE_BYTES	(CN_L3E_KEY_TBL_BYTES + CN_L3E_FIB_TBL_BYTES)

struct cn_l3e_tables {
	void		*key_virt;
	dma_addr_t	key_pa;
	void		*fib_virt;
	dma_addr_t	fib_pa;
};

/* One-time engine arm (ordered init chain, ... -- dev/MEASURED-cortina-l3fe.h.md sec 2. */
int cortina_l3fe_engine_init(void __iomem *ne, const struct cn_l3e_tables *t);

/* Program the stock profile/tuple + mask-table classify ...
 * dev/MEASURED-cortina-l3fe.h.md sec 3. */
int cortina_l3fe_classify_setup(void __iomem *ne);

/* Mask-table entry write (64 x 256-bit, two 128-bit beats, ...
 * dev/MEASURED-cortina-l3fe.h.md sec 10. */
int cortina_l3fe_mask_write(void __iomem *ne, u32 idx,
			    const u32 lo[4], const u32 hi[4]);

/* ★ Divergence B - enable HW L3-forwarding into the main hash ...
 * dev/MEASURED-cortina-l3fe.h.md sec 4. */
int cortina_l3fe_hw_l3_forward_enable(void __iomem *ne, const u8 *router_mac);

/* ★ The per-L3-interface T2 ADMISSION (stock ca_l3_intf_add ...
 * dev/MEASURED-cortina-l3fe.h.md sec 5. */
int cortina_l3fe_intf_add(void __iomem *ne, const u8 *lan_mac);

/* HS_SWO on-chip CRC engine: feed @nwords key words (HDR_I ...
 * dev/MEASURED-cortina-l3fe.h.md sec 6. */
int cortina_l3fe_swo_crc(void __iomem *ne, const u32 *words, int nwords,
			 u32 mask_id, u32 *crc32_out, u16 *crc16_out);

/* PPPoE WAN egress encap: program egress L3-IF table entry ...
 * dev/MEASURED-cortina-l3fe.h.md sec 7. */
int cortina_l3fe_pppoe_l3if_set(void __iomem *ne, u32 idx, u16 session,
				u8 an_sel);
/* A2 next-hop L2 rewrite: mac_da_idx MAC table + IPoE egress-SMAC L3-IF entry.
 * cortina_l3fe_ipoe_l3if_set is used for BOTH egress directions - the entry is
 * just "substitute the egress SMAC named by @an_sel": idx 2 / an_sel 2 = the
 * WAN MAC (US leg), idx 3 / an_sel 1 = the LAN router MAC (DS leg). */
int cortina_l3fe_macda_idx_set(void __iomem *ne, u32 idx, const u8 *mac);
int cortina_l3fe_ipoe_l3if_set(void __iomem *ne, u32 idx, u8 an_sel);

/* Point ONE main-hash profile's TUPLE0 maskptr at the routed ...
 * dev/MEASURED-cortina-l3fe.h.md sec 8. */
int cortina_l3fe_hash_profile_mask_repoint(void __iomem *ne, u32 profile);

#endif /* _CORTINA_L3FE_H */
