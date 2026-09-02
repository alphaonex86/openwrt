/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cortina_ni_tx_logic.h -- logic hoisted out of cortina-ni-tx.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _CORTINA_NI_TX_LOGIC_H
#define _CORTINA_NI_TX_LOGIC_H

#include <linux/types.h>

u64 ca_ni_mac_key(const u8 *mac);

/* ------------------------------------------------------------------ */
/* TX round (2026-09-02): the TX shell's DECISIONS move here -- the    */
/* CPU->LAN egress-port pick with its DA/FDB codec, the ring-space     */
/* arithmetic, the DMA-LSO VP->lspid classifier, the L2FE ARB          */
/* ldpid->pdpid map semantics and the descriptor / PON HEADER_A        */
/* encoders.  The shell keeps the table PROTOCOL: the DATA-register    */
/* order, the GO kicks, the polls, the doorbells and the locks.        */
/* ------------------------------------------------------------------ */

/*
 * CPU->LAN egress policy (ex-cortina-ni-tx.c file-local enum; the values are
 * the lan_tx_mode module-param ABI and may not be renumbered).
 */
enum {
	CA_NI_LAN_TX_FIXED	= 0,	/* every frame -> the fixed port (revert) */
	CA_NI_LAN_TX_FLOOD	= 1,	/* every frame -> every linked port     */
	CA_NI_LAN_TX_LEARN	= 2,	/* learned port, flood fallback         */
};

/*
 * ★ DELIBERATE IDENTICAL REDEFINITION (same rule as cortina_ni_rx_logic.h).
 * The constant below spells the same value as cortina-ni-regs.h, token for
 * token.  The pick function range-checks WITH it and may not read the shell's
 * register header; the shell still includes both headers (its WARN message
 * prints the same bound), so if either side ever moves, every build of the
 * shell warns on the redefinition instead of the two homes drifting silently.
 */
#define CA_NI_TX_DEST_LDPID_COUNT	16

/* One FDB bucket for @mac; @mask = table size - 1 (a power of two).  The
 * shell's table is sized by its own struct, so the mask is the argument --
 * the same declared deviation as the rx ladder's tick count. */
unsigned int ca_ni_mac_bucket(const u8 *mac, unsigned int mask);

/* The lan_fdb bucket codec: one u64 entry published atomically as
 * {mac[47:0], port[50:48], valid[51]} (layout moved from cortina-ni-tx.c;
 * its only reader and writer call these two). */
u64 ca_ni_lan_fdb_ent(u64 mac_key, u32 port);
/* -> the bound port, or -1 when invalid / keyed to another MAC. */
int ca_ni_lan_fdb_ent_port(u64 ent, u64 mac_key);

/*
 * The CPU->LAN egress-port decision for one frame.  Inputs are everything
 * the old shell-resident decision read; the outputs carry WHAT WAS DECIDED
 * so the shell can keep its own counters and its WARN (kernel services).
 * @ports is never empty.
 */
struct ca_ni_lan_tx_pick {
	u32	ports;		/* egress port bitmap, never empty */
	bool	hit;		/* learned unicast port used (shell: lan_hit++) */
	bool	flood;		/* flood fallback taken (shell: lan_flood++) */
	bool	force_oor;	/* force_ldpid set but out of range (shell WARNs) */
};

void cortina_ni_lan_tx_pick(int force_ldpid, int mode, u32 link,
			    unsigned int fixed_port, bool da_mc,
			    u64 fdb_ent, u64 da_key,
			    struct ca_ni_lan_tx_pick *p);

/* Free descriptors of a ring with SW write pointer @wptr and oldest
 * un-reclaimed slot @finished, keeping one in hand (moved verbatim;
 * the ring size is the argument for the same reason as the FDB mask). */
unsigned int cortina_ni_ring_free_desc(unsigned int wptr,
				       unsigned int finished,
				       unsigned int size);

/* DMA-LSO VP->source-lspid map value for map entry @vp (stock
 * rtk_ni_init_tx_dma_lso): VP 1..11 source from CPU logical port 0x10+n,
 * every other entry from CPU port 0x10. */
u32 cortina_ni_tx_vp_lspid(unsigned int vp);

/* L2FE ARB PDPID table index: {my_mac[7], dbuf[6], ldpid[5:0]}. */
u32 cortina_ni_arb_idx(u32 my_mac, u32 dbuf, u32 ldpid);

/*
 * What pdpid the vendor map (aal_port.c global port init) programs at ARB
 * index @idx, or -1 for an index this driver's init never writes.  One home
 * for the map SEMANTICS; the two shell walkers keep their write ORDER.
 */
int cortina_ni_arb_pdpid(u32 idx);

/* Direct-TX-to-LAN descriptor word1 (plain frame, no header-A, HP=11,
 * auto checksum); the per-port DEST field is OR'd on by _dest(). */
u32 cortina_ni_tx_desc1_direct(u32 len, u32 cos);
u32 cortina_ni_tx_desc1_dest(u32 port);

/* The 2-descriptor PON HEADER_A chain's word1 pair: SOF + HP=01 header
 * block, then the EOF frame of @frame_len bytes. */
u32 cortina_ni_tx_desc1_pon_sof(void);
u32 cortina_ni_tx_desc1_pon_eof(u32 frame_len);

/*
 * Fill the 16-byte DMA-LSO header block {LSO para0 = 0, LSO para1 =
 * @pkt_size, HEADER_A hi, HEADER_A lo} for a CPU-injected US PON frame.
 * Explicit little-endian byte math (this tier is endianness-agnostic).
 * fe_bypass + no_drop are unconditional on this path and the source lspid
 * is always CPU0; the caller chooses cos / ldpid / pol_id (OMCI vs data).
 */
#define CA_NI_PON_HDR_BLK_LEN	16	/* lso0 + lso1 + HEADER_A */
void cortina_ni_pon_hdr_blk_fill(u8 *blk, u32 pkt_size, u32 cos, u32 ldpid,
				 u32 pol_id);

#endif /* _CORTINA_NI_TX_LOGIC_H */
