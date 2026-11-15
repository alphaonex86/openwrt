/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: VENDOR */
/*
 * gpon_olt_zte.h -- the ZTE GPON OLT PON MAC block, one window per PON link, as the ZX279200 (PCIe) and the ZX279221's
 * packet processor both carry it: PLOAM both ways and the indirect BW-map memories. A chip gives its accessors, each
 * link's window and the offsets and command fields that differ; what a memory entry means (serial-number window,
 * ranging grant) and the bring-up stay in the chip file.
 */
#ifndef GPON_OLT_ZTE_H
#define GPON_OLT_ZTE_H

#include <linux/types.h>

struct zte_ponmac_layout {
	u16 ploamu_msg;			/* 12 octets: ONU-ID, message ID, 10 data */
	u16 ploamu_rtd;
	u32 ploamu_rtd_mask;
	u16 ploamu_held;		/* bit 0 reads 1 while a message is held; writing 1 takes it */
	u16 ploamd_ctrl;		/* bit 0 start (the chip clears it), 2:1 repeat */
	u16 ploamd_msg;			/* 12 octets; the chip adds the CRC */
	u16 map_cmd;			/* the BW-map memories' command word */
	u8 map_read_bit;
	u8 map_table_shift;		/* the entry sits below it */
	bool ind_ready_first;		/* the chip's stock waits for ready before each command */
};

struct zte_ponmac {
	const struct zte_ponmac_layout *lay;
	u32 (*rd)(void *ctx, u32 off);
	void (*wr)(void *ctx, u32 off, u32 val);
	void *ctx;
	const u32 *window;		/* link -> the offset of its window */
};

/* 0, or -EBUSY while the previous message is still being sent. */
int zte_ponmac_ploam_tx(const struct zte_ponmac *pm, unsigned int link, const u8 *msg);
/* -> true with the held upstream message taken and its round trip. */
bool zte_ponmac_ploam_rx(const struct zte_ponmac *pm, unsigned int link, u8 *msg, u32 *rtd);
/* The block's indirect memories share one shape: a command word at @cmd_off, done in bit 0 of the word after it,
 * the data words from +8. One entry, @n data words; -ETIMEDOUT when the memory stays busy (a write waits after its
 * data, and before its command when the layout says so; a read after its command). Never sleeps. */
int zte_ponmac_ind_write(const struct zte_ponmac *pm, unsigned int link, u32 cmd_off, u32 cmd, const u32 *w,
			 unsigned int n);
int zte_ponmac_ind_read(const struct zte_ponmac *pm, unsigned int link, u32 cmd_off, u32 cmd, u32 *w,
			unsigned int n);
/* The same on the BW-map's memories, the command made of @table and @entry. */
int zte_ponmac_map_write(const struct zte_ponmac *pm, unsigned int link, u32 table, u32 entry, const u32 *w,
			 unsigned int n);
int zte_ponmac_map_read(const struct zte_ponmac *pm, unsigned int link, u32 table, u32 entry, u32 *w,
			unsigned int n);

#endif /* GPON_OLT_ZTE_H */
