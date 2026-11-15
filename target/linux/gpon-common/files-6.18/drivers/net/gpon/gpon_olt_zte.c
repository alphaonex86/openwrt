// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: VENDOR */
/* gpon_olt_zte.c -- the ZTE GPON OLT PON MAC block's PLOAM registers and indirect BW-map access (gpon_olt_zte.h). */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/unaligned.h>

#include "gpon_olt_zte.h"

#define ZTE_PONMAC_PLOAM_WORDS	3
#define ZTE_PONMAC_PLOAMD_SEND	0x7		/* start, three copies: what both stocks send */
#define ZTE_PONMAC_IND_STATUS	4
#define ZTE_PONMAC_IND_DATA	8
#define ZTE_PONMAC_IND_TRIES	20
#define ZTE_PONMAC_IND_POLL_US	50

static u32 zpm_rd(const struct zte_ponmac *pm, unsigned int link, u32 off)
{
	return pm->rd(pm->ctx, pm->window[link] + off);
}

static void zpm_wr(const struct zte_ponmac *pm, unsigned int link, u32 off, u32 val)
{
	pm->wr(pm->ctx, pm->window[link] + off, val);
}

int zte_ponmac_ploam_tx(const struct zte_ponmac *pm, unsigned int link, const u8 *msg)
{
	u32 ctrl = zpm_rd(pm, link, pm->lay->ploamd_ctrl);
	unsigned int i;

	if (ctrl & BIT(0))
		return -EBUSY;
	for (i = 0; i < ZTE_PONMAC_PLOAM_WORDS; i++)
		zpm_wr(pm, link, pm->lay->ploamd_msg + 4 * i, get_unaligned_be32(msg + 4 * i));
	zpm_wr(pm, link, pm->lay->ploamd_ctrl, ctrl | ZTE_PONMAC_PLOAMD_SEND);
	return 0;
}
EXPORT_SYMBOL_GPL(zte_ponmac_ploam_tx);

bool zte_ponmac_ploam_rx(const struct zte_ponmac *pm, unsigned int link, u8 *msg, u32 *rtd)
{
	unsigned int i;

	if (!(zpm_rd(pm, link, pm->lay->ploamu_held) & BIT(0)))
		return false;
	for (i = 0; i < ZTE_PONMAC_PLOAM_WORDS; i++)
		put_unaligned_be32(zpm_rd(pm, link, pm->lay->ploamu_msg + 4 * i), msg + 4 * i);
	*rtd = zpm_rd(pm, link, pm->lay->ploamu_rtd) & pm->lay->ploamu_rtd_mask;
	zpm_wr(pm, link, pm->lay->ploamu_held, BIT(0));
	return true;
}
EXPORT_SYMBOL_GPL(zte_ponmac_ploam_rx);

static int zpm_ind_done(const struct zte_ponmac *pm, unsigned int link, u32 cmd_off)
{
	unsigned int i;

	for (i = 0; i < ZTE_PONMAC_IND_TRIES; i++) {
		if (zpm_rd(pm, link, cmd_off + ZTE_PONMAC_IND_STATUS) & BIT(0))
			return 0;
		udelay(ZTE_PONMAC_IND_POLL_US);
	}
	return -ETIMEDOUT;
}

/* The command, then the data high to low (the write of word 0 stores the entry), then done; ready first when the
 * chip's stock waits for it. */
int zte_ponmac_ind_write(const struct zte_ponmac *pm, unsigned int link, u32 cmd_off, u32 cmd, const u32 *w,
			 unsigned int n)
{
	int ret = pm->lay->ind_ready_first ? zpm_ind_done(pm, link, cmd_off) : 0;

	if (ret)
		return ret;
	zpm_wr(pm, link, cmd_off, cmd);
	while (n--)
		zpm_wr(pm, link, cmd_off + ZTE_PONMAC_IND_DATA + 4 * n, w[n]);
	return zpm_ind_done(pm, link, cmd_off);
}
EXPORT_SYMBOL_GPL(zte_ponmac_ind_write);

int zte_ponmac_ind_read(const struct zte_ponmac *pm, unsigned int link, u32 cmd_off, u32 cmd, u32 *w,
			unsigned int n)
{
	unsigned int i;
	int ret;

	zpm_wr(pm, link, cmd_off, cmd);
	ret = zpm_ind_done(pm, link, cmd_off);
	for (i = 0; !ret && i < n; i++)
		w[i] = zpm_rd(pm, link, cmd_off + ZTE_PONMAC_IND_DATA + 4 * i);
	return ret;
}
EXPORT_SYMBOL_GPL(zte_ponmac_ind_read);

int zte_ponmac_map_write(const struct zte_ponmac *pm, unsigned int link, u32 table, u32 entry, const u32 *w,
			 unsigned int n)
{
	return zte_ponmac_ind_write(pm, link, pm->lay->map_cmd, table << pm->lay->map_table_shift | entry, w, n);
}
EXPORT_SYMBOL_GPL(zte_ponmac_map_write);

int zte_ponmac_map_read(const struct zte_ponmac *pm, unsigned int link, u32 table, u32 entry, u32 *w,
			unsigned int n)
{
	return zte_ponmac_ind_read(pm, link, pm->lay->map_cmd,
				   BIT(pm->lay->map_read_bit) | table << pm->lay->map_table_shift | entry, w, n);
}
EXPORT_SYMBOL_GPL(zte_ponmac_map_read);

MODULE_DESCRIPTION("ZTE GPON OLT PON MAC block: PLOAM and BW-map access");
MODULE_LICENSE("GPL");
