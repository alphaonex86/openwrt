/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CHIP */
/*
 * ZX279200 MAC/TM bring-up as the unit's own firmware runs it (2.3.1R, in its VM), block by block. Blocks carry the
 * names of the firmware's own module and register maps (gpptest); a block's tables are filled through its table
 * engine: a command word (table, entry), its ready bit 0, the entry's data words. Values are the firmware's; a field
 * nobody has decoded stays a number.
 */
#ifndef ZX279200_MAC_TM_H
#define ZX279200_MAC_TM_H

#include <linux/bits.h>

#include "gpon_regseq.h"

#define ZX_POLL_TRIES			500	/* x GPON_REGSEQ_POLL_US = 100 ms; the firmware waits 1 + 2 + 4 ms */

/* PON link n's MAC window (gpptest 2.3.1R ZX2763_*): upstream receiver, downstream sender, bandwidth maps. */
#define ZX_W(n)				(0x14000 + ((n) >> 1) * 0x4000 + ((n) & 1) * 0x1800)
#define ZX_URX				0x0800
#define ZX_DTX				0x1000
#define ZX_DBA				0x1800
#define ZX_DS_CTRL			0x1000	/* bit 1 the link's downstream sender on */
#define ZX_DS_ON			BIT(1)
#define ZX_DBA_CBAC			0x1908	/* 0 CBA mode, 8 map switch, 16 ready (read only) */
#define ZX_CBA_ON			BIT(0)	/* no BWmap at all before it: 0 SN requests at the ONU (unit, 2026-11-18) */
#define ZX_CBA_SWITCH			BIT(8)
#define ZX_CBA_READY			BIT(16)
#define ZX_LDCFG			0x08c0
#define ZX_DBA_TMRCTL			0x1914
#define ZX_DBA_ACCESS			0x1b00
#define ZX_DBA_BACFG1			0x1b04
#define ZX_DBA_BACFG2			0x1b08
#define ZX_DBA_BATOT1			0x1b0c
#define ZX_DBA_BATOT2			0x1b10
#define ZX_DBA_CPU_CTRL			0x1b14
#define ZX_DBA_BR_CFG1			0x1b18
#define ZX_DBA_BR_CFG2			0x1b1c
#define ZX_DBA_BR_CFG3			0x1b20

/* The TM and switch blocks, at the firmware's module map; the SerDes block is outside it. */
#define ZX_GLB				0x00000
#define ZX_SGLOBAL			0x04000
#define ZX_SMAC				0x04400
#define ZX_SIPC				0x04c00
#define ZX_SMSW				0x05400
#define ZX_SQMG				0x05800
#define ZX_SPPV				0x06000
#define ZX_GLOBAL			0x08000
#define ZX_DCLA				0x08c00
#define ZX_QPP				0x09000
#define ZX_DPMG				0x09400
#define ZX_DADM				0x09800
#define ZX_DRED				0x09c00
#define ZX_DSCH				0x0a400
#define ZX_URSM0			0x0b000
#define ZX_URSM1			0x0b200
#define ZX_URSM2			0x0b400
#define ZX_URSM3			0x0b600
#define ZX_UPMG				0x0b800
#define ZX_UPA				0x0c400
#define ZX_UCLA				0x0c800
#define ZX_UADM				0x0cc00
#define ZX_URED				0x0d000
#define ZX_USCH				0x0d800
#define ZX_CLA				0x0e000
#define ZX_MACSW			0x0e400
#define ZX_VMAC				0x0e800
#define ZX_PCIPP			0x0f000
#define ZX_STAT				0x0f800
#define ZX_NCLA				0x0fc00
#define ZX_LINK_CFG(n)			(ZX_URSM0 + 0x024 + 0x100 * (n))
#define ZX_SERDES			0x30000

/* The global indirect bus, and every block's table engine. */
#define ZX_IND_ADDR			0x00040
#define ZX_IND_READY			0x00044
#define ZX_IND_DATA			0x00048
#define ZX_TBL_CMD			0x14
#define ZX_TBL_READY			0x18
#define ZX_TBL_DATA(i)			(0x1c + 4 * (i))

struct zx_seq {
	const struct gpon_regseq_io *io;
	int err;
};

/* The first timeout ends the bring-up: every later access is skipped, as a table run stops there. */
static void zx_put(struct zx_seq *s, u32 off, u32 val)
{
	if (!s->err)
		s->io->wr(off, val);
}

static void zx_ready(struct zx_seq *s, u32 off)
{
	const struct gpon_regseq_op poll = GPON_POLL(off, 0, ZX_POLL_TRIES);

	if (!s->err)
		s->err = gpon_regseq_run(s->io, &poll, 1);
}

/* DBA totals of the 8 links, then the switch side. */
static void zx_tm_dba_switch(struct zx_seq *s)
{
	unsigned int j;

	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_W(2 * j) + ZX_DBA_BATOT1, 0x44012ef4);
		zx_put(s, ZX_W(2 * j) + ZX_DBA_BATOT2, 0x1000959c);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DBA_BATOT1, 0x44012ef4);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DBA_BATOT2, 0x1000959c);
	}
	zx_put(s, ZX_W(0) + ZX_DBA + ZX_TBL_CMD, 0x0100047e);
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_W(2 * j) + ZX_URX + 0x084, 0x0000000e);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_URX + 0x084, 0x0000000e);
	}
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_W(2 * j) + ZX_URX + 0x088, 0x00000345);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_URX + 0x088, 0x00000345);
	}
	zx_put(s, ZX_SMSW + 0x040, 0x000000fe);
	zx_put(s, ZX_SGLOBAL + 0x014, 0x00000101);
	zx_put(s, ZX_SQMG + 0x040, 0x00078000);
	for (j = 0; j < 8; j++)
		zx_put(s, ZX_LINK_CFG(j), 0x00000000);
}

/* Through the chip's indirect bus. */
static void zx_tm_ind_bus(struct zx_seq *s)
{
	zx_ready(s, ZX_IND_READY);
	zx_put(s, ZX_IND_ADDR, 0x4021c008);
	zx_put(s, ZX_IND_DATA, 0x00000040);
	zx_put(s, ZX_IND_ADDR, 0x4021c001);
	zx_put(s, ZX_IND_DATA, 0x0000e840);
	zx_put(s, ZX_IND_ADDR, 0x4021c003);
	zx_put(s, ZX_IND_DATA, 0x000000c0);
	zx_put(s, ZX_IND_ADDR, 0xc0218025);
}

/* The PON SerDes and its burst CDR. */
static void zx_tm_serdes(struct zx_seq *s)
{
	unsigned int j;

	for (j = 0; j < 8; j++) {
		zx_put(s, ZX_W(0) + 0x008 + j * 0x4, 0x80000000);
		zx_put(s, ZX_W(0) + 0x008 + j * 0x4, 0x00000000);
	}
	for (j = 0; j < 8; j++)
		zx_put(s, ZX_SERDES + j * 0x100, 0x00000001);
	for (j = 0; j < 8; j++)
		zx_put(s, ZX_SERDES + 0x010 + j * 0x100, 0x00004000);
	zx_put(s, ZX_SERDES + 0x324, 0x00000000);
	zx_put(s, ZX_SERDES + 0x424, 0x00000000);
	zx_put(s, ZX_SERDES + 0x328, 0x04040000);
	zx_put(s, ZX_SERDES + 0x428, 0x04040000);
	zx_put(s, ZX_SERDES + 0x320, 0x0400242c);
	zx_put(s, ZX_SERDES + 0x420, 0x0400242c);
	zx_put(s, ZX_SERDES + 0x320, 0x04002d2c);
	zx_put(s, ZX_SERDES + 0x420, 0x04002d2c);
	zx_put(s, ZX_SERDES + 0x4d4, 0x04000300);
	zx_put(s, ZX_SERDES + 0x4d8, 0x04040600);
	zx_put(s, ZX_SERDES + 0x4dc, 0x00000004);
	zx_put(s, ZX_SERDES + 0x4e0, 0x00003000);
	zx_put(s, ZX_SERDES + 0x464, 0x000f0003);
	zx_put(s, ZX_SERDES + 0x464, 0x0f0f0003);
	zx_put(s, ZX_SERDES + 0x468, 0xff0ffe00);
	zx_put(s, ZX_SERDES + 0x468, 0xff0f3f00);
	zx_put(s, ZX_SERDES + 0x468, 0xff1f3f00);
	zx_put(s, ZX_SERDES + 0x468, 0x401f3f00);
	zx_put(s, ZX_SERDES + 0x46c, 0xf3100b1f);
	zx_put(s, ZX_SERDES + 0x46c, 0xf310441f);
	zx_put(s, ZX_SERDES + 0x46c, 0xf31f441f);
	zx_put(s, ZX_SERDES + 0x46c, 0x3c1f441f);
	zx_put(s, ZX_SERDES + 0x470, 0x000fff1f);
	zx_put(s, ZX_SERDES + 0x470, 0x000f401f);
	zx_put(s, ZX_SERDES + 0x470, 0x001f401f);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x080 + j * 0x100, 0x00000100);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x580 + j * 0x100, 0x00000100);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x08c + j * 0x100, 0x00000001);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x58c + j * 0x100, 0x00000001);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x08c + j * 0x100, 0x00000001);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x58c + j * 0x100, 0x00000001);
	for (j = 0; j < 8; j++)
		zx_put(s, ZX_SERDES + 0x0a4 + j * 0x100, 0x00020400);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x0a8 + j * 0x100, 0x00000000);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x5a8 + j * 0x100, 0x00000000);
	for (j = 0; j < 8; j++)
		zx_put(s, ZX_SERDES + 0x0ac + j * 0x100, 0x10000000);
	for (j = 0; j < 8; j++)
		zx_put(s, ZX_SERDES + 0x0c0 + j * 0x100, 0x08000001);
	zx_put(s, ZX_SERDES + 0x4d0, 0x00010002);
	zx_put(s, ZX_SERDES + 0x4d0, 0x00010202);
	zx_put(s, ZX_SERDES + 0x4d0, 0x00000202);
	zx_put(s, ZX_SERDES + 0x4d0, 0x00000202);
	zx_put(s, ZX_SERDES + 0x4d4, 0x04000300);
	zx_put(s, ZX_SERDES + 0x4d4, 0x02000300);
	zx_put(s, ZX_SERDES + 0xb10, 0x00e7ffff);
	zx_put(s, ZX_SERDES + 0xb10, 0x00e7ffff);
	for (j = 0; j < 8; j++)
		zx_put(s, ZX_SERDES + 0x810 + j * 0x40, 0x05010001);
	for (j = 0; j < 8; j++)
		zx_put(s, ZX_SERDES + 0x814 + j * 0x40, 0x10000000);
	zx_put(s, ZX_SERDES + 0xa20, 0x0d1a0000);
	zx_put(s, ZX_SERDES + 0xa60, 0x00000101);
	zx_put(s, ZX_SERDES + 0xb60, 0x00000101);
	zx_put(s, ZX_SERDES + 0xa00, 0x00000001);
	zx_put(s, ZX_SERDES + 0xa08, 0x00000401);
	zx_put(s, ZX_SERDES + 0xa08, 0x00000701);
	zx_put(s, ZX_SERDES + 0xb08, 0x00000001);
	zx_put(s, ZX_SERDES + 0xa20, 0x0d1a0001);
	zx_put(s, ZX_SERDES + 0x408, 0x001f0000);
	zx_put(s, ZX_SERDES + 0x400, 0x00000005);
	zx_put(s, ZX_SERDES + 0x460, 0x00070001);
	for (j = 0; j < 4; j++)
		zx_put(s, ZX_SERDES + 0x008 + j * 0x100, 0x0000001f);
	zx_put(s, ZX_SERDES + 0x408, 0x001f001f);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x508 + j * 0x100, 0x0000001f);
	for (j = 0; j < 4; j++)
		zx_put(s, ZX_SERDES + 0x008 + j * 0x100, 0x00001f1f);
	zx_put(s, ZX_SERDES + 0x408, 0x001f1f1f);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x508 + j * 0x100, 0x00001f1f);
	zx_put(s, ZX_SERDES + 0x4dc, 0x00000304);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + j * 0x100, 0x00000101);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x500 + j * 0x100, 0x00000101);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + j * 0x100, 0x00010101);
	zx_put(s, ZX_SERDES + 0x300, 0x00010001);
	zx_put(s, ZX_SERDES + 0x400, 0x00010005);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x500 + j * 0x100, 0x00010101);
	zx_put(s, ZX_SERDES + 0xa00, 0x00000007);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + j * 0x100, 0x00010107);
	zx_put(s, ZX_SERDES + 0x300, 0x00010007);
	zx_put(s, ZX_SERDES + 0x400, 0x00010007);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x500 + j * 0x100, 0x00010107);
	zx_put(s, ZX_SERDES + 0xb00, 0x00000001);
	zx_put(s, ZX_SERDES + 0xb00, 0x0000ff01);
	for (j = 0; j < 8; j++) {
		zx_put(s, ZX_W(0) + 0x0c0 + j * 0x4, 0x80000000);
		zx_put(s, ZX_W(0) + 0x0c0 + j * 0x4, 0x00000000);
	}
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + j * 0x100, 0x00010105);
	zx_put(s, ZX_SERDES + 0x300, 0x00010005);
	zx_put(s, ZX_SERDES + 0x400, 0x00010005);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x500 + j * 0x100, 0x00010105);
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_W(2 * j) + ZX_DBA_BATOT1, 0x44012ef4);
		zx_put(s, ZX_W(2 * j) + ZX_DBA_BATOT2, 0x1000959c);
		zx_put(s, ZX_W(2 * j) + ZX_DBA_BATOT1, 0x44025afd);
		zx_put(s, ZX_W(2 * j) + ZX_DBA_BATOT2, 0x1001c467);
		zx_put(s, ZX_W(2 * j) + ZX_DBA_TMRCTL, 0x00000000);
		zx_put(s, ZX_W(2 * j) + ZX_DBA_TMRCTL, 0x00000000);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DBA_BATOT1, 0x44012ef4);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DBA_BATOT2, 0x1000959c);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DBA_BATOT1, 0x44025afd);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DBA_BATOT2, 0x1001c467);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DBA_TMRCTL, 0x00000000);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DBA_TMRCTL, 0x00000000);
	}
	for (j = 0; j < 4; j++)
		zx_put(s, ZX_W(0) + 0x034, 0x00000000);
	for (j = 0; j < 4; j++)
		zx_put(s, ZX_W(0) + 0x038, 0x00000000);
}

/* Every link's tables cleared, the reassembly blocks. */
static void zx_tm_link_tables(struct zx_seq *s)
{
	unsigned int i, j;

	for (i = 0; i < 8; i++) {
		for (j = 0; j < 1152; j++) {
			zx_ready(s, ZX_W(i) + ZX_URX + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_URX + ZX_TBL_CMD, 0x00c00000 + j);
			zx_put(s, ZX_W(i) + ZX_URX + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 4096; j++) {
			zx_ready(s, ZX_W(i) + ZX_DTX + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_DTX + ZX_TBL_CMD, 0x00000000 + j);
			zx_put(s, ZX_W(i) + ZX_DTX + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 1152; j++) {
			zx_ready(s, ZX_W(i) + ZX_DBA + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_CMD, 0x00800000 + j * 0x8);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
			zx_ready(s, ZX_W(i) + ZX_DBA + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_CMD, 0x00800001 + j * 0x8);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
			zx_ready(s, ZX_W(i) + ZX_DBA + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_CMD, 0x00800002 + j * 0x8);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
			zx_ready(s, ZX_W(i) + ZX_DBA + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_CMD, 0x00800003 + j * 0x8);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
			zx_ready(s, ZX_W(i) + ZX_DBA + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_CMD, 0x00800004 + j * 0x8);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
			zx_ready(s, ZX_W(i) + ZX_DBA + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_CMD, 0x00800005 + j * 0x8);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 1152; j++) {
			zx_ready(s, ZX_W(i) + ZX_DBA + ZX_TBL_READY);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_CMD, 0x01000000 + j);
			zx_put(s, ZX_W(i) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (j = 0; j < 8192; j++) {
		zx_ready(s, ZX_URSM0 + ZX_TBL_READY);
		zx_put(s, ZX_URSM0 + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_URSM0 + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 8192; j++) {
		zx_ready(s, ZX_URSM1 + ZX_TBL_READY);
		zx_put(s, ZX_URSM1 + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_URSM1 + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 8192; j++) {
		zx_ready(s, ZX_URSM2 + ZX_TBL_READY);
		zx_put(s, ZX_URSM2 + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_URSM2 + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 8192; j++) {
		zx_ready(s, ZX_URSM3 + ZX_TBL_READY);
		zx_put(s, ZX_URSM3 + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_URSM3 + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_W(2 * j) + ZX_URX + 0x084, 0x0000000c);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_URX + 0x084, 0x0000000c);
	}
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_W(2 * j) + ZX_URX + 0x088, 0x00000349);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_URX + 0x088, 0x00000349);
	}
}

/* Classifier, queue mapping, statistics and MAC switch tables. */
static void zx_tm_classifier(struct zx_seq *s)
{
	unsigned int i, j;

	zx_put(s, ZX_GLOBAL + 0x018, 0x88a88100);
	zx_put(s, ZX_GLOBAL + 0x020, 0x88a88100);
	zx_put(s, ZX_GLOBAL + 0x024, 0x88a88100);
	zx_put(s, ZX_GLOBAL + 0x028, 0x00000000);
	zx_put(s, ZX_GLOBAL + 0x03c, 0x00000001);
	zx_ready(s, ZX_UPA + ZX_TBL_READY);
	for (j = 0; j < 32768; j++) {
		zx_put(s, ZX_UPA + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_UPA + ZX_TBL_DATA(0), 0x00000000);
	}
	zx_ready(s, ZX_CLA + ZX_TBL_READY);
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_CLA + ZX_TBL_CMD, i * 0x400000 + j);
			zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
			zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x02000000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x02400000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 1024; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x03000000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 1024; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x03400000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 1024; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x03800000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 1024; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x03c00000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04400000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04800000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 256; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04c00000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 2048; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x05000000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 2048; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x05400000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 2048; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x05800000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 2048; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x05c00000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00000000);
	}
	zx_ready(s, ZX_QPP + ZX_TBL_READY);
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x00400000 + i * 0x400000 + j);
			zx_put(s, ZX_QPP + ZX_TBL_DATA(1), 0x00000000);
			zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x02400000 + j);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00000000);
	}
	zx_ready(s, ZX_STAT + ZX_TBL_READY);
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_STAT + ZX_TBL_CMD, i * 0x400000 + j);
			zx_put(s, ZX_STAT + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	zx_ready(s, ZX_MACSW + ZX_TBL_READY);
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 2048; j++) {
			zx_put(s, ZX_MACSW + ZX_TBL_CMD, i * 0x400000 + j);
			zx_put(s, ZX_MACSW + ZX_TBL_DATA(2), 0x00000000);
			zx_put(s, ZX_MACSW + ZX_TBL_DATA(1), 0x00000000);
			zx_put(s, ZX_MACSW + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	zx_put(s, ZX_MACSW + 0x008, 0xffffffff);
	zx_put(s, ZX_MACSW + 0x00c, 0x00000005);
	zx_put(s, ZX_MACSW + 0x004, 0x006e00ff);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_MACSW + ZX_TBL_CMD, 0x02000000 + j);
		zx_put(s, ZX_MACSW + ZX_TBL_DATA(0), 0x00003fff);
	}
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_MACSW + ZX_TBL_CMD, 0x02400000 + j);
		zx_put(s, ZX_MACSW + ZX_TBL_DATA(0), 0x00000000);
	}
	zx_put(s, ZX_VMAC + 0x004, 0x00000001);
	zx_ready(s, ZX_CLA + ZX_TBL_READY);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x02c00000 + j);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x00193fff);
	}
	zx_put(s, ZX_CLA + 0x074, 0xffff0000);
	zx_put(s, ZX_UCLA + 0x008, 0xaaaa0001);
	zx_put(s, ZX_DCLA + 0x008, 0xaaaa0000);
	zx_put(s, ZX_UCLA + 0x004, 0xaaaaaaa8);
	zx_put(s, ZX_DCLA + 0x004, 0xaaaaaaa8);
	zx_ready(s, ZX_NCLA + ZX_TBL_READY);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_NCLA + ZX_TBL_CMD, 0x02800000 + j);
		zx_put(s, ZX_NCLA + ZX_TBL_DATA(1), 0x00006aaa);
		zx_put(s, ZX_NCLA + ZX_TBL_DATA(0), 0xaaaaaaaa);
	}
	zx_ready(s, ZX_CLA + ZX_TBL_READY);
	zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04c000fc);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000004);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x1fffffff);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0xf0000000);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x04888000);
	zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04c000fd);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000004);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x1fffffff);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0xf0000eff);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0xf4888012);
	zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04c000ff);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000004);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x1fffffff);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0xf00000ff);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0xf4888012);
	zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04c000fe);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000004);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x1fffffff);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0xfffff0ff);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0xf4888a12);
	zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04c000fb);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000004);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x10fff000);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x01c00000);
	zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04c000f2);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000004);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x4ffff000);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000000);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x02800000);
	zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04c000f1);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(3), 0x00000004);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x4ffffff8);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x0ff00000);
	zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x02860f80);
	zx_put(s, ZX_NCLA + 0x004, 0x00010001);
	zx_put(s, ZX_NCLA + 0x008, 0xf0fe8000);
	zx_put(s, ZX_NCLA + 0x010, 0x3ffff101);
	zx_put(s, ZX_NCLA, 0x00000001);
	zx_ready(s, ZX_UADM + ZX_TBL_READY);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 32768; j++) {
		zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x00400000 + j);
		zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x00000000);
	}
}

/* Admission tables, both directions. */
static void zx_tm_admission(struct zx_seq *s)
{
	unsigned int i, j;

	for (i = 0; i < 6; i++) {
		for (j = 0; j < (i ? 128 : 4096); j++) {
			zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x00800000 + i * 0x400000 + j);
			zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (i = 0; i < 13; i++) {
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x02000000 + i * 0x400000 + j);
			zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	zx_ready(s, ZX_DADM + ZX_TBL_READY);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_DADM + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_DADM + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 32768; j++) {
		zx_put(s, ZX_DADM + ZX_TBL_CMD, 0x00400000 + j);
		zx_put(s, ZX_DADM + ZX_TBL_DATA(0), 0x00000000);
	}
	for (i = 0; i < 6; i++) {
		for (j = 0; j < (i ? 128 : 4096); j++) {
			zx_put(s, ZX_DADM + ZX_TBL_CMD, 0x00800000 + i * 0x400000 + j);
			zx_put(s, ZX_DADM + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (i = 0; i < 11; i++) {
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_DADM + ZX_TBL_CMD, 0x02000000 + i * 0x400000 + j);
			zx_put(s, ZX_DADM + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	zx_put(s, ZX_UADM, 0x00000012);
	zx_put(s, ZX_DADM, 0x00000002);
	zx_put(s, ZX_UADM + 0x004, 0x00000014);
	zx_put(s, ZX_DADM + 0x004, 0x00000014);
	zx_put(s, ZX_UADM + 0x03c, 0x000003e8);
	zx_put(s, ZX_DADM + 0x03c, 0x000003e8);
	zx_ready(s, ZX_UADM + ZX_TBL_READY);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x00000001);
	}
	for (i = 0; i < 8; i++) {
		zx_ready(s, ZX_DADM + ZX_TBL_READY);
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_DADM + ZX_TBL_CMD, (i ? 0x02800000 + i * 0x400000 : 0x00000000) + j);
			zx_put(s, ZX_DADM + ZX_TBL_DATA(0), (i ? 0x000013e8 : 0x00000001));
		}
		zx_ready(s, ZX_UADM + ZX_TBL_READY);
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x02400000 + i * 0x400000 + j);
			zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x000013e8);
		}
	}
	zx_ready(s, ZX_DADM + ZX_TBL_READY);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_DADM + ZX_TBL_CMD, 0x04800000 + j);
		zx_put(s, ZX_DADM + ZX_TBL_DATA(0), 0x000013e8);
	}
	zx_put(s, ZX_PCIPP + 0x07c, 0xbe8003e8);
	zx_put(s, ZX_DSCH + 0x010, 0x00000408);
	zx_put(s, ZX_DSCH + 0x054, 0x00000408);
	zx_put(s, ZX_DSCH + 0x058, 0x00001010);
	zx_ready(s, ZX_DSCH + ZX_TBL_READY);
	for (j = 0; j < 64; j++) {
		zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00080000 + j);
		zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 64; j++) {
		zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00100000 + j);
		zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x000003ff);
	}
}

/* Downstream then upstream scheduler tables. */
static void zx_tm_schedulers(struct zx_seq *s)
{
	unsigned int i, j;

	for (j = 0; j < 32; j++) {
		zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x02300000 + j);
		zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x0fffffff);
	}
	for (j = 0; j < 32; j++) {
		zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x03300000 + j);
		zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x0fffffff);
	}
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 16; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00180000 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 512; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00200000 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 512; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00280000 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 64; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00400000 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 64; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00480000 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 16; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, i * 0x10 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x0007ffff);
		}
		zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00400000 + i * 0x800000);
		zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x0007ffff);
		zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00480000 + i * 0x800000);
		zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x000003ff);
		for (j = 0; j < 2048; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00380000 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x0000003f);
		}
		for (j = 0; j < 2048; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00500000 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x00000064);
		}
		for (j = 0; j < 2048; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00500800 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x0003ffff);
		}
		for (j = 0; j < 256; j++) {
			zx_put(s, ZX_DSCH + ZX_TBL_CMD, 0x00580000 + i * 0x800000 + j);
			zx_put(s, ZX_DSCH + ZX_TBL_DATA(0), 0x00000001);
		}
	}
	for (j = 0; j < 8; j++) {
		zx_put(s, ZX_DSCH + ZX_TBL_DATA(2) + j * 0x40, 0x00020206);
		zx_put(s, ZX_DSCH + ZX_TBL_DATA(3) + j * 0x40, 0x000003ff);
		zx_put(s, ZX_DSCH + 0x030 + j * 0x40, 0x00020206);
		zx_put(s, ZX_DSCH + 0x038 + j * 0x40, 0x000003ff);
	}
	zx_put(s, ZX_DSCH + 0x048, 0x00ffffff);
	zx_put(s, ZX_DSCH + 0x04c, 0x00ffffff);
	zx_put(s, ZX_DSCH + 0x00c, 0x000000ff);
	zx_put(s, ZX_DSCH + 0x22c, 0x00000000);
	zx_put(s, ZX_USCH + 0x010, 0x00000408);
	zx_put(s, ZX_USCH + 0x054, 0x00000408);
	zx_put(s, ZX_USCH + 0x058, 0x00001010);
	zx_ready(s, ZX_USCH + ZX_TBL_READY);
	for (j = 0; j < 32; j++) {
		zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x02300000 + j);
		zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x0fffffff);
	}
	for (j = 0; j < 32; j++) {
		zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x03300000 + j);
		zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x0fffffff);
	}
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 16; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00180000 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 512; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00200000 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 512; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00280000 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 64; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00400000 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 64; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00480000 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x00000000);
		}
		for (j = 0; j < 16; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, i * 0x10 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x0007ffff);
		}
		zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00400000 + i * 0x800000);
		zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x0007ffff);
		zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00480000 + i * 0x800000);
		zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x000003ff);
		for (j = 0; j < 2048; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00380000 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x0000003f);
		}
		for (j = 0; j < 2048; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00500000 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x00000064);
		}
		for (j = 0; j < 2048; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00500800 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x0003ffff);
		}
		for (j = 0; j < 256; j++) {
			zx_put(s, ZX_USCH + ZX_TBL_CMD, 0x00580000 + i * 0x800000 + j);
			zx_put(s, ZX_USCH + ZX_TBL_DATA(0), 0x00000001);
		}
	}
	for (j = 0; j < 8; j++) {
		zx_put(s, ZX_USCH + ZX_TBL_DATA(2) + j * 0x40, 0x00020206);
		zx_put(s, ZX_USCH + ZX_TBL_DATA(3) + j * 0x40, 0x000003ff);
	}
	zx_put(s, ZX_USCH + 0x048, 0x00ffffff);
	zx_put(s, ZX_USCH + 0x04c, 0x00ffffff);
	zx_put(s, ZX_USCH + 0x00c, 0x000000ff);
	zx_put(s, ZX_USCH + 0x22c, 0x00000000);
	zx_put(s, ZX_DSCH + 0x004, 0x00000000);
	zx_put(s, ZX_DSCH, 0x00000000);
	zx_put(s, ZX_DSCH + 0x008, 0x00000000);
	zx_put(s, ZX_USCH, 0x00000000);
	zx_put(s, ZX_USCH + 0x008, 0x00000000);
	zx_ready(s, ZX_QPP + ZX_TBL_READY);
}

/* Queue managers and RED profiles. */
static void zx_tm_queues(struct zx_seq *s)
{
	unsigned int i, j;

	for (i = 0; i < 9; i++) {
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_QPP + ZX_TBL_CMD, i * 0x400000 + j);
			zx_put(s, ZX_QPP + ZX_TBL_DATA(1), 0x00000000);
			zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x02400000 + j);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(3), 0x00000000);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(2), 0x00000000);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(1), 0x00000000);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 64; j++) {
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x02800000 + j);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 32; j++) {
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x02c00000 + j);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 64; j++) {
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x0a800000 + j);
		zx_ready(s, ZX_QPP + ZX_TBL_READY);
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x02800000 + j);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00200000);
		zx_put(s, ZX_QPP + 0x140 + j * 0x4, 0x00000000);
	}
	for (j = 0; j < 8; j++) {
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x02c00000 + j * 0x4);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00000fff);
	}
	for (j = 0; j < 64; j++) {
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x02800000 + j);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00200000);
	}
	zx_put(s, ZX_QPP + 0x060, 0x000000ff);
	zx_put(s, ZX_QPP + 0x064, 0x00005555);
	zx_put(s, ZX_QPP + 0x100, 0x00053977);
	zx_put(s, ZX_QPP + 0x280, 0x00053977);
	zx_put(s, ZX_QPP + 0x240, 0x00fac688);
	zx_put(s, ZX_DPMG + 0x040, 0x0000efff);
	zx_put(s, ZX_DPMG + 0x044, 0x000007ff);
	zx_put(s, ZX_DPMG + 0x048, 0x000007ff);
	zx_put(s, ZX_UPMG, 0x00003fff);
	zx_ready(s, ZX_URED + ZX_TBL_READY);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_URED + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_URED + ZX_TBL_DATA(0), 0x00000000);
	}
	for (j = 0; j < 32768; j++) {
		zx_put(s, ZX_URED + ZX_TBL_CMD, 0x00400000 + j);
		zx_put(s, ZX_URED + ZX_TBL_DATA(0), 0x00000000);
	}
	zx_put(s, ZX_DRED + 0x040, 0x000002ff);
	zx_put(s, ZX_DRED + 0x050, 0x000002ff);
	zx_put(s, ZX_DRED + 0x044, 0x00008080);
	zx_put(s, ZX_DRED + 0x054, 0x00008080);
	zx_put(s, ZX_DRED + 0x048, 0x0012c12c);
	zx_put(s, ZX_DRED + 0x058, 0x0012c12c);
	zx_ready(s, ZX_DRED + ZX_TBL_READY);
	for (j = 0; j < 8; j++) {
		zx_put(s, ZX_DRED + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_DRED + ZX_TBL_DATA(2), 0x00000160);
		zx_put(s, ZX_DRED + ZX_TBL_DATA(1), 0x00580016);
		zx_put(s, ZX_DRED + ZX_TBL_DATA(0), 0x00058000);
	}
	zx_put(s, ZX_DRED + ZX_TBL_CMD, 0x00800000);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(2), 0x00004ff0);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(1), 0xbfc7fcff);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(0), 0xff7f7f7f);
	zx_put(s, ZX_DRED + ZX_TBL_CMD, 0x00800001);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(2), 0x000007f0);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(1), 0x0fc07c0f);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(0), 0xff7f7f7f);
	zx_put(s, ZX_DRED + ZX_TBL_CMD, 0x00800002);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(2), 0x00000000);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(1), 0x00000000);
	zx_put(s, ZX_DRED + ZX_TBL_DATA(0), 0xffffffff);
	for (j = 0; j < 16448; j++) {
		zx_put(s, ZX_DRED + ZX_TBL_CMD, 0x00400000 + j);
		zx_put(s, ZX_DRED + ZX_TBL_DATA(0), 0x00000010);
	}
	for (j = 0; j < 64; j++) {
		zx_put(s, ZX_DRED + ZX_TBL_CMD, 0x00404000 + j);
		zx_put(s, ZX_DRED + ZX_TBL_DATA(0), 0x00000820);
	}
	zx_ready(s, ZX_URED + ZX_TBL_READY);
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_URED + ZX_TBL_CMD, 0x01000000 + j);
		zx_put(s, ZX_URED + ZX_TBL_DATA(3), 0x00000017);
		zx_put(s, ZX_URED + ZX_TBL_DATA(2), 0xff5ffd7f);
		zx_put(s, ZX_URED + ZX_TBL_DATA(1), 0xf5ffd7ff);
		zx_put(s, ZX_URED + ZX_TBL_DATA(0), 0x00000000);
	}
	zx_put(s, ZX_URED + ZX_TBL_CMD, 0x00c00000);
	zx_put(s, ZX_URED + ZX_TBL_DATA(3), 0x00000001);
	zx_put(s, ZX_URED + ZX_TBL_DATA(2), 0x7dfbefbf);
	zx_put(s, ZX_URED + ZX_TBL_DATA(1), 0xfcffe3ff);
	zx_put(s, ZX_URED + ZX_TBL_DATA(0), 0xff7f7f7f);
	zx_put(s, ZX_URED + ZX_TBL_CMD, 0x00c00001);
	zx_put(s, ZX_URED + ZX_TBL_DATA(3), 0x00000000);
	zx_put(s, ZX_URED + ZX_TBL_DATA(2), 0x07f03f80);
	zx_put(s, ZX_URED + ZX_TBL_DATA(1), 0xfc03e00f);
	zx_put(s, ZX_URED + ZX_TBL_DATA(0), 0xff7f7f7f);
	for (j = 0; j < 16384; j++) {
		zx_put(s, ZX_URED + ZX_TBL_CMD, 0x00800000 + j);
		zx_put(s, ZX_URED + ZX_TBL_DATA(0), 0x00000008);
	}
	zx_put(s, ZX_URED + 0x040, 0x000000ff);
	zx_put(s, ZX_URED + 0x100, 0x00053977);
	zx_put(s, ZX_URED + 0x0c0, 0x00000000);
	zx_put(s, ZX_URED + 0x004, 0x00000003);
	zx_put(s, ZX_DRED + 0x004, 0x00000003);
	zx_put(s, ZX_STAT + 0x00c, 0x0000000f);
	zx_ready(s, ZX_IND_READY);
	zx_put(s, ZX_IND_ADDR, 0xc023c002);
	zx_put(s, ZX_SMSW, 0x000e0001);
	zx_put(s, ZX_SMSW + 0x004, 0x0015d461);
	zx_put(s, ZX_SMSW, 0x000e0073);
	zx_put(s, ZX_SMSW + 0x040, 0x000000fc);
	zx_put(s, ZX_SMSW + 0x044, 0x000000fc);
	for (j = 0; j < 4096; j++) {
		zx_ready(s, ZX_SMSW + ZX_TBL_READY);
		zx_put(s, ZX_SMSW + ZX_TBL_CMD, 0x03400000 + j);
		zx_put(s, ZX_SMSW + ZX_TBL_DATA(0), 0x00000111);
		zx_put(s, ZX_SMSW + ZX_TBL_CMD, 0x0b400000 + j);
	}
	zx_ready(s, ZX_SMSW + ZX_TBL_READY);
}

/* The switch-side blocks. */
static void zx_tm_switch(struct zx_seq *s)
{
	unsigned int i, j;

	for (i = 0; i < 8; i++) {
		for (j = 0; j < 4096; j++) {
			zx_put(s, ZX_SMSW + ZX_TBL_CMD, i * 0x400000 + j);
			zx_put(s, ZX_SMSW + ZX_TBL_DATA(2), 0x00000000);
			zx_put(s, ZX_SMSW + ZX_TBL_DATA(1), 0x00000000);
			zx_put(s, ZX_SMSW + ZX_TBL_DATA(0), 0x00000000);
		}
	}
	zx_put(s, ZX_SGLOBAL + 0x014, 0x00000101);
	zx_put(s, ZX_SGLOBAL + 0x014, 0x00000101);
	zx_put(s, ZX_SGLOBAL + 0x014, 0x00000109);
	zx_put(s, ZX_SIPC, 0x00000003);
	zx_put(s, ZX_SIPC + 0x00c, 0x22222222);
	zx_put(s, ZX_SIPC + 0x010, 0x00000002);
	zx_put(s, ZX_SIPC + 0x014, 0xaaaaaaa8);
	zx_put(s, ZX_SIPC + 0x018, 0xaaaa0000);
	for (j = 0; j < 7; j++) {
		zx_put(s, ZX_SQMG + 0x040 + j * 0x40, 0x00040000);
		zx_put(s, ZX_SQMG + 0x044 + j * 0x40, 0x00040000);
		zx_put(s, ZX_SQMG + 0x048 + j * 0x40, 0x00040000);
		zx_put(s, ZX_SQMG + 0x04c + j * 0x40, 0x00040000);
		zx_put(s, ZX_SQMG + 0x050 + j * 0x40, 0x00040000);
		zx_put(s, ZX_SQMG + 0x054 + j * 0x40, 0x00040000);
		zx_put(s, ZX_SQMG + 0x058 + j * 0x40, 0x00040000);
		zx_put(s, ZX_SQMG + 0x05c + j * 0x40, 0x00040000);
	}
	zx_put(s, ZX_SQMG + 0x200, 0x00040000);
	zx_put(s, ZX_SPPV, 0x00000001);
	zx_ready(s, ZX_SPPV + ZX_TBL_READY);
	for (j = 0; j < 4096; j++) {
		zx_put(s, ZX_SPPV + ZX_TBL_CMD, 0x00000000 + j);
		zx_put(s, ZX_SPPV + ZX_TBL_DATA(1), 0x00006aaa);
		zx_put(s, ZX_SPPV + ZX_TBL_DATA(0), 0xaaaaaaaa);
	}
	zx_put(s, ZX_SGLOBAL + 0x03c, 0x00000001);
}

/* Each PON's admission, classifier and switch entries. */
static void zx_tm_pon_flows(struct zx_seq *s)
{
	unsigned int i, j;

	for (i = 0; i < 8; i++) {
		zx_ready(s, ZX_UADM + ZX_TBL_READY);
		zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x02000001 + i);
		for (j = 0; j < 10; j++) {
			zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x00000000);
			zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x02000001 + i + j * 0x400000);
		}
		zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x00000000);
		zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x08000001 + i);
		zx_ready(s, ZX_UADM + ZX_TBL_READY);
		zx_put(s, ZX_UADM + ZX_TBL_CMD, 0x00000001 + i);
		zx_put(s, ZX_UADM + ZX_TBL_DATA(0), 0x00000001);
		zx_ready(s, ZX_DADM + ZX_TBL_READY);
		zx_put(s, ZX_DADM + ZX_TBL_CMD, 0x08000001 + i);
		zx_ready(s, ZX_DADM + ZX_TBL_READY);
		zx_put(s, ZX_DADM + ZX_TBL_CMD, 0x00000001 + i);
		zx_put(s, ZX_DADM + ZX_TBL_DATA(0), 0x00000001);
		zx_ready(s, ZX_MACSW + ZX_TBL_READY);
		zx_put(s, ZX_MACSW + ZX_TBL_CMD, 0x0a400001 + i);
		zx_ready(s, ZX_MACSW + ZX_TBL_READY);
		zx_put(s, ZX_MACSW + ZX_TBL_CMD, 0x02400001 + i);
		zx_put(s, ZX_MACSW + ZX_TBL_DATA(0), 0x00000000);
		zx_ready(s, ZX_CLA + ZX_TBL_READY);
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x0a800001 + i);
		zx_ready(s, ZX_CLA + ZX_TBL_READY);
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x02800001 + i);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), (i ? 0x00000002 : 0x00000000));
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x0286001d);
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x0c400001 + i);
		zx_ready(s, ZX_CLA + ZX_TBL_READY);
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04400001 + i);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(2), 0x0003fffa);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), (i ? 0x00000002 : 0x00000000));
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x0286001d);
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x0c800001 + i);
		zx_ready(s, ZX_CLA + ZX_TBL_READY);
		zx_put(s, ZX_CLA + ZX_TBL_CMD, 0x04800001 + i);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(1), 0x00000002);
		zx_put(s, ZX_CLA + ZX_TBL_DATA(0), 0x0286001d);
		zx_ready(s, ZX_MACSW + ZX_TBL_READY);
		zx_put(s, ZX_MACSW + ZX_TBL_CMD, 0x0a000001 + i);
		zx_ready(s, ZX_MACSW + ZX_TBL_READY);
		zx_put(s, ZX_MACSW + ZX_TBL_CMD, 0x02000001 + i);
		zx_put(s, ZX_MACSW + ZX_TBL_DATA(0), 0x00003fff);
		zx_ready(s, ZX_QPP + ZX_TBL_READY);
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x08000001 + i);
		zx_ready(s, ZX_QPP + ZX_TBL_READY);
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x00000001 + i);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(1), (i ? 0x00000008 : 0x00000000));
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), (i ? 0x000007fd + i * 0x2000 : 0x002007fd));
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x08400001 + i);
		zx_ready(s, ZX_QPP + ZX_TBL_READY);
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x00400001 + i);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(1), 0x00000008);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x00000fff + i * 0x2000);
		zx_ready(s, ZX_UPA + ZX_TBL_READY);
		zx_put(s, ZX_UPA + ZX_TBL_CMD, 0x00000fff + i * 0x1000);
		zx_put(s, ZX_UPA + ZX_TBL_DATA(0), 0x00000001 + i);
	}
}

/* Global settings after the tables. */
static void zx_tm_globals(struct zx_seq *s)
{
	unsigned int j;

	zx_ready(s, ZX_QPP + ZX_TBL_READY);
	for (j = 0; j < 8; j++) {
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x08000001 + j);
		zx_ready(s, ZX_QPP + ZX_TBL_READY);
		zx_put(s, ZX_QPP + ZX_TBL_CMD, 0x00000001 + j);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(1), 0x00000008);
		zx_put(s, ZX_QPP + ZX_TBL_DATA(0), 0x0000efff);
	}
	zx_put(s, ZX_GLB + 0x084, 0x00000003);
	zx_put(s, ZX_GLB + 0x054, 0x00000000);
	zx_put(s, ZX_GLB + 0x060, 0x00000000);
	zx_put(s, ZX_GLB + 0x084, 0x00000000);
	zx_put(s, ZX_GLB + 0x084, 0x0000000c);
	zx_put(s, ZX_GLB + 0x06c, 0x000001f0);
	zx_put(s, ZX_GLB + 0x078, 0x000001f0);
	zx_put(s, ZX_GLB + 0x084, 0x00000000);
	zx_ready(s, ZX_IND_READY);
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_IND_ADDR, 0x0043800f + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x00008000);
	}
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_IND_ADDR, 0x0063800f + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x00008000);
	}
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_IND_ADDR, 0x0003800f + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x00008000);
	}
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_IND_ADDR, 0x0023800f + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x00008000);
	}
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_IND_ADDR, 0x01e1c008 + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x00000040);
		zx_put(s, ZX_IND_ADDR, 0x01e1c001 + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x0000e840);
		zx_put(s, ZX_IND_ADDR, 0x01e1c003 + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x000000c0);
		zx_put(s, ZX_IND_ADDR, 0x01e18007 + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x0000c800);
		zx_put(s, ZX_IND_ADDR, 0x01e1800c + j * 0x10000000);
		zx_put(s, ZX_IND_DATA, 0x00000400);
	}
	zx_put(s, ZX_IND_ADDR, 0x81e18025);
	for (j = 0; j < 3; j++) {
		zx_put(s, ZX_IND_ADDR, 0x91e18025 + j * 0x10000000);
		zx_ready(s, ZX_IND_READY);
	}
	zx_put(s, ZX_SGLOBAL + 0x014, 0x00000109);
	zx_put(s, ZX_GLB + 0x054, 0x00000200);
	zx_put(s, ZX_GLB + 0x060, 0x00000200);
	zx_put(s, ZX_SGLOBAL + 0x014, 0x00000108);
	zx_put(s, ZX_GLB + 0x054, 0x00000000);
	zx_put(s, ZX_GLB + 0x060, 0x00000000);
	zx_put(s, ZX_SMAC + 0x040, 0x0307d840);
	zx_put(s, ZX_SMAC + 0x080, 0x0307d840);
}

/* The links and SerDes started. */
static void zx_tm_links_start(struct zx_seq *s)
{
	unsigned int j;

	for (j = 0; j < 8; j++) {
		zx_put(s, ZX_W(0) + 0x0c0 + j * 0x4, 0x80000000);
		zx_put(s, ZX_W(0) + 0x0c0 + j * 0x4, 0x00000000);
	}
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + j * 0x100, 0x00010107);
	zx_put(s, ZX_SERDES + 0x300, 0x00010007);
	zx_put(s, ZX_SERDES + 0x400, 0x00010007);
	for (j = 0; j < 3; j++)
		zx_put(s, ZX_SERDES + 0x500 + j * 0x100, 0x00010107);
	for (j = 0; j < 4; j++) {
		zx_put(s, ZX_W(2 * j) + ZX_DS_CTRL, 0x00000002);
		zx_put(s, ZX_W(1 + 2 * j) + ZX_DS_CTRL, 0x00000002);
	}
	zx_put(s, ZX_W(0) + ZX_URX + 0x080, 0x00000002);
	zx_put(s, ZX_W(0) + ZX_URX + 0x08c, 0x07ab5983);
	zx_put(s, ZX_W(0) + ZX_LDCFG, 0x00000000);
	zx_put(s, ZX_W(0) + ZX_URX + 0x088, 0x00000344);
	zx_put(s, ZX_W(0) + ZX_URX + 0x0bc, 0x00000001);
	zx_put(s, ZX_URSM0 + ZX_TBL_DATA(1), 0x000007d8);
	zx_put(s, ZX_URSM0 + ZX_TBL_DATA(3), 0x000e07bc);
	zx_put(s, ZX_W(0) + ZX_DTX + 0x04c, 0x00c00080);
	zx_put(s, ZX_W(0) + ZX_DS_CTRL, 0x00000016);
	zx_put(s, ZX_W(0) + ZX_URX + 0x080, 0x00000002);
	zx_put(s, ZX_W(0) + ZX_URX + 0x08c, 0x07ab5983);
	zx_put(s, ZX_W(0) + ZX_LDCFG, 0x00000000);
	zx_put(s, ZX_W(0) + ZX_URX + 0x088, 0x00000344);
	zx_put(s, ZX_W(0) + ZX_URX + 0x0bc, 0x00000001);
	zx_put(s, ZX_URSM0 + ZX_TBL_DATA(1), 0x000007d8);
	zx_put(s, ZX_URSM0 + ZX_TBL_DATA(3), 0x000e07bc);
	zx_put(s, ZX_W(0) + ZX_DTX + 0x04c, 0x00c00080);
	zx_put(s, ZX_W(0) + ZX_DS_CTRL, 0x00000016);
	zx_put(s, ZX_URSM0 + ZX_TBL_DATA(2), 0x00000001);
	zx_put(s, ZX_W(0) + ZX_DS_CTRL, 0x00000016);
	zx_put(s, ZX_W(0) + ZX_DBA_ACCESS, 0x1000002c);
	zx_put(s, ZX_W(0) + ZX_DBA + 0x348, 0x21012100);
	zx_put(s, ZX_W(0) + ZX_DBA + 0x328, 0x00000003);
	zx_put(s, ZX_W(0) + ZX_DBA_BATOT1, 0x55125afd);
	zx_put(s, ZX_W(0) + ZX_DBA_BATOT2, 0x1001c467);
	zx_put(s, ZX_W(0) + ZX_DBA_BACFG1, 0x097e0000);
	zx_put(s, ZX_W(0) + ZX_DBA_BACFG2, 0x02000000);
	zx_put(s, ZX_W(0) + ZX_DBA_BR_CFG1, 0x00000001);
	zx_put(s, ZX_W(0) + ZX_DBA_BR_CFG2, 0x000000c0);
	zx_put(s, ZX_W(0) + ZX_DBA_BR_CFG3, 0x00000000);
	zx_put(s, ZX_W(0) + ZX_DBA_CPU_CTRL, 0x00000000);
	zx_put(s, ZX_W(0) + ZX_DBA + 0x34c, 0x00000001);
	zx_put(s, ZX_W(0) + ZX_DBA_CPU_CTRL, 0x00000001);
	zx_ready(s, ZX_W(0) + ZX_DBA + ZX_TBL_READY);
	zx_put(s, ZX_W(0) + ZX_DBA + ZX_TBL_CMD, 0x00000000);
	zx_put(s, ZX_W(0) + ZX_DBA + ZX_TBL_DATA(1), 0x23000000);
	zx_put(s, ZX_W(0) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
	zx_ready(s, ZX_W(0) + ZX_DBA + ZX_TBL_READY);
	zx_put(s, ZX_W(0) + ZX_DBA + ZX_TBL_CMD, 0x00400000);
	zx_put(s, ZX_W(0) + ZX_DBA + ZX_TBL_DATA(1), 0x23000000);
	zx_put(s, ZX_W(0) + ZX_DBA + ZX_TBL_DATA(0), 0x00000000);
	zx_put(s, ZX_W(0) + ZX_DBA_CPU_CTRL, 0x00000000);
	zx_put(s, ZX_W(0) + ZX_DBA_TMRCTL, 0x00000000);
	zx_put(s, ZX_W(0) + ZX_DBA_TMRCTL, 0x00000001);
	zx_put(s, ZX_W(0) + ZX_DTX + 0x284, 0xff050000);
	zx_put(s, ZX_W(0) + ZX_DTX + 0x288, 0x00000000);
	zx_put(s, ZX_W(0) + ZX_DTX + 0x28c, 0x00000000);
}

/* The stages in the firmware's order. -> 0, or the first timeout. */
static int zx_mac_tm_init(const struct gpon_regseq_io *io)
{
	struct zx_seq s = { .io = io };

	zx_tm_dba_switch(&s);
	zx_tm_ind_bus(&s);
	zx_tm_serdes(&s);
	zx_tm_link_tables(&s);
	zx_tm_classifier(&s);
	zx_tm_admission(&s);
	zx_tm_schedulers(&s);
	zx_tm_queues(&s);
	zx_tm_switch(&s);
	zx_tm_pon_flows(&s);
	zx_tm_globals(&s);
	zx_tm_links_start(&s);
	return s.err;
}

#endif /* ZX279200_MAC_TM_H */
