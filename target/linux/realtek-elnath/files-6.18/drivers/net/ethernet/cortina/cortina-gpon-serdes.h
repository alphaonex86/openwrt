/* SPDX-License-Identifier: GPL-2.0 */
/* GPON PON-SerDes analog/CMU/PLL profile for RTL9607F Elnath.
 * Register FACTS extracted from the board's stock /etc/serdes/pon_serdes_gpon.txt
 * (header "RL6900_P7_GPON_20230921"): the CMU/PLL/CDR/TX-driver config the SerDes needs to
 * generate the PON APB clock.  Each row: writel(val, pon_window + off).
 * Applied via the PSDS DATAIN(0xa08c)/ACCESS(0xa088) register pair. */
struct cg_serdes_row { u16 off; u32 val; u16 delay_us; };

/* ★★ THE TABLE IS PAIRS, AND UNTIL 2026-09-02 THE FILE MADE YOU COUNT THEM.
 *    266 rows alternating between two addresses are really 133 LOGICAL WRITES
 *    to indexed SerDes registers: the value goes to DATAIN, then a command
 *    naming the register goes to ACCESS.  The header above always said so in
 *    prose; the table did not, so a reader could not tell which SerDes
 *    register any row programmed, nor that 0x420..0x7bf is the index space.
 *
 * ⚠ THE NAMES ARE NOT INVENTED: this file's own header already calls the pair
 *   "DATAIN(0xa08c)/ACCESS(0xa088)".  Naming them records a fact that was
 *   already written down; no SerDes register beyond the pair is named, because
 *   nothing in this tree establishes one.
 *
 * ★ THE MACRO EXPANDS TO THE SAME TWO ROWS, so the write ORDER is unchanged and
 *   so is the walker's per-row delay -- cortina-gpon.c applies
 *   `udelay(delay_us ? delay_us : 10)` to EVERY row, so one logical write has
 *   always cost two 10 us waits and still does.
 *
 * ★ THE INDEX ORDER IS DELIBERATE AND NOT SORTED (measured: 133 distinct
 *   indices in 0x420..0x7bf, not monotonic).  Sorting this table would change
 *   the bring-up sequence with nothing to see. */
#define CG_PSDS_DATAIN	0xa08c		/* the value the next command will write */
#define CG_PSDS_ACCESS	0xa088		/* command: CG_PSDS_WRITE | serdes reg index */
#define CG_PSDS_WRITE	0xc0000000u

#define CG_SDS_W(idx, value)						\
	{ CG_PSDS_DATAIN, (value), 0 },					\
	{ CG_PSDS_ACCESS, CG_PSDS_WRITE | (idx), 0 }

static const struct cg_serdes_row cg_serdes_gpon[] = {
	CG_SDS_W(0x420, 0x00004802),
	CG_SDS_W(0x421, 0x00000000),
	CG_SDS_W(0x422, 0x00000000),
	CG_SDS_W(0x423, 0x00000000),
	CG_SDS_W(0x424, 0x0000f0f0),
	CG_SDS_W(0x425, 0x0000f0f0),
	CG_SDS_W(0x426, 0x0000f0f0),
	CG_SDS_W(0x427, 0x0000f0f0),
	CG_SDS_W(0x428, 0x00000000),
	CG_SDS_W(0x429, 0x00008007),
	CG_SDS_W(0x42a, 0x00000304),
	CG_SDS_W(0x42b, 0x00000180),
	CG_SDS_W(0x42c, 0x0000ffff),
	CG_SDS_W(0x42d, 0x00000000),
	CG_SDS_W(0x42e, 0x00000000),
	CG_SDS_W(0x42f, 0x0000f000),
	CG_SDS_W(0x430, 0x00001800),
	CG_SDS_W(0x431, 0x0000ffd0),
	CG_SDS_W(0x432, 0x0000b87e),
	CG_SDS_W(0x433, 0x0000e005),
	CG_SDS_W(0x434, 0x00000401),
	CG_SDS_W(0x435, 0x00001000),
	CG_SDS_W(0x436, 0x00004000),
	CG_SDS_W(0x437, 0x00000000),
	CG_SDS_W(0x438, 0x00000000),
	CG_SDS_W(0x439, 0x00000000),
	CG_SDS_W(0x43a, 0x00000000),
	CG_SDS_W(0x43b, 0x00000000),
	CG_SDS_W(0x43c, 0x00000000),
	CG_SDS_W(0x43d, 0x00000000),
	CG_SDS_W(0x43e, 0x00000000),
	CG_SDS_W(0x43f, 0x00001000),
	CG_SDS_W(0x6c0, 0x00000000),
	CG_SDS_W(0x6c1, 0x0000ff00),
	CG_SDS_W(0x6c2, 0x0000ff01),
	CG_SDS_W(0x6c3, 0x00000000),
	CG_SDS_W(0x6c4, 0x0000ff00),
	CG_SDS_W(0x600, 0x00005a35),
	CG_SDS_W(0x601, 0x00006004),
	CG_SDS_W(0x602, 0x0000f0f0),
	CG_SDS_W(0x603, 0x0000f0f9),
	CG_SDS_W(0x604, 0x00001e40),
	CG_SDS_W(0x605, 0x0000878c),
	CG_SDS_W(0x606, 0x000001c0),
	CG_SDS_W(0x607, 0x00003e00),
	CG_SDS_W(0x608, 0x0000d125),
	CG_SDS_W(0x609, 0x00002504),
	CG_SDS_W(0x60a, 0x00004006),
	CG_SDS_W(0x60b, 0x00002424),
	CG_SDS_W(0x60c, 0x00004012),
	CG_SDS_W(0x60d, 0x00004f18),
	CG_SDS_W(0x60e, 0x00000042),
	CG_SDS_W(0x60f, 0x00005458),
	CG_SDS_W(0x610, 0x000049b3),
	CG_SDS_W(0x611, 0x0000f0ae),
	CG_SDS_W(0x612, 0x0000f070),
	CG_SDS_W(0x613, 0x00001e20),
	CG_SDS_W(0x614, 0x0000200a),
	CG_SDS_W(0x615, 0x00008a69),
	CG_SDS_W(0x616, 0x00001597),
	CG_SDS_W(0x617, 0x0000f0f0),
	CG_SDS_W(0x618, 0x0000f0f2),
	CG_SDS_W(0x619, 0x0000f0f0),
	CG_SDS_W(0x61a, 0x000042e8),
	CG_SDS_W(0x61b, 0x00000100),
	CG_SDS_W(0x61c, 0x0000f0f0),
	CG_SDS_W(0x61d, 0x00000000),
	CG_SDS_W(0x61e, 0x00008120),
	CG_SDS_W(0x61f, 0x00000008),
	CG_SDS_W(0x620, 0x0000360b),
	CG_SDS_W(0x621, 0x0000034d),
	CG_SDS_W(0x622, 0x00000000),
	CG_SDS_W(0x623, 0x0000fe40),
	CG_SDS_W(0x624, 0x00000000),
	CG_SDS_W(0x625, 0x00000096),
	CG_SDS_W(0x626, 0x00000000),
	CG_SDS_W(0x627, 0x00000000),
	CG_SDS_W(0x628, 0x00000000),
	CG_SDS_W(0x629, 0x00005848),
	CG_SDS_W(0x62a, 0x0000ff27),
	CG_SDS_W(0x62b, 0x0000d800),
	CG_SDS_W(0x62c, 0x00000080),
	CG_SDS_W(0x62d, 0x00008000),
	CG_SDS_W(0x62e, 0x00004000),
	CG_SDS_W(0x62f, 0x0000180c),
	CG_SDS_W(0x630, 0x00000804),
	CG_SDS_W(0x631, 0x000003fc),
	CG_SDS_W(0x632, 0x000086ff),
	CG_SDS_W(0x633, 0x00000020),
	CG_SDS_W(0x634, 0x0000f0ff),
	CG_SDS_W(0x635, 0x0000ffff),
	CG_SDS_W(0x636, 0x0000ffff),
	CG_SDS_W(0x637, 0x00003e00),
	CG_SDS_W(0x638, 0x00000000),
	CG_SDS_W(0x639, 0x00000500),
	CG_SDS_W(0x63a, 0x000007ff),
	CG_SDS_W(0x63b, 0x00000fe8),
	CG_SDS_W(0x63c, 0x00003fdf),
	CG_SDS_W(0x63d, 0x0000001e),
	CG_SDS_W(0x63e, 0x00008000),
	CG_SDS_W(0x63f, 0x00000000),
	CG_SDS_W(0x7a0, 0x00000018),
	CG_SDS_W(0x7a1, 0x000011e0),
	CG_SDS_W(0x7a2, 0x000003ff),
	CG_SDS_W(0x7a3, 0x00000447),
	CG_SDS_W(0x7a4, 0x00004000),
	CG_SDS_W(0x7a5, 0x00008000),
	CG_SDS_W(0x7a6, 0x00000219),
	CG_SDS_W(0x7a7, 0x000000c5),
	CG_SDS_W(0x7a8, 0x00000f00),
	CG_SDS_W(0x7a9, 0x00004000),
	CG_SDS_W(0x7aa, 0x00000000),
	CG_SDS_W(0x7ab, 0x00000255),
	CG_SDS_W(0x7ac, 0x00000255),
	CG_SDS_W(0x7ad, 0x0000c800),
	CG_SDS_W(0x7ae, 0x00001070),
	CG_SDS_W(0x7af, 0x00000000),
	CG_SDS_W(0x7b0, 0x00000000),
	CG_SDS_W(0x7b1, 0x00000400),
	CG_SDS_W(0x7b2, 0x0000004c),
	CG_SDS_W(0x7b3, 0x00000000),
	CG_SDS_W(0x7b4, 0x00000000),
	CG_SDS_W(0x7b5, 0x0000f000),
	CG_SDS_W(0x7b6, 0x00005477),
	CG_SDS_W(0x7b7, 0x0000f0b0),
	CG_SDS_W(0x7b8, 0x00007030),
	CG_SDS_W(0x7b9, 0x00001c3f),
	CG_SDS_W(0x7ba, 0x00000f0f),
	CG_SDS_W(0x7bb, 0x0000e116),
	CG_SDS_W(0x7bc, 0x00006d19),
	CG_SDS_W(0x7bd, 0x00006117),
	CG_SDS_W(0x7be, 0x00009500),
	CG_SDS_W(0x7bf, 0x00000f0f),
};
