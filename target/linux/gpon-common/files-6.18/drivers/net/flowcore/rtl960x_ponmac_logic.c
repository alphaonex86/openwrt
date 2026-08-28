// SPDX-License-Identifier: GPL-2.0-only
/* See rtl960x_ponmac_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>

#include "rtl960x_ponmac_logic.h"

/* True for the SerDes offsets our golden table writes but the stock rev-A bring-up never does. */
bool c2_off_overconfig(u32 off)
{
	u32 a = off & 0xffffu;
	return (a >= 0x2608 && a <= 0x265c) ||	/* duplicate GPON per-rate bank 1 */
	       (a >= 0x2688 && a <= 0x26dc) ||	/* duplicate GPON per-rate bank 2 */
	       (a >= 0x2788 && a <= 0x27dc) ||	/* duplicate GPON per-rate bank 3 */
	       (a >= 0x2c00 && a <= 0x2df8);	/* the 4 FIB-bank bodies            */
}
