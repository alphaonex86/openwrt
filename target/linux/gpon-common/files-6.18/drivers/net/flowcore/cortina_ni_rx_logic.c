// SPDX-License-Identifier: GPL-2.0-only
/* See cortina_ni_rx_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>

#include "cortina_ni_rx_logic.h"

/* extract an LSB-first bitfield (width <= 32) from the 32 LE HDR_I words */
u32 rx_hdri_get(const u32 *w, unsigned int bit, unsigned int width)
{
	u64 v = ((u64)w[(bit >> 5) + 1] << 32) | w[bit >> 5];

	v >>= bit & 31;
	return v & (width < 32 ? (1u << width) - 1 : 0xffffffffu);
}
