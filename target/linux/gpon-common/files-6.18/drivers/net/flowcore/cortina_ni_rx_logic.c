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

/* Pack a MAC into the FDB key words (aal __aal_mac_2_fdb_data; vid/scind/dot1p
 * = 0) - shared by the append and the lookup-only path in cortina-ni-rx.c so
 * both hash to the same bucket. */
void cortina_ni_l2fe_fdb_key(const u8 *mac, u32 *d3, u32 *d2, u32 *d1)
{
	*d3 = (mac[0] >> 5) & 0x7;
	*d2 = ((u32)(mac[0] & 0x1f) << 27) | ((u32)mac[1] << 19) |
	      ((u32)mac[2] << 11) | ((u32)mac[3] << 3) | ((mac[4] >> 5) & 0x7);
	*d1 = (u32)(((mac[4] & 0x1f) << 8) | mac[5]) << 19;
}

/*
 * Declared deviation from moved-verbatim, both functions below: the shell
 * passed its device state (`const struct cortina_ni_rx *rx`) and read
 * rx->bringup_ticks; that struct is the shell's and cannot cross the tier,
 * so the tick count itself is the argument.  It stays u64 like the field it
 * mirrors - `unsigned int` would change the ladder at the 2^32 wrap.  The
 * bodies are otherwise verbatim.
 */
unsigned int cortina_ni_rx_bringup_period(u64 ticks)
{
	if (ticks <= CA_NI_RX_BRINGUP_FAST_TICKS)
		return 1u;
	if (ticks <= CA_NI_RX_BRINGUP_MID_TICKS)
		return CA_NI_RX_BRINGUP_MID_PERIOD;
	return CA_NI_RX_BRINGUP_SLOW_PERIOD;
}

bool cortina_ni_rx_bringup_due(u64 ticks)
{
	unsigned int period = cortina_ni_rx_bringup_period(ticks);

	/* `bringup_ticks` has already been incremented for THIS tick, so tick
	 * 1 is due (1 % 1 == 0) and the ladder never leaves a silent gap at a
	 * phase boundary. */
	return (ticks % period) == 0u;
}
