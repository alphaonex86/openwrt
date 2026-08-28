// SPDX-License-Identifier: GPL-2.0-only
/* See cortina_ni_tx_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>

#include "cortina_ni_tx_logic.h"

/* explicit byte math: this driver must stay endianness-agnostic */
u64 ca_ni_mac_key(const u8 *mac)
{
	return ((u64)mac[0] << 40) | ((u64)mac[1] << 32) | ((u64)mac[2] << 24) |
	       ((u64)mac[3] << 16) | ((u64)mac[4] << 8) | mac[5];
}
