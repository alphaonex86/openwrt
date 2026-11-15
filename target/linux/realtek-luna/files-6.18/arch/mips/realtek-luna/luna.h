/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: FAMILY */
#ifndef _REALTEK_LUNA_H
#define _REALTEK_LUNA_H

void luna_mark(char c);		/* bring-up bisect marker, CONFIG_LUNA_DEBUG */

#ifdef CONFIG_PCI
void luna_pcie_endpoint_reset(void);
#else
static inline void luna_pcie_endpoint_reset(void) { }
#endif

#endif
