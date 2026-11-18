/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: FAMILY */
#ifndef _REALTEK_LUNA_H
#define _REALTEK_LUNA_H

void luna_mark(char c);		/* bring-up bisect marker, CONFIG_LUNA_DEBUG */

#ifdef CONFIG_PCI
void luna_pcie_endpoint_reset(void);
void luna_pcie_endpoint_hold(void);
#else
static inline void luna_pcie_endpoint_reset(void) { }
static inline void luna_pcie_endpoint_hold(void) { }
#endif

#if IS_BUILTIN(CONFIG_RTL9602C_ETH) || IS_BUILTIN(CONFIG_LUNA_ETH)
void luna_gmac_quiesce_inherited(void);
#else
static inline void luna_gmac_quiesce_inherited(void) { }
#endif

#endif
