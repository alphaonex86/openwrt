/* SPDX-License-Identifier: GPL-2.0-only */
/* gpon_ddm.h -- optical diagnostic (DDM) unit conversion, ...
 * dev/MEASURED-gpon_ddm.h.md sec 1. */
#ifndef _GPON_DDM_H
#define _GPON_DDM_H

#include <linux/types.h>

/* gpon_ddm_uw10_to_cdbm() - 0.1 uW count to centi-dBm (1 mW = ...
 * dev/MEASURED-gpon_ddm.h.md sec 2. */
s32 gpon_ddm_uw10_to_cdbm(u32 raw);

/* gpon_ddm_cdbm_to_anig() - centi-dBm to the G.988 ANI-G (ME ...
 * dev/MEASURED-gpon_ddm.h.md sec 3. */
s16 gpon_ddm_cdbm_to_anig(s32 cdbm);

#endif /* _GPON_DDM_H */
