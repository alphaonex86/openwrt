/* SPDX-License-Identifier: GPL-2.0 */
/* SFF-8472 A2h digital-diagnostic-monitoring (DDM) decode + ...
 * dev/MEASURED-cortina-gpon-ddm.h.md sec 1.
 * ★ THE DECODE MOVED TO THE CORE (gpon_ddm.h, 2026-09-16): SFF-8472 is a
 *   standard, not this family's.  What stays here is this family's REPORTING
 *   policy -- the "no reading" sentinel and the +-60 dBm clamp. */

#ifndef _CORTINA_GPON_DDM_H_
#define _CORTINA_GPON_DDM_H_

#include <linux/types.h>
#include "gpon_ddm.h"	/* the SFF-8472 decode and the one 0.1 uW -> centi-dBm conversion */

/* An optical power that cannot come from any real reading (the weakest real
 * value, raw = 1, is -40.00 dBm), used for "the optic reports zero light". */
#define CG_DDM_CDBM_NONE	(-100000)

/* Clamp for the OMCI conversion: +-60 dBm covers every physical reading and
 * keeps the G.988 0.002 dB result inside s16. */
#define CG_DDM_CDBM_MIN		(-6000)
#define CG_DDM_CDBM_MAX		(6000)

/* Optical power in centi-dBm from an SFF-8472 0.1 uW word. ...
 * dev/MEASURED-cortina-gpon-ddm.h.md sec 3. */
static inline s32 cg_ddm_uw10_to_cdbm(u16 raw)
{
	if (!raw)
		return CG_DDM_CDBM_NONE;
	return gpon_ddm_uw10_to_cdbm(raw);
}

/* centi-dBm -> the G.988 ANI-G (ME 263) #10/#14 wire form: ...
 * dev/MEASURED-cortina-gpon-ddm.h.md sec 4. */
static inline u16 cg_ddm_cdbm_to_omci(s32 cdbm)
{
	if (cdbm < CG_DDM_CDBM_MIN)
		cdbm = CG_DDM_CDBM_MIN;
	else if (cdbm > CG_DDM_CDBM_MAX)
		cdbm = CG_DDM_CDBM_MAX;
	/* the +-60 dBm clamp above is THIS family's reporting policy; the unit
	 * conversion is everyone's, so it is named in the core. */
	return (u16)gpon_ddm_cdbm_to_anig(cdbm);
}

#endif /* _CORTINA_GPON_DDM_H_ */
