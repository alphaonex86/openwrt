/* SPDX-License-Identifier: GPL-2.0 */
/* SFF-8472 A2h digital-diagnostic-monitoring (DDM) decode + ...
 * dev/MEASURED-cortina-gpon-ddm.h.md sec 1. */

#ifndef _CORTINA_GPON_DDM_H_
#define _CORTINA_GPON_DDM_H_

#include <linux/types.h>
#include "gpon_ddm.h"	/* the one 0.1 uW -> centi-dBm conversion */

/* SFF-8472 A2h real-time diagnostics: byte 96, ten bytes, read-only. */
#define CG_DDM_BASE		0x60
#define CG_DDM_LEN		10

/* Why a sample is not usable.  Anything but CG_DDM_OK means NO number may be
 * printed or reported -- a fabricated 0 for a failed read is the one thing this
 * whole file exists to prevent. */
enum cg_ddm_status {
	CG_DDM_OK = 0,
	CG_DDM_ERR_IO,		/* an i2c transfer failed */
	CG_DDM_ERR_ALL_ZERO,	/* every byte 0x00: unrouted pinmux fake-ACK */
	CG_DDM_ERR_ALL_ONES,	/* every byte 0xFF: floating bus / no device */
};

/* One decoded sample.  The RAW 16-bit words are kept verbatim (spy/dump is a
 * first-class feature, and it makes the scaling re-checkable after the fact). */
struct cg_bosa_ddm {
	u16	temp;		/* 0x60-61 */
	u16	vcc;		/* 0x62-63 */
	u16	bias;		/* 0x64-65 */
	u16	tx_pwr;		/* 0x66-67 */
	u16	rx_pwr;		/* 0x68-69 */
	u8	raw[CG_DDM_LEN];
	u8	status;		/* enum cg_ddm_status */
};

/* An optical power that cannot come from any real reading (the weakest real
 * value, raw = 1, is -40.00 dBm), used for "the optic reports zero light". */
#define CG_DDM_CDBM_NONE	(-100000)

/* Clamp for the OMCI conversion: +-60 dBm covers every physical reading and
 * keeps the G.988 0.002 dB result inside s16. */
#define CG_DDM_CDBM_MIN		(-6000)
#define CG_DDM_CDBM_MAX		(6000)

/* Decode ten raw A2h bytes. @io_err is nonzero if any i2c ...
 * dev/MEASURED-cortina-gpon-ddm.h.md sec 2. */
static inline int cg_ddm_decode(const u8 *raw, int io_err, struct cg_bosa_ddm *d)
{
	unsigned int i;
	u8 and_all = 0xff, or_all = 0x00;

	d->temp = 0;
	d->vcc = 0;
	d->bias = 0;
	d->tx_pwr = 0;
	d->rx_pwr = 0;
	for (i = 0; i < CG_DDM_LEN; i++)
		d->raw[i] = 0;

	if (io_err) {
		d->status = CG_DDM_ERR_IO;
		return d->status;
	}

	for (i = 0; i < CG_DDM_LEN; i++) {
		d->raw[i] = raw[i];
		and_all &= raw[i];
		or_all |= raw[i];
	}
	if (or_all == 0x00) {
		d->status = CG_DDM_ERR_ALL_ZERO;
		return d->status;
	}
	if (and_all == 0xff) {
		d->status = CG_DDM_ERR_ALL_ONES;
		return d->status;
	}

	/* explicit byte math, never a cast over wire bytes: the same code has to
	 * run on LE ARM64 and BE MIPS (project endianness rule) */
	d->temp   = ((u16)raw[0] << 8) | raw[1];
	d->vcc    = ((u16)raw[2] << 8) | raw[3];
	d->bias   = ((u16)raw[4] << 8) | raw[5];
	d->tx_pwr = ((u16)raw[6] << 8) | raw[7];
	d->rx_pwr = ((u16)raw[8] << 8) | raw[9];
	d->status = CG_DDM_OK;
	return d->status;
}

/* Temperature in milli-degrees C (s16, 1/256 degC).  Truncates toward zero. */
static inline s32 cg_ddm_temp_mdegc(u16 raw)
{
	return ((s32)(s16)raw * 1000) / 256;
}

/* Temperature in deci-degrees C -- the unit the existing rpcd/LuCI and test
 * scrapers already parse ("temp_dc="); full precision stays available through
 * the raw word (raw / 256 degC). */
static inline s32 cg_ddm_temp_dc(u16 raw)
{
	return ((s32)(s16)raw * 10) / 256;
}

/* Supply voltage in mV (100 uV units). */
static inline u32 cg_ddm_vcc_mv(u16 raw)
{
	return (u32)raw / 10;
}

/* Laser bias current in uA (2 uA units). */
static inline u32 cg_ddm_bias_ua(u16 raw)
{
	return (u32)raw * 2;
}

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

/* Human-readable reason a sample is unusable.  Never returns NULL. */
static inline const char *cg_ddm_status_str(u8 status)
{
	switch (status) {
	case CG_DDM_OK:
		return "live";
	case CG_DDM_ERR_IO:
		return "unavailable (i2c read error)";
	case CG_DDM_ERR_ALL_ZERO:
		return "unavailable (DEAD BUS: all ten bytes 0x00 - i2c0 pinmux unrouted?)";
	default:
		return "unavailable (DEAD BUS: all ten bytes 0xff - BOSA absent?)";
	}
}

#endif /* _CORTINA_GPON_DDM_H_ */
