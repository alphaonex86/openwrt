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

/* SFF-8472 A2h real-time diagnostics: byte 96, ten bytes, read-only.  The
 * DECODE is the standard's and lives here once; the bus that fetches the
 * bytes is the family's. */
#define GPON_DDM_A2H_BASE	0x60
#define GPON_DDM_A2H_LEN	10

/* Why a sample is not usable.  Anything but GPON_DDM_OK means NO number may
 * be reported from it. */
enum gpon_ddm_status {
	GPON_DDM_OK = 0,
	GPON_DDM_ERR_IO,	/* a bus transfer failed */
	GPON_DDM_ERR_ALL_ZERO,	/* every byte 0x00: unrouted pinmux fake-ACK */
	GPON_DDM_ERR_ALL_ONES,	/* every byte 0xFF: floating bus / no device */
};

/* One decoded sample.  The RAW bytes are kept verbatim: spy/dump is a
 * first-class capability, never bolted on. */
struct gpon_ddm_a2h {
	u16	temp;		/* 0x60-61, 1/256 degC, signed */
	u16	vcc;		/* 0x62-63, 100 uV */
	u16	bias;		/* 0x64-65, 2 uA */
	u16	tx_pwr;		/* 0x66-67, 0.1 uW */
	u16	rx_pwr;		/* 0x68-69, 0.1 uW */
	u8	raw[GPON_DDM_A2H_LEN];
	u8	status;		/* enum gpon_ddm_status */
};

/* Decode ten raw A2h bytes.  @io_err nonzero: a transfer failed and @raw is
 * not to be trusted.  Every field is zeroed first, so an unusable sample never
 * carries a number.  Explicit byte math, never a cast over wire bytes. */
static inline int gpon_ddm_a2h_decode(const u8 *raw, int io_err,
				      struct gpon_ddm_a2h *d)
{
	unsigned int i;
	u8 and_all = 0xff, or_all = 0x00;

	d->temp = 0;
	d->vcc = 0;
	d->bias = 0;
	d->tx_pwr = 0;
	d->rx_pwr = 0;
	for (i = 0; i < GPON_DDM_A2H_LEN; i++)
		d->raw[i] = 0;
	if (io_err) {
		d->status = GPON_DDM_ERR_IO;
		return d->status;
	}
	for (i = 0; i < GPON_DDM_A2H_LEN; i++) {
		d->raw[i] = raw[i];
		and_all &= raw[i];
		or_all |= raw[i];
	}
	if (or_all == 0x00) {
		d->status = GPON_DDM_ERR_ALL_ZERO;
		return d->status;
	}
	if (and_all == 0xff) {
		d->status = GPON_DDM_ERR_ALL_ONES;
		return d->status;
	}
	d->temp   = ((u16)raw[0] << 8) | raw[1];
	d->vcc    = ((u16)raw[2] << 8) | raw[3];
	d->bias   = ((u16)raw[4] << 8) | raw[5];
	d->tx_pwr = ((u16)raw[6] << 8) | raw[7];
	d->rx_pwr = ((u16)raw[8] << 8) | raw[9];
	d->status = GPON_DDM_OK;
	return d->status;
}

/* Temperature in deci-degrees C (s16, 1/256 degC), the unit /proc/gpon and
 * the suite read.  Truncates toward zero. */
static inline s32 gpon_ddm_temp_dc(u16 raw)
{
	return ((s32)(s16)raw * 10) / 256;
}

/* Supply voltage in mV (100 uV units). */
static inline u32 gpon_ddm_vcc_mv(u16 raw)
{
	return (u32)raw / 10;
}

/* Laser bias current in uA (2 uA units). */
static inline u32 gpon_ddm_bias_ua(u16 raw)
{
	return (u32)raw * 2;
}

/* Human-readable reason a sample is unusable.  Never returns NULL. */
static inline const char *gpon_ddm_status_str(u8 status)
{
	switch (status) {
	case GPON_DDM_OK:
		return "live";
	case GPON_DDM_ERR_IO:
		return "unavailable (i2c read error)";
	case GPON_DDM_ERR_ALL_ZERO:
		return "unavailable (DEAD BUS: all ten bytes 0x00 - i2c0 pinmux unrouted?)";
	default:
		return "unavailable (DEAD BUS: all ten bytes 0xff - BOSA absent?)";
	}
}

#endif /* _GPON_DDM_H */
