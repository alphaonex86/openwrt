/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_rtl9602c_logic.h -- logic hoisted out of gpon-rtl9602c.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _GPON_RTL9602C_LOGIC_H
#define _GPON_RTL9602C_LOGIC_H

#include <linux/types.h>

u8 bosa_slave_for(u16 reg);
s32 ddm_word_to_level(int raw);
s32 bosa_code_to_cdbm(u32 code);

/*
 * "Could not measure" sentinel for the RX optical chain. bosa_rx_code_calc()
 * floors a genuine reading at 11, so 0 can never be a measurement and is free
 * to mean "no reading". The caller must render it as "n/a", never a number
 * (the shell's BOSA_RX_CDBM_NA is deliberately NOT the -4000 dBm floor a real
 * dark reading produces -- the two must not look alike).
 */
#define BOSA_RX_CODE_NA		0u

/* Per-board optical calibration. Stock loads this from rtl8290b.data into its
 * europa_param struct; the shell owns the instance (compiled defaults are that
 * board's confirmed values) and passes it down BY ARGUMENT -- the logic never
 * reaches back for file-scope state. */
struct bosa_optical_cal {
	/* Faithful RTL8290B RX-power chain (re-expressed from europa_drv.ko
	 * rtl8290b_rxPower_get + _rtl8290b_rx_power_cal; reproduces the reference to
	 * the centi-dBm across 5 samples). The dark term (rx_vthr) makes it AFFINE,
	 * not proportional -- a through-origin fraction fit is wrong off-anchor:
	 *   V     = (rssi - tap_lo)*3.3Vuv/(tap_hi - tap_lo)     (ratiometric, uV)
	 *   irssi = 1000*(V - rx_vthr)*(r1+r2)/(r1*r2)           (rx_vthr = dark level)
	 *   code  = ((b*(irssi/s1)/8192)*s1 + 1000*c/4096)/100,  s1 = irssi<65536?10:100
	 *   dBm   = 1000*log10(code) - 4000  (centi-dBm; bosa_code_to_cdbm)
	 * Per-board constants from rtl8290b.data (europa_param). */
	u32 rx_vthr;		/* RSSI detection threshold / dark level, uV (data @0x552) */
	u32 rx_r1, rx_r2;	/* RSSI load resistors, ohm (data @0x5df, @0x5e1, x10) */
	s32 rx_poly_b, rx_poly_c;	/* code poly, a=0 on this board (data @0x54a, @0x54e) */
	s16 temp_off;		/* temperature offset, degC (data @0x568) */
	s32 tx_slope, tx_offset;	/* TX word = (mpd*slope*10)>>8 + (offset*10>>5) (data @0x55c/0x560) */
};

u32 bosa_rx_code_calc(u32 rssi, u32 tap_lo, u32 tap_hi,
		      const struct bosa_optical_cal *cal);
s32 bosa_temp_dc_calc(const int *a, const int *b, s16 temp_off);
u32 bosa_bias_ua_calc(int h, int l);
u32 bosa_tx_sample_contrib(s32 vmpd, s32 dark, int iavg, int range);
u32 bosa_tx_word_calc(u64 sum, int n, s32 tx_slope, s32 tx_offset);

/* One packed-array slot: where a `bits`-wide entry lives inside the pi register
 * space. Set and get share this ONE locate result -- before the hoist they
 * could only agree by parallel maintenance, which is how the contiguous-pack
 * SID2QID mis-addressing stayed invisible (the wrong get mirrored the wrong
 * set). */
struct pi_packed_slot {
	u32 reg;		/* byte address of the 32-bit word (driver-relative) */
	unsigned int shift;	/* bit position of the field inside that word */
	u32 mask;		/* field mask, unshifted */
};

struct pi_packed_slot pi_packed_locate(u32 base, unsigned int idx,
				       unsigned int bits);
u32 pi_packed_insert(u32 word, const struct pi_packed_slot *slot, u32 val);
u32 pi_packed_extract(u32 word, const struct pi_packed_slot *slot);

#endif /* _GPON_RTL9602C_LOGIC_H */
