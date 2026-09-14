/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The netdev station-address ladder, in the core so it exists once.
 *
 * MEASURED 2026-09-10: four drivers spelled this precedence and two of them
 * (cortina-ni-tx.c, cortina-gpon.c) ended in a compiled-in 02:96:07:f0:00:01 /
 * ...:02 with no random rung, so every X400AXF shipped one address.
 */
#ifndef _GPON_HWADDR_H
#define _GPON_HWADDR_H

#include <linux/types.h>

#define GPON_HWADDR_BYTES	6

/* In PRECEDENCE ORDER: the enum is the rule. */
enum gpon_hwaddr_src {
	GPON_HWADDR_BOOTARG = 0,
	GPON_HWADDR_DT,
	GPON_HWADDR_ENGINE,
	GPON_HWADDR_RANDOM,
};

/**
 * gpon_hwaddr_parse() - "xx:xx:xx:xx:xx:xx" -> six bytes.
 * Return: true when well formed; @out untouched otherwise.
 * REFUSES trailing text, which sscanf("%hhx:...") accepted while dropping it.
 */
bool gpon_hwaddr_parse(const char *s, u8 out[GPON_HWADDR_BYTES]);

/** Return: true for a non-zero unicast address. Refuses NULL. */
bool gpon_hwaddr_usable(const u8 *mac);

/**
 * gpon_hwaddr_resolve() - first usable candidate, else a random LAA.
 * @bootarg / @dt / @engine: NULL when this shell has no such source, or when it
 * rejected what it read (a fleet-wide bring-up default is the shell's fact).
 * Return: the rung that filled @out. Cannot fail, so no caller can skip the
 * random rung -- which is the step both Cortina copies omitted.
 */
enum gpon_hwaddr_src gpon_hwaddr_resolve(const char *bootarg, const u8 *dt,
					 const u8 *engine,
					 u8 out[GPON_HWADDR_BYTES]);

/** Return: the rung in words, for the probe log. */
const char *gpon_hwaddr_src_name(enum gpon_hwaddr_src src);

/**
 * gpon_hwaddr_derive() - a second station address @add above @base.
 *
 * A 48-bit ripple-carry add over the six bytes, so an offset that crosses an
 * octet boundary is still one address and not a corrupted one.  It is how
 * every stock firmware on this bench gives its WAN service netdev an identity
 * of its own -- the address the OLT and the ISP recognise the ONU by -- and
 * @add is a PRODUCT fact (3 on the RTL9602C X111W, 5 on the RTL9603CVD G24W),
 * never a family one.  @add == 0 copies @base, which is a real answer: stock's
 * own L2 nas0 presents the base address unchanged.
 *
 * REFUSES nothing; @out and @base may not be NULL and may be the same buffer.
 *
 * ⚠ rtl9602c_l34_logic.c:rtl9602c_wan_mac_add() IS THIS FUNCTION, under a chip
 * name, in an object gated on CONFIG_RTL9602C_ETH -- so the family's other
 * Ethernet shell cannot link it and the arithmetic came here rather than being
 * spelled a third time.  That copy is OWED A REBASE onto this one; it was left
 * standing only because rtl9602c_eth.c and the L34 sources were being edited
 * elsewhere in the same session.
 */
void gpon_hwaddr_derive(u8 out[GPON_HWADDR_BYTES],
			const u8 base[GPON_HWADDR_BYTES], unsigned int add);

#endif /* _GPON_HWADDR_H */
