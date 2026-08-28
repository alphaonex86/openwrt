// SPDX-License-Identifier: GPL-2.0-only
/* See cortina_gpon_logic.h. Moved verbatim; no line was rewritten.
 */
#include <linux/types.h>
#include <linux/errno.h>

#include "gpon_sn.h"

#include "cortina_gpon_logic.h"

/* One 32-bit register value from 4 wire-order bytes (endianness-agnostic). */
u32 cg_sn_word(const u8 *p)
{
	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

/*
 * ★★★ THE SERIAL-NUMBER CODEC MOVED TO THE COMMON CORE (2026-08-27), operator:
 * *"se deberia migrar ya todo los funciones del X400AXF a la familia ... el
 * resto no funciona y X400AXF fue muy verificado y funciona"*.
 *
 * The G.984.3 ONU-SN format is a SPEC, not a property of this silicon, so it
 * is decoded once in drivers/net/gpon/gpon_sn.c and both targets call it. The
 * implementation promoted there is THIS one -- it validated length, the vendor
 * characters and every hex digit, and refused a malformed string -- because
 * the Luna copy it replaces silently turned a bad digit into 0xff.
 *
 * These two remain as one-line shims ONLY so the ten call sites below and the
 * driver's -EINVAL contract are untouched by the move. Nothing else changed:
 * that is what makes this step verifiable on the board rather than argued.
 */
int cg_sn_parse(const char *s, u8 out[8])
{
	return gpon_sn_parse(s, out) ? -EINVAL : 0;
}
