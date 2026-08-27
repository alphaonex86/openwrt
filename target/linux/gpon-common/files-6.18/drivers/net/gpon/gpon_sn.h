/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_sn.h -- the ITU-T G.984.3 ONU Serial Number, decoded and encoded ONCE.
 *
 * ★★★ WHY THIS IS COMMON (operator, 2026-08-27: *"se deberia migrar ya todo
 * los funciones del X400AXF a la familia ... el resto no funciona y X400AXF
 * fue muy verificado y funciona"*, and *"codigo universal que funciona sin
 * depender de la arquitectura"*).
 *
 * The ONU-SN is 4 ASCII vendor characters followed by 4 bytes written as 8 hex
 * digits ("HWTC01234567"). That is a SPEC, not a silicon property: it is the
 * same on every chip, every vendor and every CPU, so two targets carrying two
 * decoders is two chances to disagree about what a serial number means -- and
 * they DID disagree, in the direction that hides:
 *
 *   Elnath / X400AXF (cortina-gpon.c, the VERIFIED board)
 *       length must be exactly 12, the four ID chars must be printable and
 *       non-space, every hex digit is checked, and a bad string is REFUSED.
 *   Luna / X111W (gpon-rtl9602c.c, the board that does not work)
 *       no length check, no character check, and hex_to_bin()'s -1 for a bad
 *       digit is assigned straight into a u8 -- so "oops" silently becomes
 *       0xff bytes and the ONU ranges under a serial nobody typed.
 *
 * The verified implementation is the one promoted here. The other is rebased
 * onto it rather than kept beside it.
 *
 * ★ ARCHITECTURE-INDEPENDENT BY CONSTRUCTION, and that is a requirement of
 * this tree, not a nicety: one image runs big-endian MIPS and little-endian
 * ARM64, so every byte is addressed EXPLICITLY. There is no cast over wire
 * bytes, no struct overlay, no host-order assumption -- and no <string.h>,
 * <ctype.h> or snprintf either, so the file compiles unchanged in the kernel
 * and on x86 for offline fuzzing.
 *
 * ★ IT OBEYS THE CORE CONTRACT: it DECIDES, it never DOES. No MMIO, no
 * readl/writel, no ioremap, no struct device, no allocation, no lock, no
 * sleeping, no clock, no jiffies. Time is not an input because there is none.
 */
#ifndef _GPON_SN_H
#define _GPON_SN_H

#include <linux/types.h>

/** Bytes in a G.984.3 ONU-SN. */
#define GPON_SN_BYTES		8
/** Characters in its printable form, excluding the NUL. */
#define GPON_SN_TEXT_LEN	12
/** Buffer a caller must provide to gpon_sn_format(): text + NUL. */
#define GPON_SN_TEXT_SIZE	(GPON_SN_TEXT_LEN + 1)

/**
 * gpon_sn_parse() - decode "AAAAhhhhhhhh" into the 8 SN bytes.
 * @s:   NUL-terminated candidate; may be NULL.
 * @out: receives GPON_SN_BYTES bytes. UNTOUCHED unless 0 is returned.
 *
 * Return: 0 when @s is a well-formed serial number, -1 otherwise.
 *
 * ★ IT REFUSES RATHER THAN GUESSING, and @out is left alone on refusal, so a
 *   rejected string can never half-overwrite a working identity.
 */
int gpon_sn_parse(const char *s, u8 out[GPON_SN_BYTES]);

/**
 * gpon_sn_format() - encode the 8 SN bytes as "AAAAhhhhhhhh".
 * @sn:  the bytes.
 * @out: at least GPON_SN_TEXT_SIZE bytes; always NUL-terminated.
 *
 * The hex digits are UPPER case, which is what the OLT prints and therefore
 * what a human compares against.
 */
void gpon_sn_format(const u8 sn[GPON_SN_BYTES], char *out);

#endif /* _GPON_SN_H */
