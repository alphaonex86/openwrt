/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpon_sn.h -- the ITU-T G.984.3 ONU Serial Number, decoded and encoded ONCE.
 *
 * The SN is 4 ASCII vendor characters then 4 bytes as 8 hex digits
 * ("HWTC01234567"): a SPEC, not a silicon property, so two decoders are two
 * chances to disagree -- and they DID, in the direction that hides.  Elnath
 * refused a malformed string; Luna had no length or character check and
 * assigned hex_to_bin()'s -1 straight into a u8, so "oops" silently became 0xff
 * bytes and the ONU ranged under a serial nobody typed.  The verified Elnath
 * implementation is the one promoted here (operator, 2026-08-27: *"codigo
 * universal que funciona sin depender de la arquitectura"*).
 *
 * Every byte is addressed EXPLICITLY -- no cast over wire bytes, no struct
 * overlay, no host-order assumption, and no <string.h>/<ctype.h>/snprintf -- so
 * one source compiles on big-endian MIPS, little-endian ARM64 and x86.
 * It DECIDES and never DOES: no MMIO, allocation, lock, sleep or clock.
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
 * @out: receives GPON_SN_BYTES bytes. UNTOUCHED unless 0 is returned, so a
 *       rejected string can never half-overwrite a working identity.
 *
 * Return: 0 when @s is a well-formed serial number, -1 otherwise.
 */
int gpon_sn_parse(const char *s, u8 out[GPON_SN_BYTES]);

/**
 * gpon_sn_format() - encode the 8 SN bytes as "AAAAhhhhhhhh".
 * @sn:  the bytes.
 * @out: at least GPON_SN_TEXT_SIZE bytes; always NUL-terminated.
 *
 * UPPER-case hex: what the OLT prints, and therefore what a human compares.
 */
void gpon_sn_format(const u8 sn[GPON_SN_BYTES], char *out);

/**
 * gpon_sn_is_set() - has a serial number been provisioned at all?
 * @sn: the 8 bytes; may be NULL.
 *
 * Return: true only when NEITHER 4-byte half is entirely 0x00 or entirely 0xff.
 *
 * ★★★ RANGING IS PERMITTED ONLY ONCE THE SERIAL IS DEFINED (operator,
 * 2026-09-10: *"tiene que permitir rangear una vez el serial definido"*), and
 * this is the predicate that decides "defined" -- a pure question over eight
 * bytes, so every family asks it instead of keeping a copy.
 *
 * ★★ A HALF-BLANK SERIAL IS THE ONE THAT ACTUALLY SHIPS.  "Any byte non-zero"
 * was the whole test until 2026-09-10 and the X400AXF drove through it: on a
 * 60 s provisioning timeout the Cortina driver installed
 * { 'X','P','O','N', 0xff,0xff,0xff,0xff } and STARTED RANGING.  The vendor-id
 * half is a fleet constant a driver may legitimately hold while the
 * vendor-specific half is the per-unit value read off the board, so half an
 * identity is exactly the shape a provisioning failure takes.  0xff is the
 * blank-flash/unburnt-efuse value; 0x00 is refused on the same argument, since
 * a half nobody programmed cannot be told from a half somebody chose.
 *
 * ★★ ALL-ZERO MEANS "NOBODY HAS TOLD THIS ONU WHO IT IS", and such an ONU may
 * not announce itself -- a Serial_Number_ONU carrying a placeholder is a second
 * ONU on the PON wearing somebody's identity.  MEASURED 2026-09-06 on the LANLY
 * G24W: the Luna driver compiled in one board's real serial as its default, so
 * a SECOND board ranged under the FIRST board's serial on the same splitter,
 * and the OLT listed the impostor's own serial nowhere.  G.984.3 has no
 * "unknown serial" value, so the only safe encoding of "not provisioned" is
 * "do not transmit".
 */
bool gpon_sn_is_set(const u8 sn[GPON_SN_BYTES]);

#endif /* _GPON_SN_H */
