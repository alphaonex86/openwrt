/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gpon_unsup.h — REPORT ANYTHING THE FIRMWARE DID NOT UNDERSTAND, WITHOUT
 * FLOODING, AND CARRY ENOUGH OF IT TO IMPLEMENT SUPPORT.
 *
 * TIER: see "THE THREE TIERS" in gpon_common.h — that is the canonical copy of
 * the rule and it is not restated here.  What IS specific to this file is the
 * paragraph below, because this header sits in the CORE directory and is the
 * one thing in it that mentions pr_*.
 *
 * ★★ WHY A pr_* MAY LIVE IN THIS DIRECTORY, AND WHY THAT IS NOT A HOLE IN THE
 *    PURITY RULE.  Read this before "fixing" it.
 *      - The rule binds the CORE OBJECTS: every .c built by this directory's
 *        Makefile decides and never does, and must never gain an MMIO access, a
 *        clock, a lock, an allocator or a log line.
 *      - The guard that enforces it is a COMPILER, not a grep:
 *        dev/rtl9607c-test/gpon_layer_hostbuild_test.sh compiles this
 *        directory's sources against dev/rtl9607c-test/fuzz_shims, which
 *        deliberately declare no register accessor, no clock, no lock and no
 *        allocator.  It globs `*.c` (verified: gpon_layer_hostbuild_test.sh,
 *        the loop over its GPON_DIR that ends in dot-c), so a HEADER is not a
 *        subject of that gate — and it must not become one, because a header is
 *        not an object and compiles into nothing on its own.
 *        (That sentence deliberately does not quote the glob: a slash followed
 *        by a star inside a block comment is a nested-comment warning under
 *        -Wcomment, and the same class of stray terminator has already closed a
 *        comment early once in this directory.)
 *      - Nothing in this directory includes this file.  It is included by
 *        SHELLS (drivers/net/ethernet/{realtek,cortina}/), which are allowed
 *        hardware, allowed logs, and are where the kernel log actually is.  The
 *        core reaches the same reporting through ->unsupported in
 *        gpon_common.h, which carries no printk at all.
 *      - It is authored HERE rather than once per target for the same reason
 *        the protocol is: two copies of a witness is one copy that drifts, and
 *        the reader on the other side of this line (unsup_scan.py) parses ONE
 *        spelling.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * THE RULE (operator, 2026-08-20)
 * ──────────────────────────────────────────────────────────────────────────
 *   *"si el codigo recive algo desconocido o fuera de rango tiene que informar
 *   (sin flood) por dmesg y el python tiene que detectarlo y informar por log y
 *   console, es tanto una logica sana de diagnostico que una forma de hacer el
 *   support de nuevos OLT si el message de error hacer un dump sin saturar el
 *   dmesg (info minimal)"*
 *
 * Two purposes, and the second is why the DUMP is mandatory rather than nice:
 *
 *   1. DIAGNOSIS.  The worked example is the alloc-CAM wall on the X111W
 *      (2026-08-20): the driver was handed upstream BWmap grants for T-CONTs it
 *      had never configured, and it said NOTHING — it used them.  The symptom
 *      surfaced weeks later at the OLT as `LOAi`, three layers from the cause.
 *      ONE rate-limited line naming the value it could not place would have
 *      ended it in one boot.
 *   2. SUPPORTING A NEW OLT.  What a foreign vendor's OLT sends and we do not
 *      model IS an "unknown" report — and because the line carries a MINIMAL
 *      DUMP, the report is the SPECIFICATION for implementing it.  Without the
 *      dump you learn that something happened; with it you can build it.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * THE LINE — ONE SPELLING, FIXED BEFORE THIS HEADER WAS WRITTEN
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   <subsys>: UNSUP kind=<slug> class=<unknown|range> val=<v> want=<text> n=<c> d=<hex>
 *
 * The READER is dev/ONU-test-case/unsup_scan.py and its `MARKER` /  `_FIELD`
 * regexes are the contract.  Two consequences that are easy to get wrong:
 *
 *   ⚠ `want=` IS PARSED UP TO THE FIRST SPACE (`\bwant=(\S+)`).  A want string
 *     containing a space is TRUNCATED at the space and the rest is silently
 *     lost.  Every want token here is therefore space-free — hyphens and
 *     underscores, never words separated by blanks.
 *   ⚠ `class=` IS A CLOSED SET.  A class the reader cannot place is carried as
 *     MALFORMED rather than guessed into a bucket, so this file renders an
 *     out-of-enum class as the deliberately-unplaceable "unclassified" instead
 *     of defaulting it to "unknown" — defaulting would file a possible DEFECT
 *     as support work, which is the reassuring direction.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * "WITHOUT FLOODING" IS A REQUIREMENT, AND pr_*_ratelimited DOES NOT MEET IT
 * ──────────────────────────────────────────────────────────────────────────
 * This prints occurrence 1, 2, 4, 8, 16, 32 … PER SITE, so a flood of N costs
 * log2(N) lines, and `n=` still carries the TRUE cumulative count: suppression
 * must never HIDE.  The kernel's own pr_*_ratelimited is not enough for either
 * half — it DROPS lines within a window and tells nobody how many, and its
 * budget is per call site in a shared token bucket, so a noisy old site can
 * exhaust the budget a NEW site needed.  Here the counter is per site and the
 * `kind` is the aggregation key on the reader's side, so a new kind can never
 * be buried under an old kind's flood.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * WHY THE HEX IS RENDERED HERE AND NOT WITH THE KERNEL'S %*phN
 * ──────────────────────────────────────────────────────────────────────────
 * %*phN would be free (vsnprintf already carries it) and it is what this
 * driver's existing UNHANDLED line uses.  It is not used here because it is a
 * KERNEL vsnprintf extension: on the x86 host the same format string renders a
 * POINTER followed by the literal "hN", so the offline case that proves the
 * emitted line matches the reader would be proving a shim's emulation instead
 * of the shipped code.  Rendering it in code makes ONE spelling that both the
 * kernel and the host compile — which is the same reason the marker itself was
 * fixed in one place before either half was written.
 * MEASURED cost of that choice, from realtek-luna's OWN cross compiler at its
 * own flags -- re-take it with `dev/rtl9607c-test/gpon_unsup_size.sh`, which
 * prices BOTH spellings side by side rather than asserting either:
 *   this header, 11 sites   728 bytes .text  (368 once + 32 per site)
 *   %*phN,       11 sites  1948 bytes .text  (nothing to call, so gcc inlines
 *                                             the backoff and the printk into
 *                                             every site)
 * i.e. the spelling that is also the TESTABLE one is the smaller one here.
 * That was not the reason for the choice and it is not an argument for it --
 * it is the number the choice is owed.
 */
#ifndef GPON_UNSUP_H
#define GPON_UNSUP_H

#include <linux/types.h>
#include <linux/printk.h>

#include "gpon_common.h"	/* enum gpon_unsup_class — ONE spelling */

/*
 * The subsystem prefix the line opens with.  DEFINE IT BEFORE INCLUDING THIS
 * HEADER, to whatever the file's existing log lines already use, so the new
 * lines sit beside the old ones in dmesg:
 *
 *   #define GPON_UNSUP_SUBSYS "rtl9602c-gpon"
 *   #include "gpon_unsup.h"
 *
 * The fallback is deliberately generic and deliberately not an #error: a shell
 * that forgets it still reports, and a grep for the token still finds it.
 */
#ifndef GPON_UNSUP_SUBSYS
#define GPON_UNSUP_SUBSYS	"gpon"
#endif

/*
 * `noinline` for the HOST builds only.
 *
 * In kernel context this is already a macro (compiler_attributes.h is force
 * -included by the kernel build), so the #ifndef is false and the kernel's own
 * definition stands untouched.  The host builds are what need it — neither the
 * offline fuzz shims nor a plain libc provide it — and this header must compile
 * on all three toolchains.  Same idiom, and for the same reason, as the
 * `fallthrough` shim at the top of gpon_common.h.
 *
 * ★ IT IS THERE FOR FOOTPRINT, AND THE NUMBER IS MEASURED, NOT ASSUMED.
 *   gpon_unsup_emit() is a `static inline` in a header, so without this gcc
 *   inlines the whole body — the backoff test, the hex loop and the eight-
 *   argument printk — into EVERY call site.  Measured with realtek-luna's own
 *   cross compiler at its own flags (-Os -O2, gpon_unsup_size.sh):
 *
 *       inlined     164 B once + 228 B per site  = 2672 B at 11 sites
 *       out-of-line 368 B once +  32 B per site  =  728 B at 11 sites
 *
 *   i.e. 1944 bytes saved on a 3 MB NAND kernel partition, for one attribute,
 *   on code that runs once per unmodelled event and is on no per-packet path.
 *   Re-take the figures with ./gpon_unsup_size.sh after any change here — they
 *   are MEASURED, and a remembered number is the thing this project keeps
 *   catching itself publishing.
 */
#ifndef noinline
# if defined(__GNUC__)
#  define noinline	__attribute__((__noinline__))
# else
#  define noinline
# endif
#endif

/*
 * `__maybe_unused` for the HOST builds only, same idiom as above.
 *
 * ★ IT IS REQUIRED, NOT DEFENSIVE, AND -Werror IS WHY.  gpon_unsup_emit() below
 *   is a plain `static` and NOT a `static inline`: writing `static inline` and
 *   then asking for `noinline` is a contradiction gcc reports as
 *   -Wattributes ("inline function given attribute noinline"), and both target
 *   kernels build with CONFIG_WERROR=y, so that warning is a hard build failure
 *   on the board and merely a message on a host build without -Werror.  A plain
 *   `static` that a translation unit includes without calling then trips
 *   -Wunused-function instead — which -Wall turns on and -Werror makes fatal
 *   too.  This attribute is what closes both, and the strict-flags probe in the
 *   gpon_unsup_test recipe (dev/rtl9607c-test/gpon_x86_harness.mk) compiles this
 *   header with exactly those flags and no call site, so the trap cannot come
 *   back silently.
 */
#ifndef __maybe_unused
# define __maybe_unused	__attribute__((__unused__))
#endif

/*
 * The dump ceiling, in OCTETS.  16 covers every datum this facility currently
 * reports — the longest is a 13-octet downstream PLOAM — and it bounds the
 * stack buffer below at 33 bytes, which is what keeps this usable from a timer
 * callback.  "Minimal dump" is the operator's own wording: enough to implement
 * support, short enough not to saturate the log.  A caller passing more is
 * CLIPPED, never truncated silently in the middle of a byte.
 */
#define GPON_UNSUP_DUMP_MAX	16u

/*
 * The class name, as the reader spells it.
 *
 * ★ AN OUT-OF-ENUM CLASS RENDERS AS "unclassified", NOT AS "unknown".  The op
 *   in gpon_common.h widens the class to int, so a caller CAN pass something
 *   that is neither; mapping that to "unknown" would file a possible defect as
 *   support work.  "unclassified" is outside the reader's closed set, so the
 *   report is carried as MALFORMED and stays visible — this file refusing to
 *   guess, exactly as unsup_scan.py refuses on the other side.
 */
static inline const char *gpon_unsup_class_name(int cls)
{
	switch (cls) {
	case GPON_UNSUP_UNKNOWN:
		return "unknown";
	case GPON_UNSUP_RANGE:
		return "range";
	default:
		return "unclassified";
	}
}

/*
 * Emit one report, or suppress it.
 *
 * @n is the caller's PER-SITE cumulative counter, supplied by the macro below;
 * it is incremented on EVERY occurrence and printed on the powers of two, so
 * the printed `n=` is the true count and the reader's max() over sightings is
 * the true total.
 *
 * Not `__printf`-checked on purpose: the format is fixed here and there is
 * exactly one call to pr_info, so there is no caller-supplied format to check.
 */
static noinline __maybe_unused void gpon_unsup_emit(const char *subsys,
					    const char *kind,
					    int cls, u32 val,
					    const char *want, const u8 *data,
					    unsigned int len, unsigned int *n)
{
	static const char hexd[] = "0123456789abcdef";
	char hex[2u * GPON_UNSUP_DUMP_MAX + 1u];
	unsigned int i, c;

	c = ++(*n);
	if (c & (c - 1u))		/* not a power of two -> suppressed */
		return;

	if (!data)
		len = 0;
	if (len > GPON_UNSUP_DUMP_MAX)
		len = GPON_UNSUP_DUMP_MAX;
	for (i = 0; i < len; i++) {
		hex[2u * i]      = hexd[(data[i] >> 4) & 0xfu];
		hex[2u * i + 1u] = hexd[data[i] & 0xfu];
	}
	hex[2u * len] = '\0';

	pr_info("%s: UNSUP kind=%s class=%s val=0x%x want=%s n=%u d=%s\n",
		subsys, kind, gpon_unsup_class_name(cls), val,
		want ? want : "-", c, hex);
}

/*
 * THE ONE THING A SHELL CALLS.
 *
 *   gpon_unsup_report("ds_ploam_type", GPON_UNSUP_UNKNOWN, type,
 *                     "G.984.3-DS-types-we-model", m, GPON_PLOAM_DS_LEN);
 *
 * It is a MACRO and not a function for exactly one reason: the rate-limit
 * counter must be PER SITE, and a shared function cannot have one.  `static`
 * inside the macro body gives each expansion its own 4-byte .bss counter, which
 * is also what makes `kind` the rate-limit key in practice — two sites with the
 * same kind still back off independently, and a new site is never born already
 * suppressed by an old one's flood.
 *
 * @kind  a STABLE slug, lower_snake_case.  It is the aggregation key on the
 *        reader's side, so renaming one splits its history.
 * @cls   GPON_UNSUP_UNKNOWN (support work) or GPON_UNSUP_RANGE (a finding).
 * @val   the offending value; printed as 0x%x.
 * @want  the DECLARED domain, SPACE-FREE (see the note at the top).  NULL
 *        prints "-".
 * @data  the minimal dump, @len octets, clipped to GPON_UNSUP_DUMP_MAX.  NULL
 *        or 0 is legal and prints an empty d=.
 */
#define gpon_unsup_report(kind, cls, val, want, data, len)		\
	do {								\
		static unsigned int _gpon_unsup_n;			\
									\
		gpon_unsup_emit(GPON_UNSUP_SUBSYS, (kind), (int)(cls),	\
				(u32)(val), (want), (const u8 *)(data),	\
				(unsigned int)(len), &_gpon_unsup_n);	\
	} while (0)

#endif /* GPON_UNSUP_H */
