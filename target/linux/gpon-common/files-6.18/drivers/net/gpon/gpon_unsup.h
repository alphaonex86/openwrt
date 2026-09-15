/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gpon_unsup.h — REPORT ANYTHING THE FIRMWARE DID NOT UNDERSTAND, WITHOUT
 * FLOODING, AND CARRY ENOUGH OF IT TO IMPLEMENT SUPPORT (operator, 2026-08-20).
 * The dump is mandatory rather than nice because of the second purpose: with it
 * the report IS the specification for supporting what arrived.  Without it, the
 * X111W alloc-CAM wall used grants for T-CONTs it had never configured, said
 * nothing, and surfaced weeks later at the OLT as LOAi.
 *
 * TIER: see "THE THREE TIERS" in gpon_common.h.
 * ★★ WHY A pr_* MAY LIVE HERE — read before "fixing" it.  The purity rule binds
 *    the core OBJECTS and its guard is a COMPILER: gpon_layer_hostbuild_test.sh
 *    compiles this directory's dot-c sources against stubs declaring no
 *    accessor, clock, lock or allocator.  A HEADER is not an object.  Nothing
 *    here includes this file -- SHELLS do, and they are allowed logs; the core
 *    reaches the same reporting through ->unsupported in gpon_common.h.
 *
 * THE LINE — ONE SPELLING, and dev/ONU-test-case/unsup_scan.py's regexes are
 * the contract:
 *
 *   <subsys>: UNSUP kind=<slug> class=<unknown|range> val=<v> want=<text> n=<c> d=<hex>
 *
 * ⚠ `want=` IS PARSED UP TO THE FIRST SPACE, so every want token is space-free.
 * ⚠ `class=` IS A CLOSED SET: an out-of-enum class renders "unclassified", not
 *   "unknown" — defaulting would file a possible DEFECT as support work, the
 *   reassuring direction.
 *
 * "WITHOUT FLOODING", AND pr_*_ratelimited DOES NOT MEET IT.  This prints
 * occurrence 1, 2, 4, 8 … PER SITE, so a flood of N costs log2(N) lines and
 * `n=` still carries the TRUE count: suppression must never HIDE.  The kernel's
 * version drops lines within a window telling nobody how many, and shares one
 * token bucket, so a noisy old site can exhaust a NEW site's budget.
 *
 * The hex is rendered here and not with %*phN because that is a KERNEL vsnprintf
 * extension: on x86 the same format renders a POINTER followed by "hN", so the
 * offline case would prove a shim's emulation instead of the shipped code.
 * MEASURED with realtek-luna's own cross compiler and flags (re-take with
 * dev/rtl9607c-test/gpon_unsup_size.sh): this spelling 728 B .text at 11 sites,
 * %*phN 1948 B.
 */
#ifndef GPON_UNSUP_H
#define GPON_UNSUP_H

#include <linux/types.h>
#include <linux/printk.h>
#include "gpon_common.h"	/* GPON_UNSUP_RANGE, GPON_UNSUP_UNKNOWN */

/* The subsystem prefix the line opens with.  DEFINE IT BEFORE INCLUDING THIS
 * HEADER, to whatever the file's existing log lines already use.  The fallback
 * is deliberately not an #error: a shell that forgets it still reports. */
#ifndef GPON_UNSUP_SUBSYS
#define GPON_UNSUP_SUBSYS	"gpon"
#endif

/* HOST builds only; in kernel context compiler_attributes.h defines both.
 * ★ noinline IS FOR FOOTPRINT, MEASURED (gpon_unsup_size.sh): inlined, the
 *   backoff + hex loop + eight-argument printk cost 164 B once + 228 B per site
 *   = 2672 B at 11 sites, against 368 B + 32 B = 728 B out of line.
 * ★ __maybe_unused IS REQUIRED, NOT DEFENSIVE: below is a plain `static`, NOT
 *   `static inline` (inline plus noinline is -Wattributes and both kernels set
 *   CONFIG_WERROR=y), and a plain `static` a TU includes without calling trips
 *   -Wunused-function.  gpon_x86_harness.mk compiles this header at exactly
 *   those flags with no call site. */
#ifndef noinline
# if defined(__GNUC__)
#  define noinline	__attribute__((__noinline__))
# else
#  define noinline
# endif
#endif
#ifndef __maybe_unused
# define __maybe_unused	__attribute__((__unused__))
#endif

/* The dump ceiling in OCTETS.  16 covers every datum this facility reports (the
 * longest is a 13-octet downstream PLOAM) and bounds the stack buffer below at
 * 33 bytes, which is what keeps this usable from a timer callback.  A caller
 * passing more is CLIPPED, never truncated mid-byte. */
#define GPON_UNSUP_DUMP_MAX	16u

/* The class name as the reader spells it; out-of-enum is deliberately
 * unplaceable, so the report is carried as MALFORMED and stays visible. */
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

/* Emit one report, or suppress it.  @n is the caller's PER-SITE cumulative
 * counter: incremented on EVERY occurrence and printed on the powers of two, so
 * the printed `n=` is the true count.  Not __printf-checked on purpose — the
 * format is fixed here and there is exactly one pr_info. */
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

/* THE ONE THING A SHELL CALLS:
 *
 *   gpon_unsup_report("ds_ploam_type", GPON_UNSUP_UNKNOWN, type,
 *                     "G.984.3-DS-types-we-model", m, GPON_PLOAM_DS_LEN);
 *
 * A MACRO and not a function for exactly one reason: the rate-limit counter
 * must be PER SITE, and a shared function cannot have one.  `static` inside the
 * macro body gives each expansion its own 4-byte .bss counter, so a new site is
 * never born already suppressed by an old one's flood.
 * @kind  a STABLE slug, lower_snake_case: it is the reader's aggregation key,
 *        so renaming one splits its history.
 * @cls   GPON_UNSUP_UNKNOWN (support work) or GPON_UNSUP_RANGE (a finding).
 * @want  the DECLARED domain, SPACE-FREE (see above).  NULL prints "-".
 * @data  the minimal dump, @len octets, clipped to GPON_UNSUP_DUMP_MAX. */
#define gpon_unsup_report(kind, cls, val, want, data, len)		\
	do {								\
		static unsigned int _gpon_unsup_n;			\
									\
		gpon_unsup_emit(GPON_UNSUP_SUBSYS, (kind), (int)(cls),	\
				(u32)(val), (want), (const u8 *)(data),	\
				(unsigned int)(len), &_gpon_unsup_n);	\
	} while (0)

#endif /* GPON_UNSUP_H */
