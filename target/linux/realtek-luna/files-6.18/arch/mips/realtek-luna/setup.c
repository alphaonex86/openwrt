// SPDX-License-Identifier: GPL-2.0-only
/*
 * "Luna" GPON ONU (RTL960xC, RLX/Taroko core) — platform setup.
 *
 * Independent implementation from the SoC's register interface (register bases,
 * reset and watchdog programming) and mainline MIPS DT-platform conventions.
 * The generic arch/mips device_tree_init() (unflatten) is used as-is.
 * Interrupts are handled by the SoC INTC (irqchip driver) via the standard
 * irqchip_init() entry; the system tick comes from the SoC TC timer (clocksource
 * driver) because the Taroko core's CP0 Count is unreliable.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/bits.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/of_clk.h>
#include <linux/of_fdt.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/cpu-features.h>
#include <asm/cpu-info.h>
#include <asm/mips-cps.h>
#include <asm/mipsregs.h>
#include <asm/prom.h>
#include <asm/reboot.h>
#include <asm/smp-ops.h>
#include <asm/time.h>

/*
 * SoC watchdog timer (KSEG1), TC block. Forcing a WDT timeout is the only reset
 * that actually re-enters the boot ROM: the control register's RESET_MODE=0 is a
 * H/W full-chip reset (resets the CPU AND the PLL/analog/SerDes domains, ~= a
 * power-on reset). Poking the swcore software-reset register (0x1b0000e0 bit7)
 * only resets the switch core, leaving the CPU spinning -> the board hangs and
 * never reboots, which also defeats the GPON WAN cold-start auto-recovery
 * (a clean reboot is what re-rolls the non-deterministic upstream analog lock).
 *
 * BSP_WDTCTRLR @0x18003268: [31] enable, [30:29] clk-scale (0=2^25 .. 3=2^28
 * LX clocks/unit), [26:22] phase-1 timeout (5b), [19:15] phase-2 timeout (5b),
 * [1:0] reset-mode (0=full chip, 1=CPU+IPSec, 2=S/W). Kick reg @0x18003260.
 *
 * ★ THE SAME BLOCK IS ALSO DRIVEN BY drivers/watchdog/luna_wdt.c, which is
 * what serves /dev/watchdog.  This path stays where it is -- it is the proven
 * one, it is what _machine_restart needs, and it must work with no driver bound
 * -- but that means one piece of hardware is described in two files.  The
 * offline case dev/rtl9607c-test/luna_wdt_test asserts that the two agree, so a
 * shift fixed in one place cannot silently rot in the other.
 *
 * ★ 2026-08-29 -- THE PREFIX IS LUNA_RSTWDT_, NOT LUNA_WDT_, AND THE DIFFERENCE
 * IS REAL.  When the family prefix luna_ was renamed to luna_, the driver's
 * LUNA_WDT_CTRL (an OFFSET, 0x08 from the DT base) and this file's
 * LUNA_WDT_CTRL (an ABSOLUTE mapped pointer, 0x18003268) collided on one name
 * with two values.  Same hardware, two views, and a reader who conflated them
 * would write an offset where a pointer is wanted.  The reset path keeps its
 * own prefix so the ambiguity cannot exist; the guard still holds the SHIFTS
 * equal, which is the part that must not drift.
 */
#define LUNA_RSTWDT_CTRL		((void __iomem *)CKSEG1ADDR(0x18003268))
#define LUNA_RSTWDT_EN		BIT(31)		/* watchdog enable          */
#define LUNA_RSTWDT_CLK_SC_SHIFT	29		/* overflow scale           */
#define LUNA_RSTWDT_PH1_TO_SHIFT	22		/* phase-1 timeout          */
#define LUNA_RSTWDT_PH2_TO_SHIFT	15		/* phase-2 timeout          */
#define LUNA_RSTWDT_RST_FULLCHIP	0u		/* RESET_MODE = full chip   */

extern char __dtb_start[];
void prom_putchar(char c);	/* bring-up bisect markers (remove later) */

static void luna_machine_restart(char *command)
{
	local_irq_disable();
	pr_emerg("Restarting via SoC watchdog full-chip reset...\n");
	/*
	 * Full-chip reset, fastest scale (2^25 LX clocks ~ 0.17s), phase-1=1,
	 * phase-2=0, enabled; do NOT kick it -> it times out almost immediately
	 * and the boot ROM takes over. Spin until the reset lands.
	 */
	__raw_writel(LUNA_RSTWDT_EN |
		     (0u << LUNA_RSTWDT_CLK_SC_SHIFT) |
		     (1u << LUNA_RSTWDT_PH1_TO_SHIFT) |
		     (0u << LUNA_RSTWDT_PH2_TO_SHIFT) |
		     LUNA_RSTWDT_RST_FULLCHIP,
		     LUNA_RSTWDT_CTRL);
	while (1)
		cpu_relax();
}

static void luna_machine_halt(void)
{
	local_irq_disable();
	pr_emerg("System halted.\n");
	while (1)
		cpu_relax();
}

#ifdef CONFIG_MIPS_CM
/*
 * The on-chip L2 (256 KB, 8-way, 32-byte lines) comes out of the boot loader
 * with UNINITIALIZED tags. This SoC reports the L2 line size as 0 in Config2
 * (the geometry is fixed in silicon, not described), so the generic MIPS cache
 * code never sizes or initializes the L2; the first cached write-back to a DRAM
 * page that maps onto a garbage-tagged set/way -- the very top of usable DRAM,
 * touched first by setup_zero_pages() -- then wedges. Invalidate every L2 line
 * with Index_Store_Tag (writing a zeroed, invalid tag; NO write-back, so no
 * garbage is flushed to memory). Run this very early, while the boot loader's
 * CCA override still keeps DRAM out of the L2, so no live kernel data is lost.
 */
#define LUNA_L2_SIZE	(256 << 10)	/* 256 KB */
#define LUNA_L2_LINE	32		/* bytes (Config2 SL reads 0 on this SoC) */

/*
 * THE ON-CHIP L2 IS HIDDEN AT RESET ON THE RTL9603CVD, AND THE VENDOR UN-HIDES
 * IT BEFORE THE KERNEL EVER PROBES A CACHE.
 *
 * Tier 1, this board's own vendor kernel on its own console (dev/lanly.log,
 * LANLY G24W, captured 2026-08-18), three consecutive lines:
 *
 *	II: Original CP0 CONFIG2: 80001307
 *	II: _enable_l23
 *	II: Configured CP0 CONFIG2: 80000347
 *	...
 *	MIPS secondary cache 128kB, 8-way, linesize 32 bytes.
 *
 * Decoding MIPS32 Config2 (SU [15:12], SS [11:8], SL [7:4], SA [3:0]):
 *
 *	reset      0x80001307  SU=1  SS=3  SL=0  SA=7   <- SL=0 means NO L2
 *	programmed 0x80000347  SU=0  SS=3  SL=4  SA=7   <- 64<<3 sets x 2<<4
 *	                                                   bytes x (7+1) ways
 *	                                                   = 512 x 32 x 8 = 128 KB
 *
 * and 128 KB / 8-way / 32-byte is exactly what that kernel then prints. So on
 * this silicon the L2 geometry fields read as ABSENT until the bypass bit is
 * cleared -- which is why generic `mips_sc_probe()` finds no secondary cache on
 * our image and the kernel then runs with the L2 bypassed.
 *
 * ★ THE READ IS DONE FIRST AND THE RESULT IS REPORTED. Only bit 12 is cleared
 *   here, deliberately: whether the geometry fields then read out by themselves
 *   is a question about the silicon, and the answer decides whether this
 *   function ever has to spell 0x347 -- a per-board constant -- at all. It is
 *   asked rather than assumed.
 */
#define LUNA_CONF2_L2_BYPASS	BIT(12)
#define LUNA_CONF2_SL_SHIFT	4
#define LUNA_CONF2_SL_MASK	(0xfu << LUNA_CONF2_SL_SHIFT)

/*
 * ★★★ NOTHING MAY TOUCH DRAM BETWEEN THE UN-BYPASS AND THE TAG INVALIDATE, AND
 * THAT IS A MEASUREMENT, NOT A STYLE RULE (G24W, 2026-08-20).
 *
 * The un-bypass makes the L2 live while its tags are still whatever the boot
 * loader left; `luna_l2_invalidate_tags()` below is what makes them safe. A
 * `pr_info()` placed between the two -- announcing the very fix -- hung the
 * board HARDER than the bug it was reporting: the boot stopped dead at `[M]`,
 * i.e. earlier than the original defect, because printk's ring buffer is a
 * large cached DRAM write and it landed on garbage-tagged L2 lines.
 *
 * So this records what it saw and says nothing. The value is REPORTED LATER,
 * from device_tree_init(), which is well past the invalidate. `luna_conf2_reset`
 * is written BEFORE the bypass bit is cleared -- while the L2 is still out of
 * the way -- for the same reason.
 */
static unsigned int luna_conf2_reset __initdata;
static unsigned int luna_conf2_now __initdata;

static void __init luna_l2_unbypass(void)
{
	unsigned int c2 = read_c0_config2();

	luna_conf2_reset = c2;
	if (c2 & LUNA_CONF2_L2_BYPASS) {
		write_c0_config2(c2 & ~LUNA_CONF2_L2_BYPASS);
		back_to_back_c0_hazard();
	}
	luna_conf2_now = read_c0_config2();
}

static void __init luna_l2_invalidate_tags(void)
{
	unsigned long addr;

	/* Zero the L2 tag/data shadow registers -> Index_Store_Tag writes an
	 * invalid tag. (CP0 $28 sel 4/5, $29 sel 5 = L2 TagLo/DataLo/DataHi.) */
	__asm__ __volatile__(
		"	.set	push		\n"
		"	.set	noreorder	\n"
		"	mtc0	$0, $28, 4	\n"
		"	mtc0	$0, $28, 5	\n"
		"	mtc0	$0, $29, 5	\n"
		"	.set	pop		\n"
		::: "memory");

	for (addr = CKSEG0; addr < CKSEG0 + LUNA_L2_SIZE; addr += LUNA_L2_LINE)
		__asm__ __volatile__(
			"	.set	push		\n"
			"	.set	noreorder	\n"
			"	cache	0x0b, 0(%0)	\n"	/* Index_Store_Tag_S */
			"	.set	pop		\n"
			:: "r" (addr) : "memory");

	__asm__ __volatile__("sync" ::: "memory");
}

/*
 * UserLocal / thread-pointer (TLS) enable.
 *
 * The C library reads the per-thread pointer with `rdhwr $29` (hardware
 * register 29 = UserLocal). That instruction runs in user mode only when
 * CP0 HWREna[29] is set, and the generic trap init sets it only for a CPU
 * whose feature set records UserLocal (Config3.ULRI). If the per-CPU probe
 * did not record it, HWREna[29] stays clear, the very first TLS read in the
 * dynamic loader traps as a Reserved Instruction, and PID 1 dies with SIGILL.
 *
 * This core implements UserLocal, so trust the architectural Config3.ULRI bit
 * directly and record the option before per_cpu_trap_init() programs HWREna.
 * If the register really were absent (ULRI == 0) we leave the feature off and
 * the kernel's rdhwr emulation handles the trap instead. Runs after cpu_probe()
 * and well before trap_init(), in setup_arch()'s device_tree_init().
 */
static void __init luna_enable_userlocal(void)
{
	unsigned int cfg3 = read_c0_config3();

	/* M1 bring-up diagnostic: show how the CPU was identified (remove later). */
	pr_emerg("LUNA-DIAG: prid=%08x cputype=%d config3=%08x ULRI=%d userlocal=%d mmips=%d\n",
		 read_c0_prid(), current_cpu_type(), cfg3,
		 !!(cfg3 & MIPS_CONF3_ULRI),
		 cpu_has_userlocal ? 1 : 0, cpu_has_mmips ? 1 : 0);

	if (cfg3 & MIPS_CONF3_ULRI)
		current_cpu_data.options |= MIPS_CPU_ULRI;

	/*
	 * ★ THE L2's OWN WITNESS, emitted here rather than where the work was
	 *   done -- see luna_l2_unbypass() for why that window must stay silent.
	 *   It prints the VALUES rather than a conclusion, so a silicon revision
	 *   that behaves differently is readable instead of merely broken.
	 *   MEASURED on the G24W: 80001307 -> 80000347, after which the generic
	 *   probe prints "MIPS secondary cache 128kB, 8-way, linesize 32 bytes"
	 *   -- the same line, byte for byte, that this board's vendor kernel
	 *   prints.
	 */
	if (luna_conf2_reset != luna_conf2_now)
		pr_info("rtl960x: L2 un-bypassed: Config2 %08x -> %08x\n",
			luna_conf2_reset, luna_conf2_now);
}
#endif /* CONFIG_MIPS_CM */

void __init plat_mem_setup(void)
{
	prom_putchar('['); prom_putchar('M'); prom_putchar(']');
#ifdef CONFIG_MIPS_CM
	luna_l2_unbypass();		/* the L2 is hidden at reset on the 9603CVD */
	luna_l2_invalidate_tags();	/* clean the boot-time garbage L2 tags */
	/*
	 * ★ THE TOP-OF-DRAM PROBE THAT USED TO RUN HERE IS GONE, and removing it
	 * is a FIX, not tidying (measured 2026-08-20 on the LANLY G24W).
	 *
	 * It was a declared bring-up diagnostic ("remove later") that wrote and
	 * read phys 0x11fff000 -- 288 MB, the RTL9607C engineering board's DRAM
	 * top -- through KSEG1 and then through KSEG0. That address is a BOARD
	 * fact spelled as a constant in code shared by every member of this
	 * subtarget, and the G24W has 128 MB (0x08000000): measured at its own
	 * U-Boot prompt, `bdinfo` -> memstart 0x80000000, memsize 0x08000000,
	 * and its own kernel's DEBUG_MEM_AUTO says mem_size=0x08000000.
	 *
	 * The failure was silent and in the reassuring direction. Unmapped DRAM
	 * ALIASES here, so the probe printed its full success sequence
	 * ("P123deadbeef4560000cafe") on a board where that page does not exist
	 * -- and left a DIRTY, CACHED line tagged for a physical address the
	 * memory controller does not answer for. Nothing failed at the write;
	 * the first whole-cache write-back afterwards is where it stopped.
	 */
#endif
	/*
	 * The preloader may arm the SoC hardware watchdog; a minimal kernel
	 * has no kicker, so disable it before any driver runs. (Replace with
	 * a real watchdog driver once one is integrated.)
	 */
	__raw_writel(0, LUNA_RSTWDT_CTRL);

	/* MMIO/peripheral window: SPI-NOR + SoC registers. */
	ioport_resource.start = 0x14000000;
	ioport_resource.end   = 0x1fffffff;
	iomem_resource.start  = 0x14000000;
	iomem_resource.end    = 0x1fffffff;

	_machine_restart = luna_machine_restart;
	_machine_halt    = luna_machine_halt;
	pm_power_off     = luna_machine_halt;

	/*
	 * The board's bootloader passes no device tree, so the image carries
	 * an appended DTB (CONFIG_MIPS_RAW_APPENDED_DTB). get_fdt() returns the
	 * appended/fw-passed/builtin blob, whichever applies.
	 */
	__dt_setup_arch(get_fdt());
}

/*
 * Unflatten the DT and, on the interAptiv (Coherent Processing System) parts,
 * register the SMP operations before the generic setup_arch() reaches
 * plat_smp_setup() -- which unconditionally dereferences the smp-ops vector, so
 * a platform that leaves it NULL hangs there (right after the reserved-memory
 * scan, before paging_init). Probe the Coherency Manager and Cluster Power
 * Controller, then register the CPS ops; fall back to uniprocessor ops if no CM
 * is present. On the single-threaded RLX parts CONFIG_SMP is off, these probes
 * compile out to no-ops and this is a plain DT unflatten.
 */
#ifdef CONFIG_SMP
/*
 * UNIPROCESSOR SMP OPERATIONS FOR A PART WITH NO COHERENCE MANAGER.
 *
 * ★★★ THIS IS THE M1 BOOT HANG, AND THIS FILE'S OWN COMMENT PREDICTED IT.
 * `plat_smp_setup()` UNCONDITIONALLY dereferences the registered ops vector, so
 * a platform that registers none dies there -- silently, because trap_init()
 * has not run yet, so the exception has nowhere to go and nothing is printed.
 * MEASURED on the LANLY G24W, 2026-08-20: the boot stopped dead after
 * device_tree_init() returned, with `MIPS CPS SMP unable to proceed without a
 * CM` as its last words.
 *
 * WHY NOTHING WAS REGISTERED. The intended fallback, `register_up_smp_ops()`,
 * is a no-op that returns -ENODEV unless CONFIG_SMP_UP is set -- and SMP_UP has
 * no prompt, so it can only be `select`ed. Every MIPS platform that ships a
 * uniprocessor-capable SMP kernel selects it; this platform's Kconfig selects
 * SYS_SUPPORTS_SMP and SYS_SUPPORTS_MIPS_CPS and NOT SMP_UP, which is fine on a
 * part that HAS a CM (CPS registers its own ops) and fatal on one that does
 * not. The RTL9603CVD has no CM: its own vendor kernel says so on this board
 * ("MIPS CPS SMP unable to proceed without a CM", "GIC isn't present!").
 *
 * WHY THE OPS LIVE HERE RATHER THAN IN THE KCONFIG. A platform's SMP ops are
 * platform code, and this tree's standing rule is that kernel wiring is done in
 * real source files under files-<ver>/ and never by editing a patch. Selecting
 * SMP_UP would mean carrying a full copy of arch/mips/Kconfig -- 3200 lines,
 * already modified by this target's platform patch -- to add one line, and that
 * copy would silently diverge from upstream at every kernel bump.
 *
 * WHAT THIS IS NOT: it is not a claim that the part is single-core. This SoC
 * has TWO VPEs of one interAptiv core and the vendor kernel brings both up
 * through MIPS-MT (SMVP). Doing the same needs SYS_SUPPORTS_MULTITHREADING,
 * which this platform does not select either. Until then the second VPE stays
 * parked -- which is exactly what CONFIG_NR_CPUS=1 in the subtarget config
 * already declares -- and the difference is a performance ceiling, not a
 * correctness one.
 */
static void luna_up_send_ipi_single(int cpu, unsigned int action)
{
	/* Nothing to signal: there is no other CPU running. */
}

static void luna_up_send_ipi_mask(const struct cpumask *mask,
				  unsigned int action)
{
}

static void luna_up_init_secondary(void)
{
}

static void luna_up_smp_finish(void)
{
}

static int luna_up_boot_secondary(int cpu, struct task_struct *idle)
{
	/* Refuse rather than pretend: no secondary is brought up here. */
	return -ENODEV;
}

static void luna_up_smp_setup(void)
{
}

static void luna_up_prepare_cpus(unsigned int max_cpus)
{
}

static const struct plat_smp_ops luna_up_smp_ops = {
	.send_ipi_single	= luna_up_send_ipi_single,
	.send_ipi_mask		= luna_up_send_ipi_mask,
	.init_secondary		= luna_up_init_secondary,
	.smp_finish		= luna_up_smp_finish,
	.boot_secondary		= luna_up_boot_secondary,
	.smp_setup		= luna_up_smp_setup,
	.prepare_cpus		= luna_up_prepare_cpus,
};
#endif /* CONFIG_SMP */

void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();

	mips_cm_probe();
	mips_cpc_probe();

#ifdef CONFIG_MIPS_CM
	/* Record the UserLocal (TLS rdhwr $29) feature before trap_init() so
	 * HWREna[29] gets programmed and the C library's TLS read does not SIGILL. */
	luna_enable_userlocal();

	/*
	 * Let Linux MANAGE the on-chip L2 so streaming DMA stays coherent without
	 * bounce buffers. With CONFIG_MIPS_CPU_SCACHE selected, cpu_cache_init() ->
	 * mips_sc_probe() sizes the L2 from Config2 -- which on this SoC reads
	 * SS=4 / SL=4 / SA=7 => 1024 sets x 32-byte line x 8 ways = 256 KB, with the
	 * bypass bit (Config2[12]) clear -- and wires the L2 into the DMA cache ops
	 * (r4k_dma_cache_wback_inv). The boot-time garbage L2 tags are invalidated
	 * first in plat_mem_setup() (luna_l2_invalidate_tags) before the kernel
	 * touches the L2. (Earlier bring-up forced MIPS_CACHE_NOT_PRESENT to dodge a
	 * setup_scache() panic; that panic was really MIPS_CPU_SCACHE being
	 * unselected for this platform, not a zero L2 line size.)
	 */

	/*
	 * The boot loader programs a CCA-default-override in the CM GCR_BASE
	 * (low byte) so accesses use a fixed cache attribute before the L2 is
	 * set up. mips_cm_probe() only fixes the default target (-> MEM); clear
	 * the override too (as the vendor L2-init does) so coherent-domain
	 * accesses fall back to the per-page cache attribute. Left set, the
	 * overridden CCA routes top-of-DRAM cached accesses through the CM
	 * coherent path beyond its range, wedging the first such access -- the
	 * memblock alloc in setup_zero_pages(), early in mm_core_init().
	 */
	if (mips_cm_present()) {
		unsigned long l2cfg = read_gcr_l2_config();

		/* M1 bring-up diagnostic: dump the CM/L2 state (remove later). */
		pr_emerg("LUNA-DIAG: cm_rev=%x config=%x config2=%x gcr_base=%llx l2cfg=%lx l2bypass=%d\n",
			 mips_cm_revision(), read_c0_config(), read_c0_config2(),
			 (unsigned long long)read_gcr_base(), l2cfg,
			 !!(l2cfg & CM_GCR_L2_CONFIG_BYPASS));

		write_gcr_base(read_gcr_base() & ~0xffULL);
	}
#endif

#ifdef CONFIG_SMP
	/*
	 * ★★★ THE WHOLE SMP REGISTRATION IS CONDITIONAL, and it was not until
	 * 2026-08-28. `luna_up_smp_ops` is defined inside the `#ifdef CONFIG_SMP`
	 * block above, and this used it unconditionally -- so the file simply did
	 * not compile with CONFIG_SMP=n:
	 *
	 *   arch/mips/realtek-luna/setup.c:440: error: 'luna_up_smp_ops'
	 *   undeclared (first use in this function)
	 *
	 * Both Luna boards on this bench are single-core and build with SMP off,
	 * so this was broken for BOTH of them.
	 *
	 * ⚠ IT SURVIVED BECAUSE NOTHING COMPILED IT. Each Luna board built from
	 * its own git worktree, and main's realtek-luna target was reached by no
	 * sanctioned build command at all -- so this error sat in the shared tree
	 * unseen. It surfaced within minutes of retiring the per-board branches
	 * and building all three boards from one tree, which is exactly what that
	 * change was for: the branches were not hiding a risk of divergence, they
	 * were hiding a target that did not build.
	 */
	if (!register_cps_smp_ops())
		return;
	/*
	 * No CM, so CPS declined. `register_up_smp_ops()` is a no-op on this
	 * platform (CONFIG_SMP_UP is not selected), and leaving the vector NULL
	 * is what hangs plat_smp_setup(). Register this platform's own.
	 */
	if (register_up_smp_ops())
		register_smp_ops(&luna_up_smp_ops);
#endif /* CONFIG_SMP */
}

void __init plat_time_init(void)
{
	prom_putchar('['); prom_putchar('T'); prom_putchar(']');
	of_clk_init(NULL);
	timer_probe();
}

void __init arch_init_irq(void)
{
	prom_putchar('['); prom_putchar('I'); prom_putchar(']');
	irqchip_init();
	prom_putchar('['); prom_putchar('i'); prom_putchar(']');	/* irqchip/GIC probe returned */
}
