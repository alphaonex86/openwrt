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

/* SoC watchdog timer (KSEG1), TC block. Forcing a WDT timeout ...
 * dev/MEASURED-setup.c.md sec 1. */
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
	/* Full-chip reset, fastest scale (2^25 LX clocks ~ 0.17s), ...
	 * dev/MEASURED-setup.c.md sec 12. */
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
/* The on-chip L2 (256 KB, 8-way, 32-byte lines) comes out of ...
 * dev/MEASURED-setup.c.md sec 2. */
#define LUNA_L2_SIZE	(256 << 10)	/* 256 KB */
#define LUNA_L2_LINE	32		/* bytes (Config2 SL reads 0 on this SoC) */

/* THE ON-CHIP L2 IS HIDDEN AT RESET ON THE RTL9603CVD, AND ... -- dev/MEASURED-setup.c.md sec 3. */
#define LUNA_CONF2_L2_BYPASS	BIT(12)
#define LUNA_CONF2_SL_SHIFT	4
#define LUNA_CONF2_SL_MASK	(0xfu << LUNA_CONF2_SL_SHIFT)

/* ★★★ NOTHING MAY TOUCH DRAM BETWEEN THE UN-BYPASS AND THE ... -- dev/MEASURED-setup.c.md sec 4. */
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

/* UserLocal / thread-pointer (TLS) enable. The C library ... -- dev/MEASURED-setup.c.md sec 5. */
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

	/* ★ THE L2's OWN WITNESS, emitted here rather than where the ...
	 * dev/MEASURED-setup.c.md sec 6. */
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
	/* ★ THE TOP-OF-DRAM PROBE THAT USED TO RUN HERE IS GONE, and ...
	 * dev/MEASURED-setup.c.md sec 7. */
#endif
	/* The preloader may arm the SoC hardware watchdog; a minimal ...
	 * dev/MEASURED-setup.c.md sec 13. */
	__raw_writel(0, LUNA_RSTWDT_CTRL);

	/* MMIO/peripheral window: SPI-NOR + SoC registers. */
	ioport_resource.start = 0x14000000;
	ioport_resource.end   = 0x1fffffff;
	iomem_resource.start  = 0x14000000;
	iomem_resource.end    = 0x1fffffff;

	_machine_restart = luna_machine_restart;
	_machine_halt    = luna_machine_halt;
	pm_power_off     = luna_machine_halt;

	/* The board's bootloader passes no device tree, so the image ...
	 * dev/MEASURED-setup.c.md sec 14. */
	__dt_setup_arch(get_fdt());
}

/* Unflatten the DT and, on the interAptiv (Coherent ... -- dev/MEASURED-setup.c.md sec 8. */
#ifdef CONFIG_SMP
/* UNIPROCESSOR SMP OPERATIONS FOR A PART WITH NO COHERENCE ... -- dev/MEASURED-setup.c.md sec 9. */
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

	/* Let Linux MANAGE the on-chip L2 so streaming DMA stays ...
	 * dev/MEASURED-setup.c.md sec 10. */
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
	/* ★★★ THE WHOLE SMP REGISTRATION IS CONDITIONAL, and it was ...
	 * dev/MEASURED-setup.c.md sec 11. */
	if (!register_cps_smp_ops())
		return;
	/* No CM, so CPS declined. `register_up_smp_ops()` is a no-op ...
	 * dev/MEASURED-setup.c.md sec 15. */
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
