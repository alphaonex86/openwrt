// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: FAMILY */
/*
 * ZX279128 secondary CPU boot. Out of reset CPU1 waits in the boot ROM until the start of IRAM holds a three-word
 * trampoline whose fourth word is a physical entry point, and a wake-up event; the stock kernel writes exactly that.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>

#define ZX_IRAM_BASE	0x00200000

static const u32 zx_trampoline[] = {
	0xe24f0008,	/* sub r0, pc, #8: r0 = this word */
	0xe280000c,	/* add r0, r0, #12 */
	0xe590f000,	/* ldr pc, [r0]: jump to the entry point that follows */
};

static void __init zx_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-scu");
	void __iomem *scu = np ? of_iomap(np, 0) : NULL;

	of_node_put(np);
	if (!scu) {
		pr_err("zx279128: no SCU in the device tree, secondary CPUs stay off\n");
		return;
	}
	scu_enable(scu);
	iounmap(scu);
}

static int zx_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	void __iomem *iram = ioremap(ZX_IRAM_BASE, sizeof(zx_trampoline) + sizeof(u32));
	unsigned int i;

	if (!iram)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(zx_trampoline); i++)
		writel_relaxed(zx_trampoline[i], iram + 4 * i);
	writel(__pa_symbol(secondary_startup), iram + sizeof(zx_trampoline));
	iounmap(iram);
	dsb_sev();
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));
	return 0;
}

static const struct smp_operations zx_smp_ops __initconst = {
	.smp_prepare_cpus	= zx_smp_prepare_cpus,
	.smp_boot_secondary	= zx_boot_secondary,
};
CPU_METHOD_OF_DECLARE(zx279128_smp, "zte,zx279128-smp", &zx_smp_ops);
