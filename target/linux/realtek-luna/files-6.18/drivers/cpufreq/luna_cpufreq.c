// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU frequency for the Realtek RTL9602C and RTL9601D: the OC0 clock management
 * unit divides the OCP (CPU) clock by 2^n. Only that divider moves. The boot
 * loader set the bus slow bits for the full clock, so a divide is offered only
 * while those bits stay right for it: DRAM slow unless OCP >= 2 x DRAM, SPI-F
 * slow unless OCP > the SPI-F PLL.
 */
#include <linux/bitfield.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spinlock.h>

#define SYS_PHYS	0x18000200
#define SYS_SIZE	0x200
#define OCP_PLL		0x008	/* [5:0] n: (n + 2) x 25 MHz, [7:6] post divide 2^x */
#define MEM_PLL		0x040	/* [31:24] n: up to (n + 2) x 25 / 2 MHz */
#define CMU_GCR		0x180
#define CMU_CR		0x188
#define SPIF_PHYS	0x18001200	/* SFCR */
#define GPHY_PLL_PHYS	0x1b01f044	/* [13:12] s: SPI-F PLL 1000 / (s + 4) MHz */

#define GCR_BUSY	BIT(31)
#define GCR_DIV		GENMASK(6, 4)
#define GCR_MODE	GENMASK(1, 0)
#define GCR_FIXED	1
#define CR_SLOW_DRAM	BIT(9)
#define CR_SLOW_SPIF	BIT(17)
#define SFCR_OCP_SLOWER	BIT(12)
#define MAX_DIV		1	/* the boot loader's own bound for the CMU on this chip */
#define LATENCY_NS	20000

static void __iomem *sys;
static DEFINE_SPINLOCK(luna_cpufreq_lock);
static unsigned int ocp_khz;
static struct cpufreq_frequency_table table[MAX_DIV + 2];

static unsigned int luna_cpufreq_get(unsigned int cpu)
{
	u32 gcr = readl(sys + CMU_GCR);

	if (FIELD_GET(GCR_MODE, gcr) == GCR_FIXED)
		return ocp_khz >> FIELD_GET(GCR_DIV, gcr);
	return ocp_khz;
}

/* The boot loader's order: back to the undivided clock, settle, then divide. */
static int luna_cpufreq_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	u32 div = table[index].driver_data;
	unsigned long flags;

	spin_lock_irqsave(&luna_cpufreq_lock, flags);
	writel(GCR_BUSY | (readl(sys + CMU_GCR) & GCR_DIV), sys + CMU_GCR);
	udelay(10);
	if (div) {
		writel(GCR_BUSY | FIELD_PREP(GCR_DIV, div), sys + CMU_GCR);
		writel(GCR_BUSY | FIELD_PREP(GCR_DIV, div) | GCR_FIXED, sys + CMU_GCR);
	}
	spin_unlock_irqrestore(&luna_cpufreq_lock, flags);
	return 0;
}

static int luna_cpufreq_init(struct cpufreq_policy *policy)
{
	cpufreq_generic_init(policy, table, LATENCY_NS);
	return 0;
}

static struct cpufreq_driver luna_cpufreq_driver = {
	.name		= "luna",
	.flags		= CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= luna_cpufreq_target_index,
	.get		= luna_cpufreq_get,
	.init		= luna_cpufreq_init,
};

static int __init luna_read(phys_addr_t phys, u32 *val)
{
	void __iomem *p = ioremap(phys, 4);

	if (!p)
		return -ENOMEM;
	*val = readl(p);
	iounmap(p);
	return 0;
}

/* -> the largest divide whose clock keeps every bus slow bit as the boot loader left it. */
static unsigned int __init luna_max_div(u32 cr, u32 sfcr, unsigned int dram_mhz, unsigned int spif_mhz)
{
	unsigned int ocp_mhz = ocp_khz / 1000, div;

	for (div = 1; div <= MAX_DIV; div++) {
		if (!(cr & CR_SLOW_DRAM) && (ocp_mhz >> div) < 2 * dram_mhz)
			break;
		if ((!(cr & CR_SLOW_SPIF) || !(sfcr & SFCR_OCP_SLOWER)) && (ocp_mhz >> div) <= spif_mhz)
			break;
	}
	return div - 1;
}

static int __init luna_cpufreq_setup(void)
{
	unsigned int dram_mhz, spif_mhz, max, i;
	u32 pll, gcr, cr, sfcr, gphy;
	int ret;

	if (!of_machine_is_compatible("realtek,rtl9602c") && !of_machine_is_compatible("realtek,rtl9601d"))
		return -ENODEV;
	sys = ioremap(SYS_PHYS, SYS_SIZE);
	if (!sys)
		return -ENOMEM;
	ret = luna_read(SPIF_PHYS, &sfcr) ?: luna_read(GPHY_PLL_PHYS, &gphy);
	if (ret)
		goto unmap;
	pll = readl(sys + OCP_PLL);
	gcr = readl(sys + CMU_GCR);
	cr = readl(sys + CMU_CR);
	ocp_khz = ((pll & 0x3f) + 2) * 25000 >> ((pll >> 6) & 3);
	dram_mhz = ((readl(sys + MEM_PLL) >> 24) + 2) * 25 / 2;
	spif_mhz = 1000 / (((gphy >> 12) & 3) + 4);

	ret = -ENODEV;
	if (FIELD_GET(GCR_MODE, gcr)) {
		pr_info("luna-cpufreq: the CMU is already on (0x%08x), left as the boot loader set it\n", gcr);
		goto unmap;
	}
	max = luna_max_div(cr, sfcr, dram_mhz, spif_mhz);
	if (!max) {
		pr_info("luna-cpufreq: %u MHz, no divide keeps the bus slow bits valid (DRAM %u, SPI-F %u MHz, CR 0x%08x)\n",
			ocp_khz / 1000, dram_mhz, spif_mhz, cr);
		goto unmap;
	}
	for (i = 0; i <= max; i++) {
		table[i].frequency = ocp_khz >> i;
		table[i].driver_data = i;
	}
	table[i].frequency = CPUFREQ_TABLE_END;

	ret = cpufreq_register_driver(&luna_cpufreq_driver);
	if (ret) {
		pr_err("luna-cpufreq: register failed: %d\n", ret);
		goto unmap;
	}
	pr_info("luna-cpufreq: %u kHz down to %u kHz (DRAM %u, SPI-F %u MHz)\n",
		ocp_khz, ocp_khz >> max, dram_mhz, spif_mhz);
	return 0;
unmap:
	iounmap(sys);
	return ret;
}
device_initcall(luna_cpufreq_setup);
