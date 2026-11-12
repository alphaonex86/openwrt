// SPDX-License-Identifier: GPL-2.0-only
/* TIER: FAMILY */
/*
 * CPU frequency for the Realtek RTL9602C, RTL9601D and RTL9603CVD: the OC0 clock
 * management unit divides the OCP (CPU) clock by 2^n. Only that divider moves. The
 * boot loader set the bus slow bits for the full clock, so a divide is offered only
 * while those bits stay right for it, by each chip's own rule below.
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
#define CMU_GCR		0x180
#define CMU_CR		0x188

/* RTL9602C/RTL9601D */
#define OCP_PLL		0x008	/* [5:0] n: (n + 2) x 25 MHz, [7:6] post divide 2^x */
#define MEM_PLL		0x040	/* [31:24] n: up to (n + 2) x 25 / 2 MHz */
#define SPIF_PHYS	0x18001200	/* SFCR */
#define GPHY_PLL_PHYS	0x1b01f044	/* [13:12] s: SPI-F PLL 1000 / (s + 4) MHz */
#define SFCR_OCP_SLOWER	BIT(12)
#define RTL9602C_MAX_DIV 1	/* the boot loader's own bound for the CMU on this chip */

/* RTL9603CVD */
#define SYSPLLCTR	0x000	/* [31] SRAM undivided, [21:16] n: (n + 2) x 50 MHz */
#define PLLCCR		0x004	/* [4] SRAM PLL 500 MHz, else 400 */
#define PLL1		0x00c	/* [18] CPU clock halved */
#define SCATS_PHYS	0x180040f8
#define SCATS_SRAM_TO_OC0 BIT(6)	/* the wide SRAM-to-OC0 pulse: the boot loader's safe value at any clock */
#define SPIF_MAX_MHZ	250	/* NOR 1000 / (4..7), NAND the LX clock 1000 / (5..8) */

#define GCR_BUSY	BIT(31)
#define GCR_DIV		GENMASK(6, 4)
#define GCR_MODE	GENMASK(1, 0)
#define GCR_FIXED	1
#define CR_SLOW_DRAM	BIT(9)
#define CR_SLOW_SRAM	BIT(13)
#define CR_SLOW_SPIF	BIT(17)
#define MAX_DIV		7
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

/* DRAM slow unless OCP >= 2 x DRAM, SPI-F slow unless OCP > the SPI-F PLL. -> the largest divide, via *max. */
static int __init rtl9602c_setup(unsigned int *max)
{
	unsigned int dram_mhz, spif_mhz, ocp_mhz, div;
	u32 pll, cr, sfcr, gphy;
	int ret;

	ret = luna_read(SPIF_PHYS, &sfcr) ?: luna_read(GPHY_PLL_PHYS, &gphy);
	if (ret)
		return ret;
	pll = readl(sys + OCP_PLL);
	cr = readl(sys + CMU_CR);
	ocp_khz = ((pll & 0x3f) + 2) * 25000 >> ((pll >> 6) & 3);
	dram_mhz = ((readl(sys + MEM_PLL) >> 24) + 2) * 25 / 2;
	spif_mhz = 1000 / (((gphy >> 12) & 3) + 4);
	ocp_mhz = ocp_khz / 1000;

	for (div = 1; div <= RTL9602C_MAX_DIV; div++) {
		if (!(cr & CR_SLOW_DRAM) && (ocp_mhz >> div) < 2 * dram_mhz)
			break;
		if ((!(cr & CR_SLOW_SPIF) || !(sfcr & SFCR_OCP_SLOWER)) && (ocp_mhz >> div) <= spif_mhz)
			break;
	}
	*max = div - 1;
	pr_info("luna-cpufreq: %u MHz, DRAM %u, SPI-F %u MHz, CR 0x%08x: down to /%u\n",
		ocp_mhz, dram_mhz, spif_mhz, cr, 1 << *max);
	return 0;
}

/*
 * The boot loader's rule on the OCP bus clock (OCP / 2): SPI-F slow if bus <= SPI-F PLL, SRAM slow
 * if bus <= SRAM, and the wide SRAM-to-OC0 pulse once SRAM x 10 > 9 x bus. The DRAM clock is not read
 * here, so a divide needs the DRAM slow bit already set. Only the SCATS pulse is written, once.
 */
static int __init rtl9603cvd_setup(unsigned int *max)
{
	u32 pll = readl(sys + SYSPLLCTR), cr = readl(sys + CMU_CR), scats;
	unsigned int sram_mhz, bus, div;
	int ret;

	ocp_khz = (FIELD_GET(GENMASK(21, 16), pll) + 2) * 50000 >> !!(readl(sys + PLL1) & BIT(18));
	sram_mhz = (readl(sys + PLLCCR) & BIT(4) ? 500 : 400) >> !(pll & BIT(31));

	for (div = 1; div <= MAX_DIV && (cr & CR_SLOW_DRAM); div++) {
		bus = ocp_khz / 1000 >> div >> 1;
		if (!(cr & CR_SLOW_SPIF) && bus <= SPIF_MAX_MHZ)
			break;
		if (!(cr & CR_SLOW_SRAM) && bus <= sram_mhz)
			break;
	}
	*max = div - 1;
	if (!*max) {
		pr_info("luna-cpufreq: %u MHz, SRAM %u MHz, CR 0x%08x: no divide keeps the slow bits valid\n",
			ocp_khz / 1000, sram_mhz, cr);
		return 0;
	}

	bus = ocp_khz / 1000 >> *max >> 1;
	ret = luna_read(SCATS_PHYS, &scats);
	if (ret)
		return ret;
	if (sram_mhz * 10 > 9 * bus && !(scats & SCATS_SRAM_TO_OC0)) {
		void __iomem *p = ioremap(SCATS_PHYS, 4);

		if (!p)
			return -ENOMEM;
		writel(scats | SCATS_SRAM_TO_OC0, p);
		scats = readl(p);
		iounmap(p);
		if (!(scats & SCATS_SRAM_TO_OC0)) {
			pr_err("luna-cpufreq: SCATS 0x%08x refused the wide SRAM pulse, no divide\n", scats);
			*max = 0;
			return 0;
		}
	}
	pr_info("luna-cpufreq: %u MHz, SRAM %u MHz, CR 0x%08x, SCATS 0x%08x: down to /%u\n",
		ocp_khz / 1000, sram_mhz, cr, scats, 1 << *max);
	return 0;
}

static int __init luna_cpufreq_setup(void)
{
	int (*chip_setup)(unsigned int *max);
	unsigned int max, i;
	u32 gcr;
	int ret;

	if (of_machine_is_compatible("realtek,rtl9602c") || of_machine_is_compatible("realtek,rtl9601d"))
		chip_setup = rtl9602c_setup;
	else if (of_machine_is_compatible("realtek,rtl9603cvd"))
		chip_setup = rtl9603cvd_setup;
	else
		return -ENODEV;
	sys = ioremap(SYS_PHYS, SYS_SIZE);
	if (!sys)
		return -ENOMEM;

	ret = -ENODEV;
	gcr = readl(sys + CMU_GCR);
	if (FIELD_GET(GCR_MODE, gcr)) {
		pr_info("luna-cpufreq: the CMU is already on (0x%08x), left as the boot loader set it\n", gcr);
		goto unmap;
	}
	ret = chip_setup(&max);
	if (ret) {
		pr_err("luna-cpufreq: reading the clock registers failed: %d\n", ret);
		goto unmap;
	}
	ret = -ENODEV;
	if (!max)
		goto unmap;
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
	pr_info("luna-cpufreq: %u kHz down to %u kHz\n", ocp_khz, ocp_khz >> max);
	return 0;
unmap:
	iounmap(sys);
	return ret;
}
device_initcall(luna_cpufreq_setup);
