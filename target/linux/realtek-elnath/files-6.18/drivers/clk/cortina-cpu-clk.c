// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CPU clock of the RTL9607F (Cortina-Access ca77xx family), so cpufreq-dt scales the Cortex-A55 pair.
 *
 * Window 0xf4320230 (global block): +0x00 the CPU clock select, +0x30 the CPU PLL, +0x34 its guard.
 *   select bit 11   CPU source: 0 the CPU PLL, 1 the fixed fallback (div_f2c)
 *   select 5:0      CPU divider, 0 = undivided; bit 6 and bit 31 are set by the vendor change
 *   PLL 23:16       N, the CPU runs at 50 MHz * (N + 2) (500..1200 MHz)
 *   PLL 9:8, 11:10  cleared while N changes, then set to 1 each
 *   guard 15:14     set while the PLL relocks
 * The order and the settle times are the stock kernel's; measured 2026-10-30 on the X400AXF, a fixed loop
 * runs 2.41x longer at 500 MHz than at 1200 MHz on both firmwares.
 */
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define CPU_SEL			0x00
#define  CPU_SEL_FALLBACK	BIT(11)
#define  CPU_SEL_DIV		GENMASK(5, 0)
#define  CPU_SEL_DIV_SET	BIT(6)
#define  CPU_SEL_OVERRIDE	BIT(31)
#define CPU_PLL			0x30
#define  CPU_PLL_N		GENMASK(23, 16)
#define  CPU_PLL_EN_LO		GENMASK(9, 8)
#define  CPU_PLL_EN_HI		GENMASK(11, 10)
#define CPU_PLL_GUARD		0x34
#define  CPU_PLL_GUARD_ON	GENMASK(15, 14)

#define CPU_STEP_HZ		50000000UL
#define CPU_MIN_HZ		500000000UL
#define CPU_MAX_HZ		1200000000UL

struct cortina_cpu_clk {
	struct clk_hw hw;
	void __iomem *base;
	spinlock_t lock;	/* read-modify-writes of the shared select word */
};

#define to_cpu_clk(_hw) container_of(_hw, struct cortina_cpu_clk, hw)

static void cpu_clk_rmw(struct cortina_cpu_clk *c, unsigned int reg, u32 clear, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&c->lock, flags);
	writel((readl(c->base + reg) & ~clear) | set, c->base + reg);
	spin_unlock_irqrestore(&c->lock, flags);
}

static unsigned long cortina_cpu_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct cortina_cpu_clk *c = to_cpu_clk(hw);
	u32 sel = readl(c->base + CPU_SEL);
	unsigned long rate = CPU_STEP_HZ * (FIELD_GET(CPU_PLL_N, readl(c->base + CPU_PLL)) + 2);

	if (sel & CPU_SEL_FALLBACK)
		pr_warn_once("cortina-cpu-clk: the CPU runs on its fallback source, the reported rate is the PLL's\n");
	return FIELD_GET(CPU_SEL_DIV, sel) ? rate / FIELD_GET(CPU_SEL_DIV, sel) : rate;
}

static int cortina_cpu_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	req->rate = clamp(rounddown(req->rate, CPU_STEP_HZ), CPU_MIN_HZ, CPU_MAX_HZ);
	return 0;
}

static int cortina_cpu_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct cortina_cpu_clk *c = to_cpu_clk(hw);
	u32 n = rate / CPU_STEP_HZ - 2;

	if (rate % CPU_STEP_HZ || rate < CPU_MIN_HZ || rate > CPU_MAX_HZ)
		return -EINVAL;
	cpu_clk_rmw(c, CPU_SEL, 0, CPU_SEL_FALLBACK);
	fsleep(2000);
	cpu_clk_rmw(c, CPU_PLL_GUARD, 0, CPU_PLL_GUARD_ON);
	cpu_clk_rmw(c, CPU_PLL, CPU_PLL_EN_HI, 0);
	cpu_clk_rmw(c, CPU_PLL, CPU_PLL_EN_LO, 0);
	cpu_clk_rmw(c, CPU_SEL, CPU_SEL_DIV, CPU_SEL_DIV_SET);
	cpu_clk_rmw(c, CPU_SEL, 0, CPU_SEL_OVERRIDE);
	cpu_clk_rmw(c, CPU_PLL, CPU_PLL_N, FIELD_PREP(CPU_PLL_N, n));
	cpu_clk_rmw(c, CPU_PLL, CPU_PLL_EN_LO, FIELD_PREP(CPU_PLL_EN_LO, 1));
	cpu_clk_rmw(c, CPU_PLL, CPU_PLL_EN_HI, FIELD_PREP(CPU_PLL_EN_HI, 1));
	fsleep(500);
	cpu_clk_rmw(c, CPU_PLL_GUARD, CPU_PLL_GUARD_ON, 0);
	fsleep(10);
	cpu_clk_rmw(c, CPU_SEL, CPU_SEL_FALLBACK, 0);
	return 0;
}

static const struct clk_ops cortina_cpu_clk_ops = {
	.recalc_rate = cortina_cpu_recalc_rate,
	.determine_rate = cortina_cpu_determine_rate,
	.set_rate = cortina_cpu_set_rate,
};

static int cortina_cpu_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_init_data init = { .name = "cpu", .ops = &cortina_cpu_clk_ops };
	struct cortina_cpu_clk *c;
	int ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	spin_lock_init(&c->lock);
	c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(c->base))
		return PTR_ERR(c->base);
	c->hw.init = &init;
	ret = devm_clk_hw_register(dev, &c->hw);
	if (ret)
		return dev_err_probe(dev, ret, "register\n");
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &c->hw);
	if (ret)
		return dev_err_probe(dev, ret, "provider\n");
	dev_info(dev, "CPU at %lu MHz\n", clk_hw_get_rate(&c->hw) / 1000000);
	return 0;
}

static const struct of_device_id cortina_cpu_clk_of_match[] = {
	{ .compatible = "cortina,cpu-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_cpu_clk_of_match);

static struct platform_driver cortina_cpu_clk_driver = {
	.probe = cortina_cpu_clk_probe,
	.driver = {
		.name = "cortina-cpu-clk",
		.of_match_table = cortina_cpu_clk_of_match,
	},
};
module_platform_driver(cortina_cpu_clk_driver);

MODULE_DESCRIPTION("RTL9607F (Cortina-Access ca77xx) CPU clock");
MODULE_LICENSE("GPL");
