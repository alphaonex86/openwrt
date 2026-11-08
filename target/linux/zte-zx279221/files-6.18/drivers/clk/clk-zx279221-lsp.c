// SPDX-License-Identifier: GPL-2.0-only
/*
 * ZTE ZX279221 LSP peripheral clock register (one per block, e.g. each watchdog at LSP0 CRM 0x1c + 4n). Facts from
 * the unit's own kernel: bit 0 gates the bus clock, bit 1 the work clock, bit 4 releases the block's reset (0 holds
 * it), bits 20:11 divide the work clock by value + 1, rounded to the closest rate.
 * Clocks: 0 = bus clock, 1 = work clock; reset 0.
 */
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#define ZX_LSP_PCLK_GATE	0
#define ZX_LSP_WCLK_GATE	1
#define ZX_LSP_RESET_BIT	4
#define ZX_LSP_DIV_SHIFT	11
#define ZX_LSP_DIV_WIDTH	10

struct zx_lsp {
	struct reset_controller_dev rcdev;
	void __iomem *reg;
	spinlock_t lock;
	struct clk_hw_onecell_data *clks;
};

static int zx_lsp_reset_write(struct reset_controller_dev *rcdev, bool released)
{
	struct zx_lsp *lsp = container_of(rcdev, struct zx_lsp, rcdev);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&lsp->lock, flags);
	val = readl(lsp->reg);
	val = released ? val | BIT(ZX_LSP_RESET_BIT) : val & ~BIT(ZX_LSP_RESET_BIT);
	writel(val, lsp->reg);
	spin_unlock_irqrestore(&lsp->lock, flags);
	return 0;
}

static int zx_lsp_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return zx_lsp_reset_write(rcdev, false);
}

static int zx_lsp_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return zx_lsp_reset_write(rcdev, true);
}

static const struct reset_control_ops zx_lsp_reset_ops = {
	.assert = zx_lsp_assert,
	.deassert = zx_lsp_deassert,
};

static int zx_lsp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *parent = of_clk_get_parent_name(dev->of_node, 0);
	struct clk_hw *pclk, *wgate, *wclk;
	struct zx_lsp *lsp;
	int ret;

	if (!parent)
		return dev_err_probe(dev, -EINVAL, "no parent clock\n");
	lsp = devm_kzalloc(dev, sizeof(*lsp), GFP_KERNEL);
	if (!lsp)
		return -ENOMEM;
	lsp->clks = devm_kzalloc(dev, struct_size(lsp->clks, hws, 2), GFP_KERNEL);
	if (!lsp->clks)
		return -ENOMEM;
	lsp->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lsp->reg))
		return PTR_ERR(lsp->reg);
	spin_lock_init(&lsp->lock);

	pclk = devm_clk_hw_register_gate(dev, devm_kasprintf(dev, GFP_KERNEL, "%s-pclk", dev_name(dev)), parent, 0,
					 lsp->reg, ZX_LSP_PCLK_GATE, 0, &lsp->lock);
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk), "bus clock gate\n");
	wgate = devm_clk_hw_register_gate(dev, devm_kasprintf(dev, GFP_KERNEL, "%s-wgate", dev_name(dev)), parent, 0,
					  lsp->reg, ZX_LSP_WCLK_GATE, 0, &lsp->lock);
	if (IS_ERR(wgate))
		return dev_err_probe(dev, PTR_ERR(wgate), "work clock gate\n");
	wclk = devm_clk_hw_register_divider_parent_hw(dev, devm_kasprintf(dev, GFP_KERNEL, "%s-wclk", dev_name(dev)),
						      wgate, 0, lsp->reg, ZX_LSP_DIV_SHIFT, ZX_LSP_DIV_WIDTH,
						      CLK_DIVIDER_ROUND_CLOSEST, &lsp->lock);
	if (IS_ERR(wclk))
		return dev_err_probe(dev, PTR_ERR(wclk), "work clock divider\n");
	lsp->clks->num = 2;
	lsp->clks->hws[0] = pclk;
	lsp->clks->hws[1] = wclk;
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, lsp->clks);
	if (ret)
		return dev_err_probe(dev, ret, "clock provider\n");

	lsp->rcdev.owner = THIS_MODULE;
	lsp->rcdev.ops = &zx_lsp_reset_ops;
	lsp->rcdev.of_node = dev->of_node;
	lsp->rcdev.nr_resets = 1;
	ret = devm_reset_controller_register(dev, &lsp->rcdev);
	if (ret)
		return dev_err_probe(dev, ret, "reset controller\n");
	return 0;
}

static const struct of_device_id zx_lsp_match[] = {
	{ .compatible = "zte,zx279221-lsp-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, zx_lsp_match);

static struct platform_driver zx_lsp_driver = {
	.probe = zx_lsp_probe,
	.driver = {
		.name = "zx279221-lsp-clk",
		.of_match_table = zx_lsp_match,
	},
};
module_platform_driver(zx_lsp_driver);

MODULE_DESCRIPTION("ZTE ZX279221 LSP peripheral clock and reset");
MODULE_LICENSE("GPL");
