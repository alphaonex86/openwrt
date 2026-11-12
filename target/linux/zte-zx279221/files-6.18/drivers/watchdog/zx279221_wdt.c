// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/*
 * ZTE ZX279221 SoC watchdog (ZXA10 C610 OLT main board). Facts from the unit's own kernel and device tree: a
 * 16-bit down-counter of the work clock (asked for 32768 Hz, as stock) through an 8-bit divider; a write lands only with the key
 * 0x1234 in its upper half; a change of the refresh register's low six bits reloads the count; on expiry the chip
 * resets once the system controller holds this block's route bits (zte,wdt-reset-sysctrl).
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/watchdog.h>

#define ZX_WDT_CFG		0x04	/* bits 15:8: divider - 1 */
#define ZX_WDT_LOAD		0x08
#define ZX_WDT_REFRESH		0x18
#define ZX_WDT_START		0x1c
#define ZX_WDT_KEY		0x12340000
#define ZX_WDT_VALUE		GENMASK(15, 0)
#define ZX_WDT_TOGGLE		GENMASK(5, 0)
#define ZX_WDT_RUN		BIT(0)
#define ZX_WDT_DIV_MIN		16	/* the stock driver's smallest divider: lower ones were never seen at work */
#define ZX_WDT_DIV_MAX		256
#define ZX_WDT_DEFAULT_TIMEOUT	32
#define ZX_WDT_CLK_HZ		32768

struct zx_wdt {
	struct watchdog_device wdd;
	void __iomem *base;
	unsigned long rate;
};

static u32 zx_wdt_read(struct zx_wdt *wdt, unsigned int reg)
{
	return readl(wdt->base + reg) & ZX_WDT_VALUE;
}

static void zx_wdt_write(struct zx_wdt *wdt, unsigned int reg, u32 val)
{
	writel(ZX_WDT_KEY | (val & ZX_WDT_VALUE), wdt->base + reg);
}

static void zx_wdt_program(struct zx_wdt *wdt, u32 div, u32 load)
{
	zx_wdt_write(wdt, ZX_WDT_CFG, (div - 1) << 8);
	zx_wdt_write(wdt, ZX_WDT_LOAD, load);
	zx_wdt_write(wdt, ZX_WDT_REFRESH, zx_wdt_read(wdt, ZX_WDT_REFRESH) ^ ZX_WDT_TOGGLE);
}

static int zx_wdt_ping(struct watchdog_device *wdd)
{
	struct zx_wdt *wdt = watchdog_get_drvdata(wdd);

	zx_wdt_write(wdt, ZX_WDT_REFRESH, zx_wdt_read(wdt, ZX_WDT_REFRESH) ^ ZX_WDT_TOGGLE);
	return 0;
}

static int zx_wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	struct zx_wdt *wdt = watchdog_get_drvdata(wdd);
	u64 ticks = (u64)timeout * wdt->rate;
	u32 div = max_t(u32, DIV_ROUND_UP_ULL(ticks, ZX_WDT_VALUE), ZX_WDT_DIV_MIN);
	u32 load = DIV_ROUND_UP_ULL(ticks, div);

	zx_wdt_program(wdt, div, load);
	wdd->timeout = div_u64((u64)div * load, wdt->rate);
	return 0;
}

static int zx_wdt_start(struct watchdog_device *wdd)
{
	struct zx_wdt *wdt = watchdog_get_drvdata(wdd);

	zx_wdt_set_timeout(wdd, wdd->timeout);
	zx_wdt_write(wdt, ZX_WDT_START, zx_wdt_read(wdt, ZX_WDT_START) | ZX_WDT_RUN);
	return 0;
}

static int zx_wdt_stop(struct watchdog_device *wdd)
{
	struct zx_wdt *wdt = watchdog_get_drvdata(wdd);

	zx_wdt_write(wdt, ZX_WDT_START, zx_wdt_read(wdt, ZX_WDT_START) & ~ZX_WDT_RUN);
	return 0;
}

/* Reached only when no earlier handler (PSCI SYSTEM_RESET) reset the chip: the shortest count, then wait. */
static int zx_wdt_restart(struct watchdog_device *wdd, unsigned long action, void *data)
{
	struct zx_wdt *wdt = watchdog_get_drvdata(wdd);

	zx_wdt_program(wdt, ZX_WDT_DIV_MIN, 1);
	zx_wdt_write(wdt, ZX_WDT_START, zx_wdt_read(wdt, ZX_WDT_START) | ZX_WDT_RUN);
	mdelay(1000);
	return 0;
}

static const struct watchdog_info zx_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "ZX279221 watchdog",
};

static const struct watchdog_ops zx_wdt_ops = {
	.owner = THIS_MODULE,
	.start = zx_wdt_start,
	.stop = zx_wdt_stop,
	.ping = zx_wdt_ping,
	.set_timeout = zx_wdt_set_timeout,
	.restart = zx_wdt_restart,
};

/* zte,wdt-reset-sysctrl = <&syscon offset value mask>: route this block's expiry to the chip reset. */
static int zx_wdt_route_reset(struct device *dev)
{
	struct of_phandle_args route;
	struct regmap *map;
	int ret;

	ret = of_parse_phandle_with_fixed_args(dev->of_node, "zte,wdt-reset-sysctrl", 3, 0, &route);
	if (ret)
		return dev_err_probe(dev, ret, "zte,wdt-reset-sysctrl: an expiry would reset nothing\n");
	map = syscon_node_to_regmap(route.np);
	of_node_put(route.np);
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "reset route syscon\n");
	ret = regmap_update_bits(map, route.args[0], route.args[2], route.args[1]);
	if (ret)
		return dev_err_probe(dev, ret, "reset route write\n");
	return 0;
}

static int zx_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zx_wdt *wdt;
	struct reset_control *rst;
	struct clk *clk, *pclk;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);
	pclk = devm_clk_get_optional_enabled(dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk), "bus clock\n");
	clk = devm_clk_get_enabled(dev, "wclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "work clock\n");
	ret = clk_set_rate(clk, ZX_WDT_CLK_HZ);
	if (ret)
		return dev_err_probe(dev, ret, "work clock at %u Hz\n", ZX_WDT_CLK_HZ);
	wdt->rate = clk_get_rate(clk);
	if (!wdt->rate)
		return dev_err_probe(dev, -EINVAL, "work clock has no rate\n");
	rst = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "block reset\n");
	ret = zx_wdt_route_reset(dev);
	if (ret)
		return ret;

	wdt->wdd.info = &zx_wdt_info;
	wdt->wdd.ops = &zx_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = div_u64((u64)ZX_WDT_VALUE * ZX_WDT_DIV_MAX, wdt->rate);
	wdt->wdd.timeout = min_t(unsigned int, ZX_WDT_DEFAULT_TIMEOUT, wdt->wdd.max_timeout);
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_init_timeout(&wdt->wdd, 0, dev);
	watchdog_stop_on_reboot(&wdt->wdd);
	if (zx_wdt_read(wdt, ZX_WDT_START) & ZX_WDT_RUN) {
		zx_wdt_set_timeout(&wdt->wdd, wdt->wdd.timeout);
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
	}
	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;
	dev_info(dev, "%u s timeout (max %u s), %lu Hz\n", wdt->wdd.timeout, wdt->wdd.max_timeout, wdt->rate);
	return 0;
}

static const struct of_device_id zx_wdt_match[] = {
	{ .compatible = "zte,zx279221-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, zx_wdt_match);

static struct platform_driver zx_wdt_driver = {
	.probe = zx_wdt_probe,
	.driver = {
		.name = "zx279221-wdt",
		.of_match_table = zx_wdt_match,
	},
};
module_platform_driver(zx_wdt_driver);

MODULE_DESCRIPTION("ZTE ZX279221 SoC watchdog");
MODULE_LICENSE("GPL");
