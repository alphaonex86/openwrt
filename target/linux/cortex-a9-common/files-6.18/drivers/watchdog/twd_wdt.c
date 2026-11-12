// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: FAMILY */
/*
 * Cortex-A9 private watchdog (TWD, PERIPHBASE + 0x620) in watchdog mode: its counter resets the SoC at zero.
 * The registers are banked per CPU, so every access runs on CPU0, whose watchdog this driver owns. Started at
 * probe, as every ONU keeps a running hardware watchdog from boot.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/smp.h>
#include <linux/watchdog.h>

#define TWD_WDT_LOAD		0x00
#define TWD_WDT_CONTROL		0x08
#define TWD_WDT_RESET_STATUS	0x10
#define TWD_WDT_DISABLE		0x14
#define TWD_WDT_ENABLE		BIT(0)
#define TWD_WDT_WATCHDOG_MODE	BIT(3)
#define TWD_WDT_PRESCALER	255	/* the slowest tick, so the 32-bit load holds the longest timeout */
#define TWD_WDT_DISABLE_KEY1	0x12345678
#define TWD_WDT_DISABLE_KEY2	0x87654321
#define TWD_WDT_CPU		0
#define TWD_WDT_DEFAULT_TIMEOUT	30

static unsigned int timeout;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default "
		 __MODULE_STRING(TWD_WDT_DEFAULT_TIMEOUT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

struct twd_wdt {
	struct watchdog_device wdd;
	void __iomem *base;
	u32 tick_hz;
	bool reset_seen;
};

static void twd_wdt_arm_on_cpu(void *data)
{
	struct twd_wdt *wdt = data;

	writel(wdt->wdd.timeout * wdt->tick_hz, wdt->base + TWD_WDT_LOAD);
	writel(TWD_WDT_PRESCALER << 8 | TWD_WDT_WATCHDOG_MODE | TWD_WDT_ENABLE, wdt->base + TWD_WDT_CONTROL);
}

static void twd_wdt_stop_on_cpu(void *data)
{
	struct twd_wdt *wdt = data;

	writel(TWD_WDT_DISABLE_KEY1, wdt->base + TWD_WDT_DISABLE);
	writel(TWD_WDT_DISABLE_KEY2, wdt->base + TWD_WDT_DISABLE);
	writel(0, wdt->base + TWD_WDT_CONTROL);
}

static void twd_wdt_reset_status_on_cpu(void *data)
{
	struct twd_wdt *wdt = data;

	wdt->reset_seen = readl(wdt->base + TWD_WDT_RESET_STATUS) & 1;
	writel(1, wdt->base + TWD_WDT_RESET_STATUS);
}

static int twd_wdt_on_cpu0(struct twd_wdt *wdt, smp_call_func_t fn)
{
	int ret = smp_call_function_single(TWD_WDT_CPU, fn, wdt, 1);

	if (ret)
		dev_err(wdt->wdd.parent, "CPU%d could not run the watchdog access: %d\n", TWD_WDT_CPU, ret);
	return ret;
}

static int twd_wdt_start(struct watchdog_device *wdd)
{
	return twd_wdt_on_cpu0(watchdog_get_drvdata(wdd), twd_wdt_arm_on_cpu);
}

static int twd_wdt_stop(struct watchdog_device *wdd)
{
	int ret = twd_wdt_on_cpu0(watchdog_get_drvdata(wdd), twd_wdt_stop_on_cpu);

	if (!ret)
		clear_bit(WDOG_HW_RUNNING, &wdd->status);
	return ret;
}

static int twd_wdt_set_timeout(struct watchdog_device *wdd, unsigned int t)
{
	wdd->timeout = t;
	return watchdog_active(wdd) || watchdog_hw_running(wdd) ? twd_wdt_start(wdd) : 0;
}

static const struct watchdog_info twd_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE | WDIOF_CARDRESET,
	.identity = "Cortex-A9 TWD watchdog",
};

static const struct watchdog_ops twd_wdt_ops = {
	.owner = THIS_MODULE,
	.start = twd_wdt_start,
	.stop = twd_wdt_stop,
	.ping = twd_wdt_start,
	.set_timeout = twd_wdt_set_timeout,
};

static int twd_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct twd_wdt *wdt;
	struct clk *clk;
	unsigned long rate;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return dev_err_probe(dev, PTR_ERR(wdt->base), "no register block\n");
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no PERIPHCLK\n");
	rate = clk_get_rate(clk);
	if (rate < TWD_WDT_PRESCALER + 1)
		return dev_err_probe(dev, -EINVAL, "PERIPHCLK rate %lu Hz is unusable\n", rate);
	wdt->tick_hz = rate / (TWD_WDT_PRESCALER + 1);

	wdt->wdd.info = &twd_wdt_info;
	wdt->wdd.ops = &twd_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = U32_MAX / wdt->tick_hz;
	wdt->wdd.timeout = TWD_WDT_DEFAULT_TIMEOUT;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	ret = watchdog_init_timeout(&wdt->wdd, timeout, dev);
	if (ret)
		dev_warn(dev, "timeout %u s refused (%d), keeping %u s\n", timeout, ret, wdt->wdd.timeout);
	watchdog_set_nowayout(&wdt->wdd, nowayout);

	ret = twd_wdt_on_cpu0(wdt, twd_wdt_reset_status_on_cpu);
	if (ret)
		return ret;
	if (wdt->reset_seen)
		wdt->wdd.bootstatus = WDIOF_CARDRESET;

	ret = twd_wdt_start(&wdt->wdd);
	if (ret)
		return ret;
	set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;
	dev_info(dev, "running, %u s timeout%s\n", wdt->wdd.timeout,
		 wdt->reset_seen ? ", the last reset was this watchdog's" : "");
	return 0;
}

static const struct of_device_id twd_wdt_of_match[] = {
	{ .compatible = "arm,cortex-a9-twd-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, twd_wdt_of_match);

static struct platform_driver twd_wdt_driver = {
	.probe = twd_wdt_probe,
	.driver = {
		.name = "twd-wdt",
		.of_match_table = twd_wdt_of_match,
	},
};
module_platform_driver(twd_wdt_driver);

MODULE_DESCRIPTION("Cortex-A9 private watchdog (TWD) in watchdog mode");
MODULE_LICENSE("GPL");
