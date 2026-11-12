// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: BOARD */
/*
 * The V-SOL V1600G board watchdog: a supervisor outside the SoC that resets the board about 11 minutes after
 * power-on unless frames arrive on two GPIO lines. A frame: data falls while the clock is high, 26 bits LSB first
 * each taken on the rising clock, data rises while the clock is high. Each ping sends the next of the two words the
 * stock firmware sends while it boots, every 5 s. Running from power-on and never stoppable: the watchdog core feeds
 * it until user space opens the device.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

#define V1600G_WDT_BITS		26
#define V1600G_WDT_EDGE_US	5
#define V1600G_WDT_HW_PING_MS	10000	/* the core pings at half: stock's 5 s */
#define V1600G_WDT_TIMEOUT	30

static const u32 v1600g_wdt_words[] = { 0x3ff55a5, 0x3ffaaa5 };

struct v1600g_wdt {
	struct watchdog_device wdd;
	struct gpio_desc *clk, *data;
	unsigned int next;
};

static void v1600g_wdt_set(struct gpio_desc *line, int level)
{
	gpiod_set_value(line, level);
	udelay(V1600G_WDT_EDGE_US);
}

static int v1600g_wdt_ping(struct watchdog_device *wdd)
{
	struct v1600g_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 word = v1600g_wdt_words[wdt->next++ % ARRAY_SIZE(v1600g_wdt_words)];
	unsigned int bit;

	v1600g_wdt_set(wdt->clk, 1);
	v1600g_wdt_set(wdt->data, 1);
	v1600g_wdt_set(wdt->data, 0);
	for (bit = 0; bit < V1600G_WDT_BITS; bit++) {
		v1600g_wdt_set(wdt->clk, 0);
		v1600g_wdt_set(wdt->data, word >> bit & 1);
		v1600g_wdt_set(wdt->clk, 1);
	}
	v1600g_wdt_set(wdt->data, 0);
	v1600g_wdt_set(wdt->data, 1);
	return 0;
}

static const struct watchdog_info v1600g_wdt_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT,
	.identity = "V1600G board watchdog",
};

static const struct watchdog_ops v1600g_wdt_ops = {
	.owner = THIS_MODULE,
	.start = v1600g_wdt_ping,
	.ping = v1600g_wdt_ping,
};

static int v1600g_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v1600g_wdt *wdt;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	wdt->clk = devm_gpiod_get(dev, "clock", GPIOD_OUT_HIGH);
	if (IS_ERR(wdt->clk))
		return dev_err_probe(dev, PTR_ERR(wdt->clk), "no clock line\n");
	wdt->data = devm_gpiod_get(dev, "data", GPIOD_OUT_HIGH);
	if (IS_ERR(wdt->data))
		return dev_err_probe(dev, PTR_ERR(wdt->data), "no data line\n");

	wdt->wdd.info = &v1600g_wdt_info;
	wdt->wdd.ops = &v1600g_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.timeout = V1600G_WDT_TIMEOUT;
	wdt->wdd.max_hw_heartbeat_ms = V1600G_WDT_HW_PING_MS;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	v1600g_wdt_ping(&wdt->wdd);
	set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;
	dev_info(dev, "running, fed every %u s until user space takes it\n", V1600G_WDT_HW_PING_MS / 2000);
	return 0;
}

static const struct of_device_id v1600g_wdt_of_match[] = {
	{ .compatible = "vsol,v1600g-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, v1600g_wdt_of_match);

static struct platform_driver v1600g_wdt_driver = {
	.probe = v1600g_wdt_probe,
	.driver = {
		.name = "vsol-v1600g-wdt",
		.of_match_table = v1600g_wdt_of_match,
	},
};
module_platform_driver(v1600g_wdt_driver);

MODULE_DESCRIPTION("V-SOL V1600G board watchdog");
MODULE_LICENSE("GPL");
