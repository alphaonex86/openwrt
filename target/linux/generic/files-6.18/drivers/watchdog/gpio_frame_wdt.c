// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE */
/*
 * A board supervisor fed by frames on two GPIO lines: a watchdog that resets the board unless frames keep coming,
 * and a latch of the frame's other bits as outputs. A frame: data falls while the clock is high, frame-bits bits LSB
 * first each taken on the rising clock, data rises while the clock is high. The board's DT gives the frame: its
 * length, the word sent from probe and the bits never offered (a frame mark, a feed bit whose clearing resets the
 * board). The other bits are a gpio_chip (line n = frame bit n; a set sends a frame at once). The watchdog core feeds
 * it until user space opens the device. One sender only: two interleaved frames are corrupt frames.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/watchdog.h>

#define FRAME_EDGE_US		5
#define FRAME_HW_PING_MS	10000	/* the core pings at half */
#define FRAME_TIMEOUT		30

struct frame_wdt {
	struct watchdog_device wdd;
	struct gpio_chip gc;
	struct gpio_desc *clk, *data;
	struct mutex lock;	/* one frame on the lines at a time */
	u32 bits, fixed, word;
};

static void frame_edge(struct gpio_desc *line, int level)
{
	gpiod_set_value_cansleep(line, level);
	udelay(FRAME_EDGE_US);
}

static void frame_send(struct frame_wdt *fw)
{
	unsigned int bit;

	lockdep_assert_held(&fw->lock);
	frame_edge(fw->clk, 1);
	frame_edge(fw->data, 1);
	frame_edge(fw->data, 0);
	for (bit = 0; bit < fw->bits; bit++) {
		frame_edge(fw->clk, 0);
		frame_edge(fw->data, fw->word >> bit & 1);
		frame_edge(fw->clk, 1);
	}
	frame_edge(fw->data, 0);
	frame_edge(fw->data, 1);
}

static int frame_ping(struct watchdog_device *wdd)
{
	struct frame_wdt *fw = watchdog_get_drvdata(wdd);

	guard(mutex)(&fw->lock);
	frame_send(fw);
	return 0;
}

static const struct watchdog_info frame_wdt_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT,
	.identity = "GPIO frame supervisor",
};

static const struct watchdog_ops frame_wdt_ops = {
	.owner = THIS_MODULE,
	.start = frame_ping,
	.ping = frame_ping,
};

static int frame_out_get(struct gpio_chip *gc, unsigned int line)
{
	struct frame_wdt *fw = gpiochip_get_data(gc);

	guard(mutex)(&fw->lock);
	return !!(fw->word & BIT(line));
}

static int frame_out_set(struct gpio_chip *gc, unsigned int line, int value)
{
	struct frame_wdt *fw = gpiochip_get_data(gc);

	guard(mutex)(&fw->lock);
	fw->word = value ? fw->word | BIT(line) : fw->word & ~BIT(line);
	frame_send(fw);
	return 0;
}

static int frame_out_direction_output(struct gpio_chip *gc, unsigned int line, int value)
{
	return frame_out_set(gc, line, value);
}

static int frame_out_get_direction(struct gpio_chip *gc, unsigned int line)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int frame_out_valid_mask(struct gpio_chip *gc, unsigned long *valid, unsigned int ngpios)
{
	struct frame_wdt *fw = gpiochip_get_data(gc);

	*valid &= ~(unsigned long)fw->fixed;
	return 0;
}

static int frame_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct frame_wdt *fw;
	int ret;

	fw = devm_kzalloc(dev, sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return -ENOMEM;
	if (device_property_read_u32(dev, "frame-bits", &fw->bits) || !fw->bits || fw->bits > 32 ||
	    device_property_read_u32(dev, "frame-boot", &fw->word) ||
	    device_property_read_u32(dev, "frame-fixed", &fw->fixed))
		return dev_err_probe(dev, -EINVAL, "frame-bits (1-32), frame-boot and frame-fixed are required\n");
	ret = devm_mutex_init(dev, &fw->lock);
	if (ret)
		return ret;
	fw->clk = devm_gpiod_get(dev, "clock", GPIOD_OUT_HIGH);
	if (IS_ERR(fw->clk))
		return dev_err_probe(dev, PTR_ERR(fw->clk), "no clock line\n");
	fw->data = devm_gpiod_get(dev, "data", GPIOD_OUT_HIGH);
	if (IS_ERR(fw->data))
		return dev_err_probe(dev, PTR_ERR(fw->data), "no data line\n");

	fw->wdd.info = &frame_wdt_info;
	fw->wdd.ops = &frame_wdt_ops;
	fw->wdd.parent = dev;
	fw->wdd.min_timeout = 1;
	fw->wdd.timeout = FRAME_TIMEOUT;
	fw->wdd.max_hw_heartbeat_ms = FRAME_HW_PING_MS;
	watchdog_set_drvdata(&fw->wdd, fw);
	frame_ping(&fw->wdd);
	set_bit(WDOG_HW_RUNNING, &fw->wdd.status);
	ret = devm_watchdog_register_device(dev, &fw->wdd);
	if (ret)
		return ret;

	fw->gc.label = dev_name(dev);
	fw->gc.parent = dev;
	fw->gc.owner = THIS_MODULE;
	fw->gc.base = -1;
	fw->gc.ngpio = fw->bits;
	fw->gc.can_sleep = true;
	fw->gc.get = frame_out_get;
	fw->gc.set = frame_out_set;
	fw->gc.direction_output = frame_out_direction_output;
	fw->gc.get_direction = frame_out_get_direction;
	fw->gc.init_valid_mask = frame_out_valid_mask;
	ret = devm_gpiochip_add_data(dev, &fw->gc, fw);
	if (ret)
		return dev_err_probe(dev, ret, "outputs not registered\n");
	dev_info(dev, "running, frame 0x%x fed every %u s until user space takes it\n", fw->word,
		 FRAME_HW_PING_MS / 2000);
	return 0;
}

static const struct of_device_id frame_wdt_of_match[] = {
	{ .compatible = "gpio-frame-watchdog" },
	{ }
};
MODULE_DEVICE_TABLE(of, frame_wdt_of_match);

static struct platform_driver frame_wdt_driver = {
	.probe = frame_wdt_probe,
	.driver = {
		.name = "gpio-frame-wdt",
		.of_match_table = frame_wdt_of_match,
	},
};
module_platform_driver(frame_wdt_driver);

MODULE_DESCRIPTION("Board supervisor fed by GPIO frames: watchdog and outputs");
MODULE_LICENSE("GPL");
