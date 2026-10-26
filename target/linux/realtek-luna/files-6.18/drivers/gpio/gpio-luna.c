// SPDX-License-Identifier: GPL-2.0-or-later
/* Realtek Luna (RTL960x) GPIO bank: 32 lines, line n = bit n of each register.
 *
 * A line reaches its pad only once its bit is set in the switch core's
 * IO_GPIO_EN word for the bank (the node's second reg); the stock kernels set
 * that bit whenever they configure a pin, and .request does the same here.
 *
 * No shadow registers: the GPON driver writes whole DIR/DATA words of its
 * board's pad recipe at init, so every access is a locked read-modify-write of
 * the live register and nothing written elsewhere is clobbered. */

#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>

#define LUNA_GPIO_DIR		0x08	/* 1 = output */
#define LUNA_GPIO_DATA		0x0c
#define LUNA_GPIO_LINES		32

struct luna_gpio {
	struct gpio_chip gc;
	void __iomem *base;
	void __iomem *pad_en;
	raw_spinlock_t lock;
};

static void luna_gpio_update(struct luna_gpio *lg, void __iomem *reg, u32 mask, bool set)
{
	unsigned long flags;
	u32 v;

	raw_spin_lock_irqsave(&lg->lock, flags);
	v = readl(reg);
	writel(set ? v | mask : v & ~mask, reg);
	raw_spin_unlock_irqrestore(&lg->lock, flags);
}

static int luna_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	struct luna_gpio *lg = gpiochip_get_data(gc);

	luna_gpio_update(lg, lg->pad_en, BIT(offset), true);
	if (!(readl(lg->pad_en) & BIT(offset))) {
		dev_err(gc->parent, "line %u: pad enable bit did not stick\n", offset);
		return -EIO;
	}
	return 0;
}

static int luna_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct luna_gpio *lg = gpiochip_get_data(gc);

	return readl(lg->base + LUNA_GPIO_DIR) & BIT(offset) ?
	       GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int luna_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct luna_gpio *lg = gpiochip_get_data(gc);

	return !!(readl(lg->base + LUNA_GPIO_DATA) & BIT(offset));
}

static int luna_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct luna_gpio *lg = gpiochip_get_data(gc);

	luna_gpio_update(lg, lg->base + LUNA_GPIO_DATA, BIT(offset), value);
	return 0;
}

static int luna_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct luna_gpio *lg = gpiochip_get_data(gc);

	luna_gpio_update(lg, lg->base + LUNA_GPIO_DIR, BIT(offset), false);
	return 0;
}

/* Level first, direction last: the pin never drives a stale level. */
static int luna_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct luna_gpio *lg = gpiochip_get_data(gc);

	luna_gpio_update(lg, lg->base + LUNA_GPIO_DATA, BIT(offset), value);
	luna_gpio_update(lg, lg->base + LUNA_GPIO_DIR, BIT(offset), true);
	return 0;
}

static int luna_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct luna_gpio *lg;
	u32 ngpios = LUNA_GPIO_LINES;

	lg = devm_kzalloc(dev, sizeof(*lg), GFP_KERNEL);
	if (!lg)
		return -ENOMEM;
	lg->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lg->base))
		return PTR_ERR(lg->base);
	lg->pad_en = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(lg->pad_en))
		return PTR_ERR(lg->pad_en);
	device_property_read_u32(dev, "ngpios", &ngpios);
	if (!ngpios || ngpios > LUNA_GPIO_LINES)
		return dev_err_probe(dev, -EINVAL, "ngpios %u out of 1..%u\n",
				     ngpios, LUNA_GPIO_LINES);
	raw_spin_lock_init(&lg->lock);

	lg->gc.label = dev_name(dev);
	lg->gc.parent = dev;
	lg->gc.owner = THIS_MODULE;
	lg->gc.base = -1;
	lg->gc.ngpio = ngpios;
	lg->gc.request = luna_gpio_request;
	lg->gc.get_direction = luna_gpio_get_direction;
	lg->gc.direction_input = luna_gpio_direction_input;
	lg->gc.direction_output = luna_gpio_direction_output;
	lg->gc.get = luna_gpio_get;
	lg->gc.set = luna_gpio_set;
	return devm_gpiochip_add_data(dev, &lg->gc, lg);
}

static const struct of_device_id luna_gpio_of_match[] = {
	{ .compatible = "realtek,luna-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, luna_gpio_of_match);

static struct platform_driver luna_gpio_driver = {
	.probe = luna_gpio_probe,
	.driver = {
		.name = "luna-gpio",
		.of_match_table = luna_gpio_of_match,
	},
};

/* Before device_initcall: the GPON driver takes its laser pins from here at
 * module_init and cannot defer. */
static int __init luna_gpio_init(void)
{
	return platform_driver_register(&luna_gpio_driver);
}
subsys_initcall(luna_gpio_init);

MODULE_DESCRIPTION("Realtek Luna GPIO bank");
MODULE_LICENSE("GPL");
