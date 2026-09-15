// SPDX-License-Identifier: GPL-2.0
/* Cortina-Access CA77xx peripheral GPIO controller. Found in ...
 * dev/MEASURED-gpio-cortina-ca77xx.c.md sec 1. */

#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>

#define CA77XX_GPIO_CFG		0x00	/* 1 = input, 0 = output */
#define CA77XX_GPIO_OUT		0x04
#define CA77XX_GPIO_IN		0x08

#define CA77XX_GPIO_PINS	32

struct ca77xx_gpio {
	struct gpio_chip	gc;
	void __iomem		*base;	/* this group's CFG/OUT/IN */
	void __iomem		*mux;	/* this group's GLB pin-mux word, or NULL */
	spinlock_t		lock;	/* serialises our own read-modify-writes */
};

/* OR one bit into a register, leaving every other bit as it was. */
static void ca77xx_set_bit(struct ca77xx_gpio *cg, unsigned int reg, u32 mask)
{
	unsigned long flags;

	spin_lock_irqsave(&cg->lock, flags);
	writel(readl(cg->base + reg) | mask, cg->base + reg);
	spin_unlock_irqrestore(&cg->lock, flags);
}

static void ca77xx_clr_bit(struct ca77xx_gpio *cg, unsigned int reg, u32 mask)
{
	unsigned long flags;

	spin_lock_irqsave(&cg->lock, flags);
	writel(readl(cg->base + reg) & ~mask, cg->base + reg);
	spin_unlock_irqrestore(&cg->lock, flags);
}

/* A pin only reaches the GPIO block once its mux bit is set; ...
 * dev/MEASURED-gpio-cortina-ca77xx.c.md sec 2. */
static int ca77xx_gpio_request(struct gpio_chip *gc, unsigned int off)
{
	struct ca77xx_gpio *cg = gpiochip_get_data(gc);
	unsigned long flags;

	if (cg->mux) {
		spin_lock_irqsave(&cg->lock, flags);
		writel(readl(cg->mux) | BIT(off), cg->mux);
		spin_unlock_irqrestore(&cg->lock, flags);
	}
	return 0;
}

static int ca77xx_gpio_get_direction(struct gpio_chip *gc, unsigned int off)
{
	struct ca77xx_gpio *cg = gpiochip_get_data(gc);

	if (readl(cg->base + CA77XX_GPIO_CFG) & BIT(off))
		return GPIO_LINE_DIRECTION_IN;
	return GPIO_LINE_DIRECTION_OUT;
}

static int ca77xx_gpio_direction_input(struct gpio_chip *gc, unsigned int off)
{
	struct ca77xx_gpio *cg = gpiochip_get_data(gc);

	ca77xx_set_bit(cg, CA77XX_GPIO_CFG, BIT(off));
	return 0;
}

static int ca77xx_gpio_set(struct gpio_chip *gc, unsigned int off, int val)
{
	struct ca77xx_gpio *cg = gpiochip_get_data(gc);

	if (val)
		ca77xx_set_bit(cg, CA77XX_GPIO_OUT, BIT(off));
	else
		ca77xx_clr_bit(cg, CA77XX_GPIO_OUT, BIT(off));
	return 0;
}

static int ca77xx_gpio_direction_output(struct gpio_chip *gc, unsigned int off,
					int val)
{
	struct ca77xx_gpio *cg = gpiochip_get_data(gc);

	/* drive the wanted level first, so the pad never glitches through the
	 * stale OUT value in the instant it becomes an output */
	ca77xx_gpio_set(gc, off, val);
	ca77xx_clr_bit(cg, CA77XX_GPIO_CFG, BIT(off));
	return 0;
}

static int ca77xx_gpio_get(struct gpio_chip *gc, unsigned int off)
{
	struct ca77xx_gpio *cg = gpiochip_get_data(gc);
	unsigned int reg = CA77XX_GPIO_IN;

	/* An output pin's IN word tracks the pad, which for an open-drain or
	 * loaded net can lag what we asked for; read back what we drive. */
	if (!(readl(cg->base + CA77XX_GPIO_CFG) & BIT(off)))
		reg = CA77XX_GPIO_OUT;

	return !!(readl(cg->base + reg) & BIT(off));
}

static int ca77xx_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ca77xx_gpio *cg;
	struct resource *res;
	u32 ngpios;

	cg = devm_kzalloc(dev, sizeof(*cg), GFP_KERNEL);
	if (!cg)
		return -ENOMEM;

	spin_lock_init(&cg->lock);

	/* devm_ioremap(), not devm_ioremap_resource(): the GPON ...
	 * dev/MEASURED-gpio-cortina-ca77xx.c.md sec 3. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	cg->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!cg->base)
		return -ENOMEM;

	/* second window = this group's pin-mux word; optional, a group whose
	 * pads are already routed to GPIO does not need it */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		cg->mux = devm_ioremap(dev, res->start, resource_size(res));
		if (!cg->mux)
			return -ENOMEM;
	}

	if (device_property_read_u32(dev, "ngpios", &ngpios))
		ngpios = CA77XX_GPIO_PINS;
	if (!ngpios || ngpios > CA77XX_GPIO_PINS)
		return dev_err_probe(dev, -EINVAL, "ngpios %u out of range\n",
				     ngpios);

	cg->gc.label = dev_name(dev);
	cg->gc.parent = dev;
	cg->gc.owner = THIS_MODULE;
	cg->gc.base = -1;
	cg->gc.ngpio = ngpios;
	cg->gc.can_sleep = false;
	cg->gc.request = ca77xx_gpio_request;
	cg->gc.get_direction = ca77xx_gpio_get_direction;
	cg->gc.direction_input = ca77xx_gpio_direction_input;
	cg->gc.direction_output = ca77xx_gpio_direction_output;
	cg->gc.get = ca77xx_gpio_get;
	cg->gc.set = ca77xx_gpio_set;

	return devm_gpiochip_add_data(dev, &cg->gc, cg);
}

static const struct of_device_id ca77xx_gpio_of_match[] = {
	{ .compatible = "cortina,ca77xx-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, ca77xx_gpio_of_match);

static struct platform_driver ca77xx_gpio_driver = {
	.driver = {
		.name		= "gpio-cortina-ca77xx",
		.of_match_table	= ca77xx_gpio_of_match,
	},
	.probe	= ca77xx_gpio_probe,
};
module_platform_driver(ca77xx_gpio_driver);

MODULE_DESCRIPTION("Cortina-Access CA77xx GPIO controller");
MODULE_LICENSE("GPL");
