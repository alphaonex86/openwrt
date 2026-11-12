// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/*
 * ZTE ZX279128 GPIO: one device tree node per bank of 16 lines, banks every 0x40 from 0x94404000.
 * Registers as the unit's own stock kernel drives them: direction (1 = output), line levels, and
 * write-1-to-set / write-1-to-clear output registers, so a line changes without touching the others.
 */
#include <linux/gpio/driver.h>
#include <linux/gpio/generic.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

enum {
	ZX_GPIO_DIR = 0x00,
	ZX_GPIO_LEVEL = 0x14,
	ZX_GPIO_SET = 0x18,
	ZX_GPIO_CLR = 0x1c,
	ZX_GPIO_LINES = 16,
};

static int zx279128_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_generic_chip *chip;
	void __iomem *base;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	chip->gc.ngpio = ZX_GPIO_LINES;
	ret = gpio_generic_chip_init(chip, &(struct gpio_generic_chip_config) {
		.dev = dev,
		.sz = 4,
		.dat = base + ZX_GPIO_LEVEL,
		.set = base + ZX_GPIO_SET,
		.clr = base + ZX_GPIO_CLR,
		.dirout = base + ZX_GPIO_DIR,
	});
	if (ret)
		return dev_err_probe(dev, ret, "generic GPIO chip init\n");

	ret = devm_gpiochip_add_data(dev, &chip->gc, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "GPIO chip registration\n");
	return 0;
}

static const struct of_device_id zx279128_gpio_of_match[] = {
	{ .compatible = "zte,zx279128-gpio" },
	{ }
};

static struct platform_driver zx279128_gpio_driver = {
	.driver = {
		.name = "zx279128-gpio",
		.of_match_table = zx279128_gpio_of_match,
	},
	.probe = zx279128_gpio_probe,
};
builtin_platform_driver(zx279128_gpio_driver);
