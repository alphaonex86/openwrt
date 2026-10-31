// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SoC temperature sensor of the RTL9607F (Cortina-Access ca77xx family), as a thermal zone.
 *
 * Window (stock DT `cortina,thermal-sensor`, 0xf4320294): CTRL_A 0x0, CTRL_B 0x4, CTRL_C 0x8, DATA 0xc.
 * DATA holds whole degrees in bits 17:10 and a 10-bit fraction in bits 9:0 that the stock driver adds as
 * millidegrees as is; bit 18 is the sign. The stock reading subtracts the board's `deviation`
 * (millidegrees, 2000 on the X400AXF); ours computes the same number, so both firmwares compare.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define TS_CTRL_A	0x0
#define TS_CTRL_B	0x4
#define TS_CTRL_C	0x8
#define TS_DATA		0xc

#define TS_SIGN		BIT(18)
#define TS_WHOLE	GENMASK(17, 10)
#define TS_FRACTION	GENMASK(9, 0)

/* the RTL9607F (chip id 0x6900) start sequence: sensor held, then released */
#define TS_CTRL_C_HOLD	0x2102011e
#define TS_CTRL_C_RUN	0x2102011f
#define TS_SETTLE_MS	300

struct cortina_thermal {
	void __iomem *base;
	s32 deviation;
};

static int cortina_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct cortina_thermal *ts = thermal_zone_device_priv(tz);
	u32 data = readl(ts->base + TS_DATA);
	int whole = FIELD_GET(TS_WHOLE, data);
	int fraction = FIELD_GET(TS_FRACTION, data);

	if (data & TS_SIGN) {
		whole -= 256;
		fraction = -fraction;
	}
	*temp = whole * 1000 + fraction - ts->deviation;
	return 0;
}

static const struct thermal_zone_device_ops cortina_thermal_ops = {
	.get_temp = cortina_thermal_get_temp,
};

static int cortina_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_zone_device *tz;
	struct cortina_thermal *ts;
	u32 ctrl_a = 0x07f80000, ctrl_b = 0x00374000;
	int temp;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ts->base))
		return PTR_ERR(ts->base);
	of_property_read_u32(dev->of_node, "tms-reg-a", &ctrl_a);
	of_property_read_u32(dev->of_node, "tms-reg-b", &ctrl_b);
	of_property_read_s32(dev->of_node, "deviation", &ts->deviation);

	writel(ctrl_a, ts->base + TS_CTRL_A);
	writel(ctrl_b, ts->base + TS_CTRL_B);
	writel(TS_CTRL_C_HOLD, ts->base + TS_CTRL_C);
	writel(TS_CTRL_C_RUN, ts->base + TS_CTRL_C);
	msleep(TS_SETTLE_MS);

	tz = devm_thermal_of_zone_register(dev, 0, ts, &cortina_thermal_ops);
	if (IS_ERR(tz))
		return dev_err_probe(dev, PTR_ERR(tz), "thermal zone not registered\n");
	if (!cortina_thermal_get_temp(tz, &temp))
		dev_info(dev, "SoC %d mC (deviation %d mC)\n", temp, ts->deviation);
	return 0;
}

static const struct of_device_id cortina_thermal_of_match[] = {
	{ .compatible = "cortina,thermal-sensor" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_thermal_of_match);

static struct platform_driver cortina_thermal_driver = {
	.probe = cortina_thermal_probe,
	.driver = {
		.name = "cortina-thermal",
		.of_match_table = cortina_thermal_of_match,
	},
};
module_platform_driver(cortina_thermal_driver);

MODULE_DESCRIPTION("RTL9607F SoC temperature sensor");
MODULE_LICENSE("GPL");
