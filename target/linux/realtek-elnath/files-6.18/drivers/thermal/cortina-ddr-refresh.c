// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DDR refresh-rate derating of the RTL9607F, as a thermal cooling device.
 *
 * The DDR controller is secure-only (its window reads 0 from Linux), so the refresh interval belongs to
 * the secure firmware. SiP call 0x82001108 sets DRR tREF (bits 23:8) to the interval it cached at its
 * first call divided by x3; x3 = 0 changes nothing, x1 and x2 are ignored, x0 returns 0. It does not
 * bound the divisor: the thermal core keeps every state within max_state, so only 1, 2 and 4 are sent.
 * State n refreshes 2^n times as often.
 */
#include <linux/arm-smccc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define DDR_REFRESH_SIP		0x82001108
#define DDR_REFRESH_MAX_STATE	2

struct ddr_refresh {
	struct device *dev;
	unsigned long state;
};

static int ddr_refresh_set_divisor(struct device *dev, unsigned long divisor)
{
	struct arm_smccc_res res;

	arm_smccc_1_1_invoke(DDR_REFRESH_SIP, 1, 0, divisor, &res);
	if (res.a0) {
		dev_err(dev, "secure firmware refused refresh divisor %lu: %ld\n", divisor, (long)res.a0);
		return -EIO;
	}
	return 0;
}

static int ddr_refresh_get_max_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = DDR_REFRESH_MAX_STATE;
	return 0;
}

static int ddr_refresh_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct ddr_refresh *ddr = cdev->devdata;

	*state = ddr->state;
	return 0;
}

static int ddr_refresh_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct ddr_refresh *ddr = cdev->devdata;
	int ret;

	if (state == ddr->state)
		return 0;
	ret = ddr_refresh_set_divisor(ddr->dev, 1UL << state);
	if (ret)
		return ret;
	ddr->state = state;
	dev_dbg(ddr->dev, "refresh x%lu\n", 1UL << state);
	return 0;
}

static const struct thermal_cooling_device_ops ddr_refresh_ops = {
	.get_max_state = ddr_refresh_get_max_state,
	.get_cur_state = ddr_refresh_get_cur_state,
	.set_cur_state = ddr_refresh_set_cur_state,
};

static int ddr_refresh_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_cooling_device *cdev;
	struct ddr_refresh *ddr;
	int ret;

	ddr = devm_kzalloc(dev, sizeof(*ddr), GFP_KERNEL);
	if (!ddr)
		return -ENOMEM;
	ddr->dev = dev;

	/* the boot interval: proves the call exists and makes state 0 true */
	ret = ddr_refresh_set_divisor(dev, 1);
	if (ret)
		return ret;

	cdev = devm_thermal_of_cooling_device_register(dev, dev->of_node, "ddr-refresh", ddr, &ddr_refresh_ops);
	if (IS_ERR(cdev))
		return dev_err_probe(dev, PTR_ERR(cdev), "cooling device not registered\n");
	dev_info(dev, "refresh derating ready, x1 now\n");
	return 0;
}

static const struct of_device_id ddr_refresh_of_match[] = {
	{ .compatible = "cortina,ddr-refresh-sip" },
	{ }
};
MODULE_DEVICE_TABLE(of, ddr_refresh_of_match);

static struct platform_driver ddr_refresh_driver = {
	.probe = ddr_refresh_probe,
	.driver = {
		.name = "cortina-ddr-refresh",
		.of_match_table = ddr_refresh_of_match,
	},
};
module_platform_driver(ddr_refresh_driver);

MODULE_DESCRIPTION("RTL9607F DDR refresh derating by temperature");
MODULE_LICENSE("GPL");
