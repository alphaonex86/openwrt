// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE */
/*
 * Bus timeout monitors: a bus slave that never answers stops the CPU for good, with no trace. An armed monitor ends
 * the access after its timeout, latches the address and raises an interrupt; this driver logs which monitor, the
 * address and the interrupted code (rate-limited), counts the trip in sysfs (trips, last_address), acknowledges it,
 * and the unit goes on. The device tree gives everything of the SoC's: the register blocks (each split into monitors
 * every monitor-stride bytes), the control word's enable mask, timeout mask and armed timeout, the status word's
 * offset and pending bit (write 1 to clear), the latched address's offset, one name per monitor, and the interrupt.
 */

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <asm/irq_regs.h>

#define BTM_MAX		32

struct btm {
	void __iomem *mon[BTM_MAX];
	const char *name[BTM_MAX];
	int n;
	u32 enable, tmask, timeout, status, pending, address;
	atomic_t trips;
	u32 last_address;
};

static irqreturn_t btm_irq(int irq, void *data)
{
	struct btm *t = data;
	struct pt_regs *regs = get_irq_regs();
	int i, hit = 0;

	for (i = 0; i < t->n; i++) {
		u32 addr;
		int trip;

		if (!(readl(t->mon[i] + t->status) & t->pending))
			continue;
		addr = readl(t->mon[i] + t->address);
		writel(t->pending, t->mon[i] + t->status);
		WRITE_ONCE(t->last_address, addr);
		trip = atomic_inc_return(&t->trips);	/* counted outside the log: a suppressed line never evaluates it */
		/* the interrupt follows the ended access: the EPC is at it, or later if interrupts were off */
		pr_emerg_ratelimited("bus timeout: %s monitor: no answer from 0x%08x within its timeout, epc %pS (trip %d)\n",
				     t->name[i], addr, regs ? (void *)instruction_pointer(regs) : NULL, trip);
		hit++;
	}
	return hit ? IRQ_HANDLED : IRQ_NONE;
}

static ssize_t trips_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", atomic_read(&((struct btm *)dev_get_drvdata(dev))->trips));
}
static DEVICE_ATTR_RO(trips);

static ssize_t last_address_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "0x%08x\n", READ_ONCE(((struct btm *)dev_get_drvdata(dev))->last_address));
}
static DEVICE_ATTR_RO(last_address);

static struct attribute *btm_attrs[] = { &dev_attr_trips.attr, &dev_attr_last_address.attr, NULL };
ATTRIBUTE_GROUPS(btm);

static int btm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct btm *t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	u32 ctl[3], st[2], stride;
	int blk, i, irq, ret;

	if (!t)
		return -ENOMEM;
	if (of_property_read_u32_array(np, "monitor-control", ctl, 3) ||
	    of_property_read_u32_array(np, "monitor-status", st, 2) ||
	    of_property_read_u32(np, "monitor-address", &t->address) ||
	    of_property_read_u32(np, "monitor-stride", &stride) || !stride)
		return dev_err_probe(dev, -EINVAL, "monitor-control, -status, -address or -stride missing\n");
	t->enable = ctl[0];
	t->tmask = ctl[1];
	t->timeout = ctl[2];
	t->status = st[0];
	t->pending = st[1];
	for (blk = 0; blk < pdev->num_resources; blk++) {
		struct resource *r = platform_get_resource(pdev, IORESOURCE_MEM, blk);
		void __iomem *base;

		if (!r)
			break;
		base = devm_ioremap_resource(dev, r);
		if (IS_ERR(base))
			return PTR_ERR(base);
		for (i = 0; i * stride < resource_size(r) && t->n < BTM_MAX; i++)
			t->mon[t->n++] = base + i * stride;
	}
	if (!t->n || of_property_read_string_array(np, "monitor-names", t->name, t->n) != t->n)
		return dev_err_probe(dev, -EINVAL, "%d monitor(s), not one name each\n", t->n);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	platform_set_drvdata(pdev, t);
	for (i = 0; i < t->n; i++)
		writel(t->pending, t->mon[i] + t->status);
	ret = devm_request_irq(dev, irq, btm_irq, 0, "bus-timeout", t);
	if (ret)
		return dev_err_probe(dev, ret, "interrupt %d\n", irq);
	for (i = 0; i < t->n; i++)
		writel((readl(t->mon[i]) & ~t->tmask) | t->enable | t->timeout, t->mon[i]);
	dev_info(dev, "%d monitors armed (control 0x%08x), interrupt %d\n", t->n, t->enable | t->timeout, irq);
	return 0;
}

static const struct of_device_id btm_of_match[] = {
	{ .compatible = "bus-timeout-monitor" },
	{ }
};

static struct platform_driver btm_driver = {
	.probe = btm_probe,
	.driver = {
		.name = "bus-timeout-monitor",
		.of_match_table = btm_of_match,
		.dev_groups = btm_groups,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(btm_driver);
