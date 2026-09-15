// SPDX-License-Identifier: GPL-2.0-or-later
/* Cortina-Access peripheral watchdog (PER_WDT). Found on the ...
 * dev/MEASURED-cortina-wdt.c.md sec 1. */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>

/* --- PER_WDT block, offsets from the block base (0xf432902c on RTL9607F) - */
#define PER_WDT_CTRL			0x00
#define  PER_WDT_CTRL_WDTEN		BIT(0)	/* counter runs		*/
#define  PER_WDT_CTRL_RSTEN		BIT(1)	/* expiry asserts reset	*/
#define  PER_WDT_CTRL_CLKSEL		BIT(2)	/* 0: DIV output, 1: 1 ms tick */
#define  PER_WDT_CTRL_DELAY		GENMASK(31, 12)
#define  PER_WDT_CTRL_DELAY_SHIFT	12
#define PER_WDT_PS			0x04	/* prescaler reload	*/
#define PER_WDT_DIV			0x08	/* prescaler divider	*/
#define PER_WDT_LD			0x0c	/* counter reload	*/
#define PER_WDT_LOADE			0x10
#define  PER_WDT_LOADE_WDT		BIT(0)	/* reload the counter	*/
#define  PER_WDT_LOADE_PRE		BIT(1)	/* reload the prescaler	*/
#define PER_WDT_CNT			0x14	/* live count		*/
#define PER_WDT_IE_0			0x18
#define  PER_WDT_IE_0_WDTE		BIT(0)
#define PER_WDT_INT_0			0x1c
#define  PER_WDT_INT_0_WDTI		BIT(0)
#define PER_WDT_STAT_0			0x20
#define PER_WDT_BLOCK_SIZE		0x30	/* CTRL..STAT_1, 12 registers */

/* GLOBAL_GLOBAL_CONFIG, the SoC reset glue (a single 32-bit ...
 * dev/MEASURED-cortina-wdt.c.md sec 2. */
#define GLOBAL_WD_RESET_SUBSYS_ENABLE	BIT(4)
#define GLOBAL_WD_RESET_PCIE		BIT(5)
#define GLOBAL_WD_RESET_ALL_BLOCKS	BIT(6)
#define GLOBAL_WD_RESET_REMAP		BIT(7)
#define GLOBAL_WD_RESET_EXT_RESET	BIT(8)
#define GLOBAL_WD_RESET_ENABLES		(GLOBAL_WD_RESET_SUBSYS_ENABLE | \
					 GLOBAL_WD_RESET_PCIE | \
					 GLOBAL_WD_RESET_ALL_BLOCKS | \
					 GLOBAL_WD_RESET_REMAP | \
					 GLOBAL_WD_RESET_EXT_RESET)

/* The prescaler is programmed for a 1 ms tick, and ... -- dev/MEASURED-cortina-wdt.c.md sec 13. */
#define PER_WDT_TICK_HZ			1000		/* the prescaled tick */
#define PER_WDT_DIV_PER_SECOND		1000		/* ticks per second   */

/* DELAY_RESET is expressed in units of a 16 kHz clock derived from the APB. */
#define PER_WDT_DELAY_HZ		16000

#define CORTINA_WDT_DEFAULT_TIMEOUT	60
#define CORTINA_WDT_MIN_TIMEOUT		1

/* The counter is 32 bits wide and, in seconds mode, counts ...
 * dev/MEASURED-cortina-wdt.c.md sec 14. */
#define CORTINA_WDT_MAX_TIMEOUT		(UINT_MAX / 1000)

static unsigned int timeout;
module_param(timeout, uint, 0444);
MODULE_PARM_DESC(timeout,
		 "Watchdog timeout in seconds (default: device tree, else "
		 __MODULE_STRING(CORTINA_WDT_DEFAULT_TIMEOUT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0444);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default: "
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/* struct cortina_wdt - one PER_WDT instance @wdd: the ...
 * dev/MEASURED-cortina-wdt.c.md sec 3. */
struct cortina_wdt {
	struct watchdog_device	wdd;
	void __iomem		*base;
	void __iomem		*rstcfg;
	struct clk		*clk;
	u32			prescaler;
	u32			delay;
	bool			reset;
	bool			irq_armed;
	spinlock_t		lock;		/* see kernel-doc above */
};

static inline struct cortina_wdt *to_cortina_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct cortina_wdt, wdd);
}

/* Load the counter and (re)start it. @count is in whole ...
 * dev/MEASURED-cortina-wdt.c.md sec 4. */
static void cortina_wdt_program(struct cortina_wdt *wdt, u32 count,
				bool millisecond, bool with_delay)
{
	u32 ctrl = PER_WDT_CTRL_WDTEN;

	if (wdt->reset)
		ctrl |= PER_WDT_CTRL_RSTEN;
	if (millisecond)
		ctrl |= PER_WDT_CTRL_CLKSEL;
	if (with_delay)
		ctrl |= (wdt->delay << PER_WDT_CTRL_DELAY_SHIFT) &
			PER_WDT_CTRL_DELAY;

	writel(wdt->prescaler, wdt->base + PER_WDT_PS);
	writel(millisecond ? 0 : PER_WDT_DIV_PER_SECOND, wdt->base + PER_WDT_DIV);
	writel(count, wdt->base + PER_WDT_LD);
	/* The expiry event is enabled even on a board that takes no ...
	 * dev/MEASURED-cortina-wdt.c.md sec 5. */
	writel(PER_WDT_IE_0_WDTE, wdt->base + PER_WDT_IE_0);
	writel(ctrl, wdt->base + PER_WDT_CTRL);
	writel(PER_WDT_LOADE_WDT | PER_WDT_LOADE_PRE, wdt->base + PER_WDT_LOADE);
}

static int cortina_wdt_start(struct watchdog_device *wdd)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	cortina_wdt_program(wdt, wdd->timeout, false, true);
	spin_unlock_irqrestore(&wdt->lock, flags);

	set_bit(WDOG_HW_RUNNING, &wdd->status);

	return 0;
}

static int cortina_wdt_stop(struct watchdog_device *wdd)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);
	unsigned long flags;
	u32 ctrl;

	spin_lock_irqsave(&wdt->lock, flags);
	ctrl = readl(wdt->base + PER_WDT_CTRL);
	writel(ctrl & ~PER_WDT_CTRL_WDTEN, wdt->base + PER_WDT_CTRL);
	spin_unlock_irqrestore(&wdt->lock, flags);

	clear_bit(WDOG_HW_RUNNING, &wdd->status);

	return 0;
}

static int cortina_wdt_ping(struct watchdog_device *wdd)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);

	writel(PER_WDT_LOADE_WDT | PER_WDT_LOADE_PRE, wdt->base + PER_WDT_LOADE);

	return 0;
}

static int cortina_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int new_timeout)
{
	wdd->timeout = new_timeout;

	/* Reprogram only when the counter is already running. ...
	 * dev/MEASURED-cortina-wdt.c.md sec 15. */
	if (watchdog_hw_running(wdd))
		return cortina_wdt_start(wdd);

	return 0;
}

static unsigned int cortina_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);

	return readl(wdt->base + PER_WDT_CNT);
}

/* Last-resort machine restart. Registered at the default ...
 * dev/MEASURED-cortina-wdt.c.md sec 6. */
static int cortina_wdt_restart(struct watchdog_device *wdd,
			       unsigned long action, void *data)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	cortina_wdt_program(wdt, 1, true, false);
	spin_unlock_irqrestore(&wdt->lock, flags);

	mdelay(1000);

	return 0;
}

/* Only reached on a board that wires the expiry interrupt AND ...
 * dev/MEASURED-cortina-wdt.c.md sec 7. */
static irqreturn_t cortina_wdt_isr(int irq, void *dev_id)
{
	struct cortina_wdt *wdt = dev_id;

	writel(PER_WDT_INT_0_WDTI, wdt->base + PER_WDT_INT_0);
	writel(0, wdt->base + PER_WDT_IE_0);

	dev_crit(wdt->wdd.parent,
		 "watchdog expired and this board has no reset-on-timeout\n");

	return IRQ_HANDLED;
}

static const struct watchdog_info cortina_wdt_info = {
	.identity	= "Cortina PER_WDT",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops cortina_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= cortina_wdt_start,
	.stop		= cortina_wdt_stop,
	.ping		= cortina_wdt_ping,
	.set_timeout	= cortina_wdt_set_timeout,
	.get_timeleft	= cortina_wdt_get_timeleft,
	.restart	= cortina_wdt_restart,
};

/* Turn an expired counter into a chip reset. These bits are ...
 * dev/MEASURED-cortina-wdt.c.md sec 8. */
static void cortina_wdt_enable_soc_reset(struct cortina_wdt *wdt)
{
	u32 val = readl(wdt->rstcfg);

	writel(val | GLOBAL_WD_RESET_ENABLES, wdt->rstcfg);
}

static int cortina_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cortina_wdt *wdt;
	struct resource *res;
	resource_size_t phys;
	unsigned long rate;
	u32 ctrl, adopted_s = 0;
	bool adopted;
	int irq, ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	spin_lock_init(&wdt->lock);

	/* devm_ioremap(), not devm_ioremap_resource(): both windows ...
	 * dev/MEASURED-cortina-wdt.c.md sec 16. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL,
				     "missing the PER_WDT register window\n");
	if (resource_size(res) < PER_WDT_BLOCK_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "PER_WDT window is %pa, need at least %#x\n",
				     &res->start, PER_WDT_BLOCK_SIZE);
	phys = res->start;
	wdt->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!wdt->base)
		return -ENOMEM;

	wdt->reset = of_property_read_bool(dev->of_node, "reset-on-timeout");
	if (wdt->reset) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res)
			return dev_err_probe(dev, -EINVAL,
					     "reset-on-timeout needs the SoC reset-config window\n");
		wdt->rstcfg = devm_ioremap(dev, res->start, resource_size(res));
		if (!wdt->rstcfg)
			return -ENOMEM;
	}

	wdt->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(wdt->clk))
		return dev_err_probe(dev, PTR_ERR(wdt->clk),
				     "no input clock\n");

	rate = clk_get_rate(wdt->clk);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "input clock reports 0 Hz\n");
	wdt->prescaler = rate / PER_WDT_TICK_HZ - 1;

	/* Optional: how long the block waits between raising the ...
	 * dev/MEASURED-cortina-wdt.c.md sec 17. */
	if (!of_property_read_u32(dev->of_node, "delay-reset", &wdt->delay)) {
		u64 cycles = (u64)wdt->delay * (rate / PER_WDT_DELAY_HZ);

		wdt->delay = min_t(u64, cycles,
				   PER_WDT_CTRL_DELAY >> PER_WDT_CTRL_DELAY_SHIFT);
	}

	/* The interrupt is optional and only matters without ...
	 * dev/MEASURED-cortina-wdt.c.md sec 18. */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, cortina_wdt_isr, 0,
				       dev_name(dev), wdt);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot take the expiry interrupt\n");
		wdt->irq_armed = true;
	}

	wdt->wdd.info = &cortina_wdt_info;
	wdt->wdd.ops = &cortina_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = CORTINA_WDT_MIN_TIMEOUT;
	wdt->wdd.max_timeout = CORTINA_WDT_MAX_TIMEOUT;
	wdt->wdd.timeout = CORTINA_WDT_DEFAULT_TIMEOUT;

	/* ★ Was it already counting when Linux took over? A ...
	 * dev/MEASURED-cortina-wdt.c.md sec 9. */
	ctrl = readl(wdt->base + PER_WDT_CTRL);
	adopted = ctrl & PER_WDT_CTRL_WDTEN;
	if (adopted) {
		u32 load = readl(wdt->base + PER_WDT_LD);

		adopted_s = (ctrl & PER_WDT_CTRL_CLKSEL) ? load / PER_WDT_TICK_HZ
							 : load;
	}

	/* watchdog_init_timeout() reports failure ONLY when a value ...
	 * dev/MEASURED-cortina-wdt.c.md sec 10. */
	ret = watchdog_init_timeout(&wdt->wdd, timeout, dev);
	if (ret)
		dev_warn(dev, "timeout out of range, keeping %u s\n",
			 wdt->wdd.timeout);

	/* Deliberately NO watchdog_stop_on_reboot(). The watchdog ...
	 * dev/MEASURED-cortina-wdt.c.md sec 11. */
	watchdog_set_nowayout(&wdt->wdd, nowayout);

	if (wdt->reset)
		cortina_wdt_enable_soc_reset(wdt);

	if (adopted) {
		/* Take the window OVER rather than inherit it. Two reasons, ...
		 * dev/MEASURED-cortina-wdt.c.md sec 12. */
		cortina_wdt_start(&wdt->wdd);
	}

	platform_set_drvdata(pdev, wdt);

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;

	if (adopted)
		dev_info(dev,
			 "PER_WDT at %pa: %u s timeout, reset-on-timeout %s, expiry irq %s, ALREADY RUNNING at kernel entry (inherited %u s window, reprogrammed to %u s)%s\n",
			 &phys, wdt->wdd.timeout, wdt->reset ? "on" : "off",
			 wdt->irq_armed ? "taken" : "not wired",
			 adopted_s, wdt->wdd.timeout,
			 nowayout ? ", nowayout" : "");
	else
		dev_info(dev,
			 "PER_WDT at %pa: %u s timeout, reset-on-timeout %s, expiry irq %s, stopped at kernel entry%s\n",
			 &phys, wdt->wdd.timeout, wdt->reset ? "on" : "off",
			 wdt->irq_armed ? "taken" : "not wired",
			 nowayout ? ", nowayout" : "");

	return 0;
}

static const struct of_device_id cortina_wdt_of_match[] = {
	{ .compatible = "cortina,wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_wdt_of_match);

static struct platform_driver cortina_wdt_driver = {
	.probe	= cortina_wdt_probe,
	.driver	= {
		.name		= "cortina-wdt",
		.of_match_table	= cortina_wdt_of_match,
	},
};
module_platform_driver(cortina_wdt_driver);

MODULE_DESCRIPTION("Cortina-Access peripheral watchdog");
MODULE_LICENSE("GPL");
