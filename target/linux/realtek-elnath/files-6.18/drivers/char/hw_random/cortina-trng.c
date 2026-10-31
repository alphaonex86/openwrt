// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * True random number generator of the RTL9607F (Cortina-Access ca77xx family), as a hwrng.
 *
 * Block (stock DT `realtek,trng`, 0x90600200, SPI 62): CTRL 0x00, READY 0x1c bit 0, one 32-bit
 * word per ready at 0x28, interrupt status 0x54 / enable 0x64. A parity error needs a soft reset.
 * Measured 2026-10-30 on the X400AXF, 2 MB each: the reset value bypasses the LFSR (CTRL bit 16) and
 * that raw stream fails FIPS 140-2 in 798/819 blocks (0xaa/0x55 dominate) while still holding about
 * 0.87 bit of min-entropy per bit; with the LFSR on, 1/819 fails, the normal rate. So the LFSR runs
 * and the core is credited half a bit per bit. The self-test (bit 18) stalls the output (READY stays
 * low), so it stays off.
 */
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define TRNG_CTRL		0x00
#define  TRNG_CTRL_SRST		BIT(0)
#define  TRNG_CTRL_CALI_SRST	BIT(3)
#define  TRNG_CTRL_LFSR_BYPASS	BIT(16)
#define  TRNG_CTRL_SELF_TEST	BIT(18)
#define TRNG_READY		0x1c
#define  TRNG_READY_OUT		BIT(0)
#define TRNG_RESULT		0x28
#define TRNG_INT		0x54
#define TRNG_INT_EN		0x64
#define  TRNG_INT_PARITY	BIT(2)

#define TRNG_READY_TIMEOUT_US	1000
#define TRNG_QUALITY		512	/* entropy bits per 1024 output bits */

struct cortina_trng {
	struct hwrng rng;
	void __iomem *base;
	spinlock_t lock;	/* a parity reset never cuts a word read in half */
	struct device *dev;
	unsigned long parity_resets;
};

static void cortina_trng_reset(struct cortina_trng *t)
{
	u32 ctrl = readl(t->base + TRNG_CTRL) & ~(TRNG_CTRL_SELF_TEST | TRNG_CTRL_LFSR_BYPASS);

	writel(ctrl | TRNG_CTRL_SRST | TRNG_CTRL_CALI_SRST, t->base + TRNG_CTRL);
	udelay(10);
	writel(ctrl & ~(TRNG_CTRL_SRST | TRNG_CTRL_CALI_SRST), t->base + TRNG_CTRL);
}

static int cortina_trng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct cortina_trng *t = container_of(rng, struct cortina_trng, rng);
	size_t done = 0;
	unsigned long flags;
	u32 ready, word;

	while (max - done >= sizeof(word)) {
		if (wait)
			readl_poll_timeout_atomic(t->base + TRNG_READY, ready, ready & TRNG_READY_OUT,
						  0, TRNG_READY_TIMEOUT_US);
		spin_lock_irqsave(&t->lock, flags);
		ready = readl(t->base + TRNG_READY);
		if (ready & TRNG_READY_OUT)
			word = readl(t->base + TRNG_RESULT);
		spin_unlock_irqrestore(&t->lock, flags);
		if (!(ready & TRNG_READY_OUT))
			break;
		memcpy(buf + done, &word, sizeof(word));
		done += sizeof(word);
	}
	if (!done && wait) {
		dev_err_ratelimited(t->dev, "no output within %d us\n", TRNG_READY_TIMEOUT_US);
		return -ETIMEDOUT;
	}
	return done;
}

static irqreturn_t cortina_trng_irq(int irq, void *data)
{
	struct cortina_trng *t = data;
	u32 status = readl(t->base + TRNG_INT);

	if (!status)
		return IRQ_NONE;
	spin_lock(&t->lock);
	if (status & TRNG_INT_PARITY) {
		cortina_trng_reset(t);
		t->parity_resets++;
	}
	writel(status, t->base + TRNG_INT);
	spin_unlock(&t->lock);
	dev_warn_ratelimited(t->dev, "interrupt status %#x, %lu parity reset(s)\n",
			     status, t->parity_resets);
	return IRQ_HANDLED;
}

static int cortina_trng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cortina_trng *t;
	u32 ready;
	int irq, ret;

	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	t->dev = dev;
	spin_lock_init(&t->lock);
	t->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(t->base))
		return PTR_ERR(t->base);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	writel(0, t->base + TRNG_INT_EN);
	cortina_trng_reset(t);
	writel(readl(t->base + TRNG_INT), t->base + TRNG_INT);
	ret = devm_request_irq(dev, irq, cortina_trng_irq, 0, dev_name(dev), t);
	if (ret)
		return dev_err_probe(dev, ret, "irq %d\n", irq);
	writel(TRNG_INT_PARITY, t->base + TRNG_INT_EN);

	ret = readl_poll_timeout(t->base + TRNG_READY, ready, ready & TRNG_READY_OUT,
				 10, TRNG_READY_TIMEOUT_US);
	if (ret)
		return dev_err_probe(dev, ret, "no output after reset (ready %#x, ctrl %#x)\n",
				     ready, readl(t->base + TRNG_CTRL));

	t->rng.name = dev_name(dev);
	t->rng.read = cortina_trng_read;
	t->rng.quality = TRNG_QUALITY;
	ret = devm_hwrng_register(dev, &t->rng);
	if (ret)
		return dev_err_probe(dev, ret, "hwrng register\n");
	dev_info(dev, "ready, ctrl %#x\n", readl(t->base + TRNG_CTRL));
	return 0;
}

static const struct of_device_id cortina_trng_of_match[] = {
	{ .compatible = "cortina,trng" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_trng_of_match);

static struct platform_driver cortina_trng_driver = {
	.probe = cortina_trng_probe,
	.driver = {
		.name = "cortina-trng",
		.of_match_table = cortina_trng_of_match,
	},
};
module_platform_driver(cortina_trng_driver);

MODULE_DESCRIPTION("RTL9607F (Cortina-Access ca77xx) true random number generator");
MODULE_LICENSE("GPL");
