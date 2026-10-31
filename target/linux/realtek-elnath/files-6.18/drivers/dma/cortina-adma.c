// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Memory-to-memory DMA engine ("LDMA") of the RTL9607F (Cortina-Access ca77xx family), as a dmaengine
 * memcpy provider. The vendor firmware describes it (stock DT `cortina,adma`) and binds no driver.
 *
 * Two channels, 0x40 apart at reg 0; each takes copy commands through three registers and queues 16:
 *   SRC_LO 0x0c (writing it queues the command), CMD 0x04, DST_LO 0x08
 *   CMD 11:0 length - 1 (4 KiB at most), bit 15 interrupt when done, 23:16 dst[39:32], 31:24 src[39:32]
 *   CTRL 0x10 bit 31 enable, bit 28 reset · INT_EN 0x18 / INT 0x1c bit 0 done, bit 1 FIFO overrun
 *   QPTR 0x20 28:24 commands still queued
 * Global (reg 1): +0 bit 31 40-bit addressing, +4 15:0 rate limit (0 = none). Interrupt routing (reg 2):
 * +8*n bit c sends channel c to line n. AXI master (reg 3): 7:0 outstanding transactions.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "virt-dma.h"

#define ADMA_CMD		0x04
#define  ADMA_CMD_LEN		GENMASK(11, 0)
#define  ADMA_CMD_IRQ		BIT(15)
#define  ADMA_CMD_DST_HI	GENMASK(23, 16)
#define  ADMA_CMD_SRC_HI	GENMASK(31, 24)
#define ADMA_DST_LO		0x08
#define ADMA_SRC_LO		0x0c
#define ADMA_CTRL		0x10
#define  ADMA_CTRL_RESET	BIT(28)
#define  ADMA_CTRL_ENABLE	BIT(31)
#define ADMA_INT_EN		0x18
#define ADMA_INT		0x1c
#define  ADMA_INT_DONE		BIT(0)
#define  ADMA_INT_OVERRUN	BIT(1)
#define ADMA_QPTR		0x20
#define  ADMA_QPTR_USED		GENMASK(28, 24)
#define ADMA_CHAN_STRIDE	0x40

#define ADMA_GLB_CTRL		0x00
#define  ADMA_GLB_40BIT		BIT(31)
#define ADMA_GLB_RATE		0x04
#define  ADMA_GLB_RATE_LIMIT	GENMASK(15, 0)
#define ADMA_AXI_OUTSTANDING	GENMASK(7, 0)

#define ADMA_CHANNELS		2
#define ADMA_QUEUE_DEPTH	16
#define ADMA_CHUNK		4096

struct cortina_adma_desc {
	struct virt_dma_desc vd;
	dma_addr_t dst, src;
	size_t len, queued;	/* bytes, and bytes handed to the hardware so far */
};

struct cortina_adma_chan {
	struct virt_dma_chan vc;
	void __iomem *base;
	struct cortina_adma_desc *cur;
	int irq;
};

struct cortina_adma {
	struct dma_device dma;
	struct cortina_adma_chan chan[ADMA_CHANNELS];
};

static struct cortina_adma_chan *to_adma_chan(struct dma_chan *c)
{
	return container_of(to_virt_chan(c), struct cortina_adma_chan, vc);
}

static void cortina_adma_reset(struct cortina_adma_chan *ch)
{
	writel(ADMA_CTRL_RESET, ch->base + ADMA_CTRL);
	udelay(10);
	writel(0, ch->base + ADMA_CTRL);
}

/* Queue as much of the current copy as the hardware has room for; the last command queued interrupts. */
static void cortina_adma_push(struct cortina_adma_chan *ch)
{
	struct cortina_adma_desc *d = ch->cur;
	u32 room = ADMA_QUEUE_DEPTH - 1 - FIELD_GET(ADMA_QPTR_USED, readl(ch->base + ADMA_QPTR));

	writel(ADMA_CTRL_ENABLE, ch->base + ADMA_CTRL);
	while (room-- && d->queued < d->len) {
		dma_addr_t src = d->src + d->queued, dst = d->dst + d->queued;
		size_t n = min_t(size_t, d->len - d->queued, ADMA_CHUNK);
		u32 cmd = FIELD_PREP(ADMA_CMD_LEN, n - 1) | FIELD_PREP(ADMA_CMD_DST_HI, upper_32_bits(dst)) |
			  FIELD_PREP(ADMA_CMD_SRC_HI, upper_32_bits(src));

		d->queued += n;
		if (!room || d->queued == d->len)
			cmd |= ADMA_CMD_IRQ;
		writel(lower_32_bits(dst), ch->base + ADMA_DST_LO);
		writel(cmd, ch->base + ADMA_CMD);
		writel(lower_32_bits(src), ch->base + ADMA_SRC_LO);
	}
}

static void cortina_adma_start_next(struct cortina_adma_chan *ch)
{
	struct virt_dma_desc *vd = vchan_next_desc(&ch->vc);

	ch->cur = NULL;
	if (!vd)
		return;
	list_del(&vd->node);
	ch->cur = container_of(vd, struct cortina_adma_desc, vd);
	cortina_adma_push(ch);
}

static irqreturn_t cortina_adma_irq(int irq, void *data)
{
	struct cortina_adma_chan *ch = data;
	u32 status = readl(ch->base + ADMA_INT);

	if (!status)
		return IRQ_NONE;
	writel(status, ch->base + ADMA_INT);
	if (status & ADMA_INT_OVERRUN)
		dev_err_ratelimited(ch->vc.chan.device->dev, "channel %d: command FIFO overrun\n",
				    ch->vc.chan.chan_id);
	spin_lock(&ch->vc.lock);
	if (ch->cur && (status & ADMA_INT_DONE) && !FIELD_GET(ADMA_QPTR_USED, readl(ch->base + ADMA_QPTR))) {
		if (ch->cur->queued < ch->cur->len) {
			cortina_adma_push(ch);
		} else {
			vchan_cookie_complete(&ch->cur->vd);
			cortina_adma_start_next(ch);
		}
	}
	spin_unlock(&ch->vc.lock);
	return IRQ_HANDLED;
}

static struct dma_async_tx_descriptor *cortina_adma_prep_memcpy(struct dma_chan *c, dma_addr_t dst,
								dma_addr_t src, size_t len, unsigned long flags)
{
	struct cortina_adma_desc *d;

	if (!len)
		return NULL;
	d = kzalloc(sizeof(*d), GFP_NOWAIT);
	if (!d)
		return NULL;
	d->dst = dst;
	d->src = src;
	d->len = len;
	return vchan_tx_prep(&to_adma_chan(c)->vc, &d->vd, flags);
}

static void cortina_adma_issue_pending(struct dma_chan *c)
{
	struct cortina_adma_chan *ch = to_adma_chan(c);
	unsigned long flags;

	spin_lock_irqsave(&ch->vc.lock, flags);
	if (vchan_issue_pending(&ch->vc) && !ch->cur)
		cortina_adma_start_next(ch);
	spin_unlock_irqrestore(&ch->vc.lock, flags);
}

static int cortina_adma_terminate_all(struct dma_chan *c)
{
	struct cortina_adma_chan *ch = to_adma_chan(c);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&ch->vc.lock, flags);
	cortina_adma_reset(ch);
	if (ch->cur) {
		vchan_terminate_vdesc(&ch->cur->vd);
		ch->cur = NULL;
	}
	vchan_get_all_descriptors(&ch->vc, &head);
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	vchan_dma_desc_free_list(&ch->vc, &head);
	return 0;
}

static void cortina_adma_synchronize(struct dma_chan *c)
{
	vchan_synchronize(&to_adma_chan(c)->vc);
}

static void cortina_adma_free_chan_resources(struct dma_chan *c)
{
	vchan_free_chan_resources(to_virt_chan(c));
}

static void cortina_adma_desc_free(struct virt_dma_desc *vd)
{
	kfree(container_of(vd, struct cortina_adma_desc, vd));
}

static int cortina_adma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *chans, *glb, *route, *axi;
	struct cortina_adma *a;
	int i, ret;

	a = devm_kzalloc(dev, sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;
	chans = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(chans))
		return PTR_ERR(chans);
	glb = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(glb))
		return PTR_ERR(glb);
	route = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(route))
		return PTR_ERR(route);
	axi = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(axi))
		return PTR_ERR(axi);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return dev_err_probe(dev, ret, "40-bit DMA mask\n");

	writel(readl(axi) | ADMA_AXI_OUTSTANDING, axi);
	writel(readl(glb + ADMA_GLB_CTRL) | ADMA_GLB_40BIT, glb + ADMA_GLB_CTRL);
	writel(readl(glb + ADMA_GLB_RATE) & ~ADMA_GLB_RATE_LIMIT, glb + ADMA_GLB_RATE);

	a->dma.dev = dev;
	INIT_LIST_HEAD(&a->dma.channels);
	dma_cap_set(DMA_MEMCPY, a->dma.cap_mask);
	a->dma.device_prep_dma_memcpy = cortina_adma_prep_memcpy;
	a->dma.device_issue_pending = cortina_adma_issue_pending;
	a->dma.device_tx_status = dma_cookie_status;
	a->dma.device_terminate_all = cortina_adma_terminate_all;
	a->dma.device_synchronize = cortina_adma_synchronize;
	a->dma.device_free_chan_resources = cortina_adma_free_chan_resources;

	for (i = 0; i < ADMA_CHANNELS; i++) {
		struct cortina_adma_chan *ch = &a->chan[i];

		ch->base = chans + i * ADMA_CHAN_STRIDE;
		ch->irq = platform_get_irq(pdev, i);
		if (ch->irq < 0)
			return ch->irq;
		cortina_adma_reset(ch);
		writel(readl(ch->base + ADMA_INT), ch->base + ADMA_INT);
		writel(readl(route + 8 * i) | BIT(i), route + 8 * i);
		ret = devm_request_irq(dev, ch->irq, cortina_adma_irq, 0, dev_name(dev), ch);
		if (ret)
			return dev_err_probe(dev, ret, "irq %d\n", ch->irq);
		writel(ADMA_INT_DONE | ADMA_INT_OVERRUN, ch->base + ADMA_INT_EN);
		ch->vc.desc_free = cortina_adma_desc_free;
		vchan_init(&ch->vc, &a->dma);
	}

	ret = dmaenginem_async_device_register(&a->dma);
	if (ret)
		return dev_err_probe(dev, ret, "register\n");
	dev_info(dev, "%d memcpy channels\n", ADMA_CHANNELS);
	return 0;
}

static const struct of_device_id cortina_adma_of_match[] = {
	{ .compatible = "cortina,adma" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_adma_of_match);

static struct platform_driver cortina_adma_driver = {
	.probe = cortina_adma_probe,
	.driver = {
		.name = "cortina-adma",
		.of_match_table = cortina_adma_of_match,
	},
};
module_platform_driver(cortina_adma_driver);

MODULE_DESCRIPTION("RTL9607F (Cortina-Access ca77xx) memory-to-memory DMA engine");
MODULE_LICENSE("GPL");
