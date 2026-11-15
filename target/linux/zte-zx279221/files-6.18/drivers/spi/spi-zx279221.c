// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/*
 * ZTE ZX279221 SSP, a polled SPI host (the board CPLD sits on SSP0). Facts from the unit's U-Boot and stock kernel:
 * 0x04 enable (bit 1; the chip select follows it), 0x08 frame format, 0x0c data FIFO (16 entries), 0x10 FIFO
 * thresholds, 0x14 status. The work clock is the one U-Boot set (it reads the CPLD over this bus before any kernel);
 * this driver does not change it. Only the first chip select is known.
 */
#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define SSP_ENABLE		0x04
#define SSP_ON			BIT(1)
#define SSP_FMT			0x08
#define SSP_FMT_CPOL		BIT(2)
#define SSP_FMT_CPHA		BIT(3)
#define SSP_FMT_BITS		GENMASK(7, 4)		/* bits per word - 1 */
#define SSP_DATA		0x0c
#define SSP_FIFO_CTRL		0x10
#define SSP_FIFO_HALF		0x770			/* both thresholds at half the FIFO, as stock */
#define SSP_STATUS		0x14
#define SSP_BUSY		BIT(4)
#define SSP_RX_COUNT		GENMASK(9, 5)
#define SSP_TX_ROOM		GENMASK(14, 10)
#define SSP_FIFO		16
#define SSP_TIMEOUT_US		20000

struct zx_ssp {
	void __iomem *base;
};

static void zx_ssp_set_cs(struct spi_device *spi, bool level)
{
	struct zx_ssp *ssp = spi_controller_get_devdata(spi->controller);
	u32 fmt = FIELD_PREP(SSP_FMT_BITS, spi->bits_per_word - 1);

	if (level) {		/* the line high: deselected */
		writel(0, ssp->base + SSP_ENABLE);
		return;
	}
	if (spi->mode & SPI_CPOL)
		fmt |= SSP_FMT_CPOL;
	if (spi->mode & SPI_CPHA)
		fmt |= SSP_FMT_CPHA;
	writel(fmt, ssp->base + SSP_FMT);
	writel(SSP_FIFO_HALF, ssp->base + SSP_FIFO_CTRL);
	writel(SSP_ON, ssp->base + SSP_ENABLE);
}

/* Full duplex, byte by byte: never more in flight than the receive FIFO holds. */
static int zx_ssp_transfer_one(struct spi_controller *ctlr, struct spi_device *spi, struct spi_transfer *t)
{
	struct zx_ssp *ssp = spi_controller_get_devdata(ctlr);
	const u8 *tx = t->tx_buf;
	u8 *rx = t->rx_buf;
	unsigned int sent = 0, got = 0, n;
	ktime_t end = ktime_add_us(ktime_get(), SSP_TIMEOUT_US);
	u32 st;

	while (got < t->len) {
		st = readl(ssp->base + SSP_STATUS);
		for (n = FIELD_GET(SSP_RX_COUNT, st); n && got < t->len; n--, got++) {
			u8 b = readl(ssp->base + SSP_DATA);

			if (rx)
				rx[got] = b;
		}
		for (n = FIELD_GET(SSP_TX_ROOM, st); n && sent < t->len && sent - got < SSP_FIFO; n--, sent++)
			writel(tx ? tx[sent] : 0, ssp->base + SSP_DATA);
		if (ktime_after(ktime_get(), end)) {
			dev_err(&ctlr->dev, "transfer timed out: %u of %u bytes sent, %u received, status 0x%x\n",
				sent, t->len, got, st);
			return -ETIMEDOUT;
		}
	}
	if (readl_poll_timeout_atomic(ssp->base + SSP_STATUS, st, !(st & SSP_BUSY), 1, SSP_TIMEOUT_US)) {
		dev_err(&ctlr->dev, "still busy after the transfer, status 0x%x\n", st);
		return -ETIMEDOUT;
	}
	return 0;
}

static int zx_ssp_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct zx_ssp *ssp;

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(*ssp));
	if (!ctlr)
		return -ENOMEM;
	ssp = spi_controller_get_devdata(ctlr);
	ssp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ssp->base))
		return PTR_ERR(ssp->base);
	writel(0, ssp->base + SSP_ENABLE);
	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->num_chipselect = 1;
	ctlr->set_cs = zx_ssp_set_cs;
	ctlr->transfer_one = zx_ssp_transfer_one;
	return devm_spi_register_controller(&pdev->dev, ctlr);
}

static const struct of_device_id zx_ssp_of_match[] = {
	{ .compatible = "zte,zx279221-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, zx_ssp_of_match);

static struct platform_driver zx_ssp_driver = {
	.probe = zx_ssp_probe,
	.driver = {
		.name = "zx279221-spi",
		.of_match_table = zx_ssp_of_match,
	},
};
module_platform_driver(zx_ssp_driver);

MODULE_DESCRIPTION("ZTE ZX279221 SSP (SPI host)");
MODULE_LICENSE("GPL");
