// SPDX-License-Identifier: GPL-2.0
/* TIER: FAMILY */
/*
 * Copyright (C) 2015 Broadcom Corporation
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>

#include "../pci.h"
#include "pcie-iproc.h"

/*
 * Hurricane2's PAXB decodes no APB error register (the mainline PAXB's 0xf40: an external abort on the V1600G) and
 * its boot loader leaves the outbound window unset: the BCMA register set, with OARR0/OMAP0 written here, as the
 * stock kernel writes them, from the one memory range of the node (a 128 MiB window).
 */
#define HR2_OARR0	0xd20
#define HR2_OMAP0	0xd40

static int iproc_hr2_ob_window(struct device *dev, void __iomem *base, struct pci_host_bridge *bridge)
{
	struct resource_entry *entry;

	resource_list_for_each_entry(entry, &bridge->windows) {
		struct resource *res = entry->res;

		if (resource_type(res) != IORESOURCE_MEM)
			continue;
		if (resource_size(res) != SZ_128M || !IS_ALIGNED(res->start, SZ_128M)) {
			dev_err(dev, "outbound window %pR: Hurricane2 takes one aligned 128 MiB window\n", res);
			return -EINVAL;
		}
		writel(lower_32_bits(res->start - entry->offset) | 1, base + HR2_OMAP0);
		writel(lower_32_bits(res->start) | 1, base + HR2_OARR0);
		return 0;
	}
	dev_err(dev, "no memory range for the outbound window\n");
	return -EINVAL;
}

/*
 * A memory BAR larger than the one window (the V1600G's ZX279200 decodes 256 MiB) can never be assigned whole: it
 * is clamped to the window, so its first 128 MiB stay reachable. Sized as the window, the kernel can only place
 * it at the window's start, which must then be aligned to the size the device decodes.
 */
static void iproc_hr2_clamp_bar(struct pci_dev *pdev)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);
	struct resource_entry *win = NULL, *entry;
	int i;

	if (!bridge->dev.parent || !of_device_is_compatible(bridge->dev.parent->of_node, "brcm,iproc-hr2-pcie"))
		return;
	resource_list_for_each_entry(entry, &bridge->windows)
		if (resource_type(entry->res) == IORESOURCE_MEM)
			win = entry;
	if (!win)
		return;
	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		struct resource *res = &pdev->resource[i];
		resource_size_t size = resource_size(res);

		if (resource_type(res) != IORESOURCE_MEM || size <= resource_size(win->res))
			continue;
		if (!IS_ALIGNED(win->res->start - win->offset, size)) {
			pci_err(pdev, "BAR %d (%pa bytes) is larger than the window %pR, which is not aligned to it\n",
				i, &size, win->res);
			continue;
		}
		res->end = res->start + resource_size(win->res) - 1;
		pci_info(pdev, "BAR %d decodes %pa bytes, clamped to the window's %pR\n", i, &size, res);
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_ANY_ID, PCI_ANY_ID, iproc_hr2_clamp_bar);

static const struct of_device_id iproc_pcie_of_match_table[] = {
	{
		.compatible = "brcm,iproc-hr2-pcie",
		.data = (int *)IPROC_PCIE_PAXB_BCMA,
	}, {
		.compatible = "brcm,iproc-pcie",
		.data = (int *)IPROC_PCIE_PAXB,
	}, {
		.compatible = "brcm,iproc-pcie-paxb-v2",
		.data = (int *)IPROC_PCIE_PAXB_V2,
	}, {
		.compatible = "brcm,iproc-pcie-paxc",
		.data = (int *)IPROC_PCIE_PAXC,
	}, {
		.compatible = "brcm,iproc-pcie-paxc-v2",
		.data = (int *)IPROC_PCIE_PAXC_V2,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, iproc_pcie_of_match_table);

static int iproc_pltfm_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iproc_pcie *pcie;
	struct device_node *np = dev->of_node;
	struct resource reg;
	struct pci_host_bridge *bridge;
	int ret;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);

	pcie->dev = dev;
	pcie->type = (uintptr_t)of_device_get_match_data(dev);

	ret = of_address_to_resource(np, 0, &reg);
	if (ret < 0) {
		dev_err(dev, "unable to obtain controller resources\n");
		return ret;
	}

	pcie->base = devm_pci_remap_cfgspace(dev, reg.start,
					     resource_size(&reg));
	if (!pcie->base) {
		dev_err(dev, "unable to map controller registers\n");
		return -ENOMEM;
	}
	pcie->base_addr = reg.start;

	if (of_device_is_compatible(np, "brcm,iproc-hr2-pcie")) {
		ret = iproc_hr2_ob_window(dev, pcie->base, bridge);
		if (ret)
			return ret;
	}

	if (of_property_read_bool(np, "brcm,pcie-ob")) {
		u32 val;

		ret = of_property_read_u32(np, "brcm,pcie-ob-axi-offset",
					   &val);
		if (ret) {
			dev_err(dev,
				"missing brcm,pcie-ob-axi-offset property\n");
			return ret;
		}
		pcie->ob.axi_offset = val;
		pcie->need_ob_cfg = true;
	}

	/*
	 * DT nodes are not used by all platforms that use the iProc PCIe
	 * core driver. For platforms that require explicit inbound mapping
	 * configuration, "dma-ranges" would have been present in DT
	 */
	pcie->need_ib_cfg = of_property_read_bool(np, "dma-ranges");

	/* PHY use is optional */
	pcie->phy = devm_phy_optional_get(dev, "pcie-phy");
	if (IS_ERR(pcie->phy))
		return PTR_ERR(pcie->phy);

	/* PAXC doesn't support legacy IRQs, skip mapping */
	switch (pcie->type) {
	case IPROC_PCIE_PAXC:
	case IPROC_PCIE_PAXC_V2:
		bridge->map_irq = NULL;
		break;
	default:
		break;
	}

	ret = iproc_pcie_setup(pcie, &bridge->windows);
	if (ret) {
		dev_err(dev, "PCIe controller setup failed\n");
		return ret;
	}

	platform_set_drvdata(pdev, pcie);
	return 0;
}

static void iproc_pltfm_pcie_remove(struct platform_device *pdev)
{
	struct iproc_pcie *pcie = platform_get_drvdata(pdev);

	iproc_pcie_remove(pcie);
}

static void iproc_pltfm_pcie_shutdown(struct platform_device *pdev)
{
	struct iproc_pcie *pcie = platform_get_drvdata(pdev);

	iproc_pcie_shutdown(pcie);
}

static struct platform_driver iproc_pltfm_pcie_driver = {
	.driver = {
		.name = "iproc-pcie",
		.of_match_table = of_match_ptr(iproc_pcie_of_match_table),
	},
	.probe = iproc_pltfm_pcie_probe,
	.remove = iproc_pltfm_pcie_remove,
	.shutdown = iproc_pltfm_pcie_shutdown,
};
module_platform_driver(iproc_pltfm_pcie_driver);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom iPROC PCIe platform driver");
MODULE_LICENSE("GPL v2");
