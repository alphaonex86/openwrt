// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CORE */
/* pcie-dw-syscon.c -- see pcie-dw-syscon.h. */
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/regmap.h>

#include "pcie-designware.h"
#include "pcie-dw-syscon.h"

struct pcie_dw_syscon {
	struct dw_pcie pci;
	void __iomem *app;
	struct regmap *map[PCIE_DW_SYSCON_MAPS];
	const struct pcie_dw_syscon_soc *soc;
	const struct pcie_dw_syscon_port *port;
};

static struct pcie_dw_syscon *to_pcie_dw_syscon(struct dw_pcie *pci)
{
	return container_of(pci, struct pcie_dw_syscon, pci);
}

static int pcie_dw_syscon_host_init(struct dw_pcie_rp *pp)
{
	struct pcie_dw_syscon *p = to_pcie_dw_syscon(to_dw_pcie_from_pp(pp));
	unsigned int i;
	int ret;

	for (i = 0; i < p->soc->n_steps; i++) {
		const struct pcie_dw_syscon_step *s = &p->soc->steps[i];
		u32 mask = p->port->mask[i];

		ret = regmap_update_bits(p->map[s->map], s->reg, mask, s->set ? mask : 0);
		if (ret) {
			dev_err(p->pci.dev, "bring-up step %u (%s %#x): %d\n", i, p->soc->maps[s->map], s->reg, ret);
			return ret;
		}
		msleep(p->soc->step_ms);
	}
	return 0;
}

static int pcie_dw_syscon_start_link(struct dw_pcie *pci)
{
	struct pcie_dw_syscon *p = to_pcie_dw_syscon(pci);
	u8 cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u16 ctl2;

	writel(readl(p->app + p->soc->app_ctrl) | p->soc->app_ltssm_en, p->app + p->soc->app_ctrl);
	if (!p->soc->retrain_gen)
		return 0;
	if (!cap) {
		dev_err(pci->dev, "no PCI Express capability in the root port: link not retrained\n");
		return -ENODEV;
	}
	ctl2 = dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKCTL2) & ~PCI_EXP_LNKCTL2_TLS;
	dw_pcie_writew_dbi(pci, cap + PCI_EXP_LNKCTL2, ctl2 | p->soc->retrain_gen);
	dw_pcie_writew_dbi(pci, cap + PCI_EXP_LNKCTL, dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKCTL) | PCI_EXP_LNKCTL_RL);
	return 0;
}

static bool pcie_dw_syscon_link_up(struct dw_pcie *pci)
{
	struct pcie_dw_syscon *p = to_pcie_dw_syscon(pci);
	u32 v = readl(p->app + p->soc->app_status) & p->soc->app_ltssm;

	return v >> __ffs(p->soc->app_ltssm) == p->soc->ltssm_l0;
}

static const struct dw_pcie_ops pcie_dw_syscon_ops = {
	.link_up = pcie_dw_syscon_link_up,
	.start_link = pcie_dw_syscon_start_link,
};

static const struct dw_pcie_host_ops pcie_dw_syscon_host_ops = {
	.init = pcie_dw_syscon_host_init,
};

int pcie_dw_syscon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct pcie_dw_syscon_soc *soc = of_device_get_match_data(dev);
	struct pcie_dw_syscon *p;
	struct resource *dbi;
	unsigned int i;

	if (!soc || soc->n_steps > PCIE_DW_SYSCON_STEPS || !soc->app_ltssm)
		return dev_err_probe(dev, -EINVAL, "no usable SoC table\n");
	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->soc = soc;
	dbi = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!dbi)
		return dev_err_probe(dev, -EINVAL, "no \"dbi\" range\n");
	for (i = 0; i < soc->n_ports; i++)
		if (soc->ports[i].dbi == dbi->start)
			p->port = &soc->ports[i];
	if (!p->port)
		return dev_err_probe(dev, -EINVAL, "%pR is no known port's DBI\n", dbi);
	p->app = devm_platform_ioremap_resource_byname(pdev, "app");
	if (IS_ERR(p->app))
		return dev_err_probe(dev, PTR_ERR(p->app), "\"app\" range\n");
	for (i = 0; i < PCIE_DW_SYSCON_MAPS && soc->maps[i]; i++) {
		p->map[i] = syscon_regmap_lookup_by_phandle(dev->of_node, soc->maps[i]);
		if (IS_ERR(p->map[i]))
			return dev_err_probe(dev, PTR_ERR(p->map[i]), "%s\n", soc->maps[i]);
	}
	for (i = 0; i < soc->n_steps; i++)
		if (soc->steps[i].map >= PCIE_DW_SYSCON_MAPS || !p->map[soc->steps[i].map])
			return dev_err_probe(dev, -EINVAL, "step %u names no syscon\n", i);
	p->pci.dev = dev;
	p->pci.ops = &pcie_dw_syscon_ops;
	p->pci.pp.ops = &pcie_dw_syscon_host_ops;
	platform_set_drvdata(pdev, p);
	return dw_pcie_host_init(&p->pci.pp);
}
