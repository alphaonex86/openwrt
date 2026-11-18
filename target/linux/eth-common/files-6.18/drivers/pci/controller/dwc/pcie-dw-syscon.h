/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* pcie-dw-syscon.h -- a DesignWare root complex its SoC brings up through syscon register steps, then the
 * application logic's LTSSM enable (and a retrain at a given generation, if the stock kernel does one); the link
 * is up when an application status field says L0. The chip gives only these tables.
 */
#ifndef _PCIE_DW_SYSCON_H
#define _PCIE_DW_SYSCON_H

#include <linux/platform_device.h>
#include <linux/types.h>

#define PCIE_DW_SYSCON_MAPS	4
#define PCIE_DW_SYSCON_STEPS	12

struct pcie_dw_syscon_step {
	u8 map;		/* index in maps[] */
	u16 reg;
	bool set;	/* set the port's mask, else clear it */
};

struct pcie_dw_syscon_port {
	phys_addr_t dbi;	/* the port is told by its "dbi" range */
	u32 mask[PCIE_DW_SYSCON_STEPS];
};

struct pcie_dw_syscon_soc {
	const char *maps[PCIE_DW_SYSCON_MAPS];	/* the syscon phandle properties, NULL after the last */
	const struct pcie_dw_syscon_step *steps;
	unsigned int n_steps, step_ms;		/* step_ms: the pause after each step */
	const struct pcie_dw_syscon_port *ports;
	unsigned int n_ports;
	u32 app_ctrl, app_ltssm_en;		/* offsets and masks in the "app" range */
	u32 app_status, app_ltssm, ltssm_l0;
	u8 retrain_gen;				/* 0: no retrain */
};

int pcie_dw_syscon_probe(struct platform_device *pdev);

#endif /* _PCIE_DW_SYSCON_H */
