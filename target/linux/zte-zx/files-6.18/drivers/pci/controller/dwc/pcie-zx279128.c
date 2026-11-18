// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/*
 * ZTE ZX279128 PCIe: two DesignWare root complexes, one lane each at gen1 (the F680's MT7603E on port 0, its MT7615
 * on port 1). The bring-up is the stock kernel's, read from its binary (ONU-F680.md): per port, mode bits and two
 * clock enables, a reset pulse with the port's pin function taken and the shared PHY control cleared, then the
 * application logic's LTSSM enable and a retrain at gen1; the link is up when the application status says L0.
 * Legacy interrupts only, on the port's own line, as stock. The logic is pcie-dw-syscon's.
 */
#include <linux/bits.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include "pcie-dw-syscon.h"

#define TOP_PCIE_CLK		0x08
#define TOP_PCIE_CTRL		0x24
#define SUB_PCIE_MODE		0x2c
#define SUB_PCIE_RESET		0x54
#define PINMUX_PCIE		0x0c

enum { TOP, SUB, PINMUX };

/* stock's order; each port's mask per step is in zx279128_pcie_ports[] */
static const struct pcie_dw_syscon_step zx279128_pcie_steps[] = {
	{ SUB, SUB_PCIE_MODE, true },
	{ TOP, TOP_PCIE_CLK, true },
	{ TOP, TOP_PCIE_CLK, true },
	{ SUB, SUB_PCIE_RESET, false },
	{ PINMUX, PINMUX_PCIE, false },
	{ TOP, TOP_PCIE_CTRL, false },
	{ SUB, SUB_PCIE_RESET, true },
	{ SUB, SUB_PCIE_MODE, false },
};

static const struct pcie_dw_syscon_port zx279128_pcie_ports[] = {
	{ 0x0f000000, { 0xb5, BIT(13), GENMASK(15, 14), BIT(0), BIT(21), BIT(26), BIT(0), BIT(8) } },
	{ 0x0f100000, { 0x16a00, BIT(25), GENMASK(27, 26), BIT(3), BIT(24), BIT(26), BIT(3), BIT(17) } },
};

static const struct pcie_dw_syscon_soc zx279128_pcie = {
	.maps = { [TOP] = "zte,topcrm", [SUB] = "zte,subcrm", [PINMUX] = "zte,pinmux" },
	.steps = zx279128_pcie_steps,
	.n_steps = ARRAY_SIZE(zx279128_pcie_steps),
	.step_ms = 25,
	.ports = zx279128_pcie_ports,
	.n_ports = ARRAY_SIZE(zx279128_pcie_ports),
	.app_ctrl = 0x00,
	.app_ltssm_en = BIT(5),
	.app_status = 0xdc,
	.app_ltssm = GENMASK(22, 17),
	.ltssm_l0 = 0x11,
	.retrain_gen = 1,
};

static const struct of_device_id zx279128_pcie_of_match[] = {
	{ .compatible = "zte,zx279128-pcie", .data = &zx279128_pcie },
	{ }
};

static struct platform_driver zx279128_pcie_driver = {
	.probe = pcie_dw_syscon_probe,
	.driver = {
		.name = "zx279128-pcie",
		.of_match_table = zx279128_pcie_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(zx279128_pcie_driver);
