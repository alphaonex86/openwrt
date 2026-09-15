// SPDX-License-Identifier: GPL-2.0
/* PCIe root-complex host driver for Cortina-Access "Venus" ...
 * dev/MEASURED-pcie-cortina.c.md sec 1. */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/build_bug.h>	/* static_assert() for the lane composition */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/reset.h>
#include <linux/uaccess.h>

#include "pcie-designware.h"

#define CORTINA_PCIE_MAX_LANES		2
#define CORTINA_PCIE_SERDES_LANE_STRIDE	0x1000

/* DT property carrying the SerDes PHY init table as <offset value> u32 pairs */
#define CORTINA_PCIE_SERDES_CFG_PROP	"serdes-cfg-dataB"

/* SerDes lock status: one register per {lane,path}, polled ...
 * dev/MEASURED-pcie-cortina.c.md sec 2. */
#define CORTINA_PCIE_SERDES_SUB_STRIDE	0x100	/* two sub-lanes per lane */
#define CORTINA_SERDES_BER_STAT		0x7c	/* within one sub-lane */
#define CORTINA_SERDES_SUB(lane, sub) \
	((lane) * CORTINA_PCIE_SERDES_LANE_STRIDE + \
	 (sub) * CORTINA_PCIE_SERDES_SUB_STRIDE)

/* the four spellings this replaced, pinned so the composition cannot drift */
static_assert(CORTINA_SERDES_SUB(0, 0) + CORTINA_SERDES_BER_STAT == 0x007c);
static_assert(CORTINA_SERDES_SUB(0, 1) + CORTINA_SERDES_BER_STAT == 0x017c);
static_assert(CORTINA_SERDES_SUB(1, 0) + CORTINA_SERDES_BER_STAT == 0x107c);
static_assert(CORTINA_SERDES_SUB(1, 1) + CORTINA_SERDES_BER_STAT == 0x117c);
#define CORTINA_SERDES_LOCK_MASK	0x7800	/* CMU/PLL lock (0x7c bits 11-14) */
#define CORTINA_SERDES_BER_POLL_MAX	500

/*
 * Glue ("glbl_regs") register block.
 */
#define CA_PCIE_GLBL_INT0		0x00
#define CA_PCIE_GLBL_INT_EN0		0x04
/* INT0 / INT_EN0 bit definitions */
#define CA_INT_RADM_INTA_ASSERTED	BIT(0)
#define CA_INT_RADM_INTB_ASSERTED	BIT(2)
#define CA_INT_RADM_INTC_ASSERTED	BIT(4)
#define CA_INT_RADM_INTD_ASSERTED	BIT(6)
#define CA_INT_MSI_CTRL			BIT(8)
#define CA_INT_SMLH_LINK_UP		BIT(9)
#define CA_INT_HP			BIT(10)
#define CA_INT_RADM_CORRECTABLE_ERR	BIT(11)
#define CA_INT_RADM_NONFATAL_ERR	BIT(12)
#define CA_INT_RADM_FATAL_ERR		BIT(13)
#define CA_INT_RADM_PM_TO_ACK		BIT(14)
#define CA_INT_RADM_PM_PME		BIT(15)
#define CA_INT_RADM_QOVERFLOW		BIT(16)
#define CA_INT_LINK_DOWN		BIT(22)
#define CA_INT_CFG_AER_RC_ERR_MSI	BIT(23)
#define CA_INT_CFG_PME_MSI		BIT(24)
#define CA_INT_HP_PME			BIT(25)
#define CA_INT_HP_MSI			BIT(26)
#define CA_INT_CFG_UR_RESP		BIT(27)

#define CA_INT_INTX_ALL			(CA_INT_RADM_INTA_ASSERTED | \
					 CA_INT_RADM_INTB_ASSERTED | \
					 CA_INT_RADM_INTC_ASSERTED | \
					 CA_INT_RADM_INTD_ASSERTED)

#define CA_INT_MISC_ALL			(CA_INT_HP | \
					 CA_INT_RADM_CORRECTABLE_ERR | \
					 CA_INT_RADM_NONFATAL_ERR | \
					 CA_INT_RADM_FATAL_ERR | \
					 CA_INT_RADM_PM_TO_ACK | \
					 CA_INT_RADM_PM_PME | \
					 CA_INT_RADM_QOVERFLOW | \
					 CA_INT_LINK_DOWN | \
					 CA_INT_CFG_AER_RC_ERR_MSI | \
					 CA_INT_CFG_PME_MSI | \
					 CA_INT_HP_PME | \
					 CA_INT_HP_MSI | \
					 CA_INT_CFG_UR_RESP)

#define CA_PCIE_CORE_CONFIG		0x18
#define CA_CORE_LTSSM_ENABLE		BIT(0)
#define CA_CORE_LINK_DOWN_RST		BIT(6)

#define CA_PCIE_PM_STATUS		0x1c
#define CA_PM_LTSSM_STATE_MASK		GENMASK(9, 4)
#define CA_PM_LTSSM_STATE_SHIFT		4
#define CA_PM_RDLH_LINK_UP		BIT(18)

/* Glue address-decoder windows: where DBI/config and iATU live in phys space */
#define CA_PCIE_CFG_START_ADDR		0x40
#define CA_PCIE_CFG_END_ADDR		0x44
#define CA_PCIE_IATU_BASE_ADDR		0x48
#define CA_PCIE_IATU_BASE_MASK		0xfff80000

struct cortina_serdes_cfg {
	u32 off;
	u32 val;
};

struct cortina_pcie {
	struct dw_pcie pci;

	void __iomem *glbl;		/* glbl_regs */
	void __iomem *serdes;		/* serdes_phy */
	resource_size_t serdes_size;
	void __iomem *rstmgr;		/* GLB reset/phy/mux window 0xf4320000 */
	void __iomem *gpio;		/* PER_GPIO window 0xf4329000 (PERST#) */

	phys_addr_t dbi_start;		/* rc_dbi resource bounds (for glue decoder) */
	phys_addr_t dbi_end;
	phys_addr_t iatu_start;		/* iatu resource base (for glue decoder) */

	struct clk *bus_clk;
	struct reset_control *core_reset;
	struct reset_control *phy_reset;
	struct reset_control *device_reset;
	struct reset_control *device_power;
	struct phy *phy[CORTINA_PCIE_MAX_LANES];

	struct irq_domain *intx_domain;

	/* SerDes PHY init table, applied to every lane at LANE_STRIDE offsets */
	struct cortina_serdes_cfg *cfg;
	int cfg_cnt;

	struct proc_dir_entry *dbg_pde;	/* per-controller /proc spy entry */

	u32 idx;
	u8 lanes;
};

#define to_cortina_pcie(x)	container_of((x), struct cortina_pcie, pci)

static inline u32 cortina_glbl_readl(struct cortina_pcie *cp, u32 reg)
{
	return readl(cp->glbl + reg);
}

static inline void cortina_glbl_writel(struct cortina_pcie *cp, u32 val, u32 reg)
{
	writel(val, cp->glbl + reg);
}

/* SerDes PHY Apply the DT SerDes register table to each lane. ...
 * dev/MEASURED-pcie-cortina.c.md sec 3. */
static void cortina_pcie_serdes_phy_init(struct cortina_pcie *cp)
{
	int lane, i;

	if (cp->cfg_cnt <= 0 || !cp->cfg) {
		dev_warn(cp->pci.dev, "no SerDes cfg table (%s missing)\n",
			 CORTINA_PCIE_SERDES_CFG_PROP);
		return;
	}

	for (lane = 0; lane < cp->lanes; lane++) {
		u32 lane_off = lane * CORTINA_PCIE_SERDES_LANE_STRIDE;

		for (i = 0; i < cp->cfg_cnt; i++) {
			u32 off = cp->cfg[i].off + lane_off;

			if (off < cp->serdes_size)
				writel(cp->cfg[i].val, cp->serdes + off);
		}
	}
}

/* pcie2 SHORT RX calibration (stock k0 @0x913034, page0 only, ...
 * dev/MEASURED-pcie-cortina.c.md sec 4. */
static void cortina_pcie_serdes_hard_relock(struct cortina_pcie *cp);
static void cortina_pcie_long_cal(struct cortina_pcie *cp, void __iomem *s,
				  int lanes, void (*relock)(struct cortina_pcie *));

/* Stock's FULL RX-cal convergence FSM (k0 LONG path ... -- dev/MEASURED-pcie-cortina.c.md sec 5. */
#define CORTINA_SERDES_CALDONE	0x10	/* 0x7c bit4 = cal-done */
/* Run stock's RX-EQ adaptation ramp for ONE sub-lane ...
 * dev/MEASURED-pcie-cortina.c.md sec 6. */
static __maybe_unused void cortina_pcie_rx_eq_ramp(void __iomem *s, u32 b, bool cmu)
{
	u32 l1_bit4 = 0;
	u32 c;
	int i;

	if (cmu) {						/* L1 shared-CMU */
		l1_bit4 = readl(s + b + 0x24) & 0x10;	/* saved so L15 can truly undo */
		writel(readl(s + b + 0x24) & ~0x10, s + b + 0x24);
		usleep_range(10, 20);
	}
	writel(readl(s + b + 0x34) | 0x2000, s + b + 0x34);	usleep_range(10, 20);	/* L2 held-A */
	if (cmu) {						/* L3/L4 shared-CMU strobe 1->0->1 */
		writel(readl(s + b + 0x24) & ~0x200, s + b + 0x24);	usleep_range(10, 20);
		writel(readl(s + b + 0x24) | 0x200, s + b + 0x24);	usleep_range(10, 20);
	}
	writel(readl(s + b + 0x34) | 0x40, s + b + 0x34);	usleep_range(10, 20);	/* L5 held-B */
	writel(readl(s + b + 0x08) | 0x200, s + b + 0x08);	usleep_range(10, 20);	/* L6 held-C */
	writel(0x194, s + b + 0x40);				usleep_range(10, 20);	/* L7 */
	writel(readl(s + b + 0xbc) | 0x400, s + b + 0xbc);	usleep_range(10, 20);	/* L8 held-D */
	writel(readl(s + b + 0x6c) & ~0x60, s + b + 0x6c);	usleep_range(10, 20);	/* L9 */
	for (i = 0; i < 50000; i++) {		/* poll cal-done bit4 (capped ~0.75s) */
		if (readl(s + b + CORTINA_SERDES_BER_STAT) & CORTINA_SERDES_CALDONE)
			break;
		usleep_range(10, 20);
	}
	writel(readl(s + b + 0x08) & ~0x200, s + b + 0x08);	usleep_range(10, 20);	/* L11 rel-C */
	writel(readl(s + b + 0x6c) & ~0x60, s + b + 0x6c);	usleep_range(10, 20);	/* L12 */
	writel(0x1a4, s + b + 0x40);				usleep_range(10, 20);	/* L13 */
	c = readl(s + b + CORTINA_SERDES_BER_STAT);						/* R1 (load-bearing) */
	writel((readl(s + b + 0x0c) & ~0x3e) | ((c & 0x1f) << 1), s + b + 0x0c);
	usleep_range(10, 20);
	if (cmu) {
		/* ★ L15 UNDO L1 -- AND IT MUST ACTUALLY UNDO IT (fixed ...
		 * dev/MEASURED-pcie-cortina.c.md sec 7. */
		writel((readl(s + b + 0x24) & ~0x10) | l1_bit4, s + b + 0x24);
		usleep_range(10, 20);
	}
	writel(readl(s + b + 0x34) & ~0x40, s + b + 0x34);	usleep_range(10, 20);	/* L16 rel-B */
	writel(readl(s + b + 0xbc) & ~0x400, s + b + 0xbc);	usleep_range(10, 20);	/* L17 rel-D */
	writel(readl(s + b + 0x6c) & ~0x60, s + b + 0x6c);	usleep_range(10, 20);	/* L18 */
	writel(0x0c, s + b + 0x40);				usleep_range(10, 20);	/* L19 final */
	c = readl(s + b + CORTINA_SERDES_BER_STAT);						/* R4 (load-bearing) */
	writel((readl(s + b + 0x2c) & ~0x1e0) | (((c >> 1) & 0xf) << 5), s + b + 0x2c);
	usleep_range(10, 20);
	writel(readl(s + b + 0x34) & ~0x2000, s + b + 0x34);	usleep_range(10, 20);	/* L20 rel-A (our silicon needs it) */

	/* Finish ASYMMETRICALLY, matching stock's live-locked steady ...
	 * dev/MEASURED-pcie-cortina.c.md sec 25. */
	writel(readl(s + b + 0x28) & ~0x60, s + b + 0x28);	usleep_range(10, 20);	/* open latch */
	if (cmu) {
		writel(readl(s + b + 0x2c) | 0x10, s + b + 0x2c);	usleep_range(10, 20);	/* manual */
		c = (readl(s + b + CORTINA_SERDES_BER_STAT) >> 9) & 0x7f;
		writel((readl(s + b + 0x90) & ~0x7f) | c, s + b + 0x90);	usleep_range(10, 20);	/* freeze */
	} else {
		writel(readl(s + b + 0x2c) & ~0x10, s + b + 0x2c);	usleep_range(10, 20);	/* auto/adaptive */
	}
	writel(readl(s + b + 0x28) | 0x60, s + b + 0x28);	usleep_range(10, 20);	/* close latch */
}
/* Stock's pcie2 SHORT RX cal (k0 @0x913034), verified ...
 * dev/MEASURED-pcie-cortina.c.md sec 8. */
static void cortina_pcie_serdes_rx_cal(struct cortina_pcie *cp)
{
	void __iomem *s = cp->serdes;

	/* pcie2 = 1 lane. The stock SHORT cal doesn't converge on our ...
	 * dev/MEASURED-pcie-cortina.c.md sec 26. */
	cortina_pcie_rx_eq_ramp(s, 0x000, true);	/* sub-lane 0 (with shared CMU strobe) */
	cortina_pcie_rx_eq_ramp(s, 0x100, false);	/* sub-lane 1 (RX-EQ only) */
	dev_info(cp->pci.dev, "rxcal(seq): 7c=%08x 17c=%08x\n",
		 readl(s + CORTINA_SERDES_SUB(0, 0) + CORTINA_SERDES_BER_STAT),
		 readl(s + CORTINA_SERDES_SUB(0, 1) + CORTINA_SERDES_BER_STAT)); /* DIAG revert */
}

/* Wait for the SerDes to report a good bit-error-rate lock on all paths. */
static bool cortina_pcie_serdes_ber_notify(struct cortina_pcie *cp)
{
	int cnt = 0;
	u32 ready;

	do {
		ready = readl(cp->serdes + CORTINA_SERDES_SUB(0, 0) + CORTINA_SERDES_BER_STAT);
		ready &= readl(cp->serdes + CORTINA_SERDES_SUB(0, 1) + CORTINA_SERDES_BER_STAT);
		if (cp->lanes == 2) {
			ready &= readl(cp->serdes + CORTINA_SERDES_SUB(1, 0) + CORTINA_SERDES_BER_STAT);
			ready &= readl(cp->serdes + CORTINA_SERDES_SUB(1, 1) + CORTINA_SERDES_BER_STAT);
		}
		if ((ready & CORTINA_SERDES_LOCK_MASK) == CORTINA_SERDES_LOCK_MASK)
			return true;

		usleep_range(1000, 2000);
	} while (cnt++ < CORTINA_SERDES_BER_POLL_MAX);

	return false;
}

static int cortina_pcie_parse_serdes_cfg(struct cortina_pcie *cp)
{
	struct device *dev = cp->pci.dev;
	struct device_node *np = dev->of_node;
	int n;

	n = of_property_count_elems_of_size(np, CORTINA_PCIE_SERDES_CFG_PROP,
					    sizeof(struct cortina_serdes_cfg));
	if (n < 1) {
		cp->cfg_cnt = 0;
		return 0;
	}

	cp->cfg = devm_kmalloc_array(dev, n, sizeof(*cp->cfg), GFP_KERNEL);
	if (!cp->cfg)
		return -ENOMEM;

	/* Each element is a <offset value> pair, i.e. two u32 cells. */
	of_property_read_u32_array(np, CORTINA_PCIE_SERDES_CFG_PROP,
				   (u32 *)cp->cfg, n * 2);
	cp->cfg_cnt = n;

	return 0;
}

/* Reset / power bring-up RTL9607F reset-mgr / PHY-mux / GPIO ...
 * dev/MEASURED-pcie-cortina.c.md sec 9. */
#define CA_RSTMGR_PHYS		0x4f4320000ULL
#define CA_RSTMGR_SIZE		0x400
#define CA_GPIO_PHYS		0x4f4329000ULL
#define CA_GPIO_SIZE		0x1000
#define CA_RST_BLOCK		0x0a0	/* core_reset (assert = SET bit) */
#define CA_RST_DPHY		0x0a8	/* phy_reset */
#define CA_GLOBAL_CONFIG	0x0c8	/* clock gates (per-block ungate bits) */
#define CA_PHY_CONTROL		0x0cc	/* SerDes->PCIe mux */
#define CA_PHY_ISO_POWER	0x39c	/* SerDes lane power/isolation */
/* pcie2 has TWO clocks: the controller clock (gate_pcie2, ...
 * dev/MEASURED-pcie-cortina.c.md sec 10. */
#define CA_CLKEN_PCIE2		BIT(14)	/* gate_pcie2   : controller clock */
#define CA_CLKEN_PCIE2_PS	BIT(24)	/* gate_pcie2_ps: SerDes PHY / refclk */
/* PERST# pad-function mux. This is a GLOBAL pinctrl register ...
 * dev/MEASURED-pcie-cortina.c.md sec 11. */
#define CA_GLB_GPIO_MUX4	0x140	/* rstmgr/GLB window: bank4 pad-function mux */
#define CA_GPIO_B4_CFG		0x390	/* PER_GPIO window: bank4 pin dir (clear = output) */
#define CA_GPIO_B4_OUT		0x394	/* PER_GPIO window: bank4 pin output level */
/* pcie2 = SerDes S2 (2.4 GHz RTL8192F) */
#define S2_CORE_RST		BIT(8)
#define S2_PHY_RST		BIT(2)
#define S2_PHY_CTRL_SET		BIT(18)
#define S2_PHY_CTRL_CLR		(BIT(20) | BIT(21) | BIT(22))
#define S2_PHY_PWR_SET		(BIT(12) | BIT(18) | BIT(28) | BIT(29))
#define S2_PHY_PWR_CLR		(BIT(4) | BIT(11))
#define S2_PERST		BIT(12)
/* Golden post-init register values, read live from stock ...
 * dev/MEASURED-pcie-cortina.c.md sec 12. */
#define CA_DPHY_RESET_GOLD	0x000007AAu	/* 0x0a8: phy(bit2) released, rest = stock */
#define CA_PHY_ISO_POWER_GOLD	0xF0FD318Cu	/* 0x39c: lanes powered / de-isolated */

/* pcie0 / SerDes S0 (5 GHz) bring-up. Stock brings pcie0/S0 ...
 * dev/MEASURED-pcie-cortina.c.md sec 13. */
#define CA_S0_SERDES_PHYS	0x4f4333000ULL	/* pcie0 serdes_phy window */
#define CA_S0_SERDES_SIZE	0x2000
#define S0_LANES		2
#define S0_CORE_RST		BIT(6)		/* rstmgr 0x0a0 core_reset (S2=bit8) */
#define S0_PHY_RST		BIT(0)		/* rstmgr 0x0a8 phy_reset  (S2=bit2) */
#define S0_PHY_CTRL_SET		0x00000000u	/* 0x0cc SET (phy0|phy1) */
#define S0_PHY_CTRL_CLR		BIT(27)		/* 0x0cc CLR */
#define S0_PHY_PWR_SET		0x00f10180u	/* 0x39c SET (lanes powered) */
#define S0_PHY_PWR_CLR		0x00000003u	/* 0x39c CLR */
#define S0_PHY_PWR_OFF		0x00000003u	/* 0x39c isolate (power_off) */
#define S0_PERST		BIT(9)		/* gpio bank4 pin9 (S2=pin12) */
/* pcie0's controller + SerDes-refclk gates in 0x0c8, from the ...
 * dev/MEASURED-pcie-cortina.c.md sec 14. */
#define CA_CLKEN_PCIE0		BIT(10)	/* gate_pcie0   : controller clock */
#define CA_CLKEN_PCIE0_PS	BIT(22)	/* gate_pcie0_ps: SerDes PHY / refclk */

static inline void ca_rmw(void __iomem *base, u32 off, u32 clr, u32 set)
{
	writel((readl(base + off) & ~clr) | set, base + off);
}

/* Hard CMU relock (stock LONG-cal @0x914108): re-assert ...
 * dev/MEASURED-pcie-cortina.c.md sec 27. */
static __maybe_unused void cortina_pcie_serdes_hard_relock(struct cortina_pcie *cp)
{
	/* Match pcie0's proven relock structure (which locks its CMU to 0x7e76):
	 * ISOLATE the lane -> assert phy+core -> re-power (RMW, not wholesale) ->
	 * deassert.  0x39c off-bit for S2/phy-idx4 = bit4 (from the phy table). */
	ca_rmw(cp->rstmgr, CA_PHY_ISO_POWER, 0, BIT(4));	usleep_range(1000, 2000);  /* isolate */
	ca_rmw(cp->rstmgr, CA_RST_DPHY, 0, S2_PHY_RST);		usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_BLOCK, 0, S2_CORE_RST);	usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_PHY_CONTROL, S2_PHY_CTRL_CLR, S2_PHY_CTRL_SET);
	ca_rmw(cp->rstmgr, CA_PHY_ISO_POWER, S2_PHY_PWR_CLR, S2_PHY_PWR_SET); usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_DPHY, S2_PHY_RST, 0);		usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_BLOCK, S2_CORE_RST, 0);	usleep_range(1000, 2000);
}

/* pcie0 / S0 precondition (replays stock's pcie0-first LONG ...
 * dev/MEASURED-pcie-cortina.c.md sec 28. */
static void s0_rmw_all(void __iomem *s, int lanes, u32 o, u32 clr, u32 set)
{
	int lane, sub;

	for (lane = 0; lane < lanes; lane++)
		for (sub = 0; sub <= 0x100; sub += CORTINA_PCIE_SERDES_SUB_STRIDE) {
			u32 a = lane * CORTINA_PCIE_SERDES_LANE_STRIDE + sub + o;

			writel((readl(s + a) & ~clr) | set, s + a);
			usleep_range(10, 20);
		}
}

/* Const-write serdes reg `o` across `lanes` lanes/both sub-lanes. */
static void s0_wr_all(void __iomem *s, int lanes, u32 o, u32 val)
{
	int lane, sub;

	for (lane = 0; lane < lanes; lane++)
		for (sub = 0; sub <= 0x100; sub += CORTINA_PCIE_SERDES_SUB_STRIDE) {
			writel(val, s + lane * CORTINA_PCIE_SERDES_LANE_STRIDE + sub + o);
			usleep_range(10, 20);
		}
}

/* Bounded poll of 0x7c bit4 (cal-done) on sub-lane 0 of `base`. */
static void s0_poll_caldone(void __iomem *s, u32 base)
{
	int i;

	for (i = 0; i < 50000; i++) {
		if (readl(s + base + CORTINA_SERDES_BER_STAT) & CORTINA_SERDES_CALDONE)
			return;
		usleep_range(10, 20);
	}
}

/* 0x0c[5:1] = 0x7c[4:0], both sub-lanes of one lane. */
static void s0_cap_0c(void __iomem *s, int lane)
{
	int sub;

	for (sub = 0; sub <= 0x100; sub += CORTINA_PCIE_SERDES_SUB_STRIDE) {
		u32 b = lane * CORTINA_PCIE_SERDES_LANE_STRIDE + sub;
		u32 c = readl(s + b + CORTINA_SERDES_BER_STAT);

		writel((readl(s + b + 0x0c) & ~0x3e) | ((c & 0x1f) << 1), s + b + 0x0c);
		usleep_range(10, 20);
	}
}

/* 0x2c[8:5] = 0x7c[4:1] (RX-EQ rate), `lanes` lanes/sub-lanes. */
static void s0_cap_2c(void __iomem *s, int lanes)
{
	int lane, sub;

	for (lane = 0; lane < lanes; lane++)
		for (sub = 0; sub <= 0x100; sub += CORTINA_PCIE_SERDES_SUB_STRIDE) {
			u32 b = lane * CORTINA_PCIE_SERDES_LANE_STRIDE + sub;
			u32 c = readl(s + b + CORTINA_SERDES_BER_STAT);

			writel((readl(s + b + 0x2c) & ~0x1e0) | (((c >> 1) & 0xf) << 5),
			       s + b + 0x2c);
			usleep_range(10, 20);
		}
}

/* S0 phy_power_on: mux SerDes S0 -> PCIe + power/de-isolate the lanes. */
static void s0_phy_power_on(struct cortina_pcie *cp)
{
	ca_rmw(cp->rstmgr, CA_PHY_CONTROL, S0_PHY_CTRL_CLR, S0_PHY_CTRL_SET);
	ca_rmw(cp->rstmgr, CA_PHY_ISO_POWER, S0_PHY_PWR_CLR, S0_PHY_PWR_SET);
}

/* Mid-cal hard CMU relock (k0 @0x914108): isolate -> assert phy+core -> re-power
 * -> deassert, re-acquiring the CMU lock the RX-cal strobe perturbed. */
static void cortina_pcie0_hard_relock(struct cortina_pcie *cp)
{
	ca_rmw(cp->rstmgr, CA_PHY_ISO_POWER, 0, S0_PHY_PWR_OFF);	usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_DPHY, 0, S0_PHY_RST);		usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_BLOCK, 0, S0_CORE_RST);	usleep_range(1000, 2000);
	s0_phy_power_on(cp);					usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_DPHY, S0_PHY_RST, 0);		usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_BLOCK, S0_CORE_RST, 0);	usleep_range(1000, 2000);
}

/* pcie0 serdes-cfg-dataB (stock DTB rtl9607f.dts:1311): ...
 * dev/MEASURED-pcie-cortina.c.md sec 15. */
#define CA_SDS_SUBLANE_STRIDE	0x100

static void cortina_pcie0_serdes_table(void __iomem *s)
{
	static const struct cortina_serdes_cfg sublane_cfg[] = {
		/* sub-lane 0 */
		{ 0x004, 0xa855 },	/* differs: sub-lane 1 writes 0xa84a */
		{ 0x008, 0x60c6 },
		{ 0x010, 0x4000 },
		{ 0x018, 0x001f },
		{ 0x020, 0x3591 },
		{ 0x028, 0xf610 },
		{ 0x02c, 0xa91d },
		{ 0x030, 0xc008 },
		{ 0x034, 0xf732 },
		{ 0x038, 0x1000 },
		{ 0x074, 0xca1f },	/* differs: sub-lane 1 writes 0xda1f */
		{ 0x078, 0xe0e2 },
		{ 0x080, 0xd488 },	/* only sub-lane 0 writes this */
		{ 0x084, 0x77dd },	/* only sub-lane 0 writes this */
		{ 0x088, 0x0023 },
		{ 0x08c, 0x1b63 },
		{ 0x090, 0x4f30 },	/* differs: sub-lane 1 writes 0x4f0c */
		{ 0x09c, 0x61d6 },
		{ 0x0a0, 0xf802 },
		{ 0x0ac, 0xb813 },
		{ 0x0c0, 0x0055 },	/* differs: sub-lane 1 writes 0x004a */
		{ 0x0c8, 0xf0f3 },
		{ 0x024, 0x520c },	/* LAST, and deliberately out of address order */

		/* sub-lane 1, at +CA_SDS_SUBLANE_STRIDE */
		{ 0x104, 0xa84a },	/* differs: sub-lane 0 writes 0xa855 */
		{ 0x108, 0x60c6 },
		{ 0x110, 0x4000 },
		{ 0x118, 0x001f },
		{ 0x120, 0x3591 },
		{ 0x128, 0xf610 },
		{ 0x12c, 0xa91d },
		{ 0x130, 0xc008 },
		{ 0x134, 0xf732 },
		{ 0x138, 0x1000 },
		{ 0x174, 0xda1f },	/* differs: sub-lane 0 writes 0xca1f */
		{ 0x178, 0xe0e2 },
		{ 0x188, 0x0023 },
		{ 0x18c, 0x1b63 },
		{ 0x190, 0x4f0c },	/* differs: sub-lane 0 writes 0x4f30 */
		{ 0x19c, 0x61d6 },
		{ 0x1a0, 0xf802 },
		{ 0x1ac, 0xb813 },
		{ 0x1bc, 0xe900 },
		{ 0x1c0, 0x004a },	/* differs: sub-lane 0 writes 0x0055 */
		{ 0x1c8, 0xf0f3 },
		{ 0x124, 0x520c },	/* LAST, and deliberately out of address order */
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(sublane_cfg); i++)
		writel(sublane_cfg[i].val, s + sublane_cfg[i].off);
}

/* 2-lane (pcie0) / 1-lane (pcie2) LONG RX-cal, faithful replay of the golden
 * k0 @0x91324c trace.  `relock` = the per-controller mid-cal CMU relock. */
static void cortina_pcie_long_cal(struct cortina_pcie *cp, void __iomem *s,
				  int lanes, void (*relock)(struct cortina_pcie *))
{
	int lane;

	s0_rmw_all(s, lanes, 0x24, 0x10, 0);	/*  1 clr b4 */
	s0_rmw_all(s, lanes, 0x34, 0, 0x2000);	/*  2 set b13 */
	s0_rmw_all(s, lanes, 0x24, 0x200, 0);	/*  3 strobe b9 low */
	s0_rmw_all(s, lanes, 0x24, 0, 0x200);	/*  4 strobe b9 high */
	s0_rmw_all(s, lanes, 0x34, 0, 0x40);	/*  5 set b6 */
	s0_rmw_all(s, lanes, 0x08, 0, 0x200);	/*  6 set b9 */
	s0_wr_all(s, lanes, 0x40, 0x194);	/*  7 */
	s0_rmw_all(s, lanes, 0xbc, 0, 0x400);	/*  8 set b10 */
	s0_rmw_all(s, lanes, 0x6c, 0x60, 0);	/*  9 clr b5,b6 */
	s0_poll_caldone(s, 0);
	s0_rmw_all(s, lanes, 0x6c, 0x60, 0x20);	/* 10 */
	s0_poll_caldone(s, 0);
	s0_rmw_all(s, lanes, 0x08, 0x200, 0);	/* 11 clr b9 */
	s0_rmw_all(s, lanes, 0x6c, 0x60, 0);	/* 12 */
	s0_wr_all(s, lanes, 0x40, 0x1a4);	/* 13 */
	s0_poll_caldone(s, 0);
	s0_cap_0c(s, 0);			/* 14 lane0 capture 0x0c */
	s0_rmw_all(s, lanes, 0x6c, 0x60, 0x20);	/* 15a re-strobe */
	s0_poll_caldone(s, 0);
	if (lanes > 1)
		s0_cap_0c(s, 1);		/* 15b lane1 capture 0x0c */
	s0_rmw_all(s, lanes, 0x24, 0, 0x10);	/* 16 set b4 */
	s0_rmw_all(s, lanes, 0x34, 0x40, 0);	/* 17 clr b6 */
	s0_rmw_all(s, lanes, 0xbc, 0x400, 0);	/* 18 clr b10 */
	s0_rmw_all(s, lanes, 0x6c, 0x60, 0);	/* 19 */
	s0_wr_all(s, lanes, 0x40, 0x0c);	/* 20 */
	s0_poll_caldone(s, 0);
	s0_cap_2c(s, lanes);			/* 21+22 capture 0x2c */
	s0_rmw_all(s, lanes, 0x34, 0x2000, 0);	/* 23 clr b13 */

	relock(cp);				/* mid-cal CMU relock */

	for (lane = 0; lane < lanes; lane++) {	/* post-restart per-lane tuning */
		u32 b = lane * CORTINA_PCIE_SERDES_LANE_STRIDE;
		u32 c;

		writel(readl(s + b + 0x28) & ~0x60, s + b + 0x28); usleep_range(10, 20);
		writel(readl(s + b + 0x2c) | 0x10, s + b + 0x2c);  usleep_range(10, 20);
		s0_poll_caldone(s, b);
		c = (readl(s + b + CORTINA_SERDES_BER_STAT) >> 9) & 0x7f;
		writel((readl(s + b + 0x90) & ~0x7f) | c, s + b + 0x90); usleep_range(10, 20);
		writel(readl(s + b + 0x28) | 0x60, s + b + 0x28);  usleep_range(10, 20);
	}
}

static bool cortina_pcie0_done;		/* S0 bring-up runs once, globally */

/* The full, proven S0 sequence: PERST0 route+assert -> ...
 * dev/MEASURED-pcie-cortina.c.md sec 16. */
static void cortina_pcie0_bringup_seq(struct cortina_pcie *cp, void __iomem *s)
{
	ca_rmw(cp->rstmgr, CA_GLOBAL_CONFIG, 0, CA_CLKEN_PCIE0 | CA_CLKEN_PCIE0_PS);
	usleep_range(1000, 2000);

	ca_rmw(cp->rstmgr, CA_GLB_GPIO_MUX4, 0, S0_PERST);	/* GLB pinmux: route PERST0 pad -> GPIO */
	ca_rmw(cp->gpio, CA_GPIO_B4_CFG, S0_PERST, 0);		/* PERST# assert (low) */
	ca_rmw(cp->gpio, CA_GPIO_B4_OUT, S0_PERST, 0);

	ca_rmw(cp->rstmgr, CA_RST_BLOCK, 0, S0_CORE_RST);	usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_DPHY, 0, S0_PHY_RST);		usleep_range(1000, 2000);

	cortina_pcie0_serdes_table(s);				/* table (in reset) */

	s0_phy_power_on(cp);					usleep_range(1000, 2000);

	ca_rmw(cp->rstmgr, CA_RST_DPHY, S0_PHY_RST, 0);		usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_BLOCK, S0_CORE_RST, 0);	usleep_range(1000, 2000);

	cortina_pcie_long_cal(cp, s, S0_LANES, cortina_pcie0_hard_relock);

	ca_rmw(cp->gpio, CA_GPIO_B4_OUT, 0, S0_PERST);		/* PERST# deassert */

	dev_info(cp->pci.dev, "pcie0/S0 bring-up: 7c=%08x 17c=%08x 107c=%08x 117c=%08x\n",
		 readl(s + CORTINA_SERDES_SUB(0, 0) + CORTINA_SERDES_BER_STAT),
		 readl(s + CORTINA_SERDES_SUB(0, 1) + CORTINA_SERDES_BER_STAT),
		 readl(s + CORTINA_SERDES_SUB(1, 0) + CORTINA_SERDES_BER_STAT),
		 readl(s + CORTINA_SERDES_SUB(1, 1) + CORTINA_SERDES_BER_STAT));
	cortina_pcie0_done = true;
}

/* Fallback when pcie0 has no (or a disabled) DT node: run the ...
 * dev/MEASURED-pcie-cortina.c.md sec 29. */
static void cortina_pcie0_precondition(struct cortina_pcie *cp)
{
	void __iomem *s = ioremap(CA_S0_SERDES_PHYS, CA_S0_SERDES_SIZE);

	if (!s) {
		dev_warn(cp->pci.dev, "pcie0 precondition: serdes ioremap failed\n");
		return;
	}
	cortina_pcie0_bringup_seq(cp, s);
	iounmap(s);
}

/* pcie0's own host reset (5 GHz RTW8852CE behind it). The ...
 * dev/MEASURED-pcie-cortina.c.md sec 17. */
static bool cortina_pcie0_host_reset(struct cortina_pcie *cp)
{
	cortina_pcie0_bringup_seq(cp, cp->serdes);

	msleep(150);						/* endpoint powers up (PCIe Tpvperl) */

	return cortina_pcie_serdes_ber_notify(cp);
}

/* Full controller bring-up. Ordering and the ~1ms settling ...
 * dev/MEASURED-pcie-cortina.c.md sec 30. */
static bool cortina_pcie_host_reset(struct cortina_pcie *cp)
{
	if (cp->idx == 0)
		return cortina_pcie0_host_reset(cp);

	/* pcie2/S2 (2.4 GHz) below; other ids have no reset sequence yet. */
	if (cp->idx != 2) {
		dev_warn(cp->pci.dev, "reset seq only implemented for pcie0/pcie2 (id=%u)\n",
			 cp->idx);
		/* Nothing was attempted, so there is no lock to wait for: say ...
		 * dev/MEASURED-pcie-cortina.c.md sec 31. */
		return true;
	}

	/* Stock brings pcie0/S0 up FIRST; its 2-lane LONG RX-cal preconditions the
	 * SHARED SerDes analog so pcie2's minimal SHORT cal converges (0x780f) rather
	 * than stalling at 0x500e.  Reproduce that order -- once, before pcie2. */
	if (!cortina_pcie0_done) {
		cortina_pcie0_precondition(cp);
		cortina_pcie0_done = true;
	}

	/* Ungate the SerDes PHY/refclk (gate_pcie2_ps) -- the bootloader leaves it
	 * gated, which is why the cold RX CMU/PLL never locks (0x7c stuck 0x500e).
	 * Ungate the controller clock too (idempotent if the bootloader set it). */
	ca_rmw(cp->rstmgr, CA_GLOBAL_CONFIG, 0, CA_CLKEN_PCIE2 | CA_CLKEN_PCIE2_PS);
	usleep_range(1000, 2000);

	/* PERST# assert: route pin140 via the GLB pinmux, then drive it as a GPIO
	 * output low.  The mux MUST be the GLB/rstmgr register (0xf4320140), not the
	 * per-GPIO window -- on a cold POR it is 0 and nothing else sets it before
	 * PCIe needs the endpoint out of reset. */
	ca_rmw(cp->rstmgr, CA_GLB_GPIO_MUX4, 0, S2_PERST);	/* GLB pinmux: route PERST2 pad -> GPIO */
	ca_rmw(cp->gpio, CA_GPIO_B4_CFG, S2_PERST, 0);		/* dir = output */
	ca_rmw(cp->gpio, CA_GPIO_B4_OUT, S2_PERST, 0);		/* drive low (assert) */

	/* ⚠ RECOVERY ATTEMPT, 2026-09-07 -- keep or drop on the next ...
	 * dev/MEASURED-pcie-cortina.c.md sec 18. */
	msleep(200);

	ca_rmw(cp->rstmgr, CA_RST_BLOCK, 0, S2_CORE_RST);	/* core_reset assert */
	usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_DPHY, 0, S2_PHY_RST);		/* phy_reset assert */
	usleep_range(1000, 2000);

	cortina_pcie_serdes_phy_init(cp);			/* serdes table (in reset) */

	/* phy_power_on: mux SerDes S2 -> PCIe2 + power/de-isolate the lane.  0x39c
	 * is written wholesale to the stock golden value (an RMW inherited the
	 * cold POR and left the lane-power bits clear). */
	ca_rmw(cp->rstmgr, CA_PHY_CONTROL, S2_PHY_CTRL_CLR, S2_PHY_CTRL_SET);
	writel(CA_PHY_ISO_POWER_GOLD, cp->rstmgr + CA_PHY_ISO_POWER);
	usleep_range(1000, 2000);

	/* Stock reset order (k0 host_init @0x912d68): deassert PHY ...
	 * dev/MEASURED-pcie-cortina.c.md sec 19. */
	writel(CA_DPHY_RESET_GOLD, cp->rstmgr + CA_RST_DPHY);	/* phy_reset deassert */
	usleep_range(1000, 2000);
	ca_rmw(cp->rstmgr, CA_RST_BLOCK, S2_CORE_RST, 0);	/* core_reset deassert */
	usleep_range(1000, 2000);

	/* Stock order (k0 host_init): the SHORT cal runs with device/PERST# still
	 * ASSERTED; PERST# is released only at the merge, AFTER the cal, then the BER
	 * poll waits for the endpoint. */
	cortina_pcie_serdes_rx_cal(cp);

	ca_rmw(cp->gpio, CA_GPIO_B4_OUT, 0, S2_PERST);		/* PERST# deassert (high) */
	msleep(150);						/* endpoint powers up (PCIe Tpvperl) */

	return cortina_pcie_serdes_ber_notify(cp);
}

/* Point the glue address decoder at the DBI/config and iATU windows. */
static void cortina_pcie_setup_windows(struct cortina_pcie *cp)
{
	cortina_glbl_writel(cp, cp->dbi_start, CA_PCIE_CFG_START_ADDR);
	cortina_glbl_writel(cp, cp->dbi_end, CA_PCIE_CFG_END_ADDR);

	if (cp->iatu_start)
		cortina_glbl_writel(cp, cp->iatu_start & CA_PCIE_IATU_BASE_MASK,
				    CA_PCIE_IATU_BASE_ADDR);
}

/* ------------------------------------------------------------------ */
/* DWC link ops								      */
/* ------------------------------------------------------------------ */

static enum dw_pcie_ltssm cortina_pcie_get_ltssm(struct dw_pcie *pci)
{
	struct cortina_pcie *cp = to_cortina_pcie(pci);
	u32 val = cortina_glbl_readl(cp, CA_PCIE_PM_STATUS);

	/* The glue LTSSM field uses the same encoding as enum dw_pcie_ltssm. */
	return (enum dw_pcie_ltssm)((val & CA_PM_LTSSM_STATE_MASK) >>
				    CA_PM_LTSSM_STATE_SHIFT);
}

static bool cortina_pcie_link_up(struct dw_pcie *pci)
{
	struct cortina_pcie *cp = to_cortina_pcie(pci);

	return !!(cortina_glbl_readl(cp, CA_PCIE_PM_STATUS) & CA_PM_RDLH_LINK_UP);
}

static int cortina_pcie_start_link(struct dw_pcie *pci)
{
	struct cortina_pcie *cp = to_cortina_pcie(pci);

	if (dw_pcie_link_up(pci))
		return 0;

	cortina_glbl_writel(cp, CA_CORE_LTSSM_ENABLE, CA_PCIE_CORE_CONFIG);
	usleep_range(1000, 2000);

	return 0;
}

static void cortina_pcie_stop_link(struct dw_pcie *pci)
{
	struct cortina_pcie *cp = to_cortina_pcie(pci);

	cortina_glbl_writel(cp, CA_CORE_LINK_DOWN_RST, CA_PCIE_CORE_CONFIG);
	usleep_range(100, 200);
	cortina_glbl_writel(cp, 0, CA_PCIE_CORE_CONFIG);
	usleep_range(100, 200);
}

static const struct dw_pcie_ops cortina_dw_pcie_ops = {
	.link_up	= cortina_pcie_link_up,
	.get_ltssm	= cortina_pcie_get_ltssm,
	.start_link	= cortina_pcie_start_link,
	.stop_link	= cortina_pcie_stop_link,
};

/* MSI (DWC built-in controller, glue-owned demux) The DWC ...
 * dev/MEASURED-pcie-cortina.c.md sec 20. */
static void cortina_pcie_msi_compose(struct irq_data *d, struct msi_msg *msg)
{
	struct dw_pcie_rp *pp = irq_data_get_irq_chip_data(d);
	u64 msi_target = (u64)pp->msi_data;

	msg->address_lo = lower_32_bits(msi_target);
	msg->address_hi = upper_32_bits(msi_target);
	msg->data = d->hwirq;
}

static void cortina_pcie_msi_mask(struct irq_data *d)
{
	struct dw_pcie_rp *pp = irq_data_get_irq_chip_data(d);
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	unsigned int ctrl, res, bit;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);
	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	pp->irq_mask[ctrl] |= BIT(bit);
	dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_MASK + res, pp->irq_mask[ctrl]);
	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void cortina_pcie_msi_unmask(struct irq_data *d)
{
	struct dw_pcie_rp *pp = irq_data_get_irq_chip_data(d);
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	unsigned int ctrl, res, bit;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);
	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	pp->irq_mask[ctrl] &= ~BIT(bit);
	dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_MASK + res, pp->irq_mask[ctrl]);
	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void cortina_pcie_msi_ack(struct irq_data *d)
{
	struct dw_pcie_rp *pp = irq_data_get_irq_chip_data(d);
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	unsigned int ctrl, res, bit;

	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_STATUS + res, BIT(bit));
}

#ifdef CONFIG_SMP
static void cortina_pcie_msi_noop_ack(struct irq_data *d) { }
#endif

/* Mirror the 6.18 mainline dw_pci_msi_bottom_irq_chip ...
 * dev/MEASURED-pcie-cortina.c.md sec 21. */
static struct irq_chip cortina_pcie_msi_bottom_chip = {
	.name			= "CA-PCIe-MSI",
	.irq_compose_msi_msg	= cortina_pcie_msi_compose,
	.irq_mask		= cortina_pcie_msi_mask,
	.irq_unmask		= cortina_pcie_msi_unmask,
#ifdef CONFIG_SMP
	.irq_ack		= cortina_pcie_msi_noop_ack,
	.irq_pre_redirect	= cortina_pcie_msi_ack,
	.irq_set_affinity	= irq_chip_redirect_set_affinity,
#else
	.irq_ack		= cortina_pcie_msi_ack,
#endif
};

/* Program the DWC built-in MSI controller: address + per-vector enable/mask. */
static void cortina_pcie_msi_hw_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	u64 msi_target = (u64)pp->msi_data;
	u32 ctrl, num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;

	for (ctrl = 0; ctrl < num_ctrls; ctrl++) {
		dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_MASK +
				   ctrl * MSI_REG_CTRL_BLOCK_SIZE,
				   pp->irq_mask[ctrl]);
		dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_ENABLE +
				   ctrl * MSI_REG_CTRL_BLOCK_SIZE, ~0);
	}

	dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_LO, lower_32_bits(msi_target));
	dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_HI, upper_32_bits(msi_target));
}

static int cortina_pcie_msi_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct cortina_pcie *cp = to_cortina_pcie(pci);
	struct device *dev = pci->dev;
	void *msi_vaddr = NULL;
	u32 ctrl, val;
	int ret;

	pp->num_vectors = MSI_DEF_NUM_VECTORS;
	for (ctrl = 0; ctrl < pp->num_vectors / MAX_MSI_IRQS_PER_CTRL; ctrl++)
		pp->irq_mask[ctrl] = ~0;

	pp->msi_irq_chip = &cortina_pcie_msi_bottom_chip;
	ret = dw_pcie_allocate_domains(pp);
	if (ret)
		return ret;

	/* The iMSI-RX write target must sit in the low 4GB if possible. */
	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (!ret)
		msi_vaddr = dmam_alloc_coherent(dev, sizeof(u64), &pp->msi_data,
						GFP_KERNEL);
	if (!msi_vaddr) {
		dev_warn(dev, "Failed to allocate 32-bit MSI address\n");
		dma_set_coherent_mask(dev, DMA_BIT_MASK(64));
		msi_vaddr = dmam_alloc_coherent(dev, sizeof(u64), &pp->msi_data,
						GFP_KERNEL);
		if (!msi_vaddr) {
			dev_err(dev, "Failed to allocate MSI address\n");
			irq_domain_remove(pp->irq_domain);
			return -ENOMEM;
		}
	}

	cortina_pcie_msi_hw_init(pp);

	/* Route the DWC MSI controller output to the shared glue interrupt. */
	val = cortina_glbl_readl(cp, CA_PCIE_GLBL_INT_EN0) | CA_INT_MSI_CTRL;
	cortina_glbl_writel(cp, val, CA_PCIE_GLBL_INT_EN0);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Legacy INTx								      */
/* ------------------------------------------------------------------ */

static void cortina_pcie_intx_mask(struct irq_data *d)
{
	struct cortina_pcie *cp = irq_data_get_irq_chip_data(d);
	u32 val;

	val = cortina_glbl_readl(cp, CA_PCIE_GLBL_INT_EN0);
	val &= ~CA_INT_INTX_ALL;
	cortina_glbl_writel(cp, val, CA_PCIE_GLBL_INT_EN0);
}

static void cortina_pcie_intx_unmask(struct irq_data *d)
{
	struct cortina_pcie *cp = irq_data_get_irq_chip_data(d);
	u32 val;

	val = cortina_glbl_readl(cp, CA_PCIE_GLBL_INT_EN0);
	val |= CA_INT_INTX_ALL;
	cortina_glbl_writel(cp, val, CA_PCIE_GLBL_INT_EN0);
}

static struct irq_chip cortina_pcie_intx_chip = {
	.name		= "CA-PCIe-INTx",
	.irq_mask	= cortina_pcie_intx_mask,
	.irq_unmask	= cortina_pcie_intx_unmask,
};

static int cortina_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
				 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &cortina_pcie_intx_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops cortina_pcie_intx_domain_ops = {
	.map	= cortina_pcie_intx_map,
	.xlate	= pci_irqd_intx_xlate,
};

static int cortina_pcie_init_intx(struct cortina_pcie *cp)
{
	struct device *dev = cp->pci.dev;
	struct device_node *node;

	/* Optional: the first child node is the PCIe INTx interrupt-controller. */
	node = of_get_next_child(dev->of_node, NULL);
	if (!node) {
		dev_info(dev, "no INTx interrupt-controller node, INTx disabled\n");
		return 0;
	}

	cp->intx_domain = irq_domain_create_linear(of_fwnode_handle(node),
						   PCI_NUM_INTX,
						   &cortina_pcie_intx_domain_ops,
						   cp);
	of_node_put(node);
	if (!cp->intx_domain) {
		dev_err(dev, "failed to create INTx IRQ domain\n");
		return -ENODEV;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Shared interrupt demux						      */
/* ------------------------------------------------------------------ */

static irqreturn_t cortina_pcie_irq_handler(int irq, void *arg)
{
	struct cortina_pcie *cp = arg;
	struct dw_pcie_rp *pp = &cp->pci.pp;
	u32 val, bit;

	val = cortina_glbl_readl(cp, CA_PCIE_GLBL_INT0);
	if (!val)
		return IRQ_NONE;

	if (val & CA_INT_MSI_CTRL)
		dw_handle_msi_irq(pp);

	if (cp->intx_domain) {
		for (bit = 0; bit < PCI_NUM_INTX; bit++) {
			/* INTA..INTD "asserted" bits are 2 apart in INT0. */
			if (val & (CA_INT_RADM_INTA_ASSERTED << (2 * bit)))
				generic_handle_domain_irq(cp->intx_domain, bit);
		}
	}

	if (val & CA_INT_LINK_DOWN) {
		dev_err(cp->pci.dev, "Link down (LTSSM 0x%x)\n",
			cortina_pcie_get_ltssm(&cp->pci));
		cortina_glbl_writel(cp, CA_CORE_LINK_DOWN_RST, CA_PCIE_CORE_CONFIG);
	}

	/* Write-1-to-clear the latched glue status. */
	cortina_glbl_writel(cp, val, CA_PCIE_GLBL_INT0);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* DWC host init (glue HW bring-up)					      */
/* ------------------------------------------------------------------ */

static int cortina_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct cortina_pcie *cp = to_cortina_pcie(pci);
	struct platform_device *pdev = to_platform_device(pci->dev);
	int ret, irq;

	/* Keep the shared line quiet until MSI/INTx are wired up. */
	cortina_glbl_writel(cp, 0, CA_PCIE_GLBL_INT_EN0);

	/* ★ WHY THIS IS ONE SHOT AGAIN, AND WHAT WAS TRIED ...
	 * dev/MEASURED-pcie-cortina.c.md sec 22. */
	if (!cortina_pcie_host_reset(cp) && cp->idx == 2)
		dev_err(pci->dev,
			"SerDes BER-notify not asserted -- the link is unlikely to train\n");

	cortina_pcie_setup_windows(cp);

	ret = cortina_pcie_init_intx(cp);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(pci->dev, irq, cortina_pcie_irq_handler,
			       IRQF_SHARED, "cortina-pcie", cp);
	if (ret) {
		dev_err(pci->dev, "failed to request IRQ %d\n", irq);
		return ret;
	}

	/* Enable link-down / error monitoring (MSI + INTx enabled later). */
	cortina_glbl_writel(cp,
			    cortina_glbl_readl(cp, CA_PCIE_GLBL_INT_EN0) |
			    CA_INT_MISC_ALL, CA_PCIE_GLBL_INT_EN0);

	return 0;
}

static const struct dw_pcie_host_ops cortina_pcie_host_ops = {
	.init		= cortina_pcie_host_init,
	.msi_init	= cortina_pcie_msi_init,
};

/* ------------------------------------------------------------------ */
/* Probe								      */
/* ------------------------------------------------------------------ */

static int cortina_pcie_map_regions(struct platform_device *pdev,
				    struct cortina_pcie *cp)
{
	struct dw_pcie *pci = &cp->pci;
	struct device *dev = &pdev->dev;
	struct resource *res;

	cp->glbl = devm_platform_ioremap_resource_byname(pdev, "glbl_regs");
	if (IS_ERR(cp->glbl))
		return PTR_ERR(cp->glbl);

	/* DBI: map here and hand it to the DWC core (it maps "dbi" otherwise). */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc_dbi");
	pci->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);
	pci->dbi_phys_addr = res->start;
	cp->dbi_start = res->start;
	cp->dbi_end = res->end;

	/* Unrolled iATU: map here and hand it to the DWC core. */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iatu");
	pci->atu_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pci->atu_base))
		return PTR_ERR(pci->atu_base);
	pci->atu_phys_addr = res->start;
	pci->atu_size = resource_size(res);
	cp->iatu_start = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "serdes_phy");
	cp->serdes = devm_ioremap_resource(dev, res);
	if (IS_ERR(cp->serdes))
		return PTR_ERR(cp->serdes);
	cp->serdes_size = resource_size(res);

	cp->rstmgr = devm_ioremap(dev, CA_RSTMGR_PHYS, CA_RSTMGR_SIZE);
	cp->gpio = devm_ioremap(dev, CA_GPIO_PHYS, CA_GPIO_SIZE);
	if (!cp->rstmgr || !cp->gpio)
		return -ENOMEM;

	return 0;
}

/* /proc/cortina_pcie_serdes<id> -- live SerDes/glue register ...
 * dev/MEASURED-pcie-cortina.c.md sec 23. */
static ssize_t cortina_pcie_dbg_write(struct file *f, const char __user *ubuf,
				      size_t len, loff_t *ppos)
{
	struct cortina_pcie *cp = pde_data(file_inode(f));
	char buf[64];
	u32 o, v;

	if (!cp || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	if (sscanf(buf, "w %x %x", &o, &v) == 2) {
		o &= 0xffc;
		writel(v, cp->serdes + o);
		dev_info(cp->pci.dev, "serdes+0x%03x <- 0x%08x (rb 0x%08x)\n",
			 o, v, readl(cp->serdes + o));
	} else if (sscanf(buf, "r %x", &o) == 1) {
		o &= 0xffc;
		dev_info(cp->pci.dev, "serdes+0x%03x = 0x%08x\n",
			 o, readl(cp->serdes + o));
	} else if (sscanf(buf, "G %x %x", &o, &v) == 2) {
		o &= 0x3fc;
		cortina_glbl_writel(cp, v, o);
		dev_info(cp->pci.dev, "glbl+0x%03x <- 0x%08x (rb 0x%08x)\n",
			 o, v, cortina_glbl_readl(cp, o));
	} else if (sscanf(buf, "g %x", &o) == 1) {
		o &= 0x3fc;
		dev_info(cp->pci.dev, "glbl+0x%03x = 0x%08x\n",
			 o, cortina_glbl_readl(cp, o));
	} else if (sscanf(buf, "C %x %x", &o, &v) == 2) {
		o &= 0x3fc;					/* soc_clks/reset-mgr 0xf4320000 */
		writel(v, cp->rstmgr + o);
		dev_info(cp->pci.dev, "rstmgr+0x%03x <- 0x%08x (rb 0x%08x)\n",
			 o, v, readl(cp->rstmgr + o));
	} else if (sscanf(buf, "c %x", &o) == 1) {
		o &= 0x3fc;
		dev_info(cp->pci.dev, "rstmgr+0x%03x = 0x%08x\n",
			 o, readl(cp->rstmgr + o));
	}
	return len;
}

static const struct proc_ops cortina_pcie_dbg_ops = {
	.proc_write = cortina_pcie_dbg_write,
	.proc_lseek = noop_llseek,
};

static int cortina_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct cortina_pcie *cp;
	char name[32];
	u32 lanes;
	int i, ret;

	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;

	cp->pci.dev = dev;
	cp->pci.ops = &cortina_dw_pcie_ops;
	cp->pci.pp.ops = &cortina_pcie_host_ops;
	platform_set_drvdata(pdev, cp);

	ret = cortina_pcie_map_regions(pdev, cp);
	if (ret)
		return ret;

	if (of_property_read_u32(np, "id", &cp->idx))
		cp->idx = 0;

	if (of_property_read_u32(np, "num-lanes", &lanes) ||
	    lanes < 1 || lanes > CORTINA_PCIE_MAX_LANES)
		lanes = 1;
	cp->lanes = lanes;
	cp->pci.num_lanes = lanes;

	ret = cortina_pcie_parse_serdes_cfg(cp);
	if (ret)
		return ret;

	/* Clocks, resets and PHYs are optional so the root complex ...
	 * dev/MEASURED-pcie-cortina.c.md sec 24. */
	cp->bus_clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(cp->bus_clk))
		return dev_err_probe(dev, PTR_ERR(cp->bus_clk),
				     "failed to get bus clock\n");

	cp->core_reset = devm_reset_control_get_optional_exclusive(dev, "core_reset");
	if (IS_ERR(cp->core_reset))
		return PTR_ERR(cp->core_reset);

	cp->phy_reset = devm_reset_control_get_optional_exclusive(dev, "phy_reset");
	if (IS_ERR(cp->phy_reset))
		return PTR_ERR(cp->phy_reset);

	cp->device_reset = devm_reset_control_get_optional_exclusive(dev, "device_reset");
	if (IS_ERR(cp->device_reset))
		return PTR_ERR(cp->device_reset);

	cp->device_power = devm_reset_control_get_optional_exclusive(dev, "device_power");
	if (IS_ERR(cp->device_power))
		return PTR_ERR(cp->device_power);
	reset_control_reset(cp->device_power);

	for (i = 0; i < cp->lanes; i++) {
		snprintf(name, sizeof(name), "pcie-phy%d", i);
		cp->phy[i] = devm_phy_optional_get(dev, name);
		if (IS_ERR(cp->phy[i]))
			return dev_err_probe(dev, PTR_ERR(cp->phy[i]),
					     "failed to get %s\n", name);
	}

	/* Per-controller name so pcie0 and pcie2 don't collide on a shared /proc
	 * entry (a duplicate proc_create WARNs at proc_register and leaves the
	 * spy tool bound to whichever probed last). */
	snprintf(name, sizeof(name), "cortina_pcie_serdes%u", cp->idx);
	cp->dbg_pde = proc_create_data(name, 0200, NULL,
				       &cortina_pcie_dbg_ops, cp);

	return dw_pcie_host_init(&cp->pci.pp);
}

static void cortina_pcie_remove(struct platform_device *pdev)
{
	struct cortina_pcie *cp = platform_get_drvdata(pdev);

	proc_remove(cp->dbg_pde);
	dw_pcie_host_deinit(&cp->pci.pp);
}

static const struct of_device_id cortina_pcie_of_match[] = {
	{ .compatible = "cortina,venus-pcie", },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_pcie_of_match);

static struct platform_driver cortina_pcie_driver = {
	.probe	= cortina_pcie_probe,
	.remove	= cortina_pcie_remove,
	.driver = {
		.name		= "cortina-pcie",
		.of_match_table	= cortina_pcie_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(cortina_pcie_driver);

MODULE_DESCRIPTION("Cortina-Access Venus PCIe host controller driver");
MODULE_LICENSE("GPL");
