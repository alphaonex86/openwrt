// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access NI (network engine) Ethernet driver for the Realtek
 * RTL9607F "Elnath" GPON SoC.
 *
 * Bring-up stage M2a: platform probe, register-window mapping,
 * reserved-memory resolution, internal MDIO bus and GbE PHY discovery.
 * Stage M2b adds the TX datapath (cortina-ni-tx.c).
 */

#include <linux/bitfield.h>
#include <linux/build_bug.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_reserved_mem.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include "cortina-ni.h"

/* CA_NI_DRV_NAME now lives in cortina-ni.h: `ethtool -i` reports it too, so
 * the driver name had to stop being a literal private to this file. */

struct cortina_ni_window_desc {
	u8		idx;
	const char	*name;
	bool		required;
};

/* Only the windows this stage touches; the rest are mapped in M2c. */
static const struct cortina_ni_window_desc cortina_ni_windows[] = {
	{ CA_NI_WIN_NI,		"ni-core",	true  },
	{ CA_NI_WIN_NE_INTR,	"ne-intr",	true  },
	{ CA_NI_WIN_MDIO,	"mdio",		true  },
	{ CA_NI_WIN_XRAM,	"xram",		false },
	{ CA_NI_WIN_DMA,	"dma-ldma",	true  },	/* TX rings */
	{ CA_NI_WIN_SRAM,	"bd-sram",	false },
	{ CA_NI_WIN_GLB,	"glb",		true  },
	{ CA_NI_WIN_AXI_REO,	"axi-reo",	true  },	/* RMU AXI rd/wr reorder - RX dequeue DMA */
	/* ★ FBM (Free Buffer Manager) + LDMA-aux windows: the RMU allocates a buffer to
	 * DMA each admitted RX frame into, and that alloc routes through the FBM.  Left
	 * unmapped/uninitialised, the RMU can't allocate -> never admits (0x6900=0,
	 * wptr=0) - the same whole-window-missing class as axi-reo.  DTS reg idx 16 +
	 * 18-21 are already present; map non-required so a DTS gap can't kill probe. */
	{ CA_NI_WIN_LDMA_AUX,	"ldma-aux",	false },
	{ CA_NI_WIN_FBM_GLB,	"fbm-glb",	false },
	{ CA_NI_WIN_FBM_AXI,	"fbm-axi",	false },
	{ CA_NI_WIN_FBM_CPU,	"fbm-cpu",	false },
	{ CA_NI_WIN_FBM_POOL,	"fbm-pool",	false },
	{ CA_NI_WIN_GPHY,	"gphy",		false },
	{ CA_NI_WIN_GPHY_WRAP,	"gphy-wrap",	false },
};

static bool cortina_ni_phy_is_internal(int addr)
{
	return addr >= CA_NI_GPHY_FIRST &&
	       addr < CA_NI_GPHY_FIRST + CA_NI_GPHY_COUNT;
}

/*
 * External MDIO master, clause 22 (stock aal_mdio_read/write_direct):
 * write the command word, pulse CTRL.start, poll CTRL.done, then - for a
 * read - fetch RDDATA *before* acking done (W1C), matching stock order.
 */
static int cortina_ni_mdio_c22_cmd(struct cortina_ni *ni, u32 cmd, u32 *data)
{
	void __iomem *base = ni->win[CA_NI_WIN_MDIO];
	u32 ctrl;
	int ret;

	writel(cmd, base + CA_NI_MDIO_ADDR);
	/*
	 * Kick the frame.  Writing DONE too clears any stale done latch
	 * (stock ORs START into the current CTRL value, which has the same
	 * effect when done was left set).
	 */
	writel(CA_NI_MDIO_CTRL_START | CA_NI_MDIO_CTRL_DONE,
	       base + CA_NI_MDIO_CTRL);

	ret = readl_poll_timeout(base + CA_NI_MDIO_CTRL, ctrl,
				 ctrl & CA_NI_MDIO_CTRL_DONE,
				 1, CA_NI_MDIO_TIMEOUT_US);
	if (ret)
		return ret;

	if (data)
		*data = readl(base + CA_NI_MDIO_RDDATA) & 0xffff;

	/* ack: done is write-1-clear */
	writel(ctrl | CA_NI_MDIO_CTRL_DONE, base + CA_NI_MDIO_CTRL);
	return 0;
}

static int cortina_ni_mdio_master_read(struct cortina_ni *ni, int addr,
				       int regnum)
{
	u32 cmd, data;
	int ret;

	cmd = FIELD_PREP(CA_NI_MDIO_ADDR_PHY, addr) |
	      FIELD_PREP(CA_NI_MDIO_ADDR_REG, regnum) |
	      FIELD_PREP(CA_NI_MDIO_ADDR_OP, CA_NI_MDIO_OP_RD) |
	      CA_NI_MDIO_ADDR_RD_WR;

	ret = cortina_ni_mdio_c22_cmd(ni, cmd, &data);
	if (ret)
		return ret;

	return data;
}

static int cortina_ni_mdio_master_write(struct cortina_ni *ni, int addr,
					int regnum, u16 val)
{
	void __iomem *base = ni->win[CA_NI_WIN_MDIO];
	u32 cmd;

	cmd = FIELD_PREP(CA_NI_MDIO_ADDR_PHY, addr) |
	      FIELD_PREP(CA_NI_MDIO_ADDR_REG, regnum) |
	      FIELD_PREP(CA_NI_MDIO_ADDR_OP, CA_NI_MDIO_OP_WR);

	writel(val, base + CA_NI_MDIO_WRDATA);
	return cortina_ni_mdio_c22_cmd(ni, cmd, NULL);
}

/*
 * Internal quad GbE PHYs: registers are memory-mapped in the GPHY window,
 * one 256K bank per PHY.  Regs 0x10..0x17 are banked by the (software)
 * page-select shadow; all other MII regs live at a fixed offset.
 */
static u32 cortina_ni_gphy_reg(struct cortina_ni *ni, int addr, int regnum)
{
	u32 off = (addr - CA_NI_GPHY_FIRST) * CA_NI_GPHY_BANK_STRIDE;

	if (regnum >= CA_NI_GPHY_PAGED_FIRST &&
	    regnum <= CA_NI_GPHY_PAGED_LAST)
		off += ni->gphy_page[addr - CA_NI_GPHY_FIRST] *
			       CA_NI_GPHY_PAGE_STRIDE +
		       (regnum - CA_NI_GPHY_PAGED_FIRST) *
			       CA_NI_GPHY_REG_STRIDE;
	else
		off += CA_NI_GPHY_MII_BASE + regnum * CA_NI_GPHY_REG_STRIDE;

	return off;
}

static int cortina_ni_gphy_read(struct cortina_ni *ni, int addr, int regnum)
{
	if (regnum == 0x1f)	/* page select: software shadow only */
		return ni->gphy_page[addr - CA_NI_GPHY_FIRST];

	return readl(ni->win[CA_NI_WIN_GPHY] +
		     cortina_ni_gphy_reg(ni, addr, regnum)) & 0xffff;
}

static int cortina_ni_gphy_write(struct cortina_ni *ni, int addr, int regnum,
				 u16 val)
{
	if (regnum == 0x1f)
		ni->gphy_page[addr - CA_NI_GPHY_FIRST] = val;
	else
		writel(val, ni->win[CA_NI_WIN_GPHY] +
			    cortina_ni_gphy_reg(ni, addr, regnum));

	/* stock delays 1 ms after every internal-PHY write */
	usleep_range(1000, 1500);
	return 0;
}

/*
 * Internal-GPHY firmware (SRAM) patch (see cortina-ni-regs.h).  The on-PHY
 * microcontroller's DSP-SRAM must be seeded with the apro_gen2 firmware body
 * before the PHY forwards a frame across its system-side GMII to the NI MAC.
 * We keep U-Boot's live link (never reset the PHY), so the uC is running - the
 * gate/lock handshake holds it, the firmware image is written word-by-word,
 * then the uC is resumed.  Applied per bank.  The raw OCP accessors are plain
 * memory-mapped readl/writel (no page shadow, no 1 ms MDIO delay - the image
 * is streamed back-to-back).
 */
struct cortina_ni_gphy_patch_word {
	u16	addr;	/* SRAM word address (via OCP 0xa436) */
	u16	data;	/* 16-bit word        (via OCP 0xa438) */
};

/*
 * apro_gen2 internal-GPHY uC firmware body -- NO LONGER A C ARRAY.
 *
 * ★ THE CRITERION IS WHICH PROCESSOR EXECUTES IT.  A register-write list the
 * SoC CPU performs stays in C, because that IS driver logic.  These 560 words
 * are seeded into the internal PHY's DSP-SRAM and executed by that PHY'S OWN
 * MICROCONTROLLER -- a body for another engine, which is what request_firmware()
 * is for.  It was the only array embedded_blob_audit.py still marked
 * EXTERNALISE.
 *
 * It now ships as "cortina-apro-gen2.fw": magic "APROGEN2", a big-endian u16
 * count, then count x (big-endian u16 addr, big-endian u16 data).  Dull on
 * purpose, and read with EXPLICIT BYTE MATH below so the same file is parsed
 * identically whatever the host's byte order.
 *
 * The file is generated FROM this driver's former table and proven identical to
 * it word for word: ONU-test-case/gphy_fw_extract.py --verify.
 *
 * ⚠ IF IT CANNOT BE LOADED THE PATCH IS SKIPPED, LOUDLY, and the driver
 * continues.  That is deliberate: refusing to probe would take the whole LAN
 * down, while skipping is exactly the state `gphy_patch=0` already selects --
 * and that state was MEASURED indistinguishable from the default on this board
 * (3/4 ports up either way, 527.7 vs 500.6 Mbps).  A missing file must never be
 * silent, and must never be fatal either.
 */

static u16 cortina_ni_gphy_ocp_read(void __iomem *bank, u16 ocp)
{
	return readl(bank + CA_NI_GPHY_OCP(ocp)) & 0xffff;
}

static void cortina_ni_gphy_ocp_write(void __iomem *bank, u16 ocp, u16 val)
{
	writel(val, bank + CA_NI_GPHY_OCP(ocp));
}

/* write one firmware word: address register, then data register */
static void cortina_ni_gphy_sram_write(void __iomem *bank, u16 addr, u16 data)
{
	cortina_ni_gphy_ocp_write(bank, CA_NI_GPHY_SRAM_ADDR, addr);
	cortina_ni_gphy_ocp_write(bank, CA_NI_GPHY_SRAM_DATA, data);
}

/*
 * Load the internal-GPHY SRAM firmware image + resume the uC, per bank.
 *
 * TIMING: the uC SRAM is only writable while the uC is HELD.  The uC only
 * becomes lockable AFTER the MDIO/GPHY init - i.e. around LINK-UP, NOT at probe
 * (where SRAM writes are ignored).  So this runs from the link-up hook
 * (cortina_ni_rx_link_up), not probe.  For each bank we take the gate/lock/
 * ready-poll handshake (cortina-ni-regs.h), stream the firmware image, then
 * clear the hold bit so the uC runs the freshly-written image (its default ROM
 * firmware does NOT forward - the SRAM image is what enables line<->system
 * forwarding).
 *
 * One-shot per bank per boot (ni->gphy_patched[]); a bank whose gate reads
 * EXT_INI (not lockable yet) is left for the next link-up.  If the ready-poll
 * times out we log it and write the words anyway (the hold bit still took).
 */
static bool gphy_patch = true;
module_param(gphy_patch, bool, 0444);
MODULE_PARM_DESC(gphy_patch,
		 "load the internal-GPHY SRAM firmware image before resuming the uC (default on)");

#define CA_NI_GPHY_FW_NAME	"cortina-apro-gen2.fw"
#define CA_NI_GPHY_FW_MAGIC	"APROGEN2"
#define CA_NI_GPHY_FW_HDR	10	/* 8 magic + 2 count */

void cortina_ni_gphy_patch_and_resume(struct cortina_ni *ni)
{
	void __iomem *win = ni->win[CA_NI_WIN_GPHY];
	const struct firmware *fw = NULL;
	const u8 *words = NULL;
	unsigned int nwords = 0;
	unsigned int b, i, t;

	if (!win)
		return;		/* GPHY window not mapped */

	/* One load for all four banks.  Every malformed shape yields NO words
	 * rather than the ones that happened to fit: seeding the uC with half an
	 * image brings the link up WRONG, which is worse than not patching. */
	if (gphy_patch && request_firmware(&fw, CA_NI_GPHY_FW_NAME, ni->dev)) {
		dev_err(ni->dev,
			"gphy: %s not found -- the internal-PHY microcode is NOT "
			"being applied. This is the same state as gphy_patch=0; "
			"the driver continues so the LAN still comes up.\n",
			CA_NI_GPHY_FW_NAME);
		fw = NULL;
	}
	if (fw) {
		if (fw->size < CA_NI_GPHY_FW_HDR ||
		    memcmp(fw->data, CA_NI_GPHY_FW_MAGIC, 8)) {
			dev_err(ni->dev, "gphy: %s has no APROGEN2 header -- "
				"refusing to seed the uC\n", CA_NI_GPHY_FW_NAME);
		} else {
			unsigned int n = ((unsigned int)fw->data[8] << 8) |
					 fw->data[9];

			if (fw->size != CA_NI_GPHY_FW_HDR + 4u * n) {
				dev_err(ni->dev,
					"gphy: %s declares %u word(s) but is %zu "
					"bytes -- truncated or padded, refusing "
					"to seed the uC\n",
					CA_NI_GPHY_FW_NAME, n, fw->size);
			} else {
				words = fw->data + CA_NI_GPHY_FW_HDR;
				nwords = n;
			}
		}
	}

	for (b = 0; b < CA_NI_GPHY_COUNT; b++) {
		void __iomem *bank = win + CA_NI_GPHY_BANK(b);
		u16 gate, ready = 0;
		bool ok = false;

		if (ni->gphy_patched[b])
			continue;	/* one-shot per bank per boot */

		/* gate: EXT_INI = uC not lockable yet, retry next link-up */
		gate = readl(bank + CA_NI_GPHY_LOCK_GATE) & 0xffff;
		if ((gate & CA_NI_GPHY_LOCK_GATE_MASK) ==
		    CA_NI_GPHY_LOCK_GATE_EXT_INI)
			continue;

		/* hold the uC (SRAM only writable while held) */
		writel(readl(bank + CA_NI_GPHY_LOCK) | CA_NI_GPHY_LOCK_HOLD,
		       bank + CA_NI_GPHY_LOCK);

		/* wait for pcs-locked, bounded; on timeout patch anyway */
		for (t = 0; t < CA_NI_GPHY_LOCK_READY_TRIES; t++) {
			ready = readl(bank + CA_NI_GPHY_LOCK_READY) & 0xffff;
			if ((ready & CA_NI_GPHY_LOCK_READY_MASK) ==
			    CA_NI_GPHY_LOCK_READY_VAL) {
				ok = true;
				break;
			}
			udelay(100);
		}

		/* stream the live-dumped firmware, one word at a time, in order */
		/* explicit byte math, never a cast over the file's bytes */
		for (i = 0; i < nwords; i++)
			cortina_ni_gphy_sram_write(bank,
				(u16)(((u16)words[4 * i] << 8) | words[4 * i + 1]),
				(u16)(((u16)words[4 * i + 2] << 8) |
				      words[4 * i + 3]));

		/* resume: clear the hold bit so the uC runs the fresh firmware */
		writel(readl(bank + CA_NI_GPHY_LOCK) & ~CA_NI_GPHY_LOCK_HOLD,
		       bank + CA_NI_GPHY_LOCK);

		ni->gphy_patched[b] = true;

		dev_info(ni->dev,
			 "gphy patch bank %u: gate=0x%04x ready=%s wrote %u words\n",
			 b, gate, ok ? "ok" : "timeout", nwords);
	}
	release_firmware(fw);
}

static int cortina_ni_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct cortina_ni *ni = bus->priv;

	regnum &= 0x1f;
	if (cortina_ni_phy_is_internal(addr) && ni->win[CA_NI_WIN_GPHY])
		return cortina_ni_gphy_read(ni, addr, regnum);

	return cortina_ni_mdio_master_read(ni, addr, regnum);
}

static int cortina_ni_mdio_write(struct mii_bus *bus, int addr, int regnum,
				 u16 val)
{
	struct cortina_ni *ni = bus->priv;

	regnum &= 0x1f;
	if (cortina_ni_phy_is_internal(addr) && ni->win[CA_NI_WIN_GPHY])
		return cortina_ni_gphy_write(ni, addr, regnum, val);

	return cortina_ni_mdio_master_write(ni, addr, regnum, val);
}

/*
 * GPHY-wrapper enable (stock aal_mdio_global_init + the patch_phy_done bit
 * from aal_internal_phy_init).  We FORCE the stock golden EN0/EN1 rather than
 * OR-onto-U-Boot: EN1 bit12 (patch_phy_done) is the GPHY->port-MAC datapath
 * release, and U-Boot leaves it non-deterministically set -> the whole
 * "RX works some boots, dead others" bug.  Writing the exact golden values
 * (EN0 = 0xFF000000, EN1 = 0x1001) makes ingress deterministic every boot.
 * The internal PHYs are memory-mapped (win GPHY), so forcing these MDIO-OCP
 * bits does not disturb our register reads.
 */
static void cortina_ni_mdio_hw_enable(struct cortina_ni *ni)
{
	void __iomem *wrap = ni->win[CA_NI_WIN_GPHY_WRAP];

	if (!wrap)
		return;

	writel(CA_NI_GPHY_WRAP_EN0_VAL, wrap + CA_NI_GPHY_WRAP_EN0);
	writel(CA_NI_GPHY_WRAP_EN1_VAL, wrap + CA_NI_GPHY_WRAP_EN1);
	usleep_range(1000, 1500);
	dev_info(ni->dev, "GPHY wrapper: EN0=0x%08x EN1=0x%08x (patch_phy_done set)\n",
		 readl(wrap + CA_NI_GPHY_WRAP_EN0),
		 readl(wrap + CA_NI_GPHY_WRAP_EN1));
}

static int cortina_ni_map_windows(struct cortina_ni *ni)
{
	struct device *dev = ni->dev;
	struct device_node *np = dev->of_node;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(cortina_ni_windows); i++) {
		const struct cortina_ni_window_desc *w = &cortina_ni_windows[i];
		struct resource res;

		ret = of_address_to_resource(np, w->idx, &res);
		if (ret || !resource_size(&res)) {
			if (w->required)
				return dev_err_probe(dev, ret ? ret : -EINVAL,
						     "no reg entry %u (%s)\n",
						     w->idx, w->name);
			dev_warn(dev, "window %u (%s) absent, skipping\n",
				 w->idx, w->name);
			continue;
		}

		ni->win[w->idx] = devm_ioremap(dev, res.start,
					       resource_size(&res));
		if (!ni->win[w->idx]) {
			if (w->required)
				return dev_err_probe(dev, -ENOMEM,
						     "cannot map window %u (%s) %pR\n",
						     w->idx, w->name, &res);
			dev_warn(dev, "cannot map window %u (%s) %pR\n",
				 w->idx, w->name, &res);
			continue;
		}
		ni->winsz[w->idx] = resource_size(&res);	/* peek bounds */

		dev_info(dev, "window %u (%s): %pR mapped\n",
			 w->idx, w->name, &res);
	}

	/*
	 * Stock ca_ne_reg_init also unconditionally maps a 4K peripheral
	 * block at (32-bit) 0xf4329000, not described in the DT.
	 */
	ni->peri = devm_ioremap(ni->dev, CA_NI_PERI_PHYS, CA_NI_PERI_SIZE);
	if (!ni->peri)
		return dev_err_probe(dev, -ENOMEM,
				     "cannot map peri block @%#x\n",
				     CA_NI_PERI_PHYS);
	dev_info(dev, "peri block @%#x (4K) mapped\n", CA_NI_PERI_PHYS);

	/* window-map liveness proof: GLB+0 is a read-only chip/JTAG ID */
	dev_info(dev, "GLB chip id: 0x%08x\n",
		 readl(ni->win[CA_NI_WIN_GLB] + CA_NI_GLB_JTAG_ID));

	return 0;
}

static void cortina_ni_log_reserved_mem(struct cortina_ni *ni)
{
	struct device_node *np = ni->dev->of_node;
	int i;

	/* two DDR pools: coherent buffer + noncache packet-buffer pool */
	for (i = 0; i < 2; i++) {
		struct device_node *rn;
		struct reserved_mem *rmem;

		rn = of_parse_phandle(np, "memory-region", i);
		if (!rn) {
			dev_warn(ni->dev,
				 "memory-region %d absent (needed from M2b)\n",
				 i);
			continue;
		}

		rmem = of_reserved_mem_lookup(rn);
		if (rmem)
			dev_info(ni->dev,
				 "reserved pool %d (%s): base %pa size %pa\n",
				 i, rmem->name, &rmem->base, &rmem->size);
		else
			dev_warn(ni->dev,
				 "memory-region %d (%pOF) not in reserved-memory\n",
				 i, rn);
		of_node_put(rn);
	}
}

static void cortina_ni_scan_phys(struct cortina_ni *ni)
{
	int addr;

	for (addr = CA_NI_GPHY_FIRST;
	     addr < CA_NI_GPHY_FIRST + CA_NI_GPHY_COUNT; addr++) {
		int id1, id2;
		u32 id;

		id1 = mdiobus_read(ni->mii, addr, MII_PHYSID1);
		id2 = mdiobus_read(ni->mii, addr, MII_PHYSID2);
		if (id1 < 0 || id2 < 0) {
			dev_warn(ni->dev, "PHY %d: read failed (%d/%d)\n",
				 addr, id1, id2);
			continue;
		}

		id = (u32)id1 << 16 | id2;
		dev_info(ni->dev, "PHY %d: id 0x%08x (%s, expected 0x%08x)\n",
			 addr, id,
			 (id & CA_NI_GPHY_PHY_ID_MASK) ==
			 (CA_NI_GPHY_PHY_ID & CA_NI_GPHY_PHY_ID_MASK) ?
			 "match" : "MISMATCH",
			 CA_NI_GPHY_PHY_ID);

		/*
		 * Diagnostic A/B: if the memory-mapped GPHY path returned
		 * nothing sane, also try the external MDIO master once so
		 * the boot log settles which path reaches these PHYs.
		 */
		if ((id == 0 || id == 0xffffffff) && ni->win[CA_NI_WIN_GPHY]) {
			id1 = cortina_ni_mdio_master_read(ni, addr,
							  MII_PHYSID1);
			id2 = cortina_ni_mdio_master_read(ni, addr,
							  MII_PHYSID2);
			dev_info(ni->dev,
				 "PHY %d via MDIO master: id1=%d id2=%d\n",
				 addr, id1, id2);
		}
	}
}

static int cortina_ni_mdio_init(struct cortina_ni *ni)
{
	struct device *dev = ni->dev;
	struct device_node *mdio_np;
	struct mii_bus *bus;
	int ret;

	bus = devm_mdiobus_alloc(dev);
	if (!bus)
		return -ENOMEM;

	bus->name = CA_NI_DRV_NAME "-mdio";
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));
	bus->parent = dev;
	bus->priv = ni;
	bus->read = cortina_ni_mdio_read;
	bus->write = cortina_ni_mdio_write;
	/* limit auto-scan to the internal PHYs at addresses 1..4 */
	bus->phy_mask = ~(u32)GENMASK(CA_NI_GPHY_FIRST + CA_NI_GPHY_COUNT - 1,
				      CA_NI_GPHY_FIRST);

	cortina_ni_mdio_hw_enable(ni);

	/*
	 * Do NOT reset the PHYs here: U-Boot already brought the TFTP port's
	 * link up and we want to keep it provable at this stage.
	 */
	mdio_np = of_get_child_by_name(dev->of_node, "mdio");
	ret = devm_of_mdiobus_register(dev, bus, mdio_np);
	of_node_put(mdio_np);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register MDIO bus\n");

	ni->mii = bus;
	return 0;
}

/* ------------------------------------------------------------------ */
/* the arbitrary-window register peek, read-only (debugfs .../cortina-ni/peek) */
/* Decisive good-vs-bad-boot diff tool - every RX-subset register reads */
/* identical on a working and a broken boot, so the deciding bit lives  */
/* in a register the fixed spy does not print.  Bounded, plain readl,   */
/* no side effects.  (project rule: dump/spy stays first-class.)        */
/* ------------------------------------------------------------------ */

/* window name -> selector.  "peri" is the non-DT 4K @0xf4329000 block. */
static const struct {
	const char	*name;
	u8		win;
} cortina_ni_peek_wins[] = {
	{ "ni",		CA_NI_WIN_NI		},
	{ "dma",	CA_NI_WIN_DMA		},
	{ "glb",	CA_NI_WIN_GLB		},
	{ "gphy",	CA_NI_WIN_GPHY		},
	{ "wrap",	CA_NI_WIN_GPHY_WRAP	},
	{ "reo",	CA_NI_WIN_AXI_REO	},
	{ "mdio",	CA_NI_WIN_MDIO		},
	{ "intr",	CA_NI_WIN_NE_INTR	},
	{ "sgmii",	CA_NI_WIN_SGMII_PCS	},
	{ "peri",	CA_NI_PEEK_PERI		},
};

/* resolve a peek window selector to {base, size}; NULL base = unavailable */
static void __iomem *cortina_ni_peek_base(struct cortina_ni *ni, u8 win,
					  size_t *size)
{
	if (win == CA_NI_PEEK_PERI) {
		*size = CA_NI_PERI_SIZE;
		return ni->peri;
	}
	if (win >= CA_NI_WIN_COUNT)
		return NULL;
	*size = ni->winsz[win];
	return ni->win[win];
}

/*
 * ★★ THE BOUND THAT WAS MISSING: A MAPPED WINDOW IS NOT A READABLE WINDOW.
 *
 * The peek already refused an offset past the mapped SIZE, and that is the
 * bound people think of - but it is not the one that takes the board down.
 * Inside these windows are UNMAPPED HOLES, and touching one is not an error
 * return: it is a synchronous external abort or an async SError, i.e. the box
 * is gone and the operator has learned nothing.  Three are recorded in this
 * tree, each paid for with a crash:
 *   - the L3 special-packet block (SPKTP / SPB) - a write hangs the CPU;
 *   - the L3FE HS_LIGHT tail, RE'd from the wrong chip's tree - a write
 *     async-SErrored on the first flow install (the t=58.5 s panic);
 *   - the per-port MAC block tail - a plain readl faults, which is why the
 *     register dump in cortina-ni-rx.c stops at +0x6c and says "never widen".
 * Only ONE of the three was guarded, only against writes, only for two of the
 * four offsets, and only on the /proc path.
 *
 * So the refusals are DECLARED DATA, applied to every caller, and each one
 * says which access it refuses and WHY - a bare -EPERM from a debug tool is
 * indistinguishable from the tool being broken.
 *
 * @read_faults separates the two classes honestly: for the special-packet and
 * HS_LIGHT holes what is MEASURED is that a WRITE kills the board, so reads
 * stay allowed rather than being refused on a guess; for the per-port tail the
 * measured fault is the READ itself.
 */
static const struct cortina_ni_peek_hole cortina_ni_peek_holes[] = {
	{ CA_NI_WIN_NI, 0x333c, 0x3340, false,
	  "L3 special-packet detect (SPKTP): unmapped on this silicon, a write hangs the CPU" },
	{ CA_NI_WIN_NI, 0x3440, 0x3444, false,
	  "L3 special-packet buffer (SPB): unmapped on this silicon, a write hangs the CPU" },
	{ CA_NI_WIN_NI, 0x3dc4, 0x3dcc, false,
	  "L3FE HS_LIGHT tail: absent on this die (the window ends at ~0x3c8c), a write async-SErrors" },
	/*
	 * The per-port MAC block tail, +0x74..+0x8c of each port's 0x90 stride.
	 * MEASURED on port 0 (0xa634..0xa64c); the other ports are refused by
	 * the stride, which is a deliberate fail-closed extension - the blocks
	 * are identical by construction, and refusing seven words of a debug
	 * peek costs nothing next to aborting the CPU.  Nothing in this driver
	 * reads that range on any port.  Narrow it if it ever hides a real
	 * register, and say which port proved it.
	 */
	{ CA_NI_WIN_NI, CA_NI_PORT_STATIC_CFG(0) + 0x74,
	  CA_NI_PORT_STATIC_CFG(0) + 0x8c, true,
	  "per-port MAC block tail: unmapped, readl faults (synchronous external abort)" },
	{ CA_NI_WIN_NI, CA_NI_PORT_STATIC_CFG(1) + 0x74,
	  CA_NI_PORT_STATIC_CFG(1) + 0x8c, true,
	  "per-port MAC block tail: unmapped, readl faults (synchronous external abort)" },
	{ CA_NI_WIN_NI, CA_NI_PORT_STATIC_CFG(2) + 0x74,
	  CA_NI_PORT_STATIC_CFG(2) + 0x8c, true,
	  "per-port MAC block tail: unmapped, readl faults (synchronous external abort)" },
	{ CA_NI_WIN_NI, CA_NI_PORT_STATIC_CFG(3) + 0x74,
	  CA_NI_PORT_STATIC_CFG(3) + 0x8c, true,
	  "per-port MAC block tail: unmapped, readl faults (synchronous external abort)" },
	{ CA_NI_WIN_NI, CA_NI_PORT_STATIC_CFG(4) + 0x74,
	  CA_NI_PORT_STATIC_CFG(4) + 0x8c, true,
	  "per-port MAC block tail: unmapped, readl faults (synchronous external abort)" },
	{ CA_NI_WIN_NI, CA_NI_PORT_STATIC_CFG(5) + 0x74,
	  CA_NI_PORT_STATIC_CFG(5) + 0x8c, true,
	  "per-port MAC block tail: unmapped, readl faults (synchronous external abort)" },
	{ CA_NI_WIN_NI, CA_NI_PORT_STATIC_CFG(6) + 0x74,
	  CA_NI_PORT_STATIC_CFG(6) + 0x8c, true,
	  "per-port MAC block tail: unmapped, readl faults (synchronous external abort)" },
};
/* The port rows above are written out one per port so each is greppable at its
 * literal offset.  If the port count ever changes, this fails the BUILD rather
 * than silently leaving the new port's hole open. */
static_assert(CA_NI_PORT_COUNT == 7,
	      "add/remove a cortina_ni_peek_holes row per NI port");

bool cortina_ni_peek_access_ok(u8 win, u32 off, bool write, const char **why)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cortina_ni_peek_holes); i++) {
		const struct cortina_ni_peek_hole *h = &cortina_ni_peek_holes[i];

		if (h->win != win || off < h->first || off > h->last)
			continue;
		if (!write && !h->read_faults)
			continue;	/* only the write is known to fault */
		*why = h->why;
		return false;
	}
	*why = NULL;
	return true;
}

void cortina_ni_peek_render(struct seq_file *m, struct cortina_ni *ni)
{
	struct cortina_ni_peek q = ni->peek;	/* snapshot */
	const char *name = "?";
	const char *why;
	void __iomem *base;
	size_t size;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(cortina_ni_peek_wins); i++)
		if (cortina_ni_peek_wins[i].win == q.win)
			name = cortina_ni_peek_wins[i].name;

	base = cortina_ni_peek_base(ni, q.win, &size);
	if (!base) {
		seq_printf(m, "%s: window unavailable\n", name);
		return;
	}
	if (!q.count) {
		seq_puts(m,
			 "usage: echo '[win] <hex_off> [count]' > this file\n"
			 "  win = ni|dma|glb|gphy|wrap|reo|mdio|intr|sgmii|peri (default ni)\n"
			 "  gphy off is raw in the 1M window (port p bank = p*0x40000)\n"
			 "  poke: echo 'poke [win] <hex_off> <hex_val>'\n"
			 "  offsets past the mapped size, and the declared unmapped holes,\n"
			 "  are REFUSED with the reason - see cortina_ni_peek_holes\n");
		return;
	}

	for (i = 0; i < q.count; i++) {
		u32 off = q.off + i * 4;

		if (off + 4 > size) {
			seq_printf(m, "%s+0x%05x = <out of range, size=0x%zx>\n",
				   name, off, size);
			break;
		}
		/* A refusal NAMES ITSELF and the scan CONTINUES: skipping a
		 * hole silently would make a dump look complete when a word of
		 * it was never taken, and aborting would make one bad word cost
		 * the whole range. */
		if (!cortina_ni_peek_access_ok(q.win, off, false, &why)) {
			seq_printf(m, "%s+0x%05x = <REFUSED: %s>\n",
				   name, off, why);
			continue;
		}
		seq_printf(m, "%s+0x%05x = 0x%08x\n", name, off,
			   readl(base + off));
	}
}

/*
 * Parse and apply one peek/poke command.  @buf is modified in place and must
 * be NUL-terminated.  Shared by the /proc node and by debugfs, so the bounds
 * cannot be present on one path and missing on the other.
 */
int cortina_ni_peek_command(struct cortina_ni *ni, char *buf)
{
	u8 win = CA_NI_WIN_NI;
	const char *why;
	char *p, *tok;
	u32 off, count = 1;
	int i;

	p = strim(buf);
	tok = strsep(&p, " \t");
	if (!tok || !*tok)
		return -EINVAL;

	/* 'poke' verb: WRITE a register for fast live RE iteration (a boot is
	 * ~200s; a poke is instant).  usage:
	 *   echo 'poke [win] <hex_off> <hex_val>'
	 * The read-back is armed to the poked reg, so a follow-up cat shows it.
	 * (project rule: dump/spy/poke stays a first-class, always-on feature.) */
	if (!strcmp(tok, "poke")) {
		void __iomem *base;
		size_t size;
		u32 val;

		tok = p ? strsep(&p, " \t") : NULL;
		if (!tok || !*tok)
			return -EINVAL;
		for (i = 0; i < ARRAY_SIZE(cortina_ni_peek_wins); i++)
			if (!strcmp(tok, cortina_ni_peek_wins[i].name)) {
				win = cortina_ni_peek_wins[i].win;
				tok = p ? strsep(&p, " \t") : NULL;
				break;
			}
		if (!tok || !*tok || kstrtou32(tok, 16, &off))
			return -EINVAL;
		tok = p ? strsep(&p, " \t") : NULL;
		if (!tok || !*tok || kstrtou32(tok, 16, &val))
			return -EINVAL;
		if (off & 3)
			return -EINVAL;			/* 32-bit aligned only */
		if (!cortina_ni_peek_access_ok(win, off, true, &why)) {
			pr_warn("%s: peek: refusing poke of window %u +0x%05x: %s\n",
				CA_NI_DRV_NAME, win, off, why);
			return -EPERM;
		}
		base = cortina_ni_peek_base(ni, win, &size);
		if (!base)
			return -ENODEV;
		if (off + 4 > size)
			return -ERANGE;
		writel(val, base + off);
		ni->peek.win = win;
		ni->peek.off = off;
		ni->peek.count = 1;
		return 0;
	}

	/* optional leading window name (non-hex first token) */
	for (i = 0; i < ARRAY_SIZE(cortina_ni_peek_wins); i++)
		if (!strcmp(tok, cortina_ni_peek_wins[i].name)) {
			win = cortina_ni_peek_wins[i].win;
			tok = p ? strsep(&p, " \t") : NULL;
			break;
		}
	if (!tok || !*tok)
		return -EINVAL;

	if (kstrtou32(tok, 16, &off))
		return -EINVAL;
	tok = p ? strsep(&p, " \t") : NULL;
	if (tok && *tok && kstrtou32(tok, 0, &count))
		return -EINVAL;

	if (off & 3)
		return -EINVAL;			/* 32-bit aligned only */
	count = clamp_t(u32, count, 1, CA_NI_PEEK_MAX);

	ni->peek.win = win;
	ni->peek.off = off;
	ni->peek.count = count;
	return 0;
}

/* ------------------------------------------------------------------ */
/* the internal-GPHY uC SRAM reader (ours-vs-stock firmware diff),      */
/* read-only.  Same access the operator used on stock: OCP 0xa436=addr, */
/* read OCP 0xa438=data.  The uC is best-effort held around the dump    */
/* (stock reads fine while held).  debugfs .../cortina-ni/gsram; it was */
/* /proc/cortina_ni_gsram and it has no test consumer, by design - it   */
/* is a bring-up tool for a human, not a measurement source.            */
/* ------------------------------------------------------------------ */
static int cortina_ni_gsram_show(struct seq_file *m, void *v)
{
	struct cortina_ni *ni = m->private;
	struct cortina_ni_gsram q = ni->gsram;	/* snapshot */
	void __iomem *win = ni->win[CA_NI_WIN_GPHY];
	void __iomem *bank;
	u32 i;

	if (!win) {
		seq_puts(m, "gphy window unavailable\n");
		return 0;
	}
	if (!q.count) {
		seq_puts(m,
			 "usage: echo '<bank> <hex_start> <count>' > "
			 "/sys/kernel/debug/cortina-ni/gsram\n"
			 "  bank 0..3, start = SRAM word addr (hex, e.g. 8000), count 1..1024\n");
		return 0;
	}
	if (q.bank >= CA_NI_GPHY_COUNT) {
		seq_printf(m, "bank %u out of range\n", q.bank);
		return 0;
	}

	bank = win + CA_NI_GPHY_BANK(q.bank);

	/* best-effort hold around the whole dump; no ready-poll (read-only) */
	writel(readl(bank + CA_NI_GPHY_LOCK) | CA_NI_GPHY_LOCK_HOLD,
	       bank + CA_NI_GPHY_LOCK);

	for (i = 0; i < q.count; i++) {
		u16 addr = q.start + i;

		cortina_ni_gphy_ocp_write(bank, CA_NI_GPHY_SRAM_ADDR, addr);
		seq_printf(m, "0x%04x=0x%04x\n", addr,
			   cortina_ni_gphy_ocp_read(bank, CA_NI_GPHY_SRAM_DATA));
	}

	writel(readl(bank + CA_NI_GPHY_LOCK) & ~CA_NI_GPHY_LOCK_HOLD,
	       bank + CA_NI_GPHY_LOCK);
	return 0;
}

static int cortina_ni_gsram_open(struct inode *inode, struct file *file)
{
	return single_open(file, cortina_ni_gsram_show, inode->i_private);
}

static ssize_t cortina_ni_gsram_write(struct file *file, const char __user *ubuf,
				      size_t len, loff_t *ppos)
{
	struct cortina_ni *ni = ((struct seq_file *)file->private_data)->private;
	char buf[64], *p, *tok;
	u32 bank, start, count;

	if (len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	p = strim(buf);

	tok = strsep(&p, " \t");
	if (!tok || !*tok || kstrtou32(tok, 0, &bank))
		return -EINVAL;
	tok = p ? strsep(&p, " \t") : NULL;
	if (!tok || !*tok || kstrtou32(tok, 16, &start))
		return -EINVAL;
	tok = p ? strsep(&p, " \t") : NULL;
	if (!tok || !*tok || kstrtou32(tok, 0, &count))
		return -EINVAL;

	if (bank >= CA_NI_GPHY_COUNT || start > 0xffff)
		return -EINVAL;
	count = clamp_t(u32, count, 1, CA_NI_GSRAM_MAX);

	ni->gsram.bank = bank;
	ni->gsram.start = start;
	ni->gsram.count = count;
	return len;
}

static const struct file_operations cortina_ni_dbgfs_gsram_fops = {
	.owner		= THIS_MODULE,
	.open		= cortina_ni_gsram_open,
	.read		= seq_read,
	.write		= cortina_ni_gsram_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* ------------------------------------------------------------------ */
/* debugfs: the peek's supported home, and the ethtool -d decode map    */
/* ------------------------------------------------------------------ */

/*
 * WHY debugfs and not /proc.
 *
 * The counters moved to `ethtool -S` because a test must be able to ask the
 * SAME question of the vendor firmware and of ours, and a /proc node named
 * after this driver can only ever BLOCK on stock.  The peek is the opposite
 * case and it is worth being explicit about: it is an arbitrary-MMIO RE tool
 * with no vendor counterpart by construction, so it is NOT a comparison
 * instrument and no case may derive a stock-vs-ours verdict from it.  What it
 * needs is a home that is unambiguously a debug surface, mountable or not at
 * the integrator's choice, and that is debugfs.
 *
 * ★ RETIRED 2026-08-08.  There are no driver-named /proc nodes left: the peek,
 * the GPHY-SRAM reader and the rx/tx/l3fe narratives are all published here,
 * and every countable VALUE moved to `ethtool -S` / `ethtool -d`, which the
 * VENDOR firmware's kernel serves too - that is what makes a stock-vs-ours
 * comparison possible at all, and it never was through a node of ours.
 */
static int cortina_ni_dbgfs_peek_show(struct seq_file *m, void *v)
{
	cortina_ni_peek_render(m, m->private);
	return 0;
}

static int cortina_ni_dbgfs_peek_open(struct inode *inode, struct file *file)
{
	return single_open(file, cortina_ni_dbgfs_peek_show, inode->i_private);
}

static ssize_t cortina_ni_dbgfs_peek_write(struct file *file,
					   const char __user *ubuf,
					   size_t len, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	char buf[64];
	int ret;

	if (len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	ret = cortina_ni_peek_command(m->private, buf);
	return ret ? ret : len;
}

static const struct file_operations cortina_ni_dbgfs_peek_fops = {
	.owner		= THIS_MODULE,
	.open		= cortina_ni_dbgfs_peek_open,
	.read		= seq_read,
	.write		= cortina_ni_dbgfs_peek_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * The decode key for `ethtool -d`.  That blob is a flat u32 array and no
 * userspace tool knows how to name its words, so the map is published beside
 * it: one line per word, index -> name -> NI-window offset, generated from the
 * same table the dump is taken from.  Without this the register snapshot is
 * only diffable against itself; with it, a word that differs can be named.
 */
static int cortina_ni_dbgfs_regmap_show(struct seq_file *m, void *v)
{
	unsigned int i, n = cortina_ni_regdump_len();

	seq_printf(m, "# ethtool -d words: %u (u32 each, NI window)\n", n);
	seq_puts(m, "# index name offset\n");
	for (i = 0; i < n; i++) {
		const char *name;
		u32 off;

		cortina_ni_regdump_entry(i, &name, &off);
		seq_printf(m, "%u %s 0x%04x\n", i, name, off);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cortina_ni_dbgfs_regmap);

static void cortina_ni_debugfs_release(void *data)
{
	debugfs_remove_recursive(data);
}

/*
 * The narrative dumps live in rx.c / tx.c / flowoffload.c beside the state they
 * print; only their PUBLICATION is here, so there is one place that answers
 * "what hand-debugging surface does this driver expose".
 */
static int cortina_ni_dbgfs_rx_open(struct inode *inode, struct file *file)
{
	return single_open(file, cortina_ni_rx_debug_show, inode->i_private);
}

static const struct file_operations cortina_ni_dbgfs_rx_fops = {
	.owner		= THIS_MODULE,
	.open		= cortina_ni_dbgfs_rx_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int cortina_ni_dbgfs_tx_open(struct inode *inode, struct file *file)
{
	return single_open(file, cortina_ni_tx_debug_show, inode->i_private);
}

static const struct file_operations cortina_ni_dbgfs_tx_fops = {
	.owner		= THIS_MODULE,
	.open		= cortina_ni_dbgfs_tx_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
static int cortina_ni_dbgfs_l3fe_open(struct inode *inode, struct file *file)
{
	return single_open(file, cortina_ni_l3fe_debug_show, inode->i_private);
}

static const struct file_operations cortina_ni_dbgfs_l3fe_fops = {
	.owner		= THIS_MODULE,
	.open		= cortina_ni_dbgfs_l3fe_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* The CONTROL is its own file and 0200 (write-only) on purpose: a bring-up
 * write is not a measurement, and a reader who lands on it by accident must not
 * come away with something that looks like one. */
static const struct file_operations cortina_ni_dbgfs_l3fe_ctl_fops = {
	.owner		= THIS_MODULE,
	.open		= cortina_ni_dbgfs_l3fe_open,
	.write		= cortina_ni_l3fe_debug_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

void cortina_ni_debugfs_init(struct cortina_ni *ni)
{
	struct dentry *d;
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	struct dentry *l;
#endif

	/* A stub when debugfs is off: debugfs_create_dir() then returns an
	 * ERR_PTR that every later call swallows, so nothing here has to be
	 * conditional - but an ERR_PTR must not be handed to devm as a pointer
	 * to free. */
	d = debugfs_create_dir(CA_NI_DRV_NAME, NULL);
	if (IS_ERR_OR_NULL(d))
		return;
	ni->dbgfs = d;

	/* teardown is tied to the device: this driver has no .remove, and a
	 * dentry left behind by a module unload would collide with the next
	 * probe's create */
	if (devm_add_action_or_reset(ni->dev, cortina_ni_debugfs_release, d)) {
		ni->dbgfs = NULL;
		return;
	}

	debugfs_create_file("peek", 0644, d, ni, &cortina_ni_dbgfs_peek_fops);
	debugfs_create_file("regdump_map", 0444, d, ni,
			    &cortina_ni_dbgfs_regmap_fops);
	debugfs_create_file("gsram", 0644, d, ni, &cortina_ni_dbgfs_gsram_fops);
	debugfs_create_file("rx_state", 0444, d, ni, &cortina_ni_dbgfs_rx_fops);
	debugfs_create_file("tx_state", 0444, d, ni, &cortina_ni_dbgfs_tx_fops);

#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	/* The offload engine gets its OWN directory, matching how it is declared
	 * to the test rig: .../cortina-l3fe/{state,control}.  Two files, not one,
	 * because a READ and a bring-up WRITE are different things and merging
	 * them is how a control ends up looking like a measurement source. */
	l = debugfs_create_dir("cortina-l3fe", NULL);
	if (!IS_ERR_OR_NULL(l)) {
		if (devm_add_action_or_reset(ni->dev,
					     cortina_ni_debugfs_release, l))
			return;
		debugfs_create_file("state", 0444, l, ni,
				    &cortina_ni_dbgfs_l3fe_fops);
		debugfs_create_file("control", 0200, l, ni,
				    &cortina_ni_dbgfs_l3fe_ctl_fops);
	}
#endif
}

/* Pulse one NE block reset: assert (set bit) -> 1ms -> deassert (clear bit),
 * stock ca_ni_global_reset order.  Returns the register value read back WHILE
 * asserted so a write that does not stick (e.g. a secure-only register) shows
 * up in the log. */
static u32 cortina_ni_block_reset_pulse(void __iomem *rst, u32 bit)
{
	u32 held;

	writel(readl(rst) | bit, rst);		/* assert  */
	held = readl(rst);			/* read-back while held */
	usleep_range(1000, 2000);		/* stock mdelay(1) */
	writel(readl(rst) & ~bit, rst);		/* deassert */
	return held;
}

/*
 * Bring up the L3QM block.  Stock ca_ni_global_reset pulses the NE block
 * resets (assert -> 1ms -> deassert, per block) so each re-runs its internal
 * init and asserts its *_init_done.  U-Boot brings up only the TX/NI path for
 * TFTP and leaves the QM un-inited, so QM_PHY_PORT_STS.qm_init_done reads 0,
 * the empty-buffer pools never activate and the CPU-push FIFO never drains.
 * A TQM-only pulse was NOT enough; nor was L2FE+L2TM+L3FE+TQM (the reset fired
 * - held bits read back correctly - but qm_init_done stayed 0).  QM_PHY_PORT_STS
 * is dominated by NI<->QM handshake bits (nirx_qm_rdy, qm_nitx_rdy, nitx_qm_vld,
 * te_qm_es_ni_ok), so the QM only completes init once the NI is reset alongside
 * it.  Match stock ca_ni_global_reset EXACTLY: pulse NI+L2FE+L2TM+L3FE+TQM in
 * that order (SDRAM is skipped - it is DRAM).  Resetting NI drops U-Boot's
 * FE-bypass direct-TX, but cortina_ni_tx_hw_init() (run right after, in
 * tx_probe) fully rebuilds the NI/TX path - same reset->full-init order stock's
 * ca_init uses.  Direct MMIO via the GLB window (same as the dphy_rst release).
 */
static void __maybe_unused cortina_ni_qm_reset(struct cortina_ni *ni)
{
	void __iomem *rst;
	u32 h_ni, h_tqm;

	if (!ni->win[CA_NI_WIN_GLB])
		return;
	rst = ni->win[CA_NI_WIN_GLB] + CA_NI_GLB_BIST_CONTROL4;

	/* ★ Reset ONLY NI + TQM, NOT L2FE/L2TM/L3FE.  Resetting the forwarding
	 * blocks and NOT re-initialising them (aal_l2fe_init/aal_arb_init/
	 * aal_l2_tm_init - a huge, table-heavy init we do not port) leaves them
	 * reset-but-dead: their indirect-access engines hang (REDIR_LDPID/FIB
	 * writes never complete, GO stuck) and they make no forwarding decision,
	 * so LAN ingress never reaches the CPU.  The boot ROM / U-Boot already
	 * brings L2FE/L2TM/L3FE up (U-Boot RX works), so leave them running and
	 * only reset the two blocks we DO fully reconfigure: NI (rebuilt by
	 * cortina_ni_tx_hw_init) and the TQM/QM (our EQ/EPP/ring config). */
	h_ni  = cortina_ni_block_reset_pulse(rst, CA_NI_GLB_RST_NI);
	h_tqm = cortina_ni_block_reset_pulse(rst, CA_NI_GLB_RST_TQM);
	usleep_range(100000, 110000);		/* stock trailing mdelay(100) */

	dev_info(ni->dev,
		 "NE reset pulsed (held ni=0x%08x tqm=0x%08x; L2FE/L2TM/L3FE left at boot init; now=0x%08x)\n",
		 h_ni, h_tqm, readl(rst));
}

static int cortina_ni_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cortina_ni *ni;
	int ret;

	ni = devm_kzalloc(dev, sizeof(*ni), GFP_KERNEL);
	if (!ni)
		return -ENOMEM;
	ni->dev = dev;
	/* guards the ONE reader of the read-and-clear NI_HV counters; must be
	 * live before anything can sample them (the rx/l3fe /proc nodes and
	 * `ethtool -S` all go through it) */
	spin_lock_init(&ni->nihv_lock);
	platform_set_drvdata(pdev, ni);

	/*
	 * Front-panel per-RJ45 link lamps: publish the per-port LED triggers
	 * before any of the bring-up below.  Early on purpose - an LED that is
	 * already registered binds immediately, and no early `return ret` on the
	 * way down can skip it.  Software only: it touches no hardware, returns
	 * void, and cannot fail the probe (cortina-ni-leds.c).
	 */
	cortina_ni_leds_probe(ni);

	ret = cortina_ni_map_windows(ni);
	if (ret)
		return ret;

	/*
	 * ★ Release the internal-GPHY <-> NI datapath sub-blocks from reset
	 * FIRST, before any GPHY/MAC bring-up (release-then-init order).
	 * U-Boot leaves GLOBAL_DPHY_RESET (GLB+0xa0) with datapath sub-blocks
	 * held in reset (0x50302340); stock releases them to 0x10000000.  With
	 * them held, NO frame crosses the internal GMII on any port even though
	 * the GPHY line side links - the bidirectional-dead gate.  A late
	 * release (at link-up) sticks but does not re-init the sub-block, so we
	 * must do it here at probe entry, ahead of MDIO/PHY/MAC init.
	 */
	if (ni->win[CA_NI_WIN_GLB]) {
		void __iomem *glb = ni->win[CA_NI_WIN_GLB];
		/* ★★ build64: the EXACT stock ca_ni_global_reset sequence (vendor source): the NE
		 * core resets done SEQUENTIAL, ONE-AT-A-TIME, IN ORDER - ni, l2fe, l2tm, l3fe, tqm -
		 * each a 1ms assert->deassert; tqm(bit5) LAST, after its deps ni/l2fe/l2tm/l3fe are
		 * back up.  SKIP sdram(bit4, resetting it kills DRAM) and ptp.  cortina,rst-mgr @
		 * glb+0xa0 is a plain bit set/clear (no clock/delay in the reg), so a direct poke is
		 * faithful; the ORDER + one-at-a-time + delays are what matter (build60's combined
		 * ni+tqm booted but the engine stayed dead - wrong SHAPE).  This runs right after the
		 * glb window is mapped, before any NI-core access.  First release the U-Boot-held DPHY
		 * datapath sub-blocks (0x10000000 resting = the original bidirectional-dead fix), then
		 * run the sequence (read fresh each iter to preserve the other bits). */
		static const int order[] = { 0, 1, 2, 3, 5 };	/* ni,l2fe,l2tm,l3fe,tqm (SKIP 4=sdram) */
		u32 was = readl(glb + CA_NI_GLB_BLOCK_RESET);
		int j;

		writel(CA_NI_GLB_BLOCK_RESET_VAL, glb + CA_NI_GLB_BLOCK_RESET);
		for (j = 0; j < ARRAY_SIZE(order); j++) {
			u32 v = readl(glb + CA_NI_GLB_BLOCK_RESET);

			writel(v | BIT(order[j]), glb + CA_NI_GLB_BLOCK_RESET);	/* ASSERT one */
			mdelay(1);
			writel(v & ~BIT(order[j]), glb + CA_NI_GLB_BLOCK_RESET);	/* DEASSERT it */
		}
		mdelay(100);					/* final settle (stock) */
		dev_info(dev, "ne-reset: sequential ni,l2fe,l2tm,l3fe,tqm at probe start (0x%08x -> 0x%08x)\n",
			 was, readl(glb + CA_NI_GLB_BLOCK_RESET));
	}

	/* ★ NE block reset DISABLED (storm bisect test): our L2FE showed a
	 * ~100k/s internal packet storm (sop/eop diverging = looping frames) with
	 * wptr=0.  qm_init_done proved a phantom, so the block reset never had a
	 * real justification - and re-resetting NI/TQM after the boot ROM already
	 * initialised the whole NE datapath can leave the L2FE->TM->QM handoff in
	 * a self-feeding loop.  Run on the pure boot-ROM datapath + only our
	 * QM/EQ/CPU-EPP + forwarding config on top, and see if the storm clears.
	 * (Re-enable cortina_ni_qm_reset(ni) here to A/B the reset.) */

	/*
	 * NOTE: the internal-GPHY firmware patch + uC resume is NOT done here -
	 * at probe the uC is still running (HOLD == 0) and its SRAM is not
	 * writable.  It is deferred to link-up (cortina_ni_rx_link_up ->
	 * cortina_ni_gphy_patch_and_resume), where the uC has entered the held
	 * state after MDIO/GPHY init.
	 */

	cortina_ni_log_reserved_mem(ni);

	ret = cortina_ni_mdio_init(ni);
	if (ret)
		return ret;

	cortina_ni_scan_phys(ni);

	ret = cortina_ni_tx_probe(ni);
	if (ret)
		return ret;

	ret = cortina_ni_rx_probe(ni);
	if (ret)
		return ret;

	/* L3FE flow-engine arm + verify (nf_flow_table HW offload, phase 1).
	 * Non-fatal: on any error the offload stays disabled and the normal
	 * datapath is untouched. */
	ret = cortina_ni_flowoffload_probe(ni);
	if (ret)
		dev_warn(dev, "L3FE flow offload disabled (%d)\n", ret);

	/* ★ EVERY hand-debugging dump this driver has, in ONE generic place.
	 * There are no driver-named /proc nodes any more: what a TEST reads is
	 * `ethtool -S` / `-d` (which stock's kernel serves too, so a comparison
	 * exists at all), and what a HUMAN reads is here.  Runs last in probe so
	 * rx/tx/l3fe are already up and every dump has something to show. */
	cortina_ni_debugfs_init(ni);

	dev_info(dev, "M2c probe complete\n");
	return 0;
}

static const struct of_device_id cortina_ni_of_match[] = {
	{ .compatible = "cortina,ni-interface" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_ni_of_match);

static struct platform_driver cortina_ni_driver = {
	.probe = cortina_ni_probe,
	.driver = {
		.name = CA_NI_DRV_NAME,
		.of_match_table = cortina_ni_of_match,
	},
};

/* ------------------------------------------------------------------ */
/* internal quad-GPHY phy_driver: inherit U-Boot's link, NEVER re-aneg  */
/* ------------------------------------------------------------------ */

/*
 * ★ THE determinism gate.  U-Boot brings the internal GPHY up to a stable
 * 1G/full link (it TFTPs our kernel over it) and hands it to Linux working
 * bidirectionally.  Generic phylib, at phy_start, RESTARTS auto-negotiation
 * (genphy_config_aneg -> BMCR restart), which bounces the GPHY LINE link
 * Up->Down->Down->Up over ~5s.  The internal GPHY<->NI-MAC datapath dies
 * across that bounce and never re-establishes - RX+TX both dead, and
 * non-deterministic ("works the boots the bounce timing spares it").  Stock
 * never restarts aneg: it inherits U-Boot's link and only MONITORS it.
 *
 * A no-op config_aneg makes phylib do exactly that: phy_start still runs the
 * state machine and calls read_status (so adjust_link fires once and we adopt
 * the live link + set up the RX datapath), but it writes NO BMCR - so the
 * line link is never bounced and the U-Boot datapath survives into Linux.
 * No .soft_reset / .config_init either (phy_init_hw then never resets the
 * PHY, preserving its U-Boot patch/calibration).  Features fall through to
 * genphy_read_abilities automatically.
 */
static int cortina_ni_gphy_config_aneg(struct phy_device *phydev)
{
	return 0;	/* never touch aneg - keep U-Boot's stable link */
}

static struct phy_driver cortina_ni_gphy_driver[] = { {
	.phy_id		= CA_NI_GPHY_PHY_ID,
	.phy_id_mask	= CA_NI_GPHY_PHY_ID_MASK,
	.name		= "Cortina RTL960x internal GPHY",
	.config_aneg	= cortina_ni_gphy_config_aneg,
	.read_status	= genphy_read_status,
} };

static int __init cortina_ni_module_init(void)
{
	int ret;

	/* register the GPHY driver FIRST so it binds the internal PHYs at MDIO
	 * scan time (during the platform probe), overriding genphy + its aneg */
	ret = phy_drivers_register(cortina_ni_gphy_driver,
				   ARRAY_SIZE(cortina_ni_gphy_driver),
				   THIS_MODULE);
	if (ret)
		return ret;

	ret = platform_driver_register(&cortina_ni_driver);
	if (ret)
		phy_drivers_unregister(cortina_ni_gphy_driver,
				       ARRAY_SIZE(cortina_ni_gphy_driver));
	return ret;
}
module_init(cortina_ni_module_init);

static void __exit cortina_ni_module_exit(void)
{
	platform_driver_unregister(&cortina_ni_driver);
	cortina_ni_flowoffload_exit();
	phy_drivers_unregister(cortina_ni_gphy_driver,
			       ARRAY_SIZE(cortina_ni_gphy_driver));
}
module_exit(cortina_ni_module_exit);

MODULE_DESCRIPTION("Cortina-Access NI Ethernet driver (RTL9607F)");
MODULE_LICENSE("GPL");
