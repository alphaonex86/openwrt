// SPDX-License-Identifier: GPL-2.0-only
/* TIER: FAMILY */
/*
 * The GMAC as the boot loader hands it over. Its RX engine may still own ring slots whose buffers lie in
 * memory the kernel is about to give its page allocator: until ndo_open resets the engine, a received frame
 * would be written into a page someone else owns. Stock stops and resets the engine at probe; this runs the
 * same stop from device_tree_init(), while memblock still owns all of memory, and says what it found.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/memblock.h>
#include <linux/printk.h>
#include <linux/stddef.h>
#include <asm/addrspace.h>

#include "luna_eth_regs.h"
#include "eth_inherited_ring.h"
#include "../../../../arch/mips/realtek-luna/luna.h"

#define LUNA_GMAC0	((void __iomem *)CKSEG1ADDR(0x18012000))	/* ethernet@18012000 on every Luna die */
#define LUNA_BUS_PHYS	0x1fffffffu	/* the boot loader ORs a bus window into its DMA addresses */

void __init luna_gmac_quiesce_inherited(void)
{
	struct eth_inherited_ring r = { 0 };
	u32 io_cmd, io_cmd1, ring;
	unsigned int n;

	if (!(readl(SOC_IP_SEL) & IPSEL_GMAC0)) {
		pr_info("luna-gmac: boot loader left the engine clock-gated: no DMA to stop\n");
		return;
	}
	io_cmd = ioread32(LUNA_GMAC0 + R_IO_CMD);
	io_cmd1 = ioread32(LUNA_GMAC0 + R_IO_CMD1);
	luna_eth_hw_stop(LUNA_GMAC0);
	if (ioread32(LUNA_GMAC0 + R_IO_CMD) || ioread32(LUNA_GMAC0 + R_IO_CMD1))
		pr_err("luna-gmac: the inherited DMA did not stop: IO_CMD %08x IO_CMD1 %08x\n",
		       ioread32(LUNA_GMAC0 + R_IO_CMD), ioread32(LUNA_GMAC0 + R_IO_CMD1));
	ring = ioread32(LUNA_GMAC0 + R_RxFDP) & LUNA_BUS_PHYS;
	n = luna_gmac_rxdesnum_entries(ioread32(LUNA_GMAC0 + R_RxDesNum));
	if (!(ring & 15) && memblock_is_region_memory(ring, n * sizeof(struct rx_desc)))
		eth_inherited_ring_walk(&r, (const u8 *)CKSEG1ADDR(ring), n, sizeof(struct rx_desc),
					offsetof(struct rx_desc, opts1), offsetof(struct rx_desc, addr),
					D_OWN, D_EOR, LUNA_BUS_PHYS, RX_BUF_SIZE);
	pr_info("luna-gmac: boot loader left IO_CMD %08x IO_CMD1 %08x, RX ring %08x%s: %u slots, %u armed (buffers %08x..%08x), %u filled after its last poll; DMA stopped\n",
		io_cmd, io_cmd1, ring, r.slots ? "" : " (not memory: not read)", r.slots, r.armed, r.buf_lo,
		r.buf_hi, r.filled);
}
