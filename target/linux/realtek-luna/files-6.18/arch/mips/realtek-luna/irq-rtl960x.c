// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL960x "Luna" SoC interrupt controller (the "GISR" aggregator).
 *
 * ONE driver, two chips, a table for what differs.  It replaces
 * irq-rtl9602c.c and irq-rtl9603cvd.c, which were 96-98%% identical: the same
 * struct, the same mask/unmask/map/dispatch/of_init, the same offsets, and two
 * spellings of every name (`LUNA_INTC_GIMR` and `GISR_GIMR`, same value).  What
 * actually differed was three things, and all three are now DATA: the routing
 * table, whether a second CPU block must be masked, and nothing else.
 *
 * ★ THE MERGE WAS DEFERRED ON PURPOSE, AND THE DEFERRAL'S OWN TERMS ARE MET.
 * irq-rtl9603cvd.c said: *"kept apart deliberately for now -- that driver
 * carries two routing overrides established empirically on the X111W, which are
 * facts about THAT board's cascade and must not be inherited here.  Merge them
 * into one rtl960x_* driver once this chip has booted and both routings are
 * proven."*  Both halves were checked before merging rather than assumed:
 *   - the per-board routing is NOT inherited: each chip keeps its own `irr[]`,
 *     which is the whole point of the table;
 *   - the RTL9602C's routing is proven by the X111W booting on it;
 *   - the RTL9603CVD's is proven by the G24W's console log of 2026-08-23,
 *     which shows `Linux version 6.18.31` and an INTERACTIVE `root@OpenWrt:~#`
 *     shell at uptime 1:50 answering commands.  That is this file's own stated
 *     test: without this aggregator delivering the UART interrupt, "early
 *     printk would work and the console would never become interactive".
 *
 * Clean-room: written from observed hardware facts only -- register bases and
 * offsets, bit semantics, the routing scheme and the vendor's own init values.
 * No proprietary source text or structure was copied.
 *
 * ---------------------------------------------------------------------
 * WHY THIS FAMILY DOES NOT USE A MIPS GIC
 *
 * The RTL9603CVD has NO MIPS GIC and no Coherence Manager.  Its own vendor
 * kernel says so on this exact board (tier 1, LANLY G24W boot console, captured
 * 2026-08-18):
 *
 *	MIPS CPS SMP unable to proceed without a CM
 *	GIC isn't present!
 *	rtl9603cvd_gisr_init: gisr_translate_d2c = ...
 *
 * so a "mti,gic" node here describes silicon that is not present: the UART
 * interrupt would never arrive, early printk would work and the console would
 * never become interactive.  Describe this aggregator instead.
 * ---------------------------------------------------------------------
 *
 * REGISTER MAP (one block PER CPU where the chip has more than one; stride is
 * a table field, 0 where there is only one block to arrange):
 *
 *	GIMR0	+0x00	mask,   inputs  0..31  (1 = enabled)
 *	GIMR1	+0x04	mask,   inputs 32..63
 *	GISR0	+0x08	status, inputs  0..31  (raw; AND with mask for pending)
 *	GISR1	+0x0c	status, inputs 32..63
 *	IRR0..6	+0x10..	routing, 4 bits per input, 8 inputs per word
 *
 * GIMR0 bit 12 is not an input: it is the master peripheral-IRQ enable and must
 * be set for any peripheral interrupt to be delivered at all.  The dispatch
 * below therefore excludes it from the pending scan.
 *
 * Realtek numbers the routing nibbles in inverted word order: input 0 lives in
 * the *last* IRR word's lowest nibble.
 *
 * ROUTING NIBBLE -> CP0 LINE.  A nibble of 0 disconnects the input; a value V
 * selects aggregator output line V-1, and output line 0 is wired to CP0 HW IRQ
 * 2, so the delivered line is CP0 HW IRQ (V + 1).  Cross-checked two ways, and
 * the two agree on every case:
 *
 *   - the vendor's own dispatchers read GISR word 0 from CP0 IP3 and GISR word
 *     1 from CP0 IP4, and take the SoC timer on IP7;
 *   - the vendor's own routing values (the tables below) give nibble 2 to every
 *     input in word 0 that it uses (switch core, GMAC0, PON NIC), nibble 3 to
 *     every input in word 1 that it uses (UART0, GPIO bank ABCD), and nibble 6
 *     to the per-CPU system timer.
 *
 *   2 -> CP0 IRQ 3	3 -> CP0 IRQ 4		6 -> CP0 IRQ 7
 *
 * The strongest single check is the per-CPU one: the vendor loads a DIFFERENT
 * word 4 on each CPU, and the only nibbles that differ are the two system-timer
 * inputs -- TC0 enabled on CPU0 and disconnected on CPU1, TC1 the other way
 * round.  That is exactly what a per-CPU clockevent needs and nothing else in
 * the map would explain it.
 *
 * INPUT NUMBERING IS THE NATIVE GISR BIT, and it is SHARED BY BOTH CHIPS --
 * which is the strongest evidence that this really is one block: the RTL9602C's
 * device tree in this same target independently uses 49 for UART0 and 43 for
 * the system timer, the same numbers the RTL9603CVD's vendor kernel uses.
 * Named inputs (a reserved input is omitted):
 *
 *	 4 ECC		 5 NAND		 8 switch core	14 USB host p2
 *	16 PCIe		17 PCM0		18 PCM1		24 PON NIC
 *	25 PON NIC DS	26 GMAC0 int0	27 GMAC0 int1	28 VoIP XSI
 *	29 VoIP SPI	31 LX bus debug	32 GPIO JKMN	39 WDT phase-1
 *	40 WDT phase-2	41 GPIO EFGH	42 GPIO ABCD	43..48 TC0..TC5
 *	49..52 UART0..3	60..62 TC6..TC8
 *
 * DELIVERY INVARIANT: every GISR interrupt is delivered to CPU 0.  Only CPU 0's
 * block carries routing and mask bits; a second CPU's block, where the chip has
 * one, is masked shut, so a secondary CPU can never take an interrupt this
 * driver did not arrange.  Per-CPU affinity is OWED work and needs a running
 * SMP kernel to verify, which this family cannot have until the CPS/CM question
 * is settled.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define RTL960X_INTC_INPUTS	64
#define RTL960X_INTC_WORDS	(RTL960X_INTC_INPUTS / 32)	/* mask/status */
#define RTL960X_INTC_GIMR(w)	(0x00 + (w) * 4)
#define RTL960X_INTC_GISR(w)	(0x08 + (w) * 4)
#define RTL960X_INTC_IRR(i)	(0x10 + (i) * 4)
/*
 * ⚠ SEVEN, AND IT IS COUNTED FROM THE HARDWARE, NOT DERIVED FROM THE INPUTS.
 * The routing is 4 bits per input, 8 inputs per word -- 64/8 = 8 words by that
 * arithmetic -- yet BOTH chips' vendor init writes SEVEN.  irq-rtl9602c.c
 * carried the derived 8 in a #define and then wrote 7 words explicitly, so the
 * constant and the code disagreed and only the code was right.  The table below
 * is sized by what the hardware is actually given.
 */
#define RTL960X_INTC_IRR_WORDS	7
#define RTL960X_INTC_PERIPH_EN	12	/* GIMR0 bit12 = master peripheral enable */

/*
 * ★★ THE PER-CHIP TABLE -- the only thing that differs between these two SoCs.
 * A third Luna chip is a table entry, never a fork of the logic.
 */
struct rtl960x_intc_chip {
	const char	*name;
	/* The vendor's own routing values for CPU 0, word 0 first. */
	u32		irr[RTL960X_INTC_IRR_WORDS];
	/* Byte distance to the NEXT CPU's block, or 0 where this chip has only
	 * one block for this driver to arrange. */
	u32		cpu_stride;
};

static const struct rtl960x_intc_chip rtl9602c_intc_chip = {
	.name	= "RTL9602C",
	/*
	 * ⚠ TWO WORDS DIFFER FROM THE SIBLING'S, AND THEY ARE THIS BOARD'S OWN.
	 * They were established EMPIRICALLY on the X111W and describe THAT
	 * board's cascade: word 2 ends 0x...226 where the sibling has 0x...222,
	 * and word 4 begins 0x6 where the sibling begins 0x3.  Keeping them in
	 * the table is exactly why the two drivers could finally become one --
	 * a shared routing constant would have silently inherited one board's
	 * cascade onto the other, which is what the deferral warned about.
	 */
	.irr	= { 0x03333330, 0x30302222, 0x00020226, 0x22020333,
		    0x63333063, 0x32322022, 0x00333000 },
	.cpu_stride = 0,	/* this driver arranges one block here */
};

static const struct rtl960x_intc_chip rtl9603cvd_intc_chip = {
	.name	= "RTL9603CVD",
	.irr	= { 0x03333330, 0x30302222, 0x00020222, 0x22020333,
		    0x33333063, 0x32322022, 0x00333000 },
	.cpu_stride = 0x40,	/* CPU 1's block, masked shut below */
};

struct rtl960x_intc {
	void __iomem			*base;	/* CPU 0's block */
	raw_spinlock_t			lock;
	struct irq_domain		*domain;
	const struct rtl960x_intc_chip	*c;
};

static void rtl960x_intc_mask(struct irq_data *d)
{
	struct rtl960x_intc *ic = irq_data_get_irq_chip_data(d);
	unsigned int word = d->hwirq / 32;
	u32 val;

	raw_spin_lock(&ic->lock);
	val = readl(ic->base + RTL960X_INTC_GIMR(word));
	val &= ~BIT(d->hwirq % 32);
	writel(val, ic->base + RTL960X_INTC_GIMR(word));
	raw_spin_unlock(&ic->lock);
}

static void rtl960x_intc_unmask(struct irq_data *d)
{
	struct rtl960x_intc *ic = irq_data_get_irq_chip_data(d);
	unsigned int word = d->hwirq / 32;
	u32 val;

	raw_spin_lock(&ic->lock);
	val = readl(ic->base + RTL960X_INTC_GIMR(word));
	val |= BIT(d->hwirq % 32);
	writel(val, ic->base + RTL960X_INTC_GIMR(word));
	raw_spin_unlock(&ic->lock);
}

static struct irq_chip rtl960x_intc_irqchip = {
	.name		= "rtl960x-intc",
	.irq_mask	= rtl960x_intc_mask,
	.irq_unmask	= rtl960x_intc_unmask,
};

static int rtl960x_intc_map(struct irq_domain *d, unsigned int irq,
			    irq_hw_number_t hw)
{
	struct rtl960x_intc *ic = d->host_data;

	irq_set_chip_and_handler(irq, &rtl960x_intc_irqchip, handle_level_irq);
	irq_set_chip_data(irq, ic);
	/* routing (GIRR) is pre-loaded with the stock-observed values in init */

	return 0;
}

static const struct irq_domain_ops rtl960x_intc_domain_ops = {
	.map	= rtl960x_intc_map,
	.xlate	= irq_domain_xlate_onecell,
};

static void rtl960x_intc_dispatch(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct rtl960x_intc *ic = irq_desc_get_handler_data(desc);
	int word;

	chained_irq_enter(chip, desc);
	for (word = 0; word < RTL960X_INTC_WORDS; word++) {
		unsigned long pending = readl(ic->base + RTL960X_INTC_GISR(word)) &
					readl(ic->base + RTL960X_INTC_GIMR(word));
		unsigned int bit;

		if (word == 0)
			pending &= ~BIT(RTL960X_INTC_PERIPH_EN);  /* aggregate, not an input */
		for_each_set_bit(bit, &pending, 32)
			generic_handle_domain_irq(ic->domain, word * 32 + bit);
	}
	chained_irq_exit(chip, desc);
}

static int __init rtl960x_intc_init(struct device_node *node,
				    const struct rtl960x_intc_chip *chip)
{
	struct rtl960x_intc *ic;
	int parent_irq, n = 0, i, ret;

	ic = kzalloc(sizeof(*ic), GFP_KERNEL);
	if (!ic)
		return -ENOMEM;
	raw_spin_lock_init(&ic->lock);
	ic->c = chip;
	ic->base = of_iomap(node, 0);
	if (!ic->base) {
		ret = -ENXIO;
		goto err_free;
	}

	writel(BIT(RTL960X_INTC_PERIPH_EN), ic->base + RTL960X_INTC_GIMR(0));
	writel(0, ic->base + RTL960X_INTC_GIMR(1));
	for (i = 0; i < RTL960X_INTC_IRR_WORDS; i++)
		writel(chip->irr[i], ic->base + RTL960X_INTC_IRR(i));

	/* Mask a second CPU's block shut where the chip has one: only CPU 0
	 * carries routing, so a secondary CPU can never take an interrupt this
	 * driver did not arrange. */
	if (chip->cpu_stride) {
		void __iomem *other = ic->base + chip->cpu_stride;

		writel(0, other + RTL960X_INTC_GIMR(0));
		writel(0, other + RTL960X_INTC_GIMR(1));
	}

	ic->domain = irq_domain_create_linear(of_fwnode_handle(node),
					      RTL960X_INTC_INPUTS,
					      &rtl960x_intc_domain_ops, ic);
	if (!ic->domain) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	for (n = 0; (parent_irq = irq_of_parse_and_map(node, n)) > 0; n++)
		irq_set_chained_handler_and_data(parent_irq,
						 rtl960x_intc_dispatch, ic);
	if (!n) {
		ret = -ENODEV;
		goto err_unmap;
	}

	pr_info("rtl960x-intc: %s aggregator, %d inputs, %d parent line(s)\n",
		chip->name, RTL960X_INTC_INPUTS, n);
	return 0;

err_unmap:
	iounmap(ic->base);
err_free:
	kfree(ic);
	return ret;
}

/* IRQCHIP_DECLARE wants a fixed signature, so each compatible gets a two-line
 * entry point that names its own table.  This is the ONLY per-chip code left. */
static int __init rtl9602c_intc_of_init(struct device_node *node,
					struct device_node *parent)
{
	return rtl960x_intc_init(node, &rtl9602c_intc_chip);
}

static int __init rtl9603cvd_intc_of_init(struct device_node *node,
					  struct device_node *parent)
{
	return rtl960x_intc_init(node, &rtl9603cvd_intc_chip);
}

IRQCHIP_DECLARE(rtl9602c_intc, "realtek,rtl9602c-intc", rtl9602c_intc_of_init);
IRQCHIP_DECLARE(rtl9603cvd_intc, "realtek,rtl9603cvd-intc",
		rtl9603cvd_intc_of_init);
