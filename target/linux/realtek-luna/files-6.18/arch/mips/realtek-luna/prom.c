// SPDX-License-Identifier: GPL-2.0-only
/* TIER: FAMILY */
/*
 * Realtek "Luna" GPON ONU (RTL960xC, RLX/Taroko core) — early boot / PROM.
 *
 * Independent implementation from the documented MIPS boot contract and the
 * SoC's bootloader handoff: the bootloader enters Linux with a legacy
 * argc/argv pair in fw_arg0/fw_arg1.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/prom.h>

#include "luna.h"

/* 16550 UART0 (KSEG1), reg-shift 2: THR @ +0x00, LSR @ +0x14, LSR.THRE = 0x20. */
#define LUNA_UART0	((void __iomem *)CKSEG1ADDR(0x18002000))
#define UART_THR	0x00
#define UART_LSR	(5 << 2)
#define UART_LSR_THRE	0x20

const char *get_system_type(void)
{
	return "Realtek RTL960xC (Luna/Taroko)";
}

void prom_putchar(char c)
{
	while (!(__raw_readb(LUNA_UART0 + UART_LSR) & UART_LSR_THRE))
		;
	__raw_writeb(c, LUNA_UART0 + UART_THR);
}

/* Bring-up bisect marker `[c]`, the earliest reliable output (CONFIG_LUNA_DEBUG). */
void luna_mark(char c)
{
	if (!IS_ENABLED(CONFIG_LUNA_DEBUG))
		return;
	prom_putchar('[');
	prom_putchar(c);
	prom_putchar(']');
}

/* The bootloader enters Linux with a legacy MIPS argument ... -- dev/MEASURED-prom.c.md sec 1. */
static void __init prom_init_cmdline(void)
{
	int argc = (int)fw_arg0;
	char **argv = (char **)CKSEG1ADDR(fw_arg1);
	int i;

	if (!argc || !argv)
		return;

	for (i = 0; i < argc; i++) {
		char *arg = (char *)CKSEG1ADDR((unsigned long)argv[i]);

		if (!arg || !*arg)
			continue;
		if (arcs_cmdline[0])
			strlcat(arcs_cmdline, " ", sizeof(arcs_cmdline));
		strlcat(arcs_cmdline, arg, sizeof(arcs_cmdline));
	}
}

#ifdef CONFIG_CPU_R3K_TLB
/* As the stock kernel's entry: the RLX on-chip instruction and data memories off
 * (cache-control registers $20 select 0 and 1: 0x20, then 0x800). */
#define LUNA_CCTL(sel, v)	"mtc0 $0, $20, " #sel "\n nop\n nop\n li $8, " #v "\n" \
				"mtc0 $8, $20, " #sel "\n nop\n nop\n"
static void __init luna_onchip_mem_off(void)
{
	__asm__ __volatile__(".set push\n .set noreorder\n .set mips32\n"
			     LUNA_CCTL(0, 0x20) LUNA_CCTL(0, 0x800) LUNA_CCTL(1, 0x20) LUNA_CCTL(1, 0x800)
			     "mtc0 $0, $20\n nop\n nop\n .set pop\n" : : : "$8", "memory");
}
#else
static inline void luna_onchip_mem_off(void) { }
#endif

void __init prom_init(void)
{
	int i;

	luna_mark('P');
	luna_onchip_mem_off();

	/* Clear the CPU's unmapped-memory-segment (UMSAR0..3 @ ...
	 * dev/MEASURED-prom.c.md sec 2. */
	for (i = 0; i < 4; i++) {
		*(volatile u32 *)(0xb8001300ul + i * 0x10) = 0;
		*(volatile u32 *)(0xb8004000ul + i * 0x10) = 0;
	}

	prom_init_cmdline();
}

void __init prom_free_prom_memory(void)
{
}
