// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: FAMILY */
/*
 * Asynchronous kernel console for the Luna 8250 UART: the 8250 access of the
 * common console (onu_console.h). At late_initcall it takes over the 8250
 * console of the same port.
 */
#include <linux/console.h>
#include <linux/init.h>
#include <linux/onu_console.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/string.h>

#include "8250.h"

static bool luna_con_fifo_on(struct uart_8250_port *up)
{
	struct uart_port *port = &up->port;

	return (up->capabilities & UART_CAP_FIFO) && up->tx_loadsz > 1 &&
	       (up->fcr & UART_FCR_ENABLE_FIFO) && port->state &&
	       test_bit(TTY_PORT_INITIALIZED, &port->state->port.iflags);
}

static void luna_con_out(struct uart_port *port, unsigned char c)
{
	serial_port_out(port, UART_TX, c);
}

static bool luna_con_fifo_empty(struct uart_port *port)
{
	return serial_lsr_in(up_to_u8250p(port)) & UART_LSR_THRE;
}

static bool luna_con_tx_busy(struct uart_port *port)
{
	return up_to_u8250p(port)->ier & UART_IER_THRI;
}

static unsigned int luna_con_fifo_room(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	return luna_con_fifo_on(up) ? up->tx_loadsz : 1;
}

static unsigned int luna_con_fifo_depth(struct uart_port *port)
{
	return up_to_u8250p(port)->tx_loadsz;
}

static const struct onu_con_uart luna_con_uart = {
	.out		= luna_con_out,
	.fifo_empty	= luna_con_fifo_empty,
	.tx_busy	= luna_con_tx_busy,
	.fifo_room	= luna_con_fifo_room,
	.fifo_depth	= luna_con_fifo_depth,
};

static struct onu_console luna_con = {
	.uart		= &luna_con_uart,
	.name		= "luna-console",
	.fallback	= "printk stays synchronous",
	.con.name	= "ttyS",
};

static int __init luna_con_init(void)
{
	struct console *c, *vendor = NULL;

	console_list_lock();
	for_each_console(c) {
		if (!strcmp(c->name, "ttyS") && (c->flags & CON_ENABLED) &&
		    !(c->flags & (CON_BOOT | CON_NBCON))) {
			vendor = c;
			break;
		}
	}
	console_list_unlock();
	if (!vendor) {
		pr_err("luna-console: no enabled ttyS console to make asynchronous -- printk stays synchronous\n");
		return -ENODEV;
	}
	return onu_console_start(&luna_con, vendor, &serial8250_get_port(vendor->index)->port);
}
late_initcall(luna_con_init);
