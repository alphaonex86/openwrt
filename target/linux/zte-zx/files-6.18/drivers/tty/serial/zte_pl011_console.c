// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: FAMILY */
/*
 * Asynchronous kernel console for the ZTE PL011 UART: the PL011 access of the
 * common console (onu_console.h). The PL011 console is nbcon, so a kernel line
 * landed inside a shell reply (kernel_console_lines_whole: 8/40 kernel lines
 * whole on the F680, 2026-11-14). At late_initcall this takes it over.
 */
#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/onu_console.h>
#include <linux/serial_core.h>
#include <linux/string.h>

#define ZTE_UART_DR	0x04	/* data register, both ZTE PL011 layouts */

static struct onu_console zte_con;

static void zte_con_out(struct uart_port *port, unsigned char c)
{
	if (port->iotype == UPIO_MEM32)
		writel(c, port->membase + ZTE_UART_DR);
	else
		writew(c, port->membase + ZTE_UART_DR);
}

static bool zte_con_fifo_empty(struct uart_port *port)
{
	return zte_con.port_ops->tx_empty(port) == TIOCSER_TEMT;
}

static bool zte_con_tx_busy(struct uart_port *port)
{
	return port->state && !kfifo_is_empty(&port->state->port.xmit_fifo);
}

static unsigned int zte_con_fifo_size(struct uart_port *port)
{
	return port->fifosize;
}

static const struct onu_con_uart zte_con_uart = {
	.out		= zte_con_out,
	.fifo_empty	= zte_con_fifo_empty,
	.tx_busy	= zte_con_tx_busy,
	.fifo_room	= zte_con_fifo_size,
	.fifo_depth	= zte_con_fifo_size,
};

static struct onu_console zte_con = {
	.uart		= &zte_con_uart,
	.name		= "zte-console",
	.fallback	= "the PL011 console stays",
	.con.name	= "ttyAMA",
};

static int __init zte_con_init(void)
{
	struct console *vendor = NULL, *c;
	struct uart_driver *drv;
	struct uart_port *port;

	console_list_lock();
	for_each_console(c) {
		if (!strcmp(c->name, "ttyAMA") && (c->flags & CON_ENABLED) && !(c->flags & CON_BOOT)) {
			vendor = c;
			break;
		}
	}
	console_list_unlock();
	drv = vendor ? vendor->data : NULL;
	port = drv && drv->state ? drv->state[vendor->index].uart_port : NULL;
	if (!port) {
		pr_err("zte-console: no enabled ttyAMA console port -- the PL011 console stays\n");
		return -ENODEV;
	}
	return onu_console_start(&zte_con, vendor, port);
}
late_initcall(zte_con_init);
