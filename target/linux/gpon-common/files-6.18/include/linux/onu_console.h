/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE */
/*
 * The asynchronous kernel console, for any UART that a struct onu_con_uart
 * describes. Included by exactly one console glue per kernel.
 *
 * A synchronous console writes every character with the port lock held and
 * IRQs off, waiting on the UART: ~6 ms per line at 115200 on a single core. The
 * GPON FSM timer printing one line lost the production OLT's Password
 * deadline (98 deactivations, 2026-10-07). An nbcon printer thread writes under
 * the same port lock, wherever the tty's own output stands.
 *
 * This console only copies into a ring (onu_console_ring.h). A thread puts
 * the ring into the UART FIFO when it is empty and sleeps while it drains, so
 * the port lock is held for one FIFO load. Panic, oops and shutdown write
 * synchronously, the ring first, so the last lines still reach the wire. At
 * late_initcall it takes over the vendor console of the same port: same name,
 * same /dev/console.
 *
 * The tty shares the UART. A kernel line starts only while the tty transmitter
 * is idle, and the tty's start_tx waits for the line's last byte, so neither
 * cuts into the other (a FIFO-sized slice of a kernel line inside a shell
 * reply broke the rig's console reads).
 */
#ifndef ONU_CONSOLE_H
#define ONU_CONSOLE_H

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/onu_console_ring.h>
#include <linux/panic.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/serial_core.h>
#include <linux/wait.h>

#define ONU_CON_RING_BYTES	(64 * 1024)
#define ONU_CON_BYTE_US		87	/* one 8N1 character at 115200 */
#define ONU_CON_SPIN_US		10000	/* bound on a synchronous wait for an empty transmitter */
#define ONU_CON_FLUSH_MS	1000
#define ONU_CON_LOAD_MAX	64

/* The UART's part. Port lock held for every call. */
struct onu_con_uart {
	void (*out)(struct uart_port *port, unsigned char c);
	bool (*fifo_empty)(struct uart_port *port);		/* the transmitter takes a load */
	bool (*tx_busy)(struct uart_port *port);		/* the tty is transmitting */
	unsigned int (*fifo_room)(struct uart_port *port);	/* bytes one kernel load may hold */
	unsigned int (*fifo_depth)(struct uart_port *port);	/* bytes of one tty FIFO */
};

struct onu_console {
	const struct onu_con_uart *uart;
	const char *name;		/* the drain thread and the log prefix */
	const char *fallback;		/* what stays when the takeover fails */
	struct console con;		/* .name set by the glue */
	struct uart_port *port;
	const struct uart_ops *port_ops;
	struct uart_ops ops;
	struct onu_con_ring ring;
	struct onu_con_gate gate;	/* port lock */
	wait_queue_head_t wq;
	struct irq_work kick;
	struct notifier_block reboot_nb;
	bool sync;
	unsigned char buf[ONU_CON_RING_BYTES];
};

static struct onu_console *onu_con_now;	/* the one console of this kernel, for start_tx */

/* The tty's start_tx: deferred while a kernel line is on the wire. Port lock held. */
static void onu_con_start_tx(struct uart_port *port)
{
	if (onu_con_gate_tty(&onu_con_now->gate))
		onu_con_now->port_ops->start_tx(port);
}

/* One FIFO load of a kernel line, never past its end. -> bytes worth of time to sleep */
static unsigned int onu_con_drain_step(struct onu_console *oc)
{
	const struct onu_con_uart *u = oc->uart;
	struct uart_port *port = oc->port;
	unsigned char chunk[ONU_CON_LOAD_MAX];
	unsigned int n = 0, i;
	unsigned long flags;
	int eol = 0;

	uart_port_lock_irqsave(port, &flags);
	if (onu_con_gate_kernel(&oc->gate, u->tx_busy(port))) {
		if (u->fifo_empty(port)) {
			n = onu_con_ring_get_line(&oc->ring, chunk,
						  min_t(unsigned int, u->fifo_room(port), sizeof(chunk)), &eol);
			for (i = 0; i < n; i++)
				u->out(port, chunk[i]);
		}
		if (onu_con_gate_loaded(&oc->gate, &oc->ring, n, eol))
			oc->port_ops->start_tx(port);
	} else {
		n = u->fifo_depth(port);	/* the tty is sending: one FIFO of its time */
	}
	uart_port_unlock_irqrestore(port, flags);
	return n;
}

static void onu_con_spin_byte(struct onu_console *oc, unsigned char c)
{
	unsigned int us;

	for (us = 0; us < ONU_CON_SPIN_US && !oc->uart->fifo_empty(oc->port); us++)
		udelay(1);
	oc->uart->out(oc->port, c);
}

/* Panic, oops, shutdown: empty the ring, then @s, on the caller's time. */
static void onu_con_write_sync(struct onu_console *oc, const char *s, unsigned int n)
{
	unsigned long flags;
	unsigned char c;
	unsigned int i;
	int locked = 1;

	if (oops_in_progress)
		locked = uart_port_trylock_irqsave(oc->port, &flags);
	else
		uart_port_lock_irqsave(oc->port, &flags);
	while (onu_con_ring_get(&oc->ring, &c, 1))
		onu_con_spin_byte(oc, c);
	for (i = 0; i < n; i++) {
		if (s[i] == '\n')
			onu_con_spin_byte(oc, '\r');
		onu_con_spin_byte(oc, s[i]);
	}
	if (locked)
		uart_port_unlock_irqrestore(oc->port, flags);
}

static void onu_con_kick(struct irq_work *w)
{
	wake_up(&container_of(w, struct onu_console, kick)->wq);
}

static void onu_con_write(struct console *co, const char *s, unsigned int n)
{
	struct onu_console *oc = container_of(co, struct onu_console, con);

	if (READ_ONCE(oc->sync) || oops_in_progress || panic_in_progress()) {
		onu_con_write_sync(oc, s, n);
		return;
	}
	if (onu_con_ring_put(&oc->ring, s, n))
		irq_work_queue(&oc->kick);
}

static int onu_con_drain(void *data)
{
	struct onu_console *oc = data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(oc->wq, kthread_should_stop() || onu_con_ring_used(&oc->ring));
		while (onu_con_ring_used(&oc->ring) && !kthread_should_stop()) {
			unsigned int us = max(onu_con_drain_step(oc), 1u) * ONU_CON_BYTE_US;

			usleep_range(us, us + 100);
		}
	}
	return 0;
}

static int onu_con_reboot(struct notifier_block *nb, unsigned long action, void *data)
{
	struct onu_console *oc = container_of(nb, struct onu_console, reboot_nb);

	WRITE_ONCE(oc->sync, true);
	onu_con_write_sync(oc, "", 0);
	return NOTIFY_DONE;
}

/* Take over @vendor, the enabled console of @port. -> 0, or the error that left @vendor in place */
static int __init onu_console_start(struct onu_console *oc, struct console *vendor, struct uart_port *port)
{
	struct task_struct *t;
	unsigned long flags;

	oc->ring.buf = oc->buf;
	oc->ring.mask = ONU_CON_RING_BYTES - 1;
	init_waitqueue_head(&oc->wq);
	init_irq_work(&oc->kick, onu_con_kick);
	oc->port = port;
	onu_con_now = oc;
	uart_port_lock_irqsave(port, &flags);
	oc->port_ops = port->ops;
	oc->ops = *oc->port_ops;
	oc->ops.start_tx = onu_con_start_tx;
	port->ops = &oc->ops;
	uart_port_unlock_irqrestore(port, flags);
	t = kthread_run(onu_con_drain, oc, "%s", oc->name);
	if (IS_ERR(t)) {
		pr_err("%s: drain thread not started (%ld) -- %s\n", oc->name, PTR_ERR(t), oc->fallback);
		return PTR_ERR(t);
	}
	oc->reboot_nb.notifier_call = onu_con_reboot;
	oc->reboot_nb.priority = INT_MIN;
	register_reboot_notifier(&oc->reboot_nb);

	oc->con.write = onu_con_write;
	oc->con.flags = CON_ANYTIME;
	oc->con.index = vendor->index;
	oc->con.device = vendor->device;
	oc->con.data = vendor->data;
	if (vendor->flags & CON_NBCON)
		pr_flush(ONU_CON_FLUSH_MS, true);	/* an nbcon console keeps no legacy seq */
	register_console(&oc->con);
	if (!(oc->con.flags & CON_ENABLED)) {
		pr_err("%s: %s%d refused the asynchronous console -- %s\n",
		       oc->name, oc->con.name, oc->con.index, oc->fallback);
		return -EBUSY;
	}
	if (vendor->flags & CON_NBCON) {
		pr_flush(ONU_CON_FLUSH_MS, true);
	} else {
		console_lock();
		oc->con.seq = vendor->seq;
		console_unlock();
	}
	unregister_console(vendor);
	pr_info("%s: %s%d asynchronous, %u-byte ring\n",
		oc->name, oc->con.name, oc->con.index, ONU_CON_RING_BYTES);
	return 0;
}

#endif /* ONU_CONSOLE_H */
