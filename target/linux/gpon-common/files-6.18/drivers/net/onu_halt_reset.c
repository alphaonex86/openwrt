// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE */
/*
 * An ONU never stays halted: nobody is there to power it back on (a field unit stayed dead two days after a
 * SysRq power-off, 2026-11-25). A halt or power-off (halt, poweroff, SysRq o, critical temperature) panics
 * instead, before the watchdog is stopped: the panic is the crash record (mtdoops, pstore) and panic_timeout
 * resets the chip as a restart does.
 */
#include <linux/init.h>
#include <linux/limits.h>
#include <linux/notifier.h>
#include <linux/panic.h>
#include <linux/reboot.h>

static int onu_halt_reset(struct notifier_block *nb, unsigned long action, void *data)
{
	if (action != SYS_HALT && action != SYS_POWER_OFF)
		return NOTIFY_DONE;
	if (!panic_timeout)
		panic_timeout = 1;	/* 0 would wait for ever */
	panic("%s refused: an ONU restarts instead", action == SYS_HALT ? "halt" : "power-off");
}

static struct notifier_block onu_halt_reset_nb = {
	.notifier_call = onu_halt_reset,
	.priority = INT_MAX,	/* before the watchdog core stops the watchdog */
};

static int __init onu_halt_reset_init(void)
{
	return register_reboot_notifier(&onu_halt_reset_nb);
}
core_initcall(onu_halt_reset_init);
