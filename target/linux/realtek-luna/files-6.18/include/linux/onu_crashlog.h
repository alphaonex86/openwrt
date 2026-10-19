/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _LINUX_ONU_CRASHLOG_H
#define _LINUX_ONU_CRASHLOG_H

#include <linux/errno.h>

#if IS_ENABLED(CONFIG_MTD_ONU_CRASHLOG)
int onu_crashlog_note(const char *event);
#else
static inline int onu_crashlog_note(const char *event)
{
	return -ENODEV;
}
#endif

#endif
