// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CORE */
/* The spent recovery budget's policy (eth_watch_spent.h), the image's by default, set at run time by
 * eth_watch.spent_stay_up=0|1 on the command line or /sys/module/eth_watch/parameters/spent_stay_up. */
#include <linux/export.h>
#include <linux/kconfig.h>
#include <linux/moduleparam.h>
#include "eth_watch_spent.h"

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "eth_watch."

bool eth_watch_spent_stay_up = IS_ENABLED(CONFIG_ETH_WATCH_SPENT_STAY_UP);
EXPORT_SYMBOL_GPL(eth_watch_spent_stay_up);
module_param_named(spent_stay_up, eth_watch_spent_stay_up, bool, 0644);
MODULE_PARM_DESC(spent_stay_up, "1: a watch whose recovery budget is spent stops recovering and the unit stays up; 0: it restarts the unit");
