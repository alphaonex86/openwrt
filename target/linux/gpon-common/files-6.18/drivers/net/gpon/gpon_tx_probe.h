/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* gpon_tx_probe.h -- CONFIG_GPON_GEM_DIAG's transmit probe: writing a GEM port to the shell's omci_tx_probe parameter
 * puts one OMCI-sized frame on its PON transmit path, whatever the PON state. The path up to the upstream scheduler
 * needs no grant, so it can be watched with the transmitter off. A shell supplies send(gem) -> 0 or -errno.
 */
#ifndef _GPON_TX_PROBE_H
#define _GPON_TX_PROBE_H

#include <linux/kernel.h>
#include <linux/moduleparam.h>

#define GPON_TX_PROBE_PARAM(send)							\
static unsigned int gpon_tx_probes;							\
static int gpon_tx_probe_set(const char *val, const struct kernel_param *kp)		\
{											\
	u16 gem;									\
	int ret = kstrtou16(val, 0, &gem);						\
											\
	if (ret)									\
		return ret;								\
	ret = send(gem);								\
	if (!ret)									\
		gpon_tx_probes++;							\
	pr_info("gpon: OMCI TX probe on GEM port %u: %d\n", gem, ret);			\
	return ret;									\
}											\
static const struct kernel_param_ops gpon_tx_probe_ops = {				\
	.set = gpon_tx_probe_set, .get = param_get_uint };				\
module_param_cb(omci_tx_probe, &gpon_tx_probe_ops, &gpon_tx_probes, 0644);		\
MODULE_PARM_DESC(omci_tx_probe, "write a GEM port: one OMCI-sized frame onto the PON transmit path, any PON state")

#endif
