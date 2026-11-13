/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE, not in the STRICT subset */
/* gpon_optics_i2c.h -- an SFF-8472 module whose A2h page is a Linux I2C client (the device tree's `optics`
 * phandle), sampled every period by a worker: the last sample for /proc, the levels for ANI-G once both are read.
 * For a module that runs itself (no host calibration); the shell keys the laser and answers the ANI-G self-test.
 */
#ifndef GPON_OPTICS_I2C_H
#define GPON_OPTICS_I2C_H

#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "gpon_ddm.h"

struct device;
struct device_node;
struct i2c_client;
struct omci_onu;

struct gpon_optics_i2c {
	struct device *dev;
	spinlock_t *lock;		/* the shell's model lock: a sample is published under it */
	struct omci_onu *onu;
	struct device_node *np;		/* the `optics` node, NULL when the board names none */
	struct i2c_client *client;	/* once its bus has probed */
	struct delayed_work work;
	struct gpon_ddm_a2h a2h;	/* the last sample */
	struct gpon_optic_reading reading;
};

/* Starts sampling when @dev's node names `optics`; nothing to do otherwise. */
void gpon_optics_i2c_start(struct gpon_optics_i2c *o, struct device *dev, spinlock_t *lock, struct omci_onu *onu);
void gpon_optics_i2c_stop(struct gpon_optics_i2c *o);
/* Under the lock: the backend's name for gpon_health, and the /proc/gpon optic lines ("" without a module). */
const char *gpon_optics_i2c_name(const struct gpon_optics_i2c *o);
void gpon_optics_i2c_fmt(const struct gpon_optics_i2c *o, char *out, size_t sz);

#endif
