/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE, not in the STRICT subset */
/* gpon_optics_i2c.h -- an SFF-8472 module's A2h page sampled every period by a worker: the last sample for /proc,
 * the levels for ANI-G once both are read. The shell owns the module (its bus, its lookup) and reads it; this file
 * only decodes and publishes. For a module that runs itself (no host calibration); the shell keys the laser and
 * answers the ANI-G self-test.
 */
#ifndef GPON_OPTICS_I2C_H
#define GPON_OPTICS_I2C_H

#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "gpon_ddm.h"

struct device;
struct omci_onu;

/* The shell reads @len bytes of A2h from GPON_DDM_A2H_BASE into @raw: bytes read or -errno (-EPROBE_DEFER: no bus yet).
 * It may sleep. */
typedef int (*gpon_optics_read_fn)(void *sh, u8 *raw, unsigned int len);

struct gpon_optics_i2c {
	struct device *dev;
	spinlock_t *lock;		/* the shell's model lock: a sample is published under it */
	struct omci_onu *onu;
	gpon_optics_read_fn read_a2h;	/* NULL when the board has no module */
	void *sh;
	struct delayed_work work;
	struct gpon_ddm_a2h a2h;	/* the last sample */
	struct gpon_optic_reading reading;
};

/* Starts sampling through @read_a2h; a NULL one (no module) samples nothing. */
void gpon_optics_i2c_start(struct gpon_optics_i2c *o, struct device *dev, spinlock_t *lock, struct omci_onu *onu,
			   gpon_optics_read_fn read_a2h, void *sh);
void gpon_optics_i2c_stop(struct gpon_optics_i2c *o);
/* Under the lock: the /proc/gpon optic lines ("" without a module). */
void gpon_optics_i2c_fmt(const struct gpon_optics_i2c *o, char *out, size_t sz);

#endif
