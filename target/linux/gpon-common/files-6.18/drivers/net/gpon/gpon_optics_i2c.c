// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CORE, not in the STRICT subset */
/* gpon_optics_i2c.c -- see gpon_optics_i2c.h. */
#include <linux/device.h>
#include <linux/jiffies.h>

#include "gpon_omci_me.h"
#include "gpon_optics_i2c.h"

#define GPON_OPTICS_PERIOD_MS	10000

static void gpon_optics_i2c_work(struct work_struct *w)
{
	struct gpon_optics_i2c *o = container_of(to_delayed_work(w), struct gpon_optics_i2c, work);
	const u8 both = GPON_OPTIC_RX | GPON_OPTIC_TX;
	struct gpon_optic_reading v = { 0 };
	u8 raw[GPON_DDM_A2H_LEN];
	struct gpon_ddm_a2h d;
	int ret = o->read_a2h(o->sh, raw, sizeof(raw));

	if (ret >= 0 && ret != sizeof(raw))
		ret = -EIO;
	if (gpon_ddm_a2h_decode(raw, ret < 0, &d) == GPON_DDM_OK)
		gpon_optic_from_a2h(&v, &d);
	else
		dev_warn_ratelimited(o->dev, "optics: %s (%d)\n", gpon_ddm_status_str(d.status), ret);
	spin_lock_bh(o->lock);
	o->a2h = d;
	o->reading = v;
	/* a zero power word is no measurement: ANI-G keeps its levels until both are read */
	if ((v.have & both) == both)
		omci_onu_set_optical(o->onu, gpon_ddm_cdbm_to_anig(v.rx_cdbm), gpon_ddm_cdbm_to_anig(v.tx_cdbm));
	spin_unlock_bh(o->lock);
	schedule_delayed_work(&o->work, msecs_to_jiffies(GPON_OPTICS_PERIOD_MS));
}

void gpon_optics_i2c_start(struct gpon_optics_i2c *o, struct device *dev, spinlock_t *lock, struct omci_onu *onu,
			   gpon_optics_read_fn read_a2h, void *sh)
{
	*o = (struct gpon_optics_i2c){ .dev = dev, .lock = lock, .onu = onu, .read_a2h = read_a2h, .sh = sh };
	o->a2h.status = GPON_DDM_ERR_IO;	/* until the first sample */
	INIT_DELAYED_WORK(&o->work, gpon_optics_i2c_work);
	/* one period in: the module's bus probes during boot, so an early "no bus yet" is never reported */
	if (read_a2h)
		schedule_delayed_work(&o->work, msecs_to_jiffies(GPON_OPTICS_PERIOD_MS));
}

void gpon_optics_i2c_stop(struct gpon_optics_i2c *o)
{
	cancel_delayed_work_sync(&o->work);
}

void gpon_optics_i2c_fmt(const struct gpon_optics_i2c *o, char *out, size_t sz)
{
	out[0] = '\0';
	if (o->read_a2h)
		gpon_ddm_a2h_fmt(&o->a2h, out, sz);
}
