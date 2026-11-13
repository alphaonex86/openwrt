// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CORE, not in the STRICT subset */
/* gpon_optics_i2c.c -- see gpon_optics_i2c.h. */
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/of.h>

#include "gpon_omci_me.h"
#include "gpon_optics_i2c.h"

#define GPON_OPTICS_PERIOD_MS	10000

static void gpon_optics_i2c_work(struct work_struct *w)
{
	struct gpon_optics_i2c *o = container_of(to_delayed_work(w), struct gpon_optics_i2c, work);
	const u8 both = GPON_OPTIC_RX | GPON_OPTIC_TX;
	struct i2c_client *client = o->client ?: of_find_i2c_device_by_node(o->np);
	struct gpon_optic_reading v = { 0 };
	u8 raw[GPON_DDM_A2H_LEN];
	struct gpon_ddm_a2h d;
	int ret = -EPROBE_DEFER;	/* the module's bus has not probed yet */

	if (client)
		ret = i2c_smbus_read_i2c_block_data(client, GPON_DDM_A2H_BASE, sizeof(raw), raw);
	if (ret >= 0 && ret != sizeof(raw))
		ret = -EIO;
	if (gpon_ddm_a2h_decode(raw, ret < 0, &d) == GPON_DDM_OK)
		gpon_optic_from_a2h(&v, &d);
	else
		dev_warn_ratelimited(o->dev, "optics: %s (%d)\n", gpon_ddm_status_str(d.status), ret);
	spin_lock_bh(o->lock);
	o->client = client;
	o->a2h = d;
	o->reading = v;
	/* a zero power word is no measurement: ANI-G keeps its levels until both are read */
	if ((v.have & both) == both)
		omci_onu_set_optical(o->onu, gpon_ddm_cdbm_to_anig(v.rx_cdbm), gpon_ddm_cdbm_to_anig(v.tx_cdbm));
	spin_unlock_bh(o->lock);
	schedule_delayed_work(&o->work, msecs_to_jiffies(GPON_OPTICS_PERIOD_MS));
}

void gpon_optics_i2c_start(struct gpon_optics_i2c *o, struct device *dev, spinlock_t *lock, struct omci_onu *onu)
{
	*o = (struct gpon_optics_i2c){ .dev = dev, .lock = lock, .onu = onu,
				       .np = of_parse_phandle(dev->of_node, "optics", 0) };
	o->a2h.status = GPON_DDM_ERR_IO;	/* until the first sample */
	INIT_DELAYED_WORK(&o->work, gpon_optics_i2c_work);
	if (o->np)
		schedule_delayed_work(&o->work, 0);
}

void gpon_optics_i2c_stop(struct gpon_optics_i2c *o)
{
	cancel_delayed_work_sync(&o->work);
	if (o->client)
		put_device(&o->client->dev);
	of_node_put(o->np);
}

const char *gpon_optics_i2c_name(const struct gpon_optics_i2c *o)
{
	return o->client ? o->client->name : "none";
}

void gpon_optics_i2c_fmt(const struct gpon_optics_i2c *o, char *out, size_t sz)
{
	out[0] = '\0';
	if (o->np)
		gpon_ddm_a2h_fmt(&o->a2h, out, sz);
}
