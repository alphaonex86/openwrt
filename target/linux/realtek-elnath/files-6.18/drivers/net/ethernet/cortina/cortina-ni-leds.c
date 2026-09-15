// SPDX-License-Identifier: GPL-2.0
/* Cortina-Access NI Ethernet driver for the Realtek RTL9607F ...
 * dev/MEASURED-cortina-ni-leds.c.md sec 1. */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/types.h>

#include "cortina-ni.h"
#include "cortina-ni-regs.h"

#if IS_ENABLED(CONFIG_LEDS_TRIGGERS)

/* One trigger per switch port. The name carries the port ...
 * dev/MEASURED-cortina-ni-leds.c.md sec 2. */
static const char *const ca_ni_link_trig_name[] = {
	"cortina-ni-port0-link",
	"cortina-ni-port1-link",
	"cortina-ni-port2-link",
	"cortina-ni-port3-link",
};

static_assert(ARRAY_SIZE(ca_ni_link_trig_name) == CA_NI_LAN_PORT_COUNT,
	      "one link trigger name per LAN port");

static struct led_trigger ca_ni_link_trig[CA_NI_LAN_PORT_COUNT];

/* Bit p set = trigger p is REGISTERED and may be fired. ★ ...
 * dev/MEASURED-cortina-ni-leds.c.md sec 3. */
static u32 ca_ni_link_trig_live;

/* Publish the per-port link triggers. Called from probe; ...
 * dev/MEASURED-cortina-ni-leds.c.md sec 4. */
void cortina_ni_leds_probe(struct cortina_ni *ni)
{
	unsigned int p;

	/* These triggers are module-global because there is exactly ...
	 * dev/MEASURED-cortina-ni-leds.c.md sec 5. */
	ca_ni_link_trig_live = 0;

	for (p = 0; p < CA_NI_LAN_PORT_COUNT; p++) {
		int ret;

		ca_ni_link_trig[p].name = ca_ni_link_trig_name[p];
		ret = devm_led_trigger_register(ni->dev, &ca_ni_link_trig[p]);
		if (ret) {
			dev_warn(ni->dev,
				 "LAN link LED trigger %s not registered (%d) - that port's lamp will not track link\n",
				 ca_ni_link_trig_name[p], ret);
			continue;
		}
		ca_ni_link_trig_live |= BIT(p);
	}
}

/* One tick of the 1 Hz poll: @link is the per-port PHY-link ...
 * dev/MEASURED-cortina-ni-leds.c.md sec 6. */
void cortina_ni_leds_link_set(u32 link)
{
	unsigned int p;

	for (p = 0; p < CA_NI_LAN_PORT_COUNT; p++) {
		if (!(ca_ni_link_trig_live & BIT(p)))
			continue;
		/*
		 * LED_ON is 1, which is exactly gpio-leds' max_brightness, so
		 * /sys/class/leds/<lamp>/brightness reads back a plain 1 or 0.
		 */
		led_trigger_event(&ca_ni_link_trig[p],
				  (link & BIT(p)) ? LED_ON : LED_OFF);
	}
}

#else	/* !CONFIG_LEDS_TRIGGERS */

/* ★ A LAMP MAY NOT BREAK THE DATAPATH AT BUILD TIME EITHER, ...
 * dev/MEASURED-cortina-ni-leds.c.md sec 7. */
void cortina_ni_leds_probe(struct cortina_ni *ni)
{
	dev_info(ni->dev,
		 "built without CONFIG_LEDS_TRIGGERS - the per-RJ45 link lamps will not be driven\n");
}

void cortina_ni_leds_link_set(u32 link)
{
}

#endif	/* CONFIG_LEDS_TRIGGERS */
