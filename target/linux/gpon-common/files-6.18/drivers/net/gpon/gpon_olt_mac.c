// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE, not in the STRICT subset */
/* gpon_olt_mac.c -- see gpon_olt_mac.h. */
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/overflow.h>
#include <linux/printk.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "gpon_olt_mac.h"

#define GPON_OLT_MAC_TICK_MS		10
#define GPON_OLT_MAC_PLOAM_BUDGET	16	/* upstream PLOAMs taken per link and tick */
#define GPON_OLT_MAC_OMCI_BUDGET	256	/* upstream frames taken per tick */

static void link_ds_ploam(void *sh, const u8 m[GPON_OLT_PLOAM_DS_LEN])
{
	struct gpon_olt_mac_link *l = sh;

	if (l->mac->ops->ploam_tx(l->mac->chip, l->link, m)) {
		l->ploam_busy++;
		pr_warn_ratelimited("%s: link %u downstream PLOAM busy, message %02x dropped\n", l->mac->name, l->link,
				    m[1]);
		return;
	}
	l->ds_ploam++;
}

static void link_sn_window(struct gpon_olt_mac_link *l, bool open)
{
	if (l->mac->ops->sn_window(l->mac->chip, l->link, open)) {
		l->grant_fail++;
		pr_warn_ratelimited("%s: link %u serial-number window not %s\n", l->mac->name, l->link,
				    open ? "opened" : "closed");
		return;
	}
	l->sn_open = open;
}

/* Open now, closed at the next tick: the window stock holds open, a few ms in each round. */
static void link_sn_grant(void *sh)
{
	link_sn_window(sh, true);
}

static void link_ranging_grant(void *sh, u8 onu_id)
{
	struct gpon_olt_mac_link *l = sh;

	if (l->mac->ops->ranging_grant(l->mac->chip, l->link, onu_id)) {
		l->grant_fail++;
		pr_warn_ratelimited("%s: link %u ONU %u not granted its ranging burst\n", l->mac->name, l->link, onu_id);
	}
}

static int link_ds_omci(void *sh, u16 gem_port, const u8 *pdu, unsigned int len)
{
	struct gpon_olt_mac_link *l = sh;
	int rc = l->mac->ops->omci_tx(l->mac->chip, l->link, gem_port, pdu, len);

	if (rc == -EBUSY) {
		l->omci_full++;
		pr_warn_ratelimited("%s: link %u OMCI downstream queue full\n", l->mac->name, l->link);
	} else if (!rc) {
		l->ds_omci++;
	}
	return rc;
}

static void link_event(void *sh, u8 onu_id, const char *what, const char *detail)
{
	struct gpon_olt_mac_link *l = sh;

	pr_info("%s: link %u onu %u %s%s%s\n", l->mac->name, l->link, onu_id, what, detail ? " " : "",
		detail ? detail : "");
}

/* What an ONU answered that the core does not handle, rung so it is never silent. */
static void link_unsupported(void *sh, const char *kind, int cls, u32 val, const char *want, const u8 *data,
			     unsigned int len)
{
	struct gpon_olt_mac_link *l = sh;

	(void)data;
	(void)len;
	pr_info_ratelimited("%s: link %u unsupported %s %d 0x%x (want %s)\n", l->mac->name, l->link, kind, cls, val,
			    want ? want : "-");
}

static const struct gpon_olt_port_ops MAC_PORT = {
	.ds_ploam = link_ds_ploam, .sn_grant = link_sn_grant, .ranging_grant = link_ranging_grant,
	.ds_omci = link_ds_omci, .event = link_event, .unsupported = link_unsupported,
};

/* G.984.3: a Serial_Number_ONU from an assigned ONU-ID is that ONU's answer to its ranging grant. */
static void link_us_ploam(struct gpon_olt_mac_link *l, const u8 *m, u32 rtd, u32 now)
{
	if (m[1] == GPON_OLT_US_SN && m[0] != GPON_ONU_ID_BROADCAST)
		gpon_olt_port_ranging_burst(&l->port, m[0], rtd, now);
	else
		gpon_olt_port_us_ploam(&l->port, m, GPON_OLT_PLOAM_US_LEN, now);
	l->us_ploam++;
}

static void mac_us_omci(struct gpon_olt_mac *mac, unsigned int link, u16 gem_port, const u8 *pdu, u32 now)
{
	if (link >= mac->n_links) {
		mac->rx_other++;
		return;
	}
	gpon_olt_port_us_omci(&mac->links[link].port, gem_port, pdu, GPON_OLT_OMCI_LEN, now);
	mac->links[link].us_omci++;
}

static void mac_poll(struct gpon_olt_mac *mac, u32 now)
{
	u8 m[GPON_OLT_PLOAM_US_LEN], pdu[GPON_OLT_OMCI_LEN];
	unsigned int i, n, link;
	u16 gem;
	u32 rtd;
	int rc;

	for (i = 0; i < mac->n_links && mac->ops->ploam_rx; i++)
		for (n = 0; n < GPON_OLT_MAC_PLOAM_BUDGET && mac->ops->ploam_rx(mac->chip, i, m, &rtd); n++)
			link_us_ploam(&mac->links[i], m, rtd, now);
	for (n = 0; n < GPON_OLT_MAC_OMCI_BUDGET && mac->ops->omci_rx; n++) {
		rc = mac->ops->omci_rx(mac->chip, &link, &gem, pdu);
		if (!rc)
			break;
		if (rc < 0)
			mac->rx_other++;	/* OMCI only: the data path is not in this shell */
		else
			mac_us_omci(mac, link, gem, pdu, now);
	}
}

static void mac_tick(struct timer_list *t)
{
	struct gpon_olt_mac *mac = timer_container_of(mac, t, tick);
	u32 now = jiffies_to_msecs(jiffies);
	unsigned int i;

	spin_lock_bh(&mac->lock);
	for (i = 0; i < mac->n_links; i++)
		if (mac->links[i].sn_open)
			link_sn_window(&mac->links[i], false);
	mac_poll(mac, now);
	for (i = 0; i < mac->n_links; i++)
		gpon_olt_port_tick(&mac->links[i].port, now);
	spin_unlock_bh(&mac->lock);
	mod_timer(&mac->tick, jiffies + msecs_to_jiffies(GPON_OLT_MAC_TICK_MS));
}

void gpon_olt_mac_us_ploam(struct gpon_olt_mac *mac, unsigned int link, const u8 msg[GPON_OLT_PLOAM_US_LEN],
			   u32 rtd)
{
	spin_lock_bh(&mac->lock);
	if (link < mac->n_links)
		link_us_ploam(&mac->links[link], msg, rtd, jiffies_to_msecs(jiffies));
	else
		mac->rx_other++;
	spin_unlock_bh(&mac->lock);
}

void gpon_olt_mac_us_omci(struct gpon_olt_mac *mac, unsigned int link, u16 gem_port,
			  const u8 pdu[GPON_OLT_OMCI_LEN])
{
	spin_lock_bh(&mac->lock);
	mac_us_omci(mac, link, gem_port, pdu, jiffies_to_msecs(jiffies));
	spin_unlock_bh(&mac->lock);
}

/* /proc/<name>: the ONU table, one line per ONU heard, then each link's counters. */
static int mac_show(struct seq_file *m, void *v)
{
	struct gpon_olt_mac *mac = m->private;
	unsigned int i, k;
	char line[192];

	(void)v;
	spin_lock_bh(&mac->lock);
	for (i = 0; i < mac->n_links; i++) {
		const struct gpon_olt_mac_link *l = &mac->links[i];

		for (k = 0; k < l->port.n_eng; k++)
			if (gpon_olt_port_show(&l->port, k, line, sizeof(line)))
				seq_puts(m, line);
		seq_printf(m, "link %u ploam ds %llu busy %llu us %llu omci ds %llu full %llu us %llu grant_fail %llu\n",
			   i, l->ds_ploam, l->ploam_busy, l->us_ploam, l->ds_omci, l->omci_full, l->us_omci,
			   l->grant_fail);
	}
	seq_printf(m, "rx_other %llu\n", mac->rx_other);
	spin_unlock_bh(&mac->lock);
	return 0;
}

int gpon_olt_mac_start(struct gpon_olt_mac *mac, struct device *dev, const char *name,
		       const struct gpon_olt_mac_ops *ops, void *chip, unsigned int n_links, unsigned int n_onus,
		       const struct gpon_olt_profile *tmpl)
{
	struct gpon_olt_omci *eng;
	const char *refused;
	unsigned int i;

	if (!ops || !ops->ploam_tx || !ops->sn_window || !ops->ranging_grant || !ops->omci_tx || !ops->stop ||
	    !n_links || !n_onus)
		return dev_err_probe(dev, -EINVAL, "%s: the OLT PON MAC shell refused its ops\n", name);
	mac->links = devm_kcalloc(dev, n_links, sizeof(*mac->links), GFP_KERNEL);
	eng = devm_kcalloc(dev, array_size(n_links, n_onus), sizeof(*eng), GFP_KERNEL);
	if (!mac->links || !eng)
		return -ENOMEM;
	mac->ops = ops;
	mac->chip = chip;
	mac->name = name;
	mac->n_links = n_links;
	spin_lock_init(&mac->lock);
	for (i = 0; i < n_links; i++) {
		mac->links[i].mac = mac;
		mac->links[i].link = i;
		refused = gpon_olt_port_init(&mac->links[i].port, &MAC_PORT, &mac->links[i], eng + i * n_onus,
					     n_onus, tmpl, true);
		if (refused)
			return dev_err_probe(dev, -EINVAL, "%s: link %u: the OLT port refused its %s\n", name, i,
					     refused);
	}
	if (!proc_create_single_data(name, 0444, NULL, mac_show, mac))
		return dev_err_probe(dev, -ENOMEM, "/proc/%s not created\n", name);
	timer_setup(&mac->tick, mac_tick, 0);
	mac->running = true;
	mod_timer(&mac->tick, jiffies + msecs_to_jiffies(GPON_OLT_MAC_TICK_MS));
	return 0;
}

void gpon_olt_mac_stop(struct gpon_olt_mac *mac)
{
	if (!mac->running)
		return;
	mac->running = false;
	timer_shutdown_sync(&mac->tick);
	remove_proc_entry(mac->name, NULL);
	mac->ops->stop(mac->chip);
}
