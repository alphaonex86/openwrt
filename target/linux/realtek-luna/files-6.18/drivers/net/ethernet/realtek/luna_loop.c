// SPDX-License-Identifier: GPL-2.0-only
/*
 * luna_loop.c -- LAN loop detection for both Luna Ethernet shells, on by default
 * like stock (operator 2026-11-02).
 *
 * A die with the switch's RLDP engine (sw_map->rldp_ports: RTL9602C, RTL9607C)
 * has the switch probe each LAN port and latch the port its probe came back on.
 * A die without one (RTL9603CVD) gets a CPU probe per port (flowcore
 * lan_loop_logic.h). Either way the looped port is taken down the way stock
 * does it -- MAC link and PHY power, through the shell's gate -- for
 * loop_block_s, then probed again.
 */
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "luna_eth_regs.h"
#include "luna_loop.h"
#include "lan_loop_logic.h"

static bool loop_detect = true;
module_param(loop_detect, bool, 0644);
MODULE_PARM_DESC(loop_detect, "LAN loop detection: the LAN port a probe returns on goes down for loop_block_s");

static unsigned int loop_probe_s = 5;
module_param(loop_probe_s, uint, 0644);
MODULE_PARM_DESC(loop_probe_s, "seconds between two loop checks (a CPU probe sent, or the switch engine read)");

static unsigned int loop_block_s = 300;
module_param(loop_block_s, uint, 0644);
MODULE_PARM_DESC(loop_block_s, "seconds a looped LAN port stays down before it is probed again (stock: 300)");

static unsigned int loop_ethertype;
module_param(loop_ethertype, uint, 0444);
MODULE_PARM_DESC(loop_ethertype, "the ethertype of the loop probe this die sends; 0 while no detector runs");

/* The switch RLDP engine [tier 3, the RTL9602C and RTL9607C register maps]. */
#define RLDP_CTRL	0x1A000	/* bit 0: the engine runs			*/
#define RLDP_CHECK	0x1A004	/* [23:8] probe period (ms), [7:0] count	*/
#define RLDP_HOLD	0x1A008	/* the same pair while a loop stands	*/
#define RLDP_MAGIC_HI	0x1A014	/* magic octets 0..1: tells our probes apart	*/
#define RLDP_MAGIC_LO	0x1A018	/* magic octets 2..5			*/
#define RLDP_TX_EN	0x1A01C	/* bit per port: the engine probes it	*/
#define RLDP_ENTER	0x1A020	/* bit per port, write 1 to clear: a loop began	*/
#define RLDP_LEAVE	0x1A024	/* bit per port, write 1 to clear: it ended	*/
#define RLDP_LOOPING	0x1A028	/* bit per port: looping now		*/
#define RLDP_PNUM	0x1A02C	/* per port: the port its probe came back on	*/
#define RLDP_TIMING	((2000u << 8) | 4)	/* the init value of both pairs	*/
#define RLDP_ETHERTYPE	0x8899	/* the Realtek protocol ethertype the engine's probe carries */

struct luna_loop {
	struct net_device		*ndev;
	void __iomem			*sw;
	const struct luna_sw_map	*map;
	const struct luna_loop_ops	*ops;
	struct delayed_work		work;
	spinlock_t			lock;	/* @seen and @hold.down vs the RX path */
	u32				seen;	/* CPU probe: ports to take down */
	u32				retry;	/* looped, but the take-down failed: no second port */
	u32				failed;	/* ports whose last gate call failed */
	u32				taking;	/* being taken down: already down to an OMCI unlock */
	struct lan_loop_hold		hold;
	bool				engine;
	u8				id[LAN_LOOP_ID_LEN];
};

static u32 loop_lan_mask(const struct luna_loop *l)
{
	u32 lan = GENMASK(l->map->n_copper - 1, 0);

	return l->engine ? lan & GENMASK(l->map->rldp_ports - 1, 0) : lan;
}

/* Read-modify-write @mask of engine register @reg, then read it back. */
static int loop_set(struct luna_loop *l, u32 reg, u32 mask, u32 val)
{
	u32 back;

	iowrite32((ioread32(l->sw + reg) & ~mask) | (val & mask), l->sw + reg);
	back = ioread32(l->sw + reg);
	if ((back & mask) == (val & mask))
		return 0;
	netdev_err(l->ndev, "loop detection FAILED: switch engine register %#x reads %08x, %08x was written under %08x\n",
		   reg, back, val, mask);
	return -EIO;
}

static int loop_engine_arm(struct luna_loop *l)
{
	const u8 *mac = l->ndev->dev_addr;
	u32 ports = GENMASK(l->map->rldp_ports - 1, 0);
	int rc;

	/* The engine sends from the switch's address, all-zero until set. */
	rc = loop_set(l, l->map->switch_mac, 0xffff, mac[0] << 8 | mac[1]);
	rc = rc ?: loop_set(l, l->map->switch_mac + 4, ~0u,
			    (u32)mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5]);
	rc = rc ?: loop_set(l, RLDP_CHECK, GENMASK(23, 0), RLDP_TIMING);
	rc = rc ?: loop_set(l, RLDP_HOLD, GENMASK(23, 0), RLDP_TIMING);
	rc = rc ?: loop_set(l, RLDP_MAGIC_HI, 0xffff, mac[0] << 8 | mac[1]);
	rc = rc ?: loop_set(l, RLDP_MAGIC_LO, ~0u,
			    (u32)mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5]);
	rc = rc ?: loop_set(l, RLDP_TX_EN, ports, loop_lan_mask(l));
	if (rc)
		return rc;
	iowrite32(ports, l->sw + RLDP_ENTER);
	iowrite32(ports, l->sw + RLDP_LEAVE);
	return loop_set(l, RLDP_CTRL, BIT(0), READ_ONCE(loop_detect));
}

/* -> the ports the engine reports looped since the last read. A release cycles
 * the engine so a loop still standing then is latched again. */
static u32 loop_engine_read(struct luna_loop *l, bool on)
{
	u32 lan = loop_lan_mask(l), looped;

	if (loop_set(l, RLDP_CTRL, BIT(0), on) || !on)
		return 0;
	looped = (ioread32(l->sw + RLDP_ENTER) | ioread32(l->sw + RLDP_LOOPING)) & lan;
	iowrite32(looped, l->sw + RLDP_ENTER);
	return lan_loop_engine_to_block(looped, ioread32(l->sw + RLDP_PNUM),
					l->map->rldp_pnum_bits, lan, READ_ONCE(l->hold.down) | READ_ONCE(l->retry));
}

static void loop_engine_rearm(struct luna_loop *l, u32 ports)
{
	iowrite32(ports, l->sw + RLDP_ENTER);
	iowrite32(ports, l->sw + RLDP_LEAVE);
	if (!loop_set(l, RLDP_CTRL, BIT(0), 0))
		loop_set(l, RLDP_CTRL, BIT(0), READ_ONCE(loop_detect));
}

static void loop_probe_send(struct luna_loop *l, unsigned int port)
{
	struct sk_buff *skb = netdev_alloc_skb(l->ndev, LAN_LOOP_FRAME_LEN);

	if (!skb) {
		l->ndev->stats.tx_dropped++;
		return;
	}
	lan_loop_probe_build(skb_put(skb, LAN_LOOP_FRAME_LEN), l->ndev->dev_addr,
			     l->id, port);
	if (l->ops->xmit(l->ndev, skb, port))
		l->ndev->stats.tx_dropped++;
}

/* Gate @p, logging a failure once per run of failures; -> true when it took. */
static bool loop_gate(struct luna_loop *l, unsigned int p, bool down)
{
	int rc = l->ops->gate(l->ndev, p, down);

	if (!rc) {
		l->failed &= ~BIT(p);
		return true;
	}
	if (!(l->failed & BIT(p)))
		netdev_err(l->ndev, "loop detection FAILED: LAN port %u %s (%d), retried every %u s, logged again once it takes\n",
			   p, down ? "looped but did not go down" : "did not come back up", rc,
			   max(READ_ONCE(loop_probe_s), 1U));
	l->failed |= BIT(p);
	return false;
}

/* Each period: release the ports whose hold expired, take down the ports found
 * looped (and those whose take-down failed before), then probe (a die without
 * the engine). A port is held only once its take-down took. */
static void loop_work(struct work_struct *w)
{
	struct luna_loop *l = container_of(to_delayed_work(w), struct luna_loop, work);
	bool on = READ_ONCE(loop_detect);
	unsigned long now = jiffies, flags;
	u32 due, take, released = 0, downed = 0, again = 0;
	unsigned int p;

	take = l->engine ? loop_engine_read(l, on) : 0;
	spin_lock_irqsave(&l->lock, flags);
	take = on ? (take | l->seen | l->retry) & ~l->hold.down : 0;
	l->taking = take;
	due = lan_loop_hold_due(&l->hold, now, !on) | (on ? 0 : l->retry);
	l->seen = 0;
	spin_unlock_irqrestore(&l->lock, flags);

	for (p = 0; p < LAN_LOOP_PORTS; p++) {
		if (!(due & BIT(p)))
			continue;
		if (!loop_gate(l, p, false)) {
			again |= BIT(p) & ~l->hold.down;	/* a half-gated port, detection off */
			continue;
		}
		released |= BIT(p);
		if (l->hold.down & BIT(p))
			netdev_warn(l->ndev, "loop detection: LAN port %u forwarding again\n", p);
	}
	for (p = 0; p < LAN_LOOP_PORTS; p++) {
		if (!(take & BIT(p)))
			continue;
		if (!loop_gate(l, p, true)) {
			again |= BIT(p);
			continue;
		}
		downed |= BIT(p);
		netdev_warn(l->ndev, "loop detected: LAN port %u down for %u s\n", p, READ_ONCE(loop_block_s));
	}
	spin_lock_irqsave(&l->lock, flags);
	l->hold.down &= ~released;
	l->retry = again;
	lan_loop_hold_add(&l->hold, downed, now, msecs_to_jiffies(READ_ONCE(loop_block_s) * 1000U));
	l->taking = 0;
	spin_unlock_irqrestore(&l->lock, flags);
	if (released && l->engine)
		loop_engine_rearm(l, released);

	if (on && !l->engine && netif_running(l->ndev))
		for (p = 0; p < LAN_LOOP_PORTS; p++)
			if (loop_lan_mask(l) & ~READ_ONCE(l->hold.down) & BIT(p))
				loop_probe_send(l, p);
	schedule_delayed_work(&l->work, max(READ_ONCE(loop_probe_s), 1U) * HZ);
}

struct luna_loop *luna_loop_attach(struct device *dev, struct net_device *ndev,
				   void __iomem *sw, const struct luna_sw_map *map,
				   const struct luna_loop_ops *ops)
{
	struct luna_loop *l = devm_kzalloc(dev, sizeof(*l), GFP_KERNEL);

	if (!l)
		return NULL;
	l->ndev = ndev;
	l->sw = sw;
	l->map = map;
	l->ops = ops;
	l->engine = map->rldp_ports && map->rldp_pnum_bits && map->switch_mac;
	spin_lock_init(&l->lock);
	INIT_DELAYED_WORK(&l->work, loop_work);
	return l;
}
EXPORT_SYMBOL_GPL(luna_loop_attach);

/* Once the switch is up. A die whose engine does not take, or that has neither
 * an engine nor a CPU probe path, says so and detects nothing. */
void luna_loop_start(struct luna_loop *l)
{
	if (!l)
		return;
	get_random_bytes(l->id, sizeof(l->id));
	if (l->engine ? loop_engine_arm(l) : !l->ops->xmit) {
		netdev_err(l->ndev, "loop detection FAILED: unavailable, %s\n",
			   l->engine ? "the switch engine did not take" : "no engine and no CPU probe path");
		return;
	}
	loop_ethertype = l->engine ? RLDP_ETHERTYPE : LAN_LOOP_ETHERTYPE;
	netdev_info(l->ndev, "loop detection: %s (ethertype %#06x) on LAN ports %#x, on=%d\n",
		    l->engine ? "switch engine" : "CPU probe", loop_ethertype, loop_lan_mask(l),
		    READ_ONCE(loop_detect));
	schedule_delayed_work(&l->work, max(READ_ONCE(loop_probe_s), 1U) * HZ);
}
EXPORT_SYMBOL_GPL(luna_loop_start);

/* Our own frame came back from the switch: note its port if it is our probe. */
void luna_loop_rx(struct luna_loop *l, const u8 *f, unsigned int len,
		  unsigned int rx_port)
{
	unsigned long flags;
	int tx, p;

	if (!l || l->engine || !READ_ONCE(loop_detect))
		return;
	tx = lan_loop_probe_tx_port(f, len, l->ndev->dev_addr, l->id);
	if (tx < 0)
		return;
	spin_lock_irqsave(&l->lock, flags);
	p = lan_loop_port_to_block(tx, rx_port, loop_lan_mask(l), l->hold.down | l->retry | l->seen);
	if (p >= 0)
		l->seen |= BIT(p);
	spin_unlock_irqrestore(&l->lock, flags);
	if (p >= 0)
		mod_delayed_work(system_wq, &l->work, 0);
}
EXPORT_SYMBOL_GPL(luna_loop_rx);

static bool loop_held(struct luna_loop *l, unsigned int port)
{
	unsigned long flags;
	bool held;

	spin_lock_irqsave(&l->lock, flags);
	held = (l->hold.down | l->taking) & BIT(port);
	spin_unlock_irqrestore(&l->lock, flags);
	return held;
}

bool luna_loop_port_down(struct luna_loop *l, unsigned int port)
{
	return l && port < LAN_LOOP_PORTS && loop_held(l, port);
}
EXPORT_SYMBOL_GPL(luna_loop_port_down);

MODULE_LICENSE("GPL");
