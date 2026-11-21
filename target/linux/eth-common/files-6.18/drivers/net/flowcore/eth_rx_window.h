/* SPDX-License-Identifier: GPL-2.0-only */
/* TIER: CORE */
/* eth_rx_window.h -- an RX ring whose RAM follows its use: the descriptor ring keeps its full length, only a
 * WINDOW of `armed` slots from the CPU's head on holds a buffer, so the hardware stops at the window end as it
 * would at a full ring. The window grows by a group when the backlog reaches half of it and gives a group back
 * after a quiet spell; the driver arms and un-arms at the window end only. PURE: no MMIO, no allocator, no lock;
 * time is an input. Spec: dev/docs/reference/network/RX-WINDOW.md.
 */
#ifndef _ETH_RX_WINDOW_H
#define _ETH_RX_WINDOW_H

#include <linux/types.h>

#define ETH_RX_WINDOW_DYNAMIC_BELOW_MIB	128	/* DRAM below this runs the window; above, it starts at the cap */
#define ETH_RX_WINDOW_GROUP		64
#define ETH_RX_WINDOW_QUIET_MS		10000	/* backlog-free spell, and time since the last grow, before a shrink */
#define ETH_RX_WINDOW_SHRINK_GAP_MS	1000
#define ETH_RX_WINDOW_GUARD		16	/* slots past the hardware position never un-armed: its prefetch */
#define ETH_RX_WINDOW_LAB_DEFAULT	false	/* the lab switch at boot: the board rule */
#define ETH_RX_WINDOW_TICK_MS		100	/* one decision per tick, however often the driver calls it */

struct eth_rx_window {
	u16 cap;		/* the largest window: one slot less than the ring, so a full ring never reads as empty */
	u16 floor;		/* the window never goes below; == cap: a fixed ring */
	u16 board_floor;	/* the floor the board rule gave: the lab switch off */
	u16 group;
	u16 target;		/* what the policy asks; the driver moves `armed` towards it */
	bool pinned;		/* ethtool -G set it: the policy no longer moves it */
	u32 busy_ms;		/* the last grow, or the last decision whose backlog forbade a shrink */
	u32 shrink_ms;
	u32 tick_ms;		/* the last decision */
};

static inline bool eth_rx_window_before(u32 now_ms, u32 then_ms, u32 span_ms)
{
	return (s32)(now_ms - then_ms) < (s32)span_ms;
}

/* The largest window on a ring of `n` descriptors: one slot less, so a full window is never read as an empty one
 * (eth_rx_window_backlog). */
static inline unsigned int eth_rx_window_cap(unsigned int n)
{
	return n - 1;
}

/* The smallest window a board runs on a ring of `n` slots: the group (it must exceed the ring's pause de-assert
 * threshold) when its DRAM is below the line, else the largest. */
static inline unsigned int eth_rx_window_floor(unsigned int n, unsigned int dram_mib)
{
	if (dram_mib >= ETH_RX_WINDOW_DYNAMIC_BELOW_MIB || n <= ETH_RX_WINDOW_GROUP)
		return eth_rx_window_cap(n);
	return ETH_RX_WINDOW_GROUP;
}

static inline void eth_rx_window_init(struct eth_rx_window *w, unsigned int n, unsigned int floor, u32 now_ms)
{
	unsigned int cap = eth_rx_window_cap(n);

	w->cap = cap;
	w->floor = w->board_floor = floor < cap ? floor : cap;
	w->group = ETH_RX_WINDOW_GROUP;
	w->target = w->floor;
	w->pinned = false;
	w->busy_ms = w->shrink_ms = now_ms;
}

/* The lab switch, RAM only (RX-WINDOW.md): true runs the window from one group up whatever the board's DRAM, false
 * gives the board rule back. A running ring then moves by the policy alone, no quiesce; a fixed ring sampled no
 * backlog, so the quiet spell starts at the flip. */
static inline void eth_rx_window_lab(struct eth_rx_window *w, bool dynamic, u32 now_ms)
{
	unsigned int floor = dynamic && w->cap > w->group ? w->group : w->board_floor;

	if (floor == w->floor)
		return;
	w->floor = floor;
	w->busy_ms = now_ms;
	if (w->target < floor)
		WRITE_ONCE(w->target, floor);
}

/* The slot `k` places past the head of a ring of `n` slots. */
static inline unsigned int eth_rx_window_slot(unsigned int head, unsigned int k, unsigned int n)
{
	return (head + k) % n;
}

/* Frames the hardware wrote and the CPU has not taken: from the head to the hardware position, at most the window.
 * The window is shorter than the ring, so a full window is never read as an empty one. */
static inline unsigned int eth_rx_window_backlog(unsigned int hw_pos, unsigned int head, unsigned int armed,
						 unsigned int n)
{
	unsigned int b = (hw_pos % n + n - head) % n;

	return b < armed ? b : armed;
}

/* The decision, once per tick: -> the window to reach. `armed` is the window now, `peak` the largest backlog
 * since the last decision. Grow one group at a backlog of half the window; shrink one group only when the backlog
 * stayed under a quarter of the SMALLER window, and no grow came, for the quiet spell, and the gap after the last
 * shrink. A backlog P therefore settles the window in (2P, 4P + group] and holds it. */
static inline unsigned int eth_rx_window_decide(struct eth_rx_window *w, unsigned int armed, unsigned int peak,
						u32 now_ms)
{
	unsigned int smaller;

	if (w->pinned || armed != w->target)
		return w->target;	/* a fixed size, or a resize still on its way */
	if (2 * peak >= armed && armed < w->cap) {
		w->target = armed + w->group < w->cap ? armed + w->group : w->cap;
		w->busy_ms = now_ms;
		return w->target;
	}
	smaller = armed > w->floor + w->group ? armed - w->group : w->floor;
	if (4 * peak >= smaller)
		w->busy_ms = now_ms;
	if (armed <= w->floor || eth_rx_window_before(now_ms, w->busy_ms, ETH_RX_WINDOW_QUIET_MS) ||
	    eth_rx_window_before(now_ms, w->shrink_ms, ETH_RX_WINDOW_SHRINK_GAP_MS))
		return w->target;
	w->target = smaller;
	w->shrink_ms = now_ms;
	return w->target;
}

/* How many slots may be un-armed now, from the window end backwards: never below `target`, at most `max`, and
 * never a slot within the guard past the hardware position (it may already hold that descriptor). */
static inline unsigned int eth_rx_window_unarmable(unsigned int armed, unsigned int backlog, unsigned int target,
						   unsigned int max)
{
	unsigned int c, safe = backlog + ETH_RX_WINDOW_GUARD;

	if (armed <= target || armed <= safe)
		return 0;
	c = armed - target;
	if (c > armed - safe)
		c = armed - safe;
	return c < max ? c : max;
}

/* The size `ethtool -G` asks, made a window: rounded up to a group, inside floor..cap. */
static inline unsigned int eth_rx_window_pin_size(const struct eth_rx_window *w, unsigned int asked)
{
	unsigned int g = asked ? (asked + w->group - 1) / w->group * w->group : w->group;

	if (g < w->floor)
		g = w->floor;
	return g < w->cap ? g : w->cap;
}

/* Rotate `n` elements of `size` bytes left by `k` in place (three reversals): slot k becomes slot 0. A recovery
 * that restarts the hardware at slot 0 moves the window there with it. */
static inline void eth_rx_window_rotate(void *base, unsigned int n, unsigned int size, unsigned int k)
{
	unsigned char *a = base;
	unsigned int r, i, j, b;

	for (r = 0; n && k % n && r < 3; r++) {
		i = r == 1 ? k % n : 0;
		j = r == 0 ? k % n : n;
		for (j--; i < j; i++, j--)
			for (b = 0; b < size; b++) {
				unsigned char t = a[i * size + b];

				a[i * size + b] = a[j * size + b];
				a[j * size + b] = t;
			}
	}
}

/* ---- The engine: how a driver's ring moves its window. The driver gives five ops on its own descriptors; they are
 * called per poll, per tick or from the worker, never per frame. The includer provides the kernel's barriers,
 * <linux/moduleparam.h> and <linux/of_address.h>. ---- */
struct eth_rx_window_buf {
	void		*buf;
	dma_addr_t	dma;
};

struct eth_rx_window_ring {
	struct eth_rx_window w;
	unsigned int	armed;		/* slots from the head holding a buffer: the poll alone moves it */
	unsigned int	peak;		/* the largest backlog a poll started with since the last tick */
	unsigned int	spare_n, drop_n;	/* release/acquire: worker -> poll, poll -> worker */
	bool		grow_failed;
	struct eth_rx_window_buf spare[ETH_RX_WINDOW_GROUP], drop[ETH_RX_WINDOW_GROUP];
	u32		grows, shrinks, grow_fails;
};

struct eth_rx_window_ops {
	u32 (*hw_pos)(void *ctx);	/* the slot the hardware writes next */
	bool (*arm)(void *ctx, unsigned int slot, struct eth_rx_window_buf b);	/* false: the slot still holds one */
	void (*disarm)(void *ctx, unsigned int slot, struct eth_rx_window_buf *b);	/* the hardware may not take it */
	int (*alloc)(void *ctx, struct eth_rx_window_buf *b);	/* process context; 0 = done */
	void (*free)(void *ctx, struct eth_rx_window_buf b);	/* any context */
};

#define ETH_RX_WINDOW_KICK_WORK	1u	/* the worker has buffers to allocate or free */
#define ETH_RX_WINDOW_KICK_POLL	2u	/* the poll has spares to arm or slots to give back */
#define ETH_RX_WINDOW_STATS	5	/* ethtool -S, in this order: */
#define ETH_RX_WINDOW_STAT_NAMES "rx_window", "rx_window_target", "rx_window_grows", "rx_window_shrinks", \
	"rx_window_grow_fail"

/* The lab switch of one driver, /sys/module/<driver>/parameters/rx_window_lab: written by the lab's test run only. */
#define ETH_RX_WINDOW_LAB_PARAM()							\
	static bool rx_window_lab = ETH_RX_WINDOW_LAB_DEFAULT;				\
	module_param(rx_window_lab, bool, 0644);					\
	MODULE_PARM_DESC(rx_window_lab, "RX window dynamic whatever the DRAM (lab only, until reboot)")

/* The DRAM the device tree declares, in MiB: the line eth_rx_window_floor() draws. */
#define ETH_RX_WINDOW_DRAM_MIB()							\
	static unsigned int eth_rx_window_dram_mib(void)				\
	{										\
		struct device_node *np;							\
		struct resource r;							\
		u64 bytes = 0;								\
		int i;									\
											\
		for_each_node_by_type(np, "memory")					\
			for (i = 0; !of_address_to_resource(np, i, &r); i++)		\
				bytes += resource_size(&r);				\
		return bytes >> 20;							\
	}

static inline bool eth_rx_window_ring_dynamic(const struct eth_rx_window_ring *r)
{
	return r->w.floor < r->w.cap;
}

/* The poll, before it takes frames: the backlog it finds is what grows the window. A fixed ring reads nothing. */
static __always_inline void eth_rx_window_ring_sample(struct eth_rx_window_ring *r, const struct eth_rx_window_ops *ops,
						      void *ctx, unsigned int head, unsigned int n)
{
	unsigned int b;

	if (!eth_rx_window_ring_dynamic(r))
		return;
	b = eth_rx_window_backlog(ops->hw_pos(ctx), head, r->armed, n);
	if (b > r->peak)
		WRITE_ONCE(r->peak, b);
}

/* The poll, the window's only descriptor writer, after it took its frames: it arms the worker's spares at the window
 * end, or gives slots back from the window end, never one within the guard past the hardware position.
 * -> true when it gave some back: the worker must free them. */
static __always_inline bool eth_rx_window_ring_resize(struct eth_rx_window_ring *r, const struct eth_rx_window_ops *ops,
						      void *ctx, unsigned int head, unsigned int n)
{
	unsigned int sn = smp_load_acquire(&r->spare_n), target = READ_ONCE(r->w.target), i, c;

	if (sn) {
		for (i = 0; i < sn && r->armed < n; i++) {
			if (!ops->arm(ctx, eth_rx_window_slot(head, r->armed, n), r->spare[i]))
				break;
			r->armed++;
		}
		for (; i < sn; i++) {	/* never reached while the window keeps its invariant: counted, not leaked */
			ops->free(ctx, r->spare[i]);
			r->grow_fails++;
		}
		smp_store_release(&r->spare_n, 0);
		r->grows++;
	}
	if (r->armed <= target || smp_load_acquire(&r->drop_n))
		return false;
	c = eth_rx_window_unarmable(r->armed, eth_rx_window_backlog(ops->hw_pos(ctx), head, r->armed, n), target,
				    ETH_RX_WINDOW_GROUP);
	for (i = 0; i < c; i++)
		ops->disarm(ctx, eth_rx_window_slot(head, --r->armed, n), &r->drop[i]);
	if (!c)
		return false;
	wmb();		/* the descriptors are the hardware's no more before the worker frees their buffers */
	smp_store_release(&r->drop_n, c);
	r->shrinks++;
	return true;
}

/* The worker (process context): free what the poll gave back, then allocate what the window still lacks, one group
 * at most. A failed allocation keeps the window as it is, counted; the next tick gives that grow up.
 * -> true when spares wait: the poll must arm them. */
static __always_inline bool eth_rx_window_ring_work(struct eth_rx_window_ring *r, const struct eth_rx_window_ops *ops,
						    void *ctx)
{
	unsigned int dn = smp_load_acquire(&r->drop_n), armed, target, want, i;

	for (i = 0; i < dn; i++)
		ops->free(ctx, r->drop[i]);
	if (dn)
		smp_store_release(&r->drop_n, 0);
	if (smp_load_acquire(&r->spare_n))
		return false;
	armed = READ_ONCE(r->armed);
	target = READ_ONCE(r->w.target);
	want = target > armed ? target - armed : 0;
	if (want > ETH_RX_WINDOW_GROUP)
		want = ETH_RX_WINDOW_GROUP;
	for (i = 0; i < want; i++)
		if (ops->alloc(ctx, &r->spare[i]))
			break;
	if (i < want) {
		r->grow_fails++;
		WRITE_ONCE(r->grow_failed, true);
	}
	if (!i)
		return false;
	smp_store_release(&r->spare_n, i);
	return true;
}

/* The tick, under the driver's lock against a pin, on every ring the lab switch may flip: at most one per
 * ETH_RX_WINDOW_TICK_MS, the switch, then the policy's decision. -> who must act (ETH_RX_WINDOW_KICK_*). */
static inline unsigned int eth_rx_window_ring_tick(struct eth_rx_window_ring *r, u32 now_ms, bool lab)
{
	unsigned int armed = READ_ONCE(r->armed), target, kick = 0;

	/* unsigned: a stale or never-set tick_ms waits one tick at most, whatever the clock's start (jiffies does
	 * not start at 0) */
	if (now_ms - r->w.tick_ms < ETH_RX_WINDOW_TICK_MS)
		return 0;
	r->w.tick_ms = now_ms;
	eth_rx_window_lab(&r->w, lab, now_ms);
	if (READ_ONCE(r->grow_failed)) {
		WRITE_ONCE(r->grow_failed, false);
		r->w.target = armed;
	}
	target = eth_rx_window_decide(&r->w, armed, xchg(&r->peak, 0), now_ms);
	if ((target > armed && !READ_ONCE(r->spare_n)) || READ_ONCE(r->drop_n))
		kick |= ETH_RX_WINDOW_KICK_WORK;
	if (target < armed || READ_ONCE(r->spare_n))
		kick |= ETH_RX_WINDOW_KICK_POLL;
	return kick;
}

/* ethtool -G rx=N, under the driver's lock, N already inside floor..cap: the policy no longer moves the window. */
static inline void eth_rx_window_ring_pin(struct eth_rx_window_ring *r, unsigned int asked)
{
	r->w.pinned = true;
	WRITE_ONCE(r->w.target, eth_rx_window_pin_size(&r->w, asked));
}

/* The ETH_RX_WINDOW_STATS values, in ETH_RX_WINDOW_STAT_NAMES order. */
static inline void eth_rx_window_ring_stats(const struct eth_rx_window_ring *r, u64 *data)
{
	data[0] = READ_ONCE(r->armed);
	data[1] = READ_ONCE(r->w.target);
	data[2] = READ_ONCE(r->grows);
	data[3] = READ_ONCE(r->shrinks);
	data[4] = READ_ONCE(r->grow_fails);
}

/* The ring closes: the buffers waiting between the poll and the worker are freed. */
static inline void eth_rx_window_ring_release(struct eth_rx_window_ring *r, const struct eth_rx_window_ops *ops,
					      void *ctx)
{
	unsigned int i;

	for (i = 0; i < r->spare_n; i++)
		ops->free(ctx, r->spare[i]);
	for (i = 0; i < r->drop_n; i++)
		ops->free(ctx, r->drop[i]);
	r->spare_n = r->drop_n = 0;
}

#endif
