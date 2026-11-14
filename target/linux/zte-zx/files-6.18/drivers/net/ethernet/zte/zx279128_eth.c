// SPDX-License-Identifier: GPL-2.0-only
/* TIER: CHIP */
/*
 * ZTE ZX279128 Ethernet: the four LAN ports of the SoC's switch, reached through its packet processor (PP).
 * One netdev per port (eth0-3, the stock kernel's names), its internal PHY on the SoC's MDIO bus through phylib.
 * Facts (ONU-F680.md): the unit's U-Boot network driver and the stock kernel's pp/switch drivers, run in the
 * board's VM and read back on the unit.
 *
 * Receive: the PP fills rings 0 and 1 of queue q (16-byte descriptors) with frames, taking buffers from its buffer
 * manager (BMU); the driver copies each frame out, frees the buffer and hands the descriptors back, both rings as stock.
 * Where the PP writes next in a ring survives every reset this driver can pulse (U-Boot's TFTP leaves it mid-ring)
 * and no register tells it: the driver zeroes each descriptor it takes and finds the position in the ring itself.
 * Transmit: the driver takes a BMU buffer, copies the frame in and queues a descriptor on transmit ring 1; the PP
 * returns the buffer to the BMU once sent. Ring 1 of each direction is the PON's (zx279128_gpon.c): what receive
 * ring 1 holds goes to it, and what it sends leaves on transmit ring 0.
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/eth_netdev_contract.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/platform_device.h>

#include "gpon_hwaddr.h"
#include "zx279128_eth_logic.h"
#include "zx279128_gpon.h"
#include "zx279128_gpon_desc.h"

#define DRV "zx279128-eth"

/* The bootarg rung, spelled `zx279128_eth.mac=<addr>` (this composite module's name). The address is per unit:
 * never in a DTS. Stock keeps it in its tagparam JFFS2 file, out of the kernel's reach. */
static char *mac_param;
module_param_named(mac, mac_param, charp, 0444);
MODULE_PARM_DESC(mac, "the unit's base MAC (xx:xx:xx:xx:xx:xx): LAN port p = base + p, the PON = base + 4. "
		 "Empty = DT/nvmem, then a random locally-administered one");
#define PHY_FIRST 0x0a		/* port p's internal PHY sits at MDIO 0x0a + p */

/* Global (offsets from 0x92000000): block resets, active low. */
#define G_RESET			0x000008
#define G_RESET_KEEP		0x03000000	/* bits U-Boot never pulses */
#define G_RESET_PORT(p)		BIT(6 + (p))
#define G_ENABLE		0x00000c
#define G_ENABLE_PP		BIT(17)
#define G_PP_AUX		0x040018	/* U-Boot: 0, then 0x1c = 0xf */
#define G_INT_MASK		0x040044	/* the chip's interrupt sources, 1 = masked (0x1ff after reset) */
#define G_INT_PP		BIT(2)

/* Packet processor. */
#define PP_CFG0			0x140004
#define PP_CFG1			0x140008
#define PP_RX_OFFSET		0x140060	/* 0x11: a received frame starts 16 bytes into its buffer */
#define PP_LIST_NORMAL		0x144010
#define PP_LIST_JUMBO		0x144014
#define PP_RX_RING		0x14401c
#define PP_POOL_NORMAL		0x144020
#define PP_POOL_JUMBO		0x144024
#define PP_BUFSZ_NORMAL		0x144028
#define PP_BUFSZ_JUMBO		0x14402c
#define PP_INT_STATUS		0x144040	/* bits 1-0: receive work pending; the line is status & ~mask */
#define PP_INT_MASK		0x144044	/* 1 = masked */
#define PP_INT_RX		0x3
#define PP_INT_CFG		0x144048
#define PP_CLS_SEL		0x14c014
#define PP_CLS_W1C		0x14c01c
#define PP_CLS_W2C		0x14c02c
#define PP_CLS_CTRL		0x14c080
#define PP_CLS_CTRL2		0x14c088
#define PP_Q_SEL		0x154014
#define PP_Q_READY		0x154018
#define PP_Q_CFG		0x15401c
#define PP_Q_CFG1		0x154020
#define PP_Q_CFG2		0x154024
#define PP_Q_CFG3		0x154028
#define PP_RX_RELEASE_GO	0x154064	/* write 1, reads 0 when taken */
#define PP_RX_RELEASE		0x154068	/* ring << 14 | n << 4 | 8 | queue */
#define PP_Q_ENABLE		0x15406c
#define PP_DMA_CTRL		0x164000
#define PP_DMA_BYTEORDER	0x000f0000
#define PP_DMA_IRQ		0x164004
#define PP_TX_SIZES		0x16403c
#define PP_TX_RING(r)		(0x164050 + 0x10 * (r))	/* ring 0 to the PON, ring 1 to the switch */
#define PP_TX_KICK(r)		(0x164054 + 0x10 * (r))
#define PP_TX_DONE(r)		(0x164058 + 0x10 * (r))	/* bits 15-0: sent since the last read */
#define TX_PON			0
#define TX_LAN			1
#define RX_PON			1
#define PP_RX_COUNT(q)		(0x164100 + 4 * (q))	/* ring 1 bits 31-16, ring 0 bits 15-0 */
#define BMU_CTRL		0x168000
#define BMU_CFG0		0x168004
#define BMU_ALLOCATED		0x16800c	/* bit 31 valid, bits 15-0 the buffer */
#define BMU_FREE		0x168010	/* buffer | jumbo << 15 */
#define BMU_ALLOC		0x168014
#define BMU_COUNT_NORMAL	0x168048	/* entries << 16 */
#define BMU_COUNT_JUMBO		0x16804c
#define BMU_CFG1		0x168058
#define BMU_FREE_STATUS		0x1680dc	/* bit 3 set: no free credit */

/* Switch. */
#define SW_CTRL			0x304000
#define SW_PORT_EN(i)		(0x308040 + 4 * (i))
#define SW_CPU_CFG		0x308140
#define SW_CPU_CFG1		0x308144
#define SW_Q_SEL		0x314014
#define SW_Q_CFG		0x31401c
#define SW_Q_CFG1		0x314020
#define SW_Q_BASE		0x314024
#define SW_INT_MASK		0x32c008
#define SW_INT_CFG		0x32c050

/* Port MAC p at 0x921c0000 + p x 0x40000. */
#define MAC(p)			(0x1c0000 + (p) * 0x40000)
#define MAC_CTRL		0x0000
#define MAC_CTRL_RXTX		0x3
#define MAC_CTRL_DUPLEX_GIGA	BIT(13)
#define MAC_CTRL_100		BIT(14)
#define MAC_CTRL_NOT_GIGA	BIT(15)

#define RX_QUEUES		8
#define RX_SLOTS		1024
#define RX_DESC_SIZE		16
#define RX_RING_BYTES		(RX_SLOTS * RX_DESC_SIZE)
#define RX_AREA_BYTES		(2 * RX_QUEUES * RX_RING_BYTES)	/* ring r of queue q at r x 0x20000 + q x 0x4000 */
#define TX_SLOTS		512
#define TX_DESC_SIZE		32
#define TX_RING_BYTES		(TX_SLOTS * TX_DESC_SIZE)
#define BUF_SIZE		ZX_BUF_SIZE
#define BUF_OFFSET		ZX_BUF_OFFSET
#define NORMAL_BUFS		512
#define JUMBO_BUFS		64
#define POLL_TRIES		200

struct zx_port {
	struct net_device *ndev;
	struct zx_eth *eth;
	struct phy_device *phy;
	unsigned int index;
	int last_speed, last_duplex, last_link;
};

struct zx_eth {
	struct device *dev;
	void __iomem *base;
	void __iomem *mdio;
	struct mii_bus *bus;
	struct napi_struct napi;
	struct net_device *napi_dev;
	struct zx_port *port[ZX_ETH_PORTS];
	struct zx_pon *pon;
	u8 mac[ETH_ALEN];	/* the unit's base address */
	spinlock_t tx_lock;	/* both transmit rings and the BMU allocator */
	u8 *pool;		/* normal pool, then jumbo pool */
	dma_addr_t pool_dma;
	__be16 *lists;		/* normal free list, then jumbo */
	dma_addr_t lists_dma;
	u32 *rx_area;
	dma_addr_t rx_dma;
	u32 *tx_area;		/* ring 0, then ring 1 */
	dma_addr_t tx_dma;
	unsigned int rx_next[2][RX_QUEUES];
	struct eth_tx_ring tx[2];
	struct eth_rx_len rx_len;
	u64 rx_pon, rx_not_lan, rx_bad_buffer, rx_bad_frame, rx_nomem, rx_unwritten, rx_resync;
	u64 bmu_alloc_fail, bmu_free_fail, release_fail;
};

static u32 rd(struct zx_eth *e, u32 off)
{
	return readl(e->base + off);
}

static void wr(struct zx_eth *e, u32 off, u32 val)
{
	writel(val, e->base + off);
}

/* --- MDIO: 0x14 control (register 4-0, PHY 9-5, op 11-10 = 1 write / 2 read, bit 14 start), 0x04 write data,
 * 0x08 read data, 0x10 bit 0 done (cleared by writing it). */
#define MD_WDATA	0x04
#define MD_RDATA	0x08
#define MD_DONE		0x10
#define MD_CTRL		0x14
#define MD_START	BIT(14)

static int zx_mdio_op(struct zx_eth *e, int addr, int reg, u32 op, u32 data)
{
	u32 done;
	int ret;

	writel(data, e->mdio + MD_WDATA);
	writel((reg & 31) | (addr & 31) << 5 | op << 10, e->mdio + MD_CTRL);
	writel((reg & 31) | (addr & 31) << 5 | op << 10 | MD_START, e->mdio + MD_CTRL);
	ret = readl_poll_timeout(e->mdio + MD_DONE, done, done & 1, 1, 10000);
	writel(0, e->mdio + MD_DONE);
	writel(0, e->mdio + MD_CTRL);
	if (ret)
		dev_err(e->dev, "MDIO PHY %d register %d: no completion\n", addr, reg);
	return ret;
}

static int zx_mdio_read(struct mii_bus *bus, int addr, int reg)
{
	struct zx_eth *e = bus->priv;
	int ret = zx_mdio_op(e, addr, reg, 2, 0);

	return ret ? ret : readl(e->mdio + MD_RDATA) & 0xffff;
}

static int zx_mdio_write(struct mii_bus *bus, int addr, int reg, u16 val)
{
	return zx_mdio_op(bus->priv, addr, reg, 1, val);
}

/* --- buffer manager */
static bool bmu_alloc(struct zx_eth *e, u32 *buf)
{
	u32 v = 0;
	int i;

	wr(e, BMU_ALLOC, 1);
	for (i = 0; i < POLL_TRIES; i++) {
		v = rd(e, BMU_ALLOCATED);
		if (v & BIT(31))
			break;
	}
	wr(e, BMU_ALLOC, 0);
	*buf = v & 0xffff;
	if (!(v & BIT(31)) || *buf >= NORMAL_BUFS) {
		e->bmu_alloc_fail++;
		if (v & BIT(31) && *buf < BIT(15))
			wr(e, BMU_FREE, *buf);
		return false;
	}
	return true;
}

static void bmu_free(struct zx_eth *e, u32 buf, bool jumbo)
{
	int i;

	for (i = 0; i < POLL_TRIES; i++) {
		if (!(rd(e, BMU_FREE_STATUS) & BIT(3))) {
			wr(e, BMU_FREE, buf | jumbo << 15);
			return;
		}
	}
	e->bmu_free_fail++;
	dev_warn_ratelimited(e->dev, "BMU refused buffer %u back (status 0x%08x)\n", buf, rd(e, BMU_FREE_STATUS));
}

static u8 *buffer(struct zx_eth *e, u32 buf, bool jumbo)
{
	return e->pool + ((jumbo ? NORMAL_BUFS : 0) + buf) * BUF_SIZE;
}

/* --- hardware init: U-Boot's network init, the rings and pools ours */
static void reset_pulse(struct zx_eth *e, u32 mask)
{
	wr(e, G_RESET, (rd(e, G_RESET) & ~mask) | G_RESET_KEEP);
	udelay(100);
	wr(e, G_RESET, 0xffffffff);
}

static int zx_hw_init(struct zx_eth *e)
{
	static const u32 q_cfg[3][2] = { { 0x000000, 0x400 }, { 0x800000, 0x200080 }, { 0x1000000, 0xc00 } };
	static const u32 dma_irq[][2] = { { 0x164028, 1 }, { 0x16402c, 1 }, { 0x164030, 100000 }, { 0x164034, 0x80 },
					  { PP_DMA_IRQ, 0x7f }, { 0x164020, 0x40 }, { 0x164024, 0x40 } };
	u32 v;
	int i, q;

	for (i = 0; i < ZX_ETH_PORTS; i++)
		wr(e, MAC(i) + MAC_CTRL, 0);
	reset_pulse(e, 0xffffffff);
	wr(e, PP_INT_CFG, 0x000fffff);
	if (readl_poll_timeout(e->base + PP_Q_READY, v, v & 1, 10, 100000)) {
		dev_err(e->dev, "packet processor queues never ready (0x%08x)\n", v);
		return -ETIMEDOUT;
	}
	for (i = 0; i < ARRAY_SIZE(q_cfg); i++) {
		for (q = 0; q < RX_QUEUES; q++) {
			wr(e, PP_Q_SEL, q_cfg[i][0] | q);
			wr(e, PP_Q_CFG3, 0);
			wr(e, PP_Q_CFG2, 0);
			wr(e, PP_Q_CFG1, 0);
			wr(e, PP_Q_CFG, q_cfg[i][1]);
		}
	}
	wr(e, PP_Q_ENABLE, 1);
	wr(e, PP_DMA_CTRL, rd(e, PP_DMA_CTRL) | PP_DMA_BYTEORDER);
	for (i = 0; i < ARRAY_SIZE(dma_irq); i++)
		wr(e, dma_irq[i][0], dma_irq[i][1]);
	wr(e, PP_TX_RING(TX_PON), e->tx_dma);
	wr(e, PP_TX_RING(TX_LAN), e->tx_dma + TX_RING_BYTES);
	wr(e, PP_TX_SIZES, 0x00200020);
	wr(e, PP_CLS_CTRL, 0x0000309a);
	wr(e, PP_CLS_CTRL2, 0);
	wr(e, PP_CLS_SEL, 1);
	wr(e, PP_CLS_W2C, 0x00140000);
	wr(e, PP_CLS_W1C, 0);

	wr(e, BMU_CTRL, 0);
	wr(e, BMU_CFG0, 0x0104c040);
	wr(e, BMU_CFG1, 0x3f);
	wr(e, PP_POOL_NORMAL, e->pool_dma);
	wr(e, PP_POOL_JUMBO, e->pool_dma + NORMAL_BUFS * BUF_SIZE);
	wr(e, PP_LIST_NORMAL, e->lists_dma);
	wr(e, PP_LIST_JUMBO, e->lists_dma + NORMAL_BUFS * sizeof(__be16));
	wr(e, PP_BUFSZ_NORMAL, BUF_SIZE);
	wr(e, PP_BUFSZ_JUMBO, BUF_SIZE);
	wr(e, BMU_COUNT_NORMAL, NORMAL_BUFS << 16);
	wr(e, BMU_COUNT_JUMBO, JUMBO_BUFS << 16);
	wr(e, BMU_ALLOC, 0);
	wr(e, BMU_CTRL, 1);
	wr(e, PP_RX_RING, e->rx_dma);
	wr(e, PP_CFG0, 0x71000050);
	wr(e, PP_CFG1, 0x00000041);

	wr(e, SW_INT_MASK, 0xffffffff);
	for (i = 0; i < 7; i++)
		wr(e, SW_PORT_EN(i), 1);
	wr(e, SW_CPU_CFG, 0x80010000);
	wr(e, SW_CPU_CFG1, 0x00000008);
	{
		static const u32 sw_table[][2] = {
			{ 0x310004, 0x040200ff }, { 0x310340, 0xff5555ff }, { 0x310344, 0x3f }, { 0x310380, 0x3f },
			{ 0x31063c, 0xaaaaaaaa }, { 0x3101c0, 0xff }, { 0x3101c4, 0x5555 }, { 0x310188, 0x000bf874 },
			{ 0x3102c0, 0xff }, { 0x310300, 0xffff }, { 0x310304, 0x3f },
		};

		for (i = 0; i < ARRAY_SIZE(sw_table); i++)
			wr(e, sw_table[i][0], sw_table[i][1]);
	}
	for (q = 0; q < 64; q++) {
		wr(e, SW_Q_SEL, 0x00800000 | q);
		wr(e, SW_Q_BASE, 0x92310000);
		wr(e, SW_Q_CFG1, 0);
		wr(e, SW_Q_CFG, 0x80);
	}
	wr(e, 0x314044, 0x1000);
	wr(e, 0x314040, 0x1000);
	wr(e, SW_CTRL, 1);
	wr(e, SW_INT_CFG, 0);
	wr(e, PP_RX_OFFSET, 0x11);
	wr(e, G_PP_AUX, 0);
	wr(e, G_PP_AUX + 4, 0xf);
	udelay(1000);
	wr(e, G_ENABLE, rd(e, G_ENABLE) | G_ENABLE_PP);
	wr(e, PP_INT_MASK, ~0u);
	wr(e, G_INT_MASK, rd(e, G_INT_MASK) & ~G_INT_PP);
	return 0;
}

/* --- port MAC, as both stock drivers program it once the PHY reports link */
static void zx_mac_up(struct zx_eth *e, unsigned int p, int speed, int duplex)
{
	static const u32 init[][2] = {
		{ 0x0000, 0x00bbe003 }, { 0x0020, 0x80000001 }, { 0x0380, 0x00011200 }, { 0x3400, 0x32 },
		{ 0x34c0, 0xa8 }, { 0x0010, 0xfffe }, { 0x01c0, 0x00300002 }, { 0x02d0, 0x4000 }, { 0x2c00, 0x0010ff11 },
	};
	u32 ctrl;
	int i;

	reset_pulse(e, G_RESET_PORT(p));
	for (i = 0; i < ARRAY_SIZE(init); i++)
		wr(e, MAC(p) + init[i][0], init[i][1]);
	ctrl = rd(e, MAC(p) + MAC_CTRL);
	if (speed == SPEED_1000) {
		ctrl = (ctrl & ~MAC_CTRL_NOT_GIGA) | MAC_CTRL_DUPLEX_GIGA;
	} else {
		ctrl |= MAC_CTRL_NOT_GIGA;
		ctrl = duplex == DUPLEX_FULL ? ctrl | MAC_CTRL_DUPLEX_GIGA : ctrl & ~MAC_CTRL_DUPLEX_GIGA;
		ctrl = speed == SPEED_100 ? ctrl | MAC_CTRL_100 : ctrl & ~MAC_CTRL_100;
	}
	wr(e, MAC(p) + MAC_CTRL, ctrl | MAC_CTRL_RXTX);
}

static void zx_adjust_link(struct net_device *ndev)
{
	struct zx_port *port = netdev_priv(ndev);
	struct phy_device *phy = port->phy;

	if (phy->link == port->last_link && phy->speed == port->last_speed && phy->duplex == port->last_duplex)
		return;
	port->last_link = phy->link;
	port->last_speed = phy->speed;
	port->last_duplex = phy->duplex;
	if (phy->link)
		zx_mac_up(port->eth, port->index, phy->speed, phy->duplex);
	else
		wr(port->eth, MAC(port->index) + MAC_CTRL, 0);
	phy_print_status(phy);
}

/* --- receive */
static bool rx_release(struct zx_eth *e, unsigned int ring, unsigned int q, unsigned int n)
{
	u32 v;

	if (readl_poll_timeout_atomic(e->base + PP_RX_RELEASE_GO, v, !(v & 1), 1, 1000)) {
		e->release_fail++;
		return false;
	}
	wr(e, PP_RX_RELEASE, ring << 14 | n << 4 | 8 | q);
	wr(e, PP_RX_RELEASE_GO, 1);
	return true;
}

static void rx_one(struct zx_eth *e, unsigned int r, u32 *desc)
{
	u32 w[ZX_RX_DESC_WORDS];
	struct net_device *ndev;
	struct sk_buff *skb;
	enum zx_rx_verdict verdict;
	struct zx_rx rx;
	int i;

	for (i = 0; i < ZX_RX_DESC_WORDS; i++)
		w[i] = le32_to_cpu(READ_ONCE(desc[i]));
	memset(desc, 0, RX_DESC_SIZE);
	verdict = zx279128_rx_parse(w, &zx279128_rx_layout, NORMAL_BUFS, JUMBO_BUFS, r == RX_PON, &rx);
	switch (verdict) {
	case ZX_RX_UNWRITTEN:
		e->rx_unwritten++;
		return;
	case ZX_RX_BAD_BUFFER:
		e->rx_bad_buffer++;
		dev_warn_ratelimited(e->dev, "receive descriptor names buffer %u outside its pool\n", rx.buf);
		return;
	case ZX_RX_OMCI:
	case ZX_RX_GEM:
		e->rx_pon++;
		zx_pon_rx(e->pon, verdict == ZX_RX_OMCI, rx.gem, buffer(e, rx.buf, rx.jumbo) + rx.fr.off, rx.fr.len);
		goto free;
	case ZX_RX_NOT_LAN:
		e->rx_not_lan++;
		goto free;
	case ZX_RX_BAD_FRAME:
		e->rx_bad_frame++;
		goto free;
	case ZX_RX_OK:
		break;
	}
	ndev = e->port[rx.port] ? e->port[rx.port]->ndev : NULL;
	if (!ndev || !netif_running(ndev)) {
		e->rx_not_lan++;
		goto free;
	}
	skb = eth_rx_frame_copy(&e->napi, buffer(e, rx.buf, rx.jumbo), &rx.fr, 0);
	if (!skb) {
		e->rx_nomem++;
		ndev->stats.rx_dropped++;
		goto free;
	}
	eth_rx_len_check(&e->rx_len, DRV, skb->data, skb->len, false);
	skb->protocol = eth_type_trans(skb, ndev);
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += skb->len;
	napi_gro_receive(&e->napi, skb);
free:
	bmu_free(e, rx.buf, rx.jumbo);
}

/* The n frames the PP counts in ring r of queue q. An unwritten descriptor where one is counted means the
 * position is not the PP's: it is found again in the ring. */
static void rx_ring(struct zx_eth *e, unsigned int r, unsigned int q, unsigned int n)
{
	u32 *ring = e->rx_area + (r * RX_QUEUES + q) * RX_RING_BYTES / 4;
	unsigned int *next = &e->rx_next[r][q], i;

	if (eth_rx_desc_unwritten(ring + *next * ZX_RX_DESC_WORDS, ZX_RX_DESC_WORDS)) {
		int start = eth_rx_ring_run_start(ring, RX_SLOTS, ZX_RX_DESC_WORDS);

		e->rx_resync++;
		dev_warn_ratelimited(e->dev, "queue %u ring %u: slot %u unwritten, the pending frames start at %d\n", q, r,
				     *next, start);
		if (start >= 0)
			*next = start;
	}
	for (i = 0; i < n; i++) {
		rx_one(e, r, ring + *next * ZX_RX_DESC_WORDS);
		*next = (*next + 1) % RX_SLOTS;
	}
	if (!rx_release(e, r, q, n))
		dev_err_ratelimited(e->dev, "queue %u ring %u: %u descriptors not handed back\n", q, r, n);
}

static int zx_poll(struct napi_struct *napi, int budget)
{
	struct zx_eth *e = container_of(napi, struct zx_eth, napi);
	int done = 0, q, r;

	for (q = RX_QUEUES - 1; q >= 0 && done < budget; q--) {
		u32 count = rd(e, PP_RX_COUNT(q));

		for (r = 1; r >= 0 && done < budget; r--) {
			unsigned int n = min_t(unsigned int, r ? count >> 16 : count & 0xffff, budget - done);

			if (n)
				rx_ring(e, r, q, n);
			done += n;
		}
	}
	if (done < budget && napi_complete_done(napi, done))
		wr(e, PP_INT_MASK, rd(e, PP_INT_MASK) & ~PP_INT_RX);
	return done;
}

static irqreturn_t zx_irq(int irq, void *data)
{
	struct zx_eth *e = data;

	if (!(rd(e, PP_INT_STATUS) & ~rd(e, PP_INT_MASK) & PP_INT_RX))
		return IRQ_NONE;
	wr(e, PP_INT_MASK, rd(e, PP_INT_MASK) | 7);
	napi_schedule(&e->napi);
	return IRQ_HANDLED;
}

/* --- transmit, tx_lock held: a free slot of ring r and a buffer, then the descriptor built around it */
static bool zx_tx_begin(struct zx_eth *e, unsigned int r, u32 *buf)
{
	if (eth_tx_ring_ask(&e->tx[r], TX_SLOTS))
		eth_tx_ring_sent(&e->tx[r], rd(e, PP_TX_DONE(r)) & 0xffff);
	return eth_tx_ring_room(&e->tx[r], TX_SLOTS) && bmu_alloc(e, buf);
}

static void zx_tx_commit(struct zx_eth *e, unsigned int r, const u32 d[ZX_TX_DESC_WORDS])
{
	u32 *slot = e->tx_area + (r * TX_RING_BYTES + eth_tx_ring_take(&e->tx[r], TX_SLOTS) * TX_DESC_SIZE) / 4;
	int i;

	for (i = 0; i < ZX_TX_DESC_WORDS; i++)
		WRITE_ONCE(slot[i], cpu_to_le32(d[i]));
	wmb();	/* descriptor and buffer before the kick */
	wr(e, PP_TX_KICK(r), 1);
}

int zx_eth_pon_tx(struct zx_eth *e, const u8 *data, unsigned int len, u16 gem, unsigned int tcont, bool omci)
{
	u32 d[ZX_TX_DESC_WORDS], buf;
	int ret = 0;

	spin_lock_bh(&e->tx_lock);
	if (!zx_tx_begin(e, TX_PON, &buf)) {
		ret = -EBUSY;
	} else if (!zx279128_pon_tx_desc(d, buf, BUF_OFFSET, len, BUF_SIZE, gem, tcont, omci)) {
		bmu_free(e, buf, false);
		ret = -EINVAL;
	} else {
		memcpy(buffer(e, buf, false) + BUF_OFFSET, data, len);
		zx_tx_commit(e, TX_PON, d);
	}
	spin_unlock_bh(&e->tx_lock);
	return ret;
}

static netdev_tx_t zx_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct zx_port *port = netdev_priv(ndev);
	struct zx_eth *e = port->eth;
	u32 d[ZX_TX_DESC_WORDS], buf, len;

	if (skb_put_padto(skb, ETH_ZLEN)) {
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	len = skb->len;
	spin_lock(&e->tx_lock);
	if (!zx_tx_begin(e, TX_LAN, &buf)) {
		spin_unlock(&e->tx_lock);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (!zx279128_tx_desc(d, buf, BUF_OFFSET, len, BUF_SIZE, port->index)) {
		bmu_free(e, buf, false);
		spin_unlock(&e->tx_lock);
		ndev->stats.tx_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	skb_copy_bits(skb, 0, buffer(e, buf, false) + BUF_OFFSET, len);
	zx_tx_commit(e, TX_LAN, d);
	spin_unlock(&e->tx_lock);
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	dev_consume_skb_any(skb);
	return NETDEV_TX_OK;
}

static int zx_open(struct net_device *ndev)
{
	struct zx_port *port = netdev_priv(ndev);

	port->last_link = port->last_speed = port->last_duplex = -1;
	phy_start(port->phy);
	netif_start_queue(ndev);
	return 0;
}

static int zx_stop(struct net_device *ndev)
{
	struct zx_port *port = netdev_priv(ndev);

	netif_stop_queue(ndev);
	phy_stop(port->phy);
	wr(port->eth, MAC(port->index) + MAC_CTRL, 0);
	return 0;
}

/* The driver's own counters, shared by the four ports (one packet processor). */
static const struct {
	char name[ETH_GSTRING_LEN];
	size_t off;
} zx_stats[] = {
#define ZX_STAT(n) { #n, offsetof(struct zx_eth, n) }
	ZX_STAT(rx_pon), ZX_STAT(rx_not_lan), ZX_STAT(rx_bad_buffer), ZX_STAT(rx_bad_frame), ZX_STAT(rx_nomem),
	ZX_STAT(rx_unwritten), ZX_STAT(rx_resync),
	ZX_STAT(bmu_alloc_fail), ZX_STAT(bmu_free_fail), ZX_STAT(release_fail),
	{ "rx_len_long_lan", offsetof(struct zx_eth, rx_len.long_lan) },
	{ "rx_len_short_lan", offsetof(struct zx_eth, rx_len.short_lan) },
#undef ZX_STAT
};

static int zx_get_sset_count(struct net_device *ndev, int sset)
{
	return sset == ETH_SS_STATS ? ARRAY_SIZE(zx_stats) : -EOPNOTSUPP;
}

static void zx_get_strings(struct net_device *ndev, u32 sset, u8 *data)
{
	int i;

	if (sset != ETH_SS_STATS)
		return;
	for (i = 0; i < ARRAY_SIZE(zx_stats); i++)
		ethtool_puts(&data, zx_stats[i].name);
}

static void zx_get_stats(struct net_device *ndev, struct ethtool_stats *stats, u64 *data)
{
	struct zx_port *port = netdev_priv(ndev);
	int i;

	for (i = 0; i < ARRAY_SIZE(zx_stats); i++)
		data[i] = *(u64 *)((u8 *)port->eth + zx_stats[i].off);
}

static const struct ethtool_ops zx_ethtool_ops = {
	.get_link = ethtool_op_get_link,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.nway_reset = phy_ethtool_nway_reset,
	.get_sset_count = zx_get_sset_count,
	.get_strings = zx_get_strings,
	.get_ethtool_stats = zx_get_stats,
};

static const struct net_device_ops zx_netdev_ops = {
	.ndo_open = zx_open,
	.ndo_stop = zx_stop,
	.ndo_start_xmit = zx_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_eth_ioctl = phy_do_ioctl_running,
};

static int zx_port_probe(struct zx_eth *e, unsigned int index)
{
	struct phy_device *phy = mdiobus_get_phy(e->bus, PHY_FIRST + index);
	struct net_device *ndev;
	struct zx_port *port;
	int ret;

	ndev = devm_alloc_etherdev(e->dev, sizeof(*port));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, e->dev);
	port = netdev_priv(ndev);
	port->ndev = ndev;
	port->eth = e;
	port->index = index;
	snprintf(ndev->name, IFNAMSIZ, "eth%u", index);
	ndev->netdev_ops = &zx_netdev_ops;
	ndev->ethtool_ops = &zx_ethtool_ops;
	ndev->max_mtu = BUF_SIZE - BUF_OFFSET - ETH_HLEN - ETH_FCS_LEN;
	eth_hw_addr_gen(ndev, e->mac, index);
	ret = phy ? phy_connect_direct(ndev, phy, zx_adjust_link, PHY_INTERFACE_MODE_INTERNAL) : -ENODEV;
	if (ret) {
		dev_err(e->dev, "port %u: no PHY at MDIO %#x: %d\n", index, PHY_FIRST + index, ret);
		return ret;
	}
	port->phy = phy;
	ret = eth_register_netdev(ndev, NULL, ETH_NO_ndo_tx_timeout | ETH_NO_get_pauseparam | ETH_NO_set_pauseparam |
					      ETH_NO_get_ringparam | ETH_NO_get_regs | ETH_NO_rx_watch);
	if (ret) {
		dev_err(e->dev, "port %u: register_netdev: %d\n", index, ret);
		phy_disconnect(port->phy);
		return ret;
	}
	e->port[index] = port;
	return 0;
}

static void zx_remove_ports(struct zx_eth *e)
{
	int i;

	for (i = 0; i < ZX_ETH_PORTS; i++) {
		if (!e->port[i])
			continue;
		unregister_netdev(e->port[i]->ndev);
		phy_disconnect(e->port[i]->phy);
		e->port[i] = NULL;
	}
}

static void zx_board_mac(struct zx_eth *e)
{
	u8 dt[ETH_ALEN];
	bool have_dt = !of_get_mac_address(e->dev->of_node, dt);
	enum gpon_hwaddr_src src = gpon_hwaddr_resolve(mac_param, have_dt ? dt : NULL, NULL, e->mac);

	if (src == GPON_HWADDR_RANDOM)
		dev_warn(e->dev, "no MAC in a boot parameter or DT: using %s %pM\n", gpon_hwaddr_src_name(src), e->mac);
	else
		dev_info(e->dev, "MAC %pM from %s\n", e->mac, gpon_hwaddr_src_name(src));
}

static int zx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zx_eth *e;
	int irq, pon_irq, ret, i;

	e = devm_kzalloc(dev, sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;
	e->dev = dev;
	spin_lock_init(&e->tx_lock);
	e->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(e->base))
		return PTR_ERR(e->base);
	e->mdio = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(e->mdio))
		return PTR_ERR(e->mdio);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	pon_irq = platform_get_irq(pdev, 1);
	if (pon_irq < 0)
		return pon_irq;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");
	e->pool = dmam_alloc_coherent(dev, (NORMAL_BUFS + JUMBO_BUFS) * BUF_SIZE, &e->pool_dma, GFP_KERNEL);
	e->lists = dmam_alloc_coherent(dev, (NORMAL_BUFS + JUMBO_BUFS) * sizeof(__be16), &e->lists_dma, GFP_KERNEL);
	e->rx_area = dmam_alloc_coherent(dev, RX_AREA_BYTES, &e->rx_dma, GFP_KERNEL);
	e->tx_area = dmam_alloc_coherent(dev, 2 * TX_RING_BYTES, &e->tx_dma, GFP_KERNEL);
	if (!e->pool || !e->lists || !e->rx_area || !e->tx_area)
		return dev_err_probe(dev, -ENOMEM, "no DMA memory for the pools and rings\n");
	for (i = 0; i < NORMAL_BUFS; i++)
		e->lists[i] = cpu_to_be16(i);
	for (i = 0; i < JUMBO_BUFS; i++)
		e->lists[NORMAL_BUFS + i] = cpu_to_be16(i);
	ret = zx_hw_init(e);
	if (ret)
		return ret;

	e->bus = devm_mdiobus_alloc(dev);
	if (!e->bus)
		return -ENOMEM;
	e->bus->name = DRV " MDIO";
	snprintf(e->bus->id, MII_BUS_ID_SIZE, "%s-mii", dev_name(dev));
	e->bus->priv = e;
	e->bus->parent = dev;
	e->bus->read = zx_mdio_read;
	e->bus->write = zx_mdio_write;
	e->bus->phy_mask = ~(GENMASK(ZX_ETH_PORTS - 1, 0) << PHY_FIRST);
	ret = devm_mdiobus_register(dev, e->bus);
	if (ret)
		return dev_err_probe(dev, ret, "MDIO bus\n");

	e->napi_dev = alloc_netdev_dummy(0);
	if (!e->napi_dev)
		return -ENOMEM;
	netif_napi_add(e->napi_dev, &e->napi, zx_poll);
	ret = devm_request_irq(dev, irq, zx_irq, 0, DRV, e);
	if (ret) {
		netif_napi_del(&e->napi);
		free_netdev(e->napi_dev);
		return dev_err_probe(dev, ret, "IRQ %d\n", irq);
	}
	zx_board_mac(e);
	for (i = 0; i < ZX_ETH_PORTS && !ret; i++)
		ret = zx_port_probe(e, i);
	if (!ret) {
		e->pon = zx_pon_probe(dev, e->base, pon_irq, e, e->mac);
		ret = PTR_ERR_OR_ZERO(e->pon);
	}
	if (ret) {
		e->pon = NULL;
		zx_remove_ports(e);
		netif_napi_del(&e->napi);
		free_netdev(e->napi_dev);
		return ret;
	}
	/* the PON's ring is received whether or not a LAN port is open */
	napi_enable(&e->napi);
	wr(e, PP_INT_MASK, rd(e, PP_INT_MASK) & ~PP_INT_RX);
	platform_set_drvdata(pdev, e);
	dev_info(dev, "%d buffers of %d bytes, receive queues 0-%d rings 0-1, transmit rings 0-1\n", NORMAL_BUFS,
		 BUF_SIZE, RX_QUEUES - 1);
	return 0;
}

static void zx_remove(struct platform_device *pdev)
{
	struct zx_eth *e = platform_get_drvdata(pdev);
	int i;

	wr(e, PP_INT_MASK, ~0u);
	napi_disable(&e->napi);
	zx_pon_remove(e->pon);
	zx_remove_ports(e);
	wr(e, G_INT_MASK, rd(e, G_INT_MASK) | G_INT_PP);
	netif_napi_del(&e->napi);
	free_netdev(e->napi_dev);
	for (i = 0; i < ZX_ETH_PORTS; i++)
		wr(e, MAC(i) + MAC_CTRL, 0);
	wr(e, BMU_CTRL, 0);
}

static const struct of_device_id zx_of_match[] = {
	{ .compatible = "zte,zx279128-eth" },
	{ }
};
MODULE_DEVICE_TABLE(of, zx_of_match);

static struct platform_driver zx_driver = {
	.probe = zx_probe,
	.remove = zx_remove,
	.driver = { .name = DRV, .of_match_table = zx_of_match },
};
module_platform_driver(zx_driver);

MODULE_DESCRIPTION("ZTE ZX279128 Ethernet and GPON");
MODULE_LICENSE("GPL");
