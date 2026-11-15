// SPDX-License-Identifier: GPL-2.0-only
/* TIER: BOARD */
/*
 * ZTE ZXA10 C610 board CPLD watchdogs, on SSP0. Facts from the unit's U-Boot and stock kernel: an 8-byte frame,
 * byte 0 = 0 read / 1 write, bytes 1-3 = register << 4, byte 7 the value (a read answers in the same frame).
 * 0x1f bit 1 clear arms the one-minute boot watchdog, which U-Boot does just before it starts a kernel and which
 * nothing feeds (the unit resets about 63 s later): set, it is closed. 0x1e is the long watchdog: bit 1 clear arms
 * it, each toggle of bit 2 feeds it. 0x18 bit 1 clear resets the whole board: the stock kernel's only restart
 * handler. This driver closes the boot watchdog, offers the long one as a standard watchdog (closed until opened)
 * and restarts the board the stock way, ahead of PSCI, whose SoC reset leaves the PON chips and the switch running.
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/spi/spi.h>
#include <linux/watchdog.h>

#define CPLD_LONG_WDT		0x1e
#define CPLD_BOOT_WDT		0x1f
#define CPLD_WDT_CLOSED		BIT(1)
#define CPLD_WDT_FEED		BIT(2)
#define CPLD_LONG_S		240	/* the stock help says five minutes, its status line four: the shorter */
#define CPLD_BOARD_RESET	0x18
#define CPLD_RESET_RUN		BIT(1)	/* active low */
#define CPLD_RESET_WAIT_MS	10000	/* the unit resets about 5 s after the write (U-Boot reads reason 0x2 "3sWD") */

static int cpld_rd(struct spi_device *spi, u8 reg, u8 *val)
{
	u8 tx[8] = { 0, 0, reg >> 4, reg << 4 }, rx[8] = {};
	struct spi_transfer t = { .tx_buf = tx, .rx_buf = rx, .len = sizeof(tx) };
	int ret = spi_sync_transfer(spi, &t, 1);

	*val = rx[7];
	return ret;
}

static int cpld_wr(struct spi_device *spi, u8 reg, u8 val)
{
	u8 tx[8] = { 1, 0, reg >> 4, reg << 4, 0, 0, 0, val };

	return spi_write(spi, tx, sizeof(tx));
}

/* new = (old & ~clear | set) ^ toggle, read back; -> 0 or -errno */
static int cpld_update(struct spi_device *spi, u8 reg, u8 clear, u8 set, u8 toggle, u8 *old)
{
	u8 want, now;
	int ret;

	ret = cpld_rd(spi, reg, old);
	if (ret)
		return ret;
	want = ((*old & ~clear) | set) ^ toggle;
	ret = cpld_wr(spi, reg, want) ?: cpld_rd(spi, reg, &now);
	if (ret)
		return ret;
	if (now != want) {
		dev_err(&spi->dev, "CPLD 0x%02x: wrote 0x%02x, reads 0x%02x\n", reg, want, now);
		return -EIO;
	}
	return 0;
}

static int long_wdt(struct watchdog_device *wdd, u8 clear, u8 set, u8 toggle)
{
	u8 old;

	return cpld_update(watchdog_get_drvdata(wdd), CPLD_LONG_WDT, clear, set, toggle, &old);
}

static int long_start(struct watchdog_device *wdd)
{
	return long_wdt(wdd, CPLD_WDT_CLOSED, 0, CPLD_WDT_FEED);
}

static int long_stop(struct watchdog_device *wdd)
{
	return long_wdt(wdd, 0, CPLD_WDT_CLOSED, 0);
}

static int long_ping(struct watchdog_device *wdd)
{
	return long_wdt(wdd, 0, 0, CPLD_WDT_FEED);
}

static const struct watchdog_info long_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "C610 CPLD watchdog",
};

static const struct watchdog_ops long_ops = {
	.owner = THIS_MODULE,
	.start = long_start,
	.stop = long_stop,
	.ping = long_ping,
};

/* PSCI, next in the restart chain, still restarts the SoC when the board reset did not take. */
static int board_restart(struct sys_off_data *data)
{
	struct spi_device *spi = data->cb_data;
	u8 val;
	int ret = cpld_rd(spi, CPLD_BOARD_RESET, &val) ?: cpld_wr(spi, CPLD_BOARD_RESET, val & ~CPLD_RESET_RUN);

	mdelay(CPLD_RESET_WAIT_MS);
	dev_emerg(&spi->dev, "CPLD board reset %s (%d): PSCI restarts the SoC\n", ret ? "not written" : "did not take", ret);
	return NOTIFY_DONE;
}

static int c610_cpld_wdt_probe(struct spi_device *spi)
{
	struct watchdog_device *wdd;
	u8 boot, lng;
	int ret;

	ret = cpld_update(spi, CPLD_BOOT_WDT, 0, CPLD_WDT_CLOSED, 0, &boot) ?: cpld_rd(spi, CPLD_LONG_WDT, &lng);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "boot watchdog not closed\n");
	dev_info(&spi->dev, "boot watchdog closed (0x1f was 0x%02x), long watchdog 0x1e = 0x%02x\n", boot, lng);
	ret = devm_register_sys_off_handler(&spi->dev, SYS_OFF_MODE_RESTART, SYS_OFF_PRIO_HIGH, board_restart, spi);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "board restart handler\n");
	wdd = devm_kzalloc(&spi->dev, sizeof(*wdd), GFP_KERNEL);
	if (!wdd)
		return -ENOMEM;
	wdd->info = &long_info;
	wdd->ops = &long_ops;
	wdd->parent = &spi->dev;
	wdd->timeout = wdd->min_timeout = wdd->max_timeout = CPLD_LONG_S;
	watchdog_set_drvdata(wdd, spi);
	if (!(lng & CPLD_WDT_CLOSED))
		set_bit(WDOG_HW_RUNNING, &wdd->status);	/* the core feeds it until a user opens it */
	return devm_watchdog_register_device(&spi->dev, wdd);
}

static const struct of_device_id c610_cpld_wdt_of_match[] = {
	{ .compatible = "zte,c610-cpld" },
	{ }
};
MODULE_DEVICE_TABLE(of, c610_cpld_wdt_of_match);

static struct spi_driver c610_cpld_wdt_driver = {
	.probe = c610_cpld_wdt_probe,
	.driver = {
		.name = "c610-cpld-wdt",
		.of_match_table = c610_cpld_wdt_of_match,
	},
};
module_spi_driver(c610_cpld_wdt_driver);

MODULE_DESCRIPTION("ZTE ZXA10 C610 board CPLD watchdogs");
MODULE_LICENSE("GPL");
