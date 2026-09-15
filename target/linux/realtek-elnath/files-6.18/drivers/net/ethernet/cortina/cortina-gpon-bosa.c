// SPDX-License-Identifier: GPL-2.0
/* Semtech GN25L95 BOSA (burst-mode laser driver + APD/TIA) ...
 * dev/MEASURED-cortina-gpon-bosa.c.md sec 1. */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "gn25l95_cal_logic.h"

#include "cortina-i2c.h"
#include "cortina-gpon-bosa.h"
#include "cortina-gpon-ddm.h"

#define BOSA_I2C_ADDR		0x51	/* SFF-8472 A2h address */

/* GN25L95 registers (un-paged, 0x00-0x7F window) */
#define BOSA_REG_TX_CTL		0x6e	/* bit6 = soft TX-disable */
#define BOSA_REG_PON_CTL	0x6f	/* 0x00 = burst TX enabled */
#define BOSA_REG_PON_CTL2	0x72
#define BOSA_REG_PON_CTL3	0x78
#define BOSA_REG_PON_CTL4	0x79
#define BOSA_REG_PASSWD		0x7b	/* 0x7b-0x7e, unlock = all 0xFF */
#define BOSA_REG_PAGE		0x7f	/* table select for 0x80-0xFF */

#define BOSA_TX_SOFT_DIS	0x40	/* TX_CTL bit6 */

/* paged registers (table.reg) */
#define BOSA_T1_ALARM_EN	0xf8	/* alarm enables (0xf8/0xf9) */
#define BOSA_T1_WARN_EN		0xfc	/* warning enables (0xfc/0xfd) */
#define BOSA_T2_SAFE_MODE	0xa0	/* safe-mode start-up */
#define BOSA_T2_PASSWD_LVL	0xbb	/* bit0 = password level */
/* The identification registers are NOT spelled here: they are the core's, in
 * gn25l95_probe_ops(). A shell copy would be a second place to keep right. */
#define BOSA_SAFE_MODE_START	0x6a

/* pages (tables) of the 0x80-0xFF window */
#define BOSA_PAGE_ALARM		0x01	/* table 1: alarm/warning enables */
#define BOSA_PAGE_DEVICE	0x02	/* table 2: device settings, slopes/offsets */
/* ⚠ THE TABLE-4 / TABLE-5 LABELS ARE DISPUTED, ONE TIER EACH ...
 * dev/MEASURED-cortina-gpon-bosa.c.md sec 2. */
#define BOSA_PAGE_BIAS_LUT	0x04	/* table 4: bias-DAC LUT (label disputed, above) */
#define BOSA_PAGE_MOD_LUT	0x05	/* table 5: modulation-DAC LUT (label disputed, above) */
#define BOSA_PAGE_APD_LUT	0x06	/* table 6: APD LUT */

static int bosa_wr(struct device *dev, u8 reg, u8 val)
{
	int ret = cg_i2c_write_byte(BOSA_I2C_ADDR, reg, val);

	if (ret)
		dev_err(dev, "BOSA: write reg 0x%02x=0x%02x failed (%d)\n",
			reg, val, ret);
	return ret;
}

static int bosa_rd(struct device *dev, u8 reg, u8 *val)
{
	int ret = cg_i2c_read_byte(BOSA_I2C_ADDR, reg, val);

	if (ret)
		dev_err(dev, "BOSA: read reg 0x%02x failed (%d)\n", reg, ret);
	return ret;
}

static int bosa_io_rd(void *ctx, u8 slave, u8 reg, u8 *val)
{
	int ret = cg_i2c_read_byte(slave, reg, val);

	if (ret)
		dev_err(ctx, "BOSA: read %02x:%02x failed (%d)\n", slave, reg, ret);
	return ret;
}

static int bosa_io_wr(void *ctx, u8 slave, u8 reg, u8 val)
{
	int ret = cg_i2c_write_byte(slave, reg, val);

	if (ret)
		dev_err(ctx, "BOSA: write %02x:%02x=%02x failed (%d)\n",
			slave, reg, val, ret);
	return ret;
}

/* POSITIVE identification. The sequence is the CORE's -- 11 ...
 * dev/MEASURED-cortina-gpon-bosa.c.md sec 3. */
static int bosa_identify(struct device *dev)
{
	struct gn_op probe[GN_PROBE_OPS];
	u8 id[GN_PROBE_NOTES] = { 0 };
	struct gn_io io = { .ctx = dev, .rd = bosa_io_rd, .wr = bosa_io_wr,
			    .notes = id, .notes_max = GN_PROBE_NOTES };
	struct gn_fail bad;
	int n, ret;

	n = gn25l95_probe_ops(probe, ARRAY_SIZE(probe));
	if (n < 0)
		return n;

	ret = gn25l95_cal_apply(probe, (u32)n, &io, &bad);
	if (ret) {
		if (bad.err)
			dev_err(dev, "BOSA: no answer to the identification probe at step %u (reg 0x%02x): %d - laser stays unprogrammed\n",
				bad.op, bad.reg, bad.err);
		else
			dev_err(dev, "BOSA: not a usable GN25L95 - probe step %u: reg 0x%02x read 0x%02x, expected 0x%02x under mask 0x%02x\n",
				bad.op, bad.reg, bad.got, bad.want, bad.mask);
		return ret;
	}
	if (gn25l95_is_gn28l9x(id)) {
		dev_err(dev, "BOSA: this part identifies as a GN28L9x (%02x %02x %02x), not a GN25L95 - refusing to program it\n",
			id[0], id[1], id[2]);
		return -ENODEV;
	}
	dev_info(dev, "BOSA: GN25L95 identified (unlocked, family id %02x %02x %02x)\n",
		 id[0], id[1], id[2]);
	return 0;
}

/* WHERE THE CALIBRATION COMES FROM, AND WHY THERE IS NO ...
 * dev/MEASURED-cortina-gpon-bosa.c.md sec 4. */
#define CG_BOSA_CAL_FW	"rtkbosa_k.bin"

static const u8 *bosa_calibration(struct device *dev, const struct firmware **fw)
{
	int ret = request_firmware_direct(fw, CG_BOSA_CAL_FW, dev);

	if (ret) {
		dev_err(dev, "BOSA: %s is not on this filesystem (%d) - refusing to program the laser from anything else\n",
			CG_BOSA_CAL_FW, ret);
		*fw = NULL;
		return NULL;
	}
	if ((*fw)->size != GN_CAL_LEN) {
		dev_err(dev, "BOSA: %s is %zu bytes, expected %u - refusing it\n",
			CG_BOSA_CAL_FW, (*fw)->size, GN_CAL_LEN);
		release_firmware(*fw);
		*fw = NULL;
		return NULL;
	}
	return (*fw)->data;
}

/* Program the GN25L95 from this unit's calibration. The ...
 * dev/MEASURED-cortina-gpon-bosa.c.md sec 5. */
int cg_bosa_init(struct device *dev)
{
	const struct firmware *fw = NULL;
	struct gn_op *plan;
	struct gn_fail bad;
	const u8 *cal;
	int n, ret;

	ret = cg_i2c_init(dev);
	if (ret)
		return ret;

	ret = bosa_identify(dev);
	if (ret)
		return ret;

	plan = kmalloc_array(GN_OPS_MAX, sizeof(*plan), GFP_KERNEL);
	if (!plan)
		return -ENOMEM;

	cal = bosa_calibration(dev, &fw);
	if (!cal) {
		ret = -ENOENT;
		goto out;
	}
	n = gn25l95_cal_ops(cal, GN_CAL_LEN, &gn_variant_x400axf, plan, GN_OPS_MAX);
	if (n < 0) {
		/* checked BEFORE the cast: a negative count read as unsigned is
		 * a walk off the end of the plan. */
		dev_err(dev, "BOSA: the calibration sequence could not be built (%d)\n", n);
		ret = n;
		goto out;
	}

	{
		struct gn_io io = { .ctx = dev, .rd = bosa_io_rd, .wr = bosa_io_wr };

		ret = gn25l95_cal_apply(plan, (u32)n, &io, &bad);
	}
	if (ret) {
		if (bad.err)
			dev_err(dev, "BOSA: stopped at operation %u of %d (reg 0x%02x): bus error %d\n",
				bad.op, n, bad.reg, bad.err);
		else
			dev_err(dev, "BOSA: stopped at operation %u of %d: reg 0x%02x read 0x%02x, expected 0x%02x under mask 0x%02x - I2C bus dead / not programmed (pinmux?)\n",
				bad.op, n, bad.reg, bad.got, bad.want, bad.mask);
		goto out;
	}

	/* Report the laser gate state: TX_CTL bit6 == 0, PON_CTL == 0
	 * dev/MEASURED-cortina-gpon-bosa.c.md sec 6. */
	{
		u8 tx = 0xff, pon = 0xff;
		int rd = bosa_rd(dev, BOSA_REG_TX_CTL, &tx);

		if (!rd)
			rd = bosa_rd(dev, BOSA_REG_PON_CTL, &pon);
		if (rd) {
			dev_err(dev, "BOSA: the part stopped answering right after programming (%d) - nothing here confirms the %d operation(s) landed\n",
				rd, n);
			ret = rd;
			goto out;
		}
		dev_info(dev, "BOSA: GN25L95 programmed (%d operations, calibration from %s), tx_ctl=0x%02x pon_ctl=0x%02x\n",
			 n, CG_BOSA_CAL_FW, tx, pon);
		if ((tx & BOSA_TX_SOFT_DIS) || pon)
			dev_warn(dev, "BOSA: laser gate NOT open after init\n");
	}
out:
	kfree(plan);
	release_firmware(fw);
	return ret;
}

/* One-shot full GN25L95 register dump to dmesg (`echo 'bosa ...
 * dev/MEASURED-cortina-gpon-bosa.c.md sec 7. */
int cg_bosa_dump(struct device *dev)
{
	/* Every table stock selects, each with WHY it is worth a dump
	 * dev/MEASURED-cortina-gpon-bosa.c.md sec 8. */
	static const u8 pages[] = {
		0x00,			/* table 0: the SFF-8472 A2h upper page (tables
					 * 0/1 alias it); the Semtech-series probe selects
					 * it, then requires PASSWD 0x7b-0x7e == 00 00 00 00 */
		BOSA_PAGE_ALARM,	/* table 1: alarm/warning enables 0xf8-0xfd */
		BOSA_PAGE_DEVICE,	/* table 2: device settings; (0xd1 & 0xf0) == 0xa0
					 * is how stock IDs a GN25L95, 0xbb == 0x1c is the
					 * post-replay verify in cg_bosa_init() */
		0x03,			/* table 3: the UX3320_S probe reads 0xf5-0xfc and
					 * wants ASCII "UX3320S0"; this GN25L95 answers
					 * 80 cf 80 ff c0 00 00 ff — meaning unnamed */
		BOSA_PAGE_BIAS_LUT,	/* table 4: LUT (label disputed, see the define) */
		BOSA_PAGE_MOD_LUT,	/* table 5: LUT (label disputed, see the define) */
		BOSA_PAGE_APD_LUT,	/* table 6: APD LUT */
		0x80,			/* table 0x80: LD_disable.sh writes 0x8a = 0xe0 then
					 * 0xa0 here to stop the laser driver; contents
					 * unnamed for the GN25L95 */
		0x81,			/* table 0x81: LD_disable.sh writes 0x94 = 0x01 here
					 * on the same path; contents unnamed for the GN25L95 */
		0x86,			/* table 0x86: the UX3360 probe reads 0x80-0x85 and
					 * wants "UX3360"; this GN25L95 answers all 0xff */
		0x87,			/* table 0x87: the UX3361/UX3322/UX3365 probes read
					 * 0x80-0x86 and want "UX33631"/"UX33640"/"UX33650";
					 * this GN25L95 answers all 0xff */
		0xff,			/* table 0xff: the part-ID table.  Semtech ID at
					 * 0x80/0x85/0x86 — a1 00 00 or ASCII "G96"/"G97"/
					 * "G98" select the GN28L9x family; a GN25L95 answers
					 * ff ff ff and is IDed via table 2 instead.  The
					 * RTL8290C/RTL8291 probes read 0x80/0x81/0x84 here */
	};
	char line[3 * 16 + 1];
	u8 curpage = 0xee, v;
	unsigned int p, r, i, rd_fail = 0;
	int ret, restore, sel;
	int selected = -1;	/* the last page the part actually accepted */

	ret = cg_i2c_init(dev);
	if (ret)
		return ret;

	ret = bosa_rd(dev, BOSA_REG_PAGE, &curpage);
	if (ret) {
		dev_err(dev, "BOSA dump: no ACK from 0x%02x (%d)\n",
			BOSA_I2C_ADDR, ret);
		return ret;
	}
	dev_info(dev, "BOSADUMP start, curpage=0x%02x\n", curpage);

	for (p = 0; p < ARRAY_SIZE(pages); p++) {
		/* ⚠ A PAGE SELECT MAY NOT OVERWRITE AN EARLIER READ FAILURE.
		 *   Assigning this to `ret` made every successful select erase the
		 *   errno of a page whose reads had failed, so a dump that lost a
		 *   whole page still returned 0 as long as the NEXT select worked. */
		sel = bosa_wr(dev, BOSA_REG_PAGE, pages[p]);
		if (sel) {
			if (!ret)
				ret = sel;
			break;
		}
		selected = (int)pages[p];
		for (r = 0; r < 0x100; r += 16) {
			for (i = 0; i < 16; i++) {
				int rd = cg_i2c_read_byte(BOSA_I2C_ADDR, r + i, &v);

				if (rd) {
					rd_fail++;
					if (!ret)
						ret = rd;	/* the FIRST one */
					sprintf(line + 3 * i, " ??");
				} else {
					sprintf(line + 3 * i, " %02x", v);
				}
			}
			dev_info(dev, "BOSADUMP p%02x %02x:%s\n",
				 pages[p], r, line);
		}
	}

	/* ⚠ RESTORE, THEN SAY WHETHER IT WORKED. This claimed "page ...
	 * dev/MEASURED-cortina-gpon-bosa.c.md sec 9. */
	restore = bosa_wr(dev, BOSA_REG_PAGE, curpage);
	if (restore) {
		/* ⚠ SAY WHICH PAGE IT IS ACTUALLY ON, or say we do not know. The
		 *   first cut named the LAST page of the table, which is only true
		 *   when every select succeeded -- after an early select failure
		 *   that sentence is a confident wrong answer about the part. */
		if (selected >= 0)
			dev_err(dev, "BOSADUMP page restore to 0x%02x FAILED (%d) - the part is left on page 0x%02x\n",
				curpage, restore, selected);
		else
			dev_err(dev, "BOSADUMP page restore to 0x%02x FAILED (%d) - which page the part is on is NOT established\n",
				curpage, restore);
		if (!ret)
			ret = restore;
	}
	dev_info(dev, "BOSADUMP %s, rd_fail=%u (page %s)\n",
		 ret ? "FAILED" : "done", rd_fail,
		 restore ? "NOT restored" : "restored");
	return ret;
}

/* Read the SFF-8472 A2h real-time diagnostics (bytes 96..105 ...
 * dev/MEASURED-cortina-gpon-bosa.c.md sec 10. */
int cg_bosa_ddm_read(struct device *dev, struct cg_bosa_ddm *d)
{
	u8 raw[CG_DDM_LEN] = { 0 };
	unsigned int i;
	int io_err = 0;

	for (i = 0; i < CG_DDM_LEN; i++) {
		if (cg_i2c_read_byte(BOSA_I2C_ADDR, CG_DDM_BASE + i, &raw[i])) {
			io_err = 1;
			break;
		}
	}

	if (cg_ddm_decode(raw, io_err, d) != CG_DDM_OK)
		dev_warn_ratelimited(dev, "BOSA DDM: %s\n",
				     cg_ddm_status_str(d->status));
	return d->status;
}

/* Always-on spy hook for /proc/gpon: read back the laser-gate ...
 * dev/MEASURED-cortina-gpon-bosa.c.md sec 11. */
void cg_bosa_proc_show(struct device *dev, struct seq_file *m)
{
	u8 tx = 0xff, pon = 0xff;

	if (cg_i2c_read_byte(BOSA_I2C_ADDR, BOSA_REG_TX_CTL, &tx) ||
	    cg_i2c_read_byte(BOSA_I2C_ADDR, BOSA_REG_PON_CTL, &pon)) {
		seq_puts(m, "bosa           = i2c read FAILED\n");
		return;
	}
	seq_printf(m, "bosa           = tx_ctl(6e)=0x%02x pon_ctl(6f)=0x%02x  (laser gate open = bit6:0 / 0x00)\n",
		   tx, pon);
}
