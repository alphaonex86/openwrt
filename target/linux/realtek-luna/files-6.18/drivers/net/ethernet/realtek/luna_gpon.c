// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: FAMILY -- the Luna GPON MAC shell: registers, DMA, interrupts and board
 * glue for EVERY Luna die. It DOES; the core DECIDES. GPON protocol logic
 * belongs in the core tier (drivers/net/gpon), and a per-die difference is a
 * TABLE entry (luna_sw_map, the chipdef offsets), never a fork.
 * Built for BOTH Luna subtargets under CONFIG_LUNA_GPON: taroko/RTL9602C and
 * interaptiv/RTL9603CVD.
 * The tier rule and the file map live in ONE place: "THE THREE TIERS" in
 * gpon-common/files-6.18/drivers/net/gpon/gpon_common.h.
 *
 * Runtime knobs are `luna_gpon.<name>` (KBUILD_MODNAME follows the object
 * basename); they were `gpon_luna.<name>` before 2026-09-10, and a stale bootarg
 * is silently ignored.
 * Provenance pointers below name EARLIER names of this file on purpose, with the
 * line numbers as they were when each move was recorded.
 */
/*
 * Realtek Luna GPON MAC. The register map below is the RTL9602C's, where this
 * shell was first brought up; per-die differences are table entries.
 *
 * Independent implementation from the SoC's register interface and G.984/G.988.
 * The GPON MAC is a sub-block of the Luna switch core (SWCORE, phys 0x1B000000);
 * its window begins at SWCORE + 0x700000 and responds without extra gating, and
 * GPON_TEST reads the power-on pattern 0x12345678.
 *
 * The activation FSM (downstream sync, ranging) runs autonomously in hardware
 * once the MAC is out of soft-reset and fed valid downstream GTC. The SerDes
 * CMU/PLL provides the MAC core clock, so without it the MAC cannot leave reset
 * and the GTC banks read floating.
 *
 * GPON register block (offsets from the GPON base 0x1B700000):
 *   0x00000  GPON_INT_DLT        aggregate interrupt delta
 *   0x0000c  GPON_RESET          [8] RST_DONE, [0] SOFT_RST (1=assert)
 *   0x00010  GPON_VERSION        [7:0] VER_ID
 *   0x00014  GPON_TEST           scratch (power-on pattern 0x12345678)
 *   0x00020  GPON_AES_BYPASS
 *   0x00040  GPON_INTR_MASK
 *   0x00044  GPON_INTR_STS
 *   0x01000  GPON_GTC_DS_INTR_DLT   downstream GTC interrupt delta
 *   0x01004  GPON_GTC_DS_INTR_MASK
 *   0x01008  GPON_GTC_DS_INTR_STS
 *   0x01010  GPON_GTC_DS_ONU_ID_STATUS [15:8] ONU_ID, [3:0] ONU_STATE (O1..O7)
 *   0x05010  GPON_GTC_US_ONU_ID     [15:8] ONU_ID (upstream copy)
 *   0x05040  GPON_GTC_US_MIN_DELAY  [15:7] MIN_DELAY1, [6:0] MIN_DELAY2
 *   0x05044  GPON_GTC_US_EQD        [26:24] EQD multiframe, [17:0] EQD in-frame
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <gpon_omci_core.h>
#include <gpon_omci_me.h>
#include <gpon_omci_diag.h>
#include <gpon_omci_trace.h>
#include <linux/net.h>
#include <gpon_data_plan.h>
#include <linux/slab.h>
#include <gpon_range_gate.h>
#include <gn25l95_cal_logic.h>
#include "gpon_sn.h"	/* the common G.984.3 ONU-SN codec */
#include "gpon_ploam.h"	/* the core PLOAM FSM + its shell contract */
#include "gpon_ploam_diag.h"	/* the core's activation diagnostic: WHAT to read at Assign/Ranging/Deact */
#include "gpon_gem_us.h"	/* GPON_GEM_US_RANGE_OK: the core's own bound predicate */
#include "luna_gpon_logic.h"	/* hoisted logic */
#include "hwio.h"	/* flowcore: the ONE canonical field-mask RMW */
#include "gpon_gtc_ploam.h"	/* the core: the DS PLOAM buffer unpack, fed this shell's gpon_io */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gfp.h>		/* __get_free_pages / GFP_KERNEL for the US PBO DRAM pool */
#include <linux/mm.h>
#include <linux/mm.h>		/* virt_to_phys */
#include <linux/kernel.h>
#include <linux/limits.h>	/* S32_MIN: the "no optical reading" sentinel */
#include <linux/string.h>	/* strscpy for the n/a fields in /proc/gpon */
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include "gpon_gem_diag.h"
#include "luna_gpon_nic.h"
#include "luna_eth_regs.h"	/* SOC_SW_ENABLE + the family register map */
#include "luna_ponmac.h"		/* clean-room family PON-MAC/SerDes bring-up lib */

/* Anything this driver receives and cannot place leaves through ONE spelling,
 * authored in the shared GPON tree and read by dev/ONU-test-case/unsup_scan.py:
 *   luna-gpon: UNSUP kind=<slug> class=<...> val=<v> want=<t> n=<c> d=<hex>
 * The subsystem name is defined BEFORE the include so the lines carry this
 * file's existing prefix. */
/* ★ THE PREFIX IS THE FAMILY'S.  This driver is built for BOTH Luna
 * subtargets under CONFIG_LUNA_GPON, and it printed a DIE's name --
 * every line it emitted on an RTL9603CVD said rtl9602c.  A misleading
 * name is a defect, and the temporary UNI seam already used luna-gpon,
 * so the file was saying two things at once. */
#define GPON_UNSUP_SUBSYS	"luna-gpon"
#include "gpon_unsup.h"		/* shared: the UNSUP report + its rate limit */

#define GPON_PHYS_BASE	0x1b700000u

#include "luna_gpon_regs.h"	/* the per-SoC offsets; the logic below is chip-agnostic */
#define   GPON_BOH_LEN		12		/* stored bytes (TOTAL_OVERHEAD_BITS(96)/8); HW extends via REPEAT */
#define   GPON_BOH_MAX_LEN	252		/* hardware BOH_LENGTH field cap */

#define   GEM_US_PORT_MAP_STRIDE 4u		/* MUST be 4: the register array's
						 * declared "32" is the element BIT width, so the
						 * byte stride is 32/8 and flow 64 lands at 0x6500.
						 * A 0x20 stride writes 0x6C00 instead -- the
						 * per-T-CONT idle-byte STAT counter, live and
						 * non-zero on any online ONU, so write and readback
						 * agree on a wrong offset while the real port-map
						 * slot stays unmapped and the OMCC drains nothing. */
/* Compile-time guard: the stride must stay 4 (see above); catch any regression. */
static_assert(GEM_US_PORT_MAP_STRIDE == 4u,
	      "GEM_US_PORT_MAP stride MUST be 4 (32-bit words) per chipdef array-offset 32");
/* Each die's own GPON_OMCI_FLOW_ID (vendor chipdef, tier 3; confirmed live on the
 * G24W's stock at O5). Named per chip so the range assertions below stay
 * COMPILE-TIME -- a runtime table value cannot be static_assert'ed. */
#define GPON_OMCC_FLOW_9602C	64u
#define GPON_OMCC_FLOW_9603CVD	127u
#define GPON_OMCC_FLOW_9607C	127u
/* The live one, from the selected chip's table. `swc` is assigned before any GPON
 * register is touched, and every use of this macro sits after that. */
#define GPON_OMCC_FLOW		((unsigned int)swc->omcc_flow)
/* The OMCC's PHYSICAL queue id, and it is PER CHIP: 64 / 127 / 120. It derives
 * from TCONT_QUEUE_MAX, which is 32 on the RTL9602C and EIGHT on the RTL9603CVD
 * -- and that die does not use the formula for this queue at all: its DAL
 * special-cases the OMCI flow to physical queue 127 and then guards
 * `physicalQid == 127 && flow != OMCI_FLOW_ID` as an error. This driver wrote 64
 * on every die, pointing the RTL9603CVD's OMCC at a queue belonging to T-CONT 8
 * rather than to the OMCC's T-CONT 16. */
#define GPON_OMCC_PHYS_QID	((unsigned int)swc->omcc_phys_qid)
#define GPON_OMCC_TCONT		16	/* the OMCC's T-CONT index; same on both dies */
#define GPON_DATA_TCONT		8	/* the WAN data GEM's own T-CONT; see GPON_DATA_ALLOC */

/*
 * PLOAM message path (G.984.3 management channel driving the ONU through
 * O3..O5). This MAC is a software-PLOAM design: received downstream PLOAMs land
 * in an 8-word buffer firmware dequeues, and upstream PLOAMs are composed in an
 * 8-word buffer and enqueued; the indicator registers expose buffer occupancy
 * and the dequeue/enqueue triggers.
 */
#define GPON_GTC_US_ONU_ID_SHIFT 8		/* [15:8] OLT-assigned ONU-ID  */

/*
 * SoC hardware I2C master (SWCORE register file). The external RTL8290B BOSA
 * hangs off bus 0 and must be initialised over this master before the real
 * optical signal-detect asserts. The master is indirect: program the per-bus
 * I2C_CONFIG, write the target register offset into I2C_IND_ADR, kick
 * I2C_IND_CMD, poll BUSY, then read I2C_IND_RD.
 */
/* Controller index -> physical panel LED, confirmed by cable test: the
 * GE-labelled LED is index 1 (driven from the GE port, UTP1) and the FE-labelled
 * LED is index 15 (driven from the FE port, UTP0). */
#define GE_LED_IDX		1u
#define FE_LED_IDX		15u

/*
 * SoC GPIO controller (its own register page at phys 0x18003300, outside the
 * switch-core window). The optical signal-detect is wired to a board GPIO, and a
 * working unit enables a specific set of pins here with GPIO 21 as its lone
 * input; configure the controller to that state so the pin is sampled and reaches
 * the GPON LOS input.
 */

/*
 * PON packet-buffer / datapath ("PON-IP", "PBO/PONNIC"): physically the top
 * window of the switch core at phys 0x1bf00000, far above the SWCORE control
 * window, so it gets its own ioremap and is only reachable once the PON
 * IP-enable bit is set. The GPON MAC drains downstream GEM frames into it and
 * sources upstream frames from it, and it must be configured -- page/SRAM
 * accounting, GPON mode, GMII enables -- before the MAC soft-reset, or the reset
 * handshake never completes. Offsets are relative to the PON-IP base, which is
 * split into upstream and downstream halves plus PONNIC IO command pages.
 */

/*
 * SRAM page accounting for GPON, 128-byte pages, no DRAM reservation.
 * US descriptor ring = 128 pages, DS = 32 pages; the registers hold count-1.
 */
#define PI_US_SRAM_NO		127u		/* 128 pages - 1               */
#define PI_DS_SRAM_NO		31u		/* 32 pages - 1                */
#define PI_US_SRAM_RUNOUT	126u		/* SRAM_NO - 1                 */
#define PI_DS_SRAM_RUNOUT	30u

/* ONU activation FSM states (HW encodes the G.984.3 O-states directly). */
static const char * const gpon_onu_state_name[] = {
	[0] = "unknown",   [1] = "O1-initial",  [2] = "O2-standby",
	[3] = "O3-serial", [4] = "O4-ranging",  [5] = "O5-operation",
	[6] = "O6-popup",  [7] = "O7-emergency",
};

static void __iomem *gpon_base;
static void __iomem *swcore_base;
static void __iomem *ponip_base;

/* PLOAM activation FSM state (the FSM is defined below the proc dump). onu_sn is
 * empty until provisioned; the parameter is writable, re-parses on write, and the
 * FSM re-reads the parsed serial for each Serial_Number_ONU. */
static void gpon_parse_sn(const char *s);	/* defined below; re-parses onu_sn */
static bool gpon_sn_differs(const char *s);	/* defined below; parsed-byte compare */
/* NO COMPILED-IN SERIAL. This defaulted to "XPON39013867", ONE unit's real
 * serial, and the same source builds the LANLY G24W: measured 2026-09-06, that
 * board reached O3 announcing the X111W's identity on the splitter the X111W was
 * online on. Empty = "not provisioned": the FSM parks at O1 and announces NOTHING
 * until a serial arrives, which is also stock's order -- SN set, THEN activate. */
static char *onu_sn = "";
static DEFINE_MUTEX(bosa_lock);
static bool luna_driver_ready, luna_activation_ready, luna_stopping;
static bool bosa_gn_identified, bosa_cal_ready;
static int bosa_cal_error;
static int luna_activate_locked(bool identity_changed);
static int luna_quiesce_locked(void);
static void luna_resume_poll(void);
static void luna_omci_identity_reset(const u8 sn[8]);
static void luna_data_suspend(void);
static void luna_omci_seq_show(struct seq_file *s);
static int luna_data_retire(void);
static u8 luna_data_qid;
static bool gpon_sn_changed;		/* SN (re)provisioned -> FSM must re-range */

/* Forward declaration: the core FSM object is defined with the rest of the shell
 * far below, but the onu_sn setter above must hand it the new serial and can fire
 * at insmod, before the GPON probe has run. gpon_ploam_init() memsets the object
 * and re-seeds the serial, so an early write is simply overwritten; what must NOT
 * happen is the core missing an identity change while it is not driving. */
static struct gpon_ploam luna_ploam;
static u8 gpon_sn_bytes[8];		/* defined here for the same reason */

static int onu_sn_set(const char *val, const struct kernel_param *kp)
{
	u8 parsed[8];
	bool changed;
	int ret;

	/* Validate before publishing either the string or the on-wire identity. */
	ret = gpon_sn_parse(val, parsed);
	if (ret || !gpon_sn_is_set(parsed))
		return -EINVAL;
	mutex_lock(&bosa_lock);
	changed = gpon_sn_differs(val);
	if (luna_stopping) {
		ret = -ESHUTDOWN;
		goto out;
	}
	if (changed && luna_driver_ready) {
		ret = luna_quiesce_locked();
		if (ret)
			goto out;
		/* Keep the old serial and model until its data ownership is gone.
		 * A failed retirement stays inhibited and retries on the next write. */
		ret = luna_data_retire();
		if (ret)
			goto out;
	}
	ret = param_set_charp(val, kp);
	if (ret)
		goto out;
	if (changed) {
		memcpy(gpon_sn_bytes, parsed, sizeof(parsed));
		gpon_sn_changed = true;
		gpon_ploam_set_sn(&luna_ploam, gpon_sn_bytes);
		luna_omci_identity_reset(gpon_sn_bytes);
	}
	/* Early parameters only latch identity. A failed late attempt is retryable
	 * with the same bytes; a healthy duplicate does not re-range. */
	if (luna_driver_ready)
		ret = luna_activate_locked(changed);
out:
	luna_resume_poll();
	mutex_unlock(&bosa_lock);
	return ret;
}
static const struct kernel_param_ops onu_sn_ops = {
	.set = onu_sn_set,
	.get = param_get_charp,
};
module_param_cb(onu_sn, &onu_sn_ops, &onu_sn, 0644);
MODULE_PARM_DESC(onu_sn, "ONU serial number (G.984.3 ONU-SN): 4 ASCII ID chars + 8 hex digits");
/* Invalidate unused alloc-CAM entries with the stock CLEAN operation before
 * management activation. Keep the existing parameter name for compatibility;
 * Alloc-ID 0xfff is assignable and cannot serve as an invalid entry value. */
static bool alloc_cam_park = true;
module_param(alloc_cam_park, bool, 0644);
MODULE_PARM_DESC(alloc_cam_park, "clean unused GTC allocation entries before management activation");

/* Diagnostic: skip the BOSA cold-init so that, on a warm boot where the BOSA is
 * already working, the SoC datapath and FSM run on top of it. */
static bool skip_bosa;
module_param(skip_bosa, bool, 0444);
MODULE_PARM_DESC(skip_bosa, "leave external BOSA as-is (warm-boot bisection)");

/* Route the 9607C I2C-indirect hole (0xB0-0xD8) via the SMI proxy? The proxy
 * currently reads back 0xffffffff (broken), so default OFF = direct MMIO, to
 * test whether the window decodes directly like the 9602C. */
static bool i2c_proxy;
module_param(i2c_proxy, bool, 0644);

/* Set once at init from the DT compatible: the RTL9607C shares the family
 * PON-MAC/GTC/PLOAM core but uses the c7 rev-C SerDes path, needs the rev>A
 * PON-IP power bit, and has NO external BOSA (internal SerDes front-end). */
static bool is_9607c;
/* The THIRD chip. Stock treats the RTL9603CVD as its own silicon -- its own DAL,
 * its own register table, PON SerDes at SWCORE +0x040000 -- and this tree already
 * carries its table; see the note at the assignment. */
static bool is_9603cvd;

/*
 * The SWCORE offsets that move between family members.
 *
 * The GPON GTC block has the same layout on all three chips, so the driver's
 * GPON_* offsets are shared. The SWCORE register file does NOT: the IO pad-mux,
 * the GPIO function-enable array and the whole PON SerDes bank sit at different
 * offsets per chip, and the SerDes moved by 0x1e000 between the 9602C and its two
 * siblings.
 *
 * Using one chip's literal on another does not fault -- the SWCORE window decodes
 * the whole range -- it silently reads or WRITES a different register. Measured
 * on the G24W 2026-08-26: the 9602C IO_MODE_EN literal 0x23018 is
 * EFUSE_BOND_CONTENT there, so the I2C_EN and OEM_EN writes landed in a
 * fuse-shadow register while the real IO_MODE_EN read with OEM_EN clear, i.e. the
 * optical pads were never enabled and every BOSA I2C transaction failed.
 * Likewise the 9602C IO_GPIO_EN literal 0x48 is CFG_PCSXF on the 9603CVD and 0x4c
 * is CFG_PHY_CTRL, so our GPIO write was corrupting the switch's PHY-address
 * base, which the Ethernet driver then repaired 6.3 s later -- invisible in a
 * resting-state dump.
 *
 * Every value below is from THAT chip's own register map, never transferred.
 */
struct gpon_swc_map {
	const char *chip;
	/* ★ THE OMCI FLOW/SID INDEX IS A PER-CHIP FACT, AND IT IS NOT THE SAME
	 * NUMBER ON THE TWO LUNA DIES (measured + RE'd 2026-09-08).  The vendor
	 * chipdef names it GPON_OMCI_FLOW_ID and declares 64 on the RTL9602C but
	 * 127 on the RTL9603CVD (and on the RTL9607C).  The G24W's own stock
	 * agrees, live at O5: `gpon show us-flow`/`ds-flow` report
	 * "Flow 127 | GEM 0 | OMCI | TCont 16".  This driver carried the RTL9602C
	 * literal for every chip, so on the RTL9603CVD the OMCC was installed at
	 * flow 64 while the OLT's downstream OMCI arrived on the flow the OMCI
	 * block watches -- 127 -- and OMCI_RX stayed 0 through a whole O5 window
	 * until the OLT's configuration timed out and it sent Deactivate_ONU-ID.
	 * The T-CONT (16) and the physical queue (0) are the SAME on both dies;
	 * only this index moves, which is why a table entry is the right shape. */
	u8  omcc_flow;		/* GPON_OMCI_FLOW_ID -- per-chip, never shared */
	/* ★ PON_SCH_QMAP's ENTRY WIDTH, and it is a WIDTH, not an address.  The
	 * vendor chipdef's "array offset" for PON_SCH_QMAP is 32 on the RTL9602C
	 * and 8 on the RTL9603CVD, so `base + tcont * 4` -- the shape this driver
	 * used everywhere -- addresses entry `tcont` on one die and entry
	 * `tcont * 4` on the other, four entries past the end of a 17-entry array
	 * for the OMCC's T-CONT 16.  entries_per_word = 32 / width is the rule
	 * pi_packed_locate() already implements.  32 is the identity case, so the
	 * chips that declare no different packing behave exactly as before. */
	u8  sch_qmap_bits;	/* PON_SCH_QMAP array-offset (bits per T-CONT entry) */
	/* ★ PON_SID_RPV_TH's ENTRY COUNT, and it is an ARRAY BOUND, not a SID
	 * count.  The per-SID reserved-page thresholds are 65 entries on the
	 * RTL9602C and only EIGHT on the RTL9603CVD (each chipdef's own "array
	 * index" range, tier 3), and the two are self-checking: 0x02458 + 65*4
	 * lands exactly on the RTL9602C's PONIP_DBG_CTRL_US and 0x026c8 + 8*4
	 * exactly on the RTL9603CVD's.  This driver looped CLASSIFY_SID_MAX(65)
	 * times with raw address arithmetic, so on the RTL9603CVD it wrote
	 * ON_TH=150/OFF_TH=130 into FIFTY-SEVEN registers PAST the array --
	 * PONIP_DBG_CTRL_US, PONIP_TOTAL_PAGE_CNT_US, PONIP_SID_USED_PAGE_CNT_US,
	 * GPON_DPRU_RPT_PRD, SID_OVER_STS/LATCH_STS, the whole 17-entry
	 * PON_SCH_RATE upstream shaper table, PON_EGR_RATE, US_SCH_EGR_IFG,
	 * PON_SCH_OPT, PONSCH_PIR_SIDOV, PON_SCH_ASIC_OPT and PON_DBRU_DBG_0/1.
	 * Nothing else in this driver writes any of those, so on that die their
	 * ONLY writer was this overrun.
	 * ⚠ THE VENDOR LOOPS 128 TIMES ON THIS CHIP AND IS STILL SAFE, which is
	 * the whole difference: its reg_array_field_write() resolves the index
	 * through the chipdef and returns RT_ERR_REG_ARRAY_INDEX_2 for anything
	 * past the declared range, so indices 8..127 never reach the bus.  We
	 * compute the address ourselves, so the bound has to be ours too.
	 * 0 = this chip has no PON-IP block at all (RTL9607C) -> write none. */
	u16 sid_rpv_entries;	/* PON_SID_RPV_TH declared entries, per chipdef */
	/* ★ THE US CLASSIFY INDEX SPACE, WHICH IS A DIFFERENT NUMBER AGAIN --
	 * "a count, a maximum, a stride and an index space are four different
	 * things".  The all-SID classify invalidation in
	 * rtl9602c_ponmac_modeset_gpon() must cover every SID the US-NIC can
	 * resolve, and that is the SMALLEST of what three chipdef facts allow:
	 * CLASSIFY_SID_MAX (65 on the RTL9602C, 128 on the RTL9603CVD and the
	 * RTL9607C), PON_SID2QID's declared entries (65 / 128) and PON_SIDVALID's
	 * (128 / 128).  The RTL9602C is the tight one and its 65 is exactly what
	 * this driver has always used; the RTL9603CVD's 128 was NOT, so on that
	 * die SIDs 65..127 were never invalidated -- and the loop's own
	 * "skip the OMCC" arm was dead, because that die's OMCC flow is 127 and
	 * the loop stopped at 64.
	 * Each pair self-checks against the next register: 0x020f8 + ceil(65/4)*4
	 * == 0x0213c (the RTL9602C's PON_SIDVALID) and 0x0210c + (128/4)*4 ==
	 * 0x0218c (the RTL9603CVD's), so the entry counts are arithmetic, not a
	 * reading of one field.
	 * ⚠ The RTL9607C's own def.h says 128 too, but its chipdef declares NO
	 * PON-IP block at all, so nothing establishes where those arrays are on
	 * that die -- it keeps the value it has always used and this comment says
	 * why rather than promoting an unverified 128. */
	u16 classify_sid_num;	/* US classify SIDs to invalidate, per chipdef */
	/* ★ A POINTER, NOT A SECOND COPY.  This file needs two switch-core facts
	 * -- which port is the PON, which is the CPU -- and the per-port register
	 * interval that turns a port number into an address.  All three already
	 * live, per chip, in luna_eth_regs.h's luna_sw_map, which the Ethernet
	 * shells use correctly.  luna_gpon.c is built for BOTH Luna boards and
	 * spelled the RTL9602C's ports 2 and 3 as literals at five sites; on the
	 * RTL9603CVD (PON 4, CPU 5, interval 0x100) those addressed P_MISC[8] and
	 * P_MISC[12], outside the six-port block, so RX_SPC never reached either
	 * port.  Re-typing the numbers here would have been the third copy; the
	 * pointer is the deduplicated form. */
	const struct luna_sw_map *sw;
	/* The OMCC's PHYSICAL queue id -- 64 / 127 / 120, see GPON_OMCC_PHYS_QID. */
	u8  omcc_phys_qid;
	/* ★★★ THE QUEUE THE INVALIDATION PRE-PASS PARKS EVERY OTHER SID ON.
	 * It must NOT be the OMCI queue: the vendor's flow2Queue_set REFUSES
	 * `physicalQid == 127 && flow != OMCI_FLOW_ID` on the RTL9603CVD
	 * (dal_rtl9603cvd_ponmac.c:2800, comment at :2332 "tcont 15 queue 7 can
	 * not be added, because omci use queue 127"), and its own pre-pass maps
	 * every flow to T-CONT 15 / queue 6 (:942-944).  Parking them on the
	 * OMCC's queue aliases any SID that becomes valid without its own qid
	 * write onto the MANAGEMENT stream. */
	u8  scratch_phys_qid;
	/* ★★★ THE T-CONT -> PHYSICAL QUEUE FORMULA, PER CHIP -- because the dies
	 * do not share it, and a 9602C literal served all three.  Each chip's own
	 * DAL, _ponmac_physicalQueueId_get (tier 3):
	 *   RTL9602C   phys = 32 * (tcont / 8) + q
	 *   RTL9607C   phys = 32 * (tcont / 8) + q
	 *   RTL9603CVD phys =  8 *  tcont      + q
	 * For the data T-CONT 8 that is 32 here and 64 there; writing 32 on the
	 * RTL9603CVD configures T-CONT 4's queue and leaves the data queue at its
	 * reset weight, so the data T-CONT's grants drain nothing.  Read through
	 * luna_tcont_phys_qid(), which is the only spelling. */
	u8  tcont_queue_max;
	u8  tcont_group;
	bool alloc_idx_swap;	/* RTL9603CVD logical T-CONT 4/7 and 5/11 permutation */
	/* ★★★ ACCEPT_MAX_LEN_CTRL IS NOT THE SAME FIELD ON THESE DIES, and the
	 * loop below wrote one chip's literal into all of them (found 2026-09-08
	 * from each chip's OWN chipdef, tier 3):
	 *   RTL9602C   MAX_LENGTH_10_100[0] + MAX_LENGTH_GIGA[1] -- two SELECTORS
	 *   RTL9603CVD ACCEPT_MAX_LENTH[13:0]                    -- a LENGTH
	 *   RTL9607C   ACCEPT_MAX_LENTH[13:0]                    -- a LENGTH
	 * So `[1:0] = 0x3` is right on the 9602C and means "accept at most THREE
	 * BYTES" on the 9603CVD. Our own 9607C path already writes 2031 into
	 * [13:0] (luna_ponmac.c:762), which is the vendor's value.
	 * @amax_msb/@amax_val carry the field and the value per chip.
	 * @amax_val is the value for every port EXCEPT the PON. */
	u8  amax_msb;
	u16 amax_val;
	/* ★★★ HOW FAR THE LOOP GOES, PER DIE AND PER EVIDENCE (2026-09-09).
	 * It used to stop at 3 on every die, which was right for the RTL9602C
	 * (four ports, PON=2 and CPU=3 both inside) and left ports 4..N unwritten
	 * on the wider dies.  It is now what THAT die's stock was MEASURED to
	 * write, and no further: raising it on a die nobody read would be a new
	 * write dressed as a widening. */
	u8  amax_last_port;
	/* ★★★ AND THE PON PORT TAKES IT TOO -- on the dies where it is a LENGTH,
	 * this register IS the PON link up/down. The vendor says so in its own
	 * words at rtk_gpon_portMacForceMode_set(): "use pon port accept packet
	 * length to implement port link up/down", writing 2031 for PORT_LINKUP
	 * and 0 for down (gponv2.c:3788-3798, via
	 * dal_rtl9603cvd_ponmac_maxPktLen_set -> ACCEPT_MAX_LEN_CTRL[PON]).
	 * The loop below stops at port 3, so on those dies the PON port keeps its
	 * RESET value and is never brought up by this mechanism at all.
	 * 0 on the 9602C, where the field is a selector pair and the PON port
	 * (2) simply takes @amax_val inside the loop like any other -- that die
	 * reaches O5 and carries WAN, so it is left exactly as it is. */
	u16 amax_pon_val;
	/* ★ THE DOWNSTREAM PBO's PAGE POOL, per chip.  0 = SRAM-only, which is
	 * what the RTL9602C's stock does and what this driver has always done;
	 * non-zero is the __get_free_pages order of a DRAM pool the DS PBO DMAs
	 * into, plus the three geometry words that describe it.  The RTL9603CVD's
	 * own stock runs the DRAM pool (see luna_gpon_regs.h), and without it the
	 * OLT's DS OMCI is dropped at descriptor-fetch with RX_DROP_CNT_DS
	 * climbing -- OMCI_RX and the trap stay 0 and the ONU looks silent. */
	u8  ds_dram_order;	/* pool order; 0 = keep the SRAM-only DS path       */
	u32 ds_dsc_cfg;		/* PON_DSC_CFG_DS   word, from that chip's stock     */
	u32 ds_dscrunout;	/* DSCRUNOUT_DS     word                            */
	u32 ds_fc_config;	/* PON_FC_CONFIG_DS word                            */
	u32 io_mode_en;		/* IO_MODE_EN                                  */
	u8  io_i2c_en_bus0;	/* IO_MODE_EN.I2C_EN low bit == I2C bus 0      */
	u8  io_oem_en;		/* IO_MODE_EN.OEM_EN bit (optical e-mode pads) */
	u32 io_gpio_en;		/* IO_GPIO_EN word 0; 0 = not declared here    */
	u32 sds_fib_status;	/* SDS_FIB_STATUS; 0 = not declared here       */
	u32 sds_reg0;		/* SDS_REG0    [1] SP_SDS_EN_RX                */
	u32 fib_reg16;		/* FIB_REG16   [10] FRC_SD [2] SEL_RX_SD       */
	u32 fib_ext_reg21;	/* FIB_EXT_REG21 [13] analog-ready             */
	u32 wsds_dig_18;	/* WSDS_DIG_18 [15:12] optic-LOS force + BEN_OE */
	/* The indirect I2C master, whose whole block sits +4 on the RTL9607C.
	 * It is in this table and NOT a ternary at the call sites because the
	 * ternary form was written ONCE, in the DDM read path, and the BOSA
	 * WRITE path a few hundred lines below kept the bare 9602C offsets --
	 * so on a 9607C the laser writes would have gone to the wrong three
	 * registers with nothing to read back and no error. Each value is from
	 * that chip's OWN register map. */
	u32 i2c_ind_wd;		/* I2C_IND_WD  [31:0] write data               */
	u32 i2c_ind_adr;	/* I2C_IND_ADR [31:0] target reg offset        */
	u32 i2c_ind_cmd;	/* I2C_IND_CMD [0]CMD_EN [1]RW_EN [2]BUSY [3]NACK */
	u32 i2c_ind_rd;		/* I2C_IND_RD  [31:0] read data                */
	/* The SerDes window base. MEASURED from the three chipdefs 2026-08-31:
	 * EVERY SerDes register moves by exactly +0x1e000 between the 9602C and
	 * the other two -- 215 registers on the 9603CVD, 245 on the 9607C, not
	 * one exception. So the window is ONE fact, and a new chip costs one
	 * number here rather than a fresh literal per register. */
	u32 sds_win;		/* base of the SDS_ANA_* / WSDS_DIG_* window   */
	const struct luna_pi_move *pi_moves;	/* PON-IP relocations, NULL = none */
	/* SDS_CFG is NOT in that window and does NOT follow it: it is 0x1d0 /
	 * 0x200 / 0x270 on the three chips, so it gets its own field. */
	u32 sds_cfg;		/* SDS_CFG [4:0] SDS_MODE                      */
	/* Base of the six contiguous OMCI packet counters.
	 *
	 * ⚠ AND IT IS A BASE, NOT A WINDOW, because the surrounding block is
	 * NOT uniform: measured across the three chipdefs, 25 registers in
	 * 0x32xxx move by +0x580 while seven STAT_* MIB pointers move by
	 * +0x0, +0x400, +0x600, +0x680, +0x6c0, +0x800 and +0x8d0. Treating
	 * that block the way the SerDes window is treated would mis-map every
	 * one of the seven. The six OMCI counters ARE contiguous on all three
	 * chips (base +0x00,04,08,0c,10,14), which is what makes one number
	 * enough for them and only them. */
	u32 omci_cnt;		/* OMCI_DROP_PKT_CNT, the first of six         */
	/* ── the IRREGULAR movers ──────────────────────────────────────────
	 * Each of these moves by its OWN delta between the three chips, so
	 * there is no window and no contiguous block to anchor them: a field
	 * each is the only honest encoding. Every value is from THAT chip's
	 * own register map.
	 *
	 * They were BARE HEX LITERALS in the code, which is why nothing found
	 * them for so long -- the guard resolved a moved register only through
	 * a NAME, and a literal has none.
	 *
	 * ★ READ LUT_SYS_LRN_LIMIT FIRST: 0x1c02c on the 9602C and 0x1c014 on
	 * both others -- it moves BACKWARDS, into what is UNKN_IP4_MC on the
	 * 9602C. A wrong write there does not fault; it changes multicast
	 * handling on a different register. */
	u32 sc_ind_wd;               /* SC_IND_WD */
	u32 wrap_gphy_misc;          /* WRAP_GPHY_MISC */
	u32 thermal_ctrl_0;          /* THERMAL_CTRL_0 */
	u32 force_p_ablty;           /* FORCE_P_ABLTY */
	u32 ablty_force_mode;        /* ABLTY_FORCE_MODE */
	u32 dyngasp_ctrl;            /* DYNGASP_CTRL */
	u32 accept_max_len_ctrl;     /* ACCEPT_MAX_LEN_CTRL */
	u32 pon_trap_cfg;            /* PON_TRAP_CFG */
	u32 cf_cfg;                  /* CF_CFG -- ⚠ NOT IN 9603cvd's map */
	u32 lut_unkn_uc_da_ctrl;     /* LUT_UNKN_UC_DA_CTRL */
	u32 lut_learn_over_ctrl;     /* LUT_LEARN_OVER_CTRL */
	u32 unkn_l2_mc;              /* UNKN_L2_MC */
	u32 unkn_ip4_mc;             /* UNKN_IP4_MC */
	u32 unkn_mc_cfg;             /* UNKN_MC_CFG */
	u32 lut_bc_flood;            /* LUT_BC_FLOOD */
	u32 lut_unkn_mc_flood;       /* LUT_UNKN_MC_FLOOD */
	u32 lut_unkn_uc_flood;       /* LUT_UNKN_UC_FLOOD */
	u32 lut_sys_lrn_limit;       /* LUT_SYS_LRN_LIMIT */
	u32 rma_ctrl01;              /* RMA_CTRL01 */
	u32 rma_ctrl02;              /* RMA_CTRL02 */
	u32 rma_cfg;                 /* RMA_CFG */
	u32 qos_uni_trap_pri_ctrl;   /* QOS_UNI_TRAP_PRI_CTRL */
	u32 oam_ctrl_0;              /* OAM_CTRL_0 */
	u32 cfg_unhiol;              /* CFG_UNHIOL */
};

/* The PON-IP relocation table's element type, declared HERE because the chip
 * map below points at one. The table itself is generated beside pi_x(). */
struct luna_pi_move { u32 from, to; };
extern const struct luna_pi_move luna_pi_moves_9603cvd[];

static const struct gpon_swc_map gpon_swc_9602c = {
	.chip = "RTL9602C",
	.omcc_flow = GPON_OMCC_FLOW_9602C,
	.ds_dram_order = 0,	/* SRAM-only DS, as this chip's stock does */
	.sch_qmap_bits = 32,	/* chipdef array offset 32 */
	.sid_rpv_entries = 65,	/* chipdef array index 0..64 */
	.classify_sid_num = 65,	/* CLASSIFY_SID_MAX 65 == PON_SID2QID's 65 entries */
	.sw = &rtl9602c_sw_map,
	.omcc_phys_qid = 64,	/* TCONT_QUEUE_MAX(32)*(16/8)+0 */
	/* ⚠ THE SAME QUEUE AS THE OMCC, AND DELIBERATELY UNCHANGED. That is what
	 * this driver has always written on this die, and this die reaches O5
	 * and carries WAN today. Moving it would be an unproven behaviour change
	 * on the one board that works -- the defect being repaired is on the
	 * 9603CVD, where the vendor forbids the write outright. OWED: the
	 * 9602C's own scratch choice, from its DAL. */
	.scratch_phys_qid = 64,
	.tcont_queue_max = 32, .tcont_group = 8,	/* data T-CONT 8 -> qid 32 */
	/* four ports (PON=2, CPU=3): 0..3 already covers every one of them,
	 * and the PON takes @amax_val like the rest. UNCHANGED. */
	.amax_msb = 1, .amax_val = 0x3, .amax_pon_val = 0,  /* two SELECTORS */
	.amax_last_port = 3,
	.io_mode_en = 0x23018, .io_i2c_en_bus0 = 13, .io_oem_en = 19,
	.io_gpio_en = 0x00048,
	.sds_fib_status = 0x001e4,
	.sds_reg0 = 0x22800, .fib_reg16 = 0x22c40, .fib_ext_reg21 = 0x22e54,
	.wsds_dig_18 = 0x22090,
	.i2c_ind_wd = 0x000b0, .i2c_ind_adr = 0x000b8,
	.i2c_ind_cmd = 0x000c0, .i2c_ind_rd = 0x000c8,
	.sds_win = 0x22000, .sds_cfg = 0x001d0,
	.omci_cnt = 0x329b8,
	.sc_ind_wd = 0x0003c,
	.wrap_gphy_misc = 0x00110,
	.thermal_ctrl_0 = 0x00130,
	.force_p_ablty = 0x00180,
	.ablty_force_mode = 0x001b4,
	.dyngasp_ctrl = 0x001ec,
	.accept_max_len_ctrl = 0x11008,
	.pon_trap_cfg = 0x111f8,
	.cf_cfg = 0x1600c,
	.lut_unkn_uc_da_ctrl = 0x1c008,
	.lut_learn_over_ctrl = 0x1c00c,
	.unkn_l2_mc = 0x1c010,
	.unkn_ip4_mc = 0x1c014,
	.unkn_mc_cfg = 0x1c01c,
	.lut_bc_flood = 0x1c020,
	.lut_unkn_mc_flood = 0x1c024,
	.lut_unkn_uc_flood = 0x1c028,
	.lut_sys_lrn_limit = 0x1c02c,
	.rma_ctrl01 = 0x1c03c,
	.rma_ctrl02 = 0x1c040,
	.rma_cfg = 0x1c084,
	.qos_uni_trap_pri_ctrl = 0x1c0cc,
	.oam_ctrl_0 = 0x1c100,
	.cfg_unhiol = 0x23040,
};

static const struct gpon_swc_map gpon_swc_9603cvd = {
	.chip = "RTL9603CVD",
	.alloc_idx_swap = true,
	.omcc_flow = GPON_OMCC_FLOW_9603CVD,
	/* the DS PBO DRAM pool, from the G24W's own stock live at O5 */
	.ds_dram_order = PI_DS_DRAM_ORDER,
	.ds_dsc_cfg = 0x1fff8007u, .ds_dscrunout = 0x1fcb0006u,
	.ds_fc_config = 0x1f950036u,
	.sch_qmap_bits = 8,	/* chipdef array offset 8 -- 4 entries per word */
	.sid_rpv_entries = 8,	/* chipdef array index 0..7 -- NOT 65 */
	.classify_sid_num = 128,	/* CLASSIFY_SID_MAX 128 == PON_SID2QID/SIDVALID entries */
	.sw = &rtl9603cvd_sw_map,
	.omcc_phys_qid = 127,	/* the DAL's OMCI special case, NOT the formula */
	/* T-CONT 15 / queue 6, the vendor's own pre-pass choice:
	 * TCONT_QUEUE_MAX(8)*15 + 6 = 126. Queue 7 there would be 127, which the
	 * DAL refuses because that IS the OMCI queue. */
	.scratch_phys_qid = 126,
	.tcont_queue_max = 8, .tcont_group = 1,	/* data T-CONT 8 -> qid 64 */
	/* ★★★ [T1] MEASURED ON STOCK SILICON, all six ports, G24W 2026-09-09
	 * (`diag register get all`, 0x1100C + port*4):
	 *   port 0..3 (LAN) = 16368 (0x3FF0)   port 4 (PON) = 2031 (0x7EF)
	 *   port 5 (CPU)    = 16368 (0x3FF0)
	 * We wrote 2031 into the LAN ports, which is EIGHT TIMES NARROWER than
	 * the firmware we replace -- stock accepts up to 16368 B there and ours
	 * dropped anything over 2031.  That is a capability regression on the
	 * ports that carry customer traffic, and it is invisible in normal
	 * traffic because 2031 still clears a 1522 B tagged frame.
	 * ⚠ AND IT WAS INVISIBLE TO THE DIFF TOO: 0x1100C is outside every block
	 * this board declared, so `swcore_diff` had never captured the register
	 * on either firmware.  See swcore_blocks_from_capture.py.
	 * The measurement also CONFIRMS pon_port=4 independently: port 4 is the
	 * only one carrying the vendor's PORT_LINKUP literal. */
	.amax_msb = 13, .amax_val = 16368, .amax_pon_val = 2031,  /* a LENGTH */
	.amax_last_port = 5,	/* six ports, and stock writes all six */
	.io_mode_en = 0x23014, .io_i2c_en_bus0 = 11, .io_oem_en = 16,
	.io_gpio_en = 0x0003c,
	.sds_fib_status = 0x00214,
	.sds_reg0 = 0x40800, .fib_reg16 = 0x40c40, .fib_ext_reg21 = 0x40e54,
	.wsds_dig_18 = 0x40090,
	.i2c_ind_wd = 0x000b0, .i2c_ind_adr = 0x000b8,
	.i2c_ind_cmd = 0x000c0, .i2c_ind_rd = 0x000c8,
	.sds_win = 0x40000, .sds_cfg = 0x00200,
	/* 16 PON-IP registers relocate on this chip -- see pi_x(). Derived from
	 * this chip's OWN chipdef, and one of them is already confirmed by the
	 * board's stock-vs-ours capture. */
	.pi_moves = luna_pi_moves_9603cvd,
	.omci_cnt = 0x32f38,
	.sc_ind_wd = 0x00028,
	.wrap_gphy_misc = 0x000ec,
	.thermal_ctrl_0 = 0x0010c,
	.force_p_ablty = 0x00198,
	.ablty_force_mode = 0x001dc,
	.dyngasp_ctrl = 0x0021c,
	.accept_max_len_ctrl = 0x1100c,
	.pon_trap_cfg = 0x110ec,
	.cf_cfg = 0x1600c,
	.lut_unkn_uc_da_ctrl = 0x1c00c,
	.lut_learn_over_ctrl = 0x1c010,
	.unkn_l2_mc = 0x1c018,
	.unkn_ip4_mc = 0x1c01c,
	.unkn_mc_cfg = 0x1c024,
	.lut_bc_flood = 0x1c028,
	.lut_unkn_mc_flood = 0x1c02c,
	.lut_unkn_uc_flood = 0x1c030,
	.lut_sys_lrn_limit = 0x1c014,
	.rma_ctrl01 = 0x1c064,
	.rma_ctrl02 = 0x1c068,
	.rma_cfg = 0x1c0ac,
	.qos_uni_trap_pri_ctrl = 0x1c214,
	.oam_ctrl_0 = 0x1c254,
	.cfg_unhiol = 0x2304c,
};

static const struct gpon_swc_map gpon_swc_9607c = {
	.chip = "RTL9607C",
	.omcc_flow = GPON_OMCC_FLOW_9607C,
	.ds_dram_order = 0,	/* not measured on this chip -- SRAM-only, unchanged */
	.sch_qmap_bits = 32,	/* no PON-IP block in this chipdef; keep the identity packing */
	.sid_rpv_entries = 0,	/* no PON-IP block in this chipdef */
	.classify_sid_num = 65,	/* UNESTABLISHED: no PON-IP block in this chipdef; unchanged */
	.sw = &rtl9607c_sw_map,
	.omcc_phys_qid = 120,	/* TCONT_QUEUE_MAX(32)*(31/8)+24 */
	/* ⚠ OWED: unchanged from what the code wrote before the per-chip split,
	 * and no evidence either way for this die. Not guessed. */
	.scratch_phys_qid = 120,
	/* ⚠ OWED: unchanged from the single #define, no evidence for this die. */
	.tcont_queue_max = 32, .tcont_group = 8,
	/* ⚠ OWED: this die has NOT been read. The values below reproduce exactly
	 * what the old loop wrote (0..3 plus the PON at 5), so nothing about this
	 * chip changes here. Closing it needs the same measurement the RTL9603CVD
	 * just had: `diag register get all` on ITS stock, ports 0..9. */
	.amax_msb = 13, .amax_val = 2031, .amax_pon_val = 2031,  /* a LENGTH */
	.amax_last_port = 3,
	.io_mode_en = 0x23014, .io_i2c_en_bus0 = 13, .io_oem_en = 19,
	/* IO_GPIO_EN is 0x38 here, but this chip's optical front-end is internal
	 * and none of the 9602C pad recipe applies, so it is left undeclared:
	 * nothing may write a GPIO pad-enable word on the 9607C. */
	.io_gpio_en = 0,
	/* SDS_FIB_STATUS is a 3-lane ARRAY here (0x28c, stride 0x20); lane 0 is
	 * the one the GPON RX uses. ddm_probe_9607c() scans all three. */
	.sds_fib_status = 0x0028c,
	.sds_reg0 = 0x40800, .fib_reg16 = 0x40c40, .fib_ext_reg21 = 0x40e54,
	.wsds_dig_18 = 0x40090,
	.i2c_ind_wd = 0x000b4, .i2c_ind_adr = 0x000bc,
	.i2c_ind_cmd = 0x000c4, .i2c_ind_rd = 0x000cc,
	.sds_win = 0x40000, .sds_cfg = 0x00270,
	.omci_cnt = 0x32f38,
	.sc_ind_wd = 0x00024,
	.wrap_gphy_misc = 0x00114,
	.thermal_ctrl_0 = 0x00150,
	.force_p_ablty = 0x001cc,
	.ablty_force_mode = 0x00238,
	.dyngasp_ctrl = 0x0029c,
	.accept_max_len_ctrl = 0x11028,
	.pon_trap_cfg = 0x11144,
	.cf_cfg = 0x16004,
	.lut_unkn_uc_da_ctrl = 0x1c00c,
	.lut_learn_over_ctrl = 0x1c010,
	.unkn_l2_mc = 0x1c018,
	.unkn_ip4_mc = 0x1c01c,
	.unkn_mc_cfg = 0x1c024,
	.lut_bc_flood = 0x1c028,
	.lut_unkn_mc_flood = 0x1c02c,
	.lut_unkn_uc_flood = 0x1c030,
	.lut_sys_lrn_limit = 0x1c014,
	.rma_ctrl01 = 0x1c0c8,
	.rma_ctrl02 = 0x1c0cc,
	.rma_cfg = 0x1c110,
	.qos_uni_trap_pri_ctrl = 0x1c2a8,
	.oam_ctrl_0 = 0x1c2ec,
	.cfg_unhiol = 0x23104,
};

/* Resolved from the DT compatible in probe(), BEFORE any sw_rd/sw_wr of a
 * chip-selected offset. Defaults to the 9602C so a boot on an undeclared board
 * behaves exactly as this driver did before the table existed. */
static const struct gpon_swc_map *swc = &gpon_swc_9602c;

/*
 * The physical queue a T-CONT drains its logical queue 0 on.
 *
 * ★ THE FORMULA IS PER DIE and the OMCC is not computed by it at all: the
 * RTL9603CVD's DAL assigns the OMCI flow queue 127 outright and then REFUSES
 * that queue for every other flow, so it comes from the table.  Everything
 * else follows that chip's own _ponmac_physicalQueueId_get (tier 3).
 */
static u8 luna_tcont_phys_qid(u8 tcont)
{
	if (tcont == GPON_OMCC_TCONT)
		return swc->omcc_phys_qid;
	return (u8)(swc->tcont_queue_max * (tcont / swc->tcont_group));
}

/* Own G24 stock tcont_{get,set,del} all apply this permutation to the alloc
 * CAM index. Scheduler indices remain logical; X111 uses the identity map. */
static u8 luna_tcont_cam_index(u8 tcont)
{
	if (!swc->alloc_idx_swap)
		return tcont;
	switch (tcont) {
	case 4: return 7;
	case 7: return 4;
	case 5: return 11;
	case 11: return 5;
	default: return tcont;
	}
}

/*
 * ★ THE UPSTREAM-OPTICS OPERATING VALUES ARE A PER-CHIP TABLE, NOT LITERALS.
 * Three GTC words that the family shell programs identically on every Luna
 * chip but whose VALUES stock chooses per silicon:
 *
 *   GTC_US_LASER      [13:8] LON_TIME / [5:0] LOFF_TIME, the laser-enable
 *                     edges around each burst.
 *   GTC_US_OPTIC_SD_TH [30:16] MISM_THRESH / [14:0] TOOLONG_THRESH, how long
 *                     the laser-driver's TX_SD may lag before the GTC judges
 *                     its own burst "too long" and gates it.
 *   GTC_US_PWR_SAV_MODE bit16 DG_TX_OPT (dying-gasp TX option), beside the
 *                     bit0 PWR_SAV_MODE every chip sets.
 *
 * RTL9602C: the values this driver has always written, taken from the X111W's
 * live stock (US_OPTIC_SD_TH 0x00504bfa read at O5 2026-06-13; US_LASER
 * 0x2028).  RTL9603CVD: its OWN stock read live at O5 on the LANLY G24W
 * (2026-09-06, gpon-oracle/2026-09-06.2/gtc-regs.json): 0x0504c=0x00001820,
 * 0x05188=0x00a07fff, 0x0526c=0x00010001 -- and the same numbers are the
 * vendor SDK's 9603CVD fallbacks (laser on 0x18 / off 0x20; MISM 0xa0 /
 * TOOLONG 0x7fff), two tiers agreeing.  The 9607C keeps the 9602C values it
 * has always run with; nothing was measured there.
 */
struct luna_gtc_tune {
	const char *chip;
	u32 us_laser;
	u32 us_optic_sd_th;
	bool us_pwr_sav_dg_tx_opt;
};

static const struct luna_gtc_tune luna_gtc_tune_9602c = {
	.chip = "RTL9602C", .us_laser = 0x2028u, .us_optic_sd_th = 0x00504bfau,
	.us_pwr_sav_dg_tx_opt = false,
};

static const struct luna_gtc_tune luna_gtc_tune_9603cvd = {
	.chip = "RTL9603CVD", .us_laser = 0x1820u, .us_optic_sd_th = 0x00a07fffu,
	.us_pwr_sav_dg_tx_opt = true,
};

static const struct luna_gtc_tune *gtune = &luna_gtc_tune_9602c;

/* Keep the register NAMES at the call sites; the value is now per chip. */
#define SOC_IO_MODE_EN		(swc->io_mode_en)
#define IO_I2C_EN_BUS0		(swc->io_i2c_en_bus0)
#define IO_OEM_EN		(1u << (swc)->io_oem_en)
#define SOC_IO_GPIO_EN		(swc->io_gpio_en)
#define SDS_FIB_STATUS		(swc->sds_fib_status)
#define SDS_REG0		(swc->sds_reg0)
#define FIB_REG16		(swc->fib_reg16)
#define FIB_EXT_REG21		(swc->fib_ext_reg21)
#define WSDS_DIG_18		(swc->wsds_dig_18)
/* These four are DEFINED IN luna_gpon_regs.h with the 9602C's values, which are
 * right for two of the three chips and silently wrong for the RTL9607C. Undef
 * and re-point at the table so BOTH the read and the write path get the same
 * per-chip answer -- the write path had never been corrected. */
#undef  I2C_IND_WD
#undef  I2C_IND_ADR
#undef  I2C_IND_CMD
#undef  I2C_IND_RD
#define I2C_IND_WD		(swc->i2c_ind_wd)
#define I2C_IND_ADR		(swc->i2c_ind_adr)
#define I2C_IND_CMD		(swc->i2c_ind_cmd)
#define I2C_IND_RD		(swc->i2c_ind_rd)

/* The SerDes window and SDS_CFG, from the table for the same reason the I2C
 * block is: luna_gpon_regs.h holds the 9602C's absolute addresses, which are
 * wrong by 0x1e000 on both other chips. SDS() is written with the 9602C
 * address the header already carries, so each line stays greppable against it. */
#define SDS(a9602c)		(swc->sds_win + ((a9602c) - 0x22000u))

/*
 * The PON-IP block moves between the Luna chips, and NOT by a window. Every PI_*
 * in luna_gpon_regs.h is a bare RTL9602C literal, and sixteen of the ones this
 * driver USES sit at a different offset on the RTL9603CVD with irregular deltas,
 * so unlike SDS() no single constant can translate them: the map is a TABLE,
 * derived from each chip's own vendor chipdef by ONU-test-case/ponip_offsets.py.
 *
 * Not a theory -- the board's own stock-vs-ours capture caught one:
 * PI_PON_US_FIFO_CTL is 0x020f0, which on the RTL9603CVD is ARB_TIMEOUT_US, and
 * ours holds 0x13 there where stock holds 0x4, while stock's 0x13 sits at
 * 0x02104, the real PON_US_FIFO_CTL, which ours leaves at 0.
 *
 * The translation is AT THE MACRO, not inside pi_rd/pi_wr, and that is
 * load-bearing: one call site computes an address arithmetically
 * (PI_PON_SID_RPV_TH + sid * stride), and translating inside the accessor would
 * receive base+stride, fail to find it and pass it through SILENTLY. Worse,
 * 0x0a0cc is BOTH a source on one die and a destination on the other, so a
 * translation applied twice would land somewhere real and wrong.
 */
/* GENERATED by ONU-test-case/ponip_offsets.py --emit -- do not edit by hand. */
const struct luna_pi_move luna_pi_moves_9603cvd[] = {
	{ 0x020e4, 0x020f8 },	/* PI_DRN_CMD */
	{ 0x0a0b4, 0x0a0cc },	/* PI_DSCRUNOUT_DS */
	{ 0x020e0, 0x020f4 },	/* PI_DSCRUNOUT_US */
	{ 0x02568, 0x026f4 },	/* PI_GPON_DPRU_RPT_PRD */
	{ 0x0a0b8, 0x0a0d0 },	/* PI_IP_MSTBASE_DS */
	{ 0x020e8, 0x020fc },	/* PI_IP_MSTBASE_US */
	{ 0x02170, 0x021c0 },	/* PI_MOCIR_FRC_MD */
	{ 0x02174, 0x021c4 },	/* PI_MOCIR_FRC_VAL */
	{ 0x02184, 0x021d4 },	/* PI_MOCIR_TH_H */
	{ 0x02188, 0x021d8 },	/* PI_MOCIR_TH_L */
	{ 0x0a0ac, 0x0a0c0 },	/* PI_PONIP_CTL_DS */
	{ 0x020d8, 0x020ec },	/* PI_PONIP_CTL_US */
	{ 0x0255c, 0x026e8 },	/* PI_PONIP_DBG_CTRL_US */
	{ 0x02578, 0x02708 },	/* PI_PONIP_SID_OVER_LATCH_STS */
	{ 0x0256c, 0x026f8 },	/* PI_PONIP_SID_OVER_STS */
	{ 0x02564, 0x026f0 },	/* PI_PONIP_SID_USED_PAGE_CNT_US */
	{ 0x02560, 0x026ec },	/* PI_PONIP_TOTAL_PAGE_CNT_US */
	{ 0x02150, 0x021a0 },	/* PI_PON_BW_THRES */
	{ 0x0a0cc, 0x0a0e4 },	/* PI_PON_DSC_CFG_DS */
	{ 0x0215c, 0x021ac },	/* PI_PON_DSC_CFG_US */
	{ 0x0a0c8, 0x0a0e0 },	/* PI_PON_DSC_STS_DS */
	{ 0x02158, 0x021a8 },	/* PI_PON_DSC_STS_US */
	{ 0x0a0bc, 0x0a0d4 },	/* PI_PON_DSC_USAGE_DS */
	{ 0x020ec, 0x02100 },	/* PI_PON_DSC_USAGE_US */
	{ 0x0a100, 0x0a114 },	/* PI_PON_DS_PBO_PAGE_Q0 */
	{ 0x0a0fc, 0x0a110 },	/* PI_PON_FC_CONFIG_DS */
	{ 0x0a0c0, 0x0a0d8 },	/* PI_PON_IPSTS_DS */
	{ 0x020f4, 0x02108 },	/* PI_PON_IPSTS_US */
	{ 0x0218c, 0x021dc },	/* PI_PON_OLT_BW_MTR_FULL */
	{ 0x02154, 0x021a4 },	/* PI_PON_OMCI_CFG */
	{ 0x02198, 0x021e8 },	/* PI_PON_QID_CIR_RATE */
	{ 0x0229c, 0x023e8 },	/* PI_PON_QID_PIR_RATE */
	{ 0x02194, 0x021e4 },	/* PI_PON_SCH_CTRL */
	{ 0x025d8, 0x0276c },	/* PI_PON_SCH_OPT */
	{ 0x023a0, 0x025e8 },	/* PI_PON_SCH_QMAP */
	{ 0x020f8, 0x0210c },	/* PI_PON_SID2QID */
	{ 0x0213c, 0x0218c },	/* PI_PON_SIDVALID */
	{ 0x02454, 0x026c4 },	/* PI_PON_SID_GLB_TH */
	{ 0x0a0e4, 0x0a0ec },	/* PI_PON_SID_Q_MAP_DS */
	{ 0x02458, 0x026c8 },	/* PI_PON_SID_RPV_TH */
	{ 0x02450, 0x026c0 },	/* PI_PON_SID_STOP_TH */
	{ 0x02190, 0x021e0 },	/* PI_PON_TB_CTRL */
	{ 0x023e4, 0x025fc },	/* PI_PON_TCONT_EN */
	{ 0x020f0, 0x02104 },	/* PI_PON_US_FIFO_CTL */
	{ 0x023e8, 0x02600 },	/* PI_PON_WFQ_TYPE */
	{ 0x023f8, 0x02614 },	/* PI_PON_WFQ_WEIGHT */
	{ 0x0a10c, 0x0a130 },	/* PI_RSVD_PONIP_DS */
	{ 0, 0 },
};

static u32 pi_x(u32 a9602c)
{
	const struct luna_pi_move *m = swc->pi_moves;

	if (!m)
		return a9602c;
	for (; m->from; m++)
		if (m->from == a9602c)
			return m->to;
	return a9602c;
}

#define PI_X(a9602c)		pi_x(a9602c)

/* ★ THE 9602C ADDRESS IS WRITTEN ONCE, IN THE HEADER (2026-09-04).
 * This block used to re-type each address beside the name it redefines, so
 * every PON-IP register had TWO copies of its address and nothing compared
 * them: editing luna_gpon_regs.h alone would have left the driver reading the
 * stale one, silently. (They did all agree when this was written -- 25 of 25
 * -- which is exactly when to remove the hazard rather than add a checker.)
 * The enum captures the header's value BEFORE the #undef, so the redefinition
 * can reach it without naming a number.  Same constants, one owner. */
enum {
	PI_A_DSCRUNOUT_DS = PI_DSCRUNOUT_DS,
	PI_A_DSCRUNOUT_US = PI_DSCRUNOUT_US,
	PI_A_IP_MSTBASE_US = PI_IP_MSTBASE_US,
	PI_A_IP_MSTBASE_DS = PI_IP_MSTBASE_DS,
	PI_A_PONIP_CTL_DS = PI_PONIP_CTL_DS,
	PI_A_PONIP_CTL_US = PI_PONIP_CTL_US,
	PI_A_PON_DSC_CFG_DS = PI_PON_DSC_CFG_DS,
	PI_A_PON_DSC_CFG_US = PI_PON_DSC_CFG_US,
	PI_A_PON_FC_CONFIG_DS = PI_PON_FC_CONFIG_DS,
	PI_A_PON_OMCI_CFG = PI_PON_OMCI_CFG,
	PI_A_DRN_CMD = PI_DRN_CMD,
	PI_A_PON_OLT_BW_MTR_FULL = PI_PON_OLT_BW_MTR_FULL,
	PI_A_PON_WFQ_TYPE = PI_PON_WFQ_TYPE,
	PI_A_RSVD_PONIP_DS = PI_RSVD_PONIP_DS,
	PI_A_PON_DSC_USAGE_DS = PI_PON_DSC_USAGE_DS,
	PI_A_PON_IPSTS_US = PI_PON_IPSTS_US,
	PI_A_PON_DSC_USAGE_US = PI_PON_DSC_USAGE_US,
	PI_A_PON_IPSTS_DS = PI_PON_IPSTS_DS,
	PI_A_PON_DSC_STS_DS = PI_PON_DSC_STS_DS,
	PI_A_PON_SID2QID = PI_PON_SID2QID,
	PI_A_PON_SIDVALID = PI_PON_SIDVALID,
	PI_A_PON_SID_GLB_TH = PI_PON_SID_GLB_TH,
	PI_A_PON_SID_Q_MAP_DS = PI_PON_SID_Q_MAP_DS,
	PI_A_PON_SID_RPV_TH = PI_PON_SID_RPV_TH,
	PI_A_PON_SID_STOP_TH = PI_PON_SID_STOP_TH,
	PI_A_PON_US_FIFO_CTL = PI_PON_US_FIFO_CTL,
	PI_A_PON_BW_THRES = PI_PON_BW_THRES,
	PI_A_PON_DSC_STS_US = PI_PON_DSC_STS_US,
	PI_A_MOCIR_FRC_MD = PI_MOCIR_FRC_MD,
	PI_A_MOCIR_FRC_VAL = PI_MOCIR_FRC_VAL,
	PI_A_MOCIR_TH_H = PI_MOCIR_TH_H,
	PI_A_MOCIR_TH_L = PI_MOCIR_TH_L,
	PI_A_PON_TB_CTRL = PI_PON_TB_CTRL,
	PI_A_PON_SCH_CTRL = PI_PON_SCH_CTRL,
	PI_A_PON_QID_CIR_RATE = PI_PON_QID_CIR_RATE,
	PI_A_PON_QID_PIR_RATE = PI_PON_QID_PIR_RATE,
	PI_A_PON_SCH_QMAP = PI_PON_SCH_QMAP,
	PI_A_PON_TCONT_EN = PI_PON_TCONT_EN,
	PI_A_PON_WFQ_WEIGHT = PI_PON_WFQ_WEIGHT,
	PI_A_PONIP_DBG_CTRL_US = PI_PONIP_DBG_CTRL_US,
	PI_A_PONIP_TOTAL_PAGE_CNT_US = PI_PONIP_TOTAL_PAGE_CNT_US,
	PI_A_PONIP_SID_USED_PAGE_CNT_US = PI_PONIP_SID_USED_PAGE_CNT_US,
	PI_A_GPON_DPRU_RPT_PRD = PI_GPON_DPRU_RPT_PRD,
	PI_A_PONIP_SID_OVER_STS = PI_PONIP_SID_OVER_STS,
	PI_A_PON_SCH_OPT = PI_PON_SCH_OPT,
	PI_A_PON_DS_PBO_PAGE_Q0 = PI_PON_DS_PBO_PAGE_Q0,
	PI_A_PONIP_SID_OVER_LATCH_STS = PI_PONIP_SID_OVER_LATCH_STS,
};

#undef  PI_DSCRUNOUT_DS
#define PI_DSCRUNOUT_DS	PI_X(PI_A_DSCRUNOUT_DS)
#undef  PI_DSCRUNOUT_US
#define PI_DSCRUNOUT_US	PI_X(PI_A_DSCRUNOUT_US)
#undef  PI_IP_MSTBASE_US
#define PI_IP_MSTBASE_US	PI_X(PI_A_IP_MSTBASE_US)
#undef  PI_IP_MSTBASE_DS
#define PI_IP_MSTBASE_DS	PI_X(PI_A_IP_MSTBASE_DS)
#undef  PI_PONIP_CTL_DS
#define PI_PONIP_CTL_DS	PI_X(PI_A_PONIP_CTL_DS)
#undef  PI_PONIP_CTL_US
#define PI_PONIP_CTL_US	PI_X(PI_A_PONIP_CTL_US)
#undef  PI_PON_DSC_CFG_DS
#define PI_PON_DSC_CFG_DS	PI_X(PI_A_PON_DSC_CFG_DS)
#undef  PI_PON_DSC_CFG_US
#define PI_PON_DSC_CFG_US	PI_X(PI_A_PON_DSC_CFG_US)
#undef  PI_PON_FC_CONFIG_DS
#define PI_PON_FC_CONFIG_DS	PI_X(PI_A_PON_FC_CONFIG_DS)
#undef  PI_PON_OMCI_CFG
#define PI_PON_OMCI_CFG	PI_X(PI_A_PON_OMCI_CFG)
#undef  PI_DRN_CMD
#define PI_DRN_CMD	PI_X(PI_A_DRN_CMD)
#undef  PI_PON_OLT_BW_MTR_FULL
#define PI_PON_OLT_BW_MTR_FULL	PI_X(PI_A_PON_OLT_BW_MTR_FULL)
#undef  PI_PON_WFQ_TYPE
#define PI_PON_WFQ_TYPE	PI_X(PI_A_PON_WFQ_TYPE)
#undef  PI_RSVD_PONIP_DS
#define PI_RSVD_PONIP_DS	PI_X(PI_A_RSVD_PONIP_DS)
#undef  PI_PON_DSC_USAGE_DS
#define PI_PON_DSC_USAGE_DS	PI_X(PI_A_PON_DSC_USAGE_DS)
#undef  PI_PON_IPSTS_US
#define PI_PON_IPSTS_US	PI_X(PI_A_PON_IPSTS_US)
#undef  PI_PON_DSC_USAGE_US
#define PI_PON_DSC_USAGE_US	PI_X(PI_A_PON_DSC_USAGE_US)
#undef  PI_PON_IPSTS_DS
#define PI_PON_IPSTS_DS	PI_X(PI_A_PON_IPSTS_DS)
#undef  PI_PON_DSC_STS_DS
#define PI_PON_DSC_STS_DS	PI_X(PI_A_PON_DSC_STS_DS)
#undef  PI_PON_SID2QID
#define PI_PON_SID2QID	PI_X(PI_A_PON_SID2QID)
#undef  PI_PON_SIDVALID
#define PI_PON_SIDVALID	PI_X(PI_A_PON_SIDVALID)
#undef  PI_PON_SID_GLB_TH
#define PI_PON_SID_GLB_TH	PI_X(PI_A_PON_SID_GLB_TH)
#undef  PI_PON_SID_Q_MAP_DS
#define PI_PON_SID_Q_MAP_DS	PI_X(PI_A_PON_SID_Q_MAP_DS)
#undef  PI_PON_SID_RPV_TH
#define PI_PON_SID_RPV_TH	PI_X(PI_A_PON_SID_RPV_TH)
#undef  PI_PON_SID_STOP_TH
#define PI_PON_SID_STOP_TH	PI_X(PI_A_PON_SID_STOP_TH)
#undef  PI_PON_US_FIFO_CTL
#define PI_PON_US_FIFO_CTL	PI_X(PI_A_PON_US_FIFO_CTL)
#undef  PI_PON_BW_THRES
#define PI_PON_BW_THRES	PI_X(PI_A_PON_BW_THRES)
#undef  PI_PON_DSC_STS_US
#define PI_PON_DSC_STS_US	PI_X(PI_A_PON_DSC_STS_US)
#undef  PI_MOCIR_FRC_MD
#define PI_MOCIR_FRC_MD	PI_X(PI_A_MOCIR_FRC_MD)
#undef  PI_MOCIR_FRC_VAL
#define PI_MOCIR_FRC_VAL	PI_X(PI_A_MOCIR_FRC_VAL)
#undef  PI_MOCIR_TH_H
#define PI_MOCIR_TH_H	PI_X(PI_A_MOCIR_TH_H)
#undef  PI_MOCIR_TH_L
#define PI_MOCIR_TH_L	PI_X(PI_A_MOCIR_TH_L)
#undef  PI_PON_TB_CTRL
#define PI_PON_TB_CTRL	PI_X(PI_A_PON_TB_CTRL)
#undef  PI_PON_SCH_CTRL
#define PI_PON_SCH_CTRL	PI_X(PI_A_PON_SCH_CTRL)
#undef  PI_PON_QID_CIR_RATE
#define PI_PON_QID_CIR_RATE	PI_X(PI_A_PON_QID_CIR_RATE)
#undef  PI_PON_QID_PIR_RATE
#define PI_PON_QID_PIR_RATE	PI_X(PI_A_PON_QID_PIR_RATE)
#undef  PI_PON_SCH_QMAP
#define PI_PON_SCH_QMAP	PI_X(PI_A_PON_SCH_QMAP)
#undef  PI_PON_TCONT_EN
#define PI_PON_TCONT_EN	PI_X(PI_A_PON_TCONT_EN)
#undef  PI_PON_WFQ_WEIGHT
#define PI_PON_WFQ_WEIGHT	PI_X(PI_A_PON_WFQ_WEIGHT)
#undef  PI_PONIP_DBG_CTRL_US
#define PI_PONIP_DBG_CTRL_US	PI_X(PI_A_PONIP_DBG_CTRL_US)
#undef  PI_PONIP_TOTAL_PAGE_CNT_US
#define PI_PONIP_TOTAL_PAGE_CNT_US	PI_X(PI_A_PONIP_TOTAL_PAGE_CNT_US)
#undef  PI_PONIP_SID_USED_PAGE_CNT_US
#define PI_PONIP_SID_USED_PAGE_CNT_US	PI_X(PI_A_PONIP_SID_USED_PAGE_CNT_US)
#undef  PI_GPON_DPRU_RPT_PRD
#define PI_GPON_DPRU_RPT_PRD	PI_X(PI_A_GPON_DPRU_RPT_PRD)
#undef  PI_PONIP_SID_OVER_STS
#define PI_PONIP_SID_OVER_STS	PI_X(PI_A_PONIP_SID_OVER_STS)
#undef  PI_PON_SCH_OPT
#define PI_PON_SCH_OPT	PI_X(PI_A_PON_SCH_OPT)
#undef  PI_PON_DS_PBO_PAGE_Q0
#define PI_PON_DS_PBO_PAGE_Q0	PI_X(PI_A_PON_DS_PBO_PAGE_Q0)
#undef  PI_PONIP_SID_OVER_LATCH_STS
#define PI_PONIP_SID_OVER_LATCH_STS	PI_X(PI_A_PONIP_SID_OVER_LATCH_STS)


#undef  SDS_ANA_COM_REG03
#define SDS_ANA_COM_REG03      SDS(0x2258c)
#undef  SDS_ANA_COM_REG08
#define SDS_ANA_COM_REG08      SDS(0x225a0)
#undef  SDS_ANA_COM_REG11
#define SDS_ANA_COM_REG11      SDS(0x225ac)
#undef  SDS_ANA_COM_REG12
#define SDS_ANA_COM_REG12      SDS(0x225b0)
#undef  SDS_ANA_COM_REG22
#define SDS_ANA_COM_REG22      SDS(0x225d8)
#undef  SDS_ANA_COM_REG26
#define SDS_ANA_COM_REG26      SDS(0x225e8)
#undef  SDS_ANA_COM_REG27
#define SDS_ANA_COM_REG27      SDS(0x225ec)
#undef  SDS_ANA_GPON_REG42
#define SDS_ANA_GPON_REG42     SDS(0x22728)
#undef  SDS_ANA_GPON_REG46
#define SDS_ANA_GPON_REG46     SDS(0x22738)
#undef  SDS_ANA_MISC_REG00
#define SDS_ANA_MISC_REG00     SDS(0x22500)
#undef  SDS_ANA_MISC_REG01
#define SDS_ANA_MISC_REG01     SDS(0x22504)
#undef  SDS_ANA_MISC_REG02
#define SDS_ANA_MISC_REG02     SDS(0x22508)
#undef  WSDS_DIG_00
#define WSDS_DIG_00            SDS(0x22030)
#undef  WSDS_DIG_01
#define WSDS_DIG_01            SDS(0x22034)
#undef  WSDS_DIG_02
#define WSDS_DIG_02            SDS(0x22038)
#undef  WSDS_DIG_03
#define WSDS_DIG_03            SDS(0x2203c)
#undef  WSDS_DIG_1D
#define WSDS_DIG_1D            SDS(0x220a4)
#undef  SDS_CFG
#define SDS_CFG			(swc->sds_cfg)
/* The six OMCI packet counters, contiguous from the per-chip base. They were
 * SIX BARE 9602C LITERALS, which on the 9603CVD and the 9607C point 0x580 short
 * of the real block -- so gpon_fsm_poll counted a different register entirely
 * on two of the three chips, and nothing could fail: a counter reads SOMETHING
 * either way. */
#define OMCI_DROP_PKT_CNT	(swc->omci_cnt + 0x00)
#define OMCI_TX_PKT_CNT		(swc->omci_cnt + 0x04)
#define OMCI_RX_PKT_CNT		(swc->omci_cnt + 0x08)
#define OMCI_TX_BYTE_CNT	(swc->omci_cnt + 0x0c)
#define OMCI_RX_BYTE_CNT	(swc->omci_cnt + 0x10)
#define OMCI_CRC_ERROR_PKT_CNT	(swc->omci_cnt + 0x14)
/* the irregular movers, named after the silicon's own names */
#define SC_IND_WD               (swc->sc_ind_wd)
#define WRAP_GPHY_MISC          (swc->wrap_gphy_misc)
#define THERMAL_CTRL_0          (swc->thermal_ctrl_0)
#define FORCE_P_ABLTY           (swc->force_p_ablty)
#define ABLTY_FORCE_MODE        (swc->ablty_force_mode)
#define DYNGASP_CTRL            (swc->dyngasp_ctrl)
#define ACCEPT_MAX_LEN_CTRL     (swc->accept_max_len_ctrl)
#define PON_TRAP_CFG            (swc->pon_trap_cfg)
#define CF_CFG                  (swc->cf_cfg)
#define LUT_UNKN_UC_DA_CTRL     (swc->lut_unkn_uc_da_ctrl)
#define LUT_LEARN_OVER_CTRL     (swc->lut_learn_over_ctrl)
#define UNKN_L2_MC              (swc->unkn_l2_mc)
#define UNKN_IP4_MC             (swc->unkn_ip4_mc)
#define UNKN_MC_CFG             (swc->unkn_mc_cfg)
#define LUT_BC_FLOOD            (swc->lut_bc_flood)
#define LUT_UNKN_MC_FLOOD       (swc->lut_unkn_mc_flood)
#define LUT_UNKN_UC_FLOOD       (swc->lut_unkn_uc_flood)
#define LUT_SYS_LRN_LIMIT       (swc->lut_sys_lrn_limit)
#define RMA_CTRL01              (swc->rma_ctrl01)
#define RMA_CTRL02              (swc->rma_ctrl02)
#define RMA_CFG                 (swc->rma_cfg)
#define QOS_UNI_TRAP_PRI_CTRL   (swc->qos_uni_trap_pri_ctrl)
#define OAM_CTRL_0              (swc->oam_ctrl_0)
#define CFG_UNHIOL              (swc->cfg_unhiol)
#define PISO_EXT                (swc->piso_ext)
/* Open the DS GEM unicast/broadcast pass gate (GEM_DS_MC_CFG). Default OFF: without
 * the PON-IP->GMAC-NIC OMCI drain, opening it backs up the DS path and stalls the US
 * (deactivate ~48s). Set =1 only when drain-path testing. */
/* Pass unicast DS GEM so the GTC de-encapsulates the OLT's unicast OMCI.
 * DEFAULT OFF: opening it de-encapsulates OMCI (verified: gem_ds_rx climbs) but
 * the de-encapsulated frame does NOT yet drain to the CPU NIC (omci_rx stays 0 —
 * the PON-IP->GMAC HW-PBO trap delivery is the open wall), so the DS path backs
 * up and the OLT Deactivates the ONU (~17-22s). Set gem_gate_open=1 to resume
 * debugging the trap delivery; keep 0 for a stable O5 baseline. */
/* Pass unicast DS GEM so the GTC de-encapsulates the OLT's unicast OMCI.
 * DEFAULT OFF: stable O5 baseline. Stage instrumentation (DSPIPE / ds_deenc_sweep,
 * gated on this flag) showed that with it ON, de-encap is 0 on ALL 128 flows and
 * the OLT is NOT sending OMCI (nonidle GEM ~0) — it deallocs + deactivates ~36s.
 * Set =1 to re-run the DS-pipeline probes. */
static bool gem_gate_open;
module_param(gem_gate_open, bool, 0444);
MODULE_PARM_DESC(gem_gate_open, "open DS GEM pass gate (needs the PON-IP->host OMCI drain; default off = stable online)");
/* Install the WAN data-GEM datapath. CONFIRMED (stability bisection): doing it ~30s
 * AFTER O5 (while Online) re-latches the US-NIC (modeset) and glitches the established
 * US burst -> OLT "Laser out" -> deactivate. FIX: install it DURING config (at
 * Configure_Port-ID, right after the OMCC), so its modeset coincides with the config
 * phase (tolerated, like the OMCC's own modeset) and the ONU reaches Online already
 * carrying the data path -- no modeset while Online. Default ON. */
static bool data_gem_en = true;
module_param(data_gem_en, bool, 0644);
MODULE_PARM_DESC(data_gem_en, "install the WAN data GEM datapath during config (default on)");
/* trace=0 (default) silences the routine per-PLOAM/per-ACK dumps so the compact
 * O5 timeline survives the lossy serial console; key-PLOAM EVT + O5 lines always print. */
static bool trace;	/* default 0: per-PLOAM/ACK tracing is SLOW (printk over serial) and perturbs the
			 * activation timing (breaks ranging when on). Set luna_gpon.trace=1 only for short diagnostics. */
module_param(trace, bool, 0644);
MODULE_PARM_DESC(trace, "verbose per-PLOAM/per-ACK serial spam (default 0)");

static bool cdr_reseat_on_reactivate = true;	/* default ON (A/B 2026-06-15): on a deactivate->O1 re-range, re-pulse the
			 * softirq-safe US-TX SerDes interface reset-B (WSDS_DIG_1D[16], the same primitive the
			 * boot path + the O3 re-sync use) so a marginal serializer lock from the prior activation
			 * is re-attempted immediately, cutting the cold-start O5<->O1 activation flapping. Opt-in
			 * A/B: TX-interface only (the locked DS RX framer is undisturbed). */
module_param(cdr_reseat_on_reactivate, bool, 0644);
MODULE_PARM_DESC(cdr_reseat_on_reactivate, "re-seat US-TX SerDes reset-B on re-range to cut activation flapping (default 0)");
static bool ploam_tx_dbg = true;	/* TEST: log per-send US-PLOAM TX (ENQ self-clear = HW transmitted) to
					 * prove whether the urgent-queue ACK/Password actually leaves the ONU
					 * (OLT raises LOAi = never gets our acks). Logs first 40 sends. */
module_param(ploam_tx_dbg, bool, 0644);
MODULE_PARM_DESC(ploam_tx_dbg, "log US-PLOAM CPU-TX ENQ self-clear per send (urgent-queue TX diagnostic)");
/* o5_rearm_burst_gate: re-apply the US burst-gate cluster (0x5188/0x526c/0x6024/0x6260)
 * and re-arm the HW auto-No_message keepalive template on every O5 entry (not just __init),
 * so a re-ranged O5 after a GMAC/SDS reset does not run on US-side reset defaults. Default on;
 * A/B with luna_gpon.o5_rearm_burst_gate=0. */
static bool o5_rearm_burst_gate = true;
module_param(o5_rearm_burst_gate, bool, 0644);
MODULE_PARM_DESC(o5_rearm_burst_gate, "re-apply US burst-gate cluster + No_message keepalive on each O5 entry (default on)");
/* o5_ploam_keepalive_ticks: emit a No_message US PLOAM (HW auto queue 0x7) every N FSM ticks
 * (10ms each) while at O5 with an assigned ONU-ID, so a valid PLOAM is present in the OLT's
 * granted slots regardless of how the shared US-PLOAM buffer was last written, defeating the
 * OLT's PLOAM/ack-liveness timeout. 0 = disabled. Default 100 (~1s). */
static uint o5_ploam_keepalive_ticks;	/* default OFF (match stock: zero unsolicited US-PLOAM at O5) */
module_param(o5_ploam_keepalive_ticks, uint, 0644);
MODULE_PARM_DESC(o5_ploam_keepalive_ticks, "emit No_message US-PLOAM every N 10ms ticks at O5 (0=off, default 0 -- stock emits no unsolicited US-PLOAM at O5)");

/* o5_provision_watchdog_ticks: a "Laser out" boot reaches O5 LOCALLY while the
 * OLT cannot frame our US burst, so it never provisions us and sends no
 * Deactivate the ONU acts on -- the ONU sits at O5 forever with the bad phase and
 * never self-recovers. If at O5 this many ticks with ZERO gpon0 WAN RX, re-range
 * to RE-ROLL the serializer phase. Gated past the slow-lease window and on
 * wan_rx==0, so it can never disturb a working or slow-leasing link.
 * PROVEN INEFFECTIVE (2026-06-16, HW A/B): it fires correctly, and re-ranging
 * does NOT recover a stuck boot -- one reached sn_tx=37 and two watchdog
 * re-ranges and still never leased. The bad US-TX serializer lock is a COLD-START
 * analog state fixed at power-on; no runtime re-range re-rolls it, only a reboot.
 * DEFAULT OFF, kept as documented negative knowledge. */
static uint o5_provision_watchdog_ticks;	/* default 0 = off (proven ineffective, see above) */
module_param(o5_provision_watchdog_ticks, uint, 0644);
MODULE_PARM_DESC(o5_provision_watchdog_ticks, "re-range if at O5 this many ticks with gpon0 RX=0 (0=off default; PROVEN INEFFECTIVE: re-range does not re-roll the cold-start serializer lock)");
/* los_rerange_ticks: autonomous downstream-LOS recovery. With the downstream
 * light gone the OLT cannot send a Deactivate, so the ONU must notice the LOS
 * itself, tear down to O1 and re-acquire when light returns; without this the FSM
 * sits stale at O5 after a fibre pull and never re-ranges on reconnect. Stock
 * exits O5->O6->O1 essentially immediately, so the debounce is SHORT, and
 * I2C pad-steal dips are rejected by ALSO requiring the SoC SerDes signal-detect
 * to be gone -- a pad-steal perturbs optic_los alone, a real pull drops both. */
static uint los_rerange_ticks = 30;		/* ~300ms of (optic_los & !sds_sdet): catches a real
						 * 2-4s fiber pull, ~3x stock's 100ms TO2; a sub-second
						 * pad-steal can neither reach it nor (lacking
						 * !sds_sdet) be counted at all */
module_param(los_rerange_ticks, uint, 0644);
MODULE_PARM_DESC(los_rerange_ticks, "drop to O1 + re-range after a REAL downstream LOS (optic_los AND no SerDes sig-detect) persists this many ~10ms ticks (fiber-pull recovery; 0=off, default 30 ~300ms ~= stock TO2)");
static u32 gpon_los_run;			/* consecutive real-LOS (optic_los & !sds_sdet) tick count */

/* The O1/O2/O3 dwell report -- the half the activation diagnostic was missing.
 * Every other PLOAM sample point fires on a TRANSITION, so a board that never
 * leaves O1 emits NOTHING AT ALL (measured on the G24W, whose O1 stall produced
 * no line to read). This reports while the FSM SITS in an early state and changes
 * no state doing it. VOLUME IS A DEFECT, so the default is 5 s and not per tick:
 * a stalled board gives ~12 lines a minute, enough to read the counters that
 * split "the OLT never asked" from "it asked and we did not answer". */
static uint early_dwell_report_ticks = 500;	/* ~5 s at the ~10ms tick */
module_param(early_dwell_report_ticks, uint, 0644);
MODULE_PARM_DESC(early_dwell_report_ticks, "report the PLOAM dwell at O1/O2/O3 every this many ~10ms ticks, with the GTC's serial-number and ranging request counters (0=silent, default 500 ~5s). Reports only; it never changes the FSM");

/* Fiber-pull / re-range diagnostic (event-driven, NO per-tick logging -- the feed_rekick flood
 * lesson). gpon_rerange_cnt = completed O5->..->O5 recoveries (LOS/deact); gpon_last_outage_ms =
 * wall time from the drop below O5 to the O5 re-entry of the most recent one. Shown on-demand in
 * the /proc/gpon "fiber:" summary; the completion is logged once per event, flap-damped so a
 * marginal/flapping link cannot storm the log (counters still count -- only log lines throttle).
 * Declared here (before the /proc show handler that reads them) so the summary line compiles. */
static u32 gpon_rerange_cnt;
static u32 gpon_last_outage_ms;
static unsigned long gpon_rerange_start_j;
static unsigned long gpon_rerange_last_log_j;

/* last DS PLOAM type drained this cycle, surfaced on the periodic O5 line */
static u8 gpon_last_ds_type;
/* DIAGNOSTIC: force the upstream laser continuously on (US_CFG.FS_LON). Tests
 * whether the SoC SerDes-TX can drive the BOSA at all, independent of the GTC
 * burst scheduler. DEV-ONLY — continuous light jams a multi-ONU PON. */
static bool force_laser;
module_param(force_laser, bool, 0444);
MODULE_PARM_DESC(force_laser, "force US laser CW on (US_CFG FS_LON) — SerDes-TX emission diagnostic");
/* Laser burst bias/mod DAC override (BOSA W54 0x236 hi-8 / W55 0x237 hi-8), applied
 * after bosa_tx_enable (which loads the A4-golden bias=0x19/mod=0x67). The OLT raises
 * "Laser out" at O5 because the un-OFFK'd laser's burst is marginal for its operational
 * burst-RX. MOD sets the burst PEAK (1-level) = stronger burst; BIAS sets the DC/0-level
 * (raising it worsens extinction -> hurts DS), so prefer raising MOD and keep BIAS low.
 * Conservative bump first (laser-safety: MPD high/low detect is disarmed in bring-up, so
 * the DAC value is the only over-power guard — do NOT crank these). Sweepable live. */
static unsigned int laser_bias = 0x32;	/* Board-C REAL per-board calib (rtl8290b.data CAL_IBIAS 9960uA->code 0x32f). 0x18/0x34 25C-LUT A/B = no rate gain (2/6 vs 3/6), reverted */
static unsigned int laser_mod = 0xbb;	/* Board-C REAL per-board calib (rtl8290b.data CAL_IMOD 36694uA->code 0xbbd) */
module_param(laser_bias, uint, 0644);
module_param(laser_mod, uint, 0644);
MODULE_PARM_DESC(laser_bias, "BOSA bias DAC hi-8 (0x236) override; 0=keep A4-golden 0x19");
MODULE_PARM_DESC(laser_mod, "BOSA mod DAC hi-8 (0x237) override for stronger burst peak; 0=keep golden 0x67");
/* rev-A bring-up US-TX SerDes CMU/PLL + TX-LA-LDO writes (part of the stock rev-A GPON
 * mode-set analog config) that our gpon_serdes_init OMITS: SDS_ANA_COM_REG02/03/08 = the
 * TX CMU/PLL that clocks the US serializer, COM_REG24=0x8001 = REG_TXLA_LDOEN (TX limiting-amp
 * LDO/output stage). Without them the laser is DC-biased but the MAC's US data is not cleanly
 * serialized/modulated onto it -> OLT "Laser out" + rxsid=0 (the unified US-TX wall). The CMU
 * is shared with RX, so this can perturb DS -> watch ranging; revert if DS breaks. */
static unsigned int serdes_modev1_tx;	/* default 0: TESTED (COM_REG02/03/08/24/25 applied+readback-confirmed) =
					 * NO effect on US-TX burst (OLT still "Laser out", rxsid=0) and slightly
					 * degraded DS (CMU shared) -> the omitted ModeV1 TX SerDes regs are NOT the
					 * gap; reverted to off. Param kept for reference. */
module_param(serdes_modev1_tx, uint, 0444);
MODULE_PARM_DESC(serdes_modev1_tx, "1=apply the rev-A bring-up US-TX SerDes CMU/PLL + TXLA_LDOEN writes (no effect)");

/* serdes_tx_xtra: 0 (default) = ORACLE-PARITY — do NOT set the 3 SerDes-TX serializer-path bits
 * (0x220a8[5:4], 0x2281c[14], 0x22a30[8]) that the LIVE stock-ref ONU leaves clear (it ranges +
 * bursts without them). 1 = restore the old behaviour (set them) for A/B if parity regresses ranging. */
static unsigned int serdes_tx_xtra;
module_param(serdes_tx_xtra, uint, 0644);
MODULE_PARM_DESC(serdes_tx_xtra, "1=set legacy SerDes-TX D2A/clk-edge bits (stock=0; default 0=match stock)");
/* serdes_cdr_reset: replicate the stock SerDes CDR-reset behavior
 * (SDS_ANA_COM_REG08 @ swcore 0x225a0): INVERT bit15, hold 10ms, RESTORE. Our clean-room
 * init OMITTED this CDR-lock pulse. It seats the upstream-TX SerDes serializer CDR; without it the
 * TX serializer lock is non-deterministic, which matches the observed cycle-to-cycle US-burst
 * variation (some O5 windows the OLT decodes hundreds of US-OMCI, others it loses the burst at once
 * = LOSi/LOAi "Laser out"). NOTE: serdes_cdr_reset is now writable (0644) so it can be
 * left default-on but A/B'd live. Default on (the fix); luna_gpon.serdes_cdr_reset=0 reverts. */
static bool serdes_cdr_reset = true;
module_param(serdes_cdr_reset, bool, 0644);
MODULE_PARM_DESC(serdes_cdr_reset, "pulse SDS_ANA_COM_REG12 (0x225b0) bit15 10ms (stock serdesCdr_reset RX_SD_POR_SEL) (stock ponmac step; default on)");
/* usnic_initrdy_poll: stock-aligned US-NIC readiness gate. PON_IPSTS_US (PON-IP
 * 0x1bf020f4) bit0 = PONIC_INITRDY = read-only US PON-IP core init-ready. Wait for
 * it before the US GMII RX/TX latch edge so the NIC latches with the core ready.
 * Bounded 200ms, read-only -> cannot reorder config/reset or worsen the lock.
 * (Stock NEVER writes 0x20f4; our old pi_wr(0x20f4,1) set a reserved bit -> removed.)
 * RE verdict: a digital-core ready flag, so probably instrumentation not the analog
 * fix — but discriminating: a timeout on a failed-WAN boot implicates the digital US
 * layer; never-timeout proves that layer innocent. Default on. */
static bool usnic_initrdy_poll = true;
module_param(usnic_initrdy_poll, bool, 0644);
MODULE_PARM_DESC(usnic_initrdy_poll, "wait PON_IPSTS_US.PONIC_INITRDY (0x1bf020f4 bit0)==1 before US GMII latch (default on; bounded 200ms, read-only)");
/* usnic_initrdy_repulse: ACTIVE escalation (experiment). On PONIC_INITRDY timeout,
 * re-pulse the CDR (COM_REG12 bit15, same as serdes_cdr_reset) + re-poll once
 * — re-rolls the analog lock without reordering config/reset. Default off. */
static bool usnic_initrdy_repulse;
module_param(usnic_initrdy_repulse, bool, 0644);
MODULE_PARM_DESC(usnic_initrdy_repulse, "on PONIC_INITRDY timeout, re-pulse CDR (COM_REG12 bit15) + re-poll once (default 0)");
/* cdr_stuck_recover: replica of the stock runtime link-state-check DS-CDR-wedge
 * recovery this driver was MISSING. From a cold power-on the DS CDR can come up
 * wedged, and a soft reboot re-runs init assuming fresh HW, so a bad lock
 * persists. Stock detects the wedge by GPON_GTC_DS_INTR_STS == 0xca0eca0f and
 * toggles SP_SDS_EN_RX 1->0->1 with a 10 ms settle; we run the same check each
 * FSM poll as a two-tick toggle, to keep the 10 ms wait out of softirq.
 * RATE-bounded, never count-capped: fast attempts first, then one per
 * GPON_CDR_STUCK_SLOW_TICKS for as long as the wedge persists. */
static bool cdr_stuck_recover = true;
module_param(cdr_stuck_recover, bool, 0644);
MODULE_PARM_DESC(cdr_stuck_recover, "recover a wedged DS CDR (GTC_DS_STS==0xca0eca0f) by toggling SP_SDS_EN_RX, like stock (default on)");

/* Periodic DS multiframe/BWmap ESD-recover (stock gpon_esdRecover). The DS framer can
 * byte-lock (LOF clear, gtc_ds_sts=0x04, OMCI/GEM fine) yet come up with the multiframe/
 * PLEND word phase mis-latched -- a 1-of-4 power-on roll of the GTC->US-scheduler grant
 * handoff. The BWmap can then not be located, no US alloc grants match (bwm_acpt stays 0),
 * no upstream burst egresses, and the OLT deacts (~1/4 cold boots; re-range keeps the bad
 * phase). Watch the cumulative PLEND/LOM fail counter (a static gtc_ds_sts snapshot hides
 * it) while byte-locked; when it climbs past the stock threshold, pulse the RX-CDR reset
 * (gpon_cdr_reset_worker, 0x225b0 bit15) to re-roll the DS word phase into a good one. */
#define GTC_DS_STS_LOF		BIT(1)		/* loss-of-frame; clear = framer byte-locked */
#define GPON_ESD_INTERVAL_MS	5000		/* stock gpon_esdRecover_interval */
#define GPON_ESD_THRESHOLD	20		/* stock gpon_esdRecover_threshold */
static bool gpon_esd_recover = true;
module_param(gpon_esd_recover, bool, 0644);
MODULE_PARM_DESC(gpon_esd_recover, "periodic RX-CDR re-lock when the DS PLEND/LOM parse fails while byte-locked (stock gpon_esdRecover; fixes the ~1/4 grant-deaf churn; default on)");
#define GPON_CDR_STUCK_MAX	8	/* FAST re-acquire attempts before backing off */
/* RATE-BOUNDED, NEVER COUNT-CAPPED -- the standing rule, and the shape
 * cortina-gpon.c already uses. After the fast budget is spent the toggle keeps
 * being attempted, once per slow interval, for as long as the wedge is there.
 * IT USED TO STOP DEAD: the attempt counter was a function-static inside
 * gpon_fsm_poll(), so nothing could refill it -- neither the state setter nor any
 * of the four teardowns -- and its ONLY reset was a status read that is not the
 * wedge sentinel, i.e. the wedge clearing by itself, the one event that would
 * have made the recovery unnecessary. Eight attempts at two ticks each, so after
 * 160 ms of a persistent wedge the recovery was disarmed for the module's
 * lifetime. Its comments claimed the cap was "per range cycle" and that it
 * "yields to the LOS/re-range path"; neither was true, and that path is gated on
 * state >= 2, which a wedged DS framer cannot reach.
 * Pinned by gpon_lifetime_test.c case [d], SEEN to fail on the pre-fix source. */
#define GPON_CDR_STUCK_SLOW_TICKS	6000	/* ~60 s at the 10 ms poll */
static unsigned int gpon_cdr_stuck_tries;	/* consecutive attempts THIS episode */
static unsigned int gpon_cdr_stuck_count;	/* diag: total wedges detected */
static unsigned int gpon_cdr_stuck_fixed;	/* diag: wedges cleared by the toggle */
static u32 gpon_gtc_ds_sts_last;		/* diag: last raw GTC DS status seen */
/* serdes_stock_seq: 1 = the EXACT stock rev-A bring-up ORDER (reset first then
 * config, single reset bit, D2A/sample-clock bits SET, no reset-B dance) instead
 * of ours (config first plus the reset-B dance). It tested whether the
 * deterministic cold-start US-TX serializer lock is an emergent property of
 * stock's order.
 * A/B RESULT 2026-06-16 (HW): the stock order DS-locks fine (O5 6/6, so our
 * reset-B dance is NOT needed) but the cold-start lease rate did not improve
 * (1/6, no better than baseline). Combined with config-values-match and
 * every-component-tested, the ~40% cold-start lock non-determinism is
 * irreducible by any register- or sequence-level ONU action -- it is below the
 * register level, in analog VCO/CMU trim and PVT. DEFAULT OFF, kept as
 * documented negative knowledge. */
static bool serdes_stock_seq;	/* default 0 = our gpon_serdes_init (stock order tested = no improvement) */
module_param(serdes_stock_seq, bool, 0644);
MODULE_PARM_DESC(serdes_stock_seq, "1=stock rev-A SerDes bring-up order (gpon_serdes_init_stock); 0=our gpon_serdes_init");

/* family_lib: bring the SerDes up via the clean-room luna_ponmac family library
 * (LUNA_CHIP_9602C path) instead of the inline gpon_serdes_init(). Validates the
 * family-lib op-table framework on real 9602C silicon: family_lib=1 must reach O5 +
 * lease + keep LAN exactly like family_lib=0 (the lib's 9602C tables are a faithful
 * translation of gpon_serdes_init). Default off; A/B with luna_gpon.family_lib=1. */
static bool family_lib = true;	/* default ON: the clean-room luna_ponmac family lib is the
				 * 9602C SerDes boot bring-up. HW-validated (O5 10/10 over two 5-boot
				 * runs, LAN ok, WAN leases at the analog rate) = equivalent to the
				 * inline path (its 9602C op-tables are a faithful, exact-match
				 * translation of gpon_serdes_init). luna_gpon.family_lib=0 = legacy inline. */
module_param(family_lib, bool, 0644);
MODULE_PARM_DESC(family_lib, "1=bring up SerDes via luna_ponmac family lib (9602C path, default); 0=inline gpon_serdes_init");
/* serdes_postmode_perturb: the family-lib path performs TWO US-TX serializer edges
 * AFTER GPON mode is committed that stock rev-A does NOT (DIG_1D[16] reset-B
 * re-sync + a post-mode serdesCdr_reset pulse). These were NEVER cleanly A/B'd
 * (the serdes_cdr_reset param does not gate the family-lib path). DEFAULT 0
 * (=stock-matching: skip them) — prime suspect for cold-start serializer-phase
 * jitter (WAN ~50%). luna_gpon.serdes_postmode_perturb=1 restores legacy behavior. */
static bool serdes_postmode_perturb;	/* default false: skip post-mode perturbations (stock rev-A) */
module_param(serdes_postmode_perturb, bool, 0644);
MODULE_PARM_DESC(serdes_postmode_perturb, "1=do post-GPON-mode DIG_1D resync + serdesCdr_reset (legacy); 0=skip (stock rev-A, default)");
/* serdes_sds_cfgrst: the family-lib SerDes reset pulse used to assert BOTH
 * CMD_SDS_CFG_RST_PS (bit7) and CMD_SDS_RST_PS (bit0); stock rev-A pulses ONLY bit0
 * (stock never writes bit7 anywhere during bring-up). Because the field-write is
 * RMW, bit7 stayed latched through the whole bring-up = an extra SDS-config reset
 * domain stock never touches -> prime suspect for the per-power-on US-TX serializer/
 * PLL phase re-roll (cold-start WAN ~50%, OLT "Laser out"). DEFAULT 0 = bit0-only
 * (stock = the fix); luna_gpon.serdes_sds_cfgrst=1 restores the legacy bit7+bit0 pulse. */
static bool serdes_sds_cfgrst;	/* default false = stock bit0-only SDS reset */
module_param(serdes_sds_cfgrst, bool, 0644);
MODULE_PARM_DESC(serdes_sds_cfgrst, "1=legacy: also pulse CMD_SDS_CFG_RST_PS bit7 in the SDS reset; 0=stock bit0-only (default, the cold-start fix)");
/* serdes_stock_analog: drive SDS_ANA_COM REG01 (0x22584)=0x73a4 + REG11 (0x225ac)
 * RX_FILT_CONFIG=0 to EXACTLY match the live-stock post-reset values — the ONLY two
 * SerDes registers that differed between stock (WAN-up, 100%) and our failing board
 * (cold-start ~50% US-TX "Laser out"). The golden table writes them correctly but the
 * SDS reset wipes REG01 bit14 (shared CMU) / REG11 RX_FILT; this re-applies them AFTER
 * the reset, like stock. DEFAULT 1 = the fix; luna_gpon.serdes_stock_analog=0 = legacy. */
static bool serdes_stock_analog = true;
module_param(serdes_stock_analog, bool, 0644);
MODULE_PARM_DESC(serdes_stock_analog, "1=match live-stock SDS REG01=0x73a4 + REG11 RX_FILT=0 post-reset (default, the cold-start fix); 0=legacy");
/* serdes_analog_postreset: program the FULL analog CMU/CDR golden table (COM_REG02/03/
 * 08/24/25 charge-pump/LDO/tank + GPON_REG46 CDR) AFTER the SDS reset (stock rev-A
 * order) instead of before it. The reset WIPES analog to defaults; pre-reset placement
 * (legacy) leaves the CMU/CDR locking against default operating-point values that the
 * partial REG01/REG11 re-apply never fully corrects -> metastable per-power-on lock =
 * the cold-start ~50% "Laser out". Post-reset = stock = deterministic lock every cold
 * boot + soft restart. DEFAULT 1 = the fix; luna_gpon.serdes_analog_postreset=0 = legacy. */
static bool serdes_analog_postreset = true;
module_param(serdes_analog_postreset, bool, 0644);
MODULE_PARM_DESC(serdes_analog_postreset, "1=program full analog CMU/CDR table AFTER the SDS reset (stock rev-A, default, the cold-start determinism fix); 0=legacy pre-reset");
/* serdes_cmu_settle_ms: ms to wait after forcing the 125M ref clock and BEFORE releasing
 * the SerDes interface reset-B (which latches the TX serializer phase) — lets the TX CMU PLL
 * lock first. 0 = legacy. Candidate fix for the cold-start ~50% US-TX "Laser out" metastability. */
static unsigned int serdes_cmu_settle_ms;
module_param(serdes_cmu_settle_ms, uint, 0644);
MODULE_PARM_DESC(serdes_cmu_settle_ms, "ms TX-CMU-lock settle between 125M ref force and reset-B release (0=legacy default)");
/* serdes_txpll_relock: at the O1/O2->O3 edge, re-lock the TX CMU PLL by toggling
 * the CMU enable 1->0->1, then re-sync the SerDes word FIFO pointer. On a fresh
 * power-on under strong downstream light the optical signal-detect can assert
 * before the CMU has settled, latching the TX PLL onto the WRONG clock rate on
 * ~50% of cold power-ons and producing a burst the OLT cannot frame.
 * The TIMING is what matters: at O3 ENTRY, after the downstream framer has locked
 * and the signal-detect transient is over, the lock becomes deterministic; the
 * same re-lock during SerDes mode-set does not help, it just re-rolls the same
 * metastability. Validated over repeated cold power-cuts: ON = 5/5 stable-O5
 * boots, OFF = the ~50% deactivate-on-cold-start failure returns. */
static bool serdes_txpll_relock = true;
module_param(serdes_txpll_relock, bool, 0644);
MODULE_PARM_DESC(serdes_txpll_relock, "1=re-lock the TX CMU PLL (toggle CMU enable + FIFO re-sync) at O3 entry before the first US burst — fixes the ~50% cold-start lock-to-wrong-rate (default on); 0=skip");
/* The periodic US-TX interface reset-B pulse while un-ranged (poll loop, every 200
 * ticks).  MEASURED 2026-09-06 on the X111W: of ~85 activation cycles on our image,
 * the ONLY one that ranged had spent 48 ms in O3 before Assign_ONU-ID; every cycle
 * that the OLT deactivated had spent ~8 s there -- long enough for at least one of
 * these pulses to land AFTER the O3-entry TX-PLL relock.  Writable at runtime
 * (/sys/module/luna_gpon/parameters/unranged_reseat) so the A/B is one echo on a
 * cycling board, no rebuild.  Default = the behaviour that shipped. */
static bool unranged_reseat = true;
module_param(unranged_reseat, bool, 0644);
MODULE_PARM_DESC(unranged_reseat, "pulse the US-TX SerDes interface reset-B every ~200 ticks while un-ranged (default 1 = as shipped; 0 = the O4-wall A/B)");
/* optical_poll: periodic ANI-G DDM optical read. DEFAULT OFF — the periodic BOSA I2C
 * read (in any context) transiently disturbs the optical path and provokes the OLT
 * op=0xff dealloc/DEACT churn (no WAN); proven by A/B (poll on=churn, off=clean WAN).
 * Until the read is made glitch-free, ANI-G reports the (plausible) snapshot value.
 * =1 re-enables the live read (for development of the glitch-free path). */
static bool optical_poll;
module_param(optical_poll, bool, 0644);
MODULE_PARM_DESC(optical_poll, "1=periodic ANI-G DDM optical poll; 0=off default (the live /proc read + LuCI refresh already show current dBm without periodic BOSA I2C, which historically churned the OLT)");
/* bosa_i2c_restore_pad: optional cleanup — return bus-0's SoC pad to the optical-SD
 * function (SOC_IO_MODE_EN[13]=0) after each BOSA I2C transaction. DEFAULT OFF: the
 * baseline leaves it at 1 and optic_los still reports LOS correctly (operator-confirmed
 * on a real fiber pull), so this is NOT needed for LOS detection and is kept off to
 * avoid perturbing the cold-start datapath. =1 only if a masked optic_los is ever
 * observed. */
static bool bosa_i2c_restore_pad;
module_param(bosa_i2c_restore_pad, bool, 0644);
MODULE_PARM_DESC(bosa_i2c_restore_pad, "restore SOC_IO_MODE_EN[13]=0 (optical-SD pad) after each BOSA I2C transaction (default 0; optic_los works without it)");
/* lan_keep_open: LAN management must be reachable INDEPENDENT of the WAN/GPON state —
 * it must NEVER be gated on O5. A bad cold-start (no O5) or a WAN/fiber disconnect
 * (LOS -> re-range) must NOT kill LAN control of the ONU. When set (default), the
 * switch VLAN_FILTER is never asserted for LAN-gating: LAN is open from boot and
 * stays open through ranging / re-range / LOS. 0 = legacy (filter on during config,
 * cleared at stable O5, re-armed on every drop below O5). */
static bool lan_keep_open = true;
module_param(lan_keep_open, bool, 0644);
MODULE_PARM_DESC(lan_keep_open, "keep LAN open from boot independent of GPON/WAN state (default 1); 0=legacy O5-gated");
/* force_soc_clk: before the SerDes bring-up, write the 3 SoC sysreg words at 0x18000100/12c/140
 * (⚠ NOT clock registers -- the stock binaries show a strap word and two DRAM-calibration result
 * words, see the soc_clk[] table; the parameter keeps its historical name) to the live-STOCK
 * (100%-deterministic) values that OUR FAIL boot
 * was found to differ from (stock 0x00440e00/0x024d024d/0x024d024d vs ours 0x00440f00/
 * 0x02490249). Candidate fix for the cold-start ~50% US-TX metastability (stock-vs-ours
 * clock-config diff, same methodology that found REG01/REG11). 0 = legacy (no write). */
static bool force_soc_clk;
module_param(force_soc_clk, bool, 0644);
MODULE_PARM_DESC(force_soc_clk, "1=write live-stock SoC clock regs 0x18000100/12c/140 before SerDes bring-up (cold-start fix candidate); 0=legacy");
/* serdes_clkgate_rstb: gate the SerDes word clock (STOP_CLK=1) across the interface
 * reset-B release and un-gate LAST, so the word divider restarts on one defined edge
 * (defeats the async-reset-on-running-divider ~50% serializer-phase coin-flip). The
 * single highest-value cold-start fix candidate (ideation rank-1 fix). 0 = legacy. */
static bool serdes_clkgate_rstb;
module_param(serdes_clkgate_rstb, bool, 0644);
MODULE_PARM_DESC(serdes_clkgate_rstb, "1=clock-gated (STOP_CLK) SerDes reset-B release, un-gate last (cold-start metastability fix candidate); 0=legacy free-running");
/* serdes_skip_rstb_dance: Live debug confirmed WSDS_DIG_1D is ALREADY 0x1c000 (interface reset-B
 * released) at mode-set entry AND that the SDS reset (CMD_SDS_RST_PS bit0) does NOT clear it — so
 * the c2_sds_rstb dance (assert DIG_1D[15/16]->0 then release ->1) is a GRATUITOUS TX/RX reset-B
 * 1->0->1 PULSE on an already-running serializer = the textbook async-reset-on-running-divider that
 * latches a metastable word-phase (cold-start ~50%). Stock rev-A bring-up NEVER pulses it. =1 skips
 * the whole dance (DIG_00=0xf30 + the DIG_1D toggles) — DIG_1D/DIG_00 are already at their
 * operational values, so no edge is issued. Cold-start determinism fix candidate; A/B vs the dance. */
static bool serdes_skip_rstb_dance;
module_param(serdes_skip_rstb_dance, bool, 0644);
MODULE_PARM_DESC(serdes_skip_rstb_dance, "1=skip the c2_sds_rstb DIG_1D reset-B dance (already-released; gratuitous phase-latching pulse); 0=legacy dance");
/* serdes_minimal_analog: our golden analog table does ~145 SerDes writes; the stock rev-A GPON
 * bring-up does only ~23 — it never writes the 3 duplicate GPON per-rate banks
 * (0x22608/0x22688/0x22788) nor the 4 FIB-bank bodies (0x22c00-0x22df8), ~134 redundant writes that
 * lengthen the bring-up before the CMU/serializer phase latches. =1 skips those over-configure writes
 * to match stock's minimal write-set (keeps the active GPON bank + FIB PDOWN-clear). Cold-start
 * determinism fix candidate (timing/transaction-count); final O5 config unchanged (HW defaults). */
static bool serdes_minimal_analog;
module_param(serdes_minimal_analog, bool, 0644);
MODULE_PARM_DESC(serdes_minimal_analog, "1=skip the golden-table writes stock omits (dup GPON banks + FIB bodies, ~134 writes) to match stock's minimal SerDes bring-up; 0=full golden table");
/* bosa_before_serdes: CROSS-SUBSYSTEM ORDER fix candidate (cold-start ~50% determinism).
 * Stock stages the external RTL8290B BOSA analog (I2C calib + reset + a BOSA-ready settle
 * via a ready-check) BEFORE bringing up the SoC SerDes/CMU (PON-MAC mode-set). OUR
 * probe inverts this: it runs the SerDes mode-set (CMU lock + the RX_EN 0->1 start edge)
 * FIRST, with the BOSA RX still powered-down, then calls bosa_rx_enable AFTER. A SerDes CMU/
 * CDR that locks against an un-settled BOSA analog/shared-reference is a clean ~50% metastable
 * lock. =1 moves bosa_probe()+bosa_rx_enable()+a settle to BEFORE the SerDes bring-up (stock
 * order); the later duplicate calls are skipped. A/B vs the legacy SerDes-first order. */
static bool bosa_before_serdes;
module_param(bosa_before_serdes, bool, 0644);
MODULE_PARM_DESC(bosa_before_serdes, "1=power+settle the BOSA RX before the SerDes bring-up (stock order); 0=legacy SerDes-first");
/* bosa_settle_ms: settle delay after bosa_rx_enable when bosa_before_serdes=1, emulating
 * the working firmware's ~21ms post-reset + ready-check analog-settle before the SerDes CMU locks. */
static uint bosa_settle_ms = 50;
module_param(bosa_settle_ms, uint, 0644);
MODULE_PARM_DESC(bosa_settle_ms, "ms to settle the BOSA analog before the SerDes bring-up when bosa_before_serdes=1 (default 50)");
/* sc_ldo_init: stock runs an LDO-init step (early in boot) that we OMIT — an
 * SC-indirect RMW of the DRAM-rail LDO byte 0xfdca (clear bits 2,3) + THERMAL_CTRL_0
 * (swcore 0x130)=0x00ec0005 (arm on-die over-temp ALARM comparator). Assessment:
 * DRAM-LDO + thermal alarm, NOT the SerDes/laser path — kept as stock platform
 * hygiene (the init we were missing), NOT expected to move the WAN cold-start rate.
 * Default on; A/B revert with luna_gpon.sc_ldo_init=0. */
static bool sc_ldo_init = true;
module_param(sc_ldo_init, bool, 0644);
MODULE_PARM_DESC(sc_ldo_init, "run stock rtk_ldo_init (SC-indirect 0xfdca analog LDO + THERMAL_CTRL_0); default on");

/* Register accessor the family lib injects: absolute phys -> KSEG1 uncached MMIO
 * (phys < 0x20000000 on this SoC: swcore 0x1b000000 / PON-IP 0x1bf00000). */
static u32 r960_phys_rd(u32 phys)
{
	return ioread32((void __iomem *)(unsigned long)(0xa0000000u | phys));
}
static void r960_phys_wr(u32 phys, u32 val)
{
	iowrite32(val, (void __iomem *)(unsigned long)(0xa0000000u | phys));
}
static const struct luna_ops rtl9602c_r960_ops = {
	.rd = r960_phys_rd,
	.wr = r960_phys_wr,
};
/* DIAGNOSTIC: skip BOSA TX power-on + APC ignition (keep RX golden / bosa_rx_enable)
 * to isolate whether laser emission is what destabilises the downstream framer
 * lock. Set true ONLY for the laser-vs-DS-RX bisection; normal operation = false. */
/* ★ THE 9603CVD PAD RECIPE, OFF BY DEFAULT.  Its values come from that board's
 * own stock firmware read in O5, but whether writing them at BRING-UP time is
 * what the RX needs is unproven -- applying them live after the SerDes was up
 * changed nothing.  A parameter answers that on ONE image. */
static bool gpio_pad_9603cvd;
module_param(gpio_pad_9603cvd, bool, 0444);
MODULE_PARM_DESC(gpio_pad_9603cvd,
		 "apply the RTL9603CVD optical-SD pad recipe derived from its own stock (default 0)");

static bool laser_off;		/* default false; set via luna_gpon.laser_off=1 for the isolation test */
module_param(laser_off, bool, 0444);
MODULE_PARM_DESC(laser_off, "skip laser TX-enable+APC (DS-RX-vs-laser isolation: laser-on deafens DS RX)");
/*
 * DEFAULT TRUE = THE RANGING FIX. Skip our clean-room APC ignition and rely on
 * the A4 register image bosa_tx_enable loads, which already configures the
 * RTL8290B laser for correct BURST operation. apc_calibrate forced the laser into
 * a continuous-emission state that DEAFENED the shared-BOSA downstream RX -- the
 * whole multi-session "OLT never ranges us" wall. With apc_off the ONU reaches
 * O5: DS RX locks, the OLT sends Assign_ONU-ID and Ranging_Time, the FSM walks
 * O1..O5.
 */
static bool apc_off = true;	/* apc_off=false was RE-TESTED and BREAKS ranging: the ~3 s CW
			 * seating loop deafens the shared-BOSA DS-RX, so the OLT only ever sees
			 * "Initial" and never O5. And the apc_off=true laser is NOT weak: bias
			 * stable at 0x19, no fault, EN_L=0 throughout, O5 reached and held ~8 s
			 * before the OLT sends Deactivate=LOS. So the LOS is not a bias-seat
			 * problem -- it is the upstream BURST not being decodable despite a
			 * healthy laser, the same wall as rxsid=0. */
module_param(apc_off, bool, 0444);
MODULE_PARM_DESC(apc_off, "skip bosa_apc_calibrate (1=A4 image alone; 0=run APC to seat OFFK laser bias)");
/*
 * RTL8290B B-variant APC/OFFK ignition. DEFAULT FALSE so the shipping default is
 * unchanged (A4 image alone) and the new flow is A/B-revertible from the command
 * line. This board's laser is a B chip, and bosa_apc_calibrate runs the NON-B
 * flow whose OFFK never completes on it: it sets up the wrong FSM, so R29 never
 * reaches 0x3c and R30 bit7 OFFK_DONE stays 0. An un-nulled modulator emits DC
 * between bursts, which deafens the shared DS-RX. rtl8290b_apc_init runs the
 * B-variant FSU/OFFK to completion, keeps EN_L burst-gated and aborts on
 * MPD/no-feedback. Stock O5 targets: R29=0x3f, R30=0xa0, 0x204=0x8e.
 */
static bool apc_offk;		/* default OFF: OFFK converges (R29=0x3f) but does NOT lift WAN rate; not the WAN root cause */
module_param(apc_offk, bool, 0444);
MODULE_PARM_DESC(apc_offk, "run rtl8290b_apc_init B-variant OFFK ignition (completes modulator offset cal; default 0 = unchanged)");
/* TEMP default true (LAN+WiFi ACCESS build): hold the ONU FSM at O1 (no ranging),
 * so the GPON never deactivates/re-ranges and br-lan + the WiFi AP stay STABLE +
 * accessible (http://192.168.1.1/ and ONU-3282AE visible). GPON/WAN is disabled
 * while held. REVERT to default false (`static bool gpon_hold;`) to resume GPON
 * ranging once the US-OMCI egress fix lands. */
static bool gpon_hold;	/* default false: range to O5 normally (so the O5 selftest fires) */
module_param(gpon_hold, bool, 0444);
MODULE_PARM_DESC(gpon_hold, "hold the GPON FSM at O1 (no ranging) -> stable br-lan/WiFi for LAN+WiFi access (GPON/WAN disabled)");
/* gpon_sn_bytes is defined near the top: the onu_sn setter needs it. */
static struct timer_list gpon_fsm_timer;
static u8 gpon_fsm_state = 1;		/* O1 */
static u8 gpon_fsm_onu_id = 0xff;

/*
 * Deferred US-TX CDR-reset (re-range path). gpon_fsm_handle() runs in the FSM
 * timer = softirq, where the stock serdesCdr_reset (invert COM_REG08 bit15, hold
 * 10ms, restore) CANNOT run (mdelay in softirq is illegal). On a Deactivate->O1
 * re-range we schedule this work so the *correct, full* CDR-lock pulse runs in
 * process context — re-seating the TX serializer CDR fresh for the next ranging,
 * instead of the softirq-only WSDS_DIG_1D[16] re-strobe (a weaker primitive that
 * does not re-lock the CDR). */
static struct work_struct gpon_cdr_reset_work;

/*
 * Last upstream-burst-overhead parameters the OLT dictated. guard/ptn/delim
 * come from Upstream_Overhead (PLOAM 0x01); t3pre is the Type-3 pre-ranged
 * preamble length from Extended_Burst_Length (PLOAM 0x14). Both PLOAMs arrive
 * independently and are broadcast repeatedly, so retain them and recompute the
 * BOH from whichever arrived (gpon_apply_boh).
 */
static u8 gpon_boh_guard;			/* Upstream_Overhead d[0]   = guard bits */
static u8 gpon_boh_ptn = 0xaa;			/* Upstream_Overhead d[3]   = Type-3 pattern */
static u8 gpon_boh_delim[3] = { 0xab, 0x59, 0x83 };	/* Upstream_Overhead d[4..6] */
static u8 gpon_boh_t3pre;			/* Extended_Burst_Length d[0] = Type-3 pre-ranged len */
static u8 gpon_boh_t3ranged;			/* Extended_Burst_Length d[1] = Type-3 ranged len */
static u32 gpon_fsm_sn_tx;
/*
 * THE TICK PERIOD IS DEFINED ONCE, AND THE TWO HALVES ARE NOT THE SAME NUMBER.
 * We ASK the timer for GPON_FSM_TICK_REQ_MS; what we GET is whatever the jiffy
 * granularity allows, because msecs_to_jiffies() rounds UP:
 *     msecs_to_jiffies(10) at HZ=100  -> 1 jiffy  = 10 ms   (interaptiv/G24W)
 *     msecs_to_jiffies(10) at HZ=250  -> 3 jiffies = 12 ms  (taroko/X111W)
 * So a literal 10 in the arithmetic is right on one subtarget and 20% slow on the
 * other -- and HZ is a per-subtarget choice nobody revisits when a board is
 * added. Deriving it means the code cannot disagree with the timer.
 * It matters most for the core_fsm A/B: only the CORE is handed milliseconds,
 * while this driver's own timeouts count TICKS, so a literal here would have made
 * the A/B compare two FSMs across two different clocks and then blame the FSM.
 *
 * The REQUESTED period is tunable so the question can be measured rather than
 * argued: stock takes a GPON interrupt where we run a timer, so on taroko we
 * cannot SEE a downstream PLOAM for up to 12 ms -- and the OLT drops this board
 * 12 ms after the ONU-ID. At HZ=250 a jiffy is 4 ms, so 4 is the floor a jiffies
 * timer can reach and 10 (which rounds to 12) is what ships. A value below the
 * floor is not refused: msecs_to_jiffies rounds it up and GPON_FSM_TICK_MS
 * reports what the timer REALLY does. Default UNCHANGED at 10 -- this exists to
 * make an experiment possible, not to quietly re-tune a board that works.
 */
static uint gpon_fsm_tick_req_ms = 10;
module_param(gpon_fsm_tick_req_ms, uint, 0644);
MODULE_PARM_DESC(gpon_fsm_tick_req_ms,
	"requested FSM/PLOAM poll period in ms (default 10; the timer rounds UP to a jiffy, so at HZ=250 this becomes 12 -- 4 is the floor). Lower it to test whether downstream-PLOAM latency is what the OLT is reacting to");

#define GPON_FSM_TICK_REQ_MS	gpon_fsm_tick_req_ms
#define GPON_FSM_TICK_JIFFIES	msecs_to_jiffies(GPON_FSM_TICK_REQ_MS)
#define GPON_FSM_TICK_MS	((u32)jiffies_to_msecs(GPON_FSM_TICK_JIFFIES))

static u32 gpon_fsm_ticks;
static u8 gpon_sds_synced;	/* one-shot SDS TX re-sync done */
static u32 gpon_ds_rx;		/* total downstream PLOAMs drained (DS-lock liveness) */
static bool gpon_omcc_installed;	/* OMCC GEM datapath installed, on Configure_Port-ID */
/* The GEM the install SUCCEEDED on.  Deliberately NOT gpon_omcc_gem_port, which
 * gpon_install_omcc() stamps on entry and therefore also holds the port of an
 * install that FAILED -- believing that one would latch the failure as done and
 * never retry.  Written only beside the flag, cleared with it. */
static u16 gpon_omcc_installed_gem;
static bool gpon_tcont_installed;	/* OMCC T-CONT/alloc-id bound (one-shot, on Assign_Alloc-ID) */
static bool gpon_data_installed;	/* WAN data GEM (193) datapath installed (one-shot, on OMCI GEM-CTP create) */
static bool gpon_data_gem_solicited;	/* OLT has sent the OMCI GEM-CTP (ME268) Create -> only THEN install
					 * our data GEM, idempotently OVER the OLT's gem. Installing it
					 * proactively at PLOAM config (before the OLT's ME268) made the OLT
					 * unable to reconcile our gem on a 2nd+ admit and churn-lock (op=0xff
					 * reclaim->DEACT). Stock waits for the OLT's create. Cleared on Deactivate
					 * so each re-admit waits for the OLT's fresh ME268. Set from the eth OMCI rx. */
static bool gpon_data_tcont_installed;	/* the OLT's DATA Alloc-ID bound to the DATA T-CONT.
					 * SESSION STATE: cleared by EVERY teardown, like its three
					 * siblings above. It used to have no teardown clear at all --
					 * only the OLT's Assign_Alloc-ID DEALLOCATE, which additionally
					 * demands the Alloc-ID of the session that just ended. A
					 * re-config handing out a DIFFERENT one was then refused by the
					 * install guard, and the only recovery was waiting for an
					 * Alloc-ID the OLT will never send again: the WAN data T-CONT
					 * dark until a reboot. Pinned by gpon_data_bind{,_policy}_test. */
/* The WIRE GEM Port-ID the OLT assigned in its OMCI ME 268 (GEM Port Network CTP)
 * Create, attribute 1 — set from the eth OMCI RX snoop, and what the data-GEM
 * install actually programs. It is the OLT's to choose (measured on this lab OLT:
 * 223 to one board, 193 to another), exactly as the OMCC's GEM Port-ID comes from
 * Configure_Port-ID. GPON_DATA_GEM_DEFAULT is only the value held before the OLT
 * has spoken; the install is gated on gpon_data_gem_solicited, so it is never the
 * value that reaches the wire on a provisioned session. */
static u16 gpon_data_gem_port = GPON_DATA_GEM_DEFAULT;
static u16 gpon_omcc_alloc;	/* OMCC Alloc-ID override; 0 (default) = bind the LIVE ONU-ID.
		 * The OMCC's upstream Alloc-ID IS the ONU-ID (G.984.3 implicit default), and
		 * stock binds the GTC alloc-CAM entry to the live ONU-ID. The OLT grants on
		 * alloc = ONU-ID with NO Assign_Alloc-ID, the CAM resolves that grant and the
		 * framer drains. The OLD 0x100 default was WRONG: the CAM then held an alloc
		 * the OLT never grants, so every default-alloc grant MISSED it, the OMCC
		 * T-CONT was unreachable, DBRu reported 0 and the OLT never escalated the
		 * BWmap flags from overhead to payload. The "0x100 makes bwm_acpt>0" note was
		 * a RED HERRING: bwm_acpt counts ONU-ID-addressed grants, not CAM hits. */
#define GPON_OMCC_TCONT_ALT	1	/* Alternative T-CONT for alloc 0x100 (OLT's tcont 1) */
module_param(gpon_omcc_alloc, ushort, 0644);
MODULE_PARM_DESC(gpon_omcc_alloc, "OMCC Alloc-ID override (0=auto, 1=ONU-ID+1 for OLTs that grant T-CONT 1 not 0)");
static u16 gpon_data_alloc;		/* the OLT's data Alloc-ID, on T-CONT 8 */
/* data_tcont: route the WAN data GEM's upstream to the DATA T-CONT's queue
 * instead of riding the OMCC T-CONT's grants. It gates ONLY the SID2QID classify;
 * the T-CONT <- data-Alloc bind is driven by the OLT's Assign_Alloc-ID either way.
 *
 * DEFAULT ON since 2026-09-08, on a measurement (X111W on HSGQ-G008). The old
 * default was OFF for a reason the board's own registers REFUTE -- the OLT DID
 * assign a data Alloc-ID, every captured allocation carried the data T-CONT, and
 * its idle-byte counter climbed ~930 Mbps of grants FILLED WITH GTC IDLE while
 * the management T-CONT got ~nothing and the WAN data SID was classified to the
 * OMCC queue.
 * Measured end to end, ONU as the endpoint over gpon0, 20 s TCP each way:
 *   data SID on the OMCC T-CONT (old default):  US 0.205 Mbps   DS   8.49 Mbps
 *   data SID on the data T-CONT (this default): US 29.4  Mbps   DS 106.7  Mbps
 * and routed transit, iperf3 TCP:
 *   OFF: US 0.26 / DS 8.76 Mbps       ON: US 106.3 / DS 53.5 Mbps
 * The downstream figure is a CONSEQUENCE of the upstream one, not a second bug:
 * DS TCP is ACK-limited, and ~8.5 Mbps of DS needs ~0.17 Mbps of upstream ACKs,
 * which is the whole of the old US ceiling.
 *
 * =0 restores the OMCC-T-CONT routing for an OLT that really does grant a single
 * Alloc-ID for both; it is an A/B knob, no longer a claim about this lab. */
static bool data_tcont = true;	/* default ON: data US rides the DATA T-CONT 8's grants */
module_param(data_tcont, bool, 0644);
MODULE_PARM_DESC(data_tcont, "route the WAN data GEM US to the data T-CONT 8 queue 32 (default ON: measured 0.2 -> 106 Mbps upstream on HSGQ-G008; =0 rides the OMCC T-CONT 16 for an OLT that grants one Alloc-ID for both)");

static inline u32 gpon_rd(u32 off) { return ioread32(gpon_base + off); }
static inline void gpon_wr(u32 off, u32 v) { iowrite32(v, gpon_base + off); }

/*
 * RTL9607C SWCORE proxy: the I2C-indirect registers (0xB0-0xD8) are a decode hole
 * on the 9607C, reachable only via the switch-internal SMI master through PHY 10.
 * The SMI control regs (0x230B8..0x230C8) ARE directly mapped. On the 9602C the
 * 0xB0-0xD8 window decodes directly, so this proxy is 9607C-only.
 */
#define   SW_MDX_M_EN		BIT(10)

/* ★ THE PROXY BODIES ARE THE FAMILY'S, in luna_gpon_regs.h.  The two SMI
 * wrappers that used to sit beside these went with them: once sw_proxy_*
 * delegates to the family (which uses the family's own SMI), nothing here
 * called them any more and -Werror=unused-function said so.  These four were
 * written twice -- here and in rtl9607c_gpon.c -- and the copies were identical
 * (sw_proxy_rd/wr byte for byte, the SMI halves differing only in comments).
 * The wrappers keep their old names so every call site and this diff stay
 * small; the only thing they add is this file's own `swcore_base`. */


static inline u32 sw_proxy_rd(u32 swc_off)
{
	return luna_sw_proxy_rd(swcore_base, swc_off);
}

static inline void sw_proxy_wr(u32 swc_off, u32 val)
{
	luna_sw_proxy_wr(swcore_base, swc_off, val);
}

/*
 * SWCORE 32-bit access. On the 9607C route the I2C-indirect hole (0xB0-0xD8)
 * through the PHY-10 proxy; everything else (and all of the 9602C) is direct.
 */
static u32 sw_rd(u32 off)
{
	if (is_9607c && i2c_proxy && off >= 0xB0u && off <= 0xD8u)
		return sw_proxy_rd(off);
	return ioread32(swcore_base + off);
}

static void sw_wr(u32 off, u32 v)
{
	if (is_9607c && i2c_proxy && off >= 0xB0u && off <= 0xD8u) {
		sw_proxy_wr(off, v);
		return;
	}
	iowrite32(v, swcore_base + off);
}

static inline u32 pi_rd(u32 off) { return ioread32(ponip_base + off); }
static inline void pi_wr(u32 off, u32 v) { iowrite32(v, ponip_base + off); }

/* The three register blocks as flowcore `struct hwio`, so the field RMW below
 * uses hwio.h's ONE canonical mask arithmetic instead of three per-block
 * copies of it (a field one bit too wide reads back correct and behaves
 * wrong, which is why hwio.h exists). ctx is unused: the block bases are this
 * file's own statics, reached by the same sw_rd/sw_wr/pi_rd/pi_wr/gpon_rd/
 * gpon_wr accessors as every non-field access (the 9607C SMI-proxy window in
 * sw_rd/sw_wr included). */
static u32  sw_hwio_rd(void *ctx, u32 off)		{ return sw_rd(off); }
static void sw_hwio_wr(void *ctx, u32 off, u32 v)	{ sw_wr(off, v); }
static const struct hwio sw_io = { .rd = sw_hwio_rd, .wr = sw_hwio_wr };
static u32  pi_hwio_rd(void *ctx, u32 off)		{ return pi_rd(off); }
static void pi_hwio_wr(void *ctx, u32 off, u32 v)	{ pi_wr(off, v); }
static const struct hwio pi_io = { .rd = pi_hwio_rd, .wr = pi_hwio_wr };
static u32  gpon_hwio_rd(void *ctx, u32 off)		{ return gpon_rd(off); }
static void gpon_hwio_wr(void *ctx, u32 off, u32 v)	{ gpon_wr(off, v); }
static const struct hwio gpon_io = { .rd = gpon_hwio_rd, .wr = gpon_hwio_wr };

/* The shell's sleep for the shared indirect-CAM helper (regtable.h) -- the
 * r960_delay_us idiom from luna_ponmac.c: the flowcore tier cannot call
 * udelay() itself, time is an explicit input there so the same function runs
 * on x86 under the write-stream differential. */
static void gpon_cam_delay_us(unsigned int us)
{
	udelay(us);
}

/* gpon_ind_poll()/gpon_ind_go() (flowcore/regtable.h) take their per-iteration
 * wait as a void(*)(void): the core has no clock, so the PACE is the SHELL's --
 * the same split gpon_cam_delay_us() serves for the shared CAM helper above.
 * This is the 1 us pace every PON-IP indirect strobe in this file uses.  It is
 * deliberately NOT shared with tbl_pause(): the SWCORE table engine also has to
 * let the scheduler in, and that yield must not be paid on a PON-IP strobe. */
static void pi_pause_1us(void)
{
	udelay(1);
}

/*
 * PONIP_DBG_CTRL_US strobe -- ONE spelling of a sequence this file had TWICE
 * (the OMCC page probe and the data-flow page probe, 21 lines apart, differing
 * only in the SID). The poll belongs to the CORE (gpon_ind_go(), over this
 * file's &pi_io): byte-for-byte the shape that was here, so the same reads and
 * pauses land in the same places, with one difference -- the caller now gets
 * -ETIMEDOUT instead of a count it could not tell from a slow success.
 *
 * DELIBERATELY NOT MERGED:
 *  - the addresses stay BARE LITERALS, and NOT because nothing names them: the
 *    pair MOVED. The RTL9602C has DBG_CTRL_US 0x255c / SID_USED_PAGE_CNT_US
 *    0x2564, the RTL9603CVD's own chipdef has 0x26e8 / 0x26f0 and NOTHING at
 *    0x255c, and this function carries no chip guard -- so on that die these
 *    literals strobe two unmapped PON-IP offsets. The fix is a PI_X() name plus
 *    two luna_pi_moves_9603cvd[] rows; not done here because it changes which
 *    register the G24W writes, and a register change is measured before it
 *    lands. FINDING-the-sid-page-strobe-hits-unmapped-offsets-on-the-9603cvd.md
 *  - the counter is still read after an EXPIRED poll: refusing the read would
 *    change the bus stream and hand /proc an unexplained zero. The caller gets
 *    the rc instead, and both call sites print it.
 *  - nothing restores the stock word afterwards. The strobe LEAVES 0x255c
 *    holding 0x86000|sid|RD_MAX, exactly as it always has; a "helpful" restore
 *    would ADD a write that does not exist today.
 *
 * The 0x00086000 OR is the GUARD: writing only SID_NO+RD_MAX clears
 * DBG_IGNORE_TAG and CFG_US_EP_IPG, which re-breaks the US GEM encapsulation.
 * 0x255c = PONIP_DBG_CTRL_US {SID_NO[6:0], RD_MAX bit7, CLR bit8, BUSY bit9};
 * 0x2564 = PONIP_SID_USED_PAGE_CNT_US {USED[12:0], MAX[28:16]}.
 * Return: >= 0 is the iteration BUSY cleared at; < 0 is -ETIMEDOUT and *cnt is
 * then a STALE word, not this SID's.
 */
static int pi_sid_page_cnt(u32 sid, u32 *cnt)
{
	int rc = gpon_ind_go(&pi_io, 0x255c,
			     0x00086000u | (sid & 0x7fu) | (1u << 7),
			     1u << 9, 2000, pi_pause_1us);

	*cnt = pi_rd(0x2564);
	return rc;
}

/* Read-modify-write the bit-field [msb:lsb] of the SWCORE register at off. */
static void sw_field(u32 off, unsigned int msb, unsigned int lsb, u32 val)
{
	hwio_rmw(&sw_io, off, msb, lsb, val);
}

/* Read-modify-write the bit-field [msb:lsb] of the PON-IP register at off. */
static void pi_field(u32 off, unsigned int msb, unsigned int lsb, u32 val)
{
	hwio_rmw(&pi_io, off, msb, lsb, val);
}

/* Packed pi-register array entry write (shared locate: pi_packed_locate in
 * flowcore_hash.c). Defined below with the SID2QID helpers; forward-
 * declared so the early scheduler init writes WFQ_TYPE through the SAME
 * addressing as everything else. */
static void pi_packed_set(u32 base, unsigned int idx, unsigned int bits, u32 val);
static u32 pi_packed_get(u32 base, unsigned int idx, unsigned int bits);

/*
 * Front-panel PON/LOS LEDs. The stock unit lights the green PON LED once it
 * ranges to the OLT and the red LOS LED only while downstream light is lost;
 * OpenWrt shipped no LED support, so without this the panel sits in its power-
 * on state (LOS stuck on, PON dark). Driven entirely from the activation FSM.
 */
static bool gpon_leds = true;
module_param(gpon_leds, bool, 0644);
MODULE_PARM_DESC(gpon_leds, "drive the front-panel PON/LOS LEDs from GPON state (default on)");

static u32 gpon_led_pon_val = ~0u;	/* last value written (forces first update) */
static u32 gpon_led_los_val = ~0u;

/* Drive one LED index's 2-bit parallel force value (0=off 1=on 2=blink). */
static void gpon_led_force(unsigned int idx, u32 val)
{
	if (is_9603cvd)		/* Board-C offsets/indices -- see gpon_led_init() */
		return;
	sw_field(LED_FORCE_VALUE, idx * 2 + 1, idx * 2, val);
}

/* Claim one index for CPU-forced parallel output (parallel + pad + force-mode). */
static void gpon_led_claim(unsigned int idx)
{
	sw_field(LED_PARA_EN, idx + 1, idx + 1, 1);		/* parallel-enable   */
	sw_field(LED_IO_EN, idx, idx, 1);			/* pad-output enable */
	sw_field(LED_DATA_CFG(idx), LED_CPU_FORCE_BIT, LED_CPU_FORCE_BIT, 1);
}

/* Configure one LED index as a hardware-auto port-link/activity LED: the switch
 * lights it directly from the port's link state + traffic, no CPU involvement,
 * so plug/unplug is reflected with zero software. CPU_FORCE_MOD stays 0. */
static void gpon_led_port(unsigned int idx, u32 type)
{
	sw_wr(LED_DATA_CFG(idx), (type << 16) | LED_LINKACT);
	sw_field(LED_PARA_EN, idx + 1, idx + 1, 1);		/* parallel-enable   */
	sw_field(LED_IO_EN, idx, idx, 1);			/* pad-output enable */
}

/* Green PON LED: solid when operational (O5), blinking while ranging (O2..O4),
 * off when down (O1/unknown). */
static void gpon_led_pon_set(u8 st)
{
	u32 v = (st >= 5) ? LED_FORCE_ON : (st >= 2) ? LED_FORCE_BLINK : LED_FORCE_OFF;

	if (!gpon_leds || v == gpon_led_pon_val)
		return;
	gpon_led_force(PON_LED_IDX, v);
	gpon_led_pon_val = v;
}

/* Red LOS LED: on only while the downstream optical signal is lost. */
static void gpon_led_los_set(bool los)
{
	u32 v = los ? LED_FORCE_ON : LED_FORCE_OFF;

	if (!gpon_leds || v == gpon_led_los_val)
		return;
	gpon_led_force(LOS_LED_IDX, v);
	gpon_led_los_val = v;
}

/* Put the controller in parallel mode and claim the PON/LOS indices. Only the
 * two indices we own are touched, so any other panel LED keeps its power-on
 * configuration. */
static void gpon_led_init(void)
{
	if (!gpon_leds)
		return;
	/* Every constant in this block is Board C's, and on the RTL9603CVD one of them
	 * lands in a PIN-FUNCTION MUX. Measured 2026-08-27: IO_MODE_EN reads 0x08c0 on
	 * stock and 0xb8c2 on ours, and OUR extra bits {1,12,13,15} are EXACTLY this
	 * block's LED indices written through LED_IO_EN = 0x23014, the Board-C literal
	 * that is IO_MODE_EN on this die -- so each "LED pad enable" is a pin-function
	 * STEAL (HS_UART_FC, I2C1, SLIC_ISI, DYING).
	 * It is NOT the GPON-RX gate: clearing the four bits back to stock's value on
	 * the live board left SDS_SDET at 0, consistent with SDS_SDET being the
	 * SerDes-internal comparator on dedicated, un-muxed RX pins. The steal is a
	 * real defect all the same, and this die's LED bring-up needs its OWN offsets
	 * and indices before any LED write may run here. */
	if (is_9603cvd) {
		pr_info("luna-gpon: panel LEDs skipped (%s: Board-C LED map; LED_IO_EN 0x23014 is IO_MODE_EN here -- measured steal of HS_UART_FC/I2C1/SLIC_ISI/DYING pins)\n",
			swc->chip);
		return;
	}
	sw_field(LED_PARA_EN, LED_SERI_DATA_EN_BIT, LED_SERI_DATA_EN_BIT, 0);
	sw_field(LED_PARA_EN, LED_SERI_CLK_EN_BIT, LED_SERI_CLK_EN_BIT, 0);
	sw_field(LED_IO_EN, LED_SERI_OUT_EN_BIT, LED_SERI_OUT_EN_BIT, 0);
	sw_field(LED_MODE_SEL, 0, 0, 0);			/* parallel output   */
	sw_field(LED_BLINK_RATE, 14, 12, LED_BLINK_512MS);
	gpon_led_claim(PON_LED_IDX);
	gpon_led_claim(LOS_LED_IDX);
	gpon_led_force(PON_LED_IDX, LED_FORCE_OFF);
	gpon_led_force(LOS_LED_IDX, LED_FORCE_OFF);
	gpon_led_pon_val = LED_FORCE_OFF;
	gpon_led_los_val = LED_FORCE_OFF;

	/* Ethernet port-link LEDs, hardware-auto: the switch lights each from its
	 * OWN port's link + activity, no CPU. FE = index 15 (switch port0, UTP0),
	 * GE = index 1 (switch port1, UTP1) -- both confirmed by cable test, so an
	 * FE-plug lights only FE and a GE-plug lights only GE. */
	gpon_led_port(FE_LED_IDX, LED_TYPE_UTP0);
	gpon_led_port(GE_LED_IDX, LED_TYPE_UTP1);

	pr_info("luna-gpon: panel LEDs init (PON idx%u / LOS idx%u force-mode; FE idx%u / GE idx%u link-auto)\n",
		PON_LED_IDX, LOS_LED_IDX, FE_LED_IDX, GE_LED_IDX);
}


/* Read-modify-write the bit-field [msb:lsb] of the GPON-block register at off.
 * Same canonical mask RMW as sw_field/pi_field (flowcore hwio.h). */
static void gpon_field(u32 off, unsigned int msb, unsigned int lsb, u32 val)
{
	hwio_rmw(&gpon_io, off, msb, lsb, val);
}

/* DEV live register poke (uses the driver's own ioremap — no /dev/mem, immune to
 * STRICT_DEVMEM). Write "<g|G|p|P> <hexoff> [hexval]" to
 * /sys/module/gpon_rtl9602c/parameters/poke :  g/G = GPON-GTC read/write,
 * p/P = PON-IP read/write, b/B = BOSA I2C read/write. Result is logged via dmesg. Lets us
 * test US-egress register experiments (watch /proc/gpon idle16/gemus_omcc/bwm_acpt) without a rebuild. */
static int bosa_read_reg(u16 reg);	/* fwd decl: poke_set 'b'/'B' use these (defined below) */
static int bosa_write_reg(u16 reg, u8 val);
static char poke_buf[8];
static int poke_set_locked(const char *val, const struct kernel_param *kp);
static int poke_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&bosa_lock);
	ret = luna_stopping ? -ESHUTDOWN : poke_set_locked(val, kp);
	mutex_unlock(&bosa_lock);
	return ret;
}

static int poke_set_locked(const char *val, const struct kernel_param *kp)
{
	char c = 0;
	unsigned int off = 0, v = 0;
	int n = sscanf(val, " %c %x %x", &c, &off, &v);

	if (n < 2) {
		pr_info("luna-gpon: poke usage: <g|G|p|P> <hexoff> [hexval]\n");
		return 0;
	}
	/* Raw writes cannot bypass the indirect-CAM owner or republish a live
	 * data path behind its ledger. Read-only diagnostics remain available. */
	if ((c == 'G' || c == 'P' || c == 'M') && READ_ONCE(luna_driver_ready))
		return -EBUSY;
	switch (c) {
	case 'g':
		pr_info("luna-gpon: poke GTC[%#x]=%#x\n", off, gpon_rd(off));
		break;
	case 'G':
		gpon_wr(off, v);
		pr_info("luna-gpon: poke GTC[%#x]<=%#x ->%#x\n", off, v, gpon_rd(off));
		break;
	case 'p':
		pr_info("luna-gpon: poke PI[%#x]=%#x\n", off, pi_rd(off));
		break;
	case 'P':
		pi_wr(off, v);
		pr_info("luna-gpon: poke PI[%#x]<=%#x ->%#x\n", off, v, pi_rd(off));
		break;
	case 'm': {	/* generic phys read (GMAC 0x18012048 CPUTAGCR, SWCORE 0x1b00xxxx) */
		void __iomem *a = ioremap(off, 4);
		if (a) { pr_info("luna-gpon: poke MEM[%#x]=%#x\n", off, ioread32(a)); iounmap(a); }
		break;
	}
	case 'M': {	/* generic phys write */
		void __iomem *a = ioremap(off, 4);
		if (a) { iowrite32(v, a); pr_info("luna-gpon: poke MEM[%#x]<=%#x ->%#x\n", off, v, ioread32(a)); iounmap(a); }
		break;
	}
	case 'b':	/* BOSA I2C read (12-bit reg; slave banking internal) — live US-TX-vs-stock diff */
		pr_info("luna-gpon: poke BOSA[%#x]=%#x\n", off, bosa_read_reg(off));
		break;
	case 'B':	/* BOSA I2C write — live laser/extinction tweak */
		bosa_write_reg(off, v);
		pr_info("luna-gpon: poke BOSA[%#x]<=%#x ->%#x\n", off, v, bosa_read_reg(off));
		break;
	default:
		pr_info("luna-gpon: poke bad cmd '%c'\n", c);
		break;
	}
	return 0;
}
static const struct kernel_param_ops poke_ops = { .set = poke_set };
module_param_cb(poke, &poke_ops, poke_buf, 0644);
MODULE_PARM_DESC(poke, "DEV: <g|G|p|P> <hexoff> [hexval] live GTC/PON-IP reg read/write");

/* RTL8290B BOSA state captured at probe for /proc display (-1 = not read). */
/* Is the optical module actually an RTL8290B? (2026-08-26)
 * This driver's whole BOSA register model -- the page mapping, the analog
 * RSSI/APC banks, the laser-servo writes -- is the RTL8290B's. The G24W's module
 * is NOT one, and said so once the I2C pads were routed: slave 0x50 answers real
 * SFF-8472 identity bytes while the RTL8290B chip-ID at 0x390 reads 0x0000. The
 * "pages" 0x54/0x55 are the SAME device answering, which is why every byte we
 * sampled there read 0x20 -- an ASCII space from a padded identity field.
 * READS off a wrong model are merely wrong; WRITES are not, because
 * bosa_i2c_write8() would put laser-servo values into the module's identity
 * EEPROM. So the write path is gated on a POSITIVE identification and fails
 * SAFE: an RTL8290B reads 0x8290 and nothing changes.
 */
static bool bosa_not_8290b;		/* set by bosa_probe() on a positive mismatch */
static int bosa_id_num __ro_after_init = -1;
static int bosa_id_vid __ro_after_init = -1;
static int bosa_w41 __ro_after_init = -1;
static int bosa_ctrl2 __ro_after_init = -1;
static int bosa_status2 __ro_after_init = -1;

/*
 * Does this board HAVE an RTL8290B register interface to read? A POSITIVE
 * identification, the same bar the write path applies.
 * A register nobody could ask renders n/a, NEVER a number: on the G24W the
 * module answers only its SFF-8472 pages and slaves 0x54/0x55 NACK every
 * register (measured 2026-09-07). Printing a byte for them turned the previous
 * read's data into "bias=0x20 mod=0x20 apc_done=0", which reads as a DEVICE
 * finding -- a laser that is not biased -- and that phantom is what an
 * investigation of this board's ranging failure was built on.
 */
static bool bosa_regs_live(void)
{
	return bosa_id_num == 0x8290 && !bosa_not_8290b;
}

/*
 * Read one 8-bit register from an I2C slave via the SoC hardware I2C master on
 * bus 0. Returns the byte (0..0xff), or negative on NACK/timeout. This is a
 * read-only path — nothing is written to the BOSA — so it is safe to run
 * unconditionally during bring-up.
 */
/*
 * Wait for ONE indirect I2C transaction to COMPLETE. -> 0, or a negative errno.
 * `cmd_out` (optional) receives the last command word read.
 *
 * POLL CMD_EN, NOT BUSY ALONE -- measured on the G24W 2026-09-07, and it is why
 * every RTL8290B register this driver printed on that board was a phantom. The
 * three loops this replaces tested only I2C_CMD_BUSY, read in the instruction
 * right after writing CMD_EN: at that instant the controller has not raised BUSY
 * and NACK still carries the PREVIOUS transaction's value, so "not busy" passes
 * on a transaction that has not started and the accessor returns whatever
 * I2C_IND_RD still holds. Measured side by side: slaves 0x54 and 0x55 NACK EVERY
 * register on the G24W while /proc/gpon reported 0x20 for each -- the ASCII space
 * the preceding slave-0x50 read had left in I2C_IND_RD.
 * CMD_EN SELF-CLEARS when the transaction ends, so "CMD_EN still set" is the one
 * unambiguous "not finished yet", and running out of budget on it is
 * COULD-NOT-ASK, never a value.
 *
 * And the status is not read before the transaction can have happened: testing
 * CMD_EN as well as BUSY was not enough on its own, because BOTH bits read back 0
 * in the instruction after the command write. One 8-bit-address read at 100 kHz
 * is ~38 bit times, ~380 us, and I2C_XACT_SETTLE_US covers that with margin
 * before the first status read. This is a probe path, never a packet path.
 */
#define I2C_XACT_SETTLE_US	600u
/*
 * ⚠ THE NACK IS LATCHED ACROSS THE WHOLE TRANSACTION, not read off the last
 *   sample -- because the VENDOR'S OWN routine latches it, and that is the only
 *   thing established here. dal_rtl9603cvd_i2c_read accumulates the bit across
 *   its polls; ours read only the terminal word, so for the CMD sample
 *   sequence 0x0d, 0x05, 0x00 the two disagree: stock errors, ours returned the
 *   byte as data. The sequence is INJECTED, and no measurement on this bench
 *   says the silicon produces it -- what is proven is a software divergence
 *   from the vendor, on a path where being wrong means a register silently not
 *   written in a 639-write calibration stream.
 *
 * ⚠ WHAT IS DELIBERATELY NOT COPIED: stock completes on BUSY alone, which
 *   accepts a command still carrying CMD_EN on the very first sample. The
 *   CMD_EN|BUSY test here is the stricter one and it stays, as do the settle
 *   and the bounded poll.
 */
static int i2c_wait_done(u32 ind_cmd, u32 *cmd_out)
{
	bool nacked = false;
	int i;

	udelay(I2C_XACT_SETTLE_US);
	for (i = 0; i < I2C_BUSY_POLL_MAX; i++) {
		u32 cmd = sw_rd(ind_cmd);

		if (cmd_out)
			*cmd_out = cmd;
		nacked |= !!(cmd & I2C_CMD_NACK);
		if (!(cmd & (I2C_CMD_EN | I2C_CMD_BUSY)))
			return nacked ? -EIO : 0;
		udelay(10);
	}
	return -ETIMEDOUT;
}

static int bosa_i2c_read8(u8 slave, u8 reg)
{
	u32 cfg, pad_off = SOC_IO_MODE_EN;	/* per-chip: gpon_swc_map */
	u32 i2c_bus0 = IO_I2C_EN_BUS0;
	/* Was a per-chip ternary here; the +4 is now in gpon_swc_map, so the
	 * write path gets it too. */
	u32 ind_adr = I2C_IND_ADR;
	u32 ind_cmd = I2C_IND_CMD;
	u32 ind_rd  = I2C_IND_RD;
	int ret;

	lockdep_assert_held(&bosa_lock);

	/* Route I2C bus 0 to its pads. I2C_EN is a 2-bit field (one bit per bus)
	 * whose position MOVES: [14:13] on the 9602C/9607C, [12:11] on the
	 * 9603CVD. Writing bit 13 on a 9603CVD hits SLIC_ISI_EN, not I2C. */
	sw_field(pad_off, i2c_bus0, i2c_bus0, 1);

	/* CONFIG: slave addr, 8-bit reg-addr + 8-bit data, the existing divider. Preserve the
	 * electrical bits (open-drain / mode / delays) already programmed. */
	cfg = sw_rd(I2C_CONFIG0);
	cfg &= ~((((1u << 7) - 1) << I2C_CFG_DEV_ID_LSB) |
		 (0x3u << I2C_CFG_AW_LSB) | (0x3u << I2C_CFG_DW_LSB) |
		 (0x3ffu << I2C_CFG_CLKDIV_LSB));
	cfg |= ((u32)(slave & 0x7f) << I2C_CFG_DEV_ID_LSB) |
	       (I2C_CLKDIV_50K << I2C_CFG_CLKDIV_LSB);
	sw_wr(I2C_CONFIG0, cfg);

	sw_wr(ind_adr, reg);
	sw_wr(ind_cmd, I2C_CMD_EN);			/* RW_EN=0 -> read */

	ret = i2c_wait_done(ind_cmd, NULL);
	if (!ret)
		ret = (int)(sw_rd(ind_rd) & 0xff);

	/* Reconnect the shared optical-SD pad so optic_los stays live (see
	 * bosa_i2c_restore_pad). */
	if (bosa_i2c_restore_pad)
		sw_field(pad_off, i2c_bus0, i2c_bus0, 0);
	return ret;
}

/*
 * RTL9607C optical-module probe (M3 diagnostic): read the SFF-8472 module over
 * I2C (through the SMI proxy) to answer "is downstream light reaching the
 * module?". A0 (0x50) = static identity (vendor name @bytes 20..35); A2 (0x51) =
 * live diagnostics incl. RX optical power (bytes 104..105, big-endian, 0.1uW/LSB).
 * Also self-tests the proxy against a directly-mapped register.
 */
static void ddm_probe_9607c(void)
{
	u32 direct, proxy;
	int v0, v1, i, lane;
	u16 rxpwr;
	char vendor[17];

	/* SerDes-lane scan: SDS_FIB_STATUS is a 3-lane array (0x28c, stride 0x20).
	 * Look for signal-detect on ANY lane in case the optical RX is not lane 0. */
	for (lane = 0; lane < 3; lane++) {
		u32 fs = ioread32(swcore_base + 0x28cu + lane * 0x20u);

		pr_info("luna-gpon: SCAN sds_lane%d fib_sts=0x%08x sdet=%u link_ok=%u fib100=%u\n",
			lane, fs, !!(fs & BIT(17)), !!(fs & BIT(4)), !!(fs & BIT(2)));
	}

	direct = ioread32(swcore_base + SDS_CFG);	/* SDS_CFG: directly mapped */
	proxy  = sw_proxy_rd(SDS_CFG);
	pr_info("luna-gpon: DDM proxy self-test SDS_CFG direct=0x%x proxy=0x%x %s\n",
		direct, proxy, direct == proxy ? "OK" : "PROXY-BROKEN");

	{
		int raw[16];

		for (i = 0; i < 16; i++)
			raw[i] = bosa_i2c_read8(0x50, 20 + i);
		bosa_sff_text(vendor, raw, 16);	/* sanitize: luna_gpon_logic.c */
	}
	pr_info("luna-gpon: DDM A0(0x50) vendor='%s'\n", vendor);

	v0 = bosa_i2c_read8(0x51, 104);
	v1 = bosa_i2c_read8(0x51, 105);
	if (v0 < 0 || v1 < 0) {
		pr_info("luna-gpon: DDM A2(0x51) rx_power read failed (v0=%d v1=%d) — module/i2c not responding\n",
			v0, v1);
		return;
	}
	rxpwr = ((u16)v0 << 8) | (u16)v1;
	pr_info("luna-gpon: DDM A2 rx_power raw=0x%04x = %u.%u uW (light %s)\n",
		rxpwr, rxpwr / 10, rxpwr % 10, rxpwr ? "PRESENT" : "ABSENT");
}

/* Read one byte from (bus, slave, reg); *cmd_out = raw I2C_IND_CMD (NACK bit3). */
static int i2c_rd_bus(int bus, u8 slave, u8 reg, u32 *cmd_out)
{
	/* Per-bus stride off the chip's OWN indirect block. Every constant here
	 * is the header's, not a second spelling of it: this scanner and the
	 * read/write paths above cannot drift apart. The pad-mux comes from the
	 * chip table rather than the 9607C literal -- identical on this chip
	 * (io_mode_en 0x23014, i2c_en_bus0 13), but correct by construction
	 * instead of correct because the only call site happens to be guarded. */
	u32 cfg_off = I2C_CONFIG0 + bus * I2C_BUS_STRIDE;
	u32 ind_adr = I2C_IND_ADR + bus * I2C_BUS_STRIDE;
	u32 ind_cmd = I2C_IND_CMD + bus * I2C_BUS_STRIDE;
	u32 ind_rd  = I2C_IND_RD  + bus * I2C_BUS_STRIDE;
	int ret;
	u32 cfg, cmd = 0;

	sw_field(SOC_IO_MODE_EN, IO_I2C_EN_BUS0 + bus, IO_I2C_EN_BUS0 + bus, 1);
	cfg = sw_rd(cfg_off);
	cfg &= ~((((1u << 7) - 1) << I2C_CFG_DEV_ID_LSB) |
		 (0x3u << I2C_CFG_AW_LSB) | (0x3u << I2C_CFG_DW_LSB) |
		 (0x3ffu << I2C_CFG_CLKDIV_LSB));
	cfg |= ((u32)(slave & 0x7f) << I2C_CFG_DEV_ID_LSB) |
	       (I2C_CLKDIV_50K << I2C_CFG_CLKDIV_LSB);
	sw_wr(cfg_off, cfg);
	sw_wr(ind_adr, reg);
	sw_wr(ind_cmd, I2C_CMD_EN);				/* read */
	ret = i2c_wait_done(ind_cmd, &cmd);
	if (!ret)
		ret = (int)(sw_rd(ind_rd) & 0xff);
	if (cmd_out)
		*cmd_out = cmd;
	return ret;
}

/* Scan both i2c buses for the common optical-module slaves. */
static void i2c_scan_9607c(void)
{
	/* 0x50 and 0x51 are the SFF-8472 standard addresses: page 0 / slave 0x50 is the
	 * A0 identification page and page 1 / slave 0x51 the A2 diagnostics page. 0x54
	 * is vendor space, named in this tree as the analog/APC "W" register page for
	 * Board C's RTL8290B.
	 * Page 3 / slave 0x55 exists in bosa_slave_for and is deliberately NOT scanned;
	 * this RECORDS the gap rather than closing it, because nothing here establishes
	 * whether the omission was intended. */
	static const u8 slaves[] = {
		0x50,	/* SFF-8472 A0 identification page */
		0x51,	/* SFF-8472 A2 diagnostics page (DDM) */
		0x54,	/* vendor page -- the analog/APC "W" registers on Board C */
	};
	int bus;
	unsigned int k;

	/* ★ WAS `k < 3`, a hand-written bound beside a three-entry array. It could
	 * only ever drift one way: add a slave and the scan silently keeps probing
	 * three. ARRAY_SIZE cannot. (k is unsigned because ARRAY_SIZE is size_t.) */
	for (bus = 0; bus < 2; bus++)
		for (k = 0; k < ARRAY_SIZE(slaves); k++) {
			u32 cmd = 0;
			int b = i2c_rd_bus(bus, slaves[k], 0, &cmd);

			pr_info("luna-gpon: SCAN i2c bus%d slave0x%02x reg0=%d cmd=0x%08x %s\n",
				bus, slaves[k], b, cmd,
				(cmd & I2C_CMD_NACK) ? "NACK(no-dev)" :
				(cmd & I2C_CMD_BUSY) ? "BUSY(timeout)" : "ACK");
		}
}

/*
 * Probe the external RTL8290B "Europa" BOSA over I2C by reading its chip ID
 * (NUM=0x8290, VID=0x0001). Read-only — this validates the I2C transport
 * end-to-end against known-good values before any RX/signal-detect-enable
 * writes are added. The real optical signal-detect (SDS_FIB_STATUS.SDS_SDET)
 * only asserts once the BOSA RX path is brought up, which is the next step.
 */
static int bosa_read_reg(u16 reg)
{
	return bosa_i2c_read8(bosa_slave_for(reg), reg & 0xff);
}

/*
 * Read a big-endian 16-bit value from two consecutive BOSA registers — used for
 * the SFF-8472 DDM optical-power words the RTL8290B MCU publishes on its A2/DDM
 * page (slave 0x51 = the 0x1xx bank): RX optical power @0x68/0x69, TX @0x66/0x67,
 * unit 0.1 microWatt/LSB. reg_hi is the high byte, reg_hi+1 the low.
 */
static int bosa_read16(u16 reg_hi)
{
	int h = bosa_read_reg(reg_hi);
	int l = bosa_read_reg(reg_hi + 1);

	if (h < 0 || l < 0)
		return -1;
	return ((h & 0xff) << 8) | (l & 0xff);
}

/* Median of three optical samples, taken under the caller's group mutex.
 * Returns -1 only if every sample failed. */
static int bosa_read16_median(u16 reg_hi)
{
	u32 v[3];
	unsigned int n = 0;
	int i;

	for (i = 0; i < 3; i++) {
		int r = bosa_read16(reg_hi);

		if (r >= 0)
			v[n++] = (u32)r;
	}
	if (n == 0)
		return -1;
	/* selection rule (sort + upper median): luna_gpon_logic.c */
	return (int)bosa_median_u32(v, n);
}

/* --- ANI-G (ME 263) live optical levels ------------------------------------
 * The OMCI GET runs in softirq (GFP_ATOMIC) and must not busy-wait on the I2C
 * bus, so the DDM optical words are sampled by a periodic workqueue (~3 s, as
 * stock's separate DDM thread does) into this cache; the GET just returns the
 * cache. Values use the
 * G.988 ANI-G encoding (2's-complement s16, 0.002 dB referred to 1 mW). The
 * defaults are the stock first-light snapshot, used until the first good read. */
static s16 anig_rx_level = (s16)0xeedc;		/* #10 Optical signal level (DS RX) */
static s16 anig_tx_level = (s16)0x04d7;		/* #14 Transmit optical level (TX)  */

/* Sample the calibrated DDM optical words and refresh the ANI-G level cache.
 * Called from process work at a slow cadence (never from the GET path);
 * a "not available" word keeps the previous cached value. */
static void gpon_optical_cache_poll(void)
{
	s32 v;

	v = ddm_word_to_level(bosa_read16_median(0x168));	/* RX @0x68/0x69 */
	if (v != INT_MIN)
		anig_rx_level = (s16)v;
	v = ddm_word_to_level(bosa_read16_median(0x166));	/* TX @0x66/0x67 */
	if (v != INT_MIN)
		anig_tx_level = (s16)v;
}

/* Exported to the OMCI responder (rtl9602c_eth.c) for the ANI-G GET: returns the
 * cached live optical levels. No I2C here, so it is safe to call from softirq. */
void gpon_anig_optical_omci(s16 *rx_level, s16 *tx_level)
{
	*rx_level = anig_rx_level;
	*tx_level = anig_tx_level;
}

/* One process worker owns the 50 ms RTL servo and separate 3 s DDM. */
static struct delayed_work gpon_optical_work;
static void gpon_optical_work_fn(struct work_struct *w);

/*
 * Write one 8-bit register to an I2C slave via the SoC HW I2C master (bus 0).
 * Same indirect kick as the read, with RW_EN set and the data staged in
 * I2C_IND_WD. Not __init: the laser-maintenance work re-runs the write path at
 * runtime to service TX faults, so the whole write helper family must survive
 * past boot.
 */
static int bosa_i2c_write_raw(u8 slave, u8 reg, u8 val)
{
	u32 cfg;
	int ret;


	lockdep_assert_held(&bosa_lock);

	sw_field(SOC_IO_MODE_EN, IO_I2C_EN_BUS0, IO_I2C_EN_BUS0, 1);

	cfg = sw_rd(I2C_CONFIG0);
	cfg &= ~((((1u << 7) - 1) << I2C_CFG_DEV_ID_LSB) |
		 (0x3u << I2C_CFG_AW_LSB) | (0x3u << I2C_CFG_DW_LSB) |
		 (0x3ffu << I2C_CFG_CLKDIV_LSB));
	cfg |= ((u32)(slave & 0x7f) << I2C_CFG_DEV_ID_LSB) |
	       (I2C_CLKDIV_50K << I2C_CFG_CLKDIV_LSB);
	sw_wr(I2C_CONFIG0, cfg);

	sw_wr(I2C_IND_ADR, reg);
	sw_wr(I2C_IND_WD, val);
	sw_wr(I2C_IND_CMD, I2C_CMD_EN | I2C_CMD_RW_WR);

	ret = i2c_wait_done(I2C_IND_CMD, NULL);

	/* Reconnect the shared optical-SD pad so optic_los stays live (see
	 * bosa_i2c_restore_pad). */
	if (bosa_i2c_restore_pad)
		sw_field(SOC_IO_MODE_EN, IO_I2C_EN_BUS0, IO_I2C_EN_BUS0, 0);
	return ret;
}

static int bosa_i2c_write8(u8 slave, u8 reg, u8 val)
{
	/* Refuse to write RTL8290B registers into a module that is not one. Reads stay
	 * allowed and still useful.
	 * A POSITIVE identification, which this WAS NOT: it was gated on a NEGATIVE
	 * flag a STRING could suppress -- bosa_not_8290b is set only in bosa_probe()'s
	 * SFF-8472 branch, and on a module whose EEPROM part name contains "RTL8290" an
	 * earlier branch wins and leaves the flag false. Measured on the G24W
	 * 2026-09-01: the chip ID reads 0x0000, the part name reads "RTL8290", so the
	 * driver logged "register path stays ENABLED" and went on to apply a laser
	 * config whose 0x20s are ASCII SPACES from the module's padded identity
	 * strings -- it was writing an identity EEPROM believing it was a laser servo.
	 * A NAME IS NOT EVIDENCE OF STRUCTURE: the chip-ID read decides. bosa_id_num
	 * < 0 means "never read" and is also refused, because "could not ask" is not
	 * "yes". No-op on the X111W, whose module answers 0x8290. */
	if (bosa_not_8290b || bosa_id_num != 0x8290) {
		pr_warn_once("luna-gpon: BOSA writes REFUSED -- no POSITIVE RTL8290B identification (chip id=0x%04x, want 0x8290%s); reg 0x%02x@0x%02x not written. A part-name string in the module's EEPROM is not evidence that its register interface exists.\n",
			     bosa_id_num,
			     bosa_not_8290b ? ", and SFF-8472 says it is another part" : "",
			     reg, slave);
		return -ENODEV;
	}

	return bosa_i2c_write_raw(slave, reg, val);
}

static int luna_gn_rd(void *ctx, u8 slave, u8 reg, u8 *val)
{
	int ret = bosa_i2c_read8(slave, reg);

	(void)ctx;
	if (ret < 0)
		return ret;
	*val = ret;
	return 0;
}

static int luna_gn_wr(void *ctx, u8 slave, u8 reg, u8 val)
{
	(void)ctx;
	return bosa_i2c_write_raw(slave, reg, val);
}

/* Caller holds bosa_lock across the entire paged probe and calibration. */
static int luna_gn_calibrate_locked(void)
{
	const struct firmware *fw;
	struct gn_op *ops;
	u8 notes[GN_PROBE_NOTES];
	struct gn_io io = { .rd = luna_gn_rd, .wr = luna_gn_wr,
		.notes = notes, .notes_max = ARRAY_SIZE(notes) };
	struct gn_fail fail = { 0 };
	int ret, n;

	if (bosa_regs_live())
		return -ENODEV;
	if (bosa_cal_ready)
		return 0;
	ret = request_firmware_direct(&fw, "rtkbosa_k.bin", NULL);
	if (ret)
		return ret;
	if (fw->size != GN_CAL_LEN) {
		ret = -EINVAL;
		goto release;
	}
	/* 7200 bytes exceeds the safe budget of the MIPS 8 KiB kernel stack. */
	ops = kcalloc(GN_OPS_MAX, sizeof(*ops), GFP_KERNEL);
	if (!ops) {
		ret = -ENOMEM;
		goto release;
	}
	if (!bosa_gn_identified) {
		n = gn25l95_probe_ops(ops, GN_OPS_MAX);
		ret = n < 0 ? n : gn25l95_cal_apply(ops, n, &io, &fail);
		if (ret)
			goto free;
		if (io.notes_n != GN_PROBE_NOTES || gn25l95_is_gn28l9x(notes)) {
			ret = -ENODEV;
			goto free;
		}
		/* This on-board part cannot change at runtime. Retain its positive
		 * physical identity after a partial calibration changes the cold
		 * probe's password predicate. An SFF name never establishes identity. */
		bosa_gn_identified = true;
	}
	io.notes = NULL;
	io.notes_max = 0;
	io.notes_n = 0;
	n = gn25l95_cal_ops(fw->data, fw->size, &gn_variant_g24w,
			      ops, GN_OPS_MAX);
	ret = n < 0 ? n : gn25l95_cal_apply(ops, n, &io, &fail);
	if (!ret)
		bosa_cal_ready = true;
free:
	if (ret)
		pr_err("luna-gpon: GN calibration refused: err=%d op=%u reg=0x%02x\n",
		       ret, fail.op, fail.reg);
	kfree(ops);
release:
	release_firmware(fw);
	return ret;
}

static int bosa_write_reg(u16 reg, u8 val)
{
	return bosa_i2c_write8(bosa_slave_for(reg), reg & 0xff, val);
}

/* Single-bit read-modify-write of a BOSA register. */
/* Masked field read-modify-write: val is the field value, placed at the mask's
 * low bit. */
static void bosa_set_field(u16 reg, u8 mask, u8 val)
{
	int r = bosa_read_reg(reg);
	u8 shift;

	if (r < 0 || !mask)
		return;
	shift = __ffs(mask);
	bosa_write_reg(reg, (r & ~mask) | ((val << shift) & mask));
}

/* One bit is a one-bit FIELD.  Every caller passes bit 0..7, so the mask fits
 * the u8 bosa_set_field takes, and (r & ~m) | ((set << bit) & m) computes the
 * same bytes the open-coded or/and-not did -- checked over all 97 call sites
 * before this became a wrapper (2026-09-04). */
static void bosa_set_bit(u16 reg, u8 bit, int set)
{
	bosa_set_field(reg, (u8)(1u << bit), set ? 1 : 0);
}

/* Read a masked field, right-justified. Returns 0 on I2C error. */
static u8 bosa_get_field(u16 reg, u8 mask)
{
	int r = bosa_read_reg(reg);

	if (r < 0 || !mask)
		return 0;
	return (r & mask) >> __ffs(mask);
}

/* Bounded poll of a BOSA status bit. Returns 1 if the bit reached @want before
 * the cap, 0 on timeout. @us is the per-iteration delay. */
static int bosa_poll_bit(u16 reg, u8 bit, int want, unsigned int us, int cap)
{
	int i;

	for (i = 0; i < cap; i++) {
		int r = bosa_read_reg(reg);

		if (r >= 0 && !!(r & (1u << bit)) == !!want)
			return 1;
		udelay(us);
	}
	return 0;
}

/*
 * RTL8290B optical DDM: raw ADC -> calibrated dBm / temp / Vcc / bias,
 * re-expressed from the stock RX-power path (sigma-delta ADC -> ratiometric RSSI
 * voltage -> endpoint scale -> per-board polynomial -> log). Two independent
 * mistakes made the naive readout (one 8-bit tap at 0x311 plus a linear fit)
 * wrong by ~10 dB:
 *   1. RX power in dBm is LOGARITHMIC in a linear 0.1 uW code, so a straight
 *      line cannot track it and the error grows toward the tails.
 *   2. The live RX signal is a 24-bit ratiometric sigma-delta ADC normalised
 *      against the on-die reference taps, not one byte of one gain range.
 * Computed ON DEMAND at the /proc read: stock's periodic DDMI kthread only warms
 * a cache, nothing external needs a periodic write, and a new periodic I2C poll
 * would risk the ~10 ms laser-servo timing. All fixed-point, explicit byte math.
 */

/*
 * "Could not measure" sentinels for the RX optical chain. BOSA_RX_CODE_NA (0)
 * lives in luna_gpon_logic.h beside bosa_rx_code_calc(), which owns the
 * floor-at-11 rule that keeps 0 free to mean "no reading". BOSA_RX_CDBM_NA is
 * deliberately NOT -4000 (the dBm floor a real dark reading produces) -- the
 * whole point is that the two must not look alike.
 */
#define BOSA_RX_CDBM_NA		S32_MIN

/* Per-board optical calibration (struct bosa_optical_cal: declared in
 * luna_gpon_logic.h, where the RX/TX/temperature conversion logic that
 * consumes it now lives). The compiled defaults are Board-C's confirmed values
 * (rtl8290b.data BE fields @0x546/0x54a/0x54e). These legacy RTL ADC
 * coefficients are separate from the per-unit GN25L95 programming image. */
static struct bosa_optical_cal bosa_cal = {
	.rx_vthr = 521509, .rx_r1 = 33000, .rx_r2 = 6200,
	.rx_poly_b = 8374, .rx_poly_c = 265, .temp_off = 20,
	.tx_slope = 2022, .tx_offset = 0,
};


/* 24-bit big-endian read of three consecutive BOSA analog-page registers. */
static u32 bosa_read24(u16 reg_hi)
{
	int a = bosa_read_reg(reg_hi);
	int b = bosa_read_reg(reg_hi + 1);
	int c = bosa_read_reg(reg_hi + 2);

	if (a < 0 || b < 0 || c < 0)
		return 0;
	return ((u32)(a & 0xff) << 16) | ((u32)(b & 0xff) << 8) | (u32)(c & 0xff);
}

/* Sample the sigma-delta ADC at a mux/gain byte: select the channel on reg
 * 0x212, kick the conversion, settle, then return the 24-bit code from
 * 0x30E-0x310 (stock rtl8290b_sdadc_code_get; RSSI uses gain bytes 0x82/0xC2). */
static u32 bosa_sdadc_read(u8 mux)
{
	u32 v;

	bosa_write_reg(0x212, mux);	/* channel/gain select, latch (bit3) clear */
	msleep(10);			/* ADC settle (stock: schedule_timeout(1)) */
	bosa_set_bit(0x212, 3, 1);	/* latch/convert trigger */
	v = bosa_read24(0x30E);
	bosa_set_bit(0x212, 3, 0);	/* ALWAYS release: a stuck bit3 freezes the
					 * result bank (0x302-0x316, temperature too) */
	return v;
}

/* Ratiometric RSSI voltage (micro-volts), re-expressed from rtl8290b_rssiVoltage_get:
 * normalise the RSSI ADC against the on-die reference taps and scale to the 3.3 V
 * full-scale, averaging the two gain ranges. The reference normalisation cancels
 * the ADC gain/offset drift that a single raw tap (0x311) cannot -- this is the
 * load-bearing correctness step. The exact ref-tap roles (which of 0x305-0x307 /
 * 0x314-0x316 is the low anchor vs the span endpoint) are transcribed from a
 * partial disasm; optic_dbg dumps all taps so the first HW read at a known
 * attenuation confirms/corrects the mapping and pins rx_thr against the anchor. */
static u32 bosa_rssi_uv(void)
{
	u32 rssi   = bosa_sdadc_read(0xC2);	/* single in-range gain (0xC2) */
	u32 ref_lo = bosa_read24(0x314);	/* low reference tap (HW: 2.9M < RSSI) */
	u32 ref_hi = bosa_read24(0x305);	/* high reference tap (HW: 13.8M > RSSI) */
	u32 span   = (ref_hi > ref_lo) ? (ref_hi - ref_lo) : 1;
	u32 da     = (rssi > ref_lo) ? rssi - ref_lo : 0;

	/* ratiometric fraction in 1/10000 (0.221 -> 2210). The reference taps track
	 * the ADC gain/offset drift, so this is bench-stable where a raw single tap
	 * (0x311) is not. Averaging the two gain ranges was WRONG (they scale
	 * differently); one in-range gain is stable. */
	return (u32)div64_u64((u64)da * 10000, span);
}

/* Faithful RX power code (0.1uW): the shell samples the SD-ADC and the two
 * reference taps; bosa_rx_code_calc (luna_gpon_logic.c) decides whether
 * the three samples constitute an optical measurement at all -- the dead-bus /
 * no-light witnesses and the affine ratiometric conversion, including the
 * floor-constant-is-not-a-measurement rule, now live there, host-fuzzable. */
static u32 bosa_rx_code(void)
{
	u32 rssi   = bosa_sdadc_read(0xC2);	/* single in-range gain */
	u32 tap_lo = bosa_read24(0x314);
	u32 tap_hi = bosa_read24(0x305);

	return bosa_rx_code_calc(rssi, tap_lo, tap_hi, &bosa_cal);
}

/* Median of 3 RX code reads -- de-noises the read-to-read variation (stock
 * medians 60; 3 is enough for a diagnostic and keeps the /proc read cheap).
 * Selection rule shared with bosa_read16_median (bosa_median_u32): NA == 0
 * sorts to the bottom, so one glitched sample is discarded. */
static u32 bosa_rx_code_median(void)
{
	u32 v[3];

	v[0] = bosa_rx_code();
	v[1] = bosa_rx_code();
	v[2] = bosa_rx_code();
	return bosa_median_u32(v, 3);
}

/* Public on-demand RX optical power in centi-dBm (for /proc + ubus + ANI-G).
 * Returns BOSA_RX_CDBM_NA when the chain could not produce a reading -- the
 * caller MUST test for it and print "n/a", never a number. The median already
 * does the right thing with a single glitched sample: BOSA_RX_CODE_NA is 0, so
 * one NA is dropped as the minimum and only two or three make the median NA. */
/*
 * The one optical measurement nothing on the SoC side can fake. SFF-8472 A2
 * bytes 104/105 are RX optical power, big-endian, 0.1 uW/LSB, measured by the
 * MODULE's own monitor on its own photodiode. It passes through neither the pad
 * mux, nor the SerDes, nor the GPON LOS input, nor this driver's register model,
 * so a pad left in GPIO mode, a forced optic-LOS and an unarmed RX CDR AFE all
 * leave it untouched -- which makes it the instrument that separates "our driver
 * cannot see the light" from "there is no light". 0x0000 and 0xffff are the
 * standard "no reading" encodings and stay n/a rather than becoming -40 dBm.
 */
static s32 bosa_rx_power_sff8472_cdbm(void)
{
	int hi = bosa_i2c_read8(0x51, 104);
	int lo = bosa_i2c_read8(0x51, 105);
	u32 uw;

	if (hi < 0 || lo < 0)
		return BOSA_RX_CDBM_NA;
	uw = ((u32)(hi & 0xff) << 8) | (u32)(lo & 0xff);
	if (uw == 0 || uw == 0xffffu)
		return BOSA_RX_CDBM_NA;
	return bosa_code_to_cdbm(uw);
}

static s32 bosa_rx_power_cdbm(void)
{
	u32 code;

	/* A module that is not an RTL8290B has no RTL8290B analog RSSI chain to
	 * read -- the "registers" that chain samples are bytes of its identity
	 * EEPROM. Ask it the standard way instead. */
	if (bosa_gn_identified || bosa_not_8290b)
		return bosa_rx_power_sff8472_cdbm();
	if (!bosa_regs_live())
		return BOSA_RX_CDBM_NA;

	code = bosa_rx_code_median();
	return code == BOSA_RX_CODE_NA ? BOSA_RX_CDBM_NA : bosa_code_to_cdbm(code);
}

/* Module temperature in deci-degC. Kelvin code from the temp ADC
 * (0x302[7:0]<<1 | 0x303[7], 233..383 K = -40..+110 C) minus the per-board
 * Kelvin trim (stock rtl8290b_temperature_get; stock stores SFF-8472 1/256 C). */
static s32 bosa_temp_dc(void)
{
	int a[14], b[14], i;

	/* 14 back-to-back samples of the free-running Kelvin ADC (stock
	 * rtl8290b_temperature_get); coarse LSB is bit7 of 0x303. Requires the
	 * SD-ADC latch released (0x212 bit3=0) -- bosa_sdadc_read always clears
	 * it, else this bank stays frozen. A failed read aborts at once (don't
	 * hammer a dead bus with the remaining reads); the clamp/sort/trimmed-
	 * mean verdict is bosa_temp_dc_calc (luna_gpon_logic.c), which
	 * re-checks for the sentinel so it stays total for a host fuzzer. */
	for (i = 0; i < 14; i++) {
		a[i] = bosa_read_reg(0x302);
		b[i] = bosa_read_reg(0x303);
		if (a[i] < 0 || b[i] < 0)
			return INT_MIN;
	}
	return bosa_temp_dc_calc(a, b, bosa_cal.temp_off);
}

/* Laser bias current in micro-amps. 12-bit monitor code (0x321[7:0]<<4 |
 * 0x322[3:0]), full-scale ~100 mA at code 8192 (stock A2 word is 2 uA/LSB). */
static u32 bosa_bias_ua(void)
{
	int h = bosa_read_reg(0x321), l = bosa_read_reg(0x322);

	return bosa_bias_ua_calc(h, l);
}

/* ===== Live TX optical power (europa_drv update_ddmi_tx_power chain) =====
 * The laser monitor-photodiode (MPD) on SD-ADC channel 2, ratiometric against the
 * on-die reference taps, minus a dark reference measured once at init with the
 * laser off; per-board slope; then the same log as RX. Reaches +2.458 dBm at
 * nominal. Stock's TXSD-FSU errata (data[0x5DC]=0, inactive here) and impedance
 * compensation (a no-op at the operating point) are dropped as proven-inactive. */

static s32 bosa_vmpd_dark = INT_MIN;	/* MPD dark reference (mV); INT_MIN = not calibrated */
static u32 bosa_tx_dbg_code;		/* last MPD raw code + taps (optic_txchain diag) */
static s32 bosa_tx_dbg_hi, bosa_tx_dbg_zero;

/* One MPD voltage (mV) via SD-ADC ch2, ratiometric vs the reference taps.
 * tap_hi = 0x3B3 (live read) or 0x30B (dark cal). INT_MIN on ADC/tap error.
 * Reads code + taps while the latch (0x212 bit3) is held, then releases it. */
static s32 bosa_vmpd_mv(u16 tap_hi)
{
	u32 code;
	s32 hi, zero;

	bosa_set_bit(0x24A, 1, 0);		/* power up the ch2 (MPD) ADC */
	bosa_write_reg(0x212, 0x62);		/* ch2 select, latch (bit3) clear */
	msleep(10);				/* settle (stock: 1 jiffy @ HZ=100) */
	bosa_set_bit(0x212, 3, 1);		/* convert trigger */
	code = bosa_read24(0x30E) >> 2;
	bosa_set_bit(0x24A, 1, 1);		/* power down the ch2 ADC */
	hi   = (s32)(bosa_read24(tap_hi) >> 2);	/* ref tap (latched); positive 24-bit, plain >>2 */
	zero = (s32)(bosa_read24(0x314) >> 2);	/* zero tap */
	bosa_set_bit(0x212, 3, 0);		/* release the latch (after the taps) */
	bosa_tx_dbg_code = code;
	bosa_tx_dbg_hi = hi;
	bosa_tx_dbg_zero = zero;
	/* validity (dead-bus shape) + ratiometric mV: luna_gpon_logic.c */
	return bosa_vmpd_mv_calc(code, hi, zero);
}

/* One-time dark (laser-off) MPD calibration. MUST run at BOSA init BEFORE the
 * laser is enabled (forces TX off for ~200 ms); never at O5. */
static void bosa_vmpd_dark_calibrate(void)
{
	int save = bosa_read_reg(0x230), i, n = 0;
	s32 sum = 0, v;

	if (save < 0)
		return;
	bosa_write_reg(0x230, (save | 0xc0) & 0xff);	/* force TX off (bits 7:6) */
	bosa_set_field(0x24B, 0x0c, 3);			/* MPD mux = 3 (dark-cal path); set_field shifts by __ffs */
	for (i = 0; i < 20; i++) {
		v = bosa_vmpd_mv(0x30B);		/* dark-cal reference tap */
		if (v != INT_MIN) {
			sum += v;
			n++;
		}
	}
	bosa_set_field(0x24B, 0x0c, 2);			/* MPD mux = 2 (live) */
	bosa_write_reg(0x230, save);			/* restore TX state */
	bosa_set_bit(0x24A, 1, 1);
	if (n)
		bosa_vmpd_dark = sum / n;
	/* ★ NO SAMPLES, NO NUMBER.  With n == 0 this printed
	 * "-2147483648 mV" -- INT_MIN rendered as a measurement, which is the
	 * one thing a log may never do: an unmeasured thing must not look
	 * measured.  MEASURED on the G24W, where the module answers no register
	 * reads at all and the calibration collects nothing. */
	if (n == 0)
		pr_warn("luna-gpon: BOSA MPD dark cal NOT MEASURED (0/20 samples) -- no value is reported\n");
	else
		pr_info("luna-gpon: BOSA MPD dark cal = %d mV (%d/20 samples)\n",
			bosa_vmpd_dark, n);
}

/* Live TX optical power in centi-dBm. 10-sample mean of the range/bias-classified
 * MPD power code -> per-board 0.1uW word -> log. INT_MIN until the dark cal ran. */
static s32 bosa_tx_power_cdbm(void)
{
	u64 sum = 0;
	u32 word;
	int i, n = 0;

	if (bosa_vmpd_dark == INT_MIN)
		return INT_MIN;
	for (i = 0; i < 10; i++) {
		s32 vmpd;
		int iavg, range;

		bosa_set_field(0x24B, 0x0c, 2);		/* MPD mux = 2 (live/operational node) */
		vmpd = bosa_vmpd_mv(0x3B3);
		if (vmpd == INT_MIN)
			continue;
		iavg  = bosa_read_reg(0x23A) & 0xff;
		range = (bosa_read_reg(0x246) >> 6) & 3;
		/* voltage->code, bias class, range shift: luna_gpon_logic.c */
		sum  += bosa_tx_sample_contrib(vmpd, bosa_vmpd_dark, iavg, range);
		n++;
	}
	if (!n)
		return INT_MIN;
	/* word(0.1uW) = (avg*slope*10)>>8 + (offset*10>>5); slope=2022, offset=0 here. */
	word = bosa_tx_word_calc(sum, n, bosa_cal.tx_slope, bosa_cal.tx_offset);
	return bosa_code_to_cdbm(word);
}

/*
 * Power up the RTL8290B optical receiver so its signal-detect asserts. On a fresh
 * boot the BOSA leaves the RX amplifier powered down (W41.RXI_PWDN_L=1), so the
 * SerDes sees no signal-detect and the GPON framer can never lock. Clearing
 * RXI_PWDN_L is the only RX-path gate that differs from a working unit. Touches
 * the RX enable only, read-modify-write so the other W41 calibration bits stay.
 */
/*
 * RTL8290B RX-path operating configuration: the steady-state values a registered
 * ONU runs, bringing the RX amplifier, signal-detect comparator reference and APD
 * bias to the point at which the real signal-detect asserts. All page-2 (slave
 * 0x54) registers; the APD bias is the device's specified operating value.
 */
static const struct { u16 reg; u8 val; } bosa_rx_golden[] __initconst = {
	{ 0x204, 0x8e },	/* W4  booster/SS clock         */
	{ 0x223, 0x08 },	/* W35 RX DAC low               */
	{ 0x224, 0xba },	/* W36 RX DAC high              */
	{ 0x226, 0xd2 },	/* W38 RX mode/swing            */
	{ 0x227, 0xa7 },	/* W39 RX-LOS reference DAC     */
	{ 0x228, 0x63 },	/* W40 RX bias                  */
	{ 0x229, 0x2b },	/* W41 RX power (RXI_PWDN_L=0)  */
	{ 0x22a, 0xe4 },	/* W42 RX gain/impedance        */
	{ 0x22b, 0x00 },	/* W43 RX hysteresis            */
	{ 0x231, 0xac },	/* W49 RX/TX path config        */
	{ 0x254, 0x4d },	/* CONTROL2 (SD/LOS pin ctrl)   */
	{ 0x264, 0x43 },	/* APD bias DAC (operating value) */
	{ 0x269, 0x08 },	/* RX_TH LOS assert threshold   */
	{ 0x26a, 0x10 },	/* RX_DE_TH LOS de-assert       */
};

/*
 * The receiver bring-up as a SEQUENCE, not as a resting snapshot.
 * bosa_rx_golden[] is the steady state; stock's rtl8290b_rx_init performs an
 * ORDERED set of field writes and finishes with three whole-register writes that
 * supersede bits it set earlier at the same addresses -- and a bring-up is an FSM
 * that must RUN, so a resting state matching stock can still be reached by a
 * wrong process.
 * The order was RECOVERED BY EXECUTION, not by reading: stock's own
 * rtl8290b_rx_init was run under Unicorn MIPS-BE with its register accessors
 * intercepted (dev/re-tools/bosa_init_trace.py), giving 38 writes with no
 * unmodelled call left.
 * OFF by default: the X111W reaches O5 with the snapshot, so switching every
 * Luna board onto a new receiver bring-up unverified would risk a working board
 * to fix a broken one. bosa_rx_seq=1 selects it; the A/B is one boot.
 * ONE STOCK WRITE IS DELIBERATELY ABSENT -- txsdFaultTimer (0x256, mask 0x380).
 * That mask does not fit an 8-bit register, so it is either a field spanning two
 * registers or a mis-decode, and bosa_regmap.py flags it as the one entry of 106
 * that fails a sanity rule. Shipping a guess for it would invent a register fact.
 */
static bool bosa_rx_seq;
module_param(bosa_rx_seq, bool, 0644);
MODULE_PARM_DESC(bosa_rx_seq,
	"1 = bring the BOSA receiver up with stock's ORDERED rtl8290b_rx_init sequence instead of the bosa_rx_golden resting snapshot (default 0 = the snapshot, which is what the X111W is known to reach O5 with)");

static void __init bosa_rx_init_seq(void)
{
	/* analog front end: LDO, output polarity/swing, offset cancel */
	bosa_set_field(BOSA_REG_W41, 0x30, 2);
	bosa_set_bit(0x22a, 3, 0);
	bosa_set_field(0x22a, 0x07, 4);
	bosa_set_field(BOSA_REG_W41, 0xc0, 0);
	bosa_set_bit(BOSA_REG_W41, 3, 0);
	bosa_set_bit(0x27b, 2, 0);
	bosa_set_bit(0x22b, 7, 0);

	/* the LOS detector, held in reset while it is configured */
	bosa_set_bit(0x224, 1, 0);		/* rxlosResetb   */
	bosa_set_bit(0x224, 0, 0);		/* rxlosClkMode  */
	bosa_set_field(0x226, 0xc0, 3);
	bosa_set_field(0x226, 0x07, 0);		/* rxlosHystSel  */
	bosa_set_bit(0x226, 3, 0);		/* rxlosRangSel  */
	bosa_set_bit(0x226, 5, 0);		/* rxlosInputSel */
	bosa_set_field(0x227, 0x7f, 0);		/* rxlosRefDac -- stock ZEROES the
						 * comparator threshold; the
						 * snapshot carries 0x27 */
	bosa_set_field(0x228, 0xc0, 0);		/* rxlosChopperFreq */
	bosa_set_field(0x228, 0x30, 0);		/* rxlosSampleSel   */
	bosa_set_bit(BOSA_REG_W41, 0, 0);		/* rxlosChopperEn   */
	bosa_set_bit(BOSA_REG_W41, 2, 0);		/* rxlosLaMagComp   */
	bosa_set_bit(BOSA_REG_W41, 1, 0);		/* rxlosBufAutozero */
	bosa_set_bit(0x231, 1, 0);		/* rxlosTestMode    */
	bosa_set_bit(0x231, 0, 0);		/* rxlosAssertSel   */
	bosa_set_field(0x257, 0xc0, 0);		/* rxlosDebounceSel */
	bosa_set_bit(0x27b, 0, 0);		/* rxlosDebounceOpt */
	bosa_set_bit(0x27b, 1, 0);
	bosa_set_bit(0x27b, 3, 1);		/* rxlosSquelch     */
	bosa_set_bit(0x25e, 0, 0);		/* rxlosPolarity    */

	/* 0x256 txsdFaultTimer is skipped on purpose -- see the note above */

	/* pin control: SD out on, LOS pin off */
	bosa_set_bit(BOSA_REG_CONTROL2, 7, 1);		/* txSdPinEn  */
	bosa_set_bit(0x3c1, 7, 1);
	bosa_set_bit(0x3c1, 5, 1);
	bosa_set_field(BOSA_REG_CONTROL2, 0x03, 0);		/* txdisCtrl  */
	bosa_set_bit(BOSA_REG_CONTROL2, 6, 0);		/* rxlosPinEn */
	bosa_set_bit(0x281, 0, 1);
	bosa_set_bit(0x257, 0, 0);
	bosa_set_bit(0x27b, 4, 1);

	/* ★ THE TAIL IS WHOLE REGISTERS AND IT SUPERSEDES BITS SET ABOVE.
	 * 0x224 carries rxlosClkMode and rxlosResetb, so writing 0xbc here is
	 * what actually decides them -- a port that replicated only the field
	 * list would leave this register holding something else entirely. */
	bosa_write_reg(0x223, 0x08);
	bosa_write_reg(0x224, 0xbc);
	bosa_write_reg(0x264, 0x43);
}

static void __init bosa_rx_enable(void)
{
	int i, sdet;

	/* Apply the RX operating point to the BOSA: stock's ordered sequence, or
	 * the resting snapshot.  Exactly one of them, so the A/B compares two
	 * bring-ups and not a bring-up layered on another. */
	if (bosa_rx_seq) {
		bosa_rx_init_seq();
	} else {
		for (i = 0; i < ARRAY_SIZE(bosa_rx_golden); i++)
			bosa_write_reg(bosa_rx_golden[i].reg,
				       bosa_rx_golden[i].val);
	}
	mdelay(50);					/* RX amp + SD comparator settle */

	/* Re-read so /proc shows the post-config state. */
	bosa_w41     = bosa_read_reg(BOSA_REG_W41);
	bosa_ctrl2   = bosa_read_reg(BOSA_REG_CONTROL2);
	bosa_status2 = bosa_read_reg(BOSA_REG_STATUS2);
	sdet = !!(sw_rd(SDS_FIB_STATUS) & SDS_FIB_SDS_SDET);
	/* ★ "APPLIED" MUST MEAN APPLIED.  On a module with no RTL8290B register
	 * interface every write above is REFUSED, and this line still said
	 * "config applied" beside six read-backs of 0x20 -- which are ASCII
	 * spaces from the module's identity strings, not register values.  A log
	 * that reports success for work that was refused is worse than silence:
	 * it is what a reader trusts. */
	if (bosa_not_8290b || bosa_id_num != 0x8290) {
		pr_warn("luna-gpon: BOSA RX config NOT APPLIED -- writes refused, no positive RTL8290B id (chip id=0x%04x); the values below are READ-BACKS of a module that is not one\n",
			bosa_id_num);
	}
	pr_info("luna-gpon: BOSA RX config %s: w4=0x%02x w41=0x%02x ctrl2=0x%02x status2=0x%02x apd=0x%02x w39=0x%02x sds_sdet=%d\n",
		(bosa_not_8290b || bosa_id_num != 0x8290) ? "read back (NOT applied)"
							  : "applied",
		bosa_read_reg(BOSA_REG_W4) & 0xff, bosa_w41 & 0xff,
		bosa_ctrl2 & 0xff, bosa_status2 & 0xff,
		bosa_read_reg(0x264) & 0xff, bosa_read_reg(0x227) & 0xff, sdet);
}

/*
 * Upstream-laser (TX) operating point -- the values a registered unit runs on
 * this BOSA. The RX table above never touches the laser driver, so without this
 * the ONU receives downstream fine and cannot transmit its upstream PLOAM bursts,
 * leaving it stuck at O3. Order follows the TX-enable flow: bias power, DAC
 * codes, DAC/APC power, fault detect, TXSD, enable mode.
 */
static const struct { u16 reg; u8 val; } bosa_tx_golden[] __initconst = {
	{ 0x22e, 0xb0 },	/* W46 TX bias power + APC clocks  */
	{ 0x236, 0x19 },	/* W54 laser BIAS DAC high          */
	{ 0x237, 0x67 },	/* W55 laser MOD DAC high           */
	{ 0x238, 0x22 },	/* W56 BIAS/MOD DAC low bits        */
	{ 0x239, 0x2d },	/* W57 APCDIG bias DAC power        */
	{ 0x235, 0xcf },	/* W53 TX/APC fault detection       */
	{ 0x23c, 0x03 },	/* W60 TIA power config             */
	{ 0x284, 0xf2 },	/* W88 DSR TX APC set-point         */
	{ 0x27c, 0xe9 },	/* W80 TX backup/state             */
	{ 0x230, 0x0e },	/* W48 TX_ENMODE (enable, last)    */
};

/*
 * BOSA full register image -- the values a registered unit runs. 988 writes,
 * strictly ascending 0x000..0x3ff, covering all four RTL8290B pages; the three
 * holes are live state the image leaves to the MCU and each is marked in place.
 * The WHOLE image, because an earlier revision wrote only the page-2 RX/TX
 * registers and left the BOSA control page (clocks / power / APC-digital
 * enables) at power-on defaults, so the APC digital block never clocked and the
 * laser stayed dark. The 0xff entries are master enable masks.
 *
 * ORDER AND VALUES ARE THE CONTRACT: bosa_tx_enable() replays this verbatim over
 * I2C. The grouping and comments below are for the reader only -- the emitted
 * { reg, val } sequence is unchanged, proven element-by-element against the
 * pre-rewrite table on the host.
 */
static const struct { u16 reg; u8 val; } bosa_init_golden[] __initconst = {
	/* ---- page 0 (I2C slave 0x50): SFF-8472 A0 identification-page image ----
	 * The module identity the MCU serves at the standard SFP EEPROM address.
	 * Field offsets below follow the public SFF-8472/SFF-8024 A0h layout; the
	 * match is measured on this data (strings at bytes 20/40/56/84, wavelength
	 * 1310 nm, 20 km reach), not assumed.  Strings are written as character
	 * literals -- same bytes, now readable. */
	/* bytes 0..19, identity header: identifier=0x02, ext-id=0x04,
	 * connector=0x0b, 8x 0xff (see the note above), encoding=0x03 (NRZ),
	 * rate=0x0c (x100 Mbit/s), reach 0x14=20 km / 0xc8=200x100 m */
	{0x000,0x02}, {0x001,0x04}, {0x002,0x0b}, {0x003,0xff}, {0x004,0xff}, {0x005,0xff},
	{0x006,0xff}, {0x007,0xff}, {0x008,0xff}, {0x009,0xff}, {0x00a,0xff}, {0x00b,0x03},
	{0x00c,0x0c}, {0x00d,0x00}, {0x00e,0x14}, {0x00f,0xc8}, {0x010,0x00}, {0x011,0x00},
	{0x012,0x00}, {0x013,0x00},
	/* bytes 20..35, vendor name: "REALTEK         " */
	{0x014,'R'}, {0x015,'E'}, {0x016,'A'}, {0x017,'L'}, {0x018,'T'}, {0x019,'E'},
	{0x01a,'K'}, {0x01b,' '}, {0x01c,' '}, {0x01d,' '}, {0x01e,' '}, {0x01f,' '},
	{0x020,' '}, {0x021,' '}, {0x022,' '}, {0x023,' '},
	/* bytes 36..39: reserved + vendor OUI, zero */
	{0x024,0x00}, {0x025,0x00}, {0x026,0x00}, {0x027,0x00},
	/* bytes 40..55, vendor part number: "RTL8290         " */
	{0x028,'R'}, {0x029,'T'}, {0x02a,'L'}, {0x02b,'8'}, {0x02c,'2'}, {0x02d,'9'},
	{0x02e,'0'}, {0x02f,' '}, {0x030,' '}, {0x031,' '}, {0x032,' '}, {0x033,' '},
	{0x034,' '}, {0x035,' '}, {0x036,' '}, {0x037,' '},
	/* bytes 56..59, vendor rev: "0001" */
	{0x038,'0'}, {0x039,'0'}, {0x03a,'0'}, {0x03b,'1'},
	/* bytes 60..67: wavelength, CC_BASE position (0xff, checksum not
	 * maintained), options */
	{0x03c,0x05},	/* wavelength 0x051e = 1310 nm (hi byte; lo is next) */
	{0x03d,0x1e}, {0x03e,0x00}, {0x03f,0xff}, {0x040,0x00}, {0x041,0x20}, {0x042,0x00},
	{0x043,0x00},
	/* bytes 68..83, vendor serial-number field: holds mangled placeholder
	 * text, kept byte-exact: vendorpxrtn'mber */
	{0x044,'v'}, {0x045,'e'}, {0x046,'n'}, {0x047,'d'}, {0x048,'o'}, {0x049,'r'},
	{0x04a,'p'}, {0x04b,'x'}, {0x04c,'r'}, {0x04d,'t'}, {0x04e,'n'}, {0x04f,'"'},
	{0x050,'m'}, {0x051,'b'}, {0x052,'e'}, {0x053,'r'},
	/* bytes 84..91, date code: "20140123" (2014-01-23) */
	{0x054,'2'}, {0x055,'0'}, {0x056,'1'}, {0x057,'4'}, {0x058,'0'}, {0x059,'1'},
	{0x05a,'2'}, {0x05b,'3'},
	/* bytes 92..94: DDM type=0x68, enhanced options=0x80, SFF-8472
	 * compliance=0x02 */
	{0x05c,0x68}, {0x05d,0x80}, {0x05e,0x02},
	/* bytes 95..255: 0xff filler (unprogrammed EEPROM area, incl. the byte-95
	 * checksum position); the lone non-0xff bytes are broken out, as captured */
	{0x05f,0xff}, {0x060,0xff}, {0x061,0xff}, {0x062,0xff}, {0x063,0xff}, {0x064,0xff},
	{0x065,0xff}, {0x066,0xff}, {0x067,0xff}, {0x068,0xff}, {0x069,0xff}, {0x06a,0xff},
	{0x06b,0xff}, {0x06c,0xff}, {0x06d,0xff}, {0x06e,0xff}, {0x06f,0xff}, {0x070,0xff},
	{0x071,0xff}, {0x072,0xff}, {0x073,0xff}, {0x074,0xff}, {0x075,0xff}, {0x076,0xff},
	{0x077,0xff}, {0x078,0xff}, {0x079,0xff}, {0x07a,0xff}, {0x07b,0xff}, {0x07c,0xff},
	{0x07d,0xff}, {0x07e,0xff}, {0x07f,0xff}, {0x080,0xff},
	{0x081,0x10},	/* lone non-0xff byte, as captured */
	{0x082,0xff}, {0x083,0xff}, {0x084,0xff}, {0x085,0xff}, {0x086,0xff},
	{0x087,0xfd},	/* lone non-0xff byte, as captured */
	{0x088,0xff}, {0x089,0xff},
	{0x08a,0x54},	/* lone non-0xff byte, as captured */
	{0x08b,0xff}, {0x08c,0xff}, {0x08d,0xff}, {0x08e,0xff}, {0x08f,0xff}, {0x090,0xff},
	{0x091,0xff}, {0x092,0xff}, {0x093,0xff}, {0x094,0xff}, {0x095,0xff}, {0x096,0xff},
	{0x097,0xff}, {0x098,0xff}, {0x099,0xff}, {0x09a,0xff}, {0x09b,0xff}, {0x09c,0xff},
	{0x09d,0xff}, {0x09e,0xff}, {0x09f,0xff}, {0x0a0,0xff}, {0x0a1,0xff}, {0x0a2,0xff},
	{0x0a3,0xff}, {0x0a4,0xff}, {0x0a5,0xff}, {0x0a6,0xff}, {0x0a7,0xff}, {0x0a8,0xff},
	{0x0a9,0xff}, {0x0aa,0xff}, {0x0ab,0xff}, {0x0ac,0xff}, {0x0ad,0xff}, {0x0ae,0xff},
	{0x0af,0xfd},	/* lone non-0xff byte, as captured */
	{0x0b0,0xff}, {0x0b1,0xff},
	{0x0b2,0x78},	/* lone non-0xff byte, as captured */
	{0x0b3,0xff}, {0x0b4,0xff},
	{0x0b5,0x4f},	/* lone non-0xff byte, as captured */
	{0x0b6,0xff}, {0x0b7,0xff}, {0x0b8,0xff}, {0x0b9,0xff}, {0x0ba,0xff}, {0x0bb,0xff},
	{0x0bc,0xff}, {0x0bd,0xff}, {0x0be,0xff}, {0x0bf,0xff}, {0x0c0,0xff}, {0x0c1,0xff},
	{0x0c2,0xff}, {0x0c3,0xff}, {0x0c4,0xff}, {0x0c5,0xff}, {0x0c6,0xff}, {0x0c7,0xff},
	{0x0c8,0xff}, {0x0c9,0xff}, {0x0ca,0xff}, {0x0cb,0xff}, {0x0cc,0xff}, {0x0cd,0xff},
	{0x0ce,0xff}, {0x0cf,0xff}, {0x0d0,0xff}, {0x0d1,0xff}, {0x0d2,0xff}, {0x0d3,0xff},
	{0x0d4,0x0c},	/* lone non-0xff byte, as captured */
	{0x0d5,0xff}, {0x0d6,0xff}, {0x0d7,0xff}, {0x0d8,0xff}, {0x0d9,0xff}, {0x0da,0xff},
	{0x0db,0xff}, {0x0dc,0xff}, {0x0dd,0xff}, {0x0de,0xff}, {0x0df,0xff}, {0x0e0,0xff},
	{0x0e1,0xff}, {0x0e2,0xff}, {0x0e3,0xff}, {0x0e4,0xff}, {0x0e5,0xff}, {0x0e6,0xff},
	{0x0e7,0xff}, {0x0e8,0xff}, {0x0e9,0xff}, {0x0ea,0xff}, {0x0eb,0xff}, {0x0ec,0xff},
	{0x0ed,0xff}, {0x0ee,0xff}, {0x0ef,0xff}, {0x0f0,0xff}, {0x0f1,0xff}, {0x0f2,0xff},
	{0x0f3,0xff},
	{0x0f4,0x6d},	/* lone non-0xff byte, as captured */
	{0x0f5,0xff},
	{0x0f6,0xfc},	/* lone non-0xff byte, as captured */
	{0x0f7,0xff}, {0x0f8,0xff}, {0x0f9,0xff},
	{0x0fa,0x70},	/* lone non-0xff byte, as captured */
	{0x0fb,0xff}, {0x0fc,0xff}, {0x0fd,0xff}, {0x0fe,0xff}, {0x0ff,0xff},
	/* ---- page 1 (I2C slave 0x51): SFF-8472 A2 diagnostics-page image ----
	 * The DDM page; the runtime reads live optical power from it (bosa_read16:
	 * TX @0x166/0x167, RX @0x168/0x169, 0.1 uW/LSB). */
	/* A2 bytes 0..39, alarm/warning thresholds (temp, Vcc, bias, TX pwr, RX
	 * pwr; each high/low alarm then high/low warning).  Decoded anchors:
	 * Vcc alarm 0x8e94=3.65V / 0x6d60=2.80V, warn 3.60/3.00V (100uV/LSB);
	 * bias alarm 0x7530=60mA / 0x05dc=3mA, warn 50/4mA (2uA/LSB); the temp
	 * rows sit at +127.996C and the TX/RX-power rows are only partly
	 * programmed */
	{0x100,0x7f}, {0x101,0xff}, {0x102,0xff}, {0x103,0xff}, {0x104,0x7f}, {0x105,0xff},
	{0x106,0xff}, {0x107,0xff}, {0x108,0x8e}, {0x109,0x94}, {0x10a,0x6d}, {0x10b,0x60},
	{0x10c,0x8c}, {0x10d,0xa0}, {0x10e,0x75}, {0x10f,0x30}, {0x110,0x75}, {0x111,0x30},
	{0x112,0x05}, {0x113,0xdc}, {0x114,0x61}, {0x115,0xa8}, {0x116,0x07}, {0x117,0xd0},
	{0x118,0x00}, {0x119,0x00}, {0x11a,0x0f}, {0x11b,0x8d}, {0x11c,0x00}, {0x11d,0x0a},
	{0x11e,0x0c}, {0x11f,0x5a}, {0x120,0x00}, {0x121,0x0c}, {0x122,0x00}, {0x123,0x00},
	{0x124,0x00}, {0x125,0x00}, {0x126,0x00}, {0x127,0x00},
	/* A2 bytes 40..55: zero */
	{0x128,0x00}, {0x129,0x00}, {0x12a,0x00}, {0x12b,0x00}, {0x12c,0x00}, {0x12d,0x00},
	{0x12e,0x00}, {0x12f,0x00}, {0x130,0x00}, {0x131,0x00}, {0x132,0x00}, {0x133,0x00},
	{0x134,0x00}, {0x135,0x00}, {0x136,0x00}, {0x137,0x00},
	/* A2 bytes 56..91, calibration-constant region: mostly zero; the
	 * 0x01,0x00 pairs are unity slopes, plus one 0x3f80 */
	{0x138,0x00}, {0x139,0x00}, {0x13a,0x00}, {0x13b,0x00}, {0x13c,0x00}, {0x13d,0x00},
	{0x13e,0x3f}, {0x13f,0x80}, {0x140,0x00}, {0x141,0x00}, {0x142,0x00}, {0x143,0x00},
	{0x144,0x00}, {0x145,0x00}, {0x146,0x01}, {0x147,0x00}, {0x148,0x00}, {0x149,0x00},
	{0x14a,0x01}, {0x14b,0x00}, {0x14c,0x00}, {0x14d,0x00}, {0x14e,0x01}, {0x14f,0x00},
	{0x150,0x00}, {0x151,0x00}, {0x152,0x01}, {0x153,0x00}, {0x154,0x00}, {0x155,0x00},
	{0x156,0x00}, {0x157,0x00}, {0x158,0x00}, {0x159,0xff}, {0x15a,0xff}, {0x15b,0xff},
	/* A2 bytes 92..95: 0xff (incl. the byte-95 CC_DMI checksum position, not
	 * maintained) */
	{0x15c,0xff}, {0x15d,0xff}, {0x15e,0xff}, {0x15f,0xff},
	/* A2 bytes 96..119, live-diagnostics area: a captured snapshot (the MCU
	 * overwrites these with real readings) */
	{0x160,0x2c}, {0x161,0x38}, {0x162,0x84}, {0x163,0x98}, {0x164,0x18}, {0x165,0x14},
	{0x166,0x3e},	/* TX optical power word hi (bosa_read16 reads it live) */
	{0x167,0x52},
	{0x168,0x00},	/* RX optical power word hi (bosa_read16) */
	{0x169,0xe5}, {0x16a,0xff}, {0x16b,0x00}, {0x16c,0x00}, {0x16d,0x00}, {0x16e,0xff},
	{0x16f,0x00}, {0x170,0x03}, {0x171,0xc0}, {0x172,0x00}, {0x173,0x00}, {0x174,0x03},
	{0x175,0xc0}, {0x176,0x00}, {0x177,0x00},
	/* A2 bytes 120..127: zero */
	{0x178,0x00}, {0x179,0x00}, {0x17a,0x00}, {0x17b,0x00}, {0x17c,0x00}, {0x17d,0x00},
	{0x17e,0x00}, {0x17f,0x00},
	/* A2 bytes 128..255, page upper half (user/vendor area): opaque content,
	 * captured as-is; nothing in this driver reads it */
	{0x180,0x73}, {0x181,0x11}, {0x182,0x7d}, {0x183,0x39}, {0x184,0x6d}, {0x185,0xdb},
	{0x186,0xf9}, {0x187,0x54}, {0x188,0xc3}, {0x189,0xc0}, {0x18a,0x59}, {0x18b,0x1a},
	{0x18c,0x4b}, {0x18d,0xe3}, {0x18e,0xfb}, {0x18f,0x94}, {0x190,0x0c}, {0x191,0xa4},
	{0x192,0x16}, {0x193,0xca}, {0x194,0xf0}, {0x195,0x51}, {0x196,0xc1}, {0x197,0xe4},
	{0x198,0x08}, {0x199,0x09}, {0x19a,0x6d}, {0x19b,0x43}, {0x19c,0xe0}, {0x19d,0x63},
	{0x19e,0x65}, {0x19f,0x1d}, {0x1a0,0x89}, {0x1a1,0x53}, {0x1a2,0x54}, {0x1a3,0x23},
	{0x1a4,0x7b}, {0x1a5,0xe5}, {0x1a6,0xda}, {0x1a7,0x2e}, {0x1a8,0x7c}, {0x1a9,0xf5},
	{0x1aa,0x7c}, {0x1ab,0xe7}, {0x1ac,0xce}, {0x1ad,0x2b}, {0x1ae,0xd1}, {0x1af,0x76},
	{0x1b0,0xf8}, {0x1b1,0xdc}, {0x1b2,0x72}, {0x1b3,0x92}, {0x1b4,0x94}, {0x1b5,0x34},
	{0x1b6,0x69}, {0x1b7,0x48}, {0x1b8,0x85}, {0x1b9,0xff}, {0x1ba,0x30}, {0x1bb,0x0c},
	{0x1bc,0x23}, {0x1bd,0xdd}, {0x1be,0x3c}, {0x1bf,0xdc}, {0x1c0,0x53}, {0x1c1,0xd3},
	{0x1c2,0x5d}, {0x1c3,0x5c}, {0x1c4,0xc4}, {0x1c5,0xef}, {0x1c6,0xdb}, {0x1c7,0xe5},
	{0x1c8,0xc8}, {0x1c9,0xff}, {0x1ca,0xdb}, {0x1cb,0x52}, {0x1cc,0x22}, {0x1cd,0x27},
	{0x1ce,0xfd}, {0x1cf,0x37}, {0x1d0,0x3b}, {0x1d1,0x33}, {0x1d2,0xc3}, {0x1d3,0x91},
	{0x1d4,0xa2}, {0x1d5,0x01}, {0x1d6,0xbd}, {0x1d7,0x7c}, {0x1d8,0x4e}, {0x1d9,0xe6},
	{0x1da,0x0d}, {0x1db,0x2d}, {0x1dc,0x2e}, {0x1dd,0x94}, {0x1de,0xa0}, {0x1df,0xeb},
	{0x1e0,0xd9}, {0x1e1,0x31}, {0x1e2,0x39}, {0x1e3,0x91}, {0x1e4,0x68}, {0x1e5,0xf6},
	{0x1e6,0x7b}, {0x1e7,0x4b}, {0x1e8,0x67}, {0x1e9,0xf4}, {0x1ea,0xc7}, {0x1eb,0x36},
	{0x1ec,0xfd}, {0x1ed,0x4d}, {0x1ee,0x86}, {0x1ef,0x76}, {0x1f0,0x36}, {0x1f1,0x2c},
	{0x1f2,0xf3}, {0x1f3,0x2e}, {0x1f4,0x3c}, {0x1f5,0xbd}, {0x1f6,0xb5}, {0x1f7,0x01},
	{0x1f8,0x37}, {0x1f9,0x11}, {0x1fa,0x04}, {0x1fb,0x7b}, {0x1fc,0x9d}, {0x1fd,0x01},
	{0x1fe,0x99}, {0x1ff,0x3a},
	/* ---- page 2 (I2C slave 0x54): analog/APC "W" registers ----
	 * Wn = page offset n (decimal), the numbering the runtime code in this
	 * file already uses (W41=0x229 ... W67=0x243).  The TX-critical subset
	 * here is byte-identical to what bosa_tx_golden re-writes afterwards in
	 * TX-enable order; the APC setpoints (W58/W59/W61) are later replaced by
	 * bosa_apc_calibrate's per-board values. */
	{0x200,0x02}, {0x201,0x89}, {0x202,0xa1}, {0x203,0xfe},
	{0x204,0x8e},	/* BOSA_REG_W4: EN_L booster */
	{0x205,0xb2}, {0x206,0x9b}, {0x207,0x90}, {0x208,0x00}, {0x209,0x49}, {0x20a,0x9f},
	{0x20b,0xff}, {0x20c,0x23}, {0x20d,0x04}, {0x20e,0x78}, {0x20f,0x7f}, {0x210,0xff},
	{0x211,0x00},
	{0x212,0x82},	/* DDM mux: channel/gain select (bosa_read_dd) */
	{0x213,0x05}, {0x214,0x00}, {0x215,0x00}, {0x216,0x01}, {0x217,0xf6}, {0x218,0xce},
	{0x219,0x90}, {0x21a,0xc0}, {0x21b,0x00}, {0x21c,0x00}, {0x21d,0x38}, {0x21e,0x24},
	{0x21f,0x40}, {0x220,0x40}, {0x221,0x00}, {0x222,0x01},
	{0x223,0x08},	/* RX config (RX bring-up also writes 0x08) */
	{0x224,0xba},	/* RX config (RX bring-up writes 0xbc; image holds 0xba) */
	{0x225,0x1e}, {0x226,0xd2},
	{0x227,0xa7},	/* "w39" of the RX status log */
	{0x228,0x63},
	{0x229,0x2b},	/* BOSA_REG_W41: RXI_PWDN_L */
	{0x22a,0xe4}, {0x22b,0x00}, {0x22c,0xe0}, {0x22d,0x01},
	{0x22e,0xb0},	/* W46 TX bias power + APC clocks (= bosa_tx_golden) */
	{0x22f,0x44},
	{0x230,0x0e},	/* W48 TX_ENMODE (= bosa_tx_golden) */
	{0x231,0xac}, {0x232,0x01}, {0x233,0x08}, {0x234,0x80},
	{0x235,0xcf},	/* W53 TX/APC fault detection (= bosa_tx_golden) */
	{0x236,0x19},	/* W54 laser BIAS DAC high (= bosa_tx_golden) */
	{0x237,0x67},	/* W55 laser MOD DAC high (= bosa_tx_golden) */
	{0x238,0x22},	/* W56 BIAS/MOD DAC low bits (= bosa_tx_golden) */
	{0x239,0x2d},	/* W57 APCDIG bias DAC power (= bosa_tx_golden) */
	{0x23a,0x62},	/* W58 DCL P0 -- bosa_apc_calibrate overwrites (0x26) */
	{0x23b,0xcf},	/* W59 DCL P1 -- bosa_apc_calibrate overwrites (0x50) */
	{0x23c,0x03},	/* W60 TIA power config (= bosa_tx_golden) */
	{0x23d,0xa2},	/* W61 DCL Pavg -- bosa_apc_calibrate overwrites (0x50) */
	{0x23e,0xfc}, {0x23f,0xfd}, {0x240,0x02}, {0x241,0x57}, {0x242,0xd0},
	{0x243,0x80},	/* W67: bit7 = APC hardware-servo enable */
	{0x244,0x00}, {0x245,0x00},
	{0x246,0x3f},	/* bias-range bits [7:6] (read by the DDM path) */
	{0x247,0xcc}, {0x248,0x4d}, {0x249,0x2a}, {0x24a,0x22}, {0x24b,0x89}, {0x24c,0x85},
	{0x24d,0xb0}, {0x24e,0x80}, {0x24f,0x3f}, {0x250,0x00}, {0x251,0x00}, {0x252,0x00},
	{0x253,0x00},
	{0x254,0x4d},	/* BOSA_REG_CONTROL2: TX_POW_CTL/ENLD_L/LOS_PIN_TRI */
	{0x255,0x30},	/* CONTROL3: bit1 = fault-release strobe (re-arm path) */
	{0x256,0x00}, {0x257,0xf4}, {0x258,0x00}, {0x259,0xfe}, {0x25a,0xff}, {0x25b,0x01},
	{0x25c,0x00}, {0x25d,0xff}, {0x25e,0x00}, {0x25f,0x02}, {0x260,0x00}, {0x261,0x03},
	{0x262,0xff}, {0x263,0x07},
	{0x264,0x43},	/* APD control ("apd" of the RX log; RX bring-up also writes 0x43) */
	{0x265,0x00}, {0x266,0xa0}, {0x267,0xc0}, {0x268,0x00}, {0x269,0x08}, {0x26a,0x10},
	{0x26b,0xe0}, {0x26c,0xe0}, {0x26d,0xe0}, {0x26e,0xff}, {0x26f,0xf4}, {0x270,0x84},
	{0x271,0x82}, {0x272,0x50}, {0x273,0x00}, {0x274,0xff}, {0x275,0x00}, {0x276,0x10},
	{0x277,0x00}, {0x278,0x00}, {0x279,0xff}, {0x27a,0x00}, {0x27b,0x08},
	{0x27c,0xe9},	/* W80 TX backup/state (= bosa_tx_golden) */
	{0x27d,0x00}, {0x27e,0x00}, {0x27f,0x00}, {0x280,0x00}, {0x281,0x01}, {0x282,0x00},
	{0x283,0x88},
	{0x284,0xf2},	/* W88 DSR TX APC set-point (= bosa_tx_golden) */
	{0x285,0x00}, {0x286,0x00}, {0x287,0x08}, {0x288,0x00}, {0x289,0x00}, {0x28a,0x00},
	{0x28b,0x00}, {0x28c,0x00}, {0x28d,0x00}, {0x28e,0x00}, {0x28f,0x00}, {0x290,0x00},
	{0x291,0x00}, {0x292,0x08}, {0x293,0x00}, {0x294,0x00}, {0x295,0x00}, {0x296,0x00},
	{0x297,0x00}, {0x298,0x00}, {0x299,0x00}, {0x29a,0x00}, {0x29b,0x00}, {0x29c,0x00},
	{0x29d,0x00}, {0x29e,0x00}, {0x29f,0x00}, {0x2a0,0x00}, {0x2a1,0x00}, {0x2a2,0x00},
	{0x2a3,0x00}, {0x2a4,0x00}, {0x2a5,0x00}, {0x2a6,0x00}, {0x2a7,0x08}, {0x2a8,0x00},
	{0x2a9,0x00}, {0x2aa,0x00}, {0x2ab,0x00}, {0x2ac,0x00}, {0x2ad,0x00}, {0x2ae,0x00},
	{0x2af,0x08}, {0x2b0,0x00}, {0x2b1,0x00}, {0x2b2,0x00}, {0x2b3,0x00}, {0x2b4,0x00},
	{0x2b5,0x00}, {0x2b6,0x00}, {0x2b7,0xca}, {0x2b8,0x00}, {0x2b9,0x00}, {0x2ba,0x00},
	{0x2bb,0x00}, {0x2bc,0x00}, {0x2bd,0x00}, {0x2be,0x00}, {0x2bf,0x00}, {0x2c0,0x00},
	{0x2c1,0xfc}, {0x2c2,0x00}, {0x2c3,0x00}, {0x2c4,0x00}, {0x2c5,0x02}, {0x2c6,0x00},
	{0x2c7,0xe3}, {0x2c8,0x00}, {0x2c9,0x00}, {0x2ca,0x00}, {0x2cb,0x00}, {0x2cc,0x00},
	{0x2cd,0x00}, {0x2ce,0x00}, {0x2cf,0x00}, {0x2d0,0x00}, {0x2d1,0x00}, {0x2d2,0x00},
	{0x2d3,0x00}, {0x2d4,0x00}, {0x2d5,0x00}, {0x2d6,0x00}, {0x2d7,0x00}, {0x2d8,0x00},
	{0x2d9,0x00}, {0x2da,0x00}, {0x2db,0x00}, {0x2dc,0x00}, {0x2dd,0x00}, {0x2de,0x00},
	{0x2df,0x00}, {0x2e0,0x00}, {0x2e1,0x00}, {0x2e2,0x00}, {0x2e3,0x00}, {0x2e4,0x00},
	{0x2e5,0x00}, {0x2e6,0x00}, {0x2e7,0x00}, {0x2e8,0x00}, {0x2e9,0x00}, {0x2ea,0x00},
	{0x2eb,0x08}, {0x2ec,0x00}, {0x2ed,0x00}, {0x2ee,0x00}, {0x2ef,0x00}, {0x2f0,0x00},
	{0x2f1,0x00}, {0x2f2,0x00}, {0x2f3,0x08}, {0x2f4,0x00}, {0x2f5,0x00}, {0x2f6,0x00},
	{0x2f7,0x00}, {0x2f8,0x00}, {0x2f9,0x00}, {0x2fa,0x00}, {0x2fb,0x00}, {0x2fc,0x00},
	{0x2fd,0x00}, {0x2fe,0x08}, {0x2ff,0x00},
	/* ---- page 3 (I2C slave 0x55): MCU control/status page ---- */
	{0x300,0xd6}, {0x301,0xca},
	{0x302,0xa9},	/* ADC readout hi (polled by bosa_read_dd) */
	{0x303,0x08},	/* ADC readout lo */
	{0x304,0xc4}, {0x305,0xe4}, {0x306,0x78}, {0x307,0x70}, {0x308,0xe5}, {0x309,0xcd},
	{0x30a,0xf8},
	/* 0x30b..0x325 not written: live MCU/APC-servo state sits here (R29
	 * 0x31d OFFK latch, R30 0x31e, ADC capture 0x320..0x322) -- left to the
	 * MCU */
	{0x326,0x00}, {0x327,0x00}, {0x328,0x00}, {0x329,0x08}, {0x32a,0x00}, {0x32b,0x00},
	{0x32c,0x00}, {0x32d,0x00}, {0x32e,0x00}, {0x32f,0x00}, {0x330,0x00}, {0x331,0x00},
	{0x332,0x00}, {0x333,0x00}, {0x334,0x00}, {0x335,0x00}, {0x336,0x00}, {0x337,0x00},
	{0x338,0x00}, {0x339,0x00}, {0x33a,0x00}, {0x33b,0x02}, {0x33c,0x00}, {0x33d,0x09},
	{0x33e,0x00}, {0x33f,0x00}, {0x340,0x00}, {0x341,0x00}, {0x342,0x00}, {0x343,0x00},
	{0x344,0x00}, {0x345,0x00}, {0x346,0x00}, {0x347,0x00}, {0x348,0x00}, {0x349,0x00},
	{0x34a,0x00}, {0x34b,0x00}, {0x34c,0x00}, {0x34d,0x00}, {0x34e,0x00}, {0x34f,0x00},
	{0x350,0x00}, {0x351,0x00}, {0x352,0x00}, {0x353,0x00}, {0x354,0x00}, {0x355,0x00},
	{0x356,0x00}, {0x357,0x00}, {0x358,0x00}, {0x359,0x00}, {0x35a,0x00}, {0x35b,0x00},
	{0x35c,0x00}, {0x35d,0x00}, {0x35e,0x00}, {0x35f,0x00}, {0x360,0x00}, {0x361,0x00},
	{0x362,0x00}, {0x363,0x00}, {0x364,0x00}, {0x365,0x00}, {0x366,0x00}, {0x367,0x00},
	{0x368,0x00}, {0x369,0x00}, {0x36a,0x00}, {0x36b,0x00}, {0x36c,0x00}, {0x36d,0x00},
	{0x36e,0x00}, {0x36f,0x00}, {0x370,0x00}, {0x371,0x00}, {0x372,0x00}, {0x373,0x00},
	{0x374,0x00}, {0x375,0x00}, {0x376,0x00}, {0x377,0x00}, {0x378,0x00}, {0x379,0x00},
	{0x37a,0x00}, {0x37b,0x00}, {0x37c,0x00}, {0x37d,0x00}, {0x37e,0x00}, {0x37f,0x00},
	{0x380,0x01}, {0x381,0x01},
	{0x382,0x04},	/* st1 -- read back right after this image loads (cksum_err = bit5) */
	/* 0x383..0x38a not written: live status the runtime polls (status2
	 * 0x383, fault-service 0x389) */
	{0x38b,0x00}, {0x38c,0x00}, {0x38d,0x21}, {0x38e,0x00}, {0x38f,0x00},
	{0x390,0x82},	/* BOSA_REG_NUM hi -- the image restores the chip id 0x8290 */
	{0x391,0x90},	/* BOSA_REG_NUM lo */
	{0x392,0x00}, {0x393,0x00},
	{0x394,0x01},	/* BOSA_REG_VID (0x0001) */
	{0x395,0x00}, {0x396,0x00}, {0x397,0x00}, {0x398,0x00},
	{0x399,0x00},	/* bit0 = TOTAL_CHIP_RESET, left clear */
	{0x39a,0x00}, {0x39b,0x15}, {0x39c,0x00}, {0x39d,0x00}, {0x39e,0x00},
	/* 0x39f (REG_LENGTH) intentionally not written */
	{0x3a0,0x00}, {0x3a1,0x00}, {0x3a2,0x00}, {0x3a3,0x02}, {0x3a4,0x00}, {0x3a5,0x00},
	{0x3a6,0x00}, {0x3a7,0x00}, {0x3a8,0x00}, {0x3a9,0x00}, {0x3aa,0x00}, {0x3ab,0x00},
	{0x3ac,0x00}, {0x3ad,0x00}, {0x3ae,0x00}, {0x3af,0x00}, {0x3b0,0x00}, {0x3b1,0x00},
	{0x3b2,0x00}, {0x3b3,0xb6}, {0x3b4,0x58}, {0x3b5,0xf8}, {0x3b6,0x01}, {0x3b7,0x00},
	{0x3b8,0x00}, {0x3b9,0x00}, {0x3ba,0x00}, {0x3bb,0x00}, {0x3bc,0x00}, {0x3bd,0x00},
	{0x3be,0x00}, {0x3bf,0x00}, {0x3c0,0x01}, {0x3c1,0xa0}, {0x3c2,0xac}, {0x3c3,0x40},
	{0x3c4,0x30}, {0x3c5,0x00}, {0x3c6,0x00}, {0x3c7,0x00}, {0x3c8,0x00}, {0x3c9,0x00},
	{0x3ca,0x00}, {0x3cb,0x00}, {0x3cc,0x00}, {0x3cd,0x14}, {0x3ce,0x00}, {0x3cf,0x00},
	{0x3d0,0x00}, {0x3d1,0x00}, {0x3d2,0x00}, {0x3d3,0x00}, {0x3d4,0x00}, {0x3d5,0x00},
	{0x3d6,0x00}, {0x3d7,0x00}, {0x3d8,0x00}, {0x3d9,0x00}, {0x3da,0x00}, {0x3db,0x00},
	{0x3dc,0x00}, {0x3dd,0x00}, {0x3de,0x00}, {0x3df,0x00}, {0x3e0,0x00}, {0x3e1,0x00},
	{0x3e2,0x00}, {0x3e3,0x00}, {0x3e4,0x00}, {0x3e5,0x00}, {0x3e6,0x00}, {0x3e7,0x00},
	{0x3e8,0x44}, {0x3e9,0x00}, {0x3ea,0x00}, {0x3eb,0x00}, {0x3ec,0x00}, {0x3ed,0x00},
	{0x3ee,0x00}, {0x3ef,0x00}, {0x3f0,0x00}, {0x3f1,0x08}, {0x3f2,0x00}, {0x3f3,0x00},
	{0x3f4,0x00}, {0x3f5,0x00}, {0x3f6,0x00}, {0x3f7,0x00}, {0x3f8,0x00}, {0x3f9,0x00},
	{0x3fa,0x00}, {0x3fb,0x00}, {0x3fc,0x00}, {0x3fd,0x00}, {0x3fe,0x00}, {0x3ff,0x00},
};

static void __init bosa_tx_enable(void)
{
	int i;

	/* Load the A4 register image (0x200-0x27c) + base/control config. This is the
	 * patch the BOSA core consumes — it is a plain register image, not a strobed
	 * "activation": no patch-length/activate register is written (REG_LENGTH 0x39f
	 * is intentionally left untouched). The actual laser ignition is the MCU-driven
	 * APC power-on flow run later in bosa_apc_calibrate() (after the SerDes/PON-IP
	 * TX clock is up). */
	for (i = 0; i < ARRAY_SIZE(bosa_init_golden); i++)
		bosa_write_reg(bosa_init_golden[i].reg, bosa_init_golden[i].val);
	mdelay(2);
	pr_info("luna-gpon: A4 image loaded: st1=0x%02x(cksum_err=%d) st2=0x%02x\n",
		bosa_read_reg(0x382) & 0xff,
		!!(bosa_read_reg(0x382) & 0x20), bosa_read_reg(0x383) & 0xff);

	for (i = 0; i < ARRAY_SIZE(bosa_tx_golden); i++)
		bosa_write_reg(bosa_tx_golden[i].reg, bosa_tx_golden[i].val);
	mdelay(10);
	pr_info("luna-gpon: BOSA TX/laser config %s (bias=0x%02x mod=0x%02x w46=0x%02x w48=0x%02x)\n",
		(bosa_not_8290b || bosa_id_num != 0x8290) ? "NOT APPLIED (writes refused)"
							  : "applied",
		bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x237) & 0xff,
		bosa_read_reg(0x22e) & 0xff, bosa_read_reg(0x230) & 0xff);
}

/* Set once the cold ignition has run, so the periodic fault-service (driven from
 * the GPON FSM timer) only touches the laser after DIGITAL_POWER_ON. */
static int bosa_laser_up;
static int apc_offk_armed;	/* rtl8290b_apc_init armed FSU/OFFK; servo will latch */
static int apc_offk_latched;	/* runtime servo latched OFFK (R29 0x31d &0x3c==0x3c) */
static u32 bosa_maint_faults;		/* recovery attempts (see the print below) */
#define BOSA_MAINT_NONE		0xffffffffu
static u32 bosa_maint_last = BOSA_MAINT_NONE;	/* last (0x383,0x389,R30) printed */
static u32 bosa_maint_since;		/* fault count when that state began */
static u32 bosa_stat_ticks;		/* heartbeat counter for the live status log */

/*
 * Laser fault re-arm -- the BOSA "light re-arm" path for a recoverable fault:
 * TX power control and laser-diode VDD on in CONTROL2, then pulse CONTROL3
 * bit1 (UNDER_RX_OVER_POWER_RELEASE) 1 -> 500 us -> 0; the 1->0 edge clears the
 * latched fault. It DELIBERATELY does NOT write 0x399 bit0: that is
 * TOTAL_CHIP_RESET, and an earlier version pulsed it on every re-arm, resetting
 * the BOSA and wiping the ignition. No mdelay, so it is callable from the FSM
 * timer; recovery is re-checked on the next tick. Never raises bias or mod --
 * the operating point stays clamped to the per-board calibrated LUT.
 */
static void bosa_fault_rearm(void)
{
	bosa_set_bit(BOSA_REG_CONTROL2, 2, 1);		/* CONTROL2 TX_POW_CTL: re-enable TX drv */
	bosa_set_bit(BOSA_REG_CONTROL2, 3, 1);		/* CONTROL2 ENLD_L: re-enable laser-diode */
	bosa_set_bit(0x255, 1, 1);		/* CONTROL3 release strobe: assert */
	udelay(500);				/* 500us release-strobe settle */
	bosa_set_bit(0x255, 1, 0);		/* de-assert: 1->500us->0 clears latch */
	bosa_set_field(BOSA_REG_CONTROL2, 0x80, 0x00);	/* clear soft TX-disable (bit7) -> emit */
}

/*
 * One laser-maintenance pass: poll the BOSA INT/fault status every ~50 ms and
 * re-arm on a recoverable fault. The cold ignition is one-shot, so a transient
 * TX_FAULT after DIGITAL_POWER_ON would otherwise leave the BOSA latched with the
 * laser dark forever. Fault sources reacted to: STATUS_2 bit4 FAULT_STATUS (the
 * authoritative "laser currently disabled") and FAULT_STATUS & 0xd1, the genuine
 * TX-kills -- TX_FAULT, TX_LV, OVER_VOL, OVER_TEMP.
 * OVER_IMPD / MPD_VHIGH are excluded: the bring-up disarms the MPD high/low HW
 * fault-detect so they neither latch the laser nor thrash this loop, and the
 * bias/mod DACs stay clamped to the calibrated LUT, so the laser cannot
 * physically over-drive even with MPD detect off.
 */
static void bosa_laser_maint(void)
{
	int s2 = bosa_read_reg(0x383);
	int fs = bosa_read_reg(0x389);

	if (s2 < 0 || fs < 0)
		return;				/* I2C glitch — retry next tick */

	/* Live laser-state heartbeat (~every 2.5s) — bring-up visibility into whether
	 * the laser actually emits (mpd != 0) and holds bias once the FSM is in O3 and
	 * bursting upstream. EN_L = W4/0x204 bit4 (laser booster output enable). */
	if (trace && (bosa_stat_ticks++ % 50) == 0)
		pr_info("luna-gpon: laser stat: 0x383=0x%02x 0x389=0x%02x R30=0x%02x bias=0x%02x mod=0x%02x mpd=%02x/%02x EN_L=%d state=O%u\n",
			s2 & 0xff, fs & 0xff, bosa_read_reg(0x31e) & 0xff,
			bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x237) & 0xff,
			bosa_read_reg(0x320) & 0xff, bosa_read_reg(0x321) & 0xff,
			!!(bosa_read_reg(0x204) & 0x10), gpon_fsm_state);

	if (s2 & BIT(5))
		return;				/* STATUS_2 b5 DEBUG_MODE — BOSA wedged, don't poke */
	if (!(fs & 0xd1))
		return;				/* healthy: react only to genuine 0x389 TX-kill,
					 * not the benign aggregate 0x383 b4 (always set in
					 * the B-flow; re-arming on it tips the BOSA into
					 * DEBUG_MODE). */

	/*
	 * PRINT ON CHANGE, NOT EVERY 32nd TIME -- the count-based limit that was here
	 * is MEASURED to be useless. On the G24W 2026-08-31 this line came out every
	 * 3.22 s for 21 HOURS, so the fault path fires ~10 times a second and
	 * one-in-32 still floods. The cost was the WHOLE log: `dmesg | wc -l` was
	 * 1542 and the grep count for this message was ALSO 1542, with its oldest
	 * entry at t=72138 s on a board up 77087 s -- boot, probe, every other driver
	 * and whatever the GPON stack said about WHY the laser was faulting had all
	 * been evicted by the report of the fault.
	 * On-change loses nothing: every one of those 1542 lines was identical, so
	 * the new form emits ONE line for that whole run and carries the suppressed
	 * count -- strictly more informative than the flood it replaces.
	 */
	{
		u32 r30 = bosa_read_reg(0x31e) & 0xff;
		u32 now = ((s2 & 0xff) << 16) | ((fs & 0xff) << 8) | r30;

		bosa_maint_faults++;
		if (now != bosa_maint_last) {
			pr_info("luna-gpon: laser maint re-arm (0x383=0x%02x 0x389=0x%02x R30=0x%02x)%s\n",
				s2 & 0xff, fs & 0xff, r30,
				bosa_maint_last == BOSA_MAINT_NONE ? "" :
				" [state changed]");
			if (bosa_maint_last != BOSA_MAINT_NONE)
				pr_info("luna-gpon: ... the previous laser-maint state held for %u re-arm(s)\n",
					bosa_maint_faults - bosa_maint_since);
			bosa_maint_last = now;
			bosa_maint_since = bosa_maint_faults;
		}
	}
	bosa_fault_rearm();
}

/*
 * The W77 MCU command walk, once. The RTL8290B's on-chip 8051 consumes W77
 * (0x24d) as a COMMAND BYTE: write one, let it settle, then read R29 (0x31d),
 * which the MCU treats as its acknowledgement. Both APC paths walked the same two
 * batches with their own byte-identical copies of the tables.
 * W77's bit map is the transceiver's OWN: the vendor europa register definition
 * (tier 3) lays it out as BIAS_MAX_EN[7] / BIAS_MAX_LOADIN[6] / MOD_MAX_EN[5] /
 * MOD_MAX_LOADIN[4] / BACKUP[3:0], and the stock ignition disassembly (tier 2)
 * writes these same twelve bytes in this order. The names live in
 * luna_gpon_regs.h and the static_asserts below pin each command to the stock
 * byte -- which settles an old contradiction: 0xb0 is
 * BIAS_MAX_EN|MOD_MAX_EN|MOD_MAX_LOADIN, not the 0xa0 one site spelled it as.
 * NOT ESTABLISHED: bit 3, set by 0xa8/0xd8/0xe8/0xb8. The definition only NAMES
 * the field it sits in ("BACKUP") and nothing on this bench says what the MCU
 * does with it, so it is spelled BOSA_W77_BACKUP_B3 -- the field's name, never a
 * meaning.
 * THE SETTLE IS A PARAMETER, NOT A CONSTANT: the calibrate path waits 10 ms per
 * command and the apc_init path 11 ms, both preserved exactly -- unifying them
 * would be a timing change wearing a cleanup's clothes.
 */
static const u8 bosa_w77_batch1[] = {
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN | BOSA_W77_BACKUP_B3,		/* 0xa8 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN | BOSA_W77_MOD_MAX_LOADIN,		/* 0xb0 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_LOADIN,	/* 0xd0 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_LOADIN
		| BOSA_W77_BACKUP_B3,							/* 0xd8 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_EN
		| BOSA_W77_BACKUP_B3,							/* 0xe8 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_EN,		/* 0xe0 */
};
static const u8 bosa_w77_batch2[] = {
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN | BOSA_W77_MOD_MAX_LOADIN,		/* 0xb0 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_LOADIN,	/* 0xd0 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN | BOSA_W77_MOD_MAX_LOADIN
		| BOSA_W77_BACKUP_B3,							/* 0xb8 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN | BOSA_W77_MOD_MAX_LOADIN,		/* 0xb0 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_LOADIN,	/* 0xd0 */
	BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN,					/* 0xc0 */
};
/* The named form MUST reproduce the stock bytes -- one line per distinct
 * command, so a re-defined bit cannot silently move a write. */
static_assert((BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN | BOSA_W77_BACKUP_B3) == 0xa8, "W77 0xa8");
static_assert((BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN | BOSA_W77_MOD_MAX_LOADIN) == 0xb0, "W77 0xb0");
static_assert((BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN | BOSA_W77_MOD_MAX_LOADIN | BOSA_W77_BACKUP_B3) == 0xb8, "W77 0xb8");
static_assert((BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN) == 0xc0, "W77 0xc0");
static_assert((BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_LOADIN) == 0xd0, "W77 0xd0");
static_assert((BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_LOADIN | BOSA_W77_BACKUP_B3) == 0xd8, "W77 0xd8");
static_assert((BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_EN) == 0xe0, "W77 0xe0");
static_assert((BOSA_W77_BIAS_MAX_EN | BOSA_W77_BIAS_MAX_LOADIN | BOSA_W77_MOD_MAX_EN | BOSA_W77_BACKUP_B3) == 0xe8, "W77 0xe8");

static void bosa_w77_walk(const u8 *cmds, unsigned int n, unsigned int settle_ms)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		bosa_write_reg(BOSA_REG_W77, cmds[i]);	/* the MCU command */
		mdelay(settle_ms);
		bosa_read_reg(BOSA_REG_R29);		/* status; the MCU consumes the read */
	}
}

/*
 * Cold laser ignition -- the RTL8290B's MCU-driven APC power-on as the
 * register-level sequence the silicon requires. The A4 register image arms the
 * on-chip APC core; this runs the ignition:
 *   MCU power-on gate -> CHECK_READY -> BIAS_POWER_ON -> DIGITAL_POWER_ON ->
 *   enable the hardware APC servo loop -> offset-cal lock loop -> TX-enable flow.
 * Once the APC loop is enabled the BOSA core servoes bias and modulation
 * autonomously, so there is no software servo here, and every laser-drive value
 * is the device's own ignition limit -- none is raised.
 * MUST run after the SerDes/PON-IP TX clock is up, because the APC-digital block
 * is clocked from it; hence it is deferred until after the GPON MAC reset.
 */
static void __init bosa_apc_calibrate(void)
{
	int i, k, locked = 0;
	u8 v;

	/* APC power setpoints (DCL P0/P1/Pavg) — the laser power TARGET the BOSA
	 * servo regulates toward. These are PER-BOARD values from the optical
	 * calibration data; a wrong target over-drives the laser and trips the
	 * MPD-VHIGH fault. TODO: load from per-board calibration at startup like the
	 * MAC/SN; this is the Board-C calibrated DCL set. Written after the A4 image
	 * (which leaves patch bytes here) and before the ignition. */
	bosa_write_reg(0x23a, 0x26);		/* W58 DCL P0   */
	bosa_write_reg(0x23b, 0x50);		/* W59 DCL P1   */
	bosa_write_reg(0x23d, 0x50);		/* W61 DCL Pavg */

	/* MCU power-on gate: wait for the BOSA core boot/power-on-reset to finish
	 * (STATUS_2 0x383: LVCMP_TX_VALID(7) + TEMP_VALID(6); DEBUG_MODE(5) stays 0 in
	 * normal operation) plus the page-3 reset-done bit; then a 15ms settle. */
	for (i = 0; i < 2000; i++) {
		int s = bosa_read_reg(0x383);

		if (s >= 0 && (s & 0xc0) == 0xc0)
			break;
		udelay(1000);
	}
	if (i == 2000)
		pr_warn("luna-gpon: BOSA MCU power-on (0x383&0xc0) not ready\n");
	bosa_poll_bit(0x301, 7, 0, 1000, 2000);		/* page-3 reset-done clears */
	mdelay(15);

	/* --- APC-enable flow, step order 0,1,2,3,4,6,5,7 --- */
	bosa_set_bit(0x3c0, 0, 1);			/* idx0: entry reset/disable */

	/* idx1 CHECK_READY: STATUS_1(0x382) bit2 = READY_STATUS */
	if (!bosa_poll_bit(0x382, 2, 1, 1000, 2000))
		pr_warn("luna-gpon: BOSA APC CHECK_READY (0x382 bit2) timeout\n");

	/* idx2 BIAS_POWER_ON: arm the analog bias front-end + the APC bias targets.
	 * bias-max W72(0x248)=0x86 then 0x87, bias-min W73(0x249)=0x06 are the
	 * ignition ceilings; the servo converges the live bias well below them. */
	bosa_set_field(0x245, 0xff, 0x10);
	bosa_set_field(0x245, 0x0c, 0x00);		/* W6932 field (default 0) */
	bosa_write_reg(0x284, 0x01);
	bosa_write_reg(0x27c, 0x08);
	bosa_write_reg(0x247, 0x05);
	bosa_write_reg(0x248, 0x86);			/* W72 bias-max */
	bosa_write_reg(0x239, 0xfc);
	bosa_set_bit(0x24a, 3, 1);
	bosa_write_reg(0x249, 0x06);			/* W73 bias-min */
	bosa_write_reg(0x24c, 0x71);
	bosa_write_reg(0x24c, 0x72);
	bosa_write_reg(0x247, 0x06);
	bosa_write_reg(0x248, 0x87);			/* W72 bias-max final */
	/* loop_mode 0 (W69=0x00): 0x23e source = !(0x239 bit3) */
	v = bosa_get_field(0x239, BIT(3)) ^ 1;
	bosa_set_field(0x23e, 0xff, v);
	bosa_write_reg(0x232, 0x07);
	bosa_write_reg(0x244, 0xf8);
	bosa_set_bit(0x252, 3, 1);			/* W82 DAC/loop commit */

	bosa_set_field(0x239, 0xff, 0xfc);		/* idx3 */
	bosa_set_field(0x23c, 0xff, 0xfd);		/* idx4 */

	/* RTL8290B MCU bias/mod-MAX loadin handshake (the APC-init W77 sequence). This
	 * BOSA's on-chip 8051 OWNS laser-enable and bias, so the ignition must drive it
	 * through power-on by writing command bytes to W77, each followed by a settle
	 * and an R29 status read the MCU consumes. An ignition that omits this
	 * handshake leaves the MCU with the laser never enabled. The bytes strobe
	 * BIAS_MAX_EN/LOADIN and MOD_MAX_EN/LOADIN to latch the max limits set above;
	 * bosa_w77_walk() owns the tables and says where the names came from. */
	{
		bosa_set_bit(0x24e, 7, 1);		/* W78 b7 (apc_init prefix) */
		bosa_w77_walk(bosa_w77_batch1, ARRAY_SIZE(bosa_w77_batch1), 10);
		bosa_set_bit(0x243, 7, 1);		/* W67 b7 */
		bosa_set_field(0x27c, 0x08, 0x00);	/* W80 clear bit3 */
		bosa_w77_walk(bosa_w77_batch2, ARRAY_SIZE(bosa_w77_batch2), 10);
		pr_info("luna-gpon: DBG post-W77hs: EN_L=%d bias=0x%02x R29=0x%02x R33=0x%02x 0x383=0x%02x R30=0x%02x\n",
			!!(bosa_read_reg(0x204) & 0x10), bosa_read_reg(0x236) & 0xff,
			bosa_read_reg(0x31d) & 0xff, bosa_read_reg(0x321) & 0xff,
			bosa_read_reg(0x383) & 0xff, bosa_read_reg(0x31e) & 0xff);
	}

	v = bosa_get_field(0x31f, 0x03);		/* idx6 */
	bosa_set_field(0x232, 0xc0, v);
	bosa_set_field(0x249, 0x18, v);

	/* TEST: pre-load this board's calibrated laser bias/mod (per-board optical
	 * calibration LUT @25C = 0x18/0x34, 12-bit DAC = byte<<4) BEFORE turning the
	 * laser on, so it ignites at the right optical power instead of the hotter
	 * default that trips MPD_VHIGH at DIGITAL_POWER_ON. */
	bosa_set_bit(0x23d, 7, 0);
	bosa_set_field(0x236, 0xff, 0x18);		/* bias hi-8 (calibrated LUT @25C). NOTE: lowering to 0x0a did NOT save DS RX — laser-on deafens RX independent of optical power; the fix is burst-gating, not bias level. */
	bosa_set_field(0x238, 0x0f, 0x00);
	bosa_set_bit(0x23d, 7, 1);
	bosa_set_bit(0x23d, 7, 0);
	bosa_set_field(0x237, 0xff, 0x34);		/* mod hi-8 */
	bosa_set_field(0x238, 0xf0, 0x00);
	bosa_set_bit(0x23d, 7, 1);

	/* Disarm W53/0x235 fault-detect for the rest of ignition (after the W77
	 * handshake — disarming it BEFORE the handshake regressed O3->O1). Keeps a
	 * (false) MPD_VHIGH from latching 0x383 b4 at DPO and zeroing the bias; the safe
	 * subset is re-armed at txEnableFlow end. */
	bosa_set_field(0x235, 0xff, 0x00);

	/* idx5 DIGITAL_POWER_ON: turn on the digital/laser power, then blind settle */
	bosa_set_bit(0x27c, 4, 1);			/* W80 bit4 = 1 */
	bosa_set_bit(0x380, 0, 1);
	mdelay(101);
	pr_info("luna-gpon: DBG post-DPO: bias(0x236)=0x%02x 0x389=0x%02x 0x383=0x%02x R30=0x%02x\n",
		bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x389) & 0xff,
		bosa_read_reg(0x383) & 0xff, bosa_read_reg(0x31e) & 0xff);

	bosa_set_bit(0x23c, 0, 1);			/* idx7 */
	bosa_set_bit(BOSA_REG_CONTROL2, 3, 1);			/* CONTROL2 bit3 = 1 */
	mdelay(5);

	/* HYPOTHESIS TEST: disable the laser TX (CONTROL2/0x254 bit7=1) during the
	 * offset-K calibration so the ADC zero-offset is measured with no emission and
	 * the TX path can't trip TX_FAULT while the analog block is mid-cal. The
	 * txEnableFlow below re-enables TX (0x254 bit7=0). */
	bosa_set_bit(BOSA_REG_CONTROL2, 7, 1);

	/* RTL8290B FSU (Field Setup Unit) offset/gain auto-cal plus DCL convergence. A
	 * plain offset-K that polls R30 bit7 OFFK_DONE never completes on this part.
	 * The W77 handshake above started the MCU biasing, but the APC must CONVERGE
	 * here before TX is enabled, or it collapses the bias to 0 -- thrashing without
	 * feedback. Flow: select DCL closed-loop mode, arm the FSU, fsuMode 0, arm the
	 * done check and poll the FSU done nibble on R29 (& 0x3c == 0x3c) -- NOT R30
	 * OFFK_DONE -- then latch. */
	bosa_set_field(0x27c, 0xc0, 0x03);	/* apcLoopMode DCL: W80[7:6]=3 */
	bosa_set_bit(0x27c, 5, 0);		/* FSU arm: W80 b5 low */
	bosa_set_bit(0x27c, 4, 1);		/*          W80 b4 high */
	bosa_set_bit(0x20e, 7, 1);		/*          W14 b7 high (LOADIN) */
	bosa_set_bit(0x27c, 5, 1);		/*          W80 b5 high (path strobe) */
	bosa_set_bit(0x241, 6, 0);		/* fsuMode 0: W65 b6 */
	bosa_write_reg(BOSA_REG_W77, BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN |
		       BOSA_W77_MOD_MAX_LOADIN);	/* W77=0xb0 arms the done-check */
	for (k = 0; k < 250; k++) {
		int r29 = bosa_read_reg(BOSA_REG_R29);

		if (r29 >= 0 && (r29 & 0x3c) == 0x3c) {
			locked = 1;
			break;
		}
		udelay(200);
	}
	bosa_set_bit(0x20e, 7, 0);		/* finalize: de-assert W14 LOADIN */
	bosa_set_bit(0x27c, 4, 0);		/*           de-assert W80 b4 -> latch */
	pr_info("luna-gpon: DBG post-FSU: done=%d R29=0x%02x bias=0x%02x R33=0x%02x 0x383=0x%02x 0x27c=0x%02x\n",
		locked, bosa_read_reg(0x31d) & 0xff, bosa_read_reg(0x236) & 0xff,
		bosa_read_reg(0x321) & 0xff, bosa_read_reg(0x383) & 0xff,
		bosa_read_reg(0x27c) & 0xff);

	/* --- txEnableFlow (B-flow: laser output enable AFTER FSU convergence) ---
	 * NB: an earlier TX-enable flow rewrote W77/0x24d (=0xa5, then low-nibble=1),
	 * which CORRUPTED the converged MCU command state and tipped the BOSA into
	 * DEBUG_MODE (every reg 0x20) once TX was enabled. Those W77 writes are removed;
	 * the FSU/DCL above already owns W77. */
	bosa_set_field(BOSA_REG_CONTROL2, 0xff, 0x8d);

	/* The laser bias/mod LUT -- the PER-BOARD calibrated operating point. The
	 * optical calibration holds a 151-entry {bias,mod} table indexed by temperature
	 * (stride 2, idx = temp_code - 233, i.e. -40..110 C). Both are 12-bit DACs =
	 * (LUT byte << 4), committed by the 0x23d bit7 strobe.
	 * Without it the DACs sit hotter than THIS laser's calibration, the part emits
	 * above its monitor-photodiode high threshold, and the BOSA safety shuts the
	 * laser off. Loading the calibrated point lets it ignite at the right optical
	 * power and the APC servo hold it.
	 * This is Board C's room-temp entry (idx65 / 25 C), and idx60..69 is flat, so
	 * it is robust ~20-29 C. PER-BOARD: the fleet image loads it from the device's
	 * own calibration data, and NEVER exceeds the per-temperature LUT byte. */
	{
		u8 lut_bias = 0x18, lut_mod = 0x34;	/* laser LUT @ 25C (calibrated). lowering bias to 0x0a did NOT save DS RX (laser-on deafens RX regardless of optical power) -> the fix is burst-gating the TX path, not the bias level */

		bosa_set_bit(0x23d, 7, 0);		/* bias DAC: strobe low */
		bosa_set_field(0x236, 0xff, lut_bias);	/* W54 bias hi-8 (= bias12[11:4]) */
		bosa_set_field(0x238, 0x0f, 0x00);	/* W56 bias12[3:0] (LUT<<4 -> 0) */
		bosa_set_bit(0x23d, 7, 1);		/* latch */
		bosa_set_bit(0x23d, 7, 0);		/* mod DAC: strobe low */
		bosa_set_field(0x237, 0xff, lut_mod);	/* W55 mod hi-8 (= mod12[11:4]) */
		bosa_set_field(0x238, 0xf0, 0x00);	/* W56 mod12[3:0] (LUT<<4 -> 0) */
		bosa_set_bit(0x23d, 7, 1);		/* latch */
	}

	/* (removed an idx4 0x245 loop_mode write — the RTL8290B loop mode is W80[7:6],
	 * set by FSU/DCL) */
	bosa_set_field(0x230, 0xff, 0x00);		/* idx5 */
	bosa_set_field(0x27c, 0xff, 0xe9);		/* W80=0xe9: converged (DCL[7:6]=3 + b5 + 0x09) */
	mdelay(51);					/* idx6 */
	/* (removed a 0x24d low-nibble write — W77 owned by the FSU above) */

	/* BOOSTER — the laser-diode driver OUTPUT stage. It must be enabled here; if it
	 * is not, W4/0x204 bit4 EN_L stays 0 (the W4=0x8e operating value leaves bit4
	 * clear) and NO laser current flows even with the calibrated bias loaded -> the
	 * monitor photodiode reads 0 -> the APC servo, seeing no optical feedback,
	 * collapses the bias DAC to 0 and the laser stays dark. Sequence: CONTROL2 bit6
	 * LOS_PIN_TRI low, assert EN_L, ~200ms laser-bias settle, CONTROL2 bit6 high. */
	bosa_set_field(BOSA_REG_CONTROL2, 0x40, 0x00);		/* CONTROL2 bit6 LOS_PIN_TRI = 0 */
	bosa_set_bit(BOSA_REG_W4, 4, 1);			/* W4 EN_L = 1: laser booster ON */
	mdelay(200);					/* 200ms laser-bias settle */
	bosa_set_field(BOSA_REG_CONTROL2, 0x40, 0x40);		/* CONTROL2 bit6 LOS_PIN_TRI = 1 */

	bosa_set_field(BOSA_REG_CONTROL2, 0x80, 0x00);		/* idx8 CONTROL2 bit7 = 0 */
	/* W53 fault-detect enables. The device default arms ALL (0xff), but on this
	 * board the MPD high/low APC fault-detect trips a FALSE MPD_VHIGH the instant TX
	 * is enabled and latches the laser off -- even though the DACs are clamped to
	 * the calibrated LUT, so no real over-power is possible. Arm everything EXCEPT
	 * that detect (0xfc); bosa_laser_maint() still watches the genuine TX-kills and
	 * the R30 MPD bits are polled in software. Laser safety is held by the LUT
	 * clamp, not by this comparator. */
	bosa_set_field(0x235, 0xff, 0xfc);		/* idx7: arm faults, MPD hi/lo OFF */
	bosa_set_field(0x25f, 0xff, 0x02);
	bosa_set_field(0x260, 0xff, 0x00);

	/* Post-enable settle + diagnostic. NO init re-arm loop here: the old bounded
	 * bosa_fault_rearm loop fired on the benign aggregate 0x383 b4, and in the
	 * converged B-flow that corrupts the MCU state and tips the BOSA into DEBUG_MODE
	 * (every reg 0x20). Just log the live state; the FSM-timer maintenance services
	 * genuine 0x389 TX-kill faults from here (and skips a DEBUG_MODE-wedged BOSA). */
	mdelay(50);
	pr_info("luna-gpon: post-txen: 0x389=0x%02x 0x383=0x%02x R30=0x%02x bias=0x%02x R33=0x%02x mod=0x%02x EN_L=%d\n",
		bosa_read_reg(0x389) & 0xff, bosa_read_reg(0x383) & 0xff,
		bosa_read_reg(0x31e) & 0xff, bosa_read_reg(0x236) & 0xff,
		bosa_read_reg(0x321) & 0xff, bosa_read_reg(0x320) & 0xff,
		!!(bosa_read_reg(0x204) & 0x10));

	/*
	 * BURST-GATE the laser. EN_L is the booster OUTPUT-enable: held at 1 it forces
	 * continuous-wave emission, whose light DEAFENS the shared-BOSA downstream RX
	 * -- the root cause of "the OLT never ranges us" (laser-on gives LOS+LOF with
	 * ds_rx frozen; laser-off gives a locked framer and ds_rx climbing). The
	 * operational burst-mode value at O5 has EN_L=0: the device does not pin it on,
	 * and the per-burst emission is gated downstream by the SoC BEN. EN_L=1 is
	 * needed only TRANSIENTLY during ignition to flow bias and seat the APC, and
	 * the bias DAC stays loaded once seated. If the bias collapses here, the APC
	 * convergence is the real gap and the readback below makes that visible.
	 */
	bosa_set_bit(BOSA_REG_W4, 4, 0);			/* EN_L = 0: burst-gate (0x8e) */
	mdelay(5);
	pr_info("luna-gpon: burst-gate EN_L=0 -> 0x204=0x%02x R33=0x%02x R30=0x%02x 0x383=0x%02x\n",
		bosa_read_reg(0x204) & 0xff, bosa_read_reg(0x321) & 0xff,
		bosa_read_reg(0x31e) & 0xff, bosa_read_reg(0x383) & 0xff);

	/* Hand off to the continuous fault-service driven from the GPON FSM timer. */
	bosa_laser_up = 1;

	pr_info("luna-gpon: laser ignite: lock=%d R30=0x%02x(offk_done=%d txsd=%d) bias=0x%02x mod=0x%02x mpd=%02x/%02x\n",
		locked, bosa_read_reg(0x31e) & 0xff,
		!!(bosa_read_reg(0x31e) & BIT(7)), !!(bosa_read_reg(0x31e) & BIT(6)),
		bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x238) & 0xff,
		bosa_read_reg(0x320) & 0xff, bosa_read_reg(0x321) & 0xff);
}

/*
 * RTL8290B B-variant laser APC/OFFK ignition -- the flow stock runs for this chip
 * type, and the one our non-B bosa_apc_calibrate() gets wrong. The decisive
 * difference is the OFFK (modulator offset) calibration: the B chip completes it
 * via the FSU done-flag (R29 & 0x3c == 0x3c), NOT the non-B R30 bit7 poll.
 * Without OFFK the modulator offset is never nulled, so the laser emits DC
 * between bursts and deafens the shared downstream RX.
 *   - gates on the MCU power-on and CHECK_READY before touching anything;
 *   - writes the W62/W63 OFFK_EN trio in B order and the per-board DCL setpoints;
 *   - runs both W77 MCU-command handshake batches;
 *   - runs the FSU/OFFK-FSM config and ARM, then POLLS R29 to completion with a
 *     bounded timeout and a pr_warn on failure -- it does not declare done on the
 *     wrong R30 poll;
 *   - latches on full-done and loads the per-board operating bias/mod LUT;
 *   - keeps EN_L burst-gated: 1 only transiently to seat the APC, then 0, so the
 *     laser is not CW and the downstream RX survives;
 *   - aborts the ignition window if the monitor photodiode reads no feedback or a
 *     hard TX-kill latches, so the booster never runs open-loop above the
 *     calibrated ceiling.
 */
static void __init rtl8290b_apc_init(void)
{
	/* Exact stock B-variant laser APC init behavior (GPON/pon=1), re-expressed from
	 * the observed stock register sequence with the per-board optics-calibration table, via
	 * the bosa_write_reg / bosa_set_bit / bosa_set_field helpers
	 * (val shifted into the mask's low bit, == stock). The OFFK offset
	 * compute+write (0x243/0x245) is GATED on optics-cfg byte 0x72 != 0 in stock; ours is 0,
	 * so stock SKIPS it -> we MUST NOT write it (the spurious write parked R29 at
	 * 0x0e). The terminal OFFK latch ((R29 0x31d &0x3c)==0x3c) is done by the
	 * runtime servo in gpon_fsm_poll, once the laser bursts at O5. */
	int i;
	u8 t8;

	pr_info("luna-gpon: rtl8290b_apc_init: B-variant OFFK (exact stock seq, base 0x578)\n");

	/* step 1: MCU power-on kick + gate + ~11ms settle */
	bosa_write_reg(0x380, 0x01);
	if ((bosa_read_reg(0x380) & 0xff) != 1) {
		pr_warn("luna-gpon: rtl8290b_apc_init: MCU power-on (0x380!=1) -> abort\n");
		return;
	}
	for (i = 0; i < 11; i++)
		udelay(1000);

	/* CHECK_READY: (0x383&0xe0)==0xc0, then (0x301 b7) */
	for (i = 0; i < 20000; i++) {
		int sret = bosa_read_reg(0x383);

		if (sret >= 0 && (sret & 0xe0) == 0xc0)
			break;
		udelay(50);
	}
	bosa_poll_bit(0x301, 7, 1, 50, 20000);

	/* step 2: OFFK enable trio (W78 b7; W62=W63=0xfd full bytes) */
	bosa_set_bit(0x24e, 7, 1);
	bosa_write_reg(0x23e, 0xfd);
	bosa_write_reg(0x23f, 0xfd);

	/* step 3: per-field config (optics-cfg table @0x578, pon=1 values) */
	bosa_write_reg(0x23a, 0x00);		/* apcIavg = 0			*/
	bosa_set_field(0x23b, 0xf0, 0x02);	/* apcEr (optics-cfg[0x1e]=2)		*/
	bosa_set_field(0x23b, 0x0c, 0x03);	/* apcApcTimer = 3		*/
	bosa_set_field(0x23b, 0x03, 0x03);	/* apcErcTimer = 3		*/
	bosa_set_field(0x240, 0x03, 0x00);	/* apcLpfBw = 0			*/
	bosa_set_bit(0x23e, 0, 0);		/* txsdMode = 0			*/
	bosa_set_field(0x24c, 0xe0, 0x01);	/* txsdTh = 1			*/
	bosa_set_bit(0x24c, 4, 1);		/* txsdTiaGain = 1		*/
	bosa_set_bit(0x24c, 3, 1);		/* txsdHighLoopGain = 1		*/
	for (i = 0; i < 11; i++)
		udelay(1000);
	bosa_read_reg(0x31d);

	/* step 4: W77 (0x24d) MCU-command walk BATCH 1 */
	bosa_w77_walk(bosa_w77_batch1, ARRAY_SIZE(bosa_w77_batch1), 11);

	/* step 5: ERC chopper + W80 b3 toggle */
	bosa_set_bit(0x243, 7, 1);
	bosa_set_bit(0x284, 6, 1);
	bosa_set_bit(0x284, 5, 1);
	bosa_set_bit(0x284, 4, 1);		/* apcErcChopperEn = 1 (b6/b5/b4) */
	t8 = bosa_read_reg(0x27c) & 0xff;
	bosa_write_reg(0x27c, t8 & 0xf7);
	bosa_write_reg(0x27c, t8 | 0x08);

	/* step 6: W77 walk BATCH 2 */
	bosa_w77_walk(bosa_w77_batch2, ARRAY_SIZE(bosa_w77_batch2), 11);

	/* step 7: OFFK offset compute+write -- STOCK SKIPS IT (optics-cfg[0x72]=0). */

	/* step 8: bias/mod init-code seeds (optics-cfg[0x65]=0 -> manual path) + loop mode */
	bosa_set_bit(0x23d, 7, 0);
	bosa_set_field(0x236, 0xff, 0x10);	/* Ibias init 0x100 >> 4 (stock seed)	*/
	bosa_set_field(0x238, 0x0f, 0x00);	/* Ibias[3:0] = 0		*/
	bosa_set_bit(0x23d, 7, 1);
	bosa_set_bit(0x23d, 7, 0);
	bosa_set_field(0x237, 0xff, 0x20);	/* Imod init 0x200 >> 4 (stock seed)	*/
	bosa_set_field(0x238, 0xf0, 0x00);	/* Imod[3:0] = 0		*/
	bosa_set_bit(0x23d, 7, 1);
	bosa_set_field(0x27c, 0xc0, 0x03);	/* apcLoopMode DCL (remap 1->3)	*/
	bosa_set_bit(0x24a, 4, 0);		/* apcLoopModeEx = 0		*/
	bosa_set_field(0x23d, 0x70, 0x04);	/* apcLaserOnDelay = 4		*/
	bosa_set_field(0x23d, 0x03, 0x02);	/* apcSettleCnt = 2		*/
	bosa_set_field(0x239, 0x38, 0x05);	/* apcApcLoopGain = 5		*/
	bosa_set_field(0x23c, 0x07, 0x00);	/* apcCmpd = 0			*/
	bosa_set_field(0x239, 0x07, 0x03);	/* apcErcLoopGain = 3		*/
	bosa_set_field(0x23c, 0xf8, 0x05);	/* apcErTrim = 5		*/

	/* step 9: FSU config (the previously-guessed block, now exact) */
	bosa_set_bit(0x241, 6, 1);		/* fsuMode = 1			*/
	bosa_set_field(0x241, 0x30, 0x01);	/* fsuApcLoopGain = 1 (b4-5)	*/
	bosa_set_field(0x241, 0x0c, 0x01);	/* fsuApcRampb = 1 (b2-3)	*/
	bosa_set_field(0x241, 0x03, 0x03);	/* fsuApcRampm = 3		*/
	bosa_set_bit(0x242, 7, 1);		/* fsuRstCount = 1		*/
	bosa_set_field(0x242, 0x60, 0x02);	/* fsuSettleCount = 2 (b5-6)	*/
	bosa_set_field(0x242, 0x18, 0x02);	/* fsuErcLoopGain = 2 (b3-4)	*/
	bosa_set_field(0x242, 0x06, 0x00);	/* fsuErcRampm = 0 (b1-2)	*/

	/* step 10: txsd off-rst */
	bosa_set_bit(0x27c, 3, 1);		/* txsdOffRstCount = 1		*/

	/* step 11: ceilings + enables (modmax 0xcc, biasmax 0x4d, biasmin 0x01) */
	bosa_set_bit(0x246, 2, 0);
	bosa_set_field(0x247, 0xff, 0xcc);
	bosa_set_bit(0x246, 2, 1);		/* apcModMax = 0xcc		*/
	bosa_set_bit(0x246, 1, 0);
	bosa_set_field(0x248, 0xff, 0x4d);
	bosa_set_bit(0x246, 1, 1);		/* apcBiasMax = 0x4d		*/
	bosa_set_bit(0x284, 7, 0);		/* apcCrossEn = 0		*/
	bosa_set_bit(0x246, 0, 0);
	bosa_set_field(0x249, 0xff, 0x01);
	bosa_set_bit(0x246, 0, 1);		/* apcBiasMin = 0x01		*/
	bosa_set_bit(0x246, 5, 1);		/* apcModMaxEn = 1		*/
	bosa_set_bit(0x246, 3, 1);		/* apcBiasMinEn = 1		*/
	bosa_set_field(0x283, 0xff, 0x01);	/* apcCrossStr = 1		*/

	/* step 12: FSU ARM (0x27c b5=0->b4=1->0x20e b7=1->0x27c b5=1) + commit */
	bosa_set_bit(0x27c, 5, 0);
	bosa_set_bit(0x27c, 4, 1);
	bosa_set_bit(0x20e, 7, 1);
	bosa_set_bit(0x27c, 5, 1);
	bosa_write_reg(0x232, 0xc0);
	bosa_write_reg(0x24a, 0x60);

	apc_offk_armed = 1;
	pr_info("luna-gpon: rtl8290b_apc_init: armed; post-cfg R29=0x%02x R30=0x%02x 0x241=0x%02x 0x242=0x%02x 0x247=0x%02x 0x248=0x%02x; servo latches at O5\n",
		bosa_read_reg(0x31d) & 0xff, bosa_read_reg(0x31e) & 0xff,
		bosa_read_reg(0x241) & 0xff, bosa_read_reg(0x242) & 0xff,
		bosa_read_reg(0x247) & 0xff, bosa_read_reg(0x248) & 0xff);
}

static void __init bosa_probe(void)
{
	int hb  = bosa_read_reg(BOSA_REG_NUM);
	int lb  = bosa_read_reg(BOSA_REG_NUM + 1);
	int vid = bosa_read_reg(BOSA_REG_VID);
	/* ★★★ A SILENT RTL8290B REGISTER MAP IS NOT A SILENT BUS, AND THIS USED TO
	 * RETURN HERE (repaired 2026-09-12). The early return made everything below
	 * -- the SFF-8472 identification this driver already implements, and whose
	 * own comment says "ask the module what it IS instead" -- UNREACHABLE on
	 * exactly the board that needs it: a module that is not an RTL8290B does not
	 * answer the RTL8290B slave, so hb/lb/vid come back -EIO and we returned
	 * before ever asking slave 0x50 anything.
	 *
	 * ⚠ WHAT THAT COST, and it is why this is a repair and not a tidy-up: the
	 * G24W's BOSA is a Semtech GN25L95 with no EEPROM (stock's own boot says so,
	 * and our capture of 2026-09-06 recorded it). With the identification
	 * unreachable the log offered only "BOSA I2C read failed" plus a chip id of
	 * 0xffffffff -- which is `bosa_id_num`'s -1 SENTINEL printed with %04x, not
	 * an identifier any device returned. A whole session read that pair as "the
	 * I2C bus is dead" and hunted a pad-mux that was never missing.
	 *
	 * ⇒ THREE STATES, and the bus is the one nobody could see before: the module
	 *   is a different part (0x50 answers) / nothing on the bus answers at all
	 *   (0x50 is silent too) / it answered and we still cannot name it.
	 *
	 * ★ NO WRITE PATH OPENS. `bosa_write_reg` refuses on `bosa_id_num != 0x8290`
	 *   and -1 satisfies that, so an unidentified module is as write-protected
	 *   after this change as it was before it. Reads are all that becomes
	 *   reachable, which is the whole point.
	 */
	bool id_answered = (hb >= 0 && lb >= 0 && vid >= 0);

	if (id_answered) {
		bosa_id_num = (hb << 8) | lb;
		bosa_id_vid = vid;

		/* Read the RX-path registers (read-only): RX power-down, SD-pin
		 * tristate, and the live RX loss-of-signal status. These confirm
		 * page-0x54 access and show what the RX-enable writes will need to
		 * change. Skipped when the map never answered -- three more -EIO
		 * stored as if they were readings is how a sentinel becomes a
		 * "measurement" in the next person's log. */
		bosa_w41     = bosa_read_reg(BOSA_REG_W41);
		bosa_ctrl2   = bosa_read_reg(BOSA_REG_CONTROL2);
		bosa_status2 = bosa_read_reg(BOSA_REG_STATUS2);
	} else {
		pr_warn("luna-gpon: the RTL8290B register map did not answer (hb=%d lb=%d vid=%d; -EIO is %d). That is the EXPECTED result for a module that is not an RTL8290B, so on its own it identifies NOTHING -- asking SFF-8472 at slave 0x50 what this module IS.\n",
			hb, lb, vid, -EIO);
	}

	/* An "UNEXPECTED" id used to be logged and then ignored. Ask the module what
	 * it IS instead: SFF-8472 A0 bytes 0/1/2/12 are the identifier, extended
	 * identifier, connector and nominal bit rate, and bytes 20..35 the vendor
	 * name. A plausible identifier byte (SFF-8024 assigns 0x01..0x2x) alongside
	 * a non-0x8290 chip id is a POSITIVE identification of a different module,
	 * which is what gates the write path. Nothing here writes. */
	if (bosa_id_num != 0x8290) {
		int ident = bosa_i2c_read8(0x50, 0);
		int extid = bosa_i2c_read8(0x50, 1);
		int conn  = bosa_i2c_read8(0x50, 2);
		int br    = bosa_i2c_read8(0x50, 12);
		int raw[16];
		char vend[17];
		char part[17];
		int k;

		for (k = 0; k < 16; k++)
			raw[k] = bosa_i2c_read8(0x50, 20 + k);
		bosa_sff_text(vend, raw, 16);
		for (k = 0; k < 16; k++)
			raw[k] = bosa_i2c_read8(0x50, 40 + k);
		bosa_sff_text(part, raw, 16);
		/* Three-outcome identity decision, hoisted: the module's own NAME
		 * outranks the mere presence of an SFF-8472 page (the inversion that
		 * cost the G24W days -- measured 2026-08-30, full rationale at
		 * bosa_module_classify() in luna_gpon_logic.c). This shell keeps
		 * only the I2C sampling above, the prints, and the flag store. */
		/* ★★★ WHAT FAILED, NOT WHAT IT MEANS -- AND THE FIRST DRAFT OF THIS
		 * VERY BLOCK GOT IT WRONG (caught in review before it ever ran,
		 * 2026-09-12). It concluded "NOTHING on I2C bus 0 answers" from two
		 * silent slaves, which is the SAME defect one level out as the early
		 * return this patch removes: a negative result on the addresses we
		 * happened to ask, stated as a verdict about the wire.
		 *
		 * It is wrong on THIS board specifically. The GN25L95's identity does
		 * not live at A0: its recovered calibration has A0[0..126] = 0 with
		 * only [127] = 0x55, and the vendor's own detector reaches it through
		 * a SELECTOR protocol on A2 (slave 0x51) -- which nothing here asks.
		 * So a module that is present and perfectly healthy produces exactly
		 * the silence this block sees, and calling that a dead bus would have
		 * sent the next reader back to the pad mux that is already correct.
		 *
		 * ⇒ REPORT THE PROBES AND NAME WHAT WAS NOT ASKED. Whether the bus is
		 *   down is decided once the Semtech path exists, not here.
		 */
		if (ident < 0 && !id_answered) {
			pr_warn("luna-gpon: BOSA identity probes FAILED: RTL8290B map silent AND SFF-8472 A0 at slave 0x50 silent (ident=%d). ⚠ A2 (slave 0x51) WAS NOT ASKED, and that is where a Semtech GN2xL9x answers -- so this is NOT evidence the I2C bus is down and must not be read as such.\n",
				ident);
			goto id_done;
		}

		switch (bosa_module_classify(ident, extid, vend, part)) {
		case BOSA_MODULE_NAMED_OURS:
			pr_info("luna-gpon: optical module identifies itself as vendor='%s' part='%s' -- RTL8290B register path stays ENABLED (the SFF-8472 page says WHAT it is; merely HAVING one never meant it was foreign)\n",
				vend, part);
			break;
		case BOSA_MODULE_FOREIGN:
			bosa_not_8290b = true;
			pr_warn("luna-gpon: optical module is NOT an RTL8290B -- SFF-8472 A0 ident=0x%02x extid=0x%02x connector=0x%02x br=%d00MBd vendor='%s'. RTL8290B register writes are now REFUSED; DDM must come from A2 (slave 0x51) bytes 104/105, not the analog banks.\n",
				ident, extid, conn, br, vend);
			break;
		case BOSA_MODULE_COULD_NOT_TELL:
		default:
			pr_warn("luna-gpon: BOSA id 0x%04x is not 0x8290 and SFF-8472 A0 did not identify it either (ident=%d) -- leaving the RTL8290B path enabled; this is 'could not tell', not 'it is an 8290B'\n",
				bosa_id_num, ident);
			break;
		}
	}

id_done:
	/* ★ AN UNREAD REGISTER IS NOT A REGISTER READING 0xff. When the map never
	 * answered, `bosa_w41`/`ctrl2`/`status2` still hold their -1 sentinels, and
	 * masking those with 0xff would print rxpwdn=1 los_tri=1 rx_los=1 -- three
	 * confident bits about an RX path nobody sampled. Say COULD NOT ASK. */
	if (!id_answered) {
		pr_info("luna-gpon: BOSA identity NOT ESTABLISHED -- the RTL8290B map is silent, so num/vid and the w41/ctrl2/status2 RX bits were never read and are NOT reported. RTL8290B register writes stay REFUSED.\n");
		return;
	}
	pr_info("luna-gpon: BOSA RTL8290B num=0x%04x vid=0x%02x %s | w41=0x%02x(rxpwdn=%d) ctrl2=0x%02x(los_tri=%d) status2=0x%02x(rx_los=%d)\n",
		bosa_id_num, bosa_id_vid,
		(bosa_id_num == 0x8290) ? "detected" : "UNEXPECTED",
		bosa_w41 & 0xff, (bosa_w41 >> 4) & 1,
		bosa_ctrl2 & 0xff, (bosa_ctrl2 >> 6) & 1,
		bosa_status2 & 0xff, (bosa_status2 >> 2) & 1);
}

/*
 * One per-rate/lane analog bank of the table below: 22 registers at 4-byte
 * stride from `base`.  All four banks share this skeleton; only the ten
 * values passed as arguments differ per bank, and each argument is
 * labelled with its offset from the bank base at the call site.  Expands,
 * in listing order, to exactly the { off, val } pairs the table previously
 * spelled out one by one (proven element-by-element on the host).
 */
#define SDS_ANA_RATE_BANK(base, v24, v28, v2c, v30, v38, v3c, v40, v48, v4c, v50)	\
	{ (base) + 0x00, 0x00000f00 }, { (base) + 0x04, 0x0000b8c6 },	\
	{ (base) + 0x08, 0x0000a112 }, { (base) + 0x0c, 0x00004280 },	\
	{ (base) + 0x10, 0x0000f53f }, { (base) + 0x14, 0x00004fdf },	\
	{ (base) + 0x18, 0x00000001 }, { (base) + 0x1c, 0x0000309b },	\
	{ (base) + 0x20, 0x0000225c }, { (base) + 0x24, (v24) },	\
	{ (base) + 0x28, (v28) }, { (base) + 0x2c, (v2c) },	\
	{ (base) + 0x30, (v30) }, { (base) + 0x34, 0x0000121e },	\
	{ (base) + 0x38, (v38) }, { (base) + 0x3c, (v3c) },	\
	{ (base) + 0x40, (v40) }, { (base) + 0x44, 0x00001012 },	\
	{ (base) + 0x48, (v48) }, { (base) + 0x4c, (v4c) },	\
	{ (base) + 0x50, (v50) }, { (base) + 0x54, 0x0000f000 }

/*
 * One FIB (fiber optical front-end) bank: the same 17 { off, val } pairs at
 * 4-byte-register granularity from `base` -- the four banks in the table are
 * value-identical, so the program is written once here.  FIB_REG0 (+0x00)
 * still carries FP_CFG_FIB_PDOWN (bit11) SET at this point; the follow-up
 * loop over fib_reg0_banks[] clears it to power the fiber front-end on.
 */
#define SDS_FIB_BANK(base)							\
	{ (base) + 0x00, 0x00001940 }, { (base) + 0x04, 0x00006109 },	\
	{ (base) + 0x08, 0x0000e001 }, { (base) + 0x0c, 0x00003290 },	\
	{ (base) + 0x10, 0x000001a0 }, { (base) + 0x1c, 0x00000004 },	\
	{ (base) + 0x3c, 0x00008000 }, { (base) + 0x40, 0x00000083 },	\
	{ (base) + 0x48, 0x00005000 }, { (base) + 0x58, 0x00000001 },	\
	{ (base) + 0x5c, 0x00004001 }, { (base) + 0x60, 0x00000004 },	\
	{ (base) + 0x64, 0x0000326a }, { (base) + 0x6c, 0x0000115d },	\
	{ (base) + 0x70, 0x000033fa }, { (base) + 0x74, 0x0000e46a },	\
	{ (base) + 0x78, 0x0000071e }

/*
 * Full SerDes analog + WSDS configuration -- the operating point an ONU runs at
 * O5. The analog block (CMU, CDR, RX front-end including the optical
 * signal-detect comparator, TX driver) has dozens of per-silicon calibration
 * registers, and programming only the handful that obviously differ from reset
 * leaves other parts of the RX/SD path un-powered, so the optical signal-detect
 * never asserts even with real downstream light. Status/monitor registers and the
 * digital reset-B/clock bank are deliberately excluded; the ordered sequence in
 * gpon_serdes_init() drives those.
 */
static const struct { u32 off; u32 val; } sds_analog_golden[] __initconst = {
	/* WSDS analog front + digital RX-path config */
	{ 0x22000, 0x00000805 }, { 0x22008, 0x0000ffff }, { 0x2201c, 0x0000ffff },
	{ 0x22020, 0x0000ffff }, { 0x22038, 0x00000900 }, { 0x22048, 0x000000ff },
	{ 0x22050, 0x00022300 }, { 0x22054, 0x00022310 }, { 0x22058, 0x083d0100 },
	{ 0x22060, 0x00000fff }, { 0x22064, 0x0000cf45 }, { 0x22068, 0x00000f45 },
	/* SDS_ANA_MISC (RX-enable force, speed-select, force-SD) */
	{ 0x22500, 0x00000030 }, { 0x22504, 0x00000030 }, { 0x22508, 0x00003000 },
	/* SDS_ANA_COM (CMU, RX CDR front-end, filters, bias) */
	{ 0x22580, 0x00003400 }, { 0x22584, 0x000073a4 }, { 0x22588, 0x00006df8 },
	{ 0x2258c, 0x00008941 }, { 0x22590, 0x00008884 }, { 0x22594, 0x0000413f },
	{ 0x22598, 0x00004fc0 }, { 0x2259c, 0x00005682 }, { 0x225a0, 0x00000713 },
	{ 0x225a4, 0x000002f5 }, { 0x225a8, 0x00002793 }, { 0x225ac, 0x0000b000 },
	{ 0x225b0, 0x00004848 }, { 0x225b4, 0x000000c8 }, { 0x225bc, 0x000008f2 },
	{ 0x225c0, 0x00001042 }, { 0x225c4, 0x0000c391 }, { 0x225c8, 0x00006a00 },
	{ 0x225cc, 0x00006600 }, { 0x225d0, 0x0000c000 },
	/* 0x225d8 (COM_REG22 TX_AMP/EMP) is set later, in gpon_serdes_init's TX
	 * section, to the rev-A (ModeV1) value 0x29 (TX_AMP=0x5, TX_EMP=0x1) via
	 * field-writes. NOT a full write here so the upper bits keep their reset
	 * state. (Boot default 0x39/TX_AMP=0x7 over-drives.) */
	{ 0x225dc, 0x00000418 }, { 0x225e0, 0x00008001 }, { 0x225e4, 0x0000001f },
	{ 0x225e8, 0x000011e4 }, { 0x225ec, 0x00009422 }, { 0x225f0, 0x00008502 },
	{ 0x225f4, 0x00000ff0 }, { 0x225f8, 0x0000000a },
	/* SDS_ANA_GPON (GPON-rate CDR/PLL/PCM config) */
	SDS_ANA_RATE_BANK(0x22708,
		/* +0x24 */ 0x00001061, /* +0x28 */ 0x0000110d,
		/* +0x2c */ 0x00004854, /* +0x30 */ 0x000080c5,
		/* +0x38 */ 0x0000307b, /* +0x3c */ 0x00000271,
		/* +0x40 */ 0x00000271, /* +0x48 */ 0x0000f162,
		/* +0x4c */ 0x00003026, /* +0x50 */ 0x0000a780),
	/* SDS_ANA_GPON additional per-rate/lane banks (the RX path selects among
	 * these; leaving them at reset starves the active RX/SD analog).
	 * 0x22608 repeats the 0x22708 tuning exactly; 0x22688 and 0x22788 are
	 * the two variant tunings, and the argument labels above make the
	 * per-bank differences readable side by side. */
	SDS_ANA_RATE_BANK(0x22608,
		/* +0x24 */ 0x00001061, /* +0x28 */ 0x0000110d,
		/* +0x2c */ 0x00004854, /* +0x30 */ 0x000080c5,
		/* +0x38 */ 0x0000307b, /* +0x3c */ 0x00000271,
		/* +0x40 */ 0x00000271, /* +0x48 */ 0x0000f162,
		/* +0x4c */ 0x00003026, /* +0x50 */ 0x0000a780),
	SDS_ANA_RATE_BANK(0x22688,
		/* +0x24 */ 0x00001062, /* +0x28 */ 0x00002000,
		/* +0x2c */ 0x00001050, /* +0x30 */ 0x000080c1,
		/* +0x38 */ 0x0000107b, /* +0x3c */ 0x00000280,
		/* +0x40 */ 0x00000280, /* +0x48 */ 0x0000f862,
		/* +0x4c */ 0x00003938, /* +0x50 */ 0x00003100),
	SDS_ANA_RATE_BANK(0x22788,
		/* +0x24 */ 0x00001062, /* +0x28 */ 0x00002000,
		/* +0x2c */ 0x00004850, /* +0x30 */ 0x000080c5,
		/* +0x38 */ 0x0000103e, /* +0x3c */ 0x00000280,
		/* +0x40 */ 0x00000280, /* +0x48 */ 0x0000f862,
		/* +0x4c */ 0x00003938, /* +0x50 */ 0x0000b100),
	/* FIB (fiber optical front-end) config — 4 identical banks (one macro,
	 * see SDS_FIB_BANK above). This block powers and configures the optical
	 * RX/SD path; leaving it at reset keeps the optical front-end down so
	 * the signal-detect never asserts. FIB_REG0 (bank base) carries
	 * FP_CFG_FIB_PDOWN at bit11, cleared separately below to turn fiber
	 * power on. */
	SDS_FIB_BANK(0x22c00),
	SDS_FIB_BANK(0x22c80),
	SDS_FIB_BANK(0x22d00),
	SDS_FIB_BANK(0x22d80),
};

/* FIB_REG0 bank bases; FP_CFG_FIB_PDOWN (bit11) cleared = fiber power on. */
#define FIB_REG0_PDOWN		BIT(11)
static const u32 fib_reg0_banks[] __initconst = {
	0x22c00, 0x22c80, 0x22d00, 0x22d80,
};

/*
 * Bring up the PON SerDes so the GPON MAC core gets its line clock and the
 * receiver recovers the downstream bitstream.
 *
 * Ordering is the whole game. A working bring-up programs the analog CMU/CDR
 * block FIRST, selects GPON mode, THEN pulses the SDS+MAC reset, and only after
 * that releases the per-datapath soft-reset-B lines and forces the 125M
 * reference: the reset latches the freshly-written analog config, and releasing
 * the reset-B lines afterwards lets the RX CDR re-lock against it. A naive
 * reset-then-configure sequence ends with the same final register values and a
 * CDR that never locks -- identical register contents, loss-of-frame, FSM stuck
 * at O1. The operational run state is WSDS_DIG_00 = 0xf30 and
 * WSDS_DIG_1D = 0x1c000.
 */
static int gpon_serdes_init(void)	/* not __init: re-run on re-range from gpon_cdr_reset_worker */
{
	int i;

	/* 1. Park CFG_SDS_MODE at the illegal/off value (0x1f) while the analog
	 *    block is programmed and reset. The SDS must stay in the illegal mode for
	 *    the WHOLE bring-up and only switch to GPON (0x08) at the very end
	 *    (step 7). Selecting GPON before the RX is armed is exactly why a
	 *    register-identical naive sequence never locks the downstream. */
	sw_field(SDS_CFG, 4, 0, SDS_MODE_OFF);
	sw_wr(WSDS_DIG_01, 0);				/* clear force-SDS dummy   */
	sw_field(WSDS_DIG_00, 0, 0, 0);			/* STOP_CLK = 0            */

	/* 2. Program the FULL analog block to the operational values (the complete
	 *    RX/SD/CDR/CMU/TX config) and turn fiber power on (clear FP_CFG_FIB_PDOWN
	 *    on every FIB bank so the optical front-end + signal-detect power up). */
	/* Through SDS(), like every named register in this function. Both tables hold
	 * 9602C offsets and were written RAW while the single registers around them
	 * translate per chip. On the RTL9602C SDS() is the IDENTITY, so the omission was
	 * invisible on the board this code was written for.
	 * On the RTL9603CVD sds_win is 0x40000, so every table write landed 0x1E000 LOW,
	 * in dead space. Measured 2026-09-01 on the G24W three ways: 0x1b022000 and
	 * 0x1b022c00 read 0x0, the real window at 0x1b040000 holds values no table
	 * wrote, and all four FIB banks still read FIB_REG0_PDOWN SET -- the optical
	 * front-end never powered up, which is optic_los=1 sdet=0 while stock sits in O5
	 * on the same fibre minutes apart. */
	for (i = 0; i < ARRAY_SIZE(sds_analog_golden); i++)
		sw_wr(SDS(sds_analog_golden[i].off), sds_analog_golden[i].val);
	for (i = 0; i < ARRAY_SIZE(fib_reg0_banks); i++)
		sw_wr(SDS(fib_reg0_banks[i]),
		      sw_rd(SDS(fib_reg0_banks[i])) & ~FIB_REG0_PDOWN);

	/* 3. Pulse the SDS reset to latch the analog config. Stock rev-A pulses ONLY
	 *    bit0 (CMD_SDS_RST_PS); the extra bit7 (CMD_SDS_CFG_RST_PS) is never written
	 *    by stock and, left RMW-latched through bring-up, re-rolls
	 *    the US-TX serializer phase per power-on. Gate bit7 behind serdes_sds_cfgrst
	 *    (default 0 = stock bit0-only = the cold-start fix). */
	if (serdes_sds_cfgrst)
		sw_field(SW_SOFTWARE_RST, 7, 7, 1);	/* legacy CMD_SDS_CFG_RST_PS */
	sw_field(SW_SOFTWARE_RST, 0, 0, 1);		/* CMD_SDS_RST_PS          */
	mdelay(10);

	/* 4. Release all datapath soft-reset-B lines and force the 125M ref clock
	 *    (golden WSDS_DIG_00 = 0xf30), then pulse the RX/TX interface reset-B
	 *    lines (golden WSDS_DIG_1D = 0x1c000). Re-clear FIB power-down, which the
	 *    reset re-asserts. */
	sw_wr(WSDS_DIG_00, WSDS_DIG00_RUN);
	sw_field(WSDS_DIG_1D, 15, 15, 0);		/* RX interface reset-B 0  */
	sw_field(WSDS_DIG_1D, 16, 16, 0);		/* TX interface reset-B 0  */
	sw_field(WSDS_DIG_1D, 14, 14, 1);		/* common interface rst-B  */
	sw_field(WSDS_DIG_1D, 15, 15, 1);		/* RX interface reset-B 1  */
	sw_field(WSDS_DIG_1D, 16, 16, 1);		/* TX interface reset-B 1  */
	mdelay(10);
	/* ★ AND THIS RE-CLEAR TOO -- it is the one that UNDOES what the SDS reset
	 * just re-asserted, so leaving it raw kept the optical front-end powered
	 * DOWN even after step 2 was fixed.  MEASURED: with only the first two
	 * sites translated, 0x1b040000 became stock's 0x807 (the analog table now
	 * lands) while the FIB banks still read PDOWN set. */
	for (i = 0; i < ARRAY_SIZE(fib_reg0_banks); i++)
		sw_wr(SDS(fib_reg0_banks[i]),
		      sw_rd(SDS(fib_reg0_banks[i])) & ~FIB_REG0_PDOWN);

	/* 5. Burst-enable output; leave optical-LOS un-forced so the real RX front-
	 *    end drives it (a working unit reaches O5 with FRC_OPTIC_LOS=0). */
	sw_field(WSDS_DIG_18, 12, 12, 1);		/* BEN_OE = 1              */
	sw_field(WSDS_DIG_18, 15, 15, 0);		/* OPTIC_LOS_SEL_EPON = 0  */
	/* Do NOT force optic_los. At O5 WSDS_DIG_18 = 0x1000 (no force) and the REAL
	 * optical signal-detect asserts — because the external RTL8290B BOSA is
	 * initialised over I2C. The SD here is driven by that real RX path (see the
	 * BOSA init above); forcing optic_los only masks a down RX and never reaches
	 * O5. */
	sw_field(WSDS_DIG_18, 14, 14, 0);		/* CFG_FRC_OPTIC_LOS = 0   */
	sw_field(WSDS_DIG_18, 13, 13, 0);		/* CFG_FRCV_OPTIC_LOS = 0  */

	/* 6. Arm the RX in the required order: enable the RX-CDR analog front
	 *    end, settle, force the line-rate select to the GPON rate, then drive the
	 *    forced RX-enable through a 0->1 edge to start the CDR. Only AFTER the
	 *    analog config + reset + reset-B release does this 0->1 edge actually
	 *    kick the receiver. Finish with EN_PDOWN_BEN=0 and TX-disable delay=0. */
	sw_field(SDS_ANA_COM_REG12, 14, 14, 0x1);	/* RX_SEL_CDR_AFEN = 1     */
	mdelay(10);
	sw_field(SDS_ANA_MISC_REG01, 7, 5, 0x1);	/* SPDSEL_VAL = GPON rate  */
	sw_field(SDS_ANA_MISC_REG01, 4, 4, 0x1);	/* SPDSEL force on         */
	sw_field(SDS_ANA_MISC_REG00, 4, 4, 0x1);	/* FRC_RX_EN_ON = 1        */
	sw_field(SDS_ANA_MISC_REG00, 5, 5, 0x0);	/* FRC_RX_EN_VAL 0 ...     */
	sw_field(SDS_ANA_MISC_REG00, 5, 5, 0x1);	/* ... -> 1 (start CDR)    */
	mdelay(50);
	sw_field(WSDS_DIG_02, 10, 10, 0x0);		/* EN_PDOWN_BEN = 0        */
	sw_field(WSDS_DIG_03, 6, 4, 0x0);		/* CFG_TXDIS_SEL_DLY = 0: the RTL9602C
							 * burst-mode TX-disable timing requires
							 * 0; 0x2 mis-times the burst TX-disable
							 * -> "Laser out". */
	sw_field(WSDS_DIG_03, 3, 0, 0x0);		/* CFG_D2ANLOG_SEL = 0 (TX data path) */
	/* FORCE_BEN (SDS 0x220e4) BEN_FORCE_MODE[0]=0: let the GTC framer drive the
	 * burst-enable (laser gate). If left at the forced default the laser is gated
	 * by BEN_FORCE_VALUE (off) and never fires the SN burst even though the MAC
	 * queue drains -> OLT "Power down". (Was wrongly written to 0x400e4, which is
	 * an unmapped address that bus-aborts — verified via /proc sds_tx readback.) */
	sw_field(SDS(0x220e4), 0, 0, 0x0);

	/*
	 * 6a-ModeV1. US-TX SerDes CMU/PLL and TX-LA-LDO -- the rev-A GPON mode-set
	 * analog full-word writes our init OMITTED. COM_REG02/03/08 program the TX
	 * CMU/PLL that clocks the upstream serializer, and COM_REG24 powers the TX
	 * limiting-amp output stage feeding the laser modulation input. Without them the
	 * laser is DC-biased but the MAC's US data is not cleanly serialized onto it, so
	 * the OLT's burst-RX sees no decodable burst. Placed before the D2A interconnect
	 * to match ModeV1 order. The CMU is shared with the RX 25M reference, so watch
	 * DS and ranging.
	 */
	if (serdes_modev1_tx) {
		sw_wr(SDS(0x22588), 0x6df8);		/* SDS_ANA_COM_REG02 TX CMU/PLL */
		sw_wr(SDS_ANA_COM_REG03, 0x8941);	/* SDS_ANA_COM_REG03 TX CMU (0x2258c) */
		sw_wr(SDS_ANA_COM_REG08, 0x0713);		/* SDS_ANA_COM_REG08 */
		sw_wr(SDS(0x225e4), 0x001f);		/* SDS_ANA_COM_REG25 */
		sw_wr(SDS(0x225e0), 0x8001);		/* SDS_ANA_COM_REG24 REG_TXLA_LDOEN */
		pr_info("luna-gpon: ModeV1 TX SerDes applied: COM_REG02=0x%04x 03=0x%04x 08=0x%04x 24=0x%04x 25=0x%04x\n",
			sw_rd(SDS(0x22588)) & 0xffff, sw_rd(SDS_ANA_COM_REG03) & 0xffff,
			sw_rd(SDS_ANA_COM_REG08) & 0xffff, sw_rd(SDS(0x225e0)) & 0xffff, sw_rd(SDS(0x225e4)) & 0xffff);
	}

	/*
	 * 6b. TX DATA PATH -- route the digital US-framer data into the analog TX
	 * serializer and set the TX-data sample-clock edges, BEFORE switching
	 * CFG_SDS_MODE to GPON, so the serializer latches the CONNECTED data-path mux
	 * and clocks at mode-entry. Setting them AFTER GPON mode (with a TX reset-B
	 * toggle to compensate) left the already-running serializer latched on the
	 * disconnected pre-config: the laser was DC-biased and carried no decodable
	 * burst, so the OLT received zero upstream.
	 */
	/* ORACLE-PARITY 2026-06-13: the LIVE stock-ref ONU (ttyUSB3 mmiord @O5) does NOT set
	 * these three serializer-path bits — stock reads 0x220a8=0x2 ([5:4]=0), 0x2281c=0x1359
	 * ([14]=0), 0x22a30=0x4 ([8]=0). Our build was setting them (0x32/0x5359/0x104), which is
	 * the only confirmed mine-vs-live-stock divergence in the SerDes-TX block (0x2280c/0x225d8
	 * already match). The old comment claimed they were required for a decodable burst, but the
	 * working stock is the existence proof: it ranges + bursts WITHOUT them. Match stock. The
	 * `serdes_tx_xtra` param restores the old behaviour for A/B if this regresses ranging. */
	sw_field(SDS(0x220a8), 5, 4, serdes_tx_xtra ? 0x3 : 0x0);	/* WSDS_DIG_1E D2A interconnect (stock=0) */
	sw_field(SDS(0x2281c), 14, 14, serdes_tx_xtra ? 0x1 : 0x0);	/* SDS_REG7 SP_CFG_NEG_CLKWR_A2D (stock=0) */
	sw_field(SDS(0x22a30), 8, 8, serdes_tx_xtra ? 0x1 : 0x0);	/* SDS_EXT_REG12 SEP_CFG_NEG_CLKRD_D2A (stock=0) */

	/*
	 * TX drive level: REG_TX_AMP=0x5, REG_TX_EMP=0x1, the rev-A drive this board
	 * requires. The SoC boot default (TX_AMP=0x7) over-drives the serializer output
	 * feeding the laser modulation input and distorts the upstream burst eye, so the
	 * OLT's burst receiver cannot reliably decode our SN/ranging burst --
	 * detect-but-no-range.
	 */
	sw_field(SDS(0x225d8), 5, 3, 0x5);			/* SDS_ANA_COM_REG22 REG_TX_AMP = 0x5 */
	sw_field(SDS(0x225d8), 2, 0, 0x1);			/* SDS_ANA_COM_REG22 REG_TX_EMP = 0x1 */

	/*
	 * 7a. Force signal-detect on. RST_DONE is gated by signal-detect; force it so
	 * the MAC reset handshake completes (MISC_REG02 = 0x3000). This
	 * only ungates the handshake — real downstream lock still shows as LOF
	 * clearing and superframe_cnt incrementing, which force-SD does NOT fake.
	 */
	sw_field(SDS_ANA_MISC_REG02, 13, 13, 0x1);	/* signal-detect value=1   */
	sw_field(SDS_ANA_MISC_REG02, 12, 12, 0x1);	/* force signal-detect     */
	mdelay(10);

	/* 7b. Finally select GPON mode — the very last step, with the RX fully armed
	 *     (CFG_SDS_MODE switches to GPON only here). */
	sw_field(SDS_CFG, 4, 0, SDS_MODE_GPON);
	mdelay(50);

	/* TX-interface reset-B re-sync: with the D2A mux + sample clocks already set
	 * before mode-entry (step 6b), pulse WSDS_DIG_1D[16] (CFG_SFT_RSTB_INF_TX) 0->1
	 * once GPON mode is live so the TX serializer (re)locks onto the now-connected
	 * framer data. The SerDes-TX serializer lock is non-deterministic; this re-sync
	 * is the mechanism that historically caught the upstream-burst lock (the OLT
	 * occasionally ranged the ONU). Only the TX interface reset-B is toggled, not
	 * the PLL, so the locked RX downstream framer is undisturbed. */
	sw_field(WSDS_DIG_1D, 16, 16, 0);
	mdelay(2);
	sw_field(WSDS_DIG_1D, 16, 16, 1);
	mdelay(10);

	/* SerDes CDR-lock pulse -- the stock CDR-reset our init OMITTED: invert
	 * SDS_ANA_COM_REG12 bit15 (REG_RX_SD_POR_SEL), hold 10 ms, restore, which
	 * re-PORs the RX signal-detect path so a recovered CDR re-acquires cleanly. Done
	 * here, after GPON mode and the TX reset-B re-sync, so the analog-ready poll
	 * below confirms the SerDes recovered.
	 * It is REG12 and not REG08 (fixed 2026-06-17): REG12[15] is REG_RX_SD_POR_SEL
	 * while REG08[15] falls in the reserved top-16 field, so the prior toggle wrote
	 * a reserved bit. The COM_REG08=0x0713 write in the stock trace is the ModeV1
	 * config constant, not this toggle. */
	if (serdes_cdr_reset) {
		u32 cdr = sw_rd(SDS_ANA_COM_REG12);

		sw_wr(SDS_ANA_COM_REG12, cdr ^ BIT(15));
		mdelay(10);
		sw_wr(SDS_ANA_COM_REG12, cdr);
		/* ★★ PRINT THE ADDRESS THAT WAS WRITTEN, NOT THE ONE THE 9602C
		 * HAPPENS TO USE.  This message carried the LITERAL 0x225b0 while
		 * the code writes SDS(0x225b0), which is the same number on the
		 * RTL9602C and 0x1e000 higher on the other two Luna chips.  A log
		 * line that names an address the code did not touch is the
		 * misleading-name defect this tree renames on sight -- and it is
		 * worse in a log, because a reader has no declaration to check it
		 * against. */
		pr_info("luna-gpon: serdesCdr_reset pulse (COM_REG12 @ 0x%05x bit15), restored=0x%08x\n",
			SDS_ANA_COM_REG12, cdr);
	}

	sw_field(WSDS_DIG_00, 0, 0, 0);			/* keep MAC clock ungated  */

	pr_info("luna-gpon: SDS cfg=0x%08x dig00=0x%08x dig1d=0x%08x fib21=0x%08x fib_reg0=0x%08x\n",
		sw_rd(SDS_CFG), sw_rd(WSDS_DIG_00), sw_rd(WSDS_DIG_1D),
		sw_rd(FIB_EXT_REG21), sw_rd(SDS(0x22c00)));

	for (i = 0; i < SDS_LOCK_POLL_MAX; i++) {
		if (sw_rd(FIB_EXT_REG21) & SDS_ANALOG_READY)
			return 0;
		udelay(200);
	}
	return -ETIMEDOUT;
}

/*
 * Stock rev-A GPON SerDes bring-up ORDER as observed from the stock device, with
 * OUR golden analog overlay applied in that order. Verified 2026-06-16 against
 * the stock GPON mode-set sequence and the chip's register and field maps.
 *
 * SCOPE: this is NOT a bit-faithful replica of stock's ANALOG config. Stock has
 * no golden table and no fib_reg0 loop -- its ModeV1 analog sub-step IS its
 * entire analog config. The golden table and fib power-on here are OUR overlay,
 * applied in stock ORDER, and the ModeV1 explicit writes overwrite every
 * overlapping golden entry, so the final operating point is the ModeV1 one.
 * What IS faithful is the ORDER (reset first), the single reset bit
 * (CMD_SDS_RST_PS only) and the D2A/sample-clock bits being SET -- the value
 * stock programs, though the LIVE stock ONU reads them clear at O5, a real
 * programmed-vs-runtime divergence worth an A/B.
 *
 * __init because it reads two __initconst tables: outside .init.text modpost
 * reports a section mismatch and it would read freed memory. Post-init
 * re-runnability would need both tables de-__initconst'd.
 */
static int __init gpon_serdes_init_stock(void)
{
	/* offsets absent from our #define block - all confirmed via the chip register map */
	const u32 WSDS_DIG_1Eo         = 0x220a8;
	const u32 SDS_REG7o            = 0x2281c;
	const u32 SDS_EXT_REG12o       = 0x22a30;
	const u32 SDS_ANA_COM_REG02o   = 0x22588;
	const u32 SDS_ANA_COM_REG19o   = 0x225cc;
	const u32 SDS_ANA_COM_REG24o   = 0x225e0;
	const u32 SDS_ANA_COM_REG25o   = 0x225e4;
	const u32 SDS_ANA_1P25G_REG46o = 0x226b8;
	const u32 SDS_ANA_EPON_REG46o  = 0x227b8;
	int i;

	/* === stock step 1: serdes mode -> illegal (0x1f) === */
	sw_field(SDS_CFG, 4, 0, SDS_MODE_OFF);		/* CFG_SDS_MODE = 0x1f     */

	/* === stock step 2: no force sds === */
	sw_wr(WSDS_DIG_01, 0);				/* WSDS_DIG_01 = 0         */

	/* === stock step 3: RESET FIRST - pulse ONLY CMD_SDS_RST_PS (bit0).
	 * Stock does NOT touch CMD_SDS_CFG_RST_PS (bit7) here. === */
	sw_field(SW_SOFTWARE_RST, 0, 0, 1);		/* CMD_SDS_RST_PS = 1      */
	mdelay(10);					/* 10ms settle             */

	/* === stock step 4: BEN on === */
	sw_field(WSDS_DIG_18, 12, 12, 1);		/* BEN_OE = 1              */

	/* === stock step 5: rev-A else-branch - adjust
	 * TX_Burst's Burst Mode Sequence === */
	sw_field(WSDS_DIG_03, 6, 4, 0x0);		/* CFG_TXDIS_SEL_DLY = 0   */
	sw_field(WSDS_DIG_03, 3, 0, 0x0);		/* CFG_D2ANLOG_SEL   = 0   */

	/* ----------------------------------------------------------------------
	 * OUR golden analog overlay + fiber power-on, applied AFTER the reset
	 * (stock ORDER). NOT part of stock; the ModeV1 writes below override every
	 * overlapping entry. Provides the per-rate/lane + FIB bank config that
	 * stock assumes is already present from an earlier mode-set.
	 * -------------------------------------------------------------------- */
	/* Through SDS(), like every named register in this function. Both tables hold
	 * 9602C offsets and were written RAW while the single registers around them
	 * translate per chip. On the RTL9602C SDS() is the IDENTITY, so the omission was
	 * invisible on the board this code was written for.
	 * On the RTL9603CVD sds_win is 0x40000, so every table write landed 0x1E000 LOW,
	 * in dead space. Measured 2026-09-01 on the G24W three ways: 0x1b022000 and
	 * 0x1b022c00 read 0x0, the real window at 0x1b040000 holds values no table
	 * wrote, and all four FIB banks still read FIB_REG0_PDOWN SET -- the optical
	 * front-end never powered up, which is optic_los=1 sdet=0 while stock sits in O5
	 * on the same fibre minutes apart. */
	for (i = 0; i < ARRAY_SIZE(sds_analog_golden); i++)
		sw_wr(SDS(sds_analog_golden[i].off), sds_analog_golden[i].val);
	for (i = 0; i < ARRAY_SIZE(fib_reg0_banks); i++)
		sw_wr(SDS(fib_reg0_banks[i]),
		      sw_rd(SDS(fib_reg0_banks[i])) & ~FIB_REG0_PDOWN);	/* fiber on */

	/* ======================================================================
	 * stock step 6: the inlined rev-A bring-up analog config.
	 * ==================================================================== */

	/* --- ### TX ### PON TX CMU/PLL + TX-LA-LDO --- */
	sw_wr(SDS_ANA_COM_REG02o, 0x6df8);	/* COM_REG02 (CMU/LDO/ISTANK) */
	sw_wr(SDS_ANA_COM_REG03, 0x8941);	/* COM_REG03 (0x2258c)        */
	sw_wr(SDS_ANA_COM_REG08, 0x0713);	/* COM_REG08 (0x225a0)        */
	sw_wr(SDS_ANA_COM_REG25o, 0x1f);	/* COM_REG25                  */

	/* WSDS_DIG_1E[5]=CFG_ANALOG2D_SEL, [4]=CFG_D2ANLOG_INF_SEL - stock SETs
	 * these =1 (opposite of our oracle-parity path which clears them). */
	sw_field(WSDS_DIG_1Eo, 5, 5, 0x1);	/* CFG_ANALOG2D_SEL = 1       */
	sw_field(WSDS_DIG_1Eo, 4, 4, 0x1);	/* CFG_D2ANLOG_INF_SEL = 1    */

	sw_wr(SDS_ANA_COM_REG24o, 0x8001);	/* COM_REG24 (REG_TXLA_LDOEN) */

	/* --- ### RX ### (ASIC, !FPGA) --- */
	sw_wr(SDS_ANA_1P25G_REG46o, 0x80c5);	/* FIB1G Rx 1.25 KP/KI        */
	sw_wr(SDS_ANA_EPON_REG46o,  0x80c5);	/* EPON  Rx 1.25 KP/KI        */
	sw_wr(SDS_ANA_GPON_REG46,   0x80c5);	/* GPON  Rx 2.488 KP/KI       */

	sw_field(SDS_ANA_COM_REG12, 14, 14, 0x1);	/* REG_RX_SEL_CDR_AFEN = 1   */
	sw_field(SDS_ANA_COM_REG11,  7,  0, 0x0);	/* REG_RX_FILT_CONFIG = 0    */
	sw_field(SDS_ANA_COM_REG19o, 14, 14, 0x1);	/* REG_CDR_RESET_MANUAL = 1  */
	sw_field(SDS_ANA_COM_REG19o, 10, 10, 0x1);	/* REG_CDR_EN_LPF_MANUAL = 1 */

	/* ### RX_EN toggle ### MISC_REG00: FRC_RX_EN_ON[4], FRC_RX_EN_VAL[5] */
	sw_wr(SDS_ANA_MISC_REG00, 0x10);	/* RX_EN force on, val=0       */
	sw_wr(SDS_ANA_MISC_REG00, 0x30);	/* RX_EN val 0->1 (start CDR)  */

	mdelay(50);				/* 50ms settle                 */

	sw_field(WSDS_DIG_02, 10, 10, 0x0);	/* REG_EN_PDOWN_BEN = 0        */

	/* TX-data sample clocks - stock SETs these =1. */
	sw_field(SDS_REG7o,      14, 14, 0x1);	/* SP_CFG_NEG_CLKWR_A2D = 1   */
	sw_field(SDS_EXT_REG12o,  8,  8, 0x1);	/* SEP_CFG_NEG_CLKRD_D2A = 1  */
	/* === end ModeV1 analog config === */

	/* === stock step 7: serdes mode -> GPON (0x8) === */
	sw_field(SDS_CFG, 4, 0, SDS_MODE_GPON);		/* CFG_SDS_MODE = 0x8      */

	/* === stock step 8 (!FPGA): force ber notify [13:12] = 0x3 === */
	sw_field(SDS_ANA_MISC_REG02, 13, 13, 0x1);	/* FRC_BER_NOTIFY_VAL = 1  */
	sw_field(SDS_ANA_MISC_REG02, 12, 12, 0x1);	/* FRC_BER_NOTIFY_ON  = 1  */

	pr_info("luna-gpon: stock rev-A SerDes init done: SDS_CFG=0x%08x DIG18=0x%08x MISC02=0x%08x fib21=0x%08x\n",
		sw_rd(SDS_CFG), sw_rd(WSDS_DIG_18), sw_rd(SDS_ANA_MISC_REG02),
		sw_rd(FIB_EXT_REG21));

	/* Local readiness gate (not part of the stock sequence) - mirrors gpon_serdes_init tail. */
	for (i = 0; i < SDS_LOCK_POLL_MAX; i++) {
		if (sw_rd(FIB_EXT_REG21) & SDS_ANALOG_READY)
			return 0;
		udelay(200);
	}
	return -ETIMEDOUT;
}

/*
 * Wait (bounded) for the GPON MAC to report RST_DONE after the SerDes sequence
 * has issued the SDS+MAC reset. Returns 0 on RST_DONE, -ETIMEDOUT otherwise.
 */
static int __init gpon_wait_rst_done(void)
{
	int i;

	for (i = 0; i < GPON_RST_POLL_MAX; i++) {
		if (gpon_rd(GPON_RESET) & GPON_RST_DONE)
			return 0;
		udelay(10);
	}
	return -ETIMEDOUT;
}

/*
 * Faithful port of the stock all-module GPON datapath bring-up, in stock MODULE
 * ORDER, run ONCE on a QUIESCENT switch (after the GMAC IP-block reset and
 * U-Boot swcore resync, before the datapath is armed). It reproduces the
 * ordering and table-clears the prior piecemeal writes did not.
 * Gated by full_datapath_init; the indirect TBL_ACCESS table-clears self-abort if
 * the engine does not respond, and VLAN filtering is only enabled if its member
 * table actually wrote.
 */
static unsigned int full_datapath_init = 1;
module_param(full_datapath_init, uint, 0644);
MODULE_PARM_DESC(full_datapath_init, "1=run the full datapath init on the quiescent switch (default), 0=legacy minimal init");
static unsigned int table_engine_ok;	/* 0: the operational VLAN setup (our Ethernet driver) owns the VLAN table now; isolate the VLAN_CTRL=0x19 enable test */
module_param(table_engine_ok, uint, 0644);
MODULE_PARM_DESC(table_engine_ok, "1=also run the L2 LUT clear + VLAN-enable table-engine steps (risky), 0=field-writes only");
/* "harmful write" hypothesis: stock NEVER writes the US-NIC
 * region (0x1bf04xxx) or these PON-IP speculative regs from the CPU; my gpon_pbo_init
 * accumulated ~15 such writes (MEDIA_STS_US, MOCIR force, credit cluster, internal-link
 * force) across earlier rounds. One may BREAK the US-NIC ingress that would otherwise
 * accept the cpu-tag frame. usnic_strip=1 SKIPS them all (stock-faithful pbo only);
 * =0 restores the prior behavior. Default 1 for the strip-and-retest. */
static unsigned int usnic_strip;	/* 0 = KEEP the US-NIC writes: live-stock same-board diff shows stock HAS
					 * MOCIR 0x2170/0x2174=0x1ffff etc. set, so stripping them DIVERGED from
					 * stock (the "stock never writes these" hypothesis was wrong; live = truth). */
module_param(usnic_strip, uint, 0644);
MODULE_PARM_DESC(usnic_strip, "1=skip the speculative non-stock US-NIC writes (strip test), 0=keep them (match stock)");
/* Faithful port of the stock PON-MAC GPON mode-set branch as ONE ordered
 * block at O5. Root cause: the US-NIC RX engine latches its
 * SID-classify table at the GMII_RX_EN rising edge; my driver sets the classify at
 * O5 but never RE-PULSES that edge, so the US-NIC keeps a stale boot-time latch and
 * discards SID-64 OMCI pre-MAC. Default OFF (protect DS/LAN/ranging); bisectable. */
static unsigned int ponmac_modeset = 1;	/* MUST be 1: the classify block and GMII
					 * re-latch at O5 are what make the GEM-US engine latch
					 * the GEM_US_PORT_MAP write. Without it the engine latches
					 * at boot, before gpon_install_omcc writes the port map,
					 * and no OMCI data reaches the US GEM. */
module_param(ponmac_modeset, uint, 0644);
MODULE_PARM_DESC(ponmac_modeset, "1=stock-ordered GPON mode_set classify block + GMII re-latch at O5");

/* finding 3 (2026-06-13): stock fires the US-NIC GMII_RX_EN latch edge ONCE,
 * at O5, AFTER the COMPLETE OMCC provisioning (classify triple + the US scheduler flow2queue
 * binding). My driver fires it at boot/ifup (gpon_pbo_init) — BEFORE the scheduler (qid 64
 * PIR/CIR/WFQ/SCH_QMAP) is bound in gpon_install_tcont. The round-32 re-latch "had no effect"
 * but that predates the SID2QID packing fix, so it re-latched the WRONG classify slot. With
 * SID2QID[64] now at the true word 0x2138=64, re-latch the edge at the END of install_tcont so
 * the US-NIC RX engine latches the complete+correct config. Default ON for this bisection step. */
static unsigned int relatch_us = 1;
module_param(relatch_us, uint, 0644);
MODULE_PARM_DESC(relatch_us, "1=re-pulse US-NIC GMII_RX_EN edge after the scheduler binding (finding 3)");
static unsigned int serdes_recommit;	/* 0 = OFF: a full SDS re-commit (CMD_SDS_RST_PS) at O5 DROPS the locked DS framer (omcirx->0, DS dead) — DISPROVEN destructive. */
module_param(serdes_recommit, uint, 0644);
MODULE_PARM_DESC(serdes_recommit, "1=also re-commit SerDes inside the O5 mode_set block (risky; gate separately)");
/* PON_GEN_PIR_DROP (bit18 of PON_SCH_CTRL 0x2194). Default 0 = the rev-A value:
 * the part powers up with this bit set (1); on the GPON rev-A it is CLEARED to 0, per the
 * rule "rev-A must turn off PON_GEN_PIR_DROP, due to the tcont 16" — i.e. a rev-A
 * erratum on T-CONT 16 / qid 64 (exactly our failing T-CONT). The prior "keep it 1"
 * guard cited a live-stock 0x66000 read, but that read was the power-up default
 * BEFORE the rev-A clear, and the "clearing didn't help" test ran while the
 * GEM_US_PORT_MAP stride bug still masked it. Stride is now fixed → re-test with the
 * correct rev-A value. A/B via bootarg luna_gpon.pir_drop=1. */
static unsigned int pir_drop;
module_param(pir_drop, uint, 0644);
MODULE_PARM_DESC(pir_drop, "PON_GEN_PIR_DROP bit18@0x2194: 0=rev-A erratum clear (default, drains T-CONT16), 1=set");

/* sch_ctrl_stock: write PON_SCH_CTRL to the EXACT value measured on LIVE STOCK
 * (0x00066000) instead of only field-poking bit18 -- PON_GEN_PIR_DROP=1,
 * METER_OP=1, PON_WFQ_BURSTSIZE=0x6000. We set ONLY bit18 and left METER_OP and
 * WFQ_BURSTSIZE at reset, so the scheduler ran with a ZERO burst allowance and
 * the wrong meter mode, and every prior pir_drop A/B toggled bit18 alone. The
 * rev-A "must clear PIR_DROP" note is DISPROVEN by this live read. */
static bool sch_ctrl_stock = true;
module_param(sch_ctrl_stock, bool, 0644);
MODULE_PARM_DESC(sch_ctrl_stock, "1=write PON_SCH_CTRL 0x2194=0x66000 verbatim from live stock (PIR_DROP+METER_OP+WFQ_BURSTSIZE); 0=legacy bit18-only (default on)");
/* DPRU_RPT_PRD (0x2568): DBA_BLKSIZE=48 (byte->block divisor the HW uses to encode the
 * DBRu queued-occupancy report the OLT reads to size grants). Stock writes 0x3002 once at
 * init; ours had regressed this write out -> the OLT reads 0 queued despite qid64 holding
 * pages -> grants once then stops -> gemus_omcc=0. Default on; A/B via luna_gpon.dbru_blksize=0. */
static bool dbru_blksize = true;
module_param(dbru_blksize, bool, 0644);
MODULE_PARM_DESC(dbru_blksize, "1=write DPRU_RPT_PRD 0x2568=0x3002 (DBA_BLKSIZE=48, DBRu report divisor; default on)");
/* Re-issue SIDVALID after the T-CONT queue-add arm -- the post-arm re-write this
 * driver had omitted. Stock writes it at TWO points: once in the classify triple
 * at mode_set (KEEP that, it matches stock) and again at the END of queue_add,
 * after drain, T-CONT-enable, qmap and rates, re-committing the SID->armed-queue
 * binding. The load-bearing detail is the ORDERING, not an "over_sts occupancy
 * latch": over_sts is a near-full PBO watermark nothing in the scheduler reads,
 * so judge success by gemus_omcc climbing and the OLT resuming grants. */
static bool sidvalid_last = true;
module_param(sidvalid_last, bool, 0644);
MODULE_PARM_DESC(sidvalid_last, "1=re-issue SIDVALID[64] after the T-CONT-16 arm (stock queue_add tail does this; we had omitted it) (default on)");
/* Bind the OMCC Alloc-ID to a SECOND T-CONT as well. NON-STOCK: stock binds
 * exactly ONE T-CONT per Alloc-ID. With two alloc-CAM entries for one alloc, a
 * BWmap grant resolves to the EMPTY alternate T-CONT, so the DBRu reports its
 * occupancy as 0 while the real queue holds the pages -- the OLT reads 0, grants
 * once, stops, and deactivates. Default OFF. */
static bool omcc_alt_bind;
module_param(omcc_alt_bind, bool, 0644);
MODULE_PARM_DESC(omcc_alt_bind, "1=also bind the OMCC alloc to T-CONT 1 (non-stock double-bind that makes the DBRu report the empty T-CONT; default off = stock one-T-CONT-per-alloc)");
/* Re-assert AUTO_PROC_SSTART (US_PROC_MODE 0x5200 bit0) at O5. It gates the HW
 * auto-processing of each grant's SStart to START the US-TX burst. Written once at
 * init, but the ranging GMAC/SerDes reset (same 0x52xx US region we already re-arm at
 * O5) can clear it -> on an operational grant the HW never opens the US window -> the
 * GEM-US framer never fires (idle16=0, gemus_omcc=0, bank_underfl=0, dead air -> OLT
 * "Laser out"). Re-writing it at O5 entry closes that. Default on; A/B via
 * luna_gpon.o5_sstart=0. */
static bool o5_sstart = true;
module_param(o5_sstart, bool, 0644);
MODULE_PARM_DESC(o5_sstart, "1=re-assert AUTO_PROC_SSTART (0x5200 bit0) at O5 so the HW starts the US burst on each grant (default on)");
/* Re-arm the GEM-US US-feed run-state with the FULL WSDS GPON-datapath reset-B edge
 * (WSDS_DIG_00 0x22030 bit10) right after the O3 TX-PLL relock — the one edge the
 * O5-light re-arm omits. The relock (a SerDes reset) re-parks the feed; this un-parks
 * it. RISK: pulsing the WSDS GPON reset-B may drop the DS framer lock at O3. Default
 * OFF (try o5_sstart first); A/B via luna_gpon.o3_feed_reset=1. */
static bool o3_feed_reset;
module_param(o3_feed_reset, bool, 0644);
MODULE_PARM_DESC(o3_feed_reset, "1=pulse WSDS GPON datapath reset-B + light feed re-arm after the O3 TX-PLL relock to un-park the GEM-US framer (default off; risks DS lock)");
/* Per-tick US-feed re-arm at O5. DEFAULT OFF since 2026-07-04: this keeper was
 * the SUSTAINED-WAN-DATA killer. It was meant to self-terminate "the instant
 * gemus_omcc advances", but neither that counter nor the dram_used it keys on
 * ever advances on this datapath -- the real US data does not flow through the
 * SRAM->DRAM staging they track -- so the condition is permanently false and the
 * keeper fires ~60x/second, pulsing a GMII_RX_EN OFF->ON edge every ~16 ms that
 * TRUNCATES any in-flight US ingest. US TX then dies after a few minutes while DS
 * survives and O5 holds. A/B: feed_rekick=0 gave 231/231 DNS round-trips over
 * 228 s with zero DEACT churn against a baseline dying at ~100-156 s.
 * Do NOT default it on again without a self-terminate keyed on a RELIABLE
 * progress signal -- a per-tick GMII edge is toxic to sustained US either way. */
static bool feed_rekick;
module_param(feed_rekick, bool, 0644);
MODULE_PARM_DESC(feed_rekick, "1=per-tick US-feed FIFO re-arm at O5 (DEFAULT OFF: the per-tick GMII edge truncates in-flight US bursts + kills sustained WAN data; opt-in diagnostic only)");
/* DIAGNOSTIC bisection: force the GTC framer to emit idle-GEM on grant windows
 * (FS_GEM_IDLE 0x6020 bit31=1), independent of the page feed. If idle16 then climbs, the
 * framer IS receiving T-CONT16 grants (fault is US-feed starvation -> feed_rekick). If
 * idle16 STAYS 0, grants never reach the framer (scheduling/drain dead). Stock=0; test only. */
static bool force_idle;
module_param(force_idle, bool, 0644);
MODULE_PARM_DESC(force_idle, "1=set FS_GEM_IDLE (0x6020 bit31) to force idle-GEM on grants -- bisection diagnostic (default off, stock=0)");

/* us_intr_svc: service (read-to-clear) the upstream GPON interrupt DELTA latches
 * each O5 tick. The known-good unit is EVENT-DRIVEN and reads GTC_US_INTR_DLT on
 * every GTC_US interrupt, which acks the latched sub-events; our poll-only port
 * never reads it, so the delta latches FOREVER. Hypothesis: the descriptor-fetch
 * FSM back-pressures on the unacked sticky delta and never starts the
 * staging->framing transfer. Reading the raw delta clears it regardless of the
 * mask, so this is a safe no-IRQ service. Default off, so the first /proc read
 * shows the pre-service latched state -- the clean discriminator. */
static bool us_intr_svc;
module_param(us_intr_svc, bool, 0644);
MODULE_PARM_DESC(us_intr_svc, "1=read-to-clear the US GPON interrupt deltas (0x5000/0x5008/0x6000/0x6008) each O5 tick (default off)");

/* swcore TBL_ACCESS engine (L2/VLAN tables) */
#define TBL_BUSY_BIT	(1u << 13)
/* The bound, named the way LUNA_SMI_TRIES and L3FE_ACCESS_TRIES already are
 * in this tree, because it is now an ARGUMENT to the shared poll and not a
 * number buried in a loop.  Why 2000 and not 0x4000: see tbl_wait(). */
#define TBL_WAIT_TRIES	2000u

static bool tbl_ok = true;

/* The SWCORE table engine's pace for gpon_ind_poll().  1 us like the PON-IP
 * strobes, PLUS the cond_resched() tbl_wait()'s comment below is about.
 *
 * The core's @pause is a void(*)(void) with no iteration index, so the "every
 * 256th" stride that used to be per-CALL is counted in a file-static here.  The
 * protection is unchanged in substance -- a yield at least every 256 pauses,
 * and the watchdog is what it protects -- only the exact yield POINTS move, and
 * nothing on the bus depends on where they fall. */
static void tbl_pause(void)
{
	static unsigned int spins;

	udelay(1);
	if ((++spins & 0xff) == 0)
		cond_resched();
}

static void tbl_wait(void)
{
	/*
	 * The budget is 2 ms, not 16 ms, and it cost this board every boot (measured
	 * 2026-08-27). This polled 0x4000 times at udelay(1) = 16.4 ms per wait;
	 * tbl_write() waits TWICE and the table init runs the loop 512 times, so an
	 * engine that does not answer burns up to 16.8 s in a BUSY-WAIT. The boot log
	 * shows 6.8 s with nothing logged and then a full-chip reset -- the watchdog's
	 * 30 s window expiring because procd never got the CPU back to feed it.
	 * udelay() SPINS, so nothing else runs and no userspace feeder can survive it
	 * however wide its window; cond_resched() lets the scheduler in, and the caller
	 * is process context, never an ISR. A shorter budget loses nothing: this engine
	 * answers in microseconds when it answers at all.
	 *
	 * The loop itself belongs to the core (gpon_ind_poll(), over this file's
	 * &sw_io) -- it was the FOURTH hand-spelled copy of one idea.
	 * _poll and not _go, because TBL_STS_OFF is POLLED and TBL_CTRL_OFF is
	 * WRITTEN: two different registers, and gpon_ind_go() would emit the
	 * transaction word INTO the status register.
	 */
	if (gpon_ind_poll(&sw_io, TBL_STS_OFF, TBL_BUSY_BIT, TBL_WAIT_TRIES,
			  tbl_pause) < 0)
		tbl_ok = false;		/* engine wedged / wrong encoding: stop */
}

static void tbl_write(u32 type, u32 addr)
{
	u32 ctrl;

	if (!tbl_ok)
		return;
	tbl_wait();
	if (!tbl_ok)
		return;
	ctrl = sw_rd(TBL_CTRL_OFF);
	ctrl = (ctrl & ~(0x7u << 0))   | ((type & 0x7u) << 0);	/* TBL_TYPE [2:0]   */
	ctrl = (ctrl & ~(0x1u << 3))   | (1u << 3);		/* CMD_TYPE=write   */
	ctrl = (ctrl & ~(0x7u << 4))   | (1u << 4);		/* ACCESS_METHOD=1  */
	ctrl = (ctrl & ~(0xfffu << 9)) | ((addr & 0xfffu) << 9);/* ADDR [20:9]      */
	sw_wr(TBL_CTRL_OFF, ctrl);
	tbl_wait();
}

void rtl9602c_datapath_tables_init(void)
{
	/* ⚠ THIS POINTER WAS CALLED `ipsel`, WHICH IS ANOTHER REGISTER'S NAME.
	 * SOC_IP_SEL is 0xb8000600; this is 0xb800063c, and the two gate
	 * different things. Renamed the day it was proven wrong. */
	void __iomem *sw_en = SOC_SW_ENABLE;
	int port, idx;

	/*
	 * This is EXPORTED and the caller is another driver, so it must not assume this
	 * one probed (measured 2026-08-27). rtl9602c_eth.c calls it from ndo_open and
	 * every access below is an offset from `swcore_base`, which is NULL until this
	 * driver's probe ioremaps it. Booting with
	 * initcall_blacklist=rtl9602c_gpon_init proved the consequence: the board died
	 * 6.36 s in with no kernel message at all, because a write to NULL + offset is
	 * an unmapped bus access and this SoC answers that with a silent FULL-CHIP
	 * RESET. There is no oops and nothing to grep, so a NULL deref here looks
	 * exactly like a hardware fault.
	 */
	if (!swcore_base) {
		pr_warn_once("luna-gpon: datapath_tables_init called before this driver probed (swcore_base is NULL) -- skipped; the switch fabric is NOT initialised\n");
		return;
	}

	if (!full_datapath_init)
		return;

	/* 1) switch_init -------------------------------------------------- */
	writel(readl(sw_en) | SW_EN_BIT, sw_en);	/* see the naming note in
							 * luna_eth_regs.h    */
	pi_field(PI_PON_TB_CTRL, 7, 0, 0x6e);			/* PON_TB_CTRL tick      */
	pi_field(PI_PON_TB_CTRL, 15, 8, 0x95);
	sw_field(SW_METER_TB_CTRL, 7, 0, 43);			/* METER_TB_CTRL tick    */
	sw_field(SW_METER_TB_CTRL, 15, 8, 189);
	sw_field(SW_SCH_WFQ_TKN_CTRL, 0, 0, 1);		/* SCH_WFQ_TKN_CTRL      */
	sw_field(SW_LINE_RATE_2500M, 18, 0, 0x3ffff);	/* LINE_RATE_2500M       */
	sw_field(WRAP_GPHY_MISC, 0, 0, 1);			/* PATCH_PHY_DONE        */
	sw_field(CFG_UNHIOL, 0, 0, 1);			/* CFG_UNHIOL IPG_COMP   */
	sw_field(SW_P_MISC_PORT(swc->sw, swc->sw->cpu_port), 2, 2, 1);	/* P_MISC[CPU] RX_SPC */
	/* The ACCEPT_MAX_LEN half is closed, and only on the die that was READ
	 * (2026-09-09). The loop stopped at 3 on every chip while the port map declares
	 * SIX ports on the RTL9603CVD and TEN on the RTL9607C, so ports 4..N never
	 * received it on the dies the per-chip work exists to serve -- and it turned up
	 * a REGRESSION rather than an omission: we wrote 2031 where stock writes 16368.
	 * THE LUT-ACTION LOOPS BELOW STAY OWED, deliberately: nothing here measured
	 * them, a trial that wrote stock's exact LUT values in this same block BROKE
	 * THE DS PATH, and "the register next door was settled" is not evidence about a
	 * different register.
	 * And OURS IS UNMEASURED: the value below is what stock does, read on stock. */
	for (port = 0; port <= swc->amax_last_port; port++)
		sw_field(ACCEPT_MAX_LEN_CTRL + port * 4, swc->amax_msb, 0,
			 (swc->amax_pon_val && port == swc->sw->pon_port)
			 ? swc->amax_pon_val : swc->amax_val); /* ACCEPT_MAX_LEN */
	/* the PON port when it lies BEYOND this die's covered range -- the
	 * RTL9607C's is port 5 and its loop stops at 3. */
	if (swc->amax_pon_val && swc->sw->pon_port > swc->amax_last_port)
		sw_field(ACCEPT_MAX_LEN_CTRL + swc->sw->pon_port * 4,
			 swc->amax_msb, 0, swc->amax_pon_val);

	/* 2) l2_init: per-port action defaults (FORWARD). NOTE: a trial that wrote the
	 * exact stock LUT-action values (0x1c000=0x1a, 0x1c00c=0xaa, ...) + the MSTI/STP
	 * region BROKE the DS path (omcirx went 0) — our other config doesn't satisfy
	 * those actions' assumptions, and STP already reads 0x0f forwarding anyway. Keep
	 * the working FORWARD(0) defaults; do NOT match those stock LUT actions. */
	sw_field(SW_LUT_CFG, 22, 22, 1);			/* LUT LINKDOWN_AGEOUT   */
	for (port = 0; port <= 3; port++) {
		sw_field(LUT_LEARN_OVER_CTRL, port * 2 + 1, port * 2, 0);
		sw_field(SW_LUT_AGEOUT_CTRL, port, port, 1);
		sw_field(SW_LUT_UNKN_SA_CTRL, port * 2 + 1, port * 2, 0);
		sw_field(SW_LUT_UNMATCHED_SA_CTRL, port * 2 + 1, port * 2, 0);
		sw_field(UNKN_IP4_MC, port * 2 + 1, port * 2, 0);
		sw_field(UNKN_L2_MC, port * 2 + 1, port * 2, 0);
		sw_field(LUT_UNKN_UC_DA_CTRL, port * 2 + 1, port * 2, 0);
		sw_field(LUT_SYS_LRN_LIMIT, port, port, 1);
	}
	for (idx = 0; idx < 0x200 && tbl_ok && table_engine_ok; idx++) {
		sw_wr(TBL_WRDATA_OFF + 0x0, 0);
		sw_wr(TBL_WRDATA_OFF + 0x4, 0);
		tbl_write(/*L2_UC*/ 0, idx);
	}

	/* 3) vlan_init: default VLAN-1 (all ports member) — only ENABLE
	 *    filtering if the member entry actually wrote (else keep VLAN off,
	 *    our working baseline). Risky table-engine path: gated by table_engine_ok. */
	for (port = 0; port <= 3; port++) {
		sw_field(SW_VLAN_PORT_ACCEPT_FRAME_TYPE, port * 2 + 1, port * 2, 0);	/* ACCEPT ALL    */
		sw_field(SW_VLAN_EGRESS_TAG + 4 * port, 1, 0, 0);		/* EGRESS ORIG   */
	}
	if (table_engine_ok) {
		tbl_ok = true;				/* retry engine for VLAN  */
		sw_wr(TBL_WRDATA_OFF + 0x0, (0xfu << 4) | 0xfu);  /* untag|mbr=all */
		sw_wr(TBL_WRDATA_OFF + 0x4, 0x7f);
		tbl_write(/*VLAN*/ 1, /*vid*/ 1);
		if (tbl_ok) {
			for (port = 0; port <= 3; port++)
				sw_field(SW_VLAN_INGRESS, port, port, 1);	/* VLAN_INGRESS  */
			/* VLAN_FILTER ON for ranging/config (reliable onlining); the FSM auto-clears
			 * it once stably at O5 to open LAN access (see vlan_lan_open in gpon_fsm_poll).
			 * lan_keep_open (default) keeps LAN open from boot -> never assert the filter,
			 * so a bad cold-start (no O5) or a WAN-disconnect can't kill LAN management. */
			if (!lan_keep_open)
				sw_field(SW_VLAN_CTRL, 0, 0, 1);	/* VLAN_FILTER on (config phase) */
			sw_field(SW_VLAN_CTRL, 4, 4, 0);
		}
	}

	/* 4) port_init: CPU + PON force link UP (already done in swcore bringup;
	 *    re-assert for stock fidelity/order). Ports 0,1 stay auto. */
	sw_field(FORCE_P_ABLTY + 3 * 4, 1, 0, 2); sw_field(FORCE_P_ABLTY + 3 * 4, 2, 2, 1);
	sw_field(FORCE_P_ABLTY + 3 * 4, 4, 4, 1); sw_wr(ABLTY_FORCE_MODE + 3 * 4, 0xfff);
	sw_field(FORCE_P_ABLTY + 2 * 4, 1, 0, 2); sw_field(FORCE_P_ABLTY + 2 * 4, 2, 2, 1);
	sw_field(FORCE_P_ABLTY + 2 * 4, 4, 4, 1); sw_wr(ABLTY_FORCE_MODE + 2 * 4, 0xfff);

	/* 5) cpu_init: TAG_AWARE AFTER the CPU port is forced link-up. */
	sw_field(SW_MAC_CPU_TAG_CTRL, 8, 8, 1);		/* TRAP_TAGET_INSERT_EN  */
	sw_field(SW_MAC_CPU_TAG_CTRL, 9, 9, 1);		/* TAG_AWARE             */

	/* 6) trap_init: RMA baseline */
	sw_field(RMA_CFG, 2, 0, 0);
	sw_field(UNKN_MC_CFG, 2, 0, 0);
	sw_field(RMA_CTRL01, 5, 4, 2);
	sw_field(RMA_CTRL02, 5, 4, 2);
	sw_field(OAM_CTRL_0, 2, 0, 0);
	sw_field(QOS_UNI_TRAP_PRI_CTRL, 0, 0, 0);

	/* 7) classification setup: CF_CFG = the EXACT live-working-stock value 0x1d009
	 *    (diffed stock-vs-ours 2026-06-13) = CF_US_PERMIT=1 + CF_SEL_PON_EN=1 (the
	 *    classification engine routes and gates the PON path) + CF_PATTERN1_NUM=128
	 *    + WANIF_DEFAULT_MULTICAST=0xe. The PON-port egress of the cpu-tagged
	 *    US-OMCI is gated by the CF engine selecting the PON path: with
	 *    CF_SEL_PON_EN=0 the frame is never routed to the PON port. A prior round
	 *    read the classification-init SOURCE default and wrongly reverted
	 *    CF_US_PERMIT to 0 -- the GPON driver init overrides it, and the LIVE stock
	 *    register is the ground truth. */
	sw_wr(CF_CFG, 0x0001d009u);
	/* Port isolation + BUM-flood masks to the live-stock values (we had 0x3fffff /
	 * 0x0b; stock = 0x000ff9ff / 0x08 CPU-only). */
	/* FOUR CONSECUTIVE ARRAY ELEMENTS, not four named registers. The chipdefs name
	 * 0x27008 PISO_EXT on the RTL9602C and 0x2700c PISO_EXT on the RTL9603CVD --
	 * not because the register moved, but because the ARRAY's extent differs and
	 * each map names a different element of it. Treating one element as a
	 * standalone register made the G24W write 0x2700c twice and never 0x27008.
	 * An offset carrying a chipdef NAME is not evidence that it is a standalone
	 * register. Written as the array it is. */
	sw_wr(SW_PISO_PORT + 0 * SW_PISO_PORT_STRIDE, 0x000ff9ffu);
	sw_wr(SW_PISO_PORT + 1 * SW_PISO_PORT_STRIDE, 0x000ff9ffu);
	sw_wr(SW_PISO_PORT + 2 * SW_PISO_PORT_STRIDE, 0x000ff9ffu);
	sw_wr(SW_PISO_PORT + 3 * SW_PISO_PORT_STRIDE, 0x000ff9ffu);
	sw_wr(LUT_BC_FLOOD, 0x00000008u); sw_wr(LUT_UNKN_MC_FLOOD, 0x00000008u); sw_wr(LUT_UNKN_UC_FLOOD, 0x00000008u);

	/* 8) ponmac_init: PON-IP scheduler + OMCI egress steering */
	sw_field(DYNGASP_CTRL, 3, 3, 1);			/* DYNGASP_CMP_INV, bit 3 on all three dies */
	if (serdes_stock_analog) {
		/* Match live-stock post-reset SDS_ANA: REG01 (0x22584)=0x73a4 (BEN_TTL_OUT
		 * bit14=1, BENLA_LDOVREF[0]=0) + REG11 (0x225ac) RX_FILT_CONFIG=0. The golden table sets
		 * these BEFORE the SDS reset, which wipes REG01 bit14 / REG11 RX_FILT to defaults
		 * (0x33a4 / 0xb008); we re-apply them here, post-reset, exactly like stock. This
		 * was the ONLY stock-vs-ours SerDes register diff. ⚠ bit14 is REG_BEN_TTL_OUT per
		 * the 9602C chipdef, not a CMU bit (2026-09-11): the value is live-stock, the old
		 * block -> the marginal TX serializer behind the cold-start ~50% US-TX "Laser out". */
		sw_field(SDS(0x22584), 14, 14, 1);		/* REG01 REG_BEN_TTL_OUT = 1 (stock 0x73a4) */
		sw_field(SDS(0x22584),  0,  0, 0);		/* REG01 BENLA_LDOVREF[0] = 0 (stock) */
		sw_field(SDS(0x225ac),  7,  0, 0);		/* REG11 RX_FILT_CONFIG = 0 (stock)   */
	} else {
		sw_field(SDS(0x22584), 0, 0, 1);		/* legacy: BENLA_LDOVREF[0] = 1       */
	}
	pi_field(PI_PON_BW_THRES, 29, 16, 5);			/* PON_BW_THRES last     */
	pi_field(PI_PON_BW_THRES, 13, 0, 5);			/* PON_BW_THRES runt     */
	if (sch_ctrl_stock)
		/* Match LIVE STOCK exactly: PIR_DROP(b18)=1 | METER_OP(b17)=1 |
		 * WFQ_BURSTSIZE[15:0]=0x6000. See sch_ctrl_stock param comment. */
		pi_wr(PI_PON_SCH_CTRL, 0x00066000u);
	else
		pi_field(PI_PON_SCH_CTRL, 18, 18, pir_drop ? 1 : 0);	/* legacy bit18-only A/B */
	for (idx = 0; idx < 8; idx++) {
		pi_packed_set(PI_PON_WFQ_TYPE, idx, 1, 0);	/* WFQ_TYPE = STRICT (1b packed) */
		pi_field(PI_PON_QID_CIR_RATE + idx * PI_QID_RATE_STRIDE, 31, 0, 0);	/* QID_CIR_RATE = 0      */
	}
	sw_field(PON_TRAP_CFG, 2, 0, 7);			/* PON_TRAP_CFG OMCI_MPCP_PRIORITY=7 -> steer OMCI egress to PON queue 7 */
	/* PIR_DROP (0x2194 bit18) is cleared for rev-A above (pir_drop param, default 0).
	 * Do NOT "restore" it to 1: the working stock firmware's own behaviour
	 * CONFIRMS rev-A clears it ("must turn off due to the tcont 16"). The old
	 * "live-stock reads 0x66000, keep it set" note read the power-up default
	 * BEFORE the rev-A clear. */
	sw_field(SW_P_MISC_PORT(swc->sw, swc->sw->pon_port), 2, 2, 1);	/* P_MISC[PON] RX_SPC */
	sw_field(SW_P_MISC_PORT(swc->sw, swc->sw->cpu_port), 2, 2, 1);	/* P_MISC[CPU] RX_SPC */

	pr_info("luna-gpon: datapath_tables_init done (tbl_ok=%d)\n", tbl_ok);
}
EXPORT_SYMBOL(rtl9602c_datapath_tables_init);

/*
 * Configure the PON packet datapath (PON-IP) for GPON before the MAC reset. The
 * block comes up disabled, the SRAM descriptor accounting is programmed for
 * 128-byte pages with no DRAM reservation, GPON mode is selected, the upstream
 * FIFO thresholds and PONNIC framing are set, and the packet buffers are enabled
 * last. Until this runs the MAC has nowhere to land downstream frames and the
 * reset handshake does not settle.
 */
/* NOT __init: also called from rtl9602c_eth_open() (the eth driver) to RE-RUN the
 * full PON US/DS-NIC bring-up AFTER the GMAC IP-block reset, so the US-NIC RX engine
 * latches against the freshly-reset GMAC (stock order: GMAC reset -> then PON-NIC). */
void gpon_pbo_init(void)
{
	luna_data_suspend();
	/* 1. Halt GMII and disable both packet buffers while reconfiguring. */
	/* Enable the upstream GMII TX/RX framer to its O5 operating value (0x90101070,
	 * GMII_TX_EN|GMII_RX_EN + US datapath bits). This was forced to 0, leaving
	 * the PON-IP upstream datapath DISABLED — the ONU composed Serial_Number_ONU
	 * but could never transmit it upstream, so the OLT never heard it and the
	 * ONU was stuck in O3. Must be set here in pbo_init (before the MAC reset);
	 * setting it post-boot is too late. */
	/* The US-NIC GMII enables are DEFERRED to the very END of this function, after
	 * the descriptor pool and PBUF_EN, mirroring the DS side and stock's
	 * off-configure-on edge. Root cause of "US-NIC RX never receives": the RX
	 * engine LATCHES the descriptor-pool and FIFO config at the GMII-RX-enable
	 * edge, so enabling GMII first latched an unprovisioned pool and every US frame
	 * was dropped at descriptor-fetch before the MAC. */
	pi_wr(PI_IO_CMD_0_US, 0x90101050);	/* GMII OFF; pool configured below, GMII enabled LAST */
	/* DS IO_CMD (the DMA/FIFO drain enable, 0x90081070) is written LAST, after the
	 * backpressure thresholds + PBUF_EN, so the DS engine drains out of a properly
	 * bounded buffer (see end of this function). */
	pi_field(PI_PONIP_CTL_US, 0, 0, 0);		/* CFG_PBUF_EN = 0        */
	pi_field(PI_PONIP_CTL_DS, 0, 0, 0);

	/* 2. Descriptor accounting (128B pages). US uses a DRAM packet pool (the
	 * US-NIC RX path REQUIRES it — see PI_IP_MSTBASE_US note); DS stays
	 * SRAM-only like stock. Allocate the 1MB US PBO DRAM pool once and point
	 * the HW at its phys base. The PON-IP DMAs received US packets into this
	 * region; the CPU never reads it, so plain contiguous pages + virt_to_phys
	 * suffice (stock uses a fixed reserved region at 0x07eff000). */
	{
		static unsigned long us_pool;

		if (!us_pool)
			us_pool = __get_free_pages(GFP_KERNEL, PI_US_DRAM_ORDER);
		if (us_pool)
			pi_wr(PI_IP_MSTBASE_US,
			      (u32)virt_to_phys((void *)us_pool));
		else
			pr_warn("luna-gpon: US PBO DRAM pool alloc failed; US-NIC RX may not work\n");
	}
	pi_field(PI_PON_DSC_CFG_US, 12, 0, PI_US_SRAM_NO);
	pi_field(PI_PON_DSC_CFG_US, 28, 16, PI_US_DRAM_PAGES);	/* RAM_NO = SRAM+DRAM (0x1fff) */
	pi_field(PI_DSCRUNOUT_US, 12, 0, PI_US_SRAM_RUNOUT);
	pi_field(PI_DSCRUNOUT_US, 28, 16, PI_US_DRAM_RUNOUT);	/* DRAM runout (0x1f58) */
	/* The DS twin. A chip that declares no DS pool keeps the SRAM-only words
	 * this driver has always written -- same fields, same values, same order. */
	if (!swc->ds_dram_order) {
		pi_field(PI_PON_DSC_CFG_DS, 12, 0, PI_DS_SRAM_NO);
		pi_field(PI_PON_DSC_CFG_DS, 28, 16, PI_DS_SRAM_NO);
		pi_field(PI_DSCRUNOUT_DS, 12, 0, PI_DS_SRAM_RUNOUT);
		pi_field(PI_DSCRUNOUT_DS, 28, 16, 0);
	} else {
		static unsigned long ds_pool;

		if (!ds_pool)
			ds_pool = __get_free_pages(GFP_KERNEL, swc->ds_dram_order);
		if (ds_pool) {
			pi_wr(PI_IP_MSTBASE_DS,
			      (u32)virt_to_phys((void *)ds_pool));
			pi_wr(PI_PON_DSC_CFG_DS, swc->ds_dsc_cfg);
			pi_wr(PI_DSCRUNOUT_DS, swc->ds_dscrunout);
		} else {
			/* ★ NO POOL ⇒ KEEP THE SMALL SRAM GEOMETRY. Writing the
			 * DRAM-scale page counts with no pool behind them would
			 * point the PBO at physical address 0. */
			pr_warn("luna-gpon: DS PBO DRAM pool alloc failed; DS stays SRAM-only and DS OMCI may be dropped\n");
			pi_field(PI_PON_DSC_CFG_DS, 12, 0, PI_DS_SRAM_NO);
			pi_field(PI_PON_DSC_CFG_DS, 28, 16, PI_DS_SRAM_NO);
			pi_field(PI_DSCRUNOUT_DS, 12, 0, PI_DS_SRAM_RUNOUT);
			pi_field(PI_DSCRUNOUT_DS, 28, 16, 0);
		}
	}

	/* PBO backpressure thresholds (SRAM-only, 128B pages). Without these every
	 * threshold field reads 0, so the PBO treats the small DS SRAM pool as
	 * instantly over-threshold and never releases switch flow-control — which is
	 * why enabling the full DS DMA backs the buffer up and stalls the US. The
	 * per-SID RPV thresholds (0x02458, stride 4) are flow-control limits, NOT SID
	 * validity (writing them for every SID is safe; SIDVALID/SID2QID untouched).
	 * Values: us_sram_runt 126 -> stop 125, glb on/off 125/123; per-SID 150/130
	 * (page scale 1, non-tripping); DS flow-ctrl on/off 2/22. */
	/* DRAM-pool thresholds (stock live values; stock formula: stop=us_dram_runt-40,
	 * global ON/OFF scaled to the 1MB pool). The old 125/125/123 were the
	 * SRAM-only values — far too small for the DRAM pool, tripping flow
	 * control immediately. */
	pi_field(PI_PON_SID_STOP_TH, 12, 0, 0x1f30);
	pi_field(PI_PON_SID_GLB_TH, 28, 16, 0x1ee0);
	pi_field(PI_PON_SID_GLB_TH, 12, 0, 0x1e40);
	/* Per-SID reserved-page thresholds, 150/130 -- the vendor's own values
	 * for this init (tier 3, rtl9603cvd_raw_pbo_memUsage_init).  THE BOUND
	 * IS THE CHIP'S ARRAY, NOT THE CHIP'S SID COUNT: see sid_rpv_entries.
	 * A chip that declares none (no PON-IP block) writes none. */
	{
		unsigned int sid;

		for (sid = 0; sid < swc->sid_rpv_entries; sid++) {
			u32 off = PI_PON_SID_RPV_TH + sid * PI_RPV_TH_STRIDE;

			pi_field(off, 28, 16, 150);
			pi_field(off, 12, 0, 130);
		}
	}
	/* DS flow-control thresholds: on the SRAM-only geometry they are 2/22
	 * pages; a chip running the DRAM pool needs its own stock's scale, or the
	 * PBO sits permanently over-threshold against an 8192-page pool. */
	if (swc->ds_dram_order && swc->ds_fc_config)
		pi_wr(PI_PON_FC_CONFIG_DS, swc->ds_fc_config);
	else {
		pi_field(PI_PON_FC_CONFIG_DS, 12, 0, 22);
		pi_field(PI_PON_FC_CONFIG_DS, 28, 16, 2);
	}

	/* 3. GPON mode (not EPON) + upstream RXC stop + US FIFO thresholds. */
	pi_field(PI_PONIP_CTL_US, 2, 2, 0);		/* CFG_EPON_MODE = 0      */
	pi_field(PI_PONIP_CTL_DS, 2, 2, 0);
	pi_field(PI_PONIP_CTL_US, 1, 1, 1);		/* CFG_STOP_RXC_EN = 1    */
	pi_field(PI_PON_US_FIFO_CTL, 5, 4, 1);		/* USFIFO_SPACE = 1       */
	pi_field(PI_PON_US_FIFO_CTL, 3, 0, 3);		/* USFIFO_START = 3       */

	/* 4. 128-byte page size everywhere (PON-IP descriptors + PONNIC pages). */
	pi_field(PI_PON_DSC_CFG_US, 14, 13, 0);
	pi_field(PI_PON_DSC_CFG_DS, 14, 13, 0);
	pi_field(PI_IO_CMD_1_US, 5, 4, 0);		/* RPAGE_SIZE = 128B      */
	pi_field(PI_IO_CMD_1_US, 1, 0, 0);		/* TPAGE_SIZE = 128B      */
	pi_field(PI_IO_CMD_1_DS, 5, 4, 0);
	pi_field(PI_IO_CMD_1_DS, 1, 0, 0);
	pi_field(PI_IO_CMD_1_DS, 27, 27, 1);		/* PRECISE_DMA_EN — DS precise/aligned DMA transfers; the O5 DS IO_CMD_1 value is 0x08000000. Without it the DS RX DMA never lands a frame, so filled stays 0. */

	/* 5. PONNIC datapath: almost-full RX backpressure + TX stop/extra. */
	/* R_DBG_FUNC_SEL[7:5] = the US-NIC ingest-source select: field-bit-1 (=reg 0x40)
	 * wires switch-PON-port-2-egress -> PON-IP US-NIC GMII-RX so the US-NIC RECEIVES
	 * + SID-stamps US OMCI (RX_SID_GOOD_CNT_US can increment). This is the SYMMETRIC
	 * twin of the DS fix at the end of this function (PI_PROBE_SELECT_DS=0x40): the
	 * old `pi_field(...,1,1,1)` wrote REGISTER bit1 (0x02) — the same bit1-vs-bit6
	 * error the DS side had — leaving [7:5]=0 so US OMCI never reached the US-NIC
	 * (rxsid=0 regardless of cpu-tag). Stock pbo_init sets field-bit-1 (live 0x40). */
	pi_field(PI_PROBE_SELECT_US, 7, 5, 2);		/* R_DBG_FUNC_SEL=010b => reg 0x40 */
	pi_field(PI_CFG_US, 26, 26, 1);			/* E_EN_RFF_AFULL         */
	pi_field(PI_CFG_US, 17, 17, 1);			/* EN_TX_STOP             */
	pi_field(PI_CFG_US, 16, 16, 1);			/* EN_TXE_EXTRA           */
	/* (0xD400 PROBE_SELECT_DS is set to the stock golden 0x40 at the end of this
	 * function, with 0xD404/0xD42C — the DS-NIC drain config, not a debug probe.) */
	pi_field(PI_CFG_DS, 26, 26, 1);
	pi_field(PI_CFG_DS, 17, 17, 1);
	pi_field(PI_CFG_DS, 16, 16, 1);
	/* CFG_DS[6:0] = RX_SID: the stream-id the DS-NIC STAMPS on every frame it egresses
	 * over the internal MII into GMAC0's GMII-RX. The GMAC's CPUtag1CR SID-64 trap only
	 * fires (and the GMAC only accepts the on-wire cpu-tag) when this SID = 64. Reset
	 * default is 0x40 but the pbo MAC reset can clear it; set it explicitly. */
	pi_field(PI_CFG_DS, 6, 0, GPON_OMCC_FLOW);	/* RX_SID = this chip's OMCC SID */
	/* CFG_US[6:0] = RX_SID for the US-NIC, symmetric to the DS stamp above. The
	 * US-NIC stamps this stream-id on frames it receives from the switch (the
	 * CPU's US OMCI), so they classify to SID 64 -> GEM port 2 -> physical queue
	 * 64 -> T-CONT 16. Without it the OMCI reaches the US-NIC with NO stream-id
	 * (the switch strips our software cpu-tag at the PON-port egress) so it never
	 * enqueues to qid 64 and the OMCC US GEM stays empty (gem2=0, idle16 high).
	 * The DS stamp alone is not enough — this is the US half that was missing. */
	pi_field(PI_CFG_US, 6, 0, GPON_OMCC_FLOW);	/* US-NIC RX_SID = this chip's OMCC SID */

	/* 6. PONNIC TX framing (IFG, preamble, padding) + RX accept-CRC-error. */
	pi_field(PI_TX_CFG_US, 12, 10, 3);		/* IFG                    */
	pi_field(PI_TX_CFG_US, 2, 1, 1);		/* preamble length        */
	pi_field(PI_TX_CFG_US, 0, 0, 1);		/* TX padding             */
	pi_field(PI_RX_CFG_US, 5, 5, 1);		/* accept CRC error       */
	pi_field(PI_TX_CFG_DS, 12, 10, 3);
	pi_field(PI_TX_CFG_DS, 2, 1, 1);
	pi_field(PI_TX_CFG_DS, 0, 0, 1);
	pi_field(PI_RX_CFG_DS, 5, 5, 1);

	/* 7. Enable upstream and downstream packet buffers. (The O5 PONIP_CTL_DS value
	 * is 0x81 = +bit7, but setting bit7 here destabilised the link with our
	 * incomplete DS routing — left at bit0 only for stability; revisit with the
	 * full DS datapath.) */
	/* US-NIC PBUF_EN=1. NOTE: live stock runs ctl_us(0x20ac)=0 (PBUF OFF, US streams
	 * to the GTC), but setting it 0 on THIS clean-room driver regressed the link to
	 * "Laser out" + deactivate (ks90/91) whereas PBUF_EN=1 holds O5 Active (ks89) —
	 * our incomplete US datapath depends on the buffer. A clean-room divergence;
	 * revisit when the full US datapath matches stock. */
	pi_field(PI_PONIP_CTL_US, 0, 0, READ_ONCE(luna_activation_ready));
	pi_field(PI_PONIP_CTL_DS, 0, 0, 1);
	pi_field(PI_PONIP_CTL_DS, 7, 7, 1);		/* CFG_TX_PAUSE low bit -> O5 value 0x81 (DS buffer release; safe now thresholds bound the buffer) */

	/* 8. Enable the full downstream PONNIC DMA drain (LAST — after the thresholds
	 * + PBUF_EN above). 0x90081070 = MAX_DMA_SEL_0[31] | EARLY_TX_EN[28] |
	 * TX_FIFO_THR[20:19]=1 | RX_FIFO_THR[12:11]=2 | RX_MAX_DMA_SEL[7:6]=1 |
	 * GMII_RX_EN[5] | GMII_TX_EN[4]. The DMA/FIFO fields start the DS engine
	 * draining de-encapsulated GEM frames out of the (now bounded) buffer into the
	 * switch -> CPU-port GMAC; DS OMCI (SID 64) egresses on the GMAC RX with the
	 * cpu-tag stream-id 64 that the NIC's OMCI hook catches. */
	/* Force the DS-NIC <-> GMAC0 internal-MII link UP (golden 0x106e8400) -- the
	 * missing symmetric twin of the US write below. The bootloader latches it once,
	 * but rtl9602c_eth_open()'s GMAC0 IP-block power-cycle tears down the GMAC0 side
	 * and nothing re-asserts it, so a de-encapped DS frame counts RX_OK at the
	 * DS-NIC MAC and never crosses the un-trained MII into the GMAC RX ring. */
	pi_wr(PI_MEDIA_STS_DS, 0x106e8400u);
	pi_wr(PI_IO_CMD_0_DS, 0x90081070u);

	/* PON-IP DS-NIC drain config. A live stock ONU online and draining real OMCI has
	 * these SET -- 0xd400=0x40, 0xd404=0x11100348, 0xd42c=0x40 -- with
	 * PKT_OK_CNT_DS climbing. Ours left them 0 (and wrote 0xd400 bit1 instead of
	 * bit6), so de-encapsulated DS OMCI was never handed off to the GMAC NIC. The
	 * earlier "stock never touches them" note was wrong, measured against the live
	 * stock datapath. Written LAST, after IO_CMD_0_DS, matching stock verbatim. */
	pi_wr(PI_PROBE_SELECT_DS, 0x00000040u);
	pi_wr(PI_DS_NIC_CFG_D404, 0x11100348u);
	pi_wr(PI_CONFIG_CLK_DS, 0x00000040u);

	/* 9. US-NIC GMII enable — the VERY LAST write, after the descriptor pool,
	 * PBUF_EN, and the DS drain, mirroring IO_CMD_0_DS above. The US RX engine
	 * latches the now-provisioned pool/FIFO config at this GMII_RX_EN[5] rising
	 * edge (the root cause of US-NIC RX never receiving: we used to enable GMII
	 * FIRST, latching an empty pool). Force the GMAC0-TX -> US-NIC internal-MII
	 * link up first, then enable GMII RX/TX. 0x90101070 = MAX_DMA_SEL_0[31] |
	 * EARLY_TX_EN[28] | TX_FIFO_THR[20:19]=2 | RX_FIFO_THR[12:11]=2 |
	 * RX_MAX_DMA_SEL[7:6]=1 | GMII_RX_EN[5] | GMII_TX_EN[4]. */
	if (!usnic_strip)
		pi_wr(PI_MEDIA_STS_US, 0x106e8400u);	/* stock NEVER writes 0x1bf04058 */

	/* Pre-arm the OMCC classification BEFORE the GMII_RX_EN latch edge below. The
	 * US-NIC RX engine latches its SID2QID/SIDVALID table at that rising edge, and
	 * this driver used to program the OMCC SID only later at OMCC-install, AFTER
	 * the edge -- so the engine latched without it and every upstream OMCI frame was
	 * dropped pre-MAC, with PKT_OK/ERR/MISS all reading 0 even though the GMAC
	 * transmitted it onto a live GMII.
	 * PACKED-TABLE ADDRESSING: the 7-bit SID2QID array is packed
	 * entries_per_word = 32/width = 4 per 32-bit word, top 4 bits unused -- NOT
	 * contiguous-bit-packed. word = sid/4, shift = (sid%4)*7. The old code wrote
	 * one word lower, which under this packing is a DIFFERENT SID's slot, and that
	 * misread is why an earlier "live-stock SID2QID[64]=63" reading was wrong: it
	 * was reading SID 56. SIDVALID and SID_Q_MAP_DS are unaffected, because 32
	 * divides evenly by their widths. */
	/* ★ THE INDEX IS THE CHIP'S, NOT A LITERAL (2026-09-08).  These two lines
	 * spelled SID 64 three ways -- word 16, the bare address 0x2144, and the
	 * shift 0 -- all of them the RTL9602C's OMCI flow.  On the RTL9603CVD the
	 * OMCI flow is 127, so the pre-arm armed a SID the OMCI block does not
	 * watch and left the real one invalid.  Derive all three from
	 * GPON_OMCC_FLOW, with the SAME packing arithmetic pi_packed_set() uses
	 * (that helper is defined further down this file, which is why the
	 * arithmetic is written out here rather than called). */
	{
		unsigned int f = GPON_OMCC_FLOW;
		unsigned int q2_lsb = (f % 4u) * 7u;	/* SID2QID: 7 bits, 4/word  */
		unsigned int v_bit  = f % 32u;		/* SIDVALID: 1 bit, 32/word */

		pi_field(PI_PON_SID2QID + (f / 4u) * PI_SID2QID_STRIDE,
			 q2_lsb + 6u, q2_lsb, GPON_OMCC_PHYS_QID & 0x7f);
		pi_field(PI_PON_SIDVALID + (f / 32u) * 4u, v_bit, v_bit, 1);
	}
	/* PON_OMCI_CFG[6:0] = OMCC SID 64 — the THIRD member of the US-NIC ingress
	 * classification triple (SID2QID + SIDVALID + OMCI_CFG). The two writes above
	 * were baked pre-edge, but OMCI_CFG was left for gpon_install_omcc (O5, AFTER
	 * this latch) — so the RX engine latched WITHOUT knowing which SID is the OMCI
	 * management stream and discarded every SID-64 upstream frame PRE-MAC (RX_OK/
	 * ERR/MISS all 0). The working firmware writes the FULL triple before the GMII
	 * edge: it sets PON_OMCI_CFG @ 0x1bf02154 [6:0] = 64, THEN does the
	 * NIC-bringup GMII edge. Complete the triple here so all three latch together. */
	/* ★ THE NAME, NOT THE LITERAL: PI_PON_OMCI_CFG is PI_X(0x02154), and on
	 *   the RTL9603CVD that register MOVED to 0x021a4 (chipdef-derived, see
	 *   luna_pi_moves_9603cvd). The bare literal wrote the 9602C address on
	 *   9603CVD silicon, so the OMCC SID never reached PON_OMCI_CFG there.
	 *   pi_x() is identity on every chip that declares no moves, so this is
	 *   unchanged on the RTL9602C and the RTL9607C. */
	pi_field(PI_PON_OMCI_CFG, 6, 0, GPON_OMCC_FLOW);	/* [6:0] = this chip's OMCC SID */

	/* Data remains inhibited after a global NIC/PBO reset. The accepted OMCI
	 * owner retires any old queue and applies its selected qid on the timer. */
	pi_field(PI_PON_SIDVALID, GPON_DATA_FLOW, GPON_DATA_FLOW, 0);

	/* MOCIR force-mode (stock QoS init): PON-IP 0x2170 MOCIR_FRC_MD=0x1FFFF,
	 * 0x2174 MOCIR_FRC_VAL=0x1FFFF — force the per-flow committed-info-rate to a
	 * fixed mode so the US-NIC's CIR auto-detection cannot trip for the OMCC SID-64
	 * flow and silently discard the OMCI frame BEFORE the MAC counts it (the exact
	 * RX_OK/ERR/MISS-all-0 symptom = a pre-MAC discard). Our minimal init omitted it. */
	if (!usnic_strip) {
		pi_wr(PI_MOCIR_FRC_MD, 0x0001ffffu);	/* MOCIR_FRC_MD  = 0x1FFFF (all flows forced)   */
		pi_wr(PI_MOCIR_FRC_VAL, 0x0001ffffu);	/* MOCIR_FRC_VAL = 0x1FFFF (forced CIR = max)   */
	}

	/* Stock PON-MAC US-NIC datapath credit/threshold cluster, pinned to the EXACT
	 * runtime values read from the live stock ONU at O5. Our minimal init left the
	 * WHOLE cluster at 0, so the US-NIC's grant/credit logic never released a
	 * credit for the OMCI flow and every upstream frame was discarded BEFORE the
	 * MAC counted it -- a pre-MAC credit discard, not a link-down. Stock writes
	 * them just ahead of the GMII rising edge that latches US-NIC ingress, so they
	 * go here. The "5 written twice" register resolves to 0x2150 on live stock, not
	 * 0x20f4. */
	if (!usnic_strip) {
		pi_wr(PI_PON_BW_THRES, 0x00050005u);	/* PON US-NIC IP-status/BW threshold (reg924, 5+5)  */
		pi_wr(PI_PON_US_FIFO_CTL, 0x00000013u);	/* US-NIC datapath cfg                              */
		/* PONIP_DBG_CTRL_US MUST be 0x00086000 to match live stock:
		 *   bit19 DBG_IGNORE_TAG = 1 -- strip the CPU tag from US frames BEFORE the
		 *         PBO encapsulates them into GEM. Without it the CPU prefix is
		 *         included in the GEM payload, every US OMCI frame is malformed, and
		 *         the OLT discards what the GEM-US engine counts as sent.
		 *   bits[18:11] CFG_US_EP_IPG = 12.
		 * The prior value was 0x40, SID_NO only, with no DBG_IGNORE_TAG.
		 * The /proc/gpon handler ALSO writes this register to read the sidpage
		 * counter, and that write MUST preserve DBG_IGNORE_TAG -- it goes through
		 * pi_sid_page_cnt(), which ORs in this same stock base. */
		pi_wr(PI_PONIP_DBG_CTRL_US, 0x00086000u);	/* stock value (DBG_IGNORE_TAG=1) */
		if (dbru_blksize)
			pi_wr(PI_GPON_DPRU_RPT_PRD, 0x00003002u);	/* DPRU_RPT_PRD: DBA_BLKSIZE=48 plus report period 2,
										 * the live-stock 0x3002. The HW DBA/DBRu engine uses
										 * this byte->block divisor to ENCODE per-SID queued
										 * occupancy into the report the OLT reads to size
										 * grants; unset, the OLT sees 0 queued while the
										 * queue holds pages, grants once and STOPS. Stock's
										 * only writer is its init; ours had regressed it. */
		/* 0x20f4 = PON_IPSTS_US, a READ-ONLY init-ready status reg (bit0=PONIC_INITRDY);
		 * stock never writes it. The old pi_wr(0x20f4,1) was a no-op write to a reserved
		 * bit -> removed. It is POLLED before the GMII latch edge below (usnic_initrdy_poll). */
		pi_wr(PI_MOCIR_TH_H, 0x00001000u);	/* MOCIR_TH_H (request/grant credit threshold)      */
		pi_wr(PI_MOCIR_TH_L, 0x00001000u);	/* MOCIR_TH_L                                       */
		pi_wr(PI_PON_OLT_BW_MTR_FULL, 0x0003ffffu);	/* PON_OLT_BW_MTR_FULL (maxFlow)                    */
		pi_wr(PI_PON_TB_CTRL, 0x0000956eu);	/* PON_TB_CTRL (token bucket)                       */
		pi_wr(PI_PON_SCH_QMAP, 0x0000000fu);	/* PON_SCH_QMAP                                     */
		pi_wr(PI_PON_TCONT_EN, 0x00010001u);	/* PON US T-CONT enable                             */
	}

	/* GMAC0<->US-NIC internal link force, from the stock PON-MAC GPON mode-set.
	 * Without it a CPU direct-TX US-OMCI frame never reaches the US-NIC ingress.
	 * Stock releases the force once trained; we hold it up. */
	if (!usnic_strip) {
		sw_field(ABLTY_FORCE_MODE, 4, 0, 0xc);	/* ABLTY_FORCE_MODE[4:0]=0xc — GPON internal-link force   */
		/* RTL9602C only: 0x0f4 is CFG_FE_POLL_WD_1 here and SSC_CTRL_1 on the
		 * RTL9603CVD, which has no front-end poll/watchdog to translate to. */
		if (!is_9603cvd)
			sw_field(0x000f4, 5, 5, 1);	/* CFG_FE_POLL_WD_1[5]=1 — front-end GMII poll/watchdog */
		mdelay(10);			/* settle the forced internal link before PCS enables+edge */
		sw_field(SDS(0x22a70), 11, 11, 0);	/* SDS_EXT_REG28[11]=0 — release non-GPON SerDes-select    */
		/* The two WAN-PCS digital-enable writes below DIVERGE from stock on
		 * purpose: guarded off, the G24W measured optic_los=1 sdet=0 and stayed
		 * at O1 (2026-09-11). A stock-parity sweep must NOT close this gap. */
		sw_field(SDS(0x220e0), 9, 8, 1);	/* WSDS_DIG_2C[9:8]=1 — WAN-PCS digital enable */
		pi_field(PI_RSVD_PONIP_DS, 13, 13, 1);	/* RSVD_PONIP_DS[13]=1 — DS-side datapath enable           */
		pi_field(PI_RSVD_PONIP_DS, 12, 12, 1);	/* RSVD_PONIP_DS[12]=1                                     */
		sw_field(SDS(0x22080), 12, 12, 1);	/* WSDS_DIG_14[12]=1 — WAN-PCS digital enable (see above) */
	}

	/* Wait for the US PON-IP core to report init-ready before the GMII latch edge
	 * below. Stock has no such poll and relies on a fixed mdelay. */
	if (usnic_initrdy_poll) {
		int n;

		for (n = 0; n < SDS_LOCK_POLL_MAX; n++) {
			if (pi_rd(PI_PON_IPSTS_US) & BIT(0))		/* PONIC_INITRDY */
				break;
			udelay(200);
		}
		if (n >= SDS_LOCK_POLL_MAX) {
			pr_warn("luna-gpon: PON_IPSTS_US.PONIC_INITRDY not set after %dus; proceeding\n",
				SDS_LOCK_POLL_MAX * 200);
			if (usnic_initrdy_repulse) {		/* ACTIVE: re-roll the CDR lock */
				u32 cdr = sw_rd(SDS_ANA_COM_REG12);

				sw_wr(SDS_ANA_COM_REG12, cdr ^ BIT(15));
				mdelay(10);
				sw_wr(SDS_ANA_COM_REG12, cdr);
				for (n = 0; n < SDS_LOCK_POLL_MAX; n++) {
					if (pi_rd(PI_PON_IPSTS_US) & BIT(0))
						break;
					udelay(200);
				}
				pr_warn("luna-gpon: PONIC_INITRDY after CDR re-pulse: %s\n",
					(pi_rd(PI_PON_IPSTS_US) & BIT(0)) ? "ready" : "still-not-ready");
			}
		} else {
			pr_info("luna-gpon: PON_IPSTS_US.PONIC_INITRDY ready after %dus\n", n * 200);
		}
	}

	pi_wr(PI_IO_CMD_0_US, 0x90101070u);
}
EXPORT_SYMBOL(gpon_pbo_init);	/* re-run from rtl9602c_eth_open() after the GMAC reset */

static bool datapath_rearm = true;
module_param(datapath_rearm, bool, 0644);
MODULE_PARM_DESC(datapath_rearm,
	"re-arm the US-feed FSM at the end of device init (WSDS GPON soft-reset edge + re-run pbo_init), matching stock dataPath_reset; fixes the empty GEM-US TX bank (gemus_omcc=0) on the first grant");

static bool o5_feed_rearm = true;
module_param(o5_feed_rearm, bool, 0644);
MODULE_PARM_DESC(o5_feed_rearm,
	"re-arm the US-feed FIFO at O5 (after OMCC install), softirq-safe (feed FIFO edge only, NO WSDS soft-reset). Needed because our O3 TX-PLL relock (a SerDes reset) re-parks the US-feed AFTER the pre-ranging datapath_rearm, so it underflows on the first grant unless re-armed post-relock");

static u32 gpon_us_feed_rearm_cnt;	/* count of US-feed re-arms (pre-FSM + O5), shown in /proc/gpon */
static u32 gpon_us_intr_svc_cnt;	/* count of US-intr delta reads that found a latched event (us_intr_svc) */

/*
 * Faithful re-express of the working firmware's GPON datapath soft-reset, run as
 * the LAST step of device initialisation: a soft-reset EDGE on WSDS_DIG_00 bit10,
 * then a full PBO / US-feed re-arm.
 * gpon_pbo_init() runs EARLY, before the MAC soft-reset and GTC/BOH bring-up,
 * which then PARK the just-armed US-feed FSM -- so its GEM-US TX bank is empty on
 * the OLT's first grant, BANK_UNDERFL fires and no US burst goes out. Stock arms
 * the US-feed LAST; re-running it here (DS not yet locked so the WSDS edge is
 * safe, process context so the GFP_KERNEL pool re-run is safe) puts our order in
 * agreement with stock.
 */
static void gpon_us_feed_rearm(void)
{
	if (!READ_ONCE(luna_activation_ready))
		return;
	sw_field(WSDS_DIG_00, 10, 10, 0);	/* assert GPON datapath reset-B */
	sw_field(WSDS_DIG_00, 10, 10, 1);	/* release -> soft-reset edge   */
	gpon_pbo_init();			/* re-arm US-feed (pool persists)*/
	gpon_us_feed_rearm_cnt++;
	pr_info("luna-gpon: US-feed re-armed (WSDS GPON reset edge + pbo re-init, cnt=%u)\n",
		gpon_us_feed_rearm_cnt);
}

/*
 * Softirq-safe US-feed FIFO re-arm: the pi_* feed writes of gpon_pbo_init WITHOUT
 * the pool alloc / PONIC_INITRDY poll / WSDS soft-reset — so it is safe in the FSM
 * timer AND keeps the DS lock at O5. Re-runs the US-feed FIFO START edge so the
 * GEM-US TX bank fills again, after a later SerDes reset (our O3 TX-PLL relock)
 * parked the edge-armed feed. US-side only; does not touch DS or the installed OMCC.
 */
static void gpon_us_feed_rearm_light(void)
{
	if (!READ_ONCE(luna_activation_ready))
		return;
	pi_wr(PI_IO_CMD_0_US, 0x90101050u);	/* GMII off: park the feed       */
	pi_field(PI_PONIP_CTL_US, 0, 0, 0);	/* PBUF_EN = 0                   */
	pi_field(PI_PON_US_FIFO_CTL, 5, 4, 1);	/* USFIFO_SPACE = 1              */
	pi_field(PI_PON_US_FIFO_CTL, 3, 0, 3);	/* USFIFO_START = 3 (feed edge)  */
	pi_field(PI_PONIP_CTL_US, 0, 0, 1);	/* PBUF_EN = 1                   */
	pi_wr(PI_IO_CMD_0_US, 0x90101070u);	/* GMII latch -> re-arm the feed */
	gpon_us_feed_rearm_cnt++;
	/* Heartbeat only: this fires every FSM tick, so logging each one floods the console
	 * and drowns serial diagnostics. The count is the useful signal -- log it periodically
	 * (still readable, no flood). */
	if (!(gpon_us_feed_rearm_cnt % 1000))
		pr_info("luna-gpon: US-feed FIFO re-armed at O5 (feed edge, no reset, cnt=%u)\n",
			gpon_us_feed_rearm_cnt);
}

/*
 * The O3 post-relock feed un-park, in ONE place. The TX-CMU relock is a SerDes
 * reset and it re-parks the GEM-US feed run-state; this un-parks it with the FULL
 * WSDS datapath reset-B edge (the one the O5-light re-arm omits), at O3 while the
 * downstream is not yet locked.
 *
 * The second half is the LIGHT re-arm, and that is the whole point: this runs from
 * the FSM timer, i.e. SOFTIRQ, while gpon_us_feed_rearm() is the process-context
 * variant whose gpon_pbo_init() reaches __get_free_pages(GFP_KERNEL).
 * luna_op_o3_feed_reset was wired to the process-context one (found 2026-09-10),
 * so with core_fsm=1 the core's O3 arm reached a GFP_KERNEL allocation and a
 * PONIC_INITRDY poll from softirq, and un-parked the feed by a DIFFERENT body than
 * the FSM it replaces. Both callers now share this one.
 */
static void gpon_o3_feed_unpark(void)
{
	sw_field(WSDS_DIG_00, 10, 10, 0);
	sw_field(WSDS_DIG_00, 10, 10, 1);
	gpon_us_feed_rearm_light();
	pr_info("luna-gpon: O3 post-relock WSDS feed-reset edge (un-park GEM-US framer)\n");
}

/* Full BOSA page2 (slave 0x54) + page3 (slave 0x55) register dump for diagnostics.
 * Reads all 512 regs via the kernel I2C path. */
static int bosadump_proc_show_locked(struct seq_file *s, void *v);
static int bosadump_proc_show(struct seq_file *s, void *v)
{
	int ret;

	mutex_lock(&bosa_lock);
	ret = luna_stopping ? -ESHUTDOWN : bosadump_proc_show_locked(s, v);
	mutex_unlock(&bosa_lock);
	return ret;
}

static int bosadump_proc_show_locked(struct seq_file *s, void *v)
{
	int i, j;

	for (i = 0; i < 256; i += 16) {
		seq_printf(s, "P2_%02x:", i);
		for (j = 0; j < 16; j++)
			seq_printf(s, " %02x", bosa_read_reg(0x200 + i + j) & 0xff);
		seq_puts(s, "\n");
	}
	for (i = 0; i < 256; i += 16) {
		seq_printf(s, "P3_%02x:", i);
		for (j = 0; j < 16; j++)
			seq_printf(s, " %02x", bosa_read_reg(0x300 + i + j) & 0xff);
		seq_puts(s, "\n");
	}
	return 0;
}

/* Full PON-IP register dump in the EXACT format of cross-compiler/stock_dump_good.txt
 * ("0x1bf0XXXX YYYYYYYY"), so mine-vs-stock diffing needs no reformatting. Covers the
 * 0x0000-0x54fc span the stock oracle captured. Read once: cat /proc/pidump. */
static int pidump_proc_show(struct seq_file *s, void *v)
{
	/* Only the mapped PON-IP windows (gaps bus-fault on read). Ranges match the
	 * readable spans of cross-compiler/stock_dump_good.txt. */
	static const u32 ranges[][2] = {
		{0x0000, 0x03fc}, {0x2000, 0x2bfc}, {0x4000, 0x40fc}, {0x5400, 0x54fc},
	};
	u32 off, val;
	int r;

	for (r = 0; r < (int)ARRAY_SIZE(ranges); r++)
		for (off = ranges[r][0]; off <= ranges[r][1]; off += 4) {
			val = pi_rd(off);
			if (val)	/* skip zeros: keeps the dump small for reliable serial read */
				seq_printf(s, "0x%08x %08x\n", 0x1bf00000u + off, val);
		}
	seq_puts(s, "0x1bf0ffff ffffffff\n");	/* end marker */
	return 0;
}

/* Switch-core + GMAC + SerDes + GTC dump (the GMAC0->switch->US-NIC handoff path that is
 * OUTSIDE PON-IP space). Same "0xADDR VAL" format as stock_dump_good.txt for direct diff.
 * 0x1bxxxxxx regs come via swcore_base (sw_rd); GMAC 0x18012/0x18013 via a local ioremap. */
static int swdump_proc_show(struct seq_file *s, void *v)
{
	static const u32 sw[][2] = {		/* offsets into swcore_base (phys 0x1b000000) */
		{0x00000, 0x000fc},
		/* ★ 0x00100..0x002fc: the three per-port ABILITY arrays
		 * (FORCE_P_ABLTY 0x198 | P_ABLTY 0x1b8 | ABLTY_FORCE_MODE 0x1dc),
		 * SDS_CFG (0x200 on the RTL9603CVD) and SDS_FIB_STATUS (0x214).
		 * MEASURED 2026-08-27: none of these was reachable through this
		 * node -- the GPON RX-path question "does the SerDes report
		 * signal-detect / link-ok" could be asked on STOCK (its vendor
		 * node reads any address) and NOT on ours, so the one diff that
		 * would settle it could never be taken. */
		{0x00100, 0x002fc},
		{0x1c000, 0x1c0fc}, {0x20800, 0x2083c},
		{0x20c00, 0x20c3c}, {0x23000, 0x230fc}, {0x27000, 0x2703c},
		/* The registers this driver writes and COULD NOT READ BACK. Fifteen offsets
		 * that switch_init() and the LED/trap setup PROGRAM were outside every range
		 * this node dumps, so nothing could ever compare them against stock: there
		 * was no measurement that could disagree with us. That is how
		 * ACCEPT_MAX_LEN_CTRL kept 2031 on the LAN ports of a die whose stock writes
		 * 16368 -- invisible to traffic, because 2031 still clears a 1522 B tagged
		 * frame, and invisible to the diff from both ends.
		 * THE RULE, checked by ONU-test-case/swdump_readback_guard.py: every offset
		 * the driver writes must be readable back through this node. Not the whole
		 * chip -- just what we ourselves program.
		 * Each range is also proven readable on the silicon, and widened to whole
		 * register spans so the three dies' differing bases all sit inside one
		 * family-safe range rather than a per-chip table that would drift. */
		/* ⚠ EACH SPAN COVERS ALL THREE DIES, and the first cut of this
		 * table did not.  Sized from the RTL9603CVD alone it left
		 * PON_TRAP_CFG blind on the other two (0x111f8 on the 9602C,
		 * 0x11144 on the 9607C) and four more on the 9607C -- the same
		 * per-chip-literal mistake the SerDes window comment above
		 * describes, made again while fixing its cousin.  The guard is
		 * per chip precisely so that shows up.
		 */
		{0x11000, 0x1104c},	/* ACCEPT_MAX_LEN_CTRL, every die + its ports  */
		{0x110e0, 0x111fc},	/* PON_TRAP_CFG: 0x110ec / 0x111f8 / 0x11144   */
		{0x12000, 0x1200c},	/* TBL_CTRL_OFF / TBL_WRDATA_OFF               */
		{0x16000, 0x1601c},	/* CF_CFG                                      */
		{0x1c100, 0x1c2fc},	/* RMA_CFG, QOS_UNI_TRAP_PRI_CTRL, OAM_CTRL_0  */
		{0x1e000, 0x1e05c},	/* LED_MODE_SEL .. LED_PARA_EN + LED_DATA_CFG  */
		{0x23100, 0x2311c},	/* CFG_UNHIOL on the 9607C                     */
		{0x2d890, 0x2d8bc},	/* SW_SCH_WFQ_TKN_CTRL, SW_LINE_RATE_2500M     */
		/* SW_P_MISC_PORT is base 0x20004 + port * macpp_stride and BOTH halves are
		 * per chip: 0x20804/0x20c04 on the RTL9602C (stride 0x400, ports 2/3),
		 * 0x20404/0x20504 on the RTL9603CVD and 0x20504/0x20904 on the RTL9607C
		 * (stride 0x100). The table dumped only the 9602C's pair, so on the G24W the
		 * stock-vs-ours diff for the very register the RX_SPC repair reproduces was
		 * impossible from our end. */
		{0x13000, 0x1300c},	/* VLAN_PORT_ACCEPT_FRAME_TYPE/INGRESS/CTRL   */
		{0x17000, 0x1700c},	/* SW_LUT_CFG, SW_LUT_AGEOUT_CTRL             */
		{0x20400, 0x20430},	/* P_MISC[4] -- the RTL9603CVD's PON          */
		{0x20500, 0x20530},	/* P_MISC[5] -- its CPU, and the 9607C's PON  */
		{0x20900, 0x20930},	/* P_MISC[9] -- the RTL9607C's CPU            */
		{0x25000, 0x2500c},	/* SW_METER_TB_CTRL                           */
		{0x2a000, 0x2a024},	/* SW_VLAN_EGRESS_TAG + 4*port, ports 0..9    */
		/* The RTL9603CVD PON SerDes page: 0x022xxx on the RTL9602C, 0x040xxx here
		 * (+0x1e000), which is the offset error that kept the GTC in reset. Five
		 * sub-ranges rather than the whole 4 KB page, because the STOCK route reads 16
		 * bytes per console round-trip and a full page would cost ~4 min:
		 *   0x40000..0x400fc  WSDS_DIG_00/02/18/1D, FORCE_BEN
		 *   0x40500..0x405fc  SDS_ANA_MISC02, SDS_ANA_COM03/09/17/20/21/26/27
		 *   0x40800..0x4083c  SDS_REG0 (bit1 SP_SDS_EN_RX, the CDR-wedge bit)
		 *   0x40c00..0x40c7c  FIB_REG16 (FRC_SD / SEL_RX_SD)
		 *   0x40e00..0x40e7c  FIB_EXT_REG21 (analog-ready status)
		 * Exactly the offsets gpon_swc_9603cvd declares, so the dump can answer every
		 * question that table can ask. */
	};
	/* The GPON MAC / GTC block, through its OWN accessor. A range at 0x701000 sat
	 * in the `sw` table and read NOTHING: SWCORE_REG_SIZE is 0x41000, so sw_rd()
	 * maps only 0x00000..0x40fff and every read there came back zero -- measured
	 * 2026-09-01, when widening it produced "295 of 573 words differ, ours all
	 * 0x00000000" and the number was entirely the instrument.
	 * The GTC has its own ioremap covering exactly these offsets, so they are read
	 * through gpon_rd() and printed at the SWCORE-equivalent address, and the diff
	 * needs no second address space.
	 * The spans are the ORACLE's own: each is a contiguous run this board's STOCK
	 * returned through `diag register get all` at O5, so every one is proven
	 * readable before we read it -- the only safe way to widen a window whose gaps
	 * bus-fault. */
	static const u32 gt[][2] = {		/* offsets into gpon_base (phys 0x1b700000) */
		{0x010a0, 0x010c4}, {0x01180, 0x011c8},
		{0x01400, 0x015fc}, {0x02400, 0x027fc},
		{0x04280, 0x042a8}, {0x05080, 0x050ac},
		{0x050e0, 0x05100}, {0x06400, 0x065fc},
		/* swdump_readback_guard.py scanned only sw_field/sw_wr while its COVERAGE
		 * table already included this gt[] block, so its headline "WRITTEN BLIND: 0"
		 * was an all-clear over 88 GTC writes it never examined. A population
		 * narrower than the coverage it judges reports the reassuring answer by
		 * construction. What was hiding in there: GPON_RESET,
		 * GPON_GTC_DS_PLOAM_CFG, GPON_GTC_DS_LOS_CFG_STS, GPON_GTC_DS_ONU_ID_STATUS,
		 * the DS PLOAM indication and the TDM/OMCI/ETH PTI trio -- core GPON state a
		 * stock-vs-ours diff could never compare. All are inside the GTC's own
		 * ioremap, so unlike the SWCORE page there is no window to run past. */
		{0x0000c, 0x00014}, {0x01004, 0x01080}, {0x01200, 0x01208},
		{0x0200c, 0x0200c}, {0x03010, 0x03024}, {0x04064, 0x04098},
		{0x05010, 0x05054}, {0x050c0, 0x050c0}, {0x05140, 0x05140},
		{0x05200, 0x05200}, {0x0526c, 0x0526c}, {0x06020, 0x06024},
		{0x06260, 0x06260},
	};
	static const u32 gm[][2] = {		/* absolute phys (separate ioremap) */
		{0x18012000, 0x180120fc}, {0x18013400, 0x180134fc},
		/* The PCIe host controller, so its link state can be COMPARED with stock
		 * instead of interpreted. pcie-rtl9602c.c prints "link not trained" from
		 * readl(0xb8b00000 + 0x728) & 0x1f, and this board's VENDOR firmware holds at
		 * that address (read 2026-08-23, three times minutes apart) byte[0] STABLE
		 * 0x11 and byte[3] STABLE 0x03, with bytes 1-2 churning as counters.
		 * 0x11 is our own LINK_UP_STATE, and on big-endian MIPS `& 0x1f` takes the
		 * OTHER end of that word. Whether the field is byte[0] or byte[3] is NOT
		 * established -- both are stable -- so this dumps the BYTES and lets the two
		 * firmwares be compared rather than betting on a mask. */
		/* The flow table, which we have never been able to look at: 0x1b800000 is a
		 * page this driver maps NOWHERE -- sw_rd() covers SWCORE and gpon_rd() the
		 * GTC, and this is neither -- while the vendor CLI reads it, so every stock
		 * capture has contained it and every one of ours was blind to it.
		 * Measured on both boards' own stock: on the X111W 148 of 180 captured words
		 * are non-zero and decode as IPv4, including 0xC0A8010C, that board's own
		 * declared LAN address, paired with remote endpoints -- which is what a
		 * connection/flow table looks like. On the G24W the same page holds pool
		 * geometry and the chipdef-named CAM_TRF / CC_* block with CC_STA=1.
		 * It earns a range because accelerator counters earn their place when ours
		 * does NOT match stock -- this is that instrument, in silicon.
		 * THE RANGE IS THE PROVEN-CONTIGUOUS ONE AND NO WIDER: the G24W answered
		 * 0x800000..0x801000 as ONE run, and it contains every word the X111W
		 * answered. The rest of the page answered in PIECES and is left out until
		 * something needs it.
		 * NOTHING IS DECODED YET -- the entry stride and field layout are not
		 * established (results/artifacts/swcore/FINDING-flow-table-page-2026-09-09.md).
		 * Dumping it is a READ; writing a guessed structure into a driver is what the
		 * OWED markers in this file exist to prevent. */
		{0x1b800000, 0x1b800ffc},	/* the flow/connection table	*/
		{0x18b00700, 0x18b0073c},	/* HOSTCFG: 0x728 = LTSSM state	*/
		{0x18b01000, 0x18b0101c},	/* HOSTEXT: 0x008 = LTSSM enable	*/
		{0x18000040, 0x1800005c},	/* SOC_PINMUX at 0x4c		*/
	};
	/* The extra spans are PER DIE, and this set is the RTL9603CVD's: the three
	 * tables above are family facts, these two are the words the G24W's OWN stock
	 * answered that no range above asks for (swcore_blocks_from_capture.py,
	 * "NEVER ASKED FOR: 1599 word(s) in 116 contiguous run(s)").
	 * THE TWO HALVES MUST MOVE TOGETHER with the host-side declaration in
	 * instrument/RTL9603CVD/LANLY/G24W/swcore_regs.py: a span declared there and
	 * not printed here comes back ABSENT, and the reader renders an absent word
	 * inside a covered block as ZERO -- a device finding manufactured by our own
	 * instrument.
	 * Worth the reads because a register outside every dumped range can never be
	 * compared against stock, so no measurement can disagree with us: that is how
	 * ACCEPT_MAX_LEN_CTRL kept 2031 on a die whose stock writes 16368.
	 * They are NOT appended to sw[]/gt[] because every word here is proven
	 * readable on the RTL9603CVD and on nothing else -- a gap inside a dumped
	 * window bus-faults, and the RTL9602C's per-port MAC stride is 0x400, so
	 * 0x20100 is not a port block there at all.
	 * TEN READ-TO-CLEAR WORDS ARE DELIBERATELY ABSENT, and their absence is not a
	 * hole: reading them CONSUMES what another reader needs. The US/DS/AES INTR
	 * deltas are what us_intr_svc exists to service and that parameter defaults
	 * OFF, so dumping them would perform on every `cat` the ack this driver
	 * deliberately does not; the five indirect counter STATs are read by this
	 * driver's own /proc/gpon, so dumping them would steal the count. Each one
	 * SPLITS its run; none is bridged. */
	static const u32 xsw9603[][2] = {	/* extra SWCORE spans, RTL9603CVD only */
		{0x10000, 0x1000c},	/*   4  MODEL_NAME_INFO/CHIP_INFO */
		{0x11058, 0x110c0},	/*  27  OUTPUT_DROP_CFG/OUTPUT_DROP_EN */
		{0x110d4, 0x110dc},	/*   3  PTP_P_EN */
		{0x12010, 0x1202c},	/*   8  TBL_ACCESS_RD_DATA */
		{0x13000, 0x13020},	/*   9  VLAN_PORT_ACCEPT_FRAME_TYPE/VLAN_INGRESS */
		{0x13040, 0x130e0},	/*  41  VLAN_PPB_VLAN_VAL/VLAN_PORT_PPB_VLAN */
		{0x14000, 0x1401c},	/*   8  SVLAN_P_SVID/SVLAN_EXT_SVID */
		{0x15008, 0x1518c},	/*  98  ACL_TEMPLATE_CTRL/ACL_EN */
		{0x15450, 0x15470},	/*   9  RNG_CHK_PKTLEN_RNG/RGF_VER_ALE_ACL */
		{0x17000, 0x171b8},	/* 111  LUT_CFG/LUT_AGEOUT_CTRL */
		{0x18000, 0x18000},	/*   1  RGF_VER_ALE_MLTVLAN */
		{0x1b008, 0x1b008},	/*   1  PTP_TIME_NSEC */
		{0x1b014, 0x1b020},	/*   4  PTP_TIME_OFFSET_8NSEC/PTP_TIME_FREQ */
		{0x1b02c, 0x1b02c},	/*   1  PTP_PON_TOD_NSEC */
		{0x1b038, 0x1b078},	/*  17  RGF_VER_ALE_EAV_AFBK/RSVD_ALE_EAV_AFBK */
		{0x1c458, 0x1c460},	/*   3  RGF_VER_ALE_DPM/RSVD_ALE_DPM */
		{0x1d000, 0x1d010},	/*   5  INTR_CTRL/INTR_DBGO_POS */
		{0x1d01c, 0x1d030},	/*   6  INTR_STAT_POS/INTR_STAT_NEG */
		{0x1e068, 0x1e070},	/*   3  LED_EN/SERI_LED_CLK_PER */
		{0x1e078, 0x1e078},	/*   1  PON_LED_CFG */
		{0x1e090, 0x1e09c},	/*   4  LED_FLT_MPCP/RGF_VER_LED */
		{0x1f000, 0x1f034},	/*  14  PHY_RG0X_CEN/PHY_RG1X_CEN */
		{0x1f040, 0x1f090},	/*  21  PHY_RG0X_PLL/PHY_RG1X_PLL */
		{0x20000, 0x20030},	/*  13  P_TX_ERR_CNT/P_MISC */
		{0x20100, 0x20130},	/*  13   */
		{0x20200, 0x20230},	/*  13   */
		{0x20300, 0x20330},	/*  13   */
		{0x22000, 0x220dc},	/*  56  EXTG_ACTYPE0/EXTG_ACTYPE1 */
		{0x23120, 0x23140},	/*   9  FC_PON_GLB_HI_TH/FC_PON_GLB_LO_TH */
		{0x23154, 0x231b0},	/*  24  SPG_GLB_CTRL/FC_SWPBO_GLB_HI_TH */
		{0x24000, 0x241fc},	/* 128  EXTHDR_DAT */
		{0x2518c, 0x2519c},	/*   5  METER_PKT_RATE/RGF_VER_ALE_METER */
		{0x26000, 0x26014},	/*   6  DOS_EN/DOS_CFG */
		{0x28040, 0x280a8},	/*  27  HSB_DATA */
		{0x280c0, 0x280c8},	/*   3  HSB_PARSER */
		{0x28100, 0x28158},	/*  23  HSA_DATA */
		{0x28164, 0x28234},	/*  53  HSM_DATA/FBHSA_DATA */
		{0x2a000, 0x2a120},	/*  73  VLAN_EGRESS_TAG/IP4MC_EGRESS_MODE */
		{0x2a130, 0x2a1a0},	/*  29  RMK_P_DSCP_SEL/RMK_P_1P_SEL */
		{0x2d000, 0x2d010},	/*   5  LOW_QUEUE_TH/HIGH_QUEUE_MSK */
		{0x2d020, 0x2d14c},	/*  76  FC_P_EGR_DROP_TH/FC_DBG_CTRL */
		{0x2d800, 0x2d88c},	/*  36  WFQ_CTRL/EGR_BWCTRL_P_CTRL */
		{0x2d8c0, 0x2d8f8},	/*  15  MOCIR_BPT/BYTE_TOKEN_METER */
		{0x31000, 0x3105c},	/*  24  SW_BIST_CFG_3/SW_BIST_CFG_6 */
		{0x31064, 0x310c8},	/*  26  SW_BIST_CFG_OQ_0/SW_BIST_CFG_OQ_1 */
		{0x32c80, 0x32f50},	/* 181  STAT_ACL_CNT/DOT3_Q_TX_FRAMES */
		{0x34000, 0x34000},	/*   1  STAT_CTRL */
		{0x34014, 0x3401c},	/*   3  STAT_PORT_RST/STAT_RST */
		{0x34034, 0x34034},	/*   1  RGF_VER_MIB_CTRL */
		{0x36000, 0x36038},	/*  15  EPON_FEC_CONFIG/EPON_ASIC_TIMING_ADJUST1 */
		{0x3609c, 0x360a4},	/*   3  EPON_MPCP_CTR/EPON_TX_CTRL */
		{0x360e8, 0x36108},	/*   9  EPON_SCB_DECRYP_KEY0/EPON_SCB_DECRYP_KEY1 */
		{0x36120, 0x361b8},	/*  39  EPON_SCH_TIMING/EPON_FEC_RST */
	};
	static const u32 xgt9603[][2] = {	/* extra GTC spans, RTL9603CVD only    */
		{0x00020, 0x00024},	/*   2  GPON_AES_BYPASS/GPON_FRAME_PULSE_CFG */
		{0x00040, 0x00044},	/*   2  GPON_INTR_MASK/GPON_INTR_STS */
		{0x00280, 0x00280},	/*   1  GPON_MAC_BIST_RSLT */
		{0x010cc, 0x010cc},	/*   1  GPON_GTC_DS_ALLOC_RD */
		{0x01100, 0x01104},	/*   2  GPON_GTC_DS_PORT_IND/GPON_GTC_DS_PORT_WR */
		{0x0110c, 0x0110c},	/*   1  GPON_GTC_DS_PORT_RD */
		{0x01140, 0x01140},	/*   1  GPON_GTC_DS_PORT_CNTR_IND */
		{0x01240, 0x01244},	/*   2  GPON_GTC_DS_BPOS_SUB/GPON_GTC_DS_BPOS_ADD */
		{0x01260, 0x0126c},	/*   4  GPON_GTC_DS_DUMMY_1/GPON_GTC_DS_DUMMY_2 */
		{0x01280, 0x0128c},	/*   4  DS_FEC/PLOAM */
		{0x01294, 0x012a4},	/*   5  BWMAP_CAP/BWMAP_BUF */
		{0x0200c, 0x02010},	/*   2  GPON_BWMAP_CTRL/GPON_BWMAP_STS */
		{0x03004, 0x03008},	/*   2  GPON_AES_INTR_MASK/GPON_AES_INTR_STS */
		{0x03280, 0x03290},	/*   5  CTL_INFO/CTL_DATA */
		{0x04040, 0x04040},	/*   1  GPON_GEM_DS_RX_CNTR_IND */
		{0x0404c, 0x0404c},	/*   1  GPON_GEM_DS_FWD_CNTR_IND */
		{0x04098, 0x040a0},	/*   3  GPON_GEM_DS_FRM_TIMEOUT/GPON_GEM_DS_MC_ADDR_PTN_IPV4 */
		{0x05004, 0x05008},	/*   2  GPON_GTC_US_INTR_MASK/GPON_GTC_US_INTR_STS */
		{0x05180, 0x05188},	/*   3  GPON_GTC_US_RDI/GPON_GTC_US_DG */
		{0x05260, 0x05270},	/*   5  G_DMY_XX_01/G_DMY_XX_02 */
		{0x05280, 0x05294},	/*   6  BWM_TBL/BWM_MEM0 */
		{0x06004, 0x06008},	/*   2  GPON_GEM_US_INTR_MASK/GPON_GEM_US_INTR_STS */
		{0x06048, 0x06048},	/*   1  GPON_GEM_US_ETH_GEM_RX_CNTR_IDX */
		{0x06054, 0x06054},	/*   1  GPON_GEM_US_PTN_CTRL */
		{0x06280, 0x06294},	/*   6  PCFG/GEM_BCNT */
	};
	const u32 (*xsw)[2] = NULL, (*xgt)[2] = NULL;
	int nxsw = 0, nxgt = 0;
	u32 off, a, val;
	int r;

	/* The per-die extras, selected by the SAME pointer every other per-chip
	 * fact in this driver comes from.  Any other die keeps NULL/0 and dumps
	 * exactly what it dumped before. */
	if (swc == &gpon_swc_9603cvd) {
		xsw = xsw9603;
		nxsw = (int)ARRAY_SIZE(xsw9603);
		xgt = xgt9603;
		nxgt = (int)ARRAY_SIZE(xgt9603);
	}

	/*
	 * The dump declares its own COVERAGE first. Words are zero-suppressed, so a
	 * reader cannot otherwise tell a register that read zero from one this node
	 * never looked at -- and that answer decides whether a stock-vs-ours row is a
	 * finding or an artefact. Measured the day this was written: the host-side
	 * reader filled every word it did not see with 0 and printed
	 * `0x23024 STRAP_CFG stock 00040000 ours 00000000` as a DIFFERENCE, with the
	 * silicon having nothing to do with it. Stated here, by the code that owns
	 * it, rather than in a host-side table that can drift.
	 */
	for (r = 0; r < (int)ARRAY_SIZE(sw); r++)
		seq_printf(s, "#cover 0x%08x 0x%08x\n",
			   0x1b000000u + sw[r][0], 0x1b000000u + sw[r][1]);
	for (r = 0; r < (int)ARRAY_SIZE(gm); r++)
		seq_printf(s, "#cover 0x%08x 0x%08x\n", gm[r][0], gm[r][1]);
	for (r = 0; r < (int)ARRAY_SIZE(gt); r++)
		seq_printf(s, "#cover 0x%08x 0x%08x\n",
			   0x1b700000u + gt[r][0], 0x1b700000u + gt[r][1]);
	for (r = 0; r < nxsw; r++)
		seq_printf(s, "#cover 0x%08x 0x%08x\n",
			   0x1b000000u + xsw[r][0], 0x1b000000u + xsw[r][1]);
	for (r = 0; r < nxgt; r++)
		seq_printf(s, "#cover 0x%08x 0x%08x\n",
			   0x1b700000u + xgt[r][0], 0x1b700000u + xgt[r][1]);
	seq_printf(s, "#cover 0x%08x 0x%08x\n",
		   0x1b000000u + SDS(0x22000), 0x1b000000u + SDS(0x22ffc));

	/* The SerDes page is per chip, so its range must be DERIVED. It was the
	 * literal {0x40000, 0x40ffc} -- the RTL9603CVD's window -- while on the
	 * RTL9602C the block is at 0x22000, so on that board this node read something
	 * that is not the SerDes at all and every question its own declaration asked
	 * came back NOT CAPTURED (measured 2026-09-01: "208 word(s) DECLARED but NOT
	 * COVERED"). Same class as the raw-offset tables: a per-chip window written
	 * as a literal is correct on exactly one die. SDS() is the translation the
	 * driver already has. */
	for (off = SDS(0x22000); off <= SDS(0x22ffc); off += 4)
		seq_printf(s, "0x%08x %08x\n", 0x1b000000u + off, sw_rd(off));

	for (r = 0; r < (int)ARRAY_SIZE(gt); r++)
		for (off = gt[r][0]; off <= gt[r][1]; off += 4)
			seq_printf(s, "0x%08x %08x\n", 0x1b700000u + off,
				   gpon_rd(off));
	for (r = 0; r < (int)ARRAY_SIZE(sw); r++)
		for (off = sw[r][0]; off <= sw[r][1]; off += 4) {
			val = sw_rd(off);
			if (val)
				seq_printf(s, "0x%08x %08x\n", 0x1b000000u + off, val);
		}
	for (r = 0; r < nxgt; r++)
		for (off = xgt[r][0]; off <= xgt[r][1]; off += 4)
			seq_printf(s, "0x%08x %08x\n", 0x1b700000u + off,
				   gpon_rd(off));
	for (r = 0; r < nxsw; r++)
		for (off = xsw[r][0]; off <= xsw[r][1]; off += 4) {
			val = sw_rd(off);
			if (val)
				seq_printf(s, "0x%08x %08x\n", 0x1b000000u + off, val);
		}
	for (r = 0; r < (int)ARRAY_SIZE(gm); r++) {
		void __iomem *b = ioremap(gm[r][0], gm[r][1] - gm[r][0] + 4);

		if (!b)
			continue;
		for (a = gm[r][0]; a <= gm[r][1]; a += 4) {
			val = ioread32(b + (a - gm[r][0]));
			if (val)
				seq_printf(s, "0x%08x %08x\n", a, val);
		}
		iounmap(b);
	}
	seq_puts(s, "0xffffffff ffffffff\n");	/* end marker */
	return 0;
}

/* Serialize complete indirect transactions, including readback. A timed-out
 * CAM retains its own pending state after the lock is released. */
static DEFINE_SPINLOCK(gpon_ind_lock);
static bool gpon_cam_pending[2];

static int luna_cam_xact(bool alloc, u32 mode, u32 index, u16 *value, bool *hit)
{
	const struct gpon_gtc_regs *r = &luna_gpon_chip.gtc;
	u32 ind = alloc ? r->ds_alloc_ind : r->ds_port_ind;
	u32 wr = alloc ? r->ds_alloc_wr : r->ds_port_wr;
	u32 rd = alloc ? r->ds_alloc_rd : r->ds_port_rd;
	u32 mask = alloc ? r->ds_alloc_idx_mask : r->ds_port_idx_mask;
	unsigned long flags;
	int rc;

	if (index > mask || (mode == GPON_GTC_CAM_OP_READ && (!value || !hit)))
		return -EINVAL;
	if (mode == GPON_GTC_CAM_OP_READ && !reg_has(rd))
		return -ENODEV;
	if (alloc)
		index = luna_tcont_cam_index(index);
	spin_lock_irqsave(&gpon_ind_lock, flags);
	rc = gpon_gtc_cam_xact_pending(&gpon_io, ind, mode, mask, index, wr,
				       mode == GPON_GTC_CAM_OP_WRITE && value ? *value : 0,
				       gpon_cam_delay_us,
				       &gpon_cam_pending[alloc]);
	if (rc >= 0 && mode == GPON_GTC_CAM_OP_READ) {
		*hit = !!(hwio_rd(&gpon_io, ind) & GPON_GTC_CAM_OP_HIT);
		*value = hwio_rd(&gpon_io, rd) & GPON_GTC_CAM_VAL_MASK;
	}
	spin_unlock_irqrestore(&gpon_ind_lock, flags);
	return rc < 0 ? rc : 0;
}

static int luna_port_cam_write(u32 flow, u16 gem)
{
	return luna_cam_xact(false, GPON_GTC_CAM_OP_WRITE, flow, &gem, NULL);
}

static int luna_alloc_cam_write(u32 tcont, u16 alloc)
{
	return luna_cam_xact(true, GPON_GTC_CAM_OP_WRITE, tcont, &alloc, NULL);
}

/* Per-flow downstream GEM Ethernet RX packet count (clear-on-read). Diagnoses
 * whether the OLT is sending ANY DS GEM frames on a flow.
 * One indirect read through the shared gpon_gtc_gem_ds_rx_cnt_read() reading
 * luna_gpon_chip; same addresses, values and access counts as the hand-spelled
 * form, proven by gpon_regtable_diff_test.
 * The pre-conversion copy was doubly wrong: it reached the register through
 * PON-IP-block names whose offsets coincide with these GTC registers, so the code
 * read as a poll of the US NIC config; and it returned STAT after an EXPIRED poll,
 * which is stale data dressed as a count -- and since STAT is clear-on-read the
 * timed-out read also STOLE the count from the next reader. */
static u32 gpon_gem_ds_rx_cnt(u8 flow)
{
	u32 cnt;
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&gpon_ind_lock, flags);
	rc = gpon_gtc_gem_ds_rx_cnt_read(&gpon_io, &luna_gpon_chip.gtc, flow,
					 &cnt, gpon_cam_delay_us);
	spin_unlock_irqrestore(&gpon_ind_lock, flags);
	return rc < 0 ? 0xffffffffu : cnt;
}

/* Per-flow downstream GEM FORWARDED-to-PON-IP count (clear-on-read), same
 * indirect protocol. GEM_NON_IDLE is a GLOBAL de-assembler counter, so its rise
 * only proves the GTC de-encapsulated SOMETHING; FWD localises the break -- if the
 * flow count climbs but FWD stays 0 the GTC de-encapsulated and did NOT forward,
 * and if FWD climbs while the PON-IP DS SRAM stays flat the PON-IP rejected it. */
static u32 gpon_gem_ds_fwd_cnt(u8 flow)
{
	u32 cnt;
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&gpon_ind_lock, flags);
	rc = gpon_gtc_gem_ds_fwd_cnt_read(&gpon_io, &luna_gpon_chip.gtc, flow,
					 &cnt, gpon_cam_delay_us);
	spin_unlock_irqrestore(&gpon_ind_lock, flags);
	return rc < 0 ? 0xffffffffu : cnt;
}

/* Read back the DS GEM-port CAM entry for `flow`: does the CAM actually hold the
 * OMCC gem at runtime?
 * One indirect-CAM transaction through the shared gpon_gtc_ds_port_read().
 * Return: >= 0 -- [11:0] stored gem, BIT(16) OP_HIT; < 0 -- the READ never
 * completed. The pre-conversion copy read RDATA back after an EXPIRED poll,
 * returning whatever the last completed transaction left there, so a caller MUST
 * check for < 0 before trusting the entry. */
static int gpon_ds_cam_read(u8 flow)
{
	u16 gem;
	bool hit;
	int rc = luna_cam_xact(false, GPON_GTC_CAM_OP_READ, flow, &gem, &hit);

	if (rc < 0)
		return rc;
	return (hit ? BIT(16) : 0) | gem;
}

/* Invalidate non-management DS GEM-port CAM entries (CLEAN op = OP_MODE 3). The CAM holds
 * only a 12-bit gemPortId per entry with NO valid bit, so at reset the entries carry
 * GARBAGE gem values (observed e0=gem3566). The lookup matches an incoming gem against
 * every entry, so a stale entry that happens to equal the OMCC gem (2) shadows flow 64
 * and steals the DS OMCI — flow 64 then de-encaps nothing. Clear them all before
 * installing the OMCC so ONLY flow 64 matches gem 2. */
static int gpon_ds_cam_clear_all(void)
{
	unsigned int f;

	for (f = 0; f < 128; f++) {
		int rc;

		if (f == GPON_OMCC_FLOW)
			continue;
		rc = luna_cam_xact(false, GPON_GTC_CAM_OP_CLEAN, f, NULL, NULL);
		if (rc)
			return rc;
		gpon_field(GPON_GTC_DS_TRAFFIC_CFG + f * DS_TRAFFIC_CFG_STRIDE, 4, 0, 0);
	}
	return 0;
}

/* GTC US MISC PM counter (GPON_GTC_US_MISC_CNTR_IDX 0x5140 [2:0] / STAT 0x5148):
 * write the raw type index, read the 32-bit count. raw idx map (stock US-GTC
 * misc-counter type mapping): 0=PLOAM_BOH_TX 1=GEM_DBRU_TX 2=PLOAM_CPU_TX
 * 3=PLOAM_AUTO_TX 4=GEM_BYTE_TX. Decisive US-emission instrument: idx2 (our ACK/SN
 * PLOAM, expect >0) vs idx4 GEM_BYTE_TX + TCONT_IDLE_BYTE_STAT[16] (expect 0 = ONU
 * transmits PLOAM but NO US GEM on its T-CONT-16 grants -> OLT deactivates ~42s). */
static u32 gpon_us_misc_cnt(u8 idx)
{
	unsigned long flags;
	u32 value;
	spin_lock_irqsave(&gpon_ind_lock, flags);
	gpon_wr(0x5140, idx & 0x7);
	udelay(5);
	value = gpon_rd(0x5148);
	spin_unlock_irqrestore(&gpon_ind_lock, flags);
	return value;
}

/* GEM DS MISC PM counter (clear-on-read) -- GLOBAL, CAM-independent de-encap
 * stage counters: 0=MC_RX 1=UC_RX 2=MC_FWD 3=MC_LEAK 4=ETH_CRC_ERR
 * 5=OVER_INTERLEAV 6=OMCI_RX. UC_RX counts every DS unicast GEM the de-assembler
 * accepts regardless of the per-port CAM, and OMCI_RX is the authoritative "the
 * GTC de-encapsulated an OMCI frame" count: UC_RX climbing with OMCI_RX at 0 means
 * unicast arrives but is not PTI-classified as OMCI.
 *
 * NOT the shared gpon_gtc_cntr_read() transaction, deliberately: the "R_ACK bit15"
 * this comment claimed for months is refuted by three sources -- both chipdefs
 * declare NO ack field here, the vendor DAL reads this counter with no poll at
 * all, and stock's own resting value has bit 15 CLEAR while all three real counter
 * INDs latch theirs SET. So the poll below tests a reserved bit, always expires,
 * and is in effect a 16x2 us settle, with the fall-through STAT read as the
 * LOAD-BEARING path. Routing it through the shared helper would return "failed" on
 * every call. Kept byte-identical on the bus pending a tier-1 measurement of
 * whether the settle is needed at all. */
static u32 gpon_gem_ds_misc_cnt(u8 idx)
{
	unsigned long flags;
	u32 value;
	int i;

	spin_lock_irqsave(&gpon_ind_lock, flags);
	gpon_wr(GPON_GEM_DS_MISC_IND, idx & 0xf);
	for (i = 0; i < 16; i++) {
		if (gpon_rd(GPON_GEM_DS_MISC_IND) & BIT(15))
			break;
		udelay(2);
	}
	value = gpon_rd(GPON_GEM_DS_MISC_CNTR_STAT);
	spin_unlock_irqrestore(&gpon_ind_lock, flags);
	return value;
}

/* Read back the GTC alloc CAM entry for a T-CONT (READ op, same indirect protocol
 * as the DS GEM CAM): does T-CONT 16 actually hold alloc 0x400 + HIT? If the BWMAP's
 * alloc-id does not match a valid CAM entry, the GTC ignores the operational grant
 * (bwm_acpt stays 0) and the ONU never transmits operational US.
 * Shared transaction (gpon_gtc_ds_alloc_read, regtable.h) since 2026-09-03.
 * Return: >= 0 -- [11:0]=stored allocId, BIT(16)=OP_HIT; < 0 -- the READ never
 * completed and there IS no value (the old copy returned stale RDATA here). */
static int gpon_alloc_cam_read(u8 tcont)
{
	u16 alloc;
	bool hit;
	int rc = luna_cam_xact(true, GPON_GTC_CAM_OP_READ, tcont, &alloc, &hit);

	if (rc < 0)
		return rc;
	return (hit ? BIT(16) : 0) | alloc;
}

/* Invalidate every allocation except the management T-CONT before activation.
 * Alloc-ID 4095 is assignable: writing 0xfff is not an invalidation. Both own
 * stock dies use CLEAN, with the CVD logical-index permutation in luna_cam_xact.
 * A failed audit/clean stops the caller before it publishes a new assignment. */
static int gpon_alloc_cam_clear_others(u8 keep)
{
	u8 t;
	int rc;

	if (keep >= 32)
		return -EINVAL;
	rc = luna_data_retire();
	if (rc)
		return rc;
	for (t = 0; t < 32; t++) {
		int rb, rc;

		if (t == keep)
			continue;
		rb = gpon_alloc_cam_read(t);
		if (rb < 0)
			return rb;
		if (rb & BIT(16)) {
			u8 dmp[4] = { t, keep, (u8)((rb >> 8) & 0xf), (u8)rb };

			gpon_unsup_report("alloc_cam_stale", GPON_UNSUP_RANGE,
					  rb & 0xfff,
					  "no-hit-outside-kept-tcont", dmp, sizeof(dmp));
		}
		rc = luna_cam_xact(true, GPON_GTC_CAM_OP_CLEAN, t, NULL, NULL);
		if (rc)
			return rc;
	}
	return 0;
}

/* DS-pipeline stage A: per-flow de-encapsulated GEM frame count (clear-on-read).
 * Unlike the ETH-only gem_ds_rx_cnt this counts ALL de-encapped GEM frames
 * including the isOMCI flow -- the reliable "did the GTC de-encapsulate OMCI?"
 * detector. Shared transaction (gpon_gtc_ds_port_cnt_read).
 * It was the ONLY one of the four counter sites that already refused its timeout,
 * through the 0xffffffff sentinel its /proc consumers decode structurally; the
 * sentinel stays because those consumers print counts as u32, and the shared
 * helper is what now guarantees STAT is not read -- and therefore not CLEARED --
 * on the timeout path. */
static u32 gpon_gem_flow_cnt(u32 idx, int rsel)
{
	u32 cnt;
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&gpon_ind_lock, flags);
	rc = gpon_gtc_ds_port_cnt_read(&gpon_io, &luna_gpon_chip.gtc, idx,
				      (rsel & 1) != 0, &cnt,
				      gpon_cam_delay_us);
	spin_unlock_irqrestore(&gpon_ind_lock, flags);
	return rc < 0 ? 0xffffffffu : cnt;
}

static int gpon_proc_show_locked(struct seq_file *s, void *v);
static int gpon_proc_show(struct seq_file *s, void *v)
{
	int ret;

	mutex_lock(&bosa_lock);
	ret = luna_stopping ? -ESHUTDOWN : gpon_proc_show_locked(s, v);
	mutex_unlock(&bosa_lock);
	return ret;
}

static int gpon_proc_show_locked(struct seq_file *s, void *v)
{
	u32 rst    = gpon_rd(GPON_RESET);
	u32 status = gpon_rd(GPON_GTC_DS_ONU_ID_STATUS);
	u32 eqd    = gpon_rd(GPON_GTC_US_EQD);
	u32 state  = status & GPON_ONU_STATE_MASK;

	luna_omci_seq_show(s);
	if (is_9607c)
		luna_c7_diag(&rtl9602c_r960_ops, s);
	seq_printf(s, "optics: backend=%s gn_cal_ready=%u upstream_ready=%u error=%d\n",
		bosa_regs_live() ? "RTL8290B" : bosa_gn_identified ? "GN25L95" : "unknown",
		bosa_cal_ready, luna_activation_ready, bosa_cal_error);
	seq_printf(s, "version:     0x%02x\n", gpon_rd(GPON_VERSION) & GPON_VER_ID_MASK);
	seq_printf(s, "reset:       0x%08x (soft_rst=%d rst_done=%d)\n",
		   rst, !!(rst & GPON_SOFT_RST), !!(rst & GPON_RST_DONE));
	seq_printf(s, "onu_state:   O%u (%s)\n", state,
		   state < ARRAY_SIZE(gpon_onu_state_name) &&
		   gpon_onu_state_name[state] ? gpon_onu_state_name[state] : "?");
	/* fiber: one-glance fibre-pull / optical recovery verdict. rerange = LOS/deact
	 * recoveries since boot plus the last outage duration; the DATA-PLANE VERDICT
	 * (omcc + DATA GEM installed) answers the question when internet does not
	 * return after a reconnect -- the link re-ranged to O5, but did the WAN data
	 * path rebuild? RX dBm near sensitivity means a marginal link. */
	{
		u32 los = gpon_rd(GPON_GTC_DS_LOS_CFG_STS);
		s32 rx_cdbm = bosa_rx_power_cdbm();	/* calibrated ratiometric RX (was a linear 0x311 fit) */
		char rx_s[16], sdet_s[8];

		/* ★ NEITHER FIELD MAY PRINT A NUMBER IT DID NOT MEASURE. rx was a
		 * hard floor constant (-29.85 dBm) on every failed I2C read, and
		 * sdet was a read of whatever the 9602C's SDS_FIB_STATUS offset
		 * happens to be on this chip. Three states each: value / n/a. */
		if (rx_cdbm == BOSA_RX_CDBM_NA)
			strscpy(rx_s, "n/a", sizeof(rx_s));
		else
			scnprintf(rx_s, sizeof(rx_s), "%d.%02ddBm", rx_cdbm / 100,
				  (rx_cdbm < 0 ? -rx_cdbm : rx_cdbm) % 100);
		if (SDS_FIB_STATUS)
			scnprintf(sdet_s, sizeof(sdet_s), "%d",
				  !!(sw_rd(SDS_FIB_STATUS) & SDS_FIB_SDS_SDET));
		else
			strscpy(sdet_s, "n/a", sizeof(sdet_s));

		/* The two OLT-ASSIGNED identities of the WAN datapath are printed
		 * beside their install flags: without them a "solicited=1
		 * DATA_GEM_inst=1" line cannot tell a correct bind from a bind to
		 * a retired Alloc-ID or to another board's gem-port. */
		seq_printf(s, "fiber:       rerange=%u last_outage=%ums | optic_los=%d sdet=%s rx=%s | omcc_inst=%d DATA_GEM_inst=%d solicited=%d gem=%u alloc=0x%x tcont_bound=%d\n",
			   gpon_rerange_cnt, gpon_last_outage_ms,
			   !!(los & GPON_OPTIC_LOS_SIG),
			   sdet_s, rx_s,
			   gpon_omcc_installed, gpon_data_installed, gpon_data_gem_solicited,
			   gpon_data_gem_port, gpon_data_alloc, gpon_data_tcont_installed);
	}
	seq_printf(s, "onu_id:      %u\n",
		   (status >> GPON_ONU_ID_SHIFT) & GPON_ONU_ID_MASK);
	seq_printf(s, "eqd:         inframe=%u multiframe=%u\n",
		   eqd & GPON_EQD_INFRAME_MASK,
		   (eqd >> GPON_EQD_MF_SHIFT) & GPON_EQD_MF_MASK);
	seq_printf(s, "min_delay:   0x%08x\n", gpon_rd(GPON_GTC_US_MIN_DELAY));
	seq_printf(s, "us_cfg:      0x%08x (ben_polar=%u scrm_dis=%u plm_dis=%u)\n",
		   gpon_rd(GPON_GTC_US_CFG),
		   (gpon_rd(GPON_GTC_US_CFG) >> 3) & 1,
		   gpon_rd(GPON_GTC_US_CFG) & 1,
		   (gpon_rd(GPON_GTC_US_CFG) >> 9) & 1);
	seq_printf(s, "us_laser:    0x%08x (lon=%u loff=%u)\n",
		   gpon_rd(GPON_GTC_US_LASER),
		   (gpon_rd(GPON_GTC_US_LASER) >> 8) & 0x3f,
		   gpon_rd(GPON_GTC_US_LASER) & 0x3f);
	{
		int i;

		seq_printf(s, "boh_cfg:     0x%08x (repeat=%u length=%u) data=",
			   gpon_rd(GPON_GTC_US_BOH_CFG),
			   (gpon_rd(GPON_GTC_US_BOH_CFG) >> 8) & 0xf,
			   gpon_rd(GPON_GTC_US_BOH_CFG) & 0xff);
		for (i = 0; i < GPON_BOH_LEN; i++)
			seq_printf(s, "%02x", gpon_rd(GPON_GTC_US_BOH_DATA + i * 4) & 0xff);
		seq_puts(s, "\n");
	}
	seq_printf(s, "test:        0x%08x\n", gpon_rd(GPON_TEST));
	seq_printf(s, "intr_mask:   0x%08x\n", gpon_rd(GPON_INTR_MASK));
	seq_printf(s, "intr_sts:    0x%08x\n", gpon_rd(GPON_INTR_STS));
	seq_printf(s, "gtc_ds_dlt:  0x%08x\n", gpon_rd(GPON_GTC_DS_INTR_DLT));
	seq_printf(s, "gtc_ds_mask: 0x%08x\n", gpon_rd(GPON_GTC_DS_INTR_MASK));
	seq_printf(s, "gtc_ds_sts:  0x%08x\n", gpon_rd(GPON_GTC_DS_INTR_STS));
	seq_printf(s, "cdr_recover: wedged=%u fixed=%u last_sts=0x%08x (sentinel=0x%08x)\n",
		   gpon_cdr_stuck_count, gpon_cdr_stuck_fixed, gpon_gtc_ds_sts_last,
		   GTC_DS_CDR_STUCK);
	{
		u32 los = gpon_rd(GPON_GTC_DS_LOS_CFG_STS);

		/* ★ cdr_los IS ONLY A STATUS WHILE ITS MONITOR IS ON. The vendor's
		 * chip-agnostic default leaves CDR_LOS_EN (bit 2) CLEAR -- measured
		 * on the G24W: los_cfg_sts=0x103, i.e. bits 0,1,8 set and bit 2
		 * clear -- so cdr_los printed the state of a DISABLED comparator as
		 * though it were a reading. Say which it is. */
		seq_printf(s, "los_cfg_sts: 0x%08x (optic_los=%d cdr_los=%s en=%d polar=%d)\n",
			   los, !!(los & GPON_OPTIC_LOS_SIG),
			   (los & GPON_CDR_LOS_EN)
				? ((los & GPON_CDR_LOS_SIG) ? "1" : "0")
				: "disabled",
			   !!(los & GPON_OPTIC_LOS_EN),
			   !!(los & GPON_OPTIC_LOS_POLAR));
	}

	/*
	 * PLOAM channel view, read-only. With no downstream PLOAM and nothing queued
	 * upstream the buffers must read empty and the assigned ONU-ID 255 -- a
	 * self-consistency check that validates the offsets before a live OLT.
	 */
	{
		u32 ds_ind  = gpon_rd(GPON_GTC_DS_PLOAM_IND);
		u32 us_ind  = gpon_rd(GPON_GTC_US_PLOAM_IND);
		u32 us_cfg  = gpon_rd(GPON_GTC_US_PLOAM_CFG);
		u32 us_onu  = gpon_rd(GPON_GTC_US_ONU_ID);

		seq_printf(s, "ds_ploam_ind:  0x%08x (buf_empty=%d buf_full=%d)\n",
			   ds_ind, !!(ds_ind & GPON_DS_PLM_BUF_EMPTY),
			   !!(ds_ind & GPON_DS_PLM_BUF_FULL));
		seq_printf(s, "us_ploam_ind:  0x%08x (nrm_empty=%d nrm_full=%d urg_empty=%d urg_full=%d)\n",
			   us_ind, !!(us_ind & GPON_US_PLM_NRM_EMPTY),
			   !!(us_ind & GPON_US_PLM_NRM_FULL),
			   !!(us_ind & GPON_US_PLM_URG_EMPTY),
			   !!(us_ind & GPON_US_PLM_URG_FULL));
		seq_printf(s, "us_ploam_cfg:  0x%08x (crc_gen=%d onuid_ovrd=%d)\n",
			   us_cfg, !!(us_cfg & GPON_US_PLM_CRC_GEN_EN),
			   !!(us_cfg & GPON_US_PLM_ONUID_OVRD));
		seq_printf(s, "us_onu_id:     %u\n",
			   (us_onu >> GPON_GTC_US_ONU_ID_SHIFT) & GPON_ONU_ID_MASK);
		seq_printf(s, "ds_ploam_cfg:  0x%08x\n",
			   gpon_rd(GPON_GTC_DS_PLOAM_CFG));
	}

	/*
	 * DS framer config against its O5 operating values. A mismatch in ds_cfg is
	 * the prime suspect for "configured correctly yet will not frame-lock".
	 */
	seq_printf(s, "gtc_cfg: ds_cfg=0x%08x intr_mask=0x%08x r1048=0x%08x r104c=0x%08x r1050=0x%08x\n",
		   gpon_rd(GPON_GTC_DS_CFG), gpon_rd(GPON_GTC_DS_INTR_MASK), gpon_rd(GPON_GTC_DS_SUPERFRAME_CNT),
		   gpon_rd(GPON_GTC_DS_TOD_SUPERFRAME_CTRL), gpon_rd(0x1050));
	/* DS_MISC counters (GTC-relative, register-map base 0x7011xx): do we even SEE/accept the
	 * OLT's BWmap grants + DS PLOAMs? ploam_acpt/bwm_acpt nonzero => GTC recognizes
	 * grants and asserts BEN (so a zero at the OLT = analog SerDes-TX emission);
	 * bwm_fail/inv nonzero => grants seen but rejected (CRC/format); all zero =>
	 * GTC never sees the OLT grants (downstream BWmap parse issue). */
	{
		/* `active=` used to print this word raw: it is two 16-bit counters,
		 * the SN requests and the RANGING requests the GTC received (chipdef
		 * field names), and the split is what decides an activation fork. */
		u32 act = gpon_rd(GPON_GTC_DS_MISC_CNTR_ACTIVE);

		seq_printf(s, "ds_cntr: ploam_acpt=%u ploam_fail=%u bwm_acpt=%u bwm_fail=%u bwm_inv=%u sn_req=%u rng_req=%u (all clear-on-read)\n",
			   gpon_rd(GPON_GTC_DS_MISC_CNTR_PLOAM_ACPT), gpon_rd(GPON_GTC_DS_MISC_CNTR_PLOAM_FAIL), gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_ACPT),
			   gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_FAIL), gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_INV),
			   GPON_GTC_DS_ACTIVE_SN_REQ(act), GPON_GTC_DS_ACTIVE_RNG_REQ(act));
	}
	/* ★ THE FLOW IS THE CHIP'S, AND THE LABEL PRINTS IT (2026-09-08).  This read
	 * and its label both said 64 -- the RTL9602C's OMCI flow -- so on the
	 * RTL9603CVD (whose OMCI flow is 127) the decisive "is the OLT sending us
	 * DS OMCI" witness was reading a flow nothing is bound to and reporting 0.
	 * A witness pinned to the wrong index reads as a DEVICE finding. */
	seq_printf(s, "gem_ds_rx: omcc(f%u)=%u f0=%u f1=%u f2(mcast)=%u  (>0 => OLT is sending DS GEM/OMCI)\n",
		   GPON_OMCC_FLOW, gpon_gem_ds_rx_cnt(GPON_OMCC_FLOW),
		   gpon_gem_ds_rx_cnt(0),
		   gpon_gem_ds_rx_cnt(1), gpon_gem_ds_rx_cnt(GPON_MCAST_FLOW));
	/* GLOBAL CAM-independent de-assembly counters (the decisive OMCI detector):
	 * UC_RX>0 => DS unicast GEM reaches the de-assembler; OMCI_RX>0 => it de-encapped
	 * an OMCI frame; ETH_CRC_ERR>0 => frames arrive but fail FCS (then are dropped). */
	seq_printf(s, "ds_misc: UC_RX=%u MC_RX=%u OMCI_RX=%u ETH_CRC_ERR=%u OVER_INTL=%u MC_LEAK=%u\n",
		   gpon_gem_ds_misc_cnt(1), gpon_gem_ds_misc_cnt(0), gpon_gem_ds_misc_cnt(6),
		   gpon_gem_ds_misc_cnt(4), gpon_gem_ds_misc_cnt(5), gpon_gem_ds_misc_cnt(3));
	/* GTC-layer GEM health (direct regs): distinguishes "GEM arrives but FAILS" (FAIL
	 * or HEC climbs => garble/sync) from "no GEM at all" (all flat). NON_IDLE/IDLE =
	 * good frames; FAIL=0x11c0 LOS=0x11b4 HEC=0x11b8 frm_to(0x4098). */
	seq_printf(s, "ds_gem: NON_IDLE=%u IDLE=%u FAIL=%u LOS=%u HEC=%u | frm_to(0x4098)=0x%x\n",
		   gpon_rd(GPON_GTC_DS_MISC_CNTR_GEM_NON_IDLE), gpon_rd(GPON_GTC_DS_MISC_CNTR_GEM_IDLE), gpon_rd(GPON_GTC_DS_MISC_CNTR_GEM_FAIL),
		   gpon_rd(GPON_GTC_DS_MISC_CNTR_GEM_LOS), gpon_rd(GPON_GTC_DS_MISC_CNTR_HEC_CORRECT), gpon_rd(GPON_GEM_DS_FRM_TIMEOUT));
	/* DS pipeline stages for the OMCI frame (read with the gate open at O5):
	 * A=de-encap pkt(f64)+global non-idle; B=PBO HIGH-queue(Q0) page cur/max;
	 * C=DS SRAM pool used/peak; D=PON-IP->NIC RX_OK/MISS/ERR + init-ready.
	 * Walk A->D: first 0 (or non-zero-meets-zero boundary) = the stall stage. */
	{
		u32 a = gpon_gem_flow_cnt(GPON_OMCC_FLOW, 0);
		u32 q0 = pi_rd(PI_PON_DS_PBO_PAGE_Q0), us = pi_rd(PI_PON_DSC_USAGE_DS),
		    sts = pi_rd(PI_PON_DSC_STS_DS);
		u32 ok = pi_rd(PI_PKT_OK_CNT_DS), ms = pi_rd(PI_PKT_MISS_CNT_DS), er = pi_rd(PI_PKT_ERR_CNT_DS);

		seq_printf(s, "ds_pipe: A_deenc(f%u)=%u nonidle=%u | B_q0cur=%u q0max=%u | C_sram=%u peak=%u | D_rxok=%u miss=%u err=%u initrdy=%u\n",
			   GPON_OMCC_FLOW, a, gpon_rd(GPON_GTC_DS_MISC_CNTR_GEM_NON_IDLE),
			   q0 & 0x1fff, (q0 >> 13) & 0x1fff,
			   us & 0x1fff, sts & 0x1fff,
			   ok & 0xffff, (ms >> 16) & 0xffff, er & 0xffff,
			   pi_rd(PI_PON_IPSTS_DS) & 1);
		/* DECISIVE break-localizer: FWD(f64)=GEM frames the GTC FORWARDED to the
		 * PON-IP (0x404c/0x4050) vs A_deenc(=de-encap'd). media_sts(0x1bf0c058) bit18
		 * = internal DS-GMII FORCELINK (stock ~0x106e8400). de-encap>FWD => GTC drops;
		 * FWD>0 & C_sram flat => PON-IP rejects (descriptor base or DS-GMII down). */
		seq_printf(s, "ds_fwd: FWD(f%u)=%u FWD(f0)=%u | media_sts=0x%08x (bit18 link=%u) gmii_en=%u\n",
			   GPON_OMCC_FLOW, gpon_gem_ds_fwd_cnt(GPON_OMCC_FLOW),
			   gpon_gem_ds_fwd_cnt(0),
			   pi_rd(PI_MEDIA_STS_DS), (pi_rd(PI_MEDIA_STS_DS) >> 18) & 1,
			   (pi_rd(PI_IO_CMD_0_DS) >> 5) & 1);
		/* Read-back the DS-engine enables (a later reset may have cleared them):
		 * ctl_ds expect 0x81 (CFG_PBUF_EN bit0 + bit7), io0_ds expect 0x90081070
		 * (GMII_RX_EN bit5 + GMII_TX_EN bit4 must be set). */
		/* The addresses in these labels are the RTL9602C's, the canonical spelling
		 * this file uses; the VALUES are read through the per-chip names, so they
		 * are right on every chip. Read a label as WHICH register, never WHERE. */
		seq_printf(s, "ds_en: ctl_ds(0xa0ac)=0x%08x io0_ds(0xd434)=0x%08x io1_ds(0xd438)=0x%08x\n",
			   pi_rd(PI_PONIP_CTL_DS), pi_rd(PI_IO_CMD_0_DS), pi_rd(PI_IO_CMD_1_DS));
		seq_printf(s, "ds_nic: cfg_ds(0xc04c)=0x%08x[RX_SID=%u] rxcfg_ds(0xc044)=0x%08x media_ds(0xc058)=0x%08x rxfdp_ds(0xd3f0)=0x%08x\n",
			   pi_rd(PI_CFG_DS), pi_rd(PI_CFG_DS) & 0x7f, pi_rd(PI_RX_CFG_DS),
			   pi_rd(PI_MEDIA_STS_DS), pi_rd(PI_RXFDP1_DS));
		/* US-NIC, compare against live stock: cfg_us=0x24030040 (bit29 set!),
		 * ctl_us=0, io0_us=0x90101070, io1_us=0x08000000, rxfdp_us=0. */
		seq_printf(s, "us_nic: cfg_us(0x404c)=0x%08x[RX_SID=%u] ctl_us(0x20ac)=0x%08x io0_us(0x5434)=0x%08x io1_us(0x5438)=0x%08x rxfdp_us(0x53f0)=0x%08x\n",
			   pi_rd(PI_CFG_US), pi_rd(PI_CFG_US) & 0x7f, pi_rd(0x20ac),
			   pi_rd(PI_IO_CMD_0_US), pi_rd(PI_IO_CMD_1_US), pi_rd(PI_RXFDP1_US));
		/* US-NIC per-group RX SID counters: good>0 confirms a CPU-injected US OMCI
		 * frame reached the US-NIC and matched THIS CHIP's OMCC SID, bad>0 is a SID
		 * mismatch. They count ONLY mask-enrolled SIDs (see the CNT_MASK_US write in
		 * the OMCC install), and the enrolled group is 0 -- the FIRST number printed.
		 * Every label here used to say 4, from a literal 0x20b0 under a
		 * mis-documented group stride, which sent the reader to a counter that is
		 * zero by design. */
		seq_printf(s, "us_rxsid[grp0..4]: good=%u/%u/%u/%u/%u bad=%u  (enrolled group is 0)\n",
			   pi_rd(PI_RX_SID_GOOD_CNT_US), pi_rd(0x2040), pi_rd(0x2044),
			   pi_rd(0x2048), pi_rd(0x204c), pi_rd(PI_RX_SID_BAD_CNT_US));
		/* ★ THE SPLIT THIS BOARD'S OPEN FAULT NEEDS: switch->PON-IP ingress
		 * (rx) beside PON->OLT egress (tx), for the same enrolled SID.  See
		 * PI_TX_SID_CNT_US for how to read the pair. */
		seq_printf(s, "us_sidcnt[grp0]: rx=%u tx=%u  (SID %u enrolled in CNT_MASK_US group 0)\n",
			   (u32)pi_rd(PI_RX_SID_CNT_US), (u32)pi_rd(PI_TX_SID_CNT_US),
			   GPON_OMCC_FLOW);
		/* The old "these bus-abort, never read them" note about PI_PKT_*_CNT_US/DS is
		 * OVERTAKEN, and corrected rather than deleted because it was true once: the
		 * ds_pipe line above reads the DS three directly and live values were quoted
		 * off them. The likely reason it WAS true is that the PON-IP NIC sub-window
		 * sits behind the PONPBO clock/reset enable taken by the Ethernet shell, so a
		 * read before that gate opens aborts and the same read after it does not.
		 * The DS three are MEASURED readable; the US three are INFERRED from their
		 * twins in the same window. If /proc/gpon ever aborts again, start here. */
		/* US-NIC arm (symmetric to DS) plus the OMCC classification, for a ustx=0
		 * triage: media_us must be 0x106e8400 (force-link UP), io0_us carries the GMII
		 * enables, gem_us_map holds the OMCC GEM port, and s2q reads SID2QID decoded
		 * with the 4-entries-per-word packing -- it should equal GPON_OMCC_PHYS_QID.
		 * The neighbouring word is kept alongside as what the old contiguous-packing
		 * bug wrote and read. */
		/* ★ EVERY INDEX HERE IS THE OMCC FLOW'S, DERIVED (2026-09-08).  The
		 * SIDVALID/SID2QID words were bare literals (0x2144 / 0x2138) chosen
		 * for SID 64: wrong flow on the RTL9603CVD, AND a bare PI literal
		 * bypasses the per-chip pi_x() relocation, which is a defect class
		 * this tree already records.  The label now prints the flow it read. */
		seq_printf(s, "us_arm: media_us(0x4058)=0x%08x io0_us(0x5434)=0x%08x gemus_map%u=0x%08x sidvld=0x%08x s2q=0x%08x s2q%u=%lu omcicfg=0x%08x\n",
			   pi_rd(PI_MEDIA_STS_US), pi_rd(PI_IO_CMD_0_US),
			   GPON_OMCC_FLOW,
			   gpon_rd(GPON_GEM_US_PORT_MAP + GPON_OMCC_FLOW * GEM_US_PORT_MAP_STRIDE),
			   pi_rd(PI_PON_SIDVALID + (GPON_OMCC_FLOW / 32u) * 4u),
			   pi_rd(PI_PON_SID2QID + (GPON_OMCC_FLOW / 4u) * PI_SID2QID_STRIDE),
			   GPON_OMCC_FLOW,
			   (pi_rd(PI_PON_SID2QID + (GPON_OMCC_FLOW / 4u) * PI_SID2QID_STRIDE)
			    >> ((GPON_OMCC_FLOW % 4u) * 7u)) & 0x7fUL,
			   pi_rd(PI_PON_OMCI_CFG));
		/* sched64: the OMCC queue's drain-side witnesses on one line.
		 *   total_pg     US pages staged in on-chip SRAM -- the raw occupancy the
		 *                DBRu reports to the OLT. On success it DRAINS toward 0.
		 *   bank_underfl the GEM-US framer fired on an empty TX bank, the drain-wall
		 *                signature. Should stay 0 once the queue drains cleanly.
		 *   over_sts     a near-FULL PBO backpressure watermark, NOT a DBRu/grant
		 *                trigger and not read by the scheduler: it stays 0 for a small
		 *                OMCI backlog EVEN ON SUCCESS, so it is a congestion
		 *                indicator only and never the pass/fail signal.
		 * PASS/FAIL = gemus_omcc climbing + total_pg draining + the OLT resuming
		 * grants. All direct reads, no DBG_CTRL_US strobe, so no encap hazard. */
		{
			/* The word is the OMCC SID's and the latch is its own register: this dump
			 * used SID 64's word and reached the latch as OVER_STS+0x14, both RTL9602C
			 * facts. The OMCC SID is 127 on the RTL9603CVD, and the gap from status to
			 * latch is 3 words there and 4 here, because the dies carry 65 and 128
			 * SIDs. */
			unsigned int ow = (GPON_OMCC_FLOW / 32u) * 4u;
			unsigned int ob = GPON_OMCC_FLOW % 32u;

			seq_printf(s, "sched64: total_pg=%u over_sts%u=%u over_latch%u=%u bank_underfl=%u dis_dbru_latch=%u [over w0/1/2=%08x/%08x/%08x latch=%08x schopt=%08x gemintr=%08x]\n",
				   pi_rd(PI_PONIP_TOTAL_PAGE_CNT_US) & 0x1fffu,
				   GPON_OMCC_FLOW,
				   (pi_rd(PI_PONIP_SID_OVER_STS + ow) >> ob) & 1u,
				   GPON_OMCC_FLOW,
				   (pi_rd(PI_PONIP_SID_OVER_LATCH_STS + ow) >> ob) & 1u,
				   gpon_rd(GPON_GEM_US_INTR_STS) & 1u,
				   (pi_rd(PI_PON_SCH_OPT) >> 19) & 1u,
				   pi_rd(PI_PONIP_SID_OVER_STS), pi_rd(PI_PONIP_SID_OVER_STS + 4),
				   pi_rd(PI_PONIP_SID_OVER_STS + 8),
				   pi_rd(PI_PONIP_SID_OVER_LATCH_STS + ow), pi_rd(PI_PON_SCH_OPT),
				   gpon_rd(GPON_GEM_US_INTR_STS));
		}
		/* feed64: the US-feed / framer witnesses. dsc_sts carries SRAM_USED (pages
		 * staged on chip) and DRAM_USED (pages handed toward the GTC); sstart is the
		 * per-grant burst-start gate and must read 1 for the framer to fire. If pages
		 * are staged but the framer emits nothing with bank_underfl=0, check sstart
		 * and whether dram_used advances. */
		seq_printf(s, "feed64: dsc_sts=0x%08x[sram_used=%u dram_used=%u] sstart(0x5200)=%u feed_cnt=%u\n",
			   pi_rd(PI_PON_DSC_STS_US), pi_rd(PI_PON_DSC_STS_US) & 0x1fffu,
			   (pi_rd(PI_PON_DSC_STS_US) >> 16) & 0x1fffu,
			   gpon_rd(GPON_GTC_US_PROC_MODE) & 1u, gpon_us_feed_rearm_cnt);
		/* usdram: the AUTHORITATIVE US SRAM->DRAM staging state -- the drain is
		 * HW-autonomous, so stalled staging is a DRAM-config fault, not a missing
		 * kick. The real used-page counter is PON_DSC_USAGE_US, not the dsc_sts above,
		 * and DSC_PIPE_VLD=0 means the descriptor pipe is idle. mstbase is the US DRAM
		 * ring base: if it is 0 or unreachable the DMA has no target, so SRAM fills
		 * and DRAM stays 0. runout's DRAM field reads 0 while DRAM is reserved, which
		 * the engine sees as permanently run-out. */
		seq_printf(s, "usdram: usage(0x20ec)=0x%08x[pipe_vld=%u] mstbase(0x20e8)=0x%08x dsccfg(0x215c)=0x%08x runout(0x20e0)=0x%08x ponipctl(0x20d8)=0x%08x[pbuf_en=%u]\n",
			   pi_rd(PI_PON_DSC_USAGE_US), (pi_rd(PI_PON_DSC_USAGE_US) >> 31) & 1u,
			   pi_rd(PI_IP_MSTBASE_US), pi_rd(PI_PON_DSC_CFG_US), pi_rd(PI_DSCRUNOUT_US),
			   pi_rd(PI_PONIP_CTL_US), pi_rd(PI_PONIP_CTL_US) & 1u);
		/* usintr: the GPON interrupt latch state -- the discriminator for an
		 * HW-event-latched FSM state a write-diff cannot see. Stock's resting
		 * GPON_INTR_MASK enables GTC_DS|GTC_US and its US events latch in
		 * GTC_US_INTR_DLT; our poll-only driver enables neither mask nor clears the
		 * latches. A US event sitting LATCHED-and-unserviced while the framer is
		 * stalled IS the wall: the US transmit FSM waits on an ack the poll never
		 * gives. */
		seq_printf(s, "usintr: top[mask0x40=0x%08x sts0x44=0x%08x] gtcus[dlt=0x%08x mask=0x%08x sts=0x%08x] gemus[dlt=0x%08x mask=0x%08x sts=0x%08x] svc_cnt=%u\n",
			   gpon_rd(GPON_INTR_MASK), gpon_rd(GPON_INTR_STS),
			   gpon_rd(GPON_GTC_US_INTR_DLT), gpon_rd(GPON_GTC_US_INTR_MASK), gpon_rd(GPON_GTC_US_INTR_STS),
			   gpon_rd(GPON_GEM_US_INTR_DLT), gpon_rd(GPON_GEM_US_INTR_MASK), gpon_rd(GPON_GEM_US_INTR_STS),
			   gpon_us_intr_svc_cnt);
		/* PON-IP OMCI packet counters. Decoder: rx>0 with the NIC filled=0 means
		 * de-encap is fine and the trap-to-CPU is the gap; drop>0 means the
		 * SID/queue/PBO mapping rejects the OMCI before it traps.
		 * The CPU side is printed beside the PON-IP side because the pair IS the
		 * diagnosis: cpu_tx counts responses the Ethernet shell QUEUED, cpu_drop the
		 * ones its ring REFUSED, ustx what the PON-IP actually TRANSMITTED. cpu_tx>0
		 * with cpu_drop=0 and ustx=0 means the frames left the MAC and the fabric
		 * swallowed them, a different fault from a ring that would not take them.
		 * cpu_tx=-1 is a shell with no OMCI datapath, never a zero. */
		seq_printf(s, "omci_pi: rx=%u drop=%u crcerr=%u ustx=%u trapcfg=0x%08x cpu_tx=%u cpu_drop=%u  (swcore OMCI block @0x%05x, PON_TRAP_CFG @0x%05x -- PER-CHIP)\n",
			   sw_rd(OMCI_RX_PKT_CNT), sw_rd(OMCI_DROP_PKT_CNT), sw_rd(OMCI_CRC_ERROR_PKT_CNT),
			   sw_rd(OMCI_TX_PKT_CNT), sw_rd(PON_TRAP_CFG),
			   rtl9602c_eth_omci_tx_dirty(), rtl9602c_eth_omci_tx_dropped(),
			   swc->omci_cnt, swc->pon_trap_cfg);
		/* The UNGATED US-NIC ingress witness, printed beside the two gated ones.
		 * us_rxsid is MASK-GATED: it reads zero both when the frame never arrived AND
		 * when the mask names a stream that carries nothing, and this board has been
		 * misread exactly that way. PKT_OK_CNT_US is the MAC's own accept counter,
		 * with no classification involved, so the pair separates them:
		 *   rx_ok=0                     the frame never reached the US-NIC MAC
		 *   rx_ok>0, us_rxsid all 0     it arrived and was NOT classified
		 *   rx_ok>0, err/miss>0         it arrived and the engine rejected it */
		seq_printf(s, "us_nic_cnt: rx_ok=%u tx_ok=%u err=%u miss=%u  (PI 0x04010/14/18, UNGATED)\n",
			   (u32)(pi_rd(PI_PKT_OK_CNT_US) & 0xffff),
			   (u32)((pi_rd(PI_PKT_OK_CNT_US) >> 16) & 0xffff),
			   (u32)pi_rd(PI_PKT_ERR_CNT_US), (u32)pi_rd(PI_PKT_MISS_CNT_US));
		/* GTC upstream grant accounting: if the ONU transmits ANY upstream GEM on
		 * the OLT's BWMAP grants these climb. */
		seq_printf(s, "us_tx: ploam_acpt(0x119c)=%u bwm_acpt(0x11b0)=%u bwm_fail(0x11a4)=%u bwm_inv(0x11a8)=%u | us_onu_id=%u ds_cfg(0x1014)=0x%08x\n",
			   gpon_rd(GPON_GTC_DS_MISC_CNTR_PLOAM_ACPT), gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_ACPT), gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_FAIL), gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_INV),
			   (gpon_rd(GPON_GTC_US_ONU_ID) >> 8) & 0xff, gpon_rd(GPON_GTC_DS_CFG));
		/* Alloc CAM read-back: T-CONT 16 must hold alloc 0x400 (HIT) for the GTC to
		 * accept the OLT's operational BWMAP grant on that alloc-id. */
		{
			int a16 = gpon_alloc_cam_read(16);

			if (a16 < 0)
				seq_printf(s, "us_alloc: tc16=READ-FAILED rc=%d\n", a16);
			else
				seq_printf(s, "us_alloc: tc16=alloc0x%x hit%u\n", a16 & 0xfff, !!(a16 & BIT(16)));
		}
		/* bwmap: what the GTC BWMAP capture engine holds -- which T-CONT the OLT's
		 * grants RESOLVE TO, and whether we ever configured it.
		 * WORD 0 (0x2400 + 8*i): [23] VALID [22] LST [21] EoB [20] SoB [19] PLOAMu
		 *                        [18] FEC [17:16] DBRu [14:12] MF [4:0] T-CONT index
		 * WORD 1 (+4): [15:0] StartTime, [31:16] StopTime.
		 * There is NO Alloc-ID in the capture: word 0 carries the T-CONT index, the
		 * value AFTER the alloc-CAM has resolved the grant, so a grant that MISSED
		 * the CAM cannot appear here at all. An allocation is TWO words at an 8-byte
		 * stride. Status carries exactly one defined bit, CAP_OVERFL -- there is no
		 * done/ready/fresh bit in this engine.
		 * THE ARM BELOW IS INCOMPLETE: the vendor pulses CAP_CLR, sets a frame count
		 * and waits 5-10 ms before reading, while ours ORs CAP_EN into whatever was
		 * there. Left as it was on purpose -- completing it means new writes and a
		 * sleep in this path with no board to prove them on. Every report is
		 * therefore gated on the per-entry VALID bit, the only freshness signal this
		 * silicon offers, so a never-armed buffer reports NOTHING rather than
		 * T-CONT 0 thirty-two times.
		 * `tcont_en` is the set of T-CONTs the US scheduler was configured for: a
		 * VALID allocation naming a T-CONT outside it is a grant answered on a queue
		 * nothing drains -- the alloc-CAM wall seen from the grant side. */
		{
			u32 en = pi_rd(PI_PON_TCONT_EN);
			int i, nvalid = 0;

			gpon_wr(GPON_BWMAP_CTRL, gpon_rd(GPON_BWMAP_CTRL) | (1u << 15));	/* CAP_EN arm */
			seq_printf(s, "bwmap: ctrl(0x200c)=0x%08x sts(0x2010)=0x%08x[OVERFL=%lu] tcont_en=0x%08x alloc[0..2]=%08x/%08x %08x/%08x %08x/%08x\n",
				   gpon_rd(GPON_BWMAP_CTRL), gpon_rd(GPON_BWMAP_STS),
				   (gpon_rd(GPON_BWMAP_STS) >> 8) & 1UL, en,
				   /* allocation i = words 2i, 2i+1 of the 256-word array;
				    * word 0 is spelled as the base so the alias guard keeps
				    * its gpon_rd() window evidence for GPON_BWMAP_DATA */
				   gpon_rd(GPON_BWMAP_DATA), gpon_rd(BWMAP_DATA(1)),
				   gpon_rd(BWMAP_DATA(2)), gpon_rd(BWMAP_DATA(3)),
				   gpon_rd(BWMAP_DATA(4)), gpon_rd(BWMAP_DATA(5)));

			/* 32 allocations is what the vendor's own readers walk, of
			 * the 128 the address space holds; entries past 32 are
			 * never exercised by any vendor code, so we do not invent a
			 * meaning for them. */
			for (i = 0; i < 32; i++) {
				u32 w0 = gpon_rd(BWMAP_DATA(2 * i));	/* 8-byte allocation i */
				u32 w1 = gpon_rd(BWMAP_DATA(2 * i + 1));
				u8 tc, dmp[9];

				if (!(w0 & BIT(23)))		/* VALID */
					continue;
				nvalid++;
				tc = (u8)(w0 & 0x1f);
				if (en & BIT(tc))		/* configured: fine */
					continue;

				/* The dump is the whole allocation plus the
				 * bitmap it was judged against, so the line is
				 * self-contained: entry index, both captured
				 * words big-endian, and tcont_en. */
				dmp[0] = (u8)i;
				dmp[1] = (u8)(w0 >> 24); dmp[2] = (u8)(w0 >> 16);
				dmp[3] = (u8)(w0 >> 8);  dmp[4] = (u8)w0;
				dmp[5] = (u8)(en >> 24); dmp[6] = (u8)(en >> 16);
				dmp[7] = (u8)(en >> 8);  dmp[8] = (u8)en;
				gpon_unsup_report("bwmap_tcont", GPON_UNSUP_RANGE,
						  tc, "a-tcont-set-in-tcont_en",
						  dmp, sizeof(dmp));
				seq_printf(s, "bwmap_grant: entry%d -> T-CONT %u NOT in tcont_en=0x%08x (w0=%08x w1=%08x)\n",
					   i, tc, en, w0, w1);
			}
			seq_printf(s, "bwmap_scan: %d valid allocation(s) of 32 examined\n",
				   nvalid);
		}
		/* US GTC emission breakdown: PLOAM (idx2 cpu / idx3 auto) against GEM (idx4
		 * byte / idx1 dbru), plus the per-T-CONT idle-GEM byte counter.
		 * TCONT_IDLE_BYTE_STAT's register-map "array offset 64" is 64 BITS, so the
		 * stride is 8 BYTES -- a named constant now, so nobody derives it a third
		 * time. The decisive signature for "OLT deactivates ~42 s, no OMCI" is cpu>0
		 * (ACKs egress) with gem_byte=0 AND idle16=0: no US GEM fills the grants. */
		seq_printf(s, "us_gtc: ploam_cpu=%u ploam_auto=%u | gem_byte=%u gem_dbru=%u | idle16=%u/%u idle8=%u gemus_omcc(f%u)=%u/%u gem2=%u(0x6810) | us_cfg=0x%04x pti=0x%08x\n",
			   gpon_us_misc_cnt(2), gpon_us_misc_cnt(3),
			   gpon_us_misc_cnt(4), gpon_us_misc_cnt(1),
			   gpon_rd(TCONT_IDLE_STAT(16)), gpon_rd(TCONT_IDLE_STAT(16) + 4),
			   gpon_rd(TCONT_IDLE_STAT(8)),
			   (unsigned int)GPON_OMCC_FLOW,
			   gpon_rd(GEM_US_STAT(GPON_OMCC_FLOW)),
			   gpon_rd(GEM_US_STAT(GPON_OMCC_FLOW) + 4),
			   gpon_rd(GEM_US_STAT(2)),
			   gpon_rd(GPON_GTC_US_CFG), gpon_rd(GPON_GEM_US_PTI_CFG));
		/* GEM_US_BYTE_STAT full scan (base 0x6800, stride 8): which flow does the
		 * port-2 OMCI actually land on? Non-zero on flow!=64 => SID-stamp/SID2QID
		 * misroute; all-zero => the US-NIC drops it before any flow (ingest gap). */
		{
			int f, n = 0; char gbuf[220];
			for (f = 0; f < 128 && n < 200; f++) {
				u32 gv = gpon_rd(GPON_GEM_US_BYTE_STAT + f * 8);
				if (gv)
					n += scnprintf(gbuf + n, sizeof(gbuf) - n, " f%d=%u", f, gv);
			}
			seq_printf(s, "gemus_scan:%s\n", n ? gbuf : " (all flows 0)");
		}
		/* US-scheduler readback: did the gpon_install_tcont writes actually LAND, or
		 * were they write-protected or at the wrong offset? Expect pirQ/cirQ 0x3ffff,
		 * the T-CONT enabled, qmap 1, wfqtype 0, wfqwt 1, drain idle.
		 * The labels carry no address: every one printed the RTL9602C offset while
		 * the read goes to this chip's own, and a label naming a register the read
		 * did not touch is the wrong-way evidence this dump exists to avoid. */
		seq_printf(s, "us_sched: pirQ%u=0x%08x cirQ%u=0x%08x tcont_en=0x%08x qmapT%u=0x%08x wfqtypeQ%u=0x%08x wfqwtQ%u=0x%08x drn=0x%08x sch_ctrl=0x%08x[PIR_DROP=%lu]\n",
			   (unsigned int)GPON_OMCC_PHYS_QID,
			   pi_rd(PI_PON_QID_PIR_RATE + GPON_OMCC_PHYS_QID * PI_QID_RATE_STRIDE),
			   (unsigned int)GPON_OMCC_PHYS_QID,
			   pi_rd(PI_PON_QID_CIR_RATE + GPON_OMCC_PHYS_QID * PI_QID_RATE_STRIDE),
			   pi_rd(PI_PON_TCONT_EN),
			   (unsigned int)GPON_OMCC_TCONT,
			   pi_packed_get(PI_PON_SCH_QMAP, GPON_OMCC_TCONT, swc->sch_qmap_bits),
			   (unsigned int)GPON_OMCC_PHYS_QID,
			   pi_packed_get(PI_PON_WFQ_TYPE, GPON_OMCC_PHYS_QID, 1),
			   (unsigned int)GPON_OMCC_PHYS_QID,
			   pi_packed_get(PI_PON_WFQ_WEIGHT, GPON_OMCC_PHYS_QID, 10),
			   pi_rd(PI_DRN_CMD),
			   pi_rd(PI_PON_SCH_CTRL), (pi_rd(PI_PON_SCH_CTRL) >> 18) & 1UL);
		/* SID page-occupancy probe, an OLT-INDEPENDENT verify of the descriptor
		 * sideband: MAX_PAGE_CNT for the OMCC SID is non-zero iff an OMCI frame was
		 * ever classified and enqueued to its physical queue. It latches even after
		 * drain and needs NO grant, so a self-test inject that makes it non-zero
		 * confirms the descriptor's stream id reaches the US-NIC classifier.
		 * The diagnostic write MUST preserve DBG_IGNORE_TAG and CFG_US_EP_IPG, set
		 * in gpon_pbo_init(): writing only SID_NO+RD_MAX clears DBG_IGNORE_TAG and
		 * re-breaks the US GEM encapsulation. The request word, that guard and the
		 * bounded poll are pi_sid_page_cnt() -- ONE spelling, shared with the
		 * data-flow probe below; its rc >= 0 is the iteration BUSY cleared at and
		 * < 0 is -ETIMEDOUT, meaning the used/max beside it is STALE. */
		{
			u32 pc;
			int prc = pi_sid_page_cnt(GPON_OMCC_FLOW, &pc);

			seq_printf(s, "sidpage_omcc: used=%u max=%u (max>0 = OMCI ENQUEUED to the OMCC queue) [r255c=0x%08x r2564=0x%08x poll=%d]\n",
				   pc & 0x1fff, (pc >> 16) & 0x1fff, pi_rd(PI_PONIP_DBG_CTRL_US), pc, prc);
		}
		/* Data-flow US datapath witness. The data flow rides the OMCC queue's grants
		 * but keeps its OWN classify entry, US gem-map and per-SID page bank. Symptom
		 * it exists for: data US works at DHCP then stops while the lease is held
		 * stale. If gpon0 TX climbs but the page bank stays flat, data frames are
		 * dropped at the US-NIC classify/ingest BEFORE the queue. */
		{
			u32 pc1;
			int prc1;
			u32 s2q1 = (pi_rd(PI_PON_SID2QID) >> ((GPON_DATA_FLOW % 4) * 7)) & 0x7fu;
			u32 svl1 = (pi_rd(PI_PON_SIDVALID) >> GPON_DATA_FLOW) & 1u;

			/* Same strobe as sidpage_omcc above, different SID: one helper.
			 * pgpoll= is NEW and is the helper's rc -- this site used to
			 * compute the poll count and THROW IT AWAY, so a wedged strobe
			 * printed a stale pgbank1 that read exactly like a fresh one. */
			prc1 = pi_sid_page_cnt(GPON_DATA_FLOW, &pc1);
			/* `q32_idle` was REMOVED here (2026-09-10) and the removal is the repair.
			 * It re-read TCONT_IDLE_STAT(8), the SAME register `idle8` reads a few
			 * lines up, and that register is CLEAR-ON-READ: measured on the G24W, one
			 * page printed idle8=9124017 and q32_idle=6450 for ONE register. The
			 * second reader in a page render gets only the microseconds between the
			 * two seq_printf calls, and on a clear-on-read counter it also STEALS
			 * counts from the first. The label lied twice over: `q32` is the
			 * RTL9602C's queue for T-CONT 8, which is 64 on this die. */
			seq_printf(s, "data1: s2q[1]=%u sidvld[1]=%u usmap1(0x6404)=0x%x usbyte1(0x6808)=%u pgbank1_used=%u max=%u pgpoll=%d\n",
				   s2q1, svl1,
				   gpon_rd(GPON_GEM_US_PORT_MAP + GPON_DATA_FLOW * GEM_US_PORT_MAP_STRIDE),
				   gpon_rd(GEM_US_STAT(GPON_DATA_FLOW)),
				   pc1 & 0x1fff, (pc1 >> 16) & 0x1fff,
				   prc1);
		}
		/* CAM read-back: does the DS GEM CAM actually map gem->flow 64 at runtime?
		 * e64 should read gem=2 (the OMCC) HIT=1; traffic_cfg[64] should be 0x4. */
		{
			int c64 = gpon_ds_cam_read(GPON_OMCC_FLOW), c0 = gpon_ds_cam_read(0);
			int c1 = gpon_ds_cam_read(GPON_DATA_FLOW);

			if (c64 < 0 || c0 < 0 || c1 < 0)
				seq_printf(s, "ds_cam: READ-FAILED e%u=%d e0=%d e%u=%d\n",
					   GPON_OMCC_FLOW, c64, c0, GPON_DATA_FLOW, c1);
			else
				seq_printf(s, "ds_cam: e%u=gem%u hit%u tcfg=0x%x | e0=gem%u hit%u | e%u(data)=gem%u hit%u tcfg=0x%x\n",
					   GPON_OMCC_FLOW, c64 & 0xfff, !!(c64 & BIT(16)),
					   gpon_rd(GPON_GTC_DS_TRAFFIC_CFG +
						   GPON_OMCC_FLOW * DS_TRAFFIC_CFG_STRIDE) & 0x1f,
					   c0 & 0xfff, !!(c0 & BIT(16)),
					   GPON_DATA_FLOW, c1 & 0xfff, !!(c1 & BIT(16)),
					   gpon_rd(GPON_GTC_DS_TRAFFIC_CFG + GPON_DATA_FLOW * 4) & 0x1f);
		}
	}
	/* Full per-flow de-encap sweep: does ANY flow de-encapsulate a GEM frame?
	 * If some flow > 0 => the OLT IS sending de-encodable GEM (find the OMCI
	 * flow). If NONE => frames arrive but de-encap is globally not happening
	 * (gem-port CAM / OMCI not classifying), or the OLT sends only idle GEM. */
	{
		int f, n = 0;

		seq_printf(s, "ds_deenc_sweep:");
		for (f = 0; f < 128; f++) {
			u32 v = gpon_gem_flow_cnt(f, 0);

			if (v && v != 0xffffffffu) {
				seq_printf(s, " f%d=%u", f, v);
				n++;
			}
		}
		seq_printf(s, "%s\n", n ? "" : " (NONE de-encap)");
	}
	/* The "operational" reference values are Board C's (RTL9602C) -- printing
	 * them beside another chip's reading invites a diff that means nothing, so
	 * they appear only on the chip they were measured on. Where the chip
	 * declares no IO_GPIO_EN the words are not read at all. */
	if (SOC_IO_GPIO_EN)
		seq_printf(s, "io: io_mode_en=0x%08x@0x%05x gpio_en0=0x%08x gpio_en1=0x%08x%s\n",
			   sw_rd(SOC_IO_MODE_EN), swc->io_mode_en,
			   sw_rd(SOC_IO_GPIO_EN), sw_rd(SOC_IO_GPIO_EN + 4),
			   swc == &gpon_swc_9602c
				? "  (operational 0x12050/0x40202006/0x819)" : "");
	else
		seq_printf(s, "io: io_mode_en=0x%08x@0x%05x gpio_en=n/a (%s declares no optical GPIO pad map)\n",
			   sw_rd(SOC_IO_MODE_EN), swc->io_mode_en, swc->chip);
	seq_printf(s, "io: oem_en(bit%u)=%d i2c_en_bus0(bit%u)=%d\n",
		   swc->io_oem_en, !!(sw_rd(SOC_IO_MODE_EN) & IO_OEM_EN),
		   swc->io_i2c_en_bus0,
		   !!(sw_rd(SOC_IO_MODE_EN) & (1u << swc->io_i2c_en_bus0)));

	/*
	 * SerDes RX/analog readback against the known-good O5 values with light
	 * present. Mismatches here explain an RX that will not lock.
	 */
	seq_printf(s, "sds: dig00=0x%08x dig1d=0x%08x  (golden 0xf30 / 0x1c000)\n",
		   sw_rd(WSDS_DIG_00), sw_rd(WSDS_DIG_1D));
	seq_printf(s, "sds: com03=0x%08x com26=0x%08x gpon42=0x%08x dig18=0x%08x@0x%05x\n",
		   sw_rd(SDS_ANA_COM_REG03), sw_rd(SDS_ANA_COM_REG26),
		   sw_rd(SDS_ANA_GPON_REG42), sw_rd(WSDS_DIG_18), swc->wsds_dig_18);
	/* ★ THE ONE LINE THAT SAYS WHETHER optic_los IS SAMPLED OR FORCED.
	 * OPTIC_LOS_SIG in los_cfg_sts cannot tell you which: a forced input and a
	 * dark fibre read identically. frc=1 means the GTC is ignoring the pad and
	 * using frcv, so "optic_los=1 frc=1 frcv=1" is our own register, not the
	 * fibre -- and "frc=0" is what makes optic_los=1 a real optical statement. */
	{
		u32 d18 = sw_rd(WSDS_DIG_18);

		seq_printf(s, "optic_los_src: %s (dig18 frc=%d frcv=%d sel_epon=%d ben_oe=%d)\n",
			   (d18 & WSDS_FRC_OPTIC_LOS) ? "FORCED -- the pad is NOT being sampled"
						      : "the pad (sampled)",
			   !!(d18 & WSDS_FRC_OPTIC_LOS), !!(d18 & WSDS_FRCV_OPTIC_LOS),
			   !!(d18 & WSDS_OPTIC_LOS_SEL_EPON), !!(d18 & BIT(12)));
	}
	seq_printf(s, "sds: misc00=0x%08x misc01=0x%08x misc02=0x%08x fib21=0x%08x\n",
		   sw_rd(SDS_ANA_MISC_REG00), sw_rd(SDS_ANA_MISC_REG01),
		   sw_rd(SDS_ANA_MISC_REG02), sw_rd(FIB_EXT_REG21));
	/* TX-path verification: confirm my TX serializer writes actually landed. */
	seq_printf(s, "sds_tx: cfg=0x%08x com22_txamp=0x%08x reg24=0x%08x dig1e_d2a=0x%08x dig02_pdben=0x%08x dig03_txdis=0x%08x forceben=0x%08x\n",
		   sw_rd(SDS_CFG), sw_rd(SDS_ANA_COM_REG22), sw_rd(SDS(0x225e0)),
		   sw_rd(SDS(0x220a8)), sw_rd(WSDS_DIG_02), sw_rd(WSDS_DIG_03), sw_rd(SDS(0x220e4)));
	if (SDS_FIB_STATUS) {
		u32 fibsts = sw_rd(SDS_FIB_STATUS);

		seq_printf(s, "sds: fib_status=0x%08x@0x%05x (sds_sdet=%u fib100_sdet=%u link_ok=%u) fib_reg16=0x%08x@0x%05x\n",
			   fibsts, swc->sds_fib_status, !!(fibsts & SDS_FIB_SDS_SDET),
			   !!(fibsts & BIT(2)), !!(fibsts & BIT(4)),
			   sw_rd(FIB_REG16), swc->fib_reg16);
	} else {
		seq_printf(s, "sds: fib_status=n/a (%s declares no SDS_FIB_STATUS in this driver)\n",
			   swc->chip);
	}
	/* The probe's own cache is subject to the same rule as the live reads: these
	 * five come from bosa_probe(), which reads the RTL8290B pages, so on a module
	 * that does not carry them they are the same phantom. */
	if (bosa_regs_live())
		seq_printf(s, "bosa: rtl8290b num=0x%04x vid=0x%02x w41=0x%02x ctrl2=0x%02x status2=0x%02x\n",
			   bosa_id_num, bosa_id_vid, bosa_w41, bosa_ctrl2,
			   bosa_status2);
	else
		seq_puts(s, "bosa: n/a -- the RTL8290B chip-ID read did not complete, so nothing this driver holds about that device is a reading\n");
	seq_printf(s, "fsm: state=O%u onu_id=%u sn_tx=%u ds_rx=%u sds_sync=%u ticks=%u sn=%*phN\n",
		   gpon_fsm_state, gpon_fsm_onu_id, gpon_fsm_sn_tx, gpon_ds_rx,
		   gpon_sds_synced, gpon_fsm_ticks, 8, gpon_sn_bytes);
	/* Live BOSA laser-emission status: R30 (txsd/valid/apc-done/mpd-fault),
	 * R33 bias-DAC readback (nonzero => bias driven), R32 mod, FAULT_STATUS. */
	if (!bosa_regs_live()) {
		char idbuf[16];

		if (bosa_id_num < 0)
			strscpy(idbuf, "COULD NOT ASK", sizeof(idbuf));
		else
			scnprintf(idbuf, sizeof(idbuf), "0x%04x", bosa_id_num);
		seq_printf(s, "bosa_regs: n/a -- this module exposes no RTL8290B register interface (chip id %s, want 0x8290; I2C slaves 0x54/0x55 do not answer). The RTL8290B-derived lines are SUPPRESSED rather than printed as values.\n",
			   idbuf);
	} else {
		int r30 = bosa_read_reg(0x31e), r33 = bosa_read_reg(0x321);
		int r32 = bosa_read_reg(0x320), fault = bosa_read_reg(0x389);

		seq_printf(s, "bosa_tx: R30=0x%02x txsd=%d valid=%d apc_done=%d mpd_vhi=%d mpd_vlo=%d bias=0x%02x mod=0x%02x fault=0x%02x\n",
			   r30 & 0xff, !!(r30 & BIT(6)), !!(r30 & BIT(5)),
			   !!(r30 & BIT(7)), !!(r30 & BIT(3)), !!(r30 & BIT(2)),
			   r33 & 0xff, r32 & 0xff, fault & 0xff);
	}
	/* Fibre optical power. The words at 0x166-0x169 are NOT live on this BOSA: our
	 * own init image programmed them and the MCU never refreshes them from the
	 * ADC, so they never move with the optic. Read the LIVE sigma-delta RSSI ADC
	 * on the page-3 bank instead, which the bosa_tx block above proves reachable.
	 * Emit the raw u16; userspace converts. 0x0000/0xffff = n/a. */
	{
		/* RX optical power: the live RSSI ADC byte at 0x311 (slave 0x55) tracks the
		 * optic -- bench-confirmed against the 5 dB attenuator: code 0x3a @ ~-26.5 dBm
		 * (attenuator in) -> 0x8c @ ~-5.8 dBm (attenuator out). The 0x30e/0x30f word is
		 * a constant/cycling mux channel, NOT RX. 2-point linear-in-dB fit, integer/no
		 * FPU: centi-dBm = code*2513/100 - 4100 (refine vs the rtl8290b.data table). */
		int rxc = bosa_read_reg(0x311) & 0xff;	/* raw 8-bit RSSI tap: kept as instrument */
		u32 rx_code = bosa_rx_code();
		u32 rssi_uv = bosa_rssi_uv();			/* ratiometric fraction*10000, info */
		int txw = bosa_read16_median(0x166);
		char cdbm_s[16];

		/* Same rule as the "fiber:" line: a chain that produced no reading
		 * prints n/a, never the -4000 floor. */
		if (rx_code == BOSA_RX_CODE_NA)
			strscpy(cdbm_s, "n/a", sizeof(cdbm_s));
		else
			scnprintf(cdbm_s, sizeof(cdbm_s), "%d",
				  bosa_code_to_cdbm(rx_code));
		if (bosa_regs_live()) {
			seq_printf(s, "optic_rx_raw: 0x%02x optic_rx_cdbm: %s optic_tx_raw: 0x%04x\n",
				   rxc, cdbm_s, txw & 0xffff);
			seq_printf(s, "optic_env:   temp_dc=%d bias_ua=%u tx_cdbm=%d\n",
				   bosa_temp_dc(), bosa_bias_ua(), bosa_tx_power_cdbm());
		}
		/* ★ The module's OWN optical monitor, raw, beside the derived dBm --
		 * the witness that is independent of the SoC pad mux and the SerDes.
		 * Printed unconditionally so the raw words are visible even when the
		 * conversion declines to produce a number. */
		{
			int a2h = bosa_i2c_read8(0x51, 104);
			int a2l = bosa_i2c_read8(0x51, 105);
			s32 a2c = bosa_rx_power_sff8472_cdbm();
			char a2s[16];

			if (a2c == BOSA_RX_CDBM_NA)
				strscpy(a2s, "n/a", sizeof(a2s));
			else
				scnprintf(a2s, sizeof(a2s), "%d.%02ddBm", a2c / 100,
					  (a2c < 0 ? -a2c : a2c) % 100);
			seq_printf(s, "optic_a2:    rx_pwr_raw=%02x%02x (0.1uW) -> %s  [SFF-8472 A2 slave 0x51 b104/105 -- the MODULE's own monitor]\n",
				   a2h < 0 ? 0xff : a2h & 0xff,
				   a2l < 0 ? 0xff : a2l & 0xff, a2s);
			/* The rest of the same page, and on a module with no RTL8290B register
			 * interface it is the ONLY optical witness there is: b96/97 temperature,
			 * b98/99 Vcc, b100/101 TX bias, b102/103 TX power. Each word prints n/a
			 * when its own read failed -- never 0, which on the TX pair would read as
			 * "the laser is dark". */
			{
				int k, b[8];
				char t[8][8];

				for (k = 0; k < 8; k++) {
					b[k] = bosa_i2c_read8(0x51, 96 + k);
					if (b[k] < 0)
						strscpy(t[k], "n/a", sizeof(t[k]));
					else
						scnprintf(t[k], sizeof(t[k]), "%02x", b[k]);
				}
				seq_printf(s, "optic_a2_env: temp=%s%s vcc=%s%s tx_bias=%s%s tx_pwr=%s%s  [b96..b103, raw]\n",
					   t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]);
			}
		}
		if (bosa_regs_live()) {
			s32 v = bosa_vmpd_mv(0x3B3);	/* one live MPD sample -> latch its taps */

			seq_printf(s, "optic_txchain: dark=%d vmpd=%d code=%u hi=%d zero=%d iavg=%02x range=%d\n",
				   bosa_vmpd_dark, v, bosa_tx_dbg_code, bosa_tx_dbg_hi,
				   bosa_tx_dbg_zero, bosa_read_reg(0x23A) & 0xff,
				   (bosa_read_reg(0x246) >> 6) & 3);
		}
		/* Full RX chain intermediates: a HW read at a known attenuation pins the exact
		 * ref-tap roles + rx_thr against the -14 dBm anchor (code 398 = 0.1uW at -14 dBm). */
		if (bosa_regs_live())
			seq_printf(s, "optic_rxchain: rssi_uv=%u code=%u cdbm=%s | adcA=%06x ref305=%06x ref314=%06x\n",
				   rssi_uv, rx_code, cdbm_s, bosa_read24(0x30e), bosa_read24(0x305), bosa_read24(0x314));
		if (bosa_regs_live())
			seq_printf(s, "optic_dbg: 30c=%02x 30d=%02x 30e=%02x 30f=%02x 310=%02x 311=%02x 312=%02x | 166=%02x 167=%02x 168=%02x 169=%02x\n",
			   bosa_read_reg(0x30c) & 0xff, bosa_read_reg(0x30d) & 0xff,
			   bosa_read_reg(0x30e) & 0xff, bosa_read_reg(0x30f) & 0xff,
			   bosa_read_reg(0x310) & 0xff, bosa_read_reg(0x311) & 0xff,
			   bosa_read_reg(0x312) & 0xff, bosa_read_reg(0x166) & 0xff,
			   bosa_read_reg(0x167) & 0xff, bosa_read_reg(0x168) & 0xff,
			   bosa_read_reg(0x169) & 0xff);
		/* ANI-G ME263 #10/#14 cached levels (OMCI 0.002 dB, 2's-comp) reported to
		 * the OLT — refreshed from the DDM words on the FSM tick. */
		seq_printf(s, "anig_rx_level: 0x%04x anig_tx_level: 0x%04x\n",
			   (u16)anig_rx_level, (u16)anig_tx_level);
	}
	/* BOSA page0 (slave 0x50) control regs — compare against the O5 operating
	 * state to find the APCDIG clock/power enable (O5 p0: 02 04 0b ff ff ff ff 0c
	 * .. 52 54 20). */
	seq_printf(s, "bosa_p0: 00=%02x 01=%02x 02=%02x 03=%02x 04=%02x 05=%02x 08=%02x 0c=%02x 10=%02x 14=%02x 18=%02x 1c=%02x\n",
		   bosa_read_reg(0x00) & 0xff, bosa_read_reg(0x01) & 0xff,
		   bosa_read_reg(0x02) & 0xff, bosa_read_reg(0x03) & 0xff,
		   bosa_read_reg(0x04) & 0xff, bosa_read_reg(0x05) & 0xff,
		   bosa_read_reg(0x08) & 0xff, bosa_read_reg(0x0c) & 0xff,
		   bosa_read_reg(0x10) & 0xff, bosa_read_reg(0x14) & 0xff,
		   bosa_read_reg(0x18) & 0xff, bosa_read_reg(0x1c) & 0xff);
	if (bosa_regs_live()) {
		seq_printf(s, "bosa_p3: 31c=%02x 31d=%02x 31f=%02x 322=%02x 323=%02x (O5 00 33 00 da 00)\n",
			   bosa_read_reg(0x31c) & 0xff, bosa_read_reg(0x31d) & 0xff,
			   bosa_read_reg(0x31f) & 0xff, bosa_read_reg(0x322) & 0xff,
			   bosa_read_reg(0x323) & 0xff);
		seq_printf(s, "bosa_p2: W54_236=%02x W56_238=%02x W57_239=%02x W61_24d=%02x W88_284=%02x (O5 19 22 2d b0 76)\n",
			   bosa_read_reg(0x236) & 0xff, bosa_read_reg(0x238) & 0xff,
			   bosa_read_reg(0x239) & 0xff, bosa_read_reg(0x24d) & 0xff,
			   bosa_read_reg(0x284) & 0xff);
		seq_printf(s, "bosa_apc: W69_245=%02x(loopmode) W58_23a=%02x(iavg) W72_248=%02x(biasmax) W73_249=%02x(biasmin 0x2a)\n",
			   bosa_read_reg(0x245) & 0xff, bosa_read_reg(0x23a) & 0xff,
			   bosa_read_reg(0x248) & 0xff, bosa_read_reg(0x249) & 0xff);
	}
	/* SerDes/serializer run-state vs LIVE working-stock O5 (stock_usburst.txt
	 * golden in [..]). Decisive good-vs-bad-boot diff for the cold-start US-TX
	 * serializer lock: any reg that DIVERGES from the [stock] value on a
	 * non-leasing boot is a live-confirmed candidate (unlike the transient
	 * mode-set programmed values, which are overwritten by the CDR reset). */
	seq_printf(s, "sds_run: 280c=%04x[3106] 281c=%04x[1359] 225a0=%04x[713] 220a8=%04x[2] 225d8=%04x[29]\n",
		   sw_rd(SDS(0x2280c)) & 0xffff, sw_rd(SDS(0x2281c)) & 0xffff,
		   sw_rd(SDS_ANA_COM_REG08) & 0xffff, sw_rd(SDS(0x220a8)) & 0xffff,
		   sw_rd(SDS_ANA_COM_REG22) & 0xffff);
	seq_printf(s, "sds_ext: 22a2c=%04x[0] 22a30=%04x[4] 22a34=%04x[326a]\n",
		   sw_rd(SDS(0x22a2c)) & 0xffff, sw_rd(SDS(0x22a30)) & 0xffff,
		   sw_rd(SDS(0x22a34)) & 0xffff);
	return 0;
}

/* FSM state exposed to /proc (defined with the FSM below). */

/*
 * This FSM also exists in the shared tree, and that is DELIBERATE (2026-08-05).
 * The block below -- the PLOAM message types, the serial parse, the upstream
 * builders, the burst-overhead and equalization-delay computations, the state
 * transitions, the downstream dispatch and the poll decisions -- was carved into
 * the hardware-decoupled core at
 *     target/linux/gpon-common/files-6.18/drivers/net/gpon/gpon_ploam.{h,c}
 * which both OpenWrt targets pull in and which also compiles on x86, so the
 * activation FSM can be driven by an adversarial OLT under libFuzzer+ASan+UBSan
 * with no board in the loop -- this project's PRIMARY correctness gate.
 * Measured honestly: it de-duplicates nothing today, because Elnath has no
 * software PLOAM FSM at all -- its MAC runs O1->O5 and auto-ACKs in silicon.
 *
 * Keep the two in step: a fix here that is not mirrored there makes the offline
 * gate lie about this driver.
 *
 * What remains between this driver and the common FSM is a SHAPE CHANGE, not an
 * obstacle: the FSM below is global-based over ~11 file-scope variables while
 * the core is object-based. It must land and be gated on its own, behind the
 * offline differential first and confirmed on the board after -- a green offline
 * gate GATES a boot, it never proves the hardware works.
 */

/*
 * G.984.3 PLOAM activation FSM (drives the ONU O1 -> O5).
 *
 * The MAC is a software-PLOAM design: a poll timer drains the downstream PLOAM
 * receive buffer, runs the state machine, and composes the upstream
 * Serial_Number_ONU. The OLT broadcasts Upstream_Overhead (type 0x01, ONU-ID
 * 0xff) to acquire unregistered ONUs; we answer with our Serial_Number_ONU, then
 * accept Assign_ONU-ID (0x03) and Ranging_Time (0x04) to reach O5.
 *
 * The per-board serial stays OUT of the image: `luna_gpon.onu_sn=` (and
 * gpon_provision's factory value) supply it, as 4 ASCII ID chars + 8 hex digits.
 */

/*
 * ===== G.984.3 PLOAM activation FSM (drives the ONU O1 -> O5) =====
 *
 * The MAC is a software-PLOAM design: a poll timer drains the downstream PLOAM
 * receive buffer, runs the activation state machine, and composes upstream
 * Serial_Number_ONU / takes the OLT-assigned ONU-ID and ranging delay. The OLT
 * (observed live) broadcasts Upstream_Overhead (type 0x01, ONU-ID 0xff) to
 * acquire unregistered ONUs; we answer with our Serial_Number_ONU, then accept
 * Assign_ONU-ID (0x03) and Ranging_Time (0x04) to reach O5.
 *
 * Per-board serial number stays OUT of the image: default below is overridable
 * via the `luna_gpon.onu_sn=` module/cmdline param (and is wired to gpon_provision's
 * factory value for the fleet). Format (G.984.3 ONU-SN): 4 ASCII ID chars + 8 hex digits.
 */

/* Decode "AAAAhhhhhhhh" into the 8-byte G.984.3 ONU-SN, through the common codec
 * (drivers/net/gpon/gpon_sn.c). This was a second decoder of one wire format and
 * it did not agree with the core's: hex_to_bin() returns -1 on a bad digit, and
 * storing that in a u8 made the byte 0xff, so "XPON1234567Z" silently decoded to
 * a serial nobody asked for; a short string left the rest of the bytes as found.
 * The core REFUSES malformed input and leaves `out` untouched, which is what
 * both call sites wanted -- a refusal now reads as "not a different serial"
 * instead of manufacturing one and triggering a re-range. */
static void gpon_parse_sn_into(u8 *out, const char *s)
{
	if (!s || !*s)
		return;		/* nothing provisioned yet: not an error, leave `out` alone */
	if (gpon_sn_parse(s, out))
		pr_warn("gpon: ONU-SN \"%s\" is not 4 ID chars + 8 hex digits -- keeping the serial in force\n",
			s);
}

static void gpon_parse_sn(const char *s)
{
	gpon_parse_sn_into(gpon_sn_bytes, s);
}

/*
 * The ONU-SN, for the OMCI shell. It lives here because PLOAM owns the identity;
 * the OMCI responder needs the same eight bytes for ME 256 (ONU-G) and must not
 * keep a second copy -- that is how two decoders of one serial came to disagree.
 */
void gpon_onu_sn(u8 out[8])
{
	int i;

	for (i = 0; i < 8; i++)
		out[i] = gpon_sn_bytes[i];
}
EXPORT_SYMBOL(gpon_onu_sn);


/* True when `s` decodes to a DIFFERENT ONU-SN than the one in force.  The
 * caller uses this to decide whether a write is an identity change (re-range)
 * or a rewrite of the same serial (do nothing). */
static bool gpon_sn_differs(const char *s)
{
	u8 want[8];

	/* Match gpon_parse_sn_into()'s "leave untouched what the string does not
	 * supply" behaviour: seed from the serial in force, so a SHORT string
	 * compares as equal exactly when it leaves every byte alone. */
	memcpy(want, gpon_sn_bytes, sizeof(want));
	gpon_parse_sn_into(want, s);
	return memcmp(want, gpon_sn_bytes, sizeof(want)) != 0;
}

/*
 * Compose and transmit a 12-byte upstream PLOAM on the given US_PLOAM_IND queue.
 * Required order: select TYPE and CLEAR ENQ, write the 6 data words, enable HW
 * CRC + ONU-ID override, THEN pulse ENQ 0->1. The enqueue is edge-triggered, so
 * ENQ must be cleared before being set or a repeat is a no-op -- the original
 * bug left ENQ stuck high with no edge and nothing transmitted.
 */
static void gpon_send_cpu_ploam(u8 queue, const u8 m[12])
{
	u32 ind;
	int i;

	if (!READ_ONCE(luna_activation_ready))
		return;

	/* The CPU US-PLOAM path has a SINGLE transmit buffer and a self-clearing ENQ
	 * bit: the HW sends the enqueued message in the next granted PLOAM slot and
	 * clears ENQ. Back-to-back sends would otherwise overwrite the buffer before
	 * the previous message went out, so the OLT never receives the important
	 * Acknowledge. Wait, bounded, for ENQ to self-clear before reloading. */
	for (i = 0; i < 1000; i++) {
		if (!(gpon_rd(GPON_GTC_US_PLOAM_IND) & GPON_US_PLM_ENQ))
			break;
		udelay(5);
	}

	ind = gpon_rd(GPON_GTC_US_PLOAM_IND);
	ind &= ~((0x7u << GPON_US_PLM_TYPE_SHIFT) | GPON_US_PLM_ENQ);
	ind |= ((u32)queue << GPON_US_PLM_TYPE_SHIFT);		/* select queue, ENQ=0 */
	gpon_wr(GPON_GTC_US_PLOAM_IND, ind);

	for (i = 0; i < 6; i++)
		gpon_wr(GPON_GTC_US_PLOAM_DATA + i * 4,
			((u32)m[2 * i] << 8) | m[2 * i + 1]);
	/* The comment said 0x13 and the expression made 0x03 (fixed 2026-09-08):
	 * CRC_GEN_EN|ONUID_OVRD is bits 1|0, the word stock rests at is 0x13, and the
	 * missing bit is PLM_FLUSH_BUF -- so this write, issued immediately before
	 * every ENQ edge, put the transmit buffer back into the FLUSHING state on
	 * every CPU PLOAM. Measured on the G24W: enq_cleared=1(0us), cputx=0 on every
	 * send, so the OLT received no Password and no Acknowledge. */
	gpon_wr(GPON_GTC_US_PLOAM_CFG, GPON_US_PLM_CFG_REST);	/* 0x13, as stock rests */

	ind |= GPON_US_PLM_ENQ;			/* ENQ 0->1 edge: transmit */
	gpon_wr(GPON_GTC_US_PLOAM_IND, ind);

	/* DBG (ploam_tx_dbg): did the HW actually TRANSMIT this CPU PLOAM? Poll ENQ
	 * self-clear (HW sends in the next granted US PLOAM slot + clears ENQ). The
	 * Serial_Number rides the auto-SN queue (0x6); the ACK/Password ride the urgent
	 * queue (0x1). Ground truth = OLT raises LOAi (never gets our ACKs) -> suspect the
	 * urgent-queue CPU TX never fires. Log the first N sends to compare per-queue. */
	if (ploam_tx_dbg) {
		static int dbgn;
		int j, cleared = 0;
		u32 i2;

		for (j = 0; j < 400; j++) {	/* up to ~2ms */
			i2 = gpon_rd(GPON_GTC_US_PLOAM_IND);
			if (!(i2 & GPON_US_PLM_ENQ)) { cleared = 1; break; }
			udelay(5);
		}
		if (dbgn++ < 40)
			pr_info("luna-gpon: PLM_TX q=%u enq_cleared=%d(%dus) IND=0x%08x urg_e=%d urg_f=%d nrm_e=%d nrm_f=%d cputx=%u autotx=%u\n",
				queue, cleared, j * 5, i2,
				!!(i2 & GPON_US_PLM_URG_EMPTY), !!(i2 & GPON_US_PLM_URG_FULL),
				!!(i2 & GPON_US_PLM_NRM_EMPTY), !!(i2 & GPON_US_PLM_NRM_FULL),
				gpon_us_misc_cnt(2), gpon_us_misc_cnt(3));
	}
}

static void gpon_send_sn(void)
{
	u8 m[12];

	m[0] = 0xff;			/* ONU-ID (unassigned)            */
	m[1] = PLM_US_SERIAL_NUMBER;	/* 0x01                           */
	memcpy(&m[2], gpon_sn_bytes, 8);/* ONU-SN: ID(4) + serial(4)      */
	m[10] = 0x00;			/* random delay (HW may fill)     */
	m[11] = 0x04;			/* G-bit set, power level 0       */

	gpon_send_cpu_ploam(PLM_US_QUEUE_SN, m);
	gpon_fsm_sn_tx++;
}

/*
 * Respond to a downstream Request_Password (0x09) with the US Password message
 * (type 0x02, 10-octet password). Ground truth, OLT poll 2026-06-13: the OLT sat
 * at O5 spamming Request_Password and Encrypted_Port-ID, never advancing, and
 * deactivated us with LOAi -- because this reply was never sent. The OLT is
 * SN-authenticated so the VALUE is ignored, but the handshake stalls without the
 * message. Sent three times on the urgent queue, as G.984.3 does over the
 * un-acked channel.
 */
static void gpon_send_password(void)
{
	u8 p[12] = { 0 };
	int i;

	p[0] = gpon_fsm_onu_id;		/* our assigned ONU-ID */
	p[1] = PLM_US_PASSWORD;		/* 0x02 */
	/* p[2..11] = 10-octet password, all zero = empty (OLT SN-auth ignores it) */
	for (i = 0; i < 3; i++)
		gpon_send_cpu_ploam(PLM_US_QUEUE_URG, p);
}

/*
 * Transmit a US Acknowledge (type 0x09) for a downstream PLOAM that requires one.
 * The OLT arms a post-ranging timer waiting for it and Deactivates the ONU after
 * ~43 s with no reply. @ds points at the full 13-byte DS message; the ack echoes
 * it and the HW fills the CRC. Sent on the urgent queue so it pre-empts the SN
 * burst.
 */
static void gpon_send_ack(const u8 *ds)
{
	u8 a[12] = { 0 };

	a[0] = gpon_fsm_onu_id;		/* our assigned ONU-ID (HW may override) */
	a[1] = PLM_US_ACKNOWLEDGE;	/* 0x09 */
	a[2] = ds[1];			/* acknowledged message type */
	a[3] = ds[0];			/* acknowledged message ONU-ID */
	a[4] = ds[1];			/* acknowledged message type (echo) */
	memcpy(&a[5], &ds[2], 7);	/* first 7 payload octets */
	gpon_send_cpu_ploam(PLM_US_QUEUE_URG, a);
}

/*
 * Respond to a downstream Request_key (0x0d): generate a 128-bit AES key and send
 * it as two upstream Encryption_Key (type 0x05) fragments (msg[2]=key_index,
 * msg[3]=row, msg[4..11]=8 key bytes). An ONU that never returns a key cannot
 * complete config. HW decryption is not programmed -- the OMCC is unencrypted --
 * so this just satisfies the OLT's key exchange.
 */
static u8 gpon_aes_key[16];
static u8 gpon_key_index;
static u32 gpon_aes_switch_time = 0xffffffff;	/* last Key_Switching_Time superframe (de-dup) */
static bool gpon_key_staged;			/* a valid AES key is loaded in the staged bank */

/* Program the 16-byte AES-128 key into the GPON hardware STAGED key bank. After
 * answering Request_Key the ONU MUST also load that same key into hardware; the
 * OLT then sends Key_Switching_Time and the HW promotes it at the given
 * superframe. Without this load the OLT treats the exchange as incomplete and
 * re-cycles Request_Key / Configure_Port-ID every ~15 s, never reaching OMCI.
 * Word i carries key[2i] (high byte) | key[2i+1] (low); words 0..7 = key 0..15. */
static void gpon_aes_stage_key(const u8 *key)
{
	int idx, i;

	gpon_wr(0x3010, 0x0000);			/* CFG_ACTIVE_KEY=0 (staged), REQ=0     */
	gpon_wr(0x3010, 0x8000);			/* KEY_CFG_REQ 0->1: config staged bank */
	for (idx = 0; idx < 8; idx++) {
		gpon_wr(0x3024, ((u32)key[2 * idx] << 8) | key[2 * idx + 1]);
		gpon_wr(0x3020, idx);			/* KEY_WORD_IDX=idx, KEY_WR_REQ=0       */
		gpon_wr(0x3020, idx | 0x8000);		/* KEY_WR_REQ 0->1: latch word idx      */
		for (i = 0; i < 200; i++) {		/* bounded; timeout != success          */
			if (gpon_rd(0x3020) & BIT(14))	/* KEY_WR_COMPL                         */
				break;
			udelay(5);
		}
	}
}

static void gpon_send_key(void)
{
	u8 m[12];
	int row, rep;

	get_random_bytes(gpon_aes_key, sizeof(gpon_aes_key));
	gpon_key_index++;
	/* Send the SAME key 3x (6 PLOAMs total), matching stock behavior (3 reps around the
	 * 2-fragment emit). The US PLOAM channel is lossy and the OLT re-issues Request_key
	 * rapidly when it does not receive a complete key, stalling config; the redundant
	 * triple-send maximises the chance the OLT accepts the key and proceeds. */
	for (rep = 0; rep < 3; rep++) {
		for (row = 0; row < 2; row++) {
			memset(m, 0, sizeof(m));
			m[0] = gpon_fsm_onu_id;		/* ONU-ID (HW may override)    */
			m[1] = PLM_US_ENCRYPT_KEY;	/* 0x05 Encryption_Key         */
			m[2] = gpon_key_index;
			m[3] = row;			/* fragment row 0/1            */
			memcpy(&m[4], gpon_aes_key + row * 8, 8);
			gpon_send_cpu_ploam(PLM_US_QUEUE_URG, m);
		}
	}
	/* Load the SAME key into the HW staged bank (the OLT waits for this before OMCI). */
	gpon_aes_stage_key(gpon_aes_key);
	gpon_key_staged = true;
	pr_info("luna-gpon: Request_key -> sent Encryption_Key idx %u (3x2 frags) + staged in HW\n",
		gpon_key_index);
}

/*
 * OMCI channel (OMCC) GEM datapath. After ranging the OLT assigns the OMCC GEM
 * port via Configure_Port-ID; install it at this chip's OMCI flow so DS OMCI GEM
 * frames are de-encapsulated and trapped to the CPU and US OMCI can egress.
 * Sequence: DS GEM-port CAM write, US GEM-port map, PON-IP OMCC bind.
 */
/* GEM_US_PORT_MAP_STRIDE and GPON_OMCC_FLOW are #defined at the TOP of this
 * file since 2026-09-05 (after the luna_gpon_regs.h include): gpon_proc_show()
 * sits ABOVE this section and read GEM_US_PORT_MAP[GPON_OMCC_FLOW] as a bare
 * 0x6500 for as long as the two names were only visible below it. */
/* Both flows this driver stamps into the US port map must be INSIDE it.  The
 * same predicate the Cortina shell uses, so one family cannot drift into a
 * private idea of what "in range" means; count is 1 because Luna stamps single
 * named flows and owns no slot RANGE. */
/* ★ ONE ASSERTION PER DECLARED CHIP, not one on "the" OMCC flow: the index is a
 * table field now, and a static_assert on a table field does not compile. Each
 * die's own constant is checked, so adding a chip whose OMCI flow runs past the
 * array is caught at build time exactly as before. */
static_assert(GPON_GEM_US_RANGE_OK(GPON_OMCC_FLOW_9602C, 1u, GEM_US_PORT_MAP_IDX_MAX),
	      "the RTL9602C OMCC flow index runs past the US port-map array");
static_assert(GPON_GEM_US_RANGE_OK(GPON_OMCC_FLOW_9603CVD, 1u, GEM_US_PORT_MAP_IDX_MAX),
	      "the RTL9603CVD OMCC flow index runs past the US port-map array");
static_assert(GPON_GEM_US_RANGE_OK(GPON_OMCC_FLOW_9607C, 1u, GEM_US_PORT_MAP_IDX_MAX),
	      "the RTL9607C OMCC flow index runs past the US port-map array");
static_assert(GPON_GEM_US_RANGE_OK(GPON_DATA_FLOW, 1u, GEM_US_PORT_MAP_IDX_MAX),
	      "the data flow index runs past the US port-map array");
/* GPON_DATA_FLOW(1) / the OLT's gem-port-id — the WAN data GEM (clean-room nas0-equivalent) —
 * are defined in luna_gpon_nic.h (shared with the eth driver's gpon0 TX descriptor). */
#define GPON_OMCC_DSQ_HIGH	2

/* Dedicated DATA T-CONT for the WAN data GEM. Stock rides it on its own data
 * T-CONT bound to the OLT-assigned data Alloc-ID (via OMCI ME262), NOT on the
 * OMCC's T-CONT 16: riding the OMCC's means the data US goes out on the mgmt
 * Alloc, so the OLT grants the data Alloc, sees it idle, and WITHHOLDS
 * downstream. */
#define GPON_DATA_ALLOC		256u	/* OLT data Alloc-ID for THIS OLT (consistent; from ME262) */
static int gpon_install_tcont(u8 tcont, u16 alloc);	/* fwd: data-GEM install binds the data T-CONT */
/* fwd: the Assign_Alloc-ID handler binds the data T-CONT through it. DECLARED
 * HERE and not only defined below, because the x86 native-FSM differential
 * (rtl9607c-test/native_fsm_impl.h) includes a SLICE of this file: the body
 * that calls it is inside the slice and the definition is not, so a
 * definition-only helper compiles on MIPS and breaks the offline gate. That
 * gate caught this the same hour the call landed. */
static void luna_data_tcont_from_assign(u16 alloc, bool assigned);

/* The CPU-side DS-OMCI count, or -1 when this board's Ethernet shell has no
 * OMCI datapath.  See GPON_OMCI_RX_UNAVAIL: an unported path must not print as
 * a device that received nothing. */
static int gpon_omci_rx_cnt(void)
{
	u32 v = rtl9602c_eth_omci_rx_count();

	return v == GPON_OMCI_RX_UNAVAIL ? -1 : (int)v;
}

/* Set a `bits`-wide entry at index `idx` in the packed pi-register array based
 * at `base`. The 32/bits-entries-per-word addressing is pi_packed_locate() in
 * flowcore_hash.c, x86-testable and SHARED with pi_packed_get so the two can no
 * longer agree only by parallel maintenance; its contiguous-pack predecessor
 * mis-addressed the 7-bit SID2QID array and the OLT never received the MIB
 * upload. Only the register read-modify-write is left here. */
static void pi_packed_set(u32 base, unsigned int idx, unsigned int bits, u32 val)
{
	struct pi_packed_slot slot = pi_packed_locate(base, idx, bits);

	pi_wr(slot.reg, pi_packed_insert(pi_rd(slot.reg), &slot, val));
}

/* Read the `bits`-wide entry back through the SAME locate, so a /proc readback
 * shows the TRUE value -- the old contiguous read showed the wrong SID's, which
 * is why the SID2QID mis-addressing stayed invisible across many boots. */
static u32 pi_packed_get(u32 base, unsigned int idx, unsigned int bits)
{
	struct pi_packed_slot slot = pi_packed_locate(base, idx, bits);

	return pi_packed_extract(pi_rd(slot.reg), &slot);
}

/*
 * Faithful port of the stock PON-MAC GPON mode-set branch, run as ONE ordered
 * block at O5: (1) invalidate every US SID's classify, (2) program the OMCI SID
 * triple in order, (3) PON/CPU-port RX_SPC, (4) [gated] SerDes re-commit, ending
 * on (5) the GMII_RX_EN rising edge that LATCHES the classify table into the
 * US-NIC RX engine. This driver set the SID classify but never re-pulsed that
 * edge, so the US-NIC kept its boot-time latch and dropped SID-64 OMCI before
 * the MAC (RX_OK=ERR=MISS=0).
 */
static void rtl9602c_ponmac_modeset_gpon(bool keep_data)
{
	unsigned int sid;

	if (!ponmac_modeset)
		return;

	/* (1) all-SID classify invalidation pre-pass, so the US-NIC resolves the
	 * OMCC SID cleanly at the re-latch, as stock does. */
	for (sid = 0; sid < swc->classify_sid_num; sid++) {
		if (sid == GPON_OMCC_FLOW || (keep_data && sid == GPON_DATA_FLOW))
			continue;
		pi_packed_set(PI_PON_SIDVALID, sid, 1, 0);
		pi_packed_set(PI_PON_SID2QID, sid, 7, swc->scratch_phys_qid & 0x7f);
	}

	/* (2) OMCI classify triple, in stock order (SID2QID -> SIDVALID -> OMCI_CFG). */
	pi_packed_set(PI_PON_SID2QID, GPON_OMCC_FLOW, 7, GPON_OMCC_PHYS_QID & 0x7f);
	pi_packed_set(PI_PON_SIDVALID, GPON_OMCC_FLOW, 1, 1);
	pi_field(PI_PON_OMCI_CFG, 6, 0, GPON_OMCC_FLOW);

	/* (2b) WAN data-GEM classify, re-asserted every modeset -- the all-SID loop
	 * above would otherwise wipe it, the same trap that hid the OMCC SID for ~30
	 * rounds. SID2QID is the data qid so the data US is reported to the OLT's DBA
	 * on the data Alloc; the legacy path rides the OMCC's T-CONT 16. */
	if (keep_data) {
		pi_packed_set(PI_PON_SID2QID, GPON_DATA_FLOW, 7,
			      luna_data_qid & 0x7f);
		pi_packed_set(PI_PON_SIDVALID, GPON_DATA_FLOW, 1, 1);
	}

	/* (3) PON-port + CPU-port RX_SPC: accept the sub-64B OMCI frame. Both the
	 * ports and the per-port stride are this chip's -- see SW_P_MISC_PORT(). */
	sw_field(SW_P_MISC_PORT(swc->sw, swc->sw->pon_port), 2, 2, 1);
	sw_field(SW_P_MISC_PORT(swc->sw, swc->sw->cpu_port), 2, 2, 1);

	/* (4) [risky, separately gated] SerDes RE-COMMIT: stock re-runs the SDS mode
	 * cycle inside the same window so the commit and the SID latch share one
	 * ordered window. Pulses CMD_SDS_RST_PS and can drop the locked DS framer. */
	if (serdes_recommit) {
		sw_field(SDS_CFG, 4, 0, SDS_MODE_OFF);		/* park = 0x1f      */
		sw_wr(WSDS_DIG_01, 0);
		sw_field(SW_SOFTWARE_RST, 0, 0, 1);		/* CMD_SDS_RST_PS   */
		mdelay(10);
		sw_field(WSDS_DIG_18, 12, 12, 1);		/* BEN_OE = 1       */
		sw_field(SDS_CFG, 4, 0, SDS_MODE_GPON);		/* commit GPON=0x08 */
	}

	/* (5) GMII re-latch: pulse GMII_RX_EN off->on so the US-NIC RX engine latches
	 * the now-complete SID classify table. This is the edge that was missing. */
	pi_wr(PI_IO_CMD_0_US, 0x90101050u);	/* GMII off */
	udelay(50);
	pi_wr(PI_IO_CMD_0_US, 0x90101070u);	/* GMII on -> rising edge latches classify */

	pr_info("luna-gpon: ponmac_modeset_gpon: re-latched US-NIC SID classify (serdes_recommit=%u)\n",
		serdes_recommit);
}

/* The OMCC GEM Port-ID currently mapped to the management flow, or 0 before one
 * is. Read by the DS OMCI snoop so the core's data-GEM decision can refuse to
 * adopt the MANAGEMENT gem as the WAN one -- the refusal that would otherwise
 * point user traffic at the OMCC. */
static u16 gpon_omcc_gem_port;

u16 gpon_omcc_gem(void)
{
	return gpon_omcc_gem_port;
}

static int gpon_install_omcc(u16 gem)
{
	int rc;

	/* the 12-bit G.984.3 wire width is the core's, once (gpon_gem_us.h):
	 * this site spelled it by hand, so a change to the field would have
	 * had to be made in two places to be made at all. */
	gpon_omcc_gem_port = gpon_gem_us_port_id(gem);

	/* Retire the owned data queue before a global table reset can erase its
	 * ownership evidence. Failed cleanup leaves Configure_Port-ID retryable. */
	rc = luna_data_retire();
	if (rc)
		return rc;
	rc = gpon_ds_cam_clear_all();
	if (rc)
		return rc;

	/* DS GEM-port CAM: map gem -> the OMCC flow, mark isOMCI. One indirect-CAM
	 * transaction through the shared gpon_gtc_ds_port_write() reading
	 * luna_gpon_chip; same addresses, values and access counts as the
	 * hand-spelled form, proven by gpon_regtable_diff_test. */
	rc = luna_port_cam_write(GPON_OMCC_FLOW, gem);
	if (rc < 0) {
		pr_err("luna-gpon: OMCC DS GEM install timeout\n");
		return rc;
	}
	gpon_wr(GPON_GTC_DS_TRAFFIC_CFG + GPON_OMCC_FLOW * DS_TRAFFIC_CFG_STRIDE,
		DS_TRAFFIC_IS_OMCI);

	/* DS OMCI PTI: tell the GTC how to detect the end of an OMCI GEM frame for
	 * reassembly. At reset this register is 0, so the GTC never recognises an OMCI
	 * frame boundary and drops every downstream OMCI frame. */
	gpon_wr(GPON_GTC_DS_OMCI_PTI, DS_OMCI_PTI_VAL);
	/* A live online stock ONU sets the ADJACENT DS-PTI registers to the same value
	 * -- GPON_GTC_DS_TDM_PTI and GPON_GTC_DS_ETH_PTI. Setting only the OMCI one
	 * produced NOTHING from the DS de-encap: reassembly needs all three. */
	gpon_wr(GPON_GTC_DS_TDM_PTI, DS_OMCI_PTI_VAL);	/* stock O5 = 0x11 */
	gpon_wr(GPON_GTC_DS_ETH_PTI, DS_OMCI_PTI_VAL);	/* stock O5 = 0x11 */

	/* GEM DS pass config: WITHOUT NON_MULTICAST_PASS the GTC drops every unicast
	 * downstream GEM frame BEFORE de-encapsulation -- including the OMCI the OLT
	 * sends unicast on the OMCC port -- and the register resets to 0. Written
	 * UNCONDITIONALLY: a live online stock ONU runs the full value with the US
	 * stable, so the earlier "US stall" fear came from the partial value's
	 * broadcast/bit0 mishandling, not from opening the gate. */
	gpon_wr(GPON_GEM_DS_MC_CFG, GEM_DS_MC_CFG_VAL);

	/* GEM-DS reassembly flush/forward timer. The RESET default has OMCI_TR_MODE=1
	 * and stock only FIELD-writes the assemble timer, preserving it; a full write
	 * of 0x10 CLEARS OMCI_TR_MODE, and with OMCI transparent mode off the DS GEM
	 * de-assembler does not pass OMCI frames at all. Field-write only. */
	gpon_field(GPON_GEM_DS_FRM_TIMEOUT, 8, 8, 1);	/* OMCI_TR_MODE = 1 (stock reset default) */
	gpon_field(GPON_GEM_DS_FRM_TIMEOUT, 4, 0, 16);	/* ASSM_TIMEOUT_FRM = 16 frames */

	/* US GEM-port map for the OMCC flow: stamp the OLT-assigned GEM Port-ID into
	 * GEM_US_PORT_MAP. The stride is 4 (32-bit words), the same as
	 * DS_TRAFFIC_CFG; a 0x20 stride is the regression that wrote this into the
	 * stat counter and left the real slot unmapped, so the GEM-US engine had no
	 * GEM-port for the OMCI flow and never drained its queue.
	 * The offset arithmetic lives in the shared gpon_gtc_us_gem_stamp(), fed by
	 * luna_gpon_chip -- this chip's offsets as DATA. */
	if (!gpon_gtc_us_gem_stamp(&gpon_io, &luna_gpon_chip.gtc,
				   GPON_OMCC_FLOW, gpon_gem_us_port_id(gem)))
		pr_err("luna-gpon: gpon_chip table declares no US port map -- OMCC gem %u not stamped\n",
		       gem);

	/* Re-assert PONIP_DBG_CTRL_US on every OMCC (re-)install: the initial pbo_init
	 * write sets DBG_IGNORE_TAG, but a DEACT/re-range cycle resets the US-NIC debug
	 * block and clears it. Without it the CPU tag is not stripped from US frames
	 * before GEM encapsulation, so the OMCI is malformed on every re-range. */
	pi_wr(PI_PONIP_DBG_CTRL_US, 0x00086000u);	/* stock (DBG_IGNORE_TAG=1) */

	/* PON-IP SID-valid + OMCI-SID, confirmed against live stock at O5. SID_Q_MAP_DS
	 * is 0 on stock: DS OMCI reaches the CPU purely via the GMAC CPU-tag trap, not
	 * a PBO queue, and our 2 MISROUTED the de-encapped OMCI away from the CPU. */
	pi_packed_set(PI_PON_SID2QID, GPON_OMCC_FLOW, 7, GPON_OMCC_PHYS_QID & 0x7f);
	pi_packed_set(PI_PON_SIDVALID, GPON_OMCC_FLOW, 1, 1);
	pi_field(PI_PON_OMCI_CFG, 6, 0, GPON_OMCC_FLOW);
	pi_packed_set(PI_PON_SID_Q_MAP_DS, GPON_OMCC_FLOW, 2, 0);
	/* Enrol THIS CHIP's OMCC SID in US counter-mask group 0: RX_SID_GOOD/BAD_CNT_US
	 * count ONLY mask-enrolled SIDs, while the UNGATED ingest counter is
	 * PKT_OK_CNT_US.
	 * GEOMETRY [tier 3, the RTL9603CVD chipdef]: CNT_MASK_US is ONE BIT PER SID,
	 * four groups of a 128-bit mask, so group g's word for SID s is
	 * 0x20a8 + g*16 + (s/32)*4, bit s%32 -- and the arithmetic cross-checks itself,
	 * since four 16-byte groups end exactly where the chipdef puts RX_DROP_CNT_US.
	 * This was a literal pi_wr(0x20b0, ...) enrolling SID 64, the RTL9602C's OMCC,
	 * while this chip's is 127 -- so on the G24W every us_rxsid readout was
	 * structurally zero WHATEVER the upstream OMCI did, and was read as "the US
	 * OMCI never reached the US-NIC". A witness that cannot fire is worse than no
	 * witness: it answers. */
	{
		u32 mask_w = 0x20a8 + (GPON_OMCC_FLOW / 32) * 4;

		pi_wr(mask_w, pi_rd(mask_w) | BIT(GPON_OMCC_FLOW % 32));
	}

	/* Read the packed entries back through pi_packed_get, the mirror of
	 * pi_packed_set, to confirm SID2QID landed at the TRUE word: the old per-word
	 * math wrote the wrong one and this readback was blind to it. */
	pr_info("luna-gpon: pi readback sid2qid[64]=%u sidvalid[64]=%u sidqmapds[64]=%u\n",
		pi_packed_get(PI_PON_SID2QID, GPON_OMCC_FLOW, 7),
		pi_packed_get(PI_PON_SIDVALID, GPON_OMCC_FLOW, 1),
		pi_packed_get(PI_PON_SID_Q_MAP_DS, GPON_OMCC_FLOW, 2));

	/* Arm the NIC OMCI trap so DS OMCC frames reach the CPU netdev, and hand the
	 * eth driver this board's ONU-SN so its ONU-G reply matches what was ranged. */
	rtl9602c_eth_set_omci_sid(GPON_OMCC_FLOW);
	rtl9602c_eth_set_omci_identity(gpon_sn_bytes);

	/* Stock-ordered GPON mode-set classify block plus GMII re-latch. The SID
	 * classify above is set AFTER gpon_pbo_init's boot GMII edge, so the edge must
	 * be re-pulsed or the US-NIC never latches the classification. */
	rtl9602c_ponmac_modeset_gpon(gpon_data_installed);

	/* GMII re-latch AFTER ALL US-NIC config is written (GEM port map, DBG_CTRL,
	 * SID2QID, SIDVALID, OMCI_CFG, scheduler) -- the working firmware's order.
	 * The GEM-US engine latches its config at the GMII_RX_EN rising edge, so a
	 * latch fired before the port-map write leaves the engine with no port map for
	 * the OMCC flow: no US OMCI, no OMCI responses, "Laser out", churn-lock. */
	if (relatch_us) {
		pi_wr(PI_IO_CMD_0_US, 0x90101050u);	/* GMII RX OFF */
		pi_wr(PI_IO_CMD_0_US, 0x90101070u);	/* GMII RX ON -> re-latch */
		pr_info("luna-gpon: relatch_us: re-pulsed GMII_RX_EN after OMCC install (io0_us=0x%08x)\n",
			pi_rd(PI_IO_CMD_0_US));
	}

	/*
	 * Full US-feed FIFO re-arm at O5. The O3 TX-PLL relock is a SerDes reset and
	 * re-parks the edge-armed US-feed AFTER the pre-ranging datapath_rearm, so the
	 * GEM-US TX bank underflows on the first grant. relatch_us only re-pulses
	 * GMII; this re-runs the FULL feed FIFO edge with no WSDS soft-reset, so the
	 * DS lock is kept.
	 */
	if (o5_feed_rearm)
		gpon_us_feed_rearm_light();

	/* Re-assert AUTO_PROC_SSTART at O5: the HW's per-grant SStart auto-processing
	 * that STARTS the US burst. Written once at init, but the ranging reset can
	 * clear this US-side register and leave the framer parked on grants. */
	if (o5_sstart && READ_ONCE(luna_activation_ready)) {
		gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_UNLOCK);
		gpon_field(GPON_GTC_US_PROC_MODE, 0, 0, 1);	/* US_PROC_MODE.AUTO_PROC_SSTART = 1 */
		gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_LOCK);
		pr_info("luna-gpon: O5 re-asserted AUTO_PROC_SSTART (0x5200 bit0=%u)\n",
			gpon_rd(GPON_GTC_US_PROC_MODE) & 1u);
	}

	pr_info("luna-gpon: OMCC installed gem=%u flow=%u (compl %d)\n",
		gem, GPON_OMCC_FLOW, rc);
	return 0;
}

/*
 * Install the WAN data GEM (the OLT's wire gem-port-id) on GPON_DATA_FLOW as a
 * BRIDGED, non-OMCI datapath -- the clean-room nas0 equivalent.
 *   DS: gem -> flow CAM, PLAIN de-encap so the de-assembler passes raw Ethernet
 *       to the PON switch port and the eth driver's demux hands it to gpon0.
 *   US: GEM_US_PORT_MAP stamps the gem-id on frames the gpon0 TX path steers,
 *       and SID2QID routes them to the T-CONT's grants.
 * One-shot, re-armed on Deactivate. Requires the OMCC up first.
 */

/* Luna OMCI ownership. The selected NIC owns the storage; this family owner
 * serializes all model accesses and accepts queued RX only on the GPON timer.
 * No RX descriptor is retained. Overflow is counted before common acceptance.
 * A provider cookie is the queue origin: attach/detach and identity replacement
 * fence the timer and empty the queue before that origin can change. */
#define LUNA_OMCI_QUEUE_LEN 64u
#define LUNA_OMCI_POLL_BUDGET 8u
struct luna_omci_frame {
	u8 msg[OMCI_LEN];
	u16 len;
};
static_assert(sizeof(struct luna_omci_frame) == 50);
static DEFINE_SPINLOCK(luna_omci_lock);
static struct {
	struct omci_onu *onu;
	void *cookie;
	int (*tx)(void *, const u8 *, unsigned int);
	void (*tx_fence)(void *);
	int (*uni_admin_set)(void *, unsigned int, bool);
	struct luna_omci_frame queue[LUNA_OMCI_QUEUE_LEN];
	struct omci_data_binding binding;
	u16 head, count;
	u8 seed;
	u32 queued, overflow, closed, accepted, resets, tx_errors;
} luna_omci;
static bool luna_data_admitted;
static struct {
	struct gpon_data_armed armed;
	bool alloc_dirty, queue_dirty, gem_dirty, sid_dirty, stamp_dirty;
	int error;
} luna_data;

bool luna_gpon_data_ready(void)
{
	return READ_ONCE(luna_data_admitted) && READ_ONCE(luna_activation_ready);
}
EXPORT_SYMBOL(luna_gpon_data_ready);

/* Process-context NIC reset exclusion. The timer is the sole accepted-OMCI
 * and CAM/scheduler consumer; no model lock is held while waiting for it. */
void luna_gpon_nic_reset_begin(void)
{
	mutex_lock(&bosa_lock);
	if (luna_driver_ready)
		timer_delete_sync(&gpon_fsm_timer);
	luna_data_suspend();
}
EXPORT_SYMBOL(luna_gpon_nic_reset_begin);

void luna_gpon_nic_reset_end(void)
{
	luna_resume_poll();
	mutex_unlock(&bosa_lock);
}
EXPORT_SYMBOL(luna_gpon_nic_reset_end);


/*
 * The board's UNI panel, read from its own device tree.
 *
 * ★ IT IS PER BOARD AND IT IS NOT COMPUTABLE.  Both boards on this bench
 *   report four PPTP Ethernet UNIs, and their PhysicalPortId runs 3,2,1,0 on
 *   the X400AXF against 0,1,2,3 on the G24W -- OPPOSITE orders -- while the
 *   G24W numbers its fourth instance 0x0401 and carries SIX UNI-Gs against its
 *   four Ethernet UNIs.  Nothing here may derive one list from the other.
 *
 * ★ A BOARD THAT DECLARES NOTHING KEEPS THE CORE'S SINGLE-UNI DEFAULT, which
 *   is what every board had before this and is never a silent downgrade: the
 *   node is the declaration, and half a node is refused rather than guessed.
 */
/* This board's declared Ethernet UNIs, as SWITCH PORT indices, in the same
 * order as the instance list.  Empty means modelled and answered, never
 * applied -- said out loud by the reader below, never guessed. */
static u8 luna_uni_port[OMCI_UNI_MAX];
static u8 luna_uni_port_n;

/*
 * The COMMON pending/apply owner for both Luna boards.
 *
 * ★ IT RUNS IN A WORK ITEM because a backend drives an indirect PHY window
 *   under a mutex, and the OMCI path that accepts the Set holds a spinlock.
 *
 * ★ A FAILED APPLY STAYS OWED: the slot is re-armed in the model and the work
 *   reschedules.  A board with no backend, or no declared port for that slot,
 *   keeps the obligation WITHOUT a retry timer -- a permanent configuration
 *   gap earns no spin.
 *
 * ★★ AND OUR PHYSICAL BEHAVIOUR IS A DECLARED IMPROVEMENT, NOT THE VENDOR'S
 *    POLICY.  Own stock differs even between these two boards: the G24W
 *    applies whatever the UNI-G capability says, and the X111W SKIPS the
 *    physical application when that capability is 1.  We apply on every
 *    declared instance, because "the OLT locks a subscriber port and the port
 *    stops forwarding" is the capability being certified.
 */
static void luna_uni_apply_work_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(luna_uni_apply_work, luna_uni_apply_work_fn);

/*
 * The enqueue gate.  Read and written under the model spinlock so that the
 * check and the queueing are ONE critical section: a bare read of a stopping
 * flag followed by a queue lets a closure cancel in between and leave the work
 * armed over state it has just finished tearing down.
 */
static bool luna_uni_closing;

static void luna_uni_apply_queue(unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&luna_omci_lock, flags);
	/* ★★★ AND NOT BEFORE THE DRIVER IS READY.  A module parameter can fire at
	 *     parse_args() inside start_kernel(), which is BEFORE the workqueue
	 *     subsystem exists -- and `luna_gpon.onu_sn=` on the kernel command
	 *     line reaches the identity reset, which kicks this.  Queueing there
	 *     WARNs and the board never finishes booting: measured on the G24W,
	 *     where the console went silent and the LAN never came up with the
	 *     bootarg and the board served fine without it.
	 *
	 *     Nothing is lost by refusing: the obligation lives in the model's
	 *     changed mask, which the reinit carries, and the first kick after
	 *     the driver is up applies it.  luna_resume_poll() has had the same
	 *     guard all along, for the same reason.
	 */
	if (!luna_uni_closing && luna_driver_ready)
		schedule_delayed_work(&luna_uni_apply_work, delay);
	spin_unlock_irqrestore(&luna_omci_lock, flags);
}

/*
 * ★★ bosa_lock IS THE LIFETIME OWNER, HELD ACROSS THE WHOLE APPLY.
 *
 * The backend sleeps on an indirect PHY window, so the model spinlock cannot
 * span the call -- and a pointer copied under the spinlock and used after it is
 * not a reference: a detach or a NIC reset landing in the gap frees the cookie
 * the copy still names, and a failure would re-arm the obligation on a model
 * that has since been replaced.  This mutex is the driver's process-context
 * owner, taken by every teardown, so holding it from the snapshot through the
 * callback and the retry decision is what makes the cookie still attached when
 * the backend is entered.  The model spinlock is taken only for the model
 * copies inside it.
 *
 * ⚠ A CLOSURE THEREFORE NEVER CANCELS THIS WORK WHILE HOLDING bosa_lock --
 *   the worker needs it to finish.  detach() does not have to cancel at all:
 *   its own bosa_lock acquisition waits for an active callback and then nulls
 *   the attachment, so the next run sees NULL and consumes nothing.
 */
static void luna_uni_apply_work_fn(struct work_struct *w)
{
	int (*set)(void *, unsigned int, bool);
	u8 changed, admin[OMCI_UNI_MAX], n, i;
	struct omci_onu *onu;
	unsigned long flags;
	void *cookie;
	bool retry = false;

	(void)w;
	mutex_lock(&bosa_lock);
	spin_lock_irqsave(&luna_omci_lock, flags);
	onu = luna_omci.onu;
	cookie = luna_omci.cookie;
	set = luna_omci.uni_admin_set;
	if (!onu) {
		spin_unlock_irqrestore(&luna_omci_lock, flags);
		mutex_unlock(&bosa_lock);
		return;		/* nothing consumed, so nothing is lost */
	}
	changed = omci_uni_take_changed(&onu->pptp_eth_uni);
	n = onu->pptp_eth_uni.n;
	for (i = 0; i < n && i < OMCI_UNI_MAX; i++)
		admin[i] = onu->pptp_eth_uni.admin[i];
	spin_unlock_irqrestore(&luna_omci_lock, flags);

	for (i = 0; changed && i < n && i < OMCI_UNI_MAX; i++) {
		int rc;

		if (!(changed & BIT(i)))
			continue;
		if (!set || i >= luna_uni_port_n) {
			pr_warn_once("luna-gpon: UNI slot %u has no %s; its administrative state is modelled and not applied\n",
				     i, set ? "declared switch port" : "apply backend");
			rc = -ENODEV;
		} else {
			rc = set(cookie, luna_uni_port[i], admin[i] != 0);
		}
		if (!rc)
			continue;		/* applied: the obligation is discharged */
		/* RE-ARMED ON FAILURE ONLY, on the very model the snapshot came
		 * from: bosa_lock has kept it attached for the whole call. */
		spin_lock_irqsave(&luna_omci_lock, flags);
		omci_uni_mark_changed(&onu->pptp_eth_uni, i);
		spin_unlock_irqrestore(&luna_omci_lock, flags);
		if (set && i < luna_uni_port_n)
			retry = true;	/* transient: the backend refused */
	}
	if (retry)
		luna_uni_apply_queue(HZ);
	mutex_unlock(&bosa_lock);
}

void luna_uni_apply_kick(void)
{
	luna_uni_apply_queue(0);
}
EXPORT_SYMBOL(luna_uni_apply_kick);

/* Shut the gate.  Named, and here beside the gate rather than spelled inline in
 * the unload path, so the host differential executes the SHIPPING closure
 * instead of a copy of it. */
static void luna_uni_apply_close(void)
{
	unsigned long flags;

	spin_lock_irqsave(&luna_omci_lock, flags);
	luna_uni_closing = true;
	spin_unlock_irqrestore(&luna_omci_lock, flags);
}

#ifdef CONFIG_GPON_OMCI_DIAG
/*
 * ★★★ A TEMPORARY DIAGNOSTIC INGRESS, AND IT COMES BACK OUT AFTER THE
 *      MEASUREMENT.  It exists to prove ON A BOARD that an administrative lock
 *      reaches the subscriber port, on both Luna boards, from ONE seam.
 *
 * Writing "<instance-hex> <0|1>" builds a baseline G.988 ME 11 Set -- message
 * type 0x08 with AR=AK=0, class 0x000b, attribute mask 0x0800, the
 * administrative state byte at offset 10 -- finalizes it through the COMMON
 * responder's own omci_finalize(), and submits it through luna_omci_enqueue(),
 * the very entry the NIC uses for a frame off the wire.  The real queue, the
 * real FSM timer, the real model and the real apply owner all run; nothing
 * here reaches into the model.
 *
 * ⚠ CONFIG_GPON_OMCI_DIAG DEFAULTS TO y, so this gate is NOT a safety net.  An
 *   ONU that can be told from its own CPU to lock a subscriber port is not a
 *   thing to ship, and the revert is part of the measurement, not a follow-up.
 *
 * ⚠ A LOCAL INJECTION IS A DIFFERENT CLAIM FROM AN OLT-DRIVEN ONE.  It
 *   certifies the ONU's own path: no OLT write, no forged upstream response.
 *
 * ⚠ AND THE ACCEPTANCE IS NOT THE COMPLETION.  This logs that the request was
 *   ADMITTED; whether the port moved is the backend's own read-back line, and
 *   a cleared `changed` mask means only that the worker consumed it -- the
 *   callback may still be running.
 */
static u16 luna_uni_test_tci = 0xd100;

/*
 * The inventory a board run needs to RESTORE what it found.
 *
 * ★ THE LIFETIME OWNER IS TAKEN FIRST, and that is what makes the pending mask
 *   mean anything: the worker CLEARS it before the backend runs, so a zero read
 *   without holding bosa_lock cannot tell "applied" from "being applied right
 *   now".  Holding it proves no callback is in flight; the model spin is taken
 *   inside it only for the copies.
 */
static void luna_uni_test_show(void)
{
	u8 admin[OMCI_UNI_MAX] = { 0 }, port[OMCI_UNI_MAX] = { 0 };
	u16 inst[OMCI_UNI_MAX] = { 0 };
	u8 changed = 0, n = 0, port_n = 0, i;
	struct omci_onu *onu;
	unsigned long flags;

	mutex_lock(&bosa_lock);
	spin_lock_irqsave(&luna_omci_lock, flags);
	onu = luna_omci.onu;
	if (onu) {
		n = onu->pptp_eth_uni.n;
		changed = onu->pptp_eth_uni.changed;
		for (i = 0; i < n && i < OMCI_UNI_MAX; i++) {
			inst[i] = onu->pptp_eth_uni.inst[i];
			admin[i] = onu->pptp_eth_uni.admin[i];
		}
	}
	/* ★ THE PORT MAP IS COPIED IN THE SAME CRITICAL SECTION as the
	 *   instances.  Reading the global after the unlock lets an attach or a
	 *   re-declaration land in between and print OLD instances against a NEW
	 *   map -- a table that never existed, in the one output a board run uses
	 *   to decide what to restore. */
	port_n = luna_uni_port_n;
	memcpy(port, luna_uni_port, sizeof(port));
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	mutex_unlock(&bosa_lock);

	pr_info("luna-gpon: uni_test SNAPSHOT attached %d slots %u pending %#x\n",
		onu ? 1 : 0, n, changed);
	for (i = 0; i < n && i < OMCI_UNI_MAX; i++)
		pr_info("luna-gpon: uni_test SNAPSHOT slot %u inst %#06x admin %u port %d pending %u\n",
			i, inst[i], admin[i],
			i < port_n ? (int)port[i] : -1,
			!!(changed & BIT(i)));
}

static int luna_uni_test_set(const char *val, const struct kernel_param *kp)
{
	u8 msg[OMCI_LEN];
	unsigned int inst, admin, i;
	unsigned long flags;
	bool declared = false;
	u16 tci;
	int rc;

	(void)kp;
	if (!strncmp(val, "show", 4)) {
		luna_uni_test_show();
		return 0;
	}
	if (sscanf(val, "%x %u", &inst, &admin) != 2 || inst > 0xffff || admin > 1)
		return -EINVAL;

	/* ★ THE INSTANCE IS CHECKED AGAINST THE BOARD'S DECLARED PANEL.  A typo
	 *   would otherwise inject a Set for a UNI this board does not have, and
	 *   the run would measure nothing while looking like it measured. */
	spin_lock_irqsave(&luna_omci_lock, flags);
	if (luna_omci.onu)
		for (i = 0; i < luna_omci.onu->pptp_eth_uni.n; i++)
			if (luna_omci.onu->pptp_eth_uni.inst[i] == (u16)inst)
				declared = true;
	tci = ++luna_uni_test_tci;
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	if (!declared) {
		pr_err("luna-gpon: uni_test REFUSED: instance %#06x is not in this board's declared panel\n",
		       inst);
		return -ENODEV;
	}

	memset(msg, 0, sizeof(msg));
	omci_put_be16(msg, tci);		/* a UNIQUE tid per injection */
	msg[2] = OMCI_MT_SET;			/* AR=0 and AK=0: no response */
	msg[3] = 0x0a;				/* device identifier: baseline */
	omci_put_be16(msg + 4, OMCI_ME_PPTP_ETH_UNI);
	omci_put_be16(msg + 6, (u16)inst);
	omci_put_be16(msg + 8, 0x0800);		/* attribute 5: administrative state */
	msg[10] = (u8)admin;
	omci_finalize(msg);

	/* ★ THE REQUEST IS ANNOUNCED BEFORE IT IS SUBMITTED.  The FSM timer can
	 *   consume it and the apply can complete on another CPU before a line
	 *   printed after the call reaches the log, and a reader would then see
	 *   the completion BEFORE the request it belongs to. */
	pr_info("luna-gpon: uni_test REQUEST tid %#06x ME 11 #%#06x administrative state %u\n",
		tci, inst, admin);
	rc = luna_omci_enqueue(luna_omci.cookie, msg, OMCI_LEN);
	pr_info("luna-gpon: uni_test ADMITTED tid %#06x rc %d\n", tci, rc);
	return rc;
}

static const struct kernel_param_ops luna_uni_test_ops = { .set = luna_uni_test_set };
module_param_cb(uni_test, &luna_uni_test_ops, NULL, 0200);
MODULE_PARM_DESC(uni_test, "TEMPORARY board diagnostic: write \"<instance-hex> <0|1>\" to inject a local ME 11 administrative-state Set through the real OMCI queue, or \"show\" to log the declared UNI inventory (instance, administrative state, switch port, pending) taken under the lifetime owner");
#endif /* CONFIG_GPON_OMCI_DIAG */

static void luna_omci_declare_uni_panel(struct omci_onu *onu)
{
	struct device_node *np = of_find_node_by_path("/omci-uni");
	const void *pptp, *unig, *cap, *ports, *type;
	int pptp_len = -1, type_len = -1, unig_len = -1, cap_len = -1, ports_len = -1;
	const char *why = "";
	enum omci_uni_decl decl;

	if (!np)
		return;
	pptp = of_get_property(np, "ethernet-uni-instances", &pptp_len);
	unig = of_get_property(np, "uni-g-instances", &unig_len);
	cap = of_get_property(np, "uni-g-management-capability", &cap_len);
	/* ★ THE SWITCH PORT PER INSTANCE, FROM THE BOARD -- never computed.  The
	 *   G24W's own mapper answers 0x0101->0 .. 0x0401->3 and the X111W's
	 *   0x0101->1, 0x0102->0; the Cortina board runs the other way.  No
	 *   arithmetic on an instance id is right on more than one of them. */
	ports = of_get_property(np, "ethernet-uni-ports", &ports_len);
	/* ★ THE PLUG-IN TYPE PER INSTANCE, FROM THE BOARD -- never computed.
	 *   G.988 Expected/Sensed type states what the UNI IS, so a fixed
	 *   integrated port answers the same byte with its link down; both Luna
	 *   boards report 47 for their GE port and 24 for every FE one, and the
	 *   model answered a single 47 for all of them. */
	type = of_get_property(np, "ethernet-uni-types", &type_len);
	/* ⚠ THE RETURNED LENGTH IS THE ANSWER, NOT THE POINTER.  of_get_property
	 * leaves the length untouched when a property is ABSENT (so it stays
	 * -1), and a PRESENT but empty one reports length 0 with a value
	 * pointer that may legitimately be NULL.  Folding NULL back into -1
	 * would re-create, out here, exactly the empty-versus-absent confusion
	 * the core decoder exists to settle. */
	/* The BYTES go to the core, which owns every length rule.  Reading the
	 * properties as typed arrays here is what put four identical
	 * malformed-input defects in both families at once: an odd byte count
	 * silently lost its tail, a short or long capability list silently
	 * became all-ones or was truncated, and an empty list was rejected as
	 * absent -- restoring an instance the board does not have. */
	decl = omci_onu_declare_unis_be(onu, pptp, pptp_len, type, type_len,
					unig, unig_len, cap, cap_len, &why);
	if (decl == OMCI_UNI_DECL_BAD)
		pr_err("luna-gpon: /omci-uni REFUSED: %s -- keeping the single-UNI default\n",
		       why);
	luna_uni_port_n = 0;
	if (decl == OMCI_UNI_DECL_OK && pptp_len > 0) {
		if (ports && ports_len == pptp_len / 2 &&
		    ports_len <= (int)ARRAY_SIZE(luna_uni_port)) {
			memcpy(luna_uni_port, ports, ports_len);
			luna_uni_port_n = (u8)ports_len;
		} else {
			pr_warn("luna-gpon: /omci-uni has no usable ethernet-uni-ports (%d bytes for %d UNIs): the administrative state is modelled and NEVER APPLIED\n",
				ports_len, pptp_len / 2);
		}
	}
	of_node_put(np);
}

int luna_omci_attach(struct omci_onu *onu, void *cookie, u8 seed,
		    int (*tx)(void *, const u8 *, unsigned int),
		    void (*tx_fence)(void *),
		    int (*uni_admin_set)(void *, unsigned int, bool))
{
	unsigned long flags;
	int rc = 0;

	if (!onu || !cookie || !tx || !tx_fence)
		return -EINVAL;
	mutex_lock(&bosa_lock);
	spin_lock_irqsave(&luna_omci_lock, flags);
	if (luna_omci.onu) {
		rc = -EBUSY;
	} else {
		omci_onu_init(onu, gpon_sn_bytes, seed);
		luna_omci_declare_uni_panel(onu);
		luna_omci.onu = onu;
		luna_omci.cookie = cookie;
		luna_omci.tx = tx;
		luna_omci.tx_fence = tx_fence;
		luna_omci.uni_admin_set = uni_admin_set;
		luna_omci.seed = seed;
		luna_omci.head = luna_omci.count = 0;
		memset(&luna_omci.binding, 0, sizeof(luna_omci.binding));
	}
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	mutex_unlock(&bosa_lock);
	return rc;
}
EXPORT_SYMBOL(luna_omci_attach);

void luna_omci_detach(void *cookie)
{
	unsigned long flags;

	/* The only response/hardware consumer is this timer. Do not hold the
	 * model lock while waiting for it, or while taking the NIC TX lock. */
	mutex_lock(&bosa_lock);
	if (luna_driver_ready)
		timer_delete_sync(&gpon_fsm_timer);
	spin_lock_irqsave(&luna_omci_lock, flags);
	if (luna_omci.cookie == cookie) {
		WRITE_ONCE(luna_data_admitted, false);
		luna_omci.onu = NULL;
		luna_omci.cookie = NULL;
		luna_omci.tx = NULL;
		luna_omci.tx_fence = NULL;
		luna_omci.uni_admin_set = NULL;
		luna_omci.count = 0;
		memset(&luna_omci.binding, 0, sizeof(luna_omci.binding));
	}
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	luna_resume_poll();
	mutex_unlock(&bosa_lock);
}
EXPORT_SYMBOL(luna_omci_detach);

int luna_omci_enqueue(void *cookie, const u8 *msg, unsigned int len)
{
	struct luna_omci_frame *f;
	unsigned long flags;
	int rc = 0;

	if (!msg || len > 0xffffu)
		return -EINVAL;
	spin_lock_irqsave(&luna_omci_lock, flags);
	if (!luna_omci.onu || luna_omci.cookie != cookie ||
	    !READ_ONCE(luna_activation_ready)) {
		luna_omci.closed++;
		rc = -ESHUTDOWN;
	} else if (luna_omci.count == LUNA_OMCI_QUEUE_LEN) {
		luna_omci.overflow++;
		rc = -ENOSPC;
	} else {
		f = &luna_omci.queue[(luna_omci.head + luna_omci.count) % LUNA_OMCI_QUEUE_LEN];
		memset(f->msg, 0, sizeof(f->msg));
		memcpy(f->msg, msg, min_t(unsigned int, len, sizeof(f->msg)));
		f->len = len;
		luna_omci.count++;
		luna_omci.queued++;
	}
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	return rc;
}
EXPORT_SYMBOL(luna_omci_enqueue);

void luna_omci_set_sn(const u8 sn[8])
{
	unsigned long flags;

	if (!sn)
		return;
	spin_lock_irqsave(&luna_omci_lock, flags);
	if (luna_omci.onu)
		omci_onu_set_sn(luna_omci.onu, sn);
	spin_unlock_irqrestore(&luna_omci_lock, flags);
}
EXPORT_SYMBOL(luna_omci_set_sn);

void luna_omci_set_optical(u16 rx, u16 tx)
{
	unsigned long flags;

	spin_lock_irqsave(&luna_omci_lock, flags);
	if (luna_omci.onu)
		omci_onu_set_optical(luna_omci.onu, rx, tx);
	spin_unlock_irqrestore(&luna_omci_lock, flags);
}
EXPORT_SYMBOL(luna_omci_set_optical);

void luna_omci_rx_errors(u32 *bad_mic, u32 *runt)
{
	unsigned long flags;

	spin_lock_irqsave(&luna_omci_lock, flags);
	*bad_mic = luna_omci.onu ? luna_omci.onu->rx_bad_mic : 0;
	*runt = luna_omci.onu ? luna_omci.onu->rx_runt : 0;
	spin_unlock_irqrestore(&luna_omci_lock, flags);
}
EXPORT_SYMBOL(luna_omci_rx_errors);

/* Called only by the timer's existing AVC cadence. */
void luna_omci_report_oper_up(void)
{
	u8 msg[OMCI_LEN];
	unsigned long flags;
	int n = 0;

	spin_lock_irqsave(&luna_omci_lock, flags);
	if (luna_omci.onu && luna_gpon_data_ready())
		n = omci_onu_emit_veip_up_avc(luna_omci.onu, msg);
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	if (n > 0 && luna_omci.tx(luna_omci.cookie, msg, n)) {
		luna_omci.tx_errors++;
		/* The common GET projection reflects delivered operational state. */
		spin_lock_irqsave(&luna_omci_lock, flags);
		luna_omci.onu->avc_veip_up_sent = false;
		spin_unlock_irqrestore(&luna_omci_lock, flags);
	}
}
EXPORT_SYMBOL(luna_omci_report_oper_up);

static void luna_omci_identity_reset(const u8 sn[8])
{
	unsigned long flags;

	/* Caller has stopped the timer under bosa_lock before publishing the SN. */
	WRITE_ONCE(luna_data_admitted, false);
	spin_lock_irqsave(&luna_omci_lock, flags);
	luna_omci.count = luna_omci.head = 0;
	/* A published SN is an identity change on a model attach() already
	 * initialised, so the declared UNI panel is carried across. */
	if (luna_omci.onu)
		omci_onu_reinit(luna_omci.onu, sn, luna_omci.seed);
	memset(&luna_omci.binding, 0, sizeof(luna_omci.binding));
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	/* The reinit carries the panel and resets the administrative state, so
	 * whatever it unlocked is owed a physical turn. */
	luna_uni_apply_kick();
	gpon_ploam_data_alloc_reset(&luna_ploam);
	luna_data.armed.installed = false;
}

static void luna_data_alloc_changed(void *sh, u16 alloc, bool assigned)
{
	(void)sh;
	gpon_ploam_data_alloc_note(&luna_ploam, alloc, assigned);
}


/* ⚠ DEFINED HERE BECAUSE IT IS USED HERE FIRST. It sat ~600 lines below
 * its first use, which every native x86 differential and both emulators
 * accepted and the MIPS cross-build did not: `gpon_avc_sent undeclared`
 * on BOTH Luna boards. A host harness that includes a driver function in
 * isolation never sees file-scope ordering. */
/*
 * The A/B switch for the FSM rewire. Default OFF: this driver's own FSM runs,
 * byte for byte as before; core_fsm=1 sends the SAME downstream PLOAM to the
 * common core through the ops table below. A change this size lands as a live
 * A/B, not as a claim.
 * Declared here, far above the ops table, because gpon_fsm_set_state() below
 * has to consult it: that setter performs two transition ACTIONS that are the
 * core's OWN ops when the core is driving, and running them a second time from
 * inside the op is the double-execution ploam_fsm_diff records as W1.
 */
static bool core_fsm;
module_param(core_fsm, bool, 0644);
MODULE_PARM_DESC(core_fsm, "dispatch downstream PLOAM through the COMMON core FSM instead of this driver's own (default 0 = this driver's; the A/B for the rewire)");

static int gpon_avc_sent;	/* OMCI oper-state AVCs emitted this O5 (reset on re-range) */

/*
 * ★★★ THE ALLOC-ID -> T-CONT BIND BELONGS TO Assign_Alloc-ID.  That PLOAM
 *     message IS the OLT telling us to make the allocation operational, and
 *     the bind had moved out of here into luna_data_reconcile(), gated on the
 *     OLT's ME 268 GEM-CTP Create.
 *
 * ⚠ WHAT WAS WRONG IS THE TIMING, NOT A MISSING FUNCTION CALL, and the
 *   distinction matters because this family NOT calling the core predicate was
 *   a DELIBERATE, DOCUMENTED choice, not an oversight:
 *   OMCI-simulate/FINDING-tcont-rule-three-ways.md records all three
 *   mechanisms (stock asks the PON-MAC, Cortina asks the core predicate with a
 *   cached flag, Luna leans on the hardware CAM search) and says plainly that
 *   none of them is obviously wrong.  Calling gpon_gem_us_tcont_decide() here
 *   makes the shared rule EXPLICIT at the site and closes that file's
 *   "exactly one caller" observation -- it is a second, cheap gate beside the
 *   CAM search, and it is not what restored the WAN.
 *   ⚠ Its BIND_IS_OMCC arm is unreachable from here: the caller already
 *     refuses `alloc == gpon_fsm_onu_id` one line above.
 *
 * ⚠ THAT GATE IS A DEADLOCK ON AN OLT THAT WAITS FOR THE T-CONT FIRST.
 *   MEASURED 2026-09-14 on the X111W (OLT row 2/0): the OLT runs its whole
 *   provisioning script, stops at `Set me=262/32768` -- the T-CONT's Alloc-ID
 *   -- and Deactivates ~2 s into O5 without ever sending Create me=268/2.  We
 *   wait for the GEM; it waits for the T-CONT.  The 2026-09-12 image, which
 *   still bound here, reaches Create me=268/2 and leases: `T-CONT 8 <- alloc
 *   0x100 bound` at +4 ms, `DATA GEM installed` at +64 ms, DHCP at +2.3 s.
 *
 * ★ THE GEM CLASSIFY STAYS DEFERRED.  Installing the data GEM before the OLT
 *   created its own is what churn-locked a 2nd+ admit, and that reason is
 *   still good; only the T-CONT bind -- which is what Assign_Alloc-ID IS --
 *   comes back here.
 */
static void luna_data_tcont_from_assign(u16 alloc, bool assigned)
{
	enum gpon_gem_us_bind bind;

	if (!assigned || !data_tcont)
		return;
	bind = gpon_gem_us_tcont_decide(alloc, gpon_fsm_onu_id,
					READ_ONCE(luna_data.armed.alloc_bound));
	if (bind == GPON_GEM_US_BIND_TCONT) {
		gpon_install_tcont(GPON_DATA_TCONT, alloc);
		return;
	}
	pr_info("luna-gpon: Alloc 0x%x NOT bound to the data T-CONT: %s\n",
		alloc, gpon_gem_us_bind_name(bind));
}

static void luna_omci_poll(void)
{
	unsigned int budget = LUNA_OMCI_POLL_BUDGET;

	while (budget--) {
		struct omci_accepted accepted;
		struct luna_omci_frame frame;
		u8 response[OMCI_LEN];
		unsigned long flags;
		int n;

		spin_lock_irqsave(&luna_omci_lock, flags);
		if (!luna_omci.onu || !luna_omci.count) {
			spin_unlock_irqrestore(&luna_omci_lock, flags);
			break;
		}
		frame = luna_omci.queue[luna_omci.head];
		luna_omci.head = (luna_omci.head + 1) % LUNA_OMCI_QUEUE_LEN;
		luna_omci.count--;
		n = omci_onu_input_ex(luna_omci.onu, frame.msg, frame.len,
				      response, &accepted);
		if (accepted.kind != OMCI_ACCEPT_NONE) {
			luna_omci.accepted++;
			if (accepted.kind == OMCI_ACCEPT_RESET) {
				luna_omci.resets++;
				gpon_avc_sent = luna_ploam.avc_sent = 0;
			}
			omci_data_binding_snapshot(luna_omci.onu, gpon_omcc_gem_port,
					   GPON_MCAST_GEM, &luna_omci.binding);
		}
		spin_unlock_irqrestore(&luna_omci_lock, flags);
		if (n > 0 && luna_omci.tx(luna_omci.cookie, response, n))
			luna_omci.tx_errors++;
#ifdef CONFIG_GPON_OMCI_DIAG
		/* Preserve the shared request/response trace at its new owner. The
		 * common predicate limits only bulk diagnostics, never acceptance. */
		if (!gpon_omci_is_bulk(frame.msg, frame.len) || net_ratelimit()) {
			char line[128];

			if (gpon_omci_diag_line(frame.msg, frame.len,
					       n == OMCI_LEN ? response : NULL, n,
					       line, sizeof(line)))
				pr_info("luna-gpon: OMCI DS: %s\n", line);
		}
#endif
		/* An accepted mutation gets its hardware turn before another PDU can
		 * replace the snapshot. In particular Reset must not collapse into a
		 * following Create/Set that happens to recreate the same effective pair. */
		if (accepted.kind != OMCI_ACCEPT_NONE) {
			/* A Set the model accepted and a MIB Reset that unlocked
			 * every UNI owe the same physical turn, and an AR=0
			 * request owes it too: the obligation is the acceptance,
			 * never the response.  This runs in the FSM timer, so it
			 * only ever schedules. */
			luna_uni_apply_kick();
			break;
		}
	}
}

/* Legacy shell entry point remains a hint only. Accepted common model state
 * is the sole source of a WAN binding; a raw Create observation cannot arm it. */
void gpon_omci_note_gem_create(u16 port_id)
{
	(void)port_id;
}

int gpon_install_data_gem(void)
{
	int rc;

	if (gpon_data_installed)
		return 0;
	if (!(core_fsm ? luna_ploam.omcc_installed : gpon_omcc_installed))
		return -EAGAIN;

	/* DS GEM-port CAM: the OLT's gem -> the data flow, the same shared helper as
	 * the OMCC CAM. */
	luna_data.gem_dirty = true;
	rc = luna_port_cam_write(GPON_DATA_FLOW, gpon_data_gem_port);
	if (rc < 0) {
		pr_err("luna-gpon: DATA GEM DS install timeout\n");
		return rc;
	}
	/* DATA flow DS routing = 0x2 (BIT1). Oracle-confirmed 2026-06-15: a live
	 * working stock RTL9602C has DS_TRAFFIC_CFG = 0x2 and 0x3 on its data flows,
	 * never 0. Our prior 0 WAS the WAN-DHCP DS bug: the GTC de-encapsulated the
	 * data and then DROPPED it, so switch port RX and gpon0 RX stayed 0 and no
	 * DHCP OFFER was delivered. BIT1 routes de-encapped unicast into the PON-IP
	 * NIC->GMAC-RX drain, where the eth demux hands it to gpon0. */
	gpon_wr(GPON_GTC_DS_TRAFFIC_CFG + GPON_DATA_FLOW * DS_TRAFFIC_CFG_STRIDE, 0x2);

	/* US GEM-port map: the data flow -> the OLT's gem, the gem-id stamped on US
	 * data frames. Same shared gpon_gtc_us_gem_stamp() as the OMCC stamp, and the
	 * 12-bit wire mask is gpon_gem_us_port_id(), the one spelling. */
	luna_data.stamp_dirty = reg_has(luna_gpon_chip.gtc.gem_us_port_map);
	if (!gpon_gtc_us_gem_stamp(&gpon_io, &luna_gpon_chip.gtc, GPON_DATA_FLOW,
				   gpon_gem_us_port_id(gpon_data_gem_port))) {
		pr_err("luna-gpon: gpon_chip table declares no US port map -- data gem %u not stamped\n",
		       gpon_data_gem_port);
		return -ENODEV;
	}

	/* The accepted binding selects either its dedicated data queue or the
	 * existing management queue when both identities share the same Alloc-ID.
	 * Never install a second T-CONT for the management allocation. */
	luna_data.sid_dirty = true;
	pi_packed_set(PI_PON_SID2QID, GPON_DATA_FLOW, 7, luna_data_qid & 0x7f);
	pi_packed_set(PI_PON_SIDVALID, GPON_DATA_FLOW, 1, 1);
	pi_packed_set(PI_PON_SID_Q_MAP_DS, GPON_DATA_FLOW, 2, 0);

	/* Multicast/broadcast GEM -> its own flow, BRIDGED and DS-only: broadcast DS
	 * such as the DHCP OFFER may ride this GEM rather than the unicast data GEM,
	 * and without a CAM entry the GTC drops it.
	 * This copy is why the four CAM writes now share ONE helper: it had NO
	 * completion check, so a timed-out mcast CAM fell through to
	 * gpon_data_installed = true while the GTC dropped the broadcast DS and the
	 * install reported success. */
	rc = luna_port_cam_write(GPON_MCAST_FLOW, GPON_MCAST_GEM);
	if (rc < 0) {
		pr_err("luna-gpon: MCAST GEM DS install timeout\n");
		return rc;
	}
	/* MCAST flow DS routing = 0x3 (BIT1|BIT0): BIT0 is the broadcast/flood
	 * variant, so DHCP-broadcast and ARP DS also drain to the NIC. */
	gpon_wr(GPON_GTC_DS_TRAFFIC_CFG + GPON_MCAST_FLOW * DS_TRAFFIC_CFG_STRIDE, 0x3);

	/* Data-flow SID classify commit -- the flow-1 US half-boot fix.
	 * The US-NIC RX engine commits its SID-classify table ONLY on a GMII_RX_EN
	 * rising edge. The OMCC SID gets two guaranteed edges, so OMCI is reliable;
	 * the data classify written just above got ZERO, because the intended relatch
	 * is a no-op by default and the install_tcont relatch fires for the OMCC
	 * T-CONT BEFORE this gem exists. So the data flow only latched when the OLT
	 * happened to re-trigger a T-CONT edge afterwards -- the ~50% boot-dependent
	 * latch, with DISCOVER never egressing while alloc, T-CONT and OMCI are fine.
	 * Pulse the same edge HERE and BLOCK on a readback commit-poll, rewriting the
	 * triple and re-pulsing if either reads back wrong. This runs in the
	 * Configure_Port-ID window the driver already tolerates, not while Online.
	 * The edge re-commits the WHOLE table, so the OMCC entry is preserved. */
	if (relatch_us) {
		int tries;

		for (tries = 0; tries < 4; tries++) {
			/* (re)assert the flow-1 classify triple so a missed edge is re-armed */
			pi_packed_set(PI_PON_SID2QID, GPON_DATA_FLOW, 7,
				      luna_data_qid & 0x7f);
			pi_packed_set(PI_PON_SIDVALID, GPON_DATA_FLOW, 1, 1);

			/* Re-force the US-NIC<->GMAC0 internal-MII link UP before the commit edge,
			 * the symmetric twin of the DS re-force below: the ifup GMAC0 power-cycle
			 * can drop this link, so the classify edge latched only ~50/50. */
			pi_wr(PI_MEDIA_STS_US, 0x106e8400u);
			/* commit edge: GMII_RX_EN OFF -> ON latches the classify table */
			pi_wr(PI_IO_CMD_0_US, 0x90101050u);	/* GMII RX OFF */
			udelay(50);
			pi_wr(PI_IO_CMD_0_US, 0x90101070u);	/* GMII RX ON -> latch flow-1 */

			/* The classify registers are not write-protected, they latch on the edge,
			 * so a correct readback is the definitive "committed" signal. */
			if (pi_packed_get(PI_PON_SIDVALID, GPON_DATA_FLOW, 1) == 1 &&
			    pi_packed_get(PI_PON_SID2QID, GPON_DATA_FLOW, 7) ==
				    (luna_data_qid & 0x7f))
				break;
			udelay(100);
		}
		if (tries == 4) {
			pr_err("luna-gpon: DATA GEM classify readback timeout\n");
			return -ETIMEDOUT;
		}
		pr_info("luna-gpon: data-gem: flow-%u classify committed after %d edge(s)\n",
			GPON_DATA_FLOW, tries + 1);

		/* DS half of the same fix: the DS-NIC drain config and GMII edge were latched
		 * at boot, before this window and before the ifup GMAC0 power-cycle may have
		 * perturbed the DS-NIC<->GMAC0 MII. Re-force the link and re-pulse the DS
		 * edge here so DS delivery to the GMAC RX ring is deterministic every boot. */
		pi_wr(PI_MEDIA_STS_DS, 0x106e8400u);	/* re-force DS-NIC<->GMAC0 internal MII link UP */
		pi_wr(PI_IO_CMD_0_DS, 0x90081050u);	/* DS GMII_RX_EN OFF */
		udelay(50);
		pi_wr(PI_IO_CMD_0_DS, 0x90081070u);	/* ON -> re-latch DS drain config + link */
	}
	/* Keep the pending flow through modeset without publishing success early. */
	rtl9602c_ponmac_modeset_gpon(true);
	if (pi_packed_get(PI_PON_SIDVALID, GPON_DATA_FLOW, 1) != 1 ||
	    pi_packed_get(PI_PON_SID2QID, GPON_DATA_FLOW, 7) != (luna_data_qid & 0x7f))
		return -EIO;
	gpon_data_installed = true;
	gpon_ploam_set_data_installed(&luna_ploam, true);

	pr_info("luna-gpon: DATA GEM installed gem=%u flow=%u qid=%u sid2qid=%u sidvalid=%u\n",
		gpon_data_gem_port, GPON_DATA_FLOW, luna_data_qid,
		pi_packed_get(PI_PON_SID2QID, GPON_DATA_FLOW, 7),
		pi_packed_get(PI_PON_SIDVALID, GPON_DATA_FLOW, 1));
	return 0;
}


/*
 * Bind an OLT-assigned Alloc-ID to a T-CONT in the GTC alloc CAM. Without it the
 * GTC does not associate the OLT's BWmap grants for that Alloc-ID with a T-CONT,
 * so the ONU never transmits upstream on it and US OMCI has no T-CONT to egress
 * on. Same indirect-CAM op as the DS GEM-port CAM.
 */
/* One queue-drain engine, owned by the timer or a process reset that has
 * stopped it. Wait for the previous BUSY before replacing its command/qid. */
static int luna_queue_drain(u8 qid)
{
	int rc;

	rc = gpon_ind_poll(&pi_io, PI_DRN_CMD, 1u, 10000, pi_pause_1us);
	if (rc < 0)
		return rc;
	rc = gpon_ind_go(&pi_io, PI_DRN_CMD,
			 (1u << 2) | ((qid & 0x7f) << 3) | (1u << 1),
			 1u, 10000, pi_pause_1us);
	return rc < 0 ? rc : 0;
}

static int gpon_install_tcont(u8 tcont, u16 alloc)
{
	int rc;

	if (tcont == GPON_OMCC_TCONT) {
		rc = luna_data_retire();
		if (rc)
			return rc;
	}

	/* The alloc CAM's own registers and 5-bit index mask come from luna_gpon_chip;
	 * the transaction is the same shared helper as the three DS GEM-port binds. */
	rc = luna_alloc_cam_write(tcont, alloc);
	if (rc < 0) {
		pr_err("luna-gpon: T-CONT alloc bind timeout\n");
		return rc;
	}

	/* PON-MAC US scheduler activation. The GTC alloc CAM above binds the OLT
	 * Alloc-ID to the T-CONT for BWmap grants, but the PON-IP scheduler never knew
	 * the T-CONT's physical queue was a live member, so the ONU could not transmit
	 * US GEM on it -- the OLT then never saw the upstream T-CONT operate, stayed at
	 * Config State "initial" and never started OMCI. Activate: enable the T-CONT,
	 * put its logical queue 0 in the schedule mask, give the physical queue STRICT
	 * type plus MAX PIR and CIR 0.
	 * PON_GEN_PIR_DROP is cleared for rev-A in rtl9602c_datapath_tables_init(), NOT
	 * here. Offsets are PON-IP driver-relative (phys - 0xF00000). */
	{
		/* The queue comes from the PER-CHIP resolver, not a formula spelled here.
		 * This was `32 * (tcont / 8)` -- the RTL9602C/9607C formula -- on every die
		 * while the CLASSIFIER had already moved to the per-chip value. Half a
		 * repair: on the RTL9603CVD the data SID is classified into queue 64 and
		 * every write below landed on queue 32, which is T-CONT 4's, so the data
		 * queue kept its RESET weight -- and a zero-weight queue is skipped by the
		 * WFQ round, so the T-CONT's grants drain nothing and go out as idle GEM.
		 * The alt bind is a default-off debug knob, not a chip fact, so it keeps
		 * its explicit override.
		 * The PIR/CIR rate arrays hold ONE 18-bit field PER 32-bit word (addr =
		 * base + qid*4), NOT bit-packed: the old (qid*18)/32 indexing wrote qid
		 * 36's word and left the real queue's shaper rate at reset. */
		u8 qid = (tcont == GPON_OMCC_TCONT_ALT) ? swc->omcc_phys_qid
						       : luna_tcont_phys_qid(tcont);
		int drain_rc;

		pi_packed_set(PI_PON_TCONT_EN, tcont, 1, 1);	/* PON_TCONT_EN[tcont] (1b packed) */
		/*
		 * Drain out the physical queue BEFORE binding it into the schedule mask --
		 * the half of the stock ponmac queue-add that was omitted. Without it the
		 * queue stays in a stale drain state and the US scheduler never PULLS it
		 * onto the T-CONT's grants, so the T-CONT fills its grants with idle GEM
		 * and the OMCC GEM never egresses.
		 * Third instance of the contract the two PONIP_DBG_CTRL_US strobes use, so
		 * it goes through the same core owner: write a request word, poll the SAME
		 * register until the block drops its busy bit, bounded. Only the BIT and
		 * the BOUND differ, and gpon_ind_go() takes both as parameters.
		 */
		drain_rc = luna_queue_drain(qid);
		if (drain_rc < 0) {
			pr_err("luna-gpon: qid %u drain-out failed (%d)\n", qid, drain_rc);
			return drain_rc;
		}
		pi_packed_set(PI_PON_SCH_QMAP, tcont, swc->sch_qmap_bits, 0x1);
						/* PON_SCH_QMAP[tcont] = logical-q0; the entry is 32b
						 * wide on the RTL9602C and 8b on the RTL9603CVD, so
						 * `base + tcont * 4` was a per-chip fact too. */
		pi_field(PI_PON_QID_PIR_RATE + qid * PI_QID_RATE_STRIDE, 17, 0, 0x3ffff);	/* PON_QID_PIR_RATE[qid] = MAX (1 word/qid) */
		pi_field(PI_PON_QID_CIR_RATE + qid * PI_QID_RATE_STRIDE, 17, 0, 0);		/* PON_QID_CIR_RATE[qid] = 0 (live-stock; STRICT uses PIR only) */
		pi_packed_set(PI_PON_WFQ_TYPE, qid, 1, 0);	/* PON_WFQ_TYPE[qid] = STRICT (1b packed) */
		/* PON_WFQ_WEIGHT[qid] = 1 (10 bits/entry, 3 entries per word): stock writes
		 * weight 1 even for a STRICT queue, because a zero-weight queue is skipped
		 * by the WFQ round. */
		pi_packed_set(PI_PON_WFQ_WEIGHT, qid, 10, 1);
		/* Re-issue the OMCC classifier's SID-valid bit NOW, after the queue is fully
		 * armed (drained, T-CONT-enabled, in the schedule mask, rated): stock's
		 * queue_add tail re-issues SIDVALID for every SID mapping into the armed
		 * queue, and we had only the early classify-triple write. A PLAIN set(1),
		 * never clear-then-set: the momentary SIDVALID=0 glitched an in-flight US
		 * burst and the OLT latched "Laser out". This is an ORDER fix.
		 * SCOPE: the OMCC qid only. With data_tcont=1 the data flow would get its
		 * own physical qid and need the same post-arm re-issue. */
		if (sidvalid_last && qid == GPON_OMCC_PHYS_QID) {
			/* arm_ctx witness: sample the queue's arm state at the instant SIDVALID is
			 * re-issued. All three are set in the correct order; a missing one means a
			 * regression moved SIDVALID before the arm again.
			 * It ACCUSED ITSELF for two days while the arm was fine: the two
			 * PON_TCONT_EN accesses here were the BARE literal 0x023e4, which on the
			 * RTL9603CVD is PON_QID_CIR_RATE[127] -- the OMCC queue's own rate word,
			 * zeroed three lines above -- so `en` could never read 1 and the G24W
			 * printed "arm INCOMPLETE" on 31 of 31 attempts. A 1:1 correlation with a
			 * failure is not a cause when the witness is a constant. Guarded by
			 * bare_offset_chip_audit.py. */
			u32 en   = pi_packed_get(PI_PON_TCONT_EN, tcont, 1);
			u32 qmap = pi_packed_get(PI_PON_SCH_QMAP, tcont, swc->sch_qmap_bits) & 0x3u;
			u32 pir  = pi_rd(PI_PON_QID_PIR_RATE + qid * PI_QID_RATE_STRIDE) & 0x3ffffu;

			if (en && qmap && pir)
				pr_info("luna-gpon: SIDVALID[%u] arm_ctx OK (tcont_en=1 sch_qmap=%u pir=0x%x) -> re-issuing on armed queue\n",
					GPON_OMCC_FLOW, qmap, pir);
			else
				pr_warn("luna-gpon: SIDVALID[%u] re-issued with arm INCOMPLETE (tcont_en=%u sch_qmap=%u pir=0x%x) -> binding committed against un-armed queue\n",
					GPON_OMCC_FLOW, en, qmap, pir);

			/* Plain re-issue = 1, matching stock's queue_add tail: a same-value RMW is
			 * still a real strobe, because the HW re-commits on the write ORDER and not
			 * on a 0->1 edge. NO clear: a momentary 0 can drop an in-flight US burst. */
			pi_packed_set(PI_PON_SIDVALID, GPON_OMCC_FLOW, 1, 1);	/* re-issue = 1 (RMW strobe) */
		}
		/* PON_SCH_CTRL is NOT written here: PIR_DROP is cleared for rev-A in
		 * rtl9602c_datapath_tables_init(), and this board's own stock k0 binary
		 * confirms rev-A clears it "due to the tcont 16". */
	}

	/* The GMII re-latch used to be here, at Assign_ONU-ID time, but stock does the
	 * ponmac mode-set at driver init -- BEFORE any PLOAM -- while the GEM port map
	 * is written LATER at Configure_Port-ID. So the re-latch fired before the port
	 * map existed and the GEM-US engine latched without it. It now lives in
	 * gpon_install_omcc(), matching the vendor order. */

	pr_info("luna-gpon: T-CONT %u <- alloc 0x%x bound (compl %d)\n",
		tcont, alloc, rc);
	return 0;
}

/* A failure leaves an ownership obligation, including an uncompleted CAM
 * request. Never erase that ledger merely because installed is false. */
static bool luna_data_dirty(void)
{
	return luna_data.alloc_dirty || luna_data.queue_dirty || luna_data.gem_dirty ||
	       luna_data.sid_dirty || luna_data.stamp_dirty;
}

static void luna_data_suspend(void)
{
	WRITE_ONCE(luna_data_admitted, false);
	luna_data.armed.installed = false;
	gpon_data_installed = false;
	gpon_ploam_set_data_installed(&luna_ploam, false);
}

static int luna_data_retire(void)
{
	u32 qmap;
	int rc;

	luna_data_suspend();
	/* A producer that already passed the WAN gate finishes publishing before
	 * hardware withdrawal. Already-owned descriptors are not claimed drained
	 * by this CPU fence; the selected dedicated PON queue is drained below. */
	if (luna_omci.tx_fence)
		luna_omci.tx_fence(luna_omci.cookie);
	if (luna_data.sid_dirty) {
		pi_packed_set(PI_PON_SIDVALID, GPON_DATA_FLOW, 1, 0);
		if (pi_packed_get(PI_PON_SIDVALID, GPON_DATA_FLOW, 1))
			return -EIO;
		luna_data.sid_dirty = false;
	}
	if (luna_data.stamp_dirty) {
		if (!gpon_gtc_us_gem_stamp(&gpon_io, &luna_gpon_chip.gtc, GPON_DATA_FLOW, 0))
			return -ENODEV;
		if (gpon_rd(luna_gpon_chip.gtc.gem_us_port_map +
			    GPON_DATA_FLOW * luna_gpon_chip.gtc.gem_us_port_stride) & 0xfff)
			return -EIO;
		luna_data.stamp_dirty = false;
	}
	if (luna_data.queue_dirty) {
		/* Only a dedicated data T-CONT is owned here. Never drain the OMCC. */
		if (luna_data.armed.rides_omcc || luna_data_qid == GPON_OMCC_PHYS_QID)
			return -EINVAL;
		qmap = pi_packed_get(PI_PON_SCH_QMAP, GPON_DATA_TCONT, swc->sch_qmap_bits);
		/* This owner has one queue. An unexpected member is not ours to
		 * strand by cleaning the T-CONT allocation after removing our bit. */
		if (qmap & ~BIT(luna_data_qid % swc->tcont_queue_max))
			return -EBUSY;
		rc = luna_queue_drain(luna_data_qid);
		if (rc)
			return rc;
		qmap &= ~BIT(luna_data_qid % swc->tcont_queue_max);
		pi_packed_set(PI_PON_SCH_QMAP, GPON_DATA_TCONT, swc->sch_qmap_bits, qmap);
		if (pi_packed_get(PI_PON_SCH_QMAP, GPON_DATA_TCONT, swc->sch_qmap_bits) != qmap)
			return -EIO;
		if (!qmap) {
			pi_packed_set(PI_PON_TCONT_EN, GPON_DATA_TCONT, 1, 0);
			if (pi_packed_get(PI_PON_TCONT_EN, GPON_DATA_TCONT, 1))
				return -EIO;
		}
		luna_data.queue_dirty = false;
	}
	if (luna_data.gem_dirty) {
		rc = luna_cam_xact(false, GPON_GTC_CAM_OP_CLEAN, GPON_DATA_FLOW, NULL, NULL);
		if (rc)
			return rc;
		/* Own stock clears these five traffic bits after CAM completion. */
		gpon_field(GPON_GTC_DS_TRAFFIC_CFG + GPON_DATA_FLOW * DS_TRAFFIC_CFG_STRIDE, 4, 0, 0);
		if (gpon_rd(GPON_GTC_DS_TRAFFIC_CFG + GPON_DATA_FLOW * DS_TRAFFIC_CFG_STRIDE) & 0x1f)
			return -EIO;
		luna_data.gem_dirty = false;
	}
	if (luna_data.alloc_dirty) {
		if (luna_data.armed.rides_omcc)
			return -EINVAL;
		rc = luna_cam_xact(true, GPON_GTC_CAM_OP_CLEAN, GPON_DATA_TCONT, NULL, NULL);
		if (rc)
			return rc;
		luna_data.alloc_dirty = false;
	}
	memset(&luna_data.armed, 0, sizeof(luna_data.armed));
	gpon_data_tcont_installed = false;
	luna_ploam.data_tcont_installed = false;
	return 0;
}

#ifdef CONFIG_GPON_GEM_DIAG
/*
 * The FAMILY's half of CONFIG_GPON_GEM_DIAG.
 *
 * ★★ IT PRINTS THREE INPUTS THE CORE CANNOT SEE, and that is the whole reason
 *    it exists rather than the core line alone.  `want.alloc_known` below is a
 *    CONJUNCTION -- the OMCI binding names a GEM, ME 262 wrote an Alloc-ID, AND
 *    that Alloc-ID is either the OMCC's own or one PLOAM has AUTHORISED -- so
 *    the core receives a single bool and can only ever answer `alloc-unknown`.
 *    Three different faults, one word, and the PLOAM-authorisation half is the
 *    one no other witness in this driver reports at all.
 *
 * ★ ONLY ON CHANGE.  The reconcile runs on every poll, so a line per call is a
 *   flood and a rate limit would DROP the transition that matters.  Printing
 *   when the rendered text differs from the last one keeps every transition and
 *   costs one line per real event -- suppression must never hide.
 */
static void luna_gem_diag_report(const struct gpon_data_armed *armed,
				 const struct gpon_data_want *want,
				 const struct omci_data_binding *binding)
{
	/* Single caller, and luna_data_reconcile is not re-entered: the FSM poll
	 * that calls it is the one context this driver runs it from. */
	static char last[200];
	char line[200];
	int n;

	n = scnprintf(line, sizeof(line),
		      "ploam-auth=%u omci-gem-present=%u omci-alloc-written=%u | ",
		      gpon_ploam_data_alloc_known(&luna_ploam, binding->alloc_id) ? 1u : 0u,
		      binding->gem_present ? 1u : 0u,
		      binding->alloc_known ? 1u : 0u);
	n += gpon_gem_diag_line(armed, want, line + n, sizeof(line) - n);
	if (n <= 0 || !strcmp(line, last))
		return;
	strscpy(last, line, sizeof(last));
	pr_info("luna-gpon: %s\n", line);
}
#endif /* CONFIG_GPON_GEM_DIAG */

static void luna_data_reconcile(void)
{
	struct omci_data_binding binding;
	struct gpon_data_armed visible = luna_data.armed;
	struct gpon_data_want want = { 0 };
	enum gpon_data_plan plan;
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&luna_omci_lock, flags);
	if (luna_omci.onu) {
		omci_data_binding_snapshot(luna_omci.onu, gpon_omcc_gem_port,
					   GPON_MCAST_GEM, &luna_omci.binding);
	}
	binding = luna_omci.binding;
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	want.omcc_alloc = gpon_omcc_alloc ? gpon_omcc_alloc :
		(core_fsm ? luna_ploam.onu_id : gpon_fsm_onu_id);
	want.omcc_up = READ_ONCE(luna_activation_ready) &&
		(core_fsm ? luna_ploam.state == GPON_O5_OPERATION && luna_ploam.omcc_installed :
		 gpon_fsm_state == GPON_O5_OPERATION && gpon_omcc_installed);
	want.gem = binding.gem_present ? binding.gem_port : 0;
	want.alloc = binding.alloc_id;
	want.alloc_known = binding.gem_present && binding.alloc_known &&
		(want.alloc == want.omcc_alloc ||
		 gpon_ploam_data_alloc_known(&luna_ploam, want.alloc));
	/* Presence here includes submitted-but-uncertain hardware ownership, not
	 * a false claim that the original CAM installation completed. */
	if (luna_data_dirty())
		visible.alloc_bound = true;
	plan = gpon_data_plan_decide(&visible, &want);
#ifdef CONFIG_GPON_GEM_DIAG
	luna_gem_diag_report(&visible, &want, &binding);
#endif
	if (plan == GPON_DATA_TEARDOWN || plan == GPON_DATA_REPLACE ||
	    (luna_data_dirty() && !luna_data.armed.installed)) {
		rc = luna_data_retire();
		goto out; /* Replacement gets a fresh poll; at most one heavy attempt. */
	}
	if (plan == GPON_DATA_KEEP) {
		WRITE_ONCE(luna_data_admitted, want.omcc_up);
		goto out;
	}
	if (plan != GPON_DATA_INSTALL || !data_gem_en) {
		WRITE_ONCE(luna_data_admitted, false);
		goto out;
	}
	luna_data.armed.alloc = want.alloc;
	luna_data.armed.gem = want.gem;
	luna_data.armed.rides_omcc = want.alloc == want.omcc_alloc;
	luna_data_qid = luna_data.armed.rides_omcc ? GPON_OMCC_PHYS_QID :
		luna_tcont_phys_qid(GPON_DATA_TCONT);
	if (!luna_data.armed.rides_omcc) {
		/* Set obligations BEFORE the first write: a timeout can mean submitted. */
		luna_data.alloc_dirty = luna_data.queue_dirty = true;
		rc = gpon_install_tcont(GPON_DATA_TCONT, want.alloc);
		if (rc)
			goto out;
	}
	luna_data.armed.alloc_bound = true;
	gpon_data_gem_port = want.gem;
	gpon_data_gem_solicited = true;
	rc = gpon_install_data_gem();
	if (!rc) {
		luna_data.armed.installed = true;
		gpon_data_alloc = want.alloc;
		gpon_data_tcont_installed = !luna_data.armed.rides_omcc;
		luna_ploam.data_alloc = want.alloc;
		luna_ploam.data_tcont_installed = gpon_data_tcont_installed;
		WRITE_ONCE(luna_data_admitted, true);
	}
out:
	if (rc && rc != luna_data.error)
		pr_warn("luna-gpon: data reconciliation failed rc=%d; ownership retained\n", rc);
	luna_data.error = rc;
}

static void luna_omci_service(void)
{
	luna_omci_poll();
	luna_data_reconcile();
}

static void luna_omci_seq_show(struct seq_file *s)
{
	unsigned long flags;
	u32 queued, count, overflow, closed, accepted, resets, tx_errors;

	spin_lock_irqsave(&luna_omci_lock, flags);
	queued = luna_omci.queued;
	count = luna_omci.count;
	overflow = luna_omci.overflow;
	closed = luna_omci.closed;
	accepted = luna_omci.accepted;
	resets = luna_omci.resets;
	tx_errors = READ_ONCE(luna_omci.tx_errors);
	spin_unlock_irqrestore(&luna_omci_lock, flags);
	seq_printf(s, "omci_queue: queued=%u pending=%u overflow=%u closed=%u accepted=%u resets=%u tx_errors=%u\n",
		   queued, count, overflow, closed, accepted, resets, tx_errors);
	seq_printf(s, "data_owner: admitted=%u installed=%u alloc_bound=%u alloc=%u gem=%u qid=%u error=%d\n",
		   luna_gpon_data_ready(), READ_ONCE(luna_data.armed.installed),
		   READ_ONCE(luna_data.armed.alloc_bound), READ_ONCE(luna_data.armed.alloc),
		   READ_ONCE(luna_data.armed.gem), READ_ONCE(luna_data_qid), READ_ONCE(luna_data.error));
}



/*
 * Several upstream-config registers (US_CFG, US_LASER, MIN_DELAY) sit behind a
 * write-protect gate: a write only takes effect while the protect register holds
 * the unlock magic. Arm it, write, relock — this bracket wraps each protected US
 * write. (BOH_DATA/EQD are not gated and are written directly.)
 */
static void gpon_wr_us_protected(u32 off, u32 val)
{
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_UNLOCK);
	gpon_wr(off, val);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_LOCK);
}

/*
 * Program the upstream burst-mode overhead (PLOu preamble + delimiter) the OLT
 * asks for in its Upstream_Overhead PLOAM. The OLT's burst receiver locks its
 * CDR on the preamble run and frames on the 3-byte delimiter; wrong overhead
 * means it cannot decode our SN burst and never ranges us.
 *   oh = [0xAA x rep][type3_ptn x (9-rep)][delim0,delim1,delim2]
 * Only GPON_BOH_LEN(12) bytes are stored: the HW emits a BOH_LENGTH-byte burst
 * by repeating oh[REPEAT] out to LENGTH-3, then the 3 stored delimiter bytes.
 *   LENGTH = rep + t3{pre,ranged} + 3
 * with t3 from the OLT's Extended_Burst_Length (0x14), falling back to the
 * 96-bit default. A too-short pre-ranged preamble is detectable by the OLT but
 * not lockable; an over-long ranged preamble wastes the operation burst.
 * These registers are not behind the US write-protect gate.
 */
static void gpon_apply_boh(bool ranged)
{
	u8 oh[GPON_BOH_LEN];
	u8 guard = gpon_boh_guard, t3 = ranged ? gpon_boh_t3ranged : gpon_boh_t3pre;
	u8 rep, i, boh_len, size;
	unsigned int want;

	/* Exact port of the stock burst-overhead build. The old code ALWAYS stored the
	 * delimiter at oh[9..11] and wrote 12 bytes, but the RANGED burst is usually
	 * SHORTER than 12 -- with BOH_LENGTH<12 the HW emits only BOH_LENGTH bytes, so
	 * the delimiter was CUT OFF and the O5 burst went out unframeable -> LOAi.
	 * Place it at the TRUE end and set BOH_REPEAT to the fill-byte index. */
	if (guard > 32)
		guard = 32;
	rep = guard / 8;			/* boh_repeat = whole guard bytes (fill index) */

	/* WIDEN BEFORE CLAMPING. `rep + t3 + 3` reaches 262 and used to be assigned
	 * straight into the u8 `boh_len`, which TRUNCATES, so the clamp below could
	 * never see a value that had already wrapped. Measured on x86 with the
	 * driver's own types: t3=253 wants 260, wrapped to 4, and the burst went out
	 * 4 bytes long instead of 252 -- the delimiter cut off, which is LOAi. `t3`
	 * comes STRAIGHT OFF THE WIRE with no bound. */
	want = t3 ? (unsigned int)rep + t3 + 3 : GPON_BOH_LEN;
	if (want > GPON_BOH_MAX_LEN) {
		/* The value came off the wire and the hardware field cannot hold
		 * it, so it is a RANGE finding, not support work: report it with
		 * the guard and t3 that produced it, then clamp. */
		u8 dmp[2] = { guard, t3 };

		gpon_unsup_report("boh_len", GPON_UNSUP_RANGE, want,
				  "at-most-252", dmp, sizeof(dmp));
		want = GPON_BOH_MAX_LEN;
	}
	boh_len = (u8)want;		/* total burst-overhead length */
	size = (boh_len > GPON_BOH_LEN) ? GPON_BOH_LEN : boh_len;	/* stored bytes (<=12) */
	if (size < 4)				/* need room for >=1 fill + 3 delimiter */
		size = 4;

	memset(oh, 0xaa, sizeof oh);
	for (i = 0; i < rep && i < (u8)(size - 3); i++)
		oh[i] = 0xaa;			/* guard bytes */
	for (; i < (u8)(size - 3); i++)
		oh[i] = gpon_boh_ptn;		/* Type-3 preamble fill (= bursthead[rep]) */
	oh[size - 3] = gpon_boh_delim[0];	/* delimiter at the TRUE end of the burst */
	oh[size - 2] = gpon_boh_delim[1];
	oh[size - 1] = gpon_boh_delim[2];

	/*
	 * BOH_REPEAT is NOT guard/8: it is the stored-byte index of the LAST preamble
	 * byte before the delimiter, (size - 4), the pointer the burst-builder
	 * replicates when extending the stored <=12 bytes out to BOH_LENGTH. Stock
	 * writes exactly `(size-4)&0xf` and ignores the caller's boh_repeat. With
	 * size=12 that is 8, matching the live-stock O5 dump BOH_CFG=0x083f. Writing
	 * guard/8 mis-positions the delimiter in the synthesized ranged burst, so the
	 * OLT never locks the O5 grant burst -- while the pre-ranged SN burst
	 * tolerates it (size==LENGTH==12, no synthesis), which is why ranging
	 * succeeds while O5 fails.
	 */
	gpon_wr(GPON_GTC_US_BOH_CFG,
		(((size - 4) & 0xf) << 8) | (boh_len & 0xff));	/* REPEAT=size-4, LENGTH=full */
	for (i = 0; i < size; i++)
		gpon_wr(GPON_GTC_US_BOH_DATA + i * 4, oh[i]);

	pr_info("luna-gpon: BOH %s guard=%u rep=%u boh_repeat=%u ptn=0x%02x delim=%02x%02x%02x t3=%u boh_len=%u size=%u oh=%*phN\n",
		ranged ? "ranged" : "prerng", guard, rep, (size - 4) & 0xf, gpon_boh_ptn,
		gpon_boh_delim[0], gpon_boh_delim[1], gpon_boh_delim[2],
		t3, boh_len, size, size, oh);
}

/*
 * Upstream equalization delay. The OLT-visible burst time is value + the local
 * MIN_DELAY1 scaled to bits (x128), split across the 19440x8-bit upstream frame
 * into a multiframe count and an in-frame offset. Pre-ranging (value 0) yields
 * 290*128 = 37120, the correct burst position before the OLT hands us an EqD.
 */
static void gpon_set_eqd(u32 value)
{
	u32 min_delay1 = (gpon_rd(GPON_GTC_US_MIN_DELAY) >> 7) & 0x1ff;
	u32 eqd1  = value + min_delay1 * 128;
	/* The CORE's constant, not a second spelling of it: this arithmetic and
	 * gpon_ploam.c's set_eqd() must divide by the same number or the A/B
	 * switch between them would range the ONU differently. */
	u32 multi = eqd1 / GPON_PLOAM_EQD_FRAME_LEN;
	u32 intra = eqd1 - multi * GPON_PLOAM_EQD_FRAME_LEN;

	gpon_wr(GPON_GTC_US_EQD,
		((multi & GPON_EQD_MF_MASK) << GPON_EQD_MF_SHIFT) |
		(intra & GPON_EQD_INFRAME_MASK));
}

/* HYBRID LAN/VLAN switch. VLAN filtering must be ON during ranging and OMCI
 * config-apply (VLAN-off cold boots fail config-apply), but this switch will NOT
 * forward LAN<->CPU traffic with filtering on, so LAN management needs it OFF.
 * Keep it on through config, clear it once O5 has held vlan_lan_o5_ticks, and
 * re-assert on any drop below O5. */
static unsigned int vlan_lan_o5_ticks = 4000;
module_param(vlan_lan_o5_ticks, uint, 0644);
MODULE_PARM_DESC(vlan_lan_o5_ticks, "poll ticks held at O5 before clearing VLAN_FILTER for LAN access (0=keep filtering on)");
static u32 gpon_o5_entry_tick;
static bool gpon_vlan_lan_open;

/*
 * Re-lock the TX CMU PLL at O3 entry. A fresh power-on under strong downstream
 * light can assert the optical signal-detect before the CMU has settled,
 * latching the TX PLL onto the wrong clock rate (~50% "Laser out"). Toggling the
 * CMU enable 1->0->1 forces a clean re-acquire now the optics are stable, then
 * the SerDes word FIFO pointer is re-synced. Called once per ranging cycle from
 * the FSM (softirq); the CMU finishes re-locking well before the burst is framed.
 */
static void gpon_txpll_relock(void)
{
	if (!serdes_txpll_relock)
		return;
	if (sw_rd(SDS_ANA_COM_REG27) & SDS_CMU_EN) {
		sw_field(SDS_ANA_COM_REG27, 10, 10, 0);		/* CMU enable -> 0      */
		udelay(100);
		sw_field(SDS_ANA_COM_REG27, 10, 10, 1);		/* -> 1 (re-acquire)    */
		udelay(100);
	}
	sw_field(WSDS_DIG_1D, 14, 14, 0);			/* FIFO r/w ptr re-sync */
	sw_field(WSDS_DIG_1D, 14, 14, 1);
	pr_info("luna-gpon: TX-PLL relock (CMU re-toggle + FIFO re-sync) at O3 entry\n");
}

/* The ACTION is lifted for the core's ops; the POLICY stays: the core decides
 * WHEN to reseat the CDR and when to arm the key switch, while whether this
 * board should skip a reseat after a healthy O5 is this shell's rule. */
static void gpon_cdr_reseat(void)
{
	/* Re-seat US-TX SerDes interface reset-B (softirq-safe, TX-only; the locked
	 * DS RX framer is undisturbed) so the next re-range starts from a fresh
	 * serializer lock instead of the prior marginal one. */
	sw_field(WSDS_DIG_1D, 16, 16, 0);
	udelay(500);
	sw_field(WSDS_DIG_1D, 16, 16, 1);
}

static void gpon_aes_arm_switch(u32 fc)
{
	gpon_aes_switch_time = fc;
	gpon_wr(0x3014, fc);	/* AES_KEY_SWITCH_TIME[29:0] */
}


/*
 * The REGISTER CLUSTER alone, and the split is the core's CONTRACT: gpon_ploam.h
 * says of the o5_rearm_burst op "re-apply the US packed-burst register cluster.
 * The core emits the No_message that follows it." The shell wired that op to
 * gpon_o5_rearm_burst(), which does BOTH, so with core_fsm=1 the No_message went
 * out twice. Splitting lets each caller take its half.
 */
static void gpon_o5_rearm_burst_regs(void)
{
	if (!READ_ONCE(luna_activation_ready))
		return;
	/* Re-apply the O5 packed-burst gate cluster on EVERY O5 entry, not just at
	 * __init: a re-range performs a GMAC/SDS reset that can clear these US-side
	 * registers ("isolated tolerates, packed exposes"). US-side only. */
	if (!o5_rearm_burst_gate)
		return;
	gpon_wr_us_protected(0x5188, gtune->us_optic_sd_th);	/* US_OPTIC_SD_TH, per chip */
	gpon_field(0x526c, 0, 0, 1);			/* US_PWR_SAV_MODE */
	if (gtune->us_pwr_sav_dg_tx_opt)
		gpon_field(0x526c, 16, 16, 1);		/* DG_TX_OPT, per chip */
	gpon_wr(GPON_GEM_US_PWR_SAV_CFG, (0x10u << 16) | 0x100u);	/* GEM_US_PWR_SAV_CFG */
	gpon_wr(GPON_GEM_US_EOB_MERGE, 0x00000028u);			/* GEM_US_EOB_MERGE */
}

/* The cluster PLUS the auto-No_message keepalive re-arm: what THIS driver's own
 * FSM has always done on an O5 entry, unchanged.  The core FSM must NOT reach
 * this one -- it sends its own No_message. */
static void gpon_o5_rearm_burst(void)
{
	if (!READ_ONCE(luna_activation_ready))
		return;
	u8 nomsg[12];

	gpon_o5_rearm_burst_regs();
	if (!o5_rearm_burst_gate)
		return;
	memset(nomsg, 0xaa, sizeof(nomsg));
	nomsg[0] = 0xff;	/* ONU-ID (HW overrides via ONUID_OVRD) */
	nomsg[1] = 0x04;	/* GPON_PLOAM_US_NOMESSAGE */
	gpon_send_cpu_ploam(PLM_US_QUEUE_NOMSG, nomsg);
}

static void gpon_below_o5(void)
{
	luna_data_suspend();
	gpon_rerange_start_j = jiffies ? jiffies : 1;	/* start the outage timer */
	gpon_o5_entry_tick = 0;
	if (gpon_vlan_lan_open && !lan_keep_open) {
		sw_field(SW_VLAN_CTRL, 0, 0, 1);	/* re-assert VLAN_FILTER for re-config */
		gpon_vlan_lan_open = false;
		pr_info("luna-gpon: re-range -> VLAN_FILTER re-armed (config phase)\n");
	}
	/* lan_keep_open (default): leave VLAN_FILTER cleared so LAN management
	 * survives the WAN-down/re-range; the OLT re-config on resume tolerates it. */
}

static void gpon_fsm_set_state(u8 st)
{
	u8 prev = gpon_fsm_state;

	/* gpon_hold: keep the FSM parked at O1 — refuse every advance past O1 so the
	 * GPON never ranges/deactivates and the shared switch datapath stops churning,
	 * leaving br-lan + the WiFi AP stable for LAN+WiFi access. (GPON/WAN off.) */
	if ((!READ_ONCE(luna_activation_ready) || gpon_hold) && st > 1)
		return;
	if (gpon_fsm_state != st)
		pr_info("luna-gpon: ONU state O%u -> O%u\n", gpon_fsm_state, st);
	if (prev != st)
		gpon_los_run = 0;	/* fresh LOS debounce window on every transition */
	gpon_fsm_state = st;

	/* Hybrid LAN/VLAN bookkeeping: mark O5 entry; on any drop below O5, re-arm VLAN
	 * filtering for the next config and reset the LAN-open timer. */
	if (st == 5 && prev != 5) {
		/* Re-range completion diagnostic: if we had dropped below O5 (fiber LOS,
		 * deact, watchdog), this O5 re-entry closes the outage. Count it + record the
		 * duration + log ONE flap-damped summary line (the fiber-pull "did it recover"
		 * witness the operator asked for). The data GEM re-installs a moment later via
		 * the FSM poll (gpon_data_gem_solicited kept across LOS) -> its own log line. */
		if (gpon_rerange_start_j) {
			gpon_rerange_cnt++;
			gpon_last_outage_ms = jiffies_to_msecs(jiffies - gpon_rerange_start_j);
			gpon_rerange_start_j = 0;
			if (!gpon_rerange_last_log_j ||
			    time_after(jiffies, gpon_rerange_last_log_j + msecs_to_jiffies(2000))) {
				pr_info("luna-gpon: re-range #%u -> O5 (outage ~%u ms); data-GEM re-install pending\n",
					gpon_rerange_cnt, gpon_last_outage_ms);
				gpon_rerange_last_log_j = jiffies;
			}
		}
		gpon_o5_entry_tick = gpon_fsm_ticks ? gpon_fsm_ticks : 1;
		gpon_avc_sent = 0;	/* re-report oper-up to the OLT each online */
		/* Re-apply the O5 packed-burst gate cluster + re-arm the HW auto-
		 * No_message keepalive on EVERY O5 entry, not just at __init. A
		 * re-range performs a GMAC/SDS reset that can clear these US-side regs,
		 * so a re-ranged O5 must not run on reset defaults ("isolated
		 * tolerates, packed exposes"). Same values as init (4791-4794, 4730-
		 * 4737); US-side only, harmless to DS/ranging.
		 *
		 * ★★★ NOT WHEN THE CORE IS DRIVING -- THIS IS ITS OP, AND IT HAS
		 * ALREADY RUN.  gpon_ploam.c's set_state() calls ops->o5_rearm_burst
		 * (+ its own No_message) and ops->on_below_o5 BEFORE it calls
		 * ops->set_hw_state, whose contract is only "reflect the state into
		 * hardware and the PON LED".  This driver maps that op to the WHOLE
		 * setter, so with core_fsm=1 both actions ran a SECOND time from in
		 * here: ONE O5 entry emitted the No_message three times and wrote the
		 * packed-burst cluster twice, and every drop below O5 doubled
		 * gpon_below_o5().  Recorded as W1 by ploam_fsm_diff, which compares
		 * the two FSMs through RECORDING shells and therefore cannot see the
		 * shipping shim; repaired here 2026-09-10 and pinned by the
		 * LUNA_SETSTATE_GUARDS_CORE arm the extractor now reads out of this
		 * file.
		 *
		 * ⚠ THE SHIPPED BEHAVIOUR IS UNCHANGED; THE SHIPPED BYTES ARE NOT.
		 * core_fsm defaults to 0, so both actions still run exactly as
		 * before -- ploam_fsm_diff drives the native FSM at that default and
		 * its 142 native-vs-core comparisons are unmoved.  What IS new in the
		 * object is a load and test of a module parameter on the O5 edge, in
		 * a path that runs once per activation.  Stating it as "no change"
		 * would be the kind of claim this tree checks with an md5.
		 *
		 * The BOOKKEEPING above stays unguarded on purpose -- gpon_fsm_state,
		 * the re-range counters, gpon_o5_entry_tick and gpon_avc_sent are
		 * this driver's globals, read by /proc, the poll and the ethernet
		 * side, and the core keeps its own copy in its object rather than
		 * writing these.  Only the two op-owned ACTIONS are suppressed. */
		if (!core_fsm)
			gpon_o5_rearm_burst();
	} else if (st < 5 && prev >= 5) {
		if (!core_fsm)	/* ops->on_below_o5 already ran; see above */
			gpon_below_o5();
	}
	/* The HW ONU_STATE field uses the same 1-based encoding as our state numbers:
	 * UNKNOWN=0, O1=1, O2=2, O3=3, O4=4, O5=5. So O3 (Serial-Number, where the
	 * GTC's auto-SN-burst transmitter is gated) = register value 3 = our st.
	 * Write st. */
	gpon_field(GPON_GTC_DS_ONU_ID_STATUS, 3, 0, st);

	gpon_led_pon_set(st);
}

/*
 * Process-context worker for the stock SerDes CDR-reset (invert
 * SDS_ANA_COM_REG12 bit15, hold 10 ms, restore), scheduled from softirq on a
 * Deactivate->O1 re-range so the upstream-TX serializer CDR is re-seated with
 * the correct primitive before the next ranging burst. TX-CDR only, so the RX
 * framer lock is undisturbed, and mdelay is safe here.
 */
static bool full_serdes_reinit;	/* default OFF: A/B 2026-06-15 found re-running the full
				 * gpon_serdes_init on re-range BREAKS the GPON (0/5 boots, link never
				 * reaches O5 — the CMU/SDS reset races the FSM re-acquisition). Keep the
				 * light CDR pulse instead. Do NOT enable. */
module_param(full_serdes_reinit, bool, 0644);
MODULE_PARM_DESC(full_serdes_reinit, "re-range: re-run full gpon_serdes_init (BROKEN, default off) vs light CDR pulse");
static void gpon_cdr_reset_worker(struct work_struct *w)
{
	if (!READ_ONCE(luna_activation_ready))
		return;
	u32 cdr;

	if (!serdes_cdr_reset)
		return;
	if (full_serdes_reinit) {
		gpon_serdes_init();	/* full analog re-init: fresh lock attempt */
		pr_info("luna-gpon: re-range FULL serdes re-init\n");
		return;
	}
	cdr = sw_rd(SDS_ANA_COM_REG12);
	sw_wr(SDS_ANA_COM_REG12, cdr ^ BIT(15));
	mdelay(10);
	sw_wr(SDS_ANA_COM_REG12, cdr);
	/* the computed address, for the reason given at the other call site */
	pr_info("luna-gpon: re-range serdesCdr_reset pulse (COM_REG12 @ 0x%05x bit15), restored=0x%08x\n",
		SDS_ANA_COM_REG12, cdr);
}

/*
 * THIS FSM IS THE DUPLICATE. DO NOT EXTEND IT (2026-08-27).
 * drivers/net/gpon/gpon_ploam.c is the common G.984.3 activation FSM: same
 * O1..O7 states, the same message set, HW-decoupled and fuzzed on x86 with time
 * as an explicit input, and BUILT by every target so it can no longer rot.
 * It is still here because the swap can be proven by the offline suite and by
 * nothing at all on hardware -- the X400AXF does activation IN SILICON and never
 * calls a software PLOAM FSM -- and this project's bar for a change is an
 * end-to-end witness.
 * A fix that belongs to the PROTOCOL goes in the common file and is MIRRORED
 * here, never the other way round.
 */
/*
 * The core PLOAM shell: this driver, expressed as struct gpon_ploam_ops.
 * INSTALLED AND SWITCHABLE behind `core_fsm` (default 0): gpon_ploam_init() is
 * called with these ops and gpon_ploam_ds() dispatches through the core when
 * the parameter is set; with it clear the FSM below runs byte for byte as
 * before. This is the A/B, not a design note.
 *
 * The COMPILER checks that every callback the core demands can be expressed
 * from this driver's existing primitives, with the right types. A shim of stubs
 * would prove nothing, so every op either does the real work or is left NULL
 * with the reason.
 *
 * Where the signatures differ, the core owns the ARITHMETIC and the shell owns
 * the REGISTER: gpon_set_eqd() here computes eqd1 from MIN_DELAY1 and writes,
 * while the core splits that into get_min_delay() + set_eqd(mf, intraframe).
 * The two arithmetics were compared line by line on 2026-08-28 and agree.
 */
static void luna_op_ploam_tx(void *sh, u8 queue, const u8 m[GPON_PLOAM_US_LEN])
{
	(void)sh;			/* single-instance driver: state is file-scope */
	gpon_send_cpu_ploam(queue, m);
}

static void luna_op_boh_write(void *sh, u32 cfg_word, const u8 *oh, u8 size)
{
	u8 i;

	(void)sh;
	/* The write half of gpon_apply_boh(): the core composed cfg_word and the
	 * overhead bytes, so only the registers are left here. */
	gpon_wr(GPON_GTC_US_BOH_CFG, cfg_word);
	for (i = 0; i < size; i++)
		gpon_wr(GPON_GTC_US_BOH_DATA + i * 4, oh[i]);
}

static u32 luna_op_get_min_delay(void *sh)
{
	(void)sh;
	return (gpon_rd(GPON_GTC_US_MIN_DELAY) >> 7) & 0x1ff;	/* MIN_DELAY1 */
}

static void luna_op_set_eqd(void *sh, u32 multiframe, u32 intraframe)
{
	(void)sh;
	gpon_wr(GPON_GTC_US_EQD,
		((multiframe & GPON_EQD_MF_MASK) << GPON_EQD_MF_SHIFT) |
		(intraframe & GPON_EQD_INFRAME_MASK));
}

static void luna_op_us_ploam_flush(void *sh)
{
	(void)sh;
	/* PLM_FLUSH_BUF is edge-triggered, 0 THEN 1, read-modify-write because
	 * CRC_GEN_EN|ONUID_OVRD share the word.
	 * IT RESTS AT 1, NOT 0 (settled 2026-09-08, three tiers): the G24W's stock
	 * reads GPON_GTC_US_PLOAM_CFG = 0x13 with bit 4 SET while Online; the
	 * vendor's own flush for this die writes 0 then 1 and STOPS; and ours,
	 * resting at 0, reported `enq_cleared=1(0us) ... cputx=0` on EVERY CPU
	 * PLOAM -- the HW accepted the enqueue and discarded it in the same instant,
	 * so the Password and every Acknowledge were never transmitted.
	 * A third write back to 0 leaves the transmit buffer permanently flushing. */
	gpon_field(GPON_GTC_US_PLOAM_CFG, 4, 4, 0);
	gpon_field(GPON_GTC_US_PLOAM_CFG, 4, 4, 1);
}

static void luna_op_set_hw_state(void *sh, enum gpon_ostate st)
{
	(void)sh;
	gpon_fsm_set_state((u8)st);
}

static void luna_op_set_hw_onu_id(void *sh, u8 onu_id)
{
	(void)sh;
	/*
	 * BOTH registers. The ONU-ID lives in two places on this GTC --
	 * GPON_GTC_DS_ONU_ID_STATUS[15:8] and GPON_GTC_US_ONU_ID[15:8] -- and every
	 * site in the native FSM writes the pair together. This op wrote only the
	 * downstream one, so with core_fsm=1 the watchdog and SN-reprovision paths
	 * would clear the DS status and leave a STALE upstream ONU-ID in hardware:
	 * the ONU would keep bursting under an identity the OLT had taken back.
	 */
	gpon_field(GPON_GTC_DS_ONU_ID_STATUS, 15, 8, onu_id);
	gpon_field(GPON_GTC_US_ONU_ID, 15, 8, onu_id);
}

static void luna_op_on_below_o5(void *sh)
{
	(void)sh;
	gpon_below_o5();
}

static void luna_op_o5_rearm_burst(void *sh)
{
	(void)sh;
	/* The CLUSTER ONLY: gpon_ploam.h's contract ends with "the core emits the
	 * No_message that follows it", and it does. Wiring this op to
	 * gpon_o5_rearm_burst(), which sends one too, put TWO No_messages on the
	 * wire per O5 entry. The native FSM keeps the pair. */
	gpon_o5_rearm_burst_regs();
}

static void luna_op_install_data_gem(void *sh, u16 gem)
{
	(void)sh;
	(void)gem; /* Reconciliation below owns the accepted model and hardware. */
}

static void luna_op_cdr_reseat(void *sh)
{
	(void)sh;
	/* BOTH halves: the interface reset-B re-strobe does not re-lock the
	 * serializer CDR, the 10 ms invert/hold/restore pulse does, and that pulse
	 * cannot run in softirq. This shim called only gpon_cdr_reseat(), so under
	 * core_fsm=1 every re-range started from the prior marginal lock -- the
	 * flapping the pulse exists to cut. Found by ploam_fsm_diff (D2).
	 * The WHEN stays the core's, and this covers all three of its call sites. */
	gpon_cdr_reseat();
	schedule_work(&gpon_cdr_reset_work);
}

static void luna_op_aes_arm_switch(void *sh, u32 superframe)
{
	(void)sh;
	gpon_aes_arm_switch(superframe);
}

static void luna_op_rng(void *sh, u8 *out, unsigned int len)
{
	(void)sh;
	/* The core wants a randomness SOURCE and this shell already uses the
	 * kernel's; gpon_send_key() reaches the same generator. */
	get_random_bytes(out, len);
}

static void luna_op_omci_report_oper_up(void *sh)
{
	(void)sh;
	/* Already an EXPORT_SYMBOL'd cross-module call this driver makes: the
	 * Ethernet driver owns the OMCI shell that reports the VEIP AVC. */
	rtl9602c_eth_omci_report_oper_up();
}

static void luna_op_analog_relock(void *sh)
{
	(void)sh;
	gpon_txpll_relock();
}

static void luna_op_o3_feed_reset(void *sh)
{
	(void)sh;
	/* The same body the native O3 branch runs, and it must be: this op is reached
	 * from gpon_ploam_ds() on the FSM timer, while gpon_us_feed_rearm() is the
	 * PROCESS-CONTEXT variant that reaches __get_free_pages(GFP_KERNEL). */
	gpon_o3_feed_unpark();
}

static void luna_op_aes_stage_key(void *sh, const u8 key[16])
{
	(void)sh;
	gpon_aes_stage_key(key);
}

static int luna_op_install_omcc(void *sh, u16 gem)
{
	(void)sh;
	return gpon_install_omcc(gem);
}

static int luna_op_install_tcont(void *sh, u8 tcont, u16 alloc)
{
	(void)sh;
	/* Park the unused CAM entries FIRST, so the entry about to be written is the
	 * only one that can match this Alloc-ID; parking afterwards would race a
	 * grant arriving in between. This shim called gpon_install_tcont() alone, so
	 * under core_fsm=1 the park did not happen and a stale entry could still
	 * match. Found by ploam_fsm_diff (D1).
	 * It belongs HERE and not in the core: parking an alloc-CAM is a Luna
	 * hardware workaround, not a G.984.3 decision, and a park op in
	 * struct gpon_ploam_ops would hand Cortina one to stub.
	 * ONLY for the OMCC T-CONT: gpon_alloc_cam_clear_others() keeps ONE entry,
	 * so running it on a data T-CONT install would strand the OMCC's binding. */
	if (alloc_cam_park && tcont == GPON_OMCC_TCONT) {
		int rc = gpon_alloc_cam_clear_others(GPON_OMCC_TCONT);

		if (rc)
			return rc;
	}
	return gpon_install_tcont(tcont, alloc);
}

/*
 * `trace` is the one op left NULL-able: gpon_ploam.h calls it "NEVER
 * load-bearing, NULL is always legal" and the core null-checks it.
 *
 * The core's FSM object is instantiated and drives when `core_fsm` says so:
 * gpon_fsm_handle() hands the downstream PLOAM to gpon_ploam_ds(), and
 * gpon_fsm_poll() drives all five gpon_ploam_poll_* entry points plus
 * gpon_ploam_tick() and gpon_ploam_sn_changed(), each beside the untouched
 * native branch behind the same switch. Both FSMs are compiled into every Luna
 * image. The remaining work is the shape change -- this driver's FSM is global
 * based over ~11 file-scope variables, the core is object-based -- and then the
 * DEFAULT, which is a board measurement and the operator's call.
 *
 * The config is this driver's own knobs, one for one: gpon_ploam_cfg's comments
 * read "Luna 16" and "Luna 8" because the core was carved out of this driver's
 * lineage, so nothing is invented to fill it.
 */
/* luna_ploam is defined near the top: the onu_sn setter needs it. */

/* File scope, NOT a local: gpon_ploam_init() keeps the pointer. */
static struct gpon_ploam_cfg luna_ploam_cfg_live __maybe_unused;

static const struct gpon_ploam_cfg luna_ploam_cfg __maybe_unused = {
	.hold			= false,	/* set from gpon_hold at init */
	.cdr_reseat_on_reactivate = true,	/* from cdr_reseat_on_reactivate */
	.o5_rearm_burst_gate	= true,		/* from o5_rearm_burst_gate */
	.o3_feed_reset		= false,	/* from o3_feed_reset */
	.data_gem_en		= true,		/* from data_gem_en */
	.omcc_alt_bind		= false,	/* from omcc_alt_bind */
	/* The STATIC default only: 0 means "use the LIVE ONU-ID", which is the rule.
	 * The live copy below carries gpon_omcc_alloc, a module_param the native FSM
	 * honours -- nothing copied it here, so with core_fsm=1 the SAME knob was
	 * silently ignored while core_fsm=0 obeyed it (ploam_fsm_diff_test W3). */
	.omcc_alloc_override	= 0,
	.omcc_tcont		= GPON_OMCC_TCONT,	/* 16 on Luna */
	.omcc_tcont_alt		= GPON_OMCC_TCONT_ALT,	/* 1, only when alt_bind */
	.data_tcont		= GPON_DATA_TCONT,	/* 8 on Luna */
};

/*
 * The core's events, spoken out loud. `trace` is the ONE callback the core
 * null-checks, so leaving it NULL is legal -- and it made core_fsm=1 a mode in
 * which the board CANNOT BE DIAGNOSED: measured 2026-08-31, the X111W reached
 * O5 and then printed "ONU state O5 -> O1" and nothing else, no Deactivate, no
 * LOS, no watchdog, while the driver's own path names every one of them.
 *
 * DESIGNATED initializers, not a positional list: the event enum has no
 * terminator, so an entry added to the core would silently shift a positional
 * table and print the WRONG NAME for every event after it. Here a new event
 * leaves a NULL hole and prints as its number -- unknown, never mislabelled.
 *
 * Only the events that DECIDE activation speak unconditionally; the per-message
 * chatter stays behind `trace`, because a per-frame pr_info once filled the ring
 * and destroyed another driver's boot messages.
 */
static const char * const luna_ev_name[] = {
	[GPON_PLOAM_EV_STATE]		= "state",
	[GPON_PLOAM_EV_RERANGE_DONE]	= "rerange_done",
	[GPON_PLOAM_EV_BOH]		= "boh",
	[GPON_PLOAM_EV_DS]		= "ds",
	[GPON_PLOAM_EV_O3_FEED_RESET]	= "o3_feed_reset",
	[GPON_PLOAM_EV_ONU_ID]		= "onu_id",
	[GPON_PLOAM_EV_CAM_READBACK]	= "cam_readback",
	[GPON_PLOAM_EV_RANGING_TIME]	= "ranging_time",
	[GPON_PLOAM_EV_DISABLE_SN]	= "disable_sn",
	[GPON_PLOAM_EV_DEACT]		= "DEACT",
	[GPON_PLOAM_EV_DEACT_KEEP_LOCK]	= "DEACT_keep_lock",
	[GPON_PLOAM_EV_EXT_BURST]	= "ext_burst",
	[GPON_PLOAM_EV_ACK]		= "ack",
	[GPON_PLOAM_EV_ASSIGN_ALLOC]	= "assign_alloc",
	[GPON_PLOAM_EV_DATA_TCONT]	= "data_tcont",
	[GPON_PLOAM_EV_REQ_KEY]		= "req_key",
	[GPON_PLOAM_EV_KEY_SENT]	= "key_sent",
	[GPON_PLOAM_EV_REQ_PW]		= "req_pw",
	[GPON_PLOAM_EV_KEY_SWITCH_ARM]	= "key_switch_arm",
	[GPON_PLOAM_EV_KEY_SWITCH]	= "key_switch",
	[GPON_PLOAM_EV_UNHANDLED]	= "UNHANDLED",
	[GPON_PLOAM_EV_SN_REPROVISIONED] = "sn_reprovisioned",
	[GPON_PLOAM_EV_O5_WATCHDOG]	= "O5_WATCHDOG",
	[GPON_PLOAM_EV_LOS_RERANGE]	= "LOS_RERANGE",
};

/* The events that DECIDE activation, and therefore speak whatever `trace` says.
 * Everything else is per-message chatter. */
static bool luna_ev_is_decisive(enum gpon_ploam_ev ev)
{
	switch (ev) {
	case GPON_PLOAM_EV_STATE:
	case GPON_PLOAM_EV_DEACT:
	case GPON_PLOAM_EV_DEACT_KEEP_LOCK:
	case GPON_PLOAM_EV_DISABLE_SN:
	case GPON_PLOAM_EV_RANGING_TIME:
	case GPON_PLOAM_EV_ONU_ID:
	case GPON_PLOAM_EV_O5_WATCHDOG:
	case GPON_PLOAM_EV_LOS_RERANGE:
	case GPON_PLOAM_EV_SN_REPROVISIONED:
	case GPON_PLOAM_EV_UNHANDLED:
		return true;
	default:
		return false;
	}
}

/*
 * The instrument must be REMOVABLE, because it is a SUSPECT. Measured
 * 2026-08-31: with core_fsm=1 one boot reached O5 on every cycle and the next
 * four never did -- and the good boot predated this trace op. Several lines per
 * activation cycle cost real time on a 115200 console, inside the very path
 * whose timing is under investigation. core_trace=0 silences it completely.
 */
static bool core_trace = true;
module_param(core_trace, bool, 0644);
MODULE_PARM_DESC(core_trace, "1=the core FSM's events are printed (default; needed to diagnose it). 0=silent, for measuring whether the printing itself perturbs activation");

/*
 * FAMILY half of the core's gpon_ploam_diag: read THIS silicon's counters and
 * print the core-formatted line. The offsets are luna_gpon_regs.h's, named
 * identically in the rtl9602c and rtl9603cvd chipdefs, so they are family facts;
 * a chip lacking one clears its GPON_PDIAG_HAS_* bit and the line says n/a.
 * CLEAR-ON-READ, measured 2026-09-06 (ploam_acpt 30 -> 6 -> 0 across three reads
 * 60 ms apart): each value is "since the previous read", and /proc/gpon reads the
 * same words, so a human reading /proc during an activation steals from this
 * line and this line steals from /proc.
 * Compiles out with CONFIG_GPON_PLOAM_DIAG=n.
 *
 * The reader's own lateness is kept by the poll and the DS-PLOAM drain loop.
 */
static unsigned long luna_poll_prev_jiffies;
static u32 luna_poll_gap_ms;	/* start of this poll - start of the previous, ms */
static u8 luna_rx_burst_idx;	/* DS PLOAMs this poll dequeued before the current one */

/*
 * FAMILY half of the core's gpon_bwcap_diag: the GTC's accepted-grant capture,
 * armed the vendor's way and harvested EVERY poll while activating (O1..O4), so
 * the O4 window is covered end to end instead of in one 4 ms slice. At O5 the
 * OLT grants every frame and the 32-entry buffer would overflow every 4 ms, so
 * harvesting stops there and the engine is left to /proc/gpon.
 */
static struct gpon_bwcap_diag luna_bwcap;
static bool luna_bwcap_armed;

/* The longest window the 8-bit CAP_FRAME_NUM allows: 255 frames = 32 ms, longer
 * than the poll period, so the harvest (which re-arms) always comes first and no
 * frame between two polls goes uncaptured. A bounded window self-clears CAP_EN
 * at its end; the unbounded 0, re-armed every poll, reported a constant 12 VALID
 * entries per harvest in O3 and O4 alike while a 20 s userspace capture of the
 * same engine held ONE -- an artefact, not a grant count. */
#define LUNA_BWCAP_FRAMES	0xffu
#define LUNA_BWCAP_RAW_N	6	/* raw entries shown per point */

static u32 luna_bwcap_raw[2 * LUNA_BWCAP_RAW_N];	/* the first VALID entries since the last point */
static unsigned int luna_bwcap_raw_n;

static void luna_bwcap_arm(void)
{
	gpon_wr(GPON_BWMAP_CTRL, 0);
	gpon_wr(GPON_BWMAP_CTRL, GPON_BWMAP_CAP_CLR | LUNA_BWCAP_FRAMES);
	gpon_wr(GPON_BWMAP_CTRL, GPON_BWMAP_CAP_EN | LUNA_BWCAP_FRAMES);
}

/* Empty the capture into the accumulator.  Does not re-arm. */
static void luna_bwcap_harvest(void)
{
	unsigned int i, found = 0;

	if (gpon_rd(GPON_BWMAP_STS) & GPON_BWMAP_CAP_OVERFL)
		luna_bwcap.overfl++;
	for (i = 0; i < GPON_BWMAP_ENTRIES; i++) {
		u32 w0 = gpon_rd(BWMAP_DATA(2 * i));
		u32 w1 = gpon_rd(BWMAP_DATA(2 * i + 1));
		u8 tc = w0 & GPON_BWMAP_ENT_TCONT_MASK;

		if (!(w0 & GPON_BWMAP_ENT_VALID))
			continue;
		found++;
		if (luna_bwcap_raw_n < LUNA_BWCAP_RAW_N) {
			luna_bwcap_raw[2 * luna_bwcap_raw_n] = w0;
			luna_bwcap_raw[2 * luna_bwcap_raw_n + 1] = w1;
			luna_bwcap_raw_n++;
		}
		luna_bwcap.entries++;
		luna_bwcap.tconts |= BIT(tc);
		if (!(w0 & GPON_BWMAP_ENT_PLOAMU))
			continue;
		luna_bwcap.ploamu++;
		if (tc == GPON_OMCC_TCONT) {
			luna_bwcap.omcc_ploamu++;
			luna_bwcap.last_raw0 = w0;
			luna_bwcap.last_raw1 = w1;
		}
	}
	luna_bwcap.harvests++;
	if (found)
		luna_bwcap.nonempty++;
	luna_bwcap.valid = GPON_BWCAP_HAS;
}

/* Called once per FSM poll, before the DS-PLOAM drain. */
static void luna_bwcap_poll(void)
{
	if (!IS_ENABLED(CONFIG_GPON_PLOAM_DIAG))
		return;
	if (gpon_fsm_state >= 5) {
		luna_bwcap_armed = false;	/* re-armed (with CLR) on the next activation */
		return;
	}
	if (luna_bwcap_armed)
		luna_bwcap_harvest();
	luna_bwcap_arm();
	luna_bwcap_armed = true;
}

static void luna_ploam_diag(enum gpon_ploam_diag_point p)
{
	struct gpon_ploam_diag d;
	char line[256];
	u32 cpu, auto_;

	if (!IS_ENABLED(CONFIG_GPON_PLOAM_DIAG))
		return;
	d.valid      = GPON_PDIAG_HAS_ALL;
	d.ploam_acpt = gpon_rd(GPON_GTC_DS_MISC_CNTR_PLOAM_ACPT);
	d.bwm_acpt   = gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_ACPT);
	d.bwm_fail   = gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_FAIL);
	d.bwm_inv    = gpon_rd(GPON_GTC_DS_MISC_CNTR_BWM_INV);
	/* US-GTC misc PM: idx 2 = PLOAM_CPU_TX, idx 3 = PLOAM_AUTO_TX. EACH IS TWO
	 * PACKED 16-BIT COUNTERS (the vendor's PM reader splits them, tier 3):
	 * CPU_TX = [15:0] normal, [31:16] urgent; AUTO_TX = [15:0] No_message,
	 * [31:16] the Serial_Number the GTC fires into a grant by itself. Summing
	 * the raw words printed one auto-fired SN as 65536. */
	cpu   = gpon_us_misc_cnt(2);
	auto_ = gpon_us_misc_cnt(3);
	d.us_ploam_tx = (cpu & 0xffff) + (cpu >> 16) + (auto_ & 0xffff) + (auto_ >> 16);
	d.us_sn_tx    = auto_ >> 16;
	d.us_onu_id  = (gpon_rd(GPON_GTC_US_ONU_ID) >> GPON_GTC_US_ONU_ID_SHIFT) & GPON_ONU_ID_MASK;
	d.ds_onu_id  = (gpon_rd(GPON_GTC_DS_ONU_ID_STATUS) >> GPON_ONU_ID_SHIFT) & GPON_ONU_ID_MASK;
	d.poll_gap_ms = luna_poll_gap_ms;
	d.rx_burst_idx = luna_rx_burst_idx;
	{
		u32 act = gpon_rd(GPON_GTC_DS_MISC_CNTR_ACTIVE);

		d.sn_req  = GPON_GTC_DS_ACTIVE_SN_REQ(act);
		d.rng_req = GPON_GTC_DS_ACTIVE_RNG_REQ(act);
	}
	gpon_ploam_diag_format(line, sizeof(line), p,
			       gpon_fsm_ticks * GPON_FSM_TICK_MS, gpon_fsm_state, &d);
	pr_info("luna-gpon: %s\n", line);
	/* The accepted-grant capture since the previous point: empty the engine into
	 * the accumulator, report, then start the next window. */
	if (luna_bwcap_armed) {
		luna_bwcap_harvest();
		luna_bwcap_arm();
	}
	gpon_bwcap_diag_format(line, sizeof(line), p,
			       gpon_fsm_ticks * GPON_FSM_TICK_MS, gpon_fsm_state,
			       luna_bwcap_armed ? &luna_bwcap : NULL);
	pr_info("luna-gpon: %s\n", line);
	if (luna_bwcap_raw_n) {
		/* The first entries since the last point, raw and family-specific, so a
		 * human can tell twelve real grants from one entry read twelve times. */
		unsigned int i;
		int pos = 0;

		for (i = 0; i < luna_bwcap_raw_n && pos < (int)sizeof(line) - 20; i++)
			pos += scnprintf(line + pos, sizeof(line) - pos, " %08x/%08x",
					 luna_bwcap_raw[2 * i], luna_bwcap_raw[2 * i + 1]);
		pr_info("luna-gpon: bwcap-raw %s first%u=%s\n",
			gpon_ploam_diag_point_name(p), luna_bwcap_raw_n, line);
	}
	memset(&luna_bwcap, 0, sizeof(luna_bwcap));
	luna_bwcap_raw_n = 0;
}

static void luna_op_trace(void *sh, enum gpon_ploam_ev ev, u32 a, u32 b)
{
	const char *nm = NULL;
	enum gpon_ploam_diag_point p = gpon_ploam_diag_point_of(ev);

	(void)sh;
	if (p != GPON_PDIAG_NONE)	/* the core FSM path: the WHEN is the core's event */
		luna_ploam_diag(p);
	if (!core_trace)
		return;
	if (!luna_ev_is_decisive(ev) && !trace)
		return;
	if ((unsigned int)ev < ARRAY_SIZE(luna_ev_name))
		nm = luna_ev_name[ev];
	if (nm)
		pr_info("luna-gpon: core %s a=%u b=%u\n", nm, a, b);
	else
		pr_info("luna-gpon: core ev%u a=%u b=%u (no name in this shell -- the core gained an event)\n",
			(unsigned int)ev, a, b);
}

static const struct gpon_ploam_ops luna_ploam_ops __maybe_unused = {
	.ploam_tx	= luna_op_ploam_tx,
	.boh_write	= luna_op_boh_write,
	.get_min_delay	= luna_op_get_min_delay,
	.set_eqd	= luna_op_set_eqd,
	.us_ploam_flush	= luna_op_us_ploam_flush,
	.set_hw_state	= luna_op_set_hw_state,
	.set_hw_onu_id	= luna_op_set_hw_onu_id,
	.on_below_o5	= luna_op_on_below_o5,
	.o5_rearm_burst	= luna_op_o5_rearm_burst,
	.install_data_gem = luna_op_install_data_gem,
	.cdr_reseat	= luna_op_cdr_reseat,
	.aes_arm_switch	= luna_op_aes_arm_switch,
	.rng		= luna_op_rng,
	.omci_report_oper_up = luna_op_omci_report_oper_up,
	.analog_relock	= luna_op_analog_relock,
	.o3_feed_reset	= luna_op_o3_feed_reset,
	.aes_stage_key	= luna_op_aes_stage_key,
	.install_omcc	= luna_op_install_omcc,
	.install_tcont	= luna_op_install_tcont,
	.data_alloc_changed = luna_data_alloc_changed,
	.trace		= luna_op_trace,
};

static void gpon_fsm_handle(const u8 *m)
{
	u8 onu_id = m[0], type = m[1];
	const u8 *d = &m[2];		/* 10 data octets */

	/* Surface any DS PLOAM that is not the repetitive broadcast acquisition
	 * traffic, so activation progress is visible. */
	gpon_last_ds_type = type;
	if (trace && type != PLM_DS_UPSTREAM_OVERHEAD && type != PLM_DS_EXT_BURST_LENGTH)
		pr_info_ratelimited("luna-gpon: DS PLOAM onu_id=0x%02x type=0x%02x d=%*phN\n",
				    onu_id, type, 8, d);

	/* Say which FSM is dispatching, once. A switch that changes what the code does
	 * and leaves no trace makes every later boot log ambiguous about its own
	 * subject: on 2026-08-31 an offline pairing proved the COMMON core's FSM
	 * correct and the conclusion was carried to a board running THIS FSM, because
	 * nothing said so. Printed at the first DS PLOAM so the line sits next to the
	 * activation it describes. */
	{
		static bool said;

		if (!said) {
			said = true;
			pr_info("luna-gpon: PLOAM dispatch = %s (core_fsm=%d)\n",
				core_fsm ? "COMMON core gpon_ploam.c"
					 : "this driver's own FSM", core_fsm);
		}
	}

	if (!READ_ONCE(luna_activation_ready))
		return;

	if (core_fsm) {
		/* Ticks x 10, NOT the wall clock, and the more accurate clock is the wrong
		 * one here: this FSM counts polls at a 10 ms mod_timer that is a TARGET,
		 * so under load the tick count falls behind wall time -- and every timeout
		 * being replaced already lives on that slipping clock. Handing the core
		 * real milliseconds would make the A/B compare two FSMs AND two clocks.
		 * See ONU-test-case/OWED-ploam-swap-time-unit.md. */
		gpon_ploam_ds(&luna_ploam, m, GPON_PLOAM_DS_LEN,
			      gpon_fsm_ticks * GPON_FSM_TICK_MS);
		return;
	}

	/*
	 * No defined serial, no participation -- once, at the dispatch (operator,
	 * 2026-09-10). The same repair the core got in gpon_ploam_ds(), here too
	 * because this FSM is a second implementation of the same protocol.
	 * It may NOT sit on the Upstream_Overhead case alone: gating that edge stops
	 * us OFFERING a serial but not ANSWERING for one, and PLM_DS_ASSIGN_ONU_ID
	 * matches on gpon_sn_bytes -- so an OLT naming those bytes walked the FSM
	 * O1 -> O4 -> O5 with an identity nobody programmed.
	 * Nothing legitimate is lost: the serial arrives from STORAGE, never from a
	 * downstream PLOAM, and the two broadcasts that reach an unaddressed ONU ask
	 * it to fall silent, which is what it is already doing.
	 */
	if (!gpon_sn_is_set(gpon_sn_bytes)) {
		static bool said;

		if (!said) {
			said = true;
			pr_info("luna-gpon: downstream PLOAM is arriving, but no ONU serial is provisioned -- parked at O1, answering NOTHING (write /sys/module/luna_gpon/parameters/onu_sn)\n");
		}
		return;
	}

	switch (type) {
	case PLM_DS_UPSTREAM_OVERHEAD:
		/* The OLT is acquiring ONUs (broadcast continuously). On the O1/O2 -> O3
		 * edge, program the burst overhead and pre-ranging EqD the OLT dictates
		 * BEFORE the first SN; the SN is re-sent, throttled, from the poll loop.
		 * Payload: d[0]=guard bits, d[3]=type-3 preamble pattern, d[4..6]=delimiter,
		 * d[7] bit5 = pre-EqD present with value d[8:9] (x32x8 bits). */
		if (gpon_fsm_state < 3) {
			u32 pre_eqd = ((d[7] >> 5) & 1) ?
				(((u32)d[8] << 8) | d[9]) * 32 * 8 : 0;

			gpon_boh_guard    = d[0];
			gpon_boh_ptn      = d[3];
			gpon_boh_delim[0] = d[4];
			gpon_boh_delim[1] = d[5];
			gpon_boh_delim[2] = d[6];
			gpon_apply_boh(false);	/* folds in any prior 0x14 t3pre */
			gpon_set_eqd(pre_eqd);
			gpon_fsm_set_state(2);
			gpon_fsm_set_state(3);
			gpon_txpll_relock();	/* re-lock TX CMU PLL now DS optics are stable, before US burst */
			if (o3_feed_reset) {
				/* The relock (SerDes reset) re-parks the GEM-US US-feed run-state. Un-park
				 * it with the FULL WSDS datapath reset-B edge, here at O3 while DS is not
				 * yet locked. Shared with luna_op_o3_feed_reset so the A/B cannot
				 * diverge on the body. */
				gpon_o3_feed_unpark();
			}
			gpon_send_sn();		/* first SN immediately */
		}
		break;
	case PLM_DS_ASSIGN_ONU_ID:
		/* d[0] = assigned ONU-ID, d[1..8] = serial number to match.
		 *
		 * THREE conditions, not one, ported edge-for-edge from the vendor's own
		 * handler (tier 3): it programs the assigned ID only in O3, AND on the
		 * broadcast ONU-ID, AND on a serial match. We guarded the serial alone, and
		 * this OLT sends Assign_ONU-ID THREE times 100 ms apart -- so #2 and #3
		 * arrive when we are ALREADY IN O4 and re-wrote US_ONU_ID, DS_ONU_ID_STATUS
		 * and the T-CONT binding WHILE RANGING WAS IN PROGRESS.
		 * Not claimed as the cause of the O4 wall: the one cycle in ~100 that did
		 * range re-processed them too. */
		if (!gpon_ploam_assign_allowed(gpon_fsm_state, onu_id))
			break;
		if (!memcmp(&d[1], gpon_sn_bytes, 8)) {
			struct gpon_omcc_tcont_plan plan;
			u16 tcont16_alloc;

			gpon_fsm_onu_id = d[0];
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, gpon_fsm_onu_id);
			gpon_field(GPON_GTC_DS_ONU_ID_STATUS, 15, 8, gpon_fsm_onu_id);
			/* Bind the OMCC's T-CONT 16 to its management Alloc-ID. The RULE lives
			 * in the core (gpon_omcc_tcont_decide): the OMCC US alloc IS the live
			 * ONU-ID, the gpon_omcc_alloc module parameter overrides it only for an
			 * A/B, and the alternate-T-CONT double-bind is refused when the plan
			 * lands on the plain ONU-ID. It was written out here AND in the core
			 * with different names -- and this is the copy that BOOTS, which is how
			 * the Configure_Port-ID one-shot defect survived here for weeks after
			 * the other copies were repaired. What stays HERE is the register write
			 * that carries the plan out. */
			gpon_omcc_tcont_decide(gpon_omcc_alloc, gpon_fsm_onu_id,
					     omcc_alt_bind, &plan);
			tcont16_alloc = plan.alloc;
			/* Park the unused CAM entries FIRST, so the entry about to be written is
			 * the only one that can match this Alloc-ID -- parking afterwards would
			 * race a grant arriving in between. A real data T-CONT is bound later by
			 * the Assign_Alloc-ID handler, which re-writes its own entry. */
			if (alloc_cam_park &&
			    gpon_alloc_cam_clear_others(GPON_OMCC_TCONT))
				break;
			if (gpon_install_tcont(GPON_OMCC_TCONT, tcont16_alloc))
				break; /* Keep O3: the next matching Assign retries. */
			{
				int rb = gpon_alloc_cam_read(GPON_OMCC_TCONT);

				/* rb < 0 = the READ never completed. Saying so beats printing stale
				 * RDATA that can MATCH `want` and confirm a bind that never landed. */
				if (rb < 0)
					pr_err("luna-gpon: CAM[16] readback FAILED rc=%d (want 0x%x unverified)\n",
					       rb, tcont16_alloc);
				else
					pr_info("luna-gpon: CAM[16] readback alloc=0x%x hit=%u (want 0x%x)\n",
						rb & 0xfff, !!(rb & BIT(16)), tcont16_alloc);
			}
			/* NON-STOCK double-bind (default OFF): binding the alloc ALSO to T-CONT 1
			 * makes the alloc-CAM resolve a BWmap grant to the EMPTY T-CONT 1, so the
			 * DBRu reports zero occupancy while qid 64 holds the pages -- the OLT
			 * grants once then stops. Stock binds to T-CONT 16 only. */
			if (plan.bind_alt &&
			    gpon_install_tcont(GPON_OMCC_TCONT_ALT, tcont16_alloc))
				break;
			/* Say whether the alloc was TYPED or is the live ONU-ID: the old label
			 * printed "previously-known OMCC alloc", which told a live diagnosis it
			 * had been LEARNED from the wire. gpon_omcc_alloc is a module parameter
			 * and is never written from the wire. */
			pr_info("luna-gpon: OLT assigned ONU-ID %u (T-CONT 16 <- alloc 0x%x, %s)\n",
				gpon_fsm_onu_id, tcont16_alloc,
				gpon_omcc_alloc ? "gpon_omcc_alloc override" :
				"live ONU-ID");
			luna_ploam_diag(GPON_PDIAG_ASSIGN);
			gpon_fsm_set_state(4);
		}
		break;
	case PLM_DS_RANGING_TIME:
		/* Accept only the main-path EqD (d[0] bit0 == 0); the protect path is not
		 * configurable. EqD is d[1..4] big-endian, folded with MIN_DELAY1. */
		if (gpon_ploam_ranging_allowed(gpon_fsm_state, onu_id,
					       gpon_fsm_onu_id, d[0])) {
			u32 eqd = ((u32)d[1] << 24) | ((u32)d[2] << 16) |
				  ((u32)d[3] << 8) | d[4];

			gpon_set_eqd(eqd);
			gpon_apply_boh(true);	/* switch to the ranged operation burst */
			/* Flush any pre-ranged-format US PLOAM still latched in the single shared
			 * CPU TX buffer before the first ranged grant fires, so the OLT's NARROW
			 * ranged window never has to frame a stale pre-ranged burst. Stock does
			 * exactly this at this O4-EqD edge.
			 * DELIBERATELY the raw writes and not a call to luna_op_us_ploam_flush():
			 * ploam_fsm_diff_test compares this FSM's RAW register vocabulary against
			 * the core's OP calls and FOLDS this sequence into the op's meaning, so
			 * if both sides called the op there would be nothing left to compare.
			 * Its FOLD RULE must mirror the luna_op_us_ploam_flush() body exactly. */
			gpon_field(GPON_GTC_US_PLOAM_CFG, 4, 4, 0);	/* PLM_FLUSH_BUF = 0 */
			gpon_field(GPON_GTC_US_PLOAM_CFG, 4, 4, 1);	/* 0->1 edge: flush, and REST here */
			pr_info("luna-gpon: Ranging_Time EqD=0x%x -> O5 (us-ploam flushed)\n", eqd);
			luna_ploam_diag(GPON_PDIAG_RANGING_TIME);	/* still O4: the window that WORKED */
			gpon_fsm_set_state(5);
		}
		break;
	case PLM_DS_DISABLE_SN:
		/* Disable_serial_number (0x06): d[0] is the disable/ENABLE code, d[1..8] the
		 * target SN, sent to ONU-ID 0xff. ONLY a real DISABLE resets us -- 0xFF for
		 * OUR SN, or 0x0F for all. An ENABLE (0x00 for our SN, the OLT re-allowing
		 * our SN) must NOT reset: the old code reset on every 0x06 and fought the
		 * OLT's re-enable into a re-range loop. */
		if (!((d[0] == 0xff && !memcmp(&d[1], gpon_sn_bytes, 8)) || d[0] == 0x0f))
			break;			/* 0x00 ENABLE / not-our-SN -> ignore, keep activating */
		pr_info("luna-gpon: Disable_SN code=0x%02x -> reset O1\n", d[0]);
		fallthrough;
	case PLM_DS_DEACTIVATE_ONU:
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			pr_info("luna-gpon: EVT t=%u DEACT(0x05) onu=%u | dsrx_omcc=%u pirx=%u omcirx=%d | ploam_cpu=%u gem_byte=%u gemus_omcc=%u idle16=%u idle8=%u\n",
				gpon_fsm_ticks, gpon_fsm_onu_id,
				gpon_gem_ds_rx_cnt(GPON_OMCC_FLOW), sw_rd(OMCI_RX_PKT_CNT), gpon_omci_rx_cnt(),
				gpon_us_misc_cnt(2), gpon_us_misc_cnt(4),
				gpon_rd(GEM_US_STAT(GPON_OMCC_FLOW)), gpon_rd(TCONT_IDLE_STAT(16)),
				gpon_rd(TCONT_IDLE_STAT(GPON_DATA_TCONT)));
			luna_ploam_diag(GPON_PDIAG_DEACT);
			gpon_fsm_onu_id = 0xff;
			/* FULL reset to O1, mirroring the SN-reprovision path. Clearing only the
			 * SW onu-id and key left the one-shot OMCC/T-CONT install guards TRUE and
			 * the HW ONU-ID registers stale, so under deactivate churn the 2nd+
			 * re-range SKIPPED the installs and never rebuilt the OMCI datapath. */
			gpon_omcc_installed = false;
			gpon_omcc_installed_gem = 0;
			gpon_tcont_installed = false;
			gpon_data_installed = false;	/* re-install WAN data GEM on re-config */
			gpon_data_gem_solicited = false;	/* re-wait for the OLT's fresh ME268 before re-installing */
			/* The OLT may hand out a DIFFERENT data Alloc-ID on re-admit; releasing
			 * the bind here is what lets it. */
			gpon_data_tcont_installed = false;
			gpon_data_alloc = 0;
			gpon_ploam_data_alloc_reset(&luna_ploam);
			gpon_aes_switch_time = 0xffffffff;	/* re-arm 0x13 on next activation */
			gpon_key_staged = false;
			gpon_field(GPON_GTC_DS_ONU_ID_STATUS, 15, 8, 0xff);
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
			/* Re-seat the serializer ONLY when the prior O5 was SHORT or marginal, a
			 * real US-burst-quality fault. A healthy long-provisioned O5 that the OLT
			 * deactivated is not one, and re-rolling there only manufactures a fresh
			 * re-range the OLT must re-admit. Stock never re-rolls the CDR on a
			 * deactivate. ~500 ticks of held O5 marks a healthy provision. */
			if (cdr_reseat_on_reactivate &&
			    !(gpon_fsm_state == 5 && gpon_o5_entry_tick &&
			      (gpon_fsm_ticks - gpon_o5_entry_tick) > 500)) {
				/* TX-only and softirq-safe: the locked DS RX framer is undisturbed. */
				gpon_cdr_reseat();
				/* The interface reset-B re-strobe does not re-lock the serializer CDR;
				 * the stock COM_REG12 pulse does, and its 10 ms hold needs process
				 * context. */
				schedule_work(&gpon_cdr_reset_work);
			} else if (cdr_reseat_on_reactivate) {
				pr_info("luna-gpon: deact after healthy O5 (%u ticks) -> skip serializer re-roll (match stock)\n",
					gpon_o5_entry_tick ? gpon_fsm_ticks - gpon_o5_entry_tick : 0);
			}
			gpon_fsm_set_state(1);
		}
		break;
	case PLM_DS_EXT_BURST_LENGTH:
		/* Extended_Burst_Length: d[0] = type-3 preamble length for the PRE-RANGED
		 * burst, d[1] = for the ranged one. Honouring d[0] lengthens our SN-burst
		 * preamble so the OLT's burst receiver can lock and range us. Acted on at
		 * O3, while still broadcast-addressed. */
		gpon_boh_t3ranged = d[1];	/* applied at the O5 transition */
		if (gpon_fsm_onu_id == 0xff && gpon_boh_t3pre != d[0]) {
			gpon_boh_t3pre = d[0];
			gpon_apply_boh(false);
			pr_info("luna-gpon: Extended_Burst_Length type3_preranged=%u ranged=%u\n",
				gpon_boh_t3pre, gpon_boh_t3ranged);
		}
		break;
	case PLM_DS_CONFIG_PORT:
		/* Configure_Port-ID (0x0e): the OLT assigns the OMCC GEM port for OMCI
		 * (d[0] bit0 = enable, gem = (d[1]<<4)|(d[2]>>4)). Install the OMCC GEM
		 * datapath THEN Acknowledge, so the ONU is RX-ready before the OLT
		 * proceeds. */
		if (onu_id == gpon_fsm_onu_id) {
			u16 gem = ((u16)d[1] << 4) | (d[2] >> 4);

			/* Rebind when the OLT MOVES the OMCC to a different GEM. This used to be
			 * one-shot, so a mid-session move was silently ignored and every later DS
			 * OMCI landed on the old port -- OMCI dead with the FSM still reporting
			 * O5. This native FSM is the copy that SHIPS, so it carried the defect
			 * into the product after the other two copies were repaired. */
			enum gpon_omcc_action act =
				gpon_omcc_decide(d[0] & 0x1, gem,
						 gpon_omcc_installed,
						 gpon_omcc_installed_gem);

			if (act == GPON_OMCC_INSTALL ||
			    act == GPON_OMCC_REBIND) {
				if (!gpon_install_omcc(gem)) {
					gpon_omcc_installed = true;
					gpon_omcc_installed_gem = gem;
				}
			}
			gpon_send_ack(m);
			/* The WAN data-GEM install is driven from the FSM poll, gated on the OLT's
			 * ME268 GEM-CTP Create: installing it here, before the OLT created its own
			 * gem, made the OLT unable to reconcile ours on a 2nd+ admit and
			 * churn-lock. */
			if (trace)
				pr_info_ratelimited("luna-gpon: ACK type=0x%02x d=%*phN\n",
						    type, 8, d);
		}
		break;
	case PLM_DS_ASSIGN_ALLOC_ID:
		/* Assign_Alloc-ID (0x0a): bind the OLT's DATA Alloc-ID to the DATA T-CONT
		 * (8), NOT the OMCC's T-CONT 16, which belongs to the management Alloc-ID
		 * set at Assign_ONU-ID. d[2]: 0x01 = allocate, 0xff = deallocate. */
		if (onu_id == gpon_fsm_onu_id) {
			u16 alloc = ((u16)d[0] << 4) | (d[1] >> 4);
			pr_info("luna-gpon: ASSIGN_ALLOC alloc=0x%x op=0x%x tcont_done=%d\n", alloc, d[2], gpon_tcont_installed);

			if (alloc != gpon_fsm_onu_id && gpon_data_alloc_valid(alloc) &&
			    (d[2] == 0x01 || d[2] == 0xff)) {
				luna_data_alloc_changed(NULL, alloc, d[2] == 0x01);
				luna_data_tcont_from_assign(alloc, d[2] == 0x01);
			}
			gpon_send_ack(m);
			if (trace)
				pr_info_ratelimited("luna-gpon: ACK type=0x%02x d=%*phN\n",
						    type, 8, d);
		}
		break;
	case PLM_DS_REQUEST_KEY:
		/* OLT requests a downstream AES key; reply with Encryption_Key (US 0x05). */
		if (onu_id == gpon_fsm_onu_id) {
			gpon_send_key();
			pr_info("luna-gpon: EVT t=%u REQ_KEY(0x0d) dsrx_omcc=%u pirx=%u omcirx=%d\n",
				gpon_fsm_ticks, gpon_gem_ds_rx_cnt(GPON_OMCC_FLOW), sw_rd(OMCI_RX_PKT_CNT),
				gpon_omci_rx_cnt());
		}
		break;
	case PLM_DS_REQUEST_PASSWORD:
		/* Request_Password (0x09): without a reply the OLT stalls at O5, spamming
		 * 0x09, and deactivates us with LOAi. The OLT is SN-authenticated so the
		 * value is ignored, but the message is required to advance activation. */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			gpon_send_password();
			pr_info("luna-gpon: EVT t=%u REQ_PW(0x09) -> sent Password dsrx_omcc=%u pirx=%u omcirx=%d\n",
				gpon_fsm_ticks, gpon_gem_ds_rx_cnt(GPON_OMCC_FLOW), sw_rd(OMCI_RX_PKT_CNT),
				gpon_omci_rx_cnt());
		}
		break;
	case PLM_DS_KEY_SWITCH:
		/* Key_Switching_Time (0x13): the OLT gives the superframe count at which the
		 * HW promotes the staged AES key. The OLT will not advance to OMCI until
		 * this handshake completes, so a missing handler leaves it re-cycling
		 * Request_Key/Configure_Port-ID forever. De-duped per superframe. */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			u32 fc = ((u32)(d[0] & 0x3f) << 24) | ((u32)d[1] << 16) |
				 ((u32)d[2] << 8) | d[3];

			/* Arm only once a key is actually staged: arming a switch to an empty or
			 * stale bank would promote garbage and corrupt AES. ACK either way. */
			if (gpon_key_staged && fc != gpon_aes_switch_time) {
				gpon_aes_arm_switch(fc);
				pr_info("luna-gpon: Key_Switching_Time -> arm switch @superframe %u\n",
					fc);
			}
			gpon_send_ack(m);
			pr_info("luna-gpon: EVT t=%u KEY_SW(0x13) staged=%d arm@%u hwswt=%u dsrx_omcc=%u pirx=%u omcirx=%d\n",
				gpon_fsm_ticks, gpon_key_staged, gpon_aes_switch_time,
				gpon_rd(GPON_AES_KEY_SWITCH_TIME) & 0x3fffffff,
				gpon_gem_ds_rx_cnt(GPON_OMCC_FLOW), sw_rd(OMCI_RX_PKT_CNT),
				gpon_omci_rx_cnt());
		}
		break;
	case PLM_DS_ENCRYPT_PORT:
		/* Encrypted_Port-ID (0x08): G.984.3 requires a US Acknowledge; the OLT
		 * arms a ~43s timer and Deactivates us if none arrives. */
		if (onu_id == gpon_fsm_onu_id) {
			gpon_send_ack(m);
			if (trace)
				pr_info_ratelimited("luna-gpon: ACK type=0x%02x d=%*phN\n",
						    type, 8, d);
		}
		break;
	case PLM_DS_CFG_VPVC:
		/* Configure_VP/VC (0x07): legacy ATM setup, unsupported on a GEM ONU, but
		 * stock still acknowledges and a missing AK raises LOAi. */
		if (onu_id == gpon_fsm_onu_id) {
			gpon_send_ack(m);
			pr_info_ratelimited("luna-gpon: ACK CFG_VPVC(0x07)\n");
		}
		break;
	case PLM_DS_BER_INTERVAL:
		/* BER_interval (0x12): the OLT configures the upstream BER reporting interval
		 * and REQUIRES an Acknowledge. The ACK is the part that prevents LOAi; REI
		 * reporting is informational and not required to stay activated. */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			gpon_send_ack(m);
			pr_info_ratelimited("luna-gpon: ACK BER_INTERVAL(0x12) d=%*phN\n",
					    8, d);
		}
		break;
	default:
		/* Any DS PLOAM addressed to us that we do not model, logged so a missing
		 * AK-required type is visible instead of a silent drop -> LOAi. PEE(0x0f),
		 * PowerLevel(0x10), PST(0x11) and Rang_Adjust(0x17) need no ACK in G.984.3. */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			/* A downstream PLOAM type another vendor's OLT sends and we do not
			 * model is not a fault -- it is the work list, and the dump is what
			 * makes it implementable. class=unknown for that reason: a foreign
			 * OLT must never be published as a broken device. `n=` is cumulative
			 * and the backoff prints 1,2,4,8,..., so suppression can never HIDE
			 * its own size the way the kernel's shared token bucket did, and the
			 * text is the one spelling unsup_scan.py parses. The dump is the
			 * WHOLE 13-octet message, because half a PLOAM implements nothing. */
			gpon_unsup_report("ds_ploam_type", GPON_UNSUP_UNKNOWN,
					  type, "G.984.3-DS-type-this-ONU-models",
					  m, 13);
		}
		break;
	}
}

/*
 * Has the WAN netdev received NOTHING?  Both watchdogs below tear a RANGED ONU
 * down on this answer, so it has to distinguish two things a bare `== 0` does
 * not: a link that is dead, and a shell that cannot answer.
 *
 * ★ IT WAS A BARE `rtl9602c_eth_wan_rx_count() == 0`, TWICE, AND IT WAS RIGHT
 * BY ARITHMETIC ACCIDENT.  A shell with no WAN netdev returns
 * GPON_OMCI_RX_UNAVAIL, which is 0xffffffff and therefore not 0 -- so the
 * watchdogs happened not to fire on a board that simply had no instrument.
 * That is a coincidence, not a rule, and the day the sentinel changes or a
 * caller compares differently it becomes a re-range loop on a healthy ONU.
 * COULD NOT ASK IS NOT DEAD, said in code.
 */
static bool gpon_wan_rx_silent(void)
{
	u32 n = rtl9602c_eth_wan_rx_count();

	return n != GPON_OMCI_RX_UNAVAIL && n == 0;
}

/* Apply identity cleanup while US is inhibited, before any rearm. The timer
 * also consumes early parameter changes through this same existing logic. */
static void gpon_apply_identity_change(void)
{
	if (core_fsm)
		gpon_ploam_sn_changed(&luna_ploam, gpon_fsm_ticks * GPON_FSM_TICK_MS);
	if (!core_fsm && gpon_sn_changed) {
		gpon_sn_changed = false;
		if (gpon_fsm_state > 1) {
			gpon_fsm_onu_id = 0xff;
			gpon_omcc_installed = false;
			gpon_omcc_installed_gem = 0;
			gpon_tcont_installed = false;
			gpon_data_installed = false;	/* re-install WAN data GEM on re-config */
			/* An SN reprovision is an IDENTITY CHANGE, so it clears
			 * gpon_data_gem_solicited exactly like an OLT Deactivate: whatever
			 * ME268 the OLT holds belongs to the serial we have stopped being.
			 * Keeping it made the NEW identity install its data GEM on the
			 * PREVIOUS identity's gem-port. NOT the fiber-LOS case below, where
			 * the OLT never deactivated us and keeps our provisioning. */
			gpon_data_gem_solicited = false;
			gpon_data_tcont_installed = false;
			gpon_data_alloc = 0;
			gpon_aes_switch_time = 0xffffffff;
			gpon_key_staged = false;
			gpon_field(GPON_GTC_DS_ONU_ID_STATUS, 15, 8, 0xff);
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
			gpon_fsm_set_state(1);
			pr_info("luna-gpon: SN reprovisioned (%8phN) -> re-ranging\n",
				gpon_sn_bytes);
		}
	}
}

static void gpon_fsm_poll(struct timer_list *t)
{
	int guard = 0;

	gpon_fsm_ticks++;
	{
		unsigned long now = jiffies;

		luna_poll_gap_ms = luna_poll_prev_jiffies ?
			jiffies_to_msecs(now - luna_poll_prev_jiffies) : 0;
		luna_poll_prev_jiffies = now;
	}
	luna_bwcap_poll();
	if (!READ_ONCE(luna_activation_ready))
		goto drain_downstream;
	/*
	 * Stage 2 of the A/B: the PERIODIC half, behind the same `core_fsm` switch
	 * that selects the downstream dispatch. Clear (the default) means every
	 * block below runs as before and the core's polls are never entered.
	 * The shape is `if (core_fsm) <core>;` beside an untouched
	 * `if (!core_fsm && <original condition>)` rather than an if/else around a
	 * restructured body: these blocks are 428 lines of shell work and FSM
	 * decisions interleaved, and a diff that moves them cannot be reviewed
	 * against the FSM it is meant to reproduce.
	 * Ticks x 10, NOT the wall clock: ONU-test-case/OWED-ploam-swap-time-unit.md.
	 */
	if (core_fsm)
		gpon_ploam_tick(&luna_ploam);
	gpon_led_los_set((gpon_rd(GPON_GTC_DS_LOS_CFG_STS) & GPON_OPTIC_LOS_SIG) != 0);
	/* The serial was (re)provisioned after ranging began. Drop to O1 and re-offer
	 * the new Serial_Number. */
	gpon_apply_identity_change();
drain_downstream:
	while (!(gpon_rd(GPON_GTC_DS_PLOAM_IND) & GPON_DS_PLM_BUF_EMPTY) &&
	       guard++ < 16) {
		u8 m[13];

		/* The word-unpack is the core's (gpon_gtc_ploam.h, x86-proven); this shell
		 * contributes the accessor and the per-SoC offset. The DEQ below advances
		 * the queue even on a refusal: the queue discipline is this loop's. */
		luna_rx_burst_idx = (u8)(guard - 1);	/* 0 = the first this poll */
		if (gpon_gtc_ds_ploam_read(&gpon_io, GPON_GTC_DS_PLOAM_MSG, m))
			gpon_fsm_handle(m);
		gpon_ds_rx++;					/* DS-lock liveness */
		gpon_wr(GPON_GTC_DS_PLOAM_IND, GPON_DS_PLM_DEQ);	/* advance */
	}
	if (!READ_ONCE(luna_activation_ready))
		goto next_poll;
	/* Both provisioning follow-ups are one core poll: the WAN data GEM once the
	 * OLT has issued its ME268 GEM-CTP Create -- idempotently over the OLT's gem,
	 * never proactively ahead of it, which was the 2nd-admit churn cause -- and
	 * the VEIP oper-up AVC. The AVC must fire AFTER config-apply AND after the
	 * OLT has created ME329 (~2500 ticks after O5); firing it during config
	 * DISRUPTED config. The OLT never polls the data MEs it creates: it un-gates
	 * downstream user data only when the ONU reports the port up.
	 * The AVC's 3 / 2500 / 150 live in the core as GPON_PLOAM_AVC_MAX,
	 * _DELAY_TICKS and _PERIOD_TICKS, checked against the values below. */
	if (core_fsm)
		gpon_ploam_poll_provision(&luna_ploam, gpon_fsm_ticks * GPON_FSM_TICK_MS);
	luna_omci_service();

	if (!core_fsm && gpon_fsm_state == 5 && gpon_omcc_installed && gpon_avc_sent < 3 &&
	    gpon_o5_entry_tick && (gpon_fsm_ticks - gpon_o5_entry_tick) > 2500 &&
	    ((gpon_fsm_ticks - gpon_o5_entry_tick) % 150) == 0) {
		rtl9602c_eth_omci_report_oper_up();
		gpon_avc_sent++;
	}

	/* feed_rekick: per-tick self-terminating US-feed FIFO re-arm. The one-shot
	 * O5-entry re-arm is re-parked by later SerDes resets before the OLT's first
	 * grant, so pages stage in PON-IP SRAM but never build a DRAM descriptor and
	 * the framer emits nothing. Auto-stops once gemus_omcc advances. */
	if (feed_rekick && gpon_fsm_state == 5 && gpon_omcc_installed) {
		u32 dsc = pi_rd(PI_PON_DSC_STS_US);

		if ((dsc & 0x1fffu) > 0 && ((dsc >> 16) & 0x1fffu) == 0 &&
		    gpon_rd(GEM_US_STAT(GPON_OMCC_FLOW)) == 0)
			gpon_us_feed_rearm_light();
	}

	/* us_intr_svc: ack the upstream GPON interrupt deltas the known-good unit
	 * services on every GTC_US event. The reads clear the sticky latch; if the
	 * fetch FSM was back-pressuring on it, the payload framer unstalls. */
	if (us_intr_svc && gpon_fsm_state == 5 && gpon_omcc_installed) {
		u32 gtcus_dlt = gpon_rd(GPON_GTC_US_INTR_DLT);	/* read-to-clear GTC_US delta */
		u32 gemus_dlt = gpon_rd(GPON_GEM_US_INTR_DLT);	/* read-to-clear GEM_US delta */

		(void)gpon_rd(GPON_GTC_US_INTR_STS);			/* GTC_US_INTR_STS */
		(void)gpon_rd(GPON_GEM_US_INTR_STS);			/* GEM_US_INTR_STS */
		if (gtcus_dlt || gemus_dlt)
			gpon_us_intr_svc_cnt++;
	}

	/* O5 provisioning watchdog. A boot that reached O5 locally but was never
	 * provisioned (gpon0 RX still 0 well past the slow-lease window) is stuck on a
	 * non-frameable US-TX serializer phase with no OLT Deactivate to recover it.
	 * Self-re-range to RE-ROLL the phase, including the CDR/reset-B re-seat that
	 * actually changes the serializer lock. wan_rx>0 on any working or
	 * slow-leasing link, so this fires only on a genuinely dead one. */
	if (core_fsm)
		gpon_ploam_poll_watchdog(&luna_ploam, gpon_wan_rx_silent(),
					 gpon_fsm_ticks * GPON_FSM_TICK_MS);
	if (!core_fsm && o5_provision_watchdog_ticks && gpon_fsm_state == 5 &&
	    gpon_fsm_onu_id != 0xff && gpon_o5_entry_tick &&
	    (gpon_fsm_ticks - gpon_o5_entry_tick) > o5_provision_watchdog_ticks &&
	    gpon_wan_rx_silent()) {
		pr_info("luna-gpon: O5 provision watchdog (%u ticks, gpon0 RX=0) -> re-range to re-roll serializer phase\n",
			gpon_fsm_ticks - gpon_o5_entry_tick);
		gpon_fsm_onu_id = 0xff;
		gpon_omcc_installed = false;
		gpon_omcc_installed_gem = 0;
		gpon_tcont_installed = false;
		gpon_data_installed = false;
		/* The data Alloc-ID bind is session state and the OLT may reissue a
		 * different one; gpon_data_gem_solicited is deliberately KEPT, because
		 * this re-range is ONU-initiated and the OLT holds our provisioning. */
		gpon_data_tcont_installed = false;
		gpon_data_alloc = 0;
		gpon_aes_switch_time = 0xffffffff;
		gpon_key_staged = false;
		gpon_field(GPON_GTC_DS_ONU_ID_STATUS, 15, 8, 0xff);
		gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
		if (cdr_reseat_on_reactivate) {
			gpon_cdr_reseat();
			schedule_work(&gpon_cdr_reset_work);
		}
		gpon_fsm_set_state(1);
	}

	/* Autonomous downstream-LOS recovery (fiber pull). The OLT cannot send a
	 * Deactivate when downstream light is gone, so the ONU must notice the
	 * sustained optical-LOS itself, tear down to O1 and re-acquire on relight.
	 * Debounced against transient dips; once at O1 the state<2 guard stops
	 * counting until the FSM climbs back past O1. */
	if (los_rerange_ticks && gpon_fsm_state >= 2) {
		bool optic_los = !!(gpon_rd(GPON_GTC_DS_LOS_CFG_STS) & GPON_OPTIC_LOS_SIG);
		/* A chip with no declared SDS_FIB_STATUS has no second witness, and an
		 * absent witness may not be manufactured into one. */
		bool sds_dark  = SDS_FIB_STATUS &&
				 !(sw_rd(SDS_FIB_STATUS) & SDS_FIB_SDS_SDET);

		/* Real DS-light loss = the GTC optical-LOS AND the SoC SerDes
		 * signal-detect both gone. An I2C pad-steal perturbs optic_los alone, an
		 * internal SerDes re-seat can blip sds_sdet alone; only a true fibre pull
		 * drops BOTH, so the AND lets the debounce stay short.
		 * This is the one poll of this set that is LIVE by default
		 * (los_rerange_ticks = 30) and the path this board is validated on, so
		 * flipping core_fsm here must be followed by N PHYSICAL fibre pulls.
		 * The two witnesses are read HERE and handed over, never re-derived
		 * inside the core -- a core that re-read them would have to know both
		 * board facts. */
		if (core_fsm)
			gpon_ploam_poll_los(&luna_ploam, optic_los, sds_dark,
					    gpon_fsm_ticks * GPON_FSM_TICK_MS);
		if (!core_fsm && optic_los && sds_dark) {
			if (++gpon_los_run == los_rerange_ticks) {
				pr_info("luna-gpon: downstream LOS %u ticks (optic_los & !sds_sdet) -> O1 (re-range on light return)\n",
					gpon_los_run);
				gpon_fsm_onu_id = 0xff;
				gpon_omcc_installed = false;
				gpon_omcc_installed_gem = 0;
				gpon_tcont_installed = false;
				gpon_data_installed = false;
				/* The data Alloc-ID bind IS session state: the OLT may
				 * reissue a different Alloc-ID on re-admit and the
				 * install guard must not refuse it. */
				gpon_data_tcont_installed = false;
				gpon_data_alloc = 0;
				/* Do NOT reset gpon_data_gem_solicited on a fiber-LOS re-range: it is
				 * ONU-initiated, the OLT never Deactivated us, so it keeps our
				 * OMCI/GEM provisioning and does NOT re-send the ME268 on re-admit.
				 * Resetting it made the data GEM wait for a create that never arrives
				 * -- O5 re-acquired with no WAN. This is not the 2nd-admit churn: that
				 * was a fresh admit where the OLT had not yet created the GEM. */
				gpon_aes_switch_time = 0xffffffff;
				gpon_key_staged = false;
				gpon_field(GPON_GTC_DS_ONU_ID_STATUS, 15, 8, 0xff);
				gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
				if (cdr_reseat_on_reactivate) {
					gpon_cdr_reseat();
					schedule_work(&gpon_cdr_reset_work);
				}
				gpon_fsm_set_state(1);
			}
		} else {
			gpon_los_run = 0;
		}
	}

	/* Hybrid LAN/VLAN: clear VLAN_FILTER so the LAN ports forward to the CPU.
	 * This silicon does not forward LAN<->CPU while the ingress VLAN filter is
	 * on, so the host's ARP round-trip dies until it is cleared.
	 * lan_keep_open (default): the LAN must be reachable from boot, independent
	 * of the FSM state. rtl9602c_eth_open() runs at userspace ifup, LATER than
	 * this poll, so it can re-assert the filter after a one-shot clear. Clearing
	 * on EVERY poll is idempotent, costs one MMIO, and undoes a late re-assert
	 * within a tick. */
	if (lan_keep_open) {
		sw_field(SW_VLAN_CTRL, 0, 0, 0);		/* VLAN_FILTER off -> LAN open, every state */
		if (!gpon_vlan_lan_open) {
			gpon_vlan_lan_open = true;
			pr_info("luna-gpon: lan_keep_open -> VLAN_FILTER off (LAN access open)\n");
		}
	} else if (gpon_fsm_state == 5 && !gpon_vlan_lan_open && vlan_lan_o5_ticks &&
		   gpon_o5_entry_tick && (gpon_fsm_ticks - gpon_o5_entry_tick) > vlan_lan_o5_ticks) {
		/* legacy O5-gated: keep filtering through ranging and config-apply, then
		 * clear once O5 has held. Re-armed on any drop below O5. */
		sw_field(SW_VLAN_CTRL, 0, 0, 0);		/* VLAN_FILTER off -> open LAN */
		gpon_vlan_lan_open = true;
		pr_info("luna-gpon: O5 stable %u ticks -> VLAN_FILTER off (LAN access open)\n",
			gpon_fsm_ticks - gpon_o5_entry_tick);
	}
	if (trace && gpon_fsm_state == 5 && (gpon_fsm_ticks % 150) == 0)
		/* rxsid/ustx/dirty were REMOVED from this fast O5 print: pi_rd in this
		 * context reproducibly HANGS the poll right after OMCC install (two boots
		 * identical). It is an indirect polled PON-IP access, unsafe here unlike
		 * sw_rd/gpon_rd; read those counters through /proc instead. */
		pr_info("luna-gpon: O5 t=%u last=0x%02x onu=%u hwst=%u eqd=0x%08x | dsrx_omcc=%u pirx=%u omcirx=%d | ploam_cpu=%u gem_byte=%u gemus_omcc=%u idle16=%u idle8=%u\n",
			gpon_fsm_ticks, gpon_last_ds_type, gpon_fsm_onu_id,
			gpon_rd(GPON_GTC_DS_ONU_ID_STATUS) & 0xf, gpon_rd(GPON_GTC_US_EQD),
			gpon_gem_ds_rx_cnt(GPON_OMCC_FLOW), sw_rd(OMCI_RX_PKT_CNT),
			gpon_omci_rx_cnt(),
			gpon_us_misc_cnt(2), gpon_us_misc_cnt(4),
			gpon_rd(GEM_US_STAT(GPON_OMCC_FLOW)), gpon_rd(TCONT_IDLE_STAT(16)),
			gpon_rd(TCONT_IDLE_STAT(GPON_DATA_TCONT)));
	/* US-OMCI egress stall localizer, SAFE reads only (no pi_rd, no cross-driver
	 * accessor -- both were the suspected hang sources). The US OMCI is queued to
	 * qid64 but gemus_omcc stays 0 while idle16 climbs, so the OLT sees a silent
	 * upstream T-CONT. ustx=0 means the frame never entered the US OMCI-PI TX
	 * path; gemus_scan_first separates an unscheduled flow 64 from a SID
	 * misroute from a US-NIC ingest drop. */
	if (trace && gpon_fsm_state == 5 && (gpon_fsm_ticks % 150) == 0) {
		int j;
		/* OLT-independent US-OMCI datapath self-test: inject synthetic OMCI frames
		 * through OUR TX path, so the rxsid read below reflects whether our
		 * steering reaches the US-NIC with no dependency on the OLT. */
		for (j = 0; j < 1; j++)	/* de-burst: ONE inject/tick. The 4-burst in one softirq tick overran the GMAC TX fetch engine — it drained ~3, parked at the producer head, the ring filled, and (no TDU re-kick) stayed parked, freezing ALL GMAC TX. */
			rtl9602c_eth_omci_selftest();
		/* rxsid = RX_SID_GOOD_CNT_US[0..4]; group [4] is the OMCC SID. Non-zero
		 * means the US OMCI frame DOES reach the US-NIC classifier, so the stall
		 * is downstream at the queue/scheduler; zero with ustx=0 means it never
		 * reaches the US-NIC at all. pi_rd is safe in this context. */
		pr_info("luna-gpon: USDIAG t=%u ustx=%u pirx=%u usdrop=%u uscrc=%u | rxsid=%u/%u/%u/%u/%u\n",
			gpon_fsm_ticks, sw_rd(OMCI_TX_PKT_CNT), sw_rd(OMCI_RX_PKT_CNT),
			sw_rd(OMCI_DROP_PKT_CNT), sw_rd(OMCI_CRC_ERROR_PKT_CNT),
			(u32)pi_rd(PI_RX_SID_GOOD_CNT_US), (u32)pi_rd(0x2040), (u32)pi_rd(0x2044),
			(u32)pi_rd(0x2048), (u32)pi_rd(0x204c));
	}
	/* DS-pipeline stage probe: A=de-encap, B=PBO high-queue, C=DS SRAM,
	 * D=PON-IP->NIC. The first zero along A->D is the stall stage. */
	if (gem_gate_open && gpon_fsm_state == 5 && (gpon_fsm_ticks % 100) == 0) {
		/* De-encap count per flow: localizes whether OMCI de-encaps ANYWHERE (a
		 * mapping issue) or nowhere (the OLT is not sending OMCI). */
		pr_emerg("DSPIPE deenc f64=%u f3=%u f2=%u f1=%u f0=%u | nonidle=%u idle=%u los=%u hec=%u | sram=%u q0=%u rxok=%u\n",
			 gpon_gem_flow_cnt(GPON_OMCC_FLOW, 0), gpon_gem_flow_cnt(3, 0),
			 gpon_gem_flow_cnt(2, 0), gpon_gem_flow_cnt(1, 0),
			 gpon_gem_flow_cnt(0, 0),
			 gpon_rd(GPON_GTC_DS_MISC_CNTR_GEM_NON_IDLE), gpon_rd(GPON_GTC_DS_MISC_CNTR_GEM_IDLE), gpon_rd(GPON_GTC_DS_MISC_CNTR_GEM_LOS),
			 gpon_rd(GPON_GTC_DS_MISC_CNTR_HEC_CORRECT),
			 pi_rd(PI_PON_DSC_USAGE_DS) & 0x1fff,
			 pi_rd(PI_PON_DS_PBO_PAGE_Q0) & 0x1fff,
			 pi_rd(PI_PKT_OK_CNT_DS) & 0xffff);
	}
	/* Periodic SerDes-TX re-sync while UN-RANGED. The upstream-burst serializer
	 * lock is non-deterministic, so re-pulse the TX-interface reset-B
	 * (WSDS_DIG_1D[16]) ~every 2 s to keep re-attempting a lock onto the framer
	 * burst data. TX-interface only, so the locked RX framer is undisturbed. */
	if (unranged_reseat && gpon_fsm_state >= 3 && gpon_fsm_onu_id == 0xff &&
	    (gpon_fsm_ticks % 200) == 0) {
		gpon_cdr_reseat();
		gpon_sds_synced++;
	}
	/* While unregistered in O3, re-offer our Serial_Number_ONU ~twice a second
	 * (the OLT grants SN windows intermittently). */
	if (core_fsm)
		gpon_ploam_poll_sn_reoffer(&luna_ploam, gpon_fsm_ticks * GPON_FSM_TICK_MS);
	if (!core_fsm && gpon_fsm_state >= 3 && gpon_fsm_onu_id == 0xff &&
	    (gpon_fsm_ticks % 50) == 0)
		gpon_send_sn();
	/* Periodic O5 upstream-PLOAM keepalive. Once ranged the FSM otherwise emits
	 * ZERO upstream PLOAM, and the shared buffer's auto-No_message template can
	 * be stale-clobbered by intervening ACK/SN sends. A fresh No_message every
	 * o5_ploam_keepalive_ticks keeps a valid PLOAM in each granted slot,
	 * defeating the PLOAM-liveness timeout that fires Deactivate 25-35 s after
	 * provisioning on about half of boots. */
	if (core_fsm)
		gpon_ploam_poll_keepalive(&luna_ploam, gpon_fsm_ticks * GPON_FSM_TICK_MS);
	if (!core_fsm && gpon_fsm_state == 5 && gpon_fsm_onu_id != 0xff &&
	    o5_ploam_keepalive_ticks &&
	    (gpon_fsm_ticks % o5_ploam_keepalive_ticks) == 0) {
		u8 nomsg[12];

		memset(nomsg, 0xaa, sizeof(nomsg));
		nomsg[0] = 0xff;	/* ONU-ID (HW overrides via ONUID_OVRD) */
		nomsg[1] = 0x04;	/* GPON_PLOAM_US_NOMESSAGE */
		gpon_send_cpu_ploam(PLM_US_QUEUE_NOMSG, nomsg);
	}

	/* Runtime DS-CDR-wedge recovery -- the stock link-state-check this driver was
	 * missing. If the GTC DS framer status latches the wedge sentinel, the DS CDR
	 * came up stuck (a cold-start lock no soft reboot clears); re-acquire by
	 * toggling SP_SDS_EN_RX 1->0->1 as stock does, as a two-tick toggle so no
	 * 10 ms busy-wait runs in this softirq. RATE-bounded, never count-capped. It
	 * must NOT hand off to the LOS/re-range path: that is gated on state >= 2 and
	 * a wedged framer delivers no PLOAM, so the FSM never leaves O1. */
	if (cdr_stuck_recover) {
		static int cdr_pending;
		u32 sts = gpon_rd(GPON_GTC_DS_INTR_STS);

		gpon_gtc_ds_sts_last = sts;
		if (cdr_pending) {
			sw_field(SDS_REG0, 1, 1, 1);		/* SP_SDS_EN_RX -> 1 */
			cdr_pending = 0;
			if (gpon_rd(GPON_GTC_DS_INTR_STS) != GTC_DS_CDR_STUCK)
				gpon_cdr_stuck_fixed++;
		} else if (sts == GTC_DS_CDR_STUCK) {
			if (gpon_cdr_stuck_tries < GPON_CDR_STUCK_MAX ||
			    (gpon_fsm_ticks % GPON_CDR_STUCK_SLOW_TICKS) == 0) {
				sw_field(SDS_REG0, 1, 1, 0);	/* SP_SDS_EN_RX -> 0 */
				cdr_pending = 1;
				gpon_cdr_stuck_tries++;
				gpon_cdr_stuck_count++;
				pr_warn_ratelimited("luna-gpon: DS CDR wedged (GTC_DS_STS=0x%08x); SP_SDS_EN_RX re-acquire #%u\n",
						    sts, gpon_cdr_stuck_count);
			}
		} else {
			gpon_cdr_stuck_tries = 0;	/* healthy -> fresh fast budget */
		}
	}

	/* Periodic DS multiframe/BWmap ESD-recover (stock gpon_esdRecover_expire,
	 * ~5 s): re-roll the DS word phase when the framer is byte-locked but the
	 * PLEND/LOM parse fails, which leaves the BWmap unlocatable and the ONU
	 * grant-deaf. Read the fail counter only while LOF is clear. The CDR pulse
	 * itself runs in a workqueue so its 10 ms wait stays off this softirq. */
	if (gpon_esd_recover && gpon_fsm_state >= 3) {
		static unsigned long esd_last_j;
		static bool esd_init;
		static u32 esd_relocks;

		if (!esd_init) {
			esd_last_j = jiffies;
			esd_init = true;
		} else if (time_after(jiffies,
				      esd_last_j + msecs_to_jiffies(GPON_ESD_INTERVAL_MS))) {
			u32 sts = gpon_rd(GPON_GTC_DS_INTR_STS);

			esd_last_j = jiffies;
			if (!(sts & GTC_DS_STS_LOF)) {		/* framer byte-locked */
				u32 v = gpon_rd(GPON_GTC_DS_MISC_CNTR_LOM);
				u32 fail = ((v >> 16) & 0xffff) + (v & 0xffff);

				if (fail > GPON_ESD_THRESHOLD) {
					esd_relocks++;
					pr_warn_ratelimited("luna-gpon: ESD-recover: DS PLEND/LOM fail=%u byte-locked at O%u (BWmap mis-phased) -> RX-CDR re-lock #%u\n",
							    fail, gpon_fsm_state, esd_relocks);
					schedule_work(&gpon_cdr_reset_work);
				}
			}
		}
	}


next_poll:
	if (!READ_ONCE(luna_stopping))
		mod_timer(&gpon_fsm_timer, jiffies + GPON_FSM_TICK_JIFFIES);
}

/* Stock's omitted LDO init step, byte-exact. SC-indirect (SerDes-Control)
 * analog-reg engine via swcore: SC_IND_CMD ADR[15:0]/CMD_EN[16]/WREN[17],
 * SC_IND_RD RD_DAT[7:0]/BUSY[8], SC_IND_WD WR_DAT[7:0]; stock settles 1 ms per
 * step and never polls BUSY. Register 0xfdca disables the DRAM LDO output so
 * the DDR switching regulator drives the rail. RE verdict: DRAM-LDO plus a
 * thermal ALARM, NOT the SerDes/laser path -- platform hygiene, not the WAN fix. */
static void __init rtl9602c_sc_ldo_init(void)
{
	u32 fdca, t130 = sw_rd(THERMAL_CTRL_0);

	if (!sc_ldo_init)
		return;
	sw_wr(SC_CMD, 0x0001fdcau);		/* SC read cmd: reg 0xfdca */
	udelay(1000);
	fdca = sw_rd(SC_DATA);
	pr_info("luna-gpon: sc_ldo_init BEFORE: 0xfdca=0x%08x THERMAL(0x130)=0x%08x (stock 0x130=0x00ec0005)\n",
		fdca, t130);
	sw_wr(SC_IND_WD, fdca & ~0xcu);		/* clear DRAM-LDO bits 2,3 (stock mask ~0xC) */
	udelay(1000);
	sw_wr(SC_CMD, 0x0003fdcau);		/* SC write commit */
	sw_wr(SC_CMD, 0x0001fdcau);		/* re-issue read (stock reads back for log) */
	udelay(1000);
	(void)sw_rd(SC_DATA);
	sw_wr(THERMAL_CTRL_0, sw_rd(THERMAL_CTRL_0) | 0x00800000u);			/* THERMAL: set bit23 */
	sw_wr(THERMAL_CTRL_0, (sw_rd(THERMAL_CTRL_0) & 0xff80ffffu) | 0x006c0000u);	/* preserve low-16 */
	pr_info("luna-gpon: sc_ldo_init AFTER: 0x130=0x%08x\n", sw_rd(THERMAL_CTRL_0));
}

/*
 * The laser's enable pins are SoC GPIOs on some boards, and the driver that
 * NEEDS the laser owns them. Two BOARD facts, declared in that board's DTS
 * under /pon-optics: realtek,tx-disable-gpio (driven 0 = laser may emit) and
 * realtek,tx-power-gpio (driven 0 = powered, active-low).
 *
 * MEASURED on the G24W, stock at O5, 2026-09-06: PABCD_DIR 0x00008014 (pins 2,
 * 4, 15 outputs), PABCD_DAT with bits 2 and 4 LOW, IO_GPIO_EN 0x00008014. Ours
 * read DIR 0x8000 -- both pins INPUTS, so the laser driver saw whatever its
 * pull-ups gave it and nothing in the GTC could reach the fibre.
 *
 * The MECHANISM is the family's (SoC GPIO block at 0x18003300, DIR/DAT per
 * 32-pin bank, plus the switch's IO_GPIO_EN); only the pin NUMBERS are the
 * board's, and a board declaring none keeps the behaviour it had. The level is
 * written BEFORE the pin becomes an output, so it never glitches.
 */
static int laser_tx_dis_gpio = -1;	/* from DT; -1 = this board has none */
static int laser_tx_pwr_gpio = -1;

static void gpon_optical_work_fn(struct work_struct *w)
{
	static unsigned long optical_next;

	mutex_lock(&bosa_lock);
	if (luna_driver_ready && !luna_stopping) {
		if (bosa_regs_live() && !laser_off && !skip_bosa) {
			if (bosa_laser_up)
				bosa_laser_maint();
			/* OFFK runtime servo (stock europa_LoopMon equivalent): latch the RTL8290B
			 * modulator-offset cal. Run from boot and NOT gated on O5 -- stock converges
			 * it continuously from driver init so the modulator is nulled BEFORE ranging
			 * and the first real bursts are clean; gating on O5 latched ~17 s too late.
			 * OFFK uses an internal reference, so no burst is needed. */
			if (apc_offk_armed && !apc_offk_latched) {
				int r;

				bosa_write_reg(BOSA_REG_W77, BOSA_W77_BIAS_MAX_EN | BOSA_W77_MOD_MAX_EN |
					       BOSA_W77_MOD_MAX_LOADIN);	/* 0xb0, as the done-check */
				r = bosa_read_reg(BOSA_REG_R29);
				if (r >= 0 && (r & 0x3c) == 0x3c) {
					bosa_set_bit(0x20e, 7, 0);
					bosa_set_bit(0x27c, 4, 0);
					apc_offk_latched = 1;
					pr_info("luna-gpon: OFFK LATCHED: R29(0x31d)=0x%02x (modulator nulled)\n",
						r & 0xff);
				}
			}
		}
		if (optical_poll && (bosa_regs_live() || bosa_cal_ready) &&
		    time_after_eq(jiffies, optical_next)) {
			gpon_optical_cache_poll();
			rtl9602c_eth_omci_set_optical(anig_rx_level, anig_tx_level);
			optical_next = jiffies + msecs_to_jiffies(3000);
		}
	}
	mutex_unlock(&bosa_lock);
	if (!READ_ONCE(luna_stopping))
		schedule_delayed_work(&gpon_optical_work, msecs_to_jiffies(50));
}

static void __init luna_laser_gpio_from_dt(void)
{
	struct device_node *np = of_find_node_by_path("/pon-optics");
	u32 v;

	if (!np)
		return;
	if (!of_property_read_u32(np, "realtek,tx-disable-gpio", &v))
		laser_tx_dis_gpio = (int)v;
	if (!of_property_read_u32(np, "realtek,tx-power-gpio", &v))
		laser_tx_pwr_gpio = (int)v;
	of_node_put(np);
	pr_info("luna-gpon: /pon-optics: tx-disable-gpio=%d tx-power-gpio=%d\n",
		laser_tx_dis_gpio, laser_tx_pwr_gpio);
}

/* Set level before enabling output to avoid a TX_DISABLE glitch. */
static int luna_gpio_drive(int pin, bool high, const char *what)
{
	void __iomem *gpio;
	u32 dir_off, dat_off, bit, v;
	int ret = 0;

	if (pin < 0)
		return 0; /* Boards without this GPIO keep their existing mechanism. */
	if (pin >= 64 || !swc->io_gpio_en)
		return -EINVAL;
	gpio = ioremap(GPIO_PHYS_BASE, GPIO_REG_SIZE);
	if (!gpio)
		return -ENOMEM;
	dir_off = pin < 32 ? GPIO_DIR_ABCD : GPIO_DIR_EFGH;
	dat_off = pin < 32 ? GPIO_DATA_ABCD : GPIO_DATA_EFGH;
	bit = 1u << (pin & 31);
	v = ioread32(gpio + dat_off);
	iowrite32(high ? v | bit : v & ~bit, gpio + dat_off);
	v = ioread32(gpio + dir_off);
	iowrite32(v | bit, gpio + dir_off);
	sw_field(SOC_IO_GPIO_EN + 4 * (pin / 32), pin & 31, pin & 31, 1);
	if (!!(ioread32(gpio + dat_off) & bit) != high ||
	    !(ioread32(gpio + dir_off) & bit) ||
	    !(sw_rd(SOC_IO_GPIO_EN + 4 * (pin / 32)) & bit))
		ret = -EIO;
	iounmap(gpio);
	if (ret)
		pr_err("luna-gpon: %s GPIO %d level %u failed: %d\n",
		       what, pin, high, ret);
	return ret;
}

static void luna_resume_poll(void)
{
	if (luna_driver_ready && !luna_stopping && !timer_pending(&gpon_fsm_timer))
		mod_timer(&gpon_fsm_timer, jiffies + msecs_to_jiffies(50));
}

/* Process context under bosa_lock. Timer callbacks never acquire this mutex. */
static int luna_quiesce_locked(void)
{
	WRITE_ONCE(luna_activation_ready, false);
	if (luna_driver_ready) {
		timer_delete_sync(&gpon_fsm_timer);
		cancel_work_sync(&gpon_cdr_reset_work);
	}
	gpon_wr_us_protected(GPON_GTC_US_CFG, 0);
	gpon_wr(GPON_GTC_US_PLOAM_CFG, 0);
	gpon_wr(GPON_GTC_US_PLOAM_IND, 0);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_UNLOCK);
	gpon_field(GPON_GTC_US_PROC_MODE, 0, 0, 0);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_LOCK);
	pi_field(PI_PONIP_CTL_US, 0, 0, 0);
	return luna_gpio_drive(laser_tx_dis_gpio, true, "TX_DISABLE inhibit");
}

static int luna_activate_locked(bool identity_changed)
{
	struct gpon_range_request request = {
		.identity_defined = gpon_sn_is_set(gpon_sn_bytes),
		.identity_changed = identity_changed,
		.ranging_now = luna_activation_ready,
		.activation_wanted = !gpon_hold,
		.laser_wanted = !laser_off && !skip_bosa,
	};
	struct gpon_range_action action;
	u8 nomsg[12];
	int ret;

	gpon_range_plan(&request, &action);
	if (!action.program) {
		if (action.quiesce) {
			ret = luna_quiesce_locked();
			if (ret)
				return ret;
		}
		return action.result;
	}
	ret = luna_quiesce_locked();
	if (ret)
		goto fail;
	ret = luna_gpio_drive(laser_tx_pwr_gpio, false, "laser TX power");
	if (ret)
		goto fail;
	if (bosa_regs_live()) {
		/* Preserve the positive RTL8290B boot path. GN never arms its servo. */
		ret = bosa_laser_up ? 0 : -ENODEV;
	} else {
		ret = luna_gn_calibrate_locked();
	}
	if (ret)
		goto fail;
	gpon_apply_identity_change();
	/* US remains disabled while the checked physical inhibit is released. The
	 * timer is stopped until the complete arming sequence has finished. */
	ret = luna_gpio_drive(laser_tx_dis_gpio, false, "TX_DISABLE release");
	if (ret)
		goto fail;
	WRITE_ONCE(luna_activation_ready, true);
	gpon_wr_us_protected(GPON_GTC_US_CFG,
		GPON_US_CFG_VAL | (force_laser ? BIT(15) : 0));
	gpon_wr(GPON_GTC_US_PLOAM_CFG,
		GPON_US_PLM_CRC_GEN_EN | GPON_US_PLM_ONUID_OVRD);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_UNLOCK);
	gpon_field(GPON_GTC_US_PROC_MODE, 0, 0, 1);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_LOCK);
	memset(nomsg, 0xaa, sizeof(nomsg));
	nomsg[0] = 0xff;
	nomsg[1] = 0x04;
	gpon_send_cpu_ploam(PLM_US_QUEUE_NOMSG, nomsg);
	if (datapath_rearm)
		gpon_us_feed_rearm();
	bosa_cal_error = 0;
	return 0;
fail:
	bosa_cal_error = ret;
	/* A release failure can leave the pin at an uncertain level. Re-inhibit,
	 * preserving the original error and reporting an independent inhibit error. */
	if (luna_quiesce_locked())
		pr_err("luna-gpon: optical activation failed and TX inhibit readback failed\n");
	return ret;
}

static int __init rtl9602c_gpon_init(void)
{
	u32 ver, rst, test;
	const char *missing;
	int ret;

	/*
	 * The op table is checked BEFORE any hardware is touched. C will not refuse
	 * a designated initialiser that omits a member -- it leaves it NULL -- so
	 * the core declares which callbacks are MANDATORY and answers with the name
	 * of the first one missing.
	 * It is here and not at the gpon_ploam_init() call below because by then
	 * this function has released the laser's TX_DISABLE: a module that then
	 * failed to load would leave a LASER ENABLED on a shared PON with no FSM
	 * behind it, the one failure on this bench that harms other equipment.
	 */
	missing = gpon_ploam_ops_missing(&luna_ploam_ops);
	if (missing) {
		pr_err("luna-gpon: realtek-luna's struct gpon_ploam_ops leaves the MANDATORY op ->%s NULL -- refusing to probe (nothing has been touched)\n",
		       missing);
		return -EINVAL;
	}

	gpon_base = ioremap(GPON_PHYS_BASE, GPON_REG_SIZE);
	if (!gpon_base) {
		pr_err("luna-gpon: ioremap 0x%08x failed\n", GPON_PHYS_BASE);
		return -ENOMEM;
	}
	swcore_base = ioremap(SWCORE_PHYS_BASE, SWCORE_REG_SIZE);
	if (!swcore_base) {
		pr_err("luna-gpon: ioremap 0x%08x failed\n", SWCORE_PHYS_BASE);
		iounmap(gpon_base);
		return -ENOMEM;
	}

	/*
	 * Detect the chip from the DT root compatible. The RTL9607C uses the c7
	 * rev-C SerDes path and an internal optical front-end, so the BOSA I2C
	 * bring-up and the 9602C-only golden analog/GPIO/LED steps are gated off.
	 */
	is_9607c = of_machine_is_compatible("realtek,rtl9607c");
	/* The THIRD chip. Its table was already in the tree, named by nothing outside
	 * luna_ponmac.{c,h}: the missing element was never missing from the source,
	 * it was unwired. Without this line the G24W fell to the ELSE branch and ran
	 * the RTL9602C recipe, whose SerDes offsets are 0x1E000 LOWER -- and on this
	 * silicon 0x22000..0x220c4 is EXTG_ACTYPE0..7, the switch's extra-Ethertype
	 * match table, so the bring-up wrote into a LIVE table on the working LAN
	 * path and the GTC never left reset. The symptom, onu_state O15 with
	 * rst_done=0, is a raw 4-bit field reading 0xF; stock's vocabulary is O1..O7
	 * and has no O15 at all. */
	is_9603cvd = of_machine_is_compatible("realtek,rtl9603cvd");

	/* Select this chip's SWCORE offsets BEFORE the first sw_rd/sw_wr of any
	 * chip-selected register (the BOSA I2C pad-mux below is the first). */
	swc = is_9607c ? &gpon_swc_9607c
	    : is_9603cvd ? &gpon_swc_9603cvd : &gpon_swc_9602c;
	gtune = is_9603cvd ? &luna_gtc_tune_9603cvd : &luna_gtc_tune_9602c;
	luna_laser_gpio_from_dt();
	ret = luna_gpio_drive(laser_tx_dis_gpio, true, "TX_DISABLE inhibit");
	if (ret)
		goto fail_maps;
	pr_info("luna-gpon: SWCORE map = %s (io_mode_en=0x%05x i2c_en_bus0=%u oem_en=%u gpio_en=0x%05x fib_status=0x%05x sds_reg0=0x%05x)\n",
		swc->chip, swc->io_mode_en, swc->io_i2c_en_bus0, swc->io_oem_en,
		swc->io_gpio_en, swc->sds_fib_status, swc->sds_reg0);

	/*
	 * IO_MODE_EN.OEM_EN routes the optical front-end pads -- TX_DISABLE, the
	 * optical TX signal-detect and the BOSA RX signal-detect -- to the GPON
	 * block. It was DEFINED here and written on no path, so the driver depended
	 * on whatever the bootloader left.
	 *
	 * Skipped on the 9607C (internal front-end, no external pads) and on the
	 * RTL9603CVD since 2026-08-27, because stock on the G24W at O5 reads
	 * IO_MODE_EN = 0x000008c0 -- OEM_EN CLEAR, I2C_EN_BUS0 SET -- in the same
	 * run that read SDS_FIB_STATUS with SDS_SDET SET. So that board's BOSA
	 * signal-detect reaches the GPON block with the bit at zero, and the
	 * premise of the write is refuted. The 0x0000b0c2 that motivated it was
	 * read on OUR image, against nothing.
	 */
	if (!is_9607c && !is_9603cvd) {
		u32 before = sw_rd(SOC_IO_MODE_EN);

		sw_field(SOC_IO_MODE_EN, swc->io_oem_en, swc->io_oem_en, 1);
		pr_info("luna-gpon: OEM_EN (optical pads) bit%u: io_mode_en 0x%08x -> 0x%08x\n",
			swc->io_oem_en, before, sw_rd(SOC_IO_MODE_EN));
	}

	if (is_9607c) {
		skip_bosa = true;
		/* Enable the switch-internal SMI master so the PHY-10 proxy can reach
		 * the I2C-indirect hole (0xB0-0xD8) for the optical-module DDM read. */
		iowrite32(ioread32(swcore_base + SW_IO_MODE_EN_9607C) | SW_MDX_M_EN,
			  swcore_base + SW_IO_MODE_EN_9607C);
		(void)ioread32(swcore_base + SW_IO_MODE_EN_9607C);
		pr_info("luna-gpon: RTL9607C detected — rev-C SerDes, internal front-end (skip BOSA), SMI proxy on\n");
	}

	/* Power up the PON packet-datapath IP domain (see SOC_IP_ENABLE_PHYS). */
	{
		void __iomem *ipen = ioremap(SOC_IP_ENABLE_PHYS, 4);

		if (ipen) {
			u32 mask = SOC_IP_EN_PON;

			/* The 9603CVD sets bit 25 unconditionally, not just the 9607C: this
			 * board's own stock kernel (tier 2) ORs 0x20 then 0x02000000 into
			 * 0xb800063c with NO chip-rev conditional, and its dying-gasp path
			 * keeps bits 5 and 25 alive at power loss so the PON can still
			 * transmit. We gated bit 25 behind is_9607c, so this die never got it. */
			if (is_9607c || is_9603cvd)
				mask |= SOC_IP_EN_PONPBO;	/* PON-IP window / packet datapath */
			writel(readl(ipen) | mask, ipen);
			(void)readl(ipen);		/* post the write */
			iounmap(ipen);
		}
	}

	/* PON-IP datapath window — only reachable now the IP-enable bit is set. */
	ponip_base = ioremap(PONIP_PHYS_BASE, PONIP_REG_SIZE);
	if (!ponip_base) {
		pr_err("luna-gpon: ioremap 0x%08x failed\n", PONIP_PHYS_BASE);
		iounmap(swcore_base);
		iounmap(gpon_base);
		return -ENOMEM;
	}

	ver  = gpon_rd(GPON_VERSION) & GPON_VER_ID_MASK;
	rst  = gpon_rd(GPON_RESET);
	test = gpon_rd(GPON_TEST);

	pr_info("luna-gpon: MAC @0x%08x ver=0x%02x reset=0x%08x test=0x%08x\n",
		GPON_PHYS_BASE, ver, rst, test);

	/* Self-test the register window with the GPON_TEST scratch register, then
	 * restore the power-on value. */
	gpon_wr(GPON_TEST, 0xa5a5a5a5u);
	if (gpon_rd(GPON_TEST) == 0xa5a5a5a5u)
		pr_info("luna-gpon: register R/W OK (scratch verified)\n");
	else
		pr_warn("luna-gpon: scratch R/W failed — MAC may be gated\n");
	gpon_wr(GPON_TEST, GPON_TEST_SCRATCH);

	/*
	 * The GPIO pad routing is Board C's, and its register MOVED. These two
	 * words and the GPIO-controller golden values are the RTL9602C optical-SD
	 * pad recipe, so a chip must opt IN rather than be excluded one at a time:
	 *  - IO_GPIO_EN is 0x48 on the 9602C, 0x3c on the 9603CVD and 0x38 on the
	 *    9607C. On the 9603CVD our 0x48/0x4c writes landed on CFG_PCSXF and
	 *    CFG_PHY_CTRL: measured 2026-08-26, "GPIO pads set (gpio_en1=0x19)" at
	 *    t=0.75 s and the Ethernet driver repairing BASE_PHYAD 25 -> 0 at
	 *    t=7.02 s -- 0x819 & 0x1f == 25, so THIS driver wrote it, not U-Boot.
	 *  - the DATA is per board even where the register agrees.
	 * The function-enable bits live in the switch-core IO_GPIO_EN words; the
	 * direction and data live in the SoC GPIO controller at 0x18003300.
	 */
	if (!is_9607c && !is_9603cvd) {
		sw_wr(SOC_IO_GPIO_EN, SOC_IO_GPIO_EN_W0);
		sw_wr(SOC_IO_GPIO_EN + 4, SOC_IO_GPIO_EN_W1);
		{
			void __iomem *gpio = ioremap(GPIO_PHYS_BASE, GPIO_REG_SIZE);

			if (gpio) {
				iowrite32(GPIO_GOLD_DIR_ABCD,  gpio + GPIO_DIR_ABCD);
				iowrite32(GPIO_GOLD_DATA_ABCD, gpio + GPIO_DATA_ABCD);
				iowrite32(GPIO_GOLD_DIR_EFGH,  gpio + GPIO_DIR_EFGH);
				iowrite32(GPIO_GOLD_DATA_EFGH, gpio + GPIO_DATA_EFGH);
				(void)ioread32(gpio + GPIO_DIR_ABCD);	/* post writes */
				iounmap(gpio);
			}
		}
		pr_info("luna-gpon: GPIO pads set (gpio_en0=0x%08x gpio_en1=0x%08x)\n",
			sw_rd(SOC_IO_GPIO_EN), sw_rd(SOC_IO_GPIO_EN + 4));
	} else if (is_9603cvd && gpio_pad_9603cvd) {
		/*
		 * The 9603CVD's OWN pad recipe, from its OWN board: MEASURED 2026-09-01
		 * by reading the G24W's stock firmware while it held O5 on this fibre
		 * (read-only), diffed against ours over 24 words:
		 *   0x18003308   stock 0x00008014   ours 0x00008000
		 *   0x18003330   stock 0x0000c000   ours 0x00000000
		 * Two more words differ and are NOT written here: both REFUSE a write
		 * from us, which is what a GPIO data register does when its pins are
		 * inputs, so their difference is a consequence of stock's routing.
		 * OFF by default: applying them live, after the SerDes was already up,
		 * changed nothing, so whether they matter at BRING-UP is the open
		 * question and a parameter answers it on one image instead of two builds.
		 */
		void __iomem *gpio = ioremap(GPIO_PHYS_BASE, GPIO_REG_SIZE);

		sw_wr(SOC_IO_GPIO_EN, 0x00008014u);	/* 0x3c on this chip */
		if (gpio) {
			iowrite32(0x00008014u, gpio + 0x08);
			iowrite32(0x0000c000u, gpio + 0x30);
			(void)ioread32(gpio + 0x08);	/* post the writes */
			iounmap(gpio);
		}
		pr_info("luna-gpon: GPIO optical-SD pad recipe APPLIED (%s, from its own stock in O5): io_gpio_en=0x%08x\n",
			swc->chip, sw_rd(SOC_IO_GPIO_EN));
	} else {
		pr_info("luna-gpon: GPIO optical-SD pad recipe skipped (%s: %s)\n",
			swc->chip,
			is_9603cvd ? "recipe exists but gpio_pad_9603cvd=0"
				   : "not Board C's pinout");
	}

	/* The panel-LED block is Board C's and is gated off on the 9603CVD at both
	 * funnels (gpon_led_init + gpon_led_force): beyond LED_FORCE_VALUE landing in
	 * LED_ACTIVE_LOW_CFG, the LED_IO_EN literal 0x23014 is IO_MODE_EN on that die,
	 * and enabling Board C's four LED indices stole the HS_UART_FC / I2C1 /
	 * SLIC_ISI / DYING pin functions (stock 0x08c0 vs ours 0xb8c2, and the XOR is
	 * exactly those four indices). No LED write reaches that chip until its own
	 * LED map is RE'd. */
	if (!is_9607c)
		gpon_led_init();

	/*
	 * Bring up the PON SerDes so the MAC core gets its clock, then confirm the
	 * MAC completed reset. Neither failing is fatal -- the register window stays
	 * usable and /proc/gpon reports the live state.
	 */
	{
		const char *via;
		int sret;

		/* The SECOND writer of the GPIO pad array: this function writes the
		 * 9602C's SC-indirect engine at 0x3c/0x40/0x44, and on the RTL9603CVD
		 * those three words ARE IO_GPIO_EN, so it stamps SC command words into
		 * the pad mux ~60 ms after we announced we were skipping it. Its 0x130
		 * is not THERMAL_CTRL_0 here either. */
		if (!is_9607c && !is_9603cvd)
			rtl9602c_sc_ldo_init();	/* 9602C SC-indirect LDO/thermal — different registers on 9603CVD/9607C */
		else if (is_9603cvd)
			pr_info("luna-gpon: sc_ldo_init skipped (%s: 0x3c/0x40/0x44 are IO_GPIO_EN here, not SC_IND_*)\n",
				swc->chip);
		luna_c2_postmode_perturb = serdes_postmode_perturb;	/* A/B: skip the post-GPON-mode US-TX perturbations (default = skip, stock rev-A) */
		luna_c2_sds_cfgrst = serdes_sds_cfgrst;	/* A/B: SDS reset = stock bit0-only (default) vs legacy bit7+bit0 */
		luna_c2_stock_analog = serdes_stock_analog;	/* A/B: match live-stock SDS REG01/REG11 post-reset (default) */
		luna_c2_analog_postreset = serdes_analog_postreset;	/* A/B: program full analog CMU/CDR table AFTER the SDS reset (stock rev-A, default) = cold-start determinism */
		luna_c2_cmu_settle_ms = serdes_cmu_settle_ms;	/* A/B: TX-CMU-lock settle before reset-B (cold-start metastability candidate) */
		luna_c2_clkgate_rstb = serdes_clkgate_rstb;	/* A/B: clock-gated reset-B release (rank-1 cold-start fix candidate) */
		luna_c2_skip_rstb_dance = serdes_skip_rstb_dance;	/* A/B: skip the gratuitous DIG_1D reset-B pulse (already released) */
		luna_c2_minimal_analog = serdes_minimal_analog;	/* A/B: skip the over-configure golden-table writes (match stock's minimal set) */
		if (force_soc_clk) {
			/* Match the live-stock values of three SoC sysreg words, written with a
			 * FULL-WORD writel() so every bit is stock's. From this unit's OWN stock
			 * binaries (tier 2, 2026-09-05) -- no register NAME for them exists in
			 * anything we may read:
			 *   0x18000100  a STRAP / PIN-STATUS word, not a clock register: stock's
			 *               _is_CKSEL_25MHz reads bit 5 as the clock-select strap.
			 *   0x1800012c  DRAM AUTO-CALIBRATION RESULT words, not clock registers;
			 *   0x18000140  only stage-1's "AK: DRAM AUTO CALIBRATION" routine writes
			 *               them, encoding the fields into both 16-bit halves -- hence
			 *               the 0x024d024d shape. Stock's real clock init is elsewhere. */
			static const struct { u32 off, val; } soc_clk[] = {
				{ 0x18000100u, 0x00440e00u },	/* strap / pin-status word (bit 5 = CKSEL, read by stock _is_CKSEL_25MHz): stock's live value */
				{ 0x1800012cu, 0x024d024du },	/* DRAM auto-calibration result word A (two identical 16-bit halves), stock stage-1 store @0x9fc14b70: stock's value on the captured boot */
				{ 0x18000140u, 0x024d024du },	/* DRAM auto-calibration result word B, stock stage-1 store @0x9fc14b74: same encoding, same boot */
			};
			unsigned int k;
			for (k = 0; k < ARRAY_SIZE(soc_clk); k++) {
				void __iomem *a = ioremap(soc_clk[k].off, 4);
				if (a) {
					writel(soc_clk[k].val, a);
					pr_info("luna-gpon: force_soc_clk [%#x]<=%#x ->%#x\n",
						soc_clk[k].off, soc_clk[k].val, readl(a));
					iounmap(a);
				}
			}
		}
		/* CROSS-SUBSYSTEM ORDER (bosa_before_serdes): stage the external BOSA RX
		 * analog and settle BEFORE the SoC SerDes bring-up, matching stock, so the
		 * CMU locks against a settled front-end. The duplicate bosa_probe() and
		 * bosa_rx_enable() below are skipped when this runs. */
		if (bosa_before_serdes && !skip_bosa) {
			mutex_lock(&bosa_lock);
			bosa_probe();
			if (bosa_regs_live())
				bosa_rx_enable();
			mutex_unlock(&bosa_lock);
			if (bosa_settle_ms)
				mdelay(bosa_settle_ms);
			pr_info("luna-gpon: BOSA RX up + %ums settle BEFORE SerDes (stock order)\n",
				bosa_settle_ms);
		}
		if (family_lib) {
			/* Clean-room family lib. RTL9607C = c7 rev-C ModeV3 SerDes; RTL9602C
			 * = rev-A. Same ops for both (same SWCORE base; the lib never touches
			 * the 9607C I2C-indirect decode hole). */
			if (is_9607c)
				sret = luna_ponmac_mode_set(LUNA_CHIP_9607C, LUNA_REV_C,
							       LUNA_SUBTYPE_NONE,
							       &rtl9602c_r960_ops);
			else if (is_9603cvd)
				/* rev/subtype are ignored by this chip's path -- one SerDes
				 * variant for every rev (luna_ponmac.c:786). */
				sret = luna_ponmac_mode_set(LUNA_CHIP_9603CVD, LUNA_REV_A,
							       LUNA_SUBTYPE_NONE,
							       &rtl9602c_r960_ops);
			else
				sret = luna_ponmac_mode_set(LUNA_CHIP_9602C, LUNA_REV_A,
							       LUNA_SUBTYPE_NONE,
							       &rtl9602c_r960_ops);
			via = is_9607c ? "family-lib 9607C"
			    : is_9603cvd ? "family-lib 9603CVD" : "family-lib 9602C";
			/* Stability fallback if the lib path ever fails to bring the analog
			 * ready. It MUST NOT fire on the 9603CVD: the inline bring-up IS the
			 * 9602C recipe, so "falling back" would resume writing 0x1E000 low,
			 * into EXTG_ACTYPE on a working LAN path. */
			if (sret && !is_9607c && !is_9603cvd) {
				pr_warn("luna-gpon: family-lib SerDes not ready (0x%08x) -> inline fallback\n",
					sw_rd(FIB_EXT_REG21));
				sret = gpon_serdes_init();
				via = "inline fallback";
			}
		} else if (is_9603cvd) {
			/* Same reason, and the guard was missing on this path:
			 * gpon_serdes_init{,_stock}() ARE the 9602C recipe, and their golden
			 * table writes ~0x226xx, which on the 9603CVD is inside the switch's
			 * EXTG_ACTYPE match table. Refuse instead of corrupting a live LAN. */
			sret = -ENOTSUPP;
			via = "REFUSED (family_lib=0 has no 9603CVD SerDes recipe)";
			pr_err("luna-gpon: family_lib=0 is not available on %s -- the inline SerDes bring-up is the RTL9602C register recipe\n",
			       swc->chip);
		} else {
			sret = serdes_stock_seq ? gpon_serdes_init_stock() : gpon_serdes_init();
			via = serdes_stock_seq ? "stock rev-A order" : "GPON mode";
		}

		if (sret)
			pr_warn("luna-gpon: SerDes analog-ready not seen (%s, FIB_EXT_REG21=0x%08x)\n",
				via, sw_rd(FIB_EXT_REG21));
		else
			pr_info("luna-gpon: PON SerDes up (%s, analog ready)\n", via);
		/* Stock powers the laser driver as the LAST step of its PON-MAC
		 * mode set, i.e. right here, once the SerDes is up. */
		ret = luna_gpio_drive(laser_tx_pwr_gpio, false, "laser TX power");
		if (ret)
			goto fail_maps;

		/*
		 * M3 DIAGNOSTIC (temporary): does forcing FIB_REG16 FRC_SD/SEL_RX_SD make
		 * SDS_SDET assert? Splits SoC-side SD gating from no light at the RX pins.
		 */
		if (is_9607c) {
			const struct luna_ops *o = &rtl9602c_r960_ops;
			u32 s0 = o->rd(0x1b00028cu);
			u32 r0 = o->rd(0x1b040c40u);
			u32 s1, s2;

			o->wr(0x1b040c40u, r0 | (1u << 10));		/* FRC_SD */
			mdelay(5);
			s1 = o->rd(0x1b00028cu);
			o->wr(0x1b040c40u, r0 | (1u << 10) | (1u << 2));	/* + SEL_RX_SD */
			mdelay(5);
			s2 = o->rd(0x1b00028cu);
			o->wr(0x1b040c40u, r0);				/* restore */
			pr_info("luna-gpon: M3-SDprobe base_sts=0x%08x reg16=0x%08x | +frc_sd=0x%08x | +sel_rx_sd=0x%08x (SDS_SDET=bit17)\n",
				s0, r0, s1, s2);
			mutex_lock(&bosa_lock);
			ddm_probe_9607c();
			i2c_scan_9607c();
			mutex_unlock(&bosa_lock);
		}
	}

	/*
	 * Probe the external RTL8290B BOSA over I2C (read-only chip-ID check): the
	 * optical RX signal-detect comes from this chip, so this validates the I2C
	 * transport before the RX-enable writes.
	 */
	mutex_lock(&bosa_lock);
	if (!bosa_before_serdes)		/* else already probed before the SerDes */
		bosa_probe();

	/*
	 * skip_bosa (warm boot): leave the externally-powered BOSA in whatever state
	 * it is in and run only the SoC side. Ranging online then means the datapath
	 * is correct and the only gap is the BOSA cold-init.
	 */
	if (skip_bosa || !bosa_regs_live()) {
		pr_info("luna-gpon: RTL boot programming skipped; GN awaits per-unit calibration\n");
		goto skip_bosa_init;
	}

	/*
	 * Power on the BOSA optical receiver (clears its RX power-down), which is
	 * what makes the real optical signal-detect assert. Before the MAC reset.
	 */
	if (!bosa_before_serdes)		/* else already RX-enabled before the SerDes */
		bosa_rx_enable();

	/* Measure the MPD dark reference while the laser is still off, for the live
	 * TX-power DDM (bosa_tx_power_cdbm). Forces TX off ~200ms; must precede
	 * bosa_tx_enable and never run at O5. */
	if (!laser_off)
		bosa_vmpd_dark_calibrate();

	/* Power on the BOSA optical transmitter so the ONU can send upstream PLOAM
	 * bursts during activation. The APC offset calibration is deferred until
	 * after the PON-IP datapath and MAC reset: the APC digital block only clocks
	 * once the PON TX clock runs, so calibrating earlier leaves OFFK_DONE dead. */
	if (!laser_off) {
		bosa_tx_enable();
		/* Burst bias/mod override: MOD raises the peak, BIAS stays low to preserve
		 * extinction. Latched via the 0x23d DAC strobe. Skipped if both 0. */
		if (laser_bias || laser_mod) {
			/* Board C's per-board laser calib (rtl8290b.data): bias DAC12=0x32f, mod
			 * DAC12=0xbbd, so 0x238 = 0xdf -- the low nibbles are part of the DAC
			 * value and must not be zeroed. */
			bosa_set_bit(0x23d, 7, 0);
			bosa_set_field(0x236, 0xff, laser_bias ? laser_bias : 0x19);
			bosa_set_field(0x238, 0x0f, 0x0f);	/* IBIAS[3:0] = 0xf */
			bosa_set_bit(0x23d, 7, 1);
			bosa_set_bit(0x23d, 7, 0);
			bosa_set_field(0x237, 0xff, laser_mod ? laser_mod : 0x67);
			bosa_set_field(0x238, 0xf0, 0xd0);	/* IMOD[3:0] = 0xd */
			bosa_set_bit(0x23d, 7, 1);
			mdelay(2);
			pr_info("luna-gpon: laser DAC override bias=0x%02x mod=0x%02x -> readback bias=0x%02x mod=0x%02x R30=0x%02x mpd=%02x/%02x\n",
				laser_bias, laser_mod, bosa_read_reg(0x236) & 0xff,
				bosa_read_reg(0x237) & 0xff, bosa_read_reg(0x31e) & 0xff,
				bosa_read_reg(0x320) & 0xff, bosa_read_reg(0x321) & 0xff);
		}
		/* bosa_laser_maint() (the ~50 ms fault re-ignite) is gated on
		 * bosa_laser_up, which was set ONLY inside bosa_apc_calibrate() -- skipped
		 * when apc_off is true, the default, because full APC deafens the
		 * shared-BOSA DS-RX. Arm it here so the fault service holds the burst lit
		 * through O5 without the DS-deafening APC. */
		/* ★★★ ONLY WHEN THIS PART IS AN RTL8290B (2026-09-12). The flag was
		 * gated on `laser_off` alone -- a module PARAMETER -- so on a board
		 * whose module is NOT an RTL8290B every write above returned -ENODEV
		 * from bosa_write_reg()'s identity gate and the laser was still
		 * marked up. The cost is not cosmetic: gpon_fsm tick :9435 then runs
		 * bosa_laser_maint() every 5 ticks, reading slave 0x55 and rewriting
		 * CONFIG on an unlocked controller of a DIFFERENT part.
		 *
		 * ⚠ WHAT THIS DOES AND DOES NOT PROVE, because the first draft of
		 *   this comment overclaimed and that is the same defect one layer
		 *   up: `bosa_regs_live()` establishes POSITIVE IDENTIFICATION and
		 *   nothing more. An allowed RTL8290B write can still NACK or time
		 *   out, so this flag is NOT evidence that the programming landed and
		 *   must never be read as calibration-ready. Confirming the writes
		 *   took is a separate, unbuilt witness -- today no read-back exists.
		 */
		if (bosa_regs_live())
			bosa_laser_up = 1;
		else
			pr_warn("luna-gpon: laser NOT marked up -- the BOSA register writes were refused (no positive RTL8290B id), so nothing was applied and the ~50ms TX-fault maintenance stays OFF rather than servicing a part it does not fit\n");
	}

skip_bosa_init:
	mutex_unlock(&bosa_lock);
	/* Configure the PON-IP packet datapath so the MAC has somewhere to land
	 * downstream frames before it is reset. */
	gpon_pbo_init();
	pr_info("luna-gpon: PON-IP datapath configured (ctl_us=0x%08x ctl_ds=0x%08x)\n",
		pi_rd(PI_PONIP_CTL_US), pi_rd(PI_PONIP_CTL_DS));

	/* With the SerDes clock present, soft-reset the GPON MAC so its RST_DONE
	 * handshake completes and the GTC banks come out of reset. */
	gpon_wr(GPON_RESET, GPON_SOFT_RST);
	gpon_wr(GPON_RESET, 0);
	if (gpon_wait_rst_done())
		pr_warn("luna-gpon: RST_DONE not seen (reset=0x%08x)\n",
			gpon_rd(GPON_RESET));
	else
		pr_info("luna-gpon: MAC reset done, ONU state O%u\n",
			gpon_rd(GPON_GTC_DS_ONU_ID_STATUS) & GPON_ONU_STATE_MASK);

	/* Optical loss-of-signal monitoring, inverted polarity. The downstream framer
	 * gates on OPTIC_LOS_SIG; until the input is enabled with the correct
	 * polarity that status reads "loss" even with real light, holding the FSM at
	 * O1. A working (O5) unit runs this register at 0x03. */
	gpon_field(GPON_GTC_DS_LOS_CFG_STS, 0, 0, 1);	/* OPTIC_LOS_EN = 1     */
	gpon_field(GPON_GTC_DS_LOS_CFG_STS, 1, 1, 1);	/* OPTIC_LOS_POLAR = 1  */
	pr_info("luna-gpon: optical-LOS monitor enabled (los_cfg=0x%08x)\n",
		gpon_rd(GPON_GTC_DS_LOS_CFG_STS));

	/* Now that the PON-IP/MAC (and thus the SerDes TX clock) are running, run
	 * the laser APC offset calibration so the laser actually biases. */
	mutex_lock(&bosa_lock);
	if (bosa_regs_live() && !skip_bosa && !laser_off && apc_offk)
		rtl8290b_apc_init();		/* B-variant: completes OFFK (DS-safe) */
	else if (bosa_regs_live() && !skip_bosa && !laser_off && !apc_off)
		bosa_apc_calibrate();		/* legacy non-B flow (A/B fallback) */
	mutex_unlock(&bosa_lock);

	proc_create_single("gpon", 0444, NULL, gpon_proc_show);
	proc_create_single("bosadump", 0444, NULL, bosadump_proc_show);
	proc_create_single("pidump", 0444, NULL, pidump_proc_show);
	proc_create_single("swdump", 0444, NULL, swdump_proc_show);

	/*
	 * Upstream burst CONFIG + laser-enable timing. The GTC MAC reset above clears
	 * US_CFG, so it must be reprogrammed here or the burst-enable polarity and
	 * the laser on/off window default wrong and the laser never modulates a burst
	 * the OLT can see. US_CFG 0x0c18 = US_BEN_POLAR, scrambler, PLOAM, auto-DG;
	 * US_LASER 0x2028 = LON_TIME 32 / LOFF_TIME 40. Both behind the US
	 * write-protect gate.
	 */
	gpon_wr_us_protected(GPON_GTC_US_CFG, 0);
	if (force_laser)
		pr_info("luna-gpon: force_laser=1 -> US_CFG.FS_LON set (CW diagnostic)\n");
	/* US GEM-header PTI vector (not write-protected). Reset is 0, so every US GEM
	 * frame would carry PTI=000 even on end-of-fragment, and some OLTs only
	 * accept OMCI with NON_END_FRAG=0 and END_FRAG=1. Stock's vector (0,1,0,1)
	 * is 0x00001010, with FS_GEM_IDLE[31]=0 keeping auto-idle. */
	gpon_wr(GPON_GEM_US_PTI_CFG, force_idle ? 0x80001010u : 0x00001010u);	/* FS_GEM_IDLE(bit31)=force_idle: bisection diag (stock=0) */
	gpon_wr_us_protected(GPON_GTC_US_LASER, gtune->us_laser);	/* per chip: 0x2028 / 0x1820 */

	/* The laser bias is loaded LOW for acquisition (see bosa_apc_calibrate): the
	 * DS-RX killer is the IDLE-emission LEVEL, i.e. a bias sitting above the
	 * lasing threshold during acquisition (0x18 dead, 0x4c dead). The SN burst's
	 * modulation rides on top, and OFFK converges later during ranging/O5. */

	/*
	 * Upstream SN-burst ARMING -- the GTC-level conditions the silicon requires.
	 * The GTC auto-fires the loaded Serial_Number PLOAM into the OLT's broadcast
	 * SN grant ONLY if all of these are armed; without them sn_tx climbs while no
	 * burst is transmitted, and the OLT reports "Received Ploams = 0".
	 * (1) ONU-ID = 0xFF (broadcast) in BOTH the DS and US ONU-ID fields --
	 *     DS_CFG has BWM_FILT_ONUID, so a grant addressed to 0xFF is filtered
	 *     out unless the programmed field matches.
	 * (2) BWM_NO_FLT, the bring-up bypass of that filter (now 0, see below).
	 * (3) US_PLOAM_CFG = CRC_GEN_EN|ONUID_OVRD armed at INIT, not only per-send:
	 *     the auto-SN burst needs a HW PLOAM CRC8 and an ONU-ID-stamped header.
	 * (4) AUTO_PROC_SSTART (US_PROC_MODE bit0, behind the US write-protect): HW
	 *     aligns the small SN burst to the granted StartTime.
	 * (5) DS_PLOAM_CFG broadcast-accept + ONU-ID filter.
	 */
	gpon_field(GPON_GTC_DS_ONU_ID_STATUS, 15, 8, 0xff);	/* DS ONU-ID = broadcast */
	gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);		/* US ONU-ID = broadcast */
	/* DS_CFG's FEC detection threshold, asserted deliberately (2026-09-08): the
	 * bootloader left this board at FEC_DET_THRSH=1 while its own stock rests at
	 * 0, and the vendor sets it from configuration in its init path. Not a BWmap
	 * acceptance gate -- all four of those already matched. */
	gpon_field(0x1014, 3, 1, 0);				/* DS_CFG FEC_DET_THRSH = 0 (stock) */
	gpon_field(0x1014, 11, 11, 0);				/* DS_CFG BWM_NO_FLT = 0 (stock). The
								 * bring-up =1 "accept all grants" left bwm_acpt=0
								 * against stock's 130k+, i.e. it broke the BWMAP
								 * parser rather than relaxing it. */
	gpon_wr(GPON_GTC_US_PLOAM_CFG, 0);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_UNLOCK);
	gpon_field(GPON_GTC_US_PROC_MODE, 0, 0, 0);
	gpon_wr(GPON_GTC_US_WRITE_PROTECT, GPON_US_WP_LOCK);
	gpon_field(GPON_GTC_DS_PLOAM_CFG, 9, 9, 1);		/* DS PLOAM BC_ACCEPT */
	gpon_field(GPON_GTC_DS_PLOAM_CFG, 8, 8, 1);		/* DS PLOAM ONUID_FILTER ON:
		* every Assign_ONU-ID this OLT sends is BROADCAST, so BC_ACC_EN already
		* delivers it. What the filter buys is real -- this bench puts two ONUs on
		* one splitter, and with it off we also accepted the other unit's PLOAMs.
		* Stock reads GPON_GTC_DS_PLOAM_CFG = 0x70b with the bit SET. */
	/* (6) DS_INTR_MASK = 0x070f, the O5 operating value: LOS/LOF/FEC/LOM (b0-3)
	 * plus SN_REQ(b8) / RNG_REQ(b9) / PLM_BUF(b10), which gate the GTC's
	 * upstream serial-number, ranging and PLOAM-buffer event handling. The MAC
	 * reset clears the register, so it must be re-set here. */
	gpon_wr(GPON_GTC_DS_INTR_MASK, 0x070f);

	/*
	 * Upstream burst TIMING. MIN_DELAY1 = 290 bits, MIN_DELAY2 = 50 guard bits
	 * (0x9132, write-protected); gpon_set_eqd then folds MIN_DELAY1 to 37120,
	 * the correct one-frame burst position before the OLT assigns a delay.
	 */
	gpon_wr_us_protected(GPON_GTC_US_MIN_DELAY, 0x9132);
	/* An __init caller of the equalization-delay computation, through the core's
	 * gpon_ploam_set_eqd(). Do NOT satisfy it by copying the arithmetic back in
	 * here -- that fork is what killed gpon_proto.c. */
	gpon_set_eqd(0);			/* pre-ranging EqD = 290*128 = 0x9100 */

	/*
	 * O5 grant-burst optical config -- the "Laser out"/LOAi wall (2026-06-13).
	 * The digital US datapath egressed while the OLT got no valid O5 burst, and
	 * the ranging SN burst worked: the wide-window isolated SN burst tolerates
	 * the reset defaults, the packed back-to-back O5 bursts do not.
	 *
	 * US_OPTIC_SD_TH: because of the laser-driver TX_SD delay, TX_SD can merge
	 * into the next burst, so stock raises TOOLONG and MISM. At reset defaults
	 * the GTC mis-judges its own packed bursts and gates them at the SerDes
	 * burst-gate, which the OLT sees as LOSi/SFi plus LOAi. The values here are
	 * the LIVE stock read at O5, which differs from stock's own INIT value.
	 * The power-save windowing pre-fires and post-holds the laser around each
	 * granted burst so the preamble is not emitted before the driver settles.
	 */
	gpon_wr_us_protected(0x5188, gtune->us_optic_sd_th);	/* US_OPTIC_SD_TH: per chip (9602C live 0x00504bfa, 9603CVD live 0x00a07fff) */
	gpon_field(0x526c, 0, 0, 1);			/* US_PWR_SAV_MODE.PWR_SAV_MODE = 1 (live stock = 1) */
	if (gtune->us_pwr_sav_dg_tx_opt)
		gpon_field(0x526c, 16, 16, 1);		/* DG_TX_OPT: 9603CVD live 0x00010001 */
	gpon_wr(GPON_GEM_US_PWR_SAV_CFG, (0x10u << 16) | 0x100u);	/* GEM_US_PWR_SAV_CFG = 0x00100100 (live stock) */
	gpon_wr(GPON_GEM_US_EOB_MERGE, 0x00000028u);			/* GEM_US_EOB_MERGE = 0x28 (live stock; mine omitted) */

	/*
	 * Default upstream burst overhead (G.984.3): 0xAA preamble run plus the
	 * standard 0xAB,0x59,0x83 delimiter. The OLT's Upstream_Overhead and
	 * Extended_Burst_Length PLOAMs reprogram this before our first SN burst;
	 * this is a sane state for the window in between.
	 */
	/* Through the core's gpon_ploam_apply_boh(). Do NOT re-implement the
	 * arithmetic locally -- that fork is what killed gpon_proto.c. */
	gpon_apply_boh(false);

	/* US-FEED RE-ARM (stock dataPath_reset arm order): pbo_init ran early and the
	 * MAC/GTC bring-up above parked the US-feed FSM. Re-arm it now, the last
	 * datapath step before the FSM ranges. */
	if (datapath_rearm)
		gpon_us_feed_rearm();

	/*
	 * The SDS upstream-TX serializer registers are deliberately NOT forced here.
	 * Setting them at init did not get the ONU online, but transitioning them
	 * wrong->right AFTER the laser is up once did, so the FSM applies them a few
	 * seconds into O3 to reproduce that post-laser-up re-sync.
	 */

	/* Stock releases TX_DISABLE in its device initialise, before activation
	 * -- the MAC is configured, the FSM is about to run. */
	/* TX_DISABLE stays asserted until validated identity and optics are ready. */

	/* Start the PLOAM activation FSM: parse the per-board serial number and
	 * begin draining downstream PLOAM to drive O1 -> O5. */
	gpon_parse_sn(onu_sn);

	/* Bring the core's FSM object up alongside ours; it does not drive yet. AFTER
	 * the serial number is in force, because gpon_ploam_init() takes it and an
	 * object seeded blank would range as a different ONU once switched on. */
	/* The config must OUTLIVE this function: gpon_ploam_init() STORES the pointer
	 * and does not copy the struct. A stack-allocated config would dangle the
	 * moment this block closed, and would read dead stack the first time
	 * core_fsm was flipped on. */
	luna_ploam_cfg_live = luna_ploam_cfg;
	luna_ploam_cfg_live.hold = gpon_hold;
	luna_ploam_cfg_live.cdr_reseat_on_reactivate = cdr_reseat_on_reactivate;
	luna_ploam_cfg_live.o5_rearm_burst_gate = o5_rearm_burst_gate;
	luna_ploam_cfg_live.o3_feed_reset = o3_feed_reset;
	luna_ploam_cfg_live.data_gem_en = data_gem_en;
	luna_ploam_cfg_live.omcc_alt_bind = omcc_alt_bind;
	/* The OMCC Alloc-ID override: the native FSM reads gpon_omcc_alloc directly,
	 * so the core must get the same value or the two FSMs bind the OMCC T-CONT
	 * to different allocs under one module parameter. */
	luna_ploam_cfg_live.omcc_alloc_override = gpon_omcc_alloc;
	/*
	 * The three tick tunables. Leaving them out was a silent asymmetry in the
	 * A/B: they were 0 in the core object while this driver ran with
	 * los_rerange_ticks=30, so the comparison would have measured a driver WITH
	 * fibre-pull recovery against a core WITHOUT it and blamed the core.
	 */
	luna_ploam_cfg_live.trace = trace;
	luna_ploam_cfg_live.los_rerange_ticks = los_rerange_ticks;
	luna_ploam_cfg_live.o5_provision_watchdog_ticks = o5_provision_watchdog_ticks;
	luna_ploam_cfg_live.o5_ploam_keepalive_ticks = o5_ploam_keepalive_ticks;
	/* The fourth, the same omission again: the core read early_dwell_report_ticks
	 * and nothing gave it a value, so the O1 diagnostic was compiled in and MUTE.
	 * ONU-test-case/dead_knob_guard.py catches this shape. */
	luna_ploam_cfg_live.early_dwell_report_ticks = early_dwell_report_ticks;
	/* The core hands back the name of any mandatory op still missing and REFUSES a
	 * half-wired table (GPON_MUST_CHECK). The check at the top of this function
	 * has already proved it complete, so this makes a FUTURE divergence a
	 * refusal instead of a warning. */
	missing = gpon_ploam_init(&luna_ploam, &luna_ploam_ops,
				  &luna_ploam_cfg_live, NULL, gpon_sn_bytes);
	if (missing) {
		pr_err("luna-gpon: gpon_ploam_init refused: mandatory op ->%s is NULL\n",
		       missing);
		return -EINVAL;
	}
	if (gpon_sn_is_set(gpon_sn_bytes))
		pr_info("luna-gpon: PLOAM FSM start, SN '%s' = %*phN\n",
			onu_sn, 8, gpon_sn_bytes);
	else
		pr_info("luna-gpon: PLOAM FSM start with NO serial provisioned -- parked at O1 until onu_sn is written\n");
	INIT_WORK(&gpon_cdr_reset_work, gpon_cdr_reset_worker);
	INIT_DELAYED_WORK(&gpon_optical_work, gpon_optical_work_fn);

	timer_setup(&gpon_fsm_timer, gpon_fsm_poll, 0);
	mutex_lock(&bosa_lock);
	luna_driver_ready = true;
	bosa_cal_error = luna_activate_locked(false);
	if (bosa_cal_error)
		pr_info("luna-gpon: upstream inhibited, activation pending: %d\n", bosa_cal_error);
	luna_resume_poll();
	mutex_unlock(&bosa_lock);
	schedule_delayed_work(&gpon_optical_work, msecs_to_jiffies(50));
	return 0;
fail_maps:
	if (ponip_base) {
		iounmap(ponip_base);
		ponip_base = NULL;
	}
	iounmap(swcore_base);
	swcore_base = NULL;
	iounmap(gpon_base);
	gpon_base = NULL;
	return ret;
}

static void __exit rtl9602c_gpon_exit(void)
{
	mutex_lock(&bosa_lock);
	luna_stopping = true;
	if (luna_quiesce_locked())
		pr_err("luna-gpon: TX inhibit failed during unload\n");
	luna_driver_ready = false;
	/* Close the UNI enqueue gate before anything is cancelled, so a
	 * producer that is already running cannot re-arm behind the cancel. */
	luna_uni_apply_close();
	mutex_unlock(&bosa_lock);
	timer_delete_sync(&gpon_fsm_timer);
	cancel_work_sync(&gpon_cdr_reset_work);
	cancel_delayed_work_sync(&gpon_optical_work);
	/* OUTSIDE bosa_lock, and after the producers: the worker holds that
	 * mutex for its whole apply, so cancelling under it would deadlock. */
	cancel_delayed_work_sync(&luna_uni_apply_work);
	remove_proc_entry("gpon", NULL);
	remove_proc_entry("bosadump", NULL);
	remove_proc_entry("pidump", NULL);
	remove_proc_entry("swdump", NULL);
	if (ponip_base)
		iounmap(ponip_base);
	if (swcore_base)
		iounmap(swcore_base);
	if (gpon_base)
		iounmap(gpon_base);
}

module_init(rtl9602c_gpon_init);
module_exit(rtl9602c_gpon_exit);

MODULE_DESCRIPTION("Realtek RTL9602C GPON MAC foundation driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("rtkbosa_k.bin");
