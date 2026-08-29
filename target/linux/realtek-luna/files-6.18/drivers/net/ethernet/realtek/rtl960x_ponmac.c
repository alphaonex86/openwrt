// SPDX-License-Identifier: GPL-2.0
/*
 * TIER: FAMILY (prefix rtl960x_) — hardware shared by one silicon
 * family.  Registers and bring-up sequences belong here; GPON PROTOCOL
 * logic does NOT — that is the core tier (drivers/net/gpon).
 * Role: Luna PON-MAC and SerDes bring-up (9602C / 9603CVD / 9607C).
 *
 * SCOPE, and the name is wider than it: the prefix rtl960x_ names Realtek's
 * whole RTL960x part-number series, but this file serves only the LUNA MIPS
 * half of it.  The RTL9607F carries a number from the same series and is a
 * Cortina Access NE core with a different register map, driven by
 * target/linux/realtek-elnath.  It used to appear here as an enum member
 * returning -ENOTSUPP, which made the coverage look complete.  It does not
 * any more; the honest reading of this file is "Luna, three chips".
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon-common/files-6.18/drivers/net/gpon/gpon_common.h.
 */
/*
 * rtl960x_ponmac.c - clean-room RTL960x family GPON PON-MAC / SerDes bring-up.
 *
 * This is an ORIGINAL, data-driven reimplementation. The per-chip register
 * SEQUENCES (which registers, what values, in what order, with what delays) are
 * hardware-interface FACTS dictated by the silicon - extracted by observing the
 * bring-up - not copied code. They are expressed here as compact declarative
 * op-tables driven by a single tiny interpreter, rather than as repetitive
 * per-register procedural boilerplate. The structure, interpreter,
 * naming, and organization are all original; only the factual register data is
 * shared with any other implementation of the same hardware.
 *
 * Design (deliberately "better code"):
 *   - one r960_op{} table per (chip, phase) = the bring-up as data
 *   - r960_run() interprets WR / FLD(RMW) / DELAY / POLL ops
 *   - loops/conditionals (scheduler & queue init, rev/subtype branches) stay as
 *     small explicit code - tables are only for straight-line register runs
 *   - absolute physical addresses throughout; the board injects rd/wr (ops)
 *
 * Tested: the 9602C path on the X111W and the 9603CVD path on the G24W.  The
 * 9607C path is register-faithful but UNTESTED (no board on the bench).
 *
 * REMOVED 2026-08-29, and the removal is the point: this file also carried a
 * complete RTL9601B bring-up and a complete EPON mode-set for all four chips
 * -- 897 lines, every one of them UNTESTED BY ITS OWN ADMISSION ("no EPON
 * hardware available").  Nothing referenced RTL960X_CHIP_9601B or
 * RTL960X_MODE_EPON outside this file; every live caller passed
 * RTL960X_MODE_GPON, so the mode parameter has gone with them.  We ship a GPON
 * product; code for a protocol we do not ship, on silicon we do not have, is
 * not a spare part -- it is 897 lines a reader must first prove irrelevant.
 *
 * ===================================================================== *
 * WHERE THE GPON PROTOCOL LAYER IS - AND WHY IT IS NOT IN THIS FILE
 * (navigation note, 2026-08-05 common-layer refactor. No code moved out of
 *  this file; this block exists so the next reader does not go looking.)
 *
 * If you arrived here from the operator's brief - "rtl960x* para la familia
 * para tener codigo comun" - this IS that file, but for the HARDWARE tier
 * only, and it is already doing the job: one object serves two chip drivers
 * (the Makefile links rtl960x_ponmac.o under BOTH CONFIG_RTL9602C_GPON and
 * CONFIG_RTL9607C_GPON) and carries four chips' tables behind one
 * enum rtl960x_chip dispatch. There is nothing to de-duplicate here.
 *
 * The 2026-08-05 refactor added a SECOND, HIGHER contract - the HW-decoupled
 * GPON protocol core and its op table:
 *     target/linux/gpon-common/files-6.18/drivers/net/gpon/gpon_common.h
 *     struct gpon_shell_ops   (14 ops: ploam_tx, set_hw_state, set_hw_onu_id,
 *                              set_eqd, apply_boh, analog_relock,
 *                              aes_stage_key, aes_set_switch_time, rng,
 *                              omcc_install, data_install, data_teardown,
 *                              omci_tx, trace)
 *
 * NONE of those 14 ops is implemented here, and none can be. Measured
 * 2026-08-05 over this file: ploam 0, gem 0, alloc 0, onu_id 0, eqd 0, aes 0,
 * boh 0 occurrences. The 34 "OMCI" and 28 "T-CONT" hits are REGISTER NAMES
 * and EGRESS-SCHEDULER SLOTS - the trap priority, and which physical queue
 * the OMCC channel is steered to (C7_OMCI_FLOW / C7_OMCI_TCONT /
 * C7_OMCI_QUEUE). This file never sees a PLOAM or OMCI message, never learns
 * an ONU-ID, an alloc-id or a GEM port-id, and holds no FSM. It configures
 * the pipe; it never reads what flows through it.
 *
 * Luna's implementation of gpon_shell_ops lives where those ops actually are,
 * which is gpon-rtl960x.c, plus rtl9602c_eth.c for the OMCI transmit. That
 * is the file the Luna op-table instance belongs in - NOT this one. The
 * implementing functions, each verified present at the line given
 * (2026-08-05):
 *     ploam_tx        :5041  gpon_send_cpu_ploam()
 *     aes_stage_key   :5180  gpon_aes_stage_key()
 *     omcc_install    :5420  gpon_install_omcc()
 *     data_install    :5609  gpon_install_data_gem()
 *     apply_boh       :5947  gpon_apply_boh()
 *     set_eqd         :6017  gpon_set_eqd()
 *     analog_relock   :6053  gpon_txpll_relock()
 *     set_hw_state    :6068  gpon_fsm_set_state()
 * Two entries are NOT stand-alone functions, and are listed apart so nobody
 * goes looking for one:
 *     :5765  gpon_install_tcont() is a helper BOTH omcc_install and
 *            data_install call to bind their T-CONT; it is not an op itself.
 *     :6499  aes_set_switch_time has no function - it is an inline
 *            gpon_wr(0x3014, fc) (AES_KEY_SWITCH_TIME[29:0]) inside the
 *            KEY_SW PLOAM handler, guarded by gpon_key_staged. Whoever wires
 *            that op has to lift it out first; the guard must come with it.
 *     data_teardown has no Luna implementation at all - the op is NULL here
 *            (gpon_common.h says so; luna's stale-CAM story is a re-arm flag).
 *
 * TIER RELATIONSHIP, stated once so it is not re-derived:
 *     gpon_*        protocol core - decides, no MMIO, runs on x86 too
 *     gpon-rtl960x.c / cortina-gpon.c   the two SHELLS - they implement
 *                                        gpon_shell_ops and do the I/O
 *     rtl960x_ponmac.c (this file) / the Cortina NE bring-up
 *                                        a tier BELOW both shells: silicon
 *                                        bring-up the shell calls at probe
 * So this file is a PEER of the Cortina bring-up, never a base class for it,
 * and nothing is promoted out of it. The two silicons share no register.
 *
 * ! DO NOT ADD a struct gpon_shell_ops instance to this file. It could only
 *   be a table of pointers into gpon-rtl960x.c's statics, which needs either
 *   12 symbols un-static'd or a runtime registration - a redesign, not code
 *   motion - and it would have no caller here. A shared-looking file with no
 *   consumer is exactly what gpon_proto.c became (dead since 2026-06-18,
 *   deleted by the refactor); do not build a second one.
 * ! DO NOT merge struct rtl960x_ops (below) into gpon_shell_ops. It is a raw
 *   REGISTER ACCESSOR {rd, wr} injected so this file needs no struct device.
 *   It is a different contract at a different tier that happens to share the
 *   word "ops".
 *
 * Two things found while verifying the above. Recorded, deliberately NOT
 * fixed - this pass is code motion, and both are pre-existing:
 *   N1. rtl960x_ponmac_serdes_cdr_reset() (the exported dispatcher at the
 *       bottom of this file) has ZERO callers tree-wide. The live CDR reset
 *       is gpon-rtl960x.c:3153's own inline pulse under its serdes_cdr_reset
 *       module param. Kept as-is: it is the family API for the boards not on
 *       the bench, the same status as the untested 9607C tables.
 *   N2. That CDR reset is NOT the analog_relock op, despite the similar name.
 *       Different registers, different purpose: CDR reset pulses
 *       SDS_ANA_COM_REG12 bit15, while analog_relock is
 *       gpon-rtl960x.c:6053 gpon_txpll_relock(), which toggles
 *       SDS_ANA_COM_REG27 bit10 (CMU enable 1->0->1) and re-syncs the SerDes
 *       word-FIFO pointer via WSDS_DIG_1D bit14. Wiring analog_relock to the
 *       CDR reset would silently replace the cold-start TX-CMU relock this
 *       board's ranging depends on.
 * ===================================================================== *
 */

#include "rtl960x_ponmac.h"
#include "rtl960x_ponmac_logic.h"	/* hoisted logic */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/bits.h>

/* ---- op-table format (original) --------------------------------------- */
/*
 * ★ THE OPCODES, THE STEP AND THE INTERPRETER ARE THE CORE'S NOW
 * (drivers/net/gpon/gpon_regseq.h).  What stays here is what is a SILICON fact:
 * the tables below -- which registers, what values, in what order, with what
 * delays -- and the accessor that reaches them.
 *
 * The old names are kept as aliases so that not one of the ~580 table lines had
 * to be retyped: the enum order and the struct layout were already identical,
 * which is what made the extraction a move rather than a rewrite.
 */
#include "gpon_regseq.h"

#define r960_op		gpon_regseq_op
#define R960_WR		GPON_REGSEQ_WR
#define R960_FLD	GPON_REGSEQ_FLD
#define R960_DLY	GPON_REGSEQ_DLY
#define R960_POLL	GPON_REGSEQ_POLL

/* The STEP is the core's too (gpon_regseq_op): same five fields, same order --
 * which is why `#define r960_op gpon_regseq_op` above is an alias and not a
 * second declaration.  Leaving this struct behind made it exactly that, and the
 * compiler said so: "redefinition of 'struct gpon_regseq_op'". */

#define WR(a, v)		GPON_WR((a), (v))
#define FLD(a, m, l, v)		GPON_FLD((a), (m), (l), (v))
#define DLY(ms)			GPON_DLY((ms))
#define POLL(a, bit, iters)	GPON_POLL((a), (bit), (iters))

/* the whole interpreter - one function for the entire family */
/*
 * ★ THE SHELL HALF, and the whole of what stayed behind: this family's SLEEP.
 * The core cannot call mdelay()/udelay() -- a tier that sleeps cannot be run on
 * a host -- so the two delays are handed over as ops, and the interpreter that
 * consumes them is now shared by every target in the tree.
 */
static void r960_delay_ms(unsigned int ms)
{
	mdelay(ms);
}

static void r960_delay_us(unsigned int us)
{
	udelay(us);
}

static int r960_run(const struct rtl960x_ops *o,
		    const struct r960_op *seq, unsigned int n)
{
	const struct gpon_regseq_io io = {
		.rd		= o->rd,
		.wr		= o->wr,
		.delay_ms	= r960_delay_ms,
		.delay_us	= r960_delay_us,
	};

	return gpon_regseq_run(&io, seq, n);
}

/* =======================================================================
 * Per-chip bring-up tables + glue.
 * Populated from the per-chip register FACTS (each chip's register/field map).
 * Each block is self-contained so a board only links
 * what it needs once the dispatch is wired by chip id.
 * ======================================================================= */

/* ---- RTL9602C (rev-A) - HW-tested on realtek-luna ---------------------- */
/* (tables filled from verified facts) */

/* ---- RTL9603CVD -------------------------------------------------------- */
/*
 * GPON PON-MAC + SerDes bring-up as data. The SerDes on this part is reached
 * through plain memory-mapped registers (no indirect command/data page window),
 * so every analog/digital tweak is a direct RMW in the tables below.
 *
 * Absolute physical addresses = SWCORE window base 0x1b000000 + register offset;
 * the PON-IP sub-block lives in the 0x1bf0xxxx window.
 */
#define C3_SWBASE		0x1b000000u

/* core / SerDes digital + analog block */
#define C3_SOFTWARE_RST		0x1b0000e0u /* global soft-reset command word     */
#define C3_SDS_CFG		0x1b000200u /* SerDes lane mode select            */
#define C3_DYNGASP_CTRL		0x1b00021cu /* dying-gasp comparator control      */
#define C3_P_MISC_PON		0x1b020404u /* P_MISC[pon4]: PpReg 0x20004 + 4*MACPP_INTERVAL(0x100) */
#define C3_PON_INBW_LBOUND	0x1b023180u /* DS in-band accumulation low bound  */
#define C3_WSDS_DIG_00		0x1b040030u /* SerDes digital: clock control      */
#define C3_WSDS_DIG_02		0x1b040038u /* SerDes digital: BEN power-down      */
#define C3_SDS_REG7		0x1b04081cu /* [14] SP_CFG_NEG_CLKWR_A2D          */
#define C3_WSDS_DIG_18		0x1b040090u /* SerDes digital: BEN output enable   */
#define C3_WSDS_DIG_1D		0x1b0400a4u /* SerDes digital: interface FIFO rstb */
#define C3_FORCE_BEN		0x1b0400e4u /* burst-enable force mode             */
#define C3_SDS_ANA_MISC02	0x1b040508u /* analog misc: BER-notify force/value */
#define C3_SDS_ANA_COM03	0x1b04058cu /* analog common: RX CDR / SD-por sel  */
#define C3_SDS_ANA_COM09	0x1b0405a4u /* analog common: BEN CML/TTL drive    */
#define C3_SDS_ANA_COM17	0x1b0405c4u /* analog common: CDR loop Kp          */
#define C3_SDS_ANA_COM20	0x1b0405d0u /* analog common: RX CMU charge-pump   */
#define C3_SDS_ANA_COM21	0x1b0405d4u /* analog common: RX CMU slew / KVCO   */
#define C3_SDS_ANA_COM26	0x1b0405e8u /* analog common: GPHY CMU LDO vref    */
#define C3_SDS_ANA_COM27	0x1b0405ecu /* analog common: GPHY CMU KVCO        */
#define C3_FIB_EXT_REG21	0x1b040e54u /* fiber ext: analog-ready status      */
#define C3_PON_TRAP_CFG		0x1b0110ecu /* OMCI/MPCP trap priority            */
/* PON-IP block */
#define C3_PON_SIDVALID		0x1bf0218cu /* per-flow SID-valid bitmap (1b/elem) */
#define C3_PON_BW_THRES		0x1bf021a0u /* upstream BW request thresholds     */
#define C3_PON_OMCI_CFG		0x1bf021a4u /* OMCI flow/SID select               */
#define C3_PON_SCH_CTRL		0x1bf021e4u /* scheduler control                  */
#define C3_PON_SID2QID		0x1bf0210cu /* flow(SID) -> physical queue (7-bit/elem) */

/* fixed chip parameters for the GPON datapath */
#define C3_SID_COUNT		128	/* classifier SID / flow slots          */
#define C3_OMCI_FLOW		127	/* flow id reserved for OMCI            */

/* SID-valid bitmap is packed 1 bit per flow: word = base + (idx/32)*4, bit idx%32 */
static inline void c3_sidvalid(const struct rtl960x_ops *o, u32 idx, u32 v)
{
	u8 b = idx & 31u;

	rtl960x_rfwr(o, C3_PON_SIDVALID + (idx >> 5) * 4u, b, b, v);
}

/* SID2QID: 7-bit physical-queue field per flow, 4 flows per 32-bit word */
static void c3_flow2queue(const struct rtl960x_ops *o, u32 flow, u32 pqid)
{
	u32 lsb = (flow % 4u) * 7u;

	rtl960x_rfwr(o, C3_PON_SID2QID + (flow / 4u) * 4u, lsb + 6u, lsb, pqid);
}

/*
 * ponmac_init: PON-MAC global defaults applied once before mode selection.
 * Single-ended burst-enable variant (TTL output driver on); request/last
 * bandwidth thresholds seeded; PIR overflow drop, OMCI trap priority and the
 * dying-gasp comparator polarity set. Per-T-cont and per-queue scheduler/rate
 * programming is owned by the datapath/scheduler driver, not this table.
 */
static const struct r960_op c3_init[] = {
	FLD(C3_SDS_ANA_COM09,  0,  0, 1),	/* BEN drive: TTL output enabled  */
	FLD(C3_PON_BW_THRES,  29, 16, 5),	/* US last-grant BW threshold     */
	FLD(C3_PON_BW_THRES,  13,  0, 5),	/* US runt BW request threshold   */
	FLD(C3_PON_SCH_CTRL,  18, 18, 1),	/* drop on PIR overflow           */
	FLD(C3_PON_TRAP_CFG,   2,  0, 7),	/* OMCI/MPCP trap = top priority  */
	FLD(C3_DYNGASP_CTRL,   3,  3, 1),	/* invert dying-gasp comparator   */
};

/*
 * GPON SerDes/PON-MAC bring-up, phase 1: analog pre-config with the lane held
 * off. Force the 125 MHz reference on so the analog has a clock, lift the BEN
 * power-down, detach the RX CDR AFE, then load the tuned CDR/CMU/KVCO analog
 * coefficients before switching the lane into GPON mode.
 */
static const struct r960_op c3_sds_pre[] = {
	/* ★ A2D SAMPLING CLOCK EDGE -- PER-CHIP, AND THIS DIE DIFFERS FROM THE 9602C.
	 * SDS_REG7[14] SP_CFG_NEG_CLKWR_A2D selects the clock edge on which the RX
	 * analog-to-digital sampler latches. The RTL9602C wants 0 (c2 path sets it
	 * to 0 explicitly), but this board's OWN stock kernel sets it to 1 in
	 * dal_rtl9603cvd_switch_init (tier 2, disassembled from the G24W's k0.vmlinux
	 * @0x802dcd04: SDS_REG7 field SP_CFG_NEG_CLKWR_A2D = 1) -- BEFORE the ponmac
	 * SerDes bring-up, and the value survives the SDS reset (stock reads
	 * SDS_REG7 = 0x5359, bit14 set, at O5). Our C3 path never set it, so the RX
	 * came up sampling on the wrong edge -> garbage samples -> SDS_SDET never
	 * asserts and the FSM never leaves O1. Set it first, before the reset, so the
	 * RX front end latches the right edge when it comes out of reset.
	 * ⚠ MEASURED 2026-08-27, AND IT IS A CORRECTION, NOT THE FIX: this shipped in
	 * a boot; the board read SDS_REG7 = 0x00005359 (bit14 set, = stock EXACTLY),
	 * and SDS_SDET STILL stayed 0 at O1. So the A2D edge was a real stock-vs-ours
	 * difference and is now closed, but it is NOT why the RX fails. Kept because it
	 * matches this die's own stock kernel; the O1 wall is still open (the cause is
	 * not any kernel SerDes/SoC register -- all now match stock -- so it lies in a
	 * non-SWCORE block (GTC/PON-IP) or the physical RX). */
	FLD(C3_SDS_REG7,      14, 14, 1),	/* A2D clock edge (9603CVD = 1)   */
	FLD(C3_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)        */
	FLD(C3_WSDS_DIG_00,    4,  4, 1),	/* force 125 MHz reference clock   */
	FLD(C3_WSDS_DIG_02,   10, 10, 0),	/* clear BEN power-down            */
	FLD(C3_SDS_ANA_COM03, 13, 13, 0),	/* RX CDR AFE: deselect            */
	FLD(C3_SDS_ANA_COM09,  4,  4, 0),	/* BEN driver: CML off             */
	FLD(C3_SDS_ANA_COM09,  0,  0, 1),	/* BEN driver: TTL output on       */
	FLD(C3_SDS_ANA_COM17, 15, 10, 0xc),	/* CDR loop proportional gain Kp   */
	FLD(C3_SDS_ANA_COM20, 11,  7, 0x1b),	/* RX CMU charge-pump current      */
	FLD(C3_SDS_ANA_COM20,  3,  2, 0x3),	/* RX CMU LDO reference            */
	FLD(C3_SDS_ANA_COM21, 13, 11, 0x2),	/* RX CMU slew rate                */
	FLD(C3_SDS_ANA_COM21,  6,  3, 0x4),	/* RX VCO gain band select         */
	FLD(C3_SDS_ANA_COM26,  3,  2, 0x3),	/* GPHY CMU LDO reference          */
	FLD(C3_SDS_ANA_COM27,  6,  3, 0x4),	/* GPHY VCO gain band select       */
};

/*
 * Phase 2: commit GPON mode and pulse the resets. Select GPON on the lane,
 * release the BER-notify force so the reset takes, reset SerDes (digital +
 * analog) and the GPON MAC, then re-arm BER-notify so a later signal-detect
 * drop will not knock the MAC down. A switch-core reset follows the mode change.
 */
static const struct r960_op c3_sds_mode[] = {
	FLD(C3_SDS_CFG,        4,  0, 0x8),	/* lane mode: GPON                 */
	FLD(C3_SDS_ANA_MISC02,12, 12, 0),	/* release BER-notify force         */
	FLD(C3_SOFTWARE_RST,   2,  0, 1),	/* reset SerDes + GPON MAC          */
	DLY(10),				/* let the reset settle             */
	FLD(C3_SDS_ANA_MISC02,13, 13, 1),	/* BER-notify hold value = 1        */
	FLD(C3_SDS_ANA_MISC02,12, 12, 1),	/* re-force BER-notify (MAC stays up)*/
	FLD(C3_SOFTWARE_RST,  10, 10, 1),	/* switch-core reset on mode change */
	DLY(10),				/* let the switch reset settle      */
};

/*
 * Phase 3: re-enable the datapath after the resets. Cycle the TX then RX
 * interface FIFO release-B, turn the burst-enable output on, allow undersize
 * frames on the PON port, and drop burst-enable force mode.
 */
static const struct r960_op c3_sds_post[] = {
	FLD(C3_WSDS_DIG_1D,   16, 16, 0),	/* TX interface FIFO: assert rstb   */
	FLD(C3_WSDS_DIG_1D,   16, 16, 1),	/* TX interface FIFO: release rstb  */
	FLD(C3_WSDS_DIG_1D,   15, 15, 0),	/* RX interface FIFO: assert rstb   */
	FLD(C3_WSDS_DIG_1D,   15, 15, 1),	/* RX interface FIFO: release rstb  */
	FLD(C3_WSDS_DIG_18,   12, 12, 1),	/* burst-enable output: on          */
	/* ★ THE OPTIC-LOS FORCE MUST BE RELEASED, AND THIS CHIP'S TABLE NEVER DID.
	 * The 9602C table (c2_sds_rx_arm) clears all three of these deliberately
	 * -- "at O5 WSDS_DIG_18 = 0x1000 (no force) and the REAL pad drives LOS".
	 * The 9603CVD table set BEN_OE and stopped, so [15:13] kept whatever reset
	 * or the bootloader left. With CFG_FRC_OPTIC_LOS=1 the GTC stops sampling
	 * the pad and substitutes CFG_FRCV_OPTIC_LOS, so FRC=1/FRCV=1 pins
	 * OPTIC_LOS_SIG at 1 -- a permanent, unfalsifiable "no downstream light"
	 * that no amount of real light can clear, and the PLOAM FSM never leaves
	 * O1. Same class as CFG_PHY_CTRL's BASE_PHYAD: a register we never wrote,
	 * left by the bootloader, that stock clears.
	 * Bit positions are this chip's own (WSDS_DIG_18: OPTIC_LOS_SEL_EPON[15],
	 * CFG_FRC_OPTIC_LOS[14], CFG_FRCV_OPTIC_LOS[13], BEN_OE[12]) -- they happen
	 * to match the 9602C's, but they were re-read here rather than assumed. */
	FLD(C3_WSDS_DIG_18,   15, 15, 0),	/* OPTIC_LOS_SEL_EPON = 0 (GPON)    */
	FLD(C3_WSDS_DIG_18,   14, 14, 0),	/* CFG_FRC_OPTIC_LOS  = 0 (use pad) */
	FLD(C3_WSDS_DIG_18,   13, 13, 0),	/* CFG_FRCV_OPTIC_LOS = 0           */
	/* ★★★ THE "RX ARM" IS REMOVED, AND THE ORACLE IS WHAT REMOVED IT
	 * (2026-08-27).
	 *
	 * This used to write REG_RX_SEL_CDR_AFEN = 1 here (COM03[13]), on the
	 * reasoning that `c3_sds_pre` deselects the RX CDR AFE and nothing put it
	 * back, and that the 9602C's own path sets the identically-named field on
	 * its die. It shipped with its own condition attached: *"WHAT IS NOT
	 * PROVEN: stock's resting value for this bit on THIS board. If a stock
	 * capture later shows 0 here, this line is the first suspect."*
	 *
	 * THE CAPTURE WAS TAKEN. Stock, on THIS board, at O5 [Operation, SERVING],
	 * SWCORE 0x4058c:
	 *
	 *     SDS_ANA_COM03 = 0x00001929   ->   bit 13 = 0
	 *
	 * and in the same capture SDS_FIB_STATUS (0x214) = 0x00020008, i.e. SDS_SDET
	 * (bit 17) SET. So the RX front end raises signal-detect on this silicon
	 * with COM03[13] resting at ZERO, and the premise of the arm -- that an
	 * unselected AFE is why SDS_SDET stays 0 on our image -- is refuted by the
	 * only source that could refute it.
	 *
	 * The board had already said the same thing more weakly: the arm was in the
	 * image booted 2026-08-26 (provable -- the `optic_a2:` line only that build
	 * prints appeared) and sds_sdet stayed 0, link_ok stayed 0, and the FSM
	 * stayed at O1.
	 *
	 * ⇒ the value is left where c3_sds_pre puts it, which is also where stock
	 * leaves it. The DLY(10) that settled the selection goes with it.
	 */
	FLD(C3_P_MISC_PON,     2,  2, 1),	/* PON port: accept undersize       */
	FLD(C3_FORCE_BEN,      0,  0, 0),	/* burst-enable force mode: off     */
};

/*
 * SerDes CDR reseat: toggle the RX signal-detect power-on select bit then
 * restore it, and bounce the 16<->20-bit transfer FIFO release-B. Used to
 * re-acquire lock without a full re-bring-up.
 */
static int c3_cdr_reset(const struct rtl960x_ops *o)
{
	u32 v = o->rd(C3_SDS_ANA_COM03);

	/* flip the SD power-on select bit (mask 0x400), leave the rest intact */
	o->wr(C3_SDS_ANA_COM03, (v & ~0x400u) | (((~v) & 0x400u)));
	mdelay(10);
	o->wr(C3_SDS_ANA_COM03, v);		/* restore original analog word    */

	rtl960x_rfwr(o, C3_WSDS_DIG_1D, 14, 14, 0); /* transfer FIFO: assert rstb  */
	mdelay(10);
	rtl960x_rfwr(o, C3_WSDS_DIG_1D, 14, 14, 1); /* transfer FIFO: release rstb */
	return 0;
}

/* GPON bring-up driver: pre-config tables + flow/OMCI wiring + analog gate. */
static int c3_gpon_mode_set(const struct rtl960x_ops *o)
{
	u32 f;
	int rc;

	/* park every data flow's SID as invalid + map to the default queue; the
	 * OMCI flow is armed afterwards. Without the SID2QID map the OMCI SID is
	 * valid but bound to no queue and no traffic passes. */
	for (f = 0; f < C3_SID_COUNT - 1u; f++) {
		c3_sidvalid(o, f, 0);
		c3_flow2queue(o, f, 0x7f);
	}
	c3_sidvalid(o, C3_OMCI_FLOW, 1);		/* OMCI flow: SID valid    */
	c3_flow2queue(o, C3_OMCI_FLOW, 0x7f);		/* OMCI flow -> queue      */
	rtl960x_rfwr(o, C3_PON_OMCI_CFG, 6, 0, C3_OMCI_FLOW); /* OMCI SID select   */

	rc = r960_run(o, c3_sds_pre,  ARRAY_SIZE(c3_sds_pre));
	if (rc)
		return rc;
	rc = r960_run(o, c3_sds_mode, ARRAY_SIZE(c3_sds_mode));
	if (rc)
		return rc;
	rc = r960_run(o, c3_sds_post, ARRAY_SIZE(c3_sds_post));
	if (rc)
		return rc;

	/*
	 * Wait for the analog to report ready (FIB_EXT_REG21 bit 13), then drop
	 * the forced 125 MHz reference to save power. The reference stays on if
	 * the gate never asserts. ~1000 * 200us upper bound.
	 */
	rc = r960_run(o, (const struct r960_op[]){
		POLL(C3_FIB_EXT_REG21, 13, 1000),
	}, 1);
	if (rc == 0)
		rtl960x_rfwr(o, C3_WSDS_DIG_00, 4, 4, 0); /* 125 MHz reference: off */

	/* DS in-band accumulation low bound for PBO */
	rtl960x_rfwr(o, C3_PON_INBW_LBOUND, 23, 0, 0xfda000);

	return rc;
}

/* RTL9603CVD top-level entry points (thin wrappers over the c3_* internals). */
static int rtl9603cvd_ponmac_init(const struct rtl960x_ops *o)
{
	return r960_run(o, c3_init, ARRAY_SIZE(c3_init));
}

static int rtl9603cvd_ponmac_mode_set(const struct rtl960x_ops *o,
				      int rev, int subtype)
{
	(void)rev; (void)subtype;	/* single SerDes variant for every rev */
	return c3_gpon_mode_set(o);
}

static int rtl9603cvd_serdes_cdr_reset(const struct rtl960x_ops *o)
{
	return c3_cdr_reset(o);
}

/* ---- RTL9607C ---------------------------------------------------------- *
 * GPON PON-MAC + SerDes bring-up as data. SerDes here is DIRECT MMIO: every
 * analog/digital knob is its own memory-mapped register written by RMW (there
 * is no indirect command/data page+register window on this part). The
 * straight-line analog/reset runs live in op-tables; the per-tcont / per-queue
 * scheduler init and the rev-dependent SerDes variant stay as explicit code.
 *
 * Absolute physical addresses = SWCORE window base 0x1b000000 + register offset;
 * the PON-IP sub-block lives in the 0x1bf0xxxx window. This part has a 5-deep
 * PON port and a 0x100 per-port MAC stride (narrower than the 9601b/9602c 0x400).
 */

/* PON-IP config / scheduler block (0x1bf0xxxx) */
#define C7_PON_SIDVALID		0x1bf02188u /* per-flow SID-valid bitmap (1b/elem)  */
#define C7_PON_OMCI_CFG		0x1bf021a0u /* OMCI flow/SID select                 */
#define C7_PON_BW_THRES		0x1bf0219cu /* upstream BW request thresholds       */
#define C7_PON_SCH_CTRL		0x1bf021e0u /* scheduler control                    */
#define C7_DRN_CMD		0x1bf020f4u /* T-cont drain command / status        */
#define C7_IO_CMD_0_US		0x1bf05434u /* upstream NIC GMII TX/RX enables       */
#define C7_PON_SID2QID		0x1bf02108u /* flow(SID) -> physical queue map       */
#define C7_PON_QID_CIR_RATE	0x1bf021e4u /* per-queue committed (CIR) rate        */
#define C7_PON_QID_PIR_RATE	0x1bf023e4u /* per-queue peak (PIR) rate             */
#define C7_PON_SCH_QMAP		0x1bf025e4u /* per-tcont queue membership mask       */
#define C7_PON_WFQ_TYPE		0x1bf02668u /* per-queue strict/WFQ select           */
#define C7_PON_WFQ_WEIGHT	0x1bf0267cu /* per-queue WFQ weight                  */
#define C7_PON_TCONT_EN		0x1bf02664u /* per-tcont schedule enable             */

/* PON trap / accept-length (0x1b011xxx) */
#define C7_PON_TRAP_CFG		0x1b011144u /* OMCI/MPCP trap priority              */
#define C7_ACCEPT_MAX_LEN	0x1b011028u /* per-port accept max length (stride 4) */

/* switch global (0x1b000xxx / 0x1b002xxx) */
#define C7_SOFTWARE_RST		0x1b000108u /* soft-reset: SW core / SerDes+GPON-MAC */
#define C7_DYNGASP_CTRL		0x1b00029cu /* dying-gasp comparator control         */
#define C7_SDS_CFG		0x1b000270u /* SerDes lane mode select               */
#define C7_PON_INBW_LBOUND	0x1b023288u /* DS in-band accumulation low bound     */
#define C7_P_MISC_PON		0x1b020504u /* per-port misc, PON port (base 0x20004 + port5*0x100) */

/* SerDes digital block (0x1b040xxx) */
#define C7_WSDS_DIG_00		0x1b040030u /* SerDes digital: 125 MHz clock control */
#define C7_WSDS_DIG_02		0x1b040038u /* SerDes digital: BEN power-down        */
#define C7_WSDS_DIG_03		0x1b04003cu /* SerDes digital: TX-disable sel delay  */
#define C7_WSDS_DIG_18		0x1b040090u /* SerDes digital: BEN output enable     */
#define C7_WSDS_DIG_1D		0x1b0400a4u /* SerDes digital: interface FIFO rstb   */
#define C7_FORCE_BEN		0x1b0400e4u /* burst-enable force mode               */

/* SerDes analog common / GPON / misc (0x1b0405xx..0x1b0407xx, 0x1b040exx) */
#define C7_SDS_ANA_MISC02	0x1b040508u /* analog misc: BER-notify force/value   */
#define C7_SDS_ANA_COM00	0x1b040580u /* analog common: CDR Kd (rev-B)         */
#define C7_SDS_ANA_COM02	0x1b040588u /* analog common: CDR Ki/Kp1/Kp2         */
#define C7_SDS_ANA_COM05	0x1b040594u /* analog common: RX EQ hold             */
#define C7_SDS_ANA_COM06	0x1b040598u /* analog common: RX filter / RX EQ in   */
#define C7_SDS_ANA_COM08	0x1b0405a0u /* analog common: RX Kp1_2 / Kp2_2       */
#define C7_SDS_ANA_COM09	0x1b0405a4u /* analog common: RX CDR/timer/re-seat   */
#define C7_SDS_ANA_COM12	0x1b0405b0u /* analog common: RX EQ2 select          */
#define C7_SDS_ANA_COM13	0x1b0405b4u /* analog common: TX amplitude           */
#define C7_SDS_ANA_COM14	0x1b0405b8u /* analog common: TX emphasis / Z0 P-adj */
#define C7_SDS_ANA_COM15	0x1b0405bcu /* analog common: Z0 N-adjust            */
#define C7_SDS_ANA_COM17	0x1b0405c4u /* analog common: BEN CML/TTL drive      */
#define C7_SDS_ANA_COM21	0x1b0405d4u /* analog common: RX CMU CCO/CP/KVCO/LPF */
#define C7_SDS_ANA_COM23	0x1b0405dcu /* analog common: CMU watchdog (RX)      */
#define C7_SDS_ANA_COM24	0x1b0405e0u /* analog common: TX CMU CP / LPF-CP     */
#define C7_SDS_ANA_COM25	0x1b0405e4u /* analog common: TX CMU LPF-RS / LC byp */
#define C7_SDS_ANA_COM26	0x1b0405e8u /* analog common: CMU watchdog (TX)      */
#define C7_SDS_ANA_COM30	0x1b0405f8u /* analog common: GPHY CMU CP/ICP/LPF-CP */
#define C7_SDS_ANA_COM31	0x1b0405fcu /* analog common: GPHY CMU LPF-RS        */
#define C7_SDS_ANA_GPON34	0x1b040708u /* analog GPON: GPHY CMU watchdog        */
#define C7_SDS_ANA_GPON36	0x1b040710u /* analog GPON: GPHY field lock-dn limit */
#define C7_SDS_ANA_GPON37	0x1b040714u /* analog GPON: GPHY dly-clk/lock-up lim */
#define C7_SDS_ANA_GPON43	0x1b04072cu /* analog GPON: TX delay-clock select    */
#define C7_FIB_EXT_REG21	0x1b040e54u /* fiber ext: analog-ready status        */
#define C7_FIB_REG0		0x1b040c00u /* [11] FP_CFG_FIB_PDOWN (0 = fiber on) */

/* fixed chip parameters for the GPON datapath */
#define C7_PON_PORT		5	/* PON port index for per-port registers */
#define C7_MACPP_STRIDE		0x100u	/* per-port MAC register stride          */
#define C7_SID_COUNT		128	/* classifier SID / flow slots           */
#define C7_GPON_TCONT_MAX	32	/* T-cont count                          */
#define C7_PON_QUEUE_MAX	128	/* physical PON queue count              */
#define C7_TCONT_QUEUE_MAX	32	/* queues per T-cont scheduler           */
#define C7_RATE_MAX		0x3ffffu/* CIR/PIR rate saturation value         */
#define C7_OMCI_FLOW		127	/* flow id reserved for OMCI            */
#define C7_OMCI_TCONT		31	/* T-cont id for the OMCI flow           */
#define C7_OMCI_QUEUE		24	/* queue id for the OMCI flow            */

/* one-shot init guard: drain T-conts only on a re-init */
static int c7_init_done;

/*
 * Packed/strided array element write. For arroff<32 a 32-bit word holds
 * (32/arroff) elements: the index picks both the word and the bit offset.
 * For arroff>=32 each element owns a word (byte stride arroff/8). 'len' is the
 * element field width.
 */
static void c7_arr(const struct rtl960x_ops *o, u32 base, u32 arroff,
		   u32 idx, u32 lsp, u32 len, u32 val)
{
	u32 phys, lsb;

	if (arroff % 32u) {
		u32 per_word = 32u / arroff;

		phys = base + (idx / per_word) * 4u;
		lsb  = (idx % per_word) * arroff + lsp;
	} else {
		phys = base + idx * (arroff / 8u);
		lsb  = lsp;
	}
	rtl960x_rfwr(o, phys, lsb + len - 1u, lsb, val);
}

/* GPON physical queue id = TCONT_QUEUE_MAX*(sched/8) + logical queue */
static void c7_flow2queue(const struct rtl960x_ops *o, u32 flow, u32 sched, u32 q)
{
	c7_arr(o, C7_PON_SID2QID, 7, flow, 0, 7,
	       C7_TCONT_QUEUE_MAX * (sched / 8u) + q);
}

/* CIR/PIR are stored as (rate-1) except for the 0/1/max sentinels */
static u32 c7_rate(u32 rate)
{
	if (rate != 0 && rate != 1 && rate != C7_RATE_MAX)
		return rate - 1u;
	return rate;
}

/*
 * Drain one T-cont, busy-polling the drain flag. On timeout, recover the
 * upstream NIC by toggling its GMII enables (RX off, TX off->on, RX on).
 */
static void c7_tcont_drain(const struct rtl960x_ops *o, u32 tcont)
{
	u32 i;

	/* queue-mode=0, drain-index=tcont, drain-pulse=1 */
	o->wr(C7_DRN_CMD, ((tcont & 0x7fu) << 3) | (1u << 1));

	for (i = 0; i < 200000u; i++)
		if (!(o->rd(C7_DRN_CMD) & 0x1u))	/* drain flag cleared */
			break;

	if (i >= 200000u) {
		rtl960x_rfwr(o, C7_IO_CMD_0_US, 5, 5, 0);	/* US NIC RX off */
		rtl960x_rfwr(o, C7_IO_CMD_0_US, 4, 4, 0);	/* US NIC TX off */
		rtl960x_rfwr(o, C7_IO_CMD_0_US, 4, 4, 1);	/* US NIC TX on  */
		rtl960x_rfwr(o, C7_IO_CMD_0_US, 5, 5, 1);	/* US NIC RX on  */
	}
}

/*
 * ponmac_init: PON-MAC global defaults applied once before mode selection.
 * Single-ended BEN (TTL output on), US BW thresholds, per-T-cont disable +
 * mask clear, PIR overflow drop, per-queue strict/CIR=0/PIR=max/weight=1,
 * OMCI trap priority and dying-gasp comparator polarity. (rev>A would also
 * init switch-PBO; that lives in a separate subsystem.)
 */
static int c7_ponmac_init(const struct rtl960x_ops *o, int rev, int subtype)
{
	u32 i;

	(void)subtype; (void)rev;

	rtl960x_rfwr(o, C7_SDS_ANA_COM17, 0, 0, 1);	/* BEN TTL output on    */
	rtl960x_rfwr(o, C7_PON_BW_THRES, 29, 16, 5);	/* US last-grant thresh */
	rtl960x_rfwr(o, C7_PON_BW_THRES, 13,  0, 5);	/* US runt-request thresh*/

	if (c7_init_done)
		for (i = 0; i < C7_GPON_TCONT_MAX; i++)
			c7_tcont_drain(o, i);

	for (i = 0; i < C7_GPON_TCONT_MAX - 1u; i++) {
		c7_arr(o, C7_PON_TCONT_EN, 1, i, 0, 1, 0);	/* T-cont disable */
		c7_arr(o, C7_PON_SCH_QMAP, 32, i, 0, 32, 0);	/* clear queue mask*/
	}

	rtl960x_rfwr(o, C7_PON_SCH_CTRL, 18, 18, 1);	/* drop on PIR overflow */

	for (i = 0; i < C7_PON_QUEUE_MAX; i++) {
		c7_arr(o, C7_PON_WFQ_TYPE,    1,  i, 0,  1, 0);			/* strict   */
		c7_arr(o, C7_PON_QID_CIR_RATE,18, i, 0, 18, c7_rate(0));		/* CIR = 0  */
		c7_arr(o, C7_PON_QID_PIR_RATE,18, i, 0, 18, c7_rate(C7_RATE_MAX));/* PIR = max*/
		c7_arr(o, C7_PON_WFQ_WEIGHT,  10, i, 0, 10, 1);			/* weight=1 */
	}

	rtl960x_rfwr(o, C7_PON_TRAP_CFG, 2, 0, 7);	/* OMCI/MPCP top priority */
	rtl960x_rfwr(o, C7_DYNGASP_CTRL, 3, 3, 1);	/* invert dying-gasp cmp  */

	c7_init_done = 1;
	return 0;
}

/*
 * Shared GPON SID/OMCI front matter (identical before each rev's SerDes patch):
 * park every data flow on T-cont 15 / queue 31 with its SID invalid, then
 * dedicate the OMCI flow to its own T-cont/queue, mark its SID valid, and point
 * the OMCI SID select at it.
 */
static void c7_gpon_pre(const struct rtl960x_ops *o)
{
	u32 f;

	for (f = 0; f < C7_SID_COUNT - 1u; f++) {
		c7_flow2queue(o, f, 15, 31);
		c7_arr(o, C7_PON_SIDVALID, 1, f, 0, 1, 0);
	}
	c7_flow2queue(o, C7_OMCI_FLOW, C7_OMCI_TCONT, C7_OMCI_QUEUE);
	c7_arr(o, C7_PON_SIDVALID, 1, C7_OMCI_FLOW, 0, 1, 1);
	rtl960x_rfwr(o, C7_PON_OMCI_CFG, 6, 0, C7_OMCI_FLOW);
}

/*
 * Shared GPON tail (identical after each rev's SerDes patch): the GPON mode
 * change needs a switch-core reset, then re-arm the TX/RX interface FIFO
 * release-B, BEN output on, accept undersize frames on the PON port, drop BEN
 * force mode, set accept max length, wait analog-ready then drop the 125 MHz
 * clock for power saving, and seed the DS in-band low bound.
 */
static int c7_gpon_post(const struct rtl960x_ops *o)
{
	u32 i;

	rtl960x_rfwr(o, C7_SOFTWARE_RST, 10, 10, 1);	/* switch-core reset */
	mdelay(10);

	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 16, 16, 0);	/* TX iface FIFO assert rstb  */
	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 16, 16, 1);	/* TX iface FIFO release rstb */
	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 15, 15, 0);	/* RX iface FIFO assert rstb  */
	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 15, 15, 1);	/* RX iface FIFO release rstb */

	rtl960x_rfwr(o, C7_WSDS_DIG_18, 12, 12, 1);	/* BEN output on        */
	rtl960x_rfwr(o, C7_P_MISC_PON, 2, 2, 1);	/* PON port accept undersize */
	rtl960x_rfwr(o, C7_FORCE_BEN, 0, 0, 0);		/* BEN force mode off   */
	rtl960x_rfwr(o, C7_ACCEPT_MAX_LEN + C7_PON_PORT * 4u, 13, 0, 2031); /* max len */

	for (i = 0; i < 10000u; i++) {		/* wait analog-ready (V2ANALOG) */
		if ((o->rd(C7_FIB_EXT_REG21) >> 13) & 0x1u)
			break;
		udelay(200);
	}
	if (i < 10000u)
		rtl960x_rfwr(o, C7_WSDS_DIG_00, 4, 4, 0);	/* 125 MHz clock off */

	rtl960x_rfwr(o, C7_PON_INBW_LBOUND, 23, 0, 0xfda000);	/* DS in-band lbound */
	return 0;
}

/*
 * rev-A SerDes patch (mode V1): park the lane, force the 125 MHz reference on,
 * load tuned TX CMU/PLL + RX CDR/CMU/EQ analog coefficients, switch the lane
 * into GPON, then reset SerDes+MAC and re-arm BER-notify so a signal-detect
 * drop will not knock the MAC down.
 */
static const struct r960_op c7_sds_v1[] = {
	WR(C7_FIB_REG0,            0x1140),	/* fiber analog: power on (PDOWN=0)*/
	FLD(C7_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)        */
	FLD(C7_WSDS_DIG_00,    4,  4, 0x1),	/* force 125 MHz reference clock   */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down            */
	FLD(C7_WSDS_DIG_03,    6,  4, 0x2),	/* TX-disable select delay         */
	FLD(C7_SDS_ANA_COM17,  4,  4, 0x0),	/* BEN driver: CML off             */
	FLD(C7_SDS_ANA_COM17,  0,  0, 0x1),	/* BEN driver: TTL output on       */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0xF),	/* TX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x0),	/* TX CMU LPF charge-pump          */
	FLD(C7_SDS_ANA_COM25, 15, 13, 0x7),	/* TX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM25,  1,  1, 0x0),	/* LC bypass off                   */
	FLD(C7_SDS_ANA_COM21, 15, 15, 0x1),	/* RX CMU CCO select               */
	FLD(C7_SDS_ANA_COM21, 14, 11, 0xC),	/* RX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM21,  6,  6, 0x0),	/* RX CMU big-KVCO off             */
	FLD(C7_SDS_ANA_COM21,  4,  2, 0x3),	/* RX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM09, 13, 13, 0x0),	/* RX CDR AFE deselect             */
	FLD(C7_SDS_ANA_COM06,  7,  0, 0x2),	/* RX filter config                */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x1),	/* CDR Ki                          */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x4),	/* CDR Kp1                         */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x4),	/* CDR Kp2                         */
	FLD(C7_SDS_ANA_COM08, 14, 12, 0x4),	/* RX Kp1_2                        */
	FLD(C7_SDS_ANA_COM08, 11,  9, 0x4),	/* RX Kp2_2                        */
	FLD(C7_SDS_ANA_COM12,  7,  4, 0x1),	/* RX EQ2 select                   */
	FLD(C7_SDS_ANA_COM12,  3,  0, 0x1),	/* RX EQ2 select 2                 */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                    */
	FLD(C7_SDS_ANA_COM14, 11,  9, 0x0),	/* TX emphasis                     */
	FLD(C7_SDS_ANA_COM14,  8,  8, 0x1),	/* TX emphasis enable              */
	FLD(C7_SDS_CFG,        4,  0, 0x8),	/* lane mode: GPON                 */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x0),	/* release BER-notify force        */
	FLD(C7_SOFTWARE_RST,   2,  0, 0x1),	/* reset SerDes + GPON MAC         */
	DLY(10),				/* let the reset settle            */
	FLD(C7_SDS_ANA_MISC02,13, 13, 0x1),	/* BER-notify hold value = 1       */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x1),	/* re-force BER-notify              */
};

/*
 * rev-B SerDes patch (mode V2): same layout as V1 with retuned CDR/RX gains
 * (Kp1/Kp2, RX Kp1_2/Kp2_2) plus a CDR Kd write that only exists on this rev.
 */
static const struct r960_op c7_sds_v2[] = {
	WR(C7_FIB_REG0,            0x1140),	/* fiber analog: power on (PDOWN=0)*/
	FLD(C7_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)        */
	FLD(C7_WSDS_DIG_00,    4,  4, 0x1),	/* force 125 MHz reference clock   */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down            */
	FLD(C7_WSDS_DIG_03,    6,  4, 0x2),	/* TX-disable select delay         */
	FLD(C7_SDS_ANA_COM17,  4,  4, 0x0),	/* BEN driver: CML off             */
	FLD(C7_SDS_ANA_COM17,  0,  0, 0x1),	/* BEN driver: TTL output on       */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0xF),	/* TX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x0),	/* TX CMU LPF charge-pump          */
	FLD(C7_SDS_ANA_COM25, 15, 13, 0x7),	/* TX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM25,  1,  1, 0x0),	/* LC bypass off                   */
	FLD(C7_SDS_ANA_COM21, 15, 15, 0x1),	/* RX CMU CCO select               */
	FLD(C7_SDS_ANA_COM21, 14, 11, 0xC),	/* RX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM21,  6,  6, 0x0),	/* RX CMU big-KVCO off             */
	FLD(C7_SDS_ANA_COM21,  4,  2, 0x3),	/* RX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM09, 13, 13, 0x0),	/* RX CDR AFE deselect             */
	FLD(C7_SDS_ANA_COM06,  7,  0, 0x2),	/* RX filter config                */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x1),	/* CDR Ki                          */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x0),	/* CDR Kp1 (rev-B)                 */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x6),	/* CDR Kp2 (rev-B)                 */
	FLD(C7_SDS_ANA_COM08, 14, 12, 0x1),	/* RX Kp1_2 (rev-B)                */
	FLD(C7_SDS_ANA_COM08, 11,  9, 0x1),	/* RX Kp2_2 (rev-B)                */
	FLD(C7_SDS_ANA_COM12,  7,  4, 0x1),	/* RX EQ2 select                   */
	FLD(C7_SDS_ANA_COM12,  3,  0, 0x1),	/* RX EQ2 select 2                 */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                    */
	FLD(C7_SDS_ANA_COM14, 11,  9, 0x0),	/* TX emphasis                     */
	FLD(C7_SDS_ANA_COM14,  8,  8, 0x1),	/* TX emphasis enable              */
	FLD(C7_SDS_ANA_COM00,  1,  1, 0x0),	/* CDR Kd (rev-B only)             */
	FLD(C7_SDS_CFG,        4,  0, 0x8),	/* lane mode: GPON                 */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x0),	/* release BER-notify force        */
	FLD(C7_SOFTWARE_RST,   2,  0, 0x1),	/* reset SerDes + GPON MAC         */
	DLY(10),				/* let the reset settle            */
	FLD(C7_SDS_ANA_MISC02,13, 13, 0x1),	/* BER-notify hold value = 1       */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x1),	/* re-force BER-notify              */
};

/*
 * rev-C+ SerDes patch (mode V3): a GPHY-CMU-centric tuning - disable the per
 * lane CMU watchdogs, retune TX/RX CMU and the GPHY CMU charge-pump/LPF and
 * lock limits, then switch into GPON and reset+re-arm as the other revs do.
 */
static const struct r960_op c7_sds_v3[] = {
	WR(C7_FIB_REG0,            0x1140),	/* fiber analog: power on (PDOWN=0)*/
	FLD(C7_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)        */
	FLD(C7_WSDS_DIG_00,    4,  4, 0x1),	/* force 125 MHz reference clock   */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down            */
	FLD(C7_WSDS_DIG_03,    6,  4, 0x2),	/* TX-disable select delay         */
	FLD(C7_SDS_ANA_GPON43,11, 11, 0x1),	/* TX delay-clock select           */
	FLD(C7_SDS_ANA_COM26,  3,  3, 0x0),	/* TX CMU watchdog off             */
	FLD(C7_SDS_ANA_COM23, 15, 15, 0x0),	/* RX CMU watchdog off             */
	FLD(C7_SDS_ANA_GPON34, 7,  7, 0x0),	/* GPHY CMU watchdog off           */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0x4),	/* TX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x1),	/* TX CMU LPF charge-pump          */
	FLD(C7_SDS_ANA_COM25, 15, 13, 0x3),	/* TX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM14,  4,  0, 0x7),	/* Z0 P-adjust                     */
	FLD(C7_SDS_ANA_COM15, 15, 12, 0x8),	/* Z0 N-adjust                     */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x6),	/* CDR Ki                          */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x1),	/* CDR Kp1                         */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x0),	/* CDR Kp2                         */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                    */
	FLD(C7_SDS_ANA_COM05,  2,  2, 0x1),	/* RX EQ hold                      */
	FLD(C7_SDS_ANA_COM06, 15,  9, 0x40),	/* RX EQ input                     */
	FLD(C7_SDS_ANA_COM09,  6,  2, 0x1f),	/* RX timer-BER                    */
	FLD(C7_SDS_ANA_GPON37, 5,  5, 0x1),	/* GPHY delay-clock select         */
	FLD(C7_SDS_ANA_GPON37,15,  6, 0x316),	/* GPHY lock-up limit              */
	FLD(C7_SDS_ANA_COM30, 15, 12, 0x3),	/* GPHY CMU charge-pump            */
	FLD(C7_SDS_ANA_COM30, 10, 10, 0x1),	/* GPHY CMU ICP low-BW             */
	FLD(C7_SDS_ANA_COM30,  4,  2, 0x2),	/* GPHY CMU LPF charge-pump        */
	FLD(C7_SDS_ANA_COM31, 15, 13, 0x0),	/* GPHY CMU LPF resistor           */
	FLD(C7_SDS_ANA_GPON36,15,  6, 0x302),	/* GPHY lock-down limit            */
	FLD(C7_SDS_CFG,        4,  0, 0x8),	/* lane mode: GPON                 */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x0),	/* release BER-notify force        */
	FLD(C7_SOFTWARE_RST,   2,  0, 0x1),	/* reset SerDes + GPON MAC         */
	DLY(10),				/* let the reset settle            */
	FLD(C7_SDS_ANA_MISC02,13, 13, 0x1),	/* BER-notify hold value = 1       */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x1),	/* re-force BER-notify              */
};

/*
 * GPON mode-set: common SID/OMCI front matter, the rev-selected SerDes patch
 * table, then the common GPON tail. rev A->V1, B->V2, C and later->V3.
 */
static int c7_gpon_mode_set(const struct rtl960x_ops *o, int rev, int subtype)
{
	const struct r960_op *sds;
	unsigned int n;
	int rc;

	(void)subtype;

	c7_gpon_pre(o);

	switch (rev) {
	case RTL960X_REV_A:
		sds = c7_sds_v1; n = ARRAY_SIZE(c7_sds_v1); break;
	case RTL960X_REV_B:
		sds = c7_sds_v2; n = ARRAY_SIZE(c7_sds_v2); break;
	default:
		sds = c7_sds_v3; n = ARRAY_SIZE(c7_sds_v3); break;
	}

	rc = r960_run(o, sds, n);
	if (rc)
		return rc;

	return c7_gpon_post(o);
}

/*
 * SerDes CDR reseat: flip the RX CDR re-seat bit (mask 0x400), settle, restore
 * the original analog word, then bounce the 16<->20-bit transfer FIFO
 * release-B. Re-acquires lock without a full re-bring-up.
 */
static int c7_cdr_reset(const struct rtl960x_ops *o)
{
	u32 v = o->rd(C7_SDS_ANA_COM09);

	o->wr(C7_SDS_ANA_COM09,
	      (v & ~0x400u) | ((u32)(!((v & 0x400u) >> 10)) << 10));
	mdelay(10);
	o->wr(C7_SDS_ANA_COM09, v);			/* restore original word   */

	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 14, 14, 0);	/* transfer FIFO assert rstb */
	mdelay(10);
	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 14, 14, 1);	/* transfer FIFO release rstb*/
	return 0;
}

/* RTL9607C top-level entry points (thin wrappers over the c7_* internals). */
static int rtl9607c_ponmac_init(const struct rtl960x_ops *o)
{
	return c7_ponmac_init(o, RTL960X_REV_A, RTL960X_SUBTYPE_NONE);
}

static int rtl9607c_ponmac_mode_set(const struct rtl960x_ops *o,
				    int rev, int subtype)
{
	return c7_gpon_mode_set(o, rev, subtype);
}

static int rtl9607c_serdes_cdr_reset(const struct rtl960x_ops *o)
{
	return c7_cdr_reset(o);
}

/* ------------------------------------------------------------------ *
 *  RTL9602C GPON PON-MAC / SerDes bring-up - clean-room op-table form.
 *  HW-TESTED on the realtek-luna board: this is a faithful translation of
 *  the in-tree gpon-rtl960x.c SerDes-init / PBO-ponmac steps
 *  into this file's op-table primitives, so the family-lib path behaves
 *  identically to that in-tree sibling driver.
 *
 *  Default parameter path baked in (the configuration that locks on hardware):
 *    serdes_cdr_reset = TRUE   -> the COM_REG08 bit15 invert/10ms/restore pulse
 *    serdes_modev1_tx = FALSE  -> SKIP the explicit COM_REG02/03/08/24/25 block
 *                                 (the golden analog table already covers them)
 *    serdes_tx_xtra   = FALSE  -> the D2A / sample-clock bits are forced to 0
 *                                 (0x220a8[5:4]=0, 0x2281c[14]=0, 0x22a30[8]=0)
 *    full_serdes_reinit = FALSE
 *
 *  Address spaces (absolute physical, supplied to the board's rd/wr via ops):
 *    SDS / swcore regs : 0x1B000000 + offset
 *    PON-IP regs       : 0x1BF00000 + offset
 *  Symbols are c2_/C2_ prefixed to stay collision-free with the other chips.
 * ------------------------------------------------------------------ */
#define C2_SWCORE_BASE		0x1B000000u	/* SDS / swcore offset base    */
#define C2_PONIP_BASE		0x1BF00000u	/* PON-IP datapath offset base */

/* swcore-relative register absolutes used by the ordered bring-up steps */
#define C2_SDS_CFG		0x1B0001D0u	/* [4:0] CFG_SDS_MODE          */
#define   C2_SDS_MODE_OFF	0x1Fu		/* illegal/off mode (park)     */
#define   C2_SDS_MODE_GPON	0x08u		/* GPON line-rate mode         */
#define C2_SW_SOFTWARE_RST	0x1B000104u	/* [7] SDS_CFG_RST [0] SDS_RST */
#define C2_WSDS_DIG_00		0x1B022030u	/* [0] STOP_CLK; run = 0xf30   */
#define   C2_WSDS_DIG00_RUN	0xF30u		/* operational run state       */
#define C2_WSDS_DIG_01		0x1B022034u	/* [31:0] CFG_DMY0 (force-SDS) */
#define C2_WSDS_DIG_02		0x1B022038u	/* [10] EN_PDOWN_BEN           */
#define C2_WSDS_DIG_03		0x1B02203Cu	/* [6:4] TXDIS_SEL_DLY [3:0]D2A*/
#define C2_WSDS_DIG_18		0x1B022090u	/* [12] BEN_OE [15:13] optic   */
#define C2_WSDS_DIG_1D		0x1B0220A4u	/* [16:14] interface reset-B   */
#define C2_WSDS_DIG_1E		0x1B0220A8u	/* [5:4] D2A interconnect      */
#define C2_SDS_REG7		0x1B02281Cu	/* [14] SP_CFG_NEG_CLKWR_A2D   */
#define C2_SDS_EXT_REG12	0x1B022A30u	/* [8] SEP_CFG_NEG_CLKRD_D2A   */
#define C2_SDS_FORCE_BEN	0x1B0220E4u	/* [0] BEN_FORCE_MODE          */
#define C2_SDS_ANA_COM_REG08	0x1B0225A0u	/* TX-CDR; [15] cdr_reset bit  */
#define C2_SDS_ANA_COM_REG12	0x1B0225B0u	/* [14] RX_SEL_CDR_AFEN        */
#define C2_SDS_ANA_COM_REG22	0x1B0225D8u	/* [5:3] TX_AMP [2:0] TX_EMP   */
#define C2_SDS_ANA_MISC_REG00	0x1B022500u	/* [5] FRC_RX_EN_VAL [4] _ON   */
#define C2_SDS_ANA_MISC_REG01	0x1B022504u	/* [7:5] SPDSEL_VAL [4] _ON    */
/* ⚠ RENAMED 2026-08-26. This was commented "[13] SD_VAL [12] SD_FORCE" and two
 * of its three call sites called it "signal-detect". It is NOT signal-detect on
 * either die: both the RTL9602C's and the RTL9603CVD's own register maps name
 * [13] FRC_BER_NOTIFY_VAL and [12] FRC_BER_NOTIFY_ON, which is what the third
 * call site and the whole 9603CVD path already called it. The wrong name
 * mattered: it made this register look like a reason SDS_SDET could read 0,
 * i.e. like an explanation for a dark fibre that it cannot provide. */
#define C2_SDS_ANA_MISC_REG02	0x1B022508u	/* [13] FRC_BER_NOTIFY_VAL [12] _ON */
#define C2_FIB_EXT_REG21	0x1B022E54u	/* [13] FEP_V2ANALOG (lock)    */
#define   C2_SDS_ANALOG_READY	13u		/* FIB_EXT_REG21 ready bit      */
#define C2_SDS_LOCK_POLL_MAX	1000u		/* x200us = up to 200 ms        */

/* FIB_REG0 bank bases (absolute); FP_CFG_FIB_PDOWN bit11 cleared = fiber on. */
#define C2_FIB_REG0_PDOWN	BIT(11)
static const u32 c2_fib_reg0_banks[] = {
	0x1B022C00u, 0x1B022C80u, 0x1B022D00u, 0x1B022D80u,
};

/*
 * Full SerDes analog + WSDS configuration golden table - the operating point
 * the ONU runs at O5 (CMU, CDR, RX front-end incl. optical signal-detect/LOS,
 * TX driver, GPON-rate banks, and the 4 FIB optical front-end banks). Each
 * entry is an ABSOLUTE swcore address (0x1B000000 + the chip offset) and its
 * operational value - register facts of this silicon. Status/monitor regs and
 * the digital reset-B/clock bank (DIG_00/18/1D) are deliberately excluded;
 * those are driven by the ordered sequence below. FIB_REG0 power-down (bit11)
 * is cleared separately, after each write, to turn fiber power on.
 */
static const struct { u32 off; u32 val; } c2_analog[] = {
	/* WSDS analog front + digital RX-path config */
	{ 0x1B022000u, 0x00000805 }, { 0x1B022008u, 0x0000ffff }, { 0x1B02201cu, 0x0000ffff },
	{ 0x1B022020u, 0x0000ffff }, { 0x1B022038u, 0x00000900 }, { 0x1B022048u, 0x000000ff },
	{ 0x1B022050u, 0x00022300 }, { 0x1B022054u, 0x00022310 }, { 0x1B022058u, 0x083d0100 },
	{ 0x1B022060u, 0x00000fff }, { 0x1B022064u, 0x0000cf45 }, { 0x1B022068u, 0x00000f45 },
	/* SDS_ANA_MISC (RX-enable force, speed-select, force-SD) */
	{ 0x1B022500u, 0x00000030 }, { 0x1B022504u, 0x00000030 }, { 0x1B022508u, 0x00003000 },
	/* SDS_ANA_COM (CMU, RX CDR front-end, filters, bias) */
	{ 0x1B022580u, 0x00003400 }, { 0x1B022584u, 0x000073a4 }, { 0x1B022588u, 0x00006df8 },
	{ 0x1B02258cu, 0x00008941 }, { 0x1B022590u, 0x00008884 }, { 0x1B022594u, 0x0000413f },
	{ 0x1B022598u, 0x00004fc0 }, { 0x1B02259cu, 0x00005682 }, { 0x1B0225a0u, 0x00000713 },
	{ 0x1B0225a4u, 0x000002f5 }, { 0x1B0225a8u, 0x00002793 }, { 0x1B0225acu, 0x0000b000 },
	{ 0x1B0225b0u, 0x00004848 }, { 0x1B0225b4u, 0x000000c8 }, { 0x1B0225bcu, 0x000008f2 },
	{ 0x1B0225c0u, 0x00001042 }, { 0x1B0225c4u, 0x0000c391 }, { 0x1B0225c8u, 0x00006a00 },
	{ 0x1B0225ccu, 0x00006600 }, { 0x1B0225d0u, 0x0000c000 },
	/* 0x225d8 (COM_REG22 TX_AMP/EMP) is set later, in the TX section, to the
	 * rev-A value 0x29 (TX_AMP=0x5, TX_EMP=0x1) via field-writes - NOT a full
	 * word here, so the upper bits keep their reset state. */
	{ 0x1B0225dcu, 0x00000418 }, { 0x1B0225e0u, 0x00008001 }, { 0x1B0225e4u, 0x0000001f },
	{ 0x1B0225e8u, 0x000011e4 }, { 0x1B0225ecu, 0x00009422 }, { 0x1B0225f0u, 0x00008502 },
	{ 0x1B0225f4u, 0x00000ff0 }, { 0x1B0225f8u, 0x0000000a },
	/* SDS_ANA_GPON (GPON-rate CDR/PLL/PCM config) */
	{ 0x1B022708u, 0x00000f00 }, { 0x1B02270cu, 0x0000b8c6 }, { 0x1B022710u, 0x0000a112 },
	{ 0x1B022714u, 0x00004280 }, { 0x1B022718u, 0x0000f53f }, { 0x1B02271cu, 0x00004fdf },
	{ 0x1B022720u, 0x00000001 }, { 0x1B022724u, 0x0000309b }, { 0x1B022728u, 0x0000225c },
	{ 0x1B02272cu, 0x00001061 }, { 0x1B022730u, 0x0000110d }, { 0x1B022734u, 0x00004854 },
	{ 0x1B022738u, 0x000080c5 }, { 0x1B02273cu, 0x0000121e }, { 0x1B022740u, 0x0000307b },
	{ 0x1B022744u, 0x00000271 }, { 0x1B022748u, 0x00000271 }, { 0x1B02274cu, 0x00001012 },
	{ 0x1B022750u, 0x0000f162 }, { 0x1B022754u, 0x00003026 }, { 0x1B022758u, 0x0000a780 },
	{ 0x1B02275cu, 0x0000f000 },
	/* SDS_ANA_GPON additional per-rate/lane banks (the RX path selects among
	 * these; leaving them at reset starves the active RX/SD analog). */
	{ 0x1B022608u, 0x00000f00 }, { 0x1B02260cu, 0x0000b8c6 }, { 0x1B022610u, 0x0000a112 },
	{ 0x1B022614u, 0x00004280 }, { 0x1B022618u, 0x0000f53f }, { 0x1B02261cu, 0x00004fdf },
	{ 0x1B022620u, 0x00000001 }, { 0x1B022624u, 0x0000309b }, { 0x1B022628u, 0x0000225c },
	{ 0x1B02262cu, 0x00001061 }, { 0x1B022630u, 0x0000110d }, { 0x1B022634u, 0x00004854 },
	{ 0x1B022638u, 0x000080c5 }, { 0x1B02263cu, 0x0000121e }, { 0x1B022640u, 0x0000307b },
	{ 0x1B022644u, 0x00000271 }, { 0x1B022648u, 0x00000271 }, { 0x1B02264cu, 0x00001012 },
	{ 0x1B022650u, 0x0000f162 }, { 0x1B022654u, 0x00003026 }, { 0x1B022658u, 0x0000a780 },
	{ 0x1B02265cu, 0x0000f000 },
	{ 0x1B022688u, 0x00000f00 }, { 0x1B02268cu, 0x0000b8c6 }, { 0x1B022690u, 0x0000a112 },
	{ 0x1B022694u, 0x00004280 }, { 0x1B022698u, 0x0000f53f }, { 0x1B02269cu, 0x00004fdf },
	{ 0x1B0226a0u, 0x00000001 }, { 0x1B0226a4u, 0x0000309b }, { 0x1B0226a8u, 0x0000225c },
	{ 0x1B0226acu, 0x00001062 }, { 0x1B0226b0u, 0x00002000 }, { 0x1B0226b4u, 0x00001050 },
	{ 0x1B0226b8u, 0x000080c1 }, { 0x1B0226bcu, 0x0000121e }, { 0x1B0226c0u, 0x0000107b },
	{ 0x1B0226c4u, 0x00000280 }, { 0x1B0226c8u, 0x00000280 }, { 0x1B0226ccu, 0x00001012 },
	{ 0x1B0226d0u, 0x0000f862 }, { 0x1B0226d4u, 0x00003938 }, { 0x1B0226d8u, 0x00003100 },
	{ 0x1B0226dcu, 0x0000f000 },
	{ 0x1B022788u, 0x00000f00 }, { 0x1B02278cu, 0x0000b8c6 }, { 0x1B022790u, 0x0000a112 },
	{ 0x1B022794u, 0x00004280 }, { 0x1B022798u, 0x0000f53f }, { 0x1B02279cu, 0x00004fdf },
	{ 0x1B0227a0u, 0x00000001 }, { 0x1B0227a4u, 0x0000309b }, { 0x1B0227a8u, 0x0000225c },
	{ 0x1B0227acu, 0x00001062 }, { 0x1B0227b0u, 0x00002000 }, { 0x1B0227b4u, 0x00004850 },
	{ 0x1B0227b8u, 0x000080c5 }, { 0x1B0227bcu, 0x0000121e }, { 0x1B0227c0u, 0x0000103e },
	{ 0x1B0227c4u, 0x00000280 }, { 0x1B0227c8u, 0x00000280 }, { 0x1B0227ccu, 0x00001012 },
	{ 0x1B0227d0u, 0x0000f862 }, { 0x1B0227d4u, 0x00003938 }, { 0x1B0227d8u, 0x0000b100 },
	{ 0x1B0227dcu, 0x0000f000 },
	/* FIB (fiber optical front-end) config - 4 identical banks. This block
	 * powers and configures the optical RX/SD path; FIB_REG0 (bank base)
	 * carries FP_CFG_FIB_PDOWN at bit11, cleared separately below to turn
	 * fiber power on. */
	{ 0x1B022c00u, 0x00001940 }, { 0x1B022c04u, 0x00006109 }, { 0x1B022c08u, 0x0000e001 },
	{ 0x1B022c0cu, 0x00003290 }, { 0x1B022c10u, 0x000001a0 }, { 0x1B022c1cu, 0x00000004 },
	{ 0x1B022c3cu, 0x00008000 }, { 0x1B022c40u, 0x00000083 }, { 0x1B022c48u, 0x00005000 },
	{ 0x1B022c58u, 0x00000001 }, { 0x1B022c5cu, 0x00004001 }, { 0x1B022c60u, 0x00000004 },
	{ 0x1B022c64u, 0x0000326a }, { 0x1B022c6cu, 0x0000115d }, { 0x1B022c70u, 0x000033fa },
	{ 0x1B022c74u, 0x0000e46a }, { 0x1B022c78u, 0x0000071e },
	{ 0x1B022c80u, 0x00001940 }, { 0x1B022c84u, 0x00006109 }, { 0x1B022c88u, 0x0000e001 },
	{ 0x1B022c8cu, 0x00003290 }, { 0x1B022c90u, 0x000001a0 }, { 0x1B022c9cu, 0x00000004 },
	{ 0x1B022cbcu, 0x00008000 }, { 0x1B022cc0u, 0x00000083 }, { 0x1B022cc8u, 0x00005000 },
	{ 0x1B022cd8u, 0x00000001 }, { 0x1B022cdcu, 0x00004001 }, { 0x1B022ce0u, 0x00000004 },
	{ 0x1B022ce4u, 0x0000326a }, { 0x1B022cecu, 0x0000115d }, { 0x1B022cf0u, 0x000033fa },
	{ 0x1B022cf4u, 0x0000e46a }, { 0x1B022cf8u, 0x0000071e },
	{ 0x1B022d00u, 0x00001940 }, { 0x1B022d04u, 0x00006109 }, { 0x1B022d08u, 0x0000e001 },
	{ 0x1B022d0cu, 0x00003290 }, { 0x1B022d10u, 0x000001a0 }, { 0x1B022d1cu, 0x00000004 },
	{ 0x1B022d3cu, 0x00008000 }, { 0x1B022d40u, 0x00000083 }, { 0x1B022d48u, 0x00005000 },
	{ 0x1B022d58u, 0x00000001 }, { 0x1B022d5cu, 0x00004001 }, { 0x1B022d60u, 0x00000004 },
	{ 0x1B022d64u, 0x0000326a }, { 0x1B022d6cu, 0x0000115d }, { 0x1B022d70u, 0x000033fa },
	{ 0x1B022d74u, 0x0000e46a }, { 0x1B022d78u, 0x0000071e },
	{ 0x1B022d80u, 0x00001940 }, { 0x1B022d84u, 0x00006109 }, { 0x1B022d88u, 0x0000e001 },
	{ 0x1B022d8cu, 0x00003290 }, { 0x1B022d90u, 0x000001a0 }, { 0x1B022d9cu, 0x00000004 },
	{ 0x1B022dbcu, 0x00008000 }, { 0x1B022dc0u, 0x00000083 }, { 0x1B022dc8u, 0x00005000 },
	{ 0x1B022dd8u, 0x00000001 }, { 0x1B022ddcu, 0x00004001 }, { 0x1B022de0u, 0x00000004 },
	{ 0x1B022de4u, 0x0000326a }, { 0x1B022decu, 0x0000115d }, { 0x1B022df0u, 0x000033fa },
	{ 0x1B022df4u, 0x0000e46a }, { 0x1B022df8u, 0x0000071e },
};

/*
 * ponmac_init scheduler / OMCI-egress steering (the SDS/PON-IP config writes).
 * BEN_TTL_OUT + DYNGASP on swcore; PON_BW_THRES (last + runt) and the transient
 * PON_GEN_PIR_DROP on PON-IP; OMCI_MPCP_PRIORITY steers OMCI egress to PON
 * queue 7 on swcore. PIR_DROP is asserted then cleared for rev-A, matching the
 * in-tree sibling driver's set/clear pair.
 */
static const struct r960_op c2_ponmac_init[] = {
	/* REG01 (SDS_ANA_COM 0x22584) is handled in rtl9602c_ponmac_init() below — the
	 * stock-good post-reset value is 0x73a4 (CMU bit14=1, BEN_TTL_OUT bit0=0), which
	 * the golden-before-reset write cannot achieve (the SDS reset wipes bit14 and the
	 * old BEN_TTL write set bit0). See rtl960x_c2_stock_analog. */
	FLD(0x1B0001ECu,  0,  0, 1),	/* DYNGASP_CMP_INV = 1                  */
	FLD(0x1BF02150u, 29, 16, 5),	/* PON_BW_THRES last-grant              */
	FLD(0x1BF02150u, 13,  0, 5),	/* PON_BW_THRES runt-grant             */
	FLD(0x1BF02194u, 18, 18, 1),	/* PON_GEN_PIR_DROP = 1                 */
	FLD(0x1B0111F8u,  2,  0, 7),	/* PON_TRAP_CFG OMCI_MPCP_PRIORITY = 7  */
	FLD(0x1BF02194u, 18, 18, 0),	/* rev-A: clear PON_GEN_PIR_DROP        */
};

/* A/B knob (gpon.serdes_stock_analog). Default 1 = drive REG01/REG11 to the live-stock
 * post-reset values: REG01 (0x22584) = 0x73a4 (CMU bit14=1, BEN_TTL_OUT bit0=0) and
 * REG11 (0x225ac) RX_FILT_CONFIG[7:0] = 0. These are the ONLY two SerDes registers that
 * differed between live-stock (WAN-up, 100%) and our failing board (the cold-start ~50%
 * US-TX lock): the golden table sets them correctly BEFORE the SDS reset, but the reset
 * wipes REG01 bit14 / REG11 RX_FILT to defaults (0x33a4 / 0xb008) and nothing re-applied
 * them post-reset (stock applies its analog config AFTER the reset). bit14 sits in the
 * shared CMU block -> a marginal TX serializer that locks only ~50% per power-on.
 * =0 restores the legacy BEN_TTL_OUT=1 and leaves REG01 bit14 / REG11 at reset defaults. */
int rtl960x_c2_stock_analog = 1;

/* A/B knob (gpon.serdes_analog_postreset). Default 1 = program the FULL analog
 * CMU/CDR golden table AFTER the SDS reset (stock rev-A order: the SDS reset
 * runs first, then the ModeV1 path programs the analog), not before it. The SDS reset
 * (CMD_SDS_RST_PS) WIPES analog back to reset defaults; programming it pre-reset
 * (legacy) leaves the CMU charge-pump/LDO/tank (COM_REG02/03/08/24/25) + GPON CDR
 * (GPON_REG46) acquiring lock against reset-DEFAULT operating-point values, which the
 * partial REG01/REG11 re-apply never fully corrects -> metastable per-power-on lock =
 * the cold-start ~50% US-TX "Laser out". Post-reset placement pins the operating point
 * BEFORE the CMU re-locks and BEFORE the RX_EN 0->1 start edge -> deterministic lock on
 * every cold boot AND soft/internal restart (re-derived from scratch each mode_set).
 * =0 keeps the legacy pre-reset placement. */
int rtl960x_c2_analog_postreset = 1;

static int rtl9602c_ponmac_init(const struct rtl960x_ops *o)
{
	int ret = r960_run(o, c2_ponmac_init, ARRAY_SIZE(c2_ponmac_init));

	if (ret)
		return ret;
	if (rtl960x_c2_stock_analog) {
		rtl960x_rfwr(o, 0x1B022584u, 14, 14, 1);	/* REG01 CMU bit14 = 1 (stock) */
		rtl960x_rfwr(o, 0x1B022584u,  0,  0, 0);	/* REG01 BEN_TTL_OUT = 0 (stock) */
		rtl960x_rfwr(o, 0x1B0225ACu,  7,  0, 0);	/* REG11 RX_FILT_CONFIG = 0 (stock) */
	} else {
		rtl960x_rfwr(o, 0x1B022584u,  0,  0, 1);	/* legacy REG_BEN_TTL_OUT = 1 */
	}
	return 0;
}

/*
 * SerDes CDR-lock pulse (the stock CDR-reset behavior): invert
 * SDS_ANA_COM_REG12 (0x1B0225B0) bit15 (REG_RX_SD_POR_SEL), hold 10 ms, restore.
 * Re-PORs the RX signal-detect path so the recovered CDR re-acquires cleanly.
 *
 * REGISTER FIX 2026-06-17: the stock CDR-reset operates on REG12 (0x225B0), NOT
 * REG08 (0x225A0) — confirmed from the observed stock register behavior
 * (the CDR-reset reads/inverts/restores REG12[15]) and the chip's register/field map
 * (REG12[15]=REG_RX_SD_POR_SEL; REG08[15] is in the RESERVED top-16 field, so
 * the prior REG08[15] toggle wrote a reserved bit = wrong/no-op-with-side-effects).
 */
static int rtl9602c_serdes_cdr_reset(const struct rtl960x_ops *o)
{
	u32 cdr = o->rd(C2_SDS_ANA_COM_REG12);

	o->wr(C2_SDS_ANA_COM_REG12, cdr ^ BIT(15));
	mdelay(10);
	o->wr(C2_SDS_ANA_COM_REG12, cdr);
	return 0;
}

/*
 * GPON SerDes (SDS) bring-up - a faithful translation of the stock SerDes-init
 * sequence.
 *
 * Ordering is the whole game: program the analog CMU/CDR block FIRST, keep
 * CFG_SDS_MODE parked at the illegal/off value, pulse the SDS+MAC reset to
 * latch the analog config, release the per-datapath soft-reset-B lines and
 * force the 125M reference clock, arm the RX-CDR through a forced RX-enable
 * 0->1 edge, set the TX data path + drive level, force signal-detect, and only
 * THEN switch CFG_SDS_MODE to GPON. A naive "reset-then-configure" sequence
 * ends with the same final register values yet a CDR that never locks.
 */

/* Step 1 + 3: park SDS mode off, clear force-SDS dummy / STOP_CLK, then pulse
 * the SDS config + datapath reset to latch the analog config (10 ms). */
static const struct r960_op c2_sds_pre[] = {
	FLD(C2_SDS_CFG, 4, 0, C2_SDS_MODE_OFF),	/* park CFG_SDS_MODE = off (0x1f) */
	WR(C2_WSDS_DIG_01, 0),			/* clear force-SDS dummy          */
	FLD(C2_WSDS_DIG_00, 0, 0, 0),		/* STOP_CLK = 0                   */
};

/* Stock rev-A GPON ModeV1 pulses ONLY CMD_SDS_RST_PS (bit0). CMD_SDS_CFG_RST_PS
 * (bit7) is never written by the stock bring-up; since our field-writes
 * are RMW, asserting it here leaves it LATCHED through the whole bring-up = an extra
 * SDS-config reset domain stock never touches, the prime suspect for the per-power-on
 * US-TX serializer/PLL phase re-roll (cold-start WAN ~50%). bit7 is now applied
 * conditionally in rtl9602c_ponmac_mode_set behind rtl960x_c2_sds_cfgrst (default 0
 * = stock bit0-only = the fix). */
static const struct r960_op c2_sds_reset[] = {
	FLD(C2_SW_SOFTWARE_RST, 0, 0, 1),	/* CMD_SDS_RST_PS (bit0 only, stock)  */
	DLY(10),
};

/* Step 4: release all datapath soft-reset-B lines + force 125M ref (DIG_00 run
 * = 0xf30), then pulse the RX/TX interface reset-B lines (DIG_1D = 0x1c000). */
static const struct r960_op c2_sds_rstb[] = {
	WR(C2_WSDS_DIG_00, C2_WSDS_DIG00_RUN),	/* run state, 125M ref forced     */
	FLD(C2_WSDS_DIG_1D, 15, 15, 0),		/* RX interface reset-B 0         */
	FLD(C2_WSDS_DIG_1D, 16, 16, 0),		/* TX interface reset-B 0         */
	FLD(C2_WSDS_DIG_1D, 14, 14, 1),		/* common interface reset-B 1     */
	FLD(C2_WSDS_DIG_1D, 15, 15, 1),		/* RX interface reset-B 1         */
	FLD(C2_WSDS_DIG_1D, 16, 16, 1),		/* TX interface reset-B 1         */
	DLY(10),
};

/*
 * Steps 5 + 6: burst-enable output (BEN_OE) with optic-LOS left un-forced (the
 * real RX front-end drives SD via the external BOSA), then arm the RX in order:
 * enable the RX-CDR analog front end, settle 10 ms, force the line-rate select
 * to the GPON rate, drive the forced RX-enable through a 0->1 edge to start the
 * CDR, settle 50 ms. Finish with EN_PDOWN_BEN=0, TXDIS_SEL_DLY=0, D2A_SEL=0 and
 * BEN_FORCE_MODE=0 (let the GTC framer drive the laser gate).
 */
static const struct r960_op c2_sds_rx_arm[] = {
	FLD(C2_WSDS_DIG_18, 12, 12, 1),		/* BEN_OE = 1                     */
	FLD(C2_WSDS_DIG_18, 15, 15, 0),		/* OPTIC_LOS_SEL_EPON = 0         */
	FLD(C2_WSDS_DIG_18, 14, 14, 0),		/* CFG_FRC_OPTIC_LOS = 0          */
	FLD(C2_WSDS_DIG_18, 13, 13, 0),		/* CFG_FRCV_OPTIC_LOS = 0         */
	FLD(C2_SDS_ANA_COM_REG12, 14, 14, 1),	/* RX_SEL_CDR_AFEN = 1            */
	DLY(10),
	FLD(C2_SDS_ANA_MISC_REG01, 7, 5, 1),	/* SPDSEL_VAL = GPON rate         */
	FLD(C2_SDS_ANA_MISC_REG01, 4, 4, 1),	/* SPDSEL force on                */
	FLD(C2_SDS_ANA_MISC_REG00, 4, 4, 1),	/* FRC_RX_EN_ON = 1               */
	FLD(C2_SDS_ANA_MISC_REG00, 5, 5, 0),	/* FRC_RX_EN_VAL 0 ...            */
	FLD(C2_SDS_ANA_MISC_REG00, 5, 5, 1),	/* ... -> 1 (start CDR)           */
	DLY(50),
	FLD(C2_WSDS_DIG_02, 10, 10, 0),		/* EN_PDOWN_BEN = 0               */
	FLD(C2_WSDS_DIG_03, 6, 4, 0),		/* CFG_TXDIS_SEL_DLY = 0          */
	FLD(C2_WSDS_DIG_03, 3, 0, 0),		/* CFG_D2ANLOG_SEL = 0 (TX path)  */
	FLD(C2_SDS_FORCE_BEN, 0, 0, 0),		/* BEN_FORCE_MODE = 0 (GTC gates) */
};

/*
 * Step 6b + TX drive (serdes_modev1_tx=FALSE / serdes_tx_xtra=FALSE defaults):
 * the golden analog table already covers COM_REG02/03/08/24/25, so SKIP the
 * explicit ModeV1 block. Force the D2A interconnect + sample-clock bits to 0
 * (match the live stock ONU). Then set the rev-A TX drive level (TX_AMP=0x5,
 * TX_EMP=0x1) via field-writes, before switching to GPON mode.
 */
static const struct r960_op c2_sds_tx[] = {
	FLD(C2_WSDS_DIG_1E,    5,  4, 0),	/* D2A interconnect = 0 (stock)   */
	FLD(C2_SDS_REG7,      14, 14, 0),	/* SP_CFG_NEG_CLKWR_A2D = 0       */
	FLD(C2_SDS_EXT_REG12,  8,  8, 0),	/* SEP_CFG_NEG_CLKRD_D2A = 0      */
	FLD(C2_SDS_ANA_COM_REG22, 5, 3, 5),	/* REG_TX_AMP = 0x5               */
	FLD(C2_SDS_ANA_COM_REG22, 2, 0, 1),	/* REG_TX_EMP = 0x1               */
};

/*
 * Steps 7a + 7b: force signal-detect on so the MAC reset handshake (RST_DONE)
 * completes (MISC_REG02 = 0x3000), settle 10 ms, then finally select GPON mode
 * with the RX fully armed, settle 50 ms. Followed by a TX-interface reset-B
 * re-sync (DIG_1D[16] 0->1) so the TX serializer re-locks onto the connected
 * framer data now that GPON mode is live.
 */
/* A/B knob set by the board (gpon.serdes_postmode_perturb). When 0, skip the TWO
 * post-GPON-mode US-TX serializer perturbations that stock rev-A does NOT do: the
 * DIG_1D[16] reset-B re-sync (c2_sds_txresync below) and the post-mode
 * serdesCdr_reset pulse. These late edges on the already-running serializer are
 * the prime suspect for per-boot serializer-phase jitter (cold-start WAN ~50%).
 * Default 1 = legacy behavior. */
int rtl960x_c2_postmode_perturb = 1;

/* A/B knob (gpon.serdes_cmu_settle_ms): milliseconds to wait AFTER forcing the 125M
 * ref clock and BEFORE releasing the SerDes interface reset-B lines, so the TX CMU PLL
 * locks to the ref before the serializer phase is latched. 0 = legacy (no extra settle).
 * Candidate fix for the cold-start ~50% US-TX "Laser out" metastable serializer phase. */
int rtl960x_c2_cmu_settle_ms;

/* A/B knob (gpon.serdes_clkgate_rstb): 1 = gate the SerDes word clock (STOP_CLK=1)
 * across the DIG_1D interface reset-B release and un-gate LAST, so the word divider
 * restarts on one defined edge (defeats the async-reset-on-running-divider ~50%
 * serializer-phase coin-flip). 0 = legacy free-running release. Cold-start fix candidate. */
int rtl960x_c2_clkgate_rstb;

/* A/B knob (gpon.serdes_skip_rstb_dance): live debug confirmed WSDS_DIG_1D is ALREADY 0x1c000 (interface
 * reset-B released) before mode_set and the SDS reset does not clear it. =1 SKIPS the c2_sds_rstb
 * dance (DIG_00=0xf30 + the DIG_1D[15/16] assert->0/release->1) entirely — issuing it is a gratuitous
 * TX/RX reset-B 1->0->1 pulse on a running serializer (async-reset-on-running-divider phase latch).
 * Stock rev-A bring-up never pulses it. Cold-start determinism fix candidate. 0 = legacy dance. */
int rtl960x_c2_skip_rstb_dance;

/* A/B knob set by the board (gpon.serdes_sds_cfgrst). Default 0 = pulse ONLY
 * CMD_SDS_RST_PS bit0 in the SerDes reset (stock rev-A = the cold-start fix); 1 =
 * also assert CMD_SDS_CFG_RST_PS bit7 (legacy, leaves the extra reset domain
 * latched through bring-up -> per-power-on US-TX phase re-roll). */
int rtl960x_c2_sds_cfgrst;

static const struct r960_op c2_sds_mode[] = {
	FLD(C2_SDS_ANA_MISC_REG02, 13, 13, 1),	/* FRC_BER_NOTIFY_VAL = 1         */
	FLD(C2_SDS_ANA_MISC_REG02, 12, 12, 1),	/* FRC_BER_NOTIFY_ON  = 1         */
	DLY(10),
	FLD(C2_SDS_CFG, 4, 0, C2_SDS_MODE_GPON),/* select GPON mode (very last)   */
	DLY(50),
};

/* Post-GPON-mode TX-interface reset-B re-sync (DIG_1D[16] 0->1) — stock rev-A
 * OMITS this; gated by rtl960x_c2_postmode_perturb. */
static const struct r960_op c2_sds_txresync[] = {
	FLD(C2_WSDS_DIG_1D, 16, 16, 0),		/* TX interface reset-B 0         */
	DLY(2),
	FLD(C2_WSDS_DIG_1D, 16, 16, 1),		/* TX interface reset-B 1         */
	DLY(10),
};

/* A/B knob (gpon.serdes_minimal_analog): skip the golden-table writes that the stock rev-A
 * GPON bring-up does NOT do (verified against stock) — the 3 DUPLICATE GPON
 * per-rate banks and the 4 FIB-bank bodies (~134 of ~145 writes). Stock leaves these at HW
 * state; they are redundant and lengthen the bring-up with ~134 extra bus transactions before
 * the CMU/serializer phase latches. The active GPON bank (0x22708) + the FIB PDOWN-clear are
 * kept. Cold-start determinism fix candidate (makes the bring-up timing stock-minimal). */
int rtl960x_c2_minimal_analog;

/* Registers the c7 SerDes/fiber DIAGNOSTIC reads.  They were declared inside
 * the (now removed) EPON section, which is why a GPON-only build could not
 * link: the two sections were never as separate as their banners claimed. */
#define C7_SDS_FIB_STATUS	0x1b00028cu
#define C7_GTC_DS_LOS		0x1b701040u
#define C7_FIB_REG16		0x1b040c40u

/* Program the full analog CMU/CDR golden table + clear fiber power-down on every
 * FIB bank. Factored so it can run either BEFORE the SDS reset (legacy) or AFTER it
 * (stock rev-A, the cold-start determinism fix) per rtl960x_c2_analog_postreset. */
static void c2_program_analog(const struct rtl960x_ops *o)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(c2_analog); i++) {
		if (rtl960x_c2_minimal_analog && c2_off_overconfig(c2_analog[i].off))
			continue;	/* stock rev-A never writes these (over-configure) */
		o->wr(c2_analog[i].off, c2_analog[i].val);
	}
	for (i = 0; i < ARRAY_SIZE(c2_fib_reg0_banks); i++)
		o->wr(c2_fib_reg0_banks[i],
		      o->rd(c2_fib_reg0_banks[i]) & ~C2_FIB_REG0_PDOWN);
}

static int rtl9602c_ponmac_mode_set(const struct rtl960x_ops *o,
				    int rev, int subtype)
{
	unsigned int i;
	int ret;

	(void)rev; (void)subtype;	/* single rev-A SerDes variant */

	/* Steps 1 + 3 share a write phase around step 2 (the analog table). */
	ret = r960_run(o, c2_sds_pre, ARRAY_SIZE(c2_sds_pre));
	if (ret)
		return ret;

	/* Step 2 (LEGACY placement): program the FULL analog block + turn fiber power
	 * on BEFORE the reset. The SDS reset wipes analog to defaults, so by default
	 * (rtl960x_c2_analog_postreset=1) this is SKIPPED and the analog is programmed
	 * post-reset below (stock rev-A order = the cold-start determinism fix). */
	if (!rtl960x_c2_analog_postreset)
		c2_program_analog(o);

	/* Step 3: pulse the SDS reset (by default the analog is programmed AFTER it,
	 * not latched by it; see rtl960x_c2_analog_postreset). Stock pulses ONLY bit0
	 * (CMD_SDS_RST_PS); legacy also asserted bit7 (CMD_SDS_CFG_RST_PS) which then
	 * stays RMW-latched through bring-up. Apply bit7 only when explicitly enabled. */
	if (rtl960x_c2_sds_cfgrst)
		rtl960x_rfwr(o, C2_SW_SOFTWARE_RST, 7, 7, 1);	/* legacy CMD_SDS_CFG_RST_PS */
	ret = r960_run(o, c2_sds_reset, ARRAY_SIZE(c2_sds_reset));
	if (ret)
		return ret;

	/* Step 2 (STOCK rev-A placement, DEFAULT): program the FULL analog CMU/CDR
	 * golden table + clear fiber power-down NOW, AFTER the reset — so the CMU
	 * charge-pump/LDO/tank + GPON CDR hold their FINAL operating-point values before
	 * the CMU re-locks and before the RX_EN 0->1 start edge (c2_sds_rx_arm). This is
	 * the cold-start determinism fix (stock rev-A order: the SDS reset runs first,
	 * then the ModeV1 path programs the analog). Gated by rtl960x_c2_analog_postreset (default 1). */
	if (rtl960x_c2_analog_postreset)
		c2_program_analog(o);

	/* Step 4: force the 125M ref clock, OPTIONALLY let the TX CMU PLL lock to it,
	 * then release the interface reset-B lines. The TX serializer phase is latched
	 * at the TX reset-B 0->1 edge (last writes of c2_sds_rstb); if the CMU has not
	 * yet locked to the freshly-forced ref the captured phase is metastable
	 * (cold-start ~50% "Laser out"). A CMU-lock settle therefore MUST be inserted
	 * HERE — between the ref-force and the reset-B release — not at the end of
	 * mode_set (by then the phase is already latched). Gated by
	 * rtl960x_c2_cmu_settle_ms (default 0 = legacy: no extra settle). */
	if (rtl960x_c2_skip_rstb_dance) {
		/* SKIP the dance entirely. Observed: DIG_1D is already 0x1c000 (interface
		 * reset-B released) and DIG_00 already 0xf30 here, and the SDS reset does not
		 * clear them — so the assert->release would be a gratuitous TX/RX reset-B
		 * 1->0->1 pulse on a running serializer; the stock rev-A bring-up never pulses
		 * it. Leave both registers at their already-operational values (no edge). */
		pr_info("rtl9602c-gpon: skip interface reset-B dance (DIG_1D=0x%x already released)\n",
			o->rd(C2_WSDS_DIG_1D));
	} else if (rtl960x_c2_clkgate_rstb) {
		/* SYNCHRONOUS clock-gated reset-B release (cold-start metastability fix
		 * candidate). Legacy releases the DIG_1D[14:16] interface reset-B with the
		 * word-divider clock FREE-RUNNING (STOP_CLK=0 in DIG00_RUN=0xf30) — the
		 * textbook async-reset-on-a-running-divider that latches a metastable ~50%
		 * serializer word-phase. Here we GATE the word clock (STOP_CLK=1, 0xf31)
		 * across the whole reset-B dance and UN-GATE it LAST, so the divider always
		 * restarts on ONE defined edge. Final state (DIG00=0xf30, DIG_1D=0x1c000) is
		 * identical to legacy -> O5 register config stays byte-identical. */
		o->wr(C2_WSDS_DIG_00, C2_WSDS_DIG00_RUN | 1u);	/* STOP_CLK=1 (gate, 0xf31) */
		ret = r960_run(o, c2_sds_rstb + 1, ARRAY_SIZE(c2_sds_rstb) - 1); /* reset-B dance, gated */
		if (ret)
			return ret;
		o->wr(C2_WSDS_DIG_00, C2_WSDS_DIG00_RUN);	/* STOP_CLK=0 (un-gate LAST, 0xf30) */
		mdelay(10);
	} else {
		ret = r960_run(o, c2_sds_rstb, 1);	/* WR DIG_00 = RUN (125M ref forced) */
		if (ret)
			return ret;
		if (rtl960x_c2_cmu_settle_ms)
			mdelay(rtl960x_c2_cmu_settle_ms);
		ret = r960_run(o, c2_sds_rstb + 1, ARRAY_SIZE(c2_sds_rstb) - 1); /* reset-B dance */
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(c2_fib_reg0_banks); i++)
		o->wr(c2_fib_reg0_banks[i],
		      o->rd(c2_fib_reg0_banks[i]) & ~C2_FIB_REG0_PDOWN);

	/* Steps 5 + 6: burst-enable + arm the RX-CDR. */
	ret = r960_run(o, c2_sds_rx_arm, ARRAY_SIZE(c2_sds_rx_arm));
	if (ret)
		return ret;

	/* Step 6b + TX drive (modev1/tx_xtra defaults baked off). */
	ret = r960_run(o, c2_sds_tx, ARRAY_SIZE(c2_sds_tx));
	if (ret)
		return ret;

	/* Re-apply the live-stock post-reset SDS_ANA values that the golden table set
	 * BEFORE the SDS reset (which wipes them): REG01 (0x22584)=0x73a4 (CMU bit14=1,
	 * BEN_TTL_OUT bit0=0) + REG11 (0x225ac) RX_FILT_CONFIG=0. Done HERE, post-reset
	 * and BEFORE the CFG_SDS_MODE=GPON commit below, so the serializer LOCKS with the
	 * stock-good analog config. This is the ONLY stock-vs-ours SerDes diff (cold-start
	 * ~50% US-TX "Laser out"); bit14 is in the shared CMU block. Gated by
	 * rtl960x_c2_stock_analog (default 1 = fix). */
	if (rtl960x_c2_stock_analog) {
		rtl960x_rfwr(o, 0x1B022584u, 14, 14, 1);	/* REG01 CMU bit14 = 1 (stock) */
		rtl960x_rfwr(o, 0x1B022584u,  0,  0, 0);	/* REG01 BEN_TTL_OUT = 0 (stock) */
		rtl960x_rfwr(o, 0x1B0225ACu,  7,  0, 0);	/* REG11 RX_FILT_CONFIG = 0 (stock) */
	}

	/* Step 7a + 7b: force-SD + commit GPON mode. */
	ret = r960_run(o, c2_sds_mode, ARRAY_SIZE(c2_sds_mode));
	if (ret)
		return ret;

	/* The TX reset-B re-sync + the post-mode serdesCdr_reset pulse are the two
	 * perturbations stock rev-A omits; do them only when explicitly enabled. */
	if (rtl960x_c2_postmode_perturb) {
		ret = r960_run(o, c2_sds_txresync, ARRAY_SIZE(c2_sds_txresync));
		if (ret)
			return ret;
		rtl9602c_serdes_cdr_reset(o);
	}

	/* Keep the MAC clock ungated. */
	rtl960x_rfwr(o, C2_WSDS_DIG_00, 0, 0, 0);

	/* Wait for the analog to report ready (FIB_EXT_REG21 bit13); ~200 ms cap.
	 * Return the poll result, matching the stock SerDes-init final return. */
	return r960_run(o, (const struct r960_op[]){
		POLL(C2_FIB_EXT_REG21, C2_SDS_ANALOG_READY, C2_SDS_LOCK_POLL_MAX),
	}, 1);
}

void rtl960x_c7_diag(const struct rtl960x_ops *o, struct seq_file *s)
{
	u32 fib0, com09, fib21, sds_sts, gtc_los, sds_cfg, wsd18, fib16;

	if (!o || !o->rd)
		return;

	fib0    = o->rd(C7_FIB_REG0);
	com09   = o->rd(C7_SDS_ANA_COM09);
	fib21   = o->rd(C7_FIB_EXT_REG21);
	sds_sts = o->rd(C7_SDS_FIB_STATUS);
	gtc_los = o->rd(C7_GTC_DS_LOS);
	sds_cfg = o->rd(C7_SDS_CFG);
	wsd18   = o->rd(C7_WSDS_DIG_18);
	fib16   = o->rd(C7_FIB_REG16);

	seq_printf(s,
		"c7_sd: fib_reg0=0x%08x com09=0x%08x fib21=0x%08x sds_fib_sts=0x%08x gtc_los=0x%08x sds_cfg=0x%08x wsd18=0x%08x fib_reg16=0x%08x\n",
		fib0, com09, fib21, sds_sts, gtc_los, sds_cfg, wsd18, fib16);
	seq_printf(s,
		"c7_sd: sds_sdet=%u fib100_sdet=%u link_ok=%u analog_ready=%u optic_los=%u frc_los=%u sel_rx_sd=%u frc_sd=%u\n",
		!!(sds_sts & BIT(17)), !!(sds_sts & BIT(2)), !!(sds_sts & BIT(4)),
		!!(fib21 & BIT(13)), !!(gtc_los & BIT(8)),
		!!(wsd18 & BIT(14)), !!(fib16 & BIT(2)), !!(fib16 & BIT(10)));
}

/* ---- dispatch --------------------------------------------------------- */
int rtl960x_ponmac_init(enum rtl960x_chip chip, int rev, int subtype,
			const struct rtl960x_ops *o)
{
	(void)rev; (void)subtype;	/* per-chip init takes (o) only */
	switch (chip) {
	case RTL960X_CHIP_9602C:
		return rtl9602c_ponmac_init(o);
	case RTL960X_CHIP_9603CVD:
		return rtl9603cvd_ponmac_init(o);
	case RTL960X_CHIP_9607C:
		return rtl9607c_ponmac_init(o);
	default:
		return -ENOTSUPP;
	}
}

int rtl960x_ponmac_mode_set(enum rtl960x_chip chip, int rev, int subtype,
			    const struct rtl960x_ops *o)
{
	switch (chip) {
	case RTL960X_CHIP_9602C:
		return rtl9602c_ponmac_mode_set(o, rev, subtype);
	case RTL960X_CHIP_9603CVD:
		return rtl9603cvd_ponmac_mode_set(o, rev, subtype);
	case RTL960X_CHIP_9607C:
		return rtl9607c_ponmac_mode_set(o, rev, subtype);
	default:
		return -ENOTSUPP;
	}
}

int rtl960x_ponmac_serdes_cdr_reset(enum rtl960x_chip chip,
				    const struct rtl960x_ops *o)
{
	switch (chip) {
	case RTL960X_CHIP_9602C:
		return rtl9602c_serdes_cdr_reset(o);
	case RTL960X_CHIP_9603CVD:
		return rtl9603cvd_serdes_cdr_reset(o);
	case RTL960X_CHIP_9607C:
		return rtl9607c_serdes_cdr_reset(o);
	default:
		return -ENOTSUPP;
	}
}
