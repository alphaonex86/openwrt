// SPDX-License-Identifier: GPL-2.0
/* TIER: CHIP — the hardware shell for exactly ONE part: ...
 * dev/MEASURED-cortina-gpon.c.md sec 1. */

#include <linux/module.h>
#include "cortina_gpon_logic.h"	/* hoisted logic */
#include <linux/kernel.h>
#include <linux/crc32.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_net.h>	/* of_get_mac_address() -- the DT rung of the ladder */
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/ktime.h>	/* the far-end capture timestamps its records */
#include <linux/math64.h>	/* div_u64: ns -> us without a 64-bit divide */
#include <linux/ratelimit.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "cortina-gpon-serdes.h"
#include "cortina-gpon-bosa.h"
#include "cortina-gpon-ddm.h"	/* SFF-8472 A2h optical decode (functional core) */
#include "gpon_sn.h"	/* the common G.984.3 ONU-SN codec */
#include "gpon_range_gate.h"	/* may this ONU transmit, and what to change */
#include "gpon_hwaddr.h"	/* the common station-address ladder */
#include "cortina-access.h"	/* the ONE indirect transaction */
#include "gpon_ind_rmw.h"	/* the core's ACCESS/DATA entry read-modify-write */
#include "cortina-ni.h"		/* cortina_ni_pon_rx_hook_set + cortina_ni_pon_tx */

/* ★★ CUT SITE — the G.988 OMCI responder MOVED OUT (code ...
 * dev/MEASURED-cortina-gpon.c.md sec 2. */
#include "gpon_omci_core.h"	/* G.988 message layer: omci_onu_input_ex()   */
#include "gpon_omci_me.h"	/* G.988 ME model: struct omci_onu, the store */
#include "gpon_omci_trace.h"	/* G.988 decode-to-a-buffer for the log    */
#include "gpon_omci_diag.h"	/* the ONE per-PDU trace line, every family */
#include "gpon_olt_diag.h"	/* the far-end capture: replayed off the board */
#include "gpon_omci_mic.h"	/* G.984.4 MIC dialect: gpon_omci_mic_conv() */
#include "gpon_gem_us.h"	/* upstream GEM/T-CONT mapping + bind verdict */
#include "gpon_data_plan.h"	/* armed-vs-provisioned reconcile + undo verdict */

#define DRV_NAME		"cortina-gpon"

/* PON register window (from the DT reg entry / SDK): phys 0x4_F5500000, 48 KiB. */
#define CG_PON_WINDOW_PHYS	0x4f5500000ULL
#define CG_PON_WINDOW_SIZE	0xc000

/* The GPON MAC register block sits at window + 0x6000 (aal_pon.h). */
#define CG_GPON_MAC_OFF		0x6000

/* PON-SerDes (PSDS) registers, direct within the PON window. ...
 * dev/MEASURED-cortina-gpon.c.md sec 3. */
#define CG_PSDS_MODE		0xa02c
#define CG_PSDS_RGB8		0xa05c	/* DS-lock status; locked = (val & 0x9c01)==0x9c00 (stock 0x19c00) */
/* The TWO lock predicates that register answers, kept ...
 * dev/MEASURED-cortina-gpon.c.md sec 4. */
#define CG_PSDS_GBOX_CTRL	0xa060	/* vendor PSDS_GBOX_CTRL (stock 0x454) */
#define CG_PSDS_PRBS_CTRL	0xa064	/* vendor PSDS_PRBS_CTRL (stock 0) */
#define CG_PSDS_PRBS_INTR	0xa068	/* vendor PSDS_PRBS_INTR (stock 1) */
#define CG_PSDS_PRBS_STS	0xa070	/* vendor PSDS_PRBS_STS  (stock 1) */

#define CG_PSDS_DS_LOCK_MASK	0x9c01u
#define CG_PSDS_DS_LOCK_VAL	0x9c00u
#define CG_PSDS_CMU_LOCK_MASK	0x8c01u
#define CG_PSDS_CMU_LOCK_VAL	0x8c00u
#define CG_PSDS_GBOX_CTRL	0xa060	/* rx/tx bit-ordering[7:4]; stock=0x454.  WAS 0xa064 (stock=0) -> our US tx_bit_ordering never took -> OLT saw US LOS (live-diff 2026-07-13) */
#define CG_PON_EPON_SPARE	0x01c8	/* EPON_GLB_SPARE_CFG (PON window); bit31 for GPON los-rst */
/* PSDS internal analog-register indirect interface (the ...
 * dev/MEASURED-cortina-gpon.c.md sec 5. */
#define CG_PSDS_IND_CMD		0xa088		/* stock reg.txt: PSDS_REG_ACCESS */
#define CG_PSDS_IND_WDATA	0xa08c		/* stock reg.txt: PSDS_REG_DATAIN */
#define CG_PSDS_IND_RDATA	0xa090		/* stock reg.txt: PSDS_REG_DATAOUT */
#define CG_PSDS_IND_READ	0x80000000u	/* command: strobe, read */
#define CG_PSDS_IND_WRITE	0xc0000000u	/* command: strobe, write */
#define CG_PSDS_CMU_IDX		0x400		/* analog reg; [7:4] = re-lock strobe.  "CMU" is OUR label:
						 * the vendor calls index 0x400 (page 0x20, num 0) ANA_MISC_REG00
						 * and its walk a CDR power-down/rx_en cycle (aal_psds.c
						 * __psds_cdr_reset, tier 3, one source) -- see cg_psds_relock() */

/*
 * GLB (global) PON/GPON reset & clock control window: phys 0x4_F4320000, 4 KiB.
 * On our minimal build the GPON MAC reads garbage (block held in reset); the
 * vendor aal_gpon glb-reset clocks it.  Offsets + released values measured on
 * live stock (the block reads "XPON" with these):
 *   EPON_CNTL(+0x078)=0x00030000  GPON_CNTL(+0x080)=0x00000003  PON_CNTL(+0x09c)=0x0000030e
 * GPON_CNTL bits: ani_rst_n[0], gpon_rst_n[1].  PON_CNTL bits: pon_serdes_rst_n[1],
 * psds_reg_rst_n[2], ptp_rst_n[3], puc_reset[8], pdc_reset[9].
 */
#define CG_GLB_WINDOW_PHYS	0x4f4320000ULL
#define CG_GLB_WINDOW_SIZE	0x1000
#define CG_GLB_EPON_CNTL	0x078
#define CG_GLB_GPON_CNTL	0x080
#define CG_GLB_PON_CNTL		0x09c
/* PON interrupt aggregation, level 1 of 2 (GLB window). The ...
 * dev/MEASURED-cortina-gpon.c.md sec 6. */
#define CG_GLB_PON_INT0		0x1b0	/* GLOBAL_PON_INTERRUPT_0 */
#define CG_GLB_PON_INTEN0	0x1b4	/* GLOBAL_PON_INTENABLE_0 */
#define CG_PON_INT0_PON_MAC	BIT(0)	/* PON_MACi/e */
/* PON interrupt aggregation, level 2 of 2: the NE global ...
 * dev/MEASURED-cortina-gpon.c.md sec 7. */
#define CG_GLB_NE_ICTL_STS	0x194	/* per-ictl STATUS  (ne_ictl) */
#define CG_GLB_NE_ICTL_EN	0x198	/* per-ictl ENABLE  (ne_ictl) */
#define CG_NE_ICTL_PON_LINE	BIT(5)	/* PON = ne_ictl line 5 */
/* GLOBAL_PSDS_INIT_CNTL: bit5 POW_PCIX powers the PON-SerDes ...
 * dev/MEASURED-cortina-gpon.c.md sec 8. */
#define CG_GLB_PSDS_INIT	0x25c
#define CG_PSDS_POW_PCIX	BIT(5)
#define CG_PSDS_BEN_OEN		BIT(4)

/* Laser TX-disable GPIO. The GN25L95's hardware TX_DIS input ...
 * dev/MEASURED-cortina-gpon.c.md sec 9. */
#define CG_PERGPIO_PHYS		0x4f4329000ULL
#define CG_PERGPIO_SIZE		0x1000
#define CG_GLB_GPIO_MUX1	0x134
#define CG_PERGPIO_CFG1		0x324
#define CG_PERGPIO_OUT1		0x328
#define CG_PERGPIO_IN1		0x32c
#define CG_LASER_PIN34		BIT(2)
/* ★ The REAL laser-enable path (live golden diff ... -- dev/MEASURED-cortina-gpon.c.md sec 10. */
#define CG_PERGPIO_CFG0		0x300
#define CG_PERGPIO_OUT0		0x304
#define CG_GLB_GPIO_MUX0	0x130	/* stock 0x00001FFF: pins 0-12 are GPIO */
#define CG_GLB_GPIO_MUX3	0x13c	/* stock 0x000390FF: pins 96-103,108,111-113 */
#define CG_GLB_GPIO_MUX4	0x140	/* stock 0x00003B00: pins 136,137,139-141 */
#define CG_GLB_PINROUTE		0x42c	/* stock 0x01101101; bit20 = laser net */
#define CG_PINROUTE_LASER	BIT(20)
#define CG_GPIO0_LASER_PINS	(BIT(6) | BIT(11) | BIT(12))
#define CG_GPIO0_PIN6		BIT(6)
/* GPIO groups 3 and 4 (PERI +0x36c/+0x370, +0x390/+0x394): ...
 * dev/MEASURED-cortina-gpon.c.md sec 11. */
#define CG_PERGPIO_CFG3		0x36c
#define CG_PERGPIO_OUT3		0x370
#define CG_PERGPIO_CFG4		0x390
#define CG_PERGPIO_OUT4		0x394
#define CG_GPIO3_CFG_STOCK	0xfffdef00
#define CG_GPIO3_OUT_STOCK	0x00021010
#define CG_GPIO4_CFG_STOCK	0xffffc5ff
#define CG_GPIO4_OUT_STOCK	0x00003200

/* GPON MAC register offsets within the block ... -- dev/MEASURED-cortina-gpon.c.md sec 12. */
#define CG_REG_GPON_DS		0x000	/* DS framer thresholds; max_packet_size low */
#define CG_REG_US		0x00c	/* us: frame_var[8:0], eqd_select[16] */
#define CG_REG_SIGNAL		0x010	/* SF/SD BER alarm thresholds */
#define CG_REG_VENDOR		0x014	/* vendor-id (ASCII "XPON") */
#define CG_REG_VENDOR_SPEC	0x018	/* vendor-specific serial number */
#define CG_REG_ALARM		0x09c	/* hdr 0x7c: LOS/LOF alarm bits (live levels) */

/* Interrupt block: header 0x84..0xa8 -> SILICON 0xa4..0xc8 ...
 * dev/MEASURED-cortina-gpon.c.md sec 13. */
#define CG_REG_INT_TOP		0x0a4	/* hdr 0x84: interrupt_top (read-clear) */
#define CG_REG_INT_TOP_EN	0x0a8	/* hdr 0x88: int_top_en */
#define CG_REG_INT		0x0ac	/* hdr 0x8c: INTERRUPT  status (W1C) - operational */
#define CG_REG_INT_EN		0x0b0	/* hdr 0x90: INTERRUPT  enable */
#define CG_REG_INT2		0x0b4	/* hdr 0x94: INTERRUPT2 status (W1C) - TC/parse err */
#define CG_REG_INT2_EN		0x0b8	/* hdr 0x98: INTERRUPT2 enable */
#define CG_REG_INT3		0x0bc	/* hdr 0x9c: INTERRUPT3 status (W1C) - negedge/MSB */
#define CG_REG_INT3_EN		0x0c0	/* hdr 0xa0: INTERRUPT3 enable */
#define CG_REG_INT4		0x0c4	/* hdr 0xa4: INTERRUPT4 status (W1C) - FEC MSB */
#define CG_REG_INT4_EN		0x0c8	/* hdr 0xa8: INTERRUPT4 enable */

#define CG_INT_TOP_EN_ALL	0xF		/* vendor GPON_MAC_GPON_INT_TOP_ENA_DEF */
/* vendor GPON_MAC_GPON_INT_ENA_DEF 0xC00AFFFF + bit27; stock at O5 reads
 * 0xC80AFFFF.  bit27 is reserved in the (older) rtl8277c header but is a real
 * source on this silicon: DS-PLOAM message received — the stock __intr_handler
 * keys the Extended_Burst_Length -> us.frame_var recompute off it. */
#define CG_INT_EN_DEFAULT	0xC80AFFFF	/* int2/3/4 enables = 0 */

/* INTERRUPT source bits we service (alarm bits 0..15 are event/diag) */
#define CG_INT_ONU_ST_CHG	BIT(31)	/* ONU activation-FSM state changed */
#define CG_INT_ONU_ID		BIT(30)	/* Assign_ONU-ID accepted -> bind OMCC T-CONT */
#define CG_INT_PLOAMD		BIT(27)	/* DS PLOAM msg received (recheck frame_var) */
#define CG_INT_KSW		BIT(19)	/* Key_Switching_Time (AES rekey; next phase) */
#define CG_INT_PORTID		BIT(17)	/* Configure_Port-ID -> omci_port valid, bind OMCC GEM */
#define CG_INT_DACT		BIT(8)	/* Deactivate_ONU-ID */

#define CG_REG_GPON_ONU		0x0dc	/* hdr 0xbc: ONU id[7:0], state[18:16]; dft id=0xff */
#define CG_ONU_ID(v)		((v) & 0xff)
#define CG_ONU_STATE(v)		(((v) >> 16) & 0x7)
#define CG_ONU_ID_NONE		0xff	/* reset default = unassigned */
/* onu.state encoding (vendor aal_gpon.h): 0=O1 Initial, 1=O2 ...
 * dev/MEASURED-cortina-gpon.c.md sec 14. */

#define CG_REG_GPON_MAIN	0x0e0	/* hdr 0xc0: equalization delay (EqD) */
#define CG_REG_OMCI_PORT	0x0e8	/* hdr 0xc8: omci_port id[11:0], en[12]; HW-filled */
/* The id field of that register IS the on-wire GEM Port-ID, so its width is
 * the core's fact and not this register map's: the name stays here beside
 * CG_OMCI_PORT_EN, the 12-bit constant does not (gpon_gem_us.h). */
#define CG_OMCI_PORT_ID(v)	gpon_gem_us_port_id(v)
#define CG_OMCI_PORT_EN		BIT(12)
#define CG_REG_T3_PREAMBLE	0x0f8	/* hdr 0xd8: extend[16], ranged[15:8], pre_range[7:0];
					 * HW-latched from the OLT's Extended_Burst_Length PLOAM */

/*
 * Indirect table access pairs, +0x20-shifted like everything >= hdr 0x50.
 * Live-confirmed on stock at Online (TCONT_ACCESS 0x14c=0x40000101,
 * US_PORT_ID_DATA 0x194=0xDF = the OLT-assigned GEM port).
 * Protocol (vendor __GPN_*_DO_INDIRCT_OP): write ACCESS = go(bit31) | rbw(bit30,
 * 1=write) | index/alloc-id, then poll ACCESS bit31 self-clear (<= 10000 reads).
 * Data flows through the DATA register (read entry -> DATA; DATA -> write entry).
 */
/* The TX-PLOAM MIB indirect pair, vendor GPON_MAC_GPON_PLM_MIB_ACCESS/_DATA
 * (0xf5506184/_188 = mac+0x184/0x188).  Same ACCESS/DATA handshake as every
 * other indirect table here: go[31] set, poll it clear, then take DATA. */
#define CG_REG_PLM_MIB_ACCESS	0x184
#define CG_REG_PLM_MIB_DATA	0x188
#define CG_REG_TCONT_ACCESS	0x14c	/* header 0x12c: alloc_id[11:0], sw_plm_en[16], rbw[30], go[31] */
#define CG_REG_TCONT_DATA	0x150	/* header 0x130: ploam_en[0], omci_en[1], index[6:2] (hw T-CONT 0-31) */
#define CG_REG_DS_GEM_ACCESS	0x154	/* header 0x134: id[11:0] (GEM port-id), sw_aes[16], rbw[30], go[31] */
#define CG_REG_DS_GEM_DATA	0x158	/* header 0x138: vld[0], aes[1], tdm[2], index[10:3] (intern gem) */
#define CG_REG_US_PORT_ACCESS	0x190	/* header 0x170: index[7:0] (us hw gem 0-255), rbw[30], go[31] */
#define CG_REG_US_PORT_DATA	0x194	/* header 0x174: id[11:0] (GEM port-id) */
/* Last slot the upstream port-map array actually has, read ...
 * dev/MEASURED-cortina-gpon.c.md sec 122. */
#define CG_US_PORT_IDX_MAX	255

#define CG_DS_GEM_VLD		BIT(0)
#define CG_DS_GEM_INDEX(x)	(((x) & 0xff) << 3)

#define CG_TBL_GO		BIT(31)
/* Poll bounds, kept apart because they were measured apart: the table accesses
 * complete in a few bus reads, the PLM MIB strobe is a slower engine and its
 * one-shot diagnostic never had more than 1000. */
#define CG_TBL_TRIES		10000u
#define CG_MIB_TRIES		1000u
#define CG_TBL_WR		BIT(30)
#define CG_TCONT_PLOAM_EN	BIT(0)
#define CG_TCONT_OMCI_EN	BIT(1)
#define CG_TCONT_INDEX(x)	(((x) & 0x1f) << 2)
#define CG_TCONT_INDEX_MASK	(0x1f << 2)

/* The three GPON-MAC ACCESS/DATA tables as the core's struct ...
 * dev/MEASURED-cortina-gpon.c.md sec 15. */
#define CG_MAC_IND_TBL(acc, dat) \
	{ .access = REG_AT(acc), .data = REG_AT(dat), .go = CG_TBL_GO, \
	  .wr = CG_TBL_WR, .tries = CG_TBL_TRIES }
static const struct gpon_ind_tbl cg_tcont_cam_tbl =
	CG_MAC_IND_TBL(CG_REG_TCONT_ACCESS, CG_REG_TCONT_DATA);
static const struct gpon_ind_tbl cg_us_port_tbl =
	CG_MAC_IND_TBL(CG_REG_US_PORT_ACCESS, CG_REG_US_PORT_DATA);
static const struct gpon_ind_tbl cg_ds_gem_tbl =
	CG_MAC_IND_TBL(CG_REG_DS_GEM_ACCESS, CG_REG_DS_GEM_DATA);

#define CG_OMCC_US_GEM_IDX_NUM	8	/* vendor AAL_GPON_OMCI_RSV_PORT_MAX: us hw gems 0..7 = OMCC */

/* Stage D — the WAN data path. ONE data T-CONT + ONE ...
 * dev/MEASURED-cortina-gpon.c.md sec 16. */
#define CG_OMCC_TCONT_IDX	0	/* hw T-CONT the OMCC alloc-id is bound to */
#define CG_DATA_TCONT_IDX	1	/* hw T-CONT of the OLT's data alloc-id */
#define CG_DATA_GEM_IDX		(CG_DATA_TCONT_IDX * 8)	/* intern gem idx = VoQ 8 */
#define CG_MCAST_GEM_IDX	(CG_DATA_GEM_IDX + 1)	/* DS-only broadcast GEM */
#define CG_MCAST_GEM_ID		4095	/* G.984 broadcast GEM port-id */

/* PDC (packet-downstream classifier) sub-block: PON window + ...
 * dev/MEASURED-cortina-gpon.c.md sec 17. */
#define CG_PDC_CTRL		0x9014	/* dft 0x2 (pdc_map_mem_en) */
#define CG_PDC_CTRL_MAP_MEM_EN	BIT(1)
#define CG_PDC_CTRL_HP_COS_SH	16	/* omci_hp_cos[18:16] */
#define CG_PDC_CTRL_HP_LDPID_SH	19	/* omci_hp_ldpid[24:19] */
#define CG_PDC_CTRL_HP_EN	BIT(25)	/* omci_hp_en */
#define CG_PDC_CTRL_HP_MASK	GENMASK(25, 16)
#define CG_PDC_MAP_ACCESS	0x9020	/* address[7:0], rbw[30], go[31] */
#define CG_PDC_MAP_DATA1	0x9024	/* pol_en[3:2], pol_id[12:4], pol_grp_id[15:13], deepq[16] */
#define CG_PDC_MAP_DATA0	0x9028	/* cos[2:0], ldpid[8:3], lspid[14:9], fe_bypass[15], no_drop[31] */
#define CG_PDC_MAP_ENTRIES	256	/* vendor AAL_PDC_MAP_ENTRY_NUM */
/* The DATA-word field layout (CG_PDC_D0_*, CG_PDC_D1_POL_ID) ...
 * dev/MEASURED-cortina-gpon.c.md sec 18. */
#define CG_PUC_BASE		0x8000	/* PON window + 0x8000 */
#define CG_PUC_PVTBL_ACCESS	(CG_PUC_BASE + 0x000)	/* addr[5:0]=T-CONT, rbw[30], go[31] */
#define CG_PUC_PVTBL_DATA4	(CG_PUC_BASE + 0x004)
#define CG_PUC_PVTBL_DATA3	(CG_PUC_BASE + 0x008)
#define CG_PUC_PVTBL_DATA2	(CG_PUC_BASE + 0x00c)	/* voq7[7:0], schmode[8], entryvld[12], wrr0/1 */
#define CG_PUC_PVTBL_DATA1	(CG_PUC_BASE + 0x010)	/* voq3[3:0],voq4,voq5,voq6,voq7[31] */
#define CG_PUC_PVTBL_DATA0	(CG_PUC_BASE + 0x014)	/* voq0,voq1,voq2,voq3[31:27] */
#define CG_PUC_VOQMAPCFG	(CG_PUC_BASE + 0x04c)	/* voqmapsel[1:0]: 0 = 8Q mode */
#define CG_PUC_BTCCFG		(CG_PUC_BASE + 0x050)
#define CG_PUC_PUCCFG		(CG_PUC_BASE + 0x08c)	/* dft 0x84040001 */
#define CG_PUC_VOQBUFLIMSEL0	(CG_PUC_BASE + 0x090)	/* 16 regs, stride 4 (0x090..0x0cc) */
#define CG_PUC_VOQBUFLIMSEL_N	16
#define CG_PUC_VOQBUFLIMIT_A	(CG_PUC_BASE + 0x0d0)
#define CG_PUC_VOQBUFLIMIT_B	(CG_PUC_BASE + 0x0d4)
#define CG_PUC_VOQBUFLIMIT_C	(CG_PUC_BASE + 0x0d8)
#define CG_PUC_BPCNTL		(CG_PUC_BASE + 0x0e4)	/* bpen[0], dropen[4], bpth[30:16] */
#define CG_PUC_VOQBPREMAP_ACCESS (CG_PUC_BASE + 0x0e8)	/* addr[7:0]=VoQ, rbw[30], go[31] */
#define CG_PUC_VOQBPREMAP_DATA	(CG_PUC_BASE + 0x0ec)	/* tqmvoqid[7:0] */
#define CG_PUC_PONCNTL_INTEN	(CG_PUC_BASE + 0x0f4)
#define CG_PUC_CTRL		(CG_PUC_BASE + 0x13c)	/* dft 0x3300007c; shp_en[30], rl_en[26] */
#define CG_PUC_CTRL1		(CG_PUC_BASE + 0x140)	/* rlovhd[4:0], shpovhd[9:5], agrshpovhd[14:10] */
#define CG_PUC_CTRL2		(CG_PUC_BASE + 0x144)	/* dft 0x03000000; pirovhd[4:0], pir_en[26] */
#define CG_PUC_VOQFLUSH		(CG_PUC_BASE + 0x0dc)	/* voqid[7:0], tcontid[12:8], openpktflushen[16], start[31] */
#define CG_PUC_VALID_VOQ0	(CG_PUC_BASE + 0x1bc)	/* valid_voqN = VALID_VOQ0 - (voq/32)*4 */
#define CG_PUC_Q2PQSRCFG01	(CG_PUC_BASE + 0x230)	/* qm_rpt_lv0[15:0], lv1[31:16] */
#define CG_PUC_Q2PQSRCFG23	(CG_PUC_BASE + 0x234)	/* qm_rpt_lv2[15:0], lv3[31:16] */
#define CG_PUC_BMC_RX_PKT	(CG_PUC_BASE + 0x17c)	/* US frames received by the PUC */
#define CG_PUC_BMC_RX_PKT_ENQ	(CG_PUC_BASE + 0x180)	/* US frames enqueued to a VoQ */
#define CG_PUC_BMC_FORCE_DROP	(CG_PUC_BASE + 0x184)	/* US frames dropped (invalid VoQ) */
#define CG_PUC_US_OMCI_HDR_A	(CG_PUC_BASE + 0x160)	/* gemid[7:0],cos[10:8],tcont[21:16],datapkt[30],en[31] */
#define CG_PUC_US_OMCI_HP_HDR_A	(CG_PUC_BASE + 0x164)	/* gemid[7:0],cos[10:8],tcont[21:16] */
#define CG_PUC_GLOBAL_PLOAM_CFG	(CG_PUC_BASE + 0x168)	/* us_hdr_min_size[21:16], us_ext_omci_en[31] */

/* The PUC's CONTROL-PACKET classifier and its two dedicated ...
 * dev/MEASURED-cortina-gpon.c.md sec 19. */
#define CG_PUC_GLOBAL_DA_SA2	(CG_PUC_BASE + 0x14c)	/* DA[0..3] */
#define CG_PUC_GLOBAL_DA_SA1	(CG_PUC_BASE + 0x150)	/* DA[4..5], SA[0..1] */
#define CG_PUC_GLOBAL_DA_SA0	(CG_PUC_BASE + 0x154)	/* SA[2..5] */
#define CG_PUC_GLOBAL_MAC_TYPE	(CG_PUC_BASE + 0x158)	/* type:32, dft 0xfff00000 */
#define CG_PUC_GLOBAL_LNK_TYPE	(CG_PUC_BASE + 0x15c)	/* type:32, dft 0xfff10000 (OMCI) */
#define CG_PUC_BMC_CTRL_PKT_MAC	(CG_PUC_BASE + 0x174)	/* cntr:16, MAC-type control frames */
#define CG_PUC_BMC_CTRL_PKT_LNK	(CG_PUC_BASE + 0x178)	/* cntr:16, OMCI-type control frames */
#define CG_PUC_BMC_LENGTH_ERROR	(CG_PUC_BASE + 0x188)	/* cntr:16, US length-check rejects */
#define CG_PUC_BMC_CNTR_MASK	0xffff	/* the cntr:16 fields' reserved upper half */
#define CG_PUC_LNK_TYPE_OMCI	0xfff1

/* The PUC<->US-scheduler interface control. ★ On this silicon ...
 * dev/MEASURED-cortina-gpon.c.md sec 20. */
#define CG_GPON_MAC_PUCIF_CTRL	0x6e00	/* dft 0x0040a100 */

/* GPON-MAC statistics counters, MAC-block-relative (the ...
 * dev/MEASURED-cortina-gpon.c.md sec 21. */
#define CG_REG_BIP_ERR		0x078	/* BIP-8 errors of the last superframe   */
#define CG_REG_BIP_ERR_ACCUM	0x07c	/* accumulated BIP-8 errors              */
#define CG_REG_BIP_ERR_FRAMES	0x080	/* frames over which BIP was accumulated */
#define CG_REG_DS_OMCI_GEM	0x084	/* DS OMCI GEM frames (hardware count)   */
#define CG_REG_DS_OMCI_PKT	0x088	/* DS OMCI packets (hardware count)      */
#define CG_REG_DS_PKT_CRC	0x08c	/* DS packets failing CRC                */
#define CG_REG_DS_UNDERSIZE	0x090	/* DS undersized packets                 */
#define CG_REG_DS_OVERSIZE	0x094	/* DS oversized packets                  */
#define CG_REG_SUPERFRAME	0x0fc	/* 125 us superframe counter             */
#define CG_REG_US_OMCC_CNT	0x200	/* upstream OMCC frames, GPON-MAC side.
					 * Present in this board's register map
					 * but stock never reads it, so there is
					 * NO stock oracle: treat as unvalidated
					 * until seen to move.  A second angle on
					 * the upstream-OMCI question the PUC
					 * _lnk counter leaves open. */
#define CG_REG_PUCIF_PROTECT	0xe14	/* b0 = PUCIF hang LATCHED, b5:1 = the
					 * T-CONT id that hung.  Stock's
					 * periodic monitor logs
					 * "pucif_hang_tcon_id:%d" and clears
					 * by writing 0.  We read it WITHOUT
					 * clearing: a sticky "ever hung" plus
					 * the offender's id costs nothing and
					 * cannot perturb stock-matching
					 * behaviour. */
#define CG_REG_O5		0x1a8	/* O5-related count (semantics unproven) */
#define CG_REG_GEM_FRAG_DROP	0x1ac	/* DS GEM fragments dropped              */
#define CG_REG_GEM_1BITERR	0x1b0	/* GEM header 1-bit errors (corrected)   */
#define CG_REG_GEM_2BITERR	0x1b4	/* GEM header 2-bit errors               */
#define CG_REG_GEM_UNCORR	0x1b8	/* GEM header uncorrectable errors       */
#define CG_REG_BWMAP_DROP	0x1bc	/* upstream BWmap entries dropped        */
#define CG_REG_OMCI_CRC		0x1c0	/* DS OMCI CRC failures                  */
#define CG_REG_PLEND_ERR	0x1c4	/* PLend field errors                    */
#define CG_REG_PLEND_BITERR	0x1c8	/* PLend bit errors                      */
#define CG_REG_DS_ASMBL_DROP	0x1cc	/* DS reassembly-FIFO drops              */
#define CG_REG_BWMAP_UNCORR	0x1f8	/* BWmap uncorrectable bit errors        */
#define CG_REG_BWMAP_CORR	0x1fc	/* BWmap corrected bit errors            */
/* FEC block.  The five counters have no clear function in stock, so whether they
 * self-clear is NOT established -- they are published raw and labelled accordingly
 * rather than presented as cumulative totals. */
#define CG_REG_FEC_CTRL		0x800
#define CG_REG_FEC_MISC_STATUS	0x804
#define CG_REG_FEC_CORR_BLK	0x808	/* correctable FEC blocks   */
#define CG_REG_FEC_UNCORR_BLK	0x80c	/* uncorrectable FEC blocks */
#define CG_REG_FEC_CLEAN_BLK	0x810	/* error-free FEC blocks    */
#define CG_REG_FEC_BLK_TOTAL	0x814	/* total FEC blocks         */
#define CG_REG_FEC_CORR_BYTES	0x818	/* bytes corrected by FEC   */

#define CG_PUC_TCONT_NUM	32	/* AAL_GPON_SYSTEM_MAX_TCONT_NUM */
#define CG_PUC_QUEUE_PER_TCONT	8	/* 8Q mode */
/* ^ cortina_gpon_logic.h carries the SAME define for ...
 * dev/MEASURED-cortina-gpon.c.md sec 123. */
#define CG_PUC_9TH_QUEUE_VOQ	127	/* the CPU high-prio inject VoQ (ldpid 0xf, cos 7) */

/* ★ THE TWO UPSTREAM SLOT RANGES, DECLARED ONCE (struct ...
 * dev/MEASURED-cortina-gpon.c.md sec 22. */
static const struct gpon_gem_us_range cg_us_omcc_slots = {
	.base		= 0,				/* us hw gems 0..7 */
	.count		= CG_OMCC_US_GEM_IDX_NUM,
	.index_max	= CG_US_PORT_IDX_MAX,
};
static const struct gpon_gem_us_range cg_us_data_slots = {
	.base		= CG_DATA_GEM_IDX,		/* = VoQ 8..15 */
	.count		= CG_PUC_QUEUE_PER_TCONT,
	.index_max	= CG_US_PORT_IDX_MAX,
};
static_assert(GPON_GEM_US_RANGE_OK(0, CG_OMCC_US_GEM_IDX_NUM,
				   CG_US_PORT_IDX_MAX),
	      "OMCC upstream slot range runs past the US port-map array");
static_assert(GPON_GEM_US_RANGE_OK(CG_DATA_GEM_IDX, CG_PUC_QUEUE_PER_TCONT,
				   CG_US_PORT_IDX_MAX),
	      "data upstream slot range runs past the US port-map array");
/* ★ RECORDED, NOT FIXED (2026-08-05): the two halves of the ...
 * dev/MEASURED-cortina-gpon.c.md sec 23. */
#define CG_REG_ONU_CFG_REAL	0x138
#define CG_ONU_CFG_VAL		0x12100900
/* The activation control register: the header calls it ...
 * dev/MEASURED-cortina-gpon.c.md sec 24. */
#define CG_REG_ONU_CTL		0x134
#define CG_ONU_CTL_VAL		0x00460262	/* stock O5 value: en(bit1) + defaults */
#define CG_ONU_CTL_EN		BIT(1)		/* the GO bit: HW starts ranging */
/* GPON_MAC_GPON_CTRL (hdr 0x1c4 -> silicon +0x1e4, dft ...
 * dev/MEASURED-cortina-gpon.c.md sec 25. */
#define CG_REG_GPON_MAC_CTRL	0x1e4
#define CG_MAC_CTRL_SW_RANDOM_EN BIT(16)
#define CG_MAC_CTRL_PTI_OMCI	BIT(17)

/* One observed order for IRQ snapshots and unaccepted baseline OMCI frames. */
#define CG_EVT_IRQ	0
#define CG_EVT_OMCI	1
#define CG_EVT_RING_SZ	32
#define CG_EVT_OMCI_LIMIT 16 /* reserve the original sixteen IRQ slots */
struct cg_evt {
	u8 type;
	union {
		struct {
			u32 intr;
			u8 state, id;
		};
		u8 pdu[OMCI_LEN];
	};
};

/* Where the ONU's G.984.3 serial number came from, strongest ...
 * dev/MEASURED-cortina-gpon.c.md sec 26. */
enum cg_sn_src {
	CG_SN_NONE = 0,		/* not provisioned yet: ranging is held off */
	CG_SN_PARAM,		/* cortina_gpon.sn= (bring-up / A-B override) */
	CG_SN_BOARD,		/* the board's own factory data, via /proc/gpon */
};

static const char *const cg_sn_src_name[] = {
	"NONE", "module-param", "board",
};

struct cortina_gpon {
	struct device *dev;
	void __iomem *pon;		/* ioremap of the whole PON window */
	void __iomem *mac;		/* pon + CG_GPON_MAC_OFF, the GPON MAC block */
	void __iomem *glb;		/* ioremap of the GLB reset/clock window */
	void __iomem *gpio;		/* ioremap of the PER_GPIO window */
	struct proc_dir_entry *proc;
#ifdef CONFIG_GPON_OLT_DIAG
	struct proc_dir_entry *oltcap_proc;	/* the far-end capture dump */
#endif

	/* post-O5 servicing (ISR top half -> event ring -> work bottom half) */
	int irq;			/* GIC SPI 1, shared NE global line */
	spinlock_t evt_lock;		/* protects the ring, taken in hardirq */
	struct cg_evt evt[CG_EVT_RING_SZ];
	unsigned int evt_head, evt_tail;
	u32 ingress_generation; /* rejects a producer delayed across identity reset */
	bool ingress_open;
	u32 omci_queue_drop;
	struct work_struct isr_work;
	u32 irq_count;			/* ISR entries that found PON work */
	u32 evt_drop;			/* events lost to a full ring */
	u8 last_state;			/* FSM tracker (0=O1 .. 6=O7) */
	bool omcc_up;			/* OMCC channel bound + link signalled */
	u16 omcc_alloc;			/* confirmed or pending T-CONT[0] owner */
	bool omcc_alloc_pending;		/* submitted write completion is unknown */
	bool omcc_alloc_valid;		/* omcc_alloc actually carries a binding.
					 * G.984.3 ONU-ID 0 is LEGAL, so 0 cannot
					 * double as "never bound": without this
					 * flag an ONU-ID of 0 makes the live-HW
					 * reconcile below think the OMCC T-CONT is
					 * already bound and never replay a lost
					 * Assign_ONU-ID (no US grant -> no OMCI
					 * answer -> OLT Deactivate). */
	u16 omcc_gem;			/* last omci_port.id bound to us-gem 0..7 */

	/* DS OMCI receive (Stage B: count + decode-log; responder = Stage C) */
	u32 omci_rx;			/* DS OMCI PDUs delivered by the NI CPU-RX hook */
	u32 omci_rx_short;		/* runt PDUs (< 8 bytes, not decodable) */
	bool pdc_ready;			/* PDC map + CTRL programmed */
	bool puc_ready;			/* PUC US-VoQ admission programmed */

	/* The PUC control-packet counters, made CUMULATIVE in ...
	 * dev/MEASURED-cortina-gpon.c.md sec 27. */
	spinlock_t puc_cnt_lock;	/* serializes the read-and-add */
	struct delayed_work puc_cnt_work;	/* samples shortly after a US OMCI TX */
	u32 puc_omci_us;		/* upstream OMCI (link-type 0xfff1) frames */
	u32 puc_ctrl_mac;		/* upstream MAC-type control frames */
	u32 puc_len_err;		/* upstream frames failing the length check */
	u32 puc_cnt_samples;		/* reads folded in (0 = never sampled) */

	/* Stage C: the G.988 OMCI responder + US OMCI TX */
	struct omci_onu *omci;		/* responder context (kzalloc'd at probe) */
	spinlock_t omci_lock;		/* common model readers vs ordered worker */
	bool omci_active;		/* transport permits requests */
	bool omci_initialized;		/* accepted MIB survives same-identity LOS */
	struct delayed_work veip_avc_work;	/* the ~31s post-O5 VEIP oper-up AVC */
	/* drives the accepted UNI administrative state onto the ports; runs in
	 * a work item because the apply sleeps and the OMCI path holds a
	 * spinlock, and reschedules itself while an apply is still owed */
	struct delayed_work uni_apply_work;
	/* ★★ SET ONCE, AT TEARDOWN, BEFORE ANY CANCEL.  The producer graph here CYCLES
	 *    -- the ISR work re-arms the coldstart work and the AVC, the coldstart work
	 *    re-arms itself, /proc writes re-arm several -- so no ordering of cancels can
	 *    be correct on its own.  A gate is the only thing that closes a cycle. */
	bool stopping;
	unsigned int veip_avc_retry_ms;	/* backoff after a failed AVC TX; 0 = none pending */
	struct delayed_work coldstart_work;	/* stuck-O1 US-lock-miss recovery */
	int coldstart_tries;		/* re-rolls THIS stuck episode (reset on leaving O1) */
	u32 coldstart_rolls;		/* total re-rolls this power-on (/proc visibility) */
	u32 omci_tx;			/* US OMCI responses enqueued to the NI */
	u32 omci_tx_fail;		/* NI TX rejected (ring/scratch busy) */
	u32 omci_ds_crc_ok;		/* DS MIC self-check (first PDUs only) */
	u32 omci_ds_crc_bad;
	u32 omci_rx_bad_mic;		/* DS frames DISCARDED on an invalid MIC */

	/* Desired binding lives only in the common accepted MIB. This identity
	 * describes actual or partially completed hardware application. */
	struct omci_data_binding hw_data_binding;
	bool data_alloc_bound;
	bool data_alloc_pending; /* write submitted, completion not established */
	bool data_installed;
	/* ★ the data rides the OMCC's T-CONT (single-alloc OLT).  A MODE, not a
	 * failure: the dedicated T-CONT CAM is deliberately left alone and the
	 * data GEM is stamped into the OMCC T-CONT's spare upstream slots. */
	bool data_rides_omcc;
	/* HW CAM identity currently ARMED in silicon for the data ...
	 * dev/MEASURED-cortina-gpon.c.md sec 28. */
	u16 hw_data_alloc;		/* alloc-id armed -> hw T-CONT 1 (0 = none) */
	u16 hw_data_gem;		/* GEM port-id armed in DS-GEM CAM + US_PORT (0 = none) */
	u32 omci_cfg_log;		/* config-ME body log budget used */
	struct net_device *wan_ndev;	/* gpon0 */

	/* The per-board PON identity, single source of truth for BOTH ...
	 * dev/MEASURED-cortina-gpon.c.md sec 124. */
	struct mutex sn_lock;		/* activation, queued control, watchdog and AVC */
	u8 sn[8];			/* wire order: 4 ASCII vendor-id + 4 VSSN bytes */
	enum cg_sn_src sn_src;
	bool activated;			/* cg_mac_activate() has run at least once */
	struct delayed_work sn_wait_work;	/* bounded wait for the board's serial */
};

/* Arm a worker unless teardown has begun.  -> true if it was armed. */
static bool cg_sched(struct cortina_gpon *cg, struct delayed_work *w,
		     unsigned long delay)
{
	if (READ_ONCE(cg->stopping))
		return false;
	return schedule_delayed_work(w, delay);
}

static bool cg_sched_mod(struct cortina_gpon *cg, struct delayed_work *w,
			 unsigned long delay)
{
	if (READ_ONCE(cg->stopping))
		return false;
	return mod_delayed_work(system_wq, w, delay);
}

static bool cg_sched_now(struct cortina_gpon *cg, struct work_struct *w)
{
	if (READ_ONCE(cg->stopping))
		return false;
	return schedule_work(w);
}


static struct cortina_gpon *cg_singleton;

static bool cg_do_reset = true;
module_param_named(reset, cg_do_reset, bool, 0444);
MODULE_PARM_DESC(reset, "release the GPON MAC from reset/clock-gate at probe (default on)");

/* Is the DOWNSTREAM path locked?  The predicate documented at CG_PSDS_RGB8. */
static bool cg_psds_ds_locked(u32 rgb8)
{
	return (rgb8 & CG_PSDS_DS_LOCK_MASK) == CG_PSDS_DS_LOCK_VAL;
}

/* Is the CMU/PLL locked?  A DIFFERENT question -- see the note at the defines. */
static bool cg_psds_cmu_locked(u32 rgb8)
{
	return (rgb8 & CG_PSDS_CMU_LOCK_MASK) == CG_PSDS_CMU_LOCK_VAL;
}

/* Wait for @locked, reading once per millisecond, bounded at ...
 * dev/MEASURED-cortina-gpon.c.md sec 29. */
static int cg_psds_wait_lock(void __iomem *pon, bool (*locked)(u32), int ms_max)
{
	int i;

	for (i = 0; i < ms_max; i++) {
		if (locked(readl(pon + CG_PSDS_RGB8)))
			break;
		mdelay(1);
	}
	return i;
}

static bool cg_do_intr = true;
module_param_named(intr, cg_do_intr, bool, 0444);
MODULE_PARM_DESC(intr, "enable the GPON MAC interrupt servicing path (default on)");

/* Put the whole PON domain into a known reset state so the ...
 * dev/MEASURED-cortina-gpon.c.md sec 30. */
static void cg_glb_reset(struct cortina_gpon *cg)
{
	void __iomem *glb = cg->glb;

	/* aal_gpon __gpon_glb_reset: SerDes power OFF first, mode ...
	 * dev/MEASURED-cortina-gpon.c.md sec 31. */
	writel(0x00000001, glb + CG_GLB_PSDS_INIT);	/* __psds_ad_reset: POW_PCIX=0, SerDes off */
	writel(0x00030000, glb + CG_GLB_EPON_CNTL);	/* select PON/ONU mode */
	writel(0x00000000, glb + CG_GLB_PON_CNTL);	/* assert all PON-domain resets */
	writel(0x00000000, glb + CG_GLB_GPON_CNTL);	/* GTC+ANI stay IN reset until SerDes lock */
	writel(0x00000004, glb + CG_GLB_PON_CNTL);	/* __psds_csr_out_of_reset: psds_reg_rst_n 0->1 only */
	mdelay(1);
}

/* Bring the PON-SerDes CMU/PLL up so it generates the PON APB ...
 * dev/MEASURED-cortina-gpon.c.md sec 32. */
static void cg_psds_init(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;
	u32 v;
	int i;

	/* aal_psds_init entry (GPON pon_mode, stock ca-ne.ko disasm ...
	 * dev/MEASURED-cortina-gpon.c.md sec 33. */
	v = readl(cg->glb + CG_GLB_PON_CNTL) | BIT(0);
	writel(v, cg->glb + CG_GLB_PON_CNTL);
	writel(v & ~BIT(2), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);
	writel(v | BIT(2), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);

	/* __psds_mode_init: GPON rate — sd_s0=1, sds_mode_s0=0x8, usx=0 */
	writel(0x00000408, pon + CG_PSDS_MODE);
	udelay(10);

	/* __psds_prof_load: the CMU/PLL/CDR/TX-driver analog profile.  Each row is
	 * a direct write to the PSDS block (applied via its DATAIN/ACCESS pair). */
	for (i = 0; i < ARRAY_SIZE(cg_serdes_gpon); i++) {
		writel(cg_serdes_gpon[i].val, pon + cg_serdes_gpon[i].off);
		udelay(cg_serdes_gpon[i].delay_us ? cg_serdes_gpon[i].delay_us : 10);
	}

	/* __psds_disable_gpon_los_rst: hold EPON in reset + set the spare-cfg bit
	 * around the lock wait (GPON-only quirk). */
	v = readl(cg->glb + CG_GLB_EPON_CNTL);
	writel(v | BIT(0), cg->glb + CG_GLB_EPON_CNTL);		/* epon_rst_n = 1 */
	v = readl(pon + CG_PON_EPON_SPARE);
	writel(v | 0x80000000, pon + CG_PON_EPON_SPARE);

	/* __psds_ad_out_of_reset: power the SerDes -> PON APB clock runs.  Keep the
	 * laser burst-enable (ben_oen) OFF during the SerDes bring-up; it is set
	 * at the END of this function once the SerDes is stable (stock 0x30).
	 * Vendor delay is mdelay(1) — the settle is the poll below, not a fixed sleep. */
	v = readl(cg->glb + CG_GLB_PSDS_INIT);
	writel((v | CG_PSDS_POW_PCIX) & ~CG_PSDS_BEN_OEN, cg->glb + CG_GLB_PSDS_INIT);
	mdelay(1);

	/* __psds_sync: bounded wait for RX clock lock, continuing on ...
	 * dev/MEASURED-cortina-gpon.c.md sec 34. */
	i = cg_psds_wait_lock(pon, cg_psds_ds_locked, 1001);
	dev_info(cg->dev, "psds: __psds_sync RX-lock wait done at %dms, rgb8=0x%08x\n",
		 i, readl(pon + CG_PSDS_RGB8));

	/* release GPON los-reset */
	v = readl(cg->glb + CG_GLB_EPON_CNTL);
	writel(v & ~BIT(0), cg->glb + CG_GLB_EPON_CNTL);	/* epon_rst_n = 0 */

	/* __psds_gbox_out_of_reset: toggle ... -- dev/MEASURED-cortina-gpon.c.md sec 35. */
	v = readl(cg->glb + CG_GLB_PON_CNTL);
	writel(v & ~BIT(1), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);
	writel(v | BIT(1), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);

	/* __psds_gbox_init: rx/tx bit-ordering = 1 (reset default already 0x454) */
	v = readl(pon + CG_PSDS_GBOX_CTRL);
	v = (v & ~((0x3u << 4) | (0x3u << 6))) | (0x1u << 4) | (0x1u << 6);
	writel(v, pon + CG_PSDS_GBOX_CTRL);
	mdelay(1);

	/* aal_gpon_glb_ctrl_init (vendor: after aal_psds_init, before ...
	 * dev/MEASURED-cortina-gpon.c.md sec 36. */
	i = cg_psds_wait_lock(pon, cg_psds_ds_locked, 2000);
	dev_info(cg->dev, "psds: pre-edge DS-lock wait done at %dms, rgb8=0x%08x\n",
		 i, readl(pon + CG_PSDS_RGB8));
	writel(0x0000030e, cg->glb + CG_GLB_PON_CNTL);	/* pon_serdes/psds/ptp + puc/pdc */
	writel(0x00000003, cg->glb + CG_GLB_GPON_CNTL);	/* ani_rst_n + gpon_rst_n: the live-clock edge */
	mdelay(100);

	/* Drive the laser burst-enable output NOW, while the MAC is ...
	 * dev/MEASURED-cortina-gpon.c.md sec 37. */
	v = readl(cg->glb + CG_GLB_PSDS_INIT);
	writel(v | CG_PSDS_BEN_OEN, cg->glb + CG_GLB_PSDS_INIT);
}

static bool cg_activate = true;
module_param_named(activate, cg_activate, bool, 0444);
MODULE_PARM_DESC(activate, "program the SN + start GPON ranging once the serial number is known (default on)");

/* The per-board GPON serial number (G.984.3 ONU-ID / "VSSN") ...
 * dev/MEASURED-cortina-gpon.c.md sec 38. */
#define CG_SN_WAIT_SECS		60

static char *cg_sn_param;
module_param_named(sn, cg_sn_param, charp, 0444);
MODULE_PARM_DESC(sn, "GPON serial number override, \"VVVVHHHHHHHH\" (4 ASCII vendor-id chars + 8 hex VSSN digits). Bring-up/A-B use ONLY: the shipping path is the board's own config volume pushed in by /etc/init.d/gpon-identity, so never bake a serial number into an image's bootargs");

static bool cg_do_bosa_init = true;
module_param_named(bosa_init, cg_do_bosa_init, bool, 0444);
MODULE_PARM_DESC(bosa_init, "program the GN25L95 BOSA laser driver over per_i2c before ranging (default on; off = no upstream burst, DS-side diagnostics only)");

static bool cg_coldstart_wd = true;
module_param_named(coldstart_wd, cg_coldstart_wd, bool, 0644);
MODULE_PARM_DESC(coldstart_wd, "stuck-O1 recovery watchdog: re-roll the SerDes/laser bring-up while the FSM sits at O1 (default on; 0 = observe-only A/B baseline — flip live via /sys/module to recover a wedged boot in place)");

/* ★★★ DS-into-L3FE routing under hw_l3_fwd. DEFAULT OFF since ...
 * dev/MEASURED-cortina-gpon.c.md sec 39. */
static bool cg_hw_l3_ds = false;
module_param_named(hw_l3_ds, cg_hw_l3_ds, bool, 0644);
MODULE_PARM_DESC(hw_l3_ds, "route the DS data GEM into the L3FE under hw_l3_fwd (default OFF = CPU_0 + FE_BYPASS, the route measured to deliver; =1 black-holes ALL downstream while ds_flows=0). ★ REQUIRED for cortina_ni.hw_ds_offload to do anything: with it off, DS frames bypass both forwarding engines and no DS hash entry is reachable. Watch the wired LAN when enabling (the DS punt window once broke it)");

/* Enable the upstream laser. ★ Proven by the ours-vs-stock ...
 * dev/MEASURED-cortina-gpon.c.md sec 40. */
static void cg_laser_on(struct cortina_gpon *cg)
{
	u32 v;

	if (!cg->gpio)
		return;
	/* route the laser-enable net (stock 0x01101101; ours cold lacks bit20) */
	v = readl(cg->glb + CG_GLB_PINROUTE);
	writel(v | CG_PINROUTE_LASER, cg->glb + CG_GLB_PINROUTE);
	/* pin muxes to EXACT stock: group 0 = 0x1fff (ours' cold 0xffff has
	 * pins 13-15 wrongly GPIO), groups 3/4 = the stock-driven pin sets */
	writel(0x00001fff, cg->glb + CG_GLB_GPIO_MUX0);
	writel(0x000390ff, cg->glb + CG_GLB_GPIO_MUX3);
	writel(0x00003b00, cg->glb + CG_GLB_GPIO_MUX4);
	/* group-0 drive: pin6=HIGH, pin11=LOW, pin12=LOW ... */
	v = readl(cg->gpio + CG_PERGPIO_OUT0);
	writel((v & ~CG_GPIO0_LASER_PINS) | CG_GPIO0_PIN6, cg->gpio + CG_PERGPIO_OUT0);
	/* ... then make pins 6/11/12 outputs (cfg bit 0 = output) */
	v = readl(cg->gpio + CG_PERGPIO_CFG0);
	writel(v & ~CG_GPIO0_LASER_PINS, cg->gpio + CG_PERGPIO_CFG0);
	/* groups 3/4: whole-group stock state, OUT before CFG so each pin
	 * drives the correct level the moment it becomes an output */
	writel(CG_GPIO3_OUT_STOCK, cg->gpio + CG_PERGPIO_OUT3);
	writel(CG_GPIO3_CFG_STOCK, cg->gpio + CG_PERGPIO_CFG3);
	writel(CG_GPIO4_OUT_STOCK, cg->gpio + CG_PERGPIO_OUT4);
	writel(CG_GPIO4_CFG_STOCK, cg->gpio + CG_PERGPIO_CFG4);
	/* pin 34: leave the reset state.  Stock at Online has GLOBAL_GPIO_MUX_1 = 0
	 * (pin 34 NOT muxed to GPIO) and PER_GPIO1_CFG all-inputs -- the live golden
	 * refutes the earlier "stock mux1 bit2=1" claim, so write nothing here.
	 * PER_GPIO1_IN bit2 still serves as the net-level readback in /proc/gpon. */
}

/* The poll half of the indirect transaction. FOUR spellings ...
 * dev/MEASURED-cortina-gpon.c.md sec 41. */
static int cg_go_poll(void __iomem *reg, unsigned int tries, bool pace)
{
	/* ★ THE LOOP IS NOT OURS ANY MORE (2026-09-04): cortina-access.h owns it for
	 * the whole driver (CG_TBL_GO and CA_NI_IND_ACCESS_GO are the same BIT(31)).
	 * This stays as the GPON block's NAME for it, because `pace` reads better than
	 * a function pointer at four call sites. */
	return ca_go_spin(reg, tries, pace ? ca_pause_udelay1 : ca_pause_none);
}

/* ONE indirect table transaction, for every block in this ...
 * dev/MEASURED-cortina-gpon.c.md sec 42. */
static void cg_tbl_timeout_warn(struct cortina_gpon *cg, u32 access_off,
				u32 cmd)
{
	dev_warn(cg->dev, "indirect access +0x%04x cmd 0x%08x timed out\n",
		 access_off, cmd);
}

static int cg_tbl_op_at(struct cortina_gpon *cg, void __iomem *base,
			u32 access_off, u32 cmd)
{
	int rc;

	writel(CG_TBL_GO | cmd, base + access_off);
	rc = cg_go_poll(base + access_off, CG_TBL_TRIES, false);
	if (rc < 0) {
		cg_tbl_timeout_warn(cg, access_off, cmd);
		return rc;
	}
	return 0;
}

/* The PUC/PDC blocks live in the PON window and always WRITE. */
static int cg_puc_ind_write(struct cortina_gpon *cg, u32 access_off, u32 index)
{
	return cg_tbl_op_at(cg, cg->pon, access_off, CG_TBL_WR | index);
}

/* One PDC map-memory entry write: DATA0/DATA1, then the ...
 * dev/MEASURED-cortina-gpon.c.md sec 43. */
static int cg_pdc_map_write(struct cortina_gpon *cg, u32 idx, u32 d0, u32 d1)
{
	writel(d0, cg->pon + CG_PDC_MAP_DATA0);
	writel(d1, cg->pon + CG_PDC_MAP_DATA1);
	return cg_puc_ind_write(cg, CG_PDC_MAP_ACCESS, idx & 0xff);
}

/* PDC init (vendor __pdc_gpon_family_init): route the DS ...
 * dev/MEASURED-cortina-gpon.c.md sec 44. */
static void cg_pdc_init(struct cortina_gpon *cg)
{
	u32 idx, d0, d1, ctrl;
	unsigned int dead = 0;		/* entries this pass could not write */

	for (idx = 0; idx < CG_PDC_MAP_ENTRIES; idx++) {
		cg_pdc_map_entry(idx, CG_OMCC_US_GEM_IDX_NUM, &d0, &d1);
		/* ★★ DO NOT ABANDON THE LIST ON ONE FAILED ENTRY ...
		 * dev/MEASURED-cortina-gpon.c.md sec 45. */
		if (cg_pdc_map_write(cg, idx, d0, d1))
			dead++;
	}

	ctrl = readl(cg->pon + CG_PDC_CTRL);
	ctrl &= ~CG_PDC_CTRL_HP_MASK;
	ctrl |= CG_PDC_CTRL_MAP_MEM_EN | CG_PDC_CTRL_HP_EN |
		(7 << CG_PDC_CTRL_HP_COS_SH) |
		(CG_LPORT_CPU_0 << CG_PDC_CTRL_HP_LDPID_SH);
	writel(ctrl, cg->pon + CG_PDC_CTRL);

	/* ★ READY MEANS EVERY ENTRY LANDED, not "we got to the end".  Claiming
	 * ready over a dead entry is what would let the supervisor stop looking
	 * while the datapath is still short one GEM index. */
	cg->pdc_ready = (dead == 0);
	if (dead)
		dev_warn(cg->dev,
			 "PDC: %u of %u map entr%s could not be written - NOT ready, the O5 supervisor will re-run this\n",
			 dead, (unsigned int)CG_PDC_MAP_ENTRIES,
			 dead == 1 ? "y" : "ies");
	else
		dev_info(cg->dev, "PDC: OMCC DS GEMs 0-7 -> CPU_0, ctrl=0x%08x\n",
			 readl(cg->pon + CG_PDC_CTRL));
}


/* One PUC per-VoQ valid bit (PUC_valid_voqN, 256-bit mask across 8 regs). */
static void cg_puc_voq_valid(struct cortina_gpon *cg, u32 voq, bool valid)
{
	u32 off = CG_PUC_VALID_VOQ0 - (voq / 32) * 4;
	u32 v = readl(cg->pon + off);

	if (valid)
		v |= BIT(voq % 32);
	else
		v &= ~BIT(voq % 32);
	writel(v, cg->pon + off);
}

/* Program one PUC pvtbl entry (per-T-CONT VoQ map) + its 8 ...
 * dev/MEASURED-cortina-gpon.c.md sec 46. */
static int cg_puc_pvtbl_program(struct cortina_gpon *cg, u32 tcont, bool ena)
{
	void __iomem *pon = cg->pon;
	u32 d0, d1, d2, q;

	cg_puc_pvtbl_words(tcont, ena, &d0, &d1, &d2);

	writel(0, pon + CG_PUC_PVTBL_DATA4);
	writel(0, pon + CG_PUC_PVTBL_DATA3);
	writel(d2, pon + CG_PUC_PVTBL_DATA2);
	writel(d1, pon + CG_PUC_PVTBL_DATA1);
	writel(d0, pon + CG_PUC_PVTBL_DATA0);
	if (cg_puc_ind_write(cg, CG_PUC_PVTBL_ACCESS, tcont))
		return -ETIMEDOUT;

	for (q = 0; q < CG_PUC_QUEUE_PER_TCONT; q++) {
		u32 qid = q + tcont * CG_PUC_QUEUE_PER_TCONT;

		if (qid <= 63) {
			writel(qid & 0x7, pon + CG_PUC_VOQBPREMAP_DATA);
			if (cg_puc_ind_write(cg, CG_PUC_VOQBPREMAP_ACCESS, qid))
				return -ETIMEDOUT;
		}
		cg_puc_voq_valid(cg, qid, ena);
	}
	return 0;
}

/* Flush one T-CONT's 8 VoQs (PUC_VOQFLUSH: start + ... -- dev/MEASURED-cortina-gpon.c.md sec 47. */
static int cg_puc_voq_flush(struct cortina_gpon *cg, u32 tcont)
{
	u32 q, v;

	for (q = 0; q < CG_PUC_QUEUE_PER_TCONT; q++) {
		v = BIT(31) | BIT(16) | ((tcont & 0x1f) << 8) |
		    ((tcont * CG_PUC_QUEUE_PER_TCONT + q) & 0xff);
		writel(v, cg->pon + CG_PUC_VOQFLUSH);
		if (cg_go_poll(cg->pon + CG_PUC_VOQFLUSH, CG_TBL_TRIES, true) < 0) {
			dev_warn(cg->dev, "VoQ %u flush timed out\n",
				 tcont * CG_PUC_QUEUE_PER_TCONT + q);
			return -ETIMEDOUT;
		}
	}	return 0;
}

/* PUC init (vendor aal_puc_init, GPON path) — the US ...
 * dev/MEASURED-cortina-gpon.c.md sec 48. */
static void cg_puc_init(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;
	u32 tcont, q, v;
	unsigned int dead = 0;		/* per-T-CONT entries this pass missed */

	/* clear the PUC interrupt-enable (vendor: PUC_PONCNTL_INTENABLE = 0) */
	writel(0, pon + CG_PUC_PONCNTL_INTEN);

	/* PUCCFG: inccfg=2 (clear-on-read), crccntl=2 (regenerate US CRC),
	 * invalid_voqdrop_enable=1 (drop frames that hit an invalid VoQ) */
	v = readl(pon + CG_PUC_PUCCFG);
	v = (v & ~(GENMASK(18, 16) | GENMASK(1, 0))) | (2u << 16) | 2u;
	v |= BIT(30);
	writel(v, pon + CG_PUC_PUCCFG);

	/* VoQ buffer limits (GPON scfg VOQBUFLIMIT A/B/C) + per-VoQ limit-select
	 * (below 8 queues use A, 8..16 use B, >16 use C; all 256 -> A) */
	writel(0x7a0, pon + CG_PUC_VOQBUFLIMIT_A);
	writel(0x3b0, pon + CG_PUC_VOQBUFLIMIT_B);
	writel(0x200, pon + CG_PUC_VOQBUFLIMIT_C);
	for (q = 0; q < CG_PUC_VOQBUFLIMSEL_N; q++)
		writel(0x55555555, pon + CG_PUC_VOQBUFLIMSEL0 + q * 4);

	/* VoQ map mode = 8Q (voqmapsel = 0) */
	writel(0, pon + CG_PUC_VOQMAPCFG);

	/* pvtbl: per-T-CONT VoQ map. Only T-CONT 0 (OMCC) has queues ...
	 * dev/MEASURED-cortina-gpon.c.md sec 49. */
	for (tcont = 0; tcont < CG_PUC_TCONT_NUM; tcont++)
		if (cg_puc_pvtbl_program(cg, tcont, tcont == 0))
			dead++;
	/* the CPU high-priority OMCI inject rides the 9th queue (VoQ 127) */
	cg_puc_voq_valid(cg, CG_PUC_9TH_QUEUE_VOQ, true);

	/* US OMCI header-A replacement: for an OMCI control frame ...
	 * dev/MEASURED-cortina-gpon.c.md sec 50. */
	writel(BIT(31) | (6u << 8) | 6u, pon + CG_PUC_US_OMCI_HDR_A);
	writel((7u << 8) | 7u, pon + CG_PUC_US_OMCI_HP_HDR_A);
	v = readl(pon + CG_PUC_GLOBAL_PLOAM_CFG);
	v = (v & ~GENMASK(21, 16)) | (30u << 16) | BIT(31);
	writel(v, pon + CG_PUC_GLOBAL_PLOAM_CFG);

	/* That same link type is what makes the control-packet ...
	 * dev/MEASURED-cortina-gpon.c.md sec 51. */
	v = readl(pon + CG_PUC_GLOBAL_LNK_TYPE) >> 16;
	if (v != CG_PUC_LNK_TYPE_OMCI)
		dev_warn(cg->dev,
			 "PUC control-packet link type is 0x%04x, expected 0x%04x: the upstream OMCI frame count will not match\n",
			 v, CG_PUC_LNK_TYPE_OMCI);

	/* back-pressure: drop off, bp on, threshold 0x100 */
	v = readl(pon + CG_PUC_BPCNTL);
	v = (v & ~(BIT(4) | GENMASK(30, 16))) | BIT(0) | (0x100u << 16);
	writel(v, pon + CG_PUC_BPCNTL);

	/* BTC (GPON): pfovrhd=5, schmode=FRAGMENT(0), wdaligned=0,
	 * minrmnwindowsz=5, sch2en=1, lrgfrmfragen=1 (segment >4095B frames) */
	v = readl(pon + CG_PUC_BTCCFG);
	v = (v & ~GENMASK(5, 0)) | 5u;
	v &= ~(BIT(8) | BIT(12));
	v |= BIT(16) | BIT(25);
	v = (v & ~GENMASK(31, 27)) | (5u << 27);
	writel(v, pon + CG_PUC_BTCCFG);

	/* QM<->PUC report-adjust levels (GPON) */
	writel(0x00c80000, pon + CG_PUC_Q2PQSRCFG01);	/* lv0=0, lv1=0xc8 */
	writel(0x05c201b8, pon + CG_PUC_Q2PQSRCFG23);	/* lv2=0x1b8, lv3=0x5c2 */

	/* aggregate shaper + PIR (rate limiter off) */
	v = readl(pon + CG_PUC_CTRL);
	v = (v | BIT(30)) & ~BIT(26);	/* shp_en=1, rl_en=0 */
	writel(v, pon + CG_PUC_CTRL);
	writel(20u | (20u << 5) | (20u << 10), pon + CG_PUC_CTRL1);
	v = readl(pon + CG_PUC_CTRL2);
	v = (v & ~GENMASK(4, 0)) | 20u | BIT(26);	/* pirovhd=20, pir_en=1 */
	writel(v, pon + CG_PUC_CTRL2);

	/* ★ READY MEANS EVERY T-CONT LANDED - see the same note in cg_pdc_init. */
	cg->puc_ready = (dead == 0);
	if (dead)
		dev_warn(cg->dev,
			 "PUC: %u of %u pvtbl entr%s could not be programmed - NOT ready, the O5 supervisor will re-run this\n",
			 dead, (unsigned int)CG_PUC_TCONT_NUM,
			 dead == 1 ? "y" : "ies");
	dev_info(cg->dev,
		 "PUC: OMCC T-CONT0 VoQs + 9th-queue enabled, puccfg=0x%08x lnk_type=0x%04x\n",
		 readl(pon + CG_PUC_PUCCFG),
		 readl(pon + CG_PUC_GLOBAL_LNK_TYPE) >> 16);
}

/* Fold one read of the PUC control-packet counters into the ...
 * dev/MEASURED-cortina-gpon.c.md sec 52. */
static void cg_puc_ctrl_sample(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;

	if (!cg->puc_ready)
		return;
	spin_lock(&cg->puc_cnt_lock);
	cg->puc_omci_us += readl(pon + CG_PUC_BMC_CTRL_PKT_LNK) &
			   CG_PUC_BMC_CNTR_MASK;
	cg->puc_ctrl_mac += readl(pon + CG_PUC_BMC_CTRL_PKT_MAC) &
			    CG_PUC_BMC_CNTR_MASK;
	cg->puc_len_err += readl(pon + CG_PUC_BMC_LENGTH_ERROR) &
			   CG_PUC_BMC_CNTR_MASK;
	cg->puc_cnt_samples++;
	spin_unlock(&cg->puc_cnt_lock);
}

/* Sample shortly after an upstream OMCI frame was handed to ...
 * dev/MEASURED-cortina-gpon.c.md sec 53. */
#define CG_PUC_CNT_TX_DELAY_MS	20
/* Backoff for a failed VEIP oper-up AVC TX.  Bounded in RATE, not in
 * attempts: the OLT never re-solicits this AVC, so a count cap would end
 * the session's only path back to Match State normal. */
#define CG_VEIP_AVC_RETRY_MIN_MS	500
#define CG_VEIP_AVC_RETRY_MAX_MS	30000

static void cg_puc_cnt_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon, puc_cnt_work);

	/* ★ ENTRY GUARD: a worker already running when teardown began keeps
	 *   going, and its context is about to be freed.  ⚠ THIS ONE DOES NOT
	 *   RE-ARM INTERRUPTS -- it samples and logs -- so the reason it stops
	 *   is lifetime, not the interrupt mask. */
	if (READ_ONCE(cg->stopping))
		return;

	cg_puc_ctrl_sample(cg);
}

/* Program the GPON MAC identity + datapath, then start the ...
 * dev/MEASURED-cortina-gpon.c.md sec 54. */
static void cg_ingress_stop(struct cortina_gpon *cg);
static void cg_transport_down(struct cortina_gpon *cg);

static int cg_mac_activate(struct cortina_gpon *cg, bool identity_changed,
			   bool *armed)
{
	void __iomem *mac = cg->mac;
	struct gpon_range_request req;
	struct gpon_range_action act;
	u32 v;
	int i, ret;

	if (armed)
		*armed = false;

	/* The PON/GPON reset release (PON_CNTL=0x30e, GPON_CNTL=0x3) ...
	 * dev/MEASURED-cortina-gpon.c.md sec 55. */
	req.identity_defined = gpon_sn_is_set(cg->sn);
	req.identity_changed = identity_changed;
	req.ranging_now = !!(readl(mac + CG_REG_ONU_CTL) & CG_ONU_CTL_EN);
	req.activation_wanted = cg_activate;
	req.laser_wanted = cg_do_bosa_init;
	gpon_range_plan(&req, &act);
	if (act.quiesce || act.program) {
		cg_ingress_stop(cg);
		cg_transport_down(cg);
	}

	/* FIRST, and before ANY programming: the serial number and the MAC
	 * config are write-while-en=0, and a live re-provision arrives with en
	 * set. The laser's own bus traffic waits for this too. */
	if (act.quiesce) {
		u32 ctl = readl(mac + CG_REG_ONU_CTL);

		writel(ctl & ~CG_ONU_CTL_EN, mac + CG_REG_ONU_CTL);
		dev_info(cg->dev, "onu_ctl.en CLEARED (was 0x%08x) before %s\n", ctl,
			 act.program ? "re-programming this ONU's identity"
				     : gpon_range_reason_str(act.reason));
	}
	if (!act.program) {
		cg->activated = act.activated;
		if (act.result)
			dev_err(cg->dev, "NOT ranging: %s\n",
				gpon_range_reason_str(act.reason));
		else
			dev_info(cg->dev, "not ranging: %s\n",
				 gpon_range_reason_str(act.reason));
		return act.result;
	}

	/* De-assert the laser TX-disable net BEFORE programming the ...
	 * dev/MEASURED-cortina-gpon.c.md sec 56. */
	cg_laser_on(cg);
	mdelay(10);	/* let the TX_DIS net settle before the i2c stream */

	ret = cg_bosa_init(cg->dev);
	if (ret) {
		cg->activated = false;
		dev_err(cg->dev,
			"REFUSING to range (%s, %d): nothing is transmitted upstream and onu_ctl.en is clear\n",
			gpon_range_reason_str(GPON_RANGE_LASER_FAILED), ret);
		return ret;
	}

	/* --- config while en=0 (serial number is range-critical) --- */
	writel(CG_ONU_CFG_VAL, mac + CG_REG_ONU_CFG_REAL);	/* laser_on_align=0x12, pre_bias=18 */
	/* The PON identity, from cg->sn (the board's serial number -- see the
	 * cg_sn_* block).  Both halves come from the SAME 8 bytes the OMCI
	 * responder is armed with, so the PLOAM and OMCI identities cannot drift. */
	writel(cg_sn_word(cg->sn), mac + CG_REG_VENDOR);	/* 4 ASCII vendor-id chars */
	writel(cg_sn_word(cg->sn + 4), mac + CG_REG_VENDOR_SPEC);	/* 4 VSSN bytes */
	/* datapath: gpon_ds.max_packet_size (bits 29:16) = 0x3FFF */
	v = readl(mac + CG_REG_GPON_DS);
	v = (v & ~(0x3fffu << 16)) | (0x3fffu << 16);
	writel(v, mac + CG_REG_GPON_DS);
	/* SF/SD BER-alarm thresholds + BER interval (stock 0x6532 -> ...
	 * dev/MEASURED-cortina-gpon.c.md sec 57. */
	cg_pdc_init(cg);
	/* PUC (US-side): the CPU-inject OMCI admission -> OMCC T-CONT/GEM-US
	 * burst.  Vendor __gpon_datapath_init runs aal_puc_init right after the
	 * PDC.  Isolated to the PON+0x8000 sub-block; safe pre-range. */
	cg_puc_init(cg);
	/* password / AES keys: deferred (not needed to range) */

	/* Wait for the downstream to lock (RGB8 bit15 BER_NOTIFY) before enabling
	 * ranging, so the FSM sees a live downstream at the moment en is asserted. */
	i = cg_psds_wait_lock(cg->pon, cg_psds_ds_locked, 8000);
	dev_info(cg->dev, "activate: DS-lock wait done at %dms, rgb8=0x%08x\n",
		 i, readl(cg->pon + CG_PSDS_RGB8));

	/* the GO --- (ben_oen was set at the end of cg_psds_init, ...
	 * dev/MEASURED-cortina-gpon.c.md sec 58. */
	writel(CG_ONU_CTL_VAL, mac + CG_REG_ONU_CTL);	/* onu_ctl.en @ +0x134 */
	cg->activated = true;
	if (armed)
		*armed = true;
	/* Freeze the SN random-delay engine (vendor ...
	 * dev/MEASURED-cortina-gpon.c.md sec 59. */
	v = readl(mac + CG_REG_GPON_MAC_CTRL);
	writel(v & ~(CG_MAC_CTRL_SW_RANDOM_EN | CG_MAC_CTRL_PTI_OMCI),
	       mac + CG_REG_GPON_MAC_CTRL);
	/* (the laser TX-disable net was de-asserted before the BOSA init above) */
	return 0;
}

/* Program the identity + start ranging, and verify the ...
 * dev/MEASURED-cortina-gpon.c.md sec 60. */
static int cg_activate_start(struct cortina_gpon *cg, bool identity_changed)
{
	char sn_str[13];
	bool armed = false;
	u32 vid;
	int ret;

	gpon_sn_format(cg->sn, sn_str);

	/* ★★★ THE IDENTITY GATE, and it is HERE because this is the ...
	 * dev/MEASURED-cortina-gpon.c.md sec 61. */
	if (!gpon_sn_is_set(cg->sn)) {
		dev_err(cg->dev,
			"REFUSING to range: %s is not a defined serial number (a 00000000/ffffffff half is blank storage, not an identity) - parked at O1, nothing transmitted. Push this board's own:  echo \"sn <VVVVHHHHHHHH>\" > /proc/gpon\n",
			sn_str);
		return -ENXIO;
	}

	dev_info(cg->dev, "activating with serial number %s (source: %s)\n",
		 sn_str, cg_sn_src_name[cg->sn_src]);

	/* ⚠ `activated` IS THE PLAN'S ANSWER, NOT "rc was 0". ...
	 * dev/MEASURED-cortina-gpon.c.md sec 62. */
	ret = cg_mac_activate(cg, identity_changed, &armed);
	if (ret)
		return ret;
	if (!cg->activated)
		return 0;	/* deliberately not ranging: nothing to supervise */
	if (!armed)
		return 0;	/* already ranging with this identity: untouched */

	vid = readl(cg->mac + CG_REG_VENDOR);
	if (vid != cg_sn_word(cg->sn))
		dev_warn(cg->dev,
			 "vendor-id readback 0x%08x != programmed 0x%08x - PON window base wrong, or the MAC is still gated\n",
			 vid, cg_sn_word(cg->sn));

	/* Post-activation snapshot, on EVERY activation path (the probe's 30-line
	 * ranging poll below only runs when the identity was known at probe).
	 * /proc/gpon carries the full picture on demand. */
	dev_info(cg->dev,
		 "activate: vendor-id=0x%08x vendor-spec=0x%08x onu_cfg=0x%08x onu_ctl=0x%08x gpon_ds=0x%08x onu=0x%08x rgb8=0x%08x\n",
		 vid, readl(cg->mac + CG_REG_VENDOR_SPEC),
		 readl(cg->mac + CG_REG_ONU_CFG_REAL),
		 readl(cg->mac + CG_REG_ONU_CTL),
		 readl(cg->mac + CG_REG_GPON_DS),
		 readl(cg->mac + CG_REG_GPON_ONU),
		 readl(cg->pon + CG_PSDS_RGB8));

	/* Arm the cold-start US-lock recovery watchdog: if the HW ...
	 * dev/MEASURED-cortina-gpon.c.md sec 125. */
	cg_sched_mod(cg, &cg->coldstart_work, 15 * HZ);
	return 0;
}

/* Latch a serial number and (re)start ranging with it. The ...
 * dev/MEASURED-cortina-gpon.c.md sec 63. */
static int cg_identity_prepare(struct cortina_gpon *cg);
static void cg_datapath_reset(struct cortina_gpon *cg);

/* Caller holds sn_lock, including before removing an event from the ring. */
static void cg_ingress_stop(struct cortina_gpon *cg)
{
	unsigned long flags;

	spin_lock_irqsave(&cg->evt_lock, flags);
	cg->ingress_open = false;
	cg->ingress_generation++;
	cg->evt_tail = cg->evt_head;
	spin_unlock_irqrestore(&cg->evt_lock, flags);
}

static void cg_ingress_start(struct cortina_gpon *cg)
{
	unsigned long flags;

	spin_lock_irqsave(&cg->evt_lock, flags);
	cg->ingress_open = cg->activated;
	spin_unlock_irqrestore(&cg->evt_lock, flags);
}

static int cg_sn_set(struct cortina_gpon *cg, const char *s, enum cg_sn_src src)
{
	u8 sn[8];
	char sn_str[13];
	bool changed;
	int ret;

	ret = cg_sn_parse(s, sn);
	if (ret) {
		dev_err(cg->dev, "rejected GPON serial number \"%s\": expected 4 vendor-id characters + 8 hex digits\n",
			s ? s : "");
		return ret;
	}
	/* ★ WELL-FORMED IS NOT DEFINED. "XPONFFFFFFFF" parses ...
	 * dev/MEASURED-cortina-gpon.c.md sec 64. */
	if (!gpon_sn_is_set(sn)) {
		dev_err(cg->dev, "rejected GPON serial number \"%s\": a 00000000/ffffffff half is blank storage, not this board's identity - ranging stays parked at O1\n",
			s ? s : "");
		return -EINVAL;
	}

	mutex_lock(&cg->sn_lock);
	changed = cg->sn_src == CG_SN_NONE || memcmp(cg->sn, sn, sizeof(sn));
	memcpy(cg->sn, sn, sizeof(sn));
	cg->sn_src = src;
	gpon_sn_format(cg->sn, sn_str);

	/* ⚠ NO SHORTCUTS HERE. This used to decide for itself whether ...
	 * dev/MEASURED-cortina-gpon.c.md sec 65. */
	if (cg->activated && changed)
		dev_warn(cg->dev, "serial number CHANGED to %s (source: %s) - re-ranging\n",
			 sn_str, cg_sn_src_name[src]);
	cancel_delayed_work(&cg->sn_wait_work);
	/* A failed cleanup keeps the old model identity. A retry with the same
	 * newly latched serial must finish that cleanup before activation. */
	if (changed || (cg->omci_initialized &&
			memcmp(cg->omci->sn, cg->sn, sizeof(cg->sn)))) {
		ret = cg_identity_prepare(cg);
		if (ret)
			goto out;
	}
	ret = cg_activate_start(cg, changed);
	cg_ingress_start(cg);
out:
	mutex_unlock(&cg->sn_lock);
	/* ⚠ THE SERIAL IS LATCHED EITHER WAY, AND THAT IS DELIBERATE: ...
	 * dev/MEASURED-cortina-gpon.c.md sec 66. */
	return ret;
}

/* Nothing provisioned a serial number in time. SAY SO, ...
 * dev/MEASURED-cortina-gpon.c.md sec 67. */
static void cg_sn_wait_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon, sn_wait_work);

	/* ★ ENTRY GUARD -- see cg_puc_cnt_work(): a worker already running when
	 *   teardown began keeps going, and its context is about to be freed. */
	if (READ_ONCE(cg->stopping))
		return;

	mutex_lock(&cg->sn_lock);
	if (cg->sn_src != CG_SN_NONE) {		/* raced with a provisioning write */
		mutex_unlock(&cg->sn_lock);
		return;
	}
	dev_err(cg->dev,
		"NO per-board GPON serial number after %ds: is /etc/init.d/gpon-identity running, and is ubi0:ubi_Config mountable? PARKED at O1 and transmitting NOTHING - an ONU that does not know its identity may not announce one. Push the real one:  echo \"sn <VVVVHHHHHHHH>\" > /proc/gpon\n",
		CG_SN_WAIT_SECS);
	mutex_unlock(&cg->sn_lock);
}

/* us.frame_var: compensate the US burst position for the ...
 * dev/MEASURED-cortina-gpon.c.md sec 68. */
static void cg_frame_var_update(struct cortina_gpon *cg)
{
	u32 t3 = readl(cg->mac + CG_REG_T3_PREAMBLE);
	u32 pre = t3 & 0xff, ranged = (t3 >> 8) & 0xff;
	u32 us, fv;

	/* ★★ RECOMPUTE UNCONDITIONALLY, exactly as stock does on ...
	 * dev/MEASURED-cortina-gpon.c.md sec 69. */
	fv = (0x200 - ((pre + ranged + 0x20) & 0xff)) & 0x1ff;
	us = readl(cg->mac + CG_REG_US);
	if ((us & 0x1ff) == fv)
		return;
	writel((us & ~0x1ffu) | fv, cg->mac + CG_REG_US);
	dev_info(cg->dev, "us.frame_var = 0x%03x (t3_preamble 0x%08x)\n", fv, t3);
}

static inline u32 cg_mac_rd(struct cortina_gpon *cg, u32 off)
{
	return readl(cg->mac + off);
}

/* Re-lock the PON-SerDes CMU/PLL (vendor aal_psds_reset). At ...
 * dev/MEASURED-cortina-gpon.c.md sec 70. */
static void cg_psds_relock(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;
	/* ANA_MISC_REG00[7:4] (vendor's name for internal reg page ...
	 * dev/MEASURED-cortina-gpon.c.md sec 71. */
	static const u8 seq[] = {
		0x8,	/* pdown=1, rx_en=0: CDR held in power-down, receiver off */
		0xd,	/* pdown=1, rx_en=1: receiver enabled while still powered down */
		0x7,	/* pdown=0, rx_en=1: CDR powered up, receiver on (+b5, unnamed) */
		0x0,	/* pdown=0, rx_en=0: strobe released, nibble left clear */
	};
	u32 base;
	int i, k;

	/* read the current CMU reg (a088 read strobe -> a090), clear field [7:4] */
	writel(CG_PSDS_IND_READ | CG_PSDS_CMU_IDX, pon + CG_PSDS_IND_CMD);
	udelay(10);
	base = readl(pon + CG_PSDS_IND_RDATA) & ~0xf0u;

	/* strobe [7:4] = 8 -> d -> 7 -> 0, ~1 ms apart (aal_psds_reset) */
	for (k = 0; k < ARRAY_SIZE(seq); k++) {
		writel(base | ((u32)seq[k] << 4), pon + CG_PSDS_IND_WDATA);
		writel(CG_PSDS_IND_WRITE | CG_PSDS_CMU_IDX, pon + CG_PSDS_IND_CMD);
		mdelay(1);
	}

	/* re-wait the CMU/PLL lock (bounded ~1000 ms, as the vendor does) */
	i = cg_psds_wait_lock(pon, cg_psds_cmu_locked, 1001);
	dev_info(cg->dev, "psds re-lock (8/d/7/0): base=0x%08x lock at %dms rgb8=0x%08x\n",
		 base, i, readl(pon + CG_PSDS_RGB8));
}

/* Re-arm the GPON MAC's interrupt enables (the four W1C groups + int_top).  The
 * GLB-level aggregation gates and the requested IRQ live outside the GTC block and
 * survive a GTC reset, so a cold-start re-roll only needs to restore THIS. */
static void cg_mac_intr_arm(struct cortina_gpon *cg)
{
	static const struct { u32 sts, en, mask; } grp[4] = {
		{ CG_REG_INT,  CG_REG_INT_EN,  CG_INT_EN_DEFAULT },
		{ CG_REG_INT2, CG_REG_INT2_EN, 0 },
		{ CG_REG_INT3, CG_REG_INT3_EN, 0 },
		{ CG_REG_INT4, CG_REG_INT4_EN, 0 },
	};
	int i;

	writel(0, cg->mac + CG_REG_INT_TOP_EN);
	(void)readl(cg->mac + CG_REG_INT_TOP);		/* read-clear stale */
	for (i = 0; i < 4; i++) {
		writel(0, cg->mac + grp[i].en);
		writel(grp[i].mask, cg->mac + grp[i].sts);	/* W1C stale */
		writel(grp[i].mask, cg->mac + grp[i].en);
	}
	writel(CG_INT_TOP_EN_ALL, cg->mac + CG_REG_INT_TOP_EN);
}

/* The poll half of the indirect transaction, spelled FOUR ...
 * dev/MEASURED-cortina-gpon.c.md sec 72. */
#define CG_COLD_FAST_TRIES	12
/* Post-O5 SUPERVISOR cadence. Once the FSM reaches Operation ...
 * dev/MEASURED-cortina-gpon.c.md sec 73. */
#define CG_O5_SUPERVISOR_SECS	30
static void cg_coldstart_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon, coldstart_work);
	u32 onu, rgb8;

	u8 state;
	bool ds_locked;

	mutex_lock(&cg->sn_lock);
	/* ★ THE GUARD IS INSIDE THE LOCK, and that is the point: read before taking
	 *   it, this worker could see `not stopping`, be preempted, let teardown take
	 *   the lock, set the flag and mask the interrupts, then acquire the lock and
	 *   re-arm the hardware it had already decided to touch. */
	if (READ_ONCE(cg->stopping))
		goto out;
	if (cg->omci_initialized && memcmp(cg->omci->sn, cg->sn, sizeof(cg->sn))) {
		if (!cg_identity_prepare(cg)) {
			cg_activate_start(cg, true);
			cg_ingress_start(cg);
		}
		cg_sched(cg, &cg->coldstart_work, 3 * HZ);
		goto out;
	}
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	rgb8 = readl(cg->pon + CG_PSDS_RGB8);
	state = CG_ONU_STATE(onu);
	ds_locked = cg_psds_ds_locked(rgb8);

	if (state != 0) {			/* left O1: ranging is progressing */
		cg->coldstart_tries = 0;	/* fresh episode = fresh fast budget */
		if (state != CG_STATE_OPERATION) {
			cg_sched(cg, &cg->coldstart_work, 5 * HZ);
			goto out;
		}
/* ONE indirect table transaction, for every block in this ...
 * dev/MEASURED-cortina-gpon.c.md sec 74. */
		if ((!cg->pdc_ready || !cg->puc_ready) && !cg->data_installed) {
			dev_warn(cg->dev,
				 "O5 supervisor: re-running %s%s init (idempotent; datapath not yet armed)\n",
				 cg->pdc_ready ? "" : "PDC ",
				 cg->puc_ready ? "" : "PUC ");
			if (!cg->pdc_ready)
				cg_pdc_init(cg);
			if (!cg->puc_ready)
				cg_puc_init(cg);
		}
		cg_sched_now(cg, &cg->isr_work);
		cg_sched(cg, &cg->coldstart_work,
				      CG_O5_SUPERVISOR_SECS * HZ);
		goto out;
	}
	if (!ds_locked) {
		/* No DS frame lock: the RX is still settling (cold boot) OR ...
		 * dev/MEASURED-cortina-gpon.c.md sec 126. */
		cg_sched(cg, &cg->coldstart_work, 3 * HZ);
		goto out;
	}
	/* state O1 with DS LOCKED = the stuck-O1 signature (no US PLOAM/burst). */
	if (!cg_coldstart_wd) {
		/* A/B baseline (coldstart_wd=0): observe the stuck-O1, never
		 * re-roll — the pre-watchdog wedge.  Flipping the param live
		 * (/sys/module/.../coldstart_wd) lets the SAME wedged boot then
		 * recover, isolating the re-roll as the fix. */
		cg_sched(cg, &cg->coldstart_work, 16 * HZ);
		goto out;
	}
	cg->coldstart_tries++;
	cg->coldstart_rolls++;
	if (cg->coldstart_tries == CG_COLD_FAST_TRIES)
		dev_warn(cg->dev,
			 "cold-start recovery: %d fast re-rolls, still O1 - backing off to 60s cadence, never stopping\n",
			 cg->coldstart_tries);
	dev_info(cg->dev,
		 "cold-start stuck O1, DS locked but no PLOAM (onu=0x%08x rgb8=0x%08x us=0x%08x t3=0x%08x) - full SerDes re-roll #%u\n",
		 onu, rgb8, cg_mac_rd(cg, CG_REG_US), cg_mac_rd(cg, CG_REG_T3_PREAMBLE),
		 cg->coldstart_rolls);
	/* re-run the whole proven bring-up = a fresh cold roll of the metastable
	 * gearbox/framer + a clean SN/ranging re-arm.  Each attempt is
	 * internally bounded (SerDes lock poll <=1 s, activate DS-wait <=8 s),
	 * so the cadence below bounds the retry RATE; nothing bounds the count. */
	cg_ingress_stop(cg);
	cg_datapath_reset(cg);
	cg_glb_reset(cg);
	cg_psds_init(cg);
	cg_mac_intr_arm(cg);	/* the GTC reset cleared the MAC int enables */
	/* sn_lock so a serial arriving mid-re-roll cannot be half-applied.
	 * ⚠ BOTH DIRECTIONS: the first cut cleared `activated` on failure and never set
	 * it again, so one failed roll made every later SUCCESSFUL one read as not
	 * activated. */
	cg_mac_activate(cg, true, NULL);	/* same identity, retained MIB */
	cg_ingress_start(cg);
	cg_sched(cg, &cg->coldstart_work,
			      cg->coldstart_tries >= CG_COLD_FAST_TRIES ?
			      60 * HZ : 16 * HZ);out:
	mutex_unlock(&cg->sn_lock);
}

/* NOTE: do NOT read the TX-PLOAM MIB (indirect ACCESS/DATA ...
 * dev/MEASURED-cortina-gpon.c.md sec 75. */

static const char *const cg_state_name[8] = {
	"O1-Initial", "O2-Standby", "O3-SerialNumber", "O4-Ranging",
	"O5-Operation", "O6-POPUP", "O7-EmergencyStop", "unknown",
};

/* There is no GPON-MAC-block spelling of cg_tbl_op_at() any ...
 * dev/MEASURED-cortina-gpon.c.md sec 76. */
static struct hwio cg_mac_io(struct cortina_gpon *cg)
{
	struct hwio io = { .rd = ca_hwio_rd, .wr = ca_hwio_wr,
			   .ctx = (void *)cg->mac };

	return io;
}

static int cg_ind_timed_out(struct cortina_gpon *cg,
			    const struct gpon_ind_tbl *t, u32 stuck)
{
	cg_tbl_timeout_warn(cg, reg_at(t->access), stuck & ~t->go);
	return -ETIMEDOUT;
}

/* ONE read-modify-write of a T-CONT CAM entry, for every site ...
 * dev/MEASURED-cortina-gpon.c.md sec 77. */
static int cg_tcont_cam_rmw(struct cortina_gpon *cg, u32 alloc,
			    u32 clr, u32 set, const char *who, bool *write_issued)
{
	struct hwio io = cg_mac_io(cg);
	u32 stuck = 0;

	if (write_issued)
		*write_issued = false;
	alloc = gpon_gem_us_alloc_id(alloc);
	if (gpon_ind_rmw(&io, &cg_tcont_cam_tbl, alloc, clr, set,
			 ca_pause_none, &stuck) == 0) {
		if (write_issued)
			*write_issued = true;
		return 0;
	}
	if (write_issued)
		*write_issued = !!(stuck & CG_TBL_WR);
	cg_ind_timed_out(cg, &cg_tcont_cam_tbl, stuck);
	if (who && (stuck & CG_TBL_WR))
		dev_warn_ratelimited(cg->dev,
			"%s: T-CONT write access timed out for alloc %u - bind NOT complete\n",
			who, alloc);
	else if (who)
		dev_warn_ratelimited(cg->dev,
			"%s: T-CONT read access timed out for alloc %u - bind NOT complete\n",
			who, alloc);
	return -ETIMEDOUT;
}

/* Stamp ONE upstream GEM Port-ID into every slot of a ...
 * dev/MEASURED-cortina-gpon.c.md sec 78. */
static int cg_us_gem_stamp_range(struct cortina_gpon *cg,
				 const struct gpon_gem_us_range *r,
				 u32 gem, const char *who)
{
	struct hwio io = cg_mac_io(cg);
	u32 n;

	for (n = 0; n < r->count; n++) {
		u32 stuck = 0;

		if (gpon_ind_rmw(&io, &cg_us_port_tbl, gpon_gem_us_index(r, n),
				 GPON_GEM_US_PORT_MASK, gpon_gem_us_port_id(gem),
				 ca_pause_none, &stuck) == 0)
			continue;
		cg_ind_timed_out(cg, &cg_us_port_tbl, stuck);
		if (who && (stuck & CG_TBL_WR))
			dev_warn_ratelimited(cg->dev,
				"%s: us-gem slot %u write access timed out - bind NOT complete\n",
				who, n);
		else if (who)
			dev_warn_ratelimited(cg->dev,
				"%s: us-gem slot %u read access timed out - bind NOT complete\n",
				who, n);
		return -ETIMEDOUT;
	}
	return 0;
}

/* Invalidate a stale T-CONT CAM entry: RMW-clear omci_en + ...
 * dev/MEASURED-cortina-gpon.c.md sec 79. */
static int cg_tcont_unbind(struct cortina_gpon *cg, u32 alloc,
			   bool *write_issued)
{
	/* masked HERE too, not only inside the helper: the dev_info below prints
	 * this value, and it printed the MASKED one before the extraction. */
	alloc = gpon_gem_us_alloc_id(alloc);
	if (cg_tcont_cam_rmw(cg, alloc,
			     CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN |
			     CG_TCONT_INDEX_MASK,	/* clear */
			     0,				/* set nothing */
			     NULL, write_issued))	/* both callers dev_err on the
							 * failure themselves */
		return -ETIMEDOUT;
	dev_info(cg->dev, "T-CONT CAM[alloc %u] invalidated (stale)\n", alloc);
	return 0;
}

/* Bind the OMCC to the T-CONT table: entry[alloc-id = onu-id] ...
 * dev/MEASURED-cortina-gpon.c.md sec 80. */
static int cg_omcc_tcont_bind(struct cortina_gpon *cg, u32 alloc_id)
{
	u16 old = cg->omcc_alloc;
	bool write_issued = false;
	int ret;

	/* masked HERE too: alloc_id is printed, warned on and stored in the
	 * shadow below, and all of those saw the MASKED value before. */
	alloc_id = gpon_gem_us_alloc_id(alloc_id);
	/* `old != 0` used to stand in for "previously bound", which is wrong for the
	 * legal ONU-ID 0: a real 0 -> N reassignment left CAM entry 0 live and able to
	 * burst into the reassigned grant slot.  Ownership of the old CAM is kept
	 * until its invalidation completes; a later supervisor attempt retries it. */
	if ((cg->omcc_alloc_valid || cg->omcc_alloc_pending) && old != alloc_id) {
		ret = cg_tcont_unbind(cg, old, &write_issued);
		if (ret) {
			if (write_issued) {
				cg->omcc_alloc_valid = false;
				cg->omcc_alloc_pending = true;
			}
			cg_transport_down(cg);
			dev_err(cg->dev,
				"T-CONT CAM[alloc %u] could NOT be invalidated; retaining it for retry before binding alloc %u\n",
				old, alloc_id);
			return ret;
		}
		cg->omcc_alloc_valid = false;
		cg->omcc_alloc_pending = false;
		cg_transport_down(cg);
	}

	/* index = 0 (the OMCC T-CONT), omci_en + ploam_en on.  "OMCC" is the
	 * label that keeps this site's two DISTINCT timeout messages -- read
	 * half vs write half -- which a bare -ETIMEDOUT could not tell apart. */
	ret = cg_tcont_cam_rmw(cg, alloc_id,
			       CG_TCONT_INDEX_MASK,		/* clear */
			       CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN,
			       "OMCC", &write_issued);
	if (ret) {
		if (write_issued) {
			cg->omcc_alloc = alloc_id;
			cg->omcc_alloc_valid = false;
			cg->omcc_alloc_pending = true;
		}
		cg_transport_down(cg);
		return ret;
	}

	cg->omcc_alloc_pending = false;
	cg->omcc_alloc = alloc_id;
	cg->omcc_alloc_valid = true;	/* set ONLY here, after both table ops
					 * succeeded: a bind that timed out leaves
					 * the shadow invalid, so the post-O5
					 * supervisor retries it on the next tick */
	dev_info(cg->dev, "OMCC: T-CONT[0] bound to alloc-id %u\n", cg->omcc_alloc);
	return 0;
}

/* Bind the OMCC upstream GEM: us-gem hw indices 0..7 are ...
 * dev/MEASURED-cortina-gpon.c.md sec 81. */
static int cg_omcc_gem_bind(struct cortina_gpon *cg, u32 gem_id)
{
	/* walk the DECLARED OMCC slot run rather than re-deriving it here; the
	 * stamping loop is shared with the data path, and the 12-bit Port-ID mask is
	 * the G.984.3 field width the shared layer states once (gpon_gem_us_port_id) */
	if (cg_us_gem_stamp_range(cg, &cg_us_omcc_slots, gem_id, "OMCC"))
		return -ETIMEDOUT;

	cg->omcc_gem = gpon_gem_us_port_id(gem_id);
	dev_info(cg->dev, "OMCC: us-gem 0..%d bound to GEM port-id %u\n",
		 CG_OMCC_US_GEM_IDX_NUM - 1, cg->omcc_gem);
	return 0;
}

/* One DS GEM CAM entry (vendor aal_gpon_ds_gem_port_set): ...
 * dev/MEASURED-cortina-gpon.c.md sec 82. */
static int cg_ds_gem_set(struct cortina_gpon *cg, u32 gem_id, u32 data)
{
	struct hwio io = cg_mac_io(cg);
	u32 stuck = 0;

	if (gpon_ind_set(&io, &cg_ds_gem_tbl, gpon_gem_us_port_id(gem_id),
			 data, ca_pause_none, &stuck) == 0)
		return 0;
	return cg_ind_timed_out(cg, &cg_ds_gem_tbl, stuck);
}

static int cg_ds_gem_bind(struct cortina_gpon *cg, u32 gem_id, u32 idx)
{
	return cg_ds_gem_set(cg, gem_id,
			     CG_DS_GEM_VLD | CG_DS_GEM_INDEX(idx));
}

/* Invalidate a DS GEM CAM entry: clear the valid bit (and index) for GEM
 * port-id `gem_id` (vendor aal_gpon_ds_gem_port_set with vld=0), so a
 * reassigned DS GEM no longer routes into this ONU's de-encap path. */
static int cg_ds_gem_unbind(struct cortina_gpon *cg, u32 gem_id)
{
	return cg_ds_gem_set(cg, gem_id, 0);		/* vld=0, index=0 */
}

/*
 * The ARMED identity, as the core's reconcile and undo verdicts want it.  Built in
 * ONE place so the two callers cannot describe the same silicon differently.
 */
static void cg_data_armed(const struct cortina_gpon *cg,
			  struct gpon_data_armed *a)
{
	a->alloc = cg->hw_data_alloc;
	a->alloc_bound = cg->data_alloc_bound || cg->data_alloc_pending;
	a->gem = cg->hw_data_gem;
	a->rides_omcc = cg->data_rides_omcc;
	a->installed = cg->data_installed;
}

/* Tear down the armed WAN data path in the vendor ... -- dev/MEASURED-cortina-gpon.c.md sec 83. */
static u8 cg_uni_port[OMCI_UNI_MAX];
static u8 cg_uni_port_n;

static void cg_omci_declare_uni_panel(struct omci_onu *onu, struct device *dev)
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
	/* ★ THE SWITCH PORT PER INSTANCE, FROM THE BOARD -- never computed.  This
	 *   board's mapper answers 0x0101->3 .. 0x0104->0, the G24W's 0x0101->0 ..
	 *   0x0401->3: opposite orders, one with a non-contiguous instance id. */
	ports = of_get_property(np, "ethernet-uni-ports", &ports_len);
	/* ★ THE PLUG-IN TYPE PER INSTANCE, FROM THE BOARD -- never computed.  G.988
	 *   Expected/Sensed type states what the UNI IS: both Luna boards report 47 for
	 *   their GE port and 24 for every FE one, and the model answered 47 for all. */
	type = of_get_property(np, "ethernet-uni-types", &type_len);
	/* ⚠ THE RETURNED LENGTH IS THE ANSWER, NOT THE POINTER. ...
	 * dev/MEASURED-cortina-gpon.c.md sec 84. */
	decl = omci_onu_declare_unis_be(onu, pptp, pptp_len, type, type_len,
					unig, unig_len, cap, cap_len, &why);
	if (decl == OMCI_UNI_DECL_BAD)
		dev_err(dev, "/omci-uni REFUSED: %s -- keeping the single-UNI default\n",
			why);
	cg_uni_port_n = 0;
	if (decl == OMCI_UNI_DECL_OK && pptp_len) {
		/* one byte per declared Ethernet UNI, in the SAME order.  Absent or
		 * the wrong length, the administrative state is still MODELLED and
		 * answered -- it is simply never APPLIED, and that is said out loud. */
		if (ports && ports_len == pptp_len / 2 &&
		    ports_len <= (int)ARRAY_SIZE(cg_uni_port)) {
			memcpy(cg_uni_port, ports, ports_len);
			cg_uni_port_n = (u8)ports_len;
		} else {
			dev_warn(dev,
				 "/omci-uni has no usable ethernet-uni-ports (%d bytes for %d UNIs): the administrative state will be modelled and NEVER APPLIED\n",
				 ports_len, pptp_len / 2);
		}
	}
	of_node_put(np);
}

static int cg_data_teardown(struct cortina_gpon *cg)
{
	const struct gpon_gem_us_range *us_slots = &cg_us_data_slots;
	struct hwio io = cg_mac_io(cg);
	struct gpon_gem_us_range ride;
	struct gpon_data_armed armed;
	struct gpon_data_undo undo;
	u32 i;
	int ret;

	if (!cg->data_alloc_bound && !cg->data_alloc_pending)
		return 0;
	cg->data_installed = false;
	if (cg->wan_ndev)
		netif_carrier_off(cg->wan_ndev);
	cg_data_armed(cg, &armed);
	gpon_data_undo_plan(&armed, cg->omcc_alloc, &undo);
	if (cg->data_rides_omcc &&
	    gpon_gem_us_ride_range(&cg_us_omcc_slots, &ride))
		us_slots = &ride;
	if (undo.drain_tcont) {
		ret = cg_puc_pvtbl_program(cg, CG_DATA_TCONT_IDX, false);
		if (ret)
			goto failed;
		ret = cg_puc_voq_flush(cg, CG_DATA_TCONT_IDX);
		if (ret)
			goto failed;
	}
	for (i = 0; i < us_slots->count; i++) {
		u32 stuck = 0;

		ret = gpon_ind_set(&io, &cg_us_port_tbl,
			gpon_gem_us_index(us_slots, i), GPON_GEM_US_PORT_NONE,
			ca_pause_none, &stuck);
		if (ret) {
			cg_ind_timed_out(cg, &cg_us_port_tbl, stuck);
			goto failed;
		}
	}
	if (undo.unbind_gem) {
		ret = cg_ds_gem_unbind(cg, cg->hw_data_gem);
		if (ret)
			goto failed;
	}
	ret = cg_ds_gem_unbind(cg, CG_MCAST_GEM_ID);
	if (ret)
		goto failed;
	if (undo.unbind_alloc) {
		ret = cg_tcont_unbind(cg, cg->hw_data_alloc, NULL);
		if (ret)
			goto failed;
	}
	/* PON tables are retired. NI accelerator flush completion is a separate
	 * contract: its current void notification cannot certify cache removal. */
	cortina_ni_gpon_data_path_set(0, 0);
	cortina_ni_pon_data_set_tcont(CG_DATA_TCONT_IDX);
	cg->data_alloc_bound = false;
	cg->data_alloc_pending = false;
	cg->hw_data_alloc = 0;
	cg->hw_data_gem = 0;
	cg->data_rides_omcc = false;
	memset(&cg->hw_data_binding, 0, sizeof(cg->hw_data_binding));
	return 0;
failed:
	dev_warn_ratelimited(cg->dev,
		"data PON withdrawal failed (%d), retaining alloc %u/gem %u for retry\n",
		ret, cg->hw_data_alloc, cg->hw_data_gem);
	return ret;
}

/* Stage D — install the OLT-provisioned WAN data path ...
 * dev/MEASURED-cortina-gpon.c.md sec 85. */
static void cg_data_try_install(struct cortina_gpon *cg,
				const struct omci_data_binding *binding)
{
	u32 alloc = binding->alloc_id;
	u32 gem = binding->gem_present ? binding->gem_port : 0;
	const struct gpon_gem_us_range *us_slots = &cg_us_data_slots;
	u32 us_tcont = CG_DATA_TCONT_IDX;
	bool rides_omcc = false;
	struct gpon_gem_us_range ride;
	struct gpon_data_armed armed;
	struct gpon_data_want want;
	enum gpon_data_plan plan;
	u32 i;

	/* ★★ "IS WHAT IS ARMED STILL WHAT THE OLT WANTS?" IS THE ...
	 * dev/MEASURED-cortina-gpon.c.md sec 86. */
	cg_data_armed(cg, &armed);
	want.alloc = (u16)alloc;
	want.alloc_known = binding->alloc_known;
	want.gem = (u16)gem;
	want.omcc_alloc = cg->omcc_alloc;
	want.omcc_up = cg->omcc_up;

	plan = gpon_data_plan_decide(&armed, &want);
	switch (plan) {
	case GPON_DATA_WAIT:
		return;
	case GPON_DATA_KEEP:
		/* Logical ME provenance changed without changing the effective path. */
		cg->hw_data_binding = *binding;
		return;
	case GPON_DATA_TEARDOWN:	/* deprovisioned: undo, nothing follows */
		cg_data_teardown(cg);
		return;
	case GPON_DATA_REPLACE:		/* a DIFFERENT identity is armed: the
					 * stale CAM goes FIRST, then install */
		if (cg_data_teardown(cg))
			return;
		break;
	case GPON_DATA_INSTALL:
		break;
	}

	/* ★★ THE ALLOC-ID -> T-CONT DECISION IS COMMON, and this is ...
	 * dev/MEASURED-cortina-gpon.c.md sec 87. */
	switch (gpon_gem_us_tcont_decide((u16)alloc, cg->omcc_alloc,
					 cg->data_installed)) {
	case GPON_GEM_US_BIND_DONE:
		return;
	case GPON_GEM_US_BIND_IS_OMCC:
			/* ★★★ SINGLE-ALLOC OLT: RIDE THE OMCC'S T-CONT. Rebinding the ...
			 * dev/MEASURED-cortina-gpon.c.md sec 88. */
		if (!gpon_gem_us_ride_range(&cg_us_omcc_slots, &ride)) {
			dev_err(cg->dev,
				"data alloc %u == OMCC alloc but the OMCC slot run cannot spare a queue: NO data path (refusing to steal an OMCI slot)\n",
				alloc);
			return;
		}
		us_slots = &ride;
		us_tcont = CG_OMCC_TCONT_IDX;
		rides_omcc = true;
		dev_info(cg->dev,
			 "data alloc %u == OMCC alloc: single-alloc OLT, data RIDES the OMCC T-CONT %u (US VoQ %u..%u; OMCI keeps the top %u by strict priority)\n",
			 alloc, us_tcont, ride.base,
			 ride.base + ride.count - 1,
			 GPON_GEM_US_OMCI_RESERVED_SLOTS);
		break;
	case GPON_GEM_US_BIND_TCONT:
		/* NULL label: this function gives up SILENTLY on a table ...
		 * dev/MEASURED-cortina-gpon.c.md sec 89. */
		{
			bool write_issued;
			int ret = cg_tcont_cam_rmw(cg, alloc, CG_TCONT_INDEX_MASK,
				CG_TCONT_INDEX(CG_DATA_TCONT_IDX) |
				CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN,
				NULL, &write_issued);

			if (ret) {
				/* A write timeout is not proof that the CAM stayed empty. */
				if (write_issued) {
					cg->data_alloc_pending = true;
					cg->hw_data_alloc = gpon_gem_us_alloc_id(alloc);
					cg->hw_data_gem = gpon_gem_us_port_id(gem);
					cg->data_rides_omcc = false;
					cg->hw_data_binding = *binding;
				}
				return;
			}
		}
		break;
	}

	/* US: every VoQ of the SELECTED T-CONT stamps the data GEM port-id.
	 * @us_slots is the declared data run, or the ride range when this ONU
	 * rides the OMCC T-CONT -- the helper walks whichever it is given. */
	cg->data_alloc_bound = true;
	cg->data_alloc_pending = false;
	cg->data_rides_omcc = rides_omcc;
	cg->hw_data_alloc = gpon_gem_us_alloc_id(alloc);
	cg->hw_data_gem = gpon_gem_us_port_id(gem);
	cg->hw_data_binding = *binding;

	if (cg_us_gem_stamp_range(cg, us_slots, gem, NULL))
		return;

	/* DS: unicast data GEM + the broadcast GEM (DHCP OFFER rides it) */
	if (cg_ds_gem_bind(cg, gem, CG_DATA_GEM_IDX) ||
	    cg_ds_gem_bind(cg, CG_MCAST_GEM_ID, CG_MCAST_GEM_IDX))
		return;

	/* PDC: both intern indices -> CPU port 0, forwarding-engine ...
	 * dev/MEASURED-cortina-gpon.c.md sec 90. */
	for (i = 0; i < 2; i++) {
		u32 idx = CG_DATA_GEM_IDX + i;
		u32 d0 = CG_PDC_D0_COS(0) |
			 CG_PDC_D0_LDPID(CG_LPORT_CPU_0) |
			 CG_PDC_D0_LSPID(CG_LPORT_PON) |
			 CG_PDC_D0_FE_BYPASS | CG_PDC_D0_NO_DROP;

		if (i == 0 && cortina_ni_hw_l3_fwd_active() && cg_hw_l3_ds) {
			d0 = CG_PDC_D0_LDPID(CG_LPORT_L3_WAN) |
			     CG_PDC_D0_LSPID(CG_LPORT_PON);
			dev_info(cg->dev,
				 "PDC: data GEM idx %u -> L3_WAN (HW L3-forward DS armed)\n",
				 idx);
		} else if (i == 0) {
			/* ★ Say it PLAINLY: this is the DS-offload precondition and
			 * its absence is invisible from the L3FE side.  With FE_BYPASS
			 * the DS data GEM skips BOTH forwarding engines, so no DS
			 * main-hash entry can be hit however correct it is. */
			dev_info(cg->dev,
				 "PDC: data GEM idx %u -> CPU_0 + FE_BYPASS (hw_l3_fwd=%d hw_l3_ds=%d) - DS frames BYPASS the L3FE, so no DS HW-flow entry can be hit; set cortina_gpon.hw_l3_ds=1 to route DS into the L3FE\n",
				 idx, cortina_ni_hw_l3_fwd_active(),
				 cg_hw_l3_ds);
		}
		if (cg_pdc_map_write(cg, idx, d0, CG_PDC_D1_POL_ID(idx)))
			return;
		if (i == 0)
			cortina_ni_gpon_ds_route_set(!(d0 & CG_PDC_D0_FE_BYPASS));
	}

	/* PUC: enable the data T-CONT's VoQs, then the flush workaround */
	if (cg_puc_pvtbl_program(cg, us_tcont, true))
		return;
	if (cg_puc_voq_flush(cg, us_tcont))
		return;

	cg->data_installed = true;
	cg->data_rides_omcc = rides_omcc;
	/* the TX header's ldpid and policer id must target the SAME T-CONT, or
	 * the frames are queued where no grant ever arrives */
	cortina_ni_pon_data_set_tcont((u8)us_tcont);
	cg->hw_data_alloc = gpon_gem_us_alloc_id(alloc);	/* record the armed */
	cg->hw_data_gem = gpon_gem_us_port_id(gem);	/* identity so a later
							 * reconfig can
							 * invalidate it */
	/* report the LIVE data-path identity to the L3FE offload backend (the
	 * US hit-action's mcgid/T-CONT source - never a compiled-in constant) */
	cortina_ni_gpon_data_path_set(cg->hw_data_gem, (u8)us_tcont);
	dev_info(cg->dev,
		 "DATA path UP%s: alloc %u -> T-CONT %u, gem %u (US VoQ %u, DS idx %u), bcast %u -> idx %u\n",
		 rides_omcc ? " (riding the OMCC T-CONT)" : "",
		 alloc, us_tcont, gem, gpon_gem_us_index(us_slots, 0),
		 CG_DATA_GEM_IDX, CG_MCAST_GEM_ID, CG_MCAST_GEM_IDX);
	if (cg->wan_ndev)
		netif_carrier_on(cg->wan_ndev);
}

/* Datapath reset on O5 exit. Drops the soft link state so the ...
 * dev/MEASURED-cortina-gpon.c.md sec 91. */
static void cg_transport_down(struct cortina_gpon *cg)
{
	cg->omcc_up = false;
	/* Disarm the transport while retaining the accepted same-identity MIB. */
	spin_lock_bh(&cg->omci_lock);
	cg->omci_active = false;
	spin_unlock_bh(&cg->omci_lock);
	cancel_delayed_work(&cg->veip_avc_work);
	/* Accepted MIB and partial hardware ownership survive transport loss. */
	cg->data_installed = false;
	if (cg->wan_ndev)
		netif_carrier_off(cg->wan_ndev);
}

static void cg_datapath_reset(struct cortina_gpon *cg)
{
	cg_transport_down(cg);
	/* Re-arm the stuck-O1 recovery watchdog: the analog lock can ...
	 * dev/MEASURED-cortina-gpon.c.md sec 127. */
	cg->coldstart_tries = 0;
	/* mod_ and not schedule_: the post-O5 supervisor leaves this delayed work
	 * permanently PENDING, and schedule_delayed_work() on a pending work is a NO-OP
	 * -- the watchdog would inherit what remained of the supervisor's 30 s deadline
	 * and could re-roll the SerDes in the middle of a healthy Deactivate re-range. */
	cg_sched_mod(cg, &cg->coldstart_work, 15 * HZ);
	dev_warn(cg->dev, "O5 exit: datapath reset (OMCC + data down, CAM shadow kept)\n");
}

/* Try to bring the OMCC link up: needs O5 + HW-filled omci_port.en. */
static int cg_identity_prepare(struct cortina_gpon *cg)
{
	int ret;

	cg_ingress_stop(cg);
	writel(cg_mac_rd(cg, CG_REG_ONU_CTL) & ~CG_ONU_CTL_EN,
	       cg->mac + CG_REG_ONU_CTL);
	cg->activated = false;
	cg_transport_down(cg);
	cancel_delayed_work(&cg->coldstart_work);
	ret = cg_data_teardown(cg);
	if (ret) {
		cg_sched_mod(cg, &cg->coldstart_work, 15 * HZ);
		return ret;
	}
	if (cg->omci) {
		spin_lock_bh(&cg->omci_lock);
		/* A re-arm keeps the declared UNI panel; a first one has none to
		 * keep and may not read the object at all.  The flag is the only
		 * thing that tells the two apart, which is why it decides here. */
		if (cg->omci_initialized) {
			omci_onu_reinit(cg->omci, cg->sn,
					OMCI_MDS_POISON_SEED);
		} else {
			omci_onu_init(cg->omci, cg->sn, OMCI_MDS_POISON_SEED);
			cg_omci_declare_uni_panel(cg->omci, cg->dev);
		}
		cg->omci_initialized = true;
		spin_unlock_bh(&cg->omci_lock);
	}
	return 0;
}

static void cg_data_reconcile(struct cortina_gpon *cg)
{
	struct omci_data_binding binding;

	if (!cg->omci_initialized)
		return;
	spin_lock_bh(&cg->omci_lock);
	omci_data_binding_snapshot(cg->omci, cg->omcc_gem,
				   CG_MCAST_GEM_ID, &binding);
	spin_unlock_bh(&cg->omci_lock);
	cg_data_try_install(cg, &binding);
}

static void cg_omcc_try_up(struct cortina_gpon *cg, u8 state)
{
	u32 omci_port, want, onu;

	if (state != CG_STATE_OPERATION)
		return;
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	if (CG_ONU_STATE(onu) != CG_STATE_OPERATION ||
	    !cg->omcc_alloc_valid || cg->omcc_alloc_pending ||
	    cg->omcc_alloc != CG_ONU_ID(onu)) {
		cg_transport_down(cg);
		return;
	}
	omci_port = cg_mac_rd(cg, CG_REG_OMCI_PORT);
	if (!(omci_port & CG_OMCI_PORT_EN)) {
		/* an en=0 Configure_Port-ID transient: write NOTHING and keep the
		 * link -- the following en=1 re-latch binds the final id */
		dev_info(cg->dev, "O5 but omci_port not enabled yet (0x%08x)\n",
			 omci_port);
		return;
	}
	want = CG_OMCI_PORT_ID(omci_port);

	/* ★★★ A ONE-SHOT GUARD HERE GAVE UP ON EVERY LATER PORTID ...
	 * dev/MEASURED-cortina-gpon.c.md sec 92. */
	switch (gpon_omcc_decide(!!(omci_port & CG_OMCI_PORT_EN), (u16)want,
				 cg->omcc_up, (u16)cg->omcc_gem)) {
	case GPON_OMCC_IGNORE:
	case GPON_OMCC_UNCHANGED:
		return;
	case GPON_OMCC_REBIND:
		if (cg_omcc_gem_bind(cg, CG_OMCI_PORT_ID(omci_port))) {
			/* the shadow keeps the OLD id (bind writes it only on
			 * success), so the next PORTID event retries to
			 * convergence rather than latching a half-done rebind */
			dev_warn_ratelimited(cg->dev,
				"OMCC: rebind to gem %u FAILED - keeping %u, will retry on the next event\n",
				want, cg->omcc_gem);
			return;
		}
		dev_info(cg->dev,
			 "OMCC gem re-assigned mid-O5 -> %u (transport rebound; responder session kept)\n",
			 cg->omcc_gem);
		return;
	case GPON_OMCC_INSTALL:
		break;
	}
	/* LATCH ONLY ON SUCCESS.  A failed bind leaves `omcc_up` clear, so the
	 * next PLOAM/state event runs this again - the retry is the event
	 * stream itself, and it costs nothing while the bind works. */
	if (cg_omcc_gem_bind(cg, CG_OMCI_PORT_ID(omci_port))) {
		dev_warn_ratelimited(cg->dev,
			"OMCC: GEM bind failed at O5 - NOT latching omcc_up, will retry on the next event\n");
		return;
	}
	cg->omcc_up = true;
	dev_info(cg->dev, "OMCC link UP (alloc %u, gem %u) - ready for OMCI\n",
		 cg->omcc_alloc, cg->omcc_gem);

	/* Stage C: arm the G.988 responder on a fresh MIB. The ME-256 ...
	 * dev/MEASURED-cortina-gpon.c.md sec 93. */
	if (cg->omci) {
		char sn_str[13];

		spin_lock_bh(&cg->omci_lock);
		/* CUT SITE: the ME model + MIB reset MOVED to omci_onu_init() ...
		 * dev/MEASURED-cortina-gpon.c.md sec 94. */
		if (!cg->omci_initialized) {
			omci_onu_init(cg->omci, cg->sn, OMCI_MDS_POISON_SEED);
			cg_omci_declare_uni_panel(cg->omci, cg->dev);
			cg->omci_initialized = true;
		}
		cg->omci_active = true;
		spin_unlock_bh(&cg->omci_lock);
		/* ★ RESUME ANY APPLY THE RESPONDER WAS NOT UP FOR: the drain returns
		 *   early while inactive without consuming anything, so without this the
		 *   obligation waits for an unrelated OMCI message to carry it. */
		cg_sched(cg, &cg->uni_apply_work, 0);
		cg->veip_avc_retry_ms = 0;
		cg_sched(cg, &cg->veip_avc_work, 31 * HZ);
		gpon_sn_format(cg->sn, sn_str);
		dev_info(cg->dev, "OMCI responder armed (%u MIB rows, mds seed 200, sn %s)\n",
			 cg->omci->nrows, sn_str);
	}
}

/* US OMCI TX (Stage C): the 48-byte PDU (trailer + MIC ...
 * dev/MEASURED-cortina-gpon.c.md sec 128. */
static int cg_omci_tx(struct cortina_gpon *cg, const u8 *pdu48)
{
	int ret = -ENODEV;

	if (IS_REACHABLE(CONFIG_CORTINA_NI))
		ret = cortina_ni_pon_tx(pdu48, OMCI_LEN);
	if (ret) {
		cg->omci_tx_fail++;
		dev_warn_ratelimited(cg->dev, "US OMCI TX failed (%d)\n", ret);
	} else {
		cg->omci_tx++;
		/* The frame is on its way to the PUC; read the OMCI-specific
		 * control-packet counter once it has arrived, while its
		 * clear-on-read window still holds it (cg_puc_ctrl_sample). */
		cg_sched(cg, &cg->puc_cnt_work,
				      msecs_to_jiffies(CG_PUC_CNT_TX_DELAY_MS));
	}
	/* Returned, not swallowed: a solicited response can be left to the OLT's
	 * own retry, but an unsolicited AVC has no such backstop -- see
	 * cg_veip_avc_work(). */
	return ret;
}

/* What ME 263 ANI-G #10/#14 currently serve the OLT, and ...
 * dev/MEASURED-cortina-gpon.c.md sec 95. */
static void cg_seq_cdbm(struct seq_file *m, s32 cdbm)
{
	if (cdbm == CG_DDM_CDBM_NONE)
		seq_puts(m, "-inf");
	else
		seq_printf(m, "%d", cdbm);
}

static void cg_optic_anig_show(struct cortina_gpon *cg, struct seq_file *m)
{
	if (!cg->omci) {
		seq_puts(m, "optic_anig     = (responder not allocated)\n");
		return;
	}
	seq_printf(m, "optic_anig     = %s  me263 #10 rx=0x%04x #14 tx=0x%04x  (G.988 0.002 dB units)\n",
		   cg->omci->anig_live ? "live" : "FALLBACK (static, no DDM sample yet)",
		   cg->omci->anig_rx_level, cg->omci->anig_tx_level);
}

/* Sample the optic's SFF-8472 A2h diagnostics, print them to ...
 * dev/MEASURED-cortina-gpon.c.md sec 96. */
static void cg_optic_sample(struct cortina_gpon *cg, struct seq_file *m)
{
	struct cg_bosa_ddm d;
	s32 rx_cdbm, tx_cdbm;

	if (cg_bosa_ddm_read(cg->dev, &d) != CG_DDM_OK) {
		if (m) {
			seq_printf(m, "optic_ddm      = %s\n",
				   cg_ddm_status_str(d.status));
			cg_optic_anig_show(cg, m);
		}
		return;
	}

	rx_cdbm = cg_ddm_uw10_to_cdbm(d.rx_pwr);
	tx_cdbm = cg_ddm_uw10_to_cdbm(d.tx_pwr);

	if (cg->omci) {
		u16 rx = cg_ddm_cdbm_to_omci(rx_cdbm);
		u16 tx = cg_ddm_cdbm_to_omci(tx_cdbm);

		spin_lock_bh(&cg->omci_lock);
		/* CUT SITE: the ME 263 optical attributes MOVED to omci_onu_set_optical() in
		 * drivers/net/gpon/gpon_omci_me.c (the i2c DDM read that feeds it stays here — it is
		 * hardware) */
		omci_onu_set_optical(cg->omci, rx, tx);
		spin_unlock_bh(&cg->omci_lock);
	}

	if (m) {
		unsigned int i;

		seq_printf(m, "optic_ddm      = live (SFF-8472 A2h 0x%02x-0x%02x)\n",
			   CG_DDM_BASE, CG_DDM_BASE + CG_DDM_LEN - 1);
		/* The RAW word sits beside every scaled value on purpose: the
		 * 0.1 uW LSB is the one thing about RX power this module has not
		 * independently confirmed (see cortina-gpon-ddm.h), so a reader must
		 * always be able to re-derive the level without a firmware change. */
		seq_printf(m, "optic_rx_raw: 0x%04x optic_rx_cdbm: ", d.rx_pwr);
		cg_seq_cdbm(m, rx_cdbm);
		seq_printf(m, " optic_tx_raw: 0x%04x\n", d.tx_pwr);
		seq_printf(m, "optic_env:   temp_dc=%d bias_ua=%u tx_cdbm=",
			   cg_ddm_temp_dc(d.temp), cg_ddm_bias_ua(d.bias));
		cg_seq_cdbm(m, tx_cdbm);
		seq_printf(m, " vcc_mv=%u\n", cg_ddm_vcc_mv(d.vcc));
		seq_printf(m, "optic_ddm_raw: %02x..%02x =",
			   CG_DDM_BASE, CG_DDM_BASE + CG_DDM_LEN - 1);
		for (i = 0; i < CG_DDM_LEN; i++)
			seq_printf(m, " %02x", d.raw[i]);
		seq_putc(m, '\n');
		cg_optic_anig_show(cg, m);
	}
}

/* The ~31s post-O5 VEIP (ME 329) operational-up AVC: the OLT waits for it
 * before marking the service matched/active (its Match State stays Initial
 * until the ONU reports the WAN egress port up). */
static void cg_veip_avc_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon,
					       veip_avc_work);
	u8 frame[OMCI_LEN];
	bool emit = false;

	/* Publish a live optical reading before the AVC: this fires ~31s after
	 * O5, i.e. just as the OLT begins auditing ANI-G, so its first optical
	 * GET already gets a measurement instead of the static fallback. */
	mutex_lock(&cg->sn_lock);
	cg_optic_sample(cg, NULL);

	spin_lock_bh(&cg->omci_lock);
	if (cg->omci_active && !cg->omci->avc_veip_up_sent) {
		/* CUT SITE: building the VEIP oper-state AVC MOVED to omci_onu_emit_veip_up_avc() in
		 * drivers/net/gpon/gpon_omci_core.c (the workqueue that times it stays here) */
		omci_onu_emit_veip_up_avc(cg->omci, frame);
		emit = true;
	}
	spin_unlock_bh(&cg->omci_lock);
	if (!emit)
		goto out;

	if (!cg_omci_tx(cg, frame)) {
		cg->veip_avc_retry_ms = 0;
		dev_info(cg->dev, "VEIP oper-up AVC emitted (~31s post-O5)\n");
		goto out;
	}

	/* The TX failed. The responder latches avc_veip_up_sent at ...
	 * dev/MEASURED-cortina-gpon.c.md sec 97. */
	spin_lock_bh(&cg->omci_lock);
	if (cg->omci_active)
		cg->omci->avc_veip_up_sent = false;
	spin_unlock_bh(&cg->omci_lock);

	cg->veip_avc_retry_ms = cg->veip_avc_retry_ms
		? min(cg->veip_avc_retry_ms * 2u,
		      (unsigned int)CG_VEIP_AVC_RETRY_MAX_MS)
		: CG_VEIP_AVC_RETRY_MIN_MS;
	dev_warn(cg->dev, "VEIP oper-up AVC TX failed; retrying in %u ms\n",
		 cg->veip_avc_retry_ms);
	cg_sched(cg, &cg->veip_avc_work,
			      msecs_to_jiffies(cg->veip_avc_retry_ms));out:
	mutex_unlock(&cg->sn_lock);
}

#ifdef CONFIG_GPON_OLT_DIAG
/* The far end's own conversation.  Written and read under cg->omci_lock, the
 * lock that already serializes this shell's OMCI path; the reader copies ONE
 * record at a time so /proc/oltcap never holds it across a whole dump. */
static struct gpon_olt_capture cg_olt_cap;

/* The PON-MAC numbers its states from 0 (0 = O1), the core enum from 1.  A
 * value the MAC does not name maps to COULD NOT ASK, never to a state. */
static u8 cg_olt_ostate(struct cortina_gpon *cg)
{
	u8 st = CG_ONU_STATE(cg_mac_rd(cg, CG_REG_GPON_ONU));

	return st < 7 ? (u8)(st + 1) : GPON_OLT_OSTATE_UNKNOWN;
}

static int cg_oltcap_show(struct seq_file *s, void *v)
{
	struct cortina_gpon *cg = READ_ONCE(cg_singleton);
	char line[160];
	unsigned int i, n;

	if (!cg)
		return 0;
	spin_lock_bh(&cg->omci_lock);
	n = gpon_olt_capture_count(&cg_olt_cap);
	gpon_olt_diag_header(&cg_olt_cap, line, sizeof(line));
	spin_unlock_bh(&cg->omci_lock);
	seq_printf(s, "%s\n", line);
	for (i = 0; i < n; i++) {
		/* zero-initialised so a NULL fetch can never leave an uninitialised
		 * record in scope; the loop breaks on it anyway. */
		struct gpon_olt_rec r = { 0 };
		const struct gpon_olt_rec *q;

		spin_lock_bh(&cg->omci_lock);
		q = gpon_olt_capture_at(&cg_olt_cap, i);
		if (q)
			r = *q;
		spin_unlock_bh(&cg->omci_lock);
		if (!q)
			break;
		if (gpon_olt_diag_line(&r, line, sizeof(line)))
			seq_printf(s, "%s\n", line);
	}
	return 0;
}
#endif /* CONFIG_GPON_OLT_DIAG */

/* Per-message OMCI trace. DEFAULT OFF. The always-on ...
 * dev/MEASURED-cortina-gpon.c.md sec 98. */
static bool cg_omci_trace;
module_param_named(omci_trace, cg_omci_trace, bool, 0644);
MODULE_PARM_DESC(omci_trace, "log one line per downstream OMCI PDU: message type, ME class/instance and, for a Get, the requested vs answered vs unmodelled attribute masks (default OFF)");

/* Emit one trace line for the PDU just processed. @resp/@n ...
 * dev/MEASURED-cortina-gpon.c.md sec 129. */
static void cg_omci_trace_one(struct cortina_gpon *cg, const u8 *pdu,
			      unsigned int len, const u8 *resp, int n)
{
	static DEFINE_RATELIMIT_STATE(rs, 5 * HZ, 512);
	char line[128];

	/* ★★ THE WHOLE LINE IS THE CORE'S (gpon_omci_diag_line), ...
	 * dev/MEASURED-cortina-gpon.c.md sec 130. */
	if (!IS_ENABLED(CONFIG_GPON_OMCI_DIAG) || !__ratelimit(&rs))
		return;
	gpon_omci_diag_line(pdu, len, n == OMCI_LEN ? resp : NULL, n,
			    line, sizeof(line));
	dev_info(cg->dev, "OMCI DS: %s\n", line);
}

/* DS OMCI receive: the NI CPU-RX hook hands us each OMCI PDU ...
 * dev/MEASURED-cortina-gpon.c.md sec 99. */
static void cg_rx_omci(const u8 *pdu, unsigned int len)
{
	struct cortina_gpon *cg = READ_ONCE(cg_singleton);
	u8 mt;
	u32 generation;
	unsigned long flags;
	bool queued = false;

	if (!cg)
		return;
	generation = READ_ONCE(cg->ingress_generation);
	if (len < 8) {
		cg->omci_rx_short++;
		return;
	}
	cg->omci_rx++;

	mt = pdu[2];
	/* log the first PDUs + then 1-in-64 (the MIB-upload walk is chatty) */
	if (cg->omci_rx <= 24 || !(cg->omci_rx & 63)) {
		char det[96];

		/* ★ THE DECODE IS THE CORE'S (gpon_omci_describe), ...
		 * dev/MEASURED-cortina-gpon.c.md sec 100. */
		gpon_omci_describe(pdu, len, det, sizeof(det));
		dev_info(cg->dev, "DS OMCI #%u: %s\n", cg->omci_rx, det);
	}

	/* DS MIC self-check on the first PDUs: decides the CRC-32 ...
	 * dev/MEASURED-cortina-gpon.c.md sec 101. */
	if (len >= OMCI_LEN && cg->omci_ds_crc_ok + cg->omci_ds_crc_bad < 16) {
		enum gpon_mic_conv conv = gpon_omci_mic_conv(pdu, len);
		u32 want = gpon_omci_mic_stamped(pdu);
		u32 be = omci_mic_compute(pdu);	/* the core's ONE spelling */
		u32 le = gpon_omci_mic_zlib_le(pdu);

		if (conv == GPON_MIC_CONV_AAL5_BE)
			cg->omci_ds_crc_ok++;
		else
			cg->omci_ds_crc_bad++;
		if (cg->omci_rx <= 4)
			dev_info(cg->dev, "DS OMCI MIC self-check: %s (want %08x be %08x le %08x)\n",
				 gpon_mic_conv_name(conv), want, be, le);
	}

	/* ★★★ THE MIC GATE, BEFORE STAGE D. The self-check above is ...
	 * dev/MEASURED-cortina-gpon.c.md sec 102. */
	if (!omci_mic_ok(pdu, len)) {
		cg->omci_rx_bad_mic++;
		dev_warn_ratelimited(cg->dev,
			"DS OMCI: MIC does not verify (len %u, mt 0x%02x) -- frame DISCARDED, "
			"acting on nothing in it; the OLT's AR retransmit is the recovery\n",
			len, mt & 0x1f);
		return;
	}

	/* Admission precedes common acceptance; AR retransmission can retry a
	 * refused frame without an acknowledged but unapplied mutation. */
	spin_lock_irqsave(&cg->evt_lock, flags);
	if (cg->ingress_open && generation == cg->ingress_generation &&
	    cg->evt_head - cg->evt_tail < CG_EVT_OMCI_LIMIT) {
		struct cg_evt *ev = &cg->evt[cg->evt_head % CG_EVT_RING_SZ];

		ev->type = CG_EVT_OMCI;
		memcpy(ev->pdu, pdu, OMCI_LEN);
		cg->evt_head++;
		queued = true;
	} else {
		cg->omci_queue_drop++;
	}
	spin_unlock_irqrestore(&cg->evt_lock, flags);
	if (queued)
		cg_sched_now(cg, &cg->isr_work);
}

/* Bottom half: drain the event ring and run the FSM tracker + ...
 * dev/MEASURED-cortina-gpon.c.md sec 103. */
static enum omci_uni_apply_rc cg_uni_apply_slot(void *sh, u8 slot, bool locked)
{
	struct cortina_gpon *cg = sh;

	if (slot >= cg_uni_port_n) {
		dev_warn_once(cg->dev,
			      "UNI slot %u has no declared switch port; its administrative state is modelled and not applied\n",
			      slot);
		return OMCI_UNI_NO_PORT;
	}
	return cortina_ni_uni_admin_set(cg_uni_port[slot], locked) ?
		OMCI_UNI_TRANSIENT : OMCI_UNI_APPLIED;
}

static void cg_uni_apply_rearm(void *sh, u8 slot)
{
	struct cortina_gpon *cg = sh;

	spin_lock_bh(&cg->omci_lock);
	omci_uni_mark_changed(&cg->omci->pptp_eth_uni, slot);
	spin_unlock_bh(&cg->omci_lock);
}

static const struct omci_uni_apply_ops cg_uni_apply_ops = {
	.apply = cg_uni_apply_slot,
	.rearm = cg_uni_apply_rearm,
};

static void cg_uni_apply_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon,
					       uni_apply_work);
	u8 changed, admin[OMCI_UNI_MAX], n, i;

	/* ★★ HELD ACROSS THE SNAPSHOT AND THE SLEEPING APPLY.  cg_identity_prepare
	 *    can re-initialise the model under this work, and a lock snapshotted
	 *    from the OLD identity must not be driven onto a port after the NEW
	 *    one is committed.  The OMCI spin section inside stays short. */
	mutex_lock(&cg->sn_lock);
	spin_lock_bh(&cg->omci_lock);
	if (!cg->omci_active) {
		/* Nothing is CONSUMED here, so nothing is lost -- and the activation
		 * path schedules this work, so a lock Set before the responder came up
		 * resumes without needing an unrelated OMCI message to carry it. */
		spin_unlock_bh(&cg->omci_lock);
		mutex_unlock(&cg->sn_lock);
		return;
	}
	changed = omci_uni_take_changed(&cg->omci->pptp_eth_uni);
	n = cg->omci->pptp_eth_uni.n;
	for (i = 0; i < n && i < OMCI_UNI_MAX; i++)
		admin[i] = cg->omci->pptp_eth_uni.admin[i];
	spin_unlock_bh(&cg->omci_lock);

	/* The WALK, the RETENTION of a failed obligation and the rule ...
	 * dev/MEASURED-cortina-gpon.c.md sec 104. */
	if (omci_uni_apply_run(&cg_uni_apply_ops, cg, changed, n, admin))
		cg_sched(cg, &cg->uni_apply_work, HZ);
	mutex_unlock(&cg->sn_lock);
}

static void cg_omci_process(struct cortina_gpon *cg, const u8 *pdu)
{
	struct omci_accepted accepted = { .kind = OMCI_ACCEPT_NONE };
	struct omci_data_binding binding;
	u8 resp[OMCI_LEN];
	bool uni_pending = false;
	int n = 0;

	spin_lock_bh(&cg->omci_lock);
	if (cg->omci_active) {
		n = omci_onu_input_ex(cg->omci, pdu, OMCI_LEN, resp, &accepted);
		if (accepted.kind != OMCI_ACCEPT_NONE)
			omci_data_binding_snapshot(cg->omci, cg->omcc_gem,
						   CG_MCAST_GEM_ID, &binding);
		/* read, never drained here: the drain runs where it may sleep */
		uni_pending = cg->omci->pptp_eth_uni.changed != 0;
	}
	spin_unlock_bh(&cg->omci_lock);
	if (uni_pending)
		cg_sched(cg, &cg->uni_apply_work, 0);
	if (accepted.kind != OMCI_ACCEPT_NONE)
		cg_data_try_install(cg, &binding);
	if (accepted.kind == OMCI_ACCEPT_RESET)
		cg_sched(cg, &cg->veip_avc_work, 31 * HZ);
	if (n == OMCI_LEN)
		cg_omci_tx(cg, resp);
	if (unlikely(cg_omci_trace))
		cg_omci_trace_one(cg, pdu, OMCI_LEN, resp, n);
#ifdef CONFIG_GPON_OLT_DIAG
	/* The clock is read HERE and handed in: the core tier reads none, which
	 * is what makes a capture replay deterministically. */
	spin_lock_bh(&cg->omci_lock);
	gpon_olt_capture_exchange(&cg_olt_cap,
				  div_u64(ktime_get_boottime_ns(), 1000),
				  cg_olt_ostate(cg), pdu, OMCI_LEN,
				  n == OMCI_LEN ? resp : NULL, n);
	spin_unlock_bh(&cg->omci_lock);
#endif
}

static void cg_isr_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(work, struct cortina_gpon, isr_work);
	struct cg_evt ev;
	unsigned long flags;

	for (;;) {
		mutex_lock(&cg->sn_lock);
		/* ★ INSIDE THE LOCK, for the same reason as the coldstart work:
		 *   a pre-lock read can go stale between the read and the lock,
		 *   and teardown sets the flag holding exactly this lock. */
		if (READ_ONCE(cg->stopping)) {
			mutex_unlock(&cg->sn_lock);
			return;
		}
		spin_lock_irqsave(&cg->evt_lock, flags);
		if (cg->evt_tail == cg->evt_head) {
			spin_unlock_irqrestore(&cg->evt_lock, flags);
			mutex_unlock(&cg->sn_lock);
			break;
		}
		ev = cg->evt[cg->evt_tail % CG_EVT_RING_SZ];
		cg->evt_tail++;
		spin_unlock_irqrestore(&cg->evt_lock, flags);

		if (ev.type == CG_EVT_OMCI) {
			cg_omci_process(cg, ev.pdu);
			mutex_unlock(&cg->sn_lock);
			continue;
		}
		if (ev.intr & CG_INT_ONU_ST_CHG) {
			u8 last = cg->last_state;

			if (last != ev.state)
				dev_info(cg->dev, "FSM %s -> %s (onu-id %u)\n",
					 cg_state_name[last & 7],
					 cg_state_name[ev.state & 7], ev.id);
			/* O5 exit = link down (vendor condition): leaving Operation ...
			 * dev/MEASURED-cortina-gpon.c.md sec 105. */
			if (cg_link_down_transition(last, ev.state))
				cg_datapath_reset(cg);
			cg->last_state = ev.state;
		}

		if ((ev.intr & CG_INT_ONU_ID) && ev.id != CG_ONU_ID_NONE)
			if (cg_omcc_tcont_bind(cg, ev.id))
				dev_warn_ratelimited(cg->dev,
					"OMCC: T-CONT bind failed for alloc %u - the post-O5 supervisor retries\n",
					ev.id);

		if (ev.intr & CG_INT_DACT)
			dev_warn(cg->dev, "Deactivate_ONU-ID received\n");

		if (ev.intr & CG_INT_KSW)
			dev_info(cg->dev, "Key_Switching_Time (AES rekey = next phase, no AES keys in use)\n");

		/* stock recomputes frame_var on every DS PLOAM (Extended_
		 * Burst_Length may arrive/change any time in O2+) */
		if (ev.intr & (CG_INT_PLOAMD | CG_INT_ONU_ST_CHG))
			cg_frame_var_update(cg);

			/* OMCC bring-up on PORTID-in-O5 (vendor path), and ALSO on ...
			 * dev/MEASURED-cortina-gpon.c.md sec 106. */
		if (ev.intr & (CG_INT_PORTID | CG_INT_ONU_ST_CHG))
			cg_omcc_try_up(cg, ev.state);
		cg_data_reconcile(cg);
		mutex_unlock(&cg->sn_lock);
	}

	/* Reconcile the soft state against the LIVE FSM register ...
	 * dev/MEASURED-cortina-gpon.c.md sec 107. */
	mutex_lock(&cg->sn_lock);
	/* ★ THE SECOND SECTION RECHECKS TOO: teardown can set the gate between the
	 *   empty-ring unlock above and this lock, and what follows reprograms
	 *   hardware. */
	if (READ_ONCE(cg->stopping)) {
		mutex_unlock(&cg->sn_lock);
		return;
	}
	if (cg->ingress_open) {
		u32 onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
		u8 live = CG_ONU_STATE(onu);
		u8 id = CG_ONU_ID(onu);

		if (live == CG_STATE_OPERATION) {
			if (cg->last_state != CG_STATE_OPERATION) {
				dev_info(cg->dev,
					 "reconcile: live FSM is %s while the tracker says %s (%u events dropped) - replaying from the register\n",
					 cg_state_name[CG_STATE_OPERATION],
					 cg_state_name[cg->last_state & 7],
					 cg->evt_drop);
				cg->last_state = CG_STATE_OPERATION;
			}
			/* A lost Assign_ONU-ID leaves the OMCC T-CONT unbound, so the ...
			 * dev/MEASURED-cortina-gpon.c.md sec 108. */
			if (id != CG_ONU_ID_NONE &&
			    (!cg->omcc_alloc_valid || cg->omcc_alloc_pending ||
			     cg->omcc_alloc != id))
				if (cg_omcc_tcont_bind(cg, id))
					dev_warn_ratelimited(cg->dev,
						"OMCC: T-CONT bind failed for alloc %u - the post-O5 supervisor retries\n",
						id);
			/* A lost DS-PLOAM edge leaves us.frame_var stale = a US
			 * burst misaligned in the grant window.  Idempotent: it
			 * writes only on a genuine change, and stock recomputes it
			 * on every received DS PLOAM. */
			cg_frame_var_update(cg);
			if (!cg->omcc_up)
				cg_omcc_try_up(cg, live);
		}
	}

	/* Stage D: (re-)install the data path once the OMCC is up and both halves of
	 * the provisioning are known (cg_rx_omci kicks this work on ME 262/268). */
	cg_data_reconcile(cg);
	mutex_unlock(&cg->sn_lock);
}

/* Service one interrupt group (vendor __do_intr_isp): read ...
 * dev/MEASURED-cortina-gpon.c.md sec 131. */
static u32 cg_intr_group_service(struct cortina_gpon *cg, u32 sts_off, u32 en_off)
{
	u32 intre, intrs;

	intre = readl(cg->mac + en_off);
	intrs = readl(cg->mac + sts_off);
	writel(0, cg->mac + en_off);
	writel(intrs & intre, cg->mac + sts_off);	/* W1C */
	writel(intre, cg->mac + en_off);
	return intrs & intre;
}

/* Top-level ISR on the shared NE global line (GIC SPI 1). ...
 * dev/MEASURED-cortina-gpon.c.md sec 109. */
static irqreturn_t cg_isr(int irq, void *data)
{
	struct cortina_gpon *cg = data;
	bool pending = false, queued = false;
	u32 glb_ie, ie, top, src;
	u32 generation = READ_ONCE(cg->ingress_generation);
	int pass;

	/* mask the PON aggregate at the GLB level (vendor __pon_top_intr_mask) */
	glb_ie = readl(cg->glb + CG_GLB_PON_INTEN0);
	writel(glb_ie & ~CG_PON_INT0_PON_MAC, cg->glb + CG_GLB_PON_INTEN0);

	for (pass = 0; pass < 32; pass++) {
		ie = readl(cg->mac + CG_REG_INT_TOP_EN);
		writel(0, cg->mac + CG_REG_INT_TOP_EN);
		top = readl(cg->mac + CG_REG_INT_TOP) & ie;	/* read-clears */

		if (top & BIT(0)) {
			src = cg_intr_group_service(cg, CG_REG_INT, CG_REG_INT_EN);
			if (src) {
				u32 onu = cg_mac_rd(cg, CG_REG_GPON_ONU);

				spin_lock(&cg->evt_lock);
				if (cg->ingress_open && generation == cg->ingress_generation &&
				    cg->evt_head - cg->evt_tail < CG_EVT_RING_SZ) {
					struct cg_evt *ev =
						&cg->evt[cg->evt_head % CG_EVT_RING_SZ];

					ev->type = CG_EVT_IRQ;
					ev->intr = src;
					ev->state = CG_ONU_STATE(onu);
					ev->id = CG_ONU_ID(onu);
					cg->evt_head++;
					queued = true;
				} else {
					cg->evt_drop++;
				}
				spin_unlock(&cg->evt_lock);
			}
		}
		/* groups 2/3/4 ship enable=0; still run the W1C/re-arm bracket */
		if (top & BIT(1))
			cg_intr_group_service(cg, CG_REG_INT2, CG_REG_INT2_EN);
		if (top & BIT(2))
			cg_intr_group_service(cg, CG_REG_INT3, CG_REG_INT3_EN);
		if (top & BIT(3))
			cg_intr_group_service(cg, CG_REG_INT4, CG_REG_INT4_EN);

		writel(ie, cg->mac + CG_REG_INT_TOP_EN);
		if (!top)
			break;
		pending = true;
	}

	/* ack the ne_ictl line (harmless if the status is a pure level view;
	 * needed if it latches — the vendor per-ictl irqchip acks it this way) */
	writel(CG_NE_ICTL_PON_LINE, cg->glb + CG_GLB_NE_ICTL_STS);
	/* unmask the PON aggregate (vendor __pon_top_intr_unmask) */
	writel(glb_ie | CG_PON_INT0_PON_MAC, cg->glb + CG_GLB_PON_INTEN0);

	if (queued)
		cg_sched_now(cg, &cg->isr_work);
	if (!pending)
		return IRQ_NONE;	/* shared line, not ours */
	cg->irq_count++;
	return IRQ_HANDLED;
}

/* Arm the interrupt path (vendor aal_gpon_intr_init order): ...
 * dev/MEASURED-cortina-gpon.c.md sec 132. */
static int cg_intr_setup(struct cortina_gpon *cg, struct platform_device *pdev)
{
	u32 v;
	int ret;

	cg->irq = platform_get_irq(pdev, 0);
	if (cg->irq < 0) {
		dev_warn(cg->dev, "no interrupt in DT (%d) - post-O5 servicing OFF\n",
			 cg->irq);
		return cg->irq;
	}
	/* request BEFORE unmasking the HW gates so no edge is lost */
	ret = devm_request_irq(cg->dev, cg->irq, cg_isr, IRQF_SHARED,
			       DRV_NAME, cg);
	if (ret) {
		dev_warn(cg->dev, "request_irq(%d) failed: %d\n", cg->irq, ret);
		return ret;
	}

	cg_mac_intr_arm(cg);	/* the four MAC int groups + int_top */

	/* GLB aggregation: PON_MACe (level 1) + ne_ictl line 5 (level 2).
	 * RMW set only our bits — other ne_ictl lines belong to the NI. */
	v = readl(cg->glb + CG_GLB_PON_INTEN0);
	writel(v | CG_PON_INT0_PON_MAC, cg->glb + CG_GLB_PON_INTEN0);
	v = readl(cg->glb + CG_GLB_NE_ICTL_EN);
	writel(v | CG_NE_ICTL_PON_LINE, cg->glb + CG_GLB_NE_ICTL_EN);

	dev_info(cg->dev, "interrupts armed: irq %d, int_en=0x%08x int_top_en=0x%x pon_inten0=0x%08x ne_ictl_en=0x%08x\n",
		 cg->irq, readl(cg->mac + CG_REG_INT_EN),
		 readl(cg->mac + CG_REG_INT_TOP_EN),
		 readl(cg->glb + CG_GLB_PON_INTEN0),
		 readl(cg->glb + CG_GLB_NE_ICTL_EN));
	return 0;
}

static void cg_intr_teardown(struct cortina_gpon *cg)
{
	u32 v;

	if (cg->irq >= 0) {
		/* close the gates innermost-out first, so nothing can queue
		 * more work behind the flush below */
		writel(0, cg->mac + CG_REG_INT_TOP_EN);
		v = readl(cg->glb + CG_GLB_NE_ICTL_EN);
		writel(v & ~CG_NE_ICTL_PON_LINE, cg->glb + CG_GLB_NE_ICTL_EN);
			/* ⚠ MASKED IS NOT STOPPED: an ISR already running on another
			 *   CPU re-enables the PON aggregate and queues isr_work on its
			 *   way out.  This waits for it. */
		synchronize_irq(cg->irq);
	}
	/* ALWAYS flush the bottom half, IRQ or not: the DS OMCI RX ...
	 * dev/MEASURED-cortina-gpon.c.md sec 133. */
	cancel_work_sync(&cg->isr_work);
}

/* ------------------------------------------------------------------ */
/* gpon0 — the WAN netdev over the GPON data path (Stage D)            */
/* ------------------------------------------------------------------ */

static int cg_wan_open(struct net_device *ndev)
{
	struct cortina_gpon *cg = cg_singleton;

	if (cg && cg->data_installed)
		netif_carrier_on(ndev);
	else
		netif_carrier_off(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int cg_wan_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static netdev_tx_t cg_wan_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct cortina_gpon *cg = cg_singleton;

	if (!cg || !cg->data_installed || !IS_REACHABLE(CONFIG_CORTINA_NI)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	return cortina_ni_pon_data_tx(skb, ndev);
}

static const struct net_device_ops cg_wan_ops = {
	.ndo_open		= cg_wan_open,
	.ndo_stop		= cg_wan_stop,
	.ndo_start_xmit		= cg_wan_xmit,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
	/* nf_flow_table HW offload: an nft flowtable with `flags offload` BINDs a
	 * flow block on EVERY hooked device, gpon0 included, so the WAN netdev must
	 * expose the same setup_tc entry as eth0.  Without it the flowtable offload
	 * setup fails (-EOPNOTSUPP).  A plain BIND writes no hardware. */
#if IS_REACHABLE(CONFIG_CORTINA_NI)
	.ndo_setup_tc		= cortina_ni_setup_tc,
#endif
};

/* gpon0's address comes off the common ladder (it used to be a compiled-in
 * 02:96:07:f0:00:02).  The +1 relation to eth0 is 05_factory_mac's and holds only
 * when that script reads ELAN_MAC_ADDR.  Carrier tracks the data-path install.
 */
static void cg_wan_create(struct cortina_gpon *cg)
{
	u8 dt[GPON_HWADDR_BYTES], mac[GPON_HWADDR_BYTES];
	enum gpon_hwaddr_src src;
	struct net_device *ndev;
	bool have_dt;

	have_dt = of_get_mac_address(cg->dev->of_node, dt) == 0;
	/* NULL bootarg/engine: no board declares one for gpon0, and the PON MAC
	 * registers hold a GPON serial, not a station address. */
	src = gpon_hwaddr_resolve(NULL, have_dt ? dt : NULL, NULL, mac);

	ndev = alloc_etherdev(0);
	if (!ndev)
		return;
	strscpy(ndev->name, "gpon0", sizeof(ndev->name));
	ndev->netdev_ops = &cg_wan_ops;
	eth_hw_addr_set(ndev, mac);
	SET_NETDEV_DEV(ndev, cg->dev);
	netif_carrier_off(ndev);
	if (register_netdev(ndev)) {
		dev_warn(cg->dev, "gpon0 register failed - no WAN netdev\n");
		free_netdev(ndev);
		return;
	}
	cg->wan_ndev = ndev;
	if (IS_REACHABLE(CONFIG_CORTINA_NI))
		cortina_ni_pon_wan_ndev_set(ndev);
	dev_info(cg->dev, "WAN netdev gpon0 registered (%pM from %s)\n", mac,
		 gpon_hwaddr_src_name(src));
}

/* Read the 4 ASCII bytes of the vendor-id register in wire order. */
static void cg_read_vendor(struct cortina_gpon *cg, char out[5])
{
	cg_vendor_unpack(cg_mac_rd(cg, CG_REG_VENDOR), out);
}

/* The GPON-MAC hardware error/statistics counters — the ...
 * dev/MEASURED-cortina-gpon.c.md sec 110. */
static void cg_show_gpon_mib(struct seq_file *m, struct cortina_gpon *cg)
{
	u32 bip = cg_mac_rd(cg, CG_REG_BIP_ERR);
	u32 accum = cg_mac_rd(cg, CG_REG_BIP_ERR_ACCUM);
	u32 frames = cg_mac_rd(cg, CG_REG_BIP_ERR_FRAMES);
	u32 fec_total = cg_mac_rd(cg, CG_REG_FEC_BLK_TOTAL);
	u32 v;
	bool live = !(bip == U32_MAX && accum == U32_MAX &&
		      frames == U32_MAX && fec_total == U32_MAX);

	seq_printf(m,
		   "gpon_ds_err    = %s bip=%u bip_accum=%u bip_frames=%u gem_frag_drop=%u gem_1bit=%u gem_2bit=%u gem_uncorr=%u omci_crc=%u ds_asmbl_drop=%u (accumulating, sw-cleared)\n",
		   live ? "live" : "UNAVAILABLE (block reads all-ones)",
		   bip, accum, frames,
		   cg_mac_rd(cg, CG_REG_GEM_FRAG_DROP),
		   cg_mac_rd(cg, CG_REG_GEM_1BITERR),
		   cg_mac_rd(cg, CG_REG_GEM_2BITERR),
		   cg_mac_rd(cg, CG_REG_GEM_UNCORR),
		   cg_mac_rd(cg, CG_REG_OMCI_CRC),
		   cg_mac_rd(cg, CG_REG_DS_ASMBL_DROP));
	seq_printf(m,
		   "gpon_ds_mib    = omci_gem=%u omci_pkt=%u ds_crc=%u undersize=%u oversize=%u superframe=%u (hardware DS counts)\n",
		   cg_mac_rd(cg, CG_REG_DS_OMCI_GEM),
		   cg_mac_rd(cg, CG_REG_DS_OMCI_PKT),
		   cg_mac_rd(cg, CG_REG_DS_PKT_CRC),
		   cg_mac_rd(cg, CG_REG_DS_UNDERSIZE),
		   cg_mac_rd(cg, CG_REG_DS_OVERSIZE),
		   cg_mac_rd(cg, CG_REG_SUPERFRAME));
	seq_printf(m,
		   "gpon_us_grant  = bwmap_drop=%u bwmap_corr=%u bwmap_uncorr=%u plend_err=%u plend_biterr=%u o5=%u us_omcc=%u (us_omcc UNVALIDATED)\n",
		   cg_mac_rd(cg, CG_REG_BWMAP_DROP),
		   cg_mac_rd(cg, CG_REG_BWMAP_CORR),
		   cg_mac_rd(cg, CG_REG_BWMAP_UNCORR),
		   cg_mac_rd(cg, CG_REG_PLEND_ERR),
		   cg_mac_rd(cg, CG_REG_PLEND_BITERR),
		   cg_mac_rd(cg, CG_REG_O5),
		   cg_mac_rd(cg, CG_REG_US_OMCC_CNT));
	/*
	 * The hardware's own upstream-wedge witness: there is NO other witness for
	 * "the GPON-MAC to PUC interface hung", and this one names the T-CONT.
	 */
	v = cg_mac_rd(cg, CG_REG_PUCIF_PROTECT);
	seq_printf(m,
		   "gpon_pucif_hang= %s (raw=0x%08x, tcont=%u) (latched, not cleared by this read)\n",
		   (v & BIT(0)) ? "★ HUNG" : "no", v, (v >> 1) & 0x1f);
	seq_printf(m,
		   "gpon_fec       = ctrl=0x%08x status=0x%08x total=%u clean=%u corr=%u uncorr=%u corr_bytes=%u (clear semantics UNPROVEN)\n",
		   cg_mac_rd(cg, CG_REG_FEC_CTRL),
		   cg_mac_rd(cg, CG_REG_FEC_MISC_STATUS),
		   fec_total,
		   cg_mac_rd(cg, CG_REG_FEC_CLEAN_BLK),
		   cg_mac_rd(cg, CG_REG_FEC_CORR_BLK),
		   cg_mac_rd(cg, CG_REG_FEC_UNCORR_BLK),
		   cg_mac_rd(cg, CG_REG_FEC_CORR_BYTES));
}

static int cg_proc_show(struct seq_file *m, void *v)
{
	struct omci_data_binding binding = {0};
	struct cortina_gpon *cg = m->private;
	char vendor[5], sn_str[13];
	u32 onu, alarm;

	mutex_lock(&cg->sn_lock);
	cg_read_vendor(cg, vendor);
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	alarm = cg_mac_rd(cg, CG_REG_ALARM);

	seq_printf(m, "gpon-mac @ phys 0x%llx + 0x%x\n",
		   (unsigned long long)CG_PON_WINDOW_PHYS, CG_GPON_MAC_OFF);
	seq_printf(m, "vendor-id      = 0x%08x (\"%s\")\n",
		   cg_mac_rd(cg, CG_REG_VENDOR), vendor);
	seq_printf(m, "vendor-spec    = 0x%08x\n", cg_mac_rd(cg, CG_REG_VENDOR_SPEC));
	/* The identity, and WHERE it came from: "board" is the only value meaning
	 * "read from this unit"; NONE = ranging is held off waiting for it.  ★ THE
	 * PARKED CASE SAYS WHY, and onu(state+id) below shows the MAC still at O1 as
	 * the independent second witness. */
	gpon_sn_format(cg->sn, sn_str);
	seq_printf(m, "serial-number  = %s\n",
		   cg->sn_src == CG_SN_NONE ? "(not provisioned)" : sn_str);
	seq_printf(m, "sn-source      = %s%s\n", cg_sn_src_name[cg->sn_src],
		   cg->activated ? "" :
		   " (ranging not started: no defined serial number)");
	seq_printf(m, "gpon_ds        = 0x%08x\n", cg_mac_rd(cg, CG_REG_GPON_DS));
	seq_printf(m, "onu(state+id)  = 0x%08x\n", onu);
	seq_printf(m, "main(eqd)      = 0x%08x\n", cg_mac_rd(cg, CG_REG_GPON_MAIN));
	seq_printf(m, "alarm          = 0x%08x%s\n", alarm,
		   alarm ? " (LOS/LOF!)" : " (no alarm, DS locked)");
	seq_printf(m, "onu_cfg        = 0x%08x\n", cg_mac_rd(cg, CG_REG_ONU_CFG_REAL));
	seq_printf(m, "us(frame_var)  = 0x%08x  t3_preamble = 0x%08x  gpon_ctrl = 0x%08x\n",
		   cg_mac_rd(cg, CG_REG_US), cg_mac_rd(cg, CG_REG_T3_PREAMBLE),
		   cg_mac_rd(cg, CG_REG_GPON_MAC_CTRL));
	/* post-O5 servicing (interrupts / FSM tracker / OMCC bind) */
	seq_puts(m, "-- post-O5 servicing --\n");
	seq_printf(m, "irq            = %d (count=%u, evt_drop=%u, omci_queue_drop=%u)\n",
		   cg->irq, cg->irq_count, cg->evt_drop, cg->omci_queue_drop);
	seq_printf(m, "fsm            = %s (live id 0x%02x), tracked %s\n",
		   cg_state_name[CG_ONU_STATE(onu)], CG_ONU_ID(onu),
		   cg_state_name[cg->last_state & 7]);
	seq_printf(m, "omcc           = %s (alloc=%u gem=%u)\n",
		   cg->omcc_up ? "UP" : "down", cg->omcc_alloc, cg->omcc_gem);
	seq_printf(m, "ds_omci_rx     = %u (short=%u)  pdc_ctrl = 0x%08x (%s, expect 0x02870002)\n",
		   cg->omci_rx, cg->omci_rx_short, readl(cg->pon + CG_PDC_CTRL),
		   cg->pdc_ready ? "programmed" : "NOT programmed");
	/* us_rx/enq/drop are the SHORT-WINDOW upstream-admission ...
	 * dev/MEASURED-cortina-gpon.c.md sec 111. */
	seq_printf(m, "puc            = %s  us_rx=%u enq=%u drop=%u  pucif=0x%08x\n",
		   cg->puc_ready ? "programmed" : "NOT programmed",
		   readl(cg->pon + CG_PUC_BMC_RX_PKT),
		   readl(cg->pon + CG_PUC_BMC_RX_PKT_ENQ),
		   readl(cg->pon + CG_PUC_BMC_FORCE_DROP) & CG_PUC_BMC_CNTR_MASK,
		   readl(cg->pon + CG_GPON_MAC_PUCIF_CTRL));
	/* and the OMCI-SPECIFIC upstream witness, CUMULATIVE: US ...
	 * dev/MEASURED-cortina-gpon.c.md sec 112. */
	cg_puc_ctrl_sample(cg);
	spin_lock(&cg->puc_cnt_lock);
	seq_printf(m,
		   "puc_ctrl       = us_omci=%u ctrl_mac=%u len_err=%u samples=%u lnk_type=0x%04x (cumulative)\n",
		   cg->puc_omci_us, cg->puc_ctrl_mac, cg->puc_len_err,
		   cg->puc_cnt_samples,
		   readl(cg->pon + CG_PUC_GLOBAL_LNK_TYPE) >> 16);
	spin_unlock(&cg->puc_cnt_lock);
	cg_show_gpon_mib(m, cg);
	seq_printf(m, "omci_resp      = %s tx=%u fail=%u ds_crc ok=%u bad=%u",
		   cg->omci_active ? "armed" : "off",
		   cg->omci_tx, cg->omci_tx_fail,
		   cg->omci_ds_crc_ok, cg->omci_ds_crc_bad);
	seq_printf(m, "omci_rx_bad_mic: %u (DS frames discarded on an invalid MIC)\n",
		   cg->omci_rx_bad_mic);
	/* ★ SEPARATE FROM bad_mic ON PURPOSE: a runt is a framing / GEM reassembly
	 * fault UPSTREAM of OMCI, a bad MIC is corruption on a well-framed PDU.  One
	 * number for both would make a broken reassembler read as a noisy fibre. */
	seq_printf(m, "omci_rx_runt:    %u (DS frames shorter than a 48-byte baseline PDU)\n",
		   cg->omci ? cg->omci->rx_runt : 0);
	if (cg->omci)
		seq_printf(m, "  mds=%u store=%u avc=%u unhandled=%u dup_replay=%u ext=%u no_ack=%u",
			   cg->omci->mds, cg->omci->store_n,
			   cg->omci->avc_count, cg->omci->unhandled,
			   cg->omci->dup_replay, cg->omci->rx_extended,
			   cg->omci->no_ack);
	seq_putc(m, '\n');
	spin_lock_bh(&cg->omci_lock);
	if (cg->omci_initialized)
		omci_data_binding_snapshot(cg->omci, cg->omcc_gem, CG_MCAST_GEM_ID, &binding);
	spin_unlock_bh(&cg->omci_lock);
	seq_printf(m, "data           = %s alloc=%u (me 0x%04x) gem=%u (tcont-ptr 0x%04x dir %u) bcast=%u carrier=%d\n",
		   cg->data_installed
			? (cg->data_rides_omcc ? "INSTALLED(rides-omcc)" : "INSTALLED")
			: "down",
		   binding.alloc_id, binding.tcont_inst, binding.gem_port, binding.tcont_inst,
		   binding.direction, CG_MCAST_GEM_ID,
		   cg->wan_ndev ? netif_carrier_ok(cg->wan_ndev) : -1);
	seq_printf(m, "omci_port      = 0x%08x (en=%d id=%u)\n",
		   cg_mac_rd(cg, CG_REG_OMCI_PORT),
		   !!(cg_mac_rd(cg, CG_REG_OMCI_PORT) & CG_OMCI_PORT_EN),
		   CG_OMCI_PORT_ID(cg_mac_rd(cg, CG_REG_OMCI_PORT)));
	seq_printf(m, "int_en/top_en  = 0x%08x / 0x%x  (int2/3/4_en = 0x%x/0x%x/0x%x)\n",
		   cg_mac_rd(cg, CG_REG_INT_EN), cg_mac_rd(cg, CG_REG_INT_TOP_EN),
		   cg_mac_rd(cg, CG_REG_INT2_EN), cg_mac_rd(cg, CG_REG_INT3_EN),
		   cg_mac_rd(cg, CG_REG_INT4_EN));
	if (cg->glb)
		seq_printf(m, "glb pon_int0   = 0x%08x en=0x%08x  ne_ictl sts=0x%08x en=0x%08x\n",
			   readl(cg->glb + CG_GLB_PON_INT0),
			   readl(cg->glb + CG_GLB_PON_INTEN0),
			   readl(cg->glb + CG_GLB_NE_ICTL_STS),
			   readl(cg->glb + CG_GLB_NE_ICTL_EN));

	/* serdes/gearbox/laser (PON-window raw offsets, for US-LOS diagnosis) */
	seq_puts(m, "-- serdes/gbox/laser --\n");
	seq_printf(m, "rgb8(a05c)     = 0x%08x  (DS-lock: (v&0x9c01)==0x9c00)\n", readl(cg->pon + CG_PSDS_RGB8));
	/* PSDS internal CMU reg 0x400 (indirect read strobe -> a090; the re-lock
	 * strobe target).  a08c shown too to disambiguate the read-data register. */
	writel(CG_PSDS_IND_READ | CG_PSDS_CMU_IDX, cg->pon + CG_PSDS_IND_CMD);
	udelay(10);
	seq_printf(m, "cmu[0x400]     = a090=0x%08x a08c=0x%08x  (re-lock strobes [7:4]; coldstart re-rolls=%u episode=%d)\n",
		   readl(cg->pon + CG_PSDS_IND_RDATA), readl(cg->pon + CG_PSDS_IND_WDATA),
		   cg->coldstart_rolls, cg->coldstart_tries);
	seq_printf(m, "gbox_ctrl(a060)= 0x%08x  (stock 0x454 rx/tx bit-order)\n", readl(cg->pon + CG_PSDS_GBOX_CTRL));
	seq_printf(m, "prbs_ctrl(a064)= 0x%08x  (stock 0)\n", readl(cg->pon + CG_PSDS_PRBS_CTRL));
	seq_printf(m, "prbs_intr(a068)= 0x%08x  (stock 1)\n", readl(cg->pon + CG_PSDS_PRBS_INTR));
	seq_printf(m, "prbs_sts(a070) = 0x%08x  (stock 1)\n", readl(cg->pon + CG_PSDS_PRBS_STS));
	seq_printf(m, "psds_init(glb) = 0x%08x  (ben_oen bit4, pow_pcix bit5)\n", readl(cg->glb + CG_GLB_PSDS_INIT));
	seq_printf(m, "laser_route    : glb(0x42c)=0x%08x mux0(0x130)=0x%08x gpio0 cfg(0x300)=0x%08x out(0x304)=0x%08x  (stock 0x01101101 / 0x00001fff / 0xffffe7bf / 0x00000040)\n",
		   readl(cg->glb + CG_GLB_PINROUTE), readl(cg->glb + CG_GLB_GPIO_MUX0),
		   readl(cg->gpio + CG_PERGPIO_CFG0), readl(cg->gpio + CG_PERGPIO_OUT0));
	seq_printf(m, "  gpio pin34   : mux(0x134)=0x%08x cfg(0x324)=0x%08x in(0x32c)=0x%08x  (stock 0x00000000 / 0xffffffff / in bit2=0 net-low=TX_DIS de-asserted)\n",
		   readl(cg->glb + CG_GLB_GPIO_MUX1), readl(cg->gpio + CG_PERGPIO_CFG1),
		   readl(cg->gpio + CG_PERGPIO_IN1));
	seq_printf(m, "  gpio grp3/4  : cfg3(0x36c)=0x%08x out3(0x370)=0x%08x cfg4(0x390)=0x%08x out4(0x394)=0x%08x  (stock 0xfffdef00/0x00021010/0xffffc5ff/0x00003200)\n",
		   readl(cg->gpio + CG_PERGPIO_CFG3), readl(cg->gpio + CG_PERGPIO_OUT3),
		   readl(cg->gpio + CG_PERGPIO_CFG4), readl(cg->gpio + CG_PERGPIO_OUT4));
	cg_bosa_proc_show(cg->dev, m);
	/* live optical diagnostics; also refreshes the ANI-G levels the OLT reads */
	cg_optic_sample(cg, m);

	/* full GPON MAC block dump (nonzero) for diffing against the stock golden.
	 * SKIP int_top (0xa4, READ-CLEARS: a cat of /proc must never eat a pending
	 * interrupt from under the ISR).  0x80-0x94 (DS MIB) and 0x1a8 (o5 count)
	 * are clear-on-read: dumped, but a read zeroes them. */
	seq_puts(m, "-- MAC block (nonzero; 0xa4 skipped; 0x80-0x94/0x1a8 clear-on-read) --\n");
	{
		u32 off, val;

		for (off = 0; off <= 0x1f4; off += 4) {
			if (off == CG_REG_INT_TOP)
				continue;
			val = cg_mac_rd(cg, off);
			if (val)
				seq_printf(m, "+0x%03x=0x%08x\n", off, val);
		}
	}
	mutex_unlock(&cg->sn_lock);
	return 0;
}

static int cg_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cg_proc_show, cg_singleton);
}

#if IS_ENABLED(CONFIG_GPON_OMCI_DIAG)
/* ★★★ THE UNI ADMINISTRATIVE-STATE INJECTION SEAM, the ...
 * dev/MEASURED-cortina-gpon.c.md sec 113. */
static u16 cg_uni_test_tci = 0xd200;

static void cg_uni_test_show(struct cortina_gpon *cg)
{
	u8 admin[OMCI_UNI_MAX] = { 0 }, port[OMCI_UNI_MAX] = { 0 };
	u16 inst[OMCI_UNI_MAX] = { 0 };
	u8 owed = 0, n = 0, port_n = 0, i;
	int active;

	/* ★★ THE LIFETIME OWNER IS TAKEN FIRST, and that is what ...
	 * dev/MEASURED-cortina-gpon.c.md sec 134. */
	mutex_lock(&cg->sn_lock);
	spin_lock_bh(&cg->omci_lock);
	active = (cg->omci_active && cg->omci) ? 1 : 0;
	if (cg->omci) {
		n = cg->omci->pptp_eth_uni.n;
		owed = cg->omci->pptp_eth_uni.changed;
		for (i = 0; i < n && i < OMCI_UNI_MAX; i++) {
			inst[i] = cg->omci->pptp_eth_uni.inst[i];
			admin[i] = cg->omci->pptp_eth_uni.admin[i];
		}
	}
	port_n = cg_uni_port_n;
	memcpy(port, cg_uni_port, sizeof(port));
	spin_unlock_bh(&cg->omci_lock);
	mutex_unlock(&cg->sn_lock);

	dev_info(cg->dev, "uni-test: SNAPSHOT omci_active %d slots %u owed 0x%x\n",
		 active, n, owed);
	for (i = 0; i < n && i < OMCI_UNI_MAX; i++) {
		int p = i < port_n ? (int)port[i] : -1;

		dev_info(cg->dev,
			 "uni-test: SNAPSHOT slot %u inst 0x%04x admin %u owed %u port %d desired_locked %d\n",
			 i, inst[i], admin[i], !!(owed & BIT(i)), p,
			 p < 0 ? -1 : (int)cortina_ni_uni_port_locked(p));
	}
}

static int cg_uni_test_set(struct cortina_gpon *cg, const char *arg)
{
	u32 rx0, drop0, mic0;
	unsigned int inst, admin;
	u8 msg[OMCI_LEN], n, i;
	int slot = -1, port;
	u16 tci;

	if (sscanf(arg, "%x %u", &inst, &admin) != 2 || inst > 0xffff || admin > 1)
		return -EINVAL;

	/* ★ THE INSTANCE IS CHECKED AGAINST THE BOARD'S DECLARED PANEL.  A typo
	 *   would otherwise inject a Set for a UNI this board does not have, and
	 *   the run would measure nothing while looking like it measured. */
	spin_lock_bh(&cg->omci_lock);
	if (cg->omci) {
		n = cg->omci->pptp_eth_uni.n;
		for (i = 0; i < n && i < OMCI_UNI_MAX; i++)
			if (cg->omci->pptp_eth_uni.inst[i] == (u16)inst)
				slot = i;
	}
	tci = ++cg_uni_test_tci;
	spin_unlock_bh(&cg->omci_lock);
	if (slot < 0) {
		dev_err(cg->dev,
			"uni-test: REFUSED: instance 0x%04x is not in this board's declared panel\n",
			inst);
		return -ENODEV;
	}
	port = slot < cg_uni_port_n ? (int)cg_uni_port[slot] : -1;

	memset(msg, 0, sizeof(msg));
	omci_put_be16(msg, tci);		/* a UNIQUE tid per injection */
	msg[2] = OMCI_MT_SET;			/* AR=0 and AK=0: no response */
	msg[3] = 0x0a;				/* device identifier: baseline */
	omci_put_be16(msg + 4, OMCI_ME_PPTP_ETH_UNI);
	omci_put_be16(msg + 6, (u16)inst);
	omci_put_be16(msg + 8, 0x0800);		/* attribute 5: administrative state */
	msg[10] = (u8)admin;
	omci_finalize(msg);			/* the SHIPPED stamper, so the
						 * MIC gate below sees a real one */

	/* ★ THE REQUEST IS ANNOUNCED BEFORE IT IS SUBMITTED: the bottom half can
	 *   consume it and complete on another CPU before a line printed after the
	 *   call reaches the log, and the reader would see completion before request. */
	dev_info(cg->dev,
		 "uni-test: REQUEST tid 0x%04x inst 0x%04x slot %d admin %u port %d\n",
		 tci, inst, slot, admin, port);
	rx0 = cg->omci_rx;
	drop0 = cg->omci_queue_drop;
	mic0 = cg->omci_rx_bad_mic;
	cg_rx_omci(msg, OMCI_LEN);
	/* ★★ THE THREE COUNTERS ARE THE ADMISSION EVIDENCE: the ring is closed
	 *    between datapath generations, and a frame dropped there looks exactly
	 *    like a port that declined to move. */
	dev_info(cg->dev,
		 "uni-test: ACCEPTED tid 0x%04x rx %u->%u drop %u->%u bad_mic %u->%u\n",
		 tci, rx0, cg->omci_rx, drop0, cg->omci_queue_drop,
		 mic0, cg->omci_rx_bad_mic);
	return 0;
}
#endif /* CONFIG_GPON_OMCI_DIAG */

/* ONE-SHOT on-demand PLOAM MIB read: `echo mib <sel-hex> > ...
 * dev/MEASURED-cortina-gpon.c.md sec 114. */
static ssize_t cg_proc_write(struct file *file, const char __user *ubuf,
			     size_t len, loff_t *ppos)
{
	struct cortina_gpon *cg = cg_singleton;
	char buf[32], *p;
	u32 sel, acc, data;
	int i;

	if (!cg || len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	p = strim(buf);
	/* `echo "sn VVVVHHHHHHHH" > /proc/gpon`: THE shipping ...
	 * dev/MEASURED-cortina-gpon.c.md sec 115. */
	if (strncmp(p, "sn ", 3) == 0) {
		int ret = cg_sn_set(cg, strim(p + 3), CG_SN_BOARD);

		return ret ? ret : len;
	}
	/* One-shot full BOSA register dump to dmesg (cold-state ...
	 * dev/MEASURED-cortina-gpon.c.md sec 116. */
	if (strcmp(p, "bosa dump") == 0) {
		int ret;

		mutex_lock(&cg->sn_lock);
		ret = cg_bosa_dump(cg->dev);
		mutex_unlock(&cg->sn_lock);
		return ret ? ret : len;
	}
#if IS_ENABLED(CONFIG_GPON_OMCI_DIAG)
	/* `uni-test show` / `uni-test <instance-hex> <0|1>`: the UNI
	 * administrative-state injection seam documented above its helpers. */
	if (strcmp(p, "uni-test show") == 0) {
		cg_uni_test_show(cg);
		return len;
	}
	if (strncmp(p, "uni-test ", 9) == 0) {
		int ret = cg_uni_test_set(cg, strim(p + 9));

		return ret ? ret : len;
	}
#endif
	/* manual SerDes CMU re-lock (the cold-start recovery primitive) -- for
	 * validating it is non-destructive on a good O5 boot before relying on it */
	if (strcmp(p, "relock") == 0) {
		mutex_lock(&cg->sn_lock);
		cg_psds_relock(cg);
		mutex_unlock(&cg->sn_lock);
		return len;
	}
	if (strncmp(p, "mib ", 4) != 0 || kstrtou32(strim(p + 4), 16, &sel))
		return -EINVAL;

	/* Only readable from a settled O5.  This strobes the TX-PLOAM MIB engine, and
	 * doing that during activation wedges the PLOAM FSM at O1 -- a diagnostic that
	 * bricks the link it is diagnosing is worse than none, and the wedge is
	 * indistinguishable from a real ranging failure.  Refuse instead. */
	mutex_lock(&cg->sn_lock);
	if (CG_ONU_STATE(cg_mac_rd(cg, CG_REG_GPON_ONU)) != CG_STATE_OPERATION) {
		mutex_unlock(&cg->sn_lock);
		return -EBUSY;
	}

	writel(CG_TBL_GO | (sel & 0x3ff), cg->mac + CG_REG_PLM_MIB_ACCESS);
	i = cg_go_poll(cg->mac + CG_REG_PLM_MIB_ACCESS, CG_MIB_TRIES, true);
	acc = readl(cg->mac + CG_REG_PLM_MIB_ACCESS);
	data = readl(cg->mac + CG_REG_PLM_MIB_DATA);
	dev_info(cg->dev,
		 "one-shot PLM MIB sel=0x%03x: access=0x%08x data=0x%08x (go %s after %d polls)\n",
		 sel, acc, data,
		 i < 0 ? "STUCK" : "cleared", i < 0 ? (int)CG_MIB_TRIES : i);
	mutex_unlock(&cg->sn_lock);
	return i < 0 ? -ETIMEDOUT : len;
}

static const struct proc_ops cg_proc_ops = {
	.proc_open	= cg_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= cg_proc_write,
};

static int cortina_gpon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cortina_gpon *cg;
	char vendor[5];
	u32 onu;

	cg = devm_kzalloc(dev, sizeof(*cg), GFP_KERNEL);
	if (!cg)
		return -ENOMEM;
	cg->dev = dev;
	cg->irq = -1;		/* until cg_intr_setup succeeds */

	/* Stage C: the G.988 responder context — allocated up front so the
	 * OMCC-up path (which can fire during the probe's ranging poll) only
	 * ever initializes it, never allocates.  ~7 KB. */
	spin_lock_init(&cg->omci_lock);
	mutex_init(&cg->sn_lock);
	spin_lock_init(&cg->puc_cnt_lock);
	/* The event ring and the bottom half are initialised HERE, ...
	 * dev/MEASURED-cortina-gpon.c.md sec 117. */
	spin_lock_init(&cg->evt_lock);
	INIT_WORK(&cg->isr_work, cg_isr_work);
	INIT_DELAYED_WORK(&cg->veip_avc_work, cg_veip_avc_work);
	INIT_DELAYED_WORK(&cg->uni_apply_work, cg_uni_apply_work);
	INIT_DELAYED_WORK(&cg->coldstart_work, cg_coldstart_work);
	INIT_DELAYED_WORK(&cg->sn_wait_work, cg_sn_wait_work);
	INIT_DELAYED_WORK(&cg->puc_cnt_work, cg_puc_cnt_work);
	cg->omci = devm_kzalloc(dev, sizeof(*cg->omci), GFP_KERNEL);
	if (!cg->omci)
		dev_warn(dev, "no OMCI responder ctx - DS OMCI will not be answered\n");

	/* Map the whole 48 KiB PON window (40-bit AXI address ...
	 * dev/MEASURED-cortina-gpon.c.md sec 118. */
	cg->pon = devm_ioremap(dev, CG_PON_WINDOW_PHYS, CG_PON_WINDOW_SIZE);
	if (!cg->pon) {
		dev_err(dev, "failed to map PON window 0x%llx\n",
			(unsigned long long)CG_PON_WINDOW_PHYS);
		return -ENOMEM;
	}
	cg->mac = cg->pon + CG_GPON_MAC_OFF;

	/* Map the GLB reset/clock window and dump the PON/GPON ...
	 * dev/MEASURED-cortina-gpon.c.md sec 119. */
	cg->glb = devm_ioremap(dev, CG_GLB_WINDOW_PHYS, CG_GLB_WINDOW_SIZE);
	cg->gpio = devm_ioremap(dev, CG_PERGPIO_PHYS, CG_PERGPIO_SIZE);
	if (cg->glb) {
		dev_info(dev, "GLB reset regs (ours): EPON_CNTL=0x%08x GPON_CNTL=0x%08x PON_CNTL=0x%08x PSDS_INIT=0x%08x\n",
			 readl(cg->glb + CG_GLB_EPON_CNTL),
			 readl(cg->glb + CG_GLB_GPON_CNTL),
			 readl(cg->glb + CG_GLB_PON_CNTL),
			 readl(cg->glb + CG_GLB_PSDS_INIT));
		dev_info(dev, "GLB reset regs (stock released): EPON_CNTL=0x00030000 GPON_CNTL=0x00000003 PON_CNTL=0x0000030e\n");

		if (cg_do_reset) {
			cg_glb_reset(cg);
			cg_psds_init(cg);
			dev_info(dev, "GLB after: EPON=0x%08x GPON=0x%08x PON=0x%08x PSDS_INIT=0x%08x\n",
				 readl(cg->glb + CG_GLB_EPON_CNTL),
				 readl(cg->glb + CG_GLB_GPON_CNTL),
				 readl(cg->glb + CG_GLB_PON_CNTL),
				 readl(cg->glb + CG_GLB_PSDS_INIT));
			dev_info(dev, "PSDS after: MODE=0x%08x RGB8=0x%08x (bit11 CKRDY_TX=%d)\n",
				 readl(cg->pon + CG_PSDS_MODE),
				 readl(cg->pon + CG_PSDS_RGB8),
				 !!(readl(cg->pon + CG_PSDS_RGB8) & BIT(11)));

			/* arm the post-O5 servicing BEFORE ranging starts so
			 * the ONU_ID/PORTID/ONU_ST_CHG events of the very
			 * first O1->O5 pass are serviced live */
			if (cg_do_intr)
				cg_intr_setup(cg, pdev);

			if (cg_activate) {
				/* The identity gate. Ranging announces the ONU's serial ...
				 * dev/MEASURED-cortina-gpon.c.md sec 120. */
				if (!cg_sn_param ||
				    cg_sn_set(cg, cg_sn_param, CG_SN_PARAM)) {
					dev_warn(dev, "GPON serial number not known yet - MAC configured, ranging DEFERRED up to %ds for /etc/init.d/gpon-identity (echo \"sn <VVVVHHHHHHHH>\" > /proc/gpon)\n",
						 CG_SN_WAIT_SECS);
					cg_sched(cg, &cg->sn_wait_work,
							      CG_SN_WAIT_SECS * HZ);
				}
			}
			if (cg->activated) {
				int i;

				/* poll the HW ranging FSM: onu.state, RGB8 (bit15 BER_NOTIFY ...
				 * dev/MEASURED-cortina-gpon.c.md sec 135. */
				for (i = 0; i < 30; i++) {
					mutex_lock(&cg->sn_lock);
					cg_frame_var_update(cg);
					mutex_unlock(&cg->sn_lock);
					dev_info(dev, "range t=%ds: onu=0x%08x rgb8=0x%08x superframe=0x%08x alarm=0x%08x us=0x%08x psds_init=0x%08x\n",
						 i, cg_mac_rd(cg, CG_REG_GPON_ONU),
						 readl(cg->pon + CG_PSDS_RGB8),
						 cg_mac_rd(cg, 0xfc),
						 cg_mac_rd(cg, CG_REG_ALARM),
						 cg_mac_rd(cg, CG_REG_US),
						 readl(cg->glb + CG_GLB_PSDS_INIT));
					msleep(200);
				}
			}
		}
	} else {
		dev_warn(dev, "failed to map GLB window 0x%llx\n",
			 (unsigned long long)CG_GLB_WINDOW_PHYS);
	}

	cg_read_vendor(cg, vendor);
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	/* Not a correctness check: before the identity is provisioned this reads
	 * the reset value.  cg_activate_start() verifies the vendor-id readback
	 * against what it programmed, which works on any board. */
	dev_info(dev, "GPON MAC vendor-id \"%s\" onu=0x%08x alarm=0x%08x\n",
		 vendor, onu, cg_mac_rd(cg, CG_REG_ALARM));

	cg_singleton = cg;
	/* Stage B: receive the DS OMCI PDUs the NI CPU-RX path classifies out
	 * (ethertype 0xfff1).  Registered after cg_singleton so the handler
	 * never sees a half-initialized context. */
	if (IS_REACHABLE(CONFIG_CORTINA_NI))
		cortina_ni_pon_rx_hook_set(cg_rx_omci);
	cg_wan_create(cg);	/* Stage D: the gpon0 WAN netdev */
	cg->proc = proc_create_data("gpon", 0644, NULL, &cg_proc_ops, cg);
#ifdef CONFIG_GPON_OLT_DIAG
	cg->oltcap_proc = proc_create_single("oltcap", 0444, NULL, cg_oltcap_show);
#endif
	platform_set_drvdata(pdev, cg);
	dev_info(dev, "cortina-gpon phase-0 probe complete (/proc/gpon)\n");
	return 0;
}

static void cortina_gpon_remove(struct platform_device *pdev)
{
	struct cortina_gpon *cg = platform_get_drvdata(pdev);

	/* ★★★ ADMISSIONS CLOSE FIRST, IN ORDER, AND ONLY THEN IS ...
	 * dev/MEASURED-cortina-gpon.c.md sec 121. */
	mutex_lock(&cg->sn_lock);
	WRITE_ONCE(cg->stopping, true);
	mutex_unlock(&cg->sn_lock);

	/*  2. /proc, which is a producer in its own right through sn_set */
	if (cg->proc) {
		proc_remove(cg->proc);
		cg->proc = NULL;
	}
#ifdef CONFIG_GPON_OLT_DIAG
	if (cg->oltcap_proc) {
		proc_remove(cg->oltcap_proc);
		cg->oltcap_proc = NULL;
	}
#endif

	/*  3. the DS hook, and it WAITS for a NAPI callback already inside it --
	 *     a bare store published NULL and returned while a reader still held
	 *     the old function pointer */
	if (IS_REACHABLE(CONFIG_CORTINA_NI)) {
		cortina_ni_pon_wan_ndev_set(NULL);
		cortina_ni_pon_rx_hook_set(NULL);
	}

	/*  4. the IRQ itself: masked and synchronised, so no new ISR can queue */
	cg_intr_teardown(cg);

	/*  5. only now, the workers */
	cancel_delayed_work_sync(&cg->veip_avc_work);
	cancel_delayed_work_sync(&cg->coldstart_work);
	cancel_delayed_work_sync(&cg->sn_wait_work);
	cancel_delayed_work_sync(&cg->puc_cnt_work);
	if (cg->wan_ndev) {
		unregister_netdev(cg->wan_ndev);
		free_netdev(cg->wan_ndev);
	}
	/* ★★ THE CONSUMER IS CANCELLED LAST, AFTER EVERY PRODUCER IS STOPPED.  It
	 *    used to run first, and an isr_work ALREADY RUNNING re-queues it from
	 *    cg_omci_process, so the cancel raced a producer it had not stopped.
	 *    Nothing here holds sn_lock, which the work itself takes. */
	cancel_delayed_work_sync(&cg->uni_apply_work);
	if (cg_singleton == cg)
		cg_singleton = NULL;
}

static const struct of_device_id cortina_gpon_of_match[] = {
	{ .compatible = "realtek,rtl9607f-gpon" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_gpon_of_match);

static struct platform_driver cortina_gpon_driver = {
	.probe	= cortina_gpon_probe,
	.remove	= cortina_gpon_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= cortina_gpon_of_match,
	},
};
module_platform_driver(cortina_gpon_driver);

MODULE_DESCRIPTION("Cortina-Access GPON MAC driver for Realtek RTL9607F Elnath");
MODULE_LICENSE("GPL");
