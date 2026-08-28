/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * RTL9602C GPON/SWCORE register offsets -- the PER-SoC half of the driver.
 *
 * ★★★ WHY THIS FILE EXISTS (operator, 2026-08-27: *"la majoria de la logica
 * devria estar en la familia, y solo workaround/offset especifica al SoC"*).
 * The driver it was extracted from is 8028 lines, and a measurement of its own
 * shape says the split is almost already there:
 *
 *     105 functions, of which only 6 carry a chip conditional
 *     180 register/offset #defines  <- THIS FILE
 *      70 module_param knobs        <- workarounds, some genuinely per-chip
 *
 * 99 of 105 functions never ask which chip they are on. What tied them to one
 * SoC was not their logic, it was these constants -- so moving the constants
 * out is what lets the logic be family code.
 *
 * ★ COMPILE-TIME, NOT A RUNTIME TABLE, and that is deliberate. This project
 * ships ONE LEAN KERNEL PER MODEL (a GPON ONU has 16 MB of flash and <=64 MB
 * of RAM), so a second chip is a second header selected by Kconfig -- not a
 * pointer indirection on every register access in the datapath hot path.
 * `rtl960x_ponmac.c` uses a runtime table because it must serve four chips
 * from one object; this header is the other, cheaper case.
 *
 * ★ ADDING A CHIP = COPYING THIS FILE AND EDITING THE VALUES. Nothing else.
 * That is the whole point of the split: the next Luna part costs a table, not
 * a fork of the logic.
 *
 * ⚠ THIS IS PURE CODE MOTION. It was verified by building before and after and
 * comparing the object file byte for byte -- see the commit message. No value
 * was changed, no define was renamed, and the file order is preserved so the
 * one define whose value references another still resolves.
 */
#ifndef _RTL9602C_GPON_REGS_H
#define _RTL9602C_GPON_REGS_H


#define GPON_PHYS_BASE	0x1b700000u
#define GPON_REG_SIZE	0x00010000u	/* covers GTC DS block at +0x1000 */

/*
 * SoC system-controller "IP enable" bank. Bit 5 powers the PON packet datapath
 * ("PONPBO" / PON-IP at phys 0x1bf00000); the neighbouring bits in this bank
 * gate other on-chip IPs (e.g. PCIe). U-Boot never sets the PON bit because it
 * does not run GPON, so the PON-IP block is unclocked and any access to it
 * hard-hangs the CPU bus until this bit is set.
 */
#define SOC_IP_ENABLE_PHYS	0x1800063cu
#define   SOC_IP_EN_PON		BIT(5)
#define   SOC_IP_EN_PONPBO	BIT(25)	/* rev>A PON packet-datapath (9607C) */

#define GPON_INT_DLT		0x0000
#define GPON_RESET		0x000c
#define   GPON_SOFT_RST		BIT(0)		/* 1 = assert soft reset      */
#define   GPON_RST_DONE		BIT(8)		/* 1 = reset cycle complete   */
#define GPON_VERSION		0x0010
#define   GPON_VER_ID_MASK	0xffu
#define GPON_TEST		0x0014
#define GPON_AES_BYPASS		0x0020
#define GPON_INTR_MASK		0x0040
#define GPON_INTR_STS		0x0044
#define GPON_GTC_DS_INTR_DLT	0x1000
#define GPON_GTC_DS_INTR_MASK	0x1004
#define GPON_GTC_DS_INTR_STS	0x1008
#define GPON_GTC_DS_LOS_CFG_STS	0x1040		/* downstream LOS status/cfg  */
#define   GPON_CDR_LOS_SIG	BIT(10)		/* 1 = CDR not recovering clk */
#define   GPON_OPTIC_LOS_SIG	BIT(8)		/* 1 = no optical signal      */
#define   GPON_OPTIC_LOS_POLAR	BIT(1)		/* invert optical-LOS input   */
#define   GPON_OPTIC_LOS_EN	BIT(0)		/* enable optical-LOS monitor */
#define GPON_GTC_DS_ONU_STATUS	0x1010
#define   GPON_ONU_STATE_MASK	0xfu		/* [3:0]  FSM state O1..O7    */
#define   GPON_ONU_ID_SHIFT	8		/* [15:8] ONU-ID             */
#define   GPON_ONU_ID_MASK	0xffu
#define GPON_GTC_US_ONU_ID	0x5010
#define GPON_GTC_US_MIN_DELAY	0x5040
#define GPON_GTC_US_EQD		0x5044
#define   GPON_EQD_INFRAME_MASK	0x3ffffu	/* [17:0]  in-frame delay    */
#define   GPON_EQD_MF_SHIFT	24		/* [26:24] multiframe count  */
#define   GPON_EQD_MF_MASK	0x7u
#define   GPON_EQD_FRAME_LEN	(19440 * 8)	/* one upstream frame, in bits */
#define GPON_GTC_US_WRITE_PROTECT 0x5018	/* gate for US config writes   */
#define   GPON_US_WP_UNLOCK	0xcc19u		/* magic: enable protected US writes */
#define   GPON_US_WP_LOCK	0x0000u
#define GPON_GTC_US_CFG		0x5014		/* [11]LESS_RANDOM [10]IND_NRM_PLM
						 * [9]PLM_DIS [4]ENA_AUTO_DG
						 * [3]US_BEN_POLAR [0]SCRM_DIS */
#define   GPON_US_CFG_VAL	0x0c18u		/* online operating value: BEN_POLAR=1,
						 * scrambler on, PLOAM on (LESS_RANDOM|
						 * IND_NRM_PLM|ENA_AUTO_DG|US_BEN_POLAR) */
#define GPON_GTC_US_LASER	0x504c		/* [13:8] LON_TIME, [5:0] LOFF_TIME */
#define   GPON_US_LASER_VAL	0x2028u		/* LON=32, LOFF=40 burst-window edges */
#define GPON_GTC_US_BOH_CFG	0x5054		/* [11:8] BOH_REPEAT, [7:0] BOH_LENGTH */
#define GPON_GTC_US_BOH_DATA	0x5080		/* 12-entry burst-overhead byte array, stride 4 */

#define GPON_GTC_DS_PLOAM_CFG	0x101c		/* [9]BC_ACC [8]ONUID_FLT [7:0]NOMSG_ID */
#define GPON_GTC_DS_PLOAM_IND	0x1080		/* DS receive-buffer indicator */
#define   GPON_DS_PLM_BUF_EMPTY	BIT(5)		/* 1 = no DS PLOAM pending     */
#define   GPON_DS_PLM_BUF_FULL	BIT(4)
#define   GPON_DS_PLM_DEQ	BIT(0)		/* W: advance to next message  */
#define GPON_GTC_DS_PLOAM_MSG	0x10a0		/* 8-word received-message buf */
#define GPON_GTC_US_PLOAM_IND	0x50c0		/* US transmit-queue indicator */
#define   GPON_US_PLM_TYPE_SHIFT 8		/* [10:8] queue/type select    */
#define   GPON_US_PLM_NRM_EMPTY	BIT(7)		/* normal queue empty          */
#define   GPON_US_PLM_NRM_FULL	BIT(6)
#define   GPON_US_PLM_URG_EMPTY	BIT(5)		/* urgent queue empty          */
#define   GPON_US_PLM_URG_FULL	BIT(4)
#define   GPON_US_PLM_ENQ	BIT(0)		/* W: queue the composed msg   */
#define GPON_GTC_US_PLOAM_DATA	0x50e0		/* 8-word transmit-message buf */
#define GPON_GTC_US_PLOAM_CFG	0x5100		/* US PLOAM buffer control     */
#define   GPON_US_PLM_FLUSH_BUF	BIT(4)
#define   GPON_US_PLM_CRC_GEN_EN BIT(1)		/* HW computes US PLOAM CRC    */
#define   GPON_US_PLM_ONUID_OVRD BIT(0)		/* override ONU-ID field       */

#define GPON_TEST_SCRATCH	0x12345678u
#define GPON_RST_POLL_MAX	1000		/* bounded RST_DONE poll      */

/*
 * PON SerDes (SDS) analog block. It lives in the SWCORE window (phys
 * 0x1B000000), NOT the GPON datapath sub-block — these are plain MMIO offsets
 * off the switch-core base. The SDS CMU/PLL recovers the line clock that feeds
 * the GPON MAC core; until it is configured and its analog-ready flag asserts,
 * the MAC core has no clock, the soft-reset never completes (RST_DONE stays 0)
 * and the GTC register banks read a floating pattern. The GPON-MAC reset is
 * issued as part of this sequence (the SDS_RST also resets the MAC).
 */
#define SWCORE_PHYS_BASE	0x1b000000u
#define SWCORE_REG_SIZE		0x00041000u	/* covers up to FIB_EXT_REG21  */

/*
 * Register offsets here are the TRUE switch-core offsets (verified against the
 * SoC register map). The SerDes digital/analog banks live at SWCORE + 0x22xxx
 * (phys 0x1b022xxx), NOT 0x40xxx — a direct access to 0x40xxx hits an unmapped
 * hole that returns the bus abort-fill 0xbad0bad0. SDS_CFG and SOFTWARE_RST are
 * in the low control page.
 */
#define SW_SOFTWARE_RST		0x00104
#define   SW_SDS_RST_PS		BIT(0)		/* [0]  CMD_SDS_RST_PS pulse   */
#define   SW_SDS_CFG_RST_PS	BIT(7)		/* [7]  CMD_SDS_CFG_RST_PS     */
#define   SW_PONMAC_RST		BIT(6)		/* [6]  GPON-MAC core reset    */
#define SDS_CFG			0x001d0		/* [4:0] CFG_SDS_MODE          */
#define   SDS_MODE_OFF		0x1fu
#define   SDS_MODE_GPON		0x08u
#define SDS_FIB_STATUS		0x001e4		/* [17] SDS_SDET [2] FIB100_SDET */
#define   SDS_FIB_SDS_SDET	BIT(17)		/* SDS-level optical sig-detect */
#define FIB_REG16		0x22c40		/* [10] FP_CFG_FRC_SD [2] SEL_RX_SD */

#define I2C_CONFIG0		0x23004		/* bus0; stride 0x20 per bus   */
#define   I2C_CFG_DEV_ID_MSB	20		/* [20:14] 7-bit slave addr    */
#define   I2C_CFG_DEV_ID_LSB	14
#define   I2C_CFG_AW_MSB	13		/* [13:12] reg-addr width 0=8b */
#define   I2C_CFG_AW_LSB	12
#define   I2C_CFG_DW_MSB	11		/* [11:10] data width 0=8b     */
#define   I2C_CFG_DW_LSB	10
#define   I2C_CFG_CLKDIV_MSB	9		/* [9:0] clock divider         */
#define   I2C_CFG_CLKDIV_LSB	0
#define   I2C_CLKDIV_100K	0x270u		/* (62500/100)-1 -> ~100 kHz   */
#define I2C_IND_WD		0x000b0		/* [31:0] write data           */
#define I2C_IND_ADR		0x000b8		/* [31:0] target reg offset    */
#define I2C_IND_CMD		0x000c0		/* [0]CMD_EN [1]RW_EN [2]BUSY [3]NACK */
#define   I2C_CMD_EN		BIT(0)
#define   I2C_CMD_RW_WR		BIT(1)		/* 1=write 0=read              */
#define   I2C_CMD_BUSY		BIT(2)
#define   I2C_CMD_NACK		BIT(3)
#define I2C_IND_RD		0x000c8		/* [31:0] read data            */
#define I2C_BUSY_POLL_MAX	1000		/* x10us = up to 10 ms         */
/*
 * RTL8290B register space is paged by I2C slave address: the full 12-bit
 * register number's high byte selects a 256-register page (page0->0x50,
 * page1->0x51, page2->0x54, page3+ ->0x55) and the low byte is the offset
 * within that page. Confirmed: chip-ID reg 0x390 reads correctly at slave 0x55
 * offset 0x90. RX path registers (NUM/page mapping from the transceiver's
 * register map):
 */
#define BOSA_REG_NUM		0x390		/* chip NUM (0x8290), 2 bytes  */
#define BOSA_REG_VID		0x394		/* manufacturer ID (0x0001)    */
#define BOSA_REG_W4		0x204		/* [4] EN_L booster (1=on)     */
#define BOSA_REG_W41		0x229		/* [4] RXI_PWDN_L (0=RX on)    */
#define BOSA_REG_CONTROL2	0x254		/* [6] LOS_PIN_TRI (0=drive SD)*/
#define BOSA_REG_STATUS2	0x383		/* [2] RX_LOS_STATUS (0=signal)*/
#define WSDS_DIG_00		0x22030		/* SDS clock + soft-reset-B bank */
#define   WSDS_STOP_CLK		BIT(0)		/* 1 = GPON MAC core clock off */
#define   WSDS_FRC_125M_EN	BIT(4)		/* force 125M ref clock enable */
#define   WSDS_FRCV_125M_EN	BIT(5)		/* forced value for 125M       */
#define   WSDS_SFT_RSTB		BIT(8)		/* digital soft reset-B        */
#define   WSDS_SFT_RSTB_EPON	BIT(9)		/* EPON datapath reset-B       */
#define   WSDS_SFT_RSTB_GPON	BIT(10)		/* GPON datapath reset-B       */
#define   WSDS_SFT_RSB_ANA	BIT(11)		/* analog reset-B              */
#define   WSDS_DIG00_RUN	0xf30u		/* operational run state       */
#define WSDS_DIG_01		0x22034		/* [31:0] CFG_DMY0 (force-SDS)  */
#define WSDS_DIG_02		0x22038		/* [10]  EN_PDOWN_BEN          */
#define WSDS_DIG_03		0x2203c		/* [6:4] CFG_TXDIS_SEL_DLY     */
#define WSDS_DIG_18		0x22090		/* [12]  BEN_OE                */
#define WSDS_DIG_1D		0x220a4		/* interface reset-B releases  */
#define   WSDS_SFT_RSTB_INF	BIT(14)		/* interface soft reset-B (FIFO r/w ptr re-sync) */
#define   WSDS_SFT_RSTB_INF_RX	BIT(15)		/* RX interface soft reset-B   */
#define   WSDS_SFT_RSTB_INF_TX	BIT(16)		/* TX interface soft reset-B   */
#define SDS_ANA_COM_REG27	0x225ec		/* TX CMU enable lives here    */
#define   SDS_CMU_EN		BIT(10)		/* TX CMU enable (toggle 1->0->1 to re-lock the PLL) */
#define SDS_ANA_COM_REG03	0x2258c		/* [15:14] CMU_ISTANK_SEL_RX   */
#define SDS_ANA_COM_REG08	0x225a0		/* TX-CDR (reg1418); [15] = the
						 * serdesCdr_reset toggle bit    */
#define SDS_ANA_COM_REG11	0x225ac		/* [7:0]  RX_FILT_CONFIG       */
#define SDS_ANA_COM_REG12	0x225b0		/* [14]   RX_SEL_CDR_AFEN      */
#define SDS_ANA_COM_REG22	0x225d8		/* [5:3] TX_AMP [2:0] TX_EMP   */
#define SDS_ANA_COM_REG26	0x225e8		/* [6:5] CMU_ISTANK_SEL_GPHY   */
#define SDS_ANA_GPON_REG42	0x22728		/* [2]   PCM_CMU_EN            */
#define SDS_ANA_GPON_REG46	0x22738		/* [9:7]KI [6:4]KP1 [3:1]KP2   */
#define SDS_ANA_MISC_REG00	0x22500		/* [5] FRC_RX_EN_VAL [4] _ON   */
#define SDS_ANA_MISC_REG01	0x22504		/* [7:5] SPDSEL_VAL [4] _ON    */
#define SDS_ANA_MISC_REG02	0x22508		/* [13] SD_VAL [12] SD_FORCE   */
#define FIB_EXT_REG21		0x22e54		/* [13]  FEP_V2ANALOG (lock)   */
#define   SDS_ANALOG_READY	BIT(13)
#define SDS_LOCK_POLL_MAX	1000		/* x200us = up to 200 ms       */
#define SDS_REG0		0x22800		/* [1] SP_SDS_EN_RX (RX enable)*/
#define   SP_SDS_EN_RX		BIT(1)
/* Stock link-state polling behavior: when GPON_GTC_DS_INTR_STS reads this
 * exact sentinel the DS CDR is wedged; stock recovers by toggling SP_SDS_EN_RX
 * 1->0->1 (SDS_REG0[1]), waiting 10ms, then re-reading. */
#define GTC_DS_CDR_STUCK	0xca0eca0fu

/*
 * SoC IO pad routing for the optical front-end (switch-core register file, so
 * these are plain SWCORE offsets like the SDS block above). IO_MODE_EN's OEM_EN
 * bit enables the optical "e-mode" pads (TX_DISABLE, optical TX_SD / RX signal-
 * detect); IO_GPIO_EN is a 1-bit-per-pin GPIO-function-enable array (32 pins per
 * 32-bit word). The optical RX signal-detect shares pad GPIO 13: while that pin
 * is in GPIO mode the BOSA's signal-detect never reaches the GPON LOS input, so
 * OPTIC_LOS_SIG reads "loss" even with real light. Releasing GPIO 13 (function
 * disabled) routes the pad to the optical-SD input.
 */
#define SOC_IO_MODE_EN		0x23018		/* [19] OEM_EN (optical pads)  */
#define   IO_OEM_EN		BIT(19)
#define SOC_IO_GPIO_EN		0x00048		/* GPIO func-enable, 1 bit/pin */
#define SOC_IO_GPIO_EN_W0	0x40202006u	/* enable GPIO 1,2,13,21,30    */
#define SOC_IO_GPIO_EN_W1	0x00000819u	/* enable GPIO 32,35,36,43     */

/*
 * Front-panel LED controller (SWCORE window). Each panel LED has an index whose
 * 2-bit "force value" the CPU can drive directly — 0=off, 1=on, 2=blink — once
 * the index is placed in CPU force-mode and enabled for parallel (vs serial-
 * shift) output. A working unit lights the green PON LED solid once ranged to
 * the OLT and lights the red LOS LED only while downstream light is absent.
 * Board X111W wires PON-status to index 12 and LOS to index 13. Offsets are
 * into swcore_base (phys 0x1b000000).
 */
#define LED_MODE_SEL		0x1e000		/* [0] 0 = parallel output      */
#define LED_DATA_CFG(idx)	(0x1e004 + (idx) * 4)	/* [12] CPU force-mode  */
#define   LED_CPU_FORCE_BIT	12
#define LED_FORCE_VALUE		0x1e04c		/* [idx*2+1:idx*2] force value  */
#define LED_BLINK_RATE		0x1e050		/* [14:12] force blink period   */
#define LED_PARA_EN		0x1e05c		/* [n+1] LEDn parallel-enable    */
#define   LED_SERI_DATA_EN_BIT	19
#define   LED_SERI_CLK_EN_BIT	18
#define LED_IO_EN		0x23014		/* [n] LEDn pad-output enable    */
#define   LED_SERI_OUT_EN_BIT	17
#define LED_FORCE_OFF		0u
#define LED_FORCE_ON		1u
#define LED_FORCE_BLINK		2u
#define LED_BLINK_512MS		4u		/* [14:12] period code           */
#define PON_LED_IDX		12u
#define LOS_LED_IDX		13u
/* Ethernet port-link LEDs: hardware-auto (the switch lights them straight from
 * port link + activity, no CPU). 0xf78 = link at every speed (bits 8..11) +
 * activity at every speed (bits 3..6). The switch port map is port0=FE(100M),
 * port1=GE(1G), so each LED is typed to its own port. */
#define LED_LINKACT		0xf78u
#define LED_TYPE_UTP0		0x01u		/* switch port 0 = FE 100M */
#define LED_TYPE_UTP1		0x02u		/* switch port 1 = GE 1G   */

#define GPIO_PHYS_BASE		0x18003300u
#define GPIO_REG_SIZE		0x40u
#define GPIO_CTRL_ABCD		0x00
#define GPIO_DIR_ABCD		0x08
#define GPIO_DATA_ABCD		0x0c
#define GPIO_CTRL_EFGH		0x1c
#define GPIO_DIR_EFGH		0x24
#define GPIO_DATA_EFGH		0x28
#define GPIO_GOLD_DIR_ABCD	0x40002006u	/* 1,2,13,30 out; 21 in        */
#define GPIO_GOLD_DATA_ABCD	0xdbff1246u
#define GPIO_GOLD_DIR_EFGH	0x00000819u
#define GPIO_GOLD_DATA_EFGH	0x000037e5u

#define PONIP_PHYS_BASE		0x1bf00000u
#define PONIP_REG_SIZE		0x00010000u	/* covers up to IO_CMD_1_DS     */

#define PI_IO_CMD_0_US		0x05434		/* [5] GMII_RX_EN [4] GMII_TX_EN */
#define PI_IO_CMD_0_DS		0x0d434
/* Internal-MII force-link for the two PON-IP NICs (symmetric pair, US = DS-0x8000):
 * MEDIA_STS_DS @0xc058 (read 0x106e8400 from the bootloader, the DS-NIC<->GMAC0 RX
 * link that DS OMCI rides) and MEDIA_STS_US @0x4058 (the GMAC0-TX -> US-NIC link
 * that US OMCI must ride). FORCELINK[18] FORCEDFULLDUP[19] FORCE_SPD[17:16]
 * FORCE_SPD_MODE[10] TRXFCE/RXFCE/TXFCE[31:29]; golden 0x106e8400. No DAL code
 * writes the US one -> it stays down -> GMAC0-TX OMCI is dropped at the US-NIC GMII
 * ingress before PON_SID2QID (ustx 0x329bc = 0). Force it up like the DS side. */
#define PI_MEDIA_STS_US		0x04058
#define PI_MEDIA_STS_DS		0x0c058
#define PI_IO_CMD_1_US		0x05438		/* [5:4] RPAGE [1:0] TPAGE size  */
#define PI_IO_CMD_1_DS		0x0d438
#define   PI_GMII_RX_EN		BIT(5)
#define   PI_GMII_TX_EN		BIT(4)
#define PI_PONIP_CTL_US		0x020d8		/* US PON-IP control             */
#define PI_PONIP_CTL_DS		0x0a0ac		/* DS PON-IP control             */
#define   PI_CFG_PBUF_EN	BIT(0)		/* [0] packet-buffer enable      */
#define   PI_CFG_STOP_RXC_EN	BIT(1)		/* [1] stop RXC enable           */
#define   PI_CFG_EPON_MODE	BIT(2)		/* [2] 0=GPON 1=EPON             */
#define PI_PON_US_FIFO_CTL	0x020f0		/* [5:4] SPACE [3:0] START       */
#define PI_PON_DSC_CFG_US	0x0215c		/* [28:16] RAM_NO [12:0] SRAM_NO */
#define PI_PON_DSC_CFG_DS	0x0a0cc		/* [14:13] PAGE_SIZE             */
#define PI_DSCRUNOUT_US		0x020e0		/* [28:16] DRAM [12:0] SRAM out  */
#define PI_IP_MSTBASE_US	0x020e8		/* CFG_PON_MSTBASE: phys base of the US PBO DRAM packet pool */
/* US PBO DRAM pool. ROOT CAUSE of "US-NIC RX never receives" (2026-06-12, found
 * via the stock memory-usage init behavior + live devmem): the US-NIC RX path
 * is a PBO that DMAs received US packets into a DRAM pool at IP_MSTBASE_US. Stock
 * sets IP_MSTBASE_US=0x07eff000, RAM_NO=0x1fff, DRAM_RUNOUT=0x1f58 (a 1MB / 8192x
 * 128B-page DRAM pool); DS is SRAM-only (IP_MSTBASE_DS=0). Our driver did US
 * SRAM-only too (copying DS) -> the US-NIC RX had no DRAM pool to land frames in,
 * so EVERY US OMCI frame was dropped at descriptor-fetch BEFORE the MAC, leaving
 * PKT_OK/ERR/MISS all 0 (the exact symptom). Allocate the pool and point the HW
 * at it. Stock memory-usage init math (usPageSize=128, us_mem_size=1MB): SRAM_NO=0x7f,
 * RAM_NO=min(size/128,0x1fff)=0x1fff, DRAM_RUNOUT=size/128-40-128=0x1f58. */
#define PI_US_DRAM_PAGES	0x1fffu		/* RAM_NO: total US PBO pages-1 (SRAM+DRAM) */
#define PI_US_DRAM_RUNOUT	0x1f58u		/* DSCRUNOUT_US DRAM portion (size/128-168) */
#define PI_US_DRAM_ORDER	8		/* __get_free_pages order: 2^8 * 4KB = 1MB */
#define PI_DSCRUNOUT_DS		0x0a0b4
#define PI_PON_SID_STOP_TH	0x02450		/* [12:0] global stop-all page threshold */
#define PI_PON_SID_GLB_TH	0x02454		/* [28:16] ON_TH [12:0] OFF_TH (global) */
#define PI_PON_SID_RPV_TH	0x02458		/* per-SID reserved-page threshold, +sid*4 */
#define PI_RPV_TH_STRIDE	4u
#define PI_SID_NUM		65u		/* SIDs 0..64 (64 = OMCI) */
#define PI_PON_FC_CONFIG_DS	0x0a0fc		/* [28:16] FC_ON_TH [12:0] FC_OFF_TH */
#define PI_CFG_US		0x0404c		/* [26] RFF_AFULL [17] TX_STOP   */
#define PI_CFG_DS		0x0c04c		/* [16] TXE_EXTRA                */
#define PI_TX_CFG_US		0x04040		/* [12:10] IFG [2:1] PRE [0] PAD */
#define PI_TX_CFG_DS		0x0c040
#define PI_RX_CFG_US		0x04044		/* [5] accept-CRC-error          */
#define PI_RX_CFG_DS		0x0c044
#define PI_PROBE_SELECT_US	0x05400		/* [1] debug func select         */
#define PI_PROBE_SELECT_DS	0x0d400		/* stock O5 = 0x40 (DS-NIC drain) */
#define PI_DS_NIC_CFG_D404	0x0d404		/* stock O5 = 0x11100348          */
#define PI_DS_NIC_CFG_D42C	0x0d42c		/* stock O5 = 0x40               */

#define GPON_GTC_DS_MISC_CNTR_LOM 0x1198	/* [31:16]=PLEND_FAIL [15:0]=SUPERFRAME_LOS(LOM); clear-on-read */

#define SMI_CTRL_0		0x230B8u
#define   SMI_CMD_EN		BIT(0)
#define SMI_CTRL_2		0x230C0u
#define SMI_CTRL_3		0x230C4u
#define SMI_BC_PHYID		0x230C8u
#define SWCORE_PROXY_PHY	10
#define SW_IO_MODE_EN_9607C	0x23014u	/* 9607C: I2C_EN[13] + MDX_M_EN[10] */

#define TBL_CTRL_OFF	0x12000u
#define TBL_STS_OFF	0x12004u	/* BUSY = bit13 */
#define TBL_WRDATA_OFF	0x12008u

#define PLM_DS_UPSTREAM_OVERHEAD	0x01
#define PLM_DS_ASSIGN_ONU_ID		0x03
#define PLM_DS_RANGING_TIME		0x04
#define PLM_DS_DEACTIVATE_ONU		0x05
#define PLM_DS_DISABLE_SN		0x06
#define PLM_DS_EXT_BURST_LENGTH		0x14
#define PLM_DS_ENCRYPT_PORT		0x08	/* Encrypted_Port-ID (ACK) */
#define PLM_DS_REQUEST_KEY		0x0d	/* Request_key (-> Encryption_Key) */
#define PLM_DS_KEY_SWITCH		0x13	/* Key_Switching_Time (-> arm HW key switch) */
#define PLM_DS_ASSIGN_ALLOC_ID		0x0a	/* Assign_Alloc-ID (ACK) */
#define PLM_DS_REQUEST_PASSWORD		0x09	/* Request_Password (-> US Password 0x02) */
#define PLM_DS_CONFIG_PORT		0x0e	/* Configure_Port-ID (ACK) */
#define PLM_DS_CFG_VPVC			0x07	/* Configure_VP/VC (ATM, unsupported; ACK) */
#define PLM_DS_BER_INTERVAL		0x12	/* BER interval (ACK; arms BER reporting) */
#define PLM_US_ENCRYPT_KEY		0x05	/* US Encryption_Key response */
#define PLM_US_PASSWORD			0x02	/* US Password response (to Request_Password) */
#define PLM_US_SERIAL_NUMBER		0x01
#define PLM_US_ACKNOWLEDGE		0x09	/* US Acknowledge message type */
#define PLM_US_QUEUE_SN			0x6	/* US_PLOAM_IND[10:8] auto-SN queue */
#define PLM_US_QUEUE_URG		0x1	/* US_PLOAM_IND[10:8] urgent queue (ACKs) */
#define PLM_US_QUEUE_NOMSG		0x7	/* US_PLOAM_IND[10:8] HW auto-No_message slot */

#define GPON_GTC_DS_PORT_IND	0x1100		/* CAM op: OP_MODE[9:8] OP_IDX[6:0] */
#define   DS_PORT_OP_REQ	BIT(15)
#define   DS_PORT_OP_COMPL	BIT(14)
#define   DS_PORT_OP_WRITE	BIT(8)		/* OP_MODE = WRITE(1) */
#define GPON_GTC_DS_PORT_WR	0x1104		/* [11:0] gemPortId */
#define GPON_GTC_DS_TRAFFIC_CFG	0x1400		/* array: base 0x1400, STRIDE 4 bytes, idx
						 * 0..127, [4:0] traffic-type. The register map's
						 * "array offset"=32 is in BITS (32b = 4 bytes), NOT
						 * bytes — confirmed on a live online stock ONU whose
						 * tcfg[64] sits at 0x1500 (=0x1400+64*4)=0x4, with
						 * 0x1c00 empty. (A 0x20 stride wrote outside the
						 * 0x1400..0x1600 array, into the void.) */
#define   DS_TRAFFIC_CFG_STRIDE	4u
#define   DS_TRAFFIC_IS_OMCI	BIT(2)
#define GPON_GTC_GEM_US_PORT_MAP 0x6400		/* array: base 0x6400, stride 4 (one 32-bit
						 * word/entry), idx 0..127, [11:0] gemPortId.
						 * Flow 64 (OMCC) = 0x6400 + 64*4 = 0x6500. */

#define GPON_GTC_DS_OMCI_PTI	0x1204		/* [6:4] PTI_MASK [2:0] END_PTI */
#define   DS_OMCI_PTI_VAL	((1u << 4) | 1u)	/* mask=1 ptn=1 -> 0x11 */
#define GPON_GEM_DS_MC_CFG	0x4080		/* [6] BROADCAST_PASS [4] NON_MULTICAST_PASS [3] FCS_CHK_EN */
#define   GEM_DS_MC_CFG_VAL	0x59u		/* stock O5 operating value (read live from an online stock ONU): BROADCAST_PASS(6)|NON_MULTICAST_PASS(4)|FCS_CHK_EN(3)|bit0. The earlier 0x18 (no broadcast/bit0) was a wrong "avoid US stall" guess — stock runs 0x59 stably online with OMCI flowing. */
#define PI_PON_SID2QID		0x020f8		/* packed 7b/SID: physical queue */
#define PI_PON_SIDVALID		0x0213c		/* packed 1b/SID */
#define PI_PON_OMCI_CFG		0x02154		/* [6:0] OMCI SID */
#define PI_PON_SID_Q_MAP_DS	0x0a0e4		/* packed 2b/SID: DS PBO queue */

#define GPON_GTC_DS_ALLOC_IND	0x10c0		/* T-CONT alloc CAM: OP_IDX[4:0]=tcont */
#define GPON_GTC_DS_ALLOC_WR	0x10c4		/* [11:0] allocateId */

#endif /* _RTL9602C_GPON_REGS_H */
