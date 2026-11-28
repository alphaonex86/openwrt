// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CHIP */
/*
 * The BCM56150 (Hurricane2) switch core of an iProc host, brought up at probe through its CMIC: stock's own switch
 * init, in stock's order and with stock's values, one function per stage. Register and memory names and addresses
 * are those of the register map in the stock firmware.
 */

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "bcm_cmic.h"

/* S-Channel block numbers */
enum {
	BLK_GPORT0 = 2,
	BLK_GPORT1 = 3,
	BLK_GPORT2 = 4,
	BLK_XLPORT0 = 5,
	BLK_XLPORT1 = 6,
	BLK_IPIPE = 10,
	BLK_EPIPE = 11,
	BLK_MMU = 12,
	BLK_OTPC = 13,
	BLK_CMIC = 14,
	BLK_IPROC = 15,
	BLK_TOP = 16,
	BLK_SER = 19,
};

/* CMIC registers: byte offsets in the CMIC window */
#define CMIC_BS0_CLK_CTRL                                0x00010704
#define CMIC_BS0_CONFIG                                  0x00010700
#define CMIC_BS0_HEARTBEAT_CTRL                          0x00010708
#define CMIC_BS0_OUTPUT_TIME_0                           0x00010714
#define CMIC_BS1_CLK_CTRL                                0x00010804
#define CMIC_BS1_CONFIG                                  0x00010800
#define CMIC_BS1_HEARTBEAT_CTRL                          0x00010808
#define CMIC_BS1_OUTPUT_TIME_0                           0x00010814
#define CMIC_CPS_RESET                                   0x00010220
#define CMIC_LEDUP0_CTRL                                 0x00020000
#define CMIC_LEDUP0_DATA_RAM                             0x00020400
#define CMIC_LEDUP0_PORT_ORDER_REMAP_0_3                 0x00020010
#define CMIC_LEDUP0_PORT_ORDER_REMAP_12_15               0x0002001c
#define CMIC_LEDUP0_PORT_ORDER_REMAP_16_19               0x00020020
#define CMIC_LEDUP0_PORT_ORDER_REMAP_20_23               0x00020024
#define CMIC_LEDUP0_PORT_ORDER_REMAP_24_27               0x00020028
#define CMIC_LEDUP0_PORT_ORDER_REMAP_28_31               0x0002002c
#define CMIC_LEDUP0_PORT_ORDER_REMAP_4_7                 0x00020014
#define CMIC_LEDUP0_PORT_ORDER_REMAP_8_11                0x00020018
#define CMIC_MIIM_BUS_SEL_MAP_19_10                      0x00011070
#define CMIC_MIIM_BUS_SEL_MAP_29_20                      0x00011074
#define CMIC_MIIM_BUS_SEL_MAP_39_30                      0x00011078
#define CMIC_MIIM_BUS_SEL_MAP_9_0                        0x0001106c
#define CMIC_MIIM_EXT_PHY_ADDR_MAP_11_8                  0x0001109c
#define CMIC_MIIM_EXT_PHY_ADDR_MAP_15_12                 0x000110a0
#define CMIC_MIIM_EXT_PHY_ADDR_MAP_19_16                 0x000110a4
#define CMIC_MIIM_EXT_PHY_ADDR_MAP_23_20                 0x000110a8
#define CMIC_MIIM_EXT_PHY_ADDR_MAP_27_24                 0x000110ac
#define CMIC_MIIM_EXT_PHY_ADDR_MAP_31_28                 0x000110b0
#define CMIC_MIIM_EXT_PHY_ADDR_MAP_3_0                   0x00011094
#define CMIC_MIIM_EXT_PHY_ADDR_MAP_7_4                   0x00011098
#define CMIC_MIIM_INT_SEL_MAP_0                          0x00011060
#define CMIC_MIIM_SCAN_CTRL                              0x00011008
#define CMIC_PCIE_USERIF_PURGE_CONTROL                   0x00010260
#define CMIC_RATE_ADJUST                                 0x00011000
#define CMIC_RATE_ADJUST_INT_MDIO                        0x00011004
#define CMIC_RXBUF_EPINTF_RELEASE_ALL_CREDITS            0x0001a000
#define CMIC_SBUS_RING_MAP_0_7                           0x00010098
#define CMIC_SBUS_RING_MAP_16_23                         0x000100a0
#define CMIC_SBUS_RING_MAP_24_31                         0x000100a4
#define CMIC_SBUS_RING_MAP_8_15                          0x0001009c
#define CMIC_SBUS_TIMEOUT                                0x00010094
#define CMIC_TIMESYNC_INTERRUPT_ENABLE                   0x00010914
#define CMIC_TIMESYNC_TIME_CAPTURE_CONTROL               0x00010908
#define CMIC_TIMESYNC_TIME_CAPTURE_MODE                  0x00010624
#define CMIC_UNNAMED_148                                 0x00000148	/* not in stock's register map */

/* Switch registers: S-Channel address of port 0, index 0; a port adds 1, an array index 0x100 */
#define AUX_ARB_CONTROL_2                                0x02000100
#define BCAST_BLOCK_MASK_64                              0x44000400
#define CNGCOSPKTLIMIT0                                  0x00006800
#define CNGCOSPKTLIMIT1                                  0x00008800
#define CNGDROPCOUNT0                                    0x00015400
#define CNGDROPCOUNT1                                    0x00015500
#define COMMAND_CONFIG                                   0x00010200
#define CPU_CONTROL_1                                    0x3a000300
#define DOS_CONTROL                                      0x06000100
#define DOS_CONTROL_2                                    0x06000200
#define DYNCELLLIMIT                                     0x00012d00
#define DYNRESETLIMPORT                                  0x0000cb00
#define DYNXQCNTPORT                                     0x0000ca00
#define EFP_METER_CONTROL                                0x2e002c00
#define EGRMETERINGBUCKET                                0x00015700
#define EGRMETERINGCONFIG                                0x00015600
#define EGR_CONFIG_1                                     0x06000500
#define EGR_EDATABUF_PARITY_CONTROL                      0x36011200
#define EGR_HW_RESET_CONTROL_0                           0x02000000
#define EGR_HW_RESET_CONTROL_1                           0x02000100
#define EGR_INITBUF_ECC_CONTROL                          0x22010000
#define EGR_INTR_ENABLE                                  0x36010300
#define EGR_IPMC_CFG2                                    0x0c000e00
#define EGR_L3_INTF_PARITY_CONTROL                       0x0e010000
#define EGR_L3_NEXT_HOP_PARITY_CONTROL                   0x0e010200
#define EGR_LOGICAL_TO_PHYSICAL_PORT_NUMBER_MAPPING      0x34001b00
#define EGR_MASK_PARITY_CONTROL                          0x46010d00
#define EGR_MC_CONTROL_1                                 0x26002500
#define EGR_MC_CONTROL_2                                 0x26002600
#define EGR_MTU_SIZE                                     0x44001b00
#define EGR_PORT                                         0x04000900
#define EGR_RSPAN_VLAN_TAG                               0x24002400
#define EGR_VLAN_CONTROL_1                               0x04000b00
#define EGR_VLAN_PARITY_CONTROL                          0x12010000
#define EGR_VLAN_STG_PARITY_CONTROL                      0x12010200
#define EGR_VLAN_XLATE_PARITY_CONTROL                    0x16000000
#define EPC_LINK_BMAP_64                                 0x46002700
#define FLUSH_CONTROL                                    0x0001cd00
#define FRM_LENGTH                                       0x00010500
#define GE0_EEE_CONFIG                                   0x02022400
#define GE1_EEE_CONFIG                                   0x02022500
#define GE2_EEE_CONFIG                                   0x02022600
#define GE3_EEE_CONFIG                                   0x02022700
#define GE4_EEE_CONFIG                                   0x02022800
#define GE5_EEE_CONFIG                                   0x02022900
#define GE6_EEE_CONFIG                                   0x02022a00
#define GE7_EEE_CONFIG                                   0x02022b00
#define GPORT_CNTMAXSIZE                                 0x02020800
#define GPORT_CONFIG                                     0x02020000
#define GPORT_RSV_MASK                                   0x02020600
#define GPORT_STAT_UPDATE_MASK                           0x02020700
#define GR1023                                           0x00001500
#define GR127                                            0x00001200
#define GR1518                                           0x00001600
#define GR2047                                           0x00001800
#define GR255                                            0x00001300
#define GR4095                                           0x00001900
#define GR511                                            0x00001400
#define GR64                                             0x00001100
#define GR9216                                           0x00001a00
#define GRALN                                            0x00000400
#define GRBCA                                            0x00001000
#define GRBYT                                            0x00001c00
#define GRCDE                                            0x00000600
#define GRFCR                                            0x00000700
#define GRFCS                                            0x00000000
#define GRFLR                                            0x00000500
#define GRFRG                                            0x00000d00
#define GRJBR                                            0x00000900
#define GRMCA                                            0x00000f00
#define GRMGV                                            0x00001700
#define GRMTUE                                           0x00000a00
#define GROVR                                            0x00000800
#define GRPKT                                            0x00001b00
#define GRPOK                                            0x00001e00
#define GRRBYT                                           0x00000e00
#define GRRPKT                                           0x00000b00
#define GRUC                                             0x00001d00
#define GRUND                                            0x00000c00
#define GRXCF                                            0x00000100
#define GRXPF                                            0x00000200
#define GRXUO                                            0x00000300
#define GRX_EEE_LPI_DURATION_COUNTER                     0x00003d00
#define GRX_EEE_LPI_EVENT_COUNTER                        0x00003c00
#define GT1023                                           0x00003200
#define GT127                                            0x00002f00
#define GT1518                                           0x00003300
#define GT2047                                           0x00003500
#define GT255                                            0x00003000
#define GT4095                                           0x00003600
#define GT511                                            0x00003100
#define GT64                                             0x00002e00
#define GT9216                                           0x00003700
#define GTBCA                                            0x00002d00
#define GTBYT                                            0x00003900
#define GTDFR                                            0x00002400
#define GTEDF                                            0x00002500
#define GTFCS                                            0x00002100
#define GTFRG                                            0x00002a00
#define GTJBR                                            0x00002000
#define GTLCL                                            0x00002800
#define GTMCA                                            0x00002c00
#define GTMCL                                            0x00002700
#define GTMGV                                            0x00003400
#define GTNCL                                            0x00002b00
#define GTOVR                                            0x00002300
#define GTPKT                                            0x00003800
#define GTPOK                                            0x00003b00
#define GTSCL                                            0x00002600
#define GTUC                                             0x00003a00
#define GTXCF                                            0x00002200
#define GTXCL                                            0x00002900
#define GTXPF                                            0x00001f00
#define GTX_EEE_LPI_DURATION_COUNTER                     0x00003f00
#define GTX_EEE_LPI_EVENT_COUNTER                        0x00003e00
#define HG_TRUNK_BITMAP                                  0x46006900
#define HG_TRUNK_GROUP                                   0x46006b00
#define HIGIG_TRUNK_CONTROL                              0x46002600
#define HOLCOSCELLMAXLIMIT                               0x0000ed00
#define HOLCOSMINXQCNT                                   0x00004800
#define HOLCOSPKTRESETLIMIT                              0x00002800
#define HOLCOSPKTSETLIMIT                                0x00000800
#define HOL_DROP                                         0x44005100
#define IARB_OAM_REFRESH_CONTROL                         0x02010600
#define IARB_PKT_ECC_CONTROL                             0x02010300
#define IARB_TDM_CONTROL                                 0x02000900
#define IBCAST                                           0x44004d00
#define IBPCELLSETLIMIT                                  0x00000200
#define IBPDISCARDSETLIMIT                               0x00000300
#define IBPPKTSETLIMIT                                   0x00000000
#define ICTRL                                            0x44004c00
#define IEGR_PORT                                        0x04000a00
#define IFP_COUNTER_PARITY_CONTROL                       0x3e011100
#define IFP_METER_PARITY_CONTROL                         0x3e010f00
#define IFP_POLICY_PARITY_CONTROL                        0x3e010d00
#define IFP_STORM_CONTROL_PARITY_CONTROL                 0x3e011300
#define IGMP_MLD_PKT_CONTROL                             0x1c000500
#define IIPMC                                            0x44004f00
#define ILTOMC                                           0x44004e00
#define IMIRROR_BITMAP                                   0x44001800
#define IMRP4                                            0x44004600
#define IMRP6                                            0x44004a00
#define ING_CONFIG_64                                    0x06000000
#define ING_EGRMSKBMAP_64                                0x44000e00
#define ING_EN_EFILTER_BITMAP_64                         0x3a000800
#define ING_HW_RESET_CONTROL_1                           0x02000200
#define ING_HW_RESET_CONTROL_2                           0x02000300
#define ING_L3_NEXT_HOP_PARITY_CONTROL                   0x42010100
#define INITIAL_NHOP_PARITY_CONTROL                      0x36010200
#define IP0_INTR_ENABLE                                  0x12010100
#define IP1_INTR_ENABLE                                  0x2a010100
#define IP2_INTR_ENABLE                                  0x46010100
#define IP_TO_CMICM_CREDIT_TRANSFER                      0x02002300
#define IUNHGI                                           0x44004b00
#define IUNKOPC                                          0x44005000
#define KNOWN_MCAST_BLOCK_MASK_64                        0x44001200
#define L2MC_PARITY_CONTROL                              0x26010100
#define L2_AGE_TIMER                                     0x02000400
#define L2_AUX_HASH_CONTROL                              0x1e000100
#define L2_ENTRY_PARITY_CONTROL                          0x1e010c00
#define L2_USER_ENTRY_CAM_DBGCTRL                        0x1e010000
#define L3MC_PARITY_CONTROL                              0x2e010100
#define L3_AUX_HASH_CONTROL                              0x2a000000
#define L3_DEFIP_PARITY_CONTROL                          0x32010000
#define L3_ENTRY_PARITY_CONTROL                          0x2a010d00
#define L3_MTU_VALUES_PARITY_CONTROL                     0x46010200
#define LMEP_COMMON                                      0x02000e00
#define LMEP_PARITY_CONTROL                              0x02010100
#define LOG_TO_PHY_PORT_MAPPING                          0x00086000
#define LWMCOSCELLSETLIMIT                               0x0000cd00
#define MAID_REDUCTION_PARITY_CONTROL                    0x36010600
#define MAXBUCKET                                        0x0001bc00
#define MAXBUCKETCONFIG                                  0x00019c00
#define MA_INDEX_PARITY_CONTROL                          0x36010400
#define MA_STATE_PARITY_CONTROL                          0x36010a00
#define MC_CONTROL_1                                     0x06000500
#define MC_CONTROL_2                                     0x06000600
#define MC_CONTROL_3                                     0x06000700
#define MC_CONTROL_4                                     0x06000800
#define MEMFAILINTMASK                                   0x02028000
#define MINBUCKET                                        0x00017c00
#define MINBUCKETCONFIG                                  0x00015c00
#define MISCCONFIG                                       0x02025a00
#define MMUFLUSHCONTROL                                  0x02030c00
#define MMUPORTENABLE                                    0x02025e00
#define MODPORT_MAP_EM_PARITY_CONTROL                    0x46011300
#define MODPORT_MAP_IM_PARITY_CONTROL                    0x46011100
#define MODPORT_MAP_SW_PARITY_CONTROL                    0x46010f00
#define OAM_LM_COUNTERS_PARITY_CONTROL                   0x46011500
#define PAUSE_CONTROL                                    0x0001cc00
#define PAUSE_QUANT                                      0x00010600
#define PKTAGINGLIMIT                                    0x02025900
#define PKTAGINGTIMER                                    0x02025800
#define PROTOCOL_PKT_CONTROL                             0x1c000400
#define R1023                                            0x00000400
#define R127                                             0x00000100
#define R1518                                            0x00000500
#define R16383                                           0x00000a00
#define R2047                                            0x00000700
#define R255                                             0x00000200
#define R4095                                            0x00000800
#define R511                                             0x00000300
#define R64                                              0x00000000
#define R9216                                            0x00000900
#define RALN                                             0x00001600
#define RBCA                                             0x00000e00
#define RBYT                                             0x00003d00
#define RDBGC0                                           0x44003a00
#define RDBGC0_SELECT                                    0x46005200
#define RDBGC1                                           0x44003b00
#define RDBGC1_SELECT                                    0x46005300
#define RDBGC2                                           0x44003c00
#define RDBGC2_SELECT                                    0x46005400
#define RDBGC3                                           0x44003d00
#define RDBGC4                                           0x44003e00
#define RDBGC5                                           0x44003f00
#define RDBGC6                                           0x44004000
#define RDBGC7                                           0x44004100
#define RDBGC8                                           0x44004200
#define RDISC                                            0x44003700
#define RDVLN                                            0x00002000
#define RERPKT                                           0x00001800
#define RFCR                                             0x00001900
#define RFCS                                             0x00000f00
#define RFLR                                             0x00001700
#define RFRG                                             0x00003500
#define RIPC4                                            0x44004400
#define RIPC6                                            0x44004800
#define RIPD4                                            0x44004300
#define RIPD6                                            0x44004700
#define RIPHE4                                           0x44004500
#define RIPHE6                                           0x44004900
#define RJBR                                             0x00001b00
#define RMCA                                             0x00000d00
#define RMCRC                                            0x00001d00
#define RMEP_PARITY_CONTROL                              0x36010800
#define RMGV                                             0x00000600
#define RMTUE                                            0x00001c00
#define ROVR                                             0x00001a00
#define RPFC0                                            0x00002b00
#define RPFC1                                            0x00002c00
#define RPFC2                                            0x00002d00
#define RPFC3                                            0x00002e00
#define RPFC4                                            0x00002f00
#define RPFC5                                            0x00003000
#define RPFC6                                            0x00003100
#define RPFC7                                            0x00003200
#define RPFCOFF0                                         0x00002300
#define RPFCOFF1                                         0x00002400
#define RPFCOFF2                                         0x00002500
#define RPFCOFF3                                         0x00002600
#define RPFCOFF4                                         0x00002700
#define RPFCOFF5                                         0x00002800
#define RPFCOFF6                                         0x00002900
#define RPFCOFF7                                         0x00002a00
#define RPKT                                             0x00000b00
#define RPOK                                             0x00002200
#define RPORTD                                           0x44003900
#define RPRM                                             0x00001e00
#define RRBYT                                            0x00003e00
#define RRPKT                                            0x00003f00
#define RSCHCRC                                          0x00003300
#define RTRFU                                            0x00002100
#define RUC                                              0x44003800
#define RUCA                                             0x00000c00
#define RUND                                             0x00003400
#define RVLN                                             0x00001f00
#define RXCF                                             0x00001000
#define RXPF                                             0x00001100
#define RXPP                                             0x00001200
#define RXUDA                                            0x00001400
#define RXUO                                             0x00001300
#define RXWSA                                            0x00001500
#define RX_EEE_LPI_DURATION_COUNTER                      0x00003700
#define RX_EEE_LPI_EVENT_COUNTER                         0x00003600
#define RX_HCFC_COUNTER                                  0x00003b00
#define RX_HCFC_CRC_COUNTER                              0x00003c00
#define RX_LLFC_CRC_COUNTER                              0x00003a00
#define RX_LLFC_LOG_COUNTER                              0x00003900
#define RX_LLFC_PHY_COUNTER                              0x00003800
#define SER_CONFIG_REG                                   0x02000000
#define SER_RANGE_0_ADDR_MASK                            0x02000300
#define SER_RANGE_0_CONFIG                               0x02002300
#define SER_RANGE_0_END                                  0x02006300
#define SER_RANGE_0_PROT_WORD_0                          0x0200a300
#define SER_RANGE_0_PROT_WORD_1                          0x0200a400
#define SER_RANGE_0_PROT_WORD_2                          0x0200a500
#define SER_RANGE_0_PROT_WORD_3                          0x0200a600
#define SER_RANGE_0_RESULT                               0x02008300
#define SER_RANGE_0_START                                0x02004300
#define SER_RANGE_1_ADDR_MASK                            0x02000400
#define SER_RANGE_1_CONFIG                               0x02002400
#define SER_RANGE_1_END                                  0x02006400
#define SER_RANGE_1_PROT_WORD_0                          0x0200a700
#define SER_RANGE_1_PROT_WORD_1                          0x0200a800
#define SER_RANGE_1_PROT_WORD_2                          0x0200a900
#define SER_RANGE_1_PROT_WORD_3                          0x0200aa00
#define SER_RANGE_1_RESULT                               0x02008400
#define SER_RANGE_1_START                                0x02004400
#define SER_RANGE_2_ADDR_MASK                            0x02000500
#define SER_RANGE_2_CONFIG                               0x02002500
#define SER_RANGE_2_END                                  0x02006500
#define SER_RANGE_2_PROT_WORD_0                          0x0200ab00
#define SER_RANGE_2_PROT_WORD_1                          0x0200ac00
#define SER_RANGE_2_PROT_WORD_2                          0x0200ad00
#define SER_RANGE_2_PROT_WORD_3                          0x0200ae00
#define SER_RANGE_2_RESULT                               0x02008500
#define SER_RANGE_2_START                                0x02004500
#define SER_RANGE_3_ADDR_MASK                            0x02000600
#define SER_RANGE_3_CONFIG                               0x02002600
#define SER_RANGE_3_END                                  0x02006600
#define SER_RANGE_3_PROT_WORD_0                          0x0200af00
#define SER_RANGE_3_PROT_WORD_1                          0x0200b000
#define SER_RANGE_3_PROT_WORD_2                          0x0200b100
#define SER_RANGE_3_PROT_WORD_3                          0x0200b200
#define SER_RANGE_3_RESULT                               0x02008600
#define SER_RANGE_3_START                                0x02004600
#define SER_RANGE_4_ADDR_MASK                            0x02000700
#define SER_RANGE_4_CONFIG                               0x02002700
#define SER_RANGE_4_END                                  0x02006700
#define SER_RANGE_4_PROT_WORD_0                          0x0200b300
#define SER_RANGE_4_PROT_WORD_1                          0x0200b400
#define SER_RANGE_4_PROT_WORD_2                          0x0200b500
#define SER_RANGE_4_PROT_WORD_3                          0x0200b600
#define SER_RANGE_4_RESULT                               0x02008700
#define SER_RANGE_4_START                                0x02004700
#define SER_RANGE_5_ADDR_MASK                            0x02000800
#define SER_RANGE_5_CONFIG                               0x02002800
#define SER_RANGE_5_END                                  0x02006800
#define SER_RANGE_5_PROT_WORD_0                          0x0200b700
#define SER_RANGE_5_PROT_WORD_1                          0x0200b800
#define SER_RANGE_5_PROT_WORD_2                          0x0200b900
#define SER_RANGE_5_PROT_WORD_3                          0x0200ba00
#define SER_RANGE_5_RESULT                               0x02008800
#define SER_RANGE_5_START                                0x02004800
#define SER_RANGE_6_ADDR_MASK                            0x02000900
#define SER_RANGE_6_CONFIG                               0x02002900
#define SER_RANGE_6_END                                  0x02006900
#define SER_RANGE_6_PROT_WORD_0                          0x0200bb00
#define SER_RANGE_6_PROT_WORD_1                          0x0200bc00
#define SER_RANGE_6_PROT_WORD_2                          0x0200bd00
#define SER_RANGE_6_PROT_WORD_3                          0x0200be00
#define SER_RANGE_6_RESULT                               0x02008900
#define SER_RANGE_6_START                                0x02004900
#define SER_RANGE_7_ADDR_MASK                            0x02000a00
#define SER_RANGE_7_CONFIG                               0x02002a00
#define SER_RANGE_7_END                                  0x02006a00
#define SER_RANGE_7_PROT_WORD_0                          0x0200bf00
#define SER_RANGE_7_PROT_WORD_1                          0x0200c000
#define SER_RANGE_7_PROT_WORD_2                          0x0200c100
#define SER_RANGE_7_PROT_WORD_3                          0x0200c200
#define SER_RANGE_7_RESULT                               0x02008a00
#define SER_RANGE_7_START                                0x02004a00
#define SER_RANGE_8_ADDR_MASK                            0x02000b00
#define SER_RANGE_8_CONFIG                               0x02002b00
#define SER_RANGE_8_END                                  0x02006b00
#define SER_RANGE_8_PROT_WORD_0                          0x0200c300
#define SER_RANGE_8_PROT_WORD_1                          0x0200c400
#define SER_RANGE_8_PROT_WORD_2                          0x0200c500
#define SER_RANGE_8_PROT_WORD_3                          0x0200c600
#define SER_RANGE_8_RESULT                               0x02008b00
#define SER_RANGE_8_START                                0x02004b00
#define SER_RANGE_ENABLE                                 0x02000200
#define SER_RING_ERR_CTRL                                0x02000100
#define SOURCE_TRUNK_MAP_PARITY_CONTROL                  0x0a010500
#define STORM_CONTROL_METER_CONFIG                       0x38000200
#define STORM_CONTROL_METER_MAPPING                      0x3a000100
#define SW2_FP_DST_ACTION_CONTROL                        0x46003400
#define SW2_RAM_CONTROL_4                                0x46010700
#define SYS_MAC_LIMIT_CONTROL                            0x1e000300
#define T1023                                            0x00004400
#define T127                                             0x00004100
#define T1518                                            0x00004500
#define T16383                                           0x00004a00
#define T2047                                            0x00004700
#define T255                                             0x00004200
#define T4095                                            0x00004800
#define T511                                             0x00004300
#define T64                                              0x00004000
#define T9216                                            0x00004900
#define TBCA                                             0x00004f00
#define TBYT                                             0x00006f00
#define TDBGC0                                           0x34000000
#define TDBGC0_SELECT                                    0x36000d00
#define TDBGC1                                           0x34000100
#define TDBGC10                                          0x34000a00
#define TDBGC11                                          0x34000b00
#define TDBGC1_SELECT                                    0x36000e00
#define TDBGC2                                           0x34000200
#define TDBGC2_SELECT                                    0x36000f00
#define TDBGC3                                           0x34000300
#define TDBGC3_SELECT                                    0x36001000
#define TDBGC4                                           0x34000400
#define TDBGC4_SELECT                                    0x36001100
#define TDBGC5                                           0x34000500
#define TDBGC5_SELECT                                    0x36001200
#define TDBGC6                                           0x34000600
#define TDBGC7                                           0x34000700
#define TDBGC8                                           0x34000800
#define TDBGC9                                           0x34000900
#define TDFR                                             0x00005600
#define TDVLN                                            0x00005f00
#define TEDF                                             0x00005700
#define TERR                                             0x00005d00
#define TFCS                                             0x00005300
#define TFRG                                             0x00005c00
#define TJBR                                             0x00005200
#define TLCL                                             0x00005a00
#define TMCA                                             0x00004e00
#define TMCL                                             0x00005900
#define TMGV                                             0x00004600
#define TNCL                                             0x00006e00
#define TOP_BS_PLL0_CTRL_3                               0x0204bc00
#define TOP_BS_PLL1_CTRL_3                               0x02078c00
#define TOP_CLOCKING_ENFORCE_PSG                         0x02032400
#define TOP_CORE_PLL_CTRL2                               0x02038800
#define TOP_CORE_PLL_CTRL4                               0x02039000
#define TOP_MISC_CONTROL_1                               0x02031400
#define TOP_PVTMON_CTRL_0                                0x02050000
#define TOP_PVTMON_CTRL_1                                0x02050400
#define TOP_SOFT_RESET_REG                               0x02030400
#define TOP_SOFT_RESET_REG_2                             0x02030800
#define TOP_XG_PLL0_CTRL_3                               0x02042c00
#define TOP_XG_PLL1_CTRL_3                               0x02044400
#define TOTALDYNCELLRESETLIMIT                           0x02030400
#define TOTALDYNCELLSETLIMIT                             0x02025b00
#define TOVR                                             0x00005500
#define TPCE                                             0x34000c00
#define TPFC0                                            0x00006200
#define TPFC1                                            0x00006300
#define TPFC2                                            0x00006400
#define TPFC3                                            0x00006500
#define TPFC4                                            0x00006600
#define TPFC5                                            0x00006700
#define TPFC6                                            0x00006800
#define TPFC7                                            0x00006900
#define TPKT                                             0x00004c00
#define TPOK                                             0x00004b00
#define TRPKT                                            0x00006000
#define TSCL                                             0x00005800
#define TUCA                                             0x00004d00
#define TUFL                                             0x00006100
#define TVLN                                             0x00005e00
#define TXCF                                             0x00005400
#define TXCL                                             0x00005b00
#define TXPF                                             0x00005000
#define TXPP                                             0x00005100
#define TX_EEE_LPI_DURATION_COUNTER                      0x00006b00
#define TX_EEE_LPI_EVENT_COUNTER                         0x00006a00
#define TX_HCFC_COUNTER                                  0x00006d00
#define TX_IPG_LENGTH                                    0x00011700
#define TX_LLFC_LOG_COUNTER                              0x00006c00
#define UNKNOWN_MCAST_BLOCK_MASK_64                      0x44000200
#define UNKNOWN_UCAST_BLOCK_MASK_64                      0x44000000
#define VFP_POLICY_PARITY_CONTROL                        0x12011000
#define VLAN_PARITY_CONTROL                              0x16010200
#define VLAN_PROTOCOL_DATA_PARITY_CONTROL                0x12010c00
#define VLAN_STG_PARITY_CONTROL                          0x16010400
#define VLAN_SUBNET_CAM_DBGCTRL                          0x12010200
#define VLAN_SUBNET_DATA_PARITY_CONTROL                  0x12010e00
#define VLAN_XLATE_PARITY_CONTROL                        0x12011200
#define WRRWEIGHT_COS                                    0x00013300
#define XLMAC_CTRL                                       0x00060000
#define XLMAC_MODE                                       0x00060100
#define XLMAC_PAUSE_CTRL                                 0x00060d00
#define XLMAC_PFC_CTRL                                   0x00060e00
#define XLMAC_RX_CTRL                                    0x00060600
#define XLMAC_RX_LSS_CTRL                                0x00060a00
#define XLMAC_RX_MAX_SIZE                                0x00060800
#define XLMAC_TX_CTRL                                    0x00060400
#define XLPORT_CNTMAXSIZE                                0x00020100
#define XLPORT_ECC_CONTROL                               0x02021a00
#define XLPORT_ENABLE_REG                                0x02020b00
#define XLPORT_FAULT_LINK_STATUS                         0x00020600
#define XLPORT_INTR_ENABLE                               0x02023000
#define XLPORT_LED_CHAIN_CONFIG                          0x02022b00
#define XLPORT_MAC_CONTROL                               0x02021000
#define XLPORT_MIB_RESET                                 0x02022c00
#define XLPORT_MODE_REG                                  0x02020a00
#define XLPORT_SOFT_RESET                                0x02020c00
#define XLPORT_TXFIFO_CTRL                               0x00020300
#define XLPORT_XGXS0_CTRL_REG                            0x02021400
#define XQCOSARBSEL                                      0x00013200

/* Switch memories: S-Channel address of entry 0 */
#define CPU_COS_MAP                                      0x444c0000
#define DSCP_TABLE                                       0x28000000
#define EFP_TCAM                                         0x2c480000
#define EGR_ENABLE                                       0x345c0000
#define EGR_L3_INTF                                      0x0c080000
#define EGR_L3_NEXT_HOP                                  0x0c040000
#define EGR_PRI_CNG_MAP                                  0x141c0000
#define EGR_VLAN                                         0x100c0000
#define EGR_VLAN_STG                                     0x10100000
#define FP_GLOBAL_MASK_TCAM                              0x3c180000
#define FP_POLICY_TABLE                                  0x3c0c0000
#define FP_PORT_FIELD_SEL                                0x38000000
#define FP_SLICE_MAP                                     0x3c080000
#define FP_STORM_CONTROL_METERS                          0x3c1c0000
#define FP_TCAM                                          0x3c040000
#define IARB_TDM_TABLE                                   0x00080000
#define IFP_REDIRECTION_PROFILE                          0x40040000
#define ING_L3_NEXT_HOP                                  0x40000000
#define ING_PHYSICAL_TO_LOGICAL_PORT_NUMBER_MAPPING_TABLE 0x000c0000
#define ING_PRI_CNG_MAP                                  0x381c0000
#define ING_SERVICE_PRI_MAP                              0x40080000
#define ING_VLAN_RANGE                                   0x081c0000
#define ING_VLAN_TAG_ACTION_PROFILE                      0x102c0000
#define IPORT_TABLE                                      0x04040000
#define L2_USER_ENTRY                                    0x1c100000
#define L3_DEFIP                                         0x30000000
#define L3_MTU_VALUES                                    0x44440000
#define LPORT_TAB                                        0x08100000
#define MMU_ARB_TDM_TABLE                                0x2c078000
#define PORT_COS_MAP                                     0x44480000
#define PORT_OR_TRUNK_MAC_LIMIT                          0x1c240000
#define PORT_TAB                                         0x04000000
#define STG_TAB                                          0x14040000
#define SYSTEM_CONFIG_TABLE                              0x04080000
#define TCP_FN                                           0x38100000
#define TOS_FN                                           0x38180000
#define TTL_FN                                           0x38140000
#define VFP_TCAM                                         0x10240000
#define VLAN_OR_VFI_MAC_LIMIT                            0x1c2c0000
#define VLAN_PROFILE_TAB                                 0x14080000
#define VLAN_SUBNET                                      0x10080000
#define VLAN_SUBNET_ONLY                                 0x100c0000
#define VLAN_TAB                                         0x14000000

static const struct bcm_cmic_mii_wr serdes_seq_1[] = {
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x00, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x082b },
	{ 0x1f, 0x8400 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0028 },
	{ 0x1f, 0x0c00 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0125 },
	{ 0x1f, 0x011b }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x002a },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x001f }, { 0x1f, 0xbc00 },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc00 }, { 0x09, 0x0600 }, { 0x00, 0x3300 },
	{ 0x1c, 0x5400 }, { 0x1c, 0xd401 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x3b00 }, { 0x1c, 0xfc00 },
	{ 0x1c, 0x7800 }, { 0x1c, 0xf820 }, { 0x10, 0x0020 }, { 0x18, 0x0007 }, { 0x18, 0x4007 }, { 0x0d, 0x0007 },
	{ 0x0e, 0x803d }, { 0x0d, 0x4007 }, { 0x0d, 0x0007 }, { 0x0e, 0x803d }, { 0x0d, 0x4007 }, { 0x0e, 0x003d },
	{ 0x0d, 0x0007 }, { 0x0e, 0x003c }, { 0x0d, 0x4007 }, { 0x0d, 0x0007 }, { 0x0e, 0x003c }, { 0x0d, 0x4007 },
	{ 0x0e, 0x0038 }, { 0x17, 0x0faf },
};

static const struct bcm_cmic_mii_wr serdes_seq_2[] = {
	{ 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x001f }, { 0x1f, 0xbc0f }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 },
	{ 0x1c, 0xb410 }, { 0x1c, 0xb863 }, { 0x1c, 0xa408 }, { 0x17, 0x0f04 }, { 0x15, 0x0000 }, { 0x1c, 0x0800 },
	{ 0x1c, 0x8800 }, { 0x00, 0x1b40 }, { 0x09, 0x0600 }, { 0x04, 0x05e1 }, { 0x09, 0x0700 }, { 0x00, 0x1b40 },
	{ 0x18, 0x7007 }, { 0x18, 0xf007 },
};

static const struct bcm_cmic_mii_wr serdes_seq_3[] = {
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x00, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x082b },
	{ 0x1f, 0x8400 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0028 },
	{ 0x1f, 0x0c00 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0125 },
	{ 0x1f, 0x011b }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x002a },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x001f }, { 0x1f, 0xbc00 },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc00 }, { 0x09, 0x0600 }, { 0x00, 0x3300 },
	{ 0x1c, 0x5400 }, { 0x1c, 0xd401 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x3b00 }, { 0x1c, 0xfc00 },
	{ 0x1c, 0x7800 }, { 0x1c, 0xf820 }, { 0x10, 0x0020 }, { 0x18, 0x0007 }, { 0x18, 0x4007 }, { 0x0d, 0x0007 },
	{ 0x0e, 0x803d }, { 0x0d, 0x4007 }, { 0x0d, 0x0007 }, { 0x0e, 0x803d }, { 0x0d, 0x4007 }, { 0x0e, 0x003d },
	{ 0x0d, 0x0007 }, { 0x0e, 0x003c }, { 0x0d, 0x4007 }, { 0x0d, 0x0007 }, { 0x0e, 0x003c }, { 0x0d, 0x4007 },
	{ 0x0e, 0x0038 }, { 0x17, 0x0faf }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0800 }, { 0x1f, 0x8000 },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x001f }, { 0x1f, 0xbc0f },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0xb410 }, { 0x1c, 0xb863 }, { 0x1c, 0xa408 }, { 0x17, 0x0f04 },
	{ 0x15, 0x0000 }, { 0x1c, 0x0800 }, { 0x1c, 0x8800 }, { 0x00, 0x1b40 }, { 0x09, 0x0600 }, { 0x04, 0x05e1 },
	{ 0x09, 0x0700 }, { 0x00, 0x1b40 }, { 0x18, 0x7007 }, { 0x18, 0xf007 },
};

static const struct bcm_cmic_mii_wr phy_seq_4[] = {
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x00, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x082b },
	{ 0x1f, 0x8400 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x01c6 },
	{ 0x1f, 0x0007 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0028 },
	{ 0x1f, 0x0c00 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0125 },
	{ 0x1f, 0x011b }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x002a },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x001f }, { 0x1f, 0xbc00 },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc00 }, { 0x09, 0x0600 }, { 0x00, 0x3300 },
	{ 0x1c, 0x5400 }, { 0x1c, 0xd401 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x3b00 }, { 0x1c, 0xfc00 },
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x3300 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 },
	{ 0x04, 0x01e0 }, { 0x1c, 0xfc00 }, { 0x1c, 0x5400 }, { 0x1c, 0xd401 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc02 },
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x1340 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7800 }, { 0x1c, 0xf827 },
	{ 0x10, 0x0020 }, { 0x18, 0x0007 }, { 0x18, 0x4007 }, { 0x0d, 0x0007 }, { 0x0e, 0x803d }, { 0x0d, 0x4007 },
	{ 0x0d, 0x0007 }, { 0x0e, 0x803d }, { 0x0d, 0x4007 }, { 0x0e, 0x003d }, { 0x0d, 0x0007 }, { 0x0e, 0x003c },
	{ 0x0d, 0x4007 }, { 0x0d, 0x0007 }, { 0x0e, 0x003c }, { 0x0d, 0x4007 }, { 0x0e, 0x0038 }, { 0x17, 0x0faf },
	{ 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0800 }, { 0x1f, 0x8000 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 },
	{ 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x001f }, { 0x1f, 0xbc0f }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 },
	{ 0x1c, 0xb410 }, { 0x1c, 0xb863 }, { 0x1c, 0xa408 }, { 0x17, 0x0f04 }, { 0x15, 0x0000 }, { 0x1c, 0x0800 },
	{ 0x1c, 0x8800 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 },
	{ 0x1e, 0x0233 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x1340 },
	{ 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 },
	{ 0x04, 0x00a0 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x17, 0x0f7e },
	{ 0x15, 0x0000 }, { 0x1e, 0x0233 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 },
	{ 0x00, 0x1340 }, { 0x1c, 0xfc00 }, { 0x1c, 0x5400 }, { 0x1c, 0xd401 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 },
	{ 0x04, 0x00a0 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 },
	{ 0x1c, 0xfc01 }, { 0x00, 0x1340 }, { 0x1c, 0xfc00 },
};

static const struct bcm_cmic_mii_wr serdes_seq_5[] = {
	{ 0x1f, 0x0000 }, { 0x13, 0x0040 }, { 0x1f, 0x0000 }, { 0x1f, 0x0000 }, { 0x13, 0x0000 }, { 0x1f, 0x0002 },
	{ 0x10, 0x0002 }, { 0x1f, 0x0000 }, { 0x00, 0x0140 }, { 0x1f, 0x0000 }, { 0x10, 0x0042 }, { 0x1f, 0x0000 },
	{ 0x1f, 0x0002 }, { 0x11, 0x0001 }, { 0x1f, 0x0000 }, { 0x1f, 0x0003 }, { 0x1f, 0x0003 }, { 0x12, 0x0180 },
	{ 0x1f, 0x0000 }, { 0x1f, 0xffd0 }, { 0x1e, 0x0000 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 },
	{ 0x1f, 0x8310 },
};

static const struct bcm_cmic_mii_wr phy_seq_6[] = {
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x00, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x082b },
	{ 0x1f, 0x8400 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x01c6 },
	{ 0x1f, 0x0007 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0028 },
	{ 0x1f, 0x0c00 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0125 },
	{ 0x1f, 0x011b }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x002a },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x001f }, { 0x1f, 0xbc00 },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc00 }, { 0x09, 0x0600 }, { 0x00, 0x3300 },
	{ 0x1c, 0x5400 }, { 0x1c, 0xd401 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x3b00 }, { 0x1c, 0xfc00 },
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x3300 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 },
	{ 0x04, 0x01e0 }, { 0x1c, 0xfc00 }, { 0x1c, 0x5400 }, { 0x1c, 0xd401 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc02 },
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x1340 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7800 }, { 0x1c, 0xf827 },
	{ 0x10, 0x0020 }, { 0x18, 0x0007 }, { 0x18, 0x4007 }, { 0x0d, 0x0007 }, { 0x0e, 0x803d }, { 0x0d, 0x4007 },
	{ 0x0d, 0x0007 }, { 0x0e, 0x803d }, { 0x0d, 0x4007 }, { 0x0e, 0x003d }, { 0x0d, 0x0007 }, { 0x0e, 0x003c },
	{ 0x0d, 0x4007 }, { 0x0d, 0x0007 }, { 0x0e, 0x003c }, { 0x0d, 0x4007 }, { 0x0e, 0x0038 }, { 0x17, 0x0faf },
};

static const struct bcm_cmic_mii_wr phy_seq_7[] = {
	{ 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x001f }, { 0x1f, 0xbc0f }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 },
	{ 0x1c, 0xb410 }, { 0x1c, 0xb863 }, { 0x1c, 0xa408 }, { 0x17, 0x0f04 }, { 0x15, 0x0000 }, { 0x1c, 0x0800 },
	{ 0x1c, 0x8800 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 },
	{ 0x1e, 0x0233 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x00, 0x1340 },
	{ 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 },
	{ 0x04, 0x00a0 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x17, 0x0f7e },
	{ 0x15, 0x0000 }, { 0x1e, 0x0233 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 },
	{ 0x00, 0x1340 }, { 0x1c, 0xfc00 }, { 0x1c, 0x5400 }, { 0x1c, 0xd401 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 },
	{ 0x04, 0x00a0 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 },
	{ 0x1c, 0xfc01 }, { 0x00, 0x1340 }, { 0x1c, 0xfc00 },
};

static const struct bcm_cmic_mii_wr serdes_seq_8[] = {
	{ 0x1f, 0xffd0 }, { 0x1e, 0x2800 }, { 0x1f, 0x8000 }, { 0x1d, 0x400f }, { 0x1f, 0x0000 }, { 0x00, 0xa040 },
	{ 0x1f, 0xffd0 }, { 0x1e, 0x0000 }, { 0x1f, 0x8310 }, { 0x1f, 0x8000 }, { 0x10, 0x0642 }, { 0x1f, 0xffd0 },
	{ 0x1e, 0x03ff }, { 0x1f, 0x8300 }, { 0x18, 0x7500 }, { 0x1f, 0x8130 }, { 0x11, 0x0000 }, { 0x1f, 0xffd0 },
	{ 0x1e, 0x0000 }, { 0x1f, 0x8000 }, { 0x10, 0x2642 }, { 0x1f, 0x8010 }, { 0x1f, 0xffe0 }, { 0x10, 0x8000 },
	{ 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 },
	{ 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 },
	{ 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0xffe0 },
	{ 0x1f, 0xffe0 }, { 0x1f, 0xffe0 }, { 0x1f, 0x8110 }, { 0x11, 0x0008 }, { 0x1f, 0x8010 }, { 0x1a, 0x0100 },
	{ 0x1f, 0x8310 }, { 0x1f, 0xffe0 }, { 0x14, 0x00a0 }, { 0x1f, 0x8320 }, { 0x19, 0x0000 }, { 0x1f, 0x8320 },
	{ 0x1f, 0x8350 }, { 0x10, 0x0000 }, { 0x1f, 0x8350 }, { 0x17, 0x0000 }, { 0x1f, 0x8300 }, { 0x11, 0x000e },
	{ 0x1f, 0xffe0 }, { 0x10, 0x0140 }, { 0x1f, 0x80a0 }, { 0x17, 0x0990 }, { 0x1f, 0x80a0 }, { 0x18, 0x0990 },
	{ 0x1f, 0x8300 }, { 0x1f, 0x8310 }, { 0x1f, 0x8300 }, { 0x12, 0x0184 }, { 0x1f, 0x8310 }, { 0x1f, 0x8050 },
	{ 0x1e, 0x1000 }, { 0x1f, 0x8050 }, { 0x1c, 0x0004 }, { 0x1f, 0x8260 }, { 0x10, 0x0021 }, { 0x1f, 0x8260 },
	{ 0x12, 0x3800 }, { 0x1f, 0x8260 }, { 0x13, 0x421b }, { 0x1f, 0x8260 }, { 0x14, 0x53e4 }, { 0x1f, 0x8260 },
	{ 0x17, 0x14a5 }, { 0x1f, 0x8260 }, { 0x18, 0x03e0 }, { 0x1f, 0x8260 }, { 0x19, 0x0fc0 }, { 0x1f, 0x8260 },
	{ 0x1b, 0x0312 }, { 0x1f, 0x8260 }, { 0x1c, 0x046a }, { 0x1f, 0x8310 }, { 0x1f, 0x81f0 }, { 0x12, 0xe070 },
	{ 0x1f, 0x81f0 }, { 0x13, 0xc0d0 }, { 0x1f, 0x81f0 }, { 0x14, 0xa0b0 }, { 0x1f, 0x81f0 }, { 0x15, 0x8090 },
	{ 0x1f, 0x81f0 }, { 0x16, 0xf0f0 }, { 0x1f, 0x81f0 }, { 0x17, 0xf0f0 }, { 0x1f, 0x81f0 }, { 0x18, 0xf0f0 },
	{ 0x1f, 0x81f0 }, { 0x19, 0xf0f0 }, { 0x1f, 0x8300 }, { 0x18, 0xf0e0 }, { 0x1f, 0x8330 }, { 0x1f, 0xffe0 },
	{ 0x10, 0x1021 }, { 0x1f, 0x8010 }, { 0x19, 0x0000 }, { 0x1f, 0x0000 }, { 0x1f, 0xffd0 }, { 0x1e, 0x0000 },
	{ 0x1f, 0x8310 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 },
};

static const struct bcm_cmic_mii_wr serdes_seq_9[] = {
	{ 0x1f, 0x0000 }, { 0x1f, 0x0000 }, { 0x1f, 0x0000 }, { 0x13, 0xc090 }, { 0x1f, 0x0002 }, { 0x10, 0x1022 },
	{ 0x1f, 0x0000 }, { 0x00, 0x0140 }, { 0x1f, 0x0000 }, { 0x10, 0x1062 }, { 0x1f, 0x0000 }, { 0x11, 0x000a },
	{ 0x1f, 0x0002 }, { 0x11, 0x0001 }, { 0x1f, 0x0000 }, { 0x1f, 0x0003 }, { 0x1f, 0x0003 }, { 0x12, 0xe1f0 },
	{ 0x1f, 0x0000 }, { 0x1f, 0xffd0 }, { 0x1e, 0x0000 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 },
	{ 0x1f, 0x8310 },
};

static const struct bcm_cmic_mii_wr serdes_seq_10[] = {
	{ 0x1f, 0x0000 }, { 0x13, 0xc0d0 }, { 0x1f, 0x0000 }, { 0x1f, 0x0000 }, { 0x13, 0xc090 }, { 0x1f, 0x0002 },
	{ 0x1f, 0x0000 }, { 0x00, 0x0140 }, { 0x1f, 0x0000 }, { 0x1f, 0x0000 }, { 0x11, 0x0000 }, { 0x1f, 0x0002 },
	{ 0x11, 0x0001 }, { 0x1f, 0x0000 }, { 0x1f, 0x0003 }, { 0x1f, 0x0003 }, { 0x1f, 0x0000 }, { 0x1f, 0xffd0 },
	{ 0x1e, 0x0000 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 },
};

static const struct bcm_cmic_mii_wr serdes_seq_11[] = {
	{ 0x1f, 0x0000 }, { 0x13, 0xc0d0 }, { 0x1f, 0x0000 }, { 0x1f, 0x0000 }, { 0x13, 0xc090 }, { 0x1f, 0x0002 },
	{ 0x1f, 0x0000 }, { 0x00, 0x0140 }, { 0x1f, 0x0000 }, { 0x1f, 0x0000 }, { 0x11, 0x0000 }, { 0x1f, 0x0002 },
	{ 0x11, 0x0001 }, { 0x1f, 0x0000 }, { 0x1f, 0x0003 }, { 0x1f, 0x0003 },
};

static const struct bcm_cmic_mii_wr phy_seq_12[] = {
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 },
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 },
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0202 },
	{ 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 },
};

static const struct bcm_cmic_mii_wr phy_seq_13[] = {
	{ 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0202 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 }, { 0x1c, 0x7c00 },
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 },
};

static const struct bcm_cmic_mii_wr phy_seq_14[] = {
	{ 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 }, { 0x1c, 0x7c00 }, { 0x1c, 0xfc01 }, { 0x1c, 0xfc00 },
	{ 0x17, 0x0f7e }, { 0x15, 0x0000 }, { 0x1e, 0x0202 }, { 0x1e, 0x0087 }, { 0x1f, 0x8000 },
};

static const u32 soft_reset_blk_port[][2] = {
	{ BLK_XLPORT0, 0x20 }, { BLK_XLPORT0, 0x20 }, { BLK_XLPORT0, 0x24 }, { BLK_XLPORT0, 0x2c },
	{ BLK_XLPORT0, 0x2e }, { BLK_XLPORT1, 0x20 }, { BLK_XLPORT1, 0x20 }, { BLK_XLPORT1, 0x24 },
	{ BLK_XLPORT1, 0x2c }, { BLK_XLPORT1, 0x2e },
};

static const u32 ing_hw_reset_control_1[][2] = {
	{ 0x10240000, 0x20200 }, { 0x1c100000, 0x20080 }, { 0x1c140000, 0x20080 }, { 0x30000000, 0x20200 },
	{ 0x30040000, 0x20200 }, { 0x3c040000, 0x20800 }, { 0x3c180000, 0x20800 }, { 0x444c0000, 0x20080 },
	{ 0x44500000, 0x20080 },
};

static const u32 port_map_ecc_vals[] = {
	0x1f, 9, 8, 7, 6, 5, 4, 3, 2, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1f, 0x1d, 0x1f, 0x1f, 0x1f,
};

static const u32 port_map_ecc_vals_2[] = {
	0, 0x3f, 9, 8, 7, 6, 5, 4, 3, 2, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1e,
};

static const u32 log_to_phy_port_mapping[][2] = {
	{ 0, 0 }, { 2, 9 }, { 3, 8 }, { 4, 7 }, { 5, 6 }, { 6, 5 }, { 7, 4 }, { 8, 3 }, { 9, 2 }, { 0xa, 0xa },
	{ 0xb, 0xb }, { 0xc, 0xc }, { 0xd, 0xd }, { 0xe, 0xe }, { 0xf, 0xf }, { 0x10, 0x10 }, { 0x11, 0x11 },
	{ 0x12, 0x12 }, { 0x13, 0x13 }, { 0x14, 0x14 }, { 0x15, 0x15 }, { 0x16, 0x16 }, { 0x17, 0x17 },
	{ 0x18, 0x18 }, { 0x19, 0x19 }, { 0x1a, 0x1a }, { 0x1b, 0x1b }, { 0x1c, 0x1c }, { 0x1d, 0x1e },
};

static const u32 egr_edatabuf_parity_control[] = {
	0x2800, 0x3800, 0x3808, 0x3818, 0x3838, 0x383a, 0x387a, 0x38fa, 0x39fa, 0x79fa, 0xf9fa, 0x1f9fa, 0x3f9fa,
	0x3f9fb,
};

static const u32 port_map_ecc_regs[] = {
	VLAN_XLATE_PARITY_CONTROL, VLAN_SUBNET_DATA_PARITY_CONTROL, VLAN_PROTOCOL_DATA_PARITY_CONTROL,
	VLAN_PARITY_CONTROL, VLAN_STG_PARITY_CONTROL, SOURCE_TRUNK_MAP_PARITY_CONTROL, VFP_POLICY_PARITY_CONTROL,
	LMEP_PARITY_CONTROL, IARB_PKT_ECC_CONTROL,
};

static const u32 port_map_ecc_regs_2[] = {
	L2_ENTRY_PARITY_CONTROL, L2MC_PARITY_CONTROL, L3_ENTRY_PARITY_CONTROL, L3_DEFIP_PARITY_CONTROL,
	L3MC_PARITY_CONTROL, MA_STATE_PARITY_CONTROL, RMEP_PARITY_CONTROL, MAID_REDUCTION_PARITY_CONTROL,
	MA_INDEX_PARITY_CONTROL, INITIAL_NHOP_PARITY_CONTROL,
};

static const u32 port_map_ecc_regs_3[] = {
	ING_L3_NEXT_HOP_PARITY_CONTROL, IFP_METER_PARITY_CONTROL, IFP_COUNTER_PARITY_CONTROL,
	IFP_POLICY_PARITY_CONTROL, IFP_STORM_CONTROL_PARITY_CONTROL, L3_MTU_VALUES_PARITY_CONTROL,
	EGR_MASK_PARITY_CONTROL, MODPORT_MAP_SW_PARITY_CONTROL, MODPORT_MAP_IM_PARITY_CONTROL,
	MODPORT_MAP_EM_PARITY_CONTROL, OAM_LM_COUNTERS_PARITY_CONTROL,
};

static const u32 xlport_ecc_control[] = {
	0x100, 0x100, 0x100, 0x100, 0x140, 0x148, 0x148, 0x149, 0x149,
};

static const u32 port_map_ecc_regs_4[] = {
	GE0_EEE_CONFIG, GE1_EEE_CONFIG, GE2_EEE_CONFIG, GE3_EEE_CONFIG, GE4_EEE_CONFIG, GE5_EEE_CONFIG,
	GE6_EEE_CONFIG, GE7_EEE_CONFIG, GE0_EEE_CONFIG, GE1_EEE_CONFIG, GE2_EEE_CONFIG,
};

static const u32 egr_enable[] = {
	0, 9, 8, 7, 6, 5, 4, 3, 2, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
	0x19, 0x1a, 0x1b, 0x1c, 0x1e,
};

static const u32 port_config_led_blk_port[][3] = {
	{ BLK_GPORT0, 3, BLK_GPORT0 }, { BLK_GPORT0, 3, BLK_GPORT0 }, { BLK_GPORT0, 3, BLK_GPORT0 },
	{ BLK_GPORT0, 3, BLK_GPORT0 }, { BLK_GPORT1, 3, BLK_GPORT1 }, { BLK_GPORT1, 3, BLK_GPORT1 },
	{ BLK_GPORT1, 3, BLK_GPORT1 }, { BLK_GPORT1, 3, BLK_GPORT1 }, { BLK_GPORT2, 3, BLK_GPORT2 },
	{ BLK_GPORT2, 3, BLK_GPORT2 }, { BLK_GPORT2, 3, BLK_GPORT2 }, { BLK_GPORT2, 3, BLK_GPORT2 },
	{ BLK_GPORT0, 1, BLK_GPORT0 }, { BLK_GPORT0, 1, BLK_GPORT0 }, { BLK_GPORT0, 1, BLK_GPORT0 },
	{ BLK_GPORT0, 1, BLK_GPORT0 }, { BLK_GPORT1, 1, BLK_GPORT1 }, { BLK_GPORT1, 1, BLK_GPORT1 },
	{ BLK_GPORT1, 1, BLK_GPORT1 }, { BLK_GPORT1, 1, BLK_GPORT1 }, { BLK_GPORT2, 1, BLK_GPORT2 },
	{ BLK_GPORT2, 1, BLK_GPORT2 }, { BLK_GPORT2, 1, BLK_GPORT2 }, { BLK_GPORT2, 1, BLK_GPORT2 },
};

static const u32 egr_vlan_control_1[] = {
	0, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
	0x19, 0x1a, 0x1b, 0x1c, 0x1d,
};

static const u32 iarb_tdm_table[][3] = {
	{ 0, 2, 9 }, { 1, 0xe, 0xe }, { 2, 0x1a, 0x1a }, { 3, 0x1b, 0x1b }, { 4, 0x1c, 0x1c }, { 5, 0x1e, 0x1d },
	{ 6, 3, 8 }, { 7, 0xf, 0xf }, { 8, 0x1a, 0x1a }, { 9, 0x1b, 0x1b }, { 0xa, 0x1c, 0x1c }, { 0xb, 0x1e, 0x1d },
	{ 0xc, 4, 7 }, { 0xd, 0x10, 0x10 }, { 0xe, 0x1a, 0x1a }, { 0xf, 0x1b, 0x1b }, { 0x10, 0x1c, 0x1c },
	{ 0x11, 0x1e, 0x1d }, { 0x12, 5, 6 }, { 0x13, 0x11, 0x11 }, { 0x14, 0x1a, 0x1a }, { 0x15, 0x1b, 0x1b },
	{ 0x16, 0x1c, 0x1c }, { 0x17, 0x1e, 0x1d }, { 0x19, 6, 5 }, { 0x1a, 0x12, 0x12 }, { 0x1b, 0x1a, 0x1a },
	{ 0x1c, 0x1b, 0x1b }, { 0x1d, 0x1c, 0x1c }, { 0x1e, 0x1e, 0x1d }, { 0x1f, 7, 4 }, { 0x20, 0x13, 0x13 },
	{ 0x21, 0x1a, 0x1a }, { 0x22, 0x1b, 0x1b }, { 0x23, 0x1c, 0x1c }, { 0x24, 0x1e, 0x1d }, { 0x25, 8, 3 },
	{ 0x26, 0x14, 0x14 }, { 0x27, 0x1a, 0x1a }, { 0x28, 0x1b, 0x1b }, { 0x29, 0x1c, 0x1c }, { 0x2a, 0x1e, 0x1d },
	{ 0x2b, 9, 2 }, { 0x2c, 0x15, 0x15 }, { 0x2d, 0x1a, 0x1a }, { 0x2e, 0x1b, 0x1b }, { 0x2f, 0x1c, 0x1c },
	{ 0x30, 0x1e, 0x1d }, { 0x31, 0x3f, 0x3f }, { 0x32, 0xa, 0xa }, { 0x33, 0x16, 0x16 }, { 0x34, 0x1a, 0x1a },
	{ 0x35, 0x1b, 0x1b }, { 0x36, 0x1c, 0x1c }, { 0x37, 0x1e, 0x1d }, { 0x38, 0xb, 0xb }, { 0x39, 0x17, 0x17 },
	{ 0x3a, 0x1a, 0x1a }, { 0x3b, 0x1b, 0x1b }, { 0x3c, 0x1c, 0x1c }, { 0x3d, 0x1e, 0x1d }, { 0x3e, 0xc, 0xc },
	{ 0x3f, 0x18, 0x18 }, { 0x40, 0x1a, 0x1a }, { 0x41, 0x1b, 0x1b }, { 0x42, 0x1c, 0x1c }, { 0x43, 0x1e, 0x1d },
	{ 0x44, 0xd, 0xd }, { 0x45, 0x19, 0x19 }, { 0x46, 0x1a, 0x1a }, { 0x47, 0x1b, 0x1b },
};

static const u32 holcospktsetlimit[] = {
	0x58f1648, 0x98c26f, 0x98c26f, 0x98c26f, 0x98c26f, 0x98c26f, 0x98c26f, 0x98c26f,
};

static const u32 egr_pri_cng_map[][2] = {
	{ 4, 2 }, { 8, 4 }, { 0xc, 6 }, { 0x10, 8 }, { 0x14, 0xa }, { 0x18, 0xc }, { 0x1c, 0xe }, { 7, 2 },
	{ 0xb, 4 }, { 0xf, 6 }, { 0x13, 8 }, { 0x17, 0xa }, { 0x1b, 0xc }, { 0x1f, 0xe }, { 1, 1 }, { 5, 3 },
	{ 9, 5 }, { 0xd, 7 }, { 0x11, 9 }, { 0x15, 0xb }, { 0x19, 0xd }, { 0x1d, 0xf }, { 0x84, 2 }, { 0x88, 4 },
	{ 0x8c, 6 }, { 0x90, 8 }, { 0x94, 0xa }, { 0x98, 0xc }, { 0x9c, 0xe }, { 0x87, 2 }, { 0x8b, 4 }, { 0x8f, 6 },
	{ 0x93, 8 }, { 0x97, 0xa }, { 0x9b, 0xc }, { 0x9f, 0xe }, { 0x81, 1 }, { 0x85, 3 }, { 0x89, 5 }, { 0x8d, 7 },
	{ 0x91, 9 }, { 0x95, 0xb }, { 0x99, 0xd }, { 0x9d, 0xf }, { 0xc4, 2 }, { 0xc8, 4 }, { 0xcc, 6 }, { 0xd0, 8 },
	{ 0xd4, 0xa }, { 0xd8, 0xc }, { 0xdc, 0xe }, { 0xc7, 2 }, { 0xcb, 4 }, { 0xcf, 6 }, { 0xd3, 8 },
	{ 0xd7, 0xa }, { 0xdb, 0xc }, { 0xdf, 0xe }, { 0xc1, 1 }, { 0xc5, 3 }, { 0xc9, 5 }, { 0xcd, 7 }, { 0xd1, 9 },
	{ 0xd5, 0xb }, { 0xd9, 0xd }, { 0xdd, 0xf }, { 0x104, 2 }, { 0x108, 4 }, { 0x10c, 6 }, { 0x110, 8 },
	{ 0x114, 0xa }, { 0x118, 0xc }, { 0x11c, 0xe }, { 0x107, 2 }, { 0x10b, 4 }, { 0x10f, 6 }, { 0x113, 8 },
	{ 0x117, 0xa }, { 0x11b, 0xc }, { 0x11f, 0xe }, { 0x101, 1 }, { 0x105, 3 }, { 0x109, 5 }, { 0x10d, 7 },
	{ 0x111, 9 }, { 0x115, 0xb }, { 0x119, 0xd }, { 0x11d, 0xf }, { 0x144, 2 }, { 0x148, 4 }, { 0x14c, 6 },
	{ 0x150, 8 }, { 0x154, 0xa }, { 0x158, 0xc }, { 0x15c, 0xe }, { 0x147, 2 }, { 0x14b, 4 }, { 0x14f, 6 },
	{ 0x153, 8 }, { 0x157, 0xa }, { 0x15b, 0xc }, { 0x15f, 0xe }, { 0x141, 1 }, { 0x145, 3 }, { 0x149, 5 },
	{ 0x14d, 7 }, { 0x151, 9 }, { 0x155, 0xb }, { 0x159, 0xd }, { 0x15d, 0xf }, { 0x184, 2 }, { 0x188, 4 },
	{ 0x18c, 6 }, { 0x190, 8 }, { 0x194, 0xa }, { 0x198, 0xc }, { 0x19c, 0xe }, { 0x187, 2 }, { 0x18b, 4 },
	{ 0x18f, 6 }, { 0x193, 8 }, { 0x197, 0xa }, { 0x19b, 0xc }, { 0x19f, 0xe }, { 0x181, 1 }, { 0x185, 3 },
	{ 0x189, 5 }, { 0x18d, 7 }, { 0x191, 9 }, { 0x195, 0xb }, { 0x199, 0xd }, { 0x19d, 0xf }, { 0x1c4, 2 },
	{ 0x1c8, 4 }, { 0x1cc, 6 }, { 0x1d0, 8 }, { 0x1d4, 0xa }, { 0x1d8, 0xc }, { 0x1dc, 0xe }, { 0x1c7, 2 },
	{ 0x1cb, 4 }, { 0x1cf, 6 }, { 0x1d3, 8 }, { 0x1d7, 0xa }, { 0x1db, 0xc }, { 0x1df, 0xe }, { 0x1c1, 1 },
	{ 0x1c5, 3 }, { 0x1c9, 5 }, { 0x1cd, 7 }, { 0x1d1, 9 }, { 0x1d5, 0xb }, { 0x1d9, 0xd }, { 0x1dd, 0xf },
	{ 0x204, 2 }, { 0x208, 4 }, { 0x20c, 6 }, { 0x210, 8 }, { 0x214, 0xa }, { 0x218, 0xc }, { 0x21c, 0xe },
	{ 0x207, 2 }, { 0x20b, 4 }, { 0x20f, 6 }, { 0x213, 8 }, { 0x217, 0xa }, { 0x21b, 0xc }, { 0x21f, 0xe },
	{ 0x201, 1 }, { 0x205, 3 }, { 0x209, 5 }, { 0x20d, 7 }, { 0x211, 9 }, { 0x215, 0xb }, { 0x219, 0xd },
	{ 0x21d, 0xf }, { 0x244, 2 }, { 0x248, 4 }, { 0x24c, 6 }, { 0x250, 8 }, { 0x254, 0xa }, { 0x258, 0xc },
	{ 0x25c, 0xe }, { 0x247, 2 }, { 0x24b, 4 }, { 0x24f, 6 }, { 0x253, 8 }, { 0x257, 0xa }, { 0x25b, 0xc },
	{ 0x25f, 0xe }, { 0x241, 1 }, { 0x245, 3 }, { 0x249, 5 }, { 0x24d, 7 }, { 0x251, 9 }, { 0x255, 0xb },
	{ 0x259, 0xd }, { 0x25d, 0xf }, { 0x284, 2 }, { 0x288, 4 }, { 0x28c, 6 }, { 0x290, 8 }, { 0x294, 0xa },
	{ 0x298, 0xc }, { 0x29c, 0xe }, { 0x287, 2 }, { 0x28b, 4 }, { 0x28f, 6 }, { 0x293, 8 }, { 0x297, 0xa },
	{ 0x29b, 0xc }, { 0x29f, 0xe }, { 0x281, 1 }, { 0x285, 3 }, { 0x289, 5 }, { 0x28d, 7 }, { 0x291, 9 },
	{ 0x295, 0xb }, { 0x299, 0xd }, { 0x29d, 0xf }, { 0x2c4, 2 }, { 0x2c8, 4 }, { 0x2cc, 6 }, { 0x2d0, 8 },
	{ 0x2d4, 0xa }, { 0x2d8, 0xc }, { 0x2dc, 0xe }, { 0x2c7, 2 }, { 0x2cb, 4 }, { 0x2cf, 6 }, { 0x2d3, 8 },
	{ 0x2d7, 0xa }, { 0x2db, 0xc }, { 0x2df, 0xe }, { 0x2c1, 1 }, { 0x2c5, 3 }, { 0x2c9, 5 }, { 0x2cd, 7 },
	{ 0x2d1, 9 }, { 0x2d5, 0xb }, { 0x2d9, 0xd }, { 0x2dd, 0xf }, { 0x304, 2 }, { 0x308, 4 }, { 0x30c, 6 },
	{ 0x310, 8 }, { 0x314, 0xa }, { 0x318, 0xc }, { 0x31c, 0xe }, { 0x307, 2 }, { 0x30b, 4 }, { 0x30f, 6 },
	{ 0x313, 8 }, { 0x317, 0xa }, { 0x31b, 0xc }, { 0x31f, 0xe }, { 0x301, 1 }, { 0x305, 3 }, { 0x309, 5 },
	{ 0x30d, 7 }, { 0x311, 9 }, { 0x315, 0xb }, { 0x319, 0xd }, { 0x31d, 0xf }, { 0x344, 2 }, { 0x348, 4 },
	{ 0x34c, 6 }, { 0x350, 8 }, { 0x354, 0xa }, { 0x358, 0xc }, { 0x35c, 0xe }, { 0x347, 2 }, { 0x34b, 4 },
	{ 0x34f, 6 }, { 0x353, 8 }, { 0x357, 0xa }, { 0x35b, 0xc }, { 0x35f, 0xe }, { 0x341, 1 }, { 0x345, 3 },
	{ 0x349, 5 }, { 0x34d, 7 }, { 0x351, 9 }, { 0x355, 0xb }, { 0x359, 0xd }, { 0x35d, 0xf }, { 0x384, 2 },
	{ 0x388, 4 }, { 0x38c, 6 }, { 0x390, 8 }, { 0x394, 0xa }, { 0x398, 0xc }, { 0x39c, 0xe }, { 0x387, 2 },
	{ 0x38b, 4 }, { 0x38f, 6 }, { 0x393, 8 }, { 0x397, 0xa }, { 0x39b, 0xc }, { 0x39f, 0xe }, { 0x381, 1 },
	{ 0x385, 3 }, { 0x389, 5 }, { 0x38d, 7 }, { 0x391, 9 }, { 0x395, 0xb }, { 0x399, 0xd }, { 0x39d, 0xf },
	{ 0x3c4, 2 }, { 0x3c8, 4 }, { 0x3cc, 6 }, { 0x3d0, 8 }, { 0x3d4, 0xa }, { 0x3d8, 0xc }, { 0x3dc, 0xe },
	{ 0x3c7, 2 }, { 0x3cb, 4 }, { 0x3cf, 6 }, { 0x3d3, 8 }, { 0x3d7, 0xa }, { 0x3db, 0xc }, { 0x3df, 0xe },
	{ 0x3c1, 1 }, { 0x3c5, 3 }, { 0x3c9, 5 }, { 0x3cd, 7 }, { 0x3d1, 9 }, { 0x3d5, 0xb }, { 0x3d9, 0xd },
	{ 0x3dd, 0xf }, { 0x404, 2 }, { 0x408, 4 }, { 0x40c, 6 }, { 0x410, 8 }, { 0x414, 0xa }, { 0x418, 0xc },
	{ 0x41c, 0xe }, { 0x407, 2 }, { 0x40b, 4 }, { 0x40f, 6 }, { 0x413, 8 }, { 0x417, 0xa }, { 0x41b, 0xc },
	{ 0x41f, 0xe }, { 0x401, 1 }, { 0x405, 3 }, { 0x409, 5 }, { 0x40d, 7 }, { 0x411, 9 }, { 0x415, 0xb },
	{ 0x419, 0xd }, { 0x41d, 0xf }, { 0x444, 2 }, { 0x448, 4 }, { 0x44c, 6 }, { 0x450, 8 }, { 0x454, 0xa },
	{ 0x458, 0xc }, { 0x45c, 0xe }, { 0x447, 2 }, { 0x44b, 4 }, { 0x44f, 6 }, { 0x453, 8 }, { 0x457, 0xa },
	{ 0x45b, 0xc }, { 0x45f, 0xe }, { 0x441, 1 }, { 0x445, 3 }, { 0x449, 5 }, { 0x44d, 7 }, { 0x451, 9 },
	{ 0x455, 0xb }, { 0x459, 0xd }, { 0x45d, 0xf }, { 0x484, 2 }, { 0x488, 4 }, { 0x48c, 6 }, { 0x490, 8 },
	{ 0x494, 0xa }, { 0x498, 0xc }, { 0x49c, 0xe }, { 0x487, 2 }, { 0x48b, 4 }, { 0x48f, 6 }, { 0x493, 8 },
	{ 0x497, 0xa }, { 0x49b, 0xc }, { 0x49f, 0xe }, { 0x481, 1 }, { 0x485, 3 }, { 0x489, 5 }, { 0x48d, 7 },
	{ 0x491, 9 }, { 0x495, 0xb }, { 0x499, 0xd }, { 0x49d, 0xf }, { 0x4c4, 2 }, { 0x4c8, 4 }, { 0x4cc, 6 },
	{ 0x4d0, 8 }, { 0x4d4, 0xa }, { 0x4d8, 0xc }, { 0x4dc, 0xe }, { 0x4c7, 2 }, { 0x4cb, 4 }, { 0x4cf, 6 },
	{ 0x4d3, 8 }, { 0x4d7, 0xa }, { 0x4db, 0xc }, { 0x4df, 0xe }, { 0x4c1, 1 }, { 0x4c5, 3 }, { 0x4c9, 5 },
	{ 0x4cd, 7 }, { 0x4d1, 9 }, { 0x4d5, 0xb }, { 0x4d9, 0xd }, { 0x4dd, 0xf }, { 0x504, 2 }, { 0x508, 4 },
	{ 0x50c, 6 }, { 0x510, 8 }, { 0x514, 0xa }, { 0x518, 0xc }, { 0x51c, 0xe }, { 0x507, 2 }, { 0x50b, 4 },
	{ 0x50f, 6 }, { 0x513, 8 }, { 0x517, 0xa }, { 0x51b, 0xc }, { 0x51f, 0xe }, { 0x501, 1 }, { 0x505, 3 },
	{ 0x509, 5 }, { 0x50d, 7 }, { 0x511, 9 }, { 0x515, 0xb }, { 0x519, 0xd }, { 0x51d, 0xf }, { 0x544, 2 },
	{ 0x548, 4 }, { 0x54c, 6 }, { 0x550, 8 }, { 0x554, 0xa }, { 0x558, 0xc }, { 0x55c, 0xe }, { 0x547, 2 },
	{ 0x54b, 4 }, { 0x54f, 6 }, { 0x553, 8 }, { 0x557, 0xa }, { 0x55b, 0xc }, { 0x55f, 0xe }, { 0x541, 1 },
	{ 0x545, 3 }, { 0x549, 5 }, { 0x54d, 7 }, { 0x551, 9 }, { 0x555, 0xb }, { 0x559, 0xd }, { 0x55d, 0xf },
	{ 0x584, 2 }, { 0x588, 4 }, { 0x58c, 6 }, { 0x590, 8 }, { 0x594, 0xa }, { 0x598, 0xc }, { 0x59c, 0xe },
	{ 0x587, 2 }, { 0x58b, 4 }, { 0x58f, 6 }, { 0x593, 8 }, { 0x597, 0xa }, { 0x59b, 0xc }, { 0x59f, 0xe },
	{ 0x581, 1 }, { 0x585, 3 }, { 0x589, 5 }, { 0x58d, 7 }, { 0x591, 9 }, { 0x595, 0xb }, { 0x599, 0xd },
	{ 0x59d, 0xf }, { 0x5c4, 2 }, { 0x5c8, 4 }, { 0x5cc, 6 }, { 0x5d0, 8 }, { 0x5d4, 0xa }, { 0x5d8, 0xc },
	{ 0x5dc, 0xe }, { 0x5c7, 2 }, { 0x5cb, 4 }, { 0x5cf, 6 }, { 0x5d3, 8 }, { 0x5d7, 0xa }, { 0x5db, 0xc },
	{ 0x5df, 0xe }, { 0x5c1, 1 }, { 0x5c5, 3 }, { 0x5c9, 5 }, { 0x5cd, 7 }, { 0x5d1, 9 }, { 0x5d5, 0xb },
	{ 0x5d9, 0xd }, { 0x5dd, 0xf }, { 0x604, 2 }, { 0x608, 4 }, { 0x60c, 6 }, { 0x610, 8 }, { 0x614, 0xa },
	{ 0x618, 0xc }, { 0x61c, 0xe }, { 0x607, 2 }, { 0x60b, 4 }, { 0x60f, 6 }, { 0x613, 8 }, { 0x617, 0xa },
	{ 0x61b, 0xc }, { 0x61f, 0xe }, { 0x601, 1 }, { 0x605, 3 }, { 0x609, 5 }, { 0x60d, 7 }, { 0x611, 9 },
	{ 0x615, 0xb }, { 0x619, 0xd }, { 0x61d, 0xf }, { 0x644, 2 }, { 0x648, 4 }, { 0x64c, 6 }, { 0x650, 8 },
	{ 0x654, 0xa }, { 0x658, 0xc }, { 0x65c, 0xe }, { 0x647, 2 }, { 0x64b, 4 }, { 0x64f, 6 }, { 0x653, 8 },
	{ 0x657, 0xa }, { 0x65b, 0xc }, { 0x65f, 0xe }, { 0x641, 1 }, { 0x645, 3 }, { 0x649, 5 }, { 0x64d, 7 },
	{ 0x651, 9 }, { 0x655, 0xb }, { 0x659, 0xd }, { 0x65d, 0xf }, { 0x684, 2 }, { 0x688, 4 }, { 0x68c, 6 },
	{ 0x690, 8 }, { 0x694, 0xa }, { 0x698, 0xc }, { 0x69c, 0xe }, { 0x687, 2 }, { 0x68b, 4 }, { 0x68f, 6 },
	{ 0x693, 8 }, { 0x697, 0xa }, { 0x69b, 0xc }, { 0x69f, 0xe }, { 0x681, 1 }, { 0x685, 3 }, { 0x689, 5 },
	{ 0x68d, 7 }, { 0x691, 9 }, { 0x695, 0xb }, { 0x699, 0xd }, { 0x69d, 0xf }, { 0x6c4, 2 }, { 0x6c8, 4 },
	{ 0x6cc, 6 }, { 0x6d0, 8 }, { 0x6d4, 0xa }, { 0x6d8, 0xc }, { 0x6dc, 0xe }, { 0x6c7, 2 }, { 0x6cb, 4 },
	{ 0x6cf, 6 }, { 0x6d3, 8 }, { 0x6d7, 0xa }, { 0x6db, 0xc }, { 0x6df, 0xe }, { 0x6c1, 1 }, { 0x6c5, 3 },
	{ 0x6c9, 5 }, { 0x6cd, 7 }, { 0x6d1, 9 }, { 0x6d5, 0xb }, { 0x6d9, 0xd }, { 0x6dd, 0xf }, { 0x704, 2 },
	{ 0x708, 4 }, { 0x70c, 6 }, { 0x710, 8 }, { 0x714, 0xa }, { 0x718, 0xc }, { 0x71c, 0xe }, { 0x707, 2 },
	{ 0x70b, 4 }, { 0x70f, 6 }, { 0x713, 8 }, { 0x717, 0xa }, { 0x71b, 0xc }, { 0x71f, 0xe }, { 0x701, 1 },
	{ 0x705, 3 }, { 0x709, 5 }, { 0x70d, 7 }, { 0x711, 9 }, { 0x715, 0xb }, { 0x719, 0xd }, { 0x71d, 0xf },
	{ 0x744, 2 }, { 0x748, 4 }, { 0x74c, 6 }, { 0x750, 8 }, { 0x754, 0xa }, { 0x758, 0xc }, { 0x75c, 0xe },
	{ 0x747, 2 }, { 0x74b, 4 }, { 0x74f, 6 }, { 0x753, 8 }, { 0x757, 0xa }, { 0x75b, 0xc }, { 0x75f, 0xe },
	{ 0x741, 1 }, { 0x745, 3 }, { 0x749, 5 }, { 0x74d, 7 }, { 0x751, 9 }, { 0x755, 0xb }, { 0x759, 0xd },
	{ 0x75d, 0xf },
};

static const u32 serdes_phy_init_phys[][2] = {
	{ 0x209, 0x202 }, { 0x208, 0x202 }, { 0x207, 0x202 }, { 0x206, 0x202 }, { 0x204, 0x201 }, { 0x203, 0x201 },
	{ 0x202, 0x201 },
};

static const u32 serdes_phy_init_phys_2[][2] = {
	{ 0x20c, 0x20b }, { 0x20d, 0x20b }, { 0x20e, 0x20b }, { 0x210, 0x20c }, { 0x211, 0x20c }, { 0x212, 0x20c },
	{ 0x213, 0x20c },
};

static const u32 serdes_phy_init_vals[][2] = {
	{ 0x1f, 0 }, { 0x1f, 0xffd0 }, { 0x1e, 0 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 }, { 0x1f, 0x8310 },
	{ 0x1f, 0x8310 },
};

static const u32 serdes_phy_init_vals_2[][3] = {
	{ 0x241, 0x1f, 0 }, { 0x241, 0x1f, 0xffd0 }, { 0x241, 0x1e, 0 }, { 0x241, 0x1f, 0x9000 },
	{ 0x241, 0x1f, 0x80f0 }, { 0x241, 0x1f, 0 }, { 0x242, 0x1f, 0 }, { 0x242, 0x1f, 0xffd0 }, { 0x242, 0x1e, 0 },
	{ 0x242, 0x1f, 0x9000 }, { 0x242, 0x1f, 0x80f0 }, { 0x242, 0x1f, 0 }, { 0x243, 0x1f, 0 },
	{ 0x243, 0x1f, 0xffd0 }, { 0x243, 0x1e, 0 }, { 0x243, 0x1f, 0x9000 }, { 0x243, 0x1f, 0x80f0 },
	{ 0x243, 0x1f, 0 }, { 0x245, 0x1f, 0 }, { 0x245, 0x1f, 0xffd0 }, { 0x245, 0x1e, 0 }, { 0x245, 0x1f, 0x9000 },
	{ 0x245, 0x1f, 0x80f0 }, { 0x245, 0x1f, 0 }, { 0x14, 0x1c, 0x7c00 }, { 0x14, 0x1c, 0xfc01 },
	{ 0x14, 0, 0x1b40 }, { 0x14, 0x1c, 0xfc00 }, { 0x251, 0x1f, 0 }, { 0x251, 0, 0x940 }, { 0x251, 0x1f, 3 },
	{ 0x251, 0x10, 0x1063 }, { 0x15, 0x1c, 0x7c00 }, { 0x15, 0x1c, 0xfc01 }, { 0x15, 0, 0x1b40 },
	{ 0x15, 0x1c, 0xfc00 }, { 0x251, 0x1f, 0x8010 }, { 0x251, 0x18, 0xf8f1 }, { 0x16, 0x1c, 0x7c00 },
	{ 0x16, 0x1c, 0xfc01 }, { 0x16, 0, 0x1b40 }, { 0x16, 0x1c, 0xfc00 }, { 0x251, 0x1f, 0 }, { 0x251, 0x1f, 3 },
	{ 0x17, 0x1c, 0x7c00 }, { 0x17, 0x1c, 0xfc01 }, { 0x17, 0, 0x1b40 }, { 0x17, 0x1c, 0xfc00 },
	{ 0x251, 0x1f, 0 }, { 0x251, 0x1f, 3 }, { 0x18, 0x1c, 0x7c00 }, { 0x18, 0x1c, 0xfc01 }, { 0x18, 0, 0x1b40 },
	{ 0x18, 0x1c, 0xfc00 }, { 0x251, 0x1f, 0 }, { 0x251, 0x1f, 3 }, { 0x19, 0x1c, 0x7c00 },
	{ 0x19, 0x1c, 0xfc01 }, { 0x19, 0, 0x1b40 }, { 0x19, 0x1c, 0xfc00 }, { 0x251, 0x1f, 0 }, { 0x251, 0x1f, 3 },
	{ 0x1a, 0x1c, 0x7c00 }, { 0x1a, 0x1c, 0xfc01 }, { 0x1a, 0, 0x1b40 }, { 0x1a, 0x1c, 0xfc00 },
	{ 0x251, 0x1f, 0 }, { 0x251, 0x1f, 3 }, { 0x1b, 0x1c, 0x7c00 }, { 0x1b, 0x1c, 0xfc01 }, { 0x1b, 0, 0x1b40 },
	{ 0x1b, 0x1c, 0xfc00 }, { 0x251, 0x1f, 0 }, { 0x251, 0x1f, 3 }, { 0x241, 0x1f, 0 }, { 0x241, 9, 1 },
	{ 0x242, 0x1f, 0 }, { 0x242, 9, 1 }, { 0x243, 0x1f, 0 }, { 0x243, 9, 1 }, { 0x245, 0x1f, 0 }, { 0x245, 9, 1 },
};

static const u32 command_config[] = {
	0x20b, 0x20c, 0x20d, 0x20e, 0x210, 0x211, 0x212, 0x213,
};

static const u32 port_mac_init_vals[][3] = {
	{ 0x14, 0x1c, 0x7c00 }, { 0x14, 0x1c, 0xfc01 }, { 0x14, 0x1c, 0xfc00 }, { 0x251, 0x1f, 3 },
	{ 0x251, 0x10, 0x1062 }, { 0x251, 0x1f, 0 }, { 0x251, 0, 0x140 }, { 0x14, 0x1c, 0x7c00 },
	{ 0x14, 0x1c, 0xfc01 }, { 0x14, 0, 0x1340 }, { 0x14, 0x1c, 0xfc00 }, { 0x251, 0x1f, 3 }, { 0x251, 0x1f, 0 },
	{ 0x251, 0x1f, 0x8310 },
};

static const u32 port_mac_init_vals_2[][3] = {
	{ 0x15, 0x1c, 0x7c00 }, { 0x15, 0x1c, 0xfc01 }, { 0x15, 0x1c, 0xfc00 }, { 0x251, 0x1f, 0x8010 },
	{ 0x251, 0x18, 0xf8e0 }, { 0x15, 0x1c, 0x7c00 }, { 0x15, 0x1c, 0xfc01 }, { 0x15, 0, 0x1340 },
	{ 0x15, 0x1c, 0xfc00 }, { 0x251, 0x1f, 0x8010 }, { 0x251, 0x1f, 0 },
};

static const u32 port_mac_init_vals_3[][3] = {
	{ 0x1b, 0x1c, 0x7c00 }, { 0x1b, 0x1c, 0xfc01 }, { 0x1b, 0x1c, 0xfc00 }, { 0x251, 0x1f, 3 },
	{ 0x251, 0x1f, 0 }, { 0x251, 0, 0x140 }, { 0x1b, 0x1c, 0x7c00 }, { 0x1b, 0x1c, 0xfc01 }, { 0x1b, 0, 0x1340 },
	{ 0x1b, 0x1c, 0xfc00 }, { 0x251, 0x1f, 3 }, { 0x251, 0x1f, 0 },
};

static const u32 cpu_cos_map[][2] = {
	{ 0xfffffc00, 0x7ff }, { 0xfffffc40, 0xfff }, { 0xfffffc80, 0x17ff }, { 0xfffffcc0, 0x1fff },
	{ 0xfffffd00, 0x27ff }, { 0xfffffd40, 0x2fff }, { 0xfffffd80, 0x37ff }, { 0xfffffdc0, 0x3fff },
	{ 0xffffffc0, 0x3fff },
};

static const u32 port_cos_map[] = {
	1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7,
};

static const u32 cmic_miim_scan_ctrl[][2] = {
	{ 0x380, 0x7000000 }, { 0x3c0, 0x7060000 }, { 0x3e0, 0x7060400 }, { 0x3f0, 0x7060403 },
};

static const u32 cmic_miim_scan_ctrl_2[][2] = {
	{ 0x1ffc, 0xd }, { 0x3ffc, 0xe0d }, { 0x7ffc, 0x100e0d }, { 0xfffc, 0x11100e0d },
};

static const u32 cmic_miim_scan_ctrl_3[] = {
	0x16, 0x1716, 0x181716, 0x19181716,
};

static const u32 miim_scan_counters_regs[][2] = {
	{ RDISC, BLK_IPIPE }, { RUC, BLK_IPIPE }, { RPORTD, BLK_IPIPE }, { RDBGC0, BLK_IPIPE }, { RDBGC1, BLK_IPIPE },
	{ RDBGC2, BLK_IPIPE }, { RDBGC3, BLK_IPIPE }, { RDBGC4, BLK_IPIPE }, { RDBGC5, BLK_IPIPE },
	{ RDBGC6, BLK_IPIPE }, { RDBGC7, BLK_IPIPE }, { RDBGC8, BLK_IPIPE }, { RIPD4, BLK_IPIPE },
	{ RIPC4, BLK_IPIPE }, { RIPHE4, BLK_IPIPE }, { IMRP4, BLK_IPIPE }, { RIPD6, BLK_IPIPE }, { RIPC6, BLK_IPIPE },
	{ RIPHE6, BLK_IPIPE }, { IMRP6, BLK_IPIPE }, { IUNHGI, BLK_IPIPE }, { ICTRL, BLK_IPIPE },
	{ IBCAST, BLK_IPIPE }, { ILTOMC, BLK_IPIPE }, { IIPMC, BLK_IPIPE }, { IUNKOPC, BLK_IPIPE },
	{ HOL_DROP, BLK_IPIPE }, { TDBGC0, BLK_EPIPE }, { TDBGC1, BLK_EPIPE }, { TDBGC2, BLK_EPIPE },
	{ TDBGC3, BLK_EPIPE }, { TDBGC4, BLK_EPIPE }, { TDBGC5, BLK_EPIPE }, { TDBGC6, BLK_EPIPE },
	{ TDBGC7, BLK_EPIPE }, { TDBGC8, BLK_EPIPE }, { TDBGC9, BLK_EPIPE }, { TDBGC10, BLK_EPIPE },
	{ TDBGC11, BLK_EPIPE }, { TPCE, BLK_EPIPE }, { CNGDROPCOUNT1, BLK_MMU }, { CNGDROPCOUNT0, BLK_MMU },
};

static const u32 miim_scan_counters_regs_2[][2] = {
	{ RDISC, BLK_IPIPE }, { RUC, BLK_IPIPE }, { RPORTD, BLK_IPIPE }, { RDBGC0, BLK_IPIPE }, { RDBGC1, BLK_IPIPE },
	{ RDBGC2, BLK_IPIPE }, { RDBGC3, BLK_IPIPE }, { RDBGC4, BLK_IPIPE }, { RDBGC5, BLK_IPIPE },
	{ RDBGC6, BLK_IPIPE }, { RDBGC7, BLK_IPIPE }, { RDBGC8, BLK_IPIPE }, { RIPD4, BLK_IPIPE },
	{ RIPC4, BLK_IPIPE }, { RIPHE4, BLK_IPIPE }, { IMRP4, BLK_IPIPE }, { RIPD6, BLK_IPIPE }, { RIPC6, BLK_IPIPE },
	{ RIPHE6, BLK_IPIPE }, { IMRP6, BLK_IPIPE }, { HOL_DROP, BLK_IPIPE }, { TDBGC0, BLK_EPIPE },
	{ TDBGC1, BLK_EPIPE }, { TDBGC2, BLK_EPIPE }, { TDBGC3, BLK_EPIPE }, { TDBGC4, BLK_EPIPE },
	{ TDBGC5, BLK_EPIPE }, { TDBGC6, BLK_EPIPE }, { TDBGC7, BLK_EPIPE }, { TDBGC8, BLK_EPIPE },
	{ TDBGC9, BLK_EPIPE }, { TDBGC10, BLK_EPIPE }, { TDBGC11, BLK_EPIPE }, { TPCE, BLK_EPIPE },
};

static const u32 miim_scan_counters_regs_3[] = {
	GRFCS, GRXCF, GRXPF, GRXUO, GRALN, GRFLR, GRCDE, GRFCR, GROVR, GRJBR, GRMTUE, GRRPKT, GRUND, GRFRG, GRRBYT,
	GRMCA, GRBCA, GR64, GR127, GR255, GR511, GR1023, GR1518, GRMGV, GR2047, GR4095, GR9216, GRPKT, GRBYT, GRUC,
	GRPOK, GTXPF, GTJBR, GTFCS, GTXCF, GTOVR, GTDFR, GTEDF, GTSCL, GTMCL, GTLCL, GTXCL, GTFRG, GTNCL, GTMCA,
	GTBCA, GT64, GT127, GT255, GT511, GT1023, GT1518, GTMGV, GT2047, GT4095, GT9216, GTPKT, GTBYT, GTUC, GTPOK,
	GRX_EEE_LPI_EVENT_COUNTER, GRX_EEE_LPI_DURATION_COUNTER, GTX_EEE_LPI_EVENT_COUNTER,
	GTX_EEE_LPI_DURATION_COUNTER,
};

static const u32 miim_scan_counters_regs_4[][2] = {
	{ RDISC, BLK_IPIPE }, { RUC, BLK_IPIPE }, { RPORTD, BLK_IPIPE }, { RDBGC0, BLK_IPIPE }, { RDBGC1, BLK_IPIPE },
	{ RDBGC2, BLK_IPIPE }, { RDBGC3, BLK_IPIPE }, { RDBGC4, BLK_IPIPE }, { RDBGC5, BLK_IPIPE },
	{ RDBGC6, BLK_IPIPE }, { RDBGC7, BLK_IPIPE }, { RDBGC8, BLK_IPIPE }, { RIPD4, BLK_IPIPE },
	{ RIPC4, BLK_IPIPE }, { RIPHE4, BLK_IPIPE }, { IMRP4, BLK_IPIPE }, { RIPD6, BLK_IPIPE }, { RIPC6, BLK_IPIPE },
	{ RIPHE6, BLK_IPIPE }, { IMRP6, BLK_IPIPE }, { IUNHGI, BLK_IPIPE }, { ICTRL, BLK_IPIPE },
	{ IBCAST, BLK_IPIPE }, { ILTOMC, BLK_IPIPE }, { IIPMC, BLK_IPIPE }, { IUNKOPC, BLK_IPIPE },
	{ HOL_DROP, BLK_IPIPE }, { TDBGC0, BLK_EPIPE }, { TDBGC1, BLK_EPIPE }, { TDBGC2, BLK_EPIPE },
	{ TDBGC3, BLK_EPIPE }, { TDBGC4, BLK_EPIPE }, { TDBGC5, BLK_EPIPE }, { TDBGC6, BLK_EPIPE },
	{ TDBGC7, BLK_EPIPE }, { TDBGC8, BLK_EPIPE }, { TDBGC9, BLK_EPIPE }, { TDBGC10, BLK_EPIPE },
	{ TDBGC11, BLK_EPIPE }, { TPCE, BLK_EPIPE },
};

static const u32 miim_scan_counters_regs_5[] = {
	R64, R127, R255, R511, R1023, R1518, RMGV, R2047, R4095, R9216, R16383, RPKT, RUCA, RMCA, RBCA, RFCS, RXCF,
	RXPF, RXPP, RXUO, RXUDA, RXWSA, RALN, RFLR, RERPKT, RFCR, ROVR, RJBR, RMTUE, RMCRC, RPRM, RVLN, RDVLN, RTRFU,
	RPOK, RPFCOFF0, RPFCOFF1, RPFCOFF2, RPFCOFF3, RPFCOFF4, RPFCOFF5, RPFCOFF6, RPFCOFF7, RPFC0, RPFC1, RPFC2,
	RPFC3, RPFC4, RPFC5, RPFC6, RPFC7, RSCHCRC, RUND, RFRG, RX_EEE_LPI_EVENT_COUNTER, RX_EEE_LPI_DURATION_COUNTER,
	RX_LLFC_PHY_COUNTER, RX_LLFC_LOG_COUNTER, RX_LLFC_CRC_COUNTER, RX_HCFC_COUNTER, RX_HCFC_CRC_COUNTER, RBYT,
	RRBYT, RRPKT, T64, T127, T255, T511, T1023, T1518, TMGV, T2047, T4095, T9216, T16383, TPOK, TPKT, TUCA, TMCA,
	TBCA, TXPF, TXPP, TJBR, TFCS, TXCF, TOVR, TDFR, TEDF, TSCL, TMCL, TLCL, TXCL, TFRG, TERR, TVLN, TDVLN, TRPKT,
	TUFL, TPFC0, TPFC1, TPFC2, TPFC3, TPFC4, TPFC5, TPFC6, TPFC7, TX_EEE_LPI_EVENT_COUNTER,
	TX_EEE_LPI_DURATION_COUNTER, TX_LLFC_LOG_COUNTER, TX_HCFC_COUNTER, TNCL, TBYT,
};

static const u32 egr_port[][2] = {
	{ 0, 0x80000108 }, { 3, 0x83000108 }, { 5, 0x85000108 }, { 7, 0x87000108 }, { 9, 0x89000108 },
	{ 0xb, 0x8b000108 }, { 0xd, 0x8d000108 }, { 0xf, 0x8f000108 }, { 0x11, 0x91000108 }, { 0x13, 0x93000108 },
	{ 0x15, 0x95000108 }, { 0x17, 0x97000108 }, { 0x19, 0x99000108 }, { 0x1b, 0x9b000108 },
};

static const u32 port_counters_mac_regs[] = {
	CMIC_BS0_OUTPUT_TIME_0, CMIC_BS1_OUTPUT_TIME_0, CMIC_BS0_CLK_CTRL, CMIC_BS1_CLK_CTRL, CMIC_BS0_HEARTBEAT_CTRL,
	CMIC_BS1_HEARTBEAT_CTRL, CMIC_TIMESYNC_INTERRUPT_ENABLE, CMIC_TIMESYNC_TIME_CAPTURE_CONTROL,
	CMIC_TIMESYNC_TIME_CAPTURE_MODE,
};

static const u32 port_counters_mac_vals[][3] = {
	{ 0x14, 0x1c, 0x7c00 }, { 0x14, 0x1c, 0xfc01 }, { 0x14, 0, 0x1b40 }, { 0x14, 0x1c, 0xfc00 },
	{ 0x251, 0x1f, 0 }, { 0x251, 0, 0x940 }, { 0x251, 0x1f, 3 }, { 0x251, 0x10, 0x1063 },
};

static const u32 port_enable_phys[] = {
	0x8310, 0, 0, 0, 0,
};

static const u32 protocol_pkt_control[] = {
	2, 3, 4, 5, 6, 7, 8, 9, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x1a, 0x1b, 0x1d,
};

static const u32 protocol_pkt_control_2[] = {
	0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x45, 0x45,
	0x45, 0x45, 0x45, 0x45, 0x40, 0x40, 0x45, 0x45, 0x40, 0x45,
};

static const u32 ing_egrmskbmap_64[] = {
	0x9840, 0x9840, 0x9840, 0x9840, 0x9840, 0x9840, 0x9840, 0x9840, 0x800, 0x800, 0x800, 0x800, 0x800, 0x800,
	0x800, 0x800, 0x9840, 0x9840, 0x9840, 0x9840, 0x9840, 0x9840, 0x800, 0x800, 0x9840, 0x9840, 0x800, 0x9840,
};

static const u32 igmp_mld_pkt_control[][2] = {
	{ 0, 0x800 }, { 2, 0x9940 }, { 3, 0x9940 }, { 4, 0x9940 }, { 5, 0x9940 }, { 6, 0x9940 }, { 7, 0x9940 },
	{ 8, 0x9940 }, { 9, 0x9940 }, { 0xa, 0x900 }, { 0xb, 0x900 }, { 0xc, 0x900 }, { 0xd, 0x900 }, { 0xe, 0x900 },
	{ 0xf, 0x900 }, { 0x10, 0x900 }, { 0x11, 0x900 }, { 0x12, 0x9940 }, { 0x13, 0x9940 }, { 0x14, 0x9940 },
	{ 0x15, 0x9940 }, { 0x16, 0x9940 }, { 0x17, 0x9940 }, { 0x18, 0x900 }, { 0x19, 0x900 }, { 0x1a, 0x9940 },
	{ 0x1b, 0x9940 }, { 0x1c, 0x900 }, { 0x1d, 0x9940 }, { 0, 0x900 },
};

static const u32 storm_control_meter_config[][2] = {
	{ 0x12, 0x48 }, { 0x13, 0x4c }, { 0x14, 0x50 }, { 0x15, 0x54 }, { 0x16, 0x58 }, { 0x17, 0x5c },
	{ 0x1a, 0x68 }, { 0x1b, 0x6c }, { 2, 8 }, { 3, 0xc }, { 4, 0x10 }, { 5, 0x14 }, { 6, 0x18 }, { 7, 0x1c },
	{ 8, 0x20 }, { 9, 0x24 },
};

static const u32 storm_control_meter_config_2[] = {
	0x100, 0x2fa0, 0x42fa0, 0x42fa0, 0x42fa0, 0x42fa0, 0x42fa0, 0x42fa0, 0x42fa0, 0x42fa0, 0x42fa0, 0x42fa0,
	0x42fa0, 0x42fa0, 0x42fa0,
};

static const u32 dscp_table[][2] = {
	{ 1, 1 }, { 0x81, 1 }, { 0xc1, 1 }, { 0x101, 1 }, { 0x141, 1 }, { 0x181, 1 }, { 0x1c1, 1 }, { 0x201, 1 },
	{ 0x241, 1 }, { 0x281, 1 }, { 0x2c1, 1 }, { 0x301, 1 }, { 0x341, 1 }, { 0x381, 1 }, { 0x3c1, 1 },
	{ 0x401, 1 }, { 0x441, 1 }, { 0x481, 1 }, { 0x4c1, 1 }, { 0x501, 1 }, { 0x541, 1 }, { 0x581, 1 },
	{ 0x5c1, 1 }, { 0x601, 1 }, { 0x641, 1 }, { 0x681, 1 }, { 0x6c1, 1 }, { 0x701, 1 }, { 0x741, 1 }, { 2, 2 },
	{ 0x82, 2 }, { 0xc2, 2 }, { 0x102, 2 }, { 0x142, 2 }, { 0x182, 2 }, { 0x1c2, 2 }, { 0x202, 2 }, { 0x242, 2 },
	{ 0x282, 2 }, { 0x2c2, 2 }, { 0x302, 2 }, { 0x342, 2 }, { 0x382, 2 }, { 0x3c2, 2 }, { 0x402, 2 },
	{ 0x442, 2 }, { 0x482, 2 }, { 0x4c2, 2 }, { 0x502, 2 }, { 0x542, 2 }, { 0x582, 2 }, { 0x5c2, 2 },
	{ 0x602, 2 }, { 0x642, 2 }, { 0x682, 2 }, { 0x6c2, 2 }, { 0x702, 2 }, { 0x742, 2 }, { 3, 3 }, { 0x83, 3 },
	{ 0xc3, 3 }, { 0x103, 3 }, { 0x143, 3 }, { 0x183, 3 }, { 0x1c3, 3 }, { 0x203, 3 }, { 0x243, 3 }, { 0x283, 3 },
	{ 0x2c3, 3 }, { 0x303, 3 }, { 0x343, 3 }, { 0x383, 3 }, { 0x3c3, 3 }, { 0x403, 3 }, { 0x443, 3 },
	{ 0x483, 3 }, { 0x4c3, 3 }, { 0x503, 3 }, { 0x543, 3 }, { 0x583, 3 }, { 0x5c3, 3 }, { 0x603, 3 },
	{ 0x643, 3 }, { 0x683, 3 }, { 0x6c3, 3 }, { 0x703, 3 }, { 0x743, 3 }, { 4, 4 }, { 0x84, 4 }, { 0xc4, 4 },
	{ 0x104, 4 }, { 0x144, 4 }, { 0x184, 4 }, { 0x1c4, 4 }, { 0x204, 4 }, { 0x244, 4 }, { 0x284, 4 },
	{ 0x2c4, 4 }, { 0x304, 4 }, { 0x344, 4 }, { 0x384, 4 }, { 0x3c4, 4 }, { 0x404, 4 }, { 0x444, 4 },
	{ 0x484, 4 }, { 0x4c4, 4 }, { 0x504, 4 }, { 0x544, 4 }, { 0x584, 4 }, { 0x5c4, 4 }, { 0x604, 4 },
	{ 0x644, 4 }, { 0x684, 4 }, { 0x6c4, 4 }, { 0x704, 4 }, { 0x744, 4 }, { 5, 5 }, { 0x85, 5 }, { 0xc5, 5 },
	{ 0x105, 5 }, { 0x145, 5 }, { 0x185, 5 }, { 0x1c5, 5 }, { 0x205, 5 }, { 0x245, 5 }, { 0x285, 5 },
	{ 0x2c5, 5 }, { 0x305, 5 }, { 0x345, 5 }, { 0x385, 5 }, { 0x3c5, 5 }, { 0x405, 5 }, { 0x445, 5 },
	{ 0x485, 5 }, { 0x4c5, 5 }, { 0x505, 5 }, { 0x545, 5 }, { 0x585, 5 }, { 0x5c5, 5 }, { 0x605, 5 },
	{ 0x645, 5 }, { 0x685, 5 }, { 0x6c5, 5 }, { 0x705, 5 }, { 0x745, 5 }, { 6, 6 }, { 0x86, 6 }, { 0xc6, 6 },
	{ 0x106, 6 }, { 0x146, 6 }, { 0x186, 6 }, { 0x1c6, 6 }, { 0x206, 6 }, { 0x246, 6 }, { 0x286, 6 },
	{ 0x2c6, 6 }, { 0x306, 6 }, { 0x346, 6 }, { 0x386, 6 }, { 0x3c6, 6 }, { 0x406, 6 }, { 0x446, 6 },
	{ 0x486, 6 }, { 0x4c6, 6 }, { 0x506, 6 }, { 0x546, 6 }, { 0x586, 6 }, { 0x5c6, 6 }, { 0x606, 6 },
	{ 0x646, 6 }, { 0x686, 6 }, { 0x6c6, 6 }, { 0x706, 6 }, { 0x746, 6 }, { 7, 7 }, { 0x87, 7 }, { 0xc7, 7 },
	{ 0x107, 7 }, { 0x147, 7 }, { 0x187, 7 }, { 0x1c7, 7 }, { 0x207, 7 }, { 0x247, 7 }, { 0x287, 7 },
	{ 0x2c7, 7 }, { 0x307, 7 }, { 0x347, 7 }, { 0x387, 7 }, { 0x3c7, 7 }, { 0x407, 7 }, { 0x447, 7 },
	{ 0x487, 7 }, { 0x4c7, 7 }, { 0x507, 7 }, { 0x547, 7 }, { 0x587, 7 }, { 0x5c7, 7 }, { 0x607, 7 },
	{ 0x647, 7 }, { 0x687, 7 }, { 0x6c7, 7 }, { 0x707, 7 }, { 0x747, 7 }, { 8, 0x48 }, { 0x88, 0x48 },
	{ 0xc8, 0x48 }, { 0x108, 0x48 }, { 0x148, 0x48 }, { 0x188, 0x48 }, { 0x1c8, 0x48 }, { 0x208, 0x48 },
	{ 0x248, 0x48 }, { 0x288, 0x48 }, { 0x2c8, 0x48 }, { 0x308, 0x48 }, { 0x348, 0x48 }, { 0x388, 0x48 },
	{ 0x3c8, 0x48 }, { 0x408, 0x48 }, { 0x448, 0x48 }, { 0x488, 0x48 }, { 0x4c8, 0x48 }, { 0x508, 0x48 },
	{ 0x548, 0x48 }, { 0x588, 0x48 }, { 0x5c8, 0x48 }, { 0x608, 0x48 }, { 0x648, 0x48 }, { 0x688, 0x48 },
	{ 0x6c8, 0x48 }, { 0x708, 0x48 }, { 0x748, 0x48 }, { 9, 0x49 }, { 0x89, 0x49 }, { 0xc9, 0x49 },
	{ 0x109, 0x49 }, { 0x149, 0x49 }, { 0x189, 0x49 }, { 0x1c9, 0x49 }, { 0x209, 0x49 }, { 0x249, 0x49 },
	{ 0x289, 0x49 }, { 0x2c9, 0x49 }, { 0x309, 0x49 }, { 0x349, 0x49 }, { 0x389, 0x49 }, { 0x3c9, 0x49 },
	{ 0x409, 0x49 }, { 0x449, 0x49 }, { 0x489, 0x49 }, { 0x4c9, 0x49 }, { 0x509, 0x49 }, { 0x549, 0x49 },
	{ 0x589, 0x49 }, { 0x5c9, 0x49 }, { 0x609, 0x49 }, { 0x649, 0x49 }, { 0x689, 0x49 }, { 0x6c9, 0x49 },
	{ 0x709, 0x49 }, { 0x749, 0x49 }, { 0xa, 0x4a }, { 0x8a, 0x4a }, { 0xca, 0x4a }, { 0x10a, 0x4a },
	{ 0x14a, 0x4a }, { 0x18a, 0x4a }, { 0x1ca, 0x4a }, { 0x20a, 0x4a }, { 0x24a, 0x4a }, { 0x28a, 0x4a },
	{ 0x2ca, 0x4a }, { 0x30a, 0x4a }, { 0x34a, 0x4a }, { 0x38a, 0x4a }, { 0x3ca, 0x4a }, { 0x40a, 0x4a },
	{ 0x44a, 0x4a }, { 0x48a, 0x4a }, { 0x4ca, 0x4a }, { 0x50a, 0x4a }, { 0x54a, 0x4a }, { 0x58a, 0x4a },
	{ 0x5ca, 0x4a }, { 0x60a, 0x4a }, { 0x64a, 0x4a }, { 0x68a, 0x4a }, { 0x6ca, 0x4a }, { 0x70a, 0x4a },
	{ 0x74a, 0x4a }, { 0xb, 0x4b }, { 0x8b, 0x4b }, { 0xcb, 0x4b }, { 0x10b, 0x4b }, { 0x14b, 0x4b },
	{ 0x18b, 0x4b }, { 0x1cb, 0x4b }, { 0x20b, 0x4b }, { 0x24b, 0x4b }, { 0x28b, 0x4b }, { 0x2cb, 0x4b },
	{ 0x30b, 0x4b }, { 0x34b, 0x4b }, { 0x38b, 0x4b }, { 0x3cb, 0x4b }, { 0x40b, 0x4b }, { 0x44b, 0x4b },
	{ 0x48b, 0x4b }, { 0x4cb, 0x4b }, { 0x50b, 0x4b }, { 0x54b, 0x4b }, { 0x58b, 0x4b }, { 0x5cb, 0x4b },
	{ 0x60b, 0x4b }, { 0x64b, 0x4b }, { 0x68b, 0x4b }, { 0x6cb, 0x4b }, { 0x70b, 0x4b }, { 0x74b, 0x4b },
	{ 0xc, 0x4c }, { 0x8c, 0x4c }, { 0xcc, 0x4c }, { 0x10c, 0x4c }, { 0x14c, 0x4c }, { 0x18c, 0x4c },
	{ 0x1cc, 0x4c }, { 0x20c, 0x4c }, { 0x24c, 0x4c }, { 0x28c, 0x4c }, { 0x2cc, 0x4c }, { 0x30c, 0x4c },
	{ 0x34c, 0x4c }, { 0x38c, 0x4c }, { 0x3cc, 0x4c }, { 0x40c, 0x4c }, { 0x44c, 0x4c }, { 0x48c, 0x4c },
	{ 0x4cc, 0x4c }, { 0x50c, 0x4c }, { 0x54c, 0x4c }, { 0x58c, 0x4c }, { 0x5cc, 0x4c }, { 0x60c, 0x4c },
	{ 0x64c, 0x4c }, { 0x68c, 0x4c }, { 0x6cc, 0x4c }, { 0x70c, 0x4c }, { 0x74c, 0x4c }, { 0xd, 0x4d },
	{ 0x8d, 0x4d }, { 0xcd, 0x4d }, { 0x10d, 0x4d }, { 0x14d, 0x4d }, { 0x18d, 0x4d }, { 0x1cd, 0x4d },
	{ 0x20d, 0x4d }, { 0x24d, 0x4d }, { 0x28d, 0x4d }, { 0x2cd, 0x4d }, { 0x30d, 0x4d }, { 0x34d, 0x4d },
	{ 0x38d, 0x4d }, { 0x3cd, 0x4d }, { 0x40d, 0x4d }, { 0x44d, 0x4d }, { 0x48d, 0x4d }, { 0x4cd, 0x4d },
	{ 0x50d, 0x4d }, { 0x54d, 0x4d }, { 0x58d, 0x4d }, { 0x5cd, 0x4d }, { 0x60d, 0x4d }, { 0x64d, 0x4d },
	{ 0x68d, 0x4d }, { 0x6cd, 0x4d }, { 0x70d, 0x4d }, { 0x74d, 0x4d }, { 0xe, 0x4e }, { 0x8e, 0x4e },
	{ 0xce, 0x4e }, { 0x10e, 0x4e }, { 0x14e, 0x4e }, { 0x18e, 0x4e }, { 0x1ce, 0x4e }, { 0x20e, 0x4e },
	{ 0x24e, 0x4e }, { 0x28e, 0x4e }, { 0x2ce, 0x4e }, { 0x30e, 0x4e }, { 0x34e, 0x4e }, { 0x38e, 0x4e },
	{ 0x3ce, 0x4e }, { 0x40e, 0x4e }, { 0x44e, 0x4e }, { 0x48e, 0x4e }, { 0x4ce, 0x4e }, { 0x50e, 0x4e },
	{ 0x54e, 0x4e }, { 0x58e, 0x4e }, { 0x5ce, 0x4e }, { 0x60e, 0x4e }, { 0x64e, 0x4e }, { 0x68e, 0x4e },
	{ 0x6ce, 0x4e }, { 0x70e, 0x4e }, { 0x74e, 0x4e }, { 0xf, 0x4f }, { 0x8f, 0x4f }, { 0xcf, 0x4f },
	{ 0x10f, 0x4f }, { 0x14f, 0x4f }, { 0x18f, 0x4f }, { 0x1cf, 0x4f }, { 0x20f, 0x4f }, { 0x24f, 0x4f },
	{ 0x28f, 0x4f }, { 0x2cf, 0x4f }, { 0x30f, 0x4f }, { 0x34f, 0x4f }, { 0x38f, 0x4f }, { 0x3cf, 0x4f },
	{ 0x40f, 0x4f }, { 0x44f, 0x4f }, { 0x48f, 0x4f }, { 0x4cf, 0x4f }, { 0x50f, 0x4f }, { 0x54f, 0x4f },
	{ 0x58f, 0x4f }, { 0x5cf, 0x4f }, { 0x60f, 0x4f }, { 0x64f, 0x4f }, { 0x68f, 0x4f }, { 0x6cf, 0x4f },
	{ 0x70f, 0x4f }, { 0x74f, 0x4f }, { 0x10, 0x90 }, { 0x90, 0x90 }, { 0xd0, 0x90 }, { 0x110, 0x90 },
	{ 0x150, 0x90 }, { 0x190, 0x90 }, { 0x1d0, 0x90 }, { 0x210, 0x90 }, { 0x250, 0x90 }, { 0x290, 0x90 },
	{ 0x2d0, 0x90 }, { 0x310, 0x90 }, { 0x350, 0x90 }, { 0x390, 0x90 }, { 0x3d0, 0x90 }, { 0x410, 0x90 },
	{ 0x450, 0x90 }, { 0x490, 0x90 }, { 0x4d0, 0x90 }, { 0x510, 0x90 }, { 0x550, 0x90 }, { 0x590, 0x90 },
	{ 0x5d0, 0x90 }, { 0x610, 0x90 }, { 0x650, 0x90 }, { 0x690, 0x90 }, { 0x6d0, 0x90 }, { 0x710, 0x90 },
	{ 0x750, 0x90 }, { 0x11, 0x91 }, { 0x91, 0x91 }, { 0xd1, 0x91 }, { 0x111, 0x91 }, { 0x151, 0x91 },
	{ 0x191, 0x91 }, { 0x1d1, 0x91 }, { 0x211, 0x91 }, { 0x251, 0x91 }, { 0x291, 0x91 }, { 0x2d1, 0x91 },
	{ 0x311, 0x91 }, { 0x351, 0x91 }, { 0x391, 0x91 }, { 0x3d1, 0x91 }, { 0x411, 0x91 }, { 0x451, 0x91 },
	{ 0x491, 0x91 }, { 0x4d1, 0x91 }, { 0x511, 0x91 }, { 0x551, 0x91 }, { 0x591, 0x91 }, { 0x5d1, 0x91 },
	{ 0x611, 0x91 }, { 0x651, 0x91 }, { 0x691, 0x91 }, { 0x6d1, 0x91 }, { 0x711, 0x91 }, { 0x751, 0x91 },
	{ 0x12, 0x92 }, { 0x92, 0x92 }, { 0xd2, 0x92 }, { 0x112, 0x92 }, { 0x152, 0x92 }, { 0x192, 0x92 },
	{ 0x1d2, 0x92 }, { 0x212, 0x92 }, { 0x252, 0x92 }, { 0x292, 0x92 }, { 0x2d2, 0x92 }, { 0x312, 0x92 },
	{ 0x352, 0x92 }, { 0x392, 0x92 }, { 0x3d2, 0x92 }, { 0x412, 0x92 }, { 0x452, 0x92 }, { 0x492, 0x92 },
	{ 0x4d2, 0x92 }, { 0x512, 0x92 }, { 0x552, 0x92 }, { 0x592, 0x92 }, { 0x5d2, 0x92 }, { 0x612, 0x92 },
	{ 0x652, 0x92 }, { 0x692, 0x92 }, { 0x6d2, 0x92 }, { 0x712, 0x92 }, { 0x752, 0x92 }, { 0x13, 0x93 },
	{ 0x93, 0x93 }, { 0xd3, 0x93 }, { 0x113, 0x93 }, { 0x153, 0x93 }, { 0x193, 0x93 }, { 0x1d3, 0x93 },
	{ 0x213, 0x93 }, { 0x253, 0x93 }, { 0x293, 0x93 }, { 0x2d3, 0x93 }, { 0x313, 0x93 }, { 0x353, 0x93 },
	{ 0x393, 0x93 }, { 0x3d3, 0x93 }, { 0x413, 0x93 }, { 0x453, 0x93 }, { 0x493, 0x93 }, { 0x4d3, 0x93 },
	{ 0x513, 0x93 }, { 0x553, 0x93 }, { 0x593, 0x93 }, { 0x5d3, 0x93 }, { 0x613, 0x93 }, { 0x653, 0x93 },
	{ 0x693, 0x93 }, { 0x6d3, 0x93 }, { 0x713, 0x93 }, { 0x753, 0x93 }, { 0x14, 0x94 }, { 0x94, 0x94 },
	{ 0xd4, 0x94 }, { 0x114, 0x94 }, { 0x154, 0x94 }, { 0x194, 0x94 }, { 0x1d4, 0x94 }, { 0x214, 0x94 },
	{ 0x254, 0x94 }, { 0x294, 0x94 }, { 0x2d4, 0x94 }, { 0x314, 0x94 }, { 0x354, 0x94 }, { 0x394, 0x94 },
	{ 0x3d4, 0x94 }, { 0x414, 0x94 }, { 0x454, 0x94 }, { 0x494, 0x94 }, { 0x4d4, 0x94 }, { 0x514, 0x94 },
	{ 0x554, 0x94 }, { 0x594, 0x94 }, { 0x5d4, 0x94 }, { 0x614, 0x94 }, { 0x654, 0x94 }, { 0x694, 0x94 },
	{ 0x6d4, 0x94 }, { 0x714, 0x94 }, { 0x754, 0x94 }, { 0x15, 0x95 }, { 0x95, 0x95 }, { 0xd5, 0x95 },
	{ 0x115, 0x95 }, { 0x155, 0x95 }, { 0x195, 0x95 }, { 0x1d5, 0x95 }, { 0x215, 0x95 }, { 0x255, 0x95 },
	{ 0x295, 0x95 }, { 0x2d5, 0x95 }, { 0x315, 0x95 }, { 0x355, 0x95 }, { 0x395, 0x95 }, { 0x3d5, 0x95 },
	{ 0x415, 0x95 }, { 0x455, 0x95 }, { 0x495, 0x95 }, { 0x4d5, 0x95 }, { 0x515, 0x95 }, { 0x555, 0x95 },
	{ 0x595, 0x95 }, { 0x5d5, 0x95 }, { 0x615, 0x95 }, { 0x655, 0x95 }, { 0x695, 0x95 }, { 0x6d5, 0x95 },
	{ 0x715, 0x95 }, { 0x755, 0x95 }, { 0x16, 0x96 }, { 0x96, 0x96 }, { 0xd6, 0x96 }, { 0x116, 0x96 },
	{ 0x156, 0x96 }, { 0x196, 0x96 }, { 0x1d6, 0x96 }, { 0x216, 0x96 }, { 0x256, 0x96 }, { 0x296, 0x96 },
	{ 0x2d6, 0x96 }, { 0x316, 0x96 }, { 0x356, 0x96 }, { 0x396, 0x96 }, { 0x3d6, 0x96 }, { 0x416, 0x96 },
	{ 0x456, 0x96 }, { 0x496, 0x96 }, { 0x4d6, 0x96 }, { 0x516, 0x96 }, { 0x556, 0x96 }, { 0x596, 0x96 },
	{ 0x5d6, 0x96 }, { 0x616, 0x96 }, { 0x656, 0x96 }, { 0x696, 0x96 }, { 0x6d6, 0x96 }, { 0x716, 0x96 },
	{ 0x756, 0x96 }, { 0x17, 0x97 }, { 0x97, 0x97 }, { 0xd7, 0x97 }, { 0x117, 0x97 }, { 0x157, 0x97 },
	{ 0x197, 0x97 }, { 0x1d7, 0x97 }, { 0x217, 0x97 }, { 0x257, 0x97 }, { 0x297, 0x97 }, { 0x2d7, 0x97 },
	{ 0x317, 0x97 }, { 0x357, 0x97 }, { 0x397, 0x97 }, { 0x3d7, 0x97 }, { 0x417, 0x97 }, { 0x457, 0x97 },
	{ 0x497, 0x97 }, { 0x4d7, 0x97 }, { 0x517, 0x97 }, { 0x557, 0x97 }, { 0x597, 0x97 }, { 0x5d7, 0x97 },
	{ 0x617, 0x97 }, { 0x657, 0x97 }, { 0x697, 0x97 }, { 0x6d7, 0x97 }, { 0x717, 0x97 }, { 0x757, 0x97 },
	{ 0x18, 0xd8 }, { 0x98, 0xd8 }, { 0xd8, 0xd8 }, { 0x118, 0xd8 }, { 0x158, 0xd8 }, { 0x198, 0xd8 },
	{ 0x1d8, 0xd8 }, { 0x218, 0xd8 }, { 0x258, 0xd8 }, { 0x298, 0xd8 }, { 0x2d8, 0xd8 }, { 0x318, 0xd8 },
	{ 0x358, 0xd8 }, { 0x398, 0xd8 }, { 0x3d8, 0xd8 }, { 0x418, 0xd8 }, { 0x458, 0xd8 }, { 0x498, 0xd8 },
	{ 0x4d8, 0xd8 }, { 0x518, 0xd8 }, { 0x558, 0xd8 }, { 0x598, 0xd8 }, { 0x5d8, 0xd8 }, { 0x618, 0xd8 },
	{ 0x658, 0xd8 }, { 0x698, 0xd8 }, { 0x6d8, 0xd8 }, { 0x718, 0xd8 }, { 0x758, 0xd8 }, { 0x19, 0xd9 },
	{ 0x99, 0xd9 }, { 0xd9, 0xd9 }, { 0x119, 0xd9 }, { 0x159, 0xd9 }, { 0x199, 0xd9 }, { 0x1d9, 0xd9 },
	{ 0x219, 0xd9 }, { 0x259, 0xd9 }, { 0x299, 0xd9 }, { 0x2d9, 0xd9 }, { 0x319, 0xd9 }, { 0x359, 0xd9 },
	{ 0x399, 0xd9 }, { 0x3d9, 0xd9 }, { 0x419, 0xd9 }, { 0x459, 0xd9 }, { 0x499, 0xd9 }, { 0x4d9, 0xd9 },
	{ 0x519, 0xd9 }, { 0x559, 0xd9 }, { 0x599, 0xd9 }, { 0x5d9, 0xd9 }, { 0x619, 0xd9 }, { 0x659, 0xd9 },
	{ 0x699, 0xd9 }, { 0x6d9, 0xd9 }, { 0x719, 0xd9 }, { 0x759, 0xd9 }, { 0x1a, 0xda }, { 0x9a, 0xda },
	{ 0xda, 0xda }, { 0x11a, 0xda }, { 0x15a, 0xda }, { 0x19a, 0xda }, { 0x1da, 0xda }, { 0x21a, 0xda },
	{ 0x25a, 0xda }, { 0x29a, 0xda }, { 0x2da, 0xda }, { 0x31a, 0xda }, { 0x35a, 0xda }, { 0x39a, 0xda },
	{ 0x3da, 0xda }, { 0x41a, 0xda }, { 0x45a, 0xda }, { 0x49a, 0xda }, { 0x4da, 0xda }, { 0x51a, 0xda },
	{ 0x55a, 0xda }, { 0x59a, 0xda }, { 0x5da, 0xda }, { 0x61a, 0xda }, { 0x65a, 0xda }, { 0x69a, 0xda },
	{ 0x6da, 0xda }, { 0x71a, 0xda }, { 0x75a, 0xda }, { 0x1b, 0xdb }, { 0x9b, 0xdb }, { 0xdb, 0xdb },
	{ 0x11b, 0xdb }, { 0x15b, 0xdb }, { 0x19b, 0xdb }, { 0x1db, 0xdb }, { 0x21b, 0xdb }, { 0x25b, 0xdb },
	{ 0x29b, 0xdb }, { 0x2db, 0xdb }, { 0x31b, 0xdb }, { 0x35b, 0xdb }, { 0x39b, 0xdb }, { 0x3db, 0xdb },
	{ 0x41b, 0xdb }, { 0x45b, 0xdb }, { 0x49b, 0xdb }, { 0x4db, 0xdb }, { 0x51b, 0xdb }, { 0x55b, 0xdb },
	{ 0x59b, 0xdb }, { 0x5db, 0xdb }, { 0x61b, 0xdb }, { 0x65b, 0xdb }, { 0x69b, 0xdb }, { 0x6db, 0xdb },
	{ 0x71b, 0xdb }, { 0x75b, 0xdb }, { 0x1c, 0xdc }, { 0x9c, 0xdc }, { 0xdc, 0xdc }, { 0x11c, 0xdc },
	{ 0x15c, 0xdc }, { 0x19c, 0xdc }, { 0x1dc, 0xdc }, { 0x21c, 0xdc }, { 0x25c, 0xdc }, { 0x29c, 0xdc },
	{ 0x2dc, 0xdc }, { 0x31c, 0xdc }, { 0x35c, 0xdc }, { 0x39c, 0xdc }, { 0x3dc, 0xdc }, { 0x41c, 0xdc },
	{ 0x45c, 0xdc }, { 0x49c, 0xdc }, { 0x4dc, 0xdc }, { 0x51c, 0xdc }, { 0x55c, 0xdc }, { 0x59c, 0xdc },
	{ 0x5dc, 0xdc }, { 0x61c, 0xdc }, { 0x65c, 0xdc }, { 0x69c, 0xdc }, { 0x6dc, 0xdc }, { 0x71c, 0xdc },
	{ 0x75c, 0xdc }, { 0x1d, 0xdd }, { 0x9d, 0xdd }, { 0xdd, 0xdd }, { 0x11d, 0xdd }, { 0x15d, 0xdd },
	{ 0x19d, 0xdd }, { 0x1dd, 0xdd }, { 0x21d, 0xdd }, { 0x25d, 0xdd }, { 0x29d, 0xdd }, { 0x2dd, 0xdd },
	{ 0x31d, 0xdd }, { 0x35d, 0xdd }, { 0x39d, 0xdd }, { 0x3dd, 0xdd }, { 0x41d, 0xdd }, { 0x45d, 0xdd },
	{ 0x49d, 0xdd }, { 0x4dd, 0xdd }, { 0x51d, 0xdd }, { 0x55d, 0xdd }, { 0x59d, 0xdd }, { 0x5dd, 0xdd },
	{ 0x61d, 0xdd }, { 0x65d, 0xdd }, { 0x69d, 0xdd }, { 0x6dd, 0xdd }, { 0x71d, 0xdd }, { 0x75d, 0xdd },
	{ 0x1e, 0xde }, { 0x9e, 0xde }, { 0xde, 0xde }, { 0x11e, 0xde }, { 0x15e, 0xde }, { 0x19e, 0xde },
	{ 0x1de, 0xde }, { 0x21e, 0xde }, { 0x25e, 0xde }, { 0x29e, 0xde }, { 0x2de, 0xde }, { 0x31e, 0xde },
	{ 0x35e, 0xde }, { 0x39e, 0xde }, { 0x3de, 0xde }, { 0x41e, 0xde }, { 0x45e, 0xde }, { 0x49e, 0xde },
	{ 0x4de, 0xde }, { 0x51e, 0xde }, { 0x55e, 0xde }, { 0x59e, 0xde }, { 0x5de, 0xde }, { 0x61e, 0xde },
	{ 0x65e, 0xde }, { 0x69e, 0xde }, { 0x6de, 0xde }, { 0x71e, 0xde }, { 0x75e, 0xde }, { 0x1f, 0xdf },
	{ 0x9f, 0xdf }, { 0xdf, 0xdf }, { 0x11f, 0xdf }, { 0x15f, 0xdf }, { 0x19f, 0xdf }, { 0x1df, 0xdf },
	{ 0x21f, 0xdf }, { 0x25f, 0xdf }, { 0x29f, 0xdf }, { 0x2df, 0xdf }, { 0x31f, 0xdf }, { 0x35f, 0xdf },
	{ 0x39f, 0xdf }, { 0x3df, 0xdf }, { 0x41f, 0xdf }, { 0x45f, 0xdf }, { 0x49f, 0xdf }, { 0x4df, 0xdf },
	{ 0x51f, 0xdf }, { 0x55f, 0xdf }, { 0x59f, 0xdf }, { 0x5df, 0xdf }, { 0x61f, 0xdf }, { 0x65f, 0xdf },
	{ 0x69f, 0xdf }, { 0x6df, 0xdf }, { 0x71f, 0xdf }, { 0x75f, 0xdf }, { 0x20, 0x120 }, { 0xa0, 0x120 },
	{ 0xe0, 0x120 }, { 0x120, 0x120 }, { 0x160, 0x120 }, { 0x1a0, 0x120 }, { 0x1e0, 0x120 }, { 0x220, 0x120 },
	{ 0x260, 0x120 }, { 0x2a0, 0x120 }, { 0x2e0, 0x120 }, { 0x320, 0x120 }, { 0x360, 0x120 }, { 0x3a0, 0x120 },
	{ 0x3e0, 0x120 }, { 0x420, 0x120 }, { 0x460, 0x120 }, { 0x4a0, 0x120 }, { 0x4e0, 0x120 }, { 0x520, 0x120 },
	{ 0x560, 0x120 }, { 0x5a0, 0x120 }, { 0x5e0, 0x120 }, { 0x620, 0x120 }, { 0x660, 0x120 }, { 0x6a0, 0x120 },
	{ 0x6e0, 0x120 }, { 0x720, 0x120 }, { 0x760, 0x120 }, { 0x21, 0x121 }, { 0xa1, 0x121 }, { 0xe1, 0x121 },
	{ 0x121, 0x121 }, { 0x161, 0x121 }, { 0x1a1, 0x121 }, { 0x1e1, 0x121 }, { 0x221, 0x121 }, { 0x261, 0x121 },
	{ 0x2a1, 0x121 }, { 0x2e1, 0x121 }, { 0x321, 0x121 }, { 0x361, 0x121 }, { 0x3a1, 0x121 }, { 0x3e1, 0x121 },
	{ 0x421, 0x121 }, { 0x461, 0x121 }, { 0x4a1, 0x121 }, { 0x4e1, 0x121 }, { 0x521, 0x121 }, { 0x561, 0x121 },
	{ 0x5a1, 0x121 }, { 0x5e1, 0x121 }, { 0x621, 0x121 }, { 0x661, 0x121 }, { 0x6a1, 0x121 }, { 0x6e1, 0x121 },
	{ 0x721, 0x121 }, { 0x761, 0x121 }, { 0x22, 0x122 }, { 0xa2, 0x122 }, { 0xe2, 0x122 }, { 0x122, 0x122 },
	{ 0x162, 0x122 }, { 0x1a2, 0x122 }, { 0x1e2, 0x122 }, { 0x222, 0x122 }, { 0x262, 0x122 }, { 0x2a2, 0x122 },
	{ 0x2e2, 0x122 }, { 0x322, 0x122 }, { 0x362, 0x122 }, { 0x3a2, 0x122 }, { 0x3e2, 0x122 }, { 0x422, 0x122 },
	{ 0x462, 0x122 }, { 0x4a2, 0x122 }, { 0x4e2, 0x122 }, { 0x522, 0x122 }, { 0x562, 0x122 }, { 0x5a2, 0x122 },
	{ 0x5e2, 0x122 }, { 0x622, 0x122 }, { 0x662, 0x122 }, { 0x6a2, 0x122 }, { 0x6e2, 0x122 }, { 0x722, 0x122 },
	{ 0x762, 0x122 }, { 0x23, 0x123 }, { 0xa3, 0x123 }, { 0xe3, 0x123 }, { 0x123, 0x123 }, { 0x163, 0x123 },
	{ 0x1a3, 0x123 }, { 0x1e3, 0x123 }, { 0x223, 0x123 }, { 0x263, 0x123 }, { 0x2a3, 0x123 }, { 0x2e3, 0x123 },
	{ 0x323, 0x123 }, { 0x363, 0x123 }, { 0x3a3, 0x123 }, { 0x3e3, 0x123 }, { 0x423, 0x123 }, { 0x463, 0x123 },
	{ 0x4a3, 0x123 }, { 0x4e3, 0x123 }, { 0x523, 0x123 }, { 0x563, 0x123 }, { 0x5a3, 0x123 }, { 0x5e3, 0x123 },
	{ 0x623, 0x123 }, { 0x663, 0x123 }, { 0x6a3, 0x123 }, { 0x6e3, 0x123 }, { 0x723, 0x123 }, { 0x763, 0x123 },
	{ 0x24, 0x124 }, { 0xa4, 0x124 }, { 0xe4, 0x124 }, { 0x124, 0x124 }, { 0x164, 0x124 }, { 0x1a4, 0x124 },
	{ 0x1e4, 0x124 }, { 0x224, 0x124 }, { 0x264, 0x124 }, { 0x2a4, 0x124 }, { 0x2e4, 0x124 }, { 0x324, 0x124 },
	{ 0x364, 0x124 }, { 0x3a4, 0x124 }, { 0x3e4, 0x124 }, { 0x424, 0x124 }, { 0x464, 0x124 }, { 0x4a4, 0x124 },
	{ 0x4e4, 0x124 }, { 0x524, 0x124 }, { 0x564, 0x124 }, { 0x5a4, 0x124 }, { 0x5e4, 0x124 }, { 0x624, 0x124 },
	{ 0x664, 0x124 }, { 0x6a4, 0x124 }, { 0x6e4, 0x124 }, { 0x724, 0x124 }, { 0x764, 0x124 }, { 0x25, 0x125 },
	{ 0xa5, 0x125 }, { 0xe5, 0x125 }, { 0x125, 0x125 }, { 0x165, 0x125 }, { 0x1a5, 0x125 }, { 0x1e5, 0x125 },
	{ 0x225, 0x125 }, { 0x265, 0x125 }, { 0x2a5, 0x125 }, { 0x2e5, 0x125 }, { 0x325, 0x125 }, { 0x365, 0x125 },
	{ 0x3a5, 0x125 }, { 0x3e5, 0x125 }, { 0x425, 0x125 }, { 0x465, 0x125 }, { 0x4a5, 0x125 }, { 0x4e5, 0x125 },
	{ 0x525, 0x125 }, { 0x565, 0x125 }, { 0x5a5, 0x125 }, { 0x5e5, 0x125 }, { 0x625, 0x125 }, { 0x665, 0x125 },
	{ 0x6a5, 0x125 }, { 0x6e5, 0x125 }, { 0x725, 0x125 }, { 0x765, 0x125 }, { 0x26, 0x126 }, { 0xa6, 0x126 },
	{ 0xe6, 0x126 }, { 0x126, 0x126 }, { 0x166, 0x126 }, { 0x1a6, 0x126 }, { 0x1e6, 0x126 }, { 0x226, 0x126 },
	{ 0x266, 0x126 }, { 0x2a6, 0x126 }, { 0x2e6, 0x126 }, { 0x326, 0x126 }, { 0x366, 0x126 }, { 0x3a6, 0x126 },
	{ 0x3e6, 0x126 }, { 0x426, 0x126 }, { 0x466, 0x126 }, { 0x4a6, 0x126 }, { 0x4e6, 0x126 }, { 0x526, 0x126 },
	{ 0x566, 0x126 }, { 0x5a6, 0x126 }, { 0x5e6, 0x126 }, { 0x626, 0x126 }, { 0x666, 0x126 }, { 0x6a6, 0x126 },
	{ 0x6e6, 0x126 }, { 0x726, 0x126 }, { 0x766, 0x126 }, { 0x27, 0x127 }, { 0xa7, 0x127 }, { 0xe7, 0x127 },
	{ 0x127, 0x127 }, { 0x167, 0x127 }, { 0x1a7, 0x127 }, { 0x1e7, 0x127 }, { 0x227, 0x127 }, { 0x267, 0x127 },
	{ 0x2a7, 0x127 }, { 0x2e7, 0x127 }, { 0x327, 0x127 }, { 0x367, 0x127 }, { 0x3a7, 0x127 }, { 0x3e7, 0x127 },
	{ 0x427, 0x127 }, { 0x467, 0x127 }, { 0x4a7, 0x127 }, { 0x4e7, 0x127 }, { 0x527, 0x127 }, { 0x567, 0x127 },
	{ 0x5a7, 0x127 }, { 0x5e7, 0x127 }, { 0x627, 0x127 }, { 0x667, 0x127 }, { 0x6a7, 0x127 }, { 0x6e7, 0x127 },
	{ 0x727, 0x127 }, { 0x767, 0x127 }, { 0x28, 0x168 }, { 0xa8, 0x168 }, { 0xe8, 0x168 }, { 0x128, 0x168 },
	{ 0x168, 0x168 }, { 0x1a8, 0x168 }, { 0x1e8, 0x168 }, { 0x228, 0x168 }, { 0x268, 0x168 }, { 0x2a8, 0x168 },
	{ 0x2e8, 0x168 }, { 0x328, 0x168 }, { 0x368, 0x168 }, { 0x3a8, 0x168 }, { 0x3e8, 0x168 }, { 0x428, 0x168 },
	{ 0x468, 0x168 }, { 0x4a8, 0x168 }, { 0x4e8, 0x168 }, { 0x528, 0x168 }, { 0x568, 0x168 }, { 0x5a8, 0x168 },
	{ 0x5e8, 0x168 }, { 0x628, 0x168 }, { 0x668, 0x168 }, { 0x6a8, 0x168 }, { 0x6e8, 0x168 }, { 0x728, 0x168 },
	{ 0x768, 0x168 }, { 0x29, 0x169 }, { 0xa9, 0x169 }, { 0xe9, 0x169 }, { 0x129, 0x169 }, { 0x169, 0x169 },
	{ 0x1a9, 0x169 }, { 0x1e9, 0x169 }, { 0x229, 0x169 }, { 0x269, 0x169 }, { 0x2a9, 0x169 }, { 0x2e9, 0x169 },
	{ 0x329, 0x169 }, { 0x369, 0x169 }, { 0x3a9, 0x169 }, { 0x3e9, 0x169 }, { 0x429, 0x169 }, { 0x469, 0x169 },
	{ 0x4a9, 0x169 }, { 0x4e9, 0x169 }, { 0x529, 0x169 }, { 0x569, 0x169 }, { 0x5a9, 0x169 }, { 0x5e9, 0x169 },
	{ 0x629, 0x169 }, { 0x669, 0x169 }, { 0x6a9, 0x169 }, { 0x6e9, 0x169 }, { 0x729, 0x169 }, { 0x769, 0x169 },
	{ 0x2a, 0x16a }, { 0xaa, 0x16a }, { 0xea, 0x16a }, { 0x12a, 0x16a }, { 0x16a, 0x16a }, { 0x1aa, 0x16a },
	{ 0x1ea, 0x16a }, { 0x22a, 0x16a }, { 0x26a, 0x16a }, { 0x2aa, 0x16a }, { 0x2ea, 0x16a }, { 0x32a, 0x16a },
	{ 0x36a, 0x16a }, { 0x3aa, 0x16a }, { 0x3ea, 0x16a }, { 0x42a, 0x16a }, { 0x46a, 0x16a }, { 0x4aa, 0x16a },
	{ 0x4ea, 0x16a }, { 0x52a, 0x16a }, { 0x56a, 0x16a }, { 0x5aa, 0x16a }, { 0x5ea, 0x16a }, { 0x62a, 0x16a },
	{ 0x66a, 0x16a }, { 0x6aa, 0x16a }, { 0x6ea, 0x16a }, { 0x72a, 0x16a }, { 0x76a, 0x16a }, { 0x2b, 0x16b },
	{ 0xab, 0x16b }, { 0xeb, 0x16b }, { 0x12b, 0x16b }, { 0x16b, 0x16b }, { 0x1ab, 0x16b }, { 0x1eb, 0x16b },
	{ 0x22b, 0x16b }, { 0x26b, 0x16b }, { 0x2ab, 0x16b }, { 0x2eb, 0x16b }, { 0x32b, 0x16b }, { 0x36b, 0x16b },
	{ 0x3ab, 0x16b }, { 0x3eb, 0x16b }, { 0x42b, 0x16b }, { 0x46b, 0x16b }, { 0x4ab, 0x16b }, { 0x4eb, 0x16b },
	{ 0x52b, 0x16b }, { 0x56b, 0x16b }, { 0x5ab, 0x16b }, { 0x5eb, 0x16b }, { 0x62b, 0x16b }, { 0x66b, 0x16b },
	{ 0x6ab, 0x16b }, { 0x6eb, 0x16b }, { 0x72b, 0x16b }, { 0x76b, 0x16b }, { 0x2c, 0x16c }, { 0xac, 0x16c },
	{ 0xec, 0x16c }, { 0x12c, 0x16c }, { 0x16c, 0x16c }, { 0x1ac, 0x16c }, { 0x1ec, 0x16c }, { 0x22c, 0x16c },
	{ 0x26c, 0x16c }, { 0x2ac, 0x16c }, { 0x2ec, 0x16c }, { 0x32c, 0x16c }, { 0x36c, 0x16c }, { 0x3ac, 0x16c },
	{ 0x3ec, 0x16c }, { 0x42c, 0x16c }, { 0x46c, 0x16c }, { 0x4ac, 0x16c }, { 0x4ec, 0x16c }, { 0x52c, 0x16c },
	{ 0x56c, 0x16c }, { 0x5ac, 0x16c }, { 0x5ec, 0x16c }, { 0x62c, 0x16c }, { 0x66c, 0x16c }, { 0x6ac, 0x16c },
	{ 0x6ec, 0x16c }, { 0x72c, 0x16c }, { 0x76c, 0x16c }, { 0x2d, 0x16d }, { 0xad, 0x16d }, { 0xed, 0x16d },
	{ 0x12d, 0x16d }, { 0x16d, 0x16d }, { 0x1ad, 0x16d }, { 0x1ed, 0x16d }, { 0x22d, 0x16d }, { 0x26d, 0x16d },
	{ 0x2ad, 0x16d }, { 0x2ed, 0x16d }, { 0x32d, 0x16d }, { 0x36d, 0x16d }, { 0x3ad, 0x16d }, { 0x3ed, 0x16d },
	{ 0x42d, 0x16d }, { 0x46d, 0x16d }, { 0x4ad, 0x16d }, { 0x4ed, 0x16d }, { 0x52d, 0x16d }, { 0x56d, 0x16d },
	{ 0x5ad, 0x16d }, { 0x5ed, 0x16d }, { 0x62d, 0x16d }, { 0x66d, 0x16d }, { 0x6ad, 0x16d }, { 0x6ed, 0x16d },
	{ 0x72d, 0x16d }, { 0x76d, 0x16d }, { 0x2e, 0x16e }, { 0xae, 0x16e }, { 0xee, 0x16e }, { 0x12e, 0x16e },
	{ 0x16e, 0x16e }, { 0x1ae, 0x16e }, { 0x1ee, 0x16e }, { 0x22e, 0x16e }, { 0x26e, 0x16e }, { 0x2ae, 0x16e },
	{ 0x2ee, 0x16e }, { 0x32e, 0x16e }, { 0x36e, 0x16e }, { 0x3ae, 0x16e }, { 0x3ee, 0x16e }, { 0x42e, 0x16e },
	{ 0x46e, 0x16e }, { 0x4ae, 0x16e }, { 0x4ee, 0x16e }, { 0x52e, 0x16e }, { 0x56e, 0x16e }, { 0x5ae, 0x16e },
	{ 0x5ee, 0x16e }, { 0x62e, 0x16e }, { 0x66e, 0x16e }, { 0x6ae, 0x16e }, { 0x6ee, 0x16e }, { 0x72e, 0x16e },
	{ 0x76e, 0x16e }, { 0x2f, 0x16f }, { 0xaf, 0x16f }, { 0xef, 0x16f }, { 0x12f, 0x16f }, { 0x16f, 0x16f },
	{ 0x1af, 0x16f }, { 0x1ef, 0x16f }, { 0x22f, 0x16f }, { 0x26f, 0x16f }, { 0x2af, 0x16f }, { 0x2ef, 0x16f },
	{ 0x32f, 0x16f }, { 0x36f, 0x16f }, { 0x3af, 0x16f }, { 0x3ef, 0x16f }, { 0x42f, 0x16f }, { 0x46f, 0x16f },
	{ 0x4af, 0x16f }, { 0x4ef, 0x16f }, { 0x52f, 0x16f }, { 0x56f, 0x16f }, { 0x5af, 0x16f }, { 0x5ef, 0x16f },
	{ 0x62f, 0x16f }, { 0x66f, 0x16f }, { 0x6af, 0x16f }, { 0x6ef, 0x16f }, { 0x72f, 0x16f }, { 0x76f, 0x16f },
	{ 0x30, 0x1b0 }, { 0xb0, 0x1b0 }, { 0xf0, 0x1b0 }, { 0x130, 0x1b0 }, { 0x170, 0x1b0 }, { 0x1b0, 0x1b0 },
	{ 0x1f0, 0x1b0 }, { 0x230, 0x1b0 }, { 0x270, 0x1b0 }, { 0x2b0, 0x1b0 }, { 0x2f0, 0x1b0 }, { 0x330, 0x1b0 },
	{ 0x370, 0x1b0 }, { 0x3b0, 0x1b0 }, { 0x3f0, 0x1b0 }, { 0x430, 0x1b0 }, { 0x470, 0x1b0 }, { 0x4b0, 0x1b0 },
	{ 0x4f0, 0x1b0 }, { 0x530, 0x1b0 }, { 0x570, 0x1b0 }, { 0x5b0, 0x1b0 }, { 0x5f0, 0x1b0 }, { 0x630, 0x1b0 },
	{ 0x670, 0x1b0 }, { 0x6b0, 0x1b0 }, { 0x6f0, 0x1b0 }, { 0x730, 0x1b0 }, { 0x770, 0x1b0 }, { 0x31, 0x1b1 },
	{ 0xb1, 0x1b1 }, { 0xf1, 0x1b1 }, { 0x131, 0x1b1 }, { 0x171, 0x1b1 }, { 0x1b1, 0x1b1 }, { 0x1f1, 0x1b1 },
	{ 0x231, 0x1b1 }, { 0x271, 0x1b1 }, { 0x2b1, 0x1b1 }, { 0x2f1, 0x1b1 }, { 0x331, 0x1b1 }, { 0x371, 0x1b1 },
	{ 0x3b1, 0x1b1 }, { 0x3f1, 0x1b1 }, { 0x431, 0x1b1 }, { 0x471, 0x1b1 }, { 0x4b1, 0x1b1 }, { 0x4f1, 0x1b1 },
	{ 0x531, 0x1b1 }, { 0x571, 0x1b1 }, { 0x5b1, 0x1b1 }, { 0x5f1, 0x1b1 }, { 0x631, 0x1b1 }, { 0x671, 0x1b1 },
	{ 0x6b1, 0x1b1 }, { 0x6f1, 0x1b1 }, { 0x731, 0x1b1 }, { 0x771, 0x1b1 }, { 0x32, 0x1b2 }, { 0xb2, 0x1b2 },
	{ 0xf2, 0x1b2 }, { 0x132, 0x1b2 }, { 0x172, 0x1b2 }, { 0x1b2, 0x1b2 }, { 0x1f2, 0x1b2 }, { 0x232, 0x1b2 },
	{ 0x272, 0x1b2 }, { 0x2b2, 0x1b2 }, { 0x2f2, 0x1b2 }, { 0x332, 0x1b2 }, { 0x372, 0x1b2 }, { 0x3b2, 0x1b2 },
	{ 0x3f2, 0x1b2 }, { 0x432, 0x1b2 }, { 0x472, 0x1b2 }, { 0x4b2, 0x1b2 }, { 0x4f2, 0x1b2 }, { 0x532, 0x1b2 },
	{ 0x572, 0x1b2 }, { 0x5b2, 0x1b2 }, { 0x5f2, 0x1b2 }, { 0x632, 0x1b2 }, { 0x672, 0x1b2 }, { 0x6b2, 0x1b2 },
	{ 0x6f2, 0x1b2 }, { 0x732, 0x1b2 }, { 0x772, 0x1b2 }, { 0x33, 0x1b3 }, { 0xb3, 0x1b3 }, { 0xf3, 0x1b3 },
	{ 0x133, 0x1b3 }, { 0x173, 0x1b3 }, { 0x1b3, 0x1b3 }, { 0x1f3, 0x1b3 }, { 0x233, 0x1b3 }, { 0x273, 0x1b3 },
	{ 0x2b3, 0x1b3 }, { 0x2f3, 0x1b3 }, { 0x333, 0x1b3 }, { 0x373, 0x1b3 }, { 0x3b3, 0x1b3 }, { 0x3f3, 0x1b3 },
	{ 0x433, 0x1b3 }, { 0x473, 0x1b3 }, { 0x4b3, 0x1b3 }, { 0x4f3, 0x1b3 }, { 0x533, 0x1b3 }, { 0x573, 0x1b3 },
	{ 0x5b3, 0x1b3 }, { 0x5f3, 0x1b3 }, { 0x633, 0x1b3 }, { 0x673, 0x1b3 }, { 0x6b3, 0x1b3 }, { 0x6f3, 0x1b3 },
	{ 0x733, 0x1b3 }, { 0x773, 0x1b3 }, { 0x34, 0x1b4 }, { 0xb4, 0x1b4 }, { 0xf4, 0x1b4 }, { 0x134, 0x1b4 },
	{ 0x174, 0x1b4 }, { 0x1b4, 0x1b4 }, { 0x1f4, 0x1b4 }, { 0x234, 0x1b4 }, { 0x274, 0x1b4 }, { 0x2b4, 0x1b4 },
	{ 0x2f4, 0x1b4 }, { 0x334, 0x1b4 }, { 0x374, 0x1b4 }, { 0x3b4, 0x1b4 }, { 0x3f4, 0x1b4 }, { 0x434, 0x1b4 },
	{ 0x474, 0x1b4 }, { 0x4b4, 0x1b4 }, { 0x4f4, 0x1b4 }, { 0x534, 0x1b4 }, { 0x574, 0x1b4 }, { 0x5b4, 0x1b4 },
	{ 0x5f4, 0x1b4 }, { 0x634, 0x1b4 }, { 0x674, 0x1b4 }, { 0x6b4, 0x1b4 }, { 0x6f4, 0x1b4 }, { 0x734, 0x1b4 },
	{ 0x774, 0x1b4 }, { 0x35, 0x1b5 }, { 0xb5, 0x1b5 }, { 0xf5, 0x1b5 }, { 0x135, 0x1b5 }, { 0x175, 0x1b5 },
	{ 0x1b5, 0x1b5 }, { 0x1f5, 0x1b5 }, { 0x235, 0x1b5 }, { 0x275, 0x1b5 }, { 0x2b5, 0x1b5 }, { 0x2f5, 0x1b5 },
	{ 0x335, 0x1b5 }, { 0x375, 0x1b5 }, { 0x3b5, 0x1b5 }, { 0x3f5, 0x1b5 }, { 0x435, 0x1b5 }, { 0x475, 0x1b5 },
	{ 0x4b5, 0x1b5 }, { 0x4f5, 0x1b5 }, { 0x535, 0x1b5 }, { 0x575, 0x1b5 }, { 0x5b5, 0x1b5 }, { 0x5f5, 0x1b5 },
	{ 0x635, 0x1b5 }, { 0x675, 0x1b5 }, { 0x6b5, 0x1b5 }, { 0x6f5, 0x1b5 }, { 0x735, 0x1b5 }, { 0x775, 0x1b5 },
	{ 0x36, 0x1b6 }, { 0xb6, 0x1b6 }, { 0xf6, 0x1b6 }, { 0x136, 0x1b6 }, { 0x176, 0x1b6 }, { 0x1b6, 0x1b6 },
	{ 0x1f6, 0x1b6 }, { 0x236, 0x1b6 }, { 0x276, 0x1b6 }, { 0x2b6, 0x1b6 }, { 0x2f6, 0x1b6 }, { 0x336, 0x1b6 },
	{ 0x376, 0x1b6 }, { 0x3b6, 0x1b6 }, { 0x3f6, 0x1b6 }, { 0x436, 0x1b6 }, { 0x476, 0x1b6 }, { 0x4b6, 0x1b6 },
	{ 0x4f6, 0x1b6 }, { 0x536, 0x1b6 }, { 0x576, 0x1b6 }, { 0x5b6, 0x1b6 }, { 0x5f6, 0x1b6 }, { 0x636, 0x1b6 },
	{ 0x676, 0x1b6 }, { 0x6b6, 0x1b6 }, { 0x6f6, 0x1b6 }, { 0x736, 0x1b6 }, { 0x776, 0x1b6 }, { 0x37, 0x1b7 },
	{ 0xb7, 0x1b7 }, { 0xf7, 0x1b7 }, { 0x137, 0x1b7 }, { 0x177, 0x1b7 }, { 0x1b7, 0x1b7 }, { 0x1f7, 0x1b7 },
	{ 0x237, 0x1b7 }, { 0x277, 0x1b7 }, { 0x2b7, 0x1b7 }, { 0x2f7, 0x1b7 }, { 0x337, 0x1b7 }, { 0x377, 0x1b7 },
	{ 0x3b7, 0x1b7 }, { 0x3f7, 0x1b7 }, { 0x437, 0x1b7 }, { 0x477, 0x1b7 }, { 0x4b7, 0x1b7 }, { 0x4f7, 0x1b7 },
	{ 0x537, 0x1b7 }, { 0x577, 0x1b7 }, { 0x5b7, 0x1b7 }, { 0x5f7, 0x1b7 }, { 0x637, 0x1b7 }, { 0x677, 0x1b7 },
	{ 0x6b7, 0x1b7 }, { 0x6f7, 0x1b7 }, { 0x737, 0x1b7 }, { 0x777, 0x1b7 }, { 0x38, 0x1f8 }, { 0xb8, 0x1f8 },
	{ 0xf8, 0x1f8 }, { 0x138, 0x1f8 }, { 0x178, 0x1f8 }, { 0x1b8, 0x1f8 }, { 0x1f8, 0x1f8 }, { 0x238, 0x1f8 },
	{ 0x278, 0x1f8 }, { 0x2b8, 0x1f8 }, { 0x2f8, 0x1f8 }, { 0x338, 0x1f8 }, { 0x378, 0x1f8 }, { 0x3b8, 0x1f8 },
	{ 0x3f8, 0x1f8 }, { 0x438, 0x1f8 }, { 0x478, 0x1f8 }, { 0x4b8, 0x1f8 }, { 0x4f8, 0x1f8 }, { 0x538, 0x1f8 },
	{ 0x578, 0x1f8 }, { 0x5b8, 0x1f8 }, { 0x5f8, 0x1f8 }, { 0x638, 0x1f8 }, { 0x678, 0x1f8 }, { 0x6b8, 0x1f8 },
	{ 0x6f8, 0x1f8 }, { 0x738, 0x1f8 }, { 0x778, 0x1f8 }, { 0x39, 0x1f9 }, { 0xb9, 0x1f9 }, { 0xf9, 0x1f9 },
	{ 0x139, 0x1f9 }, { 0x179, 0x1f9 }, { 0x1b9, 0x1f9 }, { 0x1f9, 0x1f9 }, { 0x239, 0x1f9 }, { 0x279, 0x1f9 },
	{ 0x2b9, 0x1f9 }, { 0x2f9, 0x1f9 }, { 0x339, 0x1f9 }, { 0x379, 0x1f9 }, { 0x3b9, 0x1f9 }, { 0x3f9, 0x1f9 },
	{ 0x439, 0x1f9 }, { 0x479, 0x1f9 }, { 0x4b9, 0x1f9 }, { 0x4f9, 0x1f9 }, { 0x539, 0x1f9 }, { 0x579, 0x1f9 },
	{ 0x5b9, 0x1f9 }, { 0x5f9, 0x1f9 }, { 0x639, 0x1f9 }, { 0x679, 0x1f9 }, { 0x6b9, 0x1f9 }, { 0x6f9, 0x1f9 },
	{ 0x739, 0x1f9 }, { 0x779, 0x1f9 }, { 0x3a, 0x1fa }, { 0xba, 0x1fa }, { 0xfa, 0x1fa }, { 0x13a, 0x1fa },
	{ 0x17a, 0x1fa }, { 0x1ba, 0x1fa }, { 0x1fa, 0x1fa }, { 0x23a, 0x1fa }, { 0x27a, 0x1fa }, { 0x2ba, 0x1fa },
	{ 0x2fa, 0x1fa }, { 0x33a, 0x1fa }, { 0x37a, 0x1fa }, { 0x3ba, 0x1fa }, { 0x3fa, 0x1fa }, { 0x43a, 0x1fa },
	{ 0x47a, 0x1fa }, { 0x4ba, 0x1fa }, { 0x4fa, 0x1fa }, { 0x53a, 0x1fa }, { 0x57a, 0x1fa }, { 0x5ba, 0x1fa },
	{ 0x5fa, 0x1fa }, { 0x63a, 0x1fa }, { 0x67a, 0x1fa }, { 0x6ba, 0x1fa }, { 0x6fa, 0x1fa }, { 0x73a, 0x1fa },
	{ 0x77a, 0x1fa }, { 0x3b, 0x1fb }, { 0xbb, 0x1fb }, { 0xfb, 0x1fb }, { 0x13b, 0x1fb }, { 0x17b, 0x1fb },
	{ 0x1bb, 0x1fb }, { 0x1fb, 0x1fb }, { 0x23b, 0x1fb }, { 0x27b, 0x1fb }, { 0x2bb, 0x1fb }, { 0x2fb, 0x1fb },
	{ 0x33b, 0x1fb }, { 0x37b, 0x1fb }, { 0x3bb, 0x1fb }, { 0x3fb, 0x1fb }, { 0x43b, 0x1fb }, { 0x47b, 0x1fb },
	{ 0x4bb, 0x1fb }, { 0x4fb, 0x1fb }, { 0x53b, 0x1fb }, { 0x57b, 0x1fb }, { 0x5bb, 0x1fb }, { 0x5fb, 0x1fb },
	{ 0x63b, 0x1fb }, { 0x67b, 0x1fb }, { 0x6bb, 0x1fb }, { 0x6fb, 0x1fb }, { 0x73b, 0x1fb }, { 0x77b, 0x1fb },
	{ 0x3c, 0x1fc }, { 0xbc, 0x1fc }, { 0xfc, 0x1fc }, { 0x13c, 0x1fc }, { 0x17c, 0x1fc }, { 0x1bc, 0x1fc },
	{ 0x1fc, 0x1fc }, { 0x23c, 0x1fc }, { 0x27c, 0x1fc }, { 0x2bc, 0x1fc }, { 0x2fc, 0x1fc }, { 0x33c, 0x1fc },
	{ 0x37c, 0x1fc }, { 0x3bc, 0x1fc }, { 0x3fc, 0x1fc }, { 0x43c, 0x1fc }, { 0x47c, 0x1fc }, { 0x4bc, 0x1fc },
	{ 0x4fc, 0x1fc }, { 0x53c, 0x1fc }, { 0x57c, 0x1fc }, { 0x5bc, 0x1fc }, { 0x5fc, 0x1fc }, { 0x63c, 0x1fc },
	{ 0x67c, 0x1fc }, { 0x6bc, 0x1fc }, { 0x6fc, 0x1fc }, { 0x73c, 0x1fc }, { 0x77c, 0x1fc }, { 0x3d, 0x1fd },
	{ 0xbd, 0x1fd }, { 0xfd, 0x1fd }, { 0x13d, 0x1fd }, { 0x17d, 0x1fd }, { 0x1bd, 0x1fd }, { 0x1fd, 0x1fd },
	{ 0x23d, 0x1fd }, { 0x27d, 0x1fd }, { 0x2bd, 0x1fd }, { 0x2fd, 0x1fd }, { 0x33d, 0x1fd }, { 0x37d, 0x1fd },
	{ 0x3bd, 0x1fd }, { 0x3fd, 0x1fd }, { 0x43d, 0x1fd }, { 0x47d, 0x1fd }, { 0x4bd, 0x1fd }, { 0x4fd, 0x1fd },
	{ 0x53d, 0x1fd }, { 0x57d, 0x1fd }, { 0x5bd, 0x1fd }, { 0x5fd, 0x1fd }, { 0x63d, 0x1fd }, { 0x67d, 0x1fd },
	{ 0x6bd, 0x1fd }, { 0x6fd, 0x1fd }, { 0x73d, 0x1fd }, { 0x77d, 0x1fd }, { 0x3e, 0x1fe }, { 0xbe, 0x1fe },
	{ 0xfe, 0x1fe }, { 0x13e, 0x1fe }, { 0x17e, 0x1fe }, { 0x1be, 0x1fe }, { 0x1fe, 0x1fe }, { 0x23e, 0x1fe },
	{ 0x27e, 0x1fe }, { 0x2be, 0x1fe }, { 0x2fe, 0x1fe }, { 0x33e, 0x1fe }, { 0x37e, 0x1fe }, { 0x3be, 0x1fe },
	{ 0x3fe, 0x1fe }, { 0x43e, 0x1fe }, { 0x47e, 0x1fe }, { 0x4be, 0x1fe }, { 0x4fe, 0x1fe }, { 0x53e, 0x1fe },
	{ 0x57e, 0x1fe }, { 0x5be, 0x1fe }, { 0x5fe, 0x1fe }, { 0x63e, 0x1fe }, { 0x67e, 0x1fe }, { 0x6be, 0x1fe },
	{ 0x6fe, 0x1fe }, { 0x73e, 0x1fe }, { 0x77e, 0x1fe }, { 0x3f, 0x1ff }, { 0xbf, 0x1ff }, { 0xff, 0x1ff },
	{ 0x13f, 0x1ff }, { 0x17f, 0x1ff }, { 0x1bf, 0x1ff }, { 0x1ff, 0x1ff }, { 0x23f, 0x1ff }, { 0x27f, 0x1ff },
	{ 0x2bf, 0x1ff }, { 0x2ff, 0x1ff }, { 0x33f, 0x1ff }, { 0x37f, 0x1ff }, { 0x3bf, 0x1ff }, { 0x3ff, 0x1ff },
	{ 0x43f, 0x1ff }, { 0x47f, 0x1ff }, { 0x4bf, 0x1ff }, { 0x4ff, 0x1ff }, { 0x53f, 0x1ff }, { 0x57f, 0x1ff },
	{ 0x5bf, 0x1ff }, { 0x5ff, 0x1ff }, { 0x63f, 0x1ff }, { 0x67f, 0x1ff }, { 0x6bf, 0x1ff }, { 0x6ff, 0x1ff },
	{ 0x73f, 0x1ff }, { 0x77f, 0x1ff },
};

static const u32 stg_vlan_fp_vals[] = {
	0x19, 0x18, 0x17, 0x16, 0x15, 0x14, 0x19, 0x18, 0x17, 0x16, 0x15, 0x14,
};

static const u32 egr_mtu_size[] = {
	0x1b, 2, 3, 4, 5, 6, 7, 8,
};

static const u32 fp_port_field_sel[] = {
	0, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
	0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x3e,
};

static void hr2_soft_reset(struct bcm_cmic *c)
{
	bcm_cmic_write(c, CMIC_RXBUF_EPINTF_RELEASE_ALL_CREDITS, 0);
	bcm_cmic_write(c, CMIC_RXBUF_EPINTF_RELEASE_ALL_CREDITS, 1);
	bcm_cmic_write(c, CMIC_CPS_RESET, 1);
	bcm_cmic_write(c, CMIC_UNNAMED_148, 0x3c00);
	bcm_cmic_write(c, CMIC_SBUS_RING_MAP_0_7, 0x11122200);
	bcm_cmic_write(c, CMIC_SBUS_RING_MAP_8_15, 0x430000);
	bcm_cmic_write(c, CMIC_SBUS_RING_MAP_16_23, 0x5004);
	bcm_cmic_write(c, CMIC_SBUS_RING_MAP_24_31, 0);
	bcm_cmic_reg(c, BLK_TOP, TOP_SOFT_RESET_REG, 0xf7fd8);
	bcm_cmic_reg(c, BLK_TOP, TOP_SOFT_RESET_REG, 0xffff8);
	bcm_cmic_reg(c, BLK_TOP, TOP_SOFT_RESET_REG_2, 0x100);
	bcm_cmic_reg(c, BLK_TOP, TOP_SOFT_RESET_REG_2, 0x300);
	bcm_cmic_write(c, CMIC_SBUS_TIMEOUT, 0x7d0);
	bcm_cmic_reg(c, BLK_TOP, TOP_SOFT_RESET_REG, 0xfffff);
	bcm_cmic_reg(c, BLK_TOP, TOP_PVTMON_CTRL_0, 0x50003);
	bcm_cmic_reg(c, BLK_TOP, TOP_SOFT_RESET_REG_2, 0x300);
	bcm_cmic_reg(c, BLK_TOP, TOP_SOFT_RESET_REG_2, 0x10300);
	bcm_cmic_reg(c, BLK_TOP, TOP_PVTMON_CTRL_1, 0);
	bcm_cmic_reg(c, BLK_TOP, TOP_PVTMON_CTRL_1, 0x80000000);
	bcm_cmic_reg(c, BLK_TOP, TOP_XG_PLL0_CTRL_3, 0x5c00000);
	bcm_cmic_reg(c, BLK_TOP, TOP_XG_PLL1_CTRL_3, 0x5400000);
	bcm_cmic_reg(c, BLK_TOP, TOP_BS_PLL0_CTRL_3, 0x5000000);
	bcm_cmic_reg(c, BLK_TOP, TOP_BS_PLL1_CTRL_3, 0x5000000);
	bcm_cmic_reg(c, BLK_TOP, TOP_MISC_CONTROL_1, 0x1e000);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x10000000);
	for (unsigned int i = 0; i < ARRAY_SIZE(soft_reset_blk_port); i++)
		bcm_cmic_reg(c, soft_reset_blk_port[i][0], XLPORT_XGXS0_CTRL_REG, soft_reset_blk_port[i][1]);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_MAC_CONTROL, 1);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_MAC_CONTROL, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_MAC_CONTROL, 1);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_MAC_CONTROL, 0);
	bcm_cmic_write(c, CMIC_PCIE_USERIF_PURGE_CONTROL, 0x13);
	bcm_cmic_write(c, CMIC_RXBUF_EPINTF_RELEASE_ALL_CREDITS, 0);
	bcm_cmic_write(c, CMIC_RXBUF_EPINTF_RELEASE_ALL_CREDITS, 1);
	bcm_cmic_reg(c, BLK_IPIPE, ING_HW_RESET_CONTROL_1, 0);
	bcm_cmic_reg(c, BLK_IPIPE, ING_HW_RESET_CONTROL_2, 0x38000);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_0, 0);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_1, 0x34000);
	bcm_cmic_poll(c, BLK_IPIPE, ING_HW_RESET_CONTROL_2, 0x40000);
	bcm_cmic_poll(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_1, 0x40000);
	bcm_cmic_reg(c, BLK_IPIPE, ING_HW_RESET_CONTROL_2, 0);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_1, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, VLAN_SUBNET, 0, 0, 0xfffffffe, 0xffffffff, 1, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, VLAN_SUBNET_ONLY, 0, 0, 0xfffffffe, 0xffffffff, 1);
	for (unsigned int i = 0; i < ARRAY_SIZE(ing_hw_reset_control_1); i++) {
		bcm_cmic_reg(c, BLK_IPIPE, ING_HW_RESET_CONTROL_1, ing_hw_reset_control_1[i][0]);
		bcm_cmic_reg(c, BLK_IPIPE, ING_HW_RESET_CONTROL_2, ing_hw_reset_control_1[i][1]);
		bcm_cmic_poll(c, BLK_IPIPE, ING_HW_RESET_CONTROL_2, 0x40000);
		bcm_cmic_reg(c, BLK_IPIPE, ING_HW_RESET_CONTROL_2, 0);
		bcm_cmic_reg(c, BLK_IPIPE, ING_HW_RESET_CONTROL_1, 0);
	}
	bcm_cmic_reg(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_0, 0x2c480000);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_1, 0x20200);
	bcm_cmic_poll(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_1, 0x40000);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_1, 0);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_HW_RESET_CONTROL_0, 0);
}

static void hr2_ipmc_vlan_sel_boot(struct bcm_cmic *c)
{
	for (int i = 0; i < 256; i++) {
		bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x5d41);
		bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x1d41);
	}
}

static void hr2_port_map_ecc(struct bcm_cmic *c)
{
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_MIB_RESET, 0xf);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_MIB_RESET, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_MIB_RESET, 0xf);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_MIB_RESET, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_map_ecc_vals); i++) {
		bcm_cmic_mem32(c, BLK_IPIPE, ING_PHYSICAL_TO_LOGICAL_PORT_NUMBER_MAPPING_TABLE + 1 + i,
		               port_map_ecc_vals[i]);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(port_map_ecc_vals_2); i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_LOGICAL_TO_PHYSICAL_PORT_NUMBER_MAPPING + i, port_map_ecc_vals_2[i]);
	for (unsigned int i = 0; i < ARRAY_SIZE(log_to_phy_port_mapping); i++) {
		bcm_cmic_reg(c, BLK_MMU, LOG_TO_PHY_PORT_MAPPING + log_to_phy_port_mapping[i][0],
		             log_to_phy_port_mapping[i][1]);
	}
	bcm_cmic_reg(c, BLK_MMU, MEMFAILINTMASK, 0xdff8);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_L3_NEXT_HOP_PARITY_CONTROL, 1);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_L3_INTF_PARITY_CONTROL, 1);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_VLAN_PARITY_CONTROL, 1);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_VLAN_STG_PARITY_CONTROL, 1);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_VLAN_XLATE_PARITY_CONTROL, 1);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_EDATABUF_PARITY_CONTROL, 0x800);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_INITBUF_ECC_CONTROL, 1);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_INITBUF_ECC_CONTROL, 1);
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_edatabuf_parity_control); i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_EDATABUF_PARITY_CONTROL, egr_edatabuf_parity_control[i]);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_INTR_ENABLE, 0x5fffe1f);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_map_ecc_regs); i++)
		bcm_cmic_reg(c, BLK_IPIPE, port_map_ecc_regs[i], 1);
	bcm_cmic_reg(c, BLK_IPIPE, IP0_INTR_ENABLE, 0x1ef1);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_map_ecc_regs_2); i++)
		bcm_cmic_reg(c, BLK_IPIPE, port_map_ecc_regs_2[i], 1);
	bcm_cmic_reg(c, BLK_IPIPE, IP1_INTR_ENABLE, 0xffbef0);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_map_ecc_regs_3); i++)
		bcm_cmic_reg(c, BLK_IPIPE, port_map_ecc_regs_3[i], 1);
	bcm_cmic_reg(c, BLK_IPIPE, IP2_INTR_ENABLE, 0x7ff);
	for (unsigned int i = 0; i < ARRAY_SIZE(xlport_ecc_control); i++)
		bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ECC_CONTROL, xlport_ecc_control[i]);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_INTR_ENABLE, 0x1ff);
	for (unsigned int i = 0; i < ARRAY_SIZE(xlport_ecc_control); i++)
		bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ECC_CONTROL, xlport_ecc_control[i]);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_INTR_ENABLE, 0x1ff);
	bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x3d41);
	bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x1d41);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_START, 0x10240000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_END, 0x102401ff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_RESULT, 0);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_PROT_WORD_0, 0x1d600);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_PROT_WORD_1, 0x1d601);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_PROT_WORD_2, 0x3aaec);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_PROT_WORD_3, 0x3aaed);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_0_CONFIG, 0x14029);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_START, 0x1c100000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_END, 0x1c10007f);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_RESULT, 0x40);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_PROT_WORD_0, 0x8a00);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_PROT_WORD_1, 0x8a01);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_PROT_WORD_2, 0x11446);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_PROT_WORD_3, 0x11447);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_1_CONFIG, 0x14029);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_START, 0x3c180000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_END, 0x3c1807ff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_RESULT, 0x50);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_PROT_WORD_0, 0x4a00);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_PROT_WORD_1, 0x4a01);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_PROT_WORD_2, 0x9426);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_PROT_WORD_3, 0x9427);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_2_CONFIG, 0x14029);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_START, 0x10080000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_END, 0x100800ff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_RESULT, 0x150);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_PROT_WORD_0, 0x8000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_PROT_WORD_1, 0x8001);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_PROT_WORD_2, 0x10041);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_PROT_WORD_3, 0x10042);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_3_CONFIG, 0x14029);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_START, 0x100c0000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_END, 0x100c00ff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_RESULT, 0x150);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_PROT_WORD_0, 0x8000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_PROT_WORD_1, 0x8001);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_PROT_WORD_2, 0x10041);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_PROT_WORD_3, 0x10042);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_4_CONFIG, 0x14029);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_START, 0x30000000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_END, 0x300001ff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_RESULT, 0x170);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_PROT_WORD_0, 0x8a00);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_PROT_WORD_1, 0x8a01);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_PROT_WORD_2, 0x11446);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_PROT_WORD_3, 0x11447);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_5_CONFIG, 0x14029);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_START, 0x444c0000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_END, 0x444c007f);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_RESULT, 0x1b0);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_PROT_WORD_0, 0x8a00);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_PROT_WORD_1, 0x8a01);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_PROT_WORD_2, 0x11446);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_PROT_WORD_3, 0x11447);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_6_CONFIG, 0x14029);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_START, 0x2c480000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_END, 0x2c4801ff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_RESULT, 0x1c0);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_PROT_WORD_0, 0x1d600);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_PROT_WORD_1, 0x1d601);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_PROT_WORD_2, 0x3aaec);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_PROT_WORD_3, 0x3aaed);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_7_CONFIG, 0x16029);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_START, 0x3c040000);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_END, 0x3c0407ff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_ADDR_MASK, 0xffffffff);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_RESULT, 0x200);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_PROT_WORD_0, 0x1d600);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_PROT_WORD_1, 0x1d601);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_PROT_WORD_2, 0x3aaec);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_PROT_WORD_3, 0x3aaed);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_8_CONFIG, 0x14021);
	bcm_cmic_reg(c, BLK_SER, SER_RING_ERR_CTRL, 0xf);
	bcm_cmic_reg(c, BLK_SER, SER_RANGE_ENABLE, 0x1ff);
	bcm_cmic_reg(c, BLK_SER, SER_CONFIG_REG, 1);
	bcm_cmic_reg(c, BLK_TOP, TOP_MISC_CONTROL_1, 0x1e800);
	bcm_cmic_reg(c, BLK_TOP, TOP_CORE_PLL_CTRL4, 0xc8);
	bcm_cmic_reg(c, BLK_TOP, TOP_CORE_PLL_CTRL2, 0xc0000);
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_GPORT0, GE0_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT0, GE1_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT0, GE2_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT0, GE3_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT0, GE4_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT0, GE5_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT0, GE6_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT0, GE7_EEE_CONFIG, 0x286);
	}
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_GPORT1, GE0_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT1, GE1_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT1, GE2_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT1, GE3_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT1, GE4_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT1, GE5_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT1, GE6_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT1, GE7_EEE_CONFIG, 0x286);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(port_map_ecc_regs_4); i++)
		bcm_cmic_reg(c, BLK_GPORT2, port_map_ecc_regs_4[i], 0x286);
}

static void hr2_eee_egr_enable(struct bcm_cmic *c)
{
	for (int i = 0; i < 6; i++) {
		bcm_cmic_reg(c, BLK_GPORT2, GE3_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT2, GE4_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT2, GE5_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT2, GE6_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT2, GE7_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT2, GE0_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT2, GE1_EEE_CONFIG, 0x286);
		bcm_cmic_reg(c, BLK_GPORT2, GE2_EEE_CONFIG, 0x286);
	}
	bcm_cmic_reg(c, BLK_GPORT2, GE3_EEE_CONFIG, 0x286);
	bcm_cmic_reg(c, BLK_GPORT2, GE4_EEE_CONFIG, 0x286);
	bcm_cmic_reg(c, BLK_GPORT2, GE5_EEE_CONFIG, 0x286);
	bcm_cmic_reg(c, BLK_GPORT2, GE6_EEE_CONFIG, 0x286);
	bcm_cmic_reg(c, BLK_GPORT2, GE7_EEE_CONFIG, 0x286);
	bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x1d61);
	bcm_cmic_reg(c, BLK_IPIPE, L2_AUX_HASH_CONTROL, 0x15);
	bcm_cmic_reg(c, BLK_IPIPE, L3_AUX_HASH_CONTROL, 0x15);
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_enable); i++)
		bcm_cmic_mem32(c, BLK_EPIPE, EGR_ENABLE + egr_enable[i], 1);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 1);
	bcm_cmic_reg(c, BLK_GPORT0, GPORT_CONFIG, 2);
	bcm_cmic_reg(c, BLK_GPORT1, GPORT_CONFIG, 2);
	bcm_cmic_reg(c, BLK_GPORT2, GPORT_CONFIG, 2);
	bcm_cmic_reg(c, BLK_GPORT0, GPORT_CONFIG, 0);
	bcm_cmic_reg(c, BLK_GPORT1, GPORT_CONFIG, 0);
	bcm_cmic_reg(c, BLK_GPORT2, GPORT_CONFIG, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_SOFT_RESET, 7);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_MODE_REG, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_CNTMAXSIZE, 0xde8);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_SOFT_RESET, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_SOFT_RESET, 1);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_MODE_REG, 0x24);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_CNTMAXSIZE, 0xde8);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_SOFT_RESET, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 1);
	bcm_cmic_reg64(c, BLK_IPIPE, ING_CONFIG_64, 0xc02080300eULL);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_CONFIG_1, 0x4001);
	bcm_cmic_write(c, CMIC_RATE_ADJUST, 0x10028);
	bcm_cmic_write(c, CMIC_RATE_ADJUST_INT_MDIO, 0x10028);
}

static void hr2_port_config_led(struct bcm_cmic *c)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(port_config_led_blk_port); i++) {
		bcm_cmic_reg(c, port_config_led_blk_port[i][0], GPORT_CONFIG, port_config_led_blk_port[i][1]);
		bcm_cmic_reg(c, port_config_led_blk_port[i][2], GPORT_CONFIG, port_config_led_blk_port[i][1]);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_vlan_control_1); i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_VLAN_CONTROL_1 + egr_vlan_control_1[i], 0x1000);
	bcm_cmic_reg(c, BLK_IPIPE, MC_CONTROL_1, 0x10000000);
	bcm_cmic_reg(c, BLK_IPIPE, MC_CONTROL_2, 0x10001000);
	bcm_cmic_reg(c, BLK_IPIPE, MC_CONTROL_3, 0x10002000);
	bcm_cmic_reg(c, BLK_IPIPE, MC_CONTROL_4, 0);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_MC_CONTROL_1, 0x10000000);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_MC_CONTROL_2, 0x2000);
	bcm_cmic_reg(c, BLK_IPIPE, SW2_FP_DST_ACTION_CONTROL, 0xc);
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_vlan_control_1); i++) {
		bcm_cmic_reg(c, BLK_EPIPE, EGR_PORT + egr_vlan_control_1[i], 0x100);
		bcm_cmic_reg(c, BLK_EPIPE, IEGR_PORT + egr_vlan_control_1[i], 0x100);
	}
	bcm_cmic_write(c, CMIC_LEDUP0_PORT_ORDER_REMAP_0_3, 0x1440c2);
	bcm_cmic_write(c, CMIC_LEDUP0_PORT_ORDER_REMAP_4_7, 0x2481c6);
	bcm_cmic_write(c, CMIC_LEDUP0_PORT_ORDER_REMAP_8_11, 0x34c2ca);
	bcm_cmic_write(c, CMIC_LEDUP0_PORT_ORDER_REMAP_12_15, 0x4503ce);
	bcm_cmic_write(c, CMIC_LEDUP0_PORT_ORDER_REMAP_16_19, 0x5544d2);
	bcm_cmic_write(c, CMIC_LEDUP0_PORT_ORDER_REMAP_20_23, 0x6585d6);
	bcm_cmic_write(c, CMIC_LEDUP0_PORT_ORDER_REMAP_24_27, 0x69b71d);
	bcm_cmic_write(c, CMIC_LEDUP0_PORT_ORDER_REMAP_28_31, 0x79f821);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_LED_CHAIN_CONFIG, 6);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_LED_CHAIN_CONFIG, 6);
}

static void hr2_led_ram_tdm(struct bcm_cmic *c)
{
	for (int i = 0; i < 256; i++)
		bcm_cmic_write(c, CMIC_LEDUP0_DATA_RAM + 4 * i, 0);
	bcm_cmic_write(c, CMIC_LEDUP0_CTRL, 0x6a);
	bcm_cmic_reg(c, BLK_TOP, TOP_CLOCKING_ENFORCE_PSG, 0);
	bcm_cmic_reg(c, BLK_IPIPE, IARB_TDM_CONTROL, 0xa7);
	for (unsigned int i = 0; i < ARRAY_SIZE(iarb_tdm_table); i++) {
		bcm_cmic_mem32(c, BLK_IPIPE, IARB_TDM_TABLE + iarb_tdm_table[i][0], iarb_tdm_table[i][1]);
		bcm_cmic_mem32(c, BLK_MMU, MMU_ARB_TDM_TABLE + iarb_tdm_table[i][0], iarb_tdm_table[i][2]);
	}
}

static void hr2_mmu_limits(struct bcm_cmic *c)
{
	bcm_cmic_mem32(c, BLK_IPIPE, IARB_TDM_TABLE + 72, 0x1c);
	bcm_cmic_mem32(c, BLK_MMU, MMU_ARB_TDM_TABLE + 72, 0x1c);
	bcm_cmic_mem32(c, BLK_IPIPE, IARB_TDM_TABLE + 73, 0x1e);
	bcm_cmic_mem32(c, BLK_MMU, MMU_ARB_TDM_TABLE + 73, 0x1d);
	bcm_cmic_mem32(c, BLK_IPIPE, IARB_TDM_TABLE + 74, 0x3f);
	bcm_cmic_mem32(c, BLK_MMU, MMU_ARB_TDM_TABLE + 74, 0x7f);
	bcm_cmic_reg(c, BLK_IPIPE, IARB_TDM_CONTROL, 0x94);
	bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x1d6d);
	bcm_cmic_reg(c, BLK_IPIPE, IP_TO_CMICM_CREDIT_TRANSFER, 0x60);
	for (unsigned int i = 0; i < ARRAY_SIZE(holcospktsetlimit); i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 0x100 * i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 0x100 * i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 0x100 * i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 0x100 * i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 0x100 * i, holcospktsetlimit[i]);
	}
	bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT, 0x5b0);
	bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT, 0x5ac);
	bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT, 0x2fff);
	bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT, 0x5e01798);
	bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT, 0xc);
	bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT, 0x50);
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 2 + i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 2 + i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 2 + i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 2 + i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 2 + i, 0x58f1648);
		for (int j = 0; j < 7; j++) {
			bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 0x102 + i + 0x100 * j, 0x5b8);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 0x102 + i + 0x100 * j, 0x5b4);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 0x102 + i + 0x100 * j, 8);
			bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 0x102 + i + 0x100 * j, 0x3000c);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 0x102 + i + 0x100 * j, 0x98c26f);
		}
		bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 2 + i, 0x5b0);
		bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 2 + i, 0x5ac);
		bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 2 + i, 0x2fff);
		bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 2 + i, 0x5e01798);
		bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 2 + i, 0xc);
		bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 2 + i, 0x50);
	}
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 10 + i + 0x100 * j, 0x5b8);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 10 + i + 0x100 * j, 0x5b4);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 10 + i + 0x100 * j, 8);
			bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 10 + i + 0x100 * j, 0x3000c);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 10 + i + 0x100 * j, 0);
		}
		bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 10 + i, 0x5b0);
		bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 10 + i, 0x5ac);
		bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 10 + i, 0x2fff);
		bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 10 + i, 0x5e01798);
		bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 10 + i, 0xc);
		bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 10 + i, 0x50);
	}
	for (int i = 0; i < 6; i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 18 + i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 18 + i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 18 + i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 18 + i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 18 + i, 0x58f1648);
		for (int j = 0; j < 7; j++) {
			bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 0x112 + i + 0x100 * j, 0x5b8);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 0x112 + i + 0x100 * j, 0x5b4);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 0x112 + i + 0x100 * j, 8);
			bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 0x112 + i + 0x100 * j, 0x3000c);
			bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 0x112 + i + 0x100 * j, 0x98c26f);
		}
		bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 18 + i, 0x5b0);
		bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 18 + i, 0x5ac);
		bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 18 + i, 0x2fff);
		bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 18 + i, 0x5e01798);
		bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 18 + i, 0xc);
		bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 18 + i, 0x50);
	}
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 24 + 0x100 * i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 24 + 0x100 * i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 24 + 0x100 * i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 24 + 0x100 * i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 24 + 0x100 * i, 0);
	}
	bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 24, 0x5b0);
	bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 24, 0x5ac);
	bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 24, 0x2fff);
	bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 24, 0x5e01798);
	bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 24, 0xc);
	bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 24, 0x50);
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 25 + 0x100 * i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 25 + 0x100 * i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 25 + 0x100 * i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 25 + 0x100 * i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 25 + 0x100 * i, 0);
	}
	bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 25, 0x5b0);
	bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 25, 0x5ac);
	bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 25, 0x2fff);
	bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 25, 0x5e01798);
	bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 25, 0xc);
	bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 25, 0x50);
	for (unsigned int i = 0; i < ARRAY_SIZE(holcospktsetlimit); i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 26 + 0x100 * i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 26 + 0x100 * i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 26 + 0x100 * i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 26 + 0x100 * i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 26 + 0x100 * i, holcospktsetlimit[i]);
	}
	bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 26, 0x5b0);
	bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 26, 0x5ac);
	bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 26, 0x2fff);
	bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 26, 0x5e01798);
	bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 26, 0xc);
	bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 26, 0x50);
	for (unsigned int i = 0; i < ARRAY_SIZE(holcospktsetlimit); i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 27 + 0x100 * i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 27 + 0x100 * i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 27 + 0x100 * i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 27 + 0x100 * i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 27 + 0x100 * i, holcospktsetlimit[i]);
	}
	bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 27, 0x5b0);
	bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 27, 0x5ac);
	bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 27, 0x2fff);
	bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 27, 0x5e01798);
	bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 27, 0xc);
	bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 27, 0x50);
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 28 + 0x100 * i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 28 + 0x100 * i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 28 + 0x100 * i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 28 + 0x100 * i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 28 + 0x100 * i, 0);
	}
	bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 28, 0x5b0);
	bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 28, 0x5ac);
	bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 28, 0x2fff);
	bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 28, 0x5e01798);
	bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 28, 0xc);
	bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 28, 0x50);
	for (unsigned int i = 0; i < ARRAY_SIZE(holcospktsetlimit); i++) {
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTSETLIMIT + 29 + 0x100 * i, 0x5b8);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSPKTRESETLIMIT + 29 + 0x100 * i, 0x5b4);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSMINXQCNT + 29 + 0x100 * i, 8);
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 29 + 0x100 * i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, HOLCOSCELLMAXLIMIT + 29 + 0x100 * i, holcospktsetlimit[i]);
	}
	bcm_cmic_reg(c, BLK_MMU, DYNXQCNTPORT + 29, 0x5b0);
	bcm_cmic_reg(c, BLK_MMU, DYNRESETLIMPORT + 29, 0x5ac);
	bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 29, 0x2fff);
	bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 29, 0x5e01798);
	bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 29, 0xc);
	bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 29, 0x50);
	bcm_cmic_reg(c, BLK_MMU, TOTALDYNCELLSETLIMIT, 0x2520);
	bcm_cmic_reg(c, BLK_MMU, TOTALDYNCELLRESETLIMIT, 0x2508);
	bcm_cmic_reg(c, BLK_MMU, PKTAGINGTIMER, 0);
	bcm_cmic_reg(c, BLK_MMU, PKTAGINGLIMIT, 0);
	bcm_cmic_reg(c, BLK_IPIPE, SW2_RAM_CONTROL_4, 0x10);
	bcm_cmic_reg(c, BLK_IPIPE, L2_USER_ENTRY_CAM_DBGCTRL, 0x10);
	bcm_cmic_reg(c, BLK_IPIPE, VLAN_SUBNET_CAM_DBGCTRL, 0x40);
	bcm_cmic_reg(c, BLK_MMU, MMUPORTENABLE, 0x3ffffffd);
	for (int i = 0; i < 7; i++) {
		bcm_cmic_mem32(c, BLK_IPIPE, ING_PRI_CNG_MAP + 1 + 2 * i, 1 + 4 * i);
		bcm_cmic_mem32(c, BLK_IPIPE, ING_PRI_CNG_MAP + 2 + 2 * i, 4 + 4 * i);
	}
	bcm_cmic_mem32(c, BLK_IPIPE, ING_PRI_CNG_MAP + 15, 0x1d);
}

static void hr2_egr_pri_cng_map(struct bcm_cmic *c)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_pri_cng_map); i++)
		bcm_cmic_mem32(c, BLK_EPIPE, EGR_PRI_CNG_MAP + egr_pri_cng_map[i][0], egr_pri_cng_map[i][1]);
}

static void hr2_vlan_tag_action(struct bcm_cmic *c)
{
	bcm_cmic_mem32(c, BLK_IPIPE, ING_VLAN_TAG_ACTION_PROFILE, 0x202104);
	BCM_CMIC_MEM(c, BLK_IPIPE, ING_VLAN_RANGE, 0x1000001, 0x10000, 0x100, 0x1000001, 0x10000, 0x100);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_PORT, 0x108);
	bcm_cmic_reg(c, BLK_EPIPE, IEGR_PORT, 0x109);
	for (int i = 0; i < 28; i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_PORT + 2 + i, 0x2000108 + 0x1000000 * i);
}

static void hr2_serdes_phy_init(struct bcm_cmic *c)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(serdes_phy_init_phys); i++) {
		bcm_cmic_miim(c, 0x000, 0x1f, 0);
		BCM_CMIC_MIIM_SEQ(c, serdes_phy_init_phys[i][0], serdes_seq_1);
		bcm_cmic_miim(c, serdes_phy_init_phys[i][1], 0x17, 0xf7e);
		bcm_cmic_miim(c, serdes_phy_init_phys[i][1], 0x15, 0);
		bcm_cmic_miim(c, serdes_phy_init_phys[i][1], 0x1e, 0x80e - 2 * i);
		bcm_cmic_miim(c, serdes_phy_init_phys[i][1], 0x1f, 0x8000);
		bcm_cmic_miim(c, serdes_phy_init_phys[i][1], 0x1e, 0x87);
		bcm_cmic_miim(c, serdes_phy_init_phys[i][1], 0x1f, 0x8000);
		BCM_CMIC_MIIM_SEQ(c, serdes_phy_init_phys[i][0], serdes_seq_2);
	}
	bcm_cmic_miim(c, 0x000, 0x1f, 0);
	BCM_CMIC_MIIM_SEQ(c, 0x201, serdes_seq_3);
	bcm_cmic_miim(c, 0x000, 0x1f, 0);
	BCM_CMIC_MIIM_SEQ(c, 0x20b, serdes_seq_3);
	for (unsigned int i = 0; i < ARRAY_SIZE(serdes_phy_init_phys_2); i++) {
		bcm_cmic_miim(c, 0x000, 0x1f, 0);
		BCM_CMIC_MIIM_SEQ(c, serdes_phy_init_phys_2[i][0], serdes_seq_1);
		bcm_cmic_miim(c, serdes_phy_init_phys_2[i][1], 0x17, 0xf7e);
		bcm_cmic_miim(c, serdes_phy_init_phys_2[i][1], 0x15, 0);
		bcm_cmic_miim(c, serdes_phy_init_phys_2[i][1], 0x1e, 0x802 + 2 * i);
		bcm_cmic_miim(c, serdes_phy_init_phys_2[i][1], 0x1f, 0x8000);
		bcm_cmic_miim(c, serdes_phy_init_phys_2[i][1], 0x1e, 0x87);
		bcm_cmic_miim(c, serdes_phy_init_phys_2[i][1], 0x1f, 0x8000);
		BCM_CMIC_MIIM_SEQ(c, serdes_phy_init_phys_2[i][0], serdes_seq_2);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(serdes_phy_init_vals); i++)
		bcm_cmic_miim(c, 0x251, serdes_phy_init_vals[i][0], serdes_phy_init_vals[i][1]);
	BCM_CMIC_MIIM_SEQ(c, 0x014, phy_seq_4);
	BCM_CMIC_MIIM_SEQ(c, 0x251, serdes_seq_5);
	BCM_CMIC_MIIM_SEQ(c, 0x015, phy_seq_6);
	bcm_cmic_miim(c, 0x014, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x014, 0x15, 0);
	bcm_cmic_miim(c, 0x014, 0x1e, 0x802);
	bcm_cmic_miim(c, 0x014, 0x1f, 0x8000);
	bcm_cmic_miim(c, 0x014, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x014, 0x1f, 0x8000);
	BCM_CMIC_MIIM_SEQ(c, 0x015, phy_seq_7);
	BCM_CMIC_MIIM_SEQ(c, 0x251, serdes_seq_8);
	BCM_CMIC_MIIM_SEQ(c, 0x016, phy_seq_6);
	bcm_cmic_miim(c, 0x014, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x014, 0x15, 0);
	bcm_cmic_miim(c, 0x014, 0x1e, 0x804);
	bcm_cmic_miim(c, 0x014, 0x1f, 0x8000);
	bcm_cmic_miim(c, 0x014, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x014, 0x1f, 0x8000);
	BCM_CMIC_MIIM_SEQ(c, 0x016, phy_seq_7);
	BCM_CMIC_MIIM_SEQ(c, 0x251, serdes_seq_9);
	for (int i = 0; i < 4; i++) {
		BCM_CMIC_MIIM_SEQ(c, 0x17 + i, phy_seq_6);
		bcm_cmic_miim(c, 0x014, 0x17, 0xf7e);
		bcm_cmic_miim(c, 0x014, 0x15, 0);
		bcm_cmic_miim(c, 0x014, 0x1e, 0x806 + 2 * i);
		bcm_cmic_miim(c, 0x014, 0x1f, 0x8000);
		bcm_cmic_miim(c, 0x014, 0x1e, 0x87);
		bcm_cmic_miim(c, 0x014, 0x1f, 0x8000);
		BCM_CMIC_MIIM_SEQ(c, 0x17 + i, phy_seq_7);
		BCM_CMIC_MIIM_SEQ(c, 0x251, serdes_seq_10);
	}
	BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_6);
	bcm_cmic_miim(c, 0x014, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x014, 0x15, 0);
	bcm_cmic_miim(c, 0x014, 0x1e, 0x80e);
	bcm_cmic_miim(c, 0x014, 0x1f, 0x8000);
	bcm_cmic_miim(c, 0x014, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x014, 0x1f, 0x8000);
	BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_7);
	BCM_CMIC_MIIM_SEQ(c, 0x251, serdes_seq_11);
	for (unsigned int i = 0; i < ARRAY_SIZE(serdes_phy_init_vals_2); i++) {
		bcm_cmic_miim(c, serdes_phy_init_vals_2[i][0], serdes_phy_init_vals_2[i][1],
		              serdes_phy_init_vals_2[i][2]);
	}
}

static void hr2_port_mac_init(struct bcm_cmic *c)
{
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_MAC_CONTROL, 1);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_MAC_CONTROL, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_MAC_CONTROL, 1);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_MAC_CONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FRM_LENGTH + 7, 0x5ee);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x10020d8);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, GPORT_RSV_MASK, 0x70);
	bcm_cmic_reg(c, BLK_GPORT0, GPORT_STAT_UPDATE_MASK, 0x70);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_GPORT0, PAUSE_CONTROL + 7, 0x3ffff);
	bcm_cmic_reg(c, BLK_GPORT0, PAUSE_QUANT + 7, 0xffff);
	bcm_cmic_reg(c, BLK_GPORT0, TX_IPG_LENGTH + 7, 0xc);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0);
	for (int i = 0; i < 7; i++) {
		bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, BIT(2 + i));
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x11802158);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x11800158);
		bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 7 - i, 1);
		bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 2 + i, 0);
		bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 2 + i, 0);
		bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 7 - i, 0);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x11802158);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x1800058);
		bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, FRM_LENGTH + 6 - i, 0x5ee);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6 - i, 0x10020d8);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, GPORT_RSV_MASK, 0x70);
		bcm_cmic_reg(c, BLK_GPORT0, GPORT_STAT_UPDATE_MASK, 0x70);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6 - i, 0x1800058);
		bcm_cmic_reg(c, BLK_GPORT0, PAUSE_CONTROL + 6 - i, 0x3ffff);
		bcm_cmic_reg(c, BLK_GPORT0, PAUSE_QUANT + 6 - i, 0xffff);
		bcm_cmic_reg(c, BLK_GPORT0, TX_IPG_LENGTH + 6 - i, 0xc);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6 - i, 0x1800058);
	}
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x200);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 9, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 9, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1802058);
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_GPORT1, FRM_LENGTH + i, 0x5ee);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x10020d8);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT1, GPORT_RSV_MASK, 0x70);
		bcm_cmic_reg(c, BLK_GPORT1, GPORT_STAT_UPDATE_MASK, 0x70);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1800058);
		bcm_cmic_reg(c, BLK_GPORT1, PAUSE_CONTROL + i, 0x3ffff);
		bcm_cmic_reg(c, BLK_GPORT1, PAUSE_QUANT + i, 0xffff);
		bcm_cmic_reg(c, BLK_GPORT1, TX_IPG_LENGTH + i, 0xc);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1800058);
		bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, BIT(10 + i));
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x11802158);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x11800158);
		bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + i, 1);
		bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 10 + i, 0);
		bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 10 + i, 0);
		bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + i, 0);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x11802158);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1800058);
		bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1802058);
	}
	bcm_cmic_reg(c, BLK_GPORT2, FRM_LENGTH, 0x5ee);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x10020d8);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, GPORT_RSV_MASK, 0x70);
	bcm_cmic_reg(c, BLK_GPORT2, GPORT_STAT_UPDATE_MASK, 0x70);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_GPORT2, PAUSE_CONTROL, 0x3ffff);
	bcm_cmic_reg(c, BLK_GPORT2, PAUSE_QUANT, 0xffff);
	bcm_cmic_reg(c, BLK_GPORT2, TX_IPG_LENGTH, 0xc);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x40000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 18, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 18, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, FRM_LENGTH + 1, 0x5ee);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x10020d8);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, GPORT_RSV_MASK, 0x70);
	bcm_cmic_reg(c, BLK_GPORT2, GPORT_STAT_UPDATE_MASK, 0x70);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_GPORT2, PAUSE_CONTROL + 1, 0x3ffff);
	bcm_cmic_reg(c, BLK_GPORT2, PAUSE_QUANT + 1, 0xffff);
	bcm_cmic_reg(c, BLK_GPORT2, TX_IPG_LENGTH + 1, 0xc);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x80000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 1, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0x8010);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 19, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 19, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0x8010);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 1, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0x8010);
	for (int i = 0; i < 6; i++) {
		bcm_cmic_reg(c, BLK_GPORT2, FRM_LENGTH + 2 + i, 0x5ee);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x10020d8);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT2, GPORT_RSV_MASK, 0x70);
		bcm_cmic_reg(c, BLK_GPORT2, GPORT_STAT_UPDATE_MASK, 0x70);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1800058);
		bcm_cmic_reg(c, BLK_GPORT2, PAUSE_CONTROL + 2 + i, 0x3ffff);
		bcm_cmic_reg(c, BLK_GPORT2, PAUSE_QUANT + 2 + i, 0xffff);
		bcm_cmic_reg(c, BLK_GPORT2, TX_IPG_LENGTH + 2 + i, 0xc);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1800058);
		bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, BIT(20 + i));
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x11802158);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x11800158);
		bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 2 + i, 1);
		bcm_cmic_miim(c, 0x251, 0x1f, 0);
		bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 20 + i, 0);
		bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 20 + i, 0);
		bcm_cmic_miim(c, 0x251, 0x1f, 0);
		bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 2 + i, 0);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x11802158);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1800058);
		bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1802058);
		bcm_cmic_miim(c, 0x251, 0x1f, 0);
	}
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 6);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1000);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_CTRL, 0x408);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PFC_CTRL, 0x1ffffc000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_MAX_SIZE, 0x3fe8);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_MODE, 0x40);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_LSS_CTRL, 0x30);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1003);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1001);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL, 0x1ffff3c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1041);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL, 0x300000c806ULL);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 6);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1001);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 26, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 26, 0);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1041);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 5);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 1, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 1, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1000);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_CTRL + 1, 0x408);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 1, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 1, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PFC_CTRL + 1, 0x1ffffc000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_MAX_SIZE + 1, 0x3fe8);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_MODE + 1, 0x40);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_LSS_CTRL + 1, 0x30);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1003);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1001);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 1, 0x1ffff3c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1041);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 1, 0x300000c806ULL);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 5);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 1, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 1, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1001);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 27, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 27, 0);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 1, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 1, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1041);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 2, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 2, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1000);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_CTRL + 2, 0x408);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 2, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 2, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PFC_CTRL + 2, 0x1ffffc000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_MAX_SIZE + 2, 0x3fe8);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_MODE + 2, 0x40);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_LSS_CTRL + 2, 0x30);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1003);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1001);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 2, 0x1ffff3c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1041);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 2, 0x300000c806ULL);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 2, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 2, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1001);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 28, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 28, 0);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 2, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 2, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1041);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_TXFIFO_CTRL, 3);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_TXFIFO_CTRL, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 1);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1000);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_RX_CTRL, 0x408);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_TX_CTRL, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_PAUSE_CTRL, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_PFC_CTRL, 0x1ffffc000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_RX_MAX_SIZE, 0x3fe8);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_MODE, 0x40);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_RX_LSS_CTRL, 0x30);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1003);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1001);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_PAUSE_CTRL, 0x1ffff3c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1041);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_TX_CTRL, 0x300000c806ULL);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_TXFIFO_CTRL, 3);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_TXFIFO_CTRL, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 1);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1001);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 29, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 29, 0);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_TX_CTRL, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_PAUSE_CTRL, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1041);
	for (int i = 0; i < 4; i++) {
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x1800058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x180005b);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x180205b);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x180005b);
		bcm_cmic_miim(c, 0x209 - i, 0, 0x1340);
	}
	for (int i = 0; i < 4; i++) {
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3 - i, 0x1800058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3 - i, 0x180005b);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3 - i, 0x180205b);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3 - i, 0x180005b);
		bcm_cmic_miim(c, 0x204 - i, 0, 0x1340);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(command_config); i++) {
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x1800058);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x180005b);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x180205b);
		bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + i, 0x180005b);
		bcm_cmic_miim(c, command_config[i], 0, 0x1340);
	}
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x180205b);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x180005b);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_mac_init_vals); i++)
		bcm_cmic_miim(c, port_mac_init_vals[i][0], port_mac_init_vals[i][1], port_mac_init_vals[i][2]);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x180205b);
	bcm_cmic_miim(c, 0x251, 0x1f, 0x8010);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x180005b);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_mac_init_vals_2); i++)
		bcm_cmic_miim(c, port_mac_init_vals_2[i][0], port_mac_init_vals_2[i][1], port_mac_init_vals_2[i][2]);
	for (int i = 0; i < 5; i++) {
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x1800058);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x180005b);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x180205b);
		bcm_cmic_miim(c, 0x251, 0x1f, 0);
		bcm_cmic_miim(c, 0x251, 0, 0x940);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2 + i, 0x180005b);
		bcm_cmic_miim(c, 0x16 + i, 0x1c, 0x7c00);
		bcm_cmic_miim(c, 0x16 + i, 0x1c, 0xfc01);
		bcm_cmic_miim(c, 0x16 + i, 0x1c, 0xfc00);
		bcm_cmic_miim(c, 0x251, 0x1f, 3);
		bcm_cmic_miim(c, 0x251, 0x1f, 0);
		bcm_cmic_miim(c, 0x251, 0, 0x140);
		bcm_cmic_miim(c, 0x16 + i, 0x1c, 0x7c00);
		bcm_cmic_miim(c, 0x16 + i, 0x1c, 0xfc01);
		bcm_cmic_miim(c, 0x16 + i, 0, 0x1340);
		bcm_cmic_miim(c, 0x16 + i, 0x1c, 0xfc00);
		bcm_cmic_miim(c, 0x251, 0x1f, 3);
		bcm_cmic_miim(c, 0x251, 0x1f, 0);
		bcm_cmic_miim(c, 0x251, 0x1f, 0);
	}
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x180205b);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0, 0x940);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x180005b);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_mac_init_vals_3); i++)
		bcm_cmic_miim(c, port_mac_init_vals_3[i][0], port_mac_init_vals_3[i][1], port_mac_init_vals_3[i][2]);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 6);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1003);
	bcm_cmic_miim(c, 0x241, 0x1f, 0);
	bcm_cmic_miim(c, 0x241, 0x1f, 0);
	bcm_cmic_miim(c, 0x241, 9, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 5);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 1, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 1, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1003);
	bcm_cmic_miim(c, 0x242, 0x1f, 0);
	bcm_cmic_miim(c, 0x242, 0x1f, 0);
	bcm_cmic_miim(c, 0x242, 9, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 2, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 2, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1003);
	bcm_cmic_miim(c, 0x243, 0x1f, 0);
	bcm_cmic_miim(c, 0x243, 0x1f, 0);
	bcm_cmic_miim(c, 0x243, 9, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_TXFIFO_CTRL, 3);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_TXFIFO_CTRL, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 1);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1003);
	bcm_cmic_miim(c, 0x245, 0x1f, 0);
	bcm_cmic_miim(c, 0x245, 0x1f, 0);
	bcm_cmic_miim(c, 0x245, 9, 0);
}

static void hr2_egr_vlan_control(struct bcm_cmic *c)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_vlan_control_1); i++) {
		bcm_cmic_reg(c, BLK_EPIPE, EGR_VLAN_CONTROL_1 + egr_vlan_control_1[i], 0x1000);
		bcm_cmic_reg(c, BLK_EPIPE, EGR_VLAN_CONTROL_1 + egr_vlan_control_1[i], 0x5000);
	}
}

static void hr2_system_config_table(struct bcm_cmic *c)
{
	for (int i = 0; i < 4096; i++)
		bcm_cmic_mem32(c, BLK_IPIPE, SYSTEM_CONFIG_TABLE + i, 0x11);
}

static void hr2_meter_buckets(struct bcm_cmic *c)
{
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGBUCKET, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG, 0x50000);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG, 0x50008);
	bcm_cmic_reg(c, BLK_IPIPE, L2_AGE_TIMER, 0);
	bcm_cmic_reg(c, BLK_IPIPE, SYS_MAC_LIMIT_CONTROL, 0xfffc);
	bcm_cmic_mem32(c, BLK_IPIPE, PORT_OR_TRUNK_MAC_LIMIT, 0xfffc);
	bcm_cmic_mem32(c, BLK_IPIPE, VLAN_OR_VFI_MAC_LIMIT, 0xfffc);
	bcm_cmic_reg(c, BLK_IPIPE, HIGIG_TRUNK_CONTROL, 0xf);
	bcm_cmic_reg(c, BLK_IPIPE, HG_TRUNK_BITMAP, 0);
	bcm_cmic_reg(c, BLK_IPIPE, HG_TRUNK_GROUP, 0);
	bcm_cmic_reg(c, BLK_IPIPE, HG_TRUNK_BITMAP + 0x100, 0);
	bcm_cmic_reg(c, BLK_IPIPE, HG_TRUNK_GROUP + 0x100, 0);
	for (int i = 0; i < 28; i++) {
		for (int j = 0; j < 8; j++) {
			bcm_cmic_reg(c, BLK_MMU, MAXBUCKETCONFIG + 2 + i + 0x100 * j, 0);
			bcm_cmic_reg(c, BLK_MMU, MINBUCKETCONFIG + 2 + i + 0x100 * j, 0);
			bcm_cmic_reg(c, BLK_MMU, MAXBUCKET + 2 + i + 0x100 * j, 0);
			bcm_cmic_reg(c, BLK_MMU, MINBUCKET + 2 + i + 0x100 * j, 0);
			bcm_cmic_reg(c, BLK_MMU, MINBUCKETCONFIG + 2 + i + 0x100 * j, 0);
			bcm_cmic_reg(c, BLK_MMU, MAXBUCKETCONFIG + 2 + i + 0x100 * j, 0);
		}
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_vlan_control_1); i++)
		bcm_cmic_reg(c, BLK_MMU, XQCOSARBSEL + egr_vlan_control_1[i], 2);
}

static void hr2_cos_weights_limits(struct bcm_cmic *c)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_vlan_control_1); i++) {
		bcm_cmic_reg(c, BLK_MMU, WRRWEIGHT_COS + egr_vlan_control_1[i], 0x80);
		bcm_cmic_reg(c, BLK_MMU, WRRWEIGHT_COS + egr_vlan_control_1[i] + 0x100, 0);
		bcm_cmic_reg(c, BLK_MMU, WRRWEIGHT_COS + egr_vlan_control_1[i] + 0x200, 0);
		bcm_cmic_reg(c, BLK_MMU, WRRWEIGHT_COS + egr_vlan_control_1[i] + 0x300, 0);
		bcm_cmic_reg(c, BLK_MMU, WRRWEIGHT_COS + egr_vlan_control_1[i] + 0x400, 0);
		bcm_cmic_reg(c, BLK_MMU, WRRWEIGHT_COS + egr_vlan_control_1[i] + 0x500, 0);
		bcm_cmic_reg(c, BLK_MMU, WRRWEIGHT_COS + egr_vlan_control_1[i] + 0x600, 0);
		bcm_cmic_reg(c, BLK_MMU, WRRWEIGHT_COS + egr_vlan_control_1[i] + 0x700, 0);
	}
	bcm_cmic_reg(c, BLK_MMU, TOTALDYNCELLSETLIMIT, 0x24c0);
	bcm_cmic_reg(c, BLK_MMU, TOTALDYNCELLRESETLIMIT, 0x24a8);
	bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT, 0x5e01798);
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 0x100 * i, 0x3000c);
		bcm_cmic_reg(c, BLK_MMU, CNGCOSPKTLIMIT0 + 0x100 * i, 0xdb);
		bcm_cmic_reg(c, BLK_MMU, CNGCOSPKTLIMIT1 + 0x100 * i, 0x124);
	}
	bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT, 0x50);
	bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT, 0xc);
	bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT, 0x2fff);
	for (int i = 0; i < 28; i++) {
		bcm_cmic_reg(c, BLK_MMU, DYNCELLLIMIT + 2 + i, 0x5e01798);
		for (int j = 0; j < 8; j++) {
			bcm_cmic_reg(c, BLK_MMU, LWMCOSCELLSETLIMIT + 2 + i + 0x100 * j, 0x3000c);
			bcm_cmic_reg(c, BLK_MMU, CNGCOSPKTLIMIT0 + 2 + i + 0x100 * j, 0xdb);
			bcm_cmic_reg(c, BLK_MMU, CNGCOSPKTLIMIT1 + 2 + i + 0x100 * j, 0x124);
		}
		bcm_cmic_reg(c, BLK_MMU, IBPCELLSETLIMIT + 2 + i, 0x50);
		bcm_cmic_reg(c, BLK_MMU, IBPPKTSETLIMIT + 2 + i, 0xc);
		bcm_cmic_reg(c, BLK_MMU, IBPDISCARDSETLIMIT + 2 + i, 0x2fff);
	}
	bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x1ded);
	for (unsigned int i = 0; i < ARRAY_SIZE(cpu_cos_map); i++) {
		BCM_CMIC_MEM(c, BLK_IPIPE, CPU_COS_MAP + 119 + i, 1 + 2 * i, 0, cpu_cos_map[i][0], 0xffffffff,
		             cpu_cos_map[i][1]);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(port_cos_map); i++)
		bcm_cmic_mem32(c, BLK_IPIPE, PORT_COS_MAP + 1 + i, port_cos_map[i]);
}

static void hr2_miim_scan_counters(struct bcm_cmic *c)
{
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_9_0, 0);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x200);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_11_8, 0x900);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_9_0, 0);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x300);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_11_8, 0x908);
	for (unsigned int i = 0; i < ARRAY_SIZE(cmic_miim_scan_ctrl); i++) {
		bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
		bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_9_0, 0);
		bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, cmic_miim_scan_ctrl[i][0]);
		bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_7_4, cmic_miim_scan_ctrl[i][1]);
	}
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_9_0, 0);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x3f8);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_3_0, 0x2000000);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_9_0, 0);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x3fc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_3_0, 0x2010000);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_19_10, 0);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x7fc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_11_8, 0xb0908);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_19_10, 0);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0xffc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_11_8, 0xc0b0908);
	for (unsigned int i = 0; i < ARRAY_SIZE(cmic_miim_scan_ctrl_2); i++) {
		bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
		bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_19_10, 0);
		bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, cmic_miim_scan_ctrl_2[i][0]);
		bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_15_12, cmic_miim_scan_ctrl_2[i][1]);
	}
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_19_10, 0);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x1fffc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_19_16, 0x12);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_19_10, 0);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x3fffc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_19_16, 0x1312);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_19_10, 0);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_19_16, 0x141312);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_19_10, 0);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_19_16, 0x15141312);
	for (unsigned int i = 0; i < ARRAY_SIZE(cmic_miim_scan_ctrl_3); i++) {
		bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
		bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_29_20, 0);
		bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_23_20, cmic_miim_scan_ctrl_3[i]);
	}
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_29_20, 0);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_27_24, 0x1a);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_29_20, 0);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_27_24, 0x1b1a);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_29_20, 0x40000);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x403fffc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_27_24, 0x11b1a);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_FAULT_LINK_STATUS, 1);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_29_20, 0x240000);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0xc03fffc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_27_24, 0x2011b1a);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_FAULT_LINK_STATUS + 1, 1);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_29_20, 0x1240000);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x1c03fffc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_31_28, 3);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_FAULT_LINK_STATUS + 2, 1);
	bcm_cmic_write(c, CMIC_MIIM_BUS_SEL_MAP_39_30, 1);
	bcm_cmic_write(c, CMIC_MIIM_INT_SEL_MAP_0, 0x5c03fffc);
	bcm_cmic_write(c, CMIC_MIIM_EXT_PHY_ADDR_MAP_31_28, 0x50003);
	bcm_cmic_write(c, CMIC_MIIM_SCAN_CTRL, 0x30000000);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_FAULT_LINK_STATUS, 1);
	bcm_cmic_reg(c, BLK_IPIPE, RDBGC0_SELECT, 0x400ad11);
	bcm_cmic_reg(c, BLK_IPIPE, RDBGC1_SELECT, 0x1000);
	bcm_cmic_reg(c, BLK_IPIPE, RDBGC2_SELECT, 0x4000);
	bcm_cmic_reg(c, BLK_EPIPE, TDBGC0_SELECT, 0x50);
	bcm_cmic_reg(c, BLK_EPIPE, TDBGC1_SELECT, 0xa0);
	bcm_cmic_reg(c, BLK_EPIPE, TDBGC2_SELECT, 0x40);
	bcm_cmic_reg(c, BLK_EPIPE, TDBGC3_SELECT, 0x40000);
	bcm_cmic_reg(c, BLK_EPIPE, TDBGC4_SELECT, 0x11);
	bcm_cmic_reg(c, BLK_EPIPE, TDBGC5_SELECT, 0x88);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 8; j++)
			bcm_cmic_reg(c, BLK_GPORT0 + i, GPORT_CNTMAXSIZE, 0x5ee);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs[i][1], miim_scan_counters_regs[i][0], 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 2, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 2, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 3, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 3, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 4, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 4, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 5, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 5, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 6, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 6, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 7, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 7, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 8, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 1, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 8, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 8, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 9, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 9, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 9, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 10, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 10, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 10, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 11, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 1, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 11, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 11, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 12, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 12, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 12, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 13, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 13, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 13, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 14, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 14, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 14, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 15, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 15, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 15, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 16, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 16, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 16, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 17, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 17, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 17, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 18, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 18, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 18, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 19, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 1, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 19, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 19, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 20, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 20, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 20, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 21, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 21, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 21, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 22, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 22, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 22, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 23, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 23, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 23, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 24, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 24, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 24, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 25, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 25, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 25, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_4); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_4[i][1], miim_scan_counters_regs_4[i][0] + 26, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_5); i++)
		bcm_cmic_reg64(c, BLK_XLPORT0, miim_scan_counters_regs_5[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 26, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 26, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_4); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_4[i][1], miim_scan_counters_regs_4[i][0] + 27, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_5); i++)
		bcm_cmic_reg64(c, BLK_XLPORT0, miim_scan_counters_regs_5[i] + 1, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 27, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 27, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_4); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_4[i][1], miim_scan_counters_regs_4[i][0] + 28, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_5); i++)
		bcm_cmic_reg64(c, BLK_XLPORT0, miim_scan_counters_regs_5[i] + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 28, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 28, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_4); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_4[i][1], miim_scan_counters_regs_4[i][0] + 29, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_5); i++)
		bcm_cmic_reg64(c, BLK_XLPORT1, miim_scan_counters_regs_5[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 29, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 29, 0);
}

static void hr2_storm_control(struct bcm_cmic *c)
{
	for (int i = 0; i < 28; i++) {
		bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_CONFIG + 2 + i, 0);
		bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_CONFIG + 2 + i, 0x1100);
	}
	bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_MAPPING, 0xe64);
	BCM_CMIC_MEM(c, BLK_EPIPE, EFP_TCAM, 0, 0, 0, 0, 0, 0, 0, 0xfffff000, 0xffffffff, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, VFP_TCAM, 0, 0, 0, 0, 0, 0, 0, 0xfffff000, 0xffffffff, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
}

static void hr2_tcp_ttl_tos(struct bcm_cmic *c)
{
	for (int i = 0; i < 63; i++)
		bcm_cmic_mem32(c, BLK_IPIPE, TCP_FN + 1 + i, 0x41 + 0x41 * i);
	for (int i = 0; i < 255; i++)
		bcm_cmic_mem32(c, BLK_IPIPE, TTL_FN + 1 + i, 0x101 + 0x101 * i);
	for (int i = 0; i < 255; i++)
		bcm_cmic_mem32(c, BLK_IPIPE, TOS_FN + 1 + i, 0x101 + 0x101 * i);
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_port); i++) {
		bcm_cmic_reg(c, BLK_EPIPE, EGR_PORT + egr_port[i][0], egr_port[i][1]);
		bcm_cmic_reg(c, BLK_EPIPE, EGR_PORT + 2 + 2 * i, 0x82000108 + 0x2000000 * i);
	}
	bcm_cmic_reg(c, BLK_EPIPE, EGR_PORT + 29, 0x9d000108);
	bcm_cmic_reg(c, BLK_EPIPE, EFP_METER_CONTROL, 0x20);
	bcm_cmic_reg(c, BLK_IPIPE, AUX_ARB_CONTROL_2, 0x407f863);
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_vlan_control_1); i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_RSPAN_VLAN_TAG + egr_vlan_control_1[i], 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_vlan_control_1); i++)
		bcm_cmic_reg(c, BLK_IPIPE, IMIRROR_BITMAP + egr_vlan_control_1[i], 0);
	bcm_cmic_mem32(c, BLK_IPIPE, L3_MTU_VALUES, 0x3fff);
}

static void hr2_l3_defip(struct bcm_cmic *c)
{
	for (int i = 0; i < 512; i++)
		BCM_CMIC_MEM(c, BLK_IPIPE, L3_DEFIP + i, 0, 0, 0xffffffc0, 0xffffffff, 0x7ff, 0);
}

static void hr2_l3_next_hop(struct bcm_cmic *c)
{
	BCM_CMIC_MEM(c, BLK_IPIPE, ING_L3_NEXT_HOP, 0x40000000, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, ING_L3_NEXT_HOP + 1, 0, 0x1ff);
	BCM_CMIC_MEM(c, BLK_EPIPE, EGR_L3_NEXT_HOP + 1, 0xffff8ff8, 0x7fffffff, 0);
	BCM_CMIC_MEM(c, BLK_EPIPE, EGR_L3_INTF + 0x1ff, 0x1000, 0xfffffffe, 0x1ffff);
	bcm_cmic_mem32(c, BLK_IPIPE, L3_MTU_VALUES + 0x1ff, 0x3fff);
}

static void hr2_ipmc_vlan_sel(struct bcm_cmic *c)
{
	for (int i = 0; i < 256; i++) {
		bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x5ded);
		bcm_cmic_reg(c, BLK_MMU, MISCCONFIG, 0x1ded);
	}
}

static void hr2_port_counters_mac(struct bcm_cmic *c)
{
	for (int i = 0; i < 28; i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_IPMC_CFG2 + 2 + i, 0);
	bcm_cmic_write(c, CMIC_BS0_CONFIG, 0x40);
	bcm_cmic_write(c, CMIC_BS1_CONFIG, 0x40);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_counters_mac_regs); i++)
		bcm_cmic_write(c, port_counters_mac_regs[i], 0);
	bcm_cmic_reg(c, BLK_IPIPE, IP1_INTR_ENABLE, 0xffbef0);
	bcm_cmic_reg(c, BLK_IPIPE, IARB_OAM_REFRESH_CONTROL, 3);
	for (int i = 0; i < 16; i++)
		bcm_cmic_mem32(c, BLK_IPIPE, ING_SERVICE_PRI_MAP + i, 8);
	bcm_cmic_reg(c, BLK_IPIPE, LMEP_COMMON, 4);
	BCM_CMIC_MEM(c, BLK_IPIPE, IPORT_TABLE, 0x1c00001, 0x2000010, 0x200000, 0x106e4000, 0x5f58d11, 0, 0x800008, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, PORT_TAB, 0x1c00001, 0x2000000, 0x200000, 0x106e4000, 0x5f58d11, 0, 0x800008, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 2, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 2, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 3, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 3, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 4, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 4, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 5, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 5, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 6, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 6, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 7, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 7, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 8, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i] + 1, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 8, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 8, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 9, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT0, miim_scan_counters_regs_3[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 9, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 9, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 10, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 10, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 10, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 11, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 1, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 11, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 11, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 12, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 12, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 12, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 13, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 13, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 13, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 14, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 14, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 14, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 15, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 15, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 15, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 16, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 16, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 16, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 17, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT1, miim_scan_counters_regs_3[i] + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 17, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 17, 0);
	bcm_cmic_miim(c, 0x014, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x014, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x014, 0x1c, 0xfc00);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x180205b);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_counters_mac_vals); i++) {
		bcm_cmic_miim(c, port_counters_mac_vals[i][0], port_counters_mac_vals[i][1],
		              port_counters_mac_vals[i][2]);
	}
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x40000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 18, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 18, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 18, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 18, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 18, 0);
	bcm_cmic_miim(c, 0x015, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x015, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x015, 0x1c, 0xfc00);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x180205b);
	bcm_cmic_miim(c, 0x015, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x015, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x015, 0, 0x1b40);
	bcm_cmic_miim(c, 0x015, 0x1c, 0xfc00);
	bcm_cmic_miim(c, 0x251, 0x1f, 0x8010);
	bcm_cmic_miim(c, 0x251, 0x18, 0xf8f1);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x80000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 1, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0x8010);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 19, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 19, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0x8010);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 1, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0x8010);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 19, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 1, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 19, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 19, 0);
	bcm_cmic_miim(c, 0x016, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x016, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x016, 0x1c, 0xfc00);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x180205b);
	bcm_cmic_miim(c, 0x016, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x016, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x016, 0, 0x1b40);
	bcm_cmic_miim(c, 0x016, 0x1c, 0xfc00);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 3);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x100000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 2, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 20, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 20, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 2, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 20, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 20, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 20, 0);
	bcm_cmic_miim(c, 0x017, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x017, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x017, 0x1c, 0xfc00);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x180205b);
	bcm_cmic_miim(c, 0x017, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x017, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x017, 0, 0x1b40);
	bcm_cmic_miim(c, 0x017, 0x1c, 0xfc00);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 3);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x200000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 3, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 21, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 21, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 3, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 21, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 21, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 21, 0);
	bcm_cmic_miim(c, 0x018, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x018, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x018, 0x1c, 0xfc00);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x180205b);
	bcm_cmic_miim(c, 0x018, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x018, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x018, 0, 0x1b40);
	bcm_cmic_miim(c, 0x018, 0x1c, 0xfc00);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 3);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x400000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 4, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 22, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 22, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 4, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 22, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 22, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 22, 0);
	bcm_cmic_miim(c, 0x019, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x019, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x019, 0x1c, 0xfc00);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x180205b);
	bcm_cmic_miim(c, 0x019, 0x1c, 0x7c00);
	bcm_cmic_miim(c, 0x019, 0x1c, 0xfc01);
	bcm_cmic_miim(c, 0x019, 0, 0x1b40);
	bcm_cmic_miim(c, 0x019, 0x1c, 0xfc00);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 3);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x180005b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x800000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 5, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 23, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 23, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 5, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 23, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 23, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 23, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 24, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 24, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 24, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_2); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_2[i][1], miim_scan_counters_regs_2[i][0] + 25, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_3); i++)
		bcm_cmic_reg(c, BLK_GPORT2, miim_scan_counters_regs_3[i] + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 25, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 25, 0);
	bcm_cmic_miim(c, 0x241, 0x1f, 0);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1001);
	bcm_cmic_miim(c, 0x241, 0x1f, 0);
	bcm_cmic_miim(c, 0x241, 9, 1);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1001);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL, 0x1ffff3c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1041);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL, 0x300000c806ULL);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 6);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1001);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 26, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 26, 0);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL, 0x1041);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_4); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_4[i][1], miim_scan_counters_regs_4[i][0] + 26, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_5); i++)
		bcm_cmic_reg64(c, BLK_XLPORT0, miim_scan_counters_regs_5[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 26, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 26, 0);
	bcm_cmic_miim(c, 0x242, 0x1f, 0);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1001);
	bcm_cmic_miim(c, 0x242, 0x1f, 0);
	bcm_cmic_miim(c, 0x242, 9, 1);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1001);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 1, 0x1ffff3c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1041);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 1, 0x300000c806ULL);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 5);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 1, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 1, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1001);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 27, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 27, 0);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 1, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 1, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 1, 0x1041);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_4); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_4[i][1], miim_scan_counters_regs_4[i][0] + 27, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_5); i++)
		bcm_cmic_reg64(c, BLK_XLPORT0, miim_scan_counters_regs_5[i] + 1, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 27, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 27, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_4); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_4[i][1], miim_scan_counters_regs_4[i][0] + 28, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_5); i++)
		bcm_cmic_reg64(c, BLK_XLPORT0, miim_scan_counters_regs_5[i] + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 28, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 28, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_4); i++)
		bcm_cmic_reg(c, miim_scan_counters_regs_4[i][1], miim_scan_counters_regs_4[i][0] + 29, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(miim_scan_counters_regs_5); i++)
		bcm_cmic_reg64(c, BLK_XLPORT1, miim_scan_counters_regs_5[i], 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT1 + 29, 0);
	bcm_cmic_reg(c, BLK_MMU, CNGDROPCOUNT0 + 29, 0);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303fffd);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303fff9);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 4);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 7, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 2, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 2, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 7, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG, 8);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGBUCKET, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG, 0x8000);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG, 0x8010);
	bcm_cmic_reg(c, BLK_IPIPE, CPU_CONTROL_1, 0x400);
	bcm_cmic_miim(c, 0x209, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x209, 0x15, 0);
	bcm_cmic_miim(c, 0x209, 0x1e, 1);
	bcm_cmic_miim(c, 0x209, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x209, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303fff1);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 8);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 6, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 3, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 3, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 6, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_miim(c, 0x208, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x208, 0x15, 0);
	bcm_cmic_miim(c, 0x208, 0x1e, 1);
	bcm_cmic_miim(c, 0x208, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x208, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303ffe1);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x10);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 5, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 4, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 4, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 5, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_miim(c, 0x207, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x207, 0x15, 0);
	bcm_cmic_miim(c, 0x207, 0x1e, 1);
	bcm_cmic_miim(c, 0x207, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x207, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303ffc1);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x20);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 4, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 5, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 5, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 4, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_miim(c, 0x206, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x206, 0x15, 0);
	bcm_cmic_miim(c, 0x206, 0x1e, 1);
	bcm_cmic_miim(c, 0x206, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x206, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303ff81);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x40);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 3, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 6, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 6, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 3, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_miim(c, 0x204, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x204, 0x15, 0);
	bcm_cmic_miim(c, 0x204, 0x1e, 1);
	bcm_cmic_miim(c, 0x204, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x204, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303ff01);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x80);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 2, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 7, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 7, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 2, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_miim(c, 0x203, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x203, 0x15, 0);
	bcm_cmic_miim(c, 0x203, 0x1e, 1);
	bcm_cmic_miim(c, 0x203, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x203, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303fe01);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x100);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 1, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 8, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 8, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL + 1, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_miim(c, 0x202, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x202, 0x15, 0);
	bcm_cmic_miim(c, 0x202, 0x1e, 1);
	bcm_cmic_miim(c, 0x202, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x202, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303fc01);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x200);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 9, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 9, 0);
	bcm_cmic_reg(c, BLK_GPORT0, FLUSH_CONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_miim(c, 0x201, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x201, 0x15, 0);
	bcm_cmic_miim(c, 0x201, 0x1e, 1);
	bcm_cmic_miim(c, 0x201, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x201, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303f801);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x400);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 10, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 10, 0);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG, 0x1802058);
	bcm_cmic_miim(c, 0x20b, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x20b, 0x15, 0);
	bcm_cmic_miim(c, 0x20b, 0x1e, 1);
	bcm_cmic_miim(c, 0x20b, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x20b, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303f001);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x800);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 1, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 11, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 11, 0);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 1, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 1, 0x1802058);
	bcm_cmic_miim(c, 0x20c, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x20c, 0x15, 0);
	bcm_cmic_miim(c, 0x20c, 0x1e, 1);
	bcm_cmic_miim(c, 0x20c, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x20c, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303e001);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x1000);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 2, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 12, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 12, 0);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 2, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 2, 0x1802058);
	bcm_cmic_miim(c, 0x20d, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x20d, 0x15, 0);
	bcm_cmic_miim(c, 0x20d, 0x1e, 1);
	bcm_cmic_miim(c, 0x20d, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x20d, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x3303c001);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x2000);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 3, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 13, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 13, 0);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 3, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 3, 0x1802058);
	bcm_cmic_miim(c, 0x20e, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x20e, 0x15, 0);
	bcm_cmic_miim(c, 0x20e, 0x1e, 1);
	bcm_cmic_miim(c, 0x20e, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x20e, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x33038001);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x4000);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 4, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 14, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 14, 0);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 4, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 4, 0x1802058);
	bcm_cmic_miim(c, 0x210, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x210, 0x15, 0);
	bcm_cmic_miim(c, 0x210, 0x1e, 1);
	bcm_cmic_miim(c, 0x210, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x210, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x33030001);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x8000);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 5, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 15, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 15, 0);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 5, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 5, 0x1802058);
	bcm_cmic_miim(c, 0x211, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x211, 0x15, 0);
	bcm_cmic_miim(c, 0x211, 0x1e, 1);
	bcm_cmic_miim(c, 0x211, 0x1e, 0x87);
}

static void hr2_port_enable(struct bcm_cmic *c)
{
	bcm_cmic_miim(c, 0x211, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x33020001);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x10000);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 6, 1);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_SLICE_MAP, 0x88fac2d0, 0xfac6);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 16, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 16, 0);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 6, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_miim(c, 0x212, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x212, 0x15, 0);
	bcm_cmic_miim(c, 0x212, 0x1e, 1);
	bcm_cmic_miim(c, 0x212, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x212, 0x1f, 0x8000);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x33000001);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x20000);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 7, 1);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 17, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 17, 0);
	bcm_cmic_reg(c, BLK_GPORT1, FLUSH_CONTROL + 7, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT1, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_miim(c, 0x213, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x213, 0x15, 0);
	bcm_cmic_miim(c, 0x213, 0x1e, 1);
	bcm_cmic_miim(c, 0x213, 0x1e, 0x87);
	bcm_cmic_miim(c, 0x213, 0x1f, 0x8000);
	BCM_CMIC_MIIM_SEQ(c, 0x014, phy_seq_12);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_EPIPE, EGR_CONFIG_1, 0x4003);
	for (int i = 0; i < 28; i++)
		bcm_cmic_reg(c, BLK_IPIPE, ING_EN_EFILTER_BITMAP_64, 0x3fffffff);
	for (int i = 0; i < 8; i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_VLAN_CONTROL_1 + 2 + i, 0x5001);
	for (unsigned int i = 0; i < ARRAY_SIZE(port_enable_phys); i++) {
		BCM_CMIC_MIIM_SEQ(c, 0x15 + i, phy_seq_12);
		bcm_cmic_miim(c, 0x251, 0x1f, port_enable_phys[i]);
		bcm_cmic_miim(c, 0x251, 0x1f, port_enable_phys[i]);
	}
	for (int i = 0; i < 20; i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_VLAN_CONTROL_1 + 10 + i, 0x5001);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x32000001);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x1000000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 6, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 24, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 24, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0, 0x140);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 6, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 6, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	BCM_CMIC_MIIM_SEQ(c, 0x01a, phy_seq_12);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x30000001);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x180205b);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0x2000000);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x11800158);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 7, 1);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0, 0x940);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 25, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 25, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0, 0x140);
	bcm_cmic_reg(c, BLK_GPORT2, FLUSH_CONTROL + 7, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x11802158);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1800058);
	bcm_cmic_reg(c, BLK_MMU, MMUFLUSHCONTROL, 0);
	bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + 7, 0x1802058);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_12);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x251, 0x1f, 0);
	bcm_cmic_miim(c, 0x241, 0x1f, 0);
	bcm_cmic_miim(c, 0x242, 0x1f, 0);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 0x20000001);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1001);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 2, 0x1ffff3c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1041);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 2, 0x300000c806ULL);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 2, 3);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_TXFIFO_CTRL + 2, 0);
	bcm_cmic_reg(c, BLK_XLPORT0, XLPORT_ENABLE_REG, 7);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1001);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 28, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 28, 0);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_TX_CTRL + 2, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 2, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_CTRL + 2, 0x1041);
	bcm_cmic_miim(c, 0x243, 0x1f, 0);
	bcm_cmic_reg(c, BLK_IPIPE, EPC_LINK_BMAP_64, 1);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1001);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_PAUSE_CTRL, 0x1ffff3c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1041);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_TX_CTRL, 0x300000c806ULL);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_TXFIFO_CTRL, 3);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_TXFIFO_CTRL, 0);
	bcm_cmic_reg(c, BLK_XLPORT1, XLPORT_ENABLE_REG, 1);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1001);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 29, 0);
	bcm_cmic_reg(c, BLK_MMU, EGRMETERINGCONFIG + 29, 0);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_TX_CTRL, 0x100000c802ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_PAUSE_CTRL, 0x1ffff7c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_CTRL, 0x1041);
	bcm_cmic_miim(c, 0x245, 0x1f, 0);
	bcm_cmic_reg(c, BLK_IPIPE, DOS_CONTROL, 0x280400);
	bcm_cmic_reg(c, BLK_IPIPE, DOS_CONTROL_2, 0x200);
}

static void hr2_protocol_pkt_masks(struct bcm_cmic *c)
{
	for (int i = 0; i < 15; i++) {
		BCM_CMIC_MIIM_SEQ(c, 0x01a, phy_seq_13);
		BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_13);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(protocol_pkt_control); i++) {
		bcm_cmic_reg(c, BLK_IPIPE, PROTOCOL_PKT_CONTROL + protocol_pkt_control[i], 4);
		bcm_cmic_reg(c, BLK_IPIPE, PROTOCOL_PKT_CONTROL + protocol_pkt_control[i], 5);
		bcm_cmic_reg(c, BLK_IPIPE, IGMP_MLD_PKT_CONTROL + protocol_pkt_control[i], 0x8000);
		bcm_cmic_reg(c, BLK_IPIPE, IGMP_MLD_PKT_CONTROL + protocol_pkt_control[i], 0x9000);
		bcm_cmic_reg(c, BLK_IPIPE, IGMP_MLD_PKT_CONTROL + protocol_pkt_control[i], 0x9040);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(protocol_pkt_control_2); i++) {
		bcm_cmic_reg(c, BLK_IPIPE, PROTOCOL_PKT_CONTROL + 2 + i, protocol_pkt_control_2[i]);
		bcm_cmic_reg(c, BLK_IPIPE, PROTOCOL_PKT_CONTROL + 2 + i, protocol_pkt_control_2[i] + 0x10);
	}
	bcm_cmic_reg(c, BLK_IPIPE, CPU_CONTROL_1, 0x800400);
	for (unsigned int i = 0; i < ARRAY_SIZE(ing_egrmskbmap_64); i++) {
		bcm_cmic_reg(c, BLK_IPIPE, ING_EGRMSKBMAP_64 + 2 + i, 0);
		bcm_cmic_reg(c, BLK_IPIPE, BCAST_BLOCK_MASK_64 + 2 + i, 0);
		bcm_cmic_reg(c, BLK_IPIPE, UNKNOWN_MCAST_BLOCK_MASK_64 + 2 + i, 1);
		bcm_cmic_reg(c, BLK_IPIPE, KNOWN_MCAST_BLOCK_MASK_64 + 2 + i, 0);
		bcm_cmic_reg(c, BLK_IPIPE, UNKNOWN_UCAST_BLOCK_MASK_64 + 2 + i, 1);
		bcm_cmic_reg(c, BLK_IPIPE, IGMP_MLD_PKT_CONTROL + 2 + i, ing_egrmskbmap_64[i]);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(igmp_mld_pkt_control); i++) {
		bcm_cmic_reg(c, BLK_IPIPE, IGMP_MLD_PKT_CONTROL + igmp_mld_pkt_control[i][0],
		             igmp_mld_pkt_control[i][1]);
	}
	BCM_CMIC_MEM(c, BLK_IPIPE, IFP_REDIRECTION_PROFILE, 0x3ffffffc, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 0x380, 0, 0, 0, 0x110, 0, 0, 0, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 0x380, 1, 0xffffffc0, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 0x380, 3, 0, 0, 0, 0, 0, 0, 0xfffff000, 0xffffffff, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 0x300, 0, 0, 0, 0x110, 0, 0, 0, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 0x300, 1, 0xffffffc0, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 0x300, 3, 0, 0, 0, 0, 0x3333ff00, 0, 0xfffff000, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xcfffffff, 0xfffffccc, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 0x280, 0, 0, 0, 0x110, 0, 0, 0, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 0x280, 1, 0xffffffc0, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 0x280, 3, 0, 0x7fc, 0, 0x8000000, 0x3fc, 0, 0xfffff000, 0xffffffff,
	             0x1fffff, 0, 0, 0xfffff020, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 0x200, 0x84249000, 0, 0, 0x110, 0x16800400, 0x1802d00, 0x2d, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 0x200, 3, 0xc0, 0x7f0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 0x200, 0x20003, 0, 0, 0, 0, 0, 0, 0xeffff000, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
}

static void hr2_l2_user_entry(struct bcm_cmic *c)
{
	for (int i = 0; i < 127; i++)
		BCM_CMIC_MEM(c, BLK_IPIPE, L2_USER_ENTRY + 1 + i, 0, 0, 0xffffffc0, 0xffffffff, 0x7ff, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, L2_USER_ENTRY, 0x84000005, 0x301, 0x80000080, 0xffc06030, 0x47fb, 4);
}

static void hr2_storm_fp(struct bcm_cmic *c)
{
	for (int i = 0; i < 5; i++) {
		BCM_CMIC_MIIM_SEQ(c, 0x01a, phy_seq_13);
		BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_13);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(storm_control_meter_config); i++) {
		bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_CONFIG + storm_control_meter_config[i][0], 0x100);
		BCM_CMIC_MEM(c, BLK_IPIPE, FP_STORM_CONTROL_METERS + storm_control_meter_config[i][1], 0, 0x200);
		bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_CONFIG + storm_control_meter_config[i][0], 0x2fa0);
		BCM_CMIC_MEM(c, BLK_IPIPE, FP_STORM_CONTROL_METERS + storm_control_meter_config[i][1] + 3, 0,
		             0xb6400200);
		bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_CONFIG + storm_control_meter_config[i][0], 0x42fa0);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(storm_control_meter_config_2); i++)
		bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_CONFIG + 29, storm_control_meter_config_2[i]);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_STORM_CONTROL_METERS + 116, 0, 0xb6400200);
	bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_CONFIG + 29, 0x42fa0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_STORM_CONTROL_METERS + 119, 0, 0xb6400200);
	bcm_cmic_reg(c, BLK_IPIPE, STORM_CONTROL_METER_CONFIG + 29, 0x42fa0);
	for (int i = 0; i < 6; i++) {
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + i, 0x11802158);
		bcm_cmic_reg(c, BLK_GPORT2, COMMAND_CONFIG + i, 0x11800158);
	}
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL, 0x1ffff1c000ULL);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_PAUSE_CTRL + 1, 0x1ffff1c000ULL);
	for (int i = 0; i < 8; i++) {
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x1802058);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x11802158);
		bcm_cmic_reg(c, BLK_GPORT0, COMMAND_CONFIG + 7 - i, 0x11800158);
	}
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_PAUSE_CTRL, 0x1ffff1c000ULL);
	BCM_CMIC_MIIM_SEQ(c, 0x01a, phy_seq_13);
	BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_13);
	bcm_cmic_reg(c, BLK_IPIPE, L2_AGE_TIMER, 0x1000e8);
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_vlan_control_1); i++)
		bcm_cmic_reg(c, BLK_MMU, XQCOSARBSEL + egr_vlan_control_1[i], 0);
}

static void hr2_dscp_table(struct bcm_cmic *c)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(dscp_table); i++)
		bcm_cmic_mem32(c, BLK_IPIPE, DSCP_TABLE + dscp_table[i][0], dscp_table[i][1]);
}

static void hr2_stg_vlan_fp(struct bcm_cmic *c)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(stg_vlan_fp_vals); i++) {
		bcm_cmic_miim(c, stg_vlan_fp_vals[i], 0x17, 0xf7e);
		bcm_cmic_miim(c, stg_vlan_fp_vals[i], 0x15, 0);
		bcm_cmic_miim(c, stg_vlan_fp_vals[i], 0x1e, 0x202);
		bcm_cmic_miim(c, stg_vlan_fp_vals[i], 0x1e, 0x87);
		bcm_cmic_miim(c, stg_vlan_fp_vals[i], 0x1f, 0x8000);
	}
	BCM_CMIC_MEM(c, BLK_IPIPE, STG_TAB + 1, 0xfffffff0, 0xfffffff);
	BCM_CMIC_MEM(c, BLK_EPIPE, EGR_VLAN_STG + 1, 0xfffffff0, 0xfffffff);
	for (int i = 0; i < 3; i++) {
		BCM_CMIC_MIIM_SEQ(c, 0x01a, phy_seq_13);
		BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_13);
	}
	for (int i = 0; i < 6; i++) {
		bcm_cmic_reg(c, BLK_GPORT2, FRM_LENGTH + i, 0x600);
		bcm_cmic_reg(c, BLK_IPIPE, EGR_MTU_SIZE + 18 + i, 0x604);
	}
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_MAX_SIZE, 0x604);
	bcm_cmic_reg(c, BLK_IPIPE, EGR_MTU_SIZE + 26, 0x604);
	bcm_cmic_reg64(c, BLK_XLPORT0, XLMAC_RX_MAX_SIZE + 1, 0x604);
	for (unsigned int i = 0; i < ARRAY_SIZE(egr_mtu_size); i++) {
		bcm_cmic_reg(c, BLK_IPIPE, EGR_MTU_SIZE + egr_mtu_size[i], 0x604);
		bcm_cmic_reg(c, BLK_GPORT0, FRM_LENGTH + 7 - i, 0x600);
	}
	bcm_cmic_reg(c, BLK_IPIPE, EGR_MTU_SIZE + 9, 0x604);
	bcm_cmic_reg64(c, BLK_XLPORT1, XLMAC_RX_MAX_SIZE, 0x604);
	bcm_cmic_reg(c, BLK_IPIPE, EGR_MTU_SIZE + 29, 0x604);
	bcm_cmic_miim(c, 0x01a, 0x17, 0xf7e);
	bcm_cmic_miim(c, 0x01b, 0x17, 0xf7e);
	bcm_cmic_mem32(c, BLK_IPIPE, VLAN_PROFILE_TAB, 0x3f001);
	BCM_CMIC_MEM(c, BLK_EPIPE, EGR_VLAN + 1, 0x5ffffffc, 0x1fffffff, 0x10);
	BCM_CMIC_MEM(c, BLK_IPIPE, VLAN_TAB + 1, 0x3ffffffd, 0xc0, 4, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(fp_port_field_sel); i++) {
		BCM_CMIC_MEM(c, BLK_IPIPE, FP_PORT_FIELD_SEL + fp_port_field_sel[i], 0x4350, 0x86a1888, 0, 0x2000000,
		             0x10000);
	}
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 128, 0, 0, 0, 0x110, 0, 0, 0, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 128, 1, 0xffffffc0, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 128, 3, 0, 0, 0, 0, 0, 0, 0xfffff000, 0xffffffff, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE, 0x9cfff000, 3, 0, 0x110, 0x16800000, 0x2d00, 0x2d, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM, 0x7ffffff9, 0xffffff80, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM, 3, 0, 0, 0, 0, 0x180c200, 0, 0xfffff000, 0xffffffff, 0xffffffff, 0xffffffff,
	             0x3ffffff, 0x3080000, 0xfffffc06, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 129, 0, 0, 0, 0x110, 0, 0, 0, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 129, 1, 0xffffffc0, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 129, 3, 0, 0, 0, 0, 0, 0, 0xfffff000, 0xffffffff, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 1, 0x10924000, 2, 0, 0x110, 0x16800000, 0x2d00, 0x2d, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 1, 3, 0xc0, 0x7f0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 1, 3, 0, 0, 0xa8758441, 0x8014, 0, 0, 0xfffff000, 0xffffffff, 0xffffffff,
	             0xd61107ff, 0xfe0052a1, 0xffffffff, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 130, 0, 0, 0, 0x110, 0, 0, 0, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 130, 1, 0xffffffc0, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 130, 3, 0, 0, 0, 0, 0, 0, 0xfffff000, 0xffffffff, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 2, 0, 0, 0, 0x110, 0x80000, 0x200010, 0, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 2, 0x7ffffff9, 0xffffff80, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 2, 3, 0, 0xffff0000, 0, 0, 0, 0, 0xfffff000, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 131, 0, 0, 0, 0x110, 0, 0, 0, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 131, 1, 0xffffffc0, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 131, 0x1640003, 0, 0, 0, 0, 0x400, 0, 0x9ffff000, 0xfffffff5, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xf01fffff, 0xffffffff, 0x3fffff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_POLICY_TABLE + 3, 0x9cfff000, 3, 0, 0x110, 0x16800400, 0x802d00, 0x2d, 0);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_GLOBAL_MASK_TCAM + 3, 0x7ffffff9, 0xffffff80, 0x7ff);
	BCM_CMIC_MEM(c, BLK_IPIPE, FP_TCAM + 3, 3, 0, 0, 0, 0, 0, 0, 0xe3fff000, 0xffffffff, 0xffffffff, 0xffffffff,
	             0xffffffff, 0xffffffff, 0xffffffff, 0x3fffff);
	bcm_cmic_miim(c, 0x209, 1, 0);
	BCM_CMIC_MIIM_SEQ(c, 0x01a, phy_seq_13);
	BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_13);
	for (int i = 0; i < 6; i++)
		BCM_CMIC_MIIM_SEQ(c, 0x14 + i, phy_seq_14);
	for (int i = 0; i < 7; i++) {
		BCM_CMIC_MEM(c, BLK_IPIPE, PORT_TAB + 2 + 4 * i, 0x1c33825, 0x2000000, 0x200010, 0x106e4000, 0x5f58d11,
		             0x40000000, 0x800028 + 0x41 * i, 0);
		BCM_CMIC_MEM(c, BLK_IPIPE, PORT_TAB + 3 + 4 * i, 0x1c33825, 0x2000000, 0x200010, 0x106e4000, 0x5f58d11,
		             0x80000000, 0x800038 + 0x41 * i, 0);
		BCM_CMIC_MEM(c, BLK_IPIPE, PORT_TAB + 4 + 4 * i, 0x1c33825, 0x2000000, 0x200010, 0x106e4000, 0x5f58d11,
		             0xc0000000, 0x800048 + 0x41 * i, 0);
		BCM_CMIC_MEM(c, BLK_IPIPE, PORT_TAB + 5 + 4 * i, 0x1c33825, 0x2000000, 0x200010, 0x106e4000, 0x5f58d11,
		             0, 0x800059 + 0x41 * i, 0);
	}
	BCM_CMIC_MEM(c, BLK_IPIPE, LPORT_TAB, 0x3801, 0, 0, 0, 0, 0, 8, 0);
}

static void hr2_egr_ipmc_cfg2(struct bcm_cmic *c)
{
	for (int i = 0; i < 28; i++)
		bcm_cmic_reg(c, BLK_EPIPE, EGR_IPMC_CFG2 + 2 + i, 0);
}

static void hr2_phy_final(struct bcm_cmic *c)
{
	for (int i = 0; i < 14; i++) {
		BCM_CMIC_MIIM_SEQ(c, 0x01a, phy_seq_13);
		BCM_CMIC_MIIM_SEQ(c, 0x01b, phy_seq_13);
	}
}

static int hr2_init(struct bcm_cmic *c)
{
	hr2_soft_reset(c);
	hr2_ipmc_vlan_sel_boot(c);
	hr2_port_map_ecc(c);
	hr2_eee_egr_enable(c);
	hr2_port_config_led(c);
	hr2_led_ram_tdm(c);
	hr2_mmu_limits(c);
	hr2_egr_pri_cng_map(c);
	hr2_vlan_tag_action(c);
	hr2_serdes_phy_init(c);
	hr2_port_mac_init(c);
	hr2_egr_vlan_control(c);
	hr2_system_config_table(c);
	hr2_meter_buckets(c);
	hr2_cos_weights_limits(c);
	hr2_miim_scan_counters(c);
	hr2_storm_control(c);
	hr2_tcp_ttl_tos(c);
	hr2_l3_defip(c);
	hr2_l3_next_hop(c);
	hr2_ipmc_vlan_sel(c);
	hr2_port_counters_mac(c);
	hr2_port_enable(c);
	hr2_protocol_pkt_masks(c);
	hr2_l2_user_entry(c);
	hr2_storm_fp(c);
	hr2_dscp_table(c);
	hr2_stg_vlan_fp(c);
	hr2_ipmc_vlan_sel(c);
	hr2_egr_ipmc_cfg2(c);
	hr2_phy_final(c);
	return bcm_cmic_done(c);
}

static int bcm56150_probe(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct bcm_cmic c = { .dev = &pdev->dev };

	/* the CMIC's GPIO words inside this window belong to the GPIO driver: map without claiming */
	if (!res)
		return dev_err_probe(&pdev->dev, -EINVAL, "no CMIC window\n");
	c.base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!c.base)
		return dev_err_probe(&pdev->dev, -ENOMEM, "CMIC window %pR not mapped\n", res);
	c.size = resource_size(res);
	return hr2_init(&c);
}

static const struct of_device_id bcm56150_of_match[] = {
	{ .compatible = "brcm,bcm56150-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm56150_of_match);

static struct platform_driver bcm56150_driver = {
	.probe = bcm56150_probe,
	.driver = {
		.name = "bcm56150-switch",
		.of_match_table = bcm56150_of_match,
	},
};
module_platform_driver(bcm56150_driver);

MODULE_DESCRIPTION("BCM56150 switch core init");
MODULE_LICENSE("GPL");
