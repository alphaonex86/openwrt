// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright(c) 2026  CatchChallenger project
 *
 * PCIe bus glue for the RTL8192XB (PCI [10ec:0192]).  The 8192XB uses the
 * same AX-generation HAXI/V1 PCIe access path as the RTL8852C, so the
 * rtw89_pci_info here mirrors the 8852C's (register facts, shared core
 * helpers).
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/workqueue.h>

#include "pci.h"
#include "reg.h"
#include "rtw8192xb.h"

static const struct rtw89_pci_bd_idx_addr rtw8192xb_bd_idx_addr_low_power = {
	.tx_bd_addrs = {R_AX_DRV_FW_HSK_0, R_AX_DRV_FW_HSK_1, R_AX_DRV_FW_HSK_2,
			R_AX_DRV_FW_HSK_3, 0, 0,
			0, 0, R_AX_DRV_FW_HSK_4,
			0, 0, 0,
			R_AX_DRV_FW_HSK_5},
	.rx_bd_addrs = {R_AX_DRV_FW_HSK_6, R_AX_DRV_FW_HSK_7},
};

static const struct rtw89_pci_info rtw8192xb_pci_info = {
	.gen_def		= &rtw89_pci_gen_ax,
	.isr_def		= &rtw89_pci_isr_ax,
	.txbd_trunc_mode	= MAC_AX_BD_TRUNC,
	.rxbd_trunc_mode	= MAC_AX_BD_TRUNC,
	.rxbd_mode		= MAC_AX_RXBD_PKT,
	.tag_mode		= MAC_AX_TAG_MULTI,
	.tx_burst		= MAC_AX_TX_BURST_V1_256B,
	.rx_burst		= MAC_AX_RX_BURST_V1_128B,
	.wd_dma_idle_intvl	= MAC_AX_WD_DMA_INTVL_256NS,
	.wd_dma_act_intvl	= MAC_AX_WD_DMA_INTVL_256NS,
	.multi_tag_num		= MAC_AX_TAG_NUM_8,
	.lbc_en			= MAC_AX_PCIE_ENABLE,
	.lbc_tmr		= MAC_AX_LBC_TMR_2MS,
	.autok_en		= MAC_AX_PCIE_DISABLE,
	.io_rcy_en		= MAC_AX_PCIE_ENABLE,
	.io_rcy_tmr		= MAC_AX_IO_RCY_ANA_TMR_6MS,
	.rx_ring_eq_is_full	= false,
	.check_rx_tag		= false,
	.no_rxbd_fs		= false,
	.group_bd_addr		= false,
	.rpp_fmt_size		= sizeof(struct rtw89_pci_rpp_fmt),

	.init_cfg_reg		= R_AX_HAXI_INIT_CFG1,
	.txhci_en_bit		= B_AX_TXHCI_EN_V1,
	.rxhci_en_bit		= B_AX_RXHCI_EN_V1,
	.rxbd_mode_bit		= B_AX_RXBD_MODE_V1,
	.exp_ctrl_reg		= R_AX_HAXI_EXP_CTRL,
	.max_tag_num_mask	= B_AX_MAX_TAG_NUM_V1_MASK,
	.rxbd_rwptr_clr_reg	= R_AX_RXBD_RWPTR_CLR_V1,
	.txbd_rwptr_clr2_reg	= R_AX_TXBD_RWPTR_CLR2_V1,
	.dma_io_stop		= {R_AX_HAXI_INIT_CFG1, B_AX_STOP_AXI_MST},
	.dma_stop1		= {R_AX_HAXI_DMA_STOP1, B_AX_TX_STOP1_MASK},
	.dma_stop2		= {R_AX_HAXI_DMA_STOP2, B_AX_TX_STOP2_ALL},
	.dma_busy1		= {R_AX_HAXI_DMA_BUSY1, DMA_BUSY1_CHECK},
	.dma_busy2_reg		= R_AX_HAXI_DMA_BUSY2,
	.dma_busy3_reg		= R_AX_HAXI_DMA_BUSY3,

	.rpwm_addr		= R_AX_PCIE_HRPWM_V1,
	.cpwm_addr		= R_AX_PCIE_CRPWM,
	.mit_addr		= R_AX_INT_MIT_RX_V1,
	.wp_sel_addr		= R_AX_WP_ADDR_H_SEL0_3,
	.tx_dma_ch_mask		= 0,
	.bd_idx_addr_low_power	= &rtw8192xb_bd_idx_addr_low_power,
	.dma_addr_set		= &rtw89_pci_ch_dma_addr_set_v1,
	.bd_ram_table		= &rtw89_bd_ram_table_dual,

	.ltr_set		= rtw89_pci_ltr_set_v1,
	.fill_txaddr_info	= rtw89_pci_fill_txaddr_info_v1,
	.parse_rpp		= rtw89_pci_parse_rpp,
	.config_intr_mask	= rtw89_pci_config_intr_mask_v1,
	.enable_intr		= rtw89_pci_enable_intr_v1,
	.disable_intr		= rtw89_pci_disable_intr_v1,
	.recognize_intrs	= rtw89_pci_recognize_intrs_v1,

	.ssid_quirks		= NULL,
};

static const struct rtw89_driver_info rtw89_8192xbe_info = {
	.chip = &rtw8192xb_chip_info,
	.variant = NULL,
	.quirks = NULL,
	.bus = {
		.pci = &rtw8192xb_pci_info,
	},
};

static const struct pci_device_id rtw89_8192xbe_id_table[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x0192),
		.driver_data = (kernel_ulong_t)&rtw89_8192xbe_info,
	},
	{},
};
MODULE_DEVICE_TABLE(pci, rtw89_8192xbe_id_table);

/* ★★★ THIS RADIO MUST NOT BE PROBED BEFORE ITS 5 GHz SIBLING EXISTS
 * (added 2026-09-08).  rtw8192xb_pwr_on_func() can put an IDLE RTL8852C into
 * the running state it needs -- but only if that chip has been probed at all,
 * and on this board it has not: the 8192XB's PCIe function is probed at 8.3 s
 * and the 8852C's at 9.5 s, so the boot probe failed -110 on EVERY boot while
 * a hand bind minutes later succeeded.
 *
 * -EPROBE_DEFER is the driver model's own answer to "my supplier is not here
 * yet", and it is retried for free whenever any driver binds -- which the
 * 8852C's own bind is.  Two things are added around it, and neither is
 * decoration:
 *
 *   1. IT IS BOUNDED IN TIME AND SAYS WHY IT STOPPED.  A board whose 8852C is
 *      fitted but whose driver never binds (built out, or its own probe
 *      failed) must not lose its 2.4 GHz radio for ever: after
 *      RTW8192XBE_SIBLING_WAIT_MS the probe goes ahead anyway and logs that it
 *      did.  Powering on without the sibling is exactly the shipped
 *      behaviour, so the worst case is never worse than today.
 *   2. IT POKES ITSELF.  Relying on some other driver to bind would make the
 *      retry depend on what else the board happens to contain, so a delayed
 *      work re-attaches the device once a second until it binds or the
 *      deadline passes.  A cap and a reported exit reason, not a spin.
 *
 * ★ AND A BOARD WITH NO 8852C AT ALL PAYS NOTHING: the deferral is armed only
 * when an RTL8852CE is actually present on the PCI bus, so a 2.4 GHz-only
 * product probes exactly as before.  The erratum belongs to the pairing, not
 * to the chip alone.
 *
 * ⚠ ONE 2.4 GHz RADIO PER BOARD is assumed by the single pending slot below;
 * a second device is simply not deferred (it takes the "no slot" path and
 * probes immediately), which is honest rather than silently wrong.
 */
#define RTW8192XBE_SIBLING_PCI_ID	0xc852	/* RTL8852CE, the 5 GHz half */
#define RTW8192XBE_SIBLING_WAIT_MS	30000
#define RTW8192XBE_SIBLING_POKE_MS	1000

static struct pci_dev *rtw8192xbe_pending;	/* holds a reference while set */
static unsigned long rtw8192xbe_deadline;

static void rtw8192xbe_poke(struct work_struct *work)
{
	struct pci_dev *pdev = READ_ONCE(rtw8192xbe_pending);
	int ret;

	if (!pdev)
		return;

	/* device_attach() is __must_check, and the value is worth reading: <0 is
	 * a real failure to even try, 0 means no driver matched (which cannot
	 * happen here, we ARE the driver), 1 means it bound.  A deferral just
	 * re-arms this work from probe, so nothing to do on 0/1.
	 */
	ret = device_attach(&pdev->dev);
	if (ret < 0)
		dev_warn(&pdev->dev,
			 "could not re-attempt the deferred probe (%d)\n", ret);
}
static DECLARE_DELAYED_WORK(rtw8192xbe_poke_work, rtw8192xbe_poke);

static void rtw8192xbe_pending_clear(void)
{
	struct pci_dev *pdev = rtw8192xbe_pending;

	if (!pdev)
		return;
	WRITE_ONCE(rtw8192xbe_pending, NULL);
	cancel_delayed_work(&rtw8192xbe_poke_work);
	pci_dev_put(pdev);
}

static bool rtw8192xbe_sibling_fitted(void)
{
	struct pci_dev *sibling;

	sibling = pci_get_device(PCI_VENDOR_ID_REALTEK,
				 RTW8192XBE_SIBLING_PCI_ID, NULL);
	if (!sibling)
		return false;
	pci_dev_put(sibling);

	return true;
}

static int rtw8192xbe_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;

	if (!rtw89_core_sibling_present(RTL8852C) && rtw8192xbe_sibling_fitted()) {
		if (!rtw8192xbe_pending) {
			rtw8192xbe_pending = pci_dev_get(pdev);
			rtw8192xbe_deadline = jiffies +
				msecs_to_jiffies(RTW8192XBE_SIBLING_WAIT_MS);
		}
		if (pdev == rtw8192xbe_pending) {
			if (time_before(jiffies, rtw8192xbe_deadline)) {
				dev_info(&pdev->dev,
					 "deferring: the 5 GHz RTL8852C must be probed before this radio's MAC can power on\n");
				schedule_delayed_work(&rtw8192xbe_poke_work,
						      msecs_to_jiffies(RTW8192XBE_SIBLING_POKE_MS));
				return -EPROBE_DEFER;
			}
			dev_warn(&pdev->dev,
				 "the 5 GHz RTL8852C did not register within %u ms -- probing without it\n",
				 RTW8192XBE_SIBLING_WAIT_MS);
		}
	}

	ret = rtw89_pci_probe(pdev, id);
	if (pdev == rtw8192xbe_pending)
		rtw8192xbe_pending_clear();

	return ret;
}

static struct pci_driver rtw89_8192xbe_driver = {
	.name		= "rtw89_8192xbe",
	.id_table	= rtw89_8192xbe_id_table,
	.probe		= rtw8192xbe_probe,
	.remove		= rtw89_pci_remove,
	.driver.pm	= &rtw89_pm_ops,
	.err_handler    = &rtw89_pci_err_handler,
};

static int __init rtw89_8192xbe_init(void)
{
	return pci_register_driver(&rtw89_8192xbe_driver);
}
module_init(rtw89_8192xbe_init);

static void __exit rtw89_8192xbe_exit(void)
{
	pci_unregister_driver(&rtw89_8192xbe_driver);
	cancel_delayed_work_sync(&rtw8192xbe_poke_work);
	rtw8192xbe_pending_clear();
}
module_exit(rtw89_8192xbe_exit);

MODULE_AUTHOR("CatchChallenger project");
MODULE_DESCRIPTION("Realtek 802.11ax wireless 8192XBE driver");
MODULE_LICENSE("Dual BSD/GPL");
