/* SPDX-License-Identifier: GPL-2.0 */
/* The Luna GPON<->NIC contract: what luna_gpon.c and ...
 * dev/MEASURED-luna_gpon_nic.h.md sec 1. */
#ifndef _LUNA_GPON_NIC_H
#define _LUNA_GPON_NIC_H

struct omci_onu;
/* @uni_admin_set: drive ONE Ethernet UNI's administrative ...
 * dev/MEASURED-luna_gpon_nic.h.md sec 2. */
int luna_omci_attach(struct omci_onu *onu, void *cookie, u8 seed,
		    int (*tx)(void *, const u8 *, unsigned int),
		    void (*tx_fence)(void *),
		    int (*uni_admin_set)(void *, unsigned int, bool));

/* Ask the common owner to drive whatever the model has accepted and not yet
 * had applied.  Safe from any context: it only schedules. */
void luna_uni_apply_kick(void);
void luna_omci_detach(void *cookie);
int luna_omci_enqueue(void *cookie, const u8 *msg, unsigned int len);
void luna_omci_set_sn(const u8 sn[8]);
void luna_omci_set_optical(u16 rx, u16 tx);
void luna_omci_rx_errors(u32 *bad_mic, u32 *runt);
void luna_omci_report_oper_up(void);
bool luna_gpon_data_ready(void);
void luna_gpon_nic_reset_begin(void);
void luna_gpon_nic_reset_end(void);

/* Arm the GMAC OMCI trap for downstream stream-id @sid (the OMCC). Provided by
 * rtl9602c_eth.c; called from the GPON Configure_Port-ID handler. */
void rtl9602c_eth_set_omci_sid(unsigned int sid);

/* Provision the ONU identity (G.984.3 ONU-SN: 4 ASCII vendor + 4 serial bytes)
 * so the eth driver's OMCI responder reports an ONU-G Vendor-ID/Serial matching
 * the PLOAM Serial_Number the OLT ranged. Provided by rtl9602c_eth.c. */
void rtl9602c_eth_set_omci_identity(const u8 *sn8);

/* DS OMCI frames that reached the CPU NIC ring (defined in ...
 * dev/MEASURED-luna_gpon_nic.h.md sec 3. */
#define GPON_OMCI_RX_UNAVAIL	0xffffffffu
u32 rtl9602c_eth_omci_rx_count(void);

/* gpon0 (WAN) RX packet count, used by the GPON O5 ... -- dev/MEASURED-luna_gpon_nic.h.md sec 4. */
u32 rtl9602c_eth_wan_rx_count(void);

/* US-OMCI TX-ring reclaim cursor ("dirty"): count of OMCC descriptors the HW has
 * consumed (OWN cleared). Non-zero => the OMCC TX ring is being fetched. Defined
 * in rtl9602c_eth.c; surfaced for the periodic O5 serial diagnostic. */
u32 rtl9602c_eth_omci_tx_dirty(void);

/* US-OMCI responses the shell's TX ring REFUSED. Read beside ...
 * dev/MEASURED-luna_gpon_nic.h.md sec 5. */
#if IS_ENABLED(CONFIG_LUNA_ETH)
u32 rtl9602c_eth_omci_tx_dropped(void);
#else
static inline u32 rtl9602c_eth_omci_tx_dropped(void) { return 0; }
#endif

/* OLT-INDEPENDENT US-OMCI datapath self-test: inject a synthetic OMCI frame
 * through the US-OMCI TX path so RX_SID_GOOD_CNT_US[4] can be checked at O5
 * without the OLT sending DS OMCI. Defined in rtl9602c_eth.c. */
void rtl9602c_eth_omci_selftest(void);

/* Full PON US/DS-NIC + PBO bring-up (defined in luna_gpon.c). Non-__init: also
 * re-run from rtl9602c_eth_open() after the GMAC IP-block reset so the US-NIC RX
 * engine re-latches against the freshly-reset GMAC (stock order: GMAC reset -> NIC). */
void gpon_pbo_init(void);

/* Live ANI-G (ME 263) optical levels for the OMCI responder, ...
 * dev/MEASURED-luna_gpon_nic.h.md sec 9. */
void gpon_anig_optical_omci(s16 *rx_level, s16 *tx_level);

/* Faithful port of the stock SDK rtk_all_module_init() GPON datapath bring-up,
 * run on the quiescent switch in the eth reset path (after the GMAC reset + swcore
 * resync, before the GMAC is programmed/armed). Defined in luna_gpon.c. */
void rtl9602c_datapath_tables_init(void);

/* WAN data-GEM datapath. GPON_DATA_FLOW = the internal ...
 * dev/MEASURED-luna_gpon_nic.h.md sec 6. */
#define GPON_DATA_FLOW		1
#define GPON_DATA_GEM_DEFAULT	193u

/* The OLT also provisions a MULTICAST/broadcast GEM (OMCI ME268 inst=1 Port-ID=0x0fff,
 * paired with ME281 Multicast GEM IWTP). Broadcast DS (e.g. the DHCP OFFER the server
 * can only broadcast to an IP-less client) may ride this GEM, not the unicast data GEM.
 * Wire it to its own internal DS flow so broadcast DS de-encapsulates to the CPU/gpon0. */
#define GPON_MCAST_FLOW	2
#define GPON_MCAST_GEM	0xfffu

/* Timer-owned data installation consumes the accepted common OMCI binding.
 * The compatibility hint has no authorization or programming effect. */
int gpon_install_data_gem(void);
void gpon_omci_note_gem_create(u16 port_id);

/* The OMCC GEM Port-ID currently installed (0 = none yet).  An INPUT to the
 * core's data-GEM decision: the geometry is the shell's, the decision is not. */
u16 gpon_omcc_gem(void);

/* Publish the cached live DDM optical levels into ME 263 ...
 * dev/MEASURED-luna_gpon_nic.h.md sec 7. */
#if IS_ENABLED(CONFIG_RTL9602C_ETH) || IS_ENABLED(CONFIG_LUNA_ETH)
void rtl9602c_eth_omci_set_optical(s16 rx_level, s16 tx_level);
#else
static inline void rtl9602c_eth_omci_set_optical(s16 rx_level, s16 tx_level)
{
}
#endif

/* Emit an OMCI Attribute-Value-Change reporting the HGU WAN-egress (VEIP ME329) operational,
 * so the OLT un-gates downstream user-data forwarding. The OLT never polls the data MEs after
 * creating them; it waits for this AVC. Defined in rtl9602c_eth.c; called from the GPON FSM a
 * few seconds after O5 (config-apply done). */
void rtl9602c_eth_omci_report_oper_up(void);


/* The ONU serial number, owned by the PLOAM layer that ...
 * dev/MEASURED-luna_gpon_nic.h.md sec 8. */
void gpon_onu_sn(u8 out[8]);

#endif /* _LUNA_GPON_NIC_H */
