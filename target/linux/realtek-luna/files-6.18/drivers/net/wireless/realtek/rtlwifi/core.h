/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2009-2012  Realtek Corporation.*/

#ifndef __RTL_CORE_H__
#define __RTL_CORE_H__

#define RTL_SUPPORTED_FILTERS		\
	(FIF_ALLMULTI | FIF_CONTROL | \
	FIF_OTHER_BSS | \
	FIF_FCSFAIL | \
	FIF_BCN_PRBRESP_PROMISC)

#define DM_DIG_THRESH_HIGH		40
#define DM_DIG_THRESH_LOW		35
#define DM_FALSEALARM_THRESH_LOW	400
#define DM_FALSEALARM_THRESH_HIGH	1000

#define DM_DIG_MAX			0x3e
#define DM_DIG_MIN			0x1e
#define DM_DIG_MAX_AP			0x32
#define DM_DIG_BACKOFF_MAX		12
#define DM_DIG_BACKOFF_MIN		-4
#define DM_DIG_BACKOFF_DEFAULT		10

enum cck_packet_detection_threshold {
	CCK_PD_STAGE_LOWRSSI = 0,
	CCK_PD_STAGE_HIGHRSSI = 1,
	CCK_FA_STAGE_LOW = 2,
	CCK_FA_STAGE_HIGH = 3,
	CCK_PD_STAGE_MAX = 4,
};

enum dm_dig_ext_port_alg_e {
	DIG_EXT_PORT_STAGE_0 = 0,
	DIG_EXT_PORT_STAGE_1 = 1,
	DIG_EXT_PORT_STAGE_2 = 2,
	DIG_EXT_PORT_STAGE_3 = 3,
	DIG_EXT_PORT_STAGE_MAX = 4,
};

enum dm_dig_connect_e {
	DIG_STA_DISCONNECT,
	DIG_STA_CONNECT,
	DIG_STA_BEFORE_CONNECT,
	DIG_MULTISTA_DISCONNECT,
	DIG_MULTISTA_CONNECT,
	DIG_AP_DISCONNECT,
	DIG_AP_CONNECT,
	DIG_AP_ADD_STATION,
	DIG_CONNECT_MAX
};

extern const struct ieee80211_ops rtl_ops;

void rtl_init_sw_leds(struct ieee80211_hw *hw);
void rtl_lamp_register(struct ieee80211_hw *hw, const char *chip,
		       void (*set)(struct ieee80211_hw *hw, bool on));
void rtl_lamp_unregister(struct ieee80211_hw *hw);
void rtl_fw_cb(const struct firmware *firmware, void *context);
void rtl_wowlan_fw_cb(const struct firmware *firmware, void *context);
void rtl_addr_delay(u32 addr);
void rtl_rfreg_delay(struct ieee80211_hw *hw, enum radio_path rfpath, u32 addr,
		     u32 mask, u32 data);
bool rtl_cmd_send_packet(struct ieee80211_hw *hw, struct sk_buff *skb);
void rtl_tx_tasklet(struct tasklet_struct *t);
bool rtl_btc_status_false(void);
void rtl_dm_diginit(struct ieee80211_hw *hw, u32 cur_igval);
void rtl_update_beacon_work_callback(struct work_struct *work);
/* The HT NDP announcement rtl8192ee's beamformer builds: Action No Ack + Order (HT Control). */
#define RTL_FC_HT_NDPA	(IEEE80211_FTYPE_MGMT | 0x00e0 | IEEE80211_FCTL_ORDER)

static inline bool rtl_is_ht_ndpa(__le16 fc)
{
	return fc == cpu_to_le16(RTL_FC_HT_NDPA);
}

/* Association or reassociation request: their subtypes differ in bit 5 only, one compare. */
static inline bool rtl_is_any_assoc_req(__le16 fc)
{
	return (fc & cpu_to_le16(IEEE80211_FCTL_FTYPE |
				 (IEEE80211_FCTL_STYPE & ~IEEE80211_STYPE_REASSOC_REQ))) ==
	       cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ASSOC_REQ);
}

/* The BSSID our BSS's frames carry. An AP's is its own address: mac80211 never signals
 * BSS_CHANGED_BSSID to an AP, so mac->bssid stays zero there. mac_addr is vif->addr, taken
 * again by add_interface after every MAC change. */
static inline const u8 *rtl_rx_own_bssid(const struct rtl_mac *mac)
{
	return mac->opmode == NL80211_IFTYPE_AP ? mac->mac_addr : mac->bssid;
}

/* The BSSID register of an AP holds its own address, as stock programs it in every mode;
 * BSS_CHANGED_BSSID never comes to an AP. Called once mac_addr is set. */
static inline void rtl_set_own_bssid(struct ieee80211_hw *hw, const struct rtl_mac *mac)
{
	if (mac->opmode == NL80211_IFTYPE_AP)
		rtl_priv(hw)->cfg->ops->set_hw_reg(hw, HW_VAR_BSSID, (u8 *)rtl_rx_own_bssid(mac));
}

/* -> the frame is of our BSS and decoded clean: it feeds the signal statistics (per-station
 * PWDB, DIG, the firmware rate adaptation). *toself: addressed to this interface, by its
 * address, not the EFUSE's that a board's own MAC replaces. */
static inline bool rtl_rx_frame_ours(const struct rtl_mac *mac, const struct ieee80211_hdr *hdr,
				     const struct rtl_stats *pstatus, bool *toself)
{
	__le16 fc = hdr->frame_control;
	const u8 *bssid = ieee80211_has_tods(fc) ? hdr->addr1 :
			  ieee80211_has_fromds(fc) ? hdr->addr2 : hdr->addr3;
	bool ours = !ieee80211_is_ctl(fc) && ether_addr_equal(rtl_rx_own_bssid(mac), bssid) &&
		    !pstatus->hwerror && !pstatus->crc && !pstatus->icv;

	*toself = ours && ether_addr_equal(hdr->addr1, mac->mac_addr);
	return ours;
}

/* An RX descriptor rate that is an HT MCS: mac80211 maps rate_idx as an MCS only when told. */
static inline bool rtl_desc_rate_is_ht(u8 desc_rate)
{
	return desc_rate >= DESC_RATEMCS0 && desc_rate < DESC_RATEVHT1SS_MCS0;
}

/* rtlpriv->status bit beside RTL_STATUS_INTERFACE_START (wifi.h, bit 0) */
#define RTL_STATUS_HW_RESTART		1
void rtl_restart_hw(struct ieee80211_hw *hw);

#endif
