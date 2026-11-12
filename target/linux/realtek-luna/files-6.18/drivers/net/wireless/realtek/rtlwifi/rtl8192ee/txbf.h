/* SPDX-License-Identifier: GPL-2.0 */
/* TIER: CHIP */
/* RTL8192E HT explicit TX beamformer (the AP sounds an HT beamformee and steers toward it).
 * Off unless rtl8192ee.txbf=1. The pure parts below need only u8/u16/u32/bool, so the host
 * test (dev/rtl9607c-test/rtl8192ee_txbf_test.c) runs them as they ship. Register facts are
 * the V2801RGW stock driver's, read from its kernel; frame layout is IEEE 802.11-2020 9.4.
 */
#ifndef __RTL92EE_TXBF_H__
#define __RTL92EE_TXBF_H__

#define RTL92EE_TXBF_SLOTS		2	/* the MAC holds two beamformee entries */
#define RTL92EE_HT_TXBF_RX_NDP		0x08	/* HT TxBF field bit 3: Receive NDP Capable */
#define RTL92EE_NDPA_FC			0x80e0	/* mgmt, Action No Ack, Order: HT Control follows */
#define RTL92EE_NDPA_LEN		32
#define RTL92EE_TXBF_FAIL_LIMIT		50	/* stock removes an entry after more than 50 failed soundings */

#define RTL92EE_REG_TXBF_CTRL		0x042c	/* 16 bit per slot: P_AID, BW of the V matrix to apply */
#define RTL92EE_REG_NDPA_OPT		0x045f	/* NDPA rate << 2 | bandwidth */
#define RTL92EE_REG_BFMEE_SEL		0x0714	/* 16 bit per slot: P_AID of the beamformee, enable */

struct rtl92ee_txbf_write {
	u16 addr;
	u8 width;	/* bytes: 1 or 2 */
	u16 val;
};

/* The 4-byte HT Transmit Beamforming Capabilities field of a (re)association request,
 * frame without FCS. False when it carries no HT Capabilities element. */
static inline bool rtl92ee_txbf_assoc_field(const u8 *frame, unsigned int len, u32 *field)
{
	unsigned int pos;

	if (len < 24)
		return false;
	pos = 24 + ((frame[0] & 0xf0) == 0x20 ? 10 : 4);
	while (pos + 2 <= len && pos + 2 + frame[pos + 1] <= len) {
		const u8 *ie = frame + pos + 2;

		if (frame[pos] == 45 && frame[pos + 1] >= 26) {
			*field = ie[21] | ie[22] << 8 | ie[23] << 16 | (u32)ie[24] << 24;
			return true;
		}
		pos += 2 + frame[pos + 1];
	}
	return false;
}

/* Arm a station whose HT TxBF field sets Receive NDP (bit 3 only), or any HT station when forced. */
static inline bool rtl92ee_txbf_arm(u32 field, bool forced)
{
	return (field & RTL92EE_HT_TXBF_RX_NDP) || forced;
}

/* Stock removes an entry once its consecutive failed soundings exceed the limit. */
static inline bool rtl92ee_txbf_given_up(u16 fails)
{
	return fails > RTL92EE_TXBF_FAIL_LIMIT;
}

/* Partial AID an AP gives a station: (AID + 32 * (BSSID[47:44] ^ BSSID[43:40])) mod 512. */
static inline u16 rtl92ee_txbf_p_aid(u16 aid, const u8 *bssid)
{
	return ((aid & 0x1ff) + 32 * ((bssid[5] >> 4) ^ (bssid[5] & 0xf))) & 0x1ff;
}

/* 0x45F: 24M once the weakest station is above 30 %, else 6M; a legacy rate sounds at 20 MHz. */
static inline u8 rtl92ee_txbf_ndpa_opt(long rssi_min_pct)
{
	return (rssi_min_pct > 30 ? 0x08 : 0x04) << 2;
}

/* TXBF_CTRL of a slot: BIT9/BIT10 apply the V matrix for 20/40 MHz once a sounding succeeded. */
static inline u16 rtl92ee_txbf_status(u8 slot, u16 p_aid, bool steering, bool bw40)
{
	u16 val = p_aid | (slot ? 0xd000 : 0);

	if (steering)
		val |= bw40 ? 0x0600 : 0x0200;
	return val;
}

static inline unsigned int rtl92ee_txbf_enter_regs(u8 slot, u16 p_aid, u8 r42f, u8 r717,
						   struct rtl92ee_txbf_write *w)
{
	if (slot == 0) {
		w[0] = (struct rtl92ee_txbf_write){ 0x042c, 2, p_aid };
		w[1] = (struct rtl92ee_txbf_write){ 0x042f, 1, r42f | 0xd0 };
		w[2] = (struct rtl92ee_txbf_write){ 0x0717, 1, (r717 & 0x03) | 0x60 };
		w[3] = (struct rtl92ee_txbf_write){ 0x0714, 2, p_aid | 0x0200 };
		return 4;
	}
	w[0] = (struct rtl92ee_txbf_write){ 0x042e, 2, p_aid | 0xd000 };
	w[1] = (struct rtl92ee_txbf_write){ 0x0716, 2, p_aid | 0xe200 };
	return 2;
}

static inline unsigned int rtl92ee_txbf_leave_regs(u8 slot, u16 r42e, u16 r716,
						   struct rtl92ee_txbf_write *w)
{
	if (slot == 0) {
		w[0] = (struct rtl92ee_txbf_write){ 0x042c, 2, 0 };
		w[1] = (struct rtl92ee_txbf_write){ 0x0714, 2, 0 };
	} else {
		w[0] = (struct rtl92ee_txbf_write){ 0x042e, 2, r42e & 0xf000 };
		w[1] = (struct rtl92ee_txbf_write){ 0x0716, 2, r716 & 0x0060 };
	}
	return 2;
}

/* HT NDP announcement: Action No Ack + HT Control (CSI/Steering = 3 steering, NDP
 * Announcement = 1) + a 4-byte vendor action header; the MAC sends the NDP after it. */
static inline unsigned int rtl92ee_txbf_build_ndpa(u8 *buf, const u8 *ra, const u8 *ta,
						   const u8 *bssid, bool bw40)
{
	static const u8 action[4] = { 0x7f, 0x00, 0xe0, 0x4c };
	u16 duration = 2 * 16 + 40 + (bw40 ? 87 : 180);
	u32 htc = 3u << 22 | 1u << 24;
	unsigned int i;

	for (i = 0; i < RTL92EE_NDPA_LEN; i++)
		buf[i] = 0;
	buf[0] = RTL92EE_NDPA_FC & 0xff;
	buf[1] = RTL92EE_NDPA_FC >> 8;
	buf[2] = duration & 0xff;
	buf[3] = duration >> 8;
	for (i = 0; i < 6; i++) {
		buf[4 + i] = ra[i];
		buf[10 + i] = ta[i];
		buf[16 + i] = bssid[i];
	}
	for (i = 0; i < 4; i++) {
		buf[24 + i] = (htc >> (8 * i)) & 0xff;
		buf[28 + i] = action[i];
	}
	return RTL92EE_NDPA_LEN;
}

struct ieee80211_hw;
struct ieee80211_vif;
struct ieee80211_sta;

extern int rtl92ee_txbf;
int rtl92ee_txbf_init(struct ieee80211_hw *hw);
void rtl92ee_txbf_deinit(struct ieee80211_hw *hw);
void rtl92ee_txbf_stop(struct ieee80211_hw *hw);
void rtl92ee_txbf_assoc_req(struct ieee80211_hw *hw, const u8 *frame, unsigned int len);
void rtl92ee_txbf_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta);
void rtl92ee_txbf_sta_remove(struct ieee80211_hw *hw, struct ieee80211_sta *sta);
void rtl92ee_txbf_report(struct ieee80211_hw *hw, const u8 *buf, u8 len);

#endif
