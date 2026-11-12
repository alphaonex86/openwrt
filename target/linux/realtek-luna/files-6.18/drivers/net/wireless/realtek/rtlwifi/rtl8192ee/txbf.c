// SPDX-License-Identifier: GPL-2.0
/* TIER: CHIP */
/* RTL8192E HT explicit TX beamformer: off by default; rtl8192ee.txbf=1 turns it on.
 *
 * A station is armed at association when its (re)association request's HT TxBF field says
 * it receives NDPs (mac80211 drops that field, so the RX path hands the request here first).
 * The latest field per station is kept, so mac80211's station replay after a restart re-arms.
 * Every 20 ms one armed station is sounded: an HT NDP announcement at MCS8, then the NDP the
 * MAC sends itself. The firmware reports the CSI outcome as C2H id 2; a success applies the
 * V matrix (TXBF_CTRL BW bits), a failure or no report clears it. After more than 50 failures
 * in a row the entry is removed: its slot's registers are cleared and the slot freed, as the
 * V2801RGW stock driver does (disassembly; it paces with its MAC timer, 0x15C = 640 at
 * 1/32 ms). Added here: the last entry leaving restores 0x42F, 0x717 and the RF/BB mode the
 * first arm changed (stock leaves them set).
 * Nothing runs per data packet: the hooks see association requests, station add/remove, the
 * C2H report and this driver's own NDPA only.
 */
#include "../wifi.h"
#include "../core.h"
#include "reg.h"
#include "txbf.h"

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

int rtl92ee_txbf;
module_param_named(txbf, rtl92ee_txbf, int, 0644);
MODULE_PARM_DESC(txbf, "Set to 1 to beamform toward HT stations that receive NDPs, read at each association (default 0)");

#ifdef CONFIG_RTLWIFI_DEBUG
static bool rtl92ee_txbf_force;
module_param_named(txbf_force, rtl92ee_txbf_force, bool, 0644);
MODULE_PARM_DESC(txbf_force, "Test only: with txbf=1, arm every HT station even without the NDP bit (default 0)");
#define txbf_forced() READ_ONCE(rtl92ee_txbf_force)
#else
#define txbf_forced() false
#endif

#define TXBF_PERIOD_MS	20

enum { TXBF_FREE, TXBF_ARMED, TXBF_SOUNDING, TXBF_STEERING };

struct rtl_txbf_slot {
	u8 mac[ETH_ALEN];
	u8 state;
	bool bw40;
	u16 p_aid;
	u16 fails;
	u32 ndpa_sent, ndpa_dropped, csi_ok, csi_failed, csi_missing;
};

struct rtl_txbf {
	spinlock_t lock;	/* slots, counters and their registers: timer, works, station ops */
	struct timer_list timer;
	struct work_struct giveup;	/* removes entries past the fail limit: rf_lock is not softirq-safe */
	struct ieee80211_hw *hw;
	struct rtl_txbf_slot slot[RTL92EE_TXBF_SLOTS];
	u8 cur;			/* the slot sounded last */
	u8 armed;
	struct {
		u8 mac[ETH_ALEN];
		u32 field;
	} seen[4];		/* HT TxBF fields of the latest (re)association requests */
	u8 seen_next;
	u8 r42f, r717, ra04;	/* before the first arm, restored after the last leave */
};

static void txbf_apply(struct rtl_priv *rtlpriv, const struct rtl92ee_txbf_write *w, unsigned int n)
{
	for (; n; n--, w++) {
		if (w->width == 1)
			rtl_write_byte(rtlpriv, w->addr, (u8)w->val);
		else
			rtl_write_word(rtlpriv, w->addr, w->val);
	}
}

/* RF mode-table entry of the RX mode with the TX IQ generator on (RF 0x32 0x77fc2), and the
 * BB two-stream steering setup; the other way, the values the PHY table and stock use. */
static void txbf_rf_mode(struct ieee80211_hw *hw, bool beamformer, u8 a04)
{
	enum radio_path path;

	for (path = RF90_PATH_A; path <= RF90_PATH_B; path++)
		rtl_set_rfreg(hw, path, RF_WE_LUT, BIT(19), 1);
	for (path = RF90_PATH_A; path <= RF90_PATH_B; path++) {
		rtl_set_rfreg(hw, path, 0x30, RFREG_OFFSET_MASK, 0x18000);
		rtl_set_rfreg(hw, path, 0x31, RFREG_OFFSET_MASK, 0x0000f);
		rtl_set_rfreg(hw, path, 0x32, RFREG_OFFSET_MASK, beamformer ? 0x77fc2 : 0x77f82);
	}
	for (path = RF90_PATH_A; path <= RF90_PATH_B; path++)
		rtl_set_rfreg(hw, path, RF_WE_LUT, BIT(19), 0);
	rtl_set_bbreg(hw, 0x90c, MASKDWORD, beamformer ? 0x83321333 : 0x81121313);
	rtl_set_bbreg(hw, 0xa04, 0xc0000000, a04);
}

static bool txbf_active(const struct rtl_txbf_slot *s)
{
	return s->state != TXBF_FREE && !rtl92ee_txbf_given_up(s->fails);
}

/* Under bf->lock: stock's leave for slot i; the last one restores what the first arm changed. */
static void txbf_leave(struct ieee80211_hw *hw, struct rtl_txbf *bf, u8 i, const char *why)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_txbf_slot *s = &bf->slot[i];
	struct rtl92ee_txbf_write w[2];
	unsigned int n;

	pr_info("txbf: slot %u %s %pM: %u NDPA sent, %u dropped, CSI %u ok, %u failed, %u unreported\n",
		i, why, s->mac, s->ndpa_sent, s->ndpa_dropped, s->csi_ok, s->csi_failed, s->csi_missing);
	s->state = TXBF_FREE;
	n = rtl92ee_txbf_leave_regs(i, rtl_read_word(rtlpriv, 0x42e), rtl_read_word(rtlpriv, 0x716), w);
	txbf_apply(rtlpriv, w, n);
	if (!--bf->armed) {
		rtl_write_byte(rtlpriv, 0x42f, bf->r42f);
		rtl_write_byte(rtlpriv, 0x717, bf->r717);
		txbf_rf_mode(hw, false, bf->ra04);
	}
}

static void txbf_giveup(struct work_struct *work)
{
	struct rtl_txbf *bf = container_of(work, struct rtl_txbf, giveup);
	u8 i;

	spin_lock_bh(&bf->lock);
	for (i = 0; i < RTL92EE_TXBF_SLOTS; i++)
		if (bf->slot[i].state != TXBF_FREE && !txbf_active(&bf->slot[i]))
			txbf_leave(bf->hw, bf, i, "dropped after more than 50 failed soundings of");
	spin_unlock_bh(&bf->lock);
}

static bool txbf_send_ndpa(struct ieee80211_hw *hw, const u8 *ra, bool bw40)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct ieee80211_vif *vif = rtlpriv->mac80211.vif;
	struct rtl_tcb_desc tcb_desc = {};
	struct ieee80211_tx_info *info;
	struct ieee80211_sta *sta;
	struct sk_buff *skb;
	bool sent = false;

	if (!vif)
		return false;
	skb = dev_alloc_skb(RTL92EE_NDPA_LEN + hw->extra_tx_headroom);
	if (!skb)
		return false;
	skb_reserve(skb, hw->extra_tx_headroom);
	rtl92ee_txbf_build_ndpa(skb_put(skb, RTL92EE_NDPA_LEN), ra, vif->addr, vif->addr, bw40);
	info = IEEE80211_SKB_CB(skb);
	info->band = hw->conf.chandef.chan->band;
	info->control.rates[0].idx = 0;	/* a driver rate: trx.c sets MCS8 for an NDPA */

	rcu_read_lock();
	sta = ieee80211_find_sta(vif, ra);
	if (sta)
		sent = !rtlpriv->intf_ops->adapter_tx(hw, sta, skb, &tcb_desc);
	else
		dev_kfree_skb_any(skb);
	rcu_read_unlock();
	return sent;
}

static void txbf_sound(struct timer_list *t)
{
	struct rtl_txbf *bf = timer_container_of(bf, t, timer);
	struct ieee80211_hw *hw = bf->hw;
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_txbf_slot *s = NULL;
	bool radio_on = !is_hal_stop(rtl_hal(rtlpriv)) && rtlpriv->psc.rfpwr_state == ERFON;
	bool again = false, giveup = false, bw40 = false;
	u8 mac[ETH_ALEN], i;

	spin_lock(&bf->lock);
	for (i = 1; i <= RTL92EE_TXBF_SLOTS && radio_on; i++) {
		u8 k = (bf->cur + i) % RTL92EE_TXBF_SLOTS;

		if (!txbf_active(&bf->slot[k]))
			continue;
		bf->cur = k;
		s = &bf->slot[k];
		break;
	}
	if (s && s->state == TXBF_SOUNDING) {	/* the previous sounding was never reported */
		s->csi_missing++;
		s->fails++;
	}
	if (s && txbf_active(s)) {
		s->state = TXBF_SOUNDING;
		rtl_write_word(rtlpriv, RTL92EE_REG_TXBF_CTRL + 2 * bf->cur,
			       rtl92ee_txbf_status(bf->cur, s->p_aid, false, s->bw40));
		rtl_write_byte(rtlpriv, RTL92EE_REG_NDPA_OPT,
			       rtl92ee_txbf_ndpa_opt(rtlpriv->dm.entry_min_undec_sm_pwdb));
		ether_addr_copy(mac, s->mac);
		bw40 = s->bw40;
	} else {
		s = NULL;
	}
	for (i = 0; i < RTL92EE_TXBF_SLOTS; i++) {
		again |= txbf_active(&bf->slot[i]);
		giveup |= bf->slot[i].state != TXBF_FREE && !txbf_active(&bf->slot[i]);
	}
	spin_unlock(&bf->lock);

	if (giveup)
		schedule_work(&bf->giveup);
	if (s) {
		bool sent = txbf_send_ndpa(hw, mac, bw40);

		spin_lock(&bf->lock);
		if (sent)
			s->ndpa_sent++;
		else if (!s->ndpa_dropped++)
			pr_warn("txbf: NDPA to %pM not queued (no skb, station or TX slot)\n", mac);
		spin_unlock(&bf->lock);
	}
	if (again)
		mod_timer(&bf->timer, jiffies + msecs_to_jiffies(TXBF_PERIOD_MS));
}

void rtl92ee_txbf_assoc_req(struct ieee80211_hw *hw, const u8 *frame, unsigned int len)
{
	struct rtl_txbf *bf = rtl_priv(hw)->txbf;
	u32 field = 0;
	u8 i;

	if (!READ_ONCE(rtl92ee_txbf) || !bf || len < 24)
		return;
	rtl92ee_txbf_assoc_field(frame, len, &field);
	spin_lock_bh(&bf->lock);
	for (i = 0; i < ARRAY_SIZE(bf->seen); i++)
		if (ether_addr_equal(bf->seen[i].mac, frame + 10))
			break;
	if (i == ARRAY_SIZE(bf->seen))
		i = bf->seen_next++ % ARRAY_SIZE(bf->seen);
	ether_addr_copy(bf->seen[i].mac, frame + 10);
	bf->seen[i].field = field;
	spin_unlock_bh(&bf->lock);
	pr_info("txbf: %pM association request, HT TxBF field 0x%08x\n", frame + 10, field);
}

void rtl92ee_txbf_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_txbf *bf = rtlpriv->txbf;
	struct rtl92ee_txbf_write w[4];
	struct rtl_txbf_slot *s;
	u32 field = 0;
	u8 i, n;

	if (!READ_ONCE(rtl92ee_txbf) || !bf || vif->type != NL80211_IFTYPE_AP ||
	    !sta->deflink.ht_cap.ht_supported || rtlpriv->phy.rf_type != RF_2T2R)
		return;
	spin_lock_bh(&bf->lock);
	for (i = 0; i < ARRAY_SIZE(bf->seen); i++)
		if (ether_addr_equal(bf->seen[i].mac, sta->addr))
			field = bf->seen[i].field;
	if (!rtl92ee_txbf_arm(field, txbf_forced())) {
		spin_unlock_bh(&bf->lock);
		return;
	}
	for (i = 0; i < RTL92EE_TXBF_SLOTS; i++)
		if (bf->slot[i].state != TXBF_FREE && ether_addr_equal(bf->slot[i].mac, sta->addr))
			break;
	if (i == RTL92EE_TXBF_SLOTS)
		for (i = 0; i < RTL92EE_TXBF_SLOTS; i++)
			if (bf->slot[i].state == TXBF_FREE)
				break;
	if (i == RTL92EE_TXBF_SLOTS) {
		spin_unlock_bh(&bf->lock);
		pr_info("txbf: %pM not armed: both beamformee slots are in use\n", sta->addr);
		return;
	}
	s = &bf->slot[i];
	if (s->state == TXBF_FREE && !bf->armed++) {
		bf->r42f = rtl_read_byte(rtlpriv, 0x42f);
		bf->r717 = rtl_read_byte(rtlpriv, 0x717);
		bf->ra04 = rtl_get_bbreg(hw, 0xa04, 0xc0000000);
		txbf_rf_mode(hw, true, 3);
	}
	rtl_write_dword(rtlpriv, 0xd80, 0x01081008);
	memset(s, 0, sizeof(*s));
	ether_addr_copy(s->mac, sta->addr);
	s->p_aid = rtl92ee_txbf_p_aid(sta->aid, vif->addr);
	s->bw40 = rtlpriv->phy.current_chan_bw == HT_CHANNEL_WIDTH_20_40 &&
		  sta->deflink.bandwidth >= IEEE80211_STA_RX_BW_40;
	s->state = TXBF_ARMED;
	n = rtl92ee_txbf_enter_regs(i, s->p_aid, rtl_read_byte(rtlpriv, 0x42f),
				    rtl_read_byte(rtlpriv, 0x717), w);
	txbf_apply(rtlpriv, w, n);
	if (!timer_pending(&bf->timer))
		mod_timer(&bf->timer, jiffies + msecs_to_jiffies(TXBF_PERIOD_MS));
	spin_unlock_bh(&bf->lock);
	pr_info("txbf: slot %u armed for %pM (AID %u, P_AID 0x%03x, HT TxBF 0x%08x%s)\n",
		i, sta->addr, sta->aid, s->p_aid, field,
		field & RTL92EE_HT_TXBF_RX_NDP ? "" : ", forced");
}

void rtl92ee_txbf_sta_remove(struct ieee80211_hw *hw, struct ieee80211_sta *sta)
{
	struct rtl_txbf *bf = rtl_priv(hw)->txbf;
	u8 i;

	if (!bf)
		return;
	spin_lock_bh(&bf->lock);
	for (i = 0; i < RTL92EE_TXBF_SLOTS; i++)
		if (bf->slot[i].state != TXBF_FREE && ether_addr_equal(bf->slot[i].mac, sta->addr))
			txbf_leave(hw, bf, i, "left by");
	spin_unlock_bh(&bf->lock);
}

/* C2H id 2: bit 0 of the first byte is whether the beamformee's CSI came back. */
void rtl92ee_txbf_report(struct ieee80211_hw *hw, const u8 *buf, u8 len)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_txbf *bf = rtlpriv->txbf;
	struct rtl_txbf_slot *s;
	bool ok;

	if (!bf || !len)
		return;
	ok = buf[0] & BIT(0);
	spin_lock_bh(&bf->lock);
	s = &bf->slot[bf->cur];
	if (s->state == TXBF_SOUNDING && txbf_active(s)) {	/* a late report never revives a given-up slot */
		s->state = ok ? TXBF_STEERING : TXBF_ARMED;
		if (ok) {
			s->fails = 0;
			s->csi_ok++;
		} else {
			s->fails++;
			s->csi_failed++;
		}
		rtl_write_word(rtlpriv, RTL92EE_REG_TXBF_CTRL + 2 * bf->cur,
			       rtl92ee_txbf_status(bf->cur, s->p_aid, ok, s->bw40));
	}
	spin_unlock_bh(&bf->lock);
}

/* The chip is going down (card disable, restart): mac80211 replays the stations after. */
void rtl92ee_txbf_stop(struct ieee80211_hw *hw)
{
	struct rtl_txbf *bf = rtl_priv(hw)->txbf;

	if (!bf)
		return;
	timer_delete_sync(&bf->timer);
	cancel_work_sync(&bf->giveup);
	spin_lock_bh(&bf->lock);
	memset(bf->slot, 0, sizeof(bf->slot));
	bf->armed = 0;
	spin_unlock_bh(&bf->lock);
}

int rtl92ee_txbf_init(struct ieee80211_hw *hw)
{
	struct rtl_txbf *bf = kzalloc(sizeof(*bf), GFP_KERNEL);

	BUILD_BUG_ON(RTL92EE_NDPA_FC != RTL_FC_HT_NDPA);
	if (!bf)
		return -ENOMEM;
	spin_lock_init(&bf->lock);
	timer_setup(&bf->timer, txbf_sound, 0);
	INIT_WORK(&bf->giveup, txbf_giveup);
	bf->hw = hw;
	rtl_priv(hw)->txbf = bf;
	return 0;
}

void rtl92ee_txbf_deinit(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);

	if (!rtlpriv->txbf)
		return;
	timer_delete_sync(&rtlpriv->txbf->timer);
	cancel_work_sync(&rtlpriv->txbf->giveup);
	kfree(rtlpriv->txbf);
	rtlpriv->txbf = NULL;
}
