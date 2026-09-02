/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cortina_ni_flowoffload_logic.h -- logic hoisted out of cortina-ni-flowoffload.c.
 *
 * Every function here was moved MECHANICALLY under one rule: it touches no
 * MMIO, calls no kernel service, and reads no file-scope state of the shell
 * it left. Operator, 2026-08-28: a port should be "una lista de registros
 * y tal vez algunos workaround", and that is only true once the LOGIC
 * exists in one place instead of once per board.
 */
#ifndef _CORTINA_NI_FLOWOFFLOAD_LOGIC_H
#define _CORTINA_NI_FLOWOFFLOAD_LOGIC_H

#include <linux/types.h>
#include <linux/bits.h>		/* BIT() - the punt verdict bits */
#include <linux/build_bug.h>	/* static_assert on the packed key */

void cn_l3e_bitrev_key(u32 *w, int n_words);
u32 cn_l3e_crc32(const u8 *p, size_t len);
u16 cn_l3e_crc16(const u8 *p, size_t len);
u32 cn_l3e_proc_parse_ip(const char *s);

/* the 92-byte packed SW key below = the CRC input (chip fact) */
#define CN_L3E_KEY_BYTES		92	/* packed key = CRC input */

/* ------------------------------------------------------------------ */
/* ★ HW HDR_I descriptor layout - the key the SWO engine ACTUALLY      */
/* hashes.  The engine does NOT hash our SW cn_l3e_key (the 92-byte    */
/* aal_hash_key_t); it hashes the 128-byte L3FE_HDR_I descriptor the   */
/* classify/parse stage builds for a packet.  So a flow's fields must  */
/* be packed into HDR_I bit positions before feeding the SWO - the     */
/* SW-tuple -> HDR_I conversion below (cn_l3e_build_hdri).             */
/*                                                                     */
/* Bit offsets are LSB-first within the 128-byte little-endian buffer, */
/* recovered TIER-1 from a single-bit SWO learn on the live engine     */
/* under the 5-tuple mask (each field's bits proven to move the CRC),  */
/* and CONFIRMED TIER-2 (2026-07-25) against the stock ca-ne.ko HDR_I  */
/* build aal_hash_crc_sw_hw_calc_check, which packs the same fields    */
/* into a 128-byte stack buffer: the port pair is one 32-bit window at */
/* buffer bit 74 (dport <<2 into the word at byte 9, sport <<18, and   */
/* an and-mask preserving everything outside bits 74..105), ip_da_0 at */
/* 233 / ip_sa_0 at 361 (the 16-byte stores at byte 29 and 45 with a   */
/* 1-bit pre-shift), ip_protocol <<4 into the word at byte 61 = 492,   */
/* and the two 1-bit flags <<16 / <<17 in that same word = 504 / 505.  */
/* These are the 9607F "07f" layout, which differs from the sibling    */
/* gen2 struct in the IP region (+24 at the DA, +20 after).  NOTE the  */
/* shipping binary also disagrees with the aal-77c HEADER at ip_ver /  */
/* ip_vld (the header's extra ip_mtu_en/ip_mtu_enc would put them at   */
/* 509/510): the BINARY is the product, so 504/505 stand - do not      */
/* "correct" them to the header's values.                              */
/* ------------------------------------------------------------------ */
#define CN_L3E_HDRI_BYTES		128
#define CN_L3E_HDRI_WORDS		(CN_L3E_HDRI_BYTES / 4)
/* 5-tuple + IP validity - each proven LIVE (moves the SWO CRC) on the real
 * engine, and each re-derived tier-2 from the stock HDR_I packer (above);
 * ip_ver/ip_vld sit at [504:505] in BOTH sources. */
#define CN_HDRI_L4_DP			74	/* dest L4 port, 16b */
#define CN_HDRI_L4_SP			90	/* src  L4 port, 16b */
#define CN_HDRI_IP_DA0			233	/* IPv4 DA / v6 DA LSW; 128b field [233:360] */
#define CN_HDRI_IP_SA0			361	/* IPv4 SA / v6 SA LSW; 128b field [361:488] */
#define CN_HDRI_IP_L4_TYPE		489	/* 3b; masked under mask 0, kept for other masks */
#define CN_HDRI_IP_PROTO		492	/* IP protocol, 8b */
#define CN_HDRI_IP_VER			504	/* 1b: 0 = IPv4 */
#define CN_HDRI_IP_VLD			505	/* 1b: 1 = has an IP header */
/* profile id stamp: HDR_I t2_ctrl (== the SW key's ctrl_set_id).  Position is
 * chip-cut dependent - a_cut(rev'A', ca_soc_data==0x41) [961:964], b_cut
 * [965:968] (tier-2 confirmed: the stock packer branches on that soc field and
 * inserts the 4-bit stamp at bit 1 vs bit 5 of the word at buffer byte 120);
 * BOTH are masked-out under the routed-flow mask (mask 0, board-
 * verified 2026-07-18), so this stamp does NOT affect a 5-tuple flow's CRC and
 * the cut choice is non-load-bearing here.  Placed at the a_cut offset,
 * mirroring stock aal_hash_crc_sw_hw_calc_check (hdr_i.t2_ctrl = ctrl_set_id).
 * (07f HDR_I has NO separate table_id field - table selection is t0/t1/t2_ctrl,
 * and the SW key's table_id is always mask-zeroed before the CRC.) */
#define CN_HDRI_T2_CTRL			961	/* 4b, a_cut */

/* ------------------------------------------------------------------ */
/* Flow key / action - packed to the engine's exact bit layout.        */
/* u64 bitfields, LSB-first on arm64: matches the on-DDR layout the    */
/* stock driver emits.  Only the fields our 5-tuple mask leaves live   */
/* need real values; everything the mask covers is zeroed before CRC.  */
/* ------------------------------------------------------------------ */

struct cn_l3e_key {
	/* L4 */
	u64 l4_chksum_zero	: 1;
	u64 tcp_flags		: 9;
	u64 l4_dport		: 16;
	u64 l4_sport		: 16;
	/* L3 */
	u64 l3_chksum_err	: 1;
	u64 spi			: 32;
	u64 spi_vld		: 3;
	u64 icmp_type		: 8;
	u64 icmp_vld		: 3;
	u64 ipv6_doh		: 1;
	u64 ipv6_rh		: 1;
	u64 ipv6_hbh		: 1;
	u64 ip_fragment		: 1;
	u64 ip_da_sa_equal	: 1;
	u64 ip_options		: 1;
	u64 ip_ttl		: 8;
	u64 ipv6_flow_lbl	: 20;
	u64 ip_da_0		: 32;	/* v4 DA or v6 DA LSW, host order */
	u64 ip_da_1		: 32;
	u64 ip_da_2		: 32;
	u64 ip_da_3		: 32;
	u64 ip_sa_0		: 32;	/* v4 SA or v6 SA LSW, host order */
	u64 ip_sa_1		: 32;
	u64 ip_sa_2		: 32;
	u64 ip_sa_3		: 32;
	u64 ip_l4_type		: 3;
	u64 ip_protocol		: 8;
	u64 ip_ecn		: 2;
	u64 ip_dscp		: 6;
	u64 ip_ver		: 1;
	u64 ip_vld		: 1;
	/* PPPoE */
	u64 ppp_proto_enc	: 4;
	u64 pppoe_session_id	: 16;
	u64 pppoe_code_enc	: 4;
	u64 pppoe_type		: 2;
	/* VLAN */
	u64 inner_dei		: 1;
	u64 inner_pcp		: 3;
	u64 inner_vid		: 12;
	u64 inner_tpid_enc	: 3;
	u64 top_dei		: 1;
	u64 top_pcp		: 3;
	u64 top_vid		: 12;
	u64 top_tpid_enc	: 3;
	u64 vlan_cnt		: 2;
	/* L2 format */
	u64 llc_type_enc	: 2;
	u64 llc_snap		: 2;
	u64 pktlen_rng_vec	: 4;
	u64 len_encoded		: 1;
	/* L2 */
	u64 ethertype_enc	: 6;
	u64 ethertype		: 16;
	u64 mac_sa_0		: 8;
	u64 mac_sa_1		: 8;
	u64 mac_sa_2		: 8;
	u64 mac_sa_3		: 8;
	u64 mac_sa_4		: 8;
	u64 mac_sa_5		: 8;
	u64 mac_da_rsvd		: 1;
	u64 mac_da_rng		: 1;
	u64 mac_da_ip_mc	: 1;
	u64 mac_da_an_sel	: 4;
	u64 mac_da_0		: 8;
	u64 mac_da_1		: 8;
	u64 mac_da_2		: 8;
	u64 mac_da_3		: 8;
	u64 mac_da_4		: 8;
	u64 mac_da_5		: 8;
	/* special packet */
	u64 spcl_pkt_hdr_mtch	: 8;
	u64 spcl_pkt_enc	: 6;
	/* metadata */
	u64 mdata		: 64;
	/* policer / cos */
	u64 qos_premark		: 1;
	u64 pol_grp_id		: 3;
	u64 pol_id		: 9;
	u64 cos			: 3;
	/* dest / source port */
	u64 mcgid		: 10;
	u64 mc			: 1;
	u64 mc_idx_vld		: 1;
	u64 orig_lspid		: 6;
	u64 lspid		: 6;
	/* hash control */
	u64 hkey_id		: 6;	/* mask-table index */
	u64 ctrl_set_id		: 4;	/* profile id (CRC input, then zeroed by its mask bit) */
	u64 table_id		: 4;
	u64 reserved		: 4;
} __packed;

static_assert(sizeof(struct cn_l3e_key) == CN_L3E_KEY_BYTES);

/* HDR_I packing - the divergence-A fix (see cn_l3e_build_hdri) */
void cn_l3e_hdri_set(u8 *h, unsigned int off, unsigned int width, u64 val);
void cn_l3e_build_hdri(const struct cn_l3e_key *k, int profile,
		       u32 words[CN_L3E_HDRI_WORDS]);

/* independent bit-numbered FIB readback (the anti-SID2QID instrument) */
u64 cn_fib_field(const void *fib, u32 bit, u32 width);

enum cn_pppoe_leg_verdict {
	CN_PPPOE_LEG_OK = 0,
	CN_PPPOE_LEG_MODE_OFF,		/* hw_pppoe=0: PPPoE stays in software */
	CN_PPPOE_LEG_NO_PUSH,		/* US rule cannot express the encap */
	CN_PPPOE_LEG_UNEXPECTED_PUSH,	/* push on the reply leg = un-RE'd model */
};

enum cn_pppoe_leg_verdict cn_pppoe_leg_check(bool pppoe_mode, bool ds_leg,
					     u16 rule_sid, u16 armed_sid);
bool cn_pppoe_shadow_stale(bool ds_leg, bool egress_is_lan,
			   u16 rule_sid, u16 armed_sid);

/* cn_pppoe_punt_classify() verdict bits */
#define CN_PPPOE_PUNT_SESSION		BIT(0)	/* it IS a 0x8864 session frame */
#define CN_PPPOE_PUNT_CTRL		BIT(1)	/* PPP control, no inner IPv4 */
#define CN_PPPOE_PUNT_SID_BAD		BIT(2)
#define CN_PPPOE_PUNT_LEN_BAD		BIT(3)	/* ★ the mangle signature */
#define CN_PPPOE_PUNT_TCP_BAD		BIT(4)	/* ★ the mangle signature */
#define CN_PPPOE_PUNT_SHORT		BIT(5)	/* truncated before a needed header */
#define CN_PPPOE_PUNT_DATA		BIT(6)	/* inner PPP proto is IPv4 = judged */
#define CN_PPPOE_PUNT_SHIFT8		BIT(7)	/* ★ malformed AND = an 8-byte insert */
#define CN_PPPOE_PUNT_DBLENC		BIT(8)	/* ★ malformed AND = a double encap */

/* what the classifier decoded, for the diagnostic line */
struct cn_pppoe_punt_info {
	u16	sid;
	u16	ppp_proto;
	u16	pppoe_len;
	u16	ip_len;
	u8	ip_ver;
	u8	ihl;
	u8	tcp_doff;
	u8	tcp_flags;
};

u32 cn_pppoe_punt_classify(const u8 *f, unsigned int len, u16 exp_sid,
			   struct cn_pppoe_punt_info *pi);

#endif /* _CORTINA_NI_FLOWOFFLOAD_LOGIC_H */
