/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIER: CORE (prefix gpon_) — decides, never touches hardware, and compiles for
 * MIPS-BE, ARM64-LE and x86.  Canonical tier rule: see "THE THREE TIERS" in
 * gpon_common.h (this directory).
 *
 * gpon_omci_vlan.h — the VLAN / classification half of the OMCI WAN service
 * model: G.988 ME 171, 78, 84, 79, 130, 280 and 281.
 *
 * ★★★ WHY IT EXISTS.  The GEM Port-ID and the Alloc-ID arrive over OMCI and
 *     the install is gated on them; everything ABOVE the GEM came from uci.
 *     The service model the OLT expresses — which VLAN, which admit list,
 *     which per-protocol filter, which priority maps to which GEM — was not
 *     modelled at all, and omci_store_create() ACKs any class, so a Create of
 *     171/84/130 completed the burst and was silently ignored.  That works on
 *     an OLT that also accepts untagged IPoE, and it is the whole
 *     compatibility exposure on one that does not.
 *
 * ★★ THE ROW LAYOUT IS STOCK'S OWN, NOT A READING OF THE SPEC.  The X111W's
 *    /lib/omci/mib_ExtVlanTagOperCfgData.so carries debug_ExtVlan() and
 *    ExtVlanTagOperCfgDataDumpMib(), and the second one decomposes the 16-byte
 *    ME 171 table row field by field with its own shifts and masks.  Read off
 *    that disassembly (tier 2), and it AGREES with G.988 Table 9.3.13 — two
 *    independent sources, which is this project's bar for believing a value.
 *
 * ★ THE TRANSLATION IS PURE, WHICH IS THE POINT.  Stock's own ME 171
 *   translators (omci_SetExtValnClassifyRule, omci_SetClassifyUsRule/Act,
 *   omci_SetClassifyDsRule/Act, omci_SetDsTagActByUsFilter,
 *   omci_SetUsRuleInvert2Ds in pf_rg.ko) make NO SDK call at all — the vendor
 *   had drawn this core/family boundary five years before naming it `_dm_`.
 *   So this file is x86-fuzzable model code, not driver code, and its host
 *   coverage is dev/rtl9607c-test/gpon_omci_vlan_test.c.
 *
 * ★ HEADER-ONLY, static inline, no .c and no Makefile line — the same shape
 *   gpon_data_plan.h already uses in this directory.  ⚠ THE CONSEQUENCE MUST
 *   BE SAID OUT LOUD: the strict-subset host-build gate walks a list of .c
 *   files and does NOT reach a header, so this file's host coverage IS
 *   dev/rtl9607c-test/gpon_omci_vlan_test.c — that binary compiles the REAL
 *   header on x86 and is therefore the purity proof.  Deleting it would
 *   silently delete one.
 *
 * ⚠ WHAT THIS FILE DOES NOT DO, SAID FIRST: it does not install anything.  It
 *   DECIDES and the family installs, through struct gpon_vlan_ops.  A decision
 *   nobody installs is COUNTED (gpon_vlan_owed()), never dropped — the silence
 *   this file exists to end must not come back one layer up.
 */
#ifndef GPON_OMCI_VLAN_H
#define GPON_OMCI_VLAN_H

#include <linux/types.h>

/* ---------------------------------------------------------------------------
 * ME 171 — the received-frame VLAN tagging operation table row
 * ------------------------------------------------------------------------- */

/* One row is 16 octets = four 32-bit words, in this order.  Stock's own
 * debug_ExtVlan() names them: [OuterFilter] [InnerFilter] [OuterTreate]
 * [InnerTreate], four octets printed per line. */
#define GPON_EXT_VLAN_ROW_LEN	16

/* The RAW row, field by field, exactly as the wire carries it — no meaning
 * applied yet.  Keeping the raw values is what lets a sentinel be recognised
 * later instead of being flattened at decode time into a value that cannot be
 * told from a real one. */
struct gpon_ext_vlan_row {
	/* filter, outer tag then inner tag */
	u16	f_out_vid;	/* 13 bits */
	u16	f_in_vid;	/* 13 bits */
	u8	f_out_pri;	/* 4 bits */
	u8	f_in_pri;	/* 4 bits */
	u8	f_out_tpid;	/* 3 bits: TPID/DE treatment of the outer tag */
	u8	f_in_tpid;	/* 3 bits */
	u8	f_ethertype;	/* 4 bits, inner-filter word only */
	/* treatment */
	u16	t_out_vid;	/* 13 bits */
	u16	t_in_vid;	/* 13 bits */
	u8	t_remove;	/* 2 bits: tags to strip before tagging */
	u8	t_out_pri;	/* 4 bits */
	u8	t_in_pri;	/* 4 bits */
	u8	t_out_tpid;	/* 3 bits */
	u8	t_in_tpid;	/* 3 bits */
};

/*
 * THE SENTINELS.  Every one of these is a value one past the legal range of
 * its field, which is how G.988 says "no value here" without a second field,
 * and it is what stock's own translators test.
 *
 * ⚠ A SENTINEL IS NOT A VALUE, and collapsing the two is the whole class of
 *   bug this model exists to avoid: priority 15 is not "priority fifteen", it
 *   is "this tag is not present / do not add one", and an install that wrote
 *   15 into a priority field would tag every frame with a priority no
 *   subscriber asked for.
 */
#define GPON_VLAN_PRI_ANY	8	/* filter: do not filter on priority.
					 * Stock's own OMCI_PRI_FILTER_IGNORE */
#define GPON_VLAN_PRI_DEFAULT	14	/* filter: part of the default-rule
					 * signature -- see _is_default() */
#define GPON_VLAN_PRI_NONE	15	/* filter: this tag must be ABSENT
					 * (stock emits NO_TAG for it) */
#define GPON_VLAN_VID_ANY	4096	/* filter: do not filter on VID.
					 * Stock's own OMCI_VID_FILTER_IGNORE */
#define GPON_VLAN_ETYPE_ANY	0	/* filter: do not filter on EtherType */

/* The TPID/DE code, 3 bits.  Stock splits it with one `sltiu code,5`: 0..4 are
 * an ordinary 0x8100 tag, 5..7 mean "use this ME's INPUT TPID attribute", and
 * when that attribute is not 0x8100 the tag is taken as a service tag rather
 * than a customer one.  That is why gpon_ext_vlan_filter() needs the ME's own
 * input TPID as an argument: the row alone does not say which tag it is. */
#define GPON_VLAN_TPID_INPUT_MIN	5
#define GPON_VLAN_TPID_C		0x8100

/* Tags-to-remove == 3 is not a count.  It is DISCARD THE FRAME (stock's own
 * OMCI_EXTVLAN_REMOVE_TAG_DISCARD), and the decoder diverts on it before it
 * emits any filter at all. */
#define GPON_VLAN_REMOVE_DISCARD	3

/* The EtherType filter codes, and the wire values they name.  Read out of the
 * jump table stock's omci_SetExtValnClassifyRule() indexes (.rodata+44), so
 * these are the DEVICE's codes; codes 1, 3 and 4 also match G.988 Table
 * 9.3.13, and the vendor splits PPPoE into a discovery code and a session one
 * rather than one "PPPoE" code. */
#define GPON_VLAN_ETYPE_MAX	5	/* ⚠ AND CODES 6..15 ARE A STOCK DEFECT WE
					 * DO NOT COPY: its table lookup falls
					 * past the end, the EtherType field is
					 * left UNWRITTEN, and the valid bit and
					 * the 0xFFFF mask are set anyway -- so
					 * the entry filters on a stale value,
					 * silently.  Nothing clamps the code on
					 * the way in either: the producer takes
					 * word1 & 0xF and sets the ETHTYPE mode
					 * bit for anything non-zero. */
#define GPON_VLAN_ETYPE_IPV4	1	/* 0x0800 */
#define GPON_VLAN_ETYPE_PPPOE_D	2	/* 0x8863 PPPoE discovery */
#define GPON_VLAN_ETYPE_ARP	3	/* 0x0806 */
#define GPON_VLAN_ETYPE_IPV6	4	/* 0x86DD */
#define GPON_VLAN_ETYPE_PPPOE_S	5	/* 0x8864 PPPoE session */

/* -> the EtherType a filter code names, or 0 for "no EtherType filter" and for
 * a code this model does not know.  A code we cannot name must never become a
 * filter on EtherType zero, which would match nothing and silently blackhole
 * the service the OLT asked for. */
static inline u16 gpon_vlan_etype_of(u8 code)
{
	switch (code) {
	case GPON_VLAN_ETYPE_IPV4:	return 0x0800;
	case GPON_VLAN_ETYPE_PPPOE_D:	return 0x8863;
	case GPON_VLAN_ETYPE_ARP:	return 0x0806;
	case GPON_VLAN_ETYPE_IPV6:	return 0x86dd;
	case GPON_VLAN_ETYPE_PPPOE_S:	return 0x8864;
	default:			return 0;
	}
}

/*
 * Decode one 16-octet row.  Explicit byte math, never a struct cast: this
 * core builds big-endian on MIPS, little-endian on ARM64 and on x86, and the
 * wire is big-endian on all three.
 */
static inline void gpon_ext_vlan_row_decode(const u8 *b,
					    struct gpon_ext_vlan_row *r)
{
	u32 w0 = ((u32)b[0] << 24) | ((u32)b[1] << 16) |
		 ((u32)b[2] << 8) | b[3];
	u32 w1 = ((u32)b[4] << 24) | ((u32)b[5] << 16) |
		 ((u32)b[6] << 8) | b[7];
	u32 w2 = ((u32)b[8] << 24) | ((u32)b[9] << 16) |
		 ((u32)b[10] << 8) | b[11];
	u32 w3 = ((u32)b[12] << 24) | ((u32)b[13] << 16) |
		 ((u32)b[14] << 8) | b[15];

	r->f_out_pri  = (u8)(w0 >> 28);
	r->f_out_vid  = (u16)((w0 >> 15) & 0x1fff);
	r->f_out_tpid = (u8)((w0 >> 12) & 0x7);
	r->f_in_pri   = (u8)(w1 >> 28);
	r->f_in_vid   = (u16)((w1 >> 15) & 0x1fff);
	r->f_in_tpid  = (u8)((w1 >> 12) & 0x7);
	r->f_ethertype = (u8)(w1 & 0xf);
	r->t_remove   = (u8)(w2 >> 30);
	r->t_out_pri  = (u8)((w2 >> 16) & 0xf);
	r->t_out_vid  = (u16)((w2 >> 3) & 0x1fff);
	r->t_out_tpid = (u8)(w2 & 0x7);
	r->t_in_pri   = (u8)((w3 >> 16) & 0xf);
	r->t_in_vid   = (u16)((w3 >> 3) & 0x1fff);
	r->t_in_tpid  = (u8)(w3 & 0x7);
}

/* The inverse, so a decode/encode round trip is testable and a stored row can
 * be served back to the OLT byte for byte.  Reserved bits are emitted as zero:
 * they are reserved, and re-emitting whatever arrived would make the round
 * trip prove less than it appears to. */
static inline void gpon_ext_vlan_row_encode(const struct gpon_ext_vlan_row *r,
					    u8 *b)
{
	u32 w0 = ((u32)(r->f_out_pri & 0xf) << 28) |
		 ((u32)(r->f_out_vid & 0x1fff) << 15) |
		 ((u32)(r->f_out_tpid & 0x7) << 12);
	u32 w1 = ((u32)(r->f_in_pri & 0xf) << 28) |
		 ((u32)(r->f_in_vid & 0x1fff) << 15) |
		 ((u32)(r->f_in_tpid & 0x7) << 12) | (r->f_ethertype & 0xf);
	u32 w2 = ((u32)(r->t_remove & 0x3) << 30) |
		 ((u32)(r->t_out_pri & 0xf) << 16) |
		 ((u32)(r->t_out_vid & 0x1fff) << 3) | (r->t_out_tpid & 0x7);
	u32 w3 = ((u32)(r->t_in_pri & 0xf) << 16) |
		 ((u32)(r->t_in_vid & 0x1fff) << 3) | (r->t_in_tpid & 0x7);

	b[0] = (u8)(w0 >> 24); b[1] = (u8)(w0 >> 16);
	b[2] = (u8)(w0 >> 8);  b[3] = (u8)w0;
	b[4] = (u8)(w1 >> 24); b[5] = (u8)(w1 >> 16);
	b[6] = (u8)(w1 >> 8);  b[7] = (u8)w1;
	b[8] = (u8)(w2 >> 24); b[9] = (u8)(w2 >> 16);
	b[10] = (u8)(w2 >> 8); b[11] = (u8)w2;
	b[12] = (u8)(w3 >> 24); b[13] = (u8)(w3 >> 16);
	b[14] = (u8)(w3 >> 8); b[15] = (u8)w3;
}

/* ---------------------------------------------------------------------------
 * The DECIDED rule — what the family is asked to install
 * ------------------------------------------------------------------------- */

/*
 * What a tag is filtered ON.  A BITMASK PER TAG, and the vocabulary is the
 * vendor's own OMCI_VLAN_FILTER_MODE_e -- recovered by name and value from the
 * G24W build's DWARF, and produced by the decoder in libomci_mib.so which is
 * the function that turns a raw ME 171 row into what the driver consumes.
 *
 * ★ IT IS A MASK AND NOT AN ENUM OF STATES, which is the thing a fresh design
 *   gets wrong: "this tag must be present and I filter on nothing about it"
 *   (CARE_TAG alone) and "I filter on its VID" (VID) are different
 *   instructions, and TCI is the pair of VID and PRI rather than a third kind.
 */
#define GPON_VLANF_NO_CARE_TAG	0x01	/* the tag may be there or not */
#define GPON_VLANF_CARE_TAG	0x02	/* it must be there; nothing else matched */
#define GPON_VLANF_NO_TAG	0x04	/* it must NOT be there */
#define GPON_VLANF_VID		0x08
#define GPON_VLANF_PRI		0x10
#define GPON_VLANF_TCI		0x20	/* VID and PRI together */
#define GPON_VLANF_ETHTYPE	0x40
#define GPON_VLANF_DSCP_PRI	0x80	/* the priority comes from the DSCP map */

struct gpon_vlan_filter {
	u8	s_mode;		/* GPON_VLANF_* for the OUTER tag */
	u8	c_mode;		/* ...and for the INNER one */
	u16	s_vid;
	u16	c_vid;
	u8	s_pri;
	u8	c_pri;
	u8	s_tpid;		/* the RAW 3-bit TPID/DE code, kept raw: it is
				 * three different instructions (do not filter /
				 * an ordinary 0x8100 tag / this ME's input
				 * TPID) and flattening it loses two of them */
	u8	c_tpid;
	u16	ethertype;	/* the WIRE value, resolved; 0 = no filter */
	bool	is_default;	/* the catch-all row */
	bool	is_discard;	/* the row says DROP what it matches */
	bool	outer_is_service_tag;	/* the outer tag's TPID code names this
					 * ME's input TPID and that is not
					 * 0x8100, so the tag is a SERVICE tag */
};

/* ---------------------------------------------------------------------------
 * The TREATMENT half — what to do with the frame the filter matched
 *
 * ★★ EVERY VALUE BELOW IS STOCK'S OWN, from the digest in libomci_mib.so that
 *    turns a raw ME 171 row into what the driver consumes.  It was the last
 *    unread hop and it is read now: the two decision helpers (priority at
 *    0x253dc, VID at 0x2545c), the TPID resolver at 0x25270, and stock's own
 *    NAME TABLE for the TPID codes at 0xeb74.
 *
 * ★ AND THE ENUM THE EARLIER PASS CALLED DISPUTED IS NOT DISPUTED: the two
 *   candidate orders belonged to two DIFFERENT enums in two different modules.
 *   The 3.18 RTL9602C build contains no `omci_pon_wan_vlan_*` string at all;
 *   what it emits is OMCI_VLAN_ACT_MODE_e, and its own name table at 0xe3d8
 *   spells it NO_ACT 0, ADD 1, REMOVE 2, MODIFY 3, TRANSPARENT 4 -- which is
 *   byte-identical to the G24W DWARF for that same enum, and is exactly what
 *   the earlier pass measured behaviourally ("2 maps to Delete, 3 to Tagging").
 *
 * ⚠ WHAT IS DELIBERATELY NOT DERIVED HERE: the per-tag VERB (add / modify /
 *   remove / leave alone).  Stock computes it to index its own rtk action map,
 *   and a family that has the six facts below -- strip count, discard, and per
 *   output tag whether it is written, where its VID comes from, where its
 *   priority comes from and what TPID/DEI it carries -- has the whole
 *   instruction without it.  The RE that settled the fields read about 55% of
 *   that digest and anchored the verb only for the cases it walked, so deriving
 *   it here would be modelling past the evidence for no gain.
 * ------------------------------------------------------------------------- */

/* TAGS_TO_REMOVE, and 3 is not a count. */
#define GPON_VLAN_REMOVE_NONE	0	/* strip nothing */
#define GPON_VLAN_REMOVE_OUTER	1	/* strip the outer (S) tag only */
#define GPON_VLAN_REMOVE_BOTH	2

/* The TREATMENT priority field.  ⚠ 8 MEANS SOMETHING DIFFERENT ON EACH SIDE OF
 * THE ROW: on the filter side it is "do not filter on priority", here it is
 * "copy the P-bits from the received inner tag".  One number, two meanings, a
 * few octets apart. */
#define GPON_VLAN_TPRI_COPY_INNER	8
#define GPON_VLAN_TPRI_COPY_OUTER	9
#define GPON_VLAN_TPRI_DSCP		10	/* via the DSCP -> P-bit map */
#define GPON_VLAN_TPRI_NONE		15	/* do not write this tag at all */

/* The TREATMENT VID field.  11..14 on the priority field and every other value
 * here are ASSIGNED RAW: stock does not reject a reserved value, it uses it,
 * and refusing where stock assigns would drop a service an OLT provisioned. */
#define GPON_VLAN_TVID_COPY_INNER	4096
#define GPON_VLAN_TVID_COPY_OUTER	4097

/* The 3-bit TPID/DE code, from stock's own name table (0xeb74).  It says BOTH
 * where the TPID comes from and what happens to the DEI bit, which is why it
 * cannot be split into two smaller fields. */
#define GPON_VLAN_TTPID_COPY_INNER	0	/* input TPID attr, DEI from inner */
#define GPON_VLAN_TTPID_COPY_OUTER	1	/* input TPID attr, DEI from outer */
#define GPON_VLAN_TTPID_OUT_DEI_INNER	2	/* output TPID attr, DEI from inner */
#define GPON_VLAN_TTPID_OUT_DEI_OUTER	3	/* output TPID attr, DEI from outer */
#define GPON_VLAN_TTPID_8100		4	/* literal 0x8100, DEI 0 */
#define GPON_VLAN_TTPID_RESERVED	5	/* stock writes NOTHING for it */
#define GPON_VLAN_TTPID_OUT_DEI0	6
#define GPON_VLAN_TTPID_OUT_DEI1	7

/* Where one written tag's VID comes from.  Values are stock's own
 * OMCI_VID_ACT_MODE_e. */
enum gpon_vlan_vid_act {
	GPON_VLAN_VID_ASSIGN = 0,	/* use .vid */
	GPON_VLAN_VID_FROM_INNER = 1,
	GPON_VLAN_VID_FROM_OUTER = 2,
	GPON_VLAN_VID_KEEP = 3,
};

/* ...and its priority.  Stock's own OMCI_PRI_ACT_MODE_e. */
enum gpon_vlan_pri_act {
	GPON_VLAN_PRI_ASSIGN = 0,	/* use .pri */
	GPON_VLAN_PRI_FROM_INNER = 1,
	GPON_VLAN_PRI_FROM_OUTER = 2,
	GPON_VLAN_PRI_FROM_DSCP = 3,
	GPON_VLAN_PRI_KEEP = 4,
};

/* One OUTPUT tag. */
struct gpon_vlan_tag_treat {
	bool	written;	/* false = this tag is not put on the frame */
	u8	vid_act;	/* enum gpon_vlan_vid_act */
	u8	pri_act;	/* enum gpon_vlan_pri_act */
	u16	vid;		/* meaningful when vid_act is ASSIGN */
	u8	pri;		/* ...and when pri_act is ASSIGN */
	u8	tpid_code;	/* the raw 3-bit code: it names a SOURCE and a DEI
				 * rule together, so it stays whole */
};

struct gpon_vlan_treat {
	bool	discard;	/* drop the frame; nothing else applies */
	u8	remove;		/* received tags to strip first: 0, 1 or 2 */
	struct gpon_vlan_tag_treat	outer;	/* word2, the S tag */
	struct gpon_vlan_tag_treat	inner;	/* word3, the C tag */
};
/* Which way the rule runs.  The values are the vendor's omci_rule_dir_e. */
enum gpon_vlan_dir {
	GPON_VLAN_US_ONLY = 0,
	GPON_VLAN_DS_ONLY = 1,
	GPON_VLAN_BOTH = 2,
};

/* One decided rule: everything a family shell needs, and no pointer into
 * anything the core owns, so a shell may copy it and install it later. */
struct gpon_vlan_rule {
	u16	me_inst;	/* the ME 171 instance it came from */
	u8	row;		/* which row of that instance */
	u8	dir;		/* enum gpon_vlan_dir */
	u8	assoc_type;	/* ME 171 #1: what this instance is attached to */
	u16	assoc_ptr;	/* ME 171 #7 */
	u16	in_tpid;	/* ME 171 #3 */
	u16	out_tpid;	/* ME 171 #4 */
	u8	ds_mode;	/* ME 171 #5 */
	struct gpon_vlan_filter	f;
	struct gpon_vlan_treat	t;
};

/* ---------------------------------------------------------------------------
 * The pure translation — the half stock proves can be pure
 * ------------------------------------------------------------------------- */

/*
 * Is this the DEFAULT (catch-all) row?  SIX conditions, not one -- this is the
 * signature stock's own decoder tests, and the canonical row it recognises is
 *
 *     F8 00 00 00 | E8 00 00 00 | 00 0F 00 00 | 00 0F 00 00
 *
 * ⚠ A LOOSER TEST IS A REAL FAULT, not a stylistic one: "filter priority is 14"
 *   alone also matches an ordinary row that happens to filter on priority 14,
 *   and calling that the catch-all would apply the subscriber's default
 *   treatment to one priority class and nothing else.
 */
static inline bool gpon_ext_vlan_row_is_default(const struct gpon_ext_vlan_row *r)
{
	return r->t_remove == 0 &&
	       r->t_out_pri == GPON_VLAN_PRI_NONE &&
	       r->t_in_pri == GPON_VLAN_PRI_NONE &&
	       r->f_out_vid == GPON_VLAN_VID_ANY &&
	       r->f_in_vid == GPON_VLAN_VID_ANY &&
	       (r->f_out_pri == GPON_VLAN_PRI_NONE ||
		r->f_out_pri == GPON_VLAN_PRI_DEFAULT) &&
	       r->f_in_pri == GPON_VLAN_PRI_DEFAULT;
}

/*
 * Is this row a DISCARD rule -- the OLT telling us to drop what it matches?
 * It is the tags-to-remove field at its one non-count value, and stock diverts
 * on it before emitting any filter.
 */
static inline bool gpon_ext_vlan_row_is_discard(const struct gpon_ext_vlan_row *r)
{
	return r->t_remove == GPON_VLAN_REMOVE_DISCARD;
}

/*
 * A row is DELETED by writing it back with BOTH TREATMENT WORDS all-ones: the
 * OLT names the row by its filter half -- the first eight octets, which stock
 * compares word for word -- and says "this one goes".
 *
 * ⚠ IT IS TESTED ON THE RAW OCTETS, not on the decoded fields, and that is not
 *   pedantry: the two treatment words carry reserved bits the decode drops, so
 *   a decoded test would call a row with those bits CLEAR a delete where stock
 *   calls it an update.  A delete that is really an update leaves a rule
 *   installed that the OLT believes it removed.
 * ⚠ AND IT IS CHECKED BEFORE THE DISCARD RULE ABOVE, in that order, because an
 *   all-ones treatment also carries tags-to-remove 3.  Stock's own plugin tests
 *   delete first; reversing the two turns every delete into "drop the frames
 *   this row matches", which is the opposite instruction.
 */
static inline bool gpon_ext_vlan_raw_is_delete(const u8 *b)
{
	unsigned int i;

	for (i = 8; i < GPON_EXT_VLAN_ROW_LEN; i++)
		if (b[i] != 0xff)
			return false;
	return true;
}

/*
 * ONE TAG's filter mode, from its raw (priority, VID) pair.  This ladder is
 * read off the producer in libomci_mib.so and it is the whole filter semantic:
 *
 *   pri 15               -> NO_TAG     the tag must be ABSENT
 *   pri 8, vid 4096      -> CARE_TAG   present, nothing about it matched
 *   pri 8, vid != 4096   -> VID
 *   pri != 8, vid 4096   -> PRI
 *   pri != 8, vid != 4096-> TCI        both
 *
 * Priority 14 is the default-rule marker and is handled by the caller before
 * this is reached; every other value 0..13 is a literal priority.
 */
static inline u8 gpon_ext_vlan_tag_mode(u8 pri, u16 vid)
{
	if (pri == GPON_VLAN_PRI_NONE)
		return GPON_VLANF_NO_TAG;
	if (pri == GPON_VLAN_PRI_ANY)
		return vid == GPON_VLAN_VID_ANY ? GPON_VLANF_CARE_TAG
						: GPON_VLANF_VID;
	return vid == GPON_VLAN_VID_ANY ? GPON_VLANF_PRI : GPON_VLANF_TCI;
}

/*
 * Turn the FILTER half of a raw row into the decided filter.
 * @in_tpid is this ME 171 instance's attribute #3, which the row needs and does
 * not carry: a TPID/DE code of 5..7 means "the input TPID", and whether that
 * makes the tag a service tag depends on its value.
 *
 * -> false when the row cannot be represented, which today is exactly one case
 *    and it is a STOCK DEFECT WE REFUSE TO COPY: an EtherType code above 5.
 *    Stock sets the entry's valid bit and mask for such a code and never writes
 *    the EtherType, so the rule filters on a stale value.  Refusing is the only
 *    honest answer -- accepting and dropping the EtherType condition would
 *    forward traffic the OLT asked us to classify.
 */
static inline bool gpon_ext_vlan_filter(const struct gpon_ext_vlan_row *r,
					u16 in_tpid,
					struct gpon_vlan_filter *f)
{
	unsigned int i;

	for (i = 0; i < sizeof(*f); i++)
		((u8 *)f)[i] = 0;
	if (r->f_ethertype > GPON_VLAN_ETYPE_MAX)
		return false;
	f->is_default = gpon_ext_vlan_row_is_default(r);
	f->is_discard = gpon_ext_vlan_row_is_discard(r);
	f->s_vid = r->f_out_vid;
	f->c_vid = r->f_in_vid;
	f->s_pri = r->f_out_pri;
	f->c_pri = r->f_in_pri;
	f->s_tpid = r->f_out_tpid;
	f->c_tpid = r->f_in_tpid;
	f->ethertype = gpon_vlan_etype_of(r->f_ethertype);
	if (f->is_default) {
		/* the catch-all matches every frame: neither tag is examined */
		f->s_mode = GPON_VLANF_NO_CARE_TAG;
		f->c_mode = GPON_VLANF_NO_CARE_TAG;
		return true;
	}
	f->s_mode = gpon_ext_vlan_tag_mode(r->f_out_pri, r->f_out_vid);
	f->c_mode = gpon_ext_vlan_tag_mode(r->f_in_pri, r->f_in_vid);
	if (r->f_ethertype != GPON_VLAN_ETYPE_ANY)
		f->c_mode |= GPON_VLANF_ETHTYPE;
	f->outer_is_service_tag = r->f_out_tpid >= GPON_VLAN_TPID_INPUT_MIN &&
				  in_tpid != GPON_VLAN_TPID_C;
	return true;
}

/* ONE output tag, from its raw treatment priority, VID and TPID code.
 * @tags_matched is how many received tags the FILTER half requires, 0..2 — the
 * copy-from-received codes need it and the row does not carry it.
 * -> false when the row asks to copy from a tag the filter does not require,
 *    which is stock's own refusal (its helper returns rc=2 for exactly this). */
static inline bool gpon_ext_vlan_tag_treat(u8 pri, u16 vid, u8 tpid_code,
					   u8 tags_matched,
					   struct gpon_vlan_tag_treat *o)
{
	o->written = pri != GPON_VLAN_TPRI_NONE;
	o->tpid_code = tpid_code;
	o->vid = 0;
	o->pri = 0;
	o->vid_act = GPON_VLAN_VID_ASSIGN;
	o->pri_act = GPON_VLAN_PRI_ASSIGN;
	if (!o->written)
		return true;

	if (pri == GPON_VLAN_TPRI_COPY_INNER) {
		if (!tags_matched)
			return false;
		/* a single matched tag has no distinct inner, so copy-inner
		 * collapses onto it -- stock's own fold, not a convenience */
		o->pri_act = tags_matched >= 2 ? GPON_VLAN_PRI_FROM_INNER
					       : GPON_VLAN_PRI_FROM_OUTER;
	} else if (pri == GPON_VLAN_TPRI_COPY_OUTER) {
		if (!tags_matched)
			return false;
		o->pri_act = GPON_VLAN_PRI_FROM_OUTER;
	} else if (pri == GPON_VLAN_TPRI_DSCP) {
		o->pri_act = GPON_VLAN_PRI_FROM_DSCP;
	} else {
		/* 0..7 AND 11..14: stock assigns a reserved value raw rather
		 * than rejecting it, and refusing where stock assigns would
		 * drop a service an OLT provisioned */
		o->pri = pri;
	}

	if (vid == GPON_VLAN_TVID_COPY_INNER)
		o->vid_act = GPON_VLAN_VID_FROM_INNER;
	else if (vid == GPON_VLAN_TVID_COPY_OUTER)
		o->vid_act = GPON_VLAN_VID_FROM_OUTER;
	else
		o->vid = vid;
	return true;
}

/*
 * The TREATMENT half of a raw row, decided.
 *
 * ⚠ DISCARD IS CHECKED FIRST AND SHORT-CIRCUITS, because stock's digest does
 *   the same -- and because stock then writes the value 3 into the field its
 *   OWN printer labels `isDefaultRule`, so a consumer reading that member has
 *   to know 3 means "drop" and not "default".  We keep discard in a field of
 *   its own rather than inherit an overload nobody has proven a reader for.
 */
static inline bool gpon_ext_vlan_treat(const struct gpon_ext_vlan_row *r,
				       u8 tags_matched,
				       struct gpon_vlan_treat *t)
{
	t->discard = gpon_ext_vlan_row_is_discard(r);
	t->remove = r->t_remove;
	if (t->discard) {
		t->outer.written = false;
		t->inner.written = false;
		return true;
	}
	return gpon_ext_vlan_tag_treat(r->t_out_pri, r->t_out_vid,
				       r->t_out_tpid, tags_matched,
				       &t->outer) &&
	       gpon_ext_vlan_tag_treat(r->t_in_pri, r->t_in_vid,
				       r->t_in_tpid, tags_matched,
				       &t->inner);
}

/* How many received tags this filter REQUIRES, 0..2 — the input the treatment
 * half needs and the row does not carry. */
static inline u8 gpon_vlan_tags_matched(const struct gpon_vlan_filter *f)
{
	u8 n = 0;

	if (!(f->s_mode & GPON_VLANF_NO_TAG))
		n++;
	if (!(f->c_mode & GPON_VLANF_NO_TAG))
		n++;
	return n;
}

/* Decide a whole row: filter, then treatment.  @in_tpid is the ME's attribute
 * #3.  -> false when the row cannot be represented (an EtherType code stock
 * itself cannot resolve, or a copy-from-received with nothing to copy). */
static inline bool gpon_ext_vlan_decide(const struct gpon_ext_vlan_row *r,
					u16 in_tpid, struct gpon_vlan_rule *out)
{
	if (!gpon_ext_vlan_filter(r, in_tpid, &out->f))
		return false;
	return gpon_ext_vlan_treat(r, gpon_vlan_tags_matched(&out->f), &out->t);
}

/* ---------------------------------------------------------------------------
 * The stored model — what the OLT has told us, per ME
 * ------------------------------------------------------------------------- */

/* How many ME 171 instances and rows this build holds.  Overridable for the
 * same reason OMCI_STORE_MAX is: one lean kernel per model, and a board with a
 * bigger panel is owed a bigger table rather than silent truncation. */
#ifndef GPON_EXT_VLAN_MAX
#define GPON_EXT_VLAN_MAX	4
#endif
#ifndef GPON_EXT_VLAN_ROWS
#define GPON_EXT_VLAN_ROWS	8
#endif

/* Zero one ME 171 instance.  A loop for the same reason the model reset uses
 * one: this header pulls in <linux/types.h> and nothing else. */
struct gpon_ext_vlan_inst;
static inline void gpon_vlan_inst_clear(struct gpon_ext_vlan_inst *e);

struct gpon_ext_vlan_inst {
	u16	inst;
	u8	nrow;
	bool	used;
	u8	row[GPON_EXT_VLAN_ROWS][GPON_EXT_VLAN_ROW_LEN];
};

/* The whole VLAN / classification model this core holds.  ME 78, 84, 79, 130,
 * 280 and 281 need no storage of their own: their whole attribute set fits the
 * dense body of the ME instance store, and the decoders below read it from
 * there.  ME 171 is the exception because one instance carries MANY rows. */
static inline void gpon_vlan_inst_clear(struct gpon_ext_vlan_inst *e)
{
	unsigned int i;

	for (i = 0; i < sizeof(*e); i++)
		((u8 *)e)[i] = 0;
}

struct gpon_vlan_model {
	struct gpon_ext_vlan_inst	ext[GPON_EXT_VLAN_MAX];
	/* ⚠ COUNTED, NOT DROPPED.  A row that did not fit and a decision no
	 * family installed are the two ways this model can go quiet again, and
	 * both are the failure it was written to end. */
	u16	rows_dropped;
	u16	owed;		/* decisions with no installer */
	/* LONG attributes accepted on the wire and not held anywhere: the two
	 * DSCP->P-bit maps (ME 171 #8, ME 130 #11) and ME 281's two multicast
	 * address tables.  Accepting them is what keeps the provisioning burst
	 * alive; COUNTING them is what stops that acceptance being a silence. */
	u16	attr_owed;
};

static inline void gpon_vlan_model_reset(struct gpon_vlan_model *m)
{
	unsigned int i;

	/* a loop, not memset(): this header includes only <linux/types.h>, the
	 * same declared deviation gpon_omci_me.h already takes for its serial
	 * setter, so the core stays free of a libc/kernel string dependency */
	for (i = 0; i < sizeof(*m); i++)
		((u8 *)m)[i] = 0;
}

static inline struct gpon_ext_vlan_inst *
gpon_ext_vlan_find(struct gpon_vlan_model *m, u16 inst)
{
	unsigned int i;

	for (i = 0; i < GPON_EXT_VLAN_MAX; i++)
		if (m->ext[i].used && m->ext[i].inst == inst)
			return &m->ext[i];
	return NULL;
}

/* Two rows address the SAME table entry when their FILTER halves agree: the
 * filter IS the key, which is what makes a replace a replace and a delete
 * addressable.  Comparing all sixteen octets instead would make every rewrite
 * a new row and the table would fill with near-duplicates. */
static inline bool gpon_ext_vlan_same_key(const u8 *a, const u8 *b)
{
	unsigned int i;

	for (i = 0; i < 8; i++)
		if (a[i] != b[i])
			return false;
	return true;
}

static inline void gpon_ext_vlan_copy_row(u8 *dst, const u8 *src)
{
	unsigned int i;

	for (i = 0; i < GPON_EXT_VLAN_ROW_LEN; i++)
		dst[i] = src[i];
}

/* Apply one ME 171 table-attribute write.  @body is the 16 octets the OLT sent.
 * -> true when the model changed (a row added, replaced or deleted).
 * A delete of a row that is not there is not an error: the OLT is entitled to
 * clean a table it did not create. */
static inline bool gpon_ext_vlan_row_set(struct gpon_vlan_model *m, u16 inst,
					 const u8 *body)
{
	struct gpon_ext_vlan_row r;
	struct gpon_ext_vlan_inst *e;
	unsigned int i, j;

	gpon_ext_vlan_row_decode(body, &r);
	(void)r;
	if (gpon_ext_vlan_raw_is_delete(body)) {
		e = gpon_ext_vlan_find(m, inst);
		if (!e)
			return false;
		for (i = 0; i < e->nrow; i++) {
			if (!gpon_ext_vlan_same_key(e->row[i], body))
				continue;
			/* close the hole: the rows are a list, not a map, and a
			 * gap would be served to an auditing OLT as a row of
			 * zeros -- a filter that matches untagged frames. */
			for (j = i; j + 1 < e->nrow; j++)
				gpon_ext_vlan_copy_row(e->row[j],
						       e->row[j + 1]);
			e->nrow--;
			return true;
		}
		return false;
	}

	e = gpon_ext_vlan_find(m, inst);
	if (!e) {
		for (i = 0; i < GPON_EXT_VLAN_MAX && !e; i++)
			if (!m->ext[i].used) {
				e = &m->ext[i];
				gpon_vlan_inst_clear(e);
				e->inst = inst;
				e->used = true;
			}
	}
	if (!e) {
		m->rows_dropped++;
		return false;
	}
	for (i = 0; i < e->nrow; i++) {
		if (!gpon_ext_vlan_same_key(e->row[i], body))
			continue;
		for (j = 0; j < GPON_EXT_VLAN_ROW_LEN; j++)
			if (e->row[i][j] != body[j])
				break;
		if (j == GPON_EXT_VLAN_ROW_LEN)
			return false;		/* the OLT re-sent what we hold */
		gpon_ext_vlan_copy_row(e->row[i], body);
		return true;
	}
	if (e->nrow >= GPON_EXT_VLAN_ROWS) {
		m->rows_dropped++;
		return false;
	}
	gpon_ext_vlan_copy_row(e->row[e->nrow], body);
	e->nrow++;
	return true;
}

/* Forget one ME 171 instance (its ME was deleted). */
static inline void gpon_ext_vlan_del(struct gpon_vlan_model *m, u16 inst)
{
	struct gpon_ext_vlan_inst *e = gpon_ext_vlan_find(m, inst);

	if (e)
		gpon_vlan_inst_clear(e);
}

/* How many rows instance @inst holds (0 when it holds none or does not exist). */
static inline u8 gpon_ext_vlan_nrow(const struct gpon_vlan_model *m, u16 inst)
{
	unsigned int i;

	for (i = 0; i < GPON_EXT_VLAN_MAX; i++)
		if (m->ext[i].used && m->ext[i].inst == inst)
			return m->ext[i].nrow;
	return 0;
}

/* The raw 16 octets of one row, or NULL.  Serving the row back is what lets an
 * OLT audit its own table instead of re-writing it forever. */
static inline const u8 *gpon_ext_vlan_raw(const struct gpon_vlan_model *m,
					  u16 inst, u8 row)
{
	unsigned int i;

	for (i = 0; i < GPON_EXT_VLAN_MAX; i++) {
		if (!m->ext[i].used || m->ext[i].inst != inst)
			continue;
		return row < m->ext[i].nrow ? m->ext[i].row[row] : NULL;
	}
	return NULL;
}

/* ---------------------------------------------------------------------------
 * The other six: pure decoders over the dense instance body
 * ------------------------------------------------------------------------- */

/* ME 78 — VLAN tagging operation configuration data, the pre-171 single-tag
 * operation ME.  An OLT that uses it instead of 171 is expressing the SAME
 * service, so it decodes into the same rule shape and the family gets one
 * install path rather than two. */
struct gpon_vlan_op {
	u8	us_mode;	/* #1 upstream tagging operation mode */
	u16	us_tci;		/* #2 upstream TCI (pri<<13 | cfi<<12 | vid) */
	u8	ds_mode;	/* #3 downstream tagging operation mode */
	u8	assoc_type;	/* #4 */
	u16	assoc_ptr;	/* #5 */
};

static inline bool gpon_vlan_op_decode(const u8 *body, unsigned int blen,
				       struct gpon_vlan_op *o)
{
	if (!body || blen < 7)
		return false;
	o->us_mode = body[0];
	o->us_tci = (u16)(((u16)body[1] << 8) | body[2]);
	o->ds_mode = body[3];
	o->assoc_type = body[4];
	o->assoc_ptr = (u16)(((u16)body[5] << 8) | body[6]);
	return true;
}

/* ⚠ AND THERE IS NO gpon_vlan_op_to_rule() HERE, DELIBERATELY.  ME 78's
 * operation-mode enum would have to be translated into the rule shape above,
 * and NOTHING on either Luna die can check that translation: measured, the
 * mib_VlanTagOpCfgData plugin imports no omci_wrapper_* on either generation,
 * so stock stores this ME and programs nothing from it.  Writing the mapping
 * anyway would be a guess with no oracle, which is the failure this project
 * pays for most.  OWED, and what settles it is an OLT that actually provisions
 * with ME 78 plus a capture of what the service then has to look like. */

/* ME 84 — VLAN tagging filter data: the per-bridge-port VLAN admit list. */
#define GPON_VLAN_FILTER_MAX	12	/* #1 is 24 octets = 12 TCI entries */

struct gpon_vlan_admit {
	u16	tci[GPON_VLAN_FILTER_MAX];	/* #1 */
	u8	fwd_op;				/* #2 forward operation */
	u8	n;				/* #3 number of valid entries */
};

static inline bool gpon_vlan_admit_decode(const u8 *body, unsigned int blen,
					  struct gpon_vlan_admit *a)
{
	unsigned int i;

	if (!body || blen < 26)
		return false;
	for (i = 0; i < GPON_VLAN_FILTER_MAX; i++)
		a->tci[i] = (u16)(((u16)body[i * 2] << 8) | body[i * 2 + 1]);
	a->fwd_op = body[24];
	a->n = body[25];
	return a->n <= GPON_VLAN_FILTER_MAX;
}

/*
 * -> is @vid one of the VIDs this list NAMES?  A fact about the list, and only
 *    that.  Each entry is a TCI: priority in the top 3 bits, CFI in bit 12,
 *    VID in the low 12 -- read off stock's own printer, which renders the
 *    entries as "FilterTbl[%d]: PRI %d,CFI %d, VID %d" with exactly that split.
 *
 * ⚠⚠ AND THERE IS DELIBERATELY NO gpon_vlan_admits() HERE.  Whether the list is
 *    a pass list, a drop list or something else is the FORWARD OPERATION
 *    (attribute #2), and NOTHING IN STOCK TESTS THAT BYTE: measured on both
 *    dies, VlanTagFilterDataDrvCfg is a log-only `return 0` and the plugin
 *    imports no omci_wrapper_* at all, so this ME is stored and never acted on.
 *    We therefore have NO oracle for the operation vocabulary, and writing one
 *    from memory of the spec is the failure mode this project pays for most --
 *    an admission verdict that reads permissive would forward traffic the OLT
 *    told us to drop.  OWED, and what settles it is an OLT that provisions with
 *    a non-trivial forward operation, captured.
 */
static inline bool gpon_vlan_admit_lists_vid(const struct gpon_vlan_admit *a,
					     u16 vid)
{
	unsigned int i;

	for (i = 0; i < a->n && i < GPON_VLAN_FILTER_MAX; i++)
		if ((a->tci[i] & 0x0fff) == (vid & 0x0fff))
			return true;
	return false;
}

/* ME 79 — MAC bridge port filter pre-assign table: the per-protocol drop
 * matrix.  Ten one-octet leaves, in the order stock's own plugin registers
 * them, which is G.988's order too. */
enum gpon_preassign_proto {
	GPON_PREASSIGN_IPV4_MCAST = 0,
	GPON_PREASSIGN_IPV6_MCAST,
	GPON_PREASSIGN_IPV4_BCAST,
	GPON_PREASSIGN_RARP,
	GPON_PREASSIGN_IPX,
	GPON_PREASSIGN_NETBEUI,
	GPON_PREASSIGN_APPLETALK,
	GPON_PREASSIGN_BRIDGE_MGMT,
	GPON_PREASSIGN_ARP,
	GPON_PREASSIGN_PPPOE_BCAST,
	GPON_PREASSIGN_N,
};

struct gpon_vlan_preassign {
	u8	filter[GPON_PREASSIGN_N];	/* 0 = forward, 1 = filter */
};

static inline bool gpon_vlan_preassign_decode(const u8 *body, unsigned int blen,
					      struct gpon_vlan_preassign *p)
{
	unsigned int i;

	if (!body || blen < GPON_PREASSIGN_N)
		return false;
	for (i = 0; i < GPON_PREASSIGN_N; i++)
		p->filter[i] = body[i];
	return true;
}

/* ME 130 — 802.1p mapper service profile: the OTHER way a GEM reaches a bridge
 * port.  An OLT that binds priorities to interworking points instead of
 * creating one ME 266 per GEM is not exotic, and a model that knows only ME
 * 266 sees nothing at all on such an OLT. */
struct gpon_pbit_map {
	u16	tp_ptr;		/* #1 the TP this mapper serves */
	u16	iw_tp[8];	/* #2..#9 per P-bit interworking TP pointer */
	u8	unmarked_opt;	/* #10 what to do with an unmarked frame */
	u8	default_pbit;	/* #12 */
	u8	tp_type;	/* #13 */
};

static inline bool gpon_pbit_map_decode(const u8 *body, unsigned int blen,
					struct gpon_pbit_map *m)
{
	unsigned int i;

	if (!body || blen < 21)
		return false;
	m->tp_ptr = (u16)(((u16)body[0] << 8) | body[1]);
	for (i = 0; i < 8; i++)
		m->iw_tp[i] = (u16)(((u16)body[2 + i * 2] << 8) |
				    body[3 + i * 2]);
	m->unmarked_opt = body[18];
	m->default_pbit = body[19];
	m->tp_type = body[20];
	return true;
}

/* -> does priority @pbit map to an interworking TP, and which?  A pointer and
 * a PRESENCE are returned separately rather than folded onto a sentinel: every
 * u16 is a legal ME instance id, so any "unmapped" value we picked would also
 * be a pointer some OLT could legitimately mean. */
static inline bool gpon_pbit_map_lookup(const struct gpon_pbit_map *m, u8 pbit,
					u16 *iw_tp)
{
	if (pbit >= 8)
		return false;
	*iw_tp = m->iw_tp[pbit];
	return true;
}

/* ME 280 — GEM traffic descriptor: CIR/PIR per GEM.  Stock lands it on a
 * shared meter; ours has nowhere to put it yet, which is exactly why the
 * decision is counted rather than dropped. */
struct gpon_gem_td {
	u32	cir;		/* #1, bytes/s */
	u32	pir;		/* #2 */
	u32	cbs;		/* #3, bytes */
	u32	pbs;		/* #4 */
	u8	colour_mode;	/* #5 */
	u8	ingress_colour;	/* #6 */
	u8	egress_colour;	/* #7 */
	u8	meter_type;	/* #8 */
};

static inline bool gpon_gem_td_decode(const u8 *body, unsigned int blen,
				      struct gpon_gem_td *d)
{
	if (!body || blen < 20)
		return false;
	d->cir = ((u32)body[0] << 24) | ((u32)body[1] << 16) |
		 ((u32)body[2] << 8) | body[3];
	d->pir = ((u32)body[4] << 24) | ((u32)body[5] << 16) |
		 ((u32)body[6] << 8) | body[7];
	d->cbs = ((u32)body[8] << 24) | ((u32)body[9] << 16) |
		 ((u32)body[10] << 8) | body[11];
	d->pbs = ((u32)body[12] << 24) | ((u32)body[13] << 16) |
		 ((u32)body[14] << 8) | body[15];
	d->colour_mode = body[16];
	d->ingress_colour = body[17];
	d->egress_colour = body[18];
	d->meter_type = body[19];
	return true;
}

/* ME 281 — multicast GEM interworking termination point: the downstream
 * multicast path.  The two address tables are separate attributes and are not
 * decoded here; what a data path needs first is the GEM this instance is
 * bound to and where it interworks. */
struct gpon_mcast_iw {
	u16	gem_ctp_ptr;	/* #1 */
	u8	iw_option;	/* #2 */
	u16	serv_prof_ptr;	/* #3 */
	u16	iw_tp_ptr;	/* #4 */
	u16	gal_prof_ptr;	/* #7 */
	u8	gal_loopback;	/* #8 */
};

static inline bool gpon_mcast_iw_decode(const u8 *body, unsigned int blen,
					struct gpon_mcast_iw *m)
{
	if (!body || blen < 12)
		return false;
	m->gem_ctp_ptr = (u16)(((u16)body[0] << 8) | body[1]);
	m->iw_option = body[2];
	m->serv_prof_ptr = (u16)(((u16)body[3] << 8) | body[4]);
	m->iw_tp_ptr = (u16)(((u16)body[5] << 8) | body[6]);
	m->gal_prof_ptr = (u16)(((u16)body[7] << 8) | body[8]);
	m->gal_loopback = body[9];
	return true;
}

/* ---------------------------------------------------------------------------
 * The family seam
 * ------------------------------------------------------------------------- */

/*
 * What a family shell must supply to make a decision real.  Every member may
 * be NULL: a shell that cannot install a kind of rule says so by leaving the
 * slot empty, and the core COUNTS the decision as owed instead of pretending
 * it landed.  That is the whole difference between this model and the silence
 * it replaces — an unimplemented install is now a number somebody can read.
 *
 * ⚠ NO OP TABLE IS FILLED IN THIS TREE TODAY, and that is stated rather than
 *   hidden: the Luna install is a classifier-entry program whose pool geometry
 *   is per-die and whose behaviour cannot be established without a board, and
 *   the board CONFIRMS, it does not DISCOVER.  What this seam buys now is that
 *   the decision EXISTS, is fuzzed on x86, and is countable.
 */
struct gpon_vlan_ops {
	int (*rule_install)(void *ctx, const struct gpon_vlan_rule *r);
	int (*rule_remove)(void *ctx, const struct gpon_vlan_rule *r);
	int (*admit_set)(void *ctx, u16 bridge_port,
			 const struct gpon_vlan_admit *a);
	int (*preassign_set)(void *ctx, u16 bridge_port,
			     const struct gpon_vlan_preassign *p);
	int (*pbit_map_set)(void *ctx, u16 inst, const struct gpon_pbit_map *m);
	int (*gem_td_set)(void *ctx, u16 inst, const struct gpon_gem_td *d);
	int (*mcast_iw_set)(void *ctx, u16 inst, const struct gpon_mcast_iw *m);
};

/* What handing a rule to the family did.  THREE answers, because "nobody could
 * install it" and "the family tried and failed" are different repairs, and
 * folding either onto "installed" is the silence this model exists to end. */
enum gpon_vlan_emit {
	GPON_VLAN_INSTALLED = 0,
	GPON_VLAN_NO_INSTALLER,		/* counted in the model's owed tally */
	GPON_VLAN_INSTALL_FAILED,
};

/*
 * Hand one decided rule to the family.  @ops and @ctx may both be NULL, which
 * is the shipped state today -- no family in this tree fills the table yet, and
 * the count is what says so out loud.
 */
static inline enum gpon_vlan_emit
gpon_vlan_rule_emit(struct gpon_vlan_model *m, const struct gpon_vlan_ops *ops,
		    void *ctx, const struct gpon_vlan_rule *r)
{
	if (!ops || !ops->rule_install) {
		m->owed++;
		return GPON_VLAN_NO_INSTALLER;
	}
	return ops->rule_install(ctx, r) ? GPON_VLAN_INSTALL_FAILED
					 : GPON_VLAN_INSTALLED;
}

/* How many decisions nobody installed.  A non-zero count on a board whose WAN
 * works means the WAN works for a reason the OLT did not ask for. */
static inline u16 gpon_vlan_owed(const struct gpon_vlan_model *m)
{
	return m->owed;
}

#endif /* GPON_OMCI_VLAN_H */
