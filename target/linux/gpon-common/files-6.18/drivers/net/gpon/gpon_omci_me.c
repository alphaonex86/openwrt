// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: G.988 managed-entity model and MIB.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 */
/*
 * gpon_omci_me.c — the ITU-T G.988 MANAGED-ENTITY model and MIB store, common
 * to every OpenWrt GPON target in this tree.  Three parts: the board identity
 * pool plus the TABLE-DRIVEN attribute descriptors (one row per class+attribute)
 * with the ONE generic filler GET and MIB-Upload-Next share, so both byte-match
 * by construction; the STATIC MIB-Upload row table; and the DYNAMIC store of the
 * instances the OLT created.  It never parses a PDU, dispatches a message type,
 * builds a response envelope, stamps a trailer or computes a MIC — that is
 * gpon_omci_core.c, the MESSAGE layer, which calls in here.
 *
 * WHY IT IS COMMON — operator, 2026-08-05: "la idea es poner en común el código
 * que corresponde para no tener mucho duplicado".  Compiled by realtek-elnath
 * (RTL9607F, Cortina, aarch64 LITTLE-endian) and by dev/rtl9607c-test on x86-64
 * under ASan+UBSan.  NOT yet by realtek-luna (MIPS BIG-endian): Luna's own model
 * in rtl9602c_eth.c emits different bytes, so adopting this one is a behaviour
 * change with its own board gate, not code motion.  See gpon_omci_me.h.
 *
 * THE CORE/SHELL RULE: it decides, it never does.  No MMIO, no lock, no
 * allocation, no sleep, no clock — every byte of state lives in the
 * caller-provided struct omci_onu, and the shell reaches in only through
 * omci_onu_set_optical().  That purity is what lets the whole model be swept on
 * x86 instead of on a ~200 s board boot.
 *
 * ENDIANNESS: attribute integers are emitted big-endian by explicit byte math
 * (omci_attr_bytes()), never a struct or pointer cast over wire bytes.
 *
 * WHAT PINS IT: the table walk is byte-for-byte equivalent to the hand-written
 * filler it replaced over all 65536 attribute masks x every modelled
 * class/instance (dev/rtl9607c-test/omci_me_table_test), and the cross-vendor
 * G.988 behaviour by omci_conformance_test.  The CHECK COUNT is compared as
 * well as the colour — a green costing fewer checks is not the same green.
 *
 * PROVENANCE (code motion 2026-08-05 from realtek-elnath's omci_responder.c)
 * and the follow-ups found while moving, still OPEN — F11 omci_store_nth()
 * renumbering on a mid-upload Delete, F18 the "HSGQ-X411AXF" equipment ID on a
 * unit certified X400AXF, the 2026-09-02 identity-pool repair, and
 * omci_me_fill()'s unobservable `over` flag:
 * dev/MEASURED-gpon-omci-me-provenance-and-followups-2026-08-05.md
 */
#include <linux/string.h>

#include "gpon_common.h"	/* GPON_GEM_BIDIR -- G.988 ME 268 direction 3 */
#include "gpon_omci_me.h"

/* ---- dynamic (OLT-provisioned) ME store ---- */

struct omci_me_inst *omci_store_find(struct omci_onu *o, u16 class_id,
				     u16 inst)
{
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used && o->store[k].class_id == class_id &&
		    o->store[k].inst == inst)
			return &o->store[k];
	return NULL;
}

/* Does the ONU hold ANY instance of @class_id?  Used to separate "I do not
 * know that class at all" (0x04) from "I know the class, not that instance"
 * (0x05) on a Set the OLT sends for something it never created. */
bool omci_store_has_class(struct omci_onu *o, u16 class_id)
{
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used && o->store[k].class_id == class_id)
			return true;
	return false;
}

/* idx-th used entry in array order — the MIB-Upload tail rows. */
struct omci_me_inst *omci_store_nth(struct omci_onu *o, u16 idx)
{
	u16 k, n = 0;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used) {
			if (n == idx)
				return &o->store[k];
			n++;
		}
	return NULL;
}

/* Insert one OLT-created instance.  Returns false when the store is FULL —
 * the caller must then NAK: a dropped Create answered OK keeps the ONU's
 * MIB-Data-Sync in lockstep with the OLT's lsync while the MIB diverged, so
 * the OLT's ME2 audit can never detect it.  NAKing freezes MDS instead, the
 * audit mismatches, and the OLT's own MIB-Reset wipes the store and
 * re-provisions from empty (the MDS-poison self-heal proven on this OLT). */
bool omci_store_put(struct omci_onu *o, u16 class_id, u16 inst,
		    const u8 *body, int blen)
{
	struct omci_me_inst *e = NULL;
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (!o->store[k].used) {
			e = &o->store[k];
			break;
		}
	if (!e)
		return false;

	memset(e, 0, sizeof(*e));
	e->used = true;
	e->class_id = class_id;
	e->inst = inst;
	e->blen = 0;
	o->store_n++;
	if (body && blen > 0) {
		if (blen > (int)sizeof(e->body))
			blen = sizeof(e->body);
		memcpy(e->body, body, blen);
		e->blen = (u8)blen;
	}
	return true;
}

/*
 * Apply a Set's attribute values to a provisioned instance.  The store holds
 * the instance's attribute bytes as an OPAQUE blob (an OLT-created class has
 * no descriptor table, so there is no attribute -> offset map for it), so a
 * Set writes its values at the head of the blob exactly as a Create does.
 * That is best-effort by construction and it is what an audit GET replays;
 * dropping the Set instead — the previous behaviour — made a
 * Create-then-Set-then-audit OLT (the common provisioning order) read back its
 * own Create defaults and re-Set forever.
 */
void omci_store_merge(struct omci_me_inst *e, const u8 *val, int vlen)
{
	if (vlen <= 0)
		return;
	if (vlen > (int)sizeof(e->body))
		vlen = sizeof(e->body);
	memcpy(e->body, val, vlen);
	if (e->blen < (u8)vlen)
		e->blen = (u8)vlen;
}

void omci_store_del(struct omci_onu *o, u16 class_id, u16 inst)
{
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);

	if (e) {
		e->used = false;
		if (o->store_n)
			o->store_n--;
	}
}

/*
 * ---- ME attribute model ----
 *
 * Constant attribute bytes live in ONE pool so a descriptor row can name
 * them with a 2-byte offset instead of a pointer (no relocation, no per-row
 * padding).  The pool is a struct with one NAMED member per G.988 identity
 * field: a row names its field (A_ID below), the offset is offsetof() and
 * the wire size is sizeof() over that member — so the reader of a row sees
 * WHICH field it serves, and a member whose length changes moves every
 * later offset WITH it instead of silently shifting the bytes under fixed
 * numbers.  The per-field _Static_asserts below pin each length to its
 * G.988 wire size, so the drift itself is a build error naming the field;
 * the flat 119-byte array this replaced could only assert the TOTAL, which
 * cannot tell 4+14 from 5+13.  Byte-for-byte equivalence with what the OLT
 * reads stays pinned by Step 4d's exhaustive GET-equivalence sweep on x86.
 */
struct omci_identity {
	/* vendor ID — ONU-G #1, Circuit-Pack #5.  The OLT recognizes HSGQ
	 * ONUs; "XPON" was rejected. */
	u8 vendor_id[4];
	/* ONU-G #2 version, zero-padded to 14 */
	u8 onu_g_version[14];
	/* SW-image bank 0 (active) version — also Circuit-Pack #4 */
	u8 sw_bank0_version[14];
	/* SW-image bank 1 version */
	u8 sw_bank1_version[14];
	/* ONU2-G #1 equipment ID, zero-padded to 20 */
	u8 equipment_id[20];
	/* logical ONU ID — ONU-G #10, CTC LoID #2, zero-padded to 24 */
	u8 loid[24];
	/* CTC #1 operation ID, zero-padded to 4 */
	u8 operator_id[4];
	/* all-zero source: SW-image #5 product code (25) and #6 hash (16),
	 * VEIP #3 interdomain name (25), ONU-G #11 logical password / CTC #3
	 * password (12).
	 *
	 * ⚠ IT WAS 16 BYTES AND THE LONGEST USER NOW WANTS 25.  Sizing this
	 * region by its longest consumer is not decoration: the region is the
	 * LAST member, so an A_ZERO row asking for more than it holds reads
	 * off the END of the object.  ASan caught exactly that the day the
	 * VEIP interdomain name (25) was corrected -- global-buffer-overflow
	 * in omci_me_fill's memcpy, nine bytes past the end. */
	u8 zeros[25];
};

static const struct omci_identity omci_id = {
	.vendor_id	  = { 'H', 'S', 'G', 'Q' },
	.onu_g_version	  = { '0', '2', 'A', '5', 'B', '1' },
	.sw_bank0_version = { 'M', '2', '2', '5', '-',
			      '2', '6', '0', '5', '2', '5' },
	.sw_bank1_version = { 'M', '2', '2', '5', '-',
			      '2', '6', '0', '5', '1', '5' },
	.equipment_id	  = { 'H', 'S', 'G', 'Q', '-',
			      'X', '4', '1', '1', 'A', 'X', 'F' },
	.loid		  = { 'u', 's', 'e', 'r' },
	.operator_id	  = { 'C', 'T', 'C' },
	/* .zeros and the tails above are 0 by omission — C zero-fills what a
	 * designated initializer does not name. */
};

/* One assert per field, NAMING the field.  The descriptor table derives its
 * wire sizes from these members (A_ID), so shrinking or growing one is
 * refused here at build time, not discovered as a shifted reply at the OLT. */
#define OMCI_ID_SIZEOF(member)						\
	sizeof(((const struct omci_identity *)0)->member)
#define OMCI_ID_ASSERT(member, n)					\
	_Static_assert(OMCI_ID_SIZEOF(member) == (n),			\
		       "omci_identity." #member " must be " #n " octets")
OMCI_ID_ASSERT(vendor_id, 4);		/* ONU-G #1 / Circuit-Pack #5 */
OMCI_ID_ASSERT(onu_g_version, 14);	/* ONU-G #2 */
OMCI_ID_ASSERT(sw_bank0_version, 14);	/* SW-image #1 / Circuit-Pack #4 */
OMCI_ID_ASSERT(sw_bank1_version, 14);	/* SW-image #1, bank 1 */
OMCI_ID_ASSERT(equipment_id, 20);	/* ONU2-G #1 */
OMCI_ID_ASSERT(loid, 24);		/* ONU-G #10 / CTC LoID #2 */
OMCI_ID_ASSERT(operator_id, 4);		/* CTC #1 */
OMCI_ID_ASSERT(zeros, 25);		/* == its longest consumer, VEIP #3 */
/* every member is a u8 array, so equality here also proves no padding crept
 * in: the pool is the same 119 wire-facing bytes the flat array held */
_Static_assert(sizeof(struct omci_identity) == 119,
	       "omci_identity is not the 119-byte pool the OLT reads");

/* Where a descriptor row takes its value bytes from. */
enum omci_attr_src {
	OMCI_SRC_CONST,		/* v = the value, big-endian in `size` bytes */
	OMCI_SRC_ID,		/* v = offsetof() into struct omci_identity */
	OMCI_SRC_SN,		/* the board serial number (8) */
	OMCI_SRC_MDS,		/* ME 2 #1 = the live MIB-Data-Sync */
	OMCI_SRC_DYN,		/* v = enum omci_attr_dyn */
	OMCI_SRC_STORE,		/* v = dense offset in the dynamic instance */
	/*
	 * A LONG attribute the 26-octet dense body cannot hold: ME 171's
	 * 16-octet row table and its 24-octet DSCP map, ME 130's DSCP map, ME
	 * 281's two multicast address tables.  v = enum omci_attr_tbl, naming
	 * where the write goes.
	 *
	 * ★★ IT EXISTS SO THE WRITE IS NOT REFUSED.  omci_me_set() is ATOMIC:
	 *    a mask bit naming an attribute no descriptor row carries comes
	 *    back UNSUPPORTED and the WHOLE Set fails rc=9 -- which is exactly
	 *    the ME 11 atomic refusal that stopped provisioning on both Luna
	 *    boards.  An OLT writing one ME 171 row would have hit it on the
	 *    first row it sent.
	 * ⚠ AND A GET OF ONE ANSWERS ZEROS.  G.988 reads a table attribute with
	 *   Get-Next, which this model does not implement (gpon_omci_core.c
	 *   answers "end of table"), so the honest state is: the WRITE is
	 *   modelled and kept, the READ-BACK is OWED.  An OLT that audits its
	 *   own ME 171 table will re-write it; one that does not, and this
	 *   bench's does not, is served correctly.
	 */
	OMCI_SRC_TBL,
};

/* Where an OMCI_SRC_TBL write goes. */
enum omci_attr_tbl {
	OMCI_TBL_EXT_VLAN_ROW,	/* ME 171 #6 -> the row table, decoded */
	OMCI_TBL_MAC_FILTER_ROW,/* ME 49 #1 -> the per-bridge-port MAC filter */
	OMCI_TBL_HELD,		/* accepted and COUNTED, not held: see
				 * gpon_vlan_model.attr_owed */
};

/* The few attributes whose value is derived from the ME INSTANCE. */
enum omci_attr_dyn {
	OMCI_DYN_UNI_ADMIN,	/* ME 11 #5: accepted reported admin state */
	/* ★★ TWO SELECTORS FOR ONE VALUE, ON PURPOSE.  #1 Expected and #2 Sensed
	 *    read the SAME declared byte today, and that is correct: the boards'
	 *    own stock reports them equal on a fixed integrated port.  They get
	 *    SEPARATE selectors so the day #1 becomes settable -- stock declares
	 *    it R/W -- the author has to decide what #2 does, instead of
	 *    inheriting a write silently.  Sensed is what the port IS; an
	 *    Expected the OLT wrote may not move it. */
	OMCI_DYN_UNI_EXPECTED,	/* ME 11 #1: the expected plug-in type */
	OMCI_DYN_UNI_TYPE,	/* ME 11 #2: the SENSED type -- the capability */
	OMCI_DYN_UNIG_ADMIN,	/* ME 264 #2: the same, for UNI-G */
	OMCI_DYN_UNIG_CAP,	/* ME 264 #3: management capability, per
				 * instance -- the extra UNI-G each board
				 * carries reports 0 where the others report 1 */
	OMCI_DYN_SW_VER,	/* ME 7 #1: per-bank version field */
	OMCI_DYN_SW_FLAG,	/* ME 7 #2/#3: bank 0 is the active+committed */
	OMCI_DYN_TCONT_ALLOC,	/* ME 262 #1: alloc-ID of this T-CONT */
	OMCI_DYN_PQ_PORT,	/* ME 277 #6: related port, counts DOWN in the
				 * 8-queue block (queue 0 -> 7, 1 -> 6, ...) */
	OMCI_DYN_TS_TCONT,	/* ME 278 #1: T-CONT pointer == the instance */
	OMCI_DYN_ANIG_RX,	/* ME 263 #10: live RX optical level */
	OMCI_DYN_ANIG_TX,	/* ME 263 #14: live TX optical level */
};

/*
 * One modelled attribute.  Rows of the same class are CONTIGUOUS and in
 * EMISSION order (G.988 packs a Get response in ascending attribute order, and
 * the order is what an OLT decoder walks — a swap silently misaligns the rest
 * of the reply).  A class with no modelled attributes carries one marker row
 * (attr 0), which is how "ME known, nothing to serve" is expressed.
 */
struct omci_attr {
	u16	class_id;
	u16	v;
	u8	attr;
	u8	size;
	u8	src;
	u8	access;	/* verified dynamic layout: read=1, write=2, create=4 */
};

/* G.988 calls the attribute writable and this ONU has exactly ONE value it can
 * realise -- the one it already reports.  A Set asking for that value changes
 * nothing and is applied by definition; any OTHER value is a real change we do
 * not implement, and it is REFUSED, never stored and ignored.
 *
 * ★ THIS IS WHAT MADE PROVISIONING STOP, and the measurement is why the bit
 *   exists rather than a blanket "accept what stock declares".  The OLT's own
 *   Set on this board asks for mask 0x2818 -- #3 auto-detection configuration,
 *   #5 administrative state, #12 ARC, #13 ARC interval -- and EVERY value in it
 *   is 0x00, which is the G.988 default for all four (measured twice, artifact
 *   results/artifacts/me11_set_refusal/RTL9603CVD/LANLY/G24W/).  We modelled
 *   only #5 as writable, so the whole request came back rc=9 ATTR_FAILED and no
 *   OMCI followed.  Three of the four attributes asked for no change at all. */
#define OMCI_ACCESS_WRITE_UNCHANGED	8

#define AT(cls, n, sz, s, arg)	{ (cls), (arg), (n), (sz), (s), 0 }
#define A_ST(cls, n, sz, off, acc) \
	{ (cls), (off), (n), (sz), OMCI_SRC_STORE, (acc) }
/* a LONG attribute the dense body cannot hold: @where is enum omci_attr_tbl */
#define A_TBL(cls, n, sz, where, acc) \
	{ (cls), (where), (n), (sz), OMCI_SRC_TBL, (acc) }
#define A_C(cls, n, sz, val)	AT(cls, n, sz, OMCI_SRC_CONST, val)
/* a constant G.988 declares writable: settable to the value it already serves */
#define A_CW(cls, n, sz, val)	{ (cls), (val), (n), (sz), OMCI_SRC_CONST,  \
				  OMCI_ACCESS_WRITE_UNCHANGED }
/* identity field: the wire size IS the named member's size — one source */
#define A_ID(cls, n, member)	AT(cls, n, OMCI_ID_SIZEOF(member),	\
				   OMCI_SRC_ID,				\
				   offsetof(struct omci_identity, member))
/* @sz octets of zeros, served from omci_id.zeros (@sz <= 25 — see the
 * zeros member: a larger ask reads off the end, and only the x86 sweep
 * plus ASan police that bound) */
#define A_ZERO(cls, n, sz)	AT(cls, n, sz, OMCI_SRC_ID,		\
				   offsetof(struct omci_identity, zeros))
#define A_SN(cls, n)		AT(cls, n, 8, OMCI_SRC_SN, 0)
#define A_MDS(cls, n)		AT(cls, n, 1, OMCI_SRC_MDS, 0)
#define A_D(cls, n, sz, dyn)	AT(cls, n, sz, OMCI_SRC_DYN, dyn)
#define A_NO_ATTRS(cls)		AT(cls, 0, 0, OMCI_SRC_CONST, 0)

static const struct omci_attr omci_attrs[] = {
	/* ---- ME 2 ONU-Data (inst 0) ---- */
	A_MDS(2, 1),				/* #1  MIB-Data-Sync */

	/* ---- ME 256 ONU-G (inst 0).  ALL 14 attributes are servable: a
	 * missing one answers a short mask and the OLT re-GETs forever. ---- */
	A_ID(256,  1, vendor_id),		/* #1  Vendor ID */
	A_ID(256,  2, onu_g_version),		/* #2  Version */
	A_SN(256, 3),				/* #3  Serial number */
	A_C(256,  4,  1, 0x02),			/* #4  Traffic-mgmt option */
	A_C(256,  5,  1, 0x00),			/* #5  ATM CC option */
	A_C(256,  6,  1, 0x00),			/* #6  Battery backup */
	A_C(256,  7,  1, 0x00),			/* #7  Admin state */
	A_C(256,  8,  1, 0x00),			/* #8  Op state */
	A_C(256,  9,  1, 0x00),			/* #9  Survival time */
	A_ID(256, 10, loid),			/* #10 Logical ONU ID */
	A_ZERO(256, 11, 12),			/* #11 Logical password */
	A_C(256, 12,  1, 0x00),			/* #12 Credentials status */
	A_C(256, 13,  2, 0x0000),		/* #13 Ext TC-layer options */
	A_C(256, 14,  1, 0x01),			/* #14 ONT state */

	/* ---- ME 257 ONU2-G (inst 0) ---- */
	A_ID(257,  1, equipment_id),		/* #1  Equipment ID */
	A_C(257,  2,  1, 0x80),			/* #2  OMCC version: G.984.4,
						 * BASELINE only — devid 0x0b is
						 * not served, and the two must
						 * stay consistent */
	A_C(257,  3,  2, 0x0031),		/* #3  Vendor product code */
	A_C(257,  4,  1, 0x01),			/* #4  Security capability */
	A_C(257,  5,  1, 0x01),			/* #5  Security mode */
	A_C(257,  6,  2, 0x0060),		/* #6  Total priority queues */
	A_C(257,  7,  1, 0x0c),			/* #7  Total traffic scheds */
	A_C(257,  8,  1, 0x01),			/* #8  Mode */
	A_C(257,  9,  2, 0x0040),		/* #9  Total GEM ports */
	A_C(257, 10,  4, 3600),			/* #10 SysUpTime — UINT32: two
						 * bytes here misaligns every
						 * later attr (proven bug) */
	A_C(257, 11,  2, 0x007f),		/* #11 Connectivity capability */
	A_C(257, 12,  1, 0x00),			/* #12 Current conn mode */
	A_C(257, 13,  2, 0x003b),		/* #13 QoS config flexibility */
	A_C(257, 14,  2, 0x0001),		/* #14 Priority-queue scale */

	/* ---- ME 5 Cardholder (inst 0x0101) ---- */
	A_C(5, 1, 1, 47),			/* #1  Actual type = Eth UNI */
	A_C(5, 2, 1, 47),			/* #2  Expected type */
	A_C(5, 3, 1, 1),			/* #3  Expected port count */

	/* ---- ME 6 Circuit-Pack (inst 0x0101) ---- */
	A_C(6,  1,  1, 47),			/* #1  Type */
	A_C(6,  2,  1, 1),			/* #2  Number of ports */
	A_SN(6, 3),				/* #3  Serial number */
	A_ID(6,  4, sw_bank0_version),		/* #4  Version */
	A_ID(6,  5, vendor_id),			/* #5  Vendor ID */
	A_C(6, 12,  1, 8),			/* #12 Total priority queues */

	/* ---- ME 7 Software-Image, banks 0 (active) + 1 ---- */
	A_D(7, 1, 14, OMCI_DYN_SW_VER),		/* #1  Version */
	A_D(7, 2,  1, OMCI_DYN_SW_FLAG),	/* #2  Is committed */
	A_D(7, 3,  1, OMCI_DYN_SW_FLAG),	/* #3  Is active */
	A_C(7, 4,  1, 1),			/* #4  Is valid */
	/* ⚠ #5 AND #6 WERE ONE ROW UNTIL 2026-08-31: this table served the image
	 * hash as #5, which is where the PRODUCT CODE lives. Measured against a
	 * real ONU's own plugin (V2801RGW mib_SWImage.so: #5 ProductCode(25),
	 * #6 ImageHash(16)) -- and it is our OWN comment that named the
	 * attribute, so the off-by-one needed no reading of the spec to see.
	 * An OLT getting #5 read 16B where 25 were due and everything after it
	 * in the same response shifted. */
	A_ZERO(7, 5, 25),			/* #5  Product code */
	A_ZERO(7, 6, 16),			/* #6  Image hash */

	/* ---- ME 11 PPTP Ethernet UNI (inst 0x0101) — THE HGU gate ---- */
	A_D(11,  1, 1, OMCI_DYN_UNI_EXPECTED),	/* #1  Expected type */
	A_D(11,  2, 1, OMCI_DYN_UNI_TYPE),	/* #2  Sensed type */
	/* ★ THE WRITABLE SET IS STOCK'S OWN, MEASURED: both boards' `mib_EthUni.so`
	 * registers the class with writable mask 0xb9fe = #1,3,4,5,8..15 (RE'd by
	 * dev/re-tools/stock_me11_registration.py, identical on the two dies), and
	 * what stock implements IS the standard here.  #2 sensed type, #6
	 * operational state and #7 configuration indication are the three G.988
	 * leaves out, and they stay read-only.  Every writable CONSTANT below is
	 * A_CW: settable to the value it already serves and refused otherwise, so
	 * an attribute with a physical side (#4 loopback, #15 power control) can
	 * never be accepted and then ignored.  #1 expected type is dynamic and
	 * per-instance and stock has no dedicated setter for it -- OWED, not
	 * modelled writable here on a guess. */
	A_CW(11,  3, 1, 0),			/* #3  Auto-detect config */
	A_CW(11,  4, 1, 0),			/* #4  Eth loopback config */
	A_D(11,  5, 1, OMCI_DYN_UNI_ADMIN),	/* #5  Admin state */
	A_C(11,  6, 1, 1),			/* #6  Op state */
	A_C(11,  7, 1, 0),			/* #7  Config ind */
	A_CW(11,  8, 2, 1518),			/* #8  Max frame size */
	A_CW(11,  9, 1, 0),			/* #9  DTE/DCE ind */
	A_CW(11, 10, 2, 0xffff),		/* #10 Pause time */
	A_CW(11, 11, 1, 2),			/* #11 Bridged/IP ind */
	A_CW(11, 12, 1, 0),			/* #12 ARC */
	A_CW(11, 13, 1, 0),			/* #13 ARC interval */
	A_CW(11, 14, 1, 0),			/* #14 PPPoE filter */
	A_CW(11, 15, 1, 0),			/* #15 Power control */

	/* ============ THE WAN SERVICE SPINE (G.988 clause 9.3) ============
	 * Every row's WIDTH and ACCESS below is EXTRACTED from the board's own
	 * stock plugin, never read off a spec from memory -- stock's per-ME
	 * shared object registers its attribute table and the `olt` field IS
	 * this `access` field (read 1 | write 2 | set-by-create 4), which the
	 * ME 268 rows further down already agree with field for field:
	 *   dev/re-tools/venv/bin/python3 dev/OMCI-simulate/me_attr_table.py \
	 *       --so <stock rootfs>/lib/omci/mib_<Name>.so
	 * The stock attribute NAME is quoted beside each row so a reader can
	 * re-run that command and check the row against its source.
	 *
	 * ⚠ THE DENSE OFFSETS ARE THE ONLY PLACE THEY ARE SPELLED.  Nothing
	 *   else in this tree reads these bodies by literal offset -- the
	 *   service resolver at the end of this file goes through the table --
	 *   so a width correction moves every reader with it.
	 *
	 * ---- ME 45 MAC bridge service profile (mib_MacBriServProf) ----
	 * The bridge instance every bridge port hangs off. */
	A_ST(45,  1, 1,  0, 7),		/* #1  spanning tree ind   SpanTreeInd */
	A_ST(45,  2, 1,  1, 7),		/* #2  learning ind        LearningInd */
	A_ST(45,  3, 1,  2, 7),		/* #3  port bridging ind   AtmBriInd */
	A_ST(45,  4, 2,  3, 7),		/* #4  priority            Priority */
	A_ST(45,  5, 2,  5, 7),		/* #5  max age             MaxAge */
	A_ST(45,  6, 2,  7, 7),		/* #6  hello time          HelloTime */
	A_ST(45,  7, 2,  9, 7),		/* #7  forward delay       ForwardDelay */
	A_ST(45,  8, 1, 11, 7),		/* #8  unknown MAC discard DiscardUnknow */
	A_ST(45,  9, 1, 12, 7),		/* #9  MAC learning depth  MacLearningDepth */
	A_ST(45, 10, 4, 13, 7),		/* #10 ageing time         DynamicFilteringAgeingTime */

	/* ---- ME 47 MAC bridge port configuration data (mib_MacBriPortCfgData)
	 * The ME that BINDS a bridge (#1) to a termination point (#4), which
	 * for a WAN service is the ME 266 below.  #3 TP type says WHICH class
	 * #4 points at and is carried as DATA: this model never branches on the
	 * numeral, it follows the pointer, so no TP-type coding is assumed. */
	A_ST(47,  1, 2,  0, 7),		/* #1  bridge ID pointer   BridgeIdPtr */
	A_ST(47,  2, 1,  2, 7),		/* #2  port number         PortNum */
	A_ST(47,  3, 1,  3, 7),		/* #3  TP type             TPType */
	A_ST(47,  4, 2,  4, 7),		/* #4  TP pointer          TPPointer */
	A_ST(47,  5, 2,  6, 7),		/* #5  port priority       PortPriority */
	A_ST(47,  6, 2,  8, 7),		/* #6  port path cost      PortPathCost */
	A_ST(47,  7, 1, 10, 7),		/* #7  port spanning tree  PortSpanTreeInd */
	A_ST(47,  8, 1, 11, 7),		/* #8  deprecated          EncapMethod */
	A_ST(47,  9, 1, 12, 7),		/* #9  deprecated          LanFCSInd */
	A_ST(47, 10, 6, 13, 1),		/* #10 port MAC address    PortMacAddr (R) */
	A_ST(47, 11, 2, 19, 3),		/* #11 outbound TD ptr     OutboundTD */
	A_ST(47, 12, 2, 21, 3),		/* #12 inbound TD ptr      InboundTD */
	A_ST(47, 13, 1, 23, 7),		/* #13 MAC learning depth  NumOfAllowedMac */

	/*
	 * ---- ME 49 MAC bridge port filter table data
	 *      (mib_MacBridgePortFilterTable) -- the per-bridge-port MAC
	 * filter.  ONE attribute and it is the whole ME: a table of 8-octet
	 * rows the OLT WRITES (stock's own OltAcc is 3 = read|write, and there
	 * is no set-by-create bit anywhere in the plugin).
	 *
	 * ★★ WHY IT IS NOT "ACKed AND IGNORED" LIKE THE OTHER SPINE CLASSES
	 *    WERE.  Those arrive as a Create, and omci_store_create() ACKs any
	 *    class.  This one never does: stock's own action mask for it is
	 *    0x04000300 -- Set, Get and Get-Next, and NOT Create or Delete
	 *    (against 0x350 = Create|Delete|Set|Get for ME 47 and ME 268; read
	 *    statically out of each plugin's mibTable_init, and IDENTICAL on
	 *    both Luna dies).  So the OLT SETS it, and until this row existed a
	 *    Set of class 49 hit an instance that did not exist and came back
	 *    rc=0x04 UNKNOWN_ME -- a refusal, mid-burst, which is the shape of
	 *    the ME 11 atomic refusal that stopped provisioning on both Luna
	 *    boards.  omci_config_apply now creates the instance with its
	 *    bridge port, exactly as G.988 and stock's action mask say.
	 *
	 * ⚠ THE WRITE IS HELD, THE READ-BACK IS OWED, same as ME 171 #6: a Get
	 *   of a table attribute answers zeros of the right WIDTH and Get-Next
	 *   answers end-of-table, so an OLT that AUDITS its filter table will
	 *   re-write it.  Re-writing is idempotent here (an ADD over a MAC we
	 *   already hold replaces it), which is why the gap is a cost and not a
	 *   fault.  OWED: the table Get/Get-Next encoding, settled by RE of
	 *   libomci_mib.so's Get handler -- not by recalling G.988.
	 */
	A_TBL(49, 1, GPON_MAC_FILTER_ROW_LEN, OMCI_TBL_MAC_FILTER_ROW, 3),
					/* #1  MAC filter table    MACFilterTable */

	/* ---- ME 50 MAC bridge port bridge table data (mib_MacBriPortBriTblData)
	 * ★ KNOWN, WITH NO MODELLED ATTRIBUTE, AND THAT IS THE HONEST ANSWER.
	 *   Its single attribute is a TABLE (stock: `BriTbl`, type 5, 8-octet
	 *   entries, read-only) and this core implements no table-attribute
	 *   encoding at all -- Get-Next already answers end-of-table.  Serving
	 *   a made-up size or a row of zeros would be a confident number nobody
	 *   measured; naming the attribute UNSUPPORTED is what lets an OLT stop
	 *   asking, which is this model's own doctrine for an attribute it does
	 *   not serve.  ⚠ OWED: the Get encoding of a table attribute, settled
	 *   by RE of libomci_mib.so's Get handler, not by recalling G.988.
	 * ★ ITS INSTANCES ARE STILL REAL: G.988 makes this ME the ONU's, created
	 *   and deleted WITH its bridge port, and omci_config_apply does exactly
	 *   that -- so the OLT sees the instance exist and a Get answers about
	 *   the right thing instead of UNKNOWN_ME. */
	A_NO_ATTRS(50),

	/* ---- ME 52 MAC bridge port PM history data
	 *      (mib_MacBridgePortPmMonitorHistoryData)
	 * ⚠ COUNTERS, NOT A DATAPATH RULE -- and NOTHING FEEDS THEM YET.  They
	 *   are served from the store, i.e. as the zeros a fresh PM interval
	 *   legitimately holds, and an OLT reading zero here is reading "this
	 *   ONU does not count bridge-port frames", not "no frames passed".
	 *   OWED, at FAMILY tier: the per-bridge-port frame counters.  Modelling
	 *   it is still strictly better than the opaque store it replaces, which
	 *   replayed whatever the Create body happened to contain. */
	A_ST(52, 1, 1,  0, 1),		/* #1 interval end time    IntervalEndTime (R) */
	A_ST(52, 2, 2,  1, 7),		/* #2 threshold data 1/2   ThresholdData12Id */
	A_ST(52, 3, 4,  3, 1),		/* #3 forwarded frames     ForwardedFrameCounter */
	A_ST(52, 4, 4,  7, 1),		/* #4 delay exceeded disc  DelayExceededDiscard */
	A_ST(52, 5, 4, 11, 1),		/* #5 MTU exceeded disc    MtuExceededDiscard */
	A_ST(52, 6, 4, 15, 1),		/* #6 received frames      ReceivedFrameCounter */
	A_ST(52, 7, 4, 19, 1),		/* #7 received+discarded   ReceivedAndDiscardedCounter */

	/*
	 * ---- ME 78 VLAN tagging operation configuration data
	 *      (mib_VlanTagOpCfgData) -- the pre-171 single-tag operation ME.
	 * MEASURED: this plugin imports NO omci_wrapper_* on either Luna die, so
	 * stock STORES it and programs nothing.  Modelling it and storing it is
	 * therefore byte-for-byte what stock does here, and there is no family
	 * install owed behind it. */
	A_ST(78, 1, 1, 0, 7),		/* #1  upstream op mode    UsTagOpMode */
	A_ST(78, 2, 2, 1, 7),		/* #2  upstream TCI        UsTagTci */
	A_ST(78, 3, 1, 3, 7),		/* #3  downstream op mode  DsTagOpMode */
	A_ST(78, 4, 1, 4, 7),		/* #4  association type    Type */
	A_ST(78, 5, 2, 5, 7),		/* #5  associated ME ptr   Pointer */

	/*
	 * ---- ME 79 MAC bridge port filter pre-assign table
	 *      (mib_MacBridgePortFilterPreassign) -- the per-protocol drop
	 * matrix.  Ten one-octet leaves and NO set-by-create bit on any of them
	 * (stock's own OltAcc is 3 = read|write): G.988 has the ONU create this
	 * ME with its bridge port, and the OLT then Sets it.  OWED at FAMILY
	 * tier: stock reaches omci_wrapper_setGroupMacFilter from here. */
	A_ST(79,  1, 1, 0, 3),		/* #1  IPv4 multicast      IPv4McastFilter */
	A_ST(79,  2, 1, 1, 3),		/* #2  IPv6 multicast      IPv6McastFilter */
	A_ST(79,  3, 1, 2, 3),		/* #3  IPv4 broadcast      IPv4BcastFilter */
	A_ST(79,  4, 1, 3, 3),		/* #4  RARP                RARPFilter */
	A_ST(79,  5, 1, 4, 3),		/* #5  IPX                 IPXFilter */
	A_ST(79,  6, 1, 5, 3),		/* #6  NetBEUI             NetBEUIFilter */
	A_ST(79,  7, 1, 6, 3),		/* #7  AppleTalk           AppleTalkFilter */
	A_ST(79,  8, 1, 7, 3),		/* #8  bridge management   BridgeManaInfofilter */
	A_ST(79,  9, 1, 8, 3),		/* #9  ARP                 ARPFilter */
	A_ST(79, 10, 1, 9, 3),		/* #10 PPPoE broadcast     PPPoeBcastFilter */

	/*
	 * ---- ME 84 VLAN tagging filter data (mib_VlanTagFilterData) -- the
	 * per-bridge-port VLAN admit list.  Its dense body is EXACTLY the 26
	 * octets the store holds, which is why the list is modelled whole.
	 * MEASURED: VlanTagFilterDataDrvCfg on the RTL9602C is a log-only
	 * `return 0` and the plugin imports no omci_wrapper_* on either die --
	 * stock stores this list and programs nothing, so storing it is exact
	 * parity and no family install is owed. */
	A_ST(84, 1, 24,  0, 7),		/* #1  VLAN filter list    FilterTbl */
	A_ST(84, 2,  1, 24, 7),		/* #2  forward operation   FwdOp */
	A_ST(84, 3,  1, 25, 7),		/* #3  number of entries   NumOfEntries */

	/*
	 * ---- ME 130 802.1p mapper service profile (mib_Map8021pServProf) --
	 * the OTHER way a GEM reaches a bridge port: priority -> interworking
	 * TP, instead of one ME 266 per GEM.  An OLT that provisions this way
	 * is not exotic, and a model carrying only ME 266 sees nothing at all
	 * on one.  OWED at FAMILY tier: stock reaches setsVeipPriQ /
	 * setsSingleVeipPriQ / setDscpRemap from here. */
	A_ST(130,  1, 2,  0, 7),	/* #1  TP pointer          TPPtr */
	A_ST(130,  2, 2,  2, 7),	/* #2  P-bit 0 -> IW TP    IwTpPtrPbit0 */
	A_ST(130,  3, 2,  4, 7),	/* #3  P-bit 1             IwTpPtrPbit1 */
	A_ST(130,  4, 2,  6, 7),	/* #4  P-bit 2             IwTpPtrPbit2 */
	A_ST(130,  5, 2,  8, 7),	/* #5  P-bit 3             IwTpPtrPbit3 */
	A_ST(130,  6, 2, 10, 7),	/* #6  P-bit 4             IwTpPtrPbit4 */
	A_ST(130,  7, 2, 12, 7),	/* #7  P-bit 5             IwTpPtrPbit5 */
	A_ST(130,  8, 2, 14, 7),	/* #8  P-bit 6             IwTpPtrPbit6 */
	A_ST(130,  9, 2, 16, 7),	/* #9  P-bit 7             IwTpPtrPbit7 */
	A_ST(130, 10, 1, 18, 7),	/* #10 unmarked frame opt  UnmarkFrmOpt */
	A_TBL(130, 11, 24, OMCI_TBL_HELD, 3),
					/* #11 DSCP -> P-bit map   DscpMap2Pbit */
	A_ST(130, 12, 1, 19, 7),	/* #12 default P-bit mark  DefPbitMark */
	A_ST(130, 13, 1, 20, 7),	/* #13 TP type             TPType */

	/*
	 * ---- ME 171 extended VLAN tagging operation configuration data
	 *      (mib_ExtVlanTagOperCfgData) -- THE SUBSCRIBER VLAN.
	 *
	 * #6 is the row table and it is the whole point: every subscriber VLAN
	 * the OLT expresses arrives there, 16 octets at a time, and a 26-octet
	 * dense body cannot hold even two rows.  It goes to gpon_vlan_model's
	 * own table (gpon_omci_vlan.h), decoded, with the layout read off
	 * stock's own ExtVlanTagOperCfgDataDumpMib.
	 *
	 * #2 is the table's MAXIMUM SIZE and is served as OUR capacity, not as
	 * whatever the OLT wrote: stock declares it writable and a written value
	 * would make this ONU state a capacity it does not have.  That is the
	 * one place in these seven where stock's own access byte is not
	 * followed, and it is deliberate.
	 */
	A_ST(171, 1, 1, 0, 7),		/* #1  association type    AssociationType */
	A_C(171, 2, 2, GPON_EXT_VLAN_ROWS),
					/* #2  table max size (R)  our capacity */
	A_ST(171, 3, 2, 1, 3),		/* #3  input TPID          InputTPID */
	A_ST(171, 4, 2, 3, 3),		/* #4  output TPID         OutputTPID */
	A_ST(171, 5, 1, 5, 3),		/* #5  downstream mode     DsMode */
	A_TBL(171, 6, GPON_EXT_VLAN_ROW_LEN, OMCI_TBL_EXT_VLAN_ROW, 3),
					/* #6  the ROW TABLE       RxFrameVlanTagOperTable */
	A_ST(171, 7, 2, 6, 7),		/* #7  associated ME ptr   AssociatedMePoint */
	A_TBL(171, 8, 24, OMCI_TBL_HELD, 3),
					/* #8  DSCP -> P-bit map   DscpToPbitMapping */

	/* ---- ME 131 OLT-G: known, no modelled attributes (the OLT Sets it) */
	A_NO_ATTRS(131),

	/* ---- ME 262 T-CONT (inst 0x8000..0x800b) ---- */
	A_D(262, 1, 2, OMCI_DYN_TCONT_ALLOC),	/* #1  Alloc-ID */
	A_C(262, 2, 1, 1),			/* #2  Mode indicator */
	A_C(262, 3, 1, 0),			/* #3  Policy */

	/* ---- ME 263 ANI-G (inst 0x8001) ---- */
	A_C(263,  1, 1, 1),			/* #1  SR indication */
	A_C(263,  2, 2, 12),			/* #2  Total T-CONTs */
	A_C(263,  3, 2, 48),			/* #3  GEM block length */
	A_C(263,  4, 1, 0),			/* #4  Piggyback DBA */
	A_C(263,  5, 1, 0),			/* #5  (deprecated) */
	A_C(263,  6, 1, 5),			/* #6  SF threshold */
	A_C(263,  7, 1, 9),			/* #7  SD threshold */
	A_C(263,  8, 1, 0),			/* #8  ARC */
	A_C(263,  9, 1, 0),			/* #9  ARC interval */
	/* #10/#14 are the LIVE optical levels, sampled from the optic's
	 * SFF-8472 A2h diagnostics by the shell (see omci_onu_set_optical);
	 * until the first successful read — and after a failed one — they serve
	 * OMCI_ANIG_{RX,TX}_FALLBACK, because the OLT must never get silence. */
	A_D(263, 10, 2, OMCI_DYN_ANIG_RX),	/* #10 RX optical level */
	A_C(263, 11, 1, 0xff),			/* #11 Lower optical thresh */
	A_C(263, 12, 1, 0xff),			/* #12 Upper optical thresh */
	A_C(263, 13, 2, 0x0000),		/* #13 ONU response time */
	A_D(263, 14, 2, OMCI_DYN_ANIG_TX),	/* #14 TX optical level */
	A_C(263, 15, 1, 0x81),			/* #15 Lower TX power thresh */
	A_C(263, 16, 1, 0x81),			/* #16 Upper TX power thresh */

	/* ---- ME 264 UNI-G (inst 0x0101) ---- */
	A_C(264, 1, 2, 0x0000),			/* #1  Config-option status */
	A_D(264, 2, 1, OMCI_DYN_UNIG_ADMIN),	/* #2  Admin state */
	A_D(264, 3, 1, OMCI_DYN_UNIG_CAP),	/* #3  Management capability */
	A_C(264, 4, 2, 0x0000),			/* #4  Non-OMCI mgmt ID */
	A_C(264, 5, 2, 0x0000),			/* #5  Relay-agent options */

	/* ---- ME 266 GEM interworking termination point (mib_GemIwTp) ----
	 * ★ THE LINK.  #1 points DOWN at the ME 268 that names the GEM port,
	 *   #4 is what an ME 47 bridge port points at, and #7 selects the GAL
	 *   profile below.  Without this class a modelled GEM has nowhere to
	 *   go and the service has to come from uci. */
	A_ST(266, 1, 2,  0, 7),		/* #1 GEM port CTP pointer GemCtpPtr */
	A_ST(266, 2, 1,  2, 7),		/* #2 interworking option  IwOpt */
	A_ST(266, 3, 2,  3, 7),		/* #3 service profile ptr  ServProPtr */
	A_ST(266, 4, 2,  5, 7),		/* #4 interworking TP ptr  IwTpPtr */
	A_ST(266, 5, 1,  7, 1),		/* #5 PPTP counter         PptpCounter (R) */
	A_ST(266, 6, 1,  8, 1),		/* #6 operational state    OpState (R) */
	A_ST(266, 7, 2,  9, 7),		/* #7 GAL profile pointer  GalProfPtr */
	A_ST(266, 8, 1, 11, 3),		/* #8 GAL loopback config  GalLoopbackCfg */

	/* ME 268 GEM-port network CTP. Dense values differ from Create after
	 * attr 5: read-only #6 is absent from SBC, so #7 moves from 9 to 10.
	 * Width/access facts: own X111W mib_GemPortCtp.so mibTable_init.
	 * The full 16-byte value fits one baseline Get or Upload row. */
	A_ST(268,  1, 2,  0, 7),
	A_ST(268,  2, 2,  2, 7),
	A_ST(268,  3, 1,  4, 7),
	A_ST(268,  4, 2,  5, 7),
	A_ST(268,  5, 2,  7, 7),
	A_ST(268,  6, 1,  9, 1),
	A_ST(268,  7, 2, 10, 7),
	A_ST(268,  8, 1, 12, 1),
	A_ST(268,  9, 2, 13, 3),
	A_ST(268, 10, 1, 15, 3),

	/* ---- ME 272 GAL Ethernet profile (mib_GalEthProf) ----
	 * 266's companion, and one attribute wide: the maximum GEM payload the
	 * interworking point may emit. */
	A_ST(272, 1, 2, 0, 7),		/* #1 max GEM payload size MaxGemPayloadSize */

	/* ---- ME 277 Priority-Queue (inst 0..7) ---- */
	A_C(277, 1, 1, 1),			/* #1  Queue config option */
	A_C(277, 2, 2, 3276),			/* #2  Max queue size */
	A_C(277, 3, 2, 3276),			/* #3  Allocated queue size */
	A_C(277, 4, 2, 0),			/* #4  Discard reset interval */
	A_C(277, 5, 2, 0),			/* #5  Threshold value */
	A_D(277, 6, 4, OMCI_DYN_PQ_PORT),	/* #6  Related port */
	A_C(277, 7, 2, 0x0000),			/* #7  Traffic-sched pointer */
	A_C(277, 8, 1, 1),			/* #8  Weight */

	/* ---- ME 278 Traffic-Scheduler (inst 0x8000..0x800b) ---- */
	A_D(278, 1, 2, OMCI_DYN_TS_TCONT),	/* #1  T-CONT pointer */
	A_C(278, 2, 2, 0x0000),			/* #2  Traffic-sched pointer */
	A_C(278, 3, 1, 1),			/* #3  Policy */
	A_C(278, 4, 1, 0),			/* #4  Priority/weight */

	/*
	 * ---- ME 280 GEM traffic descriptor (mib_GemTrafficDescriptor) --
	 * CIR/PIR/CBS/PBS per GEM, pointed at by ME 268 #5 and #9.
	 *
	 * ⚠ EIGHT attributes, not G.988's eleven: stock implements ONE
	 *   direction's four rates plus the colour set, IDENTICALLY on both
	 *   Luna dies, and this table is the DEVICE's, not the spec's.
	 * ⚠⚠ AND IT REACHES NO DRIVER PATH AT ALL -- MEASURED on both dies: the
	 *   plugin imports no omci_wrapper_*, and pf_rg.ko's only two calls to
	 *   rtk_rg_shareMeter_set are in pf_rtl96xx_SetDot1RateLimiter (a
	 *   different, vendor ME) and in pf_rtl96xx_ResetMib (teardown).  So
	 *   stock STORES this descriptor and meters nothing from it; storing it
	 *   is exact parity and no family install is owed. */
	A_ST(280, 1, 4,  0, 7),		/* #1  CIR                 CIR */
	A_ST(280, 2, 4,  4, 7),		/* #2  PIR                 PIR */
	A_ST(280, 3, 4,  8, 7),		/* #3  CBS                 CBS */
	A_ST(280, 4, 4, 12, 7),		/* #4  PBS                 PBS */
	A_ST(280, 5, 1, 16, 7),		/* #5  colour mode         ColourMode */
	A_ST(280, 6, 1, 17, 7),		/* #6  ingress colour mark IngressColourMarking */
	A_ST(280, 7, 1, 18, 7),		/* #7  egress colour mark  EgressColourMarking */
	A_ST(280, 8, 1, 19, 7),		/* #8  meter type          MeterType */

	/*
	 * ---- ME 281 multicast GEM interworking termination point
	 *      (mib_MultiGemIwTp) -- the DOWNSTREAM multicast path, and the ME
	 * that NAMES the multicast GEM.  Both our shells hardcode
	 * GPON_MCAST_GEM_PORT (4095) today, so an OLT that assigns a different
	 * one is obeyed nowhere.  OWED at FAMILY tier: stock reaches
	 * omci_wrapper_cfgGemFlow from here -- the SAME entry point the data
	 * GEM install uses, which is why this one is the cheapest of the three
	 * remaining family installs.
	 * #5 and #6 are read-only on the wire and nothing on our side counts
	 * PPTPs or tracks a per-ME operational state, so they are constants
	 * rather than a store slot that would report whatever a Create left. */
	A_ST(281, 1, 2, 0, 7),		/* #1  GEM CTP pointer     GemCtpPtr */
	A_ST(281, 2, 1, 2, 7),		/* #2  interworking option IwOpt */
	A_ST(281, 3, 2, 3, 7),		/* #3  service profile ptr ServProPtr */
	A_ST(281, 4, 2, 5, 7),		/* #4  IW TP pointer       IwTpPtr */
	A_C(281, 5, 1, 0),		/* #5  PPTP counter (R)    PptpCounter */
	A_C(281, 6, 1, 0),		/* #6  operational state   OpState (R) */
	A_ST(281, 7, 2, 7, 7),		/* #7  GAL profile pointer GalProfPtr */
	A_ST(281, 8, 1, 9, 7),		/* #8  GAL loopback config GalLoopbackCfg */
	A_TBL(281,  9, 12, OMCI_TBL_HELD, 3),
					/* #9  IPv4 multicast addr IPv4MCastAddrTable */
	A_TBL(281, 10, 24, OMCI_TBL_HELD, 3),
					/* #10 IPv6 multicast addr IPv6MCastAddrTable */

	/* ---- ME 329 VEIP (inst 0x0601) — the HGU marker ---- */
	A_C(329, 1, 1, 0),			/* #1  Admin state */
	A_C(329, 2, 1, 0),			/* #2  Op state */
	/* ⚠ THE SAME OFF-BY-ONE, and on the ME this port's WAN path depends on.
	 * This row was #3 at 2 bytes; a real ONU's plugin (V2801RGW mib_VEIP.so)
	 * has #3 InterDomainName(25) and #4 TcpUdpPtr(2). The 2-byte pointer was
	 * the right VALUE at the wrong NUMBER, so it moves to #4 and #3 becomes
	 * the 25-byte name it always was. */
	A_ZERO(329, 3, 25),			/* #3  Interdomain name */
	A_C(329, 4,  2, 0x0000),		/* #4  TCP/UDP pointer */

	/* ---- ME 65530 CTC LoID authentication (inst 0) ---- */
	A_ID(65530, 1, operator_id),		/* #1  Operation ID */
	A_ID(65530, 2, loid),			/* #2  LoID */
	A_ZERO(65530, 3, 12),			/* #3  Password ("" but MUST be
						 * servable, proven) */
	A_C(65530, 4,  1, 0x01),		/* #4  Auth status = success */

	{ 0, 0, 0, 0, 0, 0 },			/* terminator (class 0 is not a
						 * G.988 class ID) */
};

/*
 * Is @class_id in a G.988 vendor-reserved range (240..255, 350..399,
 * 65280..65535)?  An ONU cannot know WHICH vendor MEs a foreign OLT audits, so
 * the whole reserved space gets ONE policy: a KNOWN ME that models no
 * attributes.  UNKNOWN_ME here aborts an OLT's config load — proven on this
 * HSGQ OLT with classes 0xfff9 and 0xffb1, which used to be hard-coded one by
 * one.  Stock does the same job as DATA (/etc/omci_ignore_mib_tbl.conf lists
 * 255, 247, 65417, 65427, 65505..65509), i.e. a set of classes to answer
 * without modelling; a range policy is the same rule without the list.
 * Vendor MEs are intentionally absent from the MIB upload.
 */
static bool omci_vendor_class(u16 class_id)
{
	return (class_id >= 240 && class_id <= 255) ||
	       (class_id >= 350 && class_id <= 399) ||
	       class_id >= 65280;
}

/* First descriptor row of @class_id, or NULL if the model does not carry it. */
static const struct omci_attr *omci_me_find(u16 class_id)
{
	const struct omci_attr *a;

	for (a = omci_attrs; a->class_id; a++)
		if (a->class_id == class_id)
			return a;
	return NULL;
}

/* Which slot of @inv holds @inst, or -1 when this board has no such instance.
 * The comparison is on the FULL u16: the G24W's fourth PPTP Ethernet UNI is
 * 0x0401 and its first is 0x0101, so a low-byte port index matches both. */
int omci_uni_slot(const struct omci_uni_inv *inv, u16 inst)
{
	u8 i;

	for (i = 0; i < inv->n; i++)
		if (inv->inst[i] == inst)
			return i;
	return -1;
}

/* The reported administrative state of @inst.  An instance this board does not
 * have reads 0 -- G.988 unlocked, and what the model answered for every
 * instance but one before the inventory existed. */
static u8 omci_uni_admin(const struct omci_uni_inv *inv, u16 inst)
{
	int slot = omci_uni_slot(inv, inst);

	return slot < 0 ? 0 : inv->admin[slot];
}

/* Record an administrative state the model has accepted, flagging the slot
 * only when the state actually MOVED: a re-Set of the value already held must
 * not make the family touch a port that is already where the OLT wants it. */
static void omci_uni_accept(struct omci_uni_inv *inv, int slot, u8 admin)
{
	if (inv->admin[slot] == admin)
		return;
	inv->admin[slot] = admin;
	inv->changed |= (u8)(1u << slot);
}

u8 omci_uni_take_changed(struct omci_uni_inv *inv)
{
	u8 changed = inv->changed;

	inv->changed = 0;
	return changed;
}

void omci_uni_mark_changed(struct omci_uni_inv *inv, u8 slot)
{
	if (slot < inv->n)
		inv->changed |= (u8)(1u << slot);
}

bool omci_uni_apply_run(const struct omci_uni_apply_ops *ops, void *sh,
			u8 changed, u8 n, const u8 *admin)
{
	bool retry = false;
	u8 i;

	if (!ops || !ops->apply || !ops->rearm || !admin)
		return false;
	if (n > OMCI_UNI_MAX)
		n = OMCI_UNI_MAX;
	for (i = 0; i < n; i++) {
		enum omci_uni_apply_rc rc;

		if (!(changed & (u8)(1u << i)))
			continue;
		rc = ops->apply(sh, i, admin[i] != 0);
		if (rc == OMCI_UNI_APPLIED)
			continue;
		/* NOT APPLIED ⇒ STILL OWED, both ways.  take_changed() has
		 * already cleared the word, so anything not put back here is
		 * gone: no later Set of the same value re-arms it and a
		 * MIB-Reset only covers the unlock direction. */
		ops->rearm(sh, i);
		if (rc != OMCI_UNI_NO_PORT)
			retry = true;
	}
	return retry;
}

/* A MIB-Reset returns every UNI to unlocked.  ⚠ AND FLAGS THE ONES THAT WERE
 * LOCKED: a port the OLT held down and then reset the MIB out from under must
 * come back up, and the changed word is the only thing that says so. */
static void omci_uni_reset(struct omci_uni_inv *inv)
{
	u8 i;

	for (i = 0; i < inv->n; i++) {
		if (!inv->admin[i])
			continue;
		inv->admin[i] = 0;
		inv->changed |= (u8)(1u << i);
	}
}

/* One list's declaration rules.  A duplicate or a zero instance id would put
 * the MIB and the OLT's copy of it permanently out of step, so the whole
 * declaration is judged before any of it is installed. */
static bool omci_uni_list_ok(const u16 *inst, u8 n)
{
	u8 i, j;

	if (n > OMCI_UNI_MAX || (n && !inst))
		return false;
	for (i = 0; i < n; i++) {
		if (!inst[i])
			return false;
		for (j = 0; j < i; j++)
			if (inst[j] == inst[i])
				return false;
	}
	return true;
}

static void omci_uni_install(struct omci_uni_inv *inv, const u16 *inst, u8 n)
{
	u8 i;

	memset(inv, 0, sizeof(*inv));
	for (i = 0; i < n; i++)
		inv->inst[i] = inst[i];
	inv->n = n;
}

/* Only these classes have a verified mutable layout in this model. Other
 * classes retain their existing compatibility handling in the message layer. */
bool omci_me_mutable(u16 class_id)
{
	switch (class_id) {
	case OMCI_ME_GEM_CTP:
	case OMCI_ME_PPTP_ETH_UNI:
	case OMCI_ME_TCONT:
	case OMCI_ME_UNI_G:
	/* The WAN service spine.  ME 50 is here with no modelled attribute:
	 * its INSTANCES are tracked (the ONU creates one per bridge port), so a
	 * Get must still validate the instance rather than answer about a
	 * bridge port that does not exist. */
	case OMCI_ME_MAC_BRIDGE_SVC:
	case OMCI_ME_MAC_BRIDGE_PORT:
	case OMCI_ME_MAC_BRIDGE_FILTER:
	case OMCI_ME_MAC_BRIDGE_TABLE:
	case OMCI_ME_MAC_BRIDGE_PM:
	case OMCI_ME_GEM_IW_TP:
	case OMCI_ME_GAL_ETH_PROF:
	/* The VLAN / classification half.  Every attribute stock declares is
	 * modelled for each of them -- including the LONG ones, through
	 * OMCI_SRC_TBL -- which is what makes the atomic Set safe here: an
	 * unmodelled mask bit would come back UNSUPPORTED and end the OLT's
	 * provisioning burst on the first VLAN row it wrote. */
	case OMCI_ME_VLAN_TAG_OP:
	case OMCI_ME_PREASSIGN_FILTER:
	case OMCI_ME_VLAN_TAG_FILTER:
	case OMCI_ME_PBIT_MAPPER:
	case OMCI_ME_EXT_VLAN:
	case OMCI_ME_GEM_TRAFFIC_DESC:
	case OMCI_ME_MCAST_GEM_IW_TP:
		return true;
	default:
		return false;
	}
}

void omci_me_reset_values(struct omci_onu *o)
{
	u16 i;

	omci_uni_reset(&o->pptp_eth_uni);
	omci_uni_reset(&o->uni_g);
	/* A MIB-Reset drops the provisioned store, and the ME 171 rows live
	 * OUTSIDE it: leaving them would serve the previous session's
	 * subscriber VLAN to an OLT that has just told us to forget
	 * everything. */
	gpon_vlan_model_reset(&o->vlan);
	o->tcont_alloc_written = 0;
	for (i = 0; i < OMCI_TCONT_COUNT; i++)
		o->tcont_alloc[i] = i ? 0x00ff : 0x0100;
}

/*
 * How many octets of DENSE attribute body @class_id's descriptor rows describe,
 * or 0 when the class has no store-backed row at all and its Create body stays
 * OPAQUE.  Derived from the table, so a width correction moves every reader.
 *
 * ⚠ A class whose rows would not fit omci_me_inst.body answers 0 and therefore
 *   falls back to the opaque path rather than writing past the buffer.  That is
 *   a coding error and not a runtime case: omci_service_spine_test's [a] arm
 *   asserts every dense class fits, so the day a row is added too wide the
 *   host gate says so instead of the store silently going opaque.
 */
u8 omci_me_dense_len(u16 class_id)
{
	const struct omci_attr *a;
	unsigned int end = 0;

	for (a = omci_me_find(class_id); a && a->class_id == class_id; a++)
		if (a->src == OMCI_SRC_STORE &&
		    (unsigned int)a->v + a->size > end)
			end = (unsigned int)a->v + a->size;
	return end > OMCI_STORE_BODY ? 0 : (u8)end;
}

/*
 * Create carries only SBC attributes, with no mask.  A class with store-backed
 * descriptor rows is EXPANDED into its dense layout before the instance is
 * committed; a class with none keeps the opaque body.
 *
 * ★ ME 268 REFUSES A SHORT BODY AND THE SERVICE-SPINE CLASSES DO NOT, and the
 *   asymmetry is deliberate.  ME 268's body ARMS THE DATAPATH -- the WAN data
 *   GEM decision reads its Port-ID, T-CONT pointer and direction -- so a
 *   truncated Create must not arm a GEM built out of zeros.  The spine classes
 *   drive no hardware, so a Create that stops early stores what arrived and
 *   leaves the rest at the zero G.988 already gives a null pointer; the
 *   resolver below then reports an UNRESOLVED link instead of inventing one.
 *   Refusing there would be the atomic-refusal failure this model already paid
 *   for on ME 11 -- one rc=9 and the OLT's provisioning burst ends.
 * ⚠ AND THE DISTINCTION IS UNREACHABLE FROM THE WIRE, which is worth saying so
 *   nobody reads an interop risk into it: a baseline PDU is 48 octets and the
 *   message layer hands the whole 40-octet tail over, so @blen is always 40
 *   there.  It matters only to a DIRECT caller -- a family shell snooping a
 *   Create body it obtained some other way.  Both halves are pinned by
 *   omci_service_spine_test case [i], which calls this function directly for
 *   exactly that reason.
 */
bool omci_store_create(struct omci_onu *o, u16 class_id, u16 inst,
		       const u8 *body, unsigned int blen)
{
	const struct omci_attr *a;
	u8 dense[OMCI_STORE_BODY] = { 0 };
	u8 dlen = omci_me_dense_len(class_id);
	unsigned int pos = 0;

	if (!dlen)
		return omci_store_put(o, class_id, inst, body,
				      blen > sizeof(dense) ? (int)sizeof(dense)
							   : (int)blen);
	if (class_id == OMCI_ME_GEM_CTP) {
		/* The stock default downstream priority-queue pointer is
		 * unassigned. */
		dense[10] = 0xff;
		dense[11] = 0xff;
	}
	for (a = omci_me_find(class_id); a && a->class_id == class_id; a++) {
		if (a->src != OMCI_SRC_STORE || !(a->access & 4))
			continue;
		if (!body || pos + a->size > blen) {
			if (class_id == OMCI_ME_GEM_CTP)
				return false;
			break;
		}
		memcpy(dense + a->v, body + pos, a->size);
		pos += a->size;
	}
	return omci_store_put(o, class_id, inst, dense, dlen);
}

/* Can this ONU actually realise the value the OLT asks for in THIS attribute?
 * The answer is per attribute and never per request, which is also why the
 * administrative bound moved here: it used to read values[0], the first octet
 * of the WHOLE request, and that is the administrative state only while ME 11
 * has exactly one writable attribute.  The moment a second one precedes it the
 * bound would have been checked against a stranger's octet. */
static bool omci_set_value_realisable(const struct omci_attr *a, const u8 *v)
{
	u32 want = 0;
	unsigned int i;

	/* An administrative state is locked or unlocked, and nothing else. */
	if ((a->class_id == OMCI_ME_PPTP_ETH_UNI && a->attr == 5) ||
	    (a->class_id == OMCI_ME_UNI_G && a->attr == 2))
		return v[0] <= 1;
	if (!(a->access & OMCI_ACCESS_WRITE_UNCHANGED))
		return true;
	if (a->size > sizeof(want))
		return false;		/* wider than we can compare: refuse */
	for (i = 0; i < a->size; i++)
		want = (want << 8) | v[i];
	return want == a->v;
}

/* Apply ONE already-validated attribute value. */
static void omci_set_apply_one(struct omci_onu *o, const struct omci_attr *a,
			       u16 inst, int pptp_slot, int unig_slot,
			       struct omci_me_inst *e, const u8 *v)
{
	if (a->access & OMCI_ACCESS_WRITE_UNCHANGED)
		return;			/* validated equal to what we serve */
	if (a->src == OMCI_SRC_STORE)
		memcpy(e->body + a->v, v, a->size);
	else if (a->src == OMCI_SRC_TBL) {
		/* TWO table writes this model HOLDS -- the subscriber VLAN and
		 * the per-bridge-port MAC filter; the rest are accepted so the
		 * burst completes and COUNTED so the acceptance is not a
		 * silence. */
		if (a->v == OMCI_TBL_EXT_VLAN_ROW)
			gpon_ext_vlan_row_set(&o->vlan, inst, v);
		else if (a->v == OMCI_TBL_MAC_FILTER_ROW)
			gpon_mac_filter_row_set(&o->vlan, inst, v);
		else
			o->vlan.attr_owed++;
	} else if (a->class_id == OMCI_ME_PPTP_ETH_UNI)
		omci_uni_accept(&o->pptp_eth_uni, pptp_slot, v[0]);
	else if (a->class_id == OMCI_ME_UNI_G)
		omci_uni_accept(&o->uni_g, unig_slot, v[0]);
	else {
		o->tcont_alloc[inst - 0x8000] = ((u16)v[0] << 8) | v[1];
		o->tcont_alloc_written |= (u16)(1u << (inst - 0x8000));
	}
}

/* Atomic masked Set: validate the entire request before changing any value.
 * Unsupported bits name absent attributes; failed bits name requested known
 * attributes left unapplied. This also reports the valid part of a rejected
 * mixed request as failed, rather than claiming partial application. */
u8 omci_me_set(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
	       const u8 *values, unsigned int len, u16 *unsupported, u16 *failed)
{
	const struct omci_attr *first = omci_me_find(class_id), *a;
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);
	/* Which inventory slot this Set addresses, or -1.  Resolved once, on the
	 * full u16, and it is what makes the attribute writable below. */
	int pptp_slot = class_id == OMCI_ME_PPTP_ETH_UNI ?
			omci_uni_slot(&o->pptp_eth_uni, inst) : -1;
	int unig_slot = class_id == OMCI_ME_UNI_G ?
			omci_uni_slot(&o->uni_g, inst) : -1;
	u8 dlen = omci_me_dense_len(class_id);
	u16 known = 0, writable = 0;
	unsigned int need = 0, pos = 0;

	for (a = first; a && a->class_id == class_id; a++) {
		u16 bit;

		if (!a->attr)
			continue;
		bit = (u16)OMCI_ATTR_BIT(a->attr);
		known |= bit;
		if ((a->access & 2) ||
		    (a->access & OMCI_ACCESS_WRITE_UNCHANGED) ||
		    /* ME 11 #5 and ME 264 #2, the administrative states.  Both
		     * are writable on EVERY instance this board declares, and
		     * on no other: the two clauses were pinned to the literal
		     * 0x0101, so a Set on the second, third or fourth UNI of
		     * either class was refused with the instance present in the
		     * MIB the OLT had just uploaded from us.
		     * ⚠ AND ME 264 WAS WORSE BEFORE THAT: until the class was
		     * modelled as mutable at all the Set never reached here,
		     * was ACKed, advanced the MIB-Data-Sync and stored
		     * nothing. */
		    (pptp_slot >= 0 && a->attr == 5) ||
		    (unig_slot >= 0 && a->attr == 2) ||
		    (class_id == OMCI_ME_TCONT && inst >= 0x8000 &&
		     inst < 0x8000 + OMCI_TCONT_COUNT && a->attr == 1))
			writable |= bit;
		if (mask & bit)
			need += a->size;
	}
	*unsupported = (u16)(mask & ~known);
	*failed = 0;
	/* A dense class is written THROUGH its instance, so the instance must
	 * exist and hold the full dense body its table describes -- otherwise
	 * omci_set_apply_one would write at a table offset into a shorter
	 * blob.  Derived from the table (16 for ME 268, as it always was). */
	if (*unsupported || (mask & ~writable) || need > len ||
	    (need && !values) ||
	    (dlen && (!e || e->blen != dlen))) {
		*failed = (u16)(mask & known);
		return OMCI_RC_ATTR_FAILED;
	}
	/* EVERY requested value is checked before ANY is applied — the whole
	 * point of an atomic Set, and the reason this is a separate pass. */
	for (a = first, pos = 0; a && a->class_id == class_id; a++) {
		if (!a->attr || !(mask & OMCI_ATTR_BIT(a->attr)))
			continue;
		if (!omci_set_value_realisable(a, values + pos)) {
			*failed = mask;
			return OMCI_RC_ATTR_FAILED;
		}
		pos += a->size;
	}
	for (a = first, pos = 0; a && a->class_id == class_id; a++) {
		if (!a->attr || !(mask & OMCI_ATTR_BIT(a->attr)))
			continue;
		omci_set_apply_one(o, a, inst, pptp_slot, unig_slot, e,
				   values + pos);
		pos += a->size;
	}
	return OMCI_RC_OK;
}

/* Does the ONU model this class at all (either a descriptor or the vendor
 * range policy)? */
bool omci_class_modelled(u16 class_id)
{
	return omci_me_find(class_id) || omci_vendor_class(class_id);
}

/* The bytes of one attribute.  Integers are big-endian, right-aligned in
 * @size octets; @scratch must hold 4 bytes. */
static const u8 *omci_attr_bytes(struct omci_onu *o,
				 const struct omci_attr *a, u16 inst,
				 u8 *scratch)
{
	u32 val;

	switch (a->src) {
	case OMCI_SRC_STORE: {
		struct omci_me_inst *e = omci_store_find(o,
						      a->class_id, inst);

		return e && a->v + a->size <= e->blen ? e->body + a->v : NULL;
	}
	case OMCI_SRC_TBL:
		/* A table attribute read as a scalar.  G.988 reads one with
		 * Get-Next and this model answers "end of table" there, so what
		 * a Get can honestly serve is a zero-filled field of the right
		 * WIDTH -- never a short one, which would misalign every
		 * attribute after it in the same response.  The widest such
		 * attribute in the model is 24 octets and omci_id.zeros is 25,
		 * which the host gate asserts rather than assumes. */
		return a->size <= OMCI_ID_SIZEOF(zeros) ? omci_id.zeros : NULL;
	case OMCI_SRC_ID:
		return (const u8 *)&omci_id + a->v;
	case OMCI_SRC_SN:
		return o->sn;
	case OMCI_SRC_MDS:
		val = o->mds;
		break;
	case OMCI_SRC_DYN:
		switch (a->v) {
		case OMCI_DYN_UNI_ADMIN:
			val = omci_uni_admin(&o->pptp_eth_uni, inst);
			break;
		case OMCI_DYN_UNIG_ADMIN:
			val = omci_uni_admin(&o->uni_g, inst);
			break;
		case OMCI_DYN_UNI_EXPECTED:
		case OMCI_DYN_UNI_TYPE: {
			int slot = omci_uni_slot(&o->pptp_eth_uni, inst);

			/* Equal today -- see the note on the selectors above. */
			val = slot < 0 ? OMCI_UNI_TYPE_DEFAULT : o->uni_type[slot];
			break;
		}
		case OMCI_DYN_UNIG_CAP: {
			int slot = omci_uni_slot(&o->uni_g, inst);

			val = slot < 0 ? 1 : o->uni_g_mgmt_cap[slot];
			break;
		}
		case OMCI_DYN_SW_VER:
			return inst ? omci_id.sw_bank1_version :
				      omci_id.sw_bank0_version;
		case OMCI_DYN_SW_FLAG:
			val = inst ? 0 : 1;
			break;
		case OMCI_DYN_TCONT_ALLOC:
			val = inst >= 0x8000 && inst < 0x8000 + OMCI_TCONT_COUNT ?
				o->tcont_alloc[inst - 0x8000] : 0x00ff;
			break;
		case OMCI_DYN_PQ_PORT:
			val = ((u32)0x0101 << 16) | (7u - (inst & 7));
			break;
		case OMCI_DYN_ANIG_RX:
			val = o->anig_rx_level;
			break;
		case OMCI_DYN_ANIG_TX:
			val = o->anig_tx_level;
			break;
		default:	/* OMCI_DYN_TS_TCONT */
			val = inst;
			break;
		}
		break;
	default:		/* OMCI_SRC_CONST */
		val = a->v;
		break;
	}
	scratch[0] = (u8)(val >> 24);
	scratch[1] = (u8)(val >> 16);
	scratch[2] = (u8)(val >> 8);
	scratch[3] = (u8)val;
	return scratch + 4 - a->size;
}

/*
 * ---- the ONE generic attribute filler ----
 * Shared by GET and MIB-Upload-Next so both byte-match.  @mask selects
 * attributes (bit15 = attr #1); the selected ones are emitted into [v..end)
 * in descriptor order, bounded.  An attribute that does not fit is SKIPPED and
 * a later smaller one may still be emitted (G.988 lets the reply carry what
 * fits and name the rest).
 *   *rmask_out = the attributes actually emitted,
 *   *known_out = every attribute this ME models, whether requested or not —
 *                which is what lets the caller distinguish "unsupported" from
 *                "did not fit" instead of answering success with a short mask.
 */
u8 omci_me_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
		u8 *v, const u8 *end, u16 *rmask_out, u16 *known_out)
{
	const struct omci_attr *a = omci_me_find(class_id);
	u16 rmask = 0, known = 0;
	bool over = false;

	*rmask_out = 0;
	*known_out = 0;
	if (!a)
		return omci_vendor_class(class_id) ? OMCI_RC_OK :
						     OMCI_RC_UNKNOWN_ME;

	for (; a->class_id == class_id; a++) {
		u16 bit;
		u8 scratch[4];
		const u8 *bytes;

		if (!a->attr)			/* marker row: no attributes */
			continue;
		bit = (u16)OMCI_ATTR_BIT(a->attr);
		known |= bit;
		if (!(mask & bit))
			continue;
		if (v + a->size > end) {
			over = true;
			continue;
		}
		bytes = omci_attr_bytes(o, a, inst, scratch);
		if (!bytes) {
			over = true;
			continue;
		}
		memcpy(v, bytes, a->size);
		v += a->size;
		rmask |= bit;
	}

	*rmask_out = rmask;
	*known_out = known;
	return over ? OMCI_RC_ATTR_FAILED : OMCI_RC_OK;
}

/*
 * Build the static MIB-Upload row table: every auto-instantiated hardware ME
 * the HSGQ-G008 OLT expects to read back, split so each row's attributes fit
 * the 26-byte Upload-Next value area.  The OLT counts the ME 11 instances to
 * classify the ONU as HGU; an empty upload loops its "ONU config load fail".
 * This table is also the ONU's statement of WHICH INSTANCES exist, so a Set of
 * an instance not listed here (and never created) is answered 0x05.
 */
static void omci_build_mib(struct omci_onu *o)
{
	u16 n = 0, dropped = 0;
	u16 i;

#define ROW(c, ins, m) do {						\
		if (n < OMCI_MIB_ROWS_MAX) {				\
			o->rows[n].class_id = (c);			\
			o->rows[n].inst = (ins);			\
			o->rows[n].mask = (m);				\
			n++;						\
		} else {						\
			dropped++;					\
		}							\
	} while (0)

	ROW(OMCI_ME_ONU_DATA, 0x0000, OMCI_ATTR_BIT(1));

	/* ME 256 ONU-G: 14 attrs split by the 26-byte cap:
	 * A = vid(4)+ver(14)+sn(8) = 26, B = #4..#9 = 6x1, C = LoID(24),
	 * D = #11(12)+#12(1)+#13(2)+#14(1) = 16. */
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(4) | OMCI_ATTR_BIT(5) |
				   OMCI_ATTR_BIT(6) | OMCI_ATTR_BIT(7) |
				   OMCI_ATTR_BIT(8) | OMCI_ATTR_BIT(9));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(10));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(11) | OMCI_ATTR_BIT(12) |
				   OMCI_ATTR_BIT(13) | OMCI_ATTR_BIT(14));

	/* ME 257 ONT2-G: A = EquipmentID(20), B = all scalars (21B). */
	ROW(OMCI_ME_ONU2_G, 0x0000, OMCI_ATTR_BIT(1));
	ROW(OMCI_ME_ONU2_G, 0x0000, OMCI_ATTR_BIT(2) | OMCI_ATTR_BIT(3) |
				    OMCI_ATTR_BIT(4) | OMCI_ATTR_BIT(5) |
				    OMCI_ATTR_BIT(6) | OMCI_ATTR_BIT(7) |
				    OMCI_ATTR_BIT(8) | OMCI_ATTR_BIT(9) |
				    OMCI_ATTR_BIT(10) | OMCI_ATTR_BIT(11) |
				    OMCI_ATTR_BIT(12) | OMCI_ATTR_BIT(13) |
				    OMCI_ATTR_BIT(14));

	ROW(OMCI_ME_CARDHOLDER, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					OMCI_ATTR_BIT(3));

	/* ME 6 Circuit-Pack: A = #1..#4 = 24B, B = #5(4)+#12(1) = 5B. */
	ROW(OMCI_ME_CIRCUIT_PACK, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					  OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_CIRCUIT_PACK, 0x0101, OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(12));

	/* ME 7 Software-Image x2 banks: A = ver+committed+active+valid = 17B,
	 * B = hash(16). */
	ROW(OMCI_ME_SW_IMAGE, 0x0000, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				      OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	/* ★ #6 Image hash (16 octets) is MODELLED and was never uploaded, so an
	 * OLT walking the MIB never learned it exists.  Its own row, beside #5's:
	 * 25 and 16 each need one. */
	ROW(OMCI_ME_SW_IMAGE, 0x0000, OMCI_ATTR_BIT(5));
	ROW(OMCI_ME_SW_IMAGE, 0x0000, OMCI_ATTR_BIT(6));
	ROW(OMCI_ME_SW_IMAGE, 0x0001, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				      OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_SW_IMAGE, 0x0001, OMCI_ATTR_BIT(5));
	ROW(OMCI_ME_SW_IMAGE, 0x0001, OMCI_ATTR_BIT(6));

	/* ME 11 PPTP Ethernet UNI: #1..#15 = 17B, one row per declared
	 * instance.  THE HGU GATE. */
	for (i = 0; i < o->pptp_eth_uni.n; i++)
		ROW(OMCI_ME_PPTP_ETH_UNI, o->pptp_eth_uni.inst[i],
					  OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					  OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
					  OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(6) |
					  OMCI_ATTR_BIT(7) | OMCI_ATTR_BIT(8) |
					  OMCI_ATTR_BIT(9) | OMCI_ATTR_BIT(10) |
					  OMCI_ATTR_BIT(11) | OMCI_ATTR_BIT(12) |
					  OMCI_ATTR_BIT(13) | OMCI_ATTR_BIT(14) |
					  OMCI_ATTR_BIT(15));

	ROW(OMCI_ME_OLT_G, 0x0000, 0x0000);

	/* ME 263 ANI-G: A = #1..#9 = 11B, B = #10..#16 = 10B. */
	ROW(OMCI_ME_ANI_G, 0x8001, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
				   OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(6) |
				   OMCI_ATTR_BIT(7) | OMCI_ATTR_BIT(8) |
				   OMCI_ATTR_BIT(9));
	ROW(OMCI_ME_ANI_G, 0x8001, OMCI_ATTR_BIT(10) | OMCI_ATTR_BIT(11) |
				   OMCI_ATTR_BIT(12) | OMCI_ATTR_BIT(13) |
				   OMCI_ATTR_BIT(14) | OMCI_ATTR_BIT(15) |
				   OMCI_ATTR_BIT(16));

	/* ME 262 T-CONT (inst 0x8000..0x800b): 4B each. */
	for (i = 0; i < OMCI_TCONT_COUNT; i++)
		ROW(OMCI_ME_TCONT, 0x8000 + i, OMCI_ATTR_BIT(1) |
					       OMCI_ATTR_BIT(2) |
					       OMCI_ATTR_BIT(3));

	/* ME 264 UNI-G, one row per declared instance.  Its inventory is its
	 * OWN: the X400AXF reports a UNI-G the PPTP list does not carry. */
	for (i = 0; i < o->uni_g.n; i++)
		ROW(OMCI_ME_UNI_G, o->uni_g.inst[i],
				   OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
				   OMCI_ATTR_BIT(5));

	/* ME 277 Priority-Queue: only the single UNI's 8 queues.  The full
	 * 96-row stock set made the upload so long the OLT's auth timer
	 * deactivated us mid-config (proven on the 9602C).
	 *
	 * ⚠ THE 96 IS UNSOURCED AND THE SHIPPED DATA SAYS 64 (measured
	 * 2026-08-31, OMCI-simulate/mib_init_pair.py): stock's own
	 * /etc/omci_mib.cfg declares exactly 64 ME 277 records, 0xff00..0xff3f,
	 * BYTE-IDENTICAL on the X111W and the G24W.  It is NOT corrected to 64
	 * here, and that restraint is the point: stock has a SECOND creation
	 * path (MIB_Set -> mib_AddEntry, from OMCI_ResetMib /
	 * omci_mib_cfg_setup_me) that nobody has decoded, so 96 may well be the
	 * live total and 64 only the file-declared part.  Changing the number to
	 * match the half we can read would be inventing a measurement.
	 * What settles it: count ME 277 instances in a live MIB-Upload from
	 * stock.  The DECISION above is unaffected either way -- 64 and 96 are
	 * both far more than 8, and the deactivation was observed. */
	for (i = 0; i < 8; i++)
		ROW(OMCI_ME_PRIORITY_QUEUE, i, OMCI_ATTR_BIT(1) |
					       OMCI_ATTR_BIT(2) |
					       OMCI_ATTR_BIT(3) |
					       OMCI_ATTR_BIT(4) |
					       OMCI_ATTR_BIT(5) |
					       OMCI_ATTR_BIT(6) |
					       OMCI_ATTR_BIT(7) |
					       OMCI_ATTR_BIT(8));

	/* ME 278 Traffic-Scheduler (inst 0x8000..0x800b): 6B each. */
	for (i = 0; i < 12; i++)
		ROW(OMCI_ME_TRAFFIC_SCHED, 0x8000 + i, OMCI_ATTR_BIT(1) |
						       OMCI_ATTR_BIT(2) |
						       OMCI_ATTR_BIT(3) |
						       OMCI_ATTR_BIT(4));

	/* ★★ THE OTHER HALF OF THE #3/#4 CORRECTION ABOVE, AND IT WAS MISSING.
	 * The attribute table was fixed against a real vendor plugin -- #3 is the
	 * 25-octet Interdomain name, #4 the 2-octet TCP/UDP pointer -- and THIS
	 * ROW was left as it had been computed when #3 was 2 octets.  So it went
	 * on declaring 1|2|3 while 1+1+25 = 27 needs a 26-octet payload: the ONU
	 * promised the OLT three attributes and could serve two.
	 *
	 * ⚠ WHAT THAT COSTS IS NOT COSMETIC.  An upload row whose returned mask
	 * is short of its requested mask is the proven OLT re-GET churn-lock
	 * class -- the OLT keeps asking for what it was told is there.  The host
	 * case says so in its own words: "row mask 0xe000 but only 0xc000
	 * servable (OLT re-GET loop)".
	 *
	 * Split so each row FITS: 1+2+4 = 4 octets, and #3 alone = 25.  #4 was
	 * not uploaded at all before, so this also stops modelling an attribute
	 * the OLT could never see. */
	ROW(OMCI_ME_VEIP, 0x0601, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				  OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_VEIP, 0x0601, OMCI_ATTR_BIT(3));

	/*
	 * ME 65530 (CTC LoID authentication) is deliberately NOT uploaded.
	 * Stock models all four of its attributes (#1 Operation ID 4B, #2 LoID
	 * 24B, #3 Password 12B, #4 Auth status 1B) and answers a Get on every
	 * one of them, but keeps the whole CLASS out of the MIB upload: its
	 * table descriptor carries stdType 0x104, and both MIB-Upload walkers
	 * (row count and row packing alike) skip a table whose stdType has bit
	 * 0x10 or 0x100 set.  The exclusion is per class and all-or-nothing --
	 * the four attributes' optionType is 1, so none of them is filtered by
	 * the separate per-attribute optionType & 0x31A rule.  Uploading a
	 * SUBSET (what this used to do: #1|#4 then #2, dropping the 12-byte #3)
	 * matches neither stock nor a complete upload and leaves the OLT with a
	 * MIB copy the ONU can answer beyond.  A Get keeps working with no row:
	 * omci_inst_exists() short-circuits on omci_vendor_class().
	 */

#undef ROW
	o->nrows = n;
	o->rows_dropped = dropped;
}

bool omci_onu_declare_unis(struct omci_onu *o,
			   const u16 *pptp_inst, const u8 *pptp_type, u8 pptp_n,
			   const u16 *unig_inst, const u8 *unig_mgmt_cap,
			   u8 unig_n)
{
	/* The panel as it stands, so a refusal can put it back EXACTLY -- the
	 * accepted administrative states and the undrained obligations
	 * included.  Re-declaring the old instance ids would not do: that
	 * resets both, and "nothing changed" would be false about the half
	 * that matters to a port.  Three small inventories, ~64 bytes of
	 * stack; the enclosing model is far too big to copy here. */
	struct omci_uni_inv was_pptp = o->pptp_eth_uni, was_unig = o->uni_g;
	u8 was_cap[OMCI_UNI_MAX], was_type[OMCI_UNI_MAX];
	u8 i;

	if (!omci_uni_list_ok(pptp_inst, pptp_n) ||
	    !omci_uni_list_ok(unig_inst, unig_n))
		return false;
	memcpy(was_cap, o->uni_g_mgmt_cap, sizeof(was_cap));
	memcpy(was_type, o->uni_type, sizeof(was_type));
	omci_uni_install(&o->pptp_eth_uni, pptp_inst, pptp_n);
	omci_uni_install(&o->uni_g, unig_inst, unig_n);
	for (i = 0; i < OMCI_UNI_MAX; i++) {
		o->uni_g_mgmt_cap[i] = i < unig_n && unig_mgmt_cap ?
				       unig_mgmt_cap[i] : 1;
		o->uni_type[i] = i < pptp_n && pptp_type ?
				 pptp_type[i] : OMCI_UNI_TYPE_DEFAULT;
	}
	omci_build_mib(o);
	if (o->rows_dropped) {
		/* ⚠ A PANEL THAT DOES NOT FIT IS PUT BACK, NOT LEFT INSTALLED.
		 * Rows the OLT never uploaded are instances it will never
		 * provision, so a half-sized panel is worse than the one that
		 * was there -- and this function's contract, which the family
		 * wrappers log, is that a refusal changes NOTHING. */
		o->pptp_eth_uni = was_pptp;
		o->uni_g = was_unig;
		memcpy(o->uni_g_mgmt_cap, was_cap, sizeof(was_cap));
		memcpy(o->uni_type, was_type, sizeof(was_type));
		omci_build_mib(o);
		return false;
	}
	return true;
}

/* The inventory every board on the bench has at minimum, and exactly what this
 * model carried before the inventory existed. */
static const u16 omci_uni_default[] = { 0x0101 };

/* One declared 16-bit list: even byte count, at most OMCI_UNI_MAX entries,
 * decoded big-endian.  Empty is legal and means "this board has none". */
static bool omci_uni_decode_list(const void *be, int len, u16 *out, u8 *n)
{
	const u8 *b = be;
	int i;

	if (len < 0 || (len & 1) || len > 2 * OMCI_UNI_MAX || (len && !b))
		return false;
	for (i = 0; i < len / 2; i++)
		out[i] = (u16)((b[2 * i] << 8) | b[2 * i + 1]);
	*n = (u8)(len / 2);
	return true;
}

enum omci_uni_decl omci_onu_declare_unis_be(struct omci_onu *o,
					    const void *pptp_be, int pptp_len,
					    const void *type, int type_len,
					    const void *unig_be, int unig_len,
					    const void *cap, int cap_len,
					    const char **why)
{
	u16 pptp[OMCI_UNI_MAX], unig[OMCI_UNI_MAX];
	u8 caps[OMCI_UNI_MAX], types[OMCI_UNI_MAX];
	u8 pptp_n = 0, unig_n = 0;
	const char *unused;
	int i;

	if (!why)
		why = &unused;
	if (pptp_len < 0 && unig_len < 0) {
		/* ⚠ A CAPABILITY LIST ON ITS OWN IS NOT "nothing declared".  It is
		 * a node that says something about UNI-Gs it never listed, and
		 * discarding it silently would accept a panel nobody can read. */
		if (cap_len >= 0) {
			*why = "a management-capability list with no UNI-G list";
			return OMCI_UNI_DECL_BAD;
		}
		if (type_len >= 0) {
			*why = "an Ethernet-UNI type list with no UNI list";
			return OMCI_UNI_DECL_BAD;
		}
		*why = "no panel declared";
		return OMCI_UNI_DECL_ABSENT;
	}
	/* Half a declaration is refused rather than completed: a board that
	 * lists its Ethernet UNIs and not its UNI-Gs has not said it has none,
	 * it has left one out. */
	if (pptp_len < 0 || unig_len < 0) {
		*why = "one UNI class declared and not the other";
		return OMCI_UNI_DECL_BAD;
	}
	if (!omci_uni_decode_list(pptp_be, pptp_len, pptp, &pptp_n)) {
		*why = "the Ethernet-UNI list is not an even count of 16-bit instances, or holds more than the model does";
		return OMCI_UNI_DECL_BAD;
	}
	if (!omci_uni_decode_list(unig_be, unig_len, unig, &unig_n)) {
		*why = "the UNI-G list is not an even count of 16-bit instances, or holds more than the model does";
		return OMCI_UNI_DECL_BAD;
	}
	for (i = 0; i < OMCI_UNI_MAX; i++) {
		caps[i] = 1;
		types[i] = OMCI_UNI_TYPE_DEFAULT;
	}
	if (type_len < 0) {
		/* absent: every UNI keeps the type this model used to hardcode */
	} else if (type_len != pptp_n || (type_len && !type)) {
		*why = "the Ethernet-UNI type list is present and is not one byte per Ethernet UNI";
		return OMCI_UNI_DECL_BAD;
	} else {
		for (i = 0; i < type_len && i < OMCI_UNI_MAX; i++)
			types[i] = ((const u8 *)type)[i];
	}
	if (cap_len < 0) {
		/* absent: every UNI-G keeps the 1 the model used to hardcode */
	} else if (cap_len != unig_n || (cap_len && !cap)) {
		*why = "the management-capability list is present and is not one byte per UNI-G";
		return OMCI_UNI_DECL_BAD;
	} else {
		/* Bounded by the ARRAY as well as by the equality above: a rule
		 * enforced only somewhere else is one a later edit can remove,
		 * and this loop writes into a fixed-size object. */
		for (i = 0; i < cap_len && i < OMCI_UNI_MAX; i++)
			caps[i] = ((const u8 *)cap)[i];
	}
	if (!omci_onu_declare_unis(o, pptp, types, pptp_n, unig, caps, unig_n)) {
		*why = "a duplicate or zero instance, or more MIB rows than the table holds";
		return OMCI_UNI_DECL_BAD;
	}
	*why = "declared";
	return OMCI_UNI_DECL_OK;
}

void omci_onu_init(struct omci_onu *o, const u8 sn[8], u8 mds_seed)
{
	/* ⚠ THIS READS NOTHING OUT OF @o.  It is a COLD init: callers hand it
	 * an object that has never been initialised -- several hand it an
	 * uninitialised stack struct -- so touching a field before the memset
	 * is an indeterminate read, not a way to keep the inventory.  Carrying
	 * the inventory across an identity change is omci_onu_reinit()'s job,
	 * and it is a separate function precisely so the cold path cannot try. */
	memset(o, 0, sizeof(*o));
	memcpy(o->sn, sn, 8);
	omci_me_reset_values(o);
	o->mds = mds_seed;
	/* Seed the ANI-G optical levels with the static fallback: a fresh MIB must
	 * be able to answer an ANI-G GET before the first DDM sample lands (the
	 * OLT audits within seconds of O5).  anig_live stays false until the shell
	 * publishes a real measurement. */
	/* the walk ships ON with the measured threshold; a shell may override
	 * either field after init for a bisect */
	o->mds_adapt = true;
	o->mds_adapt_reads = OMCI_MDS_ADAPT_READS;
	o->anig_rx_level = OMCI_ANIG_RX_FALLBACK;
	o->anig_tx_level = OMCI_ANIG_TX_FALLBACK;
	omci_onu_declare_unis(o, omci_uni_default, NULL, 1,
			      omci_uni_default, NULL, 1);
}

/* Everything the family still owes a port after the model was reset under it:
 * every slot that WAS locked, because the reset only unlocks the report, plus
 * anything that was flagged and not yet drained, because a later same-value
 * Set or a replayed PDU will never recreate a consumed bit. */
static void omci_uni_carry_obligations(struct omci_uni_inv *now,
				       const struct omci_uni_inv *was)
{
	u8 i;

	now->changed |= was->changed;
	for (i = 0; i < now->n && i < was->n; i++)
		if (was->admin[i])
			now->changed |= (u8)(1u << i);
}

void omci_onu_reinit(struct omci_onu *o, const u8 sn[8], u8 mds_seed)
{
	/* Safe to read: @o is LIVE here by this function's contract, which is
	 * the whole reason it is not the same function as omci_onu_init(). */
	struct omci_uni_inv pptp = o->pptp_eth_uni, unig = o->uni_g;
	u8 cap[OMCI_UNI_MAX], type[OMCI_UNI_MAX];

	memcpy(cap, o->uni_g_mgmt_cap, sizeof(cap));
	/* ★ THE TYPE IS CARRIED TOO.  It is the panel's, not the session's: an
	 *   identity change does not turn an FE port into a GE one. */
	memcpy(type, o->uni_type, sizeof(type));
	omci_onu_init(o, sn, mds_seed);
	/* ⚠ UNCONDITIONALLY, INCLUDING WITH BOTH LISTS EMPTY.  Guarding this on
	 * "something was declared" made a board that legally declares no UNI at
	 * all come back with the default 0x0101 on both classes -- an instance
	 * the board does not have, invented by the identity change. */
	omci_onu_declare_unis(o, pptp.inst, type, pptp.n, unig.inst, cap, unig.n);
	omci_uni_carry_obligations(&o->pptp_eth_uni, &pptp);
	omci_uni_carry_obligations(&o->uni_g, &unig);
}

/* Is (class, inst) a MIB instance this ONU holds?  Three sources: the static
 * auto-instantiated set (== the MIB-Upload rows, which is what the OLT learned
 * from us), any vendor-reserved class (we model no attributes but the OLT is
 * entitled to address them), and anything the OLT itself created. */
bool omci_inst_exists(struct omci_onu *o, u16 class_id, u16 inst)
{
	u16 i;

	if (omci_vendor_class(class_id))
		return true;
	if (omci_store_find(o, class_id, inst))
		return true;
	for (i = 0; i < o->nrows; i++)
		if (o->rows[i].class_id == class_id && o->rows[i].inst == inst)
			return true;
	return false;
}

/* ========================================================================
 * THE WAN DATA GEM -- which ME 268 the OLT meant for user traffic.
 *
 * G.988 clause 9.2.3, ME 268 (GEM Port Network CTP), Set-by-Create body, as
 * the store holds it (attribute 1 at body[0], i.e. the wire from octet 8):
 *
 *   body[0..1]  attr 1  GEM Port-ID          (12 significant bits, G.984.3)
 *   body[2..3]  attr 2  T-CONT pointer       (ME 262 instance)
 *   body[4]     attr 3  direction            1=US, 2=DS, 3=bidirectional
 *
 * ★ THE DIRECTION TEST IS THE LOAD-BEARING ONE, and it is why this may not be
 *   a first-match scan of class 268. On the lab OLT the FIRST ME 268 Create is
 *   the DS-only broadcast CTP (Port-ID 4095, T-CONT ptr 0, dir 2); the WAN one
 *   arrives afterwards. A scan without the test adopts the broadcast port and
 *   points the WAN at it -- which is exactly what the pre-2026-08-27 Luna
 *   snoop did, guarded only by a multicast Port-ID literal that a different
 *   OLT need not use.
 *
 * ★ AND THE ORDER OF THE REFUSALS IS DELIBERATE: shape (RUNT) before value
 *   (ZERO) before identity (OMCC/MCAST) before semantics (NOT_BIDIR), so the
 *   reported reason is always the FIRST thing wrong rather than whichever test
 *   happened to be written last.
 * ======================================================================== */

enum omci_dgem omci_dgem_classify(const u8 *body, u8 blen,
				  u16 omcc_gem, u16 mcast_gem, u16 *port_id)
{
	u16 g;

	/* attr 1..3 need 5 octets; a runt Create carries no direction and must
	 * never be read past -- an unfuzzable implicit length is precisely what
	 * this project refuses. */
	if (!body || blen < 5)
		return OMCI_DGEM_RUNT;

	/* explicit byte math: big-endian on the wire, and this same source is
	 * compiled for MIPS-BE, ARM64-LE and x86. */
	g = (u16)(((u16)body[0] << 8) | body[1]) & 0x0fff;

	if (!g)
		return OMCI_DGEM_ZERO;
	if (g == (omcc_gem & 0x0fff))
		return OMCI_DGEM_IS_OMCC;
	if (g == (mcast_gem & 0x0fff))
		return OMCI_DGEM_IS_MCAST;
	if (body[4] != GPON_GEM_BIDIR)
		return OMCI_DGEM_NOT_BIDIR;

	if (port_id)
		*port_id = g;
	return OMCI_DGEM_YES;
}

const char *omci_dgem_name(enum omci_dgem v)
{
	switch (v) {
	case OMCI_DGEM_YES:		return "data GEM";
	case OMCI_DGEM_RUNT:		return "runt Create";
	case OMCI_DGEM_ZERO:		return "Port-ID 0";
	case OMCI_DGEM_IS_OMCC:		return "the OMCC GEM";
	case OMCI_DGEM_IS_MCAST:	return "the multicast GEM";
	case OMCI_DGEM_NOT_BIDIR:	return "uni-directional CTP";
	}
	return "?";
}

void omci_data_binding_snapshot(const struct omci_onu *o, u16 omcc_gem,
				u16 mcast_gem, struct omci_data_binding *binding)
{
	u16 i;

	if (!binding)
		return;
	memset(binding, 0, sizeof(*binding));
	if (!o)
		return;
	for (i = 0; i < OMCI_STORE_MAX; i++) {
		const struct omci_me_inst *e = &o->store[i];
		u16 port_id, tcont;

		if (!e->used || e->class_id != OMCI_ME_GEM_CTP ||
		    omci_dgem_classify(e->body, e->blen, omcc_gem, mcast_gem,
				       &port_id) != OMCI_DGEM_YES)
			continue;
		binding->gem_present = true;
		binding->gem_inst = e->inst;
		binding->gem_port = port_id;
		binding->direction = e->body[4];
		tcont = ((u16)e->body[2] << 8) | e->body[3];
		binding->tcont_inst = tcont;
		if (tcont >= 0x8000 && tcont < 0x8000 + OMCI_TCONT_COUNT &&
		    (o->tcont_alloc_written & (1u << (tcont - 0x8000)))) {
			binding->alloc_known = true;
			binding->alloc_id = o->tcont_alloc[tcont - 0x8000];
		}
		return;
	}
}

bool omci_data_gem_port(struct omci_onu *o, u16 omcc_gem, u16 mcast_gem,
			u16 *port_id)
{
	struct omci_data_binding binding;

	omci_data_binding_snapshot(o, omcc_gem, mcast_gem, &binding);
	if (binding.gem_present && port_id)
		*port_id = binding.gem_port;
	return binding.gem_present;
}

/* ME 262 T-CONT snoop: the parse (Set-with-mask vs Create SBC layout) and the
 * against-the-shadow decision, hoisted from cortina-gpon.c cg_rx_omci Stage D
 * bit-for-bit (its guard was len >= 12, i.e. blen >= 4 -- kept exactly, so a
 * 2-octet Create is refused, never read). */
enum omci_tcont_verdict omci_tcont_snoop(u8 mt, const u8 *body,
					 unsigned int blen, u16 inst,
					 u16 cur_alloc, u16 cur_inst,
					 u16 *alloc)
{
	u8 m = mt & 0x1f;
	u16 a = 0;

	if (!body || blen < 4)
		return OMCI_TCONT_NONE;

	if (m == OMCI_MT_SET && ((((u16)body[0] << 8) | body[1]) & 0x8000))
		a = ((u16)body[2] << 8) | body[3];	/* attr 1, via the mask */
	else if (m == OMCI_MT_CREATE)
		a = ((u16)body[0] << 8) | body[1];	/* SBC: alloc first */

	/* ★★ 0xffff IS THE G.988 DEALLOCATE, NOT NOISE (2026-08-05).  An
	 * `a != 0xffff` filter alone DROPPED it, so an OLT that detached the
	 * T-CONT the standard way left the shell's shadow -- and therefore the
	 * armed HW T-CONT CAM -- still matching an alloc-id the OLT was free
	 * to hand to ANOTHER subscriber.  Only a MIB-Reset cleared it.  It is
	 * the teardown half of the same message, and it is decided here so a
	 * third board cannot re-lose it. */
	if (a == 0xffff && cur_alloc && (!cur_inst || inst == cur_inst))
		return OMCI_TCONT_DEALLOC;
	if (a && a != 0xffff && a != cur_alloc) {
		if (alloc)
			*alloc = a;
		return OMCI_TCONT_ALLOC;
	}
	return OMCI_TCONT_NONE;
}

bool omci_dgem_delete(u8 mt, u16 inst, u16 cur_gem, u16 cur_inst)
{
	if ((mt & 0x1f) != OMCI_MT_DELETE)
		return false;
	return cur_gem && (!cur_inst || inst == cur_inst);
}

/* ========================================================================
 * THE WAN SERVICE SPINE -- where the OLT said this GEM port GOES.
 *
 * ★★★ WHY IT EXISTS.  We model ME 268, so we know the OLT named a GEM port.
 *     G.988 clause 9.3 says nothing about what that port is FOR until the
 *     chain is followed:
 *
 *         ME 268 GEM port network CTP   -- names the Port-ID
 *           ^  ME 266 #1
 *         ME 266 GEM interworking TP    -- interworks it, points at a GAL
 *           ^  ME 47 #4                    profile (ME 272) for the payload
 *         ME 47  MAC bridge port config -- hangs it on a bridge port
 *           |  ME 47 #1
 *         ME 45  MAC bridge service prof-- the bridge itself
 *
 *     Every one of those links is a POINTER the OLT wrote, and until the six
 *     classes were modelled there was nothing to follow: a Create was ACKed
 *     and its body kept opaque, so the ONU could not have said which bridge
 *     the OLT put its WAN on even in principle.
 *
 * ★ IT DECIDES AND NEVER DOES.  A pure read over the store: no MMIO, no
 *   allocation, no clock, no op table.  What a family does with the answer --
 *   and whether it installs anything at all -- is the family's, and today no
 *   family calls it: our WAN is an untagged IPoE interface built from uci and
 *   the GEM install is gated on ME 268 + ME 262 + PLOAM, unchanged by this.
 *   ⇒ this reports the OLT's INTENT so the two can be COMPARED; it does not
 *   yet act on it, and saying otherwise would be claiming a datapath repair
 *   nobody measured.
 *
 * ★ NO ATTRIBUTE OFFSET IS SPELLED HERE and no TP-type numeral is branched on.
 *   Values come through the descriptor table, and the chain is followed by
 *   matching POINTERS, so nothing in this code can go wrong about a G.988
 *   attribute-type coding nobody on this bench has measured.  ME 47 #3 is
 *   carried out as DATA for a caller that has its own evidence for it.
 * ======================================================================== */

/* One attribute of a provisioned instance, as a host integer.  False when the
 * class is not dense, the instance is absent, or the attribute is not modelled
 * -- three different "no", all of which mean the chain does not resolve. */
static bool omci_store_attr(struct omci_onu *o, u16 class_id, u16 inst,
			    u8 attr, u32 *out)
{
	const struct omci_attr *a;
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);
	unsigned int i;
	u32 v = 0;

	if (!e)
		return false;
	for (a = omci_me_find(class_id); a && a->class_id == class_id; a++) {
		if (a->attr != attr || a->src != OMCI_SRC_STORE)
			continue;
		if (a->size > sizeof(v) || (unsigned int)a->v + a->size > e->blen)
			return false;
		for (i = 0; i < a->size; i++)
			v = (v << 8) | e->body[a->v + i];
		*out = v;
		return true;
	}
	return false;
}

/* The first instance of @class_id whose attribute @attr holds @val. */
static bool omci_store_by_attr(struct omci_onu *o, u16 class_id, u8 attr,
			       u32 val, u16 *inst_out)
{
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++) {
		u32 got;

		if (!o->store[k].used || o->store[k].class_id != class_id)
			continue;
		if (omci_store_attr(o, class_id, o->store[k].inst, attr, &got) &&
		    got == val) {
			*inst_out = o->store[k].inst;
			return true;
		}
	}
	return false;
}

u8 omci_service_resolve(struct omci_onu *o, u16 gem_port,
			struct omci_service_path *p)
{
	u32 v;

	if (!o || !p)
		return 0;
	memset(p, 0, sizeof(*p));
	p->gem_port = gem_port;

	if (!omci_store_by_attr(o, OMCI_ME_GEM_CTP, 1, gem_port, &p->gem_ctp))
		return p->have;
	p->have |= OMCI_SVC_GEM_CTP;

	if (!omci_store_by_attr(o, OMCI_ME_GEM_IW_TP, 1, p->gem_ctp, &p->iw_tp))
		return p->have;
	p->have |= OMCI_SVC_IW_TP;
	if (omci_store_attr(o, OMCI_ME_GEM_IW_TP, p->iw_tp, 2, &v))
		p->iw_option = (u16)v;
	if (omci_store_attr(o, OMCI_ME_GEM_IW_TP, p->iw_tp, 7, &v)) {
		p->gal_prof = (u16)v;
		if (omci_store_attr(o, OMCI_ME_GAL_ETH_PROF, p->gal_prof, 1, &v)) {
			p->gal_payload = (u16)v;
			p->have |= OMCI_SVC_GAL;
		}
	}

	if (!omci_store_by_attr(o, OMCI_ME_MAC_BRIDGE_PORT, 4, p->iw_tp,
				&p->bridge_port)) {
		/* ★ THE MAPPER ROUTE.  No bridge port names this ME 266
		 *   directly, so look for an 802.1p mapper service profile that
		 *   does: ME 130's eight P-bit pointers are attributes #2..#9,
		 *   and a bridge port then points at the MAPPER instead. */
		u8 pbit;

		for (pbit = 2; pbit <= 9; pbit++)
			if (omci_store_by_attr(o, OMCI_ME_PBIT_MAPPER, pbit,
					       p->iw_tp, &p->mapper))
				break;
		if (pbit > 9 ||
		    !omci_store_by_attr(o, OMCI_ME_MAC_BRIDGE_PORT, 4,
					p->mapper, &p->bridge_port)) {
			p->mapper = 0;
			return p->have;
		}
		p->pbit = (u8)(pbit - 2);	/* #2 is P-bit 0 */
		p->have |= OMCI_SVC_PBIT_MAPPER;
	}
	p->have |= OMCI_SVC_BRIDGE_PORT;
	if (omci_store_attr(o, OMCI_ME_MAC_BRIDGE_PORT, p->bridge_port, 3, &v))
		p->tp_type = (u16)v;

	if (!omci_store_attr(o, OMCI_ME_MAC_BRIDGE_PORT, p->bridge_port, 1, &v))
		return p->have;
	p->bridge = (u16)v;
	/* ★ THE BRIDGE MUST EXIST, not merely be POINTED AT.  An ME 47 whose
	 *   bridge-ID pointer names an ME 45 the OLT never created is a dangling
	 *   link, and reporting it as a resolved bridge would be this model
	 *   asserting a service the OLT has not finished expressing. */
	if (omci_store_find(o, OMCI_ME_MAC_BRIDGE_SVC, p->bridge))
		p->have |= OMCI_SVC_BRIDGE;
	return p->have;
}

u16 omci_service_bridge_of_gem(struct omci_onu *o, u16 gem_port)
{
	struct omci_service_path p;

	if (omci_service_resolve(o, gem_port, &p) & OMCI_SVC_BRIDGE)
		return p.bridge;
	return 0;
}
