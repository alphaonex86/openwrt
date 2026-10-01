// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE (prefix gpon_) — protocol only. NEVER touches ...
 * dev/MEASURED-gpon_omci_me.c.md sec 1. */
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

/* Insert one OLT-created instance. Returns false when the ...
 * dev/MEASURED-gpon_omci_me.c.md sec 2. */
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

/* Apply a Set's attribute values to a provisioned instance. ...
 * dev/MEASURED-gpon_omci_me.c.md sec 3. */
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

/* ME attribute model ---- Constant attribute bytes live in ...
 * dev/MEASURED-gpon_omci_me.c.md sec 4. struct omci_identity is in the header:
 * each ONU carries its own copy, seeded from this build default. */
static const struct omci_identity omci_id_default_bytes = {
	/* The HSGQ-G008 OLT recognizes HSGQ ONUs; "XPON" was rejected. */
	.vendor_id	  = { 'H', 'S', 'G', 'Q' },
	.onu_g_version	  = { '0', '2', 'A', '5', 'B', '1' },
	.sw_bank0_version = { 'M', '2', '2', '5', '-',
			      '2', '6', '0', '5', '2', '5' },
	.sw_bank1_version = { 'M', '2', '2', '5', '-',
			      '2', '6', '0', '5', '1', '5' },
	.equipment_id	  = { 'H', 'S', 'G', 'Q', '-',
			      'X', '4', '1', '1', 'A', 'X', 'F' },
	.product_code	  = { 0x00, 0x31 },
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
OMCI_ID_ASSERT(product_code, 2);	/* ONU2-G #3 */
OMCI_ID_ASSERT(loid, 24);		/* ONU-G #10 / CTC LoID #2 */
OMCI_ID_ASSERT(loid_passwd, 12);	/* ONU-G #11 / CTC LoID #3 */
OMCI_ID_ASSERT(operator_id, 4);		/* CTC #1 */
OMCI_ID_ASSERT(zeros, 25);		/* == its longest consumer, VEIP #3 */
/* every member is a u8 array, so equality here also proves no padding crept
 * in: the flat 119-byte pool plus product code (2) and LoID password (12) */
_Static_assert(sizeof(struct omci_identity) == 133,
	       "omci_identity is not the 133-byte pool the OLT reads");

/* Where a descriptor row takes its value bytes from. */
enum omci_attr_src {
	OMCI_SRC_CONST,		/* v = the value, big-endian in `size` bytes */
	OMCI_SRC_ID,		/* v = offsetof() into struct omci_identity */
	OMCI_SRC_SN,		/* the board serial number (8) */
	OMCI_SRC_MDS,		/* ME 2 #1 = the live MIB-Data-Sync */
	OMCI_SRC_DYN,		/* v = enum omci_attr_dyn */
	OMCI_SRC_STORE,		/* v = dense offset in the dynamic instance */
	/* A LONG attribute the 26-octet dense body cannot hold: ME ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 6. */
	OMCI_SRC_TBL,
	/* ★★ A COUNTER THIS MODEL DECLARES AND THIS ONU CANNOT ESTABLISH.
	 * It occupies its dense slot (so the layout stays stock's) and serves
	 * NO VALUE: the Get leaves its bit out of the response mask, which puts
	 * it in the ATTRIBUTES-FAILED word -- G.988's own "could not ask".
	 * A missing counter rendered as 0 is a measurement nobody took. */
	OMCI_SRC_CNT,
	OMCI_SRC_ANIG_THRESHOLD,
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
	/* ★★ TWO SELECTORS FOR ONE VALUE, ON PURPOSE. #1 Expected and ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 7. */
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
	/* ME 5 / ME 6, from the declared slot of that instance */
	OMCI_DYN_SLOT_TYPE,
	OMCI_DYN_SLOT_PORTS,
	OMCI_DYN_SLOT_TBUF,
	OMCI_DYN_SLOT_PRIQ,
	OMCI_DYN_SLOT_SCHED,
	OMCI_DYN_IP_HOST_MAC,	/* ME 134 #2: the WAN MAC, where declared */
	OMCI_DYN_TCONT_N,	/* ANI-G #2: the board's T-CONT count */
	OMCI_DYN_ONU2G_PQ,	/* ONU2-G #6 */
	OMCI_DYN_SCHED_N,	/* ONU2-G #7 */
	OMCI_DYN_GEM_N,		/* ONU2-G #9 */
};

/* One modelled attribute. Rows of the same class are ...
 * dev/MEASURED-gpon_omci_me.c.md sec 8. */
struct omci_attr {
	u16	class_id;
	u16	v;
	u8	attr;
	u8	size;
	u8	src;
	u8	access;	/* verified dynamic layout: read=1, write=2, create=4 */
};

/* G.988 calls the attribute writable and this ONU has exactly ...
 * dev/MEASURED-gpon_omci_me.c.md sec 9. */
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
/* @sz octets of zeros, served from the zeros member (@sz <= 25 — see the
 * zeros member: a larger ask reads off the end, and only the x86 sweep
 * plus ASan police that bound) */
#define A_ZERO(cls, n, sz)	AT(cls, n, sz, OMCI_SRC_ID,		\
				   offsetof(struct omci_identity, zeros))
#define A_SN(cls, n)		AT(cls, n, 8, OMCI_SRC_SN, 0)
#define A_MDS(cls, n)		AT(cls, n, 1, OMCI_SRC_MDS, 0)
#define A_D(cls, n, sz, dyn)	AT(cls, n, sz, OMCI_SRC_DYN, dyn)
#define A_NO_ATTRS(cls)		AT(cls, 0, 0, OMCI_SRC_CONST, 0)
/* a read-only COUNTER at dense offset @off that nothing feeds: see
 * OMCI_SRC_CNT.  It keeps the dense layout stock's and answers COULD NOT ASK. */
#define A_CNT(cls, n, sz, off)	{ (cls), (off), (n), (sz), OMCI_SRC_CNT, 1 }

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
	A_ID(256, 11, loid_passwd),		/* #11 Logical password */
	A_C(256, 12,  1, 0x00),			/* #12 Credentials status */
	A_C(256, 13,  2, 0x0000),		/* #13 Ext TC-layer options */
	A_C(256, 14,  1, 0x01),			/* #14 ONT state */

	/* ---- ME 257 ONU2-G (inst 0) ---- */
	A_ID(257,  1, equipment_id),		/* #1  Equipment ID */
	A_C(257,  2,  1, 0x80),			/* #2  OMCC version: G.984.4,
						 * BASELINE only — devid 0x0b is
						 * not served, and the two must
						 * stay consistent */
	A_ID(257,  3, product_code),		/* #3  Vendor product code */
	A_C(257,  4,  1, 0x01),			/* #4  Security capability */
	A_C(257,  5,  1, 0x01),			/* #5  Security mode */
	A_D(257,  6,  2, OMCI_DYN_ONU2G_PQ),	/* #6  Total priority queues */
	A_D(257,  7,  1, OMCI_DYN_SCHED_N),	/* #7  Total traffic scheds */
	A_C(257,  8,  1, 0x01),			/* #8  Mode */
	A_D(257,  9,  2, OMCI_DYN_GEM_N),	/* #9  Total GEM ports */
	A_C(257, 10,  4, 3600),			/* #10 SysUpTime — UINT32: two
						 * bytes here misaligns every
						 * later attr (proven bug) */
	A_C(257, 11,  2, 0x007f),		/* #11 Connectivity capability */
	A_C(257, 12,  1, 0x00),			/* #12 Current conn mode */
	A_C(257, 13,  2, 0x003b),		/* #13 QoS config flexibility */
	A_C(257, 14,  2, 0x0001),		/* #14 Priority-queue scale */

	/* ---- ME 5 Cardholder, one per declared slot ---- */
	A_D(5, 1, 1, OMCI_DYN_SLOT_TYPE),	/* #1  Actual type */
	A_D(5, 2, 1, OMCI_DYN_SLOT_TYPE),	/* #2  Expected type */
	A_D(5, 3, 1, OMCI_DYN_SLOT_PORTS),	/* #3  Expected port count */

	/* ---- ME 6 Circuit-Pack, one per declared slot ---- */
	A_D(6,  1,  1, OMCI_DYN_SLOT_TYPE),	/* #1  Type */
	A_D(6,  2,  1, OMCI_DYN_SLOT_PORTS),	/* #2  Number of ports */
	A_SN(6, 3),				/* #3  Serial number */
	A_ID(6,  4, sw_bank0_version),		/* #4  Version */
	A_ID(6,  5, vendor_id),			/* #5  Vendor ID */
	A_CW(6,  6,  1, 0),			/* #6  Admin state */
	A_C(6,  7,  1, 0),			/* #7  Op state */
	A_CW(6,  8,  1, 0),			/* #8  Bridged/IP ind */
	A_CW(6, 10,  1, 0),			/* #10 Card configuration */
	A_D(6, 11,  1, OMCI_DYN_SLOT_TBUF),	/* #11 Total T-CONT buffers */
	A_D(6, 12,  1, OMCI_DYN_SLOT_PRIQ),	/* #12 Total priority queues */
	A_D(6, 13,  1, OMCI_DYN_SLOT_SCHED),	/* #13 Total traffic schedulers */
	A_CW(6, 14,  4, 0),			/* #14 Power shed override */

	/* ---- ME 7 Software-Image, banks 0 (active) + 1 ---- */
	A_D(7, 1, 14, OMCI_DYN_SW_VER),		/* #1  Version */
	A_D(7, 2,  1, OMCI_DYN_SW_FLAG),	/* #2  Is committed */
	A_D(7, 3,  1, OMCI_DYN_SW_FLAG),	/* #3  Is active */
	A_C(7, 4,  1, 1),			/* #4  Is valid */
	/* ⚠ #5 AND #6 WERE ONE ROW UNTIL 2026-08-31: this table ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 10. */
	A_ZERO(7, 5, 25),			/* #5  Product code */
	A_ZERO(7, 6, 16),			/* #6  Image hash */

	/* ---- ME 11 PPTP Ethernet UNI (inst 0x0101) — THE HGU gate ---- */
	A_D(11,  1, 1, OMCI_DYN_UNI_EXPECTED),	/* #1  Expected type */
	A_D(11,  2, 1, OMCI_DYN_UNI_TYPE),	/* #2  Sensed type */
	/* ★ THE WRITABLE SET IS STOCK'S OWN, MEASURED: both boards' ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 11. */
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

	/* ---- ME 53 PPTP POTS UNI, one per declared instance: what stock
	 * reports for a port no phone is wired to ---- */
	A_CW(53,  1, 1, 0),			/* #1  Admin state */
	A_CW(53,  2, 2, 0),			/* #2  Interworking TP pointer */
	A_CW(53,  3, 1, 0),			/* #3  ARC */
	A_CW(53,  4, 1, 0),			/* #4  ARC interval */
	A_CW(53,  5, 1, 0),			/* #5  Impedance */
	A_CW(53,  6, 1, 0),			/* #6  Transmission path */
	A_CW(53,  7, 1, 0),			/* #7  Rx gain */
	A_CW(53,  8, 1, 0),			/* #8  Tx gain */
	A_C(53,  9, 1, 0),			/* #9  Op state */
	A_C(53, 10, 1, 0),			/* #10 Hook state */
	A_CW(53, 11, 2, 0),			/* #11 POTS holdover time */
	A_CW(53, 12, 1, 0),			/* #12 Nominal feed voltage */

	/* ---- ME 134 IP host config data, one per declared instance.  The
	 * current-address group (#9..#13) reads 0: stock mirrors the live WAN
	 * lease there, which this model does not know. ---- */
	A_CW(134,  1, 1, 0),			/* #1  IP options */
	A_D(134,  2, 6, OMCI_DYN_IP_HOST_MAC),	/* #2  MAC address */
	A_ZERO(134, 3, 25),			/* #3  ONU identifier */
	A_CW(134,  4, 4, 0),			/* #4  IP address */
	A_CW(134,  5, 4, 0),			/* #5  Mask */
	A_CW(134,  6, 4, 0),			/* #6  Gateway */
	A_CW(134,  7, 4, 0),			/* #7  Primary DNS */
	A_CW(134,  8, 4, 0),			/* #8  Secondary DNS */
	A_C(134,  9, 4, 0),			/* #9  Current address */
	A_C(134, 10, 4, 0),			/* #10 Current mask */
	A_C(134, 11, 4, 0),			/* #11 Current gateway */
	A_C(134, 12, 4, 0),			/* #12 Current primary DNS */
	A_C(134, 13, 4, 0),			/* #13 Current secondary DNS */
	A_ZERO(134, 14, 25),			/* #14 Domain name */
	A_ZERO(134, 15, 25),			/* #15 Host name */
	A_CW(134, 16, 2, 0),			/* #16 Relay agent options */

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

	/* ME 47 MAC bridge port configuration data ... -- dev/MEASURED-gpon_omci_me.c.md sec 39. */
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

	/* ME 49 MAC bridge port filter table data ... -- dev/MEASURED-gpon_omci_me.c.md sec 12. */
	A_TBL(49, 1, GPON_MAC_FILTER_ROW_LEN, OMCI_TBL_MAC_FILTER_ROW, 3),
					/* #1 MAC filter table MACFilterTable ---- ME 50 MAC bridge ...
					 * dev/MEASURED-gpon_omci_me.c.md sec 13. */
	A_NO_ATTRS(50),

	/* ME 52 MAC bridge port PM history data ... -- dev/MEASURED-gpon_omci_me.c.md sec 14. */
	A_ST(52, 1, 1,  0, 1),		/* #1 interval end time    IntervalEndTime (R) */
	A_ST(52, 2, 2,  1, 7),		/* #2 threshold data 1/2   ThresholdData12Id */
	/* #3..#7 ARE COUNTERS AND NOTHING COUNTS. ... -- dev/MEASURED-gpon_omci_me.c.md sec 44. */
	A_CNT(52, 3, 4,  3),		/* #3 forwarded frames     ForwardedFrameCounter */
	A_CNT(52, 4, 4,  7),		/* #4 delay exceeded disc  DelayExceededDiscard */
	A_CNT(52, 5, 4, 11),		/* #5 MTU exceeded disc    MtuExceededDiscard */
	A_CNT(52, 6, 4, 15),		/* #6 received frames      ReceivedFrameCounter */
	A_CNT(52, 7, 4, 19),		/* #7 received+discarded   ReceivedAndDiscardedCounter */

	/* ME 78 VLAN tagging operation configuration data ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 15. */
	A_ST(78, 1, 1, 0, 7),		/* #1  upstream op mode    UsTagOpMode */
	A_ST(78, 2, 2, 1, 7),		/* #2  upstream TCI        UsTagTci */
	A_ST(78, 3, 1, 3, 7),		/* #3  downstream op mode  DsTagOpMode */
	A_ST(78, 4, 1, 4, 7),		/* #4  association type    Type */
	A_ST(78, 5, 2, 5, 7),		/* #5  associated ME ptr   Pointer */

	/* ME 79 MAC bridge port filter pre-assign table ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 16. */
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

	/* ME 84 VLAN tagging filter data (mib_VlanTagFilterData) -- ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 17. */
	A_ST(84, 1, 24,  0, 7),		/* #1  VLAN filter list    FilterTbl */
	A_ST(84, 2,  1, 24, 7),		/* #2  forward operation   FwdOp */
	A_ST(84, 3,  1, 25, 7),		/* #3  number of entries   NumOfEntries */

	/* ME 130 802.1p mapper service profile (mib_Map8021pServProf)
	 * dev/MEASURED-gpon_omci_me.c.md sec 18. */
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

	/* ME 171 extended VLAN tagging operation configuration data
	 * dev/MEASURED-gpon_omci_me.c.md sec 19. */
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
	A_D(263,  2, 2, OMCI_DYN_TCONT_N),			/* #2  Total T-CONTs */
	A_CW(263,  3, 2, 48),			/* #3  GEM block length */
	A_CW(263,  4, 1, 0),			/* #4  Piggyback DBA */
	A_C(263,  5, 1, 0),			/* #5  (deprecated) */
	A_CW(263,  6, 1, 5),			/* #6  SF threshold */
	A_CW(263,  7, 1, 9),			/* #7  SD threshold */
	A_CW(263,  8, 1, 0),			/* #8  ARC */
	A_CW(263,  9, 1, 0),			/* #9  ARC interval */
	/* #10/#14 are the LIVE optical levels, sampled from the optic's
	 * SFF-8472 A2h diagnostics by the shell (see omci_onu_set_optical);
	 * until the first successful read — and after a failed one — they serve
	 * OMCI_ANIG_{RX,TX}_FALLBACK, because the OLT must never get silence. */
	A_D(263, 10, 2, OMCI_DYN_ANIG_RX),	/* #10 RX optical level */
	{ 263, 0, 11, 1, OMCI_SRC_ANIG_THRESHOLD, 3 },
	{ 263, 1, 12, 1, OMCI_SRC_ANIG_THRESHOLD, 3 },
	A_C(263, 13, 2, 0x0000),		/* #13 ONU response time */
	A_D(263, 14, 2, OMCI_DYN_ANIG_TX),	/* #14 TX optical level */
	{ 263, 2, 15, 1, OMCI_SRC_ANIG_THRESHOLD, 3 },
	{ 263, 3, 16, 1, OMCI_SRC_ANIG_THRESHOLD, 3 },

	/* ---- ME 264 UNI-G (inst 0x0101) ---- */
	A_C(264, 1, 2, 0x0000),			/* #1  Config-option status */
	A_D(264, 2, 1, OMCI_DYN_UNIG_ADMIN),	/* #2  Admin state */
	A_D(264, 3, 1, OMCI_DYN_UNIG_CAP),	/* #3  Management capability */
	A_C(264, 4, 2, 0x0000),			/* #4  Non-OMCI mgmt ID */
	A_C(264, 5, 2, 0x0000),			/* #5  Relay-agent options */

	/* ME 266 GEM interworking termination point (mib_GemIwTp)
	 * dev/MEASURED-gpon_omci_me.c.md sec 40. */
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

	/* ME 280 GEM traffic descriptor (mib_GemTrafficDescriptor)
	 * dev/MEASURED-gpon_omci_me.c.md sec 20. */
	A_ST(280, 1, 4,  0, 7),		/* #1  CIR                 CIR */
	A_ST(280, 2, 4,  4, 7),		/* #2  PIR                 PIR */
	A_ST(280, 3, 4,  8, 7),		/* #3  CBS                 CBS */
	A_ST(280, 4, 4, 12, 7),		/* #4  PBS                 PBS */
	A_ST(280, 5, 1, 16, 7),		/* #5  colour mode         ColourMode */
	A_ST(280, 6, 1, 17, 7),		/* #6  ingress colour mark IngressColourMarking */
	A_ST(280, 7, 1, 18, 7),		/* #7  egress colour mark  EgressColourMarking */
	A_ST(280, 8, 1, 19, 7),		/* #8  meter type          MeterType */

	/* ME 281 multicast GEM interworking termination point ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 21. */
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
	/* ⚠ THE SAME OFF-BY-ONE, and on the ME this port's WAN path ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 41. */
	A_ZERO(329, 3, 25),			/* #3  Interdomain name */
	A_C(329, 4,  2, 0x0000),		/* #4  TCP/UDP pointer */

	/* ---- ME 65530 CTC LoID authentication (inst 0) ---- */
	A_ID(65530, 1, operator_id),		/* #1  Operation ID */
	A_ID(65530, 2, loid),			/* #2  LoID */
	A_ID(65530, 3, loid_passwd),		/* #3  Password (MUST be
						 * servable, proven) */
	A_C(65530, 4,  1, 0x01),		/* #4  Auth status = success */

	{ 0, 0, 0, 0, 0, 0 },			/* terminator (class 0 is not a
						 * G.988 class ID) */
};

/* Is @class_id in a G.988 vendor-reserved range (240..255, ...
 * dev/MEASURED-gpon_omci_me.c.md sec 22. */
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

/* Does a Get validate the instance?  Every mutable class, and the classes
 * whose instances a board declares: equipment, queues, IP hosts, POTS. */
bool omci_get_checks_inst(u16 class_id)
{
	return omci_me_mutable(class_id) || class_id == OMCI_ME_CARDHOLDER ||
	       class_id == OMCI_ME_CIRCUIT_PACK ||
	       class_id == OMCI_ME_PRIORITY_QUEUE ||
	       class_id == OMCI_ME_IP_HOST ||
	       class_id == OMCI_ME_PPTP_POTS_UNI;
}

/* Only these classes have a verified mutable layout in this model. Other
 * classes retain their existing compatibility handling in the message layer. */
bool omci_me_mutable(u16 class_id)
{
	switch (class_id) {
	case OMCI_ME_ANI_G:
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
	/* The VLAN / classification half. Every attribute stock ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 42. */
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

static void omci_anig_alarms_refresh(struct omci_onu *o)
{
	const u8 *t = o->anig_threshold;
	s32 rx = (s16)o->anig_rx_level, tx = (s16)o->anig_tx_level;
	u16 bits = 0;

	if (!o->anig_live)
		return;
	/* Threshold LSB = 0.5 dB; level LSB = 0.002 dB. No guessed vendor policy. */
	if (t[0] != 0xff && rx < -(s32)t[0] * 250)
		bits |= OMCI_ANIG_RX_LOW;
	if (t[1] != 0xff && rx > -(s32)t[1] * 250)
		bits |= OMCI_ANIG_RX_HIGH;
	if (t[2] != 0x81 && tx < (s32)(s8)t[2] * 250)
		bits |= OMCI_ANIG_TX_LOW;
	if (t[3] != 0x81 && tx > (s32)(s8)t[3] * 250)
		bits |= OMCI_ANIG_TX_HIGH;
	o->alarm_class = OMCI_ME_ANI_G;
	o->alarm_inst = 0x8001;
	o->alarm_active = (o->alarm_active & ~(OMCI_ANIG_RX_LOW |
		OMCI_ANIG_RX_HIGH | OMCI_ANIG_TX_LOW | OMCI_ANIG_TX_HIGH)) | bits;
}

#define OMCI_ID_FIELD(member) \
	{ offsetof(struct omci_identity, member), OMCI_ID_SIZEOF(member) }
static const struct { u8 off, size; } omci_id_fields[OMCI_ID_FIELDS] = {
	[OMCI_ID_VENDOR]	= OMCI_ID_FIELD(vendor_id),
	[OMCI_ID_HW_VERSION]	= OMCI_ID_FIELD(onu_g_version),
	[OMCI_ID_SW_VERSION0]	= OMCI_ID_FIELD(sw_bank0_version),
	[OMCI_ID_SW_VERSION1]	= OMCI_ID_FIELD(sw_bank1_version),
	[OMCI_ID_EQUIPMENT]	= OMCI_ID_FIELD(equipment_id),
	[OMCI_ID_PRODUCT_CODE]	= OMCI_ID_FIELD(product_code),
	[OMCI_ID_LOID]		= OMCI_ID_FIELD(loid),
	[OMCI_ID_LOID_PASSWD]	= OMCI_ID_FIELD(loid_passwd),
};

const char *const omci_id_names[OMCI_ID_FIELDS] = {
	[OMCI_ID_VENDOR]	= "vendor_id",
	[OMCI_ID_HW_VERSION]	= "hw_ver",
	[OMCI_ID_SW_VERSION0]	= "sw_ver1",
	[OMCI_ID_SW_VERSION1]	= "sw_ver2",
	[OMCI_ID_EQUIPMENT]	= "model",
	[OMCI_ID_PRODUCT_CODE]	= "product_code",
	[OMCI_ID_LOID]		= "loid",
	[OMCI_ID_LOID_PASSWD]	= "loid_passwd",
};

void omci_id_default(struct omci_identity *id)
{
	*id = omci_id_default_bytes;
}

bool omci_id_set(struct omci_identity *id, enum omci_id_field f,
		 const u8 *val, unsigned int len)
{
	u8 *dst;

	if ((unsigned int)f >= OMCI_ID_FIELDS || len > omci_id_fields[f].size ||
	    (f == OMCI_ID_PRODUCT_CODE && len != 2))
		return false;
	if (!id)
		return true;
	dst = (u8 *)id + omci_id_fields[f].off;
	memset(dst, 0, omci_id_fields[f].size);
	if (len)
		memcpy(dst, val, len);
	return true;
}

bool omci_id_set_str(struct omci_identity *id, enum omci_id_field f,
		     const char *s)
{
	unsigned int len = 0, code = 0;
	u8 be[2];

	while (s[len] && s[len] != '\n')
		len++;
	if (f != OMCI_ID_PRODUCT_CODE)
		return omci_id_set(id, f, (const u8 *)s, len);
	if (!len || len > 4)
		return false;
	for (; *s && *s != '\n'; s++) {
		char c = *s;

		if (c >= '0' && c <= '9')
			code = code << 4 | (unsigned int)(c - '0');
		else if ((c | 0x20) >= 'a' && (c | 0x20) <= 'f')
			code = code << 4 | (unsigned int)((c | 0x20) - 'a' + 10);
		else
			return false;
	}
	be[0] = (u8)(code >> 8);
	be[1] = (u8)code;
	return omci_id_set(id, f, be, 2);
}

void omci_onu_set_optical(struct omci_onu *o, u16 rx_level, u16 tx_level)
{
	o->anig_rx_level = rx_level;
	o->anig_tx_level = tx_level;
	o->anig_live = true;
	omci_anig_alarms_refresh(o);
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
	for (i = 0; i < OMCI_TCONT_MAX; i++)
		o->tcont_alloc[i] = i ? 0x00ff : 0x0100;
	o->anig_threshold[0] = o->anig_threshold[1] = 0xff;
	o->anig_threshold[2] = o->anig_threshold[3] = 0x81;
	omci_anig_alarms_refresh(o);
}

/* How many octets of DENSE attribute body @class_id's ...
 * dev/MEASURED-gpon_omci_me.c.md sec 23. */
u8 omci_me_dense_len(u16 class_id)
{
	const struct omci_attr *a;
	unsigned int end = 0;

	for (a = omci_me_find(class_id); a && a->class_id == class_id; a++)
		if ((a->src == OMCI_SRC_STORE || a->src == OMCI_SRC_CNT) &&
		    (unsigned int)a->v + a->size > end)
			end = (unsigned int)a->v + a->size;
	return end > OMCI_STORE_BODY ? 0 : (u8)end;
}

/* Create carries only SBC attributes, with no mask. A class ...
 * dev/MEASURED-gpon_omci_me.c.md sec 24. */
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

/* Can this ONU actually realise the value the OLT asks for in ...
 * dev/MEASURED-gpon_omci_me.c.md sec 25. */
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
	else if (a->src == OMCI_SRC_ANIG_THRESHOLD)
		o->anig_threshold[a->v] = v[0];
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
		o->tcont_alloc_written |= 1u << (inst - 0x8000);
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
		    /* ME 11 #5 and ME 264 #2, the administrative states. Both are ...
		     * dev/MEASURED-gpon_omci_me.c.md sec 26. */
		    (pptp_slot >= 0 && a->attr == 5) ||
		    (unig_slot >= 0 && a->attr == 2) ||
		    (class_id == OMCI_ME_TCONT && inst >= 0x8000 &&
		     inst < 0x8000 + o->cap.tcont_n && a->attr == 1))
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
	if (class_id == OMCI_ME_ANI_G)
		omci_anig_alarms_refresh(o);
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
static const struct omci_slot *omci_slot_find(const struct omci_onu *o,
					      u16 inst)
{
	u8 i;

	for (i = 0; i < o->slots_n; i++)
		if (o->slots[i].inst == inst)
			return &o->slots[i];
	return NULL;
}

static int omci_ip_host_index(const struct omci_onu *o, u16 inst)
{
	int i;

	for (i = 0; i < o->ip_host_n; i++)
		if (o->ip_host[i] == inst)
			return i;
	return -1;
}

static u8 omci_slot_value(const struct omci_slot *s, u16 dyn)
{
	switch (dyn) {
	case OMCI_DYN_SLOT_TYPE:
		return s->type;
	case OMCI_DYN_SLOT_PORTS:
		return s->ports;
	case OMCI_DYN_SLOT_TBUF:
		return s->tcont_buf;
	case OMCI_DYN_SLOT_PRIQ:
		return s->priq;
	default:		/* OMCI_DYN_SLOT_SCHED */
		return s->scheds;
	}
}

/* ME 277 #6 as stock numbers its queues: blocks of 8 counting DOWN (queue 0
 * -> priority 7).  Downstream block b is the board's b-th declared queue port;
 * upstream queue 0x8000 + 8t + p belongs to T-CONT 0x8000 + t. */
static u32 omci_pq_related_port(const struct omci_onu *o, u16 inst)
{
	u16 block = (u16)((inst & 0x7fff) / 8), port;

	if (inst & 0x8000)
		port = (u16)(0x8000 + block);
	else
		port = block < o->cap.dsq_n ? o->cap.dsq_port[block] : 0;
	return ((u32)port << 16) | (7u - (inst & 7));
}

static const u8 *omci_attr_bytes(struct omci_onu *o,
				 const struct omci_attr *a, u16 inst,
				 u8 *scratch)
{
	u32 val;

	switch (a->src) {
	case OMCI_SRC_ANIG_THRESHOLD:
		return o->anig_threshold + a->v;
	case OMCI_SRC_STORE: {
		struct omci_me_inst *e = omci_store_find(o,
						      a->class_id, inst);

		return e && a->v + a->size <= e->blen ? e->body + a->v : NULL;
	}
	case OMCI_SRC_TBL:
		/* A table attribute read as a scalar. G.988 reads one with ...
		 * dev/MEASURED-gpon_omci_me.c.md sec 27. */
		return a->size <= OMCI_ID_SIZEOF(zeros) ? o->id.zeros : NULL;
	case OMCI_SRC_CNT:
		return NULL;		/* could not ask -- never a zero */
	case OMCI_SRC_ID:
		return (const u8 *)&o->id + a->v;
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
			return inst ? o->id.sw_bank1_version :
				      o->id.sw_bank0_version;
		case OMCI_DYN_SW_FLAG:
			val = inst ? 0 : 1;
			break;
		case OMCI_DYN_TCONT_ALLOC:
			val = inst >= 0x8000 && inst < 0x8000 + o->cap.tcont_n ?
				o->tcont_alloc[inst - 0x8000] : 0x00ff;
			break;
		case OMCI_DYN_PQ_PORT:
			val = omci_pq_related_port(o, inst);
			break;
		case OMCI_DYN_TCONT_N:
			val = o->cap.tcont_n;
			break;
		case OMCI_DYN_ONU2G_PQ:
			val = o->cap.onu2g_pq;
			break;
		case OMCI_DYN_SCHED_N:
			val = o->cap.sched_n;
			break;
		case OMCI_DYN_GEM_N:
			val = o->cap.gem_ports;
			break;
		case OMCI_DYN_IP_HOST_MAC: {
			int i = omci_ip_host_index(o, inst);

			if (i < 0)
				return NULL;
			if (!o->ip_host_mac[i])
				return o->id.zeros;
			return o->wan_mac_set ? o->wan_mac : NULL;
		}
		case OMCI_DYN_SLOT_TYPE:
		case OMCI_DYN_SLOT_PORTS:
		case OMCI_DYN_SLOT_TBUF:
		case OMCI_DYN_SLOT_PRIQ:
		case OMCI_DYN_SLOT_SCHED: {
			const struct omci_slot *slot = omci_slot_find(o, inst);

			if (!slot)
				return NULL;
			val = omci_slot_value(slot, a->v);
			break;
		}
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

/* the ONE generic attribute filler ---- Shared by GET and ...
 * dev/MEASURED-gpon_omci_me.c.md sec 28. */
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

/* Build the static MIB-Upload row table: every ... -- dev/MEASURED-gpon_omci_me.c.md sec 29. */
static void omci_build_mib(struct omci_onu *o)
{
	u16 n = 0, dropped = 0;
	u16 i, ds_q, us_q;

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

	/* Per slot: ME 5 (#1..#3), then ME 6 A = #1..#4 = 24B and
	 * B = #5(4) + #6..#8, #10..#13 (7x1) + #14(4) = 15B. */
	for (i = 0; i < o->slots_n; i++) {
		u16 slot = o->slots[i].inst;

		ROW(OMCI_ME_CARDHOLDER, slot, OMCI_ATTR_BIT(1) |
					      OMCI_ATTR_BIT(2) |
					      OMCI_ATTR_BIT(3));
		ROW(OMCI_ME_CIRCUIT_PACK, slot, OMCI_ATTR_BIT(1) |
						OMCI_ATTR_BIT(2) |
						OMCI_ATTR_BIT(3) |
						OMCI_ATTR_BIT(4));
		ROW(OMCI_ME_CIRCUIT_PACK, slot, OMCI_ATTR_BIT(5) |
						OMCI_ATTR_BIT(6) |
						OMCI_ATTR_BIT(7) |
						OMCI_ATTR_BIT(8) |
						OMCI_ATTR_BIT(10) |
						OMCI_ATTR_BIT(11) |
						OMCI_ATTR_BIT(12) |
						OMCI_ATTR_BIT(13) |
						OMCI_ATTR_BIT(14));
	}

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

	/* ME 53 PPTP POTS UNI: #1..#12 = 14B. */
	for (i = 0; i < o->pots_uni_n; i++)
		ROW(OMCI_ME_PPTP_POTS_UNI, o->pots_uni[i],
		    OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) | OMCI_ATTR_BIT(3) |
		    OMCI_ATTR_BIT(4) | OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(6) |
		    OMCI_ATTR_BIT(7) | OMCI_ATTR_BIT(8) | OMCI_ATTR_BIT(9) |
		    OMCI_ATTR_BIT(10) | OMCI_ATTR_BIT(11) | OMCI_ATTR_BIT(12));

	ROW(OMCI_ME_OLT_G, 0x0000, 0x0000);

	/* ME 134 IP host config data: A = #1 + #2 + #4..#7 = 23B, #3 = 25B,
	 * C = #8..#13 = 24B, #14 = 25B, #15 = 25B, #16 = 2B. */
	for (i = 0; i < o->ip_host_n; i++) {
		u16 ih = o->ip_host[i];

		ROW(OMCI_ME_IP_HOST, ih, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					 OMCI_ATTR_BIT(4) | OMCI_ATTR_BIT(5) |
					 OMCI_ATTR_BIT(6) | OMCI_ATTR_BIT(7));
		ROW(OMCI_ME_IP_HOST, ih, OMCI_ATTR_BIT(3));
		ROW(OMCI_ME_IP_HOST, ih, OMCI_ATTR_BIT(8) | OMCI_ATTR_BIT(9) |
					 OMCI_ATTR_BIT(10) | OMCI_ATTR_BIT(11) |
					 OMCI_ATTR_BIT(12) | OMCI_ATTR_BIT(13));
		ROW(OMCI_ME_IP_HOST, ih, OMCI_ATTR_BIT(14));
		ROW(OMCI_ME_IP_HOST, ih, OMCI_ATTR_BIT(15));
		ROW(OMCI_ME_IP_HOST, ih, OMCI_ATTR_BIT(16));
	}

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

	/* ME 262 T-CONT (inst 0x8000 + i): 4B each. */
	for (i = 0; i < o->cap.tcont_n; i++)
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

	/* ME 277 Priority-Queue: 8 per declared downstream port from 0, and 8
	 * per T-CONT from 0x8000 when the board declares its capacity (the old
	 * single-UNI cap of 8: ... dev/MEASURED-gpon_omci_me.c.md sec 30). */
	ds_q = (u16)(8 * o->cap.dsq_n);
	us_q = o->cap.usq ? (u16)(8 * o->cap.tcont_n) : 0;
	for (i = 0; i < ds_q + us_q; i++)
		ROW(OMCI_ME_PRIORITY_QUEUE, i < ds_q ? i : 0x8000 + i - ds_q,
					       OMCI_ATTR_BIT(1) |
					       OMCI_ATTR_BIT(2) |
					       OMCI_ATTR_BIT(3) |
					       OMCI_ATTR_BIT(4) |
					       OMCI_ATTR_BIT(5) |
					       OMCI_ATTR_BIT(6) |
					       OMCI_ATTR_BIT(7) |
					       OMCI_ATTR_BIT(8));

	/* ME 278 Traffic-Scheduler (inst 0x8000 + i): 6B each. */
	for (i = 0; i < o->cap.sched_n; i++)
		ROW(OMCI_ME_TRAFFIC_SCHED, 0x8000 + i, OMCI_ATTR_BIT(1) |
						       OMCI_ATTR_BIT(2) |
						       OMCI_ATTR_BIT(3) |
						       OMCI_ATTR_BIT(4));

	/* ★★ THE OTHER HALF OF THE #3/#4 CORRECTION ABOVE, AND IT WAS ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 31. */
	ROW(OMCI_ME_VEIP, OMCI_VEIP_INST, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					  OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_VEIP, OMCI_VEIP_INST, OMCI_ATTR_BIT(3));

	/* ME 65530 (CTC LoID authentication) is deliberately NOT ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 32. */

#undef ROW
	o->nrows = n;
	o->rows_dropped = dropped;
}

bool omci_onu_declare_unis(struct omci_onu *o,
			   const u16 *pptp_inst, const u8 *pptp_type, u8 pptp_n,
			   const u16 *unig_inst, const u8 *unig_mgmt_cap,
			   u8 unig_n)
{
	/* The panel as it stands, so a refusal can put it back ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 33. */
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
		/* ⚠ A PANEL THAT DOES NOT FIT IS PUT BACK, NOT LEFT INSTALLED
		 * dev/MEASURED-gpon_omci_me.c.md sec 43. */
		o->pptp_eth_uni = was_pptp;
		o->uni_g = was_unig;
		memcpy(o->uni_g_mgmt_cap, was_cap, sizeof(was_cap));
		memcpy(o->uni_type, was_type, sizeof(was_type));
		omci_build_mib(o);
		return false;
	}
	return true;
}

bool omci_onu_declare_slots(struct omci_onu *o, const struct omci_slot *slot,
			    u8 n)
{
	struct omci_slot was[OMCI_SLOT_MAX];
	u8 was_n = o->slots_n, i, j;

	if (n > OMCI_SLOT_MAX || (n && !slot))
		return false;
	for (i = 0; i < n; i++) {
		if (!slot[i].inst)
			return false;
		for (j = 0; j < i; j++)
			if (slot[j].inst == slot[i].inst)
				return false;
	}
	memcpy(was, o->slots, sizeof(was));
	if (n)
		memcpy(o->slots, slot, n * sizeof(*slot));
	o->slots_n = n;
	omci_build_mib(o);
	if (o->rows_dropped) {
		memcpy(o->slots, was, sizeof(was));
		o->slots_n = was_n;
		omci_build_mib(o);
		return false;
	}
	return true;
}

/* The inventory every board on the bench has at minimum, and exactly what this
 * model carried before the inventory existed. */
static const u16 omci_uni_default[] = {
	0x0101 /* default PPTP Ethernet UNI instance */
};

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

/* One plain Ethernet pack with its 8 downstream queues: what this model
 * carried before a board could declare its slots. */
static const struct omci_slot omci_slot_default = {
	.inst = 0x0101, .type = OMCI_UNI_TYPE_DEFAULT, .ports = 1, .priq = 8,
};

#define OMCI_SLOT_BE_FIELDS 6

enum omci_uni_decl omci_onu_declare_slots_be(struct omci_onu *o,
					     const void *be, int len,
					     const char **why)
{
	struct omci_slot slot[OMCI_SLOT_MAX];
	u16 v[OMCI_SLOT_BE_FIELDS];
	const u8 *b = be;
	const char *unused;
	int i, f, n;

	if (!why)
		why = &unused;
	if (len < 0) {
		*why = "no slots declared";
		return OMCI_UNI_DECL_ABSENT;
	}
	if ((len && !b) || len % (2 * OMCI_SLOT_BE_FIELDS) ||
	    len > 2 * OMCI_SLOT_BE_FIELDS * OMCI_SLOT_MAX) {
		*why = "the slot list is not 0..8 sextuples of 16-bit cells";
		return OMCI_UNI_DECL_BAD;
	}
	n = len / (2 * OMCI_SLOT_BE_FIELDS);
	for (i = 0; i < n; i++) {
		for (f = 0; f < OMCI_SLOT_BE_FIELDS; f++, b += 2)
			v[f] = (u16)((b[0] << 8) | b[1]);
		for (f = 1; f < OMCI_SLOT_BE_FIELDS; f++)
			if (v[f] > 0xff) {
				*why = "a slot value wider than its one-octet attribute";
				return OMCI_UNI_DECL_BAD;
			}
		slot[i] = (struct omci_slot){ .inst = v[0], .type = (u8)v[1],
			.ports = (u8)v[2], .priq = (u8)v[3],
			.tcont_buf = (u8)v[4], .scheds = (u8)v[5] };
	}
	if (!omci_onu_declare_slots(o, slot, (u8)n)) {
		*why = "a zero or duplicate instance, or more MIB rows than the table holds";
		return OMCI_UNI_DECL_BAD;
	}
	*why = "declared";
	return OMCI_UNI_DECL_OK;
}

enum omci_uni_decl omci_onu_declare_ip_hosts_be(struct omci_onu *o,
						const void *inst_be, int inst_len,
						const void *mac, int mac_len,
						const char **why)
{
	u16 inst[OMCI_IP_HOST_MAX], was[OMCI_IP_HOST_MAX];
	u8 was_mac[OMCI_IP_HOST_MAX], was_n = o->ip_host_n;
	const u8 *b = inst_be;
	const char *unused;
	int i, j, n;

	if (!why)
		why = &unused;
	if (inst_len < 0 && mac_len < 0) {
		*why = "no IP hosts declared";
		return OMCI_UNI_DECL_ABSENT;
	}
	n = inst_len / 2;
	if (!b || inst_len <= 0 || (inst_len & 1) || n > OMCI_IP_HOST_MAX ||
	    !mac || mac_len != n) {
		*why = "the IP-host list is not 1..4 16-bit instances with one MAC flag each";
		return OMCI_UNI_DECL_BAD;
	}
	for (i = 0; i < n; i++) {
		inst[i] = (u16)((b[2 * i] << 8) | b[2 * i + 1]);
		for (j = 0; j < i; j++)
			if (inst[j] == inst[i]) {
				*why = "a duplicate IP-host instance";
				return OMCI_UNI_DECL_BAD;
			}
	}
	memcpy(was, o->ip_host, sizeof(was));
	memcpy(was_mac, o->ip_host_mac, sizeof(was_mac));
	memcpy(o->ip_host, inst, n * sizeof(*inst));
	memcpy(o->ip_host_mac, mac, n);
	o->ip_host_n = (u8)n;
	omci_build_mib(o);
	if (o->rows_dropped) {
		memcpy(o->ip_host, was, sizeof(was));
		memcpy(o->ip_host_mac, was_mac, sizeof(was_mac));
		o->ip_host_n = was_n;
		omci_build_mib(o);
		*why = "more MIB rows than the table holds";
		return OMCI_UNI_DECL_BAD;
	}
	*why = "declared";
	return OMCI_UNI_DECL_OK;
}

#define OMCI_CAP_BE_FIELDS 4

enum omci_uni_decl omci_onu_declare_queues_be(struct omci_onu *o,
					      const void *cap, int cap_len,
					      const void *dsq, int dsq_len,
					      const char **why)
{
	struct omci_capacity was = o->cap;
	const u8 *b = cap;
	u16 v[OMCI_CAP_BE_FIELDS], port[OMCI_DSQ_MAX];
	u8 n = 0;
	const char *unused;
	int f;

	if (!why)
		why = &unused;
	if (cap_len < 0 && dsq_len < 0) {
		*why = "no capacity or queue order declared";
		return OMCI_UNI_DECL_ABSENT;
	}
	if (cap_len >= 0) {
		if (!b || cap_len != 2 * OMCI_CAP_BE_FIELDS) {
			*why = "the capacity is not <T-CONTs priority-queues schedulers GEM-ports>";
			return OMCI_UNI_DECL_BAD;
		}
		for (f = 0; f < OMCI_CAP_BE_FIELDS; f++)
			v[f] = (u16)((b[2 * f] << 8) | b[2 * f + 1]);
		if (!v[0] || v[0] > OMCI_TCONT_MAX || !v[2] || v[2] > OMCI_TCONT_MAX) {
			*why = "a T-CONT or scheduler count outside 1..32";
			return OMCI_UNI_DECL_BAD;
		}
		o->cap.tcont_n = (u8)v[0];
		o->cap.onu2g_pq = v[1];
		o->cap.sched_n = (u8)v[2];
		o->cap.gem_ports = v[3];
		o->cap.usq = true;
	}
	if (dsq_len >= 0) {
		if (!dsq_len || !omci_uni_decode_list(dsq, dsq_len, port, &n) ||
		    !omci_uni_list_ok(port, n)) {
			o->cap = was;
			*why = "the downstream queue ports are not 1..8 distinct non-zero instances";
			return OMCI_UNI_DECL_BAD;
		}
		memcpy(o->cap.dsq_port, port, n * sizeof(*port));
		o->cap.dsq_n = n;
	}
	omci_build_mib(o);
	if (o->rows_dropped) {
		o->cap = was;
		omci_build_mib(o);
		*why = "more MIB rows than the table holds";
		return OMCI_UNI_DECL_BAD;
	}
	*why = "declared";
	return OMCI_UNI_DECL_OK;
}

enum omci_uni_decl omci_onu_declare_pots_be(struct omci_onu *o, const void *be,
					    int len, const char **why)
{
	u16 inst[OMCI_UNI_MAX], was[OMCI_UNI_MAX];
	u8 n = 0, was_n = o->pots_uni_n;
	const char *unused;

	if (!why)
		why = &unused;
	if (len < 0) {
		*why = "no POTS UNIs declared";
		return OMCI_UNI_DECL_ABSENT;
	}
	if (!len || !omci_uni_decode_list(be, len, inst, &n) ||
	    !omci_uni_list_ok(inst, n)) {
		*why = "the POTS-UNI list is not 1..8 distinct non-zero 16-bit instances";
		return OMCI_UNI_DECL_BAD;
	}
	memcpy(was, o->pots_uni, sizeof(was));
	memcpy(o->pots_uni, inst, n * sizeof(*inst));
	o->pots_uni_n = n;
	omci_build_mib(o);
	if (o->rows_dropped) {
		memcpy(o->pots_uni, was, sizeof(was));
		o->pots_uni_n = was_n;
		omci_build_mib(o);
		*why = "more MIB rows than the table holds";
		return OMCI_UNI_DECL_BAD;
	}
	*why = "declared";
	return OMCI_UNI_DECL_OK;
}

const char *omci_onu_declare_equipment(struct omci_onu *o, omci_prop_fn prop,
				       void *ctx, const char **why)
{
	const char *first = NULL, *reason = NULL;
	const void *a, *b;
	int alen, blen;

#define OMCI_DECLARED(call, name) do {					\
		if ((call) == OMCI_UNI_DECL_BAD && !first) {			\
			first = (name);						\
			*why = reason;						\
		}								\
	} while (0)
	a = prop(ctx, "circuit-packs", &alen);
	OMCI_DECLARED(omci_onu_declare_slots_be(o, a, alen, &reason), "circuit-packs");
	a = prop(ctx, "ip-host-instances", &alen);
	b = prop(ctx, "ip-host-wan-mac", &blen);
	OMCI_DECLARED(omci_onu_declare_ip_hosts_be(o, a, alen, b, blen, &reason),
		      "ip-host-instances");
	a = prop(ctx, "pots-uni-instances", &alen);
	OMCI_DECLARED(omci_onu_declare_pots_be(o, a, alen, &reason),
		      "pots-uni-instances");
	a = prop(ctx, "onu-capacity", &alen);
	b = prop(ctx, "downstream-queue-ports", &blen);
	OMCI_DECLARED(omci_onu_declare_queues_be(o, a, alen, b, blen, &reason),
		      "onu-capacity");
#undef OMCI_DECLARED
	return first;
}

void omci_onu_set_wan_mac(struct omci_onu *o, const u8 mac[6])
{
	memcpy(o->wan_mac, mac, sizeof(o->wan_mac));
	o->wan_mac_set = true;
}

void omci_onu_init(struct omci_onu *o, const u8 sn[8], u8 mds_seed)
{
	/* ⚠ THIS READS NOTHING OUT OF @o. It is a COLD init: callers ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 34. */
	memset(o, 0, sizeof(*o));
	memcpy(o->sn, sn, 8);
	omci_id_default(&o->id);
	omci_me_reset_values(o);
	o->mds = mds_seed;
	/* Seed the ANI-G optical levels with the static fallback: a ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 35. */
	o->mds_adapt = true;
	o->mds_adapt_reads = OMCI_MDS_ADAPT_READS;
	o->anig_rx_level = OMCI_ANIG_RX_FALLBACK;
	o->anig_tx_level = OMCI_ANIG_TX_FALLBACK;
	o->slots[0] = omci_slot_default;
	o->slots_n = 1;
	o->cap.tcont_n = OMCI_TCONT_COUNT;
	o->cap.sched_n = OMCI_TCONT_COUNT;
	o->cap.onu2g_pq = 0x0060;
	o->cap.gem_ports = 0x0040;
	o->cap.dsq_port[0] = 0x0101;
	o->cap.dsq_n = 1;
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
	struct omci_identity id = o->id;	/* the unit's, not the session's */
	struct omci_slot slots[OMCI_SLOT_MAX];
	u8 slots_n = o->slots_n;
	u16 ip_host[OMCI_IP_HOST_MAX];
	u8 ip_host_mac[OMCI_IP_HOST_MAX], ip_host_n = o->ip_host_n;
	u16 pots_uni[OMCI_UNI_MAX];
	u8 pots_uni_n = o->pots_uni_n;
	struct omci_capacity capacity = o->cap;
	u8 wan_mac[6];
	bool wan_mac_set = o->wan_mac_set;
	u8 cap[OMCI_UNI_MAX], type[OMCI_UNI_MAX];

	memcpy(cap, o->uni_g_mgmt_cap, sizeof(cap));
	/* ★ THE TYPE IS CARRIED TOO.  It is the panel's, not the session's: an
	 *   identity change does not turn an FE port into a GE one. */
	memcpy(type, o->uni_type, sizeof(type));
	memcpy(slots, o->slots, sizeof(slots));
	memcpy(ip_host, o->ip_host, sizeof(ip_host));
	memcpy(ip_host_mac, o->ip_host_mac, sizeof(ip_host_mac));
	memcpy(pots_uni, o->pots_uni, sizeof(pots_uni));
	memcpy(wan_mac, o->wan_mac, sizeof(wan_mac));
	omci_onu_init(o, sn, mds_seed);
	o->id = id;
	memcpy(o->slots, slots, sizeof(slots));
	o->slots_n = slots_n;
	memcpy(o->ip_host, ip_host, sizeof(ip_host));
	memcpy(o->ip_host_mac, ip_host_mac, sizeof(ip_host_mac));
	o->ip_host_n = ip_host_n;
	memcpy(o->pots_uni, pots_uni, sizeof(pots_uni));
	o->pots_uni_n = pots_uni_n;
	o->cap = capacity;
	memcpy(o->wan_mac, wan_mac, sizeof(wan_mac));
	o->wan_mac_set = wan_mac_set;
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

/* THE WAN DATA GEM -- which ME 268 the OLT meant for user ...
 * dev/MEASURED-gpon_omci_me.c.md sec 36. */

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
		if (tcont >= 0x8000 && tcont < 0x8000 + o->cap.tcont_n &&
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

	/* ★★ 0xffff IS THE G.988 DEALLOCATE, NOT NOISE (2026-08-05). ...
	 * dev/MEASURED-gpon_omci_me.c.md sec 37. */
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

/* THE WAN SERVICE SPINE -- where the OLT said this GEM port ...
 * dev/MEASURED-gpon_omci_me.c.md sec 38. */
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
