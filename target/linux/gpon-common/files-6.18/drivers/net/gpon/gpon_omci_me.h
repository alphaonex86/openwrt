/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIER: CORE (prefix gpon_) — decides, never touches hardware, and compiles for
 * MIPS-BE, ARM64-LE and x86.  Canonical tier rule and guard: see "THE THREE
 * TIERS" in gpon_common.h (this directory).
 *
 * gpon_omci_me.h — the ITU-T G.988 MANAGED-ENTITY model and MIB store: which
 * managed entities this ONU holds and what their attribute bytes are.  Three
 * things, and nothing else: the TABLE-DRIVEN attribute descriptors (one row per
 * class+attribute) with the one generic filler that walks them; the STATIC
 * MIB-Upload row table; and the DYNAMIC store of the instances the OLT itself
 * created, plus the context struct that holds all of it.
 *
 * It does NOT parse a PDU, dispatch a message type, build an envelope, stamp a
 * trailer or compute a MIC — that is gpon_omci_core.{h,c}, which CALLS this
 * one.  G.988 message rules are identical on every ONU ever built; the set of
 * MEs a product serves is a property of the PRODUCT.
 *
 * Table-driven is a FOOTPRINT decision (<=64 MB RAM, small NAND) and it makes
 * three cross-vendor invariants STRUCTURAL rather than per-case: every reply
 * bounded by the Get value area; the attribute set of an ME derivable, so a Get
 * NAMES what it does not support and what did not fit instead of answering
 * success with a short mask and generating OLT re-GET churn; and one policy per
 * vendor-reserved class RANGE instead of a list of the class IDs one OLT asked
 * for.
 *
 * ⚠ realtek-luna does NOT compile this yet: it carries a third, independently
 *   written model in rtl9602c_eth.c on file-scope globals, with a different
 *   identity and a different ME 7 / ME 11 attribute set.  Pointing Luna here
 *   CHANGES LUNA'S EMITTED BYTES, so it needs its own board gate (F1/F2/F3).
 *   Nothing here was altered to accommodate it.
 *
 * ENDIANNESS: every attribute value is emitted big-endian by explicit byte math
 * (omci_attr_bytes() in the .c), never a cast over wire bytes.
 */
#ifndef GPON_OMCI_ME_H
#define GPON_OMCI_ME_H

#include <linux/types.h>

/* The MESSAGE layer's facts: OMCI_LEN, the result codes omci_me_fill() returns,
 * OMCI_ATTR_BIT().  It forward-declares struct omci_onu, which this header then
 * defines, so the include runs one way only and the two never cycle. */
#include "gpon_omci_core.h"

/* The VLAN / classification model this store feeds: ME 171's row table needs
 * storage of its own (one instance holds MANY 16-octet rows, which no dense
 * body can), and the other six decode straight out of their dense bodies. */
#include "gpon_omci_vlan.h"

/* ★★ A CAPACITY IS A PER-BOARD VALUE, THE LOGIC IS COMMON (2026-08-27).  Both
 * store ceilings are overridable so a board can rebase onto this store WITHOUT
 * losing room: the Luna shell kept its own 128-entry and 200-row tables, and
 * swapping them for a fixed 64/72 would have silently dropped provisioned MEs —
 * a regression wearing the clothes of a cleanup.  One lean kernel per model, so
 * each target compiles the core with its own number. */
/* The same header chooses the layout for the NIC owner and every common
 * responder translation unit. Per-directory compiler flags split this ABI. */
#ifndef OMCI_STORE_MAX
#if defined(CONFIG_LUNA_GPON) || defined(CONFIG_LUNA_GPON_MODULE)
#define OMCI_STORE_MAX 128
#else
#define OMCI_STORE_MAX 64
#endif
#endif

/* A dynamic ME instance the OLT provisioned (Create).  Stored so GET and the
 * MIB-Upload reflect the ACTUAL configured MIB — without it the OLT's
 * post-config audit gets UNKNOWN_ME, re-runs the whole
 * MIB-Reset/Upload/Create sequence every ~50 s, and finally Deactivates. */
/* The attribute body one provisioned instance holds -- also the ceiling every
 * dense class must fit, which is why it is a name and not a literal in three
 * places. */
#define OMCI_STORE_BODY 26

struct omci_me_inst {
	u16	class_id;
	u16	inst;
	u8	body[OMCI_STORE_BODY];	/* dense attributes for a modelled
					 * class, opaque for the rest */
	u8	blen;
	bool	used;
};

/* One MIB-Upload-Next row: (class, instance, attr-mask) whose selected
 * attributes fit the 26-byte Upload-Next value area. */
struct omci_mib_row {
	u16	class_id;
	u16	inst;
	u16	mask;
};

/* Overridable for the same reason as OMCI_STORE_MAX above. */
#ifndef OMCI_MIB_ROWS_MAX
#if defined(CONFIG_LUNA_GPON) || defined(CONFIG_LUNA_GPON_MODULE)
#define OMCI_MIB_ROWS_MAX 200
#else
#define OMCI_MIB_ROWS_MAX 72
#endif
#endif

/* The auto-instantiated T-CONT range in the static MIB. */
#define OMCI_TCONT_COUNT 12

/* How many instances of ONE administratively-controlled UNI class this model
 * carries.  MEASURED in the two boards' own captured stock MIBs: ME 11 has 4
 * rows on the G24W and 4 on the X400AXF, ME 264 has 6 and 5. */
#define OMCI_UNI_MAX 8

/*
 * The instances of ONE UNI class the ONU presents, and the administrative
 * state each of them last ACCEPTED.
 *
 * ★ ME 11 (PPTP Ethernet UNI) AND ME 264 (UNI-G) GET ONE EACH, AND THE TWO
 *   INVENTORIES ARE INDEPENDENT -- not two views of one list: the X400AXF
 *   reports a UNI-G at 0x0604 with no PPTP Ethernet UNI beside it (0x0604 is
 *   not the VEIP, which is 0x0601).  Folding them would invent an instance on
 *   one class or drop one from the other.
 *
 * ★ THE KEY IS THE FULL u16.  The G24W numbers its fourth PPTP Ethernet UNI
 *   0x0401 while the first three are 0x0101..0x0103, so 0x0101 and 0x0401
 *   share a low byte: an 8-bit port index identifies nothing here.
 *
 * ★ NOTHING HERE COMBINES THE TWO STATES.  Whether a locked UNI-G also stops
 *   the PPTP -- and which physical port either is -- is a FAMILY question with
 *   a per-board answer: stock keys its Ethernet-UNI apply on the MANAGEMENT
 *   CAPABILITY value (0, 1, 2), not a port index, and the G24W runs the apply
 *   for all three while the X400AXF skips whenever the capability is 1.
 *   ⚠ THAT IS NOT "the X400AXF skips one port": all four of its Ethernet UNI-G
 *   rows report capability 1.  The core reports what was accepted, per class.
 */
struct omci_uni_inv {
	u16	inst[OMCI_UNI_MAX];	/* instance ids, [0, n) valid */
	u8	admin[OMCI_UNI_MAX];	/* G.988: 0 unlocked, 1 locked */
	u8	n;
	/* Bit i: slot i's administrative state MOVED and the family has not
	 * drained it yet.  A separate word rather than a compare against a
	 * shadow copy, because a re-Set to the value already held must NOT
	 * re-apply and a MIB-Reset back to unlocked MUST. */
	u8	changed;
};

struct omci_onu {
	u8	sn[8];			/* PLOAM serial number (vendor+VSSN) */
	u8	mds;			/* ME 2 attr 1: MIB-Data-Sync */
	/* The two UNI inventories.  ⚠ EACH WAS A SINGLE u8 FOR ONE HARDCODED
	 * INSTANCE, 0x0101: a Get on any other instance answered 0 whatever had
	 * been Set, and a Set on any other instance wrote the FIRST one's state.
	 * Both boards' stock MIBs carry four ME 11 instances, and six (G24W) /
	 * five (X400AXF) ME 264.
	 * ⚠ AND ME 264 WAS A CONSTANT ZERO BEFORE THAT: the Set was ACKed, the
	 * MIB-Data-Sync advanced, and the Get answered 0 -- so the OLT was told
	 * the write took and then shown that it had not.
	 *
	 * ★★ AND THE G24W's OWN STOCK STILL DOES EXACTLY THAT, DELIBERATELY --
	 * so our storing it is a DECLARED DELTA, not an oversight.  Executing
	 * that board's own binaries: ME 264's AdminState carries OltAcc 1
	 * against a requested Set permission of 2, the effective write mask is
	 * ZERO, and four Set requests across 0x0101, 0x0601 and 0x0801 keep the
	 * old value, advance the MIB-Data-Sync and answer 0.  Its config
	 * handler is eight bytes of `jr ra`.  Copying that back would make our
	 * own MIB-Data-Sync a lie by construction, which is the defect this
	 * model already paid for once.  Re-runnable proof, with the one-byte
	 * metadata control that makes the arms mean something:
	 * ONU-test-case/instrument/RTL9603CVD/LANLY/G24W/stock_uni_contract.py */
	struct omci_uni_inv	pptp_eth_uni;	/* ME 11, attribute 5 */
	struct omci_uni_inv	uni_g;		/* ME 264, attribute 2 */
	/* ME 264 #3 management capability, per uni_g slot.  ⚠ IT WAS THE
	 * CONSTANT 1, which was true of the ONE modelled instance and is false
	 * of the extra one each board carries: the X400AXF's 0x0604 and the
	 * G24W's 0x0601 both report 0 in their own stock MIB.  Modelling more
	 * instances while keeping the constant would have made the extended
	 * inventory contradict the capture it came from. */
	u8	uni_g_mgmt_cap[OMCI_UNI_MAX];
	/* ★ ME 11 #1/#2, PER INSTANCE.  Expected and Sensed type are the G.988
	 *   plug-in type coding, which states what the UNI IS -- not what it
	 *   negotiated -- so a fixed integrated port answers the same byte with
	 *   its link down.  It was one constant 47 for every instance, and that
	 *   is right for the GE port on both Luna boards and wrong for every FE
	 *   one, which their own stock reports as 24.  Like the switch port, it
	 *   is a fact of the panel and is not computable from the instance id. */
	u8	uni_type[OMCI_UNI_MAX];
	/* MIB-Upload rows the row table had no room for.  ⚠ THE BUILDER DROPS
	 * THEM SILENTLY, and a dropped row is an instance the OLT never learns
	 * exists and therefore never provisions -- a whole subscriber port
	 * missing, with every message on the wire succeeding. */
	u16	rows_dropped;
	u16	tcont_alloc[OMCI_TCONT_COUNT];
	u16	tcont_alloc_written;	/* accepted attr-1 Set, not a boot default */
	/* ⚠ THE ADAPTIVE MDS WALK's state.  It lived in the Luna shell and was
	 * DELETED by the 2026-08-27 responder deduplication (836b76be01) --
	 * deleted, not moved -- so omci_mds_provisionable_test has been RED on
	 * twelve arms ever since.  A container moved without its consumers. */
	u16	audit_reads;		/* DS OMCI reads since the OLT last PROVISIONED */
	u8	mds_tries;		/* how far the adaptive MDS walk has stepped */
	/* The walk's knobs, set by the shell at init so a bisect can turn the
	 * shipped behaviour off.  They live HERE, not in the shell's call, so
	 * the "GET arm and nowhere else" property is inside the core where a
	 * guard can read it. */
	bool	mds_adapt;
	u16	mds_adapt_reads;
	u16	nrows;			/* static MIB row count */
	u16	store_n;		/* provisioned-ME count */
	struct omci_mib_row	rows[OMCI_MIB_ROWS_MAX];
	struct omci_me_inst	store[OMCI_STORE_MAX];
	/* ME 171's row table.  It is NOT in the store above because one ME 171
	 * instance carries many 16-octet rows and the dense body holds 26
	 * octets total -- the subscriber VLAN simply does not fit the shape
	 * every other modelled ME uses. */
	struct gpon_vlan_model	vlan;
	/* G.988 11.2.2.1 retained last response.  The OMCC is stop-and-wait, so
	 * ONE entry covers every retransmission, and a byte-identical repeat is
	 * REPLAYED from here instead of re-executed -- otherwise a lost US
	 * response bumps MIB-Data-Sync twice for one OLT transaction. */
	u8	last_req[40];		/* bytes 0..39 (trailer+MIC derived) */
	u8	last_resp[OMCI_LEN];
	bool	have_last;
	/* spy counters (dump/probe capability is first-class, project rule) */
	u32	unhandled;		/* DS message types with no ONU action */
	u32	dup_replay;		/* retransmissions served from the cache */
	u32	rx_extended;		/* devid 0x0b frames seen (not served) */
	u32	rx_bad_mic;		/* DS frames DISCARDED on an invalid MIC */
	u32	rx_runt;		/* DS frames shorter than a 48-byte baseline PDU */
	u32	no_ack;			/* requests with AR clear: applied, not
					 * answered — a silent path must still be
					 * countable */
	u32	avc_count;		/* autonomous AVC frames emitted */
	bool	avc_veip_up_sent;
	/* ★★ THE ALARM STATE.  Our ONU saw LOS/LOF in silicon, printed it to
	 * /proc and told every OLT that nothing was wrong.  The core owns the
	 * MESSAGE, the EDGE and the ACCOUNTING; the family owns only which
	 * silicon bit means what, handed in as (class, instance, bitmap). */
	u16	alarm_class;	/* the ME the conditions belong to (0 = none) */
	u16	alarm_inst;
	u16	alarm_active;	/* what the SILICON asserts right now */
	u16	alarm_told;	/* what the OLT has been told -- the EDGE */
	u8	alarm_seq;	/* G.988 alarm sequence number, wraps at 255 */
	u32	alarm_emitted;	/* spy counter: autonomous alarms sent */
	/* ME 263 ANI-G #10 RX / #14 TX optical level in the G.988 wire form
	 * (2's complement, 0.002 dB steps referred to 1 mW).  Seeded by
	 * omci_onu_init() to the static fallback below and overwritten by the
	 * shell from the live SFF-8472 A2h DDM read.  The fallback survives a
	 * FAILED read because the OLT must never get silence, and @anig_live
	 * says which of the two a reader is looking at, so a stub is never
	 * mistaken for a measurement.  The host oracle never calls the setter,
	 * so its GET responses stay byte-identical to the reference snapshot. */
	u16	anig_rx_level;
	u16	anig_tx_level;
	bool	anig_live;
};

/* The static ANI-G optical levels served until (and after a failed) DDM read:
 * -8.77 dBm received, +2.47 dBm launched.  Both plausible for this class-B+
 * optic, which is exactly why they must be labelled — a fabricated value that
 * looks right is the hardest kind to notice. */
#define OMCI_ANIG_RX_FALLBACK	0xeedc
#define OMCI_ANIG_TX_FALLBACK	0x04d7

/* Publish a live optical measurement into ME 263 #10/#14.  The caller does the
 * SLEEPING i2c read outside whatever lock guards the responder and passes the
 * two already-converted wire values in. */
static inline void omci_onu_set_optical(struct omci_onu *o, u16 rx_level,
					u16 tx_level)
{
	o->anig_rx_level = rx_level;
	o->anig_tx_level = tx_level;
	o->anig_live = true;
}

/* Re-provision the G.984.3 ONU serial after init.
 * ★ A SETTER AND NOT A SECOND COPY IN THE SHELL: a shell may only learn the
 *   real serial from PLOAM after probe, and re-running omci_onu_init() to
 *   deliver it would ZERO the whole MIB mid-session — created instances, MDS
 *   and all.  The one reader serves these bytes at GET time, so writing them is
 *   complete and no row needs rebuilding.
 * ⚠ THE ALTERNATIVE ALREADY WENT WRONG ONCE: the Luna shell's
 *   set_omci_identity() copies the serial into a PRIVATE omci_sn[8] that only a
 *   /proc line has read since the responder was rebased onto this core, so
 *   ONU-G there still answers whatever probe happened to seed. */
static inline void omci_onu_set_sn(struct omci_onu *o, const u8 sn[8])
{
	unsigned int i;	/* a loop, not memcpy(): this header includes only
			 * <linux/types.h>, the same declared deviation the
			 * flowcore tier already takes for ether_addr_copy */
	for (i = 0; i < 8; i++)
		o->sn[i] = sn[i];
}

/* ME class IDs presented in the MIB upload (G.988 + the HSGQ OLT's set).
 * ONU_DATA and VEIP are also defined identically by gpon_omci_core.h, which
 * reasons about those two itself; a repeated object-like #define with the same
 * replacement list is a benign redefinition, so each header stays readable on
 * its own. */
#define OMCI_ME_ONU_DATA	2
#define OMCI_ME_CARDHOLDER	5
#define OMCI_ME_CIRCUIT_PACK	6
#define OMCI_ME_SW_IMAGE	7
#define OMCI_ME_PPTP_ETH_UNI	11	/* THE HGU gate: the OLT's
					 * gpon_ont_sync_capability counts these */
/* ★★ THE WAN SERVICE SPINE (G.988 clause 9.3): the classes through which an
 * OLT says WHERE a GEM port goes.  268 names the GEM, 266 interworks it, 47
 * hangs it on a bridge port and 45 is the bridge; 272 carries the GEM payload
 * ceiling and 50/52 are the bridge port's read-back views.  Until these were
 * modelled a Create of any of them was ACKed and stored opaquely, so an OLT
 * that expresses the service here provisioned us successfully and got a data
 * path built from uci instead -- which works only because this bench's OLT
 * also accepts untagged IPoE.
 * Attribute widths and access below are EXTRACTED from the board's own stock
 * plugins, not read off a spec from memory:
 *   dev/re-tools/venv/bin/python3 dev/OMCI-simulate/me_attr_table.py \
 *       --so <stock rootfs>/lib/omci/mib_<Name>.so                          */
#define OMCI_ME_MAC_BRIDGE_SVC	45	/* mib_MacBriServProf */
#define OMCI_ME_MAC_BRIDGE_PORT	47	/* mib_MacBriPortCfgData */
#define OMCI_ME_MAC_BRIDGE_FILTER 49	/* mib_MacBridgePortFilterTable -- the
					 * per-bridge-port MAC filter table.
					 * ⚠ 49 IS THE FILTER TABLE and 52 is
					 * PM history; the numbers are the
					 * measurement and the names are not */
#define OMCI_ME_MAC_BRIDGE_TABLE 50	/* mib_MacBriPortBriTblData */
#define OMCI_ME_MAC_BRIDGE_PM	52	/* mib_MacBridgePortPmMonitorHistoryData
					 * -- COUNTERS.  ⚠ class 49 is the filter
					 * table; 52 is PM history, and getting
					 * that backwards turns the cheapest row
					 * in the gap list into a datapath repair */
/* The VLAN / CLASSIFICATION half of the same service model.  Widths and access
 * extracted the same way, from the same plugins, and MEASURED IDENTICAL on both
 * Luna dies (X111W RTL9602C 3.18 and G24W RTL9603CVD 4.4) — the one difference
 * in all seven is ME 171 #8's DataType, which is a per-BUILD type vocabulary
 * and not a per-die attribute layout.  Semantics: gpon_omci_vlan.h. */
#define OMCI_ME_VLAN_TAG_OP	78	/* mib_VlanTagOpCfgData -- the pre-171
					 * single-tag operation ME */
#define OMCI_ME_PREASSIGN_FILTER 79	/* mib_MacBridgePortFilterPreassign --
					 * the per-protocol drop matrix */
#define OMCI_ME_VLAN_TAG_FILTER	84	/* mib_VlanTagFilterData -- the
					 * per-bridge-port VLAN admit list */
#define OMCI_ME_PBIT_MAPPER	130	/* mib_Map8021pServProf -- the OTHER way
					 * a GEM reaches a bridge port */
#define OMCI_ME_EXT_VLAN	171	/* mib_ExtVlanTagOperCfgData -- THE
					 * SUBSCRIBER VLAN.  Its rows ride inside
					 * the ME 47 frame in stock; only the
					 * DSCP half gets a command of its own */
#define OMCI_ME_OLT_G		131
#define OMCI_ME_ONU_G		256
#define OMCI_ME_ONU2_G		257
#define OMCI_ME_TCONT		262
#define OMCI_ME_ANI_G		263
#define OMCI_ME_UNI_G		264
#define OMCI_ME_GEM_IW_TP	266	/* mib_GemIwTp -- THE LINK: G.988 joins
					 * the GEM CTP below to a bridge port
					 * THROUGH this ME */
#define OMCI_ME_GEM_CTP		268	/* GEM Port Network CTP -- the ME that
					 * NAMES the WAN data GEM Port-ID */
#define OMCI_ME_GAL_ETH_PROF	272	/* mib_GalEthProf -- 266's companion */
#define OMCI_ME_PRIORITY_QUEUE	277
#define OMCI_ME_TRAFFIC_SCHED	278
#define OMCI_ME_GEM_TRAFFIC_DESC 280	/* mib_GemTrafficDescriptor -- CIR/PIR */
#define OMCI_ME_MCAST_GEM_IW_TP	281	/* mib_MultiGemIwTp -- the DS multicast
					 * path.  It NAMES the multicast GEM,
					 * which both our shells hardcode to
					 * GPON_MCAST_GEM_PORT today */
#define OMCI_ME_VEIP		329
#define OMCI_ME_CTC_LOID_AUTH	65530	/* 0xFFFA — CTC extension the OLT audits */

/* ★★ THE MIB-DATA-SYNC POISON SEED IS PROTOCOL POLICY, AND IT LIVES ONCE.
 * We hold no persistent MIB, so a warm re-admit MUST make the OLT re-provision
 * from MIB-Reset.  Two mechanisms exist and they are NOT equally strong:
 *   1..30      satisfies this OLT's own gate UNCONDITIONALLY — MEASURED on the
 *              Luna side: it treats rsync < 31 as not-in-sync and re-provisions
 *              whatever lsync it stored;
 *   any other  works only by MISMATCH against the stored lsync, and fails
 *              exactly when the OLT stored OUR OWN previous seed (the X111W
 *              warm-readmit lesson).  The Cortina shell carried a literal 200
 *              for weeks, CITING that lesson while using the value it argues
 *              against.
 * The default is the measured-safe band; a shell may expose a tunable, but its
 * DEFAULT is this. */
#define OMCI_MDS_POISON_SEED	7

/* ★★ THE WALK: WHAT TO DO WHEN THE SEED IS NOT ENOUGH.  The seed makes the
 * FIRST admit re-provision and cannot help once the OLT has STORED that value:
 * from then on rsync == lsync, every escape clause in its audit is false, and
 * it reads us forever provisioning nothing — a poisoned value that is STABLE is
 * indistinguishable from being in sync.  So after N reads with no MIB-Reset and
 * no applied config, STEP the reported value.
 * ⚠ 0 IS EXCLUDED FROM THE SEARCH SPACE: G.988 gives it the meaning "just
 *   MIB-Reset", and folding 0 onto 1 splices the orbit into an 83-value CYCLE
 *   instead of an exhaustive 1..255 walk. */
#define OMCI_MDS_WALK_STEP	37	/* coprime with 255: enumerates 1..255 */
#define OMCI_MDS_ADAPT_READS	12	/* reads with no provisioning before a step */

/* Step the reported MIB-Data-Sync when the OLT reads without provisioning.
 * Call ONLY from the GET arm: a timer-driven walk would step on a link the OLT
 * is not even reading. */
void omci_mds_walk(struct omci_onu *o);

/* Does this DS frame's AAL5-BE MIC verify?  A frame shorter than OMCI_LEN
 * cannot carry one and is therefore NOT ok: unverifiable is not acceptable. */
bool omci_mic_ok(const u8 *msg, unsigned int len);

/* COLD init: @o need not have been initialised, and this reads nothing out of
 * it before the memset.  A shell declares its UNI inventory straight after. */
void omci_onu_init(struct omci_onu *o, const u8 sn[8], u8 mds_seed);

/*
 * Re-init a LIVE model for a new identity, carrying its declared UNI inventory
 * across.  This is the one to call on an identity change.
 *
 * ⚠ omci_onu_init() MEMSETS, and a board does not gain or lose UNIs when its
 *   serial number changes: a bare re-init drops three of four ME 11 instances
 *   out of the MIB mid-session, which the OLT discovers only at its next
 *   upload.  The two are separate functions because the cold caller's object
 *   is indeterminate -- several callers pass an uninitialised stack struct --
 *   so the cold path may not read the inventory it would need to preserve.
 */
void omci_onu_reinit(struct omci_onu *o, const u8 sn[8], u8 mds_seed);

/*
 * Declare which instances of ME 11 and ME 264 this board presents, and rebuild
 * the MIB-Upload rows around them.  Called by the FAMILY shell after
 * omci_onu_init(); until it is, the model carries the single 0x0101 of each
 * that every board on the bench has.
 *
 * The two lists are independent and either may be EMPTY -- a board with no
 * Ethernet UNI at all is a legal declaration, not a request for the default.
 * @unig_mgmt_cap is ME 264 #3 per instance; NULL means 1 for all of them.
 *
 * Refuses a list longer than OMCI_UNI_MAX, a zero instance id, a duplicate, or
 * a panel whose rows do not fit the MIB table -- each of those would put the
 * MIB and the OLT's copy of it permanently out of step.
 *
 * ★ A REFUSAL CHANGES NOTHING, AND THAT INCLUDES THE STATE A PORT DEPENDS ON:
 *   the accepted administrative states and the undrained apply obligations are
 *   exactly as they were.  The capacity case is checked by building and putting
 *   the previous panel back, so there is no second capacity model to keep in
 *   step with the row builder.
 */
bool omci_onu_declare_unis(struct omci_onu *o,
			   const u16 *pptp_inst, const u8 *pptp_type, u8 pptp_n,
			   const u16 *unig_inst, const u8 *unig_mgmt_cap,
			   u8 unig_n);

/* What ME 11 #1/#2 answer for an instance nobody typed: the value this model
 * hardcoded for every UNI before the declaration existed, so a board that
 * declares no types behaves exactly as it did. */
#define OMCI_UNI_TYPE_DEFAULT	47

/* What a board's declaration turned out to be. */
enum omci_uni_decl {
	OMCI_UNI_DECL_OK,	/* installed */
	OMCI_UNI_DECL_ABSENT,	/* nothing declared: the default is kept */
	OMCI_UNI_DECL_BAD,	/* declared and malformed, or it does not fit:
				 * NOTHING was changed, the accepted admin
				 * states and pending obligations included */
};

/*
 * Decode a board's panel out of the RAW BYTES of its declaration and install
 * it.  The instance lists are 16-bit BIG-ENDIAN, which is how a device tree
 * stores them; @cap is one byte per UNI-G.  A negative length means the
 * property is ABSENT, which is a different answer from present-and-empty.
 *
 * ★ IT IS HERE, AND IT TAKES BYTES, FOR TWO REASONS.  The core may hold no
 *   device-tree handle (it builds on x86 against no kernel at all), and both
 *   families were about to carry their own copy of this parsing -- which is
 *   how they came to share four malformed-input defects on the day they were
 *   written.  Bytes in, panel out, fuzzable on a host.
 *
 * ★ EVERY LENGTH IS CHECKED RATHER THAN ROUNDED.  An odd byte count is not a
 *   list with the tail dropped, a capability array that is short or long is
 *   not one padded with the old constant, and a list written in the device
 *   tree's DEFAULT 32-bit cell width is none of those things either -- it is
 *   a board saying something it did not mean, and the only safe answer is to
 *   refuse it and say which property.
 *
 * ★ AND cap DEFAULTS TO 1 ONLY WHEN IT IS ABSENT.  Present-but-wrong silently
 *   becoming all-ones is exactly the failure that would make an extended
 *   inventory contradict the capture it came from.  @type is one byte per
 *   ETHERNET UNI and follows the same rule against OMCI_UNI_TYPE_DEFAULT.
 */
enum omci_uni_decl omci_onu_declare_unis_be(struct omci_onu *o,
					    const void *pptp_be, int pptp_len,
					    const void *type, int type_len,
					    const void *unig_be, int unig_len,
					    const void *cap, int cap_len,
					    const char **why);

/* The inventory slot holding @inst, or NULL when this board has no such
 * instance.  Answers on the FULL u16. */
int omci_uni_slot(const struct omci_uni_inv *inv, u16 inst);

/*
 * Take the set of slots whose administrative state moved since the last call,
 * and clear it.  This is the seam the family applies to a port and its PHY.
 *
 * ★ THE CORE NEVER NAMES A PORT, A PHY OR A REGISTER.  Which physical socket
 *   an instance is remains unproven on both boards -- the mapping seen so far
 *   is what each board's own stock MIB REPORTS, which is not a connector
 *   proof -- so the family owns it and the core hands over an instance id.
 *
 * ★ AND A PHYSICAL FAILURE IS NOT REPAIRABLE FROM HERE.  The Set response is
 *   already committed (and a duplicate-TID replay answers from the cache
 *   without reaching this path at all), so a family that cannot bring a port
 *   down must report that as its own fault -- never by rewriting a response
 *   byte after the fact, which would claim the model failed when it did not.
 */
u8 omci_uni_take_changed(struct omci_uni_inv *inv);

/*
 * Put a drained obligation BACK, because the family could not apply it.
 *
 * ★ AN APPLY THAT FAILED IS STILL OWED.  take_changed() clears the word, so a
 *   family that drains a slot and then cannot drive its port would leave the
 *   model reporting a lock the socket never took, with nothing left to retry
 *   from: no later Set of the same value re-arms it (that is deliberate) and a
 *   MIB-Reset only covers the unlock direction.
 */
void omci_uni_mark_changed(struct omci_uni_inv *inv, u8 slot);

/*
 * ★★★ DRAINING THE OBLIGATION IS G.988 WORK, AND IT WAS WRITTEN TWICE
 * (measured 2026-09-14).  `luna_uni_apply_work_fn` and `cg_uni_apply_work` are
 * two spellings of one rule: walk the taken changed-mask, drive each flagged
 * slot, and — the half that is easy to get wrong — put the obligation BACK
 * whenever the port did not take it, arming a retry timer only when the
 * failure can still go away.  Nothing in that decision is per-silicon: which
 * slots are flagged is the model's, what a permanent failure means is G.988's,
 * and only the CALL and the LOCK around it belong to a shell.
 *
 * ⚠ THE TWO COPIES HAD ALREADY DRIFTED IN THEIR PROSE.  Cortina's carried a
 * note that dropping the obligation on a permanent error "made the text above
 * a lie"; Luna's arrived at the same behaviour through a different expression
 * (`retry` gated on the backend existing) and said nothing about it.  Two
 * bodies agreeing today with no shared statement of WHY is the state a repair
 * to one of them ends.
 */
enum omci_uni_apply_rc {
	/* the port took the administrative state: the obligation is discharged */
	OMCI_UNI_APPLIED = 0,
	/* THIS BOARD HAS NO PORT for that slot — permanent.  The obligation is
	 * retained (a corrected port list must still apply what the OLT Set)
	 * and it earns NO retry timer: spinning on it would never converge. */
	OMCI_UNI_NO_PORT,
	/* the backend refused and may not next time: retained AND retried. */
	OMCI_UNI_TRANSIENT,
};

/**
 * struct omci_uni_apply_ops - the two doors the drain needs into a shell
 * @apply: drive UNI slot @slot to @locked (G.988: 1 = locked).  The shell maps
 *         its own backend's failure onto the enum; the core never sees an
 *         errno, a port number or a register.
 * @rearm: put slot @slot's obligation back — omci_uni_mark_changed() under
 *         whatever lock that shell keeps the model under, which is why the
 *         core cannot do it itself.
 *
 * Both are MANDATORY: a drain with no way to re-arm silently loses a lock the
 * OLT has already been told took effect.
 */
struct omci_uni_apply_ops {
	enum omci_uni_apply_rc (*apply)(void *sh, u8 slot, bool locked);
	void (*rearm)(void *sh, u8 slot);
};

/**
 * omci_uni_apply_run() - drive one drained changed-mask onto the ports
 * @ops:     the shell's two doors; a NULL member makes this a no-op that
 *           re-arms nothing, because losing the obligation silently is worse
 *           than not draining it.
 * @sh:      opaque shell handle, handed back to every op
 * @changed: the mask omci_uni_take_changed() returned
 * @n:       how many slots the inventory declares
 * @admin:   the snapshot of the administrative states, @n entries
 *
 * Return: true when a RETRY TIMER is owed — at least one slot failed in a way
 * that can still succeed.  A board with no port for a slot never sets it.
 */
bool omci_uni_apply_run(const struct omci_uni_apply_ops *ops, void *sh,
			u8 changed, u8 n, const u8 *admin);

/* The ME-model API the MESSAGE layer calls.  These nine were `static` while the
 * model and the message rules shared one translation unit; the split is the only
 * reason they are declared here, and the contract of each stays written at its
 * DEFINITION in gpon_omci_me.c — a duplicated contract in a header drifts from
 * the code it describes. */

/* dynamic (OLT-provisioned) ME store */
struct omci_me_inst *omci_store_find(struct omci_onu *o, u16 class_id, u16 inst);
bool omci_store_has_class(struct omci_onu *o, u16 class_id);
struct omci_me_inst *omci_store_nth(struct omci_onu *o, u16 idx);
bool omci_store_put(struct omci_onu *o, u16 class_id, u16 inst,
		    const u8 *body, int blen);
void omci_store_merge(struct omci_me_inst *e, const u8 *val, int vlen);
bool omci_store_create(struct omci_onu *o, u16 class_id, u16 inst,
		       const u8 *body, unsigned int blen);
bool omci_me_mutable(u16 class_id);
/* octets of DENSE attribute body this class's descriptor rows describe, 0 when
 * its Create body is kept opaque */
u8 omci_me_dense_len(u16 class_id);
void omci_me_reset_values(struct omci_onu *o);
u8 omci_me_set(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
	       const u8 *values, unsigned int len, u16 *unsupported,
	       u16 *failed);
void omci_store_del(struct omci_onu *o, u16 class_id, u16 inst);

/* the descriptor-table engine: emit @mask's attributes into [v..end) in
 * descriptor order, reporting what was emitted and what this ME models */
u8 omci_me_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
		u8 *v, const u8 *end, u16 *rmask_out, u16 *known_out);

/* does the model carry this class at all (descriptor row or vendor range)? */
bool omci_class_modelled(u16 class_id);

/* is (class, inst) a MIB instance this ONU holds? */
bool omci_inst_exists(struct omci_onu *o, u16 class_id, u16 inst);

/* ★★★ THE WAN SERVICE SPINE — where the OLT said a GEM port GOES.
 *
 * G.988 clause 9.3 expresses a service as a chain of POINTERS, and the ONU can
 * only answer "which bridge is my WAN on" by following it:
 *
 *     ME 268 (GEM port CTP)  <- #1 of  ME 266 (GEM interworking TP)
 *     ME 266                 <- #4 of  ME 47  (MAC bridge port config data)
 *     ME 45  (bridge)        <- #1 of  ME 47
 *     ME 272 (GAL Eth prof)  <- #7 of  ME 266
 *
 * Each bit of @have says one LINK resolved, and they are cumulative in that
 * order: a chain that stops names exactly how far the OLT's provisioning got,
 * which is the difference between "the OLT has not finished" and "the OLT means
 * something we do not implement".
 *
 * ⚠ REPORTING, NOT INSTALLING.  Nothing in either family calls it yet: the WAN
 *   above the GEM is ours from uci and the GEM install stays gated on ME 268 +
 *   ME 262 + PLOAM.  It exists so the OLT's intent and our installed path can
 *   be COMPARED at all — before this, they could not be.
 *
 * ★ BOTH ROUTES ARE WALKED (the second one landed 2026-09-14).  G.988 lets a
 *   bridge port reach a GEM interworking TP EITHER directly (ME 47 #4 -> ME 266)
 *   OR through an 802.1p mapper service profile (ME 47 #4 -> ME 130, whose eight
 *   P-bit pointers each name an ME 266).  An OLT using the mapper is not exotic,
 *   and before the second route this reported the chain stopping at the
 *   interworking TP — true about what resolved, and read by a human as "the OLT
 *   provisioned nothing".  @have names WHICH route was taken, because the two
 *   are different service models and not two spellings of one.
 *
 *     ME 268 (GEM port CTP)  <- #1 of  ME 266 (GEM interworking TP)
 *     ME 266                 <- #4 of  ME 47   ... the DIRECT route
 *                            <- #2..#9 of ME 130 <- #4 of ME 47 ... the MAPPER
 */
#define OMCI_SVC_GEM_CTP	0x01	/* an ME 268 carries this Port-ID */
#define OMCI_SVC_IW_TP		0x02	/* an ME 266 interworks that CTP */
#define OMCI_SVC_BRIDGE_PORT	0x04	/* an ME 47 points at that ME 266 */
#define OMCI_SVC_BRIDGE		0x08	/* and its ME 45 bridge EXISTS */
#define OMCI_SVC_GAL		0x10	/* the ME 266's GAL profile exists */
#define OMCI_SVC_PBIT_MAPPER	0x20	/* ...reached THROUGH an ME 130, not
					 * directly: a different service model,
					 * and the caller is entitled to know */

struct omci_service_path {
	u16	gem_port;	/* what was asked for */
	u16	gem_ctp;	/* ME 268 instance carrying it */
	u16	iw_tp;		/* ME 266 instance */
	u16	iw_option;	/* ME 266 #2, carried as DATA */
	u16	gal_prof;	/* ME 266 #7 */
	u16	gal_payload;	/* ME 272 #1 */
	u16	bridge_port;	/* ME 47 instance */
	u16	tp_type;	/* ME 47 #3, carried as DATA — never branched on:
				 * no TP-type coding has been measured here */
	u16	bridge;		/* ME 47 #1 -> the ME 45 instance */
	u16	mapper;		/* the ME 130 instance, when the mapper route was
				 * taken; 0 on the direct one */
	u8	pbit;		/* which P-bit of that mapper named the ME 266 */
	u8	have;		/* OMCI_SVC_* — which links resolved */
};

u8 omci_service_resolve(struct omci_onu *o, u16 gem_port,
			struct omci_service_path *p);

/* The bridge this GEM's service model lands on, or 0 when the chain does not
 * reach one.  The flat form, for callers that cannot take the struct. */
u16 omci_service_bridge_of_gem(struct omci_onu *o, u16 gem_port);

/* ★★★ WHICH ME 268 IS THE WAN DATA GEM — a decision, in the core, once.  Both
 * targets answered it privately and they had DIVERGED:
 *
 *   rule                                    elnath   luna (pre-2026-08-27)
 *   direction must be BIDIRECTIONAL          yes      ABSENT
 *   refuse the OMCC's own GEM                yes      ABSENT
 *   refuse Port-ID 0                         yes      ABSENT
 *   refuse the multicast GEM                 via dir  yes, by port-id
 *
 * ⚠ AND LUNA'S COPY IS NOT MERELY WEAKER, IT IS GONE: its ME 268 snoop lived
 *   inside the shell's own responder and was deleted with it by 836b76be01, the
 *   core gained no replacement, so `data_gem_solicited` lost its only setter and
 *   the WAN data GEM is never installed from the OLT's Create at all.
 * ★ A QUERY AND NOT A NOTIFICATION, so it needs no callback and no lifecycle:
 *   the responder ALREADY stores every Set-by-Create body, so the answer is a
 *   pure read of state the core holds anyway, and it is exercisable on x86.
 * ★ THE GEOMETRY STAYS THE SHELL'S: @omcc_gem and @mcast_gem are INPUTS. */

/* Why a candidate ME 268 is, or is not, the WAN data GEM.  Six outcomes,
 * because "not the data GEM" is five different facts and a shell that cannot
 * say WHICH one is a shell that cannot explain a dead WAN. */
enum omci_dgem {
	OMCI_DGEM_YES = 0,	/* adopt: bidirectional, and nobody else's */
	OMCI_DGEM_RUNT,		/* Create body too short to carry attr 1..3 */
	OMCI_DGEM_ZERO,		/* Port-ID 0 is not a provisioned port */
	OMCI_DGEM_IS_OMCC,	/* the management GEM -- adopting it as the
				 * data GEM points the WAN at the OMCC */
	OMCI_DGEM_IS_MCAST,	/* the broadcast GEM (this OLT Creates it FIRST,
				 * so a first-match rule picks it by default) */
	OMCI_DGEM_NOT_BIDIR,	/* a uni-directional CTP: G.988 direction != 3 */
};

/* Classify ONE stored ME 268 Set-by-Create body.  @body/@blen are the bytes the
 * store holds (attribute 1 first, i.e. the wire from octet 8).  On
 * OMCI_DGEM_YES, *@port_id is the 12-bit G.984.3 wire Port-ID.
 * Pure: no state, no side effect, safe from any context.
 * ⚠ @blen IS A u8, SO THE CALLER MUST CLAMP -- NEVER CAST.  A shell that wrote
 *   `(u8)(len - 8)` WRAPPED: len 264 became 0 and read as a RUNT, and len 512
 *   became 248 -- a plausible-looking body length, which is the worse of the
 *   two.  Bound the frame to OMCI_LEN first, then subtract the 8-octet header. */
enum omci_dgem omci_dgem_classify(const u8 *body, u8 blen,
				  u16 omcc_gem, u16 mcast_gem, u16 *port_id);

/* One-line name for a verdict, for logs and test failure messages. */
const char *omci_dgem_name(enum omci_dgem v);

/* Copied committed state, including explicit absence and unresolved pointers.
 * alloc_id is raw and meaningful only when alloc_known: this layer does not
 * decide whether an allocation is usable by a hardware family. */
struct omci_data_binding {
	bool gem_present;
	bool alloc_known;
	u8 direction;
	u16 gem_inst;
	u16 gem_port;
	u16 tcont_inst;
	u16 alloc_id;
};

/* Caller holds the same serialization as input/reset for this entire copy.
 * Selection remains the first eligible GEM, even if its T-CONT is unresolved. */
void omci_data_binding_snapshot(const struct omci_onu *o, u16 omcc_gem,
				u16 mcast_gem, struct omci_data_binding *binding);

/* Walk the provisioned store and return the WAN data GEM Port-ID, if the OLT
 * has created one.  -> false when it has not, which is the NORMAL state before
 * provisioning and must never be read as a failure. */
bool omci_data_gem_port(struct omci_onu *o, u16 omcc_gem, u16 mcast_gem,
			u16 *port_id);

/* ★ THE DATA-PATH SNOOP'S OTHER TWO DECISIONS — core, once (2026-09-02).  Same
 * shape and reason as omci_dgem_classify(): the Elnath shell answered "does
 * this ME 262 move or detach the data alloc-id?" and "does this ME 268 Delete
 * name the latched data GEM?" privately, and Luna has NO copy at all since
 * 836b76be01, so the next board would have re-derived both from G.988.  The
 * SHADOW is an INPUT: the core decides, the shell keeps the CAM writes. */

/* What one MIC-verified ME 262 (T-CONT) PDU means for the DATA alloc-id. */
enum omci_tcont_verdict {
	OMCI_TCONT_NONE = 0,	/* nothing actionable: not a Create/Set, runt
				 * body, attr 1 absent from the Set mask,
				 * alloc 0, or the value already latched */
	OMCI_TCONT_ALLOC,	/* a NEW data alloc-id -- *@alloc holds it */
	OMCI_TCONT_DEALLOC,	/* the G.988 0xffff detach of the latched
				 * instance: the teardown half of the message */
};

/* Decide from @mt (the raw msg-type octet; masked here) and @body/@blen (the
 * wire from octet 8).  A Set carries {mask[0:1], alloc[2:3] when the attr-1 bit
 * is set}; a Create's SBC body has the alloc first.  @cur_alloc / @cur_inst are
 * the caller's latched shadow (0 = none).  Pure, safe from any context. */
enum omci_tcont_verdict omci_tcont_snoop(u8 mt, const u8 *body,
					 unsigned int blen, u16 inst,
					 u16 cur_alloc, u16 cur_inst,
					 u16 *alloc);

/* Does this ME 268 Delete tear down the latched data GEM?  A Delete carries
 * only the class and the instance, which is exactly why the Create had to latch
 * @cur_inst.  @cur_inst 0 means "never latched" and matches any instance — the
 * shell's pre-existing permissive fallback, kept bit-for-bit. */
bool omci_dgem_delete(u8 mt, u16 inst, u16 cur_gem, u16 cur_inst);

#endif /* GPON_OMCI_ME_H */
