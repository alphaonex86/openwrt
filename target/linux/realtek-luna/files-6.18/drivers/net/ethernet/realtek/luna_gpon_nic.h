/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The Luna GPON<->NIC contract: what luna_gpon.c and whichever Ethernet
 * shell the board builds (rtl9602c_eth.c on taroko, luna_eth.c on
 * interaptiv) owe each other. Independent implementation from the SoC's
 * register interface
 * and the G.984/G.988 protocols. Once the OLT assigns the OMCC GEM port and the
 * GPON driver installs the OMCC GEM datapath, it arms the NIC OMCI trap so that
 * downstream OMCI frames on the OMCC stream-id are delivered to the CPU netdev.
 */
#ifndef _LUNA_GPON_NIC_H
#define _LUNA_GPON_NIC_H

struct omci_onu;
/*
 * @uni_admin_set: drive ONE Ethernet UNI's administrative state onto its switch
 *	port and PHY.  -> 0 applied, negative NOT applied, in which case the
 *	obligation stays owed in the model and is retried.
 *
 * ★★ ONE PENDING/APPLY OWNER, TWO BACKENDS.  The bookkeeping -- which slot
 *    changed, which port that is on this board, what is still owed -- lives
 *    once in luna_gpon.c for both Luna boards.  Only the register work differs,
 *    and it differs a lot: the G24W builds luna_eth.c and the X111W builds
 *    rtl9602c_eth.c with CONFIG_LUNA_ETH=n, so a setter living in either one
 *    closes exactly one board.  May be NULL, which means this board models and
 *    answers the administrative state and never applies it.
 */
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

/* DS OMCI frames that reached the CPU NIC ring (defined in rtl9602c_eth.c). */
/* ★★ "COULD NOT ASK" IS NOT ZERO (2026-09-08).  The Ethernet shell that serves
 * the RTL9603CVD implements no CPU-side OMCI datapath yet -- its whole OMCI glue
 * is empty stubs -- and this counter returned a literal 0.  Every activation log
 * therefore printed `omcirx=0`, which reads as "the OLT sent us no OMCI", and it
 * was read that way during this very investigation while the PON-IP's own OMCI
 * counter was proving 12 frames had arrived.  A shell with no path returns
 * GPON_OMCI_RX_UNAVAIL and the printers render it `-1`. */
#define GPON_OMCI_RX_UNAVAIL	0xffffffffu
u32 rtl9602c_eth_omci_rx_count(void);

/* gpon0 (WAN) RX packet count, used by the GPON O5 provisioning watchdog.
 * Defined by whichever Ethernet shell this board builds.
 *
 * ★ THE SAME "COULD NOT ASK IS NOT ZERO" RULE AS THE OMCI COUNTER ABOVE: a
 * shell with no WAN netdev returns GPON_OMCI_RX_UNAVAIL, never 0, because 0
 * here reads as "the OLT forwarded us no downstream data" -- a DEVICE finding
 * drawn from an absent instrument.  ⚠ THE TWO SHELLS DISAGREE TODAY: the
 * LUNA_ETH one returns UNAVAIL when gpon0 is absent, the RTL9602C_ETH one still
 * returns 0.  That is OWED A REPAIR on the 9602C side; it is recorded here
 * rather than silently fixed because that file was held elsewhere.
 *
 * ⚠ AND A REAL 0 IS STILL TWO THINGS: no downstream data, or a demux that does
 * not recognise the port it arrives on.  The counter cannot tell them apart and
 * must not pretend to -- the shells' per-ingress-port log ledger is what does. */
u32 rtl9602c_eth_wan_rx_count(void);

/* US-OMCI TX-ring reclaim cursor ("dirty"): count of OMCC descriptors the HW has
 * consumed (OWN cleared). Non-zero => the OMCC TX ring is being fetched. Defined
 * in rtl9602c_eth.c; surfaced for the periodic O5 serial diagnostic. */
u32 rtl9602c_eth_omci_tx_dirty(void);

/* US-OMCI responses the shell's TX ring REFUSED.  Read beside the queued count
 * and the PON-IP's own OMCI_TX_PKT_CNT: queued>0 & dropped=0 & pi_ustx=0 means
 * the frames left the MAC and the fabric swallowed them, which is a different
 * fault from a ring that would not take them.  Defined by whichever Ethernet
 * shell this board builds; the LUNA_ETH one returns a real count, the 9602C one
 * does not implement it. */
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

/* Live ANI-G (ME 263) optical levels for the OMCI responder, in G.988 encoding
 * (2's-complement s16, 0.002 dB referred to 1 mW): #10 RX signal level, #14 TX
 * level. Reads a cache refreshed on the periodic FSM tick from the RTL8290B's
 * calibrated SFF-8472 DDM page — no I2C in the (softirq) GET path. Defined in
 * luna_gpon.c. */
void gpon_anig_optical_omci(s16 *rx_level, s16 *tx_level);

/* Faithful port of the stock SDK rtk_all_module_init() GPON datapath bring-up,
 * run on the quiescent switch in the eth reset path (after the GMAC reset + swcore
 * resync, before the GMAC is programmed/armed). Defined in luna_gpon.c. */
void rtl9602c_datapath_tables_init(void);

/* WAN data-GEM datapath. GPON_DATA_FLOW = the internal SID/flow the gpon0 WAN netdev
 * steers US frames to (tx_dst_stream_id).
 * Shared so the eth driver's gpon0 TX descriptor uses the same SID the GPON install programs.
 *
 * ★ THE WIRE GEM PORT-ID IS NOT A CONSTANT, AND THE OLD NAME SAID IT WAS.
 * GPON_DATA_GEM was commented "the OLT-assigned wire gem-port-id" while being a
 * compile-time 193, and the Port-ID the OLT really assigns (ME 268 attribute 1)
 * was logged and discarded. Measured: this same OLT gives gem 223 to one board
 * and 193 to this one. The live value now lives in gpon_data_gem_port, set from
 * the ME 268 snoop; this macro is only the pre-OLT default and is named for what
 * it is. A misleading name is a defect — renamed the day it was proven wrong. */
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

/* Publish the cached live DDM optical levels into ME 263 ANI-G #10/#14.
 * Called from the GPON driver's 3 s optical workqueue.
 *
 * ⚠ THE CALLER AND THE CALLEE ARE BUILT UNDER DIFFERENT CONFIG SYMBOLS, and
 *   that broke a board. luna_gpon.c compiles under CONFIG_LUNA_GPON, which
 *   both Luna boards set; this function is DEFINED in rtl9602c_eth.c, which
 *   compiles under CONFIG_RTL9602C_ETH -- and the G24W (interaptiv) sets
 *   `# CONFIG_RTL9602C_ETH is not set` because it has its own NIC. Declaring it
 *   unconditionally therefore linked on the X111W and failed on the G24W with
 *   `undefined reference to rtl9602c_eth_omci_set_optical`, MEASURED 2026-09-02.
 *
 * ★ THE STUB IS A NO-OP BECAUSE THERE IS NOTHING TO PUBLISH INTO: the OMCI ME
 *   store this writes lives in that same Ethernet driver, so a board that does
 *   not build it has no ANI-G instance to update -- not a value we are
 *   choosing to drop.
 *   ⇒ WHEN THAT BOARD GAINS ITS OWN OMCI RESPONDER, THIS STUB BECOMES A SILENT
 *     HOLE and must be replaced by a call into it. It is deliberately not a
 *     `weak` symbol: a link error is how this was found, and a stub that hides
 *     the next one would be worse than the bug.
 */
/* ★ THE HOLE THIS COMMENT PREDICTED IS CLOSED (2026-09-08).  luna_eth.c gained
 * the common OMCI responder, so CONFIG_LUNA_ETH now has an ANI-G instance to
 * publish into and defines this symbol too.  The inline no-op survives for a
 * config that builds NEITHER Ethernet shell -- and it is still deliberately not
 * a `weak` symbol, because a link error is how the first hole was found. */
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


/*
 * The ONU serial number, owned by the PLOAM layer that provisions it.
 * ★ WHY AN ACCESSOR AND NOT A SECOND COPY: the OMCI responder needs the same
 *   eight bytes to answer ME 256 (ONU-G), and this tree has already paid for
 *   two copies of one serial number -- the two decoders disagreed, one of them
 *   turning a bad hex digit into 0xff without a word. One owner, one reader.
 */
void gpon_onu_sn(u8 out[8]);

#endif /* _LUNA_GPON_NIC_H */
