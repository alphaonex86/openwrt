# A clean-room OpenWrt port for GPON ONUs

An open-source replacement firmware for fibre-to-the-home ONUs, on kernel **6.18**, running today on
three boards from two silicon families and two CPU architectures.

A semi-personal project by **Herman Brule**, who owns **Confiabits S.R.L.** — which is why the commits
are authored personally and the work is open. It is offered for collaboration; it is not a company
product line, and this page does not pretend otherwise.

The vendor firmware on these devices is a 3.18 or 5.10 kernel that will never be patched again. This
replaces it with a maintained one, and measures both so the claim can be checked rather than believed.

---

## What runs today

| board | SoC | arch | state |
|---|---|---|---|
| **HSGQ X111W** | RTL9602C | MIPS32 big-endian (RLX "Taroko") | GPON O5 + WAN, WiFi, LAN, fibre LOS→re-range recovery |
| **LANLY G24W** | RTL9603CVD | MIPS32 R2 big-endian (interAptiv) | bring-up |
| **HSGQ X400AXF** | RTL9607F | ARM64 (Cortina Access NE) | GPON O5 + WAN, WiFi, hardware L3 offload at line rate |

Full GPON: PLOAM activation O1→O5, OMCI/G.988 management, GEM ports and T-CONTs, DBA upstream bursts,
BOSA laser calibration and SerDes bring-up — all written from the silicon's own register behaviour, not
copied from a vendor SDK.

## How the code is organised — three tiers, and the split is enforced

    target/linux/gpon-common/files-6.18/drivers/net/gpon/     the CORE
    target/linux/realtek-luna/                                the LUNA family   (MIPS)
    target/linux/realtek-elnath/                              the CORTINA family (ARM64)

| tier | prefix | holds | builds on |
|---|---|---|---|
| **core** | `gpon_*` | the PROTOCOL: PLOAM state machine, OMCI/G.988 ME model, GEM, TC flow-offload lifecycle | MIPS-BE, ARM64-LE **and x86** |
| **family** | `luna_*` · `cortina-*` | the SHELL: registers, DMA, IRQ, PON-MAC, PHY | its own target |
| **chip** | `rtl9602c_*` `rtl9607f_*` | exactly one SoC — offsets and tables, never logic | one board |

**No core file touches a register** — no `readl`/`writel`/`ioremap`, on any of them. Inside the core the
line is drawn once more, and it is drawn by measurement rather than by intention:

| | files | may use |
|---|---|---|
| **strict** | `gpon_ploam` `gpon_omci_core` `gpon_omci_me` `gpon_omci_trace` `gpon_gem_us` `gpon_sn` `gpon_regseq` | nothing. No allocator, no lock, no sleep — and **no clock: time is an explicit input**. Compiles and fuzzes on x86 with no kernel behind it |
| **widened** | `gpon_flow` `gpon_flow_offload` | kernel infrastructure that cannot honestly be stubbed — `rhashtable`, `net_device`, `kzalloc`, the TC dissector — but **still no register** |

`gpon_flow.c` is in the second list because it was put in the first and **failed all three host passes** on
`<net/flow_offload.h>`. Stubbing the kernel's TC dissector would have meant fuzzing our own stub, which
proves nothing about the kernel's — so the file moved and the reason is written next to it. A file may not
be widened to make a failure go away; the gate says so and checks it.

That split is what lets the protocol run identically on big-endian MIPS, little-endian ARM64 and x86, and
it is enforced at build time, not by convention.

Measured, not asserted:

| tier | lines | share |
|---|---:|---:|
| core (shared by every board) | 2 594 | 8.9 % |
| family (shared within one silicon family) | 23 836 | 82.0 % |
| chip (one SoC only) | 2 624 | **9.0 %** |
| **reused across boards (core + family)** | **26 430** | **91.0 %** |

(29 054 lines total, excluding the upstream wireless drivers we did not write. Re-measure any time:)

    python3 ONU-test-case/code_share.py --verbose

A new board should cost a table, not a fork of the logic. 9 % chip-specific code is that rule holding.

## Building

    python3 dev/build-openwrt.py                                  # every declared board
    python3 dev/build-openwrt.py --board=RTL9602C/HSGQ/X111W      # one

**A green `make` is not the verdict.** The builder compares the SOURCE bytes against the STAGED bytes and
refuses the image if the change is not in it — `make` has exited 0 on an image that did not contain the
patch more than once here. It also proves declared witnesses (a symbol's shipped default, a compiled-in
string) are present in the actual kernel binary.

Subtargets are named after the **CPU core** they select a toolchain for — `taroko` (RLX/Lexra, carries the
MDU-erratum workaround) and `interaptiv` (MIPS32 R2) — because a part-number range is not what a toolchain
is chosen by. The RTL9603CVD is an interAptiv part despite its name; its own boot banner says so.

## How it is tested

Correctness is proven **off the hardware first**. The board confirms; it does not discover.

- **A host suite that needs no board** — the protocol core compiled on x86 under ASan/UBSan and driven by
  an adversarial OLT: LOS, fibre pull, Deactivate, ONU-ID churn, key switch, malformed frames. 70 test
  binaries, each runnable alone, each self-recording its result.
- **Differential tests against the shipping driver.** The real function is extracted from the driver
  source at build time and compared with an independent re-implementation, so the two cannot silently
  drift. When code moves, the extractor reports it and a guard fails the gate.
- **An offline gate of 304 jobs** — static and structural checks that need no device.
- **201 registered device cases** (198 automatic, 3 manual) run against a real ONU on a real OLT:
  Ethernet at every printed socket, WiFi both bands, DHCP / PPPoE / static WAN, tagged and untagged,
  NAT, IPv6 over a static prefix, hardware offload, watchdog, LED behaviour, and disconnect→reconnect
  for every port and the fibre.
- **Every guard is mutation-proven**: disable it and the gate must go red. A check that cannot fail is
  not a check.

## How the numbers are reported

Every board gets a dated artifact and a certificate comparing the vendor firmware against ours at the
same operating point. The honesty rules are in the tooling, not in a reviewer's discipline:

- two figures are compared only when both are real measurements at a **comparable operating point** —
  otherwise the row splits, publishes both, and refuses the delta in words;
- a bound (generator limit, cable limit) is never compared as a capability;
- `not measured` is never drawn as 0, and a blocked cell carries no number at all;
- a result carries the **version of the benchmark that produced it**, so comparability is a property of
  the method and not of the date.

## Clean-room

Register addresses, field semantics, initialisation order and exported symbol names are hardware
interface facts and are used and named directly. The vendor SDK's **source expression** is not: nothing
here is copied from it, and every re-expression is written as new idiomatic mainline-style C with its own
SPDX header. Firmware blobs the hardware itself executes (PHY SRAM patches, RF tables) are extracted from
the device's own image as data, never transcribed from a vendor header.

## Working with us

This port is developed in the open at `github.com/alphaonex86/openwrt`. If you build ONU hardware and
want it supported — or you have a board this should run on — open an issue there, or write to
**alpha_one_x86@first-world.info** — the same address that authors the commits here.
