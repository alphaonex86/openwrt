# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the Realtek network device drivers.
#
#
# OpenWrt/realtek-luna -- WHOLE-FILE OVERLAY, migrated out of a patch.
#
# This is Linux 6.18.31's own file plus the realtek-luna GPON/ETH wiring
# that used to live in patches-6.18/103-realtek-luna-gpon.patch.  The patch
# and this file are mutually exclusive: files- overlays are copied into the
# kernel tree BEFORE patches are applied (include/quilt.mk:93-107), so with
# both present patch 103 applies to this file, its context no longer matches
# and the build FAILS LOUDLY.  That is the intended direction -- but it means
# patch 103 must be deleted in the SAME commit that adds this file.
#
# WHY the migration is a prerequisite and not a cleanup: the shared GPON
# core (target/linux/gpon-common) is reached by `select GPON_CORE` from the
# vendor Kconfig, and every later move changes luna's obj-y lines.  Both
# edits land in files that only existed inside a forbidden patch.
# Operator hard rule: driver wiring is a browsable source file, never a
# patches-<ver>/*.patch -- not even for Kconfig/Makefile.
#
# ★ NO LONGER byte-identical to what patch 103 produced, and this line says so
# rather than leaving a stale claim standing.  It WAS identical when it was
# migrated (verified 2026-08-05 against
# build_dir/target-mips_mips32_musl/.../linux-6.18.31); the ONE deliberate
# difference since is the include path added below.  The object list and every
# CFLAGS_ line are still untouched, so nothing that was compiled before is
# compiled differently now.
#
# THE INCLUDE PATH: this directory's sources include headers authored in the
# SHARED GPON tree, target/linux/gpon-common/files-6.18/drivers/net/gpon/, which
# reaches the build through FILES_DIR (see this target's Makefile).  Without the
# -I below, `#include "gpon_unsup.h"` (and "gpon_ploam.h", and the rest) simply
# does not resolve from here -- which is exactly what gpon-rtl960x.c:5097
# recorded as reason 3 for why the PLOAM rewire could not land.
#
# WHY AN -I AND NOT A RELATIVE INCLUDE, and why this spelling: the shared tree is
# a SEPARATE files- overlay merged into the kernel source where upstream's own
# directories land, so its path relative to THIS file is an artefact of the
# overlay mechanism, not a property of the source.  $(srctree) says what is
# actually meant -- "the in-tree GPON core" -- and keeps working if either
# overlay moves.  It is the SAME line realtek-elnath already carries
# (files-6.18/drivers/net/ethernet/cortina/Makefile:22), deliberately spelled
# identically so the two targets cannot drift.
#
# ccflags-y (whole directory) rather than one CFLAGS_<obj>.o per file: a
# per-object list is a list somebody forgets to extend, and the next source that
# includes a shared header would fail to compile with nothing here to see.  It
# adds an include path and no code: an object that includes nothing from the
# shared tree is byte-identical with and without it.
ccflags-y += -I$(srctree)/drivers/net/gpon
# The hoisted-logic headers (drivers/net/flowcore/*_logic.h): pure functions
# moved out of this directory's shells into the shared tree; the shells still
# call them, so their prototypes must resolve from here.
ccflags-y += -I$(srctree)/drivers/net/flowcore

# ★ THIS BOARD'S OMCI CAPACITIES. The core defaults (64 MEs / 72 MIB rows) are
# the Elnath's; the Luna shell has always carried 128/200 and rebasing it onto
# the common store at the smaller size would DROP provisioned MEs. The limit is
# a per-board value, the store's logic is not -- so it is set here, next to the
# board, and the common code stays one copy.
ccflags-y += -DOMCI_STORE_MAX=128 -DOMCI_MIB_ROWS_MAX=200

obj-$(CONFIG_RTL9602C_ETH) += rtl9602c_eth.o
obj-$(CONFIG_RTL9602C_GPON) += gpon-rtl960x.o rtl960x_ponmac.o
obj-$(CONFIG_RTL9607C_GPON) += rtl9607c_gpon.o rtl960x_ponmac.o
obj-$(CONFIG_RTL960X_ETH) += rtl960x_eth.o
# The datapath hot path keeps -O2 even under the size-optimized (-Os) kernel:
# the NIC RX/TX + GEM/PON-MAC forwarding must not lose per-packet performance.
CFLAGS_rtl9602c_eth.o += -O2
CFLAGS_gpon-rtl960x.o += -O2
CFLAGS_rtl960x_eth.o += -O2
CFLAGS_rtl9607c_gpon.o += -O2
CFLAGS_rtl960x_ponmac.o += -O2
obj-$(CONFIG_8139CP) += 8139cp.o
obj-$(CONFIG_8139TOO) += 8139too.o
obj-$(CONFIG_ATP) += atp.o
r8169-y += r8169_main.o r8169_firmware.o r8169_phy_config.o
r8169-$(CONFIG_R8169_LEDS) += r8169_leds.o
obj-$(CONFIG_R8169) += r8169.o
obj-$(CONFIG_RTASE) += rtase/
