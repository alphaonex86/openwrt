#!/bin/sh
# Sync canonical open-maple/ sources into the OpenWRT package layout.
# Re-run whenever open-maple/ changes.
set -e
cd "$(dirname "$0")/../.."

SRC=open-maple
DST_KMOD=openwrt-src/package/kernel/maple/src
DST_BIN=openwrt-src/package/utils/maple-tools/src

# Kernel module sources
mkdir -p "$DST_KMOD"
for f in maple_pci.c maple_ring.c maple_fw.c maple_bal.c maple_onu.c maple_ni.c maple_gnl.c maple_epld.c \
         maple_pci.h maple_ring.h maple_fw.h maple_bal.h maple_bal_msg.h maple_onu.h maple_ni.h maple_gnl.h \
         maple_hw.h maple_regs.h maple_codec.h; do
    cp "$SRC/$f" "$DST_KMOD/"
done
cp "$SRC/maple_codec_gen_all.h" "$DST_KMOD/maple_codec_gen.h"

# Userspace tool sources
mkdir -p "$DST_BIN"
for f in maplectl.c maple_snmp.c maple_lib.c maple_lib.h; do
    cp "$SRC/$f" "$DST_BIN/"
done

echo "Synced open-maple/ -> package/kernel/maple/src + package/utils/maple-tools/src"
