# SPDX-License-Identifier: GPL-2.0-only

# The vendor U-Boot 2018.08 passes ATAGs, no device tree: a legacy uImage around a zImage with the DTB appended.
# It loads it with `nand read 0x62000000 0x600000 0x700000`, so the whole uImage must fit 7 MiB.
define Device/vsol_v1600g
  DEVICE_VENDOR := V-SOL
  DEVICE_MODEL := V1600G
  DEVICE_DTS := bcm56150-vsol-v1600g
  DEVICE_DTS_DIR := $(DTS_DIR)/broadcom
  SOC := bcm56150
  KERNEL_LOADADDR := 0x61008000
  KERNEL := kernel-bin | append-dtb | uImage none | check-size 7168k
  KERNEL_INITRAMFS := kernel-bin | append-dtb | uImage none | check-size 7168k
endef
TARGET_DEVICES += vsol_v1600g
