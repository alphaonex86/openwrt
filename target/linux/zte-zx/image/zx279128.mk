# SPDX-License-Identifier: GPL-2.0-only

# The vendor U-Boot 2013.04 bootm's a legacy uImage loaded at 0x40008000 (as its own 0uImage): an uncompressed
# wrapper around a zImage with the device tree appended, the kernel decompressing itself.
define Device/zte_f680
  DEVICE_VENDOR := ZTE
  DEVICE_MODEL := ZXHN F680
  DEVICE_DTS := zx279128-f680
  DEVICE_DTS_DIR := $(DTS_DIR)/zte
  SOC := zx279128
  KERNEL := kernel-bin | append-dtb | uImage none
  KERNEL_INITRAMFS := kernel-bin | append-dtb | uImage none
endef
TARGET_DEVICES += zte_f680
