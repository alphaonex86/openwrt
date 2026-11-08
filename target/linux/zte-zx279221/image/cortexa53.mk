# SPDX-License-Identifier: GPL-2.0-only

# A uImage of the uncompressed arm64 Image, as stock. Stock loads its 4.19 Image (text_offset 0x80000) at
# 0x40080000; a 6.x Image has text_offset 0 and must sit on a 2 MiB boundary, which bootm does not move it to.
define Device/zte_c610
  DEVICE_VENDOR := ZTE
  DEVICE_MODEL := ZXA10 C610
  DEVICE_DTS := zx279221-zte-c610
  DEVICE_DTS_DIR := $(DTS_DIR)/zte
  SOC := zx279221
  KERNEL_LOADADDR := 0x40200000
  KERNEL := kernel-bin | uImage none | zte-osimage
  KERNEL_INITRAMFS := kernel-bin | uImage none | zte-osimage
endef
TARGET_DEVICES += zte_c610
