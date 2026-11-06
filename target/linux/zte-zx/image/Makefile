# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

define Device/Default
  PROFILES := Default
  KERNEL_NAME := zImage
  KERNEL_LOADADDR := 0x40008000
  IMAGES :=
endef

include $(SUBTARGET).mk

$(eval $(call BuildImage))
