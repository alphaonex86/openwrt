# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=arm
BOARD:=zte-zx279221
BOARDNAME:=ZTE ZX279221 GPON OLT main board
DEVICE_TYPE:=basic
FEATURES:=ramdisk
SUBTARGETS:=cortexa53

KERNEL_PATCHVER:=6.18

define Target/Description
	Build firmware images for the ZTE ZX279221 (4x Cortex-A53 without
	FP/NEON), the main board of the ZTE ZXA10 C610 OLT. As its stock: an
	arm64 kernel (CONFIG_ARM64_KERNEL_CROSS) running a 32-bit soft-float
	user space. Bring-up is run-from-RAM: the vendor U-Boot boots an OS
	image tar of uImage, ramdisk.uImage and board.dtb.
endef

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES := $(filter-out urngd,$(DEFAULT_PACKAGES))

$(eval $(call BuildTarget))
