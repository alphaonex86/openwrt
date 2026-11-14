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

# The common trees: the GPON core's OLT role and its virtual PON MAC (the VM variant) come from gpon-common.
ifeq ($(TARGET_BUILD),1)
FILES_DIR += "$(TOPDIR)/target/linux/eth-common/files-$(KERNEL_PATCHVER)"
FILES_DIR += "$(TOPDIR)/target/linux/gpon-common/files-$(KERNEL_PATCHVER)"
# kernel-defaults.mk froze the copy command inside target.mk: re-expand it (why: realtek-luna/Makefile).
Kernel/Patch := $(Kernel/Patch/Default)
endif

DEFAULT_PACKAGES := $(filter-out urngd,$(DEFAULT_PACKAGES))

$(eval $(call BuildTarget))
