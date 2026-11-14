# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=arm
BOARD:=broadcom-iproc
BOARDNAME:=Broadcom iProc GPON OLT
DEVICE_TYPE:=basic
FEATURES:=ramdisk nand
SUBTARGETS:=cortexa9

KERNEL_PATCHVER:=6.18

define Target/Description
	Build firmware images for GPON OLTs whose host is a Broadcom iProc
	switch SoC (Hurricane2 BCM56150: V-SOL V1600G; Saber2 BCM56260:
	HSGQ G008). Bring-up is run-from-RAM: an initramfs uImage booted by
	the vendor U-Boot.
endef

include $(INCLUDE_DIR)/target.mk

ifeq ($(TARGET_BUILD),1)
FILES_DIR += "$(TOPDIR)/target/linux/cortex-a9-common/files-$(KERNEL_PATCHVER)"
# The GPON core (gpon_regseq, the OLT role) the ZX279200 PON MAC driver builds on.
FILES_DIR += "$(TOPDIR)/target/linux/gpon-common/files-$(KERNEL_PATCHVER)"
# kernel-defaults.mk froze the copy command inside target.mk: re-expand it (why: realtek-luna/Makefile).
Kernel/Patch := $(Kernel/Patch/Default)
endif

DEFAULT_PACKAGES := $(filter-out urngd,$(DEFAULT_PACKAGES))

$(eval $(call BuildTarget))
