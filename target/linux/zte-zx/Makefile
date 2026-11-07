# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=arm
BOARD:=zte-zx
BOARDNAME:=ZTE ZX GPON ONU
DEVICE_TYPE:=basic
FEATURES:=ramdisk nand
SUBTARGETS:=zx279128

KERNEL_PATCHVER:=6.18

define Target/Description
	Build firmware images for ZTE ZX GPON ONU SoCs (ZX279128: dual ARM
	Cortex-A9 MPCore, PL310 L2, 128 MB DDR, Denali NAND). Bring-up is
	run-from-RAM: an initramfs uImage booted by the vendor U-Boot 2013.04.
endef

include $(INCLUDE_DIR)/target.mk

ifeq ($(TARGET_BUILD),1)
FILES_DIR += "$(TOPDIR)/target/linux/cortex-a9-common/files-$(KERNEL_PATCHVER)"
# kernel-defaults.mk froze the copy command inside target.mk: re-expand it (why: realtek-luna/Makefile).
Kernel/Patch := $(Kernel/Patch/Default)
endif

DEFAULT_PACKAGES := $(filter-out urngd,$(DEFAULT_PACKAGES))

$(eval $(call BuildTarget))
