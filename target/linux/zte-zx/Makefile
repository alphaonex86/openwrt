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

DEFAULT_PACKAGES := $(filter-out urngd,$(DEFAULT_PACKAGES))

$(eval $(call BuildTarget))
