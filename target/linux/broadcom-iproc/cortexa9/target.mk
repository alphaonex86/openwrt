# SPDX-License-Identifier: GPL-2.0-only

ARCH:=arm
SUBTARGET:=cortexa9
BOARD:=broadcom-iproc
BOARDNAME:=Broadcom iProc Cortex-A9
# Soft float as the other Cortex-A9 target (zte-zx): one toolchain until the unit's VFP use is measured.
CPU_TYPE:=cortex-a9
KERNELNAME:=zImage dtbs

define Target/Description
	Build firmware images for the Broadcom iProc Cortex-A9 OLT hosts.
endef
