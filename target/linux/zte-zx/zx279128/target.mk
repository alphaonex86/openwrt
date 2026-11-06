# SPDX-License-Identifier: GPL-2.0-only

ARCH:=arm
SUBTARGET:=zx279128
BOARD:=zte-zx
BOARDNAME:=ZTE ZX279128 Cortex-A9
# The stock userland and kernel use no FPU: soft float until the unit's VFP/NEON is measured.
CPU_TYPE:=cortex-a9
KERNELNAME:=zImage dtbs

define Target/Description
	Build firmware images for the ZTE ZX279128 GPON ONU SoC (ZTE ZXHN F680).
endef
