# SPDX-License-Identifier: GPL-2.0-only

ARCH:=arm
SUBTARGET:=cortexa53
BOARD:=zte-zx279221
BOARDNAME:=ZTE ZX279221 Cortex-A53
# The user space is 32-bit soft float like the stock's: the Cortex-A9 boards' packages run unchanged.
CPU_TYPE:=cortex-a9
KERNELNAME:=Image

define Target/Description
	Build firmware images for the ZTE ZX279221 (ZTE ZXA10 C610 main board).
endef
