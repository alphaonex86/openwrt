# SPDX-License-Identifier: GPL-2.0-only

ARCH:=aarch64
SUBTARGET:=virt
BOARD:=realtek-elnath
BOARDNAME:=QEMU virt (a virtual Elnath ONU)
KERNELNAME:=Image dtbs

define Target/Description
	The Elnath image on QEMU's arm64 `virt` machine (virt-10.2, Cortex-A55, GICv3):
	the same tree, core and userspace as the boards, with virtio for every port,
	a PL011 console and the virtual PON in place of the SoC drivers. Boot it
	with dev/virt/virt_onu.py; it is a filter before the board, never a verdict.
endef
