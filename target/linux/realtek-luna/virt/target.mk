# SPDX-License-Identifier: GPL-2.0-only

ARCH:=mips
SUBTARGET:=virt
BOARD:=realtek-luna
BOARDNAME:=QEMU Malta (a virtual Luna ONU)
# The taroko CPU type and flags: both subtargets build into the one mips_mips32 package tree.
CPU_TYPE:=mips32
CFLAGS:=-Os -pipe -mno-branch-likely -mips32 -mtune=mips32 -fno-schedule-insns2 -mfix-rlx

define Target/Description
	The Luna images on QEMU's big-endian Malta board (24Kc): the same tree,
	gpon_* core and userspace as the boards, virtio for every port, the 8250
	console and the virtual PON in place of the SoC drivers. Boot it with
	dev/virt/virt_onu.py; it is a filter before the board, never a verdict.
endef
