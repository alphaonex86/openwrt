# SPDX-License-Identifier: GPL-2.0-only
# Shells, not core: each holds a device, a lock and a timer, so they are outside drivers/net/gpon/.
ccflags-y += -I$(srctree)/drivers/net/gpon
obj-$(CONFIG_PON_VIRT) += ponvirt.o
obj-$(CONFIG_PON_VIRT_OLT) += ponvirt_olt.o
