# SPDX-License-Identifier: GPL-2.0-only
#
# drivers/net/flowcore -- the hardware-decoupled half of L3/L4 flow offload.
#
# A SIBLING OF drivers/net/gpon, NOT part of it: that directory is the GPON
# PROTOCOL core, and flow offload is not GPON. Authored once in
# target/linux/gpon-common/files-6.18/ and reaching both targets through the
# FILES_DIR they already carry, so a second common layer costs one obj- line in
# drivers/net/Makefile and no change to any target Makefile.
obj-y += flowcore_hash.o

# ── hoisted logic, one object per shell it left (bulk_hoist.py) ──────────
# Each object is gated on the SAME config symbol as the shell it was hoisted
# from, so a target never compiles (or links) logic for a driver it does not
# build -- the lean-kernel-per-model rule.  The functions kept here passed the
# hoist rule for real: no MMIO, no kernel service, no shell file-scope state.
# Everything the mechanical pass moved that failed that rule (I2C/MMIO helpers,
# net/debugfs/param callbacks, MIPS arch glue) was moved back into its shell.
#
# cg_sn_parse wraps the common core's gpon_sn_parse(), hence the include path
# (same spelling as both shells' own -I for the shared GPON core).
ccflags-y += -I$(srctree)/drivers/net/gpon
obj-$(CONFIG_CORTINA_GPON) += cortina_gpon_logic.o
obj-$(CONFIG_CORTINA_NI) += cortina_ni_rx_logic.o cortina_ni_tx_logic.o
obj-$(CONFIG_CORTINA_NI_FLOWOFFLOAD) += cortina_ni_flowoffload_logic.o \
	cortina_l3fe_logic.o
obj-$(CONFIG_RTL9602C_GPON) += gpon_rtl9602c_logic.o luna_ponmac_logic.o
obj-$(CONFIG_RTL9607C_GPON) += luna_ponmac_logic.o
obj-$(CONFIG_RTL9602C_ETH) += rtl9602c_l34_logic.o
