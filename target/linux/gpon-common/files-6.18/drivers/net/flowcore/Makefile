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
