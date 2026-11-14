# SPDX-License-Identifier: GPL-2.0-or-later
# The files every OLT target ships (an ONU target includes base-files.mk instead).
OLT_BASE_FILES := $(TOPDIR)/target/linux/gpon-common/olt-base-files
PKG_FILE_DEPENDS += $(OLT_BASE_FILES)/

define Package/base-files/install-target
	$(CP) $(OLT_BASE_FILES)/. $(1)/
endef
