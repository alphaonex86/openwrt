# SPDX-License-Identifier: GPL-2.0-only
#
# The RTL9607C rev B GPHY EN70 calibration program, extracted from the UNKNOWN
# board's own stock kernel; luna_eth loads it with request_firmware().

include $(TOPDIR)/rules.mk

PKG_NAME:=rtl9607c-gphy-firmware
PKG_RELEASE:=1
PKG_LICENSE:=Proprietary

include $(INCLUDE_DIR)/package.mk

define Package/rtl9607c-gphy-firmware
  SECTION:=firmware
  CATEGORY:=Firmware
  TITLE:=RTL9607C rev B GPHY EN70 calibration program
endef

define Build/Compile
endef

define Package/rtl9607c-gphy-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware
	$(INSTALL_DATA) ./files/rtl9607c-revb-en70.fw $(1)/lib/firmware/
endef

$(eval $(call BuildPackage,rtl9607c-gphy-firmware))
