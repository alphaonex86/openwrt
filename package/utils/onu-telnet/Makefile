#
# Clean-room OpenWrt package: the ONU LAN-only telnet management service.
# SPDX-License-Identifier: GPL-2.0-or-later
#
include $(TOPDIR)/rules.mk

PKG_NAME:=onu-telnet
PKG_VERSION:=1
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=Confiared <brule.herman@confiared.com>

include $(INCLUDE_DIR)/package.mk

define Package/onu-telnet
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=LAN-only telnet management for an ONU (busybox telnetd)
  DEPENDS:=+busybox
  PKGARCH:=all
endef

define Package/onu-telnet/description
  Busybox telnetd bound to the LAN address only, supervised by procd and
  respawned forever, so a technician on the LAN can log in and run onu-diag;
  it opens no socket on the WAN side.
endef

define Build/Compile
endef

define Package/onu-telnet/install
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/telnet.init $(1)/etc/init.d/telnet
endef

$(eval $(call BuildPackage,onu-telnet))
