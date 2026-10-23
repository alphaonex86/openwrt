#
# A TR-069 (CWMP) CPE answering from a data model file: what a unit presenting an ISP's own ONU tells
# that ISP's ACS. Off unless /etc/config/cwmp enables it.
# SPDX-License-Identifier: GPL-2.0-or-later
#
include $(TOPDIR)/rules.mk

PKG_NAME:=onu-cwmp
PKG_VERSION:=1
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=Confiared <brule.herman@confiared.com>

include $(INCLUDE_DIR)/package.mk

define Package/onu-cwmp
  SECTION:=net
  CATEGORY:=Network
  TITLE:=TR-069 CPE answering from a data model file
endef

define Package/onu-cwmp/conffiles
/etc/config/cwmp
/etc/cwmp/
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_LDFLAGS) -o $(PKG_BUILD_DIR)/onu-cwmp \
		$(PKG_BUILD_DIR)/cwmp.c $(PKG_BUILD_DIR)/cwmp_core.c $(PKG_BUILD_DIR)/md5.c
endef

define Package/onu-cwmp/install
	$(INSTALL_DIR) $(1)/usr/sbin $(1)/etc/init.d $(1)/etc/config
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/onu-cwmp $(1)/usr/sbin/onu-cwmp
	$(INSTALL_BIN) ./files/onu-cwmp.init $(1)/etc/init.d/onu-cwmp
	$(INSTALL_CONF) ./files/cwmp.config $(1)/etc/config/cwmp
endef

$(eval $(call BuildPackage,onu-cwmp))
