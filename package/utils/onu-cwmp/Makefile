#
# A TR-069 (CWMP) CPE answering from a data model file (an ISP's own, else the unit's native TR-098 one). Off
# unless /etc/config/cwmp enables it: the OLT over OMCI (gpon-acs) or the WAN's DHCP option 43 does.
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
	$(INSTALL_DIR) $(1)/usr/sbin $(1)/usr/libexec $(1)/etc/init.d $(1)/etc/config $(1)/etc/udhcpc.user.d
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/onu-cwmp $(1)/usr/sbin/onu-cwmp
	$(INSTALL_BIN) ./files/onu-cwmp-firmware $(1)/usr/libexec/onu-cwmp-firmware
	$(INSTALL_DATA) ./files/onu-cwmp-acs.udhcpc $(1)/etc/udhcpc.user.d/onu-cwmp-acs
	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_DATA) ./files/onu-cwmp.uci-defaults $(1)/etc/uci-defaults/90-onu-cwmp-dhcp
	$(INSTALL_BIN) ./files/onu-cwmp.init $(1)/etc/init.d/onu-cwmp
	$(INSTALL_CONF) ./files/cwmp.config $(1)/etc/config/cwmp
endef

$(eval $(call BuildPackage,onu-cwmp))
