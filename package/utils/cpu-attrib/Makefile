#
# Clean-room OpenWrt package: where a busy CPU goes, recorded by the unit itself.
# SPDX-License-Identifier: GPL-2.0-or-later
#
include $(TOPDIR)/rules.mk

PKG_NAME:=cpu-attrib
PKG_VERSION:=1
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=Confiared <brule.herman@confiared.com>

include $(INCLUDE_DIR)/package.mk

define Package/cpu-attrib
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Record where a busy CPU goes, on the unit itself
endef

define Package/cpu-attrib/description
  When the CPU stays busy, one report (CPU split, top tasks with their workqueue, softirqs, IRQs,
  workqueue items per queue, netdev rates, frames of a VLAN no netdev claims, conntrack inserts,
  pcsample when present) goes to a RAM ring under /tmp/cpu-attrib and its summary to the kernel
  log, so a unit too busy for a login keeps the evidence. `cpu-attrib snap` reports on demand.
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_LDFLAGS) -Wall -Wextra -Werror \
		-o $(PKG_BUILD_DIR)/cpu-attrib $(PKG_BUILD_DIR)/cpu-attrib.c
endef

define Package/cpu-attrib/install
	$(INSTALL_DIR) $(1)/usr/sbin $(1)/etc/init.d
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/cpu-attrib $(1)/usr/sbin/cpu-attrib
	$(INSTALL_BIN) ./files/cpu-attrib.init $(1)/etc/init.d/cpu-attrib
endef

$(eval $(call BuildPackage,cpu-attrib))
