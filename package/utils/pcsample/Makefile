#
# Clean-room OpenWrt package: a kernel+module CPU profile without perf userspace.
# SPDX-License-Identifier: GPL-2.0-or-later
#
include $(TOPDIR)/rules.mk

PKG_NAME:=pcsample
PKG_VERSION:=1
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=Confiared <brule.herman@confiared.com>

include $(INCLUDE_DIR)/package.mk

define Package/pcsample
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Sample where the CPUs spend their time, kernel modules included
  DEPENDS:=@KERNEL_PERF_EVENTS
endef

define Package/pcsample/description
  `pcsample <seconds> [min_percent]` samples the cpu-clock software event on
  every online CPU and prints each kallsyms symbol's share, busiest first.
  Needs only the kernel's perf events: no perf binary, no libelf.
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_LDFLAGS) -Wall -Wextra -Werror \
		-o $(PKG_BUILD_DIR)/pcsample $(PKG_BUILD_DIR)/pcsample.c
endef

define Package/pcsample/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/pcsample $(1)/usr/sbin/pcsample
endef

$(eval $(call BuildPackage,pcsample))
