#
# SPDX-License-Identifier: GPL-2.0-or-later
#
include $(TOPDIR)/rules.mk

PKG_NAME:=memtester
PKG_VERSION:=4.7.1
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=https://pyropus.ca./software/memtester/old-versions/
PKG_HASH:=e427de663f7bd22d1ebee8af12506a852c010bd4fcbca1e0e6b02972d298b5bb

PKG_LICENSE:=GPL-2.0-only
PKG_LICENSE_FILES:=COPYING

include $(INCLUDE_DIR)/package.mk

define Package/memtester
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Userspace RAM test (a refurbished unit's RAM check)
  URL:=https://pyropus.ca./software/memtester/
endef

define Package/memtester/description
  memtester <size>[B|K|M|G] [passes]: locks the memory and runs the stuck-address,
  pattern and bit-flip tests over it; exit 0 = no error. Its upstream source, built as is.
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CPPFLAGS) $(TARGET_CFLAGS) -DPOSIX -D_POSIX_C_SOURCE=200809L \
		-D_FILE_OFFSET_BITS=64 -DTEST_NARROW_WRITES -Wno-parentheses $(TARGET_LDFLAGS) \
		-o $(PKG_BUILD_DIR)/memtester $(PKG_BUILD_DIR)/memtester.c $(PKG_BUILD_DIR)/tests.c
endef

define Package/memtester/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/memtester $(1)/usr/sbin/memtester
endef

$(eval $(call BuildPackage,memtester))
