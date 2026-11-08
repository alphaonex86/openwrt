# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

define Device/Default
  PROFILES := Default
  KERNEL_NAME := Image
  IMAGES :=
endef

# The stock U-Boot boots the file VER/rcpucfg.ini names: a GNU tar of uImage, ramdisk.uImage and board.dtb, in
# that order. Our root fs is the initramfs inside the kernel, so the ramdisk is an empty cpio U-Boot still finds,
# uncompressed: the kernel carries no gzip decompressor for it.
define Build/zte-osimage
	rm -rf $@.osimage
	mkdir -p $@.osimage
	mv $@ $@.osimage/uImage
	$(CP) $(KDIR)/image-$(firstword $(DEVICE_DTS)).dtb $@.osimage/board.dtb
	$(LINUX_DIR)/usr/gen_init_cpio /dev/null > $@.osimage/ramdisk.cpio
	SOURCE_DATE_EPOCH="$(SOURCE_DATE_EPOCH)" mkimage -A $(LINUX_KARCH) -O linux -T ramdisk -C none \
		-n 'OpenWrt empty ramdisk' -d $@.osimage/ramdisk.cpio $@.osimage/ramdisk.uImage
	tar --format=gnu --numeric-owner --owner=0 --group=0 --mtime=@$(SOURCE_DATE_EPOCH) \
		-C $@.osimage -cf $@ uImage ramdisk.uImage board.dtb
	rm -rf $@.osimage
endef

include $(SUBTARGET).mk

$(eval $(call BuildImage))
