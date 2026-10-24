# SPDX-License-Identifier: GPL-2.0-only
# A virtual unit per board: QEMU boots the raw Image (initramfs inside) with this board's virt DTB.

define Device/virt_x400axf
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X400AXF (virtual)
  DEVICE_DTS := virt_x400axf
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-elnath
  KERNEL := kernel-bin
  KERNEL_INITRAMFS := kernel-bin
  IMAGES := dtb
  IMAGE/dtb := append-dtb
  DEVICE_PACKAGES := dnsmasq firewall4 \
	luci-base luci-mod-admin-full luci-theme-bootstrap \
	uhttpd uhttpd-mod-ubus rpcd rpcd-mod-file \
	ppp ppp-mod-pppoe kmod-ppp kmod-pppox kmod-pppoe \
	odhcpd-ipv6only
endef
TARGET_DEVICES += virt_x400axf
