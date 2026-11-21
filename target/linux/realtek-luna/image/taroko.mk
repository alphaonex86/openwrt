# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the `taroko` subtarget (RLX/Lexra "Taroko"
# core; renamed from rtl960x 2026-08-29 -- the subtarget selects a CORE).
# Boards are brought up run-from-RAM first: the initramfs uImage is TFTP'd
# into RAM and bootm'd by the vendor U-Boot (no flash write during bring-up).

define Device/hsgq_x111w
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X111W
  DEVICE_DTS := rtl9602c_x111w
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9602c
  # NOR persistent image (raw MTD; the vendor fwu.sh flashes a uImage -> k0 and a
  # squashfs -> r0). Partition sizes from the live device /proc/mtd:
  #   k0 = 0x300000 (3072k) kernel; r0 = 0x4a0000 (4736k) rootfs, then the bank's last 128k the crashlog.
  # SIZE GUARD: KERNEL_SIZE auto-fires check-size on the kernel (image.mk), and the
  # explicit check-size on kernel.bin/rootfs.bin makes the BUILD fail if either
  # overflows its partition -- an oversize image is never written to the device.
  KERNEL_SIZE := $(call flash_kernel_size,3072k)
  IMAGE_SIZE := 4736k
  IMAGES := kernel.bin rootfs.bin sysupgrade.bin
  IMAGE/kernel.bin := append-kernel | check-size $$(KERNEL_SIZE)
  IMAGE/rootfs.bin := append-rootfs | pad-rootfs | check-size
  IMAGE/sysupgrade.bin := append-rootfs | pad-rootfs | check-size | sysupgrade-tar rootfs=$$$$@ | append-metadata
  # luci-app-gpon builds (validated) but its luci-base dep needs cgi-io from the
  # 'packages' feed; add "luci-app-gpon luci" here once that feed is installed.
  # Router stack so the ONU actually routes/NATs LAN<->WAN and serves DHCP:
  #   dnsmasq           - LAN DHCPv4 + DNS server
  #   firewall4(+nft)   - lan/wan zones, lan->wan forward + masquerade (NAT)
  #   odhcpd-ipv6only   - LAN IPv6 RA/DHCPv6 server (distributes the WAN PD)
  #   odhcp6c           - WAN DHCPv6 client (PD request toward the OLT)
  #   ppp ppp-mod-pppoe - PPPoE WAN client (alternative WAN proto to IPoE/DHCP)
  #   hostapd-basic-mbedtls - WiFi AP daemon for the rtl8192fe AP: WPA2-PSK(psk2)
  #     + WPA3-SAE personal + PMF, mbedtls TLS (libmbedtls is already pulled by
  #     libustream-mbedtls). AP only: no board runs WiFi client mode, so no
  #     wpa_supplicant runs beside it (wpad cost one more daemon, 0.6 MB, on the
  #     X111W). Listed here so 'make defconfig' keeps it tied to the device
  #     profile (a bare .config regen would otherwise drop a manual selection).
  # (urngd is excluded target-wide in target/linux/realtek-luna/Makefile: it
  #  burned ~60s of boot CPU on this core for nothing - crng seeds itself.)
  DEVICE_PACKAGES := gpon-provision luci-app-gpon luci-base luci-mod-admin-full luci-theme-bootstrap uhttpd uhttpd-mod-ubus rpcd rpcd-mod-file \
	dnsmasq firewall4 odhcpd-ipv6only odhcp6c ppp ppp-mod-pppoe hostapd-basic-mbedtls
endef
TARGET_DEVICES += hsgq_x111w

define Device/vsol_v2801rgw
  DEVICE_VENDOR := VSOL
  DEVICE_MODEL := V2801RGW
  DEVICE_DTS := rtl9602c_v2801rgw
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9602c
  # One bank on an 8 MiB Winbond EF4017, from the unit's own /proc/mtd and U-Boot env:
  #   k0 = 0x2a0000 (2688k) kernel at 0xa0000, r0 = 0x4a0000 (4736k) rootfs at 0x340000,
  #   the 128k crashlog at 0x7e0000 (operator 2026-10-18: a crashlog on every model).
  # check-size fails the BUILD when either overflows: an oversize image never reaches a unit.
  KERNEL_SIZE := $(call flash_kernel_size,2688k)
  IMAGE_SIZE := 4736k
  IMAGES := kernel.bin rootfs.bin sysupgrade.bin
  IMAGE/kernel.bin := append-kernel | check-size $$(KERNEL_SIZE)
  IMAGE/rootfs.bin := append-rootfs | pad-rootfs | check-size
  IMAGE/sysupgrade.bin := append-rootfs | pad-rootfs | check-size | sysupgrade-tar rootfs=$$$$@ | append-metadata
  # The same router stack as the X111W (see its profile for the why of each package).
  DEVICE_PACKAGES := gpon-provision luci-app-gpon luci-base luci-mod-admin-full luci-theme-bootstrap uhttpd uhttpd-mod-ubus rpcd rpcd-mod-file \
	dnsmasq firewall4 odhcpd-ipv6only odhcp6c ppp ppp-mod-pppoe hostapd-basic-mbedtls
endef
TARGET_DEVICES += vsol_v2801rgw

define Device/hsgq_x100dg
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X100DG
  DEVICE_DTS := rtl9601d_x100dg
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9601d
  # 32 MB of RAM cannot unpack a 10 MB initramfs: the dev loop boots the squashfs pair from RAM
  # through phram. The same pair is its flash image: one bank over both of the vendor's 8 MiB,
  #   k0 = 0x230000 (2240k) at 0x80000, r0 = 0x530000 (5312k) at 0x2b0000, the 128k crashlog at 0x7e0000.
  KERNEL_SIZE := $(call flash_kernel_size,2240k)
  IMAGE_SIZE := 5312k
  IMAGES := kernel.bin rootfs.bin sysupgrade.bin
  IMAGE/kernel.bin := append-kernel | check-size $$(KERNEL_SIZE)
  IMAGE/rootfs.bin := append-rootfs | check-size
  IMAGE/sysupgrade.bin := append-rootfs | pad-rootfs | check-size | sysupgrade-tar rootfs=$$$$@ | append-metadata
  # gpon-provision: its SN, MAC and RTL8290B laser cal come from its own config partition.
  DEVICE_PACKAGES := -iw gpon-provision
endef
TARGET_DEVICES += hsgq_x100dg
