# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the rtl9607x subtarget.
# Bring-up is run-from-RAM: the initramfs uImage is TFTP'd into RAM and
# bootm'd by the vendor U-Boot ("9607C#"), no flash write during bring-up.

define Device/realtek_rtl9607c
  DEVICE_VENDOR := Realtek
  DEVICE_MODEL := RTL9607C
  DEVICE_DTS := rtl9607c_engboard
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9607c
  # M1 brings the SoC up headless to a serial console + initramfs shell; the
  # full router/GPON package set is added once the 9607C datapath drivers land.
endef
TARGET_DEVICES += realtek_rtl9607c

# LANLY G24W (RTL9603CVD). Same interAptiv MIPS32 R2 core as the RTL9607C
# above -- MEASURED from the board's own /proc/cpuinfo ("MIPS interAptiv
# V2.0", isa mips32r2, tlb_entries 64) -- so it belongs in THIS subtarget
# and shares its 24kc toolchain. It is emphatically NOT an rtl960x/RLX part
# despite the "9603" in the name; see rtl9603cvd.dtsi for that argument.
define Device/lanly_g24w
  DEVICE_VENDOR := LANLY
  DEVICE_MODEL := G24W
  DEVICE_DTS := rtl9603cvd_g24w
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9603cvd
  # Flash partition sizes MEASURED from the live device /proc/mtd:
  #   k0 = 0x334000 (3280k) kernel, r0 = 0xaa0000 (10880k) rootfs.
  # KERNEL_SIZE auto-fires check-size on the kernel (image.mk), so a kernel
  # that would not fit k0 fails the BUILD instead of being discovered on the
  # device. IMAGE_SIZE records the measured rootfs bound for the same reason.
  # NOTE these duplicate the DTS partition table by hand -- nothing in the
  # build checks the two agree, so they are changed together or not at all.
  KERNEL_SIZE := 3280k
  IMAGE_SIZE := 10880k
  # Device/Default leaves IMAGES empty, so this builds the initramfs uImage only
  # -- no flashable artifact is produced for a SINGLE-BANK board whose only k0
  # holds the stock ORACLE. Do not "fix" that.
  #
  # ★★★ THE PACKAGE SET IS DECLARED HERE, NOT IN A .config (2026-08-27).
  #
  # MEASURED: this device declared NO DEVICE_PACKAGES at all, so the built image
  # carried exactly ONE kmod (kmod-gpio-button-hotplug) against the X111W's 18,
  # and the 17 absent ones are the netfilter and PPP stacks -- i.e. NAT, the
  # firewall and PPPoE could not work here, each surfacing as its own
  # unrelated-looking test failure rather than as one cause.
  #
  # An earlier sitting reached for `.config` instead (dnsmasq, odhcpd-ipv6only,
  # wpad-basic-mbedtls, hostapd-common were selected there by hand). That works
  # for one build and leaves nothing behind: a `.config` is a session artifact,
  # while DEVICE_PACKAGES is the record every future build of this device reads.
  # The list is the X111W's own, minus the luci/gpon-provision half that depends
  # on drivers this board does not have yet, plus hostapd-common.
  #
  # ⚠ wpad-basic-mbedtls and hostapd-common ship the AP daemons; they do NOT
  # give this board a radio. There is no 802.11 PHY here at all -- CONFIG_PCI is
  # OFF in this subtarget's kernel config, so no PCI bus exists, no PCIe host
  # driver for this SoC exists, and no DT node declares one. That is three
  # stacked gaps, and none of them is a package.
  # (2026-08-27: all three closed -- CONFIG_PCI=y, pcie-rtl960x.c, phy0 up.)
  #
  # ★★★ wifi-scripts IS the AP bring-up, and it was MISSING (2026-08-27).
  # MEASURED on this board with phy0 registered and hostapd running: hostapd's
  # ucode glue died with `Unable to resolve path for module 'common'` (that
  # module is /usr/share/ucode/wifi/common.uc, shipped by wifi-scripts along
  # with /lib/netifd/wireless/mac80211.sh), netifd said `Wireless module not
  # found`, network.wireless never existed, and wlan0 stayed `type managed`,
  # DOWN, 0 dBm -- no beacon, wifi_ap_onair FAIL after the full 180 s wait.
  # The X111W's working AP rests on CONFIG_PACKAGE_wifi-scripts=y selected BY
  # HAND in its build tree's .config -- a session artifact this device list
  # (copied from the X111W's DECLARED list) never contained. Same trap as
  # firewall4 above: the .config also carried the explicit deselection lines,
  # which `make defconfig` preserves; they were removed with it.
  #
  # ★ wireless-regdb: same story -- =y by hand in the X111W's .config, absent
  # here; without it cfg80211 logs `regulatory.db is malformed or signature is
  # missing/invalid` and country BO cannot be applied (the suite's regdomain
  # rules read from the official regdb, never hand-written).
  #
  # ★ gpon-provision: ALREADY in this image via .config =y (another session
  # artifact) and RUNNING -- it is the mechanism that applies this unit's own
  # MAC from the factory config partition (rtk_factory reads ELAN_MAC_ADDR out
  # of the Realtek MIB container on this product). Declared here so the record
  # matches the image.
  DEVICE_PACKAGES := dnsmasq firewall4 odhcpd-ipv6only odhcp6c \
	ppp ppp-mod-pppoe wpad-basic-mbedtls hostapd-common \
	wifi-scripts wireless-regdb gpon-provision
endef
TARGET_DEVICES += lanly_g24w
