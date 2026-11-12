# SPDX-License-Identifier: GPL-2.0-only
# A virtual unit per board: QEMU Malta boots the ELF kernel with the board's squashfs on virtio-blk; the small
# DTB holds only the board's /omci-uni, loaded above the kernel's RAM and read by ponvirt.panel_fdt.

define Device/virt-luna
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  KERNEL_NAME := vmlinux.elf
  KERNEL_INITRAMFS_NAME := vmlinux-initramfs.elf
  KERNEL := kernel-bin
  KERNEL_INITRAMFS := kernel-bin
  IMAGES := dtb kernel rootfs
  IMAGE/dtb := append-dtb
  IMAGE/kernel := append-kernel
  # a zero tail where fstools looks for rootfs_data: it finds no filesystem there and mounts a tmpfs overlay
  IMAGE/rootfs := append-rootfs | pad-to 64k | pad-extra 64k
endef

define Device/virt_x111w
  $(Device/virt-luna)
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X111W (virtual)
  DEVICE_DTS := virt_x111w
endef
TARGET_DEVICES += virt_x111w

define Device/virt_v2801rgw
  $(Device/virt-luna)
  DEVICE_VENDOR := VSOL
  DEVICE_MODEL := V2801RGW (virtual)
  DEVICE_DTS := virt_v2801rgw
endef
TARGET_DEVICES += virt_v2801rgw

define Device/virt_x100dg
  $(Device/virt-luna)
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X100DG (virtual)
  DEVICE_DTS := virt_x100dg
endef
TARGET_DEVICES += virt_x100dg

define Device/virt_g24w
  $(Device/virt-luna)
  DEVICE_VENDOR := LANLY
  DEVICE_MODEL := G24W (virtual)
  DEVICE_DTS := virt_g24w
endef
TARGET_DEVICES += virt_g24w
