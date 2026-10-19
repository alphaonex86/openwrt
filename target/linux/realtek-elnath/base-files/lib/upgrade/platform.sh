# SPDX-License-Identifier: GPL-2.0-only
# One behaviour on every ONU (operator, 2026-10-18): the new image replaces the running one in the
# same bank, the configuration kept. A bank is the vendor's pair of UBI volumes -- ubi_k0/ubi_r0 booted
# with root=31:8, ubi_k1/ubi_r1 with root=31:11 (gluebi) -- the kernel a FIT its U-Boot reads 6 MiB
# of. They are updated in place, never removed and recreated, so their size and volume number -- what
# that U-Boot and that root= rely on -- stay.

REQUIRE_IMAGE_METADATA=1
CI_UBIPART=ubi_device

onu_member() {
	tar tf "$1" 2>/dev/null | grep -m 1 "^sysupgrade-[^/]*/$2\$"
}

onu_volume() {
	nand_find_volume "$(nand_find_ubi "$CI_UBIPART")" "$1"
}

onu_bank() {
	case " $(cat /proc/cmdline) " in
	*" root=31:8 "*) echo 0 ;;
	*" root=31:11 "*) echo 1 ;;
	esac
}

platform_check_image() {
	local img="$1" kernel root part vol size bank

	grep -qs ' /rom squashfs ' /proc/mounts || {
		echo "This ONU runs from RAM, not from its flash: the flash is not ours to replace."
		return 1
	}
	bank=$(onu_bank)
	[ -n "$bank" ] || {
		echo "This ONU did not boot from either of its banks (no root=31:8 or 31:11)."
		return 1
	}
	kernel=$(onu_member "$img" kernel)
	root=$(onu_member "$img" root)
	[ -n "$kernel" ] && [ -n "$root" ] || {
		echo "Not an ONU firmware image: no sysupgrade-<board>/kernel and root."
		return 1
	}
	[ "$(tar xf "$img" "$kernel" -O | dd bs=4 count=1 2>/dev/null | hexdump -v -e '1/1 "%02x"')" = d00dfeed ] || {
		echo "The kernel is not a FIT image."
		return 1
	}
	for part in ubi_k$bank:kernel ubi_r$bank:root; do
		vol=$(onu_volume "${part%%:*}")
		[ -n "$vol" ] || {
			echo "This ONU's UBI has no ${part%%:*} volume."
			return 1
		}
		size=$(tar xf "$img" "$(onu_member "$img" "${part#*:}")" -O | wc -c)
		[ "$size" -le "$(cat "/sys/class/ubi/$vol/data_bytes")" ] || {
			echo "The image's ${part#*:} ($size bytes) is larger than ${part%%:*}."
			return 1
		}
	done
	[ -n "$(onu_volume rootfs_data)" ] || {
		echo "This ONU's UBI has no rootfs_data volume: the configuration could not be kept."
		return 1
	}
	return 0
}

platform_do_upgrade() {
	local member vol bank=$(onu_bank)

	sync
	for member in ubi_k$bank:kernel ubi_r$bank:root; do
		vol=$(onu_volume "${member%%:*}")
		tar xf "$1" "$(onu_member "$1" "${member#*:}")" -O |
			ubiupdatevol "/dev/$vol" -s "$(tar xf "$1" "$(onu_member "$1" "${member#*:}")" -O | wc -c)" - || return 1
	done
	ubiupdatevol "/dev/$(onu_volume rootfs_data)" -t || return 1
	[ -z "$UPGRADE_BACKUP" ] || nand_restore_config "$UPGRADE_BACKUP"
}
