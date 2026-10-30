# SPDX-License-Identifier: GPL-2.0-only
# One behaviour on every ONU (operator, 2026-10-18): the new image replaces the running one in the
# same k0 and r0, the configuration kept. The build already refuses an image too big for them.

REQUIRE_IMAGE_METADATA=1
RAMFS_COPY_BIN="ubirename sha256sum head"

# A UBI-bank ONU (RTL9607C UNKNOWN): U-Boot boots bank 1 from the volume ubi_k1, the native firmware
# stays in bank 0. The image goes to the spare slot ubi_k1_b, is read back, and the two names swap in
# one atomic UBI rename; the next boot is a trial (sw_tryactive=1, sw_commit=0) that falls back to
# bank 0 unless onu-bank-commit sees SSH. A power cut at any step leaves a bootable bank.
onu_ubi_sys() {
	local f

	f=$(grep -lsx "$1" /sys/class/ubi/ubi0_*/name) && echo "${f%/name}"
}

onu_ubi_vol() {
	local d

	d=$(onu_ubi_sys "$1") && echo "/dev/${d##*/}"
}

onu_ubi_bytes() {
	cat "$(onu_ubi_sys "$1")/data_bytes" 2>/dev/null
}

onu_ubi_check_image() {
	local img="$1" size

	[ "$(dd if="$img" bs=4 count=1 2>/dev/null | hexdump -v -e '1/1 "%02x"')" = 27051956 ] &&
		dd if="$img" bs=1 skip=32 count=32 2>/dev/null | grep -qs OpenWrt || {
		echo "The image is not an OpenWrt uImage."
		return 1
	}
	size=$(wc -c < "$img")
	[ "$size" -le "$(onu_ubi_bytes ubi_k1)" ] || {
		echo "The image ($size bytes) is larger than ubi_k1."
		return 1
	}
}

onu_ubi_upgrade() {
	local img="$1" slot size want

	if [ -z "$(onu_ubi_vol ubi_k1_b)" ]; then
		slot=$(onu_ubi_vol ubi_r1) &&
			ubirsvol -n "${slot##*_}" -s "$(onu_ubi_bytes ubi_k1)" /dev/ubi0 &&
			ubirename /dev/ubi0 ubi_r1 ubi_k1_b || {
			echo "Could not turn ubi_r1 into the spare slot ubi_k1_b."
			return 1
		}
	fi
	slot=$(onu_ubi_vol ubi_k1_b)
	size=$(wc -c < "$img")
	want=$(sha256sum < "$img" | cut -d' ' -f1)
	ubiupdatevol "$slot" "$img" || return 1
	[ "$(head -c "$size" "$slot" | sha256sum | cut -d' ' -f1)" = "$want" ] || {
		echo "ubi_k1_b does not read back as written: ubi_k1 kept."
		return 1
	}
	mkdir -p /var/lock && printf 'sw_commit 0\nsw_tryactive 1\n' > /tmp/onu-bank.env &&
		fw_setenv -s /tmp/onu-bank.env || {
		echo "Could not arm the trial boot: ubi_k1 kept."
		return 1
	}
	ubirename /dev/ubi0 ubi_k1 ubi_k1_b ubi_k1_b ubi_k1
}

onu_member() {
	tar tf "$1" 2>/dev/null | grep -m 1 "^sysupgrade-[^/]*/$2\$"
}

onu_mtd() {
	local d

	for d in /sys/class/mtd/mtd[0-9]*; do
		[ "$(cat "$d/name" 2>/dev/null)" = "$1" ] && { echo "$d"; return 0; }
	done
	return 1
}

platform_check_image() {
	local img="$1" kernel root part dev size

	[ -n "$(onu_ubi_vol ubi_k1)" ] && {
		onu_ubi_check_image "$img"
		return
	}
	grep -qs ' /rom squashfs ' /proc/mounts || {
		echo "This ONU runs from RAM, not from its flash: the flash is not ours to replace."
		return 1
	}
	kernel=$(onu_member "$img" kernel)
	root=$(onu_member "$img" root)
	[ -n "$kernel" ] && [ -n "$root" ] || {
		echo "Not an ONU firmware image: no sysupgrade-<board>/kernel and root."
		return 1
	}
	[ "$(tar xf "$img" "$kernel" -O | dd bs=4 count=1 2>/dev/null | hexdump -v -e '1/1 "%02x"')" = 27051956 ] &&
		tar xf "$img" "$kernel" -O | dd bs=1 skip=32 count=32 2>/dev/null | grep -qs OpenWrt || {
		echo "The kernel is not an OpenWrt uImage."
		return 1
	}
	for part in k0:kernel r0:root; do
		dev=$(onu_mtd "${part%%:*}") || {
			echo "This ONU's flash has no ${part%%:*} partition."
			return 1
		}
		[ $(( $(cat "$dev/flags") & 0x400 )) -ne 0 ] || {
			echo "${part%%:*} is read-only on this ONU."
			return 1
		}
		size=$(tar xf "$img" "$(onu_member "$img" "${part#*:}")" -O | wc -c)
		[ "$size" -le "$(cat "$dev/size")" ] || {
			echo "The image's ${part#*:} ($size bytes) is larger than ${part%%:*}."
			return 1
		}
	done
	return 0
}

platform_do_upgrade() {
	local kernel root

	[ -n "$(onu_ubi_vol ubi_k1)" ] && {
		onu_ubi_upgrade "$1"
		return
	}
	kernel=$(onu_member "$1" kernel)
	root=$(onu_member "$1" root)
	sync
	tar xf "$1" "$kernel" -O | mtd write - k0 || return 1
	tar xf "$1" "$root" -O | mtd ${UPGRADE_BACKUP:+-j "$UPGRADE_BACKUP"} write - r0
}
