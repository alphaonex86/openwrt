# SPDX-License-Identifier: GPL-2.0-or-later
# A unit's MAC identity: six lowercase octets, unicast, and none of the
# addresses every unit of this family carries (Realtek's LAN and WLAN defaults).

ONU_MAC_PLACEHOLDERS="00:00:00:00:00:00 00:e0:4c:86:70:01 00:e0:4c:07:68:02"

unit_mac_valid() {
	case "$1" in
	??:??:??:??:??:??) ;;
	*) return 1 ;;
	esac
	case "$1" in
	?[13579bdf]:*) return 1 ;;
	esac
	case " $ONU_MAC_PLACEHOLDERS " in
	*" $1 "*) return 1 ;;
	esac
	return 0
}

# A `<module>.mac=` boot parameter (a flasher's per-unit pin), lowercased; the
# module is matched, not spelled: rtl960x_eth, rtl9602c_eth and luna_eth all exist.
cmdline_mac() {
	sed -n 's/.*[ \t][A-Za-z0-9_]\{1,\}\.mac=\([0-9A-Fa-f:]\{17\}\).*/\1/p' \
		"${ONU_CMDLINE:-/proc/cmdline}" 2>/dev/null | tr 'A-F' 'a-f'
}

# This unit's MAC in the operator's order (2026-10-01): its factory config
# partition, else a boot parameter a flasher pinned, else none -- nothing is
# invented, and never eth0's current address (without a pin, the driver's
# random). Sets UNIT_MAC and UNIT_MAC_SRC; returns 1 when no source has one.
unit_mac() {
	UNIT_MAC=$(${ONU_RTK_FACTORY:-/usr/sbin/rtk_factory} -p config mac 2>/dev/null | tr 'A-F' 'a-f')
	UNIT_MAC_SRC="this unit's factory config partition"
	unit_mac_valid "$UNIT_MAC" && return 0
	UNIT_MAC=$(cmdline_mac)
	UNIT_MAC_SRC="the mac= boot parameter"
	unit_mac_valid "$UNIT_MAC" && return 0
	UNIT_MAC= UNIT_MAC_SRC=
	return 1
}

# The software version the OLT reads (OMCI SW-image), operator 2026-09-30:
# OWRT-<short commit> of the build in /etc/build-id, "+" for a dirty tree.
owrt_version() {
	local id rest
	read -r id rest < "${ONU_BUILD_ID:-/etc/build-id}" || return 1
	case "$id" in
	*-dirty) id=${id%-dirty}; rest=+ ;;
	*) rest= ;;
	esac
	case "$id" in
	[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f]*) ;;
	*) return 1 ;;
	esac
	echo "OWRT-$(echo "$id" | cut -c1-7)$rest"
}
