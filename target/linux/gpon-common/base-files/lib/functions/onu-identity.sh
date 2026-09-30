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
