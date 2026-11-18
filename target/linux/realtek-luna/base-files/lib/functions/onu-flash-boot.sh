# SPDX-License-Identifier: GPL-2.0-or-later
# Is this boot's root our own flash bank? A RAM boot maps its squashfs from RAM (phram, an mtd of type
# "ram"), and writes nothing to a flash whose bank it did not boot from (operator, 2026-11-17).

onu_flash_boot() {
	local dev type

	dev=$(awk '$5 == "/rom" || ($5 == "/" && / - squashfs /) { d = $3 } END { print d }' "$ONU_ROOT/proc/self/mountinfo")
	[ "${dev%%:*}" = 31 ] || return 1
	type=$(cat "$ONU_ROOT/sys/class/mtd/mtd${dev#*:}/type" 2>/dev/null)
	[ "$type" = nor ] || [ "$type" = nand ]
}
