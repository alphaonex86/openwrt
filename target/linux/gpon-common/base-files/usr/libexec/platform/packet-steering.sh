#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
# OpenWrt's steering, then the link-control RX queue (rx-1, onu_rx_control.h) of the ONU netdevs
# whose driver fills it back to no RPS map: DHCP, ARP, PPP and ND are handled on the NAPI CPU and
# never wait in a backlog that a flood on another CPU has filled.
/usr/libexec/network/packet-steering.uc "$@" || logger -t packet-steering "generic steering failed: $?"
for dev in gpon0 eth0; do
	q=/sys/class/net/$dev/queues/rx-1/rps_cpus
	[ -e "$q" ] || continue
	echo 0 > "$q" || logger -t packet-steering "cannot clear $q"
done

# Stock's IRQ placement, per board: network.globals.irq_cpu_<wifi|eth>, derived from the board's stock
# /proc/interrupts by netcfg_gen. A WiFi netdev then gets no RPS (stock never steers wlan), so its
# receive path stays on the one CPU that takes its interrupt. Absent options: nothing is touched.
pin_irq() { # irq cpu netdev
	echo "$(printf %x $((1 << $2)))" > "/proc/irq/$1/smp_affinity" ||
		logger -t packet-steering "cannot pin IRQ $1 ($3) to CPU$2"
}
wifi_cpu="$(uci -q get network.globals.irq_cpu_wifi)"
eth_cpu="$(uci -q get network.globals.irq_cpu_eth)"
[ -n "$wifi_cpu$eth_cpu" ] || exit 0
for n in /sys/class/net/*; do
	dev="${n##*/}"
	[ -e "$n/device" ] && [ -z "$(ls -d "$n"/lower_* 2>/dev/null)" ] || continue
	if [ -e "$n/phy80211" ]; then
		[ -n "$wifi_cpu" ] || continue
		pci="$(readlink -f "$n/device")"
		irqs="$(ls "$pci/msi_irqs" 2>/dev/null)"
		[ -n "$irqs" ] || irqs="$(cat "$pci/irq" 2>/dev/null)"
		[ -n "$irqs" ] || logger -t packet-steering "no IRQ found for $dev"
		for irq in $irqs; do pin_irq "$irq" "$wifi_cpu" "$dev"; done
		for q in "$n"/queues/rx-*/rps_cpus; do
			[ -e "$q" ] || continue
			echo 0 > "$q" || logger -t packet-steering "cannot clear $q"
		done
	elif [ -n "$eth_cpu" ]; then
		for irq in $(awk -v d="$dev" '$NF == d { sub(":", "", $1); print $1 }' /proc/interrupts); do
			pin_irq "$irq" "$eth_cpu" "$dev"
		done
	fi
done
