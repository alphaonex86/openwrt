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
