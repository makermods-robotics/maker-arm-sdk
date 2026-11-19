#!/usr/bin/env bash
# Bring up a serial SLCAN USB-CAN adapter (CANable-class) as a SocketCAN interface at 1 Mbit/s.
# Usage: sudo bash scripts/linux/setup_slcan.sh [serial_port=/dev/ttyACM0] [iface_name=can0]
# Re-run after unplugging the adapter. The SDK's default socketcan backend then uses it; candump works too.
set -e
DEV=${1:-/dev/ttyACM0}
IFACE=${2:-can0}

modprobe can
modprobe can_raw
modprobe slcan
systemctl stop ModemManager 2>/dev/null || true   # ModemManager grabs new serial ports
pkill -f "slcand.*${IFACE}" 2>/dev/null || true    # a stale instance from before a replug
sleep 0.2
slcand -o -c -s8 -t hw -S 921600 "${DEV}" "${IFACE}"   # -s8 = 1 Mbit/s CAN; hardware flow control, 921600 baud serial
sleep 0.2
ip link set up "${IFACE}"
ip link set "${IFACE}" txqueuelen 1000
ip -br link show "${IFACE}"
echo "OK: ${IFACE} @1Mbps (${DEV}). Check with: candump ${IFACE}"
