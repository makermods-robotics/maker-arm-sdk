#!/usr/bin/env bash
# Bring up an SLCAN USB-CAN dongle (CANable-class, same as the metal arm uses) as a SocketCAN interface.
# Usage: sudo bash scripts/setup_slcan.sh [serial_port=/dev/ttyACM0] [iface_name=can0]
# After this, all SDK tools can just use the default socketcan backend; candump also works.
set -e
DEV=${1:-/dev/ttyACM0}
IFACE=${2:-can0}

modprobe can
modprobe can_raw
modprobe slcan
systemctl stop ModemManager 2>/dev/null || true   # prevent the serial port from being grabbed
pkill -f "slcand.*${IFACE}" 2>/dev/null || true    # clear any stale instance (re-run after unplug/replug)
sleep 0.2
slcand -o -c -s8 -t hw -S 921600 "${DEV}" "${IFACE}"   # -s8 = 1 Mbps; -t hw -S 921600 = metal-arm proven serial settings (lower latency under 60Hz x 7-motor load)
sleep 0.2
ip link set up "${IFACE}"
ip link set "${IFACE}" txqueuelen 1000
ip -br link show "${IFACE}"
echo "OK: ${IFACE} @1Mbps (${DEV}). Verify with: candump ${IFACE}; or python tools/scan_bus.py --channel ${IFACE}"
