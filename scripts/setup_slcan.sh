#!/usr/bin/env bash
# 把 SLCAN USB-CAN 棒（CANable 类，与 metal 臂同款）挂成 SocketCAN 网卡。
# 用法: sudo bash scripts/setup_slcan.sh [串口=/dev/ttyACM0] [接口名=can0]
# 之后 SDK 全部工具用默认 socketcan 后端即可，candump 也可用。
set -e
DEV=${1:-/dev/ttyACM0}
IFACE=${2:-can0}

modprobe can
modprobe can_raw
modprobe slcan
systemctl stop ModemManager 2>/dev/null || true   # 防串口被抢占
pkill -f "slcand.*${IFACE}" 2>/dev/null || true    # 清掉旧实例（拔插后重挂）
sleep 0.2
slcand -o -c -s8 "${DEV}" "${IFACE}"               # -s8 = CAN 1 Mbps（RobStride 总线速率）
sleep 0.2
ip link set up "${IFACE}"
ip link set "${IFACE}" txqueuelen 1000
ip -br link show "${IFACE}"
echo "OK: ${IFACE} @1Mbps (${DEV})。验证: candump ${IFACE}；或 python tools/scan_bus.py --channel ${IFACE}"
