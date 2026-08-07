#!/usr/bin/env python3
"""MIT 模式单电机点动：以当前位置为起点移动 --delta 度，结束自动泄力。

⚠️ 电机必须已切到 MIT 协议（tools/switch_protocol.py --to mit + 断电重启）。
⚠️ 本工具用 lerobot 的 RobstrideMotorsBus——要在 metal-lerobot 环境运行：
    ~/miniconda3/envs/metal-lerobot/bin/python tools/mit_move.py --id 127 --delta 15
"""

import argparse
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, required=True, help="电机 CAN ID")
    ap.add_argument("--delta", type=float, required=True, help="移动角度（度，负数反向）")
    ap.add_argument("--kp", type=float, default=15.0)
    ap.add_argument("--kd", type=float, default=0.8)
    ap.add_argument("--seconds", type=float, default=2.4, help="发令时长")
    ap.add_argument("--model", default="O0", help="O 系列型号名（默认 O0=RS00）")
    ap.add_argument("--channel", default="can0")
    a = ap.parse_args()

    try:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.robstride import RobstrideMotorsBus
    except ImportError:
        raise SystemExit("缺 lerobot——请用 metal-lerobot 环境跑：~/miniconda3/envs/metal-lerobot/bin/python tools/mit_move.py ...")

    m = Motor(a.id, a.model, MotorNormMode.DEGREES)
    m.recv_id = a.id
    m.motor_type_str = a.model
    bus = RobstrideMotorsBus(port=a.channel, motors={"m": m},
                             can_interface="socketcan", use_can_fd=False, bitrate=1_000_000)
    bus.connect()
    try:
        p0 = bus.sync_read("Present_Position")["m"]
        print(f"当前 {p0:.2f}°")
        bus.sync_write("Kp", {"m": a.kp})
        bus.sync_write("Kd", {"m": a.kd})
        bus.enable_torque()
        target = p0 + a.delta
        ticks = max(1, int(a.seconds / 0.02))
        for _ in range(ticks):
            bus.sync_write("Goal_Position", {"m": target})
            time.sleep(0.02)
        p1 = bus.sync_read("Present_Position")["m"]
        ok = abs(p1 - p0 - a.delta) < max(3.0, abs(a.delta) * 0.2)
        print(f"目标 {target:.2f}° -> 实到 {p1:.2f}°（误差 {p1 - target:+.2f}°）{' ✅' if ok else ' ⚠️ 未按预期移动'}")
    finally:
        try:
            bus.disable_torque()
        except Exception as e:
            print(f"泄力失败（注意手动断电）: {e}")
        bus.disconnect()


if __name__ == "__main__":
    main()
