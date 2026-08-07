#!/usr/bin/env python3
"""MIT 模式单电机持续旋转（速度模式：kp=0、kd 作速度增益、直接给目标速度）。

为什么不用位置爬坡：MIT 位置编码范围 ±12.57 rad（±720°），爬坡最多两圈就饱和；
kp=0 + 目标速度则无限旋转。Ctrl-C 停止并自动泄力。

⚠️ 电机必须已在 MIT 协议；用 metal-lerobot 环境运行：
    ~/miniconda3/envs/metal-lerobot/bin/python tools/mit_spin.py --id 127 --speed 30
"""

import argparse
import time

MAX_SPEED_DEG_S = 720.0   # 安全上限（RS00 物理上限 ~1890°/s，台架用不到）


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, required=True, help="电机 CAN ID")
    ap.add_argument("--speed", type=float, default=30.0, help="目标速度 度/秒（负数反向）")
    ap.add_argument("--kd", type=float, default=1.0, help="速度增益（转不动加大，抖动减小）")
    ap.add_argument("--seconds", type=float, default=0.0, help="旋转时长；0 = 直到 Ctrl-C")
    ap.add_argument("--model", default="O0")
    ap.add_argument("--channel", default="can0")
    a = ap.parse_args()
    if abs(a.speed) > MAX_SPEED_DEG_S:
        raise SystemExit(f"--speed 超过安全上限 ±{MAX_SPEED_DEG_S} 度/秒")

    try:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.robstride import RobstrideMotorsBus
    except ImportError:
        raise SystemExit("缺 lerobot——用 metal-lerobot 环境跑：~/miniconda3/envs/metal-lerobot/bin/python tools/mit_spin.py ...")

    m = Motor(a.id, a.model, MotorNormMode.DEGREES)
    m.recv_id = a.id
    m.motor_type_str = a.model
    bus = RobstrideMotorsBus(port=a.channel, motors={"m": m},
                             can_interface="socketcan", use_can_fd=False, bitrate=1_000_000)
    bus.connect()
    try:
        bus.enable_torque()
        print(f"旋转中 {a.speed:+.0f} 度/秒（kd={a.kd}）。Ctrl-C 停止并泄力。")
        t0 = time.monotonic()
        last_print = 0.0
        while a.seconds <= 0 or time.monotonic() - t0 < a.seconds:
            # 速度模式帧：kp=0（位置项失效）、kd=速度增益、目标速度=speed
            bus._mit_control("m", 0.0, a.kd, 0.0, a.speed, 0.0)
            now = time.monotonic()
            if now - last_print >= 1.0:
                st = bus.sync_read("Present_Velocity")
                print(f"  实测速度 {st['m']:+7.1f} 度/秒")
                last_print = now
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        try:
            bus.disable_torque()
            print("已泄力")
        except Exception as e:
            print(f"泄力失败（注意手动断电）: {e}")
        bus.disconnect()


if __name__ == "__main__":
    main()
