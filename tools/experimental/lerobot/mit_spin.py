#!/usr/bin/env python3
"""MIT-mode single-motor continuous rotation (velocity mode: kp=0, kd as velocity gain, target velocity commanded directly).

Why not a position ramp: the MIT position encoding range is ±12.57 rad (±720°), so a ramp
saturates after at most two turns; kp=0 + target velocity gives unlimited rotation.
Ctrl-C stops and auto-releases torque.

⚠️ The motor must already be on the MIT protocol; run with the metal-lerobot environment:
    ~/miniconda3/envs/metal-lerobot/bin/python tools/experimental/lerobot/mit_spin.py --id 127 --speed 30
"""

import argparse
import time

MAX_SPEED_DEG_S = 720.0   # safety cap (RS00 physical limit ~1890°/s, never needed on the bench)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, required=True, help="motor CAN ID")
    ap.add_argument("--speed", type=float, default=30.0, help="target speed deg/s (negative = reverse)")
    ap.add_argument("--kd", type=float, default=1.0, help="velocity gain (increase if it won't turn, decrease if it judders)")
    ap.add_argument("--seconds", type=float, default=0.0, help="spin duration; 0 = until Ctrl-C")
    ap.add_argument("--model", default="O0")
    ap.add_argument("--channel", default="can0")
    a = ap.parse_args()
    if abs(a.speed) > MAX_SPEED_DEG_S:
        raise SystemExit(f"--speed exceeds the safety cap of ±{MAX_SPEED_DEG_S} deg/s")

    try:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.robstride import RobstrideMotorsBus
    except ImportError:
        raise SystemExit("missing lerobot — run this experimental tool with the metal-lerobot environment")

    m = Motor(a.id, a.model, MotorNormMode.DEGREES)
    m.recv_id = a.id
    m.motor_type_str = a.model
    bus = RobstrideMotorsBus(port=a.channel, motors={"m": m},
                             can_interface="socketcan", use_can_fd=False, bitrate=1_000_000)
    bus.connect()
    try:
        bus.enable_torque()
        print(f"spinning {a.speed:+.0f} deg/s (kd={a.kd}). Ctrl-C to stop and release torque.")
        t0 = time.monotonic()
        last_print = 0.0
        while a.seconds <= 0 or time.monotonic() - t0 < a.seconds:
            # Velocity-mode frame: kp=0 (position term disabled), kd=velocity gain, target velocity=speed
            bus._mit_control("m", 0.0, a.kd, 0.0, a.speed, 0.0)
            now = time.monotonic()
            if now - last_print >= 1.0:
                st = bus.sync_read("Present_Velocity")
                print(f"  measured speed {st['m']:+7.1f} deg/s")
                last_print = now
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        try:
            bus.disable_torque()
            print("torque released")
        except Exception as e:
            print(f"failed to release torque (power off manually): {e}")
        bus.disconnect()


if __name__ == "__main__":
    main()
