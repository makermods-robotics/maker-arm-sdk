#!/usr/bin/env python3
"""MIT-mode single-motor jog: move --delta degrees from the current position, then auto-release torque.

⚠️ The motor must already be switched to the MIT protocol (tools/switch_protocol.py --to mit + power-cycle).
⚠️ This tool uses lerobot's RobstrideMotorsBus — run it in the metal-lerobot environment:
    ~/miniconda3/envs/metal-lerobot/bin/python tools/mit_move.py --id 127 --delta 15
"""

import argparse
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, required=True, help="motor CAN ID")
    ap.add_argument("--delta", type=float, required=True, help="move angle (degrees, negative = reverse)")
    ap.add_argument("--kp", type=float, default=15.0)
    ap.add_argument("--kd", type=float, default=0.8)
    ap.add_argument("--seconds", type=float, default=2.4, help="command duration")
    ap.add_argument("--model", default="O0", help="O-series model name (default O0=RS00)")
    ap.add_argument("--channel", default="can0")
    a = ap.parse_args()

    try:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.robstride import RobstrideMotorsBus
    except ImportError:
        raise SystemExit("missing lerobot — please run with the metal-lerobot environment: ~/miniconda3/envs/metal-lerobot/bin/python tools/mit_move.py ...")

    m = Motor(a.id, a.model, MotorNormMode.DEGREES)
    m.recv_id = a.id
    m.motor_type_str = a.model
    bus = RobstrideMotorsBus(port=a.channel, motors={"m": m},
                             can_interface="socketcan", use_can_fd=False, bitrate=1_000_000)
    bus.connect()
    try:
        p0 = bus.sync_read("Present_Position")["m"]
        print(f"current {p0:.2f}°")
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
        print(f"target {target:.2f}° -> actual {p1:.2f}° (error {p1 - target:+.2f}°){' ✅' if ok else ' ⚠️ did not move as expected'}")
    finally:
        try:
            bus.disable_torque()
        except Exception as e:
            print(f"failed to release torque (power off manually): {e}")
        bus.disconnect()


if __name__ == "__main__":
    main()
