#!/usr/bin/env python3
"""Zero-position calibration: manually move the arm to the zero pose, then run this. Zeroes each motor and saves to flash."""

import argparse
import time

from maker_arm.arm import Arm


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/maker_arm.yaml")
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    a = ap.parse_args()

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    try:
        print("current motor coordinates:", [f"{x:+.3f}" for x in arm.get_joint_positions()])
        input("confirm the arm is at the zero pose and no motor is enabled, press ENTER to start zeroing (Ctrl-C to cancel) > ")
        for m in arm.motors:
            m.set_zero()
            time.sleep(0.05)
            m.save_params()
            time.sleep(0.05)
            print(f"motor {m.motor_id}: zeroed and saved")
        time.sleep(0.2)
        arm.refresh()
        time.sleep(0.2)
        print("position after zeroing (should be ~0):", [f"{x:+.3f}" for x in arm.get_joint_positions()])
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
