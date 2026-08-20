#!/usr/bin/env python3
"""Zero one replacement motor or the full arm at the model's defined reference pose."""

import argparse
import time

from maker_arm.arm import Arm
from maker_arm.profiles import DEFAULT_ARM_CONFIG


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    target = ap.add_mutually_exclusive_group(required=True)
    target.add_argument("--motor", type=int, help="zero one replacement motor by CAN ID")
    target.add_argument("--all", action="store_true", help="zero all configured motors")
    a = ap.parse_args()

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    try:
        selected = arm.motors if a.all else [m for m in arm.motors if m.motor_id == a.motor]
        if not selected:
            raise SystemExit(f"motor {a.motor} is not present in {a.config}")
        print("current motor coordinates:", [f"{x:+.3f}" for x in arm.get_joint_positions()])
        ids = ", ".join(str(m.motor_id) for m in selected)
        input(
            f"confirm CAN motor(s) {ids} are at the model-defined zero pose and torque is released; "
            "press ENTER to write zero (Ctrl-C cancels) > "
        )
        for m in selected:
            m.set_zero()
            time.sleep(0.05)
            m.save_params()
            time.sleep(0.05)
            print(f"motor {m.motor_id}: zeroed and saved")
        time.sleep(0.2)
        fresh = arm.refresh(wait=True, timeout=0.05)
        stale = [arm.motors[i].motor_id for i, ok in enumerate(fresh) if not ok]
        if stale:
            print(f"warning: no fresh post-zero feedback from CAN motors {stale}")
        print("post-zero motor coordinates:", [f"{x:+.3f}" for x in arm.get_joint_positions()])
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
