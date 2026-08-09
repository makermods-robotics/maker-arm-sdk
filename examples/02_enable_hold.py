#!/usr/bin/env python3
"""Lesson 2: enable and hold in place for 30 seconds. For first power-on trial runs / tuning kp kd. ⚠️ Make sure the area is clear of obstacles."""

import time

from _args import arm_from_args, make_parser


def _safe(fn):
    try:
        fn()
    except Exception as e:
        print(f"cleanup step failed (continuing): {e}")


def main():
    ap = make_parser(__doc__)
    ap.add_argument("--seconds", type=float, default=30.0)
    a = ap.parse_args()
    arm = arm_from_args(a)
    arm.connect()
    try:
        input("about to enable (holding in place, there should be no motion). Confirm and press ENTER > ")
        arm.enable()
        t_end = time.monotonic() + a.seconds
        while time.monotonic() < t_end and arm.state.name == "ENABLED":
            time.sleep(0.2)
            print(" ".join(f"J{i+1}={x:+7.3f}" for i, x in enumerate(arm.get_joint_positions())), flush=True)
        if arm.state.name == "FAULT":
            print("FAULT:", arm.fault_reason)
    except KeyboardInterrupt:
        pass
    finally:
        if arm.state.name in ("ENABLED", "FAULT"):
            print("\n🔒 arm is holding position -- steady it or move it to a safe pose, then press ENTER to release torque (Ctrl-C again also forces release)...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass
        _safe(arm.disable)
        _safe(arm.disconnect)
        print("torque released, exiting")


if __name__ == "__main__":
    main()
