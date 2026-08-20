#!/usr/bin/env python3
"""Lesson 2: enable and hold in place for 30 seconds. For first power-on trial runs / tuning kp kd. ⚠️ Make sure the area is clear of obstacles."""

import time

from maker_arm.cli.common import arm_from_args, make_parser
from maker_arm.cli.safety import release_if_holding, require_interactive_terminal


def _safe(fn):
    try:
        fn()
    except Exception as e:
        print(f"cleanup step failed (continuing): {e}")


def main():
    ap = make_parser(__doc__)
    ap.add_argument("--seconds", type=float, default=30.0)
    a = ap.parse_args()
    require_interactive_terminal("powered hold")
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
        torque_was_enabled = arm.state.name in ("ENABLED", "FAULT")
        _safe(lambda: release_if_holding(arm))
        _safe(arm.disconnect)
        if torque_was_enabled:
            print("torque released, exiting")
        else:
            print("torque was not enabled; transport closed without sending disable")


if __name__ == "__main__":
    main()
