#!/usr/bin/env python3
"""Lesson 3: small-amplitude sine tracking on a single joint. Verifies control-loop smoothness. ⚠️ Run on the bench / in open space first."""

import math
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
    ap.add_argument("--joint", type=int, default=6, help="1-based joint number, defaults to the end joint")
    ap.add_argument("--amp", type=float, default=0.2, help="amplitude, rad")
    ap.add_argument("--freq", type=float, default=0.2, help="frequency, Hz")
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args()
    require_interactive_terminal("powered sine test")
    arm = arm_from_args(a)
    arm.connect()
    try:
        if not 1 <= a.joint <= arm.config.n_joints:
            raise SystemExit(f"--joint must be between 1 and {arm.config.n_joints}, got {a.joint}")
        input(f"about to enable and drive J{a.joint} in a ±{a.amp} rad sine. Confirm and press ENTER > ")
        arm.enable()
        start = arm.get_joint_positions()
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.seconds and arm.state.name == "ENABLED":
            t = time.monotonic() - t0
            targets = list(start)
            targets[a.joint - 1] = start[a.joint - 1] + a.amp * math.sin(2 * math.pi * a.freq * t)
            arm.set_joint_targets(targets)
            time.sleep(0.02)   # setting the target at 50Hz is enough; the 200Hz loop handles smoothing
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
