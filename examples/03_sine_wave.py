#!/usr/bin/env python3
"""Drive one joint along a small sine with set_joint_targets in a loop, then return to rest.

    python examples/03_sine_wave.py [--arm NAME] [--joint wrist_roll] [--amplitude 10] [--period 4]
"""

import argparse
import math
import time

from maker_arm.rig import Rig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arm", help="arm name from the rig file (default: the first arm)")
    ap.add_argument("--joint", default="wrist_roll")
    ap.add_argument("--amplitude", type=float, default=10.0, help="degrees")
    ap.add_argument("--period", type=float, default=4.0, help="seconds")
    ap.add_argument("--cycles", type=int, default=3)
    a = ap.parse_args()
    rig = Rig.load()
    arm = rig.arm(a.arm or rig.names(["all"])[0])
    i = arm.joint_names.index(a.joint)
    arm.connect()
    try:
        input(f"support the arm, then press ENTER to move {a.joint} ±{a.amplitude:g} deg > ")
        arm.enable()
        start = arm.get_joint_positions()
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < a.cycles * a.period and arm.state.name == "ENABLED":
            q = list(start)
            q[i] += math.radians(a.amplitude) * math.sin(2 * math.pi * t / a.period)
            arm.set_joint_targets(q)   # the arm's own loop rate-limits and sends the frames
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if not arm.stop():
            input("return to rest not verified; support the arm and press ENTER to release > ")
        arm.disconnect()


if __name__ == "__main__":
    main()
