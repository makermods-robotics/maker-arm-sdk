#!/usr/bin/env python3
"""Print joint angles while you move the arm by hand. Torque stays off.

    python examples/01_read_joints.py [--arm NAME]
"""

import argparse
import math
import time

from maker_arm.rig import Rig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arm", help="arm name from the rig file (default: the first arm)")
    a = ap.parse_args()
    rig = Rig.load()
    arm = rig.arm(a.arm or rig.names(["all"])[0])
    arm.connect()
    print("move the joints by hand; Ctrl-C to exit")
    try:
        while True:
            arm.refresh()
            print("  ".join(f"{n}={math.degrees(q):+7.1f}" for n, q in zip(arm.joint_names, arm.get_joint_positions())))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
