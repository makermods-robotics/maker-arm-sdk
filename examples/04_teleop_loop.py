#!/usr/bin/env python3
"""A teleop loop written out: read a Star leader, send its targets to the follower.

The SDK's Teleop class (``maker-arm teleop``) does the same with a slow startup sync, a speed cap
and leader-loss handling; this is the minimum to build your own on.

    python examples/04_teleop_loop.py --star-port /dev/ttyUSB0 [--arm NAME]
"""

import argparse
import time

from maker_arm.leaders import StarLeader
from maker_arm.rig import Rig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arm", help="arm name from the rig file (default: the first arm)")
    ap.add_argument("--star-port", required=True)
    a = ap.parse_args()
    rig = Rig.load()
    arm = rig.arm(a.arm or rig.names(["all"])[0])
    leader = StarLeader(a.star_port)
    leader.connect()
    arm.connect()
    try:
        input("put the leader in the follower's pose, then press ENTER to enable > ")
        arm.enable()
        while arm.state.name == "ENABLED":
            q = leader.read()             # follower joint targets (rad), or None on a missed read
            if q is not None:
                arm.set_joint_targets(q)  # rate-limited toward q by the arm's control loop
            time.sleep(1 / 50)
        print("stopped:", arm.fault_reason)
    except KeyboardInterrupt:
        pass
    finally:
        if not arm.stop():
            input("return to rest not verified; support the arm and press ENTER to release > ")
        arm.disconnect()
        leader.close()


if __name__ == "__main__":
    main()
