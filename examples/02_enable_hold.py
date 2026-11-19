#!/usr/bin/env python3
"""Enable at the current pose, hold it, then return to rest and release. Keep the area clear.

    python examples/02_enable_hold.py [--arm NAME] [--seconds 10]
"""

import argparse
import time

from maker_arm.rig import Rig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arm", help="arm name from the rig file (default: the first arm)")
    ap.add_argument("--seconds", type=float, default=10.0)
    a = ap.parse_args()
    rig = Rig.load()
    arm = rig.arm(a.arm or rig.names(["all"])[0])
    arm.connect()
    try:
        input("support the arm, then press ENTER to enable (no motion expected) > ")
        arm.enable()
        deadline = time.monotonic() + a.seconds
        while time.monotonic() < deadline and arm.state.name == "ENABLED":
            time.sleep(0.1)
        if arm.fault_reason:
            print("fault:", arm.fault_reason)
    except KeyboardInterrupt:
        pass
    finally:
        if not arm.stop():   # return to rest, then release; False: still holding where it is
            input("return to rest not verified; support the arm and press ENTER to release > ")
        arm.disconnect()


if __name__ == "__main__":
    main()
