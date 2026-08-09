#!/usr/bin/env python3
"""Lesson 1: read-only, no motion. Confirm communication, angle readings, and that manual-push direction matches intuition."""

import time

from _args import arm_from_args, make_parser


def main():
    arm = arm_from_args(make_parser(__doc__).parse_args())
    arm.connect()
    print("connected. Push joints by hand and watch the angles change, Ctrl-C to exit.")
    try:
        while True:
            arm.refresh()
            time.sleep(0.1)
            print(" ".join(f"J{i+1}={x:+7.3f}" for i, x in enumerate(arm.get_joint_positions())), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
