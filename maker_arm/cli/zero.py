#!/usr/bin/env python3
"""Set the zero of the follower motors (and optionally the Star leader) at the zero pose.

Same flow as MakerModsLab's Maker calibration: torque is released, you move the arm by hand
to the ZERO POSE (folded against the base, gripper fully closed), then the current position
becomes zero. The zero persists in the motors (and in the leader servos).
"""

import math
import time

from maker_arm.cli.common import arm_from_args, make_parser
from maker_arm.cli.safety import require_interactive_terminal

ZERO_POSE = "folded against the base, gripper fully closed"


def show(arm, label: str) -> None:
    arm.refresh(wait=True)
    print(label, "  ".join(f"{j.label}={math.degrees(p):+.1f}" for j, p in
                           zip(arm.config.joints, arm.get_joint_positions())))


def zero_leader(port: str, servo_ids: list[int]) -> None:
    try:
        from motorbridge_smart_servo import FashionStarServo
    except ImportError:
        raise SystemExit("missing motorbridge-smart-servo: pip install motorbridge-smart-servo")
    bus = FashionStarServo(port, baudrate=1_000_000)
    try:
        for sid in servo_ids:
            if not bus.ping(sid):
                raise SystemExit(f"Star leader servo {sid} did not answer; nothing written")
        input(f"Move the Star Arm 102 leader by hand to its ZERO POSE -- {ZERO_POSE} -- then press ENTER > ")
        for sid in servo_ids:
            bus.unlock(sid)
            time.sleep(0.01)
            bus.set_origin_point(sid)
        print("Star leader zero set")
    finally:
        bus.close()


def main():
    ap = make_parser(__doc__)
    target = ap.add_mutually_exclusive_group(required=True)
    target.add_argument("--all", action="store_true", help="zero every follower motor (normal calibration)")
    target.add_argument("--motor", type=int, help="zero one follower motor by CAN ID (after replacing it)")
    target.add_argument("--leader-only", action="store_true", help="zero only the Star leader (needs --star-port)")
    ap.add_argument("--star-port", help="also zero the Star Arm 102 leader on this serial port")
    ap.add_argument("--star-ids", default="0,1,2,3,4,5,6")
    a = ap.parse_args()
    require_interactive_terminal("zeroing")
    if a.leader_only:
        if not a.star_port:
            ap.error("--leader-only needs --star-port")
        zero_leader(a.star_port, [int(x) for x in a.star_ids.split(",")])
        return

    arm = arm_from_args(a)
    selected = arm.motors if a.all else [m for m in arm.motors if m.motor_id == a.motor]
    if not selected:
        raise SystemExit(f"motor {a.motor} is not in {a.config}")
    arm.connect()
    try:
        for m in arm.motors:
            m.disable()   # the arm is moved by hand: torque stays off for the whole procedure
            time.sleep(0.01)
        print("Torque is off: support the arm.")
        show(arm, "current:")
        ids = ", ".join(str(m.motor_id) for m in selected)
        input(f"Move the arm by hand to its ZERO POSE -- {ZERO_POSE} -- then press ENTER to zero motor(s) {ids} "
              "(Ctrl-C cancels) > ")
        show(arm, "pre-zero:")
        for m in selected:
            m.set_zero()
            time.sleep(0.01)
        time.sleep(0.2)
        show(arm, "post-zero:")
        print("zero set; it persists in the motors")
    finally:
        arm.disconnect()
    if a.star_port:
        zero_leader(a.star_port, [int(x) for x in a.star_ids.split(",")])


if __name__ == "__main__":
    main()
