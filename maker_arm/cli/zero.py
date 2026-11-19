#!/usr/bin/env python3
"""Set the zero of the follower motors (and optionally the Star leader) at the zero pose.

Torque is released, you move the arm by hand to the ZERO POSE, then the present position becomes
zero. The zero persists in the motors (and in the leader servos). A wrong zero makes every later
motion wrong, so there is no plain --yes: without a terminal pass --yes-i-am-at-the-zero-pose,
which zeroes the arm where it already is.
"""

from __future__ import annotations

import math
import time

from .._product import ZERO_POSE
from ..leaders import StarLeader
from .common import CliError, confirm, make_parser, pose_text, run, target
from .leader import zero_leader


def body(a, out) -> dict:
    if a.leader_only:
        if not a.star_port:
            raise CliError("--leader-only needs --star-port")
        zero_leader(StarLeader(a.star_port, mapping=a.map), a, out)
        return {"leader": True}
    t = target(a)
    arm = t.build()
    selected = [i for i, m in enumerate(arm.motors) if a.all or m.motor_id == a.motor]
    if not selected:
        raise CliError(f"motor {a.motor} is not in the profile")
    ids = [arm.motors[i].motor_id for i in selected]
    if not a.at_zero_pose:   # before anything is sent: releasing torque drops an energized arm
        confirm(a, out, f"{t.label}: torque will be released on every motor so you can move the arm by hand; "
                        "is it supported?")
    arm.connect()
    try:
        for m in arm.motors:
            m.disable()   # the arm is moved by hand: torque stays off for the whole procedure
            time.sleep(0.01)
        arm.refresh(wait=True)
        before = arm.get_joint_positions()
        out.say(f"{t.label}: torque is off, support the arm. Current: {pose_text(arm.joint_names, before)}")
        if not a.at_zero_pose:
            confirm(a, out, f"Move the arm by hand to its ZERO POSE -- {ZERO_POSE} -- then confirm to zero "
                            f"motor(s) {ids}", word="zero")
        arm.refresh(wait=True)
        pre = arm.get_joint_positions()
        arm.set_zero(selected)
        time.sleep(0.2)
        arm.refresh(wait=True)
        post = arm.get_joint_positions()
        out.say(f"pre-zero:  {pose_text(arm.joint_names, pre)}")
        out.say(f"post-zero: {pose_text(arm.joint_names, post)}")
        out.say("zero set; it persists in the motors")
    finally:
        arm.disconnect()
    names = [arm.joint_names[i] for i in selected]
    residual = {n: post[i] for n, i in zip(names, selected)}
    if a.star_port:
        zero_leader(StarLeader(a.star_port, mapping=a.map), a, out)
    ok = all(math.isfinite(v) and abs(v) < math.radians(1.0) for v in residual.values())
    return {"ok": ok, "arm": t.name, "zeroed": names, "pre_zero": dict(zip(arm.joint_names, pre)),
            "post_zero": residual, "leader": bool(a.star_port), "units": "rad"}


def main():
    ap = make_parser(__doc__, arms="one")
    what = ap.add_mutually_exclusive_group(required=True)
    what.add_argument("--all", action="store_true", help="zero every follower motor (normal calibration)")
    what.add_argument("--motor", type=int, help="zero one follower motor by CAN ID (after replacing it)")
    what.add_argument("--leader-only", action="store_true", help="zero only the Star leader (needs --star-port)")
    ap.add_argument("--star-port", help="also zero the Star leader on this serial port")
    ap.add_argument("--map", help="Star mapping JSON (default: the shipped one)")
    ap.add_argument("--yes-i-am-at-the-zero-pose", dest="at_zero_pose", action="store_true",
                    help="zero without asking: the arm (and leader) are at the zero pose now")
    run("zero", ap.parse_args(), body)


if __name__ == "__main__":
    main()
