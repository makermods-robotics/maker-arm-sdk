#!/usr/bin/env python3
"""Star leader tools.

  leader zero --star-port P     make the leader's present pose its zero (it must be at the zero pose)
  leader read --star-port P     print the follower joint targets the leader maps to, live (Ctrl-C)

--arm NAME takes the port and mapping from that arm's leader entry in the rig file. Without a
terminal, zero needs --yes-i-am-at-the-zero-pose.
"""

from __future__ import annotations

import json
import math
import time

from .._product import ZERO_POSE
from ..leaders import StarLeader
from ..leaders.base import LeaderError
from .common import CliError, Interrupt, add_for_arg, clean, confirm, load_rig, make_parser, run


def star_leader(a) -> StarLeader:
    if a.star_port:
        return StarLeader(a.star_port, mapping=a.map)
    rig = load_rig(a)
    names = a.arm or [n for n, s in rig.leaders.items() if s.type == "star"]
    if len(names) != 1:
        raise CliError("pass --star-port, or --arm NAME whose rig leader is a Star leader")
    spec = rig.leaders.get(names[0])
    if spec is None or spec.type != "star":
        raise CliError(f"arm {names[0]!r} has no Star leader in the rig file")
    return rig.leader(names[0], follower_arm=None)


def zero_leader(leader: StarLeader, a, out) -> None:
    """Set the Star leader's origin at its present pose, after the operator confirms the pose."""
    leader.connect()
    try:
        if not getattr(a, "at_zero_pose", False):
            confirm(a, out, f"Move the Star leader by hand to its ZERO POSE -- {ZERO_POSE} -- then confirm. Zero it?",
                    word="yes")
        leader.zero()
        out.say("Star leader zero set")
    finally:
        leader.close()


def read(leader: StarLeader, a, out) -> dict:
    leader.connect()
    names = [str(i) for i in leader.servo_ids]
    last, count = None, 0
    try:
        with Interrupt() as stop:
            t0 = time.monotonic()
            while not stop.event.is_set() and (a.seconds is None or time.monotonic() - t0 < a.seconds):
                targets = leader.read()
                if targets is not None:
                    last, count = targets, count + 1
                    if out.json and a.stream:
                        print(json.dumps(clean({"t": time.monotonic() - t0, "targets": targets})), flush=True)
                    elif not out.json:
                        out.say("  ".join(f"{math.degrees(v):+7.1f}" for v in targets))
                stop.wait(1.0 / a.rate)
    finally:
        leader.close()
    if last is None:
        raise LeaderError("no reading from the Star leader")
    return {"servo_ids": names, "readings": count, "targets": last, "units": "rad (follower joints)"}


def body(a, out) -> dict:
    leader = star_leader(a)
    if a.action == "zero":
        zero_leader(leader, a, out)
        return {"zeroed": True, "port": leader.port}
    return read(leader, a, out)


def main() -> None:
    ap = make_parser(__doc__, arms=None)
    ap.add_argument("action", choices=("zero", "read"))
    ap.add_argument("--star-port", help="Star leader serial port")
    ap.add_argument("--map", help="Star mapping JSON (default: the shipped one)")
    ap.add_argument("--rig")
    ap.add_argument("--arm", action="append", metavar="NAME", help="take the leader from this arm's rig entry")
    ap.add_argument("--rate", type=float, default=10.0, help="read: prints per second (default 10)")
    ap.add_argument("--stream", action="store_true", help="read --json: one JSON line per reading")
    ap.add_argument("--yes-i-am-at-the-zero-pose", dest="at_zero_pose", action="store_true",
                    help="zero: skip the confirmation (the leader is already at the zero pose)")
    add_for_arg(ap, "read")
    run("leader", ap.parse_args(), body)


if __name__ == "__main__":
    main()
