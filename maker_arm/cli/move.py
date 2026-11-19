#!/usr/bin/env python3
"""Move one arm: joints, a named pose, or the tool point in Cartesian space.

  move --joint elbow_flex --to 90 --joint wrist_flex --by -10     degrees; repeat the pairs
  move --pose zero                         every joint to 0 (clipped into its limits)
  move --pose rest                         the profile's rest_pose, else the zero pose clipped into limits
  move --pose shoulder_lift=-40,elbow_flex=60
  move --xyz 0.25 0 0.15 [--rpy 180 0 90]  tool point, metres (orientation in degrees; omitted = any)
  move --dxyz 0 0 0.05                     relative to the present tool pose, orientation kept

The arm is enabled at its current pose, moves at --speed (deg/s per joint, for Cartesian moves
too), then --then:
hold (default) keeps holding until Ctrl-C and then returns to rest; rest returns to the pose it
was enabled at and releases; stay exits with torque on.
"""

from __future__ import annotations

import argparse
import math

from ..rig import ArmGroup
from .common import (Interrupt, achieved, add_for_arg, add_gravity_arg, add_then_arg, add_yes_arg,
                     apply_gravity_arg, CliError, check_stay, confirm, make_parser, pose_text, run, target,
                     wind_down)


class JointAction(argparse.Action):
    """--joint NAME opens an entry; --to/--by complete the latest one."""

    def __call__(self, parser, ns, value, option=None):
        moves = getattr(ns, "moves", None) or []
        if self.dest == "joint":
            moves.append({"joint": value})
        elif not moves or len(moves[-1]) > 1:
            parser.error(f"{option} needs a --joint NAME before it")
        else:
            moves[-1][self.dest] = math.radians(value)
        ns.moves = moves


def clip(config, q: list[float]) -> list[float]:
    m = config.limit_margin
    return [min(j.hi - m, max(j.lo + m, v)) for j, v in zip(config.joints, q)]


def joint_goal(a, config, present: list[float]) -> list[float]:
    names = [j.name for j in config.joints]
    goal = list(present)

    def index(name: str) -> int:
        if name not in names:
            raise CliError(f"no joint {name!r}; joints: {', '.join(names)}")
        return names.index(name)

    if a.pose in ("zero", "rest"):
        rest = config.rest_pose if a.pose == "rest" else None
        goal = list(rest) if rest else [0.0] * len(names)
    elif a.pose:
        for item in a.pose.split(","):
            name, _, value = item.partition("=")
            if not value:
                raise CliError(f"--pose expects rest, zero or NAME=DEG,...; got {a.pose!r}")
            goal[index(name.strip())] = math.radians(float(value))
    for move in a.moves or []:
        if len(move) == 1:
            raise CliError(f"--joint {move['joint']} needs --to DEG or --by DEG")
        i = index(move["joint"])
        goal[i] = move["to"] if "to" in move else present[i] + move["by"]
    return clip(config, goal)


def cartesian(a, arm) -> tuple[list[float], list[float] | None]:
    if a.xyz:
        return list(a.xyz), [math.radians(v) for v in a.rpy] if a.rpy else None
    xyz, rpy = arm.ee_pose()
    return [p + d for p, d in zip(xyz, a.dxyz)], [math.radians(v) for v in a.rpy] if a.rpy else list(rpy)


def body(a, out) -> dict:
    if sum(bool(x) for x in (a.moves or a.pose, a.xyz, a.dxyz)) != 1:
        raise CliError("give exactly one of --joint/--pose, --xyz or --dxyz")
    if a.speed is not None and not a.speed > 0:
        raise CliError("--speed must be > 0 deg/s")
    t = target(a)
    arm = t.build()
    apply_gravity_arg(arm, a, out.say)
    names = arm.joint_names
    speed = math.radians(a.speed) if a.speed else None
    result: dict = {"arm": t.name}
    then = "rest" if a.then == "hold" else a.then
    check_stay([arm], then)
    with Interrupt() as stop:
        try:
            arm.connect()
            start = arm.get_joint_positions()
            if a.xyz or a.dxyz:
                xyz, rpy = cartesian(a, arm)
                out.say(f"{t.label}: tool point to xyz {['%.3f' % v for v in xyz]} m"
                        + (f", rpy {[round(math.degrees(v), 1) for v in rpy]} deg" if rpy else ""))
            else:
                goal = joint_goal(a, arm.config, start)
                out.say(f"{t.label}:\n  from {pose_text(names, start)}\n  to   {pose_text(names, goal)}")
            out.say(f"speed {a.speed or math.degrees(arm.config.max_velocity):.0f} deg/s, then {a.then}")
            confirm(a, out, "energize the motors and move?")
            arm.enable(cancel=stop.event)
            if a.xyz or a.dxyz:
                reached = bool(arm.move_ee(xyz, rpy, speed=speed, timeout=a.timeout, cancel=stop.event))
                goal = arm.get_commanded_positions()
                result["ee_target"] = {"xyz": xyz, "rpy": rpy}
                result["ee_pose"] = dict(zip(("xyz", "rpy"), arm.ee_pose()))
            else:
                reached = arm.move_to(goal, speed=speed, timeout=a.timeout, cancel=stop.event)
            final = arm.get_joint_positions()
            error = [g - f for g, f in zip(goal, final)]
            out.say(("reached" if reached else "NOT reached") + f": {pose_text(names, final)}")
            out.say("tracking error deg: " + "  ".join(f"{n}={math.degrees(e):+.2f}" for n, e in zip(names, error)))
            result.update(reached=reached, goal=dict(zip(names, goal)), final=dict(zip(names, final)),
                          error=dict(zip(names, error)), units="rad (joint), m")
            if a.then == "hold" and arm.state.name == "ENABLED":
                out.say("holding" + ("" if a.seconds is not None else "; Ctrl-C to return to rest"))
                stop.wait(a.seconds, stop_if=lambda: None if arm.state.name == "ENABLED" else arm.fault_reason)
        finally:
            result["then"] = wind_down(ArmGroup({t.name: arm}), then, out)[t.name]
    result["fault"] = arm.fault_reason
    result["ok"] = bool(result.get("reached")) and not arm.fault_reason and achieved(result["then"], then)
    return result


def main() -> None:
    ap = make_parser(__doc__, arms="one")
    ap.add_argument("--joint", action=JointAction, metavar="NAME", help="joint to move (pair with --to or --by)")
    ap.add_argument("--to", action=JointAction, type=float, metavar="DEG", help="absolute target, degrees")
    ap.add_argument("--by", action=JointAction, type=float, metavar="DEG", help="relative move, degrees")
    ap.add_argument("--pose", help="rest, zero, or NAME=DEG,NAME=DEG,...")
    ap.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"), help="tool point target, metres")
    ap.add_argument("--rpy", type=float, nargs=3, metavar=("R", "P", "Y"), help="tool orientation, degrees")
    ap.add_argument("--dxyz", type=float, nargs=3, metavar=("DX", "DY", "DZ"), help="relative tool move, metres")
    ap.add_argument("--speed", type=float, metavar="DEG_S", help="per-joint speed cap, deg/s, for joint and Cartesian moves (default: the profile's)")
    ap.add_argument("--timeout", type=float, metavar="S", help="give up on arrival after S seconds")
    add_then_arg(ap, "hold", choices=("hold", "rest", "stay"))
    add_for_arg(ap)
    add_gravity_arg(ap)
    add_yes_arg(ap)
    ap.set_defaults(moves=None)
    run("move", ap.parse_args(), body)


if __name__ == "__main__":
    main()
