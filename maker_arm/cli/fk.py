#!/usr/bin/env python3
"""Tool pose (forward kinematics) for the arm's current joints, or for joints you give. No motion.

  fk                                   read the joints (read-only bus access)
  fk --joints 0 -90 90 0 0 0           degrees, in profile order; no hardware needed
  fk --joints ... --set elbow_flex=45  override single joints

Needs a profile that names its URDF. Position in metres, orientation as roll/pitch/yaw degrees
(URDF fixed axes), in the arm's base frame.
"""

from __future__ import annotations

import math

import numpy as np

from ..kinematics import chain_from_config, pose_to_xyz_rpy
from .common import Bus, CliError, make_parser, run, target


def chain_for(config):
    chain = chain_from_config(config)
    if chain is None:
        raise CliError("the arm profile names no URDF (profile key 'urdf'), so it has no kinematic model")
    return chain


def joints_from_args(a, t, config) -> tuple[list[float], str]:
    """Joint angles (rad, profile order) from --joints/--set, else read from the arm."""
    names = [j.name for j in config.joints]
    if a.joints:
        if len(a.joints) not in (len(names), len(names) - 1):
            raise CliError(f"--joints needs {len(names)} values (or {len(names) - 1} without the gripper)")
        q, source = [math.radians(v) for v in a.joints] + [0.0] * (len(names) - len(a.joints)), "given"
    else:
        with Bus(t.backend(), config) as bus:
            rows = {r["motor_id"]: r for r in bus.read_all()}
        q = [rows.get(j.motor_id, {}).get("position") for j in config.joints]
        if any(v is None for v in q):
            raise CliError(f"{t.name}: could not read every joint (pass --joints to compute without the arm)")
        source = "measured"
    for item in a.set or []:
        name, _, value = item.partition("=")
        if name not in names or not value:
            raise CliError(f"--set expects NAME=DEG with NAME one of {', '.join(names)}")
        q[names.index(name)] = math.radians(float(value))
    return q, source


def body(a, out) -> dict:
    t = target(a)
    config = t.config()
    chain = chain_for(config)
    q, source = joints_from_args(a, t, config)
    xyz, rpy = pose_to_xyz_rpy(chain.fk(np.asarray(q[:chain.n])))
    names = [j.name for j in config.joints]
    out.say(f"joints ({source}, deg): " + "  ".join(f"{n}={math.degrees(v):+.1f}" for n, v in zip(names, q)))
    out.say(f"xyz m:   {xyz[0]:+.4f} {xyz[1]:+.4f} {xyz[2]:+.4f}")
    out.say(f"rpy deg: {math.degrees(rpy[0]):+.2f} {math.degrees(rpy[1]):+.2f} {math.degrees(rpy[2]):+.2f}")
    return {"joints": dict(zip(names, q)), "source": source, "xyz": list(xyz), "rpy": list(rpy),
            "rpy_deg": [math.degrees(v) for v in rpy], "units": "rad, m"}


def main() -> None:
    ap = make_parser(__doc__, arms="one")
    ap.add_argument("--joints", type=float, nargs="+", metavar="DEG", help="joint angles in profile order, degrees")
    ap.add_argument("--set", action="append", metavar="NAME=DEG", help="override one joint (repeatable)")
    run("fk", ap.parse_args(), body)


if __name__ == "__main__":
    main()
