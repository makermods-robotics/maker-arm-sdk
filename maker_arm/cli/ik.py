#!/usr/bin/env python3
"""Joint angles that put the tool at a pose (inverse kinematics). No motion.

  ik --xyz 0.25 0 0.15                       position only, seeded from the arm's current joints
  ik --xyz 0.25 0 0.15 --rpy 180 0 90        full pose (degrees)
  ik --xyz ... --seed 0 -90 90 0 0 0         seed in degrees; no hardware needed
  ik --xyz ... --to-xyz 0.25 0.05 0.15 --steps 50    then a straight tool line from there (trajectory IK)

Solutions stay inside both the URDF and the profile's soft limits. Exit status 1 when the pose is
not reached within tolerance (the best joints found are still printed). With ``--to-xyz`` the line
runs at the profile's control rate (minimum-jerk timing, the orientation turning to the one a
position-only solve reaches at its end, or kept at ``--rpy``) and the largest joint step per tick
is printed; it also exits 1 when the line ends more than 1 mm / 0.5 deg off.
"""

from __future__ import annotations

import math

import numpy as np

from .. import trajectory_ik
from ..arm import EE_ANGLE_TOL, EE_POSITION_TOL
from .common import Bus, CliError, make_parser, run, target
from .fk import chain_for


def body(a, out) -> dict:
    t = target(a)
    config = t.config()
    chain = chain_for(config)
    joints = list(config.joints)[:chain.n]
    if a.seed:
        if len(a.seed) < chain.n:
            raise CliError(f"--seed needs {chain.n} values")
        seed, source = [math.radians(v) for v in a.seed[:chain.n]], "given"
    else:
        with Bus(t.backend(), config) as bus:
            rows = {r["motor_id"]: r for r in bus.read_all()}
        seed = [rows.get(j.motor_id, {}).get("position") for j in joints]
        if any(v is None for v in seed):
            raise CliError(f"{t.name}: could not read the joints for a seed (pass --seed)")
        source = "measured"
    lower = np.maximum(chain.lower, [j.lo + config.limit_margin for j in joints])
    upper = np.minimum(chain.upper, [j.hi - config.limit_margin for j in joints])
    rpy = [math.radians(v) for v in a.rpy] if a.rpy else None
    result = chain.ik(np.asarray(a.xyz, dtype=float), np.clip(seed, lower, upper), rpy=rpy, limits=(lower, upper))
    names = [j.name for j in joints]
    out.say(("solved" if result.ok else "NOT solved") + f" (seed {source}, {result.iters} iterations)")
    out.say("joints deg: " + "  ".join(f"{n}={math.degrees(v):+.2f}" for n, v in zip(names, result.q)))
    out.say(f"residual: position {result.pos_err * 1000:.2f} mm" +
            (f", orientation {math.degrees(result.rot_err):.2f} deg" if rpy else " (position only)"))
    doc = {"ok": result.ok, "joints": dict(zip(names, result.q)),
           "joints_deg": {n: math.degrees(v) for n, v in zip(names, result.q)},
           "position_error": result.pos_err, "orientation_error": result.rot_err, "iterations": result.iters,
           "seed": source, "units": "rad, m"}
    if a.to_xyz:
        doc["path"] = path(a, config, chain, np.asarray(result.q), lower, upper, names, out)
        doc["ok"] = doc["ok"] and doc["path"]["ok"]
    return doc


def path(a, config, chain, q0, lower, upper, names, out) -> dict:
    """The straight tool line from the solved pose to --to-xyz in --steps control ticks."""
    if a.steps < 1:
        raise CliError("--steps must be >= 1")
    ik = trajectory_ik.TrajectoryIK.from_config(config, chain=chain)
    start = chain.fk(q0)
    goal = start.copy()
    goal[:3, 3] = a.to_xyz
    if not a.rpy:
        goal[:3, :3] = chain.fk(chain.ik(np.asarray(a.to_xyz, dtype=float), q0, limits=(lower, upper)).q)[:3, :3]
    rate = config.control_rate_hz
    plan = trajectory_ik.solve_path(ik, trajectory_ik.straight_line(start, goal, a.steps), q0, rate)
    step = np.abs(np.diff(np.vstack([q0, plan.q]), axis=0))
    worst = step.max(axis=0)
    ok = bool(plan.pos_err[-1] <= EE_POSITION_TOL and plan.rot_err[-1] <= EE_ANGLE_TOL)
    out.say(f"path: {a.steps} ticks at {rate:g} Hz ({a.steps / rate:.2f} s), " + ("reached" if ok else "NOT reached")
            + f", end {plan.pos_err[-1] * 1000:.2f} mm / {math.degrees(plan.rot_err[-1]):.2f} deg off")
    out.say("max joint step per tick deg: " + "  ".join(f"{n}={math.degrees(v):.3f}" for n, v in zip(names, worst)))
    out.say(f"max tool error on the way: {plan.pos_err.max() * 1000:.2f} mm, "
            f"{math.degrees(plan.rot_err.max()):.2f} deg; ticks at the speed cap: {int(plan.capped.sum())}")
    return {"ok": ok, "ticks": a.steps, "rate_hz": rate, "max_step": dict(zip(names, worst.tolist())),
            "max_step_deg": {n: math.degrees(v) for n, v in zip(names, worst)},
            "end_position_error": float(plan.pos_err[-1]), "end_orientation_error": float(plan.rot_err[-1]),
            "max_position_error": float(plan.pos_err.max()), "capped_ticks": int(plan.capped.sum()),
            "joints_end": dict(zip(names, plan.q[-1].tolist()))}


def main() -> None:
    ap = make_parser(__doc__, arms="one")
    ap.add_argument("--xyz", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"), help="tool point, metres")
    ap.add_argument("--rpy", type=float, nargs=3, metavar=("R", "P", "Y"), help="tool orientation, degrees")
    ap.add_argument("--seed", type=float, nargs="+", metavar="DEG", help="seed joints, degrees (default: measured)")
    ap.add_argument("--to-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="then solve a straight tool line from --xyz to here, metres")
    ap.add_argument("--steps", type=int, default=50, metavar="N", help="control ticks along the line (default 50)")
    run("ik", ap.parse_args(), body)


if __name__ == "__main__":
    main()
