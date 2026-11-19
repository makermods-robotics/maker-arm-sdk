#!/usr/bin/env python3
"""Float the arm for hand-guiding, or check the gravity model against measured torque.

  gravity              kp 0 on every joint, light damping, gravity feed-forward only: the arm holds
                       its own weight while you move it by hand. Ctrl-C (or --for S) re-engages the
                       profile gains where you left it, then --then (default rest).
  gravity --check      hold still at the current pose for --seconds, then print per joint the measured
                       torque, the model's holding torque and their difference.
"""

from __future__ import annotations

import math
import time

from ..gravity import from_config
from ..rig import ArmGroup
from .common import (CliError, Interrupt, achieved, add_for_arg, add_gravity_arg, add_then_arg, add_yes_arg,
                     apply_gravity_arg, check_stay, confirm, group_of, make_parser, not_enabled, num, run,
                     targets, wind_down)


def model_for(config):
    """``q -> holding torque per chain joint`` (N*m, with the profile's scales), or None."""
    ff = from_config(config)
    return ff.model if ff is not None else None


def float_arms(a, out) -> dict:
    chosen = targets(a, many=True)
    group = group_of(chosen)
    for name, arm in group:
        apply_gravity_arg(arm, a, out.say)
        if model_for(arm.config) is None:
            out.warn(f"{name}: the profile has no gravity model; floating joints will fall under their weight")
    check_stay(group.arms.values(), a.then)
    until = f"for {a.seconds:g} s" if a.seconds is not None else "until Ctrl-C"
    out.say(f"will enable {', '.join(t.label for t in chosen)} and float every joint (kp 0, gravity "
            f"feed-forward) {until}; then re-engage gains where it is left, then: {a.then}")
    confirm(a, out, "energize the motors and float?")
    reason = "not started"
    with Interrupt() as stop:
        try:
            group.connect()
            group.enable(cancel=stop.event)
            for _, arm in group:
                arm.float_mode(kd=a.kd)
            out.say("floating: move the arm by hand" + ("" if a.seconds is not None else "; Ctrl-C to finish"))
            reason = stop.wait(a.seconds, stop_if=lambda: not_enabled(group.arms))
            for _, arm in group:
                if arm.state.name in ("ENABLED", "FAULT"):
                    arm.hold_current_position()   # target first, then stiffness: never yank toward an old target
                    arm.set_gains()
        finally:
            faults = {n: arm.fault_reason for n, arm in group if arm.fault_reason}
            outcome = wind_down(group, a.then, out)
    return {"ok": not faults and achieved(outcome, a.then), "stopped_by": reason, "then": outcome,
            "faults": faults}


def check(a, out) -> dict:
    chosen = targets(a, many=False)
    t = chosen[0]
    arm = t.build()
    apply_gravity_arg(arm, a, out.say)
    model = model_for(arm.config)
    if model is None:
        raise CliError("the profile has no gravity model (no URDF), nothing to check against")
    check_stay([arm], a.then)
    out.say(f"will enable {t.label} at its current pose, hold still {a.seconds_check:g} s and compare "
            f"torques, then: {a.then}")
    confirm(a, out, "energize the motors?")
    samples = []
    with Interrupt() as stop:
        try:
            arm.connect()
            arm.enable(cancel=stop.event)
            time.sleep(a.settle)
            deadline = time.monotonic() + a.seconds_check
            while time.monotonic() < deadline and arm.state.name == "ENABLED" and not stop.event.is_set():
                samples.append((arm.get_joint_positions(), arm.get_joint_torques()))
                time.sleep(0.01)
        finally:
            fault = arm.fault_reason
            then = wind_down(ArmGroup({t.name: arm}), a.then, out)[t.name]
    if not samples or fault:
        raise CliError(f"no measurement: {fault or 'interrupted'}")
    n = len(samples[0][0])
    q = [sum(s[0][i] for s in samples) / len(samples) for i in range(n)]
    measured = [sum(s[1][i] for s in samples) / len(samples) for i in range(n)]
    predicted = list(model(q))
    rows = {}
    out.say("joint            measured_Nm  model_Nm   diff_Nm")
    for i, name in enumerate(arm.joint_names):
        m = predicted[i] if i < len(predicted) else math.nan
        rows[name] = {"position": q[i], "measured": measured[i], "model": m, "difference": measured[i] - m}
        out.say(f"{name:<16} {num(measured[i], 11, 3)} {num(m, 9, 3)} {num(measured[i] - m, 9, 3)}")
    return {"ok": achieved(then, a.then), "arm": t.name, "samples": len(samples), "joints": rows, "then": then,
            "units": "rad, N*m"}


def body(a, out) -> dict:
    return check(a, out) if a.check else float_arms(a, out)


def main() -> None:
    ap = make_parser(__doc__, arms="many")
    ap.add_argument("--check", action="store_true", help="compare measured holding torque with the model")
    ap.add_argument("--seconds", dest="seconds_check", type=float, default=2.0, help="--check: averaging time")
    ap.add_argument("--settle", type=float, default=0.5, help="--check: wait after enable before measuring, s")
    ap.add_argument("--kd", type=float, help="damping while floating (default: the SDK's float damping)")
    add_for_arg(ap, "float")
    add_then_arg(ap, "rest")
    add_gravity_arg(ap)
    add_yes_arg(ap)
    run("gravity", ap.parse_args(), body)


if __name__ == "__main__":
    main()
