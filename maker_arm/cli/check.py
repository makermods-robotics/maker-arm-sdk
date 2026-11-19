#!/usr/bin/env python3
"""Powered checks of one arm, all joints held together.

Default (jog): each motor receives one small, soft-limit-safe joint jog while the others hold;
the measured displacement decides PASS/FAIL, then the joint returns to its start.

--sine AMPL_DEG --period S --cycles N: the selected joints (default: all but the gripper) follow
a sine around the current pose while commanded and measured positions are logged at the control
rate. Reports per joint the RMS and max tracking error, a lag estimate, peak torque and
temperature rise; --csv keeps the samples.

--at NAME=DEG,... moves there first (slowly), so the test runs clear of the table, e.g.
--at shoulder_lift=-30,elbow_flex=40.

Afterwards the arm returns to rest and releases (--then).
"""

from __future__ import annotations

import csv
import math
import time

import numpy as np

from ..arm import Arm, ArmState
from ..rig import ArmGroup
from .common import (CliError, Interrupt, achieved, add_gravity_arg, add_then_arg, add_yes_arg, apply_gravity_arg,
                     check_stay, confirm, make_parser, run, target, wind_down)


# ── jog check ──
def choose_test_delta(position: float, lo: float, hi: float, magnitude: float) -> float:
    """Choose a signed delta that points into the configured range and stays within it."""
    if not all(math.isfinite(x) for x in (position, lo, hi, magnitude)) or magnitude <= 0:
        raise ValueError("position, limits, and positive magnitude must be finite")
    plus_room = hi - position
    minus_room = position - lo
    if plus_room >= magnitude and plus_room >= minus_room:
        return magnitude
    if minus_room >= magnitude:
        return -magnitude
    if plus_room >= magnitude:
        return magnitude
    raise ValueError(f"position {position:+.3f} has less than {magnitude} rad room inside [{lo}, {hi}]")


def wait_and_measure(arm: Arm, index: int, start: float, signed_delta: float,
                     seconds: float, stop=None) -> tuple[float, float]:
    direction = 1.0 if signed_delta > 0 else -1.0
    peak = -math.inf
    final = math.nan
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and arm.state is ArmState.ENABLED and not (stop and stop.is_set()):
        final = arm.get_joint_positions()[index]
        if math.isfinite(final):
            peak = max(peak, (final - start) * direction)
        time.sleep(0.02)
    return max(0.0, peak) if math.isfinite(peak) else math.nan, final


def interrupted(arm: Arm, stop) -> None:
    if stop is not None and stop.is_set():
        arm.hold_current_position()
        raise CliError("interrupted")


def jog(arm: Arm, a, say, stop=None) -> dict:
    """``stop`` (an Event) ends the jog between and during steps: the arm holds where it is."""
    results = {}
    for index, joint in enumerate(arm.config.joints):
        if a.motor is not None and joint.motor_id != a.motor:
            continue
        interrupted(arm, stop)
        if arm.state is not ArmState.ENABLED:
            raise CliError(f"arm left ENABLED state: {arm.fault_reason}")
        baseline = arm.get_joint_positions()
        start = baseline[index]
        signed_delta = choose_test_delta(start, joint.lo, joint.hi, a.delta)
        arm.set_joint_targets([start + signed_delta if i == index else p for i, p in enumerate(baseline)])
        peak, final = wait_and_measure(arm, index, start, signed_delta, a.move_seconds, stop)
        interrupted(arm, stop)
        passed = math.isfinite(peak) and peak >= a.minimum_response
        say(f"motor {joint.motor_id} {joint.label}: command {signed_delta:+.3f} rad, peak response {peak:+.4f} rad "
            f"{'PASS' if passed else 'FAIL'}")
        results[joint.label] = {"motor_id": joint.motor_id, "command": signed_delta, "peak": peak,
                                "final": final, "passed": passed}
        back = arm.get_joint_positions()
        back[index] = start
        arm.set_joint_targets(back)
        wait_and_measure(arm, index, start, -signed_delta, a.return_seconds, stop)
        time.sleep(0.1)
    if not results:
        raise CliError(f"motor {a.motor} is not in the profile")
    passed = sum(r["passed"] for r in results.values())
    say(f"\nresponse summary: {passed}/{len(results)} motors passed")
    return {"ok": passed == len(results), "mode": "jog", "joints": results}


# ── sine tracking ──
def sine_targets(q0, selected, amplitude, period, t):
    """Sine around q0; the amplitude eases in over the first half period."""
    s = amplitude * min(1.0, t / (period / 2)) * math.sin(2 * math.pi * t / period)
    return [q + s if i in selected else q for i, q in enumerate(q0)]


def lag_estimate(t, cmd, pos, period, window_start) -> float:
    """The delay (s) that best lines the measured trace up with the commanded one."""
    best, best_err = 0.0, math.inf
    for lag in np.arange(0.0, period / 4, 0.001):
        mask = t >= window_start + lag
        if mask.sum() < 3:
            break
        err = float(np.sqrt(np.mean((pos[mask] - np.interp(t[mask] - lag, t, cmd)) ** 2)))
        if err < best_err:
            best, best_err = float(lag), err
    return best


def analyse(log, names, selected, period) -> dict:
    t = np.array([row[0] for row in log])
    window = period / 2   # after the ease-in
    mask = t >= window
    report = {}
    for i in selected:
        cmd = np.array([row[1][i] for row in log])
        pos = np.array([row[2][i] for row in log])
        tau = np.array([row[3][i] for row in log])
        temp = np.array([row[4][i] for row in log])
        err = (cmd - pos)[mask]
        report[names[i]] = {
            "rms_error": float(np.sqrt(np.mean(err ** 2))) if err.size else math.nan,
            "max_error": float(np.max(np.abs(err))) if err.size else math.nan,
            "lag_s": lag_estimate(t, cmd, pos, period, window),
            "peak_torque": float(np.nanmax(np.abs(tau))),
            "temperature_rise": float(temp[-1] - temp[0]),
        }
    return report


def sine(arm: Arm, a, say, stop) -> dict:
    cfg = arm.config
    names = arm.joint_names
    wanted = a.joints or [n for n in names if n != "gripper"]
    unknown = [n for n in wanted if n not in names]
    if unknown:
        raise CliError(f"unknown joints {unknown}; joints: {', '.join(names)}")
    selected = [names.index(n) for n in wanted]
    amplitude = math.radians(a.sine)
    q0 = arm.get_joint_positions()
    for i in selected:
        j = cfg.joints[i]
        if not (j.lo + cfg.limit_margin <= q0[i] - amplitude and q0[i] + amplitude <= j.hi - cfg.limit_margin):
            raise CliError(f"{j.label}: {math.degrees(q0[i]):+.1f} deg +/- {a.sine:g} leaves its limits "
                           f"[{math.degrees(j.lo):.1f}, {math.degrees(j.hi):.1f}]")
    rate = cfg.control_rate_hz
    log = []
    t0 = time.monotonic()
    duration = a.cycles * a.period
    while not stop.event.is_set():
        t = time.monotonic() - t0
        if t > duration:
            break
        cmd = sine_targets(q0, selected, amplitude, a.period, t)
        if not arm.set_joint_targets(cmd):
            raise CliError(f"targets refused: {arm.state.name} {arm.fault_reason or ''}")
        log.append((t, cmd, arm.get_joint_positions(), arm.get_joint_torques(), arm.get_temperatures()))
        time.sleep(max(0.0, (len(log) / rate) - (time.monotonic() - t0)))
    arm.move_to(q0, timeout=2.0, cancel=stop.event)
    if len(log) < 10:
        raise CliError("interrupted before enough samples")
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + [f"{names[i]}_{k}" for i in selected for k in ("cmd", "pos", "torque", "temp")])
            for t, cmd, pos, tau, temp in log:
                w.writerow([f"{t:.4f}"] + [f"{v:.6g}" for i in selected for v in (cmd[i], pos[i], tau[i], temp[i])])
        say(f"samples: {a.csv}")
    report = analyse(log, names, selected, a.period)
    say(f"{len(log)} samples over {log[-1][0]:.1f} s ({len(log) / log[-1][0]:.0f} Hz)")
    say("joint            rms_deg  max_deg  lag_ms  peak_Nm  temp_rise_C")
    for name, r in report.items():
        say(f"{name:<16} {math.degrees(r['rms_error']):7.2f}  {math.degrees(r['max_error']):7.2f}  "
            f"{r['lag_s'] * 1000:6.0f}  {r['peak_torque']:7.2f}  {r['temperature_rise']:+11.1f}")
    return {"mode": "sine", "amplitude": amplitude, "period": a.period, "cycles": a.cycles, "samples": len(log),
            "joints": report, "units": "rad, s, N*m, C"}


AT_SPEED = math.radians(20.0)   # rad/s, the move to --at


def go_to(arm: Arm, text: str, say, stop) -> None:
    """Move to the pose given as NAME=DEG,... before the test."""
    q = arm.get_joint_positions()
    names = arm.joint_names
    for item in text.split(","):
        name, _, value = item.partition("=")
        if name.strip() not in names:
            raise CliError(f"--at expects NAME=DEG,...; joints: {', '.join(names)}")
        q[names.index(name.strip())] = math.radians(float(value))
    say(f"moving to {text}")
    if not arm.move_to(q, speed=AT_SPEED, cancel=stop.event):
        raise CliError("could not reach the --at pose")
    time.sleep(1.0)


def body(a, out) -> dict:
    if a.sine is None:
        if not 0 < a.delta <= 0.1:
            raise CliError("--delta must be in (0, 0.1] rad")
        if not 0 < a.minimum_response < a.delta:
            raise CliError("--minimum-response must be positive and smaller than --delta")
        if a.move_seconds < 0.5 or a.return_seconds < 0.5:
            raise CliError("move/return durations must each be at least 0.5 seconds")
    else:
        if not 0 < a.sine <= 45 or not a.period > 0 or not a.cycles >= 1:
            raise CliError("--sine must be in (0, 45] deg, --period > 0 and --cycles >= 1")
    t = target(a)
    arm = t.build()
    apply_gravity_arg(arm, a, out.say)
    check_stay([arm], a.then)
    if a.sine is None:
        arm.config.max_velocity = 0.12
        out.say(f"will enable {t.label} and jog each motor {a.delta} rad, then: {a.then}")
    else:
        peak = math.radians(a.sine) * 2 * math.pi / a.period
        if peak > arm.config.max_velocity:
            raise CliError(f"the sine's peak speed {math.degrees(peak):.0f} deg/s exceeds the arm's limit "
                           f"{math.degrees(arm.config.max_velocity):.0f} deg/s: lower --sine or raise --period")
        out.say(f"will enable {t.label} and run a {a.sine:g} deg, {a.period:g} s sine for {a.cycles} cycles "
                f"(peak {math.degrees(peak):.0f} deg/s), then: {a.then}")
    confirm(a, out, "energize the motors and move?")
    with Interrupt() as stop:
        try:
            arm.connect()
            arm.enable(cancel=stop.event)
            time.sleep(0.3)
            if a.at:
                go_to(arm, a.at, out.say, stop)
            result = jog(arm, a, out.say, stop.event) if a.sine is None else sine(arm, a, out.say, stop)
        finally:
            then = wind_down(ArmGroup({t.name: arm}), a.then, out)[t.name]
    result["then"] = then
    result["ok"] = result.get("ok", True) and achieved(then, a.then)
    return result


def main() -> None:
    ap = make_parser(__doc__, arms="one")
    ap.add_argument("--delta", type=float, default=0.05, help="jog: commanded test motion, rad")
    ap.add_argument("--minimum-response", type=float, default=0.008, help="jog: displacement to pass, rad")
    ap.add_argument("--move-seconds", type=float, default=1.2)
    ap.add_argument("--return-seconds", type=float, default=0.9)
    ap.add_argument("--motor", type=int, help="jog: check only this motor id")
    ap.add_argument("--at", metavar="NAME=DEG,...", help="move to this pose before the test")
    ap.add_argument("--sine", type=float, metavar="AMPL_DEG", help="sine tracking test amplitude, degrees")
    ap.add_argument("--period", type=float, default=2.0, metavar="S", help="sine period, s (default 2)")
    ap.add_argument("--cycles", type=int, default=3, help="sine cycles (default 3)")
    ap.add_argument("--joints", nargs="+", metavar="NAME", help="sine: joints to move (default: all but the gripper)")
    ap.add_argument("--csv", metavar="PATH", help="sine: write every sample to this CSV")
    add_then_arg(ap, "rest")
    add_gravity_arg(ap)
    add_yes_arg(ap)
    run("check", ap.parse_args(), body)


if __name__ == "__main__":
    main()
