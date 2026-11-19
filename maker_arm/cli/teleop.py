#!/usr/bin/env python3
"""Teleoperation: a leader drives each follower arm.

  teleop --leader star --star-port /dev/ttyUSB0              Star leader -> the (only) follower
  teleop --leader arm --leader-arm spare --arm left          a second arm, floated, leads 'left'
  teleop --arm left --arm right                              leaders from the rig file (bimanual)
  teleop --leader star --star-port P1 --arm left --star-port P2 --arm right

Pairs are matched in order (--arm with --star-port / --leader-arm). The loop below is the whole
program: connect the leaders, enable the followers, run one Teleop per pair until Ctrl-C (or any
pair stops), then return every arm to rest.
"""

from __future__ import annotations

import math
import threading
from pathlib import Path

from ..leaders import ArmLeader, StarLeader
from ..rig import ArmGroup
from ..stall import MotorStateRecorder, StallThresholds, default_stall_log_path
from ..teleop import Teleop, run_many
from .common import (CliError, Interrupt, achieved, add_for_arg, add_gravity_arg, add_yes_arg, apply_gravity_arg,
                     confirm, load_rig, make_parser, run, targets, wind_down)


def pairs(a) -> list[tuple[str, object, object]]:
    """(name, follower Arm, leader) per follower; arm leaders share one Arm object per bus."""
    rig = load_rig(a)
    followers = targets(a, many=True)
    if a.leader is None:
        a.leader = "star" if a.star_port else ("arm" if a.leader_arm else None)
    arms = {t.name: t.build() for t in followers}
    if a.leader == "star":
        ports = a.star_port or []
        if len(ports) != len(followers):
            raise CliError(f"{len(followers)} follower(s) but {len(ports)} --star-port")
        return [(t.name, arms[t.name], StarLeader(p, mapping=a.map)) for t, p in zip(followers, ports)]
    if a.leader == "arm":
        names = a.leader_arm or []
        if len(names) != len(followers):
            raise CliError(f"{len(followers)} follower(s) but {len(names)} --leader-arm")
        out = []
        for t, name in zip(followers, names):
            if name in arms:
                raise CliError(f"arm {name!r} cannot lead and follow at once")
            out.append((t.name, arms[t.name], ArmLeader(rig.arm(name), follower=arms[t.name])))
        return out
    for t in followers:
        spec = rig.leaders.get(t.name)
        if spec is not None and spec.type == "arm" and spec.arm in arms:
            raise CliError(f"arm {spec.arm!r} cannot lead and follow at once")
    return [(t.name, arms[t.name], rig.leader(t.name, arms[t.name])) for t in followers]


def recorder_for(a, name: str, arm, many: bool):
    if a.record_motor_states is None:
        return None
    path = (Path(a.record_motor_states) if a.record_motor_states
            else default_stall_log_path(a.payload_kg, a.telemetry_dir))
    if many:
        path = path.with_name(f"{path.stem}_{name}{path.suffix}")
    thresholds = StallThresholds(error_rad=a.stall_error, velocity_rad_s=a.stall_velocity, torque_nm=a.stall_torque,
                                 persistence_s=a.stall_seconds,
                                 max_feedback_age_s=min(arm.config.feedback_timeout, 0.15))
    return MotorStateRecorder(path, a.payload_kg, arm, thresholds)


def check_args(a) -> None:
    if not math.isfinite(a.payload_kg) or a.payload_kg < 0:
        raise CliError("--payload-kg must be finite and non-negative")
    if not all(math.isfinite(v) and v > 0 for v in (a.stall_error, a.stall_velocity, a.stall_torque, a.stall_seconds)):
        raise CliError("stall thresholds must be finite and greater than zero")
    for flag, value in (("--rate", a.rate), ("--max-velocity", a.max_velocity), ("--control-rate", a.control_rate)):
        if value is not None and not (math.isfinite(value) and value > 0):
            raise CliError(f"{flag} must be finite and greater than zero")


def body(a, out) -> dict:
    check_args(a)
    chosen = pairs(a)
    followers = ArmGroup({name: arm for name, arm, _ in chosen})
    for _, arm, _ in chosen:
        apply_gravity_arg(arm, a, out.say)
        if a.control_rate:
            arm.config.control_rate_hz = a.control_rate
        if a.max_velocity:
            arm.config.max_velocity = a.max_velocity
    rate = a.rate or min(100.0, min(arm.config.control_rate_hz for _, arm, _ in chosen))
    for name, _, leader in chosen:
        source = getattr(leader, "port", None) or "the floated leader arm"
        out.say(f"{name}: follows the {leader.name} leader on {source}")
    until = f"for {a.seconds:g} s" if a.seconds is not None else "until Ctrl-C"
    arm_leaders = [leader.arm for _, _, leader in chosen if isinstance(leader, ArmLeader)]
    out.say(f"will enable the followers{' and leader arms' if arm_leaders else ''} and follow at "
            f"{rate:g} Hz {until}; a large startup gap is closed slowly; then every arm returns to rest")
    confirm(a, out, "start teleoperation?")

    recorders, leaders = {}, [leader for _, _, leader in chosen]
    reasons = ["not started"] * len(chosen)
    timer = None
    with Interrupt() as stop:
        try:
            for leader in leaders:
                if stop.event.is_set():
                    raise CliError("interrupted")
                leader.connect(cancel=stop.event)      # an arm leader is enabled and floated here
            if stop.event.is_set():
                raise CliError("interrupted")
            followers.connect()
            followers.enable(cancel=stop.event)
            loops = []
            for name, arm, leader in chosen:
                recorders[name] = recorder_for(a, name, arm, len(chosen) > 1)
                loops.append(Teleop(leader, arm, rate_hz=rate, sync_threshold=a.sync_threshold,
                                    stall=recorders[name]))
            out.say("teleoperating" + ("" if a.seconds is not None else "; Ctrl-C to stop"))
            if a.seconds is not None:
                timer = threading.Timer(a.seconds, stop.event.set)
                timer.daemon = True
                timer.start()
            reasons = run_many(loops, stop.event)
        finally:
            if timer is not None:
                timer.cancel()
            for leader in leaders:
                leader.close()                         # an arm leader stiffens where the hand left it
            outcome = wind_down(followers, "rest", out)
            leader_group = ArmGroup({f"leader {i}": arm for i, arm in enumerate(arm_leaders, 1)})
            outcome.update(wind_down(leader_group, "rest", out))
            for (name, _, _), reason in zip(chosen, reasons):
                rec = recorders.get(name)
                if rec is not None:
                    out.say(f"{name}: motor-state CSV {rec.path}, stall summary {rec.close(reason)}")
    for (name, _, _), reason in zip(chosen, reasons):
        out.say(f"{name}: stopped ({reason})")
    stopped = {name: reason for (name, _, _), reason in zip(chosen, reasons)}
    ok = all(r == "stopped" for r in stopped.values()) and achieved(outcome, "rest")
    return {"ok": ok, "stopped": stopped, "then": outcome}


def main():
    ap = make_parser(__doc__, arms="many")
    ap.add_argument("--leader", choices=("star", "arm"), help="leader type (default: each arm's rig leader)")
    ap.add_argument("--star-port", action="append", help="Star leader serial port (repeat per follower)")
    ap.add_argument("--map", help="Star mapping JSON (default: the shipped one)")
    ap.add_argument("--leader-arm", action="append", metavar="NAME",
                    help="rig arm used as leader (repeat per follower)")
    ap.add_argument("--rate", type=float, help="leader reads per second (default: 100, 25 on serial SLCAN)")
    ap.add_argument("--control-rate", type=float, help="follower control-loop rate, Hz")
    ap.add_argument("--max-velocity", type=float, help="follower velocity limit, rad/s (low for first-motion checks)")
    ap.add_argument("--sync-threshold", type=float, default=0.8,
                    help="startup gap (rad) above which the follower closes it slowly")
    ap.add_argument("--record-motor-states", nargs="?", const="", metavar="CSV",
                    help="record every motor state and stop on a stall; optionally the CSV path")
    ap.add_argument("--payload-kg", type=float, default=0.0, help="payload mass stored with telemetry")
    ap.add_argument("--telemetry-dir", default="logs", help="directory for automatically named telemetry files")
    ap.add_argument("--stall-error", type=float, default=0.12, help="stall: min |command-position|, rad")
    ap.add_argument("--stall-velocity", type=float, default=0.04, help="stall: max |velocity|, rad/s")
    ap.add_argument("--stall-torque", type=float, default=0.5, help="stall: min |torque|, N*m")
    ap.add_argument("--stall-seconds", type=float, default=0.5, help="stall: persistence before it counts, s")
    add_for_arg(ap, "follow")
    add_gravity_arg(ap)
    add_yes_arg(ap)
    run("teleop", ap.parse_args(), body)


if __name__ == "__main__":
    main()
