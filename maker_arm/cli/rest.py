#!/usr/bin/env python3
"""Bring one arm to a supported pose and release it, whether its torque is off or it was left
energized (`hold --then stay`, a crashed program).

  rest                                   to the profile's rest_pose, else the zero pose clipped into limits
  rest --pose zero | --pose NAME=DEG,... other joints stay where they are

An arm whose motors report torque on is taken over where it holds (attach: no sag); otherwise it
is enabled at its present pose. It then moves at --speed and, once arrival is verified, torque goes
off. Ctrl-C or SIGTERM stops the motion and the arm keeps holding; so does an arrival that cannot
be verified.
"""

from __future__ import annotations

import math

from .common import CliError, Interrupt, add_yes_arg, confirm, make_parser, pose_text, rest_command, run, target
from .move import pose_goal
from .safety import detached_text, leave_holding, release_if_holding


def body(a, out) -> dict:
    if a.speed is not None and not a.speed > 0:
        raise CliError("--speed must be > 0 deg/s")
    t = target(a)
    arm = t.build()
    names = arm.joint_names
    speed = math.radians(a.speed) if a.speed else arm.config.rest_speed
    result: dict = {"arm": t.name, "pose": a.pose}
    with Interrupt() as stop:
        try:
            arm.connect(clear_released=True)
            arm.refresh()   # torque state as the firmware probed at connect reports it
            start = arm.get_joint_positions()
            torque = [m.feedback.enabled for m in arm.motors]
            attach = any(on is not False for on in torque)
            goal = pose_goal(a.pose, arm.config, start)
            held = [n for n, on in zip(names, torque) if on]
            how = (f"holding with torque on ({', '.join(held)}): taking it over where it is" if any(torque) else
                   "torque state not reported: taking it over where it is" if attach else
                   "torque off: enabling it where it is")
            out.say(f"{t.label}: {how}\n  from {pose_text(names, start)}\n  to   {pose_text(names, goal)}")
            out.say(f"speed {math.degrees(speed):.0f} deg/s, then torque off once there")
            confirm(a, out, "move the arm to rest and release it?")
            result["attached"] = attach
            (arm.attach if attach else arm.enable)(cancel=stop.event, speed=speed)
            reached = arm.move_to(goal, speed=speed, cancel=stop.event)
            final = arm.get_joint_positions()
            out.say(("reached" if reached else "NOT reached") + f": {pose_text(names, final)}")
            result.update(reached=reached, goal=dict(zip(names, goal)), final=dict(zip(names, final)))
            if stop.event.is_set():
                out.say("stopped; the arm is holding its pose")
                result["then"] = leave_holding(arm, out.say)
            else:
                arm.rest_pose = goal   # what stop() verifies before torque goes off
                result["then"] = release_if_holding(arm, say=out.say)
        finally:
            if arm.state.name in ("ENABLED", "FAULT"):
                releasing = arm.detach()
                out.warn(f"{t.name}: left holding with torque on: {detached_text(arm, releasing)}. "
                         f"Run '{rest_command(t.name)}' to bring it down")
                result["then"] = "holding"
            else:
                arm.disconnect()
    result["fault"] = arm.fault_reason
    result["ok"] = result.get("then") == "rest" and not arm.fault_reason
    return result


def main() -> None:
    ap = make_parser(__doc__, arms="one")
    ap.add_argument("--pose", default="rest", help="rest (default), zero, or NAME=DEG,NAME=DEG,...")
    ap.add_argument("--speed", type=float, metavar="DEG_S",
                    help="per-joint speed cap, deg/s (default: the profile's rest_speed)")
    add_yes_arg(ap)
    run("rest", ap.parse_args(), body)


if __name__ == "__main__":
    main()
