#!/usr/bin/env python3
"""Enable each arm at its current pose and hold it there until Ctrl-C or SIGTERM (or --for S).

Afterwards (--then): rest = verified return to the pose it was enabled at, then torque off
(default); stay = exit with torque on; release = torque off where it is. A fault ends the hold
early; the faulted arm keeps holding its pose until --then runs.
"""

from __future__ import annotations

from .common import (Interrupt, achieved, add_for_arg, add_gravity_arg, add_then_arg, add_yes_arg,
                     apply_gravity_arg, check_stay, confirm, group_of, make_parser, not_enabled, run, targets,
                     wind_down)


def body(a, out) -> dict:
    chosen = targets(a, many=True)
    group = group_of(chosen)
    for _, arm in group:
        apply_gravity_arg(arm, a, out.say)
    check_stay(group.arms.values(), a.then)
    names = ", ".join(t.label for t in chosen)
    until = f"for {a.seconds:g} s" if a.seconds is not None else "until Ctrl-C"
    out.say(f"will enable every joint of {names} at its current pose and hold {until}, then: {a.then}")
    confirm(a, out, "energize the motors?")
    reason = "not started"
    with Interrupt() as stop:   # Ctrl-C ends the hold; it does not interrupt the return to rest
        try:
            group.connect()
            group.enable(cancel=stop.event)
            out.say("holding" + ("" if a.seconds is not None else "; Ctrl-C to finish"))
            reason = stop.wait(a.seconds, stop_if=lambda: not_enabled(group.arms))
        finally:
            faults = {n: arm.fault_reason for n, arm in group if arm.fault_reason}
            outcome = wind_down(group, a.then, out)
    if faults:
        out.say("faults: " + "; ".join(f"{n}: {r}" for n, r in faults.items()))
    return {"ok": not faults and achieved(outcome, a.then), "stopped_by": reason, "then": outcome, "faults": faults}


def main() -> None:
    ap = make_parser(__doc__, arms="many")
    add_for_arg(ap)
    add_then_arg(ap, "rest")
    add_gravity_arg(ap)
    add_yes_arg(ap)
    run("hold", ap.parse_args(), body)


if __name__ == "__main__":
    main()
