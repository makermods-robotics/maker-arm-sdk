#!/usr/bin/env python3
"""Clear motor faults once their cause is fixed, then re-check each motor.

Only motors that report a fault are cleared; torque is never enabled. A fault that comes straight
back (still present after clear) has a cause that is still there: overtemperature, a stalled
joint, undervoltage.
"""

from __future__ import annotations

from .common import Bus, add_yes_arg, confirm, make_parser, run, targets


def body(a, out) -> dict:
    results, ok = {}, True
    for t in targets(a, many=True):
        with Bus(t.backend(), t.config()) as bus:
            before = bus.read_all()
            faulted = [r for r in before if r["fault"]]
            if faulted:
                listing = ", ".join(f"{r['joint'] or r['motor_id']} ({r['fault']})" for r in faulted)
                confirm(a, out, f"{t.name}: clear faults on {listing}?", destructive=False)
                after = [bus.read(r["motor_id"], clear=True) for r in faulted]
            else:
                after = []
            missing = bus.missing(before)
        remaining = [r["joint"] or r["motor_id"] for r in after if r is None or r["fault"]]
        ok = ok and not remaining and not missing
        cleared = [r["joint"] or r["motor_id"] for r in after if r is not None and not r["fault"]]
        out.say(f"{t.name}: " + ("no faults" if not faulted else
                                 f"cleared {', '.join(map(str, cleared)) or 'none'}"
                                 + (f"; still faulted: {', '.join(map(str, remaining))}" if remaining else ""))
                + (f"; no reply from motors {missing}" if missing else ""))
        results[t.name] = {"cleared": cleared, "still_faulted": remaining, "missing": missing}
    return {"ok": ok, "arms": results}


def main() -> None:
    ap = make_parser(__doc__, arms="many")
    add_yes_arg(ap)
    run("clear-faults", ap.parse_args(), body)


if __name__ == "__main__":
    main()
