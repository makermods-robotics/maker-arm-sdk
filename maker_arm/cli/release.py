#!/usr/bin/env python3
"""Torque off, verified, for an arm left energized (by `hold --then stay`, a crashed program, ...).

Each motor first gets a zero-gain command at its measured position (so a later enable cannot
resume an old setpoint), then a disable whose reply must confirm torque off. The arm drops if it
is not resting on something: support it first.
"""

from __future__ import annotations

import time

from .common import Bus, CliError, add_yes_arg, confirm, make_parser, run, targets

ATTEMPTS = 3
REPLY_S = 0.05


def release_motor(m) -> bool:
    fb = m.feedback
    if fb is not None:
        m.send_command(fb.position, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.005)
    return m.disable_confirmed(REPLY_S, ATTEMPTS)


def body(a, out) -> dict:
    results, ok = {}, True
    for t in targets(a, many=True):
        with Bus(t.backend(), t.config()) as bus:
            rows = bus.read_all()
            if not rows:
                raise CliError(f"{t.name}: no motor answers on {t.spec.address}")
            on = [r["joint"] for r in rows if r["enabled"]]
            state = f"joints reporting torque on: {', '.join(on)}" if on else "no joint reports torque on"
            out.say(f"{t.name}: {state}. The arm DROPS when released unless it is at rest or supported.")
            confirm(a, out, f"release torque on {t.name}?")
            unconfirmed = [r["joint"] or r["motor_id"] for r in rows if not release_motor(bus.motors[r["motor_id"]])]
            missing = bus.missing(rows)
        if unconfirmed:
            out.say(f"{t.name}: torque off NOT confirmed on {unconfirmed}; cut motor power if they still hold")
        else:
            out.say(f"{t.name}: torque off confirmed on {len(rows)} motors")
        if missing:
            out.say(f"{t.name}: no reply from motors {missing}")
        ok = ok and not unconfirmed and not missing
        results[t.name] = {"released": len(rows) - len(unconfirmed), "unconfirmed": unconfirmed, "missing": missing}
    return {"ok": ok, "arms": results}


def main() -> None:
    ap = make_parser(__doc__, arms="many")
    add_yes_arg(ap)
    run("release", ap.parse_args(), body)


if __name__ == "__main__":
    main()
