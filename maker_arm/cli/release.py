#!/usr/bin/env python3
"""Torque off where it is: the arm drops unless it is at rest or supported. To bring an arm left
energized (`hold --then stay`, a crashed program) down first, use the rest command.

Each motor first gets a zero-gain command at its measured position (so a later enable cannot
resume an old setpoint), then a disable whose reply must confirm torque off. Works where rest
cannot connect (a motor fault, a missing motor).
"""

from __future__ import annotations

from .._product import CLI_NAME
from ..bus import release_motor  # noqa: F401
from .common import Bus, CliError, add_yes_arg, confirm, make_parser, run, targets


def body(a, out) -> dict:
    results, ok = {}, True
    for t in targets(a, many=True):
        with Bus(t.backend(), t.config()) as bus:
            rows = bus.read_all()
            if not rows:
                raise CliError(f"{t.name}: no motor answers on {t.spec.address}")
            on = [r["joint"] for r in rows if r["enabled"]]
            state = f"joints reporting torque on: {', '.join(on)}" if on else "no joint reports torque on"
            out.say(f"{t.name}: {state}. The arm DROPS when released unless it is at rest or supported; "
                    f"'{CLI_NAME} rest' brings an energized arm down first.")
            confirm(a, out, f"release torque on {t.name}?")
            result = bus.release(rows)
        unconfirmed, missing = result["unconfirmed"], bus.missing(rows)
        if unconfirmed:
            out.say(f"{t.name}: torque off NOT confirmed on {unconfirmed}; cut motor power if they still hold")
        else:
            out.say(f"{t.name}: torque off confirmed on {len(rows)} motors")
        if missing:
            out.say(f"{t.name}: no reply from motors {missing}")
        ok = ok and result["ok"]
        results[t.name] = {"released": len(rows) - len(unconfirmed), "unconfirmed": unconfirmed, "missing": missing}
    return {"ok": ok, "arms": results}


def main() -> None:
    ap = make_parser(__doc__, arms="many")
    add_yes_arg(ap)
    run("release", ap.parse_args(), body)


if __name__ == "__main__":
    main()
