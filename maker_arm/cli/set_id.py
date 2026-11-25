#!/usr/bin/env python3
"""Change a motor's CAN ID (persistent), with torque off. Only that one motor should answer on
FROM, and it must be de-energized; the command refuses when TO already answers or belongs to another
joint, and when the motor reports torque on, holds torque or moves. Update the profile afterwards.
"""

from __future__ import annotations

from .._product import CLI_NAME
from ..motor import ID_RANGE
from .common import CliError, add_yes_arg, confirm, make_parser, run, target


def body(a, out) -> dict:
    lo, hi = ID_RANGE
    if not (lo <= a.old <= hi and lo <= a.new <= hi) or a.old == a.new:
        raise CliError(f"--from and --to must differ and lie in [{lo}, {hi}]")
    t = target(a)
    arm = t.build()
    out.say(f"{t.label}: change the motor answering on ID {a.old} to ID {a.new}. It is written to the motor "
            f"and survives power cycles; any other motor on ID {a.old} would change too. The motor must be "
            f"de-energized (torque off, no other program driving it): some firmware cannot report it.")
    confirm(a, out, f"motor {a.old} is de-energized: change its ID to {a.new}?")
    arm.set_motor_id(a.old, a.new, torque_off_confirmed=True)
    out.say(f"motor now answers on ID {a.new}. If it does not, power-cycle it. Then update motor_id in the "
            f"profile and run '{CLI_NAME} scan' to check.")
    return {"arm": t.name, "from": a.old, "to": a.new}


def main() -> None:
    ap = make_parser(__doc__, arms="one")
    ap.add_argument("--from", dest="old", type=int, required=True, metavar="A", help="present motor ID")
    ap.add_argument("--to", dest="new", type=int, required=True, metavar="B", help="new motor ID")
    add_yes_arg(ap)
    run("set-id", ap.parse_args(), body)


if __name__ == "__main__":
    main()
