#!/usr/bin/env python3
"""Change a motor's CAN ID (persistent), with torque off. Only that one motor should answer on
FROM; the command refuses when TO already answers. Update the profile afterwards.
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
            f"and survives power cycles; any other motor on ID {a.old} would change too.")
    confirm(a, out, f"change motor ID {a.old} -> {a.new}?")
    arm.set_motor_id(a.old, a.new)
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
