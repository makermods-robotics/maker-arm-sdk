#!/usr/bin/env python3
"""Motor parameters (engineering use).

  params list                                   the parameters this motor type exposes (no bus traffic)
  params get NAME --motor ID
  params set NAME VALUE --motor ID [--save]     write, read back; --save also stores it in flash
"""

from __future__ import annotations

from ..motor import Motor
from .common import Bus, CliError, add_yes_arg, confirm, make_parser, run, target


def body(a, out) -> dict:
    if a.action == "list":
        for name, spec in Motor.PARAMS.items():
            out.say(f"{name:<20} 0x{spec.index:04X} {spec.dtype:<4} {spec.doc}")
        return {"params": {n: {"index": s.index, "dtype": s.dtype, "doc": s.doc} for n, s in Motor.PARAMS.items()}}
    if a.motor is None:
        raise CliError(f"params {a.action} needs --motor ID")
    if a.name not in Motor.PARAMS:
        raise CliError(f"unknown parameter {a.name!r}; see 'params list'")
    spec = Motor.PARAMS[a.name]
    t = target(a)
    with Bus(t.backend(), t.config(), ids=[a.motor]) as bus:
        m = bus.motors[a.motor]
        if a.action == "get":
            value = m.read_param(a.name)
            out.say(f"motor {a.motor} {a.name} = {value}")
            return {"arm": t.name, "motor": a.motor, "name": a.name, "value": value}
        if a.value is None:
            raise CliError("params set needs a VALUE")
        value = float(a.value) if spec.dtype == "f" else int(a.value, 0)
        old = m.read_param(a.name)
        out.say(f"motor {a.motor} {a.name}: {old} -> {value}" + (" (saved to flash)" if a.save else ""))
        confirm(a, out, f"write {a.name} on motor {a.motor}?")
        readback = m.write_param(a.name, value, save=a.save)
        out.say(f"read back: {readback}")
        return {"ok": readback == value or (spec.dtype == "f" and abs(readback - value) < 1e-6 * max(1.0, abs(value))),
                "arm": t.name, "motor": a.motor, "name": a.name, "old": old, "value": readback, "saved": a.save}


def main() -> None:
    ap = make_parser(__doc__, arms="one")
    ap.add_argument("action", choices=("list", "get", "set"))
    ap.add_argument("name", nargs="?")
    ap.add_argument("value", nargs="?")
    ap.add_argument("--motor", type=int, help="motor CAN ID")
    ap.add_argument("--save", action="store_true", help="set: also store the value in the motor's flash")
    add_yes_arg(ap)
    run("params", ap.parse_args(), body)


if __name__ == "__main__":
    main()
