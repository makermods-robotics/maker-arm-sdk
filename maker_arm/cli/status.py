#!/usr/bin/env python3
"""Per-joint state of each arm: position, velocity, torque, temperatures, enabled, fault.

Read-only: fault queries and state reads, never a torque change, so it also works on an arm left
energized or reporting a fault. --watch repeats at HZ (default 2) until Ctrl-C; with --json each
refresh is one JSON line.
"""

from __future__ import annotations

import json
import time
from contextlib import ExitStack

from .common import UNITS, Bus, Interrupt, clean, deg, make_parser, num, run, targets


def show(name: str, rows: list[dict], missing: list[int], say) -> None:
    say(f"{name}:")
    say("  joint            pos_deg  vel_dps  torque_Nm  temp_C  enabled  fault")
    for r in rows:
        enabled = "-" if r["enabled"] is None else ("yes" if r["enabled"] else "no")
        temps = "/".join(f"{v:.0f}" for v in r["temperatures"].values()) or num(r["temperature"]).strip()
        say(f"  {r['joint'] or r['motor_id']:<14} {deg(r['position'])} {deg(r['velocity'])} "
            f"{num(r['torque'], 10, 2)}  {temps:>6}  {enabled:>7}  {r['fault'] or ''}")
    if missing:
        say(f"  no reply from motors {missing}")


def body(a, out) -> dict:
    chosen = targets(a, many=True)
    period = 1.0 / a.watch if a.watch else 0.0
    with ExitStack() as stack:
        buses = {t.name: stack.enter_context(Bus(t.backend(), t.config())) for t in chosen}
        stop = stack.enter_context(Interrupt())
        count = 0
        while True:
            t0 = time.monotonic()
            snapshot = {}
            for name, bus in buses.items():
                rows = bus.read_all()
                snapshot[name] = {"joints": rows, "missing": bus.missing(rows)}
            count += 1
            if a.watch and out.json:
                print(json.dumps(clean({"time": time.time(), "arms": snapshot})), flush=True)
            elif not out.json:
                for name, snap in snapshot.items():
                    show(name, snap["joints"], snap["missing"], out.say)
            if not a.watch or (a.count and count >= a.count) or stop.event.is_set():
                break
            if stop.wait(max(0.0, period - (time.monotonic() - t0))) == "interrupted":
                break
    if a.watch and out.json:
        raise SystemExit(0 if all(not s["missing"] for s in snapshot.values()) else 1)
    ok = all(not s["missing"] for s in snapshot.values())
    return {"ok": ok, "arms": snapshot, "units": UNITS}


def main() -> None:
    ap = make_parser(__doc__, arms="many")
    ap.add_argument("--watch", type=float, nargs="?", const=2.0, metavar="HZ", help="refresh at HZ until Ctrl-C")
    ap.add_argument("--count", type=int, help="with --watch: stop after this many refreshes")
    run("status", ap.parse_args(), body)


if __name__ == "__main__":
    main()
