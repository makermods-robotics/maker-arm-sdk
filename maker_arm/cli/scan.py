#!/usr/bin/env python3
"""Find motors on each arm's bus and show position, temperature and fault status.

Never enables or releases torque. Each ID first gets a non-clearing fault query; only
fault-free motors are then asked for their position, so fault evidence is kept unless you
pass --clear-faults.
"""

from __future__ import annotations

from ..motor import ID_RANGE
from .common import Bus, CliError, deg, make_parser, num, run, targets


def scan(t, max_id: int, clear: bool) -> dict:
    config = t.config()
    with Bus(t.backend(), config, ids=list(range(ID_RANGE[0], max_id + 1))) as bus:
        rows = bus.read_all(clear)
        missing = bus.missing(rows)
    return {"name": t.name, "backend": t.spec.backend, "address": t.spec.address, "motors": rows, "missing": missing}


def show(result: dict, say) -> None:
    say(f"{result['name']} ({result['backend']} {result['address']}):")
    if not result["motors"]:
        say("  no motors found -- check power, wiring, termination, 1 Mbps, that the motors use the MIT "
            "protocol, and that no other program holds the adapter")
        return
    say("   ID joint            position_deg  temp_C  enabled  status")
    for r in result["motors"]:
        enabled = "-" if r["enabled"] is None else ("yes" if r["enabled"] else "no")
        if r["fault"]:
            text = f"FAULT 0x{r['fault_code']:08X}: {r['fault']}"
            if r["cleared"]:
                text += " (still present after clear)"
        else:
            text = "ok (fault cleared)" if r["cleared"] else "ok"
            if r["outside_limits"]:
                text += " -- outside limits, re-zero needed"
        say(f"  {r['motor_id']:3} {r['joint'] or '(not in profile)':<16} {deg(r['position'], 12)}  "
            f"{num(r['temperature'])}  {enabled:>7}  {text}")
    if result["missing"]:
        say(f"  in the profile but not found: {result['missing']}")


def body(a, out) -> dict:
    lo, hi = ID_RANGE
    if not lo <= a.max_id <= hi:
        raise CliError(f"--max-id must be in [{lo}, {hi}]")
    results = []
    for t in targets(a, many=True):
        result = scan(t, a.max_id, a.clear_faults)
        show(result, out.say)
        results.append(result)
    found = all(r["motors"] for r in results)
    return {"ok": found, "arms": results, "units": {"position": "rad (joint)", "temperature": "C"}}


def main():
    ap = make_parser(__doc__, arms="many")
    ap.add_argument("--max-id", type=int, default=7)
    ap.add_argument("--clear-faults", action="store_true",
                    help="clear faults on motors that report one (only once the cause is fixed)")
    run("scan", ap.parse_args(), body)


if __name__ == "__main__":
    main()
