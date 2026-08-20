"""Top-level ``maker-arm`` command dispatcher."""

from __future__ import annotations

import importlib
import sys


COMMANDS = {
    "doctor": ("maker_arm.cli.doctor", "inspect the computer and installed profiles"),
    "scan": ("maker_arm.cli.scan", "scan the follower CAN bus"),
    "assign-id": ("maker_arm.cli.assign_id", "assign an ID to one replacement motor"),
    "zero": ("maker_arm.cli.zero", "zero one replacement motor or the full arm"),
    "check": ("maker_arm.cli.check", "run a bounded powered motor response check"),
    "teleop": ("maker_arm.cli.teleop", "teleoperate from the Star leader"),
}


def _usage() -> str:
    lines = ["usage: maker-arm <command> [options]", "", "commands:"]
    lines.extend(f"  {name:<12} {description}" for name, (_, description) in COMMANDS.items())
    lines.extend(["", "Run 'maker-arm <command> --help' for command-specific options."])
    return "\n".join(lines)


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] in {"-h", "--help"}:
        print(_usage())
        return
    command = sys.argv[1]
    target = COMMANDS.get(command)
    if target is None:
        raise SystemExit(f"unknown command {command!r}\n\n{_usage()}")
    sys.argv = [f"maker-arm {command}", *sys.argv[2:]]
    importlib.import_module(target[0]).main()


if __name__ == "__main__":
    main()
