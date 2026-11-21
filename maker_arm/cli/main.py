"""Top-level command dispatcher."""

from __future__ import annotations

import importlib
import sys
from typing import Optional, Sequence

from .._product import CLI_NAME

COMMANDS = {
    "doctor": ("doctor", "inspect the computer, profiles and rig file (no bus traffic)"),
    "ports": ("ports", "list CAN adapters, cameras and serial ports (no bus traffic)"),
    "rig": ("rig", "show or edit the rig file: arms, cameras, leaders"),
    "identify": ("identify", "wiggle each arm's gripper to tell which arm is which"),
    "scan": ("scan", "find motors: position, temperature, faults (never enables)"),
    "status": ("status", "per-joint state, once or live (read-only)"),
    "clear-faults": ("clear_faults", "clear motor faults once the cause is fixed"),
    "hold": ("hold", "enable at the current pose and hold until Ctrl-C"),
    "rest": ("rest", "bring an arm to rest and release it, also one left energized"),
    "release": ("release", "torque off where it is (the arm drops); verified"),
    "move": ("move", "move joints, to a pose, or the tool point in Cartesian space"),
    "fk": ("fk", "tool pose for the current or given joints (no motion)"),
    "ik": ("ik", "joints for a tool pose (no motion)"),
    "gravity": ("gravity", "float the arm for hand-guiding, or check the gravity model"),
    "check": ("check", "bounded powered response check, or sine tracking test"),
    "zero": ("zero", "set the zero pose (follower, optionally the Star leader)"),
    "set-id": ("set_id", "change a motor's CAN ID"),
    "params": ("params", "list, read or write motor parameters"),
    "teleop": ("teleop", "teleoperate from a Star leader or a second arm"),
    "leader": ("leader", "Star leader: set its zero, or print its mapped targets"),
    "cameras": ("cameras", "list cameras, save snapshots, measure frame rate"),
}


def _usage() -> str:
    lines = [f"usage: {CLI_NAME} <command> [options]", "", "commands:"]
    lines.extend(f"  {name:<13} {description}" for name, (_, description) in COMMANDS.items())
    lines.extend(["", "Every command takes --json. Arms are chosen with --arm NAME (from the rig file) or",
                  "--backend/--channel/--port.", f"Run '{CLI_NAME} <command> --help' for its options."])
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in {"-h", "--help"}:
        print(_usage())
        return
    command = args[0]
    target = COMMANDS.get(command)
    if target is None:
        raise SystemExit(f"unknown command {command!r}\n\n{_usage()}")
    sys.argv = [f"{CLI_NAME} {command}", *args[1:]]
    importlib.import_module(f".{target[0]}", __package__).main()


if __name__ == "__main__":
    main()
