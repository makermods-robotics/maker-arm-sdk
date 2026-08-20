"""Interactive safeguards shared by powered CLI commands and examples."""

from __future__ import annotations

import sys
import time


def require_interactive_terminal(operation: str) -> None:
    if not sys.stdin.isatty():
        raise SystemExit(
            f"{operation} requires an interactive terminal for safe torque-release confirmation"
        )


def wait_for_release() -> None:
    print("\n🔒 arm is holding. Type RELEASE and press ENTER only when it is safe to release torque.")
    eof_reported = False
    while True:
        try:
            if input("> ").strip().upper() == "RELEASE":
                return
            print("torque remains enabled; type RELEASE only when the arm is supported")
        except KeyboardInterrupt:
            print("\ntorque remains enabled; type RELEASE when safe")
        except EOFError:
            if not eof_reported:
                print("input is unavailable; torque remains enabled while this process is alive")
                eof_reported = True
            time.sleep(1.0)


def release_if_holding(arm) -> bool:
    """Ask before disabling only when this process actually enabled/holds the arm.

    A connection or pre-enable validation failure must close the transport without
    broadcasting a disable command to motors that may be held by another controller.
    """
    if arm.state.name not in ("ENABLED", "FAULT"):
        return False
    wait_for_release()
    arm.disable()
    return True
