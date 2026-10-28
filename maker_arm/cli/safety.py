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
    """Return to rest and release torque, as MakerModsLab's stop path does.

    Only acts when this process actually enabled/holds the arm: a connection or pre-enable
    failure closes the transport without sending anything to motors another controller may
    hold. If the return to rest cannot be verified, the arm keeps holding until the operator
    types RELEASE.
    """
    if arm.state.name not in ("ENABLED", "FAULT"):
        return False
    print("returning to the rest pose...")
    if arm.stop():
        print("rest pose reached; torque released")
        return True
    print("could not verify the return to rest; the arm is holding its pose")
    wait_for_release()
    arm.disable()
    return True
