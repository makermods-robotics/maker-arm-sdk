"""Torque-release safeguards for powered commands: return to rest, else hold until the operator says so.

Outcomes: ``rest`` (verified return, then released), ``released`` (torque off where it is),
``holding`` (process exited, torque still on), ``stay`` (left on as asked), ``unconfirmed``
(torque off sent but not confirmed on every motor).
"""

from __future__ import annotations

import sys
import time
from typing import Callable

from .._product import CLI_NAME


def wait_for_release(say: Callable = print) -> None:
    say("\narm is holding. Type RELEASE and press ENTER only when it is safe to release torque:")
    eof_reported = False
    while True:
        try:
            if input("").strip().upper() == "RELEASE":   # no prompt text: stdout may carry JSON
                return
            say("torque remains enabled; type RELEASE only when the arm is supported")
        except KeyboardInterrupt:
            say("\ntorque remains enabled; type RELEASE when safe")
        except EOFError:
            if not eof_reported:
                say("input is unavailable; torque remains enabled while this process is alive")
                eof_reported = True
            time.sleep(1.0)


def released(arm, outcome: str = "released") -> str:
    """``outcome`` once torque off was confirmed on every motor, else ``unconfirmed``."""
    return "unconfirmed" if arm.unconfirmed_disable else outcome


def detached_text(arm, releasing: dict) -> str:
    """What a detached arm does next: which joints release on their watchdog, which keep holding."""
    others = [n for n in arm.joint_names if n not in releasing]
    parts = [f"{n} releases after {s:g} s (motor watchdog)" for n, s in releasing.items()]
    if others:
        parts.append(f"{', '.join(others)} keep holding their last command")
    return "; ".join(parts)


def leave_holding(arm, say: Callable = print) -> str:
    """Keep an energized arm holding its present pose without releasing it: detach when no
    stiff joint would release on its watchdog, else keep this process holding until RELEASE."""
    arm.hold_current_position()
    unsafe = arm.detach_unsafe()
    if unsafe:
        say(f"{', '.join(unsafe)} would release on their motor watchdog if this process stopped "
            "commanding them; this process keeps holding the arm")
        wait_for_release(say)
        arm.disable()
        return released(arm)
    releasing = arm.detach()
    say(f"arm left holding with torque on: {detached_text(arm, releasing)}. Support it, then run "
        f"'{CLI_NAME} release'")
    return "holding"


def release_if_holding(arm, say: Callable = print) -> str:
    """Return to rest and release torque; returns the outcome.

    Only acts when this process actually enabled/holds the arm (else ``not_enabled``): a
    connection or pre-enable failure closes the transport without sending anything to motors
    another controller may hold. If the return to rest cannot be verified the arm keeps holding:
    at a terminal until the operator types RELEASE; without one, or after a second Ctrl-C, it is
    left holding (leave_holding()).
    """
    if arm.state.name not in ("ENABLED", "FAULT"):
        return "not_enabled"
    say("returning to the rest pose...")
    try:
        verified = arm.stop()
    except KeyboardInterrupt:
        say("return to rest interrupted; the arm is holding its pose")
        return leave_holding(arm, say)
    if verified:
        say("rest pose reached; torque released")
        return released(arm, "rest")
    say("could not verify the return to rest; the arm is holding its pose")
    if not sys.stdin.isatty():
        return leave_holding(arm, say)
    wait_for_release(say)
    arm.disable()
    return released(arm)
