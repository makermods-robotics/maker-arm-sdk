"""What every command shares: arm selection (rig names or explicit bus flags), --json output,
confirmation gates, Ctrl-C/SIGTERM handling, and read-only motor access.

Arm selection, in order: ``--arm NAME`` (repeatable, ``all`` = every arm in the rig file); else
explicit ``--backend/--channel/--port`` for one unnamed arm; else the only arm in the rig file;
else the platform default bus (SocketCAN can0 on Linux, the only USB-SLCAN adapter on macOS).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from .._product import CLI_NAME
from ..arm import Arm
from ..config import ArmConfig, JointConfig
from ..errors import ArmError, FaultLatched, ParamTimeout
from ..gravity import MAX_GRAVITY_SCALE
from ..leaders.base import LeaderError
from ..motor import Motor, route
from ..rig import BACKENDS, ArmGroup, ArmSpec, Rig, default_backend
from ..transport.base import CanBackend
from .safety import detached_text, release_if_holding, released

UNITS = {"angle": "rad", "velocity": "rad/s", "torque": "N*m", "temperature": "C", "length": "m"}
log = logging.getLogger(__name__)


class CliError(Exception):
    """A user-facing failure: printed (or put in the JSON document) and exit status 1."""


# ── output ──
class Out:
    """Human text on stdout, or (``--json``) one JSON document on stdout and the text on stderr."""

    def __init__(self, as_json: bool = False):
        self.json = as_json

    def say(self, *parts) -> None:
        print(*parts, file=sys.stderr if self.json else sys.stdout, flush=True)

    @staticmethod
    def warn(*parts) -> None:
        print(*parts, file=sys.stderr, flush=True)


def clean(value):
    """JSON-safe copy: NaN/inf become None, tuples lists, paths strings."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "tolist"):
        return clean(value.tolist())
    return value


ERRORS = (CliError, ArmError, LeaderError, ValueError, LookupError, OSError, ImportError, TimeoutError)


def run(command: str, a, body: Callable[[argparse.Namespace, Out], Optional[dict]]) -> None:
    """Run a command body: ``body(a, out)`` returns the JSON document (``ok`` defaults to True).
    Failures print a message (or ``{"ok": false, "error": ...}``) and exit 1; Ctrl-C exits 130."""
    logging.basicConfig(level=logging.INFO if getattr(a, "verbose", False) else logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s", stream=sys.stderr)
    out = Out(getattr(a, "json", False))
    code, error, doc = 0, None, {}
    try:
        doc = body(a, out) or {}
    except KeyboardInterrupt:
        code, error = 130, "interrupted"
    except SystemExit as e:
        if e.code is None or isinstance(e.code, int):
            raise
        code, error = 1, str(e.code)
    except ERRORS as e:
        if getattr(a, "verbose", False):
            log.exception("failed")
        code, error = 1, str(e) or type(e).__name__
    ok = error is None and doc.pop("ok", True)
    if out.json:
        result = {"command": command, "ok": bool(ok)}
        if error is not None:
            result["error"] = error
        result.update(doc)
        print(json.dumps(clean(result), allow_nan=False))
    elif error is not None:
        print(f"{CLI_NAME} {command}: {error}", file=sys.stderr)
    if error is not None or not ok:
        raise SystemExit(code or 1)


# ── parsers ──
def make_parser(desc: str, arms: Optional[str] = "one") -> argparse.ArgumentParser:
    """``arms``: "one", "many" (``--arm`` repeatable) or None (no arm selection)."""
    # No abbreviations: --yes must never pass for --yes-i-am-at-the-zero-pose.
    ap = argparse.ArgumentParser(description=desc, formatter_class=argparse.RawDescriptionHelpFormatter,
                                 allow_abbrev=False)
    ap.add_argument("--json", action="store_true", help="print one JSON document on stdout (text goes to stderr)")
    ap.add_argument("-v", "--verbose", action="store_true", help="log progress to stderr")
    if arms:
        add_arm_args(ap, many=arms == "many")
    return ap


def add_rig_arg(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--rig", help="rig file (default: $%s_RIG or ~/.config/%s/rig.yaml)"
                    % (CLI_NAME.upper().replace("-", "_"), CLI_NAME))


def add_arm_args(ap: argparse.ArgumentParser, many: bool = False) -> None:
    add_rig_arg(ap)
    ap.add_argument("--arm", action="append", metavar="NAME",
                    help="arm name from the rig file" + ("; repeat for several, 'all' for every arm" if many else ""))
    ap.add_argument("--config", help="arm profile YAML (default: the rig entry's, else the shipped one)")
    add_backend_args(ap)


def add_backend_args(ap: argparse.ArgumentParser) -> None:
    """Bus flags; left None when not given, so an explicit choice can be told apart from the rig file."""
    ap.add_argument("--backend", choices=BACKENDS, help=f"CAN backend (default: {default_backend()})")
    ap.add_argument("--channel", help="SocketCAN interface (default: can0)")
    ap.add_argument("--port", help="SLCAN serial device, or gs_usb:<serial> (default: the only adapter found)")
    ap.add_argument("--serial-baudrate", type=int, help="SLCAN serial baud rate (default: 115200)")
    ap.add_argument("--slcan-startup-delay", type=float,
                    help="seconds to wait after opening an SLCAN serial device (default: 2.5)")


def add_yes_arg(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("-y", "--yes", action="store_true",
                    help="do not ask for confirmation (required when not on a terminal)")


def add_then_arg(ap: argparse.ArgumentParser, default: str, choices=("rest", "stay", "release")) -> None:
    ap.add_argument("--then", choices=choices, default=default,
                    help="afterwards: rest = verified return to rest, then release; stay = exit leaving "
                         "torque on (a joint with a motor watchdog, the gripper, releases once it times out; "
                         "release the rest later with the release command); release = torque off where it is"
                         + ("; hold = keep holding until Ctrl-C, then rest" if "hold" in choices else ""))


def add_for_arg(ap: argparse.ArgumentParser, what: str = "hold") -> None:
    ap.add_argument("--for", dest="seconds", type=float, metavar="S",
                    help=f"{what} for S seconds instead of until Ctrl-C")


# ── arm selection ──
@dataclass
class Target:
    """One selected arm: its rig name ("arm" when unnamed) and how to reach it."""
    name: str
    spec: ArmSpec
    base: Optional[Path] = None
    profile: Optional[str] = None

    def config(self) -> ArmConfig:
        return ArmConfig.from_yaml(self.profile) if self.profile else self.spec.config(self.base)

    def build(self) -> Arm:
        arm = self.spec.build(self.base, self.profile)
        if self.spec.backend == "slcan":
            arm.config.control_rate_hz = min(arm.config.control_rate_hz, 25.0)   # verified serial SLCAN rate
        return arm

    def backend(self) -> CanBackend:
        return self.spec.open_backend()

    @property
    def label(self) -> str:
        return f"{self.name} ({self.spec.backend} {self.spec.address})"


def load_rig(a) -> Rig:
    return Rig.load(getattr(a, "rig", None))


def explicit_spec(a) -> Optional[ArmSpec]:
    if not any(getattr(a, k, None) is not None for k in ("backend", "channel", "port")):
        return None
    backend = a.backend
    if backend is None:
        backend = "socketcan" if a.channel else ("gs_usb" if (a.port or "").startswith("gs_usb") else default_backend())
    return ArmSpec(backend, channel=a.channel if backend == "socketcan" else None, port=a.port,
                   baudrate=getattr(a, "serial_baudrate", None), startup_delay=getattr(a, "slcan_startup_delay", None))


def targets(a, many: bool = False) -> list[Target]:
    names = getattr(a, "arm", None) or []
    explicit = explicit_spec(a)
    if names and explicit:
        raise CliError("pass either --arm or --backend/--channel/--port, not both")
    profile = getattr(a, "config", None)
    if explicit:
        return [Target("arm", explicit, profile=profile)]
    rig = load_rig(a)
    if names:
        chosen = rig.names(names)
        if len(chosen) > 1 and not many:
            raise CliError(f"this command takes one arm; got {', '.join(chosen)}")
        return [Target(n, rig.spec(n), rig.base, profile) for n in chosen]
    if len(rig.arms) == 1:
        name = rig.arm_names[0]
        return [Target(name, rig.spec(name), rig.base, profile)]
    if len(rig.arms) > 1:
        log.warning("the rig names several arms (%s) and no --arm was given: using the default bus",
                    ", ".join(rig.arm_names))
    return [Target("arm", ArmSpec(default_backend()), profile=profile)]


def target(a) -> Target:
    return targets(a, many=False)[0]


def add_gravity_arg(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--gravity-scale", type=float,
                    help=f"override the gravity feed-forward scale, 0..{MAX_GRAVITY_SCALE} (0 = off)")


def apply_gravity_arg(arm: Arm, a, say: Callable = print) -> None:
    if getattr(a, "gravity_scale", None) is None:
        return
    if not 0 <= a.gravity_scale <= MAX_GRAVITY_SCALE:
        raise SystemExit(f"--gravity-scale must be in [0, {MAX_GRAVITY_SCALE}]")
    arm.config.gravity_scale = a.gravity_scale
    say(f"gravity feed-forward scale: {a.gravity_scale:g}")


# ── confirmation and stopping ──
def confirm(a, out: Out, question: str, destructive: bool = True, word: Optional[str] = None) -> None:
    """Ask before acting. ``--yes`` skips the question; without a terminal a destructive action is
    refused (pass --yes) and a harmless one goes ahead. ``word`` must then be typed exactly."""
    if getattr(a, "yes", False):
        return
    if not sys.stdin.isatty():
        if destructive:
            raise CliError(f"{question.rstrip('?')}: refusing without --yes (stdin is not a terminal)")
        return
    hint = f"type {word}" if word else "y/N"
    sys.stderr.write(f"{question} [{hint}] ")
    sys.stderr.flush()
    try:
        answer = input().strip()
    except EOFError:
        answer = ""
    if (answer != word) if word else (answer.lower() not in ("y", "yes")):
        raise CliError("cancelled")


class Interrupt:
    """While active, Ctrl-C and SIGTERM set ``event`` instead of raising (a second Ctrl-C raises)."""

    def __init__(self):
        self.event = threading.Event()
        self._old: dict = {}

    def _handle(self, signum, frame):
        if self.event.is_set() and signum == signal.SIGINT:
            raise KeyboardInterrupt
        self.event.set()

    def __enter__(self) -> "Interrupt":
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT, signal.SIGTERM):
                self._old[sig] = signal.signal(sig, self._handle)
        return self

    def __exit__(self, *exc) -> None:
        for sig, old in self._old.items():
            signal.signal(sig, old)
        self._old.clear()

    def wait(self, seconds: Optional[float] = None, stop_if: Optional[Callable[[], Optional[str]]] = None,
             poll: float = 0.05) -> str:
        """Block until Ctrl-C/SIGTERM ("interrupted"), ``seconds`` ("done"), or ``stop_if()`` returns a reason."""
        deadline = None if seconds is None else time.monotonic() + seconds
        while not self.event.is_set():
            if stop_if is not None:
                reason = stop_if()
                if reason:
                    return reason
            if deadline is not None and time.monotonic() >= deadline:
                return "done"
            self.event.wait(poll if deadline is None else max(0.0, min(poll, deadline - time.monotonic())))
        return "interrupted"


def not_enabled(arms: dict) -> Optional[str]:
    """A reason when some arm left ENABLED (a fault), for ``Interrupt.wait(stop_if=...)``."""
    for name, arm in arms.items():
        if arm.state.name != "ENABLED":
            return f"{name}: {arm.state.name.lower()} ({arm.fault_reason})"
    return None


def check_stay(arms, then: str) -> None:
    """Refuse ``--then stay`` before anything is energized when a stiff joint (not
    release_on_fault) has a motor watchdog configured: it would drop once commands stop."""
    if then != "stay":
        return
    for arm in arms:
        drops = [j.label for j in arm.config.joints if j.can_timeout_ms is not None and not j.release_on_fault]
        if drops:
            raise CliError(f"--then stay would drop {', '.join(drops)}: their motor watchdog releases torque "
                           "once commands stop. Use --then rest")


OUTCOMES = {"rest": "rest", "stay": "stay", "release": "released", "hold": "rest"}


def achieved(outcome, then: str) -> bool:
    """Did every arm end as ``--then`` asked? ``outcome``: one result or {name: result}."""
    results = outcome.values() if isinstance(outcome, dict) else [outcome]
    return all(r == OUTCOMES[then] for r in results)


def finish(arm: Arm, then: str, out: Out) -> str:
    """Apply ``--then`` to an arm this process enabled. Returns the outcome (see safety)."""
    if arm.state.name not in ("ENABLED", "FAULT"):
        return "not_enabled"
    if then == "stay":
        releasing = arm.detach()
        out.say(f"leaving torque on: {detached_text(arm, releasing)}. Release it with "
                f"'{CLI_NAME} release' before you touch it.")
        return "stay"
    if then == "release":
        out.say("releasing torque where the arm is")
        arm.disable()
        if arm.unconfirmed_disable:
            out.warn(f"torque off not confirmed on motors {arm.unconfirmed_disable}; cut motor power if they hold")
        return released(arm)
    return release_if_holding(arm, say=out.say)


def group_of(chosen: list[Target]) -> ArmGroup:
    """The selected arms, not connected; parallel when every arm has its own bus."""
    buses = [(t.spec.backend, t.spec.address) for t in chosen]
    return ArmGroup({t.name: t.build() for t in chosen}, parallel=len(set(buses)) == len(buses))


def wind_down(group: ArmGroup, then: str, out: Out) -> dict[str, str]:
    """``--then`` for every arm this process enabled, then close every bus (always). An arm still
    energized at the end (an error or a second Ctrl-C during ``--then``) is left holding
    (detach()), never released."""
    outcome = {}
    try:
        for name, arm in group:
            try:
                outcome[name] = finish(arm, then, out)
            except Exception as e:
                out.warn(f"{name}: {then} failed: {e}")
                outcome[name] = f"failed: {e}"
    finally:
        for name, arm in group:
            try:
                if arm.state.name in ("ENABLED", "FAULT"):
                    releasing = arm.detach()
                    out.warn(f"{name}: left holding with torque on: {detached_text(arm, releasing)}. "
                             f"Support it, then run '{CLI_NAME} release'")
                    outcome[name] = "holding"
                else:
                    arm.disconnect()
            except Exception as e:
                log.error("%s: closing the bus failed: %s", name, e)
    return outcome


# ── read-only motor access (never enables, never clears unless asked) ──
def joint_position(joint: JointConfig, motor_position: float) -> float:
    return (motor_position - joint.offset) * joint.direction


class Bus:
    """The motors of one profile on their bus, without an Arm: fault queries and state reads only.
    Works when a motor reports a fault (Arm.connect refuses then) and never changes torque."""

    def __init__(self, backend: CanBackend, config: ArmConfig, ids: Optional[list[int]] = None):
        self.backend, self.config = backend, config
        joints = {j.motor_id: j for j in config.joints}
        ids = list(joints) if ids is None else ids
        self.joints = {i: joints.get(i) or JointConfig(motor_id=i, lo=-math.pi, hi=math.pi, kp=0.0, kd=0.0)
                       for i in ids}
        self.motors = {i: Motor(j, backend, config.host_id) for i, j in self.joints.items()}
        self.known = set(joints)

    def _receive(self, can_id, data) -> None:
        m = self.motors.get(route(can_id, data, self.config.host_id))
        if m is not None:
            m.handle_frame(can_id, data)

    def __enter__(self) -> "Bus":
        self.backend.set_recv_callback(self._receive)
        try:
            self.backend.open()
        except Exception as e:
            raise CliError(f"could not open the CAN adapter: {e}. Close other programs using it and retry.") from e
        return self

    def __exit__(self, *exc) -> None:
        self.backend.close()

    def read(self, motor_id: int, clear: bool = False, timeout: float = 0.1) -> Optional[dict]:
        """One motor's state in joint coordinates, or None when it does not answer."""
        m, j = self.motors[motor_id], self.joints[motor_id]
        try:
            status = m.check(timeout=timeout, attempts=2)
        except ParamTimeout:
            return None
        cleared = False
        if status.has_fault and clear:
            m.clear_fault()
            time.sleep(0.01)
            status = m.check(timeout=timeout, attempts=2)
            cleared = True
        fb = None
        if not status.has_fault:
            try:
                fb = m.read_state(timeout=timeout / 2, attempts=3)
            except (ParamTimeout, FaultLatched):
                pass
        fault = (status.text or f"code 0x{status.code:X}") if status.has_fault else m.fault_latched
        row = {"motor_id": motor_id, "joint": j.name if motor_id in self.known else None,
               "position": None, "velocity": None, "torque": None, "temperature": None, "temperatures": {},
               "enabled": None, "fault": fault, "fault_code": status.code if status.has_fault else 0,
               "cleared": cleared, "outside_limits": False}
        if fb is not None:
            q = joint_position(j, fb.position)
            row.update(position=q, velocity=fb.velocity * j.direction, torque=fb.torque * j.direction,
                       temperature=fb.temperature, temperatures=dict(fb.temperatures), enabled=fb.enabled)
            if motor_id in self.known:
                grace = self.config.full_turn_grace
                row["outside_limits"] = not any(j.lo - grace <= q + k <= j.hi + grace
                                                for k in (0.0, -2 * math.pi, 2 * math.pi))
        return row

    def read_all(self, clear: bool = False) -> list[dict]:
        rows = [self.read(i, clear) for i in self.motors]
        return [r for r in rows if r is not None]

    def missing(self, rows: list[dict]) -> list[int]:
        found = {r["motor_id"] for r in rows}
        return [i for i in self.motors if i in self.known and i not in found]


# ── formatting ──
def deg(value: Optional[float], width: int = 8, digits: int = 1) -> str:
    if value is None or not math.isfinite(value):
        return "-".rjust(width)
    return f"{math.degrees(value):+{width}.{digits}f}"


def num(value: Optional[float], width: int = 6, digits: int = 1) -> str:
    if value is None or not math.isfinite(value):
        return "-".rjust(width)
    return f"{value:{width}.{digits}f}"


def pose_text(names: list[str], q: list[float]) -> str:
    return "  ".join(f"{n}={math.degrees(v):+.1f}" for n, v in zip(names, q) if math.isfinite(v))
