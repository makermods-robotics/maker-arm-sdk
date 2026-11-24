"""A bench described once: which adapter drives which arm, the cameras, and the teleop leaders.

    rig = Rig.load()                       # ~/.config/<cli>/rig.yaml, $<CLI>_RIG, or a path
    with rig.group(["left", "right"]) as arms:
        arms.enable()                      # both arms, in parallel
        arms["left"].move_to(q)
    # leaving the block returns every arm to rest and releases it

The file (YAML)::

    arms:
      left:  {backend: gs_usb, port: "gs_usb:<serial>"}   # profile: shipped default
      right: {backend: socketcan, channel: can1, profile: my_profile.yaml}
    cameras: {top: "usb:...", left_wrist: "usb:...", right_wrist: "index:2"}
    leaders:
      left:  {type: star, port: /dev/ttyUSB0}       # map: Star mapping JSON (default: shipped)
      right: {type: arm, arm: spare}                # another arm entry, floated by hand

A leader entry is keyed by the follower arm it drives. Relative profile and map paths are relative
to the rig file. Nothing here opens a device until an arm connects or a camera opens.
"""

from __future__ import annotations

import glob
import logging
import os
import platform
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Optional, Union

import yaml

from ._product import CLI_NAME, DEFAULT_PROFILE
from .arm import Arm, ArmState
from .cameras import Camera, is_camera_id
from .config import ArmConfig
from .errors import ArmError
from .leaders import ArmLeader, StarLeader
from .transport.base import CanBackend, create_backend

log = logging.getLogger(__name__)

BACKENDS = ("socketcan", "slcan", "gs_usb")
LEADER_TYPES = ("star", "arm")
ENV_VAR = CLI_NAME.upper().replace("-", "_") + "_RIG"
DEFAULT_CHANNEL = "can0"
SLCAN_BAUDRATE = 115200
SLCAN_STARTUP_DELAY = 2.5


class RigError(ArmError):
    """The rig file is invalid, or names something it does not contain."""


def default_path() -> Path:
    """$<CLI>_RIG when set, else ~/.config/<cli>/rig.yaml."""
    env = os.environ.get(ENV_VAR)
    return Path(env).expanduser() if env else Path.home() / ".config" / CLI_NAME / "rig.yaml"


def default_backend() -> str:
    """SocketCAN on Linux; serial SLCAN elsewhere (macOS and Windows have no SocketCAN)."""
    return "socketcan" if platform.system() == "Linux" else "slcan"


def slcan_candidates() -> list[str]:
    """Serial devices that may be a USB-SLCAN adapter (or a Star leader)."""
    system = platform.system()
    if system == "Darwin":
        return sorted(glob.glob("/dev/cu.usbmodem*"))
    if system == "Windows":
        from serial.tools import list_ports
        return sorted(p.device for p in list_ports.comports())
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


@dataclass
class ArmSpec:
    """How to reach one arm: a CAN backend and the profile describing it."""
    backend: str
    channel: Optional[str] = None     # socketcan interface
    port: Optional[str] = None        # slcan serial device, or gs_usb:<serial> (None: the only one)
    profile: Optional[str] = None     # None: the shipped profile
    baudrate: Optional[int] = None    # slcan serial baud rate
    startup_delay: Optional[float] = None  # slcan: seconds to wait after opening the port

    def __post_init__(self):
        if self.backend not in BACKENDS:
            raise RigError(f"backend must be one of {', '.join(BACKENDS)}, got {self.backend!r}")

    @property
    def address(self) -> str:
        """Where the bus is: "can1", "/dev/ttyACM0", "gs_usb:<serial>" ("gs_usb" = the only adapter)."""
        if self.backend == "socketcan":
            return self.channel or DEFAULT_CHANNEL
        if self.backend == "gs_usb":
            return self.port or "gs_usb"
        return self.port or "slcan"

    def config(self, base: Optional[Path] = None) -> ArmConfig:
        return ArmConfig.from_yaml(str(_resolve(self.profile, base) if self.profile else DEFAULT_PROFILE))

    def backend_kwargs(self) -> dict:
        if self.backend == "socketcan":
            return {"channel": self.channel or DEFAULT_CHANNEL}
        if self.backend == "gs_usb":
            return {"port": self.port}
        return {"port": self.port or only_serial_port(), "rtscts": False,
                "baudrate": self.baudrate or SLCAN_BAUDRATE,
                "startup_delay": SLCAN_STARTUP_DELAY if self.startup_delay is None else self.startup_delay}

    def open_backend(self) -> CanBackend:
        """A new (closed) backend for this arm's bus."""
        return create_backend(self.backend, **self.backend_kwargs())

    def build(self, base: Optional[Path] = None, profile: Optional[str] = None) -> Arm:
        """An Arm for this entry, not connected. ``profile`` overrides the entry's."""
        config = ArmConfig.from_yaml(profile) if profile else self.config(base)
        return Arm(config, self.open_backend())

    def to_yaml(self) -> dict:
        return {k: v for k, v in (("backend", self.backend), ("channel", self.channel), ("port", self.port),
                                  ("profile", self.profile), ("baudrate", self.baudrate),
                                  ("startup_delay", self.startup_delay)) if v is not None}


@dataclass
class LeaderSpec:
    type: str                        # "star" | "arm"
    port: Optional[str] = None       # star: serial port
    map: Optional[str] = None        # star: mapping JSON (None: shipped)
    arm: Optional[str] = None        # arm: name of the arm entry used as leader

    def __post_init__(self):
        if self.type not in LEADER_TYPES:
            raise RigError(f"leader type must be one of {', '.join(LEADER_TYPES)}, got {self.type!r}")
        if self.type == "star" and not self.port:
            raise RigError("a star leader needs a port")
        if self.type == "arm" and not self.arm:
            raise RigError("an arm leader needs arm: <name of another arm entry>")

    def to_yaml(self) -> dict:
        return {k: v for k, v in (("type", self.type), ("port", self.port), ("map", self.map),
                                  ("arm", self.arm)) if v is not None}


def only_serial_port() -> str:
    found = slcan_candidates()
    if len(found) == 1:
        log.info("using CAN adapter %s", found[0])
        return found[0]
    if not found:
        raise RigError("no USB CAN adapter found; plug it in or pass --port "
                       "(on Linux with SocketCAN use --backend socketcan)")
    raise RigError(f"several serial devices found ({', '.join(found)}); pass --port")


def _resolve(path: str, base: Optional[Path]) -> Path:
    p = Path(path).expanduser()
    return p if p.is_absolute() or base is None else base / p


def _fields(raw, cls, where: str) -> dict:
    if not isinstance(raw, dict):
        raise RigError(f"{where}: expected a mapping, got {raw!r}")
    known = set(cls.__dataclass_fields__)   # type: ignore[attr-defined]
    unknown = sorted(set(raw) - known)
    if unknown:
        raise RigError(f"{where}: unknown keys {unknown} (known: {sorted(known)})")
    return raw


@dataclass
class Rig:
    arms: dict[str, ArmSpec] = field(default_factory=dict)
    cameras: dict[str, str] = field(default_factory=dict)
    leaders: dict[str, LeaderSpec] = field(default_factory=dict)
    path: Optional[Path] = None

    # ── file ──
    @classmethod
    def load(cls, path: Union[str, Path, None] = None, missing_ok: bool = True) -> "Rig":
        """The rig at ``path`` (default: ``default_path()``). A missing file is an empty rig."""
        path = Path(path).expanduser() if path else default_path()
        if not path.exists():
            if missing_ok:
                return cls(path=path)
            raise RigError(f"no rig file at {path}")
        try:
            raw = yaml.safe_load(path.read_text()) or {}
        except yaml.YAMLError as e:
            raise RigError(f"{path}: not valid YAML ({e})") from e
        return cls.from_dict(raw, path)

    @classmethod
    def from_dict(cls, raw: dict, path: Optional[Path] = None) -> "Rig":
        where = str(path or "rig")
        if not isinstance(raw, dict) or set(raw) - {"arms", "cameras", "leaders"}:
            raise RigError(f"{where}: expected a mapping with arms, cameras and leaders")
        for key in ("arms", "cameras", "leaders"):
            if not isinstance(raw.get(key) or {}, dict):
                raise RigError(f"{where}: {key}: expected a mapping of names, got {raw[key]!r}")
        try:
            arms = {str(n): ArmSpec(**_fields(v, ArmSpec, f"{where}: arms.{n}"))
                    for n, v in (raw.get("arms") or {}).items()}
            leaders = {str(n): LeaderSpec(**_fields(v, LeaderSpec, f"{where}: leaders.{n}"))
                       for n, v in (raw.get("leaders") or {}).items()}
        except TypeError as e:
            raise RigError(f"{where}: {e}") from e
        cameras = {str(n): str(v) for n, v in (raw.get("cameras") or {}).items()}
        rig = cls(arms, cameras, leaders, path)
        rig.validate()
        return rig

    def to_dict(self) -> dict:
        return {"arms": {n: s.to_yaml() for n, s in self.arms.items()},
                "cameras": dict(self.cameras),
                "leaders": {n: s.to_yaml() for n, s in self.leaders.items()}}

    def save(self, path: Union[str, Path, None] = None) -> Path:
        self.validate()
        path = Path(path).expanduser() if path else (self.path or default_path())
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(yaml.safe_dump(self.to_dict(), sort_keys=False))
        tmp.replace(path)
        self.path = path
        return path

    def validate(self) -> None:
        reserved = {"all"}
        for name in self.arms:
            if name in reserved or not name:
                raise RigError(f"{name!r} cannot name an arm")
        seen: dict[tuple, str] = {}
        for name, spec in self.arms.items():
            key = (spec.backend, spec.address)
            if key in seen and spec.address not in ("gs_usb", "slcan"):
                raise RigError(f"arms {seen[key]!r} and {name!r} use the same bus {spec.address}")
            seen[key] = name
        for name, spec in self.leaders.items():
            if name not in self.arms:
                raise RigError(f"leaders.{name}: no arm named {name!r} to follow it")
            if spec.type == "arm":
                if spec.arm not in self.arms:
                    raise RigError(f"leaders.{name}: no arm named {spec.arm!r}")
                if spec.arm == name:
                    raise RigError(f"leaders.{name}: an arm cannot lead itself")

    # ── lookups ──
    @property
    def base(self) -> Optional[Path]:
        return self.path.parent if self.path else None

    @property
    def arm_names(self) -> list[str]:
        return list(self.arms)

    def spec(self, name: str) -> ArmSpec:
        if name not in self.arms:
            known = ", ".join(self.arms) or "none"
            raise RigError(f"no arm named {name!r} in {self.path or 'the rig'} (arms: {known})")
        return self.arms[name]

    def names(self, selected: Iterable[str]) -> list[str]:
        """Expand ``all`` and check every name; order kept, duplicates dropped."""
        out: list[str] = []
        for name in selected:
            for n in (self.arm_names if name == "all" else [name]):
                self.spec(n)
                if n not in out:
                    out.append(n)
        if not out:
            raise RigError(f"no arms in {self.path or 'the rig'}")
        return out

    def arm(self, name: str, profile: Optional[str] = None) -> Arm:
        """The named arm, not connected."""
        return self.spec(name).build(self.base, profile)

    def camera_id(self, name: str) -> str:
        """A rig camera name's id; an id (``usb:``, ``path:``, ``index:``, ``url:http...``) passes through."""
        if name in self.cameras:
            return self.cameras[name]
        if is_camera_id(name):
            return name
        known = ", ".join(self.cameras) or "none"
        raise RigError(f"no camera named {name!r} in {self.path or 'the rig'} (cameras: {known})")

    def camera(self, name: str, **kwargs) -> Camera:
        """The named camera (or an id), not opened."""
        return Camera(self.camera_id(name), **kwargs)

    def leader(self, name: str, follower_arm: Arm, arms: Optional[dict[str, Arm]] = None):
        """The leader configured for the follower ``name``. An arm leader reuses ``arms[leader]``
        when given (so one Arm object owns each bus), else builds it."""
        if name not in self.leaders:
            raise RigError(f"no leader for arm {name!r} in {self.path or 'the rig'}")
        spec = self.leaders[name]
        if spec.type == "star":
            mapping = str(_resolve(spec.map, self.base)) if spec.map else None
            return StarLeader(spec.port, mapping=mapping)
        leader_arm = (arms or {}).get(spec.arm) or self.arm(spec.arm)
        return ArmLeader(leader_arm, follower=follower_arm)

    def group(self, names: Iterable[str] = ("all",), profile: Optional[str] = None) -> "ArmGroup":
        names = self.names(names)
        buses = [(self.arms[n].backend, self.arms[n].address) for n in names]
        return ArmGroup({n: self.arm(n, profile) for n in names}, parallel=len(set(buses)) == len(buses))


class ArmGroup:
    """Several arms driven together: connect, enable and stop all of them, in parallel when each
    has its own bus. If one arm fails to enable, the arms already enabled are stopped again.

    Leaving a ``with`` block, or close(), stops every enabled arm (verified return to rest, then
    release) and disconnects; close() says per arm whether torque off was confirmed. An arm whose
    return is not verified (or is interrupted) is never released: it is detached holding its pose
    when no stiff joint has a motor watchdog armed, else left ENABLED with its control loop holding
    while this process runs (stop() or disable() it).
    """

    def __init__(self, arms: Union[dict[str, Arm], Iterable[Arm]], parallel: bool = True):
        self.arms = dict(arms) if isinstance(arms, dict) else {str(i): a for i, a in enumerate(arms)}
        self.parallel = parallel

    def __getitem__(self, name: str) -> Arm:
        return self.arms[name]

    def __iter__(self):
        return iter(self.arms.items())

    def __len__(self) -> int:
        return len(self.arms)

    def each(self, fn: Callable[[Arm], object], names: Optional[Iterable[str]] = None) -> dict[str, object]:
        """``fn(arm)`` for every arm (in parallel when allowed). Every call runs to completion;
        then the first exception, if any, is raised."""
        names = list(self.arms) if names is None else list(names)
        results: dict[str, object] = {}
        errors: dict[str, BaseException] = {}

        def call(name):
            try:
                results[name] = fn(self.arms[name])
            except BaseException as e:   # reported below, after every arm has finished
                errors[name] = e

        if self.parallel and len(names) > 1:
            threads = [threading.Thread(target=call, args=(n,), name=f"arm-{n}") for n in names]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
        else:
            for n in names:
                call(n)
        if errors:
            name, error = next(iter(errors.items()))
            if len(errors) > 1:
                log.error("several arms failed: %s", {n: str(e) for n, e in errors.items()})
            raise error if isinstance(error, ArmError) else ArmError(f"{name}: {error}") from error
        return {n: results.get(n) for n in names}

    def connect(self, clear_faults: bool = False, clear_released: bool = False) -> None:
        try:
            self.each(lambda a: a.connect(clear_faults=clear_faults, clear_released=clear_released))
        except BaseException:
            self.disconnect()
            raise

    def enable(self, cancel: Optional[threading.Event] = None) -> None:
        try:
            self.each(lambda a: a.enable(cancel=cancel))
        except BaseException:
            self.stop()
            raise

    def stop(self) -> dict[str, bool]:
        """Return every enabled arm to rest and release it; False where the return was not verified
        (that arm keeps holding)."""
        return self.each(lambda a: a.stop())

    def disable(self) -> None:
        self.each(lambda a: a.disable())

    def estop(self) -> None:
        for arm in self.arms.values():
            arm.estop()

    def disconnect(self) -> None:
        for arm in self.arms.values():
            try:
                arm.disconnect()
            except Exception as e:
                log.error("disconnect failed: %s", e)

    def __enter__(self) -> "ArmGroup":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> dict[str, str]:
        """Stop every arm and disconnect, as leaving a ``with`` block does. Outcome per arm:
        "released" (torque off confirmed by every motor), "energized" (not enabled here but its
        motors report torque on, e.g. left holding by an earlier process: bus closed, still on),
        "unknown" (torque off not confirmed: a motor did not answer, reports no mode, or the arm
        was never connected), "detached" (return to rest not verified: bus closed with torque on,
        holding its pose; joints with a motor watchdog release when it expires), "holding" (return
        not verified and a stiff joint would drop on its watchdog: left ENABLED, control loop
        holding) or "error: <reason>". Only "released" means the arm is off."""
        enabled = {n for n, a in self.arms.items() if a.state in (ArmState.ENABLED, ArmState.FAULT)}

        def stop(arm: Arm) -> None:
            try:
                arm.stop()
            except Exception as e:   # the arm is handled below by the state it is left in
                log.error("stop failed: %s", e)

        self.each(stop)
        outcomes: dict[str, str] = {}
        for name, arm in self.arms.items():
            try:
                if arm.state not in (ArmState.ENABLED, ArmState.FAULT):
                    if name in enabled:
                        outcomes[name] = "unknown" if arm.unconfirmed_disable else "released"
                    else:
                        outcomes[name] = _reported_torque(arm)
                    arm.disconnect()
                    if outcomes[name] != "released":
                        log.error("%s: torque off not confirmed (%s)", name, outcomes[name])
                    continue
                arm.hold_current_position()
                unsafe = arm.detach_unsafe()
                if unsafe:
                    outcomes[name] = "holding"
                    log.error("%s: return to rest not verified; still holding with torque on (control loop "
                              "running): %s would drop on their watchdog if commands stopped", name,
                              ", ".join(unsafe))
                else:
                    releasing = arm.detach()
                    outcomes[name] = "detached"
                    log.error("%s: return to rest not verified; left holding with torque on (%s release "
                              "on their watchdog). Support it, then run '%s release'", name,
                              ", ".join(releasing) or "no joints", CLI_NAME)
            except Exception as e:
                outcomes[name] = f"error: {e}"
                log.error("%s: closing failed: %s", name, e)
        return outcomes


def _reported_torque(arm: Arm) -> str:
    """"released", "energized" or "unknown" from fresh replies of an arm that is not enabled."""
    if arm.state is not ArmState.CONNECTED:
        return "unknown"
    try:
        fresh = arm.refresh()
    except Exception as e:
        log.error("torque state not read: %s", e)
        return "unknown"
    if not fresh or not all(fresh):
        return "unknown"
    reported = [j["enabled"] for j in arm.status()["joints"].values()]
    if any(on is True for on in reported):
        return "energized"
    return "unknown" if None in reported else "released"
