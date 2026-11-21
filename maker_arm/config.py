"""Arm profile: machine-specific data (limits, gains, protection, model) read from YAML.

Joint limits may be given in degrees (``limits_deg: [lo, hi]``) or radians (``lo``/``hi``); the
SDK API is radians. Any other ``<name>_deg`` setting, top-level or per joint, a number or a list,
is converted to ``<name>`` in radians (``max_velocity_deg``, ``rest_pose_deg``, ``tcp_rpy_deg``, ...).

Model fields (read by kinematics.py, gravity.py and leaders/arm.py): ``urdf`` (path, absolute or
relative to the package directory), ``urdf_joints`` (actuated URDF joints in order; default: every
revolute joint on the longest chain), ``urdf_base_rpy`` (mount rotation turning the URDF z-up),
``tcp_xyz``/``tcp_rpy`` (tool point on the last arm link); per joint ``urdf_sign``/``urdf_offset``
(URDF angle = joint * sign + offset), ``gravity_limit_nm`` (cap, required on every URDF joint when
gravity is on), ``gravity_factor`` (0..2, per-joint correction), and the leader-arm friction terms
``friction_viscous``, ``friction_coulomb``, ``friction_rest_nm``, ``friction_velocity_clamp`` with
the arm-level ``friction_deadzone``.

``ik`` (read by trajectory_ik.py): a mapping of the trajectory IK's weights and ``limit_margin``
(names as ``IKWeights``) and per URDF joint ``rest_pose``, ``rest_weight`` and ``max_speed`` (rad/s);
``_deg`` forms convert as above.
"""

import math
from dataclasses import dataclass, field
from os import PathLike
from typing import Optional

import yaml

from .gravity import DEFAULT_GRAVITY_SCALE, MAX_GRAVITY_SCALE
from .motor import DEFAULT_HOST_ID, DEFAULT_MODEL, ID_RANGE, KD_RANGE, KP_RANGE, MODELS
from .trajectory_ik import JOINT_FIELDS, WEIGHT_FIELDS


@dataclass(frozen=True)
class JointConfig:
    motor_id: int
    lo: float         # rad, soft limit in joint coordinates
    hi: float
    kp: float
    kd: float
    model: str = DEFAULT_MODEL
    name: str = ""
    direction: int = 1     # ±1
    offset: float = 0.0    # rad: motor = joint*direction + offset
    can_timeout_ms: Optional[int] = None      # motor-side watchdog, where firmware supports it
    max_temperature_c: Optional[float] = None    # stop (fault) at or above this
    enable_temperature_c: Optional[float] = None  # refuse enable unless below this
    return_to_rest: bool = True    # included in the verified return to rest on stop
    release_on_fault: bool = False  # disable this motor (instead of holding) when it faults
    warning_is_fault: bool = False  # treat the feedback warning flag as a fault
    hold_torque_nm: Optional[float] = None  # gripper holding-effort regulation (see gripper.py)
    closing_direction: int = 1      # gripper: sign of joint motion that closes the jaws
    open_position: Optional[float] = None   # gripper: rad, jaws fully open (None: the limit away from closing)
    reply_id: Optional[int] = None  # CAN ID the motor replies on, where the protocol has one
    # Model (URDF) and feed-forward, see the module docstring.
    urdf_sign: int = 1                  # ±1: URDF angle = joint * urdf_sign + urdf_offset
    urdf_offset: float = 0.0            # rad
    gravity_limit_nm: Optional[float] = None   # cap on this joint's gravity feed-forward
    gravity_factor: float = 1.0         # 0..2, scales this joint's gravity torque
    friction_viscous: float = 0.0       # N*m*s/rad (leader arm float mode)
    friction_velocity_clamp: float = math.inf   # rad/s, velocity the viscous term saturates at
    friction_coulomb: float = 0.0       # N*m in the direction of travel
    friction_rest_nm: float = 0.0       # N*m while still (breakaway)

    @property
    def label(self) -> str:
        return self.name or f"motor {self.motor_id}"


@dataclass
class ArmConfig:
    host_id: int = DEFAULT_HOST_ID
    required_motor_ids: Optional[list[int]] = None
    control_rate_hz: float = 200.0
    max_velocity: float = 1.5          # rad/s, command rate limit toward the user target
    limit_margin: float = 0.0          # rad, soft-limit inward margin
    feedback_timeout: float = 0.25     # s, host-side feedback watchdog
    fault_poll_interval: float = 0.25  # s, non-clearing fault query per motor while enabled
    full_turn_grace: float = math.radians(20.0)  # rad, see Arm.connect()
    hold_on_fault: bool = True         # on FAULT, hold the current pose (prevents a drop)
    gravity_scale: float = DEFAULT_GRAVITY_SCALE  # 0..1.2, read at enable; see gravity.py
    urdf: Optional[str] = None         # kinematic model of the arm
    urdf_joints: Optional[list[str]] = None   # actuated URDF joints, in joint order
    urdf_base_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # rad, URDF root -> world (z-up)
    tcp_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # m, tool point on the last arm link
    tcp_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # rad
    friction_deadzone: float = 0.05    # rad/s, below it a leader joint counts as still
    rest_pose: Optional[list[float]] = None   # rad per joint, the named "rest" pose (move --pose rest)
    ik: Optional[dict] = None          # trajectory IK settings, see the module docstring
    # Startup sync: until a joint is synced its command stays within step of the measured
    # position; synced once |target - present| <= synced, or released after it moves no more
    # than stuck for stuck_seconds.
    startup_step: float = math.radians(1.0)
    startup_synced: float = math.radians(3.0)
    startup_stuck: float = math.radians(0.2)
    startup_stuck_seconds: float = 1.0
    # Return to rest (Arm.return_to_rest).
    rest_speed: float = math.radians(30.0)     # rad/s
    rest_rate_hz: float = 30.0
    rest_timeout: float = 10.0                 # s
    rest_arrived: float = math.radians(2.0)
    rest_settle: float = math.radians(6.0)     # accepted after a stall
    rest_stall_improvement: float = math.radians(0.25)
    rest_stall_polls: int = 15
    joints: list[JointConfig] = field(default_factory=list)

    @property
    def n_joints(self) -> int:
        return len(self.joints)

    @classmethod
    def from_yaml(cls, path: str) -> "ArmConfig":
        with open(path) as f:
            raw = yaml.safe_load(f)
        try:
            joints = [_joint_from_yaml(j) for j in raw.pop("joints")]
            if isinstance(raw.get("ik"), dict):
                raw["ik"] = _degrees_to_radians(raw["ik"])
            cfg = cls(joints=joints, **_degrees_to_radians(raw))
        except KeyError as e:
            raise ValueError(f"{path}: missing config item {e}") from e
        except TypeError as e:
            raise ValueError(f"{path}: unrecognized or missing config field -- check spelling ({e})") from e
        cfg._validate()
        return cfg

    def _validate(self) -> None:
        if not 0 <= self.host_id <= 0x7FF:
            raise ValueError("host_id must be a standard CAN ID")
        positive = ("control_rate_hz", "max_velocity", "feedback_timeout", "fault_poll_interval",
                    "startup_step", "startup_synced", "startup_stuck", "startup_stuck_seconds",
                    "rest_speed", "rest_rate_hz", "rest_timeout", "rest_arrived", "rest_settle",
                    "rest_stall_improvement", "rest_stall_polls")
        for name in positive:
            if not getattr(self, name) > 0:
                raise ValueError(f"{name} must be >0, got {getattr(self, name)}")
        if self.limit_margin < 0 or self.full_turn_grace < 0:
            raise ValueError("limit_margin and full_turn_grace must be >=0")
        if not 0 <= self.gravity_scale <= MAX_GRAVITY_SCALE:
            raise ValueError(f"gravity_scale must be in [0, {MAX_GRAVITY_SCALE}], got {self.gravity_scale}")
        for name in ("tcp_xyz", "tcp_rpy", "urdf_base_rpy"):
            value = getattr(self, name)
            if len(value) != 3 or not all(math.isfinite(v) for v in value):
                raise ValueError(f"{name} must be three finite numbers, got {value}")
        if self.urdf is not None and not isinstance(self.urdf, (str, PathLike)):
            raise ValueError(f"urdf must be a path, got {self.urdf!r}")
        if self.urdf_joints is not None and not (
                self.urdf_joints and all(isinstance(n, str) and n for n in self.urdf_joints)
                and len(set(self.urdf_joints)) == len(self.urdf_joints)):
            raise ValueError(f"urdf_joints must be distinct joint names, got {self.urdf_joints}")
        if self.urdf_joints is not None and len(self.urdf_joints) > len(self.joints):
            raise ValueError(f"urdf_joints names {len(self.urdf_joints)} joints, the profile has {len(self.joints)}")
        if self.rest_pose is not None and not (
                len(self.rest_pose) == len(self.joints) and all(math.isfinite(v) for v in self.rest_pose)):
            raise ValueError(f"rest_pose needs one finite value per joint ({len(self.joints)}), got {self.rest_pose}")
        self._validate_ik()
        if not (math.isfinite(self.friction_deadzone) and self.friction_deadzone >= 0):
            raise ValueError(f"friction_deadzone must be finite and >=0, got {self.friction_deadzone}")
        ids = [j.motor_id for j in self.joints]
        if len(set(ids)) != len(ids):
            raise ValueError(f"motor_id must be unique: {ids}")
        if self.required_motor_ids is not None and ids != self.required_motor_ids:
            raise ValueError(f"motor IDs must be exactly {self.required_motor_ids}, in order; got {ids}")
        for j in self.joints:
            if not ID_RANGE[0] <= j.motor_id <= ID_RANGE[1]:
                raise ValueError(f"motor_id must be in [{ID_RANGE[0]}, {ID_RANGE[1]}], got {j.motor_id}")
            if j.reply_id is not None and not 0 <= j.reply_id <= 0x7FF:
                raise ValueError(f"motor {j.motor_id}: reply_id must be a standard CAN ID")
            if j.model not in MODELS:
                raise ValueError(f"motor {j.motor_id}: unknown model {j.model!r} (supported: {sorted(MODELS)})")
            if j.direction not in (1, -1):
                raise ValueError(f"motor {j.motor_id}: direction can only be ±1, got {j.direction}")
            if not j.lo + 2 * self.limit_margin < j.hi:
                raise ValueError(f"motor {j.motor_id}: limits must satisfy lo + 2*margin < hi, got [{j.lo}, {j.hi}]")
            check_gains(j.motor_id, j.kp, j.kd)
            if j.can_timeout_ms is not None and not 0 < j.can_timeout_ms <= 200_000:
                raise ValueError(f"motor {j.motor_id}: can_timeout_ms must be in (0, 200000]; it never disables the watchdog")
            if j.hold_torque_nm is not None and not 0.1 <= j.hold_torque_nm <= 2.0:
                raise ValueError(f"motor {j.motor_id}: hold_torque_nm must be in [0.1, 2.0]")
            if j.closing_direction not in (1, -1):
                raise ValueError(f"motor {j.motor_id}: closing_direction can only be ±1")
            if j.open_position is not None and not (
                    math.isfinite(j.open_position) and j.lo <= j.open_position <= j.hi
                    and j.open_position != (j.hi if j.closing_direction > 0 else j.lo)):
                raise ValueError(f"motor {j.motor_id}: open_position must lie within the limits, off the closed end, "
                                 f"got {j.open_position}")
            if (j.max_temperature_c is not None and j.enable_temperature_c is not None
                    and not j.enable_temperature_c < j.max_temperature_c):
                raise ValueError(f"motor {j.motor_id}: enable_temperature_c must be below max_temperature_c")
            _check_model_fields(j)

    def _validate_ik(self) -> None:
        if self.ik is None:
            return
        if not isinstance(self.ik, dict):
            raise ValueError(f"ik must be a mapping, got {self.ik!r}")
        unknown = sorted(set(self.ik) - set(WEIGHT_FIELDS) - set(JOINT_FIELDS))
        if unknown:
            raise ValueError(f"ik: unknown settings {unknown} (known: {sorted(WEIGHT_FIELDS + JOINT_FIELDS)})")
        for name in WEIGHT_FIELDS:
            value = self.ik.get(name)
            if value is not None and not (isinstance(value, (int, float)) and math.isfinite(value) and value >= 0):
                raise ValueError(f"ik.{name} must be a finite number >=0, got {value!r}")
        for name in JOINT_FIELDS:
            value = self.ik.get(name)
            if value is None:
                continue
            if not (isinstance(value, (list, tuple)) and 0 < len(value) <= len(self.joints)
                    and all(isinstance(v, (int, float)) and math.isfinite(v) for v in value)):
                raise ValueError(f"ik.{name} must list one finite number per URDF joint, got {value!r}")
            if name == "max_speed" and not all(v > 0 for v in value):
                raise ValueError(f"ik.max_speed must be >0 per joint, got {value!r}")
            if name == "rest_weight" and not all(v >= 0 for v in value):
                raise ValueError(f"ik.rest_weight must be >=0 per joint, got {value!r}")


def check_gains(motor_id: int, kp: float, kd: float) -> None:
    if not (math.isfinite(kd) and KD_RANGE[0] <= kd <= KD_RANGE[1]):
        raise ValueError(f"motor {motor_id}: kd out of protocol range [{KD_RANGE[0]:g},{KD_RANGE[1]:g}], got {kd}")
    if not (math.isfinite(kp) and KP_RANGE[0] <= kp <= KP_RANGE[1]):
        raise ValueError(f"motor {motor_id}: kp out of protocol range [{KP_RANGE[0]:g},{KP_RANGE[1]:g}], got {kp}")


def _check_model_fields(j: JointConfig) -> None:
    label = f"motor {j.motor_id}"
    if j.urdf_sign not in (1, -1):
        raise ValueError(f"{label}: urdf_sign can only be ±1, got {j.urdf_sign}")
    if not math.isfinite(j.urdf_offset):
        raise ValueError(f"{label}: urdf_offset must be finite")
    if j.gravity_limit_nm is not None and not (math.isfinite(j.gravity_limit_nm) and j.gravity_limit_nm >= 0):
        raise ValueError(f"{label}: gravity_limit_nm must be finite and >=0, got {j.gravity_limit_nm}")
    if not (math.isfinite(j.gravity_factor) and 0 <= j.gravity_factor <= 2):
        raise ValueError(f"{label}: gravity_factor must be in [0, 2], got {j.gravity_factor}")
    for name in ("friction_viscous", "friction_coulomb", "friction_rest_nm"):
        value = getattr(j, name)
        if not (math.isfinite(value) and value >= 0):
            raise ValueError(f"{label}: {name} must be finite and >=0, got {value}")
    if not (j.friction_velocity_clamp > 0 and not math.isnan(j.friction_velocity_clamp)):
        raise ValueError(f"{label}: friction_velocity_clamp must be >0 (inf = no clamp), got {j.friction_velocity_clamp}")


def _degrees_to_radians(raw: dict) -> dict:
    """``<name>_deg`` keys (a number or a list) become ``<name>`` in radians."""
    out = {}
    for key, value in raw.items():
        if key.endswith("_deg"):
            key = key[:-4]
            if key in raw:
                raise ValueError(f"both {key} and {key}_deg are set")
            value = [math.radians(v) for v in value] if isinstance(value, (list, tuple)) else math.radians(value)
        out[key] = value
    return out


def _joint_from_yaml(raw: dict) -> JointConfig:
    raw = dict(raw)
    if "limits_deg" in raw:
        lo, hi = raw.pop("limits_deg")
        raw["lo"], raw["hi"] = math.radians(lo), math.radians(hi)
    return JointConfig(**_degrees_to_radians(raw))
