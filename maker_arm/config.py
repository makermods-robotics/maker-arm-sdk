"""Machine-specific data (limits/gains/protection) lives entirely in YAML; zero hardcoding in code.

The shipped profile mirrors MakerModsLab's Maker Arm v1 follower (LeRobot MakerFollowerConfig
plus MakerModsLab's gripper bound and gripper protection). YAML joint limits may be given in
degrees (``limits_deg: [lo, hi]``) or radians (``lo``/``hi``); the SDK API is radians.
"""

import math
from dataclasses import dataclass, field
from typing import Optional

import yaml

from . import protocol


@dataclass(frozen=True)
class JointConfig:
    motor_id: int
    lo: float         # rad, soft limit in joint coordinates
    hi: float
    kp: float
    kd: float
    model: str = "RS00"
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

    @property
    def label(self) -> str:
        return self.name or f"motor {self.motor_id}"


@dataclass
class ArmConfig:
    host_id: int = protocol.HOST_CAN_ID
    required_motor_ids: Optional[list[int]] = None
    control_rate_hz: float = 200.0
    max_velocity: float = 1.5          # rad/s, command rate limit toward the user target
    limit_margin: float = 0.0          # rad, soft-limit inward margin
    feedback_timeout: float = 0.25     # s, host-side feedback watchdog
    fault_poll_interval: float = 0.25  # s, non-clearing fault query per motor while enabled
    full_turn_grace: float = math.radians(20.0)  # rad, see Arm.connect()
    hold_on_fault: bool = True         # on FAULT, hold the current pose (prevents a drop)
    # Startup sync (LeRobot MakerFollower): until a joint is synced its command stays within
    # step of the measured position; synced once |target - present| <= synced, or released
    # after it moves no more than stuck for stuck_seconds.
    startup_step: float = math.radians(1.0)
    startup_synced: float = math.radians(3.0)
    startup_stuck: float = math.radians(0.2)
    startup_stuck_seconds: float = 1.0
    # Return to rest (MakerModsLab maker_rest_pose.py).
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
            for key in [k for k in raw if k.endswith("_deg")]:
                raw[key[:-4]] = math.radians(raw.pop(key))
            cfg = cls(joints=joints, **raw)
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
        ids = [j.motor_id for j in self.joints]
        if len(set(ids)) != len(ids):
            raise ValueError(f"motor_id must be unique: {ids}")
        if self.required_motor_ids is not None and ids != self.required_motor_ids:
            raise ValueError(f"motor IDs must be exactly {self.required_motor_ids}, in order; got {ids}")
        for j in self.joints:
            if not 1 <= j.motor_id <= 127:
                raise ValueError(f"motor_id must be in [1, 127], got {j.motor_id}")
            if j.model not in protocol.MOTOR_PARAMS:
                raise ValueError(f"motor {j.motor_id}: unknown model {j.model!r} (supported: {sorted(protocol.MOTOR_PARAMS)})")
            if j.direction not in (1, -1):
                raise ValueError(f"motor {j.motor_id}: direction can only be ±1, got {j.direction}")
            if not j.lo + 2 * self.limit_margin < j.hi:
                raise ValueError(f"motor {j.motor_id}: limits must satisfy lo + 2*margin < hi, got [{j.lo}, {j.hi}]")
            if not protocol.KD_MIN <= j.kd <= protocol.KD_MAX:
                raise ValueError(f"motor {j.motor_id}: kd out of protocol range [0,5], got {j.kd}")
            if not protocol.KP_MIN <= j.kp <= protocol.KP_MAX:
                raise ValueError(f"motor {j.motor_id}: kp out of protocol range [0,500], got {j.kp}")
            if j.can_timeout_ms is not None and not 0 < j.can_timeout_ms <= 200_000:
                raise ValueError(f"motor {j.motor_id}: can_timeout_ms must be in (0, 200000]; it never disables the watchdog")
            if j.hold_torque_nm is not None and not 0.1 <= j.hold_torque_nm <= 2.0:
                raise ValueError(f"motor {j.motor_id}: hold_torque_nm must be in [0.1, 2.0]")
            if j.closing_direction not in (1, -1):
                raise ValueError(f"motor {j.motor_id}: closing_direction can only be ±1")
            if (j.max_temperature_c is not None and j.enable_temperature_c is not None
                    and not j.enable_temperature_c < j.max_temperature_c):
                raise ValueError(f"motor {j.motor_id}: enable_temperature_c must be below max_temperature_c")


def _joint_from_yaml(raw: dict) -> JointConfig:
    raw = dict(raw)
    if "limits_deg" in raw:
        lo, hi = raw.pop("limits_deg")
        raw["lo"], raw["hi"] = math.radians(lo), math.radians(hi)
    return JointConfig(**raw)
