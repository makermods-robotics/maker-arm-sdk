"""Machine-specific data (direction/offset/limits/gains) lives entirely in YAML; zero hardcoding in code."""

from dataclasses import dataclass, field

import yaml

from . import protocol


@dataclass(frozen=True)
class JointConfig:
    motor_id: int
    direction: int    # ±1
    offset: float     # rad: motor = joint*direction + offset
    lo: float         # rad, soft limit in joint coordinates
    hi: float
    kp: float
    kd: float
    model: str = "RS00"   # motor model -> protocol.MOTOR_PARAMS lookup table


@dataclass
class ArmConfig:
    control_rate_hz: float = 200.0
    max_velocity: float = 1.5          # rad/s, speed-limiting approach ceiling
    limit_margin: float = 0.05         # rad, soft-limit inward margin
    feedback_timeout: float = 0.2      # s, host-side watchdog
    motor_can_timeout_ms: int = 200    # motor-side watchdog (must never be 0)
    hold_on_fault: bool = True         # on FAULT, lock and hold current pose (prevents drop); False = release torque immediately
    joints: list[JointConfig] = field(default_factory=list)

    @property
    def n_joints(self) -> int:
        return len(self.joints)

    @classmethod
    def from_yaml(cls, path: str) -> "ArmConfig":
        with open(path) as f:
            raw = yaml.safe_load(f)
        try:
            joints = [JointConfig(**j) for j in raw.pop("joints")]
            cfg = cls(joints=joints, **raw)
        except KeyError as e:
            raise ValueError(f"{path}: missing config item {e}") from e
        except TypeError as e:
            raise ValueError(f"{path}: unrecognized or missing config field -- check spelling ({e})") from e
        cfg._validate()
        return cfg

    def _validate(self) -> None:
        if self.motor_can_timeout_ms <= 0:
            raise ValueError("motor_can_timeout_ms must be >0: the motor-side CAN_TIMEOUT watchdog must never be disabled")
        if self.control_rate_hz <= 0:
            raise ValueError(f"control_rate_hz must be >0, got {self.control_rate_hz}")
        if self.max_velocity <= 0:
            raise ValueError(f"max_velocity must be >0, got {self.max_velocity}")
        if self.feedback_timeout <= 0:
            raise ValueError(f"feedback_timeout must be >0, got {self.feedback_timeout}")
        if self.limit_margin < 0:
            raise ValueError(f"limit_margin must be >=0, got {self.limit_margin}")
        ids = [j.motor_id for j in self.joints]
        if len(set(ids)) != len(ids):
            raise ValueError(f"motor_id must be unique: {ids}")
        for j in self.joints:
            if j.model not in protocol.MOTOR_PARAMS:
                raise ValueError(f"motor {j.motor_id}: unknown model {j.model!r} (supported: {sorted(protocol.MOTOR_PARAMS)})")
            if j.direction not in (1, -1):
                raise ValueError(f"motor {j.motor_id}: direction can only be ±1, got {j.direction}")
            if not j.lo < j.hi:
                raise ValueError(f"motor {j.motor_id}: limits must satisfy lo < hi, got [{j.lo}, {j.hi}]")
            if not j.lo + 2 * self.limit_margin < j.hi:
                raise ValueError(f"motor {j.motor_id}: limit range too narrow (lo+2*margin >= hi)")
            if not (protocol.KD_MIN <= j.kd <= protocol.KD_MAX):
                raise ValueError(f"motor {j.motor_id}: kd out of protocol range [0,5], got {j.kd}")
            if not (protocol.KP_MIN <= j.kp <= protocol.KP_MAX):
                raise ValueError(f"motor {j.motor_id}: kp out of protocol range [0,500], got {j.kp}")
