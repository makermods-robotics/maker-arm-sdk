"""机器特有数据（方向/偏移/限位/增益）全在 YAML，代码零硬编码。"""

from dataclasses import dataclass, field

import yaml

from . import protocol


@dataclass(frozen=True)
class JointConfig:
    motor_id: int
    direction: int    # ±1
    offset: float     # rad: motor = joint*direction + offset
    lo: float         # rad, 关节坐标软限位
    hi: float
    kp: float
    kd: float


@dataclass
class ArmConfig:
    control_rate_hz: float = 200.0
    max_velocity: float = 1.5          # rad/s，限速趋近上限
    limit_margin: float = 0.05         # rad，软限位内缩
    feedback_timeout: float = 0.2      # s，主机侧看门狗
    motor_can_timeout_ms: int = 200    # 电机侧看门狗（禁止 0）
    joints: list[JointConfig] = field(default_factory=list)

    @property
    def n_joints(self) -> int:
        return len(self.joints)

    @classmethod
    def from_yaml(cls, path: str) -> "ArmConfig":
        with open(path) as f:
            raw = yaml.safe_load(f)
        joints = [JointConfig(**j) for j in raw.pop("joints")]
        cfg = cls(joints=joints, **raw)
        cfg._validate()
        return cfg

    def _validate(self) -> None:
        if self.motor_can_timeout_ms <= 0:
            raise ValueError("motor_can_timeout_ms 必须 >0：电机侧 CAN_TIMEOUT 看门狗禁止关闭")
        ids = [j.motor_id for j in self.joints]
        if len(set(ids)) != len(ids):
            raise ValueError(f"motor_id 必须唯一: {ids}")
        for j in self.joints:
            if j.direction not in (1, -1):
                raise ValueError(f"电机 {j.motor_id}: direction 只能是 ±1，得到 {j.direction}")
            if not j.lo < j.hi:
                raise ValueError(f"电机 {j.motor_id}: 限位必须 lo < hi，得到 [{j.lo}, {j.hi}]")
            if not (protocol.KD_MIN <= j.kd <= protocol.KD_MAX):
                raise ValueError(f"电机 {j.motor_id}: kd 超协议范围 [0,5]，得到 {j.kd}")
            if not (protocol.KP_MIN <= j.kp <= protocol.KP_MAX):
                raise ValueError(f"电机 {j.motor_id}: kp 超协议范围 [0,500]，得到 {j.kp}")
