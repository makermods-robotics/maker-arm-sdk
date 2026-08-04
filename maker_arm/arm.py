"""机械臂层：状态机 + 控制循环 + 全部安全逻辑。对外只暴露关节坐标（SI 单位）。"""

import logging
import math
import threading
import time
from enum import Enum
from typing import Optional

from . import protocol
from .config import ArmConfig
from .errors import ConnectError, StateError, fault_text
from .motor import Motor
from .transport.base import CanBackend, create_backend

log = logging.getLogger("maker_arm")


class ArmState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ENABLED = "enabled"
    FAULT = "fault"


class Arm:
    def __init__(self, config: ArmConfig, backend: CanBackend):
        self.config = config
        self._backend = backend
        self.motors = [Motor(j.motor_id, backend) for j in config.joints]
        self._by_id = {m.motor_id: m for m in self.motors}
        self._state = ArmState.DISCONNECTED
        self._target_lock = threading.Lock()
        self._user_targets: list[float] = [0.0] * config.n_joints
        self._internal: list[float] = [0.0] * config.n_joints
        self._running = False
        self._loop_thread: Optional[threading.Thread] = None
        self.fault_reason: Optional[str] = None

    @classmethod
    def from_yaml(cls, path: str, backend: str = "socketcan", **backend_kwargs) -> "Arm":
        return cls(ArmConfig.from_yaml(path), create_backend(backend, **backend_kwargs))

    @property
    def state(self) -> ArmState:
        return self._state

    # ── RX 分发（传输层回调：快进快出） ──
    def _on_frame(self, can_id: int, data: bytes) -> None:
        msg = protocol.parse_frame(can_id, data)
        if msg is None:
            return
        m = self._by_id.get(msg.motor_id)
        if m:
            m.handle_frame(msg)

    # ── 连接管理 ──
    def connect(self, timeout: float = 2.0) -> None:
        if self._state is not ArmState.DISCONNECTED:
            raise StateError(f"connect() 需要 DISCONNECTED 态，当前 {self._state.name}")
        self._backend.set_recv_callback(self._on_frame)
        self._backend.open()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            missing = [m for m in self.motors if m.feedback is None]
            if not missing:
                break
            for m in missing:
                m.probe()
            time.sleep(0.05)
        missing_ids = [m.motor_id for m in self.motors if m.feedback is None]
        if missing_ids:
            online = [m.motor_id for m in self.motors if m.feedback is not None]
            self._backend.close()
            raise ConnectError(f"电机 {missing_ids} 无反馈（在线: {online or '无'}）——查接线/终端电阻/电源/ID")
        self._state = ArmState.CONNECTED
        log.info("connected: %d motors online", len(self.motors))

    def disconnect(self) -> None:
        if self._state is ArmState.ENABLED:
            self.disable()
        self._backend.close()
        self._state = ArmState.DISCONNECTED

    def refresh(self) -> None:
        """非 ENABLED 态轮询反馈（probe=停止帧，勿在使能时调用）。"""
        if self._state is ArmState.ENABLED:
            return
        for m in self.motors:
            m.probe()

    # ── 坐标换算（方向/偏移只出现在这两个函数） ──
    def _to_motor(self, i: int, joint_pos: float) -> float:
        j = self.config.joints[i]
        return joint_pos * j.direction + j.offset

    def _to_joint(self, i: int, motor_pos: float) -> float:
        j = self.config.joints[i]
        return (motor_pos - j.offset) * j.direction

    # ── 反馈 getter（关节坐标；无反馈 → nan / 0） ──
    def get_joint_positions(self) -> list[float]:
        return [self._to_joint(i, m.feedback.position) if m.feedback else math.nan
                for i, m in enumerate(self.motors)]

    def get_joint_velocities(self) -> list[float]:
        return [m.feedback.velocity * self.config.joints[i].direction if m.feedback else math.nan
                for i, m in enumerate(self.motors)]

    def get_joint_torques(self) -> list[float]:
        return [m.feedback.torque * self.config.joints[i].direction if m.feedback else math.nan
                for i, m in enumerate(self.motors)]

    def get_temperatures(self) -> list[float]:
        return [m.feedback.temperature if m.feedback else math.nan for m in self.motors]

    def get_faults(self) -> list[int]:
        return [m.feedback.fault_bits if m.feedback else 0 for m in self.motors]
