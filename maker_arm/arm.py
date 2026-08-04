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

    # ── 使能与控制循环 ──
    def enable(self, start_loop: bool = True) -> None:
        if self._state is not ArmState.CONNECTED:
            raise StateError(f"enable() 需要 CONNECTED 态，当前 {self._state.name}")
        for m in self.motors:
            m.write_param(protocol.ParamIndex.RUN_MODE, 0, "u8")          # 0=运控(MIT)
            m.write_param(protocol.ParamIndex.CAN_TIMEOUT,
                          self.config.motor_can_timeout_ms, "u32")        # 电机侧看门狗
        # 首帧保护：目标 = 当前实际位置（probe 刷新反馈，重试 10 次）
        for _ in range(10):
            self.refresh()
            time.sleep(0.02)
            if all(m.feedback_age < 0.5 for m in self.motors):
                break
        pos = self.get_joint_positions()
        if any(math.isnan(x) for x in pos):
            raise ConnectError("使能前读不到全部关节反馈，拒绝使能")
        with self._target_lock:
            self._user_targets = list(pos)
        self._internal = list(pos)
        for m in self.motors:
            m.enable()
        self._state = ArmState.ENABLED
        self.fault_reason = None
        if start_loop:
            self._running = True
            self._loop_thread = threading.Thread(target=self._control_loop,
                                                 daemon=True, name="maker-arm-ctrl")
            self._loop_thread.start()

    def disable(self) -> None:
        self._running = False
        if self._loop_thread:
            self._loop_thread.join(timeout=1.0)
            self._loop_thread = None
        for m in self.motors:
            m.disable()
        if self._state in (ArmState.ENABLED, ArmState.FAULT):
            self._state = ArmState.CONNECTED

    def estop(self) -> None:
        """任何状态可调：立即失能全部电机。"""
        self._running = False
        for m in self.motors:
            m.disable()
        if self._state is ArmState.ENABLED:
            self._state = ArmState.CONNECTED

    def set_joint_targets(self, targets: list[float]) -> bool:
        if self._state is not ArmState.ENABLED or len(targets) != self.config.n_joints:
            log.warning("set_joint_targets 被拒绝: state=%s", self._state.name)
            return False
        with self._target_lock:
            self._user_targets = list(targets)
        return True

    # ── 控制循环 ──
    @staticmethod
    def _busy_wait_us(us: float) -> None:
        end = time.perf_counter() + us / 1e6
        while time.perf_counter() < end:
            pass

    def _tick(self, dt: float) -> None:
        with self._target_lock:
            user = list(self._user_targets)
        maxstep = self.config.max_velocity * dt
        for i, j in enumerate(self.config.joints):
            lo, hi = j.lo + self.config.limit_margin, j.hi - self.config.limit_margin
            goal = min(hi, max(lo, user[i]))
            step = min(maxstep, max(-maxstep, goal - self._internal[i]))
            self._internal[i] += step
            self.motors[i].send_mit(self._to_motor(i, self._internal[i]), 0.0, j.kp, j.kd, 0.0)
            self._busy_wait_us(150)   # 拉开帧间隔防总线拥塞
        self._check_health()

    def _control_loop(self) -> None:
        dt = 1.0 / self.config.control_rate_hz
        next_t = time.perf_counter()
        while self._running:
            self._tick(dt)
            next_t += dt
            remain = next_t - time.perf_counter()
            if remain > 0:
                time.sleep(remain)
            else:
                next_t = time.perf_counter()   # 落后了就重置，不追帧

    def _check_health(self) -> None:
        for m in self.motors:
            if m.feedback_age > self.config.feedback_timeout:
                self._enter_fault(f"电机 {m.motor_id} 反馈超时 {m.feedback_age:.2f}s——查总线/电源")
                return
            fb = m.feedback
            if fb and fb.fault_bits:
                self._enter_fault(f"电机 {m.motor_id}: {fault_text(fb.fault_bits)} (bits=0b{fb.fault_bits:06b})")
                return

    def _enter_fault(self, reason: str) -> None:
        log.error("FAULT: %s", reason)
        self.fault_reason = reason
        self._running = False           # 控制线程随后自然退出
        for m in self.motors:
            m.disable()
        self._state = ArmState.FAULT

    def clear_faults(self) -> None:
        if self._state is not ArmState.FAULT:
            raise StateError(f"clear_faults() 需要 FAULT 态，当前 {self._state.name}")
        for m in self.motors:
            m.disable(clear_fault=True)
        self.fault_reason = None
        self._state = ArmState.CONNECTED
