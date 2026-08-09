"""机械臂层：状态机 + 控制循环 + 全部安全逻辑。对外只暴露关节坐标（SI 单位）。"""

import logging
import math
import threading
import time
from enum import Enum
from typing import Optional

from . import protocol
from .config import ArmConfig
from .errors import ConnectError, ParamTimeout, StateError, fault_text
from .motor import Motor
from .transport.base import CanBackend, create_backend

log = logging.getLogger("maker_arm")


class ArmState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ENABLED = "enabled"
    FAULT = "fault"


class Arm:
    MODE_FAULT_TICKS = 5   # mode≠2 连续拍数阈值（200Hz 下 25ms）——容忍使能瞬间的陈旧反馈
    ENABLE_LIMIT_GRACE = 0.35   # rad：使能时允许的越限宽限；超过=疑似 2π 跳变，拒绝使能
    def __init__(self, config: ArmConfig, backend: CanBackend):
        self.config = config
        self._backend = backend
        self.motors = [Motor(j.motor_id, backend,
                             params=protocol.MOTOR_PARAMS[j.model]) for j in config.joints]
        self._by_id = {m.motor_id: m for m in self.motors}
        self._state = ArmState.DISCONNECTED
        self._target_lock = threading.Lock()
        self._user_targets: list[float] = [0.0] * config.n_joints
        self._internal: list[float] = [0.0] * config.n_joints
        self._running = False
        self._loop_thread: Optional[threading.Thread] = None
        self.fault_reason: Optional[str] = None
        self._mode_bad: dict[int, int] = {}
        self._wrap: list[float] = [0.0] * config.n_joints   # 会话内 2π 补偿（joint 坐标，connect 时计算）

    @classmethod
    def from_yaml(cls, path: str, backend: str = "socketcan", **backend_kwargs) -> "Arm":
        return cls(ArmConfig.from_yaml(path), create_backend(backend, **backend_kwargs))

    @property
    def state(self) -> ArmState:
        return self._state

    # ── RX 分发（传输层回调：快进快出） ──
    def _on_frame(self, can_id: int, data: bytes) -> None:
        # 电机发来的帧（feedback/param/fault）motor_id 都在 Bit8~15——先查型号再解码
        m = self._by_id.get((can_id >> 8) & 0xFF)
        msg = protocol.parse_frame(can_id, data, m.params if m else None)
        if msg is None:
            return
        m2 = self._by_id.get(msg.motor_id)
        if m2:
            m2.handle_frame(msg)

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
        # 会话内 2π 补偿：电机重启会把读数搬移 ±2π（多圈计数复位，zero_sta 治不了——
        # 真机证伪）。装配限位行程 < 一圈，读数带整圈偏移物理上不可能是真实姿态，
        # 因此把它移回限位区间是无歧义且安全的；补偿对读数和指令双向透明。
        two_pi = 2 * math.pi
        self._wrap = [0.0] * self.config.n_joints
        for i, j in enumerate(self.config.joints):
            raw = self._to_joint(i, self.motors[i].feedback.position)   # _wrap 已清零 → 原始值
            if j.lo - self.ENABLE_LIMIT_GRACE <= raw <= j.hi + self.ENABLE_LIMIT_GRACE:
                continue
            for k in (-two_pi, two_pi):
                if j.lo - self.ENABLE_LIMIT_GRACE <= raw + k <= j.hi + self.ENABLE_LIMIT_GRACE:
                    self._wrap[i] = k
                    log.warning("电机 %d 读数 %+.3f rad 含 2π 跳变，会话内补偿 %+.3f",
                                j.motor_id, raw, k)
                    break
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
        return (joint_pos - self._wrap[i]) * j.direction + j.offset

    def _to_joint(self, i: int, motor_pos: float) -> float:
        j = self.config.joints[i]
        return (motor_pos - j.offset) * j.direction + self._wrap[i]

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
        # canTimeout 协议单位为 50µs/计数（20000=1s）：毫秒 → 计数值 ×20
        timeout_counts = self.config.motor_can_timeout_ms * protocol.CAN_TIMEOUT_PER_MS
        for m in self.motors:
            m.write_param(protocol.ParamIndex.RUN_MODE, 0, "u8")          # 0=运控(MIT)
            m.write_param(protocol.ParamIndex.CAN_TIMEOUT, timeout_counts, "u32")  # 电机侧看门狗
        # 回读校验：确认电机确实接受了 RUN_MODE / CAN_TIMEOUT 设置
        for m in self.motors:
            try:
                run_mode = m.read_param(protocol.ParamIndex.RUN_MODE, dtype="u8")
                if run_mode != 0:
                    raise ConnectError(f"电机 {m.motor_id} RUN_MODE 回读 {run_mode}≠0（运控模式未生效）")
                rb = m.read_param(protocol.ParamIndex.CAN_TIMEOUT, dtype="u32")
                if rb != timeout_counts:
                    log.warning("电机 %d CAN_TIMEOUT 回读 %s ≠ 期望 %d（=%dms×20）",
                                m.motor_id, rb, timeout_counts, self.config.motor_can_timeout_ms)
            except ParamTimeout as e:
                raise ConnectError(f"使能前参数回读失败: {e}") from e
        # 首帧保护：目标 = 当前实际位置（probe 刷新反馈，重试 10 次）
        for _ in range(10):
            self.refresh()
            time.sleep(0.02)
            if all(m.feedback_age < 0.5 for m in self.motors):
                break
        pos = self.get_joint_positions()
        if any(math.isnan(x) for x in pos):
            raise ConnectError("使能前读不到全部关节反馈，拒绝使能")
        # 2π 跳变守门：位置远超软限位时拒绝使能——否则控制循环会默默把关节
        # 大幅钳回限位（真机事故：J1 每次使能被拽半圈）。宽限内的小越限仍允许（钳回是安全的）。
        for i, j in enumerate(self.config.joints):
            if pos[i] < j.lo - self.ENABLE_LIMIT_GRACE or pos[i] > j.hi + self.ENABLE_LIMIT_GRACE:
                raise ConnectError(
                    f"电机 {j.motor_id} 当前位置 {pos[i]:+.3f} rad 远超软限位 [{j.lo}, {j.hi}]"
                    f"（宽限 {self.ENABLE_LIMIT_GRACE}）——疑似 2π 跳变或未标零，拒绝使能。"
                    "排查: tools/scan_bus.py 看读数；必要时 tools/set_zero.py 重新标零")
        with self._target_lock:
            self._user_targets = list(pos)
        self._internal = list(pos)
        for m in self.motors:
            m.enable()
        self._state = ArmState.ENABLED
        self.fault_reason = None
        self._mode_bad.clear()
        if start_loop:
            self._running = True
            self._loop_thread = threading.Thread(target=self._control_loop,
                                                 daemon=True, name="maker-arm-ctrl")
            self._loop_thread.start()

    def disable(self) -> None:
        self._running = False
        if self._loop_thread and threading.current_thread() is not self._loop_thread:
            self._loop_thread.join(timeout=1.0)
            self._loop_thread = None
        for m in self.motors:
            try:
                m.disable()
            except Exception as e:
                log.error("失能电机 %d 失败: %s", m.motor_id, e)
        if self._state in (ArmState.ENABLED, ArmState.FAULT):
            self._state = ArmState.CONNECTED

    def estop(self) -> None:
        """任何状态可调：立即失能全部电机。"""
        self._running = False
        if self._loop_thread and threading.current_thread() is not self._loop_thread:
            self._loop_thread.join(timeout=1.0)
            self._loop_thread = None
        for m in self.motors:
            try:
                m.disable()
            except Exception as e:
                log.error("急停失能电机 %d 失败: %s", m.motor_id, e)
        if self._state is ArmState.ENABLED:
            self._state = ArmState.CONNECTED

    def set_joint_targets(self, targets: list[float]) -> bool:
        if self._state is not ArmState.ENABLED or len(targets) != self.config.n_joints:
            log.warning("set_joint_targets 被拒绝: state=%s", self._state.name)
            return False
        if not all(math.isfinite(t) for t in targets):
            log.warning("set_joint_targets 被拒绝: 目标包含非有限值 %s", targets)
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
        while self._running and threading.current_thread() is self._loop_thread:
            try:
                self._tick(dt)
            except Exception as e:
                self._enter_fault(f"控制循环异常: {e}")
                break
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
            # mode 检查带持续性容忍：使能后头几拍缓存里可能还是探测时代的
            # mode=0 旧反馈（真机竞态，毫秒级），连续 MODE_FAULT_TICKS 拍异常才判真故障。
            if fb and fb.mode != 2:
                i = self._mode_bad.setdefault(m.motor_id, 0) + 1
                self._mode_bad[m.motor_id] = i
                if i >= self.MODE_FAULT_TICKS:
                    self._enter_fault(f"电机 {m.motor_id} 模式异常 mode={fb.mode}（未在运控状态，持续 {i} 拍）")
                    return
            else:
                self._mode_bad[m.motor_id] = 0

    def _enter_fault(self, reason: str) -> None:
        log.error("FAULT: %s", reason)
        self.fault_reason = reason
        self._running = False           # 控制线程随后自然退出
        for m in self.motors:
            try:
                m.disable()
            except Exception as e:
                log.error("失能电机 %d 失败: %s", m.motor_id, e)
        self._state = ArmState.FAULT

    def clear_faults(self) -> None:
        if self._state is not ArmState.FAULT:
            raise StateError(f"clear_faults() 需要 FAULT 态，当前 {self._state.name}")
        for m in self.motors:
            m.disable(clear_fault=True)
        self.fault_reason = None
        self._state = ArmState.CONNECTED
