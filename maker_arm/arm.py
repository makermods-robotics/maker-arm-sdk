"""Arm layer: state machine + control loop + all safety logic. Only exposes joint coordinates (SI units) externally."""

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
    MODE_FAULT_TICKS = 5   # threshold for consecutive ticks with mode≠2 (25ms at 200Hz) -- tolerates stale feedback right at enable
    ENABLE_LIMIT_GRACE = 0.35   # rad: allowed over-limit grace margin at enable time; beyond this = suspected 2π jump, refuse to enable
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
        self._wrap: list[float] = [0.0] * config.n_joints   # per-session 2π compensation (joint coordinates, computed at connect)

    @classmethod
    def from_yaml(cls, path: str, backend: str = "socketcan", **backend_kwargs) -> "Arm":
        return cls(ArmConfig.from_yaml(path), create_backend(backend, **backend_kwargs))

    @property
    def state(self) -> ArmState:
        return self._state

    # ── RX dispatch (transport-layer callback: fast in, fast out) ──
    def _on_frame(self, can_id: int, data: bytes) -> None:
        # For frames from a motor (feedback/param/fault), motor_id is always at Bit8~15 -- look up the model before decoding
        m = self._by_id.get((can_id >> 8) & 0xFF)
        msg = protocol.parse_frame(can_id, data, m.params if m else None)
        if msg is None:
            return
        m2 = self._by_id.get(msg.motor_id)
        if m2:
            m2.handle_frame(msg)

    # ── connection management ──
    def connect(self, timeout: float = 2.0) -> None:
        if self._state is not ArmState.DISCONNECTED:
            raise StateError(f"connect() requires DISCONNECTED state, currently {self._state.name}")
        self._backend.set_recv_callback(self._on_frame)
        self._backend.open()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            missing = [m for m in self.motors if m.feedback is None]
            if not missing:
                break
            for m in missing:
                sequence = m.feedback_sequence
                m.probe()
                # Probe sequentially and wait for this motor's reply before sending the
                # next request. Small USB-SLCAN firmware queues otherwise return only the
                # first few motors from a burst even though the whole bus is healthy.
                probe_deadline = min(deadline, time.monotonic() + 0.05)
                while m.feedback_sequence == sequence and time.monotonic() < probe_deadline:
                    time.sleep(0.001)
                if time.monotonic() >= deadline:
                    break
        missing_ids = [m.motor_id for m in self.motors if m.feedback is None]
        if missing_ids:
            online = [m.motor_id for m in self.motors if m.feedback is not None]
            self._backend.close()
            raise ConnectError(f"motor {missing_ids} has no feedback (online: {online or 'none'}) -- check wiring/termination resistor/power/ID")
        # Per-session 2π compensation: a motor restart shifts the reading by ±2π (multi-turn
        # count reset, which zero_sta cannot fix -- falsified on real hardware). The mechanical
        # limit travel is < one full turn, so a reading with a whole-turn offset physically
        # cannot be the real pose; moving it back into the limit range is therefore unambiguous
        # and safe. The compensation is transparent in both directions, for readings and commands.
        two_pi = 2 * math.pi
        self._wrap = [0.0] * self.config.n_joints
        for i, j in enumerate(self.config.joints):
            raw = self._to_joint(i, self.motors[i].feedback.position)   # _wrap already zeroed -> raw value
            if j.lo - self.ENABLE_LIMIT_GRACE <= raw <= j.hi + self.ENABLE_LIMIT_GRACE:
                continue
            for k in (-two_pi, two_pi):
                if j.lo - self.ENABLE_LIMIT_GRACE <= raw + k <= j.hi + self.ENABLE_LIMIT_GRACE:
                    self._wrap[i] = k
                    log.info("motor %d reading %+.3f rad includes a 2π jump, compensating %+.3f for this session",
                                j.motor_id, raw, k)
                    break
        self._state = ArmState.CONNECTED
        log.info("connected: %d motors online", len(self.motors))

    def disconnect(self) -> None:
        if self._state is ArmState.ENABLED:
            self.disable()
        self._backend.close()
        self._state = ArmState.DISCONNECTED

    def refresh(self, wait: bool = False, timeout: float = 0.05) -> Optional[list[bool]]:
        """Poll feedback while not enabled.

        The default remains non-blocking for existing callers. ``wait=True`` probes one motor
        at a time and verifies a new feedback sequence before probing the next. This avoids
        accepting stale cached values when small USB-SLCAN queues drop burst replies.
        """
        if self._state is ArmState.ENABLED:
            return [False] * len(self.motors) if wait else None
        if timeout <= 0:
            raise ValueError("refresh timeout must be positive")
        if not wait:
            for m in self.motors:
                m.probe()
            return None
        fresh = []
        for m in self.motors:
            sequence = m.feedback_sequence
            m.probe()
            deadline = time.monotonic() + timeout
            while m.feedback_sequence == sequence and time.monotonic() < deadline:
                time.sleep(0.001)
            fresh.append(m.feedback_sequence != sequence)
        return fresh

    # ── coordinate conversion (direction/offset only appear in these two functions) ──
    def _to_motor(self, i: int, joint_pos: float) -> float:
        j = self.config.joints[i]
        return (joint_pos - self._wrap[i]) * j.direction + j.offset

    def _to_joint(self, i: int, motor_pos: float) -> float:
        j = self.config.joints[i]
        return (motor_pos - j.offset) * j.direction + self._wrap[i]

    # ── feedback getters (joint coordinates; no feedback -> nan / 0) ──
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

    # ── enabling and the control loop ──
    def enable(self, start_loop: bool = True) -> None:
        if self._state is not ArmState.CONNECTED:
            raise StateError(f"enable() requires CONNECTED state, currently {self._state.name}")
        # canTimeout protocol unit is 50µs/count (20000=1s): milliseconds -> count value ×20
        timeout_counts = self.config.motor_can_timeout_ms * protocol.CAN_TIMEOUT_PER_MS
        for m in self.motors:
            m.write_param(protocol.ParamIndex.RUN_MODE, 0, "u8")          # 0=control mode (MIT)
            m.write_param(protocol.ParamIndex.CAN_TIMEOUT, timeout_counts, "u32")  # motor-side watchdog
            time.sleep(0.005)  # keep small SLCAN firmware TX queues from being burst-filled
        # Read-back verification: confirm the motor actually accepted the RUN_MODE / CAN_TIMEOUT settings
        for m in self.motors:
            values = []
            for index, dtype in (
                (protocol.ParamIndex.RUN_MODE, "u8"),
                (protocol.ParamIndex.CAN_TIMEOUT, "u32"),
            ):
                last_error = None
                for attempt in range(5):
                    try:
                        values.append(m.read_param(index, dtype=dtype, timeout=0.2))
                        break
                    except ParamTimeout as e:
                        last_error = e
                        if attempt < 4:
                            time.sleep(0.02)
                else:
                    raise ConnectError(f"param read-back before enable failed after 5 attempts: {last_error}") from last_error
            run_mode, rb = values
            if run_mode != 0:
                raise ConnectError(f"motor {m.motor_id} RUN_MODE read-back {run_mode} != 0 (control mode not in effect)")
            if rb != timeout_counts:
                log.warning("motor %d CAN_TIMEOUT read-back %s != expected %d (=%dms×20)",
                            m.motor_id, rb, timeout_counts, self.config.motor_can_timeout_ms)
        # First-frame protection: target = a fresh current position from every motor.
        # Sequential probing avoids mistaking plausible cached SLCAN data for a new reply.
        fresh = self.refresh(wait=True, timeout=0.05)
        if not all(fresh):
            stale = [self.motors[i].motor_id for i, ok in enumerate(fresh) if not ok]
            raise ConnectError(
                f"could not obtain fresh feedback for CAN motors {stale} before enable, refusing to enable"
            )
        pos = self.get_joint_positions()
        if any(math.isnan(x) for x in pos):
            raise ConnectError("could not read feedback for all joints before enable, refusing to enable")
        # Absolute-coordinate safety gate: a current position far outside the calibrated
        # mechanical envelope indicates a bad zero or unexplained encoder wrap. Never shift
        # the envelope to accommodate it; refuse to enable instead.
        for i, j in enumerate(self.config.joints):
            if pos[i] < j.lo - self.ENABLE_LIMIT_GRACE or pos[i] > j.hi + self.ENABLE_LIMIT_GRACE:
                raise ConnectError(
                    f"motor {j.motor_id} current position {pos[i]:+.3f} rad far exceeds soft limits [{j.lo}, {j.hi}]"
                    f"(grace {self.ENABLE_LIMIT_GRACE}) -- suspected 2π jump or uncalibrated zero, refusing to enable. "
                    "Troubleshoot with 'maker-arm scan'; if a replacement motor needs a new reference, use 'maker-arm zero'")
        with self._target_lock:
            self._user_targets = list(pos)
        self._internal = list(pos)
        for m in self.motors:
            m.enable()
            time.sleep(0.005)  # avoid burst-filling small USB-SLCAN firmware TX queues
        self._state = ArmState.ENABLED
        self.fault_reason = None
        self._mode_bad.clear()
        if start_loop:
            self._running = True
            self._loop_thread = threading.Thread(target=self._control_loop,
                                                 daemon=True, name="maker-arm-ctrl")
            self._loop_thread.start()

    def disable(self) -> None:
        if self._state is ArmState.DISCONNECTED:
            return
        self._running = False
        if self._loop_thread and threading.current_thread() is not self._loop_thread:
            self._loop_thread.join(timeout=1.0)
            self._loop_thread = None
        for m in self.motors:
            try:
                m.disable()
            except Exception as e:
                log.error("failed to disable motor %d: %s", m.motor_id, e)
        if self._state in (ArmState.ENABLED, ArmState.FAULT):
            self._state = ArmState.CONNECTED

    def estop(self) -> None:
        """Callable from any state: immediately disables all motors."""
        self._running = False
        if self._loop_thread and threading.current_thread() is not self._loop_thread:
            self._loop_thread.join(timeout=1.0)
            self._loop_thread = None
        for m in self.motors:
            try:
                m.disable()
            except Exception as e:
                log.error("failed to disable motor %d during e-stop: %s", m.motor_id, e)
        if self._state is ArmState.ENABLED:
            self._state = ArmState.CONNECTED

    def set_joint_targets(self, targets: list[float]) -> bool:
        if self._state is not ArmState.ENABLED or len(targets) != self.config.n_joints:
            log.warning("set_joint_targets rejected: state=%s", self._state.name)
            return False
        if not all(math.isfinite(t) for t in targets):
            log.warning("set_joint_targets rejected: targets contain non-finite values %s", targets)
            return False
        with self._target_lock:
            self._user_targets = list(targets)
        return True

    def get_commanded_positions(self) -> list[float]:
        """Return the rate-limited joint command currently sent by the control loop."""
        with self._target_lock:
            return list(self._internal)

    def hold_current_position(self) -> bool:
        """Freeze the enabled arm at its latest measured pose without releasing torque."""
        if self._state is not ArmState.ENABLED:
            return False
        positions = self.get_joint_positions()
        if not all(math.isfinite(position) for position in positions):
            return False
        with self._target_lock:
            self._user_targets = list(positions)
            self._internal = list(positions)
        return True

    # ── control loop ──
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
            commands = list(self._internal)
        for i, j in enumerate(self.config.joints):
            motor = self.motors[i]
            sequence = motor.feedback_sequence
            motor.send_mit(self._to_motor(i, commands[i]), 0.0, j.kp, j.kd, 0.0)
            if getattr(self._backend, "feedback_paced_control", False):
                # Old USB-SLCAN firmware has no outbound USB queue. Waiting for
                # this reply before commanding the next motor bounds both its
                # CAN TX queue and three-frame CAN RX FIFO. A timeout does not
                # stop the remaining motors from receiving their commands; the
                # normal feedback watchdog below still makes the safety decision.
                deadline = time.monotonic() + getattr(
                    self._backend, "feedback_wait_timeout", 0.015
                )
                while motor.feedback_sequence == sequence and time.monotonic() < deadline:
                    time.sleep(0.0005)
            else:
                self._busy_wait_us(150)   # space out frame intervals to prevent bus congestion
        self._check_health()

    def _control_loop(self) -> None:
        dt = 1.0 / self.config.control_rate_hz
        next_t = time.perf_counter()
        while self._running and threading.current_thread() is self._loop_thread:
            try:
                self._tick(dt)
            except Exception as e:
                self._enter_fault(f"control loop exception: {e}")
                break
            next_t += dt
            remain = next_t - time.perf_counter()
            if remain > 0:
                time.sleep(remain)
            else:
                next_t = time.perf_counter()   # if we've fallen behind, reset rather than trying to catch up

    def _check_health(self) -> None:
        if self._state is not ArmState.ENABLED:
            return   # already in FAULT hold state, etc.: don't re-evaluate fault
        for m in self.motors:
            if m.feedback_age > self.config.feedback_timeout:
                self._enter_fault(f"motor {m.motor_id} feedback timeout {m.feedback_age:.2f}s -- check bus/power")
                return
            fb = m.feedback
            if fb and fb.fault_bits:
                self._enter_fault(f"motor {m.motor_id}: {fault_text(fb.fault_bits)} (bits=0b{fb.fault_bits:06b})")
                return
            # The mode check has persistence tolerance: right after enabling, the cache may still
            # hold stale mode=0 feedback from the probing era (real-hardware race, millisecond
            # scale) -- only judged a real fault after MODE_FAULT_TICKS consecutive abnormal ticks.
            if fb and fb.mode != 2:
                i = self._mode_bad.setdefault(m.motor_id, 0) + 1
                self._mode_bad[m.motor_id] = i
                if i >= self.MODE_FAULT_TICKS:
                    self._enter_fault(f"motor {m.motor_id} mode anomaly mode={fb.mode} (not in control mode, persisted {i} ticks)")
                    return
            else:
                self._mode_bad[m.motor_id] = 0

    def _enter_fault(self, reason: str) -> None:
        log.error("FAULT: %s", reason)
        self.fault_reason = reason
        if self.config.hold_on_fault:
            # Lock and hold (prevents a drop): the target freezes at the current readable
            # position, the control loop keeps sending hold frames, and the arm is locked in
            # place by kp. A motor that's unreachable or has dropped out of control mode cannot
            # be locked (it releases torque via its own canTimeout) -- everything that can be
            # locked, is. Released by: disable()/estop()/clear_faults() (all of which release torque).
            pos = self.get_joint_positions()
            with self._target_lock:
                for i, p_ in enumerate(pos):
                    if math.isfinite(p_):
                        self._internal[i] = p_
                        self._user_targets[i] = p_
            self._state = ArmState.FAULT
            log.error("hold_on_fault: current pose locked and held (loop continues); disable()/Ctrl-C releases torque")
            return
        self._running = False           # the control thread then exits naturally
        for m in self.motors:
            try:
                m.disable()
            except Exception as e:
                log.error("failed to disable motor %d: %s", m.motor_id, e)
        self._state = ArmState.FAULT

    def clear_faults(self) -> None:
        if self._state is not ArmState.FAULT:
            raise StateError(f"clear_faults() requires FAULT state, currently {self._state.name}")
        for m in self.motors:
            m.disable(clear_fault=True)
        self.fault_reason = None
        self._state = ArmState.CONNECTED
