"""Arm layer: state machine + control loop + all safety logic. Only exposes joint coordinates (SI units) externally.

Behaviour mirrors MakerModsLab driving a Maker Arm v1 through LeRobot's MakerFollower:
non-clearing fault checks, startup sync from the measured pose, full-turn correction,
latched fault evidence, gripper holding-effort regulation and temperature protection, and a
verified return to rest before torque is released.
"""

import logging
import math
import threading
import time
from enum import Enum
from typing import Optional

from . import protocol
from .config import ArmConfig
from .errors import ConnectError, FaultLatched, ParamTimeout, StateError
from .gripper import GripperHold
from .motor import Motor
from .transport.base import CanBackend, create_backend

log = logging.getLogger("maker_arm")


class ArmState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ENABLED = "enabled"
    FAULT = "fault"


class Arm:
    MODE_FAULT_TICKS = 5   # consecutive ticks a motor may report mode != run before it is a fault
    REPLY_TIMEOUT = 0.05   # s, per request (LeRobot handshake / MakerModsLab fresh reads)
    REPLY_ATTEMPTS = 3
    PARAM_TIMEOUT = 0.12   # s, typed parameter replies (MakerModsLab gripper hold)

    def __init__(self, config: ArmConfig, backend: CanBackend):
        config._validate()
        self.config = config
        self._backend = backend
        self.motors = [Motor(j.motor_id, backend, protocol.MOTOR_PARAMS[j.model]) for j in config.joints]
        self._by_id = {m.motor_id: m for m in self.motors}
        self._state = ArmState.DISCONNECTED
        self._lifecycle = threading.RLock()   # connect/enable/disable/stop/clear_faults
        self._generation = 0                  # bumped by estop() to cancel an enable in progress
        self._enable_gate = threading.Lock()  # estop() vs. an enable frame about to go out
        self._session = 0                     # bumped on disable: a stale loop thread may not send
        self._target_lock = threading.Lock()
        n = config.n_joints
        self._user_targets: list[float] = [0.0] * n
        self._internal: list[float] = [0.0] * n
        self._synced: list[bool] = [True] * n
        self._targets_received = False
        self._stuck_anchor: list[tuple[float, float]] = [(0.0, 0.0)] * n
        self._returning = False               # a return to rest owns the targets
        self._released: set[int] = set()      # disabled after a fault (release_on_fault)
        self._faulted: set[int] = set()       # no longer commanded
        self._watchdog_restore: dict[int, int] = {}
        self._hold: dict[int, GripperHold] = {}
        self._hold_out: dict[int, tuple[int, float, float, float]] = {}   # seq, position, kp, kd
        self._release_sent: dict[int, float] = {}
        self._poll_sent: dict[int, float] = {}
        self._tick_count = 0
        self._running = False
        self._loop_thread: Optional[threading.Thread] = None
        self.fault_reason: Optional[str] = None
        self.fault_reasons: list[str] = []
        self.unconfirmed_disable: list[int] = []
        self._mode_bad: dict[int, int] = {}
        self._wrap: list[float] = [0.0] * n   # per-session full-turn correction (joint coordinates)
        self.rest_pose: Optional[list[float]] = None

    @classmethod
    def from_yaml(cls, path: str, backend: str = "socketcan", **backend_kwargs) -> "Arm":
        return cls(ArmConfig.from_yaml(path), create_backend(backend, **backend_kwargs))

    @property
    def state(self) -> ArmState:
        return self._state

    # ── RX dispatch (transport-layer callback: fast in, fast out) ──
    def _on_frame(self, can_id: int, data: bytes) -> None:
        m = self._by_id.get(protocol.reply_motor_id(can_id, data))
        if m is None:
            return
        m.handle_frame(protocol.parse_frame(can_id, data, m.params, self.config.host_id))

    # ── connection management ──
    def connect(self, clear_faults: bool = False) -> None:
        """Find every motor without changing torque state.

        Each motor first gets a non-clearing fault query. A faulted motor is reported and the
        fault is kept (pass ``clear_faults=True`` to clear it explicitly). Fault-free motors
        then return a fresh position. Torque is never enabled or released here.
        """
        with self._lifecycle:
            if self._state is not ArmState.DISCONNECTED:
                raise StateError(f"connect() requires DISCONNECTED state, currently {self._state.name}")
            for m in self.motors:
                m.reset_cache()
            self._released.clear()
            self._faulted.clear()
            self._backend.set_recv_callback(self._on_frame)
            self._backend.open()
            try:
                self._check_motors(clear_faults)
                for m in self.motors:
                    if m.probe_params(self.PARAM_TIMEOUT, attempts=2):
                        run_mode = m.read_param(protocol.ParamIndex.RUN_MODE, "u8", self.PARAM_TIMEOUT)
                        if run_mode != 0:
                            raise ConnectError(f"motor {m.motor_id} RUN_MODE is {run_mode}, not 0 (MIT control)")
            except BaseException:
                self._backend.close()
                raise
            # Full-turn correction (LeRobot MakerFollower): a power cycle can reset the motor's
            # multi-turn count, shifting a reading by ±2π. Joint travel is under one turn, so a
            # reading outside the limits that lands inside them after a whole-turn shift is
            # unambiguous. Anything else is a stale zero and enable() refuses it.
            two_pi = 2 * math.pi
            grace = self.config.full_turn_grace
            self._wrap = [0.0] * self.config.n_joints
            for i, j in enumerate(self.config.joints):
                raw = self._to_joint(i, self.motors[i].feedback.position)
                if j.lo - grace <= raw <= j.hi + grace:
                    continue
                for k in (-two_pi, two_pi):
                    if j.lo - grace <= raw + k <= j.hi + grace:
                        self._wrap[i] = k
                        log.info("%s reading %+.3f rad is one turn off, correcting %+.3f for this session",
                                 j.label, raw, k)
                        break
            self._state = ArmState.CONNECTED
            log.info("connected: %d motors online", len(self.motors))

    def _check_motors(self, clear_faults: bool = False) -> None:
        """Non-clearing fault query, then a fresh state read, for every motor."""
        missing, faulted = [], []
        for m, j in zip(self.motors, self.config.joints):
            try:
                status = m.query_fault(self.REPLY_TIMEOUT, self.REPLY_ATTEMPTS)
            except ParamTimeout:
                missing.append(m.motor_id)
                continue
            if status.has_fault and clear_faults:
                log.warning("%s: clearing fault 0x%08X (%s) on request", j.label, status.word, status.text)
                m.clear_fault()
                time.sleep(0.01)
                status = m.query_fault(self.REPLY_TIMEOUT, self.REPLY_ATTEMPTS)
            if status.has_fault:
                faulted.append(f"{m.motor_id} ({j.label}: {status.text})")
        if not missing and not faulted:
            for m, j in zip(self.motors, self.config.joints):
                try:
                    m.read_state(self.REPLY_TIMEOUT, self.REPLY_ATTEMPTS)
                except ParamTimeout:
                    missing.append(m.motor_id)
                except FaultLatched:
                    pass
                if m.fault_latched:
                    faulted.append(f"{m.motor_id} ({j.label}: {m.fault_latched})")
        if missing or faulted:
            online = [m.motor_id for m in self.motors if m.motor_id not in missing]
            parts = []
            if missing:
                parts.append(f"no reply from motors {missing} (online: {online or 'none'}) -- check power, "
                             "wiring, termination, 1 Mbps, MIT protocol and that no other program holds the adapter")
            if faulted:
                parts.append(f"motors reporting a fault: {', '.join(faulted)} -- fault kept for inspection; "
                             "clear it explicitly (maker-arm scan --clear-faults) once the cause is fixed")
            raise ConnectError("; ".join(parts))

    def disconnect(self) -> None:
        """Release torque immediately and close the bus. Call stop() first for a safe return."""
        with self._lifecycle:
            if self._state in (ArmState.ENABLED, ArmState.FAULT):
                self.disable()
            self._backend.close()
            self._state = ArmState.DISCONNECTED

    def refresh(self, wait: bool = True, timeout: float = 0.05) -> Optional[list[bool]]:
        """Request fresh feedback while not enabled. Never changes torque state or clears a reported fault.

        Each motor gets a non-clearing fault query first; only a fault-free motor is asked for
        its position. With ``wait=True`` (default) the result says which motors replied.
        """
        if self._state in (ArmState.ENABLED, ArmState.FAULT):
            return [False] * len(self.motors) if wait else None
        if timeout <= 0:
            raise ValueError("refresh timeout must be positive")
        fresh = []
        for m in self.motors:
            try:
                m.read_state_safe(timeout)
                fresh.append(True)
            except (ParamTimeout, FaultLatched):
                fresh.append(False)
        return fresh if wait else None

    # ── coordinate conversion (direction/offset only appear in these two functions) ──
    def _to_motor(self, i: int, joint_pos: float) -> float:
        j = self.config.joints[i]
        return (joint_pos - self._wrap[i]) * j.direction + j.offset

    def _to_joint(self, i: int, motor_pos: float) -> float:
        j = self.config.joints[i]
        return (motor_pos - j.offset) * j.direction + self._wrap[i]

    # ── feedback getters (joint coordinates; no feedback -> nan / None) ──
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

    def get_faults(self) -> list[Optional[int]]:
        """Latest MIT fault word per motor (0 = no fault, None = never queried)."""
        return [m.fault_status.word if m.fault_status else None for m in self.motors]

    # ── enabling ──
    def enable(self, start_loop: bool = True) -> None:
        with self._lifecycle:
            if self._state is not ArmState.CONNECTED:
                raise StateError(f"enable() requires CONNECTED state, currently {self._state.name}")
            if self._loop_thread is not None and self._loop_thread.is_alive():
                raise StateError("the previous control loop is still running; refusing to enable")
            generation = self._generation
            for m in self.motors:
                if m.fault_latched:
                    raise ConnectError(f"motor {m.motor_id} has a latched fault ({m.fault_latched}); "
                                       "clear_faults() before enabling")
            # Fresh fault status and position from every motor; cached values never authorize torque.
            self._check_motors()
            pos = self.get_joint_positions()
            grace = self.config.full_turn_grace
            for i, j in enumerate(self.config.joints):
                if not math.isfinite(pos[i]):
                    raise ConnectError(f"{j.label}: no position before enable, refusing to enable")
                if pos[i] < j.lo - grace or pos[i] > j.hi + grace:
                    raise ConnectError(
                        f"{j.label} (motor {j.motor_id}) at {math.degrees(pos[i]):+.1f} deg is outside its limits "
                        f"[{math.degrees(j.lo):.1f}, {math.degrees(j.hi):.1f}] deg even after full-turn correction -- "
                        "stale or missing zero. Re-zero with 'maker-arm zero --all' at the zero pose")
                temp = self.motors[i].feedback.temperature
                limit = j.enable_temperature_c if j.enable_temperature_c is not None else j.max_temperature_c
                if limit is not None and temp >= limit:
                    raise ConnectError(f"{j.label} is {temp:.1f} °C; enable needs it below {limit:.0f} °C")
            self._arm_watchdogs()
            now = time.monotonic()
            with self._target_lock:
                self._user_targets = list(pos)
                self._internal = list(pos)
                self._synced = [False] * self.config.n_joints
                self._targets_received = False
                self._returning = False
                self._stuck_anchor = [(p, now) for p in pos]
            self._hold = {i: GripperHold(j.hold_torque_nm, (j.lo, j.hi), j.closing_direction)
                          for i, j in enumerate(self.config.joints) if j.hold_torque_nm is not None}
            self._hold_out.clear()
            self._release_sent.clear()
            self.rest_pose = list(pos)
            self._released.clear()
            self._faulted.clear()
            self._mode_bad.clear()
            self.fault_reason = None
            self.fault_reasons = []
            try:
                for i, m in enumerate(self.motors):
                    if self._generation != generation:
                        raise StateError("e-stop during enable")
                    # Neutral preload (kp=kd=0 at the present pose) so the enable cannot resume
                    # a stale setpoint left in the motor by an earlier session.
                    if not self._neutralize(i, pos[i]):
                        raise ConnectError(f"motor {m.motor_id} did not acknowledge the neutral preload")
                    with self._enable_gate:
                        if self._generation != generation:
                            raise StateError("e-stop during enable")
                        sequence = m.feedback_sequence
                        m.enable()
                    if not self._wait_reply(m, sequence, self.REPLY_TIMEOUT):
                        raise ConnectError(f"motor {m.motor_id} did not reply to enable")
                    fb = m.feedback
                    if m.fault_latched:
                        raise ConnectError(f"motor {m.motor_id} reported a fault on enable ({m.fault_latched})")
                    if m.params_supported and fb.mode != protocol.MODE_RUN:
                        raise ConnectError(f"motor {m.motor_id} enable rejected: mode={fb.mode}")
                    time.sleep(0.01)
                if self._generation != generation:
                    raise StateError("e-stop during enable")
            except BaseException:
                # Includes cancellation and a motor whose enable may have landed before the error.
                self._release_all()
                raise
            now = time.monotonic()
            self._poll_sent = {m.motor_id: now for m in self.motors}
            self._state = ArmState.ENABLED
            if start_loop:
                self._running = True
                self._loop_thread = threading.Thread(target=self._control_loop, args=(self._session,),
                                                     daemon=True, name="maker-arm-ctrl")
                self._loop_thread.start()

    @staticmethod
    def _wait_reply(m: Motor, sequence: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while m.feedback_sequence == sequence and time.monotonic() < deadline:
            time.sleep(0.0005)
        return m.feedback_sequence != sequence

    def _neutralize(self, i: int, joint_pos: Optional[float] = None) -> bool:
        """Send a zero-gain command at the given (default: measured) pose; True once acknowledged."""
        m = self.motors[i]
        if joint_pos is None:
            joint_pos = self.get_joint_positions()[i]
        if not math.isfinite(joint_pos):
            return False
        self._wait_quiet(m)
        sequence = m.feedback_sequence
        m.send_command(self._to_motor(i, joint_pos), 0.0, 0.0, 0.0, 0.0)
        return self._wait_reply(m, sequence, self.REPLY_TIMEOUT)

    @staticmethod
    def _wait_quiet(m: Motor, quiet: float = 0.005, limit: float = 0.05) -> None:
        """Let late replies to earlier requests arrive, so the next reply answers our request
        (MIT replies carry no transaction ID)."""
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            sequence = m.feedback_sequence
            time.sleep(quiet)
            if m.feedback_sequence == sequence:
                return

    def _arm_watchdogs(self) -> None:
        """Set and verify the motor-side CAN timeout where configured (MakerModsLab gripper)."""
        self._watchdog_restore.clear()
        try:
            for i, j in enumerate(self.config.joints):
                if j.can_timeout_ms is None:
                    continue
                m = self.motors[i]
                if not m.params_supported:
                    log.warning("%s: firmware has no parameter access (needs RS00 >=0.0.3.27 / RS02 >=0.2.3.32); "
                                "running without its %d ms motor watchdog", j.label, j.can_timeout_ms)
                    continue
                ticks = j.can_timeout_ms * protocol.CAN_TIMEOUT_TICKS_PER_MS
                original = m.read_param(protocol.ParamIndex.CAN_TIMEOUT, "u32", self.PARAM_TIMEOUT)
                self._watchdog_restore[i] = original
                readback = m.write_param(protocol.ParamIndex.CAN_TIMEOUT, ticks, "u32", self.PARAM_TIMEOUT)
                if readback != ticks:
                    raise ConnectError(f"{j.label}: CAN timeout read back {readback}, expected {ticks}")
        except ParamTimeout as exc:
            self._restore_watchdogs(set(self._watchdog_restore))
            raise ConnectError(f"motor watchdog setup failed: {exc}") from exc
        except BaseException:
            self._restore_watchdogs(set(self._watchdog_restore))
            raise

    def _restore_watchdogs(self, indices: set[int]) -> None:
        """Restore the original CAN timeout, only on motors confirmed disabled."""
        for i in indices:
            original = self._watchdog_restore.pop(i, None)
            if original is None:
                continue
            try:
                self.motors[i].write_param(protocol.ParamIndex.CAN_TIMEOUT, original, "u32", self.PARAM_TIMEOUT)
            except Exception as e:
                log.error("could not restore %s CAN timeout: %s", self.config.joints[i].label, e)

    def _disable_all(self) -> None:
        """Fast unconfirmed torque-off broadcast (e-stop / fault path)."""
        for m in self.motors:
            try:
                m.disable()
            except Exception as e:
                log.error("failed to disable motor %d: %s", m.motor_id, e)

    def _disable_confirmed(self) -> list[int]:
        """Neutralize then disable each motor, confirming the reply (and mode 0 where the
        firmware reports it). Returns the motor IDs that could not be confirmed."""
        # Torque off everywhere first so an offline motor cannot delay the healthy ones, after
        # a zero-gain command so a later enable cannot resume the old setpoint.
        positions = self.get_joint_positions()
        for i, m in enumerate(self.motors):
            if math.isfinite(positions[i]):
                try:
                    m.send_command(self._to_motor(i, positions[i]), 0.0, 0.0, 0.0, 0.0)
                except Exception as e:
                    log.error("failed to neutralize motor %d: %s", m.motor_id, e)
        self._disable_all()
        unconfirmed = []
        for m in self.motors:
            confirmed = False
            for _ in range(self.REPLY_ATTEMPTS):
                try:
                    self._wait_quiet(m)
                    sequence = m.feedback_sequence
                    m.disable()
                except Exception as e:
                    log.error("failed to disable motor %d: %s", m.motor_id, e)
                    continue
                if self._wait_reply(m, sequence, self.REPLY_TIMEOUT) and (
                        not m.params_supported or m.feedback.mode == protocol.MODE_RESET):
                    confirmed = True
                    break
            if not confirmed:
                unconfirmed.append(m.motor_id)
        return unconfirmed

    def _release_all(self) -> None:
        unconfirmed = self._disable_confirmed()
        self._restore_watchdogs({i for i, m in enumerate(self.motors) if m.motor_id not in unconfirmed})
        self.unconfirmed_disable = unconfirmed
        if unconfirmed:
            log.error("could not confirm torque release on motors %s; their watchdog stays armed. "
                      "Cut motor power if they are still holding.", unconfirmed)

    def _stop_loop(self) -> None:
        self._running = False
        self._session += 1   # a tick still in flight stops sending at its next command
        thread = self._loop_thread
        if thread and threading.current_thread() is not thread:
            thread.join(timeout=1.0)
            if thread.is_alive():
                log.error("control loop did not stop within 1 s; waiting")
                thread.join(timeout=2.0)
            if thread.is_alive():
                log.error("control loop is still alive; enable() stays refused until it exits")
            else:
                self._loop_thread = None

    def disable(self) -> None:
        """Release torque immediately. Prefer stop(), which returns to rest first."""
        with self._lifecycle:
            if self._state is ArmState.DISCONNECTED:
                return
            self._stop_loop()
            self._release_all()
            self._released.clear()
            self._faulted.clear()
            self._returning = False
            if self._state in (ArmState.ENABLED, ArmState.FAULT):
                self._state = ArmState.CONNECTED

    def estop(self) -> None:
        """Callable from any state and thread: torque off now (the arm will drop)."""
        log.warning("e-stop: releasing torque on all motors")
        with self._enable_gate:
            self._generation += 1
            self._running = False
            self._session += 1
        if self._state is not ArmState.DISCONNECTED:
            self._disable_all()
        self.disable()

    def stop(self) -> bool:
        """Return to the rest pose, then release torque only if the return was verified.

        Mirrors MakerModsLab's mandatory stop path: an arm without brakes must never be
        released away from rest. If the return cannot be verified the arm keeps holding its
        current pose and False is returned; the caller decides when to disable().
        """
        with self._lifecycle:
            if self._state not in (ArmState.ENABLED, ArmState.FAULT):
                return True
            try:
                verified = self._return_to_rest()
            except Exception:
                log.exception("return to rest failed")
                verified = False
            if verified:
                self.disable()
                return True
            self._returning = False
            self.hold_current_position()
            return False

    # ── targets ──
    def set_joint_targets(self, targets: list[float]) -> bool:
        if len(targets) != self.config.n_joints or not all(math.isfinite(t) for t in targets):
            log.warning("set_joint_targets rejected: need %d finite targets, got %s", self.config.n_joints, targets)
            return False
        with self._target_lock:
            # Admission is checked under the lock a return to rest takes ownership with, so a
            # target can never land after the return has started.
            if self._state is not ArmState.ENABLED or self._returning:
                log.warning("set_joint_targets rejected: state=%s returning=%s", self._state.name, self._returning)
                return False
            if not self._targets_received:
                # Startup sync is judged against the first real target, not the enable pose.
                now = time.monotonic()
                self._stuck_anchor = [(p, now) for p in self.get_joint_positions()]
                self._targets_received = True
            self._user_targets = list(targets)
        return True

    def get_commanded_positions(self) -> list[float]:
        """Return the rate-limited joint command currently sent by the control loop."""
        with self._target_lock:
            return list(self._internal)

    def hold_current_position(self) -> bool:
        """Freeze the arm at its latest measured pose without releasing torque."""
        if self._state not in (ArmState.ENABLED, ArmState.FAULT):
            return False
        positions = self.get_joint_positions()
        with self._target_lock:
            for i, p in enumerate(positions):
                if math.isfinite(p):
                    self._user_targets[i] = p
                    self._internal[i] = p
        return True

    def return_to_rest(self, rest: Optional[list[float]] = None) -> bool:
        """Drive joints marked return_to_rest back to ``rest`` (default: the pose at enable).

        Linear ramp from the measured pose at rest_speed, setpoints at rest_rate_hz, judged by
        convergence: arrived within rest_arrived, or within rest_settle once improvement has
        stalled (a loaded joint holds a standing error). Bounded by rest_timeout. Never raises.
        New targets are refused while it runs.
        """
        try:
            return self._return_to_rest(rest)
        except Exception:
            log.exception("return to rest failed")
            return False
        finally:
            self._returning = False

    def _return_to_rest(self, rest: Optional[list[float]] = None) -> bool:
        cfg = self.config
        rest = list(rest if rest is not None else (self.rest_pose or []))
        if not self._rest_active() or len(rest) != cfg.n_joints:
            return False
        # Take exclusive ownership: refuse new targets, freeze at the measured pose and let any
        # command already computed by the loop go out before measuring the start.
        positions = self.get_joint_positions()
        with self._target_lock:
            self._returning = True
            for i, p in enumerate(positions):
                if math.isfinite(p):
                    self._user_targets[i] = p
                    self._internal[i] = p
        if not self._wait_ticks(2):
            log.error("return to rest aborted: control loop is not running")
            return False
        include = [i for i, j in enumerate(cfg.joints) if j.return_to_rest and i not in self._released]
        if not include:
            return True
        start = self.get_joint_positions()
        if not self._rest_feedback_ok(include, start):
            return False
        with self._target_lock:
            self._synced = [True] * cfg.n_joints
        worst = max(abs(start[i] - rest[i]) for i in include)
        if worst <= cfg.rest_arrived:
            return True
        period = 1.0 / cfg.rest_rate_hz
        steps = max(1, math.ceil(worst / cfg.rest_speed * cfg.rest_rate_hz))
        deadline = time.monotonic() + cfg.rest_timeout
        for k in range(1, steps + 1):
            if time.monotonic() >= deadline or not self._rest_active():
                break
            frac = k / steps
            with self._target_lock:
                for i in include:
                    self._user_targets[i] = start[i] + (rest[i] - start[i]) * frac
            time.sleep(period)
            if not self._rest_feedback_ok(include, self.get_joint_positions()):
                return False
        best, stalled = math.inf, 0
        while time.monotonic() < deadline and self._rest_active():
            pos = self.get_joint_positions()
            if not self._rest_feedback_ok(include, pos):
                return False
            worst = max(abs(pos[i] - rest[i]) for i in include)
            if worst <= cfg.rest_arrived:
                return True
            if best - worst < cfg.rest_stall_improvement:
                stalled += 1
                if stalled >= cfg.rest_stall_polls:
                    return worst <= cfg.rest_settle
            else:
                stalled = 0
            best = min(best, worst)
            time.sleep(period)
        return False

    def _wait_ticks(self, n: int, timeout: float = 1.0) -> bool:
        target = self._tick_count + n
        deadline = time.monotonic() + timeout
        while self._tick_count < target and time.monotonic() < deadline and self._running:
            time.sleep(0.001)
        return self._tick_count >= target

    def _rest_active(self) -> bool:
        return self._running and self._state in (ArmState.ENABLED, ArmState.FAULT)

    def _rest_feedback_ok(self, include: list[int], positions: list[float]) -> bool:
        for i in include:
            label = self.config.joints[i].label
            if i in self._faulted:
                log.error("return to rest aborted: %s is faulted", label)
                return False
            if not math.isfinite(positions[i]) or self.motors[i].feedback_age > self.config.feedback_timeout:
                log.error("return to rest aborted: %s feedback is stale", label)
                return False
        return True

    # ── control loop ──
    @staticmethod
    def _busy_wait_us(us: float) -> None:
        end = time.perf_counter() + us / 1e6
        while time.perf_counter() < end:
            pass

    def _skipped(self, i: int) -> bool:
        return i in self._released or i in self._faulted

    def _may_send(self, session: Optional[int]) -> bool:
        return session is None or (self._running and session == self._session)

    def _tick(self, dt: float, session: Optional[int] = None) -> None:
        cfg = self.config
        now = time.monotonic()
        present = self.get_joint_positions()
        with self._target_lock:
            maxstep = cfg.max_velocity * dt
            for i, j in enumerate(cfg.joints):
                if self._skipped(i):
                    continue   # released: off; faulted: its target stays frozen at the fault pose
                goal = min(j.hi - cfg.limit_margin, max(j.lo + cfg.limit_margin, self._user_targets[i]))
                if not self._synced[i] and self._targets_received and math.isfinite(present[i]):
                    goal = self._startup_sync(i, goal, present[i], now)
                step = min(maxstep, max(-maxstep, goal - self._internal[i]))
                self._internal[i] += step
            commands = list(self._internal)
        paced = getattr(self._backend, "feedback_paced_control", False)
        for i, j in enumerate(cfg.joints):
            if not self._may_send(session):
                return
            motor = self.motors[i]
            if i in self._released:
                # Keep re-sending torque-off: a single lost disable must not leave it energized.
                if now - self._release_sent.get(i, 0.0) >= 0.1:
                    self._release_sent[i] = now
                    try:
                        motor.disable()
                    except Exception as e:
                        log.error("failed to disable motor %d: %s", motor.motor_id, e)
                continue
            # A faulted (not released) joint keeps receiving its frozen hold command.
            position, kp, kd = commands[i], j.kp, j.kd
            sequence = motor.feedback_sequence
            try:
                hold = self._hold.get(i)
                if hold is not None and i not in self._faulted:
                    position, kp, kd = self._gripper_command(i, hold, commands[i], now)
                motor.send_command(self._to_motor(i, position), 0.0, kp, kd, 0.0)
            except Exception as e:
                if i not in self._faulted:
                    self._enter_fault(f"{j.label}: command failed ({e})", i)
                continue
            if paced:
                # Old USB-SLCAN firmware has no outbound USB queue and a three-frame CAN RX
                # FIFO: wait for this reply before commanding the next motor. A timeout does not
                # skip the remaining motors; the feedback watchdog makes the safety decision.
                self._wait_reply(motor, sequence, getattr(self._backend, "feedback_wait_timeout", 0.015))
            else:
                self._busy_wait_us(150)
        # Non-clearing fault poll: at most one motor per tick (the longest-waiting one) so the
        # replies never burst into a small adapter FIFO.
        due = [(self._poll_sent.get(m.motor_id, 0.0), i) for i, m in enumerate(self.motors)
               if not self._skipped(i)]
        if due and self._may_send(session):
            sent_at, i = min(due)
            if now - sent_at >= cfg.fault_poll_interval:
                self._poll_sent[self.motors[i].motor_id] = now
                try:
                    self.motors[i].query_fault_async()
                except Exception as e:
                    log.error("fault query to motor %d failed: %s", self.motors[i].motor_id, e)
        self._tick_count += 1
        self._check_health()

    def _gripper_command(self, i: int, hold: GripperHold, target: float, now: float):
        """Holding regulation on one fresh, self-consistent feedback sample (MakerModsLab reads a
        matching sample before each update); without a new sample the last output is reused."""
        j = self.config.joints[i]
        m = self.motors[i]
        fb, sequence = m.feedback, m.feedback_sequence
        last = self._hold_out.get(i)
        if last is not None and last[0] == sequence:
            return last[1], last[2], last[3]
        position = hold.update(target, self._to_joint(i, fb.position), fb.velocity * j.direction,
                               fb.torque * j.direction, j.kp, now)
        kp, kd = hold.gains(j.kp, j.kd)
        self._hold_out[i] = (sequence, position, kp, kd)
        return position, kp, kd

    def _startup_sync(self, i: int, goal: float, present: float, now: float) -> float:
        """Cap the goal near the measured pose until the joint has caught up with its target."""
        cfg = self.config
        if abs(goal - present) <= cfg.startup_synced:
            self._synced[i] = True
            return goal
        anchor, since = self._stuck_anchor[i]
        if abs(present - anchor) > cfg.startup_stuck:
            self._stuck_anchor[i] = (present, now)
        elif now - since >= cfg.startup_stuck_seconds:
            self._synced[i] = True
            return goal
        return min(present + cfg.startup_step, max(present - cfg.startup_step, goal))

    def _control_loop(self, session: int) -> None:
        dt = 1.0 / self.config.control_rate_hz
        next_t = time.perf_counter()
        while self._may_send(session):
            try:
                self._tick(dt, session)
            except Exception as e:
                # Never abandon energized motors: the hold only exists while commands go out,
                # so keep servicing (and reporting) until disable()/estop().
                if self._state is ArmState.ENABLED or len(self.fault_reasons) < 100:
                    self._enter_fault(f"control loop exception: {e}")
            next_t += dt
            remain = next_t - time.perf_counter()
            if remain > 0:
                time.sleep(remain)
            else:
                next_t = time.perf_counter()   # if we've fallen behind, reset rather than trying to catch up

    def _check_health(self) -> None:
        """Evaluated every tick, in FAULT too: a second motor can still fail while holding."""
        if self._state not in (ArmState.ENABLED, ArmState.FAULT):
            return
        cfg = self.config
        status_deadline = 2 * cfg.fault_poll_interval + cfg.feedback_timeout
        for i, (m, j) in enumerate(zip(self.motors, cfg.joints)):
            if self._skipped(i):
                continue
            reason = None
            fb = m.feedback
            if m.feedback_age > cfg.feedback_timeout:
                reason = f"feedback timeout {m.feedback_age:.2f}s -- check bus/power"
            elif m.fault_latched:
                reason = f"motor fault -- {m.fault_latched}"
            elif m.fault_age > status_deadline:
                reason = f"no fault-status reply for {m.fault_age:.2f}s"
            elif j.max_temperature_c is not None and fb.temperature >= j.max_temperature_c:
                reason = f"overtemperature {fb.temperature:.1f} °C (limit {j.max_temperature_c:.0f} °C)"
            elif j.warning_is_fault and fb.warning_flag:
                reason = "motor warning flag set"
            else:
                # Mode bits only exist on firmware with parameter access; legacy firmware reports
                # 0 there. Calibration (1) and undefined (3) modes are never legitimate.
                bad_mode = fb.mode in (protocol.MODE_CALIBRATION, 3) or (
                    m.params_supported and fb.mode != protocol.MODE_RUN)
                count = self._mode_bad.get(m.motor_id, 0) + 1 if bad_mode else 0
                self._mode_bad[m.motor_id] = count
                if count >= self.MODE_FAULT_TICKS:
                    reason = f"left run mode (mode={fb.mode} for {count} ticks)"
            if reason:
                self._enter_fault(f"{j.label}: {reason}", i)

    def _enter_fault(self, reason: str, index: Optional[int] = None) -> None:
        log.error("FAULT: %s", reason)
        self.fault_reasons.append(reason)
        if self.fault_reason is None:
            self.fault_reason = reason
        if index is not None:
            # A faulted joint holds its measured pose and is excluded from the return to rest
            # (which then cannot be verified); the gripper is released rather than held, as
            # MakerModsLab's gripper protection does.
            position = self.get_joint_positions()[index]
            with self._target_lock:
                if math.isfinite(position):
                    self._user_targets[index] = self._internal[index] = position
            self._faulted.add(index)
            if self.config.joints[index].release_on_fault:
                self._released.add(index)
                try:
                    self.motors[index].disable()
                except Exception as e:
                    log.error("failed to disable motor %d: %s", self.motors[index].motor_id, e)
        if not self.config.hold_on_fault:
            self._running = False
            self._disable_all()
            self._state = ArmState.FAULT
            return
        if self._state is ArmState.ENABLED:
            # Hold the measured pose (prevents a drop); the loop keeps sending hold frames.
            # Released by stop()/disable()/estop()/clear_faults().
            self._state = ArmState.FAULT
            self.hold_current_position()
            log.error("holding the current pose; stop() returns to rest, disable() releases torque")

    def clear_faults(self) -> None:
        """Explicit operator action: release torque, clear motor faults and latched evidence."""
        with self._lifecycle:
            if self._state is ArmState.DISCONNECTED:
                raise StateError("clear_faults() requires a connected arm")
            if self._state in (ArmState.ENABLED, ArmState.FAULT):
                self.disable()
            for m in self.motors:
                m.clear_fault()
                time.sleep(0.01)
            self.fault_reason = None
            self.fault_reasons = []
            self._state = ArmState.CONNECTED
