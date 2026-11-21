"""Arm layer: state machine, control loop and all safety logic, in joint coordinates (SI units).

Non-clearing fault checks, startup sync from the measured pose, full-turn correction, latched
fault evidence, gripper holding-effort regulation and temperature protection, and a verified
return to rest before torque is released. Everything motor-specific goes through motor.Motor.
"""

import logging
import math
import threading
import time
from dataclasses import replace
from enum import Enum
from typing import Callable, Optional, Sequence

import numpy as np

from . import gravity, kinematics, trajectory_ik
from ._product import CLI_NAME
from .config import ArmConfig, JointConfig, check_gains
from .errors import ArmError, ConnectError, FaultLatched, ParamTimeout, StateError
from .gripper import GripperHold
from .motor import ID_RANGE, Motor, route
from .transport.base import CanBackend, create_backend

log = logging.getLogger(__package__)
# move_ee: how far the planned path may end from the goal (m, rad).
EE_POSITION_TOL = 1e-3
EE_ANGLE_TOL = math.radians(0.5)
EE_MIN_SECONDS = 0.25   # shortest path


class ArmState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ENABLED = "enabled"
    FAULT = "fault"


def _finite(x: float) -> Optional[float]:
    return x if math.isfinite(x) else None


# Extra feed-forward added while floating: (joint positions, joint velocities) -> N*m per joint.
ExtraTorque = Callable[[list[float], list[float]], Sequence[float]]


class Arm:
    REPLY_TIMEOUT = 0.05   # s, per request
    REPLY_ATTEMPTS = 3
    PARAM_TIMEOUT = 0.12   # s, typed parameter replies
    FLOAT_KD = 0.1         # damping used by float_mode()

    def __init__(self, config: ArmConfig, backend: CanBackend):
        config._validate()
        self.config = config
        self._backend = backend
        self.motors = [Motor(j, backend, config.host_id) for j in config.joints]
        self._by_id = {m.motor_id: m for m in self.motors}
        self._profile_gains = [(j.kp, j.kd) for j in config.joints]
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
        self._speed: Optional[float] = None   # move_to() speed cap, cleared by the next target
        self._stuck_anchor: list[tuple[float, float]] = [(0.0, 0.0)] * n
        self._returning = False               # a return to rest owns the targets
        self._released: set[int] = set()      # disabled after a fault (release_on_fault)
        self._faulted: set[int] = set()       # frozen at the fault pose
        self._watchdog_restore: dict[int, object] = {}
        self._hold: dict[int, GripperHold] = {}
        self._hold_out: dict[int, tuple[int, float, float, float]] = {}   # seq, position, kp, kd
        self._release_sent: dict[int, float] = {}
        self._poll_sent: dict[int, float] = {}
        self._polling_since = 0.0             # when the control loop took over status polling
        self._gravity = None
        self._extra_torque: Optional[ExtraTorque] = None   # float_mode(extra_torque=...)
        self._floating = False                # float_mode() until set_gains() or a fault
        self._chain: Optional[tuple[tuple, kinematics.Chain]] = None   # (profile key, chain)
        self._tick_count = 0
        self._running = False
        self._loop_thread: Optional[threading.Thread] = None
        self.fault_reason: Optional[str] = None
        self.fault_reasons: list[str] = []
        self.unconfirmed_disable: list[int] = []
        self._wrap: list[float] = [0.0] * n   # per-session full-turn correction (joint coordinates)
        self.rest_pose: Optional[list[float]] = None

    @classmethod
    def from_yaml(cls, path: str, backend: str = "socketcan", **backend_kwargs) -> "Arm":
        return cls(ArmConfig.from_yaml(path), create_backend(backend, **backend_kwargs))

    @property
    def state(self) -> ArmState:
        return self._state

    @property
    def joint_names(self) -> list[str]:
        return [j.label for j in self.config.joints]

    @property
    def backend(self) -> CanBackend:
        return self._backend

    # ── RX dispatch (transport-layer callback: fast in, fast out) ──
    def _on_frame(self, can_id: int, data: bytes) -> None:
        m = self._by_id.get(route(can_id, data, self.config.host_id))
        if m is not None:
            m.handle_frame(can_id, data)

    # ── connection management ──
    def connect(self, clear_faults: bool = False, clear_released: bool = False) -> None:
        """Find every motor without changing torque state.

        Each motor first gets a non-clearing fault query. A faulted motor is reported and the
        fault is kept (pass ``clear_faults=True`` to clear it explicitly). ``clear_released=True``
        clears only the state a release_on_fault joint's motor watchdog leaves behind (the gripper
        let go after commands stopped; see Motor.watchdog_released). Fault-free motors then return
        a fresh position. Torque is never enabled or released here.
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
                self._check_motors(clear_faults, clear_released)
                for m in self.motors:
                    m.validate(self.PARAM_TIMEOUT)
            except BaseException:
                self._backend.close()
                raise
            # Full-turn correction: a power cycle can reset the motor's multi-turn count, shifting
            # a reading by ±2π. Joint travel is under one turn, so a reading outside the limits
            # that lands inside them after a whole-turn shift is unambiguous. Anything else is a
            # stale zero and enable() refuses it.
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

    def _check_motors(self, clear_faults: bool = False, clear_released: bool = False) -> None:
        """Non-clearing fault query, then a fresh state read, for every motor."""
        missing, faulted = [], []
        for m, j in zip(self.motors, self.config.joints):
            try:
                status = m.check(self.REPLY_TIMEOUT, self.REPLY_ATTEMPTS)
            except ParamTimeout:
                missing.append(m.motor_id)
                continue
            if status.has_fault and clear_released and j.release_on_fault and m.watchdog_released:
                log.info("%s: clearing the state its motor watchdog left (%s)", j.label, status.text)
                m.clear_fault()
                time.sleep(0.01)
                status = m.check(self.REPLY_TIMEOUT, self.REPLY_ATTEMPTS)
            if status.has_fault and clear_faults:
                log.warning("%s: clearing fault 0x%08X (%s) on request", j.label, status.code, status.text)
                m.clear_fault()
                time.sleep(0.01)
                status = m.check(self.REPLY_TIMEOUT, self.REPLY_ATTEMPTS)
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
                             f"clear it explicitly ({CLI_NAME} scan --clear-faults) once the cause is fixed")
            raise ConnectError("; ".join(parts))

    def disconnect(self) -> None:
        """Release torque immediately and close the bus. Call stop() first for a safe return."""
        with self._lifecycle:
            if self._state in (ArmState.ENABLED, ArmState.FAULT):
                self.disable()
            self._backend.close()
            self._state = ArmState.DISCONNECTED

    def detach(self) -> dict[str, float]:
        """Stop the control loop and close the bus, leaving torque on: each motor holds its last
        command, except that a joint with an armed motor watchdog releases once it times out.
        Returns those joints (see watchdog_joints())."""
        with self._lifecycle:
            releasing = self.watchdog_joints()
            self._stop_loop()
            self._backend.close()
            self._state = ArmState.DISCONNECTED
            return releasing

    def watchdog_joints(self) -> dict[str, float]:
        """Joints whose motor watchdog this session armed: label -> seconds after the last command
        at which that motor releases torque by itself."""
        joints = self.config.joints
        return {joints[i].label: joints[i].can_timeout_ms / 1000 for i in sorted(self._watchdog_restore)}

    def detach_unsafe(self) -> list[str]:
        """Joints that would drop after detach(): an armed watchdog on a joint that is not release_on_fault."""
        by_label = {j.label: j for j in self.config.joints}
        return [n for n in self.watchdog_joints() if not by_label[n].release_on_fault]

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
                if m.check(timeout, attempts=1).has_fault:
                    raise FaultLatched(f"motor {m.motor_id}: {m.fault_latched}")
                m.read_state(timeout, attempts=1)
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
        """Latest fault code per motor (0 = no fault, None = never queried)."""
        return [(m.status.code if m.status.has_fault else 0) if m.status else None for m in self.motors]

    def status(self) -> dict:
        """JSON-serialisable snapshot: arm state plus, per joint, the latest feedback (None where
        there is none), its age and any motor fault."""
        pos, vel, tau = self.get_joint_positions(), self.get_joint_velocities(), self.get_joint_torques()
        joints = {}
        for i, (m, j) in enumerate(zip(self.motors, self.config.joints)):
            fb, st = m.feedback, m.status
            joints[j.label] = {
                "motor_id": m.motor_id,
                "position": _finite(pos[i]),
                "velocity": _finite(vel[i]),
                "torque": _finite(tau[i]),
                "temperature": _finite(fb.temperature) if fb else None,
                "temperatures": dict(fb.temperatures) if fb else {},
                "enabled": fb.enabled if fb else None,
                "fault": m.fault_latched or (st.text if st is not None and st.has_fault else None),
                "feedback_age": _finite(m.feedback_age),
            }
        return {"state": self._state.value, "fault_reason": self.fault_reason, "joints": joints}

    # ── gripper (the joint named "gripper") ──
    def _gripper_range(self) -> tuple[int, float, float]:
        """(index, closed position, fully open position) of the gripper joint, rad."""
        for i, j in enumerate(self.config.joints):
            if j.name == "gripper":
                closed, far = (j.hi, j.lo) if j.closing_direction > 0 else (j.lo, j.hi)
                return i, closed, far if j.open_position is None else j.open_position
        raise ArmError("this profile has no joint named 'gripper'")

    def gripper_opening(self) -> float:
        """Measured gripper opening: 0 closed, 1 fully open (not clipped; nan without feedback)."""
        i, closed, full = self._gripper_range()
        return (self.get_joint_positions()[i] - closed) / (full - closed)

    def gripper_target(self, opening: float) -> float:
        """Gripper joint position (rad) for ``opening`` (0 closed, 1 fully open; clipped to [0, 1])."""
        if not math.isfinite(opening):
            raise ValueError(f"opening must be finite, got {opening}")
        _, closed, full = self._gripper_range()
        return closed + min(1.0, max(0.0, opening)) * (full - closed)

    # ── enabling ──
    def enable(self, start_loop: bool = True, cancel: Optional[threading.Event] = None,
               speed: Optional[float] = None) -> None:
        """Torque on, holding the measured pose. ``cancel`` (like estop()) aborts an enable in
        progress: torque already applied is released and StateError is raised. ``speed`` (rad/s)
        caps the motion until the first target, such as a joint resting past a limit being brought
        inside it (default: max_velocity)."""
        if speed is not None and not (math.isfinite(speed) and speed > 0):
            raise ValueError("speed must be positive")
        with self._lifecycle:
            if self._state is not ArmState.CONNECTED:
                raise StateError(f"enable() requires CONNECTED state, currently {self._state.name}")
            if self._loop_thread is not None and self._loop_thread.is_alive():
                raise StateError("the previous control loop is still running; refusing to enable")
            generation = self._generation

            def abort_if_cancelled() -> None:
                if self._generation != generation:
                    raise StateError("e-stop during enable")
                if cancel is not None and cancel.is_set():
                    raise StateError("enable cancelled")

            abort_if_cancelled()
            for m in self.motors:
                if m.fault_latched:
                    raise ConnectError(f"motor {m.motor_id} has a latched fault ({m.fault_latched}); "
                                       "clear_faults() before enabling")
            # Fresh fault status and position from every motor; cached values never authorize torque.
            self._check_motors()
            # A motor already holding (another session, `--then stay`) would lose its torque to the
            # neutral preload, and a failed enable releases every motor: refuse before any frame.
            holding = [f"{m.motor_id} ({j.label})" for m, j in zip(self.motors, self.config.joints)
                       if m.feedback is not None and m.feedback.enabled]
            if holding:
                raise ConnectError(f"motors {', '.join(holding)} already have torque on; nothing was sent to "
                                   f"them. Take the arm over with attach() ('{CLI_NAME} rest' brings it down "
                                   "and releases it)")
            pos = self.get_joint_positions()
            self._check_start(pos, range(self.config.n_joints))
            self._gravity = self._make_gravity()
            self._arm_watchdogs()
            self._start_targets(pos, speed)
            self.rest_pose = list(pos)
            try:
                for i, m in enumerate(self.motors):
                    abort_if_cancelled()
                    # Neutral preload (kp=kd=0 at the present pose) so the enable cannot resume
                    # a stale setpoint left in the motor by an earlier session.
                    if not m.preload(self._to_motor(i, pos[i]), self.REPLY_TIMEOUT):
                        raise ConnectError(f"motor {m.motor_id} did not acknowledge the neutral preload")
                    with self._enable_gate:
                        abort_if_cancelled()
                        confirmed = m.enable_confirmed(self.REPLY_TIMEOUT)
                    if m.fault_latched:
                        raise ConnectError(f"motor {m.motor_id} reported a fault on enable ({m.fault_latched})")
                    if not confirmed:
                        raise ConnectError(f"motor {m.motor_id} enable rejected: no reply, or its feedback "
                                           "does not report torque on")
                    time.sleep(0.01)
                abort_if_cancelled()
            except BaseException:
                # Includes cancellation and a motor whose enable may have landed before the error.
                self._release_all()
                raise
            self._go_enabled(start_loop)

    def attach(self, cancel: Optional[threading.Event] = None, speed: Optional[float] = None) -> None:
        """Take over an arm left holding with torque on and no owning process (``--then stay``, a
        crashed program), from CONNECTED, without letting it sag: nothing is sent to a motor that
        reports torque on except holding frames at its measured pose (profile gains, gravity
        feed-forward) from the control loop. A motor that reports torque off (a gripper its watchdog
        released; connect with ``clear_released=True``) is enabled at its measured pose as enable()
        does. A motor whose firmware does not report torque state first gets one holding frame, then
        an enable, so whether it was holding or not it ends holding its measured pose.

        Refused (ConnectError, nothing sent) when every motor reports torque off (use enable()), on
        missing or stale feedback, a latched fault, or a joint outside its limits. On a later
        failure only the motors this call enabled are released. Then ENABLED, with every protection
        enable() gives. ``rest_pose`` stays None, so stop() holds rather than releasing until a rest
        target is given (return_to_rest(rest), or set ``rest_pose``). ``cancel`` and ``speed`` as
        in enable().
        """
        if speed is not None and not (math.isfinite(speed) and speed > 0):
            raise ValueError("speed must be positive")
        with self._lifecycle:
            if self._state is not ArmState.CONNECTED:
                raise StateError(f"attach() requires CONNECTED state, currently {self._state.name}")
            if self._loop_thread is not None and self._loop_thread.is_alive():
                raise StateError("the previous control loop is still running; refusing to attach")
            generation = self._generation

            def abort_if_cancelled() -> None:
                if self._generation != generation:
                    raise StateError("e-stop during attach")
                if cancel is not None and cancel.is_set():
                    raise StateError("attach cancelled")

            abort_if_cancelled()
            for m in self.motors:
                if m.fault_latched:
                    raise ConnectError(f"motor {m.motor_id} has a latched fault ({m.fault_latched}); "
                                       "refusing to attach")
            self._check_motors()
            torque = [m.feedback.enabled for m in self.motors]
            if not any(t is not False for t in torque):
                raise ConnectError("no motor reports torque on: nothing to attach to; use enable()")
            for m, j in zip(self.motors, self.config.joints):
                if m.feedback_age > self.config.feedback_timeout:
                    raise ConnectError(f"{j.label}: feedback is {m.feedback_age:.2f} s old; refusing to attach")
            pos = self.get_joint_positions()
            enabling = [i for i, t in enumerate(torque) if t is not True]
            self._check_start(pos, enabling, "attach")
            self._gravity = self._make_gravity()
            self._arm_watchdogs()
            self._start_targets(pos, speed)
            self.rest_pose = None
            enabled: list[int] = []
            try:
                feedforward = self._feedforward(list(pos), time.monotonic())
                for i in enabling:
                    m, j = self.motors[i], self.config.joints[i]
                    abort_if_cancelled()
                    if torque[i] is None:
                        # Holds now if the motor is energized; the enable below resumes this frame.
                        m.send_command(self._to_motor(i, pos[i]), 0.0, j.kp, j.kd, feedforward[i])
                        time.sleep(0.002)
                    elif not m.preload(self._to_motor(i, pos[i]), self.REPLY_TIMEOUT):
                        raise ConnectError(f"motor {m.motor_id} did not acknowledge the neutral preload")
                    with self._enable_gate:
                        abort_if_cancelled()
                        enabled.append(i)
                        confirmed = m.enable_confirmed(self.REPLY_TIMEOUT)
                    if m.fault_latched:
                        raise ConnectError(f"motor {m.motor_id} reported a fault on enable ({m.fault_latched})")
                    if not confirmed:
                        raise ConnectError(f"motor {m.motor_id} enable rejected: no reply, or its feedback "
                                           "does not report torque on")
                    time.sleep(0.01)
                abort_if_cancelled()
            except BaseException:
                # The motors that were holding keep their torque: releasing them would drop the arm.
                self._release_enabled([i for i in enabled if torque[i] is False])
                self._restore_watchdogs(set(self._watchdog_restore))
                raise
            self._go_enabled(True)

    def _check_start(self, pos: list[float], check_temperature, action: str = "enable") -> None:
        """Refuse to start without a position, outside the limits, or (joints in
        ``check_temperature``) at or above the enable temperature."""
        grace = self.config.full_turn_grace
        hot = set(check_temperature)
        for i, j in enumerate(self.config.joints):
            if not math.isfinite(pos[i]):
                raise ConnectError(f"{j.label}: no position before {action}, refusing to {action}")
            if pos[i] < j.lo - grace or pos[i] > j.hi + grace:
                raise ConnectError(
                    f"{j.label} (motor {j.motor_id}) at {math.degrees(pos[i]):+.1f} deg is outside its limits "
                    f"[{math.degrees(j.lo):.1f}, {math.degrees(j.hi):.1f}] deg even after full-turn correction -- "
                    f"stale or missing zero. Re-zero with '{CLI_NAME} zero --all' at the zero pose")
            temp = self.motors[i].feedback.temperature
            limit = j.enable_temperature_c if j.enable_temperature_c is not None else j.max_temperature_c
            if i in hot and limit is not None and temp >= limit:
                raise ConnectError(f"{j.label} is {temp:.1f} °C; {action} needs it below {limit:.0f} °C")

    def _start_targets(self, pos: list[float], speed: Optional[float]) -> None:
        """Targets, gripper regulation and fault evidence for a session starting at ``pos``."""
        now = time.monotonic()
        with self._target_lock:
            self._user_targets = list(pos)
            self._internal = list(pos)
            self._synced = [False] * self.config.n_joints
            self._targets_received = False
            self._returning = False
            self._speed = speed
            self._stuck_anchor = [(p, now) for p in pos]
        self._hold = {i: GripperHold(j.hold_torque_nm, (j.lo, j.hi), j.closing_direction)
                      for i, j in enumerate(self.config.joints) if j.hold_torque_nm is not None}
        self._hold_out.clear()
        self._release_sent.clear()
        self._released.clear()
        self._faulted.clear()
        self.fault_reason = None
        self.fault_reasons = []

    def _go_enabled(self, start_loop: bool) -> None:
        now = time.monotonic()
        # Each motor's next status poll is due from its last status reply, so a slow enable
        # cannot leave a reply older than the status deadline.
        self._poll_sent = {m.motor_id: now - min(m.status_age, now) for m in self.motors}
        self._polling_since = now
        self._state = ArmState.ENABLED
        if start_loop:
            self._running = True
            self._loop_thread = threading.Thread(target=self._control_loop, args=(self._session,),
                                                 daemon=True, name=f"{CLI_NAME}-ctrl")
            self._loop_thread.start()

    def _release_enabled(self, indices: list[int]) -> None:
        """Verified torque-off on the motors at ``indices`` only."""
        positions = self.get_joint_positions()
        unconfirmed = []
        for i in indices:
            m = self.motors[i]
            try:
                if math.isfinite(positions[i]):
                    m.send_command(self._to_motor(i, positions[i]), 0.0, 0.0, 0.0, 0.0)
                if not m.disable_confirmed(self.REPLY_TIMEOUT, self.REPLY_ATTEMPTS):
                    unconfirmed.append(m.motor_id)
            except Exception as e:
                log.error("failed to disable motor %d: %s", m.motor_id, e)
                unconfirmed.append(m.motor_id)
        self.unconfirmed_disable = unconfirmed
        if unconfirmed:
            log.error("could not confirm torque release on motors %s; cut motor power if they hold", unconfirmed)

    @staticmethod
    def _wait_reply(m: Motor, sequence: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while m.feedback_sequence == sequence and time.monotonic() < deadline:
            time.sleep(0.0005)
        return m.feedback_sequence != sequence

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
        """Set and verify the motor-side CAN timeout where configured."""
        self._watchdog_restore.clear()
        try:
            for i, j in enumerate(self.config.joints):
                if j.can_timeout_ms is None:
                    continue
                token = self.motors[i].arm_watchdog(j.can_timeout_ms, self.PARAM_TIMEOUT)
                if token is not None:
                    self._watchdog_restore[i] = token
        except ParamTimeout as exc:
            self._restore_watchdogs(set(self._watchdog_restore))
            raise ConnectError(f"motor watchdog setup failed: {exc}") from exc
        except BaseException:
            self._restore_watchdogs(set(self._watchdog_restore))
            raise

    def _restore_watchdogs(self, indices: set[int]) -> None:
        """Restore the original CAN timeout, only on motors confirmed disabled."""
        for i in indices:
            token = self._watchdog_restore.pop(i, None)
            if token is None:
                continue
            try:
                self.motors[i].restore_watchdog(token, self.PARAM_TIMEOUT)
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
        """Neutralize then disable each motor, confirming the reply (and the reported mode where
        the firmware reports it). Returns the motor IDs that could not be confirmed."""
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
                if self._wait_reply(m, sequence, self.REPLY_TIMEOUT) and m.confirms("disabled"):
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

        An arm without brakes must never be released away from rest. If the return cannot be verified the arm keeps holding its
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

    # ── operator actions while torque is off ──
    def set_zero(self, indices: Optional[Sequence[int]] = None) -> None:
        """Make each selected joint's (default: every joint's) present position its motor's zero.
        Persists in the motors. Torque must be off (CONNECTED); refresh() afterwards."""
        with self._lifecycle:
            if self._state is not ArmState.CONNECTED:
                raise StateError(f"set_zero() requires CONNECTED state, currently {self._state.name}")
            selected = list(range(self.config.n_joints)) if indices is None else list(indices)
            for i in selected:
                if not 0 <= i < self.config.n_joints:
                    raise IndexError(f"joint index {i} out of range")
            for i in selected:
                self.motors[i].set_zero()
                self._wrap[i] = 0.0
                time.sleep(0.01)

    def set_motor_id(self, old: int, new: int, timeout: float = 0.5) -> None:
        """Persistently change a motor's CAN ID from ``old`` to ``new`` while torque is off.

        Works connected or disconnected (the bus is opened for the call). Refuses if ``new``
        already answers. A configured joint that had ``old`` is routed to ``new`` from then on
        (refresh() reads it again); update the profile file to keep the change.
        """
        lo, hi = ID_RANGE
        for value in (old, new):
            if not lo <= value <= hi:
                raise ValueError(f"motor IDs must be in [{lo}, {hi}], got {value}")
        if old == new:
            raise ValueError("old and new motor IDs are the same")
        with self._lifecycle:
            if self._state not in (ArmState.DISCONNECTED, ArmState.CONNECTED):
                raise StateError(f"set_motor_id() requires torque off, currently {self._state.name}")
            joint = next((j for j in self.config.joints if j.motor_id == old), None) or JointConfig(
                motor_id=old, lo=-math.pi, hi=math.pi, kp=0.0, kd=0.0)
            target = Motor(joint, self._backend, self.config.host_id)
            probe = Motor(replace(joint, motor_id=new), self._backend, self.config.host_id)
            temporary = {old: target, new: probe}
            host_id = self.config.host_id

            def receive(can_id, data):
                m = temporary.get(route(can_id, data, host_id))
                if m is not None:
                    m.handle_frame(can_id, data)

            opened = self._state is ArmState.DISCONNECTED
            self._backend.set_recv_callback(receive)
            try:
                if opened:
                    self._backend.open()
                try:
                    probe.check(self.REPLY_TIMEOUT, self.REPLY_ATTEMPTS)
                    taken = True
                except ParamTimeout:
                    taken = False
                if taken:
                    raise ArmError(f"motor ID {new} already answers on the bus")
                target.set_id(new, timeout)
                self._rekey(old, new, target)
            finally:
                if opened:
                    self._backend.close()
                self._backend.set_recv_callback(self._on_frame)

    def _rekey(self, old: int, new: int, changed: Motor) -> None:
        """After an id change, route the joint that had ``old`` to its motor at ``new``."""
        index = next((i for i, j in enumerate(self.config.joints) if j.motor_id == old), None)
        if index is None:
            return
        joint = self.config.joints[index]
        reply_id = getattr(changed, "reply_id", None) if joint.reply_id is not None else None
        joint = replace(joint, motor_id=new, reply_id=reply_id)
        self.config.joints[index] = joint
        if self.config.required_motor_ids is not None:
            self.config.required_motor_ids = [new if i == old else i for i in self.config.required_motor_ids]
        self.motors[index] = Motor(joint, self._backend, self.config.host_id)
        self._by_id = {m.motor_id: m for m in self.motors}
        self._poll_sent.pop(old, None)

    # ── gains ──
    def set_gains(self, kp: Optional[Sequence[float]] = None, kd: Optional[Sequence[float]] = None) -> None:
        """Per-joint gains for the frames that follow (one value per joint; None keeps the
        present ones). With neither given, the profile's gains come back. A joint with kp 0
        floats: its command follows the measured pose, so raising kp again holds it there."""
        n = self.config.n_joints
        if kp is None and kd is None:
            gains = list(self._profile_gains)
        else:
            kps = [j.kp for j in self.config.joints] if kp is None else [float(v) for v in kp]
            kds = [j.kd for j in self.config.joints] if kd is None else [float(v) for v in kd]
            if len(kps) != n or len(kds) != n:
                raise ValueError(f"need {n} kp and kd values")
            gains = list(zip(kps, kds))
        for j, (p, d) in zip(self.config.joints, gains):
            check_gains(j.motor_id, p, d)
        with self._target_lock:
            self.config.joints = [replace(j, kp=p, kd=d) for j, (p, d) in zip(self.config.joints, gains)]
            self._extra_torque = None
            self._floating = False

    def float_mode(self, kd: Optional[float] = None, extra_torque: Optional[ExtraTorque] = None) -> None:
        """Back-drivable arm (teleop leader): kp 0 and light damping on every joint, gravity
        feed-forward only. ``extra_torque(positions, velocities)`` (joint coordinates, one N*m value
        per joint) is added to the feed-forward every tick, e.g. friction compensation; if it raises
        or returns a bad value it is dropped (logged) and the frames go on without it. set_gains()
        ends float mode: with no arguments the profile's gains come back. A fault, or a failure of
        the gravity feed-forward, ends it too: the arm stiffens at the measured pose with the
        profile's gains and holds there as on any fault."""
        n = self.config.n_joints
        if extra_torque is not None and not callable(extra_torque):
            raise TypeError("extra_torque must be callable")
        self.set_gains(kp=[0.0] * n, kd=[self.FLOAT_KD if kd is None else kd] * n)
        with self._target_lock:
            self._extra_torque = extra_torque
            self._floating = True

    def _end_float(self) -> None:
        """Leave float mode on a fault: targets at the measured pose, then the profile gains, no extra torque."""
        positions = self.get_joint_positions()
        with self._target_lock:
            for i, p in enumerate(positions):
                if math.isfinite(p):
                    self._user_targets[i] = self._internal[i] = p
            self.config.joints = [replace(j, kp=p, kd=d) for j, (p, d) in zip(self.config.joints, self._profile_gains)]
            self._extra_torque = None
            self._floating = False

    # ── kinematics (needs the profile's urdf) ──
    @property
    def chain(self) -> kinematics.Chain:
        """The arm's kinematic chain from the profile (built on first use). ArmError without a URDF."""
        cfg = self.config
        joints = cfg.joints
        key = (str(cfg.urdf) if cfg.urdf else None, tuple(cfg.urdf_joints or ()), tuple(cfg.urdf_base_rpy),
               tuple(cfg.tcp_xyz), tuple(cfg.tcp_rpy), tuple((j.urdf_sign, j.urdf_offset) for j in joints))
        if self._chain is None or self._chain[0] != key:
            if not cfg.urdf:
                raise ArmError("the profile names no urdf: kinematics (ee_pose, move_ee) is unavailable")
            try:
                chain = kinematics.chain_from_config(cfg)
            except (OSError, ValueError) as e:
                raise ArmError(f"cannot build the kinematic chain from {cfg.urdf}: {e}") from e
            self._chain = (key, chain)
        return self._chain[1]

    def ee_pose(self) -> tuple[list[float], list[float]]:
        """Measured tool point: (xyz in m, rpy in rad) in the world frame."""
        chain = self.chain
        q = self.get_joint_positions()[:chain.n]
        if not all(math.isfinite(v) for v in q):
            raise StateError("no position feedback; connect() or refresh() first")
        xyz, rpy = kinematics.pose_to_xyz_rpy(chain.fk(q))
        return xyz.tolist(), rpy.tolist()

    def move_ee(self, xyz: Sequence[float], rpy: Optional[Sequence[float]] = None, speed: Optional[float] = None,
                timeout: Optional[float] = None, wait: bool = True, cancel: Optional[threading.Event] = None) -> bool:
        """Move the tool point in a straight line to ``xyz`` (m, world frame), turning to ``rpy`` (rad) when given;
        without it the end orientation is the one a position-only IK reaches with the least joint motion.
        ``speed`` caps each joint in rad/s, as in move_to().

        The path starts at the commanded pose and eases in and out (minimum jerk); each control tick of it is
        solved by the numpy trajectory IK (trajectory_ik.py; never the JAX one, whose first solve compiles for tens
        of seconds) and streamed through the joint targets, so no tick moves
        a joint further than its speed allows and a small tool move stays a small joint move. Joints past the
        chain (the gripper) keep their command. Refused (False, nothing sent) when not ENABLED or when the end
        pose is out of reach within the soft limits. With ``wait`` it blocks until arrival as move_to() does
        (False on timeout, a fault or newer targets taking over); without it the path streams in the background.
        Setting ``cancel`` stops the motion where it is (holding the measured pose) and returns False.
        ArmError without a URDF.
        """
        chain = self.chain
        if self._state is not ArmState.ENABLED:
            log.warning("move_ee rejected: state=%s", self._state.name)
            return False
        target = [float(v) for v in xyz]
        if len(target) != 3 or (rpy is not None and len(rpy) != 3):
            raise ValueError("xyz (and rpy) need three values")
        if speed is not None and not (math.isfinite(speed) and speed > 0):
            raise ValueError("speed must be positive")
        cfg = self.config
        ik = trajectory_ik.TrajectoryIK.from_config(cfg, chain=chain)
        current = self.get_commanded_positions()
        q0 = np.asarray(current[:chain.n], dtype=float)
        end = chain.ik(target, np.clip(q0, ik.lo, ik.hi), rpy=None if rpy is None else [float(v) for v in rpy],
                       limits=(ik.lo, ik.hi))
        if not end.ok:
            log.warning("move_ee rejected: no solution within the limits (%.1f mm, %.2f deg off)",
                        end.pos_err * 1e3, math.degrees(end.rot_err))
            return False
        start, goal = chain.fk(q0), chain.fk(end.q)
        goal[:3, 3] = target
        if rpy is not None:
            goal[:3, :3] = kinematics.rpy_to_matrix([float(v) for v in rpy])
        rate = cfg.control_rate_hz
        speeds = np.minimum(ik.max_velocity, math.inf if speed is None else speed)
        # Minimum jerk peaks at 1.875x the mean speed; a path the cap still shortens is retimed slower.
        seconds = max(EE_MIN_SECONDS, 1.25 * float(np.max(1.875 * np.abs(np.asarray(end.q) - q0) / speeds)))
        ticks = int(math.ceil(seconds * rate))
        for _ in range(4):
            # the goal held a while at the end lets the smoothed solve settle onto it
            hold = max(int(math.ceil(EE_MIN_SECONDS * rate / 2)), ticks // 5)
            poses = trajectory_ik.straight_line(start, goal, ticks)
            poses = np.concatenate([poses, np.repeat(goal[None], hold, axis=0)])
            plan = trajectory_ik.solve_path(ik, poses, q0, rate, speed)
            if not plan.capped.any():
                break
            ticks = int(math.ceil(ticks * 1.5))
        if plan.pos_err[-1] > EE_POSITION_TOL or plan.rot_err[-1] > EE_ANGLE_TOL:
            log.warning("move_ee rejected: the path ends %.1f mm, %.2f deg off the goal",
                        plan.pos_err[-1] * 1e3, math.degrees(plan.rot_err[-1]))
            return False
        rows = [q.tolist() + current[chain.n:] for q in plan.q]
        if cancel is not None and cancel.is_set():
            return False
        deadline = None if timeout is None else time.monotonic() + timeout
        if not wait:
            if not self._set_targets(rows[0], speed):
                return False
            threading.Thread(target=self._stream, args=(rows, speed, cancel, deadline), daemon=True,
                             name="move_ee").start()
            return True
        if not self._stream(rows, speed, cancel, deadline):
            return False
        remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
        return self.move_to(rows[-1], speed=speed, timeout=remaining, wait=True, cancel=cancel)

    def _stream(self, rows: list[list[float]], speed: Optional[float], cancel: Optional[threading.Event],
                deadline: Optional[float]) -> bool:
        """Send ``rows`` as joint targets, one per control period. False when cancelled (holding the measured
        pose), past ``deadline``, refused, or when newer targets take over."""
        period = 1.0 / self.config.control_rate_hz
        due = time.monotonic()
        sent = None
        for row in rows:
            if cancel is not None and cancel.is_set():
                self.hold_current_position()
                return False
            if deadline is not None and time.monotonic() > deadline:
                return False
            with self._target_lock:
                if sent is not None and self._user_targets != sent:
                    return False
            if not self._set_targets(row, speed):
                return False
            sent = row
            due += period
            time.sleep(max(0.0, due - time.monotonic()))
        return True

    # ── targets ──
    def set_joint_targets(self, targets: list[float]) -> bool:
        return self._set_targets(targets, None)

    def _set_targets(self, targets: Sequence[float], speed: Optional[float]) -> bool:
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
            self._speed = speed
        return True

    def move_to(self, q: Sequence[float], speed: Optional[float] = None, timeout: Optional[float] = None,
                wait: bool = True, cancel: Optional[threading.Event] = None) -> bool:
        """Drive to joint pose ``q`` (rad) at up to ``speed`` rad/s (default: max_velocity).

        With ``wait`` it blocks until every stiff joint (kp > 0) is within rest_arrived of its
        clipped goal and returns True; False on timeout (default: generous for the distance), a
        fault, the loop stopping, or newer targets taking over. Setting ``cancel`` stops the
        motion where it is (holding the measured pose) and returns False. Targets are refused
        unless ENABLED.
        """
        q = [float(x) for x in q]
        if speed is not None and not (math.isfinite(speed) and speed > 0):
            raise ValueError("speed must be positive")
        if cancel is not None and cancel.is_set():
            return False
        if not self._set_targets(q, speed):
            return False
        if not wait:
            return True
        cfg = self.config
        goal = [min(j.hi - cfg.limit_margin, max(j.lo + cfg.limit_margin, t)) for j, t in zip(cfg.joints, q)]
        rate = cfg.max_velocity if speed is None else min(cfg.max_velocity, speed)
        if timeout is None:
            start = self.get_joint_positions()
            distance = max((abs(g - s) for g, s in zip(goal, start) if math.isfinite(s)), default=0.0)
            timeout = 1.5 * distance / rate + 2.0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._state is not ArmState.ENABLED or not self._running:
                return False
            if cancel is not None and cancel.is_set():
                self.hold_current_position()
                return False
            with self._target_lock:
                if self._user_targets != q:
                    return False
            pos = self.get_joint_positions()
            if all(math.isfinite(p) and abs(p - g) <= cfg.rest_arrived
                   for p, g, j in zip(pos, goal, cfg.joints) if j.kp > 0):
                return True
            time.sleep(0.005)
        return False

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
            self._speed = None
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
        duration = worst / cfg.rest_speed
        began = time.monotonic()
        deadline = began + cfg.rest_timeout
        frac = 0.0
        while frac < 1.0:
            # The ramp follows the clock, so late wake-ups do not slow it below rest_speed.
            frac = min(1.0, max(period, time.monotonic() - began) / duration)
            if time.monotonic() >= deadline or not self._rest_active():
                break
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
            speed = cfg.max_velocity if self._speed is None else min(cfg.max_velocity, self._speed)
            maxstep = speed * dt
            for i, j in enumerate(cfg.joints):
                if self._skipped(i):
                    continue   # released: off; faulted: its target stays frozen at the fault pose
                if j.kp == 0:
                    # Floating joint: the command follows the measured pose, so restoring kp holds here.
                    if math.isfinite(present[i]):
                        self._user_targets[i] = self._internal[i] = present[i]
                    continue
                goal = min(j.hi - cfg.limit_margin, max(j.lo + cfg.limit_margin, self._user_targets[i]))
                if not self._synced[i] and self._targets_received and math.isfinite(present[i]):
                    goal = self._startup_sync(i, goal, present[i], now)
                step = min(maxstep, max(-maxstep, goal - self._internal[i]))
                self._internal[i] += step
            commands = list(self._internal)
        feedforward = self._feedforward(commands, now)
        paced = self._backend.feedback_paced_control
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
                if hold is not None and i not in self._faulted and kp > 0:
                    position, kp, kd = self._gripper_command(i, j, hold, commands[i], now)
                motor.send_command(self._to_motor(i, position), 0.0, kp, kd, feedforward[i])
            except Exception as e:
                if i not in self._faulted:
                    self._enter_fault(f"{j.label}: command failed ({e})", i)
                continue
            if paced:
                # Old USB-SLCAN firmware has no outbound USB queue and a three-frame CAN RX
                # FIFO: wait for this reply before commanding the next motor. A timeout does not
                # skip the remaining motors; the feedback watchdog makes the safety decision.
                self._wait_reply(motor, sequence, self._backend.feedback_wait_timeout)
            else:
                self._busy_wait_us(150)
        # Non-clearing status poll: at most one motor per tick (the longest-waiting one) so the
        # replies never burst into a small adapter FIFO.
        due = [(self._poll_sent.get(m.motor_id, 0.0), i) for i, m in enumerate(self.motors)
               if not self._skipped(i)]
        if due and self._may_send(session):
            sent_at, i = min(due)
            if now - sent_at >= cfg.fault_poll_interval:
                self._poll_sent[self.motors[i].motor_id] = now
                try:
                    self.motors[i].poll_status()
                except Exception as e:
                    log.error("fault query to motor %d failed: %s", self.motors[i].motor_id, e)
        self._tick_count += 1
        self._check_health()

    def _make_gravity(self) -> Optional[gravity.GravityFeedforward]:
        """This session's gravity feed-forward, or None (no URDF or gravity_scale 0). Raises
        ValueError for a bad scale or a missing cap, before any frame goes out."""
        cfg = self.config
        if not 0 <= cfg.gravity_scale <= gravity.MAX_GRAVITY_SCALE:
            raise ValueError(f"gravity_scale must be in [0, {gravity.MAX_GRAVITY_SCALE}], got {cfg.gravity_scale}")
        if not cfg.urdf or not cfg.gravity_scale:
            return None
        try:
            chain = self.chain
        except ArmError as e:
            raise ValueError(str(e)) from e
        return gravity.from_config(cfg, chain)

    def _feedforward(self, commands: list[float], now: float) -> list[float]:
        """Torque per joint in motor sign for this tick's frames: gravity (0 past the model's
        joints) plus float mode's extra torque. A failure of either drops that term (gravity until
        the next enable, the extra torque until float_mode() again), never the hold frames."""
        cfg = self.config
        n = cfg.n_joints
        tau = [0.0] * n
        model = self._gravity
        if model is not None:
            try:
                k = model.n
                ages = [m.feedback_age for m in self.motors[:k]]
                tau[:k] = model.torques(self.get_joint_positions()[:k], ages, commands[:k], now)
            except Exception as e:
                log.exception("gravity feed-forward failed; off until the next enable")
                self._gravity = None
                tau = [0.0] * n
                if self._floating:
                    self._enter_fault(f"gravity feed-forward failed while floating ({e})")
        extra = self._extra_torque
        if extra is not None:
            try:
                add = [float(v) for v in extra(self.get_joint_positions(), self.get_joint_velocities())]
                if len(add) != n or not all(math.isfinite(v) for v in add):
                    raise ValueError(f"extra_torque must return {n} finite values, got {add}")
                tau = [t + a for t, a in zip(tau, add)]
            except Exception:
                log.exception("extra torque failed; off until float_mode() sets it again")
                self._extra_torque = None
        return [t * j.direction for t, j in zip(tau, cfg.joints)]

    def _gripper_command(self, i: int, j, hold: GripperHold, target: float, now: float):
        """Holding regulation on one fresh, self-consistent feedback sample; without a new sample
        the last output is reused. ``j`` is the tick's own copy of the joint, so a gain change
        made meanwhile cannot reach the regulator half-applied."""
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
        # Only the control loop asks for status, so a reply's age counts from when it took over.
        polling_for = time.monotonic() - self._polling_since
        for i, (m, j) in enumerate(zip(self.motors, cfg.joints)):
            if self._skipped(i):
                continue
            reason = None
            fb = m.feedback
            status_age = min(m.status_age, polling_for)
            if m.feedback_age > cfg.feedback_timeout:
                reason = f"feedback timeout {m.feedback_age:.2f}s -- check bus/power"
            elif m.fault_latched:
                reason = f"motor fault -- {m.fault_latched}"
            elif status_age > status_deadline:
                reason = f"no fault-status reply for {status_age:.2f}s"
            elif j.max_temperature_c is not None and fb.temperature >= j.max_temperature_c:
                reason = f"overtemperature {fb.temperature:.1f} °C (limit {j.max_temperature_c:.0f} °C)"
            elif j.warning_is_fault and fb.warning:
                reason = "motor warning flag set"
            else:
                reason = m.health_problem()
            if reason:
                self._enter_fault(f"{j.label}: {reason}", i)

    def _enter_fault(self, reason: str, index: Optional[int] = None) -> None:
        log.error("FAULT: %s", reason)
        self.fault_reasons.append(reason)
        if self.fault_reason is None:
            self.fault_reason = reason
        if self._floating:
            self._end_float()
        if index is not None:
            # A faulted joint holds its measured pose and is excluded from the return to rest
            # (which then cannot be verified); a release_on_fault joint (the gripper) is released
            # rather than held.
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
