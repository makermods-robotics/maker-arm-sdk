"""RobStride motor behind the Motor seam: one CAN ID = one Motor. Commands are async, queries
are sync, feedback is timestamped. Everything RobStride-specific that the arm layer needs lives
here (frame routing, fault query, RUN_MODE check, mode bits, CAN-timeout watchdog, firmware
parameter support); protocol.py stays a pure codec.

Fault evidence is preserved: once a motor reports a fault (fault word or feedback fault flag),
every command that would clear it is refused until the caller explicitly runs clear_fault().
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, NamedTuple, Optional, Union

from . import protocol
from .errors import ArmError, ConnectError, FaultLatched, ParamTimeout
from .transport.base import CanBackend

if TYPE_CHECKING:
    from .config import JointConfig

log = logging.getLogger(__package__)

MODELS: tuple[str, ...] = tuple(protocol.MOTOR_PARAMS)
DEFAULT_MODEL = "RS00"
KP_RANGE = (protocol.KP_MIN, protocol.KP_MAX)
KD_RANGE = (protocol.KD_MIN, protocol.KD_MAX)
ID_RANGE = (1, 127)
DEFAULT_HOST_ID = protocol.HOST_CAN_ID


def route(can_id: int, data: bytes, host_id: int = DEFAULT_HOST_ID) -> Optional[int]:
    """Motor ID a received frame belongs to, or None. Parameter replies carry it in the CAN ID
    (their data[0] is a register byte); MIT replies arrive on the host ID with it in data[0]."""
    if getattr(can_id, "extended", False) or not 0 <= can_id <= 0x7FF:
        return None
    if can_id & 0x700 in (0x300, 0x400):
        return can_id & 0xFF
    if can_id == host_id and data:
        return data[0]
    return None


@dataclass(frozen=True)
class MotorFeedback:
    """One feedback sample, motor coordinates."""
    motor_id: int
    position: float                # rad
    velocity: float                # rad/s
    torque: float                  # N·m
    temperature: float             # °C, hottest sensor
    temperatures: dict = field(default_factory=dict)
    enabled: Optional[bool] = None  # None: old firmware reports no mode bits
    warning: bool = False
    mode: int = 0                  # RobStride: 0 reset, 1 calibration, 2 run
    fault_flag: bool = False


@dataclass(frozen=True)
class MotorStatus:
    motor_id: int
    has_fault: bool
    code: int                      # MIT fault word, see protocol.FAULT_BITS
    text: str                      # "" when healthy
    enabled: Optional[bool] = None  # the fault-status reply carries no mode


class ParamSpec(NamedTuple):
    index: int
    dtype: str
    doc: str


class Motor:
    MODE_FAULT_TICKS = 5   # consecutive health checks a motor may report mode != run
    PARAMS: dict[str, ParamSpec] = {
        "run_mode": ParamSpec(protocol.ParamIndex.RUN_MODE, "u8", "control mode, 0 = MIT"),
        "mech_pos": ParamSpec(protocol.ParamIndex.MECH_POS, "f", "mechanical position, rad"),
        "can_timeout": ParamSpec(protocol.ParamIndex.CAN_TIMEOUT, "u32",
                                 "CAN timeout, 20 ticks per ms; 0 disables the watchdog"),
    }

    def __init__(self, joint: "JointConfig", backend: CanBackend, host_id: int = DEFAULT_HOST_ID):
        self.joint = joint
        self.motor_id = joint.motor_id
        self._backend = backend
        self._host_id = host_id
        self.params = protocol.MOTOR_PARAMS[joint.model]
        self._feedback: Optional[MotorFeedback] = None
        self._fb_time: Optional[float] = None
        self._feedback_sequence = 0
        self._status: Optional[MotorStatus] = None
        self._status_time: Optional[float] = None
        self._status_sequence = 0
        self._latched: Optional[str] = None
        self._query_lock = threading.Lock()
        self._param_event = threading.Event()
        self._param_reply: Optional[protocol.ParamReply] = None
        self._param_expect: Optional[int] = None
        self._mode_bad = 0
        # None = unknown, True/False after probe_params(). Typed parameter access needs
        # RS00 >= 0.0.3.27 / RS02 >= 0.2.3.32; that firmware also reports mode/fault bits.
        self.params_supported: Optional[bool] = None

    @property
    def torque_limit(self) -> float:
        """Largest torque the MIT frame can carry for this model, N·m."""
        return self.params.t_max

    # -- receive path (invoked by the RX dispatcher) --
    def handle_frame(self, can_id: int, data: bytes) -> None:
        msg = protocol.parse_frame(can_id, data, self.params, self._host_id)
        if msg is None or msg.motor_id != self.motor_id:
            return
        if isinstance(msg, protocol.MotorFeedback):
            self._feedback = MotorFeedback(
                motor_id=msg.motor_id, position=msg.position, velocity=msg.velocity,
                torque=msg.torque, temperature=msg.temperature,
                temperatures={"motor": msg.temperature},
                enabled=(msg.mode == protocol.MODE_RUN) if self.params_supported else None,
                warning=msg.warning_flag, mode=msg.mode, fault_flag=msg.fault_flag)
            self._fb_time = time.monotonic()
            self._feedback_sequence += 1
            if msg.fault_flag and self._latched is None:
                self._latched = "feedback fault flag set"
        elif isinstance(msg, protocol.FaultStatus):
            self._status = MotorStatus(msg.motor_id, msg.has_fault, msg.word,
                                       msg.text if msg.has_fault else "")
            self._status_time = time.monotonic()
            self._status_sequence += 1
            if msg.has_fault and self._latched is None:
                self._latched = f"fault word 0x{msg.word:08X} ({msg.text})"
        elif isinstance(msg, protocol.ParamReply):
            if not msg.write and msg.index == self._param_expect:
                self._param_reply = msg
                self._param_event.set()

    # -- cached state --
    @property
    def feedback(self) -> Optional[MotorFeedback]:
        return self._feedback

    @property
    def feedback_age(self) -> float:
        return math.inf if self._fb_time is None else time.monotonic() - self._fb_time

    @property
    def feedback_sequence(self) -> int:
        """Monotonic count used to distinguish a fresh reply from cached feedback."""
        return self._feedback_sequence

    @property
    def status(self) -> Optional[MotorStatus]:
        return self._status

    @property
    def status_age(self) -> float:
        return math.inf if self._status_time is None else time.monotonic() - self._status_time

    @property
    def fault_latched(self) -> Optional[str]:
        """Reason a fault was latched, or None."""
        return self._latched

    def reset_cache(self) -> None:
        self._feedback = None
        self._fb_time = None
        self._status = None
        self._status_time = None
        self._latched = None
        self._mode_bad = 0

    def _send(self, frame) -> None:
        self._backend.send(*frame, extended=False)

    # -- async commands --
    def enable(self) -> None:
        self._mode_bad = 0
        self._send(protocol.encode_enable(self.motor_id))

    def disable(self) -> None:
        self._send(protocol.encode_disable(self.motor_id))

    def send_command(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self._send(protocol.encode_control(self.motor_id, pos, vel, kp, kd, tau, self.params))

    def _wait_feedback(self, sequence: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while self._feedback_sequence == sequence and time.monotonic() < deadline:
            time.sleep(0.0005)
        return self._feedback_sequence != sequence

    def _wait_quiet(self, quiet: float = 0.005, limit: float = 0.05) -> None:
        """Let late replies to earlier requests arrive, so the next reply answers ours (MIT
        replies carry no transaction ID)."""
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            sequence = self._feedback_sequence
            time.sleep(quiet)
            if self._feedback_sequence == sequence:
                return

    def preload(self, pos: float, timeout: float = 0.05) -> bool:
        """Zero-gain command at ``pos`` (motor coordinates); True once the motor replies."""
        if not math.isfinite(pos):
            return False
        self._wait_quiet()
        sequence = self._feedback_sequence
        self.send_command(pos, 0.0, 0.0, 0.0, 0.0)
        return self._wait_feedback(sequence, timeout)

    def enable_confirmed(self, timeout: float = 0.05) -> bool:
        """Enable; True once a reply arrives that confirms torque on (any reply on old firmware)."""
        sequence = self._feedback_sequence
        self.enable()
        return self._wait_feedback(sequence, timeout) and self.confirms("enabled")

    def poll_status(self) -> None:
        """Send a non-clearing fault query; the reply updates status."""
        self._send(protocol.encode_fault_query(self.motor_id))

    def request_state(self) -> None:
        """Request feedback without changing torque state.

        That frame also clears faults, so it is refused once a fault is latched.
        """
        if self._latched is not None:
            raise FaultLatched(f"motor {self.motor_id}: {self._latched}; not sending a fault-clearing read")
        self._send(protocol.encode_clear_fault(self.motor_id))

    # -- sync queries --
    def check(self, timeout: float = 0.05, attempts: int = 3) -> MotorStatus:
        """Fresh, non-clearing fault query. Never changes torque; does not refresh feedback."""
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        with self._query_lock:
            for _ in range(attempts):
                sequence = self._status_sequence
                self.poll_status()
                deadline = time.monotonic() + timeout
                while self._status_sequence == sequence and time.monotonic() < deadline:
                    time.sleep(0.0005)
                if self._status_sequence != sequence:
                    return self._status
        raise ParamTimeout(f"motor {self.motor_id} fault query timed out")

    def read_state(self, timeout: float = 0.05, attempts: int = 3) -> MotorFeedback:
        """Fresh feedback (not cached). Raises ParamTimeout after every attempt misses.

        RobStride's state read also clears faults, so it raises FaultLatched once a fault is
        latched; run check() first so a fault the motor reports is latched, not cleared.
        """
        for attempt in range(attempts):
            sequence = self._feedback_sequence
            self.request_state()
            deadline = time.monotonic() + timeout
            while self._feedback_sequence == sequence and time.monotonic() < deadline:
                time.sleep(0.0005)
            if self._feedback_sequence != sequence:
                return self._feedback
            if attempt + 1 < attempts:
                time.sleep(0.05)
        raise ParamTimeout(f"motor {self.motor_id} did not reply to a state request")

    def validate(self, timeout: float = 0.12) -> None:
        """Connect-time check: firmware with parameter access must be in MIT control mode."""
        if self.probe_params(timeout, attempts=2):
            run_mode = self.read_param("run_mode", timeout=timeout)
            if run_mode != 0:
                raise ConnectError(f"motor {self.motor_id} RUN_MODE is {run_mode}, not 0 (MIT control)")

    def confirms(self, expected: str) -> bool:
        """Does the latest feedback confirm "enabled" / "disabled"? Old firmware has no mode
        bits, so any reply counts there."""
        want = {"enabled": protocol.MODE_RUN, "disabled": protocol.MODE_RESET}[expected]
        fb = self._feedback
        if fb is None:
            return False
        return not self.params_supported or fb.mode == want

    def health_problem(self) -> Optional[str]:
        """Per-tick check while enabled: the motor left run mode for MODE_FAULT_TICKS ticks.
        Mode bits only exist on firmware with parameter access (old firmware reports 0); calibration
        (1) and undefined (3) modes are never legitimate."""
        fb = self._feedback
        if fb is None:
            return None
        bad = fb.mode in (protocol.MODE_CALIBRATION, 3) or (
            bool(self.params_supported) and fb.mode != protocol.MODE_RUN)
        self._mode_bad = self._mode_bad + 1 if bad else 0
        if self._mode_bad >= self.MODE_FAULT_TICKS:
            return f"left run mode (mode={fb.mode} for {self._mode_bad} ticks)"
        return None

    # -- operator actions --
    def clear_fault(self) -> None:
        """Explicit operator action: clear the motor fault and the latched evidence."""
        self._latched = None
        self._send(protocol.encode_clear_fault(self.motor_id))

    def set_zero(self) -> None:
        """Make the current position the motor's zero. Persists in the motor."""
        self._send(protocol.encode_set_zero(self.motor_id))

    def set_id(self, new_id: int, timeout: float = 0.5) -> None:
        raise ArmError(
            "changing a RobStride motor's CAN ID over the MIT protocol is not supported: the "
            "ID-change frame is not documented in the sources this SDK follows. Set the ID with "
            "RobStride Motor Studio (docs/motor-replacement.md)")

    def arm_watchdog(self, timeout_ms: int, timeout: float = 0.12) -> Optional[int]:
        """Set and verify the motor-side CAN timeout. Returns the previous value as the restore
        token, or None when the firmware has no parameter access."""
        if not self.params_supported:
            log.warning("%s: firmware has no parameter access (needs RS00 >=0.0.3.27 / RS02 >=0.2.3.32); "
                        "running without its %d ms motor watchdog", self.joint.label, timeout_ms)
            return None
        ticks = timeout_ms * protocol.CAN_TIMEOUT_TICKS_PER_MS
        original = self.read_param("can_timeout", timeout=timeout)
        try:
            readback = self.write_param("can_timeout", ticks, timeout=timeout)
            if readback != ticks:
                raise ConnectError(f"{self.joint.label}: CAN timeout read back {readback}, expected {ticks}")
        except BaseException:
            try:
                self.restore_watchdog(original, timeout)
            except Exception as e:
                log.error("could not restore %s CAN timeout: %s", self.joint.label, e)
            raise
        return original

    def restore_watchdog(self, token: int, timeout: float = 0.12) -> None:
        self.write_param("can_timeout", token, timeout=timeout)

    # -- engineering --
    def _param(self, name_or_index: Union[str, int], dtype: Optional[str]) -> tuple[int, str]:
        if isinstance(name_or_index, str):
            if name_or_index not in self.PARAMS:
                raise ValueError(f"unknown parameter {name_or_index!r} (known: {sorted(self.PARAMS)})")
            spec = self.PARAMS[name_or_index]
            return spec.index, dtype or spec.dtype
        for spec in self.PARAMS.values():
            if spec.index == name_or_index:
                return spec.index, dtype or spec.dtype
        return int(name_or_index), dtype or "f"

    def read_param(self, name_or_index: Union[str, int], dtype: Optional[str] = None, timeout: float = 0.12):
        index, dtype = self._param(name_or_index, dtype)
        with self._query_lock:
            self._param_event.clear()
            self._param_reply = None
            self._param_expect = index
            try:
                self._send(protocol.encode_read_param(self.motor_id, index))
                if not self._param_event.wait(timeout):
                    raise ParamTimeout(
                        f"motor {self.motor_id} read param 0x{index:04X} timed out after {timeout}s")
                return self._param_reply.value(dtype)
            finally:
                self._param_expect = None

    def write_param(self, name_or_index: Union[str, int], value, dtype: Optional[str] = None,
                    timeout: float = 0.12, save: bool = False):
        """Write a parameter and return the read-back value (raises ParamTimeout)."""
        if save:
            raise ArmError("saving parameters to flash is not supported over the RobStride MIT protocol")
        index, dtype = self._param(name_or_index, dtype)
        self._send(protocol.encode_write_param(self.motor_id, index, value, dtype))
        time.sleep(0.005)
        return self.read_param(index, dtype, timeout)

    def probe_params(self, timeout: float = 0.12, attempts: int = 2) -> bool:
        """Detect typed parameter support (newer firmware). Read-only."""
        self.params_supported = False
        for _ in range(attempts):
            try:
                self.read_param("run_mode", timeout=timeout)
                self.params_supported = True
                break
            except ParamTimeout:
                continue
        return self.params_supported

    def disable_confirmed(self, timeout: float = 0.05, attempts: int = 3) -> bool:
        """Disable and confirm the reply (and mode 0 where the firmware reports mode)."""
        for _ in range(attempts):
            sequence = self._feedback_sequence
            self.disable()
            deadline = time.monotonic() + timeout
            while self._feedback_sequence == sequence and time.monotonic() < deadline:
                time.sleep(0.0005)
            if self._feedback_sequence != sequence and self.confirms("disabled"):
                return True
        return False
