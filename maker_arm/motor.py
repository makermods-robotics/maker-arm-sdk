"""Motor layer: one CAN ID = one Motor. Commands are async, queries are sync, feedback is timestamped.

Fault evidence is preserved the way MakerModsLab does it: once a motor reports a fault (fault
word or feedback fault flag), every command that would clear it is refused until the caller
explicitly runs clear_fault().
"""

import math
import threading
import time
from typing import Optional

from . import protocol
from .errors import FaultLatched, ParamTimeout
from .protocol import FaultStatus, MotorFeedback, ParamReply
from .transport.base import CanBackend


class Motor:
    def __init__(self, motor_id: int, backend: CanBackend,
                 params: protocol.MotorParams = protocol.RS00):
        self.motor_id = motor_id
        self._backend = backend
        self.params = params
        self._feedback: Optional[MotorFeedback] = None
        self._fb_time: Optional[float] = None
        self._feedback_sequence = 0
        self.fault_status: Optional[FaultStatus] = None
        self._fault_time: Optional[float] = None
        self._fault_sequence = 0
        self._latched: Optional[str] = None
        self._query_lock = threading.Lock()
        self._param_event = threading.Event()
        self._param_reply: Optional[ParamReply] = None
        self._param_expect: Optional[int] = None
        # None = unknown, True/False after probe_params(). Typed parameter access needs
        # RS00 >= 0.0.3.27 / RS02 >= 0.2.3.32; that firmware also reports mode/fault bits.
        self.params_supported: Optional[bool] = None

    # -- receive path (invoked by the RX dispatcher) --
    def handle_frame(self, msg) -> None:
        if msg is None or getattr(msg, "motor_id", None) != self.motor_id:
            return
        if isinstance(msg, MotorFeedback):
            self._feedback = msg
            self._fb_time = time.monotonic()
            self._feedback_sequence += 1
            if msg.fault_flag and self._latched is None:
                self._latched = "feedback fault flag set"
        elif isinstance(msg, FaultStatus):
            self.fault_status = msg
            self._fault_time = time.monotonic()
            self._fault_sequence += 1
            if msg.has_fault and self._latched is None:
                self._latched = f"fault word 0x{msg.word:08X} ({msg.text})"
        elif isinstance(msg, ParamReply):
            if not msg.write and msg.index == self._param_expect:
                self._param_reply = msg
                self._param_event.set()

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
    def fault_age(self) -> float:
        return math.inf if self._fault_time is None else time.monotonic() - self._fault_time

    @property
    def fault_latched(self) -> Optional[str]:
        """Reason a fault was latched, or None."""
        return self._latched

    def reset_cache(self) -> None:
        self._feedback = None
        self._fb_time = None
        self.fault_status = None
        self._fault_time = None
        self._latched = None

    def _send(self, frame) -> None:
        self._backend.send(*frame, extended=False)

    # -- async commands --
    def enable(self) -> None:
        self._send(protocol.encode_enable(self.motor_id))

    def disable(self) -> None:
        self._send(protocol.encode_disable(self.motor_id))

    def disable_confirmed(self, timeout: float = 0.05, attempts: int = 3) -> bool:
        """Disable and confirm the reply (and mode 0 where the firmware reports mode)."""
        for _ in range(attempts):
            sequence = self._feedback_sequence
            self.disable()
            deadline = time.monotonic() + timeout
            while self._feedback_sequence == sequence and time.monotonic() < deadline:
                time.sleep(0.0005)
            if self._feedback_sequence != sequence and (
                    not self.params_supported or self._feedback.mode == protocol.MODE_RESET):
                return True
        return False

    def set_zero(self) -> None:
        """Make the current position the motor's zero. Persists in the motor."""
        self._send(protocol.encode_set_zero(self.motor_id))

    def send_command(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self._send(protocol.encode_control(self.motor_id, pos, vel, kp, kd, tau, self.params))

    def request_state(self) -> None:
        """Request feedback without changing torque state (the LeRobot read frame).

        That frame also clears faults, so it is refused once a fault is latched.
        """
        if self._latched is not None:
            raise FaultLatched(f"motor {self.motor_id}: {self._latched}; not sending a fault-clearing read")
        self._send(protocol.encode_clear_fault(self.motor_id))

    def query_fault_async(self) -> None:
        """Send a non-clearing fault query; the reply updates fault_status."""
        self._send(protocol.encode_fault_query(self.motor_id))

    def clear_fault(self) -> None:
        """Explicit operator action: clear the motor fault and the latched evidence."""
        self._latched = None
        self._send(protocol.encode_clear_fault(self.motor_id))

    # -- sync queries --
    def read_state(self, timeout: float = 0.05, attempts: int = 3) -> MotorFeedback:
        """Fresh feedback (not cached). Raises ParamTimeout after every attempt misses."""
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

    def read_state_safe(self, timeout: float = 0.05) -> MotorFeedback:
        """Fresh feedback without erasing a known fault: a non-clearing fault query comes first and
        a reported fault raises FaultLatched instead of sending the clearing state read. A fault
        arising in the milliseconds between the two is still cleared by the read, as in LeRobot."""
        status = self.query_fault(timeout)
        if status.has_fault:
            raise FaultLatched(f"motor {self.motor_id}: {self._latched}")
        return self.read_state(timeout, attempts=1)

    def query_fault(self, timeout: float = 0.1, attempts: int = 1) -> FaultStatus:
        """Fresh, non-clearing fault query. Does not refresh encoder feedback."""
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        with self._query_lock:
            for _ in range(attempts):
                sequence = self._fault_sequence
                self.query_fault_async()
                deadline = time.monotonic() + timeout
                while self._fault_sequence == sequence and time.monotonic() < deadline:
                    time.sleep(0.0005)
                if self._fault_sequence != sequence:
                    return self.fault_status
        raise ParamTimeout(f"motor {self.motor_id} fault query timed out")

    def read_param(self, index: int, dtype: str = "f", timeout: float = 0.12):
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

    def write_param(self, index: int, value, dtype: str = "f", timeout: float = 0.12):
        """Write a parameter and return the read-back value (raises ParamTimeout)."""
        self._send(protocol.encode_write_param(self.motor_id, index, value, dtype))
        time.sleep(0.005)
        return self.read_param(index, dtype, timeout)

    def probe_params(self, timeout: float = 0.12, attempts: int = 2) -> bool:
        """Detect typed parameter support (newer firmware). Read-only."""
        self.params_supported = False
        for _ in range(attempts):
            try:
                self.read_param(protocol.ParamIndex.RUN_MODE, "u8", timeout)
                self.params_supported = True
                break
            except ParamTimeout:
                continue
        return self.params_supported
