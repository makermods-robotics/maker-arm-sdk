"""Motor layer: one CAN ID = one Motor. Commands are async, param read/write is sync, feedback cache is timestamped."""

import math
import threading
import time
from typing import Optional

from . import protocol
from .errors import ParamTimeout
from .protocol import FaultReport, MotorFeedback, ParamReply
from .transport.base import CanBackend


class Motor:
    def __init__(self, motor_id: int, backend: CanBackend,
                 host_id: int = protocol.HOST_CAN_ID,
                 params: protocol.MotorParams = None):
        self.motor_id = motor_id
        self._backend = backend
        self._host_id = host_id
        self.params = params or protocol.RS00   # model lookup table (T/V range differs by model)
        self._feedback: Optional[MotorFeedback] = None
        self._fb_time: Optional[float] = None
        self._feedback_sequence = 0
        self._param_lock = threading.Lock()
        self._param_event = threading.Event()
        self._param_reply: Optional[ParamReply] = None
        self._param_expect: Optional[int] = None
        self.last_fault_report: Optional[FaultReport] = None

    # -- feedback cache (invoked by Arm's RX dispatch) --
    def handle_frame(self, msg) -> None:
        if msg is None or getattr(msg, "motor_id", None) != self.motor_id:
            return
        if isinstance(msg, MotorFeedback):
            self._feedback = msg
            self._fb_time = time.monotonic()
            self._feedback_sequence += 1
        elif isinstance(msg, ParamReply):
            if msg.index == self._param_expect:
                self._param_reply = msg
                self._param_event.set()
        elif isinstance(msg, FaultReport):
            self.last_fault_report = msg

    @property
    def feedback(self) -> Optional[MotorFeedback]:
        return self._feedback

    @property
    def feedback_age(self) -> float:
        if self._fb_time is None:
            return math.inf
        return time.monotonic() - self._fb_time

    @property
    def feedback_sequence(self) -> int:
        """Monotonic count used to distinguish a fresh reply from cached feedback."""
        return self._feedback_sequence

    # -- async commands --
    def enable(self) -> None:
        self._backend.send(*protocol.encode_enable(self.motor_id, self._host_id))

    def disable(self, clear_fault: bool = False) -> None:
        self._backend.send(*protocol.encode_disable(self.motor_id, clear_fault, self._host_id))

    def probe(self) -> None:
        """Send a stop frame to trigger one feedback frame (the RobStride read-only probing convention).

        ⚠️ Calling this on an already-enabled motor will stop it (release torque) -- only use when the motor is not enabled.
        """
        self.disable()

    def set_zero(self) -> None:
        self._backend.send(*protocol.encode_set_zero(self.motor_id, self._host_id))

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self._backend.send(*protocol.encode_mit(self.motor_id, pos, vel, kp, kd, tau, self.params))

    # -- sync param read/write --
    def read_param(self, index: int, dtype: str = "f", timeout: float = 0.1):
        with self._param_lock:
            self._param_event.clear()
            self._param_reply = None
            self._param_expect = index
            try:
                self._backend.send(*protocol.encode_read_param(self.motor_id, index, self._host_id))
                if not self._param_event.wait(timeout):
                    raise ParamTimeout(f"motor {self.motor_id} read param 0x{index:04X} timed out after {timeout}s -- check bus/power/ID")
                return self._param_reply.value(dtype)
            finally:
                self._param_expect = None

    def write_param(self, index: int, value, dtype: str = "f") -> None:
        self._backend.send(*protocol.encode_write_param(self.motor_id, index, value, dtype, self._host_id))

    def save_params(self) -> None:
        self._backend.send(*protocol.encode_save_params(self.motor_id, self._host_id))
