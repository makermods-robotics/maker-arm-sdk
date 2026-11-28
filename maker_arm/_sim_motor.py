"""Simulated RobStride MIT motors behind MockBackend, for tests and ``testing``.

A SimMotor answers the frames the SDK sends: MIT commands (an enabled motor jumps to the commanded
position when kp > 0), enable/disable/zero/clear, the non-clearing status query and, when
``params_supported``, parameter reads/writes (run mode, CAN watchdog). ``log`` records what it
received ("status_query", "clear", "enable", "disable", "zero", ("set_id", new id), ("cmd", kp), ("write", index)),
``frames`` every MIT command as (position, kp, tau).
"""

import struct
from typing import Optional

from . import protocol as p
from .transport.mock import MockBackend

PREFIX = bytes([0xFF] * 6)
FAULT_CODE = 1                       # what ``faulted = True`` reports
FAULT_TEXT = "motor overtemperature"  # how the SDK describes FAULT_CODE

# What the protocol can show:
FULL_TURN_SUPPORTED = True   # a reading one whole turn off fits the position range (multi-turn count)
WARNING_SUPPORTED = True     # feedback carries a warning flag (``warning=True``)


def feedback_bytes(motor_id, pos=0.0, vel=0.0, tau=0.0, temp=30.0, mode=0, fault=False, warn=False,
                   params=p.RS00):
    q = p.float_to_uint(pos, params.p_min, params.p_max, 16)
    v = p.float_to_uint(vel, params.v_min, params.v_max, 12)
    t = p.float_to_uint(tau, params.t_min, params.t_max, 12)
    temp10 = int(round(temp * 10))
    status = (mode << 6) | (int(fault) << 5) | (int(warn) << 4) | ((temp10 >> 8) & 0x0F)
    return bytes([motor_id, q >> 8, q & 0xFF, v >> 4, ((v & 0xF) << 4) | (t >> 8), t & 0xFF,
                  status, temp10 & 0xFF])


def classify(can_id: int, data: bytes) -> tuple[str, Optional[int]]:
    """(kind, motor id) of a host->motor frame: "command", "enable", "disable", "zero", "set_id", "clear",
    "status_query", "param_read", "param_write" or "other"."""
    kind = can_id & 0x700
    if kind == 0x300:
        return "param_read", can_id & 0xFF
    if kind == 0x400:
        return "param_write", can_id & 0xFF
    if data[:6] == PREFIX:
        arg, op = data[6], data[7]
        name = {p.OP_ENABLE: "enable", p.OP_DISABLE: "disable", p.OP_SET_ZERO: "zero",
                p.OP_SET_ID: "set_id"}.get(op)
        if op == p.OP_CLEAR_FAULT:
            name = "status_query" if arg == 0x00 else "clear"
        return name or "other", can_id
    return "command", can_id


class SimMotor:
    def __init__(self, motor_id: int, model: str = "RS00", *, position: float = 0.0,
                 temperature: float = 30.0, enabled: bool = False, online: bool = True,
                 track: bool = True, faulted: bool = False, warning: bool = False,
                 fault_word: Optional[int] = None, fault_flag: bool = False,
                 params_supported: bool = False, run_mode: int = 0, can_timeout: int = 0,
                 host_id: int = p.HOST_CAN_ID, param_replies_to_host: bool = False):
        self.motor_id = motor_id
        self.model = model
        self.params = p.MOTOR_PARAMS[model]
        self.position = position
        self.temperature = temperature
        self.enabled = enabled
        self.online = online
        self.track = track               # an enabled motor jumps to the commanded position
        self.warning = warning
        self.fault_word = (FAULT_CODE if faulted else 0) if fault_word is None else fault_word
        self.fault_flag = fault_flag
        self.params_supported = params_supported   # False: old firmware, no parameter access, mode bits read 0
        self.run_mode = run_mode
        self.can_timeout = can_timeout
        self.host_id = host_id           # CAN id replies are sent to
        self.param_replies_to_host = param_replies_to_host   # True: parameter replies go to the host ID
        self.apply_id_change = True      # False: an id write is taken but the motor keeps its id
        self.uid = struct.pack("<Q", 0x5354524944450000 | motor_id)   # the MCU identifier an id write returns
        self.log: list = []
        self.last_kp = 0.0
        self.frames: list = []           # every MIT frame received: (position, kp, tau)

    @property
    def faulted(self) -> bool:
        return bool(self.fault_word or self.fault_flag)

    @faulted.setter
    def faulted(self, value: bool) -> None:
        self.fault_word = FAULT_CODE if value else 0
        if not value:
            self.fault_flag = False

    @property
    def mode(self) -> int:
        return (2 if self.enabled else 0) if self.params_supported else 0

    def feedback(self):
        return self.host_id, feedback_bytes(self.motor_id, self.position, temp=self.temperature,
                                            mode=self.mode, fault=self.fault_flag, warn=self.warning,
                                            params=self.params)

    def handle(self, can_id, data):
        if not self.online:
            return []
        kind, _ = classify(can_id, data)
        if kind in ("param_read", "param_write"):
            if not self.params_supported:
                return []
            idx = struct.unpack("<H", data[:2])[0]
            reply_id = self.host_id if self.param_replies_to_host else can_id
            if kind == "param_write":
                self.log.append(("write", idx))
                if idx == p.ParamIndex.CAN_TIMEOUT:
                    self.can_timeout = struct.unpack("<I", data[4:8])[0]
                return [(reply_id, bytes(data))]
            value = {p.ParamIndex.RUN_MODE: self.run_mode, p.ParamIndex.CAN_TIMEOUT: self.can_timeout}.get(idx, 0)
            return [(reply_id, bytes(data[:4]) + struct.pack("<I", value))]
        if kind == "status_query":
            self.log.append("status_query")
            return [(self.host_id, bytes([self.motor_id]) + struct.pack("<I", self.fault_word))]
        if kind == "set_id":
            self.log.append(("set_id", data[6]))
            if self.apply_id_change and not self.enabled:
                self.motor_id = data[6]
            return [(self.motor_id, self.uid)]
        if kind != "command":
            self.log.append(kind)
            if kind == "clear":
                self.fault_word, self.fault_flag = 0, False
            elif kind == "enable":
                self.enabled = True
            elif kind == "disable":
                self.enabled = False
            elif kind == "zero":
                self.position = 0.0
            return [self.feedback()]
        q = (data[0] << 8) | data[1]
        kp = ((data[3] & 0xF) << 8 | data[4]) * p.KP_MAX / 4095
        self.last_kp = kp
        self.log.append(("cmd", round(kp, 1)))
        position = p.uint_to_float(q, self.params.p_min, self.params.p_max, 16)
        tau = p.uint_to_float((data[6] & 0xF) << 8 | data[7], self.params.t_min, self.params.t_max, 12)
        self.frames.append((position, kp, tau))
        if self.enabled and self.track and kp > 0:
            self.position = position
        return [self.feedback()]


def respond(motors: dict, can_id: int, data: bytes) -> list:
    """Replies of the simulated motors to one host frame (motor replies are ignored)."""
    if getattr(can_id, "extended", False) or can_id == p.HOST_CAN_ID:
        return []
    key = can_id & 0x7F
    m = motors.get(key)
    if m is None:
        return []
    replies = m.handle(can_id, data)
    if m.motor_id != key and m.motor_id not in motors:      # an id write moved it
        motors[m.motor_id] = motors.pop(key)
    return replies


class SimBus(MockBackend):
    """MockBackend whose responder routes each frame to the SimMotor it addresses."""

    def __init__(self, motors):
        super().__init__()
        self.motors = {m.motor_id: m for m in motors}
        self.responder = lambda can_id, data: respond(self.motors, can_id, data)


def watchdog_timeout(motor: SimMotor) -> None:
    """What the motor's CAN watchdog does once commands stop: reset mode (torque off), no fault."""
    motor.enabled = False


def for_joint(joint, host_id: int, position: float = 0.0, **options) -> SimMotor:
    """A SimMotor answering like the motor of ``joint`` (a JointConfig) at ``position`` (motor rad).
    Firmware with parameter access unless ``params_supported=False``."""
    options.setdefault("params_supported", True)
    return SimMotor(joint.motor_id, joint.model, position=position, host_id=host_id, **options)
