"""Simulated RobStride MIT motors behind MockBackend: enough behaviour to test the Arm pipeline.

Shared tests use only the generic surface: FakeMotor(motor_id, model=..., position=...,
temperature=..., enabled=..., online=..., track=..., faulted=..., warning=...), its ``log``
("status_query", "clear", "enable", "disable", "zero", ("cmd", kp)), ``frames`` ((position,
kp, tau) per command), ``last_kp`` and ``handle(can_id, data) -> replies``; FakeBus;
two_joint_config(); make_arm(); classify(); FAULT_CODE / FAULT_TEXT; FULL_TURN_SUPPORTED /
WARNING_SUPPORTED. Everything else is RobStride detail.
"""

import struct
from typing import Optional

from maker_arm import protocol as p
from maker_arm.arm import Arm
from maker_arm.config import ArmConfig, JointConfig
from maker_arm.transport.mock import MockBackend

PREFIX = bytes([0xFF] * 6)
FAULT_CODE = 1                       # what ``faulted = True`` reports
FAULT_TEXT = "motor overtemperature"  # how the SDK describes FAULT_CODE

# What the protocol can show, for shared tests that need it:
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
    """(kind, motor id) of a host->motor frame: "command", "enable", "disable", "zero", "clear",
    "status_query", "param_read", "param_write" or "other"."""
    kind = can_id & 0x700
    if kind == 0x300:
        return "param_read", can_id & 0xFF
    if kind == 0x400:
        return "param_write", can_id & 0xFF
    if data[:6] == PREFIX:
        arg, op = data[6], data[7]
        name = {p.OP_ENABLE: "enable", p.OP_DISABLE: "disable", p.OP_SET_ZERO: "zero"}.get(op)
        if op == p.OP_CLEAR_FAULT:
            name = "status_query" if arg == 0x00 else "clear"
        return name or "other", can_id
    return "command", can_id


class FakeMotor:
    def __init__(self, motor_id: int, model: str = "RS00", *, position: float = 0.0,
                 temperature: float = 30.0, enabled: bool = False, online: bool = True,
                 track: bool = True, faulted: bool = False, warning: bool = False,
                 fault_word: Optional[int] = None, fault_flag: bool = False,
                 params_supported: bool = False, run_mode: int = 0, can_timeout: int = 0):
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
        self.params_supported = params_supported   # old firmware: no parameter access, mode bits read 0
        self.run_mode = run_mode
        self.can_timeout = can_timeout
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
        return p.HOST_CAN_ID, feedback_bytes(self.motor_id, self.position, temp=self.temperature,
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
            if kind == "param_write":
                self.log.append(("write", idx))
                if idx == p.ParamIndex.CAN_TIMEOUT:
                    self.can_timeout = struct.unpack("<I", data[4:8])[0]
                return [(can_id, bytes(data))]
            value = {p.ParamIndex.RUN_MODE: self.run_mode, p.ParamIndex.CAN_TIMEOUT: self.can_timeout}.get(idx, 0)
            return [(can_id, bytes(data[:4]) + struct.pack("<I", value))]
        if kind == "status_query":
            self.log.append("status_query")
            return [(p.HOST_CAN_ID, bytes([self.motor_id]) + struct.pack("<I", self.fault_word))]
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
    m = motors.get(can_id & 0x7F)
    return m.handle(can_id, data) if m else []


class FakeBus(MockBackend):
    def __init__(self, motors):
        super().__init__()
        self.motors = {m.motor_id: m for m in motors}
        self.responder = lambda can_id, data: respond(self.motors, can_id, data)


def two_joint_config(**overrides) -> ArmConfig:
    cfg = dict(control_rate_hz=200, max_velocity=50.0, joints=[
        JointConfig(motor_id=1, name="j1", lo=-2.0, hi=2.0, kp=60, kd=2.0),
        JointConfig(motor_id=2, name="gripper", lo=-1.6, hi=-0.04, kp=20, kd=0.5, can_timeout_ms=500,
                    max_temperature_c=55, enable_temperature_c=45, return_to_rest=False,
                    release_on_fault=True),
    ])
    cfg.update(overrides)
    return ArmConfig(**cfg)


def make_arm(motors=None, **overrides):
    """(arm, bus, {motor_id: FakeMotor}) for two_joint_config(**overrides); default motors sit at
    0.3 and -0.5 rad."""
    motors = motors or [FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5)]
    bus = FakeBus(motors)
    arm = Arm(two_joint_config(**overrides), bus)
    return arm, bus, {m.motor_id: m for m in motors}
