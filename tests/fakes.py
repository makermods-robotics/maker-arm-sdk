"""Simulated RobStride MIT motors behind MockBackend: enough behaviour to test the Arm pipeline."""

import struct
from dataclasses import dataclass, field

from maker_arm import protocol as p
from maker_arm.config import ArmConfig, JointConfig
from maker_arm.transport.mock import MockBackend

PREFIX = bytes([0xFF] * 6)


def feedback_bytes(motor_id, pos=0.0, vel=0.0, tau=0.0, temp=30.0, mode=0, fault=False, warn=False,
                   params=p.RS00):
    q = p.float_to_uint(pos, params.p_min, params.p_max, 16)
    v = p.float_to_uint(vel, params.v_min, params.v_max, 12)
    t = p.float_to_uint(tau, params.t_min, params.t_max, 12)
    temp10 = int(round(temp * 10))
    status = (mode << 6) | (int(fault) << 5) | (int(warn) << 4) | ((temp10 >> 8) & 0x0F)
    return bytes([motor_id, q >> 8, q & 0xFF, v >> 4, ((v & 0xF) << 4) | (t >> 8), t & 0xFF,
                  status, temp10 & 0xFF])


@dataclass
class FakeMotor:
    motor_id: int
    params: p.MotorParams = p.RS00
    position: float = 0.0
    temperature: float = 30.0
    fault_word: int = 0
    fault_flag: bool = False
    enabled: bool = False
    online: bool = True
    params_supported: bool = False   # legacy firmware: no parameter access, mode bits read 0
    run_mode: int = 0
    can_timeout: int = 0
    track: bool = True               # enabled motor jumps to the commanded position
    log: list = field(default_factory=list)
    last_kp: float = 0.0

    @property
    def mode(self) -> int:
        return (2 if self.enabled else 0) if self.params_supported else 0

    def feedback(self):
        return p.HOST_CAN_ID, feedback_bytes(self.motor_id, self.position, temp=self.temperature,
                                             mode=self.mode, fault=self.fault_flag, params=self.params)

    def handle(self, can_id, data):
        if not self.online:
            return []
        kind = can_id & 0x700
        if kind in (0x300, 0x400):
            if not self.params_supported:
                return []
            idx = struct.unpack("<H", data[:2])[0]
            if kind == 0x400:
                self.log.append(("write", idx))
                if idx == p.ParamIndex.CAN_TIMEOUT:
                    self.can_timeout = struct.unpack("<I", data[4:8])[0]
                return [(can_id, bytes(data))]
            value = {p.ParamIndex.RUN_MODE: self.run_mode, p.ParamIndex.CAN_TIMEOUT: self.can_timeout}.get(idx, 0)
            return [(can_id, bytes(data[:4]) + struct.pack("<I", value))]
        if data[:6] == PREFIX:
            arg, op = data[6], data[7]
            if op == p.OP_CLEAR_FAULT and arg == 0x00:
                self.log.append("fault_query")
                return [(p.HOST_CAN_ID, bytes([self.motor_id]) + struct.pack("<I", self.fault_word))]
            if op == p.OP_CLEAR_FAULT:
                self.log.append("clear")
                self.fault_word, self.fault_flag = 0, False
            elif op == p.OP_ENABLE:
                self.log.append("enable")
                self.enabled = True
            elif op == p.OP_DISABLE:
                self.log.append("disable")
                self.enabled = False
            elif op == p.OP_SET_ZERO:
                self.log.append("zero")
                self.position = 0.0
            return [self.feedback()]
        q = (data[0] << 8) | data[1]
        kp = ((data[3] & 0xF) << 8 | data[4]) * p.KP_MAX / 4095
        self.last_kp = kp
        self.log.append(("cmd", round(kp, 1)))
        if self.enabled and self.track and kp > 0:
            self.position = p.uint_to_float(q, self.params.p_min, self.params.p_max, 16)
        return [self.feedback()]


class FakeBus(MockBackend):
    def __init__(self, motors):
        super().__init__()
        self.motors = {m.motor_id: m for m in motors}
        self.responder = self._respond

    def _respond(self, can_id, data):
        m = self.motors.get(can_id & 0x7F)
        return m.handle(can_id, data) if m else []


def two_joint_config(**overrides) -> ArmConfig:
    cfg = dict(control_rate_hz=200, max_velocity=50.0, joints=[
        JointConfig(motor_id=1, name="j1", lo=-2.0, hi=2.0, kp=60, kd=2.0),
        JointConfig(motor_id=2, name="gripper", lo=-1.6, hi=-0.04, kp=20, kd=0.5, can_timeout_ms=500,
                    max_temperature_c=55, enable_temperature_c=45, return_to_rest=False,
                    release_on_fault=True),
    ])
    cfg.update(overrides)
    return ArmConfig(**cfg)
