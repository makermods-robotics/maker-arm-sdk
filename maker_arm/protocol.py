"""RobStride MIT wire protocol (standard 11-bit CAN, 1 Mbps): pure encode/decode, zero IO.

This is the only protocol the SDK speaks, matching MakerModsLab and its LeRobot RobStride
driver. Every frame is a classic standard-ID frame addressed to the motor ID (1..127).
Replies arrive on the host ID (0xFD) with the motor ID in payload byte 0; typed parameter
replies arrive on 0x300|id (read) or 0x400|id (write).

Sources: RobStride RS00/RS02 manuals 260713 section 6; LeRobot RobStride driver
(makermods-robotics/lerobot@5ce2fe41a); MakerModsLab maker_fault_diagnostics.py and
maker_gripper_hold.py.
"""

import math
import struct
from dataclasses import dataclass

HOST_CAN_ID = 0xFD
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0


@dataclass(frozen=True)
class MotorParams:
    """Per-model MIT mapping ranges. The frame format is identical across models; picking the
    wrong table scales torque/velocity readings by the wrong factor."""
    p_max: float   # rad
    v_max: float   # rad/s
    t_max: float   # Nm

    @property
    def p_min(self) -> float:
        return -self.p_max

    @property
    def v_min(self) -> float:
        return -self.v_max

    @property
    def t_min(self) -> float:
        return -self.t_max


# LeRobot type O0 = RS00, O1 = RS02. Not O2 (±33 rad/s, ±20 Nm), which is a different tier.
# RS02 firmware <=0.2.2.11 maps position to ±12.5 rad; check firmware during bring-up.
RS00 = MotorParams(p_max=12.57, v_max=33.0, t_max=14.0)
RS02 = MotorParams(p_max=12.57, v_max=44.0, t_max=17.0)
MOTOR_PARAMS = {"RS00": RS00, "RS02": RS02}


class ParamIndex:
    RUN_MODE = 0x7005      # u8: 0 = MIT control mode
    MECH_POS = 0x7019      # f32, rad
    CAN_TIMEOUT = 0x7028   # u32, 20 ticks per ms (10000 = 500 ms); 0 disables the watchdog


CAN_TIMEOUT_TICKS_PER_MS = 20

# Command opcodes (payload byte 7 after six 0xFF bytes).
OP_CLEAR_FAULT = 0xFB    # byte 6 = 0xFF clears faults and returns feedback; 0x00 reads faults only
OP_ENABLE = 0xFC
OP_DISABLE = 0xFD
OP_SET_ZERO = 0xFE

# MIT fault word bits (LE uint32 in the fault-status reply), per MakerModsLab.
FAULT_BITS = {
    0: "motor overtemperature",
    1: "driver chip fault",
    2: "undervoltage",
    3: "overvoltage",
    4: "phase B overcurrent",
    5: "phase C overcurrent",
    7: "encoder uncalibrated",
    8: "hardware identification fault",
    9: "position initialization fault",
    14: "stall/overload",
    16: "phase A overcurrent",
}

MODE_RESET = 0       # disabled
MODE_CALIBRATION = 1
MODE_RUN = 2         # enabled


def fault_text(word: int | None) -> str:
    if word is None:
        return "unknown"
    if not word:
        return "no fault"
    return ", ".join(FAULT_BITS.get(bit, f"bit{bit}") for bit in range(32) if word & (1 << bit))


def _id(motor_id: int) -> int:
    if not 1 <= motor_id <= 127:
        raise ValueError(f"MIT motor ID must be in [1, 127], got {motor_id}")
    return motor_id


def command(motor_id: int, opcode: int, argument: int = 0xFF) -> tuple[int, bytes]:
    return _id(motor_id), bytes([0xFF] * 6 + [argument & 0xFF, opcode])


def encode_enable(motor_id: int) -> tuple[int, bytes]:
    return command(motor_id, OP_ENABLE)


def encode_disable(motor_id: int) -> tuple[int, bytes]:
    return command(motor_id, OP_DISABLE)


def encode_clear_fault(motor_id: int) -> tuple[int, bytes]:
    """Clear faults and request feedback. Only send when no fault evidence must be kept."""
    return command(motor_id, OP_CLEAR_FAULT)


def encode_fault_query(motor_id: int) -> tuple[int, bytes]:
    """Read the fault word without clearing it or changing torque state."""
    return command(motor_id, OP_CLEAR_FAULT, 0x00)


def encode_set_zero(motor_id: int) -> tuple[int, bytes]:
    return command(motor_id, OP_SET_ZERO)


def float_to_uint(value: float, lo: float, hi: float, bits: int) -> int:
    if not math.isfinite(value):
        raise ValueError("MIT commands must be finite")
    return round((min(hi, max(lo, value)) - lo) * ((1 << bits) - 1) / (hi - lo))


def uint_to_float(value: int, lo: float, hi: float, bits: int) -> float:
    return value * (hi - lo) / ((1 << bits) - 1) + lo


def encode_control(motor_id: int, pos: float, vel: float, kp: float, kd: float, tau: float,
                   params: MotorParams = RS00) -> tuple[int, bytes]:
    """MIT impedance command: tau_out = kp*(pos - q) + kd*(vel - dq) + tau."""
    q = float_to_uint(pos, params.p_min, params.p_max, 16)
    v = float_to_uint(vel, params.v_min, params.v_max, 12)
    k = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    d = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t = float_to_uint(tau, params.t_min, params.t_max, 12)
    payload = bytes([q >> 8, q & 0xFF, v >> 4,
                     ((v & 0xF) << 4) | (k >> 8), k & 0xFF, d >> 4,
                     ((d & 0xF) << 4) | (t >> 8), t & 0xFF])
    if payload[:6] == bytes([0xFF] * 6):
        raise ValueError("MIT control payload collides with the reserved command prefix")
    return _id(motor_id), payload


def encode_read_param(motor_id: int, index: int) -> tuple[int, bytes]:
    return 0x300 | _id(motor_id), struct.pack("<H6x", index)


_PARAM_FORMATS = {"f": "<f", "u8": "<B3x", "u16": "<H2x", "u32": "<I"}


def encode_write_param(motor_id: int, index: int, value, dtype: str = "f") -> tuple[int, bytes]:
    payload = struct.pack(_PARAM_FORMATS[dtype], value if dtype == "f" else int(value))
    return 0x400 | _id(motor_id), struct.pack("<H2x", index) + payload


@dataclass(frozen=True)
class MotorFeedback:
    motor_id: int
    position: float      # rad, motor coordinates
    velocity: float      # rad/s
    torque: float        # Nm
    temperature: float   # °C (winding)
    mode: int            # 0 reset/disabled, 1 calibration, 2 run/enabled
    fault_flag: bool
    warning_flag: bool


@dataclass(frozen=True)
class FaultStatus:
    motor_id: int
    word: int            # LE uint32 fault bits, see FAULT_BITS

    @property
    def has_fault(self) -> bool:
        return self.word != 0

    @property
    def text(self) -> str:
        return fault_text(self.word)


@dataclass(frozen=True)
class ParamReply:
    motor_id: int
    index: int
    raw: bytes           # data[4:8]
    write: bool = False  # True for a write acknowledgement (0x400|id), never a read result

    def value(self, dtype: str = "f"):
        return struct.unpack(_PARAM_FORMATS[dtype], self.raw)[0]


def reply_motor_id(can_id: int, data: bytes) -> int | None:
    """Motor ID of a reply without decoding it (parameter replies carry it in the CAN ID)."""
    if getattr(can_id, "extended", False) or not 0 <= can_id <= 0x7FF:
        return None
    if can_id & 0x700 in (0x300, 0x400):
        return can_id & 0xFF
    return data[0] if data else None


def parse_frame(can_id: int, data: bytes, params: MotorParams = RS00,
                host_id: int = HOST_CAN_ID):
    """Decode one received frame; anything that is not an MIT reply returns None.

    Fault-status replies are five bytes on legacy firmware and zero-padded to eight on newer
    firmware. A motion sample with the same padding signature (temperature 0.0 °C, mode 0) is
    conservatively treated as status so a late fault reply can never become an encoder sample.
    """
    if getattr(can_id, "extended", False) or not 0 <= can_id <= 0x7FF:
        return None
    if can_id & 0x700 in (0x300, 0x400):
        # Parameter replies: the register index low byte sits in data[0], which can alias a
        # motor ID, so these are filtered by arbitration ID before any feedback decoding.
        if len(data) == 8:
            return ParamReply(can_id & 0xFF, struct.unpack("<H", data[:2])[0], bytes(data[4:8]),
                              write=can_id & 0x700 == 0x400)
        return None
    if can_id != host_id or len(data) not in (5, 8):
        return None
    if len(data) == 5 or data[5:8] == bytes(3):
        return FaultStatus(data[0], struct.unpack("<I", bytes(data[1:5]))[0])
    q = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0xF) << 8) | data[5]
    # Byte 6 carries mode (bits 7-6) and fault/warning flags (bits 5-4) above the 12-bit
    # temperature. Legacy firmware leaves those bits zero for any real temperature, so masking
    # is correct for both layouts.
    status = data[6]
    return MotorFeedback(
        motor_id=data[0],
        position=uint_to_float(q, params.p_min, params.p_max, 16),
        velocity=uint_to_float(v, params.v_min, params.v_max, 12),
        torque=uint_to_float(t, params.t_min, params.t_max, 12),
        temperature=(((status & 0x0F) << 8) | data[7]) / 10.0,
        mode=status >> 6,
        fault_flag=bool(status & 0x20),
        warning_flag=bool(status & 0x10),
    )
