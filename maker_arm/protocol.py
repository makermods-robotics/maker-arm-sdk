"""RobStride RS00 private CAN protocol: pure-function encode/decode, zero IO.

Byte-order convention: control frames/feedback frames are big-endian; param read/write
frames (Type 17/18) are little-endian.
τ_ff is encoded in Bit23~8 of the 29-bit ID, not in the 8-byte data field.
"""

import struct
from dataclasses import dataclass

# ── RS00 physical quantity <-> uint16 mapping ranges (unified across the whole arm) ──
P_MIN, P_MAX = -12.57, 12.57      # rad (encoding range, independent of joint limits)
V_MIN, V_MAX = -33.0, 33.0        # rad/s
T_MIN, T_MAX = -14.0, 14.0        # Nm
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0         # protocol hard ceiling
HOST_CAN_ID = 0xFD


@dataclass(frozen=True)
class MotorParams:
    """Per-model uint16 mapping ranges. The frame format is identical across models, only
    the ranges differ -- picking the wrong table shows up as torque/velocity readings
    scaled by the wrong factor (a pitfall the protocol docs explicitly warn about)."""
    t_min: float
    t_max: float
    v_min: float
    v_max: float
    p_min: float = P_MIN
    p_max: float = P_MAX


RS00 = MotorParams(t_min=T_MIN, t_max=T_MAX, v_min=V_MIN, v_max=V_MAX)
# RS02: ±17Nm/±44rad/s (rated 6Nm, 7.75:1 reduction ratio).
# ⚠️ Firmware ≤0.2.2.11 maps position to ±12.5 (here we use ±12.57 for newer firmware) -- check the firmware version during bring-up.
RS02 = MotorParams(t_min=-17.0, t_max=17.0, v_min=-44.0, v_max=44.0)
MOTOR_PARAMS = {"RS00": RS00, "RS02": RS02}

# ── communication type (29-bit ID Bit28~24) ──
COMM_GET_ID = 0
COMM_MIT = 1
COMM_FEEDBACK = 2
COMM_ENABLE = 3
COMM_DISABLE = 4
COMM_SET_ZERO = 6
COMM_SET_CAN_ID = 7
COMM_READ_PARAM = 17
COMM_WRITE_PARAM = 18
COMM_FAULT = 21
COMM_SAVE = 22
COMM_VERSION = 26


CAN_TIMEOUT_PER_MS = 20   # canTimeout unit is 50µs/count (protocol: 20000=1s) -- verified on real hardware


class ParamIndex:
    RUN_MODE = 0x7005      # u8: 0=control mode (MIT)
    LIMIT_TORQUE = 0x700B  # f32
    LOC_REF = 0x7016       # f32
    LIMIT_SPD = 0x7017     # f32
    MECH_POS = 0x7019      # f32
    VBUS = 0x701C          # f32
    CAN_TIMEOUT = 0x7028   # u32, unit is 50µs/count (20000=1s, verified on real hardware) -- convert with CAN_TIMEOUT_PER_MS when writing


def float_to_u16(x: float, lo: float, hi: float) -> int:
    x = min(hi, max(lo, x))
    return int(round((x - lo) * 65535.0 / (hi - lo)))


def u16_to_float(v: int, lo: float, hi: float) -> float:
    return v * (hi - lo) / 65535.0 + lo


def make_can_id(comm: int, data2: int, target: int) -> int:
    return ((comm & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target & 0xFF)


def encode_enable(motor_id: int, host_id: int = HOST_CAN_ID) -> tuple[int, bytes]:
    return make_can_id(COMM_ENABLE, host_id, motor_id), bytes(8)


def encode_disable(motor_id: int, clear_fault: bool = False,
                   host_id: int = HOST_CAN_ID) -> tuple[int, bytes]:
    data = bytearray(8)
    if clear_fault:
        data[0] = 1
    return make_can_id(COMM_DISABLE, host_id, motor_id), bytes(data)


def encode_mit(motor_id: int, pos: float, vel: float, kp: float, kd: float,
               tau: float, params: MotorParams = None) -> tuple[int, bytes]:
    pr = params or RS00
    data = struct.pack(">HHHH",
                       float_to_u16(pos, pr.p_min, pr.p_max),
                       float_to_u16(vel, pr.v_min, pr.v_max),
                       float_to_u16(kp, KP_MIN, KP_MAX),
                       float_to_u16(kd, KD_MIN, KD_MAX))
    tau_u16 = float_to_u16(tau, pr.t_min, pr.t_max)
    return make_can_id(COMM_MIT, tau_u16, motor_id), data


def encode_set_zero(motor_id: int, host_id: int = HOST_CAN_ID) -> tuple[int, bytes]:
    data = bytearray(8)
    data[0] = 1
    return make_can_id(COMM_SET_ZERO, host_id, motor_id), bytes(data)


def encode_read_param(motor_id: int, index: int,
                      host_id: int = HOST_CAN_ID) -> tuple[int, bytes]:
    return make_can_id(COMM_READ_PARAM, host_id, motor_id), struct.pack("<H6x", index)


def encode_write_param(motor_id: int, index: int, value, dtype: str = "f",
                       host_id: int = HOST_CAN_ID) -> tuple[int, bytes]:
    fmt = {"f": "<f", "u8": "<B3x", "u16": "<H2x", "u32": "<I"}[dtype]
    payload = struct.pack(fmt, value if dtype == "f" else int(value))
    return (make_can_id(COMM_WRITE_PARAM, host_id, motor_id),
            struct.pack("<H2x", index) + payload)


def encode_save_params(motor_id: int, host_id: int = HOST_CAN_ID) -> tuple[int, bytes]:
    return make_can_id(COMM_SAVE, host_id, motor_id), bytes(8)


@dataclass
class MotorFeedback:
    motor_id: int
    position: float      # rad, motor coordinates (direction/offset conversion happens at the arm layer)
    velocity: float      # rad/s
    torque: float        # Nm
    temperature: float   # °C
    mode: int            # 0=Reset 1=Cali 2=Motor
    fault_bits: int      # 6-bit fault code, nonzero means faulted


@dataclass
class ParamReply:
    motor_id: int
    index: int
    raw: bytes           # data[4:8], decoded per the param's actual type

    def value(self, dtype: str = "f"):
        fmt = {"f": "<f", "u8": "<B3x", "u16": "<H2x", "u32": "<I"}[dtype]
        return struct.unpack(fmt, self.raw)[0]


@dataclass
class FaultReport:
    motor_id: int
    raw: bytes


def parse_frame(can_id: int, data: bytes, params: MotorParams = None):
    pr = params or RS00
    comm = (can_id >> 24) & 0x1F
    if comm == COMM_FEEDBACK:
        pos_u, vel_u, tau_u, temp_u = struct.unpack(">HHHH", data[:8])
        return MotorFeedback(
            motor_id=(can_id >> 8) & 0xFF,
            position=u16_to_float(pos_u, pr.p_min, pr.p_max),
            velocity=u16_to_float(vel_u, pr.v_min, pr.v_max),
            torque=u16_to_float(tau_u, pr.t_min, pr.t_max),
            temperature=temp_u / 10.0,
            mode=(can_id >> 22) & 0x03,
            fault_bits=(can_id >> 16) & 0x3F,
        )
    if comm == COMM_READ_PARAM:
        index = struct.unpack("<H", data[:2])[0]
        return ParamReply(motor_id=(can_id >> 8) & 0xFF, index=index, raw=bytes(data[4:8]))
    if comm == COMM_FAULT:
        return FaultReport(motor_id=(can_id >> 8) & 0xFF, raw=bytes(data))
    return None


# ── protocol switching and MIT interop (minimal subset) ─────────────────────────────
# The motor's communication protocol is a persistent, mutually-exclusive mode (private=default/CANopen/MIT); a switch only takes effect after a power cycle.
COMM_SET_PROTOCOL = 25


def encode_set_protocol(motor_id: int, f_cmd: int,
                        host_id: int = HOST_CAN_ID) -> tuple[int, bytes]:
    """Private-protocol Type25: switch the motor's communication protocol (0=private 1=CANopen 2=MIT).

    The magic sequence 01..06 must be at byte0~5 (anchored by real-hardware testing on
    motor 7, 2026-08-07); F_CMD is at byte6. The reply is a Type0 device-ID frame. Takes
    effect after a power cycle.
    """
    return (make_can_id(COMM_SET_PROTOCOL, host_id, motor_id),
            bytes([1, 2, 3, 4, 5, 6, f_cmd & 0xFF, 0]))


def mit_switch_protocol_data(f_cmd: int) -> bytes:
    """MIT protocol command 8 (protocol switch) data field; the frame uses an 11-bit standard frame, arbitration_id = motor id."""
    return bytes([0xFF] * 6 + [f_cmd & 0xFF, 0xFD])


def mit_fault_query_data() -> bytes:
    """MIT protocol command 5 (F_CMD=0 read fault status, no side effects) -- used as an MIT-mode probe ping."""
    return bytes([0xFF] * 6 + [0x00, 0xFB])
