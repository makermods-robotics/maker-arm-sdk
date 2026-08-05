"""RobStride RS00 私有 CAN 协议：纯函数编解码，零 IO。

字节序约定：运控帧/反馈帧大端；参数读写帧(Type 17/18)小端。
τ_ff 编码在 29 位 ID 的 Bit23~8，不在 8 字节数据域里。
"""

import struct
from dataclasses import dataclass

# ── RS00 物理量 ↔ uint16 映射范围（全臂统一） ──
P_MIN, P_MAX = -12.57, 12.57      # rad（编码范围，与关节限位无关）
V_MIN, V_MAX = -33.0, 33.0        # rad/s
T_MIN, T_MAX = -14.0, 14.0        # Nm
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0         # 协议硬上限
HOST_CAN_ID = 0xFD


@dataclass(frozen=True)
class MotorParams:
    """按型号的 uint16 映射范围。帧格式各型号相同，仅范围不同——选错表的症状是
    力矩/速度读数按错误比例缩放（协议文档明确警告过的坑）。"""
    t_min: float
    t_max: float
    v_min: float
    v_max: float
    p_min: float = P_MIN
    p_max: float = P_MAX


RS00 = MotorParams(t_min=T_MIN, t_max=T_MAX, v_min=V_MIN, v_max=V_MAX)
# RS02：±17Nm/±44rad/s（额定 6Nm，减速比 7.75:1）。
# ⚠️ 固件 ≤0.2.2.11 位置映射为 ±12.5（此处按新固件 ±12.57），bring-up 时查固件版本。
RS02 = MotorParams(t_min=-17.0, t_max=17.0, v_min=-44.0, v_max=44.0)
MOTOR_PARAMS = {"RS00": RS00, "RS02": RS02}

# ── 通信类型（29 位 ID 的 Bit28~24） ──
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


class ParamIndex:
    RUN_MODE = 0x7005      # u8: 0=运控(MIT)
    LIMIT_TORQUE = 0x700B  # f32
    LOC_REF = 0x7016       # f32
    LIMIT_SPD = 0x7017     # f32
    MECH_POS = 0x7019      # f32
    VBUS = 0x701C          # f32
    CAN_TIMEOUT = 0x7028   # u32，毫秒（真机 bring-up 时用 read_param 验证单位）


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
    position: float      # rad，电机坐标（方向/偏移换算在机械臂层）
    velocity: float      # rad/s
    torque: float        # Nm
    temperature: float   # °C
    mode: int            # 0=Reset 1=Cali 2=Motor
    fault_bits: int      # 6 位故障码，非 0 即故障


@dataclass
class ParamReply:
    motor_id: int
    index: int
    raw: bytes           # data[4:8]，按参数实际类型再解

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
