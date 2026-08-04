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
