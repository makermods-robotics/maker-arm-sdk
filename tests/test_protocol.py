import struct

import pytest

from maker_arm import protocol as p


def test_constants():
    assert p.P_MAX == 12.57 and p.P_MIN == -12.57
    assert p.V_MAX == 33.0 and p.T_MAX == 14.0
    assert p.KP_MAX == 500.0 and p.KD_MAX == 5.0
    assert p.HOST_CAN_ID == 0xFD


def test_u16_roundtrip_and_bounds():
    assert p.float_to_u16(p.P_MIN, p.P_MIN, p.P_MAX) == 0
    assert p.float_to_u16(p.P_MAX, p.P_MIN, p.P_MAX) == 65535
    assert p.float_to_u16(0.0, p.P_MIN, p.P_MAX) == 32768
    assert p.float_to_u16(99.0, p.P_MIN, p.P_MAX) == 65535  # 超界钳位
    assert p.float_to_u16(-99.0, p.P_MIN, p.P_MAX) == 0
    assert p.u16_to_float(32768, p.P_MIN, p.P_MAX) == pytest.approx(0.0, abs=1e-3)
    x = 1.234
    back = p.u16_to_float(p.float_to_u16(x, p.P_MIN, p.P_MAX), p.P_MIN, p.P_MAX)
    assert back == pytest.approx(x, abs=25.14 / 65535)


def test_make_can_id():
    # 使能帧：comm=3, data2=主机ID 0xFD, target=电机1
    assert p.make_can_id(p.COMM_ENABLE, p.HOST_CAN_ID, 1) == 0x0300FD01


def test_encode_enable_disable_zero():
    cid, data = p.encode_enable(1)
    assert cid == 0x0300FD01 and data == bytes(8)
    cid, data = p.encode_disable(2)
    assert cid == 0x0400FD02 and data == bytes(8)
    cid, data = p.encode_disable(2, clear_fault=True)
    assert data[0] == 1
    cid, data = p.encode_set_zero(3)
    assert cid == 0x0600FD03 and data[0] == 1


def test_encode_mit_golden():
    # 全零指令 + kp=10, kd=1.0：手算黄金样例
    cid, data = p.encode_mit(1, 0.0, 0.0, 10.0, 1.0, 0.0)
    assert cid == 0x01800001            # comm=1, τ_ff=0 → 0x8000 在 Bit23~8
    assert data == bytes.fromhex("80008000051F3333")  # 大端 pos vel kp kd


def test_encode_params_little_endian():
    cid, data = p.encode_read_param(1, p.ParamIndex.VBUS)
    assert cid == 0x1100FD01
    assert data == bytes.fromhex("1C70000000000000")  # 索引小端
    cid, data = p.encode_write_param(1, p.ParamIndex.CAN_TIMEOUT, 200, dtype="u32")
    assert cid == 0x1200FD01
    assert data == bytes.fromhex("2870" + "0000" + "C8000000")  # 值小端 u32
    cid, data = p.encode_write_param(1, p.ParamIndex.LIMIT_SPD, 2.0, dtype="f")
    assert data[:2] == bytes.fromhex("1770") and data[4:8] == struct.pack("<f", 2.0)
    cid, data = p.encode_save_params(1)
    assert cid == 0x1600FD01


def test_parse_feedback_golden():
    # comm=2, mode=2(Motor), fault=0, motor=1, target=host
    cid = (2 << 24) | (2 << 22) | (0 << 16) | (1 << 8) | 0xFD
    assert cid == 0x028001FD
    data = bytes.fromhex("800080008000" + "0159")  # pos/vel/tau≈0, temp=34.5°C
    fb = p.parse_frame(cid, data)
    assert isinstance(fb, p.MotorFeedback)
    assert fb.motor_id == 1 and fb.mode == 2 and fb.fault_bits == 0
    assert fb.position == pytest.approx(0.0, abs=1e-3)
    assert fb.velocity == pytest.approx(0.0, abs=1e-3)
    assert fb.torque == pytest.approx(0.0, abs=1e-3)
    assert fb.temperature == pytest.approx(34.5)


def test_parse_feedback_fault_bits():
    cid = (2 << 24) | (2 << 22) | (0x21 << 16) | (3 << 8) | 0xFD
    fb = p.parse_frame(cid, bytes(8))
    assert fb.motor_id == 3 and fb.fault_bits == 0x21


def test_parse_param_reply():
    # 电机 2 回读 VBUS=24.5：comm=17，ID Bit8~15=电机ID，目标=主机
    cid = (17 << 24) | (2 << 8) | 0xFD
    data = struct.pack("<H2x", p.ParamIndex.VBUS) + struct.pack("<f", 24.5)
    r = p.parse_frame(cid, data)
    assert isinstance(r, p.ParamReply)
    assert r.motor_id == 2 and r.index == p.ParamIndex.VBUS
    assert r.value("f") == pytest.approx(24.5)
    r2 = p.parse_frame(cid, struct.pack("<H2x", 0x7028) + struct.pack("<I", 200))
    assert r2.value("u32") == 200


def test_parse_unknown_returns_none():
    assert p.parse_frame((22 << 24) | 0xFD, bytes(8)) is None
