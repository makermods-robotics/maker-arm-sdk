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
