import math
import struct

import pytest

from fakes import feedback_bytes
from maker_arm import protocol as p
from maker_arm.transport.base import CanId


@pytest.mark.parametrize("encode, payload", [
    (p.encode_enable, "ffffffffffffffFC"),
    (p.encode_disable, "ffffffffffffffFD"),
    (p.encode_clear_fault, "ffffffffffffffFB"),
    (p.encode_fault_query, "ffffffffffff00FB"),
    (p.encode_set_zero, "ffffffffffffffFE"),
])
def test_commands_are_standard_frames_to_the_motor_id(encode, payload):
    assert encode(7) == (7, bytes.fromhex(payload))


@pytest.mark.parametrize("bad", [0, 128])
def test_motor_id_range(bad):
    with pytest.raises(ValueError):
        p.encode_enable(bad)


def test_control_packing_matches_lerobot_layout():
    cid, data = p.encode_control(3, 0.0, 0.0, 500.0, 5.0, 0.0, p.RS02)
    assert cid == 3
    q = (data[0] << 8) | data[1]
    kp = ((data[3] & 0xF) << 8) | data[4]
    kd = (data[5] << 4) | (data[6] >> 4)
    assert q == round(12.57 * 65535 / 25.14)
    assert kp == 4095 and kd == 4095


def test_control_clamps_and_rejects_non_finite():
    _, hi = p.encode_control(1, 99.0, 0, 0, 0, 0)
    assert hi[:2] == b"\xff\xff"
    with pytest.raises(ValueError):
        p.encode_control(1, math.nan, 0, 0, 0, 0)


def test_feedback_decode_masks_status_bits_out_of_temperature():
    data = feedback_bytes(5, pos=1.0, temp=41.3, mode=2, fault=False, warn=True)
    fb = p.parse_frame(p.HOST_CAN_ID, data)
    assert fb.motor_id == 5
    assert fb.position == pytest.approx(1.0, abs=1e-3)
    assert fb.temperature == pytest.approx(41.3)
    assert fb.mode == 2 and fb.warning_flag and not fb.fault_flag


def test_legacy_feedback_reads_mode_zero_and_true_temperature():
    data = bytearray(feedback_bytes(1, temp=35.0))
    assert data[6] >> 4 == 0
    fb = p.parse_frame(p.HOST_CAN_ID, bytes(data))
    assert fb.temperature == 35.0 and fb.mode == 0


def test_model_scaling_differs():
    data = feedback_bytes(2, tau=10.0, params=p.RS02)
    assert p.parse_frame(p.HOST_CAN_ID, data, p.RS02).torque == pytest.approx(10.0, abs=0.01)
    assert p.parse_frame(p.HOST_CAN_ID, data, p.RS00).torque != pytest.approx(10.0, abs=0.5)


@pytest.mark.parametrize("data", [bytes([7, 1, 0, 0, 0]), bytes([7, 1, 0, 0, 0, 0, 0, 0])])
def test_fault_status_five_or_padded_eight_bytes(data):
    st = p.parse_frame(p.HOST_CAN_ID, data)
    assert isinstance(st, p.FaultStatus)
    assert st.motor_id == 7 and st.word == 1 and st.has_fault
    assert st.text == "motor overtemperature"


def test_fault_text_decodes_bits_and_reports_unknown_bits():
    assert p.fault_text(0) == "no fault"
    assert p.fault_text(None) == "unknown"
    assert p.fault_text((1 << 2) | (1 << 14)) == "undervoltage, stall/overload"
    assert p.fault_text(1 << 6) == "bit6"


def test_param_replies_are_filtered_by_arbitration_id():
    # data[0] is the register index low byte (0x28) and must not be mistaken for a motor ID.
    reply = struct.pack("<H2xI", p.ParamIndex.CAN_TIMEOUT, 10000)
    msg = p.parse_frame(0x300 | 7, reply)
    assert isinstance(msg, p.ParamReply)
    assert msg.motor_id == 7 and msg.value("u32") == 10000
    assert p.reply_motor_id(0x300 | 7, reply) == 7
    assert p.parse_frame(0x400 | 7, reply).motor_id == 7


def test_extended_and_foreign_frames_are_ignored():
    data = feedback_bytes(1)
    assert p.parse_frame(CanId(p.HOST_CAN_ID, extended=True), data) is None
    assert p.parse_frame(0x123, data) is None
    assert p.reply_motor_id(CanId(0x02FD01, extended=True), data) is None


def test_write_param_layout():
    cid, data = p.encode_write_param(7, p.ParamIndex.CAN_TIMEOUT, 10000, "u32")
    assert cid == 0x407
    assert data == struct.pack("<H2xI", 0x7028, 10000)
    assert p.encode_read_param(7, 0x7005) == (0x307, struct.pack("<H6x", 0x7005))
