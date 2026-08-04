import math
import struct

import pytest

from conftest import feedback_frame
from maker_arm import protocol as p
from maker_arm.errors import ParamTimeout
from maker_arm.motor import Motor
from maker_arm.transport.mock import MockBackend


def make_motor(mid=1):
    be = MockBackend()
    m = Motor(mid, be)
    be.set_recv_callback(lambda cid, d: (msg := p.parse_frame(cid, d)) and m.handle_frame(msg))
    be.open()
    return m, be


def test_commands_produce_expected_frames():
    m, be = make_motor(1)
    m.enable(); m.disable(); m.probe(); m.set_zero()
    m.send_mit(0.0, 0.0, 10.0, 1.0, 0.0)
    cids = [cid for cid, _ in be.sent]
    assert cids == [0x0300FD01, 0x0400FD01, 0x0400FD01, 0x0600FD01, 0x01800001]


def test_feedback_cache_and_age():
    m, be = make_motor(2)
    assert m.feedback is None and m.feedback_age == math.inf
    be.inject(*feedback_frame(2, pos=1.0, temp=42.0))
    assert m.feedback.position == pytest.approx(1.0, abs=1e-3)
    assert m.feedback.temperature == pytest.approx(42.0)
    assert m.feedback_age < 0.1


def test_feedback_ignores_other_motor():
    m, be = make_motor(2)
    be.inject(*feedback_frame(2, pos=1.0))
    m2_only = m.feedback.position
    # 其它电机的帧由 Arm 分发，Motor.handle_frame 假定只收到自己的消息——
    # 这里验证 handle_frame 对 motor_id 不匹配的消息直接忽略（防御）
    m.handle_frame(p.parse_frame(*feedback_frame(3, pos=2.0)))
    assert m.feedback.position == m2_only


def test_read_param_sync_roundtrip():
    m, be = make_motor(1)

    def responder(cid, data):
        if (cid >> 24) & 0x1F == p.COMM_READ_PARAM:
            idx = struct.unpack("<H", data[:2])[0]
            reply_id = (p.COMM_READ_PARAM << 24) | (1 << 8) | p.HOST_CAN_ID
            return [(reply_id, struct.pack("<H2x", idx) + struct.pack("<f", 24.5))]
    be.responder = responder
    assert m.read_param(p.ParamIndex.VBUS) == pytest.approx(24.5)


def test_read_param_timeout():
    m, be = make_motor(1)
    with pytest.raises(ParamTimeout):
        m.read_param(p.ParamIndex.VBUS, timeout=0.05)


def test_read_param_ignores_mismatched_reply():
    m, be = make_motor(1)

    def stale_responder(cid, data):
        if (cid >> 24) & 0x1F == p.COMM_READ_PARAM:
            reply_id = (p.COMM_READ_PARAM << 24) | (1 << 8) | p.HOST_CAN_ID
            # 回一个 index 不匹配的迟到回包：必须被忽略 → 超时
            return [(reply_id, struct.pack("<H2x", 0x9999) + struct.pack("<f", 1.0))]
    be.responder = stale_responder
    with pytest.raises(ParamTimeout):
        m.read_param(p.ParamIndex.VBUS, timeout=0.05)
