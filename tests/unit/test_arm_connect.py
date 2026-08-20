import math
import struct

import pytest

from conftest import feedback_frame
from maker_arm import protocol as p
from maker_arm.arm import Arm, ArmState
from maker_arm.config import ArmConfig, JointConfig
from maker_arm.errors import ConnectError
from maker_arm.transport.mock import MockBackend


def two_joint_cfg():
    return ArmConfig(joints=[
        JointConfig(motor_id=1, direction=1, offset=0.0, lo=-3.0, hi=3.0, kp=60, kd=2.0),
        JointConfig(motor_id=2, direction=-1, offset=0.5, lo=-3.0, hi=3.0, kp=60, kd=2.0),
    ])


def auto_feedback(positions):
    """Mock responder: any command frame gets back one current-position feedback frame
    (simulating a real motor); COMM_READ_PARAM additionally gets back one param read-back
    frame (RUN_MODE=0, CAN_TIMEOUT=4000 counts, everything else 0.0)."""
    def responder(cid, data):
        mid = cid & 0xFF
        if mid not in positions:
            return None
        comm = (cid >> 24) & 0x1F
        if comm == p.COMM_READ_PARAM:
            idx = struct.unpack("<H", data[:2])[0]
            if idx == p.ParamIndex.RUN_MODE:
                value = struct.pack("<B3x", 0)
            elif idx == p.ParamIndex.CAN_TIMEOUT:
                value = struct.pack("<I", 4000)   # 200ms × 20 counts/ms
            else:
                value = struct.pack("<f", 0.0)
            reply_cid = (p.COMM_READ_PARAM << 24) | (mid << 8) | p.HOST_CAN_ID
            return [(reply_cid, struct.pack("<H2x", idx) + value)]
        return [feedback_frame(mid, pos=positions[mid])]
    return responder


def test_connect_success_and_getters():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 1.0, 2: 0.5})
    arm = Arm(cfg, be)
    assert arm.state is ArmState.DISCONNECTED
    arm.connect(timeout=1.0)
    assert arm.state is ArmState.CONNECTED
    # joint coordinate conversion: joint = (motor - offset) * direction
    pos = arm.get_joint_positions()
    assert pos[0] == pytest.approx(1.0, abs=1e-3)
    assert pos[1] == pytest.approx((0.5 - 0.5) * -1, abs=1e-3)
    assert arm.get_faults() == [0, 0]
    arm.disconnect()
    assert arm.state is ArmState.DISCONNECTED


def test_connect_reports_missing_motor():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 0.0})   # motor 2 is not online
    arm = Arm(cfg, be)
    with pytest.raises(ConnectError, match="2"):
        arm.connect(timeout=0.3)
    assert arm.state is ArmState.DISCONNECTED


def test_refresh_probes_all():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 0.0, 2: 0.0})
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    n = len(be.sent)
    arm.refresh()
    assert len(be.sent) == n + 2   # one probe (stop frame) per joint


def test_waiting_refresh_reports_only_fresh_replies():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 0.0, 2: 0.0})
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    be.responder = auto_feedback({1: 0.1})
    assert arm.refresh(wait=True, timeout=0.005) == [True, False]
    assert arm.get_joint_positions()[0] == pytest.approx(0.1, abs=1e-3)
