import math

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
    """Mock 应答器：任何指令帧都回一帧当前位置反馈（模拟真电机）。"""
    def responder(cid, data):
        mid = cid & 0xFF
        if mid in positions:
            return [feedback_frame(mid, pos=positions[mid])]
    return responder


def test_connect_success_and_getters():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 1.0, 2: 0.5})
    arm = Arm(cfg, be)
    assert arm.state is ArmState.DISCONNECTED
    arm.connect(timeout=1.0)
    assert arm.state is ArmState.CONNECTED
    # 关节坐标换算：joint = (motor - offset) * direction
    pos = arm.get_joint_positions()
    assert pos[0] == pytest.approx(1.0, abs=1e-3)
    assert pos[1] == pytest.approx((0.5 - 0.5) * -1, abs=1e-3)
    assert arm.get_faults() == [0, 0]
    arm.disconnect()
    assert arm.state is ArmState.DISCONNECTED


def test_connect_reports_missing_motor():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 0.0})   # 电机 2 不在线
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
    assert len(be.sent) == n + 2   # 每关节一帧 probe(停止帧)
