import time

import pytest

from conftest import feedback_frame
from maker_arm.arm import Arm, ArmState
from maker_arm.transport.mock import MockBackend
from test_arm_connect import auto_feedback, two_joint_cfg


def make_arm(responder):
    cfg, be = two_joint_cfg(), MockBackend()
    cfg.feedback_timeout = 0.1
    be.responder = responder
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    arm.enable(start_loop=False)
    return arm, be


def test_feedback_timeout_enters_fault():
    responder = auto_feedback({1: 0.0, 2: 0.0})
    arm, be = make_arm(responder)
    be.responder = None            # 模拟通信丢失：不再回反馈
    time.sleep(0.15)               # 超过 feedback_timeout=0.1
    arm._tick(dt=0.005)
    assert arm.state is ArmState.FAULT
    assert "超时" in arm.fault_reason


def test_motor_fault_bits_enter_fault_with_name():
    arm, be = make_arm(auto_feedback({1: 0.0, 2: 0.0}))
    be.responder = None            # 模拟通信丢失（防止 responder 覆盖注入的故障反馈）
    be.inject(*feedback_frame(2, fault=0b000100))   # bit2=过温
    arm._tick(dt=0.005)
    assert arm.state is ArmState.FAULT
    assert "电机 2" in arm.fault_reason and "过温" in arm.fault_reason


def test_clear_faults_returns_to_connected():
    arm, be = make_arm(auto_feedback({1: 0.0, 2: 0.0}))
    be.responder = None            # 模拟通信丢失（防止 responder 覆盖注入的故障反馈）
    be.inject(*feedback_frame(1, fault=0b000001))
    arm._tick(dt=0.005)
    assert arm.state is ArmState.FAULT
    arm.clear_faults()
    assert arm.state is ArmState.CONNECTED
    # clear_faults 发的是 data[0]=1 的停止帧
    assert any(d[0] == 1 for cid, d in be.sent if (cid >> 24) & 0x1F == 4)


def test_fault_disables_motors():
    arm, be = make_arm(auto_feedback({1: 0.0, 2: 0.0}))
    n_dis_before = sum(1 for cid, _ in be.sent if (cid >> 24) & 0x1F == 4)
    be.responder = None            # 模拟通信丢失（防止 responder 覆盖注入的故障反馈）
    be.inject(*feedback_frame(1, fault=0b000001))
    arm._tick(dt=0.005)
    n_dis_after = sum(1 for cid, _ in be.sent if (cid >> 24) & 0x1F == 4)
    assert n_dis_after >= n_dis_before + 2
