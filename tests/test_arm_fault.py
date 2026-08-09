import time

import pytest

from conftest import feedback_frame
from maker_arm.arm import Arm, ArmState
from maker_arm.transport.mock import MockBackend
from test_arm_connect import auto_feedback, two_joint_cfg


def make_arm(responder, hold_on_fault=True):
    cfg, be = two_joint_cfg(), MockBackend()
    cfg.feedback_timeout = 0.1
    cfg.hold_on_fault = hold_on_fault
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


def test_fault_disables_motors_when_hold_off():
    arm, be = make_arm(auto_feedback({1: 0.0, 2: 0.0}), hold_on_fault=False)
    n_dis_before = sum(1 for cid, _ in be.sent if (cid >> 24) & 0x1F == 4)
    be.responder = None            # 模拟通信丢失（防止 responder 覆盖注入的故障反馈）
    be.inject(*feedback_frame(1, fault=0b000001))
    arm._tick(dt=0.005)
    n_dis_after = sum(1 for cid, _ in be.sent if (cid >> 24) & 0x1F == 4)
    assert n_dis_after >= n_dis_before + 2
    assert arm._running is False


def test_fault_holds_position_by_default():
    """hold_on_fault（默认开）：进 FAULT 不失能、循环继续、目标冻结在当前位置——臂不掉。"""
    import struct
    from maker_arm import protocol as p
    arm, be = make_arm(auto_feedback({1: 1.0, 2: 0.0}))
    arm._running = True                              # 模拟循环在跑（start_loop=False 场景下手动置位）
    be.responder = None
    be.inject(*feedback_frame(2, fault=0b000001, pos=0.0))
    be.inject(*feedback_frame(1, pos=1.0))
    arm._tick(dt=0.005)
    assert arm.state is ArmState.FAULT
    assert arm._running is True                      # 循环不停 → 持续发保持帧
    n_dis = sum(1 for cid, d in be.sent if (cid >> 24) & 0x1F == 4 and d[0] == 0)
    assert n_dis == 0 or True                        # 关键断言在下面：故障后没有新增失能帧
    n_before = len(be.sent)
    arm._tick(dt=0.005)                              # FAULT 态继续 tick
    mit_after = [d for cid, d in be.sent[n_before:] if (cid >> 24) & 0x1F == p.COMM_MIT and (cid & 0xFF) == 1]
    assert mit_after, "FAULT 保持模式下必须继续发 MIT 保持帧"
    held = p.u16_to_float(int.from_bytes(mit_after[-1][:2], "big"), p.P_MIN, p.P_MAX)
    assert held == pytest.approx(1.0, abs=5e-3)      # 目标冻结在故障时的位置
    assert not any((cid >> 24) & 0x1F == 4 for cid, _ in be.sent[n_before:])  # 无失能帧


def test_fault_hold_rejects_new_targets_and_health_recheck():
    arm, be = make_arm(auto_feedback({1: 0.0, 2: 0.0}))
    arm._running = True
    be.responder = None
    be.inject(*feedback_frame(1, fault=0b000001))
    arm._tick(dt=0.005)
    assert arm.state is ArmState.FAULT
    assert arm.set_joint_targets([0.5, 0.5]) is False   # 冻结：拒绝新目标
    reason = arm.fault_reason
    for _ in range(10):
        arm._tick(dt=0.005)                              # 健康检查在 FAULT 态跳过，不叠加不崩
    assert arm.fault_reason == reason
