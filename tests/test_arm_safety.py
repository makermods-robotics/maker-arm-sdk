import math
import time

import pytest

from conftest import feedback_frame
from maker_arm.arm import Arm, ArmState
from maker_arm.transport.mock import MockBackend
from test_arm_connect import auto_feedback, two_joint_cfg


def make_connected_arm():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 0.0, 2: 0.0})
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    return arm, be


def test_estop_never_raises_even_when_backend_closed():
    arm, be = make_connected_arm()
    arm.disconnect()
    be.send = None  # 模拟后端彻底不可用
    arm.estop()     # 不得抛异常


def test_control_loop_exception_enters_fault():
    arm, be = make_connected_arm()
    arm.enable()    # 真线程
    def boom(cid, data):
        raise OSError(105, "No buffer space available")
    be.send = boom
    deadline = time.monotonic() + 2.0
    while arm.state is not ArmState.FAULT and time.monotonic() < deadline:
        time.sleep(0.02)
    assert arm.state is ArmState.FAULT
    assert "控制循环异常" in arm.fault_reason


def test_nonfinite_targets_rejected():
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    assert arm.set_joint_targets([math.nan, 0.0]) is False
    assert arm.set_joint_targets([math.inf, 0.0]) is False
    assert arm.set_joint_targets([0.0, 0.0]) is True


def test_enable_verifies_run_mode():
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)   # auto_feedback 回 RUN_MODE=0 → 成功
    assert arm.state is ArmState.ENABLED


def test_enable_hard_fails_on_wrong_run_mode():
    import struct
    from maker_arm import protocol as p
    from maker_arm.errors import ConnectError
    cfg, be = two_joint_cfg(), MockBackend()
    base = auto_feedback({1: 0.0, 2: 0.0})

    def responder(cid, data):
        if (cid >> 24) & 0x1F == p.COMM_READ_PARAM:
            idx = int.from_bytes(data[:2], "little")
            mid = cid & 0xFF
            if idx == p.ParamIndex.RUN_MODE:
                reply_id = (p.COMM_READ_PARAM << 24) | (mid << 8) | p.HOST_CAN_ID
                return [(reply_id, struct.pack("<H2x", idx) + struct.pack("<B3x", 1))]  # RUN_MODE=1 ≠ 0
        return base(cid, data)
    be.responder = responder
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    with pytest.raises(ConnectError, match="运控模式未生效"):
        arm.enable(start_loop=False)
    assert arm.state is ArmState.CONNECTED


def test_bad_mode_feedback_enters_fault():
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    be.responder = None
    be.inject(*feedback_frame(1, mode=0))   # 电机掉出运控态
    arm._tick(dt=0.005)
    assert arm.state is ArmState.FAULT
    assert "模式异常" in arm.fault_reason
