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


def test_enable_writes_can_timeout_in_50us_counts():
    """协议单位：canTimeout 20000=1s（50µs/计数）。config 的 200ms 必须写成 4000。
    真机教训：直接写 200 = 10ms 阈值，USB 抖动即让电机自我泄力回 reset(mode=0)。"""
    import struct
    from maker_arm import protocol as p
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    writes = [(cid, d) for cid, d in be.sent if (cid >> 24) & 0x1F == p.COMM_WRITE_PARAM
              and int.from_bytes(d[:2], "little") == p.ParamIndex.CAN_TIMEOUT]
    assert writes, "必须写 CAN_TIMEOUT"
    for _, d in writes:
        assert struct.unpack("<I", d[4:8])[0] == 200 * p.CAN_TIMEOUT_PER_MS == 4000


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


def test_bad_mode_feedback_enters_fault_after_persistence():
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    be.responder = None
    be.inject(*feedback_frame(1, mode=0))   # 电机掉出运控态（持续）
    for _ in range(4):                      # 前 4 拍：容忍（可能是使能后的陈旧缓存）
        arm._tick(dt=0.005)
        assert arm.state is ArmState.ENABLED
    arm._tick(dt=0.005)                     # 第 5 拍仍异常 → 判真故障
    assert arm.state is ArmState.FAULT
    assert "模式异常" in arm.fault_reason


def test_transient_bad_mode_is_tolerated():
    """使能后前几拍收到探测时代的 mode=0 旧反馈（真机竞态）——不得误报 FAULT。"""
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    be.responder = None
    be.inject(*feedback_frame(1, mode=0))   # 陈旧缓存
    arm._tick(dt=0.005)
    arm._tick(dt=0.005)
    assert arm.state is ArmState.ENABLED
    be.inject(*feedback_frame(1, mode=2))   # 新鲜反馈到达
    for _ in range(10):
        arm._tick(dt=0.005)
    assert arm.state is ArmState.ENABLED    # 计数已复位，永不误报


def test_enable_refuses_unexplainable_out_of_limit_position():
    """守门：越限且 ±2π 都解释不了（真异常，非跳变）→ 拒绝使能，绝不默默钳位拖关节。"""
    from maker_arm import protocol as p
    from maker_arm.config import ArmConfig, JointConfig
    from maker_arm.errors import ConnectError
    cfg = ArmConfig(joints=[
        JointConfig(motor_id=1, direction=1, offset=0.0, lo=-1.0, hi=1.0, kp=30, kd=1.0),
        JointConfig(motor_id=2, direction=1, offset=0.0, lo=-1.0, hi=1.0, kp=30, kd=1.0),
    ])
    be = MockBackend()
    be.responder = auto_feedback({1: 2.0, 2: 0.0})   # 2.0 与 2.0±2π 都不在 [-1.35,1.35]
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    assert arm._wrap == [0.0, 0.0]                    # 无 2π 解释 → 不补偿
    with pytest.raises(ConnectError, match="拒绝使能"):
        arm.enable(start_loop=False)
    assert arm.state is ArmState.CONNECTED
    assert not any((cid >> 24) & 0x1F == p.COMM_ENABLE for cid, _ in be.sent)


def test_enable_allows_small_excursion_within_grace():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 3.2, 2: 0.0})    # 超限 0.2 < 宽限 0.35 → 允许
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    arm.enable(start_loop=False)
    assert arm.state is ArmState.ENABLED


def test_connect_compensates_2pi_wrap_transparently():
    """会话内 2π 补偿：重启跳变的读数在 connect 时自动修正，读数/指令双向透明。"""
    import math as _m
    from maker_arm import protocol as p
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 0.3 + 2 * _m.pi, 2: 0.0})   # 电机1 物理在 0.3，读数 +2π
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    pos = arm.get_joint_positions()
    assert pos[0] == pytest.approx(0.3, abs=2e-3)                 # 读数已补偿
    arm.enable(start_loop=False)                                  # 守门放行（补偿后在限位内）
    arm.set_joint_targets([0.5, 0.0])
    for _ in range(200):
        arm._tick(dt=0.01)
    mit = [(cid, d) for cid, d in be.sent if (cid >> 24) & 0x1F == p.COMM_MIT and (cid & 0xFF) == 1]
    sent_pos = p.u16_to_float(int.from_bytes(mit[-1][1][:2], "big"), p.P_MIN, p.P_MAX)
    assert sent_pos == pytest.approx(0.5 + 2 * _m.pi, abs=5e-3)   # 指令自动加回 +2π（电机坐标）


def test_connect_no_compensation_for_sane_readings():
    arm, be = make_connected_arm()
    assert arm._wrap == [0.0, 0.0]
