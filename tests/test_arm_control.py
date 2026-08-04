import pytest

from conftest import feedback_frame
from maker_arm import protocol as p
from maker_arm.arm import Arm, ArmState
from maker_arm.transport.mock import MockBackend
from test_arm_connect import auto_feedback, two_joint_cfg


def make_enabled_arm():
    cfg, be = two_joint_cfg(), MockBackend()
    cfg.max_velocity = 1.0
    be.responder = auto_feedback({1: 1.0, 2: 0.0})
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    arm.enable(start_loop=False)   # 测试直接驱动 _tick，不起线程
    return arm, be


def mit_frames(be):
    return [(cid, d) for cid, d in be.sent if (cid >> 24) & 0x1F == p.COMM_MIT]


def test_enable_writes_watchdog_and_inits_targets():
    arm, be = make_enabled_arm()
    assert arm.state is ArmState.ENABLED
    # 写了 RUN_MODE 和 CAN_TIMEOUT（Type 18 小端）
    wp = [(cid, d) for cid, d in be.sent if (cid >> 24) & 0x1F == p.COMM_WRITE_PARAM]
    indices = {int.from_bytes(d[:2], "little") for cid, d in wp}
    assert p.ParamIndex.RUN_MODE in indices and p.ParamIndex.CAN_TIMEOUT in indices
    # 首帧保护：目标 = 当前位置（电机1 在 1.0 rad）
    assert arm._user_targets[0] == pytest.approx(1.0, abs=1e-3)
    assert arm._internal[0] == pytest.approx(1.0, abs=1e-3)


def test_tick_rate_limits_toward_target():
    arm, be = make_enabled_arm()
    assert arm.set_joint_targets([2.0, 0.0])
    arm._tick(dt=0.005)             # max_velocity=1.0 → 每 tick 最多 0.005 rad
    assert arm._internal[0] == pytest.approx(1.005, abs=1e-3)
    f = mit_frames(be)[-2]          # 本 tick 关节1 的 MIT 帧
    pos = p.u16_to_float(int.from_bytes(f[1][:2], "big"), p.P_MIN, p.P_MAX)
    assert pos == pytest.approx(1.005, abs=1e-3)


def test_tick_clamps_to_soft_limits():
    arm, be = make_enabled_arm()
    arm.set_joint_targets([99.0, 0.0])   # 远超 hi=3.0
    for _ in range(1000):
        arm._tick(dt=0.01)
    assert arm._internal[0] == pytest.approx(3.0 - arm.config.limit_margin, abs=1e-6)


def test_direction_offset_applied_in_mit():
    arm, be = make_enabled_arm()
    arm.set_joint_targets([1.0, 1.0])
    for _ in range(1000):
        arm._tick(dt=0.01)
    f = mit_frames(be)[-1]              # 关节2: direction=-1, offset=0.5
    pos = p.u16_to_float(int.from_bytes(f[1][:2], "big"), p.P_MIN, p.P_MAX)
    assert pos == pytest.approx(1.0 * -1 + 0.5, abs=1e-3)


def test_set_targets_rejected_when_not_enabled():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 0.0, 2: 0.0})
    arm = Arm(cfg, be)
    assert arm.set_joint_targets([0.0, 0.0]) is False


def test_disable_and_estop():
    arm, be = make_enabled_arm()
    arm.disable()
    assert arm.state is ArmState.CONNECTED
    dis = [cid for cid, _ in be.sent if (cid >> 24) & 0x1F == p.COMM_DISABLE]
    assert len(dis) >= 2
