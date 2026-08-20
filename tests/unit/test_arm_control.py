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
    arm.enable(start_loop=False)   # the test drives _tick directly, no thread started
    return arm, be


def mit_frames(be):
    return [(cid, d) for cid, d in be.sent if (cid >> 24) & 0x1F == p.COMM_MIT]


def test_enable_writes_watchdog_and_inits_targets():
    arm, be = make_enabled_arm()
    assert arm.state is ArmState.ENABLED
    # RUN_MODE and CAN_TIMEOUT were written (Type 18, little-endian)
    wp = [(cid, d) for cid, d in be.sent if (cid >> 24) & 0x1F == p.COMM_WRITE_PARAM]
    indices = {int.from_bytes(d[:2], "little") for cid, d in wp}
    assert p.ParamIndex.RUN_MODE in indices and p.ParamIndex.CAN_TIMEOUT in indices
    # first-frame protection: target = current position (motor 1 is at 1.0 rad)
    assert arm._user_targets[0] == pytest.approx(1.0, abs=1e-3)
    assert arm._internal[0] == pytest.approx(1.0, abs=1e-3)


def test_tick_rate_limits_toward_target():
    arm, be = make_enabled_arm()
    assert arm.set_joint_targets([2.0, 0.0])
    arm._tick(dt=0.005)             # max_velocity=1.0 -> at most 0.005 rad per tick
    assert arm._internal[0] == pytest.approx(1.005, abs=1e-3)
    f = mit_frames(be)[-2]          # this tick's MIT frame for joint 1
    pos = p.u16_to_float(int.from_bytes(f[1][:2], "big"), p.P_MIN, p.P_MAX)
    assert pos == pytest.approx(1.005, abs=1e-3)


def test_tick_clamps_to_soft_limits():
    arm, be = make_enabled_arm()
    arm.set_joint_targets([99.0, 0.0])   # far exceeds hi=3.0
    for _ in range(1000):
        arm._tick(dt=0.01)
    assert arm._internal[0] == pytest.approx(3.0 - arm.config.limit_margin, abs=1e-6)


def test_direction_offset_applied_in_mit():
    arm, be = make_enabled_arm()
    arm.set_joint_targets([1.0, 1.0])
    for _ in range(1000):
        arm._tick(dt=0.01)
    f = mit_frames(be)[-1]              # joint 2: direction=-1, offset=0.5
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
