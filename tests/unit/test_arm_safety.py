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
    be.send = None  # simulate a completely unavailable backend
    arm.estop()     # must not raise


def test_control_loop_exception_enters_fault():
    arm, be = make_connected_arm()
    arm.enable()    # a real thread
    def boom(cid, data):
        raise OSError(105, "No buffer space available")
    be.send = boom
    deadline = time.monotonic() + 2.0
    while arm.state is not ArmState.FAULT and time.monotonic() < deadline:
        time.sleep(0.02)
    assert arm.state is ArmState.FAULT
    assert "control loop exception" in arm.fault_reason


def test_nonfinite_targets_rejected():
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    assert arm.set_joint_targets([math.nan, 0.0]) is False
    assert arm.set_joint_targets([math.inf, 0.0]) is False
    assert arm.set_joint_targets([0.0, 0.0]) is True


def test_hold_current_position_resets_rate_limited_command_to_feedback():
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    assert arm.set_joint_targets([1.0, -1.0])
    arm._tick(dt=0.1)
    assert arm.get_commanded_positions() != pytest.approx([0.0, 0.0])
    assert arm.hold_current_position()
    assert arm.get_commanded_positions() == pytest.approx(arm.get_joint_positions())


def test_enable_writes_can_timeout_in_50us_counts():
    """Protocol unit: canTimeout 20000=1s (50µs/count). The config's 200ms must be written as 4000.
    Real-hardware lesson: writing 200 directly gives a 10ms threshold, and USB jitter alone
    makes the motor release its own torque back to reset(mode=0)."""
    import struct
    from maker_arm import protocol as p
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    writes = [(cid, d) for cid, d in be.sent if (cid >> 24) & 0x1F == p.COMM_WRITE_PARAM
              and int.from_bytes(d[:2], "little") == p.ParamIndex.CAN_TIMEOUT]
    assert writes, "must write CAN_TIMEOUT"
    for _, d in writes:
        assert struct.unpack("<I", d[4:8])[0] == 200 * p.CAN_TIMEOUT_PER_MS == 4000


def test_enable_verifies_run_mode():
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)   # auto_feedback returns RUN_MODE=0 -> succeeds
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
                return [(reply_id, struct.pack("<H2x", idx) + struct.pack("<B3x", 1))]  # RUN_MODE=1 != 0
        return base(cid, data)
    be.responder = responder
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    with pytest.raises(ConnectError, match="control mode not in effect"):
        arm.enable(start_loop=False)
    assert arm.state is ArmState.CONNECTED


def test_bad_mode_feedback_enters_fault_after_persistence():
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    be.responder = None
    be.inject(*feedback_frame(1, mode=0))   # motor has dropped out of control mode (persistently)
    for _ in range(4):                      # first 4 ticks: tolerated (may be a stale post-enable cache)
        arm._tick(dt=0.005)
        assert arm.state is ArmState.ENABLED
    arm._tick(dt=0.005)                     # tick 5 still abnormal -> judged a real fault
    assert arm.state is ArmState.FAULT
    assert "mode anomaly" in arm.fault_reason


def test_transient_bad_mode_is_tolerated():
    """The first few ticks after enable receive stale mode=0 feedback from the probing era (real-hardware race) -- must not falsely report FAULT."""
    arm, be = make_connected_arm()
    arm.enable(start_loop=False)
    be.responder = None
    be.inject(*feedback_frame(1, mode=0))   # stale cache
    arm._tick(dt=0.005)
    arm._tick(dt=0.005)
    assert arm.state is ArmState.ENABLED
    be.inject(*feedback_frame(1, mode=2))   # fresh feedback arrives
    for _ in range(10):
        arm._tick(dt=0.005)
    assert arm.state is ArmState.ENABLED    # counter has been reset, never falsely reports


def test_enable_refuses_unexplainable_out_of_limit_position():
    """Gate: out-of-limit that neither ±2π can explain (a real anomaly, not a jump) -> refuse to enable, never silently clamp and drag the joint."""
    from maker_arm import protocol as p
    from maker_arm.config import ArmConfig, JointConfig
    from maker_arm.errors import ConnectError
    cfg = ArmConfig(joints=[
        JointConfig(motor_id=1, direction=1, offset=0.0, lo=-1.0, hi=1.0, kp=30, kd=1.0),
        JointConfig(motor_id=2, direction=1, offset=0.0, lo=-1.0, hi=1.0, kp=30, kd=1.0),
    ])
    be = MockBackend()
    be.responder = auto_feedback({1: 2.0, 2: 0.0})   # neither 2.0 nor 2.0±2π is in [-1.35,1.35]
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    assert arm._wrap == [0.0, 0.0]                    # no 2π explanation -> no compensation
    with pytest.raises(ConnectError, match="refusing to enable"):
        arm.enable(start_loop=False)
    assert arm.state is ArmState.CONNECTED
    assert not any((cid >> 24) & 0x1F == p.COMM_ENABLE for cid, _ in be.sent)


def test_enable_allows_small_excursion_within_grace():
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 3.2, 2: 0.0})    # 0.2 over limit < 0.35 grace -> allowed
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    arm.enable(start_loop=False)
    assert arm.state is ArmState.ENABLED


def test_connect_compensates_2pi_wrap_transparently():
    """Per-session 2π compensation: a restart-jumped reading is auto-corrected at connect, transparent in both directions for readings and commands."""
    import math as _m
    from maker_arm import protocol as p
    cfg, be = two_joint_cfg(), MockBackend()
    be.responder = auto_feedback({1: 0.3 + 2 * _m.pi, 2: 0.0})   # motor 1 is physically at 0.3, reading is +2π
    arm = Arm(cfg, be)
    arm.connect(timeout=1.0)
    pos = arm.get_joint_positions()
    assert pos[0] == pytest.approx(0.3, abs=2e-3)                 # reading has been compensated
    arm.enable(start_loop=False)                                  # gate passes (within limits after compensation)
    arm.set_joint_targets([0.5, 0.0])
    for _ in range(200):
        arm._tick(dt=0.01)
    mit = [(cid, d) for cid, d in be.sent if (cid >> 24) & 0x1F == p.COMM_MIT and (cid & 0xFF) == 1]
    sent_pos = p.u16_to_float(int.from_bytes(mit[-1][1][:2], "big"), p.P_MIN, p.P_MAX)
    assert sent_pos == pytest.approx(0.5 + 2 * _m.pi, abs=5e-3)   # command automatically adds back +2π (motor coordinates)


def test_connect_no_compensation_for_sane_readings():
    arm, be = make_connected_arm()
    assert arm._wrap == [0.0, 0.0]
