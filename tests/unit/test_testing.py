"""The testing module: simulated arms an application can drive without hardware."""

import math
import time

import pytest

from conftest import sdk

testing = sdk.testing


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(sdk.arm.Arm, "PARAM_TIMEOUT", 0.005)


def wait_for(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.005)
    return condition()


def test_testing_does_not_need_pytest():
    assert "pytest" not in open(testing.__file__).read()


def test_simulated_arm_on_the_shipped_profile_moves_and_stops():
    arm, sim = testing.simulated_arm()
    joints = arm.config.joints
    assert sim.positions == pytest.approx([min(j.hi, max(j.lo, 0.0)) for j in joints], abs=1e-3)
    arm.connect()
    arm.enable()
    try:
        goal = [(j.lo + j.hi) / 2 for j in joints]
        assert arm.move_to(goal, timeout=10.0)
        assert sim.positions == pytest.approx(goal, abs=0.05)   # move_to arrives within its tolerance
        assert arm.get_joint_positions() == pytest.approx(sim.positions, abs=1e-3)
    finally:
        assert arm.stop()
        arm.disconnect()
    assert not any(m.enabled for m in sim.motors.values())


def test_simulated_arm_starts_where_asked_and_reports_temperatures():
    config = sdk.config.ArmConfig.from_yaml(str(sdk._product.DEFAULT_PROFILE))
    start = [min(j.hi, max(j.lo, 0.1)) for j in config.joints]
    arm, sim = testing.simulated_arm(config, start, temperature=40.0)
    arm.connect()
    try:
        assert arm.get_joint_positions() == pytest.approx(start, abs=1e-3)
        assert arm.get_temperatures() == pytest.approx([40.0] * len(start), abs=1.0)
        with pytest.raises(ValueError, match="one value per joint"):
            testing.simulated_arm(config, [0.0])
    finally:
        arm.disconnect()


def test_injected_fault_overheating_and_silence_stop_the_arm():
    for inject in (lambda sim: sim.fault(0), lambda sim: sim.set_temperature("gripper", 150.0),
                   lambda sim: sim.offline("gripper")):
        arm, sim = testing.simulated_arm()
        arm.connect()
        arm.enable()
        try:
            inject(sim)
            assert wait_for(lambda: arm.state.name == "FAULT"), arm.status()
            assert arm.fault_reason
        finally:
            arm.disable()
            arm.disconnect()


def test_simulated_arms_relax_only_the_loop_timing():
    shipped = sdk.config.ArmConfig.from_yaml(str(sdk._product.DEFAULT_PROFILE))
    arm, _ = testing.simulated_arm()
    assert arm.config.feedback_timeout == max(shipped.feedback_timeout, testing.SIM_FEEDBACK_TIMEOUT)
    assert arm.config.control_rate_hz == min(shipped.control_rate_hz, testing.SIM_CONTROL_RATE_HZ)
    assert arm.config.startup_step * arm.config.control_rate_hz == pytest.approx(
        shipped.startup_step * shipped.control_rate_hz)                    # the same startup speed, rad/s
    assert arm.config.joints == shipped.joints and arm.config.max_velocity == shipped.max_velocity


def test_a_late_tick_of_a_simulated_arm_catches_up_with_the_clock():
    arm, _ = testing.simulated_arm()
    arm.connect()
    arm.enable(start_loop=False)
    try:
        arm._synced = [True] * arm.config.n_joints
        q = arm.get_joint_positions()
        start, q[0] = q[0], q[0] + 1.0
        assert arm.set_joint_targets(q)
        dt = 1.0 / arm.config.control_rate_hz
        step = arm.config.max_velocity * dt
        arm._tick(dt)                                   # the first tick: one period
        time.sleep(3 * dt)
        arm._tick(dt)                                   # three periods late: it catches up, within the cap
        moved = arm.get_commanded_positions()[0] - start
        assert 3.5 * step < moved <= (1 + testing.SIM_MAX_TICK_PERIODS) * step + 1e-9
    finally:
        arm.disable()
        arm.disconnect()


def test_unknown_joint_is_named():
    _, sim = testing.simulated_arm()
    with pytest.raises(KeyError, match="no joint"):
        sim.fault("tail")


def test_simulated_group_from_a_rig_description():
    rig = {"arms": {"left": {"backend": "gs_usb"}, "right": {"backend": "socketcan", "channel": "can1"}}}
    group, sims = testing.simulated_group(rig)
    assert set(sims) == {"left", "right"} and group.parallel
    with group:
        group.enable()
        target = group["left"].gripper_target(1.0)
        q = group["left"].get_joint_positions()
        q[-1] = target
        assert group["left"].move_to(q, timeout=10.0)
        assert sims["left"].positions[-1] == pytest.approx(target, abs=0.05)
        assert group["left"].gripper_opening() == pytest.approx(1.0, abs=0.05)
    assert all(not m.enabled for sim in sims.values() for m in sim.motors.values())
    assert all(math.isfinite(v) for v in sims["right"].positions)
