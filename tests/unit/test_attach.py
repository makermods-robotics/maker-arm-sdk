"""Arm.attach(): taking over an arm left holding with torque on, on simulated motors."""

import math
import threading
import time

import pytest

from conftest import sdk

Arm, ArmState = sdk.arm.Arm, sdk.arm.ArmState
ConnectError, StateError = sdk.errors.ConnectError, sdk.errors.StateError
testing = sdk.testing
HOME = {"shoulder_lift": -1.0, "elbow_flex": 1.2, "wrist_flex": 0.3}


@pytest.fixture(autouse=True)
def fast_param_probe(monkeypatch):
    monkeypatch.setattr(Arm, "PARAM_TIMEOUT", 0.005)


def home_pose(config):
    q = [min(j.hi, max(j.lo, 0.0)) for j in config.joints]
    for i, j in enumerate(config.joints):
        if j.name in HOME:
            q[i] = min(j.hi - 0.05, max(j.lo + 0.05, HOME[j.name]))
    return q


def left_energized(**options):
    """(sim, backend): a previous session enabled the arm, moved it home and detached, then the
    gripper's watchdog released it."""
    first, sim = testing.simulated_arm(**options)
    first.connect()
    first.enable()
    assert first.move_to(home_pose(first.config), speed=20.0)
    first.detach()
    sim.watchdog_timeout("gripper")
    return sim, first.backend


def fresh_arm(sim, backend):
    arm = Arm(sim.config, backend)
    arm._busy_wait_us = lambda us: None
    return arm


def clear_logs(sim):
    for m in sim.motors.values():
        m.log.clear()
        m.frames.clear()


def stiff(sim):
    return [m for j, m in zip(sim.config.joints, sim.motors.values()) if not j.release_on_fault]


def wait(predicate, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not predicate():
        time.sleep(0.005)
    return predicate()


def test_attach_takes_over_without_a_sag_and_rests_only_on_an_explicit_target():
    sim, backend = left_energized()
    arm = fresh_arm(sim, backend)
    if sim.motor("gripper").faulted:   # the protocol reports the watchdog release as a status
        with pytest.raises(ConnectError):
            arm.connect()
        arm = fresh_arm(sim, backend)
    arm.connect(clear_released=True)
    held = [m.position for m in sim.motors.values()]
    before = sim.positions
    clear_logs(sim)
    arm.attach()
    try:
        assert arm.state is ArmState.ENABLED and arm.rest_pose is None
        assert wait(lambda: all(len(m.frames) >= 5 for m in sim.motors.values()))
        for m, position in zip(stiff(sim), held):
            assert "enable" not in m.log and "disable" not in m.log
            assert all(kp > 0 for _, kp, _ in m.frames)            # no zero-gain frame
            assert m.frames[0][0] == pytest.approx(position, abs=1e-3)   # first command: the measured pose
        assert "enable" in sim.motor("gripper").log                # re-enabled where it is
        assert sim.positions == pytest.approx(before, abs=1e-3)    # nothing moved
        assert arm.stop() is False                                 # no rest pose to verify: keeps holding
        assert arm.state is ArmState.ENABLED and all(m.enabled for m in sim.motors.values())
        rest = [min(j.hi, max(j.lo, 0.0)) for j in arm.config.joints]
        assert arm.return_to_rest(rest)
    finally:
        arm.disable()
    assert not any(m.enabled for m in sim.motors.values())


def test_attach_refuses_an_arm_with_torque_off():
    arm, sim = testing.simulated_arm()
    arm.connect()
    clear_logs(sim)
    with pytest.raises(ConnectError, match="use enable"):
        arm.attach()
    assert arm.state is ArmState.CONNECTED
    assert not any(k in m.log for m in sim.motors.values() for k in ("enable", "disable"))
    assert not any(m.frames for m in sim.motors.values())


def refused(sim, arm, match):
    clear_logs(sim)
    with pytest.raises(ConnectError, match=match):
        arm.attach()
    assert arm.state is ArmState.CONNECTED
    for m in stiff(sim):
        assert m.enabled and not m.frames and "disable" not in m.log and "enable" not in m.log


def test_attach_refuses_a_fault_a_silent_motor_and_a_joint_outside_its_limits():
    sim, backend = left_energized()
    arm = fresh_arm(sim, backend)
    arm.connect(clear_released=True)
    sim.fault("elbow_flex")
    refused(sim, arm, "fault")
    sim.fault("elbow_flex", False)

    arm = fresh_arm(sim, backend)
    arm.connect(clear_released=True)
    sim.offline("wrist_roll")
    refused(sim, arm, "no reply")
    sim.offline("wrist_roll", False)

    config = testing.simulated_arm()[0].config
    q = home_pose(config)
    i = [j.name for j in config.joints].index("elbow_flex")
    q[i] = config.joints[i].lo - math.radians(40)
    arm, sim = testing.simulated_arm(config, q, enabled=True)
    arm.connect()
    refused(sim, arm, "outside its limits")


def test_attach_cancelled_sends_nothing():
    sim, backend = left_energized()
    arm = fresh_arm(sim, backend)
    arm.connect(clear_released=True)
    clear_logs(sim)
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(StateError):
        arm.attach(cancel=cancel)
    assert all(m.enabled and not m.frames for m in stiff(sim))


@pytest.mark.parametrize("was_on", [True, False])
def test_unknown_torque_gets_a_holding_frame_before_its_enable(was_on):
    config = testing.simulated_arm()[0].config
    arm, sim = testing.simulated_arm(config, home_pose(config), enabled=was_on, params_supported=False)
    arm.connect()
    if any(m.feedback.enabled is not None for m in arm.motors):
        pytest.skip("this protocol always reports torque state")
    before = sim.positions
    clear_logs(sim)
    arm.attach()
    try:
        for j, m in zip(config.joints, sim.motors.values()):
            first = m.log.index("enable")
            kind, kp = m.log[first - 1]                            # holding frame, profile gains, then enable
            assert kind == "cmd" and kp == pytest.approx(j.kp, rel=0.01)
            assert all(kp > 0 for _, kp, _ in m.frames)
        assert all(m.enabled for m in sim.motors.values())
        assert sim.positions == pytest.approx(before, abs=1e-3)
    finally:
        arm.disable()
