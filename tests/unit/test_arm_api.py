import json
import math
import threading
import time
from dataclasses import replace

import pytest

import fakes
from conftest import sdk
from fakes import FAULT_TEXT, FakeMotor, classify, make_arm, two_joint_config

Arm, ArmState = sdk.arm.Arm, sdk.arm.ArmState
ArmError, StateError = sdk.errors.ArmError, sdk.errors.StateError


@pytest.fixture(autouse=True)
def fast_param_probe(monkeypatch):
    monkeypatch.setattr(Arm, "PARAM_TIMEOUT", 0.005)


def enabled_arm(motors=None, start_loop=False, **overrides):
    arm, bus, fakes = make_arm(motors, **overrides)
    arm.connect()
    arm.enable(start_loop=start_loop)
    return arm, bus, fakes


def wait(predicate, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not predicate():
        time.sleep(0.005)
    return predicate()


# ── introspection ──
def test_joint_names():
    arm, _, _ = make_arm()
    assert arm.joint_names == ["j1", "gripper"]


def test_detach_leaves_torque_on_and_closes_the_bus():
    arm, bus, motors = enabled_arm(start_loop=True)
    arm.detach()
    assert arm.state is ArmState.DISCONNECTED and arm.backend is bus and not bus.is_open
    assert all(m.enabled for m in motors.values())


def test_detach_names_the_joints_whose_watchdog_releases_them():
    arm, _, _ = enabled_arm([FakeMotor(1, position=0.3, params_supported=True),
                             FakeMotor(2, position=-0.5, params_supported=True)], start_loop=True)
    assert arm.watchdog_joints() == {"gripper": 0.5} and arm.detach_unsafe() == []
    assert arm.detach() == {"gripper": 0.5}
    joints = [replace(j, can_timeout_ms=200) for j in two_joint_config().joints]
    arm, _, _ = enabled_arm([FakeMotor(1, position=0.3, params_supported=True),
                             FakeMotor(2, position=-0.5, params_supported=True)], joints=joints)
    assert arm.detach_unsafe() == ["j1"]   # a stiff joint would drop
    arm.disable()


def test_move_to_stops_where_it_is_when_cancelled():
    arm, _, _ = enabled_arm(start_loop=True, max_velocity=0.5)
    try:
        arm._synced = [True, True]
        cancel = threading.Event()
        threading.Timer(0.2, cancel.set).start()
        started = time.monotonic()
        assert arm.move_to([1.5, -0.5], cancel=cancel) is False
        assert time.monotonic() - started < 1.0
        here = arm.get_joint_positions()[0]
        assert 0.05 < here < 1.0
        time.sleep(0.1)
        assert arm.get_joint_positions()[0] == pytest.approx(here, abs=0.01)   # held, not still moving
        assert arm.move_to([1.5, -0.5], cancel=cancel) is False   # already cancelled: nothing sent
        assert arm.get_commanded_positions()[0] == pytest.approx(here, abs=0.01)
    finally:
        arm.disable()


def test_status_is_json_before_and_after_connect():
    arm, _, _ = make_arm()
    before = arm.status()
    assert before["state"] == "disconnected"
    assert before["joints"]["j1"]["position"] is None and before["joints"]["j1"]["feedback_age"] is None
    json.dumps(before, allow_nan=False)
    arm.connect()
    after = arm.status()
    json.dumps(after, allow_nan=False)
    j1 = after["joints"]["j1"]
    assert after["state"] == "connected" and after["fault_reason"] is None
    assert j1["motor_id"] == 1 and j1["position"] == pytest.approx(0.3, abs=1e-3)
    assert j1["temperature"] == pytest.approx(30.0) and j1["temperatures"]
    assert j1["fault"] is None and j1["feedback_age"] >= 0


def test_status_reports_a_motor_fault():
    arm, _, _ = make_arm([FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5, faulted=True)])
    with pytest.raises(sdk.errors.ConnectError):
        arm.connect()
    assert FAULT_TEXT in arm.status()["joints"]["gripper"]["fault"]


# ── enable(cancel=...) ──
def test_enable_cancelled_before_it_starts_sends_nothing():
    arm, _, fakes = make_arm()
    arm.connect()
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(StateError, match="cancelled"):
        arm.enable(start_loop=False, cancel=cancel)
    assert arm.state is ArmState.CONNECTED
    assert all("enable" not in f.log for f in fakes.values())


def test_enable_cancelled_midway_releases_torque():
    cancel = threading.Event()

    class CancelOnEnable(FakeMotor):
        def handle(self, can_id, data):
            replies = super().handle(can_id, data)
            if classify(can_id, data)[0] == "enable":
                cancel.set()
            return replies

    arm, _, fakes = make_arm([CancelOnEnable(1, position=0.3), FakeMotor(2, position=-0.5)])
    arm.connect()
    with pytest.raises(StateError, match="cancelled"):
        arm.enable(start_loop=False, cancel=cancel)
    assert arm.state is ArmState.CONNECTED
    assert not fakes[1].enabled and "enable" not in fakes[2].log


# ── gains ──
def test_set_gains_reaches_the_next_frames_and_restores():
    arm, _, fakes = enabled_arm()
    arm.set_gains(kp=[30.0, 10.0], kd=[1.0, 0.2])
    arm._tick(0.005)
    assert fakes[1].last_kp == pytest.approx(30.0, abs=0.2)
    assert [j.kp for j in arm.config.joints] == [30.0, 10.0]
    arm.set_gains(kd=[0.5, 0.5])
    assert [j.kp for j in arm.config.joints] == [30.0, 10.0]
    arm.set_gains()
    assert [(j.kp, j.kd) for j in arm.config.joints] == [(60, 2.0), (20, 0.5)]


@pytest.mark.parametrize("kp, kd", [([1.0], None), ([math.nan, 1.0], None), ([-1.0, 1.0], None),
                                    (None, [sdk.motor.KD_RANGE[1] + 1, 0.0])])
def test_set_gains_rejects_bad_values(kp, kd):
    arm, _, _ = make_arm()
    with pytest.raises(ValueError):
        arm.set_gains(kp=kp, kd=kd)
    assert [j.kp for j in arm.config.joints] == [60, 20]


def test_float_mode_follows_the_hand_and_restoring_gains_holds_there():
    hold = replace(two_joint_config().joints[1], hold_torque_nm=0.5)
    arm, _, fakes = enabled_arm(joints=[two_joint_config().joints[0], hold])
    arm.float_mode()
    arm._tick(0.005)
    assert [f.last_kp for f in fakes.values()] == [0.0, 0.0]
    assert all(j.kd == Arm.FLOAT_KD for j in arm.config.joints)
    fakes[1].position = 0.9          # moved by hand
    arm._tick(0.005)
    arm._tick(0.005)
    assert arm.state is ArmState.ENABLED
    assert arm.get_commanded_positions()[0] == pytest.approx(0.9, abs=1e-3)
    arm.set_gains()
    arm._tick(0.005)
    assert fakes[1].frames[-1][1] > 0 and fakes[1].frames[-1][0] == pytest.approx(0.9, abs=1e-3)


# ── move_to ──
def test_move_to_waits_for_arrival():
    arm, _, _ = enabled_arm(start_loop=True)
    try:
        arm._synced = [True, True]
        assert arm.move_to([1.0, -0.5]) is True
        assert arm.get_joint_positions()[0] == pytest.approx(1.0, abs=0.04)
    finally:
        arm.disable()
        arm.disconnect()


def test_move_to_speed_caps_the_command_rate():
    arm, _, _ = enabled_arm(start_loop=True)
    try:
        arm._synced = [True, True]
        start = time.monotonic()
        assert arm.move_to([0.8, -0.5], speed=2.0) is True
        assert time.monotonic() - start >= 0.2       # 0.5 rad at 2 rad/s
        assert arm._speed == 2.0
        assert arm.set_joint_targets([0.8, -0.5]) and arm._speed is None
    finally:
        arm.disable()
        arm.disconnect()


def test_move_to_times_out_when_blocked_and_refuses_when_not_enabled():
    arm, _, fakes = make_arm()
    assert arm.move_to([0.5, -0.5], wait=False) is False
    arm.connect()
    arm.enable()
    try:
        arm._synced = [True, True]
        fakes[1].track = False
        assert arm.move_to([1.0, -0.5], timeout=0.2) is False
        assert arm.state is ArmState.ENABLED
        assert arm.move_to([0.3, -0.5], wait=False) is True
        with pytest.raises(ValueError):
            arm.move_to([0.3, -0.5], speed=0.0)
    finally:
        arm.disable()
        arm.disconnect()


def test_move_to_returns_false_when_targets_are_taken_over():
    arm, _, fakes = enabled_arm(start_loop=True)
    try:
        fakes[1].track = False
        threading.Timer(0.05, lambda: arm.set_joint_targets([0.3, -0.5])).start()
        assert arm.move_to([1.5, -0.5], timeout=1.0) is False
    finally:
        arm.disable()
        arm.disconnect()


# ── operator actions ──
def test_set_zero_only_with_torque_off():
    turned = fakes.FULL_TURN_SUPPORTED
    arm, _, fake = make_arm([FakeMotor(1, position=0.3 + (2 * math.pi if turned else 0.0)),
                             FakeMotor(2, position=-0.5)])
    with pytest.raises(StateError):
        arm.set_zero()
    arm.connect()
    assert (arm._wrap[0] != 0.0) == turned
    arm.set_zero([0])
    assert "zero" in fake[1].log and "zero" not in fake[2].log
    assert arm._wrap[0] == 0.0
    arm.refresh()
    assert arm.get_joint_positions()[0] == pytest.approx(0.0, abs=1e-3)
    with pytest.raises(IndexError):
        arm.set_zero([2])
    arm.enable(start_loop=False)
    with pytest.raises(StateError):
        arm.set_zero()


def test_set_motor_id_validation():
    arm, _, _ = make_arm()
    hi = sdk.motor.ID_RANGE[1]
    with pytest.raises(ValueError):
        arm.set_motor_id(1, 1)
    with pytest.raises(ValueError):
        arm.set_motor_id(1, hi + 1)
    arm.connect()
    arm.enable(start_loop=False)
    with pytest.raises(StateError):
        arm.set_motor_id(1, 9)


def test_set_motor_id_refuses_an_id_already_on_the_bus_and_cleans_up():
    stranger = FakeMotor(5, position=0.1)                   # on the bus, not in the profile
    arm, bus, _ = make_arm([FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5), stranger])
    with pytest.raises(ArmError, match="already answers"):
        arm.set_motor_id(1, 5, torque_off_confirmed=True)
    assert not bus.is_open and arm.state is ArmState.DISCONNECTED
    arm.connect()
    with pytest.raises(ArmError, match="already answers"):
        arm.set_motor_id(1, 5, torque_off_confirmed=True)
    assert bus.is_open and arm.refresh() == [True, True]   # the arm's receive path is back


def test_set_motor_id_gives_a_motor_the_profile_lacks_the_id_of_a_joint_without_its_motor():
    # Motor replacement, or a motor moved off its joint's id and back: the joint's own id is free on the bus.
    arm, bus, fakes = make_arm([FakeMotor(9, position=0.3), FakeMotor(2, position=-0.5)])
    arm.set_motor_id(9, 1, torque_off_confirmed=True, replacing="j1")
    assert not bus.is_open and arm.config.joints[0].motor_id == 1
    arm.connect()
    assert arm.refresh() == [True, True]
    arm.disconnect()


def test_set_motor_id_gives_a_joints_id_only_to_the_motor_named_as_its_replacement():
    # Across two calls a joint's motor moved to a spare id looks like a stranger; it must not take another joint's id.
    arm, bus, fakes = make_arm([FakeMotor(9, position=0.3), FakeMotor(1, position=0.3)])   # the gripper (2) is off
    with pytest.raises(ArmError, match="replacing"):
        arm.set_motor_id(9, 2, torque_off_confirmed=True)
    assert not bus.is_open and fakes[9].motor_id == 9
    with pytest.raises(ArmError, match="replacing"):
        arm.set_motor_id(9, 2, torque_off_confirmed=True, replacing="j1")   # named, but not the joint with id 2
    arm.set_motor_id(9, 2, torque_off_confirmed=True, replacing="gripper")
    assert arm.config.joints[1].motor_id == 2


def test_set_motor_id_refuses_an_id_another_joint_has_before_opening_the_bus():
    arm, bus, fakes = make_arm()
    fakes[2].online = False                                 # even when that joint does not answer
    with pytest.raises(ArmError, match="belongs to"):
        arm.set_motor_id(1, 2, torque_off_confirmed=True)
    assert not bus.is_open and bus.sent == []


def test_gripper_opening_and_target_from_the_gripper_joint():
    arm, _, motors = make_arm()   # gripper limits [-1.6, -0.04], closing toward +: closed -0.04, open -1.6
    arm.connect()
    try:
        assert arm.gripper_target(0.0) == pytest.approx(-0.04) and arm.gripper_target(1.0) == pytest.approx(-1.6)
        assert arm.gripper_target(2.0) == pytest.approx(-1.6)    # clipped
        motors[2].position = arm.gripper_target(0.25)
        arm.refresh()
        assert arm.gripper_opening() == pytest.approx(0.25, abs=1e-3)
        with pytest.raises(ValueError):
            arm.gripper_target(math.nan)
    finally:
        arm.disconnect()
    joints = two_joint_config().joints
    arm, _, _ = make_arm(joints=[joints[0], replace(joints[1], open_position=-1.0)])
    assert arm.gripper_target(1.0) == pytest.approx(-1.0)
    arm, _, _ = make_arm(joints=[joints[0], replace(joints[1], name="hand")])
    with pytest.raises(sdk.errors.ArmError, match="gripper"):
        arm.gripper_opening()
