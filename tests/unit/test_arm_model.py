"""The Arm's use of the profile's model (URDF): gravity feed-forward in the frames, float mode's
extra torque, ee_pose / move_ee. A one-joint pendulum URDF drives the shared two-joint test arm."""

import math
import threading
import time
from dataclasses import replace

import pytest

from conftest import sdk
from fakes import FakeMotor, make_arm, two_joint_config

Arm, ArmState = sdk.arm.Arm, sdk.arm.ArmState
ArmError = sdk.errors.ArmError
gravity, kinematics = sdk.gravity, sdk.kinematics

PENDULUM = """<?xml version="1.0"?>
<robot name="pendulum">
  <link name="base"><inertial><mass value="0.5"/><origin xyz="0 0 0.02"/></inertial></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="10" velocity="5"/>
  </joint>
  <link name="arm"><inertial><mass value="1.0"/><origin xyz="0.2 0 0"/></inertial></link>
</robot>
"""
LIMIT_NM = 3.0
TCP = [0.3, 0.0, 0.0]   # m on the pendulum link: the tool point moves with the joint
TOL = 0.02   # N*m: one LSB of a 12-bit torque field and some


@pytest.fixture(autouse=True)
def fast_param_probe(monkeypatch):
    monkeypatch.setattr(Arm, "PARAM_TIMEOUT", 0.005)


@pytest.fixture
def urdf(tmp_path):
    path = tmp_path / "pendulum.urdf"
    path.write_text(PENDULUM)
    return str(path)


def model_joints(**j1):
    base = two_joint_config().joints
    return [replace(base[0], **{"gravity_limit_nm": LIMIT_NM, **j1}), base[1]]


def model_arm(urdf, motors=None, connect=True, **overrides):
    overrides.setdefault("joints", model_joints())
    overrides.setdefault("tcp_xyz", list(TCP))
    arm, bus, fakes = make_arm(motors, urdf=urdf, **overrides)
    if connect:
        arm.connect()
    return arm, bus, fakes


def holding(urdf, q):
    """Joint-sign holding torque of the pendulum at joint angle q (independent of the arm)."""
    chain = kinematics.Chain.from_urdf(urdf)
    return float(chain.gravity_torques([q])[0])


def ticking(urdf, **kwargs):
    arm, bus, fakes = model_arm(urdf, **kwargs)
    arm.enable(start_loop=False)
    arm._gravity.level = 1.0
    arm._tick(0.005)
    return arm, fakes


def sent_tau(fakes):
    return [fakes[i].frames[-1][2] for i in sorted(fakes)]


def teardown(arm):
    arm.disable()
    arm.disconnect()


# ── gravity feed-forward ──
def test_pendulum_model_sanity(urdf):
    assert abs(holding(urdf, 0.0)) == pytest.approx(9.81 * 0.2, abs=1e-9)
    assert holding(urdf, 0.3) == pytest.approx(holding(urdf, 0.0) * math.cos(0.3), abs=1e-9)


def test_gravity_reaches_the_frames_and_the_gripper_gets_none(urdf):
    arm, fakes = ticking(urdf)
    assert sent_tau(fakes) == pytest.approx([holding(urdf, 0.3), 0.0], abs=TOL)
    teardown(arm)


def test_a_flipped_joint_direction_sends_motor_sign(urdf):
    motors = [FakeMotor(1, position=0.1 - 0.3), FakeMotor(2, position=-0.5)]
    arm, fakes = ticking(urdf, motors=motors, joints=model_joints(direction=-1, offset=0.1))
    assert arm.get_joint_positions()[0] == pytest.approx(0.3, abs=1e-3)
    assert sent_tau(fakes)[0] == pytest.approx(-holding(urdf, 0.3), abs=TOL)
    teardown(arm)


def test_the_cap_and_scale_apply(urdf):
    arm, fakes = ticking(urdf, joints=model_joints(gravity_limit_nm=0.5))
    assert abs(sent_tau(fakes)[0]) == pytest.approx(0.5, abs=TOL)
    teardown(arm)
    arm, fakes = ticking(urdf, gravity_scale=0.5)
    assert sent_tau(fakes)[0] == pytest.approx(holding(urdf, 0.3) / 2, abs=TOL)
    teardown(arm)


def test_every_enable_eases_in_from_zero(urdf):
    arm, _, fakes = model_arm(urdf)
    for _ in range(2):
        arm.enable(start_loop=False)
        arm._tick(0.005)
        assert fakes[1].frames[-1][1] > 0 and fakes[1].frames[-1][2] == pytest.approx(0.0, abs=TOL)
        time.sleep(gravity.RAMP_STEP_S * 2)
        arm._tick(0.005)
        fraction = gravity.RAMP_STEP_S / gravity.RAMP_S
        assert fakes[1].frames[-1][2] == pytest.approx(holding(urdf, 0.3) * fraction, abs=TOL)
        arm.disable()
    arm.disconnect()


def test_zero_gain_frames_carry_no_torque_and_disable_resends_zero(urdf):
    arm, fakes = ticking(urdf)
    arm.disable()
    zero_gain = [frame for frame in fakes[1].frames if frame[1] == 0.0]
    assert len(zero_gain) >= 2   # the enable preload and the release
    assert all(abs(frame[2]) < TOL for frame in zero_gain)
    assert fakes[1].frames[-1][1] == 0.0
    arm.disconnect()


def test_stale_feedback_falls_back_to_the_goal(urdf):
    arm, fakes = ticking(urdf)
    with arm._target_lock:
        arm._user_targets[0] = arm._internal[0] = 0.8
    for m in arm.motors:
        m._fb_time = time.monotonic()
    arm._tick(0.005)
    assert sent_tau(fakes)[0] == pytest.approx(holding(urdf, 0.3), abs=TOL)
    arm.motors[0]._fb_time = time.monotonic() - (gravity.STALE_S + 0.05)
    arm._tick(0.005)
    assert sent_tau(fakes)[0] == pytest.approx(holding(urdf, 0.8), abs=TOL)
    teardown(arm)


def test_a_model_failure_turns_gravity_off_and_keeps_holding(urdf, monkeypatch):
    arm, fakes = ticking(urdf)

    def boom(q):
        raise RuntimeError("boom")

    monkeypatch.setattr(arm._gravity, "model", boom)
    before = len(fakes[1].frames)
    arm._tick(0.005)
    arm._tick(0.005)
    assert arm._gravity is None and arm.state is ArmState.ENABLED
    assert len(fakes[1].frames) == before + 2 and fakes[1].frames[-1][1] > 0
    assert sent_tau(fakes) == pytest.approx([0.0, 0.0], abs=TOL)
    teardown(arm)


def test_a_fault_hold_carries_gravity(urdf):
    arm, fakes = ticking(urdf)
    arm._enter_fault("test fault", 0)
    assert arm.state is ArmState.FAULT
    arm._tick(0.005)
    assert sent_tau(fakes)[0] == pytest.approx(holding(urdf, 0.3), abs=TOL)
    teardown(arm)


@pytest.mark.parametrize("overrides", [dict(urdf=None), dict(gravity_scale=0.0)])
def test_no_urdf_or_scale_zero_means_no_gravity(urdf, overrides):
    arm, _, fakes = make_arm(joints=model_joints(), **{"urdf": urdf, **overrides})
    arm.connect()
    arm.enable(start_loop=False)
    assert arm._gravity is None
    arm._tick(0.005)
    assert sent_tau(fakes) == pytest.approx([0.0, 0.0], abs=TOL)
    teardown(arm)


def test_bad_gravity_settings_are_refused_before_any_frame(urdf):
    arm, _, fakes = model_arm(urdf)
    arm.config.gravity_scale = 5.0
    with pytest.raises(ValueError, match="gravity_scale"):
        arm.enable(start_loop=False)
    arm.config.gravity_scale = 1.0
    arm.config.joints[0] = replace(arm.config.joints[0], gravity_limit_nm=None)
    with pytest.raises(ValueError, match="gravity_limit_nm"):
        arm.enable(start_loop=False)
    assert all("enable" not in f.log and not f.frames for f in fakes.values())
    arm.disconnect()


# ── float mode's extra torque ──
def test_extra_torque_is_added_each_tick_and_set_gains_ends_it(urdf):
    calls = []

    def friction(positions, velocities):
        calls.append((list(positions), list(velocities)))
        return [0.25, 0.1]

    arm, fakes = ticking(urdf)
    arm.float_mode(extra_torque=friction)
    arm._tick(0.005)
    assert calls and calls[-1][0] == pytest.approx([0.3, -0.5], abs=1e-3)
    assert sent_tau(fakes) == pytest.approx([holding(urdf, 0.3) + 0.25, 0.1], abs=TOL)
    assert [f.frames[-1][1] for f in fakes.values()] == [0.0, 0.0]
    arm.set_gains()
    arm._tick(0.005)
    assert sent_tau(fakes) == pytest.approx([holding(urdf, 0.3), 0.0], abs=TOL)
    teardown(arm)


def test_extra_torque_in_motor_sign_without_a_model():
    arm, _, fakes = make_arm([FakeMotor(1, position=-0.3), FakeMotor(2, position=-0.5)],
                             joints=[replace(two_joint_config().joints[0], direction=-1),
                                     two_joint_config().joints[1]])
    arm.connect()
    arm.enable(start_loop=False)
    arm.float_mode(extra_torque=lambda q, v: [0.2, 0.0])
    arm._tick(0.005)
    assert sent_tau(fakes) == pytest.approx([-0.2, 0.0], abs=TOL)
    teardown(arm)


@pytest.mark.parametrize("bad", [
    lambda q, v: 1 / 0,
    lambda q, v: [0.1],
    lambda q, v: [math.nan, 0.0],
])
def test_a_failing_extra_torque_is_dropped_never_the_frames(urdf, bad):
    arm, fakes = ticking(urdf)
    arm.float_mode(extra_torque=bad)
    before = len(fakes[1].frames)
    arm._tick(0.005)
    arm._tick(0.005)
    assert arm._extra_torque is None and arm.state is ArmState.ENABLED
    assert len(fakes[1].frames) == before + 2
    assert sent_tau(fakes) == pytest.approx([holding(urdf, 0.3), 0.0], abs=TOL)
    teardown(arm)


def test_float_mode_refuses_a_non_callable():
    arm, _, _ = make_arm()
    with pytest.raises(TypeError):
        arm.float_mode(extra_torque=0.1)


# ── ee_pose / move_ee ──
def test_kinematics_need_a_urdf():
    arm, _, _ = make_arm()
    arm.connect()
    with pytest.raises(ArmError, match="urdf"):
        arm.ee_pose()
    with pytest.raises(ArmError, match="urdf"):
        arm.move_ee([0.1, 0.0, 0.1])
    arm.disconnect()


def tool_chain(urdf):
    return kinematics.Chain.from_urdf(urdf, tcp_xyz=TCP)


def test_ee_pose_is_the_measured_tool_point(urdf):
    arm, _, _ = model_arm(urdf)
    xyz, rpy = arm.ee_pose()
    chain = tool_chain(urdf)
    expected_xyz, expected_rpy = kinematics.pose_to_xyz_rpy(chain.fk([arm.get_joint_positions()[0]]))
    assert xyz == pytest.approx(expected_xyz.tolist(), abs=1e-9)
    assert rpy == pytest.approx(expected_rpy.tolist(), abs=1e-9)
    assert xyz[2] == pytest.approx(0.1 - 0.3 * math.sin(0.3), abs=1e-3)   # tilted down by +q about y
    arm.disconnect()


def test_move_ee_reaches_the_point_and_leaves_the_gripper(urdf):
    arm, _, fakes = model_arm(urdf)
    arm.enable()
    try:
        goal, _ = kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([0.8]))
        gripper = arm.get_commanded_positions()[1]
        assert arm.move_ee(goal.tolist(), timeout=3.0)
        assert arm.get_joint_positions()[0] == pytest.approx(0.8, abs=0.01)
        assert arm.get_commanded_positions()[1] == pytest.approx(gripper)
        xyz, _ = arm.ee_pose()
        assert xyz == pytest.approx(goal.tolist(), abs=2e-3)
    finally:
        teardown(arm)


@pytest.mark.parametrize("q_goal", [2.5, None])
def test_move_ee_refuses_unreachable_or_out_of_limit_points(urdf, q_goal):
    arm, _, _ = model_arm(urdf)
    arm.enable(start_loop=False)
    goal = (kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([q_goal]))[0].tolist() if q_goal is not None
            else [1.0, 0.0, 1.0])
    before = list(arm._user_targets)
    assert arm.move_ee(goal, wait=False) is False
    assert arm._user_targets == before
    teardown(arm)


def test_move_ee_streams_bounded_steps_along_the_line(urdf):
    arm, _, _ = model_arm(urdf)
    arm.enable()
    sent = []
    original = arm._set_targets
    arm._set_targets = lambda targets, speed, *token: sent.append(list(targets)) or original(targets, speed, *token)
    try:
        start = arm.get_commanded_positions()
        goal, _ = kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([0.6]))
        assert arm.move_ee(goal.tolist(), speed=2.0, timeout=5.0)
        rows = [start] + sent
        steps = [abs(b[0] - a[0]) for a, b in zip(rows, rows[1:])]
        assert len(sent) > 10 and max(steps) <= 2.0 / arm.config.control_rate_hz + 1e-9
        assert all(r[1] == pytest.approx(start[1]) for r in sent)
        assert sent[-1][0] == pytest.approx(0.6, abs=5e-3)
    finally:
        teardown(arm)


def test_move_ee_cancel_stops_and_holds(urdf):
    arm, _, _ = model_arm(urdf)
    arm.enable()
    cancel = threading.Event()
    try:
        goal, _ = kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([1.2]))
        threading.Timer(0.1, cancel.set).start()
        assert arm.move_ee(goal.tolist(), speed=1.0, cancel=cancel) is False
        assert arm.get_commanded_positions()[0] < 1.0
    finally:
        teardown(arm)


@pytest.fixture
def hanging_planner(monkeypatch):
    """move_ee's planning helper takes the request and never answers."""
    planner = sdk._planner
    real_helper = planner._Helper
    monkeypatch.setattr(planner, "_pool", planner._Pool())
    monkeypatch.setattr(planner, "_Helper",
                        lambda: real_helper("import sys, time; sys.stdin.buffer.read(1); time.sleep(60)"))


@pytest.mark.parametrize("stop", ["timeout", "cancel"])
def test_move_ee_planning_that_does_not_finish_returns_false_holding(urdf, hanging_planner, caplog, stop):
    arm, _, _ = model_arm(urdf)
    arm.enable()
    cancel = threading.Event()
    try:
        goal, _ = kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([0.6]))
        before, ticks = arm.get_commanded_positions(), arm._tick_count
        if stop == "cancel":
            threading.Timer(0.3, cancel.set).start()
        t0 = time.monotonic()
        with caplog.at_level("WARNING"):
            ok = arm.move_ee(goal.tolist(), timeout=0.3 if stop == "timeout" else 30.0, cancel=cancel)
        took = time.monotonic() - t0
        assert ok is False and took < 1.5
        assert ("timed out" if stop == "timeout" else "cancelled") in caplog.text and "nothing was sent" in caplog.text
        assert arm.state is ArmState.ENABLED and arm.get_commanded_positions() == pytest.approx(before)
        assert arm._tick_count - ticks > 0.5 * took * arm.config.control_rate_hz   # the loop kept running
        assert arm.move_to(before, timeout=2.0)                                     # and the arm still takes targets
    finally:
        teardown(arm)


@pytest.fixture
def slow_planner(monkeypatch):
    """move_ee's planning helper solves the path correctly, one second late."""
    planner = sdk._planner
    real_helper = planner._Helper
    code = (f"from {sdk.__name__} import _planner as p\nimport time\nsolve = p.plan_ee\n"
            "p.plan_ee = lambda *a, **k: (time.sleep(1.0), solve(*a, **k))[1]\np._serve()")
    monkeypatch.setattr(planner, "_pool", planner._Pool())
    monkeypatch.setattr(planner, "_Helper", lambda: real_helper(code))


def move_ee_in_background(arm, goal, **kw):
    result = {}

    def run():
        result["ok"] = arm.move_ee(goal, **kw)
        result["at"] = time.monotonic()
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, result


def test_a_newer_command_during_planning_owns_the_arm(urdf, slow_planner):
    arm, _, _ = model_arm(urdf)
    arm.enable()
    try:
        goal, _ = kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([0.8]))
        thread, result = move_ee_in_background(arm, goal.tolist(), timeout=10.0)
        time.sleep(0.3)
        newer = [0.5] + arm.get_commanded_positions()[1:]
        assert arm.move_to(newer, wait=False)
        superseded = time.monotonic()
        thread.join(5.0)
        assert result["ok"] is False and result["at"] - superseded < 0.5
        time.sleep(1.2)   # past the moment the plan would have been ready
        with arm._target_lock:
            assert arm._user_targets == pytest.approx(newer)
    finally:
        teardown(arm)


def test_estop_and_enable_again_during_planning_sends_nothing_of_the_old_move(urdf, slow_planner):
    arm, _, _ = model_arm(urdf)
    arm.enable()
    try:
        goal, _ = kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([0.8]))
        thread, result = move_ee_in_background(arm, goal.tolist(), timeout=10.0)
        time.sleep(0.3)
        arm.estop()
        arm.enable()
        held = arm.get_commanded_positions()
        thread.join(5.0)
        time.sleep(1.2)
        assert result["ok"] is False
        with arm._target_lock:
            assert arm._user_targets == pytest.approx(held)
    finally:
        teardown(arm)


def test_a_motion_superseded_after_planning_commits_no_row(urdf, monkeypatch):
    # The last moment: the plan is ready, then a stop's hold takes over before the first row is committed.
    arm, _, _ = model_arm(urdf)
    arm.enable()
    try:
        goal, _ = kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([0.8]))
        real_plan = sdk._planner.plan

        def plan_then_hold(*a, **k):
            k["cancel"] = None
            result = real_plan(*a, **k)
            arm.hold_current_position()
            return result
        monkeypatch.setattr(sdk._planner, "plan", plan_then_hold)
        before = arm.get_commanded_positions()
        assert arm.move_ee(goal.tolist(), timeout=5.0) is False
        with arm._target_lock:
            assert arm._user_targets == pytest.approx(before, abs=1e-3)
    finally:
        teardown(arm)


def test_move_ee_without_wait_streams_in_the_background(urdf):
    arm, _, _ = model_arm(urdf)
    arm.enable()
    try:
        goal, _ = kinematics.pose_to_xyz_rpy(tool_chain(urdf).fk([0.5]))
        assert arm.move_ee(goal.tolist(), speed=2.0, wait=False) is True
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and abs(arm.get_joint_positions()[0] - 0.5) > 0.01:
            time.sleep(0.01)
        assert arm.get_joint_positions()[0] == pytest.approx(0.5, abs=0.01)
    finally:
        teardown(arm)


def test_move_ee_refused_unless_enabled(urdf):
    arm, _, _ = model_arm(urdf)
    assert arm.move_ee([0.2, 0.0, 0.1]) is False
    arm.disconnect()


# ── id change ──
def test_set_motor_id_reroutes_the_joint(monkeypatch):
    arm, _, _ = make_arm()
    arm.config.required_motor_ids = [1, 2]
    monkeypatch.setattr(sdk.motor.Motor, "set_id", lambda self, new_id, timeout=0.5: None)
    arm.set_motor_id(1, 9)
    assert arm.config.joints[0].motor_id == 9 and arm.config.required_motor_ids == [9, 2]
    assert arm.motors[0].motor_id == 9 and set(arm._by_id) == {9, 2}
    arm.config._validate()
