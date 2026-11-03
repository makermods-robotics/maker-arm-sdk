"""gravity_fixtures.json holds the values of the verified reference model on a spread of poses."""

import json
import logging
import math
import random
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from fakes import FakeBus, FakeMotor, two_joint_config
from maker_arm import gravity, protocol
from maker_arm.arm import Arm, ArmState
from maker_arm.cli.common import apply_gravity_arg
from maker_arm.config import ArmConfig
from maker_arm.gravity import FRESH_S, LIMIT_NM, RAMP_S, RAMP_STEP_S, GravityFeedforward, gravity_nm, model_applies
from maker_arm.profiles import DEFAULT_ARM_CONFIG

HERE = Path(__file__).resolve().parent
URDF = ET.parse(HERE.parents[1] / "urdf" / "maker_arm" / "robot.urdf").getroot()
FIXTURES = json.loads((HERE / "gravity_fixtures.json").read_text())
RAISED = [5.88, -52.67, 52.31, 3.78, 8.85, 4.41]
ONE_LSB = 0.01  # N*m: a 12-bit torque field on RS00 / RS02
FRESH = [0.0] * 6


@pytest.fixture(autouse=True)
def fast_param_probe(monkeypatch):
    monkeypatch.setattr(Arm, "PARAM_TIMEOUT", 0.005)


def floats(text):
    return tuple(float(v) for v in text.split())


# ── model ──
@pytest.mark.parametrize("fixture", FIXTURES, ids=lambda f: str(f["q_deg"]))
def test_model_matches_the_reference(fixture):
    assert gravity_nm(fixture["q_deg"]) == pytest.approx(fixture["tau_nm"], abs=1e-9)


def test_model_sanity_value():
    assert gravity_nm(RAISED) == pytest.approx([0, 1.588, 4.682, 1.356, 0.007, -0.001], abs=1e-3)


def test_model_is_scaled_and_periodic():
    assert gravity_nm(RAISED, 0.5) == pytest.approx([t / 2 for t in gravity_nm(RAISED)])
    assert gravity_nm([RAISED[0] + 360, *RAISED[1:]]) == pytest.approx(gravity_nm(RAISED))


def test_model_of_a_non_finite_pose_is_nan_not_an_error():
    assert all(math.isnan(t) for t in gravity_nm([math.inf, *RAISED[1:]]))


def test_runtime_does_not_need_numpy():
    code = "import sys; sys.modules['numpy'] = None; import maker_arm; from maker_arm.gravity import gravity_nm; gravity_nm([0] * 6)"
    subprocess.run([sys.executable, "-c", code], check=True)


def test_link_table_equals_the_urdf():
    def inertial(name):
        node = URDF.find(f"link[@name='{name}']/inertial")
        origin = node.find("origin")
        assert not any(floats(origin.get("rpy")))
        return float(node.find("mass").get("value")), floats(origin.get("xyz"))

    assert gravity.LINKS == tuple(inertial(f"link_00{n}") for n in range(2, 8))
    assert gravity.JAWS == (inertial("gripper_left"), inertial("gripper_right"))


def test_jaws_lump_at_the_link_007_origin():
    for name in ("gripper_left_joint", "gripper_right_joint"):
        origin = URDF.find(f"joint[@name='{name}']/origin")
        assert not any(floats(origin.get("xyz"))) and not any(floats(origin.get("rpy")))


def test_chain_equals_the_urdf_with_the_wrist_yaw_axis_flipped():
    for n, (xyz, rpy, axis) in enumerate(gravity.CHAIN):
        joint = URDF.find(f"joint[@name='link_00{n + 2}_joint']")
        assert joint.find("parent").get("link") == ("base_link" if n == 0 else f"link_00{n + 1}")
        assert floats(joint.find("origin").get("xyz")) == xyz
        assert floats(joint.find("origin").get("rpy")) == rpy
        sign = -1.0 if n == 4 else 1.0
        assert tuple(sign * a for a in floats(joint.find("axis").get("xyz"))) == axis


# ── feed-forward safeguards ──
def test_ease_in_starts_at_zero_and_takes_the_ramp_time():
    ff = GravityFeedforward(1.0)
    assert ff.torques(RAISED, FRESH, RAISED, 10.0) == pytest.approx([0.0] * 6)
    frames = int(RAMP_S / 0.02)
    for k in range(1, frames):
        tau = ff.torques(RAISED, FRESH, RAISED, 10.0 + k * 0.02)
        assert tau[2] == pytest.approx(gravity_nm(RAISED)[2] * k / frames)
    assert ff.torques(RAISED, FRESH, RAISED, 10.0 + frames * 0.02) == pytest.approx(gravity_nm(RAISED))
    assert ff.level == 1.0


def test_a_pause_never_skips_the_ease_in():
    ff = GravityFeedforward(1.0)
    ff.torques(RAISED, FRESH, RAISED, 0.0)
    ff.torques(RAISED, FRESH, RAISED, 100.0)
    assert ff.level == pytest.approx(RAMP_STEP_S / RAMP_S)


def test_a_clock_step_backwards_does_not_advance_or_undo_the_ease_in():
    ff = GravityFeedforward(1.0)
    ff.torques(RAISED, FRESH, RAISED, 10.0)
    ff.torques(RAISED, FRESH, RAISED, 10.02)
    level = ff.level
    ff.torques(RAISED, FRESH, RAISED, 5.0)
    assert ff.level == level


def full_feedforward(scale=1.0):
    ff = GravityFeedforward(scale)
    ff.level = 1.0
    return ff


def test_measured_pose_is_used_while_every_joint_is_fresh():
    goal = [q + 20 for q in RAISED]
    ages = [FRESH_S] * 6
    assert full_feedforward().torques(RAISED, ages, goal, 0.0) == pytest.approx(gravity_nm(RAISED))


@pytest.mark.parametrize("stale_joint", range(6))
def test_one_stale_joint_falls_back_to_the_goal(stale_joint):
    goal = [q + 20 for q in RAISED]
    ages = [0.0] * 6
    ages[stale_joint] = FRESH_S + 0.001
    assert full_feedforward().torques(RAISED, ages, goal, 0.0) == pytest.approx(gravity_nm(goal))


def test_non_finite_measurement_falls_back_to_the_goal():
    measured = [math.nan, *RAISED[1:]]
    assert full_feedforward().torques(measured, FRESH, RAISED, 0.0) == pytest.approx(gravity_nm(RAISED))


def test_non_finite_pose_everywhere_gives_zero_torque():
    bad = [math.nan] * 6
    assert full_feedforward().torques(bad, FRESH, bad, 0.0) == [0.0] * 6


def test_cap_is_per_joint_and_both_signs(monkeypatch):
    monkeypatch.setattr(gravity, "gravity_nm", lambda q, scale: [100.0, -100.0, 100.0, -100.0, 100.0, -100.0])
    tau = full_feedforward().torques(RAISED, FRESH, RAISED, 0.0)
    assert tau == [LIMIT_NM[0], -LIMIT_NM[1], LIMIT_NM[2], -LIMIT_NM[3], LIMIT_NM[4], -LIMIT_NM[5]]


def test_a_single_non_finite_torque_becomes_zero(monkeypatch):
    monkeypatch.setattr(gravity, "gravity_nm", lambda q, scale: [1.0, math.nan, math.inf, 0.1, 0.1, 0.1])
    assert full_feedforward().torques(RAISED, FRESH, RAISED, 0.0) == [0.5, 0.0, 0.0, 0.1, 0.1, 0.1]


def test_cap_is_not_reached_inside_the_soft_limits():
    config = ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    rng = random.Random(20261103)
    worst = [0.0] * 6
    for _ in range(5000):
        q = [math.degrees(rng.uniform(j.lo, j.hi)) for j in config.joints[:6]]
        worst = [max(w, abs(t)) for w, t in zip(worst, gravity_nm(q))]
    assert all(w < limit for w, limit in zip(worst, LIMIT_NM))
    assert worst[1] > 9.0  # the shoulder's model maximum, which the cap has to clear


def test_cap_stays_under_each_motors_torque_range():
    config = ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    for limit, joint in zip(LIMIT_NM, config.joints):
        assert limit < protocol.MOTOR_PARAMS[joint.model].t_max


@pytest.mark.parametrize("scale", [-0.1, 1.21, math.nan])
def test_scale_out_of_range_is_refused(scale):
    with pytest.raises(ValueError, match="gravity_scale"):
        GravityFeedforward(scale)
    with pytest.raises(ValueError, match="gravity_scale"):
        ArmConfig(gravity_scale=scale)._validate()


# ── config ──
def test_scale_is_configurable_from_yaml(tmp_path):
    path = tmp_path / "arm.yaml"
    path.write_text("gravity_scale: 0.8\njoints: []\n")
    assert ArmConfig.from_yaml(path).gravity_scale == 0.8
    path.write_text("gravity_scale: 2\njoints: []\n")
    with pytest.raises(ValueError, match="gravity_scale"):
        ArmConfig.from_yaml(path)


def test_shipped_profile_uses_the_default_scale_and_fits_the_model():
    config = ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    assert config.gravity_scale == gravity.DEFAULT_GRAVITY_SCALE
    assert model_applies([j.name for j in config.joints])
    assert config.joints[6].name == "gripper"


def test_cli_override():
    arm = SimpleNamespace(config=SimpleNamespace(gravity_scale=1.0))
    apply_gravity_arg(arm, SimpleNamespace(gravity_scale=None))
    assert arm.config.gravity_scale == 1.0
    apply_gravity_arg(arm, SimpleNamespace(gravity_scale=0.0))
    assert arm.config.gravity_scale == 0.0
    with pytest.raises(SystemExit):
        apply_gravity_arg(arm, SimpleNamespace(gravity_scale=1.5))


# ── the arm's frames ──
def maker(q_deg=RAISED, track=False, config=None, **overrides):
    config = config or ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    config = replace(config, gravity_scale=overrides.pop("gravity_scale", 1.0), **overrides)
    raw = [math.radians(q) for q in q_deg] + [-0.5]
    motors = [FakeMotor(j.motor_id, protocol.MOTOR_PARAMS[j.model], position=raw[i], track=track)
              for i, j in enumerate(config.joints)]
    arm = Arm(config, FakeBus(motors))
    arm.connect()
    return arm, motors


def ticking(**kwargs):
    arm, motors = maker(**kwargs)
    arm.enable(start_loop=False)
    if arm._gravity is not None:
        arm._gravity.level = 1.0
    arm._tick(0.005)
    return arm, motors


def sent_tau(motors):
    return [m.frames[-1][2] for m in motors]


def teardown(arm):
    arm.disable()
    arm.disconnect()


def test_tau_reaches_the_frames_in_motor_sign():
    arm, motors = ticking()
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(RAISED), abs=ONE_LSB)
    teardown(arm)


def test_gripper_frames_carry_no_gravity():
    arm, motors = ticking()
    arm._tick(0.005)
    assert motors[6].frames[-1][1] > 0
    assert abs(sent_tau(motors)[6]) < ONE_LSB
    teardown(arm)


def test_first_frame_after_enable_starts_from_zero():
    arm, motors = maker()
    arm.enable(start_loop=False)
    arm._tick(0.005)
    assert motors[2].frames[-1][1] > 0
    assert sent_tau(motors[:6]) == pytest.approx([0.0] * 6, abs=ONE_LSB)
    time.sleep(RAMP_STEP_S * 3)
    arm._tick(0.005)
    fraction = RAMP_STEP_S / RAMP_S
    assert sent_tau(motors[:6]) == pytest.approx([t * fraction for t in gravity_nm(RAISED)], abs=ONE_LSB)
    teardown(arm)


def test_every_enable_starts_the_ease_in_again():
    arm, motors = ticking()
    arm.disable()
    arm.enable(start_loop=False)
    arm._tick(0.005)
    assert sent_tau(motors[:6]) == pytest.approx([0.0] * 6, abs=ONE_LSB)
    teardown(arm)


def test_zero_gain_frames_stay_at_zero_torque():
    arm, motors = ticking()
    arm.disable()
    for motor in motors:
        zero_gain = [frame for frame in motor.frames if frame[1] == 0.0]
        assert len(zero_gain) >= 2  # the enable preload and the release
        assert all(abs(frame[2]) < ONE_LSB for frame in zero_gain)
    arm.disconnect()


def test_measured_pose_then_goal_when_a_reply_is_stale():
    arm, motors = maker()
    arm.enable(start_loop=False)
    arm._gravity.level = 1.0
    goal_deg = [q + 5 for q in RAISED]
    with arm._target_lock:
        arm._user_targets[:6] = arm._internal[:6] = [math.radians(q) for q in goal_deg]
    arm._tick(0.005)
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(RAISED), abs=ONE_LSB)
    arm.motors[2]._fb_time = time.monotonic() - (FRESH_S + 0.02)
    arm._tick(0.005)
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(goal_deg), abs=ONE_LSB)
    arm._tick(0.005)  # the stale joint replied to the last frame
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(RAISED), abs=ONE_LSB)
    teardown(arm)


def test_a_flipped_joint_direction_keeps_motor_sign():
    config = ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    joints = list(config.joints)
    joints[1] = replace(joints[1], direction=-1, offset=0.1, lo=0.0, hi=3.0)
    joints[2] = replace(joints[2], direction=-1, offset=-0.2, lo=-5.0, hi=0.0)
    arm, motors = ticking(config=replace(config, joints=joints))
    assert arm.get_joint_positions()[1] > 0 > arm.get_joint_positions()[2]
    expected = gravity_nm(RAISED)
    assert expected[1] > 0 and expected[2] > 0
    assert sent_tau(motors[:6]) == pytest.approx(expected, abs=ONE_LSB)
    # the frame's goal is in motor sign too: stale fallback and position agree with the raw pose
    arm.motors[0]._fb_time = time.monotonic() - (FRESH_S + 0.02)
    arm._tick(0.005)
    assert sent_tau(motors[:6]) == pytest.approx(expected, abs=ONE_LSB)
    assert [m.frames[-1][0] for m in motors[:6]] == pytest.approx([math.radians(q) for q in RAISED], abs=1e-3)
    teardown(arm)


def test_a_full_turn_correction_does_not_change_the_torque():
    q = [RAISED[0] + 360, *RAISED[1:]]
    arm, motors = ticking(q_deg=q)
    assert math.degrees(arm.get_joint_positions()[0]) == pytest.approx(RAISED[0], abs=0.05)
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(RAISED), abs=ONE_LSB)
    assert motors[0].frames[-1][0] == pytest.approx(math.radians(q[0]), abs=1e-3)
    teardown(arm)


def test_scale_reaches_the_frames_and_zero_turns_it_off():
    arm, motors = ticking(gravity_scale=1.2)
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(RAISED, 1.2), abs=ONE_LSB)
    teardown(arm)
    arm, motors = ticking(gravity_scale=0.0)
    assert sent_tau(motors) == pytest.approx([0.0] * 7, abs=ONE_LSB)
    teardown(arm)


def test_a_scale_set_after_construction_is_checked_before_any_frame():
    arm, motors = maker()
    arm.config.gravity_scale = 5.0
    with pytest.raises(ValueError, match="gravity_scale"):
        arm.enable(start_loop=False)
    assert all("enable" not in m.log and not m.frames for m in motors)
    arm.disconnect()


def test_other_arms_get_no_gravity(caplog):
    motors = [FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5)]
    arm = Arm(two_joint_config(), FakeBus(motors))
    arm.connect()
    with caplog.at_level(logging.WARNING, logger="maker_arm"):
        arm.enable(start_loop=False)
    assert arm._gravity is None
    assert sum("gravity feed-forward skipped" in r.getMessage() for r in caplog.records) == 1
    arm._tick(0.005)
    assert sent_tau(motors) == pytest.approx([0.0, 0.0], abs=ONE_LSB)
    teardown(arm)


def test_no_skip_warning_when_the_model_applies_or_gravity_is_off(caplog):
    with caplog.at_level(logging.WARNING, logger="maker_arm"):
        teardown(ticking()[0])
        motors = [FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5)]
        arm = Arm(two_joint_config(gravity_scale=0.0), FakeBus(motors))
        arm.connect()
        arm.enable(start_loop=False)
        teardown(arm)
    assert not any("gravity feed-forward skipped" in r.getMessage() for r in caplog.records)


def test_a_base_only_config_gets_no_gravity():
    full = ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    config = replace(full, required_motor_ids=None, joints=full.joints[:1])
    motors = [FakeMotor(1, position=0.1)]
    arm = Arm(config, FakeBus(motors))
    arm.connect()
    arm.enable(start_loop=False)
    arm._tick(0.005)
    assert motors[0].frames[-1][1] > 0 and abs(motors[0].frames[-1][2]) < ONE_LSB
    teardown(arm)


def test_a_model_failure_keeps_the_hold_frames_going(monkeypatch):
    arm, motors = maker()
    arm.enable(start_loop=False)

    def boom(q, scale):
        raise RuntimeError("boom")

    monkeypatch.setattr(gravity, "gravity_nm", boom)
    before = len(motors[2].frames)
    arm._tick(0.005)
    arm._tick(0.005)
    assert len(motors[2].frames) == before + 2 and motors[2].frames[-1][1] > 0
    assert sent_tau(motors) == pytest.approx([0.0] * 7, abs=ONE_LSB)
    assert arm.state is ArmState.ENABLED
    teardown(arm)


def test_hold_current_position_carries_gravity():
    arm, motors = ticking()
    assert arm.hold_current_position()
    arm._tick(0.005)
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(RAISED), abs=ONE_LSB)
    teardown(arm)


def test_a_fault_hold_carries_gravity():
    arm, motors = ticking()
    arm._enter_fault("test fault", 2)
    assert arm.state is ArmState.FAULT
    arm._tick(0.005)
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(RAISED), abs=ONE_LSB)
    teardown(arm)


def test_a_released_gripper_leaves_the_arm_carrying_gravity():
    arm, motors = ticking()
    frames = len(motors[6].frames)
    arm._enter_fault("gripper fault", 6)
    arm._tick(0.005)
    assert len(motors[6].frames) == frames and not motors[6].enabled
    assert sent_tau(motors[:6]) == pytest.approx(gravity_nm(RAISED), abs=ONE_LSB)
    teardown(arm)


def wait(predicate, seconds=3.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_control_loop_eases_in_and_the_return_to_rest_carries_gravity():
    arm, motors = maker(track=True)
    arm.enable()
    assert wait(lambda: arm._gravity.level == 1.0)
    assert wait(lambda: abs(motors[2].frames[-1][2] - gravity_nm(RAISED)[2]) < ONE_LSB)
    elbow = [frame[2] for frame in motors[2].frames if frame[1] > 0]
    assert abs(elbow[0]) < ONE_LSB
    assert all(b >= a - ONE_LSB for a, b in zip(elbow, elbow[1:]))

    moved = list(arm.get_joint_positions())
    moved[2] += math.radians(2.0)
    assert arm.set_joint_targets(moved)
    assert wait(lambda: abs(motors[2].position - moved[2]) < 1e-3)
    before_stop = len(motors[2].frames)
    assert arm.stop()
    assert arm.state is ArmState.CONNECTED

    for motor in motors[:6]:
        assert motor.frames[-1][1] == 0.0 and abs(motor.frames[-1][2]) < ONE_LSB
    held = [frame for frame in motors[2].frames[before_stop:] if frame[1] > 0]
    assert held
    for position, _, tau in held:
        pose = list(RAISED)
        pose[2] = math.degrees(position)
        assert tau == pytest.approx(gravity_nm(pose)[2], abs=0.05)
    arm.disconnect()
