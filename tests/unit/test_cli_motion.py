"""Powered commands on simulated motors: hold, move, gravity, zero, set-id, and their gates."""

import json
import math
import sys
from dataclasses import replace

import pytest

from conftest import sdk
from fakes import FakeBus, two_joint_config
from test_cli_bench import POSE, PROFILE, Bench, gs  # noqa: F401  (Bench used by the fixture)


@pytest.fixture
def bench(monkeypatch, capsys, tmp_path):
    return Bench(monkeypatch, capsys, tmp_path)


def enables(motors):
    return sum(m.log.count("enable") for m in motors.values())


# ── gates ──
@pytest.mark.parametrize("argv", [
    ["hold", "--for", "0.1"], ["move", "--pose", "zero"], ["gravity", "--for", "0.1"], ["check"],
    ["check", "--sine", "5"], ["set-id", "--from", "7", "--to", "8"], ["teleop", "--star-port", "/dev/x"],
    ["identify", "--no-cameras", "--channel", "can0"], ["release"],
])
def test_energizing_commands_refuse_without_yes_off_a_terminal(bench, argv):
    motors = bench.add_arm()
    doc = bench.json(*argv, code=1)
    assert doc["ok"] is False and "refusing without --yes" in doc["error"]
    assert enables(motors) == 0


def test_zero_needs_its_own_flag_even_with_a_terminal_flag(bench):
    motors = bench.add_arm(enabled=True)   # an arm left energized
    code, _, err = bench.run("zero", "--all", "--yes")
    assert code == 2 and "unrecognized arguments: --yes" in err
    doc = bench.json("zero", "--all", code=1)
    assert "refusing without --yes" in doc["error"]
    assert not any("zero" in m.log for m in motors.values())
    # refused before any torque-off: the arm was not dropped
    assert all(m.enabled and "disable" not in m.log for m in motors.values())


def test_stay_is_refused_before_energizing_when_a_stiff_joint_has_a_watchdog():
    joints = [replace(j, can_timeout_ms=200) for j in two_joint_config().joints]
    arm = sdk.arm.Arm(two_joint_config(joints=joints), FakeBus([]))
    with pytest.raises(sdk.cli.common.CliError, match="would drop j1"):
        sdk.cli.common.check_stay([arm], "stay")
    sdk.cli.common.check_stay([arm], "rest")
    sdk.cli.common.check_stay([sdk.arm.Arm(two_joint_config(), FakeBus([]))], "stay")   # gripper only: allowed


def test_unverified_return_leaves_the_arm_holding_and_fails(bench, monkeypatch):
    motors = bench.add_arm()
    monkeypatch.setattr(sdk.arm.Arm, "stop", lambda self: (self.hold_current_position(), False)[1])
    code, out, err = bench.run("hold", "--for", "0.1", "--yes", "--json")
    doc = json.loads(out)
    assert code == 1 and doc["ok"] is False and doc["then"] == {"arm": "holding"}
    assert all(m.enabled and "disable" not in m.log for m in motors.values())
    assert "left holding with torque on" in err and "release" in err


def test_typed_release_after_an_unverified_return_is_reported_as_released(bench, monkeypatch):
    motors = bench.add_arm()
    monkeypatch.setattr(sdk.arm.Arm, "stop", lambda self: (self.hold_current_position(), False)[1])
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True, raising=False)
    monkeypatch.setattr("builtins.input", lambda *_: "RELEASE")
    doc = bench.json("hold", "--for", "0.1", "--yes", code=1)
    assert doc["then"] == {"arm": "released"} and not any(m.enabled for m in motors.values())


# ── hold ──
def test_hold_then_rest(bench):
    motors = bench.add_arm()
    doc = bench.json("hold", "--for", "0.2", "--yes")
    assert doc["stopped_by"] == "done" and doc["then"] == {"arm": "rest"} and doc["faults"] == {}
    assert enables(motors) == 7 and not any(m.enabled for m in motors.values())


def test_hold_two_arms_in_parallel_then_stay(bench):
    left, right = bench.add_arm("gs_usb:AAA"), bench.add_arm("gs_usb:BBB")
    bench.write_rig({"left": gs("AAA"), "right": gs("BBB")})
    doc = bench.json("hold", "--arm", "all", "--for", "0.1", "--then", "stay", "--yes")
    assert doc["then"] == {"left": "stay", "right": "stay"}
    assert all(m.enabled for m in [*left.values(), *right.values()])   # left energized on purpose
    doc = bench.json("release", "--arm", "left", "--arm", "right", "--yes")
    assert not any(m.enabled for m in [*left.values(), *right.values()])


def test_hold_stops_on_a_fault_and_reports_it(bench):
    motors = bench.add_arm()
    motors["wrist_roll"].online = False

    doc = bench.json("hold", "--yes", code=1)   # connect fails: never enabled
    assert "no reply from motors [6]" in doc["error"] and enables(motors) == 0


# ── move ──
def test_move_joint_by_and_to(bench):
    motors = bench.add_arm()
    doc = bench.json("move", "--joint", "elbow_flex", "--by", "10", "--joint", "wrist_flex", "--to", "5",
                     "--then", "rest", "--yes")
    assert doc["reached"] is True and doc["then"] == "rest"
    assert doc["goal"]["elbow_flex"] == pytest.approx(1.0 + math.radians(10), abs=1e-3)
    assert doc["goal"]["wrist_flex"] == pytest.approx(math.radians(5))
    assert abs(doc["error"]["elbow_flex"]) < math.radians(2.5)
    # back at the enable pose and released
    assert motors["elbow_flex"].position == pytest.approx(1.0, abs=math.radians(2.5))
    assert not any(m.enabled for m in motors.values())


def test_move_pose_names_and_clipping(bench):
    bench.add_arm(pose=[2.6, *POSE[1:]])
    doc = bench.json("move", "--pose", "shoulder_lift=-60,shoulder_pan=500", "--then", "rest", "--yes")
    upper = sdk.config.ArmConfig.from_yaml(str(PROFILE)).joints[0].hi
    assert doc["goal"]["shoulder_lift"] == pytest.approx(math.radians(-60))
    assert doc["goal"]["shoulder_pan"] == pytest.approx(upper)   # clipped to the limit


def test_move_argument_errors(bench):
    bench.add_arm()
    assert "exactly one" in bench.json("move", "--yes", code=1)["error"]
    assert "no joint 'elbow'" in bench.json("move", "--joint", "elbow", "--to", "1", "--yes", code=1)["error"]
    assert "needs --to DEG or --by DEG" in bench.json("move", "--joint", "elbow_flex", "--yes", code=1)["error"]
    code, _, err = bench.run("move", "--to", "5")
    assert code == 2 and "needs a --joint NAME before it" in err


def test_move_then_hold_until_time_then_rest(bench):
    motors = bench.add_arm()
    doc = bench.json("move", "--joint", "wrist_flex", "--by", "-5", "--for", "0.1", "--yes")
    assert doc["reached"] and doc["then"] == "rest" and not any(m.enabled for m in motors.values())


def test_move_cartesian_uses_move_ee(bench, monkeypatch):
    bench.add_arm()
    calls = []

    def move_ee(self, xyz, rpy=None, speed=None, timeout=None, wait=True, cancel=None):
        assert cancel is not None and not cancel.is_set()   # Ctrl-C reaches the blocking move
        calls.append((list(xyz), rpy, speed))
        return True

    monkeypatch.setattr(sdk.arm.Arm, "move_ee", move_ee, raising=False)
    monkeypatch.setattr(sdk.arm.Arm, "ee_pose", lambda self: ([0.2, 0.0, 0.1], [0.0, 0.0, 0.5]), raising=False)
    doc = bench.json("move", "--dxyz", "0", "0", "0.05", "--speed", "30", "--then", "rest", "--yes")
    assert calls[0][0] == pytest.approx([0.2, 0.0, 0.15]) and calls[0][1] == [0.0, 0.0, 0.5]
    assert calls[0][2] == pytest.approx(math.radians(30))
    assert doc["ee_target"]["xyz"] == pytest.approx([0.2, 0.0, 0.15])
    bench.json("move", "--xyz", "0.3", "0", "0.2", "--then", "rest", "--yes")
    assert calls[1][0] == [0.3, 0.0, 0.2] and calls[1][1] is None


def test_move_pose_rest_uses_the_profile_rest_pose(bench, tmp_path):
    bench.add_arm()
    rest = [0.0, -50.0, 60.0, 10.0, 0.0, 0.0, 0.0]
    profile = tmp_path / "with_rest.yaml"
    profile.write_text(PROFILE.read_text() + f"rest_pose_deg: {rest}\n")
    doc = bench.json("move", "--pose", "rest", "--config", profile, "--then", "rest", "--yes")
    assert doc["goal"]["shoulder_lift"] == pytest.approx(math.radians(-50))
    assert doc["goal"]["elbow_flex"] == pytest.approx(math.radians(60))


# ── gravity ──
def test_gravity_float_then_rest(bench, monkeypatch):
    motors = bench.add_arm()
    floats = []
    original = sdk.arm.Arm.float_mode
    monkeypatch.setattr(sdk.arm.Arm, "float_mode", lambda self, **kw: (floats.append(kw), original(self, **kw)))
    doc = bench.json("gravity", "--for", "0.2", "--kd", "0.2", "--yes")
    assert floats == [{"kd": 0.2}] and doc["then"] == {"arm": "rest"}
    # floating (kp 0) frames went out, then the profile gains came back before the release
    kps = [kp for _, kp, _ in motors["elbow_flex"].frames]
    assert 0.0 in kps and kps.index(0.0) < len(kps) - 1 and max(kps[kps.index(0.0):]) > 0
    assert not any(m.enabled for m in motors.values())


def test_gravity_check_compares_with_the_model(bench, monkeypatch):
    bench.add_arm()
    model = lambda q: [0.1 * (i + 1) for i in range(6)]   # noqa: E731
    monkeypatch.setattr(sdk.cli.gravity, "model_for", lambda config: model)
    doc = bench.json("gravity", "--check", "--seconds", "0.2", "--settle", "0", "--yes")
    assert doc["samples"] >= 2 and doc["then"] == "rest"
    elbow = doc["joints"]["elbow_flex"]
    assert elbow["model"] == pytest.approx(0.3) and elbow["difference"] == pytest.approx(elbow["measured"] - 0.3)
    assert doc["joints"]["gripper"]["model"] is None


def test_gravity_check_without_a_model(bench, monkeypatch):
    bench.add_arm()
    monkeypatch.setattr(sdk.cli.gravity, "model_for", lambda config: None)
    assert "no gravity model" in bench.json("gravity", "--check", "--yes", code=1)["error"]


# ── zero / set-id ──
def test_zero_all_with_the_explicit_flag(bench):
    motors = bench.add_arm()
    doc = bench.json("zero", "--all", "--yes-i-am-at-the-zero-pose")
    assert doc["zeroed"] == list(motors)
    assert all(m.log.count("zero") == 1 for m in motors.values())
    assert all(abs(v) < 1e-3 for v in doc["post_zero"].values())
    assert not any(m.enabled for m in motors.values())


def test_zero_one_motor(bench):
    motors = bench.add_arm()
    doc = bench.json("zero", "--motor", "7", "--yes-i-am-at-the-zero-pose")
    assert doc["zeroed"] == ["gripper"] and motors["gripper"].log.count("zero") == 1
    assert motors["elbow_flex"].log.count("zero") == 0
    assert "not in the profile" in bench.json("zero", "--motor", "42", "--yes-i-am-at-the-zero-pose", code=1)["error"]


def test_set_id_goes_through_the_arm_and_reports_its_error(bench, monkeypatch):
    bench.add_arm()
    calls = []
    monkeypatch.setattr(sdk.arm.Arm, "set_motor_id", lambda self, old, new, timeout=0.5: calls.append((old, new)))
    doc = bench.json("set-id", "--from", "7", "--to", "9", "--yes")
    assert calls == [(7, 9)] and doc["to"] == 9
    assert "must differ" in bench.json("set-id", "--from", "7", "--to", "7", "--yes", code=1)["error"]
    monkeypatch.setattr(sdk.arm.Arm, "set_motor_id",
                        lambda self, old, new, timeout=0.5: (_ for _ in ()).throw(sdk.errors.ArmError("unsupported")))
    assert bench.json("set-id", "--from", "7", "--to", "9", "--yes", code=1)["error"] == "unsupported"


def test_hold_then_release_drops_torque_where_it_is(bench):
    motors = bench.add_arm()
    doc = bench.json("hold", "--for", "0.1", "--then", "release", "--yes")
    assert doc["then"] == {"arm": "released"} and not any(m.enabled for m in motors.values())


def test_slcan_arms_run_at_the_verified_serial_rate(bench, monkeypatch):
    common = sdk.cli.common
    spec = sdk.rig.ArmSpec("slcan", port="/dev/ttyACM9")
    assert common.Target("arm", spec).build().config.control_rate_hz == 25.0
    assert common.Target("arm", sdk.rig.ArmSpec("socketcan")).build().config.control_rate_hz > 25.0
