"""Rig file: load/save round trips, validation, lookups, and ArmGroup on simulated arms."""

import threading
from dataclasses import replace
from pathlib import Path

import pytest

from conftest import sdk
from fakes import FakeBus, FakeMotor, make_arm, two_joint_config

rig_module = sdk.rig
Rig, ArmSpec, LeaderSpec, RigError, ArmGroup = (rig_module.Rig, rig_module.ArmSpec, rig_module.LeaderSpec,
                                                 rig_module.RigError, rig_module.ArmGroup)


@pytest.fixture(autouse=True)
def fast(monkeypatch, tmp_path):
    monkeypatch.setattr(sdk.arm.Arm, "PARAM_TIMEOUT", 0.005)
    monkeypatch.setenv(rig_module.ENV_VAR, str(tmp_path / "default.yaml"))


def sample() -> Rig:
    return Rig({"left": ArmSpec("gs_usb", port="gs_usb:0036"), "right": ArmSpec("socketcan", channel="can1"),
                "spare": ArmSpec("slcan", port="/dev/ttyACM0", profile="spare.yaml", baudrate=921600)},
               {"top": "usb:platform-1.2", "left_wrist": "index:2"},
               {"left": LeaderSpec("star", port="/dev/ttyUSB0", map="maps/star.json"),
                "right": LeaderSpec("arm", arm="spare")})


def test_round_trip(tmp_path):
    path = sample().save(tmp_path / "sub" / "rig.yaml")
    loaded = Rig.load(path)
    assert loaded.arms == sample().arms and loaded.cameras == sample().cameras and loaded.leaders == sample().leaders
    assert loaded.arm_names == ["left", "right", "spare"] and loaded.path == path
    text = path.read_text()
    assert "profile: null" not in text and "backend: gs_usb" in text


def test_default_path_and_missing_file(tmp_path, monkeypatch):
    assert rig_module.default_path() == tmp_path / "default.yaml"
    assert rig_module.ENV_VAR.endswith("_RIG") and rig_module.ENV_VAR == rig_module.ENV_VAR.upper()
    rig = Rig.load()
    assert rig.arms == {} and rig.path == tmp_path / "default.yaml"
    with pytest.raises(RigError, match="no rig file"):
        Rig.load(missing_ok=False)
    monkeypatch.delenv(rig_module.ENV_VAR)
    assert rig_module.default_path() == Path.home() / ".config" / sdk._product.CLI_NAME / "rig.yaml"


@pytest.mark.parametrize("text, message", [
    ("arms: {a: {backend: can}}", "backend must be one of"),
    ("arms: {a: {backend: gs_usb, speed: 1}}", "unknown keys"),
    ("arms: [a, b]", "expected a mapping"),
    ("robots: {}", "expected a mapping with arms"),
    ("arms: {a: {backend: gs_usb}}\nleaders: {b: {type: star, port: /dev/x}}", "no arm named 'b' to follow"),
    ("arms: {a: {backend: gs_usb}}\nleaders: {a: {type: arm, arm: a}}", "cannot lead itself"),
    ("arms: {a: {backend: gs_usb}}\nleaders: {a: {type: arm, arm: z}}", "no arm named 'z'"),
    ("arms: {a: {backend: gs_usb}}\nleaders: {a: {type: star}}", "needs a port"),
    ("arms: {a: {backend: socketcan}, b: {backend: socketcan, channel: can0}}", "same bus can0"),
    ("arms: {all: {backend: gs_usb}}", "cannot name an arm"),
    ("arms: {a: {backend: gs_usb}", "not valid YAML"),
])
def test_validation(tmp_path, text, message):
    path = tmp_path / "rig.yaml"
    path.write_text(text)
    with pytest.raises(RigError, match=message):
        Rig.load(path)


def test_save_refuses_an_invalid_rig(tmp_path):
    rig = sample()
    rig.leaders["ghost"] = LeaderSpec("star", port="/dev/x")
    with pytest.raises(RigError):
        rig.save(tmp_path / "rig.yaml")
    assert not (tmp_path / "rig.yaml").exists()


def test_names_and_lookups(tmp_path):
    rig = sample()
    assert rig.names(["all"]) == ["left", "right", "spare"]
    assert rig.names(["right", "left", "right"]) == ["right", "left"]
    with pytest.raises(RigError, match="arms: left, right, spare"):
        rig.names(["middle"])
    assert rig.camera_id("top") == "usb:platform-1.2" and rig.camera_id("index:5") == "index:5"
    with pytest.raises(RigError, match="cameras: top, left_wrist"):
        rig.camera_id("side")
    assert rig.camera("top").id == "usb:platform-1.2"   # not opened
    assert rig.spec("right").address == "can1" and ArmSpec("gs_usb").address == "gs_usb"


def test_backend_kwargs():
    assert ArmSpec("socketcan").backend_kwargs() == {"channel": "can0"}
    assert ArmSpec("gs_usb").backend_kwargs() == {"port": None}
    kw = ArmSpec("slcan", port="/dev/ttyACM1", baudrate=921600).backend_kwargs()
    assert kw["port"] == "/dev/ttyACM1" and kw["baudrate"] == 921600 and kw["rtscts"] is False


def test_arm_builds_from_the_entry_with_relative_profile(tmp_path, monkeypatch):
    made = []
    monkeypatch.setattr(rig_module, "create_backend", lambda name, **kw: made.append((name, kw)) or FakeBus([]))
    profile = Path(sdk._product.DEFAULT_PROFILE)
    (tmp_path / "mine.yaml").write_text(profile.read_text())
    rig = Rig({"left": ArmSpec("gs_usb", port="gs_usb:X", profile="mine.yaml")}, path=tmp_path / "rig.yaml")
    arm = rig.arm("left")
    assert made == [("gs_usb", {"port": "gs_usb:X"})]
    assert arm.state.name == "DISCONNECTED" and arm.config.n_joints == 7
    rig.arms["left"].profile = "missing.yaml"
    with pytest.raises(OSError):
        rig.arm("left")


def test_leaders(tmp_path, monkeypatch):
    monkeypatch.setattr(rig_module, "create_backend", lambda name, **kw: FakeBus([]))
    rig = sample()
    rig.path = tmp_path / "rig.yaml"
    follower = rig.arm("left")
    with pytest.raises(OSError):   # the map resolves next to the rig file
        rig.leader("left", follower)
    (tmp_path / "maps").mkdir()
    (tmp_path / "maps" / "star.json").write_text(Path(sdk._product.DEFAULT_STAR_MAPPING).read_text())
    star = rig.leader("left", follower)
    assert isinstance(star, sdk.leaders.StarLeader) and star.port == "/dev/ttyUSB0" and star.bus is None
    (tmp_path / "spare.yaml").write_text(Path(sdk._product.DEFAULT_PROFILE).read_text())
    spare = rig.arm("spare")
    leader = rig.leader("right", rig.arm("right"), arms={"spare": spare})
    assert isinstance(leader, sdk.leaders.ArmLeader) and leader.arm is spare
    with pytest.raises(RigError, match="no leader for arm 'spare'"):
        rig.leader("spare", spare)


# ── ArmGroup ──
def two_arms():
    a, _, fa = make_arm()
    b, _, fb = make_arm()
    return {"a": a, "b": b}, {**{f"a{k}": m for k, m in fa.items()}, **{f"b{k}": m for k, m in fb.items()}}


def test_group_connects_enables_and_stops_in_parallel(monkeypatch):
    arms, motors = two_arms()
    threads = set()
    original = sdk.arm.Arm.enable

    def enable(self, *args, **kwargs):
        threads.add(threading.current_thread().name)
        return original(self, *args, **kwargs)

    monkeypatch.setattr(sdk.arm.Arm, "enable", enable)
    with ArmGroup(arms) as group:
        group.enable()
        assert all(arm.state.name == "ENABLED" for arm in arms.values())
        assert len(group) == 2 and group["a"] is arms["a"]
    assert threads == {"arm-a", "arm-b"}
    assert all(arm.state.name == "DISCONNECTED" for arm in arms.values())
    assert not any(m.enabled for m in motors.values())


def test_group_enable_failure_stops_the_others():
    a, _, fa = make_arm()
    b, _, fb = make_arm([FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5, temperature=50.0)])
    group = ArmGroup({"a": a, "b": b}, parallel=False)
    group.connect()
    with pytest.raises(sdk.errors.ArmError, match="enable needs it below"):
        group.enable()
    assert a.state.name == "CONNECTED" and not any(m.enabled for m in fa.values())
    group.disconnect()


def test_group_connect_failure_disconnects_everything():
    a, bus_a, _ = make_arm()
    b, _, _ = make_arm([FakeMotor(1, position=0.3), FakeMotor(2, online=False)])
    with pytest.raises(sdk.errors.ArmError, match="no reply"):
        ArmGroup([a, b]).connect()
    assert a.state.name == "DISCONNECTED" and not bus_a.is_open


def test_rig_group_runs_in_parallel_only_on_separate_buses(monkeypatch):
    monkeypatch.setattr(rig_module, "create_backend", lambda name, **kw: FakeBus([]))
    rig = Rig({"a": ArmSpec("gs_usb", port="gs_usb:1"), "b": ArmSpec("gs_usb", port="gs_usb:2")})
    assert rig.group().parallel is True and list(rig.group(["b"]).arms) == ["b"]
    rig = Rig({"a": ArmSpec("gs_usb"), "b": ArmSpec("gs_usb")})   # both "the only adapter"
    assert rig.group().parallel is False


def test_rig_module_is_product_neutral():
    product = sdk._product
    text = Path(rig_module.__file__).read_text().lower()
    names = {product.PACKAGE, product.CLI_NAME, product.DISPLAY_NAME.lower(), product.DISPLAY_NAME.split()[0].lower(),
             product.MOTOR_VENDOR.lower()}
    assert not [n for n in names if n.lower() in text]
    for path in Path(sdk.cli.__file__).parent.glob("*.py"):
        assert not [n for n in names if n.lower() in path.read_text().lower()], path.name
    for path in Path(__file__).parent.glob("test_cli*.py"):
        assert not [n for n in names if n.lower() in path.read_text().lower()], path.name


def test_group_exit_never_releases_an_arm_whose_return_was_not_verified(monkeypatch):
    a, _, fa = make_arm([FakeMotor(1, position=0.3, params_supported=True),
                         FakeMotor(2, position=-0.5, params_supported=True)])
    monkeypatch.setattr(a, "stop", lambda: False)
    with ArmGroup({"a": a}):
        a.enable()
    # only the gripper (release_on_fault) has a watchdog: detached, still holding
    assert a.state.name == "DISCONNECTED" and all(m.enabled for m in fa.values())
    assert not any("disable" in m.log for m in fa.values())


def test_group_exit_keeps_holding_when_a_stiff_joint_would_drop_on_its_watchdog(monkeypatch):
    joints = [replace(j, can_timeout_ms=200) for j in two_joint_config().joints]
    a, _, fa = make_arm([FakeMotor(1, position=0.3, params_supported=True),
                         FakeMotor(2, position=-0.5, params_supported=True)], joints=joints)
    monkeypatch.setattr(a, "stop", lambda: False)
    with ArmGroup({"a": a}):
        a.enable()
    try:
        assert a.state.name == "ENABLED" and a._running and fa[1].enabled
    finally:
        a.disable()


def test_group_close_reports_each_arm(monkeypatch):
    arms, motors = two_arms()
    group = ArmGroup(arms)
    group.connect()
    group.enable()
    monkeypatch.setattr(arms["b"], "stop", lambda: False)
    assert group.close() == {"a": "released", "b": "detached"}
    assert arms["a"].state.name == arms["b"].state.name == "DISCONNECTED"
    assert not any(motors[f"a{i}"].enabled for i in (1, 2)) and motors["b1"].enabled


def test_group_close_reports_an_arm_left_energized_by_an_earlier_process():
    a, _, fa = make_arm([FakeMotor(1, position=0.3, params_supported=True, enabled=True),
                         FakeMotor(2, position=-0.5, params_supported=True, enabled=True)])
    b, _, _ = make_arm([FakeMotor(1, position=0.3, params_supported=True),
                        FakeMotor(2, position=-0.5, params_supported=True)])
    group = ArmGroup({"a": a, "b": b})
    group.connect()
    assert group.close() == {"a": "energized", "b": "released"}
    assert fa[1].enabled and fa[2].enabled


def test_group_close_reports_an_unconfirmed_release(monkeypatch):
    arms, motors = two_arms()
    group = ArmGroup(arms)
    group.connect()
    group.enable()
    motors["a2"].online = False
    monkeypatch.setattr(arms["a"], "_return_to_rest", lambda rest=None: True)
    monkeypatch.setattr(arms["a"], "_settle_onto_support", lambda token: True)
    assert group.close() == {"a": "unknown", "b": "released"}
    assert ArmGroup(arms).close() == {"a": "unknown", "b": "unknown"}   # never connected: nothing confirmed
