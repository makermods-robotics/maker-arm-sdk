"""teleop (Star and arm leaders, bimanual), leader zero/read, identify and cameras, with fakes for
the leader, the cameras and the motors."""

import math
import time

import numpy as np
import pytest

from conftest import sdk
from test_cli_bench import POSE, Bench, gs

rig_module = sdk.rig


class FakeStar:
    """StarLeader by duck typing: reads ``targets`` (follower joints, rad)."""
    name = "star"
    instances: list = []

    def __init__(self, port, mapping=None, **_):
        self.port, self.mapping = port, mapping
        self.targets = list(POSE)
        self.servo_ids = list(range(7))
        self.calls = []
        FakeStar.instances.append(self)

    def connect(self, cancel=None):
        self.calls.append("connect")

    def read(self):
        return list(self.targets)

    def close(self):
        self.calls.append("close")

    def zero(self):
        self.calls.append("zero")


@pytest.fixture
def bench(monkeypatch, capsys, tmp_path):
    FakeStar.instances = []
    for module in (sdk.cli.teleop, sdk.cli.leader, sdk.cli.zero, rig_module):
        monkeypatch.setattr(module, "StarLeader", FakeStar)
    return Bench(monkeypatch, capsys, tmp_path)


# ── teleop ──
def test_teleop_star_follows_and_returns_to_rest(bench):
    motors = bench.add_arm()
    doc = bench.json("teleop", "--star-port", "/dev/ttyUSB0", "--for", "0.4", "--rate", "50", "--yes")
    star = FakeStar.instances[-1]
    assert star.port == "/dev/ttyUSB0" and star.calls == ["connect", "close"]
    assert doc["stopped"] == {"arm": "stopped"} and doc["then"] == {"arm": "rest"}
    assert not any(m.enabled for m in motors.values())


def test_teleop_star_moves_the_follower(bench, monkeypatch):
    motors = bench.add_arm()

    class Leading(FakeStar):
        def read(self):
            return [*POSE[:2], 1.1, *POSE[3:]]

    monkeypatch.setattr(sdk.cli.teleop, "StarLeader", Leading)
    seen = []
    original = sdk.arm.Arm.set_joint_targets
    monkeypatch.setattr(sdk.arm.Arm, "set_joint_targets", lambda self, t: (seen.append(list(t)), original(self, t))[1])
    bench.json("teleop", "--star-port", "/dev/x", "--for", "0.5", "--yes")
    assert seen and seen[-1][2] == pytest.approx(1.1, abs=1e-6)
    assert any(abs(f[0] - 1.1) < 1e-3 for f in motors["elbow_flex"].frames)


def test_teleop_cancel_while_connecting_a_leader_stops_before_the_next(bench, monkeypatch):
    left, right = bench.add_arm("gs_usb:AAA"), bench.add_arm("gs_usb:BBB")
    bench.write_rig({"left": gs("AAA"), "right": gs("BBB")})

    def connect(self, cancel=None):
        self.calls.append("connect")
        cancel.set()   # Ctrl-C while this leader connects

    monkeypatch.setattr(FakeStar, "connect", connect)
    doc = bench.json("teleop", "--leader", "star", "--star-port", "/dev/a", "--arm", "left",
                     "--star-port", "/dev/b", "--arm", "right", "--yes", code=1)
    assert doc["error"] == "interrupted"
    assert ["connect" in s.calls for s in FakeStar.instances] == [True, False]
    assert not any("enable" in m.log for m in [*left.values(), *right.values()])


def test_teleop_pairs_must_match(bench):
    bench.add_arm("gs_usb:AAA")
    bench.add_arm("gs_usb:BBB")
    bench.write_rig({"left": gs("AAA"), "right": gs("BBB")})
    doc = bench.json("teleop", "--arm", "all", "--star-port", "/dev/x", "--yes", code=1)
    assert "2 follower(s) but 1 --star-port" in doc["error"]
    doc = bench.json("teleop", "--arm", "left", "--leader", "arm", "--leader-arm", "left", "--yes", code=1)
    assert "cannot lead and follow" in doc["error"]
    assert "no leader for arm 'left'" in bench.json("teleop", "--arm", "left", "--yes", code=1)["error"]


def test_teleop_bimanual_from_the_rig_with_an_arm_leader(bench):
    left, right, spare = bench.add_arm("gs_usb:AAA"), bench.add_arm("gs_usb:BBB"), bench.add_arm("gs_usb:CCC")
    bench.write_rig({"left": gs("AAA"), "right": gs("BBB"), "spare": gs("CCC")},
                    leaders={"left": rig_module.LeaderSpec("star", port="/dev/ttyUSB0"),
                             "right": rig_module.LeaderSpec("arm", arm="spare")})
    doc = bench.json("teleop", "--arm", "left", "--arm", "right", "--for", "0.4", "--yes")
    assert doc["stopped"] == {"left": "stopped", "right": "stopped"}
    assert set(doc["then"]) == {"left", "right", "leader 1"}
    # the leader arm floated (kp 0 frames), then everything was released
    assert any(kp == 0.0 for _, kp, _ in spare["elbow_flex"].frames)
    assert not any(m.enabled for m in [*left.values(), *right.values(), *spare.values()])


def test_teleop_records_motor_states(bench, tmp_path):
    bench.add_arm()
    path = tmp_path / "states.csv"
    bench.json("teleop", "--star-port", "/dev/x", "--for", "0.3", "--record-motor-states", path, "--yes")
    assert path.exists() and path.read_text().startswith("elapsed_s")


# ── leader ──
def test_leader_read_and_zero(bench):
    doc = bench.json("leader", "read", "--star-port", "/dev/x", "--for", "0.2", "--rate", "50")
    assert doc["readings"] >= 1 and doc["targets"] == pytest.approx(POSE)
    assert "refusing without --yes" in bench.json("leader", "zero", "--star-port", "/dev/x", code=1)["error"]
    assert "zero" not in FakeStar.instances[-1].calls
    bench.json("leader", "zero", "--star-port", "/dev/x", "--yes-i-am-at-the-zero-pose")
    assert FakeStar.instances[-1].calls == ["connect", "zero", "close"]


def test_leader_from_the_rig_and_zero_leader_only(bench):
    bench.write_rig({"left": gs("AAA")}, leaders={"left": rig_module.LeaderSpec("star", port="/dev/ttyUSB3")})
    bench.json("leader", "zero", "--arm", "left", "--yes-i-am-at-the-zero-pose")
    assert FakeStar.instances[-1].port == "/dev/ttyUSB3"
    bench.json("zero", "--leader-only", "--star-port", "/dev/ttyUSB4", "--yes-i-am-at-the-zero-pose")
    assert FakeStar.instances[-1].calls == ["connect", "zero", "close"]


# ── cameras ──
class FakeCamera:
    """Camera by duck typing; ``scene(id)`` gives each frame."""
    scene = staticmethod(lambda cam_id: np.full((8, 16), 100.0))

    def __init__(self, cam_id, **_):
        self.id = cam_id

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

    def latest(self, max_age_s=None):
        return self.scene(self.id)

    def read(self, timeout=1.0):
        time.sleep(0.01)
        return self.scene(self.id), time.time() - 0.002

    def snapshot(self, path, max_age_s=0.5):
        path.write_bytes(b"jpeg")
        return path


def test_cameras_list_snapshot_stream(bench, monkeypatch, tmp_path):
    monkeypatch.setattr(rig_module, "Camera", FakeCamera)
    monkeypatch.setattr(sdk.cameras, "list_cameras",
                        lambda: [sdk.cameras.CameraInfo("usb:p-1", 0, "Cam", "/dev/video0")])
    bench.write_rig(cameras={"top": "usb:p-1"})
    doc = bench.json("cameras", "list")
    assert doc["cameras"] == [{"id": "usb:p-1", "index": 0, "name": "Cam", "path": "/dev/video0", "rig_name": "top"}]
    doc = bench.json("cameras", "snapshot", "--out", tmp_path / "shots")
    assert doc["snapshots"] == {"top": str(tmp_path / "shots" / "top.jpg")}
    doc = bench.json("cameras", "stream", "--camera", "index:3", "--seconds", "0.2")
    stats = doc["cameras"]["index:3"]
    assert stats["frames"] >= 2 and stats["fps"] > 5 and 0 < stats["mean_age_s"] < 0.05
    assert "no camera named 'side'" in bench.json("cameras", "snapshot", "--camera", "side", code=1)["error"]


# ── identify ──
def test_identify_names_arms_and_wrist_cameras(bench, monkeypatch):
    a = bench.add_arm("gs_usb:AAA")["gripper"]
    b = bench.add_arm("gs_usb:BBB")["gripper"]
    bench.write_rig({"arm1": gs("AAA"), "arm2": gs("BBB")},
                    cameras={"top": "index:0", "cam_x": "index:1", "cam_y": "index:2"})

    def scene(cam_id):
        level = lambda motor: 100.0 + 200.0 * (motor.position + 0.5)   # noqa: E731
        if cam_id == "index:0":   # A on the left half of the top camera, B on the right
            return np.hstack([np.full((32, 32), level(a)), np.full((32, 32), level(b))])
        return np.full((32, 64), level(b) if cam_id == "index:1" else level(a))

    FakeCamera.scene = staticmethod(scene)
    monkeypatch.setattr(rig_module, "Camera", FakeCamera)
    monkeypatch.setattr(sdk.cli.identify, "STEPS", 4)
    monkeypatch.setattr(sdk.cli.identify, "STEP_S", 0.005)
    doc = bench.json("identify", "--assign", "--yes")
    found = {f["address"]: f for f in doc["arms"]}
    assert found["gs_usb:AAA"]["side"] == "left" and found["gs_usb:AAA"]["wrist_camera"] == "cam_y"
    assert found["gs_usb:BBB"]["side"] == "right" and found["gs_usb:BBB"]["wrist_camera"] == "cam_x"
    assert math.degrees(found["gs_usb:AAA"]["moved"]) == pytest.approx(15.0, abs=0.5)
    assert doc["assigned"] == {"gs_usb:AAA": "left", "gs_usb:BBB": "right"}
    rig = rig_module.Rig.load(bench.rig_path)
    assert rig.arms == {"left": gs("AAA"), "right": gs("BBB")}
    assert rig.cameras["left_wrist"] == "index:2" and rig.cameras["right_wrist"] == "index:1"
    # only the grippers were ever energized, and they are off again
    assert not a.enabled and not b.enabled
    for motors in bench.buses.values():
        assert all("enable" not in m.log for i, m in motors.motors.items() if i != 7)


def test_identify_without_cameras_and_a_dead_adapter(bench, monkeypatch):
    bench.add_arm("gs_usb:AAA")
    bench.write_rig({"arm1": gs("AAA"), "arm2": gs("GONE")})
    monkeypatch.setattr(sdk.cli.identify, "STEPS", 2)
    monkeypatch.setattr(sdk.cli.identify, "STEP_S", 0.005)
    doc = bench.json("identify", "--no-cameras", "--yes", code=1)
    found = {f["candidate"]: f for f in doc["arms"]}
    assert found["arm1"]["released"] and found["arm1"]["side"] is None
    assert "no reply" in found["arm2"]["error"]


class Recording(sdk.cli.common.Interrupt):
    last = None

    def __enter__(self):
        Recording.last = self
        return super().__enter__()


def test_identify_signal_mid_wiggle_releases_and_restores_the_watchdog(bench, monkeypatch):
    motors = bench.add_arm(params_supported=True)
    monkeypatch.setattr(sdk.cli.identify, "Interrupt", Recording)
    monkeypatch.setattr(sdk.cli.identify, "STEPS", 4)
    monkeypatch.setattr(sdk.cli.identify, "STEP_S", 0.005)
    grabs, watchdog = [], []
    for name in ("arm_watchdog", "restore_watchdog"):
        original = getattr(sdk.motor.Motor, name)
        monkeypatch.setattr(sdk.motor.Motor, name,
                            lambda self, *args, _n=name, _o=original, **kw: (watchdog.append((_n, self.motor_id)),
                                                                             _o(self, *args, **kw))[1])

    def grab(cams):
        grabs.append(1)
        if len(grabs) == 2:          # at the peak of the wiggle
            Recording.last.event.set()   # what SIGTERM does
        return {}

    monkeypatch.setattr(sdk.cli.identify, "grab", grab)
    doc = bench.json("identify", "--no-cameras", "--channel", "can0", "--yes", code=1)
    assert "interrupted" in doc["arms"][0]["error"]
    gripper = motors["gripper"]
    assert not gripper.enabled and "disable" in gripper.log
    assert watchdog == [("arm_watchdog", gripper.motor_id), ("restore_watchdog", gripper.motor_id)]


def test_identify_counts_only_motion_that_follows_every_cycle():
    import numpy as np
    judge = sdk.cli.identify.judge
    still = np.zeros((64, 128))
    gripper = still.copy()
    gripper[48:, :32] = 200          # bottom-left, present whenever the gripper is open
    waved = still.copy()
    waved[:16, 96:] = 200            # someone moved on the right during one cycle only
    closed = [{"top": still, "wrist": still, "other": still}] * 3
    opened = [{"top": gripper + waved, "wrist": gripper, "other": waved},
              {"top": gripper, "wrist": gripper, "other": still}]
    verdict = judge(closed, opened, "top", mirror=False)
    assert verdict["side"] == "left" and verdict["wrist_camera"] == "wrist"
