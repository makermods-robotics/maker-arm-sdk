"""A simulated bench for the CLI tests (``Bench``), and the read-only commands: doctor, ports, rig,
scan, status, clear-faults, release, params."""

import json
import sys

import pytest

from conftest import sdk
from fakes import FAULT_TEXT, FakeBus, FakeMotor

cli = sdk.cli.main
rig_module = sdk.rig
PROFILE = sdk._product.DEFAULT_PROFILE
# Motor coordinates inside the shipped profile's limits, the gripper a little open.
_GRIPPER = sdk.config.ArmConfig.from_yaml(str(PROFILE)).joints[-1]
POSE = [0.0, -1.0, 1.0, 0.2, 0.0, 0.0, -0.5 * _GRIPPER.closing_direction]


class Bench:
    """Fake arms keyed by bus address, a temporary rig file, and ``run(*argv)`` for the CLI."""

    def __init__(self, monkeypatch, capsys, tmp_path):
        self.capsys = capsys
        self.tmp = tmp_path
        self.rig_path = tmp_path / "rig.yaml"
        self.buses: dict = {}
        self.opened: list = []
        monkeypatch.setenv(rig_module.ENV_VAR, str(self.rig_path))
        monkeypatch.setattr(rig_module, "create_backend", self.create_backend)
        monkeypatch.setattr(sdk.arm.Arm, "PARAM_TIMEOUT", 0.005)
        load = sdk.config.ArmConfig.from_yaml   # the CLI's arms get the simulated arms' timing
        monkeypatch.setattr(sdk.config.ArmConfig, "from_yaml",
                            classmethod(lambda cls, path: sdk.testing.simulation_timing(load(path))))
        monkeypatch.setattr(sdk.cli.common, "default_backend", lambda: "socketcan")
        monkeypatch.setattr(sys.stdin, "isatty", lambda: False, raising=False)
        monkeypatch.setattr(sdk.cli.ports, "gs_usb_adapters", lambda: (self.adapters(), None))
        monkeypatch.setattr(sdk.cli.ports, "socketcan_interfaces", lambda: [])
        monkeypatch.setattr(sdk.cli.ports, "serial_ports", lambda: [])
        monkeypatch.setattr(sdk.cameras, "list_cameras", lambda: [])

    def adapters(self):
        return [{"serial": a.split(":", 1)[1], "port": a, "usb_path": f"1-{i}", "product": "fake",
                 "manufacturer": "", "vid_pid": "1d50:606f"}
                for i, a in enumerate(self.buses) if a.startswith("gs_usb:")]

    def add_arm(self, address="can0", pose=POSE, special=None, **motor_kw) -> dict:
        """Fake motors for the shipped profile on ``address``; ``special`` maps a joint name to a
        ``factory(motor_id, model, position)`` for that joint's motor."""
        config = sdk.config.ArmConfig.from_yaml(str(PROFILE))
        special = special or {}
        motors = [special[j.name](j.motor_id, j.model, p) if j.name in special
                  else FakeMotor(j.motor_id, j.model, position=p, **motor_kw) for j, p in zip(config.joints, pose)]
        self.buses[address] = FakeBus(motors)
        return {j.name: m for j, m in zip(config.joints, motors)}

    def create_backend(self, name, **kwargs):
        address = kwargs.get("channel") or kwargs.get("port") or name
        self.opened.append((name, address))
        return self.buses.get(address) or FakeBus([])

    def write_rig(self, arms=None, cameras=None, leaders=None):
        rig_module.Rig(arms or {}, cameras or {}, leaders or {}).save(self.rig_path)

    def run(self, *argv):
        """(exit code, stdout, stderr)."""
        self.capsys.readouterr()
        try:
            cli.main([str(a) for a in argv])
            code = 0
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else 1
        out = self.capsys.readouterr()
        return code, out.out, out.err

    def json(self, *argv, code=0):
        rc, out, err = self.run(*argv, "--json")
        assert rc == code, (rc, out, err)
        return json.loads(out)


@pytest.fixture
def bench(monkeypatch, capsys, tmp_path):
    return Bench(monkeypatch, capsys, tmp_path)


def gs(serial):
    return rig_module.ArmSpec("gs_usb", port=f"gs_usb:{serial}")


# ── doctor / ports ──
def test_doctor_reports_adapters_and_an_invalid_rig(bench):
    bench.add_arm("gs_usb:AAA")
    bench.rig_path.write_text("arms: {left: {backend: nope}}\n")
    doc = bench.json("doctor")
    assert doc["ok"] and doc["gs_usb"][0]["port"] == "gs_usb:AAA"
    assert doc["rig"]["valid"] is False and "backend" in doc["rig"]["problems"][0]
    assert bench.opened == []   # no bus opened


def test_doctor_lists_rig_problems(bench):
    bench.write_rig({"left": gs("AAA"), "right": gs("MISSING")}, cameras={"top": "usb:nowhere"})
    bench.add_arm("gs_usb:AAA")
    doc = bench.json("doctor")
    problems = " ".join(doc["rig"]["problems"])
    assert "gs_usb:MISSING is not plugged in" in problems and "usb:nowhere is not connected" in problems
    assert doc["rig"]["arms"] == ["left", "right"]


def test_ports_json_shape(bench):
    bench.add_arm("gs_usb:AAA")
    doc = bench.json("ports")
    assert set(doc) >= {"command", "ok", "gs_usb", "socketcan", "serial", "cameras"}
    assert doc["command"] == "ports" and doc["gs_usb"][0]["serial"] == "AAA"
    _, text, _ = bench.run("ports")
    assert "gs_usb:AAA" in text


# ── rig command ──
def test_rig_edit_round_trip(bench):
    assert bench.run("rig", "set-arm", "left", "--port", "gs_usb:AAA")[0] == 0
    assert bench.run("rig", "set-arm", "right", "--backend", "socketcan", "--channel", "can1",
                     "--profile", "p.yaml")[0] == 0
    assert bench.run("rig", "set-camera", "top", "usb:platform-xhci-usb-0:1.2:1.0")[0] == 0
    assert bench.run("rig", "set-leader", "left", "--type", "star", "--star-port", "/dev/ttyUSB0")[0] == 0
    assert bench.run("rig", "set-leader", "right", "--type", "arm", "--leader-arm", "left")[0] == 0
    doc = bench.json("rig", "show")
    assert doc["rig"]["arms"] == {"left": {"backend": "gs_usb", "port": "gs_usb:AAA"},
                                  "right": {"backend": "socketcan", "channel": "can1", "profile": "p.yaml"}}
    assert doc["rig"]["cameras"] == {"top": "usb:platform-xhci-usb-0:1.2:1.0"}
    assert doc["rig"]["leaders"]["right"] == {"type": "arm", "arm": "left"}
    assert bench.run("rig", "set-arm", "left", "--remove")[0] == 0   # its leaders go with it
    rig = rig_module.Rig.load(bench.rig_path)
    assert rig.arm_names == ["right"] and rig.leaders == {}


def test_rig_rejects_bad_edits_and_keeps_the_file(bench):
    bench.run("rig", "set-arm", "left", "--port", "gs_usb:AAA")
    before = bench.rig_path.read_text()
    code, _, err = bench.run("rig", "set-leader", "ghost", "--type", "star", "--star-port", "/dev/x")
    assert code == 1 and "no arm named 'ghost'" in err
    assert bench.run("rig", "set-camera", "top", "webcam")[0] == 1
    assert bench.run("rig", "set-arm", "all", "--port", "gs_usb:B")[0] == 1
    assert bench.rig_path.read_text() == before


def test_rig_init_detect_and_force(bench):
    bench.add_arm("gs_usb:AAA")
    bench.add_arm("gs_usb:BBB")
    doc = bench.json("rig", "init", "--detect")
    assert doc["rig"]["arms"] == {"arm1": {"backend": "gs_usb", "port": "gs_usb:AAA"},
                                  "arm2": {"backend": "gs_usb", "port": "gs_usb:BBB"}}
    assert bench.json("rig", "init", code=1)["error"].endswith("pass --force to start over")
    assert bench.json("rig", "init", "--force")["rig"]["arms"] == {}


# ── scan / status ──
def test_scan_json_and_text(bench):
    bench.add_arm()
    doc = bench.json("scan")
    arm = doc["arms"][0]
    assert arm["name"] == "arm" and arm["backend"] == "socketcan" and arm["address"] == "can0"
    assert [m["motor_id"] for m in arm["motors"]] == list(range(1, 8)) and arm["missing"] == []
    assert arm["motors"][1]["position"] == pytest.approx(-1.0, abs=1e-3)
    code, text, _ = bench.run("scan")
    assert code == 0 and "shoulder_lift" in text and "-57.3" in text


def test_scan_without_motors_fails(bench):
    doc = bench.json("scan", "--channel", "can9", code=1)
    assert doc["ok"] is False and doc["arms"][0]["motors"] == []


def test_scan_by_rig_name_and_all(bench):
    bench.add_arm("gs_usb:AAA")
    bench.add_arm("gs_usb:BBB")
    bench.write_rig({"left": gs("AAA"), "right": gs("BBB")})
    doc = bench.json("scan", "--arm", "all")
    assert [a["name"] for a in doc["arms"]] == ["left", "right"]
    assert bench.json("scan", "--arm", "middle", code=1)["error"].startswith("no arm named 'middle'")
    assert "not both" in bench.json("scan", "--arm", "left", "--channel", "can0", code=1)["error"]


def test_only_rig_arm_is_the_default(bench):
    bench.add_arm("gs_usb:AAA")
    bench.write_rig({"solo": gs("AAA")})
    assert bench.json("scan")["arms"][0]["name"] == "solo"


def test_status_reports_every_joint_and_a_fault(bench):
    motors = bench.add_arm()
    motors["gripper"].faulted = True
    doc = bench.json("status")
    joints = {j["joint"]: j for j in doc["arms"]["arm"]["joints"]}
    assert doc["units"]["angle"] == "rad"
    assert joints["elbow_flex"]["position"] == pytest.approx(1.0, abs=1e-3)
    assert joints["elbow_flex"]["enabled"] is None or joints["elbow_flex"]["enabled"] is False
    assert FAULT_TEXT in joints["gripper"]["fault"]
    assert all(m.log.count("enable") == 0 for m in motors.values())


def test_status_missing_motor_exits_nonzero(bench):
    motors = bench.add_arm()
    motors["wrist_roll"].online = False
    doc = bench.json("status", code=1)
    assert doc["arms"]["arm"]["missing"] == [6]


def test_status_watch_prints_json_lines(bench):
    bench.add_arm()
    code, out, _ = bench.run("status", "--watch", "50", "--count", "3", "--json")
    lines = [json.loads(line) for line in out.splitlines()]
    assert code == 0 and len(lines) == 3 and "arm" in lines[0]["arms"]


# ── clear-faults / release ──
def test_clear_faults_only_touches_faulted_motors(bench):
    motors = bench.add_arm()
    motors["gripper"].faulted = True
    doc = bench.json("clear-faults")
    assert doc["arms"]["arm"]["cleared"] == ["gripper"]
    assert not motors["gripper"].faulted
    assert not any("enable" in m.log for m in motors.values())


def test_release_needs_yes_without_a_terminal(bench):
    motors = bench.add_arm(enabled=True)
    doc = bench.json("release", code=1)
    assert "refusing without --yes" in doc["error"]
    assert all(m.enabled for m in motors.values())
    doc = bench.json("release", "--yes")
    assert doc["arms"]["arm"] == {"released": 7, "unconfirmed": [], "missing": []}
    assert not any(m.enabled for m in motors.values())
    # neutral (zero-gain) command before the disable
    assert motors["elbow_flex"].frames[-1][1] == 0.0


# ── params ──
def test_params_list_needs_no_bus(bench):
    doc = bench.json("params", "list")
    assert set(doc["params"]) == set(sdk.motor.Motor.PARAMS) and bench.opened == []


def test_params_get_and_set(bench):
    motors = bench.add_arm(params_supported=True)
    name = next(n for n, s in sdk.motor.Motor.PARAMS.items() if s.dtype != "f")
    assert bench.json("params", "get", name, code=1)["error"] == "params get needs --motor ID"
    doc = bench.json("params", "get", name, "--motor", "7")
    assert doc["motor"] == 7 and "value" in doc
    assert "refusing" in bench.json("params", "set", name, "0", "--motor", "7", code=1)["error"]
    assert motors["gripper"].log.count(("write", sdk.motor.Motor.PARAMS[name].index)) == 0


def test_clear_faults_reports_a_fault_that_stays(bench, monkeypatch):
    motors = bench.add_arm()
    gripper = motors["gripper"]
    gripper.faulted = True
    original = gripper.handle

    def stubborn(can_id, data):
        replies = original(can_id, data)
        gripper.faulted = True   # the cause is still there
        return replies

    gripper.handle = stubborn
    doc = bench.json("clear-faults", code=1)
    assert doc["arms"]["arm"]["still_faulted"] == ["gripper"] and doc["arms"]["arm"]["cleared"] == []
