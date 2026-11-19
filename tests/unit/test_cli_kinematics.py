"""fk / ik on a small planar test chain: no motion, and no hardware when joints are given."""

import math

import pytest

from fakes import FakeBus, FakeMotor
from test_cli_bench import Bench

URDF = """<robot name="planar">
  <link name="base"/><link name="l1"/><link name="l2"/>
  <joint name="a" type="revolute"><parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="b" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0.2 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
</robot>
"""


@pytest.fixture
def bench(monkeypatch, capsys, tmp_path):
    b = Bench(monkeypatch, capsys, tmp_path)
    (tmp_path / "planar.urdf").write_text(URDF)
    b.profile = tmp_path / "planar.yaml"
    b.profile.write_text(f"urdf: {tmp_path / 'planar.urdf'}\ntcp_xyz: [0.1, 0, 0]\njoints:\n"
                         "  - {name: a, motor_id: 1, limits_deg: [-170, 170], kp: 20, kd: 1}\n"
                         "  - {name: b, motor_id: 2, limits_deg: [-170, 170], kp: 20, kd: 1}\n")
    return b


def test_fk_given_joints_needs_no_hardware(bench):
    doc = bench.json("fk", "--config", bench.profile, "--joints", "0", "90")
    assert doc["xyz"] == pytest.approx([0.2, 0.1, 0.1]) and doc["source"] == "given"
    assert doc["rpy_deg"][2] == pytest.approx(90.0)
    assert bench.opened == []
    doc = bench.json("fk", "--config", bench.profile, "--joints", "0", "0", "--set", "a=90")
    assert doc["xyz"] == pytest.approx([0.0, 0.3, 0.1], abs=1e-12)


def test_fk_reads_the_arm(bench):
    bench.buses["can0"] = FakeBus([FakeMotor(1, position=math.pi / 2), FakeMotor(2, position=0.0)])
    doc = bench.json("fk", "--config", bench.profile)
    assert doc["source"] == "measured" and doc["xyz"] == pytest.approx([0.0, 0.3, 0.1], abs=1e-3)
    assert not any("enable" in m.log for m in bench.buses["can0"].motors.values())


def test_fk_errors(bench, tmp_path):
    bare = tmp_path / "bare.yaml"
    bare.write_text("\n".join(line for line in bench.profile.read_text().splitlines() if not line.startswith("urdf")))
    assert "names no URDF" in bench.json("fk", "--config", bare, "--joints", "0", "0", code=1)["error"]
    assert "needs 2 values" in bench.json("fk", "--config", bench.profile, "--joints", "1", "2", "3", code=1)["error"]
    assert "could not read" in bench.json("fk", "--config", bench.profile, code=1)["error"]


def test_ik_solves_and_reports_the_residual(bench):
    doc = bench.json("ik", "--config", bench.profile, "--xyz", "0.2", "0.1", "0.1", "--seed", "10", "60")
    assert doc["ok"] and doc["position_error"] < 1e-4 and doc["seed"] == "given"
    fk = bench.json("fk", "--config", bench.profile, "--joints", *[str(v) for v in doc["joints_deg"].values()])
    assert fk["xyz"] == pytest.approx([0.2, 0.1, 0.1], abs=1e-4)


def test_ik_unreachable_exits_nonzero(bench):
    doc = bench.json("ik", "--config", bench.profile, "--xyz", "1.0", "0", "0.1", "--seed", "0", "10", code=1)
    assert doc["ok"] is False and doc["position_error"] > 0.5


def test_ik_full_pose(bench):
    doc = bench.json("ik", "--config", bench.profile, "--xyz", "0.2", "0.1", "0.1", "--rpy", "0", "0", "90",
                     "--seed", "10", "60")
    assert doc["ok"] and doc["orientation_error"] < 1e-3
    assert doc["joints_deg"] == pytest.approx({"a": 0.0, "b": 90.0}, abs=0.1)


def test_ik_path_reports_the_largest_joint_step(bench):
    doc = bench.json("ik", "--config", bench.profile, "--xyz", "0.2", "0.1", "0.1", "--seed", "10", "60",
                     "--to-xyz", "0.22", "0.12", "0.1", "--steps", "200")
    path = doc["path"]
    assert doc["ok"] and path["ok"] and path["ticks"] == 200 and path["end_position_error"] < 1e-3
    assert 0 < max(path["max_step"].values()) < math.radians(2)
    assert bench.opened == []


def test_ik_path_to_an_unreachable_point_exits_nonzero(bench):
    doc = bench.json("ik", "--config", bench.profile, "--xyz", "0.2", "0.1", "0.1", "--seed", "10", "60",
                     "--to-xyz", "1.0", "0", "0.1", "--steps", "20", code=1)
    assert doc["path"]["ok"] is False and doc["path"]["end_position_error"] > 0.5
