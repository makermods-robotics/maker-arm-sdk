"""Physical-consistency checks for the local designer-asset revision."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import pytest

from tools.engineering.build_gripper_urdf import GAP_M, JAW_PARTS, build, inertial_values, rotation, triangles, volume_moments
from tools.diagnostics.urdf_viewer.server import describe_model

SOURCE = Path.home()/"Downloads/Maker Arm_urdf_260902.zip"
ENDPOINTS = Path(__file__).resolve().parents[1]/"output/urdf-revisions/gripper_mit_endpoints.json"


def test_tetrahedral_integration_has_known_volume_centroid_and_moments():
    o, a, b, c = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype=float)
    faces = np.array([[a,b,c], [o,b,a], [o,a,c], [o,c,b]])
    v, first, second = volume_moments(faces)
    assert v == pytest.approx(1/6)
    assert first/v == pytest.approx([.25,.25,.25])
    assert second.diagonal()/v == pytest.approx([.1,.1,.1])
    assert second[0,1]/v == pytest.approx(.05)
    vr, fr, sr = volume_moments(faces[:,::-1])
    assert vr == pytest.approx(v)
    assert fr == pytest.approx(first)
    assert sr == pytest.approx(second)


@pytest.fixture
def revision(tmp_path):
    if not SOURCE.exists() or not ENDPOINTS.exists():
        pytest.skip("Local designer ZIP and hardware capture required")
    target = tmp_path/"revision.zip"
    build(SOURCE,target,ENDPOINTS)
    return target


def test_revised_tree_keeps_six_arm_joints_and_one_independent_gripper(revision):
    m = describe_model(revision)
    assert m["gripper_joint_present"]
    assert len(m["joints"]) == 6
    assert len(m["all_joints"]) == 9
    assert m["links"] == 10
    assert m["mass_kg"] == pytest.approx(3.38)
    assert not m["revision"]["rl_ready"]
    with zipfile.ZipFile(SOURCE) as source, zipfile.ZipFile(revision) as target:
        a, b = ET.fromstring(source.read("robot.urdf")), ET.fromstring(target.read("robot.urdf"))
        for old,new in zip(a.findall("joint"),b.findall("joint")[:6]):
            assert ET.tostring(old).strip() == ET.tostring(new).strip()


def test_closed_mass_com_and_full_inertia_are_preserved(revision):
    with zipfile.ZipFile(SOURCE) as source, zipfile.ZipFile(revision) as target:
        original = ET.fromstring(source.read("robot.urdf"))
        revised = ET.fromstring(target.read("robot.urdf"))
        expected = inertial_values(original.find("./link[@name='link_007']/inertial"))
        actual = [0., np.zeros(3), np.zeros((3,3))]
        for name in ["link_007", *JAW_PARTS]:
            vals = inertial_values(revised.find(f"./link[@name='{name}']/inertial"))
            for i in range(3):
                actual[i] += vals[i]
        for measured, inherited in zip(actual,expected):
            assert measured == pytest.approx(inherited, abs=2e-12)


def test_jaw_gap_collision_membership_and_original_mesh_bytes(revision):
    with zipfile.ZipFile(SOURCE) as source, zipfile.ZipFile(revision) as target:
        robot = ET.fromstring(target.read("robot.urdf"))
        axes=[]
        for name,parts in JAW_PARTS.items():
            link=robot.find(f"./link[@name='{name}']")
            joint=robot.find(f"./joint[@name='{name}_joint']")
            axis=np.array(list(map(float,joint.find("axis").get("xyz").split())))
            assert np.linalg.norm(axis)==pytest.approx(1)
            assert float(joint.find("limit").get("upper"))*2==pytest.approx(GAP_M)
            axes.append(axis)
            collider_ids={int(m.get("filename").split("part_")[1][:3]) for m in link.findall("./collision/geometry/mesh")}
            assert collider_ids==parts
            assert not any(int(m.get("filename").split("part_")[1][:3]) in (46,47) for m in link.findall("./visual/geometry/mesh"))
        assert axes[0]==pytest.approx(-axes[1])
        inner_faces=[]
        for name,part in [("gripper_left",45),("gripper_right",44)]:
            visual=next(v for v in robot.find(f"./link[@name='{name}']").findall("visual") if f"part_{part:03d}_" in v.find("geometry/mesh").get("filename"))
            verts=triangles(target.read(visual.find("geometry/mesh").get("filename")))
            o=visual.find("origin")
            transform=rotation(o.get("rpy")); offset=np.array(list(map(float,o.get("xyz").split())))
            plane=verts[np.max(np.abs(verts[:,:,2]),axis=1)<1e-7]
            positions=plane.reshape(-1,3)@transform.T+offset
            inner_faces.append(positions@axes[0])
        closed_gap=inner_faces[0].mean()-inner_faces[1].mean()
        assert closed_gap==pytest.approx(0,abs=2e-8)
        assert closed_gap+GAP_M==pytest.approx(.104825,abs=2e-8)
        for name in source.namelist():
            if name.endswith('.stl'):
                assert target.read(name)==source.read(name)
