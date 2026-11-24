"""Chain from URDF: FK, Jacobian, gravity, IK on a hand-checkable planar arm and on this repo's arm."""

import math
import time

import numpy as np
import pytest

from chain_fixture import LIMITS, ROOT, SPEC, load, repo_chain

kinematics = load("kinematics")
Chain = kinematics.Chain

L1, LC1, LC2, M1, M2 = 0.3, 0.12, 0.1, 1.5, 0.8
JAW_M, JAW_COM = 0.2, (0.05, 0.0, 0.0)
TOOL_X = 0.25
G = 9.81

# Two pitch joints (axis y) in the x-z plane, a jaw on a prismatic joint and a massive fixed tool link
# under link2, plus a mast on its own branch of the base that is not part of the chain.
PLANAR = f"""<?xml version="1.0"?>
<robot name="planar">
  <link name="world"/>
  <link name="base"><inertial><origin xyz="0 0 0.05"/><mass value="3"/></inertial></link>
  <link name="mast"><inertial><origin xyz="0 0 0.5"/><mass value="1"/></inertial></link>
  <link name="link1"><inertial><origin xyz="{LC1} 0 0" rpy="0.3 0 0"/><mass value="{M1}"/></inertial></link>
  <link name="link2"><inertial><origin xyz="{LC2 - 0.05} 0 0"/><mass value="{M2 / 2}"/></inertial></link>
  <link name="jaw"><inertial><origin xyz="{JAW_COM[0]} 0 0"/><mass value="{JAW_M}"/></inertial></link>
  <link name="flange"><inertial><origin xyz="0.05 0 0"/><mass value="{M2 / 2}"/></inertial></link>
  <link name="tool"/>
  <joint name="mount" type="fixed"><parent link="world"/><child link="base"/></joint>
  <joint name="mast_joint" type="fixed"><parent link="base"/><child link="mast"/><origin xyz="0.2 0 0"/></joint>
  <joint name="j1" type="revolute"><parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-2" upper="2"/></joint>
  <joint name="j2" type="revolute"><parent link="link1"/><child link="link2"/>
    <origin xyz="{L1} 0 0"/><axis xyz="0 1 0"/><limit lower="-2.5" upper="1"/></joint>
  <joint name="flange_joint" type="fixed"><parent link="link2"/><child link="flange"/><origin xyz="0.1 0 0"/></joint>
  <joint name="jaw_joint" type="prismatic"><parent link="link2"/><child link="jaw"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="0" upper="0.04"/></joint>
  <joint name="tool_joint" type="fixed"><parent link="link2"/><child link="tool"/><origin xyz="{TOOL_X} 0 0"/></joint>
</robot>
"""


@pytest.fixture
def planar_urdf(tmp_path):
    path = tmp_path / "planar.urdf"
    path.write_text(PLANAR)
    return path


def planar_tool(q):
    """Tool point by hand: a pitch about y turns +x toward -z."""
    t1, t12 = q[0], q[0] + q[1]
    return np.array([L1 * math.cos(t1) + TOOL_X * math.cos(t12), 0.0,
                     0.1 - L1 * math.sin(t1) - TOOL_X * math.sin(t12)])


def planar_gravity(q):
    """dV/dq with V = g * sum(m z): link2 carries its own mass, the flange and the jaw."""
    t1, t12 = q[0], q[0] + q[1]
    m2 = M2 + JAW_M
    m2c = M2 * LC2 + JAW_M * JAW_COM[0]
    return np.array([-G * (M1 * LC1 * math.cos(t1) + m2 * L1 * math.cos(t1) + m2c * math.cos(t12)),
                     -G * m2c * math.cos(t12)])


def numeric_jacobian(chain, q, h=1e-6):
    jac = np.zeros((6, chain.n))
    pose = chain.fk(q)
    for i in range(chain.n):
        dq = np.zeros(chain.n)
        dq[i] = h
        plus, minus = chain.fk(q + dq), chain.fk(q - dq)
        jac[:3, i] = (plus[:3, 3] - minus[:3, 3]) / (2 * h)
        jac[3:, i] = kinematics.rotation_error(plus[:3, :3], minus[:3, :3]) / (2 * h)
    assert np.allclose(pose, chain.fk(q))
    return jac


def numeric_gravity(chain, q, h=1e-6):
    out = np.zeros(chain.n)
    for i in range(chain.n):
        dq = np.zeros(chain.n)
        dq[i] = h
        out[i] = (chain.potential_energy(q + dq) - chain.potential_energy(q - dq)) / (2 * h)
    return out


# ── parsing ──
def test_default_chain_follows_the_revolute_branch(planar_urdf):
    chain = Chain.from_urdf(planar_urdf)
    assert chain.joint_names == ["j1", "j2"]
    assert chain.lower.tolist() == [-2, -2.5] and chain.upper.tolist() == [2, 1]


def test_off_chain_links_are_lumped_and_the_base_branch_carries_no_joint(planar_urdf):
    chain = Chain.from_urdf(planar_urdf)
    assert chain.masses.tolist() == pytest.approx([3 + 1, M1, M2 + JAW_M])


def test_explicit_tip_extends_through_fixed_joints(planar_urdf):
    chain = Chain.from_urdf(planar_urdf, tip="tool")
    assert np.allclose(chain.fk([0.4, -0.7])[:3, 3], planar_tool([0.4, -0.7]))


def test_joint_list_holds_the_other_movable_joints_at_zero(planar_urdf):
    chain = Chain.from_urdf(planar_urdf, joints=["j2"], tip="tool")
    full = Chain.from_urdf(planar_urdf, tip="tool")
    assert chain.n == 1
    assert np.allclose(chain.fk([0.3]), full.fk([0.0, 0.3]))


@pytest.mark.parametrize("bad", [["j2", "j1"], ["jaw_joint"], ["nope"]])
def test_bad_joint_lists_are_refused(planar_urdf, bad):
    with pytest.raises(ValueError):
        Chain.from_urdf(planar_urdf, joints=bad)


# ── kinematics ──
@pytest.mark.parametrize("q", [[0.0, 0.0], [0.4, -0.7], [-1.2, 0.9]])
def test_fk_and_gravity_match_the_hand_model(planar_urdf, q):
    chain = Chain.from_urdf(planar_urdf, tcp_xyz=(TOOL_X, 0, 0))
    assert np.allclose(chain.fk(q)[:3, 3], planar_tool(q))
    assert np.allclose(chain.gravity_torques(q), planar_gravity(q))


def test_sign_and_offset_map_joint_coordinates_onto_urdf_angles(planar_urdf):
    plain = Chain.from_urdf(planar_urdf)
    mapped = Chain.from_urdf(planar_urdf, sign=[-1, 1], offset=[0.2, -0.1])
    q = np.array([0.3, 0.5])
    theta = q * [-1, 1] + [0.2, -0.1]
    assert np.allclose(mapped.fk(q), plain.fk(theta))
    assert np.allclose(mapped.gravity_torques(q), plain.gravity_torques(theta) * [-1, 1])
    assert np.allclose(mapped.jacobian(q), plain.jacobian(theta) * [-1, 1])
    # URDF j1 limits [-2, 2] with sign -1, offset 0.2 -> q in [-1.8, 2.2]
    assert mapped.lower[0] == pytest.approx(-1.8) and mapped.upper[0] == pytest.approx(2.2)


def test_base_turn_rotates_the_world_frame(planar_urdf):
    turned = Chain.from_urdf(planar_urdf, base_rpy=(0, 0, math.pi / 2), base_xyz=(1, 0, 0))
    plain = Chain.from_urdf(planar_urdf)
    x, y, z = plain.fk([0.2, 0.1])[:3, 3]
    assert np.allclose(turned.fk([0.2, 0.1])[:3, 3], [1 - y, x, z])


def test_pose_helpers_round_trip():
    rng = np.random.default_rng(3)
    for _ in range(200):
        rpy = rng.uniform([-math.pi, -1.5, -math.pi], [math.pi, 1.5, math.pi])
        xyz, back = kinematics.pose_to_xyz_rpy(kinematics.xyz_rpy_to_pose([1, 2, 3], rpy))
        assert np.allclose(xyz, [1, 2, 3])
        assert np.allclose(kinematics.rpy_to_matrix(back), kinematics.rpy_to_matrix(rpy))


@pytest.mark.parametrize("angle", [1e-8, 0.3, math.pi - 1e-6, math.pi])
def test_rotation_error_is_axis_times_angle(angle):
    axis = np.array([1.0, -2.0, 0.5]) / np.linalg.norm([1.0, -2.0, 0.5])
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    rot = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
    err = kinematics.rotation_error(rot, np.eye(3))
    assert np.linalg.norm(err) == pytest.approx(angle, abs=1e-9)
    assert abs(err @ axis) == pytest.approx(angle, abs=1e-9)


# ── this repo's arm ──
def test_repo_chain_is_the_profile_arm():
    chain = repo_chain()
    assert chain.n == len(SPEC["sign"])
    assert (ROOT / SPEC["urdf"]).exists()


def test_repo_jacobian_matches_finite_differences():
    chain = repo_chain()
    rng = np.random.default_rng(11)
    for _ in range(20):
        q = rng.uniform(*LIMITS)
        assert np.allclose(chain.jacobian(q), numeric_jacobian(chain, q), atol=1e-6)


def test_repo_gravity_is_the_gradient_of_potential_energy():
    chain = repo_chain()
    rng = np.random.default_rng(12)
    for _ in range(50):
        q = rng.uniform(*LIMITS)
        assert np.allclose(chain.gravity_torques(q), numeric_gravity(chain, q), atol=1e-6)


def test_repo_tcp_moves_the_tool_point_along_the_tip_frame():
    bare, tool = repo_chain(tcp=False), repo_chain()
    q = np.mean(LIMITS, axis=0)
    flange = bare.fk(q)
    assert np.allclose(tool.fk(q)[:3, 3], flange[:3, :3] @ SPEC["tcp_xyz"] + flange[:3, 3])


# ── IK ──
def ik_rates(chain, n, full_pose, seed_noise=0.2, rng_seed=5):
    rng = np.random.default_rng(rng_seed)
    lo, hi = LIMITS
    good, times = 0, []
    for _ in range(n):
        q_true = rng.uniform(lo, hi)
        seed = np.clip(q_true + rng.uniform(-seed_noise, seed_noise, chain.n), lo, hi)
        pose = chain.fk(q_true)
        start = time.perf_counter()
        result = chain.ik(pose if full_pose else pose[:3, 3], seed, limits=(lo, hi))
        times.append(time.perf_counter() - start)
        good += result.pos_err < 1e-3 and (not full_pose or result.rot_err < math.radians(0.5))
        assert np.all(np.array(result.q) >= lo - 1e-12) and np.all(np.array(result.q) <= hi + 1e-12)
    return good / n, float(np.median(times))


def test_position_ik_converges_from_a_nearby_seed():
    rate, _ = ik_rates(repo_chain(), 150, full_pose=False)
    assert rate > 0.99


def test_full_pose_ik_converges_from_a_nearby_seed():
    rate, _ = ik_rates(repo_chain(), 150, full_pose=True)
    assert rate > 0.95


def test_ik_result_reports_the_solution_it_found():
    chain = repo_chain()
    q_true = np.mean(LIMITS, axis=0)
    target = chain.fk(q_true)
    result = chain.ik(target[:3, 3], q_true + 0.1, rpy=kinematics.matrix_to_rpy(target[:3, :3]), limits=LIMITS)
    assert result.ok and result.iters > 0
    reached = chain.fk(result.q)
    assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) == pytest.approx(result.pos_err, abs=1e-12)
    assert result.rot_err < 1e-3


def test_position_ik_stays_near_the_seed_in_the_redundant_directions():
    chain = repo_chain()
    q0 = np.mean(LIMITS, axis=0)
    target = chain.fk(q0)[:3, 3]
    result = chain.ik(target, q0, limits=LIMITS)
    assert result.ok and result.iters == 0 and np.allclose(result.q, q0)


def test_unreachable_target_is_not_ok_but_returns_the_closest_pose():
    chain = repo_chain()
    q0 = np.mean(LIMITS, axis=0)
    result = chain.ik([5.0, 0.0, 0.0], q0, limits=LIMITS)
    assert not result.ok and result.pos_err > 3.0
    assert len(result.q) == chain.n and all(math.isfinite(v) for v in result.q)


@pytest.mark.parametrize("bad", [dict(target=[0, 0]), dict(target=[0, 0, math.nan]), dict(q0=[0.0])])
def test_ik_refuses_malformed_input(bad):
    chain = repo_chain()
    args = dict(target=[0.2, 0.0, 0.2], q0=np.mean(LIMITS, axis=0)) | bad
    with pytest.raises(ValueError):
        chain.ik(args["target"], args["q0"])


def test_repo_gravity_float_version_matches_the_numpy_reference():
    chain = repo_chain()
    rng = np.random.default_rng(11)
    for _ in range(50):
        q = rng.uniform(chain.lower, chain.upper)
        g = rng.normal(size=3) * 5
        assert chain.gravity_torques(q, g) == pytest.approx(chain._gravity_torques_numpy(q, g), abs=1e-12)
        assert chain.gravity_torques(q) == pytest.approx(chain._gravity_torques_numpy(q), abs=1e-12)
