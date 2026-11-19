"""Trajectory IK: the original solver's answers (tests/unit/trajectory_ik_fixtures.json), and the guarantees on
this repo's arm: speed cap, limits, continuity across chunks, small moves near a singularity, unreachable poses."""

import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from conftest import sdk

TI, kinematics, config_mod = sdk.trajectory_ik, sdk.kinematics, sdk.config
FIXTURES = json.loads((Path(__file__).resolve().parent / "trajectory_ik_fixtures.json").read_text())
RATE = 20.0


# ── the original's answers ──
@pytest.fixture(scope="module")
def fixture_ik(tmp_path_factory):
    path = tmp_path_factory.mktemp("urdf") / "fixture.urdf"
    path.write_text(FIXTURES["urdf"])
    chain = kinematics.Chain.from_urdf(path, joints=FIXTURES["joints"], tip=FIXTURES["tip"],
                                       tcp_xyz=FIXTURES["tcp_xyz"], tcp_rpy=FIXTURES["tcp_rpy"])
    return TI.TrajectoryIK(chain, None, FIXTURES["max_speed"], FIXTURES["rest_pose"],
                           rest_weight=FIXTURES["rest_weight"])


def poses(rows):
    out = np.broadcast_to(np.eye(4), (len(rows), 4, 4)).copy()
    out[:, :3] = np.asarray(rows, dtype=float).reshape(-1, 3, 4)
    return out


@pytest.mark.parametrize("case", FIXTURES["cases"], ids=lambda c: f"{c['method']}-cap{c['speed_cap']}")
def test_reproduces_the_original_solver(fixture_ik, case):
    result = fixture_ik.solve(poses(case["targets"]), case["q_last"], case["q_before"],
                              rate_hz=FIXTURES["rate_hz"], speed_cap=case["speed_cap"], method=case["method"])
    diff = np.abs(result.q - np.asarray(case["q"]))
    assert np.percentile(diff, 95) < 2e-3 and diff.max() < 1e-2


# ── this repo's arm ──
@pytest.fixture(scope="module")
def profile():
    return config_mod.ArmConfig.from_yaml(str(sdk._product.DEFAULT_PROFILE))


@pytest.fixture(scope="module")
def ik(profile):
    return TI.TrajectoryIK.from_config(profile)


def path_from(ik, q, offset, ticks, turn=(0.0, 0.0, 0.0), ease=True):
    start = ik.chain.fk(q)
    goal = start.copy()
    goal[:3, 3] += offset
    goal[:3, :3] = start[:3, :3] @ TI.so3_exp(np.asarray(turn, dtype=float))
    return TI.straight_line(start, goal, ticks, ease)


def steps(q, q_last):
    return np.abs(np.diff(np.vstack([q_last, q]), axis=0))


def test_from_config_reads_the_profile(ik, profile):
    n = ik.n
    assert ik.rest.tolist() == pytest.approx(profile.ik["rest_pose"])
    assert np.all(ik.max_velocity <= profile.max_velocity + 1e-12)
    joints = profile.joints[:n]
    assert np.all(ik.lo >= np.array([j.lo for j in joints]) + ik.weights.limit_margin - 1e-12)
    assert np.all(ik.hi <= np.array([j.hi for j in joints]) - ik.weights.limit_margin + 1e-12)


def test_reaches_a_smooth_path_and_reports_the_error(ik):
    q0 = ik.rest
    targets = path_from(ik, q0, [0.03, -0.02, 0.02], 40, turn=[0.1, 0.0, 0.1])
    result = ik.solve(targets, q0, rate_hz=RATE)
    assert result.q.shape == (40, ik.n) and not result.capped.any()
    assert result.pos_err.max() < 3e-3 and result.rot_err.max() < 0.02
    pos, rot = TI.tool_errors(ik.chain, result.q, targets)
    assert result.pos_err == pytest.approx(pos) and result.rot_err == pytest.approx(rot)


@pytest.mark.parametrize("method", ["traj", "tick"])
def test_speed_cap_is_honoured_with_the_direction_kept(ik, method):
    q0 = ik.rest
    targets = path_from(ik, q0, [0.15, 0.1, -0.1], 10, ease=False)
    result = ik.solve(targets, q0, rate_hz=RATE, speed_cap=0.5, method=method)
    cap = ik.max_step(RATE, 0.5)
    assert np.all(steps(result.q, q0) <= cap + 1e-12)
    assert result.capped.any()
    t = int(np.argmax(result.capped))
    before = q0 if t == 0 else result.q[t - 1]
    first = result.q[t] - before
    want = result.q_solution[t] - before
    assert np.allclose(first / np.linalg.norm(first), want / np.linalg.norm(want))


def test_limits_are_honoured(ik):
    q0 = ik.rest
    targets = path_from(ik, q0, [0.0, 0.0, -2.0], 20)   # far below the base: limits must hold
    result = ik.solve(targets, q0, rate_hz=RATE)
    assert np.all(result.q >= ik.lo - 1e-12) and np.all(result.q <= ik.hi + 1e-12)
    assert np.all(result.q_solution >= ik.lo - 1e-12) and np.all(result.q_solution <= ik.hi + 1e-12)


def test_a_joint_past_its_margin_never_goes_further(ik):
    q0 = ik.rest.copy()
    q0[0] = ik.hi[0] + 0.01
    targets = np.repeat(ik.chain.fk(q0)[None], 10, axis=0)
    result = ik.solve(targets, q0, rate_hz=RATE)
    assert np.all(result.q[:, 0] <= q0[0] + 1e-12)


def test_unreachable_target_is_reported_not_raised(ik):
    q0 = ik.rest
    targets = path_from(ik, q0, [3.0, 0.0, 0.0], 10)
    result = ik.solve(targets, q0, rate_hz=RATE)
    assert np.all(np.isfinite(result.q)) and result.pos_err[-1] > 1.0
    assert np.all(steps(result.q, q0) <= ik.max_step(RATE) + 1e-12)
    assert np.all(result.q >= ik.lo - 1e-12) and np.all(result.q <= ik.hi + 1e-12)


def test_successive_chunks_continue_without_a_velocity_jump(ik):
    q0 = ik.rest
    targets = TI.straight_line(ik.chain.fk(q0), ik.chain.fk(q0) @ kinematics.xyz_rpy_to_pose([0.08, 0.0, 0.05]),
                               60, ease=False)
    stream = TI.IKStream(ik, q0, RATE)
    a = stream.feed(targets[:30]).q
    b = stream.feed(targets[30:]).q
    joined = np.vstack([q0, a, b])
    vel = np.diff(joined, axis=0)
    acc = np.abs(np.diff(vel, axis=0))
    seam = acc[30 - 1]   # velocity change from the last tick of a into the first of b
    assert seam.max() < 0.25 * np.abs(vel[25:35]).max() + 1e-4
    # without the history the solve starts the new chunk as if the arm were still
    cold = ik.solve(targets[30:], a[-1], rate_hz=RATE).q
    assert np.abs(cold[0] - a[-1] - (a[-1] - a[-2])).max() > seam.max()


def test_step_streams_single_poses_under_the_cap(ik):
    q0 = ik.rest
    stream = TI.IKStream(ik, q0, RATE, speed_cap=1.0)
    rows = np.array([stream.step(p) for p in path_from(ik, q0, [0.05, 0.05, 0.0], 15)])
    assert np.all(steps(rows, q0) <= ik.max_step(RATE, 1.0) + 1e-12)
    assert np.linalg.norm(ik.chain.fk(rows[-1])[:3, 3] - path_from(ik, q0, [0.05, 0.05, 0.0], 15)[-1][:3, 3]) < 5e-3


def test_small_tool_move_near_a_singularity_is_a_small_joint_move(ik):
    # The most singular pose one joint away from rest (the elbow folded or stretched to its limit).
    def smallest(q):
        return np.linalg.svd(ik.chain.jacobian(q), compute_uv=False)[-1]

    candidates = []
    for j in range(ik.n):
        for v in np.linspace(ik.lo[j], ik.hi[j], 41):
            q = ik.rest.copy()
            q[j] = v
            candidates.append(q)
    q0 = min(candidates, key=smallest)
    assert smallest(q0) < 1e-2
    held = TI.TrajectoryIK(ik.chain, (ik.lo, ik.hi), ik.max_velocity, q0, replace(ik.weights, limit_margin=0.0))
    for offset in ([0.001, 0.0, 0.0], [0.0, 0.001, 0.0], [0.0, 0.0, 0.001]):
        result = held.solve(path_from(ik, q0, offset, 20), q0, rate_hz=RATE)
        assert np.abs(result.q - q0).max() < 0.02
        assert result.pos_err[-1] < 2e-3


# ── pieces ──
def test_pose_residual_jacobian_matches_finite_differences(ik):
    rng = np.random.default_rng(1)
    q = np.clip(ik.rest + rng.normal(0, 0.3, (3, ik.n)), ik.lo, ik.hi)
    poses_, jacs = ik.chain.fk_jacobian_many(q)
    targets = poses_ @ kinematics.xyz_rpy_to_pose([0.02, -0.01, 0.03], [0.3, -0.2, 0.4])
    r, J = TI.pose_residual(targets, poses_, jacs)
    for j in range(ik.n):
        d = np.zeros(ik.n)
        d[j] = 1e-6
        up = TI.pose_residual(targets, *ik.chain.fk_jacobian_many(q + d))[0]
        down = TI.pose_residual(targets, *ik.chain.fk_jacobian_many(q - d))[0]
        assert np.allclose((up - down) / 2e-6, J[:, :, j], atol=1e-6)


def test_batched_fk_matches_fk_and_jacobian(ik):
    q = np.clip(ik.rest + np.linspace(-0.3, 0.3, ik.n), ik.lo, ik.hi)
    poses_, jacs = ik.chain.fk_jacobian_many(q[None])
    assert np.allclose(poses_[0], ik.chain.fk(q)) and np.allclose(jacs[0], ik.chain.jacobian(q))


def test_rotation_log_and_exp_round_trip_to_a_half_turn():
    w = np.array([[0.0, 0.0, 0.0], [1e-9, 0, 0], [0.3, -0.2, 0.1], [0, math.pi - 1e-7, 0], [math.pi, 0, 0]])
    back = TI.so3_log(TI.so3_exp(w))
    assert np.allclose(np.abs(back), np.abs(w), atol=1e-6)
    assert np.allclose(TI.so3_exp(back), TI.so3_exp(w), atol=1e-9)


def test_savgol_keeps_a_cubic_and_smooth_poses_keeps_a_line():
    t = np.linspace(0, 1, 30)[:, None]
    cubic = np.hstack([t**3 - t, 2 * t**2, -t])
    assert np.allclose(TI.savgol(cubic, 15, 3), cubic)
    line = TI.straight_line(np.eye(4), kinematics.xyz_rpy_to_pose([0.1, 0, 0], [0, 0, 0.5]), 30, ease=False)
    assert np.allclose(TI.smooth_poses(line), line, atol=1e-9)
    assert TI.smooth_poses(line[:3]) is not None and np.allclose(TI.smooth_poses(line, window=0), line)


def test_extend_poses_continues_the_last_step():
    line = TI.straight_line(np.eye(4), kinematics.xyz_rpy_to_pose([0.1, 0, 0], [0, 0, 0.5]), 10, ease=False)
    longer = TI.straight_line(np.eye(4), kinematics.xyz_rpy_to_pose([0.15, 0, 0], [0, 0, 0.75]), 15, ease=False)
    assert np.allclose(TI.extend_poses(line, 5), longer, atol=1e-12)


def test_seam_blend_eases_in_from_the_last_command():
    q = np.tile(np.linspace(0, 1, 10)[:, None], (1, 2))
    out = TI.seam_blend(q, np.array([0.5, 0.5]), 4, -10, 10, np.array([0.2, 0.2]))
    assert np.all(np.abs(np.diff(np.vstack([[0.5, 0.5], out]), axis=0)) <= 0.2 + 1e-12)
    assert np.allclose(out[5:], q[5:])


def test_profile_ik_settings_are_validated(profile):
    for bad in ({"nope": 1}, {"pos": -1}, {"max_speed": [1.0] * 99}, {"max_speed": [0.0]}, {"rest_pose": "x"}):
        with pytest.raises(ValueError, match="ik"):
            replace(profile, ik=bad)._validate()
    with pytest.raises(ValueError, match="ik.rest_pose"):
        TI.TrajectoryIK.from_config(replace(profile, ik={"rest_pose": [0.0]}))
    assert TI.TrajectoryIK.from_config(replace(profile, ik={"pos": 20.0})).weights.pos == 20.0
