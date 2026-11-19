"""Gravity feed-forward: the URDF model reproduces this repo's reference torques
(tests/unit/gravity_fixtures.json), and the ease-in / stale-feedback / cap guards hold."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from chain_fixture import CASES, LIMITS, ROOT, SPEC, load, repo_chain

gravity = load("gravity")
GravityFeedforward = gravity.GravityFeedforward

CAPS = SPEC["gravity_limit_nm"]
FACTORS = SPEC["gravity_factor"]
N = len(CAPS)
FRESH = [0.0] * N
POSE = np.mean(LIMITS, axis=0).tolist()


def feedforward(scale=1.0, caps=CAPS, level=None):
    ff = GravityFeedforward(repo_chain(tcp=False), scale, caps, joint_scale=FACTORS)
    if level is not None:
        ff.level = level
    return ff


# ── model ──
@pytest.mark.parametrize("case", CASES, ids=lambda c: str(c["q_deg"]))
def test_model_matches_the_reference(case):
    assert feedforward().model(np.radians(case["q_deg"])) == pytest.approx(case["tau_nm"], abs=1e-9)


def test_model_is_the_gradient_of_potential_energy_times_the_factors():
    chain = repo_chain(tcp=False)
    rng = np.random.default_rng(4)
    for _ in range(30):
        q = rng.uniform(*LIMITS)
        numeric = [(chain.potential_energy(q + h) - chain.potential_energy(q - h)) / 2e-6
                   for h in np.eye(N) * 1e-6]
        assert feedforward().model(q) == pytest.approx(np.multiply(numeric, FACTORS), abs=1e-6)


def test_model_is_scaled_and_periodic():
    q = np.array(POSE)
    assert feedforward(0.5).model(q) == pytest.approx([t / 2 for t in feedforward().model(q)])
    turned = q.copy()
    turned[0] += 2 * math.pi
    assert feedforward().model(turned) == pytest.approx(feedforward().model(q))


def test_model_of_a_non_finite_pose_is_nan_not_an_error():
    assert all(math.isnan(t) for t in feedforward().model([math.inf, *POSE[1:]]))


def test_cap_is_not_reached_inside_the_soft_limits():
    rng = np.random.default_rng(20261103)
    ff = feedforward()
    worst = np.zeros(N)
    for _ in range(3000):
        worst = np.maximum(worst, np.abs(ff.model(rng.uniform(*LIMITS))))
    assert np.all(worst < CAPS)


# ── ease-in and guards ──
def test_ease_in_starts_at_zero_and_takes_the_ramp_time():
    ff = feedforward()
    full = ff.model(POSE)
    assert ff.torques(POSE, FRESH, POSE, 10.0) == pytest.approx([0.0] * N)
    frames = int(gravity.RAMP_S / 0.02)
    for k in range(1, frames):
        tau = ff.torques(POSE, FRESH, POSE, 10.0 + k * 0.02)
        assert tau == pytest.approx([t * k / frames for t in full])
    assert ff.torques(POSE, FRESH, POSE, 10.0 + frames * 0.02) == pytest.approx(full)
    assert ff.level == 1.0


def test_a_pause_never_skips_the_ease_in():
    ff = feedforward()
    ff.torques(POSE, FRESH, POSE, 0.0)
    ff.torques(POSE, FRESH, POSE, 100.0)
    assert ff.level == pytest.approx(gravity.RAMP_STEP_S / gravity.RAMP_S)


def test_a_clock_step_backwards_does_not_advance_or_undo_the_ease_in():
    ff = feedforward()
    ff.torques(POSE, FRESH, POSE, 10.0)
    ff.torques(POSE, FRESH, POSE, 10.02)
    level = ff.level
    ff.torques(POSE, FRESH, POSE, 5.0)
    assert ff.level == level


def test_reset_starts_the_ease_in_again():
    ff = feedforward(level=1.0)
    ff.reset()
    assert ff.torques(POSE, FRESH, POSE, 0.0) == [0.0] * N


def test_measured_pose_is_used_while_every_joint_is_fresh():
    goal = [q + 0.3 for q in POSE]
    ages = [gravity.STALE_S] * N
    assert feedforward(level=1).torques(POSE, ages, goal, 0.0) == pytest.approx(feedforward().model(POSE))


@pytest.mark.parametrize("stale_joint", range(N))
def test_one_stale_joint_falls_back_to_the_goal(stale_joint):
    goal = [q + 0.3 for q in POSE]
    ages = [0.0] * N
    ages[stale_joint] = gravity.STALE_S + 0.001
    assert feedforward(level=1).torques(POSE, ages, goal, 0.0) == pytest.approx(feedforward().model(goal))


def test_non_finite_measurement_falls_back_to_the_goal():
    measured = [math.nan, *POSE[1:]]
    assert feedforward(level=1).torques(measured, FRESH, POSE, 0.0) == pytest.approx(feedforward().model(POSE))


def test_non_finite_pose_everywhere_gives_zero_torque():
    bad = [math.nan] * N
    assert feedforward(level=1).torques(bad, FRESH, bad, 0.0) == [0.0] * N


def test_cap_is_per_joint_and_both_signs(monkeypatch):
    ff = feedforward(level=1)
    monkeypatch.setattr(ff, "model", lambda q: [100.0 * (-1) ** i for i in range(N)])
    assert ff.torques(POSE, FRESH, POSE, 0.0) == [CAPS[i] * (-1) ** i for i in range(N)]


def test_a_single_non_finite_torque_becomes_zero(monkeypatch):
    ff = feedforward(level=1)
    values = [0.01 * (i + 1) for i in range(N)]
    values[1], values[2] = math.nan, math.inf
    monkeypatch.setattr(ff, "model", lambda q: values)
    expected = [0.0 if not math.isfinite(v) else min(v, c) for v, c in zip(values, CAPS)]
    assert ff.torques(POSE, FRESH, POSE, 0.0) == expected


@pytest.mark.parametrize("scale", [-0.1, 1.21, math.nan])
def test_scale_out_of_range_is_refused(scale):
    with pytest.raises(ValueError, match="gravity_scale"):
        feedforward(scale)


@pytest.mark.parametrize("caps", [None, [1.0], [math.inf] * 6, [-1.0] * 6])
def test_caps_must_be_one_finite_value_per_joint(caps):
    with pytest.raises(ValueError, match="limits"):
        feedforward(caps=caps)


# ── profile ──
def profile(**overrides):
    joints = [SimpleNamespace(name=f"j{i}", gravity_limit_nm=CAPS[i], gravity_factor=FACTORS[i],
                              urdf_sign=SPEC["sign"][i], urdf_offset=math.radians(SPEC["offset_deg"][i]))
              for i in range(N)]
    joints.append(SimpleNamespace(name="gripper"))
    fields = dict(urdf=str(ROOT / SPEC["urdf"]),
                  gravity_scale=1.0, joints=joints, urdf_base_rpy=np.radians(SPEC["base_rpy_deg"]).tolist())
    return SimpleNamespace(**(fields | overrides))


def test_from_config_builds_the_profile_model():
    ff = gravity.from_config(profile())
    assert ff.n == N
    for case in CASES[:5]:
        assert ff.model(np.radians(case["q_deg"])) == pytest.approx(case["tau_nm"], abs=1e-9)


@pytest.mark.parametrize("overrides", [dict(urdf=None), dict(urdf=""), dict(gravity_scale=0.0)])
def test_from_config_is_off_without_a_urdf_or_at_scale_zero(overrides):
    assert gravity.from_config(profile(**overrides)) is None


def test_from_config_needs_a_cap_on_every_arm_joint():
    config = profile()
    del config.joints[1].gravity_limit_nm
    with pytest.raises(ValueError, match="gravity_limit_nm"):
        gravity.from_config(config)
