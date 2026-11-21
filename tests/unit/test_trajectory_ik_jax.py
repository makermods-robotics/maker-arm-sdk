"""The solver choice (trajectory_ik.from_config) and the JAX solver's lazy import, with stand-in modules; with the
``jax`` extra installed, the JAX solver against the original's answers and the numpy solver."""

import json
import math
import subprocess
import sys
import types
from pathlib import Path

import numpy as np
import pytest

from conftest import sdk

TI, TJ, kinematics, config_mod = sdk.trajectory_ik, sdk.trajectory_ik_jax, sdk.kinematics, sdk.config
FIXTURES = json.loads((Path(__file__).resolve().parent / "trajectory_ik_fixtures.json").read_text())
HAS_EXTRA = TJ.available()


@pytest.fixture(scope="module")
def profile():
    return config_mod.ArmConfig.from_yaml(str(sdk._product.DEFAULT_PROFILE))


# ── choosing the solver ──
def test_importing_the_package_or_the_module_loads_no_jax():
    code = (f"import sys, {sdk.__name__}, {TJ.__name__}; "
            "loaded = [m for m in ('jax', 'dream_ik') if m in sys.modules]; assert not loaded, loaded")
    subprocess.run([sys.executable, "-c", code], check=True)


def test_numpy_on_request(profile):
    ik = TI.from_config(profile, solver="numpy")
    assert type(ik) is TI.TrajectoryIK


def test_unknown_solver(profile):
    with pytest.raises(ValueError, match="solver"):
        TI.from_config(profile, solver="torch")


def test_without_the_extra_auto_is_numpy_and_jax_names_the_extra(profile, monkeypatch):
    monkeypatch.setitem(sys.modules, "dream_ik", None)
    assert not TJ.available()
    assert type(TI.from_config(profile)) is TI.TrajectoryIK
    with pytest.raises(ImportError, match=r"\[jax\]"):
        TI.from_config(profile, solver="jax")


def test_a_broken_install_names_the_extra(profile, monkeypatch):
    monkeypatch.setitem(sys.modules, "dream_ik", types.ModuleType("dream_ik"))   # no solver inside
    monkeypatch.setitem(sys.modules, "dream_ik.solver", None)
    with pytest.raises(ImportError, match=r"'jax' extra"):
        TI.from_config(profile, solver="jax")


def test_auto_is_numpy_even_with_the_extra_and_jax_only_on_request(profile, monkeypatch):
    for name in ("dream_ik", "jax"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    built = []
    monkeypatch.setattr(TJ.JaxTrajectoryIK, "__init__", lambda self, *a, **k: built.append(a))
    assert type(TI.from_config(profile)) is TI.TrajectoryIK and not built
    ik = TI.from_config(profile, solver="jax")
    assert isinstance(ik, TJ.JaxTrajectoryIK) and isinstance(ik, TI.TrajectoryIK)
    chain, (lower, upper), speed, rest = built[0][:4]
    numpy_ik = TI.TrajectoryIK.from_config(profile)
    assert chain.n == numpy_ik.n and np.allclose(speed, numpy_ik.max_velocity)


def test_move_ee_uses_the_numpy_solver():
    assert "numpy" in sdk.arm.Arm.move_ee.__doc__


# ── the JAX solver itself (needs the extra) ──
needs_extra = pytest.mark.skipif(not HAS_EXTRA, reason="the jax extra is not installed")


def poses(rows):
    out = np.broadcast_to(np.eye(4), (len(rows), 4, 4)).copy()
    out[:, :3] = np.asarray(rows, dtype=float).reshape(-1, 3, 4)
    return out


def solve(ik, case, **kw):
    return ik.solve(poses(case["targets"]), case["q_last"], case["q_before"], rate_hz=FIXTURES["rate_hz"],
                    speed_cap=case["speed_cap"], method=case["method"], **kw)


@pytest.fixture(scope="module")
def fixture_chain(tmp_path_factory):
    path = tmp_path_factory.mktemp("urdf") / "fixture.urdf"
    path.write_text(FIXTURES["urdf"])
    return kinematics.Chain.from_urdf(path, joints=FIXTURES["joints"], tip=FIXTURES["tip"],
                                      tcp_xyz=FIXTURES["tcp_xyz"], tcp_rpy=FIXTURES["tcp_rpy"])


@pytest.fixture(scope="module")
def jax_on_fixture(fixture_chain):
    return TJ.JaxTrajectoryIK(fixture_chain, None, FIXTURES["max_speed"], FIXTURES["rest_pose"],
                              rest_weight=FIXTURES["rest_weight"])


@pytest.fixture(scope="module")
def jax_on_profile(profile):
    return TI.from_config(profile, solver="jax")


@needs_extra
@pytest.mark.parametrize("case", FIXTURES["cases"], ids=lambda c: f"{c['method']}-cap{c['speed_cap']}")
def test_jax_reproduces_the_original_solver(jax_on_fixture, case):
    diff = np.abs(solve(jax_on_fixture, case).q - np.asarray(case["q"]))
    assert np.percentile(diff, 95) < 2e-3 and diff.max() < 1e-2


@needs_extra
@pytest.mark.parametrize("case", FIXTURES["cases"], ids=lambda c: f"{c['method']}-cap{c['speed_cap']}")
def test_jax_and_numpy_agree_on_the_profile(profile, jax_on_profile, case):
    """Each solver on its own model: numpy on the profile's chain, JAX on the embodiment's URDF. Where the two
    models differ, the joints differ by what it takes to put the tool at the same pose on each."""
    numpy_ik = TI.TrajectoryIK.from_config(profile)
    rj, rn = solve(jax_on_profile, case), solve(numpy_ik, case)
    targets = poses(case["targets"])
    own_pos, own_rot = TI.tool_errors(numpy_ik.chain, rn.q, targets)
    jax_fk = jax_on_profile._ik.fk(jax_on_profile.to_solver(rj.q), jax_on_profile.sides[0])
    jax_pos = np.linalg.norm(jax_fk[:, :3, 3] - targets[:, :3, 3], axis=-1)
    q_grid = np.random.default_rng(1).uniform(numpy_ik.lo, numpy_ik.hi, (64, numpy_ik.n))
    model_gap, _ = TI.tool_errors(numpy_ik.chain, q_grid,
                                  jax_on_profile._ik.fk(jax_on_profile.to_solver(q_grid), jax_on_profile.sides[0]))
    same_model = model_gap.max() < 1e-4
    assert np.percentile(np.abs(rj.q - rn.q), 95) < (2e-3 if same_model else 0.03)
    assert np.percentile(np.abs(jax_pos - own_pos), 95) < (1e-4 if same_model else 3e-3)
    assert own_rot.max() < math.pi


@needs_extra
def test_stream_on_jax_keeps_the_caps(jax_on_profile):
    ik = jax_on_profile
    q0 = ik.rest.copy()
    start = ik.chain.fk(q0)
    goal = start.copy()
    goal[:3, 3] += [0.05, -0.03, 0.04]
    path = TI.straight_line(start, goal, 48)
    stream = TI.IKStream(ik, q0, rate_hz=FIXTURES["rate_hz"], speed_cap=1.0)
    q = np.concatenate([stream.feed(path[:24]).q, stream.feed(path[24:]).q, [stream.step(goal)]])
    steps = np.abs(np.diff(np.vstack([q0, q]), axis=0))
    assert np.all(steps <= ik.max_step(FIXTURES["rate_hz"], 1.0) + 1e-9)
    assert np.all((q >= ik.lo - 1e-9) & (q <= ik.hi + 1e-9))
    assert set(ik.compile_s) >= {24, "tick"}
