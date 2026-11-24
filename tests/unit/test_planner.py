"""move_ee's path planning in a helper process: same plan as in this process, refusals and errors carried back,
a fallback when no helper can run, and the caller's main module never imported by the helper."""

import math
import subprocess
import sys

import numpy as np
import pytest

from conftest import sdk
from chain_fixture import repo_chain

planner, trajectory_ik = sdk._planner, sdk.trajectory_ik


@pytest.fixture
def setup():
    chain = repo_chain()
    ik = trajectory_ik.TrajectoryIK(chain, limits=(chain.lower, chain.upper), max_velocity=2.0)
    q0 = np.clip((ik.lo + ik.hi) / 2, ik.lo, ik.hi)
    start = chain.fk(q0)[:3, 3]
    return chain, ik, q0, start


def test_helper_plan_equals_the_in_process_plan(setup):
    chain, ik, q0, start = setup
    target = (start + [0.0, 0.0, 0.02]).tolist()
    here = planner.plan(chain, ik, target, None, q0, 50.0, None, isolated=False)
    there = planner.plan(chain, ik, target, None, q0, 50.0, None, isolated=True)
    assert here.q is not None and there.q is not None
    assert np.allclose(here.q, there.q, atol=1e-9, rtol=0)   # BLAS thread counts may differ in the last bits
    assert planner._pool.unavailable is None


def test_helper_carries_refusals_back(setup):
    chain, ik, q0, _ = setup
    result = planner.plan(chain, ik, [5.0, 0.0, 5.0], None, q0, 50.0, None)
    assert result.q is None and "no solution" in result.reason


def test_helper_carries_errors_back(setup):
    chain, ik, q0, start = setup
    with pytest.raises(ValueError):
        planner.plan(chain, ik, start.tolist(), None, np.zeros(chain.n + 1), 50.0, None)
    # the helper is still usable afterwards
    assert planner.plan(chain, ik, (start + [0, 0, 0.01]).tolist(), None, q0, 50.0, None).q is not None


def test_falls_back_to_this_process_when_no_helper_starts(setup, monkeypatch, caplog):
    chain, ik, q0, start = setup
    pool = planner._Pool()
    monkeypatch.setattr(planner, "_pool", pool)
    monkeypatch.setattr(planner.sys, "executable", "/nonexistent/python")
    with caplog.at_level("WARNING"):
        result = planner.plan(chain, ik, (start + [0, 0, 0.01]).tolist(), None, q0, 50.0, None)
    assert result.q is not None
    assert pool.unavailable and "helper failed" in caplog.text


def test_helper_never_imports_the_callers_main_module(tmp_path):
    # A script without a main guard: if the helper imported it, the marker file would appear twice.
    marker = tmp_path / "ran.txt"
    script = tmp_path / "unguarded.py"
    script.write_text(f"""
import numpy as np
open({str(marker)!r}, "a").write("x")
from {sdk.__name__} import _planner, trajectory_ik
import sys; sys.path.insert(0, {str(tmp_path.parent)!r})
""" + """
from chain_fixture import repo_chain
chain = repo_chain()
ik = trajectory_ik.TrajectoryIK(chain, limits=(chain.lower, chain.upper), max_velocity=2.0)
q0 = (ik.lo + ik.hi) / 2
target = (chain.fk(q0)[:3, 3] + [0, 0, 0.01]).tolist()
assert _planner.plan(chain, ik, target, None, q0, 50.0, None).q is not None
assert _planner._pool.unavailable is None
""")
    tests_dir = str(__import__("pathlib").Path(__file__).resolve().parents[1])
    unit_dir = str(__import__("pathlib").Path(__file__).resolve().parent)
    env = dict(__import__("os").environ, PYTHONPATH=f"{tests_dir}{__import__('os').pathsep}{unit_dir}")
    result = subprocess.run([sys.executable, str(script)], capture_output=True, text=True, env=env, timeout=120)
    assert result.returncode == 0, result.stderr
    assert marker.read_text() == "x"
