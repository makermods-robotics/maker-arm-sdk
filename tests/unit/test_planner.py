"""move_ee's path planning in a helper process: same plan as in this process, refusals and errors carried back,
a fallback only when no helper can run, planning bounded by time and cancel, and the caller's main module never
imported by the helper."""

import math
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from conftest import sdk
from chain_fixture import repo_chain

planner, trajectory_ik = sdk._planner, sdk.trajectory_ik
real_helper_class = planner._Helper


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
    assert pool.unavailable and "could not be started" in caplog.text


# ── bounded planning: helpers that hang, die or are too slow ──
HANG = "import sys, time; sys.stdin.buffer.read(1); time.sleep(60)"
EXIT_AT_ONCE = "import sys; sys.exit(3)"


def patched_serve(patch: str) -> str:
    """Helper code that runs ``patch`` (with ``p`` = the helper's _planner module) before serving."""
    return f"from {sdk.__name__} import _planner as p\n{patch}\np._serve()"


DIE_WHILE_SOLVING = patched_serve(
    "import os\np.trajectory_ik.solve_path = lambda *a, **k: os._exit(3)")
SLOW_SOLVE = patched_serve(
    "import time\nsolve = p.trajectory_ik.solve_path\n"
    "p.trajectory_ik.solve_path = lambda *a, **k: (time.sleep(30), solve(*a, **k))[1]")


@pytest.fixture
def helper_code(monkeypatch):
    """Use a fresh pool whose helpers run the code given to the returned setter; plan_ee in this process fails."""
    pool = planner._Pool()
    monkeypatch.setattr(planner, "_pool", pool)
    real_helper = planner._Helper

    def use(code):
        monkeypatch.setattr(planner, "_Helper", lambda: real_helper(code))

    def no_local_solve(*a, **k):
        raise AssertionError("the plan was solved in this process")
    monkeypatch.setattr(planner, "plan_ee", no_local_solve)
    use.pool = pool
    return use


def timed_plan(setup, **kw):
    chain, ik, q0, start = setup
    t0 = time.monotonic()
    result = planner.plan(chain, ik, (start + [0, 0, 0.01]).tolist(), None, q0, 50.0, None, **kw)
    return result, time.monotonic() - t0


def test_a_hanging_helper_is_stopped_after_the_callers_limit(setup, helper_code):
    helper_code(HANG)
    result, took = timed_plan(setup, limit=0.4)
    assert result.q is None and "timed out after 0.4" in result.reason
    assert 0.35 < took < 1.5
    assert helper_code.pool._idle == [] and helper_code.pool.unavailable is None


def test_a_hanging_helper_is_stopped_after_its_start_allowance(setup, helper_code, monkeypatch):
    helper_code(HANG)
    monkeypatch.setattr(planner, "PLAN_START_S", 0.4)
    result, took = timed_plan(setup)
    assert result.q is None and "timed out" in result.reason and took < 1.5


def test_a_helper_slower_than_its_allowance_per_tick_is_stopped(setup, helper_code, monkeypatch):
    helper_code(SLOW_SOLVE)
    monkeypatch.setattr(planner, "PLAN_MARGIN_S", 0.3)
    monkeypatch.setattr(planner, "PLAN_S_PER_TICK", 0.001)
    result, took = timed_plan(setup)
    assert result.q is None and "timed out" in result.reason
    assert took < planner.PLAN_START_S + 1.0


def test_cancel_stops_planning_at_once(setup, helper_code):
    helper_code(HANG)
    cancel = threading.Event()
    threading.Timer(0.3, cancel.set).start()
    result, took = timed_plan(setup, cancel=cancel)
    assert result.q is None and "cancelled" in result.reason and took < 1.0


def test_a_helper_that_dies_while_solving_fails_the_plan_without_solving_it_here(setup, helper_code, caplog):
    helper_code(DIE_WHILE_SOLVING)
    with caplog.at_level("WARNING"):
        result, took = timed_plan(setup, limit=30.0)
    assert result.q is None and "planning helper failed" in result.reason and took < 10.0
    assert helper_code.pool.unavailable is None   # the next call starts a new helper


def test_a_helper_that_dies_before_reporting_refuses_the_plan_and_the_next_plan_starts_a_fresh_one(
        setup, helper_code, monkeypatch):
    helper_code(EXIT_AT_ONCE)
    result, took = timed_plan(setup, limit=30.0)
    assert result.q is None and "planning helper failed" in result.reason and took < 10.0
    assert helper_code.pool.unavailable is None and helper_code.pool._idle == []
    monkeypatch.setattr(planner, "_Helper", real_helper_class)
    result, _ = timed_plan(setup, limit=30.0)
    assert result.q is not None


def slow_chunks(monkeypatch, seconds=0.2):
    feed = trajectory_ik.IKStream.feed

    def slow(self, *a, **k):
        time.sleep(seconds)
        return feed(self, *a, **k)
    monkeypatch.setattr(trajectory_ik.IKStream, "feed", slow)


@pytest.mark.parametrize("stop", ["timeout", "cancel"])
def test_planning_in_this_process_honours_timeout_and_cancel(setup, monkeypatch, stop):
    monkeypatch.setattr(planner, "_pool", planner._Pool())
    monkeypatch.setattr(planner.sys, "executable", "/nonexistent/python")   # no helper process can be created
    slow_chunks(monkeypatch)
    cancel = threading.Event()
    if stop == "cancel":
        threading.Timer(0.3, cancel.set).start()
    chain, ik, q0, start = setup
    t0 = time.monotonic()
    result = planner.plan(chain, ik, (start + [0, 0, 0.03]).tolist(), None, q0, 200.0, 0.05,
                          limit=0.3 if stop == "timeout" else None, cancel=cancel)
    assert result.q is None and ("timed out" if stop == "timeout" else "cancelled") in result.reason
    assert time.monotonic() - t0 < 1.0
    assert planner._pool.unavailable


def test_a_path_longer_than_the_tick_budget_is_refused_before_it_is_built(setup, monkeypatch):
    chain, ik, q0, start = setup
    monkeypatch.setattr(planner, "MAX_PATH_TICKS", 200)
    monkeypatch.setattr(trajectory_ik, "straight_line", lambda *a: pytest.fail("the path was built"))
    result = planner.plan_ee(chain, ik, (start + [0, 0, 0.03]).tolist(), None, q0, 200.0, 0.05)
    assert result.q is None and "split the move" in result.reason


def test_the_planning_ceiling_bounds_a_plan_across_retimings(setup, helper_code, monkeypatch):
    helper_code(SLOW_SOLVE)
    monkeypatch.setattr(planner, "PLAN_CEILING_S", 0.8)
    monkeypatch.setattr(planner, "PLAN_S_PER_TICK", 10.0)   # the per-tick allowance alone would wait for minutes
    result, took = timed_plan(setup)
    assert result.q is None and "timed out" in result.reason and took < 2.0


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
