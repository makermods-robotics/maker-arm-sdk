"""``Arm.move_ee``'s path planning, run in a helper process.

Solving a Cartesian path is pure Python and numpy: a 2-3 cm move takes about 0.9 s on a Cortex-A72 and 2.8 s on a
Cortex-A53. Solved in the process that owns an enabled arm, it holds the interpreter lock that the arm's control loop
and CAN receive thread need many times per tick; on a small board the loop fell to 20 Hz and the arm faulted on
stale feedback. Here the solve runs in a separate Python process (one per concurrent call, reused), so the arm's
threads never wait on it.

The helper is started with ``python -c "from <package> import _planner; _planner._serve()"``: it never imports the
caller's main module, so a script without an ``if __name__ == "__main__"`` guard is safe.

Planning is bounded. A path longer than ``MAX_PATH_TICKS`` is refused before it is built. A helper has
``PLAN_START_S`` to start and solve the end pose; it then reports how many ticks the path has, and gets
``PLAN_MARGIN_S`` plus ``PLAN_S_PER_TICK`` per tick to solve it (again for each slower retiming), never more than
``PLAN_CEILING_S`` in all. The caller's ``limit`` and ``cancel`` apply on top. A helper that runs out of time, is
cancelled, dies or cannot be talked to is killed and the plan is refused (``EEPlan.q`` None, with the reason); the
next plan starts a fresh helper. Only when no helper process can be created (no subprocess support, or none for the
moment) is that plan solved in this process, under the same limits, checked between chunks of the path and once
more at the end; the next plan tries a helper again.
"""

from __future__ import annotations

import atexit
import logging
import math
import os
import pickle
import queue
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np

from . import kinematics, trajectory_ik

log = logging.getLogger(__package__)

# move_ee: how far the planned path may end from the goal (m, rad).
EE_POSITION_TOL = 1e-3
EE_ANGLE_TOL = math.radians(0.5)
EE_MIN_SECONDS = 0.25   # shortest path
PLAN_START_S = 5.0       # a helper must start (about 2 s on a Cortex-A53) and solve the end pose within this
PLAN_S_PER_TICK = 0.025  # path solve time allowed per tick (about 8 ms on a Cortex-A72, 21 ms on a Cortex-A53)
PLAN_MARGIN_S = 3.0      # added to the per-tick allowance
PLAN_CEILING_S = 60.0    # the most any one plan may take, all retimings included
MAX_PATH_TICKS = 12000   # the longest path planned (60 s at 200 Hz); longer moves are refused: split them
_POLL_S = 0.05           # how often a waiting caller looks at its cancel event
_HEADER = struct.Struct("<I")


class PlanStopped(Exception):
    """Planning ran out of time or was cancelled."""


class _Budget:
    """The time one plan may take: PLAN_START_S until the path length is known, then PLAN_MARGIN_S plus
    PLAN_S_PER_TICK per tick, never past PLAN_CEILING_S or the caller's ``limit``; ``cancel`` (anything with
    ``is_set()``) stops it at once."""

    def __init__(self, limit: Optional[float] = None, cancel=None):
        self.started = time.monotonic()
        self.cancel = cancel
        self.ceiling = self.started + PLAN_CEILING_S
        self.caller = None if limit is None else self.started + limit
        self.allowed = self.started + PLAN_START_S

    def ticks(self, n: int) -> None:
        self.allowed = time.monotonic() + PLAN_MARGIN_S + n * PLAN_S_PER_TICK

    def end(self) -> float:
        return min(t for t in (self.allowed, self.ceiling, self.caller) if t is not None)

    def check(self) -> None:
        """PlanStopped when cancelled or out of time."""
        now = time.monotonic()
        if self.cancel is not None and self.cancel.is_set():
            raise PlanStopped(f"path planning cancelled after {now - self.started:.2f} s")
        if now >= self.end():
            raise PlanStopped(f"path planning timed out after {now - self.started:.2f} s")


@dataclass
class EEPlan:
    q: Optional[np.ndarray]     # (T, chain.n) joint commands, one per control tick; None when refused
    reason: str = ""            # why it was refused


def plan_ee(chain: kinematics.Chain, ik: trajectory_ik.TrajectoryIK, target: Sequence[float],
            rpy: Optional[Sequence[float]], q0: np.ndarray, rate: float, speed: Optional[float],
            progress: Optional[Callable[[int], None]] = None,
            check: Optional[Callable[[], None]] = None) -> EEPlan:
    """The straight-line tool path from the chain pose ``q0`` to ``target`` (and ``rpy``), solved per control tick.
    ``progress`` is called with the number of ticks before each solve of the path, ``check`` between its chunks
    (it may raise to stop the solve). Paths longer than MAX_PATH_TICKS are refused before they are built."""
    target = [float(v) for v in target]
    rpy = None if rpy is None else [float(v) for v in rpy]
    q0 = np.asarray(q0, dtype=float)
    end = chain.ik(target, np.clip(q0, ik.lo, ik.hi), rpy=rpy, limits=(ik.lo, ik.hi))
    if not end.ok:
        return EEPlan(None, f"no solution within the limits ({end.pos_err * 1e3:.1f} mm, "
                            f"{math.degrees(end.rot_err):.2f} deg off)")
    start, goal = chain.fk(q0), chain.fk(end.q)
    goal[:3, 3] = target
    if rpy is not None:
        goal[:3, :3] = kinematics.rpy_to_matrix(rpy)
    speeds = np.minimum(ik.max_velocity, math.inf if speed is None else speed)
    # Minimum jerk peaks at 1.875x the mean speed; a path the cap still shortens is retimed slower.
    seconds = max(EE_MIN_SECONDS, 1.25 * float(np.max(1.875 * np.abs(np.asarray(end.q) - q0) / speeds)))
    ticks = int(math.ceil(seconds * rate))
    for _ in range(4):
        # the goal held a while at the end lets the smoothed solve settle onto it
        hold = max(int(math.ceil(EE_MIN_SECONDS * rate / 2)), ticks // 5)
        if ticks + hold > MAX_PATH_TICKS:
            return EEPlan(None, f"the path needs {(ticks + hold) / rate:.0f} s at this speed, longer than the "
                                f"{MAX_PATH_TICKS / rate:.0f} s planned at most: split the move or raise the speed")
        poses = trajectory_ik.straight_line(start, goal, ticks)
        poses = np.concatenate([poses, np.repeat(goal[None], hold, axis=0)])
        if progress is not None:
            progress(len(poses))
        plan = trajectory_ik.solve_path(ik, poses, q0, rate, speed, between=check)
        if not plan.capped.any():
            break
        ticks = int(math.ceil(ticks * 1.5))
    if plan.pos_err[-1] > EE_POSITION_TOL or plan.rot_err[-1] > EE_ANGLE_TOL:
        return EEPlan(None, f"the path ends {plan.pos_err[-1] * 1e3:.1f} mm, "
                            f"{math.degrees(plan.rot_err[-1]):.2f} deg off the goal")
    return EEPlan(np.asarray(plan.q))


# ── the helper process ──
def _send(stream, obj) -> None:
    data = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    stream.write(_HEADER.pack(len(data)) + data)
    stream.flush()


def _receive(stream):
    header = stream.read(_HEADER.size)
    if len(header) < _HEADER.size:
        raise EOFError("the planning helper closed its pipe")
    size, = _HEADER.unpack(header)
    data = stream.read(size)
    if len(data) < size:
        raise EOFError("the planning helper closed its pipe")
    return pickle.loads(data)


class _Helper:
    """One helper process; one request at a time."""

    def __init__(self, code: Optional[str] = None):
        package_root = str(Path(__file__).resolve().parent.parent)
        env = dict(os.environ)
        env["PYTHONPATH"] = package_root + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        for var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
            env.setdefault(var, "1")   # one small solve: a BLAS thread pool would only take cores from the arm
        code = code or f"from {__package__} import _planner; _planner._serve()"
        self.proc = subprocess.Popen([sys.executable, "-c", code],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env, close_fds=True)

    def _exchange(self, request, out: queue.Queue) -> None:
        # Runs in its own thread, so the caller can stop waiting at any time; killing the helper ends it.
        try:
            _send(self.proc.stdin, request)
            while True:
                message = _receive(self.proc.stdout)
                out.put(message)
                if message[0] != "ticks":
                    return
        except Exception as e:  # noqa: BLE001 - handed to the caller
            out.put(("closed", e))

    def call(self, request, budget: _Budget):
        """The helper's final reply, ("ok", plan) or ("error", exception). PlanStopped when ``budget`` runs out or
        is cancelled; the exception that broke the pipe if it failed."""
        messages: queue.Queue = queue.Queue()
        threading.Thread(target=self._exchange, args=(request, messages), daemon=True, name="move_ee planner").start()
        while True:
            budget.check()
            try:
                kind, value = messages.get(timeout=max(0.0, min(budget.end() - time.monotonic(), _POLL_S)))
            except queue.Empty:
                continue
            if kind == "closed":
                raise value
            if kind == "ticks":
                budget.ticks(value)
                continue
            return kind, value

    def kill(self) -> None:
        self.proc.kill()
        try:
            self.proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            pass

    def close(self) -> None:
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.proc.kill()


class _Pool:
    def __init__(self):
        self._lock = threading.Lock()
        self._idle: list[_Helper] = []
        self.unavailable: Optional[str] = None   # why the last helper could not be created, if it could not

    def plan(self, args, limit: Optional[float] = None, cancel=None) -> EEPlan:
        budget = _Budget(limit, cancel)
        with self._lock:
            helper = self._idle.pop() if self._idle else None
        if helper is None:
            try:
                helper = _Helper()
                self.unavailable = None
            except (OSError, ValueError, NotImplementedError) as e:
                # No process could be created (no subprocess support, or none for now): solve this plan here, under
                # the same limits; the next plan tries a helper again.
                self.unavailable = str(e) or type(e).__name__
                log.warning("move_ee planning helper could not be started (%s); solving this path in this process",
                            self.unavailable)
        if helper is not None:
            return self._plan_in(helper, args, budget)
        return _plan_here(args, budget)

    def _plan_in(self, helper: _Helper, args, budget: _Budget) -> EEPlan:
        try:
            status, value = helper.call(args, budget)
        except BaseException as e:
            # Stopped, interrupted or broken mid-request: its reply could still arrive, so it is never reused.
            helper.kill()
            if isinstance(e, PlanStopped):
                return EEPlan(None, str(e))
            if not isinstance(e, Exception):
                raise
            reason = str(e) or type(e).__name__
            log.warning("move_ee planning helper failed (%s)", reason)
            return EEPlan(None, f"the planning helper failed ({reason})")
        with self._lock:
            self._idle.append(helper)
        if status == "error":
            raise value   # the solve itself raised, as it would have in this process
        return value

    def close(self) -> None:
        with self._lock:
            idle, self._idle = self._idle, []
        for helper in idle:
            helper.close()


_pool = _Pool()
atexit.register(_pool.close)


def _forget_helpers_after_fork() -> None:
    # A forked child shares the parent's helper pipes: it starts with an empty pool and leaves those helpers to the
    # parent (nothing is closed or killed here).
    _pool._lock = threading.Lock()
    _pool._idle = []


if hasattr(os, "register_at_fork"):
    os.register_at_fork(after_in_child=_forget_helpers_after_fork)


def _plan_here(args, budget: _Budget) -> EEPlan:
    """``plan_ee`` in this process, stopped between chunks of the path when ``budget`` runs out or is cancelled."""
    try:
        budget.check()
        result = plan_ee(*args, progress=budget.ticks, check=budget.check)
        budget.check()   # the last chunk may have crossed the limit: a late plan authorises nothing
        return result
    except PlanStopped as e:
        return EEPlan(None, str(e))


def plan(chain, ik, target, rpy, q0, rate: float, speed: Optional[float], isolated: bool = True,
         limit: Optional[float] = None, cancel=None) -> EEPlan:
    """``plan_ee`` in a helper process (``isolated``), or in this one; either way the plan is refused (q None)
    after ``limit`` seconds, when ``cancel`` (anything with ``is_set()``) is set, past the planning allowance
    (module docstring), or when the helper fails."""
    args = (chain, ik, target, rpy, q0, rate, speed)
    return _pool.plan(args, limit, cancel) if isolated else _plan_here(args, _Budget(limit, cancel))


def _serve() -> None:
    # Replies go out on a private copy of stdout; anything printed by imported code goes to stderr.
    out = os.fdopen(os.dup(sys.stdout.fileno()), "wb")
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    inp = sys.stdin.buffer
    while True:
        try:
            request = _receive(inp)
        except EOFError:
            return
        try:
            reply = ("ok", plan_ee(*request, progress=lambda ticks: _send(out, ("ticks", ticks))))
        except Exception as e:  # noqa: BLE001 - handed back to the caller
            reply = ("error", e)
        try:
            _send(out, reply)
        except (pickle.PickleError, TypeError, AttributeError):
            _send(out, ("error", RuntimeError(repr(reply[1]))))
        except OSError:
            return

