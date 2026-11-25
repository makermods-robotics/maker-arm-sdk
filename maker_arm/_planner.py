"""``Arm.move_ee``'s path planning, run in a helper process.

Solving a Cartesian path is pure Python and numpy: a 2-3 cm move takes about 0.9 s on a Cortex-A72 and 2.8 s on a
Cortex-A53. Solved in the process that owns an enabled arm, it holds the interpreter lock that the arm's control loop
and CAN receive thread need many times per tick; on a small board the loop fell to 20 Hz and the arm faulted on
stale feedback. Here the solve runs in a separate Python process (one per concurrent call, reused), so the arm's
threads never wait on it.

The helper is started with ``python -c "from <package> import _planner; _planner._serve()"``: it never imports the
caller's main module, so a script without an ``if __name__ == "__main__"`` guard is safe.

Planning is bounded. A helper has ``PLAN_START_S`` to start and solve the end pose; it then reports how many ticks the
path has, and gets ``PLAN_MARGIN_S`` plus ``PLAN_S_PER_TICK`` per tick to solve it (again for each slower retiming).
The caller's ``limit`` and ``cancel`` apply on top. A helper that runs out of time, is cancelled or dies is killed and
the plan is refused (``EEPlan.q`` None, with the reason): the same request is never solved again in this process,
where it would hold the interpreter lock for as long as it takes. Only when no helper can run at all (no subprocess
support, a frozen application whose interpreter cannot run ``-c``) is the plan solved in this process, with a warning.
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
_POLL_S = 0.05           # how often a waiting caller looks at its cancel event
_HEADER = struct.Struct("<I")


@dataclass
class EEPlan:
    q: Optional[np.ndarray]     # (T, chain.n) joint commands, one per control tick; None when refused
    reason: str = ""            # why it was refused


def plan_ee(chain: kinematics.Chain, ik: trajectory_ik.TrajectoryIK, target: Sequence[float],
            rpy: Optional[Sequence[float]], q0: np.ndarray, rate: float, speed: Optional[float],
            progress: Optional[Callable[[int], None]] = None) -> EEPlan:
    """The straight-line tool path from the chain pose ``q0`` to ``target`` (and ``rpy``), solved per control tick.
    ``progress`` is called with the number of ticks before each solve of the path."""
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
        poses = trajectory_ik.straight_line(start, goal, ticks)
        poses = np.concatenate([poses, np.repeat(goal[None], hold, axis=0)])
        if progress is not None:
            progress(len(poses))
        plan = trajectory_ik.solve_path(ik, poses, q0, rate, speed)
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


class PlanStopped(Exception):
    """The helper ran out of time or the caller cancelled; the helper was killed."""


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
        self.worked = False   # it has sent something: it did start

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

    def call(self, request, limit: Optional[float] = None, cancel: Optional[threading.Event] = None):
        """The helper's final reply, ("ok", plan) or ("error", exception). PlanStopped past its time, past
        ``limit`` seconds or when ``cancel`` is set; the exception that broke the pipe if it failed."""
        started = time.monotonic()
        allowed = started + PLAN_START_S
        caller_end = None if limit is None else started + limit
        messages: queue.Queue = queue.Queue()
        threading.Thread(target=self._exchange, args=(request, messages), daemon=True, name="move_ee planner").start()
        while True:
            end = allowed if caller_end is None else min(allowed, caller_end)
            now = time.monotonic()
            if cancel is not None and cancel.is_set():
                raise PlanStopped(f"path planning cancelled after {now - started:.1f} s")
            if now >= end:
                raise PlanStopped(f"path planning timed out after {now - started:.1f} s")
            try:
                kind, value = messages.get(timeout=min(end - now, _POLL_S) if cancel is not None else end - now)
            except queue.Empty:
                continue
            if kind == "closed":
                raise value
            self.worked = True
            if kind == "ticks":
                allowed = time.monotonic() + PLAN_MARGIN_S + value * PLAN_S_PER_TICK
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
        self.unavailable: Optional[str] = None   # why helpers are not used in this process

    def plan(self, args, limit: Optional[float] = None, cancel: Optional[threading.Event] = None) -> EEPlan:
        if self.unavailable is None:
            with self._lock:
                helper = self._idle.pop() if self._idle else None
            try:
                if helper is None:
                    helper = _Helper()
                status, value = helper.call(args, limit, cancel)
            except BaseException as e:
                # Interrupted, stopped or broken mid-request: its reply could still arrive, so it is never reused.
                if helper is not None:
                    helper.kill()
                if isinstance(e, PlanStopped):
                    return EEPlan(None, str(e))
                if not isinstance(e, (OSError, EOFError, pickle.PickleError, ValueError, TypeError, AttributeError)):
                    raise
                reason = str(e) or type(e).__name__
                if helper is not None and helper.worked:
                    log.warning("move_ee planning helper failed (%s)", reason)
                    return EEPlan(None, f"the planning helper failed ({reason})")
                self.unavailable = reason   # no helper can run here: stop trying
                log.warning("move_ee planning helper could not be started (%s); solving paths in this process", reason)
            else:
                with self._lock:
                    self._idle.append(helper)
                if status == "error":
                    raise value   # the solve itself raised, as it would have in this process
                return value
        if cancel is not None and cancel.is_set():
            return EEPlan(None, "path planning cancelled")
        return plan_ee(*args)

    def close(self) -> None:
        with self._lock:
            idle, self._idle = self._idle, []
        for helper in idle:
            helper.close()


_pool = _Pool()
atexit.register(_pool.close)


def plan(chain, ik, target, rpy, q0, rate: float, speed: Optional[float], isolated: bool = True,
         limit: Optional[float] = None, cancel: Optional[threading.Event] = None) -> EEPlan:
    """``plan_ee`` in a helper process (``isolated``), or in this one. In a helper the plan is refused (q None)
    after ``limit`` seconds, when ``cancel`` is set, or when the helper runs past its own allowance or dies."""
    args = (chain, ik, target, rpy, q0, rate, speed)
    return _pool.plan(args, limit, cancel) if isolated else plan_ee(*args)


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

