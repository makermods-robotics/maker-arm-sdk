"""``Arm.move_ee``'s path planning, run in a helper process.

Solving a Cartesian path is pure Python and numpy: a 2-3 cm move takes about 0.9 s on a Cortex-A72 and 2.8 s on a
Cortex-A53. Solved in the process that owns an enabled arm, it holds the interpreter lock that the arm's control loop
and CAN receive thread need many times per tick; on a small board the loop fell to 20 Hz and the arm faulted on
stale feedback. Here the solve runs in a separate Python process (one per concurrent call, reused), so the arm's
threads never wait on it.

The helper is started with ``python -c "from <package> import _planner; _planner._serve()"``: it never imports the
caller's main module, so a script without an ``if __name__ == "__main__"`` guard is safe. If a helper cannot be started or stops answering (a frozen
application, no subprocess support), the plan is solved in this process instead, as before, with a warning.
"""

from __future__ import annotations

import atexit
import logging
import math
import os
import pickle
import struct
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from . import kinematics, trajectory_ik

log = logging.getLogger(__package__)

# move_ee: how far the planned path may end from the goal (m, rad).
EE_POSITION_TOL = 1e-3
EE_ANGLE_TOL = math.radians(0.5)
EE_MIN_SECONDS = 0.25   # shortest path
PLAN_TIMEOUT_S = 120.0  # a helper that has not answered by then is stopped and the plan solved here
_HEADER = struct.Struct("<I")


@dataclass
class EEPlan:
    q: Optional[np.ndarray]     # (T, chain.n) joint commands, one per control tick; None when refused
    reason: str = ""            # why it was refused


def plan_ee(chain: kinematics.Chain, ik: trajectory_ik.TrajectoryIK, target: Sequence[float],
            rpy: Optional[Sequence[float]], q0: np.ndarray, rate: float, speed: Optional[float]) -> EEPlan:
    """The straight-line tool path from the chain pose ``q0`` to ``target`` (and ``rpy``), solved per control tick."""
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


class _Helper:
    """One helper process; one request at a time."""

    def __init__(self):
        package_root = str(Path(__file__).resolve().parent.parent)
        env = dict(os.environ)
        env["PYTHONPATH"] = package_root + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        for var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
            env.setdefault(var, "1")   # one small solve: a BLAS thread pool would only take cores from the arm
        self.proc = subprocess.Popen([sys.executable, "-c", f"from {__package__} import _planner; _planner._serve()"],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env, close_fds=True)
        self.answered = False

    def call(self, request, timeout: float):
        timer = threading.Timer(timeout, self.proc.kill)
        timer.daemon = True
        timer.start()
        try:
            _send(self.proc.stdin, request)
            reply = _receive(self.proc.stdout)
        finally:
            timer.cancel()
        self.answered = True
        return reply

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

    def plan(self, *args) -> EEPlan:
        if self.unavailable is None:
            with self._lock:
                helper = self._idle.pop() if self._idle else None
            try:
                if helper is None:
                    helper = _Helper()
                status, value = helper.call(args, PLAN_TIMEOUT_S)
            except BaseException as e:
                # Interrupted or broken mid-request: its reply could still arrive, so it is never reused.
                if (helper is None or not helper.answered) and isinstance(e, Exception):
                    self.unavailable = str(e) or type(e).__name__   # never worked here: stop trying
                if helper is not None:
                    helper.kill()
                if not isinstance(e, (OSError, EOFError, pickle.PickleError, ValueError)):
                    raise
                log.warning("move_ee planning helper failed (%s); solving the path in this process", e)
            else:
                with self._lock:
                    self._idle.append(helper)
                if status == "error":
                    raise value   # the solve itself raised, as it would have in this process
                return value
        return plan_ee(*args)

    def close(self) -> None:
        with self._lock:
            idle, self._idle = self._idle, []
        for helper in idle:
            helper.close()


_pool = _Pool()
atexit.register(_pool.close)


def plan(chain, ik, target, rpy, q0, rate: float, speed: Optional[float], isolated: bool = True) -> EEPlan:
    """``plan_ee`` in a helper process (``isolated``), or in this one."""
    return _pool.plan(chain, ik, target, rpy, q0, rate, speed) if isolated else plan_ee(
        chain, ik, target, rpy, q0, rate, speed)


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
            reply = ("ok", plan_ee(*request))
        except Exception as e:  # noqa: BLE001 - handed back to the caller
            reply = ("error", e)
        try:
            _send(out, reply)
        except (pickle.PickleError, TypeError, AttributeError):
            _send(out, ("error", RuntimeError(repr(reply[1]))))
        except OSError:
            return

