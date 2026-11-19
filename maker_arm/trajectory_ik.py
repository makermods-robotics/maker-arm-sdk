"""Trajectory inverse kinematics: tool poses to joint commands with a bounded step per tick (numpy only).

The algorithm is Dream Labs' ``dream_ik`` (Apache-2.0, see NOTICE), solved here on this package's ``Chain``.

Per tick: reach the target tool pose (residual ``log(target^-1 @ tool)``, position : orientation weights 50 : 10),
a stiff push back inside the margined joint limits, a light pull toward the rest pose and a pull toward the previous
tick's solution, by Levenberg-Marquardt from that solution. Whole chunk (``method="traj"``, the default): every tick
in one least-squares problem with the same pose, limit and rest costs plus joint velocity, acceleration and jerk along
the chunk and into it from the last commands, warm-started from the per-tick solve. Either way the answer is clipped
to the margined limits, then the command steps toward it under the per-joint speed caps, the step scaled by its
most-limited joint so its direction is kept: no tick moves a joint more than ``max_speed / rate_hz``.

Poses are 4x4 tool poses in the chain's world frame; joints are the chain's joint coordinates (rad).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields, replace
from typing import Optional, Sequence

import numpy as np

from .kinematics import Chain, chain_from_config

# The trajectory solve's orientation weight on an arm with fewer than six free joints, as a share of its own.
FEW_JOINTS_ORI_SCALE = 0.5


@dataclass(frozen=True)
class IKWeights:
    """Cost weights and the limit margin. ``vel``, ``acc``, ``jerk`` act on joint differences between ticks (rad);
    ``history`` multiplies them where a chunk meets the commands already sent."""

    pos: float = 50.0
    ori: float = 10.0
    limit: float = 1000.0
    last_pose: float = 1.0
    vel: float = 5.0
    acc: float = 50.0
    jerk: float = 200.0
    history: float = 5.0
    limit_margin: float = math.radians(2.0)


WEIGHT_FIELDS = tuple(f.name for f in fields(IKWeights))
# Per-joint profile ``ik:`` entries next to the weights.
JOINT_FIELDS = ("rest_pose", "rest_weight", "max_speed")


@dataclass(frozen=True)
class IKResult:
    q: np.ndarray           # (T, dof) joint commands
    q_solution: np.ndarray  # (T, dof) the solve's answer before the speed cap
    pos_err: np.ndarray     # (T,) m, commanded tool position vs target
    rot_err: np.ndarray     # (T,) rad
    capped: np.ndarray      # (T,) bool, the speed cap shortened this tick's step


# ── rotations, vectorized over leading axes ──
def _skew(v: np.ndarray) -> np.ndarray:
    out = np.zeros(v.shape[:-1] + (3, 3))
    out[..., 0, 1], out[..., 0, 2], out[..., 1, 2] = -v[..., 2], v[..., 1], -v[..., 0]
    out[..., 1, 0], out[..., 2, 0], out[..., 2, 1] = v[..., 2], -v[..., 1], v[..., 0]
    return out


def so3_exp(w: np.ndarray) -> np.ndarray:
    """(..., 3) rotation vectors -> (..., 3, 3)."""
    w = np.asarray(w, dtype=float)
    theta = np.linalg.norm(w, axis=-1)[..., None, None]
    small = theta < 1e-8
    t = np.where(small, 1.0, theta)
    a = np.where(small, 1.0 - theta**2 / 6.0, np.sin(t) / t)
    b = np.where(small, 0.5 - theta**2 / 24.0, (1.0 - np.cos(t)) / t**2)
    k = _skew(w)
    return np.eye(3) + a * k + b * (k @ k)


def _quaternion_log(R: np.ndarray) -> np.ndarray:
    """so3_log through the unit quaternion: well conditioned at any angle, half turns included."""
    m00, m11, m22 = R[..., 0, 0], R[..., 1, 1], R[..., 2, 2]
    # Each of the four candidates is well conditioned where its component is the largest.
    cand = np.stack([
        np.stack([1 + m00 + m11 + m22, R[..., 2, 1] - R[..., 1, 2], R[..., 0, 2] - R[..., 2, 0],
                  R[..., 1, 0] - R[..., 0, 1]], -1),
        np.stack([R[..., 2, 1] - R[..., 1, 2], 1 + m00 - m11 - m22, R[..., 0, 1] + R[..., 1, 0],
                  R[..., 0, 2] + R[..., 2, 0]], -1),
        np.stack([R[..., 0, 2] - R[..., 2, 0], R[..., 0, 1] + R[..., 1, 0], 1 - m00 + m11 - m22,
                  R[..., 1, 2] + R[..., 2, 1]], -1),
        np.stack([R[..., 1, 0] - R[..., 0, 1], R[..., 0, 2] + R[..., 2, 0], R[..., 1, 2] + R[..., 2, 1],
                  1 - m00 - m11 + m22], -1),
    ], -2)  # (..., 4 candidates, wxyz)
    pick = np.argmax(np.stack([m00 + m11 + m22, m00, m11, m22], -1), axis=-1)
    q = np.take_along_axis(cand, pick[..., None, None], axis=-2)[..., 0, :]
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    q = np.where(q[..., :1] < 0, -q, q)
    n = np.linalg.norm(q[..., 1:], axis=-1, keepdims=True)
    angle = 2.0 * np.arctan2(n, q[..., :1])
    return np.where(n < 1e-12, 2.0, angle / np.where(n < 1e-12, 1.0, n)) * q[..., 1:]


def so3_log(R: np.ndarray) -> np.ndarray:
    """(..., 3, 3) -> (..., 3) rotation vectors, angle in [0, pi]."""
    R = np.asarray(R, dtype=float)
    vee = np.stack([R[..., 2, 1] - R[..., 1, 2], R[..., 0, 2] - R[..., 2, 0], R[..., 1, 0] - R[..., 0, 1]], -1)
    s = np.linalg.norm(vee, axis=-1) / 2.0
    c = (R[..., 0, 0] + R[..., 1, 1] + R[..., 2, 2] - 1.0) / 2.0
    angle = np.arctan2(s, c)
    w = vee * np.where(s < 1e-9, 0.5, angle / (2.0 * np.where(s < 1e-9, 1.0, s)))[..., None]
    near_half_turn = c < -0.9   # the axis from vee is ill conditioned there
    if np.any(near_half_turn):
        w[near_half_turn] = _quaternion_log(R[near_half_turn])
    return w


def _se3_log(R: np.ndarray, p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(B, 3, 3), (B, 3) -> the twist [v, w] (B, 6) with ``exp(twist)`` the pose, and ``jlog`` (B, 6, 6),
    d log(pose @ exp(xi)) / d xi at xi = 0 (the formulas of jaxlie's SE3.log / SE3.jlog)."""
    w = so3_log(R)
    theta2 = np.einsum("bi,bi->b", w, w)
    small = theta2 < 1e-12
    theta = np.sqrt(np.where(small, 1.0, theta2))
    st, ct = np.sin(theta), np.cos(theta)
    half = np.where(small, 0.5, 1.0 / (2.0 * (1.0 - ct)))
    # V(w)^-1 = I - W / 2 + c W^2, and jlog of SO(3) is its transpose
    c = np.where(small, 1.0 / 12.0, (1.0 - theta * st * half) / np.where(small, 1.0, theta2))
    W = _skew(w)
    WW = W @ W
    v_inv = np.eye(3) - 0.5 * W + c[:, None, None] * WW
    twist = np.concatenate([np.einsum("bij,bj->bi", v_inv, p), w], axis=-1)
    jlog_so3 = np.swapaxes(v_inv, 1, 2)
    inv2 = 1.0 / theta**2
    beta = inv2 - st / theta * half
    beta_dot = -2.0 * inv2**2 + (1.0 + st / theta) * inv2 * half
    wp = np.einsum("bi,bi->b", w, p)
    v3 = (beta_dot * wp)[:, None] * w - (theta2 * beta_dot + 2.0 * beta)[:, None] * p
    P = 0.5 * _skew(p)
    C = (np.einsum("bi,bj->bij", v3, w) + beta[:, None, None] * np.einsum("bi,bj->bij", w, p)
         + (wp * beta)[:, None, None] * np.eye(3) + P)
    jlog = np.zeros((len(w), 6, 6))
    jlog[:, :3, :3] = jlog_so3
    jlog[:, 3:, 3:] = jlog_so3
    jlog[:, :3, 3:] = np.where(small[:, None, None], P, C @ jlog_so3)
    return twist, jlog


def pose_residual(targets: np.ndarray, poses: np.ndarray, jacobians: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``log(target^-1 @ pose)`` (T, 6) and its Jacobian (T, 6, N) in the joints, from the chain's world-frame
    Jacobians (T, 6, N)."""
    Rt = np.swapaxes(targets[:, :3, :3], 1, 2)
    R_err = Rt @ poses[:, :3, :3]
    p_err = np.einsum("tij,tj->ti", Rt, poses[:, :3, 3] - targets[:, :3, 3])
    r, jlog = _se3_log(R_err, p_err)
    Rt_tool = np.swapaxes(poses[:, :3, :3], 1, 2)
    body = np.concatenate([Rt_tool @ jacobians[:, :3], Rt_tool @ jacobians[:, 3:]], axis=1)
    return r, jlog @ body


def tool_errors(chain: Chain, q: np.ndarray, targets: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Position (m) and angle (rad) of the tool at each row of q (T, N) from targets (T, 4, 4)."""
    poses, _ = chain.fk_jacobian_many(q)
    pos = np.linalg.norm(poses[:, :3, 3] - targets[:, :3, 3], axis=-1)
    rot = np.linalg.norm(so3_log(poses[:, :3, :3] @ np.swapaxes(targets[:, :3, :3], 1, 2)), axis=-1)
    return pos, rot


def _levenberg_marquardt(x, evaluate, lo, hi, weight, max_iters=100, rtol=1e-10, lam=1e-3):
    """Minimise a sum of squares from ``x``: ``evaluate(x) -> (cost, J^T r, J^T J)`` of the smooth costs, plus the
    stiff limit cost ``weight * max(0, x - hi)`` and ``weight * max(0, lo - x)`` per coordinate. Marquardt damping;
    a step that would cross a limit is re-solved with that limit's (linear) residual in the model."""
    w2 = weight**2

    def total(x):
        cost, g, H = evaluate(x)
        over, under = np.maximum(0.0, x - hi), np.maximum(0.0, lo - x)
        return cost + w2 * (over @ over + under @ under), g + w2 * (over - under), H

    cost, g, H = total(x)
    for _ in range(max_iters):
        diag = np.diag(H) + 1e-12
        upper, lower = x > hi, x < lo
        for _ in range(4):   # grow the set of limits the step meets until it stops crossing new ones
            active = upper | lower
            bound = np.where(upper, hi, lo)
            A = H + np.diag(lam * (diag + w2 * active) + w2 * active)
            dx = np.linalg.solve(A, -(g - w2 * (np.maximum(0.0, x - hi) - np.maximum(0.0, lo - x))
                                      + w2 * active * (x - bound)))
            new_upper, new_lower = (x + dx > hi) & ~active, (x + dx < lo) & ~active
            if not (new_upper.any() or new_lower.any()):
                break
            upper, lower = upper | new_upper, lower | new_lower
        if -(2.0 * g @ dx + dx @ H @ dx) <= rtol * cost:   # converged: the model gains nothing more
            break
        x_new = x + dx
        cost_new, g_new, H_new = total(x_new)
        if cost_new < cost:
            done = cost - cost_new <= rtol * cost
            x, cost, g, H = x_new, cost_new, g_new, H_new
            lam = max(lam * 0.1, 1e-9)
            if done:
                break
        else:
            lam *= 10.0
            if lam > 1e10:
                break
    return x


class TrajectoryIK:
    """Joint commands for tool poses on ``chain``. ``limits`` (lower, upper) in joint coordinates (default: the
    chain's URDF limits), ``weights.limit_margin`` is kept inside them; ``max_velocity`` rad/s per joint (or one
    value); ``rest`` the pose the solve leans toward (default: zeros, clipped into the limits), with ``rest_weight``
    per joint (default 0.15 on the first joint, 0.3 on the others)."""

    def __init__(self, chain: Chain, limits=None, max_velocity=math.inf, rest=None,
                 weights: IKWeights = IKWeights(), rest_weight=None):
        n = chain.n
        self.chain, self.n, self.weights = chain, n, weights
        lower, upper = (chain.lower, chain.upper) if limits is None else limits
        lower = np.broadcast_to(np.asarray(lower, dtype=float), (n,))
        upper = np.broadcast_to(np.asarray(upper, dtype=float), (n,))
        if np.any(lower > upper):
            raise ValueError(f"limits need lower <= upper, got {lower.tolist()} {upper.tolist()}")
        lo, hi = lower + weights.limit_margin, upper - weights.limit_margin
        mid = (lower + upper) / 2
        self.lo, self.hi = np.where(lo > hi, mid, lo), np.where(lo > hi, mid, hi)
        self.max_velocity = np.broadcast_to(np.asarray(max_velocity, dtype=float), (n,)).copy()
        if not np.all(self.max_velocity > 0):
            raise ValueError(f"max_velocity must be >0, got {self.max_velocity.tolist()}")
        rest = np.zeros(n) if rest is None else np.asarray(rest, dtype=float)
        if rest.shape != (n,):
            raise ValueError(f"rest needs {n} values")
        self.rest = np.clip(rest, self.lo, self.hi)
        self.rest_weight = (np.array([0.15] + [0.3] * (n - 1)) if rest_weight is None
                            else np.broadcast_to(np.asarray(rest_weight, dtype=float), (n,)).copy())
        self.free = int(np.sum(self.hi > self.lo))

    @classmethod
    def from_config(cls, config, weights: Optional[IKWeights] = None,
                    chain: Optional[Chain] = None) -> "TrajectoryIK":
        """From an arm profile: its chain (or ``chain``, built from it), its soft limits less ``limit_margin`` (the
        ranges the arm is driven within; the URDF's own limits are not read), its ``max_velocity`` and its ``ik:``
        mapping (weights, ``rest_pose``, ``rest_weight``, ``max_speed`` per joint, each no higher than
        ``max_velocity``)."""
        chain = chain_from_config(config) if chain is None else chain
        if chain is None:
            raise ValueError("the profile names no urdf")
        joints = list(config.joints)[:chain.n]
        lower = [j.lo + config.limit_margin for j in joints]
        upper = [j.hi - config.limit_margin for j in joints]
        ik = dict(getattr(config, "ik", None) or {})
        for name in JOINT_FIELDS:
            if name in ik and len(ik[name]) != chain.n:
                raise ValueError(f"ik.{name} needs {chain.n} values, got {len(ik[name])}")
        if weights is None:
            weights = replace(IKWeights(), **{k: float(v) for k, v in ik.items() if k in WEIGHT_FIELDS})
        rest = ik.get("rest_pose")
        if rest is None and getattr(config, "rest_pose", None):
            rest = config.rest_pose[:chain.n]
        speed = np.minimum(np.asarray(ik.get("max_speed", [config.max_velocity] * chain.n), dtype=float),
                           config.max_velocity)
        return cls(chain, (lower, upper), speed, rest, weights, ik.get("rest_weight"))

    # ── limits and caps ──
    def bounds(self, q_last) -> tuple[np.ndarray, np.ndarray]:
        """Margined limits from ``q_last``: a joint already past its margin may stay there, never go further."""
        q_last = np.asarray(q_last, dtype=float)
        return np.minimum(self.lo, q_last), np.maximum(self.hi, q_last)

    def max_step(self, rate_hz: float, speed_cap: Optional[float] = None) -> np.ndarray:
        """(dof,) rad per tick: the arm's speeds, or ``speed_cap`` (rad/s) where lower."""
        if not rate_hz > 0:
            raise ValueError(f"rate_hz must be >0, got {rate_hz}")
        cap = math.inf if speed_cap is None else float(speed_cap)
        if not cap > 0:
            raise ValueError(f"speed_cap must be >0, got {speed_cap}")
        return np.minimum(self.max_velocity, cap) / rate_hz

    @staticmethod
    def cap(q_solution: np.ndarray, q_last, max_step: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Step toward each row from the command before, the step scaled by its most-limited joint."""
        q, capped = np.empty_like(q_solution), np.zeros(len(q_solution), bool)
        prev = np.asarray(q_last, dtype=float)
        for t, target in enumerate(q_solution):
            step = target - prev
            scale = min(1.0, float(np.min(max_step / np.maximum(np.abs(step), 1e-12))))
            prev = prev + scale * step
            q[t], capped[t] = prev, scale < 1.0
        return q, capped

    # ── the solves ──
    def solve_tick(self, target: np.ndarray, q_prev, lo=None, hi=None) -> np.ndarray:
        """One pose from the previous tick's solution ``q_prev``: the per-tick costs, clipped to [lo, hi]
        (default: ``bounds(q_prev)``)."""
        w = self.weights
        q_prev = np.asarray(q_prev, dtype=float)
        if lo is None or hi is None:
            lo, hi = self.bounds(q_prev)
        target = np.asarray(target, dtype=float)[None]
        scale = np.array([w.pos] * 3 + [w.ori] * 3)
        diag = self.rest_weight**2 + w.last_pose**2
        pull = self.rest_weight**2 * self.rest + w.last_pose**2 * q_prev

        def evaluate(q):
            poses, jacs = self.chain.fk_jacobian_many(q[None])
            r, J = pose_residual(target, poses, jacs)
            r, J = scale * r[0], scale[:, None] * J[0]
            cost = (r @ r + np.sum((self.rest_weight * (q - self.rest))**2)
                    + np.sum((w.last_pose * (q - q_prev))**2))
            return cost, J.T @ r + diag * q - pull, J.T @ J + np.diag(diag)

        return np.clip(_levenberg_marquardt(q_prev.copy(), evaluate, lo, hi, w.limit), lo, hi)

    def solve(self, targets, q_last, q_before=None, *, rate_hz: float, speed_cap: Optional[float] = None,
              method: str = "traj") -> IKResult:
        """Joint commands for ``targets`` (T, 4, 4) continuing from ``q_last`` (dof,), the last command.

        ``q_before``: the commands before it, (dof,) or (rows, dof) newest first; two rows also set the jerk into
        the chunk; None = the arm was still. ``speed_cap`` (rad/s) lowers the arm's speeds. ``method`` "traj"
        solves the chunk as one problem, "tick" tick by tick."""
        targets = np.asarray(targets, dtype=float)
        if targets.ndim != 3 or targets.shape[1:] != (4, 4) or not len(targets):
            raise ValueError(f"targets must be (T, 4, 4), got {targets.shape}")
        if not np.all(np.isfinite(targets)):
            raise ValueError("targets must be finite")
        q_last = np.asarray(q_last, dtype=float)
        if q_last.shape != (self.n,) or not np.all(np.isfinite(q_last)):
            raise ValueError(f"q_last must be {self.n} finite values")
        max_step = self.max_step(rate_hz, speed_cap)
        lo, hi = self.bounds(q_last)
        warm, prev = np.empty((len(targets), self.n)), q_last
        for t, target in enumerate(targets):
            prev = warm[t] = self.solve_tick(target, prev, lo, hi)
        if method == "traj":
            before = q_last[None] if q_before is None else np.asarray(q_before, dtype=float).reshape(-1, self.n)
            q_sol = np.clip(self._solve_traj(targets, warm, q_last, before, lo, hi), lo, hi)
        elif method == "tick":
            q_sol = warm
        else:
            raise ValueError(f"method must be 'traj' or 'tick', got {method!r}")
        q, capped = self.cap(q_sol, q_last, max_step)
        pos, rot = tool_errors(self.chain, q, targets)
        return IKResult(q=q, q_solution=q_sol, pos_err=pos, rot_err=rot, capped=capped)

    def _smoothing(self, T: int, last, before, before2, jerk0: float) -> tuple[np.ndarray, np.ndarray]:
        """The velocity / acceleration / jerk costs as one linear residual ``D @ q - c`` with q (T, dof):
        D (rows, T) per joint, c (rows, dof)."""
        w = self.weights
        hist = {-1: last, -2: before, -3: before2}   # commands before tick 0, by tick index
        rows, consts = [], []

        def add(weight, taps):
            """weight * sum(coef * q[tick]) over taps (tick, coef); ticks < 0 are the known commands."""
            if weight == 0 or any(t >= T for t, _ in taps):
                return
            row, const = np.zeros(T), np.zeros(self.n)
            for t, coef in taps:
                if t >= 0:
                    row[t] += weight * coef
                else:
                    const -= weight * coef * hist[t]
            rows.append(row)
            consts.append(const)

        for k in range(T - 1):
            add(w.vel, [(k + 1, 1.0), (k, -1.0)])
        for k in range(T - 2):
            add(w.acc, [(k + 2, 1.0), (k + 1, -2.0), (k, 1.0)])
        for k in range(T - 3):
            add(w.jerk, [(k + 3, 1.0), (k + 2, -3.0), (k + 1, 3.0), (k, -1.0)])
        h = w.history
        add(h * w.vel, [(0, 1.0), (-1, -1.0)])
        add(h * w.acc, [(0, 1.0), (-1, -2.0), (-2, 1.0)])
        add(h * w.acc, [(1, 1.0), (0, -2.0), (-1, 1.0)])
        add(h * jerk0, [(0, 1.0), (-1, -3.0), (-2, 3.0), (-3, -1.0)])
        add(h * w.jerk, [(1, 1.0), (0, -3.0), (-1, 3.0), (-2, -1.0)])
        add(h * w.jerk, [(2, 1.0), (1, -3.0), (0, 3.0), (-1, -1.0)])
        return np.array(rows).reshape(-1, T), np.array(consts).reshape(-1, self.n)

    def _solve_traj(self, targets, init, last, before, lo, hi) -> np.ndarray:
        """Every tick at once: pose, limit and rest costs per tick plus the smoothing, from ``init`` (T, dof)."""
        w = self.weights
        T, n = init.shape
        jerk0 = w.jerk if len(before) >= 2 else 0.0
        D, c = self._smoothing(T, last, before[0], before[1] if len(before) >= 2 else before[0], jerk0)
        H_s = np.kron(D.T @ D, np.eye(n))
        g_s = (D.T @ c).reshape(-1)
        c_s = float(np.sum(c * c))
        ori = w.ori if self.free >= 6 else w.ori * FEW_JOINTS_ORI_SCALE
        scale = np.array([w.pos] * 3 + [ori] * 3)
        rw2 = self.rest_weight**2
        idx = np.arange(T * n).reshape(T, n)

        def evaluate(x):
            q = x.reshape(T, n)
            poses, jacs = self.chain.fk_jacobian_many(q)
            r, J = pose_residual(targets, poses, jacs)
            r, J = r * scale, J * scale[:, None]
            Hq = H_s @ x
            cost = np.sum(r * r) + np.sum(rw2 * (q - self.rest)**2) + x @ Hq - 2.0 * g_s @ x + c_s
            g = (np.einsum("tki,tk->ti", J, r) + rw2 * (q - self.rest)).reshape(-1) + Hq - g_s
            H = H_s.copy()
            blocks = np.einsum("tki,tkj->tij", J, J)
            blocks[:, np.arange(n), np.arange(n)] += rw2
            H[idx[:, :, None], idx[:, None, :]] += blocks
            return cost, g, H

        lo_x, hi_x = np.broadcast_to(lo, (T, n)).reshape(-1), np.broadcast_to(hi, (T, n)).reshape(-1)
        return _levenberg_marquardt(init.reshape(-1).copy(), evaluate, lo_x, hi_x, w.limit).reshape(T, n)


class IKStream:
    """Successive chunks (``feed``) or single poses (``step``) solved from the commands already sent, so each
    continues the last: the trajectory solve reads the last three commands, a single pose the last one."""

    def __init__(self, ik: TrajectoryIK, q_last, rate_hz: float, speed_cap: Optional[float] = None):
        self.ik, self.rate_hz, self.speed_cap = ik, rate_hz, speed_cap
        self.reset(q_last)

    def reset(self, q_last) -> None:
        """Start over from ``q_last`` with the arm still."""
        q_last = np.asarray(q_last, dtype=float)
        self.history = [q_last]          # newest first, at most three
        self.solution = q_last           # the previous tick's solution (``step``)

    @property
    def q_last(self) -> np.ndarray:
        return self.history[0]

    def _sent(self, rows: np.ndarray, solution: np.ndarray) -> None:
        self.history = ([r for r in rows[::-1]] + self.history)[:3]
        self.solution = solution

    def feed(self, targets, method: str = "traj") -> IKResult:
        """The joint commands for a chunk of poses (T, 4, 4); they become the history of the next."""
        before = np.array(self.history[1:]) if len(self.history) > 1 else None
        result = self.ik.solve(targets, self.q_last, before, rate_hz=self.rate_hz, speed_cap=self.speed_cap,
                               method=method)
        self._sent(result.q, result.q_solution[-1])
        return result

    def step(self, pose) -> np.ndarray:
        """The next joint command for one pose (4, 4): the per-tick solve from the previous solution, capped."""
        lo, hi = self.ik.bounds(self.q_last)
        solution = self.ik.solve_tick(pose, self.solution, lo, hi)
        q, _ = self.ik.cap(solution[None], self.q_last, self.ik.max_step(self.rate_hz, self.speed_cap))
        self._sent(q, solution)
        return q[0]


# ── tool paths ──
def straight_line(start: np.ndarray, goal: np.ndarray, ticks: int, ease: bool = True) -> np.ndarray:
    """(ticks, 4, 4) poses from just after ``start`` to ``goal``: the position on the line between them, the
    rotation turning about one fixed axis; ``ease`` times them minimum-jerk (still at both ends)."""
    if ticks < 1:
        raise ValueError("ticks must be >= 1")
    u = np.arange(1, ticks + 1) / ticks
    s = 10 * u**3 - 15 * u**4 + 6 * u**5 if ease else u
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    turn = so3_log(start[:3, :3].T @ goal[:3, :3])
    out = np.broadcast_to(np.eye(4), (ticks, 4, 4)).copy()
    out[:, :3, :3] = start[:3, :3] @ so3_exp(s[:, None] * turn)
    out[:, :3, 3] = start[:3, 3] + s[:, None] * (goal[:3, 3] - start[:3, 3])
    return out


# ── pose pre-processing ──
def _savgol_matrix(window: int, polyorder: int) -> np.ndarray:
    """(window, window): a degree-``polyorder`` least-squares fit over a window, evaluated at each of its ticks."""
    t = np.arange(window) - window // 2
    V = np.vander(t, polyorder + 1, increasing=True).astype(float)
    return V @ np.linalg.pinv(V)


def savgol(x: np.ndarray, window: int, polyorder: int) -> np.ndarray:
    """Savitzky-Golay along axis -2 of (..., ticks, d); the first and last ``window // 2`` ticks take the fit over
    the first / last window (scipy's ``mode="interp"``)."""
    x = np.asarray(x, dtype=float)
    P = _savgol_matrix(window, polyorder)
    h, n = window // 2, x.shape[-2]
    out = np.empty_like(x)
    for i in range(h, n - h):
        out[..., i, :] = np.einsum("k,...kd->...d", P[h], x[..., i - h:i + h + 1, :])
    out[..., :h, :] = np.einsum("ik,...kd->...id", P[:h], x[..., :window, :])
    out[..., n - h:, :] = np.einsum("ik,...kd->...id", P[window - h:], x[..., n - window:, :])
    return out


def smooth_poses(T: np.ndarray, window: int = 15, polyorder: int = 3) -> np.ndarray:
    """Savitzky-Golay over the ticks of (..., ticks, 4, 4) poses: positions as they are, rotations as rotation
    vectors relative to the first tick. The window shrinks to the chunk (odd); 0 or too short = unchanged."""
    T = np.asarray(T, dtype=float)
    n = T.shape[-3]
    w = min(window | 1, n if n % 2 else n - 1)
    if window <= 0 or w <= polyorder:
        return T
    R0 = T[..., :1, :3, :3]
    rv = so3_log(np.swapaxes(R0, -1, -2) @ T[..., :3, :3])
    out = T.copy()
    out[..., :3, :3] = R0 @ so3_exp(savgol(rv, w, polyorder))
    out[..., :3, 3] = savgol(T[..., :3, 3], w, polyorder)
    return out


def extend_poses(T: np.ndarray, rows: int, k: int = 4) -> np.ndarray:
    """(..., ticks, 4, 4) -> (..., ticks + rows, 4, 4): the added rows continue the last ``k`` rows' mean step, the
    position in a line and the rotation about a fixed axis of the tool's own frame (a chunk padded this way does not
    brake into a hold at its last pose in the trajectory solve)."""
    T = np.asarray(T, dtype=float)
    if rows <= 0:
        return T
    k = min(k, T.shape[-3] - 1)
    last, back = T[..., -1, :, :], T[..., -1 - k, :, :]
    dp = (last[..., :3, 3] - back[..., :3, 3]) / k
    w = so3_log(np.swapaxes(back[..., :3, :3], -1, -2) @ last[..., :3, :3]) / k
    steps = np.arange(1, rows + 1, dtype=float)
    ext = np.broadcast_to(last[..., None, :, :], last.shape[:-2] + (rows, 4, 4)).copy()
    ext[..., :3, 3] = last[..., None, :3, 3] + steps[:, None] * dp[..., None, :]
    ext[..., :3, :3] = last[..., None, :3, :3] @ so3_exp(steps[:, None] * w[..., None, :])
    return np.concatenate([T, ext], axis=-3)


def seam_blend(q: np.ndarray, q_from, ticks: int, lo, hi, max_step) -> np.ndarray:
    """Ease a tick-solved chunk q (T, dof) in from the last command ``q_from``: the plan shifted by its start offset,
    the shift fading to zero over the first ``ticks`` ticks, clipped to [lo, hi] and each tick kept within
    ``max_step`` of the one before (the trajectory solve needs none of this: continuity is in its costs)."""
    q, q_from = np.asarray(q, dtype=float), np.asarray(q_from, dtype=float)
    fade = np.clip(1.0 - np.arange(1, len(q) + 1) / (ticks + 1), 0.0, 1.0)
    eased = np.clip(q + fade[:, None] * (q_from - q[0]), lo, hi)
    out, prev = np.empty_like(eased), q_from
    for t in range(len(eased)):
        prev = np.clip(eased[t], prev - max_step, prev + max_step)
        out[t] = prev
    return out


def solve_path(ik: TrajectoryIK, poses: Sequence[np.ndarray], q_last, rate_hz: float,
               speed_cap: Optional[float] = None, chunk: int = 64) -> IKResult:
    """A long pose path (T, 4, 4) solved in chunks of ``chunk`` ticks, each continuing the one before."""
    poses = np.asarray(poses, dtype=float)
    stream = IKStream(ik, q_last, rate_hz, speed_cap)
    parts = [stream.feed(poses[i:i + chunk]) for i in range(0, len(poses), chunk)]
    return IKResult(*(np.concatenate([getattr(p, f.name) for p in parts]) for f in fields(IKResult)))
