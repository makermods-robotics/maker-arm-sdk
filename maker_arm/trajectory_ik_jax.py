"""Trajectory IK on JAX: Dream Labs' ``dream_ik`` solver itself (the ``dreamlabs`` package, this SDK's ``jax`` extra)
behind ``trajectory_ik.TrajectoryIK``'s interface, so ``IKStream`` and ``solve_path`` take either.

Same costs, limits and speed caps as the numpy solver, compiled with XLA: the first solve of each chunk length
compiles (``compile_s`` keeps the seconds per length), later ones run in a fraction of the numpy solve's time. It
solves on the embodiment's own URDF and tool point (``_product.DREAM_EMBODIMENT``, tool ``"tcp"``), with the
profile's soft limits, rest pose, weights and speeds in place of the embodiment's; construction checks that its tool
pose matches the profile's chain within ``FK_TOLERANCE``. Joints are this package's joint coordinates, mapped to the
embodiment's by ``_product.DREAM_JOINT_SIGN`` / ``DREAM_JOINT_OFFSET``. ``pos_err`` / ``rot_err`` are measured on the
profile's chain, as the numpy solver's are.

Importing this module does not import jax; constructing the solver does.
"""

from __future__ import annotations

import importlib.util
import math
import sys
import time
from typing import Optional, Sequence

import numpy as np

from . import _product
from .kinematics import Chain
from .trajectory_ik import IKResult, IKWeights, TrajectoryIK, tool_errors

INSTALL_HINT = f"pip install '{_product.DIST_NAME}[jax]'"
# Largest tool pose difference (m, rad) allowed between the embodiment's model and the profile's chain.
FK_TOLERANCE = (0.01, math.radians(2.0))
TOOL = "tcp"


def _present(name: str) -> bool:
    if name in sys.modules:
        return sys.modules[name] is not None
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def available() -> bool:
    """True when the ``jax`` extra is installed (checked without importing it)."""
    return _present("dream_ik") and _present("jax")


def _import_solver():
    try:
        from dream_ik import solver, specs   # before jax: dream_ik keeps JAX on the CPU
        import jax.numpy as jnp
        import jax_dataclasses as jdc
    except ImportError as e:
        raise ImportError(f"the JAX trajectory IK needs the optional 'jax' extra ({INSTALL_HINT}): {e}") from e
    try:
        from loguru import logger
        logger.disable("jaxls")   # an INFO line per compiled problem
    except ImportError:
        pass
    return solver, specs, jnp, jdc


class JaxTrajectoryIK(TrajectoryIK):
    """``TrajectoryIK`` solved by ``dream_ik.ChunkIK``. ``sides`` are the embodiment's arms solved together
    (``solve`` takes one side, ``solve_arms`` any number); ``embodiment``, ``joint_sign`` and ``joint_offset`` default
    to this package's ``_product`` values. Constructing it imports jax (ImportError naming the extra without it)."""

    def __init__(self, chain: Chain, limits=None, max_velocity=math.inf, rest=None,
                 weights: IKWeights = IKWeights(), rest_weight=None, *, embodiment: Optional[str] = None,
                 sides: Sequence[str] = ("right",), joint_sign=None, joint_offset=None):
        super().__init__(chain, limits, max_velocity, rest, weights, rest_weight)
        solver, specs, jnp, jdc = _import_solver()
        self._solver, self._jnp = solver, jnp
        name = _product.DREAM_EMBODIMENT if embodiment is None else embodiment
        if name not in specs.EMBODIMENTS:
            raise ValueError(f"dream_ik has no embodiment {name!r}")
        emb = specs.EMBODIMENTS[name]
        self.sides = tuple(sides)
        if not self.sides or any(s not in emb.sides for s in self.sides):
            raise ValueError(f"sides must be among {emb.sides}, got {self.sides}")
        if self.sides != emb.sides:   # an embodiment of just these sides: no solve for the others
            name = f"{name}:{'+'.join(self.sides)}"
            if name not in specs.EMBODIMENTS:
                arm = {s: emb.side_arm(s) for s in self.sides} if isinstance(emb.arm, dict) else emb.arm
                specs.EMBODIMENTS[name] = specs.Embodiment(
                    arm=arm, sides=self.sides, obstacles={s: emb.obstacles[s] for s in self.sides})
        for s in self.sides:
            if len(emb.side_arm(s).joints) != self.n:
                raise ValueError(f"the embodiment's arm has {len(emb.side_arm(s).joints)} joints, the chain {self.n}")
        self.embodiment = name
        sign = _product.DREAM_JOINT_SIGN if joint_sign is None else joint_sign
        offset = _product.DREAM_JOINT_OFFSET if joint_offset is None else joint_offset
        self.joint_sign = np.asarray(sign, dtype=float).reshape(self.n)
        self.joint_offset = np.asarray(offset, dtype=float).reshape(self.n)
        if not np.all(np.abs(self.joint_sign) == 1.0):
            raise ValueError(f"joint_sign must be +1 or -1 per joint, got {self.joint_sign.tolist()}")
        self._ik = solver.ChunkIK(name, TOOL, 30.0)   # rate unused: the speed cap is applied here
        self._configure(jnp, jdc)
        self.serve = specs.ServeSpec(smooth_window=0, seam_blend_ticks=0, solve="traj",
                                     traj_vel_weight=weights.vel, traj_acc_weight=weights.acc,
                                     traj_jerk_weight=weights.jerk, traj_history_weight=weights.history,
                                     interarm_weight=0.0)
        self.compile_s: dict = {}
        self._check_model()

    # ── joint coordinates ──
    def to_solver(self, q) -> np.ndarray:
        return np.asarray(q, dtype=float) * self.joint_sign + self.joint_offset

    def from_solver(self, q) -> np.ndarray:
        return (np.asarray(q, dtype=float) - self.joint_offset) * self.joint_sign

    def _solver_bounds(self, lo, hi) -> tuple[np.ndarray, np.ndarray]:
        a, b = self.to_solver(lo), self.to_solver(hi)
        return np.minimum(a, b), np.maximum(a, b)

    def _configure(self, jnp, jdc) -> None:
        """The profile's limits, rest pose and weights in place of the embodiment's, in every group."""
        w = self.weights
        lo, hi = self._solver_bounds(self.lo, self.hi)
        rest = self.to_solver(self.rest)
        for g in self._ik._groups:
            arm, order = g.arm, list(g.arm.to_pyroki)
            g.arm = jdc.replace(arm, lo=jnp.asarray(lo[order], arm.lo.dtype), hi=jnp.asarray(hi[order], arm.hi.dtype),
                                rest=jnp.asarray(rest[order], arm.rest.dtype),
                                rest_weight=jnp.asarray(self.rest_weight[order], arm.rest_weight.dtype))
            g.rest = rest.copy()
            g.w = {**g.w, **{k: jnp.asarray(getattr(w, k), g.w[k].dtype) for k in ("pos", "ori", "limit", "last_pose")}}
        ik = self._ik
        ik.side_rest = np.stack([g.rest for g in ik._side_groups])
        ik.side_lo = np.stack([g.to_spec(g.arm.lo) for g in ik._side_groups])
        ik.side_hi = np.stack([g.to_spec(g.arm.hi) for g in ik._side_groups])

    def _check_model(self) -> None:
        """The embodiment's tool pose against the chain's over the joint range."""
        rng = np.random.default_rng(0)
        q = np.vstack([self.rest, rng.uniform(self.lo, self.hi, (15, self.n))])
        for side in self.sides:
            pos, rot = tool_errors(self.chain, q, self._ik.fk(self.to_solver(q), side))
            if pos.max() > FK_TOLERANCE[0] or rot.max() > FK_TOLERANCE[1]:
                raise ValueError(f"the embodiment {self.embodiment!r} puts the tool up to {pos.max() * 1e3:.1f} mm, "
                                 f"{math.degrees(rot.max()):.2f} deg from the profile's chain: not the same arm, "
                                 "frame or joint convention")

    # ── compile ──
    def warmup(self, n_ticks: int) -> float:
        """Compile the chunk solves for ``n_ticks`` rows (done by the first solve of that length otherwise);
        returns the seconds it took."""
        if n_ticks not in self.compile_s:
            t0 = time.perf_counter()
            self._ik.warmup(int(n_ticks), self.serve)
            self.compile_s[n_ticks] = time.perf_counter() - t0
        return self.compile_s[n_ticks]

    # ── the solves ──
    def solve(self, targets, q_last, q_before=None, *, rate_hz: float, speed_cap: Optional[float] = None,
              method: str = "traj") -> IKResult:
        """As ``TrajectoryIK.solve``, for a solver of one side."""
        if len(self.sides) != 1:
            raise ValueError(f"solve takes one arm; this solver has {self.sides}: use solve_arms")
        targets = np.asarray(targets, dtype=float)
        return self.solve_arms(targets[None], np.asarray(q_last, dtype=float)[None],
                               None if q_before is None else np.asarray(q_before, dtype=float)[None],
                               rate_hz=rate_hz, speed_cap=speed_cap, method=method)[0]

    def solve_arms(self, targets, q_last, q_before=None, *, rate_hz: float, speed_cap: Optional[float] = None,
                   method: str = "traj") -> list[IKResult]:
        """Every side in one call: targets (arms, T, 4, 4), q_last (arms, dof), q_before None or (arms, dof) or
        (arms, rows, dof) newest first; one IKResult per side, in ``sides`` order."""
        targets = np.asarray(targets, dtype=float)
        arms = len(self.sides)
        if targets.ndim != 4 or targets.shape[0] != arms:
            raise ValueError(f"targets must be ({arms}, T, 4, 4), got {targets.shape}")
        q_last = np.asarray(q_last, dtype=float).reshape(arms, -1)
        for k in range(arms):
            self._check_inputs(targets[k], q_last[k])
        if method not in ("traj", "tick"):
            raise ValueError(f"method must be 'traj' or 'tick', got {method!r}")
        max_step = self.max_step(rate_hz, speed_cap)
        self.warmup(targets.shape[1])
        q0 = self.to_solver(q_last)
        if method == "traj":
            before = q_last[:, None] if q_before is None else np.asarray(q_before, dtype=float).reshape(arms, -1, self.n)
            sol = self._ik.solve_traj(targets, q0, self.to_solver(before), math.inf, self.serve).q_solution
        else:
            sol = self._ik.solve(targets, q0, math.inf).q_solution
        out = []
        for k in range(arms):
            lo, hi = self.bounds(q_last[k])
            q_sol = np.clip(self.from_solver(np.asarray(sol[k], dtype=float)), lo, hi)
            q, capped = self.cap(q_sol, q_last[k], max_step)
            pos, rot = tool_errors(self.chain, q, targets[k])
            out.append(IKResult(q=q, q_solution=q_sol, pos_err=pos, rot_err=rot, capped=capped))
        return out

    def solve_tick(self, target: np.ndarray, q_prev, lo=None, hi=None) -> np.ndarray:
        """As ``TrajectoryIK.solve_tick`` (one pose, the first side's arm)."""
        jnp, g = self._jnp, self._ik._groups[0]
        q_prev = np.asarray(q_prev, dtype=float)
        if lo is None or hi is None:
            lo, hi = self.bounds(q_prev)
        lo_s, hi_s = self._solver_bounds(lo, hi)
        order = list(g.arm.to_pyroki)
        qp = g.to_pyroki(self.to_solver(q_prev)[None])
        tgt = jnp.asarray(self._solver.poses_to_wxyz_xyz(np.asarray(target, dtype=float)[None]))
        t0 = time.perf_counter()
        _, q_sol = self._solver._solve_tick(g.arm, g.obstacles, g.w, tgt, qp, qp, jnp.asarray(lo_s[order][None]),
                                            jnp.asarray(hi_s[order][None]), jnp.full(self.n, jnp.inf))
        q = np.clip(self.from_solver(np.asarray(g.to_spec(q_sol)[0], dtype=float)), lo, hi)
        self.compile_s.setdefault("tick", time.perf_counter() - t0)
        return q
