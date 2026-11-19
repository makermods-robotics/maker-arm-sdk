"""Gravity feed-forward for the arm joints' MIT frames, from the profile's URDF.

Per joint the torque that holds the arm still at a pose (``Chain.gravity_torques``), times the
profile's overall ``gravity_scale`` and the joint's ``gravity_factor`` (a per-joint correction for
mass the URDF does not describe: cabling, gearboxes), eased in after torque-on and capped per
joint at ``gravity_limit_nm``. Angles and torques are joint coordinates (rad, N*m).
"""

from __future__ import annotations

import logging
import math
from typing import Optional, Sequence

import numpy as np

from .kinematics import Chain, chain_from_config

log = logging.getLogger(__name__)

DEFAULT_GRAVITY_SCALE = 1.0  # 1 = the URDF model, 0 = no feed-forward
MAX_GRAVITY_SCALE = 1.2

RAMP_S = 0.5         # ease-in from zero after torque-on
RAMP_STEP_S = 0.05   # the most one frame may advance it, so a pause never skips it
STALE_S = 0.15       # a measured pose needs a reply from every joint within this


class GravityFeedforward:
    """The chain joints' torques for one session of torque-on, with its ease-in and guards."""

    def __init__(self, chain: Chain, scale: float = DEFAULT_GRAVITY_SCALE,
                 limits_nm: Optional[Sequence[float]] = None, ramp_s: float = RAMP_S,
                 stale_s: float = STALE_S, joint_scale: Optional[Sequence[float]] = None):
        if not 0.0 <= scale <= MAX_GRAVITY_SCALE:
            raise ValueError(f"gravity_scale must be in [0, {MAX_GRAVITY_SCALE}], got {scale}")
        if limits_nm is None or len(limits_nm) != chain.n:
            raise ValueError(f"gravity limits need one value per chain joint ({chain.n}), got {limits_nm}")
        if not all(math.isfinite(v) and v >= 0 for v in limits_nm):
            raise ValueError(f"gravity limits must be finite and >= 0, got {list(limits_nm)}")
        factors = [1.0] * chain.n if joint_scale is None else list(joint_scale)
        if len(factors) != chain.n or not all(math.isfinite(f) and 0 <= f <= 2 for f in factors):
            raise ValueError(f"gravity factors need {chain.n} values in [0, 2], got {factors}")
        if not ramp_s > 0 or not stale_s > 0:
            raise ValueError("ramp_s and stale_s must be > 0")
        self.chain = chain
        self.scale = scale
        self.limits_nm = np.asarray(limits_nm, dtype=float)
        self.joint_scale = np.asarray(factors, dtype=float)
        self.ramp_s = ramp_s
        self.stale_s = stale_s
        self.level = 0.0
        self._last: Optional[float] = None

    @property
    def n(self) -> int:
        return self.chain.n

    def model(self, q: Sequence[float]) -> list[float]:
        """Holding torque at ``q`` with the scales, before the ease-in and caps; NaN for a non-finite pose."""
        q = np.asarray(q, dtype=float)
        if not np.all(np.isfinite(q)):
            return [math.nan] * self.n
        return (self.chain.gravity_torques(q) * self.joint_scale * self.scale).tolist()

    def reset(self) -> None:
        """Start the ease-in again (call at torque-on)."""
        self.level = 0.0
        self._last = None

    def torques(self, measured_q: Sequence[float], ages: Sequence[float], goal_q: Sequence[float],
                now: float) -> list[float]:
        """Torques for the frames about to go out, one step further into the ease-in: at the measured
        pose when every joint replied within ``stale_s``, else at the goal. Non-finite values become 0."""
        if self._last is not None:
            self.level = min(1.0, self.level + min(RAMP_STEP_S, max(0.0, now - self._last)) / self.ramp_s)
        self._last = now
        fresh = all(math.isfinite(q) and age <= self.stale_s for q, age in zip(measured_q, ages))
        tau = self.model(list(measured_q if fresh else goal_q)[:self.n])
        return [max(-limit, min(limit, t * self.level)) if math.isfinite(t) else 0.0
                for t, limit in zip(tau, self.limits_nm)]


def from_config(config, chain: Optional[Chain] = None) -> Optional[GravityFeedforward]:
    """The profile's feed-forward, or None when it names no URDF or its ``gravity_scale`` is 0.

    Reads ``gravity_scale`` and, for each of the chain's joints, ``gravity_limit_nm`` (required)
    and ``gravity_factor`` (default 1).
    """
    scale = getattr(config, "gravity_scale", DEFAULT_GRAVITY_SCALE)
    if not scale:
        return None
    chain = chain if chain is not None else chain_from_config(config)
    if chain is None:
        return None
    joints = list(config.joints)[:chain.n]
    limits = [getattr(j, "gravity_limit_nm", None) for j in joints]
    missing = [getattr(j, "name", "") or str(i) for i, (j, v) in enumerate(zip(joints, limits)) if v is None]
    if missing:
        raise ValueError(f"gravity feed-forward needs gravity_limit_nm on joints {missing}")
    return GravityFeedforward(chain, scale, limits,
                              joint_scale=[getattr(j, "gravity_factor", 1.0) for j in joints])
