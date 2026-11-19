"""A second arm of the same kind as a back-drivable leader.

The leader arm is enabled and put in ``float_mode()``: kp 0, a little damping, and gravity
feed-forward, so it holds its own weight while a hand moves it. Optional friction compensation
(per joint, from the profile) is added through ``float_mode(extra_torque=...)``: a viscous term on
the measured velocity (clamped), a Coulomb term in the direction of travel, and a constant
breakaway torque at rest. ``read()`` returns its joint positions 1:1, clipped to the follower's
limits.

Profile fields (per joint, all optional, 0 = off): ``friction_viscous`` (N*m*s/rad),
``friction_coulomb`` (N*m), ``friction_rest_nm`` (N*m), ``friction_velocity_clamp`` (rad/s); arm
level ``friction_deadzone`` (rad/s, below it a joint counts as still).
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional, Sequence

from .base import LeaderError

DEFAULT_DEADZONE = 0.05  # rad/s: motor velocity noise must not flip the Coulomb term at rest


@dataclass(frozen=True)
class Friction:
    """Per-joint friction feed-forward: torque(velocity) in joint coordinates."""
    viscous: tuple[float, ...]
    coulomb: tuple[float, ...]
    rest: tuple[float, ...]
    clamp: tuple[float, ...]          # inf = no clamp
    deadzone: float = DEFAULT_DEADZONE

    @classmethod
    def from_config(cls, config) -> Optional["Friction"]:
        joints = list(config.joints)

        def field(name, default=0.0):
            values = tuple(float(getattr(j, name, None) or default) for j in joints)
            if not all(math.isfinite(v) or (name == "friction_velocity_clamp" and v == math.inf) for v in values):
                raise ValueError(f"{name} must be finite")
            return values

        friction = cls(field("friction_viscous"), field("friction_coulomb"), field("friction_rest_nm"),
                       field("friction_velocity_clamp", math.inf),
                       float(getattr(config, "friction_deadzone", None) or DEFAULT_DEADZONE))
        return friction if any(friction.viscous + friction.coulomb + friction.rest) else None

    def torques(self, positions: Sequence[float], velocities: Sequence[float]) -> list[float]:
        out = []
        for i, v in enumerate(velocities):
            if not math.isfinite(v):
                out.append(0.0)
            elif abs(v) < self.deadzone:
                out.append(self.rest[i])
            else:
                limit = self.clamp[i]
                out.append(self.viscous[i] * max(-limit, min(limit, v)) + math.copysign(self.coulomb[i], v))
        return out


def _state(arm) -> str:
    state = getattr(arm, "state", None)
    return getattr(state, "name", str(state)).upper()


class ArmLeader:
    name = "arm"

    def __init__(self, arm, follower=None, friction: Optional[Friction] = None):
        """``follower`` (an Arm) supplies the limits ``read()`` clips to; default: the leader's own."""
        self.arm = arm
        limits_from = follower if follower is not None else arm
        self.lower = [j.lo for j in limits_from.config.joints]
        self.upper = [j.hi for j in limits_from.config.joints]
        if len(self.lower) != len(arm.config.joints):
            raise ValueError(f"leader has {len(arm.config.joints)} joints, follower {len(self.lower)}")
        self.friction = friction if friction is not None else Friction.from_config(arm.config)

    def connect(self, cancel: Optional[threading.Event] = None) -> None:
        """Enable (``cancel`` aborts the enable, as in Arm.enable) and float the leader arm."""
        if _state(self.arm) == "DISCONNECTED":
            self.arm.connect()
        if _state(self.arm) != "ENABLED":
            self.arm.enable(cancel=cancel)
        if self.friction is not None:
            self.arm.float_mode(extra_torque=self.friction.torques)
        else:
            self.arm.float_mode()

    def read(self) -> Optional[list[float]]:
        state = _state(self.arm)
        if state == "FAULT":
            raise LeaderError(f"leader arm faulted: {getattr(self.arm, 'fault_reason', None)}")
        if state != "ENABLED":
            return None
        q = self.arm.get_joint_positions()
        if len(q) != len(self.lower) or not all(math.isfinite(v) for v in q):
            return None
        return [max(lo, min(hi, v)) for v, lo, hi in zip(q, self.lower, self.upper)]

    def close(self) -> None:
        """Stop floating: the profile gains back, holding where the hand left it. Torque stays on;
        the caller returns the leader to rest or releases it."""
        if _state(self.arm) not in ("ENABLED", "FAULT"):
            return
        # Target first, gains second: stiffening toward a stale target would yank the arm.
        self.arm.hold_current_position()
        self.arm.set_gains()
