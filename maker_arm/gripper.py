"""Gripper holding-effort regulation, ported from MakerModsLab (metal_gripper_hold.py
HoldingController and maker_gripper_hold.py) with the Maker settings: closing direction +1,
closing-effort cap on, 0.5 Nm default holding effort.

Free closing aims at the requested goal with Kp scaled so Kp x error never exceeds the closing
cap (max(1, 2 x hold) Nm) and Kd lowered to CAPPED_CLOSING_KD. Once contact is detected (closing
effort > 0.5 Nm at |v| < 0.3 rad/s for 100 ms) the command is regulated so the measured effort
settles at the holding target. Opening, or a sustained loss of contact, ends holding.
"""

import math
from collections import deque

from . import protocol

CAPPED_CLOSING_KD = 0.02
_KP_STEP = (protocol.KP_MAX - protocol.KP_MIN) / 4095


class GripperHold:
    """Radians throughout. ``update`` returns the position to send; read ``kp_scale`` and
    ``closing_capped`` afterwards for the gains."""

    def __init__(self, torque_nm: float, limits: tuple[float, float], closing_direction: int = 1):
        if not (math.isfinite(torque_nm) and 0.1 <= torque_nm <= 2.0):
            raise ValueError("gripper hold torque must be in [0.1, 2.0] Nm")
        if closing_direction not in (-1, 1):
            raise ValueError("closing direction must be -1 or +1")
        self.torque_nm = torque_nm
        self.limits = limits
        self.closing_direction = closing_direction
        self.reset()

    def reset(self) -> None:
        self.holding = False
        self.closing_capped = False
        self.kp_scale = 1.0
        self.last_command = None
        self.filtered = None
        self.history: deque = deque()
        self.last_time = None
        self.contact_started = None
        self.low_effort_since = None

    @property
    def closing_cap_nm(self) -> float:
        return max(1.0, 2.0 * self.torque_nm)

    def update(self, goal: float, position: float, velocity: float, effort: float,
               kp: float, now: float) -> float:
        if not all(math.isfinite(x) for x in (goal, position, velocity, effort, kp, now)) or kp <= 0:
            raise ValueError("invalid gripper holding feedback/gain")
        s = self.closing_direction
        lo, hi = self.limits
        goal = min(hi, max(lo, goal))
        dt = 0.0 if self.last_time is None else max(0.0, now - self.last_time)
        self.last_time = now
        self.history.append((now, effort))
        while self.history and self.history[0][0] < now - 0.1:
            self.history.popleft()
        closing_effort = s * sum(e for _, e in self.history) / len(self.history)
        opening = s * (goal - position) < -math.radians(0.25)
        if self.holding:
            if closing_effort < min(0.2, self.torque_nm * 0.4):
                if self.low_effort_since is None:
                    self.low_effort_since = now
            else:
                self.low_effort_since = None
            lost_contact = self.low_effort_since is not None and now - self.low_effort_since >= 0.3
            if opening or lost_contact:
                self.holding = False
                self.contact_started = None
                self.low_effort_since = None
        elif s * (goal - position) > 0 and closing_effort > 0.5 and abs(velocity) < 0.3:
            if self.contact_started is None:
                self.contact_started = now
            if now - self.contact_started >= 0.1:
                self.holding = True
        else:
            self.contact_started = None

        if self.holding:
            raw = self.last_command + s * (self.torque_nm - abs(effort)) / kp
            alpha = 1.0 - math.exp(-min(dt, 0.05) / 0.19)
            self.filtered += alpha * (raw - self.filtered)
            bounded = max(goal, self.filtered) if s < 0 else min(goal, self.filtered)
            command = min(hi, max(lo, bounded))
            self.filtered = command
        else:
            command = goal
            self.filtered = position
        self.closing_capped = False
        self.kp_scale = 1.0
        sent = command
        lead = self.closing_cap_nm / kp
        error = s * (command - position)
        if error > lead:
            equivalent = min(hi, max(lo, position + s * lead))
            if self.holding:
                command = sent = equivalent
                self.filtered = command
            else:
                self.closing_capped = True
                self.kp_scale = lead / error
                command = equivalent
        self.last_command = command
        return sent

    def gains(self, kp: float, kd: float) -> tuple[float, float]:
        """Gains for the last update: a capped Kp is rounded DOWN to the 12-bit grid so
        quantization can never lift Kp x error above the cap."""
        if self.kp_scale >= 1.0:
            return kp, kd
        scaled = kp * self.kp_scale
        return (math.floor(scaled / _KP_STEP) + 1e-6) * _KP_STEP, (CAPPED_CLOSING_KD if self.closing_capped else kd)
