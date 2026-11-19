"""Teleoperation: a leader drives a follower arm. The reference loop, written to be read.

    leader.connect()
    follower.connect(); follower.enable()
    reason = Teleop(leader, follower).run(stop_event)   # returns why it stopped; the arm is holding
    follower.stop()                                     # return to rest, then release
    leader.close()

Each cycle reads the leader (follower joint targets, rad), shapes them (a slow approach at the
start when the poses differ, then an optional speed/acceleration cap), and hands them to
``follower.set_joint_targets``. The follower's own control loop does the rest (rate limit, soft
limits, gravity, faults). The loop stops, and the follower holds its measured pose, when:

- the stop event is set (``"stopped"``),
- the leader raises ``LeaderError`` or misses ``miss_limit`` reads in a row (``"leader_lost: ..."``),
- the follower leaves ENABLED, e.g. on a fault (``"follower_fault: ..."``),
- the stall hook reports a stall (``"stall: motor N"``).

What happens next (return to rest, release, retry) is the caller's decision. Bimanual: one
``Teleop`` per pair, run together with ``run_many``.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from .leaders.base import LeaderError

log = logging.getLogger(__name__)

MAX_STEP_S = 0.05  # the most time one cycle may claim for the speed cap, so a stall never banks motion


@dataclass(frozen=True)
class TeleopStep:
    """What one cycle did, for ``on_step`` (logging, recording, a UI)."""
    elapsed_s: float
    leader: Optional[list[float]]   # None when this cycle's read missed
    command: list[float]            # what went to set_joint_targets
    syncing: bool


class SpeedCap:
    """Per-joint speed and acceleration limit on a moving target (rad, rad/s, rad/s^2).

    The command speeds up at most ``max_acceleration`` and starts braking early enough to stop on
    the goal instead of overshooting it. A joint
    with both limits infinite follows its goal exactly.
    """

    def __init__(self, start: Sequence[float]):
        self.position = list(start)
        self.velocity = [0.0] * len(self.position)

    def step(self, goal: Sequence[float], dt: float, max_velocity: Sequence[float],
             max_acceleration: Sequence[float]) -> list[float]:
        for i, (target, vmax, accel) in enumerate(zip(goal, max_velocity, max_acceleration)):
            error = target - self.position[i]
            if math.isinf(vmax) and math.isinf(accel):
                self.position[i], self.velocity[i] = target, 0.0
                continue
            if dt <= 0:
                continue
            change = accel * dt
            # Fastest speed that can still brake to a stop on the goal, counted in whole ticks.
            braking = vmax if math.isinf(accel) else \
                0.5 * (-change + math.sqrt(change * change + 8.0 * abs(error) * change / dt))
            wanted = math.copysign(min(vmax, braking), error)
            v = wanted if math.isinf(accel) else \
                self.velocity[i] + max(-change, min(change, wanted - self.velocity[i]))
            v = max(-vmax, min(vmax, v))
            move = v * dt
            if (error >= 0 and move > error) or (error <= 0 and move < error):
                move = error
                if math.isinf(accel):
                    v = 0.0
            self.position[i] += move
            self.velocity[i] = v
        return list(self.position)


def _state(arm) -> str:
    state = getattr(arm, "state", None)
    return getattr(state, "name", str(state)).upper()


class Teleop:
    def __init__(self, leader, follower, rate_hz: float = 100.0, max_velocity: Optional[float] = None,
                 max_acceleration: Optional[float] = None, sync_threshold: float = 0.8,
                 sync_velocity: float = 0.3, miss_limit: int = 25, startup_timeout: float = 2.0,
                 stall=None, on_step: Optional[Callable[[TeleopStep], None]] = None,
                 exempt: Sequence[str] = ("gripper",),
                 clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep):
        """
        rate_hz          leader reads per second.
        max_velocity     rad/s cap on every joint's command (None: none beyond the follower's own).
        max_acceleration rad/s^2 cap (None: none).
        sync_threshold   rad: a larger startup gap between leader and follower is closed slowly ...
        sync_velocity    ... at this speed (rad/s), before normal following starts.
        miss_limit       consecutive leader misses that stop following.
        stall            optional hook: ``stall.record(elapsed_s, targets) -> list of events with
                         .motor_id`` (the stall module's MotorStateRecorder).
        exempt           joint names the speed cap leaves alone after the sync (the gripper).
        """
        if not rate_hz > 0 or not sync_velocity > 0 or not sync_threshold >= 0 or miss_limit < 1:
            raise ValueError("rate_hz and sync_velocity must be > 0, sync_threshold >= 0, miss_limit >= 1")
        for name, value in (("max_velocity", max_velocity), ("max_acceleration", max_acceleration)):
            if value is not None and not value > 0:
                raise ValueError(f"{name} must be > 0")
        self.leader = leader
        self.follower = follower
        self.period = 1.0 / rate_hz
        self.sync_threshold = sync_threshold
        self.sync_velocity = sync_velocity
        self.miss_limit = miss_limit
        self.startup_timeout = startup_timeout
        self.stall = stall
        self.on_step = on_step
        self.clock, self.sleep = clock, sleep
        names = [j.name for j in follower.config.joints]
        cap = math.inf if max_velocity is None else max_velocity
        accel = math.inf if max_acceleration is None else max_acceleration
        self._follow_velocity = [math.inf if n in exempt else cap for n in names]
        self._follow_acceleration = [math.inf if n in exempt else accel for n in names]
        self._sync_acceleration = [accel] * len(names)
        self.startup_gap: Optional[float] = None
        self.stop_reason: Optional[str] = None

    # ── the loop ──
    def run(self, stop_event: Optional[threading.Event] = None) -> str:
        """Follow until a stop condition; returns its reason with the follower holding its pose."""
        stop_event = stop_event or threading.Event()
        try:
            self.stop_reason = self._follow(stop_event)
        except Exception as e:
            log.exception("teleop loop failed")
            self.stop_reason = f"error: {e}"
        finally:
            if _state(self.follower) in ("ENABLED", "FAULT"):
                self.follower.hold_current_position()
        log.info("teleop stopped: %s", self.stop_reason)
        return self.stop_reason

    def _first_reading(self, stop_event) -> Optional[list[float]]:
        deadline = self.clock() + self.startup_timeout
        while not stop_event.is_set() and self.clock() < deadline:
            targets = self.leader.read()
            if targets is not None:
                return targets
            self.sleep(self.period)
        return None

    def _follow(self, stop_event: threading.Event) -> str:
        if _state(self.follower) != "ENABLED":
            return f"follower_not_enabled: {_state(self.follower)}"
        try:
            goal = self._first_reading(stop_event)
        except LeaderError as e:
            return f"leader_lost: {e}"
        if goal is None:
            return "stopped" if stop_event.is_set() else "leader_lost: no reading at startup"

        # Startup alignment: a large gap is closed at sync_velocity, never at follow speed.
        present = self.follower.get_joint_positions()
        self.startup_gap = max(abs(g - p) for g, p in zip(goal, present))
        syncing = self.startup_gap > self.sync_threshold
        if syncing:
            log.warning("leader and follower differ by %.2f rad: syncing slowly at %.2f rad/s",
                        self.startup_gap, self.sync_velocity)
        shaper = SpeedCap(present)
        sync_velocity = [self.sync_velocity] * len(goal)

        misses = 0
        start = self.clock()
        last = start - self.period  # the first cycle moves one period's worth
        while not stop_event.is_set():
            now = self.clock()
            dt = min(max(now - last, 0.0), MAX_STEP_S)
            last = now

            # 1. read the leader; a miss keeps the last goal
            try:
                reading = self.leader.read()
            except LeaderError as e:
                return f"leader_lost: {e}"
            except Exception as e:
                log.warning("leader read failed: %s", e)
                reading = None
            if reading is None:
                misses += 1
                if misses >= self.miss_limit:
                    return f"leader_lost: {misses} reads missed in a row"
            else:
                misses, goal = 0, reading

            # 2. shape it and send it
            if syncing:
                command = shaper.step(goal, dt, sync_velocity, self._sync_acceleration)
            else:
                command = shaper.step(goal, dt, self._follow_velocity, self._follow_acceleration)
            if syncing and all(abs(c - g) < 1e-9 for c, g in zip(command, goal)):
                syncing = False
                log.info("leader and follower aligned: following")
            if not self.follower.set_joint_targets(command) and _state(self.follower) != "ENABLED":
                return f"follower_{_state(self.follower).lower()}: {getattr(self.follower, 'fault_reason', None)}"

            # 3. hooks
            elapsed = now - start
            if self.stall is not None:
                events = self.stall.record(elapsed, list(goal))
                if events:
                    return f"stall: motor {events[0].motor_id}"
            if self.on_step is not None:
                try:
                    self.on_step(TeleopStep(elapsed, reading, command, syncing))
                except Exception:
                    log.exception("on_step failed")

            remain = self.period - (self.clock() - now)
            if remain > 0:
                self.sleep(remain)
        return "stopped"


def run_many(teleops: Sequence[Teleop], stop_event: Optional[threading.Event] = None) -> list[str]:
    """Run several pairs (bimanual) on their own threads. When one stops, for any reason, all stop
    and hold: one arm carrying on alone is never what the operator meant. Returns each reason."""
    shared = threading.Event()
    reasons: list[Optional[str]] = [None] * len(teleops)

    def worker(i: int) -> None:
        try:
            reasons[i] = teleops[i].run(shared)
        finally:
            shared.set()

    threads = [threading.Thread(target=worker, args=(i,), daemon=True, name=f"teleop-{i}")
               for i in range(len(teleops))]
    for t in threads:
        t.start()
    while any(t.is_alive() for t in threads):
        if stop_event is not None and stop_event.is_set():
            shared.set()
        for t in threads:
            t.join(timeout=0.02)
    return [r or "stopped" for r in reasons]
