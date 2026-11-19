import math
import threading
from types import SimpleNamespace

import pytest

from chain_fixture import load
from teleop_fakes import FakeArm, FakeClock, ScriptedLeader, State

teleop = load("teleop")
LeaderError = load("leaders").LeaderError
Teleop, SpeedCap, run_many = teleop.Teleop, teleop.SpeedCap, teleop.run_many

ZERO = [0.0] * 7


def pose(*head):
    return list(head) + [0.0] * (7 - len(head))


class StopAfter:
    """on_step hook that sets the stop event after n cycles and keeps every step."""

    def __init__(self, event, n):
        self.event, self.n, self.steps = event, n, []

    def __call__(self, step):
        self.steps.append(step)
        if len(self.steps) >= self.n:
            self.event.set()


def run(leader, follower, cycles=20, **kwargs):
    clock = FakeClock()
    stop = threading.Event()
    hook = StopAfter(stop, cycles)
    loop = Teleop(leader, follower, on_step=hook, clock=clock, sleep=clock.sleep, **kwargs)
    return loop.run(stop), hook.steps, loop


# ── following ──
def test_targets_pass_straight_through_without_a_cap():
    follower = FakeArm(state=State.ENABLED)
    reason, steps, _ = run(ScriptedLeader([pose(0.1), pose(0.2), pose(0.3)]), follower, cycles=3)
    assert reason == "stopped"
    assert [t[0] for t in follower.targets] == pytest.approx([0.2, 0.3, 0.3])  # first read was the startup read
    assert follower.calls[-1] == "hold"


def test_follower_must_be_enabled():
    reason, _, _ = run(ScriptedLeader([ZERO]), FakeArm(state=State.CONNECTED))
    assert reason.startswith("follower_not_enabled")


def test_max_velocity_caps_each_joint_but_not_the_gripper():
    follower = FakeArm(state=State.ENABLED)
    goal = pose(1.0, -1.0)[:6] + [1.0]
    _, steps, _ = run(ScriptedLeader([goal]), follower, cycles=10, rate_hz=100, max_velocity=2.0,
                      sync_threshold=2.0)
    first = follower.targets[0]
    assert first[0] == pytest.approx(0.02) and first[1] == pytest.approx(-0.02)   # 2 rad/s * 10 ms
    assert first[6] == 1.0                                                     # exempt
    assert follower.targets[-1][0] == pytest.approx(0.2)


def test_acceleration_cap_ramps_the_speed_up():
    cap = SpeedCap([0.0])
    moves = [cap.step([10.0], 0.01, [5.0], [100.0])[0] for _ in range(5)]
    speeds = [b - a for a, b in zip([0.0] + moves, moves)]
    assert speeds == pytest.approx([0.01, 0.02, 0.03, 0.04, 0.05])


def test_acceleration_cap_brakes_onto_the_goal_without_overshoot():
    cap = SpeedCap([0.0])
    path = [cap.step([0.3], 0.01, [5.0], [20.0])[0] for _ in range(200)]
    assert max(path) <= 0.3 + 1e-12 and path[-1] == pytest.approx(0.3)


def test_a_paused_cycle_cannot_bank_motion():
    follower = FakeArm(state=State.ENABLED)
    clock = FakeClock()
    stop = threading.Event()

    def hook(step):
        clock.now += 10.0  # the machine froze for 10 s after this cycle
        stop.set() if len(follower.targets) >= 2 else None

    Teleop(ScriptedLeader([pose(5.0)]), follower, max_velocity=1.0, sync_threshold=10,
           on_step=hook, clock=clock, sleep=clock.sleep).run(stop)
    assert follower.targets[1][0] - follower.targets[0][0] == pytest.approx(teleop.MAX_STEP_S * 1.0)


# ── startup alignment ──
def test_a_large_startup_gap_is_closed_slowly_then_following_resumes():
    follower = FakeArm(state=State.ENABLED)
    reason, steps, loop = run(ScriptedLeader([pose(1.0)]), follower, cycles=400, rate_hz=100,
                              sync_threshold=0.5, sync_velocity=0.5)
    assert loop.startup_gap == pytest.approx(1.0)
    assert follower.targets[0][0] == pytest.approx(0.005)
    synced = next(i for i, s in enumerate(steps) if not s.syncing)
    assert 195 <= synced <= 205   # 1 rad at 0.5 rad/s, 100 Hz
    assert all(abs(b[0] - a[0]) <= 0.005 + 1e-9 for a, b in zip(follower.targets, follower.targets[1:]))


def test_a_small_startup_gap_follows_at_once():
    follower = FakeArm(state=State.ENABLED, positions=pose(0.1))
    _, steps, loop = run(ScriptedLeader([pose(0.3)]), follower, cycles=2, sync_threshold=0.5)
    assert not steps[0].syncing and follower.targets[0][0] == pytest.approx(0.3)


def test_no_leader_reading_at_startup_never_moves_the_follower():
    follower = FakeArm(state=State.ENABLED)
    reason, _, _ = run(ScriptedLeader([None]), follower, startup_timeout=0.5)
    assert reason.startswith("leader_lost") and follower.targets == []


# ── failsafes ──
def test_consecutive_misses_stop_following_and_hold():
    follower = FakeArm(state=State.ENABLED)
    script = [pose(0.1), pose(0.1)] + [None] * 10
    reason, _, _ = run(ScriptedLeader(script), follower, cycles=100, miss_limit=5)
    assert reason == "leader_lost: 5 reads missed in a row"
    assert len(follower.targets) == 1 + 4  # the good cycle, then the last goal held through 4 misses
    assert all(t[0] == pytest.approx(0.1) for t in follower.targets)
    assert follower.calls[-1] == "hold"


def test_intermittent_misses_and_read_errors_are_tolerated():
    follower = FakeArm(state=State.ENABLED)
    script = [ZERO] + [None, pose(0.1), OSError("glitch"), pose(0.2), None, pose(0.3)]
    reason, _, _ = run(ScriptedLeader(script), follower, cycles=6, miss_limit=2)
    assert reason == "stopped" and follower.targets[-1][0] == pytest.approx(0.3)


def test_a_leader_error_stops_at_once():
    follower = FakeArm(state=State.ENABLED)
    reason, _, _ = run(ScriptedLeader([ZERO, LeaderError("servo 3 lost")]), follower)
    assert reason == "leader_lost: servo 3 lost"


def test_a_follower_fault_stops_the_loop():
    follower = FakeArm(state=State.ENABLED)
    clock = FakeClock()
    stop = threading.Event()

    def hook(step):
        if len(follower.targets) == 3:
            follower.fault("motor 2 overcurrent")

    reason = Teleop(ScriptedLeader([ZERO]), follower, on_step=hook, clock=clock, sleep=clock.sleep).run(stop)
    assert reason == "follower_fault: motor 2 overcurrent"
    assert follower.calls[-1] == "hold"


def test_a_stall_holds_and_names_the_motor():
    follower = FakeArm(state=State.ENABLED)
    records = []

    class Stall:
        def record(self, elapsed, targets):
            records.append((elapsed, targets))
            return [SimpleNamespace(motor_id=3)] if len(records) == 4 else []

    reason, _, _ = run(ScriptedLeader([pose(0.1)]), follower, stall=Stall())
    assert reason == "stall: motor 3" and len(records) == 4
    assert follower.calls[-1] == "hold"


def test_a_crash_inside_the_loop_still_holds():
    follower = FakeArm(state=State.ENABLED)
    follower.set_joint_targets = lambda targets: 1 / 0
    reason, _, _ = run(ScriptedLeader([ZERO]), follower)
    assert reason.startswith("error") and follower.calls[-1] == "hold"


@pytest.mark.parametrize("kwargs", [dict(rate_hz=0), dict(max_velocity=0), dict(max_acceleration=-1),
                                    dict(miss_limit=0), dict(sync_velocity=math.nan)])
def test_bad_settings_are_refused(kwargs):
    with pytest.raises(ValueError):
        Teleop(ScriptedLeader([ZERO]), FakeArm(), **kwargs)


# ── bimanual ──
def test_run_many_stops_both_pairs_when_one_stops():
    left, right = FakeArm(state=State.ENABLED), FakeArm(state=State.ENABLED)
    bad = ScriptedLeader([ZERO, LeaderError("unplugged")])
    good = ScriptedLeader([pose(0.1)])
    reasons = run_many([Teleop(bad, left, rate_hz=200), Teleop(good, right, rate_hz=200)])
    assert reasons == ["leader_lost: unplugged", "stopped"]
    assert left.calls[-1] == right.calls[-1] == "hold"


def test_run_many_honours_the_callers_stop_event():
    arms = [FakeArm(state=State.ENABLED), FakeArm(state=State.ENABLED)]
    stop = threading.Event()
    timer = threading.Timer(0.1, stop.set)
    timer.start()
    reasons = run_many([Teleop(ScriptedLeader([ZERO]), a, rate_hz=200) for a in arms], stop)
    assert reasons == ["stopped", "stopped"] and all(a.targets for a in arms)
