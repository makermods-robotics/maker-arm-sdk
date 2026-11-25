"""Arm behaviour that depends on RobStride specifics: parameter-capable firmware, mode bits,
RUN_MODE, the CAN-timeout watchdog and the separate fault poll."""

import math
import time
from dataclasses import replace

import pytest

from fakes import FakeMotor, classify, make_arm
from maker_arm.arm import Arm, ArmState
from maker_arm.errors import ArmError, ConnectError, StateError
from maker_arm.motor import Motor


@pytest.fixture(autouse=True)
def fast_param_probe(monkeypatch):
    monkeypatch.setattr(Arm, "PARAM_TIMEOUT", 0.005)


def enabled_arm(motors=None, **overrides):
    arm, bus, fakes = make_arm(motors, **overrides)
    arm.connect()
    arm.enable(start_loop=False)
    return arm, bus, fakes


def test_connect_refuses_motor_not_in_mit_control_mode():
    arm, _, _ = make_arm([FakeMotor(1), FakeMotor(2, position=-0.5, params_supported=True, run_mode=1)])
    with pytest.raises(ConnectError, match="RUN_MODE"):
        arm.connect()


def test_gripper_watchdog_armed_only_with_parameter_support_and_restored():
    gripper = FakeMotor(2, position=-0.5, params_supported=True, can_timeout=0)
    arm, _, fakes = make_arm([FakeMotor(1, params_supported=True), gripper])
    arm.connect()
    arm.enable(start_loop=False)
    assert gripper.can_timeout == 10000          # 500 ms
    assert fakes[1].can_timeout == 0              # joints: no watchdog
    arm.disable()
    assert gripper.can_timeout == 0


def test_watchdog_restore_of_a_zero_original_still_happens():
    gripper = FakeMotor(2, position=-0.5, params_supported=True, can_timeout=0)
    arm, _, _ = make_arm([FakeMotor(1), gripper])
    arm.connect()
    arm.enable(start_loop=False)
    assert arm._watchdog_restore == {1: 0}
    arm.disable()
    assert gripper.can_timeout == 0 and arm._watchdog_restore == {}


def test_old_firmware_runs_without_watchdog(caplog):
    arm, _, _ = make_arm()
    arm.connect()
    arm.enable(start_loop=False)
    assert arm.state is ArmState.ENABLED
    assert any("no parameter access" in r.getMessage() for r in caplog.records)


def test_enable_rejected_when_mode_does_not_reach_run():
    class Stubborn(FakeMotor):
        @property
        def mode(self):
            return 0
    arm, _, fakes = make_arm([FakeMotor(1, params_supported=True), Stubborn(2, position=-0.5, params_supported=True)])
    arm.connect()
    with pytest.raises(ConnectError, match="enable rejected"):
        arm.enable(start_loop=False)
    assert not fakes[1].enabled


def test_tick_polls_faults_without_clearing():
    arm, _, fakes = enabled_arm()
    arm._poll_sent = {1: 0.0, 2: 0.0}
    fakes[1].log.clear()
    fakes[2].log.clear()
    arm._tick(0.005)
    assert fakes[1].log.count("status_query") == 1 and "status_query" not in fakes[2].log
    arm._tick(0.005)
    assert fakes[2].log.count("status_query") == 1    # one motor per tick, round-robin
    assert "clear" not in fakes[1].log + fakes[2].log


def test_status_deadline_counts_from_the_first_unanswered_query():
    arm, _, fakes = enabled_arm()
    answer = fakes[1].handle
    fakes[1].handle = lambda can_id, data: [] if classify(can_id, data)[0] == "status_query" else answer(can_id, data)
    arm.motors[0]._status_time = time.monotonic() - 5.0   # a slow enable: the last reply is old
    arm._tick(0.005)
    assert arm.state is ArmState.ENABLED                   # nothing was asked yet in this session
    arm._poll_sent = {1: 0.0, 2: 1e9}
    arm._tick(0.005)                                       # asks motor 1, which never answers
    assert arm.state is ArmState.ENABLED
    arm._awaiting_status[0] -= 1.0                         # ... for a second
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT and "no fault-status reply" in arm.fault_reason


def test_status_queries_that_fail_to_send_still_count_as_unanswered():
    arm, _, _ = enabled_arm()

    def broken(*a, **k):
        raise OSError("send failed")
    arm.motors[0].poll_status = broken
    arm.motors[0]._status_time = time.monotonic() - 5.0    # its last status reply is old
    arm._poll_sent = {1: 0.0, 2: 1e9}
    arm._tick(0.005)                                       # the query to motor 1 fails to go out
    assert arm.state is ArmState.ENABLED
    deadline = 2 * arm.config.fault_poll_interval + arm.config.feedback_timeout
    arm._awaiting_status[0] -= deadline + 0.1              # ... and nothing reached it since
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT and "no fault-status reply" in arm.fault_reason


def test_a_loop_that_could_not_poll_is_not_a_missing_status_reply():
    arm, _, fakes = enabled_arm()
    arm._poll_sent = {1: 0.0, 2: 1e9}
    arm._tick(0.005)                                       # motor 1 answers this query
    arm.motors[0]._status_time = time.monotonic() - 5.0    # then the loop did not run for five seconds
    arm._tick(0.005)
    assert arm.state is ArmState.ENABLED


def test_modern_firmware_leaving_run_mode_faults():
    arm, _, fakes = enabled_arm([FakeMotor(1, params_supported=True), FakeMotor(2, position=-0.5)])
    for _ in range(Motor.MODE_FAULT_TICKS - 1):
        fakes[1].enabled = False   # motor dropped out of run mode by itself
        arm._tick(0.005)
    assert arm.state is ArmState.ENABLED
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT and "left run mode" in arm.fault_reason


def test_mode_count_restarts_at_every_enable():
    arm, _, fakes = enabled_arm([FakeMotor(1, params_supported=True), FakeMotor(2, position=-0.5)])
    fakes[1].enabled = False
    for _ in range(Motor.MODE_FAULT_TICKS - 1):
        arm._tick(0.005)
    arm.disable()
    arm.enable(start_loop=False)
    fakes[1].enabled = False
    arm._tick(0.005)
    assert arm.state is ArmState.ENABLED


def test_unconfirmed_disable_keeps_gripper_watchdog():
    class DropsDisable(FakeMotor):
        def handle(self, can_id, data):
            if data == bytes.fromhex("fffffffffffffffd"):
                return []
            return super().handle(can_id, data)

    gripper = DropsDisable(2, position=-0.5, params_supported=True)
    arm, _, _ = make_arm([FakeMotor(1, params_supported=True), gripper])
    arm.connect()
    arm.enable(start_loop=False)
    arm.disable()
    assert arm.unconfirmed_disable == [2]
    assert gripper.can_timeout == 10000   # not removed while the gripper may still be enabled


def test_watchdog_readback_mismatch_refuses_enable_and_restores():
    class Clamps(FakeMotor):
        def handle(self, can_id, data):
            replies = super().handle(can_id, data)
            if can_id & 0x700 == 0x400:
                self.can_timeout = 7   # firmware stores something else
            return replies

    gripper = Clamps(2, position=-0.5, params_supported=True, can_timeout=123)
    arm, _, _ = make_arm([FakeMotor(1), gripper])
    arm.connect()
    with pytest.raises(ConnectError, match="read back"):
        arm.enable(start_loop=False)
    assert ("write", 0x7028) in gripper.log
    assert "enable" not in gripper.log


def test_set_motor_id_changes_the_motor_and_routes_the_joint_to_it():
    arm, bus, motors = make_arm()
    arm.set_motor_id(1, 9, torque_off_confirmed=True)
    assert motors[1].motor_id == 9 and not bus.is_open
    assert arm.config.joints[0].motor_id == 9
    arm.set_motor_id(9, 1, torque_off_confirmed=True)
    assert motors[1].motor_id == 1 and arm.config.joints[0].motor_id == 1


def id_writes(bus):
    return [i for i, d in bus.sent if bytes(d)[:6] == bytes([0xFF] * 6) and bytes(d)[7] == 0xFA]


def test_set_motor_id_refuses_an_id_another_joint_has_in_the_profile_even_when_it_is_offline():
    arm, bus, motors = make_arm()
    motors[2].online = False
    with pytest.raises(ArmError, match="belongs to"):
        arm.set_motor_id(1, 2, torque_off_confirmed=True)
    assert id_writes(bus) == [] and motors[1].motor_id == 1


def test_set_motor_id_reads_the_motor_before_writing():
    # The public entry point builds a fresh Motor, which has no feedback until set_id reads it.
    arm, bus, motors = make_arm([FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5)])
    with pytest.raises(StateError, match="torque_off_confirmed"):
        arm.set_motor_id(1, 9)                                   # old firmware: torque state unknown
    motors[1].torque = 0.3                                       # holding torque: energized
    with pytest.raises(StateError, match="energized"):
        arm.set_motor_id(1, 9, torque_off_confirmed=True)
    motors[1].torque = 0.0
    motors[1].params_supported = True
    motors[1].enabled = True                                     # energized by another program
    with pytest.raises(StateError, match="torque on"):
        arm.set_motor_id(1, 9, torque_off_confirmed=True)
    assert id_writes(bus) == [] and motors[1].motor_id == 1


# ── settling onto the support before torque off ──
class SupportedMotor(FakeMotor):
    """A loaded joint (gravity pulls toward lower angles) that lies on a support at ``support`` rad. Holding at a
    command it sags by load/kp; coming down it stops up to ``lag`` rad short (on the lifted side), as a real joint's
    friction leaves it; released, it stays where it is (static friction) unless nothing holds it up."""

    def __init__(self, motor_id, position, load=0.3, support=0.3, lag=0.03, **kw):
        super().__init__(motor_id, position=position, **kw)
        self.load, self.support, self.lag = load, support, lag
        self.track = False                       # the position is this model's, not the command's

    def handle(self, can_id, data):
        before = len(self.frames)
        replies = super().handle(can_id, data)
        if len(self.frames) > before and self.enabled:
            cmd, kp, tau = self.frames[-1]
            free = cmd - (self.load - tau) / kp if kp > 1e-6 else -1e9
            x = self.position if len(self.frames) > 1 else free
            prev = getattr(self, "_x", self.position)
            if free < prev - self.lag:
                prev = free + self.lag          # coming down: stops up to lag above where it is pulled
            elif free > prev:
                prev = free
            self._x = max(self.support, prev)
            self.position = self._x
        return replies


def supported_arm(position, support=0.3, lag=0.03, **overrides):
    joint = SupportedMotor(1, position, support=support, lag=lag)
    arm, bus, fakes = make_arm([joint, FakeMotor(2, position=-0.5)],
                               **{"rest_speed": 6.0, "rest_rate_hz": 200, "rest_timeout": 2.0, **overrides})
    return arm, bus, fakes


def run_cycle(arm, up=0.8):
    arm.connect()
    arm.enable()
    arm._synced = [True, True]
    q = arm.get_commanded_positions()
    assert arm.move_to([up, q[1]], timeout=3.0)
    released = arm.stop()
    arm.disconnect()
    return released


HOVER_DEG = [0.0, -15.0, 22.0, 0.0, 0.0, 0.0]


def shoulder_on_its_support(sim, support_deg=-4.5, force_nm=1.0):
    """The simulated shoulder_lift as the real one behaves at rest: it cannot go past its support (``support_deg``,
    short of the profile's clipped rest pose); coming back toward it under power it stops where the proportional
    term balances a standing ``force_nm`` (friction and model error: force/kp short of the command, on the lifted
    side); released, it stays where it is (static friction near its balance point)."""
    motor = sim.motor("shoulder_lift")
    motor.track = False                          # the position is this model's, not the command's
    support = math.radians(support_deg)
    handle, state = motor.handle, {"q": motor.position}

    def supported(can_id, data):
        before = len(motor.frames)
        replies = handle(can_id, data)
        if len(motor.frames) > before:
            cmd, kp, _ = motor.frames[-1]
            q = state["q"]
            if motor.enabled and kp >= 1.0:
                q = max(q, cmd - force_nm / kp) if cmd >= q else cmd
                q = min(q, support)
            state["q"] = motor.position = q
        return replies

    motor.handle = supported


def maker_cycle(config, backend):
    """One process's session: enable at rest, go to hover, stop(); returns the resting shoulder_lift, deg."""
    arm = Arm(config, backend)
    arm._busy_wait_us = lambda us: None
    arm.connect()
    arm.enable()
    try:
        q = arm.get_joint_positions()
        assert arm.move_to([math.radians(v) for v in HOVER_DEG] + q[6:], timeout=5.0)
        assert arm.stop() is True
    finally:
        if arm.state is not ArmState.DISCONNECTED:
            arm.disable()
            arm.disconnect()
    arm.connect()
    lift = math.degrees(arm.get_joint_positions()[1])
    arm.disconnect()
    return lift


def test_enable_and_rest_cycles_in_separate_processes_keep_the_arm_on_its_support():
    from maker_arm import testing
    rest = [0.0, math.radians(-4.5), math.radians(10.0), math.radians(-1.2), 0.0, 0.0, math.radians(-3.0)]
    config = testing.simulation_timing(testing._config(None))
    config = replace(config, rest_speed=math.radians(120), rest_rate_hz=200)
    backend, sim = testing.simulated_backend(config, rest)
    shoulder_on_its_support(sim)
    lifts = [maker_cycle(sim.config, backend) for _ in range(8)]
    assert all(abs(v - (-4.5)) < 0.5 for v in lifts), lifts    # every session ends on the support


def test_an_arm_enabled_away_from_rest_returns_where_it_started():
    from maker_arm import testing
    arm, sim = testing.simulated_arm(positions=[math.radians(v) for v in HOVER_DEG] + [math.radians(-3.0)])
    arm.connect()
    arm.enable()
    try:
        assert arm.rest_pose[:6] == pytest.approx([math.radians(v) for v in HOVER_DEG], abs=1e-3)
    finally:
        arm.disable()
        arm.disconnect()


def test_a_set_id_that_leaves_both_ids_answering_keeps_the_joint_from_enabling():
    source, occupant = FakeMotor(1, position=0.3), FakeMotor(9, online=False)
    source.apply_id_change = False                               # the write is ignored ...
    original = source.handle

    def wake_occupant(can_id, data):                             # ... and a motor on 9, silent before, answers after
        if bytes(data)[:6] == bytes([0xFF] * 6) and bytes(data)[7] == 0xFA:
            occupant.online = True
        return original(can_id, data)

    source.handle = wake_occupant
    arm, bus, _ = make_arm([source, FakeMotor(2, position=-0.5), occupant])
    with pytest.raises(StateError, match="both answer"):
        arm.set_motor_id(1, 9, torque_off_confirmed=True)
    arm.connect()
    with pytest.raises(ConnectError, match="unverified"):
        arm.enable()
    assert not source.enabled
    arm.disconnect()


def test_the_press_toward_the_support_is_bounded_by_effort_and_stays_inside_the_limits():
    from maker_arm import testing
    rest = [0.0, math.radians(-4.5), math.radians(10.0), math.radians(-1.2), 0.0, 0.0, math.radians(-3.0)]
    arm, _ = testing.simulated_arm(positions=rest)
    arm.connect()
    arm.enable()
    try:
        cfg, q = arm.config, arm.get_joint_positions()
        for i in (1, 2, 3):                                         # the joints that carry the arm's weight
            press = abs(arm.rest_pose[i] - q[i])
            assert press <= min(math.radians(3.0), 2.0 / cfg.joints[i].kp) + 1e-9
            assert press * cfg.joints[i].kp <= 2.0 + 1e-9               # N*m of proportional effort at most
            assert cfg.joints[i].lo <= arm.rest_pose[i] <= cfg.joints[i].hi
        assert arm.rest_pose[0] == pytest.approx(q[0]) and arm.rest_pose[4:] == pytest.approx(q[4:])
    finally:
        arm.disable()
        arm.disconnect()
