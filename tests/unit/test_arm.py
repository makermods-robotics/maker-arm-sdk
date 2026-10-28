import math
from dataclasses import replace
import time

import pytest

from fakes import FakeBus, FakeMotor, two_joint_config
from maker_arm.arm import Arm, ArmState
from maker_arm.errors import ConnectError, StateError


@pytest.fixture(autouse=True)
def fast_param_probe(monkeypatch):
    monkeypatch.setattr(Arm, "PARAM_TIMEOUT", 0.005)


def make(motors=None, **overrides):
    motors = motors or [FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5)]
    bus = FakeBus(motors)
    arm = Arm(two_joint_config(**overrides), bus)
    return arm, bus, {m.motor_id: m for m in motors}


def commands(fake):
    return [entry for entry in fake.log if isinstance(entry, tuple) and entry[0] == "cmd"]


# ── connect ──
def test_connect_reads_positions_without_touching_torque():
    arm, _, fakes = make([FakeMotor(1, position=0.3, enabled=True), FakeMotor(2, position=-0.5)])
    arm.connect()
    assert arm.state is ArmState.CONNECTED
    assert arm.get_joint_positions() == pytest.approx([0.3, -0.5], abs=1e-3)
    assert fakes[1].enabled  # an energized arm is not dropped by connect
    for fake in fakes.values():
        assert fake.log[0] == "fault_query"
        assert "disable" not in fake.log and "enable" not in fake.log


def test_connect_names_missing_motors():
    arm, bus, _ = make([FakeMotor(1), FakeMotor(2, online=False)])
    arm.REPLY_TIMEOUT = 0.005
    with pytest.raises(ConnectError, match=r"no reply from motors \[2\]"):
        arm.connect()
    assert not bus.is_open and arm.state is ArmState.DISCONNECTED


def test_connect_keeps_fault_evidence_by_default():
    arm, _, fakes = make([FakeMotor(1), FakeMotor(2, fault_word=1)])
    with pytest.raises(ConnectError, match="motor overtemperature"):
        arm.connect()
    assert fakes[2].fault_word == 1
    assert "clear" not in fakes[2].log


def test_connect_can_clear_faults_explicitly():
    arm, _, fakes = make([FakeMotor(1), FakeMotor(2, fault_word=1, position=-0.5)])
    arm.connect(clear_faults=True)
    assert fakes[2].fault_word == 0 and arm.state is ArmState.CONNECTED


def test_connect_refuses_motor_not_in_mit_control_mode():
    arm, _, _ = make([FakeMotor(1), FakeMotor(2, position=-0.5, params_supported=True, run_mode=1)])
    with pytest.raises(ConnectError, match="RUN_MODE"):
        arm.connect()


def test_full_turn_correction_is_applied_per_session():
    arm, _, _ = make([FakeMotor(1, position=0.3 + 2 * math.pi), FakeMotor(2, position=-0.5)])
    arm.connect()
    assert arm.get_joint_positions()[0] == pytest.approx(0.3, abs=1e-3)


# ── enable ──
def test_enable_preloads_neutral_command_then_enables():
    arm, _, fakes = make()
    arm.connect()
    arm.enable(start_loop=False)
    for fake in fakes.values():
        i = fake.log.index("enable")
        assert fake.log[i - 1] == ("cmd", 0.0)
    assert arm.state is ArmState.ENABLED
    assert arm.rest_pose == pytest.approx([0.3, -0.5], abs=1e-3)


def test_enable_refuses_stale_zero():
    arm, _, fakes = make([FakeMotor(1, position=3.0), FakeMotor(2, position=-0.5)])
    arm.connect()
    with pytest.raises(ConnectError, match="stale or missing zero"):
        arm.enable(start_loop=False)
    assert "enable" not in fakes[1].log


def test_enable_refuses_hot_gripper():
    arm, _, fakes = make([FakeMotor(1), FakeMotor(2, position=-0.5, temperature=46.0)])
    arm.connect()
    with pytest.raises(ConnectError, match="below 45"):
        arm.enable(start_loop=False)
    assert all("enable" not in f.log for f in fakes.values())


def test_gripper_watchdog_armed_only_with_parameter_support_and_restored():
    gripper = FakeMotor(2, position=-0.5, params_supported=True, can_timeout=0)
    arm, _, fakes = make([FakeMotor(1, params_supported=True), gripper])
    arm.connect()
    arm.enable(start_loop=False)
    assert gripper.can_timeout == 10000          # 500 ms
    assert fakes[1].can_timeout == 0              # joints: no watchdog, as MakerModsLab
    arm.disable()
    assert gripper.can_timeout == 0


def test_legacy_firmware_runs_without_watchdog():
    arm, _, _ = make()
    arm.connect()
    arm.enable(start_loop=False)
    assert arm.state is ArmState.ENABLED


def test_enable_rejected_when_mode_does_not_reach_run():
    class Stubborn(FakeMotor):
        @property
        def mode(self):
            return 0
    arm, _, fakes = make([FakeMotor(1, params_supported=True), Stubborn(2, position=-0.5, params_supported=True)])
    arm.connect()
    with pytest.raises(ConnectError, match="enable rejected"):
        arm.enable(start_loop=False)
    assert not fakes[1].enabled


# ── control ──
def enabled_arm(motors=None, **overrides):
    arm, bus, fakes = make(motors, **overrides)
    arm.connect()
    arm.enable(start_loop=False)
    return arm, bus, fakes


def test_startup_sync_caps_command_near_measured_pose():
    arm, _, fakes = enabled_arm()
    assert arm.set_joint_targets([1.5, -0.5])
    arm._tick(0.005)
    step = math.radians(1.0)
    assert arm.get_commanded_positions()[0] == pytest.approx(0.3 + step, abs=1e-3)


def test_joint_is_synced_once_within_three_degrees():
    arm, _, _ = enabled_arm()
    arm.set_joint_targets([0.3 + math.radians(2.5), -0.5])
    arm._tick(0.005)
    assert arm._synced[0]


def test_stuck_joint_is_released_after_one_second():
    arm, _, fakes = enabled_arm()
    fakes[1].track = False
    arm.set_joint_targets([1.5, -0.5])
    arm._tick(0.005)
    assert not arm._synced[0]
    anchor, since = arm._stuck_anchor[0]
    arm._stuck_anchor[0] = (anchor, since - 1.1)
    arm._tick(0.005)
    assert arm._synced[0]


def test_targets_are_clipped_to_limits():
    arm, _, _ = enabled_arm()
    arm._synced = [True, True]
    arm.set_joint_targets([5.0, 0.5])
    arm._tick(1.0)
    assert arm.get_commanded_positions() == pytest.approx([2.0, -0.04])


def test_non_finite_targets_rejected():
    arm, _, _ = enabled_arm()
    assert not arm.set_joint_targets([math.nan, 0.0])


def test_tick_polls_faults_without_clearing():
    arm, _, fakes = enabled_arm()
    arm._poll_sent = {1: 0.0, 2: 0.0}
    fakes[1].log.clear()
    fakes[2].log.clear()
    arm._tick(0.005)
    assert fakes[1].log.count("fault_query") == 1 and "fault_query" not in fakes[2].log
    arm._tick(0.005)
    assert fakes[2].log.count("fault_query") == 1    # one motor per tick, round-robin
    assert "clear" not in fakes[1].log + fakes[2].log


# ── protection ──
def test_gripper_overtemperature_releases_gripper_and_holds_arm():
    arm, _, fakes = enabled_arm()
    fakes[2].temperature = 55.0
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT
    assert "overtemperature" in arm.fault_reason
    assert not fakes[2].enabled and fakes[1].enabled
    sent = len(commands(fakes[2]))
    arm._tick(0.005)
    assert len(commands(fakes[2])) == sent   # released gripper gets no more commands


def test_fault_word_latches_and_holds_pose():
    arm, _, fakes = enabled_arm()
    fakes[1].fault_word = 1 << 14
    arm._poll_sent = {1: 0.0, 2: 1e9}
    arm._tick(0.005)   # sends the query; reply latches
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT and "stall/overload" in arm.fault_reason
    assert fakes[1].enabled                   # joints hold instead of dropping
    assert fakes[1].fault_word == 1 << 14     # evidence kept


def test_feedback_timeout_faults():
    arm, _, fakes = enabled_arm(feedback_timeout=0.01)
    fakes[1].online = False
    time.sleep(0.02)
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT and "feedback timeout" in arm.fault_reason


def test_modern_firmware_leaving_run_mode_faults():
    arm, _, fakes = enabled_arm([FakeMotor(1, params_supported=True), FakeMotor(2, position=-0.5)])
    fakes[1].enabled = False   # motor dropped out of run mode by itself
    for _ in range(Arm.MODE_FAULT_TICKS):
        fakes[1].enabled = False
        arm._tick(0.005)
    assert arm.state is ArmState.FAULT and "left run mode" in arm.fault_reason


def test_clear_faults_releases_and_clears():
    arm, _, fakes = enabled_arm()
    fakes[1].fault_word = 1
    arm._poll_sent = {1: 0.0, 2: 1e9}
    arm._tick(0.005)
    arm._tick(0.005)
    arm.clear_faults()
    assert arm.state is ArmState.CONNECTED and fakes[1].fault_word == 0
    assert all(m.fault_latched is None for m in arm.motors)


def test_clear_faults_requires_connection():
    arm, _, _ = make()
    with pytest.raises(StateError):
        arm.clear_faults()


# ── stop path ──
def fast_rest(**overrides):
    return {"rest_speed": math.radians(600), "rest_rate_hz": 200, "rest_timeout": 2.0, **overrides}


def test_stop_returns_to_rest_then_releases():
    arm, _, fakes = make(**fast_rest())
    arm.connect()
    arm.enable()
    try:
        arm._synced = [True, True]
        arm.set_joint_targets([1.0, -1.0])
        time.sleep(0.15)
        assert arm.get_joint_positions()[0] == pytest.approx(1.0, abs=0.01)
        assert arm.stop() is True
        assert arm.get_joint_positions()[0] == pytest.approx(0.3, abs=math.radians(2))
        assert arm.get_joint_positions()[1] == pytest.approx(-1.0, abs=0.01)   # gripper excluded
        assert arm.state is ArmState.CONNECTED
        assert not fakes[1].enabled
    finally:
        arm.disconnect()


def test_unverified_return_keeps_holding():
    arm, _, fakes = make(**fast_rest(rest_timeout=0.3))
    arm.connect()
    arm.enable()
    try:
        arm._synced = [True, True]
        arm.set_joint_targets([1.0, -0.5])
        time.sleep(0.15)
        fakes[1].track = False   # joint can no longer move (blocked)
        assert arm.stop() is False
        assert arm.state is ArmState.ENABLED and fakes[1].enabled
    finally:
        arm.disable()
        arm.disconnect()


def test_already_at_rest_releases_immediately():
    arm, _, fakes = make(**fast_rest())
    arm.connect()
    arm.enable()
    try:
        assert arm.stop() is True
        assert not fakes[1].enabled
    finally:
        arm.disconnect()


# ── regressions from the independent review ──
def test_second_fault_while_holding_is_still_handled():
    arm, _, fakes = enabled_arm()
    fakes[1].fault_word = 1
    arm._poll_sent = {1: 0.0, 2: 1e9}
    arm._tick(0.005)
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT and fakes[2].enabled
    fakes[2].temperature = 70.0
    arm._tick(0.005)
    assert not fakes[2].enabled and 1 in arm._released   # gripper is joint index 1
    sent = len(commands(fakes[2]))
    held = fakes[1].position
    fakes[1].log.clear()
    arm._tick(0.005)
    assert len(commands(fakes[2])) == sent            # released gripper gets no commands
    assert "disable" in fakes[2].log or arm._release_sent   # torque-off keeps being re-sent
    assert commands(fakes[1]) and fakes[1].position == pytest.approx(held, abs=1e-3)   # j1 holds


def test_startup_sync_waits_for_the_first_real_target():
    arm, _, _ = enabled_arm()
    arm._tick(0.005)
    arm._tick(0.005)
    assert arm._synced == [False, False]
    arm.set_joint_targets([1.5, -0.5])
    arm._tick(0.005)
    assert arm.get_commanded_positions()[0] == pytest.approx(0.3 + math.radians(1.0), abs=1e-3)


def test_targets_refused_while_returning_to_rest():
    arm, _, _ = enabled_arm()
    arm._returning = True
    assert not arm.set_joint_targets([0.3, -0.5])


def test_stop_with_pending_target_still_ends_at_rest():
    arm, _, fakes = make(**fast_rest())
    arm.connect()
    arm.enable()
    try:
        arm._synced = [True, True]
        arm.set_joint_targets([1.5, -0.5])
        assert arm.stop() is True
        assert fakes[1].position == pytest.approx(0.3, abs=math.radians(2))
        assert not fakes[1].enabled
    finally:
        arm.disconnect()


def test_estop_during_enable_cancels_it():
    import threading

    class EstopOnEnable(FakeMotor):
        arm = None

        def handle(self, can_id, data):
            if data == bytes.fromhex("fffffffffffffffc"):
                threading.Thread(target=self.arm.estop).start()
                time.sleep(0.02)
            return super().handle(can_id, data)

    motor2 = EstopOnEnable(2, position=-0.5)
    arm, _, fakes = make([FakeMotor(1, position=0.3), motor2])
    motor2.arm = arm
    arm.connect()
    with pytest.raises(StateError, match="e-stop"):
        arm.enable(start_loop=False)
    time.sleep(0.05)
    assert arm.state is ArmState.CONNECTED
    assert not fakes[1].enabled and not fakes[2].enabled


def test_send_error_keeps_the_loop_holding_the_other_joints():
    arm, bus, fakes = make()
    arm.connect()
    arm.enable()
    try:
        original = bus.send
        failed = []

        def flaky(can_id, data, extended=True):
            if can_id == 1 and data[:6] != bytes([0xFF] * 6) and not failed:
                failed.append(True)
                raise OSError("USB hiccup")
            return original(can_id, data, extended)

        bus.send = flaky
        assert wait(lambda: arm.state is ArmState.FAULT)
        before = len(commands(fakes[2]))
        time.sleep(0.05)
        assert arm._running and len(commands(fakes[2])) > before
    finally:
        arm.disable()
        arm.disconnect()


def test_unconfirmed_disable_keeps_gripper_watchdog():
    class DropsDisable(FakeMotor):
        def handle(self, can_id, data):
            if data == bytes.fromhex("fffffffffffffffd"):
                return []
            return super().handle(can_id, data)

    gripper = DropsDisable(2, position=-0.5, params_supported=True)
    arm, _, _ = make([FakeMotor(1, params_supported=True), gripper])
    arm.connect()
    arm.enable(start_loop=False)
    arm.disable()
    assert arm.unconfirmed_disable == [2]
    assert gripper.can_timeout == 10000   # not removed while the gripper may still be enabled


def test_enable_refused_when_neutral_preload_is_not_acknowledged():
    class IgnoresCommandsWhileDisabled(FakeMotor):
        def handle(self, can_id, data):
            if data[:6] != bytes([0xFF] * 6) and can_id & 0x700 == 0 and not self.enabled:
                return []
            return super().handle(can_id, data)

    arm, _, fakes = make([IgnoresCommandsWhileDisabled(1, position=0.3), FakeMotor(2, position=-0.5)])
    arm.connect()
    with pytest.raises(ConnectError, match="neutral preload"):
        arm.enable(start_loop=False)
    assert "enable" not in fakes[1].log


def test_refresh_never_erases_a_new_fault():
    arm, _, fakes = make()
    arm.connect()
    fakes[1].fault_word = 1 << 14
    fakes[1].log.clear()
    assert arm.refresh() == [False, True]
    assert fakes[1].fault_word == 1 << 14 and "clear" not in fakes[1].log


def test_gripper_warning_flag_is_a_fault():
    arm, _, fakes = enabled_arm(joints=[
        two_joint_config().joints[0],
        replace(two_joint_config().joints[1], warning_is_fault=True),
    ])
    original = fakes[2].feedback

    def warned():
        cid, data = original()
        data = bytearray(data)
        data[6] |= 0x10
        return cid, bytes(data)

    fakes[2].feedback = warned
    arm._tick(0.005)
    assert arm.state is ArmState.FAULT and "warning" in arm.fault_reason
    assert not fakes[2].enabled


def test_gripper_closing_effort_is_capped():
    hold_joint = replace(two_joint_config().joints[1], hold_torque_nm=0.5)
    arm, bus, fakes = enabled_arm(joints=[two_joint_config().joints[0], hold_joint])
    fakes[2].track = False   # jaws blocked by an object
    arm._synced = [True, True]
    arm.set_joint_targets([0.3, -0.04])   # close fully: 0.46 rad of error
    arm._tick(1.0)
    assert fakes[2].last_kp * 0.46 <= 1.0 + 1e-6   # Kp x error <= 1 Nm closing cap


def test_released_gripper_disable_is_retried_after_a_lost_frame():
    arm, bus, fakes = enabled_arm()
    original = bus.send
    dropped = []

    def drop_first_disable(can_id, data, extended=True):
        if can_id == 2 and data == bytes.fromhex("fffffffffffffffd") and not dropped:
            dropped.append(True)
            return None
        return original(can_id, data, extended)

    bus.send = drop_first_disable
    fakes[2].temperature = 60.0
    arm._tick(0.005)
    assert dropped and fakes[2].enabled         # the first torque-off was lost
    arm._release_sent[1] = 0.0
    arm._tick(0.005)
    assert not fakes[2].enabled                  # re-sent on a later tick


def test_disable_sends_torque_off_even_if_neutral_command_fails():
    arm, bus, fakes = enabled_arm()
    original = bus.send

    def no_commands(can_id, data, extended=True):
        if data[:6] != bytes([0xFF] * 6) and can_id & 0x700 == 0:
            raise OSError("command path broken")
        return original(can_id, data, extended)

    bus.send = no_commands
    arm.disable()
    assert not fakes[1].enabled and not fakes[2].enabled


def test_offline_motor_does_not_delay_torque_off_to_the_others():
    arm, bus, fakes = enabled_arm()
    fakes[1].online = False
    order = []
    original = bus.send

    def record(can_id, data, extended=True):
        if data == bytes.fromhex("fffffffffffffffd"):
            order.append((can_id, time.monotonic()))
        return original(can_id, data, extended)

    bus.send = record
    start = time.monotonic()
    arm.disable()
    first_gripper_off = next(t for cid, t in order if cid == 2)
    assert first_gripper_off - start < 0.02
    assert arm.unconfirmed_disable == [1]


def test_stale_loop_session_cannot_send():
    arm, _, fakes = enabled_arm()
    stale = arm._session
    arm._session += 1
    arm._running = True
    fakes[1].log.clear()
    arm._tick(0.005, session=stale)
    assert not commands(fakes[1])
    arm._running = False


def test_enable_refused_while_previous_loop_thread_lives():
    import threading
    arm, _, _ = make()
    arm.connect()
    gate = threading.Event()
    arm._loop_thread = threading.Thread(target=gate.wait)
    arm._loop_thread.start()
    try:
        with pytest.raises(StateError, match="previous control loop"):
            arm.enable(start_loop=False)
    finally:
        gate.set()
        arm._loop_thread.join()


def wait(predicate, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not predicate():
        time.sleep(0.005)
    return predicate()
