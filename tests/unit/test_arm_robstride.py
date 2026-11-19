"""Arm behaviour that depends on RobStride specifics: parameter-capable firmware, mode bits,
RUN_MODE, the CAN-timeout watchdog and the separate fault poll."""

import pytest

from fakes import FakeMotor, make_arm
from maker_arm.arm import Arm, ArmState
from maker_arm.errors import ConnectError
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


def test_set_motor_id_is_refused_without_a_frame():
    arm, bus, _ = make_arm()
    with pytest.raises(Exception, match="Motor Studio"):
        arm.set_motor_id(1, 9)
    assert all(data[:6] == bytes([0xFF] * 6) and data[6:] == bytes([0x00, 0xFB]) for _, data in bus.sent)
    assert not bus.is_open
