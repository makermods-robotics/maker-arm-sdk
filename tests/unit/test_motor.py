import math
import struct

import pytest

from fakes import FakeBus, FakeMotor, feedback_bytes
from maker_arm import motor as motor_module
from maker_arm import protocol as p
from maker_arm.config import JointConfig
from maker_arm.errors import ArmError, ConnectError, FaultLatched, ParamTimeout
from maker_arm.motor import Motor, MotorFeedback, MotorStatus, route
from maker_arm.transport.base import CanId


def joint(motor_id, model="RS00", **kw):
    return JointConfig(motor_id=motor_id, lo=-1.0, hi=1.0, kp=10.0, kd=1.0, model=model, **kw)


def wired(fake: FakeMotor) -> tuple[Motor, FakeBus]:
    bus = FakeBus([fake])
    motor = Motor(joint(fake.motor_id, fake.model, name="j"), bus)
    bus.set_recv_callback(motor.handle_frame)
    return motor, bus


# ── module-level seam ──
def test_constants_describe_robstride_mit():
    assert motor_module.MODELS == ("RS00", "RS02")
    assert motor_module.DEFAULT_MODEL == "RS00" and JointConfig(1, -1, 1, 1, 1).model == "RS00"
    assert motor_module.KP_RANGE == (0.0, 500.0) and motor_module.KD_RANGE == (0.0, 5.0)
    assert motor_module.ID_RANGE == (1, 127)
    assert motor_module.DEFAULT_HOST_ID == 0xFD


def test_route_uses_the_can_id_for_param_replies_and_data0_on_the_host_id():
    reply = struct.pack("<H2xI", p.ParamIndex.CAN_TIMEOUT, 1)   # data[0] is 0x28, not a motor
    assert route(0x307, reply, 0xFD) == 7
    assert route(0x407, reply, 0xFD) == 7
    assert route(0xFD, feedback_bytes(5), 0xFD) == 5
    assert route(0xFE, feedback_bytes(5), 0xFD) is None
    assert route(CanId(0xFD, extended=True), feedback_bytes(5), 0xFD) is None
    assert route(0xFD, b"", 0xFD) is None


def test_feedback_is_neutral_with_robstride_extras():
    motor, _ = wired(FakeMotor(3, position=0.7, temperature=41.5, enabled=True, warning=True))
    fb = motor.read_state()
    assert isinstance(fb, MotorFeedback)
    assert fb.position == pytest.approx(0.7, abs=1e-3)
    assert fb.temperature == pytest.approx(41.5) and fb.temperatures == {"motor": pytest.approx(41.5)}
    assert fb.enabled is None          # parameter support not probed: mode bits unknown
    assert fb.warning and fb.mode == 0 and not fb.fault_flag


def test_enabled_is_reported_on_parameter_capable_firmware():
    fake = FakeMotor(1, params_supported=True, enabled=True)
    motor, _ = wired(fake)
    motor.validate()
    assert motor.read_state().enabled is True
    fake.enabled = False
    assert motor.read_state().enabled is False


def test_torque_limit_follows_the_model():
    assert Motor(joint(1, "RS00"), FakeBus([])).torque_limit == 14.0
    assert Motor(joint(1, "RS02"), FakeBus([])).torque_limit == 17.0


def test_frames_for_another_motor_are_ignored():
    motor, _ = wired(FakeMotor(1))
    motor.handle_frame(0xFD, feedback_bytes(2, pos=0.5))
    assert motor.feedback is None and motor.feedback_sequence == 0


# ── commands ──
def test_every_command_is_a_standard_frame():
    motor, bus = wired(FakeMotor(1))
    motor.enable()
    motor.send_command(0.1, 0, 10, 1, 0)
    motor.disable()
    motor.poll_status()
    assert bus.sent_extended == [False] * 4


def test_read_state_returns_fresh_feedback_without_changing_torque():
    fake = FakeMotor(3, position=0.7, enabled=True)
    motor, _ = wired(fake)
    fb = motor.read_state()
    assert fb.position == pytest.approx(0.7, abs=1e-3)
    assert fake.enabled


def test_check_does_not_clear_and_latches():
    fake = FakeMotor(2, fault_word=1 << 2)
    motor, _ = wired(fake)
    status = motor.check()
    assert isinstance(status, MotorStatus)
    assert status.has_fault and status.code == 1 << 2 and status.text == "undervoltage"
    assert status.enabled is None
    assert fake.fault_word == 1 << 2
    assert "undervoltage" in motor.fault_latched
    with pytest.raises(FaultLatched):
        motor.read_state()
    assert "clear" not in fake.log


def test_healthy_status_has_empty_text_and_an_age():
    motor, _ = wired(FakeMotor(1))
    assert motor.status is None and motor.status_age == math.inf
    status = motor.check()
    assert not status.has_fault and status.code == 0 and status.text == ""
    assert motor.status is status and motor.status_age < 1.0


def test_poll_status_sends_the_non_clearing_query():
    fake = FakeMotor(1, fault_word=1)
    motor, _ = wired(fake)
    motor.poll_status()
    assert fake.log == ["status_query"] and motor.status.has_fault and fake.fault_word == 1


def test_feedback_fault_flag_latches():
    motor, _ = wired(FakeMotor(1, fault_flag=True, enabled=True))
    motor.send_command(0.0, 0, 10, 1, 0)
    assert motor.fault_latched == "feedback fault flag set"


def test_explicit_clear_fault_resets_latch():
    fake = FakeMotor(1, fault_word=1)
    motor, _ = wired(fake)
    motor.check()
    motor.clear_fault()
    assert motor.fault_latched is None and fake.fault_word == 0


def test_reset_cache_forgets_everything():
    motor, _ = wired(FakeMotor(1, fault_word=1))
    motor.check()
    motor.reset_cache()
    assert motor.status is None and motor.feedback is None and motor.fault_latched is None


def test_missing_motor_times_out():
    motor, _ = wired(FakeMotor(1, online=False))
    with pytest.raises(ParamTimeout):
        motor.read_state(timeout=0.005, attempts=2)
    with pytest.raises(ParamTimeout):
        motor.check(timeout=0.005, attempts=1)


def test_set_zero_persists_in_the_motor():
    fake = FakeMotor(1, position=0.4)
    motor, _ = wired(fake)
    motor.set_zero()
    assert fake.log == ["zero"] and fake.position == 0.0


def test_set_id_is_not_documented_for_mit_and_sends_nothing():
    motor, bus = wired(FakeMotor(1))
    with pytest.raises(ArmError, match="not documented"):
        motor.set_id(5)
    assert bus.sent == []


# ── connect-time validation and mode bits ──
def test_validate_accepts_old_and_mit_mode_firmware():
    old, _ = wired(FakeMotor(1))
    old.validate(timeout=0.005)
    assert old.params_supported is False
    modern, _ = wired(FakeMotor(2, params_supported=True))
    modern.validate()
    assert modern.params_supported is True


def test_validate_refuses_a_motor_outside_mit_mode():
    motor, _ = wired(FakeMotor(1, params_supported=True, run_mode=1))
    with pytest.raises(ConnectError, match="RUN_MODE is 1"):
        motor.validate()


def test_confirms_uses_mode_bits_only_where_the_firmware_has_them():
    old, _ = wired(FakeMotor(1))
    assert not old.confirms("enabled")          # no feedback yet
    old.read_state()
    assert old.confirms("enabled") and old.confirms("disabled")
    fake = FakeMotor(2, params_supported=True)
    modern, _ = wired(fake)
    modern.validate()
    modern.enable()
    assert modern.confirms("enabled") and not modern.confirms("disabled")
    modern.disable()
    assert modern.confirms("disabled") and not modern.confirms("enabled")
    with pytest.raises(KeyError):
        modern.confirms("sleeping")


def test_health_problem_after_mode_fault_ticks_and_reset_on_enable():
    fake = FakeMotor(1, params_supported=True)
    motor, _ = wired(fake)
    motor.validate()
    motor.enable()
    fake.enabled = False
    motor.send_command(0.0, 0, 10, 1, 0)
    for _ in range(Motor.MODE_FAULT_TICKS - 1):
        assert motor.health_problem() is None
    assert "left run mode (mode=0" in motor.health_problem()
    motor.enable()
    assert motor.health_problem() is None


def test_calibration_mode_is_never_healthy_even_on_old_firmware():
    motor, _ = wired(FakeMotor(1))
    motor.handle_frame(0xFD, feedback_bytes(1, mode=1))
    problems = [motor.health_problem() for _ in range(Motor.MODE_FAULT_TICKS)]
    assert problems[-1] and not any(problems[:-1])


# ── watchdog and parameters ──
def test_watchdog_armed_verified_and_restored():
    fake = FakeMotor(7, params_supported=True, can_timeout=42)
    motor, _ = wired(fake)
    motor.validate()
    token = motor.arm_watchdog(500)
    assert token == 42 and fake.can_timeout == 10000
    motor.restore_watchdog(token)
    assert fake.can_timeout == 42


def test_watchdog_unsupported_on_old_firmware():
    fake = FakeMotor(7)
    motor, bus = wired(fake)
    motor.validate(timeout=0.005)
    bus.sent.clear()
    assert motor.arm_watchdog(500) is None
    assert bus.sent == []


def test_params_by_name_index_and_write_readback():
    motor, _ = wired(FakeMotor(7, params_supported=True, run_mode=0))
    assert motor.probe_params() is True
    assert motor.read_param("run_mode") == 0
    assert motor.write_param("can_timeout", 10000) == 10000
    assert motor.read_param(p.ParamIndex.CAN_TIMEOUT) == 10000     # dtype from PARAMS
    assert motor.write_param(p.ParamIndex.CAN_TIMEOUT, 5, "u32") == 5
    assert set(Motor.PARAMS) == {"run_mode", "mech_pos", "can_timeout"}
    with pytest.raises(ValueError):
        motor.read_param("no_such_param")
    with pytest.raises(ArmError, match="flash"):
        motor.write_param("can_timeout", 1, save=True)


def test_params_probe_on_old_firmware():
    old, _ = wired(FakeMotor(1))
    assert old.probe_params(timeout=0.01) is False


def test_write_acknowledgement_is_not_a_read_result():
    motor, bus = wired(FakeMotor(7, params_supported=True))
    bus.responder = lambda cid, data: [(0x407, struct.pack("<H2xI", p.ParamIndex.CAN_TIMEOUT, 10000))]
    with pytest.raises(ParamTimeout):
        motor.read_param(p.ParamIndex.CAN_TIMEOUT, "u32", timeout=0.01)


def test_disable_confirmed_needs_mode_zero_on_modern_firmware():
    class StaysOn(FakeMotor):
        @property
        def mode(self):
            return 2
    motor, _ = wired(StaysOn(1, params_supported=True))
    motor.validate()
    assert motor.disable_confirmed(timeout=0.005, attempts=2) is False
    old, _ = wired(FakeMotor(2))
    assert old.disable_confirmed() is True
