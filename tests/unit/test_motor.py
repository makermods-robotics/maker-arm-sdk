import pytest

from fakes import FakeBus, FakeMotor
from maker_arm import protocol as p
from maker_arm.errors import FaultLatched, ParamTimeout
from maker_arm.motor import Motor


def wired(fake: FakeMotor) -> tuple[Motor, FakeBus]:
    bus = FakeBus([fake])
    motor = Motor(fake.motor_id, bus, fake.params)
    bus.set_recv_callback(lambda cid, d: motor.handle_frame(p.parse_frame(cid, d, motor.params)))
    return motor, bus


def test_every_command_is_a_standard_frame():
    motor, bus = wired(FakeMotor(1))
    motor.enable()
    motor.send_command(0.1, 0, 10, 1, 0)
    motor.disable()
    assert bus.sent_extended == [False, False, False]


def test_read_state_returns_fresh_feedback_without_changing_torque():
    fake = FakeMotor(3, position=0.7, enabled=True)
    motor, _ = wired(fake)
    fb = motor.read_state()
    assert fb.position == pytest.approx(0.7, abs=1e-3)
    assert fake.enabled


def test_fault_query_does_not_clear_and_latches():
    fake = FakeMotor(2, fault_word=1 << 2)
    motor, _ = wired(fake)
    status = motor.query_fault()
    assert status.has_fault and fake.fault_word == 1 << 2
    assert "undervoltage" in motor.fault_latched
    with pytest.raises(FaultLatched):
        motor.request_state()
    assert "clear" not in fake.log


def test_feedback_fault_flag_latches():
    motor, _ = wired(FakeMotor(1, fault_flag=True, enabled=True))
    motor.send_command(0.0, 0, 10, 1, 0)
    assert motor.fault_latched == "feedback fault flag set"


def test_explicit_clear_fault_resets_latch():
    fake = FakeMotor(1, fault_word=1)
    motor, _ = wired(fake)
    motor.query_fault()
    motor.clear_fault()
    assert motor.fault_latched is None and fake.fault_word == 0


def test_missing_motor_times_out():
    motor, _ = wired(FakeMotor(1, online=False))
    with pytest.raises(ParamTimeout):
        motor.read_state(timeout=0.005, attempts=2)
    with pytest.raises(ParamTimeout):
        motor.query_fault(timeout=0.005)


def test_params_probe_and_write_readback():
    legacy, _ = wired(FakeMotor(1))
    assert legacy.probe_params(timeout=0.01) is False
    modern, _ = wired(FakeMotor(7, params_supported=True))
    assert modern.probe_params() is True
    assert modern.write_param(p.ParamIndex.CAN_TIMEOUT, 10000, "u32") == 10000


def test_write_acknowledgement_is_not_a_read_result():
    import struct
    motor, bus = wired(FakeMotor(7, params_supported=True))
    bus.responder = lambda cid, data: [(0x407, struct.pack("<H2xI", p.ParamIndex.CAN_TIMEOUT, 10000))]
    with pytest.raises(ParamTimeout):
        motor.read_param(p.ParamIndex.CAN_TIMEOUT, "u32", timeout=0.01)


def test_read_state_safe_refuses_on_a_fresh_fault():
    fake = FakeMotor(1, fault_word=1)
    motor, _ = wired(fake)
    with pytest.raises(FaultLatched):
        motor.read_state_safe()
    assert fake.fault_word == 1
