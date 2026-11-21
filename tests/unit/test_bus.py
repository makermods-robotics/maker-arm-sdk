"""Bus.release / release(): verified torque-off motor by motor, where Arm.connect refuses."""

import pytest

from conftest import sdk
from fakes import FakeBus, FakeMotor, two_joint_config


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(sdk.arm.Arm, "PARAM_TIMEOUT", 0.005)


def motors(**second):
    return [FakeMotor(1, position=0.3, params_supported=True, enabled=True, faulted=True),
            FakeMotor(2, position=-0.5, params_supported=True, enabled=True, **second)]


def test_release_turns_off_an_arm_with_a_latched_fault():
    fakes = motors()
    with pytest.raises(sdk.errors.ConnectError):
        sdk.arm.Arm(two_joint_config(), FakeBus(fakes)).connect()
    result = sdk.bus.release(FakeBus(fakes), two_joint_config())
    assert result == {"ok": True, "released": ["j1", "gripper"], "unconfirmed": [], "missing": []}
    assert not any(m.enabled for m in fakes)


def test_release_reports_a_missing_motor_and_releases_the_rest():
    fakes = motors(online=False)
    result = sdk.bus.release(FakeBus(fakes), two_joint_config())
    assert not result["ok"] and result["missing"] == ["gripper"] and result["released"] == ["j1"]
    assert not fakes[0].enabled


def test_release_of_a_silent_bus_is_not_ok():
    fakes = [FakeMotor(1, online=False), FakeMotor(2, online=False)]
    result = sdk.bus.release(FakeBus(fakes), two_joint_config())
    assert not result["ok"] and result["missing"] == ["j1", "gripper"] and result["released"] == []
