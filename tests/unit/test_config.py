import math

import pytest

from maker_arm.config import ArmConfig, JointConfig


def write(tmp_path, text):
    path = tmp_path / "arm.yaml"
    path.write_text(text)
    return path


def test_limits_in_degrees_and_deg_suffixed_settings(tmp_path):
    cfg = ArmConfig.from_yaml(write(tmp_path, """
startup_step_deg: 2.0
joints:
  - {motor_id: 1, limits_deg: [-90, 45], kp: 10, kd: 1}
"""))
    assert (cfg.joints[0].lo, cfg.joints[0].hi) == pytest.approx((-math.pi / 2, math.pi / 4))
    assert cfg.startup_step == pytest.approx(math.radians(2.0))


def test_unknown_field_is_reported(tmp_path):
    with pytest.raises(ValueError, match="unrecognized"):
        ArmConfig.from_yaml(write(tmp_path, "wire_protocol: private\njoints: []\n"))


@pytest.mark.parametrize("joint, message", [
    (dict(motor_id=0, lo=-1, hi=1, kp=1, kd=1), "motor_id"),
    (dict(motor_id=1, lo=-1, hi=1, kp=1, kd=6), "kd"),
    (dict(motor_id=1, lo=-1, hi=1, kp=1, kd=1, model="RS99"), "unknown model"),
    (dict(motor_id=1, lo=1, hi=-1, kp=1, kd=1), "limits"),
    (dict(motor_id=1, lo=-1, hi=1, kp=1, kd=1, can_timeout_ms=0), "can_timeout_ms"),
    (dict(motor_id=1, lo=-1, hi=1, kp=1, kd=1, max_temperature_c=50, enable_temperature_c=50), "enable_temperature_c"),
])
def test_validation(joint, message):
    with pytest.raises(ValueError, match=message):
        ArmConfig(joints=[JointConfig(**joint)])._validate()


def test_required_ids_in_order():
    joints = [JointConfig(motor_id=i, lo=-1, hi=1, kp=1, kd=1) for i in (2, 1)]
    with pytest.raises(ValueError, match="exactly"):
        ArmConfig(required_motor_ids=[1, 2], joints=joints)._validate()
