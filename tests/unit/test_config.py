import math

import pytest

from conftest import sdk

ArmConfig, JointConfig = sdk.config.ArmConfig, sdk.config.JointConfig
ID_LO = sdk.motor.ID_RANGE[0]
KD_HI = sdk.motor.KD_RANGE[1]


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
    (dict(motor_id=ID_LO - 1, lo=-1, hi=1, kp=1, kd=1), "motor_id"),
    (dict(motor_id=1, lo=-1, hi=1, kp=1, kd=KD_HI + 1), "kd"),
    (dict(motor_id=1, lo=-1, hi=1, kp=1, kd=1, model="NO-SUCH-MODEL"), "unknown model"),
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


def test_rest_pose_in_degrees_one_value_per_joint(tmp_path):
    cfg = ArmConfig.from_yaml(write(tmp_path, """
rest_pose_deg: [10]
joints:
  - {motor_id: 1, limits_deg: [-90, 45], kp: 10, kd: 1}
"""))
    assert cfg.rest_pose == pytest.approx([math.radians(10)])
    with pytest.raises(ValueError, match="rest_pose"):
        ArmConfig(rest_pose=[0.0, 0.0], joints=cfg.joints)._validate()


def test_gripper_open_position_in_degrees_inside_the_limits(tmp_path):
    cfg = ArmConfig.from_yaml(write(tmp_path, """
joints:
  - {name: gripper, motor_id: 1, limits_deg: [0, 120], kp: 10, kd: 1, closing_direction: -1, open_position_deg: 100}
"""))
    assert cfg.joints[0].open_position == pytest.approx(math.radians(100))
    for bad in (1.5, -0.1, 0.0, math.nan):   # outside the limits, or the closed end
        with pytest.raises(ValueError, match="open_position"):
            ArmConfig(joints=[JointConfig(motor_id=1, lo=0.0, hi=1.0, kp=1, kd=1, closing_direction=-1,
                                          open_position=bad)])._validate()
