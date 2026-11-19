"""The shipped profile and Star mapping hold the reference values below."""

import json
import math

import pytest

from maker_arm.config import ArmConfig
from maker_arm.mapping import JointMapper
from maker_arm._product import DEFAULT_PROFILE, DEFAULT_STAR_MAPPING

# name: (motor_id, model, limits_deg, kp, kd)
REFERENCE = {
    "shoulder_pan": (1, "RS00", (-158.2, 156.1), 60, 4.0),
    "shoulder_lift": (2, "RS02", (-175.3, -3.2), 150, 4.5),
    "elbow_flex": (3, "RS02", (2.8, 236.1), 90, 3.0),
    "wrist_flex": (4, "RS00", (-65.2, 104.0), 30, 2.0),
    "wrist_yaw": (5, "RS00", (-97.6, 78.0), 30, 2.0),
    "wrist_roll": (6, "RS00", (-153.2, 152.0), 30, 2.0),
    "gripper": (7, "RS00", (-96.0, -2.5), 20, 0.5),
}

STAR_DIRECTIONS = {
    "shoulder_pan": 1.446501, "shoulder_lift": -0.859866, "elbow_flex": -0.857332,
    "wrist_flex": -0.85871, "wrist_yaw": -1.322941, "wrist_roll": 0.954213, "gripper": -1.983613,
}


def test_profile_matches_the_reference():
    config = ArmConfig.from_yaml(DEFAULT_PROFILE)
    assert [j.name for j in config.joints] == list(REFERENCE)
    for j in config.joints:
        motor_id, model, (lo, hi), kp, kd = REFERENCE[j.name]
        assert (j.motor_id, j.model, j.kp, j.kd, j.direction, j.offset) == (motor_id, model, kp, kd, 1, 0.0)
        assert (math.degrees(j.lo), math.degrees(j.hi)) == pytest.approx((lo, hi))


def test_only_the_gripper_has_watchdog_and_temperature_protection():
    config = ArmConfig.from_yaml(DEFAULT_PROFILE)
    *arm_joints, gripper = config.joints
    assert (gripper.can_timeout_ms, gripper.max_temperature_c, gripper.enable_temperature_c) == (500, 55, 45)
    assert gripper.release_on_fault and not gripper.return_to_rest
    for j in arm_joints:
        assert j.can_timeout_ms is None and j.max_temperature_c is None
        assert j.return_to_rest and not j.release_on_fault
    assert config.feedback_timeout == 0.25


def test_star_mapping_matches_the_reference():
    mapping = json.loads(DEFAULT_STAR_MAPPING.read_text())
    assert [(j["name"], j["servo"]) for j in mapping["joints"]] == [(n, i) for i, n in enumerate(STAR_DIRECTIONS)]
    for j in mapping["joints"]:
        assert j["direction"] == STAR_DIRECTIONS[j["name"]]
        lo, hi = REFERENCE[j["name"]][2]
        if j["name"] != "gripper":
            assert j["range_deg"] == [int(lo), int(hi)]
    JointMapper.from_json(DEFAULT_STAR_MAPPING)


def test_profile_model_is_the_gravity_reference():
    """urdf, base turn, signs, TCP and caps in the profile are the ones gravity_fixtures.json was checked with."""
    from pathlib import Path

    from maker_arm import protocol
    from maker_arm._product import URDF
    from maker_arm.kinematics import resolve_urdf

    spec = json.loads((Path(__file__).resolve().parent / "gravity_fixtures.json").read_text())["chain"]
    config = ArmConfig.from_yaml(DEFAULT_PROFILE)
    arm = config.joints[:6]
    assert resolve_urdf(config.urdf) == URDF == Path(__file__).resolve().parents[2] / spec["urdf"]
    assert [math.degrees(v) for v in config.urdf_base_rpy] == pytest.approx(spec["base_rpy_deg"])
    assert config.tcp_xyz == pytest.approx(spec["tcp_xyz"])
    assert [math.degrees(v) for v in config.tcp_rpy] == pytest.approx(spec["tcp_rpy_deg"])
    assert [j.urdf_sign for j in arm] == spec["sign"]
    assert [math.degrees(j.urdf_offset) for j in arm] == spec["offset_deg"]
    assert [j.gravity_limit_nm for j in arm] == spec["gravity_limit_nm"]
    assert [j.gravity_factor for j in arm] == spec["gravity_factor"]
    assert [[round(math.degrees(j.lo), 6), round(math.degrees(j.hi), 6)] for j in arm] == spec["limits_deg"]
    for j in arm:
        assert j.gravity_limit_nm < protocol.MOTOR_PARAMS[j.model].t_max
