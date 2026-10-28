"""The packaged profile must stay identical to what MakerModsLab drives (LeRobot @5ce2fe41a)."""

import json
import math

import pytest

from maker_arm.config import ArmConfig
from maker_arm.mapping import JointMapper
from maker_arm.profiles import DEFAULT_ARM_CONFIG, DEFAULT_STAR_MAPPING

# name: (motor_id, model, limits_deg, kp, kd) from MakerFollowerConfig; gripper open bound
# from MakerModsLab gripper_settings.py.
MAKERMODSLAB = {
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


def test_profile_matches_makermodslab():
    config = ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    assert [j.name for j in config.joints] == list(MAKERMODSLAB)
    for j in config.joints:
        motor_id, model, (lo, hi), kp, kd = MAKERMODSLAB[j.name]
        assert (j.motor_id, j.model, j.kp, j.kd, j.direction, j.offset) == (motor_id, model, kp, kd, 1, 0.0)
        assert (math.degrees(j.lo), math.degrees(j.hi)) == pytest.approx((lo, hi))


def test_only_the_gripper_has_watchdog_and_temperature_protection():
    config = ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    *arm_joints, gripper = config.joints
    assert (gripper.can_timeout_ms, gripper.max_temperature_c, gripper.enable_temperature_c) == (500, 55, 45)
    assert gripper.release_on_fault and not gripper.return_to_rest
    for j in arm_joints:
        assert j.can_timeout_ms is None and j.max_temperature_c is None
        assert j.return_to_rest and not j.release_on_fault
    assert config.feedback_timeout == 0.25


def test_star_mapping_matches_lerobot_leader_preset():
    mapping = json.loads(DEFAULT_STAR_MAPPING.read_text())
    assert [(j["name"], j["servo"]) for j in mapping["joints"]] == [(n, i) for i, n in enumerate(STAR_DIRECTIONS)]
    for j in mapping["joints"]:
        assert j["direction"] == STAR_DIRECTIONS[j["name"]]
        lo, hi = MAKERMODSLAB[j["name"]][2]
        if j["name"] != "gripper":
            assert j["range_deg"] == [int(lo), int(hi)]
    JointMapper.from_json(DEFAULT_STAR_MAPPING)
