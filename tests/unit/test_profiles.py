import json

from maker_arm.config import ArmConfig
from maker_arm.mapping import JointMapper
from maker_arm.profiles import DEFAULT_ARM_CONFIG, DEFAULT_STAR_MAPPING


def test_packaged_profiles_are_complete_and_endpoint_safe():
    config = ArmConfig.from_yaml(DEFAULT_ARM_CONFIG)
    mapping = json.loads(DEFAULT_STAR_MAPPING.read_text())
    capture_path = (
        DEFAULT_ARM_CONFIG.parents[2]
        / "tools/engineering/calibration/artifacts/maker_arm02/star_limits_capture.json"
    )
    star = json.loads(capture_path.read_text())["joints"]

    assert config.n_joints == len(mapping["joints"]) == len(star) == 7
    for endpoint in ("min_deg", "max_deg"):
        raw = {joint["servo"]: joint[endpoint] for joint in star}
        targets = JointMapper.from_json(DEFAULT_STAR_MAPPING).map(raw)
        for target, joint in zip(targets, config.joints):
            assert joint.lo - 1e-4 <= target <= joint.hi + 1e-4


def test_public_mapping_is_absolute_only():
    mapping = json.loads(DEFAULT_STAR_MAPPING.read_text())
    assert "mode" not in mapping
    assert all({"zero_deg", "base_rad", "direction", "scale"} <= joint.keys()
               for joint in mapping["joints"])
