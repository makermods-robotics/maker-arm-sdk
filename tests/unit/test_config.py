import pytest
import yaml

from maker_arm.config import ArmConfig

GOOD = {
    "control_rate_hz": 200, "max_velocity": 1.5, "limit_margin": 0.05,
    "feedback_timeout": 0.2, "motor_can_timeout_ms": 200,
    "joints": [
        {"motor_id": 1, "direction": 1, "offset": 0.0, "lo": -2.79, "hi": 2.79, "kp": 60, "kd": 2.0},
        {"motor_id": 2, "direction": -1, "offset": 0.1, "lo": 0.0, "hi": 3.66, "kp": 80, "kd": 3.0},
    ],
}


def write(tmp_path, cfg):
    f = tmp_path / "arm.yaml"
    f.write_text(yaml.safe_dump(cfg))
    return str(f)


def test_load_good(tmp_path):
    cfg = ArmConfig.from_yaml(write(tmp_path, GOOD))
    assert cfg.n_joints == 2
    assert cfg.joints[1].direction == -1 and cfg.joints[1].offset == 0.1
    assert cfg.control_rate_hz == 200


@pytest.mark.parametrize("mutate,msg", [
    (lambda c: c["joints"][0].update(direction=2), "direction"),
    (lambda c: c["joints"][0].update(lo=5.0), "lo"),
    (lambda c: c["joints"][1].update(motor_id=1), "unique"),
    (lambda c: c["joints"][0].update(kd=9.0), "kd"),
    (lambda c: c.update(motor_can_timeout_ms=0), "CAN_TIMEOUT"),
    (lambda c: c.update(limit_margin=-0.5), "limit_margin"),
])
def test_validation(tmp_path, mutate, msg):
    import copy
    bad = copy.deepcopy(GOOD)
    mutate(bad)
    with pytest.raises(ValueError, match=msg):
        ArmConfig.from_yaml(write(tmp_path, bad))


@pytest.mark.parametrize("mutate", [
    lambda c: c.pop("joints"),                       # missing joints
    lambda c: c.update(unknown_key=1),               # unknown top-level key
    lambda c: c["joints"][0].update(typo_field=1),   # misspelled joint field
])
def test_malformed_yaml_raises_valueerror(tmp_path, mutate):
    import copy
    bad = copy.deepcopy(GOOD)
    mutate(bad)
    with pytest.raises(ValueError):
        ArmConfig.from_yaml(write(tmp_path, bad))
