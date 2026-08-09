import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from calib_star_map import validate_anchor_positions  # noqa: E402

from maker_arm.config import JointConfig  # noqa: E402


def _j(mid, lo, hi):
    return JointConfig(motor_id=mid, direction=1, offset=0.0, lo=lo, hi=hi, kp=30, kd=1.0)


def test_anchor_validation_flags_wrapped_reading():
    joints = [_j(1, -2.78, 2.72), _j(7, -2.08, -0.05)]
    bad = validate_anchor_positions([6.36, -1.0], joints)
    assert len(bad) == 1 and "电机1" in bad[0] and "2π" in bad[0]


def test_anchor_validation_passes_sane_and_grace():
    joints = [_j(1, -2.78, 2.72), _j(7, -2.08, -0.05)]
    assert validate_anchor_positions([0.0, -1.0], joints) == []
    assert validate_anchor_positions([2.75, 0.02], joints) == []   # 各超 0.03/0.07 < 0.1 宽限
