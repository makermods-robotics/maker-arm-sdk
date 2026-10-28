import math

import pytest

from maker_arm.mapping import JointMapper

JOINTS = [
    {"servo": 0, "direction": 1.5, "range_deg": [-150, 150]},
    {"servo": 6, "direction": -2.0, "range_deg": [-120, -2]},
]


def test_follower_is_raw_times_direction_in_radians():
    out = JointMapper(JOINTS).map({0: 40.0, 6: 30.0})
    assert out == pytest.approx([math.radians(60.0), math.radians(-60.0)])


def test_output_is_clipped_to_range():
    out = JointMapper(JOINTS).map({0: 110.0, 6: -10.0})
    assert out == pytest.approx([math.radians(150.0), math.radians(-2.0)])


def test_whole_servo_turns_are_unwrapped():
    out = JointMapper(JOINTS).map({0: 40.0 + 360.0, 6: 30.0 - 720.0})
    assert out == pytest.approx([math.radians(60.0), math.radians(-60.0)])


def test_missing_reading_holds_last_and_starts_nan():
    m = JointMapper(JOINTS)
    first = m.map({0: 10.0})
    assert math.isnan(first[1])
    assert m.map({6: 20.0})[0] == first[0]


def test_zero_direction_rejected():
    with pytest.raises(ValueError):
        JointMapper([{"servo": 0, "direction": 0, "range_deg": [-1, 1]}])
