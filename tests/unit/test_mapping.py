import json
import math

import pytest

from chain_fixture import load

mapping = load("mapping")
product = load("_product")
JointMapper = mapping.JointMapper

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


def test_unwrap_window_is_the_follower_range_in_leader_degrees():
    # Follower [0, 170] at scale 0.5 is leader [0, 340]: 300 leader deg is 150 follower deg (a window
    # centred on the sign-only 85 would unwrap it to -60 and clip the joint to 0).
    m = JointMapper([{"servo": 1, "direction": 0.5, "range_deg": [0, 170]}])
    assert m.map({1: 300.0}) == pytest.approx([math.radians(150.0)])


def test_missing_reading_holds_last_and_starts_nan():
    m = JointMapper(JOINTS)
    first = m.map({0: 10.0})
    assert math.isnan(first[1])
    assert m.map({6: 20.0})[0] == first[0]


def test_non_finite_reading_is_a_missing_reading():
    m = JointMapper(JOINTS)
    first = m.map({0: 10.0, 6: 5.0})
    assert m.map({0: math.nan, 6: 5.0}) == first


def test_reset_forgets_the_last_output():
    m = JointMapper(JOINTS)
    m.map({0: 10.0, 6: 5.0})
    m.reset()
    assert all(math.isnan(v) for v in m.map({}))


def test_offset_shifts_the_follower_and_its_unwrap_window():
    m = JointMapper([{"servo": 0, "direction": 1.0, "offset_deg": 90.0, "range_deg": [0, 300]}])
    assert m.map({0: 200.0}) == pytest.approx([math.radians(290.0)])  # window centred on 60 leader deg


@pytest.mark.parametrize("raw", [-50.0, 50.0, 50.0 - 360.0])
def test_magnitude_joint_maps_the_distance_pulled_either_way(raw):
    m = JointMapper([{"servo": 6, "direction": -1.2, "magnitude": True, "range_deg": [-120, 0]}])
    assert m.map({6: raw}) == pytest.approx([math.radians(-60.0)])


@pytest.mark.parametrize("joints", [
    [{"servo": 0, "direction": 0, "range_deg": [-1, 1]}],
    [{"servo": 0, "direction": 1, "range_deg": [1, 1]}],
    [{"servo": 0, "direction": 1, "range_deg": [-1, 1]}, {"servo": 0, "direction": 1, "range_deg": [-1, 1]}],
])
def test_bad_mappings_are_rejected(joints):
    with pytest.raises(ValueError):
        JointMapper(joints)


def test_shipped_star_mapping_loads_and_names_seven_servos(tmp_path):
    m = JointMapper.from_json(product.DEFAULT_STAR_MAPPING)
    assert len(m.servo_ids) == len(set(m.servo_ids)) == 7
    raw = json.loads(product.DEFAULT_STAR_MAPPING.read_text())
    assert m.names == [j["name"] for j in raw["joints"]]
    out = m.map({sid: 0.0 for sid in m.servo_ids})
    for value, joint in zip(out, raw["joints"]):
        lo, hi = joint["range_deg"]
        assert math.radians(lo) <= value <= math.radians(hi)
