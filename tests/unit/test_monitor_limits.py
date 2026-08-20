import math
import pytest

from tools.engineering.calibration.capture_maker_limits import compute_limits, update_yaml_limits


def test_compute_limits_backoff():
    out = compute_limits([-1.0, 0.0], [2.0, 1.0], backoff=0.05)
    assert out[0] == (-0.95, 1.95)
    assert out[1] == (0.05, 0.95)


def test_compute_limits_guards():
    mins = [math.inf, -0.1, -1.0]
    maxs = [-math.inf, 0.1, 2.0]
    out = compute_limits(mins, maxs, backoff=0.05)
    assert out[0] is None          # never received data
    assert out[1] is None          # travel 0.2 < 0.3, treated as unmeasured
    assert out[2] == (-0.95, 1.95)


def test_compute_limits_backoff_collapse():
    # travel is just past the sanity threshold but backoff is too large, causing lo>=hi -> None
    out = compute_limits([0.0], [0.4], backoff=0.25)
    assert out == [None]


YAML = (
    "# top comment must be preserved\n"
    "control_rate_hz: 200\n"
    "joints:\n"
    "  - {motor_id: 1, direction: 1, offset: 0.0, lo: -3.0, hi: 3.0, kp: 60, kd: 2.0}\n"
    "  - {motor_id: 2, direction: -1, offset: 0.0, lo: -3.0, hi: 3.0, kp: 80, kd: 3.0}  # inline comment\n"
)


def test_update_yaml_replaces_only_target_motor():
    new = update_yaml_limits(YAML, {2: (-0.95, 1.95)})
    assert "# top comment must be preserved" in new and "# inline comment" in new
    assert "motor_id: 1, direction: 1, offset: 0.0, lo: -3.0, hi: 3.0, kp: 60" in new
    assert "motor_id: 2, direction: -1, offset: 0.0, lo: -0.95, hi: 1.95, kp: 80" in new


def test_update_yaml_multiple_and_negative_values():
    new = update_yaml_limits(YAML, {1: (-2.7, 2.7), 2: (0.0, 3.05)})
    assert "motor_id: 1, direction: 1, offset: 0.0, lo: -2.7, hi: 2.7," in new
    assert "motor_id: 2, direction: -1, offset: 0.0, lo: 0.0, hi: 3.05," in new
    assert "kp: 60" in new and "kd: 3.0" in new  # other fields untouched
