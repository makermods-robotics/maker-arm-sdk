import pytest

from maker_arm.cli.check import choose_test_delta


def test_choose_delta_prefers_larger_available_side():
    assert choose_test_delta(0.0, -1.0, 2.0, 0.05) == 0.05
    assert choose_test_delta(1.5, -2.0, 2.0, 0.05) == -0.05


def test_choose_delta_points_inward_when_start_is_just_outside_limit():
    assert choose_test_delta(1.02, -2.0, 0.98, 0.05) == -0.05
    assert choose_test_delta(-1.02, -0.98, 2.0, 0.05) == 0.05


def test_choose_delta_rejects_no_safe_room():
    with pytest.raises(ValueError):
        choose_test_delta(0.0, -0.01, 0.01, 0.05)
