import math

import pytest

from maker_arm.gripper import CAPPED_CLOSING_KD, GripperHold

LIMITS = (math.radians(-96.0), math.radians(-2.5))


def test_free_closing_scales_kp_to_the_cap_and_lowers_kd():
    hold = GripperHold(0.5, LIMITS)
    sent = hold.update(goal=-0.05, position=-1.0, velocity=0.0, effort=0.0, kp=20.0, now=0.0)
    assert sent == pytest.approx(-0.05)
    kp, kd = hold.gains(20.0, 0.5)
    assert kp * (sent - -1.0) <= hold.closing_cap_nm + 1e-9
    assert kd == CAPPED_CLOSING_KD and hold.closing_capped


def test_opening_uses_full_gains():
    hold = GripperHold(0.5, LIMITS)
    hold.update(goal=-1.0, position=-0.1, velocity=0.0, effort=0.0, kp=20.0, now=0.0)
    assert hold.gains(20.0, 0.5) == (20.0, 0.5)


def test_contact_switches_to_holding_regulation():
    hold = GripperHold(0.5, LIMITS)
    t = 0.0
    for _ in range(10):   # blocked: closing effort 0.8 Nm, not moving, for > 100 ms
        sent = hold.update(goal=-0.05, position=-0.5, velocity=0.0, effort=0.8, kp=20.0, now=t)
        t += 0.02
    assert hold.holding
    # Effort above target: the command relaxes back toward the measured position.
    assert sent < -0.05


def test_opening_releases_the_hold():
    hold = GripperHold(0.5, LIMITS)
    t = 0.0
    for _ in range(10):
        hold.update(goal=-0.05, position=-0.5, velocity=0.0, effort=0.8, kp=20.0, now=t)
        t += 0.02
    hold.update(goal=-1.0, position=-0.5, velocity=0.0, effort=0.8, kp=20.0, now=t)
    assert not hold.holding


@pytest.mark.parametrize("torque", [0.0, 3.0, math.nan])
def test_torque_range(torque):
    with pytest.raises(ValueError):
        GripperHold(torque, LIMITS)
