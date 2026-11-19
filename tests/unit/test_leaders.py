import math
import sys
import threading

import pytest

from chain_fixture import load
from teleop_fakes import NAMES, FakeArm, FakeServoBus, State, joint

leaders = load("leaders")
mapping = load("mapping")
StarLeader, ArmLeader, LeaderError = leaders.StarLeader, leaders.ArmLeader, leaders.LeaderError
Friction = load("leaders.arm").Friction

JOINTS = [
    {"name": "a", "servo": 0, "direction": 1.0, "range_deg": [-90, 90]},
    {"name": "b", "servo": 1, "direction": -2.0, "range_deg": [-120, 0]},
]


def star(bus, **kwargs):
    leader = StarLeader("/dev/fake", mapping.JointMapper(JOINTS), bus_factory=lambda port, baud: bus,
                        sleep=lambda s: None, **kwargs)
    leader.connect()
    return leader


# ── Star ──
def test_leader_protocol_is_met():
    bus = FakeServoBus({0: 0.0, 1: 0.0})
    assert isinstance(star(bus), leaders.Leader)
    assert isinstance(ArmLeader(FakeArm()), leaders.Leader)


def test_connect_pings_then_unlocks_then_resets_every_servo():
    bus = FakeServoBus({0: 0.0, 1: 0.0})
    star(bus)
    kinds = [c[0] for c in bus.calls]
    assert kinds == ["ping", "ping", "unlock", "unlock", "reset_multi_turn", "reset_multi_turn"]


def test_connect_refuses_a_silent_servo_and_closes_the_port():
    bus = FakeServoBus({0: 0.0, 1: 0.0}, dead={1})
    with pytest.raises(LeaderError, match=r"\[1\]"):
        star(bus)
    assert bus.closed


def test_read_maps_servo_degrees_to_follower_radians():
    leader = star(FakeServoBus({0: 30.0, 1: 20.0}))
    assert leader.read() == pytest.approx([math.radians(30.0), math.radians(-40.0)])


def test_a_missed_servo_keeps_its_last_target():
    bus = FakeServoBus({0: 30.0, 1: 20.0})
    leader = star(bus)
    first = leader.read()
    bus.silent, bus.angles[0] = {1}, 40.0
    assert leader.read() == pytest.approx([math.radians(40.0), first[1]])
    assert leader.misses == {0: 0, 1: 1}


def test_no_target_until_every_servo_has_replied_once():
    leader = star(FakeServoBus({0: 30.0, 1: 20.0}, silent={1}))
    assert leader.read() is None


def test_a_servo_lost_for_miss_limit_reads_ends_the_session():
    bus = FakeServoBus({0: 30.0, 1: 20.0})
    leader = star(bus, miss_limit=3)
    leader.read()
    bus.silent = {0}
    leader.read()
    leader.read()
    with pytest.raises(LeaderError, match=r"\[0\]"):
        leader.read()


def test_a_bus_error_is_a_miss_on_every_servo():
    bus = FakeServoBus({0: 30.0, 1: 20.0})
    leader = star(bus, miss_limit=2)
    leader.read()
    bus.raise_next = OSError("serial timeout")
    assert leader.read() is None
    bus.raise_next = OSError("serial timeout")
    with pytest.raises(LeaderError, match="serial timeout"):
        leader.read()


def test_a_reply_after_misses_resets_the_count():
    bus = FakeServoBus({0: 30.0, 1: 20.0})
    leader = star(bus, miss_limit=2)
    bus.silent = {1}
    leader.read()
    bus.silent = set()
    leader.read()
    bus.silent = {1}
    leader.read()  # one miss again, not two
    assert leader.misses[1] == 1


def test_zero_sets_every_origin_and_resets_the_turn_counters():
    bus = FakeServoBus({0: 30.0, 1: 20.0})
    leader = star(bus)
    leader.read()
    bus.calls.clear()
    leader.zero()
    assert [c for c in bus.calls if c[0] == "set_origin_point"] == [("set_origin_point", 0), ("set_origin_point", 1)]
    assert bus.calls[-2:] == [("reset_multi_turn", 0), ("reset_multi_turn", 1)]
    assert leader.read() == pytest.approx([0.0, 0.0])


def test_close_tolerates_a_vanished_port():
    bus = FakeServoBus({0: 0.0, 1: 0.0})
    leader = star(bus)
    bus.close = lambda: (_ for _ in ()).throw(OSError("device gone"))
    leader.close()
    assert leader.bus is None


def test_default_mapping_is_the_products_and_the_library_is_imported_lazily(monkeypatch):
    monkeypatch.setitem(sys.modules, "motorbridge_smart_servo", None)
    leader = StarLeader("/dev/fake")
    assert len(leader.servo_ids) == 7
    with pytest.raises(LeaderError, match="motorbridge-smart-servo"):
        leader.connect()


# ── Arm leader ──
def test_arm_leader_enables_then_floats():
    arm = FakeArm(state=State.DISCONNECTED)
    ArmLeader(arm).connect()
    assert arm.calls == ["connect", "enable", "float_mode"]
    assert arm.float_kwargs == {}


def test_arm_leader_reads_positions_clipped_to_the_follower():
    follower = FakeArm(joints=[joint(n, lo=-1.0, hi=1.0) for n in NAMES])
    arm = FakeArm(positions=[0.5, -1.5, 1.5, 0, 0, 0, 0.2], state=State.ENABLED)
    assert ArmLeader(arm, follower).read() == [0.5, -1.0, 1.0, 0, 0, 0, 0.2]


def test_arm_leader_has_no_reading_unless_enabled_and_finite():
    arm = FakeArm(state=State.CONNECTED)
    leader = ArmLeader(arm)
    assert leader.read() is None
    arm.state = State.ENABLED
    arm.positions[2] = math.nan
    assert leader.read() is None


def test_a_faulted_leader_arm_ends_the_session():
    arm = FakeArm(state=State.ENABLED)
    arm.fault("rotor overheat")
    with pytest.raises(LeaderError, match="rotor overheat"):
        ArmLeader(arm).read()


def test_arm_leader_close_holds_then_restores_the_profile_gains():
    arm = FakeArm(state=State.ENABLED)
    ArmLeader(arm).close()
    assert arm.calls == ["hold", ("set_gains", None, None)]   # no arguments: the profile's gains


def test_a_follower_of_another_size_is_refused():
    with pytest.raises(ValueError):
        ArmLeader(FakeArm(), FakeArm(joints=[joint("gripper")]))


# ── friction compensation ──
FRICTION = dict(friction_viscous=0.5, friction_coulomb=0.1, friction_rest_nm=0.06, friction_velocity_clamp=1.0)


def friction_arm():
    joints = [joint(n) for n in NAMES[:6]] + [joint("gripper", **FRICTION)]
    joints[1] = joint("shoulder_lift", friction_viscous=1.0)
    return FakeArm(joints=joints, state=State.ENABLED, friction_deadzone=0.05)


def test_friction_terms_come_from_the_profile_and_reach_float_mode():
    arm = friction_arm()
    leader = ArmLeader(arm)
    leader.connect()
    assert arm.float_kwargs["extra_torque"] == leader.friction.torques
    tau = leader.friction.torques([0.0] * 7, [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, -3.0])
    assert tau[1] == pytest.approx(0.2)                 # viscous only
    assert tau[6] == pytest.approx(0.5 * -1.0 - 0.1)    # clamped viscous + Coulomb, both along the motion
    assert tau[0] == 0.0


def test_friction_inside_the_deadzone_is_the_rest_torque_only():
    friction = Friction.from_config(friction_arm().config)
    tau = friction.torques([0.0] * 7, [0.01, 0.04, 0, 0, 0, 0, 0.049])
    assert tau == [0.0] * 6 + [0.06]


def test_friction_is_off_when_the_profile_has_none():
    assert Friction.from_config(FakeArm().config) is None
    assert Friction.from_config(friction_arm().config).torques([0] * 7, [math.nan] * 7) == [0.0] * 7


def test_arm_leader_passes_cancel_to_the_enable():
    arm = FakeArm(state=State.CONNECTED)
    cancel = threading.Event()
    ArmLeader(arm).connect(cancel=cancel)
    assert arm.cancel is cancel


def test_arm_leader_close_stiffens_a_real_arm_with_the_profile_gains():
    from fakes import make_arm
    arm, _, _ = make_arm()
    leader = ArmLeader(arm)
    leader.connect()
    assert [j.kp for j in arm.config.joints] == [0.0, 0.0]
    leader.close()
    assert [(j.kp, j.kd) for j in arm.config.joints] == [(60, 2.0), (20, 0.5)]
    arm.disable()
