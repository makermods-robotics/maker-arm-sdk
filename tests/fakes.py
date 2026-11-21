"""Simulated RobStride MIT motors behind MockBackend (the package's _sim_motor), plus the shared
two-joint test arm.

Shared tests use only the generic surface: FakeMotor(motor_id, model=..., position=...,
temperature=..., enabled=..., online=..., track=..., faulted=..., warning=...), its ``log``
("status_query", "clear", "enable", "disable", "zero", ("cmd", kp)), ``frames`` ((position,
kp, tau) per command), ``last_kp`` and ``handle(can_id, data) -> replies``; FakeBus;
two_joint_config(); make_arm(); classify(); FAULT_CODE / FAULT_TEXT; FULL_TURN_SUPPORTED /
WARNING_SUPPORTED. Everything else is RobStride detail.
"""

from maker_arm._sim_motor import (FAULT_CODE, FAULT_TEXT, FULL_TURN_SUPPORTED, PREFIX,  # noqa: F401
                                  WARNING_SUPPORTED, classify, feedback_bytes, respond)
from maker_arm._sim_motor import SimBus as FakeBus
from maker_arm._sim_motor import SimMotor as FakeMotor
from maker_arm.arm import Arm
from maker_arm.config import ArmConfig, JointConfig


def two_joint_config(**overrides) -> ArmConfig:
    cfg = dict(control_rate_hz=200, max_velocity=50.0, joints=[
        JointConfig(motor_id=1, name="j1", lo=-2.0, hi=2.0, kp=60, kd=2.0),
        JointConfig(motor_id=2, name="gripper", lo=-1.6, hi=-0.04, kp=20, kd=0.5, can_timeout_ms=500,
                    max_temperature_c=55, enable_temperature_c=45, return_to_rest=False,
                    release_on_fault=True),
    ])
    cfg.update(overrides)
    return ArmConfig(**cfg)


def make_arm(motors=None, **overrides):
    """(arm, bus, {motor_id: FakeMotor}) for two_joint_config(**overrides); default motors sit at
    0.3 and -0.5 rad."""
    motors = motors or [FakeMotor(1, position=0.3), FakeMotor(2, position=-0.5)]
    bus = FakeBus(motors)
    arm = Arm(two_joint_config(**overrides), bus)
    return arm, bus, {m.motor_id: m for m in motors}
