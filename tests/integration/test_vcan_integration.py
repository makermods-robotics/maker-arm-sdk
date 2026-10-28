import os
import time

import pytest

from fake_motor import FakeMotorFleet
from maker_arm.arm import Arm, ArmState
from maker_arm.config import ArmConfig, JointConfig
from maker_arm.transport.socketcan import SocketCanBackend

VCAN_UP = os.path.exists("/sys/class/net/vcan0")
pytestmark = pytest.mark.skipif(not VCAN_UP, reason="vcan0 does not exist")


@pytest.fixture
def fleet():
    f = FakeMotorFleet("vcan0", [1, 2])
    f.start()
    yield f
    f.stop()


@pytest.fixture
def arm(fleet):
    cfg = ArmConfig(max_velocity=5.0, feedback_timeout=0.3, joints=[
        JointConfig(motor_id=1, lo=-3.0, hi=3.0, kp=60, kd=2.0),
        JointConfig(motor_id=2, direction=-1, lo=-3.0, hi=3.0, kp=60, kd=2.0),
    ])
    a = Arm(cfg, SocketCanBackend("vcan0"))
    yield a
    if a.state is not ArmState.DISCONNECTED:
        a.disconnect()


def wait_for(predicate, seconds=3.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not predicate():
        time.sleep(0.02)
    return predicate()


def test_end_to_end_control_path(arm, fleet):
    arm.connect()
    arm.enable()
    assert arm.set_joint_targets([1.0, -0.5])
    assert wait_for(lambda: arm.get_joint_positions() == pytest.approx([1.0, -0.5], abs=0.01))
    assert arm.stop() is True
    assert arm.state is ArmState.CONNECTED


def test_watchdog_faults_on_comm_loss(arm, fleet):
    arm.connect()
    arm.enable()
    fleet.paused = True
    assert wait_for(lambda: arm.state is ArmState.FAULT, 2.0)
    assert "timeout" in arm.fault_reason
