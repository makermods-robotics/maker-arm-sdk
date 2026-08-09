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
        JointConfig(motor_id=1, direction=1, offset=0.0, lo=-3.0, hi=3.0, kp=60, kd=2.0),
        JointConfig(motor_id=2, direction=-1, offset=0.0, lo=-3.0, hi=3.0, kp=60, kd=2.0),
    ])
    a = Arm(cfg, SocketCanBackend("vcan0"))
    yield a
    a.estop()
    a.disconnect() if a.state is not ArmState.DISCONNECTED else None


def test_end_to_end_teleop_path(arm, fleet):
    arm.connect(timeout=2.0)
    arm.enable()                       # a real 200Hz thread
    assert arm.set_joint_targets([1.0, -0.5])
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        pos = arm.get_joint_positions()
        if abs(pos[0] - 1.0) < 0.01 and abs(pos[1] - (-0.5)) < 0.01:
            break
        time.sleep(0.05)
    pos = arm.get_joint_positions()
    assert pos[0] == pytest.approx(1.0, abs=0.01)
    assert pos[1] == pytest.approx(-0.5, abs=0.01)
    arm.disable()
    assert arm.state is ArmState.CONNECTED


def test_watchdog_faults_on_comm_loss(arm, fleet):
    arm.connect(timeout=2.0)
    arm.enable()
    fleet.paused = True                # disconnect
    deadline = time.monotonic() + 2.0
    while arm.state is not ArmState.FAULT and time.monotonic() < deadline:
        time.sleep(0.05)
    assert arm.state is ArmState.FAULT
    assert "timeout" in arm.fault_reason
