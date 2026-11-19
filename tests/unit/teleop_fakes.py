"""Stand-ins for the teleop tests: an object exposing only the Arm API the leaders and the loop use,
a FashionStar servo bus, and a scripted leader."""

import math
from enum import Enum
from types import SimpleNamespace

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]


class State(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ENABLED = "enabled"
    FAULT = "fault"


def joint(name, lo=-2.0, hi=2.0, **extra):
    return SimpleNamespace(name=name, lo=lo, hi=hi, kp=30.0, kd=2.0, motor_id=NAMES.index(name) + 1, **extra)


class FakeArm:
    """Arm API by duck typing. ``track=True`` makes the measured pose follow every accepted target."""

    def __init__(self, positions=None, joints=None, track=True, state=State.CONNECTED, **config):
        self.config = SimpleNamespace(joints=joints or [joint(n) for n in NAMES], **config)
        self.positions = list(positions if positions is not None else [0.0] * len(self.config.joints))
        self.velocities = [0.0] * len(self.positions)
        self.state = state
        self.track = track
        self.targets = []
        self.calls = []
        self.fault_reason = None
        self.float_kwargs = None

    def connect(self):
        self.calls.append("connect")
        self.state = State.CONNECTED

    def enable(self, cancel=None):
        self.calls.append("enable")
        self.cancel = cancel
        self.state = State.ENABLED

    def float_mode(self, **kwargs):
        self.calls.append("float_mode")
        self.float_kwargs = kwargs

    def set_gains(self, kp=None, kd=None):
        self.calls.append(("set_gains", kp, kd))

    def get_joint_positions(self):
        return list(self.positions)

    def get_joint_velocities(self):
        return list(self.velocities)

    def set_joint_targets(self, targets):
        if self.state is not State.ENABLED or len(targets) != len(self.positions) \
                or not all(math.isfinite(t) for t in targets):
            return False
        self.targets.append(list(targets))
        if self.track:
            self.positions = list(targets)
        return True

    def hold_current_position(self):
        self.calls.append("hold")
        return self.state in (State.ENABLED, State.FAULT)

    def fault(self, reason="overcurrent"):
        self.state, self.fault_reason = State.FAULT, reason


class Monitor(SimpleNamespace):
    pass


class FakeServoBus:
    """FashionStarServo by duck typing: ``angles`` is what each servo reports; ``silent`` never reply."""

    def __init__(self, angles, silent=(), dead=()):
        self.angles = dict(angles)
        self.silent = set(silent)      # never answer sync_monitor
        self.dead = set(dead)          # never answer ping
        self.calls = []
        self.closed = False
        self.raise_next = None

    def ping(self, sid):
        self.calls.append(("ping", sid))
        return sid not in self.dead

    def unlock(self, sid):
        self.calls.append(("unlock", sid))

    def reset_multi_turn(self, sid):
        self.calls.append(("reset_multi_turn", sid))

    def set_origin_point(self, sid):
        self.calls.append(("set_origin_point", sid))
        self.angles[sid] = 0.0

    def sync_monitor(self, ids):
        if self.raise_next is not None:
            error, self.raise_next = self.raise_next, None
            raise error
        return {i: (None if i in self.silent else Monitor(angle_deg=self.angles[i])) for i in ids}

    def close(self):
        self.closed = True


class ScriptedLeader:
    """Returns ``script`` items in order (an Exception item is raised), then repeats the last one."""

    name = "scripted"

    def __init__(self, script):
        self.script = list(script)
        self.reads = 0

    def connect(self, cancel=None):
        pass

    def read(self):
        self.reads += 1
        item = self.script.pop(0) if len(self.script) > 1 else self.script[0]
        if isinstance(item, BaseException):
            raise item
        return None if item is None else list(item)

    def close(self):
        pass


class FakeClock:
    """Monotonic time that only moves when the loop sleeps (each read also costs ``tick``)."""

    def __init__(self, start=100.0, tick=0.0):
        self.now = start
        self.tick = tick

    def __call__(self):
        self.now += self.tick
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, seconds)
