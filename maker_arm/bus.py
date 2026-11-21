"""The motors of one profile on their bus without an Arm, and a verified torque-off that works where
``Arm.connect`` refuses (a motor reporting a fault, a motor not answering).

    from <package>.bus import release
    result = release(spec.open_backend(), spec.config())   # opens and closes the bus
    result["ok"]          # every motor of the profile answered and confirmed torque off

The arm drops where it is not resting on something: support it first.
"""

from __future__ import annotations

import math
import time
from typing import Optional

from .config import ArmConfig, JointConfig
from .errors import ConnectError, FaultLatched, ParamTimeout
from .motor import Motor, route
from .transport.base import CanBackend

RELEASE_ATTEMPTS = 3
RELEASE_REPLY_S = 0.05


def joint_position(joint: JointConfig, motor_position: float) -> float:
    return (motor_position - joint.offset) * joint.direction


def release_motor(m: Motor) -> bool:
    """A zero-gain command at the measured position (a later enable cannot resume an old setpoint),
    then a disable whose reply must confirm torque off."""
    fb = m.feedback
    if fb is not None:
        m.send_command(fb.position, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.005)
    return m.disable_confirmed(RELEASE_REPLY_S, RELEASE_ATTEMPTS)


class Bus:
    """The motors of one profile on their bus, without an Arm. Reads never change torque; only
    release() does. Works when a motor reports a fault (Arm.connect refuses then)."""

    def __init__(self, backend: CanBackend, config: ArmConfig, ids: Optional[list[int]] = None):
        self.backend, self.config = backend, config
        joints = {j.motor_id: j for j in config.joints}
        ids = list(joints) if ids is None else ids
        self.joints = {i: joints.get(i) or JointConfig(motor_id=i, lo=-math.pi, hi=math.pi, kp=0.0, kd=0.0)
                       for i in ids}
        self.motors = {i: Motor(j, backend, config.host_id) for i, j in self.joints.items()}
        self.known = set(joints)

    def _receive(self, can_id, data) -> None:
        m = self.motors.get(route(can_id, data, self.config.host_id))
        if m is not None:
            m.handle_frame(can_id, data)

    def __enter__(self) -> "Bus":
        self.backend.set_recv_callback(self._receive)
        try:
            self.backend.open()
        except Exception as e:
            raise ConnectError(f"could not open the CAN adapter: {e}. Close other programs using it and retry.") from e
        return self

    def __exit__(self, *exc) -> None:
        self.backend.close()

    def read(self, motor_id: int, clear: bool = False, timeout: float = 0.1) -> Optional[dict]:
        """One motor's state in joint coordinates, or None when it does not answer."""
        m, j = self.motors[motor_id], self.joints[motor_id]
        try:
            status = m.check(timeout=timeout, attempts=2)
        except ParamTimeout:
            return None
        cleared = False
        if status.has_fault and clear:
            m.clear_fault()
            time.sleep(0.01)
            status = m.check(timeout=timeout, attempts=2)
            cleared = True
        fb = None
        if not status.has_fault:
            try:
                fb = m.read_state(timeout=timeout / 2, attempts=3)
            except (ParamTimeout, FaultLatched):
                pass
        fault = (status.text or f"code 0x{status.code:X}") if status.has_fault else m.fault_latched
        row = {"motor_id": motor_id, "joint": j.name if motor_id in self.known else None,
               "position": None, "velocity": None, "torque": None, "temperature": None, "temperatures": {},
               "enabled": None, "fault": fault, "fault_code": status.code if status.has_fault else 0,
               "cleared": cleared, "outside_limits": False}
        if fb is not None:
            q = joint_position(j, fb.position)
            row.update(position=q, velocity=fb.velocity * j.direction, torque=fb.torque * j.direction,
                       temperature=fb.temperature, temperatures=dict(fb.temperatures), enabled=fb.enabled)
            if motor_id in self.known:
                grace = self.config.full_turn_grace
                row["outside_limits"] = not any(j.lo - grace <= q + k <= j.hi + grace
                                                for k in (0.0, -2 * math.pi, 2 * math.pi))
        return row

    def read_all(self, clear: bool = False) -> list[dict]:
        rows = [self.read(i, clear) for i in self.motors]
        return [r for r in rows if r is not None]

    def missing(self, rows: list[dict]) -> list[int]:
        found = {r["motor_id"] for r in rows}
        return [i for i in self.motors if i in self.known and i not in found]

    def release(self, rows: Optional[list[dict]] = None) -> dict:
        """Torque off, verified, on every motor in ``rows`` (default: every motor that answers now).
        ``ok`` only when every motor of the profile answered and confirmed it; ``unconfirmed`` and
        ``missing`` name the others (joint names, motor ids where the joint is unknown)."""
        rows = self.read_all() if rows is None else rows
        unconfirmed = [r["joint"] or r["motor_id"] for r in rows if not release_motor(self.motors[r["motor_id"]])]
        missing = [self.joints[i].name for i in self.missing(rows)]
        released = [r["joint"] or r["motor_id"] for r in rows if (r["joint"] or r["motor_id"]) not in unconfirmed]
        return {"ok": bool(rows) and not unconfirmed and not missing, "released": released,
                "unconfirmed": unconfirmed, "missing": missing}


def release(backend: CanBackend, config: ArmConfig) -> dict:
    """Torque off, verified, motor by motor, on an arm this process has not connected (``backend``
    closed: opened and closed here): see Bus.release(). Raises ConnectError when the bus cannot be
    opened; a bus where no motor answers gives ``ok`` False with every joint ``missing``."""
    with Bus(backend, config) as bus:
        return bus.release()
