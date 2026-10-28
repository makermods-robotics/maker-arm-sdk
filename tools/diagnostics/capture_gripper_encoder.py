"""Read-only gripper encoder capture over the MIT protocol.

Sends only a non-clearing fault query and, if the gripper reports no fault, twenty MIT state
requests to the gripper motor. Never enables, disables, zeros or moves anything. The capture
is refused if the gripper reports a fault, so fault evidence is never cleared.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

from maker_arm import protocol as p
from maker_arm.cli.common import add_backend_args, backend_kwargs
from maker_arm.config import ArmConfig
from maker_arm.errors import FaultLatched, ParamTimeout
from maker_arm.motor import Motor
from maker_arm.profiles import DEFAULT_ARM_CONFIG
from maker_arm.transport.base import CanBackend, create_backend

SAMPLES = 20


class ReadOnlyBackend(CanBackend):
    """Wraps a backend and refuses every frame except the gripper's fault query / state request."""

    def __init__(self, inner: CanBackend, motor_id: int):
        self._inner = inner
        self._allowed = {p.encode_fault_query(motor_id), p.encode_clear_fault(motor_id)}

    def open(self):
        self._inner.open()

    def close(self):
        self._inner.close()

    def set_recv_callback(self, cb):
        self._inner.set_recv_callback(cb)

    def send(self, can_id, data, extended=True):
        if extended or (can_id, bytes(data)) not in self._allowed:
            raise ValueError("only gripper fault queries and state requests are permitted")
        self._inner.send(can_id, data, extended=False)


def gripper_joint(config: ArmConfig):
    for j in config.joints:
        if j.name == "gripper":
            return j
    return config.joints[-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    add_backend_args(ap)
    ap.add_argument("--pose", required=True)
    ap.add_argument("--gap-mm", required=True, type=float)
    ap.add_argument("--output", required=True, type=Path)
    a = ap.parse_args()
    if not math.isfinite(a.gap_mm) or not 0 <= a.gap_mm <= 104.825:
        ap.error("Gap must be between 0 and 104.825 mm")
    if a.output.exists():
        ap.error("Output already exists; choose a new capture filename")
    config = ArmConfig.from_yaml(a.config)
    joint = gripper_joint(config)
    kw = backend_kwargs(a)
    if "port" in kw:
        try:
            occupied = subprocess.run(["lsof", "-t", kw["port"]], capture_output=True, text=True)
            if occupied.stdout.strip():
                ap.error("Adapter is in use by PID(s): " + occupied.stdout.strip())
        except FileNotFoundError:
            pass
    be = ReadOnlyBackend(create_backend(a.backend, **kw), joint.motor_id)
    motor = Motor(joint.motor_id, be, p.MOTOR_PARAMS[joint.model])
    raw_replies = []

    def receive(cid, data):
        if p.reply_motor_id(cid, data) != joint.motor_id:
            return
        msg = p.parse_frame(cid, data, motor.params, config.host_id)
        if isinstance(msg, (p.MotorFeedback, p.FaultStatus)):
            raw_replies.append({"utc": datetime.now(timezone.utc).isoformat(),
                                "kind": type(msg).__name__,
                                "can_id": hex(cid), "data_hex": bytes(data).hex()})
        motor.handle_frame(msg)

    be.set_recv_callback(receive)
    samples = []
    be.open()
    try:
        try:
            status = motor.query_fault(timeout=0.1, attempts=3)
        except ParamTimeout:
            raise SystemExit(f"gripper motor {joint.motor_id} did not answer the fault query")
        if status.has_fault:
            raise SystemExit(f"gripper motor {joint.motor_id} reports fault 0x{status.word:08X} "
                             f"({status.text}); not reading, fault kept for inspection")
        for _ in range(SAMPLES):
            try:
                fb = motor.read_state(timeout=0.1, attempts=3)
            except FaultLatched as e:
                raise SystemExit(f"gripper fault latched during capture: {e}")
            if not math.isfinite(fb.position):
                raise ValueError("Non-finite encoder reading")
            samples.append(fb.position)
            time.sleep(0.05)
    finally:
        be.close()
    median = statistics.median(samples)
    result = {"pose": a.pose, "gap_mm_user_reported": a.gap_mm,
              "motor_id": joint.motor_id,
              "motor_identity_basis": f"{Path(a.config).name}; confirm by gripper-only movement",
              "backend": a.backend, "backend_kwargs": {k: v for k, v in kw.items() if k in ("port", "channel")},
              "protocol": "MIT, standard 11-bit CAN, 1 Mbps",
              "source": "MIT feedback position", "units": "radians",
              "median_rad": median, "median_deg": math.degrees(median),
              "range_rad": [min(samples), max(samples)],
              "spread_rad": max(samples) - min(samples),
              "samples_rad": samples, "raw_replies": raw_replies,
              "motor_commands_sent": f"1+ non-clearing fault queries and {SAMPLES} state requests only; "
                                     "no enable, disable, zero, or motion commands"}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k not in ("samples_rad", "raw_replies")}, indent=2))


if __name__ == "__main__":
    main()
