#!/usr/bin/env python3
"""Find MIT motors on the follower bus and show position, temperature and fault status.

Never enables or releases torque. Each ID first gets a non-clearing fault query; only
fault-free motors are then asked for their position, so fault evidence is kept unless you
pass --clear-faults.
"""

import argparse
import math

from maker_arm import protocol
from maker_arm.cli.common import add_backend_args, backend_from_args
from maker_arm.config import ArmConfig
from maker_arm.errors import ParamTimeout
from maker_arm.motor import Motor
from maker_arm.profiles import DEFAULT_ARM_CONFIG


def open_backend(be) -> None:
    try:
        be.open()
    except Exception as e:
        raise SystemExit(f"could not open the CAN adapter: {e}\n"
                         "Close other programs using it (for example MakerModsLab) and retry.") from e


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    add_backend_args(ap)
    ap.add_argument("--max-id", type=int, default=7)
    ap.add_argument("--clear-faults", action="store_true",
                    help="clear faults on motors that report one (only once the cause is fixed)")
    a = ap.parse_args()
    if not 1 <= a.max_id <= 127:
        ap.error("--max-id must be in [1, 127]")
    config = ArmConfig.from_yaml(a.config)
    joints = {j.motor_id: j for j in config.joints}

    be = backend_from_args(a)
    motors = {mid: Motor(mid, be, protocol.MOTOR_PARAMS[joints[mid].model] if mid in joints else protocol.RS00)
              for mid in range(1, a.max_id + 1)}

    def receive(can_id, data):
        m = motors.get(protocol.reply_motor_id(can_id, data))
        if m:
            m.handle_frame(protocol.parse_frame(can_id, data, m.params, config.host_id))

    be.set_recv_callback(receive)
    open_backend(be)
    rows = []
    try:
        for mid, m in motors.items():
            try:
                status = m.query_fault(timeout=0.1, attempts=2)
            except ParamTimeout:
                continue
            cleared = False
            if status.has_fault and a.clear_faults:
                m.clear_fault()
                status = m.query_fault(timeout=0.1, attempts=2)
                cleared = True
            fb = None
            if not status.has_fault:
                try:
                    fb = m.read_state(timeout=0.05, attempts=3)
                except ParamTimeout:
                    pass
            rows.append((mid, status, fb, cleared, m.fault_latched))
    finally:
        be.close()

    if not rows:
        print("no motors found -- check power, wiring, termination, 1 Mbps, that the motors use the MIT "
              "protocol, and that no other program (for example MakerModsLab) holds the adapter")
        return
    print(" ID joint          position_deg  temp_C  mode  status")
    for mid, status, fb, cleared, latched in rows:
        j = joints.get(mid)
        name = j.label if j else "(not in profile)"
        if fb is None:
            pos = temp = mode = "-"
        else:
            pos = f"{math.degrees(fb.position):+12.1f}"
            temp = f"{fb.temperature:6.1f}"
            mode = str(fb.mode)
        if status.has_fault:
            text = f"FAULT 0x{status.word:08X}: {status.text}" + (" (still present after clear)" if cleared else "")
        elif latched:
            text = f"FAULT: {latched}"
        else:
            text = "ok (fault cleared)" if cleared else "ok"
            if j and fb and not j.lo - config.full_turn_grace <= fb.position <= j.hi + config.full_turn_grace \
                    and not any(j.lo - config.full_turn_grace <= fb.position + k <= j.hi + config.full_turn_grace
                                for k in (-2 * math.pi, 2 * math.pi)):
                text += " -- outside limits, re-zero needed"
        print(f"{mid:3} {name:<14} {pos:>12}  {temp:>6}  {mode:>4}  {text}")
    missing = [mid for mid in joints if mid <= a.max_id and mid not in {r[0] for r in rows}]
    if missing:
        print(f"missing from profile: {missing}")


if __name__ == "__main__":
    main()
