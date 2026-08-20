#!/usr/bin/env python3
"""Persistent Star→Maker zero-pose calibration, with both arms unpowered.

Move both arms to the same defined zero/reference pose and sample once. This updates only the
absolute zero_deg/base_rad anchors; the measured direction and scale for each joint remain
fixed in the mapping file, matching the Metal arm's zero-only calibration model.
"""

import argparse
import json
import time

from maker_arm.arm import Arm
from maker_arm.profiles import DEFAULT_ARM_CONFIG


def _safe(fn):
    try:
        fn()
    except Exception as e:
        print(f"cleanup step failed (continuing): {e}")


def read_star(bus, ids, timeout=2.0):
    values = {}
    deadline = time.monotonic() + timeout
    while len(values) != len(ids) and time.monotonic() < deadline:
        data = bus.sync_monitor(ids)
        values.update({i: data[i].angle_deg for i in ids
                       if data.get(i) and data[i].reliable})
    return values


def read_maker(arm):
    fresh = arm.refresh(wait=True, timeout=0.05)
    if not all(fresh):
        stale = [arm.config.joints[i].motor_id for i, ok in enumerate(fresh) if not ok]
        raise SystemExit(f"fresh follower feedback unavailable for CAN motors {stale}")
    return arm.get_joint_positions()


def validate_anchor_positions(positions, joints, grace=0.1):
    """Anchor sanity check: base_rad must fall within each joint's soft limits (small grace
    margin), otherwise it's treated as a bad reading and writing to disk is refused.

    Real-hardware incident: an anchor sampled during a 2π jump baked 6.36 into the file,
    and teleop lurched violently on startup.
    """
    bad = []
    for pos, j in zip(positions, joints):
        if pos < j.lo - grace or pos > j.hi + grace:
            bad.append(f"motor{j.motor_id}: base_rad {pos:+.3f} out of limits [{j.lo}, {j.hi}]"
                       " (suspected 2π jump, check the zero point before re-anchoring)")
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--serial-baudrate", type=int, default=115200,
                    help="SLCAN serial rate (default: 115200)")
    ap.add_argument("--slcan-startup-delay", type=float, default=2.5)
    ap.add_argument("--star-port", required=True)
    ap.add_argument("--star-ids", default="0,1,2,3,4,5,6")
    ap.add_argument("--out", required=True,
                    help="mapping JSON to modify; no production profile is selected implicitly")
    a = ap.parse_args()
    star_ids = [int(x) for x in a.star_ids.split(",")]

    from motorbridge_smart_servo import FashionStarServo
    bus = FashionStarServo(a.star_port, baudrate=1_000_000)
    if a.backend == "socketcan":
        kw = {"channel": a.channel}
    elif a.backend == "slcan":
        kw = {"port": a.port, "baudrate": a.serial_baudrate,
              "rtscts": False, "startup_delay": a.slcan_startup_delay}
    else:
        kw = {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    try:
        arm.connect()
        with open(a.out) as f:
            existing = json.load(f)
        by_servo = {j["servo"]: j for j in existing["joints"]}
        missing_cfg = [sid for sid in star_ids if sid not in by_servo]
        if missing_cfg:
            raise SystemExit(f"mapping file has no servo {missing_cfg}")
        input("Move both arms to the corresponding zero pose, press ENTER to calibrate > ")
        star_z, maker_z = read_star(bus, star_ids), read_maker(arm)
        missing = [sid for sid in star_ids if sid not in star_z]
        if missing:
            raise SystemExit(f"servo {missing} readings unreliable — reposition and rerun")
        bad = validate_anchor_positions(maker_z, arm.config.joints)
        if bad:
            raise SystemExit("zero pose out of limits, refusing to write to disk:\n  " + "\n  ".join(bad))
        for i, sid in enumerate(star_ids):
            by_servo[sid]["zero_deg"] = round(star_z[sid], 3)
            by_servo[sid]["base_rad"] = round(maker_z[i], 4)
            print(
                f"J{i + 1} <- servo {sid}: zero_deg={by_servo[sid]['zero_deg']} "
                f"base_rad={by_servo[sid]['base_rad']}"
            )
        with open(a.out, "w") as f:
            json.dump(existing, f, indent=2, ensure_ascii=False)
        print(f"absolute zero-pose calibration written to {a.out} (direction/scale unchanged)")
    finally:
        _safe(arm.disconnect)
        _safe(bus.close)


if __name__ == "__main__":
    main()
