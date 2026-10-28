#!/usr/bin/env python3
"""Star -> Maker mapping calibration (LeRobot-style ``{servo, direction, range_deg}``), both arms unpowered.

The mapping has no offset: ``follower_deg = unwrap(leader_raw_deg) * direction``. It is only
valid when both arms read zero in the same pose (folded against the base, gripper fully
closed). This tool therefore

1. checks the zero pose: every Star servo must read ~0 deg and every follower joint must be
   inside its soft limits (both arms zeroed in the same pose);
2. measures ``direction`` (sign x scale) per joint from a second, matching pose:
   ``direction = follower_deg / leader_deg`` relative to the zero pose.

``range_deg`` is kept from the mapping file unless ``--range-from-profile`` is given, which
writes the follower profile's limits truncated to integer degrees (as LeRobot's
MakerFollowerConfig.joint_limits). The follower torque stays released throughout.
"""

import argparse
import json
import math
import time

from maker_arm.cli.common import add_backend_args, arm_from_args
from maker_arm.mapping import JointMapper
from maker_arm.profiles import DEFAULT_ARM_CONFIG

ZERO_TOLERANCE_DEG = 5.0      # leader reading at the zero pose
MIN_LEADER_TRAVEL_DEG = 15.0  # second pose must move each joint at least this far


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
    """Follower zero-pose sanity check: each joint must be inside its soft limits (small grace
    margin), otherwise the reading is treated as bad and nothing is written.

    Real-hardware incident: a pose sampled during a 2π jump read 6.36 rad, and the resulting
    calibration made teleop lurch violently on startup.
    """
    bad = []
    for pos, j in zip(positions, joints):
        if pos < j.lo - grace or pos > j.hi + grace:
            bad.append(f"motor{j.motor_id}: position {pos:+.3f} out of limits [{j.lo}, {j.hi}]"
                       " (suspected 2π jump or stale zero; re-zero with 'maker-arm zero' first)")
    return bad


def validate_leader_zero(star_deg, servo_ids, tolerance=ZERO_TOLERANCE_DEG):
    """Leader zero-pose check: the offset-free mapping needs every servo near 0 deg."""
    bad = []
    for sid in servo_ids:
        value = JointMapper.unwrap(star_deg[sid], -180.0, 180.0)
        if abs(value) > tolerance:
            bad.append(f"servo {sid}: reads {value:+.1f} deg at the zero pose (|x| <= {tolerance} required)")
    return bad


def measure_direction(leader0, leader1, follower0_rad, follower1_rad,
                      min_travel=MIN_LEADER_TRAVEL_DEG):
    """direction = follower travel / leader travel (degrees per degree), or None if too little travel."""
    d_leader = leader1 - leader0
    if not math.isfinite(d_leader) or abs(d_leader) < min_travel:
        return None
    return math.degrees(follower1_rad - follower0_rad) / d_leader


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(DEFAULT_ARM_CONFIG))
    add_backend_args(ap)
    ap.add_argument("--star-port", required=True)
    ap.add_argument("--out", required=True,
                    help="mapping JSON to modify; no production profile is selected implicitly")
    ap.add_argument("--range-from-profile", action="store_true",
                    help="also set range_deg from the follower profile limits (integer degrees)")
    a = ap.parse_args()

    with open(a.out) as f:
        existing = json.load(f)
    entries = existing["joints"]
    from motorbridge_smart_servo import FashionStarServo
    bus = FashionStarServo(a.star_port, baudrate=1_000_000)
    arm = arm_from_args(a)
    try:
        if len(entries) != arm.config.n_joints:
            raise SystemExit(f"mapping has {len(entries)} joints, profile has {arm.config.n_joints}")
        servo_ids = [e["servo"] for e in entries]
        arm.connect()

        input("Move BOTH arms to the zero pose (folded against the base, gripper closed), ENTER > ")
        star0, maker0 = read_star(bus, servo_ids), read_maker(arm)
        missing = [sid for sid in servo_ids if sid not in star0]
        if missing:
            raise SystemExit(f"servo {missing} readings unreliable — reposition and rerun")
        bad = (validate_anchor_positions(maker0, arm.config.joints)
               + validate_leader_zero(star0, servo_ids)
               + [f"{j.label}: follower reads {math.degrees(q):+.1f} deg at the zero pose "
                  f"(|x| <= {ZERO_TOLERANCE_DEG} required; re-zero with 'maker-arm zero')"
                  for q, j in zip(maker0, arm.config.joints)
                  if abs(math.degrees(q)) > ZERO_TOLERANCE_DEG])
        if bad:
            raise SystemExit("zero pose check failed, nothing written:\n  " + "\n  ".join(bad))

        input("Move BOTH arms to a second matching pose, every joint well away from zero, ENTER > ")
        star1, maker1 = read_star(bus, servo_ids), read_maker(arm)
        missing = [sid for sid in servo_ids if sid not in star1]
        if missing:
            raise SystemExit(f"servo {missing} readings unreliable — reposition and rerun")

        for i, (entry, joint) in enumerate(zip(entries, arm.config.joints)):
            sid = entry["servo"]
            direction = measure_direction(star0[sid], star1[sid], maker0[i], maker1[i])
            label = entry.get("name") or joint.label
            if direction is None:
                print(f"{label} (servo {sid}): leader moved < {MIN_LEADER_TRAVEL_DEG} deg, "
                      f"direction kept at {entry['direction']}")
            else:
                entry["direction"] = round(direction, 6)
                print(f"{label} (servo {sid}): direction = {entry['direction']}")
            if a.range_from_profile:
                entry["range_deg"] = [int(math.degrees(joint.lo)), int(math.degrees(joint.hi))]
                print(f"  range_deg = {entry['range_deg']}")
        JointMapper(entries)   # validates direction != 0 and lo < hi before writing
        with open(a.out, "w") as f:
            json.dump(existing, f, indent=2, ensure_ascii=False)
            f.write("\n")
        print(f"mapping written to {a.out}")
    finally:
        _safe(arm.disconnect)
        _safe(bus.close)


if __name__ == "__main__":
    main()
