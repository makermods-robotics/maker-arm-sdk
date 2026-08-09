#!/usr/bin/env python3
"""End goal: teleoperate maker-arm from a Star 102 leader.

All exit paths release torque uniformly: Ctrl-C / leader read error / SDK FAULT -> finally disable.
"""

import time

from _args import arm_from_args, make_parser
from maker_arm.mapping import JointMapper


def _safe(fn):
    try:
        fn()
    except Exception as e:
        print(f"cleanup step failed (continuing): {e}")


def main():
    ap = make_parser(__doc__)
    ap.add_argument("--star-port", required=True)
    ap.add_argument("--star-ids", default="0,1,2,3,4,5,6")
    ap.add_argument("--map", dest="map_path", default="configs/star_to_maker.json")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--sync-threshold", type=float, default=0.8, help="startup pose-difference confirmation threshold, rad")
    ap.add_argument("--rebase", action="store_true",
                    help="on startup, reset the anchor to both sides' current pose (relative mode, zero jump at startup; recommended for bench/demo)")
    a = ap.parse_args()
    star_ids = [int(x) for x in a.star_ids.split(",")]

    try:
        from motorbridge_smart_servo import FashionStarServo
    except ImportError:
        raise SystemExit("missing motorbridge_smart_servo: conda run -n maker-arm pip install motorbridge-smart-servo (or see the plan's Task 18 environment notes)")

    bus = FashionStarServo(a.star_port, baudrate=1_000_000)
    mapper = JointMapper.from_json(a.map_path)
    arm = arm_from_args(a)
    try:
        arm.connect()
        data = bus.sync_monitor(star_ids)
        raw = {i: data[i].angle_deg for i in star_ids if data.get(i) and data[i].reliable}
        if len(raw) != len(star_ids):
            raise SystemExit(f"leader only read {sorted(raw)} -- check the star serial port/IDs")
        if a.rebase:
            arm.refresh()
            time.sleep(0.2)
            mapper.rebase(raw, arm.get_joint_positions())
            print("anchor reset to the current pose (relative mode, zero jump at startup)")
        targets = mapper.map(raw)
        diff = max(abs(t - c) for t, c in zip(targets, arm.get_joint_positions()))
        print(f"startup pose difference max={diff:.2f} rad (the follower will ramp up to it at the rate limit)")
        if diff > a.sync_threshold:
            input(f"⚠️ pose difference exceeds {a.sync_threshold} rad, confirm the area is clear and press ENTER to start > ")
        arm.enable()
        dt = 1.0 / a.rate
        print("teleoperating, Ctrl-C to exit.")
        miss = 0
        while arm.state.name == "ENABLED":
            t0 = time.perf_counter()
            try:
                data = bus.sync_monitor(star_ids)
            except Exception as e:
                print(f"leader read error, stopping follow: {e}")
                break
            raw = {i: data[i].angle_deg for i in star_ids if data.get(i) and data[i].reliable}
            if not raw:
                miss += 1
                if miss >= 50:
                    print("leader readings lost, stopping follow")
                    break
            else:
                miss = 0
            arm.set_joint_targets(mapper.map(raw))
            remain = dt - (time.perf_counter() - t0)
            if remain > 0:
                time.sleep(remain)
        if arm.state.name == "FAULT":
            print("FAULT:", arm.fault_reason)
    except KeyboardInterrupt:
        pass
    finally:
        if arm.state.name in ("ENABLED", "FAULT"):
            print("\n🔒 arm is holding position -- steady it or move it to a safe pose, then press ENTER to release torque (Ctrl-C again also forces release)...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass
        _safe(arm.disable)
        _safe(arm.disconnect)
        _safe(bus.close)
        print("torque released, exiting")


if __name__ == "__main__":
    main()
