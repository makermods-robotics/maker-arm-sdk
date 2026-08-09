#!/usr/bin/env python3
"""Star→maker 标定。两种模式，两臂全程不使能（手摆）：

默认（两姿势全标定）：摆两个对应姿势采样，解每关节 direction/scale/锚点。
--anchor-only（仅锚点）：两臂摆到对应零位采一次样，只更新 zero_deg/base_rad，
  保留映射文件里已实证的 direction/scale——绝对模式遥操（不带 --rebase）用它。
"""

import argparse
import json
import math
import time

from maker_arm.arm import Arm


def _safe(fn):
    try:
        fn()
    except Exception as e:
        print(f"清理步骤失败（继续）: {e}")


def read_star(bus, ids):
    data = bus.sync_monitor(ids)
    return {i: data[i].angle_deg for i in ids if data.get(i) and data[i].reliable}


def read_maker(arm):
    arm.refresh()
    time.sleep(0.2)
    return arm.get_joint_positions()


def validate_anchor_positions(positions, joints, grace=0.1):
    """Anchor sanity check: base_rad must fall within each joint's soft limits (small grace
    margin), otherwise it's treated as a bad reading and writing to disk is refused.

    Real-hardware incident: an anchor sampled during a 2π jump baked 6.36 into the file,
    and absolute-mode teleop lurched violently on startup.
    """
    bad = []
    for pos, j in zip(positions, joints):
        if pos < j.lo - grace or pos > j.hi + grace:
            bad.append(f"motor{j.motor_id}: base_rad {pos:+.3f} out of limits [{j.lo}, {j.hi}]"
                       " (suspected 2π jump, check the zero point before re-anchoring)")
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/maker_arm.yaml")
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--star-port", required=True)
    ap.add_argument("--star-ids", default="0,1,2,3,4,5,6")
    ap.add_argument("--out", default="configs/star_to_maker.json")
    ap.add_argument("--anchor-only", action="store_true",
                    help="只更新零位锚点（zero_deg/base_rad），direction/scale 原样保留")
    a = ap.parse_args()
    star_ids = [int(x) for x in a.star_ids.split(",")]

    from motorbridge_smart_servo import FashionStarServo
    bus = FashionStarServo(a.star_port, baudrate=1_000_000)
    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    try:
        arm.connect()
        if a.anchor_only:
            with open(a.out) as f:
                existing = json.load(f)
            by_servo = {j["servo"]: j for j in existing["joints"]}
            missing_cfg = [sid for sid in star_ids if sid not in by_servo]
            if missing_cfg:
                raise SystemExit(f"映射文件里没有 servo {missing_cfg} ——先跑一次全标定")
            input("两臂摆到对应零位姿势，回车采锚点 > ")
            star_z, maker_z = read_star(bus, star_ids), read_maker(arm)
            missing = [sid for sid in star_ids if sid not in star_z]
            if missing:
                raise SystemExit(f"servo {missing} 读数不可靠——重摆后重跑")
            bad = validate_anchor_positions(maker_z, arm.config.joints)
            if bad:
                raise SystemExit("锚点越限，拒绝写盘：\n  " + "\n  ".join(bad))
            for i, sid in enumerate(star_ids):
                by_servo[sid]["zero_deg"] = round(star_z[sid], 3)
                by_servo[sid]["base_rad"] = round(maker_z[i], 4)
                print(f"J{i+1} <- servo {sid}: zero_deg={by_servo[sid]['zero_deg']} base_rad={by_servo[sid]['base_rad']}")
            with open(a.out, "w") as f:
                json.dump(existing, f, indent=2, ensure_ascii=False)
            print(f"锚点已写入 {a.out}（direction/scale 未动）")
            return
        input("两臂摆到姿势 A（建议零位），回车采样 > ")
        star_a, maker_a = read_star(bus, star_ids), read_maker(arm)
        input("两臂摆到姿势 B（各关节尽量大位移），回车采样 > ")
        star_b, maker_b = read_star(bus, star_ids), read_maker(arm)

        missing = [sid for sid in star_ids if sid not in star_a or sid not in star_b]
        if missing:
            raise SystemExit(f"servo {missing} 读数不可靠——重摆姿势后重跑")

        joints = []
        for i, sid in enumerate(star_ids):
            d_star = math.radians(star_b[sid] - star_a[sid])
            d_maker = maker_b[i] - maker_a[i]
            if abs(d_star) < math.radians(5.0):
                raise SystemExit(f"servo {sid} 两姿势位移不足 5°，无法解 scale——重摆姿势 B")
            k = d_maker / d_star
            joints.append({"servo": sid, "zero_deg": star_a[sid], "base_rad": maker_a[i],
                           "direction": 1.0 if k >= 0 else -1.0, "scale": abs(k)})
            print(f"J{i+1} <- servo {sid}: direction={joints[-1]['direction']:+.0f} scale={abs(k):.3f}")
        with open(a.out, "w") as f:
            json.dump({"alpha": 0.3, "joints": joints}, f, indent=2, ensure_ascii=False)
        print(f"已写入 {a.out}")
    finally:
        _safe(arm.disconnect)
        _safe(bus.close)


if __name__ == "__main__":
    main()
