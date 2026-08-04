#!/usr/bin/env python3
"""Star→maker 两姿势标定：两臂各摆到两个对应姿势采样，解每关节线性映射。

姿势 A 建议取零位；姿势 B 各关节尽量大位移。两臂全程不使能（手摆）。
"""

import argparse
import json
import math
import time

from maker_arm.arm import Arm


def read_star(bus, ids):
    data = bus.sync_monitor(ids)
    return {i: data[i].angle_deg for i in ids if data.get(i) and data[i].reliable}


def read_maker(arm):
    arm.refresh()
    time.sleep(0.2)
    return arm.get_joint_positions()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/maker_arm_6dof.yaml")
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--star-port", required=True)
    ap.add_argument("--star-ids", default="0,1,2,3,4,5")
    ap.add_argument("--out", default="configs/star_to_maker.json")
    a = ap.parse_args()
    star_ids = [int(x) for x in a.star_ids.split(",")]

    from motorbridge_smart_servo import FashionStarServo
    bus = FashionStarServo(a.star_port, baudrate=1_000_000)
    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    try:
        input("两臂摆到姿势 A（建议零位），回车采样 > ")
        star_a, maker_a = read_star(bus, star_ids), read_maker(arm)
        input("两臂摆到姿势 B（各关节尽量大位移），回车采样 > ")
        star_b, maker_b = read_star(bus, star_ids), read_maker(arm)

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
        arm.disconnect()
        bus.close()


if __name__ == "__main__":
    main()
