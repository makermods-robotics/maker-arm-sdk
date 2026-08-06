#!/usr/bin/env python3
"""最终目标：Star 102 leader 遥操 maker-arm。

退出路径统一泄力：Ctrl-C / leader 读数异常 / SDK FAULT → finally disable。
"""

import time

from _args import arm_from_args, make_parser
from maker_arm.mapping import JointMapper


def _safe(fn):
    try:
        fn()
    except Exception as e:
        print(f"清理步骤失败（继续）: {e}")


def main():
    ap = make_parser(__doc__)
    ap.add_argument("--star-port", required=True)
    ap.add_argument("--star-ids", default="0,1,2,3,4,5,6")
    ap.add_argument("--map", dest="map_path", default="configs/star_to_maker.json")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--sync-threshold", type=float, default=0.8, help="启动姿势差确认阈值 rad")
    a = ap.parse_args()
    star_ids = [int(x) for x in a.star_ids.split(",")]

    try:
        from motorbridge_smart_servo import FashionStarServo
    except ImportError:
        raise SystemExit("缺 motorbridge_smart_servo：conda run -n maker-arm pip install motorbridge-smart-servo（或见计划 Task 18 环境注意）")

    bus = FashionStarServo(a.star_port, baudrate=1_000_000)
    mapper = JointMapper.from_json(a.map_path)
    arm = arm_from_args(a)
    try:
        arm.connect()
        data = bus.sync_monitor(star_ids)
        raw = {i: data[i].angle_deg for i in star_ids if data.get(i) and data[i].reliable}
        if len(raw) != len(star_ids):
            raise SystemExit(f"leader 只读到 {sorted(raw)} —— 查 star 串口/ID")
        targets = mapper.map(raw)
        diff = max(abs(t - c) for t, c in zip(targets, arm.get_joint_positions()))
        print(f"启动姿势差 max={diff:.2f} rad（follower 将以限速平滑跟过去）")
        if diff > a.sync_threshold:
            input(f"⚠️ 姿势差超过 {a.sync_threshold} rad，确认周围无障碍后回车开始 > ")
        arm.enable()
        dt = 1.0 / a.rate
        print("遥操中，Ctrl-C 退出。")
        miss = 0
        while arm.state.name == "ENABLED":
            t0 = time.perf_counter()
            try:
                data = bus.sync_monitor(star_ids)
            except Exception as e:
                print(f"leader 读数异常，退出跟随: {e}")
                break
            raw = {i: data[i].angle_deg for i in star_ids if data.get(i) and data[i].reliable}
            if not raw:
                miss += 1
                if miss >= 50:
                    print("leader 读数丢失，退出跟随")
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
        _safe(arm.disable)
        _safe(arm.disconnect)
        _safe(bus.close)
        print("已泄力退出")


if __name__ == "__main__":
    main()
