#!/usr/bin/env python3
"""第三课：单关节小幅正弦跟踪。验证控制循环平滑度。⚠️ 先在台架/空旷处跑。"""

import math
import time

from _args import arm_from_args, make_parser


def _safe(fn):
    try:
        fn()
    except Exception as e:
        print(f"清理步骤失败（继续）: {e}")


def main():
    ap = make_parser(__doc__)
    ap.add_argument("--joint", type=int, default=6, help="1-based 关节号，默认末端关节")
    ap.add_argument("--amp", type=float, default=0.2, help="幅值 rad")
    ap.add_argument("--freq", type=float, default=0.2, help="频率 Hz")
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args()
    arm = arm_from_args(a)
    arm.connect()
    try:
        if not 1 <= a.joint <= arm.config.n_joints:
            raise SystemExit(f"--joint 必须在 1~{arm.config.n_joints}，得到 {a.joint}")
        input(f"即将使能并让 J{a.joint} 做 ±{a.amp} rad 正弦。确认后回车 > ")
        arm.enable()
        start = arm.get_joint_positions()
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.seconds and arm.state.name == "ENABLED":
            t = time.monotonic() - t0
            targets = list(start)
            targets[a.joint - 1] = start[a.joint - 1] + a.amp * math.sin(2 * math.pi * a.freq * t)
            arm.set_joint_targets(targets)
            time.sleep(0.02)   # 50Hz 设目标即可，200Hz 循环负责平滑
        if arm.state.name == "FAULT":
            print("FAULT:", arm.fault_reason)
    except KeyboardInterrupt:
        pass
    finally:
        _safe(arm.disable)
        _safe(arm.disconnect)
        print("已泄力退出")


if __name__ == "__main__":
    main()
