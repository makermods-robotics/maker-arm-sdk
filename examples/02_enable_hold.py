#!/usr/bin/env python3
"""第二课：使能并原地抱住 30 秒。首次上电试运行/调 kp kd 用。⚠️ 确保周围无障碍。"""

import time

from _args import arm_from_args, make_parser


def main():
    ap = make_parser(__doc__)
    ap.add_argument("--seconds", type=float, default=30.0)
    a = ap.parse_args()
    arm = arm_from_args(a)
    arm.connect()
    try:
        input("即将使能（原地抱住，不应有任何运动）。确认后回车 > ")
        arm.enable()
        t_end = time.monotonic() + a.seconds
        while time.monotonic() < t_end and arm.state.name == "ENABLED":
            time.sleep(0.2)
            print(" ".join(f"J{i+1}={x:+7.3f}" for i, x in enumerate(arm.get_joint_positions())), flush=True)
        if arm.state.name == "FAULT":
            print("FAULT:", arm.fault_reason)
    except KeyboardInterrupt:
        pass
    finally:
        arm.disable()
        arm.disconnect()
        print("已泄力退出")


if __name__ == "__main__":
    main()
