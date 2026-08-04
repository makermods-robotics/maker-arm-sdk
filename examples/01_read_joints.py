#!/usr/bin/env python3
"""第一课：只读不动。确认通信、角度有数、手推方向符合直觉。"""

import time

from _args import arm_from_args, make_parser


def main():
    arm = arm_from_args(make_parser(__doc__).parse_args())
    arm.connect()
    print("已连接。手推关节观察角度变化，Ctrl-C 退出。")
    try:
        while True:
            arm.refresh()
            time.sleep(0.1)
            print(" ".join(f"J{i+1}={x:+7.3f}" for i, x in enumerate(arm.get_joint_positions())), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
