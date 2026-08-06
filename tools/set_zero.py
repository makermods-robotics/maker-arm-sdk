#!/usr/bin/env python3
"""零位标定：把臂手动摆到零位姿态后运行。逐电机设零 + 掉电保存。"""

import argparse
import time

from maker_arm.arm import Arm


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/maker_arm01.yaml")
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    a = ap.parse_args()

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    try:
        print("当前电机坐标:", [f"{x:+.3f}" for x in arm.get_joint_positions()])
        input("确认臂已摆到零位姿态且各电机未使能，回车开始设零（Ctrl-C 取消）> ")
        for m in arm.motors:
            m.set_zero()
            time.sleep(0.05)
            m.save_params()
            time.sleep(0.05)
            print(f"电机 {m.motor_id}: 已设零并保存")
        time.sleep(0.2)
        arm.refresh()
        time.sleep(0.2)
        print("设零后位置（应≈0）:", [f"{x:+.3f}" for x in arm.get_joint_positions()])
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
