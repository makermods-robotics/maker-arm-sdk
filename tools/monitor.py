#!/usr/bin/env python3
"""只读监视：CONNECTED 态 10Hz 轮询并刷屏打印各关节状态。手推关节核对方向用。"""

import argparse
import time

from maker_arm.arm import Arm
from maker_arm.errors import fault_text


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/maker_arm_6dof.yaml")
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    a = ap.parse_args()

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    print("已连接（未使能，可安全手推）。Ctrl-C 退出。")
    try:
        while True:
            arm.refresh()
            time.sleep(0.1)
            pos, vel = arm.get_joint_positions(), arm.get_joint_velocities()
            tmp, flt = arm.get_temperatures(), arm.get_faults()
            rows = [f"J{i+1}: {pos[i]:+7.3f} rad {vel[i]:+6.2f} rad/s {tmp[i]:5.1f}°C {fault_text(flt[i])}"
                    for i in range(arm.config.n_joints)]
            print("\x1b[2J\x1b[H" + "\n".join(rows), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
