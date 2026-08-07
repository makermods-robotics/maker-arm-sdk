#!/usr/bin/env python3
"""私有协议态：把电机侧 CAN 超时看门狗固化进闪存（默认 200ms→4000 计数）。

切到 MIT 协议前必跑一次——MIT 模式没有参数表访问，这是电机断链自动泄力的唯一保障。
"""

import argparse
import time

from maker_arm import protocol as p
from maker_arm.motor import Motor
from maker_arm.transport.base import create_backend


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--ids", default="1,2,3,4,5,6,7")
    ap.add_argument("--timeout-ms", type=int, default=200)
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]
    counts = a.timeout_ms * p.CAN_TIMEOUT_PER_MS

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    be = create_backend(a.backend, **kw)
    motors = {mid: Motor(mid, be) for mid in ids}
    be.set_recv_callback(lambda cid, d: (
        (msg := p.parse_frame(cid, d)) and motors.get(getattr(msg, "motor_id", None))
        and motors[msg.motor_id].handle_frame(msg)))
    be.open()
    ok, fail = [], []
    try:
        for mid, m in motors.items():
            try:
                m.write_param(p.ParamIndex.CAN_TIMEOUT, counts, "u32")
                time.sleep(0.05)
                rb = m.read_param(p.ParamIndex.CAN_TIMEOUT, dtype="u32")
                if rb != counts:
                    raise RuntimeError(f"回读 {rb} ≠ {counts}")
                m.save_params()
                time.sleep(0.1)
                ok.append(mid)
                print(f"电机{mid}: canTimeout={counts} 计数(={a.timeout_ms}ms) 已写入并掉电保存")
            except Exception as e:
                fail.append(mid)
                print(f"电机{mid}: ❌ {e}")
    finally:
        be.close()
    print(f"完成：成功 {ok}，失败 {fail}")
    raise SystemExit(1 if fail else 0)


if __name__ == "__main__":
    main()
