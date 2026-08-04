#!/usr/bin/env python3
"""扫总线：对 ID 范围逐个发停止帧探测，打印在线电机的位置/温度/故障。"""

import argparse
import time

from maker_arm import protocol as p
from maker_arm.errors import fault_text
from maker_arm.transport.base import create_backend


def backend_from_args(a):
    if a.backend == "socketcan":
        return create_backend("socketcan", channel=a.channel)
    return create_backend("at", port=a.port)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--max-id", type=int, default=14)
    a = ap.parse_args()

    found = {}
    be = backend_from_args(a)
    be.set_recv_callback(lambda cid, d: (
        (msg := p.parse_frame(cid, d)) and isinstance(msg, p.MotorFeedback)
        and found.__setitem__(msg.motor_id, msg)))
    be.open()
    try:
        for mid in range(1, a.max_id + 1):
            for _ in range(3):
                be.send(*p.encode_disable(mid))
                time.sleep(0.02)
        time.sleep(0.2)
    finally:
        be.close()

    if not found:
        print("未发现任何电机——查电源/接线/终端电阻/波特率(1M)")
        return
    print(f"{'ID':>3} {'位置rad':>10} {'温度°C':>8} 故障")
    for mid in sorted(found):
        fb = found[mid]
        print(f"{mid:>3} {fb.position:>10.3f} {fb.temperature:>8.1f} {fault_text(fb.fault_bits)}")


if __name__ == "__main__":
    main()
