#!/usr/bin/env python3
"""双后端性能对比：RTT（参数读往返） + 持续吞吐（停止帧探测 @rate）。零运动风险。"""

import argparse
import csv
import statistics
import time

from maker_arm import protocol as p
from maker_arm.errors import ParamTimeout
from maker_arm.motor import Motor
from maker_arm.transport.base import create_backend


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", choices=["socketcan", "at"], required=True)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--rtt-n", type=int, default=500)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=200.0)
    ap.add_argument("--csv", default="bench_results.csv")
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    be = create_backend(a.backend, **kw)
    rx_count = [0]
    motors = {mid: Motor(mid, be) for mid in ids}

    def on_frame(cid, d):
        msg = p.parse_frame(cid, d)
        if msg is None:
            return
        if isinstance(msg, p.MotorFeedback):
            rx_count[0] += 1
        m = motors.get(getattr(msg, "motor_id", None))
        if m:
            m.handle_frame(msg)

    be.set_recv_callback(on_frame)
    be.open()
    try:
        # ── RTT ──
        rtts = []
        m0 = motors[ids[0]]
        for _ in range(a.rtt_n):
            t0 = time.perf_counter()
            try:
                m0.read_param(p.ParamIndex.VBUS, timeout=0.5)
                rtts.append((time.perf_counter() - t0) * 1000.0)
            except ParamTimeout:
                pass
            time.sleep(0.002)
        rtts.sort()
        p50 = statistics.median(rtts) if rtts else float("nan")
        p95 = rtts[int(len(rtts) * 0.95)] if rtts else float("nan")
        pmax = rtts[-1] if rtts else float("nan")
        print(f"RTT ms: p50={p50:.2f} p95={p95:.2f} max={pmax:.2f} (成功 {len(rtts)}/{a.rtt_n})")

        # ── 持续吞吐：rate Hz × len(ids) 探测帧 ──
        rx_count[0] = 0
        tx = overruns = 0
        dt = 1.0 / a.rate
        t_end = time.perf_counter() + a.seconds
        next_t = time.perf_counter()
        while time.perf_counter() < t_end:
            for mid in ids:
                be.send(*p.encode_disable(mid))
                tx += 1
            next_t += dt
            remain = next_t - time.perf_counter()
            if remain > 0:
                time.sleep(remain)
            else:
                overruns += 1
                next_t = time.perf_counter()
        time.sleep(0.2)
        rx = rx_count[0]
        print(f"吞吐: tx={tx} rx={rx} rx/tx={rx / max(tx, 1):.3f} tick超时={overruns}")

        with open(a.csv, "a", newline="") as f:
            w = csv.writer(f)
            if f.tell() == 0:
                w.writerow(["backend", "rtt_p50_ms", "rtt_p95_ms", "rtt_max_ms",
                            "tx", "rx", "rx_ratio", "overruns", "rate", "seconds"])
            w.writerow([a.backend, f"{p50:.3f}", f"{p95:.3f}", f"{pmax:.3f}",
                        tx, rx, f"{rx / max(tx, 1):.4f}", overruns, a.rate, a.seconds])
        print(f"已追加到 {a.csv}")
    finally:
        be.close()


if __name__ == "__main__":
    main()
