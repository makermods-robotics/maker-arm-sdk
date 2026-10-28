#!/usr/bin/env python3
"""SocketCAN/SLCAN performance comparison: RTT plus sustained torque-free probe throughput.

Uses only the MIT non-clearing fault query (FF x6 00 FB), which neither changes torque state
nor clears fault evidence, so it is safe on a powered arm.
"""

import argparse
import csv
import statistics
import time

from maker_arm import protocol as p
from maker_arm.cli.common import add_backend_args, backend_from_args
from maker_arm.errors import ParamTimeout
from maker_arm.motor import Motor


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    add_backend_args(ap)
    ap.add_argument("--ids", default="1,2,3,4,5,6,7")
    ap.add_argument("--rtt-n", type=int, default=500)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=200.0)
    ap.add_argument("--csv", default="bench_results.csv")
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]

    be = backend_from_args(a)
    rx_count = [0]
    motors = {mid: Motor(mid, be) for mid in ids}

    def on_frame(cid, d):
        m = motors.get(p.reply_motor_id(cid, d))
        if m is None:
            return
        msg = p.parse_frame(cid, d, m.params)
        if msg is None:
            return
        rx_count[0] += 1
        m.handle_frame(msg)

    be.set_recv_callback(on_frame)
    be.open()
    try:
        # ── RTT: one non-clearing fault query round trip ──
        rtts = []
        m0 = motors[ids[0]]
        for _ in range(a.rtt_n):
            t0 = time.perf_counter()
            try:
                m0.query_fault(timeout=0.5)
                rtts.append((time.perf_counter() - t0) * 1000.0)
            except ParamTimeout:
                pass
            time.sleep(0.002)
        rtts.sort()
        p50 = statistics.median(rtts) if rtts else float("nan")
        p95 = rtts[int(len(rtts) * 0.95)] if rtts else float("nan")
        pmax = rtts[-1] if rtts else float("nan")
        print(f"RTT ms: p50={p50:.2f} p95={p95:.2f} max={pmax:.2f} (succeeded {len(rtts)}/{a.rtt_n})")

        # ── sustained throughput: rate Hz × len(ids) probe frames ──
        rx_count[0] = 0
        tx = overruns = 0
        dt = 1.0 / a.rate
        t_end = time.perf_counter() + a.seconds
        next_t = time.perf_counter()
        while time.perf_counter() < t_end:
            for mid in ids:
                be.send(*p.encode_fault_query(mid), extended=False)
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
        print(f"throughput: tx={tx} rx={rx} rx/tx={rx / max(tx, 1):.3f} tick_overruns={overruns}")

        with open(a.csv, "a", newline="") as f:
            w = csv.writer(f)
            if f.tell() == 0:
                w.writerow(["backend", "rtt_p50_ms", "rtt_p95_ms", "rtt_max_ms",
                            "tx", "rx", "rx_ratio", "overruns", "rate", "seconds"])
            w.writerow([a.backend, f"{p50:.3f}", f"{p95:.3f}", f"{pmax:.3f}",
                        tx, rx, f"{rx / max(tx, 1):.4f}", overruns, a.rate, a.seconds])
        print(f"appended to {a.csv}")
    finally:
        be.close()


if __name__ == "__main__":
    main()
