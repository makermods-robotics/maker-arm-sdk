#!/usr/bin/env python3
"""Private-protocol state: persist the motor-side CAN timeout watchdog into flash (default 200ms -> 4000 counts).

Must run once before switching to the MIT protocol — MIT mode has no parameter-table access,
so this is the only guarantee that the motor auto-releases torque on a disconnect.
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
                    raise RuntimeError(f"readback {rb} != {counts}")
                m.save_params()
                time.sleep(0.1)
                ok.append(mid)
                print(f"motor{mid}: canTimeout={counts} counts (={a.timeout_ms}ms) written and saved to flash")
            except Exception as e:
                fail.append(mid)
                print(f"motor{mid}: ❌ {e}")
    finally:
        be.close()
    print(f"done: succeeded {ok}, failed {fail}")
    raise SystemExit(1 if fail else 0)


if __name__ == "__main__":
    main()
