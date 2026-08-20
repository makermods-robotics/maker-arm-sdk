#!/usr/bin/env python3
"""Bus scan: send a stop-frame probe to each ID in the range, print position/temperature/fault for motors online."""

import argparse
import time

from maker_arm import protocol as p
from maker_arm.errors import fault_text
from maker_arm.transport.base import create_backend


def backend_from_args(a):
    if a.backend == "socketcan":
        return create_backend("socketcan", channel=a.channel)
    return create_backend(a.backend, port=a.port)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
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
                # Keep only one unanswered probe in flight. Bursting IDs through a
                # CANable-class serial adapter can consistently hide later motors.
                deadline = time.monotonic() + 0.05
                while mid not in found and time.monotonic() < deadline:
                    time.sleep(0.001)
                if mid in found:
                    break
    finally:
        be.close()

    if not found:
        print("no motors found — check power/wiring/termination resistor/baud rate (1M)")
        return
    print(f"{'ID':>3} {'pos_rad':>10} {'temp_C':>8} fault")
    for mid in sorted(found):
        fb = found[mid]
        print(f"{mid:>3} {fb.position:>10.3f} {fb.temperature:>8.1f} {fault_text(fb.fault_bits)}")


if __name__ == "__main__":
    main()
