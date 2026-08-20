#!/usr/bin/env python3
"""Assign a CAN ID to a motor (factory default 127).

⚠️ Only ONE motor awaiting an ID change may be connected to the bus at a time — SET_CAN_ID
addresses by current ID, so multiple motors all at 127 would get changed together. Standard
flow: connect one motor -> assign -> disconnect -> connect the next.

Usage: maker-arm assign-id --current-id 127 --new-id 1
Frame format (protocol Type 7): ID bits 23~16=new ID, bits 15~8=host ID, bits 7~0=current ID; data[0]=1.
"""

import argparse
import time

from maker_arm import protocol as p
from maker_arm.transport.base import create_backend


def probe(be, found, motor_id, tries=3):
    """Send stop-frame probes to an ID, return whether feedback was received."""
    found.clear()
    for _ in range(tries):
        be.send(*p.encode_disable(motor_id))
        time.sleep(0.05)
    return motor_id in found


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--current-id", type=int, required=True)
    ap.add_argument("--new-id", type=int, required=True)
    a = ap.parse_args()
    if not (1 <= a.new_id <= 127):
        raise SystemExit("--new-id must be between 1 and 127")

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    be = create_backend(a.backend, **kw)
    found = {}
    be.set_recv_callback(lambda cid, d: (
        (msg := p.parse_frame(cid, d)) and isinstance(msg, p.MotorFeedback)
        and found.__setitem__(msg.motor_id, msg)))
    be.open()
    try:
        if not probe(be, found, a.current_id):
            raise SystemExit(f"no response from current ID {a.current_id} — check the ID is correct / motor is powered")
        if probe(be, found, a.new_id):
            raise SystemExit(f"new ID {a.new_id} is already in use — disconnect that motor first or pick another ID")
        input(f"confirm only this one motor is on the bus (ID {a.current_id}->{a.new_id}), press ENTER to proceed > ")

        # Type 7: data2 = (new ID << 8) | host ID; data[0]=1
        cid = p.make_can_id(p.COMM_SET_CAN_ID, (a.new_id << 8) | p.HOST_CAN_ID, a.current_id)
        data = bytearray(8)
        data[0] = 1
        be.send(cid, bytes(data))
        time.sleep(0.3)

        if not probe(be, found, a.new_id):
            raise SystemExit(f"probe of new ID {a.new_id} failed after the change — retry or verify with the vendor tool")
        be.send(*p.encode_save_params(a.new_id))   # save-to-flash fallback (Type 7 itself takes effect immediately)
        time.sleep(0.2)
        fb = found[a.new_id]
        print(f"✅ motor is now ID {a.new_id} (position {fb.position:+.3f} rad, {fb.temperature:.1f}°C), save-to-flash sent")
    finally:
        be.close()


if __name__ == "__main__":
    main()
