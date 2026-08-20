#!/usr/bin/env python3
"""Bidirectional motor communication protocol switch (private <-> MIT). Requires a power-cycle to take effect.

private = maintenance state (maker-arm SDK/tools), MIT = lerobot runtime state (RobstrideMotorsBus).
Each motor's current protocol is probed before acting; frame formats anchored by real-hardware
testing on 2026-08-07. This is an unsupported engineering experiment, not part of the public SDK workflow.
⚠️ Probing uses a private stop frame: running this tool on a motor that is "still enabled" will
release its torque — make sure the motor is disabled / the arm is supported before running.
"""

import argparse
import time

from maker_arm import protocol as p
from maker_arm.transport.base import create_backend

PROBE_WAIT = 0.3


def classify_reply(can_id: int) -> str:
    """>0x7FF can only be a 29-bit extended frame (private-protocol reply); otherwise it's an 11-bit standard frame (MIT reply)."""
    return "private" if can_id > 0x7FF else "mit"


def private_alive(replies: list, motor_id: int) -> bool:
    return any(classify_reply(cid) == "private"
               and (cid >> 24) & 0x1F == p.COMM_FEEDBACK
               and (cid >> 8) & 0xFF == motor_id for cid, _ in replies)


def mit_alive(replies: list, motor_id: int, host_id: int = p.HOST_CAN_ID) -> bool:
    return any(classify_reply(cid) == "mit" and cid == host_id
               and len(d) >= 1 and d[0] == motor_id for cid, d in replies)


def switch_acked(replies: list, motor_id: int) -> bool:
    """Switch-command ack detection -- whitelist: only recognize three verified/documented forms (8-byte MCU-code payload).

    Verified: the private->MIT Type0 reply has cid=(motor ID<<8)|0xFE (motor 7 -> 0x7FE, motor
    127 -> 0x7FFE, confirmed twice on real hardware on 2026-08-07); the MIT->private command-8
    reply has cid=motor id; also kept per the MIT docs is the "sent to host with
    payload[0]=motor id" form. Nothing else is accepted.
    """
    for cid, d in replies:
        if len(d) != 8:
            continue
        if ((cid >> 8) == motor_id and (cid & 0xFF) == 0xFE) or cid == motor_id:
            return True
        if cid == p.HOST_CAN_ID and d[0] == motor_id:
            return True
    return False


def detect(be, replies: list, motor_id: int) -> str | None:
    """Probe the motor's current protocol: send a private stop frame and an MIT read-only fault query, see which gets a reply."""
    replies.clear()
    be.send(*p.encode_disable(motor_id))                          # private probe (extended frame)
    time.sleep(PROBE_WAIT)
    if private_alive(list(replies), motor_id):
        return "private"
    replies.clear()
    be.send(motor_id, p.mit_fault_query_data(), extended=False)   # MIT probe (standard frame, no side effects)
    time.sleep(PROBE_WAIT)
    if mit_alive(list(replies), motor_id):
        return "mit"
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--backend", choices=["socketcan", "slcan"], default="socketcan")
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="serial device for --backend slcan")
    ap.add_argument("--serial-baudrate", type=int, default=115200,
                    help="SLCAN UART rate (USB CDC CANable devices ignore this value)")
    ap.add_argument("--ids", required=True, help="comma-separated motor IDs, e.g. 3,4,5,6,7")
    ap.add_argument("--to", choices=["mit", "private"], required=True)
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]

    replies: list = []
    if a.backend == "socketcan":
        be = create_backend("socketcan", channel=a.channel)
    else:
        be = create_backend("slcan", port=a.port, baudrate=a.serial_baudrate,
                            rtscts=False)
    be.set_recv_callback(lambda cid, d: replies.append((cid, bytes(d))))
    be.open()
    switched, skipped, failed = [], [], []
    try:
        for mid in ids:
            cur = detect(be, replies, mid)
            if cur is None:
                failed.append(mid)
                print(f"motor{mid}: ❌ no response on either protocol — check power/wiring/whether it was switched but not power-cycled")
                continue
            if cur == a.to:
                skipped.append(mid)
                print(f"motor{mid}: already {cur}, skipping")
                continue
            replies.clear()
            if a.to == "mit":
                be.send(*p.encode_set_protocol(mid, 2))            # private Type25 -> MIT
            else:
                be.send(mid, p.mit_switch_protocol_data(0), extended=False)  # MIT command 8 -> private
            time.sleep(0.5)
            if switch_acked(list(replies), mid):
                switched.append(mid)
                print(f"motor{mid}: {cur} -> {a.to} command acked ✅")
            else:
                failed.append(mid)
                print(f"motor{mid}: ⚠️ switch command not acked — retry or check the protocol state")
    finally:
        be.close()
    print(f"\nswitched {switched}, skipped {skipped}, failed {failed}")
    if switched:
        print("⚠️ takes effect only after power-cycling: please power-cycle the motors, then verify --"
              f"{'maker-arm scan (expect no private response)' if a.to == 'mit' else 'maker-arm scan (expect all online)'}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
