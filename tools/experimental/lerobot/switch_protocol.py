#!/usr/bin/env python3
"""One-time factory step: switch RobStride motors between the private protocol and MIT.

New motors ship speaking RobStride's private 29-bit protocol. The SDK, MakerModsLab and LeRobot
speak only MIT (standard 11-bit frames), so each motor is switched once with ``--to mit``, then
power-cycled, then verified with ``maker-arm scan``. ``--to private`` exists only to undo it.
The switch takes effect after a power cycle.

maker_arm no longer contains the private protocol, so the few private frames needed here are
defined locally. Frame formats were anchored by real-hardware testing on 2026-08-07.
This is an unsupported engineering tool, not part of the public SDK workflow.
⚠️ Probing a private-protocol motor uses a private stop frame: running this tool on a motor that
is still enabled will release its torque — make sure the arm is supported before running.
"""

import argparse
import time

from maker_arm import protocol
from maker_arm.cli.common import add_backend_args, backend_from_args

PROBE_WAIT = 0.3
HOST_ID = protocol.HOST_CAN_ID

# Private protocol (29-bit extended ID = type<<24 | data16<<8 | motor_id). Only what this tool needs.
PRIVATE_FEEDBACK = 2       # motor -> host status frame
PRIVATE_STOP = 4           # host -> motor stop (disable); answered with a feedback frame
PRIVATE_SET_PROTOCOL = 25  # host -> motor protocol switch; answered with a type-0 device-ID frame
PROTOCOL_PRIVATE, PROTOCOL_MIT = 0, 2


def private_id(comm_type: int, data16: int, motor_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (motor_id & 0xFF)


def private_stop_frame(motor_id: int, host_id: int = HOST_ID) -> tuple[int, bytes]:
    """Private type-4 stop: the probe for a motor still on the private protocol (extended frame)."""
    return private_id(PRIVATE_STOP, host_id, motor_id), bytes(8)


def private_to_mit_frame(motor_id: int, host_id: int = HOST_ID) -> tuple[int, bytes]:
    """Private type-25 protocol switch to MIT (extended frame); bytes 0-5 are the magic 01..06."""
    return (private_id(PRIVATE_SET_PROTOCOL, host_id, motor_id),
            bytes([1, 2, 3, 4, 5, 6, PROTOCOL_MIT, 0]))


def mit_to_private_frame(motor_id: int) -> tuple[int, bytes]:
    """MIT protocol switch back to private (standard frame to the motor ID): FF x6, 00, FD."""
    return motor_id, bytes([0xFF] * 6 + [PROTOCOL_PRIVATE, 0xFD])


def mit_probe_frame(motor_id: int) -> tuple[int, bytes]:
    """MIT non-clearing fault query (FF x6, 00, FB): no torque change, fault evidence kept."""
    return protocol.encode_fault_query(motor_id)


def classify_reply(can_id: int) -> str:
    """Extended (29-bit) frames are private-protocol replies; standard (11-bit) frames are MIT."""
    return "private" if getattr(can_id, "extended", can_id > 0x7FF) else "mit"


def private_alive(replies: list, motor_id: int) -> bool:
    return any(classify_reply(cid) == "private"
               and (cid >> 24) & 0x1F == PRIVATE_FEEDBACK
               and (cid >> 8) & 0xFF == motor_id for cid, _ in replies)


def mit_alive(replies: list, motor_id: int, host_id: int = HOST_ID) -> bool:
    return any(classify_reply(cid) == "mit" and cid == host_id
               and len(d) >= 1 and d[0] == motor_id for cid, d in replies)


def switch_acked(replies: list, motor_id: int) -> bool:
    """Switch-command ack detection -- whitelist: only recognize three verified/documented forms (8-byte MCU-code payload).

    Verified: the private->MIT type-0 reply has cid=(motor ID<<8)|0xFE (motor 7 -> 0x7FE, motor
    127 -> 0x7FFE, confirmed twice on real hardware on 2026-08-07); the MIT->private reply has
    cid=motor id; also kept per the MIT docs is the "sent to host with payload[0]=motor id"
    form. Nothing else is accepted.
    """
    for cid, d in replies:
        if len(d) != 8:
            continue
        if ((cid >> 8) == motor_id and (cid & 0xFF) == 0xFE) or cid == motor_id:
            return True
        if cid == HOST_ID and d[0] == motor_id:
            return True
    return False


def detect(be, replies: list, motor_id: int) -> str | None:
    """Probe the motor's current protocol: a private stop frame, then an MIT fault query."""
    replies.clear()
    can_id, data = private_stop_frame(motor_id)
    be.send(can_id, data, extended=True)
    time.sleep(PROBE_WAIT)
    if private_alive(list(replies), motor_id):
        return "private"
    replies.clear()
    can_id, data = mit_probe_frame(motor_id)
    be.send(can_id, data, extended=False)
    time.sleep(PROBE_WAIT)
    if mit_alive(list(replies), motor_id):
        return "mit"
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_backend_args(ap)
    ap.add_argument("--ids", required=True, help="comma-separated motor IDs, e.g. 1,2,3,4,5,6,7")
    ap.add_argument("--to", choices=["mit", "private"], default="mit",
                    help="target protocol (default: mit; private only undoes the factory switch)")
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]
    if not all(1 <= mid <= 127 for mid in ids):
        ap.error("motor IDs must be in [1, 127]")

    replies: list = []
    be = backend_from_args(a)
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
                can_id, data = private_to_mit_frame(mid)
                be.send(can_id, data, extended=True)
            else:
                can_id, data = mit_to_private_frame(mid)
                be.send(can_id, data, extended=False)
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
        expect = "all switched motors online" if a.to == "mit" else "switched motors missing (no longer MIT)"
        print(f"⚠️ takes effect only after power-cycling: power-cycle the motors, then verify with "
              f"`maker-arm scan` (expect {expect})")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
