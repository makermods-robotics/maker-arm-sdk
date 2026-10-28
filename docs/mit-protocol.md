# MIT wire protocol

The SDK speaks only RobStride's MIT protocol: classic CAN, **standard 11-bit IDs, 1 Mbps**,
matching MakerModsLab and its LeRobot RobStride driver. Commands go to arbitration ID = motor
ID. Replies come back on the host ID `0xFD` with the motor ID in payload byte 0; typed
parameter replies come back on `0x300|id` (read) or `0x400|id` (write).

| Frame | Payload | Effect |
|---|---|---|
| Enable | `FF FF FF FF FF FF FF FC` | torque on, returns feedback |
| Disable | `FF FF FF FF FF FF FF FD` | torque off, returns feedback |
| State read / clear fault | `FF FF FF FF FF FF FF FB` | clears faults, returns feedback |
| Fault query | `FF FF FF FF FF FF 00 FB` | returns the fault word, changes nothing |
| Set zero | `FF FF FF FF FF FF FF FE` | current position becomes zero (persists) |
| MIT control | q16 · dq12 · kp12 · kd12 · tau12 | impedance command, returns feedback |
| Param read | ID `0x300|id`: index LE16, 6 zero bytes | firmware >= RS00 0.0.3.27 / RS02 0.2.3.32 |
| Param write | ID `0x400|id`: index LE16, 2 zero bytes, value LE32 | same firmware |

The SDK reads state with the fault-clearing `FB` frame only after a non-clearing fault query
shows no fault; once a motor reports a fault it refuses that frame until `clear_fault()`.

## Feedback

Bytes 1–2 position (16 bit, ±12.57 rad), 3–4.4 velocity (12 bit), 4.4–5 torque (12 bit).
Byte 6 bits 7–6 are the mode (0 disabled, 2 enabled), bit 5 the fault flag, bit 4 the warning
flag; the low 12 bits of bytes 6–7 are winding temperature ×10 °C. Older firmware leaves the
mode and flag bits zero, so the SDK only checks for mode 2 on motors that support parameter
access. Ranges: RS00 ±33 rad/s, ±14 Nm; RS02 ±44 rad/s, ±17 Nm (LeRobot types O0 and O1).

## Fault status

Five bytes (legacy) or zero-padded eight bytes: motor ID, then a LE uint32 fault word. Bits:
0 overtemperature, 1 driver chip, 2 undervoltage, 3 overvoltage, 4/5/16 phase B/C/A
overcurrent, 7 encoder uncalibrated, 8 hardware identification, 9 position initialization,
14 stall/overload.

## Parameters

`0x7005` RUN_MODE (must be 0 = MIT control) and `0x7028` CAN timeout (20 ticks per ms). The
SDK arms a 500 ms CAN timeout on the gripper only, verifies it, and restores the previous
value on disable — exactly as MakerModsLab. On firmware without parameter access it logs a
warning and runs without it, as MakerModsLab does.

RS02 firmware <= 0.2.2.11 maps position to ±12.5 rad instead of ±12.57; the SDK assumes the
newer range, so check J2/J3 firmware during bring-up.

## Private protocol

New RobStride motors ship on the private 29-bit protocol. Switching to MIT is a one-time
factory step (`tools/experimental/lerobot/switch_protocol.py`, then a power cycle). A motor
still on the private protocol does not answer the SDK.

Sources: RobStride RS00/RS02 manuals 260713 section 6; makermods-robotics/lerobot@5ce2fe41a
`motors/robstride`; MakerModsLab `maker_fault_diagnostics.py`, `maker_gripper_hold.py`.
