# Factory protocol switch (private -> MIT)

New RobStride motors ship speaking RobStride's private 29-bit protocol. The SDK, MakerModsLab and
LeRobot speak only MIT (standard 11-bit CAN, 1 Mbps), so every new motor is switched once. The
switch is persistent and takes effect only after a power cycle.

1. **Switch to MIT** (arm supported: probing a motor still on the private protocol sends a
   private stop frame, which releases its torque)
   `python tools/experimental/lerobot/switch_protocol.py --ids 1,2,3,4,5,6,7`
   (`--to mit` is the default; backend options are the same as `maker-arm`: `--backend`,
   `--channel`, `--port`). Each motor is probed first; motors already on MIT are skipped.
   Expected: every switched motor reports "command acked".
2. **Power-cycle the motors.**
3. **Verify with the SDK**: `maker-arm scan`
   Expected: every switched motor is listed, with position, temperature and fault status.

`--to private` undoes the switch (again followed by a power cycle); it is not part of the normal
workflow.

The motor-side CAN watchdog is not persisted here: as in MakerModsLab, the SDK arms the gripper's
500 ms watchdog at enable time from the profile's `can_timeout_ms` (firmware RS00 >= 0.0.3.27).

`mit_move.py` and `mit_spin.py` are single-motor LeRobot (`RobstrideMotorsBus`) bench helpers for
motors already on MIT; they need a LeRobot environment and are not used by the SDK.
