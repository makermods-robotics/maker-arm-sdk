# maker-arm

Python SDK and command-line tools for the Maker Arm v1: six RobStride joints plus an RS00
gripper, teleoperated in absolute coordinates from a Star 102 leader.

Officially supported platforms:

- Linux using SocketCAN.
- macOS using a serial SLCAN adapter.

The SDK speaks only the RobStride **MIT protocol** (standard 11-bit CAN at 1 Mbps) and drives
the arm exactly as MakerModsLab does through its LeRobot `MakerFollower`: the same joint
limits, models and gains, the same zero pose, the same Star leader mapping, and the same
safety behaviour (see [Safety](#safety)). An arm set up and zeroed in MakerModsLab works with
the SDK unchanged, and the other way round. Close MakerModsLab before using the SDK: only one
program can use the CAN adapter at a time.

Limits, models, gains and the Star mapping are fixed model data in `maker_arm/profiles/`.
End users only set the zero pose, after assembly or a motor replacement. See
[MIT protocol](docs/mit-protocol.md) for wire details.

## Install

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[dev]"
maker-arm --help
maker-arm doctor
```

## Supported commands

```text
maker-arm doctor       inspect dependencies, adapters, and profiles (does not contact motors)
maker-arm scan         find motors: position, temperature, and fault status
maker-arm zero         set the zero pose (follower, optionally the Star leader)
maker-arm check        run a bounded powered response check
maker-arm teleop       teleoperate from the Star 102 leader
```

Defaults follow the platform: SocketCAN `can0` on Linux, serial SLCAN on macOS with the
USB CAN adapter found automatically when exactly one is plugged in (`--port` otherwise).

## Linux

For a serial SLCAN dongle, create `can0` at 1 Mbps:

```bash
sudo bash scripts/linux/setup_slcan.sh /dev/ttyACM0 can0
maker-arm scan
maker-arm teleop --star-port /dev/ttyUSB0
```

Native SocketCAN adapters only need a configured `can0`; they do not use the SLCAN setup script.
See `docs/linux.md`.

## macOS

```bash
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
maker-arm scan
maker-arm teleop --star-port /dev/cu.usbserial-LEADER --record-motor-states
```

Serial SLCAN defaults to the verified 25 Hz control/update rate and waits for fresh feedback
from each motor before commanding the next. This is required by older
`normaldotcom/canable-fw`, whose unbuffered USB path can otherwise hide motors 4–7.
See `docs/macos.md`.

## Zero calibration and motor replacement

The zero pose is the one MakerModsLab uses: **folded against the base, gripper fully closed**.
Torque is released, you move the arm there by hand, and the zero is stored in the motors:

```bash
maker-arm zero --all                       # whole follower
maker-arm zero --all --star-port PORT      # follower, then the Star leader in the same pose
maker-arm zero --motor 4                   # one replaced motor
maker-arm check --motor 4
```

Motor IDs (1–7) and the one-time switch of a new motor from RobStride's private protocol to
MIT are factory steps; see `docs/motor-replacement.md`.

## Safety

The SDK mirrors MakerModsLab's Maker arm behaviour:

- **Read-only connect.** Every motor gets a non-clearing fault query first. A reported fault is
  shown decoded and kept for inspection; `maker-arm scan --clear-faults` clears it explicitly.
  Connecting never enables or releases torque.
- **Enable from the measured pose.** Each motor is preloaded with a zero-gain command at its
  present position before enable, so an old setpoint cannot jump the arm. A joint more than
  20° outside its limits after full-turn correction (stale zero) refuses to enable.
- **Startup sync.** Until a joint has caught up with its target, its command stays within 1° of
  the measured position (synced within 3°, or released if it cannot move for 1 s). Targets
  are always clipped to the joint limits.
- **Faults are latched.** A fault word or feedback fault flag stops the arm holding its current
  pose (no drop) and fault-clearing commands are refused until an explicit `clear_faults()`.
  Feedback older than 250 ms is a fault. Fault status is polled every 250 ms without clearing.
  Health checks keep running while holding, so a second failing motor is still caught; a faulted
  joint is no longer commanded.
- **Gripper holding and protection** (MakerModsLab's default gripper hold): closing effort is
  capped at 1 Nm and, on contact, the grip is regulated to 0.5 Nm instead of squeezing at full
  gain. It stops at 55 °C, will not enable unless below 45 °C, and stops on its warning or fault
  flag; a faulted or hot gripper is released rather than held. A 500 ms motor watchdog is armed
  on firmware that supports it (RS00 >= 0.0.3.27) and removed only after torque-off is confirmed.
- **Verified return to rest.** Stopping returns the joints (not the gripper) to the pose they had
  at enable at 30°/s and releases torque only once they arrive. If the return cannot be
  verified, the arm keeps holding until the operator types `RELEASE`.
- Joints 1–6 have no motor watchdog (as in MakerModsLab): if the host dies they hold their
  last command. Always support the arm before enable, fault recovery or torque release.

Read `docs/safety.md` before powered operation.

## Telemetry and stall detection

Add `--record-motor-states` to teleoperation to create a CSV plus summary JSON under `logs/`.
The recorder captures targets, rate-limited commands, position error, velocity, measured torque,
temperature, mode, faults, and feedback age for all seven motors. A confirmed persistent stall
freezes leader following and leaves the arm holding until the operator types `RELEASE`.

## Engineering and experimental tools

- `tools/diagnostics/`: read-only inspection, backend benchmarking, and bounded motor diagnosis.
- `tools/engineering/calibration/`: factory range capture and mapping work; these can overwrite
  production calibration and are intentionally not installed as public commands.
- `tools/experimental/lerobot/`: the factory private-to-MIT protocol switch.

See `tools/README.md` and `docs/engineering-tools.md`.

## Development

```bash
python -m pytest -q
```

Linux vcan integration tests run when a `vcan0` interface is available. CI covers Linux and
macOS; hardware acceptance remains a separate operator-run step.
