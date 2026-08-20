# maker-arm

Python SDK and command-line tools for the Maker Arm v1: six RobStride joints plus an RS00
gripper, teleoperated in absolute coordinates from a Star 102 leader.

Officially supported platforms:

- Linux using SocketCAN.
- macOS using a serial SLCAN adapter.

The production joint limits, motor models, gains, directions, and Star-to-Maker scale are fixed
model data shipped in `maker_arm/profiles/`. End users calibrate only motor zero position after
initial assembly or replacement. Range capture and mapping-generation utilities remain in
`tools/engineering/` and are not part of the supported user workflow.

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
maker-arm doctor       inspect dependencies, ports, and profiles
maker-arm scan         find follower motors on CAN
maker-arm assign-id    assign an ID to one factory/replacement motor
maker-arm zero         zero one replacement motor or the full arm
maker-arm check        run a bounded powered response check
maker-arm teleop       run absolute Star-to-Maker teleoperation
```

## Linux

For a serial SLCAN dongle, create `can0` at 1 Mbps:

```bash
sudo bash scripts/linux/setup_slcan.sh /dev/ttyACM0 can0
maker-arm scan --channel can0 --max-id 7
maker-arm teleop --channel can0 --star-port /dev/ttyUSB0 --max-velocity 5
```

Native SocketCAN adapters only need a configured `can0`; they do not use the SLCAN setup script.
See `docs/linux.md`.

## macOS

Locate the follower CANable and Star leader ports:

```bash
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
maker-arm scan --backend slcan --port /dev/cu.usbmodem-FOLLOWER --max-id 7
maker-arm teleop \
  --backend slcan \
  --port /dev/cu.usbmodem-FOLLOWER \
  --star-port /dev/cu.usbserial-LEADER \
  --max-velocity 5 \
  --record-motor-states
```

Serial SLCAN defaults to the verified 25 Hz control/update rate and waits for fresh feedback
from each motor before commanding the next. This is required by older
`normaldotcom/canable-fw`, whose unbuffered USB path can otherwise hide motors 4–7.
See `docs/macos.md`.

## Motor replacement and zero calibration

Connect only the unassigned replacement motor while changing its ID:

```bash
maker-arm assign-id --backend slcan --port /dev/cu.usbmodem-FOLLOWER \
  --current-id 127 --new-id 4
```

Install the motor, place that joint in the Maker Arm v1 documented zero pose, then run:

```bash
maker-arm zero --backend slcan --port /dev/cu.usbmodem-FOLLOWER --motor 4
maker-arm check --backend slcan --port /dev/cu.usbmodem-FOLLOWER --motor 4
```

Use `maker-arm zero --all` only for factory assembly or a deliberate complete re-zero. Zeroing
does not rewrite model limits or Star mapping. The exact mechanical zero fixture/pose must be
documented and validated before the first public hardware release; see
`docs/zero-calibration.md`.

## Safety

- Enabling begins from fresh measured positions and approaches targets through a velocity limit.
- Fixed soft limits are enforced in absolute joint coordinates.
- A host feedback watchdog, motor fault checks, and the motor-side CAN watchdog remain active.
- Ctrl-C stops leader following and holds the latest measured pose.
- The supported powered commands require the operator to type `RELEASE` before intentionally
  releasing torque. Forced process termination or loss of CAN/power can still trigger the
  motor-side watchdog and drop an unsupported arm.
- Always support the arm and payload before enable, fault recovery, or torque release.

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
- `tools/experimental/lerobot/`: unsupported protocol-switching experiments.

See `tools/README.md` and `docs/engineering-tools.md`.

## Development

```bash
python -m pytest -q
```

Linux vcan integration tests run when a `vcan0` interface is available. CI covers Linux and
macOS; hardware acceptance remains a separate operator-run step.
