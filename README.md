# maker-arm

Python SDK and command-line tool for the Maker Arm: six RobStride joints and a gripper, driven over
CAN from Linux, macOS or Windows. The [`metal-arm`](https://github.com/makermods-robotics/metal-arm-sdk) SDK drives the Metal Arm with the same
API and the same commands.

## Install

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[cameras]"
maker-arm doctor                    # checks the computer; talks to no motor
```

Pick the CAN adapter you have:

| Adapter | How to use it |
|---|---|
| candleLight / gs_usb (USB) | `--backend gs_usb`. Needs libusb (`apt install libusb-1.0-0` or `brew install libusb`; bundled on Windows) and, on Linux, the udev rule below. |
| SocketCAN (Linux) | `--backend socketcan --channel can0`, after `sudo ip link set can0 up type can bitrate 1000000`. |
| Serial SLCAN | `--backend slcan --port /dev/ttyACM0` (`COM3` on Windows). Runs the arm at 25 Hz. |

```bash
# Linux, once: let non-root users open gs_usb adapters
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="0666"' |
  sudo tee /etc/udev/rules.d/99-gs-usb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## First run

The arm has no brakes: it drops when torque goes off away from its rest pose. Keep the workspace
clear.

```bash
maker-arm ports                          # 1. what is plugged in: adapters, cameras, serial ports
maker-arm rig init --detect              # 2. write the rig file: arm1, arm2, camera1, ...
maker-arm cameras snapshot               # 3. look at the JPEGs to see which camera is overhead
maker-arm identify --assign --top camera1    # 4. wiggle each gripper; names the arms left / right
maker-arm scan --arm left                # 5. motors 1-7: position, temperature, faults
maker-arm hold --arm left --for 5        # 6. torque on for 5 s, then back to rest and off
maker-arm check --arm left --at shoulder_lift=-20,elbow_flex=25 --sine 10    # 7. lift, then a tracking test
```

The rig file (`~/.config/maker-arm/rig.yaml`) names your arms, cameras and leaders, so every command
takes `--arm left`. Without a rig file, pass `--backend` and `--port` or `--channel`.

A new arm, or one with a replaced motor, needs its zero set once:
[docs/zero-calibration.md](docs/zero-calibration.md).

## Commands

Every command takes `--json` and `--help`.

**Look, without touching motors**

| Command | What it does |
|---|---|
| `doctor` | Check the computer, the profile and the rig file. |
| `ports` | List CAN adapters, cameras and serial ports. |
| `rig` | `show`, `init --detect`, `set-arm`, `set-camera`, `set-leader`: edit the rig file. |
| `cameras` | `list`, `snapshot` (one JPEG per camera), `stream` (measure frame rate). |
| `fk` | Tool pose for given joints, or for the arm's current joints. |
| `ik` | Joints for a tool pose, or for a short tool path (`--to-xyz ... --steps N`). |

**Read motors (never enables them)**

| Command | What it does |
|---|---|
| `scan` | Find motors by ID: position, temperature, faults. |
| `status` | Position, velocity, torque, temperatures and faults per joint. `--watch` repeats. |
| `params` | `list`, `get NAME`, `set NAME VALUE`: motor parameters. |

**Move the arm (torque on)**

| Command | What it does |
|---|---|
| `identify` | Wiggle each gripper and use the cameras to tell which arm is left or right. |
| `hold` | Hold the current pose until Ctrl-C or `--for S`. |
| `move` | `--joint NAME --to DEG` or `--by DEG`, `--pose zero`, or the tool point: `--xyz X Y Z`, `--dxyz DX DY DZ`. |
| `check` | Jog each joint and report PASS/FAIL. `--sine DEG` runs a tracking test; `--at` lifts first. |
| `gravity` | Float the arm so you can move it by hand. `--check` compares the gravity model with measured torque. |
| `teleop` | Follow a Star leader (`--star-port`) or a second arm (`--leader arm --leader-arm NAME`). |
| `leader` | Star leader: `zero` sets its origin, `read` prints what it would command. |

These commands end with `--then rest` (return to where the arm started, then torque off; the usual
default), `--then stay` (exit with torque on) or `--then release` (torque off where it is). If the
return to rest cannot be confirmed, the arm keeps holding and the command exits with status 1.

**Change the motors**

| Command | What it does |
|---|---|
| `rest` | Move to the rest pose and turn torque off; an arm left on is taken over where it holds. |
| `release` | Torque off where the arm is: it drops unless it is at rest or supported. |
| `clear-faults` | Clear motor faults once the cause is fixed. |
| `zero` | Make the current pose the zero. Stored in the motors. |
| `set-id` | Not supported on RobStride motors: set IDs with RobStride Motor Studio. |

## Python

```python
import math, time
from maker_arm.rig import Rig

arm = Rig.load().arm("left")
arm.connect()                                   # finds the motors; torque stays off
print(dict(zip(arm.joint_names, arm.get_joint_positions())))    # radians

arm.enable()                                    # torque on, holding the current pose
start = arm.get_joint_positions()
for i in range(200):                            # 4 s: swing wrist_roll by 10 degrees
    q = list(start)
    q[5] += math.radians(10) * math.sin(math.pi * i / 50)
    arm.set_joint_targets(q)
    time.sleep(0.02)

xyz, rpy = arm.ee_pose()                        # tool point: metres, radians
arm.move_ee([xyz[0], xyz[1], xyz[2] + 0.03])    # straight up 3 cm
arm.stop()                                      # back to rest, then torque off
arm.disconnect()
```

Units are radians, rad/s, N·m, °C and metres. Joints are ordered `shoulder_pan, shoulder_lift,
elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper`.

| You want to | Call |
|---|---|
| Send targets from your own loop | `arm.set_joint_targets(q)` |
| Move and wait | `arm.move_to(q)`, `arm.move_ee(xyz, rpy)` |
| Read the arm | `get_joint_positions()`, `get_joint_velocities()`, `get_joint_torques()`, `get_temperatures()`, `status()`, `ee_pose()` |
| Hand-guide it | `arm.float_mode()`, then `arm.set_gains()` to stiffen again |
| Stop | `arm.stop()` (rest, then off), `arm.disable()` (off now), `arm.estop()` |
| Open or close the gripper | `arm.gripper_opening()` (measured, 0 closed to 1 open), `arm.gripper_target(opening)` (joint rad) |
| Use two arms | `Rig.load().group(["left", "right"])` |
| Stop a group and see how each arm ended | `group.close()` (what leaving the `with` block does): per arm `"released"` only when torque off was confirmed |
| Turn torque off on an arm that will not connect (a fault, a missing motor) | `maker_arm.bus.release(spec.open_backend(), spec.config())` |
| Read a camera without waiting | `camera.newest()` (`frame, stamp = ...`, or None; `.age_s` on the monotonic clock) |
| Test without hardware | `testing.simulated_arm()`, `testing.simulated_backend()`, `testing.simulated_group(rig)` |

Once enabled, the arm runs its own 200 Hz loop. It keeps targets inside the joint limits, limits
speed, adds gravity compensation, and holds the pose if anything faults.

The loop is a Python thread, so heavy Python work in your own threads (a policy, IK in a loop) slows it: it
shares the interpreter lock with them. `arm.loop_rate_hz` is the measured rate; below 90 % of the profile's the
SDK warns once, and each tick still moves by the time that really passed, so speed limits hold. On a big.LITTLE
board the loop and the CAN receive thread move to the fast cores by themselves, and `move_ee` solves its path in a
helper process. Set `ARM_SDK_THREAD_TUNING=off` to leave thread placement and the GIL switch interval alone.

### Your own teleoperation loop

```python
import time
from maker_arm.leaders import StarLeader
from maker_arm.rig import Rig

arm = Rig.load().arm("left")
leader = StarLeader("/dev/ttyUSB0")
leader.connect()
arm.connect()
arm.enable()
try:
    while arm.state.name == "ENABLED":          # a fault ends the loop; the arm then holds
        q = leader.read()                       # joint targets in radians, or None on a missed read
        if q is not None:
            arm.set_joint_targets(q)
        time.sleep(1 / 50)
finally:
    arm.stop()
    arm.disconnect()
    leader.close()
```

`maker_arm.teleop.Teleop` is the complete version (slow start when the poses differ, speed limit,
leader loss). `ArmLeader(other_arm)` makes a second arm the leader.

### Tool poses from a policy

`IKStream` turns chunks of tool poses into joint commands. Each chunk continues smoothly from the
commands already sent, and no joint moves more than its speed limit allows in one tick, so a small
tool move is always a small joint move.

```python
import numpy as np
from maker_arm.trajectory_ik import IKStream, TrajectoryIK

ik = TrajectoryIK.from_config(arm.config, chain=arm.chain)
stream = IKStream(ik, np.array(arm.get_commanded_positions()[:6]), rate_hz=20)

result = stream.feed(poses)                     # poses: (T, 4, 4) tool poses at 20 Hz
for q in result.q:                              # (T, 6) joint commands
    arm.set_joint_targets(list(q) + [gripper])
    time.sleep(1 / 20)
```

`stream.step(pose)` solves one pose at a time. `arm.move_ee` uses the same solver. The algorithm is
Dream Labs' `dream_ik`, ported to numpy; see [`NOTICE`](NOTICE).

The `jax` extra (`pip install -e '.[jax]'`, Python 3.11+) adds Dream Labs' original solver, compiled with JAX, behind
the same interface: `trajectory_ik.from_config(arm.config, chain=arm.chain, solver="jax")`. It is used only when asked
for (`"auto"`, the default, is numpy): importing it sets process-wide JAX settings. Its first solve of each chunk
length compiles for about half a minute (`ik.warmup(rows)` does it ahead), then a chunk solves several times faster
than with numpy, so choose it for a long-running policy loop; `move_ee` and the CLI always use numpy.

### Cameras

```python
with Rig.load().camera("top") as cam:
    frame, stamp = cam.read()                   # BGR image, time in seconds
```

A camera another program already serves over HTTP (a camera server, mjpg-streamer, an IP camera) is named by
its URL: `maker-arm rig set-camera front url:http://127.0.0.1:8090/front/stream.mjpg`. A multipart MJPEG stream or
a single-JPEG URL both work; only the newest frame is kept, frames are stamped when they arrive, and a stream that
drops is reopened.

Camera ids (`usb:<path>`) stay the same while a camera stays in the same USB port. On Windows cameras
are not listed; name one by its index, `index:0`.

More runnable code is in [`examples/`](examples).

## Safety

- **No brakes.** The arm drops when torque goes off away from rest.
- **Stopping returns to rest first.** Torque goes off only after the arm is confirmed back where it
  started. Otherwise it keeps holding and waits for you to type `RELEASE`.
- **A fault holds the arm in place.** Stale feedback, a motor fault or 55 °C on the gripper
  stops motion; the gripper lets go, the other joints hold.
- **If the program is killed,** joints 1-6 keep holding their last command and the gripper lets go
  after 0.5 s. Run `maker-arm rest` to bring it down and turn torque off (`release` drops it where it is).
- **Keep a hardware emergency stop** on motor power.

More: [docs/safety.md](docs/safety.md).

## Hardware

| Joint | Motor ID | Model | Limits (deg) |
|---|---|---|---|
| shoulder_pan | 1 | RS00 | -158.2 .. 156.1 |
| shoulder_lift | 2 | RS02 | -175.3 .. -3.2 |
| elbow_flex | 3 | RS02 | 2.8 .. 236.1 |
| wrist_flex | 4 | RS00 | -65.2 .. 104.0 |
| wrist_yaw | 5 | RS00 | -97.6 .. 78.0 |
| wrist_roll | 6 | RS00 | -153.2 .. 152.0 |
| gripper | 7 | RS00 | -96.0 .. -2.5 (closes toward 0) |

RobStride motors on the MIT protocol, classic CAN at 1 Mbit/s, host ID 0xFD.

Zero pose: folded against the base, gripper fully closed. The arm's limits, gains and tool point are in
[`maker_arm/profiles/maker_arm_v1.yaml`](maker_arm/profiles/maker_arm_v1.yaml). The tool point comes from the Dream IK spec;
check it on your arm before relying on Cartesian moves. Wire protocol: [docs/mit-protocol.md](docs/mit-protocol.md). Replacing a motor: [docs/motor-replacement.md](docs/motor-replacement.md).

## Development

```bash
pip install -e ".[dev]"
python -m pytest -q
python tools/sync_shared.py --check ../metal-arm-sdk      # shared files must be identical
```

Most of this package is shared, byte for byte, with `metal-arm-sdk`. Only `_product.py`,
`protocol.py`, `motor.py`, `profiles/` and `urdf/` belong to this arm. Edit shared files in
`maker-arm-sdk` and copy them over with `tools/sync_shared.py --copy-to ../metal-arm-sdk`.
