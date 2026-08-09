# maker-arm

A pure-Python SDK for a self-built RobStride-motor robotic arm (**6 joints + 1 gripper**, the
gripper is an RS00 @ID7 running as a plain MIT joint 7 — grip force = kp x position error,
small kp means compliant force limiting). Shares its protocol with EDULITE A3 (RobStride
private CAN @1Mbps). Supports mixed models per joint: each joint's `model:` field in the YAML
(RS00/RS02, default RS00) determines its T/V mapping table. The deployed arm uses J2/J3=RS02
with the rest RS00: `configs/maker_arm.yaml` is the default config for every tool.
Design doc: `docs/superpowers/specs/2026-08-04-maker-arm-sdk-design.md` in the makermods repo.

## Install

    conda create -y -n maker-arm python=3.11 && conda run -n maker-arm pip install -e ".[dev]"

## Layers

transport(socketcan/at/mock) -> protocol(pure functions) -> motor -> arm(state machine + 200Hz control loop) -> tools/examples

## Quick start (10-line teleop kernel)

    from maker_arm import Arm
    arm = Arm.from_yaml("configs/maker_arm.yaml", backend="socketcan", channel="can0")
    arm.connect(); arm.enable()
    arm.set_joint_targets([0.0]*7)   # rad; the 200Hz loop rate-limits and smooths the approach
    arm.disable(); arm.disconnect()

## First-time setup (run once, in order — don't skip steps)

> ⚠️ For interactive tools (monitor's limit capture, set_zero, examples), run the environment's python directly (e.g. `~/miniconda3/envs/maker-arm/bin/python tools/monitor.py`) — `conda run` doesn't pass through terminal stdin (Enter/Ctrl-C behave oddly), so monitor auto-downgrades to read-only monitoring.

1. Bring up the CAN interface: `sudo bash scripts/setup_slcan.sh` (SLCAN dongle -> can0 @1Mbps).
2. Assign motor CAN IDs (factory default is 127): connect ONE motor at a time and run `python tools/set_can_id.py --current-id 127 --new-id N` (N = 1..7, base to gripper).
3. `python tools/scan_bus.py` — are all 7 IDs (6 joints + gripper) online?
4. `python tools/monitor.py` — push each joint by hand, does direction/value make sense? (use this to fill in `direction` in configs; flip direction BEFORE zeroing/limits — it changes the sign of every reading)
5. `python tools/set_zero.py` — move the arm to its zero pose and zero all motors.
6. Measure limits: run `python tools/monitor.py` again, push each joint to both end limits by hand while the screen shows min/max/travel live; press **Enter** to automatically write the backed-off lo/hi into `configs/maker_arm.yaml` (comments preserved, `.bak` backup, auto rollback if post-write validation fails), raw record in `configs/limits_capture.json`; joints that were never pushed are skipped with a warning. Ctrl-C exits without writing.
7. `python examples/02_enable_hold.py` — first powered hold; tune kp/kd (sagging = raise kp, buzzing = lower kp / raise kd).

> Note: the mode!=2 health check has a 25ms persistence tolerance (5 consecutive ticks before it's flagged as a fault), so the stale-feedback race at the instant of enabling won't false-trigger; if it still reports "abnormal mode" it genuinely never entered motion-control state — check that motor's enable acknowledgment.

8. Teleop calibration: full two-pose `python tools/calib_star_map.py --star-port /dev/ttyUSBx`, or `--anchor-only` (both arms at their zero poses) if direction/scale are already set in `configs/star_to_maker.json`.

## Normal use (daily)

1. If `can0` is missing (fresh boot, or the dongle was replugged): `sudo bash scripts/setup_slcan.sh`
2. Teleop: `~/miniconda3/envs/maker-arm/bin/python examples/04_teleop_star.py --star-port /dev/ttyUSB0`
   - Absolute-anchor mode by default: the follower moves (rate-limited, confirmation prompt if the pose gap exceeds 0.8 rad) to the pose matching the leader, then tracks. Add `--rebase` for relative mode (anchor to the current poses, zero motion at start).
   - Ctrl-C to stop: the arm **stays locked** until you press Enter (support it or park it first), then torque is released.
3. That's it. No re-zeroing or re-calibration needed day to day:
   - Motor reboots that shift readings by ±2π are detected and compensated automatically at connect (info log).
   - Re-run anchor calibration (step 8 above) only if you re-zero the motors or move the leader's zero convention.

## SLCAN dongle (same CANable-class adapter as the metal arm)

Plug in and bring it up as can0 with one command; the SDK uses the default socketcan backend, zero code changes:

    sudo bash scripts/setup_slcan.sh

(defaults to /dev/ttyACM0 -> can0 @1Mbps; just rerun after unplugging/replugging. The serial port name / interface name can be passed as arguments.)

## Dual-backend comparison

    python tools/bench_backend.py --backend socketcan --channel can0
    python tools/bench_backend.py --backend at --port /dev/ttyUSB0

## Safety design

- On enable, the target is set to the current position (first-frame protection); the target can only approach at a max_velocity rate limit (prevents runaway motion). Enable refuses out-of-limit positions that no ±2π shift explains.
- Soft limits inset by limit_margin; host-side watchdog (feedback timeout) and per-motor fault/mode checks trigger FAULT; with `hold_on_fault: true` (default) FAULT **locks the joints at the current pose** instead of letting the arm drop — motors that can't be reached release themselves via the motor-side CAN_TIMEOUT=200ms watchdog (which also covers host crashes). `estop()` always releases torque.
- Fault codes are translated and reported loudly, never silently dropped; ±2π reading jumps after motor reboots are compensated per session at connect.

## Offline tests

    conda run -n maker-arm pytest -q                          # protocol/transport/motor/arm unit tests
    sudo modprobe vcan && sudo ip link add dev vcan0 type vcan; sudo ip link set up vcan0
    conda run -n maker-arm pytest tests/test_vcan_integration.py -q   # full chain, no hardware needed

## lerobot teleop

For integrating with lerobot (RobStride MIT protocol + the upstream RobstrideMotorsBus) see
`docs/LEROBOT_BRINGUP.md`; the protocol-switch tool is `tools/switch_protocol.py`, and the
watchdog-persistence tool is `tools/persist_can_timeout.py`.
