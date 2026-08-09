# maker-arm

A pure-Python SDK for a self-built RobStride-motor robotic arm (**6 joints + 1 gripper**, the
gripper is an RS00 @ID7 running as a plain MIT joint 7 — grip force = kp x position error,
small kp means compliant force limiting). Shares its protocol with EDULITE A3 (RobStride
private CAN @1Mbps). Supports mixed models per joint: each joint's `model:` field in the YAML
(RS00/RS02, default RS00) determines its T/V mapping table. Currently deployed as maker-arm02
(J2/J3=RS02, the rest RS00): `configs/maker_arm.yaml` is the default config for every tool.
Design doc: `docs/superpowers/specs/2026-08-04-maker-arm-sdk-design.md` in the makermods repo.

## Install

    conda create -y -n maker-arm python=3.11 && conda run -n maker-arm pip install -e ".[dev]"

## Layers

transport(socketcan/at/mock) -> protocol(pure functions) -> motor -> arm(state machine + 200Hz control loop) -> tools/examples

## Quick start (10-line teleop kernel)

    from maker_arm import Arm
    arm = Arm.from_yaml("configs/maker_arm_6dof.yaml", backend="socketcan", channel="can0")
    arm.connect(); arm.enable()
    arm.set_joint_targets([0.0]*6)   # rad; the 200Hz loop rate-limits and smooths the approach
    arm.disable(); arm.disconnect()

## Real-hardware bring-up sequence (run in order, don't skip steps)

1. `python tools/scan_bus.py` — are all 7 IDs (6 joints + gripper) online?
2. `python tools/monitor.py` — push each joint by hand, does direction/value make sense? (use this to fill in `direction` in configs)
3. Single-motor bench test: `python examples/03_sine_wave.py --joint N`
4. `python tools/set_zero.py` — move to the zero pose and zero it
> ⚠️ For interactive tools (monitor's limit capture, set_zero, examples), run the environment's python directly (e.g. `~/miniconda3/envs/maker-arm/bin/python tools/monitor.py`) — `conda run` doesn't pass through terminal stdin (Enter/Ctrl-C behave oddly), so monitor auto-downgrades to read-only monitoring.

5. Measure limits: after zeroing, run `python tools/monitor.py` again, push each joint to both end limits by hand while the screen shows min/max live; press **Enter** to automatically write the backed-off lo/hi into `configs/maker_arm_6dof.yaml` (comments preserved, `.bak` backup, auto rollback if post-write validation fails), with the raw record stored in `configs/limits_capture.json`; joints that were never pushed are skipped with a warning. Ctrl-C exits without writing. Then use `python examples/02_enable_hold.py` to tune kp/kd

> Note: the mode!=2 health check has a 25ms persistence tolerance (5 consecutive ticks before it's flagged as a fault), so the stale-feedback race at the instant of enabling won't false-trigger; if it still reports "abnormal mode" it genuinely never entered motion-control state — check that motor's enable acknowledgment.

6. `python tools/calib_star_map.py --star-port /dev/ttyUSBx` -> `python examples/04_teleop_star.py --star-port /dev/ttyUSBx`

## SLCAN dongle (same CANable-class adapter as the metal arm)

Plug in and bring it up as can0 with one command; the SDK uses the default socketcan backend, zero code changes:

    sudo bash scripts/setup_slcan.sh

(defaults to /dev/ttyACM0 -> can0 @1Mbps; just rerun after unplugging/replugging. The serial port name / interface name can be passed as arguments.)

## Dual-backend comparison

    python tools/bench_backend.py --backend socketcan --channel can0
    python tools/bench_backend.py --backend at --port /dev/ttyUSB0

## Safety design

- On enable, the target is set to the current position (first-frame protection); the target can only approach at a max_velocity rate limit (prevents runaway motion)
- Soft limits inset by limit_margin; host-side feedback timeout -> FAULT releases torque; motor-side CAN_TIMEOUT=200ms (motor auto-releases torque if the process crashes)
- Fault codes are translated and reported loudly, never silently dropped

## Offline tests

    conda run -n maker-arm pytest -q                          # protocol/transport/motor/arm unit tests
    sudo modprobe vcan && sudo ip link add dev vcan0 type vcan; sudo ip link set up vcan0
    conda run -n maker-arm pytest tests/test_vcan_integration.py -q   # full chain, no hardware needed

## lerobot teleop

For integrating with lerobot (RobStride MIT protocol + the upstream RobstrideMotorsBus) see
`docs/LEROBOT_BRINGUP.md`; the protocol-switch tool is `tools/switch_protocol.py`, and the
watchdog-persistence tool is `tools/persist_can_timeout.py`.
