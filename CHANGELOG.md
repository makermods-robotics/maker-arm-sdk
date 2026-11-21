# Changelog

## 0.1.0

First release of the Maker Arm SDK (RobStride motors, MIT protocol).

- `Arm`: connect, enable, hold, `set_joint_targets`, `move_to`, `move_ee`/`ee_pose`, float mode,
  gravity feed-forward from the URDF, verified return to rest, fault and temperature protection.
- Backends: SocketCAN, serial SLCAN, gs_usb/candleLight over USB; Linux and macOS.
- Rig file for several arms, cameras and leaders; parallel arm groups.
- Trajectory IK for chunks of tool poses (`IKStream`), continuous with the commands already sent.
- Optional JAX trajectory IK (`jax` extra, `solver="jax"`): Dream Labs' compiled `dream_ik` solver behind the same
  interface.
- `testing`: simulated arms for application tests; `Camera.newest()` (a `Frame` with a monotonic age); gripper
  `open_position` with `Arm.gripper_opening()`/`gripper_target()`; `ArmGroup.close()` with confirmed per-arm
  outcomes; `bus.release()`, a verified torque-off for an arm that will not connect.
- `Arm.enable(speed=)`: caps the motion until the first target (a joint resting past a limit is brought inside
  at that speed).
- `Arm.attach()`: takes over an arm left holding with torque on, without a sag; `connect(clear_released=True)`
  clears what the gripper's watchdog left.
- Teleoperation from a Star 102 leader or a second arm, single or bimanual.
- `maker-arm` CLI: doctor, ports, rig, identify, scan, status, clear-faults, hold, rest, release, move,
  fk, ik, gravity, check, zero, set-id, params, teleop, leader, cameras; `--json` on every command.
