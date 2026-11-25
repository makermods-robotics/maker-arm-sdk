# Changelog

## Unreleased

- The control loop keeps its rate on small computers. On a big.LITTLE board (Cortex-A53 + A72, gs_usb) it held
  about 147 Hz at rest and fell to 20 Hz while `move_ee` solved its path, until the arm faulted on stale feedback;
  now about 192 Hz at rest and 199 Hz during `move_ee` planning.
  - `move_ee` solves its path in a helper process (`_planner.py`), so the solve no longer holds the interpreter
    lock the control loop and the CAN receive thread need. Falls back to solving in-process if no helper starts.
  - The control loop and the receive threads run on the fastest cores of a big.LITTLE CPU, and while a loop runs
    the GIL switch interval is 0.5 ms instead of 5 ms (`realtime.py`; `ARM_SDK_THREAD_TUNING=off` leaves both
    alone). Computers whose cores are all alike are not pinned.
  - Less work per tick: the gravity model in Python floats (3-5x faster), and frames are spaced 200 us start to
    start instead of 150 us after each send, so a slow USB write is not followed by a needless wait.
- A computer that cannot keep the profile's rate is reported and stays safe: `Arm.loop_rate_hz` (also in
  `status()`), a warning once per session below 90 % of the rate, and each tick moves the commands by the time
  that really passed (up to four periods), so speed limits hold in real time.
- Feedback and fault-status silence count from the oldest unanswered command or query, not from the last reply:
  a control loop that could not run for a while is no longer reported as a motor feedback timeout.
- `url:` camera ids read a camera another program serves over HTTP: a multipart MJPEG stream or a single-JPEG
  URL (`rig set-camera front url:http://127.0.0.1:8090/front/stream.mjpg`). A receive thread keeps only the
  newest JPEG, frames are stamped when they arrive (less the server's `X-Frame-Age-Ms` when given), and a stream
  that drops is reopened. `cameras list` shows the rig's `url:` cameras; `snapshot`, `stream` and
  `Rig.camera()` take them like USB ones.

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
