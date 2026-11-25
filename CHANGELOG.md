# Changelog

## Unreleased

- The control loop keeps its rate on small computers. On a big.LITTLE board (Cortex-A53 + A72, gs_usb) it held
  about 147 Hz at rest and fell to 20 Hz while `move_ee` solved its path, until the arm faulted on stale feedback;
  now about 192 Hz at rest and 199 Hz during `move_ee` planning.
  - `move_ee` solves its path in a helper process (`_planner.py`), so the solve no longer holds the interpreter
    lock the control loop and the CAN receive thread need.
  - Planning is bounded: `move_ee`'s `timeout` and `cancel` apply while the path is planned, and the helper gets
    5 s to start and solve the end pose, then 3 s plus 25 ms per path tick, never more than 60 s in all; a path
    longer than 60 s at the control rate is refused before it is built. A helper that runs out of time, is
    cancelled, dies or cannot be talked to is killed and `move_ee` returns False with nothing sent; the next
    plan starts a fresh helper. Only where no helper process can be created at all is the path solved
    in-process, under the same limits. A forked child starts with its own helpers.
  - Each motion has an owner: a newer target, `stop()`, `estop()`, `disable()`, a fault or a return to rest
    supersedes a `move_ee` that is still planning or streaming, and none of its rows is sent afterwards.
  - The control loop and the receive threads run on the fastest cores of a big.LITTLE CPU, and while a loop runs
    the GIL switch interval is 0.5 ms instead of 5 ms (`realtime.py`; `ARM_SDK_THREAD_TUNING=off` leaves both
    alone). Computers whose cores are all alike are not pinned.
  - Less work per tick: the gravity model in Python floats (3-5x faster), and frames are spaced 200 us start to
    start instead of 150 us after each send, so a slow USB write is not followed by a needless wait.
- A computer that cannot keep the profile's rate is reported and stays safe: `Arm.loop_rate_hz` (also in
  `status()`), a warning once per session below 90 % of the rate, and each tick moves the commands by the time
  that really passed since the previous one (up to four periods), so speed limits hold in real time.
- Feedback and fault-status silence count from the oldest unanswered command or query (a query that fails to
  send counts), not from the last reply: a control loop that could not run for a while is no longer reported as
  a motor feedback timeout. The return to rest still releases torque only on readings younger than
  `feedback_timeout`.
- `url:` camera ids read a camera another program serves over HTTP: a multipart MJPEG stream or a single-JPEG
  URL (`rig set-camera front url:http://127.0.0.1:8090/front/stream.mjpg`). A receive thread keeps only the
  newest JPEG, frames are stamped when they arrive (less the server's `X-Frame-Age-Ms` when given), and a stream
  that drops is reopened. Header lines (8 KiB), header count (64) and frames (8 MiB) are bounded; closing a
  `url:` camera never waits on a server that sends slowly. `cameras list` shows the rig's `url:` cameras;
  `snapshot`, `stream` and `Rig.camera()` take them like USB ones.
- `set-id` (`Arm.set_motor_id`) changes a RobStride motor's CAN ID with MIT command 7, with torque off. The motor
  is read first and refused when it reports a fault or torque on, reads more than 0.03 N*m or moves faster than
  0.3 rad/s, or, where its firmware cannot report torque state, without `torque_off_confirmed=True` (the CLI's
  confirmation gives it). Refused when the new ID answers, or when a joint's motor would take another joint's ID; afterwards
  the motor must answer on the new ID and nothing on the old one. A joint's ID goes only to a motor not in the
  profile and only when `replacing` (`--replacing JOINT`) names that joint; if both IDs answer afterwards, that
  Arm refuses to enable or attach until the bus is checked and a new Arm is made.
- `stop()` ends on the support: near the supported rest pose (profile `rest_window_deg`; the Maker profile sets
  12) the weight-carrying joints go a little past where the enable found them toward it (at most 3 deg and
  1.75 N*m of proportional effort, inside the limits), so a return that stops a little short no longer becomes
  the next session's rest pose. After a verified return the control loop stops and each returned joint is read
  once more; torque goes off only if the readings are fresh and no joint moved away from rest. stop() holds
  instead when no joint returns to rest or a weight-carrying joint that does not return stands away from rest,
  and every way out of stop() that does not release (a second Ctrl-C included) leaves the arm holding at full
  gains with the control loop running.
- Maker URDF: `link_003`'s centre of mass is 48 mm from the shoulder axis (was 134 mm), fitted to holding torques
  measured on the arm at 16 low poses (`tests/unit/gravity_measured.json`): the shoulder_lift residual falls from
  0.58 to 0.15 N*m RMS (static friction there is about 0.06), elbow_flex and wrist_flex are unchanged. Over that
  range a constant counter-torque on the shoulder fits as well; the change moves the shoulder term by at most
  0.59 N*m anywhere in the workspace.
- `gravity --check` gives the gravity model the chain's joints only, so it compares the measured holding torque
  with the model on an arm with a gripper.

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
