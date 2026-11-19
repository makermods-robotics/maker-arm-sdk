# Changelog

## 0.1.0

First release of the Maker Arm SDK (RobStride motors, MIT protocol).

- `Arm`: connect, enable, hold, `set_joint_targets`, `move_to`, `move_ee`/`ee_pose`, float mode,
  gravity feed-forward from the URDF, verified return to rest, fault and temperature protection.
- Backends: SocketCAN, serial SLCAN, gs_usb/candleLight over USB; Linux and macOS.
- Rig file for several arms, cameras and leaders; parallel arm groups.
- Trajectory IK for chunks of tool poses (`IKStream`), continuous with the commands already sent.
- Teleoperation from a Star 102 leader or a second arm, single or bimanual.
- `maker-arm` CLI: doctor, ports, rig, identify, scan, status, clear-faults, hold, release, move,
  fk, ik, gravity, check, zero, set-id, params, teleop, leader, cameras; `--json` on every command.
