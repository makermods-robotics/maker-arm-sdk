# Engineering tools

For developing an arm revision, not for end users. Writing captured ranges or mappings into a
production profile without mechanical validation can make the arm unsafe.

- `switch_protocol.py`: one-time factory switch of a new RobStride motor from its private protocol
  to MIT (`--ids 1,2,3 --to mit`, then power-cycle, then `maker-arm scan`). Probing a motor still on
  the private protocol sends a private stop frame, which releases its torque: support the arm.
- `calibration/`: follower limit capture and Star mapping calibration; see its README.
- `gen_urdf.py`, `build_gripper_urdf.py`, `revise_gripper_assets.py`: URDF generation and gripper
  asset revision from hand-measured geometry; no hardware access.
