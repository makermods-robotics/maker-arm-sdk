# Engineering-only tools

These utilities are retained for Maker Arm developers. They are not end-user calibration and are
not installed as `maker-arm` commands. Range capture and mapping changes can make the arm unsafe
if they are written into a production profile without mechanical validation.

- `calibration/`: follower limit capture (`limits_deg`) and Star mapping calibration
  (`{servo, direction, range_deg}`); see its README.
- `gen_urdf.py`, `build_gripper_urdf.py`, `revise_gripper_assets.py`: URDF generation from the
  profile limits plus hand-measured geometry; no hardware access.
