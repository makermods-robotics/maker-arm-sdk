# Replacing a motor

1. Support and fully depower the arm.
2. Install the correct model (`RS02` for J2/J3, `RS00` elsewhere).
3. Factory preparation of the replacement (done once, before it joins the arm):
   - set its CAN ID to the joint's ID (1–7) with RobStride's Motor Studio, and
   - switch it from the private protocol to MIT with
     `python tools/experimental/lerobot/switch_protocol.py --ids ID --to mit`, then power-cycle.
   The SDK only speaks MIT, so a motor still on the private protocol never answers `scan`.
4. Install the linkage and put the arm in the zero pose (folded against the base, gripper fully
   closed).
5. Run `maker-arm zero --motor ID`.
6. Run `maker-arm scan`, followed by `maker-arm check --motor ID`.
7. Verify direction and small motion before returning to teleoperation.

Do not use the engineering endpoint capture merely because a motor was replaced; the mechanical
limits belong to the arm model, not to an individual encoder.
