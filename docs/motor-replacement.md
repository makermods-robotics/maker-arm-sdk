# Replacing a motor

1. Support and fully depower the arm.
2. Install the correct model (`RS02` for J2/J3, `RS00` elsewhere).
3. Connect only the factory-ID replacement to the CAN bus and assign its final ID.
4. Install the linkage at the documented model zero pose.
5. Run `maker-arm zero --motor ID`.
6. Run `maker-arm scan`, followed by `maker-arm check --motor ID`.
7. Verify direction and small motion before returning to teleoperation.

Do not use the engineering endpoint capture merely because a motor was replaced; the mechanical
limits belong to the arm model, not to an individual encoder.
