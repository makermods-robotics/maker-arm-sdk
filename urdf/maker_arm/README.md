# Maker Arm URDF

Load [`robot.urdf`](robot.urdf) directly, keeping the adjacent `meshes/` folder.
No ZIP extraction or viewer is needed. Mesh paths are relative to this URDF.

The model contains six revolute arm joints and a symmetric sliding gripper.
Set `gripper_left_joint` from **0 to 0.0524125 metres**; `gripper_right_joint`
mimics that coordinate along the opposite axis. Total inside jaw opening is
**0–104.825 mm**. The fixed `grasp_center` frame is at the CAD inner-face area
centroid, midway between the jaws. Simulators must support URDF mimic joints or
enforce the same relationship explicitly.

The two finger-loop visuals are removed. Their collision meshes and mass remain
on the corresponding moving jaws because the physical arm retains these parts.
The internal crank, rods, and associated drive hardware are omitted from visual
and collision geometry; their inertia is lumped into the wrist.

## Validation status

This revision is ready for visual and kinematic inspection. It is **not yet
validated for RL dynamics**:

- Jaw opening and equal travel are user-provided measurements. Intermediate
  motor-angle-to-gap behavior has not been calibrated.
- Moving-jaw mass and inertia are redistributed from the original CAD density
  estimate. The original total mass, center of mass, and inertia are preserved
  at the closed pose; these are not measured physical properties.
- Gripper effort **20 N** and velocity **0.1 m/s** are placeholders.
- The six arm joints retain the designer's axes, origins, and limits. Their
  physical zero alignment and actuator limits still require validation.
- Collision meshes need validation/decomposition in the target simulator.
  Friction, backlash, and self-collision exclusions remain unverified.
- The designer export omitted `part_003`; its identity remains unresolved.

[`revision_report.json`](revision_report.json) records source provenance, mesh
assignments, inertia estimates, and the remaining checks. Its motor endpoint
values are diagnostic observations: the opening capture reached a previous
safe limit with backoff, and the 104.825 mm gap was not remeasured at that encoder
position. Do not use the two-point interpolation as a calibrated transmission.

The original CAD coordinates are retained. The inspection viewer used a +90° X
display rotation for this CAD's Y-up geometry; that display transform is not
encoded as a change to the URDF base frame.
