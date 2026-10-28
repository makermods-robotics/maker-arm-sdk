# Maker Arm URDF bench

A local 3D inspection viewer for the Maker Arm designer export and the revised
two-slider gripper, connected to the Star leader's seven angle sensors. It serves
XML and meshes from the selected ZIP. It sends only FashionStar **angle queries (command 16)**;
there is no motor actuation, zeroing, torque release, CAN connection, or SDK
calibration write operation.

From the SDK repository root:

```sh
npm ci --prefix tools/diagnostics/urdf_viewer
npm run build --prefix tools/diagnostics/urdf_viewer
python3 -m tools.diagnostics.urdf_viewer \
  --zip output/urdf-revisions/Maker_Arm_gripper_inspection.zip \
  --star-port /dev/cu.usbserial-1120 \
  --reference-file output/urdf-revisions/star_rest_reference.json
```

Open <http://127.0.0.1:8420/>. Keep the terminal running. Ctrl+C stops the viewer
and closes its sensor connection. Use `--no-serial` for CAD inspection alone.
Dependencies are the SDK's existing Python dependencies and the two pinned npm
packages. Browser assets are served locally; no CDN is required. The server is
bound to loopback because the Star is attached to this computer. Do not run
another serial reader or teleop process against the same port at the same time.

## Start at rest

The first complete fresh set of readings becomes the session's Star rest
reference. It is captured by the server, so loading the browser later does not
silently capture a different pose. `Capture current Star as rest` changes the
reference in that browser only. It never writes encoder zero or changes the SDK.

The model initially shows the designer's exported zero pose. This does **not**
prove it matches the real Star or Maker resting pose. Use the per-joint reference
angle controls to investigate an offset, and record observations. These are
unverified viewer settings, never production calibration.

For joint i, live mode displays:

```
q_urdf = q_at_rest + viewer_sign * sdk_direction * sdk_scale * radians(star_now - star_rest)
```

The alternative `1:1 Star angle` mode omits the SDK direction and scale. The
existing absolute SDK target is included only as a separate quantity in the
downloaded report; it is not mistaken for a calibrated URDF angle. Angles are
neither wrapped nor stretched to fit the designer's limits. Out-of-range angles
are displayed and marked orange rather than silently clamped.

All six arm readings must be fresh (under 0.5 seconds) to update the robot. The
seventh gripper reading drives both sliding jaws in the revised model, and is
reported separately in an original export without gripper joints. A stale
gripper reading freezes the jaws. Actual packet timeouts retain the last angle with its old timestamp.
Browser-side freshness also expires if the server stops responding.

The installed MotorBridge 0.0.4 reliability filter conflates near-zero filtered
samples and missing replies. The viewer therefore reads raw multi-turn replies
directly, checking header, length, checksum, command and servo ID. Genuine 0° is
valid data. Raw readings may show sensor glitches; this diagnostic has no physical
actuation path. The fixed read packet and decoder follow the upstream
[FashionStar protocol implementation](https://github.com/motorbridge/motorbridge-smart-servo/blob/main/smart_servo_vendors/fashionstar/src/protocol.rs).

## Inspect in this order

1. **CAD zero:** check all visible parts, their placement, and the folded shape.
2. **One joint:** move the Star gently by a few degrees. Check the expected row,
   highlighted pivot and all downstream parts. Look for gaps opening at pivots.
3. **Direction and magnitude:** verify at least two measured Maker joint angles.
   SDK mode includes the Star-to-Maker scale. One matching pose only checks an
   offset, and the Star's geometry is not a dimensional reference for the Maker.
4. **Dimensions:** compare shaft-center distances and actual Maker tool position
   in several poses. The panel's origin distance is between frame origins; it
   equals a shaft-center spacing only if those frames were placed at those centers.
5. **Limits:** orange means outside the designer's limits. Compare mechanical
   travel and SDK soft limits only after their zero/direction mapping is verified.
6. **Gripper:** Star ID 6 should move both fingers equally. Each jaw travels
   52.4125 mm; total opening is 104.825 mm. Manual gap controls are model-only.
   The motor-to-gap interpolation is a preview approximation. The hardware open
   capture reached a pre-existing safe limit with backoff; that gap was not
   physically remeasured, and intermediate linkage nonlinearity is unmeasured.
7. **Physical properties:** collision and COM overlays show the declared model.
   They do not validate the numbers. Weigh rigid subassemblies and check inertia,
   friction, actuation and contact in the target physics simulator.

The up selector applies a **display-only** +90° X rotation to stand this CAD's
Y-up geometry on a Z-up grid. The original coordinates remain available under
`URDF +Z (unaltered)`. Base lift is also only a display translation. The report
records both. The viewer does not run physics or collision detection.

Use `Download inspection report` to retain source hashes, the Star reference,
current readings, viewer adjustments, checklist results and notes. This is an
inspection record, not a URDF correctness certificate.

## Verification

```sh
python3 -m pytest tests/test_urdf_viewer.py tests/test_gripper_revision.py -q
npm test --prefix tools/diagnostics/urdf_viewer
```

The viewer accepts the six named revolute arm joints, optionally accompanied by
gripper_left_joint, gripper_right_joint (mimic), and fixed grasp_center_joint.
Unknown additional joints are rejected rather than silently assigned sensors.

## Decorative loops

The `Hide decorative loops (visual only)` toggle hides the two identified STL
components `part_046` and `part_047`, and is enabled by default for the requested
original export. In the gripper revision those visual references are absent, so
the checkbox is disabled. Their collision geometry and inertia are on the moving
jaws because the user confirmed the real arm keeps the loops. They still appear
in collision mode. Inspection reports record the hidden mesh names.

`python3 -m tools.engineering.revise_gripper_assets --source ORIGINAL.zip --output REVISED.zip`
creates the equivalent portable visual-only ZIP and a geometry/measurement
manifest for completing the gripper. It does not claim the missing gripper motion
is fixed and does not invent mechanical travel or mass values.

## Build the gripper revision

```sh
python3 -m tools.engineering.build_gripper_urdf \
  --source "$HOME/Downloads/Maker Arm_urdf_260902.zip" \
  --output output/urdf-revisions/Maker_Arm_gripper_inspection.zip \
  --endpoints output/urdf-revisions/gripper_mit_endpoints.json
```

This reduced model omits the internal crank-and-rod geometry, keeps its inertia
lumped into the wrist, and redistributes jaw mass/inertia from the source CAD
estimate. Original total mass, center of mass and full inertia are conserved at
CAD zero. Jaw effort 20 N and speed 0.1 m/s are explicit placeholders. The archive
contains the exact scope, mesh assignments and physical-validation requirements
in revision_report.json. It is ready for visual inspection, not validated RL.
