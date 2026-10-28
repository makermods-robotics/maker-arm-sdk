# Getting started

1. Install the package and run `maker-arm doctor`. It checks the computer and lists adapters;
   it does not contact the motors.
2. Configure Linux SocketCAN, or plug in the macOS USB CAN adapter. Close MakerModsLab or any
   other program using the adapter.
3. Run `maker-arm scan` and verify CAN IDs 1–7 report `ok` with sensible positions.
4. If the arm was never zeroed (or a motor was replaced), place it in the zero pose — folded
   against the base, gripper fully closed — and run `maker-arm zero --all` (or `--motor ID`).
   An arm already zeroed in MakerModsLab needs nothing.
5. Run `maker-arm check` with the arm supported and workspace clear.
6. Run `maker-arm teleop --star-port ...`.

Limits, gains, and the Star mapping come from the versioned model profiles, identical to
MakerModsLab's. Normal setup never captures or rewrites endpoint ranges.
