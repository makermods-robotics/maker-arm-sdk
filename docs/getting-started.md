# Getting started

1. Install the package and run `maker-arm doctor`.
2. Configure Linux SocketCAN or locate both macOS serial ports.
3. Run `maker-arm scan --max-id 7` and verify CAN IDs 1–7.
4. For factory assembly, place the complete arm in the defined model zero pose and run
   `maker-arm zero --all`. For a replacement, use `--motor ID`.
5. Run `maker-arm check` with the arm supported and workspace clear.
6. Run `maker-arm teleop --star-port ...`.

Limits, directions, gains, and the Star mapping come from the versioned model profiles. Normal
setup never captures or rewrites endpoint ranges.
