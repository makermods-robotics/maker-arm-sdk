# Zero calibration

Zero calibration is the only supported end-user calibration. It is intended for initial factory
assembly or replacement of a failed motor. It never changes model limits, directions, gains, or
the Star-to-Maker mapping.

```bash
maker-arm zero --motor 4
maker-arm zero --all
```

The arm must be torque-free and the selected joint must be placed in the Maker Arm v1 defined
mechanical reference pose before writing zero. The command requires an explicit target so an
operator cannot accidentally zero every motor.

Release blocker: the production mechanical zero fixture/pose and expected post-zero coordinates
for all seven joints still need a drawing and acceptance tolerances. Do not publish a hardware
release until that reference is documented and verified against the fixed profile.
