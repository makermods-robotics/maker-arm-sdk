# Zero calibration

Zero calibration is the only supported end-user calibration, identical to MakerModsLab's Maker
calibration. It never changes model limits, gains, or the Star-to-Maker mapping.

```bash
maker-arm zero --all                    # follower
maker-arm zero --all --star-port PORT   # follower, then the Star leader
maker-arm zero --leader-only --star-port PORT
maker-arm zero --motor 4                # one replaced motor
```

Torque is released for the whole procedure. Move the arm by hand to the ZERO POSE — folded
against the base, gripper fully closed — and confirm. The command prints the positions before
and after; the zero is stored in the motors (RobStride set-zero) and in the leader servos
(FashionStar origin point). A power cycle can reset a motor's multi-turn count; the SDK
corrects a whole-turn offset automatically at connect.
