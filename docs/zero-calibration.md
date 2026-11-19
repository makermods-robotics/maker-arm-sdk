# Zero calibration

## The zero pose

Folded against the base, gripper fully closed. Zeroing is the only end-user calibration: limits,
gains and the Star mapping are model data in the shipped profile.

## Commands

```bash
maker-arm zero --all                         # every follower motor
maker-arm zero --all --star-port PORT        # the follower, then the Star leader in the same pose
maker-arm zero --leader-only --star-port PORT
maker-arm zero --motor 4                     # one replaced motor
```

Torque stays off for the whole procedure: move the arm to the zero pose by hand and confirm. The
command prints the positions before and after. The zero is stored in the motors (RobStride
set-zero) and in the leader servos. Without a terminal, pass `--yes-i-am-at-the-zero-pose`.

## After a power cycle

A power cycle can reset a motor's multi-turn count. `connect()` corrects a reading that is exactly
one turn off; any other reading outside the limits is a stale zero and `enable()` refuses it.
