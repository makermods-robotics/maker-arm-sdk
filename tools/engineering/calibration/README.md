# Factory calibration development

- `capture_maker_limits.py`: manually characterize follower mechanical endpoints; writes
  backed-off `limits_deg: [lo, hi]` into an explicitly selected profile.
- `capture_star_ranges.py`: characterize Star leader servo endpoints (raw degrees).
- `calibrate_star_mapping.py`: measure the LeRobot-style Star mapping (`{servo, direction,
  range_deg}`, no offset) after checking both arms read zero in the same pose; optionally set
  `range_deg` from the follower profile limits.
- `artifacts/`: raw measurements retained as provenance for a named arm.

Motor replacement uses `maker-arm zero --motor ID`; it does not use these tools.
