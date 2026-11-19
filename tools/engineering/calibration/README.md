# Calibration development

- `capture_maker_limits.py`: characterize the follower's mechanical endpoints and write backed-off
  `limits_deg: [lo, hi]` into the profile given with `--config`.
- `capture_star_ranges.py`: characterize the Star leader's servo endpoints (raw degrees).
- `calibrate_star_mapping.py`: measure the Star mapping (`{servo, direction, range_deg}`) after
  checking both arms read zero in the same pose; `--range-from-profile` takes `range_deg` from the
  follower limits.
- `artifacts/`: raw measurements for a named arm.

A replaced motor only needs `maker-arm zero --motor ID`, not these tools.
