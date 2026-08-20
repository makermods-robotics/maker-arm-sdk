# Factory calibration development

- `capture_maker_limits.py`: manually characterize follower mechanical endpoints.
- `capture_star_ranges.py`: characterize Star leader servo endpoints.
- `calibrate_star_mapping.py`: update absolute mapping anchors during profile development.
- `artifacts/`: raw measurements retained as provenance for a named arm.

Motor replacement uses `maker-arm zero --motor ID`; it does not use these tools.
