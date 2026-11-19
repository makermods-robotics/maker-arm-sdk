# Tools

Developer tools, not installed with the package. Use the `maker-arm` command for normal work.
Tools that open the bus take the same arm options as `maker-arm` (`--arm`, `--backend`,
`--channel`, `--port`, `--config`).

- `sync_shared.py`: check or copy the files shared byte for byte with `metal-arm-sdk`.
- `diagnostics/`: torque-free inspection: `encoder_gui.py` (live encoder ranges),
  `bench_backend.py` (adapter round-trip and throughput), `capture_gripper_encoder.py`
  (gripper encoder at a measured jaw gap), `urdf_viewer/` (URDF viewer driven by the Star leader;
  no CAN access).
- `engineering/`: factory and model development; see its README. These can rewrite limits,
  mappings or motor settings.
