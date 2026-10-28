# Tools

- `diagnostics/`: optional inspection and bounded bench diagnosis.
- `engineering/`: internal factory/profile development; may overwrite calibration.
- `experimental/`: unsupported research and integration work, including the one-time factory
  switch of new motors from RobStride's private protocol to MIT (`experimental/lerobot/`).

Every tool speaks only the RobStride MIT protocol through the SDK, and takes the same backend
options as `maker-arm` (`--backend`, `--channel`, `--port`, ...). The retired AT-serial backend
is retained under `experimental/transports/` and is not included in the release wheel.

Normal users should use the installed `maker-arm` command instead of scripts in this directory.
