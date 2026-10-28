# Safety

- Support the arm before enabling or releasing torque.
- Keep people and obstacles outside the complete workspace.
- Begin new hardware checks with a low velocity and bounded motion.
- Never depend on software holding as the only support for a payload.
- On a fault the arm holds its measured pose; the gripper is released instead. Fault evidence is
  kept until you clear it explicitly.
- Stopping returns the arm to its enable pose at 30°/s and releases torque only after the
  return is verified. Otherwise supported commands keep holding until you type `RELEASE`.
- Joints 1–6 have no motor-side watchdog, matching MakerModsLab: if the process is killed they
  keep holding their last command. The gripper's 500 ms watchdog (on firmware that supports
  it) releases it instead. Unplugging power or a motor fault can still remove holding torque.
- Use a hardware emergency stop and mechanically appropriate power isolation.
