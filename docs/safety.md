# Safety

- Support the arm before enabling or releasing torque.
- Keep people and obstacles outside the complete workspace.
- Begin new hardware checks with a low velocity and bounded motion.
- Never depend on software holding as the only support for a payload.
- On missing CAN traffic, a motor's watchdog may release torque even while reachable joints hold.
- Supported powered commands hold after Ctrl-C and require the word `RELEASE` for intentional
  torque release. Killing the process, unplugging USB, losing power, or a motor fault can still
  remove holding torque.
- Use a hardware emergency stop and mechanically appropriate power isolation.
