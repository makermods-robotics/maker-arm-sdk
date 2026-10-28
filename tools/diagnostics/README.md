# Diagnostics

These tools are optional and are not installed as public commands. `encoder_gui.py`,
`bench_backend.py` and `capture_gripper_encoder.py` are torque-free and never clear a motor
fault. `jog_one_motor.py` and `open_gripper_mit.py` apply torque to one motor and must be used
only with the arm supported and their documented motion bound understood.
`urdf_viewer/` reads only the Star leader and never touches the CAN bus.
