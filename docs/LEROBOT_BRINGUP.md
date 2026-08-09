# lerobot teleop bench acceptance runbook (for the operator to run)

Prerequisites: motors 3~7 on can0 (slcand already up), Star on /dev/ttyUSB0, conda env `metal-lerobot`.
private<->MIT is a persistent, mutually-exclusive protocol; a power-cycle is required after
switching. If something goes wrong, go back to the private state and troubleshoot with the SDK tools.

1. **Persist the motor-side watchdog (private state)**
   `conda run -n maker-arm python tools/persist_can_timeout.py --ids 3,4,5,6,7`
   Expected: all 5 report "written and saved to flash".
2. **Switch to MIT**
   `conda run -n maker-arm python tools/switch_protocol.py --ids 3,4,5,6,7 --to mit`
   -> power-cycle the motors -> verify the private protocol is now unresponsive: `conda run -n maker-arm python tools/scan_bus.py --max-id 8` (expected: no motors found).
3. **MIT handshake + read (no torque)**
   `cd ~/makermods/lerobot && ~/miniconda3/envs/metal-lerobot/bin/python -c "from lerobot.motors import Motor, MotorNormMode; from lerobot.motors.robstride import RobstrideMotorsBus; ms={n: Motor(i,'O0',MotorNormMode.DEGREES) for n,i in [('elbow_flex',3),('wrist_flex',4),('wrist_yaw',5),('wrist_roll',6),('gripper',7)]}; [setattr(m,'recv_id',m.id) or setattr(m,'motor_type_str','O0') for m in ms.values()]; bus=RobstrideMotorsBus(port='can0',motors=ms,can_interface='socketcan',use_can_fd=False,bitrate=1000000); bus.connect(); print(bus.sync_read('Present_Position')); bus.disconnect()"`
   Expected: a dict of degrees for the 5 joints. Push any motor by hand and rerun — the value should change.
4. **Hold test (verify canTimeout doesn't false-trigger in MIT mode)**
   Just run the teleoperate from step 5 and keep Star still for 60 seconds = the hold test. If a motor releases torque / drops out partway through: canTimeout behaves abnormally in MIT mode -> switch back to private and re-persist with --timeout-ms raised to 1000, or set it to 0 and log it (fallback path) — setting it to 0 disables the motor-side watchdog entirely, a last resort; restore it as soon as possible after logging.
5. **Teleop**
   `cd ~/makermods/lerobot && conda run --no-capture-output -n metal-lerobot lerobot-teleoperate --config_path /home/ethan/makermods/maker_arm_bench.json`
   The first run enters the zero-calibration interactive flow (recommended to first zero using
   set_zero in the private state). Tracking criteria: all 5 of Star's channels follow
   independently, direction is correct (if direction is wrong, flip the sign of that channel's
   `joint_directions` in the json), and it sits still without oscillation when released. Tuning
   order: sluggish tracking -> raise `robot.max_relative_target` / it naturally goes full-speed
   once startup_sync completes; soft -> gains kp +10; jittery -> kd +0.3. Also: gripper travel
   will saturate at the `joint_limits` placeholder values (bench ±170°) — don't mistake that for
   a direction/gain problem before the limits are calibrated and written back.
6. **Round-trip ferry verification**
   Ctrl-C out of teleoperate -> `conda run -n maker-arm python tools/switch_protocol.py --ids 3,4,5,6,7 --to private` -> power-cycle -> `conda run -n maker-arm python tools/scan_bus.py --max-id 8` (expected: 3~7 all online).
7. **Wrap-up**: write the tuned gains/directions back into both json files; once bench
   acceptance is complete, after full-arm assembly redo this runbook with
   maker_arm_lerobot.json (ids become 1~7).

Known issue: MIT-mode set_zero persistence is unverified — if calibration is lost across a
power cycle, zero it in the private state with tools/set_zero.py instead (which the workflow
already recommends anyway).

> Note (as of 2026-08-07, arm #02 is deployed): J2/J3 (shoulder_lift/elbow_flex) are RS02
> motors; on the lerobot side the model is tentatively set to **O1** (±17Nm/±44rad/s) per the
> protocol doc data. This must be verified against real hardware once assembled (suspicion: the
> upstream O-series and RS-series numbering are offset — O2=±20Nm/±33 is a different tier) —
> method: spin J2 at a fixed speed with mit_spin and compare MIT Present_Velocity against the
> private-protocol reading; a 44/33x discrepancy means the wrong model was picked.
