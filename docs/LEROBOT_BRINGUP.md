# lerobot 遥操台架验收 runbook（操作者执行）

前置：电机 3~7 在 can0（slcand 已挂）、Star 在 /dev/ttyUSB0、conda env `metal-lerobot`。
私有↔MIT 是持久互斥协议，切换后必须断电重启。出问题先回私有态用 SDK 工具排查。

1. **固化电机侧看门狗（私有态）**
   `conda run -n maker-arm python tools/persist_can_timeout.py --ids 3,4,5,6,7`
   预期：5 台全部"已写入并掉电保存"。
2. **切 MIT**
   `conda run -n maker-arm python tools/switch_protocol.py --ids 3,4,5,6,7 --to mit`
   → 给电机断电重启 → 验证私有已失聪：`conda run -n maker-arm python tools/scan_bus.py --max-id 8`（预期：未发现任何电机）。
3. **MIT 握手 + 读数（无力矩）**
   `cd ~/makermods/lerobot && ~/miniconda3/envs/metal-lerobot/bin/python -c "from lerobot.motors import Motor, MotorNormMode; from lerobot.motors.robstride import RobstrideMotorsBus; ms={n: Motor(i,'O0',MotorNormMode.DEGREES) for n,i in [('elbow_flex',3),('wrist_flex',4),('wrist_yaw',5),('wrist_roll',6),('gripper',7)]}; [setattr(m,'recv_id',m.id) or setattr(m,'motor_type_str','O0') for m in ms.values()]; bus=RobstrideMotorsBus(port='can0',motors=ms,can_interface='socketcan',use_can_fd=False,bitrate=1000000); bus.connect(); print(bus.sync_read('Present_Position')); bus.disconnect()"`
   预期：5 关节度数字典。手推任一电机重跑，数值应变。
4. **抱住测试（验证 canTimeout 在 MIT 模式不误触发）**
   直接跑步骤 5 的 teleoperate，Star 保持不动 60 秒 = 抱住测试。若电机中途泄力/掉线：canTimeout 在 MIT 模式语义异常 → 切回私有把 --timeout-ms 加大到 1000 重新固化，或置 0 并记录（回退路径）——置 0 即关闭电机侧看门狗，属最后手段，记录后尽快恢复。
5. **遥操**
   `cd ~/makermods/lerobot && conda run --no-capture-output -n metal-lerobot lerobot-teleoperate --config_path /home/ethan/makermods/maker_arm_bench.json`
   首次会进零位标定交互（建议先用私有态 set_zero 的零位姿态）。跟手判据：Star 摇 5 路各自跟随、方向正确（方向不对改 json 里该路 joint_directions 符号）、松手静止无振荡。调优顺序：跟得肉 → robot.max_relative_target 加大/startup_sync 完成后自然全速；软 → gains kp +10；抖 → kd +0.3。另：夹爪行程会在 joint_limits 占位值处饱和（bench ±170°），标定限位并回写前不要当方向/增益问题排查。
6. **摆渡船往返验证**
   Ctrl-C 退出 teleoperate → `conda run -n maker-arm python tools/switch_protocol.py --ids 3,4,5,6,7 --to private` → 断电重启 → `conda run -n maker-arm python tools/scan_bus.py --max-id 8`（预期：3~7 全在线）。
7. **收尾**：把调优后的 gains/方向回写进两份 json；台架验收完成，整臂装配后用 maker_arm_lerobot.json 重走本 runbook（ids 换 1~7）。

已知项：MIT 模式 set_zero 持久性未验证——若标定跨断电丢失，改为在私有态用 tools/set_zero.py 标零（工作流本就推荐）。

> 注（2026-08-07 起现役=02 号臂）：J2/J3（shoulder_lift/elbow_flex）为 RS02 电机，lerobot 侧型号按协议文档数据暂定 **O1**（±17Nm/±44rad/s）。装臂后必须实测核对（嫌疑：上游 O 系列与 RS 系列编号错位，O2=±20Nm/±33 是另一档）——方法：J2 用 mit_spin 定速转，对比 MIT Present_Velocity 与私有协议读数；若相差 44/33 倍即型号选错。
