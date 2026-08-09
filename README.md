# maker-arm

RobStride 电机自研机械臂（**6 关节 + 1 夹爪**，夹爪为 RS00 @ID7，作普通 MIT 第 7 关节——夹持力 = kp×位置误差，kp 小即柔性限力）纯 Python SDK。协议同源 EDULITE A3（RobStride 私有 CAN @1Mbps）。支持按关节混装型号：YAML 里每关节 `model:` 字段（RS00/RS02，默认 RS00）决定 T/V 映射表。现役为 maker-arm02（J2/J3=RS02，其余 RS00）：`configs/maker_arm.yaml` 即各工具默认配置。设计文档：makermods 仓库 `docs/superpowers/specs/2026-08-04-maker-arm-sdk-design.md`。

## 安装

    conda create -y -n maker-arm python=3.11 && conda run -n maker-arm pip install -e ".[dev]"

## 分层

transport(socketcan/at/mock) → protocol(纯函数) → motor → arm(状态机+200Hz 控制循环) → tools/examples

## 快速开始（10 行遥操内核）

    from maker_arm import Arm
    arm = Arm.from_yaml("configs/maker_arm_6dof.yaml", backend="socketcan", channel="can0")
    arm.connect(); arm.enable()
    arm.set_joint_targets([0.0]*6)   # rad；200Hz 循环限速平滑跟随
    arm.disable(); arm.disconnect()

## 真机 bring-up 序列（按序执行，勿跳步）

1. `python tools/scan_bus.py` — 7 个 ID（6 关节+夹爪）都在线？
2. `python tools/monitor.py` — 手推关节，方向/数值对？（据此填 configs 的 direction）
3. 单电机台架 `python examples/03_sine_wave.py --joint N` 
4. `python tools/set_zero.py` — 摆零位姿态标零
> ⚠️ 交互式工具（monitor 限位采集、set_zero、示例）请用环境内 python 直接跑（如 `~/miniconda3/envs/maker-arm/bin/python tools/monitor.py`）——`conda run` 不传递终端 stdin（Enter/Ctrl-C 行为异常），monitor 会自动降级为只读监视。

5. 量限位：标零后再跑 `python tools/monitor.py`，逐关节手推到两端极限，屏幕实时显示 min/max；按 **Enter** 自动把回退后的 lo/hi 写入 `configs/maker_arm_6dof.yaml`（保留注释，`.bak` 备份，写后校验失败自动回滚），原始记录存 `configs/limits_capture.json`；没推过的关节会被跳过并警告。Ctrl-C 退出不写。之后 `python examples/02_enable_hold.py` 调 kp/kd

> 注：mode≠2 健康检查带 25ms 持续性容忍（连续 5 拍才判故障），使能瞬间的陈旧反馈竞态不会误报；若仍报"模式异常"即是真没进运控态，查该电机使能应答。

6. `python tools/calib_star_map.py --star-port /dev/ttyUSBx` → `python examples/04_teleop_star.py --star-port /dev/ttyUSBx`

## SLCAN 棒（与 metal 臂同款 CANable 类适配器）

插上后一条命令挂成 can0，SDK 走默认 socketcan 后端，零代码：

    sudo bash scripts/setup_slcan.sh

（默认 /dev/ttyACM0 → can0 @1Mbps；拔插后重跑即可。串口名/接口名可作参数传入。）

## 双后端对比

    python tools/bench_backend.py --backend socketcan --channel can0
    python tools/bench_backend.py --backend at --port /dev/ttyUSB0

## 安全设计

- 使能瞬间目标=当前位置（首帧保护）；目标只能以 max_velocity 限速趋近（防飞车）
- 软限位内缩 limit_margin；主机侧反馈超时→FAULT 泄力；电机侧 CAN_TIMEOUT=200ms（进程崩溃电机自动泄力）
- 故障码翻译成中文大声报告，绝不静默丢弃

## 离线测试

    conda run -n maker-arm pytest -q                          # 协议/传输/电机/机械臂单测
    sudo modprobe vcan && sudo ip link add dev vcan0 type vcan; sudo ip link set up vcan0
    conda run -n maker-arm pytest tests/test_vcan_integration.py -q   # 无硬件全链路

## lerobot 遥操

接入 lerobot（RobStride MIT 协议 + 上游 RobstrideMotorsBus）见 `docs/LEROBOT_BRINGUP.md`；协议摆渡工具 `tools/switch_protocol.py`，看门狗固化 `tools/persist_can_timeout.py`。
