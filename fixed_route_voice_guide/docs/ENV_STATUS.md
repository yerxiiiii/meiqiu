# 环境与 SDK 核对（本机实测）

更新时间: 2026-07-10

## 控制栈（里程碑 1–2）

| 项 | 状态 | 说明 |
|----|------|------|
| ROS1 Noetic | OK | `/opt/ros/noetic` |
| sim2real | OK | `joy_control_pi_plus_orin.launch` 常驻 |
| `/cmd_vel` | OK | `geometry_msgs/Twist`，订阅方 `sim2real_master_node` |
| `/fsm_state` | OK | `std_msgs/Int32`；当前常见可走值 **5**（ExecDefault） |
| `/joy_msg` | OK | `sim2real_msg/Joy`（含 lt/rt/lb，用于解锁检测） |
| `/imu/data` | OK | yesense |
| 步态策略 | OK | AMP walk：`walk/pi_plus_amp_1110_policy_1.rknn`，`control_mode: Soccer`，RL 50Hz / PD 1000Hz |
| 机型 | OK | PiPlusPro `S-12L8A0G2H0W` |

**不是**通用 SLAM 导航；带路是预制时间片路线 + 底层 RL 步态跟 `/cmd_vel`。

## 手柄与 demo 顺序（已按此实现）

```
sim2real 运行
    -> 启动 guide_demo（先不抢 /cmd_vel）
    -> 手柄 LT+RT+LB 解锁
    -> /fsm_state 进入 ready 集合 -> ARMED
    -> 语音/文本 go_to 才执行
    -> 运动前 takeover 暂停 /joy_teleop
    -> 结束/Ctrl+C 零速并尝试恢复手柄
```

## 感知 / 语音 SDK（里程碑 3–4）

| 项 | 状态 | 说明 |
|----|------|------|
| JetPack | OK | R35.6.0 (Orin) |
| ZED SDK | OK | **5.0.2**，`import pyzed.sl` 成功 |
| OpenCV | OK | 4.12.0 |
| Vosk | 未装 | 需 `pip install vosk` + 中文小模型 |
| sounddevice/PyAudio | 未核 | 板载 APE 声卡在；建议外接 USB 麦再测 |
| pyserial | 未装 | UWB 串口前需安装 |
| 串口 | 有设备 | `ttyUSB0/1`=CP210x（候选 UWB），`ttyACM*`=Livelybot 通信板 |
| `/dev/video0/1` | 有 | 需确认是否 ZED Mini 节点 |

## 建议下一步

1. 真机：`python3 scripts/guide_demo_node.py --smoke-move 1.5`（先解锁）
2. 若一直 `WAIT_ARM`：`rostopic echo /fsm_state`，把值写入 `config/safety.yaml` 的 `ready_fsm_states`
3. 里程碑 3：装 Vosk；里程碑 4：ZED 深度 ROI + UWB 串口协议确认
