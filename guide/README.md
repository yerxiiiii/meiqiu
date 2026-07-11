# Mini Pi Plus 固定场地语音带路

固定场地预设路线（非 SLAM）。用户说「小派带我去炳胜餐厅」后，离线关键词触发，按 `config/destinations.yaml` 时间片带路。

## 目录

```
guide/
├── guide_demo_node.py      # 状态机 + 路线执行（唯一运动出口）
├── voice_keyword_demo.py   # Vosk/文本 → /guide/voice_command
├── common/motion_io.py     # /joy_msg + /cmd_vel
├── common/safety.py        # 订 /moon/obstacle
├── common/uwb_distance.py  # UWB 掉队等待
├── config/destinations.yaml
├── config/voice_keywords.yaml
├── audio/                  # 可选预制 WAV
└── scripts/
```

## 硬约束

- **同一时刻只允许一个节点发 `/cmd_vel`**。带路前停掉：
  ```bash
  sudo systemctl stop uwb-follow.service
  ```
  不要同时跑 `keyboard_teleop.py` / 手部跟踪。
- 速度上限：`linear.x ≤ 0.12`，`angular.z ≤ 0.25`（见 `motion_io.py` / destinations defaults）。
- Ctrl+C / stop / 障碍 STOP → 零速；退出时恢复 `joy_teleop`。

## 快速验证

```bash
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash

# 1) dry-run（不控机器人）
./scripts/run_dry_run.sh
# 或
python3 guide_demo_node.py --dry-run --text "小派带我去炳胜餐厅"

# 2) 语音命令冒烟（无麦）
python3 voice_keyword_demo.py --text "小派带我去炳胜餐厅"
python3 voice_keyword_demo.py --text "小派停下"

# 3) 实机（机器人已 STANDBY/可进 RUNNING）
sudo systemctl stop uwb-follow.service
# 手柄 LT+RT+LB 进 RUNNING，或加 --enter-running
python3 guide_demo_node.py --text "小派带我去炳胜餐厅" --once
# 带避障 + UWB 掉队：
python3 guide_demo_node.py --text "小派带我去炳胜餐厅" --once \
  --enable-obstacle --enable-uwb
```

另开终端跑 ZED 障碍（可选）：

```bash
python3 /home/nvidia/moon/vision/zed_obstacle_node.py
```

## 语音（Vosk）

```bash
pip3 install vosk sounddevice
cd /home/nvidia/moon/guide/models
wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip
unzip vosk-model-small-cn-0.22.zip

# 终端 A：带路节点
python3 guide_demo_node.py --enable-obstacle

# 终端 B：麦克风
python3 voice_keyword_demo.py --model models/vosk-model-small-cn-0.22
```

话题：`/guide/voice_command`（`std_msgs/String`），内容如 `go_to:bingsheng`、`stop`。

预制音频：把 WAV 放到 `audio/`，文件名与 yaml 中 `audio:` 字段一致；没有文件则只打日志。

## 状态机

`IDLE → ACK → LEAD_TO_DEST → ARRIVED → IDLE`

- 障碍 `forward_cap==0` 或 UWB 过远 → `PAUSED`（零速，段剩余时间保留）
- 障碍清除 / 用户跟上 → 从当前段剩余时间继续

## 现场标定

编辑 `config/destinations.yaml` 中每段 `duration` / `vx` / `wz`。空场先 dry-run 再低速实机。

## 故障排查

| 现象 | 处理 |
|------|------|
| 发了速度但不走 | 确认 FSM 为 RUNNING；看 `/fsm_state` |
| 乱动/抢控制 | 停 `uwb-follow`、确认无其它 teleop |
| 无 `/cmd_vel` | `source` ROS setup；`rostopic list \| grep cmd_vel` |
| Vosk 起不来 | 检查模型路径；先用 `--text` |
| 障碍不生效 | 先起 `zed_obstacle_node`；加 `--enable-obstacle` |
| UWB 无数据 | `python3 /home/nvidia/moon/uwb_detector.py`；勿与 follow 同开串口 |

## 日志

控制台打印 `[STATE]` / `[ROUTE]` / `[OBS]` / `[UWB]`。UWB 跟随旧日志仍在 `/home/nvidia/moon/logs/`。
