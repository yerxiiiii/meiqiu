# Moon Brain — 中央决策

## 原则

感知 / 语音 **只上报**；[`mode_arbiter.py`](mode_arbiter.py) **唯一下发** `/joy_msg`、`/cmd_vel`、`/pi_plus_absolute`。

## 模式

| 口令 | `/moon/voice_cmd` | 模式 | 行为 |
|------|-------------------|------|------|
| 小派看我 | `face_look` | FACE_LOOK | 只转头 |
| 小派我们走 | `uwb_follow` | UWB_FOLLOW | 切 **`amp_right_hold`** → 确认 RUNNING → 软启动跟随 |
| 小派停止 | `stop` | IDLE | 停腿，头回中 |

跟随固定 walk 策略：`amp_right_hold`。  
障碍门控默认 **关**（与 `uwb_follow.py` 对齐）；联调加 `--enable-obstacle-gate`。  
`fsm=8`（PROTECTION_SHUTDOWN）立即零速并退出跟随。

## 运行（干跑）

```bash
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash

# 终端 1
python3 /home/nvidia/moon/brain/mode_arbiter.py --dry-run --no-camera-manage

# 终端 2
python3 /home/nvidia/moon/voice/voice_sim.py
```

## 运行（真机）

```bash
# 必须：停掉独立跟随，避免双写 /cmd_vel
sudo systemctl stop uwb-follow.service
pkill -f uwb_follow.py || true

# 机器人先起立到 STANDBY，再启 arbiter
python3 /home/nvidia/moon/brain/mode_arbiter.py
# 避障联调：
# python3 /home/nvidia/moon/brain/mode_arbiter.py --enable-obstacle-gate
```

## 文件

| 文件 | 职责 |
|------|------|
| `modes.py` | 模式与话题契约 |
| `policy_switch.py` | 切运控 walk（`amp_right_hold`） |
| `fsm_guard.py` | `/fsm_state` + running 日志监护 |
| `process_mutex.py` | 与 `uwb_follow` 双写检测 |
| `uwb_intent.py` | UWB 解析 + 保守跟随意图 |
| `neck_control.py` | 脸偏差 → 脖子 |
| `camera_owner.py` | ZED 占用互斥 |
| `joy_monitor.py` | 手柄让路 |
| `mode_arbiter.py` | 决策主节点 |
