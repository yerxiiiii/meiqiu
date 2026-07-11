# Moon Vision — 跟随安全感知

## 放哪里？

**放在 `moon/vision/`，单独进程；不要塞进 `uwb_follow.py`。**

| 方案 | 结论 |
|------|------|
| 全塞进 `uwb_follow.py` | ❌ 串口 + ZED + 控制缠在一起，难测、难复用 |
| 单独顶层仓库（如 `moon_vision`） | 以后多机复用再拆；现在过早 |
| **`moon/vision/` + 独立节点** | ✅ 同仓部署、职责分离、跟随/遥控都能订障碍话题 |

`moon/` = 本机机器人应用集合；`vision/` = 其中的感知子系统。

## 分层

```
┌─────────────────────────────────────────────────────────┐
│  perception（只看世界，不发电机指令）                      │
│  zed_obstacle_node.py                                   │
│    ZED 深度 → 左/中/右距离 → /moon/obstacle              │
└───────────────────────────┬─────────────────────────────┘
                            │ ObstacleState
┌───────────────────────────▼─────────────────────────────┐
│  safety（纯逻辑，无硬件）                                 │
│  safety_gate.py                                         │
│    (期望 fwd/rot, ObstacleState) → (安全 fwd/rot)        │
└───────────────────────────┬─────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────┐
│  behavior（意图）                                        │
│  uwb_follow.py     → UWB 算期望摇杆，再过 safety_gate    │
│  keyboard_teleop   → 以后也可订同一话题做限速（可选）      │
└───────────────────────────┬─────────────────────────────┘
                            │
                     /joy_msg + /cmd_vel
```

**该区分的：**

- 感知 ≠ 跟随：换 RealSense 只改 `zed_obstacle_node`，跟随不动
- 安全门控 ≠ 感知：刹停/绕障规则可单测，不依赖相机
- 意图 ≠ 执行：UWB 只表达“想往哪走”，最终速度由门控裁剪

## 话题

| 话题 | 类型 | 含义 |
|------|------|------|
| `/moon/obstacle` | `Float32MultiArray` | `[left_m, center_m, right_m, forward_cap, rotate_bias, valid]` |

- 距离单位：**米**；无效深度用 `nan`
- `forward_cap`：`0~1`，乘到前进摇杆上
- `rotate_bias`：`-1~1`，中间堵、一侧通时的转向建议（跟随可选用）
- `valid`：`1` 本帧有效，`0` 相机异常

## 运行

```bash
# 终端 1：视觉（需 ZED 已接好）
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash
python3 /home/nvidia/moon/vision/zed_obstacle_node.py

# 终端 2：跟随（会自动订 /moon/obstacle）
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash
python3 /home/nvidia/moon/uwb_follow.py
```

无视觉节点时：跟随仍可跑（`OBSTACLE_REQUIRED=False`）；有视觉超时且要求强制安全时才刹停。

## 文件

| 文件 | 职责 |
|------|------|
| `obstacle_state.py` | 状态结构 + ROS 编解码 |
| `safety_gate.py` | 限速 / 刹停 / 绕障偏置 |
| `zed_obstacle_node.py` | ZED → `/moon/obstacle` |
| `face_obs_node.py` | ZED RGB → `/moon/face`（只感知，不控头） |
| `zed-obstacle.service` | 可选开机自启 |

与 `moon/brain/mode_arbiter.py` 配合：FACE_LOOK 启 `face_obs_node`，UWB_FOLLOW 启 `zed_obstacle_node`（ZED 互斥）。
