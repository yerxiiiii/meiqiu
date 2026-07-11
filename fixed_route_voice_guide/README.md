# Mini Pi Plus 固定场地语音带路与跟随

路径: `/home/nvidia/moon/fixed_route_voice_guide`

## 正确启动顺序

1. 确认 sim2real 已运行（本机通常开机自启）
2. **先启动 demo**（此时还不抢 `/cmd_vel`）
3. 手柄 **LT + RT + LB** 解锁到可走
4. 看到日志 `ARMED` 后，才会执行带路 / smoke-move
5. 运动开始前才会暂时暂停 `/joy_teleop`，避免零速覆盖

```bash
source /opt/ros/noetic/setup.bash
source /home/nvidia/sim2real/install/setup.bash
cd /home/nvidia/moon/fixed_route_voice_guide

# 逻辑验证（不动机器人）
python3 scripts/guide_demo_node.py --dry-run --fast-dry-run --text "小派带我去炳胜餐厅"

# 真机短测：启动后按 LT+RT+LB，等 ARMED，再前进 1.5s
python3 scripts/guide_demo_node.py --smoke-move 1.5

# 真机完整路线
python3 scripts/guide_demo_node.py --text "小派带我去炳胜餐厅"
```

若解锁后一直 `WAIT_ARM`：

```bash
rostopic echo /fsm_state
# 把实际值写进 config/safety.yaml 的 ready_fsm_states
# 或: --ready-fsm 5,6
```

环境与 SDK 核对见 [docs/ENV_STATUS.md](docs/ENV_STATUS.md)。

## 目录

| 路径 | 说明 |
|------|------|
| `config/destinations.yaml` | 炳胜餐厅动作段 |
| `config/safety.yaml` | 速度上限、可走 fsm |
| `scripts/guide_demo_node.py` | 带路执行器 |
| `scripts/voice_keyword_demo.py` | 关键词 -> `/guide/voice_command` |

## 注意

- dry-run 日志带 `[DRY-RUN]`，机器人不会动
- 初期速度: `linear.x <= 0.12`, `angular.z <= 0.25`
- 路线按时长执行，需现场标定
