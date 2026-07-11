# amp_right_hold — 右臂固定 + 左臂摆动行走策略

## 1. 标准命名（决策层请用此字符串）

| 字段 | 标准值 | 说明 |
|------|--------|------|
| **policy name** | `amp_right_hold` | 注册名 / `changePolicy` 入参 / 日志里的 `policy name` |
| algorithm | `amp` | 复用 AMP 推理与观测管线，**不是**新算法类 |
| motor_group | `all-amp-rhold` | 与原版 `all-amp` 隔离，避免控组混淆 |
| 模型文件 | `walk/pi_plus_amp_1110_policy_1.rknn`（及同名 `.trt`） | 与 `amp` 同一套权重 |
| 配置文件 | `config/walk/amp_pi_plus_20dof_right_hold.yaml` | 本策略唯一行为定义处 |

**禁止**用模糊别名（如 `amp2`、`right_hold`）调用；决策层、日志检索、文档一律使用 **`amp_right_hold`**。

---

## 2. 行为定义

| 部位 | 行为 | 实现方式 |
|------|------|----------|
| 双腿 | 正常 AMP 行走 | 网络输出 × `action_scale` |
| 左臂 | 实时摆臂 | 同 AMP，`action_scales=1` |
| 右臂 | 固定 hold 姿态 | `action_scales=0`，停在本策略 `urdf_offset` |

当前右臂 hold（策略顺序，已实机验证）：

- `r_shoulder_pitch` = **2.735**
- `r_shoulder_roll`  = **1.57**
- `r_upper_arm`      = **0.00**
- `r_elbow`          = **0.00**（伸直）

微调只改 `amp_pi_plus_20dof_right_hold.yaml` 里上述四个 `urdf_offset`，**不要**改原版 `amp_pi_plus_20dof.yaml`。

---

## 3. 与其它 walk 策略的关系

注册顺序（`pi_plus_22dof_rl_config.yaml`）：

```text
amp  →  amp_right_hold  →  lr  →  footstep
```

| name | 上肢 | 适用 |
|------|------|------|
| `amp` | 双臂都摆 | 默认全身走 |
| **`amp_right_hold`** | **左摆、右固定** | **决策层默认推荐行走**（本需求） |
| `lr` | 双臂 hold（不进策略） | 仅下半身走 |
| `footstep` | 按 footstep 配置 | 落脚点类 |

注意：`getAllBodyWalkPolicy()` 会返回第一个 `motor_group` 含 `"all"` 的 walk（当前是 **`amp`**）。  
决策层若要右臂固定，必须 **显式** 切到 `amp_right_hold`，不要依赖「全身 walk 默认」。

---

## 4. 启动

```bash
# 确保没有残留双 launch
pkill -f 'roslaunch sim2real_master' || true

cd /home/nvidia/sim2real_master-feature-master_and_slave
source ./install/setup.bash
roslaunch sim2real_master joy_control_pi_plus_orin.launch
```

启动成功标志：

```text
rl group resgester: [amp_right_hold] [all-amp-rhold]
```

切到本策略后：

```text
Sim2Real algorithm: [amp], policy name: [amp_right_hold], ...
ctrl group:(all-amp-rhold)
```

---

## 5. 手柄操作（人工验收）

前提：FSM 为 **STANDBY**（非 RUNNING）。

| 操作 | 组合 | 作用 |
|------|------|------|
| 起立 / zero | `LT+RT+Start` | STANDING → STANDBY |
| 切策略 | `LT+RT` + 十字键左右 | 上一个 / 下一个 walk |
| 进行走 | `LT+RT+LB` | STANDBY → RUNNING |
| 退回待机 | `LT+RT+LB` | RUNNING → STANDING → STANDBY |
| 速度 | 左摇杆等 → `/cmd_vel` | 仅 RUNNING 时有效 |

十字键右切换顺序：`amp` → **`amp_right_hold`** → `lr` → `footstep` → …

---

## 6. 决策层如何调用（标准化接口约定）

### 6.1 状态机（必须遵守）

```text
STANDBY  --changePolicy("amp_right_hold")-->  STANDING(回 zero)  -->  STANDBY
STANDBY  --enter RUNNING-->  RUNNING
RUNNING  --发 /cmd_vel-->  行走
RUNNING  --exit-->  STANDBY（再允许换策略）
```

规则：

1. **只在 STANDBY（或 INIT）切换策略**；RUNNING 中不要切 `amp_right_hold`。
2. 切策略后等 standing / zero 轨迹结束，再进 RUNNING。
3. 行走指令只走 **`/cmd_vel`**（`geometry_msgs/Twist`）。
4. 用 **`/fsm_state`** 确认状态后再发下一步。

### 6.2 策略切换（按名字）

运控内部 API（C++）：

```cpp
changePolicy("amp_right_hold");  // 精确名字，禁止空字符串轮询碰运气
```

当前对外主要入口仍是手柄话题 **`/joy_msg`**（`sim2real_msg/Joy`）：  
决策层可复现「LT+RT + dpad」序列完成切换，或在运控侧后续增加专用 service（建议名见下）。

**推荐后续补齐的 ROS 接口（标准化建议，便于决策层）：**

```text
Service:  /sim2real/change_walk_policy
Request:  policy_name: string   # 例: "amp_right_hold"
Response: success: bool, message: string

Service:  /sim2real/set_run_state
Request:  state: string         # "standby" | "running"
Response: success: bool

Topic:    /cmd_vel              # 已有，行走速度
Topic:    /fsm_state            # 已有，状态反馈
```

在 service 落地前，决策层临时方案：

1. 发布 `/joy_msg` 模拟 `LT+RT+Start` → 起立  
2. 在 STANDBY 下模拟 `LT+RT+dpad` 直到日志 / 内部状态为 `amp_right_hold`（或直接扩展运控提供按名切换）  
3. `LT+RT+LB` → RUNNING  
4. 持续发 `/cmd_vel`

### 6.3 速度指令约定

| 字段 | 含义 | 参考限幅（见 yaml Soccer） |
|------|------|---------------------------|
| `linear.x` | 前后 | 约 [-0.8, 0.8]，LT 加速可到 1.5 |
| `linear.y` | 左右 | 约 [-0.5, 0.5] |
| `angular.z` | 转向 | 约 [-1.57, 1.57] |

小死区由 AMP 策略内部滤波处理；决策层避免高频抖动指令。

### 6.4 伪代码（决策层）

```text
function start_walk_right_arm_hold(vx, vy, dyaw):
    assert fsm in {STANDBY, INIT}
    change_walk_policy("amp_right_hold")   # 标准名
    wait_until fsm == STANDBY               # standing 结束
    set_run_state(RUNNING)
    publish cmd_vel(vx, vy, dyaw)

function stop_walk():
    publish cmd_vel(0, 0, 0)
    set_run_state(STANDBY)
```

### 6.5 Moon 项目对接（已实现）

路径：`/home/nvidia/moon/brain/`

| 文件 | 作用 |
|------|------|
| `policy_switch.py` | `ensure_walk_policy(..., "amp_right_hold")`：STANDBY 下 LT+RT+dpad Next，并用 rosout/日志校验 |
| `mode_arbiter.py` | 进入 `UWB_FOLLOW` 时先切 `amp_right_hold`，再发 RUNNING + `/cmd_vel` |

口令「小派我们走」→ 自动使用本策略，无需再手柄手动切。

更完整的决策侧说明与注意项见：  
`/home/nvidia/moon/brain/README.md`（章节 **使用注意**）。

**对接时务必记住：**

1. 运控只开一个 `roslaunch`；先 `LT+RT+Start` 到 **STANDBY** 再让 arbiter 切策略。  
2. **不要**同时跑 `uwb_follow.py` / `uwb-follow.service`（会抢 `/cmd_vel`）。  
3. 成功标志：arbiter `[POLICY] 已切换到 amp_right_hold`；运控 `policy name: [amp_right_hold]`。  
4. 决策层字符串固定为 `amp_right_hold`，不要依赖默认全身 walk（那是 `amp`）。
---

## 7. 关键配置路径

运行时（install）：

```text
install/share/sim2real/config/walk/amp_pi_plus_20dof_right_hold.yaml
install/share/sim2real/config/pi_plus_22dof_config/pi_plus_22dof_rl_config.yaml
```

源码同步副本：

```text
sim2real_beijingfootball/sim2real_master/src/sim2real/config/walk/amp_pi_plus_20dof_right_hold.yaml
zx_workspace/.../src/sim2real/config/walk/amp_pi_plus_20dof_right_hold.yaml
```

改 yaml 后需 **重启** `roslaunch` 才生效（无需重编译；本策略未改 C++）。

---

## 8. 验收清单

- [ ] 日志出现 `policy name: [amp_right_hold]`、`ctrl group:(all-amp-rhold)`
- [ ] RUNNING 时左臂摆动、右臂保持固定角
- [ ] 双腿步态正常、可跟 `/cmd_vel`
- [ ] 切回 `amp` 后双臂恢复摆动（对照）
- [ ] 同时只存在一个 `roslaunch`（避免节点名冲突 / FSM 保护关机）

---

## 9. 变更记录

| 日期 | 内容 |
|------|------|
| 2026-07-11 | 初版：复用 amp 模型；右臂 `action_scales=0` + hold `urdf_offset`；注册名 `amp_right_hold` |
