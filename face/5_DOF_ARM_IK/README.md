# 5_DOF_ARM_IK — PiPlus 右臂逆运动学

右手 **5 自由度**数值 IK（`torso_link` → `r_wrist_link`，不含夹爪关节参与 IK）。

## 目录

```
5_DOF_ARM_IK/
├── arm_ik/              # FK / IK / URDF / 实机桥接 robot_bridge.py
├── config/right_arm.yaml
├── urdf/PiPlusPro_S_12L10A2G2H1W_ZedMini_260322.urdf
├── scripts/
│   ├── keyboard_teleop_demo.py
│   ├── run_keyboard_demo.sh
│   └── inspect_urdf.py
└── requirements.txt
```

## 安装

```bash
cd ~/Bird_ws/5_DOF_ARM_IK
pip install -r requirements.txt
```

实机模式还需 ROS Noetic + sim2real（`run_keyboard_demo.sh` 会自动 source）。

## 键盘末端 Demo

在**本机终端**运行（需交互式 TTY）：

```bash
# 默认：控制实机（需 sim2real + FSM=EXEC_DEFAULT）
./scripts/run_keyboard_demo.sh

# 仅 IK，不发实机
./scripts/run_keyboard_demo.sh --sim-only

# 联调：连 ROS 但不发布
./scripts/run_keyboard_demo.sh --dry-run
```

### 前提（实机 `--robot`）

1. `sim2real_master` 已运行
2. 手柄 **Start** 进入站立（`fsm_state=5`，EXEC_DEFAULT）
3. 本程序**只控右臂** 5 关节 + 夹爪，不发布左臂

### IK 参考原点

- 键盘末端增量以 **`standing_home_q`**（Start 站立时右臂默认关节角）为零点，**不是** URDF 全零 `q=0`（手臂伸直）构型。
- 在 `config/right_arm.yaml` 中按实机关节反馈微调；`python3 scripts/inspect_urdf.py` 可对比伸直 vs 站立 FK。
- 实机启动只 **保持当前关节** 不下发；首次按键才从站立原点解 IK 并运动。

### 操作（平移方向在 `config/right_arm.yaml` → `teleop` 标定）

| 功能 | 按键 |
|------|------|
| 前 / 后 | `W` / `S` 或 ↑ / ↓ |
| 左 / 右 | `A` / `D` 或 ← / → |
| 上 / 下 | `Q` / `E` 或 PgUp / PgDn |
| **夹爪开/合** | **`F`** |
| Roll ±（绕 X） | `I` / `K` 或 `7` / `8` |
| Pitch ±（绕 Y） | `J` / `L` 或 `4` / `5` |
| Yaw ±（绕 Z） | `U` / `O` 或 `1` / `2` |
| 一键回站立默认位 | 空格 |
| 帮助 | `H` |
| 退出 | `Esc` |

腰 yaw 非零：`--q-waist <rad>`。

性能：IK 使用解析雅可比 + 早停（约数 ms/次）；实机斜坡 `joint_ramp_rad_per_sec: 7`、下发 50Hz。仍卡顿时可加大 `goal_max_step_rad` 或 `--ik-mode position`。

若某键方向仍不对，改 `teleop.forward/back/left/right/up/down` 六个单位向量（实机标定表见 yaml 注释）。

```bash
python3 scripts/keyboard_teleop_demo.py --pos-step 0.005 --rot-step-deg 3 --ik-mode tool_z
python3 scripts/keyboard_teleop_demo.py --robot --no-fsm   # 跳过 FSM 检查（慎用）
```

默认 `arm_backend: absolute`，经 `/pi_plus_absolute` 按关节名只下发右臂。可选 `--develop` 使用 lowlevel 双臂轨迹。

## 运动学链

| 关节 | 轴 |
|------|-----|
| `r_shoulder_pitch_joint` | Y |
| `r_shoulder_roll_joint` | X |
| `r_upper_arm_joint` | Z |
| `r_elbow_joint` | Y |
| `r_wrist_joint` | Z |

## Python API

```python
from arm_ik import RightArmIKSolver, load_standing_home_q

solver = RightArmIKSolver.from_config("config/right_arm.yaml")
q_home = load_standing_home_q("config/right_arm.yaml")
T_home = solver.fk(q_home)
res = solver.ik_position_orientation(T_home, q_seed=q_home)
```

目标在 `base_link` 系时用 `solver.ik_in_base_frame(T, q_seed=q_home, q_waist=0.0)`。

## 检查 URDF

```bash
python3 scripts/inspect_urdf.py
```
