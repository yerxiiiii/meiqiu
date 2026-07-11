# Bird_ws — 启动说明

实机前请先启动 `sim2real`，手柄 **Start** 站立（`fsm_state=5`）。  
同一时刻只运行**一个**占 ZED 或脖子的视觉程序；勿用 **Ctrl+Z** 挂起进程。

---

## 手势识别（0~5 + 脸跟踪 + 动作）

**功能**：ZED + MediaPipe 识别手掌手势 0~5；默认**脸部跟踪**（脖子跟人脸）；手势 1~4 稳定约 2s 触发机器人动作。

| 手势 | 作用 |
|------|------|
| 0 | 急停；持续按住约 5s 退出程序 |
| 1 | 撒娇扭腰（仅 `waist_yaw`） |
| 2 | 抬手 |
| 3 | 挥双手 |
| 4 | 踢球 |
| 5 | 稳定 **8s** → 自动进入「手部跟踪」（此时关闭脸跟踪） |

```bash
cd ~/Bird_ws/hand_identify
./start_gesture_recognition.sh
```

- **Esc** 退出窗口；**Ctrl+C** 结束进程  
- ZED 被占用：`pkill -f zed_gesture_recognition.py` 后加 `--force` 再启  
- 仅识别不控机：`./start_gesture_recognition.sh --preview`

---

## 手部跟踪（底盘跟手）

**功能**：手掌在画面左右偏移 → 底盘转向（`/cmd_vel` `angular.z`）；识别**手势 5** 时再做前后距离保持（`linear.x`）。默认**无 GUI**。

- 与手势识别可**自动切换**：手势里比 5 并保持 **8s** 进入本模式；跟手中 **8s** 没有五指则回到手势识别  
- 手柄有输入时约 5s 内不发 `/cmd_vel`（避免盖 teleop）  
- 不要与手势识别**同时**各开一份

```bash
cd ~/Bird_ws/hand_identify
./start_hand_tracking.sh
```

- 调试画面：`./start_hand_tracking.sh --gui`  
- 只打印不发速：`./start_hand_tracking.sh --dry-run`  
- **Ctrl+C** 退出，退出前会发零速

---

## 单独脸部跟踪

**功能**：仅做人脸检测与脖子跟随，下发 `/pi_plus_absolute`（`head_yaw` / `head_pitch`）。  
独立进程，用 OpenCV 读 `/dev/video0`，**不含**手势与底盘控制。  
与手势程序里的内嵌脸跟踪**控制律相同**，二选一即可。

```bash
 
```

- 后台无窗：`python3 locate_face.py --no-gui`  
- **Esc** / **q** 退出，脖子回中

---

## 右臂 5-DOF IK（键盘末端）

**功能**：键盘控制右臂末端位姿 → 数值 IK → 实机关节（`torso_link` → `r_wrist_link`，含夹爪）。与上述视觉功能独立。

- 实机：Start 站立（FSM=5）后运行；**W/S/A/D/Q/E** 平移，**空格** 回站立默认姿，**F** 夹爪  
- 仅解算、不发实机：加 `--sim-only`

```bash
cd ~/Bird_ws/5_DOF_ARM_IK/scripts
./run_keyboard_demo.sh
```

---

## 对照

| 功能 | 启动命令 | 脸 | 手势 | 底盘 | 手臂 |
|------|----------|----|------|------|------|
| 手势识别 | `start_gesture_recognition.sh` | ✓ | 0~5 | — | 动作 1~4 |
| 手部跟踪 | `start_hand_tracking.sh` | — | 5（距离） | ✓ | — |
| 单独脸跟踪 | `locate_face.py` | ✓ | — | — | — |
| 右臂 IK | `run_keyboard_demo.sh` | — | — | — | ✓ |

更多参数见：`hand_identify/`、`locate_face/`、`5_DOF_ARM_IK/` 下各 `README.m d`。
