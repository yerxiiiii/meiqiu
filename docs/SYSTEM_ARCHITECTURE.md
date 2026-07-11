# 机器人系统控制架构

整机控制两层分工：上层 `moon`（语音 / 感知 / 决策），下层 `sim2real`（运控 / 电机）。  
原则：**感知和语音只上报，`mode_arbiter` 唯一下发执行指令。**

---

## 图 1 — 系统总架构

```mermaid
flowchart TB
  subgraph L1["① 语音层 moon/voice"]
    MIC[麦克风 / 键盘模拟]
    KWS[kws_node / voice_sim]
    MIC --> KWS
    KWS -->|"/moon/voice_cmd<br/>face_look | uwb_follow | stop"| VOICE_OUT
  end

  subgraph L2["② 决策层 moon/brain"]
    ARB[mode_arbiter<br/>中央仲裁 · ~50Hz]
    POL[policy_switch]
    INT[uwb_intent]
    NECK[neck_control]
    CAM[camera_owner]
    FSMG[fsm_guard]
    ARB --> POL
    ARB --> INT
    ARB --> NECK
    ARB --> CAM
    ARB --> FSMG
  end

  subgraph L3["③ 感知层 moon/vision + UWB"]
    FACE[face_obs_node<br/>人脸偏差]
    ZED[zed_obstacle_node<br/>障碍门控]
    UWB[UWB 串口<br/>距离/角度]
    SAFE[safety_gate]
    FACE -->|"/moon/face"| ARB
    ZED -->|"/moon/obstacle"| SAFE
    UWB --> INT
    SAFE --> ARB
  end

  subgraph L4["④ 执行接口 ROS"]
    JOY["/joy_msg"]
    CMD["/cmd_vel"]
    ABS["/pi_plus_absolute"]
  end

  subgraph L5["⑤ 运控层 sim2real_master"]
    MSM[sim2real_master_node<br/>FSM: INIT→STANDING→STANDBY↔RUNNING]
    AMP[AMP / amp_right_hold / lr / footstep]
    MOT[电机 · IMU · OLED]
    MSM --> AMP --> MOT
  end

  VOICE_OUT --> ARB
  CAM -.->|启停| FACE
  CAM -.->|启停| ZED
  POL --> JOY
  ARB --> JOY
  ARB --> CMD
  NECK --> ABS
  JOY --> MSM
  CMD --> MSM
  ABS --> MSM
  MSM -->|"/fsm_state"| FSMG
```

---

## 图 2 — 控制闭环：语音 → 模式 → 感知 → 决策 → 执行

```mermaid
sequenceDiagram
  participant V as 语音
  participant A as mode_arbiter
  participant P as 感知
  participant S as sim2real

  V->>A: /moon/voice_cmd
  Note over A: 切模式 IDLE / FACE_LOOK / UWB_FOLLOW<br/>发布 /moon/mode

  alt FACE_LOOK
    A->>P: 启动 face_obs
    P->>A: /moon/face
    A->>S: /pi_plus_absolute（只转头）
  else UWB_FOLLOW
    A->>S: /joy_msg 切 amp_right_hold（须 STANDBY）
    A->>P: 启动 zed_obstacle
    P->>A: UWB + /moon/obstacle
    A->>S: LT+RT+LB → RUNNING
    A->>S: /joy_msg + /cmd_vel（跟随速度）
  else IDLE / stop
    A->>S: 零速停腿，头回中，释放相机
  end

  S-->>A: /fsm_state + rosout 策略确认
```

---

## 口令映射

| 口令 | 命令 | 模式 | 行为 |
|------|------|------|------|
| 小派看我 | `face_look` | `FACE_LOOK` | 只转头 |
| 小派我们走 | `uwb_follow` | `UWB_FOLLOW` | 切 `amp_right_hold` → UWB 跟随 + 障碍门控 |
| 小派停止 | `stop` | `IDLE` | 停腿、头回中 |

## 分层路径

| 层 | 路径 |
|----|------|
| 语音 | `moon/voice/` |
| 决策 | `moon/brain/` |
| 感知 | `moon/vision/` + UWB |
| 运控 | `sim2real_master` |

## 上电自启

| 服务 | 作用 |
|------|------|
| `moon-kws.service` | 默认开录音设备 + 离线 KWS → `/moon/voice_cmd` |
| `moon-arbiter.service` | 中央决策，订阅口令 |

安装：见 `moon/voice/README.md`「上电默认开麦」。  
**不要**与独立 `uwb-follow.service` 同时 enable（抢串口 / `cmd_vel`）。
