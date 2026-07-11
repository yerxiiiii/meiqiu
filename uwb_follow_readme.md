# UWB 自主跟随系统使用说明

## 系统概述

`uwb_follow.py` 是高擎 Pi Plus 机器人的 UWB 自主跟随控制程序。它读取 UWB 主站串口数据，计算目标距离与偏航角，向机器人注入虚拟手柄信号实现自动追踪，并支持 USB 热插拔与开机自启。

当前运行环境：NVIDIA Orin（用户 `nvidia`，工作目录 `/home/nvidia/moon`）。

### 相关文件

| 文件 | 作用 |
|------|------|
| `uwb_follow.py` | 跟随主程序（唯一控制逻辑） |
| `uwb-follow.service` | systemd 开机自启 |
| `uwb_detector.py` | 串口热插拔/权限诊断工具（不发控制指令） |
| `uwb_follow_readme.md` | 本说明 |

> 诊断时不要与 `uwb_follow.py` 同时开串口；也不要与 `keyboard_teleop.py` 同时运行（会在不同话题层叠指令）。

---

## 工作原理

### 控制管线

```
UWB 主站 (动态探测 /dev/ttyUSB*，115200)
    ↓  解析 ###1.9 协议
uwb_follow.py
    ├─ /joy_input (sensor_msgs/Joy)  → 前后/旋转 + LT/RT 使能
    └─ /joy_msg   (sim2real_msg/Joy) → 首次有效信号时发 LB 边沿，切入 RUNNING
         ↓
humanoid_driver / joy_teleop / sim2real_master_node
         ↓
电机驱动
```

现场两台 UWB（671 / 672），**672 为主站（跟随用）**。脚本不写死 `ttyUSB0`：优先选有 `###1.9` 数据的口；候选排序为 CP210x 优先，且 `ttyUSB1` 先于 `ttyUSB0`（避免误连空口或与 IMU 抢口）。

### 三层控制状态

| 状态 | 条件 | 机器人行为 | 手柄 |
|------|------|-----------|------|
| **等待设备** | 无 `/dev/ttyUSB*` | 原地静止 | ✅ 完全可用 |
| **信号丢失** | 已连接但 > 1.2s 无更新 | 原地静止 | ✅ 自动接管 |
| **跟随中** | 有有效 UWB 数据 | 自动跟随 | ❌ 摇杆被覆盖 |

### 热插拔

- **插入 UWB USB** → 自动探测并连接，进入跟随
- **拔出** → 停发 `/joy_input`，手柄恢复，约 2s 后重新探测
- **再插入** → 自动重连

### RUNNING 状态

sim2real 仅在 **RUNNING** 时接收速度指令。默认 `AUTO_ENTER_RUNNING = True`：首次收到有效 UWB 数据后，经 `/joy_msg` 发送一次 LB 上升沿（`STANDBY → RUNNING`）。若关闭该开关，需手动用手柄 LB 切入。

跟随发布时会保持 `axes[2]/axes[5] = 1.0`（LT/RT 使能），与键盘遥控一致。

---

## 跟随参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `TARGET_DISTANCE` | 70 cm | 理想跟随距离 |
| `DISTANCE_DEADZONE` | ±10 cm | 距离死区（约 60~80 cm 不前后动） |
| `ANGLE_DEADZONE` | ±6° | 角度死区 |
| `FORWARD_MIN / MAX` | 0.15 / 0.6 | 前后摇杆最小/最大推幅 |
| `ROTATE_MIN / MAX` | 0.30 / 0.8 | 旋转摇杆最小/最大推幅 |
| `ANGLE_SCALE` | 45° | 角度满量程 |
| `DISTANCE_SCALE` | 100 cm | 距离满量程 |
| `UWB_TIMEOUT` | 1.2 s | 信号丢失判定 |
| `CONTROL_RATE` | 50 Hz | 发布频率 |
| `SERIAL_RETRY_INTERVAL` | 2.0 s | 未找到设备/断线后重试间隔 |
| `UWB_PROBE_SECONDS` | 0.8 s | 探测口是否有 `###1.9` 的等待时间 |
| `AUTO_ENTER_RUNNING` | `True` | 首次有效信号时自动经 `/joy_msg` 发 LB |

修改参数后重启脚本或服务即可生效。

---

## 手动运行

### 前置条件

1. UWB 主站（672）已通过 USB 转串口接入
2. ROS Master / 机器人控制栈已启动
3. 串口权限（首次执行一次）：
   ```bash
   sudo usermod -aG dialout nvidia
   # 重新登录后生效；临时赋权示例：
   sudo chmod 666 /dev/ttyUSB*
   ```

### 启动

```bash
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash
python3 /home/nvidia/moon/uwb_follow.py
```

### 退出

`Ctrl+C` → 停止发布，手柄恢复控制。

---

## 开机自启（systemd）

### 安装

```bash
sudo cp /home/nvidia/moon/uwb-follow.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable uwb-follow.service
sudo systemctl start uwb-follow.service
```

### 管理

```bash
sudo systemctl status uwb-follow.service
sudo journalctl -u uwb-follow.service -f
sudo systemctl stop uwb-follow.service
sudo systemctl disable uwb-follow.service
sudo systemctl restart uwb-follow.service
```

### 注意

- 服务依赖 ROS 环境（`ExecStart` 内会 `source` setup.bash）
- UWB 未插入时安静等待，不影响手柄
- 崩溃后约 5 秒由 systemd 自动重启

---

## 串口诊断（可选）

仅排查插拔/权限时使用：

```bash
python3 /home/nvidia/moon/uwb_detector.py
```

会标出疑似 CP210x（UWB）设备并检查读写权限。**不要与跟随脚本同时占用同一串口。**

---

## 日志说明

```
[WAITING]       灰色   未检测到 ttyUSB，手柄控制中
[DETECT]        绿/黄  找到有 ###1.9 的主站口 / 暂用候选口
[CONNECTED]     绿色   串口已连接，进入跟随
[NOTE]          黄色   RUNNING 切入方式提示
[RUNNING]       黄色   已通过 /joy_msg 发送 LB 边沿
[FOLLOW ]       绿色   正常跟随（距离/角度/xyz/摇杆）
[ANOMALY]       黄色   Y 轴退化异常，旋转冻结、前后限幅
[STANDBY]       灰色   信号丢失，手柄已接管
[DISCONNECTED]  红色   USB 拔出，手柄已恢复
[ERROR]         红色   无法打开串口，稍后重试
```

---

## 故障排查

| 问题 | 处理 |
|------|------|
| `ModuleNotFoundError: No module named 'rospy'` | 先 `source .../install/setup.bash` |
| `无法打开串口` | `sudo chmod 666 /dev/ttyUSB*`，或加入 `dialout` 后重新登录；确认未与 detector/其他进程抢口 |
| 连错口 / 无数据 | 看 `[DETECT]` 日志；确认 672 主站在发 `###1.9`；可用 `uwb_detector.py` 看设备列表 |
| 机器人不动 | 看是否有 `[RUNNING]`；确认策略已在 RUNNING；检查 `/joy_input` 是否在发 |
| 只转不走 / 使能异常 | 确认 `axes[2]/[5]`（LT/RT）为 1.0；检查 joy_teleop / master 管线 |
| 旋转无反应 | 角度可能在 ±6° 死区内，拉大横向偏移 |
| 与手柄抢控制 | 跟随激活时属预期；拔 UWB 或停脚本即可恢复 |
| 与键盘遥控冲突 | 不要同时运行 `keyboard_teleop.py` |
| 与 IMU 抢 `ttyUSB0` | Orin launch 可能把 yesense 指到 `ttyUSB0`；跟随脚本会优先有数据的口，插拔后注意设备号是否对调 |
