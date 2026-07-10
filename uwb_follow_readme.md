# UWB 自主跟随系统使用说明

## 系统概述

`uwb_follow.py` 是高擎 Pi Plus 机器人的 UWB 自主跟随控制程序。它通过读取 UWB 测距模块的串口数据，实时计算目标距离和偏航角，向机器人注入虚拟手柄信号实现自动追踪。

当前运行环境：NVIDIA Orin（用户 `nvidia`，工作目录 `/home/nvidia/moon`）。

---

## 工作原理

### 控制管线

```
UWB 模块 (/dev/ttyUSB0, 串口 115200)
    ↓  解析 ###1.9 协议数据
uwb_follow.py (计算前后/旋转摇杆值)
    ↓  发布到 /joy_input (sensor_msgs/Joy)
humanoid_driver (IMU 姿态补偿)
    ↓
joy_teleop (格式转换 + 速度映射)
    ↓
sim2real_master_node (RL 步态策略 → 电机驱动)
```

### 三层控制状态

| 状态 | 条件 | 机器人行为 | 手柄 |
|------|------|-----------|------|
| **等待设备** | UWB USB 未插入 | 原地静止 | ✅ 完全可用 |
| **信号丢失** | USB 已插但信号 > 1.2s 无更新 | 原地静止 | ✅ 自动接管 |
| **跟随中** | UWB 信号正常 | 自动跟随目标 | ❌ 被覆盖 |

### 热插拔支持

- **插入 UWB USB** → 脚本自动检测并连接，进入跟随模式
- **拔出 UWB USB** → 脚本自动检测断开，手柄立即恢复，2 秒后重新探测
- **重新插入** → 自动重新连接，恢复跟随

---

## 跟随参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `TARGET_DISTANCE` | 70 cm | 理想跟随距离 |
| `DISTANCE_DEADZONE` | ±10 cm | 距离死区（60~80cm 内不前后运动） |
| `ANGLE_DEADZONE` | ±6° | 角度死区（偏差小于 6° 不旋转） |
| `FORWARD_MIN / MAX` | 0.15 / 0.6 | 前后摇杆最小/最大推幅 |
| `ROTATE_MIN / MAX` | 0.30 / 0.8 | 旋转摇杆最小/最大推幅 |
| `UWB_TIMEOUT` | 1.2 秒 | 信号丢失判定阈值 |

---

## 手动运行

### 前置条件

1. UWB 模块已通过 USB 转串口线插入机器人
2. 串口权限已配置（首次需执行一次）：
   ```bash
   sudo usermod -aG dialout nvidia
   # 或临时赋权：
   sudo chmod 666 /dev/ttyUSB0
   ```
3. 用手柄按 **LT + RT + LB** 将机器人解锁到 **RUNNING** 状态

### 启动命令

```bash
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash
python3 /home/nvidia/moon/uwb_follow.py
```

### 退出

按 **Ctrl+C**，手柄立即恢复控制。

---

## 开机自启（systemd 服务）

### 安装服务

```bash
# 1. 复制服务文件
sudo cp /home/nvidia/moon/uwb-follow.service /etc/systemd/system/

# 2. 重载 systemd 配置
sudo systemctl daemon-reload

# 3. 启用开机自启
sudo systemctl enable uwb-follow.service

# 4. 立即启动服务
sudo systemctl start uwb-follow.service
```

### 管理命令

```bash
# 查看服务状态
sudo systemctl status uwb-follow.service

# 查看实时日志
sudo journalctl -u uwb-follow.service -f

# 停止服务
sudo systemctl stop uwb-follow.service

# 禁用开机自启
sudo systemctl disable uwb-follow.service

# 重启服务
sudo systemctl restart uwb-follow.service
```

### 注意事项

- 服务会在 ROS Master 启动后自动运行
- 如果 UWB 设备未插入，服务会安静等待，不影响手柄操控
- 如果服务崩溃，systemd 会在 5 秒后自动重启

---

## 日志输出说明

```
[WAITING]       灰色   UWB 设备未检测到，手柄控制中
[CONNECTED]     绿色   UWB 串口已连接，进入跟随模式
[FOLLOW ]       绿色   正常跟随中，显示距离/角度/摇杆值
[ANOMALY]       黄色   Y 轴退化异常，角速度已冻结
[STANDBY]       灰色   UWB 信号丢失，手柄已接管
[DISCONNECTED]  红色   UWB USB 被拔出，手柄已恢复
```

---

## 故障排查

| 问题 | 解决方案 |
|------|---------|
| `ModuleNotFoundError: No module named 'rospy'` | 先运行 `source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash` |
| `无法打开串口` | 运行 `sudo chmod 666 /dev/ttyUSB0`，或将 nvidia 加入 dialout 组后重新登录 |
| 机器人不动 | 确认已用手柄 LT+RT+LB 解锁到 RUNNING |
| 只转不走 | 这不应该发生（已通过 /joy_input 管线解决），请检查日志 |
| 旋转无反应 | 角度偏差可能小于 6° 死区，尝试拉大偏移 |
