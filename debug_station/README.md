# Moon Debug Station（调试上位机）

**与功能代码隔离的只读调试台。**

- 目录独立：`moon/debug_station/`
- **不**修改 / 不 import `uwb_follow.py` 控制逻辑
- **不**打开 UWB 串口（避免与跟随抢口）
- **不**发布 `/cmd_vel`、`/joy_msg`
- 数据来源：已有 ROS 话题 + 跟随会话日志尾读 + 视觉节点已有 FPV `:8080`

```
moon/
├── uwb_follow.py          # 功能：跟随（控腿）
├── vision/                # 功能：感知
└── debug_station/         # 调试：只读观察 ← 本目录
```

## 启动

```bash
# 终端 A（可选）：视觉 FPV + /moon/obstacle
source /home/nvidia/sim2real/install/setup.bash
python3 /home/nvidia/moon/vision/zed_obstacle_node.py

# 终端 B（可选）：UWB 跟随（写 logs + 发 cmd_vel）
source /home/nvidia/sim2real/install/setup.bash
python3 /home/nvidia/moon/uwb_follow.py

# 终端 C：调试台
source /home/nvidia/sim2real/install/setup.bash
python3 /home/nvidia/moon/debug_station/server.py
```

浏览器打开：`http://<机器人IP>:8090/`

笔记本 SSH 转发：

```bash
ssh -L 8090:localhost:8090 -L 8080:localhost:8080 nvidia@<机器人IP>
# 然后打开 http://localhost:8090/
```

仅看日志、无 ROS：

```bash
python3 /home/nvidia/moon/debug_station/server.py --no-ros
```

## 页面内容

| 区域 | 数据 |
|------|------|
| 感知 / 决策 / 执行 三色灯 | 话题新鲜度 + 日志 MOTION + fsm |
| FPV | `http://主机:8080/stream.mjpg`（视觉节点） |
| 障碍三区 | `/moon/obstacle` |
| UWB | 最新 `moon/logs/uwb_follow_*.log` 中的 MOTION |
| 决策 | 同上日志中的 fwd/rot/gate/soft |
| 执行 | `/cmd_vel` `/fsm_state` `/joy_msg` |
| 日志 | 会话文件 tail |

深度伪彩整图：后续 P1（当前用三区距离代替，避免改功能节点）。

## API

- `GET /api/snapshot` — 全量状态 JSON
- `GET /api/health` — 三层灯
- `GET /api/logs?n=120` — 日志尾

## ZED Mini 开关

页面 FPV 卡片上有 **开启 ZED / 关闭 ZED**：

- 开启：由调试台拉起 `vision/zed_obstacle_node.py`（本机 `:8080` FPV + `/moon/obstacle`）
- 页面画面走 **同端口代理** `http://…:8090/fpv/stream.mjpg`（SSH 只转 8090 即可看到图）
- 关闭：结束该进程（含外部同名残留）
- 节点 stdout → `moon/logs/zed_obstacle_debug_station.log`
- **仍不发控腿指令**；只管理视觉进程

API：`POST /api/zed/start` · `POST /api/zed/stop` · `GET /api/zed/status`

## 注意

- 跟随未启动时，UWB/决策灯会红或黄（正常）
- 视觉未启动时 FPV 黑屏、障碍 stale（正常）；可用页面按钮开启
- 本服务崩溃不影响跟随；关掉 ZED 只停视觉进程
- **改完调试台代码后需重启** `server.py` 再刷新浏览器
