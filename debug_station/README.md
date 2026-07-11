# Moon Debug Station（语音链路测试上位机）

**与功能代码隔离的测试台**：语音关键词 → 模式切换 → IMU/UWB 多模态跟随。

- 目录：`moon/debug_station/`
- **不**修改跟随/视觉功能代码
- **不**打开 UWB 串口
- 终端可发布 `/moon/voice_cmd`、`/guide/voice_command`（白名单载荷）
- 不发 `/cmd_vel`、`/joy_msg`（除非你在终端手动跑其它脚本）

## 启动

```bash
source /home/nvidia/sim2real/install/setup.bash

# 终端 A：语音栈（KWS + guide + arbiter，含麦克监视 :8091）
bash /home/nvidia/moon/fixed_route_voice_guide/scripts/start_voice_stack.sh run

# 终端 B：测试上位机
python3 /home/nvidia/moon/debug_station/server.py
```

浏览器：`http://<机器人IP>:8090/`

SSH 转发：

```bash
ssh -L 8090:localhost:8090 -L 8091:localhost:8091 nvidia@<机器人IP>
# http://localhost:8090/
```

无 ROS 时仅看日志：

```bash
python3 /home/nvidia/moon/debug_station/server.py --no-ros
```

## 页面四大块（你要求的）

| 区域 | 说明 |
|------|------|
| **命令终端** | 输入 `voice uwb_follow`、`guide 小派跟我走`、白名单 shell；历史输出滚动 |
| **IMU** | 订阅 `/imu/data`：roll/pitch/yaw、加速度、角速度 |
| **流程状态** | 四段链路：麦克风开/关 → KWS → mode_arbiter → 跟随反应；含 `/moon/mode`、`/guide/state` |
| **常用命令** | `commands.yaml` 配置的一键按钮（模式切换、语音口令、终端片段、后台启动） |

另有：进程灯、UWB/障碍、cmd_vel/fsm、可选 FPV。

## API

- `GET /api/snapshot` — 全量状态（含 IMU、voice_chain、quick_buttons）
- `POST /api/cmd` — `{"cmd": "voice stop"}`
- `POST /api/cmd/action` — 快捷按钮 action JSON
- `GET /api/terminal?n=100` — 终端历史
- `GET /api/logs?n=120` — uwb_follow 日志尾

## 配置

常用命令编辑：`debug_station/commands.yaml`

## 注意

- 麦克开/关状态来自 KWS 的 `:8091` 电平服务（与 KWS 同进程）；KWS 未跑时显示关麦
- IMU 需 sim2real 里 yesense 节点发布 `/imu/data`
- 改完代码需重启 `server.py` 并刷新浏览器
