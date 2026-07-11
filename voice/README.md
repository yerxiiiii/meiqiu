# Moon Voice — 只发口令，不控电机

话题：`/moon/voice_cmd`（`std_msgs/String`）  
载荷：`face_look` | `uwb_follow` | `stop`

口令映射见 [`keywords.yaml`](keywords.yaml)。  
Sherpa 拼音词表：[`sherpa_keywords.txt`](sherpa_keywords.txt)（由 `sherpa_keywords_raw.txt` 生成）。

## 依赖（本机已装）

```bash
pip3 install --user sherpa-onnx sounddevice pypinyin sentencepiece
```

KWS 模型目录：

`models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01/`  
（WenetSpeech 中文 zipformer，约 3.3M）

## 模拟（不启麦）

```bash
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash
python3 /home/nvidia/moon/voice/voice_sim.py
# 1=看我  2=我们走  0=停止
python3 /home/nvidia/moon/voice/kws_node.py --text "小派我们走"
```

## Sherpa 离线 KWS（接麦）

```bash
source /home/nvidia/sim2real_master-feature-master_and_slave/install/setup.bash

# 决策侧需常驻（语音只发话题，不启 uwb_follow.py）
# python3 /home/nvidia/moon/brain/mode_arbiter.py

python3 /home/nvidia/moon/voice/kws_node.py
# 等价于自动找默认模型 + sherpa_keywords.txt
```

说「小派我们走」→ `/moon/voice_cmd` = `uwb_follow` → `mode_arbiter` 进 `UWB_FOLLOW`。

## 麦克风电平窗口

KWS 启动后自动开监视页（与识别同一路音频）：

浏览器打开：`http://<机器人IP>:8091/`

可看实时 **RMS / peak**、设备名、最近关键词命中。条接近 0 = 没收音。

## 上电默认开麦（systemd）

当前已用 **用户级 systemd**（`~/.config/systemd/user/`）enable：

- `moon-kws`：上电/登录后开录音设备 + 离线 KWS  
- `moon-arbiter`：收口令并决策  

```bash
systemctl --user status moon-kws moon-arbiter
tail -f /home/nvidia/moon/logs/moon_kws_boot.log
```

**无人值守上电也要听麦**（不登录图形界面）还需 linger（一次，要 sudo）：

```bash
sudo loginctl enable-linger nvidia
```

或装成系统服务：

```bash
sudo cp /home/nvidia/moon/voice/moon-kws.service /etc/systemd/system/
sudo cp /home/nvidia/moon/brain/moon-arbiter.service /etc/systemd/system/
# 系统单元里 WantedBy 请保持 multi-user.target（仓库内 *.service 已是）
sudo systemctl daemon-reload
sudo systemctl enable --now moon-kws.service moon-arbiter.service
```

不要同时 enable 独立 `uwb-follow.service`（抢串口 / `cmd_vel`）。

指定麦设备：

```bash
python3 /home/nvidia/moon/voice/kws_node.py --device pulse
# 或 --device 4
```

## 改口令后重生成拼音词表

```bash
sherpa-onnx-cli text2token \
  --tokens models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01/tokens.txt \
  --tokens-type ppinyin \
  sherpa_keywords_raw.txt sherpa_keywords.txt
```
