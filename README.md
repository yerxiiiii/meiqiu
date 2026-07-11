# Moon

UWB + 视觉感知 → 决策（brain）→ 运控（sim2real）。

## 先读

- 决策与跟随注意：[`brain/README.md`](brain/README.md)（含 **使用注意**）
- 运控策略 `amp_right_hold`：[`docs/AMP_RIGHT_HOLD.md`](docs/AMP_RIGHT_HOLD.md)（配置见 `sim2real/config/`）

## 真机最短路径

1. 单实例：`roslaunch sim2real_master joy_control_pi_plus_orin.launch`
2. 手柄 `LT+RT+Start` → STANDBY
3. `sudo systemctl stop uwb-follow.service`（勿另开 `uwb_follow.py`）
4. `python3 /home/nvidia/moon/brain/mode_arbiter.py`
5. 口令「小派我们走」→ 自动切 `amp_right_hold` 并跟随
