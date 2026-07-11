# Moon

UWB + 视觉感知 → 决策（brain）→ 运控（sim2real）。

## 先读

- 决策与跟随注意：[`brain/README.md`](brain/README.md)（含 **使用注意**）
- 运控策略 `amp_right_hold`：[`docs/AMP_RIGHT_HOLD.md`](docs/AMP_RIGHT_HOLD.md)（配置见 `sim2real/config/`）

## 换机必做（别人 clone 后）

1. **固定运控路径**（二选一）：
   ```bash
   # 推荐：软链到短名
   ln -sfn /path/to/your/sim2real_ws ~/sim2real
   # 或显式指定
   export SIM2REAL_WS=/path/to/your/sim2real_ws
   ```
2. **安装 amp_right_hold 配置到运控**（只拷 yaml，不需整仓）：
   ```bash
   cd ~/moon   # 或你的 clone 路径
   ./scripts/install_amp_right_hold.sh
   ```
3. 重启 `sim2real_master`，日志应出现 `amp_right_hold` / `all-amp-rhold`。

路径解析逻辑见 `common/sim2real_env.py` / `scripts/sim2real_env.sh`（优先 `$SIM2REAL_WS`，再 `~/sim2real`）。

## 真机最短路径

1. 单实例：`roslaunch sim2real_master joy_control_pi_plus_orin.launch`
2. 手柄 `LT+RT+Start` → STANDBY
3. `sudo systemctl stop uwb-follow.service`（勿另开 `uwb_follow.py`）
4. `python3 /home/nvidia/moon/brain/mode_arbiter.py`
5. 口令「小派我们走」→ 自动切 `amp_right_hold` 并跟随
